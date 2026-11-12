#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <optional>
#include <vector>
#include <array>
#include <limits>
#include <cstdint>
#include "DownPeakTrafficProfile.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "StandardMetricsCollector.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;
using openEGCS::examples::DownPeakTrafficProfile;

// The down-peak example, a non-stationary profile: thinning for the time, and
// the Barney down-peak template (Table 17.4) for the shape. These tests
// confirm the *shape* of the traffic (everyone goes down to the terminal;
// demand concentrates in the peak period) and reproducibility by seed. They
// do NOT assert byte-identity with anything - a peaked profile changes the
// stream by design.

// Accumulates arrivals over one modelled hour (3600 s) and bins them into the
// twelve 5-minute periods of Table 17.4.
static std::array<int, 12> binArrivalsOverHour(DownPeakTrafficProfile& profile, uint64_t seed) {
	RandomProvider rng{std::optional<uint64_t>(seed)};
	std::array<int, 12> bins{};
	double t = 0.0;
	for (;;) {
		t += profile.nextInterArrivalTime(t, rng);
		if (t >= 3600.0) break;
		int period = static_cast<int>(t / 300.0);
		if (period >= 0 && period < 12) bins[period]++;
	}
	return bins;
}

TEST_CASE("Down-peak sends every passenger down to the main terminal", "[traffic][downpeak]") {
	DownPeakTrafficProfile profile;
	RandomProvider rng(std::optional<uint64_t>(7));

	for (int i = 0; i < 500; i++) {
		int origin = -1, destination = -1;
		profile.sampleTrip(/*t=*/0.0, rng, /*maxFloors=*/15, origin, destination);
		REQUIRE(destination == 0);			// the main terminal
		REQUIRE(origin >= 1);				// never the lobby itself
		REQUIRE(origin <= 15);
		REQUIRE(origin != destination);		// so the call direction is always DOWN
	}
}

TEST_CASE("Down-peak demand concentrates in the Barney peak period (period 3)", "[traffic][downpeak]") {
	DownPeakTrafficProfile profile;
	std::array<int, 12> bins = binArrivalsOverHour(profile, /*seed=*/42);

	int total = 0;
	int argmax = 0;
	for (int i = 0; i < 12; i++) {
		total += bins[i];
		if (bins[i] > bins[argmax]) argmax = i;
	}

	// The template peaks at period 3 (index 2, 180 persons/5-min vs 22 in
	// period 1). The busiest bin must be period 3, and it must dwarf the
	// quietest opening period - 180/22 is ~8x, so a 3x floor is comfortably
	// inside sampling noise.
	REQUIRE(argmax == 2);
	REQUIRE(bins[2] >= 3 * bins[0]);
	// One hour of the scale-1.0 curve totals 636 persons; sampling noise keeps a
	// single run well inside a broad band.
	REQUIRE(total > 400);
	REQUIRE(total < 900);
}

TEST_CASE("Down-peak is reproducible by seed and sensitive to it", "[traffic][downpeak]") {
	DownPeakTrafficProfile a, b, c;
	std::array<int, 12> binsA = binArrivalsOverHour(a, 123);
	std::array<int, 12> binsB = binArrivalsOverHour(b, 123);
	std::array<int, 12> binsC = binArrivalsOverHour(c, 456);

	REQUIRE(binsA == binsB);			// same seed -> identical
	REQUIRE(binsA != binsC);			// a different seed moves the arrivals
}

TEST_CASE("Scale densifies the down-peak while keeping its shape", "[traffic][downpeak]") {
	DownPeakTrafficProfile base(/*scale=*/1.0);
	DownPeakTrafficProfile dense(/*scale=*/2.0);
	std::array<int, 12> binsBase = binArrivalsOverHour(base, 99);
	std::array<int, 12> binsDense = binArrivalsOverHour(dense, 99);

	int totalBase = 0, totalDense = 0;
	for (int i = 0; i < 12; i++) { totalBase += binsBase[i]; totalDense += binsDense[i]; }

	REQUIRE(totalDense > totalBase);	// twice the rate -> more arrivals
	// The peak is still period 3 under either scale.
	int argmaxDense = 0;
	for (int i = 0; i < 12; i++) if (binsDense[i] > binsDense[argmaxDense]) argmaxDense = i;
	REQUIRE(argmaxDense == 2);
}

// End to end: a full simulation driven by the down-peak profile must still
// terminate (every created customer reaches their destination) and report
// finite metrics - the thinning loop does not hang and the fixed
// destination 0 is a valid trip for the engine.
TEST_CASE("A full run under the down-peak profile terminates with finite metrics", "[traffic][downpeak][e2e]") {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(2024));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto profile = std::make_shared<DownPeakTrafficProfile>();

	Simulator sim(1, "downpeak", /*floors=*/12, /*elevators=*/3, /*customers=*/120,
		/*arrivalRate=*/0.5, /*speed=*/1.0f, /*sane=*/true,
		rng, dispatch, metrics, /*logger=*/nullptr, /*seed_for_metadata=*/std::nullopt,
		std::numeric_limits<double>::infinity(), /*capacity=*/nullptr,
		/*mean_weight=*/70.0, /*stddev_weight=*/15.0, /*dwell=*/nullptr, /*travel=*/nullptr,
		/*passenger=*/nullptr, /*group=*/nullptr, /*traffic_profile=*/profile);

	sim.startSimulation();

	REQUIRE(metrics->getAverageWaitingTime() >= 0.0);
	REQUIRE(metrics->getAverageTravelTime() > 0.0);
}
