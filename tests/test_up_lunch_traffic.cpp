#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <optional>
#include <array>
#include <limits>
#include <cstdint>
#include <fstream>
#include <nlohmann/json.hpp>
#include "UpPeakTrafficProfile.h"
#include "LunchPeakTrafficProfile.h"
#include "StationaryUniformTrafficProfile.h"
#include "DownPeakTrafficProfile.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "StandardMetricsCollector.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;
using json = nlohmann::json;
using openEGCS::examples::UpPeakTrafficProfile;
using openEGCS::examples::LunchPeakTrafficProfile;

// Up-peak (Barney Table 17.2), lunch-peak (Cortes 2004 section 4.1), and the
// metadata.trafficProfile key. Shape and reproducibility, never byte-identity.

// ---------- up-peak ----------

TEST_CASE("Up-peak brings everyone up from the main terminal", "[traffic][uppeak]") {
	UpPeakTrafficProfile profile;
	RandomProvider rng{std::optional<uint64_t>(7)};
	for (int i = 0; i < 500; i++) {
		int origin = -1, destination = -1;
		profile.sampleTrip(0.0, rng, /*maxFloors=*/15, origin, destination);
		REQUIRE(origin == 0);				// the main terminal
		REQUIRE(destination >= 1);
		REQUIRE(destination <= 15);
		REQUIRE(destination != origin);		// always an UP call
	}
}

TEST_CASE("Up-peak demand concentrates in the Barney peak window (periods 6-8)", "[traffic][uppeak]") {
	UpPeakTrafficProfile profile;
	RandomProvider rng{std::optional<uint64_t>(42)};
	std::array<int, 12> bins{};
	double t = 0.0;
	for (;;) {
		t += profile.nextInterArrivalTime(t, rng);
		if (t >= 3600.0) break;
		int p = static_cast<int>(t / 300.0);
		if (p >= 0 && p < 12) bins[p]++;
	}
	int argmax = 0;
	for (int i = 0; i < 12; i++) if (bins[i] > bins[argmax]) argmax = i;
	// Table 17.2 peaks at period 8 (120), rising from period 1 (29). Periods 7
	// and 8 are close (105 vs 120), so sampling noise can swap the single
	// busiest bin between them - the robust claim is that the busiest bin sits
	// in the late-morning peak window and dwarfs the opening period.
	REQUIRE(argmax >= 5);					// period 6 or later (index 5)
	REQUIRE(argmax <= 7);					// period 8 or earlier (index 7)
	REQUIRE(bins[argmax] >= 2 * bins[0]);
}

TEST_CASE("Up-peak is reproducible by seed", "[traffic][uppeak]") {
	auto binHour = [](uint64_t seed) {
		UpPeakTrafficProfile p;
		RandomProvider rng{std::optional<uint64_t>(seed)};
		std::array<int, 12> b{};
		double t = 0.0;
		for (;;) { t += p.nextInterArrivalTime(t, rng); if (t >= 3600.0) break;
			int i = static_cast<int>(t / 300.0); if (i >= 0 && i < 12) b[i]++; }
		return b;
	};
	REQUIRE(binHour(123) == binHour(123));
	REQUIRE(binHour(123) != binHour(456));
}

// ---------- lunch-peak ----------

// Counts origin/destination facts over many trips sampled at a fixed instant.
struct ODtally { int fromGround = 0; int toGround = 0; int toAdmin = 0; int toTop = 0; };
static ODtally tallyAt(LunchPeakTrafficProfile& p, double t, int n, uint64_t seed) {
	RandomProvider rng{std::optional<uint64_t>(seed)};
	ODtally r;
	for (int i = 0; i < n; i++) {
		int o = -1, d = -1;
		p.sampleTrip(t, rng, /*maxFloors=*/11, o, d);
		if (o == 0) r.fromGround++;
		if (d == 0) r.toGround++;
		if (d == 6) r.toAdmin++;		// the 7th floor
		if (d == 11) r.toTop++;			// the 12th floor
	}
	return r;
}

TEST_CASE("Lunch-peak is bidirectional: exit goes down, return comes from the ground", "[traffic][lunchpeak]") {
	LunchPeakTrafficProfile profile(/*floors=*/12);

	// Exit peak at ~minute 30 (1800 s): the upper-floor stream dominates, almost
	// everyone riding down to the ground.
	ODtally exit = tallyAt(profile, /*t=*/1800.0, 2000, /*seed=*/7);
	REQUIRE(exit.fromGround < 100);			// essentially nobody starts at ground yet
	REQUIRE(exit.toGround > 1600);			// the vast majority go down to the terminal

	// Return peak at ~minute 95 (5700 s): the ground stream is now present and
	// dominant, people riding up into the building.
	ODtally ret = tallyAt(profile, /*t=*/5700.0, 2000, /*seed=*/7);
	REQUIRE(ret.fromGround > ret.toGround);	// the flow has flipped
	REQUIRE(ret.fromGround > 1000);
}

TEST_CASE("Lunch-peak ground OD favours the admin floor over the top (15% vs 4%)", "[traffic][lunchpeak]") {
	LunchPeakTrafficProfile profile(/*floors=*/12);
	ODtally ret = tallyAt(profile, /*t=*/5700.0, 2000, /*seed=*/7);
	// Of the ground-origin trips, destination 7th (15%) must clearly outnumber
	// destination 12th (4%) - the Cortes section 4.1 attraction.
	REQUIRE(ret.toAdmin > ret.toTop);
	REQUIRE(ret.toAdmin > 0);
	REQUIRE(ret.toTop > 0);
}

TEST_CASE("Lunch-peak is reproducible by seed", "[traffic][lunchpeak]") {
	auto binRun = [](uint64_t seed) {
		LunchPeakTrafficProfile p(12);
		RandomProvider rng{std::optional<uint64_t>(seed)};
		std::array<int, 26> b{};
		double t = 0.0;
		for (;;) { t += p.nextInterArrivalTime(t, rng); if (t >= 7800.0) break;
			int i = static_cast<int>(t / 300.0); if (i >= 0 && i < 26) b[i]++; }
		return b;
	};
	REQUIRE(binRun(321) == binRun(321));
	REQUIRE(binRun(321) != binRun(654));
}

TEST_CASE("A full run under each peak profile terminates with finite metrics", "[traffic][e2e]") {
	auto runWith = [](std::shared_ptr<ITrafficProfile> profile, int floors) {
		auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(2024));
		auto metrics = std::make_shared<StandardMetricsCollector>();
		auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
		Simulator sim(1, "peak", floors, /*elevators=*/3, /*customers=*/120,
			/*arrivalRate=*/0.5, /*speed=*/1.0f, /*sane=*/true,
			rng, dispatch, metrics, nullptr, std::nullopt,
			std::numeric_limits<double>::infinity(), nullptr, 70.0, 15.0,
			nullptr, nullptr, nullptr, nullptr, profile);
		sim.startSimulation();
		REQUIRE(metrics->getAverageWaitingTime() >= 0.0);
		REQUIRE(metrics->getAverageTravelTime() > 0.0);
	};
	runWith(std::make_shared<UpPeakTrafficProfile>(), 12);
	runWith(std::make_shared<LunchPeakTrafficProfile>(12), 12);
}

// ---------- metadata (schemaVersion 9) ----------

TEST_CASE("Each profile reports its name", "[traffic][metadata]") {
	REQUIRE(StationaryUniformTrafficProfile().name() == "stationary-uniform");
	REQUIRE(examples::DownPeakTrafficProfile().name() == "down-peak (Barney)");
	REQUIRE(UpPeakTrafficProfile().name() == "up-peak (Barney)");
	REQUIRE(LunchPeakTrafficProfile().name() == "lunch-peak (Cortes 2004)");
}

TEST_CASE("The snapshot records the traffic profile at schemaVersion 9", "[traffic][metadata][e2e]") {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(99));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto profile = std::make_shared<examples::DownPeakTrafficProfile>();
	Simulator sim(1, "meta", /*floors=*/10, /*elevators=*/2, /*customers=*/20,
		/*arrivalRate=*/0.5, /*speed=*/1.0f, /*sane=*/true,
		rng, dispatch, metrics, nullptr, std::nullopt,
		std::numeric_limits<double>::infinity(), nullptr, 70.0, 15.0,
		nullptr, nullptr, nullptr, nullptr, profile);
	sim.startSimulation();
	const std::string path = "test_traffic_meta.json";
	sim.writeSnapshot(path);

	std::ifstream in(path);
	REQUIRE(in.is_open());
	json root;
	in >> root;
	REQUIRE(root["schemaVersion"] == 11);
	REQUIRE(root["metadata"]["trafficProfile"] == "down-peak (Barney)");
}
