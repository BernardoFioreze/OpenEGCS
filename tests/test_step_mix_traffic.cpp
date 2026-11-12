#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include "BeamurgiaDispatchStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "StandardMetricsCollector.h"
#include "StepMixTrafficProfile.h"

using namespace openEGCS;
using namespace openEGCS::examples;

// The traffic of Beamurgia et al. 2016 (section 6.2, Table 1). The tests
// measure what the profile was configured to produce - the rate per 5-minute
// period and the share of each kind of call - in the pattern of the other
// profile tests.

namespace {

// C1 of Table 1: 12 floors (0..11, the main terminal and 11 populated), 80
// people per populated floor -> a population of 880.
StepMixTrafficProfile::Params c1(double incoming, double outgoing, double interfloor)
{
	StepMixTrafficProfile::Params p;
	p.populatedFloors = 11;
	p.populationPerFloor = 80.0;
	p.incoming = incoming;
	p.outgoing = outgoing;
	p.interfloor = interfloor;
	return p;
}

} // namespace

TEST_CASE("Step-mix: the demand steps from 11% to 13% per 5 minutes and holds", "[traffic][beamurgia]") {
	StepMixTrafficProfile profile(c1(0.0, 0.0, 1.0));
	REQUIRE(profile.percentAt(0.0) == Catch::Approx(11.0));
	REQUIRE(profile.percentAt(299.9) == Catch::Approx(11.0));
	REQUIRE(profile.percentAt(300.0) == Catch::Approx(12.0));
	REQUIRE(profile.percentAt(600.0) == Catch::Approx(13.0));
	REQUIRE(profile.percentAt(3000.0) == Catch::Approx(13.0));
}

TEST_CASE("Step-mix: the arrivals per period match the configured percentage of the population", "[traffic][beamurgia]") {
	// 880 people: 11%, 12%, 13% of them per 5 minutes is 96.8, 105.6 and 114.4
	// arrivals, then 114.4 held. Averaged over 40 seeds a period's count has a
	// standard error of about 1.7, so 5% is several of them.
	StepMixTrafficProfile profile(c1(0.0, 0.0, 1.0));
	const int periods = 6, seeds = 40;
	std::array<double, periods> mean{};
	for (uint64_t seed = 1; seed <= seeds; ++seed) {
		RandomProvider rng{std::optional<uint64_t>(seed)};
		double t = 0.0;
		for (;;) {
			t += profile.nextInterArrivalTime(t, rng);
			if (t >= periods * 300.0) break;
			mean[static_cast<int>(t / 300.0)] += 1.0 / seeds;
		}
	}
	const std::array<double, periods> expected = {96.8, 105.6, 114.4, 114.4, 114.4, 114.4};
	for (int k = 0; k < periods; ++k) {
		REQUIRE(mean[k] == Catch::Approx(expected[k]).epsilon(0.05));
	}
}

TEST_CASE("Step-mix: each kind of call goes where its definition says", "[traffic][beamurgia]") {
	RandomProvider rng{std::optional<uint64_t>(11)};
	int origin = -1, destination = -1;

	StepMixTrafficProfile incoming(c1(1.0, 0.0, 0.0));
	StepMixTrafficProfile outgoing(c1(0.0, 1.0, 0.0));
	StepMixTrafficProfile interfloor(c1(0.0, 0.0, 1.0));
	std::array<int, 12> interfloorDestinations{};
	for (int i = 0; i < 2000; ++i) {
		incoming.sampleTrip(0.0, rng, 11, origin, destination);
		REQUIRE(origin == 0);
		REQUIRE(destination >= 1);
		REQUIRE(destination <= 11);

		outgoing.sampleTrip(0.0, rng, 11, origin, destination);
		REQUIRE(destination == 0);
		REQUIRE(origin >= 1);
		REQUIRE(origin <= 11);

		interfloor.sampleTrip(0.0, rng, 11, origin, destination);
		REQUIRE(origin >= 1);
		REQUIRE(destination >= 1);
		REQUIRE(origin <= 11);
		REQUIRE(destination <= 11);
		REQUIRE(origin != destination);
		++interfloorDestinations[destination];
	}
	// Uniform over the eleven populated floors: each about 2000/11 = 182, the
	// main terminal never.
	REQUIRE(interfloorDestinations[0] == 0);
	for (int f = 1; f <= 11; ++f) REQUIRE(interfloorDestinations[f] > 120);
}

TEST_CASE("Step-mix: STEP1's 45/45/10 comes out in those shares", "[traffic][beamurgia]") {
	// Section 6.5's STEP1. With 30,000 draws a share's standard error is under
	// 0.3 points, so 1.5 points is five of them.
	StepMixTrafficProfile profile(c1(0.45, 0.45, 0.10));
	RandomProvider rng{std::optional<uint64_t>(45)};
	int in = 0, out = 0, inter = 0;
	const int n = 30000;
	for (int i = 0; i < n; ++i) {
		int origin = -1, destination = -1;
		profile.sampleTrip(0.0, rng, 11, origin, destination);
		if (origin == 0) ++in;
		else if (destination == 0) ++out;
		else ++inter;
	}
	REQUIRE(in / double(n) == Catch::Approx(0.45).margin(0.015));
	REQUIRE(out / double(n) == Catch::Approx(0.45).margin(0.015));
	REQUIRE(inter / double(n) == Catch::Approx(0.10).margin(0.015));
}

TEST_CASE("Step-mix: refuses a configuration it cannot honour", "[traffic][beamurgia]") {
	REQUIRE_THROWS_AS(StepMixTrafficProfile(c1(0.5, 0.4, 0.0)), std::invalid_argument);		// shares add to 0.9
	REQUIRE_THROWS_AS(StepMixTrafficProfile(c1(-0.1, 0.6, 0.5)), std::invalid_argument);	// negative share

	StepMixTrafficProfile::Params noPeople = c1(0.0, 0.0, 1.0);
	noPeople.populationPerFloor = 0.0;
	REQUIRE_THROWS_AS(StepMixTrafficProfile(noPeople), std::invalid_argument);

	StepMixTrafficProfile::Params falling = c1(0.0, 0.0, 1.0);
	falling.maxPercent = 9.0;
	REQUIRE_THROWS_AS(StepMixTrafficProfile(falling), std::invalid_argument);

	StepMixTrafficProfile::Params oneFloor = c1(0.0, 0.0, 1.0);
	oneFloor.populatedFloors = 1;
	REQUIRE_THROWS_AS(StepMixTrafficProfile(oneFloor), std::invalid_argument);

	// The profile was built for 11 populated floors; a 15-floor run is refused.
	StepMixTrafficProfile profile(c1(0.0, 0.0, 1.0));
	RandomProvider rng{std::optional<uint64_t>(1)};
	int origin = -1, destination = -1;
	REQUIRE_THROWS_AS(profile.sampleTrip(0.0, rng, 15, origin, destination), std::invalid_argument);

	REQUIRE(profile.name() == "step-mix (Beamurgia 2016)");
}

TEST_CASE("Step-mix: a C1 outgoing run under the Beamurgia GA terminates", "[traffic][beamurgia][e2e]") {
	// The bench's scenario shape, small: C1 (6 cars, floors 0..11, 1600 kg),
	// pure outgoing, every adjustment, R1'. Every created customer has to reach
	// their destination for the run to end at all.
	const int floors = 11;
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(2016));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	BeamurgiaDispatchStrategy::Params p;
	p.seeding = true;
	p.lastBestIndividual = true;
	p.penalty = BeamurgiaDispatchStrategy::Penalty::Dynamic;
	p.estimator.reversal = BeamurgiaReversal::ExpectedExtremeFloor;
	p.estimator.lowestFloor = 0;
	p.estimator.highestFloor = floors;
	auto beamurgia = std::make_shared<BeamurgiaDispatchStrategy>(p);
	auto profile = std::make_shared<StepMixTrafficProfile>(c1(0.0, 1.0, 0.0));
	Simulator sim(1, "step-mix-c1", floors, /*elevators=*/6, /*customers=*/80,
		/*arrivalRate=*/0.5, /*speed=*/1.0f, /*sane=*/true,
		rng, dispatch, metrics, nullptr, std::nullopt,
		/*maxWeight=*/1600.0, nullptr, 70.0, 15.0,
		nullptr, nullptr, nullptr, beamurgia, profile);
	sim.startSimulation();
	REQUIRE(std::isfinite(metrics->getAverageWaitingTime()));
	REQUIRE(metrics->getAverageWaitingTime() > 0.0);
	REQUIRE(metrics->getWaitingTimeDistribution().count == 80);
	REQUIRE(beamurgia->optimizations() > 0);
}
