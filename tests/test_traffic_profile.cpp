#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <deque>
#include <limits>
#include <cstdint>
#include "ITrafficProfile.h"
#include "StationaryUniformTrafficProfile.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "StandardMetricsCollector.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// The default ITrafficProfile (stationary Poisson + uniform floors). Its RNG
// draw order is a contract - it is what makes a seed reproduce a run - so
// these tests pin the order down two ways: against a recording of which
// methods were called, and against the exact values the draws must produce.

namespace {

// Records the order of IRandomProvider calls while delegating to a real
// seeded provider, so a test can assert *which* draws a profile made and in
// what order (the thing byte-identity actually depends on).
class RecordingRandomProvider : public IRandomProvider {
	public:
		explicit RecordingRandomProvider(uint64_t seed) : inner(std::optional<uint64_t>(seed)) {}

		double nextExponential(double rate) override {
			calls.push_back("exp");
			return inner.nextExponential(rate);
		}
		int nextUniformInt(int lo, int hi) override {
			calls.push_back("uni");
			return inner.nextUniformInt(lo, hi);
		}
		double nextNormal(double mean, double stddev) override {
			calls.push_back("normal");
			return inner.nextNormal(mean, stddev);
		}

		std::vector<std::string> calls;
	private:
		RandomProvider inner;
};

// Returns a scripted sequence of ints for nextUniformInt, so a test can force
// the destination==origin rejection loop deterministically (a real RNG rarely
// collides on demand). Exponential/Normal are not used here.
class ScriptedRandomProvider : public IRandomProvider {
	public:
		explicit ScriptedRandomProvider(std::deque<int> ints) : scripted(std::move(ints)) {}

		double nextExponential(double) override { return 0.0; }
		int nextUniformInt(int, int) override {
			int v = scripted.front();
			scripted.pop_front();
			++uniformDraws;
			return v;
		}
		double nextNormal(double, double) override { return 0.0; }

		int uniformDraws = 0;
	private:
		std::deque<int> scripted;
};

} // namespace

TEST_CASE("Stationary default makes exactly one exponential draw for inter-arrival", "[traffic]") {
	StationaryUniformTrafficProfile profile(/*rate=*/0.8);
	RecordingRandomProvider rng(42);

	double delta = profile.nextInterArrivalTime(/*now=*/0.0, rng);

	REQUIRE(delta >= 0.0);
	REQUIRE(rng.calls == std::vector<std::string>{"exp"});
}

TEST_CASE("Stationary default samples a trip with two uniform draws and a distinct destination",
          "[traffic]") {
	StationaryUniformTrafficProfile profile;
	RecordingRandomProvider rng(42);

	int origin = -1, destination = -1;
	profile.sampleTrip(/*t=*/0.0, rng, /*maxFloors=*/9, origin, destination);

	REQUIRE(origin >= 0);
	REQUIRE(origin <= 9);
	REQUIRE(destination >= 0);
	REQUIRE(destination <= 9);
	REQUIRE(origin != destination);
	// At least two uniform draws (one origin, one destination), and nothing but
	// uniform draws - no exponential or normal leaks into the trip sample.
	REQUIRE(rng.calls.size() >= 2);
	for (const auto& c : rng.calls) {
		REQUIRE(c == "uni");
	}
}

TEST_CASE("Stationary default re-draws the destination until it differs from the origin",
          "[traffic]") {
	// origin=3, destination=3 (collision), re-draw destination=3 (again),
	// re-draw destination=5 (accepted): four uniform draws total.
	auto rng = ScriptedRandomProvider(std::deque<int>{3, 3, 3, 5});
	StationaryUniformTrafficProfile profile;

	int origin = -1, destination = -1;
	profile.sampleTrip(/*t=*/0.0, rng, /*maxFloors=*/9, origin, destination);

	REQUIRE(origin == 3);
	REQUIRE(destination == 5);
	REQUIRE(rng.uniformDraws == 4);
}

TEST_CASE("Stationary default reproduces the exact pre-interface draw sequence", "[traffic]") {
	// The inline code was: enq += nextExponential(rate); origin = nextUniformInt;
	// destination = nextUniformInt (repeat while ==origin). Feed two providers
	// seeded identically - one through the profile, one through the raw calls -
	// and require the same values in the same order. This is the byte-identity
	// contract reduced to the draws it rests on.
	const double rate = 0.8;
	const int maxFloors = 12;

	RandomProvider raw(std::optional<uint64_t>(777));
	double rawDelta = raw.nextExponential(rate);
	int rawOrigin = raw.nextUniformInt(0, maxFloors);
	int rawDest = raw.nextUniformInt(0, maxFloors);
	while (rawDest == rawOrigin) {
		rawDest = raw.nextUniformInt(0, maxFloors);
	}

	StationaryUniformTrafficProfile profile(rate);
	RandomProvider viaProfile(std::optional<uint64_t>(777));
	double profDelta = profile.nextInterArrivalTime(0.0, viaProfile);
	int profOrigin = -1, profDest = -1;
	profile.sampleTrip(0.0, viaProfile, maxFloors, profOrigin, profDest);

	REQUIRE(profDelta == rawDelta);
	REQUIRE(profOrigin == rawOrigin);
	REQUIRE(profDest == rawDest);
}

// End-to-end: a run with the default (null) profile and a run with an
// explicitly injected StationaryUniformTrafficProfile built on the same rate
// must agree exactly - proving the Simulator resolves null to the stationary
// default and that routing the draws through the interface changed nothing.
static double runAvgWait(uint64_t seed, std::shared_ptr<ITrafficProfile> profile) {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();

	Simulator sim(1, "test", /*floors=*/10, /*elevators=*/2, /*customers=*/40,
		/*arrivalRate=*/0.8, /*speed=*/1.0f, /*sane=*/true,
		rng, dispatch, metrics, /*logger=*/nullptr, /*seed_for_metadata=*/std::nullopt,
		/*max_weight=*/std::numeric_limits<double>::infinity(), /*capacity=*/nullptr,
		/*mean_weight=*/70.0, /*stddev_weight=*/15.0, /*dwell=*/nullptr, /*travel=*/nullptr,
		/*passenger=*/nullptr, /*group=*/nullptr, /*traffic_profile=*/profile);

	sim.startSimulation();
	return metrics->getAverageWaitingTime();
}

TEST_CASE("Injecting the stationary default equals the null-profile path", "[traffic][e2e]") {
	double viaNull = runAvgWait(555, nullptr);
	double viaExplicit = runAvgWait(555, std::make_shared<StationaryUniformTrafficProfile>(0.8));
	REQUIRE(viaNull == viaExplicit);
}
