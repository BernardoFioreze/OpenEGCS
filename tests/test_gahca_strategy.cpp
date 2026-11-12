#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include "GahcaDispatchStrategy.h"
#include "IGroupDispatchStrategy.h"
#include "Elevator.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "ISimulationLogger.h"
#include "GreedyNearestDispatchStrategy.h"
#include "Task.h"	// UP, DOWN

using namespace openEGCS;
using namespace openEGCS::examples;

// The GAHCA genetic algorithm as a pure component - allocate(calls,
// elevators, rng) -> one car id per call. These drive it on hand-built
// inputs, no engine, and check three properties: determinism,
// non-worsening, and that a committed call never moves.

namespace {

std::vector<Elevator> twoIdleCarsAtLobby() {
	std::vector<Elevator> cars;
	cars.push_back(Elevator(1, "", 0, 1));
	cars.push_back(Elevator(2, "", 0, 1));
	return cars;
}

// {floor, direction, registeredAt, currentElevatorId, committed}
HallCall call(int floor, unsigned int dir, int car, bool committed) {
	return HallCall{floor, dir, 0.0, car, committed};
}

// Concatenates every logged event into one string, so two full runs can be
// compared for byte-level identity of their timelines (used for determinism).
struct SignatureLogger : public ISimulationLogger {
	std::ostringstream sig;
	int events = 0;
	void setMetadata(const SimulationMetadata&) override {}
	void logCustomer(const Customer&) override {}
	void logEvent(double time, const std::string& type, int elevatorId,
		unsigned long int customerId, int floor) override {
		sig << time << '|' << type << '|' << elevatorId << '|' << customerId << '|' << floor << '\n';
		++events;
	}
	void finalize(const IMetricsCollector&, const std::string&) override {}
};

std::string gahcaRunSignature(uint64_t seed, int& eventCount) {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed));
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto logger = std::make_shared<SignatureLogger>();
	auto gahca = std::make_shared<GahcaDispatchStrategy>();
	Simulator sim(1, "gahca-determinism", /*floors=*/15, /*elevators=*/3, /*customers=*/120,
		/*rate=*/1.5, 1.0f, true, rng, dispatch, nullptr, logger, std::nullopt,
		std::numeric_limits<double>::infinity(), nullptr, 70.0, 15.0,
		nullptr, nullptr, nullptr, gahca);
	sim.startSimulation();
	eventCount = logger->events;
	return logger->sig.str();
}

// The current/seed allocation the GA starts from: each call's current car, or
// car 1 for a pooled call.
std::vector<int> seedOf(const std::vector<HallCall>& calls) {
	std::vector<int> s;
	for (const HallCall& c : calls) s.push_back(c.currentElevatorId >= 1 ? c.currentElevatorId : 1);
	return s;
}

} // namespace

TEST_CASE("GAHCA is deterministic for a given seed and state", "[gahca]") {
	std::vector<HallCall> calls = { call(5, UP, 0, false), call(8, UP, 0, false), call(3, DOWN, 0, false) };
	std::vector<Elevator> cars = twoIdleCarsAtLobby();
	GahcaDispatchStrategy gahca;

	RandomProvider rngA(std::optional<uint64_t>(42));
	RandomProvider rngB(std::optional<uint64_t>(42));
	std::vector<int> a = gahca.allocate(calls, cars, rngA);
	std::vector<int> b = gahca.allocate(calls, cars, rngB);

	REQUIRE(a == b);				// same seed, same state -> same allocation
	REQUIRE(a.size() == calls.size());
	for (int car : a) { REQUIRE(car >= 1); REQUIRE(car <= 2); }	// every gene a valid car
}

TEST_CASE("GAHCA never returns a worse allocation than the current one", "[gahca]") {
	// Both up-calls start piled on cabin 1 (currentElevatorId = 1), which is the
	// suboptimal seed; the GA should match or beat it (splitting the work lowers
	// the balancing term). Elitism + seeding individual 0 with the current state
	// guarantees non-worsening.
	std::vector<HallCall> calls = { call(5, UP, 1, false), call(8, UP, 1, false) };
	std::vector<Elevator> cars = twoIdleCarsAtLobby();
	GahcaDispatchStrategy gahca;

	RandomProvider rng(std::optional<uint64_t>(7));
	std::vector<int> result = gahca.allocate(calls, cars, rng);

	double seedObjective = gahca.objective(seedOf(calls), calls, cars);
	double resultObjective = gahca.objective(result, calls, cars);
	REQUIRE(resultObjective <= seedObjective + 1e-9);
}

TEST_CASE("GAHCA splits work it is worth splitting", "[gahca]") {
	// Two up-calls far apart, two idle cabins: the lower-objective allocation
	// puts one on each cabin. This exercises the k2 (Hct - Lct) balancing term
	// that is GAHCA's whole difference from the greedy baseline.
	std::vector<HallCall> calls = { call(4, UP, 1, false), call(9, UP, 1, false) };
	std::vector<Elevator> cars = twoIdleCarsAtLobby();
	GahcaDispatchStrategy gahca;

	RandomProvider rng(std::optional<uint64_t>(123));
	std::vector<int> result = gahca.allocate(calls, cars, rng);

	// Objective of the split vs everything on one cabin: the GA must not do
	// worse than the split optimum.
	double splitObjective = gahca.objective({1, 2}, calls, cars);
	double resultObjective = gahca.objective(result, calls, cars);
	REQUIRE(resultObjective <= splitObjective + 1e-9);
	REQUIRE(result[0] != result[1]);	// one call per cabin
}

TEST_CASE("GAHCA never moves a committed call", "[gahca]") {
	// The middle call is committed to cabin 2; whatever the GA does with the
	// others, that one stays on cabin 2.
	std::vector<HallCall> calls = {
		call(5, UP, 1, false),
		call(7, DOWN, 2, true),		// committed to cabin 2
		call(3, UP, 0, false),
	};
	std::vector<Elevator> cars = twoIdleCarsAtLobby();
	GahcaDispatchStrategy gahca;

	RandomProvider rng(std::optional<uint64_t>(99));
	std::vector<int> result = gahca.allocate(calls, cars, rng);

	REQUIRE(result[1] == 2);	// the committed gene is fixed
}

TEST_CASE("GAHCA drives a byte-deterministic full run", "[gahca]") {
	// The metaheuristic is only reproducible if every draw goes through the one
	// seeded IRandomProvider and the re-optimise-every-event cadence is itself
	// deterministic (plan §5 risk 3). Two full runs with the same seed must
	// produce byte-identical event timelines.
	int eventsA = 0, eventsB = 0;
	std::string a = gahcaRunSignature(/*seed=*/2024, eventsA);
	std::string b = gahcaRunSignature(/*seed=*/2024, eventsB);

	REQUIRE(eventsA > 0);		// the run actually did something
	REQUIRE(eventsA == eventsB);
	REQUIRE(a == b);			// identical timelines, every event
}
