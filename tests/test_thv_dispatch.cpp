#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <memory>
#include <optional>
#include <vector>
#include "ThvDispatchStrategy.h"
#include "Elevator.h"
#include "Simulator.h"
#include "ISimulationLogger.h"
#include "RandomProvider.h"
#include "StandardMetricsCollector.h"
#include "Task.h"	// UP, DOWN, GO, CALL, DEFAULT

using namespace openEGCS;
using namespace openEGCS::examples;

// The THV baseline of Cortes et al. (2004), Appendix A - the duplex algorithm
// GAHCA is compared against in the paper. Hand-computed fitnesses straight
// from the pseudocode (N = 10 floors), plus the selection and a full-run
// sanity.

namespace {

// A BUSY cabin at `floor` heading `GO`-wards to `target` (above => UP, below =>
// DOWN), so getPosition()/getStatus()/getDirection() read as the case needs.
Elevator busyCar(unsigned long int id, double floor, int target) {
	Elevator e(id, "", floor, 1);
	Task go(0, "", GO, DEFAULT, target, 0.0);
	e.addTask(go, 1);
	return e;
}

struct NoopLogger : public ISimulationLogger {
	void setMetadata(const SimulationMetadata&) override {}
	void logCustomer(const Customer&) override {}
	void logEvent(double, const std::string&, int, unsigned long int, int) override {}
	void finalize(const IMetricsCollector&, const std::string&) override {}
};

} // namespace

TEST_CASE("THV fitness matches Appendix A in each of the four cases", "[thv]") {
	ThvDispatchStrategy thv(/*floors=*/10);

	// Stopped (idle) car at floor 2, call UP at 5: N - d = 10 - 3 = 7.
	Elevator idleAt2(1, "", 2, 1);
	REQUIRE(thv.fitnessOf(idleAt2, /*callFloor=*/5, UP) == Catch::Approx(7.0));

	// Homing SAME direction: BUSY going up at 4, call UP at 5: N+1-d = 10+1-1 = 10.
	Elevator upAt4 = busyCar(2, 4, /*target=*/9);
	REQUIRE(thv.fitnessOf(upAt4, 5, UP) == Catch::Approx(10.0));

	// Homing DIFFERENT direction: same car (going up, approaching 5), call DOWN
	// at 5: N - d = 10 - 1 = 9.
	REQUIRE(thv.fitnessOf(upAt4, 5, DOWN) == Catch::Approx(9.0));

	// Has left the floor: BUSY going up at 7, call UP at 5 (behind it): fitness 1.
	Elevator upAt7 = busyCar(3, 7, /*target=*/9);
	REQUIRE(thv.fitnessOf(upAt7, 5, UP) == Catch::Approx(1.0));

	// The descending mirror, homing same direction: going down at 8, call DOWN at
	// 5: N+1-d = 10+1-3 = 8.
	Elevator downAt8 = busyCar(4, 8, /*target=*/2);
	REQUIRE(thv.fitnessOf(downAt8, 5, DOWN) == Catch::Approx(8.0));
}

TEST_CASE("THV picks the highest-fitness car, ties to the lowest id", "[thv]") {
	ThvDispatchStrategy thv(10);

	// Car 1 idle at 2 (fitness 7), car 2 already going up toward 5 (fitness 10):
	// the one already heading the right way wins.
	std::vector<Elevator> cars;
	cars.push_back(Elevator(1, "", 2, 1));
	cars.push_back(busyCar(2, 4, 9));
	Task call(0, "", CALL, UP, 5, 0.0);
	REQUIRE(thv.selectElevator(call, cars) == 2);
	// THV takes no dry-run, so the real cars are untouched.
	REQUIRE(cars[1].getStatus() == BUSY);

	// Two idle cars equidistant from the call: the tie goes to the lower id.
	std::vector<Elevator> tied;
	tied.push_back(Elevator(1, "", 3, 1));
	tied.push_back(Elevator(2, "", 7, 1));
	Task mid(0, "", CALL, UP, 5, 0.0);
	REQUIRE(thv.selectElevator(mid, tied) == 1);
}

TEST_CASE("THV drives a full run to completion", "[thv]") {
	const int floors = 15, elevators = 3, customers = 120;
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(42));
	auto thv = std::make_shared<ThvDispatchStrategy>(floors);
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto logger = std::make_shared<NoopLogger>();
	Simulator sim(1, "thv", floors, elevators, customers, /*rate=*/1.5, 1.0f, true,
		rng, thv, metrics, logger);
	sim.startSimulation();

	// Every customer boarded once and alighted once - a baseline that stranded
	// somebody would fail here.
	REQUIRE(metrics->getWaitingTimeDistribution().count == customers);
	REQUIRE(metrics->getJourneyTimeDistribution().count == customers);
}
