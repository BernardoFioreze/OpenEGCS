#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <vector>
#include "ElevatorSystem.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;
using namespace openEGCS::examples;

// IDispatchStrategy::selectElevator must leave `elevators` unmodified
// (what-if insertions undone). A strategy that only undoes its dry-run on
// some elevators leaves ghost tasks in the others, and checking
// `status == IDLE` does not catch it - an elevator holding a ghost task still
// satisfies that. Stop ids are the observable here: a leaked task shows up as
// a stop in the route.

namespace {

// True if the elevator's route holds no stop at all (ids are handed out
// sequentially from 1, so scanning a generous range is enough).
bool routeIsEmpty(const Elevator& e) {
	for (unsigned long int id = 1; id <= 50; id++) {
		if (e.getStopArrivalTime(id) >= 0) return false;
	}
	return true;
}

// Commits a task into elevator 1 and picks it - what a strategy that fails to
// undo its what-if insertion looks like to the engine.
struct LeakyDispatch : IDispatchStrategy {
	int selectElevator(Task task, std::vector<Elevator>& elevators) override {
		elevators[0].addTask(task, 1);
		return 1;
	}
};

} // namespace

TEST_CASE("GreedyNearestDispatchStrategy leaves no ghost tasks when the last elevator merges the call", "[dispatch][regression]") {
	std::vector<Elevator> elevators;
	elevators.push_back(Elevator(1, "", 0, 1)); // idle, empty route
	elevators.push_back(Elevator(2, "", 0, 1));
	// Elevator 2 is already heading to floor 5, so a CALL at 5 going UP merges
	// into that stop - the dry-run on the LAST elevator returns the existing task.
	Task toFive(0, "", GO, DEFAULT, 5, 1.0);
	unsigned long int stopFive = elevators[1].addTask(toFive, 1).getStopId();

	Task call(0, "", CALL, UP, 5, 2.0);
	GreedyNearestDispatchStrategy strategy;
	strategy.selectElevator(call, elevators);

	REQUIRE(routeIsEmpty(elevators[0])); // the dry-run on elevator 1 was undone
	REQUIRE(elevators[0].getStatus() == IDLE);
	// ... and elevator 2 still has exactly its original stop.
	REQUIRE(elevators[1].getStopArrivalTime(stopFive) == 5.0);
}

TEST_CASE("GreedyNearestDispatchStrategy restores route re-anchoring done by a front-of-queue dry-run", "[dispatch][regression]") {
	// A task that becomes the front of a busy elevator's queue makes addTask
	// re-anchor the leg in flight (processingTask/direction/leg start). Undoing
	// only the task would leave the elevator heading for the removed stop.
	std::vector<Elevator> elevators;
	elevators.push_back(Elevator(1, "", 0, 1));
	Task toNine(0, "", GO, DEFAULT, 9, 1.0);
	unsigned long int stopNine = elevators[0].addTask(toNine, 1).getStopId(); // busy, heading up to 9

	Task call(0, "", CALL, UP, 3, 2.0); // nearer stop on the way: goes to the front
	GreedyNearestDispatchStrategy strategy;
	strategy.selectElevator(call, elevators);

	REQUIRE(elevators[0].getProcessingTask().getDest() == 9);
	REQUIRE(elevators[0].getStopArrivalTime(stopNine) == 9.0);
}

TEST_CASE("ElevatorSystem enforces the dispatch contract even for a strategy that leaks", "[dispatch][regression]") {
	ElevatorSystem system(/*no_elevators=*/2, /*no_floors=*/10, /*speed=*/1.0f,
		std::make_shared<LeakyDispatch>());

	Task call(0, "", CALL, UP, 4, 1.0);
	REQUIRE(system.FindBestElevator(call) == 1);

	// The strategy committed the task into the elevator it was handed, but that
	// was a scratch copy: the real elevators are untouched.
	for (const Elevator& e : system.getElevators()) {
		REQUIRE(e.getStatus() == IDLE);
		REQUIRE(routeIsEmpty(e));
	}
}
