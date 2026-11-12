#include <catch2/catch_test_macros.hpp>
#include <vector>
#include "GreedyNearestDispatchStrategy.h"
#include "Elevator.h"
#include "Task.h"

using namespace openEGCS;
using namespace openEGCS::examples;

TEST_CASE("GreedyNearestDispatchStrategy picks the closest idle elevator", "[dispatch]") {
	std::vector<Elevator> elevators;
	elevators.push_back(Elevator(1, "", 0, 1));	// at floor 0
	elevators.push_back(Elevator(2, "", 10, 1));	// at floor 10

	// A call placed at floor 8, wanting to go UP: elevator 2 (closer) should win.
	Task callTask(0, "", CALL, UP, 8, 0);

	GreedyNearestDispatchStrategy strategy;
	int chosen = strategy.selectElevator(callTask, elevators);

	REQUIRE(chosen == 2);
}

TEST_CASE("GreedyNearestDispatchStrategy does not leave side effects on elevators", "[dispatch]") {
	std::vector<Elevator> elevators;
	elevators.push_back(Elevator(1, "", 0, 1));
	elevators.push_back(Elevator(2, "", 10, 1));

	Task callTask(0, "", CALL, UP, 8, 0);

	GreedyNearestDispatchStrategy strategy;
	strategy.selectElevator(callTask, elevators);

	// Dry-run insertion must be undone: both elevators should remain IDLE.
	REQUIRE(elevators[0].getStatus() == IDLE);
	REQUIRE(elevators[1].getStatus() == IDLE);
}
