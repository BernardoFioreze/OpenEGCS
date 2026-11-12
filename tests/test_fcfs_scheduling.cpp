#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "Elevator.h"
#include "LinearTravelTimeStrategy.h"
#include "Task.h"
#include "Timer.h"
#include "FcfsSchedulingStrategy.h"

using namespace openEGCS;

// examples::FcfsSchedulingStrategy, the naive
// route baseline. The cabin-level cases replay scenarios from
// test_elevator_look.cpp on purpose - same calls, same order - so that the
// difference between the two policies is stated side by side: where LOOK
// serves a nearer request first or defers an opposite one to the reversal,
// FCFS serves floors in the order they were asked for.
//
// Same technique as test_elevator_look.cpp, and for the same reason: a
// one-unit-per-floor linear clock, so "move the cabin to floor 7" is "advance
// the clock by 7".

namespace {

Task peekFront(Elevator& e) {
	return e.processNextTask();
}

Task advanceAndPop(Elevator& e, double& simTime, double dest) {
	simTime += std::abs(dest - e.getPosition());
	e.update(Timer(simTime));
	return e.processNextTask();
}

Elevator fcfsElevator(double position = 0) {
	return Elevator(1, "", position, /*time=*/1, std::make_shared<examples::FcfsSchedulingStrategy>(),
		std::numeric_limits<double>::infinity(), nullptr, nullptr,
		std::make_shared<LinearTravelTimeStrategy>(1.0));
}

std::vector<Task> routeOf(std::initializer_list<int> floors) {
	std::vector<Task> route;
	for (int f : floors) route.push_back(Task(0, "", GO, DEFAULT, f, 0));
	return route;
}

} // namespace

TEST_CASE("FCFS sends a floor the route does not have to the back", "[fcfs][scheduling]") {
	examples::FcfsSchedulingStrategy fcfs;
	std::vector<Task> route = routeOf({7, 2, 5});
	Task callAt3(0, "", CALL, UP, 3, 0);
	REQUIRE(fcfs.findInsertionPoint(route, UP, 0.0, callAt3) == route.end());
}

TEST_CASE("FCFS returns a floor's existing stop wherever it sits in the route", "[fcfs][scheduling]") {
	// Not only the last entry: the first request for a floor fixed its place in
	// the queue, and a later one joins it there (one stop per floor).
	examples::FcfsSchedulingStrategy fcfs;
	std::vector<Task> route = routeOf({7, 2, 5});
	Task goTo7(0, "", GO, DEFAULT, 7, 0);
	Task callAt2(0, "", CALL, DOWN, 2, 0);
	REQUIRE(fcfs.findInsertionPoint(route, UP, 0.0, goTo7) == route.begin());
	REQUIRE(fcfs.findInsertionPoint(route, UP, 0.0, callAt2) == route.begin() + 1);
}

TEST_CASE("FCFS does not read the cabin's direction or position", "[fcfs][scheduling]") {
	examples::FcfsSchedulingStrategy fcfs;
	std::vector<Task> route = routeOf({7, 2, 5});
	Task callAt3(0, "", CALL, DOWN, 3, 0);
	Task goTo5(0, "", GO, DEFAULT, 5, 0);
	for (unsigned int direction : {UP, DOWN}) {
		for (double position : {0.0, 4.5, 9.0}) {
			INFO("direction " << direction << ", position " << position);
			REQUIRE(fcfs.findInsertionPoint(route, direction, position, callAt3) == route.end());
			REQUIRE(fcfs.findInsertionPoint(route, direction, position, goTo5) == route.begin() + 2);
		}
	}
}

TEST_CASE("FCFS leaves a route untouched when asked to reorder it", "[fcfs][scheduling]") {
	// The default reorder: arrival order does not depend on the cabin's state.
	examples::FcfsSchedulingStrategy fcfs;
	std::vector<Task> route = routeOf({7, 2, 5});
	fcfs.reorder(route, DOWN, 9.0);
	REQUIRE(route[0].getDest() == 7);
	REQUIRE(route[1].getDest() == 2);
	REQUIRE(route[2].getDest() == 5);
}

TEST_CASE("An FCFS cabin moving UP does not put a nearer request first", "[fcfs][scheduling][elevator]") {
	// test_elevator_look.cpp, "Elevator moving UP inserts a nearer request before
	// a farther one": LOOK's route is [3, 5]. FCFS keeps the one it already had.
	Elevator e = fcfsElevator();
	double simTime = 0;
	Task go5(0, "", GO, DEFAULT, 5, 0);
	e.addTask(go5, 1);
	Task callAt3(0, "", CALL, UP, 3, 0);
	e.addTask(callAt3, 1);

	REQUIRE(peekFront(e).getDest() == 5);
	REQUIRE(advanceAndPop(e, simTime, 5).getDest() == 3);
}

TEST_CASE("An FCFS cabin serves a DOWN call ahead of it in arrival order", "[fcfs][scheduling][elevator]") {
	// test_elevator_look.cpp, "defers a DOWN hall call ahead of it until the
	// reversal": LOOK serves 3, 5, then 4. FCFS serves 5, 3, 4 - it turns around
	// twice, which is rule 4 of Closs broken by design (see the header).
	Elevator e = fcfsElevator();
	double simTime = 0;
	Task go5(0, "", GO, DEFAULT, 5, 0);
	e.addTask(go5, 1);
	Task callAt3Up(0, "", CALL, UP, 3, 0);
	e.addTask(callAt3Up, 1);
	Task callAt4Down(0, "", CALL, DOWN, 4, 0);
	e.addTask(callAt4Down, 1);

	REQUIRE(peekFront(e).getDest() == 5);
	REQUIRE(advanceAndPop(e, simTime, 5).getDest() == 3);
	REQUIRE(advanceAndPop(e, simTime, 3).getDest() == 4);
}

TEST_CASE("An FCFS cabin makes one stop for a floor asked for twice", "[fcfs][scheduling][elevator]") {
	Elevator e = fcfsElevator();
	Task go5(0, "", GO, DEFAULT, 5, 0);
	e.addTask(go5, 1);
	Task callAt3(0, "", CALL, UP, 3, 0);
	e.addTask(callAt3, 1);
	Task goTo5Again(0, "", GO, DEFAULT, 5, 0);
	e.addTask(goTo5Again, 1);

	REQUIRE(e.getRoute().size() == 2);
	REQUIRE(e.getRoute()[0].getDest() == 5);
	REQUIRE(e.getRoute()[1].getDest() == 3);
}

TEST_CASE("An FCFS cabin never retargets the leg it is on", "[fcfs][scheduling][elevator]") {
	// LOOK retargets a cabin heading for 9 when a call at 4 arrives mid-leg
	// (the new stop goes to the front). FCFS cannot: a new stop goes behind at
	// least one existing one, so the front stays the floor it was heading to.
	Elevator e = fcfsElevator();
	Task go9(0, "", GO, DEFAULT, 9, 0);
	e.addTask(go9, 1);
	e.update(Timer(2.0));			// two floors into a nine-floor leg
	REQUIRE(e.getPosition() > 1.0);
	Task callAt4(0, "", CALL, UP, 4, 0);
	e.addTask(callAt4, 1);
	REQUIRE(e.getRoute().front().getDest() == 9);
	REQUIRE(e.getProcessingTask().getDest() == 9);
}
