#include <catch2/catch_test_macros.hpp>
#include <memory>
#include "Elevator.h"
#include "ElevatorSystem.h"
#include "Customer.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// Events need a stable link to the route stop (Task) they belong to, because
// a pointer or iterator into Elevator's task vector is invalidated by every
// insertion. This pins down the contract of that link (Task::getStopId /
// Event::getStopId), which event retiming depends on.

TEST_CASE("Elevator::addTask assigns a unique, non-zero stopId to each new route entry", "[stop_identity][elevator]") {
	Elevator elevator(1, "", /*pos=*/0, /*time=*/1.0f);

	Task first(0, "", GO, DEFAULT, 5, 1.0);
	Task second(0, "", GO, DEFAULT, 8, 2.0);
	REQUIRE(first.getStopId() == 0); // unassigned until offered to an elevator

	Task& a = elevator.addTask(first, 1); // idle -> departs
	Task& b = elevator.addTask(second, 0);
	REQUIRE(a.getStopId() != 0);
	REQUIRE(b.getStopId() != 0);
	REQUIRE(a.getStopId() != b.getStopId());
}

TEST_CASE("Elevator::addTask keeps the existing stopId when a task is merged into an existing stop", "[stop_identity][elevator]") {
	Elevator elevator(1, "", /*pos=*/0, /*time=*/1.0f);

	Task first(0, "", GO, DEFAULT, 5, 1.0);
	unsigned long int firstStop = elevator.addTask(first, 1).getStopId();

	// Same floor, same sweep, different customer (different timestamp): the
	// elevator makes ONE stop for both - the merged request reports that stop.
	Task sameFloor(0, "", GO, DEFAULT, 5, 2.0);
	Task& merged = elevator.addTask(sameFloor, 0);
	REQUIRE(merged.getStopId() == firstStop);

	Task otherFloor(0, "", GO, DEFAULT, 7, 3.0);
	REQUIRE(elevator.addTask(otherFloor, 0).getStopId() != firstStop);
}

TEST_CASE("Task copies and assignments preserve stopId, equality ignores it", "[stop_identity][task]") {
	Task original(0, "", GO, DEFAULT, 5, 1.0);
	original.setStopId(42);

	Task copy(original);
	REQUIRE(copy.getStopId() == 42);

	Task assigned;
	assigned = original;
	REQUIRE(assigned.getStopId() == 42);

	// Dispatch's dry-run/undo (GreedyNearestDispatchStrategy) relies on
	// equality comparing the *request*, not which stop it landed in.
	Task unassigned(0, "", GO, DEFAULT, 5, 1.0);
	REQUIRE(original == unassigned);
}

TEST_CASE("Event copies preserve stopId", "[stop_identity][event]") {
	Event ev(1, "", ENTER, UP, 5, 3.0);
	REQUIRE(ev.getStopId() == 0);
	ev.setStopId(7);

	Event copy(ev);
	REQUIRE(copy.getStopId() == 7);
	Event assigned;
	assigned = ev;
	REQUIRE(assigned.getStopId() == 7);
}

TEST_CASE("ElevatorSystem gives each route stop exactly one ARRIVE, tagged with that stop", "[stop_identity][elevator_system]") {
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0f, dispatchStrategy);

	// Two customers calling from the same floor in the same direction share
	// one stop; a third calling from elsewhere gets its own.
	Customer a(1, "", 0.0, /*at_floor=*/5, /*to_floor=*/9, UP);
	Customer b(2, "", 0.5, /*at_floor=*/5, /*to_floor=*/8, UP);
	Customer c(3, "", 1.0, /*at_floor=*/7, /*to_floor=*/9, UP);

	Event callA(1, "", CUST_CALL, a.getDir(), a.getAt(), a.getTime(), &a);
	Event callB(2, "", CUST_CALL, b.getDir(), b.getAt(), b.getTime(), &b);
	Event callC(3, "", CUST_CALL, c.getDir(), c.getAt(), c.getTime(), &c);

	system.giveEvent(callA);
	system.giveEvent(callB);
	system.giveEvent(callC);

	// Two arrivals for three customers: a and b pressed the same button, so
	// they are one call on one stop with one ARRIVE, and c's floor is a second
	// stop. The stop identity this case is about is read off those events.
	REQUIRE(system.upcomingEvents.size() == 2);
	Event firstArrival = system.upcomingEvents.pop();
	Event secondArrival = system.upcomingEvents.pop();

	REQUIRE(firstArrival.getType() == ARRIVE);
	REQUIRE(secondArrival.getType() == ARRIVE);
	REQUIRE(firstArrival.getStopId() != 0);
	REQUIRE(secondArrival.getStopId() != 0);
	REQUIRE(firstArrival.getStopId() != secondArrival.getStopId());
	REQUIRE(firstArrival.getFloor() != secondArrival.getFloor());
}
