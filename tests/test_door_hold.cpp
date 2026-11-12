#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <limits>
#include <stdexcept>

#include "DoorHoldDwellTimeStrategy.h"
#include "Elevator.h"
#include "Task.h"

using namespace openEGCS;
using Catch::Approx;

// Doors held open for a minimum time, Barney & Al-Sharif (2015) sec. 5.5.2.

namespace {

DoorHoldDwellTimeStrategy::Params officeDoor(bool detector)
{
	DoorHoldDwellTimeStrategy::Params p;
	p.doorOpenS = 2.5;
	p.doorCloseS = 3.0;
	p.perPassengerS = 1.2;
	p.detector = detector;
	return p;	// holds left at the sec. 5.5.2 defaults: 2.0 car call, 3.0 landing call, 0.5 detector
}

StopContext stop(int boarding, int alighting, bool carCall, bool hallCall)
{
	StopContext c;
	c.boarding = boarding;
	c.alighting = alighting;
	c.carCall = carCall;
	c.hallCall = hallCall;
	return c;
}

} // namespace

TEST_CASE("the hold is the car-call one only for a car call alone", "[door_hold]") {
	DoorHoldDwellTimeStrategy s(officeDoor(false));
	REQUIRE(s.holdFor(stop(0, 1, /*car*/true, /*hall*/false)) == 2.0);
	REQUIRE(s.holdFor(stop(1, 0, false, true)) == 3.0);
	REQUIRE(s.holdFor(stop(1, 1, true, true)) == 3.0);	// somebody still has to walk up
	// A stop with no request left was made for a landing call another cabin
	// took (a car call is never withdrawn), so it gets the landing hold.
	REQUIRE(s.holdFor(stop(0, 0, false, false)) == 3.0);
}

TEST_CASE("timed doors: the open phase is max(hold, transfer)", "[door_hold]") {
	DoorHoldDwellTimeStrategy s(officeDoor(false));
	// One person boarding at a landing call: 1.2 s of transfer, 3.0 s held.
	REQUIRE(s.getTransferTime(stop(1, 0, false, true)) == 3.0);
	// One person alighting on a car call: 1.2 s of transfer, 2.0 s held.
	REQUIRE(s.getTransferTime(stop(0, 1, true, false)) == 2.0);
	// Nobody moves: the timer runs out all the same.
	REQUIRE(s.getTransferTime(stop(0, 0, false, false)) == 3.0);
	// Five people: 6.0 s of transfer outlasts the hold; a hold never cuts a
	// transfer short.
	REQUIRE(s.getTransferTime(stop(3, 2, true, true)) == Approx(6.0));
	// Opening and closing are untouched.
	REQUIRE(s.getDoorOpenTime(stop(1, 0, false, true)) == 2.5);
	REQUIRE(s.getDoorCloseTime(stop(1, 0, false, true)) == 3.0);
}

TEST_CASE("a detector cuts the hold to 0.5 s once somebody crosses", "[door_hold]") {
	DoorHoldDwellTimeStrategy s(officeDoor(true));
	REQUIRE(s.getTransferTime(stop(1, 0, false, true)) == Approx(1.2 + 0.5));
	REQUIRE(s.getTransferTime(stop(0, 1, true, false)) == Approx(1.2 + 0.5));
	REQUIRE(s.getTransferTime(stop(3, 2, true, true)) == Approx(6.0 + 0.5));
	// Nobody crosses, so nothing cuts the timer.
	REQUIRE(s.getTransferTime(stop(0, 0, false, false)) == 3.0);
	REQUIRE(s.getTransferTime(stop(0, 0, true, false)) == 2.0);
}

TEST_CASE("door-hold times are validated like the fixed strategy's", "[door_hold]") {
	DoorHoldDwellTimeStrategy::Params p = officeDoor(false);
	p.landingCallHoldS = -1.0;
	REQUIRE_THROWS_AS(DoorHoldDwellTimeStrategy(p), std::invalid_argument);
	p = officeDoor(false);
	p.carCallHoldS = std::numeric_limits<double>::infinity();
	REQUIRE_THROWS_AS(DoorHoldDwellTimeStrategy(p), std::invalid_argument);
}

TEST_CASE("a stop tells the dwell strategy why the cabin stops there", "[door_hold][stop_context]") {
	// Elevator::stopContextOf reads the reason off the route entry. The engine
	// path is the one the hold depends on, so it is pinned here rather than
	// trusted.
	Task hall(1, "", CALL, UP, /*floor=*/4, 0.0);
	hall.initPassengerCounters();
	hall.addHallCall(UP);
	StopContext c = Elevator::stopContextOf(hall);
	REQUIRE(c.hallCall);
	REQUIRE_FALSE(c.carCall);

	Task go(2, "", GO, DEFAULT, /*floor=*/7, 0.0);
	go.initPassengerCounters();
	c = Elevator::stopContextOf(go);
	REQUIRE(c.carCall);
	REQUIRE_FALSE(c.hallCall);
}
