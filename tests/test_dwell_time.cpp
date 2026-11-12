#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include "FixedDwellTimeStrategy.h"
#include "LinearTravelTimeStrategy.h"
#include "ElevatorSystem.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// The IDwellTimeStrategy extension point and its default implementation:
// the plumbing (one shared instance per installation) and the values the
// default charges.

TEST_CASE("FixedDwellTimeStrategy defaults to no dwell at all", "[dwell]") {
	FixedDwellTimeStrategy strategy;
	StopContext stop{/*boarding=*/3, /*alighting=*/2};

	REQUIRE(strategy.getDoorOpenTime(stop) == 0.0);
	REQUIRE(strategy.getTransferTime(stop) == 0.0);
	REQUIRE(strategy.getDoorCloseTime(stop) == 0.0);
	REQUIRE(strategy.isZero());
}

TEST_CASE("FixedDwellTimeStrategy charges the transfer per passenger, the doors per stop", "[dwell]") {
	FixedDwellTimeStrategy strategy(/*door_open=*/2.0, /*door_close=*/3.0, /*per_passenger=*/0.5);

	StopContext onePassenger{/*boarding=*/1, /*alighting=*/0};
	StopContext fourPassengers{/*boarding=*/3, /*alighting=*/1};

	// The doors do not care how many people are moving.
	REQUIRE(strategy.getDoorOpenTime(onePassenger) == Catch::Approx(2.0));
	REQUIRE(strategy.getDoorOpenTime(fourPassengers) == Catch::Approx(2.0));
	REQUIRE(strategy.getDoorCloseTime(onePassenger) == Catch::Approx(3.0));
	REQUIRE(strategy.getDoorCloseTime(fourPassengers) == Catch::Approx(3.0));

	// The transfer does, and counts boarding and alighting alike.
	REQUIRE(strategy.getTransferTime(onePassenger) == Catch::Approx(0.5));
	REQUIRE(strategy.getTransferTime(fourPassengers) == Catch::Approx(2.0));
	REQUIRE_FALSE(strategy.isZero());
}

TEST_CASE("FixedDwellTimeStrategy rejects a dwell that is negative or not finite", "[dwell]") {
	// A negative dwell would run the clock backwards at a stop and a
	// non-finite one would poison every event timestamp derived from the
	// route, so both are refused at construction rather than surfacing later
	// as an impossible timeline.
	const double infinite = std::numeric_limits<double>::infinity();
	const double notANumber = std::numeric_limits<double>::quiet_NaN();

	REQUIRE_THROWS_AS(FixedDwellTimeStrategy(-1.0, 0.0, 0.0), std::invalid_argument);
	REQUIRE_THROWS_AS(FixedDwellTimeStrategy(0.0, -1.0, 0.0), std::invalid_argument);
	REQUIRE_THROWS_AS(FixedDwellTimeStrategy(0.0, 0.0, -1.0), std::invalid_argument);
	REQUIRE_THROWS_AS(FixedDwellTimeStrategy(infinite, 0.0, 0.0), std::invalid_argument);
	REQUIRE_THROWS_AS(FixedDwellTimeStrategy(0.0, notANumber, 0.0), std::invalid_argument);

	// Zero is explicitly legal: it is the default, and it is what keeps an
	// un-configured run identical to its pre-item-#9 results.
	REQUIRE_NOTHROW(FixedDwellTimeStrategy(0.0, 0.0, 0.0));
}

TEST_CASE("An elevator built without a dwell strategy gets a usable zero one", "[dwell][elevator]") {
	Elevator elevator(/*el_id=*/1, "", /*pos=*/0, /*time=*/1);

	// Never null - callers that time a route must not have to null-check it.
	REQUIRE(elevator.getDwellTimeStrategy() != nullptr);

	StopContext stop{/*boarding=*/2, /*alighting=*/2};
	REQUIRE(elevator.getDwellTimeStrategy()->getDoorOpenTime(stop) == 0.0);
	REQUIRE(elevator.getDwellTimeStrategy()->getTransferTime(stop) == 0.0);
	REQUIRE(elevator.getDwellTimeStrategy()->getDoorCloseTime(stop) == 0.0);
}

TEST_CASE("ElevatorSystem hands every elevator the same dwell strategy instance", "[dwell][elevator_system]") {
	// "All the cabins in a building have the same doors" should hold by
	// construction rather than by every elevator happening to build the same
	// default - the same reasoning as the shared ICapacityStrategy.
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(1.5, 2.5, 0.25);
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/4, /*no_floors=*/10, /*speed=*/1.0f, dispatchStrategy,
		/*metrics_collector=*/nullptr,
		/*max_weight=*/std::numeric_limits<double>::infinity(),
		/*capacity_strategy=*/nullptr, dwell);

	const auto& elevators = system.getElevators();
	REQUIRE(elevators.size() == 4);
	for (const Elevator& elevator : elevators) {
		REQUIRE(elevator.getDwellTimeStrategy().get() == dwell.get());
	}
}

TEST_CASE("ElevatorSystem defaults to a shared centre-door dwell strategy", "[dwell][elevator_system]") {
	// Item #56: the default is the handbook's centre-opening door, normal
	// opening, office transfer (DwellPresets.h). Item #9 shipped zero here, and
	// this case asserted it until the cited figure was read.
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/3, /*no_floors=*/10, /*speed=*/1.0f, dispatchStrategy);

	const auto& elevators = system.getElevators();
	const IDwellTimeStrategy* shared = elevators.front().getDwellTimeStrategy().get();
	REQUIRE(shared != nullptr);

	// Unlimited cabin -> 1100 mm door (sec. 5.3.3.1): 2.5 s open, 3.0 s close
	// (Table 5.3), 1.2 s per passenger (sec. 5.5.1), and the doors held open at
	// least 2.0 s for a car call / 3.0 s for a landing call (sec. 5.5.2).
	StopContext landing{/*boarding=*/1, /*alighting=*/1, /*carCall=*/true, /*hallCall=*/true};
	StopContext carOnly{/*boarding=*/0, /*alighting=*/1, /*carCall=*/true, /*hallCall=*/false};
	StopContext crowd{/*boarding=*/4, /*alighting=*/1, /*carCall=*/true, /*hallCall=*/true};
	for (const Elevator& elevator : elevators) {
		REQUIRE(elevator.getDwellTimeStrategy().get() == shared);
		REQUIRE(elevator.getDwellTimeStrategy()->getDoorOpenTime(landing) == 2.5);
		REQUIRE(elevator.getDwellTimeStrategy()->getDoorCloseTime(landing) == 3.0);
		REQUIRE(elevator.getDwellTimeStrategy()->getTransferTime(landing) == 3.0);	// 2 x 1.2 < 3.0
		REQUIRE(elevator.getDwellTimeStrategy()->getTransferTime(carOnly) == 2.0);	// 1.2 < 2.0
		REQUIRE(elevator.getDwellTimeStrategy()->getTransferTime(crowd) == Catch::Approx(6.0));	// 5 x 1.2 > 3.0
	}
}

TEST_CASE("The default door width follows the installation's rated load", "[dwell][elevator_system]") {
	// A 600 kg group gets the 800 mm centre door (2.0 / 2.0 s): the width is
	// what ties the default preset to the cabin it is used with.
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/2, /*no_floors=*/10, /*speed=*/1.0f, dispatchStrategy,
		/*metrics=*/nullptr, /*max_weight=*/600.0);
	StopContext stop{/*boarding=*/0, /*alighting=*/0};
	const auto& dwell = system.getElevators().front().getDwellTimeStrategy();
	REQUIRE(dwell->getDoorOpenTime(stop) == 2.0);
	REQUIRE(dwell->getDoorCloseTime(stop) == 2.0);
}

TEST_CASE("An Elevator built directly keeps the zero dwell of a component", "[dwell]") {
	// Plan decision D6: the default belongs to the installation (ElevatorSystem),
	// which knows the rated load; a bare Elevator, as unit tests build it, keeps
	// the instantaneous stop - the same split as the travel model's.
	Elevator elevator(0, "", 0, 1.0);
	StopContext stop{/*boarding=*/1, /*alighting=*/0};
	REQUIRE(elevator.getDwellTimeStrategy()->getDoorOpenTime(stop) == 0.0);
	REQUIRE(elevator.getDwellTimeStrategy()->getTransferTime(stop) == 0.0);
}

// ---------------------------------------------------------------------------
// The time model. These pin the exact instants, because "the numbers went
// up" is not evidence that a stop is modelled correctly.
//
// All of them use one elevator at floor 0 with staticTime 1.0 (one floor per
// time unit) and a dwell of door-open 2, door-close 3, 1 per passenger, so
// every expected value below can be worked out by hand in the comment next to
// it rather than copied from a run.
//
// They inject LinearTravelTimeStrategy EXPLICITLY, because "one floor per time
// unit" is what makes those by-hand numbers checkable, and the default is the
// kinematic model. These cases are about the DWELL - that a stop costs
// door-open before the boarding and transfer plus door-close after it - and
// the travel model they sit on is scaffolding; the kinematic model has its own
// cases in test_travel_time.cpp.
// ---------------------------------------------------------------------------

namespace {

ElevatorSystem oneElevatorWithDwell() {
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(/*open=*/2.0, /*close=*/3.0, /*per_passenger=*/1.0);
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	return ElevatorSystem(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0f, dispatchStrategy,
		/*metrics_collector=*/nullptr,
		/*max_weight=*/std::numeric_limits<double>::infinity(),
		/*capacity_strategy=*/nullptr, dwell,
		std::make_shared<LinearTravelTimeStrategy>(1.0));
}

} // namespace

TEST_CASE("A single customer's stop costs door-open before boarding and transfer plus door-close after", "[dwell][timing]") {
	ElevatorSystem system = oneElevatorWithDwell();

	Customer customer(1, "", /*t=*/0.0, /*at_floor=*/5, /*to_floor=*/8, UP);
	Event call(1, "", CUST_CALL, customer.getDir(), customer.getAt(), customer.getTime(), &customer);
	system.giveEvent(call);
	Event arrival = system.upcomingEvents.pop();

	// Cabin leaves floor 0 at once and takes 5 time units to reach floor 5;
	// the doors then take 2 more before anyone can step in. The instant the
	// stop's event carries is the instant of the boarding it produces: both
	// come from getStopEventTime.
	REQUIRE(arrival.getType() == ARRIVE);
	REQUIRE(arrival.getFloor() == 5);
	REQUIRE(arrival.getTime() == Catch::Approx(7.0));	// 5 travel + 2 door open
	REQUIRE(system.drainDepartures().size() == 1);		// the trip that left floor 0

	Timer clock;
	clock.updateTick(arrival.getTime());
	system.update(clock);
	REQUIRE(system.getElevators()[0].getPosition() == Catch::Approx(5.0));

	// Serving the stop boards the passenger and registers their car call, which
	// is the stop at their destination.
	system.giveEvent(arrival);
	REQUIRE(system.drainTransfers().size() == 1);

	// One passenger transfers (1) and the doors close (3), so the cabin cannot
	// leave floor 5 before t = 7 + 4 = 11. The queue no longer goes empty for
	// an instant here, because the car call is registered before the stop is
	// popped - but legStartTime still has to carry the tail, which is what
	// this assertion pins.
	auto departures = system.drainDepartures();
	REQUIRE(departures.size() == 1);
	REQUIRE(departures[0].fromFloor == 5);
	REQUIRE(departures[0].time == Catch::Approx(11.0));

	// Then 3 floors of travel and another door opening at the destination.
	Event destination = system.upcomingEvents.pop();
	REQUIRE(destination.getType() == ARRIVE);
	REQUIRE(destination.getFloor() == 8);
	REQUIRE(destination.getTime() == Catch::Approx(16.0));	// 11 + 3 travel + 2 door open
}

TEST_CASE("The cabin stays put for the whole dwell", "[dwell][timing]") {
	ElevatorSystem system = oneElevatorWithDwell();

	Customer customer(1, "", 0.0, /*at_floor=*/5, /*to_floor=*/8, UP);
	Event call(1, "", CUST_CALL, customer.getDir(), customer.getAt(), customer.getTime(), &customer);
	system.giveEvent(call);
	Event arrival = system.upcomingEvents.pop();

	Timer clock;
	clock.updateTick(arrival.getTime());
	system.update(clock);
	system.giveEvent(arrival);

	// Boarding at 7, departure at 11: the cabin is at floor 5 throughout, and
	// this needs no special case in Elevator::update - a legStartTime in the
	// future already clamps the travelled distance to zero.
	for (double t : {7.0, 8.0, 9.5, 10.999}) {
		clock.updateTick(t);
		system.update(clock);
		REQUIRE(system.getElevators()[0].getPosition() == Catch::Approx(5.0));
	}

	// Moving again right after.
	clock.updateTick(12.0);
	system.update(clock);
	REQUIRE(system.getElevators()[0].getPosition() == Catch::Approx(6.0));	// 1 unit past floor 5
}

TEST_CASE("A stop shared by two customers pays the doors once and the transfer per passenger", "[dwell][timing]") {
	// The per-passenger term scales with the people at the stop, the door terms
	// do not.
	ElevatorSystem system = oneElevatorWithDwell();

	Customer first(1, "", 0.0, /*at_floor=*/5, /*to_floor=*/8, UP);
	Customer second(2, "", 0.0, /*at_floor=*/5, /*to_floor=*/9, UP);
	Event firstCall(1, "", CUST_CALL, first.getDir(), first.getAt(), first.getTime(), &first);
	Event secondCall(2, "", CUST_CALL, second.getDir(), second.getAt(), second.getTime(), &second);
	system.giveEvent(firstCall);
	system.giveEvent(secondCall);

	// One stop, ONE event, two transfers: the door opens once, not twice.
	REQUIRE(system.upcomingEvents.size() == 1);
	Event arrival = system.upcomingEvents.pop();
	REQUIRE(arrival.getTime() == Catch::Approx(7.0));
	system.drainDepartures();

	Timer clock;
	clock.updateTick(arrival.getTime());
	system.update(clock);

	system.giveEvent(arrival);
	REQUIRE(system.drainTransfers().size() == 2);

	// Two passengers transfer (2 x 1) and the doors close once (3), so the
	// cabin leaves floor 5 at 7 + 5 = 12 - not 7 + 4 = 11 as it would for a
	// single passenger, and not 7 + 2x(1+3) for charging the doors per customer.
	auto departures = system.drainDepartures();
	REQUIRE(departures.size() == 1);
	REQUIRE(departures[0].fromFloor == 5);
	REQUIRE(departures[0].time == Catch::Approx(12.0));

	// And that departure stands: a stop fires once, so nothing later in the
	// same stop can reset it.
	REQUIRE(system.getElevators()[0].getLegStartTime() == Catch::Approx(12.0));
}

TEST_CASE("A stop inserted mid-route pushes every later stop by its whole dwell", "[dwell][timing]") {
	ElevatorSystem system = oneElevatorWithDwell();

	// First customer going up to floor 9 from floor 8.
	Customer far(1, "", 0.0, /*at_floor=*/8, /*to_floor=*/9, UP);
	Event farCall(1, "", CUST_CALL, far.getDir(), far.getAt(), far.getTime(), &far);
	system.giveEvent(farCall);
	// The stop's event is already in the queue - the engine pushes it itself -
	// so retimePendingEvents can reach it.
	REQUIRE(system.upcomingEvents.size() == 1);

	// A second customer calls from floor 3, on the way: the cabin now stops
	// there first, and floor 8 is pushed back by that stop's whole dwell -
	// door open 2 + one passenger 1 + door close 3 = 6 - plus nothing extra in
	// travel, since floor 3 is on the way.
	Customer near(2, "", 0.0, /*at_floor=*/3, /*to_floor=*/7, UP);
	Event nearCall(2, "", CUST_CALL, near.getDir(), near.getAt(), near.getTime(), &near);
	system.giveEvent(nearCall);

	// Both stops are pending, and the one at floor 8 must have been re-timed by
	// retimePendingEvents.
	bool foundFar = false, foundNear = false;
	while (!system.upcomingEvents.empty()) {
		Event pending = system.upcomingEvents.pop();
		REQUIRE(pending.getType() == ARRIVE);
		if (pending.getFloor() == 8) {
			foundFar = true;
			REQUIRE(pending.getTime() == Catch::Approx(16.0));	// 10 + 6 of dwell at floor 3
		}
		if (pending.getFloor() == 3) {
			foundNear = true;
			REQUIRE(pending.getTime() == Catch::Approx(5.0));	// 3 travel + 2 door open
		}
	}
	REQUIRE(foundFar);
	REQUIRE(foundNear);
}

// ---------------------------------------------------------------------------
// A stop keeps every request merged into it.
// ---------------------------------------------------------------------------

TEST_CASE("A stop merged at a sweep reversal keeps both directions", "[dwell][stop_identity]") {
	// The case this exists for: the cabin is going up, someone called UP from the
	// top floor and someone else is waiting there to go DOWN. A real car opens its
	// doors once and takes both. The route entry must therefore record that it
	// serves downward demand too - collapsing it to a single direction field would
	// throw that away, and LookSchedulingStrategy reads exactly this to order the
	// route.
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(2.0, 3.0, 1.0);
	Elevator elevator(1, "", /*pos=*/0, /*time=*/1.0f, nullptr,
		std::numeric_limits<double>::infinity(), nullptr, dwell);

	Task upCall(0, "", CALL, UP, /*dest=*/8, 0.0);
	Task& stop = elevator.addTask(upCall, 1);
	REQUIRE(stop.hasHallCall(UP));
	REQUIRE_FALSE(stop.hasHallCall(DOWN));

	Task downCall(0, "", CALL, DOWN, /*dest=*/8, 0.1);
	Task& merged = elevator.addTask(downCall, 1);

	// One stop, not two.
	REQUIRE(merged.getStopId() == stop.getStopId());

	// And it now serves both: nothing was discarded.
	REQUIRE(merged.hasHallCall(UP));
	REQUIRE(merged.hasHallCall(DOWN));
	REQUIRE(merged.servesSweep(UP));
	REQUIRE(merged.servesSweep(DOWN));

	// Two passengers at one stop, so the doors open once and the transfer is
	// charged twice: 2 + 2x1 + 3.
	REQUIRE(merged.getBoardingPassengers() == 2);
	REQUIRE(elevator.getStopArrivalTime(merged.getStopId()) == Catch::Approx(8.0));
	REQUIRE(elevator.getStopEventTime(merged.getStopId()) == Catch::Approx(10.0));
}

TEST_CASE("A car call makes a stop reachable in either sweep", "[dwell][stop_identity]") {
	// A passenger inside the cabin wanting floor 4 must be let out on whichever
	// sweep passes floor 4 - it is not a hall call and has no direction of its
	// own, so servesSweep answers yes to both.
	Task carCall(0, "", GO, DEFAULT, /*dest=*/4, 0.0);
	REQUIRE(carCall.hasCarCallRequest());
	REQUIRE(carCall.servesSweep(UP));
	REQUIRE(carCall.servesSweep(DOWN));

	// A hall call only answers for the direction its passenger asked for.
	Task hallUp(0, "", CALL, UP, /*dest=*/4, 0.0);
	REQUIRE_FALSE(hallUp.hasCarCallRequest());
	REQUIRE(hallUp.servesSweep(UP));
	REQUIRE_FALSE(hallUp.servesSweep(DOWN));
}

TEST_CASE("A leg that is over keeps its end while the doors open", "[dwell]") {
	// A cabin five floors from a stop, at 1 s/floor, reaches it at t = 5 and its doors
	// take 2 s, so the ARRIVE is at 7. Asked at t = 6 the arrival is still 5, for the
	// event time and for the dispatch cost alike. Answering "now", 6, would restart
	// the door opening every time the route is touched.
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(2.0, 3.0, 1.0);
	Elevator cabin(1, "", 0.0, 1, nullptr, std::numeric_limits<double>::infinity(), nullptr, dwell);
	Task go(0, "", GO, DEFAULT, 5, 0.0);
	Task added = cabin.addTask(go, 1);

	cabin.update(Timer(6.0));

	REQUIRE(cabin.getStopArrivalTime(added.getStopId()) == Catch::Approx(5.0));
	REQUIRE(cabin.getStopEventTime(added.getStopId()) == Catch::Approx(7.0));
	REQUIRE(cabin.getFinishTime(go) == Catch::Approx(5.0));
}

