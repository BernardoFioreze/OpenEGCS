#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <vector>
#include "ElevatorSystem.h"
#include "Customer.h"
#include "StandardMetricsCollector.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// ElevatorSystem's public observation API, tested directly rather than only
// through Simulator-level end-to-end runs: getElevators() (read-only
// observation, also used by test_elevator_position_bounds.cpp) and
// drainDepartures()/Departure (the signal that feeds the snapshot's "DEPART"
// records, see ElevatorSystem.h and ElevatorSystem::giveEvent).

TEST_CASE("ElevatorSystem::getElevators exposes read-only elevator state", "[elevator_system]") {
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/3, /*no_floors=*/10, /*speed=*/1.0f, dispatchStrategy);

	const auto& elevators = system.getElevators();
	REQUIRE(elevators.size() == 3);
	for (const Elevator& e : elevators) {
		REQUIRE(e.getPosition() == 0.0f);
		REQUIRE(e.getStatus() == IDLE);
	}
}

TEST_CASE("ElevatorSystem::drainDepartures reports a fresh departure and clears itself", "[elevator_system][departure]") {
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/2, /*no_floors=*/10, /*speed=*/1.0f, dispatchStrategy);

	REQUIRE(system.drainDepartures().empty()); // nothing has happened yet

	Customer customer(1, "", /*t=*/0.0, /*at_floor=*/5, /*to_floor=*/9, UP);
	Event call(1, "", CUST_CALL, customer.getDir(), customer.getAt(), customer.getTime(), &customer);
	system.giveEvent(call);

	auto departures = system.drainDepartures();
	REQUIRE(departures.size() == 1);
	REQUIRE(departures[0].elevatorId >= 1);
	REQUIRE(departures[0].elevatorId <= 2);
	REQUIRE(departures[0].fromFloor == 0); // both elevators start at floor 0
	REQUIRE(departures[0].time == 0.0);

	// Draining again returns nothing until the next departure happens.
	REQUIRE(system.drainDepartures().empty());
}

TEST_CASE("ElevatorSystem::drainDepartures reports nothing when the elevator was already moving", "[elevator_system][departure]") {
	// giveEvent only records a departure when the elevator transitions
	// IDLE->BUSY (a real "start of a new trip") - a second call picked up by
	// an already-moving elevator is just a route update, not a departure.
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0f, dispatchStrategy);

	Customer first(1, "", 0.0, /*at_floor=*/5, /*to_floor=*/9, UP);
	Event firstCall(1, "", CUST_CALL, first.getDir(), first.getAt(), first.getTime(), &first);
	system.giveEvent(firstCall);
	REQUIRE(system.drainDepartures().size() == 1);

	Customer second(2, "", 0.0, /*at_floor=*/7, /*to_floor=*/9, UP);
	Event secondCall(2, "", CUST_CALL, second.getDir(), second.getAt(), second.getTime(), &second);
	system.giveEvent(secondCall);
	REQUIRE(system.drainDepartures().empty());
}

namespace {

// Records every onElevatorDeparted call so a test can count them, while still
// behaving like the real collector (a duplicate charge would inflate the
// energy model).
struct DepartureCountingMetrics : public StandardMetricsCollector {
	std::vector<double> departureDistances;
	std::vector<double> departureLoads;
	void onElevatorDeparted(int elevatorId, double floorsTravelled, double currentLoad, double time) override {
		departureDistances.push_back(floorsTravelled);
		departureLoads.push_back(currentLoad);
		StandardMetricsCollector::onElevatorDeparted(elevatorId, floorsTravelled, currentLoad, time);
	}
};

} // namespace

// A departure belongs to the *stop*, not to each transfer at it.
// Elevator::addTask merges tasks for the same floor into a single stop, so the
// leg that follows such a stop must be reported exactly once. Reporting one
// departure per boarding customer would write two DEPART records for one leg
// and charge that leg twice in the energy model.
TEST_CASE("A stop shared by merged tasks reports exactly one departure", "[elevator_system][departure][regression]") {
	auto metrics = std::make_shared<DepartureCountingMetrics>();
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0f, dispatchStrategy, metrics);

	// Two customers call from the same floor at the same instant: the single
	// elevator merges both CALL tasks, so both ENTERs share one stop and fire
	// together.
	Customer first(1, "", 0.0, /*at_floor=*/5, /*to_floor=*/9, UP);
	Customer second(2, "", 0.0, /*at_floor=*/5, /*to_floor=*/8, UP);
	Event firstCall(1, "", CUST_CALL, first.getDir(), first.getAt(), first.getTime(), &first);
	Event secondCall(2, "", CUST_CALL, second.getDir(), second.getAt(), second.getTime(), &second);
	system.giveEvent(firstCall);
	system.giveEvent(secondCall);
	// One stop, one ARRIVE, two people behind it: the second press joined the
	// call the first one lit.
	REQUIRE(system.upcomingEvents.size() == 1);
	Event arrival = system.upcomingEvents.pop();
	REQUIRE(arrival.getType() == ARRIVE);

	// Move the clock to the stop and forget the DEPART of the trip that brought
	// the cabin there - only the leg leaving this stop is under test.
	Timer clock;
	clock.updateTick(arrival.getTime());
	system.update(clock);
	system.drainDepartures();

	// NOTE: the metrics are deliberately NOT cleared here. A leg is priced when
	// it ARRIVES, so at this point nothing has been charged at all - the 0 -> 5
	// trip is still in flight and gets its charge when the cabin leaves the stop
	// below. Clearing here would discard nothing and then count the arriving leg
	// as if it were the departing one.
	REQUIRE(metrics->departureDistances.empty());

	// Serve the stop. One event does the whole of it: both passengers board,
	// each registering their own car call, and the records come back through
	// drainTransfers.
	system.giveEvent(arrival);
	std::vector<ElevatorSystem::Transfer> transfers = system.drainTransfers();
	REQUIRE(transfers.size() == 2);
	REQUIRE(transfers[0].type == ENTER);
	REQUIRE(transfers[1].type == ENTER);
	REQUIRE(transfers[0].time == transfers[1].time);	// one stop, one instant

	// Exactly one DEPART for the merged stop - the property this case is named
	// for: it is about how many legs LEAVE the stop, and that is observable at
	// departure time.
	REQUIRE(system.drainDepartures().size() == 1);

	// And exactly one charge so far: the 0 -> 5 leg that arrived, priced with
	// the distance it really covered and with nobody aboard. The leg leaving
	// floor 5 is in flight and will be charged when it gets somewhere, which is
	// why this case cannot count the departing leg's charge at this instant.
	REQUIRE(metrics->departureDistances.size() == 1);
	REQUIRE(metrics->departureDistances[0] == 5.0);
	REQUIRE(metrics->departureLoads[0] == 0.0);
}

// The energy of a leg is priced on the load the cabin carries, and the
// boarding passengers of the stop the leg starts from are part of that load:
// they must be on board before the departure is reported, just as the
// alighting ones are already off.
//
// No magic number: the load reported for the leg must equal the sum of the
// weights of everyone the test put on board.
TEST_CASE("The leg leaving a stop is priced with every boarding passenger aboard", "[elevator_system][departure][regression]") {
	auto metrics = std::make_shared<DepartureCountingMetrics>();
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0f, dispatchStrategy, metrics);

	// Three customers call from the same floor at the same instant, so all
	// three CALL tasks merge into one stop and their ENTERs share it.
	Customer first(1, "", 0.0, /*at_floor=*/5, /*to_floor=*/9, UP, /*weight=*/70.0);
	Customer second(2, "", 0.0, /*at_floor=*/5, /*to_floor=*/8, UP, /*weight=*/80.0);
	Customer third(3, "", 0.0, /*at_floor=*/5, /*to_floor=*/7, UP, /*weight=*/90.0);
	Event firstCall(1, "", CUST_CALL, first.getDir(), first.getAt(), first.getTime(), &first);
	Event secondCall(2, "", CUST_CALL, second.getDir(), second.getAt(), second.getTime(), &second);
	Event thirdCall(3, "", CUST_CALL, third.getDir(), third.getAt(), third.getTime(), &third);
	system.giveEvent(firstCall);
	system.giveEvent(secondCall);
	system.giveEvent(thirdCall);
	// Three people, one lit button, one stop, one ARRIVE.
	REQUIRE(system.upcomingEvents.size() == 1);
	Event arrival = system.upcomingEvents.pop();

	Timer clock;
	clock.updateTick(arrival.getTime());
	system.update(clock);
	system.drainDepartures();

	// All three board on this one event. The leg decision is held until the
	// last of them, and the loop over the stop's transfers is what knows which
	// one is last.
	system.giveEvent(arrival);
	REQUIRE(system.drainTransfers().size() == 3);

	// Exactly one DEPART for the merged stop, and the leg it starts is in
	// flight: a leg is priced when it ARRIVES, with the distance it really
	// covered, so the load captured at departure is not observable through this
	// hook until the cabin gets somewhere. The only charge so far is the 0 -> 5
	// leg that arrived, carrying nobody.
	REQUIRE(system.drainDepartures().size() == 1);
	REQUIRE(metrics->departureLoads.size() == 1);
	REQUIRE(metrics->departureLoads[0] == 0.0);

	// So drive the cabin to the end of that leg. The LOOK route from floor 5
	// going UP serves 7 first, and the lowest car call is the third customer's.
	Event nextStop = system.upcomingEvents.pop();
	REQUIRE(nextStop.getFloor() == 7);
	clock.updateTick(nextStop.getTime());
	system.update(clock);
	system.giveEvent(nextStop);

	// Now the leg that left the stop is priced - with all three aboard. The
	// load is the one captured at DEPARTURE, not the one at arrival, which is
	// why it still counts three passengers even though one of them has just got
	// out.
	REQUIRE(metrics->departureLoads.size() == 2);
	REQUIRE(metrics->departureLoads[1] == 70.0 + 80.0 + 90.0);
	REQUIRE(metrics->departureDistances[1] == 2.0);	// 5 -> 7, the distance really covered
}
