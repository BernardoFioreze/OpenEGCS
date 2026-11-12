#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "Elevator.h"
#include "ElevatorSystem.h"
#include "Task.h"
#include "Customer.h"
#include "Event.h"
#include "StandardMetricsCollector.h"
#include "IDispatchStrategy.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// A landing call is one per (floor, direction), not one per passenger.
//
// This file starts at the bottom, with the primitive the rest is built on
// (Elevator::joinHallCall): the second person to press an already-lit button
// must join the route entry the first person's call created, without
// consulting the dispatcher and without going anywhere near addTask's
// insertion logic.

TEST_CASE("A second passenger joins the stop that already serves their call", "[hall_call][elevator]") {
	Elevator e(1, "", /*pos=*/0, /*time=*/1.0);
	Task call(0, "", CALL, UP, /*destination=*/5, /*time=*/0.0);
	Task& placed = e.addTask(call, 1);
	unsigned long int stop = placed.getStopId();
	REQUIRE(stop != 0);
	REQUIRE(e.getRoute().size() == 1);
	REQUIRE(e.getRoute().front().getBoardingPassengers() == 1);

	const Task* joined = e.joinHallCall(stop, UP);

	// The entry they joined is the live route entry, not a copy: the stop they
	// will be boarded at is the one the cabin is actually going to.
	REQUIRE(joined != nullptr);
	REQUIRE(joined->getStopId() == stop);
	REQUIRE(joined == &e.getRoute().front());

	// One more person to pick up at the same stop - which is what the dwell is
	// priced on - and NOT a second stop. No route entry was created, no
	// position in the route changed, and no stop id was consumed.
	REQUIRE(e.getRoute().size() == 1);
	REQUIRE(e.getRoute().front().getBoardingPassengers() == 2);
	REQUIRE(e.getRoute().front().getDest() == 5);
	REQUIRE(e.getStopEventTime(stop) >= 0);
}

TEST_CASE("Joining with the other direction makes the stop serve both sweeps", "[hall_call][elevator]") {
	// A floor where somebody wants up and somebody wants down is one stop that
	// the cabin serves in whichever sweep reaches it - the same property
	// Elevator::addTask's merge relies on. Joining has to preserve it, or the
	// scheduling strategy stops seeing that there is downward demand at that
	// floor.
	Elevator e(1, "", /*pos=*/0, /*time=*/1.0);
	Task upCall(0, "", CALL, UP, /*destination=*/7, /*time=*/0.0);
	Task& placed = e.addTask(upCall, 1);
	unsigned long int stop = placed.getStopId();
	REQUIRE(e.getRoute().front().servesSweep(UP));
	REQUIRE_FALSE(e.getRoute().front().servesSweep(DOWN));

	REQUIRE(e.joinHallCall(stop, DOWN) != nullptr);

	REQUIRE(e.getRoute().size() == 1);
	REQUIRE(e.getRoute().front().servesSweep(UP));
	REQUIRE(e.getRoute().front().servesSweep(DOWN));
	REQUIRE(e.getRoute().front().getBoardingPassengers() == 2);
}

TEST_CASE("Joining and releasing are mirror images, down to the counters", "[hall_call][elevator]") {
	// releaseHallCall takes one hall call back out of a stop when the cabin
	// filled up; joinHallCall puts one in. The round trip must land exactly where
	// it started, because the two run against the same stop in the same run: a
	// cabin can pick up a joined passenger, fill up, and give the rest of that
	// stop back.
	Elevator e(1, "", /*pos=*/0, /*time=*/1.0);
	Task first(0, "", CALL, UP, /*destination=*/4, /*time=*/0.0);
	e.addTask(first, 1);
	// The round trip is done on the SECOND stop, because the front of the
	// route is the one the cabin is committed to and releaseHallCall refuses to
	// drop it - a cabin in flight cannot un-brake. That exemption is asserted at
	// the end of this case.
	Task later(0, "", CALL, UP, /*destination=*/9, /*time=*/0.0);
	Task& placed = e.addTask(later, 1);
	unsigned long int stop = placed.getStopId();
	REQUIRE(e.getRoute().size() == 2);

	REQUIRE(e.joinHallCall(stop, UP) != nullptr);
	REQUIRE(e.getRoute().back().getBoardingPassengers() == 2);

	REQUIRE(e.releaseHallCall(stop, UP));
	REQUIRE(e.getRoute().size() == 2);				// still serving the first passenger
	REQUIRE(e.getRoute().back().getBoardingPassengers() == 1);

	// And releasing the original one too empties the stop, which is then
	// dropped - proof that joining did not leave a phantom request behind that
	// would keep a pointless stop alive.
	REQUIRE(e.releaseHallCall(stop, UP));
	REQUIRE(e.getRoute().size() == 1);
	REQUIRE(e.getRoute().front().getDest() == 4);

	// The committed stop keeps its place even with nobody left to pick up,
	// which is the deliberate exception: the cabin still stops there. Joining
	// it afterwards is still legal and still finds it.
	unsigned long int committed = e.getRoute().front().getStopId();
	REQUIRE(e.releaseHallCall(committed, UP));
	REQUIRE(e.getRoute().size() == 1);
	REQUIRE(e.getRoute().front().getBoardingPassengers() == 0);
	REQUIRE(e.joinHallCall(committed, UP) != nullptr);
	REQUIRE(e.getRoute().front().getBoardingPassengers() == 1);
}

TEST_CASE("Joining a stop that is not in the route reports so and changes nothing", "[hall_call][elevator]") {
	// This is the caller's test for "is the allocation still alive": a stop
	// that has already been served is gone from the route, and the passenger
	// has to be offered to the dispatcher again rather than attached to a
	// cabin that has left.
	Elevator e(1, "", /*pos=*/0, /*time=*/1.0);
	Task call(0, "", CALL, UP, /*destination=*/5, /*time=*/0.0);
	Task& placed = e.addTask(call, 1);
	unsigned long int stop = placed.getStopId();

	REQUIRE(e.joinHallCall(stop + 999, UP) == nullptr);
	REQUIRE(e.joinHallCall(0, UP) == nullptr);
	REQUIRE(e.getRoute().size() == 1);
	REQUIRE(e.getRoute().front().getBoardingPassengers() == 1);

	e.processNextTask(stop);
	REQUIRE(e.getRoute().empty());
	REQUIRE(e.joinHallCall(stop, UP) == nullptr);	// served, so nobody joins it
}

TEST_CASE("A hall call without a direction is a programming error, not a no-op", "[hall_call][elevator]") {
	// Silently accepting it would put a passenger in the stop's boarding count
	// with no direction keeping the stop alive, so releaseHallCall could never
	// take them back out - a leak that only shows up as a dwell charged for a
	// transfer that never happens.
	Elevator e(1, "", /*pos=*/0, /*time=*/1.0);
	Task call(0, "", CALL, UP, /*destination=*/5, /*time=*/0.0);
	Task& placed = e.addTask(call, 1);
	REQUIRE_THROWS_AS(e.joinHallCall(placed.getStopId(), DEFAULT), std::logic_error);
	REQUIRE_THROWS_AS(e.joinHallCall(placed.getStopId(), GO), std::logic_error);
	REQUIRE(e.getRoute().front().getBoardingPassengers() == 1);
}

// ---------------------------------------------------------------------------
// The call itself: one per (floor, direction), not one per passenger
// ---------------------------------------------------------------------------

namespace {

// Counts how often the engine asks "which elevator?". It must be asked once
// per lit button and not once per person: in the saturated reference
// scenario that is about 12 thousand consultations for 3,000 passengers,
// against more than a million if every person were a call.
struct CountingDispatch : public IDispatchStrategy {
	std::shared_ptr<IDispatchStrategy> inner;
	mutable int calls = 0;

	CountingDispatch() : inner(std::make_shared<examples::GreedyNearestDispatchStrategy>()) {}
	int selectElevator(Task task, std::vector<Elevator>& elevators) override {
		calls++;
		return inner->selectElevator(task, elevators);
	}
};

struct Fixture {
	std::shared_ptr<CountingDispatch> dispatch;
	std::shared_ptr<StandardMetricsCollector> metrics;
	ElevatorSystem system;

	Fixture(int elevators = 1, int floors = 10,
		double maxWeight = std::numeric_limits<double>::infinity())
		: dispatch(std::make_shared<CountingDispatch>()),
		  metrics(std::make_shared<StandardMetricsCollector>()),
		  system(elevators, floors, 1.0, dispatch, metrics, maxWeight) {}

	void arrive(Customer& cus) {
		Event call(0, "", CUST_CALL, cus.getDir(), cus.getAt(), cus.getTime(), &cus);
		// Nothing comes back to push: the stop's ARRIVE is pushed by the engine
		// itself.
		system.giveEvent(call);
	}

	// Drives whatever is in the queue, discarding the transfer records.
	void drive(int steps = 2000) {
		for (int i = 0; i < steps && !system.upcomingEvents.empty(); i++) {
			Event next = system.upcomingEvents.pop();
			system.giveEvent(next);
			system.drainTransfers();
		}
	}
};

} // namespace

TEST_CASE("Two people pressing the same button are one call and one dispatch decision", "[hall_call][elevator_system]") {
	Fixture f;
	Customer first(1, "", /*t=*/0.0, /*at=*/5, /*to=*/9, UP, /*weight=*/70.0);
	Customer second(2, "", /*t=*/1.0, /*at=*/5, /*to=*/8, UP, /*weight=*/70.0);

	f.arrive(first);
	REQUIRE(f.dispatch->calls == 1);
	f.arrive(second);
	// The light was already on: no second dispatch decision, and no second
	// cabin sent to the same floor for the same direction.
	REQUIRE(f.dispatch->calls == 1);

	// Both are booked onto the same stop of the same cabin, and that stop knows
	// it is serving two transfers - which is what the dwell is priced on.
	REQUIRE(f.system.getElevators().front().getRoute().size() == 1);
	const Task& stop = f.system.getElevators().front().getRoute().front();
	REQUIRE(stop.getDest() == 5);
	REQUIRE(stop.getBoardingPassengers() == 2);

	// ONE event for the stop, however many people are behind the call. The two
	// passengers are in the queue of the call and board when that event fires.
	unsigned long int stopId = stop.getStopId();
	REQUIRE(f.system.upcomingEvents.size() == 1);
	Event arrival = f.system.upcomingEvents.pop();
	REQUIRE(arrival.getType() == ARRIVE);
	REQUIRE(arrival.getStopId() == stopId);
	REQUIRE(arrival.getElevator() == 1);

	Timer clock;
	clock.updateTick(arrival.getTime());
	f.system.update(clock);
	f.system.giveEvent(arrival);

	// Both board, in arrival order: first come, first served, which is the one
	// ordering rule that needs no figure from anywhere.
	std::vector<ElevatorSystem::Transfer> transfers = f.system.drainTransfers();
	REQUIRE(transfers.size() == 2);
	REQUIRE(transfers[0].type == ENTER);
	REQUIRE(transfers[1].type == ENTER);
	REQUIRE(transfers[0].customerId == 1);
	REQUIRE(transfers[1].customerId == 2);
}

TEST_CASE("The same floor in the other direction is a different call", "[hall_call][elevator_system]") {
	// Two buttons per landing, and they are not the same button. Collapsing
	// them would send one cabin to serve people going opposite ways.
	Fixture f;
	Customer up(1, "", /*t=*/0.0, /*at=*/5, /*to=*/9, UP, /*weight=*/70.0);
	Customer down(2, "", /*t=*/1.0, /*at=*/5, /*to=*/1, DOWN, /*weight=*/70.0);

	f.arrive(up);
	f.arrive(down);
	REQUIRE(f.dispatch->calls == 2);
}

TEST_CASE("A call is cancelled when the car arrives, so the next arrival lights a new one", "[hall_call][elevator_system]") {
	// Peters section 8.3.5: CancelLandingCalls, "cancel landing call when lift
	// arrives at floor". Without it the registry would keep a call alive
	// forever and a later passenger at that floor would be attached to a stop
	// that has already happened.
	Fixture f;
	Customer first(1, "", /*t=*/0.0, /*at=*/5, /*to=*/9, UP, /*weight=*/70.0);
	f.arrive(first);
	REQUIRE(f.dispatch->calls == 1);

	// Drive the boarding and the alighting through.
	f.drive();

	Customer later(2, "", /*t=*/50.0, /*at=*/5, /*to=*/8, UP, /*weight=*/70.0);
	f.arrive(later);
	REQUIRE(f.dispatch->calls == 2);		// a fresh button press, a fresh decision
}

TEST_CASE("The number of registered calls is bounded by the building, not by the crowd", "[hall_call][elevator_system]") {
	// The invariant of the call model. Twelve people at three floors are at
	// most six calls (three floors x two directions), and here exactly three,
	// because they all want to go the same way.
	Fixture f(/*elevators=*/1, /*floors=*/10, /*maxWeight=*/100.0);
	std::vector<Customer> crowd;
	for (int i = 0; i < 12; i++) {
		crowd.push_back(Customer(i + 1, "", /*t=*/i * 0.5, /*at=*/3 + (i % 3), /*to=*/9, UP, /*weight=*/70.0));
	}
	for (size_t i = 0; i < crowd.size(); i++) f.arrive(crowd[i]);

	// Then run the thing, which is what makes the cabin fill up and start
	// refusing: arrivals alone allocate everybody to the one empty cabin, and
	// the bound is only interesting once there is congestion to bound.
	int maxCalls = 0;
	int maxPassengers = 0;
	for (int step = 0; step < 2000 && !f.system.upcomingEvents.empty(); step++) {
		Event next = f.system.upcomingEvents.pop();
		f.system.giveEvent(next);
		f.system.drainTransfers();
		if (f.system.getUnallocatedCallCount() > maxCalls) maxCalls = f.system.getUnallocatedCallCount();
		if (f.system.getUnallocatedPassengerCount() > maxPassengers) {
			maxPassengers = f.system.getUnallocatedPassengerCount();
		}
		// The invariant that gives the item its name, checked at EVERY step and
		// not just at the end: a building of N floors has 2(N+1) buttons, and
		// no amount of demand can register more calls than it has buttons.
		REQUIRE(f.system.getUnallocatedCallCount() <= 2 * (10 + 1));
	}

	// And the point of it: the crowd is bigger than the number of buttons they
	// are standing under.
	REQUIRE(maxPassengers > maxCalls);
	REQUIRE(maxCalls <= 3);			// three floors, all of them going up
}

TEST_CASE("Unallocated calls and unallocated passengers are different questions", "[hall_call][elevator_system]") {
	// getUnallocatedCallCount is capped at two per floor by construction, so it
	// cannot measure congestion; getUnallocatedPassengerCount is the number for
	// that. This case keeps the two apart, so neither silently becomes the
	// other.
	Fixture f(/*elevators=*/1, /*floors=*/10, /*maxWeight=*/100.0);
	Customer riding(1, "", /*t=*/0.0, /*at=*/5, /*to=*/9, UP, /*weight=*/70.0);
	f.arrive(riding);
	// Board them, so the cabin is full and the next calls have nowhere to go.
	Event arrival = f.system.upcomingEvents.pop();
	REQUIRE(arrival.getType() == ARRIVE);
	f.system.giveEvent(arrival);
	REQUIRE(f.system.drainTransfers().size() == 1);
	REQUIRE(f.system.getElevators().front().isBypassingCalls());

	Customer a(2, "", /*t=*/1.0, /*at=*/2, /*to=*/8, UP, /*weight=*/70.0);
	Customer b(3, "", /*t=*/2.0, /*at=*/2, /*to=*/7, UP, /*weight=*/70.0);
	f.arrive(a);
	f.arrive(b);

	REQUIRE(f.system.getUnallocatedCallCount() == 1);		// one lit button
	REQUIRE(f.system.getUnallocatedPassengerCount() == 2);	// two people under it
}
