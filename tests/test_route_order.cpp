#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ElevatorSystem.h"
#include "RandomProvider.h"
#include "CustomerFactory.h"
#include "FixedDwellTimeStrategy.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// LookSchedulingStrategy searches the route with a binary search, which
// assumes the vector is ordered by the key it compares with - but "route
// order" is relative to where the cabin is and which way it is going, so it
// has to be re-established when those change. A route built while going up
// stores the stops needing a reversal from the highest floor down; the
// instant the cabin reverses, the order those same stops must be served in is
// exactly inverted.
//
// These tests assert the *observable* consequence rather than the internal sort
// key, so they stay meaningful if the key is ever reformulated: a LOOK cabin
// sweeps one way, then turns around once. Its route therefore reads as one
// monotone run of floors followed by at most one more in the opposite direction
// - never a sawtooth such as (2, 0, 8, 4, 3, 2, 1, 0, 1, 0, 0, 1, 2, ...).

namespace {

// How many times the floor sequence changes direction. A correct LOOK route
// allows one: the reversal at the end of the current sweep.
int directionChanges(const std::vector<Task>& route) {
	int changes = 0;
	int previousSign = 0;
	for (size_t i = 0; i + 1 < route.size(); i++) {
		double delta = route[i + 1].getDest() - route[i].getDest();
		if (std::fabs(delta) < 0.001) continue;		// same floor twice: one stop, no movement
		int sign = (delta > 0) ? 1 : -1;
		if (previousSign != 0 && sign != previousSign) changes++;
		previousSign = sign;
	}
	return changes;
}

struct Scenario { int floors, elevators, customers; double rate; uint64_t seed; };

// Drives a full workload the way Simulator does, checking every elevator's route
// after every route change.
int worstDirectionChanges(const Scenario& s, std::shared_ptr<IDwellTimeStrategy> dwell = nullptr) {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(s.seed));
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(s.elevators, s.floors, /*speed=*/1.0f, dispatch, nullptr,
		std::numeric_limits<double>::infinity(), nullptr, dwell);
	CustomerFactory factory(rng, s.floors, /*sane_customers=*/true);

	double enqueued = 0;
	unsigned long int customerId = 1, eventId = 1;
	auto pushArrival = [&]() {
		enqueued += rng->nextExponential(s.rate);
		Customer* customer = new Customer(factory.createArrival(customerId, enqueued));
		Event arrival(eventId, "", CUST_CALL, customer->getDir(), customer->getAt(), enqueued, customer);
		system.upcomingEvents.push(arrival);
		customerId++;
		eventId++;
	};
	for (int i = 0; i < 5; i++) pushArrival();

	Timer clock;
	int served = 0, worst = 0;
	while (!system.upcomingEvents.empty() && served < s.customers) {
		Event toProcess = system.upcomingEvents.pop();
		clock.updateTick(toProcess.getTime());
		system.update(clock);
		// Nothing is handed back to push, and an alighting is a record rather
		// than an event.
		system.giveEvent(toProcess);
		for (const ElevatorSystem::Transfer& tr : system.drainTransfers()) {
			if (tr.type == LEAVE) served++;
		}
		if (toProcess.getType() == CUST_CALL) pushArrival();

		for (const Elevator& elevator : system.getElevators()) {
			int changes = directionChanges(elevator.getRoute());
			if (changes > worst) worst = changes;
		}
	}
	return worst;
}

} // namespace

TEST_CASE("A LOOK route sweeps one way and turns around at most once", "[route_order][regression]") {
	const Scenario scenarios[] = {
		{10, 3, 50, 0.8, 42},
		{8, 1, 60, 1.5, 3},			// the shape the sawtooth above came from
		{10, 2, 400, 2.5, 11},
		{12, 5, 300, 4.0, 1},
		{20, 4, 400, 3.0, 7},		// 79.5% of searches hit a disordered route before the fix
	};
	for (const Scenario& s : scenarios) {
		INFO("floors=" << s.floors << " elevators=" << s.elevators
			<< " customers=" << s.customers << " rate=" << s.rate << " seed=" << s.seed);
		REQUIRE(worstDirectionChanges(s) <= 1);
	}
}

TEST_CASE("A LOOK route keeps its shape with a stop duration", "[route_order][regression][dwell]") {
	// Door time changes when insertions happen relative to the cabin's movement,
	// so the ordering has to hold under it too.
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(2.5, 1.5, 0.75);
	const Scenario scenarios[] = {
		{12, 5, 300, 4.0, 1},
		{20, 4, 400, 3.0, 7},
	};
	for (const Scenario& s : scenarios) {
		INFO("floors=" << s.floors << " elevators=" << s.elevators << " seed=" << s.seed);
		REQUIRE(worstDirectionChanges(s, dwell) <= 1);
	}
}
