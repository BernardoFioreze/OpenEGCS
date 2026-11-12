#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <optional>
#include "ElevatorSystem.h"
#include "RandomProvider.h"
#include "CustomerFactory.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// Elevator::update computes `position` analytically from the leg in flight
// and clamps it to the leg's destination. Integrated tick by tick with
// nothing clamping it, position drifts outside [0, floors] under a saturated,
// reversal-heavy workload (20 floors, 4 elevators, saturated arrival rate,
// seed 7 reached -12.8 / 23.0). This scenario must stay exactly within
// [0, 20].
//
// Every assertion here is a BOUND (the cabin stays within [0, floors]), which
// is scale-free, so it holds under any travel model. It does NOT catch a cabin
// that overshoots its leg's destination but stays inside the building; the
// cases that do are the random-workload invariant in test_event_timestamps.cpp
// and "a kinematic cabin never passes the floor it is going to" in
// test_travel_time.cpp.
//
// This drives ElevatorSystem's own public API directly instead of going
// through Simulator, because Simulator::startSimulation() has no per-tick
// observation hook - the loop below mirrors Simulator::startSimulation()'s
// own event loop closely enough to be a faithful reproduction, using the
// minimum needed to observe position after each ElevatorSystem::update()
// call via the getElevators() accessor.
TEST_CASE("Elevator position stays within [0, floors] under a saturated, reversal-heavy workload", "[elevator][regression]") {
	const int floors = 20;
	const int elevatorCount = 4;
	const int customersToSimulate = 3000;
	const int arrivalLookahead = 5;

	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(7));
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(elevatorCount, floors, /*speed=*/1.0f, dispatchStrategy);
	CustomerFactory customerFactory(rng, floors, /*sane_customers=*/true);

	Timer clock;
	double enqTime = 0;
	unsigned long int custId = 1;
	unsigned long int evId = 1;
	int departed = 0;

	auto pushArrival = [&]() {
		enqTime += rng->nextExponential(3.0); // saturated: high arrival rate
		Customer* cus = new Customer(customerFactory.createArrival(custId, enqTime));
		Event arrival(evId, "", CUST_CALL, cus->getDir(), cus->getAt(), enqTime, cus);
		system.upcomingEvents.push(arrival);
		custId++;
		evId++;
	};

	for (int i = 0; i < arrivalLookahead; i++) pushArrival();

	while (departed < customersToSimulate) {
		Event toProcess = system.upcomingEvents.pop();
		clock.updateTick(toProcess.getTime());
		system.update(clock);

		int elevatorIndex = 0;
		for (const Elevator& e : system.getElevators()) {
			INFO("elevator index " << elevatorIndex << " position " << e.getPosition()
				<< " at time " << clock.getCurrentTime());
			REQUIRE(e.getPosition() >= -1e-6);
			REQUIRE(e.getPosition() <= floors + 1e-6);
			elevatorIndex++;
		}

		// Nothing is handed back to push, and an alighting is a record rather
		// than an event.
		system.giveEvent(toProcess);
		for (const ElevatorSystem::Transfer& tr : system.drainTransfers()) {
			if (tr.type == LEAVE) departed++;
		}
		if (toProcess.getType() == CUST_CALL) pushArrival();
	}
}
