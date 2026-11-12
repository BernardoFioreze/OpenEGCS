#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <memory>
#include <vector>
#include "ElevatorSystem.h"
#include "Customer.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// A passenger's destination is a *car call*, and it has to be in the route
// before the cabin decides where to go next from the stop where they boarded.
// If it entered one step too late, the cabin would choose its next sweep
// direction while the passenger who had just stepped in existed nowhere in
// the route, and could carry them away from their destination.
//
// What is asserted here is the observable consequence, not the internals: a
// passenger is never carried past their boarding floor in the direction
// opposite to their destination. That survives a change of scheduling strategy,
// which an assertion about `direction` or about route order would not.
namespace {

// Minimal copy of Simulator's loop: pop, advance the clock, hand the event to
// the system, read back the transfers it reported. Returns, per customer, the
// floors the cabin visited while they were on board.
std::vector<std::pair<int, std::vector<int> > > ridesOf(ElevatorSystem& system) {
	Timer clock;
	std::vector<std::pair<int, std::vector<int> > > rides;
	std::vector<int> onBoard;
	while (!system.upcomingEvents.empty()) {
		Event ev = system.upcomingEvents.pop();
		clock.updateTick(ev.getTime());
		system.update(clock);

		// The transfers of this stop come back from the engine as records, so
		// they are read after the event is handed over rather than off the event
		// itself.
		system.giveEvent(ev);
		std::vector<ElevatorSystem::Transfer> transfers = system.drainTransfers();
		for (size_t t = 0 ; t < transfers.size() ; t++) {
			const ElevatorSystem::Transfer& tr = transfers[t];
			if (tr.type == ENTER) {
				onBoard.push_back((int)tr.customerId);
				rides.push_back(std::make_pair((int)tr.customerId, std::vector<int>()));
			}
			if (tr.type == ENTER || tr.type == LEAVE) {
				// Everyone currently riding has just experienced this floor.
				for (size_t i = 0 ; i < rides.size() ; i++) {
					if (std::find(onBoard.begin(), onBoard.end(), rides[i].first) == onBoard.end()) continue;
					rides[i].second.push_back(tr.floor);
				}
			}
			if (tr.type == LEAVE) {
				onBoard.erase(std::remove(onBoard.begin(), onBoard.end(), (int)tr.customerId),
					onBoard.end());
			}
		}
	}
	return rides;
}

} // namespace

TEST_CASE("A passenger is not carried away from their destination after boarding", "[car_call][regression]") {
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0f, dispatch);

	// `up` exists only so that the route holds something above floor 3 when
	// `down` boards there: that is what the cabin would turn towards if it did
	// not yet know that `down` wanted floor 0.
	std::vector<Customer> customers;
	customers.reserve(2);	// pointers into this vector are stored in events
	customers.emplace_back(1, "", 0.0, /*at=*/7, /*to=*/8, UP);
	customers.emplace_back(2, "", 1.0, /*at=*/3, /*to=*/0, DOWN);
	unsigned long int evId = 1;
	for (Customer& c : customers) {
		Event call(evId++, "", CUST_CALL, c.getDir(), c.getAt(), c.getTime(), &c);
		system.upcomingEvents.push(call);
	}

	std::vector<std::pair<int, std::vector<int> > > rides = ridesOf(system);
	REQUIRE(rides.size() == 2);

	for (size_t i = 0 ; i < rides.size() ; i++) {
		const Customer& c = customers[rides[i].first - 1];
		bool goingUp = c.getTo() > c.getAt();
		for (size_t k = 0 ; k < rides[i].second.size() ; k++) {
			int floor = rides[i].second[k];
			INFO("customer " << c.getId() << " boarded at " << c.getAt()
				<< " heading for " << c.getTo() << ", cabin took them to " << floor);
			// Never on the far side of the boarding floor from the destination.
			if (goingUp) REQUIRE(floor >= c.getAt());
			else REQUIRE(floor <= c.getAt());
		}
		REQUIRE(rides[i].second.back() == c.getTo());	// and they do arrive
	}
}
