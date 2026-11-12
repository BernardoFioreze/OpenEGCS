#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <memory>
#include <vector>
#include "ElevatorSystem.h"
#include "IGroupDispatchStrategy.h"
#include "ImmediateGroupAdapter.h"
#include "IDispatchStrategy.h"
#include "IRandomProvider.h"
#include "Customer.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// The group dispatch extension point (IGroupDispatchStrategy) and its
// behaviour-preserving default (ImmediateGroupAdapter): the seam is a real,
// injectable extension point, and the adapter delegates exactly once, to the
// right car, drawing no randomness.

namespace {

// Records what selectElevator was asked and how often, and returns a fixed id.
// The fields, not a copy of the Task, because what matters is that the adapter
// built the CALL request from the call (type/direction/floor/first-press time),
// not the Task object's identity.
struct SpyDispatch : IDispatchStrategy {
	int returnId = 1;
	int calls = 0;
	unsigned int seenType = 0;
	unsigned int seenDirection = 0;
	int seenDest = 0;
	double seenTime = -1.0;
	int selectElevator(Task task, std::vector<Elevator>& /*elevators*/) override {
		calls++;
		seenType = task.getType();
		seenDirection = task.getDirection();
		seenDest = task.getDest();
		seenTime = task.getTime();
		return returnId;
	}
};

// Counts every draw so a test can assert the default adapter takes none.
struct CountingRng : IRandomProvider {
	int draws = 0;
	double nextExponential(double /*rate*/) override { draws++; return 0.0; }
	int nextUniformInt(int lo, int /*hi*/) override { draws++; return lo; }
	double nextNormal(double mean, double /*stddev*/) override { draws++; return mean; }
};

// A group strategy that ignores the per-call policy entirely and sends every
// call to car 2. It exists to prove the engine routes allocation through the
// injected IGroupDispatchStrategy and not through the raw IDispatchStrategy:
// with two idle cars at floor 0, the greedy would pick car 1 on the tie, so an
// observed car 2 can only have come from here.
struct AlwaysSecond : IGroupDispatchStrategy {
	std::vector<int> allocate(const std::vector<HallCall>& calls,
		const std::vector<Elevator>& /*elevators*/, IRandomProvider& /*rng*/) override {
		return std::vector<int>(calls.size(), 2);
	}
};

// Forces every call onto one fixed cabin, so a test can pile two stops on one
// car (the greedy would spread them) and then reassign the one behind the front.
struct ToCabin : IDispatchStrategy {
	int id;
	explicit ToCabin(int cabin) : id(cabin) {}
	int selectElevator(Task /*task*/, std::vector<Elevator>& /*elevators*/) override { return id; }
};

// True if the cabin's route holds a stop at `floor`.
bool routeHasFloor(const Elevator& e, int floor) {
	for (const Task& t : e.getRoute()) {
		if (t.getDest() == floor) return true;
	}
	return false;
}

// A group strategy that forces every pool call onto cabin 1 and, once floor 7
// up is allocated there and not committed, moves it to cabin 2. Everything else
// stays where it is. Used to prove the reoptimize trigger fires through
// giveEvent end to end (§9.4), not just that the default is a no-op.
struct ForcePoolToOneThenMoveSeven : IGroupDispatchStrategy {
	std::vector<int> allocate(const std::vector<HallCall>& calls,
		const std::vector<Elevator>& /*elevators*/, IRandomProvider& /*rng*/) override {
		std::vector<int> r;
		for (const HallCall& c : calls) {
			if (c.currentElevatorId == 0) r.push_back(1);					// pool -> cabin 1
			else if (c.floor == 7 && c.direction == UP) r.push_back(2);		// move floor 7 to cabin 2
			else r.push_back(c.currentElevatorId);							// keep
		}
		return r;
	}
};

std::vector<Elevator> twoIdleCars() {
	std::vector<Elevator> cars;
	cars.push_back(Elevator(1, "", 0, 1));
	cars.push_back(Elevator(2, "", 0, 1));
	return cars;
}

} // namespace

TEST_CASE("ImmediateGroupAdapter rebuilds the CALL request and delegates once", "[group_dispatch]") {
	auto spy = std::make_shared<SpyDispatch>();
	spy->returnId = 2;
	ImmediateGroupAdapter adapter(spy);

	std::vector<Elevator> cars = twoIdleCars();
	CountingRng rng;
	HallCall call{/*floor=*/7, /*direction=*/UP, /*registeredAt=*/3.5,
		/*currentElevatorId=*/0, /*committed=*/false};

	std::vector<int> picks = adapter.allocate(std::vector<HallCall>{call}, cars, rng);

	REQUIRE(picks.size() == 1);
	REQUIRE(picks[0] == 2);		// the id the inner policy returned, passed straight through
	REQUIRE(spy->calls == 1);	// delegated exactly once
	// The Task the adapter handed the inner policy: CALL, the call's own
	// direction and floor, and its first-press instant (not `now`).
	REQUIRE(spy->seenType == CALL);
	REQUIRE(spy->seenDirection == UP);
	REQUIRE(spy->seenDest == 7);
	REQUIRE(spy->seenTime == 3.5);
	// The default adapter is deterministic by construction: it draws nothing.
	REQUIRE(rng.draws == 0);
}

TEST_CASE("ImmediateGroupAdapter keeps a committed call on its car without consulting the inner policy", "[group_dispatch]") {
	auto spy = std::make_shared<SpyDispatch>();
	ImmediateGroupAdapter adapter(spy);

	std::vector<Elevator> cars = twoIdleCars();
	CountingRng rng;
	// A committed call already belongs to car 2 and cannot be moved (a braking
	// cabin cannot un-brake). The adapter must hand car 2 back untouched.
	HallCall committed{/*floor=*/4, /*direction=*/DOWN, /*registeredAt=*/1.0,
		/*currentElevatorId=*/2, /*committed=*/true};

	std::vector<int> picks = adapter.allocate(std::vector<HallCall>{committed}, cars, rng);

	REQUIRE(picks.size() == 1);
	REQUIRE(picks[0] == 2);
	REQUIRE(spy->calls == 0);	// a committed call is never offered to the inner policy
	REQUIRE(rng.draws == 0);
}

TEST_CASE("ImmediateGroupAdapter keeps an allocated non-committed call on its car", "[group_dispatch]") {
	// A call that already has a cabin keeps it even when it is NOT committed:
	// the adapter never reallocates, so when the engine offers already-allocated
	// calls to the group, the default answers "leave it where it is".
	auto spy = std::make_shared<SpyDispatch>();
	ImmediateGroupAdapter adapter(spy);

	std::vector<Elevator> cars = twoIdleCars();
	CountingRng rng;
	HallCall allocated{/*floor=*/6, /*direction=*/UP, /*registeredAt=*/2.0,
		/*currentElevatorId=*/1, /*committed=*/false};

	std::vector<int> picks = adapter.allocate(std::vector<HallCall>{allocated}, cars, rng);

	REQUIRE(picks.size() == 1);
	REQUIRE(picks[0] == 1);		// kept on its current car...
	REQUIRE(spy->calls == 0);	// ...without asking the inner policy to reconsider
	REQUIRE(rng.draws == 0);
}

TEST_CASE("ImmediateGroupAdapter preserves order across a batch", "[group_dispatch]") {
	auto spy = std::make_shared<SpyDispatch>();
	spy->returnId = 1;
	ImmediateGroupAdapter adapter(spy);

	std::vector<Elevator> cars = twoIdleCars();
	CountingRng rng;
	// All three kinds in one batch, each in its own slot: a pool call goes to
	// the inner id (1), a committed one keeps its car (2), an allocated
	// non-committed one keeps its car (1), and another pool call to the inner.
	std::vector<HallCall> batch = {
		{3, UP, 0.0, 0, false},		// pool -> inner -> 1
		{5, DOWN, 1.0, 2, true},	// committed -> keep 2
		{2, DOWN, 0.5, 1, false},	// allocated, not committed -> keep 1
		{8, UP, 2.0, 0, false},		// pool -> inner -> 1
	};

	std::vector<int> picks = adapter.allocate(batch, cars, rng);

	REQUIRE(picks.size() == 4);
	REQUIRE(picks[0] == 1);
	REQUIRE(picks[1] == 2);
	REQUIRE(picks[2] == 1);
	REQUIRE(picks[3] == 1);
	REQUIRE(spy->calls == 2);	// only the two pool calls reach the inner policy
}

TEST_CASE("ImmediateGroupAdapter refuses a null inner policy", "[group_dispatch]") {
	REQUIRE_THROWS_AS(ImmediateGroupAdapter(nullptr), std::invalid_argument);
}

TEST_CASE("ElevatorSystem routes allocation through the injected group strategy", "[group_dispatch]") {
	// Both cars idle at floor 0, so the greedy per-call policy would answer a
	// call at floor 5 with car 1. Injecting a group strategy that forces car 2
	// and observing car 2 move is proof the engine consults the group seam, not
	// the raw dispatch underneath it.
	auto greedy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto group = std::make_shared<AlwaysSecond>();
	ElevatorSystem system(/*no_elevators=*/2, /*no_floors=*/10, /*speed=*/1.0f,
		greedy, /*metrics=*/nullptr, std::numeric_limits<double>::infinity(),
		/*capacity=*/nullptr, /*dwell=*/nullptr, /*travel=*/nullptr,
		/*passenger_behavior=*/nullptr, /*random_provider=*/nullptr,
		/*group_dispatch_strategy=*/group);

	Customer customer(1, "", /*t=*/0.0, /*at_floor=*/5, /*to_floor=*/9, UP);
	Event call(1, "", CUST_CALL, customer.getDir(), customer.getAt(), customer.getTime(), &customer);
	system.giveEvent(call);

	const std::vector<Elevator>& cars = system.getElevators();
	REQUIRE(cars[0].getStatus() == IDLE);		// car 1 was NOT chosen...
	REQUIRE(cars[1].getStatus() != IDLE);		// ...car 2 was, because the group strategy said so
}

TEST_CASE("the reoptimize trigger moves an allocated call at the arrival instant", "[group_dispatch]") {
	// End to end: a group strategy forces calls onto cabin 1 and moves floor 7 to
	// cabin 2. The reoptimize pass at the END of the CUST_CALL branch makes the
	// move happen in floor 7's OWN arrival event - "each time t the set is
	// re-allocated" - not one event later.
	auto greedy = std::make_shared<examples::GreedyNearestDispatchStrategy>();	// required non-null; unused on the allocation path
	auto group = std::make_shared<ForcePoolToOneThenMoveSeven>();
	ElevatorSystem system(/*no_elevators=*/2, /*no_floors=*/10, /*speed=*/1.0f,
		greedy, nullptr, std::numeric_limits<double>::infinity(),
		nullptr, nullptr, nullptr, nullptr, nullptr, group);

	// Floor 3 arrives first and becomes cabin 1's committed front.
	Customer a(1, "", 0.0, /*at=*/3, /*to=*/9, UP);
	system.giveEvent(Event(1, "", CUST_CALL, a.getDir(), a.getAt(), a.getTime(), &a));

	// Floor 7 arrives: allocateCall places it on cabin 1 (the group forces pool
	// calls there), then the post-registration reoptimize immediately moves it to
	// cabin 2 - all within this one event.
	Customer b(2, "", 0.0, /*at=*/7, /*to=*/9, UP);
	system.giveEvent(Event(2, "", CUST_CALL, b.getDir(), b.getAt(), b.getTime(), &b));

	REQUIRE_FALSE(routeHasFloor(system.getElevators()[0], 7));	// cabin 1 gave floor 7 up, same event
	REQUIRE(routeHasFloor(system.getElevators()[1], 7));		// cabin 2 took it
	REQUIRE(routeHasFloor(system.getElevators()[0], 3));		// its committed front is untouched

	std::vector<HallCall> pending = system.snapshotPendingCalls();
	bool sevenOnTwo = false;
	for (const HallCall& h : pending) {
		if (h.floor == 7 && h.direction == UP) { REQUIRE(h.currentElevatorId == 2); sevenOnTwo = true; }
	}
	REQUIRE(sevenOnTwo);
}

TEST_CASE("ElevatorSystem::snapshotPendingCalls tags each registered call with its cabin and commitment", "[group_dispatch]") {
	// The pending-demand snapshot the reassignment pass reads. One idle cabin,
	// two up-calls: it heads to floor 3 first (its route front, so committed -
	// cannot be reassigned) and floor 7 sits behind it in the sweep (allocated,
	// not committed - reassignable).
	auto greedy = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0f, greedy);

	REQUIRE(system.snapshotPendingCalls().empty());		// nothing registered yet

	Customer a(1, "", /*t=*/0.0, /*at=*/3, /*to=*/9, UP);
	system.giveEvent(Event(1, "", CUST_CALL, a.getDir(), a.getAt(), a.getTime(), &a));
	Customer b(2, "", /*t=*/0.0, /*at=*/7, /*to=*/9, UP);
	system.giveEvent(Event(2, "", CUST_CALL, b.getDir(), b.getAt(), b.getTime(), &b));

	std::vector<HallCall> pending = system.snapshotPendingCalls();
	REQUIRE(pending.size() == 2);
	// callIndex order: floor 3 (index 6) before floor 7 (index 14).
	REQUIRE(pending[0].floor == 3);
	REQUIRE(pending[0].direction == UP);
	REQUIRE(pending[0].currentElevatorId == 1);
	REQUIRE(pending[0].committed == true);		// the route front: not reassignable
	REQUIRE(pending[1].floor == 7);
	REQUIRE(pending[1].direction == UP);
	REQUIRE(pending[1].currentElevatorId == 1);
	REQUIRE(pending[1].committed == false);		// behind the front: reassignable
}

TEST_CASE("ElevatorSystem::reassign moves a non-committed call between cabins", "[group_dispatch]") {
	// Force both up-calls onto cabin 1 (ToCabin), so floor 3 is its committed
	// front and floor 7 sits behind it. Then move floor 7 to the idle cabin 2.
	auto toOne = std::make_shared<ToCabin>(1);
	ElevatorSystem system(/*no_elevators=*/2, /*no_floors=*/10, /*speed=*/1.0f, toOne);

	Customer a(1, "", /*t=*/0.0, /*at=*/3, /*to=*/9, UP);
	system.giveEvent(Event(1, "", CUST_CALL, a.getDir(), a.getAt(), a.getTime(), &a));
	Customer b(2, "", /*t=*/0.0, /*at=*/7, /*to=*/9, UP);
	system.giveEvent(Event(2, "", CUST_CALL, b.getDir(), b.getAt(), b.getTime(), &b));

	const std::vector<Elevator>& cars = system.getElevators();
	REQUIRE(routeHasFloor(cars[0], 7));			// precondition: floor 7 is on cabin 1
	REQUIRE(cars[1].getStatus() == IDLE);		// cabin 2 is free

	SECTION("a committed call cannot be moved") {
		// Floor 3 is cabin 1's front: the move is refused and nothing changes.
		REQUIRE(system.reassign(3, UP, /*to=*/2, /*now=*/0.0) == false);
		REQUIRE(routeHasFloor(cars[0], 3));
		REQUIRE(cars[1].getStatus() == IDLE);
	}

	SECTION("an already-on-that-cabin or unregistered move is refused") {
		REQUIRE(system.reassign(7, UP, /*to=*/1, /*now=*/0.0) == false);	// already on cabin 1
		REQUIRE(system.reassign(5, UP, /*to=*/2, /*now=*/0.0) == false);	// nobody waiting at 5
	}

	SECTION("a non-committed call moves, as a unit, to the other cabin") {
		REQUIRE(system.reassign(7, UP, /*to=*/2, /*now=*/0.0) == true);

		// Cabin 1 lost floor 7 but kept its committed front at 3; cabin 2 now
		// heads to 7.
		REQUIRE_FALSE(routeHasFloor(cars[0], 7));
		REQUIRE(routeHasFloor(cars[0], 3));
		REQUIRE(routeHasFloor(cars[1], 7));
		REQUIRE(cars[1].getStatus() != IDLE);

		// The landing call now points at cabin 2, and is its committed front.
		std::vector<HallCall> pending = system.snapshotPendingCalls();
		REQUIRE(pending.size() == 2);
		REQUIRE(pending[0].floor == 3);
		REQUIRE(pending[0].currentElevatorId == 1);
		REQUIRE(pending[1].floor == 7);
		REQUIRE(pending[1].currentElevatorId == 2);
		REQUIRE(pending[1].committed == true);		// it is cabin 2's only stop, so its front
	}
}
