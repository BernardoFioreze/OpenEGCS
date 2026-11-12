#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <vector>
#include "ElevatorSystem.h"
#include "FixedDwellTimeStrategy.h"
#include "RandomProvider.h"
#include "CustomerFactory.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// The invariant that ties the clock to the cabin: when the ARRIVE event of a
// stop of elevator `e` at floor `f` fires, e's cabin is at floor `f`
// (|position - f| < 0.01, the same tolerance Elevator::processNextTask uses to
// decide it "arrived").
// An event dated from a stale route, or a ghost task left by a dispatch
// strategy in an elevator it did not pick, makes events fire while the cabin
// is somewhere else, and processNextTask then consumes the wrong stop.
// Out-of-order event timestamps surface through this invariant too - an event
// that fires early or late finds the cabin away from its floor - so they are
// not asserted separately.
//
// The boarding and alighting records of a stop are produced by its ARRIVE, at
// its time, so a cabin in the wrong place when an ARRIVE fires is a cabin in
// the wrong place for every transfer of that stop. The transfers themselves
// are counted from ElevatorSystem::drainTransfers().
//
// The loop below mirrors Simulator::startSimulation() and
// test_elevator_position_bounds.cpp, observing the cabin through
// ElevatorSystem::getElevators() right after ElevatorSystem::update().

namespace {

struct Observation {
	int enters = 0;
	int leaves = 0;
	int violations = 0;
	double maxError = 0;
	std::vector<double> eventTimes; // every popped event's timestamp, in pop order (determinism check)
};

constexpr double kArrivalTolerance = 0.01;

// Drives `system` until `done(leaves)` or the queue runs dry. `onCustCall` runs
// after each CUST_CALL is processed (random scenarios use it to keep the
// look-ahead window of arrivals full, like Simulator does).
Observation observe(ElevatorSystem& system, const std::function<void()>& onCustCall,
	const std::function<bool(int)>& done) {
	Observation obs;
	Timer clock;
	while (!system.upcomingEvents.empty() && !done(obs.leaves)) {
		Event toProcess = system.upcomingEvents.pop();
		obs.eventTimes.push_back(toProcess.getTime());
		clock.updateTick(toProcess.getTime());
		system.update(clock);

		if (toProcess.getType() == ARRIVE) {
			const Elevator& e = system.getElevators()[toProcess.getElevator() - 1];
			double err = std::fabs(e.getPosition() - toProcess.getFloor());
			if (err > kArrivalTolerance) {
				obs.violations++;
				if (err > obs.maxError) obs.maxError = err;
			}
		}

		// Nothing is handed back to push: a stop is one ARRIVE and every event
		// the engine creates it pushes itself.
		system.giveEvent(toProcess);
		// The transfers of the stop just served, which is where the boardings
		// and alightings live now.
		for (const ElevatorSystem::Transfer& tr : system.drainTransfers()) {
			if (tr.type == ENTER) obs.enters++;
			else if (tr.type == LEAVE) obs.leaves++;
		}
		if (toProcess.getType() == CUST_CALL && onCustCall) onCustCall();
	}
	return obs;
}

// Random workload exactly like Simulator's: Poisson arrivals, constant look-ahead.
// `dwell` is optional: with a stop duration the invariant below has to keep
// holding, and the cabin sits parked through a dwell instead of leaving the
// instant a passenger boards.
Observation runRandom(int floors, int elevators, int customers, double rate, uint64_t seed,
	std::shared_ptr<IDwellTimeStrategy> dwell = nullptr,
	double maxWeight = std::numeric_limits<double>::infinity()) {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed));
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(elevators, floors, /*speed=*/1.0f, dispatch, /*metrics_collector=*/nullptr,
		/*max_weight=*/maxWeight, /*capacity_strategy=*/nullptr, dwell);
	CustomerFactory factory(rng, floors, /*sane_customers=*/true);
	double enq = 0;
	unsigned long int custId = 1, evId = 1;
	auto pushArrival = [&]() {
		enq += rng->nextExponential(rate);
		Customer* c = new Customer(factory.createArrival(custId, enq)); // outlives the run, like Simulator's
		Event arrival(evId, "", CUST_CALL, c->getDir(), c->getAt(), enq, c);
		system.upcomingEvents.push(arrival);
		custId++;
		evId++;
	};
	for (int i = 0; i < 5; i++) pushArrival();
	return observe(system, pushArrival, [customers](int leaves) { return leaves >= customers; });
}

} // namespace

TEST_CASE("A stop's ARRIVE fires with the cabin at its floor - minimal scripted scenario", "[event_timestamps][regression]") {
	// One elevator, no randomness. A stop on the way (customer 1 boarding at 1
	// and heading to 4) while an earlier call to 6 is already queued: the later
	// stops must be re-timed from the new route, or customer 2's stop at floor 6
	// would fire after customer 3's at floor 4, with the cabin already at 6.
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0f, dispatch);

	std::vector<Customer> customers;
	customers.reserve(3); // pointers into this vector are stored in events
	customers.emplace_back(1, "", 1.4373, /*at=*/1, /*to=*/4, UP);
	customers.emplace_back(2, "", 2.2107, /*at=*/6, /*to=*/0, DOWN);
	customers.emplace_back(3, "", 4.7163, /*at=*/4, /*to=*/2, DOWN);
	unsigned long int evId = 1;
	for (Customer& c : customers) {
		Event call(evId++, "", CUST_CALL, c.getDir(), c.getAt(), c.getTime(), &c);
		system.upcomingEvents.push(call);
	}

	Observation obs = observe(system, nullptr, [](int) { return false; });
	INFO("max |position - event floor| = " << obs.maxError);
	REQUIRE(obs.violations == 0);
	REQUIRE(obs.leaves == 3); // everyone is served
	REQUIRE(obs.enters == 3);
}

TEST_CASE("A stop's ARRIVE fires with the cabin at its floor - random workloads", "[event_timestamps][regression]") {
	struct Scenario { int floors, elevators, customers; double rate; uint64_t seed; };
	// First: a small deterministic scenario. Second: saturated and
	// reversal-heavy, the same one test_elevator_position_bounds.cpp uses.
	// The rest vary elevator count, load and seed.
	const Scenario scenarios[] = {
		{10, 3, 25, 1.2, 42},
		{20, 4, 3000, 3.0, 7},
		{8, 1, 60, 1.5, 3},
		{8, 1, 60, 1.5, 5},
		{12, 5, 400, 4.0, 1},
		{12, 5, 400, 4.0, 2},
	};
	for (const Scenario& s : scenarios) {
		INFO("floors=" << s.floors << " elevators=" << s.elevators << " customers=" << s.customers
			<< " rate=" << s.rate << " seed=" << s.seed);
		Observation obs = runRandom(s.floors, s.elevators, s.customers, s.rate, s.seed);
		INFO("violations=" << obs.violations << " maxError=" << obs.maxError);
		REQUIRE(obs.violations == 0);
		// At LEAST the requested number: the loop stops once that many have
		// alighted, and a single ARRIVE can discharge several passengers at
		// once, so it overshoots by whatever the last stop emptied.
		REQUIRE(obs.leaves >= s.customers);
		REQUIRE(obs.enters >= obs.leaves);		// nobody gets out without getting in
	}
}

TEST_CASE("A stop's ARRIVE fires with the cabin at its floor - small cabin, low arrival rate",
	"[event_timestamps][regression][capacity]") {
	// A small cabin at a low arrival rate: a parked cabin is often handed a
	// landing call at its own floor, a zero-length leg. If such a cabin were left
	// with `direction == IDLE` while BUSY, LOOK would sort the stop it is
	// standing at behind the next car call, the stop would never finish, the
	// cabin's position would freeze, and its later ARRIVEs would fire at floors
	// it was not at.
	Observation obs = runRandom(20, 4, 300, 0.4, 7, nullptr, /*maxWeight=*/600.0);
	INFO("violations=" << obs.violations << " maxError=" << obs.maxError);
	REQUIRE(obs.violations == 0);
	REQUIRE(obs.leaves >= 300);
}

TEST_CASE("Event timeline is deterministic for a given seed", "[event_timestamps][determinism]") {
	Observation first = runRandom(20, 4, 500, 3.0, 7);
	Observation second = runRandom(20, 4, 500, 3.0, 7);
	REQUIRE(first.eventTimes == second.eventTimes);
}

TEST_CASE("Elevator::getStopArrivalTime follows the route as it is, for any scheduling policy", "[event_timestamps][elevator]") {
	// A first-come-first-served policy (append to the end), nothing like LOOK:
	// arrival times must come from the task vector's actual order, not from any
	// LOOK-specific assumption (the old Elevator::updateTasks hardcoded one).
	struct FcfsStrategy : IElevatorSchedulingStrategy {
		std::vector<Task>::iterator findInsertionPoint(std::vector<Task>& tasks, unsigned int,
			double, const Task&) const override { return tasks.end(); }
	};
	Elevator elevator(1, "", /*pos=*/0, /*time=*/1.0f, std::make_shared<FcfsStrategy>());

	Task toFive(0, "", GO, DEFAULT, 5, 1.0);
	Task toTwo(0, "", GO, DEFAULT, 2, 2.0);
	Task toNine(0, "", GO, DEFAULT, 9, 3.0);
	unsigned long int s5 = elevator.addTask(toFive, 1).getStopId();
	unsigned long int s2 = elevator.addTask(toTwo, 0).getStopId();
	unsigned long int s9 = elevator.addTask(toNine, 0).getStopId();

	// 1 time unit per floor, starting at floor 0 at t=0: 0->5, 5->2, 2->9.
	REQUIRE(elevator.getStopArrivalTime(s5) == 5.0);
	REQUIRE(elevator.getStopArrivalTime(s2) == 8.0);
	REQUIRE(elevator.getStopArrivalTime(s9) == 15.0);
	REQUIRE(elevator.getStopArrivalTime(s9 + 100) == -1); // not in the route

	// Letting time pass moves the cabin but must not move the ETAs: the same
	// route at the same speed reaches each stop at the same instant.
	Timer clock;
	clock.updateTick(2.0);
	elevator.update(clock);
	REQUIRE(elevator.getPosition() == 2.0f);
	REQUIRE(elevator.getStopArrivalTime(s5) == 5.0);
	REQUIRE(elevator.getStopArrivalTime(s2) == 8.0);
	REQUIRE(elevator.getStopArrivalTime(s9) == 15.0);
}

TEST_CASE("EventQueue::retime rewrites only the given elevator's events and keeps the heap ordered", "[event_timestamps][event_queue]") {
	EventQueue q;
	Event a(1, "", ENTER, UP, 3, 10.0, nullptr, /*elevator=*/1);
	Event b(2, "", LEAVE, DEFAULT, 5, 20.0, nullptr, /*elevator=*/1);
	Event c(3, "", ENTER, UP, 7, 15.0, nullptr, /*elevator=*/2);
	Event d(4, "", CUST_CALL, UP, 1, 12.0, nullptr, /*elevator=*/1);
	q.push(a); q.push(b); q.push(c); q.push(d);

	// Elevator 1: ENTER (id 1) now at 30, LEAVE (id 2) now at 5, CUST_CALL left alone (negative).
	q.retime(1, [](const Event& e) -> double {
		if (e.getId() == 1) return 30.0;
		if (e.getId() == 2) return 5.0;
		return -1.0;
	});

	REQUIRE(q.pop().getId() == 2); // 5.0
	REQUIRE(q.pop().getId() == 4); // 12.0, untouched
	REQUIRE(q.pop().getId() == 3); // 15.0, other elevator untouched
	REQUIRE(q.pop().getId() == 1); // 30.0
	REQUIRE(q.empty());
}

// The invariant needs no change for stop duration, which is worth stating as
// a test rather than as a claim. The event fires at arrival + door-open
// instead of at arrival, and the cabin sits parked for the whole dwell, so it
// is still exactly at the floor when its event fires - checked here against
// routes whose timing the door time reshapes.
TEST_CASE("A stop's ARRIVE fires with the cabin at its floor - with a stop duration", "[event_timestamps][regression][dwell]") {
	struct Scenario { int floors, elevators, customers; double rate; uint64_t seed; };
	const Scenario scenarios[] = {
		{10, 3, 25, 1.2, 42},
		{8, 1, 60, 1.5, 3},
		{12, 5, 300, 4.0, 1},
		{20, 4, 400, 3.0, 7},
	};
	// Deliberately lopsided: a long door-open relative to the travel time of one
	// floor, so a stop costs more than the trip to it and the dwell dominates.
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(/*open=*/2.5, /*close=*/1.5, /*per_passenger=*/0.75);

	for (const Scenario& s : scenarios) {
		INFO("floors=" << s.floors << " elevators=" << s.elevators << " customers=" << s.customers
			<< " rate=" << s.rate << " seed=" << s.seed);
		Observation obs = runRandom(s.floors, s.elevators, s.customers, s.rate, s.seed, dwell);
		INFO("violations=" << obs.violations << " maxError=" << obs.maxError);
		REQUIRE(obs.violations == 0);
		REQUIRE(obs.leaves >= s.customers);	// see the no-dwell case above
		REQUIRE(obs.enters >= obs.leaves);
	}
}

TEST_CASE("Event timeline with a stop duration is deterministic for a given seed", "[event_timestamps][determinism][dwell]") {
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(2.5, 1.5, 0.75);
	Observation first = runRandom(20, 4, 400, 3.0, 7, dwell);
	Observation second = runRandom(20, 4, 400, 3.0, 7, dwell);
	REQUIRE(first.eventTimes == second.eventTimes);

	// And a different dwell must actually change the timeline, otherwise the
	// equality above would also pass for a dwell that is silently ignored.
	auto otherDwell = std::make_shared<FixedDwellTimeStrategy>(1.0, 1.0, 0.1);
	Observation third = runRandom(20, 4, 400, 3.0, 7, otherDwell);
	REQUIRE(third.eventTimes != first.eventTimes);
}

// Elevator::processNextTask must identify the stop that just finished by stop
// identity, not by floor proximity. A route legitimately holds two distinct
// entries for one floor - a cabin stops at floor 5 going up and at 5 again
// coming down - and removing by proximity would, for a merged stop
// immediately followed by another entry for the same floor, remove the
// *neighbour's* entry. Its event would then fire later with the cabin a floor
// away, and remove whatever was near the cabin in turn, cascading through the
// route.
//
// A zero dwell hides it: the two entries are zero travel apart, so their
// events share an instant and the cabin is at the floor for both. Stop
// duration separates them in time, which is why this case configures one.
TEST_CASE("An event only finishes its own stop, even with same-floor duplicates", "[event_timestamps][dwell][regression]") {
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(/*open=*/2.5, /*close=*/1.5, /*per_passenger=*/0.75);
	Observation obs = runRandom(12, 5, 300, 4.0, 1, dwell);
	INFO("violations=" << obs.violations << " maxError=" << obs.maxError);
	REQUIRE(obs.leaves >= 300);		// one ARRIVE can discharge several
	REQUIRE(obs.violations == 0);
}
