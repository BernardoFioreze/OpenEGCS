#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "Elevator.h"
#include "ElevatorSystem.h"
#include "Task.h"
#include "Customer.h"
#include "Event.h"
#include "Simulator.h"
#include "RandomProvider.h"
#include "StandardMetricsCollector.h"
#include "ISimulationLogger.h"
#include "PercentageCapacityStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "GahcaDispatchStrategy.h"
#include "LunchPeakTrafficProfile.h"
#include "FixedDwellTimeStrategy.h"
#include "LinearTravelTimeStrategy.h"

using namespace openEGCS;

// Capacity is a constraint, not advice. The rated load and the load-weighing
// threshold have these consequences:
//
//   1. no cabin can carry more than its rated load, ever;
//   2. a cabin that has filled up stops answering landing calls - it gives the
//      pickups it has not made back to the dispatcher (Barney & Al-Sharif
//      10.8.3, "load bypass") instead of making stops where nobody can board;
//   3. a landing call is never cancelled by any of that ("landing calls should
//      not be cancelled", same section), so no passenger is ever lost.
//
// A passenger declining a car that is merely crowded (Peters'
// m_LoadingThreshold, section 8.3.6) is covered in test_passenger_behavior.cpp.

namespace {

// Discards the snapshot instead of writing ./snapshot.json, so these cases do
// not compete for that file with the others in the suite.
struct NullLogger : public ISimulationLogger {
	std::vector<std::string> eventTypes;
	std::vector<unsigned long int> eventCustomers;

	void setMetadata(const SimulationMetadata&) override {}
	void logCustomer(const Customer&) override {}
	void logEvent(double, const std::string& type, int, unsigned long int customerId, int) override {
		eventTypes.push_back(type);
		eventCustomers.push_back(customerId);
	}
	void finalize(const IMetricsCollector&, const std::string&) override {}
};

struct Run {
	std::shared_ptr<StandardMetricsCollector> metrics;
	std::shared_ptr<NullLogger> logger;
};

Run runWithCapacity(double maxWeight, int floors, int elevators, int customers,
	double arrivalRate, uint64_t seed, double meanWeight = 70.0, double stddevWeight = 15.0) {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto logger = std::make_shared<NullLogger>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();

	Simulator sim(1, "capacity", floors, elevators, customers, arrivalRate,
		/*speed=*/1.0, /*sane=*/true, rng, dispatch, metrics, logger,
		/*seed_for_metadata=*/std::nullopt, maxWeight, /*capacity_strategy=*/nullptr,
		meanWeight, stddevWeight);
	sim.startSimulation();
	return Run{metrics, logger};
}

} // namespace

// ---------------------------------------------------------------------------
// The hard limit
// ---------------------------------------------------------------------------

TEST_CASE("A cabin refuses to be loaded past its rated weight", "[capacity][elevator]") {
	Elevator e(1, "", 0, 1, nullptr, /*max_weight=*/100.0);

	Customer first(1, "", /*t=*/0, /*at=*/0, /*to=*/1, UP, /*weight=*/60);
	REQUIRE(e.canBoard(60.0));
	e.addCustomer(first);

	// 60 + 50 > 100: a boarding that would overload the cabin. Without this
	// check, saturated scenarios board through it routinely and leave cabins at
	// several times their rated load.
	Customer second(2, "", /*t=*/1, /*at=*/0, /*to=*/1, UP, /*weight=*/50);
	REQUIRE_FALSE(e.canBoard(50.0));
	REQUIRE_THROWS_AS(e.addCustomer(second), std::logic_error);
	REQUIRE(e.getCurrentLoad() == Catch::Approx(60.0));

	// Exactly at the limit fits: the rated load is a maximum, not an exclusive
	// bound.
	Customer exact(3, "", /*t=*/2, /*at=*/0, /*to=*/1, UP, /*weight=*/40);
	REQUIRE(e.canBoard(40.0));
	e.addCustomer(exact);
	REQUIRE(e.getCurrentLoad() == Catch::Approx(100.0));
}

TEST_CASE("An unlimited cabin has room for anybody", "[capacity][elevator]") {
	Elevator e(1, "", 0, 1);	// default max_weight: infinity
	REQUIRE(e.canBoard(1e9));
	REQUIRE(e.canAnswerHallCall(1e9));
	for (int i = 0; i < 20; i++) {
		Customer c(i + 1, "", /*t=*/i, /*at=*/0, /*to=*/1, UP, /*weight=*/1000.0);
		e.addCustomer(c);
	}
	REQUIRE(e.getCurrentLoad() == Catch::Approx(20000.0));
	REQUIRE_FALSE(e.isBypassingCalls());
}

TEST_CASE("A bypassing cabin may not be given a hall call, even with room to spare", "[capacity][elevator]") {
	Elevator e(1, "", 0, 1, nullptr, /*max_weight=*/100.0);	// default policy: 60% -> 60
	Customer aboard(1, "", /*t=*/0, /*at=*/0, /*to=*/1, UP, /*weight=*/65);
	e.addCustomer(aboard);

	REQUIRE(e.isBypassingCalls());		// 65 >= 60
	REQUIRE(e.canBoard(30.0));			// and 65 + 30 <= 100: physically it fits
	// ...but the load-weighing device has already declared the car full as far
	// as landing calls go (Barney & Al-Sharif 10.8.3), and that is equipment
	// rather than policy, so the engine will not allocate one.
	REQUIRE_FALSE(e.canAnswerHallCall(30.0));
	// A passenger who cannot fit is refused on both counts.
	REQUIRE_FALSE(e.canAnswerHallCall(40.0));
}

// ---------------------------------------------------------------------------
// Giving a call back: the route bookkeeping
// ---------------------------------------------------------------------------

TEST_CASE("Releasing one hall call of a merged stop leaves the other one standing", "[capacity][task]") {
	// Two people waiting to go up at floor 5, merged into one stop by
	// Elevator::addTask. Releasing one of them must not erase the fact that
	// somebody is still going up from there, which a pair of booleans could not
	// express - see Task::releaseHallCall.
	Task stop(0, "", CALL, UP, 5, 0);
	stop.initPassengerCounters();
	Task second(0, "", CALL, UP, 5, 1);
	stop.addPassengerOf(second);
	REQUIRE(stop.getBoardingPassengers() == 2);
	REQUIRE(stop.hasHallCall(UP));

	stop.releaseHallCall(UP);
	REQUIRE(stop.getBoardingPassengers() == 1);
	REQUIRE(stop.hasHallCall(UP));
	REQUIRE(stop.servesAnyone());

	stop.releaseHallCall(UP);
	REQUIRE(stop.getBoardingPassengers() == 0);
	REQUIRE_FALSE(stop.hasHallCall(UP));
	REQUIRE_FALSE(stop.servesAnyone());

	// Releasing a direction this stop never served changes nothing.
	Task other(0, "", CALL, DOWN, 7, 0);
	other.initPassengerCounters();
	other.releaseHallCall(UP);
	REQUIRE(other.getBoardingPassengers() == 1);
	REQUIRE(other.hasHallCall(DOWN));
}

TEST_CASE("A released stop leaves the route, unless the cabin is committed to it", "[capacity][elevator]") {
	Elevator e(1, "", 0, 1, nullptr, /*max_weight=*/1000.0);
	Task near(0, "", CALL, UP, 2, 0);
	Task far(0, "", CALL, UP, 8, 0);
	unsigned long int nearStop = e.addTask(near, 1).getStopId();
	unsigned long int farStop = e.addTask(far, 1).getStopId();

	REQUIRE(e.getRoute().size() == 2);
	REQUIRE(e.committedStopId() == nearStop);

	// The far one can still be taken away: the cabin has not started braking
	// for it.
	REQUIRE(e.releaseHallCall(farStop, UP));
	REQUIRE(e.getRoute().size() == 1);
	REQUIRE(e.getRoute().front().getStopId() == nearStop);

	// The committed one cannot - a cabin in flight has no way to un-brake -
	// but its passenger count drops all the same, so nobody is charged a
	// transfer they did not make.
	REQUIRE(e.releaseHallCall(nearStop, UP));
	REQUIRE(e.getRoute().size() == 1);
	REQUIRE(e.getRoute().front().getBoardingPassengers() == 0);
	REQUIRE(e.committedStopId() == nearStop);

	// A stop that is not there, and a direction that was never requested, are
	// both no-ops rather than errors.
	REQUIRE_FALSE(e.releaseHallCall(farStop, UP));
	REQUIRE_FALSE(e.releaseHallCall(nearStop, DOWN));
}

// ---------------------------------------------------------------------------
// End to end
// ---------------------------------------------------------------------------

TEST_CASE("A whole run with a real capacity overloads nothing and loses nobody", "[capacity][simulation]") {
	// The capacity rule as a whole, end to end: the engine cannot put anybody
	// into a full cabin, so a regression shows up as a std::logic_error out of
	// startSimulation rather than as a number nobody can eyeball.
	Run run = runWithCapacity(/*maxWeight=*/300.0, /*floors=*/10, /*elevators=*/3,
		/*customers=*/50, /*arrivalRate=*/0.8, /*seed=*/42);

	// Item #20's invariant, restated for a run where calls move between cabins:
	// everybody boards and everybody arrives. This is the one that would catch
	// a call taken out of the timeline and never put back.
	REQUIRE(run.metrics->getWaitingTimeDistribution().count == 50);
	REQUIRE(run.metrics->getJourneyTimeDistribution().count == 50);

	std::optional<PassByReport> report = run.metrics->getPassByReport();
	REQUIRE(report.has_value());
	// This scenario really does exercise both paths (a few refusals at a door
	// and many calls handed back). Asserting "more than zero" rather than the
	// exact figures: the counts move with any legitimate change to route timing,
	// while "it happened at all" is what this case is for.
	REQUIRE(report->refusedAtDoor.events > 0);
	REQUIRE(report->reallocated.events > 0);
	// Occurrences are at least people, since one passenger can be passed more
	// than once and every occurrence belongs to somebody.
	REQUIRE(report->refusedAtDoor.events >= (int)report->refusedAtDoor.passengers.size());
	REQUIRE(report->reallocated.events >= (int)report->reallocated.passengers.size());

	// Each PASS line in the timeline belongs to a passenger who also boards
	// later: being passed is a delay, not an exit.
	int passLines = 0;
	for (std::size_t i = 0; i < run.logger->eventTypes.size(); i++) {
		if (run.logger->eventTypes[i] == "PASS") passLines++;
	}
	REQUIRE(passLines == report->refusedAtDoor.events);
}

TEST_CASE("A call nobody can answer stays registered and unallocated", "[capacity][elevator_system]") {
	// A landing call that is registered and belongs to no cabin. It is what
	// "landing calls should not be cancelled" (Barney & Al-Sharif 10.8.3) needs
	// somewhere to live, and what lets a tool see demand the lifts have not
	// taken.
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto capacity = std::make_shared<PercentageCapacityStrategy>(0.6);
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0,
		dispatch, /*metrics_collector=*/nullptr, /*max_weight=*/100.0, capacity);

	REQUIRE(system.getUnallocatedCallCount() == 0);

	// 70 kg into an empty 100 kg cabin: allocated normally, and the cabin is
	// bypassing from here on (70 >= 60).
	Customer first(1, "", /*t=*/0.0, /*at=*/5, /*to=*/9, UP, /*weight=*/70.0);
	Event firstCall(1, "", CUST_CALL, first.getDir(), first.getAt(), first.getTime(), &first);
	// A call the engine could allocate puts a stop in a cabin's route and an
	// ARRIVE in the queue; a call it could not allocate creates no event at all,
	// which is the distinction this case is about.
	system.giveEvent(firstCall);
	REQUIRE(system.upcomingEvents.size() == 1);
	Event arrival = system.upcomingEvents.pop();
	REQUIRE(arrival.getType() == ARRIVE);
	REQUIRE(system.getUnallocatedCallCount() == 0);
	REQUIRE_FALSE(system.wouldRefuseBoarding(1, first, first.getAt(), arrival.getTime()));
	system.giveEvent(arrival);
	REQUIRE(system.drainTransfers().size() == 1);		// they boarded
	REQUIRE(system.getElevators().front().isBypassingCalls());

	// Now the only cabin is bypassing, so this call has nowhere to go. It is
	// not lost and not refused - it waits, which is the whole point.
	Customer second(2, "", /*t=*/1.0, /*at=*/3, /*to=*/8, UP, /*weight=*/20.0);
	Event secondCall(2, "", CUST_CALL, second.getDir(), second.getAt(), second.getTime(), &second);
	int pendingBefore = system.upcomingEvents.size();
	system.giveEvent(secondCall);
	// No stop, so no arrival event: nothing was added to the queue for them.
	REQUIRE(system.upcomingEvents.size() == pendingBefore);
	REQUIRE(system.getUnallocatedCallCount() == 1);
}

TEST_CASE("Capacity does not cost determinism", "[capacity][simulation][determinism]") {
	// Capacity adds a pool of waiting calls that is iterated and re-offered to
	// the dispatcher, which is exactly the kind of thing that leaks a hash
	// order or a pointer order into the results. Same seed, same everything.
	Run first = runWithCapacity(300.0, 10, 3, 50, 0.8, /*seed=*/42);
	Run second = runWithCapacity(300.0, 10, 3, 50, 0.8, /*seed=*/42);

	REQUIRE(first.metrics->getAverageWaitingTime() == second.metrics->getAverageWaitingTime());
	REQUIRE(first.metrics->getTotalEnergyConsumption() == second.metrics->getTotalEnergyConsumption());
	REQUIRE(first.metrics->getPassByReport()->refusedAtDoor.events
		== second.metrics->getPassByReport()->refusedAtDoor.events);
	REQUIRE(first.metrics->getPassByReport()->reallocated.events
		== second.metrics->getPassByReport()->reallocated.events);
	REQUIRE(first.metrics->getPassByReport()->reallocated.passengers
		== second.metrics->getPassByReport()->reallocated.passengers);
	REQUIRE(first.logger->eventTypes == second.logger->eventTypes);
	REQUIRE(first.logger->eventCustomers == second.logger->eventCustomers);

	// A different seed has to produce a different run, or the comparison above
	// would pass for a simulator that ignores its input.
	Run other = runWithCapacity(300.0, 10, 3, 50, 0.8, /*seed=*/43);
	REQUIRE(other.metrics->getAverageWaitingTime() != first.metrics->getAverageWaitingTime());
}

TEST_CASE("A passenger heavier than the cabin is reported, not waited on forever", "[capacity][simulation]") {
	// The liveness guard. A call nobody can answer is retried whenever a cabin
	// unloads, which terminates because every boarded passenger alights - but
	// a passenger who does not fit in an EMPTY cabin can never be served, and
	// that is a scenario that cannot be run rather than a result. Without the
	// guard the event loop would drain and blame "a customer was dropped",
	// which points at the wrong thing entirely.
	REQUIRE_THROWS_AS(
		runWithCapacity(/*maxWeight=*/10.0, /*floors=*/5, /*elevators=*/1,
			/*customers=*/3, /*arrivalRate=*/0.5, /*seed=*/1,
			/*meanWeight=*/70.0, /*stddevWeight=*/1.0),
		std::logic_error);
}

TEST_CASE("A cabin that empties weighs exactly nothing", "[capacity][elevator]") {
	// The load is a running double sum, and adding two weights and taking them
	// off again does not return to zero: 70.1 + 80.2 - 70.1 - 80.2 is 1.42e-14 in
	// IEEE 754, and the max(0, ...) in the alighting path only stops it going
	// negative. An emptied cabin must report exactly zero.
	//
	// Passengers are handed over as temporaries: addCustomer takes them by
	// value, and copying a named Customer triggers -Wdeprecated-copy.
	Elevator e(1, "", 0, 1, nullptr, /*max_weight=*/200.0);
	e.addCustomer(Customer(1, "", /*t=*/0, /*at=*/0, /*to=*/3, UP, /*weight=*/70.1));
	e.addCustomer(Customer(2, "", /*t=*/1, /*at=*/0, /*to=*/3, UP, /*weight=*/80.2));
	REQUIRE(e.getPassengerCount() == 2);

	REQUIRE(e.dischargeAt(3).size() == 2);
	REQUIRE(e.getPassengerCount() == 0);
	// Exact, not Approx: the property is that an empty cabin is
	// indistinguishable from one nobody ever boarded.
	REQUIRE(e.getCurrentLoad() == 0.0);

	// removeCustomer is the other way off, and it has the same arithmetic. It
	// matches by value (Customer::operator==), hence the distinct arrival times.
	Customer a(1, "", /*t=*/0, /*at=*/0, /*to=*/3, UP, /*weight=*/70.1);
	Customer b(2, "", /*t=*/1, /*at=*/0, /*to=*/3, UP, /*weight=*/80.2);
	e.addCustomer(Customer(1, "", /*t=*/0, /*at=*/0, /*to=*/3, UP, /*weight=*/70.1));
	e.addCustomer(Customer(2, "", /*t=*/1, /*at=*/0, /*to=*/3, UP, /*weight=*/80.2));
	REQUIRE(e.getPassengerCount() == 2);
	e.removeCustomer(&a);
	e.removeCustomer(&b);
	REQUIRE(e.getPassengerCount() == 0);
	REQUIRE(e.getCurrentLoad() == 0.0);
}

TEST_CASE("The liveness guard is not fooled by an empty cabin's rounding residue", "[capacity][elevator_system]") {
	// The guard in ElevatorSystem::deferCall asks "is anybody on board
	// anywhere?". Asked as `load > 0`, a cabin everybody had left, holding
	// 1.42e-14 kg of rounding, would count as carrying somebody, and a call that
	// can never be answered would be parked in the pool instead of being
	// reported. Asked by head count, it throws.
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto capacity = std::make_shared<PercentageCapacityStrategy>(0.6);
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0,
		dispatch, /*metrics_collector=*/nullptr, /*max_weight=*/200.0, capacity);

	// Two people behind one landing call, the pair of weights the case above
	// shows does not cancel. Both fit (150.3 <= 200) and ride to floor 8.
	Customer a(1, "", /*t=*/0.0, /*at=*/2, /*to=*/8, UP, /*weight=*/70.1);
	Customer b(2, "", /*t=*/0.5, /*at=*/2, /*to=*/8, UP, /*weight=*/80.2);
	Event callA(1, "", CUST_CALL, a.getDir(), a.getAt(), a.getTime(), &a);
	Event callB(2, "", CUST_CALL, b.getDir(), b.getAt(), b.getTime(), &b);
	system.giveEvent(callA);
	system.giveEvent(callB);

	int boarded = 0, alighted = 0;
	while (system.upcomingEvents.size() > 0) {
		Event next = system.upcomingEvents.pop();
		system.giveEvent(next);
		for (const ElevatorSystem::Transfer& t : system.drainTransfers()) {
			if (t.type == ENTER) boarded++;
			if (t.type == LEAVE) alighted++;
		}
	}
	REQUIRE(boarded == 2);
	REQUIRE(alighted == 2);
	REQUIRE(system.getElevators().front().getPassengerCount() == 0);

	// Nobody is aboard anywhere, and this passenger does not fit in the empty
	// cabin: no future state differs from this one, so the call must be
	// reported, not waited on.
	Customer heavy(3, "", /*t=*/100.0, /*at=*/4, /*to=*/1, DOWN, /*weight=*/250.0);
	Event callHeavy(3, "", CUST_CALL, heavy.getDir(), heavy.getAt(), heavy.getTime(), &heavy);
	REQUIRE_THROWS_AS(system.giveEvent(callHeavy), std::logic_error);
}

TEST_CASE("An unlimited run never reaches any of the capacity machinery", "[capacity][simulation]") {
	Run run = runWithCapacity(std::numeric_limits<double>::infinity(), 10, 3, 50, 0.8, /*seed=*/42);

	std::optional<PassByReport> report = run.metrics->getPassByReport();
	REQUIRE(report.has_value());
	REQUIRE(report->refusedAtDoor.events == 0);
	REQUIRE(report->reallocated.events == 0);
	for (const std::string& type : run.logger->eventTypes) {
		REQUIRE(type != "PASS");
	}
}

// A group strategy that keeps naming a cabin the capacity veto refuses (a
// metaheuristic can) must not leave the call pooled while an idle, empty
// cabin stands at the floor - the queue would drain and the Simulator would
// throw "event queue ran dry". The engine's liveness floor in allocateCall
// redirects the call instead of pooling it.
//
// This case drives that path: GAHCA under a saturated lunch-peak with finite
// 20-person cabins and a 13 s/stop cabin (so capacity actually binds). It is
// a termination + conservation test, needing no expected value: the run must
// finish and every created customer must have alighted.
TEST_CASE("A saturated GAHCA lunch-peak with finite cabins serves everyone", "[capacity][gahca][e2e]") {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(1));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto gahca = std::make_shared<examples::GahcaDispatchStrategy>();
	auto profile = std::make_shared<examples::LunchPeakTrafficProfile>(12, /*scale=*/5.0);
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(6.5, 6.5, 0.0);	// ~13 s/stop
	auto travel = std::make_shared<LinearTravelTimeStrategy>(5.0);

	const int customers = 60;
	Simulator sim(1, "item46", /*floors=*/12, /*elevators=*/2, customers,
		/*arrivalRate=*/0.5, /*speed=*/1.0f, /*sane=*/true,
		rng, dispatch, metrics, /*logger=*/nullptr, /*seed_for_metadata=*/std::nullopt,
		/*maxWeight=*/20.0, /*capacity=*/nullptr, /*mean_weight=*/1.0, /*stddev_weight=*/0.0,
		dwell, travel, /*passenger=*/nullptr, gahca, profile);

	REQUIRE_NOTHROW(sim.startSimulation());
	// Everyone created reached their destination: the journey distribution
	// counts one entry per alighting.
	REQUIRE(metrics->getJourneyTimeDistribution().count == customers);
}
