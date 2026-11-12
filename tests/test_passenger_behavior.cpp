#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "ElevatorSystem.h"
#include "Simulator.h"
#include "Customer.h"
#include "Event.h"
#include "RandomProvider.h"
#include "StandardMetricsCollector.h"
#include "ISimulationLogger.h"
#include "IPassengerBehaviorStrategy.h"
#include "StandardPassengerBehavior.h"
#include "PercentageCapacityStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "LoadingThresholdPassengerBehavior.h"

using namespace openEGCS;

// The decisions that belong to the passenger rather than to the installation
// (IPassengerBehaviorStrategy). The shipped StandardPassengerBehavior boards
// every car it fits into, so with it most of what is worth asserting is an
// absence, plus proof that the hook is really load bearing and not
// decoration. Then a policy that declines (examples::
// LoadingThresholdPassengerBehavior): event by event first, and finally
// through a whole simulation, which must terminate.

namespace {

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

// Says no to everybody, always. Pure: the answer does not depend on how often
// it has been asked, which is the contract willBoard imposes.
struct NeverBoards : public IPassengerBehaviorStrategy {
	bool willBoard(const Customer&, const BoardingContext&) const override { return false; }
	double nextCallTime(const Customer&, const RefusalContext& r) const override {
		return r.cabinDeparture;
	}
};

// Says yes to everybody and records what it was shown. Counting is explicitly
// allowed by the contract; varying the answer is not.
struct RecordingBehavior : public IPassengerBehaviorStrategy {
	mutable int calls = 0;
	mutable double lastLoad = -1;
	mutable double lastMaxWeight = -1;
	mutable int lastFloor = -1;
	mutable unsigned int lastDirection = 0;

	bool willBoard(const Customer&, const BoardingContext& cabin) const override {
		calls++;
		lastLoad = cabin.currentLoad;
		lastMaxWeight = cabin.maxWeight;
		lastFloor = cabin.floor;
		lastDirection = cabin.cabinDirection;
		return true;
	}
	double nextCallTime(const Customer&, const RefusalContext& r) const override {
		return r.cabinDeparture;
	}
};

} // namespace

TEST_CASE("The shipped passenger behaviour boards every car, however full", "[passenger_behavior]") {
	// The default has no opinion at all - not "an opinion that happens to be
	// permissive for these inputs". Peters' 80% loading threshold (section
	// 8.3.6) is cited in the interface's own comment and deliberately NOT
	// shipped, because a non-trivial default would move every comparison
	// recorded before this interface existed.
	StandardPassengerBehavior behavior;
	Customer heavy(1, "", /*t=*/0.0, /*at=*/3, /*to=*/9, UP, /*weight=*/120.0);

	BoardingContext empty;
	empty.currentLoad = 0.0;
	empty.maxWeight = 600.0;
	REQUIRE(behavior.willBoard(heavy, empty));

	BoardingContext crammed;
	crammed.currentLoad = 599.0;
	crammed.maxWeight = 600.0;
	REQUIRE(behavior.willBoard(heavy, crammed));

	BoardingContext unlimited;
	unlimited.currentLoad = 10000.0;
	unlimited.maxWeight = std::numeric_limits<double>::infinity();
	REQUIRE(behavior.willBoard(heavy, unlimited));
}

TEST_CASE("A passenger who declines is not boarded, and is not lost either", "[passenger_behavior][elevator_system]") {
	// The hook is load bearing: with a policy that says no, a boarding the
	// cabin had room for does not happen. The same machinery as "the cabin is
	// full" (a boarding that can fail, a PASS in the timeline, a re-call)
	// reached through the other door.
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto behavior = std::make_shared<NeverBoards>();
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0,
		dispatch, metrics, /*max_weight=*/std::numeric_limits<double>::infinity(),
		/*capacity_strategy=*/nullptr, /*dwell_strategy=*/nullptr,
		/*travel_strategy=*/nullptr, behavior);

	Customer cus(1, "", /*t=*/0.0, /*at=*/5, /*to=*/9, UP, /*weight=*/70.0);
	Event call(1, "", CUST_CALL, cus.getDir(), cus.getAt(), cus.getTime(), &cus);
	system.giveEvent(call);
	Event arrival = system.upcomingEvents.pop();

	// The call was allocated normally: nothing about a passenger's preference
	// reaches the dispatcher, and it should not - the car comes, and only then
	// does the person decide.
	REQUIRE(arrival.getType() == ARRIVE);
	REQUIRE(system.getUnallocatedCallCount() == 0);

	// The cabin has infinite capacity, so the ONLY possible reason for this to
	// be true is the injected policy.
	REQUIRE(system.wouldRefuseBoarding(1, cus, cus.getAt(), arrival.getTime()));

	system.giveEvent(arrival);
	// One transfer record, and it is a refusal rather than a boarding: the
	// cabin came, opened its doors and left without them.
	std::vector<ElevatorSystem::Transfer> transfers = system.drainTransfers();
	REQUIRE(transfers.size() == 1);
	REQUIRE(transfers[0].type == PASS);
	REQUIRE(transfers[0].customerId == cus.getId());
	REQUIRE(system.getElevators().front().getCurrentLoad() == 0.0);

	std::optional<PassByReport> report = metrics->getPassByReport();
	REQUIRE(report.has_value());
	REQUIRE(report->refusedAtDoor.events == 1);
	REQUIRE(report->refusedAtDoor.passengers.size() == 1);

	// Not lost, and specifically not lost in the dispatcher's pool: the
	// passenger presses the button again, at an instant of their own.
	REQUIRE(system.getUnallocatedCallCount() == 0);
	REQUIRE_FALSE(system.upcomingEvents.empty());
	Event again = system.upcomingEvents.pop();
	REQUIRE(again.getType() == RECALL);
	REQUIRE(again.getCustomer() == &cus);
	REQUIRE(again.getFloor() == 5);
	// The cabin that refused them rides on the event, because it is the one
	// cabin the new call may not be handed back to.
	REQUIRE(again.getElevator() == 1);
	REQUIRE(again.getTime() >= arrival.getTime());
}

TEST_CASE("The passenger is only asked once they fit, and is shown the cabin as it is", "[passenger_behavior][elevator_system]") {
	// Two halves of one contract. The interface promises an implementation
	// that it is asked only after the rated load has been checked, so a policy
	// never has to re-check capacity; and the context it is handed is the load
	// the cabin carries BEFORE this passenger, which is what makes Peters'
	// "the lift will then be >80% full" computable at all.
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto behavior = std::make_shared<RecordingBehavior>();
	auto capacity = std::make_shared<PercentageCapacityStrategy>(0.6);
	ElevatorSystem system(/*no_elevators=*/1, /*no_floors=*/10, /*speed=*/1.0,
		dispatch, metrics, /*max_weight=*/100.0, capacity,
		/*dwell_strategy=*/nullptr, /*travel_strategy=*/nullptr, behavior);

	Customer first(1, "", /*t=*/0.0, /*at=*/5, /*to=*/9, UP, /*weight=*/70.0);
	Event firstCall(1, "", CUST_CALL, first.getDir(), first.getAt(), first.getTime(), &first);
	system.giveEvent(firstCall);
	Event arrival = system.upcomingEvents.pop();
	REQUIRE(arrival.getType() == ARRIVE);
	REQUIRE_FALSE(system.wouldRefuseBoarding(1, first, first.getAt(), arrival.getTime()));
	REQUIRE(behavior->calls == 1);
	REQUIRE(behavior->lastLoad == 0.0);			// before this passenger, not after
	REQUIRE(behavior->lastMaxWeight == 100.0);
	REQUIRE(behavior->lastFloor == 5);
	REQUIRE(behavior->lastDirection == UP);
	system.giveEvent(arrival);
	REQUIRE(system.getElevators().front().getCurrentLoad() == 70.0);

	// 40 kg into a 100 kg cabin already carrying 70: physically impossible, so
	// the refusal is decided before the passenger is consulted. The call
	// counter not moving is the whole assertion.
	int callsBefore = behavior->calls;
	Customer tooHeavy(2, "", /*t=*/1.0, /*at=*/5, /*to=*/9, UP, /*weight=*/40.0);
	REQUIRE(system.wouldRefuseBoarding(1, tooHeavy, /*floor=*/5, /*time=*/1.0));
	REQUIRE(behavior->calls == callsBefore);
}

TEST_CASE("The loading-threshold policy refuses exactly when the car would be too full", "[passenger_behavior]") {
	// The loading threshold: Peters 8.3.6, "will not load lift if the lift will
	// then be >80% full". The passenger reads currentLoad (the load BEFORE them)
	// and maxWeight; the engine has already checked they fit.
	examples::LoadingThresholdPassengerBehavior policy(0.80);
	Customer person(1, "", /*t=*/0.0, /*at=*/3, /*to=*/9, UP, /*weight=*/70.0);

	BoardingContext justUnder;			// (410 + 70) / 600 = 0.80 -> boards (<=)
	justUnder.currentLoad = 410.0;
	justUnder.maxWeight = 600.0;
	REQUIRE(policy.willBoard(person, justUnder));

	BoardingContext justOver;			// (420 + 70) / 600 = 0.8166... -> refuses
	justOver.currentLoad = 420.0;
	justOver.maxWeight = 600.0;
	REQUIRE_FALSE(policy.willBoard(person, justOver));

	// Purity: the answer is a function of (person, cabin) alone, never of how
	// often it has been asked - the contract willBoard imposes, and the thing a
	// policy that said "no then yes" would break.
	REQUIRE_FALSE(policy.willBoard(person, justOver));
	REQUIRE_FALSE(policy.willBoard(person, justOver));

	// An unlimited cabin makes the ratio zero, so a run that configured no
	// capacity never sees this policy refuse - the correct no-op, not a special
	// case.
	BoardingContext unlimited;
	unlimited.currentLoad = 1e6;
	unlimited.maxWeight = std::numeric_limits<double>::infinity();
	REQUIRE(policy.willBoard(person, unlimited));

	// A threshold outside (0, 1] is a configuration error, not a run with an
	// extreme policy: 0 would refuse every car and the run could never end.
	REQUIRE_THROWS_AS(examples::LoadingThresholdPassengerBehavior(0.0), std::invalid_argument);
	REQUIRE_THROWS_AS(examples::LoadingThresholdPassengerBehavior(1.5), std::invalid_argument);
	REQUIRE_NOTHROW(examples::LoadingThresholdPassengerBehavior(1.0));
}

TEST_CASE("A declining policy drives a saturated run to completion without livelock", "[passenger_behavior][simulation]") {
	// A passenger who DECLINES a car (not "does not fit") keeps their call
	// registered, and nothing must hand that call straight back to the cabin
	// they just refused at the same instant, or the allocate/refuse/allocate
	// cycle never advances the clock. The whole test is that startSimulation()
	// RETURNS - a regression hangs here rather than failing - and that every
	// created customer is nonetheless served.
	//
	// Small on purpose. The failure this guards against is a livelock, which a
	// handful of refusals reaches as surely as a crowd does; scaling the run up
	// only multiplies the refuse/recall cascade a declining policy produces in
	// saturation, which is pure cost with nothing more proved.
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(7));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto logger = std::make_shared<NullLogger>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto behavior = std::make_shared<examples::LoadingThresholdPassengerBehavior>(0.80);
	const int customers = 40;
	Simulator sim(1, "threshold", /*floors=*/8, /*elevators=*/2, /*customers=*/customers,
		/*arrivalRate=*/0.8, /*speed=*/1.0, /*sane=*/true, rng, dispatch, metrics, logger,
		/*seed_for_metadata=*/std::nullopt, /*max_weight=*/300.0,
		/*capacity_strategy=*/nullptr, /*mean=*/70.0, /*stddev=*/15.0,
		/*dwell_strategy=*/nullptr, /*travel_strategy=*/nullptr, behavior);
	sim.startSimulation();

	// Everybody created reached their destination - the call that bounced off a
	// too-full cabin was not dropped, only deferred until a cabin came that the
	// passenger accepted.
	REQUIRE(metrics->getJourneyTimeDistribution().count == customers);

	// And the policy actually bit: at a 300 kg cabin with 70 kg passengers, a
	// third boarder would push it past 80% (210 + 70 = 280 > 240), so a saturated
	// run must record refusals at the door. Without this the test would pass
	// vacuously on a run where nobody ever declined.
	std::optional<PassByReport> report = metrics->getPassByReport();
	REQUIRE(report.has_value());
	REQUIRE(report->refusedAtDoor.events > 0);
}

TEST_CASE("Injecting the default behaviour explicitly changes nothing", "[passenger_behavior][simulation]") {
	// The byte-identity gate of this phase, stated as a test so it does not
	// depend on anyone remembering to diff a snapshot: the same scenario with
	// the policy left to the default and with StandardPassengerBehavior handed
	// in by name must produce the same timeline, event for event, and the same
	// aggregates to the bit.
	auto run = [](std::shared_ptr<IPassengerBehaviorStrategy> behavior) {
		auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(42));
		auto metrics = std::make_shared<StandardMetricsCollector>();
		auto logger = std::make_shared<NullLogger>();
		auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
		Simulator sim(1, "behavior", /*floors=*/10, /*elevators=*/3, /*customers=*/50,
			/*arrivalRate=*/0.8, /*speed=*/1.0, /*sane=*/true, rng, dispatch, metrics, logger,
			/*seed_for_metadata=*/std::nullopt, /*max_weight=*/300.0,
			/*capacity_strategy=*/nullptr, /*mean=*/70.0, /*stddev=*/15.0,
			/*dwell_strategy=*/nullptr, /*travel_strategy=*/nullptr, behavior);
		sim.startSimulation();
		return std::make_pair(metrics, logger);
	};

	auto implicit = run(nullptr);
	auto explicitDefault = run(std::make_shared<StandardPassengerBehavior>());

	REQUIRE(implicit.second->eventTypes == explicitDefault.second->eventTypes);
	REQUIRE(implicit.second->eventCustomers == explicitDefault.second->eventCustomers);
	REQUIRE(implicit.first->getAverageWaitingTime() == explicitDefault.first->getAverageWaitingTime());
	REQUIRE(implicit.first->getAverageTravelTime() == explicitDefault.first->getAverageTravelTime());
	REQUIRE(implicit.first->getTotalEnergyConsumption() == explicitDefault.first->getTotalEnergyConsumption());
	// A capacity was configured on purpose, so the run really does exercise
	// the refusal path this interface shares with a full cabin.
	REQUIRE(implicit.first->getPassByReport()->refusedAtDoor.events
		== explicitDefault.first->getPassByReport()->refusedAtDoor.events);
}

TEST_CASE("The loading-threshold policy is a no-op where no capacity is configured", "[passenger_behavior][simulation]") {
	// A policy whose only opinion is "will the car be too full" can have no
	// opinion at all when the car is unlimited: (load + weight) / infinity is
	// zero, below any threshold. So an uncapacitated run with the threshold
	// policy injected must be byte-identical to the default - the same proof the
	// case above makes for StandardPassengerBehavior, so that "it moves results"
	// is a property of a configured CAPACITY and never of merely injecting the
	// policy.
	auto run = [](std::shared_ptr<IPassengerBehaviorStrategy> behavior) {
		auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(42));
		auto metrics = std::make_shared<StandardMetricsCollector>();
		auto logger = std::make_shared<NullLogger>();
		auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
		Simulator sim(1, "behavior", /*floors=*/10, /*elevators=*/3, /*customers=*/50,
			/*arrivalRate=*/0.8, /*speed=*/1.0, /*sane=*/true, rng, dispatch, metrics, logger,
			/*seed_for_metadata=*/std::nullopt,
			/*max_weight=*/std::numeric_limits<double>::infinity(),
			/*capacity_strategy=*/nullptr, /*mean=*/70.0, /*stddev=*/15.0,
			/*dwell_strategy=*/nullptr, /*travel_strategy=*/nullptr, behavior);
		sim.startSimulation();
		return std::make_pair(metrics, logger);
	};

	auto baseline = run(nullptr);
	auto withPolicy = run(std::make_shared<examples::LoadingThresholdPassengerBehavior>(0.80));

	REQUIRE(baseline.second->eventTypes == withPolicy.second->eventTypes);
	REQUIRE(baseline.second->eventCustomers == withPolicy.second->eventCustomers);
	REQUIRE(baseline.first->getAverageWaitingTime() == withPolicy.first->getAverageWaitingTime());
	REQUIRE(baseline.first->getTotalEnergyConsumption() == withPolicy.first->getTotalEnergyConsumption());
}
