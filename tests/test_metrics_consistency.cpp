#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "Simulator.h"
#include "RandomProvider.h"
#include "StandardMetricsCollector.h"
#include "ISimulationLogger.h"
#include "LiftMotorModel.h"
#include "FixedDwellTimeStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "IGroupDispatchStrategy.h"
#include "ITrafficProfile.h"
#include "LunchPeakTrafficProfile.h"
#include "FcfsSchedulingStrategy.h"
#include "StandardPassengerBehavior.h"

using namespace openEGCS;

// Why this file exists
// ===================
// test_metrics_collector.cpp tests the metric *machinery* (it feeds the
// IMetricsCollector hooks by hand) and test_lift_motor_model.cpp tests the
// *physics* (it pins the motor model's eq 7.5-7.12 to Peters' worked example,
// Et = 1.911e5 J). This file tests the *wiring* between the running simulation
// and the machinery. A defect can live exactly there with every unit test
// green and the physics right: an engine that calls onElevatorDeparted once
// per transfer instead of once per leg reports an energy total tens of percent
// too high in a saturated scenario.
//
// An aggregate number cannot be judged by looking at it. What can be judged is
// whether it satisfies identities and whether it matches a second, independent
// derivation. That is what the two groups below do:
//
//   [metrics_oracle]     recompute each reported aggregate from the logged event
//                        timeline and require the two to agree. It catches a
//                        wrong number, not merely a changed one.
//   [metrics_invariants] properties that must hold for any seed and any
//                        scenario. They carry no expected value, so they never
//                        need updating and never go stale.
//
// Both run through the real Simulator, so a future change to the event loop,
// to the hook call sites, or to route/timing code has to keep them true. Each
// case below says what it catches: the point is to know which failures are
// actually covered, instead of trusting a green suite.

namespace {

// Captures what the simulation would have written to snapshot.json, without
// touching the filesystem (the real JsonSimulationLogger is covered by
// test_json_snapshot.cpp; here the timeline itself is the subject).
struct RecordingLogger : public ISimulationLogger {
	struct LoggedEvent {
		double time;
		std::string type;
		int elevatorId;
		unsigned long int customerId;
		int floor;
	};

	std::vector<LoggedEvent> events;
	std::map<unsigned long int, double> arrivalTimeByCustomer;

	void setMetadata(const SimulationMetadata&) override {}
	void logCustomer(const Customer& customer) override {
		arrivalTimeByCustomer[customer.getId()] = customer.getTime();
	}
	void logEvent(double time, const std::string& type, int elevatorId,
		unsigned long int customerId, int floor) override {
		events.push_back({time, type, elevatorId, customerId, floor});
	}
	void finalize(const IMetricsCollector&, const std::string&) override {}
};

// The real collector, plus a record of every energy charge it was handed.
struct RecordingMetrics : public StandardMetricsCollector {
	// Inherited so a scenario can configure the dissatisfaction thresholds;
	// the no-argument form keeps working because every base parameter has a
	// default.
	using StandardMetricsCollector::StandardMetricsCollector;

	struct Charge {
		int elevatorId;
		double floorsTravelled;	// signed
		double currentLoad;
		double time;
	};

	std::vector<Charge> charges;

	void onElevatorDeparted(int elevatorId, double floorsTravelled, double currentLoad, double time) override {
		charges.push_back({elevatorId, floorsTravelled, currentLoad, time});
		StandardMetricsCollector::onElevatorDeparted(elevatorId, floorsTravelled, currentLoad, time);
	}
};

// A group strategy that genuinely REASSIGNS calls during a run, yet converges
// (no livelock): a pool call is placed by the greedy, and an allocated
// non-committed call is steered toward a cabin that is a pure function of its
// floor (floor % N + 1). Because the target depends only on the floor, once a
// call reaches it the next reoptimize returns the same cabin and it stops
// moving; a call pinned as a cabin's front is excluded. The greedy's first pick
// usually differs from the floor target, so reassignments really happen, and
// every invariant in this file then has to survive a landing call changing
// cabins mid-run.
struct FloorTargetGroup : IGroupDispatchStrategy {
	std::shared_ptr<examples::GreedyNearestDispatchStrategy> greedy =
		std::make_shared<examples::GreedyNearestDispatchStrategy>();
	int elevators;
	long movesRequested = 0;		// steers that asked to change a call's cabin (proves the path was exercised)
	explicit FloorTargetGroup(int n) : elevators(n) {}
	std::vector<int> allocate(const std::vector<HallCall>& calls,
		const std::vector<Elevator>& cars, IRandomProvider& /*rng*/) override {
		std::vector<int> out;
		out.reserve(calls.size());
		for (const HallCall& c : calls) {
			if (c.currentElevatorId == 0) {
				// Pool call: let the greedy place it, on a scratch copy.
				Task req(0, "", CALL, c.direction, c.floor, c.registeredAt);
				std::vector<Elevator> scratch(cars.begin(), cars.end());
				out.push_back(greedy->selectElevator(req, scratch));
			} else if (!c.committed) {
				int target = (c.floor % elevators) + 1;		// steer toward the floor's target cabin
				if (target != c.currentElevatorId) movesRequested++;
				out.push_back(target);
			} else {
				out.push_back(c.currentElevatorId);			// committed: cannot move
			}
		}
		return out;
	}
};

struct Scenario {
	int floors;
	int elevators;
	int customers;
	double arrivalRate;
	uint64_t seed;
	// Stop duration. Zero for most scenarios; the ones with a real dwell make
	// every invariant in this file be exercised against a route whose timing is
	// shaped by door and transfer time, not only by travel.
	double doorOpen = 0.0;
	double doorClose = 0.0;
	double perPassenger = 0.0;
	// Dissatisfaction thresholds. Unset on most scenarios on purpose: the
	// oracle below has to hold with and without a verdict, and "unset" is the
	// shipped default.
	std::optional<double> waitingThreshold = std::nullopt;
	std::optional<double> journeyThreshold = std::nullopt;
	// Cabin capacity. Unlimited on most scenarios because that is the shipped
	// default, but several scenarios below configure it: with a real capacity
	// the engine refuses boardings, takes allocated calls back from full cabins
	// and re-dispatches them, and every invariant in this file has to survive
	// that - most pointedly "nobody is left behind", since a call that comes out
	// of the timeline and is not put back is a passenger lost.
	double maxWeight = std::numeric_limits<double>::infinity();
	// A group dispatch strategy. Null on every scenario but one, because null
	// gives the ImmediateGroupAdapter; the one that sets it reassigns calls
	// between cabins mid-run, which every invariant in this file then has to
	// survive.
	std::shared_ptr<IGroupDispatchStrategy> group = nullptr;
	// A demand profile. Null gives the stationary uniform default; a peaked
	// profile is what gives a run a real, large drain - the regime the windowed
	// reading exists for. `arrivalRate` is ignored when a profile is set (the
	// profile owns its rate).
	std::shared_ptr<ITrafficProfile> trafficProfile = nullptr;
	// A route-ordering policy. Null gives LOOK; the scenarios that set it run
	// examples::FcfsSchedulingStrategy, which reverses with people aboard, passes
	// floors where someone wants out and never retargets a leg - route shapes
	// LOOK never produces. Every invariant in this file is about the engine, not
	// about LOOK, so every one of them has to hold under it; one that did not
	// would be an engine defect LOOK had been hiding.
	std::shared_ptr<IElevatorSchedulingStrategy> scheduling = nullptr;
	// What the passenger decides at the door. Null gives the default, which
	// boards every car it fits into.
	std::shared_ptr<IPassengerBehaviorStrategy> behavior = nullptr;
};

struct Run {
	std::shared_ptr<RecordingMetrics> metrics;
	std::shared_ptr<RecordingLogger> logger;
	Scenario scenario;
};

Run runSimulation(const Scenario& s) {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(s.seed));

	ServiceThresholds thresholds;
	thresholds.waiting = s.waitingThreshold;
	thresholds.journey = s.journeyThreshold;
	// The first four arguments are the collector's own defaults, spelled out
	// only because the thresholds come last.
	auto metrics = std::make_shared<RecordingMetrics>(/*energy_per_floor=*/1.0,
		/*floorHeightM=*/3.5, defaultDynamicsFor(20.0), LiftMotorParams(), thresholds);
	auto logger = std::make_shared<RecordingLogger>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();

	auto dwell = std::make_shared<FixedDwellTimeStrategy>(s.doorOpen, s.doorClose, s.perPassenger);

	Simulator sim(1, "metrics-consistency", s.floors, s.elevators, s.customers,
		s.arrivalRate, /*speed=*/1.0f, /*sane=*/true, rng, dispatch, metrics, logger,
		/*seed_for_metadata=*/std::nullopt,
		/*max_weight=*/s.maxWeight,
		/*capacity_strategy=*/nullptr,
		/*mean_customer_weight=*/70.0, /*stddev_customer_weight=*/15.0, dwell,
		/*travel_strategy=*/nullptr, /*passenger_behavior=*/s.behavior,
		/*group_dispatch_strategy=*/s.group,
		/*traffic_profile=*/s.trafficProfile,
		/*scheduling_strategy=*/s.scheduling);
	sim.startSimulation();

	return Run{metrics, logger, s};
}

// Boarding/alighting instants as the *timeline* records them, which is an
// entirely different path from the collector's own hooks.
struct TimelineJourneys {
	std::map<unsigned long int, double> boardTime;
	std::map<unsigned long int, double> alightTime;
	std::map<unsigned long int, int> enterCount;
	std::map<unsigned long int, int> leaveCount;
	int departRecords = 0;
};

TimelineJourneys journeysFrom(const RecordingLogger& logger) {
	TimelineJourneys j;
	for (const RecordingLogger::LoggedEvent& e : logger.events) {
		if (e.type == "ENTER") {
			j.boardTime[e.customerId] = e.time;
			j.enterCount[e.customerId]++;
		} else if (e.type == "LEAVE") {
			j.alightTime[e.customerId] = e.time;
			j.leaveCount[e.customerId]++;
		} else if (e.type == "DEPART") {
			j.departRecords++;
		}
	}
	return j;
}

// The scenarios the rest of the file runs against. The saturated one matters:
// a per-transfer energy over-count is much larger there than in the small
// scenario, and a boarded/alighted population split only shows up under
// saturation. Together they take about a second in a Release build.
const std::vector<Scenario> kScenarios = {
	{/*floors=*/10, /*elevators=*/3, /*customers=*/50, /*rate=*/0.8, /*seed=*/42},
	// Same scenario, now with both thresholds configured. The pair is chosen so
	// the verdict is neither empty nor everybody (3 of 50 dissatisfied by waiting
	// and 6 of 50 by journey): a threshold that nobody or everybody crosses would
	// let a broken filter pass the oracle below.
	{/*floors=*/10, /*elevators=*/3, /*customers=*/50, /*rate=*/0.8, /*seed=*/42,
		/*doorOpen=*/0.0, /*doorClose=*/0.0, /*perPassenger=*/0.0,
		/*waitingThreshold=*/10.0, /*journeyThreshold=*/15.0},
	{/*floors=*/10, /*elevators=*/1, /*customers=*/12, /*rate=*/0.1, /*seed=*/1},
	{/*floors=*/15, /*elevators=*/2, /*customers=*/200, /*rate=*/2.0, /*seed=*/99},
	{/*floors=*/20, /*elevators=*/4, /*customers=*/400, /*rate=*/3.0, /*seed=*/7},
	// With a real stop duration: door time reshapes the route (a slower cabin
	// batches more passengers per stop), so this is not the same run with
	// larger numbers - it is a different route, and every invariant below has
	// to hold for it too.
	{/*floors=*/12, /*elevators=*/2, /*customers=*/120, /*rate=*/1.5, /*seed=*/4242,
		/*doorOpen=*/2.0, /*doorClose=*/3.0, /*perPassenger=*/1.0},
	// With a real cabin capacity. 300 kg against a mean passenger of 70 kg is a
	// four-person cabin, and this run really does refuse boardings and
	// re-dispatch calls, so it exercises the capacity paths rather than merely
	// enabling them. A capacity nobody reaches would let that code rot untested
	// behind a green suite.
	{/*floors=*/10, /*elevators=*/3, /*customers=*/50, /*rate=*/0.8, /*seed=*/42,
		/*doorOpen=*/0.0, /*doorClose=*/0.0, /*perPassenger=*/0.0,
		/*waitingThreshold=*/std::nullopt, /*journeyThreshold=*/std::nullopt,
		/*maxWeight=*/300.0},
	// Capacity *and* a dwell, because the two interact: a slower stop batches
	// more passengers, which fills a cabin sooner, which is what makes a
	// capacity bite.
	{/*floors=*/12, /*elevators=*/2, /*customers=*/120, /*rate=*/1.5, /*seed=*/4242,
		/*doorOpen=*/2.0, /*doorClose=*/3.0, /*perPassenger=*/1.0,
		/*waitingThreshold=*/std::nullopt, /*journeyThreshold=*/std::nullopt,
		/*maxWeight=*/500.0},
	// Two saturated regimes for what a stop costs: with an unlimited cabin, where
	// a cabin boards a floor's whole queue though only some of it was registered
	// on its stop, and with a finite one, where it refuses most of a queue that
	// was. The dwell must be charged for the transfers that really happen in
	// both. The two dwell scenarios above reach neither.
	{/*floors=*/20, /*elevators=*/4, /*customers=*/400, /*rate=*/3.0, /*seed=*/7,
		/*doorOpen=*/2.0, /*doorClose=*/3.0, /*perPassenger=*/1.2},
	{/*floors=*/11, /*elevators=*/4, /*customers=*/500, /*rate=*/1.0, /*seed=*/7,
		/*doorOpen=*/2.0, /*doorClose=*/2.0, /*perPassenger=*/1.2,
		/*waitingThreshold=*/std::nullopt, /*journeyThreshold=*/std::nullopt,
		/*maxWeight=*/1000.0},
	// A small cabin at a LOW arrival rate, which is a different regime from the
	// two capacity scenarios above: cabins go idle often and calls come back
	// from the pool, so a parked cabin is regularly handed a landing call at
	// the floor it is standing at - a leg of zero length. Such a cabin must get
	// a direction, or the stop it stands at is sorted behind the next car call
	// and from then on its stops complete nothing. None of the other scenarios
	// reach this regime.
	{/*floors=*/20, /*elevators=*/4, /*customers=*/300, /*rate=*/0.4, /*seed=*/7,
		/*doorOpen=*/0.0, /*doorClose=*/0.0, /*perPassenger=*/0.0,
		/*waitingThreshold=*/std::nullopt, /*journeyThreshold=*/std::nullopt,
		/*maxWeight=*/600.0},
	// A run that genuinely REASSIGNS calls between cabins mid-flight.
	// FloorTargetGroup steers each allocated non-committed call toward a
	// floor-based cabin, so a landing call really does change cabins after it
	// was allocated - and every invariant in this file has to survive that, most
	// pointedly "every created customer reaches their destination" (a call lost
	// in a move would strand a passenger) and the per-cabin DEPART/ARRIVE
	// alternation (a move that left a dangling ARRIVE or dropped a DEPART would
	// break it).
	{/*floors=*/15, /*elevators=*/3, /*customers=*/200, /*rate=*/1.5, /*seed=*/123,
		/*doorOpen=*/0.0, /*doorClose=*/0.0, /*perPassenger=*/0.0,
		/*waitingThreshold=*/std::nullopt, /*journeyThreshold=*/std::nullopt,
		/*maxWeight=*/std::numeric_limits<double>::infinity(),
		/*group=*/std::make_shared<FloorTargetGroup>(3)},
	// The same engine under FCFS route ordering, in four of the regimes above:
	// the small uncapacitated one, the saturated one, a capacity that refuses
	// boardings, and the small cabin at a low rate. Unlimited-capacity defaults
	// are spelled out only because the scheduling field comes last.
	{/*floors=*/10, /*elevators=*/3, /*customers=*/50, /*rate=*/0.8, /*seed=*/42,
		/*doorOpen=*/0.0, /*doorClose=*/0.0, /*perPassenger=*/0.0,
		/*waitingThreshold=*/std::nullopt, /*journeyThreshold=*/std::nullopt,
		/*maxWeight=*/std::numeric_limits<double>::infinity(),
		/*group=*/nullptr, /*trafficProfile=*/nullptr,
		/*scheduling=*/std::make_shared<examples::FcfsSchedulingStrategy>()},
	{/*floors=*/20, /*elevators=*/4, /*customers=*/400, /*rate=*/3.0, /*seed=*/7,
		/*doorOpen=*/0.0, /*doorClose=*/0.0, /*perPassenger=*/0.0,
		/*waitingThreshold=*/std::nullopt, /*journeyThreshold=*/std::nullopt,
		/*maxWeight=*/std::numeric_limits<double>::infinity(),
		/*group=*/nullptr, /*trafficProfile=*/nullptr,
		/*scheduling=*/std::make_shared<examples::FcfsSchedulingStrategy>()},
	{/*floors=*/12, /*elevators=*/2, /*customers=*/120, /*rate=*/1.5, /*seed=*/4242,
		/*doorOpen=*/2.0, /*doorClose=*/3.0, /*perPassenger=*/1.0,
		/*waitingThreshold=*/std::nullopt, /*journeyThreshold=*/std::nullopt,
		/*maxWeight=*/500.0,
		/*group=*/nullptr, /*trafficProfile=*/nullptr,
		/*scheduling=*/std::make_shared<examples::FcfsSchedulingStrategy>()},
	{/*floors=*/20, /*elevators=*/4, /*customers=*/300, /*rate=*/0.4, /*seed=*/7,
		/*doorOpen=*/0.0, /*doorClose=*/0.0, /*perPassenger=*/0.0,
		/*waitingThreshold=*/std::nullopt, /*journeyThreshold=*/std::nullopt,
		/*maxWeight=*/600.0,
		/*group=*/nullptr, /*trafficProfile=*/nullptr,
		/*scheduling=*/std::make_shared<examples::FcfsSchedulingStrategy>()},
};

// A leg of (near) zero length is real: a customer calls from the floor an idle
// cabin already sits on, so the cabin "departs" a distance of 0 and the real
// leg starts at the same instant. The residual is float noise in
// `processingTask.getDest() - position` (~1e-8 floors), not an exact zero -
// hence a tolerance in floors rather than == 0.
constexpr double kZeroLegFloors = 1e-6;

// Nearest-rank quantile, restated here from its definition rather than shared
// with the collector. The independence this file claims is of the *input*: the
// values below come from the logged event timeline, a path that can disagree
// with the hooks while every unit test stays green. The formula being the same
// formula is the point - two derivations of the same definition over two
// different inputs have to agree.
double quantileOf(std::vector<double> values, double p) {
	if (values.empty()) return 0.0;
	std::sort(values.begin(), values.end());
	std::size_t rank = static_cast<std::size_t>(std::ceil(p * values.size()));
	if (rank == 0) rank = 1;
	if (rank > values.size()) rank = values.size();
	return values[rank - 1];
}

double meanOf(const std::vector<double>& values) {
	if (values.empty()) return 0.0;
	double total = 0.0;
	for (double v : values) total += v;
	return total / values.size();
}

// Per-customer waiting and journey times as the timeline records them, in id
// order (arrivalTimeByCustomer is a std::map), which is deliberately *not*
// the service order the collector reports its dissatisfied list in - so the
// comparison has to sort, and a service-order bug cannot hide behind it.
struct TimelineTimes {
	std::vector<double> waits;
	std::vector<double> journeys;
	std::vector<unsigned long int> waitedAbove;		// ids, id order
	std::vector<unsigned long int> journeyedAbove;
};

TimelineTimes timelineTimes(const RecordingLogger& logger, const TimelineJourneys& j,
	const std::optional<double>& waitingThreshold, const std::optional<double>& journeyThreshold) {
	TimelineTimes t;
	for (const auto& entry : logger.arrivalTimeByCustomer) {
		unsigned long int id = entry.first;
		double arrival = entry.second;
		double wait = j.boardTime.at(id) - arrival;
		double journey = j.alightTime.at(id) - arrival;

		t.waits.push_back(wait);
		t.journeys.push_back(journey);
		if (waitingThreshold.has_value() && wait > waitingThreshold.value()) t.waitedAbove.push_back(id);
		if (journeyThreshold.has_value() && journey > journeyThreshold.value()) t.journeyedAbove.push_back(id);
	}
	return t;
}

// The per-window split and the derived queue, recomputed from the timeline -
// the independent path those methods are checked against. The boundary is
// the last arrival instant; a sample is "peak" if the event that closes it
// (boarding for waiting, alighting for journey) is at or before the boundary
// (the <= tie); the queue is +1 at each arrival, -1 at each boarding, with
// arrivals before boardings at an equal instant (the inclusive tie).
struct TimelineWindows {
	double boundary = 0.0;
	int waitPeak = 0, waitDrain = 0;
	int journeyPeak = 0, journeyDrain = 0;
	double waitPeakMean = 0.0, waitDrainMean = 0.0;
	int maxQueue = 0;
	int queueAtEnd = 0;
};

TimelineWindows timelineWindows(const RecordingLogger& logger, const TimelineJourneys& j) {
	TimelineWindows w;
	for (const auto& e : logger.arrivalTimeByCustomer)
		if (e.second > w.boundary) w.boundary = e.second;

	std::vector<double> waitPeakVals, waitDrainVals;
	for (const auto& e : logger.arrivalTimeByCustomer) {
		unsigned long int id = e.first;
		double arrival = e.second;
		double board = j.boardTime.at(id);
		double alight = j.alightTime.at(id);
		if (board <= w.boundary) { w.waitPeak++; waitPeakVals.push_back(board - arrival); }
		else { w.waitDrain++; waitDrainVals.push_back(board - arrival); }
		if (alight <= w.boundary) w.journeyPeak++; else w.journeyDrain++;
		if (board > w.boundary) w.queueAtEnd++;
	}
	w.waitPeakMean = meanOf(waitPeakVals);
	w.waitDrainMean = meanOf(waitDrainVals);

	struct QEvent { double t; int d; };
	std::vector<QEvent> evs;
	evs.reserve(logger.arrivalTimeByCustomer.size() + j.boardTime.size());
	for (const auto& e : logger.arrivalTimeByCustomer) evs.push_back({e.second, +1});
	for (const auto& e : j.boardTime) evs.push_back({e.second, -1});
	std::sort(evs.begin(), evs.end(), [](const QEvent& a, const QEvent& b) {
		if (a.t != b.t) return a.t < b.t;
		return a.d > b.d;
	});
	int cur = 0, mx = 0;
	for (const QEvent& e : evs) { cur += e.d; if (cur > mx) mx = cur; }
	w.maxQueue = mx;
	return w;
}

} // namespace

// ---------------------------------------------------------------------------
// Layer 1 - the oracle: every reported aggregate must equal an independent
// recomputation from the logged timeline.
// ---------------------------------------------------------------------------

// Both averages are over the same population, and that population is every
// customer the run created: the simulation only ends once all of them have
// reached their destination, so "boarded", "alighted" and "created" are the
// same set. If the run could end early, getAverageWaitingTime() would average
// over whoever had boarded while getAverageTravelTime() averaged over whoever
// had alighted.
TEST_CASE("Reported averages equal an independent recomputation from the event timeline", "[metrics_oracle]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);
		TimelineJourneys j = journeysFrom(*run.logger);

		double waitingOverBoarded = 0;
		int boarded = 0;
		for (const auto& entry : j.boardTime) {
			waitingOverBoarded += entry.second - run.logger->arrivalTimeByCustomer.at(entry.first);
			boarded++;
		}

		double travelOverAlighted = 0;
		int alighted = 0;
		for (const auto& entry : j.alightTime) {
			travelOverAlighted += entry.second - j.boardTime.at(entry.first);
			alighted++;
		}

		REQUIRE(alighted == s.customers);
		REQUIRE(boarded == alighted); // one population, see the NOTE above

		REQUIRE(run.metrics->getAverageWaitingTime()
			== Catch::Approx(waitingOverBoarded / boarded).epsilon(1e-9));
		REQUIRE(run.metrics->getAverageTravelTime()
			== Catch::Approx(travelOverAlighted / alighted).epsilon(1e-9));
	}
}

// What this does and does not catch: the energy charges and the snapshot's
// DEPART records leave giveEvent through different paths (the collector hook
// vs ElevatorSystem::drainDepartures -> Simulator -> ISimulationLogger) but
// under the same condition, so this detects a path that drops or duplicates
// records - a forgotten drain in a new branch - and NOT a charge emitted per
// transfer instead of per leg. The per-leg property is pinned by
// "No elevator is charged twice at one instant" below.
//
// The exact oracle for the leg count is not available from the timeline: a
// DEPART carries only (elevator, floor, time), so the number of legs cannot be
// re-derived from the snapshot alone (a leg can be retargeted mid-flight, and
// zero-length legs add charges without adding stops). Recording the signed
// distance and the load on each DEPART would close that gap, additively, and
// would also enable a per-elevator energy breakdown in the snapshot.
TEST_CASE("Every energy charge corresponds to exactly one logged DEPART", "[metrics_oracle]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);
		TimelineJourneys j = journeysFrom(*run.logger);

		REQUIRE(run.metrics->charges.size() > 0);
		REQUIRE(static_cast<int>(run.metrics->charges.size()) == j.departRecords);
	}
}

// Re-prices every recorded leg with tripEnergyJ and requires the sum to be the
// reported total. This separates "the model is wrong" from "the model is fed
// wrong": it pins the collector's accumulation (per-elevator bookkeeping, the
// deprecated onElevatorMoved staying out of the total, idle power staying off
// by default), and deliberately says nothing about whether the right number of
// legs was fed in - that is the invariant group's job.
TEST_CASE("Total energy equals the independent pricing of the recorded legs", "[metrics_oracle]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);

		// Same installation parameters StandardMetricsCollector defaults to.
		const double floorHeightM = 3.5;
		LiftDynamics dynamics = defaultDynamicsFor(20.0);
		LiftMotorParams motorParams;

		double independentTotal = 0;
		for (const RecordingMetrics::Charge& c : run.metrics->charges) {
			independentTotal += tripEnergyJ(c.floorsTravelled * floorHeightM, c.currentLoad,
				dynamics, motorParams);
		}

		REQUIRE(run.metrics->getTotalEnergyConsumption()
			== Catch::Approx(independentTotal).epsilon(1e-9));
	}
}

// ---------------------------------------------------------------------------
// Layer 2 - invariants: no expected values, so they never need maintenance.
// ---------------------------------------------------------------------------

TEST_CASE("Total energy is the sum of the per-elevator energies, finite and non-negative", "[metrics_invariants]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);

		double sumPerElevator = 0;
		for (int id = 1; id <= s.elevators; id++) {
			double perElevator = run.metrics->getEnergyConsumption(id);
			REQUIRE(std::isfinite(perElevator));
			REQUIRE(perElevator >= 0.0);
			sumPerElevator += perElevator;
		}

		double total = run.metrics->getTotalEnergyConsumption();
		REQUIRE(std::isfinite(total));
		REQUIRE(total >= 0.0);
		REQUIRE(total == Catch::Approx(sumPerElevator).epsilon(1e-9));
	}
}

TEST_CASE("Every served customer boards once, alights once, and in that order", "[metrics_invariants]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);
		TimelineJourneys j = journeysFrom(*run.logger);

		for (const auto& entry : j.alightTime) {
			unsigned long int customerId = entry.first;
			REQUIRE(j.enterCount.at(customerId) == 1);
			REQUIRE(j.leaveCount.at(customerId) == 1);

			double arrival = run.logger->arrivalTimeByCustomer.at(customerId);
			double board = j.boardTime.at(customerId);
			double alight = entry.second;
			REQUIRE(arrival <= board);
			REQUIRE(board <= alight);
		}
	}
}

// Charging a leg once per transfer, stated as a property. Two charges at the
// same instant for the same elevator are legitimate only when one of the two
// legs has zero length (see kZeroLegFloors); charging the same moving leg
// twice violates this.
TEST_CASE("No elevator is charged twice at one instant for a leg it travels once", "[metrics_invariants]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);

		std::map<std::pair<int, long long>, std::vector<double>> distancesByInstant;
		for (const RecordingMetrics::Charge& c : run.metrics->charges) {
			// Quantised so float noise in the timestamp cannot split a genuine
			// tie; event times at one stop are produced by the same expression.
			long long instant = std::llround(c.time * 1e6);
			distancesByInstant[{c.elevatorId, instant}].push_back(c.floorsTravelled);
		}

		for (const auto& entry : distancesByInstant) {
			const std::vector<double>& distances = entry.second;
			if (distances.size() < 2) continue;

			int movingLegs = 0;
			for (double d : distances) {
				if (std::fabs(d) > kZeroLegFloors) movingLegs++;
			}
			REQUIRE(movingLegs <= 1);
		}
	}
}

// The distance the energy model is charged for must be the distance the
// cabins actually covered. Charged at DEPARTURE as `destination - position`,
// a leg is billed for where the cabin intended to go; a nearer call inserted
// mid-flight re-anchors the route, the cabin stops early, and the remaining
// leg is charged again from there - so the abandoned stretch is paid for
// twice. Legs are priced once they have arrived, when the next one begins,
// with the distance that is then a fact.
//
// A property, not an expected value, so it cannot go stale: it compares the
// charges against the cabins' own stop sequence from the logged timeline.
TEST_CASE("Energy is charged for the distance travelled, not the distance intended", "[metrics_invariants]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);

		// What the energy model was charged for, per elevator.
		std::map<int, double> chargedByElevator;
		for (const RecordingMetrics::Charge& c : run.metrics->charges) {
			chargedByElevator[c.elevatorId] += std::fabs(c.floorsTravelled);
		}

		// What the cabins actually covered: the floors they stopped at, in
		// order, starting from floor 0. One ARRIVE per stop, which is the
		// direct and complete list. Reconstructing it from the transfer rows
		// would miss the stops where nobody got in or out (a cabin cannot
		// un-brake, so a stop whose call another cabin served is still made),
		// and the cabin travelled to get there.
		std::map<int, std::vector<int>> stopsByElevator;
		for (const RecordingLogger::LoggedEvent& e : run.logger->events) {
			if (e.type != "ARRIVE") continue;
			std::vector<int>& stops = stopsByElevator[e.elevatorId];
			if (stops.empty() || stops.back() != e.floor) stops.push_back(e.floor);
		}

		for (const auto& entry : stopsByElevator) {
			int elevatorId = entry.first;
			const std::vector<int>& stops = entry.second;
			double travelled = std::fabs(static_cast<double>(stops.front()));	// from floor 0
			for (size_t i = 1; i < stops.size(); ++i) {
				travelled += std::fabs(static_cast<double>(stops[i]) - stops[i - 1]);
			}

			INFO("scenario " << s.floors << "/" << s.elevators << " seed " << s.seed
				<< ", elevator " << elevatorId << ": charged "
				<< chargedByElevator[elevatorId] << " floors, travelled " << travelled);
			REQUIRE(chargedByElevator[elevatorId] == Catch::Approx(travelled).margin(1e-9));
		}
	}
}

// The conservation law of the whole simulation: the run creates exactly
// `customers` people and may not finish until every one of them has reached
// their destination. A loop that stopped at the Nth alighting while the
// arrival process kept running would end a saturated run with people still
// riding and people still queued on floors, unserved, and the ones still
// waiting invisible to every metric.
TEST_CASE("Nobody is left behind: every created customer reaches their destination", "[metrics_invariants]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);
		TimelineJourneys j = journeysFrom(*run.logger);

		// Exactly the requested number of customers exists...
		REQUIRE(static_cast<int>(run.logger->arrivalTimeByCustomer.size()) == s.customers);

		// ...and each one called, boarded and alighted exactly once. Any
		// customer left riding, left queued on a floor, or never arrived at
		// all would break one of these three.
		int called = 0;
		for (const RecordingLogger::LoggedEvent& e : run.logger->events) {
			if (e.type == "CUST_CALL") called++;
		}
		REQUIRE(called == s.customers);
		REQUIRE(static_cast<int>(j.boardTime.size()) == s.customers);
		REQUIRE(static_cast<int>(j.alightTime.size()) == s.customers);

		for (const auto& entry : run.logger->arrivalTimeByCustomer) {
			REQUIRE(j.alightTime.count(entry.first) == 1);
		}
	}
}

TEST_CASE("Every energy charge is a finite distance carrying a non-negative load", "[metrics_invariants]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);

		for (const RecordingMetrics::Charge& c : run.metrics->charges) {
			REQUIRE(std::isfinite(c.floorsTravelled));
			REQUIRE(std::fabs(c.floorsTravelled) <= static_cast<double>(s.floors));
			REQUIRE(std::isfinite(c.currentLoad));
			REQUIRE(c.currentLoad >= 0.0);
			REQUIRE(c.elevatorId >= 1);
			REQUIRE(c.elevatorId <= s.elevators);
		}
	}
}

TEST_CASE("A leg that leaves with nobody aboard is priced carrying exactly nothing", "[metrics_invariants]") {
	// The load is a running sum of non-integer weights, and boarding and
	// alighting the same people need not bring it back to zero (stray loads of
	// 1e-15 to 1e-11 kg). Harmless to the energy, but it is a cabin the engine
	// would believe is carrying somebody, and the liveness guard asks exactly
	// that. An emptied cabin must price its next leg at exactly zero load.
	//
	// Who is aboard comes from the timeline (ENTER/LEAVE, in the order they
	// were logged, which is the physical order inside an instant), and the load
	// from the collector's hook - two independent paths. A charge is for the leg
	// that DEPARTed at its own `time`, one leg behind, so per cabin the k-th
	// charge is the k-th DEPART.
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);

		std::map<int, int> aboard;
		std::map<int, std::vector<std::pair<double, int>>> departures;	// per cabin: (time, people aboard)
		for (const RecordingLogger::LoggedEvent& e : run.logger->events) {
			if (e.type == "ENTER") aboard[e.elevatorId]++;
			if (e.type == "LEAVE") aboard[e.elevatorId]--;
			if (e.type == "DEPART") departures[e.elevatorId].push_back({e.time, aboard[e.elevatorId]});
		}

		std::map<int, size_t> seen;
		int emptyLegs = 0;
		for (const RecordingMetrics::Charge& c : run.metrics->charges) {
			size_t k = seen[c.elevatorId]++;
			REQUIRE(k < departures[c.elevatorId].size());
			const std::pair<double, int>& d = departures[c.elevatorId][k];
			INFO("scenario " << s.floors << "/" << s.elevators << "/" << s.customers << " seed " << s.seed
				<< ", elevator " << c.elevatorId << " leg " << k << " at t=" << c.time
				<< ": " << d.second << " aboard, load " << c.currentLoad);
			REQUIRE(c.time == d.first);
			if (d.second == 0) {
				emptyLegs++;
				REQUIRE(c.currentLoad == 0.0);
			} else {
				REQUIRE(c.currentLoad > 0.0);
			}
		}
		// Every scenario sends some cabin out empty (to a landing call, at
		// least the first one), so the exact branch is never vacuous.
		REQUIRE(emptyLegs > 0);
	}
}

// ---------------------------------------------------------------------------
// The dissatisfied-passenger metric.
//
// These live in this file and not only in test_metrics_collector.cpp for the
// reason the header of this file gives: the unit tests feed the hooks by hand,
// and a defect in the *wiring* between the running simulation and the
// collector passes every unit test. A distribution is a consumer of that same
// wiring, so it gets the same treatment - recomputed from the logged timeline,
// which is an independent path from the hooks.
// ---------------------------------------------------------------------------

TEST_CASE("The reported time distributions equal an independent recomputation from the event timeline", "[metrics_oracle]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);
		TimelineJourneys j = journeysFrom(*run.logger);
		TimelineTimes t = timelineTimes(*run.logger, j, s.waitingThreshold, s.journeyThreshold);

		TimeDistribution waiting = run.metrics->getWaitingTimeDistribution();
		TimeDistribution journey = run.metrics->getJourneyTimeDistribution();

		REQUIRE(waiting.count == static_cast<int>(t.waits.size()));
		REQUIRE(journey.count == static_cast<int>(t.journeys.size()));

		REQUIRE(waiting.average == Catch::Approx(meanOf(t.waits)));
		REQUIRE(waiting.p50 == Catch::Approx(quantileOf(t.waits, 0.50)));
		REQUIRE(waiting.p90 == Catch::Approx(quantileOf(t.waits, 0.90)));
		REQUIRE(waiting.max == Catch::Approx(*std::max_element(t.waits.begin(), t.waits.end())));

		REQUIRE(journey.average == Catch::Approx(meanOf(t.journeys)));
		REQUIRE(journey.p50 == Catch::Approx(quantileOf(t.journeys, 0.50)));
		REQUIRE(journey.p90 == Catch::Approx(quantileOf(t.journeys, 0.90)));
		REQUIRE(journey.max == Catch::Approx(*std::max_element(t.journeys.begin(), t.journeys.end())));

		// The verdict, when one was configured: the same people, not merely
		// the same count. Sorted on both sides because the collector reports
		// service order and the recomputation walks id order.
		REQUIRE(waiting.threshold.has_value() == s.waitingThreshold.has_value());
		REQUIRE(journey.threshold.has_value() == s.journeyThreshold.has_value());

		std::vector<unsigned long int> reportedWaiting = waiting.dissatisfied;
		std::vector<unsigned long int> reportedJourney = journey.dissatisfied;
		std::sort(reportedWaiting.begin(), reportedWaiting.end());
		std::sort(reportedJourney.begin(), reportedJourney.end());
		REQUIRE(reportedWaiting == t.waitedAbove);
		REQUIRE(reportedJourney == t.journeyedAbove);
	}
}

TEST_CASE("Configuring a dissatisfaction threshold changes nothing else about the run", "[metrics_invariants]") {
	// Dissatisfaction is a label applied after the fact, so turning it on must
	// not move the simulation by one event or one aggregate. If a change makes
	// the threshold reach the engine - a passenger who gives up, say - this is
	// the test that fails, and it should.
	Scenario without = {/*floors=*/15, /*elevators=*/2, /*customers=*/200, /*rate=*/2.0, /*seed=*/99};
	Scenario with = without;
	with.waitingThreshold = 5.0;
	with.journeyThreshold = 9.0;

	Run a = runSimulation(without);
	Run b = runSimulation(with);

	REQUIRE(a.metrics->getAverageWaitingTime() == b.metrics->getAverageWaitingTime());
	REQUIRE(a.metrics->getAverageTravelTime() == b.metrics->getAverageTravelTime());
	REQUIRE(a.metrics->getTotalEnergyConsumption() == b.metrics->getTotalEnergyConsumption());
	REQUIRE(a.metrics->getWaitingTimeDistribution().p90 == b.metrics->getWaitingTimeDistribution().p90);

	// Same timeline, event for event - the strongest form of the claim.
	REQUIRE(a.logger->events.size() == b.logger->events.size());
	for (std::size_t i = 0; i < a.logger->events.size(); i++) {
		REQUIRE(a.logger->events[i].time == b.logger->events[i].time);
		REQUIRE(a.logger->events[i].type == b.logger->events[i].type);
		REQUIRE(a.logger->events[i].customerId == b.logger->events[i].customerId);
		REQUIRE(a.logger->events[i].floor == b.logger->events[i].floor);
	}

	// ...and the verdict only exists on the run that asked for one.
	REQUIRE_FALSE(a.metrics->getWaitingTimeDistribution().dissatisfiedCount().has_value());
	REQUIRE(b.metrics->getWaitingTimeDistribution().dissatisfiedCount().has_value());
}

TEST_CASE("Distribution invariants: ordered quantiles, and journey dominates waiting at every rank", "[metrics_invariants]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);

		TimeDistribution waiting = run.metrics->getWaitingTimeDistribution();
		TimeDistribution journey = run.metrics->getJourneyTimeDistribution();

		for (const TimeDistribution& d : {waiting, journey}) {
			REQUIRE(d.count == s.customers);			// everybody is served
			REQUIRE(std::isfinite(d.average));
			REQUIRE(d.p50 >= 0.0);
			REQUIRE(d.p50 <= d.p90);
			REQUIRE(d.p90 <= d.max);
			REQUIRE(d.average <= d.max);
			if (d.threshold.has_value()) {
				REQUIRE(d.dissatisfiedCount().value() <= d.count);
				REQUIRE(d.dissatisfiedRate().value() >= 0.0);
				REQUIRE(d.dissatisfiedRate().value() <= 1.0);
			}
		}

		// Every passenger's journey is at least their own wait (travel time is
		// never negative), and elementwise domination carries to every
		// quantile - so this holds rank by rank, not just on the mean.
		REQUIRE(journey.p50 >= waiting.p50);
		REQUIRE(journey.p90 >= waiting.p90);
		REQUIRE(journey.max >= waiting.max);

		// The complete-set identity, on the aggregate: journey = waiting +
		// travel with no period missing. It holds exactly here only because
		// both averages are over the same population.
		REQUIRE(journey.average == Catch::Approx(waiting.average + run.metrics->getAverageTravelTime()));
	}
}

TEST_CASE("The dissatisfied count never grows when the threshold is raised", "[metrics_invariants]") {
	// A property with no expected value: whatever the scenario does, a more
	// tolerant passenger cannot be dissatisfied more often. It is the cheapest
	// guard against the comparison drifting to the wrong side, which would
	// otherwise be invisible in an absolute number nobody can eyeball.
	Scenario base = {/*floors=*/12, /*elevators=*/2, /*customers=*/120, /*rate=*/1.5, /*seed=*/4242,
		/*doorOpen=*/2.0, /*doorClose=*/3.0, /*perPassenger=*/1.0};

	int previous = base.customers + 1;
	for (double threshold : {1.0, 3.0, 5.0, 10.0, 20.0, 1000.0}) {
		Scenario s = base;
		s.waitingThreshold = threshold;
		Run run = runSimulation(s);

		int count = run.metrics->getWaitingTimeDistribution().dissatisfiedCount().value();
		REQUIRE(count <= previous);
		previous = count;
	}
	// The most tolerant threshold of all leaves nobody dissatisfied, which is
	// the other end of the ramp and a sanity check that the filter runs at all.
	REQUIRE(previous == 0);
}

// ---------------------------------------------------------------------------
// Capacity. Several of the scenarios above configure a finite cabin; these
// cases are about what that must and must not do.
// ---------------------------------------------------------------------------

TEST_CASE("No leg is ever priced carrying more than the cabin's rated load", "[metrics_invariants]") {
	// The metrics-side view of the rated load as a hard limit. It is worth
	// asserting here, and not only in the engine, because the load is what the
	// energy model is priced on: an overloaded cabin would make every reported
	// joule describe a cabin that cannot exist.
	//
	// Elevator::addCustomer throws rather than overload, so a regression would
	// most likely surface as an exception out of runSimulation; this case also
	// covers the path where the load reaches the collector by some route that
	// does not go through addCustomer.
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);
		for (const RecordingMetrics::Charge& charge : run.metrics->charges) {
			REQUIRE(charge.currentLoad >= 0.0);
			REQUIRE(charge.currentLoad <= s.maxWeight);
		}
	}
}

TEST_CASE("Every PASS in the timeline is one refusal the collector counted", "[metrics_oracle]") {
	// Two independent paths to the same fact: the PASS rows of the snapshot
	// (transfer records drained by Simulator and written by the logger) and the
	// collector's onCustomerPassedBy count. If those two ever disagree, the
	// snapshot and the metrics disagree about what happened to a passenger.
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);

		int passLines = 0;
		std::map<unsigned long int, int> passengersSeen;
		for (const RecordingLogger::LoggedEvent& e : run.logger->events) {
			if (e.type != "PASS") continue;
			passLines++;
			passengersSeen[e.customerId]++;
		}

		std::optional<PassByReport> report = run.metrics->getPassByReport();
		REQUIRE(report.has_value());
		REQUIRE(report->refusedAtDoor.events == passLines);
		REQUIRE(report->refusedAtDoor.passengers.size() == passengersSeen.size());

		// A refusal is never the end of a journey: the passenger's call stays
		// registered, so everybody who was passed still boards and still
		// alights (which the "nobody is left behind" case above checks for the
		// whole population, but this says it of exactly the ones it happened
		// to).
		TimelineJourneys j = journeysFrom(*run.logger);
		for (unsigned long int id : report->refusedAtDoor.passengers) {
			REQUIRE(j.boardTime.count(id) == 1);
			REQUIRE(j.alightTime.count(id) == 1);
		}
	}
}

TEST_CASE("Nobody boards twice, and every PASS is followed by a boarding", "[metrics_invariants]") {
	// One landing call serves several people, so a call that is re-registered
	// while some of its waiters are already being served must not board them a
	// second time. That shows up here as a customer boarding twice - which the
	// oracle above would also catch, but only for the customers it recomputes;
	// this says it of everybody, in every scenario, including the ones where
	// nothing goes wrong.
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);

		std::map<unsigned long int, int> boardings;
		std::map<unsigned long int, int> alightings;
		std::map<unsigned long int, int> refusals;
		for (const RecordingLogger::LoggedEvent& e : run.logger->events) {
			if (e.type == "ENTER") boardings[e.customerId]++;
			else if (e.type == "LEAVE") alightings[e.customerId]++;
			else if (e.type == "PASS") refusals[e.customerId]++;
		}

		for (std::map<unsigned long int, int>::const_iterator it = boardings.begin();
			it != boardings.end(); ++it) {
			REQUIRE(it->second == 1);
			REQUIRE(alightings[it->first] == 1);
		}
		// A refusal is never the end of the line: whoever was passed over
		// boarded later, which is what the RECALL event exists to guarantee.
		for (std::map<unsigned long int, int>::const_iterator it = refusals.begin();
			it != refusals.end(); ++it) {
			REQUIRE(boardings[it->first] == 1);
		}
	}
}

TEST_CASE("A run with no capacity configured leaves nobody behind at all", "[metrics_invariants]") {
	// With the shipped default (unlimited capacity) none of the capacity
	// machinery can fire - no cabin is ever bypassing, no call is ever taken back,
	// no boarding is ever refused.
	for (const Scenario& s : kScenarios) {
		if (std::isfinite(s.maxWeight)) continue;
		Run run = runSimulation(s);

		std::optional<PassByReport> report = run.metrics->getPassByReport();
		REQUIRE(report.has_value());
		REQUIRE(report->refusedAtDoor.events == 0);
		REQUIRE(report->reallocated.events == 0);

		for (const RecordingLogger::LoggedEvent& e : run.logger->events) {
			REQUIRE(e.type != "PASS");
		}
	}
}

// Every leg begins with a departure and ends at a stop, so each cabin's
// timeline alternates DEPART, ARRIVE, DEPART, ARRIVE... - starting with a
// DEPART (cabins start parked) and possibly ending with one (a cabin still on
// its way somewhere when the last passenger alights). A property of the
// timeline alone, with no expected value.
//
// It catches two engine defects that the energy invariant above does not
// separate. An ARRIVE that fires for a stop the route had sorted away from
// the front finishes nothing, so no leg starts and the next ARRIVE follows
// with no DEPART between. And a stop that gets a second ARRIVE - a pooled
// call merged into the stop whose ARRIVE is already being processed - puts
// two ARRIVEs in a row at the same instant. Neither need move any aggregate,
// so only this case can see them.
// What a stop costs, read off the timeline against what the dwell model was told.
// An ARRIVE is dated at the doors having opened, so from there to the cabin's
// next DEPART the doors take the transfers and close: n * tp + close, with n the
// ENTER and LEAVE rows of that stop. Nothing else in the suite asks this. The
// oracles above recompute the averages from the timeline, and a timeline in which
// 42 people board in 6.6 s is perfectly coherent with itself: the cabin really
// did leave that early, and everything after it follows.
//
// Only stops somebody boarded at are held to the equality: they leave a car call,
// so the cabin cannot have parked there, and a parked cabin's next DEPART says
// nothing about its doors. Every other stop has to be at least that long.
struct StopRecord {
	double arrive;
	double depart;
	int enter = 0;
	int leave = 0;
	int passed = 0;
	bool opensWithRefusal = false;	// the stop's first transfer row is a PASS
};

std::vector<StopRecord> stopsFrom(const RecordingLogger& logger) {
	std::vector<StopRecord> stops;
	std::map<int, StopRecord> open;
	for (const RecordingLogger::LoggedEvent& e : logger.events) {
		if (e.type == "ARRIVE") {
			StopRecord stop;
			stop.arrive = e.time;
			stop.depart = e.time;
			open[e.elevatorId] = stop;
			continue;
		}
		auto it = open.find(e.elevatorId);
		if (it == open.end()) continue;
		const StopRecord& stop = it->second;
		const bool firstRow = stop.enter + stop.leave + stop.passed == 0;
		if (e.type == "ENTER") it->second.enter++;
		else if (e.type == "LEAVE") it->second.leave++;
		else if (e.type == "PASS") {
			it->second.passed++;
			if (firstRow) it->second.opensWithRefusal = true;
		}
		else if (e.type == "DEPART") {
			it->second.depart = e.time;
			stops.push_back(it->second);
			open.erase(it);
		}
	}
	return stops;
}

TEST_CASE("A stop lasts what its real transfers cost, no more and no less", "[metrics_invariants]") {
	int checked = 0, withRefusals = 0;
	for (const Scenario& s : kScenarios) {
		if (s.perPassenger <= 0.0) continue;
		Run run = runSimulation(s);
		for (const StopRecord& stop : stopsFrom(*run.logger)) {
			const double due = (stop.enter + stop.leave) * s.perPassenger + s.doorClose;
			const double lasted = stop.depart - stop.arrive;
			CAPTURE(s.floors, s.maxWeight, stop.arrive, stop.enter, stop.leave, stop.passed);
			REQUIRE(lasted >= due - 1e-6);
			if (stop.enter > 0) {
				REQUIRE(lasted == Catch::Approx(due).margin(1e-6));
				++checked;
				if (stop.passed > 0) ++withRefusals;
			}
		}
	}
	// A case that checked nothing would pass for a broken engine.
	REQUIRE(checked > 100);
	REQUIRE(withRefusals > 10);
}

namespace {

// Half the passengers say no once, and then get in. A refusal charged twice
// only shows when the FIRST thing a stop does is refuse somebody, because the
// first transfer takes the stop off the route. A full cabin almost never
// produces that order, and neither does refusing everybody once: a queue is
// oldest first, so whoever was already refused boards ahead of the newcomers.
// Only a cabin with somebody aboard is refused, because that one has to
// leave: refusing a parked, empty one is the known issue below.
class RefusesOnce : public StandardPassengerBehavior {
	public:
		bool willBoard(const Customer& customer, const BoardingContext& cabin) const override {
			if (cabin.currentLoad <= 0.0 || customer.getId() % 2 == 0) return true;
			return !seen.insert(customer.getId()).second;
		}
	private:
		mutable std::set<unsigned long int> seen;
};

} // namespace

TEST_CASE("Known issue: a policy that refuses a parked cabin does not end the run", "[.known_issue]") {
	// Fails: it throws "no elevator can answer a landing call", with two cabins free.
	// The refusing cabin stands idle at the floor, never departs, so its exclusion is
	// never released, and the dispatcher keeps naming it, the nearest. Happens with
	// no door at all. Hidden (the tag starts with ".") until the engine handles it.
	class RefusesEveryoneOnce : public StandardPassengerBehavior {
		public:
			bool willBoard(const Customer& customer, const BoardingContext&) const override {
				return !seen.insert(customer.getId()).second;
			}
		private:
			mutable std::set<unsigned long int> seen;
	};
	Scenario s{/*floors=*/12, /*elevators=*/3, /*customers=*/150, /*rate=*/0.3, /*seed=*/9};
	s.behavior = std::make_shared<RefusesEveryoneOnce>();
	Run run = runSimulation(s);
	REQUIRE(run.logger->arrivalTimeByCustomer.size() == 150);
}

TEST_CASE("A refusal costs no transfer, even when it is the first thing a stop does", "[metrics_invariants]") {
	Scenario s{/*floors=*/12, /*elevators=*/3, /*customers=*/300, /*rate=*/1.0, /*seed=*/9,
		/*doorOpen=*/2.0, /*doorClose=*/3.0, /*perPassenger=*/1.2};
	s.behavior = std::make_shared<RefusesOnce>();
	Run run = runSimulation(s);

	int opensWithRefusal = 0;
	for (const StopRecord& stop : stopsFrom(*run.logger)) {
		const double lasted = stop.depart - stop.arrive;
		const double due = (stop.enter + stop.leave) * s.perPassenger + s.doorClose;
		CAPTURE(stop.arrive, stop.enter, stop.leave, stop.passed);
		REQUIRE(lasted >= due - 1e-6);
		if (stop.enter == 0) continue;
		REQUIRE(lasted == Catch::Approx(due).margin(1e-6));
		if (stop.opensWithRefusal) ++opensWithRefusal;
	}
	REQUIRE(opensWithRefusal > 0);
}

TEST_CASE("Each cabin's timeline alternates DEPART and ARRIVE", "[metrics_invariants]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);
		std::map<int, std::string> last;		// per elevator: the previous trajectory row
		for (const RecordingLogger::LoggedEvent& e : run.logger->events) {
			if (e.type != "ARRIVE" && e.type != "DEPART") continue;
			std::string& previous = last[e.elevatorId];
			INFO("scenario " << s.floors << "/" << s.elevators << "/" << s.customers << " seed " << s.seed
				<< ", elevator " << e.elevatorId << ": " << e.type << " at t=" << e.time
				<< " floor " << e.floor << " after " << (previous.empty() ? "nothing" : previous));
			if (e.type == "ARRIVE") REQUIRE(previous == "DEPART");
			else REQUIRE(previous != "DEPART");
			previous = e.type;
		}
	}
}

// The reassigning scenario in kScenarios above is run through every oracle in
// this file, but only this case asserts that it actually REASSIGNED -
// otherwise its clean pass could mean the group strategy never moved
// anything. It drives the same scenario with its own FloorTargetGroup
// instance so it can read the move counter, and checks both that moves
// really happened and that all 200 customers still reached their destination
// (the invariant a lost-in-transit call would break).
TEST_CASE("A run that reassigns calls mid-flight still serves everyone", "[metrics_invariants]") {
	auto group = std::make_shared<FloorTargetGroup>(3);
	Scenario s{/*floors=*/15, /*elevators=*/3, /*customers=*/200, /*rate=*/1.5, /*seed=*/123};
	s.group = group;

	Run run = runSimulation(s);

	// The strategy really did steer calls to other cabins (not a vacuous pass).
	REQUIRE(group->movesRequested > 0);

	// Every created customer boarded once and alighted once - a call dropped in a
	// reassignment would strand somebody, failing this.
	TimelineJourneys j = journeysFrom(*run.logger);
	REQUIRE(static_cast<int>(j.alightTime.size()) == s.customers);
	REQUIRE(static_cast<int>(j.boardTime.size()) == s.customers);
	for (const auto& entry : j.enterCount) REQUIRE(entry.second == 1);
	for (const auto& entry : j.leaveCount) REQUIRE(entry.second == 1);
}

// ---------------------------------------------------------------------------
// Item #21(a)+(c): the windowed reading and the derived queue, checked against
// an independent recomputation from the timeline, and the invariants that tie
// the two parts together. Run over every scenario above - including the
// capacity and call-reassignment ones, which have the largest drains.
// ---------------------------------------------------------------------------

// The collector partitions its own samples by the boarding/alighting instant;
// this recomputes the partition from the ENTER/LEAVE rows of the timeline, a
// different input. They must agree exactly: the boarding instant the
// collector sees and the ENTER time the timeline records are the same
// ev.getTime(), so there is no boundary ambiguity to absorb.
TEST_CASE("Windowed waiting and journey equal an independent recomputation from the timeline", "[metrics_oracle]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);
		TimelineJourneys j = journeysFrom(*run.logger);
		TimelineWindows w = timelineWindows(*run.logger, j);

		auto cw = run.metrics->getWaitingTimeByWindow();
		auto cj = run.metrics->getJourneyTimeByWindow();
		REQUIRE(cw.has_value());
		REQUIRE(cj.has_value());

		REQUIRE(cw->boundaryTime == Catch::Approx(w.boundary));
		REQUIRE(cw->peak.count == w.waitPeak);
		REQUIRE(cw->drain.count == w.waitDrain);
		REQUIRE(cj->peak.count == w.journeyPeak);
		REQUIRE(cj->drain.count == w.journeyDrain);
		if (w.waitPeak > 0)
			REQUIRE(cw->peak.average == Catch::Approx(w.waitPeakMean).epsilon(1e-9));
		if (w.waitDrain > 0)
			REQUIRE(cw->drain.average == Catch::Approx(w.waitDrainMean).epsilon(1e-9));
	}
}

TEST_CASE("Queue report equals an independent recomputation from the timeline", "[metrics_oracle]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);
		TimelineJourneys j = journeysFrom(*run.logger);
		TimelineWindows w = timelineWindows(*run.logger, j);

		auto q = run.metrics->getQueueReport();
		REQUIRE(q.has_value());
		REQUIRE(q->boundaryTime == Catch::Approx(w.boundary));
		REQUIRE(q->maxQueue == w.maxQueue);
		REQUIRE(q->queueAtArrivalsEnd == w.queueAtEnd);
	}
}

// Properties that carry no expected value: the partition loses nobody, and the
// end-of-arrivals queue IS the waiting drain (every passenger arrives by T_fim,
// so "still waiting at T_fim" and "boarded after T_fim" are the same set). A
// move that lost a sample in the partition, or a queue sweep that double-counted
// an arrival, breaks one of these without any number to update.
TEST_CASE("Windowed partition loses nobody, and the end-of-arrivals queue equals the waiting drain", "[metrics_invariants]") {
	for (const Scenario& s : kScenarios) {
		Run run = runSimulation(s);
		auto cw = run.metrics->getWaitingTimeByWindow();
		auto cj = run.metrics->getJourneyTimeByWindow();
		auto q = run.metrics->getQueueReport();

		REQUIRE(cw->peak.count + cw->drain.count == run.metrics->getWaitingTimeDistribution().count);
		REQUIRE(cj->peak.count + cj->drain.count == run.metrics->getJourneyTimeDistribution().count);

		REQUIRE(q->queueAtArrivalsEnd == cw->drain.count);
		REQUIRE(q->queueAtArrivalsEnd >= 0);
		REQUIRE(q->queueAtArrivalsEnd <= q->maxQueue);
		REQUIRE(q->maxQueue <= s.customers);
		REQUIRE(q->maxQueue >= 1);		// every scenario places at least one call
	}
}

// The regime the windowed reading exists for: a lunch-peak (Cortes 2004 §4.1,
// via examples::LunchPeakTrafficProfile) saturated with a finite cabin, so a
// real backlog forms during the peak and is only worked off in the drain. This
// is where peak and drain diverge - under the stationary default the drain is
// small. It prints the split and asserts the shape: a non-empty drain, the
// queue-at-end equal to the waiting drain count, and the peak being an
// optimistic floor of the global mean.
TEST_CASE("Saturated lunch-peak: a real drain, and the peak is an optimistic floor", "[metrics_invariants][lunchpeak]") {
	Scenario s;
	s.floors = 12;
	s.elevators = 2;
	s.customers = 800;
	s.arrivalRate = 0.0;		// ignored: the profile owns its rate
	s.seed = 2024;
	s.maxWeight = 450.0;		// ~6 passengers at 70 kg mean: a cabin that fills
	// scale concentrates the lunch curve's demand so the 800 arrivals land in a
	// short window the two cabins cannot clear in real time - the saturation the
	// windowed reading exists for.
	s.trafficProfile = std::make_shared<examples::LunchPeakTrafficProfile>(12, /*scale=*/10.0);

	Run run = runSimulation(s);
	auto cw = run.metrics->getWaitingTimeByWindow();
	auto q = run.metrics->getQueueReport();
	REQUIRE(cw.has_value());
	REQUIRE(q.has_value());

	double globalMean = run.metrics->getWaitingTimeDistribution().average;
	std::cout << "\n[lunchpeak] T_fim=" << q->boundaryTime
		<< "  maxQueue=" << q->maxQueue << "  queueAtArrivalsEnd=" << q->queueAtArrivalsEnd
		<< "\n[lunchpeak] waiting: global mean=" << globalMean
		<< "  peak(" << cw->peak.count << ")=" << cw->peak.average
		<< "  drain(" << cw->drain.count << ")=" << cw->drain.average << std::endl;

	REQUIRE(cw->drain.count > 0);					// saturation left a backlog for the drain
	REQUIRE(q->queueAtArrivalsEnd == cw->drain.count);
	REQUIRE(cw->peak.average <= globalMean);		// the peak is an optimistic floor
}
