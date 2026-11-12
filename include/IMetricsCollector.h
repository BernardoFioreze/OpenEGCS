/*
IMetricsCollector.h

Decoupled metrics collection. ElevatorSystem/Simulator report events
through this interface instead of accumulating statistics as scattered
fields inside Elevator. This makes metrics independently
testable/replaceable (e.g. someone could plug a collector that also
reports p95 waiting time, or streams metrics to an external system).
*/

#ifndef __IMETRICSCOLLECTOR__
#define __IMETRICSCOLLECTOR__

#include <string>
#include <optional>
#include <vector>
#include "Customer.h"
#include "LiftMotorModel.h"

namespace openEGCS {

// What this project can honestly say about one per-passenger time: a
// distribution, not a mean. The mean alone hides the tail, and the tail is
// where a dispatch algorithm that trades the worst case for the average shows
// up - which is exactly what a benchmarking platform should expose.
//
// The quantile pair (p50/p90) and the "above a threshold" count are the shape
// the elevator-traffic literature reports service quality in: Barney &
// Al-Sharif, Elevator Traffic Handbook, 2nd ed., table 6.5 (p. 122) grades an
// installation by the percentage of calls answered within 30 s and 60 s and by
// the time to answer 50% and 90% of them.
struct TimeDistribution {
	int count = 0;					// how many passengers the figures below are over
	double average = 0.0;
	double p50 = 0.0;				// nearest-rank quantile (inverse CDF), no interpolation
	double p90 = 0.0;
	double max = 0.0;				// the worst thing that happened to anybody

	// nullopt = no threshold was configured for this scenario, which is NOT
	// the same claim as "nobody was dissatisfied". The project ships no
	// default threshold: whatever default it shipped would silently become the
	// figure every benchmark is read against.
	std::optional<double> threshold;

	// Ids of the passengers strictly above `threshold`, in the order the
	// simulation served them (deterministic; never an unordered_map's order).
	// Empty whenever `threshold` is nullopt. Kept as ids rather than a bare
	// count because "are there any?" and "which ones?" are different
	// questions, and only the second one lets someone debug their algorithm.
	std::vector<unsigned long int> dissatisfied;

	// nullopt when no threshold was configured; 0 when one was and nobody
	// exceeded it. The whole point of the optional is to keep those apart.
	std::optional<int> dissatisfiedCount() const
	{
		if (!threshold.has_value()) return std::nullopt;
		return static_cast<int>(dissatisfied.size());
	}

	// Fraction of `count`, in [0, 1]. nullopt when no threshold was
	// configured, or when nobody was served at all (no population to be a
	// fraction of).
	std::optional<double> dissatisfiedRate() const
	{
		if (!threshold.has_value() || count == 0) return std::nullopt;
		return static_cast<double>(dissatisfied.size()) / count;
	}
};

// The same per-passenger time distribution as TimeDistribution, split in two by
// the instant the run stopped receiving arrivals. A run ends once all of its
// <Customers to simulate> have reached their destination, so it has an arrival
// window - demand rising and falling - followed by a drain, during which no new
// call is placed and the building only empties. The two regimes are not
// comparable: the drain's long waits dominate a global average, so the
// literature reports the peak alone. `peak` is the sub-population whose metric
// closed (boarding for waiting, alighting for journey) at or before
// `boundaryTime` (the last arrival instant); `drain` is the rest.
//
// Read them TOGETHER, never `peak` alone: by partitioning on the terminal
// event, `peak` excludes exactly the passengers who waited through the window
// and were only served as the building drained, so it is an optimistic floor.
// `boundaryTime` is derived, so this is reported for every run, with no
// threshold to configure.
struct WindowedTimeReport {
	TimeDistribution peak;		// terminal instant <= boundaryTime
	TimeDistribution drain;		// terminal instant >  boundaryTime
	double boundaryTime = 0.0;	// the last arrival instant
};

// What a run can say about the backlog it carried, derived from the same
// per-passenger instants as WindowedTimeReport. The "queue" is the number of
// passengers who have arrived but not yet boarded, summed over all floors - a
// group-level saturation signal, distinct from ElevatorSystem's instantaneous
// getUnallocated* counters (which only exist when a cabin weight is configured
// and count calls/people with no cabin assigned).
struct QueueReport {
	int maxQueue = 0;				// peak of arrivals(t) - boardings(t) over the run
	int queueAtArrivalsEnd = 0;		// that count evaluated at boundaryTime (= the drain
									// count of the waiting distribution, by construction)
	double boundaryTime = 0.0;		// the last arrival instant
};

// One kind of thing that happens to somebody still waiting because the lift
// that was coming for them is full: how many times it happened, and to whom.
struct PassByTally {
	int events = 0;		// occurrences, which is not the same as people
	// Ids of everybody it happened to at least once, in the order it first
	// happened to them (deterministic; never an unordered_map's order).
	std::vector<unsigned long int> passengers;
};

// What a run can say about demand the lifts did not take on the spot, because a
// cabin was full. With unlimited cabins every call is absorbed by the first car
// that opens its doors, which makes saturation invisible from inside the model;
// with a rated load configured, this is where it shows.
//
// The two tallies are kept apart because only one of them is something the
// passenger can see happen:
//
//  - `refusedAtDoor`: a cabin arrived at their floor, opened, and there was no
//    room. The passenger watched a full car leave without them. Also in the
//    snapshot timeline, as a PASS event, because it is the only record of a
//    stop where nobody got in or out.
//  - `reallocated`: the cabin that had been assigned to their call filled up
//    somewhere else and gave the call back to the dispatcher, so it will not
//    be coming. Nothing happens at their floor at that instant - no car
//    arrives, nothing is visible - which is why it is NOT a timeline event.
//    It counts people: a call is one per (floor, direction), so one call
//    handed back charges everybody waiting behind it.
//
// In both cases the landing call stays registered - "landing calls should not
// be cancelled" (Barney & Al-Sharif 10.8.3) - so neither is the end of a
// passenger's journey, and their waiting time runs throughout. Both are
// therefore *causes* of the waiting distribution's tail, not a separate
// population.
struct PassByReport {
	PassByTally refusedAtDoor;
	PassByTally reallocated;
};

// The per-scenario tolerance thresholds a passenger is judged against. Both
// optional: a run configures either, both, or neither.
//
// `waiting` is the primary criterion. `journey` (arrival to alighting) is robust
// to anything that moves a passenger's time between waiting and riding, because
// it adds the two. Travel time has no threshold here: the literature's travel
// tolerance is defined by intolerance *of stops* (Barney & Al-Sharif section
// 4.3.2; Strakosch's figures in section 6.7.1 stretch with how many passengers
// transfer per stop), which a single figure does not capture.
struct ServiceThresholds {
	std::optional<double> waiting;		// arrival -> boarding
	std::optional<double> journey;		// arrival -> alighting
};

// The energy model a collector prices legs with, for the snapshot's metadata:
// the motion profile and floor height it integrates over, which need not be the
// travel model's, and every motor and installation parameter. Without it two
// snapshots with different motors report energies nobody can compare.
struct EnergyModelDescription {
	std::string model;			// e.g. "dc-static-converter" (Peters 1998, ch. 7)
	double floorHeightM = 0.0;
	LiftDynamics dynamics{};
	LiftMotorParams motor{};
};

class IMetricsCollector {
	public:
		virtual ~IMetricsCollector() = default;

		// A customer has arrived at a floor and placed a call.
		virtual void onCustomerArrived(const Customer& customer) = 0;

		// A customer has boarded elevatorId at `time` (waiting time = time - customer.getTime()).
		virtual void onCustomerBoarded(const Customer& customer, int elevatorId, double time) = 0;

		// A customer has alighted from elevatorId at `time`. Travel time is
		// computed internally by the collector as time - boardingTime
		// (boardingTime was recorded by a previous onCustomerBoarded call
		// for the same customer id).
		virtual void onCustomerAlighted(const Customer& customer, int elevatorId, double time) = 0;

		// `elevatorId` arrived at `customer`'s floor, opened its doors and could
		// not take them. Their landing call stays registered, so this is not the
		// end of their journey - onCustomerBoarded still comes, later - and it
		// can happen to one passenger more than once.
		//
		// This and onHallCallReallocated below have a default (empty)
		// implementation rather than being pure virtual, so a collector that
		// does not count them still compiles. Such a collector silently does not
		// count them, which is why getPassByReport() returns an optional: "nobody
		// was left behind" and "nobody counted" cannot be mistaken for each other
		// (the same policy as the dissatisfaction threshold above).
		virtual void onCustomerPassedBy(const Customer& customer, int elevatorId, double time) { (void)customer; (void)elevatorId; (void)time; }

		// `elevatorId` had been allocated `customer`'s landing call and filled
		// up before reaching them, so the call went back to the dispatcher to
		// be given to another cabin. Nothing is visible at the passenger's
		// floor when this fires - no car arrives - and it is deliberately not
		// in the snapshot timeline for that reason; see PassByReport.
		virtual void onHallCallReallocated(const Customer& customer, int elevatorId, double time) { (void)customer; (void)elevatorId; (void)time; }

		// elevatorId has moved `distance` floors (can be fractional).
		// Deprecated: this was the sole input to an old "distance x constant"
		// energy proxy, superseded by onElevatorDeparted's electromechanical
		// model. Kept for interface compatibility; StandardMetricsCollector
		// does not fold it into getTotalEnergyConsumption()/getEnergyConsumption().
		virtual void onElevatorMoved(int elevatorId, double distance) = 0;

		// elevatorId completed a continuous run of `floorsTravelled` floors
		// (signed: positive = up, negative = down) that began at simulation
		// time `time`, carrying `currentLoad` kg of passengers. Used to derive
		// a physically grounded energy-consumption estimate (load torque,
		// inertia, DC motor model - see LiftMotorModel.h).
		//
		// Two things about WHEN this fires matter to any implementation:
		//
		// It is called once the run is OVER, not when it starts, even though
		// `time` is still the instant it started - the departure is what the
		// trip is identified by, and the idle-power accounting measures from
		// it. Charging at departure would charge the distance the cabin
		// INTENDED to cover; a nearer call inserted mid-flight makes it stop
		// early, and the stretch it never flew would be charged anyway.
		//
		// Consequently the call arrives one leg behind: at the moment a cabin
		// leaves a stop, the charge you receive is for the leg that brought it
		// there. The last leg of each elevator is flushed at the end of the
		// run. An implementation that wants the departure instants themselves
		// should read the DEPART events of the snapshot, which are emitted when
		// the cabin leaves.
		virtual void onElevatorDeparted(int elevatorId, double floorsTravelled, double currentLoad, double time) = 0;

		virtual void onSimulationEnd(double totalTime) = 0;

		virtual double getAverageWaitingTime() const = 0;
		virtual double getAverageTravelTime() const = 0;

		// The same waiting time as getAverageWaitingTime(), plus its spread,
		// its worst case and (when a threshold is configured) who was
		// dissatisfied. Waiting is what the literature calls the prime
		// psychological constraint; see TimeDistribution above.
		virtual TimeDistribution getWaitingTimeDistribution() const = 0;

		// Arrival to alighting - the total time a passenger spent on the lift
		// system. In this simulator journey == waiting + travel exactly, with
		// no period missing, because a boarding is one instant: the wait ends
		// and the ride starts at the same timestamp.
		virtual TimeDistribution getJourneyTimeDistribution() const = 0;

		// The waiting and journey distributions split into the arrival window
		// and the drain that follows it (see WindowedTimeReport). Default
		// nullopt - "this collector does not measure it", NOT "the drain was
		// empty", which is a WindowedTimeReport whose `drain` has count 0. The
		// snapshot omits the key for nullopt, the same policy as
		// getPassByReport() below.
		virtual std::optional<WindowedTimeReport> getWaitingTimeByWindow() const { return std::nullopt; }
		virtual std::optional<WindowedTimeReport> getJourneyTimeByWindow() const { return std::nullopt; }

		// The run's peak backlog and the queue still waiting when arrivals
		// ended (see QueueReport). Default nullopt, same policy as above.
		virtual std::optional<QueueReport> getQueueReport() const { return std::nullopt; }

		// Who the cabins left behind, and how often. nullopt means this
		// collector does not measure it - NOT that nobody was left behind. An
		// implementation that overrides onCustomerPassedBy/onHallCallReallocated
		// must return a value here, or the snapshot will omit the figures for a
		// run that did measure them.
		virtual std::optional<PassByReport> getPassByReport() const { return std::nullopt; }

		virtual double getTotalEnergyConsumption() const = 0;
		virtual double getEnergyConsumption(int elevatorId) const = 0;

		virtual void printSummary() const = 0;

		// The energy model behind getTotalEnergyConsumption(), for the snapshot.
		// std::nullopt for a collector that does not price energy with a model,
		// which the snapshot writes as "none" rather than as numbers it does not
		// have.
		virtual std::optional<EnergyModelDescription> energyModel() const = 0;
};

} // namespace openEGCS

#endif
