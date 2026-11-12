/*
StandardMetricsCollector.h

Default IMetricsCollector implementation. Tracks:
  - average customer waiting time (arrival -> boarding)
  - average customer travel time (boarding -> alighting)
  - the waiting and journey (arrival -> alighting) time distributions:
    mean, p50, p90, worst case, and who was dissatisfied when the scenario
    configures a tolerance threshold. Dissatisfaction is a label applied
    after the fact - no passenger ever refuses to board or gives up, so
    this costs the simulation nothing and changes no other number.
  - the same distributions split into the arrival window and the drain, and
    the run's queue of passengers waiting.
  - total/per-elevator energy consumption, modelled per trip
    (onElevatorDeparted) via the electromechanical model in
    LiftMotorModel.h (load torque, inertia, DC motor, power integrated
    over a jerk-limited velocity profile from LiftKinematics.h).

  The old "distance x constant" proxy (onElevatorMoved/energyPerFloor) is
  kept for interface compatibility but is not part of
  getTotalEnergyConsumption()/getEnergyConsumption() - summing it with the
  motor model would double-count the cruise-speed traction work, which is
  already inside the motor model's load torque/power terms.
*/

#ifndef __STANDARDMETRICSCOLLECTOR__
#define __STANDARDMETRICSCOLLECTOR__

#include <unordered_map>
#include <utility>
#include <vector>
#include "IMetricsCollector.h"
#include "LiftKinematics.h"
#include "LiftMotorModel.h"

namespace openEGCS {

class StandardMetricsCollector : public IMetricsCollector {
	private:
		double totalWaitingTime = 0;
		int customersBoarded = 0;

		double totalTravelTime = 0;
		int customersAlighted = 0;

		std::unordered_map<unsigned long int, double> boardingTimeByCustomer;

		// One served passenger's contribution to a metric, in the order the
		// simulation served them, so every figure derived from them is
		// deterministic - an unordered_map here would make the dissatisfied
		// list's order depend on hashing, and byte-identity for a seed is a hard
		// invariant. Kept as samples rather than running sums because a quantile
		// and a worst case cannot be accumulated.
		// `instant` is the absolute time of the event that closes the metric
		// (boarding for waiting, alighting for journey); it is what the windowed
		// reports partition on.
		struct Sample {
			unsigned long int id;
			double value;		// the measured duration
			double instant;		// absolute time of the terminal event
		};
		std::vector<Sample> waitingSamples;		// arrival -> boarding
		std::vector<Sample> journeySamples;		// arrival -> alighting

		// Every arrival instant, captured in onCustomerArrived - which fires
		// exactly once per passenger, never on a RECALL (ElevatorSystem::giveEvent),
		// so these are first presses. Stored exactly rather than reconstructed as
		// boarding - waiting, so the queue sweep compares real arrival instants
		// against the boundary with no rounding.
		std::vector<double> arrivalInstants;

		// The last arrival instant, max over arrivalInstants: the boundary between
		// the arrival window and the drain. Derived here with no coupling to
		// Simulator and no configured constant.
		double lastArrivalTime = 0.0;

		// Builds the reported distribution from one sample set: mean, p50/p90
		// by nearest rank (index = ceil(p * n) - 1 over the sorted values, no
		// interpolation, so every figure reported is a time some passenger
		// actually experienced), worst case, and the ids strictly above the
		// threshold. Sorts a copy per call - the getters are const and are
		// called a handful of times at the end of a run, not in the hot path.
		TimeDistribution distributionOf(const std::vector<Sample>& samples,
			const std::optional<double>& threshold) const;

		// Partitions `samples` by `instant <= lastArrivalTime` (peak) vs. the
		// rest (drain) and runs distributionOf on each. The threshold is passed
		// through so both halves carry the same dissatisfaction verdict as the
		// global distribution.
		WindowedTimeReport windowedDistribution(const std::vector<Sample>& samples,
			const std::optional<double>& threshold) const;

		// Prints one distribution block of printSummary(). Static because it
		// reads nothing but its argument, and the "no threshold configured"
		// wording lives here so both blocks say it identically.
		static void printDistribution(const std::string& label, const TimeDistribution& d);

		std::unordered_map<int, double> energyByElevator;			// deprecated proxy, see header comment above
		double energyPerFloor;										// Consumption unit per floor traveled (deprecated proxy only).

		std::unordered_map<int, double> motorEnergyByElevator;		// Joules, from the electromechanical model.
		std::unordered_map<int, double> arrivalTimeByElevator;		// end time of each elevator's last trip, for idle-power accounting.

		double floorHeightM;
		LiftDynamics dynamics;
		LiftMotorParams motorParams;

		// Declared here, after the energy-model fields, because the constructor
		// initialises it last: member initialisation follows *declaration*
		// order, and an init-list that disagrees with it is a -Wreorder warning
		// today and a silent misinitialisation the day one member reads another.
		ServiceThresholds thresholds;

		double simulationEndTime = 0;

		// Pass-bys and reallocations: the count of occurrences and the people each
		// happened to, first-occurrence order. A vector and a linear scan rather
		// than a set, for the same reason the samples above are a vector:
		// iteration order has to be a function of the simulation, not of a hash.
		// The scan is over the distinct passengers affected at least once, which
		// is bounded by the population.
		PassByTally refusedAtDoor;
		PassByTally reallocated;

		// Counts one occurrence against `tally` and records `customerId` if it
		// is the first time for them. Shared by the two hooks because the
		// bookkeeping is identical and only the meaning differs.
		static void tally(PassByTally& tally, unsigned long int customerId);

	public:
		// `floorHeightM`/`dynamics`/`motorParams` feed the electromechanical
		// energy model (see LiftKinematics.h/LiftMotorModel.h); `energy_per_floor`
		// only affects the deprecated onElevatorMoved proxy. `service_thresholds`
		// are the per-scenario tolerances a passenger is judged dissatisfied
		// against - both unset by default, which reports the distributions
		// without any dissatisfaction verdict at all.
		explicit StandardMetricsCollector(double energy_per_floor = 1.0,
			double floorHeightM = 3.5,
			LiftDynamics dynamics = defaultDynamicsFor(20.0),
			LiftMotorParams motorParams = LiftMotorParams(),
			ServiceThresholds service_thresholds = ServiceThresholds());

		void onCustomerArrived(const Customer& customer) override;
		void onCustomerBoarded(const Customer& customer, int elevatorId, double time) override;
		void onCustomerAlighted(const Customer& customer, int elevatorId, double time) override;
		void onCustomerPassedBy(const Customer& customer, int elevatorId, double time) override;
		void onHallCallReallocated(const Customer& customer, int elevatorId, double time) override;
		void onElevatorMoved(int elevatorId, double distance) override;
		void onElevatorDeparted(int elevatorId, double floorsTravelled, double currentLoad, double time) override;
		void onSimulationEnd(double totalTime) override;

		double getAverageWaitingTime() const override;
		double getAverageTravelTime() const override;
		TimeDistribution getWaitingTimeDistribution() const override;
		TimeDistribution getJourneyTimeDistribution() const override;
		// Always a value for this collector. The drain half can have count 0 (a
		// run whose last passenger boarded before the last arrival), which is a
		// WindowedTimeReport, not nullopt.
		std::optional<WindowedTimeReport> getWaitingTimeByWindow() const override;
		std::optional<WindowedTimeReport> getJourneyTimeByWindow() const override;
		std::optional<QueueReport> getQueueReport() const override;
		// Always a value, never nullopt: this collector does measure both
		// tallies, so a run that had none reports zeros rather than
		// "not measured".
		std::optional<PassByReport> getPassByReport() const override;
		double getTotalEnergyConsumption() const override;
		double getEnergyConsumption(int elevatorId) const override;

		void printSummary() const override;
		// "dc-static-converter" with the floor height, dynamics and motor
		// parameters this collector was built with.
		std::optional<EnergyModelDescription> energyModel() const override;
};

} // namespace openEGCS

#endif
