/*
ISimulationLogger.h

Structured event logging / final snapshot generation (the only file output
of a run). Implementations record every relevant event during the run and,
at the end, dump a full snapshot (event timeline + aggregated metrics) to
disk for later analysis / comparison between dispatch algorithms.
*/

#ifndef __ISIMULATIONLOGGER__
#define __ISIMULATIONLOGGER__

#include <string>
#include <optional>
#include <cstdint>
#include "IMetricsCollector.h"
#include "IDwellTimeStrategy.h"
#include "ITravelTimeStrategy.h"
#include "Customer.h"

namespace openEGCS {

// The installation parameters that change a run's timeline and are not a
// strategy's to describe.
struct InstallationDescription {
	std::optional<double> ratedLoadKg;			// absent: unlimited cabin
	std::optional<double> bypassThresholdKg;	// absent with an unlimited cabin
	double meanPassengerKg = 0.0;				// Normal(mean, stddev) passenger weights
	double stddevPassengerKg = 0.0;
};

// Static configuration of the simulation run, recorded once so a downstream
// reader can reconstruct the scenario (building size, number of elevators,
// physics, demand) from the snapshot file alone.
struct SimulationMetadata {
	int floors;
	int elevatorCount;
	// The <Elevator speed> CLI argument, recorded as given. The default
	// (kinematic) travel model ignores it, so it does not describe the run;
	// it is kept for readers that expect the key. Read `oneFloorLegSeconds`,
	// not this.
	float speed;
	double arrivalRate;
	bool sane;
	int customersToSimulate;
	std::optional<uint64_t> seed;		// Empty when the run was not seeded (non-deterministic).

	// Which travel model produced this run's timestamps
	// (ITravelTimeStrategy::name(): "kinematic" or "linear"), and that model's
	// own answer for one floor from rest.
	//
	// `oneFloorLegSeconds` says what a time unit means in this run. For the
	// linear model it equals `speed`; for the kinematic one it is the real cost
	// of a one-floor hop including starting and stopping (4.95 s for a 10-floor
	// building's defaults). Two snapshots are comparable on time only if these
	// two fields match.
	std::string travelModel;
	double oneFloorLegSeconds = 0;

	// Which demand pattern generated this run's arrivals
	// (ITrafficProfile::name(), e.g. "stationary-uniform", "down-peak (Barney)",
	// "lunch-peak (Cortes 2004)"). Two snapshots are comparable on time only
	// when this AND the travel fields match: the same clock is a precondition,
	// but a lunch-peak and a uniform run are not the same scenario even on it.
	std::string trafficProfile;

	// What the rest of the run's physics was. The same command with a zero
	// door and with the default one produce very different times, so the
	// snapshot has to say which. Each is asked of the component that holds it -
	// the resolved dwell, the travel model, the collector's energy model - so
	// the file cannot disagree with the engine. Two snapshots are comparable on
	// time when travel, dwell and installation match, and on energy when the
	// energy model matches too.
	InstallationDescription installation;
	TravelDescription travel;
	DwellDescription dwell;
	std::optional<EnergyModelDescription> energy;	// absent: the collector models no energy
};

class ISimulationLogger {
	public:
		virtual ~ISimulationLogger() = default;

		// Records the static configuration of the run. Called once, before
		// any events are logged.
		virtual void setMetadata(const SimulationMetadata& metadata) = 0;

		// Records a customer's origin/destination/arrival time, keyed by
		// customer id. Called once per customer, when they are generated.
		virtual void logCustomer(const Customer& customer) = 0;

		// Records a single occurred event. `type` is one of:
		//   "CUST_CALL"  a customer arrives at a landing and calls
		//   "RECALL"     a customer refused at an open door calls again;
		//                `elevatorId` is the cabin that refused them
		//   "ARRIVE"     a cabin reaches one stop of its route; dated when its doors
		//                have opened (IDwellTimeStrategy::getDoorOpenTime)
		//   "ENTER"      a customer boards (same instant as its stop's ARRIVE)
		//   "LEAVE"      a customer alights
		//   "PASS"       a customer could not board (cabin full, or they declined)
		//   "DEPART"     a cabin starts moving towards its next stop; `floor`
		//                is the floor it departs from, `customerId` is 0
		virtual void logEvent(double time, const std::string& type, int elevatorId, unsigned long int customerId, int floor) = 0;

		// Writes the full snapshot (metadata + customers + all logged events
		// + metrics summary) to filePath.
		virtual void finalize(const IMetricsCollector& metrics, const std::string& filePath) = 0;
};

} // namespace openEGCS

#endif
