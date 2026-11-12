#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include "JsonSimulationLogger.h"

namespace openEGCS {

using json = nlohmann::json;

namespace {

// One snapshot block per reported time distribution. The threshold-dependent
// keys are *absent* when no threshold was configured, instead of present as
// zero: "nobody was dissatisfied" and "nobody measured it" are different
// claims, and a reader of the snapshot has to be able to tell them apart.
// That is also why `threshold` is written as null rather than omitted - it
// says explicitly which of the two this is.
json distributionJson(const TimeDistribution& d)
{
	json block = {
		{"count", d.count},
		{"average", d.average},
		{"p50", d.p50},
		{"p90", d.p90},
		{"max", d.max},
		{"threshold", d.threshold.has_value() ? json(d.threshold.value()) : json(nullptr)}
	};

	if (d.threshold.has_value())
	{
		block["dissatisfiedCount"] = d.dissatisfiedCount().value();
		block["dissatisfiedRate"] = d.dissatisfiedRate().has_value()
			? json(d.dissatisfiedRate().value()) : json(nullptr);
		// Which passengers, not just how many: the ids index straight into
		// the "customers" map and the event timeline, so whoever is testing a
		// dispatch algorithm can go from "there are 3" to "these 3, at these
		// floors, at these instants". Service order, which is deterministic.
		block["dissatisfiedCustomers"] = d.dissatisfied;
	}

	return block;
}

} // namespace

void JsonSimulationLogger::setMetadata(const SimulationMetadata& meta)
{
	metadata = meta;
}

void JsonSimulationLogger::logCustomer(const Customer& customer)
{
	std::string direction = (customer.getDir() == UP) ? "UP" : (customer.getDir() == DOWN ? "DOWN" : "DEFAULT");
	customers[customer.getId()] = LoggedCustomer{customer.getAt(), customer.getTo(), direction, customer.getTime()};
}

void JsonSimulationLogger::logEvent(double time, const std::string& type, int elevatorId, unsigned long int customerId, int floor)
{
	events.push_back(LoggedEvent{time, type, elevatorId, customerId, floor});
}

void JsonSimulationLogger::finalize(const IMetricsCollector& metrics, const std::string& filePath)
{
	json root;
	// The snapshot format is documented key by key in docs/snapshot-format.md.
	// Three things a reader must know:
	//
	// - Inside one instant the rows are in physical order: a stop's ARRIVE,
	//   then its transfers (LEAVE, then ENTER/PASS), then the DEPART of the leg
	//   the cabin leaves on.
	// - ARRIVE is the complete list of a cabin's stops, including the rare stop
	//   at which nothing happened (a cabin cannot un-brake, so a stop whose call
	//   another car served is still made). RECALL's `elevatorId` is the cabin
	//   that refused the passenger, not one that is coming, so it must not be fed
	//   into any cabin's trajectory.
	// - `metadata.speed` is the CLI argument as given; the kinematic travel model
	//   ignores it. `oneFloorLegSeconds` is what a time unit means in this run.
	//
	// The schema only grows between versions: a key, once written, keeps its
	// meaning, so a reader written against an earlier version keeps working.
	root["schemaVersion"] = 11;

	if (metadata.has_value())
	{
		json startingPositions = json::array();
		for (int i = 0; i < metadata->elevatorCount; i++) startingPositions.push_back(0);

		root["metadata"] = {
			{"floors", metadata->floors},
			{"elevators", metadata->elevatorCount},
			{"speed", metadata->speed},
			// Which travel model produced these timestamps, and what that model
			// charges for one floor from rest. `speed` does not describe the run -
			// see SimulationMetadata.
			{"travelModel", metadata->travelModel},
			{"oneFloorLegSeconds", metadata->oneFloorLegSeconds},
			// Which demand pattern generated the arrivals.
			{"trafficProfile", metadata->trafficProfile},
			{"arrivalRate", metadata->arrivalRate},
			{"sane", metadata->sane},
			{"customersToSimulate", metadata->customersToSimulate},
			{"seed", metadata->seed.has_value() ? json(metadata->seed.value()) : json(nullptr)},
			{"startingPositions", startingPositions}
		};

		// The run's physics: installation, travel, dwell and energy model.
		auto optional = [](const std::optional<double>& v) { return v.has_value() ? json(*v) : json(nullptr); };
		const InstallationDescription& inst = metadata->installation;
		root["metadata"]["installation"] = {
			{"ratedLoadKg", optional(inst.ratedLoadKg)},
			{"bypassThresholdKg", optional(inst.bypassThresholdKg)},
			{"meanPassengerKg", inst.meanPassengerKg},
			{"stddevPassengerKg", inst.stddevPassengerKg}
		};
		const TravelDescription& tr = metadata->travel;
		json travel = {{"model", tr.model}};
		if (tr.secondsPerFloor) travel["secondsPerFloor"] = *tr.secondsPerFloor;
		if (tr.floorHeightM) travel["floorHeightM"] = *tr.floorHeightM;
		if (tr.ratedSpeedMs) travel["ratedSpeedMs"] = *tr.ratedSpeedMs;
		if (tr.accelMs2) travel["accelMs2"] = *tr.accelMs2;
		if (tr.jerkMs3) travel["jerkMs3"] = *tr.jerkMs3;
		root["metadata"]["travel"] = travel;
		const DwellDescription& dw = metadata->dwell;
		json dwell = {
			{"model", dw.model},
			{"doorOpenS", dw.doorOpenS},
			{"doorCloseS", dw.doorCloseS},
			{"perPassengerS", dw.perPassengerS},
			{"hold", dw.hold}
		};
		if (dw.hold != "none") {
			dwell["carCallHoldS"] = dw.carCallHoldS;
			dwell["landingCallHoldS"] = dw.landingCallHoldS;
		}
		if (dw.hold == "detector") dwell["detectorHoldS"] = dw.detectorHoldS;
		root["metadata"]["dwell"] = dwell;
		if (metadata->energy.has_value()) {
			const EnergyModelDescription& e = *metadata->energy;
			root["metadata"]["energy"] = {
				{"model", e.model},
				{"floorHeightM", e.floorHeightM},
				{"ratedSpeedMs", e.dynamics.ratedSpeedMs},
				{"accelMs2", e.dynamics.accelMs2},
				{"jerkMs3", e.dynamics.jerkMs3},
				{"carMassKg", e.motor.carMassKg},
				{"counterweightMassKg", e.motor.counterweightMassKg},
				{"gearEfficiency", e.motor.gearEfficiency},
				{"driveSheaveDiameterM", e.motor.driveSheaveDiameterM},
				{"gearRatio", e.motor.gearRatio},
				{"ropingRatio", e.motor.ropingRatio},
				{"armatureResistanceOhm", e.motor.armatureResistanceOhm},
				{"motorConstant", e.motor.motorConstant},
				{"residualInertiaKgM2", e.motor.residualInertiaKgM2},
				{"regenerative", e.motor.regenerative},
				{"idlePowerWatts", e.motor.idlePowerWatts}
			};
		} else {
			root["metadata"]["energy"] = {{"model", "none"}};
		}
	}

	json customersJson = json::object();
	for (const auto& kv : customers)
	{
		customersJson[std::to_string(kv.first)] = {
			{"atFloor", kv.second.atFloor},
			{"toFloor", kv.second.toFloor},
			{"direction", kv.second.direction},
			{"arrivalTime", kv.second.arrivalTime}
		};
	}
	root["customers"] = customersJson;

	json eventsJson = json::array();
	for (const auto& e : events)
	{
		eventsJson.push_back({
			{"time", e.time},
			{"type", e.type},
			{"elevatorId", e.elevatorId},
			{"customerId", e.customerId},
			{"floor", e.floor}
		});
	}
	root["events"] = eventsJson;

	root["metrics"] = {
		{"averageWaitingTime", metrics.getAverageWaitingTime()},
		{"averageTravelTime", metrics.getAverageTravelTime()},
		{"totalEnergyConsumption", metrics.getTotalEnergyConsumption()},
		// averageWaitingTime is kept as its own key, unchanged, so a v3
		// consumer keeps working; waitingTime.average is the same number.
		{"waitingTime", distributionJson(metrics.getWaitingTimeDistribution())},
		{"journeyTime", distributionJson(metrics.getJourneyTimeDistribution())}
	};

	// Omitted entirely, rather than written as zeros, when the collector does
	// not measure this: "no cabin left anyone behind" and "nobody counted" must
	// not look alike, which is the same rule the dissatisfaction verdict
	// follows.
	//
	// `refusedAtDoor` corresponds one-to-one with the PASS rows in the
	// timeline; `reallocated` has no rows at all, by design - see
	// PassByReport.
	std::optional<PassByReport> passedBy = metrics.getPassByReport();
	if (passedBy.has_value())
	{
		auto tallyJson = [](const PassByTally& t) {
			json passengers = json::array();
			for (unsigned long int id : t.passengers) passengers.push_back(id);
			return json{{"events", t.events}, {"passengers", passengers}};
		};
		root["metrics"]["passedBy"] = {
			{"refusedAtDoor", tallyJson(passedBy->refusedAtDoor)},
			{"reallocated", tallyJson(passedBy->reallocated)}
		};
	}

	// Per-window reading and derived queue, omitted when not measured for the
	// same reason as passedBy above. The window blocks reuse distributionJson,
	// so each half carries the same shape (and the same dissatisfaction
	// verdict) as the global distribution.
	std::optional<WindowedTimeReport> waitingByWindow = metrics.getWaitingTimeByWindow();
	if (waitingByWindow.has_value())
	{
		auto windowJson = [](const WindowedTimeReport& w) {
			return json{
				{"peak", distributionJson(w.peak)},
				{"drain", distributionJson(w.drain)},
				{"boundaryTime", w.boundaryTime}
			};
		};
		root["metrics"]["waitingTimeByWindow"] = windowJson(waitingByWindow.value());
		std::optional<WindowedTimeReport> journeyByWindow = metrics.getJourneyTimeByWindow();
		root["metrics"]["journeyTimeByWindow"] = windowJson(journeyByWindow.value());
	}

	std::optional<QueueReport> queue = metrics.getQueueReport();
	if (queue.has_value())
	{
		root["metrics"]["queue"] = {
			{"max", queue->maxQueue},
			{"atArrivalsEnd", queue->queueAtArrivalsEnd},
			{"boundaryTime", queue->boundaryTime}
		};
	}

	std::ofstream out(filePath);
	if (!out.is_open())
	{
		std::cerr << "ERROR: could not open " << filePath << " for writing snapshot." << std::endl;
		return;
	}
	out << root.dump(2);
	out.close();
}

} // namespace openEGCS
