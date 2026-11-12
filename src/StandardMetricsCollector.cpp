#include <algorithm>
#include <cmath>
#include <iostream>
#include "StandardMetricsCollector.h"

namespace openEGCS {

StandardMetricsCollector::StandardMetricsCollector(double energy_per_floor,
	double floorHeightM, LiftDynamics dynamics, LiftMotorParams motorParams,
	ServiceThresholds service_thresholds)
	: energyPerFloor(energy_per_floor), floorHeightM(floorHeightM),
	  dynamics(dynamics), motorParams(motorParams), thresholds(service_thresholds)
{
}

void StandardMetricsCollector::onCustomerArrived(const Customer& customer)
{
	// Record the arrival instant and keep the last arrival as the running
	// maximum of them. This hook fires exactly once per passenger - a passenger
	// left behind who re-presses the button is a RECALL, which does NOT call
	// this (ElevatorSystem::giveEvent) - so arrivalInstants holds one first
	// press per passenger and the maximum is the true last arrival.
	double arrival = customer.getTime();
	arrivalInstants.push_back(arrival);
	if (arrival > lastArrivalTime) lastArrivalTime = arrival;
}

void StandardMetricsCollector::onCustomerBoarded(const Customer& customer, int /*elevatorId*/, double time)
{
	totalWaitingTime += (time - customer.getTime());
	customersBoarded++;
	boardingTimeByCustomer[customer.getId()] = time;
	// Definition note (Barney & Al-Sharif, definition 6.6, p. 119): the book
	// ends the wait when the lift *begins to open* its doors. This hook fires
	// on the ENTER record, which Elevator::getStopEventTime dates at
	// arrival + getDoorOpenTime, so what is recorded here is the wait up to
	// the passenger actually stepping in - the book's figure plus the door
	// opening. The difference is exactly getDoorOpenTime per passenger (zero
	// only with an instantaneous door), and any comparison against the
	// literature's 20/25/30 s has to account for it.
	waitingSamples.push_back({customer.getId(), time - customer.getTime(), time});
}

void StandardMetricsCollector::onCustomerAlighted(const Customer& customer, int /*elevatorId*/, double time)
{
	auto it = boardingTimeByCustomer.find(customer.getId());
	if (it != boardingTimeByCustomer.end())
	{
		totalTravelTime += (time - it->second);
		customersAlighted++;
		boardingTimeByCustomer.erase(it);
		// Journey is measured from the customer's own arrival, not from the
		// boarding instant, so it is waiting + travel with nothing missing
		// between the two. Recorded in the same branch as travel time on
		// purpose: the two populations must not be able to drift apart.
		journeySamples.push_back({customer.getId(), time - customer.getTime(), time});
	}
}

void StandardMetricsCollector::tally(PassByTally& t, unsigned long int customerId)
{
	// Occurrences and people are counted separately on purpose: the same
	// passenger watching a second full car go by is a second event and not a
	// second person (see PassByReport).
	t.events++;
	for (std::vector<unsigned long int>::const_iterator it = t.passengers.begin() ;
		it != t.passengers.end() ; ++it)
	{
		if (*it == customerId) return;
	}
	t.passengers.push_back(customerId);
}

void StandardMetricsCollector::onCustomerPassedBy(const Customer& customer, int elevatorId, double time)
{
	// Nothing else is recorded here - the passenger's waiting time keeps
	// running and is still closed by onCustomerBoarded, because watching a full
	// car leave is not the end of their journey.
	(void)elevatorId;
	(void)time;
	tally(refusedAtDoor, customer.getId());
}

void StandardMetricsCollector::onHallCallReallocated(const Customer& customer, int elevatorId, double time)
{
	(void)elevatorId;
	(void)time;
	tally(reallocated, customer.getId());
}

void StandardMetricsCollector::onElevatorMoved(int elevatorId, double distance)
{
	// Deprecated proxy, kept for interface compatibility only - see the
	// header comment. Not part of getTotalEnergyConsumption()/
	// getEnergyConsumption().
	energyByElevator[elevatorId] += std::abs(distance) * energyPerFloor;
}

void StandardMetricsCollector::onElevatorDeparted(int elevatorId, double floorsTravelled, double currentLoad, double time)
{
	double signedDistanceM = floorsTravelled * floorHeightM;

	// Idle-power accounting (plan §2.6): add idlePowerWatts * (time spent
	// stationary since this elevator's previous trip ended). With the
	// default idlePowerWatts = 0 this contributes nothing. The very first
	// departure of an elevator has no recorded prior arrival, so no idle
	// time is charged before it.
	auto arrivalIt = arrivalTimeByElevator.find(elevatorId);
	if (arrivalIt != arrivalTimeByElevator.end() && motorParams.idlePowerWatts != 0.0)
	{
		double idleDurationS = time - arrivalIt->second;
		if (idleDurationS > 0.0)
		{
			motorEnergyByElevator[elevatorId] += motorParams.idlePowerWatts * idleDurationS;
		}
	}

	motorEnergyByElevator[elevatorId] += tripEnergyJ(signedDistanceM, currentLoad, dynamics, motorParams);

	KinematicProfile profile = buildProfile(signedDistanceM, dynamics.ratedSpeedMs, dynamics.accelMs2, dynamics.jerkMs3);
	arrivalTimeByElevator[elevatorId] = time + profile.totalTimeS;
}

void StandardMetricsCollector::onSimulationEnd(double totalTime)
{
	simulationEndTime = totalTime;
}

double StandardMetricsCollector::getAverageWaitingTime() const
{
	return customersBoarded == 0 ? 0.0 : totalWaitingTime / customersBoarded;
}

double StandardMetricsCollector::getAverageTravelTime() const
{
	return customersAlighted == 0 ? 0.0 : totalTravelTime / customersAlighted;
}

TimeDistribution StandardMetricsCollector::distributionOf(const std::vector<Sample>& samples,
	const std::optional<double>& threshold) const
{
	TimeDistribution d;
	d.threshold = threshold;
	d.count = static_cast<int>(samples.size());
	if (samples.empty()) return d;		// every figure stays 0, and the count says why

	std::vector<double> values;
	values.reserve(samples.size());
	double total = 0.0;
	for (const Sample& sample : samples)
	{
		values.push_back(sample.value);
		total += sample.value;
		// Strictly above the threshold: a passenger who waited exactly the
		// tolerance is not yet dissatisfied. The choice is arbitrary, so it is
		// made once, here, and pinned by a test instead of being rediscovered.
		if (threshold.has_value() && sample.value > threshold.value())
		{
			d.dissatisfied.push_back(sample.id);
		}
	}

	std::sort(values.begin(), values.end());
	d.average = total / values.size();
	d.max = values.back();

	// Nearest rank: the p-th quantile is the smallest sample at or above p of
	// the population, so every reported figure is a time somebody really had.
	auto quantile = [&values](double p) {
		std::size_t rank = static_cast<std::size_t>(std::ceil(p * values.size()));
		if (rank == 0) rank = 1;
		if (rank > values.size()) rank = values.size();
		return values[rank - 1];
	};
	d.p50 = quantile(0.50);
	d.p90 = quantile(0.90);

	return d;
}

TimeDistribution StandardMetricsCollector::getWaitingTimeDistribution() const
{
	return distributionOf(waitingSamples, thresholds.waiting);
}

TimeDistribution StandardMetricsCollector::getJourneyTimeDistribution() const
{
	return distributionOf(journeySamples, thresholds.journey);
}

WindowedTimeReport StandardMetricsCollector::windowedDistribution(const std::vector<Sample>& samples,
	const std::optional<double>& threshold) const
{
	std::vector<Sample> peak;
	std::vector<Sample> drain;
	for (const Sample& s : samples)
	{
		// <= the last arrival is peak: a metric that closed at the exact instant
		// of the last arrival belongs to the window, not the drain.
		if (s.instant <= lastArrivalTime) peak.push_back(s);
		else drain.push_back(s);
	}
	WindowedTimeReport r;
	r.peak = distributionOf(peak, threshold);
	r.drain = distributionOf(drain, threshold);
	r.boundaryTime = lastArrivalTime;
	return r;
}

std::optional<WindowedTimeReport> StandardMetricsCollector::getWaitingTimeByWindow() const
{
	return windowedDistribution(waitingSamples, thresholds.waiting);
}

std::optional<WindowedTimeReport> StandardMetricsCollector::getJourneyTimeByWindow() const
{
	return windowedDistribution(journeySamples, thresholds.journey);
}

std::optional<QueueReport> StandardMetricsCollector::getQueueReport() const
{
	QueueReport q;
	q.boundaryTime = lastArrivalTime;

	// Queue at end of arrivals = passengers whose boarding is strictly after
	// the last arrival. Because that is the maximum arrival instant, every
	// passenger arrived at or before it, so this is exactly the drain count of
	// the waiting distribution - an equality test_metrics_consistency asserts.
	int atEnd = 0;
	for (const Sample& s : waitingSamples)
	{
		if (s.instant > lastArrivalTime) atEnd++;
	}
	q.queueAtArrivalsEnd = atEnd;

	// Max queue = peak of arrivals(t) - boardings(t). +1 at each arrival, -1 at
	// each boarding, sorted by instant with arrivals before boardings at an
	// equal instant (so a passenger arriving the moment another boards is
	// counted present at the peak - the inclusive convention, fixed here). The
	// running sum's maximum is the peak backlog.
	struct QueueEvent { double time; int delta; };
	std::vector<QueueEvent> events;
	events.reserve(arrivalInstants.size() + waitingSamples.size());
	for (double a : arrivalInstants) events.push_back({a, +1});
	for (const Sample& s : waitingSamples) events.push_back({s.instant, -1});
	std::sort(events.begin(), events.end(), [](const QueueEvent& a, const QueueEvent& b) {
		if (a.time != b.time) return a.time < b.time;
		return a.delta > b.delta;		// +1 before -1 at the same instant
	});

	int current = 0;
	int maximum = 0;
	for (const QueueEvent& e : events)
	{
		current += e.delta;
		if (current > maximum) maximum = current;
	}
	q.maxQueue = maximum;

	return q;
}

std::optional<PassByReport> StandardMetricsCollector::getPassByReport() const
{
	PassByReport report;
	report.refusedAtDoor = refusedAtDoor;
	report.reallocated = reallocated;
	return report;
}

double StandardMetricsCollector::getTotalEnergyConsumption() const
{
	double total = 0;
	for (const auto& kv : motorEnergyByElevator) total += kv.second;
	return total;
}

double StandardMetricsCollector::getEnergyConsumption(int elevatorId) const
{
	auto it = motorEnergyByElevator.find(elevatorId);
	return it == motorEnergyByElevator.end() ? 0.0 : it->second;
}

void StandardMetricsCollector::printDistribution(const std::string& label, const TimeDistribution& d)
{
	std::cout << "\n" << label << ": p50 = " << d.p50 << ", p90 = " << d.p90
		<< ", worst = " << d.max << " (over " << d.count << " passengers)" << std::endl;

	if (!d.threshold.has_value())
	{
		// Deliberately not "0 dissatisfied": nobody measured.
		std::cout << "  dissatisfied: not measured (no tolerance threshold configured)" << std::endl;
		return;
	}

	std::cout << "  dissatisfied (above " << d.threshold.value() << "): "
		<< d.dissatisfiedCount().value();
	if (d.dissatisfiedRate().has_value())
	{
		std::cout << " of " << d.count << " (" << (d.dissatisfiedRate().value() * 100.0) << "%)";
	}
	std::cout << std::endl;
}

std::optional<EnergyModelDescription> StandardMetricsCollector::energyModel() const
{
	EnergyModelDescription d;
	d.model = "dc-static-converter";
	d.floorHeightM = floorHeightM;
	d.dynamics = dynamics;
	d.motor = motorParams;
	return d;
}

void StandardMetricsCollector::printSummary() const
{
	std::cout << "\n\n--- SIMULATION METRICS ---\n" << std::endl;
	std::cout << "Average waiting time: " << getAverageWaitingTime() << std::endl;
	std::cout << "Average travel time: " << getAverageTravelTime() << std::endl;
	std::cout << "Total energy consumption: " << getTotalEnergyConsumption() << std::endl;
	std::cout << "Simulation duration: " << simulationEndTime << std::endl;

	printDistribution("Waiting time", getWaitingTimeDistribution());
	printDistribution("Journey time (arrival -> alighting)", getJourneyTimeDistribution());

	// Per-window reading: the peak alone is an optimistic floor, so both
	// halves are always shown together.
	std::optional<WindowedTimeReport> ww = getWaitingTimeByWindow();
	if (ww.has_value())
	{
		std::cout << "\nBy window (arrivals ended at t = " << ww->boundaryTime << "):" << std::endl;
		printDistribution("  waiting, peak (served while arriving)", ww->peak);
		printDistribution("  waiting, drain (served after arrivals ended)", ww->drain);
		std::optional<WindowedTimeReport> jw = getJourneyTimeByWindow();
		printDistribution("  journey, peak (served while arriving)", jw->peak);
		printDistribution("  journey, drain (served after arrivals ended)", jw->drain);
	}

	// Backlog: the run's peak queue and the queue still waiting the instant
	// arrivals ended - a saturation signal that does not need a configured
	// cabin weight.
	std::optional<QueueReport> q = getQueueReport();
	if (q.has_value())
	{
		std::cout << "\nQueue: peak waiting at once = " << q->maxQueue
			<< ", still waiting when arrivals ended = " << q->queueAtArrivalsEnd << std::endl;
	}

	// Printed even when both are zero, because zero is the answer for every run
	// that configures no capacity and that is worth stating: it says the cabins
	// were elastic, not that they were well dispatched.
	std::cout << "\nFull cabin arrived and could not take them: " << refusedAtDoor.events
		<< " time(s), affecting " << refusedAtDoor.passengers.size() << " passenger(s)" << std::endl;
	std::cout << "Landing calls given back by a cabin that filled up: " << reallocated.events
		<< " time(s), affecting " << reallocated.passengers.size() << " passenger(s)" << std::endl;
}

} // namespace openEGCS
