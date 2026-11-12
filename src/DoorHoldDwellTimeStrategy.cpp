#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "DoorHoldDwellTimeStrategy.h"

namespace openEGCS {

namespace {

void requireTime(double value, const char* name)
{
	// Same reason as FixedDwellTimeStrategy: these are summed into every
	// timestamp derived from the route, so a negative or non-finite one would
	// surface much later as a nonsensical timeline.
	if (!std::isfinite(value) || value < 0.0) {
		throw std::invalid_argument(std::string("DoorHoldDwellTimeStrategy: ") + name + " must be finite and >= 0");
	}
}

} // namespace

DoorHoldDwellTimeStrategy::DoorHoldDwellTimeStrategy(const Params& params) : p(params)
{
	requireTime(p.doorOpenS, "doorOpenS");
	requireTime(p.doorCloseS, "doorCloseS");
	requireTime(p.perPassengerS, "perPassengerS");
	requireTime(p.carCallHoldS, "carCallHoldS");
	requireTime(p.landingCallHoldS, "landingCallHoldS");
	requireTime(p.detectorHoldS, "detectorHoldS");
}

double DoorHoldDwellTimeStrategy::getDoorOpenTime(const StopContext& /*stop*/) const
{
	return p.doorOpenS;
}

double DoorHoldDwellTimeStrategy::holdFor(const StopContext& stop) const
{
	// A car call alone: the people getting out are already at the door. Any
	// landing call, or no request at all (a stop whose landing-call passengers
	// another cabin took - a car call is never withdrawn), waits for people to
	// walk up from the landing.
	return (stop.carCall && !stop.hallCall) ? p.carCallHoldS : p.landingCallHoldS;
}

double DoorHoldDwellTimeStrategy::getTransferTime(const StopContext& stop) const
{
	const int passengers = std::max(0, stop.boarding) + std::max(0, stop.alighting);
	const double transfer = passengers * p.perPassengerS;
	const double hold = holdFor(stop);
	if (p.detector) {
		// Nobody crosses: the full hold runs out. Somebody does: the detector
		// keeps the doors open while they pass, then the reduced hold.
		return passengers == 0 ? hold : transfer + p.detectorHoldS;
	}
	return std::max(hold, transfer);
}

double DoorHoldDwellTimeStrategy::getDoorCloseTime(const StopContext& /*stop*/) const
{
	return p.doorCloseS;
}

DwellDescription DoorHoldDwellTimeStrategy::describe() const
{
	DwellDescription d;
	d.model = "door-hold";
	d.doorOpenS = p.doorOpenS;
	d.doorCloseS = p.doorCloseS;
	d.perPassengerS = p.perPassengerS;
	d.hold = p.detector ? "detector" : "timed";
	d.carCallHoldS = p.carCallHoldS;
	d.landingCallHoldS = p.landingCallHoldS;
	if (p.detector) d.detectorHoldS = p.detectorHoldS;
	return d;
}

} // namespace openEGCS
