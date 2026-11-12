#include <cmath>
#include <stdexcept>
#include "FixedDwellTimeStrategy.h"

namespace openEGCS {

FixedDwellTimeStrategy::FixedDwellTimeStrategy(double door_open_time,
	double door_close_time, double per_passenger_time)
	: doorOpenTime(door_open_time), doorCloseTime(door_close_time),
	  perPassengerTime(per_passenger_time)
{
	// Negative dwell would run the simulation clock backwards at a stop, and
	// a non-finite one would poison every event timestamp derived from the
	// route (Elevator::getStopArrivalTime sums these), so both are rejected
	// at construction rather than being allowed to surface as a nonsensical
	// timeline much later.
	if (!std::isfinite(door_open_time) || door_open_time < 0) {
		throw std::invalid_argument("FixedDwellTimeStrategy: door_open_time must be finite and >= 0");
	}
	if (!std::isfinite(door_close_time) || door_close_time < 0) {
		throw std::invalid_argument("FixedDwellTimeStrategy: door_close_time must be finite and >= 0");
	}
	if (!std::isfinite(per_passenger_time) || per_passenger_time < 0) {
		throw std::invalid_argument("FixedDwellTimeStrategy: per_passenger_time must be finite and >= 0");
	}
}

double FixedDwellTimeStrategy::getDoorOpenTime(const StopContext& /*stop*/) const
{
	return doorOpenTime;
}

double FixedDwellTimeStrategy::getTransferTime(const StopContext& stop) const
{
	int passengers = stop.boarding + stop.alighting;
	if (passengers < 0) passengers = 0;
	return passengers * perPassengerTime;
}

double FixedDwellTimeStrategy::getDoorCloseTime(const StopContext& /*stop*/) const
{
	return doorCloseTime;
}

DwellDescription FixedDwellTimeStrategy::describe() const
{
	DwellDescription d;
	d.model = "fixed";
	d.doorOpenS = doorOpenTime;
	d.doorCloseS = doorCloseTime;
	d.perPassengerS = perPassengerTime;
	return d;		// hold stays "none": this class never holds the doors
}

bool FixedDwellTimeStrategy::isZero() const
{
	return doorOpenTime == 0.0 && doorCloseTime == 0.0 && perPassengerTime == 0.0;
}

} // namespace openEGCS
