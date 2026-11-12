/*
FixedDwellTimeStrategy.h

Default IDwellTimeStrategy: a constant door-open and door-close time, plus a
transfer time proportional to the number of passengers moving at the stop.

The constructor's three values default to zero: the instantaneous stop, which
is what a bare Elevator gets. The SIMULATOR's default is built by
DwellPresets.h from Barney & Al-Sharif 2015's Table 5.3 door times and sec.
5.5.1 / Table 5.7 transfer times; since that preset holds the doors open a
minimum time, it is a DoorHoldDwellTimeStrategy, and makeDwellStrategy picks
this class only when the preset has no hold.

The values are seconds, added to the travel time unscaled; one simulation time
unit is a second, so a cited figure goes in as is.
*/

#ifndef __FIXEDDWELLTIMESTRATEGY__
#define __FIXEDDWELLTIMESTRATEGY__

#include "IDwellTimeStrategy.h"

namespace openEGCS {

class FixedDwellTimeStrategy : public IDwellTimeStrategy {
	private:
		// Seconds. These are added to the travel time unscaled, and travel
		// time comes from the installation's motion profile in SI seconds, so
		// a "2 seconds of door" typed here really is two seconds.
		double doorOpenTime;		// seconds per stop
		double doorCloseTime;		// seconds per stop
		double perPassengerTime;	// seconds per passenger boarding or alighting

	public:
		// Throws std::invalid_argument if any argument is negative or not
		// finite. Zero is allowed and is the default of the arguments (not of
		// the simulator - see the file comment and DwellPresets.h).
		explicit FixedDwellTimeStrategy(double door_open_time = 0.0,
			double door_close_time = 0.0,
			double per_passenger_time = 0.0);

		double getDoorOpenTime(const StopContext& stop) const override;

		// (boarding + alighting) x per_passenger_time. Linear on purpose: a
		// non-linear rule (loading slowing down as the cabin fills) is a
		// different policy and belongs in a different IDwellTimeStrategy, not
		// in a flag on this one.
		double getTransferTime(const StopContext& stop) const override;

		double getDoorCloseTime(const StopContext& stop) const override;

		// model "fixed", the three values, hold "none".
		DwellDescription describe() const override;

		// True when all three values are zero, i.e. when this strategy is the
		// instantaneous stop. Nothing in the engine calls it: the zero-dwell path is
		// arithmetically identical without any special-casing. It exists for tests
		// and for a caller that wants to report whether a run was configured with a
		// dwell at all.
		bool isZero() const;
};

} // namespace openEGCS

#endif
