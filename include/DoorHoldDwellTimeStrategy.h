/*
DoorHoldDwellTimeStrategy.h

An IDwellTimeStrategy whose doors stay open for a minimum time, as a real lift
controller holds them. FixedDwellTimeStrategy charges n x tp for the
open-doors phase and nothing more, so a stop that moves one person keeps its
doors open for 1.2 s and a stop that moves nobody for none at all. Barney &
Al-Sharif (2015) sec. 5.5.2 separates the two: the doors are held open for a
"dwell time" so people can get in or out - 2.0 s for a car call, because the
passengers inside "will be prepared to exit", and 3.0 s for a landing call,
because the people outside "will need to walk to the car entrance".

The open-doors phase (getTransferTime) is therefore:
  - Timed doors (no passenger detector): max(T, n x tp). The doors close when
    the timer runs out, unless people are still crossing.
  - Detector ("differential door timing"): T if nobody crosses; n x tp + the
    reduced hold (0.5 s in the book) if somebody does - once a passenger has
    crossed the threshold the remaining hold drops to that, and the detector
    keeps the doors open while people are passing.
where n = boarding + alighting and T is the car-call hold when the stop serves
a car call and no landing call, and the landing-call hold otherwise (a stop with
neither was made for a landing call whose passengers another cabin took: see
StopContext).

Door opening and closing are constants, as in FixedDwellTimeStrategy. This is a
separate class and not a flag on that one because a floored or detector-driven
open phase is a different rule.

The simulator's default (through DwellPresets.h), with timed doors: a detector
is equipment, and the default does not presume equipment - the same reasoning
that makes normal opening, not advanced, the default.
*/

#ifndef __DOORHOLDDWELLTIMESTRATEGY__
#define __DOORHOLDDWELLTIMESTRATEGY__

#include "IDwellTimeStrategy.h"

namespace openEGCS {

class DoorHoldDwellTimeStrategy : public IDwellTimeStrategy {
	public:
		struct Params {
			double doorOpenS = 0.0;			// seconds, per stop
			double doorCloseS = 0.0;		// seconds, per stop
			double perPassengerS = 0.0;		// seconds per passenger boarding or alighting
			double carCallHoldS = 2.0;		// sec. 5.5.2: stop for a car call only
			double landingCallHoldS = 3.0;	// sec. 5.5.2: stop for a landing call
			bool detector = false;			// differential door timing fitted
			double detectorHoldS = 0.5;		// sec. 5.5.2: hold left once somebody crosses
		};

		// Throws std::invalid_argument if any time is negative or not finite.
		explicit DoorHoldDwellTimeStrategy(const Params& params);

		double getDoorOpenTime(const StopContext& stop) const override;
		// The open-doors phase described in the file comment. Never shorter
		// than n x tp: a hold delays the doors, it never cuts a transfer short.
		double getTransferTime(const StopContext& stop) const override;
		double getDoorCloseTime(const StopContext& stop) const override;

		// model "door-hold", the values, hold "timed" or "detector".
		DwellDescription describe() const override;

		// The minimum hold T this stop gets: the car-call hold only when the
		// stop serves a car call and no landing call, the landing-call hold
		// otherwise. Public because it is the one judgement in this class and is
		// worth testing on its own.
		double holdFor(const StopContext& stop) const;

		const Params& params() const { return p; }

	private:
		Params p;
};

} // namespace openEGCS

#endif
