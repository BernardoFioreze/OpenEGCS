/*
IDwellTimeStrategy.h

Strategy interface for how long an elevator spends *stopped* at a floor:
door opening, the passenger transfer itself, and door closing. In a real
building this dead time is one of the largest components of the round-trip
time, so leaving it out makes every reported waiting and travel time
optimistic.

Why an interface rather than three constants on Elevator: *how much* a stop
costs is policy, not physics. A dimensioning guide gives different figures
per door type, per building class and per direction of travel, and someone
benchmarking a dispatch algorithm may want to model transfer time as
non-linear in the number of passengers (loading slows down as the cabin
fills) or to charge nothing at all. Same reasoning as ICapacityStrategy -
the number is the easy part, the rule for deriving it is the pluggable part.
Implementations shipped: FixedDwellTimeStrategy (constant doors, n x tp open
phase; all zero by default, the instantaneous stop a bare Elevator keeps) and
DoorHoldDwellTimeStrategy (the same, with the doors held open a minimum time).
The simulator's default, built by DwellPresets.h, is the second one with the
handbook's centre-opening door.

Time model: a stop runs
    arrival --door opens--> ENTER/LEAVE --transfer--> door closes --> DEPART
so the ENTER/LEAVE records of a stop are dated arrival + getDoorOpenTime(),
while the cabin only starts moving again at arrival + the full dwell.
*/

#ifndef __IDWELLTIMESTRATEGY__
#define __IDWELLTIMESTRATEGY__

#include <string>

namespace openEGCS {

// What is known about a stop when its dwell time is computed. Passed by
// const reference so fields can be added later (floor, current load, time
// of day) without breaking existing implementations.
//
// `boarding`/`alighting` count the *passengers* served at this one stop, not
// the tasks: Elevator::addTask merges tasks for the same floor into a single
// stop, so a stop can serve several customers at once. Both are >= 0. Both CAN
// be 0: a stop a cabin was already braking for when another car took its
// passengers is still made, and nobody moves at it.
//
// `carCall`/`hallCall` say why the cabin stops here: somebody inside asked for
// this floor, somebody on the landing pressed the button, or both. Barney &
// Al-Sharif sec. 5.5.2 holds the doors open longer for a landing call, because
// the people outside must walk to the car. A stop with neither was made for a
// landing call whose passengers another cabin took - a car call is never
// withdrawn (Closs rule 2). Both default to false, so an implementation that
// does not need them simply never reads them.
struct StopContext {
	int boarding = 0;		// passengers getting in at this stop
	int alighting = 0;		// passengers getting out at this stop
	bool carCall = false;	// a passenger inside asked for this floor
	bool hallCall = false;	// a landing call (either direction) is served here
};

// What a dwell strategy says about itself, for the snapshot's metadata. The
// resolved values, not the preset they came from: a config file can override
// any one of them, and what makes two snapshots comparable is the numbers.
// `model` names the rule ("fixed", "door-hold"); `hold` is "none", "timed" or
// "detector", and the hold fields mean something only when it is not "none"
// (`detectorHoldS` only for "detector").
struct DwellDescription {
	std::string model;
	double doorOpenS = 0.0;			// seconds per stop
	double doorCloseS = 0.0;		// seconds per stop
	double perPassengerS = 0.0;		// seconds per passenger boarding or alighting
	std::string hold = "none";
	double carCallHoldS = 0.0;		// minimum open phase at a stop for car calls only
	double landingCallHoldS = 0.0;	// minimum open phase at any other stop
	double detectorHoldS = 0.0;		// what is left of the hold once somebody crosses
};

class IDwellTimeStrategy {
	public:
		virtual ~IDwellTimeStrategy() = default;

		// Time from the cabin arriving at the floor until passengers may
		// start moving in or out. The ENTER/LEAVE records of the stop are dated
		// at the end of it, so this is the part of the dwell that counts towards
		// a waiting customer's wait.
		virtual double getDoorOpenTime(const StopContext& stop) const = 0;

		// How long the doors stand open: from the end of the opening until the
		// doors start to close. In the simplest model it is the time the
		// passengers themselves take to get in and out; a door controller that
		// holds the doors open for a minimum time makes it longer than that
		// (DoorHoldDwellTimeStrategy). Charged after the ENTER/LEAVE records, so
		// it delays the cabin's departure without changing when the boarding is
		// recorded.
		virtual double getTransferTime(const StopContext& stop) const = 0;

		// Time from the last passenger being clear until the cabin may start
		// moving towards its next stop.
		virtual double getDoorCloseTime(const StopContext& stop) const = 0;

		// The values this strategy charges, for the snapshot. Pure virtual on
		// purpose: a strategy that cannot say what it charges would write a
		// snapshot whose times nobody can interpret. A rule with no fixed values
		// describes what it can and names itself in `model`.
		virtual DwellDescription describe() const = 0;
};

} // namespace openEGCS

#endif
