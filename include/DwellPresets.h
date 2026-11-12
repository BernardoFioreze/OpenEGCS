/*
DwellPresets.h

Named, cited stop durations for the dwell strategies. Every number in this file
comes from Barney, G.C. & Al-Sharif, L. (2015), *Elevator Traffic Handbook:
Theory and Practice*, 2nd ed., chapter 5, and each one names its table or
section.

What it is NOT: a new kind of strategy. A preset is a FixedDwellTimeStrategy (or
DoorHoldDwellTimeStrategy) with cited values, built by a free function, so
nothing in ElevatorSystem or Elevator branches on a "door kind". It is the
sibling of TrafficCapacity.h: a concrete, literature-anchored calculation in the
core, rather than an interface, because it is a table lookup, not a rival model.

The choices a preset makes, and the default of each:
  - door type (Table 5.3): side or centre opening. Default CENTRE.
  - opening mode (Table 5.3): normal, or advanced (the doors start opening while
    the car levels, which needs equipment, sec. 5.3.3.3). Default NORMAL.
  - transfer profile: office 1.2 s or residential 1.75 s per person (sec.
    5.5.1), or the per-rated-load average of Table 5.7. Default OFFICE.
  - door hold (sec. 5.5.2): see DoorHold. Default TIMED.
The door WIDTH is not a choice: it follows the rated load (sec. 5.3.3.1), and
that is what ties a preset to the cabin it is used with.

DoorType::None is the instantaneous stop: doors and transfer at zero, for
studies that leave the stop out.
*/

#ifndef __DWELLPRESETS__
#define __DWELLPRESETS__

#include <memory>

#include "FixedDwellTimeStrategy.h"

namespace openEGCS {

enum class DoorType { None, Side, Centre };
enum class DoorOpening { Normal, Advanced };
enum class TransferProfile { Office, Residential, RatedLoad };
// How the door controller holds the doors open (sec. 5.5.2): not at all
// beyond the transfer (None), for a minimum time (Timed: 2.0 s car call /
// 3.0 s landing call), or with a passenger detector that cuts the hold to
// 0.5 s once somebody crosses (Detector).
enum class DoorHold { None, Timed, Detector };

// The choices of a preset. Default-constructed it is the project's default
// dwell: centre-opening doors, normal opening, office transfer, and the
// doors held open 2.0 s (car call) / 3.0 s (landing call), timed.
struct DwellPreset {
	DoorType door = DoorType::Centre;
	DoorOpening opening = DoorOpening::Normal;
	TransferProfile transfer = TransferProfile::Office;
	DoorHold hold = DoorHold::Timed;		// doors held open 2.0 s (car call) / 3.0 s (landing call)

	// The pre-item-#56 stop: zero doors, zero transfer, no hold, whatever the
	// other fields say.
	static DwellPreset instantaneous();
};

// The three values a preset resolves to, in seconds. Plain data, so a caller
// that needs the numbers themselves (the CLI's saturation warning prices td and
// tp into the round trip time) does not have to reach inside the strategy.
struct DwellTimes {
	double doorOpenS = 0.0;
	double doorCloseS = 0.0;
	double perPassengerS = 0.0;
	DoorHold hold = DoorHold::None;	// the two holds below are read only when not None
	double carCallHoldS = 0.0;
	double landingCallHoldS = 0.0;
};

// Barney & Al-Sharif sec. 5.5.2 (offices): how long the doors are held open for
// a stop answering a car call, and for one answering a landing call.
constexpr double kCarCallHoldS = 2.0;
constexpr double kLandingCallHoldS = 3.0;

// Door clear width, in mm, for a cabin of this rated load (sec. 5.3.3.1): 800
// up to and including 1000 kg, 1100 above. An unlimited cabin (infinity) is
// "above 1000 kg" and gets 1100. Throws std::invalid_argument for a load that
// is NaN or <= 0, which is not a cabin.
int doorWidthMmFor(double ratedLoadKg);

// Opening and closing time of a door (Table 5.3). `widthMm` must be 800 or 1100,
// the two widths the table has - anything else throws std::invalid_argument
// rather than interpolating a figure the book does not give. DoorType::None
// returns zero for both, whatever the width and opening.
DwellTimes doorTimes(DoorType door, int widthMm, DoorOpening opening);

// Per-passenger transfer time, one way (sec. 5.5.1 and Table 5.7). Office is
// 1.2 s and Residential 1.75 s regardless of the cabin. RatedLoad reads Table
// 5.7 (offices) at the row of the largest tabulated load that does not exceed
// the cabin's - 900 kg reads the 800 kg row - which is this project's extension
// of a table that only lists ISO rated loads (plan decision D3a): below 320 kg
// it is 1.2 s, above 2500 kg or unlimited it is 0.8 s. Throws
// std::invalid_argument for a load that is NaN or <= 0.
double transferTimeFor(TransferProfile profile, double ratedLoadKg);

// The values of `preset` for a cabin of `ratedLoadKg`. For DoorType::None
// everything is zero and the hold is None (the transfer profile and the hold
// are ignored: the instantaneous stop has neither). Otherwise a hold other than
// None carries the sec. 5.5.2 figures.
DwellTimes resolveDwellTimes(const DwellPreset& preset, double ratedLoadKg);

// The strategy for a set of resolved values. A FixedDwellTimeStrategy when the
// hold is None - or when both door times are zero: with no door cycle there is
// no door to hold open, which is what keeps an explicit `0 0 0` the
// instantaneous stop. A DoorHoldDwellTimeStrategy otherwise.
std::shared_ptr<IDwellTimeStrategy> makeDwellStrategy(const DwellTimes& times);

// makeDwellStrategy(resolveDwellTimes(preset, ratedLoadKg)).
std::shared_ptr<IDwellTimeStrategy> makeDwellStrategy(const DwellPreset& preset, double ratedLoadKg);

} // namespace openEGCS

#endif
