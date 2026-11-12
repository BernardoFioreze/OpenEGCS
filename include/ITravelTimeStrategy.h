/*
ITravelTimeStrategy.h

Strategy interface for how long an elevator spends *moving* between two
floors, and where it is partway through that move. Sibling of
IDwellTimeStrategy, which owns the other half of a round trip: how long it
spends stopped.

Why this is an interface rather than a constant on Elevator: a constant speed
(`|destination - position| * timePerFloor`) charges a floor the same whether
the cabin arrives at it from rest or at full speed. The real cost of a leg is
dominated by starting and stopping, which is fixed per leg rather than
proportional to distance - the project's jerk-limited profile (LiftKinematics.h,
validated against Peters) prices nine one-floor hops at 46.5 s and one
nine-floor run at 22.6 s, while a linear model charges 45 for both. So the
travel model is a *model*, not a parameter. Two ship:
  - KinematicTravelTimeStrategy (the default) drives the clock from the same
    profile that prices energy, so time and energy describe the same journey.
  - LinearTravelTimeStrategy is the constant-speed model, kept for comparison
    and for reproducing studies that assume it.

This strategy owns every place that converts distance to time: the dispatch
cost estimate (Elevator::getFinishTime), the event timestamps
(Elevator::getStopArrivalTime) and the cabin's position (Elevator::update), so
a new travel model moves all three together.

One shared instance per ElevatorSystem, like ICapacityStrategy and
IDwellTimeStrategy: the travel model is a property of the installation, not
of an individual cabin.
*/

#ifndef __ITRAVELTIMESTRATEGY__
#define __ITRAVELTIMESTRATEGY__

#include <optional>
#include <string>

namespace openEGCS {

// What a travel model says about itself, for the snapshot's metadata. `model`
// repeats name(). The other fields are the model's parameters, each present
// only when the model has it: a constant-speed model has a time per floor, a
// kinematic one a floor height and a motion profile. Absent is not zero.
struct TravelDescription {
	std::string model;
	std::optional<double> secondsPerFloor;	// constant-speed models
	std::optional<double> floorHeightM;		// metres per floor
	std::optional<double> ratedSpeedMs;		// m/s
	std::optional<double> accelMs2;			// m/s^2
	std::optional<double> jerkMs3;			// m/s^3
};

class ITravelTimeStrategy {
	public:
		virtual ~ITravelTimeStrategy() = default;

		// How long a single continuous leg of `signedFloors` floors takes,
		// starting and ending at rest. Signed so an implementation can model
		// up and down differently (a real installation's counterweight makes
		// them differ in energy, and a ratcheted or hydraulic lift in time);
		// the sign is the direction of travel, positive being up.
		//
		// "Continuous" matters: this is one run between two stops, not a
		// multi-stop journey. The route sum is the caller's job, because only
		// the caller knows what the cabin does *between* legs - see
		// Elevator::getStopArrivalTime, which adds each stop's dwell in
		// between. An implementation that charged a per-stop cost here would
		// be charging it twice.
		virtual double legDuration(double signedFloors) const = 0;

		// Signed distance in floors covered `elapsed` time units after that
		// leg began. This is what a caller deriving a cabin's position needs,
		// and it is a separate method rather than a derived quantity because
		// for any model other than the linear one the position is not
		// proportional to elapsed time.
		//
		// Contract, and it is the load-bearing one:
		//
		//     displacement(d, legDuration(d)) == d
		//
		// exactly, not to within rounding, and clamped outside the leg -
		// `elapsed <= 0` gives 0 and `elapsed >= legDuration(d)` gives `d`.
		// An event's timestamp comes from a sum of whole-leg durations while
		// the cabin's position comes from this function, so the invariant
		// "when a stop's ARRIVE fires, the cabin is at that stop's floor" is
		// exactly the statement that the two agree at a leg's end. An
		// implementation that lets the two drift breaks that invariant, and the
		// failure looks like an event firing with the cabin somewhere else.
		virtual double displacement(double signedFloors, double elapsed) const = 0;

		// How much of a leg of `signedFloors` is still left to run, `elapsed`
		// time units after it began. `elapsed <= 0` gives the whole
		// legDuration(signedFloors); `elapsed >= legDuration(signedFloors)`
		// gives 0.
		//
		// The obvious way to time a route - take the cabin's current position
		// and price the distance from there to the next stop as a fresh leg -
		// is only correct for a model whose duration is ADDITIVE in distance.
		// The linear model is: splitting a leg in two and charging both halves
		// gives the same total. A kinematic one is not, because each leg is
		// charged a start and a stop: pricing "the remaining 0.03 floors" as a
		// fresh run bills a whole acceleration and deceleration ramp for a cabin
		// that is already moving and about to arrive. So the leg in flight is the
		// model's business, not the caller's arithmetic.
		virtual double remainingDuration(double signedFloors, double elapsed) const = 0;

		// The shortest TOTAL leg this cabin can still make, given that it is
		// `elapsed` into a leg of `signedFloors`. Magnitude, measured from the
		// leg's own start - not the distance that is left to run.
		//
		// This answers "a nearer call just arrived; can the cabin still stop
		// there?". A leg can be RETARGETED mid-flight: an insertion that becomes
		// the front of the route changes where the cabin is going while it is
		// already moving. The source this project's kinematics comes from
		// prescribes exactly that - Peters, section 6.6 "Minimum Travel
		// Distances" (p. 6-13): the profile equations "can be reset with a new
		// value of d mid-way through a journey", and the resulting motion "will
		// remain continuous" provided the new d is at least this. Peters' own
		// simulator has the same pair, as Motion::MinDistance() and
		// Motion::ConfirmDestination().
		//
		// Why it belongs on the travel model and not on Elevator: the answer is
		// a property of the model. A constant-velocity cabin stops dead at any
		// instant, so for LinearTravelTimeStrategy this is just the distance
		// already covered and no retarget is ever refused. A jerk-limited cabin
		// cannot: past the start of its deceleration it is committed, and this
		// returns `|signedFloors|` itself.
		//
		// Contracts, both relied on by the caller:
		//   - monotone non-decreasing in `elapsed` (a cabin's options only
		//     narrow as it travels), and
		//   - never greater than `|signedFloors|`, with equality exactly when
		//     the destination can no longer change (Peters eq 6.107).
		// `elapsed <= 0` gives 0: a leg that has not begun can still be
		// retargeted anywhere.
		virtual double minimumLegDistance(double signedFloors, double elapsed) const = 0;

		// A short, stable identifier for this model, recorded in the snapshot's
		// metadata (`travelModel`). A snapshot that cannot say which travel
		// model produced its timestamps is not comparable with another one: two
		// runs with the same `<Elevator speed>` and different models would be
		// indistinguishable from the file alone.
		//
		// Keep it machine-readable and unchanged once published - a viewer or a
		// comparison script may key on it.
		virtual const char* name() const = 0;

		// name() and the parameters behind it. Two snapshots with the same
		// name() and different parameters do not share a clock.
		virtual TravelDescription describe() const = 0;
};

} // namespace openEGCS

#endif
