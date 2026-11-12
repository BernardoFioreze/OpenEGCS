/*
KinematicTravelTimeStrategy.h

The default travel model: a leg's duration and the cabin's position during it
come from the jerk-limited motion profile of LiftKinematics.h - the same
profile that prices energy - instead of from `|floors| * staticTime`.

What that changes, concretely. A linear model charges the same for nine
one-floor hops as for one nine-floor run (45 time units either way, at
speed 5), while the profile prices them at 46.5 s and 22.6 s. The cost of
stopping is precisely what separates a good dispatch algorithm from a bad one,
so a clock that is blind to it cannot rank them.

Units. The interface speaks floors, because that is what a route is made of;
the profile speaks metres. This class owns the bridge, `floors * floorHeightM`,
and that bridge exists in two places - here and in StandardMetricsCollector,
which converts the same way to price energy. Those two must be built from the
same LiftDynamics and the same floor height or a run will report a time and an
energy for different journeys. main.cpp therefore derives both from one
`<Floor height>` argument and one defaultDynamicsFor() call.

One assumption: a single uniform floor height. A mezzanine or a deeper
basement would need the conversion to be per-floor.
*/

#ifndef __KINEMATICTRAVELTIMESTRATEGY__
#define __KINEMATICTRAVELTIMESTRATEGY__

#include "ITravelTimeStrategy.h"
#include "LiftKinematics.h"

namespace openEGCS {

class KinematicTravelTimeStrategy : public ITravelTimeStrategy {
	private:
		double floorHeightM;
		LiftDynamics dynamics;

	public:
		// `floor_height_m` must match the one the metrics collector prices
		// energy with - see this file's header on why. `dynamics` comes from
		// defaultDynamicsFor(total building travel) unless the caller has
		// figures for a specific installation.
		KinematicTravelTimeStrategy(double floor_height_m = 3.5,
			LiftDynamics lift_dynamics = defaultDynamicsFor(3.5 * 9));

		// buildProfile(...).totalTimeS for the leg: one acceleration ramp, an
		// optional cruise, one deceleration ramp, starting and ending at rest.
		// The per-leg cost of starting and stopping is therefore charged here,
		// which is the whole point - and it is charged once per leg, not once
		// per floor, so nine one-floor hops cost strictly more than one
		// nine-floor run.
		double legDuration(double signedFloors) const override;

		// displacementAt(...) converted back to floors.
		//
		// The endpoint is returned as `signedFloors` itself rather than as
		// `displacementAt(p, t) / floorHeightM`, and that is not a
		// micro-optimisation: `(d * h) / h` is NOT exactly `d` in IEEE 754 -
		// with d = 0.1 and h = 3.0 the round trip lands on
		// 0.10000000000000002 - so computing it would break the interface's
		// exact endpoint contract, and with it the invariant that a stop's ARRIVE
		// fires with the cabin at that stop's floor. Same shape of fix as
		// LiftKinematics::displacementAt, for the same reason.
		double displacement(double signedFloors, double elapsed) const override;

		// `legDuration(signedFloors) - elapsed`, floored at 0. Exactly
		// consistent with displacement() by construction: the cabin reaches
		// the destination when elapsed hits legDuration, which is the same
		// instant displacement()'s clamp puts it there. That consistency is
		// the whole reason this method is on the interface - see
		// ITravelTimeStrategy.h.
		double remainingDuration(double signedFloors, double elapsed) const override;

		// Peters' d_min (section 6.6, eq 6.103-6.112), per phase of the
		// profile, converted back to floors. One set of branches covers all
		// three conditions rather than three, because the Condition C results
		// fall out of it: with t2 = t4 = 0 the "committed to the peak" branch
		// evaluates to `d` itself, which IS eq 6.112.
		//
		// The branches are continuous at every phase boundary, checked
		// algebraically and by test - which matters because a jump here would
		// make a retarget be accepted or refused on the strength of a rounding.
		//
		// One equation is NOT transcribed from the thesis. The PDF's eq 6.104
		// (the constant-acceleration phase) extracts with a corrupted sign, and
		// read literally it predicts continuity where measurement shows a jump.
		// The form used here is derived instead: the profile's prefix survives a
		// retarget while `elapsed` still falls inside the NEW profile's own
		// phase 2, i.e. while its peak would be at least `a * elapsed`, so
		// d_min is that peak's zero-cruise trapezoid, `a*t^2 + a^2*t/j`. That
		// gives 2.696 m where the corrupted form gives 0.642, and 2.696 is what
		// the simulated motion agrees with.
		double minimumLegDistance(double signedFloors, double elapsed) const override;

		// "kinematic" - see ITravelTimeStrategy::name().
		const char* name() const override;
		// "kinematic", the floor height and the LiftDynamics.
		TravelDescription describe() const override;
};

} // namespace openEGCS

#endif
