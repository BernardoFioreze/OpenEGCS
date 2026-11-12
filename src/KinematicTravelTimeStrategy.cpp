#include "KinematicTravelTimeStrategy.h"

#include <cmath>

#include <cmath>

namespace openEGCS {

KinematicTravelTimeStrategy::KinematicTravelTimeStrategy(double floor_height_m,
	LiftDynamics lift_dynamics)
	: floorHeightM(floor_height_m), dynamics(lift_dynamics)
{
}

double KinematicTravelTimeStrategy::legDuration(double signedFloors) const
{
	KinematicProfile profile = buildProfile(signedFloors * floorHeightM,
		dynamics.ratedSpeedMs, dynamics.accelMs2, dynamics.jerkMs3);
	return profile.totalTimeS;
}

double KinematicTravelTimeStrategy::displacement(double signedFloors, double elapsed) const
{
	if (elapsed <= 0.0) {
		return 0.0;
	}

	KinematicProfile profile = buildProfile(signedFloors * floorHeightM,
		dynamics.ratedSpeedMs, dynamics.accelMs2, dynamics.jerkMs3);

	// The interface's endpoint contract has to hold exactly, so past the end
	// of the leg this returns the caller's own `signedFloors` rather than
	// dividing the profile's distance back out - `(d * h) / h` is not `d` in
	// IEEE 754. See the header.
	if (elapsed >= profile.totalTimeS) {
		return signedFloors;
	}

	double covered = displacementAt(profile, elapsed) / floorHeightM;

	// Clamped to the leg: a cabin never reports itself past the floor it is
	// going to, which is both the "no overshoot" guarantee and the invariant
	// that a stop's ARRIVE fires with the cabin at that stop's floor.
	// `signedFloors` is returned literally so the clamped value stays exact.
	//
	// The clamp is load-bearing: `displacementAt` returns metres and this
	// divides by floorHeightM, and `(d * h) / h` is not `d` in IEEE 754 - so a
	// leg that genuinely ended at its floor can come back a few ulps beyond it
	// (worst seen: 1.78e-15 floors, on a seven-floor leg; a few percent of all
	// calls). Without the clamp, cabins end up ulps past their floor, which
	// changes the route and breaks the energy accounting. Same shape of trap as
	// `(d * h) / h` at the endpoint above - see the header.
	if (std::abs(covered) > std::abs(signedFloors)) {
		return signedFloors;
	}
	return covered;
}

double KinematicTravelTimeStrategy::remainingDuration(double signedFloors, double elapsed) const
{
	double total = legDuration(signedFloors);
	if (elapsed <= 0.0) return total;
	return (elapsed >= total) ? 0.0 : total - elapsed;
}

double KinematicTravelTimeStrategy::minimumLegDistance(double signedFloors, double elapsed) const
{
	if (elapsed <= 0.0) {
		return 0.0;	// not started: can still be retargeted anywhere
	}

	KinematicProfile p = buildProfile(signedFloors * floorHeightM,
		dynamics.ratedSpeedMs, dynamics.accelMs2, dynamics.jerkMs3);
	if (p.totalTimeS <= 0.0) {
		return 0.0;	// unusable dynamics: nothing moves, so nothing is committed
	}

	double a = p.attainedAccelMs2, j = p.jerkMs3, peak = p.peakVelocityMs;
	double d = std::abs(p.signedDistanceM);

	// Same phase boundaries, in the same order, as sampleProfile and
	// displacementAt - the three have to be read side by side.
	double T1 = p.t1;
	double T2 = T1 + p.t2;
	double T3 = T2 + p.t1;
	double T4 = T3 + p.t4;

	double dMinM;
	if (elapsed <= T1) {
		// Still ramping acceleration up. Stop ramping now and mirror: that is
		// the Condition C profile whose tau equals `elapsed` (Peters eq 6.103,
		// and eq 6.111 for a leg that is itself Condition C).
		dMinM = 2.0 * j * elapsed * elapsed * elapsed;
	} else if (elapsed <= T2) {
		// At rated acceleration. The prefix survives any retarget whose own
		// peak is at least a*elapsed, so d_min is that peak's zero-cruise
		// trapezoid. Derived, not transcribed - see the header.
		dMinM = a * elapsed * elapsed + (a * a * elapsed) / j;
	} else if (elapsed <= T3) {
		// Ramping acceleration back down: committed to reaching `peak`, so the
		// shortest leg left is the zero-cruise trapezoid at that peak
		// (eq 6.105). Continuous with the branch above, where peak == a*T2.
		dMinM = (peak * peak) / a + (peak * a) / j;
	} else if (elapsed <= T4) {
		// Cruising: decelerate from here, so the whole leg is elapsed*peak
		// (eq 6.106). Continuous with the branch above, where T3 == a/j + peak/a.
		dMinM = elapsed * peak;
	} else {
		// Already decelerating: the destination can no longer change at all
		// (eq 6.107).
		dMinM = d;
	}

	// The interface promises this never exceeds |signedFloors|. The branches
	// above satisfy that algebraically; the clamp is here for the same reason
	// displacement() has one - `(d * h) / h` is not `d` in IEEE 754, so the
	// conversion back to floors can round up past the leg.
	double dMinFloors = dMinM / floorHeightM;
	double legFloors = std::abs(signedFloors);
	return (dMinFloors > legFloors) ? legFloors : dMinFloors;
}

const char* KinematicTravelTimeStrategy::name() const
{
	return "kinematic";
}

TravelDescription KinematicTravelTimeStrategy::describe() const
{
	TravelDescription d;
	d.model = name();
	d.floorHeightM = floorHeightM;
	d.ratedSpeedMs = dynamics.ratedSpeedMs;
	d.accelMs2 = dynamics.accelMs2;
	d.jerkMs3 = dynamics.jerkMs3;
	return d;
}

} // namespace openEGCS
