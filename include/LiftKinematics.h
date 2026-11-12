/*
LiftKinematics.h

Ideal lift motion profiles (jerk-limited trapezoidal speed curve), used to
turn a travel distance into a physically plausible velocity/acceleration
profile over time. This drives BOTH the energy model (LiftMotorModel) and the
event timeline itself, through ITravelTimeStrategy/KinematicTravelTimeStrategy
-- so one time unit in this simulator is one second.

Sources:
  - Barney, G. & Al-Sharif, L., "Elevator Traffic Handbook", 2nd ed.,
    Routledge, 2016: Table 5.2 "Typical lift dynamics" (p. 90) for default
    rated speed/acceleration/jerk by building travel; Appendix "Lift
    kinematics" (p. 407-419) for the Case I / Case II motion equations.
  - Peters, R. D., "Vertical Transportation Planning in Buildings", EngD
    thesis, Brunel University, 1998: Chapter 6 "Lift Kinematics"
    ("Condition A" / "Condition B", p. 6-4 to 6-11) -- the primary source
    the handbook's Appendix is itself drawn from; cross-checked here to be
    the same equations. The specific results this file is checked against
    (largest relative residual 1.3e-16 over Condition A and B, up and down,
    for the dynamics of both reference scenarios):
      * eq 6.36 (p. 6-7): total journey time = d/v + a/j + v/a for
        Condition A -- an independent anchor on buildProfile's totalTimeS.
      * eq 6.33 (p. 6-7): the cruise ends at t = d/v.
      * eq 6.50 (p. 6-10): Condition B holds for 2a^3/j^2 <= d <
        (a^2*v + v^2*j)/(j*a). That upper bound is the same expression
        peakVelocity() uses as its Case I threshold.
      * eq 6.16 (p. 6-5) and 6.60 (p. 6-10): D(t) = j*t^3/6 on the first
        jerk ramp; eq 6.20 (p. 6-6) and 6.64 (p. 6-10): D(t) =
        a^3/(6j^2) - a^2*t/(2j) + a*t^2/2 on the constant-acceleration
        slice. Peters gives D(t) per time slice in ABSOLUTE time, while
        displacementAt() below accumulates per slice in relative time, so
        the check between them is a change of variable rather than a
        restatement. These are the displacement equations an event timeline
        needs.
      * eq 6.102 (p. 6-13): a^2 <= v*j confirms rated acceleration is
        reached before rated speed, which Peters calls the condition for a
        sensible design. Checked: all six rows of the dynamics table in
        LiftKinematics.cpp satisfy it.

THE THREE CONDITIONS, AND WHY THERE IS NO FOURTH

Condition C (Peters section 6.4, p. 6-12) is the short run: too short to
reach even rated acceleration, so the cabin is still ramping acceleration
up when it has to start ramping it down. Peters derives it in full
(eq 6.81-6.101); Barney & Al-Sharif instead conclude it is impractical for
real buildings (p. 417) and leave it out. This file implements it, because
in this simulator it IS reached -- see the next paragraph. The additional
results used:

  * eq 6.81 (p. 6-12): Condition C holds for d < 2a^3/j^2. Per dynamics-table
    row: 1.2196, 1.2642, 1.4580, 1.2800, 1.5360 and 1.0667 m.
  * eq 6.82-6.85 (p. 6-12): the four phase boundaries are tau, 2*tau, 3*tau
    and 4*tau for tau = cbrt(d/2j) -- i.e. four jerk phases of EQUAL
    duration. Peters writes them in absolute time and in a different
    algebraic form ((4d/j)^(1/3), (27d/2j)^(1/3), (32d/j)^(1/3)); those are
    the same numbers, confirmed term by term.
  * Derived here independently rather than transcribed, so the two agree from
    both ends: integrating A(t) = j*t then A(t) = 2*j*tau - j*t gives
    D(2*tau) = j*tau^3, so d = 2*j*tau^3 by symmetry of the decel half, hence
    tau = cbrt(d/2j) (= eq 6.82), attained acceleration j*tau, peak velocity
    j*tau^2, total time 4*tau (= eq 6.85). Validity falls out of the same
    algebra: j*tau <= a is exactly d <= 2a^3/j^2 (= eq 6.81).
  * Continuity with Condition B at the boundary, checked numerically on both
    reference scenarios' dynamics: at d = 2a^3/j^2 Condition C gives
    tau = a/j, peak = a^2/j and total time 4a/j, which is what the
    Condition B branch returns there. The two branches meet, so a distance
    oscillating around the bound does not make the cost jump.

Why a run shorter than one floor happens at all: a leg does not always start
at a floor, because Elevator::addTask re-anchors the leg at the cabin's current
(fractional) position when an insertion changes which stop comes first. In the
saturated reference scenario a few hundred of several million legDuration calls
fall below the Condition C bound, the shortest at 0.2259 m. Energy is never
priced on such a run, because it is charged from a served stop, hence from a
whole floor.

Condition D does not exist: reaching rated speed without reaching rated
acceleration would be, in Peters' words (section 6.5, p. 6-13), "a non-sensical
design", and eq 6.102 (a^2 <= v*j) is his test that an installation does not
have that anomaly. Under it the three conditions partition every d > 0, in two
lines: the Condition A threshold is a*v/j + v^2/a, and a^2 <= v*j gives both
a*v/j >= a^3/j^2 and v^2/a >= a^3/j^2, so that threshold is >= 2a^3/j^2, the
Condition C bound -- with equality only when a^2 = v*j. No gap is left over.
That precondition is documented and checked against the dynamics table by
test, not defended at runtime: dynamics that violate it are outside the model.
*/

#ifndef __LIFTKINEMATICS__
#define __LIFTKINEMATICS__

namespace openEGCS {

// Rated dynamics of a lift installation.
struct LiftDynamics {
	double ratedSpeedMs;
	double accelMs2;
	double jerkMs3;
};

// Default (rated speed, acceleration, jerk) for a building whose total lift
// travel is totalTravelM meters, looked up from Table 5.2 (Barney &
// Al-Sharif, p. 90). Where the book gives a range for acceleration, the
// upper bound is used (more conservative in terms of energy). The book
// lists two overlapping rows for ~20 m travel (different drive classes);
// this lookup keeps only the higher-acceleration one to avoid ambiguity.
LiftDynamics defaultDynamicsFor(double totalTravelM);

// Peak velocity (m/s) actually reached over a single continuous run of
// distanceM meters (always positive; direction is handled by the caller),
// given the installation's rated speed/acceleration/jerk.
//
// Case I - rated speed attained (Barney eq. A.7 / Peters "Condition A"):
//   distanceM >= (accel^2*ratedSpeed + ratedSpeed^2*jerk) / (accel*jerk)
//   => peak velocity = ratedSpeed
// That threshold is exactly the upper bound of Peters' Condition B range
// (eq 6.50, p. 6-10), confirmed term by term: 4.6222 m for the 10-floor
// reference scenario's dynamics, 24.8333 m for the 20-floor one.
// Case II - rated acceleration attained, rated speed not attained (Barney
// eq. A.12 / Peters "Condition B", eq. 6.50): peak velocity derived from
// the zero-cruise trapezoid distance (Barney eq. A.24-A.28 with d4=0):
//   peakVelocity^2 + (accel^2/jerk)*peakVelocity - accel*distanceM = 0
// That root solves distanceM = peak^2/a + peak*a/j, which PRESUPPOSES rated
// acceleration is reached, i.e. peak >= a^2/j. Solving the quadratic at
// distanceM = 2a^3/j^2 gives peak == a^2/j exactly, so the presupposition
// fails for anything shorter - which is why the next case has to exist and
// not be approximated by this one.
// Case III - neither attained (Peters "Condition C", eq 6.81/6.82):
//   peak velocity = jerk * cbrt(distanceM / (2*jerk))^2
// See this file's header for the derivation. Used below its domain, the
// Case II root would make the profile travel further than asked (+88% of the
// distance at 0.05 m).
double peakVelocity(double distanceM, double ratedSpeedMs, double accelMs2, double jerkMs3);

// A signed motion profile (positive = moving up, negative = moving down)
// for a continuous run of `signedDistanceM` meters, built from the
// jerk-limited trapezoidal speed curve. `signedDistanceM == 0` yields a
// zero-duration, zero-velocity profile.
struct KinematicProfile {
	double signedDistanceM = 0;
	// Both `peakVelocityMs` and `attainedAccelMs2` are what this run ACTUALLY
	// reached, not what the installation is rated for - they coincide with the
	// rated figures only on a run long enough to get there. Named "attained"
	// to keep it apart from LiftDynamics::accelMs2, the rated one.
	double peakVelocityMs = 0;        // magnitude, sign applied when sampling
	double attainedAccelMs2 = 0;      // == LiftDynamics::accelMs2 in Condition A/B, below it in C
	double jerkMs3 = 0;
	double t1 = 0;               // jerk ramp duration (up to rated/attained accel)
	double t2 = 0;               // constant-acceleration duration (exactly 0 in Condition C, where it is a result and not an omission)
	double t4 = 0;               // cruise duration (0 in Case II)
	double totalTimeS = 0;       // 4*t1 + 2*t2 + t4
};

KinematicProfile buildProfile(double signedDistanceM, double ratedSpeedMs, double accelMs2, double jerkMs3);

// Instantaneous signed velocity (m/s) and acceleration (m/s^2) at time t
// (seconds) since the start of the run. Returns {0, 0} outside [0, totalTimeS].
struct MotionState {
	double velocityMs = 0;
	double accelMs2 = 0;
};
MotionState sampleProfile(const KinematicProfile& profile, double t);

// Signed distance (m) covered t seconds after the start of the run: the exact
// integral of sampleProfile's velocity, phase by phase (Peters eq 6.16/6.60 and
// 6.20/6.64 and their siblings; see this file's header). This is what a
// position-over-time caller needs: sampleProfile answers "how fast", and the
// event timeline needs "how far".
//
// Outside [0, totalTimeS] this is deliberately NOT symmetric with
// sampleProfile, which returns rest at both ends: a run that has not started
// has covered nothing, and one that has finished has covered everything. So
// t <= 0 gives 0 and t >= totalTimeS gives `profile.signedDistanceM` *itself*,
// returned literally rather than evaluated from the cubic. That is what makes
// the endpoint identity
//
//     displacementAt(p, p.totalTimeS) == p.signedDistanceM
//
// hold bit-for-bit instead of to within rounding - which the travel model
// needs, because an event's timestamp comes from a sum of whole-leg durations
// while the cabin's position comes from this function, and the invariant that
// a stop's ARRIVE fires with the cabin at that stop's floor is exactly the
// statement that the two agree at the end of a leg.
//
// A degenerate profile (totalTimeS <= 0, which buildProfile returns for a
// non-positive distance or non-positive dynamics while still copying
// signedDistanceM into it) gives 0 at every t, never the distance: a caller
// deriving a position must not teleport the cabin because the dynamics were
// unusable.
//
// Accuracy is exactly that of sampleProfile, for every distance:
// buildProfile solves Condition C rather than approximating it, so nothing
// here is approximate either.
double displacementAt(const KinematicProfile& profile, double t);

} // namespace openEGCS

#endif
