#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include "LiftKinematics.h"

using namespace openEGCS;
using Catch::Approx;

// Characterization tests for the jerk-limited motion profile (LiftKinematics.h).
// Reference values cross-checked against:
//  - Barney & Al-Sharif, "Elevator Traffic Handbook", 2nd ed., Appendix
//    "Lift kinematics" (Figure A.5/A.6/A.8, p. 413-415) and Table 5.2 (p. 90).
//  - Peters, "Vertical Transportation Planning in Buildings" (Brunel EngD
//    thesis, 1998), Chapter 6 "Lift Kinematics" ("Condition A"/"Condition B").

TEST_CASE("defaultDynamicsFor looks up Table 5.2 by total building travel", "[lift_kinematics]") {
	// <=20 m -> the book's higher-acceleration row for ~20 m travel (0.7/0.75).
	LiftDynamics d20 = defaultDynamicsFor(20.0);
	REQUIRE(d20.ratedSpeedMs == Approx(1.00));
	REQUIRE(d20.accelMs2 == Approx(0.7));
	REQUIRE(d20.jerkMs3 == Approx(0.75));

	LiftDynamics d32 = defaultDynamicsFor(32.0);
	REQUIRE(d32.ratedSpeedMs == Approx(1.60));

	// Beyond the table's last bracket (>100 m).
	LiftDynamics dTall = defaultDynamicsFor(500.0);
	REQUIRE(dTall.ratedSpeedMs == Approx(6.00));
	REQUIRE(dTall.accelMs2 == Approx(1.2));
	REQUIRE(dTall.jerkMs3 == Approx(1.80));
}

TEST_CASE("peakVelocity reaches rated speed in a one-floor jump (Barney Figure A.5)", "[lift_kinematics]") {
	// v=1.6 m/s, a=1 m/s^2, j=1 m/s^3 attains rated speed in a single floor
	// jump of 4.5 m (Barney & Al-Sharif Appendix, Figure A.5/A.6 example).
	double v_peak = peakVelocity(4.5, 1.6, 1.0, 1.0);
	REQUIRE(v_peak == Approx(1.6));
}

TEST_CASE("peakVelocity falls short of rated speed on a short hop (Case II)", "[lift_kinematics]") {
	// Same dynamics as above but a much shorter hop: rated speed of 1.6 m/s
	// is not attainable in 2 m, only rated acceleration is.
	double v_peak = peakVelocity(2.0, 1.6, 1.0, 1.0);
	REQUIRE(v_peak < 1.6);
	REQUIRE(v_peak > 0.0);

	// Cross-check against the closed-form solution of
	// v_peak^2 + (a^2/j)*v_peak - a*d = 0 for a=j=1, d=2: v^2+v-2=0 -> v=1.
	REQUIRE(v_peak == Approx(1.0));
}

TEST_CASE("peakVelocity is continuous across the Case I / Case II boundary", "[lift_kinematics]") {
	double a = 1.0, j = 1.0, v = 1.6;
	double thresholdM = (a * a * v + v * v * j) / (a * j);

	double justBelow = peakVelocity(thresholdM - 1e-6, v, a, j);
	double justAbove = peakVelocity(thresholdM + 1e-6, v, a, j);
	REQUIRE(justBelow == Approx(v).margin(1e-3));
	REQUIRE(justAbove == Approx(v).margin(1e-3));
}

TEST_CASE("peakVelocity/buildProfile reject degenerate (non-positive) rated speed or dynamics", "[lift_kinematics]") {
	// A non-positive ratedSpeedMs would otherwise make buildProfile's
	// t4 = distanceM / peakVelocity divide by zero (+inf), which later casts
	// to int as an undefined-behaviour step count in tripEnergyJ. Guard
	// unreachable via defaultDynamicsFor()/the CLI, but must hold for any
	// direct caller of these public functions.
	REQUIRE(peakVelocity(4.5, /*ratedSpeedMs=*/0.0, 1.0, 1.0) == 0.0);
	REQUIRE(peakVelocity(4.5, -1.0, 1.0, 1.0) == 0.0);
	REQUIRE(peakVelocity(4.5, 1.6, /*accelMs2=*/0.0, 1.0) == 0.0);
	REQUIRE(peakVelocity(4.5, 1.6, 1.0, /*jerkMs3=*/0.0) == 0.0);

	KinematicProfile p = buildProfile(4.5, /*ratedSpeedMs=*/0.0, 1.0, 1.0);
	REQUIRE(p.totalTimeS == 0.0);
	REQUIRE(std::isfinite(p.totalTimeS));
}

TEST_CASE("buildProfile of zero distance is a zero-duration profile", "[lift_kinematics]") {
	KinematicProfile p = buildProfile(0.0, 1.6, 1.0, 1.0);
	REQUIRE(p.totalTimeS == 0.0);

	MotionState state = sampleProfile(p, 0.0);
	REQUIRE(state.velocityMs == 0.0);
	REQUIRE(state.accelMs2 == 0.0);
}

TEST_CASE("buildProfile starts and ends at rest", "[lift_kinematics]") {
	KinematicProfile p = buildProfile(4.5, 1.6, 1.0, 1.0);
	REQUIRE(p.totalTimeS > 0.0);

	MotionState start = sampleProfile(p, 0.0);
	MotionState end = sampleProfile(p, p.totalTimeS);
	REQUIRE(start.velocityMs == Approx(0.0).margin(1e-9));
	REQUIRE(start.accelMs2 == Approx(0.0).margin(1e-9));
	REQUIRE(end.velocityMs == Approx(0.0).margin(1e-9));
	REQUIRE(end.accelMs2 == Approx(0.0).margin(1e-9));
}

TEST_CASE("buildProfile reaches (and holds) peak velocity when a cruise phase exists", "[lift_kinematics]") {
	// A long run (well past the Case I threshold) has a genuine cruise phase.
	KinematicProfile p = buildProfile(50.0, 1.6, 1.0, 1.0);
	REQUIRE(p.t4 > 0.0);

	double midCruiseT = p.t1 + p.t2 + p.t1 + p.t4 / 2.0;
	MotionState mid = sampleProfile(p, midCruiseT);
	REQUIRE(mid.velocityMs == Approx(1.6).margin(1e-6));
	REQUIRE(mid.accelMs2 == Approx(0.0).margin(1e-6));
}

TEST_CASE("buildProfile has no cruise phase in Case II", "[lift_kinematics]") {
	KinematicProfile p = buildProfile(2.0, 1.6, 1.0, 1.0);
	REQUIRE(p.t4 == Approx(0.0).margin(1e-9));
}

TEST_CASE("sampleProfile flips sign for downward travel", "[lift_kinematics]") {
	KinematicProfile up = buildProfile(4.5, 1.6, 1.0, 1.0);
	KinematicProfile down = buildProfile(-4.5, 1.6, 1.0, 1.0);

	double midT = up.totalTimeS / 2.0;
	MotionState upState = sampleProfile(up, midT);
	MotionState downState = sampleProfile(down, midT);

	REQUIRE(downState.velocityMs == Approx(-upState.velocityMs));
	REQUIRE(downState.accelMs2 == Approx(-upState.accelMs2));
}

TEST_CASE("sampleProfile outside [0, totalTimeS] returns rest", "[lift_kinematics]") {
	KinematicProfile p = buildProfile(4.5, 1.6, 1.0, 1.0);
	MotionState before = sampleProfile(p, -1.0);
	MotionState after = sampleProfile(p, p.totalTimeS + 1.0);
	REQUIRE(before.velocityMs == 0.0);
	REQUIRE(after.velocityMs == 0.0);
}

TEST_CASE("integrating the velocity profile over time reproduces the travel distance", "[lift_kinematics]") {
	// Numerically integrate v(t) and compare against the requested distance,
	// for both Case I (long run) and Case II (short hop).
	auto integratedDistance = [](const KinematicProfile& p, int steps) {
		double dt = p.totalTimeS / steps;
		double d = 0.0;
		double prev = sampleProfile(p, 0.0).velocityMs;
		for (int i = 1; i <= steps; ++i) {
			double cur = sampleProfile(p, i * dt).velocityMs;
			d += 0.5 * (prev + cur) * dt;
			prev = cur;
		}
		return d;
	};

	KinematicProfile caseI = buildProfile(50.0, 1.6, 1.0, 1.0);
	REQUIRE(integratedDistance(caseI, 2000) == Approx(50.0).epsilon(0.001));

	KinematicProfile caseII = buildProfile(2.0, 1.6, 1.0, 1.0);
	REQUIRE(integratedDistance(caseII, 2000) == Approx(2.0).epsilon(0.001));
}

// ---------------------------------------------------------------------------
// displacementAt: "how far", where sampleProfile answers "how fast". This is
// what lets the event timeline be driven by the profile. The last two cases
// are a literature anchor on the *kinematics* (Peters' equations), alongside
// the energy anchor in test_lift_motor_model.cpp (Peters' worked example,
// Et = 1.911e5 J).
// ---------------------------------------------------------------------------

namespace {

// The dynamics of the two reference scenarios: defaultDynamicsFor(9 * 3.5) and
// defaultDynamicsFor(19 * 3.5), i.e. the 10- and 20-floor buildings the
// project benchmarks with.
const LiftDynamics kSmallBuilding = { 1.60, 0.8, 0.90 };
const LiftDynamics kTallBuilding = { 5.00, 1.2, 1.50 };

double integratedVelocity(const KinematicProfile& p, int steps) {
	double dt = p.totalTimeS / steps;
	double d = 0.0;
	double prev = sampleProfile(p, 0.0).velocityMs;
	for (int i = 1; i <= steps; ++i) {
		double cur = sampleProfile(p, i * dt).velocityMs;
		d += 0.5 * (prev + cur) * dt;
		prev = cur;
	}
	return d;
}

} // namespace

TEST_CASE("displacementAt reaches exactly the requested distance at the end of the run", "[lift_kinematics]") {
	// The endpoint identity, which the "cabin is at its floor when the stop's
	// ARRIVE fires" invariant depends on: an event's timestamp comes from a sum
	// of whole-leg durations while the cabin's position comes from
	// displacementAt, so that invariant is the statement that the two agree at a
	// leg's end. It is bit-exact rather than approximate because the function
	// returns signedDistanceM itself past totalTimeS instead of evaluating the
	// cubic.
	const double distances[] = { 3.5, 7.0, 17.5, 31.5, -3.5, -17.5, 59.5 };
	for (double d : distances) {
		KinematicProfile p = buildProfile(d, kSmallBuilding.ratedSpeedMs,
			kSmallBuilding.accelMs2, kSmallBuilding.jerkMs3);
		REQUIRE(displacementAt(p, p.totalTimeS) == d);		// exact, not Approx
		REQUIRE(displacementAt(p, p.totalTimeS + 1.0) == d);
	}
}

TEST_CASE("displacementAt starts at zero and never goes backwards", "[lift_kinematics]") {
	// Monotonicity. A cabin that moves backwards mid-leg would break the
	// position clamp Elevator::update relies on, so this is not decoration.
	KinematicProfile caseI = buildProfile(31.5, kSmallBuilding.ratedSpeedMs,
		kSmallBuilding.accelMs2, kSmallBuilding.jerkMs3);
	KinematicProfile caseII = buildProfile(3.5, kSmallBuilding.ratedSpeedMs,
		kSmallBuilding.accelMs2, kSmallBuilding.jerkMs3);

	for (const KinematicProfile& p : { caseI, caseII }) {
		REQUIRE(displacementAt(p, 0.0) == 0.0);
		REQUIRE(displacementAt(p, -1.0) == 0.0);
		double last = 0.0;
		for (int i = 0; i <= 1000; ++i) {
			double s = displacementAt(p, p.totalTimeS * i / 1000.0);
			REQUIRE(s >= last - 1e-12);
			last = s;
		}
	}
}

TEST_CASE("displacementAt is the integral of sampleProfile's velocity", "[lift_kinematics]") {
	// Gate 2, continued: the two functions are an integral apart, and nothing
	// but this case stops them from drifting if one is edited alone.
	KinematicProfile caseI = buildProfile(31.5, kSmallBuilding.ratedSpeedMs,
		kSmallBuilding.accelMs2, kSmallBuilding.jerkMs3);
	KinematicProfile caseII = buildProfile(3.5, kSmallBuilding.ratedSpeedMs,
		kSmallBuilding.accelMs2, kSmallBuilding.jerkMs3);

	// Just inside the end, so the closed form is exercised rather than the
	// literal endpoint shortcut.
	double insideI = caseI.totalTimeS * (1 - 1e-12);
	double insideII = caseII.totalTimeS * (1 - 1e-12);
	REQUIRE(displacementAt(caseI, insideI) == Approx(integratedVelocity(caseI, 200000)).epsilon(1e-6));
	REQUIRE(displacementAt(caseII, insideII) == Approx(integratedVelocity(caseII, 200000)).epsilon(1e-6));

	// And at an arbitrary interior instant, not only at the end.
	KinematicProfile p = buildProfile(17.5, kTallBuilding.ratedSpeedMs,
		kTallBuilding.accelMs2, kTallBuilding.jerkMs3);
	double tMid = p.totalTimeS * 0.4;
	KinematicProfile truncated = p;
	truncated.totalTimeS = tMid;		// integrate only up to tMid
	REQUIRE(displacementAt(p, tMid) == Approx(integratedVelocity(truncated, 200000)).epsilon(1e-6));
}

TEST_CASE("displacementAt matches Peters' per-slice distance equations", "[lift_kinematics][validation]") {
	// Gate 3. Peters, "Vertical Transportation Planning in Buildings" (Brunel
	// EngD thesis, 1998), Chapter 6, gives D(t) per time slice in ABSOLUTE
	// time; displacementAt accumulates per slice in relative time, so this is
	// a change of variable, not a restatement.
	//   eq 6.16 (p. 6-5) / 6.60 (p. 6-10): D(t) = j*t^3/6 on the first ramp.
	//   eq 6.20 (p. 6-6) / 6.64 (p. 6-10): D(t) = a^3/(6j^2) - a^2*t/(2j)
	//                                             + a*t^2/2 on the next slice.
	const LiftDynamics dyns[] = { kSmallBuilding, kTallBuilding };
	const double distances[] = { 3.5, 7.0, 31.5, 59.5 };
	for (const LiftDynamics& dyn : dyns) {
		for (double dist : distances) {
			KinematicProfile p = buildProfile(dist, dyn.ratedSpeedMs, dyn.accelMs2, dyn.jerkMs3);
			double a = dyn.accelMs2, j = dyn.jerkMs3;

			double tRamp = p.t1 * 0.7;		// inside slice 1
			REQUIRE(displacementAt(p, tRamp) == Approx(j * tRamp * tRamp * tRamp / 6.0).epsilon(1e-12));

			if (p.t2 > 1e-9) {
				double t = p.t1 + p.t2 * 0.6;	// inside slice 2, absolute time
				double peters = a * a * a / (6 * j * j) - a * a * t / (2 * j) + a * t * t / 2.0;
				REQUIRE(displacementAt(p, t) == Approx(peters).epsilon(1e-12));
			}
		}
	}
}

TEST_CASE("buildProfile's journey time matches Peters' closed form", "[lift_kinematics][validation]") {
	// Gate 4, and the piece the suite was missing entirely: every other
	// kinematics case here tests a structural property (starts at rest, has a
	// cruise, Case II has none). This is a closed-form expression for the
	// WHOLE of totalTimeS, independent of the implementation.
	//   eq 6.36 (p. 6-7): t7 = d/v + a/j + v/a         (Condition A)
	//   eq 6.33 (p. 6-7): the cruise ends at t = d/v
	//   eq 6.29 (p. 6-7): d = 2*D(t3) + v*(t4 - t3)
	const LiftDynamics dyns[] = { kSmallBuilding, kTallBuilding };
	const double distances[] = { 17.5, 31.5, 59.5, 105.0 };
	for (const LiftDynamics& dyn : dyns) {
		for (double d : distances) {
			KinematicProfile p = buildProfile(d, dyn.ratedSpeedMs, dyn.accelMs2, dyn.jerkMs3);
			if (p.t4 <= 0.0) continue;		// Condition A only
			double a = dyn.accelMs2, j = dyn.jerkMs3, v = dyn.ratedSpeedMs;

			REQUIRE(p.totalTimeS == Approx(d / v + a / j + v / a).epsilon(1e-12));

			double t3 = 2 * p.t1 + p.t2;	// absolute instant the cruise starts
			REQUIRE(t3 + p.t4 == Approx(d / v).epsilon(1e-12));
			REQUIRE(2 * displacementAt(p, t3) + v * p.t4 == Approx(d).epsilon(1e-12));
		}
	}
}

TEST_CASE("the dynamics table satisfies Peters' sensible-design condition", "[lift_kinematics][validation]") {
	// eq 6.102 (p. 6-13): a^2 <= v*j confirms rated acceleration is reached
	// before rated speed. Peters excludes the opposite case as "a non-sensical
	// design", and buildProfile assumes it throughout. This guards the project's
	// own Table 5.2 DATA rather than its code, so it cannot go stale.
	const double travels[] = { 15.0, 25.0, 40.0, 55.0, 80.0, 150.0, 500.0 };
	for (double travel : travels) {
		LiftDynamics d = defaultDynamicsFor(travel);
		REQUIRE(d.accelMs2 * d.accelMs2 <= d.ratedSpeedMs * d.jerkMs3);
	}
}

TEST_CASE("buildProfile solves Condition C instead of approximating it", "[lift_kinematics]") {
	// Below Peters' eq 6.81 bound (d < 2a^3/j^2, i.e. 1.2642 m for these
	// dynamics) the profile must solve Condition C. Approximated by the Case II
	// expressions, it would travel FURTHER than asked - 1.0790 m for a requested
	// 1.0 m (+7.9%), 0.6578 for 0.5 (+31.6%), 0.0931 for 0.05 m (+86.3%).
	//
	// The range is reachable in the simulator: a leg re-anchored mid-flight starts
	// at a fractional position (see the header of LiftKinematics.h), and runs as
	// short as 0.2259 m occur.
	double bound = 2 * std::pow(kSmallBuilding.accelMs2, 3) / std::pow(kSmallBuilding.jerkMs3, 2);
	REQUIRE(bound == Approx(1.2642).epsilon(1e-4));

	const double shortRuns[] = { 1.0, 0.5, 0.2259, 0.05 };
	for (double d : shortRuns) {
		REQUIRE(d < bound);
		KinematicProfile p = buildProfile(d, kSmallBuilding.ratedSpeedMs,
			kSmallBuilding.accelMs2, kSmallBuilding.jerkMs3);

		// Four jerk phases of equal duration, no constant-acceleration slice
		// and no cruise (Peters eq 6.82-6.85). t2 == 0 held before the fix too,
		// but as a clamped negative; here it is the answer.
		REQUIRE(p.t2 == 0.0);
		REQUIRE(p.t4 == 0.0);
		REQUIRE(p.totalTimeS == Approx(4 * p.t1).epsilon(1e-12));

		// Rated acceleration is NOT attained, which is what defines the
		// condition - and the field says so, which it could not before the
		// rename (it carried the rated figure).
		REQUIRE(p.attainedAccelMs2 < kSmallBuilding.accelMs2);
		REQUIRE(p.attainedAccelMs2 == Approx(kSmallBuilding.jerkMs3 * p.t1).epsilon(1e-12));

		// The defect, gone: the profile travels exactly what it was asked to.
		// Checked through the integral of sampleProfile's own velocity, not
		// just through displacementAt, because the overshoot was sampleProfile's
		// and displacementAt only reported it faithfully.
		double inside = p.totalTimeS * (1 - 1e-12);
		double profileSays = displacementAt(p, inside);
		REQUIRE(profileSays == Approx(integratedVelocity(p, 200000)).epsilon(1e-6));

		// "Never further than asked", up to rounding. A bare `profileSays <= d`
		// fails on some toolchains (MinGW g++ -O3 on Windows): at 1e-12 of the leg
		// from its end the closed-form sum lands 2 ulps past 0.5 and 4 past 0.05 -
		// the position is d minus a j*t^3/6 tail far below one ulp, so which side of
		// d the sum rounds to is the toolchain's call. The engine never sees it:
		// displacement() clamps to the leg (KinematicTravelTimeStrategy.cpp). Eight
		// ulps is ~1e-15 relative; the defect this guards against is +7.9% to
		// +86.3%, and any algebraic overshoot is orders of magnitude above the
		// margin.
		double ulp = std::nextafter(d, 2.0 * d) - d;
		REQUIRE(profileSays <= d + 8 * ulp);
		REQUIRE(profileSays == Approx(d).epsilon(1e-9));
	}
}

TEST_CASE("Condition C's journey time matches Peters' closed form", "[lift_kinematics][validation]") {
	// Sibling of the Condition A case above, and an independent anchor for the
	// same reason: eq 6.85 (p. 6-12) gives the whole of totalTimeS in a form
	// the code does not use. The code computes 4*cbrt(d/2j); Peters writes
	// t4 = (32*d/j)^(1/3). Equal because 4 * 2^(-1/3) == 32^(1/3), which is
	// the point - if the implementation were restructured the identity would
	// still have to hold.
	//
	// eq 6.82 is checked the same way: t1 = (d/2j)^(1/3) against cbrt, and the
	// three intermediate boundaries (eq 6.83/6.84) against 2*t1 and 3*t1.
	const LiftDynamics dyns[] = { kSmallBuilding, kTallBuilding };
	for (const LiftDynamics& dyn : dyns) {
		double bound = 2 * std::pow(dyn.accelMs2, 3) / std::pow(dyn.jerkMs3, 2);
		const double fractions[] = { 0.02, 0.1, 0.35, 0.7, 0.99 };
		for (double f : fractions) {
			double d = bound * f;
			KinematicProfile p = buildProfile(d, dyn.ratedSpeedMs, dyn.accelMs2, dyn.jerkMs3);
			double j = dyn.jerkMs3;

			REQUIRE(p.totalTimeS == Approx(std::pow(32.0 * d / j, 1.0 / 3.0)).epsilon(1e-12));
			REQUIRE(p.t1 == Approx(std::pow(d / (2 * j), 1.0 / 3.0)).epsilon(1e-12));
			REQUIRE(2 * p.t1 == Approx(std::pow(4.0 * d / j, 1.0 / 3.0)).epsilon(1e-12));
			REQUIRE(3 * p.t1 == Approx(std::pow(27.0 * d / (2 * j), 1.0 / 3.0)).epsilon(1e-12));

			// And the velocity profile integrates to d, which is the property
			// the whole item was about.
			REQUIRE(integratedVelocity(p, 200000) == Approx(d).epsilon(1e-6));
		}
	}
}

TEST_CASE("Condition B and C agree on the boundary between them", "[lift_kinematics][validation]") {
	// Sibling of "peakVelocity is continuous across the Case I / Case II
	// boundary". It matters more than tidiness: legDuration feeds the dispatch
	// cost, so a jump at d = 2a^3/j^2 (eq 6.81) would make a leg whose length
	// oscillates around the bound flip between two costs. The two branches do
	// not merely approach each other - they return the same profile there.
	const LiftDynamics dyns[] = { kSmallBuilding, kTallBuilding };
	for (const LiftDynamics& dyn : dyns) {
		double a = dyn.accelMs2, j = dyn.jerkMs3, v = dyn.ratedSpeedMs;
		double bound = 2 * a * a * a / (j * j);

		KinematicProfile at = buildProfile(bound, v, a, j);
		REQUIRE(at.attainedAccelMs2 == Approx(a).epsilon(1e-12));	// rated, exactly
		REQUIRE(at.peakVelocityMs == Approx(a * a / j).epsilon(1e-12));
		REQUIRE(at.totalTimeS == Approx(4 * a / j).epsilon(1e-12));
		REQUIRE(at.t2 == Approx(0.0).margin(1e-12));

		KinematicProfile below = buildProfile(bound * (1 - 1e-9), v, a, j);
		KinematicProfile above = buildProfile(bound * (1 + 1e-9), v, a, j);
		REQUIRE(below.totalTimeS == Approx(above.totalTimeS).epsilon(1e-7));
		REQUIRE(below.peakVelocityMs == Approx(above.peakVelocityMs).epsilon(1e-7));
		REQUIRE(below.attainedAccelMs2 == Approx(above.attainedAccelMs2).epsilon(1e-7));

		// Below the bound rated acceleration is genuinely not reached; above it
		// is. The boundary is where that flips, and nowhere else.
		REQUIRE(below.attainedAccelMs2 < a);
		REQUIRE(above.attainedAccelMs2 == Approx(a).epsilon(1e-12));
	}
}
