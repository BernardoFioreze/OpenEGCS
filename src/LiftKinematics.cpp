#include "LiftKinematics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace openEGCS {

namespace {

struct DynamicsRow {
	double maxTravelM;
	double ratedSpeedMs;
	double accelMs2;
	double jerkMs3;
};

// Barney & Al-Sharif, "Elevator Traffic Handbook", 2nd ed., Table 5.2, p. 90.
// Acceleration uses the upper bound of the book's range for each row. The
// book's two overlapping rows for ~20 m travel (0.4 and 0.4-0.7 m/s^2) are
// collapsed into one, keeping the higher acceleration.
const std::array<DynamicsRow, 6> kTypicalLiftDynamics = {{
	{ 20.0,  1.00, 0.7, 0.75 },
	{ 32.0,  1.60, 0.8, 0.90 },
	{ 50.0,  2.50, 0.9, 1.00 },
	{ 60.0,  3.00, 1.0, 1.25 },
	{ 100.0, 5.00, 1.2, 1.50 },
	{ std::numeric_limits<double>::infinity(), 6.00, 1.2, 1.80 },
}};

} // namespace

LiftDynamics defaultDynamicsFor(double totalTravelM) {
	for (const auto& row : kTypicalLiftDynamics) {
		if (totalTravelM <= row.maxTravelM) {
			return { row.ratedSpeedMs, row.accelMs2, row.jerkMs3 };
		}
	}
	const auto& last = kTypicalLiftDynamics.back();
	return { last.ratedSpeedMs, last.accelMs2, last.jerkMs3 };
}

double peakVelocity(double distanceM, double ratedSpeedMs, double accelMs2, double jerkMs3) {
	if (accelMs2 <= 0.0 || jerkMs3 <= 0.0 || ratedSpeedMs <= 0.0 || distanceM <= 0.0) {
		return 0.0; // degenerate input: no attainable peak velocity
	}

	double a = accelMs2, j = jerkMs3, v = ratedSpeedMs;

	// Case I (Barney eq. A.7 / Peters "Condition A"): rated speed attained.
	double caseIThresholdM = (a * a * v + v * v * j) / (a * j);
	if (distanceM >= caseIThresholdM) {
		return v;
	}

	// Case III (Peters "Condition C", eq 6.81): the run is too short to even
	// reach rated acceleration, so the cabin is still ramping acceleration up
	// when it has to start ramping it down. The profile is four jerk phases of
	// equal duration tau = cbrt(d/2j) (eq 6.82), over which it covers
	// d = 2*j*tau^3 - so the peak velocity is j*tau^2. See LiftKinematics.h for
	// the derivation and for why this branch exists at all: without it the
	// Case II root below would be used outside its own domain, and the profile
	// would travel further than it was asked to - up to +88% at 0.05 m.
	if (distanceM < 2.0 * a * a * a / (j * j)) {
		double tau = std::cbrt(distanceM / (2.0 * j));
		return j * tau * tau;
	}

	// Case II (Barney eq. A.12 / Peters "Condition B"): rated acceleration
	// attained, rated speed not attained. Positive root of
	// v_peak^2 + (a^2/j)*v_peak - a*distanceM = 0 (see LiftKinematics.h).
	double b = (a * a) / j;
	return (-b + std::sqrt(b * b + 4 * a * distanceM)) / 2.0;
}

KinematicProfile buildProfile(double signedDistanceM, double ratedSpeedMs, double accelMs2, double jerkMs3) {
	KinematicProfile profile;
	profile.signedDistanceM = signedDistanceM;
	profile.attainedAccelMs2 = accelMs2;
	profile.jerkMs3 = jerkMs3;

	double distanceM = std::abs(signedDistanceM);
	if (distanceM <= 0.0 || accelMs2 <= 0.0 || jerkMs3 <= 0.0 || ratedSpeedMs <= 0.0) {
		// A non-positive ratedSpeedMs would otherwise make peakVelocity()
		// return 0 (Case I), then t4 = distanceM / 0 = +inf below - and
		// casting +inf to int in tripEnergyJ's step count is undefined
		// behaviour. Unreachable via defaultDynamicsFor()/the CLI (always
		// >= 1.0 m/s), but guarded here since this is a public function.
		return profile; // zero-duration, zero-velocity profile
	}

	double peak = peakVelocity(distanceM, ratedSpeedMs, accelMs2, jerkMs3);
	profile.peakVelocityMs = peak;

	// Condition C (Peters eq 6.81-6.85): four jerk phases of equal duration,
	// no constant-acceleration slice and no cruise. Returns early because
	// rampDistanceM/t4 below are Case II expressions, valid only for a peak
	// that reached rated acceleration.
	//
	// The one thing that makes this three lines instead of a rewrite: the
	// acceleration stored here is the ATTAINED one (j*tau < accelMs2), and with
	// that substitution sampleProfile's and displacementAt's existing phase
	// expressions become the Condition C ones - the jump between their phase 1
	// and phase 3 closes, and the distance comes out exactly d.
	if (distanceM < 2.0 * accelMs2 * accelMs2 * accelMs2 / (jerkMs3 * jerkMs3)) {
		double tau = std::cbrt(distanceM / (2.0 * jerkMs3));
		profile.attainedAccelMs2 = jerkMs3 * tau;
		profile.t1 = tau;
		profile.t2 = 0.0;	// a result here, not an omission: Condition C has no such phase
		profile.t4 = 0.0;
		profile.totalTimeS = 4.0 * tau;	// eq 6.85, (32*d/j)^(1/3) in Peters' form
		return profile;
	}

	double t1 = accelMs2 / jerkMs3;
	// Zero only on the Condition B/C boundary, where peak == a^2/j exactly and
	// the two branches agree.
	double t2 = std::max(0.0, peak / accelMs2 - t1);
	// Distance covered by both accel and decel ramps (Barney eq. A.28,
	// with v = peak): peak^2/a + peak*a/j.
	double rampDistanceM = (peak * peak) / accelMs2 + (peak * accelMs2) / jerkMs3;
	double t4 = std::max(0.0, (distanceM - rampDistanceM) / peak);

	profile.t1 = t1;
	profile.t2 = t2;
	profile.t4 = t4;
	profile.totalTimeS = 4 * t1 + 2 * t2 + t4;
	return profile;
}

MotionState sampleProfile(const KinematicProfile& profile, double t) {
	if (profile.totalTimeS <= 0.0 || t < 0.0 || t > profile.totalTimeS) {
		return {};
	}

	double sign = (profile.signedDistanceM < 0.0) ? -1.0 : 1.0;
	double a = profile.attainedAccelMs2, j = profile.jerkMs3, peak = profile.peakVelocityMs;
	double t1 = profile.t1, t2 = profile.t2, t4 = profile.t4;

	double T1 = t1;
	double T2 = T1 + t2;
	double T3 = T2 + t1;
	double T4 = T3 + t4;
	double T5 = T4 + t1;
	double T6 = T5 + t2;
	// T7 (= T6 + t1) equals profile.totalTimeS.

	double v, acc;
	if (t <= T1) {
		double tau = t;
		acc = j * tau;
		v = 0.5 * j * tau * tau;
	} else if (t <= T2) {
		double tau = t - T1;
		acc = a;
		v = (a * a) / (2 * j) + a * tau;
	} else if (t <= T3) {
		double tau = t - T2;
		acc = a - j * tau;
		v = (peak - (a * a) / (2 * j)) + a * tau - 0.5 * j * tau * tau;
	} else if (t <= T4) {
		acc = 0.0;
		v = peak;
	} else if (t <= T5) {
		double tau = t - T4;
		acc = -j * tau;
		v = peak - 0.5 * j * tau * tau;
	} else if (t <= T6) {
		double tau = t - T5;
		acc = -a;
		v = (peak - (a * a) / (2 * j)) - a * tau;
	} else {
		double tau = t - T6;
		acc = -a + j * tau;
		v = (a * a) / (2 * j) - a * tau + 0.5 * j * tau * tau;
	}

	return { sign * v, sign * acc };
}

double displacementAt(const KinematicProfile& profile, double t) {
	// Deliberately NOT symmetric with sampleProfile at the ends, and the
	// endpoint is returned literally rather than evaluated - see the contract
	// in LiftKinematics.h, which explains why the "cabin is at its floor when
	// the stop's ARRIVE fires" invariant depends on that being bit-exact.
	if (profile.totalTimeS <= 0.0) {
		return 0.0; // degenerate profile: nothing moves, however far it was asked to go
	}
	if (t <= 0.0) {
		return 0.0;
	}
	if (t >= profile.totalTimeS) {
		return profile.signedDistanceM;
	}

	double sign = (profile.signedDistanceM < 0.0) ? -1.0 : 1.0;
	double a = profile.attainedAccelMs2, j = profile.jerkMs3, peak = profile.peakVelocityMs;
	double t1 = profile.t1, t2 = profile.t2, t4 = profile.t4;

	// Same phase boundaries as sampleProfile, in the same order, on purpose:
	// the two functions are an integral apart and have to be read side by side.
	double T1 = t1;
	double T2 = T1 + t2;
	double T3 = T2 + t1;
	double T4 = T3 + t4;
	double T5 = T4 + t1;
	double T6 = T5 + t2;

	// Velocity at the end of the first jerk ramp, which is also the constant
	// term of every slice that starts from rated acceleration (a^2/2j in
	// Peters' notation).
	double c = (a * a) / (2 * j);

	// Distance covered by each *completed* phase. d1 is Peters eq 6.16/6.60
	// (j*t^3/6); the rest are the same integrals evaluated over their own
	// duration. d3 and d5 mirror each other, as do d2 and d6: the deceleration
	// ramp covers the same ground as the acceleration ramp.
	double d1 = j * t1 * t1 * t1 / 6.0;
	double d2 = c * t2 + 0.5 * a * t2 * t2;
	double d3 = (peak - c) * t1 + 0.5 * a * t1 * t1 - j * t1 * t1 * t1 / 6.0;
	double d4 = peak * t4;
	double d5 = peak * t1 - j * t1 * t1 * t1 / 6.0;
	double d6 = (peak - c) * t2 - 0.5 * a * t2 * t2;

	double s, tau;
	if (t <= T1) {
		tau = t;
		s = j * tau * tau * tau / 6.0;
	} else if (t <= T2) {
		tau = t - T1;
		s = d1 + c * tau + 0.5 * a * tau * tau;
	} else if (t <= T3) {
		tau = t - T2;
		s = d1 + d2 + (peak - c) * tau + 0.5 * a * tau * tau - j * tau * tau * tau / 6.0;
	} else if (t <= T4) {
		tau = t - T3;
		s = d1 + d2 + d3 + peak * tau;
	} else if (t <= T5) {
		tau = t - T4;
		s = d1 + d2 + d3 + d4 + peak * tau - j * tau * tau * tau / 6.0;
	} else if (t <= T6) {
		tau = t - T5;
		s = d1 + d2 + d3 + d4 + d5 + (peak - c) * tau - 0.5 * a * tau * tau;
	} else {
		tau = t - T6;
		s = d1 + d2 + d3 + d4 + d5 + d6 + c * tau - 0.5 * a * tau * tau + j * tau * tau * tau / 6.0;
	}

	return sign * s;
}

} // namespace openEGCS
