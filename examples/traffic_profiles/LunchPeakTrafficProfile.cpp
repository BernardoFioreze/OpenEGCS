/*
LunchPeakTrafficProfile.cpp

See LunchPeakTrafficProfile.h. The two curves are digitised from Cortes 2004
Fig. 5 at 10-minute intervals; the OD probabilities are the section 4.1 list.
*/

#include "LunchPeakTrafficProfile.h"
#include <cmath>

namespace openEGCS {
namespace examples {

namespace {
	// Cortes 2004 Fig. 5, "arrival rate to the hall of the floors" (the exit
	// stream, one asymmetric bell PER upper floor), persons/min, read at
	// minutes 0,10,...,130. Peak ~0.37 near minute 30.
	const double kFloorsCurve[14] =
		{0.10, 0.27, 0.34, 0.37, 0.36, 0.31, 0.26, 0.21, 0.17, 0.13, 0.10, 0.075, 0.05, 0.035};

	// Cortes 2004 Fig. 5, "arrival rate to the hall of the ground floor" (the
	// return stream), persons/min: a trapezoid, zero until ~70 min, plateau
	// ~2.4 over ~[80,115], back to zero by ~120.
	const double kGroundCurve[14] =
		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.4, 2.4, 2.4, 2.4, 0.0, 0.0};

	const double kGridMinutes = 10.0;
	const double kSpanSeconds = 7800.0;		// 130 min: the paper's replication length

	// OD (section 4.1). From the ground floor:
	const double kGroundToAdmin = 0.15;		// to the 7th floor
	const double kGroundToTop = 0.04;		// to the 12th floor
	// (the rest: 9% each, the complement)
	// From any other floor:
	const double kOtherToGround = 0.95;		// to the ground floor
	// (the rest: 5% total, uniform among floors that are neither ground nor self)

	// Linear interpolation of a 14-point curve (minutes 0..130) at `minute`.
	double interpCurve(const double* curve, double minute) {
		int idx = static_cast<int>(minute / kGridMinutes);
		if (idx < 0) idx = 0;
		if (idx >= 13) return curve[13];
		double frac = (minute - idx * kGridMinutes) / kGridMinutes;
		return curve[idx] * (1.0 - frac) + curve[idx + 1] * frac;
	}

	// Minute within the periodic 130-minute window for absolute time `t` (s).
	double minuteOf(double t) {
		double within = std::fmod(t, kSpanSeconds);
		if (within < 0.0) within += kSpanSeconds;
		return within / 60.0;
	}
} // namespace

double LunchPeakTrafficProfile::ceilingRate(int floors, double scale)
{
	int nUpper = (floors >= 2) ? (floors - 1) : 1;
	double maxPerMin = 0.0;
	for (int i = 0; i < 14; i++) {
		double aggregate = nUpper * kFloorsCurve[i] + kGroundCurve[i];
		if (aggregate > maxPerMin) maxPerMin = aggregate;
	}
	// The sum of two piecewise-linear curves is piecewise-linear, so its maximum
	// is at a grid node - scanning the nodes gives the exact ceiling.
	return (maxPerMin / 60.0) * scale;
}

LunchPeakTrafficProfile::LunchPeakTrafficProfile(int floors, double scale_arg)
	: PeakTrafficProfile(ceilingRate(floors, scale_arg)),
	  nUpper((floors >= 2) ? (floors - 1) : 1),
	  adminIdx(0),
	  topIdx(0),
	  scale(scale_arg)
{
	adminIdx = (nUpper >= 6) ? 6 : nUpper;	// the 7th floor, clamped for a small building
	topIdx = nUpper;						// the 12th floor = the top upper floor
}

double LunchPeakTrafficProfile::floorsRatePerSec(double t) const
{
	return (interpCurve(kFloorsCurve, minuteOf(t)) / 60.0) * scale;
}

double LunchPeakTrafficProfile::groundRatePerSec(double t) const
{
	return (interpCurve(kGroundCurve, minuteOf(t)) / 60.0) * scale;
}

double LunchPeakTrafficProfile::rate(double t) const
{
	return nUpper * floorsRatePerSec(t) + groundRatePerSec(t);
}

namespace {
	// Uniform pick among the upper floors [1, nUpper] excluding `exclA` and
	// (when >= 1) `exclB`. Returns a valid upper floor; falls back to floor 1
	// only in the degenerate case of nothing left (a building too small to have
	// a "rest"), which the Cortes 12-floor scenario never reaches.
	int pickUpperExcluding(IRandomProvider& rng, int nUpper, int exclA, int exclB) {
		int count = nUpper;
		if (exclA >= 1 && exclA <= nUpper) count--;
		if (exclB >= 1 && exclB <= nUpper && exclB != exclA) count--;
		if (count <= 0) return 1;
		int k = rng.nextUniformInt(0, count - 1);
		for (int f = 1; f <= nUpper; f++) {
			if (f == exclA || f == exclB) continue;
			if (k == 0) return f;
			k--;
		}
		return 1;	// unreachable
	}
} // namespace

void LunchPeakTrafficProfile::sampleTrip(double t, IRandomProvider& rng, int maxFloors,
	int& origin, int& destination) const
{
	(void)maxFloors;	// uses the stored Cortes topology; run this with 12 floors

	double lf = floorsRatePerSec(t);
	double lg = groundRatePerSec(t);
	double lambdaUpper = nUpper * lf;
	double total = lambdaUpper + lg;

	double u = uniform01(rng);
	bool groundOrigin = (total > 0.0) && (u < lg / total);

	if (groundOrigin) {
		// Return stream: from the ground to an upper floor (section 4.1).
		origin = 0;
		double v = uniform01(rng);
		if (v < kGroundToAdmin) {
			destination = adminIdx;
		} else if (v < kGroundToAdmin + kGroundToTop) {
			destination = topIdx;
		} else {
			destination = pickUpperExcluding(rng, nUpper, adminIdx, topIdx);
		}
	} else {
		// Exit stream: from an upper floor, mostly down to the ground.
		origin = rng.nextUniformInt(1, nUpper);
		double v = uniform01(rng);
		if (v < kOtherToGround) {
			destination = 0;
		} else {
			destination = pickUpperExcluding(rng, nUpper, origin, -1);
		}
	}
}

std::string LunchPeakTrafficProfile::name() const
{
	return "lunch-peak (Cortes 2004)";
}

} // namespace examples
} // namespace openEGCS
