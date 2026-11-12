/*
UpPeakTrafficProfile.cpp

See UpPeakTrafficProfile.h. Curve and OD both cited there (Barney & Al-Sharif
2nd ed., Table 17.2 and Definition 4.1).
*/

#include "UpPeakTrafficProfile.h"
#include <cmath>

namespace openEGCS {
namespace examples {

namespace {
	// Table 17.2 "Arrivals by period during uppeak", persons per 5-minute period.
	const int kUpPeakCurve[12] = {29, 38, 50, 63, 76, 91, 105, 120, 81, 52, 35, 28};
	const double kPeriodSeconds = 300.0;	// one 5-minute period
	const double kHourSeconds = 3600.0;		// twelve periods
	const int kPeakArrivals = 120;			// max of the curve (period 8)
} // namespace

UpPeakTrafficProfile::UpPeakTrafficProfile(double scale_arg)
	: PeakTrafficProfile((kPeakArrivals / kPeriodSeconds) * scale_arg),
	  scale(scale_arg)
{
}

double UpPeakTrafficProfile::rate(double t) const
{
	double tt = (t < 0.0) ? 0.0 : t;
	double within = std::fmod(tt, kHourSeconds);
	int idx = static_cast<int>(within / kPeriodSeconds);
	if (idx < 0) idx = 0;
	if (idx > 11) idx = 11;
	return (kUpPeakCurve[idx] / kPeriodSeconds) * scale;
}

void UpPeakTrafficProfile::sampleTrip(double t, IRandomProvider& rng, int maxFloors,
	int& origin, int& destination) const
{
	(void)t;
	// Up peak: origin the main terminal, destination anywhere above it.
	origin = 0;
	destination = rng.nextUniformInt(1, maxFloors);
}

std::string UpPeakTrafficProfile::name() const
{
	return "up-peak (Barney)";
}

} // namespace examples
} // namespace openEGCS
