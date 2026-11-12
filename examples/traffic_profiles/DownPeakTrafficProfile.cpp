/*
DownPeakTrafficProfile.cpp

See DownPeakTrafficProfile.h. The curve and the OD model are both cited there
(Barney & Al-Sharif 2nd ed., Table 17.4 and Definition 4.2).
*/

#include "DownPeakTrafficProfile.h"
#include <cmath>

namespace openEGCS {
namespace examples {

namespace {
	// Table 17.4 "Arrivals during down peak", persons per 5-minute period.
	const int kDownPeakCurve[12] = {22, 47, 180, 115, 71, 46, 33, 26, 24, 24, 23, 25};
	const double kPeriodSeconds = 300.0;	// one 5-minute period
	const double kHourSeconds = 3600.0;		// twelve periods
	const int kPeakArrivals = 180;			// max of the curve (period 3)
} // namespace

DownPeakTrafficProfile::DownPeakTrafficProfile(double scale_arg)
	// Ceiling rate for thinning = the curve's peak (persons/5-min / 300 s),
	// scaled. 180/300 = 0.6 arrivals/s at scale 1.0.
	: PeakTrafficProfile((kPeakArrivals / kPeriodSeconds) * scale_arg),
	  scale(scale_arg)
{
}

double DownPeakTrafficProfile::rate(double t) const
{
	double tt = (t < 0.0) ? 0.0 : t;
	double within = std::fmod(tt, kHourSeconds);	// periodic over the hour
	int idx = static_cast<int>(within / kPeriodSeconds);
	if (idx < 0) idx = 0;
	if (idx > 11) idx = 11;							// guard the float edge at exactly 3600 s
	return (kDownPeakCurve[idx] / kPeriodSeconds) * scale;
}

void DownPeakTrafficProfile::sampleTrip(double t, IRandomProvider& rng, int maxFloors,
	int& origin, int& destination) const
{
	(void)t;
	// Down peak: origin anywhere above the lobby, destination the main terminal.
	origin = rng.nextUniformInt(1, maxFloors);
	destination = 0;
}

std::string DownPeakTrafficProfile::name() const
{
	return "down-peak (Barney)";
}

} // namespace examples
} // namespace openEGCS
