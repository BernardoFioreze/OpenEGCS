/*
PeakTrafficProfile.cpp

See PeakTrafficProfile.h. The thinning loop and the uniform-from-integer helper
are the two pieces worth reading here.
*/

#include "PeakTrafficProfile.h"

namespace openEGCS {
namespace examples {

PeakTrafficProfile::PeakTrafficProfile(double lambda_max)
	: lambdaMax(lambda_max)
{
}

double PeakTrafficProfile::uniform01(IRandomProvider& rng)
{
	// [0, kDenominator-1] / kDenominator lands in [0, 1). One integer draw.
	const int kDenominator = 1000000;
	return rng.nextUniformInt(0, kDenominator - 1) / static_cast<double>(kDenominator);
}

double PeakTrafficProfile::nextInterArrivalTime(double now, IRandomProvider& rng) const
{
	// Lewis-Shedler thinning: generate the ceiling process at lambdaMax and
	// accept a candidate at time t with probability rate(t)/lambdaMax. Written
	// as u * lambdaMax <= rate(t) to avoid a division. rate(t) > 0 for all t is
	// the subclass's responsibility, so this loop always terminates.
	double t = now;
	for (;;) {
		t += rng.nextExponential(lambdaMax);
		double u = uniform01(rng);
		if (u * lambdaMax <= rate(t)) {
			return t - now;
		}
	}
}

} // namespace examples
} // namespace openEGCS
