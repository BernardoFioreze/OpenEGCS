/*
StepMixTrafficProfile.cpp

See StepMixTrafficProfile.h. Bounds and mix from Beamurgia et al. (2016)
section 6.2 / Table 1; the shape of the step and the populated floors are
substitutions declared there.
*/

#include "StepMixTrafficProfile.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace openEGCS {
namespace examples {

namespace {

// The ceiling of the curve, in arrivals/s, validated first so the base class
// never sees a non-positive one.
double ceilingRate(const StepMixTrafficProfile::Params& p)
{
	if (!(p.populationPerFloor > 0.0) || p.populatedFloors < 1 || !(p.periodSeconds > 0.0)
		|| !(p.minPercent > 0.0) || !(p.maxPercent >= p.minPercent) || !(p.stepPercent > 0.0)) {
		throw std::invalid_argument("StepMixTrafficProfile: population, period and a positive, increasing step are required");
	}
	return (p.maxPercent / 100.0) * p.populationPerFloor * p.populatedFloors / p.periodSeconds;
}

} // namespace

StepMixTrafficProfile::StepMixTrafficProfile(Params parameters)
	: PeakTrafficProfile(ceilingRate(parameters)),
	  params(parameters),
	  population(parameters.populationPerFloor * parameters.populatedFloors)
{
	if (params.incoming < 0.0 || params.outgoing < 0.0 || params.interfloor < 0.0
		|| std::fabs(params.incoming + params.outgoing + params.interfloor - 1.0) > 1e-9) {
		throw std::invalid_argument("StepMixTrafficProfile: the incoming/outgoing/interfloor shares must be non-negative and add up to 1");
	}
	if (params.interfloor > 0.0 && params.populatedFloors < 2) {
		throw std::invalid_argument("StepMixTrafficProfile: interfloor traffic needs at least two populated floors");
	}
}

double StepMixTrafficProfile::percentAt(double t) const
{
	const double period = std::floor((t < 0.0 ? 0.0 : t) / params.periodSeconds);
	return std::min(params.maxPercent, params.minPercent + params.stepPercent * period);
}

double StepMixTrafficProfile::rate(double t) const
{
	return (percentAt(t) / 100.0) * population / params.periodSeconds;
}

void StepMixTrafficProfile::sampleTrip(double t, IRandomProvider& rng, int maxFloors,
	int& origin, int& destination) const
{
	(void)t;	// the mix does not change over the run
	if (maxFloors != params.populatedFloors) {
		throw std::invalid_argument("StepMixTrafficProfile: the run's floor count is not the profile's populated floors");
	}
	const int n = params.populatedFloors;
	const double u = uniform01(rng);
	if (u < params.incoming) {
		origin = 0;
		destination = rng.nextUniformInt(1, n);
	} else if (u < params.incoming + params.outgoing || params.interfloor <= 0.0) {
		// The second clause only catches rounding in a sum that is 1 within
		// 1e-9: with no interfloor share, nothing may fall through to it.
		origin = rng.nextUniformInt(1, n);
		destination = 0;
	} else {
		// Two distinct populated floors in one draw each: the second is drawn
		// from the n - 1 others and shifted past the origin.
		origin = rng.nextUniformInt(1, n);
		destination = rng.nextUniformInt(1, n - 1);
		if (destination >= origin) ++destination;
	}
}

std::string StepMixTrafficProfile::name() const
{
	return "step-mix (Beamurgia 2016)";
}

} // namespace examples
} // namespace openEGCS
