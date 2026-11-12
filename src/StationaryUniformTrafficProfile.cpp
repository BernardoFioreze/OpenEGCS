/*
StationaryUniformTrafficProfile.cpp

See StationaryUniformTrafficProfile.h. The draw order here is load-bearing for
determinism: it reproduces exactly what Simulator::generateNextArrival and
CustomerFactory::createArrival did before ITrafficProfile existed.
*/

#include "StationaryUniformTrafficProfile.h"

namespace openEGCS {

StationaryUniformTrafficProfile::StationaryUniformTrafficProfile(double arrival_rate)
	: rate(arrival_rate)
{
}

double StationaryUniformTrafficProfile::nextInterArrivalTime(double now, IRandomProvider& rng) const
{
	(void)now;
	return rng.nextExponential(rate);
}

void StationaryUniformTrafficProfile::sampleTrip(double t, IRandomProvider& rng, int maxFloors,
	int& origin, int& destination) const
{
	(void)t;
	origin = rng.nextUniformInt(0, maxFloors);
	destination = rng.nextUniformInt(0, maxFloors);
	while (destination == origin) {
		destination = rng.nextUniformInt(0, maxFloors);
	}
}

std::string StationaryUniformTrafficProfile::name() const
{
	return "stationary-uniform";
}

} // namespace openEGCS
