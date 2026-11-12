/*
StationaryUniformTrafficProfile.h

The default ITrafficProfile: a stationary Poisson arrival process at a fixed
rate, with origin and destination floors drawn independently and uniformly.

It holds the arrival rate (lambda) because nextInterArrivalTime takes no rate
argument: for a stationary process the rate is constant and belongs to the
profile. sampleTrip does not use lambda at all, which is why a caller that only
samples trips (e.g. a CustomerFactory constructed standalone in a test) can use
the default-rate constructor without the rate meaning anything.

No parameter here is a cited figure, because there is nothing to cite: this is
the degenerate "no pattern" profile. The named patterns (up/down/lunch peak)
carry their numbers from the paper they reproduce and live in
examples/traffic_profiles.
*/

#ifndef __STATIONARYUNIFORMTRAFFICPROFILE__
#define __STATIONARYUNIFORMTRAFFICPROFILE__

#include "ITrafficProfile.h"

namespace openEGCS {

class StationaryUniformTrafficProfile : public ITrafficProfile {
	private:
		double rate;	// Lambda of the Poisson process (expected arrivals per
						// unit time). Used only by nextInterArrivalTime.

	public:
		// `arrival_rate` is the fixed Poisson rate. It defaults to 0.5, the
		// same default Simulator uses, so a standalone CustomerFactory (which
		// only ever calls sampleTrip) can default-construct this without the
		// rate carrying any meaning.
		explicit StationaryUniformTrafficProfile(double arrival_rate = 0.5);

		// One Exponential(rate) draw, ignoring `now`: the arrival process does
		// not depend on the current time. This is the single first draw per
		// customer in the pre-interface order.
		double nextInterArrivalTime(double now, IRandomProvider& rng) const override;

		// Two uniform draws in [0, maxFloors] with the destination != origin
		// rejection loop, ignoring `t`: every floor is equally likely as origin
		// or destination at any instant. Exactly the draws (and loop) the
		// pre-interface CustomerFactory made.
		void sampleTrip(double t, IRandomProvider& rng, int maxFloors,
		                int& origin, int& destination) const override;

		// "stationary-uniform": the degenerate "no pattern" demand.
		std::string name() const override;
};

} // namespace openEGCS

#endif
