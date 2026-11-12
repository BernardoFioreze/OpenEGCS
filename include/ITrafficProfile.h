/*
ITrafficProfile.h

Strategy interface for the *demand* side of a run: which calls are born, when,
and between which floors. This is the counterpart of IDispatchStrategy on the
opposite side of the engine - dispatch decides *which cabin* answers a call,
this decides *what calls exist in the first place* - and the two never touch.

It is the seam that up-peak, down-peak and lunch-peak traffic plug into: the
named patterns the dispatch literature (Cortes 2004, Crites & Barto 1998,
Bolat, Beamurgia...) measures against. The simulation clock is in seconds, so
lambda(t) and a peak window are stated over real time.

The two halves of demand are split across two methods on purpose:
  - nextInterArrivalTime answers "when does the next customer arrive".
    Returning the delta (not a rate) lets the stationary default make exactly
    ONE exponential draw, while a peaked profile is free to choose its own
    mechanism (non-homogeneous Poisson by thinning) that consumes a variable
    number of draws.
  - sampleTrip answers "from which floor to which floor". What is NOT traffic
    stays in CustomerFactory: the sane/insane call direction and the Normal
    weight draw, which are properties of the person and the installation, not
    of the demand pattern.

Keeping both on one object (rather than two interfaces) is deliberate: a
realistic profile correlates *when* a call happens with *where* it goes (lunch
calls cluster on the canteen floor during the lunch window), so a single object
sees both questions and can answer them consistently.

Default implementation: StationaryUniformTrafficProfile - one exponential draw
at a fixed rate, two uniform floor draws. Peaked profiles ship as examples/,
never as the default: a non-trivial profile changes every reported number.

RNG ORDER IS A CONTRACT. Determinism is a byte-for-byte invariant
(tests/test_simulation_determinism.cpp), and every draw a profile takes from
the shared IRandomProvider moves every draw after it. An implementation must
document how many draws it consumes, in what order.
*/

#ifndef __ITRAFFICPROFILE__
#define __ITRAFFICPROFILE__

#include <string>
#include "IRandomProvider.h"

namespace openEGCS {

class ITrafficProfile {
	public:
		virtual ~ITrafficProfile() = default;

		// Time from `now` until the next customer arrives (a non-negative
		// delta, added to the running arrival clock by the caller - NOT an
		// absolute instant). `now` is the current arrival clock, given so a
		// time-varying profile can read it; the stationary default ignores it.
		//
		// Draw contract: the default consumes exactly one
		// IRandomProvider::nextExponential draw and nothing else, the first draw
		// per customer. A non-homogeneous profile (thinning) consumes a variable
		// number of draws and so produces a different run than the stationary
		// case for the same seed; what stays invariant is "same seed + same
		// build -> same snapshot".
		virtual double nextInterArrivalTime(double now, IRandomProvider& rng) const = 0;

		// Samples the origin/destination floor pair for a customer arriving at
		// instant `t`, writing them into `origin` and `destination`. Both are in
		// [0, maxFloors]; `destination` is guaranteed != `origin`.
		//
		// Draw contract: the default consumes two
		// IRandomProvider::nextUniformInt(0, maxFloors) draws for origin and
		// destination, plus one more for every rejected destination that equals
		// the origin. This runs AFTER nextInterArrivalTime for the same
		// customer, so the per-customer order is (inter-arrival, origin,
		// destination[, re-draws]); the weight draw that follows is the
		// factory's, not the profile's.
		virtual void sampleTrip(double t, IRandomProvider& rng, int maxFloors,
		                        int& origin, int& destination) const = 0;

		// A short, stable identifier for this demand pattern, recorded in the
		// snapshot's metadata (`trafficProfile`) so two snapshots are comparable
		// on time only when their profiles match - the same role
		// metadata.travelModel plays for the clock. Like
		// ITravelTimeStrategy::name(), it names the model, not the parameters.
		virtual std::string name() const = 0;
};

} // namespace openEGCS

#endif
