/*
PeakTrafficProfile.h

Abstract base for the non-stationary (peaked) ITrafficProfile examples. It owns
the *when* half of demand for a rate that varies with the simulated time,
lambda(t), and leaves the *where* half (sampleTrip) to the concrete pattern
below it.

It lives in examples/, not liftsim_core, for the same reason the dispatch
baselines do: a peaked profile is one named scenario to benchmark against, not a
default. The default stays StationaryUniformTrafficProfile; a peaked profile
changes every reported number on purpose.

HOW lambda(t) IS SAMPLED: Lewis & Shedler's thinning (1979). A homogeneous
Poisson process at the ceiling rate lambdaMax is generated, and each candidate
arrival at time t is kept with probability lambda(t)/lambdaMax. The kept points
are a non-homogeneous Poisson process with the intended rate lambda(t). This is
the standard, exact method - no analytic inversion of the integrated rate is
needed, which is what lets a subclass describe lambda(t) as any curve it likes
(e.g. the twelve tabulated 5-minute periods of the Barney down-peak template)
rather than a closed form.

DRAW CONTRACT (and why this is NOT byte-identical to the stationary default):
each thinning iteration consumes one IRandomProvider::nextExponential and one
uniform draw (see uniform01), and the number of iterations is itself random, so
a peaked profile consumes a variable number of draws per arrival. A different
arrival process gives a different run for the same seed, by construction. The
invariant that holds is "same seed + same build -> same snapshot": the draw
order within a run is fixed and deterministic.

TERMINATION: nextInterArrivalTime loops until a candidate is accepted, so a
subclass MUST keep rate(t) strictly positive for all t it will be asked about
(the engine keeps requesting arrivals until it has created `cusToSimulate`
customers, which can be well past the modelled hour). A rate that reaches zero
would spin forever. The down-peak example guarantees this by making lambda(t)
periodic over a curve whose minimum is positive.
*/

#ifndef __PEAK_TRAFFIC_PROFILE__
#define __PEAK_TRAFFIC_PROFILE__

#include "ITrafficProfile.h"

namespace openEGCS {
namespace examples {

class PeakTrafficProfile : public ITrafficProfile {
	public:
		// `lambda_max` is the ceiling rate (arrivals per unit time, i.e. per
		// second) used by the thinning process. It must be >= rate(t) for every
		// t, and > 0. A subclass passes the maximum of its own curve.
		explicit PeakTrafficProfile(double lambda_max);

		// Thinning: draw candidate arrivals from a Poisson(lambdaMax) process
		// and accept one with probability rate(candidate)/lambdaMax, returning
		// the accepted delta from `now`. Consumes a variable number of draws
		// (one exponential + one uniform per candidate).
		double nextInterArrivalTime(double now, IRandomProvider& rng) const override;

		// sampleTrip stays abstract: the origin/destination distribution is
		// what distinguishes one named pattern from another.

	protected:
		// lambda(t): the intended arrival rate at absolute simulated time `t`,
		// in arrivals per unit time. Must satisfy 0 < rate(t) <= lambdaMax for
		// every t (see TERMINATION above). This is where the pattern lives.
		virtual double rate(double t) const = 0;

		// A uniform real in [0, 1), built from the integer primitive so no new
		// method is added to the core IRandomProvider: the acceptance test only
		// needs modest precision, and a 1-in-a-million discretisation is far
		// below the noise of a Monte-Carlo arrival process. Consumes exactly one
		// nextUniformInt draw. Protected so a subclass can reuse it to pick an
		// origin/destination branch in sampleTrip (the lunch-peak does).
		static double uniform01(IRandomProvider& rng);

		double lambdaMax;
};

} // namespace examples
} // namespace openEGCS

#endif
