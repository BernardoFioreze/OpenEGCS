/*
GahcaFitness.h

The estimated service time of ONE cabin, after Cortes, Larranaga & Onieva
(2004), "Genetic algorithm for controllers in elevator groups: analysis and
simulation during lunchpeak traffic", Applied Soft Computing 4, section 3.2.
This is the closed-form estimator the GAHCA genetic algorithm scores a candidate
assignment with: given where a cabin is, which way it is going, and the floors
it would serve up and down, it returns the time to clear them, from four peak
floors P1-P4 - WITHOUT simulating the route. Pure function, no engine state.

The paper's definitions, verified against section 3.2:

  Stopped or going up:                      Going down (the mirror):
    P1 = current floor                        P1 = current floor
    P2 = highest floor to board UP            P2 = lowest  floor to board DOWN
    P3 = lowest  floor to board DOWN          P3 = highest floor to board UP
    P4 = highest UP floor below P1 (P4<P1)    P4 = lowest DOWN floor above P1 (P4>P1)
    time = [(P2-P1)+(P2-P3)+(P4-P3)] * t      time = [(P1-P2)+(P3-P2)+(P3-P4)] * t

where t is the inter-floor trip time. "It includes the maximum known upward trip
plus the maximum known downward trip plus the subsequent maximum known
not-served upward trip" (and the mirror going down). Note that P2/P3 are the
highest/lowest call of their direction *anywhere* - only P4 is restricted to one
side of P1.

This is, in the paper's own words, "an approximate function (in s) to estimate
[...] the expected time in which the elevator group would serve the entire
allocated hall calls" - an estimation, "because of the incapability of
predicting the passenger future behaviour". Two consequences this file honours
rather than "improves", because the point is to REPRODUCE the paper, not to
out-estimate it:
  - The three-leg sweep is approximate. A degenerate case it does not model well
    (e.g. an up-call below an ascending cabin with no down-call to trigger the
    reversal) estimates as roughly unserved, time ~0. Making it geometrically
    exact would be a DIFFERENT estimator, not Cortes' - so it is left as is.
  - The paper assumes its lunchpeak ordering (P3 < P4 < P1 < P2, mirrored). Under
    uniform traffic a leg could come out negative; since the function "returns
    the expected time in s", each leg is clamped to >= 0 (a negative duration is
    not a reading the paper allows). In the paper's regime nothing is clamped.
This is also why the paper's absolute numbers are comparable only under its
lunchpeak traffic (examples/traffic_profiles/LunchPeakTrafficProfile) - under
uniform traffic this estimator drives the GA but is out of its intended regime.

The delays belong to the caller's reproduction environment, not here: the paper
uses `interFloorTime` = 5 s (cruise) and a per-stop delay of 13 s = 2 s decel +
2 s door open + 5 s transfer + 2 s door close + 2 s accel (section 3.2, "Usual
values are 2 s for every delays excepting [...] 5 s" for transfer). These are
the paper's figures, passed in - not invented here.
*/

#ifndef __GAHCA_FITNESS__
#define __GAHCA_FITNESS__

#include <vector>

namespace openEGCS {
namespace examples {

// Estimated time for a cabin at `currentFloor`, travelling `direction` (DOWN =
// descending branch; anything else, including UP and a parked cabin, = the
// "stopped or going up" branch, per the paper), to serve everybody boarding
// going up at the floors in `upStops` and going down at those in `downStops`.
//
// `interFloorTime` is the cruise time for one floor; `perStopTime` is the fixed
// delay of one stop (decel + doors + transfer + accel), charged once per listed
// stop - a floor in both `upStops` and `downStops` is two stops (one per sweep),
// as collective control makes it.
//
// The floors need not be sorted and may lie on either side of `currentFloor`.
double gahcaCabinServiceTime(int currentFloor, unsigned int direction,
	const std::vector<int>& upStops, const std::vector<int>& downStops,
	double interFloorTime, double perStopTime);

} // namespace examples
} // namespace openEGCS

#endif
