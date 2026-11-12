/*
LunchPeakTrafficProfile.h

Lunch-peak traffic as defined by Cortes, Larranaga & Onieva (2004), "Genetic
algorithm for controllers in elevator groups: analysis and simulation during
lunchpeak traffic", Applied Soft Computing 4, 159-174, section 4.1. It is the
scenario examples::GahcaDispatchStrategy was published against, so running
GAHCA *under this profile* is what makes a comparison with the paper
meaningful. Under uniform traffic GAHCA does not clearly beat the greedy
baseline (its fitness is a lunch-peak estimator); the paper's advantage needs
this traffic, saturated with finite cabins.

Lunch peak "is the most critical situation in vertical traffic, because it
includes the uppeak and downpeak traffic effects" (Cortes section 4.1): there
are TWO opposing streams at once, and this profile is their superposition.

EVERYTHING HERE IS CITED, from Cortes section 4.1:

  BUILDING. 12 floors; 30 workers per floor except the 7th (60, the admin dept)
  and the 12th (15, the manager dept); two 20-person lifts; 375 workers total.
  0-indexed here: ground = 0, "7th floor" = index 6, "12th floor" = index 11.

  lambda(t), Fig. 5 (two arrival-rate curves, persons/min, over 130 minutes =
  the paper's 7800 s replication length). The exact curves are figures, so
  they are digitised at 10-minute intervals and linearly interpolated (the
  standard way to reproduce a published curve; the values live in the .cpp and
  name their source):
    - "arrival rate to the hall of the floors": the exit stream, one asymmetric
      bell per upper floor, peaking at ~0.37 persons/min near minute 30 (workers
      leaving for lunch, [14:00, 15:00]).
    - "arrival rate to the hall of the ground floor": the return stream, a
      trapezoid plateauing at ~2.4 persons/min over roughly [80, 115] min
      (workers coming back, [15:20, 16:00]).

  ORIGIN/DESTINATION (interfloor travel probabilities, section 4.1):
    From the ground floor:  7th 15%,  12th 4%,  each other upper floor 9%.
    From any other floor:   ground 95%,  the rest (not ground, not self) 5%.

HOW THE TWO STREAMS BECOME ONE ITrafficProfile. nextInterArrivalTime (the base's
thinning) uses the aggregate rate Lambda(t) = nUpper x lambda_floors(t) +
lambda_ground(t). sampleTrip, given the same t, splits by the instantaneous mix:
a new arrival is a ground (return) arrival with probability
lambda_ground(t)/Lambda(t) and an upper-floor (exit) arrival otherwise, then the
origin/destination is drawn from the matching row of the OD table above. So the
two curves and the OD table are reproduced exactly, consistently at every t.

MODELLING CHOICES (documented, not invented):
  - This profile IS the 12-floor Cortes building: the 7th/12th attractions only
    mean something there. Run it with 12 floors. The constructor takes the floor
    count so nUpper is right, and clamps the special floors defensively for a
    smaller building, but the cited scenario is 12 floors.
  - PERIODICITY. The template is 130 minutes. A run that outlives it (the engine
    keeps producing arrivals until `cusToSimulate` customers exist) repeats it -
    Lambda(t) is periodic with period 7800 s. The exit curve is positive
    everywhere in the window, so Lambda(t) > 0 for all t and thinning always
    terminates.
  - COUNT vs SHAPE. The number of customers is `cusToSimulate`; this profile
    shapes when they arrive and where they go, not how many.
  - `scale` multiplies both curves uniformly (default 1.0 = the literal Fig. 5
    rates).
  - FIDELITY CAVEAT. This digitisation is faithful to Fig. 5, but Fig. 5 is
    faithful to the paper's *figure*, not to the traffic it actually simulated:
    integrated at scale 1 over 130 min the two curves give only ~394 trips
    (~3 persons/min), far too light to saturate 2 cabins, whereas the paper's
    Table 1 reports ~195 s mean / ~2899 s max waiting - heavy congestion. The
    published figure under-represents the real load, so the paper's ABSOLUTE
    numbers are not reproducible from it; a `scale` well above 1 is needed to
    reach the saturated regime, and only the RELATIVE GAHCA-vs-THV result
    reproduces.
*/

#ifndef __LUNCH_PEAK_TRAFFIC_PROFILE__
#define __LUNCH_PEAK_TRAFFIC_PROFILE__

#include "PeakTrafficProfile.h"

namespace openEGCS {
namespace examples {

class LunchPeakTrafficProfile : public PeakTrafficProfile {
	public:
		// `floors` is the building's floor count (default 12, the Cortes
		// building); `scale` multiplies the Fig. 5 curves (default 1.0 = the
		// literal rates). The thinning ceiling is the peak of the aggregate rate
		// nUpper x lambda_floors(t) + lambda_ground(t).
		explicit LunchPeakTrafficProfile(int floors = 12, double scale = 1.0);

		// Splits a new arrival into the exit stream (origin an upper floor,
		// mostly bound for the ground) or the return stream (origin the ground,
		// bound for an upper floor) by the instantaneous rate mix at `t`, then
		// draws the OD from the Cortes section 4.1 table. Consumes a variable
		// number of draws.
		void sampleTrip(double t, IRandomProvider& rng, int maxFloors,
		                int& origin, int& destination) const override;

		// "lunch-peak (Cortes 2004)".
		std::string name() const override;

	protected:
		// Aggregate rate Lambda(t) = nUpper x lambda_floors(t) + lambda_ground(t)
		// in arrivals/s, periodic over 7800 s. Always positive.
		double rate(double t) const override;

	private:
		int nUpper;		// number of upper floors (indices 1..nUpper = floors-1)
		int adminIdx;	// the 7th floor, index 6 (clamped to <= nUpper)
		int topIdx;		// the 12th floor, index nUpper (the manager dept)
		double scale;

		// The two Fig. 5 curves in persons/s at absolute time `t` (periodic).
		double floorsRatePerSec(double t) const;	// per upper-floor hall
		double groundRatePerSec(double t) const;	// the ground hall

		// The thinning ceiling, computed from the digitised grid at construction.
		static double ceilingRate(int floors, double scale);
};

} // namespace examples
} // namespace openEGCS

#endif
