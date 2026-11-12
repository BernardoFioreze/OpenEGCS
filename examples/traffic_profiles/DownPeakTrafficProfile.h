/*
DownPeakTrafficProfile.h

The evening down-peak: the dominant traffic flow is downward, with every
passenger leaving the building at the main terminal. This is one named
ITrafficProfile example, so it lives in examples/ and is never the default.

EVERY NUMBER HERE IS CITED. Both the rate curve and the origin/destination model
come from Barney & Al-Sharif, "Elevator Traffic Handbook: Theory and Practice",
2nd ed., 2015:

  - lambda(t): Table 17.4 "Arrivals during down peak" (section 17.7.3, Example
    17.2), the "Barney down peak (one hour)" template. One hour split into
    twelve 5-minute periods, arrivals in persons per 5 minutes:

        period  1  2   3    4    5   6   7   8   9  10  11  12
        value  22 47 180  115   71  46  33  26  24  24  23  25

    The peak is period 3 at 180 persons (the handbook's 22.5% peak value of the
    636/hour total: 0.225 x ... see Table 17.3, "Peak traffic value 22.5%").
    Converted to arrivals per second by dividing by 300 (5 minutes), so the
    ceiling rate lambdaMax is 180/300 = 0.6 arrivals/s (one simulation time
    unit is one second).

  - origin/destination: Definition 4.2 ("all, or the majority of, passengers
    leaving the lift system at the main terminal") with Table 17.3's "Floor bias
    (attraction) Equal". Modelled in the strict form: origin uniform over the
    upper floors {1..maxFloors}, destination always the main terminal (floor 0).
    "Equal" is the uniform origin; "all ... at the main terminal" is the fixed
    destination. The mixed "majority" case is deliberately out of scope - the
    strict form is the clean down-peak and needs no extra invented fraction.

MODELLING CHOICES (documented, not invented):
  - PERIODICITY. The template is one hour. A run that outlives it (the engine
    keeps producing arrivals until `cusToSimulate` customers exist) repeats the
    same hour - lambda(t) is periodic with period 3600 s. The curve's minimum
    (22 persons/5 min) is positive, so thinning always terminates.
  - COUNT vs SHAPE. This profile shapes *when* arrivals happen and *where* they
    go; the *number* of customers is still `cusToSimulate` (a Simulator
    contract). So the one-hour total of 636 is not a target - ask for 3000
    customers and they arrive shaped by the periodic curve over several hours.
    The CLI `arrivalRate` is ignored when a profile is injected, as the
    interface documents.
  - `scale` multiplies lambda(t) uniformly, for a user who wants a denser or
    lighter down-peak while keeping the handbook's shape. It defaults to 1.0,
    i.e. the literal tabulated rates; it is not a smuggled-in figure.
*/

#ifndef __DOWN_PEAK_TRAFFIC_PROFILE__
#define __DOWN_PEAK_TRAFFIC_PROFILE__

#include "PeakTrafficProfile.h"

namespace openEGCS {
namespace examples {

class DownPeakTrafficProfile : public PeakTrafficProfile {
	public:
		// `scale` multiplies the handbook rate curve (default 1.0 = the literal
		// Table 17.4 values in persons/5-min). The ceiling rate handed to the
		// thinning base is the curve's maximum (180/300 arrivals/s) times scale.
		explicit DownPeakTrafficProfile(double scale = 1.0);

		// Down-peak OD (Definition 4.2 + Table 17.3): origin uniform over the
		// upper floors [1, maxFloors], destination fixed at the main terminal
		// (floor 0). One uniform draw. destination != origin always holds
		// because origin is never 0.
		void sampleTrip(double t, IRandomProvider& rng, int maxFloors,
		                int& origin, int& destination) const override;

		// "down-peak (Barney)".
		std::string name() const override;

	protected:
		// Piecewise-constant over the twelve 5-minute periods of Table 17.4,
		// periodic with period 3600 s, in arrivals/s (persons-per-5-min / 300),
		// times `scale`. Always positive, so the thinning loop terminates.
		double rate(double t) const override;

	private:
		double scale;
};

} // namespace examples
} // namespace openEGCS

#endif
