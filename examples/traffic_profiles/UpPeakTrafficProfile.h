/*
UpPeakTrafficProfile.h

The morning up-peak: the dominant flow is upward, every passenger entering at
the main terminal and riding to an upper floor. One named ITrafficProfile
example, so it lives in examples/ and is never the default. It is the mirror
image of DownPeakTrafficProfile: same thinning base, opposite
origin/destination.

EVERY NUMBER HERE IS CITED, from Barney & Al-Sharif, "Elevator Traffic
Handbook: Theory and Practice", 2nd ed., 2015:

  - lambda(t): Table 17.2 "Arrivals by period during uppeak" (section 17.7.2,
    Example 17.1), the "Barney uppeak (one hour)" template. Twelve 5-minute
    periods, arrivals in persons per 5 minutes:

        period  1  2  3  4  5  6   7    8   9  10 11 12
        value  29 38 50 63 76 91 105 120  81  52 35 28

    The peak is period 8 at 120 persons (the handbook's 15% peak value, Table
    17.1). Converted to arrivals/s by dividing by 300, so lambdaMax is 120/300 =
    0.4 arrivals/s.

  - origin/destination: Definition 4.1 (uppeak - passengers enter at the main
    terminal and travel up) with Table 17.1's "Floor bias (attraction) Equal".
    Strict form: origin always the main terminal (floor 0), destination uniform
    over the upper floors {1..maxFloors}.

Modelling choices are the same as the down-peak's and documented there: lambda(t)
is periodic over the hour (minimum 28 persons/5-min > 0, so thinning always
terminates); the profile shapes when and where, not how many (that is
`cusToSimulate`); `scale` multiplies the curve uniformly and defaults to 1.0
(the literal Table 17.2 values).
*/

#ifndef __UP_PEAK_TRAFFIC_PROFILE__
#define __UP_PEAK_TRAFFIC_PROFILE__

#include "PeakTrafficProfile.h"

namespace openEGCS {
namespace examples {

class UpPeakTrafficProfile : public PeakTrafficProfile {
	public:
		// `scale` multiplies the Table 17.2 curve (default 1.0 = the literal
		// persons/5-min values). The thinning ceiling is the curve's maximum
		// (120/300 arrivals/s) times scale.
		explicit UpPeakTrafficProfile(double scale = 1.0);

		// Up-peak OD (Definition 4.1 + Table 17.1): origin fixed at the main
		// terminal (floor 0), destination uniform over the upper floors
		// [1, maxFloors]. One uniform draw; destination != origin always because
		// destination is never 0.
		void sampleTrip(double t, IRandomProvider& rng, int maxFloors,
		                int& origin, int& destination) const override;

		// "up-peak (Barney)".
		std::string name() const override;

	protected:
		// Piecewise-constant over the twelve 5-minute periods of Table 17.2,
		// periodic with period 3600 s, in arrivals/s (persons-per-5-min / 300),
		// times `scale`. Always positive, so the thinning loop terminates.
		double rate(double t) const override;

	private:
		double scale;
};

} // namespace examples
} // namespace openEGCS

#endif
