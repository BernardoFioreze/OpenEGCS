/*
StepMixTrafficProfile.h

The traffic of Beamurgia et al. (2016), "A modified genetic algorithm applied
to the elevator dispatching problem", Soft Computing 20, section 6.2 and Table
1: "Intensity was determined by a step profile with a minimum demand of 11 % and
a maximum demand of 13 % of the population per 5 min", over a building whose
populated floors each hold the same population (Table 1: 80 per floor in C1,
30 in C2), with a mix of incoming, outgoing and interfloor calls (the pure
profiles of Tables 2-4, and STEP1-3's 45/45/10, 0/100/0 and 80/15/5 of section
6.5). One named ITrafficProfile example, never the default.

What is the paper's and what is not:
  - THE PAPER'S: the 11% and 13% bounds, demand as a percentage of the
    population per 5 minutes (its HC, the handbook's Definition 4.7 period),
    the population per populated floor, the three-way mix.
  - SUBSTITUTED, because the paper does not say (it ran ELEVATE's own step
    profile and does not define it): the SHAPE of the step. Here the demand
    rises one percentage point per 5-minute period - 11%, 12%, 13% - and holds
    at 13% after that. Five minutes is the period the demand is stated in and
    the one ELEVATE's demand tables use; one point per period is the smallest
    step that reaches the maximum. Every one of these is a parameter.
  - SUBSTITUTED: which floors are populated. Floor 0 is the main terminal and
    holds nobody; floors 1..N each hold `populationPerFloor`. Incoming and
    outgoing are the handbook's Definitions 4.1/4.2 (from / to the main
    terminal), with the destination / origin uniform over the populated floors
    because they are equally populated (the handbook's "floor bias equal",
    Table 17.1); interfloor is Definition 4.5's random interfloor, origin and
    destination uniform over the populated floors and distinct.

Worth knowing before reading a result: the paper says the installation's
theoretical handling capacity is 10% (section 6.5), so 11% to 13% is demand
ABOVE capacity on purpose - a saturated regime, where the queue grows and the
drain matters (read the peak and drain windows together).

The thinning base (PeakTrafficProfile) consumes a variable number of draws for
the time; sampleTrip consumes one uniform01 for the category, then one
nextUniformInt per floor it draws (one for incoming and outgoing, two for
interfloor, never a rejection loop).
*/

#ifndef __STEP_MIX_TRAFFIC_PROFILE__
#define __STEP_MIX_TRAFFIC_PROFILE__

#include "PeakTrafficProfile.h"

namespace openEGCS {
namespace examples {

class StepMixTrafficProfile : public PeakTrafficProfile {
	public:
		struct Params {
			// Floors 1..populatedFloors are populated; floor 0 is the main
			// terminal. Must equal the run's floor count (`floors`, whose
			// floors run 0..floors), and sampleTrip refuses another one: a
			// mismatch would move the demand without anybody noticing.
			int populatedFloors = 0;
			double populationPerFloor = 0.0;	// Table 1: 80 (C1), 30 (C2)

			// Fractions of the calls; must add up to 1.
			double incoming = 0.0;		// main terminal -> a populated floor (Definition 4.1)
			double outgoing = 0.0;		// a populated floor -> main terminal (Definition 4.2)
			double interfloor = 1.0;	// populated floor -> another one (Definition 4.5)

			// Demand in percent of the population per period: starts at
			// `minPercent`, rises `stepPercent` per period, holds at
			// `maxPercent` (section 6.2's bounds; the shape is substituted).
			double minPercent = 11.0;
			double maxPercent = 13.0;
			double stepPercent = 1.0;
			double periodSeconds = 300.0;
		};

		// Throws std::invalid_argument for a non-positive population or
		// period, fewer than one populated floor (two if any call is
		// interfloor), a negative share, shares not adding up to 1, or a step
		// profile that is not positive and increasing.
		explicit StepMixTrafficProfile(Params parameters);

		// One uniform01 picks the category by its share; then incoming is
		// origin 0 and destination uniform over 1..N, outgoing the reverse,
		// interfloor two distinct floors uniform over 1..N. Throws
		// std::invalid_argument if `maxFloors` is not `populatedFloors`.
		void sampleTrip(double t, IRandomProvider& rng, int maxFloors,
		                int& origin, int& destination) const override;

		// "step-mix (Beamurgia 2016)".
		std::string name() const override;

		// The demand in percent of the population per period at instant `t`:
		// min(maxPercent, minPercent + stepPercent * period index). Public so
		// a test and a report can read the curve the run used.
		double percentAt(double t) const;

	protected:
		// percentAt(t) / 100 x population, per period, in arrivals/s. Always
		// positive (minPercent > 0), so the thinning loop terminates.
		double rate(double t) const override;

	private:
		Params params;
		double population;
};

} // namespace examples
} // namespace openEGCS

#endif
