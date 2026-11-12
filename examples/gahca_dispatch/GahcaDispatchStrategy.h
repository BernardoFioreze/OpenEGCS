/*
GahcaDispatchStrategy.h

GAHCA - the genetic algorithm hall-call allocation of Cortes, Larranaga & Onieva
(2004), as an IGroupDispatchStrategy. Each time it is consulted it (re)allocates
the whole set of pending hall calls to the cars, scoring a candidate allocation
with the closed-form per-cabin estimator in GahcaFitness.h and searching with a
genetic algorithm. It is *one* example strategy, not the default - it lives in
examples/, like GreedyNearestDispatch.

Representation (equivalent to the paper's, chosen to be feasible by
construction): an individual is one car id per call - `assignment[j]` is the car
serving `calls[j]`. The paper encodes per-lift bit arrays (section 3.1, Fig. 3)
where each hall call's bit is 1 in exactly one lift; a car-per-call vector is the
same information with that one-car-per-call constraint built in, so uniform
crossover and the "reassign a hall call" mutation never produce an infeasible
individual to repair. A `committed` call (its stop is the front of its cabin's
route) is a FIXED gene: it keeps its current car and crossover and mutation
never touch it - a braking cabin cannot un-brake.

Objective (minimised), from the paper (Bolat 2011, cited by Cortes): for each
car i a service-time estimate ct_i (GahcaFitness), then
    f = k1 * sum_i(ct_i) + k2 * (Hct - Lct),  k1 = 1.5, k2 = 2
where Hct / Lct are the highest / lowest ct_i. The second term balances the work
across cars - the real difference from the greedy baseline, which only minimises
the new call's own finish.

All randomness is drawn from the injected IRandomProvider, in a fixed order, so
a run is reproducible.

The search follows the paper's Appendix B pseudocode (STEADY-STATE, not
generational):
  - population of 20, fitness computed and ranked;
  - WHILE fewer than `generations` (50) offspring have been accepted:
      p = Rnd;
      IF p < 0.85: crossover - pick two parents at random, re-pick until they
                   are not "incestuous" (identical), uniform-crossover -> one
                   offspring;
      ELSE (15%):  mutation - pick one parent at random, reassign one random
                   hall call -> one offspring;
      duplicity control: if the offspring equals an existing individual, discard
                   it and do NOT count it;
      otherwise evaluate it, SELECT ONE INDIVIDUAL FOR REPLACEMENT by the
                   hypergeometric rule (section 3.4: the individual at worst-rank
                   position p is chosen with probability ~q(1-q)^p, q=0.60),
                   replace it, update the fitness table, count the offspring.
Two documented readings where the pseudocode leaves a choice: individual 0 of
the initial population is seeded with the CURRENT allocation (the paper
re-optimises from the current state each t), and the returned solution is the
best individual seen anywhere in the run, not only in the final population - an
elitist safeguard that makes the result never worse than the current allocation
(the seed is in the initial population). Everything else - the fitness, the
objective, the encoding, the operators, the 20/50/0.85/0.60 parameters - is the
paper's.
*/

#ifndef __GAHCA_DISPATCH_STRATEGY__
#define __GAHCA_DISPATCH_STRATEGY__

#include <vector>
#include "IGroupDispatchStrategy.h"

namespace openEGCS {
namespace examples {

class GahcaDispatchStrategy : public IGroupDispatchStrategy {
	public:
		// Every field defaults to the paper's value; override to experiment.
		struct Params {
			double k1 = 1.5;				// weight of the summed service time
			double k2 = 2.0;				// weight of the load-balancing term (Hct - Lct)
			double interFloorTime = 5.0;	// s per floor at cruise (Cortes section 3.2)
			double perStopTime = 13.0;		// s per stop: 2 decel + 2 door open + 5 transfer + 2 door close + 2 accel
			int populationSize = 20;		// "we have maintained a population size of 20" (Appendix B: IF Population size = 20)
			int generations = 50;			// accepted offspring (Appendix B: WHILE i < 50); "at least 20", 50-100 similar
			int crossoverPercent = 85;		// Appendix B: IF p < 0.85 crossover, ELSE mutation
			double replacementQ = 0.60;		// q, "between 55 and 65%. The main tests are run with 60%"
		};

		GahcaDispatchStrategy();						// the paper's parameters
		explicit GahcaDispatchStrategy(Params parameters);

		std::vector<int> allocate(const std::vector<HallCall>& calls,
		                          const std::vector<Elevator>& elevators,
		                          IRandomProvider& rng) override;

		// The objective f above, for one assignment. Public so a test can check
		// the GA does not return something worse than the trivial allocation.
		double objective(const std::vector<int>& assignment,
		                 const std::vector<HallCall>& calls,
		                 const std::vector<Elevator>& elevators) const;

	private:
		Params params;
};

} // namespace examples
} // namespace openEGCS

#endif
