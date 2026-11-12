/*
BeamurgiaDispatchStrategy.h

The modified genetic algorithm of Beamurgia, Basagoiti, Rodriguez & Rodriguez
(2016), "A modified genetic algorithm applied to the elevator dispatching
problem", Soft Computing 20, as an IGroupDispatchStrategy. It is *one* example
strategy, never a default - it lives in examples/, like GahcaDispatchStrategy.

The paper's first level (section 5.1), as built:
  - an individual is one car per landing call (Fig. 1); a `committed` call is a
    fixed gene, because the engine will not move it;
  - fitness 1 / EWTP^2 (eq. 4 with eq. 6), EWTP = sum of the calls' EWT + the
    penalty, each EWT from beamurgiaEstimateRoute - the paper's second level;
  - generational: roulette selection on fitness (eq. 5), elitism of one ("the
    winner individual from the previous population evaluated is added to the
    new population"), two children from two different parents with crossover
    probability pc (else the parents are copied), each child mutated with
    probability pm by giving one call to a different car (Fig. 3);
  - crossover: genes common to both parents are inherited, each other gene is
    taken from either parent, independently for each child (Fig. 2);
  - 10 individuals, 10 iterations, pc = 0.8, pm = 0.1 (section 6.2).

The adjustments of section 6.3, each a switch in Params:
  - S (stability) is NOT a switch: the engine already refuses to move a call
    whose stop is the front of its cabin's route, for any load, which is
    stricter than the paper's S. Every run of this strategy is a "GA+S" run,
    and the paper's variants without S cannot be reproduced.
  - Sd (seeding): one initial individual gives each call to the nearest car
    that can take it under the rules of section 4.
  - LBI (last best individual): one initial individual is the best of the
    previous consultation, with the calls it no longer has dropped and the new
    ones given a random car. State between consultations, held here.
  - P (death penalty, eq. 8): + 1000 x I if any call's EWT exceeds 30 s.
  - P3 (dynamic penalty, eq. 9): + sum over calls of max(0, EWT - max(RWT,
    30))^2, RWT the mean time the calls handed over have already waited.
  P and P3 add seconds and squared seconds to a sum of seconds; that is the
  paper's eq. 6, kept as written.

What the engine hands this strategy: the calls in `allocate` are a SUBSET of
the building's calls - a new call alone (ElevatorSystem::allocateCall) or the
allocated, non-committed ones (reoptimizeAllocations), which runs right after a
new call is allocated and so re-decides it with the others in view. Calls not
handed over are not genes and do not enter the sum; the committed calls at the
front of a cabin's route are read off the route and ride in the estimate as
passengers boarding at that fixed stop, because they shape the route the others
wait on. `allocate` is given no clock, so the current time is read off the
cabins as well.

Every draw comes from one IRandomProvider, in a fixed order, so a run is
reproducible: the engine's own by default, or one the caller gives the strategy
(Params::random) to keep the GA's draws out of the stream the arrivals come
from - so variants run with one seed see the same passengers.
*/

#ifndef __BEAMURGIA_DISPATCH_STRATEGY__
#define __BEAMURGIA_DISPATCH_STRATEGY__

#include <map>
#include <memory>
#include <tuple>
#include <vector>
#include "BeamurgiaEstimator.h"
#include "IGroupDispatchStrategy.h"

namespace openEGCS {
namespace examples {

class BeamurgiaDispatchStrategy : public IGroupDispatchStrategy {
	public:
		enum class Penalty {
			None,		// no penalty term
			Death,		// P, eq. 8
			Dynamic		// P3, eq. 9
		};

		// Every field defaults to the paper's value (section 6.2/6.3) or, where
		// the paper gives none, to the substitution named next to it.
		struct Params {
			int populationSize = 10;			// "ten individuals" (section 6.2)
			int iterations = 10;				// "ten iterations" (section 6.2)
			double crossoverProbability = 0.8;	// section 6.2
			double mutationProbability = 0.1;	// section 6.2

			bool seeding = false;				// Sd
			bool lastBestIndividual = false;	// LBI
			Penalty penalty = Penalty::None;	// P or P3 (the paper never combines them)

			double waitingThreshold = 30.0;		// s, eqs. 8 and 9 (Sorsa et al. 2003)
			double deathPenaltyPerCar = 1000.0;	// eq. 8: 1000 x I, I = number of cars

			// D7: run the GA only when the set of calls handed over changed
			// since the last run, and otherwise keep every call where it is.
			// The engine consults a group strategy at every event; the paper
			// ran its GA when the system changed, and with LBI the difference
			// is more generations between arrivals. Never skips while a call
			// has no car, or a pooled call would be handed back unallocated
			// forever.
			bool reoptimizeOnlyOnChange = true;

			// Capacity in passengers = floor(rated load / this). The paper gives
			// no passenger mass; 70 kg is the project's default mean customer
			// weight, so the estimate's "full" matches the run's own draw.
			double passengerMassKg = 70.0;

			// R1 / R1' and the floor range (BeamurgiaEstimator.h).
			BeamurgiaEstimatorOptions estimator;

			// Where the GA's draws come from. Null (the default) is the
			// engine's own provider, handed to allocate(), as the interface
			// asks. A provider of its own, seeded per run, keeps the GA's
			// draws OUT of the stream the arrivals come from, so two variants
			// run with the same seed see the very same passengers (common
			// random numbers, plan §10) and can be compared run by run. Either
			// way a run is reproducible: same seeds, same snapshot.
			std::shared_ptr<IRandomProvider> random;
		};

		BeamurgiaDispatchStrategy();						// the paper's raw GA (+ S, imposed by the engine)
		explicit BeamurgiaDispatchStrategy(Params parameters);

		std::vector<int> allocate(const std::vector<HallCall>& calls,
		                          const std::vector<Elevator>& elevators,
		                          IRandomProvider& rng) override;

		// EWTP of one assignment (eq. 6): the summed EWT of `calls`, plus the
		// configured penalty. Lower is better; fitness is 1 / EWTP^2. Public so
		// a test can enumerate every assignment and compare the GA with the
		// exact optimum, as the paper's section 6.1 does.
		double objective(const std::vector<int>& assignment,
		                 const std::vector<HallCall>& calls,
		                 const std::vector<Elevator>& elevators) const;

		// The estimator's view of one cabin at `now` (plan §8, A2-A5):
		//  - empty route: standing at its rounded position, parked if IDLE;
		//  - standing at the floor of its route's front: standing there;
		//  - otherwise: moving, its front a fixed first stop reached at that
		//    stop's arrival time (which already includes any dwell left here);
		//  - doorCloseTime (DCT_0) = what is left of the dwell it is sitting
		//    in, the engine's own number for "the doors still have to shut";
		//  - capacity = floor(maxWeight / passengerMassKg), unlimited if the
		//    rated load is;
		//  - one car call per passenger alighting at each route stop.
		static BeamurgiaCabin cabinStateOf(const Elevator& cabin, double now, double passengerMassKg);

		// How often allocate() was consulted, and how often it actually ran
		// the GA (it does not when there is no choice to make).
		long consultations() const { return consulted; }
		long optimizations() const { return optimized; }

	private:
		// A call's identity across consultations: the same button pressed
		// again after being served is a new call, with a new registeredAt.
		typedef std::tuple<int, unsigned int, double> CallKey;

		struct Context;		// per-consultation state: now, cabin states, background boarders
		Context contextOf(const std::vector<HallCall>& calls, const std::vector<Elevator>& elevators) const;
		double evaluate(const std::vector<int>& assignment, const std::vector<HallCall>& calls,
		                const Context& context) const;
		std::vector<int> seededIndividual(const std::vector<HallCall>& calls,
		                                  const std::vector<Elevator>& elevators,
		                                  const Context& context) const;

		Params params;

		std::map<CallKey, int> lastBest;			// LBI: the car each call had in the last best individual
		std::vector<CallKey> lastOptimizedSet;		// D7
		long consulted = 0;
		long optimized = 0;
};

} // namespace examples
} // namespace openEGCS

#endif
