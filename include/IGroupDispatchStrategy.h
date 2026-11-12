/*
IGroupDispatchStrategy.h

Strategy interface for GROUP dispatch algorithms: the ones that decide the
whole assignment of pending landing calls to cars at once, and may MOVE a call
that has not yet been served from one car to another if that improves the
group. This is a different control model from IDispatchStrategy - not another
instance of the same concept - and it is the extension point the optimisation /
metaheuristic family of the literature needs (Cortes 2004's GAHCA, Bolat 2010,
Bolat 2013, Beamurgia 2016, Tartan 2014, Zhang & Cui 2024): each of those
encodes its solution as the full assignment of all pending calls to cars and
re-optimises it in a batch, which the immediate, per-call, non-revocable
IDispatchStrategy cannot express.

The default wired into ElevatorSystem is ImmediateGroupAdapter, which delegates
straight to an injected IDispatchStrategy and never reallocates. A real group
strategy is opt-in by injection, exactly as GreedyNearestDispatchStrategy is for
the per-call point.
*/

#ifndef __IGROUPDISPATCHSTRATEGY__
#define __IGROUPDISPATCHSTRATEGY__

#include <vector>
#include "Elevator.h"
#include "IRandomProvider.h"

namespace openEGCS {

// One pending landing call as the group sees it, decoupled from the engine's
// internal LandingCall so a strategy reasons about calls without touching the
// representation. It is a call-unit, one per (floor, direction), NOT one per
// passenger: the group reasons over lit buttons, and the people behind a call
// ride with it.
struct HallCall {
	int floor;
	unsigned int direction;		// UP / DOWN (Task.h)
	double registeredAt;		// instant of the first press; a call does not get younger because somebody joined it
	int currentElevatorId;		// 1-based id of the cabin it is allocated to now, or 0 for none (the pool) - matches ElevatorSystem's allocatedElevator convention
	bool committed;				// being served / its cabin's front is pinned: NOT reassignable (a cabin that is braking for a stop cannot un-brake)
};

class IGroupDispatchStrategy {
	public:
		virtual ~IGroupDispatchStrategy() = default;

		// Given every pending landing call and the current state of all cars,
		// return the 1-based id of the car assigned to each call, in the SAME
		// order as `calls`. The strategy may move a call that is not
		// `committed` between cars; for a `committed` call it must return the
		// car that call already has (`currentElevatorId`) - the engine ignores
		// anything else for it, because that cabin is already braking for it.
		// Returning an id outside 1..N (e.g. 0) for a non-committed call leaves
		// that call unallocated this round, which the engine treats as "no cabin
		// could take it" - the same decline the per-call point already allows.
		//
		// `elevators` is handed as a const reference and this interface returns
		// an assignment rather than mutating anything, so the "leave no side
		// effect" contract holds by construction: the engine is what applies the
		// result. A strategy that wants to score a what-if does it on its own
		// copy, or on the closed-form fitness its paper defines.
		//
		// `rng` is the simulator's own IRandomProvider: a GA/PSO MUST draw every
		// random choice from it (or from a provider of its own seeded from the
		// run's seed), and apply its reassignments in a deterministic order, or
		// two runs with the same seed stop producing the same snapshot. The
		// default adapter draws nothing from it.
		virtual std::vector<int> allocate(const std::vector<HallCall>& calls,
		                                  const std::vector<Elevator>& elevators,
		                                  IRandomProvider& rng) = 0;
};

} // namespace openEGCS

#endif
