/*
IElevatorSchedulingStrategy.h

Strategy interface for "how does THIS elevator order its own task queue"
(as opposed to IDispatchStrategy, which decides "which elevator" answers a
call in the first place), so the route-ordering policy can be tested and
swapped in isolation, following the same inversion-of-control pattern as
IDispatchStrategy/IMetricsCollector.

The bundled default (LookSchedulingStrategy) makes every elevator behave
like a LOOK elevator: continue in the current direction serving every
request found along the way, and only reverse once nothing is left ahead
in that direction.

How it is injected: pass an instance to Simulator (or ElevatorSystem), which
hands that ONE instance to every cabin of the group, the same way it shares
ICapacityStrategy, IDwellTimeStrategy and ITravelTimeStrategy. Two
consequences for an implementation:
  - it is consulted for every cabin, so it must keep no per-cabin state. The
    methods are const and receive everything a cabin's route depends on - the
    route itself, the cabin's position and its direction - as arguments, and
    the cabin does not identify itself;
  - it is also consulted on the dispatcher's what-if copies of the cabins
    (ElevatorSystem::FindBestElevator dry-runs a task into a copy's route), so
    the dispatch cost estimate is priced under this policy and not under LOOK.
*/

#ifndef __IELEVATORSCHEDULINGSTRATEGY__
#define __IELEVATORSCHEDULINGSTRATEGY__

#include <vector>
#include "Task.h"

namespace openEGCS {

class IElevatorSchedulingStrategy {
	public:
		virtual ~IElevatorSchedulingStrategy() = default;

		// Returns the position in `tasks` (already kept in "route order" by
		// previous calls to this method) where `newTask` must be inserted
		// so the elevator keeps following its scheduling policy, given it
		// is currently at `position` moving in `direction` (UP/DOWN, see
		// Task.h). `tasks` is assumed non-empty; the IDLE / first-task case
		// is handled directly by Elevator, before this strategy is ever
		// consulted.
		//
		// If the iterator returned points to a task whose destination is
		// already equal to `newTask`'s destination, the caller treats it as
		// a duplicate (reuses the existing task instead of inserting a new
		// one).
		virtual std::vector<Task>::iterator findInsertionPoint(
			std::vector<Task>& tasks,
			unsigned int direction,
			double position,
			const Task& newTask) const = 0;

		// Re-establishes route order for a cabin that is now at `position` moving
		// in `direction`. Elevator calls this before every insertion and after
		// every completed stop, because "route order" is not an absolute: it is
		// relative to where the cabin is and which way it is going, and both
		// change as the simulation runs.
		//
		// findInsertionPoint is free to assume `tasks` is already ordered (a
		// binary search needs that), and this is what keeps the assumption true.
		// Example: a LOOK route built while going up stores the stops that need a
		// reversal from the highest floor down; the moment the cabin reverses, the
		// order those same stops must be served in is exactly inverted.
		//
		// The default does nothing, so a strategy whose order does not depend on
		// the cabin's state (or that maintains its own invariant) needs no change.
		// Implementations must keep this *stable* with respect to entries that
		// compare equal, otherwise the simulation stops being reproducible.
		virtual void reorder(
			std::vector<Task>& /*tasks*/,
			unsigned int /*direction*/,
			double /*position*/) const {}
};

} // namespace openEGCS

#endif
