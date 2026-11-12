/*
IDispatchStrategy.h

Strategy interface for elevator dispatch algorithms (the "which elevator
should answer this call" decision). This is the main extension point for
people wanting to benchmark their own dispatch algorithms against this
simulator: implement this interface and inject it into ElevatorSystem
instead of using the example GreedyNearestDispatchStrategy (see
examples/greedy_nearest_dispatch).

A call is one per (floor, direction), not one per passenger: the strategy
is asked once when a landing button lights up, and somebody arriving at a
floor whose button is already lit joins that call without a new decision.
The Task handed over is timestamped at the first press.
*/

#ifndef __IDISPATCHSTRATEGY__
#define __IDISPATCHSTRATEGY__

#include <vector>
#include "Task.h"
#include "Elevator.h"

namespace openEGCS {

class IDispatchStrategy {
	public:
		virtual ~IDispatchStrategy() = default;

		// Given a CALL task and the current state of all elevators, returns
		// the 1-based id of the elevator that should be assigned the task.
		// Implementations MUST NOT leave any side effect on `elevators`
		// (any dry-run/what-if task insertion used to estimate cost must be
		// undone before returning).
		// The engine does not depend on this for correctness: ElevatorSystem hands the strategy a
		// scratch COPY of the elevators, so a violation cannot corrupt the simulation -
		// but code that calls selectElevator directly (tests, tools) still gets the
		// real elevators, so honour the contract.
		//
		// Elevator::isBypassingCalls() ("load bypass") and Elevator::canAnswerHallCall()
		// tell whether a cabin can take a new landing call. The engine enforces them:
		// a call allocated to a cabin that is bypassing or has no room goes to another
		// cabin that can answer it, or stays pending until one can. A strategy may
		// consult them to avoid choosing such a cabin in the first place; the reference
		// implementation (examples::GreedyNearestDispatchStrategy) does.
		virtual int selectElevator(Task task, std::vector<Elevator>& elevators) = 0;
};

} // namespace openEGCS

#endif
