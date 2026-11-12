/*
GreedyNearestDispatchStrategy.h

Example IDispatchStrategy implementation, provided as a ready-to-use
baseline: for each elevator, does a dry-run insertion of the task into
that elevator's own route (Elevator::addTask with flag=0) and picks
whichever elevator reports the smallest estimated finish time
(Elevator::getFinishTime). This is the algorithm the original liftsim
hardcoded inside ElevatorSystem::FindBestElevator, extracted into
IDispatchStrategy so it can be swapped out.

This lives under examples/ (rather than the core library) to make it
clear that it is *a* dispatch algorithm, not *the* dispatch algorithm -
liftsim is a benchmarking platform where the interesting part is
plugging in your own IDispatchStrategy and comparing it against this
(or other) baseline(s).

What it is deliberately NOT, i.e. the things it gets "wrong" on purpose
because it is a baseline. A patch that "improves" one of them silently
moves the yardstick and invalidates every comparison recorded before it:
 - It is greedy in the strict sense: it minimises the estimated finish
   time of the *new* call only, and never evaluates what inserting it
   does to the passengers already assigned to that elevator. A call that
   saves itself two floors at the cost of delaying four people on board
   still wins.
 - It has no model of the future: no load balancing across the group, no
   parking or repositioning of idle cars, no notion that a floor is about
   to generate demand.
 - It treats Elevator::getFinishTime as the cost, which assumes the route
   will not change after the insertion. It will, as soon as the next call
   arrives.
 - Its awareness of direction is emergent, not explicit: the dry-run
   insertion goes through the elevator's own scheduling strategy, so a
   call that needs a reversal lands in the reversal group and reports a
   later finish time. Nobody wrote "prefer the elevator already heading
   that way" - it falls out of the route model, and it would stop falling
   out if a scheduling strategy with a different notion of order were
   injected.
  - It copies each whole Elevator to undo its dry-run, once per elevator
    per call. That is the price of not trusting the strategy contract,
    paid here on purpose.
 */

#ifndef __GREEDYNEARESTDISPATCHSTRATEGY__
#define __GREEDYNEARESTDISPATCHSTRATEGY__

#include "IDispatchStrategy.h"

namespace openEGCS {
namespace examples {

class GreedyNearestDispatchStrategy : public IDispatchStrategy {
	public:
		int selectElevator(Task task, std::vector<Elevator>& elevators) override;
};

} // namespace examples
} // namespace openEGCS

#endif
