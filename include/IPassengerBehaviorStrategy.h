/*
IPassengerBehaviorStrategy.h

Strategy interface for the decisions that belong to the *passenger*, not to
the installation: whether they get into the car that just opened its doors
in front of them, and when they press the button again after a car came and
could not take them.

Why this is a separate extension point rather than more methods on
ICapacityStrategy: those two answer different questions and the literature
puts them in different objects. ICapacityStrategy models the *equipment* -
the load-weighing device of Barney & Al-Sharif 10.8.3, which decides whether
the car stops for further landing calls, and whose 60% default is that
section's own figure. This one models the *person*: Peters' thesis gives the
passenger their own threshold, `person::m_LoadingThreshold`, described as
"threshold determining whether passenger will get into this lift or wait for
the next (%) e.g. 80% means that passenger will not load lift if the lift
will then be >80% full" (section 8.3.6). A cabin at 65% is simultaneously
"not stopping for more landing calls" (equipment) and "still worth getting
into" (person), and collapsing the two loses that.

One shared instance per ElevatorSystem, like ICapacityStrategy,
IDwellTimeStrategy and ITravelTimeStrategy - but unlike those three, which
are shared because they are properties of the building, this one is shared
only because the engine does not keep per-person state. The Customer is
handed to every method, so an implementation is free to answer differently
for different people (by weight, by id, by anything on the Customer)
without the engine having to store a policy per passenger.

Default implementation: StandardPassengerBehavior, which accepts every car
it fits into. The 80% figure above is not the default, because a non-trivial
willBoard changes results immediately; it ships as a separate, opt-in
implementation, examples::LoadingThresholdPassengerBehavior (in
examples/loading_threshold_behavior/).
*/

#ifndef __IPASSENGERBEHAVIORSTRATEGY__
#define __IPASSENGERBEHAVIORSTRATEGY__

#include "Customer.h"

namespace openEGCS {

// What a waiting passenger can see of the car whose doors have just opened
// in front of them. Passed by const reference so fields can be added later
// without breaking existing implementations.
//
// `currentLoad` is what the cabin is carrying *before* this passenger, so
// "the lift will then be >80% full" of Peters section 8.3.6 is
// (currentLoad + customer.getWeight()) / maxWeight. `maxWeight` is
// +infinity for an un-configured (unlimited) run, which makes every such
// ratio zero - a policy with a load threshold therefore has no effect on a
// run that configured no capacity, which is the correct behaviour rather
// than a special case.
//
// `cabinDirection` is the direction the cabin is travelling *now* (UP, DOWN,
// or IDLE when parked), which is NOT the direction it announced for this
// stop. The two differ exactly at a sweep reversal: the cabin is still
// travelling the old way while its next leg goes the other way.
//
// A policy does not need the announced direction: the engine announces it
// before anybody boards (ElevatorSystem::boardingDirectionAt) and filters the
// queue by it, so everybody this method is asked about is going the cabin's
// announced way. `cabinDirection` is there because "which way is this car
// moving right now" is a fact a policy may legitimately want.
struct BoardingContext {
	int elevatorId = 0;					// 1-indexed, matches Elevator::getId()
	int floor = 0;						// floor the doors are open at
	double time = 0;					// simulation instant of the boarding
	double currentLoad = 0;				// cabin load before this passenger boards
	double maxWeight = 0;				// cabin's rated load; +infinity when unlimited
	unsigned int cabinDirection = 0;	// UP / DOWN / IDLE - see above
};

// What a passenger knows when a car has just left without them: which car it
// was, where they are, and when that car actually pulls away. Passed by const
// reference so fields can be added later without breaking implementations.
//
// `cabinDeparture` is the instant the doors shut and the cabin starts moving,
// which is >= `now` and equal to it only when the installation's dwell is
// zero. It is what the default policy answers with: the person watches the car
// go and presses the button again. It needs no constant, which is why it is the
// default: "how long somebody hesitates before pressing a button again" is not
// a figure any of the cited sources give.
struct RefusalContext {
	int elevatorId = 0;			// the cabin that could not take them
	int floor = 0;				// where they still are
	double now = 0;				// the instant they were refused
	double cabinDeparture = 0;	// the instant that cabin leaves the floor
};

class IPassengerBehaviorStrategy {
	public:
		virtual ~IPassengerBehaviorStrategy() = default;

		// Does `customer` get into this car? Asked only after the engine has
		// already established that they physically fit (Elevator::canBoard),
		// so an implementation never has to re-check the rated load: a false
		// here means the passenger *declines* a car they could have entered,
		// which is a different fact from the cabin being full and must stay
		// distinguishable from it.
		//
		// Asked exactly once per boarding, from the one predicate the engine
		// consults when a cabin arrives (ElevatorSystem::wouldRefuseBoarding),
		// and the same place records the outcome in the timeline, so the
		// snapshot and the behaviour cannot disagree about what happened to a
		// passenger. The answer must not depend on how often the method has
		// been asked: counting the calls is fine, varying the answer is not.
		//
		// A passenger who declines keeps their landing call registered and
		// stays on the floor - nobody is ever lost by saying no.
		//
		// LIVENESS. The engine re-offers a declined call, and must not hand it
		// straight back to the cabin that was just declined, at the same instant,
		// forever. For a *capacity* refusal the engine's veto prevents that: a
		// cabin that is full or bypassing may not be allocated a landing call at
		// all. For a *preference* refusal the cabin is not full and is a
		// legitimate candidate, so the engine excludes it explicitly: the re-call
		// is never allocated to the cabin that refused, and
		// ElevatorSystem::preferenceExclusions keeps it out of the pool re-offer
		// while it is still at the refusal floor, releasing it once it departs.
		virtual bool willBoard(const Customer& customer, const BoardingContext& cabin) const = 0;

		// When does `customer` press the landing button again, having not got
		// into the car that stopped for them? The engine schedules a RECALL
		// event at the returned instant; a value in the past is clamped to
		// `refusal.now`, so an implementation cannot move a passenger
		// backwards in time, and the clock therefore never goes backwards
		// because of a policy.
		//
		// This is where the passenger's own patience would live. The re-call
		// has an instant of its own, so it is an event rather than a re-offer
		// repeated at every event until it sticks. The new call is never
		// allocated to the cabin that just refused them, which the engine
		// enforces - a policy cannot opt out of that.
		//
		// Nothing about the passenger's recorded waiting time changes: it runs
		// from the first press (onCustomerArrived) to the boarding, so a
		// person who is left behind twice simply waits longer, which is the
		// honest reading of what happened to them.
		virtual double nextCallTime(const Customer& customer, const RefusalContext& refusal) const = 0;
};

} // namespace openEGCS

#endif
