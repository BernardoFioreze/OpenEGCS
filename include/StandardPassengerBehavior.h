/*
StandardPassengerBehavior.h

The default IPassengerBehaviorStrategy: a passenger who gets into every car
that stops for them and has room. See IPassengerBehaviorStrategy.h for why
that is the shipped default rather than Peters' 80% loading threshold.

This is the behaviour the simulator has always had - before the interface
existed, boarding was unconditional once capacity allowed it - so an
un-configured run produces a byte-identical snapshot to a pre-interface one.
That byte-identity is the gate this class exists to pass, and it is the
reason the class has no state and no parameters: anything configurable here
would be a number nobody has cited yet.
*/

#ifndef __STANDARDPASSENGERBEHAVIOR__
#define __STANDARDPASSENGERBEHAVIOR__

#include "IPassengerBehaviorStrategy.h"

namespace openEGCS {

class StandardPassengerBehavior : public IPassengerBehaviorStrategy {
	public:
		// Always true. The engine has already checked that the passenger
		// fits, and this passenger has no opinion beyond that - they take
		// whatever comes. (Closs rule 3, no boarding against one's own
		// direction, is enforced upstream: the engine only offers a cabin the
		// queue going its announced way.)
		//
		// Both arguments are deliberately unused: a subclass or a sibling
		// implementation is where they get read, and this class's whole job
		// is to not change anything.
		bool willBoard(const Customer& customer, const BoardingContext& cabin) const override;

		// The instant the car that refused them pulls away: the person sees it
		// go and presses the button again. Derived from the model rather than
		// from a figure nobody has cited - see IPassengerBehaviorStrategy.h.
		// With the shipped (zero) dwell that is the same instant they were
		// refused, which is as impatient as this simulator can be and is also
		// what keeps the un-configured run's behaviour simple to reason about.
		double nextCallTime(const Customer& customer, const RefusalContext& refusal) const override;
};

} // namespace openEGCS

#endif
