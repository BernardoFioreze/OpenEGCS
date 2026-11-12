/*
LoadingThresholdPassengerBehavior.h

A non-trivial IPassengerBehaviorStrategy: a passenger who refuses to board a
car that would then be fuller than their personal loading threshold, and waits
for the next one instead. The engine provides the rest (the refusal at the
door, the PASS record, the re-call); this is a policy that says no.

The number is Peters' thesis (whose simulator is also called Liftsim), section
8.3.6: `person::m_LoadingThreshold`, "threshold determining whether passenger
will get into this lift or wait for the next (%) e.g. 80% means that passenger
will not load lift if the lift will then be >80% full". It is a preference of
the PERSON, not a limit of the equipment - which is why it lives here and not in
ICapacityStrategy (the 60% load-weighing device of Barney & Al-Sharif 10.8.3).
A cabin at 65% is simultaneously "not stopping for more landing calls"
(equipment) and "still worth getting into" (person); collapsing the two loses
that distinction.

It lives in examples/ and is NEVER the default: a non-trivial willBoard
changes results immediately and would silently move the yardstick under every
comparison recorded before it. The shipped default stays
StandardPassengerBehavior (boards everything it fits into).

LIVENESS: a policy that declines can livelock the engine unless the engine
excludes the refusing cabin from re-answering the same call until it leaves the
floor - that is the engine's job (ElevatorSystem's pool exclusion), not a
concern of this class. This policy is pure and stateless; the termination
guarantee is the engine's.
*/

#ifndef __LOADINGTHRESHOLDPASSENGERBEHAVIOR__
#define __LOADINGTHRESHOLDPASSENGERBEHAVIOR__

#include "IPassengerBehaviorStrategy.h"

namespace openEGCS {
namespace examples {

class LoadingThresholdPassengerBehavior : public IPassengerBehaviorStrategy {
	public:
		// `threshold` is the fraction of rated load at/above which the
		// passenger refuses - 0.80 for Peters' worked example. A value outside
		// (0, 1] is a configuration error and throws std::invalid_argument: a
		// threshold of 0 would refuse every car (nobody ever boards, the run
		// cannot terminate) and a negative or >1 one is meaningless as a
		// fraction of capacity.
		explicit LoadingThresholdPassengerBehavior(double threshold = 0.80);

		// Boards unless the cabin would then be more than `threshold` full:
		// (cabin.currentLoad + customer.getWeight()) / cabin.maxWeight >
		// threshold. The engine has already checked the passenger physically
		// fits, so a false here is a DECLINE, not "no room" - a different fact
		// the snapshot keeps distinct (PASS at the door, then a RECALL).
		//
		// With an unlimited cabin (maxWeight == +infinity) the ratio is zero and
		// this never refuses - a run that configured no capacity sees this
		// policy as a no-op, which is correct rather than a special case.
		//
		// Pure: the answer depends only on (customer, cabin), never on how often
		// it has been asked, as willBoard's contract requires.
		bool willBoard(const Customer& customer, const BoardingContext& cabin) const override;

		// The same model-derived answer as the default: the person watches the
		// car that could not take them pull away and presses the button again
		// at that instant. No patience figure is invented - none is cited.
		double nextCallTime(const Customer& customer, const RefusalContext& refusal) const override;

		// The configured threshold, for tests and reporting.
		double getThreshold() const { return m_threshold; }

	private:
		double m_threshold;
};

} // namespace examples
} // namespace openEGCS

#endif
