/*
PetersGreenDispatchStrategy.h

Green Strategy No. 2 of Peters (1998), "Vertical Transportation Planning in
Buildings", EngD thesis, Brunel University, section 9.3: reduce the number of
stops by "forcing the dispatcher to allocate a landing call to a lift when it is
already due to stop at that floor for a passenger's car call, and travelling in
the right direction to serve the landing call." One example strategy, never a
default.

A decorator, because that sentence is one: it wraps any IDispatchStrategy. When a
cabin meets the condition it gets the call, and otherwise the wrapped dispatcher
decides, so the rule can be measured against any baseline. Peters applied it to
Liftsim's dynamic sectoring, which does not exist here.

The condition, read literally. A cabin qualifies when:
  - its sweep is the call's direction, which a parked cabin never matches;
  - the call's floor is strictly ahead of it. A cabin standing at the floor, or
    past it, is not "travelling" to it;
  - its route has a stop at that floor with a car call;
  - it can answer the call (Elevator::canAnswerHallCall for one passenger), or the
    engine's veto would undo the choice anyway. The paper predates capacity.
No braking distance is asked for: the stop is already on the route, so the cabin
is already going to stop there, which is the whole point of the rule.

Several qualify: the nearest, then the lowest id. There is no random step and no
re-allocation, so a run needs nothing from the group seam, and the same seed gives
the same passengers with and without the rule.
*/

#ifndef __PETERS_GREEN_DISPATCH_STRATEGY__
#define __PETERS_GREEN_DISPATCH_STRATEGY__

#include <memory>
#include <vector>
#include "IDispatchStrategy.h"

namespace openEGCS {
namespace examples {

class PetersGreenDispatchStrategy : public IDispatchStrategy {
	public:
		// `base` decides every call the rule does not force, and must not be
		// null. `passengerMassKg` is the weight the capacity check asks about
		// (75 kg, the thesis's passenger, section 5.9).
		explicit PetersGreenDispatchStrategy(std::shared_ptr<IDispatchStrategy> base,
			double passengerMassKg = 75.0);

		int selectElevator(Task task, std::vector<Elevator>& elevators) override;

		// The 1-based id of the cabin the rule forces for `call`, or 0 when no
		// cabin qualifies. Public so the condition can be tested on its own.
		int forcedCabin(const Task& call, const std::vector<Elevator>& elevators) const;

		// How many calls were decided, and how many of them the rule forced to a
		// cabin the wrapped dispatcher had not picked (the wrapped dispatcher is
		// always asked, so that this count means something).
		long decisions() const { return decided; }
		long forced() const { return overruled; }

	private:
		std::shared_ptr<IDispatchStrategy> wrapped;
		double passengerMassKg;
		long decided = 0;
		long overruled = 0;
};

} // namespace examples
} // namespace openEGCS

#endif
