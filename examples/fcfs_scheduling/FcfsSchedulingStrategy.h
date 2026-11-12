/*
FcfsSchedulingStrategy.h

FCFS - first come, first served - as a route-ordering policy: a cabin visits
its floors in the order they were first asked for, wherever it is and whichever
way it is going. It is the naive end of the disk-scheduling family the name
"LOOK" comes from (FCFS, SSTF, SCAN, C-SCAN, LOOK), and it plays on the route
side the role GreedyNearestDispatchStrategy plays on the dispatch side: a
baseline that is simple ON PURPOSE, to be beaten. It lives in examples/ and is
never a default, because swapping the route policy moves every number a run
reports.

What it does, against the IElevatorSchedulingStrategy contract:
  - findInsertionPoint: a floor the route already has a stop for returns THAT
    stop, which Elevator merges into (one stop per floor, see below); any other
    floor goes to the back of the route. `direction` and `position` are not
    read at all - that is the policy, not an omission.
  - reorder: not overridden. The default does nothing, which is exactly right:
    arrival order does not depend on where the cabin is or which way it moves,
    so there is nothing to re-establish when either changes.

Because a new stop always goes behind at least one existing one, FCFS never
retargets a leg in flight (Elevator::addTask retargets only when the insertion
lands at the front of the route). The one stop a cabin is heading to stays its
next stop until it is reached.

ONE STOP PER FLOOR. A second call for a floor already in the route joins that
floor's existing stop instead of queueing a second one further back, so a floor
keeps the place in the queue its FIRST request gave it. The alternative - one
stop per call - is rejected for a reason the engine gives rather than taste:
when a cabin stands at a floor, everybody bound for it gets out and the waiting
people going the announced way get in, whatever stop they were filed under, so
a second entry for the same floor reaches it with nobody left to serve - the
stop Closs's rule 1 forbids, and an empty cabin driving to it. LOOK almost
never produces that route shape. The tolerance is the one Elevator::addTask
merges with.

What FCFS deliberately does NOT do, in terms of the five rules of lift traffic
control (Barney & Al-Sharif section 10.4, Closs 1970):
  - rule 2, never pass a floor where someone wants to alight: it drives past a
    car call's floor if that floor came later in the queue;
  - rule 4, do not reverse while carrying passengers: it turns around whenever
    the next floor in the queue lies behind it, whoever is aboard;
  - rule 5, car calls outrank landing calls: both kinds wait their turn alike.
Rule 3 (nobody boards a car travelling against their direction) does NOT
fully hold under it either, and that is the engine's assumption, not this
class's: the engine announces the direction a cabin arrived in and boards
that queue, trusting the cabin to leave the same way - which LOOK does and
FCFS does not. In the saturated reference scenario 852 of 3,000 riders boarded
a cabin announced their way that then left the other way, 0 under LOOK. Two
consequences for reading a run: passengers carried away from their destination
are EXPECTED under FCFS, so a wrong-way count measured under it says nothing
about the engine on its own; and this is the policy that exposes what the
engine only guarantees while the route policy keeps sweeping.
*/

#ifndef __FCFS_SCHEDULING_STRATEGY__
#define __FCFS_SCHEDULING_STRATEGY__

#include <vector>
#include "IElevatorSchedulingStrategy.h"

namespace openEGCS {
namespace examples {

class FcfsSchedulingStrategy : public IElevatorSchedulingStrategy {
	public:
		// The existing stop for newTask's floor if the route has one, else
		// tasks.end(). Stateless and position-blind, so one instance can be
		// shared by every cabin of a group (the contract ElevatorSystem relies
		// on).
		std::vector<Task>::iterator findInsertionPoint(
			std::vector<Task>& tasks,
			unsigned int direction,
			double position,
			const Task& newTask) const override;
};

} // namespace examples
} // namespace openEGCS

#endif
