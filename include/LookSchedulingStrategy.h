/*
LookSchedulingStrategy.h

Default IElevatorSchedulingStrategy: implements the LOOK algorithm.

An elevator running LOOK keeps sweeping in its current direction, serving
every GO (car call) and same-direction CALL (hall call) request it finds
ahead of its position, in the order it reaches them. A hall call that
requests the opposite direction, or any request that lies behind the
elevator, is only served after the elevator exhausts everything ahead in
the current sweep and reverses. Unlike SCAN, LOOK does not travel all the
way to the end of the shaft when there is nothing left to serve there.
*/

#ifndef __LOOKSCHEDULINGSTRATEGY__
#define __LOOKSCHEDULINGSTRATEGY__

#include "IElevatorSchedulingStrategy.h"

namespace openEGCS {

class LookSchedulingStrategy : public IElevatorSchedulingStrategy {
	public:
		std::vector<Task>::iterator findInsertionPoint(
			std::vector<Task>& tasks,
			unsigned int direction,
			double position,
			const Task& newTask) const override;

		// Stable-sorts the route by the same key findInsertionPoint searches with,
		// so the binary search always has the ordering it assumes. Stable because
		// ties - two stops with the same key - must keep the order they were
		// inserted in, or the simulation stops being reproducible for a given seed.
		void reorder(
			std::vector<Task>& tasks,
			unsigned int direction,
			double position) const override;
};

} // namespace openEGCS

#endif
