#include "GreedyNearestDispatchStrategy.h"
#include "Elevator.h"
#include "Task.h"

namespace openEGCS {
namespace examples {

int GreedyNearestDispatchStrategy::selectElevator(Task task, std::vector<Elevator>& elevators)
{
	std::vector<Elevator>::iterator it;
	double min = 0;
	int best = -1;
	bool skippedAny = false;

	// Load bypass: a near-full elevator should not be handed new pickup
	// (CALL) calls, so it can focus on dropping off the passengers already
	// on board. Car calls (already-boarded passengers) never reach
	// selectElevator, so no special-casing is needed for them.
	for (it = elevators.begin(); it != elevators.end(); ++it)
	{
		if (task.getType() == CALL && it->isBypassingCalls())
		{
			skippedAny = true;
			continue;
		}
		// Dry-run on the live elevator, then restore it whole. Undoing by
		// removeTask alone would leave ghost tasks behind when the task was
		// merged into an existing stop, and would not revert the route
		// re-anchoring addTask performs when the task becomes the front of the
		// queue.
		Elevator backup = *it;
		Task temp_task = it->addTask(task, 0);
		double finish = it->getFinishTime(temp_task);
		*it = backup;
		if (best == -1 || finish < min)
		{
			min = finish;
			best = it->getId();
		}
	}

	// Fallback: every elevator was bypassing (all near-full). Re-run
	// without the bypass filter rather than leaving the call unserved.
	if (best == -1 && skippedAny)
	{
		for (it = elevators.begin(); it != elevators.end(); ++it)
		{
			Elevator backup = *it;
			Task temp_task = it->addTask(task, 0);
			double finish = it->getFinishTime(temp_task);
			*it = backup;
			if (best == -1 || finish < min)
			{
				min = finish;
				best = it->getId();
			}
		}
	}

	return best;
}

} // namespace examples
} // namespace openEGCS
