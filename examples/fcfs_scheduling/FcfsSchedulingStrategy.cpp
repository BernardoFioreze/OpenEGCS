/*
FcfsSchedulingStrategy.cpp - see FcfsSchedulingStrategy.h.
*/

#include "FcfsSchedulingStrategy.h"

#include <cmath>

namespace openEGCS {
namespace examples {

namespace {

// Two destinations closer than this are the same floor - the tolerance
// Elevator::addTask merges route entries with, so the policy and the engine
// cannot disagree about whether a stop already exists.
constexpr double kSameFloor = 0.001;

} // namespace

std::vector<Task>::iterator FcfsSchedulingStrategy::findInsertionPoint(
	std::vector<Task>& tasks,
	unsigned int /*direction*/,
	double /*position*/,
	const Task& newTask) const
{
	// A linear scan rather than a search: the route is in arrival order, not
	// sorted by floor, so there is nothing to bisect.
	for (std::vector<Task>::iterator it = tasks.begin(); it != tasks.end(); ++it)
	{
		if (std::abs(it->getDest() - newTask.getDest()) < kSameFloor) return it;
	}
	return tasks.end();
}

} // namespace examples
} // namespace openEGCS
