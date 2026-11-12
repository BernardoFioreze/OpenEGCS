#include "LookSchedulingStrategy.h"
#include <algorithm>
#include <cmath>

namespace openEGCS {

namespace {

// Large enough to push every "next sweep" task after every "current sweep"
// task, regardless of floor numbering, without needing a two-level sort key.
constexpr double kSweepOffset = 1'000'000.0;

// True when `t` requires the elevator to reverse direction before it can be
// served: either it lies behind the current position, or nothing it serves can
// be served while travelling this way.
//
// Task::servesSweep, rather than a single getDirection(), because a route entry
// can serve several requests at once and at a sweep reversal those requests
// disagree about direction. A stop at the top floor that absorbed both an
// up-call and the down-call waiting there is reachable in the current upward
// sweep - the cabin opens its doors once and takes both - and asking it for
// "the" direction would answer with whichever request arrived first, sending
// the other one's passengers to the next sweep or losing them from the
// ordering entirely.
bool needsReversal(const Task& t, unsigned int direction, double position)
{
	if (direction == UP)
	{
		if (t.getDest() < position) return true;
	}
	else if (t.getDest() > position) return true;
	return !t.servesSweep(direction);
}

// Maps a task to a single scalar so that sorting `tasks` ascending by this
// key reproduces LOOK order: everything reachable in the current sweep
// first (nearest first), followed by everything that requires a reversal
// (in the order it will be reached once the elevator turns around).
double routeKey(const Task& t, unsigned int direction, double position)
{
	if (!needsReversal(t, direction, position))
	{
		return (direction == UP) ? t.getDest() : -t.getDest();
	}
	return kSweepOffset + ((direction == UP) ? -t.getDest() : t.getDest());
}

} // namespace

void LookSchedulingStrategy::reorder(
	std::vector<Task>& tasks,
	unsigned int direction,
	double position) const
{
	if (tasks.size() < 2) return;
	std::stable_sort(tasks.begin(), tasks.end(),
		[direction, position](const Task& a, const Task& b) {
			return routeKey(a, direction, position) < routeKey(b, direction, position);
		});
}

std::vector<Task>::iterator LookSchedulingStrategy::findInsertionPoint(
	std::vector<Task>& tasks,
	unsigned int direction,
	double position,
	const Task& newTask) const
{
	const double key = routeKey(newTask, direction, position);
	return std::lower_bound(
		tasks.begin(), tasks.end(), key,
		[direction, position](const Task& t, double k) {
			return routeKey(t, direction, position) < k;
		});
}

} // namespace openEGCS
