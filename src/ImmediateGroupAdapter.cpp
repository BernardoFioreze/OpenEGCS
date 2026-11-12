/*
ImmediateGroupAdapter.cpp - see ImmediateGroupAdapter.h.
*/

#include <stdexcept>
#include "ImmediateGroupAdapter.h"
#include "Task.h"

namespace openEGCS {

ImmediateGroupAdapter::ImmediateGroupAdapter(std::shared_ptr<IDispatchStrategy> inner_strategy)
	: inner(inner_strategy)
{
	if (!inner) {
		throw std::invalid_argument(
			"ImmediateGroupAdapter requires a non-null IDispatchStrategy to delegate to.");
	}
}

std::vector<int> ImmediateGroupAdapter::allocate(const std::vector<HallCall>& calls,
	const std::vector<Elevator>& elevators, IRandomProvider& /*rng*/)
{
	std::vector<int> assignment;
	assignment.reserve(calls.size());
	for (std::vector<HallCall>::const_iterator c = calls.begin() ; c != calls.end() ; ++c)
	{
		// A call that already has a cabin keeps it: the adapter NEVER
		// reallocates. This covers a committed call - which is not
		// reallocatable anyway, a braking cabin cannot un-brake - and an
		// allocated-but-not-committed one, which the adapter simply chooses not
		// to move. Only a pool call (currentElevatorId == 0) is a fresh decision
		// for the inner policy.
		if (c->currentElevatorId != 0)
		{
			assignment.push_back(c->currentElevatorId);
			continue;
		}
		// The CALL Task for this call: id 0, no name, type CALL, the call's
		// direction and floor, and its FIRST-press instant as the time (so the
		// dispatch cost sees how long it has been registered, not `now`).
		Task request(0, "", CALL, c->direction, c->floor, c->registeredAt);
		// A scratch copy, for the same reason FindBestElevator makes one: the
		// per-call strategy is allowed to leave a dry-run insertion behind, and
		// the engine must not have its real cars corrupted by one that does. The
		// engine hands us `elevators` by const reference precisely so it cannot
		// be touched; the copy is ours.
		std::vector<Elevator> scratch(elevators.begin(), elevators.end());
		assignment.push_back(inner->selectElevator(request, scratch));
	}
	return assignment;
}

} // namespace openEGCS
