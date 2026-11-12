/*
ImmediateGroupAdapter.h

The default IGroupDispatchStrategy: it wraps an IDispatchStrategy and answers
each call by asking that per-call strategy, once, and NEVER reallocates a call
already assigned. It is what makes the group extension point transparent when
nobody opts into a real group algorithm: with this adapter, the engine behaves
exactly as a per-call dispatcher.

It lives in liftsim_core, not examples/, for the same reason StandardMetricsCollector
and StandardPassengerBehavior do and GreedyNearestDispatchStrategy does not: it is
the behaviour-preserving default, not one of the algorithms being benchmarked.
*/

#ifndef __IMMEDIATEGROUPADAPTER__
#define __IMMEDIATEGROUPADAPTER__

#include <memory>
#include <vector>
#include "IGroupDispatchStrategy.h"
#include "IDispatchStrategy.h"

namespace openEGCS {

class ImmediateGroupAdapter : public IGroupDispatchStrategy {
	public:
		// `inner` is the per-call policy every call is delegated to. Must be
		// non-null: the adapter has no decision of its own to fall back on.
		explicit ImmediateGroupAdapter(std::shared_ptr<IDispatchStrategy> inner);

		// For each call: one that already has a cabin (currentElevatorId != 0),
		// committed or not, keeps that cabin unchanged - the adapter NEVER
		// reallocates. Only a pool call (currentElevatorId == 0) is handed to
		// `inner->selectElevator`, as a CALL Task built from the HallCall and
		// with a scratch COPY of `elevators`, so a strategy that mutates its
		// argument cannot affect the run. No draw is taken from `rng`.
		std::vector<int> allocate(const std::vector<HallCall>& calls,
		                          const std::vector<Elevator>& elevators,
		                          IRandomProvider& rng) override;

	private:
		std::shared_ptr<IDispatchStrategy> inner;
};

} // namespace openEGCS

#endif
