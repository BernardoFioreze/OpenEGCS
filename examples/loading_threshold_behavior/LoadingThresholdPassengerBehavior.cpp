/*
LoadingThresholdPassengerBehavior.cpp

See LoadingThresholdPassengerBehavior.h.
*/

#include "LoadingThresholdPassengerBehavior.h"

#include <stdexcept>

namespace openEGCS {
namespace examples {

LoadingThresholdPassengerBehavior::LoadingThresholdPassengerBehavior(double threshold)
	: m_threshold(threshold)
{
	// A threshold of 0 refuses every car (the load after boarding is always > 0
	// for a passenger with weight), so nobody would ever board and no run could
	// terminate; anything outside (0, 1] is not a fraction of capacity.
	if (!(threshold > 0.0 && threshold <= 1.0))
	{
		throw std::invalid_argument(
			"LoadingThresholdPassengerBehavior: threshold must be in (0, 1]");
	}
}

bool LoadingThresholdPassengerBehavior::willBoard(const Customer& customer,
	const BoardingContext& cabin) const
{
	// "the lift will then be >threshold full" (Peters 8.3.6). currentLoad is the
	// load BEFORE this passenger, so the load after is currentLoad + weight. With
	// maxWeight == +infinity (unlimited cabin) the quotient is zero and the
	// comparison is always false, so an uncapacitated run is never affected -
	// which is the correct no-op, not a special case.
	double loadAfter = cabin.currentLoad + customer.getWeight();
	return loadAfter / cabin.maxWeight <= m_threshold;
}

double LoadingThresholdPassengerBehavior::nextCallTime(const Customer& customer,
	const RefusalContext& refusal) const
{
	(void)customer;
	return refusal.cabinDeparture;
}

} // namespace examples
} // namespace openEGCS
