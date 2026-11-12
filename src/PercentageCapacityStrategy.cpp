#include <stdexcept>
#include "PercentageCapacityStrategy.h"

namespace openEGCS {

PercentageCapacityStrategy::PercentageCapacityStrategy(double bypass_fraction)
	: bypassFraction(bypass_fraction)
{
	if (bypass_fraction <= 0 || bypass_fraction > 1) {
		throw std::invalid_argument("PercentageCapacityStrategy: bypass_fraction must be in (0, 1]");
	}
}

double PercentageCapacityStrategy::getBypassThreshold(double maxWeight) const
{
	return maxWeight * bypassFraction;
}

} // namespace openEGCS
