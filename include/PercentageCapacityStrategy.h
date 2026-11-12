/*
PercentageCapacityStrategy.h

Default ICapacityStrategy: the load-bypass threshold is a fixed fraction
of the elevator's maximum weight capacity. The default, 60%, is the
literature's own load-weighing setting: Barney & Al-Sharif section 14.3.5
(p. 320), "a lift with just over 60% loading will not stop for further
landing calls ahead".
*/

#ifndef __PERCENTAGECAPACITYSTRATEGY__
#define __PERCENTAGECAPACITYSTRATEGY__

#include "ICapacityStrategy.h"

namespace openEGCS {

class PercentageCapacityStrategy : public ICapacityStrategy {
	private:
		double bypassFraction;		// e.g. 0.6 == bypass at 60% of maxWeight

	public:
		// Throws std::invalid_argument if bypass_fraction is not in (0, 1].
		explicit PercentageCapacityStrategy(double bypass_fraction = 0.6);

		double getBypassThreshold(double maxWeight) const override;
};

} // namespace openEGCS

#endif
