/*
ICapacityStrategy.h

Strategy interface for computing an elevator's "load bypass" threshold:
the load (see Elevator::getCurrentLoad()) at or above which a cabin stops
taking new landing calls, while still serving the passengers already on
board (their car calls are unaffected).

This is the pluggable counterpart to Elevator's maxWeight: the capacity
itself is just a number, but *how* the bypass threshold is derived from it
is a policy someone benchmarking a dispatch algorithm may want to change
(e.g. a fixed threshold instead of a percentage of capacity). Default
implementation: PercentageCapacityStrategy.
*/

#ifndef __ICAPACITYSTRATEGY__
#define __ICAPACITYSTRATEGY__

namespace openEGCS {

class ICapacityStrategy {
	public:
		virtual ~ICapacityStrategy() = default;

		// Given the elevator's maximum weight capacity, returns the load
		// (same unit as Customer::getWeight()) at or above which the
		// elevator stops accepting new landing calls.
		virtual double getBypassThreshold(double maxWeight) const = 0;
};

} // namespace openEGCS

#endif
