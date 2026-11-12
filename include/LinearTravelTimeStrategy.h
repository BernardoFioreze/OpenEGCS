/*
LinearTravelTimeStrategy.h

The constant-speed travel model: a leg costs `|floors| * staticTime` and the
cabin covers it at constant speed, as if it reached full speed instantly and
stopped instantly.

The default is KinematicTravelTimeStrategy. This one is kept as a strategy for
comparison, and for reproducing studies that assume a constant time per floor
(Cortes 2004's 5 s per floor, for instance, in the GAHCA bench). It lives in
liftsim_core next to the interface because it is the simplest valid
implementation of it, the one the contract is easiest to check against.
*/

#ifndef __LINEARTRAVELTIMESTRATEGY__
#define __LINEARTRAVELTIMESTRATEGY__

#include "ITravelTimeStrategy.h"

namespace openEGCS {

class LinearTravelTimeStrategy : public ITravelTimeStrategy {
	private:
		// Time units per floor traversed. Held as a double, but the value
		// handed in has historically come from a float (Elevator::staticTime,
		// itself from the <Elevator speed> CLI argument); widening a float to
		// a double is exact, so this stores precisely what the old inline
		// arithmetic used.
		double staticTime;

	public:
		// `static_time` is time units per floor. Defaults to 1 - one time
		// unit per floor - which is what the old Elevator constructor
		// defaulted to.
		explicit LinearTravelTimeStrategy(double static_time = 1.0);

		// |signedFloors| * staticTime. Direction does not matter in this
		// model, which is one of the things that makes it unphysical: a real
		// installation's up and down legs differ.
		double legDuration(double signedFloors) const override;

		// Linear interpolation, clamped to the leg's own distance. The clamp
		// is why a cabin cannot overshoot its destination no matter how event
		// timestamps are shifted around it, and it is also what makes the
		// interface's endpoint contract hold here: past legDuration() the clamp
		// returns exactly `signedFloors`.
		double displacement(double signedFloors, double elapsed) const override;

		// Computed as "distance still to cover, times staticTime"
		// (`|dest - position| * staticTime`, with `position` being the leg's start
		// plus whatever it had covered). Writing it as `legDuration(all) - elapsed`
		// would be algebraically the same and need not be the same bits.
		double remainingDuration(double signedFloors, double elapsed) const override;

		// `|displacement(...)|` - a constant-velocity cabin has no momentum to
		// shed, so the shortest leg it can still make is simply the distance it
		// has already covered, and it can be retargeted to any floor it has not
		// passed yet.
		double minimumLegDistance(double signedFloors, double elapsed) const override;

		// "linear" - see ITravelTimeStrategy::name().
		const char* name() const override;
		// "linear" and the seconds per floor.
		TravelDescription describe() const override;
};

} // namespace openEGCS

#endif
