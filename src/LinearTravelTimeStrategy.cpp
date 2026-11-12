#include "LinearTravelTimeStrategy.h"

#include <cmath>

namespace openEGCS {

LinearTravelTimeStrategy::LinearTravelTimeStrategy(double static_time)
	: staticTime(static_time)
{
}

double LinearTravelTimeStrategy::legDuration(double signedFloors) const
{
	// `abs(dest - pos) * staticTime`, in that operand order: a different but
	// algebraically equal expression need not give the same bits.
	return std::abs(signedFloors) * staticTime;
}

double LinearTravelTimeStrategy::displacement(double signedFloors, double elapsed) const
{
	// Lifted verbatim out of Elevator::update, including the order of the two
	// clamps. The original computed `totalFloors` from the leg's own endpoints
	// and then chose a sign from `dest >= legStartPosition`; here the sign of
	// `signedFloors` carries the same information, and `a + (-b)` is exactly
	// `a - b` in IEEE 754, so the caller's reconstruction of the position is
	// unchanged down to the bit.
	double totalFloors = std::abs(signedFloors);
	double elapsedFloors = elapsed / staticTime;
	if (elapsedFloors < 0) elapsedFloors = 0;
	if (elapsedFloors > totalFloors) elapsedFloors = totalFloors;
	return (signedFloors >= 0) ? elapsedFloors : -elapsedFloors;
}

double LinearTravelTimeStrategy::remainingDuration(double signedFloors, double elapsed) const
{
	// `signedFloors - displacement(...)` is exactly the `dest - position` the
	// old inline code fed to its multiplication, since position was the leg's
	// origin plus its covered distance. Same operands, same order, same bits.
	double remainingFloors = signedFloors - displacement(signedFloors, elapsed);
	return std::abs(remainingFloors) * staticTime;
}

double LinearTravelTimeStrategy::minimumLegDistance(double signedFloors, double elapsed) const
{
	// Reuses displacement() rather than recomputing `elapsed / staticTime`, so
	// that the clamping at both ends of the leg (and therefore the interface's
	// "never greater than |signedFloors|" contract) holds by construction
	// instead of by a second copy of the same two ifs.
	return std::abs(displacement(signedFloors, elapsed));
}

const char* LinearTravelTimeStrategy::name() const
{
	return "linear";
}

TravelDescription LinearTravelTimeStrategy::describe() const
{
	TravelDescription d;
	d.model = name();
	d.secondsPerFloor = staticTime;
	return d;
}

} // namespace openEGCS
