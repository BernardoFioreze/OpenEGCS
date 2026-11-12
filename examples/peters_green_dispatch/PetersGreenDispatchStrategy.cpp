#include "PetersGreenDispatchStrategy.h"

#include <limits>
#include <stdexcept>
#include "Elevator.h"
#include "Task.h"	// UP

namespace openEGCS {
namespace examples {

PetersGreenDispatchStrategy::PetersGreenDispatchStrategy(std::shared_ptr<IDispatchStrategy> base,
	double passengerMass)
	: wrapped(base), passengerMassKg(passengerMass)
{
	if (!wrapped) throw std::invalid_argument("PetersGreenDispatchStrategy: needs a dispatcher to wrap");
}

int PetersGreenDispatchStrategy::forcedCabin(const Task& call, const std::vector<Elevator>& elevators) const
{
	const int floor = call.getDest();
	const unsigned int direction = call.getDirection();
	const double sign = (direction == UP) ? 1.0 : -1.0;

	int best = 0;
	double bestDistance = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < elevators.size(); ++i) {
		const Elevator& cabin = elevators[i];
		if (cabin.getDirection() != direction) continue;	// a parked cabin reads IDLE, so it never matches
		const double ahead = (floor - cabin.getPosition()) * sign;
		if (ahead <= 0.0 || ahead >= bestDistance) continue;
		if (!cabin.canAnswerHallCall(passengerMassKg)) continue;
		for (const Task& stop : cabin.getRoute()) {
			if (stop.getDest() == floor && stop.hasCarCallRequest()) {
				best = static_cast<int>(i) + 1;
				bestDistance = ahead;
				break;
			}
		}
	}
	return best;
}

int PetersGreenDispatchStrategy::selectElevator(Task task, std::vector<Elevator>& elevators)
{
	++decided;
	const int chosen = wrapped->selectElevator(task, elevators);
	const int forcedTo = forcedCabin(task, elevators);
	if (forcedTo == 0) return chosen;
	if (forcedTo != chosen) ++overruled;
	return forcedTo;
}

} // namespace examples
} // namespace openEGCS
