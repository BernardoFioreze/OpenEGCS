/*
ThvDispatchStrategy.cpp - see ThvDispatchStrategy.h. Cortes et al. (2004),
Appendix A.
*/

#include "ThvDispatchStrategy.h"
#include "Task.h"	// UP, DOWN

#include <cmath>
#include <cstdlib>

namespace openEGCS {
namespace examples {

ThvDispatchStrategy::ThvDispatchStrategy(int numberOfFloors) : floors(numberOfFloors) {}

double ThvDispatchStrategy::fitnessOf(const Elevator& car, int callFloor, unsigned int callDirection) const
{
	const int carFloor = static_cast<int>(std::lround(car.getPosition()));
	const int d = std::abs(carFloor - callFloor);
	const int N = floors;

	if (car.getStatus() == IDLE) return N - d;		// stopped

	// BUSY: getDirection() is the sweep direction (UP or DOWN).
	const unsigned int dir = car.getDirection();
	const bool homing = (dir == UP && carFloor <= callFloor) || (dir == DOWN && carFloor >= callFloor);
	if (!homing) return 1;							// has left / passed the call floor
	return (dir == callDirection) ? (N + 1 - d) : (N - d);
}

int ThvDispatchStrategy::selectElevator(Task task, std::vector<Elevator>& elevators)
{
	const int callFloor = task.getDest();
	const unsigned int callDirection = task.getDirection();
	const int n = static_cast<int>(elevators.size());

	// Best (maximum) fitness wins; ties go to the lowest id. Considered twice:
	// first only cars that can actually answer (not full / bypassing), then - if
	// none can - all cars, so a call is never orphaned (see the header; not the
	// paper's THV, an engine accommodation like the greedy baseline's).
	for (int pass = 0; pass < 2; ++pass)
	{
		int best = -1;
		double bestFitness = 0.0;
		for (int i = 0; i < n; ++i)
		{
			if (pass == 0 && !elevators[i].canAnswerHallCall(0.0)) continue;
			const double f = fitnessOf(elevators[i], callFloor, callDirection);
			if (best == -1 || f > bestFitness)
			{
				bestFitness = f;
				best = i;
			}
		}
		if (best != -1) return best + 1;	// 1-based id
	}
	return -1;	// no cars at all
}

} // namespace examples
} // namespace openEGCS
