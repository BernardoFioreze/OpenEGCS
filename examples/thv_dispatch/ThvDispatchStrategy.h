/*
ThvDispatchStrategy.h

THV - the duplex/triplex baseline Cortes et al. (2004) compares GAHCA against
(Appendix A). "The THV algorithm assigns the hall call to the nearest lift in
the adequate trip direction." It is an IMMEDIATE, per-call, non-revocable
dispatch - so it is an IDispatchStrategy, like the bundled
GreedyNearestDispatchStrategy, and NOT a group strategy. It lives in examples/
because it is one algorithm to benchmark against, not a default.

Appendix A pseudocode (verified from the rendered PDF page), N = number of
floors, d = |call floor - elevator floor|; the car with the HIGHEST fitness wins:

    IF the car is homing to the call floor in the SAME trip direction as the call
        fitness = N + 1 - d
    ELSE IF homing to the call floor in a DIFFERENT trip direction
        fitness = N - d
    ELSE IF the car has just left the call floor
        fitness = 1
    ELSE (the car is stopped)
        fitness = N - d
    allocation = the car of best (maximum) fitness

So a nearby car already going the right way is best (N+1-d), a stopped or
wrong-way-but-approaching car is middling (N-d), and a car that has passed the
floor is worst (1). Mapping to the engine (a BUSY car's getDirection() is its
sweep direction, UP or DOWN):
  - stopped            = getStatus() == IDLE                       -> N - d
  - homing to F        = UP and carFloor <= F, or DOWN and carFloor >= F
      same direction   = getDirection() == call direction          -> N + 1 - d
      different         = otherwise                                 -> N - d
  - has left the floor = BUSY and not homing (F is behind it)       -> 1
carFloor is getPosition() rounded to the nearest floor.

Ties go to the lowest car id (deterministic; the paper does not specify).

`numberOfFloors` is N, a property of the building the caller supplies (the
Elevator does not carry it). It only shifts the constant offset between the
cases; the relative ordering - and so the allocation - is what matters.

Not in the paper, kept for the engine: the paper's THV predates capacity
modelling, so when a cabin cannot answer (Elevator::canAnswerHallCall - full or
load-bypassing) THV skips it, falling back to all cabins only if none can
answer, so a call is never orphaned - exactly as the greedy baseline does.
Uncapacitated runs never reach this.
*/

#ifndef __THV_DISPATCH_STRATEGY__
#define __THV_DISPATCH_STRATEGY__

#include <vector>
#include "IDispatchStrategy.h"

namespace openEGCS {
namespace examples {

class ThvDispatchStrategy : public IDispatchStrategy {
	public:
		explicit ThvDispatchStrategy(int numberOfFloors);

		// Returns the 1-based id of the highest-fitness car (Appendix A). Leaves
		// `elevators` untouched - THV takes no dry-run, so the "no side effect"
		// contract holds trivially.
		int selectElevator(Task task, std::vector<Elevator>& elevators) override;

		// The Appendix A fitness of one car for one call. Public so a test can
		// check each of the four cases directly.
		double fitnessOf(const Elevator& car, int callFloor, unsigned int callDirection) const;

	private:
		int floors;
};

} // namespace examples
} // namespace openEGCS

#endif
