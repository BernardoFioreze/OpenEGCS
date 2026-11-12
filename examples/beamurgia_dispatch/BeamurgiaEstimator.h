/*
BeamurgiaEstimator.h

The estimated waiting time (EWT) of every landing call assigned to ONE cabin,
after Beamurgia, Basagoiti, Rodriguez & Rodriguez (2016), "A modified genetic
algorithm applied to the elevator dispatching problem", Soft Computing 20,
sections 3, 4.2 and 5.2. This is what the paper's GA scores a candidate
assignment with: unlike Cortes' closed-form P1-P4 (GahcaFitness.h), it
SIMULATES a route - the paper's "second level" - and reads each call's waiting
time off it. Pure function, no engine state.

The paper's pieces, as read:

  EWT_{h,i,j} = DCT + runTime + DOT + DTT                               (eq. 1)

  "first the ith lift needs to close the doors if they are open (DCT); second,
  to cover the distance from its position to the floor of the passenger
  (runTime); third, to open the doors when arrives to the floor (DOT) and
  finally, the passenger might wait some passengers to leave before entering
  into the lift (DTT)". Initial values (j = 0): DCT_0 = DCT if the doors are
  open, else 0; DTT_0 = 0; runTime_0 = from the cabin's position to its first
  destination.

  The route (section 5.2), iterated "until there are no more calls":
    - a moving cabin's current destination is fixed - "a restriction";
    - from the possible stops (landing calls and car calls) the closest one is
      chosen; if the cabin has no room, the closest CAR call (landing calls
      ignored - Fig. 4);
    - once a stop is chosen, all the calls at that floor join it;
    - Closs's rules hold (section 3): no boarding against the direction of a
      cabin carrying people, no reversal with somebody inside;
    - "if a lift is stopped and has no assigned direction, it will attend to
      the passenger with the longest waiting time" (section 3).

Readings this file takes where the paper is silent, each recorded here so it
can be argued with:

  - EWT is CUMULATIVE along the route and counted FROM THE PRESS: the
    returned value is `waited + time until the doors are open and the
    alighters are out`, where the time includes every earlier stop's whole
    dwell. Eq. 1 lists one leg's components; a waiting time along a route is
    the sum of the legs before it, and P3 compares EWT with the waiting
    already elapsed (RWT), which only makes sense if EWT includes it.
  - runTime comes from the run's own ITravelTimeStrategy::legDuration and the
    stop's dwell from the run's own IDwellTimeStrategy, so the estimate
    and the physics are the same model. DOT/DCT are the strategy's door
    times for that stop; DTT is its transfer time for the ALIGHTERS only
    (the boarder is waiting for them, not for himself); DTT_0 = 0 at the
    first stop is honoured as the paper writes it.
  - "Behind each call there is only one passenger" (section 3), and with
    conventional landing buttons the estimator does not know where he is
    going. A passenger boarded IN THE ESTIMATE therefore counts towards the
    cabin's load and locks its direction (Closs) for the rest of the sweep,
    and is taken to have left by the end of it: when nothing is left ahead
    in the travel direction, the estimated boarders are released. Real
    passengers (the car calls) leave at their own floor. Without a release
    the estimate could not terminate (a cabin full of passengers with no
    destination) - the paper does not say which it does; this is the
    reading that keeps Closs and terminates.
    WHERE the sweep ends is an option (BeamurgiaReversal), because
    the paper does not say and the choice moves estimates by tens of seconds:
      LastStopAhead (R1) - the boarders are gone by the last stop ahead, so
        the cabin reverses there. What LOOK does if their destinations
        happened to coincide with it; optimistic.
      ExpectedExtremeFloor (R1') - the cabin carries on to the floor where it
        is EXPECTED to reverse, with each boarder's destination uniform over
        the floors beyond the one they boarded at, and pays the expected
        number of alighting stops on the way. Barney & Al-Sharif (2015) eq.
        5.14 (highest reversal floor H, p. 100) and eq. 5.7 (expected stops S,
        p. 99), the pair the round trip time is built from. The book derives
        them for P passengers who all board at the main terminal; here every
        boarder has a floor of their own, so the product of per-passenger
        probabilities those equations are made of (eqs. 5.5, 5.10) is taken
        with non-identical factors - that extension is this project's, not the
        book's. Uniform, not the traffic profile's own O-D: the paper does not
        say the controller knows the demand, so the estimator is not given it.
  - Ties in "the closest" are broken car call first ("car calls have
    priority over landing calls", section 3), then a floor ahead in the
    current direction, then the lower floor, then UP before DOWN (the
    chromosome's order, Fig. 1), then input order - deterministic by
    construction.
  - At a stop of a cabin that is empty after its alighters leave, the calls
    boarded are the target call's direction (or, at a car-call stop, the
    arrival direction if a call waits that way, else the other one). Only
    one direction boards at one stop; the other call at the same floor is
    another stop, later, with its own door cycle.
*/

#ifndef __BEAMURGIA_ESTIMATOR__
#define __BEAMURGIA_ESTIMATOR__

#include <climits>
#include <vector>
#include "IDwellTimeStrategy.h"
#include "ITravelTimeStrategy.h"
#include "Task.h"	// UP, DOWN

namespace openEGCS {
namespace examples {

// One landing call assigned to the cabin being estimated.
struct BeamurgiaCall {
	int floor;
	unsigned int direction;		// UP / DOWN
	double waited;				// time since the first press (now - registeredAt); EWT counts from the press (D3)
};

// The state of one cabin at the instant the GA is called - the "variable
// information" of section 3, reduced to what the estimator reads.
struct BeamurgiaCabin {
	// Standing cabin: the floor it is at. Ignored while `moving`.
	int floor = 0;

	// UP / DOWN; anything else is a parked cabin with no assigned direction.
	// A moving cabin must have UP or DOWN.
	unsigned int direction = 0;

	// In flight towards `fixedDestination`, which then becomes the first stop
	// whatever the assignment ("a restriction", section 5.2). The time to it is
	// given rather than recomputed because a leg already under way is NOT a
	// fresh leg from the cabin's fractional position: the kinematic model is
	// not additive in distance (ITravelTimeStrategy::remainingDuration), so
	// pricing it here would bill a second ramp.
	bool moving = false;
	int fixedDestination = 0;
	double timeToFixedDestination = 0.0;

	// DCT_0: the time to close the doors if they are open now, else 0.
	double doorCloseTime = 0.0;

	// In passengers ("enough space to carry a new passenger", section 5.2).
	// At least 1. The engine sizes cabins in kg; converting is the caller's
	// business (BeamurgiaDispatchStrategy does it).
	int capacity = INT_MAX;

	// One entry per passenger aboard: the floor they get out at. The number of
	// real passengers aboard is the size of this list.
	std::vector<int> carCalls;
};

// Where a sweep that carries passengers of unknown destination ends - the R1 /
// R1' choice of the file comment.
enum class BeamurgiaReversal {
	LastStopAhead,			// R1: at the last stop ahead
	ExpectedExtremeFloor	// R1': at the expected highest/lowest floor (Barney & Al-Sharif eq. 5.14), paying eq. 5.7's expected stops
};

struct BeamurgiaEstimatorOptions {
	BeamurgiaReversal reversal = BeamurgiaReversal::LastStopAhead;

	// The floors the building has. Read by ExpectedExtremeFloor only, which
	// needs the range a destination is drawn from; every call and car call
	// must then lie inside it.
	int lowestFloor = 0;
	int highestFloor = 0;
};

// R1' pieces, exposed so they can be checked against the book on their own.
//
// The expected floor a cabin at `pos` going `direction` reverses at, given
// passengers aboard who boarded at `boardingFloors` and each leave at a floor
// drawn uniformly from those beyond their boarding floor (up to
// `highestFloor` going UP, down to `lowestFloor` going DOWN): E[max(pos,
// max destination)] going UP, its mirror going DOWN. Never behind `pos`. For
// P boarders at the lowest floor and `pos` there, this is exactly eq. 5.14:
// N - sum_{i=1}^{N-1} (i/N)^P, N the floors above.
double beamurgiaExpectedReversalFloor(double pos, unsigned int direction,
	const std::vector<int>& boardingFloors, int lowestFloor, int highestFloor);

// The expected number of floors strictly beyond `pos` at which at least one
// of those passengers gets out. For P boarders at the lowest floor and `pos`
// there, exactly eq. 5.7: N (1 - ((N-1)/N)^P).
double beamurgiaExpectedStopsBeyond(double pos, unsigned int direction,
	const std::vector<int>& boardingFloors, int lowestFloor, int highestFloor);

struct BeamurgiaRoute {
	// EWT of each call, in the same order as the `calls` argument: seconds,
	// counted from the call's first press.
	std::vector<double> ewt;

	// The floors the estimated route stops at, in order, up to the stop that
	// serves the last landing call: car calls left after it change no EWT and
	// are not routed. A floor appears twice if the cabin stops there twice (one
	// call each way). Exists so the caller can compare the estimate's first
	// stop with the cabin's real route.
	// Under ExpectedExtremeFloor, the expected excursion past the last stop
	// is not a stop and is not listed.
	std::vector<int> stops;
};

// Routes `cabin` through every call in `calls` by the section 5.2 heuristic and
// returns each call's estimated waiting time. Every call is served: the route
// terminates by construction (each step serves at least one call or car call,
// or ends a sweep, which can happen at most once between two services).
//
// Throws std::invalid_argument if `capacity` < 1, a call's direction is not UP
// or DOWN, a moving cabin's direction is not UP or DOWN, or a time is negative;
// and, under ExpectedExtremeFloor, if the floor range is empty or a floor lies
// outside it. Throws std::logic_error if the route ever chooses a stop that
// serves nobody (the progress check the termination rests on, plan §6 R7).
BeamurgiaRoute beamurgiaEstimateRoute(const BeamurgiaCabin& cabin,
	const std::vector<BeamurgiaCall>& calls,
	const ITravelTimeStrategy& travel, const IDwellTimeStrategy& dwell,
	const BeamurgiaEstimatorOptions& options = {});

} // namespace examples
} // namespace openEGCS

#endif
