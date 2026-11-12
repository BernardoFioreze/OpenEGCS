#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>
#include "BeamurgiaEstimator.h"
#include "FixedDwellTimeStrategy.h"
#include "KinematicTravelTimeStrategy.h"
#include "LinearTravelTimeStrategy.h"
#include "Task.h"	// UP, DOWN

using namespace openEGCS;
using namespace openEGCS::examples;

// The EWT estimator of Beamurgia et al. 2016 (sections 3, 4.2, 5.2). Pure
// function, so every case is hand-computed. The travel model is the linear
// one at 1 s/floor and the dwell is 2 s to open, 3 s to close and 1 s per
// passenger - three different numbers, so a term charged in the wrong place
// shows up in the arithmetic.

namespace {

const LinearTravelTimeStrategy kOneSecondPerFloor(1.0);
const FixedDwellTimeStrategy kDwell(/*open=*/2.0, /*close=*/3.0, /*perPassenger=*/1.0);
const FixedDwellTimeStrategy kNoDwell;

BeamurgiaCabin standing(int floor, unsigned int direction)
{
	BeamurgiaCabin c;
	c.floor = floor;
	c.direction = direction;
	return c;
}

} // namespace

TEST_CASE("Beamurgia estimator: Fig. 4 - a full cabin ignores the nearer landing call", "[beamurgia]") {
	// Section 5.2, Fig. 4: capacity five, a landing call one floor away and the
	// car call two floors away. Five aboard -> the car call; four -> the landing call.
	std::vector<BeamurgiaCall> calls = {{3, UP, 0.0}};

	BeamurgiaCabin full = standing(2, UP);
	full.capacity = 5;
	full.carCalls = {4, 4, 4, 4, 4};
	BeamurgiaRoute r = beamurgiaEstimateRoute(full, calls, kOneSecondPerFloor, kDwell);
	REQUIRE(r.stops == std::vector<int>{4, 3});
	// 2->4: 2 s; stop at 4: open 2 + five alight 5 + close 3 = 10 -> 12;
	// 4->3: 1 s -> 13; open 2, nobody alights at 3 -> EWT 15.
	REQUIRE(r.ewt[0] == Catch::Approx(15.0));

	BeamurgiaCabin room = full;
	room.carCalls = {4, 4, 4, 4};
	r = beamurgiaEstimateRoute(room, calls, kOneSecondPerFloor, kDwell);
	REQUIRE(r.stops == std::vector<int>{3});	// the car call at 4 comes after the last landing call: not routed
	// 2->3: 1 s; open 2; DTT_0 = 0 at the first stop -> EWT 3.
	REQUIRE(r.ewt[0] == Catch::Approx(3.0));
}

TEST_CASE("Beamurgia estimator: EWT is cumulative and DCT_0 is charged once", "[beamurgia]") {
	// D3: EWT counts from the press and accumulates every earlier stop's whole
	// dwell. DCT_0 (doors open now) delays everything after it, once.
	std::vector<BeamurgiaCall> calls = {{2, UP, 4.0}, {5, UP, 1.0}};

	BeamurgiaCabin closed = standing(0, UP);
	BeamurgiaRoute r = beamurgiaEstimateRoute(closed, calls, kOneSecondPerFloor, kDwell);
	REQUIRE(r.stops == std::vector<int>{2, 5});
	// call 0: waited 4 + run 2 + open 2 (+ DTT_0 = 0) = 8
	REQUIRE(r.ewt[0] == Catch::Approx(8.0));
	// stop at 2 shuts at 2 + (2 + 1 boarder + 3) = 8; run 3 -> 11; open 2 -> 13;
	// call 1: waited 1 + 13 = 14
	REQUIRE(r.ewt[1] == Catch::Approx(14.0));

	BeamurgiaCabin open = closed;
	open.doorCloseTime = 3.0;
	BeamurgiaRoute o = beamurgiaEstimateRoute(open, calls, kOneSecondPerFloor, kDwell);
	REQUIRE(o.ewt[0] == Catch::Approx(r.ewt[0] + 3.0));
	REQUIRE(o.ewt[1] == Catch::Approx(r.ewt[1] + 3.0));
}

TEST_CASE("Beamurgia estimator: the boarder waits for the alighters, except at the first stop", "[beamurgia]") {
	// DTT is the transfer of the people getting OUT; DTT_0 = 0 (section 4.2).
	std::vector<BeamurgiaCall> calls = {{3, UP, 0.0}, {6, UP, 0.0}};
	BeamurgiaCabin c = standing(0, UP);
	c.carCalls = {3, 6, 6};
	BeamurgiaRoute r = beamurgiaEstimateRoute(c, calls, kOneSecondPerFloor, kDwell);
	REQUIRE(r.stops == std::vector<int>{3, 6});
	// first stop: run 3, open 2, one alights but DTT_0 = 0 -> 5
	REQUIRE(r.ewt[0] == Catch::Approx(5.0));
	// stop 3 shuts at 3 + (2 + 2 transfers + 3) = 10; run 3 -> 13; open 2 -> 15;
	// two alight first (2 s) -> 17
	REQUIRE(r.ewt[1] == Catch::Approx(17.0));
}

TEST_CASE("Beamurgia estimator: a parked cabin attends the longest-waiting passenger first", "[beamurgia]") {
	// Section 3: "if a lift is stopped and has no assigned direction, it will
	// attend to the passenger with the longest waiting time" - not the nearest.
	std::vector<BeamurgiaCall> calls = {{6, UP, 1.0}, {1, DOWN, 10.0}};

	BeamurgiaRoute parked = beamurgiaEstimateRoute(standing(5, 0), calls, kOneSecondPerFloor, kNoDwell);
	REQUIRE(parked.stops == std::vector<int>{1, 6});

	// The same cabin with a direction takes the nearest.
	BeamurgiaRoute going = beamurgiaEstimateRoute(standing(5, UP), calls, kOneSecondPerFloor, kNoDwell);
	REQUIRE(going.stops == std::vector<int>{6, 1});
}

TEST_CASE("Beamurgia estimator: Closs - no boarding against the direction, no reversal with people aboard", "[beamurgia]") {
	// Going up with a passenger for 8: the down call at 5 is the nearest call
	// but is skipped; the up call at 7 is taken on the way.
	std::vector<BeamurgiaCall> calls = {{5, DOWN, 0.0}, {7, UP, 0.0}};
	BeamurgiaCabin c = standing(0, UP);
	c.carCalls = {8};
	BeamurgiaRoute r = beamurgiaEstimateRoute(c, calls, kOneSecondPerFloor, kNoDwell);
	REQUIRE(r.stops == std::vector<int>{7, 8, 5});	// the nearest call, 5, goes last
	REQUIRE(r.ewt[1] == Catch::Approx(7.0));
	REQUIRE(r.ewt[0] == Catch::Approx(7.0 + 1.0 + 3.0));
}

TEST_CASE("Beamurgia estimator: a passenger boarded in the estimate also locks the direction", "[beamurgia]") {
	// Nobody is aboard for real, but the cabin picks up the up call at 3 on its
	// way: from then on it carries somebody going up, so the down call at 2 -
	// one floor behind, the nearest - waits until the sweep ends, after 6.
	BeamurgiaCabin c;
	c.moving = true;
	c.direction = UP;
	c.fixedDestination = 3;
	c.timeToFixedDestination = 3.0;
	std::vector<BeamurgiaCall> calls = {{3, UP, 0.0}, {2, DOWN, 0.0}, {6, UP, 0.0}};
	BeamurgiaRoute r = beamurgiaEstimateRoute(c, calls, kOneSecondPerFloor, kNoDwell);
	REQUIRE(r.stops == std::vector<int>{3, 6, 2});
	REQUIRE(r.ewt[2] == Catch::Approx(6.0));
	REQUIRE(r.ewt[1] == Catch::Approx(6.0 + 4.0));
}

TEST_CASE("Beamurgia estimator: a moving cabin's destination is a fixed first stop", "[beamurgia]") {
	// Section 5.2: "the lift's destination floor at that moment can be regarded
	// as defined and as a restriction" - even with a nearer call behind it.
	BeamurgiaCabin c;
	c.moving = true;
	c.direction = UP;
	c.fixedDestination = 9;
	c.timeToFixedDestination = 4.0;
	std::vector<BeamurgiaCall> calls = {{3, UP, 0.0}};
	BeamurgiaRoute r = beamurgiaEstimateRoute(c, calls, kOneSecondPerFloor, kDwell);
	REQUIRE(r.stops == std::vector<int>{9, 3});
	// 4 s to 9; that stop serves nobody and still opens and shuts: +5 -> 9;
	// 9->3: 6 s -> 15; open 2 -> 17.
	REQUIRE(r.ewt[0] == Catch::Approx(17.0));
}

TEST_CASE("Beamurgia estimator: one direction boards per stop", "[beamurgia]") {
	// An empty cabin reaching a floor with both buttons lit takes one of them
	// (UP before DOWN, Fig. 1) and comes back for the other with its own door cycle.
	std::vector<BeamurgiaCall> calls = {{3, DOWN, 0.0}, {3, UP, 0.0}};
	BeamurgiaRoute r = beamurgiaEstimateRoute(standing(0, UP), calls, kOneSecondPerFloor, kDwell);
	REQUIRE(r.stops == std::vector<int>{3, 3});
	REQUIRE(r.ewt[1] == Catch::Approx(3.0 + 2.0));
	// first stop shuts at 3 + (2 + 1 + 3) = 9; no run; open 2 -> 11
	REQUIRE(r.ewt[0] == Catch::Approx(11.0));
}

TEST_CASE("Beamurgia estimator: runTime is the run's own travel model", "[beamurgia]") {
	// A nine-floor run is one kinematic leg, not nine one-floor ones: the
	// kinematic model is not additive in distance.
	KinematicTravelTimeStrategy kinematic;
	std::vector<BeamurgiaCall> calls = {{9, UP, 0.0}};
	BeamurgiaRoute r = beamurgiaEstimateRoute(standing(0, UP), calls, kinematic, kNoDwell);
	REQUIRE(r.ewt[0] == Catch::Approx(kinematic.legDuration(9.0)));
	REQUIRE(r.ewt[0] < 9.0 * kinematic.legDuration(1.0));
}

TEST_CASE("Beamurgia estimator: rejects inputs it cannot route", "[beamurgia]") {
	std::vector<BeamurgiaCall> calls = {{3, UP, 0.0}};
	BeamurgiaCabin zero = standing(0, UP);
	zero.capacity = 0;
	REQUIRE_THROWS_AS(beamurgiaEstimateRoute(zero, calls, kOneSecondPerFloor, kNoDwell), std::invalid_argument);

	std::vector<BeamurgiaCall> noDirection = {{3, 0, 0.0}};
	REQUIRE_THROWS_AS(beamurgiaEstimateRoute(standing(0, UP), noDirection, kOneSecondPerFloor, kNoDwell), std::invalid_argument);

	BeamurgiaCabin movingIdle;
	movingIdle.moving = true;
	movingIdle.direction = 0;
	REQUIRE_THROWS_AS(beamurgiaEstimateRoute(movingIdle, calls, kOneSecondPerFloor, kNoDwell), std::invalid_argument);
}

TEST_CASE("Beamurgia estimator: every call is served, after it was pressed, on random states", "[beamurgia]") {
	// No expected value: the properties that must hold for any state - the route
	// terminates, every call gets an EWT no smaller than what it has already
	// waited, and nobody is served before the cabin could open its doors.
	std::mt19937 gen(53);
	std::uniform_int_distribution<int> floorOf(0, 19);
	std::uniform_int_distribution<int> small(0, 6);
	std::uniform_int_distribution<int> coin(0, 1);
	std::uniform_real_distribution<double> waited(0.0, 60.0);

	for (int trial = 0; trial < 2000; ++trial) {
		BeamurgiaCabin c;
		c.capacity = 1 + small(gen);
		c.moving = coin(gen) && coin(gen);
		int dirPick = small(gen) % 3;
		c.direction = dirPick == 0 ? UP : dirPick == 1 ? DOWN : 0;
		if (c.moving && c.direction == 0) c.direction = UP;
		c.floor = floorOf(gen);
		c.fixedDestination = floorOf(gen);
		c.timeToFixedDestination = c.moving ? 1.0 + small(gen) : 0.0;
		c.doorCloseTime = coin(gen) ? 3.0 : 0.0;
		int aboard = small(gen) % (c.capacity + 1);
		for (int k = 0; k < aboard; ++k) c.carCalls.push_back(floorOf(gen));

		std::vector<BeamurgiaCall> calls;
		int n = small(gen);
		for (int k = 0; k < n; ++k) {
			calls.push_back({floorOf(gen), static_cast<unsigned int>(coin(gen) ? UP : DOWN), waited(gen)});
		}

		// Both reversal rules (plan §6, R1 and R1').
		for (BeamurgiaReversal rule : {BeamurgiaReversal::LastStopAhead, BeamurgiaReversal::ExpectedExtremeFloor}) {
			BeamurgiaEstimatorOptions options;
			options.reversal = rule;
			options.lowestFloor = 0;
			options.highestFloor = 19;
			BeamurgiaRoute r = beamurgiaEstimateRoute(c, calls, kOneSecondPerFloor, kDwell, options);
			REQUIRE(r.ewt.size() == calls.size());
			for (size_t k = 0; k < calls.size(); ++k) {
				REQUIRE(std::isfinite(r.ewt[k]));
				REQUIRE(r.ewt[k] >= calls[k].waited + c.doorCloseTime + 2.0);	// at least DCT_0 + a door opening
			}
			// Each stop serves somebody, except a moving cabin's fixed one.
			REQUIRE(r.stops.size() <= calls.size() + c.carCalls.size() + (c.moving ? 1 : 0));
		}
	}
}

// ---- R1': the expected reversal floor (plan §6) ----

TEST_CASE("Beamurgia R1': with everybody boarding at the lowest floor it is the book's H and S", "[beamurgia]") {
	// Barney & Al-Sharif (2015) eq. 5.14, H = N - sum_{i=1}^{N-1} (i/N)^P, and
	// eq. 5.7, S = N (1 - ((N-1)/N)^P): P passengers leaving the main terminal
	// for N floors above it. Computed here straight from the equations, not
	// from the code under test.
	const int N = 10;
	const int P = 6;
	double sum = 0.0;
	for (int i = 1; i <= N - 1; ++i) sum += std::pow(static_cast<double>(i) / N, P);
	double H = N - sum;
	double S = N * (1.0 - std::pow(static_cast<double>(N - 1) / N, P));

	std::vector<int> atLobby(P, 0);
	REQUIRE(beamurgiaExpectedReversalFloor(0, UP, atLobby, 0, N) == Catch::Approx(H));
	REQUIRE(beamurgiaExpectedStopsBeyond(0, UP, atLobby, 0, N) == Catch::Approx(S));

	// The mirror: the same six leaving the top floor going down.
	std::vector<int> atTop(P, N);
	REQUIRE(beamurgiaExpectedReversalFloor(N, DOWN, atTop, 0, N) == Catch::Approx(N - H));
	REQUIRE(beamurgiaExpectedStopsBeyond(N, DOWN, atTop, 0, N) == Catch::Approx(S));

	// Nobody aboard: no excursion, no stops.
	REQUIRE(beamurgiaExpectedReversalFloor(4, UP, {}, 0, N) == Catch::Approx(4.0));
	REQUIRE(beamurgiaExpectedStopsBeyond(4, UP, {}, 0, N) == Catch::Approx(0.0));
	// Never behind the cabin: at the top there is nowhere further to go.
	REQUIRE(beamurgiaExpectedReversalFloor(N, UP, {3, 6}, 0, N) == Catch::Approx(static_cast<double>(N)));
}

TEST_CASE("Beamurgia R1': the cabin carries on to where it is expected to reverse", "[beamurgia]") {
	// The example the plan's §6 R1' works by hand: floors 0-11, a cabin
	// already braking for the up call at 3, also given the down call at 2 and
	// the up call at 6. Boarders at 3 and 6, destinations uniform above each:
	//   P(max <= j) = (j-3)/8 * (j-6)/5 -> 0, .1, .25, .45, .7 for j = 6..10
	//   H = 11 - 1.5 = 9.5
	//   S = floors 7..11, each 1 - (7/8)(4/5) = 0.3 -> 1.5
	BeamurgiaCabin c;
	c.moving = true;
	c.direction = UP;
	c.fixedDestination = 3;
	c.timeToFixedDestination = 3.0;
	std::vector<BeamurgiaCall> calls = {{3, UP, 0.0}, {2, DOWN, 0.0}, {6, UP, 0.0}};
	BeamurgiaEstimatorOptions r1prime;
	r1prime.reversal = BeamurgiaReversal::ExpectedExtremeFloor;
	r1prime.lowestFloor = 0;
	r1prime.highestFloor = 11;

	REQUIRE(beamurgiaExpectedReversalFloor(6, UP, {3, 6}, 0, 11) == Catch::Approx(9.5));
	REQUIRE(beamurgiaExpectedStopsBeyond(6, UP, {3, 6}, 0, 11) == Catch::Approx(1.5));

	// No dwell, 1 s/floor: R1 reverses at 6 and reaches 2 at 6 + 4 = 10;
	// R1' goes on to 9.5 (3.5 s, linear so the hops add up) and back 7.5: 17.
	BeamurgiaRoute atLastStop = beamurgiaEstimateRoute(c, calls, kOneSecondPerFloor, kNoDwell);
	BeamurgiaRoute expected = beamurgiaEstimateRoute(c, calls, kOneSecondPerFloor, kNoDwell, r1prime);
	REQUIRE(atLastStop.ewt[1] == Catch::Approx(10.0));
	REQUIRE(expected.ewt[1] == Catch::Approx(17.0));
	// Same stops: the excursion is not a stop.
	REQUIRE(expected.stops == atLastStop.stops);
	// The calls served before the sweep ends are not touched.
	REQUIRE(expected.ewt[0] == Catch::Approx(atLastStop.ewt[0]));
	REQUIRE(expected.ewt[2] == Catch::Approx(atLastStop.ewt[2]));

	// With the 2/3/1 dwell:
	//   3 s to 3, open 2 -> call 0 at 5; shut at 3+2+1+3 = 9; 3 s to 6 -> 12;
	//   open 2 -> call 2 at 14; shut at 12+2+1+3 = 18;
	//   excursion 3.5 s + 1.5 stops x (2+3) + expected alighters past 6
	//   (5/8 + 1) x 1 s = 3.5 + 7.5 + 1.625 -> 30.625; 7.5 s to 2 -> 38.125;
	//   open 2 -> call 1 at 40.125.
	BeamurgiaRoute dwelling = beamurgiaEstimateRoute(c, calls, kOneSecondPerFloor, kDwell, r1prime);
	REQUIRE(dwelling.ewt[0] == Catch::Approx(5.0));
	REQUIRE(dwelling.ewt[2] == Catch::Approx(14.0));
	REQUIRE(dwelling.ewt[1] == Catch::Approx(40.125));
}

TEST_CASE("Beamurgia R1': each expected stop pays its own ramps under the kinematic model", "[beamurgia]") {
	// One boarder at 0 going up, floors 0-10: H = 5.5, S = 1 (one passenger,
	// one stop). A second boarder at 0 makes S = 1.9 and H = 10 - sum (i/10)^2
	// = 7.15, run as 1.9 hops of 7.15/1.9 floors, not as one 7.15-floor leg.
	KinematicTravelTimeStrategy kinematic;
	BeamurgiaEstimatorOptions r1prime;
	r1prime.reversal = BeamurgiaReversal::ExpectedExtremeFloor;
	r1prime.lowestFloor = 0;
	r1prime.highestFloor = 10;

	BeamurgiaCabin c = standing(0, UP);
	std::vector<BeamurgiaCall> calls = {{0, UP, 0.0}, {0, UP, 0.0}, {0, DOWN, 0.0}};
	// Both up calls board at 0 (first stop, t = 0, the doors open at once with
	// no dwell); the down call at 0 waits for the excursion and the way back.
	BeamurgiaRoute r = beamurgiaEstimateRoute(c, calls, kinematic, kNoDwell, r1prime);
	double H = beamurgiaExpectedReversalFloor(0, UP, {0, 0}, 0, 10);
	double S = beamurgiaExpectedStopsBeyond(0, UP, {0, 0}, 0, 10);
	REQUIRE(H == Catch::Approx(7.15));
	REQUIRE(S == Catch::Approx(1.9));
	double expectedEwt = S * kinematic.legDuration(H / S) + kinematic.legDuration(-H);
	REQUIRE(r.ewt[2] == Catch::Approx(expectedEwt));
	REQUIRE(r.ewt[2] > kinematic.legDuration(H) + kinematic.legDuration(-H));
}

TEST_CASE("Beamurgia R1': rejects a building it cannot draw destinations from", "[beamurgia]") {
	BeamurgiaEstimatorOptions r1prime;
	r1prime.reversal = BeamurgiaReversal::ExpectedExtremeFloor;
	r1prime.lowestFloor = 0;
	r1prime.highestFloor = 0;
	std::vector<BeamurgiaCall> calls = {{3, UP, 0.0}};
	REQUIRE_THROWS_AS(beamurgiaEstimateRoute(standing(0, UP), calls, kOneSecondPerFloor, kNoDwell, r1prime), std::invalid_argument);

	r1prime.highestFloor = 2;	// the call at 3 is outside
	REQUIRE_THROWS_AS(beamurgiaEstimateRoute(standing(0, UP), calls, kOneSecondPerFloor, kNoDwell, r1prime), std::invalid_argument);

	// R1 does not read the range, so the same call is fine there.
	REQUIRE_NOTHROW(beamurgiaEstimateRoute(standing(0, UP), calls, kOneSecondPerFloor, kNoDwell));
}
