#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "TrafficCapacity.h"

using namespace openEGCS;
using Catch::Approx;

// Anchor tests for the handling-capacity ruler. The capacity calculation is
// pinned to the worked examples of Barney & Al-Sharif (2015), *Elevator Traffic
// Handbook*, 2nd ed., the same way test_lift_motor_model.cpp pins the energy
// model to Peters' worked example: a reported capacity figure is not something
// anyone can judge by eye, so it is tied to a number a textbook printed.

// Worked Example 5.4 (p. 100-101). Given N = 10 floors above the lobby,
// df = 3.0 m, v = 1.5 m/s, tf(1) = 5.0 s, tc = 2.5 s, to = 2.0 s, P = 6.4,
// tp = 1.2 s, the book computes tv = df/v = 2.0 s, ts = 5.0 + 4.5 - 2.0 = 7.5 s,
// S = 4.9, H = 9.09 and RTT = 2*9.09*2.0 + (4.9+1)*7.5 + 2*6.4*1.2 = 95.97 s.
//
// The book carries S and H to two significant figures in the intermediate
// steps; evaluating the formulae exactly gives S = 4.9043, H = 9.0955 and
// RTT = 96.02 s, so the margins below are the book's own rounding, not slack in
// the code. P = 6.4 is reached here as 0.8 * 8 (an eight-person car at the 80%
// load factor), which is how the CLI will supply it.
TEST_CASE("estimateUppeakCapacity reproduces Barney & Al-Sharif Example 5.4", "[traffic_capacity]") {
	UppeakCapacity c = estimateUppeakCapacity(/*floorsAboveLobby=*/10, /*cars=*/1,
		/*carCapacityPersons=*/8.0, /*loadFactor=*/0.8,
		/*singleFloorTransitTv=*/2.0, /*singleFloorFlightTf1=*/5.0,
		/*doorOperatingTd=*/4.5, /*passengerTransferTp=*/1.2);

	REQUIRE(c.passengersPerTrip == Approx(6.4));
	REQUIRE(c.expectedStops == Approx(4.9).margin(0.01));
	REQUIRE(c.highestReversalFloor == Approx(9.09).margin(0.01));
	REQUIRE(c.roundTripTimeS == Approx(95.97).margin(0.1));
}

// S and H standalone on the same inputs, so a regression points at the right
// equation rather than at the RTT that combines all three.
TEST_CASE("expectedStops and highestReversalFloor match eq 5.7 / 5.14 at Example 5.4", "[traffic_capacity]") {
	REQUIRE(expectedStops(10, 6.4) == Approx(4.9).margin(0.01));
	REQUIRE(highestReversalFloor(10, 6.4) == Approx(9.09).margin(0.01));
}

// eq 4.4 and eq 4.5 are arithmetic on RTT; assert the identities hold whatever
// the RTT comes out to, so the struct can never report an interval and a
// capacity that disagree with its own round trip time.
TEST_CASE("interval and handling capacity are consistent with the round trip time", "[traffic_capacity]") {
	UppeakCapacity c = estimateUppeakCapacity(15, 4, 10.0, 0.8, 1.4, 4.0, 3.0, 1.2);

	REQUIRE(c.intervalS == Approx(c.roundTripTimeS / 4.0));
	REQUIRE(c.handlingCapacity == Approx(300.0 / c.roundTripTimeS * c.passengersPerTrip * 4.0));
	// More cars clear more passengers in the same five minutes, proportionally.
	UppeakCapacity one = estimateUppeakCapacity(15, 1, 10.0, 0.8, 1.4, 4.0, 3.0, 1.2);
	REQUIRE(c.handlingCapacity == Approx(one.handlingCapacity * 4.0));
}

// Monotonicities that must hold for any building, no expected value needed:
// a bigger car reaches higher and stops more often (up to the building top),
// and S and H are bounded by the number of served floors.
TEST_CASE("stops and reversal floor move the right way with load and height", "[traffic_capacity]") {
	REQUIRE(expectedStops(20, 12.0) > expectedStops(20, 4.0));
	REQUIRE(highestReversalFloor(20, 12.0) > highestReversalFloor(20, 4.0));

	for (double P : {1.0, 6.4, 20.0}) {
		REQUIRE(expectedStops(20, P) <= 20.0);
		REQUIRE(highestReversalFloor(20, P) <= 20.0);
		REQUIRE(expectedStops(20, P) > 0.0);
		REQUIRE(highestReversalFloor(20, P) > 0.0);
	}
}

// A one-floor-above-lobby building: every passenger shares the single served
// floor, so the average trip makes exactly one stop and reverses at floor 1.
TEST_CASE("a single served floor gives one stop and reversal at floor one", "[traffic_capacity]") {
	REQUIRE(expectedStops(1, 6.4) == Approx(1.0));
	REQUIRE(highestReversalFloor(1, 6.4) == Approx(1.0));
}
