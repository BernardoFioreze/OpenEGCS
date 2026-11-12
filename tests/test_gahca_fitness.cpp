#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <vector>
#include "GahcaFitness.h"
#include "Task.h"	// UP, DOWN

using namespace openEGCS;
using namespace openEGCS::examples;

// The GAHCA per-cabin service-time
// estimator (Cortes et al. 2004, section 3.2). Pure function, so these are
// hand-computed from the paper's four-peak formula, directly against its
// definitions - no engine, no simulation. Each case names the P1-P4 it exercises.

TEST_CASE("GAHCA fitness: ascending, the paper's canonical three-leg sweep", "[gahca]") {
	// Cabin at 5 going up, up-calls at {3,8}, down-call at {2}.
	//   P2 = highest up = 8, P3 = lowest down = 2, P4 = highest up below 5 = 3
	//   floors = (8-5) + (8-2) + (3-2) = 3 + 6 + 1 = 10
	double t = gahcaCabinServiceTime(5, UP, {3, 8}, {2}, /*interFloor=*/1.0, /*perStop=*/0.0);
	REQUIRE(t == Catch::Approx(10.0));
}

TEST_CASE("GAHCA fitness: descending is the mirror", "[gahca]") {
	// Cabin at 5 going down, down-calls at {2,7}, up-call at {9}.
	//   P2 = lowest down = 2, P3 = highest up = 9, P4 = lowest down above 5 = 7
	//   floors = (5-2) + (9-2) + (9-7) = 3 + 7 + 2 = 12
	double t = gahcaCabinServiceTime(5, DOWN, {9}, {2, 7}, /*interFloor=*/1.0, /*perStop=*/0.0);
	REQUIRE(t == Catch::Approx(12.0));
}

TEST_CASE("GAHCA fitness: the paper's delays (5 s/floor, 13 s/stop)", "[gahca]") {
	// A lobby cabin (floor 0) with an up-call at 5 and a down-call at 3, under
	// the paper's reproduction constants.
	//   P2 = 5, P3 = 3, P4 = none below 0 -> P3 = 3; floors = (5-0)+(5-3)+(3-3) = 7
	//   time = 7*5 + 2 stops*13 = 35 + 26 = 61
	double t = gahcaCabinServiceTime(0, UP, {5}, {3}, /*interFloor=*/5.0, /*perStop=*/13.0);
	REQUIRE(t == Catch::Approx(61.0));
}

TEST_CASE("GAHCA fitness: a floor served both ways is two stops", "[gahca]") {
	// Up and down call at the same floor 5, from the lobby.
	//   floors = (5-0)+(5-5)+(5-5) = 5; stops = 2; time = 5*1 + 2*10 = 25
	double t = gahcaCabinServiceTime(0, UP, {5}, {5}, /*interFloor=*/1.0, /*perStop=*/10.0);
	REQUIRE(t == Catch::Approx(25.0));
}

TEST_CASE("GAHCA fitness: missing legs collapse to zero", "[gahca]") {
	// Only an up-call above, no down-call: just the upward leg.
	//   P2 = 8, P3 -> P2 = 8, P4 -> P3 = 8; floors = (8-5)+0+0 = 3
	REQUIRE(gahcaCabinServiceTime(5, UP, {8}, {}, 1.0, 0.0) == Catch::Approx(3.0));

	// Ascending cabin with only a down-call below: it goes down to serve it.
	//   P2 -> P1 = 5, P3 = 2, P4 -> P3 = 2; floors = 0+(5-2)+0 = 3
	REQUIRE(gahcaCabinServiceTime(5, UP, {}, {2}, 1.0, 0.0) == Catch::Approx(3.0));

	// No calls at all: zero.
	REQUIRE(gahcaCabinServiceTime(5, UP, {}, {}, 1.0, 0.0) == Catch::Approx(0.0));
}

TEST_CASE("GAHCA fitness: the paper's approximation is reproduced, not corrected", "[gahca]") {
	// An up-call below an ascending cabin with no down-call to trigger the
	// reversal: the three-leg estimate treats it as unserved, time ~0. This is
	// the paper's "approximate function" (section 3.2), kept on purpose - a
	// geometrically exact estimator (the cabin would really descend to 3) would
	// be a DIFFERENT estimator, not Cortes'. See GahcaFitness.h.
	REQUIRE(gahcaCabinServiceTime(5, UP, {3}, {}, 1.0, 0.0) == Catch::Approx(0.0));

	// The function never reports a negative duration, whatever the ordering.
	REQUIRE(gahcaCabinServiceTime(5, UP, {1, 2}, {}, 1.0, 0.0) >= 0.0);
	REQUIRE(gahcaCabinServiceTime(5, DOWN, {}, {8, 9}, 1.0, 0.0) >= 0.0);
}
