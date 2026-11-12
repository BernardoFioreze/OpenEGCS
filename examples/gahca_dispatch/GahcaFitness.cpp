/*
GahcaFitness.cpp - see GahcaFitness.h. Cortes et al. (2004) section 3.2.
*/

#include "GahcaFitness.h"
#include "Task.h"	// DOWN

#include <algorithm>
#include <limits>

namespace openEGCS {
namespace examples {

namespace {

// Highest floor in `floors`, optionally only those strictly below `pivot`;
// `fallback` when the (filtered) set is empty.
int highest(const std::vector<int>& floors, bool onlyBelowPivot, int pivot, int fallback) {
	int best = std::numeric_limits<int>::min();
	for (int f : floors) {
		if (onlyBelowPivot && f >= pivot) continue;
		if (f > best) best = f;
	}
	return best == std::numeric_limits<int>::min() ? fallback : best;
}

// Lowest floor in `floors`, optionally only those strictly above `pivot`;
// `fallback` when the (filtered) set is empty.
int lowest(const std::vector<int>& floors, bool onlyAbovePivot, int pivot, int fallback) {
	int best = std::numeric_limits<int>::max();
	for (int f : floors) {
		if (onlyAbovePivot && f <= pivot) continue;
		if (f < best) best = f;
	}
	return best == std::numeric_limits<int>::max() ? fallback : best;
}

// A leg never contributes negative time. The three-leg formula is exact in the
// paper's lunchpeak regime (P3 < P4 < P1 < P2 ascending, mirrored descending);
// outside it - a degenerate ordering that uniform traffic can produce - a raw
// term can go negative, and the paper's own words forbid that reading: the
// function "returns the expected time in s". Clamping each leg to >= 0 leaves
// the in-regime value untouched and refuses to report a negative duration. It
// does NOT try to be more accurate than the paper's approximation (e.g. an
// up-call below an ascending cabin with no down-call to trigger the reversal
// estimates as unserved, time ~0 - that imprecision is the paper's "approximate
// function", and reproducing it faithfully means keeping it, not fixing it).
double leg(int a, int b) { return a > b ? static_cast<double>(a - b) : 0.0; }

} // namespace

double gahcaCabinServiceTime(int currentFloor, unsigned int direction,
	const std::vector<int>& upStops, const std::vector<int>& downStops,
	double interFloorTime, double perStopTime)
{
	const int P1 = currentFloor;
	double floorsTravelled;

	if (direction == DOWN) {
		// Descending: down to the lowest down-call (P2), up to the highest
		// up-call (P3), down to the lowest down-call above here (P4, > P1).
		const int P2 = lowest(downStops, /*onlyAbovePivot=*/false, P1, /*fallback=*/P1);
		const int P3 = highest(upStops, /*onlyBelowPivot=*/false, P1, /*fallback=*/P2);
		const int P4 = lowest(downStops, /*onlyAbovePivot=*/true, P1, /*fallback=*/P3);
		floorsTravelled = leg(P1, P2) + leg(P3, P2) + leg(P3, P4);
	} else {
		// Stopped or ascending: up to the highest up-call (P2), down to the
		// lowest down-call (P3), up to the highest up-call below here (P4, < P1).
		const int P2 = highest(upStops, /*onlyBelowPivot=*/false, P1, /*fallback=*/P1);
		const int P3 = lowest(downStops, /*onlyAbovePivot=*/false, P1, /*fallback=*/P2);
		const int P4 = highest(upStops, /*onlyBelowPivot=*/true, P1, /*fallback=*/P3);
		floorsTravelled = leg(P2, P1) + leg(P2, P3) + leg(P4, P3);
	}

	const double stops = static_cast<double>(upStops.size() + downStops.size());
	return floorsTravelled * interFloorTime + stops * perStopTime;
}

} // namespace examples
} // namespace openEGCS
