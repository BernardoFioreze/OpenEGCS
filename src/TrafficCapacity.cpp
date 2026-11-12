/*
TrafficCapacity.cpp

Barney & Al-Sharif (2015) ch. 4-5, transcribed from the definitions. The whole
file is arithmetic; the only judgement is in the header (why a function, how the
times map onto the book's parameters). The worked Example 5.4 (p. 100-101) is
the anchor test in tests/test_traffic_capacity.cpp - its inputs and its printed
S/H/RTT are what this code is checked against, the same way the energy model is
pinned to Peters' worked example.
*/

#include "TrafficCapacity.h"

#include <cmath>

namespace openEGCS {

double expectedStops(int floorsAboveLobby, double passengers) {
	// eq 5.7: S = N*(1 - ((N-1)/N)^P). With N = 1 every passenger shares the one
	// served floor, so the car makes exactly one stop, which the formula already
	// gives (1*(1 - 0^P) = 1 for P > 0); the guard is only against N <= 0.
	const double N = static_cast<double>(floorsAboveLobby);
	if (N <= 0.0) return 0.0;
	return N * (1.0 - std::pow((N - 1.0) / N, passengers));
}

double highestReversalFloor(int floorsAboveLobby, double passengers) {
	// eq 5.14: H = N - sum_{i=1}^{N-1} (i/N)^P. The sum is the probability mass
	// that the trip reverses below floor i accumulated floor by floor; summing
	// the definition is exact and cheap for any building this simulator runs.
	const double N = static_cast<double>(floorsAboveLobby);
	if (N <= 0.0) return 0.0;
	double sum = 0.0;
	for (int i = 1; i <= floorsAboveLobby - 1; ++i) {
		sum += std::pow(static_cast<double>(i) / N, passengers);
	}
	return N - sum;
}

UppeakCapacity estimateUppeakCapacity(int floorsAboveLobby, int cars,
	double carCapacityPersons, double loadFactor,
	double singleFloorTransitTv, double singleFloorFlightTf1,
	double doorOperatingTd, double passengerTransferTp) {

	const double P = loadFactor * carCapacityPersons;	// average car load per trip
	const double S = expectedStops(floorsAboveLobby, P);
	const double H = highestReversalFloor(floorsAboveLobby, P);

	// ts is the "artificial" stopping time of eq 4.9: the flight + door time of
	// one stop, minus the rated-speed transit the 2*H*tv term already charged.
	const double ts = singleFloorFlightTf1 + doorOperatingTd - singleFloorTransitTv;

	// eq 4.8: the three terms are travel (H), stopping (S) and transfer (P).
	const double rtt = 2.0 * H * singleFloorTransitTv
		+ (S + 1.0) * ts
		+ 2.0 * P * passengerTransferTp;

	const double interval = rtt / static_cast<double>(cars);			// eq 4.5
	const double upphc = (300.0 / rtt) * P * static_cast<double>(cars);	// eq 4.4

	return UppeakCapacity{H, S, P, rtt, interval, upphc};
}

} // namespace openEGCS
