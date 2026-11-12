/*
TrafficCapacity.h

The a-priori handling-capacity ruler: given a building and its lifts, how many
passengers can the group clear in the peak five minutes, and therefore at what
arrival rate does the configured scenario stop being a steady regime and start
being a building that only drains? Nothing here runs the simulation - it is the
sizing calculation a lift engineer does on paper before a single car is bought,
and the CLI uses it to warn when the demand it was handed exceeds it.

Why a concrete function and NOT a pluggable interface. Every other "how should
this be computed" in this project is an I...Strategy because it is a knob the
platform exists to compare dispatch algorithms across. Handling capacity is the
opposite: it is *the* reference figure the whole field sizes systems against,
one equation with "international acceptance" (Barney & Al-Sharif, below eq
4.12), not one of several rival models. It is the sibling of the energy model
(LiftMotorModel.h), which is likewise a concrete, literature-anchored
calculation in the core rather than a strategy.

Everything here is Barney, G.C. & Al-Sharif, L. (2015), *Elevator Traffic
Handbook: Theory and Practice*, 2nd ed., chapters 4-5. Equation numbers are the
book's own. The identical figure is reached by Siikonen (2000) as
HC = 0.8*CC*300/I with I = RTT/L - the two sources agree term for term.

UNITS. Everything is SI: seconds, metres, persons. One simulation time unit is
one second, and the book's 300 is five minutes, so the clock and the literature
speak the same unit.
*/

#ifndef __TRAFFICCAPACITY__
#define __TRAFFICCAPACITY__

namespace openEGCS {

// The six parameters the round-trip-time equation (Barney & Al-Sharif eq 4.8)
// is built from, plus the three figures a designer reads off it. Returned as a
// whole so a caller can print the chain, not just the verdict - the point of a
// capacity warning is to be auditable, which a bare boolean is not.
struct UppeakCapacity {
	double highestReversalFloor;	// H, eq 5.14: how high the average trip goes
	double expectedStops;			// S, eq 5.7: how many stops the average trip makes
	double passengersPerTrip;		// P = loadFactor * car capacity
	double roundTripTimeS;			// RTT, eq 4.8 (seconds)
	double intervalS;				// INT = RTT / L, eq 4.5 (seconds between car arrivals)
	double handlingCapacity;		// UPPHC, eq 4.4 (persons per 5 minutes)
};

// Expected number of stops a car makes carrying `passengers` to uniformly
// random destinations among `floorsAboveLobby` floors (Barney & Al-Sharif
// eq 5.7, Basset Jones 1923): S = N*(1 - ((N-1)/N)^P). Public because it is
// the smallest literature-anchored piece and is worth testing on its own.
double expectedStops(int floorsAboveLobby, double passengers);

// Expected highest floor the car reverses at (eq 5.14, Schroeder 1955):
// H = N - sum_{i=1}^{N-1} (i/N)^P. Computed from the definition every time
// rather than from Table 5.11, so it is exact for any (N, P) and not limited to
// the book's tabulated cases.
double highestReversalFloor(int floorsAboveLobby, double passengers);

// The up-peak handling-capacity calculation (Barney & Al-Sharif ch. 4-5),
// evaluated from the installation's own physics. Preconditions, all the
// caller's job to meet (the function does not guess a sensible default for a
// nonsensical building): floorsAboveLobby >= 1, cars >= 1, carCapacityPersons
// finite and > 0 (an unlimited cabin has no handling capacity to speak of, so
// the CLI must not call this without a configured <Elevator max weight>),
// loadFactor in (0, 1], and every time in seconds and >= 0.
//
// The time inputs map one-to-one onto the book's parameters, and the mapping is
// where the faithfulness lives:
//   singleFloorTransitTv  tv  = df / v_rated  (eq 4.21, transit past a floor at
//                               rated speed; this is NOT the kinematic rest-to-
//                               rest time, it is the express-run cost the 2*H*tv
//                               term prices)
//   singleFloorFlightTf1  tf(1) (eq 4.20, doors-locked to level at the next
//                               floor, rest to rest - the kinematic leg)
//   doorOperatingTd       td  = tc + to  (eq 4.17)
//   passengerTransferTp   tp  (eq 4.14, per passenger, one way)
// from which ts = tf(1) + td - tv (eq 4.9) and
//   RTT = 2*H*tv + (S+1)*ts + 2*P*tp     (eq 4.8).
// Mixing tv (rated-speed transit) and tf(1) (rest-to-rest flight) is the book's
// own construction, not a shortcut: ts is "an artificial time ... that cannot be
// measured directly" precisely because it is the stopping overhead left over
// once the rated-speed travel is accounted separately.
UppeakCapacity estimateUppeakCapacity(int floorsAboveLobby, int cars,
	double carCapacityPersons, double loadFactor,
	double singleFloorTransitTv, double singleFloorFlightTf1,
	double doorOperatingTd, double passengerTransferTp);

} // namespace openEGCS

#endif
