#include "BeamurgiaEstimator.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <tuple>

namespace openEGCS {
namespace examples {

namespace {

bool isTravelDirection(unsigned int d)
{
	return d == UP || d == DOWN;
}

// Strictly ahead of `pos` going `dir`. A floor the cabin is standing at is not
// ahead: whatever was there has been served by the stop it just made.
bool isAhead(int floor, double pos, unsigned int dir)
{
	return dir == UP ? floor > pos : floor < pos;
}

// A candidate next stop: a car call (`call` = -1, `carCall` its index) or a
// landing call (`call` its index in the input).
struct Candidate {
	int floor;
	long call;
};

// P(a passenger who boarded at `boardedAt` going `dir` gets out at or before
// floor `j`), destinations uniform over the floors beyond `boardedAt`
// (plan §6 R1'). "At or before" is in the direction of travel: <= j going UP,
// >= j going DOWN. A boarder with no floor beyond is certain to be out.
double outBy(int boardedAt, unsigned int dir, int j, int lowestFloor, int highestFloor)
{
	int beyond = dir == UP ? highestFloor - boardedAt : boardedAt - lowestFloor;
	if (beyond <= 0) return 1.0;
	int travelled = dir == UP ? j - boardedAt : boardedAt - j;
	return std::min(1.0, std::max(0.0, static_cast<double>(travelled) / beyond));
}

// P(that passenger gets out at exactly floor `j`).
double outAt(int boardedAt, unsigned int dir, int j, int lowestFloor, int highestFloor)
{
	int beyond = dir == UP ? highestFloor - boardedAt : boardedAt - lowestFloor;
	bool reachable = dir == UP ? (j > boardedAt && j <= highestFloor) : (j < boardedAt && j >= lowestFloor);
	return beyond > 0 && reachable ? 1.0 / beyond : 0.0;
}

void requireRange(int lowestFloor, int highestFloor)
{
	if (highestFloor <= lowestFloor) {
		throw std::invalid_argument("beamurgia R1': the building needs at least two floors (highestFloor > lowestFloor)");
	}
}

} // namespace

double beamurgiaExpectedReversalFloor(double pos, unsigned int direction,
	const std::vector<int>& boardingFloors, int lowestFloor, int highestFloor)
{
	requireRange(lowestFloor, highestFloor);
	if (!isTravelDirection(direction)) {
		throw std::invalid_argument("beamurgiaExpectedReversalFloor: direction must be UP or DOWN");
	}
	int at = static_cast<int>(std::lround(pos));
	// E[max(pos, M)] = top - sum_{j=pos}^{top-1} P(M <= j), with P(M <= j) the
	// product over passengers - eqs. 5.10/5.11 with a factor per passenger.
	// Going DOWN, the mirror: bottom + sum_{j=bottom+1}^{pos} P(m >= j).
	double sum = 0.0;
	if (direction == UP) {
		for (int j = at; j < highestFloor; ++j) {
			double allOut = 1.0;
			for (int f : boardingFloors) allOut *= outBy(f, UP, j, lowestFloor, highestFloor);
			sum += allOut;
		}
		return highestFloor - sum;
	}
	for (int j = lowestFloor + 1; j <= at; ++j) {
		double allOut = 1.0;
		for (int f : boardingFloors) allOut *= outBy(f, DOWN, j, lowestFloor, highestFloor);
		sum += allOut;
	}
	return lowestFloor + sum;
}

double beamurgiaExpectedStopsBeyond(double pos, unsigned int direction,
	const std::vector<int>& boardingFloors, int lowestFloor, int highestFloor)
{
	requireRange(lowestFloor, highestFloor);
	if (!isTravelDirection(direction)) {
		throw std::invalid_argument("beamurgiaExpectedStopsBeyond: direction must be UP or DOWN");
	}
	int at = static_cast<int>(std::lround(pos));
	// Eq. 5.6 per floor - 1 minus the probability that nobody gets out there -
	// summed over the floors beyond `pos`, as eq. 5.7 sums it over the building.
	double stops = 0.0;
	int first = direction == UP ? at + 1 : lowestFloor;
	int last = direction == UP ? highestFloor : at - 1;
	for (int j = first; j <= last; ++j) {
		double nobody = 1.0;
		for (int f : boardingFloors) nobody *= 1.0 - outAt(f, direction, j, lowestFloor, highestFloor);
		stops += 1.0 - nobody;
	}
	return stops;
}

BeamurgiaRoute beamurgiaEstimateRoute(const BeamurgiaCabin& cabin,
	const std::vector<BeamurgiaCall>& calls,
	const ITravelTimeStrategy& travel, const IDwellTimeStrategy& dwell,
	const BeamurgiaEstimatorOptions& options)
{
	if (cabin.capacity < 1) {
		throw std::invalid_argument("beamurgiaEstimateRoute: capacity must be at least 1 passenger");
	}
	if (cabin.moving && !isTravelDirection(cabin.direction)) {
		throw std::invalid_argument("beamurgiaEstimateRoute: a moving cabin must be going UP or DOWN");
	}
	if (!(cabin.doorCloseTime >= 0.0) || !(cabin.timeToFixedDestination >= 0.0)) {
		throw std::invalid_argument("beamurgiaEstimateRoute: times must be non-negative");
	}
	bool expectedReversal = options.reversal == BeamurgiaReversal::ExpectedExtremeFloor;
	if (expectedReversal) {
		requireRange(options.lowestFloor, options.highestFloor);
		auto inside = [&](int f) { return f >= options.lowestFloor && f <= options.highestFloor; };
		bool ok = cabin.moving ? inside(cabin.fixedDestination) : inside(cabin.floor);
		for (int f : cabin.carCalls) ok = ok && inside(f);
		for (const BeamurgiaCall& c : calls) ok = ok && inside(c.floor);
		if (!ok) {
			throw std::invalid_argument("beamurgiaEstimateRoute: a floor lies outside [lowestFloor, highestFloor]");
		}
	}
	for (const BeamurgiaCall& c : calls) {
		if (!isTravelDirection(c.direction)) {
			throw std::invalid_argument("beamurgiaEstimateRoute: a landing call must be UP or DOWN");
		}
		if (!(c.waited >= 0.0)) {
			throw std::invalid_argument("beamurgiaEstimateRoute: a call cannot have waited a negative time");
		}
	}

	BeamurgiaRoute out;
	out.ewt.assign(calls.size(), 0.0);

	std::vector<bool> served(calls.size(), false);
	size_t remaining = calls.size();
	std::vector<int> carCalls = cabin.carCalls;
	std::vector<int> boardedAt;		// where each passenger boarded on this sweep, destination unknown (file comment)

	double t = cabin.doorCloseTime;	// DCT_0, then every stop's whole dwell and every leg
	double pos = cabin.floor;		// a floor, except after an R1' excursion, which ends at the expected (fractional) reversal floor
	unsigned int dir = isTravelDirection(cabin.direction) ? cabin.direction : 0;
	bool firstStop = true;

	// Serves one stop at `floor`, reached going `arrivalDir` (0 = no
	// direction), chosen for landing call `target` (-1 = a car call or the
	// fixed destination). `t` is the arrival instant on entry and the instant
	// the doors are shut again on return. Returns how many people got in or
	// out, which the loop below uses as its progress check.
	auto serveStop = [&](int floor, unsigned int arrivalDir, long target) -> int {
		int alighting = static_cast<int>(std::count(carCalls.begin(), carCalls.end(), floor));
		carCalls.erase(std::remove(carCalls.begin(), carCalls.end(), floor), carCalls.end());

		int carrying = static_cast<int>(carCalls.size() + boardedAt.size());
		unsigned int boardDir = arrivalDir;
		// A target call is boarded unless people aboard already fixed the
		// direction before this stop (`dir` is still the pre-arrival one). The
		// second clause is the parked cabin that has car calls but no direction
		// yet: nothing fixes it, and without it a target at the cabin's own
		// floor would be chosen and refused forever.
		if (target >= 0 && (carrying == 0 || !isTravelDirection(dir))) {
			boardDir = calls[target].direction;
		} else if (carrying == 0) {
			// A car-call stop (or the fixed destination) that leaves the cabin
			// empty: keep going the way it came if somebody waits that way
			// here, else take whoever waits the other way.
			bool sameWay = false;
			long other = -1;
			for (size_t k = 0; k < calls.size(); ++k) {
				if (served[k] || calls[k].floor != floor) continue;
				if (calls[k].direction == arrivalDir) sameWay = true;
				else if (other < 0) other = static_cast<long>(k);
			}
			if (!sameWay && other >= 0) boardDir = calls[other].direction;
		}

		std::vector<size_t> boarders;
		for (size_t k = 0; k < calls.size(); ++k) {
			if (served[k] || calls[k].floor != floor || calls[k].direction != boardDir) continue;
			if (carrying + static_cast<int>(boarders.size()) >= cabin.capacity) break;
			boarders.push_back(k);
		}

		StopContext stop;
		stop.boarding = static_cast<int>(boarders.size());
		stop.alighting = alighting;
		StopContext alightersOnly;
		alightersOnly.alighting = alighting;

		double open = dwell.getDoorOpenTime(stop);
		double dtt = firstStop ? 0.0 : dwell.getTransferTime(alightersOnly);	// DTT_0 = 0 (section 4.2)
		for (size_t k : boarders) {
			out.ewt[k] = calls[k].waited + t + open + dtt;
			served[k] = true;
			--remaining;
		}
		for (size_t k = 0; k < boarders.size(); ++k) boardedAt.push_back(floor);

		t += open + dwell.getTransferTime(stop) + dwell.getDoorCloseTime(stop);
		pos = floor;
		if (isTravelDirection(boardDir)) dir = boardDir;
		firstStop = false;
		out.stops.push_back(floor);
		return alighting + static_cast<int>(boarders.size());
	};

	if (cabin.moving) {
		t += cabin.timeToFixedDestination;
		serveStop(cabin.fixedDestination, cabin.direction, -1);
	}

	// "If a lift is stopped and has no assigned direction, it will attend to
	// the passenger with the longest waiting time" (section 3) - the first
	// pick of a parked, empty cabin only.
	bool longestWaitingFirst = !cabin.moving && !isTravelDirection(cabin.direction)
		&& cabin.carCalls.empty();

	while (remaining > 0) {
		int carrying = static_cast<int>(carCalls.size() + boardedAt.size());
		bool full = carrying >= cabin.capacity;
		bool locked = carrying > 0 && isTravelDirection(dir);	// Closs: no reversal, no boarding against

		std::vector<Candidate> candidates;
		for (int f : carCalls) {
			if (!locked || isAhead(f, pos, dir)) candidates.push_back({f, -1});
		}
		if (!full) {
			for (size_t k = 0; k < calls.size(); ++k) {
				if (served[k]) continue;
				if (locked && (calls[k].direction != dir || !isAhead(calls[k].floor, pos, dir))) continue;
				candidates.push_back({calls[k].floor, static_cast<long>(k)});
			}
		}

		if (candidates.empty()) {
			if (!boardedAt.empty()) {
				// End of the sweep: whoever boarded in the estimate is taken to
				// have left. Under R1 by the last stop ahead, where the cabin
				// stands; under R1' the cabin first carries on to where it is
				// expected to reverse (file comment, plan §6).
				if (expectedReversal) {
					int lo = options.lowestFloor, hi = options.highestFloor;
					double reversal = beamurgiaExpectedReversalFloor(pos, dir, boardedAt, lo, hi);
					double stops = beamurgiaExpectedStopsBeyond(pos, dir, boardedAt, lo, hi);
					double alighters = 0.0;	// expected number still aboard past `pos`
					int at = static_cast<int>(std::lround(pos));
					for (int f : boardedAt) alighters += 1.0 - outBy(f, dir, at, lo, hi);

					// The run is `stops` equal hops rather than one leg, so each
					// expected stop pays its own ramps - the kinematic model is
					// not additive in distance. Fewer than one expected stop is
					// still one leg.
					double hops = std::max(1.0, stops);
					if (reversal != pos) t += hops * travel.legDuration((reversal - pos) / hops);

					// Each expected stop opens and shuts once; the transfer is
					// paid per expected alighter, like eq. 4.8's separate
					// passenger-transfer term.
					StopContext one;
					one.alighting = 1;
					t += stops * (dwell.getDoorOpenTime(one) + dwell.getDoorCloseTime(one))
						+ alighters * dwell.getTransferTime(one);
					pos = reversal;
				}
				boardedAt.clear();
				continue;
			}
			// Only real passengers left, and none of them is ahead: a state the
			// engine does not forbid (Closs rule 4 is not enforced by the engine).
			// Go to the nearest car call wherever it is.
			for (int f : carCalls) candidates.push_back({f, -1});
		}

		auto key = [&](const Candidate& c) {
			bool isLanding = c.call >= 0;
			bool notAhead = !(isTravelDirection(dir) && isAhead(c.floor, pos, dir));
			int dirRank = isLanding && calls[c.call].direction == DOWN ? 1 : 0;
			return std::make_tuple(std::abs(c.floor - pos), isLanding, notAhead, c.floor, dirRank, c.call);
		};
		Candidate next = *std::min_element(candidates.begin(), candidates.end(),
			[&](const Candidate& a, const Candidate& b) { return key(a) < key(b); });

		if (longestWaitingFirst) {
			longestWaitingFirst = false;
			if (next.call >= 0) {
				for (const Candidate& c : candidates) {
					if (c.call >= 0 && calls[c.call].waited > calls[next.call].waited) next = c;
				}
			}
		}

		unsigned int arrivalDir = next.floor > pos ? UP : next.floor < pos ? DOWN : dir;
		if (next.floor != pos) t += travel.legDuration(next.floor - pos);
		// Termination rests on every chosen stop serving somebody: the
		// candidate filter above and the boarding rule in serveStop must agree
		// on who can board. If they ever disagree the loop would choose the
		// same stop forever (it did, under a mutation that dropped the Closs
		// lock: the test process grew to 25 GB before the OOM killer took it),
		// so a stop that serves nobody is a bug, reported at once.
		if (serveStop(next.floor, arrivalDir, next.call) == 0) {
			throw std::logic_error("beamurgiaEstimateRoute: chose a stop that serves nobody");
		}
	}

	return out;
}

} // namespace examples
} // namespace openEGCS
