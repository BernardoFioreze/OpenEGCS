/*
BeamurgiaDispatchStrategy.cpp - see BeamurgiaDispatchStrategy.h. Beamurgia et
al. (2016).
*/

#include "BeamurgiaDispatchStrategy.h"
#include "Task.h"	// UP, DOWN

#include <algorithm>
#include <climits>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace openEGCS {
namespace examples {

namespace {

// A uniform number in [0,1) from the injected provider, which only offers
// integer/exponential/normal draws - the same construction as GAHCA, so
// determinism stays tied to one source.
double uniform01(IRandomProvider& rng)
{
	return rng.nextUniformInt(0, 1000000) / 1000001.0;
}

bool isTravelDirection(unsigned int d)
{
	return d == UP || d == DOWN;
}

} // namespace

struct BeamurgiaDispatchStrategy::Context {
	double now = 0.0;
	std::vector<BeamurgiaCabin> cabins;
	std::vector<std::vector<BeamurgiaCall> > background;	// per cabin: committed calls at its fixed stop that are not genes
	std::vector<const ITravelTimeStrategy*> travel;
	std::vector<const IDwellTimeStrategy*> dwell;
	std::vector<double> positions;
	double realWaitingTime = 0.0;	// RWT: mean time the calls handed over have already waited (eq. 9)
};

BeamurgiaDispatchStrategy::BeamurgiaDispatchStrategy() : params() {}
BeamurgiaDispatchStrategy::BeamurgiaDispatchStrategy(Params parameters) : params(parameters) {}

BeamurgiaCabin BeamurgiaDispatchStrategy::cabinStateOf(const Elevator& cabin, double now, double passengerMassKg)
{
	BeamurgiaCabin s;
	const std::vector<Task>& route = cabin.getRoute();
	const double pos = cabin.getPosition();
	const double dwellLeft = std::max(0.0, cabin.getLegStartTime() - now);

	const double maxWeight = cabin.getMaxWeight();
	if (std::isinf(maxWeight) || !(passengerMassKg > 0.0)) {
		s.capacity = INT_MAX;
	} else {
		s.capacity = std::max(1, static_cast<int>(std::floor(maxWeight / passengerMassKg)));
	}

	const bool standingAtFront = !route.empty() && std::fabs(route.front().getDest() - pos) < 1e-9;
	for (size_t k = 0; k < route.size(); ++k) {
		const Task& stop = route[k];
		if (!stop.hasCarCallRequest()) continue;
		if (k == 0 && standingAtFront) continue;	// being served: they are getting out now
		int alighting = std::max(1, stop.getAlightingPassengers());
		for (int p = 0; p < alighting; ++p) s.carCalls.push_back(stop.getDest());
	}

	if (route.empty() || standingAtFront) {
		s.moving = false;
		s.floor = route.empty() ? static_cast<int>(std::lround(pos)) : route.front().getDest();
		s.direction = cabin.getDirection();	// IDLE for a parked cabin: the estimator's "no assigned direction"
		s.doorCloseTime = dwellLeft;
		return s;
	}

	const Task& front = route.front();
	s.moving = true;
	s.fixedDestination = front.getDest();
	s.direction = cabin.getDirection();
	if (!isTravelDirection(s.direction)) s.direction = front.getDest() > pos ? UP : DOWN;
	s.timeToFixedDestination = std::max(0.0, cabin.getStopArrivalTime(front.getStopId()) - now);
	s.floor = static_cast<int>(std::lround(pos));
	return s;
}

BeamurgiaDispatchStrategy::Context BeamurgiaDispatchStrategy::contextOf(
	const std::vector<HallCall>& calls, const std::vector<Elevator>& elevators) const
{
	Context ctx;
	// Elevator::getTime() is not const, and the engine hands the elevators as
	// a const reference; a copy reads the clock without touching the engine.
	// Every cabin's clock was advanced to the event's instant before the
	// consultation (Simulator: system.update, then system.giveEvent).
	if (!elevators.empty()) {
		Elevator probe = elevators.front();
		ctx.now = probe.getTime();
	}

	double waitedSum = 0.0;
	for (const HallCall& c : calls) waitedSum += std::max(0.0, ctx.now - c.registeredAt);
	ctx.realWaitingTime = calls.empty() ? 0.0 : waitedSum / calls.size();

	for (const Elevator& e : elevators) {
		BeamurgiaCabin state = cabinStateOf(e, ctx.now, params.passengerMassKg);
		ctx.cabins.push_back(state);
		ctx.positions.push_back(e.getPosition());
		if (!e.getTravelTimeStrategy() || !e.getDwellTimeStrategy()) {
			throw std::logic_error("BeamurgiaDispatchStrategy: a cabin has no travel or dwell model");
		}
		ctx.travel.push_back(e.getTravelTimeStrategy().get());
		ctx.dwell.push_back(e.getDwellTimeStrategy().get());

		// The landing calls at a moving cabin's fixed first stop board there
		// whatever the assignment; the ones not handed over as genes (the
		// committed ones) ride in the estimate, so the route the others wait
		// on carries them. Their own EWT is not summed: their stop is fixed,
		// so it is the same in every individual.
		std::vector<BeamurgiaCall> fixedBoarders;
		if (state.moving) {
			const Task& front = e.getRoute().front();
			for (unsigned int dir : {static_cast<unsigned int>(UP), static_cast<unsigned int>(DOWN)}) {
				if (!front.hasHallCall(dir)) continue;
				bool isGene = false;
				for (const HallCall& c : calls) {
					if (c.floor == front.getDest() && c.direction == dir) { isGene = true; break; }
				}
				if (!isGene) fixedBoarders.push_back(BeamurgiaCall{front.getDest(), dir, 0.0});
			}
		}
		ctx.background.push_back(fixedBoarders);
	}
	return ctx;
}

double BeamurgiaDispatchStrategy::evaluate(const std::vector<int>& assignment,
	const std::vector<HallCall>& calls, const Context& ctx) const
{
	const int N = static_cast<int>(ctx.cabins.size());
	double sum = 0.0;
	bool anyLong = false;
	double dynamicPenalty = 0.0;
	const double reference = std::max(ctx.realWaitingTime, params.waitingThreshold);

	for (int c = 0; c < N; ++c) {
		std::vector<BeamurgiaCall> mine;
		for (size_t j = 0; j < calls.size(); ++j) {
			if (assignment[j] != c + 1) continue;
			mine.push_back(BeamurgiaCall{calls[j].floor, calls[j].direction,
				std::max(0.0, ctx.now - calls[j].registeredAt)});
		}
		if (mine.empty()) continue;	// a cabin with no gene adds nothing to the sum
		const size_t genes = mine.size();
		mine.insert(mine.end(), ctx.background[c].begin(), ctx.background[c].end());

		BeamurgiaRoute route = beamurgiaEstimateRoute(ctx.cabins[c], mine,
			*ctx.travel[c], *ctx.dwell[c], params.estimator);
		for (size_t k = 0; k < genes; ++k) {
			const double ewt = route.ewt[k];
			sum += ewt;
			if (ewt > params.waitingThreshold) anyLong = true;
			const double excess = std::max(0.0, ewt - reference);
			dynamicPenalty += excess * excess;
		}
	}

	switch (params.penalty) {
		case Penalty::Death:	return sum + (anyLong ? params.deathPenaltyPerCar * N : 0.0);
		case Penalty::Dynamic:	return sum + dynamicPenalty;
		case Penalty::None:		break;
	}
	return sum;
}

double BeamurgiaDispatchStrategy::objective(const std::vector<int>& assignment,
	const std::vector<HallCall>& calls, const std::vector<Elevator>& elevators) const
{
	return evaluate(assignment, calls, contextOf(calls, elevators));
}

std::vector<int> BeamurgiaDispatchStrategy::seededIndividual(const std::vector<HallCall>& calls,
	const std::vector<Elevator>& elevators, const Context& ctx) const
{
	// Sd: each call to "the nearest lifts that satisfy all the requirements and
	// assumptions explained in Sect. 4" - room for one more passenger and, for
	// a cabin carrying people, the call going its way and ahead of it (Closs).
	// Nearest by distance from the cabin's position; ties to the lower id; no
	// cabin satisfying the rules -> the nearest one regardless.
	std::vector<int> seed(calls.size());
	for (size_t j = 0; j < calls.size(); ++j) {
		if (calls[j].committed) { seed[j] = calls[j].currentElevatorId; continue; }
		int bestCompatible = 0, bestAny = 0;
		double dCompatible = std::numeric_limits<double>::infinity();
		double dAny = std::numeric_limits<double>::infinity();
		for (size_t c = 0; c < elevators.size(); ++c) {
			const BeamurgiaCabin& cab = ctx.cabins[c];
			const double pos = ctx.positions[c];
			const double d = std::fabs(calls[j].floor - pos);
			if (d < dAny) { dAny = d; bestAny = static_cast<int>(c) + 1; }

			bool room = static_cast<int>(cab.carCalls.size()) < cab.capacity;
			bool compatible = room;
			if (compatible && !cab.carCalls.empty() && isTravelDirection(cab.direction)) {
				bool ahead = cab.direction == UP ? calls[j].floor >= pos : calls[j].floor <= pos;
				compatible = calls[j].direction == cab.direction && ahead;
			}
			if (compatible && d < dCompatible) { dCompatible = d; bestCompatible = static_cast<int>(c) + 1; }
		}
		seed[j] = bestCompatible != 0 ? bestCompatible : bestAny;
	}
	return seed;
}

std::vector<int> BeamurgiaDispatchStrategy::allocate(const std::vector<HallCall>& calls,
	const std::vector<Elevator>& elevators, IRandomProvider& engineRng)
{
	IRandomProvider& rng = params.random ? *params.random : engineRng;
	++consulted;
	const int N = static_cast<int>(elevators.size());
	const size_t M = calls.size();

	std::vector<int> current(M);
	std::vector<char> fixed(M, 0);
	std::vector<int> freeGenes;
	bool allAllocated = true;
	for (size_t j = 0; j < M; ++j) {
		current[j] = calls[j].currentElevatorId;
		if (current[j] < 1) allAllocated = false;
		if (calls[j].committed) fixed[j] = 1; else freeGenes.push_back(static_cast<int>(j));
	}
	if (M == 0 || N == 0) return current;

	// D7: same calls as the last run, each with a car -> keep them where they are.
	std::vector<CallKey> keys;
	for (const HallCall& c : calls) keys.push_back(CallKey(c.floor, c.direction, c.registeredAt));
	if (params.reoptimizeOnlyOnChange && allAllocated && keys == lastOptimizedSet) return current;
	lastOptimizedSet = keys;
	++optimized;
	if (freeGenes.empty()) return current;

	const Context ctx = contextOf(calls, elevators);
	const int P = std::max(2, params.populationSize);

	// Initial population, built in index order so the draws come in a fixed
	// order: Sd's individual first, LBI's next, the rest random over the free
	// genes. A fixed gene keeps its current car everywhere.
	std::vector<std::vector<int> > pop;
	if (params.seeding) pop.push_back(seededIndividual(calls, elevators, ctx));
	if (params.lastBestIndividual && static_cast<int>(pop.size()) < P) {
		std::vector<int> lbi = current;
		for (int g : freeGenes) {
			std::map<CallKey, int>::const_iterator it = lastBest.find(keys[g]);
			lbi[g] = (it != lastBest.end() && it->second >= 1 && it->second <= N) ? it->second : rng.nextUniformInt(1, N);
		}
		pop.push_back(lbi);
	}
	while (static_cast<int>(pop.size()) < P) {
		std::vector<int> ind = current;
		for (int g : freeGenes) ind[g] = rng.nextUniformInt(1, N);
		pop.push_back(ind);
	}
	std::vector<double> ewtp(P);
	for (int i = 0; i < P; ++i) ewtp[i] = evaluate(pop[i], calls, ctx);

	auto bestIndex = [&](const std::vector<double>& v) {
		return static_cast<int>(std::min_element(v.begin(), v.end()) - v.begin());	// first of equals
	};

	for (int it = 0; it < params.iterations; ++it) {
		// Roulette over fitness = 1 / EWTP^2 (eqs. 4, 5). A zero EWTP (nobody
		// waits at all) is floored so its slot is huge but finite.
		std::vector<double> fitness(P);
		double total = 0.0;
		for (int i = 0; i < P; ++i) {
			const double e = std::max(ewtp[i], 1e-9);
			fitness[i] = 1.0 / (e * e);
			total += fitness[i];
		}
		auto spin = [&]() {
			const double r = uniform01(rng) * total;
			double cum = 0.0;
			for (int i = 0; i < P; ++i) {
				cum += fitness[i];
				if (r < cum) return i;
			}
			return P - 1;
		};

		// Elitism: the previous population's winner goes into the new one.
		const int elite = bestIndex(ewtp);
		std::vector<std::vector<int> > next{pop[elite]};
		std::vector<double> nextEwtp{ewtp[elite]};

		while (static_cast<int>(next.size()) < P) {
			// "From two different selected individuals": re-spin the second
			// until it is another slot (bounded, like GAHCA's incest guard).
			const int a = spin();
			int b = spin();
			for (int t = 0; t < 8 && b == a; ++t) b = spin();

			std::vector<int> c1 = pop[a], c2 = pop[b];
			if (uniform01(rng) < params.crossoverProbability) {
				// D4: common genes inherited; every other gene taken from
				// either parent, independently for each child (Fig. 2).
				for (size_t j = 0; j < M; ++j) {
					if (fixed[j] || pop[a][j] == pop[b][j]) { c1[j] = c2[j] = pop[a][j]; continue; }
					c1[j] = rng.nextUniformInt(0, 1) ? pop[a][j] : pop[b][j];
					c2[j] = rng.nextUniformInt(0, 1) ? pop[a][j] : pop[b][j];
				}
			}
			for (std::vector<int>* child : {&c1, &c2}) {
				if (static_cast<int>(next.size()) >= P) break;
				if (N > 1 && uniform01(rng) < params.mutationProbability) {
					// Fig. 3: one call, chosen at random, given to a different car.
					const int g = freeGenes[rng.nextUniformInt(0, static_cast<int>(freeGenes.size()) - 1)];
					int car = rng.nextUniformInt(1, N - 1);
					if (car >= (*child)[g]) ++car;
					(*child)[g] = car;
				}
				next.push_back(*child);
				nextEwtp.push_back(evaluate(*child, calls, ctx));
			}
		}
		pop.swap(next);
		ewtp.swap(nextEwtp);
	}

	std::vector<int> best = pop[bestIndex(ewtp)];
	for (size_t j = 0; j < M; ++j) if (fixed[j]) best[j] = calls[j].currentElevatorId;
	for (size_t j = 0; j < M; ++j) lastBest[keys[j]] = best[j];
	return best;
}

} // namespace examples
} // namespace openEGCS
