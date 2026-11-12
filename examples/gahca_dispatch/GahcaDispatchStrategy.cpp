/*
GahcaDispatchStrategy.cpp - see GahcaDispatchStrategy.h. Cortes et al. (2004).
*/

#include "GahcaDispatchStrategy.h"
#include "GahcaFitness.h"
#include "Task.h"	// UP, DOWN

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace openEGCS {
namespace examples {

GahcaDispatchStrategy::GahcaDispatchStrategy() : params() {}
GahcaDispatchStrategy::GahcaDispatchStrategy(Params parameters) : params(parameters) {}

double GahcaDispatchStrategy::objective(const std::vector<int>& assignment,
	const std::vector<HallCall>& calls, const std::vector<Elevator>& elevators) const
{
	const int N = static_cast<int>(elevators.size());
	if (N == 0) return 0.0;

	// The up/down boarding floors each car would serve under this assignment.
	std::vector<std::vector<int> > up(N), down(N);
	for (std::size_t j = 0; j < calls.size(); ++j) {
		const int car = assignment[j];
		if (car < 1 || car > N) continue;	// an out-of-range gene scores as unserved; the engine veto handles feasibility on apply
		if (calls[j].direction == UP) up[car - 1].push_back(calls[j].floor);
		else down[car - 1].push_back(calls[j].floor);
	}

	double sum = 0.0;
	double hi = -std::numeric_limits<double>::infinity();
	double lo = std::numeric_limits<double>::infinity();
	for (int c = 0; c < N; ++c) {
		// P1 = the cabin's current floor. getPosition() is the public accessor
		// (stoppingPosition(), the braking-aware floor, is private to
		// Elevator); rounded to the nearest floor for the integer peak math.
		const int floor = static_cast<int>(std::lround(elevators[c].getPosition()));
		const double ct = gahcaCabinServiceTime(floor, elevators[c].getDirection(),
			up[c], down[c], params.interFloorTime, params.perStopTime);
		sum += ct;
		if (ct > hi) hi = ct;
		if (ct < lo) lo = ct;
	}
	return params.k1 * sum + params.k2 * (hi - lo);
}

namespace {

// A uniform number in [0,1) drawn from the injected provider, which only offers
// integer/exponential/normal draws - so determinism stays tied to one source.
double uniform01(IRandomProvider& rng) {
	return rng.nextUniformInt(0, 1000000) / 1000001.0;
}

} // namespace

std::vector<int> GahcaDispatchStrategy::allocate(const std::vector<HallCall>& calls,
	const std::vector<Elevator>& elevators, IRandomProvider& rng)
{
	const int N = static_cast<int>(elevators.size());
	const std::size_t M = calls.size();

	// The current allocation, which genes are fixed (committed calls), and the
	// free ones. A pool call (currentElevatorId 0) has no car yet; seed it to
	// car 1, a valid placeholder the GA is free to move.
	std::vector<int> current(M);
	std::vector<char> fixed(M, 0);
	std::vector<int> freeGenes;
	for (std::size_t j = 0; j < M; ++j) {
		current[j] = calls[j].currentElevatorId >= 1 ? calls[j].currentElevatorId : 1;
		if (calls[j].committed) fixed[j] = 1; else freeGenes.push_back(static_cast<int>(j));
	}
	// Nothing to search: no cars, no calls, or every call is pinned.
	if (N == 0 || M == 0 || freeGenes.empty()) return current;

	const int P = params.populationSize > 1 ? params.populationSize : 2;

	// Generate the population: individual 0 is the current allocation (the paper
	// re-optimises from the current state each t), the rest random over the free
	// genes. Then calculate the fitness population.
	std::vector<std::vector<int> > pop(P, current);
	for (int i = 1; i < P; ++i)
		for (int g : freeGenes) pop[i][g] = rng.nextUniformInt(1, N);
	std::vector<double> fit(P);
	for (int i = 0; i < P; ++i) fit[i] = objective(pop[i], calls, elevators);

	// The best individual seen anywhere in the run - the elitist safeguard over
	// the literal "best of the final population", which makes the result never
	// worse than the current allocation (pop[0]).
	std::vector<int> best = pop[0];
	double bestFit = fit[0];
	for (int i = 1; i < P; ++i) if (fit[i] < bestFit) { bestFit = fit[i]; best = pop[i]; }

	// Steady-state loop (Appendix B): keep making one offspring until
	// `generations` of them have been accepted. A `guard` caps pathological
	// cases where the search space is too small to find a non-duplicate.
	int accepted = 0;
	long guard = 0;
	const long guardLimit = static_cast<long>(params.generations) * 200 + 1000;
	while (accepted < params.generations && guard++ < guardLimit) {
		std::vector<int> child;
		if (uniform01(rng) < params.crossoverPercent / 100.0) {
			// Crossover: two parents, re-picked until they are not identical
			// (incest prevention), then uniform crossover into one offspring. A
			// gene equal in both parents is inherited; a differing one comes from
			// either with equal chance. Fixed genes are equal in both (both are
			// `current`), so they pass through.
			int a = rng.nextUniformInt(0, P - 1);
			int b = rng.nextUniformInt(0, P - 1);
			for (int t = 0; t < 8 && (b == a || pop[a] == pop[b]); ++t) b = rng.nextUniformInt(0, P - 1);
			child.resize(M);
			for (std::size_t j = 0; j < M; ++j) {
				if (fixed[j] || pop[a][j] == pop[b][j]) child[j] = pop[a][j];
				else child[j] = rng.nextUniformInt(0, 1) ? pop[a][j] : pop[b][j];
			}
		} else {
			// Mutation: one parent, reassign one random free call to a random car
			// (the paper's "replaces a hall call allocation").
			child = pop[rng.nextUniformInt(0, P - 1)];
			const int g = freeGenes[rng.nextUniformInt(0, static_cast<int>(freeGenes.size()) - 1)];
			child[g] = rng.nextUniformInt(1, N);
		}

		// Duplicity control: a duplicate offspring is discarded and NOT counted.
		bool duplicate = false;
		for (int k = 0; k < P; ++k) if (pop[k] == child) { duplicate = true; break; }
		if (duplicate) continue;

		const double childFit = objective(child, calls, elevators);

		// Select one individual for replacement by the hypergeometric rule
		// (section 3.4): rank worst (highest objective) first, the p-th worst is
		// chosen with probability ~q(1-q)^p. Worst individuals are the likeliest
		// to be replaced; the best is almost never touched.
		std::vector<int> rank(P);
		std::iota(rank.begin(), rank.end(), 0);
		std::sort(rank.begin(), rank.end(), [&](int x, int y) {
			return fit[x] != fit[y] ? fit[x] > fit[y] : x < y;	// worst first, stable by index
		});
		const double u = uniform01(rng);
		int victim = rank[P - 1];	// fallback (u beyond the cumulative): the best, rare
		double cum = 0.0;
		for (int p = 0; p < P; ++p) {
			cum += params.replacementQ * std::pow(1.0 - params.replacementQ, p);
			if (u < cum) { victim = rank[p]; break; }
		}

		pop[victim] = child;
		fit[victim] = childFit;
		if (childFit < bestFit) { bestFit = childFit; best = child; }
		++accepted;
	}

	// Committed genes are returned as their current car no matter what (defence
	// in depth: they were never free, but allocate()'s contract is explicit).
	for (std::size_t j = 0; j < M; ++j) if (fixed[j]) best[j] = calls[j].currentElevatorId;
	return best;
}

} // namespace examples
} // namespace openEGCS
