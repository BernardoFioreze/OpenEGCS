#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include "BeamurgiaDispatchStrategy.h"
#include "Elevator.h"
#include "GreedyNearestDispatchStrategy.h"
#include "ISimulationLogger.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "Task.h"	// UP, DOWN, CALL

using namespace openEGCS;
using namespace openEGCS::examples;

// The modified GA of Beamurgia et al. 2016 as an IGroupDispatchStrategy.
// Directly-built cabins run the linear model at 1 s/floor with no dwell, so
// the EWT of a hand-built state is countable.

namespace {

std::vector<Elevator> idleCarsAt(const std::vector<double>& floors)
{
	std::vector<Elevator> cars;
	for (size_t k = 0; k < floors.size(); ++k) cars.push_back(Elevator(k + 1, "", floors[k], 1));
	return cars;
}

// {floor, direction, registeredAt, currentElevatorId, committed}
HallCall call(int floor, unsigned int dir, int car, bool committed = false, double registeredAt = 0.0)
{
	return HallCall{floor, dir, registeredAt, car, committed};
}

// Every assignment of `calls` to `cars` cars, the exact optimum of section 6.1.
double exhaustiveOptimum(const BeamurgiaDispatchStrategy& s, const std::vector<HallCall>& calls,
	const std::vector<Elevator>& cars)
{
	const int N = static_cast<int>(cars.size());
	std::vector<int> a(calls.size(), 1);
	double best = std::numeric_limits<double>::infinity();
	for (;;) {
		best = std::min(best, s.objective(a, calls, cars));
		size_t k = 0;
		while (k < a.size() && a[k] == N) a[k++] = 1;
		if (k == a.size()) break;
		++a[k];
	}
	return best;
}

// Section 6.1's static instance: 20 floors, 4 lifts, 5 landing calls - 2 up,
// 3 down - so 4^5 = 1024 assignments. The paper does not give the floors; these
// are spread so that no single car is the obvious answer for everybody.
std::vector<HallCall> staticCalls()
{
	return { call(3, UP, 1), call(9, UP, 1), call(15, DOWN, 1), call(10, DOWN, 1), call(5, DOWN, 1) };
}
std::vector<Elevator> staticCars() { return idleCarsAt({0, 6, 12, 19}); }

struct SignatureLogger : public ISimulationLogger {
	std::ostringstream sig;
	int events = 0;
	void setMetadata(const SimulationMetadata&) override {}
	void logCustomer(const Customer&) override {}
	void logEvent(double time, const std::string& type, int elevatorId,
		unsigned long int customerId, int floor) override {
		sig << time << '|' << type << '|' << elevatorId << '|' << customerId << '|' << floor << '\n';
		++events;
	}
	void finalize(const IMetricsCollector&, const std::string&) override {}
};

BeamurgiaDispatchStrategy::Params everyAdjustment(int highestFloor)
{
	BeamurgiaDispatchStrategy::Params p;
	p.seeding = true;
	p.lastBestIndividual = true;
	p.penalty = BeamurgiaDispatchStrategy::Penalty::Dynamic;
	p.estimator.reversal = BeamurgiaReversal::ExpectedExtremeFloor;
	p.estimator.lowestFloor = 0;
	p.estimator.highestFloor = highestFloor;
	return p;
}

std::string fullRunSignature(uint64_t seed, int& events, long& consultations, long& optimizations)
{
	const int floors = 15;	// floors 0..15: the traffic profile draws them inclusive
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed));
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	auto logger = std::make_shared<SignatureLogger>();
	auto beamurgia = std::make_shared<BeamurgiaDispatchStrategy>(everyAdjustment(floors));
	Simulator sim(1, "beamurgia-determinism", floors, /*elevators=*/3, /*customers=*/120,
		/*rate=*/1.5, 1.0f, true, rng, dispatch, nullptr, logger, std::nullopt,
		/*maxWeight=*/800.0, nullptr, 70.0, 15.0,
		nullptr, nullptr, nullptr, beamurgia);
	sim.startSimulation();
	events = logger->events;
	consultations = beamurgia->consultations();
	optimizations = beamurgia->optimizations();
	return logger->sig.str();
}

} // namespace

TEST_CASE("Beamurgia GA: deterministic for a given seed and state", "[beamurgia]") {
	std::vector<HallCall> calls = staticCalls();
	std::vector<Elevator> cars = staticCars();
	BeamurgiaDispatchStrategy a(everyAdjustment(19)), b(everyAdjustment(19));
	RandomProvider rngA{std::optional<uint64_t>(42)}, rngB{std::optional<uint64_t>(42)};
	std::vector<int> ra = a.allocate(calls, cars, rngA);
	std::vector<int> rb = b.allocate(calls, cars, rngB);
	REQUIRE(ra == rb);
	REQUIRE(ra.size() == calls.size());
	for (int car : ra) { REQUIRE(car >= 1); REQUIRE(car <= 4); }
}

TEST_CASE("Beamurgia GA: section 6.1 - large GAs reach the exact optimum, and more search gets closer", "[beamurgia]") {
	// The paper's static study compares the raw GA with the optimum found by
	// enumerating all 1024 assignments, and finds the gap shrinks as the
	// population and the iterations grow (Fig. 5).
	std::vector<HallCall> calls = staticCalls();
	std::vector<Elevator> cars = staticCars();
	BeamurgiaDispatchStrategy probe;
	const double optimum = exhaustiveOptimum(probe, calls, cars);

	auto meanGap = [&](int size, int iterations) {
		BeamurgiaDispatchStrategy::Params p;
		p.populationSize = size;
		p.iterations = iterations;
		p.reoptimizeOnlyOnChange = false;
		double gap = 0.0;
		for (uint64_t seed = 1; seed <= 20; ++seed) {
			BeamurgiaDispatchStrategy ga(p);
			RandomProvider rng{std::optional<uint64_t>(seed)};
			const double got = ga.objective(ga.allocate(calls, cars, rng), calls, cars);
			REQUIRE(got >= optimum - 1e-9);	// nothing beats the enumeration
			gap += got - optimum;
		}
		return gap / 20.0;
	};

	const double small = meanGap(5, 5);
	const double paper = meanGap(10, 10);
	const double large = meanGap(30, 30);
	REQUIRE(large <= paper + 1e-9);
	REQUIRE(paper <= small + 1e-9);
	REQUIRE(large < small);
	REQUIRE(large == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("Beamurgia GA: a committed call keeps its car", "[beamurgia]") {
	std::vector<HallCall> calls = { call(5, UP, 1), call(7, DOWN, 2, /*committed=*/true), call(3, UP, 0) };
	std::vector<Elevator> cars = idleCarsAt({0, 10});
	BeamurgiaDispatchStrategy ga;
	RandomProvider rng{std::optional<uint64_t>(99)};
	REQUIRE(ga.allocate(calls, cars, rng)[1] == 2);
}

TEST_CASE("Beamurgia GA: the penalties of eqs. 8 and 9", "[beamurgia]") {
	// One car at 0, one call 40 floors up: EWT 40 s (1 s/floor, no dwell).
	std::vector<Elevator> car = idleCarsAt({0});
	std::vector<HallCall> fresh = { call(40, UP, 1) };
	BeamurgiaDispatchStrategy::Params p;

	p.penalty = BeamurgiaDispatchStrategy::Penalty::None;
	REQUIRE(BeamurgiaDispatchStrategy(p).objective({1}, fresh, car) == Catch::Approx(40.0));

	// P: 1000 x I, I = 1 car, because 40 > 30.
	p.penalty = BeamurgiaDispatchStrategy::Penalty::Death;
	REQUIRE(BeamurgiaDispatchStrategy(p).objective({1}, fresh, car) == Catch::Approx(1040.0));

	// P3: nobody has waited yet, so the reference is max(RWT = 0, 30) = 30:
	// 40 + (40 - 30)^2.
	p.penalty = BeamurgiaDispatchStrategy::Penalty::Dynamic;
	REQUIRE(BeamurgiaDispatchStrategy(p).objective({1}, fresh, car) == Catch::Approx(140.0));

	// Pressed 50 s ago: EWT = 50 + 40 = 90 and RWT = 50 > 30, so the
	// reference is 50: 90 + 40^2.
	std::vector<HallCall> old = { call(40, UP, 1, false, /*registeredAt=*/-50.0) };
	REQUIRE(BeamurgiaDispatchStrategy(p).objective({1}, old, car) == Catch::Approx(1690.0));

	// Under 30 s nobody is penalised.
	std::vector<HallCall> near = { call(20, UP, 1) };
	p.penalty = BeamurgiaDispatchStrategy::Penalty::Death;
	REQUIRE(BeamurgiaDispatchStrategy(p).objective({1}, near, car) == Catch::Approx(20.0));
}

TEST_CASE("Beamurgia GA: Sd seeds the nearest car that can take the call", "[beamurgia]") {
	// No iterations and a population of two: what comes back is the better of
	// the seed and one random individual. The seed is the right answer here.
	BeamurgiaDispatchStrategy::Params p;
	p.seeding = true;
	p.populationSize = 2;
	p.iterations = 0;
	std::vector<Elevator> cars = idleCarsAt({0, 10});
	std::vector<HallCall> calls = { call(9, UP, 1), call(1, DOWN, 2), call(8, DOWN, 1) };
	for (uint64_t seed = 1; seed <= 10; ++seed) {
		BeamurgiaDispatchStrategy ga(p);
		RandomProvider rng{std::optional<uint64_t>(seed)};
		REQUIRE(ga.allocate(calls, cars, rng) == std::vector<int>{2, 1, 2});
	}
}

TEST_CASE("Beamurgia GA: elitism never loses the best individual between generations", "[beamurgia]") {
	// "The fitness of the best individual of the new population will always be
	// the same as or better than that in the old population" (section 5.1).
	// Seeded with Sd's individual, nothing the generations do may return
	// something worse than it. On the static instance it is, by hand, the
	// nearest car per call with ties to the lower id: {1, 2, 3, 3, 2}.
	std::vector<HallCall> calls = staticCalls();
	std::vector<Elevator> cars = staticCars();
	BeamurgiaDispatchStrategy::Params p;
	p.seeding = true;
	p.populationSize = 4;
	p.iterations = 10;
	p.reoptimizeOnlyOnChange = false;
	const double seedValue = BeamurgiaDispatchStrategy(p).objective({1, 2, 3, 3, 2}, calls, cars);
	for (uint64_t seed = 1; seed <= 30; ++seed) {
		BeamurgiaDispatchStrategy ga(p);
		RandomProvider rng{std::optional<uint64_t>(seed)};
		REQUIRE(ga.objective(ga.allocate(calls, cars, rng), calls, cars) <= seedValue + 1e-9);
	}
}

TEST_CASE("Beamurgia GA: LBI carries the last best individual into the next consultation", "[beamurgia]") {
	// With no iterations, each consultation returns the better of the LBI
	// individual and one random one, so with LBI the objective can never go up
	// from one consultation to the next on the same calls. Without it, every
	// consultation starts from scratch.
	std::vector<Elevator> cars = idleCarsAt({0, 4, 8, 12, 16, 19});
	std::vector<HallCall> calls = { call(2, UP, 1), call(6, DOWN, 1), call(9, UP, 1), call(11, DOWN, 1),
		call(13, UP, 1), call(14, DOWN, 1), call(17, DOWN, 1), call(18, UP, 1) };
	BeamurgiaDispatchStrategy::Params p;
	p.lastBestIndividual = true;
	p.populationSize = 2;
	p.iterations = 0;
	p.reoptimizeOnlyOnChange = false;
	BeamurgiaDispatchStrategy ga(p);
	RandomProvider rng{std::optional<uint64_t>(7)};
	double previous = std::numeric_limits<double>::infinity();
	for (int k = 0; k < 15; ++k) {
		std::vector<int> got = ga.allocate(calls, cars, rng);
		const double value = ga.objective(got, calls, cars);
		REQUIRE(value <= previous + 1e-9);
		previous = value;
	}
}

TEST_CASE("Beamurgia GA: D7 - the GA runs only when the calls change, never leaving a call without a car", "[beamurgia]") {
	std::vector<Elevator> cars = idleCarsAt({0, 10});
	BeamurgiaDispatchStrategy ga;
	RandomProvider rng{std::optional<uint64_t>(3)};

	std::vector<HallCall> allocated = { call(5, UP, 1), call(8, DOWN, 2) };
	ga.allocate(allocated, cars, rng);
	std::vector<int> again = ga.allocate(allocated, cars, rng);
	REQUIRE(ga.consultations() == 2);
	REQUIRE(ga.optimizations() == 1);
	REQUIRE(again == std::vector<int>{1, 2});	// skipped: each call stays where it is

	// A call without a car is never skipped, or it would be handed back
	// unallocated at every consultation.
	std::vector<HallCall> pooled = { call(3, UP, 0) };
	std::vector<int> first = ga.allocate(pooled, cars, rng);
	std::vector<int> second = ga.allocate(pooled, cars, rng);
	REQUIRE(ga.optimizations() == 3);
	REQUIRE(first[0] >= 1);
	REQUIRE(second[0] >= 1);
}

TEST_CASE("Beamurgia GA: a provider of its own keeps the GA out of the engine's stream", "[beamurgia]") {
	// With `random` set, what the engine hands allocate() is not drawn from:
	// two very different engine streams give the same allocation, and the
	// engine's stream is left exactly where it was.
	std::vector<HallCall> calls = staticCalls();
	std::vector<Elevator> cars = staticCars();
	auto allocateWith = [&](uint64_t engineSeed) {
		BeamurgiaDispatchStrategy::Params p;
		p.random = std::make_shared<RandomProvider>(std::optional<uint64_t>(5));
		BeamurgiaDispatchStrategy ga(p);
		RandomProvider engine{std::optional<uint64_t>(engineSeed)};
		std::vector<int> got = ga.allocate(calls, cars, engine);
		RandomProvider untouched{std::optional<uint64_t>(engineSeed)};
		REQUIRE(engine.nextUniformInt(0, 1000000) == untouched.nextUniformInt(0, 1000000));
		return got;
	};
	REQUIRE(allocateWith(1) == allocateWith(999));
}

TEST_CASE("Beamurgia GA: what the estimator is told about a cabin", "[beamurgia]") {
	// Parked: standing at its floor with no direction, room for 700/70 = 10.
	Elevator parked(1, "", 3, 1, nullptr, 700.0);
	BeamurgiaCabin s = BeamurgiaDispatchStrategy::cabinStateOf(parked, 0.0, 70.0);
	REQUIRE_FALSE(s.moving);
	REQUIRE(s.floor == 3);
	REQUIRE(s.direction != UP);
	REQUIRE(s.direction != DOWN);
	REQUIRE(s.capacity == 10);
	REQUIRE(s.carCalls.empty());

	// Given a call at 8: on its way there, a fixed first stop 5 s away.
	Elevator going(1, "", 3, 1);
	Task request(1, "", CALL, UP, 8, 0.0);
	going.addTask(request, 1);
	s = BeamurgiaDispatchStrategy::cabinStateOf(going, 0.0, 70.0);
	REQUIRE(s.moving);
	REQUIRE(s.direction == UP);
	REQUIRE(s.fixedDestination == 8);
	REQUIRE(s.timeToFixedDestination == Catch::Approx(5.0));
	REQUIRE(s.capacity == std::numeric_limits<int>::max());	// unlimited rated load
}

TEST_CASE("Beamurgia GA: a committed call at the fixed stop shapes the route of the others", "[beamurgia]") {
	// The cabin is on its way up to 5 for an up call that is committed - not a
	// gene, so not among `calls` - and the GA weighs giving it the down call
	// at 3. The up passenger it picks up at 5 is still aboard in the estimate,
	// so under R1' it carries on to where it is expected to reverse
	// (uniform over 6..10 -> 8, one expected stop) before coming back:
	//   5 s to 5, 3 s on to 8, 5 s back to 3 -> EWT 13.
	// Without that passenger in the estimate the cabin would turn at 5: 7.
	Elevator going(1, "", 0, 1);
	Task committed(1, "", CALL, UP, 5, 0.0);
	going.addTask(committed, 1);
	std::vector<Elevator> cars{going};
	std::vector<HallCall> calls = { call(3, DOWN, 1) };

	BeamurgiaDispatchStrategy::Params p;
	p.estimator.reversal = BeamurgiaReversal::ExpectedExtremeFloor;
	p.estimator.lowestFloor = 0;
	p.estimator.highestFloor = 10;
	REQUIRE(BeamurgiaDispatchStrategy(p).objective({1}, calls, cars) == Catch::Approx(13.0));
}

TEST_CASE("Beamurgia GA: drives a byte-deterministic full run with every adjustment", "[beamurgia]") {
	int eventsA = 0, eventsB = 0;
	long consultedA = 0, optimizedA = 0, consultedB = 0, optimizedB = 0;
	std::string a = fullRunSignature(2024, eventsA, consultedA, optimizedA);
	std::string b = fullRunSignature(2024, eventsB, consultedB, optimizedB);
	REQUIRE(eventsA > 0);
	REQUIRE(a == b);
	REQUIRE(optimizedA == optimizedB);
	// D7 does skip consultations in a real run, and the GA does run.
	REQUIRE(optimizedA > 0);
	REQUIRE(optimizedA < consultedA);
}
