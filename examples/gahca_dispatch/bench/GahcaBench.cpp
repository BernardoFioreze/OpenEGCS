/*
GahcaBench.cpp - the measurement driver for examples::GahcaDispatchStrategy:
runs the saturated lunch-peak of Cortes et al. (2004) under one controller for
a range of seeds, and writes one snapshot per run for analyze.py (next to this
file) to read. A measurement tool, not part of either library - the counterpart
of examples/beamurgia_dispatch/bench/.

    gahca_bench <outDir> <controller> <firstSeed> <lastSeed>
    python3 -I analyze.py <outDir>

The scenario (also the regime of test_capacity_enforcement's "A saturated GAHCA
lunch-peak with finite cabins serves everyone", at scale 3 and 300 customers
instead of 5 and 60):
  12 floors, 2 cars;
  20 people per car: maxWeight 20, every passenger weighs exactly 1;
  LinearTravelTimeStrategy(5.0)   - Cortes section 3.2's 5 s per floor;
  FixedDwellTimeStrategy(6.5, 6.5, 0.0) - ~13 s per stop, section 3.2's
                                    2 decel + 2 door + 5 transfer + 2 door + 2 accel,
                                    per stop and not per passenger;
  LunchPeakTrafficProfile(12, scale 3.0) - Fig. 5 digitised, scaled up because
                                    the published figure is too light to
                                    saturate two cars (see that profile's
                                    fidelity caveat);
  300 customers.
Every collaborator the scenario names is injected explicitly, so the engine's
own defaults (kinematic travel, the handbook door and its hold) do not reach
this run: the physics are the paper's.

Controllers:
  thv        examples::ThvDispatchStrategy, per call, behind the default adapter
             (the paper's baseline, its Appendix A);
  gahca      examples::GahcaDispatchStrategy as the group strategy, drawing from
             the engine's one provider, so its GA draws interleave with the
             arrival draws and the passengers it sees diverge from THV's after
             its first optimisation;
  gahca-crn  the same GA drawing from a provider of its own, seeded from the
             run's seed, so it sees the same passengers as thv and greedy
             (common random numbers, as beamurgia_bench does);
  greedy     examples::GreedyNearestDispatchStrategy, the project's per-call
             baseline, a column the paper does not have.
*/

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "FixedDwellTimeStrategy.h"
#include "GahcaDispatchStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "LinearTravelTimeStrategy.h"
#include "LunchPeakTrafficProfile.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "StandardMetricsCollector.h"
#include "ThvDispatchStrategy.h"

using namespace openEGCS;
using namespace openEGCS::examples;

namespace {

const int kFloors = 12;
const int kCars = 2;
const int kCustomers = 300;
const double kScale = 3.0;
const double kCapacityPeople = 20.0;

// Hands the wrapped group strategy a provider of its own instead of the
// engine's, so the GA's draws do not move the arrival stream. Bench-only: the
// engine still passes its provider, this just does not forward it.
class OwnStreamGroupStrategy : public IGroupDispatchStrategy {
	public:
		OwnStreamGroupStrategy(std::shared_ptr<IGroupDispatchStrategy> inner, std::shared_ptr<IRandomProvider> rng)
			: inner(std::move(inner)), rng(std::move(rng)) {}
		std::vector<int> allocate(const std::vector<HallCall>& calls, const std::vector<Elevator>& elevators,
			IRandomProvider&) override
		{
			return inner->allocate(calls, elevators, *rng);
		}
	private:
		std::shared_ptr<IGroupDispatchStrategy> inner;
		std::shared_ptr<IRandomProvider> rng;
};

} // namespace

int main(int argc, char** argv)
{
	if (argc != 5) {
		std::cerr << "usage: gahca_bench <outDir> <thv|gahca|gahca-crn|greedy> <firstSeed> <lastSeed>\n";
		return 2;
	}
	const std::string outDir = argv[1], controller = argv[2];
	const uint64_t first = std::stoull(argv[3]), last = std::stoull(argv[4]);
	if (controller != "thv" && controller != "gahca" && controller != "gahca-crn" && controller != "greedy") {
		std::cerr << "unknown controller " << controller << "\n";
		return 2;
	}

	std::ofstream runs(outDir + "/runs.tsv", std::ios::app);
	for (uint64_t seed = first; seed <= last; ++seed) {
		auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed));
		auto metrics = std::make_shared<StandardMetricsCollector>();
		std::shared_ptr<IDispatchStrategy> dispatch;
		if (controller == "thv") dispatch = std::make_shared<ThvDispatchStrategy>(kFloors);
		else dispatch = std::make_shared<GreedyNearestDispatchStrategy>();	// unused under a group strategy
		std::shared_ptr<IGroupDispatchStrategy> group;
		if (controller == "gahca") group = std::make_shared<GahcaDispatchStrategy>();
		else if (controller == "gahca-crn")
			group = std::make_shared<OwnStreamGroupStrategy>(std::make_shared<GahcaDispatchStrategy>(),
				std::make_shared<RandomProvider>(std::optional<uint64_t>(seed * 1000003ULL + 17ULL)));
		auto dwell = std::make_shared<FixedDwellTimeStrategy>(6.5, 6.5, 0.0);
		auto travel = std::make_shared<LinearTravelTimeStrategy>(5.0);
		auto profile = std::make_shared<LunchPeakTrafficProfile>(kFloors, kScale);

		const std::string tag = controller + "__s" + std::to_string(seed);
		std::streambuf* saved = std::cout.rdbuf();
		std::ostringstream sink;
		std::cout.rdbuf(sink.rdbuf());	// the simulator's own console report
		auto t0 = std::chrono::steady_clock::now();
		Simulator sim(seed, tag, kFloors, kCars, kCustomers, /*arrivalRate=*/0.5, /*speed=*/1.0f, /*sane=*/true,
			rng, dispatch, metrics, /*logger=*/nullptr, std::optional<uint64_t>(seed),
			kCapacityPeople, /*capacity=*/nullptr, /*mean_weight=*/1.0, /*stddev_weight=*/0.0,
			dwell, travel, /*passenger=*/nullptr, group, profile);
		sim.startSimulation();
		const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		sim.writeSnapshot(outDir + "/" + tag + ".json");
		std::cout.rdbuf(saved);

		runs << controller << '\t' << seed << '\t' << wall << '\t' << metrics->getAverageWaitingTime() << '\t'
			<< metrics->getJourneyTimeDistribution().average << '\n';
		runs.flush();
		std::cerr << tag << "  wall " << wall << " s  wait " << metrics->getAverageWaitingTime()
			<< "  journey " << metrics->getJourneyTimeDistribution().average << "\n";
	}
	return 0;
}
