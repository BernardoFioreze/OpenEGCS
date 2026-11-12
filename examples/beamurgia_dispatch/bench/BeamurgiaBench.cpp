/*
BeamurgiaBench.cpp - the measurement driver for examples::BeamurgiaDispatchStrategy:
runs one scenario of Beamurgia et al. (2016) under one variant of the GA for a
range of seeds, and writes one snapshot per run for analyze.py (next to this
file) to read. A measurement tool, not part of either library.

    beamurgia_bench <outDir> <scenario> <variant> <R1|R1p> <firstSeed> <lastSeed>
    python3 -I analyze.py <outDir>

<R1|R1p> picks where the estimator ends a sweep (BeamurgiaReversal in
BeamurgiaEstimator.h): R1 = LastStopAhead, R1p = ExpectedExtremeFloor.

Scenarios (Table 1 of the paper). The paper only says the physics are "the
default ones in Elevate", so the installation's physics below are substitutions,
each from the handbook:
  C1-out, C1-in, C1-inter   6 cars, floors 0..11, 1600 kg, 80 people per
                            populated floor, 400 passengers; one pure profile
  C2-inter                  6 cars, floors 0..32, 850 kg, 30 people per
                            populated floor, 350 passengers; interfloor
  C2-inter-hc10             C2-inter with the 2.5 m/s row of Table 5.2 instead
                            of the 6.0 m/s one its travel selects: the row
                            whose up-peak handling capacity (eq. 4.4, the
                            project's TrafficCapacity) is the 10% of the
                            population per 5 minutes the paper states for its
                            installation (section 6.5). The travel row gives
                            13.8%, so the paper's 11-13% demand does not
                            saturate it; this one does. A published property
                            of the scenario, not a result, is what is matched.
Both: floor height 3.3 m; rated speed/acceleration/jerk by travel from
Barney & Al-Sharif Table 5.2 (defaultDynamicsFor); centre-opening doors with
normal opening from Table 5.3 (1100 mm for 1600 kg: 2.5 s open, 3.0 s close;
800 mm for 850 kg: 2.0 / 2.0); passenger transfer from Table 5.7 (1600 kg:
0.8 s; 850 kg: 1.2 s, the 320-800 kg row, the nearest); demand from
StepMixTrafficProfile (11% -> 13% of the population per 5 minutes).

Variants: S, S+P, S+P3, S+LBI+Sd, S+LBI+Sd+P, S+LBI+Sd+P3 (the paper's six C2
variants; every one has S because the engine imposes it), and greedy - the
project's per-call baseline behind the default adapter, a column the paper
does not have.

The GA draws from a provider of its own, seeded from the run's seed, so every
variant run with a given seed sees the same passengers (common random
numbers): the arrivals are drawn from the engine's provider, which nothing
else touches.
*/

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "BeamurgiaDispatchStrategy.h"
#include "FixedDwellTimeStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "KinematicTravelTimeStrategy.h"
#include "LiftKinematics.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "StandardMetricsCollector.h"
#include "StepMixTrafficProfile.h"

using namespace openEGCS;
using namespace openEGCS::examples;

namespace {

struct Scenario {
	int floors;					// floors run 0..floors
	int cars;
	double maxWeightKg;
	double populationPerFloor;
	int passengers;
	double incoming, outgoing, interfloor;
	double doorOpen, doorClose, transferPerPassenger;
	bool dynamicsOverride = false;
	LiftDynamics dynamics{0, 0, 0};
};

const double kFloorHeightM = 3.3;

Scenario scenarioNamed(const std::string& name)
{
	const Scenario c1{11, 6, 1600.0, 80.0, 400, 0, 0, 0, 2.5, 3.0, 0.8};
	const Scenario c2{32, 6, 850.0, 30.0, 350, 0, 0, 0, 2.0, 2.0, 1.2};
	Scenario s;
	if (name == "C1-out") { s = c1; s.outgoing = 1.0; }
	else if (name == "C1-in") { s = c1; s.incoming = 1.0; }
	else if (name == "C1-inter") { s = c1; s.interfloor = 1.0; }
	else if (name == "C2-inter") { s = c2; s.interfloor = 1.0; }
	else if (name == "C2-inter-hc10") {
		s = c2;
		s.interfloor = 1.0;
		s.dynamicsOverride = true;
		s.dynamics = LiftDynamics{2.5, 0.9, 1.0};	// Table 5.2, the <= 50 m row
	}
	else throw std::invalid_argument("unknown scenario " + name);
	return s;
}

// nullptr for the greedy reference (the default adapter runs it).
std::shared_ptr<BeamurgiaDispatchStrategy> variantNamed(const std::string& name, bool expectedReversal,
	int floors, uint64_t seed)
{
	if (name == "greedy") return nullptr;
	BeamurgiaDispatchStrategy::Params p;
	if (name == "S") {}
	else if (name == "S+P") p.penalty = BeamurgiaDispatchStrategy::Penalty::Death;
	else if (name == "S+P3") p.penalty = BeamurgiaDispatchStrategy::Penalty::Dynamic;
	else if (name == "S+LBI+Sd") { p.seeding = true; p.lastBestIndividual = true; }
	else if (name == "S+LBI+Sd+P") { p.seeding = true; p.lastBestIndividual = true; p.penalty = BeamurgiaDispatchStrategy::Penalty::Death; }
	else if (name == "S+LBI+Sd+P3") { p.seeding = true; p.lastBestIndividual = true; p.penalty = BeamurgiaDispatchStrategy::Penalty::Dynamic; }
	else throw std::invalid_argument("unknown variant " + name);
	if (expectedReversal) p.estimator.reversal = BeamurgiaReversal::ExpectedExtremeFloor;
	p.estimator.lowestFloor = 0;
	p.estimator.highestFloor = floors;
	p.random = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed * 1000003ULL + 17ULL));
	return std::make_shared<BeamurgiaDispatchStrategy>(p);
}

} // namespace

int main(int argc, char** argv)
{
	if (argc != 7) {
		std::cerr << "usage: beamurgia_bench <outDir> <scenario> <variant> <R1|R1p> <firstSeed> <lastSeed>\n";
		return 2;
	}
	const std::string outDir = argv[1], scenarioName = argv[2], variant = argv[3], reversal = argv[4];
	const uint64_t first = std::stoull(argv[5]), last = std::stoull(argv[6]);
	if (reversal != "R1" && reversal != "R1p") {
		std::cerr << "reversal must be R1 or R1p\n";
		return 2;
	}
	const Scenario sc = scenarioNamed(scenarioName);

	std::ofstream runs(outDir + "/runs.tsv", std::ios::app);
	for (uint64_t seed = first; seed <= last; ++seed) {
		auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed));
		auto metrics = std::make_shared<StandardMetricsCollector>();
		auto greedy = std::make_shared<GreedyNearestDispatchStrategy>();
		auto dwell = std::make_shared<FixedDwellTimeStrategy>(sc.doorOpen, sc.doorClose, sc.transferPerPassenger);
		auto travel = std::make_shared<KinematicTravelTimeStrategy>(kFloorHeightM,
			sc.dynamicsOverride ? sc.dynamics : defaultDynamicsFor(sc.floors * kFloorHeightM));
		StepMixTrafficProfile::Params tp;
		tp.populatedFloors = sc.floors;
		tp.populationPerFloor = sc.populationPerFloor;
		tp.incoming = sc.incoming;
		tp.outgoing = sc.outgoing;
		tp.interfloor = sc.interfloor;
		auto profile = std::make_shared<StepMixTrafficProfile>(tp);
		auto ga = variantNamed(variant, reversal == "R1p", sc.floors, seed);

		const std::string tag = scenarioName + "__" + variant + "__" + reversal + "__s" + std::to_string(seed);
		std::streambuf* saved = std::cout.rdbuf();
		std::ostringstream sink;
		std::cout.rdbuf(sink.rdbuf());	// the simulator's own console report
		auto t0 = std::chrono::steady_clock::now();
		Simulator sim(seed, tag, sc.floors, sc.cars, sc.passengers, /*arrivalRate=*/0.5, 1.0f, true,
			rng, greedy, metrics, nullptr, std::optional<uint64_t>(seed),
			sc.maxWeightKg, nullptr, 70.0, 15.0,
			dwell, travel, nullptr, ga, profile);
		sim.startSimulation();
		const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		sim.writeSnapshot(outDir + "/" + tag + ".json");
		std::cout.rdbuf(saved);

		runs << scenarioName << '\t' << variant << '\t' << reversal << '\t' << seed << '\t' << wall << '\t'
			<< (ga ? ga->consultations() : 0) << '\t' << (ga ? ga->optimizations() : 0) << '\t'
			<< metrics->getAverageWaitingTime() << '\n';
		runs.flush();
		std::cerr << tag << "  wall " << wall << " s  AWT " << metrics->getAverageWaitingTime() << "\n";
	}
	return 0;
}
