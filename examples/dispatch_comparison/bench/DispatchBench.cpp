/*
DispatchBench.cpp - runs every dispatch controller the platform ships under ONE
environment, so that a difference between two rows of the result can only come
from the controllers. The two paper benches (examples/gahca_dispatch/bench and
examples/beamurgia_dispatch/bench) do the opposite on purpose: each rebuilds its
own paper's installation. A measurement tool, not part of either library.
analyze.py (next to this file) reads what it writes.

    dispatch_bench --plan
    dispatch_bench <outDir> <scenario> <greedy|thv|gahca|beamurgia> <firstSeed> <lastSeed> [residualInertiaKgM2]
    python3 -I analyze.py <outDir>

--plan prints the handling-capacity chain and what each scenario derives from
it, computed, so a table built from these runs can be audited against the code.
The optional last argument overrides the energy model's lumped inertia (see
below).

THE ENVIRONMENT is the platform's defaults, stated here because three of them
have no default a run could inherit:
  12 levels, 0..11. The engine's `floors` argument is the HIGHEST floor, not a
       count: floors run 0..floors inclusive (StationaryUniformTrafficProfile,
       CustomerFactory). So the Simulator gets 11, and the 12-level
       LunchPeakTrafficProfile(12, ...) and ThvDispatchStrategy(12) describe the
       same building.
  4 cars, 1000 kg rated (the size of the group and the rated load are this
       bench's choice: the engine's default rated load is unlimited, which has
       no handling capacity to compare a demand with). 1000 kg is the load the
       default masses of LiftMotorParams were written next to.
  floor height 3.5 m, the project's default.
  LiftDynamics from defaultDynamicsFor(travel) = Table 5.2's 2.5 m/s, 0.9 m/s^2,
       1.0 m/s^3 row. Built once and given to BOTH the travel model and the
       energy model (as main.cpp does), so time and energy describe one journey.
  KinematicTravelTimeStrategy, the default dwell (centre door, office transfer,
       timed hold; left null so the engine builds it), LOOK (null), 60% load
       bypass (null), weights Normal(70, 15), everybody boarding what they fit
       into (null). Nothing here is any paper's physics.

THE TRAFFIC. Three patterns, each at a demand below and above the group's up-peak
handling capacity HC (TrafficCapacity.h, Barney & Al-Sharif eq. 4.4), the ruler
the field sizes installations with. A scenario is named <pattern>-<rho>, rho being
the peak five minutes' demand over HC:
  up       UpPeakTrafficProfile, Table 17.2's hour;
  lunch    LunchPeakTrafficProfile, Fig. 5's 130 minutes, both streams;
  uniform  the engine's default, stationary uniform, for one hour.
The count of customers is what the curve integrates to, so a run spans the whole
pattern. HC is an up-peak figure and only a ruler for the other two: rho = 0.7 is
below it everywhere, and "above" is the smallest of {1.3, 1.6, 2.0} at which the
GREEDY's queue when the arrivals end reaches 5% of the passengers. Only the greedy
was run to pick it, so the list of cells depends on no controller's result:
up-2.0, lunch-2.0, uniform-2.0. Over ten seeds, at rho 1.3 / 1.6 / 2.0:
up 0.2% / 1.3% / 10.9%, lunch 0.1% / 1.0% / 10.0%, uniform 2.2% / 3.4% / 13.6%.
The rule has to be run again whenever the engine's throughput moves.
scenarioNames() is that list; any other <pattern>-<rho> still runs, which is how
near-capacity cells such as up-1.3 and lunch-1.6 can be measured too.

THE CONTROLLERS. greedy and thv draw nothing. gahca and beamurgia each draw from
a provider of their own, seeded from the run's seed, so that nothing but the
arrivals touches the engine's provider and every controller run with one seed
sees the same passengers (common random numbers). analyze.py checks that instead
of assuming it. A controller is one line in controllerNamed().

Both controllers' parameters are their papers' defaults and are NOT adapted to
this environment. In particular GahcaDispatchStrategy's estimator keeps Cortes'
5 s per floor and 13 s per stop whatever the cabin really does: a controller's
model of the building is part of the controller. BeamurgiaDispatchStrategy's
estimator reads the run's own travel and dwell models, so it has no such gap.

ENERGY. The snapshot carries the default drive, non-regenerative: braking power is
dissipated. TwoDriveCollector prices the same legs a second time with
regenerative = true, which needs no second run because energy feeds nothing back
into the simulation, and writes it to runs.tsv, the one figure not in the
snapshot.
*/

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "BeamurgiaDispatchStrategy.h"
#include "DwellPresets.h"
#include "GahcaDispatchStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "KinematicTravelTimeStrategy.h"
#include "LiftKinematics.h"
#include "LunchPeakTrafficProfile.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "StandardMetricsCollector.h"
#include "ThvDispatchStrategy.h"
#include "TrafficCapacity.h"
#include "UpPeakTrafficProfile.h"

using namespace openEGCS;
using namespace openEGCS::examples;

namespace {

const int kTopFloor = 11;				// levels 0..11
const int kCars = 4;
const double kRatedLoadKg = 1000.0;
const double kFloorHeightM = 3.5;
const double kMeanWeightKg = 70.0;
const double kStddevWeightKg = 15.0;
const double kDesignLoadFactor = 0.8;	// the handbook's UPPHC, as main.cpp's warning uses it

LiftDynamics dynamics()
{
	return defaultDynamicsFor(kTopFloor * kFloorHeightM);
}

// The ruler, from this environment's own physics. Same mapping as main.cpp's
// saturation warning, except floorsAboveLobby: main.cpp reads its `floors`
// argument as a count (floors - 1), and in this bench the highest floor is known.
UppeakCapacity capacity()
{
	const LiftDynamics d = dynamics();
	const KinematicTravelTimeStrategy travel(kFloorHeightM, d);
	const DwellTimes dwell = resolveDwellTimes(DwellPreset{}, kRatedLoadKg);
	return estimateUppeakCapacity(kTopFloor, kCars, kRatedLoadKg / kMeanWeightKg, kDesignLoadFactor,
		kFloorHeightM / d.ratedSpeedMs, travel.legDuration(1.0),
		dwell.doorOpenS + dwell.doorCloseS, dwell.perPassengerS);
}

// A shipped peaked curve read back at scale 1, so that a scale is derived from the
// profile's own numbers and not from a copy of them that could drift.
template <class Profile>
class CurveOf : public Profile {
	public:
		using Profile::Profile;
		double peakPerFiveMinutes() const { return this->lambdaMax * 300.0; }
		double expectedArrivals(double horizonS) const
		{
			const double dt = 1.0;
			double sum = 0.0;
			for (double t = 0.5 * dt; t < horizonS; t += dt) sum += this->rate(t) * dt;
			return sum;
		}
};

struct Scenario {
	std::string pattern;		// "up", "lunch" or "uniform"
	double rho;
	double scale;				// of the peaked curve; 0 for uniform
	double arrivalRate;			// arrivals/s; used by uniform only
	int customers;
	double horizonS;			// the part of the pattern the customers span
};

// "<pattern>-<rho>": up-2.0, lunch-0.7.
Scenario scenarioNamed(const std::string& name)
{
	const std::string unknown = "unknown scenario " + name;
	Scenario s;
	const size_t dash = name.rfind('-');
	if (dash == std::string::npos) throw std::invalid_argument(unknown);
	s.pattern = name.substr(0, dash);
	s.rho = std::stod(name.substr(dash + 1));
	const double demand = s.rho * capacity().handlingCapacity;	// persons per 5 minutes

	if (s.pattern == "up") {
		const CurveOf<UpPeakTrafficProfile> curve;
		s.horizonS = 3600.0;			// the template's hour
		s.scale = demand / curve.peakPerFiveMinutes();
		s.arrivalRate = 0.0;
		s.customers = static_cast<int>(s.scale * curve.expectedArrivals(s.horizonS) + 0.5);
	} else if (s.pattern == "lunch") {
		const CurveOf<LunchPeakTrafficProfile> curve(kTopFloor + 1);
		s.horizonS = 7800.0;			// the template's 130 minutes, both streams
		s.scale = demand / curve.peakPerFiveMinutes();
		s.arrivalRate = 0.0;
		s.customers = static_cast<int>(s.scale * curve.expectedArrivals(s.horizonS) + 0.5);
	} else if (s.pattern == "uniform") {
		s.horizonS = 3600.0;
		s.scale = 0.0;
		s.arrivalRate = demand / 300.0;
		s.customers = static_cast<int>(s.arrivalRate * s.horizonS + 0.5);
	} else {
		throw std::invalid_argument(unknown);
	}
	return s;
}

// The cells of the published table.
const std::vector<std::string>& scenarioNames()
{
	static const std::vector<std::string> names{
		"up-0.7", "up-2.0", "lunch-0.7", "lunch-2.0", "uniform-0.7", "uniform-2.0"};
	return names;
}

std::shared_ptr<ITrafficProfile> profileOf(const Scenario& s)
{
	if (s.pattern == "up") return std::make_shared<UpPeakTrafficProfile>(s.scale);
	if (s.pattern == "lunch") return std::make_shared<LunchPeakTrafficProfile>(kTopFloor + 1, s.scale);
	return nullptr;		// the engine's default, StationaryUniformTrafficProfile at arrivalRate
}

// Hands the wrapped group strategy a provider of its own instead of the engine's,
// so the GA's draws do not move the arrival stream. The same few lines as in
// GahcaBench.cpp, kept separate so each bench stands alone.
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

struct Controller {
	std::shared_ptr<IDispatchStrategy> dispatch;
	std::shared_ptr<IGroupDispatchStrategy> group;	// null: the default adapter around `dispatch`
};

Controller controllerNamed(const std::string& name, uint64_t seed)
{
	auto ownStream = [seed]() {
		return std::make_shared<RandomProvider>(std::optional<uint64_t>(seed * 1000003ULL + 17ULL));
	};
	// A group strategy decides every allocation, so its per-call dispatch is
	// never consulted; the engine still requires one.
	auto greedy = std::make_shared<GreedyNearestDispatchStrategy>();
	if (name == "greedy") return {greedy, nullptr};
	if (name == "thv") return {std::make_shared<ThvDispatchStrategy>(kTopFloor + 1), nullptr};
	if (name == "gahca")
		return {greedy, std::make_shared<OwnStreamGroupStrategy>(std::make_shared<GahcaDispatchStrategy>(), ownStream())};
	if (name == "beamurgia") {
		// The paper's full variant, S+LBI+Sd+P3 (S is imposed by the engine).
		BeamurgiaDispatchStrategy::Params p;
		p.seeding = true;
		p.lastBestIndividual = true;
		p.penalty = BeamurgiaDispatchStrategy::Penalty::Dynamic;
		p.estimator.lowestFloor = 0;
		p.estimator.highestFloor = kTopFloor;
		p.passengerMassKg = kMeanWeightKg;
		p.random = ownStream();
		return {greedy, std::make_shared<BeamurgiaDispatchStrategy>(p)};
	}
	throw std::invalid_argument("unknown controller " + name);
}

// The default collector, plus the same legs priced for a regenerative drive.
// Every hook but the energy one goes to the base class alone: the second
// collector exists for one number. The hook is the only input energy has, so
// the two figures describe the very same legs and loads.
class TwoDriveCollector : public StandardMetricsCollector {
	public:
		TwoDriveCollector(LiftDynamics dynamics, LiftMotorParams motor)
			: StandardMetricsCollector(1.0, kFloorHeightM, dynamics, motor),
			  regenerative(1.0, kFloorHeightM, dynamics, regenerativeOf(motor)) {}

		void onElevatorDeparted(int elevatorId, double floorsTravelled, double currentLoad, double time) override
		{
			StandardMetricsCollector::onElevatorDeparted(elevatorId, floorsTravelled, currentLoad, time);
			regenerative.onElevatorDeparted(elevatorId, floorsTravelled, currentLoad, time);
		}

		double regenerativeEnergyJ() const { return regenerative.getTotalEnergyConsumption(); }

	private:
		static LiftMotorParams regenerativeOf(LiftMotorParams motor)
		{
			motor.regenerative = true;
			return motor;
		}
		StandardMetricsCollector regenerative;
};

int printPlan()
{
	const LiftDynamics d = dynamics();
	const KinematicTravelTimeStrategy travel(kFloorHeightM, d);
	const DwellTimes dwell = resolveDwellTimes(DwellPreset{}, kRatedLoadKg);
	const UppeakCapacity c = capacity();
	const LiftMotorParams motor;
	std::cout << "building: levels 0.." << kTopFloor << ", " << kCars << " cars, " << kRatedLoadKg << " kg, "
		<< kFloorHeightM << " m per floor\n"
		<< "dynamics: " << d.ratedSpeedMs << " m/s, " << d.accelMs2 << " m/s2, " << d.jerkMs3 << " m/s3; tv "
		<< kFloorHeightM / d.ratedSpeedMs << " s, tf(1) " << travel.legDuration(1.0) << " s\n"
		<< "door: open " << dwell.doorOpenS << " s, close " << dwell.doorCloseS << " s, transfer "
		<< dwell.perPassengerS << " s per passenger\n"
		<< "drive: car " << motor.carMassKg << " kg, counterweight " << motor.counterweightMassKg
		<< " kg, snapshot energy " << (motor.regenerative ? "regenerative" : "non-regenerative") << "\n"
		<< "capacity: P " << c.passengersPerTrip << ", H " << c.highestReversalFloor << ", S " << c.expectedStops
		<< ", RTT " << c.roundTripTimeS << " s, interval " << c.intervalS << " s, HC " << c.handlingCapacity
		<< " persons/5 min\n";
	for (const std::string& name : scenarioNames()) {
		const Scenario s = scenarioNamed(name);
		std::cout << name << ": rho " << s.rho << ", peak demand " << s.rho * c.handlingCapacity
			<< " persons/5 min, scale " << s.scale << ", rate " << s.arrivalRate << "/s, customers "
			<< s.customers << ", horizon " << s.horizonS << " s\n";
	}
	return 0;
}

} // namespace

int main(int argc, char** argv)
{
	if (argc == 2 && std::string(argv[1]) == "--plan") return printPlan();
	if (argc != 6 && argc != 7) {
		std::cerr << "usage: dispatch_bench --plan\n"
			"       dispatch_bench <outDir> <scenario> <greedy|thv|gahca|beamurgia> <firstSeed> <lastSeed>"
			" [residualInertiaKgM2]\n";
		return 2;
	}
	// The energy model's lumped inertia, 10 kg.m^2 by default. A sensitivity
	// probe, not part of the environment: 0 is the lower bound (car,
	// counterweight and load only), and the question it answers is whether the
	// controllers' order by energy survives it (see LiftMotorParams). The
	// snapshot records the value used (metadata.energy).
	LiftMotorParams motor;
	if (argc == 7) motor.residualInertiaKgM2 = std::stod(argv[6]);
	const std::string outDir = argv[1], scenarioName = argv[2], controller = argv[3];
	const uint64_t first = std::stoull(argv[4]), last = std::stoull(argv[5]);
	const Scenario sc = scenarioNamed(scenarioName);
	controllerNamed(controller, first);		// a typo fails here, not in a shell loop's log

	std::ofstream runs(outDir + "/runs.tsv", std::ios::app);
	for (uint64_t seed = first; seed <= last; ++seed) {
		const LiftDynamics d = dynamics();
		auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed));
		auto metrics = std::make_shared<TwoDriveCollector>(d, motor);
		auto travel = std::make_shared<KinematicTravelTimeStrategy>(kFloorHeightM, d);
		const Controller ctl = controllerNamed(controller, seed);

		const std::string tag = scenarioName + "__" + controller + "__s" + std::to_string(seed);
		std::streambuf* saved = std::cout.rdbuf();
		std::ostringstream sink;
		std::cout.rdbuf(sink.rdbuf());	// the simulator's own console report
		auto t0 = std::chrono::steady_clock::now();
		Simulator sim(seed, tag, kTopFloor, kCars, sc.customers, sc.arrivalRate, /*speed=*/1.0f, /*sane=*/true,
			rng, ctl.dispatch, metrics, /*logger=*/nullptr, std::optional<uint64_t>(seed),
			kRatedLoadKg, /*capacity=*/nullptr, kMeanWeightKg, kStddevWeightKg,
			/*dwell=*/nullptr, travel, /*passenger=*/nullptr, ctl.group, profileOf(sc));	// null: the engine's default, for this rated load
		sim.startSimulation();
		const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		sim.writeSnapshot(outDir + "/" + tag + ".json");
		std::cout.rdbuf(saved);

		runs << scenarioName << '\t' << controller << '\t' << seed << '\t' << wall << '\t'
			<< metrics->regenerativeEnergyJ() << '\n';
		runs.flush();
		std::cerr << tag << "  wall " << wall << " s  wait " << metrics->getAverageWaitingTime()
			<< "  energy " << metrics->getTotalEnergyConsumption() << "\n";
	}
	return 0;
}
