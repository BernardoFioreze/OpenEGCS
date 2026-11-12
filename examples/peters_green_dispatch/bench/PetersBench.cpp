/*
PetersBench.cpp - the measurement driver for examples::PetersGreenDispatchStrategy:
runs the lunch-time scenario of Peters (1998), section 9.3, under one
dispatcher, with or without his Green Strategy No. 2, for a range of seeds. One
snapshot per run plus a line of runs.tsv for analyze.py (next to this file). A
measurement tool, not part of either library.

    peters_bench <outDir> <thv|greedy> <plain|green> <percent> <outgoing> <tese|s592>
                 <firstSeed> <lastSeed> [total10]
    python3 -I analyze.py <outDir>

The building is the thesis's: 14 storeys (floors 0..13) and 6 lifts (section 9.3).
What section 9.3 does not state comes from the same thesis: 3.6 m floors, doors
1.8 s / 2.9 s, 1.2 s per passenger, 75 kg passengers, 1250 kg cars (section
5.9.2, his lunch-time example). <percent> is the demand in percent of the
population per 5 minutes and <outgoing> the fraction going down to the ground,
the rest interfloor: neither is published, so both are arguments. 80 people per
floor, one hour.

Kinematics:
  tese    1.875 m/s, 0.375 m/s^2, 0.375 m/s^3, the worked example of chapter 7 that
          Fig. 9.3 plots. The regime the motor model was validated in.
  s592    2.5 m/s, 0.8 m/s^2, 2 m/s^3, section 5.9.2 (a sensitivity check).

Energy: the project's default motor, whose J = 10 is the thesis's, regenerative
as Fig. 9.3 shows, which is how the bench turns it on, and with gear efficiency
1, because below 1 the regenerative branch returns more than mgh. Each leg is
also priced clamped, and its potential term is summed, so a run splits into
E = dPE + losses. The optional `total10` reads J = 10 as the TOTAL inertia
instead (see below).
*/

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

#include "FixedDwellTimeStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "KinematicTravelTimeStrategy.h"
#include "LiftKinematics.h"
#include "LiftMotorModel.h"
#include "PetersGreenDispatchStrategy.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "StandardMetricsCollector.h"
#include "StepMixTrafficProfile.h"
#include "ThvDispatchStrategy.h"

using namespace openEGCS;
using namespace openEGCS::examples;

namespace {

const int kHighestFloor = 13;
const int kCars = 6;
const double kFloorHeightM = 3.6;
const double kRatedLoadKg = 1250.0;
const double kPopulationPerFloor = 80.0;
const double kGravity = 9.81;
const double kMinutes = 60.0;

class EnergyBalanceCollector : public StandardMetricsCollector {
	public:
		EnergyBalanceCollector(LiftDynamics d, LiftMotorParams p)
			// The base prices legs with the same motor as the two figures below, so
			// the snapshot's energy is energyRegenJ and its metadata.energy
			// describes the motor the analysis used.
			: StandardMetricsCollector(1.0, kFloorHeightM, d, p), dynamics(d), regenerative(p), clamped(p)
		{
			clamped.regenerative = false;
		}

		void onElevatorDeparted(int elevatorId, double floorsTravelled, double currentLoad, double time) override
		{
			StandardMetricsCollector::onElevatorDeparted(elevatorId, floorsTravelled, currentLoad, time);
			const double metres = floorsTravelled * kFloorHeightM;
			if (metres != 0.0) ++starts;
			distanceM += std::fabs(metres);
			energyRegenJ += tripEnergyJ(metres, currentLoad, dynamics, regenerative);
			energyClampedJ += tripEnergyJ(metres, currentLoad, dynamics, clamped);
			potentialJ += (regenerative.carMassKg - regenerative.counterweightMassKg + currentLoad) * kGravity * metres;
		}

		void onSimulationEnd(double totalTime) override
		{
			StandardMetricsCollector::onSimulationEnd(totalTime);
			endTime = totalTime;
		}

		double energyRegenJ = 0.0, energyClampedJ = 0.0, potentialJ = 0.0, distanceM = 0.0, endTime = 0.0;
		long starts = 0;

	private:
		LiftDynamics dynamics;
		LiftMotorParams regenerative, clamped;
};

bool fileIsEmpty(const std::string& path)
{
	struct stat s;
	return stat(path.c_str(), &s) != 0 || s.st_size == 0;
}

} // namespace

int main(int argc, char** argv)
{
	if (argc != 9 && argc != 10) {
		std::cerr << "usage: peters_bench <outDir> <thv|greedy> <plain|green> <percent> <outgoing> "
			"<tese|s592> <firstSeed> <lastSeed> [total10]\n";
		return 2;
	}
	// "total10" reads the thesis's J = 10 as the TOTAL inertia, as its worked
	// example does, so the lumped constant is 10 minus this installation's mass
	// term at balanced load; by default it is added on top of the masses, as
	// the energy model always has.
	const bool total10 = argc == 10 && std::string(argv[9]) == "total10";
	if (argc == 10 && !total10) {
		std::cerr << "the optional last argument is total10\n";
		return 2;
	}
	const std::string outDir = argv[1], baseline = argv[2], rule = argv[3], kinematics = argv[6];
	const double percent = std::stod(argv[4]), outgoing = std::stod(argv[5]);
	const uint64_t first = std::stoull(argv[7]), last = std::stoull(argv[8]);
	if ((baseline != "thv" && baseline != "greedy") || (rule != "plain" && rule != "green")
		|| (kinematics != "tese" && kinematics != "s592")) {
		std::cerr << "unknown baseline, rule or kinematics\n";
		return 2;
	}

	const LiftDynamics dynamics = kinematics == "tese" ? LiftDynamics{1.875, 0.375, 0.375} : LiftDynamics{2.5, 0.8, 2.0};
	LiftMotorParams motor;
	motor.regenerative = true;
	motor.gearEfficiency = 1.0;
	motor.counterweightMassKg = motor.carMassKg + kRatedLoadKg / 2.0;
	if (total10)
		motor.residualInertiaKgM2 = 10.0 - equivalentInertia(motor.carMassKg, kRatedLoadKg / 2.0,
			motor.counterweightMassKg, motor.driveSheaveDiameterM, motor.gearRatio, motor.ropingRatio, 0.0);
	const int passengers = static_cast<int>(std::lround(percent / 100.0 * kHighestFloor * kPopulationPerFloor
		* kMinutes / 5.0));

	const std::string runsPath = outDir + "/runs.tsv";
	const bool writeHeader = fileIsEmpty(runsPath);
	std::ofstream runs(runsPath, std::ios::app);
	if (writeHeader) {
		runs << "baseline\trule\tpercent\toutgoing\tkinematics\tseed\tpassengers\tendTime\tenergyRegenJ"
			"\tenergyClampedJ\tpotentialJ\tstarts\tdistanceM\tdecisions\tforced\ttag\n";
	}

	for (uint64_t seed = first; seed <= last; ++seed) {
		auto metrics = std::make_shared<EnergyBalanceCollector>(dynamics, motor);
		std::shared_ptr<IDispatchStrategy> dispatch;
		if (baseline == "thv") dispatch = std::make_shared<ThvDispatchStrategy>(kHighestFloor + 1);
		else dispatch = std::make_shared<GreedyNearestDispatchStrategy>();
		std::shared_ptr<PetersGreenDispatchStrategy> green;
		if (rule == "green") {
			green = std::make_shared<PetersGreenDispatchStrategy>(dispatch);
			dispatch = green;
		}

		StepMixTrafficProfile::Params traffic;
		traffic.populatedFloors = kHighestFloor;
		traffic.populationPerFloor = kPopulationPerFloor;
		traffic.outgoing = outgoing;
		traffic.interfloor = 1.0 - outgoing;
		traffic.minPercent = percent;
		traffic.maxPercent = percent;

		std::ostringstream tagStream;
		tagStream << baseline << "__" << rule << "__p" << percent << "__o" << outgoing << "__" << kinematics << "__s" << seed;
		const std::string tag = tagStream.str();

		std::streambuf* saved = std::cout.rdbuf();
		std::ostringstream sink;
		std::cout.rdbuf(sink.rdbuf());	// the simulator's own console report
		Simulator sim(seed, tag, kHighestFloor, kCars, passengers, 0.5, 1.0f, true,
			std::make_shared<RandomProvider>(std::optional<uint64_t>(seed)), dispatch, metrics, nullptr,
			std::optional<uint64_t>(seed), kRatedLoadKg, nullptr, 75.0, 15.0,
			std::make_shared<FixedDwellTimeStrategy>(1.8, 2.9, 1.2),
			std::make_shared<KinematicTravelTimeStrategy>(kFloorHeightM, dynamics), nullptr, nullptr,
			std::make_shared<StepMixTrafficProfile>(traffic));
		sim.startSimulation();
		sim.writeSnapshot(outDir + "/" + tag + ".json");
		std::cout.rdbuf(saved);

		runs.precision(12);
		runs << baseline << '\t' << rule << '\t' << percent << '\t' << outgoing << '\t' << kinematics << '\t' << seed << '\t'
			<< passengers << '\t' << metrics->endTime << '\t' << metrics->energyRegenJ << '\t'
			<< metrics->energyClampedJ << '\t' << metrics->potentialJ << '\t' << metrics->starts << '\t'
			<< metrics->distanceM << '\t' << (green ? green->decisions() : 0) << '\t'
			<< (green ? green->forced() : 0) << '\t' << tag << '\n';
		runs.flush();
	}
	return 0;
}
