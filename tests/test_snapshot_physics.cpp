// The snapshot records the physics that produced its timestamps and its
// energy - the installation, the travel model's parameters, the resolved
// dwell and the collector's energy model - each asked of the component that
// holds it. Without it, the same command with the default door and with a
// zero door would write metadata blocks with the same keys and values while
// waiting 70.86 s and 31.62 s on average.
//
// What these cases pin is that the file says what the engine RAN, not what a
// caller meant to configure: the dwell is read after the system resolved the
// default for the rated load, and the energy model is the injected
// collector's.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <nlohmann/json.hpp>
#include "Simulator.h"
#include "RandomProvider.h"
#include "StandardMetricsCollector.h"
#include "GreedyNearestDispatchStrategy.h"
#include "FixedDwellTimeStrategy.h"
#include "DoorHoldDwellTimeStrategy.h"
#include "LinearTravelTimeStrategy.h"

using namespace openEGCS;
using json = nlohmann::json;
using Catch::Approx;

namespace {

struct Run {
	double maxWeight = std::numeric_limits<double>::infinity();
	std::shared_ptr<IMetricsCollector> metrics;
	std::shared_ptr<IDwellTimeStrategy> dwell;
	std::shared_ptr<ITravelTimeStrategy> travel;
};

// A small seeded run; returns the snapshot's metadata block.
json metadataOf(const Run& r, const std::string& path)
{
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(7));
	auto metrics = r.metrics ? r.metrics : std::make_shared<StandardMetricsCollector>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	Simulator sim(1, "physics", /*floors=*/8, /*elevators=*/2, /*customers=*/20,
		/*arrivalRate=*/0.5, /*speed=*/1.0f, /*sane=*/true,
		rng, dispatch, metrics, nullptr, std::optional<uint64_t>(7),
		r.maxWeight, nullptr, 70.0, 15.0, r.dwell, r.travel);
	sim.startSimulation();
	sim.writeSnapshot(path);
	std::ifstream in(path);
	REQUIRE(in.is_open());
	json root;
	in >> root;
	std::remove(path.c_str());
	REQUIRE(root["schemaVersion"] == 11);
	return root["metadata"];
}

} // namespace

TEST_CASE("The default run records the handbook door, the kinematic travel and the motor", "[snapshot][e2e]") {
	const json m = metadataOf(Run{}, "test_physics_default.json");

	// Unlimited cabin: no rated load and no bypass threshold, written as null.
	REQUIRE(m["installation"]["ratedLoadKg"].is_null());
	REQUIRE(m["installation"]["bypassThresholdKg"].is_null());
	REQUIRE(m["installation"]["meanPassengerKg"] == Approx(70.0));
	REQUIRE(m["installation"]["stddevPassengerKg"] == Approx(15.0));

	REQUIRE(m["travel"]["model"] == "kinematic");
	REQUIRE(m["travel"]["floorHeightM"] == Approx(3.5));
	REQUIRE(m["travel"].contains("ratedSpeedMs"));
	REQUIRE_FALSE(m["travel"].contains("secondsPerFloor"));

	// Unlimited is "above 1000 kg": the 1100 mm centre door, office transfer,
	// timed hold.
	REQUIRE(m["dwell"]["model"] == "door-hold");
	REQUIRE(m["dwell"]["doorOpenS"] == Approx(2.5));
	REQUIRE(m["dwell"]["doorCloseS"] == Approx(3.0));
	REQUIRE(m["dwell"]["perPassengerS"] == Approx(1.2));
	REQUIRE(m["dwell"]["hold"] == "timed");
	REQUIRE(m["dwell"]["carCallHoldS"] == Approx(2.0));
	REQUIRE(m["dwell"]["landingCallHoldS"] == Approx(3.0));
	REQUIRE_FALSE(m["dwell"].contains("detectorHoldS"));

	REQUIRE(m["energy"]["model"] == "dc-static-converter");
	REQUIRE(m["energy"]["residualInertiaKgM2"] == Approx(10.0));
	REQUIRE(m["energy"]["regenerative"] == false);
}

// The default door depends on the rated load, so reading the dwell before
// the system resolves it would record the wrong door on one side of the
// 1000 kg limit.
TEST_CASE("The recorded door follows the rated load across the 1000 kg limit", "[snapshot][e2e]") {
	Run small; small.maxWeight = 1000.0;
	Run large; large.maxWeight = 1001.0;
	const json a = metadataOf(small, "test_physics_1000.json");
	const json b = metadataOf(large, "test_physics_1001.json");

	REQUIRE(a["dwell"]["doorOpenS"] == Approx(2.0));
	REQUIRE(a["dwell"]["doorCloseS"] == Approx(2.0));
	REQUIRE(b["dwell"]["doorOpenS"] == Approx(2.5));
	REQUIRE(b["dwell"]["doorCloseS"] == Approx(3.0));

	// The default 60% load-weighing setting, resolved for each cabin.
	REQUIRE(a["installation"]["ratedLoadKg"] == Approx(1000.0));
	REQUIRE(a["installation"]["bypassThresholdKg"] == Approx(600.0));
	REQUIRE(b["installation"]["bypassThresholdKg"] == Approx(600.6));
}

TEST_CASE("An injected zero dwell and a linear clock are recorded as such", "[snapshot][e2e]") {
	Run r;
	r.dwell = std::make_shared<FixedDwellTimeStrategy>(0.0, 0.0, 0.0);
	r.travel = std::make_shared<LinearTravelTimeStrategy>(2.0);
	const json m = metadataOf(r, "test_physics_zero.json");

	REQUIRE(m["dwell"]["model"] == "fixed");
	REQUIRE(m["dwell"]["doorOpenS"] == Approx(0.0));
	REQUIRE(m["dwell"]["hold"] == "none");
	REQUIRE_FALSE(m["dwell"].contains("carCallHoldS"));

	REQUIRE(m["travel"]["model"] == "linear");
	REQUIRE(m["travel"]["secondsPerFloor"] == Approx(2.0));
	REQUIRE_FALSE(m["travel"].contains("floorHeightM"));
}

TEST_CASE("A detector door records the hold it cuts to", "[snapshot][e2e]") {
	DoorHoldDwellTimeStrategy::Params p;
	p.doorOpenS = 1.0; p.doorCloseS = 1.5; p.perPassengerS = 0.8; p.detector = true;
	Run r;
	r.dwell = std::make_shared<DoorHoldDwellTimeStrategy>(p);
	const json m = metadataOf(r, "test_physics_detector.json");

	REQUIRE(m["dwell"]["hold"] == "detector");
	REQUIRE(m["dwell"]["detectorHoldS"] == Approx(0.5));
	REQUIRE(m["dwell"]["doorCloseS"] == Approx(1.5));
}

// The energy block is the collector's, not a constant: change the motor and
// the drive, and the file says so.
TEST_CASE("The energy block is the injected collector's motor", "[snapshot][e2e]") {
	LiftMotorParams motor;
	motor.residualInertiaKgM2 = 3.0;
	motor.regenerative = true;
	motor.carMassKg = 1250.0;
	LiftDynamics d{1.875, 0.375, 0.375};
	Run r;
	r.metrics = std::make_shared<StandardMetricsCollector>(1.0, 3.6, d, motor);
	const json m = metadataOf(r, "test_physics_motor.json");

	REQUIRE(m["energy"]["residualInertiaKgM2"] == Approx(3.0));
	REQUIRE(m["energy"]["regenerative"] == true);
	REQUIRE(m["energy"]["carMassKg"] == Approx(1250.0));
	REQUIRE(m["energy"]["floorHeightM"] == Approx(3.6));
	REQUIRE(m["energy"]["accelMs2"] == Approx(0.375));
}
