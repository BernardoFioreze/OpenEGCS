#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include "JsonSimulationLogger.h"
#include "StandardMetricsCollector.h"
#include "Customer.h"

using json = nlohmann::json;
using namespace openEGCS;
using Catch::Approx;

TEST_CASE("JsonSimulationLogger produces schema v11 with metadata and customers", "[snapshot]") {
	JsonSimulationLogger logger;
	StandardMetricsCollector metrics;

	SimulationMetadata meta;
	meta.floors = 10;
	meta.elevatorCount = 3;
	meta.speed = 1.0f;
	meta.arrivalRate = 0.8;
	meta.sane = true;
	meta.customersToSimulate = 5;
	meta.seed = std::optional<uint64_t>(42);
	meta.travelModel = "kinematic";
	meta.oneFloorLegSeconds = 4.945436;
	meta.trafficProfile = "stationary-uniform";
	logger.setMetadata(meta);

	Customer c1(1, "", /*t=*/0.0, /*at*/2, /*to*/8, UP);
	logger.logCustomer(c1);

	logger.logEvent(0.0, "CUST_CALL", 0, 1, 2);
	logger.logEvent(1.5, "ENTER", 1, 1, 2);

	const std::string path = "test_snapshot.json";
	logger.finalize(metrics, path);

	std::ifstream in(path);
	REQUIRE(in.is_open());
	json root;
	in >> root;

	REQUIRE(root["schemaVersion"] == 11);
	REQUIRE(root["metadata"]["floors"] == 10);
	REQUIRE(root["metadata"]["elevators"] == 3);
	REQUIRE(root["metadata"]["seed"] == 42);
	REQUIRE(root["metadata"]["startingPositions"].size() == 3);
	// The snapshot has to say which travel model produced its timestamps:
	// two runs with the same `speed` and different models are otherwise
	// indistinguishable from the file alone.
	REQUIRE(root["metadata"]["travelModel"] == "kinematic");
	REQUIRE(root["metadata"]["oneFloorLegSeconds"] == Approx(4.945436));

	// Which demand pattern generated the arrivals, the demand-side analogue of
	// travelModel, used to decide comparability.
	REQUIRE(root["metadata"]["trafficProfile"] == "stationary-uniform");

	// StandardMetricsCollector always measures pass-bys, so the block is present
	// with zeros rather than absent - absent means "this collector does not count
	// them", which is a different claim.
	REQUIRE(root["metrics"].contains("passedBy"));
	REQUIRE(root["metrics"]["passedBy"]["refusedAtDoor"]["events"] == 0);
	REQUIRE(root["metrics"]["passedBy"]["refusedAtDoor"]["passengers"].empty());
	REQUIRE(root["metrics"]["passedBy"]["reallocated"]["events"] == 0);
	REQUIRE(root["metrics"]["passedBy"]["reallocated"]["passengers"].empty());

	REQUIRE(root["customers"]["1"]["atFloor"] == 2);
	REQUIRE(root["customers"]["1"]["toFloor"] == 8);
	REQUIRE(root["customers"]["1"]["direction"] == "UP");

	REQUIRE(root["events"].size() == 2);
	REQUIRE(root["events"][0]["type"] == "CUST_CALL");
	REQUIRE(root["events"][1]["type"] == "ENTER");

	// The distribution behind each average.
	REQUIRE(root["metrics"].contains("averageWaitingTime"));
	for (const char* block : {"waitingTime", "journeyTime"}) {
		const json& d = root["metrics"][block];
		REQUIRE(d["count"] == 0);			// no boarding was fed to this collector
		REQUIRE(d["threshold"].is_null());	// no threshold was configured...
		// ...so the verdict keys are absent rather than zero: "nobody was
		// dissatisfied" and "nobody measured" must not look alike in the file.
		REQUIRE_FALSE(d.contains("dissatisfiedCount"));
		REQUIRE_FALSE(d.contains("dissatisfiedRate"));
		REQUIRE_FALSE(d.contains("dissatisfiedCustomers"));
	}

	// StandardMetricsCollector always measures the windowed reading and the
	// queue, so the blocks are present (with zeros here, since no hook was fed)
	// rather than absent - absent means "this collector does not measure it", a
	// different claim.
	for (const char* block : {"waitingTimeByWindow", "journeyTimeByWindow"}) {
		REQUIRE(root["metrics"].contains(block));
		const json& w = root["metrics"][block];
		REQUIRE(w.contains("peak"));
		REQUIRE(w.contains("drain"));
		REQUIRE(w.contains("boundaryTime"));
		REQUIRE(w["peak"]["count"] == 0);
		REQUIRE(w["drain"]["count"] == 0);
	}
	REQUIRE(root["metrics"].contains("queue"));
	REQUIRE(root["metrics"]["queue"]["max"] == 0);
	REQUIRE(root["metrics"]["queue"]["atArrivalsEnd"] == 0);

	std::remove(path.c_str());
}

// The optional policy from the other side: a collector that does not override
// the window/queue getters (they default to nullopt) must produce a snapshot
// with NO window/queue keys, so "not measured" and "measured as zero" stay
// distinct - the same guarantee the passedBy block already has.
namespace {
class NonMeasuringCollector : public IMetricsCollector {
	public:
		void onCustomerArrived(const Customer&) override {}
		void onCustomerBoarded(const Customer&, int, double) override {}
		void onCustomerAlighted(const Customer&, int, double) override {}
		void onElevatorMoved(int, double) override {}
		void onElevatorDeparted(int, double, double, double) override {}
		void onSimulationEnd(double) override {}
		double getAverageWaitingTime() const override { return 0.0; }
		double getAverageTravelTime() const override { return 0.0; }
		TimeDistribution getWaitingTimeDistribution() const override { return {}; }
		TimeDistribution getJourneyTimeDistribution() const override { return {}; }
		double getTotalEnergyConsumption() const override { return 0.0; }
		double getEnergyConsumption(int) const override { return 0.0; }
		void printSummary() const override {}
		std::optional<EnergyModelDescription> energyModel() const override { return std::nullopt; }
		// getWaitingTimeByWindow/getJourneyTimeByWindow/getQueueReport and
		// getPassByReport are left at their nullopt defaults on purpose.
};
}

TEST_CASE("A collector that does not measure windows or queue omits those keys", "[snapshot]") {
	JsonSimulationLogger logger;
	NonMeasuringCollector metrics;

	const std::string path = "test_snapshot_nowindow.json";
	logger.finalize(metrics, path);

	std::ifstream in(path);
	REQUIRE(in.is_open());
	json root;
	in >> root;

	REQUIRE_FALSE(root["metrics"].contains("waitingTimeByWindow"));
	REQUIRE_FALSE(root["metrics"].contains("journeyTimeByWindow"));
	REQUIRE_FALSE(root["metrics"].contains("queue"));
	REQUIRE_FALSE(root["metrics"].contains("passedBy"));

	std::remove(path.c_str());
}

// Item #58: a collector without an energy model writes {"model": "none"}, not
// a block of zeros that would read as a motor that costs nothing.
TEST_CASE("A collector without an energy model writes energy model none", "[snapshot]") {
	JsonSimulationLogger logger;
	NonMeasuringCollector metrics;
	SimulationMetadata meta;
	meta.floors = 4;
	meta.elevatorCount = 1;
	meta.travelModel = "kinematic";
	meta.trafficProfile = "stationary-uniform";
	meta.energy = metrics.energyModel();
	logger.setMetadata(meta);

	const std::string path = "test_snapshot_noenergy.json";
	logger.finalize(metrics, path);
	std::ifstream in(path);
	REQUIRE(in.is_open());
	json root;
	in >> root;
	REQUIRE(root["metadata"]["energy"] == json({{"model", "none"}}));
	std::remove(path.c_str());
}

TEST_CASE("JsonSimulationLogger writes the dissatisfaction verdict when a threshold is configured", "[snapshot]") {
	JsonSimulationLogger logger;

	ServiceThresholds thresholds;
	thresholds.waiting = 5.0;
	StandardMetricsCollector metrics(/*energy_per_floor=*/1.0, /*floorHeightM=*/3.5,
		defaultDynamicsFor(20.0), LiftMotorParams(), thresholds);

	Customer patient(1, "", /*t=*/0.0, /*at*/0, /*to*/5, UP);
	Customer unhappy(2, "", /*t=*/0.0, /*at*/0, /*to*/5, UP);
	metrics.onCustomerBoarded(patient, 1, /*time=*/2.0);	// waited 2
	metrics.onCustomerBoarded(unhappy, 1, /*time=*/9.0);	// waited 9 > 5

	const std::string path = "test_snapshot_threshold.json";
	logger.finalize(metrics, path);

	std::ifstream in(path);
	REQUIRE(in.is_open());
	json root;
	in >> root;

	const json& waiting = root["metrics"]["waitingTime"];
	REQUIRE(waiting["threshold"] == 5.0);
	REQUIRE(waiting["count"] == 2);
	REQUIRE(waiting["dissatisfiedCount"] == 1);
	REQUIRE(waiting["dissatisfiedRate"] == 0.5);
	// Which passenger, not just how many: the id indexes into "customers" and
	// into the event timeline, which is what makes the snapshot debuggable.
	REQUIRE(waiting["dissatisfiedCustomers"] == json::array({2}));

	// The journey threshold was left unset, and that block says so on its own.
	REQUIRE(root["metrics"]["journeyTime"]["threshold"].is_null());
	REQUIRE_FALSE(root["metrics"]["journeyTime"].contains("dissatisfiedCount"));

	std::remove(path.c_str());
}

TEST_CASE("JsonSimulationLogger without a seed emits null seed", "[snapshot]") {
	JsonSimulationLogger logger;
	StandardMetricsCollector metrics;

	SimulationMetadata meta;
	meta.floors = 5;
	meta.elevatorCount = 1;
	meta.speed = 1.0f;
	meta.arrivalRate = 0.5;
	meta.sane = true;
	meta.customersToSimulate = 3;
	meta.seed = std::nullopt;
	logger.setMetadata(meta);

	const std::string path = "test_snapshot_noseed.json";
	logger.finalize(metrics, path);

	std::ifstream in(path);
	json root;
	in >> root;
	REQUIRE(root["metadata"]["seed"].is_null());

	std::remove(path.c_str());
}

// "DEPART" marks the real instant an elevator starts moving towards its next
// stop, so a trajectory reader need not assume motion fills the entire gap
// between two stops.
TEST_CASE("JsonSimulationLogger records DEPART events", "[snapshot]") {
	JsonSimulationLogger logger;
	StandardMetricsCollector metrics;

	logger.logEvent(0.0, "CUST_CALL", 0, 1, 2);
	logger.logEvent(0.0, "DEPART", 1, 0, 0);
	logger.logEvent(2.0, "ENTER", 1, 1, 2);

	const std::string path = "test_snapshot_depart.json";
	logger.finalize(metrics, path);

	std::ifstream in(path);
	json root;
	in >> root;

	REQUIRE(root["events"][1]["type"] == "DEPART");
	REQUIRE(root["events"][1]["elevatorId"] == 1);
	REQUIRE(root["events"][1]["floor"] == 0);

	std::remove(path.c_str());
}
