#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <sstream>
#include "Simulator.h"
#include "GahcaDispatchStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "RandomProvider.h"
#include "ISimulationLogger.h"
#include "StandardMetricsCollector.h"
#include "FixedDwellTimeStrategy.h"
#include "LinearTravelTimeStrategy.h"
#include "ThvDispatchStrategy.h"

using namespace openEGCS;
using namespace openEGCS::examples;

// Wall-clock cost of running GAHCA through a full simulation, measured rather
// than guessed. HIDDEN (tag starts with ".") - it does not run in the normal
// suite; invoke it directly: ./build/tests/liftsim_tests "[.gahca_bench]" --success

namespace {

// Discards everything - the timing is of the dispatch, not of snapshot I/O.
struct NoopLogger : public ISimulationLogger {
	void setMetadata(const SimulationMetadata&) override {}
	void logCustomer(const Customer&) override {}
	void logEvent(double, const std::string&, int, unsigned long int, int) override {}
	void finalize(const IMetricsCollector&, const std::string&) override {}
};

struct Scenario { int floors, elevators, customers; double rate; uint64_t seed; };

double timeRun(const Scenario& s, std::shared_ptr<IGroupDispatchStrategy> group) {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(s.seed));
	auto dispatch = std::make_shared<GreedyNearestDispatchStrategy>();
	auto logger = std::make_shared<NoopLogger>();
	Simulator sim(1, "bench", s.floors, s.elevators, s.customers, s.rate, 1.0f, true,
		rng, dispatch, nullptr, logger, std::nullopt,
		std::numeric_limits<double>::infinity(), nullptr, 70.0, 15.0,
		nullptr, nullptr, nullptr, group);
	auto t0 = std::chrono::steady_clock::now();
	sim.startSimulation();
	auto t1 = std::chrono::steady_clock::now();
	return std::chrono::duration<double>(t1 - t0).count();
}

} // namespace

TEST_CASE("GAHCA wall-clock cost vs the default", "[.gahca_bench]") {
	const Scenario scenarios[] = {
		{10, 3, 50, 0.8, 42},
		{15, 2, 200, 2.0, 99},
		{20, 4, 400, 3.0, 7},
		{20, 4, 3000, 3.0, 7},		// the saturated reference scenario
	};
	std::ostringstream report;
	report << "\n=== GAHCA bench (seconds) ===\n";
	for (const Scenario& s : scenarios) {
		double base = timeRun(s, nullptr);						// default ImmediateGroupAdapter
		double gahca = timeRun(s, std::make_shared<GahcaDispatchStrategy>());
		report << s.floors << "/" << s.elevators << "/" << s.customers
			<< " rate " << s.rate << ": default=" << base << "  gahca=" << gahca
			<< "  factor=" << (base > 0 ? gahca / base : 0.0) << "\n";
	}
	WARN(report.str());
	SUCCEED();
}

// GAHCA vs the greedy baseline under the paper's reproduction environment
// (linear travel 5 s/floor + Cortes dwell: 2 s door open, 5 s transfer, 2 s
// door close; the 2+2 s accel/decel of the paper's per-stop delay have no home
// in the linear model and are not charged - a documented limitation). Both run
// the SAME environment, so the comparison is fair. Uniform traffic here; the
// paper's lunch-peak comparison is examples/gahca_dispatch/bench.
// HIDDEN; run with ./build/tests/liftsim_tests "[.gahca_repro]".
namespace {

struct Result { double awt; double ajt; double energy; double p90; };

Result reproRun(const Scenario& s, std::shared_ptr<IDispatchStrategy> dispatch,
	std::shared_ptr<IGroupDispatchStrategy> group) {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(s.seed));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto logger = std::make_shared<NoopLogger>();
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(2.0, 2.0, 5.0);
	auto travel = std::make_shared<LinearTravelTimeStrategy>(5.0);
	Simulator sim(1, "repro", s.floors, s.elevators, s.customers, s.rate, 1.0f, true,
		rng, dispatch, metrics, logger, std::nullopt,
		std::numeric_limits<double>::infinity(), nullptr, 70.0, 15.0,
		dwell, travel, nullptr, group);
	sim.startSimulation();
	return Result{metrics->getAverageWaitingTime(), metrics->getAverageTravelTime(),
		metrics->getTotalEnergyConsumption(), metrics->getWaitingTimeDistribution().p90};
}

} // namespace

// The paper's own comparison is GAHCA vs THV; the project's greedy is kept as a
// third reference. All three run the SAME environment (linear 5s/floor + Cortes
// dwell), so the only difference is the allocation. GAHCA runs as a group
// strategy; THV and greedy are per-call IDispatchStrategies behind the default
// adapter.
TEST_CASE("GAHCA vs THV vs greedy, Cortes reproduction environment", "[.gahca_repro]") {
	const Scenario scenarios[] = {
		{10, 3, 50, 0.8, 42},
		{15, 2, 200, 2.0, 99},
		{20, 4, 400, 3.0, 7},
	};
	std::ostringstream report;
	report << "\n=== GAHCA vs THV vs greedy (linear 5s/floor + dwell 2/5/2), uniform traffic ===\n";
	for (const Scenario& s : scenarios) {
		auto greedy = std::make_shared<GreedyNearestDispatchStrategy>();
		Result g = reproRun(s, greedy, nullptr);									// greedy baseline
		Result t = reproRun(s, std::make_shared<ThvDispatchStrategy>(s.floors), nullptr);	// THV baseline (the paper's)
		Result h = reproRun(s, greedy, std::make_shared<GahcaDispatchStrategy>());	// GAHCA (group)
		report << s.floors << "/" << s.elevators << "/" << s.customers << " rate " << s.rate
			<< "   (waiting / in-cabin travel / p90 waiting / energy)\n"
			<< "   greedy: " << g.awt << " / " << g.ajt << " / " << g.p90 << " / " << g.energy << "\n"
			<< "   THV   : " << t.awt << " / " << t.ajt << " / " << t.p90 << " / " << t.energy << "\n"
			<< "   GAHCA : " << h.awt << " / " << h.ajt << " / " << h.p90 << " / " << h.energy << "\n";
	}
	WARN(report.str());
	SUCCEED();
}
