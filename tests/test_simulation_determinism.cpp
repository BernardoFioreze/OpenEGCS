#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include "Simulator.h"
#include "RandomProvider.h"
#include "StandardMetricsCollector.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// Runs a small deterministic simulation twice with the same seed and
// asserts the resulting metrics are identical - this is the core guarantee
// needed to compare different dispatch algorithms on the same scenario.
static double runAndGetAvgWait(uint64_t seed) {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();

	Simulator sim(1, "test", /*floors=*/10, /*elevators=*/2, /*customers=*/15,
		/*arrivalRate=*/1.0, /*speed=*/1.0f, /*sane=*/true,
		rng, dispatchStrategy, metrics);

	sim.startSimulation();
	return metrics->getAverageWaitingTime();
}

TEST_CASE("Same seed produces identical simulation metrics", "[determinism][e2e]") {
	double a = runAndGetAvgWait(123);
	double b = runAndGetAvgWait(123);
	REQUIRE(a == b);
}

TEST_CASE("Different seeds tend to produce different simulation metrics", "[determinism][e2e]") {
	double a = runAndGetAvgWait(1);
	double b = runAndGetAvgWait(2);
	REQUIRE(a != b);
}

// snapshot.json is the only output: no plain-text logs are produced.
TEST_CASE("Running a simulation does not create the legacy text logs", "[e2e][logs]") {
	std::filesystem::remove("EventLog.log");
	std::filesystem::remove("StatusLog.log");

	runAndGetAvgWait(123);

	REQUIRE_FALSE(std::filesystem::exists("EventLog.log"));
	REQUIRE_FALSE(std::filesystem::exists("StatusLog.log"));
}

// The real determinism claim is "same seed, same snapshot.json byte for
// byte". The case above only compares one average of one scenario, which is
// a much weaker statement: an energy over-count or unserved customers can
// leave it passing. This compares the artefact itself, which is also the
// gate any change claiming "no behaviour change" has to clear.
static std::string runAndReadSnapshot(uint64_t seed, const std::string& path) {
	std::filesystem::remove(path);

	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(seed));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto dispatchStrategy = std::make_shared<examples::GreedyNearestDispatchStrategy>();

	Simulator sim(1, "snapshot-determinism", /*floors=*/10, /*elevators=*/3, /*customers=*/40,
		/*arrivalRate=*/1.2, /*speed=*/1.0f, /*sane=*/true,
		rng, dispatchStrategy, metrics);

	sim.startSimulation();		// writes ./snapshot.json
	// startSimulation() always writes the default path, so the run is copied
	// out to `path` before the next one overwrites it.
	std::filesystem::copy_file("snapshot.json", path,
		std::filesystem::copy_options::overwrite_existing);

	std::ifstream in(path, std::ios::binary);
	REQUIRE(in.good());
	std::ostringstream buffer;
	buffer << in.rdbuf();
	return buffer.str();
}

TEST_CASE("Same seed produces a byte-identical snapshot", "[determinism][e2e][snapshot]") {
	std::string first = runAndReadSnapshot(2024, "determinism_first.json");
	std::string second = runAndReadSnapshot(2024, "determinism_second.json");

	REQUIRE_FALSE(first.empty());
	REQUIRE(first.size() == second.size());
	REQUIRE(first == second);

	// A different seed must actually produce a different artefact, otherwise
	// the comparison above would pass for a logger that writes a constant.
	std::string other = runAndReadSnapshot(2025, "determinism_other.json");
	REQUIRE(other != first);

	std::filesystem::remove("determinism_first.json");
	std::filesystem::remove("determinism_second.json");
	std::filesystem::remove("determinism_other.json");
}
