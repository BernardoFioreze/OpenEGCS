#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include "RunConfig.h"

using namespace openEGCS;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

// Unit tests for the JSON config loader. These are possible because RunConfig
// lives in liftsim_core rather than in main() - the parser can be exercised
// directly instead of only through the binary. The byte-identity-with-the-
// positional-CLI claim is a separate CLI test (tests/CMakeLists.txt); here we
// pin the parsing rules themselves.

namespace {

// Write `contents` to a throwaway file and return its path. Named uniquely so
// the cases don't collide; removed by the OS temp sweep (small and harmless).
std::string tempConfig(const std::string& contents, const std::string& tag) {
	std::filesystem::path p = std::filesystem::temp_directory_path() /
		("liftsim_cfg_" + tag + ".json");
	std::ofstream(p) << contents;
	return p.string();
}

} // namespace

TEST_CASE("a minimal config leaves every optional field at the positional default", "[run_config]") {
	RunConfig cfg = loadRunConfig(tempConfig(R"({"floors":10,"elevators":3,"customers":50})", "minimal"));

	REQUIRE(cfg.floors == 10);
	REQUIRE(cfg.elevators == 3);
	REQUIRE(cfg.customers == 50);
	REQUIRE(cfg.arrivalRate == Approx(0.5));
	REQUIRE(cfg.sane == true);
	REQUIRE_FALSE(cfg.seed.has_value());
	REQUIRE(cfg.maxWeightKg == std::numeric_limits<double>::infinity());
	REQUIRE(cfg.meanWeightKg == Approx(70.0));
	REQUIRE(cfg.stddevWeightKg == Approx(15.0));
	REQUIRE(cfg.bypassFraction == Approx(0.6));
	REQUIRE(cfg.floorHeightM == Approx(3.5));
	REQUIRE(cfg.carMassKg == Approx(1000.0));
	REQUIRE(cfg.counterweightMassKg == Approx(1300.0));
	// Item #56: an omitted door time is "use the preset", not 0.
	REQUIRE_FALSE(cfg.doorOpenTime.has_value());
	REQUIRE_FALSE(cfg.doorCloseTime.has_value());
	REQUIRE_FALSE(cfg.perPassengerTransferTime.has_value());
	REQUIRE_FALSE(cfg.waitingThreshold.has_value());
	REQUIRE_FALSE(cfg.journeyThreshold.has_value());
	REQUIRE(cfg.travelModel == TravelModel::Kinematic);
}

TEST_CASE("every named key is read through", "[run_config]") {
	RunConfig cfg = loadRunConfig(tempConfig(R"({
		"floors":20,"elevators":4,"customers":3000,
		"arrivalRate":0.8,"sane":false,"seed":42,
		"maxWeightKg":600,"meanWeightKg":75,"stddevWeightKg":10,"bypassFraction":0.5,
		"floorHeightM":3.0,"carMassKg":1200,"counterweightMassKg":1400,
		"doorOpenTime":2,"doorCloseTime":3,"perPassengerTransferTime":1,
		"waitingThreshold":30,"journeyThreshold":60,
		"elevatorSpeed":1.6,"travelModel":"linear"
	})", "full"));

	REQUIRE(cfg.floors == 20);
	REQUIRE(cfg.sane == false);
	REQUIRE(cfg.seed.has_value());
	REQUIRE(*cfg.seed == 42);
	REQUIRE(cfg.maxWeightKg == Approx(600.0));
	REQUIRE(cfg.bypassFraction == Approx(0.5));
	REQUIRE(cfg.doorCloseTime.has_value());
	REQUIRE(*cfg.doorCloseTime == Approx(3.0));
	REQUIRE(cfg.waitingThreshold.has_value());
	REQUIRE(*cfg.waitingThreshold == Approx(30.0));
	REQUIRE(cfg.elevatorSpeed == Approx(1.6));
	REQUIRE(cfg.travelModel == TravelModel::Linear);
}

TEST_CASE("an unknown key fails loudly and names itself", "[run_config]") {
	// The whole reason the file format beats positional defaulting: a typo must
	// not silently fall back to a default.
	REQUIRE_THROWS_WITH(
		loadRunConfig(tempConfig(R"({"floors":10,"elevators":3,"customers":50,"elevaters":4})", "unknown")),
		ContainsSubstring("elevaters"));
}

TEST_CASE("a missing required key is refused by name", "[run_config]") {
	REQUIRE_THROWS_WITH(
		loadRunConfig(tempConfig(R"({"elevators":3,"customers":50})", "missing")),
		ContainsSubstring("floors"));
}

TEST_CASE("a value of the wrong type is refused by name", "[run_config]") {
	REQUIRE_THROWS_WITH(
		loadRunConfig(tempConfig(R"({"floors":10,"elevators":3,"customers":50,"arrivalRate":"fast"})", "wrongtype")),
		ContainsSubstring("arrivalRate"));
}

TEST_CASE("a negative stddev is refused, matching the positional parser", "[run_config]") {
	REQUIRE_THROWS_WITH(
		loadRunConfig(tempConfig(R"({"floors":10,"elevators":3,"customers":50,"stddevWeightKg":-1})", "negstddev")),
		ContainsSubstring("stddevWeightKg"));
}

TEST_CASE("an unknown travelModel is refused", "[run_config]") {
	REQUIRE_THROWS_WITH(
		loadRunConfig(tempConfig(R"({"floors":10,"elevators":3,"customers":50,"travelModel":"magic"})", "badmodel")),
		ContainsSubstring("travelModel"));
}

TEST_CASE("the dwell preset selectors are read through", "[run_config][dwell_presets]") {
	RunConfig cfg = loadRunConfig(tempConfig(R"({
		"floors":10,"elevators":3,"customers":50,
		"doorType":"side","doorOpening":"advanced","transferProfile":"residential"
	})", "dwellsel"));
	REQUIRE(cfg.dwell.door == DoorType::Side);
	REQUIRE(cfg.dwell.opening == DoorOpening::Advanced);
	REQUIRE(cfg.dwell.transfer == TransferProfile::Residential);

	cfg = loadRunConfig(tempConfig(R"({
		"floors":10,"elevators":3,"customers":50,
		"doorType":"none","transferProfile":"ratedLoad"
	})", "dwellsel2"));
	REQUIRE(cfg.dwell.door == DoorType::None);
	REQUIRE(cfg.dwell.transfer == TransferProfile::RatedLoad);
}

TEST_CASE("an unknown dwell selector value is refused by name", "[run_config][dwell_presets]") {
	REQUIRE_THROWS_WITH(
		loadRunConfig(tempConfig(R"({"floors":10,"elevators":3,"customers":50,"doorType":"revolving"})", "baddoor")),
		ContainsSubstring("doorType") && ContainsSubstring("revolving"));
	REQUIRE_THROWS_WITH(
		loadRunConfig(tempConfig(R"({"floors":10,"elevators":3,"customers":50,"doorOpening":2})", "badopening")),
		ContainsSubstring("doorOpening"));
	REQUIRE_THROWS_WITH(
		loadRunConfig(tempConfig(R"({"floors":10,"elevators":3,"customers":50,"transferProfile":"hospital"})", "badtp")),
		ContainsSubstring("transferProfile"));
}

TEST_CASE("dwellTimesOf resolves the preset for the cabin and applies each explicit override",
	"[run_config][dwell_presets]") {
	// Centre door, 1600 kg cabin: 1100 mm (sec. 5.3.3.1), normal opening 2.5 s,
	// closing 3.0 s (Table 5.3); rated-load transfer 0.8 s (Table 5.7).
	RunConfig cfg = loadRunConfig(tempConfig(R"({
		"floors":10,"elevators":3,"customers":50,"maxWeightKg":1600,
		"doorType":"centre","transferProfile":"ratedLoad"
	})", "resolve"));
	DwellTimes t = dwellTimesOf(cfg);
	REQUIRE(t.doorOpenS == 2.5);
	REQUIRE(t.doorCloseS == 3.0);
	REQUIRE(t.perPassengerS == 0.8);

	// One override replaces one value and leaves the other two on the preset.
	cfg.doorCloseTime = 7.0;
	t = dwellTimesOf(cfg);
	REQUIRE(t.doorOpenS == 2.5);
	REQUIRE(t.doorCloseS == 7.0);
	REQUIRE(t.perPassengerS == 0.8);

	// An explicit 0 is an instantaneous door, not "use the preset".
	cfg.doorOpenTime = 0.0;
	REQUIRE(dwellTimesOf(cfg).doorOpenS == 0.0);
}

TEST_CASE("values are clamped exactly as the positional parser clamps them", "[run_config]") {
	RunConfig cfg = loadRunConfig(tempConfig(R"({
		"floors":10,"elevators":3,"customers":50,
		"arrivalRate":0,"maxWeightKg":-5,"bypassFraction":2,"floorHeightM":-1,
		"carMassKg":0,"doorOpenTime":-3,"waitingThreshold":0
	})", "clamp"));

	REQUIRE(cfg.arrivalRate == Approx(0.5));						// <= 0 -> default
	REQUIRE(cfg.maxWeightKg == std::numeric_limits<double>::infinity());	// <= 0 -> unlimited
	REQUIRE(cfg.bypassFraction == Approx(0.6));					// out of (0,1] -> default
	REQUIRE(cfg.floorHeightM == Approx(3.5));					// <= 0 -> default
	REQUIRE(cfg.carMassKg == Approx(1000.0));					// <= 0 -> default
	REQUIRE(cfg.doorOpenTime.has_value());						// given, so it overrides the preset...
	REQUIRE(*cfg.doorOpenTime == Approx(0.0));					// ...and < 0 -> 0
	REQUIRE_FALSE(cfg.waitingThreshold.has_value());			// <= 0 -> unset
}

TEST_CASE("a file that cannot be opened is reported, not silently defaulted", "[run_config]") {
	REQUIRE_THROWS_WITH(loadRunConfig("/no/such/liftsim/config.json"),
		ContainsSubstring("cannot open"));
}

TEST_CASE("invalid JSON is reported as such", "[run_config]") {
	REQUIRE_THROWS_WITH(loadRunConfig(tempConfig("{ not json", "badjson")),
		ContainsSubstring("JSON"));
}
