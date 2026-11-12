/*
RunConfig.cpp

JSON -> RunConfig. The contract (what throws, what clamps) is on the header; this
file is the transcription of it. The one rule worth stating twice: the clamps
here must stay identical to src/main.cpp's positional parser, because the item's
correctness test is that a config and the equivalent positional command produce
a byte-identical snapshot - if the two parsers ever disagree on a clamp, that
test is what catches it.
*/

#include "RunConfig.h"

#include <algorithm>
#include <fstream>
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace openEGCS {

namespace {

// Every key the loader understands. A key outside this set is a typo until
// proven otherwise, and failing loudly on it is the whole reason the file format
// beats silent positional defaulting.
const std::set<std::string> kKnownKeys = {
	"floors", "elevators", "customers",
	"arrivalRate", "sane", "seed",
	"maxWeightKg", "meanWeightKg", "stddevWeightKg", "bypassFraction",
	"floorHeightM", "carMassKg", "counterweightMassKg",
	"doorOpenTime", "doorCloseTime", "perPassengerTransferTime",
	"doorType", "doorOpening", "transferProfile",
	"doorHold", "carCallHoldTime", "landingCallHoldTime",
	"waitingThreshold", "journeyThreshold",
	"elevatorSpeed", "travelModel",
};

[[noreturn]] void fail(const std::string& msg) {
	throw std::runtime_error("config: " + msg);
}

// A number of any kind (int or float); the clamps downstream decide what a legal
// value is. Named in the error so the user knows which key to fix.
double requireNumber(const json& obj, const std::string& key) {
	const json& v = obj.at(key);
	if (!v.is_number()) fail("key \"" + key + "\" must be a number");
	return v.get<double>();
}

// A string that must be one of `choices`; the error names the key and lists them.
std::string requireChoice(const json& obj, const std::string& key, std::initializer_list<const char*> choices) {
	std::string list;
	for (const char* c : choices) list += (list.empty() ? "" : ", ") + std::string("\"") + c + "\"";
	const json& v = obj.at(key);
	if (!v.is_string()) fail("key \"" + key + "\" must be one of " + list);
	const std::string s = v.get<std::string>();
	for (const char* c : choices) if (s == c) return s;
	fail("key \"" + key + "\" must be one of " + list + ", not \"" + s + "\"");
}

} // namespace

RunConfig loadRunConfig(const std::string& path) {
	std::ifstream file(path);
	if (!file) fail("cannot open config file: " + path);

	json root;
	try {
		root = json::parse(file);
	} catch (const json::parse_error& e) {
		fail("invalid JSON in " + path + ": " + e.what());
	}
	if (!root.is_object()) fail("top level of " + path + " must be a JSON object");

	for (auto it = root.begin(); it != root.end(); ++it) {
		if (kKnownKeys.find(it.key()) == kKnownKeys.end()) {
			fail("unknown key \"" + it.key() + "\" in " + path);
		}
	}

	RunConfig cfg;

	// Required trio. The positional CLI parses these as unsigned and has no
	// default, so their absence is an error here too.
	for (const char* key : {"floors", "elevators", "customers"}) {
		if (!root.contains(key)) fail(std::string("missing required key \"") + key + "\"");
		const json& v = root.at(key);
		if (!v.is_number_unsigned() && !(v.is_number_integer() && v.get<long long>() >= 0)) {
			fail(std::string("key \"") + key + "\" must be a non-negative integer");
		}
	}
	cfg.floors = root.at("floors").get<unsigned int>();
	cfg.elevators = root.at("elevators").get<unsigned int>();
	cfg.customers = root.at("customers").get<unsigned int>();

	// Everything below is optional: absent leaves the default, present is read
	// and then clamped exactly as main.cpp clamps the positional argument.
	if (root.contains("arrivalRate")) {
		cfg.arrivalRate = requireNumber(root, "arrivalRate");
		if (cfg.arrivalRate <= 0.0) cfg.arrivalRate = 0.5;
	}
	if (root.contains("sane")) {
		const json& v = root.at("sane");
		if (!v.is_boolean()) fail("key \"sane\" must be a boolean");
		cfg.sane = v.get<bool>();
	}
	if (root.contains("seed")) {
		const json& v = root.at("seed");
		if (!v.is_number_unsigned() && !(v.is_number_integer() && v.get<long long>() >= 0)) {
			fail("key \"seed\" must be a non-negative integer");
		}
		cfg.seed = v.get<uint64_t>();
	}

	if (root.contains("maxWeightKg")) {
		cfg.maxWeightKg = requireNumber(root, "maxWeightKg");
		if (cfg.maxWeightKg <= 0.0) cfg.maxWeightKg = std::numeric_limits<double>::infinity();
	}
	if (root.contains("meanWeightKg")) {
		cfg.meanWeightKg = requireNumber(root, "meanWeightKg");
		if (cfg.meanWeightKg <= 0.0) cfg.meanWeightKg = 70.0;
	}
	if (root.contains("stddevWeightKg")) {
		cfg.stddevWeightKg = requireNumber(root, "stddevWeightKg");
		// A negative stddev means nothing and is refused by name, matching the
		// positional parser; 0 is legal and means everyone weighs the mean.
		if (cfg.stddevWeightKg < 0.0) fail("key \"stddevWeightKg\" must be >= 0 (0 = every passenger weighs the mean)");
	}
	if (root.contains("bypassFraction")) {
		cfg.bypassFraction = requireNumber(root, "bypassFraction");
		if (cfg.bypassFraction <= 0.0 || cfg.bypassFraction > 1.0) cfg.bypassFraction = 0.6;
	}

	if (root.contains("floorHeightM")) {
		cfg.floorHeightM = requireNumber(root, "floorHeightM");
		if (cfg.floorHeightM <= 0.0) cfg.floorHeightM = 3.5;
	}
	if (root.contains("carMassKg")) {
		cfg.carMassKg = requireNumber(root, "carMassKg");
		if (cfg.carMassKg <= 0.0) cfg.carMassKg = 1000.0;
	}
	if (root.contains("counterweightMassKg")) {
		cfg.counterweightMassKg = requireNumber(root, "counterweightMassKg");
		if (cfg.counterweightMassKg <= 0.0) cfg.counterweightMassKg = 1300.0;
	}

	// The dwell preset. Each selector is a string from a closed set, refused
	// by name otherwise, like travelModel.
	if (root.contains("doorType")) {
		const std::string v = requireChoice(root, "doorType", {"centre", "side", "none"});
		cfg.dwell.door = (v == "centre") ? DoorType::Centre : (v == "side") ? DoorType::Side : DoorType::None;
	}
	if (root.contains("doorOpening")) {
		const std::string v = requireChoice(root, "doorOpening", {"normal", "advanced"});
		cfg.dwell.opening = (v == "normal") ? DoorOpening::Normal : DoorOpening::Advanced;
	}
	if (root.contains("transferProfile")) {
		const std::string v = requireChoice(root, "transferProfile", {"office", "residential", "ratedLoad"});
		cfg.dwell.transfer = (v == "office") ? TransferProfile::Office
			: (v == "residential") ? TransferProfile::Residential : TransferProfile::RatedLoad;
	}

	if (root.contains("doorHold")) {
		const std::string v = requireChoice(root, "doorHold", {"timed", "detector", "none"});
		cfg.dwell.hold = (v == "timed") ? DoorHold::Timed : (v == "detector") ? DoorHold::Detector : DoorHold::None;
	}

	// An explicit time overrides the preset's value. A negative one is clamped
	// to 0, as the positional parser does.
	if (root.contains("doorOpenTime")) {
		cfg.doorOpenTime = std::max(0.0, requireNumber(root, "doorOpenTime"));
	}
	if (root.contains("doorCloseTime")) {
		cfg.doorCloseTime = std::max(0.0, requireNumber(root, "doorCloseTime"));
	}
	if (root.contains("perPassengerTransferTime")) {
		cfg.perPassengerTransferTime = std::max(0.0, requireNumber(root, "perPassengerTransferTime"));
	}
	if (root.contains("carCallHoldTime")) {
		cfg.carCallHoldTime = std::max(0.0, requireNumber(root, "carCallHoldTime"));
	}
	if (root.contains("landingCallHoldTime")) {
		cfg.landingCallHoldTime = std::max(0.0, requireNumber(root, "landingCallHoldTime"));
	}

	// A threshold is set only when strictly positive; <= 0 (or absent) is unset,
	// which reports no verdict rather than a verdict of zero.
	if (root.contains("waitingThreshold")) {
		double t = requireNumber(root, "waitingThreshold");
		if (t > 0.0) cfg.waitingThreshold = t;
	}
	if (root.contains("journeyThreshold")) {
		double t = requireNumber(root, "journeyThreshold");
		if (t > 0.0) cfg.journeyThreshold = t;
	}

	if (root.contains("elevatorSpeed")) {
		cfg.elevatorSpeed = requireNumber(root, "elevatorSpeed");
		if (cfg.elevatorSpeed < 0.0) cfg.elevatorSpeed = 5.0;
	}
	if (root.contains("travelModel")) {
		const json& v = root.at("travelModel");
		if (!v.is_string()) fail("key \"travelModel\" must be \"kinematic\" or \"linear\"");
		const std::string model = v.get<std::string>();
		if (model == "kinematic") cfg.travelModel = TravelModel::Kinematic;
		else if (model == "linear") cfg.travelModel = TravelModel::Linear;
		else fail("key \"travelModel\" must be \"kinematic\" or \"linear\", not \"" + model + "\"");
	}

	return cfg;
}

DwellTimes dwellTimesOf(const RunConfig& cfg) {
	DwellTimes t = resolveDwellTimes(cfg.dwell, cfg.maxWeightKg);
	if (cfg.doorOpenTime) t.doorOpenS = *cfg.doorOpenTime;
	if (cfg.doorCloseTime) t.doorCloseS = *cfg.doorCloseTime;
	if (cfg.perPassengerTransferTime) t.perPassengerS = *cfg.perPassengerTransferTime;
	if (t.hold != DoorHold::None) {
		if (cfg.carCallHoldTime) t.carCallHoldS = *cfg.carCallHoldTime;
		if (cfg.landingCallHoldTime) t.landingCallHoldS = *cfg.landingCallHoldTime;
	}
	return t;
}

} // namespace openEGCS
