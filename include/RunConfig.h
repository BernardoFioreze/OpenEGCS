/*
RunConfig.h

The whole configuration of one simulation run, named rather than positional.
It is the single description of what a run *is* - the same surface the
Simulator constructor takes, gathered into one struct so there is one place to
read, document and validate it.

It lives in liftsim_core rather than in main.cpp because it is input plumbing,
not engine: it holds values and builds no strategy, injects nothing, and adds
no branch to ElevatorSystem/Elevator. Having it here keeps the config surface
documented in exactly one place and makes the JSON parsing unit-testable
(unknown key, wrong type, defaults, the linear-model round trip) rather than
only through the binary. Turning a RunConfig into the running simulation -
choosing GreedyNearest, the kinematic or linear travel model, the dwell and
capacity strategies - stays in main.cpp, because that is wiring and reaches into
liftsim_examples.

Defaults are exactly the positional CLI's defaults, and loadRunConfig clamps the
same way the positional parser does, so a config file and the equivalent
positional command produce a byte-identical snapshot (tests/test_run_config.cpp
and a CLI test).
*/

#ifndef __RUNCONFIG__
#define __RUNCONFIG__

#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "DwellPresets.h"

namespace openEGCS {

// Which ITravelTimeStrategy a run uses. Kinematic is the default; Linear is the
// constant-speed model, and the positional CLI has no slot to select it.
enum class TravelModel { Kinematic, Linear };

struct RunConfig {
	// Required (the positional CLI has no default for these three either).
	unsigned int floors = 0;
	unsigned int elevators = 0;
	unsigned int customers = 0;

	// Optional - every default below is the positional CLI's own default.
	double arrivalRate = 0.5;					// Poisson lambda, arrivals per second
	bool sane = true;							// whether users press the call button for the right direction
	std::optional<uint64_t> seed = std::nullopt;	// absent -> non-deterministic run

	double maxWeightKg = std::numeric_limits<double>::infinity();	// absent/<=0 -> unlimited (load bypass never fires)
	double meanWeightKg = 70.0;
	double stddevWeightKg = 15.0;				// 0 is legal (everyone weighs the mean); negative is an error
	double bypassFraction = 0.6;				// load-weighing bypass threshold, fraction of maxWeight

	double floorHeightM = 3.5;					// feeds the energy and kinematic models
	double carMassKg = 1000.0;
	double counterweightMassKg = 1300.0;

	// Stop duration. `dwell` picks a cited preset from Barney & Al-Sharif ch. 5
	// (DwellPresets.h); the door width and, for TransferProfile::RatedLoad, the
	// transfer time follow maxWeightKg. The three optionals below override one
	// value of the preset each when present - the positional CLI's arguments
	// 15-17 and the config file's doorOpenTime/doorCloseTime/
	// perPassengerTransferTime both land here, so "omitted" (use the preset)
	// and "given as 0" (an explicit instantaneous door) stay distinct. Read the
	// resolved values through dwellTimesOf().
	DwellPreset dwell;		// centre door, normal opening, office transfer
	std::optional<double> doorOpenTime = std::nullopt;				// dwell: part of the wait a boarder sees
	std::optional<double> doorCloseTime = std::nullopt;			// dwell: delays departure
	std::optional<double> perPassengerTransferTime = std::nullopt;	// dwell: per passenger in/out
	// The minimum time the doors stay open (sec. 5.5.2), overriding the
	// preset's 2.0 s (car call) / 3.0 s (landing call). Ignored when the
	// preset's hold is None.
	std::optional<double> carCallHoldTime = std::nullopt;
	std::optional<double> landingCallHoldTime = std::nullopt;

	std::optional<double> waitingThreshold = std::nullopt;	// dissatisfaction tolerances; absent -> no verdict
	std::optional<double> journeyThreshold = std::nullopt;

	// Only read when travelModel == Linear: time units (seconds) per floor for
	// the constant-speed model (the old <Elevator speed> argument's real
	// meaning). Ignored by the kinematic model, which derives leg time from the
	// motion profile and <Floor height>.
	double elevatorSpeed = 5.0;
	TravelModel travelModel = TravelModel::Kinematic;
};

// Read and validate a JSON config file into a RunConfig. Throws
// std::runtime_error whose message names the offending key on: a file that
// cannot be opened, JSON that does not parse or is not an object, an unknown key
// (a typo must fail loudly rather than silently fall back to a default), a value
// of the wrong JSON type, a missing required key (floors/elevators/customers), a
// negative stddevWeightKg, or a travelModel that is not "kinematic"/"linear". A
// value the positional CLI silently clamps to a default (arrivalRate <= 0,
// maxWeightKg <= 0, bypassFraction out of (0,1], a non-positive floor
// height/mass, a negative door time) is clamped identically here, and a
// threshold <= 0 means unset - so the run matches the positional one bit for
// bit.
RunConfig loadRunConfig(const std::string& path);

// The stop duration this run will use: cfg.dwell resolved for cfg.maxWeightKg
// (resolveDwellTimes in DwellPresets.h), then each of cfg.doorOpenTime,
// cfg.doorCloseTime and cfg.perPassengerTransferTime that is present replaces
// the matching value. The one place both front ends and the saturation warning
// read the dwell from, so the warning prices the same doors the run has.
DwellTimes dwellTimesOf(const RunConfig& cfg);

} // namespace openEGCS

#endif
