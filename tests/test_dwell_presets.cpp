#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "DwellPresets.h"
#include "DoorHoldDwellTimeStrategy.h"

using namespace openEGCS;

// Every expected value here is read straight off Barney & Al-Sharif (2015),
// ch. 5 - the test is the transcription check, so a typo in a table cell
// fails by name. Compared with == on purpose: these are table cells, not
// computations.

namespace {
const double kUnlimited = std::numeric_limits<double>::infinity();
}

TEST_CASE("door width follows the rated load (sec. 5.3.3.1)", "[dwell_presets]") {
	REQUIRE(doorWidthMmFor(320.0) == 800);
	REQUIRE(doorWidthMmFor(1000.0) == 800);		// "up to 1,000 kg" is inclusive
	REQUIRE(doorWidthMmFor(1000.5) == 1100);	// "over 1,000 kg"
	REQUIRE(doorWidthMmFor(2500.0) == 1100);
	REQUIRE(doorWidthMmFor(kUnlimited) == 1100);	// an unlimited cabin is "over 1,000 kg"
	REQUIRE_THROWS_AS(doorWidthMmFor(0.0), std::invalid_argument);
	REQUIRE_THROWS_AS(doorWidthMmFor(-1.0), std::invalid_argument);
	REQUIRE_THROWS_AS(doorWidthMmFor(std::nan("")), std::invalid_argument);
}

TEST_CASE("door times are Table 5.3, cell by cell", "[dwell_presets]") {
	// Side opening
	REQUIRE(doorTimes(DoorType::Side, 800, DoorOpening::Advanced).doorOpenS == 1.0);
	REQUIRE(doorTimes(DoorType::Side, 1100, DoorOpening::Advanced).doorOpenS == 1.5);
	REQUIRE(doorTimes(DoorType::Side, 800, DoorOpening::Normal).doorOpenS == 2.5);
	REQUIRE(doorTimes(DoorType::Side, 1100, DoorOpening::Normal).doorOpenS == 3.0);
	REQUIRE(doorTimes(DoorType::Side, 800, DoorOpening::Normal).doorCloseS == 3.0);
	REQUIRE(doorTimes(DoorType::Side, 1100, DoorOpening::Normal).doorCloseS == 4.0);
	// Centre opening
	REQUIRE(doorTimes(DoorType::Centre, 800, DoorOpening::Advanced).doorOpenS == 0.5);
	REQUIRE(doorTimes(DoorType::Centre, 1100, DoorOpening::Advanced).doorOpenS == 0.8);
	REQUIRE(doorTimes(DoorType::Centre, 800, DoorOpening::Normal).doorOpenS == 2.0);
	REQUIRE(doorTimes(DoorType::Centre, 1100, DoorOpening::Normal).doorOpenS == 2.5);
	REQUIRE(doorTimes(DoorType::Centre, 800, DoorOpening::Normal).doorCloseS == 2.0);
	REQUIRE(doorTimes(DoorType::Centre, 1100, DoorOpening::Normal).doorCloseS == 3.0);
	// Closing does not depend on the opening mode: advanced opening overlaps
	// levelling, it does not change how the doors shut.
	REQUIRE(doorTimes(DoorType::Centre, 1100, DoorOpening::Advanced).doorCloseS == 3.0);
	REQUIRE(doorTimes(DoorType::Side, 800, DoorOpening::Advanced).doorCloseS == 3.0);
	// A door time never carries a transfer time.
	REQUIRE(doorTimes(DoorType::Centre, 800, DoorOpening::Normal).perPassengerS == 0.0);
}

TEST_CASE("door times refuse a width the table does not have", "[dwell_presets]") {
	REQUIRE_THROWS_AS(doorTimes(DoorType::Centre, 900, DoorOpening::Normal), std::invalid_argument);
	REQUIRE_THROWS_AS(doorTimes(DoorType::Side, 1300, DoorOpening::Normal), std::invalid_argument);
}

TEST_CASE("DoorType::None is the instantaneous door at any width", "[dwell_presets]") {
	DwellTimes t = doorTimes(DoorType::None, 1100, DoorOpening::Normal);
	REQUIRE(t.doorOpenS == 0.0);
	REQUIRE(t.doorCloseS == 0.0);
}

TEST_CASE("transfer profiles: office and residential are fixed (sec. 5.5.1)", "[dwell_presets]") {
	REQUIRE(transferTimeFor(TransferProfile::Office, 630.0) == 1.2);
	REQUIRE(transferTimeFor(TransferProfile::Office, kUnlimited) == 1.2);
	REQUIRE(transferTimeFor(TransferProfile::Residential, 630.0) == 1.75);
	REQUIRE(transferTimeFor(TransferProfile::Residential, 2000.0) == 1.75);
}

TEST_CASE("transfer by rated load is Table 5.7, read at the row not above the cabin", "[dwell_presets]") {
	// The table's own rows
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 320.0) == 1.2);
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 800.0) == 1.2);
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 1000.0) == 1.0);
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 1275.0) == 0.9);
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 1600.0) == 0.8);
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 2500.0) == 0.8);
	// Plan decision D3a: between rows, the row below; outside, the nearest end.
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 900.0) == 1.2);
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 1100.0) == 1.0);
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 1500.0) == 0.9);
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 200.0) == 1.2);
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, 4000.0) == 0.8);
	REQUIRE(transferTimeFor(TransferProfile::RatedLoad, kUnlimited) == 0.8);
	REQUIRE_THROWS_AS(transferTimeFor(TransferProfile::RatedLoad, 0.0), std::invalid_argument);
}

TEST_CASE("the default preset is centre, normal opening, office transfer", "[dwell_presets]") {
	DwellPreset p;
	REQUIRE(p.door == DoorType::Centre);
	REQUIRE(p.opening == DoorOpening::Normal);
	REQUIRE(p.transfer == TransferProfile::Office);

	// Unlimited cabin (the CLI default): 1100 mm centre door, 2.5 / 3.0 s, 1.2 s.
	DwellTimes big = resolveDwellTimes(p, kUnlimited);
	REQUIRE(big.doorOpenS == 2.5);
	REQUIRE(big.doorCloseS == 3.0);
	REQUIRE(big.perPassengerS == 1.2);

	// 600 kg cabin: 800 mm, 2.0 / 2.0 s; office transfer is still 1.2 s.
	DwellTimes small = resolveDwellTimes(p, 600.0);
	REQUIRE(small.doorOpenS == 2.0);
	REQUIRE(small.doorCloseS == 2.0);
	REQUIRE(small.perPassengerS == 1.2);
}

TEST_CASE("the instantaneous preset is all zeros, whatever its transfer profile says", "[dwell_presets]") {
	DwellPreset p = DwellPreset::instantaneous();
	p.transfer = TransferProfile::Residential;
	DwellTimes t = resolveDwellTimes(p, 800.0);
	REQUIRE(t.doorOpenS == 0.0);
	REQUIRE(t.doorCloseS == 0.0);
	REQUIRE(t.perPassengerS == 0.0);
	auto fixed = std::dynamic_pointer_cast<FixedDwellTimeStrategy>(makeDwellStrategy(p, 800.0));
	REQUIRE(fixed != nullptr);	// the instantaneous stop has no hold, so it is the plain fixed strategy
	REQUIRE(fixed->isZero());
}

TEST_CASE("makeDwellStrategy holds the resolved values", "[dwell_presets]") {
	DwellPreset p;
	p.door = DoorType::Side;
	p.opening = DoorOpening::Advanced;
	auto s = makeDwellStrategy(p, 1275.0);	// 1100 mm side, advanced: 1.5 open, 4.0 close
	StopContext stop;
	stop.boarding = 2;
	stop.alighting = 1;
	REQUIRE(s->getDoorOpenTime(stop) == 1.5);
	REQUIRE(s->getDoorCloseTime(stop) == 4.0);
	REQUIRE(s->getTransferTime(stop) == 3 * 1.2);
}

// ---------------------------------------------------------------------------
// Item #57: the door hold (sec. 5.5.2) as part of a preset.

TEST_CASE("a hold carries the sec. 5.5.2 figures and builds the door-hold strategy", "[dwell_presets][door_hold]") {
	DwellPreset p;
	p.hold = DoorHold::Timed;
	DwellTimes t = resolveDwellTimes(p, kUnlimited);
	REQUIRE(t.hold == DoorHold::Timed);
	REQUIRE(t.carCallHoldS == 2.0);
	REQUIRE(t.landingCallHoldS == 3.0);
	REQUIRE(std::dynamic_pointer_cast<DoorHoldDwellTimeStrategy>(makeDwellStrategy(p, kUnlimited)) != nullptr);

	p.hold = DoorHold::None;
	DwellTimes none = resolveDwellTimes(p, kUnlimited);
	REQUIRE(none.carCallHoldS == 0.0);
	REQUIRE(std::dynamic_pointer_cast<FixedDwellTimeStrategy>(makeDwellStrategy(p, kUnlimited)) != nullptr);
}

TEST_CASE("no door cycle means no door to hold open", "[dwell_presets][door_hold]") {
	// The instantaneous preset ignores the hold field...
	DwellPreset p = DwellPreset::instantaneous();
	p.hold = DoorHold::Timed;
	auto instant = std::dynamic_pointer_cast<FixedDwellTimeStrategy>(makeDwellStrategy(p, 800.0));
	REQUIRE(instant != nullptr);
	REQUIRE(instant->isZero());

	// ...and explicit zero door times (the positional `0 0 0`) drop it too,
	// even with a transfer time, so they stay the instantaneous stop.
	DwellTimes t;
	t.hold = DoorHold::Timed;
	t.carCallHoldS = 2.0;
	t.landingCallHoldS = 3.0;
	t.perPassengerS = 1.2;
	REQUIRE(std::dynamic_pointer_cast<FixedDwellTimeStrategy>(makeDwellStrategy(t)) != nullptr);
}
