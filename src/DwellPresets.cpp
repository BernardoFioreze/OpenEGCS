#include <cmath>
#include <stdexcept>
#include <string>

#include "DwellPresets.h"
#include "DoorHoldDwellTimeStrategy.h"

namespace openEGCS {

namespace {

void requireCabinLoad(double ratedLoadKg, const char* who)
{
	// infinity is a legal cabin (the CLI's unlimited default); NaN and <= 0 are not.
	if (std::isnan(ratedLoadKg) || ratedLoadKg <= 0.0) {
		throw std::invalid_argument(std::string(who) + ": rated load must be > 0 kg (or unlimited)");
	}
}

} // namespace

DwellPreset DwellPreset::instantaneous()
{
	DwellPreset p;
	p.door = DoorType::None;
	return p;
}

int doorWidthMmFor(double ratedLoadKg)
{
	requireCabinLoad(ratedLoadKg, "doorWidthMmFor");
	// Barney & Al-Sharif sec. 5.3.3.1: 800 mm "up to 1,000 kg", 1,100 mm "over
	// 1,000 kg". The boundary is inclusive on the 800 side, as the text reads.
	return ratedLoadKg <= 1000.0 ? 800 : 1100;
}

DwellTimes doorTimes(DoorType door, int widthMm, DoorOpening opening)
{
	if (widthMm != 800 && widthMm != 1100) {
		throw std::invalid_argument("doorTimes: Table 5.3 only has 800 and 1100 mm doors, not "
			+ std::to_string(widthMm));
	}
	DwellTimes t;
	if (door == DoorType::None) return t;

	const bool wide = (widthMm == 1100);
	// Barney & Al-Sharif Table 5.3, "Typical door closing and opening times (s)
	// for stated door width". Columns: opening (advanced), opening (normal),
	// closing; each at 800 / 1100 mm.
	if (door == DoorType::Side) {
		t.doorOpenS = (opening == DoorOpening::Advanced) ? (wide ? 1.5 : 1.0) : (wide ? 3.0 : 2.5);
		t.doorCloseS = wide ? 4.0 : 3.0;
	} else {	// Centre
		t.doorOpenS = (opening == DoorOpening::Advanced) ? (wide ? 0.8 : 0.5) : (wide ? 2.5 : 2.0);
		t.doorCloseS = wide ? 3.0 : 2.0;
	}
	return t;
}

double transferTimeFor(TransferProfile profile, double ratedLoadKg)
{
	requireCabinLoad(ratedLoadKg, "transferTimeFor");
	switch (profile) {
		case TransferProfile::Office:
			return 1.2;		// sec. 5.5.1: "may be taken as 1.2 s (each way)"
		case TransferProfile::Residential:
			return 1.75;	// sec. 5.5.1, citing ISO 4190-6 for residential buildings
		case TransferProfile::RatedLoad:
			break;
	}
	// Table 5.7, average passenger transfer times for offices: 320-800 kg 1.2,
	// 1000 kg 1.0, 1275 kg 0.9, 1600-2500 kg 0.8. Read at the largest tabulated
	// load not above the cabin's (plan D3a). The lower figures for big cars are
	// averages that already discount double-file boarding through wide doors.
	if (ratedLoadKg >= 1600.0) return 0.8;
	if (ratedLoadKg >= 1275.0) return 0.9;
	if (ratedLoadKg >= 1000.0) return 1.0;
	return 1.2;
}

DwellTimes resolveDwellTimes(const DwellPreset& preset, double ratedLoadKg)
{
	if (preset.door == DoorType::None) {
		requireCabinLoad(ratedLoadKg, "resolveDwellTimes");
		return DwellTimes{};
	}
	DwellTimes t = doorTimes(preset.door, doorWidthMmFor(ratedLoadKg), preset.opening);
	t.perPassengerS = transferTimeFor(preset.transfer, ratedLoadKg);
	t.hold = preset.hold;
	if (t.hold != DoorHold::None) {
		t.carCallHoldS = kCarCallHoldS;
		t.landingCallHoldS = kLandingCallHoldS;
	}
	return t;
}

std::shared_ptr<IDwellTimeStrategy> makeDwellStrategy(const DwellTimes& t)
{
	const bool noDoorCycle = (t.doorOpenS == 0.0 && t.doorCloseS == 0.0);
	if (t.hold == DoorHold::None || noDoorCycle) {
		return std::make_shared<FixedDwellTimeStrategy>(t.doorOpenS, t.doorCloseS, t.perPassengerS);
	}
	DoorHoldDwellTimeStrategy::Params p;
	p.doorOpenS = t.doorOpenS;
	p.doorCloseS = t.doorCloseS;
	p.perPassengerS = t.perPassengerS;
	p.carCallHoldS = t.carCallHoldS;
	p.landingCallHoldS = t.landingCallHoldS;
	p.detector = (t.hold == DoorHold::Detector);
	return std::make_shared<DoorHoldDwellTimeStrategy>(p);
}

std::shared_ptr<IDwellTimeStrategy> makeDwellStrategy(const DwellPreset& preset, double ratedLoadKg)
{
	return makeDwellStrategy(resolveDwellTimes(preset, ratedLoadKg));
}

} // namespace openEGCS
