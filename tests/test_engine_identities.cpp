#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "DwellPresets.h"
#include "FixedDwellTimeStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "ISimulationLogger.h"
#include "KinematicTravelTimeStrategy.h"
#include "LiftKinematics.h"
#include "LiftMotorModel.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "StandardMetricsCollector.h"
#include "ThvDispatchStrategy.h"

using namespace openEGCS;

// The timeline checked against independent sources. A timeline can be
// coherent with itself and still wrong - a stop whose length does not match
// the people it carried, for instance - so this file asks of the timeline
// only what an independent source can answer: the travel model, the dwell
// model, the customers' own floors. It runs over a grid of door, capacity,
// demand and dispatcher, because engine defects of this kind tend to show up
// in a regime hand-picked scenarios do not reach. It carries no expected
// value, so nothing here goes stale.

namespace {

struct Row {
	double time;
	std::string type;
	int cabin;
	unsigned long int customer;
	int floor;
};

struct Passenger {
	int at;
	int to;
	double arrival;
	double weight;
};

struct Timeline : public ISimulationLogger {
	std::vector<Row> rows;
	std::map<unsigned long int, Passenger> passengers;
	void setMetadata(const SimulationMetadata&) override {}
	void logCustomer(const Customer& c) override {
		passengers[c.getId()] = Passenger{c.getAt(), c.getTo(), c.getTime(), c.getWeight()};
	}
	void logEvent(double time, const std::string& type, int cabin, unsigned long int customer, int floor) override {
		rows.push_back(Row{time, type, cabin, customer, floor});
	}
	void finalize(const IMetricsCollector&, const std::string&) override {}
};

enum class Door { None, Fixed, DefaultHold };

struct Cell {
	Door door;
	double maxWeight;
	double rate;
	bool thv;
	uint64_t seed;
};

const int kFloors = 12;
const int kCars = 4;
const int kCustomers = 250;
const double kHeight = 3.5;
const double kOpen = 2.0, kClose = 3.0, kTransfer = 1.2;

LiftDynamics dynamics()
{
	return defaultDynamicsFor(kFloors * kHeight);
}

std::shared_ptr<Timeline> run(const Cell& c)
{
	auto timeline = std::make_shared<Timeline>();
	std::shared_ptr<IDwellTimeStrategy> dwell;	// null: the engine's default, the handbook door with its hold
	if (c.door == Door::None) dwell = std::make_shared<FixedDwellTimeStrategy>(0.0, 0.0, 0.0);
	if (c.door == Door::Fixed) dwell = std::make_shared<FixedDwellTimeStrategy>(kOpen, kClose, kTransfer);
	std::shared_ptr<IDispatchStrategy> dispatch;
	if (c.thv) dispatch = std::make_shared<examples::ThvDispatchStrategy>(kFloors + 1);
	else dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();

	std::streambuf* saved = std::cout.rdbuf();
	std::ostringstream sink;
	std::cout.rdbuf(sink.rdbuf());
	Simulator sim(c.seed, "identities", kFloors, kCars, kCustomers, c.rate, 1.0f, true,
		std::make_shared<RandomProvider>(std::optional<uint64_t>(c.seed)), dispatch,
		std::make_shared<StandardMetricsCollector>(1.0, kHeight, dynamics()), timeline,
		std::nullopt, c.maxWeight, nullptr, 70.0, 15.0, dwell,
		std::make_shared<KinematicTravelTimeStrategy>(kHeight, dynamics()));
	sim.startSimulation();
	std::cout.rdbuf(saved);
	return timeline;
}

std::vector<Cell> grid()
{
	std::vector<Cell> cells;
	for (Door door : {Door::None, Door::Fixed, Door::DefaultHold}) {
		for (double weight : {std::numeric_limits<double>::infinity(), 1000.0, 300.0}) {
			for (double rate : {0.2, 2.0}) {
				for (bool thv : {false, true}) {
					for (uint64_t seed : {uint64_t(1), uint64_t(2)}) {
						cells.push_back(Cell{door, weight, rate, thv, seed});
					}
				}
			}
		}
	}
	return cells;
}

const std::vector<std::pair<Cell, std::shared_ptr<Timeline>>>& runs()
{
	static std::vector<std::pair<Cell, std::shared_ptr<Timeline>>> all;
	if (all.empty()) {
		for (const Cell& c : grid()) all.push_back({c, run(c)});
	}
	return all;
}

#define CAPTURE_CELL(c) \
	int door_ = static_cast<int>((c).door); bool thv_ = (c).thv; double weight_ = (c).maxWeight; \
	double rate_ = (c).rate; uint64_t seed_ = (c).seed; \
	CAPTURE(door_, weight_, rate_, thv_, seed_)

} // namespace

TEST_CASE("Every passenger boards at their floor and alights at their destination", "[identities]") {
	for (const auto& r : runs()) {
		CAPTURE_CELL(r.first);
		std::map<unsigned long int, int> boarded, alighted;
		for (const Row& row : r.second->rows) {
			if (row.type != "ENTER" && row.type != "LEAVE") continue;
			const Passenger& p = r.second->passengers.at(row.customer);
			if (row.type == "ENTER") { REQUIRE(row.floor == p.at); boarded[row.customer]++; }
			else { REQUIRE(row.floor == p.to); alighted[row.customer]++; }
		}
		REQUIRE(boarded.size() == r.second->passengers.size());
		REQUIRE(alighted.size() == r.second->passengers.size());
	}
}

TEST_CASE("Nobody transfers at a floor their cabin is not standing at", "[identities]") {
	for (const auto& r : runs()) {
		CAPTURE_CELL(r.first);
		std::map<int, Row> lastArrive;
		for (const Row& row : r.second->rows) {
			if (row.type == "ARRIVE") { lastArrive[row.cabin] = row; continue; }
			if (row.type != "ENTER" && row.type != "LEAVE" && row.type != "PASS") continue;
			REQUIRE(lastArrive.count(row.cabin) == 1);
			REQUIRE(lastArrive[row.cabin].floor == row.floor);
			REQUIRE(row.time == lastArrive[row.cabin].time);
		}
	}
}

TEST_CASE("A cabin leaves from the floor it last arrived at", "[identities]") {
	for (const auto& r : runs()) {
		CAPTURE_CELL(r.first);
		std::map<int, int> at;
		for (const Row& row : r.second->rows) {
			if (row.type == "ARRIVE") at[row.cabin] = row.floor;
			else if (row.type == "DEPART") {
				const int from = at.count(row.cabin) ? at[row.cabin] : 0;
				CAPTURE(row.cabin, row.time);
				REQUIRE(row.floor == from);
			}
		}
	}
}

TEST_CASE("Every instant of a cabin's timeline moves forward", "[identities]") {
	for (const auto& r : runs()) {
		CAPTURE_CELL(r.first);
		std::map<int, double> last;
		for (const Row& row : r.second->rows) {
			if (row.type != "ARRIVE" && row.type != "DEPART" && row.type != "ENTER" && row.type != "LEAVE"
				&& row.type != "PASS") continue;	// RECALL names the refusing cabin, at the passenger's own instant
			CAPTURE(row.type, row.cabin, row.time);
			if (last.count(row.cabin)) REQUIRE(row.time >= last[row.cabin] - 1e-9);
			last[row.cabin] = row.time;
		}
	}
}

// Each leg: from the DEPART to its ARRIVE, in the order they come.
template <typename Check>
void forEachLeg(const Timeline& timeline, Check check)
{
	std::map<int, Row> departed;
	for (const Row& row : timeline.rows) {
		if (row.type == "DEPART") { departed[row.cabin] = row; continue; }
		if (row.type != "ARRIVE" || departed.count(row.cabin) == 0) continue;
		check(departed[row.cabin], row);
		departed.erase(row.cabin);
	}
}

TEST_CASE("A leg takes what the travel model says, and the door opening on top", "[identities]") {
	// A stop's arrival must not be re-derived from "now" once the leg is over:
	// anything that touches the cabin's route while its doors are opening would
	// then start the opening again, always late, never early. The timeline would
	// stay coherent with itself, so only the model can tell.
	const KinematicTravelTimeStrategy travel(kHeight, dynamics());
	for (const auto& r : runs()) {
		if (r.first.door == Door::DefaultHold) continue;	// the preset's door depends on the rated load
		CAPTURE_CELL(r.first);
		const double open = r.first.door == Door::None ? 0.0 : kOpen;
		forEachLeg(*r.second, [&](const Row& d, const Row& a) {
			if (a.floor == d.floor) return;
			CAPTURE(a.time, d.time, d.floor, a.floor);
			REQUIRE(a.time - d.time == Catch::Approx(travel.legDuration(a.floor - d.floor) + open).margin(1e-6));
		});
	}
}

TEST_CASE("A stop at the floor a cabin is standing at costs the door and nothing else", "[identities]") {
	// A call for the floor of a cabin whose doors are closing, or of a parked one,
	// gives a leg of zero floors: a DEPART with no movement, then the doors open.
	// The cabin never left. The margin is a position a rounding away from the
	// floor, which a jerk-limited profile prices at tens of microseconds.
	for (const auto& r : runs()) {
		if (r.first.door == Door::DefaultHold) continue;
		CAPTURE_CELL(r.first);
		const double open = r.first.door == Door::None ? 0.0 : kOpen;
		forEachLeg(*r.second, [&](const Row& d, const Row& a) {
			if (a.floor != d.floor) return;
			CAPTURE(a.time, d.time, d.floor);
			REQUIRE(a.time - d.time <= open + 1e-3);
			REQUIRE(a.time - d.time >= open - 1e-6);
		});
	}
}

namespace {

// Watches every cabin each time a call is decided: within one leg, a cabin only ever
// gets further from where the leg began. A retarget to a floor behind it would move
// the position back, with no travel, because the leg keeps its origin and its clock.
class WatchesThePosition : public IDispatchStrategy {
	public:
		explicit WatchesThePosition(std::shared_ptr<IDispatchStrategy> base) : wrapped(std::move(base)) {}
		int selectElevator(Task task, std::vector<Elevator>& cars) override {
			for (size_t i = 0; i < cars.size(); ++i) {
				const double start = cars[i].getLegStartTime();
				Seen& seen = legs[i];
				if (seen.legStart != start) { seen = Seen{start, cars[i].getPosition(), 0.0}; continue; }
				const double reached = std::fabs(cars[i].getPosition() - seen.origin);
				if (reached + 1e-9 < seen.furthest) ++backwards;
				if (reached > seen.furthest) { seen.furthest = reached; ++moving; }
			}
			return wrapped->selectElevator(task, cars);
		}
		long backwards = 0;
		long moving = 0;
	private:
		struct Seen { double legStart; double origin; double furthest; };
		std::map<size_t, Seen> legs;
		std::shared_ptr<IDispatchStrategy> wrapped;
};

} // namespace

TEST_CASE("Within one leg a cabin never moves back towards where it started", "[identities]") {
	// That zero-length leg, counted: it must take no measurable time.
	long watched = 0;
	for (double weight : {std::numeric_limits<double>::infinity(), 300.0}) {
		for (double rate : {0.2, 2.0}) {
			auto watch = std::make_shared<WatchesThePosition>(std::make_shared<examples::GreedyNearestDispatchStrategy>());
			std::streambuf* saved = std::cout.rdbuf();
			std::ostringstream sink;
			std::cout.rdbuf(sink.rdbuf());
			Simulator sim(3, "position", kFloors, kCars, kCustomers, rate, 1.0f, true,
				std::make_shared<RandomProvider>(std::optional<uint64_t>(3)), watch,
				std::make_shared<StandardMetricsCollector>(1.0, kHeight, dynamics()), std::make_shared<Timeline>(),
				std::nullopt, weight, nullptr, 70.0, 15.0,
				std::make_shared<FixedDwellTimeStrategy>(kOpen, kClose, kTransfer),
				std::make_shared<KinematicTravelTimeStrategy>(kHeight, dynamics()));
			sim.startSimulation();
			std::cout.rdbuf(saved);
			CAPTURE(weight, rate);
			REQUIRE(watch->backwards == 0);
			watched += watch->moving;
		}
	}
	REQUIRE(watched > 100);		// a watch that saw nothing would pass for anything
}

TEST_CASE("A stop lasts what its transfers cost, with and without a door hold", "[identities]") {
	for (const auto& r : runs()) {
		if (r.first.door == Door::None) continue;
		CAPTURE_CELL(r.first);
		const bool hold = r.first.door == Door::DefaultHold;
		const DwellTimes times = resolveDwellTimes(DwellPreset(), r.first.maxWeight);
		const double close = hold ? times.doorCloseS : kClose;
		const double tp = hold ? times.perPassengerS : kTransfer;

		struct Stop { double arrive; int transfers; int boarded; };
		std::map<int, Stop> open;
		for (const Row& row : r.second->rows) {
			if (row.type == "ARRIVE") { open[row.cabin] = Stop{row.time, 0, 0}; continue; }
			auto it = open.find(row.cabin);
			if (it == open.end()) continue;
			if (row.type == "ENTER") { it->second.transfers++; it->second.boarded++; }
			else if (row.type == "LEAVE") it->second.transfers++;
			else if (row.type == "DEPART") {
				const Stop stop = it->second;
				open.erase(it);
				if (stop.boarded == 0) continue;	// a cabin nobody boarded may have parked here
				const double lasted = row.time - stop.arrive;
				const double transfer = stop.transfers * tp;
				CAPTURE(stop.arrive, stop.transfers, lasted);
				if (!hold) {
					REQUIRE(lasted == Catch::Approx(transfer + close).margin(1e-6));
				} else {
					// The hold depends on why the cabin stopped, which the timeline
					// does not say, so either of the two figures will do.
					const bool carCallHold = std::fabs(lasted - (std::max(times.carCallHoldS, transfer) + close)) < 1e-6;
					const bool hallCallHold = std::fabs(lasted - (std::max(times.landingCallHoldS, transfer) + close)) < 1e-6;
					REQUIRE((carCallHold || hallCallHold));
				}
			}
		}
	}
}

TEST_CASE("Nobody boards a cabin that then leaves against them", "[identities]") {
	// Closs's rule 3, which the engine enforces under LOOK.
	for (const auto& r : runs()) {
		CAPTURE_CELL(r.first);
		std::map<int, std::vector<unsigned long int>> boardedHere;
		for (const Row& row : r.second->rows) {
			if (row.type == "ENTER") boardedHere[row.cabin].push_back(row.customer);
			else if (row.type == "ARRIVE" && !boardedHere[row.cabin].empty()) {
				std::vector<unsigned long int> stillHere;
				for (unsigned long int c : boardedHere[row.cabin]) {
					const Passenger& p = r.second->passengers.at(c);
					CAPTURE(c, p.at, p.to, row.floor, row.time);
					// The same floor again is a zero-length leg, not this rule: nobody has
					// been taken anywhere yet, so the verdict waits for the next stop.
					if (row.floor == p.at) { stillHere.push_back(c); continue; }
					REQUIRE(((row.floor - p.at) * (p.to - p.at) > 0));
				}
				boardedHere[row.cabin] = stillHere;
			}
		}
	}
}

TEST_CASE("A cabin never carries more than its rated load, read from the timeline", "[identities]") {
	for (const auto& r : runs()) {
		CAPTURE_CELL(r.first);
		std::map<int, double> load;
		for (const Row& row : r.second->rows) {
			if (row.type == "ENTER") {
				load[row.cabin] += r.second->passengers.at(row.customer).weight;
				REQUIRE(load[row.cabin] <= r.first.maxWeight + 1e-6);
			} else if (row.type == "LEAVE") {
				load[row.cabin] -= r.second->passengers.at(row.customer).weight;
				REQUIRE(load[row.cabin] >= -1e-6);
			}
		}
	}
}

TEST_CASE("Regenerative energy never beats the potential energy of the leg", "[identities]") {
	// E = dPE + losses, with losses >= 0, at any load and distance. Gear
	// efficiency 1, because below that the regenerative branch returns more than
	// mgh and the model breaks this identity.
	LiftMotorParams motor;
	motor.regenerative = true;
	motor.gearEfficiency = 1.0;
	motor.counterweightMassKg = motor.carMassKg + 500.0;
	for (double residual : {0.0, 10.0}) {
		motor.residualInertiaKgM2 = residual;
		for (double load : {0.0, 250.0, 500.0, 750.0, 1000.0}) {
			for (double metres : {-30.0, -3.5, 3.5, 30.0}) {
				const double potential = (motor.carMassKg + load - motor.counterweightMassKg) * 9.81 * metres;
				CAPTURE(residual, load, metres);
				REQUIRE(tripEnergyJ(metres, load, dynamics(), motor) >= potential - 1.0);
			}
		}
	}
}
