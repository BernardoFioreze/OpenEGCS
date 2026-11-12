#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>
#include "Simulator.h"
#include "ElevatorSystem.h"
#include "DwellPresets.h"
#include "RandomProvider.h"
#include "StandardMetricsCollector.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;

// A stop is one event, and the cabin that ARRIVES is the one that serves the
// landing - not necessarily the cabin the dispatcher picked.
//
// Everything here is asserted from the LOGGED TIMELINE rather than from the
// engine's own state, on purpose: these are claims an outside reader can
// check, and the timeline is the artefact the project ships. It is the same
// independence test_metrics_consistency.cpp is built on.

namespace {

struct LoggedEvent {
	double time;
	std::string type;
	int elevatorId;
	unsigned long int customerId;
	int floor;
};

// Records the timeline instead of writing a file, so the cases can read it.
struct RecordingLogger : ISimulationLogger {
	std::vector<LoggedEvent> events;
	std::map<unsigned long int, Customer> customers;

	void logCustomer(const Customer& customer) override {
		customers.insert(std::make_pair(customer.getId(), customer));
	}
	void logEvent(double time, const std::string& type, int elevatorId,
		unsigned long int customerId, int floor) override {
		events.push_back(LoggedEvent{time, type, elevatorId, customerId, floor});
	}
	void setMetadata(const SimulationMetadata&) override {}
	void finalize(const IMetricsCollector&, const std::string&) override {}
};

struct Scenario {
	int floors, elevators, customers;
	double rate;
	uint64_t seed;
	double maxWeight = std::numeric_limits<double>::infinity();
};

const Scenario kScenarios[] = {
	{10, 3, 50, 0.8, 42},
	{20, 4, 400, 3.0, 7},
	{12, 2, 120, 1.5, 4242},
	{10, 3, 50, 0.8, 42, /*maxWeight=*/300.0},
	{20, 4, 400, 1.0, 7, /*maxWeight=*/1200.0},
};

std::shared_ptr<RecordingLogger> run(const Scenario& s) {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(s.seed));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto logger = std::make_shared<RecordingLogger>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	Simulator sim(1, "stop-arrival", s.floors, s.elevators, s.customers, s.rate,
		/*speed=*/1.0f, /*sane=*/true, rng, dispatch, metrics, logger,
		/*seed_for_metadata=*/std::nullopt, s.maxWeight);
	sim.startSimulation();
	return logger;
}

std::string describe(const Scenario& s) {
	return std::to_string(s.floors) + " floors, " + std::to_string(s.elevators)
		+ " elevators, seed " + std::to_string(s.seed)
		+ (s.maxWeight == std::numeric_limits<double>::infinity()
			? ", unlimited" : ", capacity " + std::to_string((int)s.maxWeight));
}

// The floors a cabin stopped at, in order, read from its ARRIVE rows.
std::map<int, std::vector<std::pair<double, int> > > stopsOf(const RecordingLogger& log) {
	std::map<int, std::vector<std::pair<double, int> > > stops;
	for (const LoggedEvent& e : log.events) {
		if (e.type != "ARRIVE") continue;
		stops[e.elevatorId].push_back(std::make_pair(e.time, e.floor));
	}
	for (auto& entry : stops) std::sort(entry.second.begin(), entry.second.end());
	return stops;
}

} // namespace

// ---------------------------------------------------------------------------
// The event and the records
// ---------------------------------------------------------------------------

TEST_CASE("Every transfer happens at a stop that has an ARRIVE of its own", "[stop_arrival][invariants]") {
	// The structural claim: a boarding, an alighting and a refusal are not
	// events, they are what a stop's ARRIVE produced. So each of them must share
	// its instant, its cabin and its floor with one.
	for (const Scenario& s : kScenarios) {
		INFO(describe(s));
		std::shared_ptr<RecordingLogger> log = run(s);

		std::set<std::tuple<int, int, long long> > arrivals;
		int arriveRows = 0;
		for (const LoggedEvent& e : log->events) {
			if (e.type != "ARRIVE") continue;
			arriveRows++;
			arrivals.insert(std::make_tuple(e.elevatorId, e.floor, (long long)(e.time * 1e9)));
		}
		REQUIRE(arriveRows > 0);
		// One ARRIVE per stop and no more: the set loses nothing.
		REQUIRE((int)arrivals.size() == arriveRows);

		int transfers = 0;
		for (const LoggedEvent& e : log->events) {
			if (e.type != "ENTER" && e.type != "LEAVE" && e.type != "PASS") continue;
			transfers++;
			INFO("transfer " << e.type << " of customer " << e.customerId
				<< " at floor " << e.floor << " t=" << e.time);
			REQUIRE(arrivals.count(std::make_tuple(e.elevatorId, e.floor,
				(long long)(e.time * 1e9))) == 1);
		}
		REQUIRE(transfers >= 2 * s.customers);		// everybody boards and alights
	}
}

TEST_CASE("A cabin's ARRIVE rows are its stops, in order, with no repeats at one instant",
	"[stop_arrival][invariants]") {
	for (const Scenario& s : kScenarios) {
		INFO(describe(s));
		std::map<int, std::vector<std::pair<double, int> > > stops = stopsOf(*run(s));
		REQUIRE(stops.size() >= 1);
		for (const auto& entry : stops) {
			const std::vector<std::pair<double, int> >& seq = entry.second;
			for (size_t i = 1; i < seq.size(); ++i) {
				// Two stops of one cabin cannot share an instant: a cabin is in
				// one place at a time, and a stop is one event.
				REQUIRE(seq[i].first > seq[i - 1].first);
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Rule 3 of the five rules of lift traffic control (Closs, 1970)
// ---------------------------------------------------------------------------

TEST_CASE("Nobody is carried away from their destination", "[stop_arrival][regression]") {
	// "A passenger does not board a car travelling in the opposite direction to
	// their own" - Barney & Al-Sharif section 10.4, Rule 3 - stated as the
	// consequence an outside reader can check: between boarding and alighting,
	// the cabin never goes further from the destination than the boarding floor
	// was. The direction filter is what makes it zero: a passenger only boards a
	// cabin whose announced direction is their own. (Without it, about a third
	// of the passengers of the saturated reference scenario are carried the
	// wrong way.)
	for (const Scenario& s : kScenarios) {
		INFO(describe(s));
		std::shared_ptr<RecordingLogger> log = run(s);
		std::map<int, std::vector<std::pair<double, int> > > stops = stopsOf(*log);

		std::map<unsigned long int, std::pair<double, int> > boarded;	// time, elevator
		for (const LoggedEvent& e : log->events) {
			if (e.type == "ENTER" && boarded.find(e.customerId) == boarded.end()) {
				boarded[e.customerId] = std::make_pair(e.time, e.elevatorId);
			}
			if (e.type != "LEAVE") continue;
			auto it = boarded.find(e.customerId);
			REQUIRE(it != boarded.end());				// nobody alights without boarding
			const Customer& c = log->customers.at(e.customerId);
			double from = it->second.first;
			int cabin = it->second.second;
			double start = std::abs((double)(c.getTo() - c.getAt()));
			for (const std::pair<double, int>& stop : stops[cabin]) {
				if (stop.first < from || stop.first > e.time) continue;
				INFO("customer " << c.getId() << " boarded at " << c.getAt()
					<< " for " << c.getTo() << ", cabin " << cabin
					<< " stopped at " << stop.second << " t=" << stop.first);
				REQUIRE(std::abs((double)(c.getTo() - stop.second)) <= start);
			}
		}
	}
}

// ---------------------------------------------------------------------------
// The item itself
// ---------------------------------------------------------------------------

TEST_CASE("A passenger boards the cabin that arrives, not the one that was sent",
	"[stop_arrival][regression]") {
	// If a cabin stops at your floor, is going your way and has room, you get in
	// - whoever the dispatcher had promised you.
	//
	// A "missed opportunity" is a stop where somebody was registered at that
	// floor for that direction, did not board, and boarded later somewhere else.
	// The direction of a stop is read from the cabin's NEXT stop, which is the
	// only source an outside reader has. Without boarding at the door, the
	// saturated reference scenario has about a thousand of them.
	//
	// The bound is a rate rather than zero, and the reason is a real limit of
	// what the engine can promise. The cabin announces its direction from the
	// route as it stands when the doors open - foresight - while this count
	// reads the direction from where the cabin actually went next - hindsight.
	// A call allocated after those doors closed can send it the other way, and
	// then a passenger who was "going the same way" by hindsight was not by
	// foresight (2 of 400 in `20 4 400 3.0 ... 7`, both of them people at floor 1
	// going up while a cabin stopped there for a car call, announced down, and
	// was given a stop above after leaving).
	//
	// The capacity scenarios are left out entirely: a cabin that is full cannot
	// take anybody, and the timeline does not carry the weights needed to tell
	// that apart from a missed chance.
	const Scenario unlimited[] = {
		{10, 3, 50, 0.8, 42},
		{20, 4, 400, 3.0, 7},
		{12, 2, 120, 1.5, 4242},
	};
	for (const Scenario& s : unlimited) {
		INFO(describe(s));
		std::shared_ptr<RecordingLogger> log = run(s);
		std::map<int, std::vector<std::pair<double, int> > > stops = stopsOf(*log);

		std::map<unsigned long int, double> called, boarded;
		for (const LoggedEvent& e : log->events) {
			if (e.type == "CUST_CALL") called.insert(std::make_pair(e.customerId, e.time));
			else if (e.type == "ENTER") boarded.insert(std::make_pair(e.customerId, e.time));
		}

		int missed = 0;
		for (const auto& entry : stops) {
			const std::vector<std::pair<double, int> >& seq = entry.second;
			for (size_t i = 0; i + 1 < seq.size(); ++i) {
				int floor = seq[i].second;
				int next = seq[i + 1].second;
				if (next == floor) continue;			// no direction to infer
				int direction = next > floor ? UP : DOWN;
				double when = seq[i].first;
				for (const auto& person : log->customers) {
					const Customer& c = person.second;
					if (c.getAt() != floor || (int)c.getDir() != direction) continue;
					auto call = called.find(c.getId());
					auto board = boarded.find(c.getId());
					if (call == called.end() || board == boarded.end()) continue;
					if (call->second > when || board->second <= when) continue;
					INFO("customer " << c.getId() << " waited at " << floor
						<< " going " << (direction == UP ? "up" : "down")
						<< " while cabin " << entry.first << " stopped there at t=" << when
						<< " and boarded only at t=" << board->second);
					missed++;
				}
			}
		}
		INFO("missed=" << missed << " of " << s.customers << " customers");
		REQUIRE(missed * 100 <= s.customers);		// 1%, against 34% before the item
	}
}

TEST_CASE("A cabin does not keep a stop for passengers another cabin already took",
	"[stop_arrival][invariants]") {
	// Rule 1 of the five rules: no stop where nobody boards or alights. It is
	// reachable when another cabin takes your passengers, and
	// ElevatorSystem::withdrawStopFrom is what takes the stop back off the
	// cabin that was promised them.
	//
	// It is asserted as a RATE and not as zero, because one case survives and is
	// physical: a cabin already braking for a stop cannot un-brake, so if its
	// passengers board somebody else while it is on its way, it still stops
	// (0 of 52 stops in the small scenario, 14 of 927 in the saturated one,
	// 4.1% in the worst of the capacity scenarios).
	for (const Scenario& s : kScenarios) {
		INFO(describe(s));
		std::shared_ptr<RecordingLogger> log = run(s);

		std::set<std::tuple<int, int, long long> > withTransfer;
		for (const LoggedEvent& e : log->events) {
			if (e.type != "ENTER" && e.type != "LEAVE" && e.type != "PASS") continue;
			withTransfer.insert(std::make_tuple(e.elevatorId, e.floor, (long long)(e.time * 1e9)));
		}
		int stops = 0, empty = 0;
		for (const LoggedEvent& e : log->events) {
			if (e.type != "ARRIVE") continue;
			stops++;
			if (withTransfer.count(std::make_tuple(e.elevatorId, e.floor,
				(long long)(e.time * 1e9))) == 0) empty++;
		}
		INFO("stops=" << stops << " empty=" << empty);
		REQUIRE(stops > 0);
		REQUIRE(empty * 100 <= stops * 10);		// at most 10% - the braking residue
	}
}

// ---------------------------------------------------------------------------
// Item #40: the cabin that arrives, empties and has nowhere to go
// ---------------------------------------------------------------------------

namespace {

// Answers the dispatch from a script, one entry per consultation, so a case can
// say which cabin each call goes to instead of depending on what a real
// strategy would estimate.
struct ScriptedDispatch : public IDispatchStrategy {
	std::vector<int> answers;
	size_t next = 0;
	explicit ScriptedDispatch(std::vector<int> a) : answers(std::move(a)) {}
	int selectElevator(Task, std::vector<Elevator>&) override {
		return next < answers.size() ? answers[next++] : 1;
	}
};

struct ScriptedRun {
	std::vector<ElevatorSystem::Transfer> transfers;
	double firstArrival = -1;	// of `cabin` at `floor`
	bool drained = false;
};

// Feeds `people` to a two-cabin, ten-floor system as CUST_CALLs and drives the
// queue the way Simulator does, recording every transfer and the first instant
// cabin `cabin` arrives at `floor`.
ScriptedRun driveScripted(std::vector<int> script, std::vector<Customer>& people, int cabin, int floor) {
	auto dispatch = std::make_shared<ScriptedDispatch>(std::move(script));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	// Instantaneous stops, pinned. Every script below is a geometry of who
	// reaches which floor first, timed for a zero dwell: with the default centre
	// door, cabin 1 spends ~6.7 s boarding A at the lobby while cabin 2 leaves
	// empty at once, so cabin 2 arrives first and the premise of the empty-cabin
	// case is gone. The subject here is the announcement, not the dwell; the
	// dwell is exercised by the scenario-wide cases further down, which run on
	// the default.
	auto instantaneous = makeDwellStrategy(DwellPreset::instantaneous(), std::numeric_limits<double>::infinity());
	ElevatorSystem system(/*elevators=*/2, /*floors=*/10, 1.0, dispatch, metrics,
		std::numeric_limits<double>::infinity(), /*capacity=*/nullptr, instantaneous);
	unsigned long int id = 1;
	for (Customer& c : people) {
		Event call(id++, "", CUST_CALL, c.getDir(), c.getAt(), c.getTime(), &c);
		system.upcomingEvents.push(call);
	}
	ScriptedRun out;
	Timer clock;
	for (int i = 0; i < 200 && !system.upcomingEvents.empty(); i++) {
		Event next = system.upcomingEvents.pop();
		clock.updateTick(next.getTime());
		system.update(clock);
		if (next.getType() == ARRIVE && next.getElevator() == cabin && next.getFloor() == floor
			&& out.firstArrival < 0) {
			out.firstArrival = next.getTime();
		}
		system.giveEvent(next);
		for (const ElevatorSystem::Transfer& t : system.drainTransfers()) out.transfers.push_back(t);
	}
	out.drained = system.upcomingEvents.empty();
	return out;
}

// The one ENTER of `customerId`, or a transfer with elevatorId 0 if there is none.
ElevatorSystem::Transfer boardingOf(const ScriptedRun& run, unsigned long int customerId, int* count) {
	ElevatorSystem::Transfer found{ENTER, 0, -1, customerId, -1.0};
	*count = 0;
	for (const ElevatorSystem::Transfer& t : run.transfers) {
		if (t.type != ENTER || t.customerId != customerId) continue;
		(*count)++;
		found = t;
	}
	return found;
}

} // namespace

TEST_CASE("An empty cabin with nowhere to go takes whoever is waiting at its floor",
	"[stop_arrival][regression]") {
	// An empty cabin at a floor where somebody waits on another cabin's call.
	// Cabin 1 carries A from 0 to 5 on a car call and has nothing else to do. B
	// is waiting at 5 to go down, and their call was allocated to cabin 2, which
	// set off from 0 one second after cabin 1 and so arrives later. When cabin
	// 1's doors open at 5 it is empty, its route holds nothing but this stop and
	// the stop carries no landing call. It must announce DOWN and take B: the
	// dispatcher is not asked again about a lit, allocated button, so this is
	// the only way B gets the cabin standing at their floor. B boards cabin 1.
	std::vector<Customer> people = {
		Customer(1, "", /*t=*/0.0, /*at=*/0, /*to=*/5, UP, /*weight=*/70.0),		// A
		Customer(2, "", /*t=*/1.0, /*at=*/5, /*to=*/2, DOWN, /*weight=*/70.0),	// B
	};
	ScriptedRun run = driveScripted({1, 2}, people, /*cabin=*/1, /*floor=*/5);
	REQUIRE(run.drained);
	REQUIRE(run.firstArrival > 0);

	// B boards exactly once, and it is cabin 1, at the instant its doors opened
	// at 5 - not cabin 2, whenever that would have turned up.
	int boardings = 0;
	ElevatorSystem::Transfer b = boardingOf(run, 2, &boardings);
	INFO("B boarded cabin " << b.elevatorId << " at floor " << b.floor << " t=" << b.time);
	REQUIRE(boardings == 1);
	REQUIRE(b.elevatorId == 1);
	REQUIRE(b.floor == 5);
	REQUIRE(b.time == run.firstArrival);

	// And they get where they were going, on the same cabin.
	bool arrived = false;
	for (const ElevatorSystem::Transfer& t : run.transfers) {
		if (t.type == LEAVE && t.customerId == 2) {
			arrived = true;
			REQUIRE(t.elevatorId == 1);
			REQUIRE(t.floor == 2);
		}
	}
	REQUIRE(arrived);
}

TEST_CASE("With both buttons lit, the empty cabin takes the older call", "[stop_arrival]") {
	// The tie-break of that rule, which none of the six reference scenarios
	// exercises with both buttons lit - so it is pinned here instead. Oldest
	// registration first, the same rule as step 3 of the announcement and as the
	// pool of unallocated calls: it needs no new number.
	//
	// B presses DOWN at 5 before C presses UP there, both calls go to cabin 2,
	// and cabin 1 reaches 5 first with nothing to do. It announces DOWN and takes
	// B; C is not taken by a cabin that is about to go the other way (Rule 3 of
	// Closs) and is left to cabin 2, which was coming for them anyway.
	std::vector<Customer> people = {
		Customer(1, "", /*t=*/0.0, /*at=*/0, /*to=*/5, UP, /*weight=*/70.0),		// A
		Customer(2, "", /*t=*/0.5, /*at=*/5, /*to=*/2, DOWN, /*weight=*/70.0),	// B, older
		Customer(3, "", /*t=*/0.7, /*at=*/5, /*to=*/8, UP, /*weight=*/70.0),		// C, younger
	};
	ScriptedRun run = driveScripted({1, 2, 2}, people, /*cabin=*/1, /*floor=*/5);
	REQUIRE(run.drained);
	REQUIRE(run.firstArrival > 0);

	int boardings = 0;
	ElevatorSystem::Transfer b = boardingOf(run, 2, &boardings);
	REQUIRE(boardings == 1);
	REQUIRE(b.elevatorId == 1);
	REQUIRE(b.time == run.firstArrival);

	ElevatorSystem::Transfer c = boardingOf(run, 3, &boardings);
	REQUIRE(boardings == 1);
	REQUIRE(c.elevatorId == 2);
}

TEST_CASE("No cabin parks empty at a floor where somebody is waiting", "[stop_arrival][invariants]") {
	// The invariant form of that rule, read from the timeline: a stop at which
	// nobody got in and nobody was refused, after which the cabin did not leave,
	// at a floor where somebody had pressed a button before the cabin got there
	// and was still waiting after it had stopped. That is a cabin announcing no
	// direction - IDLE - in front of a waiting passenger.
	//
	// "Did not leave" is read from the cabin's next DEPART (the cabins alternate
	// DEPART and ARRIVE). A refusal at the door (PASS) counts as having been
	// offered the cabin, which is all this promises: a full cabin is not the
	// defect.
	for (const Scenario& s : kScenarios) {
		INFO(describe(s));
		std::shared_ptr<RecordingLogger> log = run(s);

		typedef std::tuple<int, int, long long> StopKey;	// elevator, floor, time
		auto key = [](const LoggedEvent& e) {
			return std::make_tuple(e.elevatorId, e.floor, (long long)(e.time * 1e9));
		};
		std::set<StopKey> offered;
		std::map<StopKey, int> transfersAt;					// ENTER + LEAVE at one stop
		std::map<int, std::vector<double> > departures;		// per cabin, in time order
		std::map<unsigned long int, double> called, boarded;
		for (const LoggedEvent& e : log->events) {
			if (e.type == "ENTER" || e.type == "PASS") offered.insert(key(e));
			if (e.type == "ENTER" || e.type == "LEAVE") transfersAt[key(e)]++;
			if (e.type == "DEPART") departures[e.elevatorId].push_back(e.time);
			if (e.type == "CUST_CALL") called.insert(std::make_pair(e.customerId, e.time));
			if (e.type == "ENTER") boarded.insert(std::make_pair(e.customerId, e.time));
		}
		// "The cabin left this stop instead of parking at it": an ARRIVE is logged
		// when the doors are open, so a cabin that leaves straight away departs
		// after its transfers and its door closing - not at the ARRIVE's own
		// instant, which is only true with a zero dwell.
		//
		// The doors are also held open for a minimum time, which depends on why the
		// cabin stopped (2.0 s car call, 3.0 s landing call) - something the log
		// does not say. The landing-call hold is the longer one, so it bounds the
		// stop from above; reading the transfers alone would count cabins leaving
		// on their own route as parked (for example a cabin that let somebody out at
		// 9, held 2.0 s, closed in 3.0 s and left for an up-call at 10, past a
		// passenger going down).
		const DwellTimes dwell = resolveDwellTimes(DwellPreset{}, s.maxWeight);
		const double longestHold = (dwell.hold == DoorHold::None) ? 0.0
			: std::max(dwell.carCallHoldS, dwell.landingCallHoldS);
		auto leftAtOnce = [&](const LoggedEvent& e) {
			const double openPhase = std::max(longestHold, transfersAt[key(e)] * dwell.perPassengerS);
			const double stopEnd = e.time + openPhase + dwell.doorCloseS;
			for (double t : departures[e.elevatorId]) {
				if (t >= e.time - 1e-9 && t <= stopEnd + 1e-6) return true;
			}
			return false;
		};

		int parkedInFrontOfSomebody = 0;
		std::string firstCase;
		for (const LoggedEvent& e : log->events) {
			if (e.type != "ARRIVE") continue;
			if (offered.count(key(e))) continue;
			if (leftAtOnce(e)) continue;
			for (const auto& person : log->customers) {
				const Customer& c = person.second;
				if (c.getAt() != e.floor) continue;
				auto call = called.find(c.getId());
				auto board = boarded.find(c.getId());
				if (call == called.end() || board == boarded.end()) continue;
				// Strictly before: somebody pressing at the very instant the cabin
				// arrives is ordered after it by the queue and is handed to the
				// allocation path, which is not this.
				if (call->second >= e.time || board->second <= e.time) continue;
				if (firstCase.empty()) {
					firstCase = "cabin " + std::to_string(e.elevatorId) + " parked empty at "
						+ std::to_string(e.floor) + " t=" + std::to_string(e.time) + " while customer "
						+ std::to_string(c.getId()) + " waited there (called t="
						+ std::to_string(call->second) + ", boarded t=" + std::to_string(board->second) + ")";
				}
				parkedInFrontOfSomebody++;
			}
		}
		INFO(parkedInFrontOfSomebody << " cases; first: " << firstCase);
		REQUIRE(parkedInFrontOfSomebody == 0);
	}
}
