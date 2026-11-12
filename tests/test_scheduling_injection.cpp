#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "Simulator.h"
#include "ElevatorSystem.h"
#include "RandomProvider.h"
#include "CustomerFactory.h"
#include "StandardMetricsCollector.h"
#include "LookSchedulingStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "FcfsSchedulingStrategy.h"

using namespace openEGCS;

// IElevatorSchedulingStrategy injected from the top: through Simulator and
// ElevatorSystem, into every cabin of the group. These cases prove the seam
// reaches a full simulation, and that it reaches every cabin rather than one.

namespace {

// Delegates to LOOK and counts how often it is consulted. Delegating exactly is
// what lets the first case compare whole snapshots: if the injected instance is
// used where the default was, the run cannot tell them apart.
class CountingLook : public IElevatorSchedulingStrategy {
	public:
		mutable std::size_t insertions = 0;
		mutable std::size_t reorders = 0;

		std::vector<Task>::iterator findInsertionPoint(std::vector<Task>& tasks,
			unsigned int direction, double position, const Task& newTask) const override {
			insertions++;
			return look.findInsertionPoint(tasks, direction, position, newTask);
		}
		void reorder(std::vector<Task>& tasks, unsigned int direction, double position) const override {
			reorders++;
			look.reorder(tasks, direction, position);
		}

	private:
		LookSchedulingStrategy look;
};

// A test double, not examples::FcfsSchedulingStrategy: it only has to order a
// route in a way LOOK never does. Every new stop goes to the back and nothing
// is re-sorted (reorder keeps its empty default), so under it a cabin's route
// stays in the order its stops were created - ascending stopId, since
// Elevator::addTask hands those out increasing.
class AppendInArrivalOrder : public IElevatorSchedulingStrategy {
	public:
		std::vector<Task>::iterator findInsertionPoint(std::vector<Task>& tasks,
			unsigned int, double, const Task&) const override {
			return tasks.end();
		}
};

std::string runAndReadSnapshot(std::shared_ptr<IElevatorSchedulingStrategy> scheduling,
	const std::string& path) {
	std::filesystem::remove(path);
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(42));
	auto metrics = std::make_shared<StandardMetricsCollector>();
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();

	Simulator sim(1, "scheduling-injection", /*floors=*/10, /*elevators=*/3, /*customers=*/60,
		/*arrivalRate=*/1.2, /*speed=*/1.0, /*sane=*/true,
		rng, dispatch, metrics, /*logger=*/nullptr, /*seed_for_metadata=*/std::nullopt,
		std::numeric_limits<double>::infinity(), /*capacity=*/nullptr,
		/*mean_weight=*/70.0, /*stddev_weight=*/15.0, /*dwell=*/nullptr, /*travel=*/nullptr,
		/*passenger_behavior=*/nullptr, /*group_dispatch=*/nullptr, /*traffic_profile=*/nullptr,
		scheduling);
	sim.startSimulation();		// writes ./snapshot.json
	std::filesystem::copy_file("snapshot.json", path,
		std::filesystem::copy_options::overwrite_existing);

	std::ifstream in(path, std::ios::binary);
	REQUIRE(in.good());
	std::ostringstream buffer;
	buffer << in.rdbuf();
	in.close();
	std::filesystem::remove(path);
	return buffer.str();
}

bool routeInStopIdOrder(const std::vector<Task>& route) {
	for (std::size_t i = 0; i + 1 < route.size(); i++) {
		if (route[i].getStopId() >= route[i + 1].getStopId()) return false;
	}
	return true;
}

struct RouteObservation {
	bool everOutOfStopIdOrder = false;
	std::vector<bool> cabinHeldTwoStops;	// per cabin: was the check ever non-trivial for it?
};

// Drives a workload the way Simulator does (same harness as test_route_order.cpp)
// and inspects every cabin's route after every event.
RouteObservation observeRoutes(std::shared_ptr<IElevatorSchedulingStrategy> scheduling) {
	const int floors = 12, cabins = 4, customers = 300;
	const double rate = 3.0;
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(7));
	auto dispatch = std::make_shared<examples::GreedyNearestDispatchStrategy>();
	ElevatorSystem system(cabins, floors, /*speed=*/1.0, dispatch, nullptr,
		std::numeric_limits<double>::infinity(), nullptr, nullptr, nullptr, nullptr, nullptr,
		nullptr, scheduling);
	CustomerFactory factory(rng, floors, /*sane_customers=*/true);

	double enqueued = 0;
	unsigned long int customerId = 1, eventId = 1;
	auto pushArrival = [&]() {
		enqueued += rng->nextExponential(rate);
		Customer* customer = new Customer(factory.createArrival(customerId, enqueued));
		Event arrival(eventId, "", CUST_CALL, customer->getDir(), customer->getAt(), enqueued, customer);
		system.upcomingEvents.push(arrival);
		customerId++;
		eventId++;
	};
	for (int i = 0; i < 5; i++) pushArrival();

	RouteObservation seen;
	seen.cabinHeldTwoStops.assign(cabins, false);
	Timer clock;
	int served = 0;
	while (!system.upcomingEvents.empty() && served < customers) {
		Event toProcess = system.upcomingEvents.pop();
		clock.updateTick(toProcess.getTime());
		system.update(clock);
		system.giveEvent(toProcess);
		for (const ElevatorSystem::Transfer& tr : system.drainTransfers()) {
			if (tr.type == LEAVE) served++;
		}
		if (toProcess.getType() == CUST_CALL) pushArrival();

		const std::vector<Elevator>& elevators = system.getElevators();
		for (std::size_t c = 0; c < elevators.size(); c++) {
			const std::vector<Task>& route = elevators[c].getRoute();
			if (route.size() >= 2) seen.cabinHeldTwoStops[c] = true;
			if (!routeInStopIdOrder(route)) seen.everOutOfStopIdOrder = true;
		}
	}
	// >= because one ARRIVE can let several people out and cross the mark; what
	// this guards is a policy that stalls the run before enough are delivered.
	REQUIRE(served >= customers);
	return seen;
}

} // namespace

TEST_CASE("A scheduling strategy injected into Simulator is the one the cabins use",
	"[scheduling][injection][e2e][snapshot]") {
	// The default is resolved to LOOK, so injecting a LOOK that only counts must
	// change nothing - down to the byte - and must actually be asked. Both halves
	// are needed: byte-identity alone would also pass if the injected instance
	// were silently dropped.
	auto counting = std::make_shared<CountingLook>();
	std::string byDefault = runAndReadSnapshot(nullptr, "scheduling_default.json");
	std::string injected = runAndReadSnapshot(counting, "scheduling_injected.json");

	REQUIRE_FALSE(byDefault.empty());
	REQUIRE(injected == byDefault);
	REQUIRE(counting->insertions > 0);
	REQUIRE(counting->reorders > 0);
}

TEST_CASE("Every cabin of the group follows the injected route policy",
	"[scheduling][injection]") {
	// Control first: under the default LOOK a route is NOT kept in creation
	// order - a nearer stop is inserted ahead of older ones - so the property
	// below discriminates between the two policies rather than holding anyway.
	RouteObservation look = observeRoutes(nullptr);
	REQUIRE(look.everOutOfStopIdOrder);

	RouteObservation append = observeRoutes(std::make_shared<AppendInArrivalOrder>());
	REQUIRE_FALSE(append.everOutOfStopIdOrder);
	// And not vacuously: every cabin, not just one, had a route long enough for
	// its order to mean something while the injected policy governed it.
	for (std::size_t c = 0; c < append.cabinHeldTwoStops.size(); c++) {
		INFO("cabin index " << c);
		REQUIRE(append.cabinHeldTwoStops[c]);
	}
}

TEST_CASE("The FCFS example keeps every cabin's route in arrival order through a full run",
	"[scheduling][injection][fcfs]") {
	// The test double above only appends; the real policy also merges a later
	// request into the floor's existing stop wherever it sits. A merge creates no
	// stop, so the creation order still holds - this checks it does in a real
	// run, with pooled calls coming back and stops being withdrawn.
	RouteObservation fcfs = observeRoutes(std::make_shared<examples::FcfsSchedulingStrategy>());
	REQUIRE_FALSE(fcfs.everOutOfStopIdOrder);
	for (std::size_t c = 0; c < fcfs.cabinHeldTwoStops.size(); c++) {
		INFO("cabin index " << c);
		REQUIRE(fcfs.cabinHeldTwoStops[c]);
	}
}
