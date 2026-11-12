#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include "Elevator.h"
#include "Task.h"
#include "Customer.h"
#include "CustomerFactory.h"
#include "RandomProvider.h"
#include "PercentageCapacityStrategy.h"
#include "GreedyNearestDispatchStrategy.h"

// Elevator capacity and the load-bypass threshold.

using namespace openEGCS;
using namespace openEGCS::examples;

TEST_CASE("PercentageCapacityStrategy computes threshold as a fraction of max weight", "[capacity]") {
	PercentageCapacityStrategy policy(0.6);
	REQUIRE(policy.getBypassThreshold(100.0) == Catch::Approx(60.0));
}

TEST_CASE("PercentageCapacityStrategy threshold is infinite for an unlimited elevator", "[capacity]") {
	PercentageCapacityStrategy policy(0.6);
	double threshold = policy.getBypassThreshold(std::numeric_limits<double>::infinity());
	REQUIRE(std::isinf(threshold));
}

TEST_CASE("PercentageCapacityStrategy rejects an out-of-range fraction", "[capacity]") {
	REQUIRE_THROWS_AS(PercentageCapacityStrategy(0.0), std::invalid_argument);
	REQUIRE_THROWS_AS(PercentageCapacityStrategy(1.5), std::invalid_argument);
}

TEST_CASE("A freshly constructed elevator with finite capacity is not bypassing calls", "[elevator][capacity]") {
	Elevator e(1, "", 0, 1, nullptr, /*max_weight=*/100.0);
	REQUIRE(e.getMaxWeight() == Catch::Approx(100.0));
	REQUIRE(e.getCurrentLoad() == Catch::Approx(0.0));
	REQUIRE_FALSE(e.isBypassingCalls());
}

TEST_CASE("Elevator starts bypassing calls once its load crosses the threshold", "[elevator][capacity]") {
	Elevator e(1, "", 0, 1, nullptr, /*max_weight=*/100.0); // default policy: 60% -> threshold 60

	Customer c1(1, "", /*t=*/0, /*at=*/0, /*to=*/1, UP, /*weight=*/50);
	e.addCustomer(c1);
	REQUIRE_FALSE(e.isBypassingCalls()); // 50 < 60

	Customer c2(2, "", /*t=*/1, /*at=*/0, /*to=*/1, UP, /*weight=*/20);
	e.addCustomer(c2);
	REQUIRE(e.getCurrentLoad() == Catch::Approx(70.0));
	REQUIRE(e.isBypassingCalls()); // 70 >= 60
}

TEST_CASE("Elevator stops bypassing once enough weight has alighted", "[elevator][capacity]") {
	Elevator e(1, "", 0, 1, nullptr, /*max_weight=*/100.0);

	Customer c1(1, "", /*t=*/0, /*at=*/0, /*to=*/1, UP, /*weight=*/50);
	Customer c2(2, "", /*t=*/1, /*at=*/0, /*to=*/1, UP, /*weight=*/20);
	e.addCustomer(c1);
	e.addCustomer(c2);
	REQUIRE(e.isBypassingCalls());

	e.removeCustomer(&c2);
	REQUIRE(e.getCurrentLoad() == Catch::Approx(50.0));
	REQUIRE_FALSE(e.isBypassingCalls());
}

TEST_CASE("An elevator with the default (unlimited) capacity never bypasses calls", "[elevator][capacity]") {
	Elevator e(1, "", 0, 1); // no max_weight given -> unlimited
	for (int i = 0; i < 50; i++) {
		Customer c(i, "", /*t=*/i, /*at=*/0, /*to=*/1, UP, /*weight=*/1000.0);
		e.addCustomer(c);
	}
	REQUIRE_FALSE(e.isBypassingCalls());
}

TEST_CASE("GreedyNearestDispatchStrategy skips an elevator that is bypassing calls", "[dispatch][capacity]") {
	std::vector<Elevator> elevators;
	Elevator e1(1, "", 5, 1, nullptr, /*max_weight=*/100.0);  // closer to the call...
	Elevator e2(2, "", 0, 1, nullptr, /*max_weight=*/100.0);  // ...but farther

	Customer heavy(1, "", /*t=*/0, /*at=*/0, /*to=*/1, UP, /*weight=*/80);
	e1.addCustomer(heavy); // 80/100 = 80% >= 60% default threshold -> bypassing

	elevators.push_back(e1);
	elevators.push_back(e2);

	REQUIRE(elevators[0].isBypassingCalls());
	REQUIRE_FALSE(elevators[1].isBypassingCalls());

	Task callTask(0, "", CALL, UP, 6, 0); // floor 6 is closer to elevator 1 (pos 5) than 2 (pos 0)

	GreedyNearestDispatchStrategy strategy;
	int chosen = strategy.selectElevator(callTask, elevators);

	// Without bypass, elevator 1 (closer) would win; with bypass, elevator
	// 2 must be chosen instead, even though it's farther away.
	REQUIRE(chosen == 2);
}

TEST_CASE("GreedyNearestDispatchStrategy falls back to picking an elevator when all are bypassing", "[dispatch][capacity]") {
	std::vector<Elevator> elevators;
	Elevator e1(1, "", 0, 1, nullptr, /*max_weight=*/100.0);
	Elevator e2(2, "", 10, 1, nullptr, /*max_weight=*/100.0);

	Customer heavy1(1, "", /*t=*/0, /*at=*/0, /*to=*/1, UP, /*weight=*/90);
	Customer heavy2(2, "", /*t=*/1, /*at=*/10, /*to=*/11, UP, /*weight=*/90);
	e1.addCustomer(heavy1);
	e2.addCustomer(heavy2);

	elevators.push_back(e1);
	elevators.push_back(e2);

	REQUIRE(elevators[0].isBypassingCalls());
	REQUIRE(elevators[1].isBypassingCalls());

	Task callTask(0, "", CALL, UP, 5, 0);
	GreedyNearestDispatchStrategy strategy;
	int chosen = strategy.selectElevator(callTask, elevators);

	REQUIRE(chosen != -1);
}

TEST_CASE("CustomerFactory never generates a non-positive weight even with a large stddev", "[capacity][customerfactory]") {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(7));
	CustomerFactory factory(rng, /*max_floors=*/10, /*sane_customers=*/true, /*mean_weight=*/5.0, /*stddev_weight=*/50.0);

	for (unsigned long int i = 0; i < 200; i++) {
		Customer c = factory.createArrival(i, /*timestamp=*/0.0);
		REQUIRE(c.getWeight() > 0);
	}
}

// A zero stddev is how a capacity is expressed in passengers: everybody
// weighs exactly the mean. It must not change WHO arrives WHERE for a seed,
// only how much they weigh - so the factory keeps its one Normal draw per
// customer and discards it.
TEST_CASE("CustomerFactory with stddev 0 gives every customer exactly the mean weight, and the same arrivals",
	"[capacity][customerfactory]") {
	auto rngFlat = std::make_shared<RandomProvider>(std::optional<uint64_t>(42));
	auto rngSpread = std::make_shared<RandomProvider>(std::optional<uint64_t>(42));
	CustomerFactory flat(rngFlat, /*max_floors=*/9, /*sane_customers=*/true, /*mean_weight=*/1.0, /*stddev_weight=*/0.0);
	CustomerFactory spread(rngSpread, /*max_floors=*/9, /*sane_customers=*/true, /*mean_weight=*/1.0, /*stddev_weight=*/15.0);

	for (unsigned long int i = 0; i < 500; i++) {
		Customer a = flat.createArrival(i, /*timestamp=*/0.0);
		Customer b = spread.createArrival(i, /*timestamp=*/0.0);
		REQUIRE(a.getWeight() == 1.0);
		REQUIRE(a.getAt() == b.getAt());
		REQUIRE(a.getTo() == b.getTo());
		REQUIRE(a.getDir() == b.getDir());
	}
}

TEST_CASE("CustomerFactory refuses a negative stddev instead of drawing from it", "[capacity][customerfactory]") {
	auto rng = std::make_shared<RandomProvider>(std::optional<uint64_t>(7));
	REQUIRE_THROWS_AS(CustomerFactory(rng, /*max_floors=*/10, /*sane_customers=*/true, /*mean_weight=*/70.0,
		/*stddev_weight=*/-1.0), std::invalid_argument);
}
