#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "StandardMetricsCollector.h"
#include "LiftKinematics.h"
#include "LiftMotorModel.h"
#include "Customer.h"

using namespace openEGCS;

TEST_CASE("StandardMetricsCollector computes average waiting time", "[metrics]") {
	StandardMetricsCollector metrics;

	Customer c1(1, "", /*t=*/0, /*at*/0, /*to*/5, UP);
	Customer c2(2, "", /*t=*/2, /*at*/1, /*to*/4, UP);

	metrics.onCustomerArrived(c1);
	metrics.onCustomerArrived(c2);

	metrics.onCustomerBoarded(c1, 1, /*time=*/4);   // waited 4
	metrics.onCustomerBoarded(c2, 1, /*time=*/6);   // waited 4

	REQUIRE(metrics.getAverageWaitingTime() == Catch::Approx(4.0));
}

TEST_CASE("StandardMetricsCollector computes average travel time", "[metrics]") {
	StandardMetricsCollector metrics;

	Customer c1(1, "", 0, 0, 5, UP);

	metrics.onCustomerBoarded(c1, 1, 4);
	metrics.onCustomerAlighted(c1, 1, 10); // traveled 6

	REQUIRE(metrics.getAverageTravelTime() == Catch::Approx(6.0));
}

// onElevatorMoved (distance x energyPerFloor) is deprecated - kept callable
// for interface compatibility, but not folded into getEnergyConsumption()/
// getTotalEnergyConsumption(), which come exclusively from the
// electromechanical model driven by onElevatorDeparted (see below).
// Summing both would double-count the cruise-speed traction work that is
// already inside the motor model's load torque/power terms.
TEST_CASE("StandardMetricsCollector no longer folds onElevatorMoved into the energy total", "[metrics]") {
	StandardMetricsCollector metrics(2.0); // energyPerFloor = 2.0 (affects only the deprecated proxy)

	metrics.onElevatorMoved(1, 3.0);
	metrics.onElevatorMoved(1, -2.0);
	metrics.onElevatorMoved(2, 1.0);

	REQUIRE(metrics.getEnergyConsumption(1) == 0.0);
	REQUIRE(metrics.getEnergyConsumption(2) == 0.0);
	REQUIRE(metrics.getTotalEnergyConsumption() == 0.0);
}

TEST_CASE("StandardMetricsCollector accumulates energy per elevator via onElevatorDeparted", "[metrics]") {
	StandardMetricsCollector metrics;

	metrics.onElevatorDeparted(1, /*floorsTravelled=*/5, /*currentLoad=*/300, /*time=*/0);
	metrics.onElevatorDeparted(2, /*floorsTravelled=*/3, /*currentLoad=*/0, /*time=*/0);

	REQUIRE(metrics.getEnergyConsumption(1) > 0.0);
	REQUIRE(metrics.getEnergyConsumption(2) > 0.0);
	REQUIRE(metrics.getTotalEnergyConsumption() == Catch::Approx(metrics.getEnergyConsumption(1) + metrics.getEnergyConsumption(2)));
}

TEST_CASE("StandardMetricsCollector: a longer trip (more stops) consumes more energy than one long trip of the same total distance", "[metrics]") {
	// Same total distance (10 floors), but one elevator does it as two
	// separate 5-floor trips (two acceleration/deceleration cycles) and the
	// other as a single 10-floor trip (one cycle) - the reason for a motor
	// model instead of a per-floor constant.
	StandardMetricsCollector twoStops;
	twoStops.onElevatorDeparted(1, 5, 0, 0);
	twoStops.onElevatorDeparted(1, 5, 0, 100);

	StandardMetricsCollector oneStop;
	oneStop.onElevatorDeparted(1, 10, 0, 0);

	REQUIRE(twoStops.getEnergyConsumption(1) > oneStop.getEnergyConsumption(1));
}

TEST_CASE("StandardMetricsCollector: idle power is not charged by default", "[metrics]") {
	StandardMetricsCollector metrics; // idlePowerWatts defaults to 0

	metrics.onElevatorDeparted(1, 5, 0, /*time=*/0);
	double energyRightAfter = metrics.getEnergyConsumption(1);

	// A second departure a long time later must not add anything just from
	// the elapsed idle time, since idlePowerWatts is 0.
	metrics.onElevatorDeparted(1, 5, 0, /*time=*/1000);
	double energyAfterLongIdle = metrics.getEnergyConsumption(1);

	// The increment must equal exactly one more trip's worth of energy (same
	// as the first departure), with no extra term for the 1000s idle gap.
	StandardMetricsCollector single;
	single.onElevatorDeparted(1, 5, 0, 0);
	REQUIRE((energyAfterLongIdle - energyRightAfter) == Catch::Approx(single.getEnergyConsumption(1)));
}

TEST_CASE("StandardMetricsCollector: idle power is charged when configured", "[metrics]") {
	LiftDynamics dynamics = defaultDynamicsFor(20.0);
	LiftMotorParams params;
	params.idlePowerWatts = 100.0;

	StandardMetricsCollector withIdlePower(1.0, 3.5, dynamics, params);
	withIdlePower.onElevatorDeparted(1, 5, 0, /*time=*/0);
	double energyRightAfter = withIdlePower.getEnergyConsumption(1);

	withIdlePower.onElevatorDeparted(1, 5, 0, /*time=*/1000);
	double energyAfterLongIdle = withIdlePower.getEnergyConsumption(1);

	StandardMetricsCollector withoutIdlePower(1.0, 3.5, dynamics, LiftMotorParams{});
	withoutIdlePower.onElevatorDeparted(1, 5, 0, 0);
	withoutIdlePower.onElevatorDeparted(1, 5, 0, 1000);

	// With idlePowerWatts > 0, the long gap between trips must cost strictly
	// more than the equivalent run with idle power disabled.
	REQUIRE(energyAfterLongIdle > withoutIdlePower.getEnergyConsumption(1));
}

// ---------------------------------------------------------------------------
// Dissatisfied-passenger metric.
//
// The cases below pin the three decisions that are arbitrary-but-fixed, each
// of which someone would otherwise rediscover by guessing: the quantile is
// nearest-rank (no interpolation), the comparison against the threshold is
// strict, and an unconfigured threshold reports *no verdict* rather than a
// verdict of zero. The last one is the whole reason the getters return
// std::optional.
// ---------------------------------------------------------------------------

namespace {

// Feeds `waits.size()` customers through a boarding each, customer id = index
// + 1, every one of them arriving at t = 0 so the wait is the boarding instant.
void feedWaits(StandardMetricsCollector& metrics, const std::vector<double>& waits) {
	for (std::size_t i = 0; i < waits.size(); i++) {
		Customer c(i + 1, "", /*t=*/0, /*at*/0, /*to*/5, UP);
		metrics.onCustomerBoarded(c, 1, /*time=*/waits[i]);
	}
}

StandardMetricsCollector collectorWith(ServiceThresholds thresholds) {
	return StandardMetricsCollector(/*energy_per_floor=*/1.0, /*floorHeightM=*/3.5,
		defaultDynamicsFor(20.0), LiftMotorParams(), thresholds);
}

} // namespace

TEST_CASE("Waiting distribution: mean, nearest-rank quantiles and worst case", "[metrics]") {
	StandardMetricsCollector metrics;
	feedWaits(metrics, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10});

	TimeDistribution d = metrics.getWaitingTimeDistribution();

	REQUIRE(d.count == 10);
	REQUIRE(d.average == Catch::Approx(5.5));
	// Nearest rank over 10 samples: p50 -> ceil(0.5*10) = 5th smallest = 5,
	// p90 -> ceil(0.9*10) = 9th smallest = 9. Interpolation would give 5.5 and
	// 9.1, which are times nobody experienced - that is the reason for the
	// choice, so the test asserts the choice and not just "something near the
	// middle".
	REQUIRE(d.p50 == Catch::Approx(5.0));
	REQUIRE(d.p90 == Catch::Approx(9.0));
	REQUIRE(d.max == Catch::Approx(10.0));

	// The distribution's mean is the same number the v3 snapshot key reports.
	REQUIRE(d.average == Catch::Approx(metrics.getAverageWaitingTime()));
}

TEST_CASE("The samples may arrive in any order: the quantiles are over the sorted population", "[metrics]") {
	StandardMetricsCollector metrics;
	feedWaits(metrics, {10, 1, 9, 2, 8, 3, 7, 4, 6, 5});

	TimeDistribution d = metrics.getWaitingTimeDistribution();

	REQUIRE(d.p50 == Catch::Approx(5.0));
	REQUIRE(d.p90 == Catch::Approx(9.0));
	REQUIRE(d.max == Catch::Approx(10.0));
}

TEST_CASE("No threshold configured reports no verdict, not a verdict of zero", "[metrics]") {
	StandardMetricsCollector metrics;
	feedWaits(metrics, {1, 2, 3});

	TimeDistribution d = metrics.getWaitingTimeDistribution();

	REQUIRE_FALSE(d.threshold.has_value());
	REQUIRE_FALSE(d.dissatisfiedCount().has_value());
	REQUIRE_FALSE(d.dissatisfiedRate().has_value());
	REQUIRE(d.dissatisfied.empty());
	// ...while the distribution itself is reported either way: it needs no
	// threshold.
	REQUIRE(d.count == 3);
	REQUIRE(d.max == Catch::Approx(3.0));
}

TEST_CASE("A threshold nobody exceeds reports zero, which is a different claim", "[metrics]") {
	ServiceThresholds thresholds;
	thresholds.waiting = 100.0;
	StandardMetricsCollector metrics = collectorWith(thresholds);
	feedWaits(metrics, {1, 2, 3});

	TimeDistribution d = metrics.getWaitingTimeDistribution();

	REQUIRE(d.threshold.has_value());
	REQUIRE(d.dissatisfiedCount().has_value());
	REQUIRE(d.dissatisfiedCount().value() == 0);
	REQUIRE(d.dissatisfiedRate().value() == Catch::Approx(0.0));
	REQUIRE(d.dissatisfied.empty());
}

TEST_CASE("Dissatisfaction is strictly above the threshold, so the exact tie is satisfied", "[metrics]") {
	ServiceThresholds thresholds;
	thresholds.waiting = 5.0;
	StandardMetricsCollector metrics = collectorWith(thresholds);
	feedWaits(metrics, {4.0, 5.0, 6.0});		// ids 1, 2, 3

	TimeDistribution d = metrics.getWaitingTimeDistribution();

	REQUIRE(d.dissatisfiedCount().value() == 1);
	REQUIRE(d.dissatisfied == std::vector<unsigned long int>{3});
	REQUIRE(d.dissatisfiedRate().value() == Catch::Approx(1.0 / 3.0));
}

TEST_CASE("The dissatisfied list is in service order, not sorted by id", "[metrics]") {
	ServiceThresholds thresholds;
	thresholds.waiting = 5.0;
	StandardMetricsCollector metrics = collectorWith(thresholds);

	// Served in the order 7, 3, 5; only the first two are dissatisfied. The
	// order matters because it is what makes the list deterministic for a
	// given seed, and sorting by id would hide a hash-order bug rather than
	// prevent one.
	for (const auto& entry : std::vector<std::pair<unsigned long int, double>>{{7, 9.0}, {3, 8.0}, {5, 2.0}}) {
		Customer c(entry.first, "", 0, 0, 5, UP);
		metrics.onCustomerBoarded(c, 1, entry.second);
	}

	TimeDistribution d = metrics.getWaitingTimeDistribution();
	REQUIRE(d.dissatisfied == std::vector<unsigned long int>{7, 3});
}

TEST_CASE("An empty population reports zeros, a count of zero and no rate", "[metrics]") {
	ServiceThresholds thresholds;
	thresholds.waiting = 5.0;
	StandardMetricsCollector metrics = collectorWith(thresholds);

	TimeDistribution d = metrics.getWaitingTimeDistribution();

	REQUIRE(d.count == 0);
	REQUIRE(d.average == Catch::Approx(0.0));
	REQUIRE(d.p50 == Catch::Approx(0.0));
	REQUIRE(d.p90 == Catch::Approx(0.0));
	REQUIRE(d.max == Catch::Approx(0.0));
	REQUIRE(d.dissatisfiedCount().value() == 0);	// nobody was dissatisfied...
	REQUIRE_FALSE(d.dissatisfiedRate().has_value());	// ...and there is no denominator
}

TEST_CASE("A single passenger is their own median, p90 and worst case", "[metrics]") {
	StandardMetricsCollector metrics;
	feedWaits(metrics, {7.5});

	TimeDistribution d = metrics.getWaitingTimeDistribution();

	REQUIRE(d.count == 1);
	REQUIRE(d.p50 == Catch::Approx(7.5));
	REQUIRE(d.p90 == Catch::Approx(7.5));
	REQUIRE(d.max == Catch::Approx(7.5));
}

TEST_CASE("Journey is waiting plus travel, with no period missing", "[metrics]") {
	StandardMetricsCollector metrics;

	Customer c(1, "", /*t=*/2.0, /*at*/0, /*to*/5, UP);
	metrics.onCustomerBoarded(c, 1, /*time=*/7.0);		// waited 5
	metrics.onCustomerAlighted(c, 1, /*time=*/12.0);	// travelled 5

	TimeDistribution waiting = metrics.getWaitingTimeDistribution();
	TimeDistribution journey = metrics.getJourneyTimeDistribution();

	REQUIRE(waiting.average == Catch::Approx(5.0));
	REQUIRE(metrics.getAverageTravelTime() == Catch::Approx(5.0));
	REQUIRE(journey.average == Catch::Approx(10.0));
	// The identity the elevator-traffic literature requires of a complete set
	// of definitions (Barney & Al-Sharif section 6.8.1): journey = waiting +
	// travel, exactly, because a boarding is a single instant here.
	REQUIRE(journey.average == Catch::Approx(waiting.average + metrics.getAverageTravelTime()));
}

TEST_CASE("The journey population is whoever alighted, not whoever boarded", "[metrics]") {
	StandardMetricsCollector metrics;

	Customer arrived(1, "", 0, 0, 5, UP);
	Customer stillRiding(2, "", 0, 0, 5, UP);
	metrics.onCustomerBoarded(arrived, 1, 3.0);
	metrics.onCustomerBoarded(stillRiding, 1, 4.0);
	metrics.onCustomerAlighted(arrived, 1, 9.0);

	// Both waited; only one has a journey. In a real run everybody alights
	// before the run ends, so the two populations are equal and this asymmetry
	// can only show up with hand-fed hooks - which is exactly why it is pinned
	// here rather than left to the oracle.
	REQUIRE(metrics.getWaitingTimeDistribution().count == 2);
	REQUIRE(metrics.getJourneyTimeDistribution().count == 1);
	REQUIRE(metrics.getJourneyTimeDistribution().max == Catch::Approx(9.0));
}

TEST_CASE("The two thresholds are independent of each other", "[metrics]") {
	ServiceThresholds thresholds;
	thresholds.journey = 8.0;		// only the journey one is configured
	StandardMetricsCollector metrics = collectorWith(thresholds);

	Customer c(1, "", 0, 0, 5, UP);
	metrics.onCustomerBoarded(c, 1, 5.0);
	metrics.onCustomerAlighted(c, 1, 10.0);		// journey 10 > 8

	REQUIRE_FALSE(metrics.getWaitingTimeDistribution().threshold.has_value());
	REQUIRE_FALSE(metrics.getWaitingTimeDistribution().dissatisfiedCount().has_value());
	REQUIRE(metrics.getJourneyTimeDistribution().dissatisfiedCount().value() == 1);
}

// ---------------------------------------------------------------------------
// Per-window reading and derived queue. The boundary is derived as the last
// arrival instant; a sample is "peak" if the event that closes it (boarding
// for waiting, alighting for journey) is at or before the boundary; the queue
// is arrivals minus boardings.
// ---------------------------------------------------------------------------

TEST_CASE("Windowed waiting: partition by boarding instant, boundary tie is peak", "[metrics][window]") {
	StandardMetricsCollector metrics;

	// Arrivals at 0, 2, 5 -> T_fim = 5.
	Customer c1(1, "", 0, 0, 5, UP);
	Customer c2(2, "", 2, 1, 4, UP);
	Customer c3(3, "", 5, 2, 6, UP);
	metrics.onCustomerArrived(c1);
	metrics.onCustomerArrived(c2);
	metrics.onCustomerArrived(c3);

	metrics.onCustomerBoarded(c1, 1, 3);	// 3 <= 5 -> peak, waited 3
	metrics.onCustomerBoarded(c2, 1, 5);	// 5 == T_fim -> peak (tie), waited 3
	metrics.onCustomerBoarded(c3, 1, 9);	// 9  > 5 -> drain, waited 4

	auto w = metrics.getWaitingTimeByWindow();
	REQUIRE(w.has_value());
	REQUIRE(w->boundaryTime == Catch::Approx(5.0));
	REQUIRE(w->peak.count == 2);
	REQUIRE(w->drain.count == 1);
	// The partition invariant: nobody is lost or double-counted.
	REQUIRE(w->peak.count + w->drain.count == metrics.getWaitingTimeDistribution().count);
	REQUIRE(w->peak.average == Catch::Approx(3.0));
	REQUIRE(w->drain.average == Catch::Approx(4.0));
}

TEST_CASE("Windowed journey partitions by alighting instant, independently of waiting", "[metrics][window]") {
	StandardMetricsCollector metrics;

	Customer c1(1, "", 0, 0, 5, UP);
	Customer c2(2, "", 2, 1, 4, UP);
	Customer c3(3, "", 5, 2, 6, UP);
	metrics.onCustomerArrived(c1);
	metrics.onCustomerArrived(c2);
	metrics.onCustomerArrived(c3);		// T_fim = 5

	metrics.onCustomerBoarded(c1, 1, 3);
	metrics.onCustomerBoarded(c2, 1, 5);
	metrics.onCustomerBoarded(c3, 1, 9);
	metrics.onCustomerAlighted(c1, 1, 4);	// 4 <= 5 -> journey peak
	metrics.onCustomerAlighted(c2, 1, 8);	// 8  > 5 -> journey drain
	metrics.onCustomerAlighted(c3, 1, 12);	// 12 > 5 -> journey drain

	auto j = metrics.getJourneyTimeByWindow();
	REQUIRE(j.has_value());
	REQUIRE(j->peak.count == 1);
	REQUIRE(j->drain.count == 2);
	REQUIRE(j->peak.count + j->drain.count == metrics.getJourneyTimeDistribution().count);
}

TEST_CASE("Queue: peak backlog uses the inclusive tie, and the end-of-arrivals count", "[metrics][window]") {
	StandardMetricsCollector metrics;

	// c1 waits across the whole run; c2 arrives at the exact instant c1 boards.
	// At that instant the inclusive convention (+1 before -1) counts both as
	// present, so the peak is 2, not 1.
	Customer c1(1, "", 0, 0, 5, UP);
	Customer c2(2, "", 10, 1, 4, UP);
	metrics.onCustomerArrived(c1);
	metrics.onCustomerArrived(c2);			// T_fim = 10
	metrics.onCustomerBoarded(c1, 1, 10);	// boards exactly at T_fim -> peak
	metrics.onCustomerBoarded(c2, 1, 20);	// boards after -> drain

	auto q = metrics.getQueueReport();
	REQUIRE(q.has_value());
	REQUIRE(q->boundaryTime == Catch::Approx(10.0));
	REQUIRE(q->maxQueue == 2);
	REQUIRE(q->queueAtArrivalsEnd == 1);	// only c2 boards after T_fim
}

TEST_CASE("Queue at end of arrivals equals the waiting drain count", "[metrics][window]") {
	StandardMetricsCollector metrics;

	Customer c1(1, "", 0, 0, 5, UP);
	Customer c2(2, "", 2, 1, 4, UP);
	Customer c3(3, "", 5, 2, 6, UP);
	metrics.onCustomerArrived(c1);
	metrics.onCustomerArrived(c2);
	metrics.onCustomerArrived(c3);
	metrics.onCustomerBoarded(c1, 1, 3);
	metrics.onCustomerBoarded(c2, 1, 5);
	metrics.onCustomerBoarded(c3, 1, 9);

	REQUIRE(metrics.getQueueReport()->queueAtArrivalsEnd
		== metrics.getWaitingTimeByWindow()->drain.count);
}

TEST_CASE("A pass-by is not a second arrival: it moves neither T_fim nor the queue", "[metrics][window]") {
	StandardMetricsCollector metrics;

	Customer c1(1, "", 0, 0, 5, UP);
	Customer c2(2, "", 5, 1, 4, UP);
	metrics.onCustomerArrived(c1);
	metrics.onCustomerArrived(c2);			// T_fim = 5, two arrivals only
	metrics.onCustomerPassedBy(c1, 1, 3);	// a full car left without c1 - not an arrival
	metrics.onCustomerBoarded(c2, 1, 6);
	metrics.onCustomerBoarded(c1, 1, 7);

	auto q = metrics.getQueueReport();
	REQUIRE(q->boundaryTime == Catch::Approx(5.0));
	// Two people waited at once (both arrived before either boarded); a leaked
	// pass-by arrival would make this 3.
	REQUIRE(q->maxQueue == 2);
	REQUIRE(q->queueAtArrivalsEnd == 2);	// both board after T_fim = 5
}

TEST_CASE("A single arrival at t=0 makes an instantaneous window: peak empty is a value, not nullopt", "[metrics][window]") {
	StandardMetricsCollector metrics;

	Customer c1(1, "", 0, 0, 5, UP);
	metrics.onCustomerArrived(c1);			// T_fim = 0
	metrics.onCustomerBoarded(c1, 1, 2);	// 2 > 0 -> drain

	auto w = metrics.getWaitingTimeByWindow();
	REQUIRE(w.has_value());					// measured, not "not measured"
	REQUIRE(w->peak.count == 0);			// empty window is count 0, a real answer
	REQUIRE(w->drain.count == 1);
}
