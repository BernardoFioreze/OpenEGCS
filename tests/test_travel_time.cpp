#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <memory>
#include <limits>
#include "Elevator.h"
#include "ElevatorSystem.h"
#include "ITravelTimeStrategy.h"
#include "LinearTravelTimeStrategy.h"
#include "KinematicTravelTimeStrategy.h"
#include "FixedDwellTimeStrategy.h"
#include "GreedyNearestDispatchStrategy.h"

using namespace openEGCS;
using Catch::Approx;

// The travel model as an injected ITravelTimeStrategy. Byte-identity of a
// default run does NOT prove the injection point works: a strategy that is
// constructed and then quietly ignored passes byte-identity too. That is what
// the cases below are for - a pluggable thing that is not actually plugged in
// is the failure mode.

TEST_CASE("LinearTravelTimeStrategy charges distance x time-per-floor", "[travel_time]") {
	LinearTravelTimeStrategy linear(2.0);
	REQUIRE(linear.legDuration(3.0) == Approx(6.0));
	REQUIRE(linear.legDuration(-3.0) == Approx(6.0));	// direction-blind, by design
	REQUIRE(linear.legDuration(0.0) == 0.0);
}

TEST_CASE("LinearTravelTimeStrategy honours the interface's endpoint contract exactly", "[travel_time]") {
	// `displacement(d, legDuration(d)) == d`, bit-exact and not merely close.
	// Item #17's invariant I1 (an ENTER/LEAVE fires with the cabin at that
	// event's floor) is exactly the statement that a position derived from
	// displacement() agrees with a timestamp derived from legDuration().
	const double speeds[] = { 1.0, 0.3, 2.5 };
	const double legs[] = { 1.0, -1.0, 4.0, -7.0, 0.5, 0.0 };
	for (double speed : speeds) {
		LinearTravelTimeStrategy linear(speed);
		for (double d : legs) {
			REQUIRE(linear.displacement(d, linear.legDuration(d)) == d);
			// Clamped on both sides: not started, and overrun.
			REQUIRE(linear.displacement(d, -1.0) == 0.0);
			REQUIRE(linear.displacement(d, linear.legDuration(d) + 100.0) == d);
		}
	}
}

TEST_CASE("LinearTravelTimeStrategy interpolates at constant speed inside a leg", "[travel_time]") {
	LinearTravelTimeStrategy linear(1.0);
	REQUIRE(linear.displacement(4.0, 1.0) == Approx(1.0));
	REQUIRE(linear.displacement(4.0, 2.0) == Approx(2.0));
	REQUIRE(linear.displacement(-4.0, 2.0) == Approx(-2.0));	// sign follows the leg
}

namespace {

// A travel model that takes twice as long as the linear one for the same
// distance. Deliberately *not* a plausible lift: the point is to be
// unmistakable in the output, so that "the injected strategy was ignored"
// cannot look like a rounding difference.
class DoubleTimeTravelStrategy : public ITravelTimeStrategy {
	public:
		double legDuration(double signedFloors) const override {
			return std::abs(signedFloors) * 2.0;
		}
		double displacement(double signedFloors, double elapsed) const override {
			double total = std::abs(signedFloors);
			double covered = elapsed / 2.0;
			if (covered < 0) covered = 0;
			if (covered > total) covered = total;
			return (signedFloors >= 0) ? covered : -covered;
		}
		double remainingDuration(double signedFloors, double elapsed) const override {
			double total = legDuration(signedFloors);
			if (elapsed <= 0.0) return total;
			return (elapsed >= total) ? 0.0 : total - elapsed;
		}
		// Constant speed like the linear model, so nothing is ever committed:
		// the shortest leg still available is whatever has been covered.
		double minimumLegDistance(double signedFloors, double elapsed) const override {
			return std::abs(displacement(signedFloors, elapsed));
		}
		const char* name() const override { return "test-double-time"; }
		TravelDescription describe() const override { TravelDescription d; d.model = name(); return d; }
};

} // namespace

TEST_CASE("the injected travel model drives route arrival times", "[travel_time]") {
	// getStopArrivalTime is where event timestamps come from, so this is the
	// call site that matters most. With the default (linear, 1 unit/floor) a
	// cabin at floor 0 reaches floor 5 at t=5; with the doubling model, t=10.
	Task toFive(0, "", GO, DEFAULT, 5, 0.0);
	Task toSeven(0, "", GO, DEFAULT, 7, 0.0);

	Elevator defaultModel(1, "", /*pos=*/0, /*time=*/1.0);
	unsigned long int d5 = defaultModel.addTask(toFive, 1).getStopId();
	Task toSevenCopy = toSeven;
	unsigned long int d7 = defaultModel.addTask(toSevenCopy, 1).getStopId();
	REQUIRE(defaultModel.getStopArrivalTime(d5) == Approx(5.0));
	REQUIRE(defaultModel.getStopArrivalTime(d7) == Approx(7.0));

	Elevator doubled(1, "", /*pos=*/0, /*time=*/1.0, nullptr,
		std::numeric_limits<double>::infinity(), nullptr, nullptr,
		std::make_shared<DoubleTimeTravelStrategy>());
	Task toFiveAgain = toFive, toSevenAgain = toSeven;
	unsigned long int x5 = doubled.addTask(toFiveAgain, 1).getStopId();
	unsigned long int x7 = doubled.addTask(toSevenAgain, 1).getStopId();
	REQUIRE(doubled.getStopArrivalTime(x5) == Approx(10.0));
	REQUIRE(doubled.getStopArrivalTime(x7) == Approx(14.0));
}

TEST_CASE("the injected travel model drives the dispatch cost", "[travel_time]") {
	// getFinishTime is the cost an IDispatchStrategy ranks elevators by. If
	// this call site ignored the travel model, a custom model would time events
	// correctly and still dispatch on the wrong cost.
	//
	// Driven exactly as GreedyNearestDispatchStrategy drives it: getFinishTime
	// sums the route up to a task that is already *in* the route, so the cost
	// of a candidate is obtained by a dry-run insertion (addTask with flag 0)
	// first. Asking before inserting returns the route start time - which is 0
	// for an idle cabin, and would pass while proving nothing.
	Task toFive(0, "", GO, DEFAULT, 5, 0.0);

	Elevator defaultModel(1, "", 0, 1.0);
	Elevator doubled(1, "", 0, 1.0, nullptr, std::numeric_limits<double>::infinity(),
		nullptr, nullptr, std::make_shared<DoubleTimeTravelStrategy>());

	Task a = toFive, b = toFive;
	Task probeA = defaultModel.addTask(a, 0);
	Task probeB = doubled.addTask(b, 0);
	REQUIRE(defaultModel.getFinishTime(probeA) == Approx(5.0));
	REQUIRE(doubled.getFinishTime(probeB) == Approx(10.0));
}

TEST_CASE("the injected travel model drives the cabin's position", "[travel_time]") {
	// Elevator::update is the third call site, and the one that needs
	// displacement over time, not just a duration. Halfway through a 4-floor leg
	// in wall-clock terms is floor 2 under the linear model and floor 1 under
	// the doubling one.
	Task toFour(0, "", GO, DEFAULT, 4, 0.0);

	Elevator defaultModel(1, "", 0, 1.0);
	Task a = toFour;
	defaultModel.addTask(a, 1);
	defaultModel.update(Timer(2.0));
	REQUIRE(defaultModel.getPosition() == Approx(2.0));

	Elevator doubled(1, "", 0, 1.0, nullptr, std::numeric_limits<double>::infinity(),
		nullptr, nullptr, std::make_shared<DoubleTimeTravelStrategy>());
	Task b = toFour;
	doubled.addTask(b, 1);
	doubled.update(Timer(2.0));
	REQUIRE(doubled.getPosition() == Approx(1.0));
}

TEST_CASE("ElevatorSystem shares one travel model with every cabin", "[travel_time]") {
	// Same guarantee ICapacityStrategy and IDwellTimeStrategy already have, and
	// here it is the strongest of the three: cabins in one group run in the
	// same building, so they cannot disagree about what a floor costs.
	auto shared = std::make_shared<DoubleTimeTravelStrategy>();
	ElevatorSystem system(3, 10, 1.0, std::make_shared<examples::GreedyNearestDispatchStrategy>(),
		nullptr, std::numeric_limits<double>::infinity(), nullptr, nullptr, shared);

	for (const Elevator& e : system.getElevators()) {
		REQUIRE(e.getTravelTimeStrategy() == shared);
	}
}

TEST_CASE("explicitly injecting the linear model reproduces the default", "[travel_time]") {
	// LinearTravelTimeStrategy injected reproduces the constant-speed timing
	// exactly.
	Task toSix(0, "", GO, DEFAULT, 6, 0.0);
	Task toTwo(0, "", GO, DEFAULT, 2, 0.0);

	Elevator byDefault(1, "", 0, 1.0);
	Elevator injected(1, "", 0, 1.0, nullptr, std::numeric_limits<double>::infinity(),
		nullptr, nullptr, std::make_shared<LinearTravelTimeStrategy>(1.0));

	Task a1 = toSix, a2 = toTwo, b1 = toSix, b2 = toTwo;
	unsigned long int da = byDefault.addTask(a1, 1).getStopId();
	unsigned long int db = byDefault.addTask(a2, 1).getStopId();
	unsigned long int ia = injected.addTask(b1, 1).getStopId();
	unsigned long int ib = injected.addTask(b2, 1).getStopId();

	REQUIRE(injected.getStopArrivalTime(ia) == byDefault.getStopArrivalTime(da));
	REQUIRE(injected.getStopArrivalTime(ib) == byDefault.getStopArrivalTime(db));
}

// ---------------------------------------------------------------------------
// The kinematic model, and the two assumptions a linear clock hides.
// ---------------------------------------------------------------------------

TEST_CASE("KinematicTravelTimeStrategy charges stopping once per leg, not per floor", "[travel_time]") {
	// Nine one-floor hops must cost strictly more than one nine-floor run,
	// because each leg pays its own acceleration and deceleration. The linear
	// model charges the same for both, which is what makes a clock blind to the
	// cost of stopping.
	KinematicTravelTimeStrategy kinematic(3.5, defaultDynamicsFor(31.5));
	double nineHops = 9 * kinematic.legDuration(1.0);
	double oneRun = kinematic.legDuration(9.0);
	REQUIRE(nineHops > oneRun);
	REQUIRE(nineHops == Approx(46.4903).epsilon(1e-4));
	REQUIRE(oneRun == Approx(22.5764).epsilon(1e-4));

	LinearTravelTimeStrategy linear(5.0);
	REQUIRE(9 * linear.legDuration(1.0) == Approx(linear.legDuration(9.0)));	// the defect, pinned
}

TEST_CASE("KinematicTravelTimeStrategy honours the exact endpoint contract", "[travel_time]") {
	// `(d * h) / h` is not `d` in IEEE 754, so the endpoint has to be returned
	// literally. Item #17's invariant I1 depends on it being exact.
	const double heights[] = { 3.5, 3.0, 2.9 };
	const double legs[] = { 1.0, -1.0, 0.1, 4.0, -9.0, 0.5 };
	for (double h : heights) {
		KinematicTravelTimeStrategy kinematic(h, defaultDynamicsFor(h * 9));
		for (double d : legs) {
			REQUIRE(kinematic.displacement(d, kinematic.legDuration(d)) == d);
			REQUIRE(kinematic.displacement(d, -1.0) == 0.0);
			REQUIRE(kinematic.displacement(d, kinematic.legDuration(d) * 2) == d);
		}
	}
}

TEST_CASE("a kinematic cabin never passes the floor it is going to", "[travel_time][regression]") {
	// Below Peters' Condition C bound, reached the way the simulator actually
	// reaches it: a leg re-anchored mid-flight is shorter than one floor. A
	// profile approximating Condition C by Case II travels further than asked
	// there - a leg of 0.12239630 floors would report 0.17297082 floors covered,
	// +41%, putting the cabin past its destination.
	//
	// The clamp in displacement() stays live for an unrelated reason (the
	// metres-to-floors round trip, see displacement()'s own comment), so a clamped
	// overshoot would also pass a "never past the destination" check. What
	// distinguishes a correct profile is the strict monotonicity below: with the
	// Case II approximation the per-slice sum of displacementAt goes BACKWARDS
	// inside a sub-bound leg.
	KinematicTravelTimeStrategy kinematic(3.5, LiftDynamics{ 2.50, 0.9, 1.00 });
	const double shortLegs[] = { 0.12239630369082211, 0.05, -0.05, 0.3, -0.3 };
	for (double d : shortLegs) {
		double total = kinematic.legDuration(d);
		double prev = 0.0;
		const int samples = 200;
		for (int i = 0; i <= samples; ++i) {
			// Strictly inside the leg, so the endpoint shortcut (which returns
			// `signedFloors` literally) never answers for us.
			double elapsed = total * i / static_cast<double>(samples + 1);
			double covered = kinematic.displacement(d, elapsed);

			REQUIRE(std::abs(covered) <= std::abs(d));
			// Never moves backwards: same sign as the leg, or still at zero.
			// (`(covered >= 0) == (d >= 0)` was the first draft of this and is
			// wrong - at elapsed 0 a downward leg has covered +0.0.)
			REQUIRE(covered * d >= 0.0);
			// Strictly increasing in magnitude after the first sample: no
			// saturation, which is what a clamp would look like here.
			if (i > 1) REQUIRE(std::abs(covered) > std::abs(prev));
			prev = covered;
		}
		// And it has NOT arrived yet just before the end, which a clamp could
		// not promise - it reported arrival early, by flattening the overshoot.
		REQUIRE(std::abs(prev) < std::abs(d));
		// But it does arrive, exactly, at the end.
		REQUIRE(kinematic.displacement(d, total) == d);
	}
}

TEST_CASE("remainingDuration agrees with displacement about when a leg ends", "[travel_time]") {
	// Timing a route by re-pricing "the distance left" as a fresh leg is only
	// valid for a model additive in distance. These two must agree about the
	// instant the leg finishes, or an event fires with the cabin somewhere else.
	KinematicTravelTimeStrategy kinematic(3.5, defaultDynamicsFor(31.5));
	LinearTravelTimeStrategy linear(1.0);
	const double legs[] = { 1.0, -1.0, 4.0, -9.0 };
	for (double d : legs) {
		for (const ITravelTimeStrategy* model : { (const ITravelTimeStrategy*)&kinematic,
		                                          (const ITravelTimeStrategy*)&linear }) {
			double total = model->legDuration(d);
			REQUIRE(model->remainingDuration(d, -1.0) == Approx(total));
			REQUIRE(model->remainingDuration(d, total) == Approx(0.0).margin(1e-12));
			REQUIRE(model->remainingDuration(d, total * 2) == Approx(0.0).margin(1e-12));
			// Mid-leg, the two halves have to add up to the whole: that is what
			// makes a route timed through remainingDuration consistent with a
			// position derived through displacement.
			double mid = total * 0.5;
			REQUIRE(model->remainingDuration(d, mid) == Approx(total - mid).epsilon(1e-9));
		}
	}
}

TEST_CASE("a kinematic leg is not additive, and the engine no longer assumes it is", "[travel_time][regression]") {
	// Splitting a leg and pricing the halves separately overcharges, because
	// each half pays its own start and stop. This is the identity the engine
	// relied on without writing it down; the assertion exists so that a future
	// travel model cannot quietly reintroduce the assumption.
	KinematicTravelTimeStrategy kinematic(3.5, defaultDynamicsFor(31.5));
	double whole = kinematic.legDuration(4.0);
	double halves = kinematic.legDuration(2.0) + kinematic.legDuration(2.0);
	REQUIRE(halves > whole);

	// And the linear model is additive, which is exactly why it hid this.
	LinearTravelTimeStrategy linear(1.0);
	REQUIRE(linear.legDuration(2.0) + linear.legDuration(2.0) == Approx(linear.legDuration(4.0)));
}

TEST_CASE("a leg in flight keeps the arrival time it was given", "[travel_time][regression]") {
	// The precise guard for the non-additivity trap, and it exists because
	// mutation testing found the gap: after the overshoot clamp went in,
	// reverting Elevator::legInFlightRemainder to "price the distance left as a
	// fresh leg" still passed all 155 other cases. The clamp hides the symptom
	// that invariant I1 had caught (a cabin past its floor) while leaving the
	// cause (an event timed early) in place.
	//
	// The invariant: once a leg has started, re-asking when the cabin arrives
	// must give the same answer, however far along the leg the question is
	// asked. A route timed by re-pricing the remaining distance fails this for
	// any model that charges a start and a stop.
	auto kinematic = std::make_shared<KinematicTravelTimeStrategy>(3.5, defaultDynamicsFor(31.5));
	Elevator e(1, "", /*pos=*/0, /*time=*/1.0, nullptr,
		std::numeric_limits<double>::infinity(), nullptr, nullptr, kinematic);

	Task toFive(0, "", GO, DEFAULT, 5, 0.0);
	unsigned long int stop = e.addTask(toFive, 1).getStopId();
	const double expected = e.getStopArrivalTime(stop);
	REQUIRE(expected == Approx(kinematic->legDuration(5.0)));

	// Ask again from a dozen points along the leg, including just before the end
	// where the remaining distance is small and a fresh-leg price is worst.
	for (int i = 1; i <= 12; ++i) {
		double t = expected * i / 13.0;
		e.update(Timer(t));
		REQUIRE(e.getStopArrivalTime(stop) == Approx(expected).epsilon(1e-9));
	}

	// And the cabin is at the destination exactly when that time arrives -
	// which is the other half of I1, asserted here against one leg rather than
	// against a whole random workload.
	e.update(Timer(expected));
	REQUIRE(e.getPosition() == Approx(5.0));
}

// ---------------------------------------------------------------------------
// minimumLegDistance: Peters' d_min (section 6.6, eq 6.103-6.112), the shortest
// TOTAL leg a cabin can still make, given it is already `elapsed` into one. It
// is what tells the engine whether a nearer call that just arrived can become
// the cabin's next stop at all. These cases check the equations directly.
// ---------------------------------------------------------------------------

TEST_CASE("the linear model never commits a cabin to its destination", "[travel_time]") {
	// A constant-speed cabin stops dead at any instant, so d_min is just what
	// it has covered and no retarget is ever refused: with this model,
	// re-anchoring a leg mid-flight and not re-anchoring it are
	// indistinguishable.
	LinearTravelTimeStrategy linear(1.0);
	REQUIRE(linear.minimumLegDistance(5.0, 0.0) == 0.0);
	REQUIRE(linear.minimumLegDistance(5.0, 2.0) == Approx(2.0));
	REQUIRE(linear.minimumLegDistance(-5.0, 2.0) == Approx(2.0));	// magnitude
	REQUIRE(linear.minimumLegDistance(5.0, 5.0) == Approx(5.0));	// arrived: committed
	REQUIRE(linear.minimumLegDistance(5.0, 99.0) == Approx(5.0));	// never exceeds the leg
}

TEST_CASE("minimumLegDistance matches Peters' per-phase d_min", "[travel_time][validation]") {
	// Three anchors on a 3-floor leg with the small reference scenario's
	// dynamics (v=1.60, a=0.80, j=0.90). Each is checked against the thesis'
	// form, written out here rather than reusing the implementation's
	// expression.
	const double h = 3.5;
	LiftDynamics dyn{ 1.60, 0.8, 0.90 };
	KinematicTravelTimeStrategy kin(h, dyn);
	double a = dyn.accelMs2, j = dyn.jerkMs3, v = dyn.ratedSpeedMs;
	double legFloors = 3.0;				// 10.5 m: Condition A, has a cruise
	KinematicProfile p = buildProfile(legFloors * h, v, a, j);
	REQUIRE(p.t4 > 0.0);

	// Phase 1, the first jerk ramp: eq 6.103, d_min = 2*t^3*j.
	double t = p.t1 * 0.5;
	REQUIRE(kin.minimumLegDistance(legFloors, t) == Approx(2.0 * t*t*t * j / h).epsilon(1e-12));

	// Phase 2, at rated acceleration: the DERIVED form (see the header on why
	// the thesis' eq 6.104 is not transcribed), d_min = a*t^2 + a^2*t/j.
	t = p.t1 + p.t2 * 0.5;
	double phase2M = a*t*t + (a*a*t)/j;
	REQUIRE(phase2M == Approx(2.69619).epsilon(1e-4));			// the anchor
	REQUIRE(kin.minimumLegDistance(legFloors, t) == Approx(phase2M / h).epsilon(1e-12));

	// Phase 4, cruising: eq 6.106, d_min = t*v. In Condition A the attained
	// peak IS rated speed, so the thesis' `v` and the profile's peak coincide.
	t = 2*p.t1 + p.t2 + p.t4 * 0.5;
	REQUIRE(kin.minimumLegDistance(legFloors, t) == Approx(t * v / h).epsilon(1e-12));
	REQUIRE(t * v == Approx(7.56111).epsilon(1e-4));			// the anchor

	// Phase 3, between the two: eq 6.105, d_min = 2*D(t3), which for a leg that
	// reaches rated speed is the Condition A threshold v*a/j + v^2/a.
	t = p.t1 + p.t2 + p.t1 * 0.5;
	REQUIRE(kin.minimumLegDistance(legFloors, t) == Approx((v*a/j + v*v/a) / h).epsilon(1e-12));

	// Past the start of deceleration: eq 6.107, no retarget at all.
	REQUIRE(kin.minimumLegDistance(legFloors, 2*p.t1 + p.t2 + p.t4 + p.t1*0.5)
		== Approx(legFloors).epsilon(1e-12));
}

TEST_CASE("minimumLegDistance is continuous at every phase boundary", "[travel_time][validation]") {
	// Not tidiness: the engine will accept or refuse a retarget by comparing a
	// distance against this number, so a jump at a boundary would make that
	// decision turn on a rounding. The branches are algebraically equal at each
	// boundary (at T2 the retarget peak a*T2 IS the profile's peak; at T3 the
	// cruise form t*peak equals the trapezoid form because T3 == a/j + peak/a),
	// and this is what keeps them that way.
	const double h = 3.5;
	const LiftDynamics dyns[] = { { 1.60, 0.8, 0.90 }, { 5.00, 1.2, 1.50 } };
	const double legs[] = { 0.2, 0.5, 1.0, 3.0, 9.0, 19.0 };	// Conditions C, B and A
	for (const LiftDynamics& dyn : dyns) {
		KinematicTravelTimeStrategy kin(h, dyn);
		for (double leg : legs) {
			KinematicProfile p = buildProfile(leg * h, dyn.ratedSpeedMs, dyn.accelMs2, dyn.jerkMs3);
			const double bounds[] = { p.t1, p.t1 + p.t2, 2*p.t1 + p.t2, 2*p.t1 + p.t2 + p.t4 };
			for (double b : bounds) {
				if (b <= 0.0) continue;
				double below = kin.minimumLegDistance(leg, b * (1 - 1e-9));
				double above = kin.minimumLegDistance(leg, b * (1 + 1e-9));
				INFO("dyn v=" << dyn.ratedSpeedMs << " leg=" << leg << " boundary=" << b);
				REQUIRE(below == Approx(above).margin(1e-7));
			}
		}
	}
}

TEST_CASE("minimumLegDistance honours its two contracts over a whole leg", "[travel_time]") {
	// Monotone non-decreasing (a cabin's options only narrow as it travels) and
	// never greater than the leg itself, with equality once the destination can
	// no longer change. Swept across all three conditions, up and down.
	const double h = 3.5;
	const LiftDynamics dyns[] = { { 1.60, 0.8, 0.90 }, { 5.00, 1.2, 1.50 } };
	const double legs[] = { 0.05, 0.2, 0.5, 1.0, 3.0, 9.0, -0.2, -3.0, -19.0 };
	for (const LiftDynamics& dyn : dyns) {
		KinematicTravelTimeStrategy kin(h, dyn);
		for (double leg : legs) {
			double total = kin.legDuration(leg);
			double prev = 0.0;
			for (int i = 0; i <= 400; ++i) {
				double elapsed = total * i / 400.0;
				double dMin = kin.minimumLegDistance(leg, elapsed);
				INFO("leg=" << leg << " elapsed=" << elapsed << " d_min=" << dMin);
				REQUIRE(dMin >= prev - 1e-12);				// monotone
				REQUIRE(dMin <= std::abs(leg) + 1e-12);		// never past the leg
				prev = dMin;
			}
			// Arrived: fully committed, exactly.
			REQUIRE(kin.minimumLegDistance(leg, total) == Approx(std::abs(leg)).epsilon(1e-12));
		}
	}
}

TEST_CASE("a Condition C leg is committed from its first jerk ramp onwards", "[travel_time][validation]") {
	// Peters eq 6.111/6.112: in Condition C, d_min = 2*t^3*j while 0<=t<=t1 and
	// d_min = d from t1 onwards. Worth its own case because the implementation
	// has no Condition C branch - the result falls out of the general phases,
	// since t2 = t4 = 0 collapses them. If that ever stops being true this
	// fails, which is the point.
	const double h = 3.5;
	LiftDynamics dyn{ 1.60, 0.8, 0.90 };
	KinematicTravelTimeStrategy kin(h, dyn);
	double leg = 0.2;						// 0.7 m, below the 1.2642 m bound
	KinematicProfile p = buildProfile(leg * h, dyn.ratedSpeedMs, dyn.accelMs2, dyn.jerkMs3);
	REQUIRE(p.t2 == 0.0);
	REQUIRE(p.t4 == 0.0);

	double t = p.t1 * 0.5;
	REQUIRE(kin.minimumLegDistance(leg, t)
		== Approx(2.0 * t*t*t * dyn.jerkMs3 / h).epsilon(1e-12));	// eq 6.111

	REQUIRE(kin.minimumLegDistance(leg, p.t1 * 1.5) == Approx(leg).epsilon(1e-12));	// eq 6.112
	REQUIRE(kin.minimumLegDistance(leg, p.t1 * 3.5) == Approx(leg).epsilon(1e-12));
}

// ---------------------------------------------------------------------------
// Retargeting a leg in flight. A leg under way keeps its origin and its clock
// and changes only its destination (Peters section 6.6), instead of being
// restarted from rest at the cabin's current position.
//
// The scenario all three cases below are built on, worked out from the
// kinematics rather than guessed. A 0 -> 9 leg with the 10-floor reference
// scenario's dynamics (v=1.60, a=0.80, j=0.90, floor 3.5 m) is Condition A and
// takes 22.576 s. At t = 10 s the cabin is cruising, 13.689 m up (floor 3.911),
// and needs 2.311 m to stop - so the nearest floor it can still stop at is
// 16.0 m, floor 4.571. Floor 5 is therefore reachable and floor 4 is NOT,
// even though floor 4 is still ahead of the cabin.
// ---------------------------------------------------------------------------

namespace {

const LiftDynamics kSmallDyn = { 1.60, 0.8, 0.90 };

std::shared_ptr<ITravelTimeStrategy> smallKinematic() {
	return std::make_shared<KinematicTravelTimeStrategy>(3.5, kSmallDyn);
}

// Numerical velocity in floors per second, read the only way a caller can:
// position at two instants. update() recomputes position analytically from the
// leg's anchors, so calling it repeatedly is not destructive.
double cabinVelocity(Elevator& e, double t) {
	const double h = 1e-6;
	e.update(Timer(t));
	double p0 = e.getPosition();
	e.update(Timer(t + h));
	double p1 = e.getPosition();
	e.update(Timer(t));			// leave the cabin where it was
	return (p1 - p0) / h;
}

int routeFront(const Elevator& e) {
	return e.getRoute().empty() ? -1 : e.getRoute().front().getDest();
}

bool routeContains(const Elevator& e, int floor) {
	for (const Task& t : e.getRoute()) if (t.getDest() == floor) return true;
	return false;
}

} // namespace

TEST_CASE("retargeting a leg in flight does not change the cabin's velocity", "[travel_time][regression]") {
	// The velocity is continuous across a retarget. Re-anchoring the leg at the
	// cabin's current position would restart the motion profile from rest, so
	// the velocity would drop to zero in an instant and be re-earned for free -
	// at every retarget, with jumps up to the rated speed.
	Elevator e(1, "", 0, 1.0, nullptr, std::numeric_limits<double>::infinity(),
		nullptr, nullptr, smallKinematic());
	Task toNine(0, "", GO, DEFAULT, 9, 0.0);
	e.addTask(toNine, 1);

	double before = cabinVelocity(e, 10.0);
	REQUIRE(before > 0.0);							// genuinely moving, or this proves nothing
	REQUIRE(e.getPosition() == Approx(13.689 / 3.5).epsilon(1e-3));

	// Floor 5 is reachable from here (17.5 m against a 16.0 m minimum), so it
	// takes over the leg.
	Task toFive(0, "", GO, DEFAULT, 5, 0.0);
	e.addTask(toFive, 1);
	REQUIRE(routeFront(e) == 5);					// the retarget really happened

	double after = cabinVelocity(e, 10.0);
	REQUIRE(after == Approx(before).epsilon(1e-9));
}

TEST_CASE("a cabin is not given a stop it can no longer brake for", "[travel_time][regression]") {
	// Floor 4 is still AHEAD of the cruising cabin (which is at floor 3.911) and
	// is nevertheless unreachable, because stopping needs 2.311 m of room and
	// floor 4 is 0.311 m away. So the cabin carries on to floor 9 and serves
	// floor 4 afterwards - which is what a real lift does when you watch it go
	// past your floor, and what Closs' rule 4 requires, since braking for it
	// would mean reversing while carrying passengers.
	Elevator e(1, "", 0, 1.0, nullptr, std::numeric_limits<double>::infinity(),
		nullptr, nullptr, smallKinematic());
	Task toNine(0, "", GO, DEFAULT, 9, 0.0);
	e.addTask(toNine, 1);
	e.update(Timer(10.0));

	Task toFour(0, "", GO, DEFAULT, 4, 0.0);
	e.addTask(toFour, 1);

	REQUIRE(routeFront(e) == 9);					// not promoted to the next stop
	REQUIRE(routeContains(e, 4));					// and not lost either
	// It is served on the way back, so it comes after floor 9 in the route.
	REQUIRE(e.getRoute().size() == 2);
	REQUIRE(e.getRoute().back().getDest() == 4);
}

TEST_CASE("the linear model never refuses a retarget", "[travel_time][regression]") {
	// Same setup, linear model: a constant-velocity cabin needs no room to
	// stop, so minimumLegDistance is just what it has covered and every nearer
	// floor is reachable. That is not a quirk - it is the reason this defect was
	// invisible for the whole life of the project, and the reason injecting the
	// linear model still reproduces an old baseline: with it, keeping a leg's
	// origin and restarting the leg give the same motion, because the model is
	// additive in distance.
	Elevator e(1, "", 0, 1.0, nullptr, std::numeric_limits<double>::infinity(),
		nullptr, nullptr, std::make_shared<LinearTravelTimeStrategy>(1.0));
	Task toNine(0, "", GO, DEFAULT, 9, 0.0);
	e.addTask(toNine, 1);
	e.update(Timer(3.0));
	REQUIRE(e.getPosition() == Approx(3.0));

	// Floor 4 is one floor ahead and the cabin stops dead on request.
	Task toFour(0, "", GO, DEFAULT, 4, 0.0);
	e.addTask(toFour, 1);
	REQUIRE(routeFront(e) == 4);

	// And the velocity is continuous across it too - trivially, since the model
	// has no acceleration to be discontinuous in. Worth asserting so that the
	// continuity invariant above is seen to hold for BOTH models.
	REQUIRE(cabinVelocity(e, 3.0) == Approx(1.0).epsilon(1e-6));
}

TEST_CASE("a retarget while the doors are still closing anchors a fresh leg", "[travel_time][dwell]") {
	// The edge to watch. legStartTime is monotone by max() because a cabin
	// sitting out a dwell has it in the FUTURE, so `elapsed` is negative there -
	// which must read as "standing at a floor", not as "in flight". If it read
	// as in flight, the retarget would preserve an origin the cabin has not left
	// yet and the departure would be dated through a shut door.
	auto dwell = std::make_shared<FixedDwellTimeStrategy>(2.0, 1.0, 0.0);
	Elevator e(1, "", 0, 1.0, nullptr, std::numeric_limits<double>::infinity(),
		nullptr, dwell, smallKinematic());
	Task toNine(0, "", GO, DEFAULT, 9, 0.0);
	unsigned long int nine = e.addTask(toNine, 1).getStopId();

	// Serve floor 9, which leaves its door-closing tail owed.
	e.update(Timer(e.getStopArrivalTime(nine)));
	e.processNextTask(nine, false);

	// A new call arrives while those doors are shutting. The cabin is standing
	// at floor 9, so this is a departure and not a retarget: the leg anchors
	// here, and its start time may not be pulled back before the doors shut.
	Task toFive(0, "", GO, DEFAULT, 5, 0.0);
	unsigned long int five = e.addTask(toFive, 1).getStopId();
	REQUIRE(routeFront(e) == 5);
	REQUIRE(e.getStopArrivalTime(five) > e.getStopArrivalTime(nine));
}
