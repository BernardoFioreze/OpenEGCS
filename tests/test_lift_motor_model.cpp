#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "LiftMotorModel.h"

using namespace openEGCS;
using Catch::Approx;

// Characterization tests for the electromechanical energy model (LiftMotorModel.h).
// Reference model and validated example from Peters, "Vertical
// Transportation Planning in Buildings" (Brunel EngD thesis, 1998),
// Chapter 7 "Motor Modelling" (p. 7-1 to 7-12).

TEST_CASE("loadTorque is zero when car+load exactly balances the counterweight", "[lift_motor]") {
	double tl = loadTorque(/*carMassKg=*/1000, /*loadMassKg=*/300, /*counterweightMassKg=*/1300,
		/*driveSheaveDiameterM=*/0.5, /*gearRatio=*/20, /*ropingRatio=*/1, /*gearEfficiency=*/0.9);
	REQUIRE(tl == Approx(0.0));
}

TEST_CASE("loadTorque is positive when car+load is heavier than the counterweight", "[lift_motor]") {
	double tl = loadTorque(1000, 800, 1300, 0.5, 20, 1, 0.9);
	REQUIRE(tl > 0.0);
}

TEST_CASE("loadTorque is negative when the counterweight is heavier (e.g. empty car)", "[lift_motor]") {
	double tl = loadTorque(1000, 0, 1300, 0.5, 20, 1, 0.9);
	REQUIRE(tl < 0.0);
}

TEST_CASE("equivalentInertia adds the point-mass term to the residual inertia", "[lift_motor]") {
	double residual = 10.0;
	double withNoMass = equivalentInertia(0, 0, 0, 0.5, 20, 1, residual);
	REQUIRE(withNoMass == Approx(residual));

	double withMass = equivalentInertia(1000, 300, 1300, 0.5, 20, 1, residual);
	REQUIRE(withMass > residual);
}

TEST_CASE("motorTorque is load torque alone at constant velocity (zero angular acceleration)", "[lift_motor]") {
	REQUIRE(motorTorque(/*loadTorqueNm=*/60.0, /*inertiaKgM2=*/10.0, /*angularAccelRadS2=*/0.0) == Approx(60.0));
}

TEST_CASE("motorTorque adds inertial torque proportionally to angular acceleration", "[lift_motor]") {
	REQUIRE(motorTorque(60.0, 10.0, 5.0) == Approx(60.0 + 10.0 * 5.0));
}

TEST_CASE("armatureState reproduces Peters eq. 7.9/7.10", "[lift_motor]") {
	double Ra = 0.2, KIf = 1.6, torque = 100.0, omega = 50.0;
	ArmatureState state = armatureState(torque, omega, Ra, KIf);

	REQUIRE(state.current == Approx(torque / KIf));
	REQUIRE(state.voltage == Approx(KIf * omega + (torque / KIf) * Ra));
}

TEST_CASE("power is the product of armature voltage and current", "[lift_motor]") {
	REQUIRE(power(230.0, 62.5) == Approx(230.0 * 62.5));
}

TEST_CASE("tripEnergyJ of a zero-distance trip is zero", "[lift_motor]") {
	LiftDynamics dynamics{ 1.6, 1.0, 1.0 };
	LiftMotorParams params;
	REQUIRE(tripEnergyJ(0.0, 0.0, dynamics, params) == 0.0);
}

TEST_CASE("tripEnergyJ scales up with passenger load", "[lift_motor]") {
	LiftDynamics dynamics{ 1.6, 1.0, 1.0 };
	LiftMotorParams params;

	double emptyEnergy = tripEnergyJ(20.0, 0.0, dynamics, params);
	double loadedEnergy = tripEnergyJ(20.0, 800.0, dynamics, params);
	REQUIRE(loadedEnergy > emptyEnergy);
}

TEST_CASE("tripEnergyJ of a longer run (more cruise time) exceeds a shorter one of the same load", "[lift_motor]") {
	LiftDynamics dynamics{ 1.6, 1.0, 1.0 };
	LiftMotorParams params;

	double shortTrip = tripEnergyJ(4.5, 300.0, dynamics, params);
	double longTrip = tripEnergyJ(45.0, 300.0, dynamics, params);
	REQUIRE(longTrip > shortTrip);
}

TEST_CASE("tripEnergyJ without regeneration never goes negative", "[lift_motor]") {
	// A near-empty car going down with a counterweight much heavier than
	// car+load produces negative instantaneous power during parts of the
	// profile (the descent is assisted); without regeneration that energy
	// must not reduce the total below zero.
	LiftDynamics dynamics{ 1.6, 1.0, 1.0 };
	LiftMotorParams params;
	params.carMassKg = 500;
	params.counterweightMassKg = 1300;
	params.regenerative = false;

	double energy = tripEnergyJ(-20.0, 0.0, dynamics, params);
	REQUIRE(energy >= 0.0);
}

TEST_CASE("tripEnergyJ with regeneration enabled can be lower than without it", "[lift_motor]") {
	LiftDynamics dynamics{ 1.6, 1.0, 1.0 };
	LiftMotorParams nonRegen;
	nonRegen.carMassKg = 500;
	nonRegen.counterweightMassKg = 1300;
	nonRegen.regenerative = false;

	LiftMotorParams regen = nonRegen;
	regen.regenerative = true;

	double nonRegenEnergy = tripEnergyJ(-20.0, 0.0, dynamics, nonRegen);
	double regenEnergy = tripEnergyJ(-20.0, 0.0, dynamics, regen);
	REQUIRE(regenEnergy <= nonRegenEnergy);
}

// Peters' worked example (thesis ch. 7, p. 7-3 to 7-9, Figures 7.2-7.7),
// reproduced to the four significant figures he publishes.
//
// The example is stated the way So's model states it, not the way a
// simulation needs it: TL = 60 Nm (p. 7-5) and J = 10 kg.m^2 (p. 7-6) are
// fixed inputs, not derived from car/load/counterweight masses, which is
// why it goes through tripEnergyFromTorqueJ. The rest of the drive is in
// the text: ds = 0.5 m, gr = 20, rr = 1 (p. 7-4), Ra = 0.2 ohm, KIf = 1.6
// (p. 7-8).
//
// The motion profile is NOT in the text - only in Figures 7.2/7.3 - so it
// was read off the plotted data itself (the figures are vector paths in
// thesis.pdf, quantised to about 0.12 s and 0.016 m/s per step):
//   * omega plateaus at 150 rad/s -> v = 150 * ds / (2 gr rr) = 1.875 m/s;
//   * epsilon holds at +/-30 rad/s^2 -> a = 0.375 m/s^2;
//   * each jerk ramp lasts 1.0 s (t = 0 to 1.0, 5.0 to 6.0) -> j = a/1 s
//     = 0.375 m/s^3;
//   * the run ends at t = 19.33 s, and eq 6.36's journey time
//     d/v + v/a + a/j gives d = 1.875 * (19.33 - 6) = 25 m, Condition A.
// The test comment this replaces read "~1.6 m/s over ~9 m" off the same
// figure; that was a guess, and it is not what the plot data says.
//
// Eq 7.12 integrates P(t) as signed, with no clamp, and Figure 7.7 goes
// negative during the deceleration: the example is a regenerating drive.
// Hence regenerative = true here, unlike LiftMotorParams' default (which
// follows the non-regenerating site of p. 7-11, not this example).
//
// The integral has a closed form, which is what makes the read-off profile
// checkable rather than fitted. Substituting eq 7.5, 7.9 and 7.10 into 7.11,
//   P = TL*omega + J*epsilon*omega + (Ra/KIf^2) * (TL + J*epsilon)^2,
// and over a trip that starts and ends at rest the inertial term integrates
// to J*[omega^2/2] = 0, the cross term 2*TL*J*integral(epsilon) to zero too,
// leaving
//   Et = TL*theta + (Ra/KIf^2) * (TL^2 * T + J^2 * integral(epsilon^2) dt)
//      = 60*2000  + 0.078125 * (3600*19.333 + 100*8400)
//      = 120000   + 71062.5 = 191062.5 J,
// with theta = d * 2 gr rr / ds = 2000 rad, T = 19.333 s, and
// integral(epsilon^2) = 80^2 * integral(a^2) = 6400 * 1.3125. Rounded to
// four figures that is Peters' 1.911e5 J exactly. dEt/dd is about 4950 J/m
// here (24 m gives 1.861e5), so the published figure's +/-50 J pins the
// read-off distance to about +/-1 cm, not just the order of magnitude.
TEST_CASE("tripEnergyFromTorqueJ reproduces Peters' worked example, Et = 1.911e5 J", "[lift_motor][validation]") {
	LiftDynamics dynamics{ /*ratedSpeedMs=*/1.875, /*accelMs2=*/0.375, /*jerkMs3=*/0.375 };
	LiftMotorParams params; // ds, gr, rr, Ra, KIf defaults are this example's values.
	REQUIRE(params.driveSheaveDiameterM == 0.5);
	REQUIRE(params.gearRatio == 20.0);
	REQUIRE(params.ropingRatio == 1.0);
	REQUIRE(params.armatureResistanceOhm == 0.2);
	REQUIRE(params.motorConstant == 1.6);
	params.regenerative = true;

	double energy = tripEnergyFromTorqueJ(/*signedDistanceM=*/25.0,
		/*loadTorqueNm=*/60.0, /*inertiaKgM2=*/10.0, dynamics, params);

	// Peters prints 1.911 x 10^5: four significant figures, so the value he
	// computed lies within half a unit of the last one, +/-50 J. epsilon(0)
	// so that this margin is the whole tolerance, not Approx's default
	// relative one on top of it.
	REQUIRE(energy == Approx(1.911e5).epsilon(0).margin(50.0));

	// And against the closed form, which separates the model from the
	// integrator. The trapezoid rule at dtS = 0.02 s does not put a sample
	// on the profile's phase boundaries, where P(t) has kinks (jerk is
	// finite, so P is continuous but not smooth); measured, it lands 1.34 J
	// (7e-6) above 191062.5.
	REQUIRE(energy == Approx(191062.5).epsilon(0).margin(2.0));
}

TEST_CASE("Peters' worked example is the signed integral: clamping regeneration misses it", "[lift_motor][validation]") {
	// The same trip without regeneration (power clamped at zero, this
	// project's default) costs about 2.575e5 J, 35% above the published
	// figure - so the example validates the regenerative branch of the
	// integration and not the clamp the simulation runs with by default.
	LiftDynamics dynamics{ 1.875, 0.375, 0.375 };
	LiftMotorParams params;
	params.regenerative = false;

	double clamped = tripEnergyFromTorqueJ(25.0, 60.0, 10.0, dynamics, params);
	REQUIRE(clamped > 1.3 * 1.911e5);
}

TEST_CASE("tripEnergyJ is tripEnergyFromTorqueJ fed by loadTorque and equivalentInertia", "[lift_motor][validation]") {
	// What the simulation calls is tripEnergyJ, which derives TL and J from
	// masses (eq 7.3/7.4) - the part Peters' example does not exercise.
	// This ties that path to the validated one: same integration, bit for
	// bit, once TL and J are in hand.
	LiftDynamics dynamics{ 1.6, 1.0, 1.0 };
	LiftMotorParams params;
	double loadKg = 300.0;

	double tl = loadTorque(params.carMassKg, loadKg, params.counterweightMassKg,
		params.driveSheaveDiameterM, params.gearRatio, params.ropingRatio, params.gearEfficiency);
	double inertia = equivalentInertia(params.carMassKg, loadKg, params.counterweightMassKg,
		params.driveSheaveDiameterM, params.gearRatio, params.ropingRatio, params.residualInertiaKgM2);

	for (double d : { 40.0, -40.0, 3.5, -10.5 }) {
		REQUIRE(tripEnergyJ(d, loadKg, dynamics, params)
			== tripEnergyFromTorqueJ(d, tl, inertia, dynamics, params));
	}
}

TEST_CASE("tripEnergyJ of a multi-floor trip with installation masses is of plausible magnitude", "[lift_motor]") {
	// A plausibility check, not a reproduction: the mass-derived TL and J
	// (eq 7.3/7.4) with LiftMotorParams' installation-specific defaults have
	// no published counterpart, so all that can be asked of a 40 m trip with
	// 300 kg aboard is that it lands in the 1e3-1e7 J range, not many orders
	// of magnitude off. The numerical anchor is the worked-example case above.
	LiftDynamics dynamics{ 1.6, 1.0, 1.0 };
	LiftMotorParams params;

	double energy = tripEnergyJ(/*signedDistanceM=*/40.0, /*loadMassKg=*/300.0, dynamics, params);
	REQUIRE(energy > 1e3);
	REQUIRE(energy < 1e7);
}
