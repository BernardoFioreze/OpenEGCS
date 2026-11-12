/*
LiftMotorModel.h

Electromechanical energy model for a single continuous lift trip (traction
lift, separately-excited DC motor fed from a fully controlled 6-pulse
converter), derived from:

  Peters, R. D., "Vertical Transportation Planning in Buildings", EngD
  thesis, Brunel University, 1998, Chapter 7 "Motor Modelling" (p. 7-1 to
  7-12). Equation numbers referenced below (7.1-7.12) are that chapter's.

The model accounts for load (passenger weight), direction (car + load vs.
counterweight imbalance) and the number of stops (each stop/start incurs an
acceleration/deceleration cost that a pure-distance model misses entirely).

Parameters not sourced from either reference (car mass, counterweight
mass, gear efficiency) are called out explicitly in LiftMotorParams'
comments; they are installation-specific and have no "typical" value in
the literature used for this project.
*/

#ifndef __LIFTMOTORMODEL__
#define __LIFTMOTORMODEL__

#include "LiftKinematics.h"

namespace openEGCS {

struct LiftMotorParams {
	// --- Not sourced from either reference; installation-specific. ---
	double carMassKg = 1000.0;           // Mc, empty car mass.
	double counterweightMassKg = 1300.0; // Mcw (a common rule of thumb is
	                                      // car mass + ~45-50% of rated
	                                      // capacity; NOT from either source).
	double gearEfficiency = 0.9;         // eta (0-1), typical gearbox figure.

	// --- Defaults from Peters' own validated worked example (p. 7-8),
	// cross-checked against another researcher ("So") and a real-site
	// measurement -- the best citable baseline in the absence of real
	// installation data, not "market-typical" values. ---
	double driveSheaveDiameterM = 0.5;  // ds
	double gearRatio = 20.0;            // gr (:1)
	double ropingRatio = 1.0;           // rr (:1)
	double armatureResistanceOhm = 0.2; // Ra
	double motorConstant = 1.6;         // KIf

	// Mechanical-component inertia (motor/brake/gear/sheave/pulleys) plus
	// rope mass, lumped into a single constant, since manufacturer data
	// for the individual components is normally unavailable (eq. 7.4). The
	// value is borrowed from So's J=10 that Peters' example uses -- but
	// there J=10 is the *total* inertia, while here the car+counterweight+
	// load point-mass term is added on top of it.
	//
	// What that means in practice: with the default drive geometry the
	// point-mass term is under 1 kg.m^2, so this constant is ~96% of the
	// inertia, and near balanced load it carries ~97% of a six-floor climb's
	// energy even in Peters' own dynamics. A run's energy therefore counts legs
	// and starts more than load. Halving or zeroing it keeps every controller
	// on the same side of the greedy baseline but can reorder controllers on
	// the same side; read energy comparisons that way. No source gives this
	// value per installation size.
	double residualInertiaKgM2 = 10.0;

	// If false (default), matches the non-regenerative drive Peters
	// measured on a real site (p. 7-11: "this drive is not regenerating"):
	// power is clamped to >= 0 before integrating, i.e. braking energy is
	// assumed dissipated, not returned to the supply. His worked example
	// (Et = 1.911e5 J) is the opposite case, a signed integral, so it is
	// reproduced with this set to true.
	bool regenerative = false;

	// Standby/idle power draw (W) while the car is stationary between
	// trips. Peters measured ~5 kW on one (>20 year old) real site
	// (p. 7-11), but flags it as equipment-specific, not a general
	// constant -- so this defaults to 0 (not modelled) rather than
	// guessing. Set explicitly if you have a real measurement.
	double idlePowerWatts = 0.0;
};

// Load torque (Nm): the static torque from the weight imbalance between
// car+load and counterweight, as "seen" by the motor through the gear and
// roping ratios (eq. 7.3). Positive when car+load is heavier than the
// counterweight. Direction-independent (this is a static imbalance, not
// a function of which way the car is moving).
double loadTorque(double carMassKg, double loadMassKg, double counterweightMassKg,
	double driveSheaveDiameterM, double gearRatio, double ropingRatio,
	double gearEfficiency, double gravityMs2 = 9.81);

// Equivalent moment of inertia as seen by the motor (kg.m^2): the
// mechanical-component term is a lumped constant (residualInertiaKgM2,
// see LiftMotorParams), the car+counterweight+load point-mass term
// (eq. 7.4's last term) is computed exactly.
double equivalentInertia(double carMassKg, double loadMassKg, double counterweightMassKg,
	double driveSheaveDiameterM, double gearRatio, double ropingRatio,
	double residualInertiaKgM2);

// Required motor torque (Nm) at an instant: load torque plus the torque
// needed to accelerate/decelerate the load (eq. 7.5). angularAccelRadS2
// is signed (positive = accelerating upward).
double motorTorque(double loadTorqueNm, double inertiaKgM2, double angularAccelRadS2);

struct ArmatureState {
	double voltage; // Va(t), Volts
	double current; // Ia(t), Amps
};

// Armature voltage/current for the given motor torque and angular
// velocity, assuming an ideal feedback control system (eq. 7.9/7.10).
ArmatureState armatureState(double torqueNm, double angularVelRadS,
	double armatureResistanceOhm, double motorConstant);

// Instantaneous electrical power (W) (eq. 7.11). Can be negative
// (regenerating) -- callers decide whether to clamp per
// LiftMotorParams::regenerative.
double power(double armatureVoltage, double armatureCurrent);

// Total energy (Joules) consumed by a single continuous trip of
// `signedDistanceM` meters (positive = up, negative = down) carrying
// `loadMassKg` of passengers, integrating eq. 7.12 numerically over the
// kinematic profile (LiftKinematics.h) at `dtS`-second steps. If
// `params.regenerative` is false (default), instantaneous power is
// clamped to >= 0 before integrating (see LiftMotorParams::regenerative).
double tripEnergyJ(double signedDistanceM, double loadMassKg,
	const LiftDynamics& dynamics, const LiftMotorParams& params, double dtS = 0.02);

// The same integration with the load torque and the equivalent inertia
// given directly instead of derived from masses (eq. 7.5-7.12 without 7.3
// and 7.4) -- tripEnergyJ is this function fed by loadTorque() and
// equivalentInertia(). It is the form Peters' worked example (p. 7-3 to
// 7-9) is stated in: So's model fixes TL=60 Nm and J=10 kg.m^2 rather than
// deriving them, and this entry point is what lets the test suite reproduce
// that example's Et. Of `params`, only the drive geometry (ds, gr, rr), the
// motor (Ra, KIf) and `regenerative` are read; the masses, gearEfficiency
// and residualInertiaKgM2 are not, because they only enter through TL and J.
double tripEnergyFromTorqueJ(double signedDistanceM, double loadTorqueNm, double inertiaKgM2,
	const LiftDynamics& dynamics, const LiftMotorParams& params, double dtS = 0.02);

} // namespace openEGCS

#endif
