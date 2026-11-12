#include "LiftMotorModel.h"

#include <algorithm>
#include <cmath>

namespace openEGCS {

double loadTorque(double carMassKg, double loadMassKg, double counterweightMassKg,
	double driveSheaveDiameterM, double gearRatio, double ropingRatio,
	double gearEfficiency, double gravityMs2)
{
	return (carMassKg + loadMassKg - counterweightMassKg) * gravityMs2 * (driveSheaveDiameterM / 2.0)
		/ (gearRatio * ropingRatio * gearEfficiency);
}

double equivalentInertia(double carMassKg, double loadMassKg, double counterweightMassKg,
	double driveSheaveDiameterM, double gearRatio, double ropingRatio,
	double residualInertiaKgM2)
{
	double gearedRadius = gearRatio * ropingRatio;
	double pointMassTerm = (driveSheaveDiameterM * driveSheaveDiameterM)
		/ (4.0 * gearedRadius * gearedRadius)
		* (carMassKg + counterweightMassKg + loadMassKg);
	return residualInertiaKgM2 + pointMassTerm;
}

double motorTorque(double loadTorqueNm, double inertiaKgM2, double angularAccelRadS2)
{
	return loadTorqueNm + inertiaKgM2 * angularAccelRadS2;
}

ArmatureState armatureState(double torqueNm, double angularVelRadS,
	double armatureResistanceOhm, double motorConstant)
{
	double current = torqueNm / motorConstant;
	double voltage = motorConstant * angularVelRadS + current * armatureResistanceOhm;
	return { voltage, current };
}

double power(double armatureVoltage, double armatureCurrent)
{
	return armatureVoltage * armatureCurrent;
}

double tripEnergyJ(double signedDistanceM, double loadMassKg,
	const LiftDynamics& dynamics, const LiftMotorParams& params, double dtS)
{
	double tl = loadTorque(params.carMassKg, loadMassKg, params.counterweightMassKg,
		params.driveSheaveDiameterM, params.gearRatio, params.ropingRatio, params.gearEfficiency);
	double inertia = equivalentInertia(params.carMassKg, loadMassKg, params.counterweightMassKg,
		params.driveSheaveDiameterM, params.gearRatio, params.ropingRatio, params.residualInertiaKgM2);
	return tripEnergyFromTorqueJ(signedDistanceM, tl, inertia, dynamics, params, dtS);
}

double tripEnergyFromTorqueJ(double signedDistanceM, double tl, double inertia,
	const LiftDynamics& dynamics, const LiftMotorParams& params, double dtS)
{
	KinematicProfile profile = buildProfile(signedDistanceM, dynamics.ratedSpeedMs,
		dynamics.accelMs2, dynamics.jerkMs3);
	if (profile.totalTimeS <= 0.0) return 0.0;

	double gearedTerm = 2.0 * params.gearRatio * params.ropingRatio / params.driveSheaveDiameterM;

	auto instantaneousPower = [&](double t) {
		MotionState state = sampleProfile(profile, t);
		double omega = state.velocityMs * gearedTerm;
		double epsilon = state.accelMs2 * gearedTerm;
		double torque = motorTorque(tl, inertia, epsilon);
		ArmatureState arm = armatureState(torque, omega, params.armatureResistanceOhm, params.motorConstant);
		double p = power(arm.voltage, arm.current);
		return params.regenerative ? p : std::max(0.0, p);
	};

	int steps = std::max(1, static_cast<int>(std::ceil(profile.totalTimeS / dtS)));
	double dt = profile.totalTimeS / steps;

	double energy = 0.0;
	double prevP = instantaneousPower(0.0);
	for (int i = 1; i <= steps; ++i) {
		double t = i * dt;
		double curP = instantaneousPower(t);
		energy += 0.5 * (prevP + curP) * dt;
		prevP = curP;
	}
	return energy;
}

} // namespace openEGCS
