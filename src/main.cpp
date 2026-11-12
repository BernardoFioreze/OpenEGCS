/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : main.cpp

Copyright (C) 2011 Nivvedan, Ankush Das

    liftsim is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    liftsim is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with liftsim.  If not, see <http://www.gnu.org/licenses/>.
    
    
Authors:

Nivvedan
nivvedan@iitb.ac.in

Ankush Das
ankushdas@iitb.ac.in



File Created on : Wednesday, October 12, 2011.
Last modified on : Tuesday, 15 November, 2011.
Modified since 2026 by Bernardo Fioreze as part of OpenEGCS (see NOTICE).

*/

/*
main.cpp - refactored CLI.

New usage:
  liftsim <floors> <elevators> <customers> [arrivalRate=0.5] [speed=5] [sane=true] [seed]

`arrivalRate` is lambda of the Poisson arrival process (expected customer
arrivals per unit of time) - replaces the old min/max inter-arrival window.
`seed` is optional: when provided, the whole run (arrivals, floors,
directions) becomes fully deterministic, so the exact same scenario can be
replayed against different dispatch strategies.

This executable wires up the example GreedyNearestDispatchStrategy (see
examples/greedy_nearest_dispatch) as the dispatch algorithm. To benchmark
your own algorithm, implement openEGCS::IDispatchStrategy and inject it
here instead.
*/

#include <iostream>
#include <string>
#include <sstream>
#include <memory>
#include <optional>
#include <limits>
#include "Simulator.h"
#include "RandomProvider.h"
#include "GreedyNearestDispatchStrategy.h"
#include "PercentageCapacityStrategy.h"
#include "FixedDwellTimeStrategy.h"
#include "StandardMetricsCollector.h"
#include "LiftKinematics.h"
#include "KinematicTravelTimeStrategy.h"
#include "LinearTravelTimeStrategy.h"
#include "TrafficCapacity.h"
#include "RunConfig.h"

using namespace std;
using namespace openEGCS;

template <class T>
bool from_string(T& t, 
                 const string& s, 
                 ios_base& (*f)(ios_base&))
{
  istringstream iss(s);
  return !(iss >> f >> t).fail();
}

void usage (string a) {
	cout<<"Usage (named config): "<< a <<" --config <file.json> "
		<<"-- a JSON file with named keys (floors, elevators, customers, arrivalRate, seed, maxWeightKg, "
		<<"meanWeightKg, stddevWeightKg, bypassFraction, floorHeightM, carMassKg, counterweightMassKg, "
		<<"doorType [\"centre\"|\"side\"|\"none\"], doorOpening [\"normal\"|\"advanced\"], "
		<<"transferProfile [\"office\"|\"residential\"|\"ratedLoad\"], doorHold [\"timed\"|\"detector\"|\"none\"], "
		<<"carCallHoldTime, landingCallHoldTime, "
		<<"doorOpenTime, doorCloseTime, perPassengerTransferTime, waitingThreshold, journeyThreshold, "
		<<"travelModel [\"kinematic\"|\"linear\"], elevatorSpeed [linear only]). floors/elevators/customers "
		<<"are required; every other key defaults exactly as the positional form below, and an unknown key "
		<<"is refused by name. travelModel and the door selectors are the knobs the positional form "
		<<"cannot express.\n";
	cout<<"Usage (positional): "<< a <<" <Number of Floors> <Number of Elevators> <Customers to Simulate> "
		<<"[OPTIONAL : <Poisson arrival rate = 0.5> <Elevator speed = 5> <Whether users are sane = true> <Seed> "
		<<"<Elevator max weight = unlimited> <Mean customer weight = 70> <Stddev customer weight = 15> "
		<<"<Load bypass threshold fraction = 0.6> <Floor height (m) = 3.5> <Car mass (kg) = 1000> "
		<<"<Counterweight mass (kg) = 1300> <Door open time = preset> <Door close time = preset> "
		<<"<Per-passenger transfer time = preset> <Waiting dissatisfaction threshold = unset> "
		<<"<Journey dissatisfaction threshold = unset>]. \n"
		<<"If <Seed> is provided, the run is fully deterministic/reproducible.\n"
		<<"<Elevator max weight> is the same for every elevator; when omitted (or <= 0) there is no capacity "
		<<"limit and load bypass never triggers.\n"
		<<"<Stddev customer weight> may be 0: every passenger then weighs exactly <Mean customer weight>, "
		<<"which expresses a capacity in passengers (e.g. max weight 20, mean 1, stddev 0 = 20 people). "
		<<"A negative value is an error.\n"
		<<"<Floor height>/<Car mass>/<Counterweight mass> feed the energy consumption model; "
		<<"car/counterweight mass have no literature-typical value and default to "
		<<"generic engineering figures, not measured constants.\n"
		<<"<Door open time>/<Door close time>/<Per-passenger transfer time> are how long a stop takes. "
		<<"Omitted, they come from Barney & Al-Sharif 2015's centre-opening "
		<<"door with normal opening (Table 5.3: 2.0 s open / 2.0 s close for a cabin up to 1000 kg, 2.5 / 3.0 s "
		<<"above it or unlimited) and the office transfer time of 1.2 s per passenger (sec. 5.5.1), with the doors "
		<<"held open at least 2.0 s for a car call and 3.0 s for a landing call (sec. 5.5.2). Given, each "
		<<"replaces its preset value; 0 0 0 is the instantaneous stop (no door "
		<<"cycle, so no door to hold open).\n"
		<<"<Waiting dissatisfaction threshold>/<Journey dissatisfaction threshold> are the tolerances a "
		<<"passenger is judged against: anyone whose wait (arrival -> boarding) or journey (arrival -> "
		<<"alighting) is strictly above them is reported as dissatisfied. "
		<<"Omitted or <= 0 means unset, and an unset threshold reports no verdict at all rather than a "
		<<"verdict of zero.\n"
		<<"  UNITS: one simulation time unit is ONE SECOND. Travel time comes from the jerk-limited "
		<<"motion profile of the installation, which works in SI seconds from <Floor height> in metres, "
		<<"and the stop-duration arguments are added to it unscaled - so "
		<<"every time figure in this program, and every time figure it reports, is in seconds. The "
		<<"literature's figures (30 s average wait in an office, 90 s residential - Barney & Al-Sharif "
		<<"section 4.3.2) can be typed in as they are.\n"
		<<"  <Elevator speed> is ACCEPTED AND IGNORED: it is kept so that the sixteen "
		<<"positional arguments after it do not shift, and it is applied only by the linear travel "
		<<"model, which is not the default. Passing it prints a warning. The linear model is selected "
		<<"with a config file (travelModel: \"linear\") or through the library API "
		<<"(inject LinearTravelTimeStrategy).\n"
		<<"  No default dissatisfaction threshold ships, "
		<<"and the distributions (p50, p90, worst case) need no threshold at all.\n";
	}

// Turn a fully-parsed RunConfig into a running simulation. Both entry points -
// the positional argument list and a --config file - fill a RunConfig and hand
// it here, so the strategy wiring, the saturation warning and the Simulator
// construction live in exactly one place and the two front ends cannot drift.
// This is the part that reaches into liftsim_examples (GreedyNearest), which
// is why it is here and not in RunConfig itself.
static int runSimulation(const RunConfig& cfg) {
	shared_ptr<IRandomProvider> rng = make_shared<RandomProvider>(cfg.seed);
	shared_ptr<IDispatchStrategy> dispatchStrategy = make_shared<examples::GreedyNearestDispatchStrategy>();
	shared_ptr<ICapacityStrategy> capacityStrategy = make_shared<PercentageCapacityStrategy>(cfg.bypassFraction);

	LiftDynamics dynamics = defaultDynamicsFor((cfg.floors - 1) * cfg.floorHeightM);
	LiftMotorParams motorParams;
	motorParams.carMassKg = cfg.carMassKg;
	motorParams.counterweightMassKg = cfg.counterweightMassKg;

	ServiceThresholds serviceThresholds;
	serviceThresholds.waiting = cfg.waitingThreshold;
	serviceThresholds.journey = cfg.journeyThreshold;

	shared_ptr<IMetricsCollector> metricsCollector =
		make_shared<StandardMetricsCollector>(/*energy_per_floor=*/1.0, cfg.floorHeightM, dynamics, motorParams,
			serviceThresholds);

	// The stop duration: the cited preset for this cabin, with any explicit
	// time on top. Resolved once, so the saturation warning below prices
	// exactly the doors the run gets.
	const DwellTimes dwell = dwellTimesOf(cfg);
	shared_ptr<IDwellTimeStrategy> dwellStrategy = makeDwellStrategy(dwell);

	// The travel model is the one selector the positional CLI cannot express:
	// a config file is the only front end that can ask for linear. Kinematic
	// is built from the SAME floorHeight and LiftDynamics the metrics
	// collector prices energy with, so time and energy describe one journey;
	// linear reads <elevatorSpeed> as time-per-floor.
	shared_ptr<ITravelTimeStrategy> travelStrategy;
	if (cfg.travelModel == TravelModel::Linear) {
		travelStrategy = make_shared<LinearTravelTimeStrategy>(cfg.elevatorSpeed);
	} else {
		travelStrategy = make_shared<KinematicTravelTimeStrategy>(cfg.floorHeightM, dynamics);
	}

	// Saturation warning. An a-priori sanity ruler, not a verdict on the run:
	// compute the group's up-peak handling capacity from the installation's own
	// physics (Barney & Al-Sharif 2015 ch. 4-5, = Siikonen 2000's
	// HC = 0.8*CC*300/I) and say so when the demand handed in exceeds it. Only
	// meaningful with a finite cabin: an unlimited <maxWeightKg> is elastic and
	// has no handling capacity to exceed, so skip rather than print an infinite
	// capacity.
	if (cfg.maxWeightKg != numeric_limits<double>::infinity() && cfg.floors > 1 && cfg.elevators > 0) {
		const double carCapacityPersons = cfg.maxWeightKg / cfg.meanWeightKg;
		// tf(1) is the leg the chosen model actually charges; tv is transit at
		// rated speed. For the kinematic model the two differ (a stop costs
		// extra), so tv = df/v. For the linear model a floor costs the same
		// stopped or flying, so tv = tf(1) = elevatorSpeed and ts collapses to
		// the door time, which is the correct reading of that model.
		const double singleFloorFlightTf1 = travelStrategy->legDuration(1.0);
		const double singleFloorTransitTv = (cfg.travelModel == TravelModel::Linear)
			? cfg.elevatorSpeed
			: cfg.floorHeightM / dynamics.ratedSpeedMs;
		const double doorOperatingTd = dwell.doorOpenS + dwell.doorCloseS;
		// 0.8 is Barney & Al-Sharif's / Siikonen's design car-loading factor for
		// capacity (handbook symbol UPPHC, "with 80% car loading"), a different
		// quantity from the 60% load-weighing bypass threshold.
		UppeakCapacity cap = estimateUppeakCapacity(
			/*floorsAboveLobby=*/static_cast<int>(cfg.floors) - 1,
			/*cars=*/static_cast<int>(cfg.elevators), carCapacityPersons,
			/*loadFactor=*/0.8, singleFloorTransitTv, singleFloorFlightTf1,
			doorOperatingTd, dwell.perPassengerS);

		// Demand in the same unit the capacity is in: passengers per 5 minutes.
		// The CLI only runs the stationary-uniform profile, so the 5-minute
		// demand is arrivalRate * 300; a peaked profile (injection-only) would
		// need its own peak rate and is out of scope here.
		const double demandPer5Min = cfg.arrivalRate * 300.0;
		if (demandPer5Min > cap.handlingCapacity) {
			cerr << "warning: configured demand (" << demandPer5Min
			     << " passengers/5 min, from arrival rate " << cfg.arrivalRate
			     << "/s) exceeds the estimated up-peak handling capacity ("
			     << cap.handlingCapacity << " passengers/5 min: round trip time "
			     << cap.roundTripTimeS << " s, interval " << cap.intervalS
			     << " s over " << cfg.elevators << " cars). The run will spend a "
			     << "drain phase after the last arrival that dominates the "
			     << "reported averages; read "
			     << "metrics.queue and the peak/drain split, not the global mean. "
			     << "This capacity is an up-peak reference figure (everyone "
			     << "boarding at the lobby); the default traffic is uniform, so "
			     << "treat it as a sizing ruler, not a verdict on this scenario."
			     << endl;
		}
	}

	Simulator sim(1, "_sim1", cfg.floors, cfg.elevators, cfg.customers, cfg.arrivalRate, cfg.elevatorSpeed, cfg.sane,
		rng, dispatchStrategy, metricsCollector, nullptr, cfg.seed,
		cfg.maxWeightKg, capacityStrategy, cfg.meanWeightKg, cfg.stddevWeightKg, dwellStrategy,
		travelStrategy);
	sim.startSimulation();
	return 0;
}

int main (int argc, char *argv[]) {

	// Config-file front end: `liftsim --config scenario.json`. Coexists with
	// the positional form below rather than replacing it. A bad config fails by
	// name on stderr.
	if (argc >= 2 && string(argv[1]) == "--config") {
		if (argc < 3) {
			cerr << "Usage: " << argv[0] << " --config <file.json>" << endl;
			return 1;
		}
		try {
			RunConfig cfg = loadRunConfig(string(argv[2]));
			return runSimulation(cfg);
		} catch (const std::exception& e) {
			cerr << e.what() << endl;
			return 1;
		}
	}

	if ( argc < 4 ) { // argc should be atleast 4

		usage(string(argv[0]));
		return 1;
		}
	
	else {
		unsigned int no_floors, no_elevators, cus_to_simulate;
  
 		if(from_string<unsigned int>(no_floors, string(argv[1]), dec));		
  		else {
  			cout << "Invalid Number of Floors" << endl;
  			usage(string(argv[0]));
  			return 1;
  			}
  		if(from_string<unsigned int>(no_elevators, string(argv[2]), dec));			
  		else {
  			cout << "Invalid Number of Elevators" << endl;
  			usage(string(argv[0]));
  			return 1;
  			}
  		if(from_string<unsigned int>(cus_to_simulate, string(argv[3]), dec));			
  		else {
  			cout << "Invalid Number of Customers to Simulate" << endl;
  			usage(string(argv[0]));
  			return 1;
  			}
  		
  		double arrival_rate;
  		float speed;
  		bool sane;
  		optional<uint64_t> seed = nullopt;
  		
  		if (argc >= 5) {
  			if(from_string<double>(arrival_rate, string(argv[4]), dec)) {
  				if (arrival_rate <= 0) {
  					cout << "Invalid Arrival Rate: must be > 0" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}		
  			else {
  				cout << "Invalid Arrival Rate" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}
  		else {
  			arrival_rate = 0.5;
  			}
  			
  		if (argc >= 6) {
  			if(from_string<float>(speed, string(argv[5]), dec)) {
  				if (speed < 0) {
  					cout << "Invalid Speed: must be >= 0" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}		
  			else {
  				cout << "Invalid Speed" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}
  		else {
  			speed = 5;
  			}
  			
  		if (argc >= 7) {
  			if (string(argv[6]) == "true") sane = true;
  			else if (string(argv[6]) == "false") sane = false;	
  			else {
  				cout << "Invalid Sane boolean" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}
  		else {
  			sane = true;
  			}

  		if (argc >= 8) {
  			uint64_t seed_val;
  			if (from_string<uint64_t>(seed_val, string(argv[7]), dec)) {
  				seed = seed_val;
  				}
  			else {
  				cout << "Invalid Seed" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		double maxWeight = numeric_limits<double>::infinity();
  		if (argc >= 9) {
  			if (from_string<double>(maxWeight, string(argv[8]), dec)) {
  				if (maxWeight <= 0) maxWeight = numeric_limits<double>::infinity();
  				}
  			else {
  				cout << "Invalid Elevator max weight" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		double meanCustomerWeight = 70.0;
  		if (argc >= 10) {
  			if(from_string<double>(meanCustomerWeight, string(argv[9]), dec)) {
  				if (meanCustomerWeight <= 0) {
  					cout << "Invalid Mean customer weight: must be > 0" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}
  			else {
  				cout << "Invalid Mean customer weight" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		// Zero is a legitimate value - every passenger weighs exactly the mean,
  		// which is how a capacity is expressed in PEOPLE rather than kilograms
  		// (`<Elevator max weight> 20 <Mean customer weight> 1 <Stddev> 0` is a
  		// 20-person cabin: a sum of 1.0s is exact, and Elevator::canBoard is <=).
  		// A negative stddev means nothing, and is refused out loud rather than
  		// swapped for a default.
  		double stddevCustomerWeight = 15.0;
  		if (argc >= 11) {
  			if(from_string<double>(stddevCustomerWeight, string(argv[10]), dec)) {
  				if (stddevCustomerWeight < 0) {
  					cout << "Invalid Stddev customer weight: must be >= 0 (0 = every passenger weighs the mean)" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}
  			else {
  				cout << "Invalid Stddev customer weight" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		double bypassFraction = 0.6;
  		if (argc >= 12) {
  			if(from_string<double>(bypassFraction, string(argv[11]), dec)) {
  				if (bypassFraction <= 0 || bypassFraction > 1) {
  					cout << "Invalid Load bypass threshold fraction: must be in (0, 1]" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}
  			else {
  				cout << "Invalid Load bypass threshold fraction" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		// Floor height / car mass / counterweight mass feed the energy
  		// consumption model. Floor height default (3.5 m) is within Barney &
  		// Al-Sharif's stated "modern commercial building" range (p. 88);
  		// car/counterweight mass have no literature-typical value in the
  		// references used - defaults here are generic engineering figures, not
  		// measured or cited constants.
  		double floorHeight = 3.5;
  		if (argc >= 13) {
  			if(from_string<double>(floorHeight, string(argv[12]), dec)) {
  				if (floorHeight <= 0) {
  					cout << "Invalid Floor height: must be > 0" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}
  			else {
  				cout << "Invalid Floor height" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		double carMass = 1000.0;
  		if (argc >= 14) {
  			if(from_string<double>(carMass, string(argv[13]), dec)) {
  				if (carMass <= 0) {
  					cout << "Invalid Car mass: must be > 0" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}
  			else {
  				cout << "Invalid Car mass" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		double counterweightMass = 1300.0;
  		if (argc >= 15) {
  			if(from_string<double>(counterweightMass, string(argv[14]), dec)) {
  				if (counterweightMass <= 0) {
  					cout << "Invalid Counterweight mass: must be > 0" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}
  			else {
  				cout << "Invalid Counterweight mass" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		// Positional arguments 16-18: door open time, door close time and
  		// per-passenger transfer time. Each one overrides its preset value only
  		// when given (see below, where they go into the RunConfig).
  		double doorOpenTime = 0.0;
  		if (argc >= 16) {
  			if(from_string<double>(doorOpenTime, string(argv[15]), dec)) {
  				if (doorOpenTime < 0) {
  					cout << "Invalid Door open time: must be >= 0" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}
  			else {
  				cout << "Invalid Door open time" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		double doorCloseTime = 0.0;
  		if (argc >= 17) {
  			if(from_string<double>(doorCloseTime, string(argv[16]), dec)) {
  				if (doorCloseTime < 0) {
  					cout << "Invalid Door close time: must be >= 0" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}
  			else {
  				cout << "Invalid Door close time" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		double perPassengerTime = 0.0;
  		if (argc >= 18) {
  			if(from_string<double>(perPassengerTime, string(argv[17]), dec)) {
  				if (perPassengerTime < 0) {
  					cout << "Invalid Per-passenger transfer time: must be >= 0" << endl;
  					usage(string(argv[0]));
  					return 1;
  					}
  				}
  			else {
  				cout << "Invalid Per-passenger transfer time" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		// Dissatisfaction thresholds. Both unset by default, and unset when given
  		// as <= 0 - the same convention <Elevator max weight> uses for "no limit
  		// configured". Unset is not a threshold of zero and not a verdict of zero:
  		// it means nobody measured, and the snapshot says so by writing the
  		// threshold as null and omitting the verdict keys.
  		ServiceThresholds serviceThresholds;

  		if (argc >= 19) {
  			double waitingThreshold = 0.0;
  			if(from_string<double>(waitingThreshold, string(argv[18]), dec)) {
  				if (waitingThreshold > 0) serviceThresholds.waiting = waitingThreshold;
  				}
  			else {
  				cout << "Invalid Waiting dissatisfaction threshold" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		if (argc >= 20) {
  			double journeyThreshold = 0.0;
  			if(from_string<double>(journeyThreshold, string(argv[19]), dec)) {
  				if (journeyThreshold > 0) serviceThresholds.journey = journeyThreshold;
  				}
  			else {
  				cout << "Invalid Journey dissatisfaction threshold" << endl;
  				usage(string(argv[0]));
  				return 1;
  				}
  			}

  		// <Elevator speed> is accepted and ignored. It keeps its positional slot
  		// so the sixteen arguments after it do not shift, and it still means
  		// "time units per floor" for the linear travel model - which the
  		// positional CLI never selects (only a config file can). Say so rather
  		// than letting someone wonder why the argument has no effect.
  		if (argc >= 6) {
  			cerr << "warning: <Elevator speed> (" << speed << ") is ignored. Travel time now comes "
  			     << "from the jerk-limited motion profile and <Floor height> (" << floorHeight
  			     << " m), so one time unit is one second. The "
  			     << "argument still applies to the old linear model, selectable through a config "
  			     << "file (travelModel: \"linear\"), not this command line." << endl;
  		}

  		// Gather the positional arguments into the same RunConfig a --config file
  		// produces, and run through the one shared path. The positional CLI is
  		// always the kinematic default; the linear model is config-file only.
  		RunConfig cfg;
  		cfg.floors = no_floors;
  		cfg.elevators = no_elevators;
  		cfg.customers = cus_to_simulate;
  		cfg.arrivalRate = arrival_rate;
  		cfg.sane = sane;
  		cfg.seed = seed;
  		cfg.maxWeightKg = maxWeight;
  		cfg.meanWeightKg = meanCustomerWeight;
  		cfg.stddevWeightKg = stddevCustomerWeight;
  		cfg.bypassFraction = bypassFraction;
  		cfg.floorHeightM = floorHeight;
  		cfg.carMassKg = carMass;
  		cfg.counterweightMassKg = counterweightMass;
  		// Given -> overrides the preset's value; omitted -> the preset.
  		if (argc >= 16) cfg.doorOpenTime = doorOpenTime;
  		if (argc >= 17) cfg.doorCloseTime = doorCloseTime;
  		if (argc >= 18) cfg.perPassengerTransferTime = perPassengerTime;
  		cfg.waitingThreshold = serviceThresholds.waiting;
  		cfg.journeyThreshold = serviceThresholds.journey;
  		cfg.elevatorSpeed = speed;
  		cfg.travelModel = TravelModel::Kinematic;

  		return runSimulation(cfg);
		}
	};
