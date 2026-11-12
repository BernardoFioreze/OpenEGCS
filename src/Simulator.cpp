/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : Simulator.cpp

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
Simulator.cpp

Changes from the original liftsim:
  - Inter-arrival times come from the injected ITrafficProfile; the default is
    a stationary Poisson process (Exponential(arrivalRate) inter-arrivals).
  - Randomness is injected via IRandomProvider; passing a seeded
    RandomProvider makes an entire run 100% reproducible (needed to compare
    dispatch algorithms on identical scenarios).
  - Dispatch strategy / metrics collector / simulation logger are all
    injected (inversion of control) instead of being hardcoded.
  - printMetrics() is called exactly once, after the loop ends.
  - A JSON snapshot of the whole run (event timeline + metrics) is written
    at the end via ISimulationLogger.
*/

#include<iostream>
#include<string>
#include<ctime>
#include<cmath>
#include<stdexcept>

#include "Simulator.h"
#include "RandomProvider.h"
#include "JsonSimulationLogger.h"
#include "StationaryUniformTrafficProfile.h"

namespace openEGCS {

#define DEBUG_SIM false

using namespace std;

Simulator::Simulator(unsigned long int sim_id, string sim_name, int no_flrs, int no_els,
	int cus_to_sim, double arrival_rate, double el_speed, bool sanity,
	shared_ptr<IRandomProvider> random_provider,
	shared_ptr<IDispatchStrategy> dispatch_strategy,
	shared_ptr<IMetricsCollector> metrics_collector,
	shared_ptr<ISimulationLogger> simulation_logger,
	std::optional<uint64_t> seed_for_metadata,
	double max_weight,
	shared_ptr<ICapacityStrategy> capacity_strategy,
	double mean_customer_weight,
	double stddev_customer_weight,
	shared_ptr<IDwellTimeStrategy> dwell_strategy,
	shared_ptr<ITravelTimeStrategy> travel_strategy,
	shared_ptr<IPassengerBehaviorStrategy> passenger_behavior,
	shared_ptr<IGroupDispatchStrategy> group_dispatch_strategy,
	shared_ptr<ITrafficProfile> traffic_profile,
	shared_ptr<IElevatorSchedulingStrategy> scheduling_strategy)
	: system(no_els, no_flrs, el_speed, dispatch_strategy, metrics_collector, max_weight, capacity_strategy,
	      dwell_strategy, travel_strategy, passenger_behavior, random_provider, group_dispatch_strategy,
	      scheduling_strategy),
	  // The SAME IRandomProvider the system resolved (the injected one, or the
	  // fallback it built): arrivals and the group dispatcher draw from one
	  // seeded stream, so there is a single RNG order to keep as a determinism
	  // contract. `system` is declared before `rng`, so it is already
	  // constructed here.
	  rng(system.getRandomProvider()),
	  logger(simulation_logger ? simulation_logger : make_shared<JsonSimulationLogger>()),
	  // Null means the stationary uniform default, built with this run's
	  // arrival_rate. The same instance is handed to the factory below, so
	  // inter-arrival and origin/destination draws come from one demand object.
	  trafficProfile(traffic_profile ? traffic_profile
	                                  : make_shared<StationaryUniformTrafficProfile>(arrival_rate)),
	  customerFactory(rng, no_flrs, sanity, mean_customer_weight, stddev_customer_weight, trafficProfile)
{
	id = sim_id;
	name = sim_name;
	
	if (sim_name == "") name = "Un-named Simulator";
	
	cout<<"\n --- Simulation Parameters --- "<<endl;
	
	floors = no_flrs;
	cout<<"\n Number of Floors = "<<floors;
	elevators = no_els;
	cout<<"\n Number of Elevators = "<<elevators;
	
	arrivalRate = arrival_rate;
	cout<<"\n Poisson arrival rate (customers/unit time) = "<<arrivalRate;
	
	speed = el_speed; 	// In seconds/Floor
	cout<<"\n Speed of the Elevator = "<<speed;
	sane = sanity;
	cout<<"\n Whether the users are sane : "<<sane;
	
	cusToSimulate = cus_to_sim;
	cout<<"\n Number of users to simulate = "<<cusToSimulate<<endl;
	departed = 0;
	
	cout<<endl;
	cout<<"--- Simulation Begins ---\n"<<endl;
	cout<<"Simulation Running..."<<endl;

	SimulationMetadata metadata;
	metadata.floors = floors;
	metadata.elevatorCount = elevators;
	metadata.speed = speed;
	metadata.arrivalRate = arrivalRate;
	metadata.sane = sane;
	metadata.customersToSimulate = cusToSimulate;
	metadata.seed = seed_for_metadata;
	// Asked of the model itself rather than reconstructed here, so the snapshot
	// cannot disagree with the engine about which clock it ran on.
	const shared_ptr<ITravelTimeStrategy>& travel = system.getTravelTimeStrategy();
	metadata.travelModel = travel->name();
	metadata.oneFloorLegSeconds = travel->legDuration(1.0);
	// Which demand pattern produced the arrivals.
	metadata.trafficProfile = trafficProfile->name();
	// The rest of the physics, each asked of the component that holds it,
	// after the system has resolved its defaults.
	metadata.travel = travel->describe();
	metadata.dwell = system.getDwellTimeStrategy()->describe();
	metadata.energy = system.getMetricsCollector()->energyModel();
	if (std::isfinite(max_weight)) {
		metadata.installation.ratedLoadKg = max_weight;
		metadata.installation.bypassThresholdKg = system.getElevators().front().getBypassThresholdKg();
	}
	metadata.installation.meanPassengerKg = mean_customer_weight;
	metadata.installation.stddevPassengerKg = stddev_customer_weight;
	logger->setMetadata(metadata);

	};

unsigned long int Simulator::getId() const {
	return id;
	}


unsigned long int Simulator::setId(unsigned long int newId) {
	id = newId;
	return id;
	}
			
string Simulator::getName() const {
	return name;
	}

string Simulator::setName(string newName) {
	if (newName == "") newName = "Un-named Simulator";
	name = newName;
	return name;
	}
	
double Simulator::getCurrentTime() const {	// Gets the Current Clock time.
	return clock.getCurrentTime();
	}

Customer* Simulator::generateNextArrival(double& enq_time, int& cus_id) {
	// The demand profile owns the inter-arrival draw. The stationary default
	// returns one Exponential(arrivalRate); a peaked profile may consume a
	// variable number of draws (thinning). The origin/destination draw that
	// follows is the factory's, through the same profile.
	enq_time += trafficProfile->nextInterArrivalTime(enq_time, *rng);
	Customer* newCus = new Customer(customerFactory.createArrival(cus_id, enq_time));
	logger->logCustomer(*newCus);
	return newCus;
	}

bool Simulator::startSimulation() {
	double enq_time = 0;
	int cus_id = 1;
	int ev_id = 1;

	// Exactly `cusToSimulate` customers are ever created: the look-ahead
	// window is capped so the arrival process stops at that count instead of
	// running for as long as the loop below does.
	for (int i = 1; i <= ARRIVAL_LOOKAHEAD && cus_id <= cusToSimulate; i++) {
		Customer* newCus = generateNextArrival(enq_time, cus_id);
		Event newEvent(ev_id, "", CUST_CALL, newCus->getDir(), newCus->getAt(), enq_time, newCus);
		system.upcomingEvents.push(newEvent);

		if (DEBUG_SIM) {
			cout<<"Setting up Customer "<<cus_id<<" to arrive at floor "<<newCus->getAt()<<" at time = "<<enq_time<<endl;
			}

		cus_id++;
		ev_id++;
		}
	
	// The run ends only once every created customer has reached their
	// destination - `departed` counts LEAVE records and `cusToSimulate` is
	// also the number of customers created, so this is "nobody is left
	// behind".
	while (departed < cusToSimulate) {

		if (system.upcomingEvents.empty()) {
			// Unreachable unless a customer was dropped: every created
			// customer has a CALL task, every boarding has a GO task, so an
			// empty queue with people unserved means some event was lost.
			throw std::logic_error("Simulator: the event queue ran dry with customers still unserved - "
				"a customer was dropped somewhere between arrival and alighting");
			}

		Event to_process = system.upcomingEvents.pop();
		
		if (DEBUG_SIM) {
			cout<<"Processing Event-id "<<to_process.getId()<<" with timestamp "<<to_process.getTime()<<endl;
			}
		clock.updateTick(to_process.getTime());

		// Events get their row here, at the instant they are popped: the arrival
		// of a customer, their re-call, and a cabin reaching a stop. What happens
		// to the PEOPLE at that stop is reported by the engine instead and logged
		// below - boarding is not an event, because who boards is not knowable
		// until the cabin is standing there.
		string typeName;
		if (to_process.getType() == CUST_CALL) typeName = "CUST_CALL";
		// A passenger pressing the landing button again after a car left
		// without them. It is in the timeline for the same reason PASS is and
		// reallocation is not: something happens at the waiting passenger's own
		// floor - their button lights up again - and it has an instant of its own.
		// `elevatorId` on this row is the cabin that refused them, not one that is
		// coming.
		else if (to_process.getType() == RECALL) typeName = "RECALL";
		// A cabin reaching one of its stops. This is the row that makes a cabin's
		// stop list complete and direct - the transfers below are what happened to
		// the PEOPLE at that stop, and a stop can legitimately have none of them.
		else if (to_process.getType() == ARRIVE) typeName = "ARRIVE";
		else typeName = "";
		if (typeName != "") {
			logger->logEvent(to_process.getTime(), typeName, to_process.getElevator(),
				to_process.getCustomer() ? to_process.getCustomer()->getId() : 0,
				to_process.getFloor());
			}

		system.update(clock);

		system.giveEvent(to_process);

		// The passenger half of a stop: who got out, who got in, and who was
		// left standing there. Records, drained in the order they happened, so
		// one stop writes its alightings, then its boardings, then the DEPART
		// of the leg it leaves on - which is the order in which those things
		// physically happen.
		//
		// A PASS is a boarding that did not happen: a cabin arrived, opened its
		// doors and did not take this passenger. It keeps the place the ENTER
		// would have had, and it is the only record of a stop at which nobody got
		// in or out.
		for (const ElevatorSystem::Transfer& tr : system.drainTransfers()) {
			string transferName = "UNKNOWN";
			if (tr.type == ENTER) transferName = "ENTER";
			else if (tr.type == LEAVE) transferName = "LEAVE";
			else if (tr.type == PASS) transferName = "PASS";
			logger->logEvent(tr.time, transferName, tr.elevatorId, tr.customerId, tr.floor);
			if (tr.type == LEAVE) {
				departed++;
				}
			}
		for (const ElevatorSystem::Departure& dep : system.drainDepartures()) {
			// The real instant an elevator starts moving towards its next stop,
			// so a trajectory reader can draw idle-then-move segments instead of
			// interpolating linearly across the whole gap.
			logger->logEvent(dep.time, "DEPART", dep.elevatorId, 0, dep.fromFloor);
			}

		// Keep a constant-size look-ahead window of pending arrivals: every
		// time an arrival is consumed, generate one more - until the
		// `cusToSimulate` customers of this run have all been created. After
		// that the building only drains: no new arrivals, and the loop runs
		// until the last of them alights.
		if (to_process.getType() == CUST_CALL && cus_id <= cusToSimulate) {
			Customer* newCus = generateNextArrival(enq_time, cus_id);
			Event newEvent(ev_id, "", CUST_CALL, newCus->getDir(), newCus->getAt(), enq_time, newCus);
			system.upcomingEvents.push(newEvent);
			cus_id++;
			ev_id++;
			}
		}

	system.printMetrics();
	writeSnapshot();
	return true;
	}

void Simulator::writeSnapshot(const string& filePath) {
	logger->finalize(*system.getMetricsCollector(), filePath);
	}

} // namespace openEGCS
