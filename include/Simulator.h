/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.
Refactored to support: Poisson arrivals (exponential inter-arrival time),
deterministic seeded runs, pluggable dispatch strategy / metrics collector /
simulation logger (inversion of control), and a final JSON snapshot.

File Name : Simulator.h

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

#ifndef __SIMULATOR__
#define __SIMULATOR__

#include<iostream>
#include<string>
#include<memory>
#include<optional>
#include<cstdint>
#include<limits>

#include "Timer.h"
#include "Event.h"
#include "ElevatorSystem.h"
#include "IRandomProvider.h"
#include "IDispatchStrategy.h"
#include "IMetricsCollector.h"
#include "ISimulationLogger.h"
#include "CustomerFactory.h"
#include "ICapacityStrategy.h"
#include "ITrafficProfile.h"


using namespace std;

namespace openEGCS {

class Simulator {	
	
	private:
		unsigned long int id;
		string name;
		
		int floors;
		int elevators;
		
		double arrivalRate;			// Lambda of the Poisson arrival process (expected arrivals per unit time).
		
		int cusToSimulate;
		int departed;
		
		float speed;
		bool sane;
		
		ElevatorSystem system;
		Timer clock;

		shared_ptr<IRandomProvider> rng;
		shared_ptr<ISimulationLogger> logger;
		// The demand pattern: when customers arrive and between which floors.
		// Shared with `customerFactory`, which uses it for the origin/destination
		// draw while this class uses it for the inter-arrival draw, so there is
		// one demand object per run. Resolved to a StationaryUniformTrafficProfile
		// when none is injected. Declared before `customerFactory` so the
		// initializer list can hand it over.
		shared_ptr<ITrafficProfile> trafficProfile;
		CustomerFactory customerFactory;

		// Number of arrival events kept queued ahead of the current simulation
		// time. It is a window, not a budget: the run creates exactly
		// `cusToSimulate` customers in total and then stops generating arrivals,
		// so the window shrinks to nothing as the last customers are consumed.
		static const int ARRIVAL_LOOKAHEAD = 5;

		Customer* generateNextArrival(double& enq_time, int& cus_id);
			
	public:
		// `dispatch_strategy` MUST be provided by the caller (it is forwarded
		// as-is to ElevatorSystem, which requires it - there is no default
		// dispatch algorithm bundled with the core library).
		// `max_weight`/`capacity_strategy` are forwarded as-is to
		// ElevatorSystem (same capacity for every elevator); default
		// `max_weight` is unlimited, matching pre-feature behaviour.
		// `mean_customer_weight`/`stddev_customer_weight` configure the
		// Normal distribution CustomerFactory draws each customer's weight
		// from.
		// `passenger_behavior` is forwarded as-is too: it is what the
		// *passenger* decides at the open door, and defaults to boarding
		// every car they fit into. It has no CLI argument; a policy such as
		// examples::LoadingThresholdPassengerBehavior is injected.
		Simulator(unsigned long int sim_id, string sim_name, int no_flrs, int no_els,
			int cus_to_sim = 5, double arrival_rate = 0.5, double el_speed = 1, bool sanity = true,
			shared_ptr<IRandomProvider> random_provider = nullptr,
			shared_ptr<IDispatchStrategy> dispatch_strategy = nullptr,
			shared_ptr<IMetricsCollector> metrics_collector = nullptr,
			shared_ptr<ISimulationLogger> simulation_logger = nullptr,
			std::optional<uint64_t> seed_for_metadata = std::nullopt,
			double max_weight = std::numeric_limits<double>::infinity(),
			shared_ptr<ICapacityStrategy> capacity_strategy = nullptr,
			double mean_customer_weight = 70.0,
			double stddev_customer_weight = 15.0,
			shared_ptr<IDwellTimeStrategy> dwell_strategy = nullptr,
			shared_ptr<ITravelTimeStrategy> travel_strategy = nullptr,
			shared_ptr<IPassengerBehaviorStrategy> passenger_behavior = nullptr,
			// The group dispatch seam, forwarded as-is to ElevatorSystem. Null
			// gives the ImmediateGroupAdapter around `dispatch_strategy`; inject a
			// real IGroupDispatchStrategy (e.g. examples::GahcaDispatchStrategy) to
			// run a group controller through a full simulation.
			shared_ptr<IGroupDispatchStrategy> group_dispatch_strategy = nullptr,
			// The demand pattern. Null gives a StationaryUniformTrafficProfile
			// built with `arrival_rate` (Poisson arrivals, uniform floors); inject
			// a peaked profile (examples/traffic_profiles) for up/down/lunch-peak
			// demand. `arrival_rate` is ignored when a profile is injected - the
			// profile owns its own rate.
			shared_ptr<ITrafficProfile> traffic_profile = nullptr,
			// How each cabin orders its own route, forwarded as-is to
			// ElevatorSystem, which shares the one instance among every cabin.
			// Null gives LookSchedulingStrategy.
			shared_ptr<IElevatorSchedulingStrategy> scheduling_strategy = nullptr);
			
		unsigned long int getId() const;
		unsigned long int setId(unsigned long int newId);
		
		string getName() const;
		string setName(string newname);
		
		double getCurrentTime() const; // Returns the current time
		bool startSimulation();

		// Writes the final snapshot (event timeline + metrics) to filePath.
		void writeSnapshot(const string& filePath = "snapshot.json");
		
	};

} // namespace openEGCS


#endif
