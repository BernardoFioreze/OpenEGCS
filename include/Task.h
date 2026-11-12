/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : Task.h

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

#ifndef __TASK__
#define __TASK__

#include<iostream>
#include<string>
#include<vector>
#include "Event.h"

#define DEBUG false
#ifndef __TASKTYPES__
#define __TASKTYPES__

#define DEFAULT 0xDEFA	// This is the default task that the constructor initializes to.
#define CALL 0xCA11		// This task type is generated when a customer on a floor calls an elevator.
#define GO 0x0060		// This task type is generated when a customer presses a button on the elevator.
#define UP 0x0900
#define DOWN 0xD047

#endif

using namespace std;

namespace openEGCS {

class Task {
	
	private:
		int destination;		// The floor number pressed on the elevator or the floor on which the call button is pressed.
		unsigned int direction;	// Stores the direction if a call button is pressed, otherwise DEFAULT
		unsigned int type;		// Stores the type of the Task - CALL or GO
		double timestamp;		// Time at which the task occurs
		unsigned long int stopId;	// Identity of the stop (route entry) this task belongs to; 0 = not yet assigned. See getStopId().
		int boardingPassengers;		// Passengers getting IN at this stop; see getBoardingPassengers().
		int alightingPassengers;	// Passengers getting OUT at this stop; see getAlightingPassengers().
		bool hasCarCall;			// This stop includes a GO (car call); see servesSweep().
		int upCalls;				// How many hall calls requesting UP this stop serves; see servesSweep() and releaseHallCall().
		int downCalls;				// How many hall calls requesting DOWN this stop serves; see servesSweep() and releaseHallCall().
		//vector<Event*> associatedEvents;
	
	public:
		Task(unsigned long int task_id = 0, string task_name = "", unsigned int task_type = DEFAULT, unsigned int task_direction = DEFAULT, int task_destination = DEFAULT, double time = 0);				// The constructor for the Task
		Task(const Task& task);								// Copy constructor for the Task
		
		int getDest() const;								// Returns the destination floor number of the Task
		unsigned int getDirection() const; 					// Returns the direction variable
		unsigned int getType() const;						// Returns the type of the Task
		double getTime() const;								// Returns the timestamp of the task

		// Identity of the *stop* this task is (or was merged into) in one
		// elevator's route. Assigned by Elevator::addTask (never by the caller):
		// unique within that elevator, never reused, 0 until the task has been
		// offered to an elevator. When addTask merges a new task into an existing
		// entry for the same floor, the entry keeps its stopId, so N customers
		// sharing one stop all see the same value - and it is also what identifies
		// the one ARRIVE event the stop has. It is NOT part of operator==/!= (those
		// compare the request itself - destination/direction/type/timestamp - which
		// the dry-run/undo logic in dispatch relies on) and is copied by every copy
		// and assignment.
		unsigned long int getStopId() const;
		void setStopId(unsigned long int id);					// Reserved for Elevator::addTask (and tests); see getStopId().

		// How many passengers get in and out at the stop this task represents. A
		// route entry is not one customer: a CALL contributes one boarding and a GO
		// one alighting, and when Elevator::addTask merges a new task into an
		// existing entry for the same floor, the entry's counters accumulate - so
		// the pair is "how many people this one stop serves", which is what
		// IDwellTimeStrategy needs to price the stop.
		//
		// Only the real (flag == 1) addTask path counts. A dry-run insertion
		// that merges into a live entry would otherwise inflate it permanently,
		// because the undo cannot subtract from an entry it did not create (the
		// merged task is not operator==-equal to the entry, so removeTask finds
		// nothing). Both are 0 on a task that has never been offered to an
		// elevator, and neither takes part in operator==/!=.
		int getBoardingPassengers() const;
		int getAlightingPassengers() const;
		// Adds this task's own contribution to the stop: one boarding for CALL or
		// one alighting for GO, and the *kind* of request it is, so the stop keeps
		// serving everything that was merged into it. Reserved for
		// Elevator::addTask (real inserts only, see getBoardingPassengers()).
		void addPassengerOf(const Task& task);

		// Can this stop be served while the cabin travels in `travelDirection`
		// (UP or DOWN)? True when the stop includes a car call - a passenger
		// inside wants this floor, which any sweep passing it satisfies - or a
		// hall call requesting that same direction.
		//
		// Why a stop needs this instead of a single getDirection(): one route
		// entry can serve several requests, and at a sweep reversal those requests
		// disagree about direction. A cabin arriving at the top floor opens its
		// doors once and takes both the passengers who called going up and the ones
		// waiting to go down. Collapsing that into one direction field would throw
		// away the fact that someone wants to go down there, and
		// IElevatorSchedulingStrategy reads exactly that to order the route - so
		// the information is kept per stop, as a set, and getDirection() keeps
		// meaning "the direction *this* request asked for".
		bool servesSweep(unsigned int travelDirection) const;
		bool hasCarCallRequest() const;		// True if a GO was merged into this stop
		bool hasHallCall(unsigned int callDirection) const;	// True if a CALL for that direction was merged in

		// Takes one hall call of `callDirection` back out of this stop: the
		// passenger it was counted for is not boarding here, because the cabin
		// filled up before it arrived (the "load bypass" of Barney & Al-Sharif
		// section 10.8.3) or another cabin took them. Decrements both the boarding
		// count the dwell is priced on and the per-direction count servesSweep()
		// reads; a no-op if this stop was not serving a call that way. Reserved for
		// Elevator::releaseHallCall, which is the only thing that knows whether the
		// route entry may now be dropped altogether.
		//
		// There is deliberately no counterpart for a car call: a passenger
		// already inside cannot have their floor taken away from them, which is
		// Rule 2 of the five rules of lift traffic control (Closs, 1970).
		void releaseHallCall(unsigned int callDirection);
		// Adds one hall call of `callDirection` to this stop: one more person
		// waiting at this floor who is going that way, so one more boarding to
		// price the dwell on and one more request keeping servesSweep() true
		// for that direction.
		//
		// The exact mirror of releaseHallCall, and it exists because a landing
		// call is one per (floor, direction) while the people behind it are
		// many: the second person to press a button that is already lit does not
		// create a route entry, they join the one that is already there. Reserved
		// for Elevator::joinHallCall, which is what knows the entry exists;
		// `callDirection` must be UP or DOWN.
		void addHallCall(unsigned int callDirection);
		// Replaces the stop's passenger counters with what really happens at it,
		// once the cabin is standing there and that is known. Until then the
		// counters are a forecast: everybody registered on the stop, which is
		// neither the people who will fit nor the rest of the floor's queue the
		// cabin will also take. Reserved for Elevator::settleServedStop. Touches
		// nothing but the two counters: why the cabin stopped here does not change.
		void settlePassengers(int boarding, int alighting);
		// Does this stop still have any reason to exist - a car call, or at
		// least one hall call in either direction? False means the cabin has
		// nothing to do at this floor any more, which only releaseHallCall can
		// bring about.
		bool servesAnyone() const;
		// Sets this task's counters to its own single passenger, for a task that
		// is becoming a brand-new route entry. Reserved for Elevator::addTask.
		void initPassengerCounters();
		
		bool operator ==(Task& a);							// Function which checks whether two Tasks are equal or not
		bool operator !=(Task& a);							// As usual
		Task operator =(const Task& a);						// As usual
		bool operator !=(vector<Task>::iterator it);		// As usual
		bool operator ==(vector<Task>::iterator it);		// As usual
		Task operator =(vector<Task>::iterator it);			// As usual
		/*
		------------------ OLD CODE (DIFEERENT IMPLEMENTATION) ---------------------
		void addEvent(Event* ev);
		void increaseTimeStamp(double val);
		void showEvents();
		void clearEvents();
		*/
};

} // namespace openEGCS

#endif
