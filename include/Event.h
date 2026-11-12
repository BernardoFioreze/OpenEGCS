/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : Event.h

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


#ifndef __EVENT__
#define __EVENT__

#include<iostream>
#include<string>
#include<vector>
#include "Customer.h"

#ifndef __EVENTTYPES__
#define __EVENTTYPES__

#define DEFAULT 0xDEFA			// This is the default task that the constructor initializes to.
#define CUST_CALL 0xCA11		// This event type is generated when a customer on a floor calls an elevator.
#define CUST_GO 0x0060			// Not an event: a car call is registered as its passenger boards. giveEvent throws on it.
#define ENTER 0xE12E			// Customer entering the elevator
#define LEAVE 0x1EA2			// Customer leaving the elevator
// A passenger who could not get into the car that stopped for them pressing the
// landing button again. It carries the customer and their floor like a
// CUST_CALL does, and its `elevator` field is the cabin that just refused them -
// which is the one cabin the new call must NOT be given back to. It is an event
// because it has an instant of its own, decided by
// IPassengerBehaviorStrategy::nextCallTime.
#define RECALL 0x2ECA			// Customer pressing the landing button again after being left behind
// A cabin reaching one stop of its route: the doors open, whoever is going to
// get out gets out, whoever is going to get in gets in. It is the ONLY event
// derived from the route, and the only one of a stop: one per (elevator,
// stopId). ENTER, LEAVE and PASS are *records* reported by the engine while
// serving it, not events, because none has an instant of its own (all are this
// one's) and none can be scheduled - *who* boards is not knowable until the
// cabin is standing there with its doors open.
//
// Carries the elevator, the floor and the stopId; no customer, because it is not
// about a person. The mirror image of the DEPART record, so the pair brackets a
// stop in the snapshot's timeline.
//
// The hex of the older types spells their names (0xCA11, 0xE12E, 0x1EA2,
// 0x2ECA); "ARRIVE" has no spelling in hex digits, so this value is arbitrary
// and only has to be distinct from the rest.
#define ARRIVE 0xA881			// A cabin arrived at one of its route stops
// A boarding that did not happen: the cabin arrived, opened its doors and the
// passenger did not get in (no room, or they declined). A record, like ENTER
// and LEAVE (ElevatorSystem::Transfer), so it needs a value saying which of
// the three it is. None of the three is an event; giveEvent throws if one
// reaches it.
#define PASS 0x9A55				// A passenger a full cabin left on the landing

#endif


using namespace std;

namespace openEGCS {

class Event {
	
	private:
		unsigned long int id;
		string name;
		int floor;								// The floor number pressed on the elevator or the floor on which the call button is pressed.
		unsigned int direction;					// Stores the direction if a call button is pressed, otherwise DEFAULT
		unsigned int type;						// Stores the type of the Task - CALL or GO or ENTER of LEAVE
		double timestamp;						// Time at which the task occurs
		Customer* cus;							// The customer to which the event is associated
		int elevator;							// Elevator number associated with the event
		unsigned long int stopId;						// Route stop (Task::getStopId) of `elevator` this event belongs to; 0 = none/unknown
	
	public:
		Event(unsigned long int ev_id = 0, string ev_name = "", unsigned int ev_type = DEFAULT, unsigned int ev_direction = DEFAULT, int ev_destination = DEFAULT, double time = 0, Customer* ev_cus = NULL, int ev_el = 0);			// The constructor for the event
		Event(const Event& event);															// Copy constructor for the event
		
		int getFloor() const;								// Returns the destination floor number of the event
		unsigned int getDirection() const; 					// Returns the direction variable
		
		unsigned long int getId() const;					// Event ID
		unsigned long int setId(unsigned long int newId);	// Sets the event ID
		
		string getName() const;								// Returns event name
		string setName(string newName);						// Gives a name to the event
		
		unsigned int getType() const;						// Returns the type of the event
		double getTime() const;								// Returns the timestamp of the event
		Customer* getCustomer() const;						// Returns pointer to the customer to which event is associated
		int getElevator() const;							// Returns the elevator number associated to the event

		// Which stop of elevator `getElevator()`'s route this ARRIVE belongs to.
		// Together (getElevator(), getStopId()) identify a route entry for as long
		// as that entry exists - unlike a pointer/iterator into the elevator's task
		// vector, which any insertion invalidates.
		//
		// Contract: exactly one event carries a given stop, and it fires at that
		// stop's arrival time. 0 for event types without a stop (CUST_CALL, RECALL)
		// and for events created before the stop was assigned. Set by
		// ElevatorSystem, copied by copy-construction/assignment, ignored by
		// operator==.
		unsigned long int getStopId() const;
		void setStopId(unsigned long int id);
		
		bool operator ==(Event& a);							// Function which checks whether two events are equal or not
		bool operator !=(Event& a);							// As usual
		Event operator =(const Event& a);					// As usual

		// Same comparison as the Event& overloads above, against the event an
		// iterator points at: equal when floor, direction, type, timestamp,
		// customer pointer and elevator all match. Deliberately ignores `id`,
		// `name` and `stopId` (see getStopId()'s note) - so this is equality by
		// value, not identity, the same choice Task::operator== makes. Tested in
		// tests/test_event.cpp.
		bool operator !=(vector<Event>::iterator it);
		bool operator ==(vector<Event>::iterator it);
		Event operator =(vector<Event>::iterator it);		// As usual

		// WARNING: EventQueue does not use these - it has its own comparator on
		// (getTime(), seq) (src/EventQueue.cpp). These four are **inverted** with
		// respect to their own names - operator> returns timestamp < a.timestamp -
		// a trick for making a max-heap pop the earliest event. Nothing calls them,
		// and `a < b` on two Events answers the opposite of what it reads as.
		bool operator >(Event& a);
		bool operator <(Event& a);
		bool operator >=(Event& a);
		bool operator <=(Event& a);
		
		void increaseTime(double val);						// Increasing the timestamp
		// Overwrites the timestamp. Only meant for EventQueue::retime, which must
		// go through the queue so the heap is re-ordered; calling this on an Event
		// that is already inside an EventQueue by any other path would corrupt its
		// ordering.
		void setTime(double time);
		void show();										// Shows the event
};

} // namespace openEGCS

#endif
