/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : Event.cpp

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


#include<iostream>
#include<string>
#include<cstdlib>

#include "Event.h"

namespace openEGCS {

using namespace std;

Event::Event(unsigned long int ev_id, string ev_name, unsigned int ev_type, unsigned int ev_direction, int ev_floor, double time, Customer* ev_cus, int ev_el)
{
	id = ev_id;
	if (ev_name == "") name = "Un-named Event";
	else name = ev_name;
	
	floor = ev_floor;
	direction = ev_direction;
	type = ev_type;
	timestamp = time;
	cus = ev_cus;
	elevator = ev_el;
	stopId = 0;
	
}

Event::Event(const Event& event)
{
	id = event.id;
	name = event.name;
	floor = event.floor;
	direction = event.direction;
	type = event.type;
	timestamp = event.timestamp;
	cus = event.cus;
	elevator = event.elevator;
	stopId = event.stopId;
}

int Event::getFloor() const
{
	return floor;
}

unsigned int Event::getDirection() const
{
	return direction;
}

unsigned long int Event::getId() const {
	return id;
	}


unsigned long int Event::setId(unsigned long int newId) {
	id = newId;
	return id;
	}
			
string Event::getName() const {
	return name;
	}

string Event::setName(string newName) {
	if (newName == "") newName = "Un-named Event";
	name = newName;
	return name;
	}
	
unsigned int Event::getType() const
{
	return type;
}

double Event::getTime() const
{
	return timestamp;
}

Customer* Event::getCustomer() const
{
	return cus;
}

int Event::getElevator() const
{
	return elevator;
}

unsigned long int Event::getStopId() const
{
	return stopId;
}

void Event::setStopId(unsigned long int id)
{
	stopId = id;
}

bool Event::operator ==(Event& a)
{
	return ((floor == a.floor) && (direction == a.direction) && (type == a.type) && (timestamp == a.timestamp) && (cus == a.cus) && (elevator == a.elevator));
}

bool Event::operator !=(Event& a)
{
	return !(*this == a);
}

Event Event::operator=(const Event& a)
{
	id = a.id;
	name = a.name;
	floor = a.floor;
	direction = a.direction;
	type = a.type;
	timestamp = a.timestamp;
	cus = a.cus;
	elevator = a.elevator;
	stopId = a.stopId;
	return *this;
}

bool Event::operator !=(vector<Event>::iterator it)
{
	return !((floor == it->floor) && (direction == it->direction) && (type == it->type) && (timestamp == it->timestamp) && (cus == it->cus) && (elevator == it->elevator));
}

bool Event::operator ==(vector<Event>::iterator it)
{
	// The negation goes through operator!=, the overload that compares the
	// fields. Written as `!(*this == it)` it would resolve to this very
	// function and recurse without end.
	return !(*this != it);
}

Event Event::operator =(vector<Event>::iterator it)
{
	floor = it->floor;
	direction = it->direction;
	type = it->type;
	timestamp = it->timestamp;
	cus = it->cus;
	elevator = it->elevator;
	stopId = it->stopId;
	return *this;
}

bool Event::operator >(Event& a)
{
	return (timestamp < a.timestamp);
}

bool Event::operator <(Event& a)
{
	return (timestamp > a.timestamp);
}

bool Event::operator >=(Event& a)
{
	return (timestamp <= a.timestamp);
}

bool Event::operator <=(Event& a)
{
	return (timestamp >= a.timestamp);
}

void Event::increaseTime(double val)
{
	timestamp += val;
}

void Event::setTime(double time)
{
	timestamp = time;
}

void Event::show()
{
	if (type == CUST_CALL) cout<<"\t\tEvent of customer call ";
	else if (type == CUST_GO) cout<<"\t\tEvent of customer go ";
	else if (type == ENTER) cout<<"\t\tEvent of enter ";
	else if (type == LEAVE) cout<<"\t\tEvent of leave ";
	else
	{
		cerr<<"ERROR: Trace -> Event.cpp ->process() -> Unknown Event type."<<endl; 
		exit(1);
	}
	cout<<"at timestamp "<<timestamp<<endl;
}

} // namespace openEGCS
