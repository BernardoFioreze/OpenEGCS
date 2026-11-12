/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : Task.cpp

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

#include "Task.h"

namespace openEGCS {

using namespace std;

Task::Task(unsigned long int task_id, string task_name, unsigned int task_type, unsigned int task_direction, int task_destination, double time)
{
	destination = task_destination;
	direction = task_direction;
	type = task_type;
	timestamp = time;
	stopId = 0;
	boardingPassengers = 0;
	alightingPassengers = 0;
	// What kind of request this is, which is intrinsic to the task - unlike the
	// passenger counters, which stay 0 until a real addTask adopts it.
	hasCarCall = (task_type == GO);
	upCalls = (task_type == CALL && task_direction == UP) ? 1 : 0;
	downCalls = (task_type == CALL && task_direction == DOWN) ? 1 : 0;
	//associatedEvents.clear();
	//if (task_name == "") name = "Un-named Task";
}

Task::Task(const Task& task)
{
	destination = task.destination;
	direction = task.direction;
	type = task.type;
	timestamp = task.timestamp;
	stopId = task.stopId;
	boardingPassengers = task.boardingPassengers;
	alightingPassengers = task.alightingPassengers;
	hasCarCall = task.hasCarCall;
	upCalls = task.upCalls;
	downCalls = task.downCalls;
	vector<Event*>::const_iterator it;
	/*for (it = (task.associatedEvents).begin() ; it != (task.associatedEvents).end() ; ++it)
	{
		associatedEvents.push_back(*it);
	}*/
}

int Task::getDest() const
{
	return destination;
}

unsigned int Task::getDirection() const
{
	return direction;
}
/*
string Task::setName(string newName)
{
	if (newName == "") newName = "Un-named Task";
	name = newName;
	return name;
}
*/
unsigned int Task::getType() const
{
	return type;
}

double Task::getTime() const
{
	return timestamp;
}

unsigned long int Task::getStopId() const
{
	return stopId;
}

void Task::setStopId(unsigned long int id)
{
	stopId = id;
}

int Task::getBoardingPassengers() const
{
	return boardingPassengers;
}

int Task::getAlightingPassengers() const
{
	return alightingPassengers;
}

void Task::initPassengerCounters()
{
	// A fresh route entry serves exactly the one customer that created it.
	boardingPassengers = (type == CALL) ? 1 : 0;
	alightingPassengers = (type == GO) ? 1 : 0;
}

void Task::addPassengerOf(const Task& task)
{
	// `task` was merged into this entry, so this stop now serves its passenger
	// too - see getBoardingPassengers(). The *kind* of request is unioned in as
	// well, so a stop that absorbed a down-call still reports that it serves
	// downward demand even though getDirection() names only the first request.
	if (task.type == CALL) boardingPassengers++;
	else if (task.type == GO) alightingPassengers++;
	hasCarCall = hasCarCall || task.hasCarCall;
	// Counted, not OR-ed: two up-calls at one floor are two requests, and
	// releasing one of them must not erase the other one's direction.
	upCalls += task.upCalls;
	downCalls += task.downCalls;
}

void Task::releaseHallCall(unsigned int callDirection)
{
	// The mirror image of addPassengerOf, for ONE hall call: the passenger it
	// was counted for is not boarding here after all (the cabin filled up, or
	// another cabin took them). Only the hall-call side exists - a car call is
	// a passenger already inside, and nothing takes their floor away from them.
	if (callDirection == UP && upCalls > 0) upCalls--;
	else if (callDirection == DOWN && downCalls > 0) downCalls--;
	else return;	// not a direction this stop served: nothing to give back
	if (boardingPassengers > 0) boardingPassengers--;
}

void Task::addHallCall(unsigned int callDirection)
{
	// Counted, not OR-ed, for the same reason releaseHallCall can take one
	// back: two up-calls at one floor are two people, and releasing one of
	// them must not erase the other one's direction. Deliberately not routed
	// through addPassengerOf: that one needs a whole Task to copy from, and
	// manufacturing a fake one to add a passenger would make the call site read
	// as an insertion when nothing is inserted.
	if (callDirection == UP) upCalls++;
	else if (callDirection == DOWN) downCalls++;
	else return;		// not a direction a hall call can have: nothing to add
	boardingPassengers++;
}

void Task::settlePassengers(int boarding, int alighting)
{
	boardingPassengers = boarding;
	alightingPassengers = alighting;
}

bool Task::servesAnyone() const
{
	return hasCarCall || upCalls > 0 || downCalls > 0;
}

bool Task::hasCarCallRequest() const
{
	return hasCarCall;
}

bool Task::hasHallCall(unsigned int callDirection) const
{
	if (callDirection == UP) return upCalls > 0;
	if (callDirection == DOWN) return downCalls > 0;
	return false;
}

bool Task::servesSweep(unsigned int travelDirection) const
{
	// A car call is satisfied by any sweep that passes the floor; a hall call
	// only by a sweep going the way its passenger asked for.
	return hasCarCall || hasHallCall(travelDirection);
}

bool Task::operator ==(Task& a)
{
	return ((destination == a.destination) && (direction == a.direction) && (type == a.type) && (timestamp == a.timestamp));
}

bool Task::operator !=(Task& a)
{
	return !(*this == a);
}

Task Task::operator=(const Task& a)
{
	boardingPassengers = a.boardingPassengers;
	alightingPassengers = a.alightingPassengers;
	hasCarCall = a.hasCarCall;
	upCalls = a.upCalls;
	downCalls = a.downCalls;
	destination = a.destination;
	direction = a.direction;
	type = a.type;
	timestamp = a.timestamp;
	stopId = a.stopId;
	vector<Event*>::const_iterator it;
	/*for (it = (a.associatedEvents).begin() ; it != (a.associatedEvents).end() ; ++it)
	{
		associatedEvents.push_back(*it);
	}*/
	return *this;
}

bool Task::operator !=(vector<Task>::iterator it)
{
	return !((destination == it->destination) && (direction == it->direction) && (type == it->type) && (timestamp == it->timestamp));
}

bool Task::operator ==(vector<Task>::iterator it)
{
	return ((destination == it->destination) && (direction == it->direction) && (type == it->type) && (timestamp == it->timestamp));
}

Task Task::operator =(vector<Task>::iterator it)
{
	boardingPassengers = it->boardingPassengers;
	alightingPassengers = it->alightingPassengers;
	hasCarCall = it->hasCarCall;
	upCalls = it->upCalls;
	downCalls = it->downCalls;
	destination = it->destination;
	direction = it->direction;
	type = it->type;
	timestamp = it->timestamp;
	stopId = it->stopId;
	vector<Event*>::iterator temp_it;

	/*
	------------------ OLD CODE (DIFEERENT IMPLEMENTATION) ---------------------
	
	for (temp_it = (it->associatedEvents).begin() ; temp_it < (it->associatedEvents).end() ; ++temp_it)
	{
		associatedEvents.push_back(*temp_it);
	}
	
	*/
	return *this;
}
/*
------------------ OLD CODE (DIFEERENT IMPLEMENTATION) ---------------------

void Task::addEvent(Event* ev)
{
	cout<<"ADDING EVENT WITH TIMESTAMP "<<ev->getTime()<<endl;
	associatedEvents.push_back(ev);
	vector<Event*>::iterator it;
	//cout<<associatedEvents.size()<<endl;
	for (it = associatedEvents.begin() ; it < associatedEvents.end() ; ++it)
	{
		(*it)->show();
	}
}

void Task::increaseTimeStamp(double val)
{	
	vector<Event*>::iterator it;
	for (it = associatedEvents.begin() ; it < associatedEvents.end() ; ++it)
	{
		(*it)->increaseTime(val);
		if (DEBUG) cout<<"Increasing timestamp by "<<val<<" for event "<<(*it)->getType()<<endl;
	}
}

void Task::showEvents()
{
	vector<Event*>::iterator it;
	//cout<<associatedEvents.size()<<endl;
	for (it = associatedEvents.begin() ; it < associatedEvents.end() ; ++it)
	{
		(*it)->show();
	}
}

void Task::clearEvents()
{
	associatedEvents.clear();
}


*/

} // namespace openEGCS
