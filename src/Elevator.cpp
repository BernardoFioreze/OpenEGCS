/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : Elevator.cpp

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
#include<cmath>
#include<stdexcept>
#include<algorithm>

#include "Elevator.h"

namespace openEGCS {

using namespace std;

Elevator::Elevator(unsigned long int el_id, string el_name, double pos, double time,
	shared_ptr<IElevatorSchedulingStrategy> scheduling_strategy,
	double max_weight, shared_ptr<ICapacityStrategy> capacity_strategy,
	shared_ptr<IDwellTimeStrategy> dwell_strategy,
	shared_ptr<ITravelTimeStrategy> travel_strategy)
{
	id = el_id;
	position = pos;
	status = IDLE;
	direction = IDLE;
	tasks.clear();
	staticTime = time;
	currentTime = 0;
	idleTime = 0;
	customers.clear();
	legStartPosition = pos;
	legStartTime = 0;
	avgWaitingTime = 0;
	totWaitingTime = 0;
	totReachingTime = 0;
	avgReachingTime = 0;
	customerServed = 0;
	customerIn = 0;
	schedulingStrategy = scheduling_strategy ? scheduling_strategy : make_shared<LookSchedulingStrategy>();
	maxWeight = max_weight;
	currentLoad = 0;
	capacityStrategy = capacity_strategy ? capacity_strategy : make_shared<PercentageCapacityStrategy>();
	// A zero FixedDwellTimeStrategy is the pre-item-#9 behaviour: the cabin
	// arrives and the next task starts in the same instant.
	dwellStrategy = dwell_strategy ? dwell_strategy : make_shared<FixedDwellTimeStrategy>();
	// A LinearTravelTimeStrategy over the same `staticTime` is the pre-item-#27
	// behaviour, bit for bit: a floor costs the same from rest as at full speed.
	travelStrategy = travel_strategy ? travel_strategy : make_shared<LinearTravelTimeStrategy>(staticTime);
	nextStopId = 1;
	legStarted = false;
	legDecisionPending = false;
	frontChanged = false;
	servingStopId = 0;
}

int Elevator::getId()
{
	return id;
}

double Elevator::getPosition() const
{
	return position;
}

unsigned int Elevator::getDirection() const
{
	return direction;
}

unsigned int Elevator::getStatus() const
{
	return status;
}

const Task& Elevator::getProcessingTask() const
{
	return processingTask;
}

Task& Elevator::addTask(Task& task, int flag)
{
	if (status == IDLE)
	{
		task.setStopId(nextStopId++);
		// Only the real call owns the passenger count (Task::getBoardingPassengers):
		// a dry-run must not leave its passenger behind in the route.
		if (flag == 1) task.initPassengerCounters();
		tasks.push_back(task);
		if (flag == 1)
		{
			status = BUSY;
			if (task.getDest() > position) direction = UP;
			else if (task.getDest() < position) direction = DOWN;
			// A task at the floor the cabin is already standing at: a leg of
			// zero length, which is how a parked cabin answers a landing call
			// from its own floor. A BUSY cabin always has a direction - the
			// scheduling strategy is promised UP or DOWN
			// (IElevatorSchedulingStrategy.h) - so it takes the call's own: a
			// car answering a down-call is going down. Left IDLE, LOOK would file
			// the stop the cabin is standing at under "needs a reversal", the car
			// call of whoever boarded there would go in front of it, and the cabin
			// would be stranded.
			else if (task.getType() == CALL) direction = task.getDirection();
			else
			{
				// A car call to the floor the cabin is at has no direction to
				// lend, and nothing in the engine creates one (CustomerFactory
				// re-draws a destination equal to the origin). Checked rather
				// than given an invented direction.
				tasks.pop_back();
				status = IDLE;
				throw std::logic_error("Elevator::addTask: a car call to the floor a parked cabin "
					"is standing at has no direction");
			}
			// A fresh departure starts a new leg right here (see
			// Elevator::update): without this, processingTask/legStartPosition
			// would stay whatever they were from the last completed leg until the
			// next processNextTask() call, and update() would integrate towards the
			// wrong target.
			processingTask = task;
			legStartPosition = position;
			// max(), not assignment: the cabin may have gone idle at a stop whose
			// doors are still closing, and this new leg cannot start before they
			// are.
			if (currentTime > legStartTime) legStartTime = currentTime;
		}
		return task;
	}

	if (task.getType() != GO && task.getType() != CALL)
	{
		throw std::logic_error("Elevator::addTask: unknown task type");
	}

	// The cabin is standing at a stop with its doors open, and whatever is
	// added now goes after it (see beginServing). A separate path rather than
	// a condition threaded through the one below, so that path stays the same
	// for every insertion made outside a stop.
	if (servingStopId != 0 && tasks.front().getStopId() == servingStopId)
	{
		return addTaskBehindServedStop(task, flag);
	}

	// Delegate to the pluggable scheduling policy (LOOK by default, see
	// LookSchedulingStrategy) to find where `task` fits in this elevator's
	// own route, keeping it ordered so the elevator finishes its current
	// sweep before reversing direction.
	// findInsertionPoint runs a binary search, which assumes the vector is ordered
	// by the key it compares with - and that order goes stale as the cabin moves
	// and reverses, so it is re-established first.
	reorderRoute();
	// Against stoppingPosition(), not position, for the same reason reorderRoute
	// sorts against it: an insertion for a floor the cabin can no longer stop at
	// must not become the next stop, it belongs in the return sweep. This is
	// what keeps the physical constraint out of the scheduling strategy, which
	// goes on ordering a route relative to "where the cabin is" and is simply
	// told a truthful answer to that.
	vector<Task>::iterator it = schedulingStrategy->findInsertionPoint(tasks, direction, stoppingPosition(), task);
	// A stop is a floor the cabin stands still at, so two route entries for the
	// same floor with nothing between them are one stop: the cabin does not move,
	// the doors open once, and the dwell is paid once. The insertion point is
	// where the new task would go, so it would end up adjacent to the entry *at*
	// that point or to the one *before* it, and either is enough to merge. The
	// predecessor check is what covers the sweep reversal, where an up-call and
	// the down-call waiting at the same floor land next to each other. That merge
	// is only lossless because the surviving entry keeps *both* requests
	// (Task::addPassengerOf unions them and Task::servesSweep reports them), so
	// the scheduling strategy still sees that there is downward demand at that
	// floor.
	vector<Task>::iterator mergeInto = tasks.end();
	if (it != tasks.end() && abs(it->getDest() - task.getDest()) < 0.001) mergeInto = it;
	else if (it != tasks.begin() && abs((it - 1)->getDest() - task.getDest()) < 0.001) mergeInto = it - 1;

	if (mergeInto != tasks.end())
	{
		// That stop now serves one more passenger, which is what the dwell time
		// is priced on. Dry-runs are excluded because their undo cannot take the
		// passenger back out - removeTask compares the caller's Task, which is
		// not operator==-equal to the entry it merged into.
		if (flag == 1) mergeInto->addPassengerOf(task);
		return *mergeInto;
	}
	task.setStopId(nextStopId++);
	if (flag == 1) task.initPassengerCounters();
	vector<Task>::iterator inserted = tasks.insert(it, task);
	if (inserted == tasks.begin())
	{
		// The new task overtakes whatever leg was in flight and becomes the
		// immediate next stop (a real LOOK elevator retargets before passing a
		// nearer request). It can only have landed here if the cabin can still
		// stop there, because findInsertionPoint above was asked against
		// stoppingPosition(). retargetLegTo keeps the leg's origin and clock
		// while changing its destination.
		retargetLegTo(*inserted);
		if (inserted->getDest() > position) direction = UP;
		else if (inserted->getDest() < position) direction = DOWN;
	}
	return task;
}

const Task& Elevator::processNextTask(unsigned long int finishedStopId,
	bool holdLegDecision)
{
	legStarted = false;
	const unsigned long int frontAtEntry = tasks.empty() ? 0 : tasks.front().getStopId();
	// The stop being served is the one finishing now: from here on the cabin
	// is deciding where to go next, and the next stop is chosen from the whole
	// route again. Released before the removal, not after, because the held
	// decision below still re-sorts the route (reorderRoute), and that sort
	// must not keep pinning a stop that is gone.
	if (finishedStopId != 0 && finishedStopId == servingStopId) servingStopId = 0;

	if (tasks.size() == 0)
	{
		status = IDLE;
		direction = IDLE;
		Task x(0 , "" , DEFAULT , DEFAULT , DEFAULT);
		processingTask = x;
		// Nothing left to decide, so nothing may stay pending: a held decision
		// left behind here would make the *next* leg report a departure that
		// belongs to a stop already dealt with.
		legDecisionPending = false;
		frontChanged = (frontAtEntry != 0);
		return processingTask;
	}

	vector<Task>::iterator it1 = tasks.begin();
	// Which stop, if any, just finished. With a stop id in hand it is the
	// front stop and only when it is that stop: a stop that is already gone
	// finishes nothing, which is what stops one wrong removal from cascading
	// into the rest of the route.
	bool finished = (finishedStopId != 0)
		? (it1->getStopId() == finishedStopId)
		: (abs(it1->getDest() - position) < 0.01);

	if (finished)
	{
		// The stop finishing right now still owes the rest of its dwell: its
		// ARRIVE fired at arrival + door-open, so what is left is the transfer
		// plus the door closing. The cabin may only start moving after that.
		// Folded into legStartTime at the *removal*, not at the leg decision,
		// because the decision may be held for a later transfer of the same stop
		// and every pending event re-timed in between has to be dated from a route
		// that no longer contains this stop - charging its full dwell instead of
		// its tail would date them one door-opening too late.
		double servedStopTail = stopDwellTail(*it1);
		tasks.erase(it1);
		double doorsShut = currentTime + servedStopTail;
		if (doorsShut > legStartTime) legStartTime = doorsShut;
		// The cabin is standing at this stop, so this is the origin of whatever
		// leg comes next - recorded now rather than only at the decision,
		// because Elevator::update runs in between when the decision is held.
		// Leaving it at the origin of the leg that just *ended* made update()
		// rewind the cabin to the floor it had departed from, firing 18 and 33
		// ENTER/LEAVE events at the wrong floor.
		legStartPosition = position;
		legDecisionPending = true;
	}

	// Hold the leg decision while this stop still has transfers to come: the
	// car call of each boarding passenger is only registered as they board.
	// The cabin keeps the processingTask it had, so startedNewLeg() stays
	// false and no departure is reported.
	if (holdLegDecision)
	{
		// Still re-establish the route order and re-anchor the leg in flight.
		// addTask's re-anchoring can leave processingTask pointing at a stop that
		// a later sort moved off the front; without re-synchronising here, a stop
		// could be served with the cabin correctly at its floor but the stop *not*
		// at the front, finish nothing, and be stranded in the route.
		//
		// reorderRoute is the right tool because it deliberately does not touch
		// `direction`: the whole point of holding the decision is that the
		// direction must not be chosen until the last boarder's car call is in.
		reorderRoute();
		frontChanged = (tasks.empty() ? 0 : tasks.front().getStopId()) != frontAtEntry;
		return processingTask;
	}

	if (tasks.size() == 0)
	{
		status = IDLE;
		direction = IDLE;

		Task y(0, "", DEFAULT , DEFAULT , DEFAULT);
		processingTask = y;
		// The queue emptied, but the doors of the stop just served may still be
		// closing. legStartTime already carries that tail (set at the removal
		// above), which is what stops the next task (addTask's IDLE branch)
		// from departing through a shut door.
		legDecisionPending = false;
	}
	else
	{
		status = BUSY;
		// The stop that just finished was the reference the order was built
		// against, so re-establish it before reading off the next stop.
		schedulingStrategy->reorder(tasks, direction, position);
		vector<Task>::iterator it;
		it = tasks.begin();
		processingTask = it;
		if (processingTask.getDest() > position) direction = UP;
		else direction = DOWN;
		// New leg starts here (see Elevator::update): position at this point
		// is the leg's true origin, whether we just arrived at a stop (erased
		// above) or this is a no-op "peek" re-confirming the same front task.
		legStartPosition = position;
		// Monotone floor: a leg cannot have started before now. The dwell tail
		// of a stop just served was already folded in at the removal, so this
		// is a no-op in that case - but the re-confirmation path reaches here
		// having removed nothing, and without it legStartTime stays in the past
		// while legStartPosition moves to now, so update() integrates from a
		// stale origin and the cabin overshoots.
		if (currentTime > legStartTime) legStartTime = currentTime;
		// A departure belongs to a leg that left a stop behind. "Did the stop id
		// in flight change?" is not a sound test once the decision can be held:
		// addTask re-anchors processingTask whenever an inserted task becomes the
		// route's front, so by the stop's last transfer the id may already have
		// changed for an unrelated reason.
		legStarted = legDecisionPending;
		legDecisionPending = false;

		// The sort above ran with the direction the cabin *had*; the lines
		// just above may have reversed it, and a reversal inverts the order
		// the reversal group must be served in. So order is re-established once
		// more, now for the direction the cabin actually has. reorderRoute leaves
		// `direction` alone on purpose: deriving it again from a new front would
		// feed the sort its own output and could oscillate, and one step of lag
		// here is corrected by the next completed stop.
		reorderRoute();
	}
	frontChanged = (tasks.empty() ? 0 : tasks.front().getStopId()) != frontAtEntry;
	return processingTask;
}

void Elevator::removeTask(Task& task)
{
	vector<Task>::iterator it;
	for (it = tasks.begin() ; it < tasks.end() ; ++it)
	{
		if (task != it) continue;
		else
		{
			tasks.erase(it);
			break;
		}
	}
}

double Elevator::stoppingPosition() const
{
	double elapsed = currentTime - legStartTime;
	// Not in flight - idle, or sitting out a dwell with legStartTime still in
	// the future - means standing at a floor, which the cabin can obviously
	// stop at.
	if (status == IDLE || elapsed <= 0.0) return position;

	double leg = processingTask.getDest() - legStartPosition;
	double reach = travelStrategy->minimumLegDistance(leg, elapsed);
	return (leg >= 0) ? legStartPosition + reach : legStartPosition - reach;
}

void Elevator::retargetLegTo(const Task& newFront)
{
	double elapsed = currentTime - legStartTime;
	if (status != IDLE && elapsed > 0.0)
	{
		// ChangeJourney: only the destination moves. legStartPosition and
		// legStartTime are deliberately NOT touched, which is the whole of item
		// #36 - Elevator::update then integrates the SAME journey over a new
		// total distance, instead of a new journey from rest.
		processingTask = newFront;
		return;
	}

	// Not in flight: the cabin is at a floor, so this is a departure and the leg
	// is anchored here. Monotone `max` on the time, because the doors of the
	// stop it is standing at may still be closing.
	processingTask = newFront;
	legStartPosition = position;
	if (currentTime > legStartTime) legStartTime = currentTime;
}

void Elevator::reorderRoute()
{
	if (tasks.size() < 2) return;
	const unsigned long int frontBefore = tasks.front().getStopId();
	// A stop being served keeps the front, and only the rest is ordered (see
	// beginServing). Taken out and put back rather than handing the strategy a
	// subrange: the interface takes the route, and a strategy is entitled to
	// assume what it is handed is all of it.
	if (servingStopId != 0 && frontBefore == servingStopId)
	{
		Task served = tasks.front();
		tasks.erase(tasks.begin());
		schedulingStrategy->reorder(tasks, direction, stoppingPosition());
		tasks.insert(tasks.begin(), served);
		return;
	}
	// Against stoppingPosition(), not position: a floor the cabin can no longer
	// stop at belongs in the return sweep, and sorting against where it can
	// stop is what puts it there - see stoppingPosition() in the header.
	schedulingStrategy->reorder(tasks, direction, stoppingPosition());
	if (tasks.front().getStopId() == frontBefore) return;

	// The sort moved a different stop to the front, so the leg in flight is no
	// longer heading for the next stop. Retarget through the shared path, which
	// preserves the leg's origin and clock; `direction` is deliberately left
	// alone, because it is the input the sort was just performed with and
	// flipping it here would invalidate the order that was only now
	// established - the next completed stop picks the new direction up.
	retargetLegTo(tasks.front());
}

StopContext Elevator::stopContextOf(const Task& stop)
{
	StopContext context;
	context.boarding = stop.getBoardingPassengers();
	context.alighting = stop.getAlightingPassengers();
	context.carCall = stop.hasCarCallRequest();
	context.hallCall = stop.hasHallCall(UP) || stop.hasHallCall(DOWN);
	return context;
}

double Elevator::stopDwellTail(const Task& stop) const
{
	// What the cabin still owes after the stop's ENTER/LEAVE events have fired:
	// those happen at arrival + door-open, so the tail is transfer + door-close.
	StopContext context = stopContextOf(stop);
	return dwellStrategy->getTransferTime(context) + dwellStrategy->getDoorCloseTime(context);
}

double Elevator::stopDwellTotal(const Task& stop) const
{
	StopContext context = stopContextOf(stop);
	return dwellStrategy->getDoorOpenTime(context)
		+ dwellStrategy->getTransferTime(context)
		+ dwellStrategy->getDoorCloseTime(context);
}

double Elevator::legInFlightRemainder(int destination) const
{
	// How much of the CURRENT leg is left, asked of the travel model rather
	// than recomputed as "a fresh leg from where the cabin is now".
	//
	// Those two are the same number only for a model whose duration is
	// additive in distance. The linear model is additive; a kinematic leg is
	// charged one start and one stop, so re-pricing the remaining 0.03 floors
	// as a fresh run bills a whole acceleration and deceleration ramp to a
	// cabin that is already moving and about to arrive - and the stop's ARRIVE
	// would fire early, with the cabin short of its floor.
	//
	// The leg is identified by the same anchors Elevator::update derives
	// position from (legStartPosition/legStartTime), which is what makes the
	// two agree by construction instead of by coincidence: the elapsed time is
	// measured from `legStartTime` and floored at 0, because a cabin still
	// sitting out a stop's dwell has a legStartTime in the future and has not
	// begun this leg yet, and `routeStartTime()` has already moved the clock
	// forward to that instant.
	double elapsed = currentTime - legStartTime;
	if (elapsed < 0) elapsed = 0;
	return travelStrategy->remainingDuration(destination - legStartPosition, elapsed);
}

double Elevator::firstLegArrival(int destination) const
{
	// A leg that is over has an end, and that end does not move. The clock
	// sum below cannot say so: with nothing left of the leg it answers "now",
	// which is only the arrival at the instant the cabin arrives. Between that
	// and its ARRIVE the doors are opening, and a route change in that gap
	// (somebody joining the stop, the pool re-offering a call, a stop taken
	// away) would date the arrival at itself and push the ARRIVE one
	// door-opening later each time.
	double total = travelStrategy->legDuration(destination - legStartPosition);
	double elapsed = currentTime - legStartTime;
	if (elapsed > 0.0 && elapsed >= total) return legStartTime + total;
	return routeStartTime() + legInFlightRemainder(destination);
}

double Elevator::routeStartTime() const
{
	// Where the route clock starts. While the cabin is in a dwell its
	// legStartTime is in the future and it has not moved yet, so travel to the
	// next stop can only begin then - accumulating from currentTime would date
	// every pending event as if the cabin had left the moment the doors opened.
	// With a zero dwell the two coincide.
	return (legStartTime > currentTime) ? legStartTime : currentTime;
}

double Elevator::getFinishTime(Task& task)
{
	vector<Task>::iterator it;
	double x = routeStartTime();
	double pos = position;
	bool firstLegInFlight = (status == BUSY && !tasks.empty());
	for (it = tasks.begin() ; it < tasks.end() ; ++it)
	{
		// The leg already in flight is the travel model's business, not this
		// loop's arithmetic: see legInFlightRemainder().
		x = (it == tasks.begin() && firstLegInFlight)
			? firstLegArrival(it->getDest())
			: x + travelStrategy->legDuration(it->getDest() - pos);
		pos = it->getDest();
		if (task == it) break;
		// Every stop before the one being estimated costs its full dwell - an
		// estimate that ignores door time is systematically optimistic, and this
		// is what the dispatch cost is built on.
		x += stopDwellTotal(*it);
	}
	return x;
}

double Elevator::getStopArrivalTime(unsigned long int stopId) const
{
	// Same accumulation as getFinishTime (the travel model's leg durations from
	// the current position, stop by stop, in route order, plus the full dwell of
	// every stop in between) but keyed by stop identity instead of Task
	// equality. The value is the instant
	// the cabin *arrives*; the ENTER/LEAVE event of the stop fires later, at
	// arrival + door-open - see getStopEventTime().
	double x = routeStartTime();
	double pos = position;
	bool firstLegInFlight = (status == BUSY && !tasks.empty());
	for (vector<Task>::const_iterator it = tasks.begin() ; it < tasks.end() ; ++it)
	{
		x = (it == tasks.begin() && firstLegInFlight)
			? firstLegArrival(it->getDest())
			: x + travelStrategy->legDuration(it->getDest() - pos);
		pos = it->getDest();
		if (it->getStopId() == stopId) return x;
		x += stopDwellTotal(*it);
	}
	return -1;
}

double Elevator::getLegStartTime() const
{
	return legStartTime;
}

bool Elevator::startedNewLeg() const
{
	return legStarted;
}

bool Elevator::routeFrontChanged() const
{
	return frontChanged;
}

double Elevator::getStopEventTime(unsigned long int stopId) const
{
	// The instant the ARRIVE of `stopId` fires (and its transfers are dated):
	// the doors have to open before anyone can move, so a customer's wait
	// includes that opening. Kept separate from getStopArrivalTime deliberately
	// - folding the door opening into the arrival would also charge it to
	// getFinishTime, i.e. to the dispatch cost estimate of a stop the elevator
	// has not reached yet.
	for (vector<Task>::const_iterator it = tasks.begin() ; it < tasks.end() ; ++it)
	{
		if (it->getStopId() != stopId) continue;
		double arrival = getStopArrivalTime(stopId);
		if (arrival < 0) return -1;
		return arrival + dwellStrategy->getDoorOpenTime(stopContextOf(*it));
	}
	return -1;
}

unsigned int Elevator::announcedDirectionAt(unsigned long int stopId) const
{
	// The first stop that is not the one being served. The route is kept
	// ordered for the cabin's current position and direction, and a cabin
	// standing at a floor does not move, so this is the entry processNextTask
	// will promote to the front once it erases `stopId` - the same reference
	// point, read one step earlier.
	for (vector<Task>::const_iterator it = tasks.begin() ; it != tasks.end() ; ++it)
	{
		if (it->getStopId() == stopId) continue;
		if (it->getDest() > position) return UP;
		if (it->getDest() < position) return DOWN;
		// A second entry for the floor the cabin is already at says nothing
		// about where it goes next; keep looking.
	}
	return IDLE;
}

const vector<Task>& Elevator::getRoute() const
{
	return tasks;
}

void Elevator::showStatus()
{
	if (status == IDLE) cout<<"The elevator number "<<getId()<<" is IDLE at floor "<<position<<endl;
	else
	{
		if (direction == UP) cout<<"The elevator number "<<getId()<<" is BUSY at floor "<<position<<" going in UP direction"<<endl;
		else if (direction == DOWN) cout<<"The elevator number "<<getId()<<" is BUSY at floor "<<position<<" going in DOWN direction"<<endl;
	}
	vector<Task>::iterator it;
	cout<<"The tasks assigned to this elevator are : "<<endl;
	for (it = tasks.begin() ; it < tasks.end() ; ++it)
	{
		if (it->getType() == GO) cout<<"Go to floor "<<it->getDest()<<" at time "<<it->getTime()<<endl;
		else if (it->getType() == CALL)
		{
			cout<<"Call to floor "<<it->getDest();
			if (it->getDirection() == UP) cout<<" in UP direction";
			else if (it->getDirection() == DOWN) cout<<" in DOWN direction";
			else cout<<"Unkown task. Exiting...."<<endl;
			cout<<" at time "<<it->getTime()<<endl;
		}
		else
		{
			cout<<"Unknown task. Exiting...."<<endl;
			exit(0);
		}
	}
}

void Elevator::update(Timer time)
{
	// Position is computed analytically from the leg currently in flight
	// (legStartPosition/legStartTime, set in addTask/processNextTask whenever
	// the immediate next stop is (re)decided) instead of being integrated tick
	// by tick, and is clamped to the leg's own destination - so it cannot
	// overshoot, however event timestamps get shifted around it.
	double temp = currentTime;
	currentTime = time.getCurrentTime();
	if (status == IDLE)
	{
		idleTime += currentTime - temp;
		return;
	}

	// The travel model owns both halves of this: how long the leg takes and
	// where the cabin is partway through it. The clamping that stops an
	// overshoot lives inside displacement(), whose contract requires it - see
	// ITravelTimeStrategy.h.
	double dest = processingTask.getDest();
	position = legStartPosition
		+ travelStrategy->displacement(dest - legStartPosition, currentTime - legStartTime);
}

void Elevator::addCustomer(Customer cus)
{
	// The rated load is a hard limit, not a target. Throwing rather than
	// clamping or warning: there is no sensible value to carry on with - the
	// passenger is either inside or on the landing - and every number the run
	// reports from here on would describe a cabin that cannot exist.
	// ElevatorSystem asks canBoard() first and leaves the passenger waiting
	// with their call still registered, so this throw is a bug in a caller,
	// not a scenario outcome.
	if (!canBoard(cus.getWeight()))
	{
		throw std::logic_error("Elevator::addCustomer: boarding would exceed the cabin's rated load "
			"- ask canBoard() first");
	}
	customers.push_back(cus);
	customerIn++;
	currentLoad += cus.getWeight();
}

void Elevator::removeCustomer(Customer* cus)
{
	vector<Customer>::iterator it;
	for (it = customers.begin() ; it < customers.end() ; ++it)
	{
		if (*it == cus)
		{
			// BUGFIX: erase() invalidates `it`; the old loop kept
			// incrementing it afterwards (undefined behaviour) instead of
			// stopping once the matching customer was found and removed.
			currentLoad = max(0.0, currentLoad - it->getWeight());
			customers.erase(it);
			if (customers.empty()) currentLoad = 0;		// see dischargeAt
			break;
		}
	}
	customerServed++;
	if (DEBUG) cout<<"CUSTOMERS SERVED : "<<customerServed<<endl<<endl;
}

vector<Customer> Elevator::dischargeAt(int floor)
{
	vector<Customer> leaving;
	vector<Customer>::iterator it = customers.begin();
	while (it != customers.end())
	{
		if (it->getTo() != floor) { ++it; continue; }
		// The same two steps the LEAVE branch ran, in the same order:
		// updateReach(cus) divided by the served count as it was BEFORE this
		// passenger, and removeCustomer(cus) incremented it afterwards. Keeping
		// that order is what makes this a refactor and not a new average.
		totReachingTime += (currentTime - it->getTime());
		avgReachingTime = totReachingTime/customerServed;
		currentLoad = max(0.0, currentLoad - it->getWeight());
		leaving.push_back(*it);
		it = customers.erase(it);
		customerServed++;
	}
	// The running sum does not come back to zero on its own: 70.1 + 80.2 -
	// 70.1 - 80.2 is 1.42e-14 in IEEE 754, and the max above only stops it
	// going negative. An empty cabin carries nothing, exactly, so every reader
	// of the load - the energy price of the next leg, a passenger policy's
	// StopContext - sees the same cabin nobody ever boarded.
	if (customers.empty()) currentLoad = 0;
	return leaving;
}

void Elevator::updateAvg(double time)
{
	totWaitingTime += currentTime - time;
	avgWaitingTime = totWaitingTime/customerIn;
}

double Elevator::getAvgWaitingTime()
{
	return avgWaitingTime;
}

double Elevator::getIdleTime()
{
	return idleTime/currentTime;
}

double Elevator::getTime()
{
	return currentTime;
}

void Elevator::updateReach(Customer* cus)
{
	totReachingTime += (currentTime - cus->getTime());
	avgReachingTime = totReachingTime/customerServed;
}

double Elevator::getAvgReachingTime()
{
	return avgReachingTime;
}

double Elevator::getCurrentLoad() const
{
	return currentLoad;
}

unsigned int Elevator::getPassengerCount() const
{
	return customers.size();
}

double Elevator::getMaxWeight() const
{
	return maxWeight;
}

const shared_ptr<ITravelTimeStrategy>& Elevator::getTravelTimeStrategy() const
{
	return travelStrategy;
}

const shared_ptr<IDwellTimeStrategy>& Elevator::getDwellTimeStrategy() const
{
	return dwellStrategy;
}

double Elevator::getBypassThresholdKg() const
{
	return capacityStrategy->getBypassThreshold(maxWeight);
}

bool Elevator::isBypassingCalls() const
{
	return currentLoad >= capacityStrategy->getBypassThreshold(maxWeight);
}

bool Elevator::canBoard(double weight) const
{
	// An unlimited cabin (maxWeight == infinity, the default) short-circuits to
	// true for any finite weight, so a run that configures no capacity never
	// refuses anybody.
	return currentLoad + weight <= maxWeight;
}

bool Elevator::canAnswerHallCall(double weight) const
{
	return !isBypassingCalls() && canBoard(weight);
}

const Task* Elevator::joinHallCall(unsigned long int stopId, unsigned int callDirection)
{
	if (callDirection != UP && callDirection != DOWN)
	{
		throw std::logic_error("Elevator::joinHallCall: a hall call must be UP or DOWN");
	}
	if (stopId == 0) return NULL;
	vector<Task>::iterator it;
	for (it = tasks.begin() ; it != tasks.end() ; ++it)
	{
		if (it->getStopId() != stopId) continue;
		it->addHallCall(callDirection);
		return &(*it);
	}
	return NULL;
}

bool Elevator::settleServedStop(unsigned long int stopId, int boarding, int alighting)
{
	for (vector<Task>::iterator it = tasks.begin() ; it != tasks.end() ; ++it)
	{
		if (it->getStopId() != stopId) continue;
		it->settlePassengers(boarding, alighting);
		return true;
	}
	return false;
}

unsigned long int Elevator::committedStopId() const
{
	return tasks.empty() ? 0 : tasks.front().getStopId();
}

bool Elevator::beginServing(unsigned long int stopId)
{
	servingStopId = 0;
	if (stopId == 0) return false;
	for (vector<Task>::iterator it = tasks.begin() ; it != tasks.end() ; ++it)
	{
		if (it->getStopId() != stopId) continue;
		// The same tolerance processNextTask uses to decide a cabin has arrived.
		// A stop whose event fired with the cabin somewhere else is not being
		// served, whatever its id says: that is a timing defect of its own, and
		// pinning it would move the cabin's next stop to a floor it is not at.
		if (std::abs(it->getDest() - position) >= 0.01) return false;
		servingStopId = stopId;
		if (it == tasks.begin()) return false;		// the normal case: nothing to repair
		// The sort had moved it. The cabin is standing here, so it goes back in
		// front, and the leg in flight - which arrived here - is told so, by the
		// same path a retarget uses, so stoppingPosition() and update() reason
		// about the stop the cabin is actually at.
		std::rotate(tasks.begin(), it, it + 1);
		retargetLegTo(tasks.front());
		return true;
	}
	return false;
}

void Elevator::endServing()
{
	servingStopId = 0;
}

Task& Elevator::addTaskBehindServedStop(Task& task, int flag)
{
	// For the floor the cabin is standing at: that is the stop being served,
	// and two entries for one floor with nothing between them are one stop.
	if (std::abs(tasks.front().getDest() - task.getDest()) < 0.001)
	{
		if (flag == 1) tasks.front().addPassengerOf(task);
		return tasks.front();
	}

	// The rest of the route, ordered and searched exactly as addTask does for
	// the whole of it - reorder first because findInsertionPoint's binary
	// search needs an ordered vector, stoppingPosition() because a floor the
	// cabin cannot stop at belongs in the return sweep.
	Task served = tasks.front();
	tasks.erase(tasks.begin());
	schedulingStrategy->reorder(tasks, direction, stoppingPosition());
	vector<Task>::iterator it = schedulingStrategy->findInsertionPoint(tasks, direction, stoppingPosition(), task);
	vector<Task>::iterator mergeInto = tasks.end();
	if (it != tasks.end() && abs(it->getDest() - task.getDest()) < 0.001) mergeInto = it;
	else if (it != tasks.begin() && abs((it - 1)->getDest() - task.getDest()) < 0.001) mergeInto = it - 1;

	unsigned long int mergedInto = 0;
	if (mergeInto != tasks.end())
	{
		if (flag == 1) mergeInto->addPassengerOf(task);
		mergedInto = mergeInto->getStopId();
	}
	else
	{
		task.setStopId(nextStopId++);
		if (flag == 1) task.initPassengerCounters();
		tasks.insert(it, task);
	}
	// Back in front. No retarget either way: the new task cannot have become the
	// next stop, the cabin is standing at it.
	tasks.insert(tasks.begin(), served);

	if (mergedInto == 0) return task;
	// Looked up again rather than kept: putting the served stop back moved
	// every element, so an iterator taken before would dangle.
	for (vector<Task>::iterator m = tasks.begin() ; m != tasks.end() ; ++m)
	{
		if (m->getStopId() == mergedInto) return *m;
	}
	throw std::logic_error("Elevator::addTaskBehindServedStop: the stop a task merged into is gone");
}

bool Elevator::releaseHallCall(unsigned long int stopId, unsigned int callDirection)
{
	if (stopId == 0) return false;
	vector<Task>::iterator it;
	for (it = tasks.begin() ; it != tasks.end() ; ++it)
	{
		if (it->getStopId() != stopId) continue;
		if (!it->hasHallCall(callDirection)) return false;
		it->releaseHallCall(callDirection);
		// The entry still has a reason to exist (a car call, or another hall
		// call that was merged into the same stop), so the cabin still stops
		// here - it just has one fewer passenger to pick up.
		if (it->servesAnyone()) return true;
		// Nobody is left at this floor for this cabin. Dropping the entry is
		// what makes it drive past instead of making a stop where nothing
		// happens - except for the stop it is already committed to, which it
		// cannot brake out of (see the header). That one keeps its place in the
		// route with its counters already decremented, and processNextTask pops
		// it as usual when its events have fired.
		if (stopId == committedStopId()) return true;
		tasks.erase(it);
		return true;
	}
	return false;
}

} // namespace openEGCS
