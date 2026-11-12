/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : Elevator.h

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

#ifndef __ELEVATOR__
#define __ELEVATOR__

#include<iostream>
#include<string>
#include<vector>
#include<utility>
#include<memory>
#include<limits>
#include "Task.h"
#include "Timer.h"
#include "Customer.h"
#include "IElevatorSchedulingStrategy.h"
#include "LookSchedulingStrategy.h"
#include "ICapacityStrategy.h"
#include "PercentageCapacityStrategy.h"
#include "IDwellTimeStrategy.h"
#include "FixedDwellTimeStrategy.h"
#include "ITravelTimeStrategy.h"
#include "LinearTravelTimeStrategy.h"

#ifndef __STATUSCODES__
#define __STATUSCODES__

#define BUSY 0x8057
#define IDLE 0x1D1E

#endif

using namespace std;

namespace openEGCS {

// A single elevator cabin: its task queue (what to do next), physical state
// (position/direction/status/load), and per-elevator statistics. Ordering
// its own route is delegated to IElevatorSchedulingStrategy (LOOK by
// default); ElevatorSystem owns a vector of these and decides which one
// answers a given call via the injected IDispatchStrategy. See
// docs/architecture.md for the full picture.
//
// `position` is derived, not integrated: update() computes it analytically
// from where/when the current leg (`processingTask`) started
// (`legStartPosition`/`legStartTime`), clamped to that leg's own distance -
// see update()'s own comment.
class Elevator
{

	private:
		int id;						// Id of the elevator
		double position;			// Stores the current position of the elevator, in floors (fractional while moving)
		Task processingTask;		// Stores the task which the elevator is currently performing
		unsigned int status;		// Has the status of the elevator BUSY or IDLE (as defined above)
		unsigned int direction; 	// Stores which direction it is currently moving - UP or DOWN. IDLE if the Elevator is IDLE.
		vector<Task> tasks;			// Stores the task list of each elevator
		double staticTime;			// Time units to cross 1 floor. Only used to build the default LinearTravelTimeStrategy; the travel model owns this once injected
		double currentTime;			// Current time synchronized everywhere
		double idleTime;			// Stores the time for which elevator was idle
		vector<Customer> customers;	// Stores the list of customers in the elevator
		double legStartPosition;	// Position at which the current leg (processingTask) started
		double legStartTime;		// currentTime at which the current leg started (see Elevator::update)
		double avgWaitingTime;
		double totWaitingTime;
		double totReachingTime;
		double avgReachingTime;
		int customerServed;
		int customerIn;
		shared_ptr<IElevatorSchedulingStrategy> schedulingStrategy;	// Pluggable "how does this elevator order its own queue" policy (default: LOOK)
		double maxWeight;			// Maximum total customer weight this cabin can carry (default: unlimited)
		double currentLoad;			// Sum of the weight of customers currently on board
		shared_ptr<ICapacityStrategy> capacityStrategy;	// Pluggable "load bypass threshold" policy (default: 60% of maxWeight)
		shared_ptr<IDwellTimeStrategy> dwellStrategy;	// Pluggable "how long a stop takes" policy (default here: all zero; the installation default is ElevatorSystem's)
		shared_ptr<ITravelTimeStrategy> travelStrategy;	// Pluggable "how long a leg takes, and where the cabin is partway through it" policy
		unsigned long int nextStopId;	// Next Task::stopId to hand out (starts at 1; 0 means "unassigned"). Never reused.

		// A stop has been completed and removed from the route, but the next
		// leg has not been chosen for it yet. See processNextTask's
		// holdLegDecision parameter.
		bool legDecisionPending;
		// Did the most recent processNextTask() complete a stop and begin a new
		// leg? See startedNewLeg().
		bool legStarted;
		// Did the most recent processNextTask() change which stop is at the
		// front of the route? See routeFrontChanged().
		bool frontChanged;
		// The stop being served right now - see beginServing(). While it is set
		// it is pinned at the front of the route, and the scheduling strategy is
		// handed the rest of the route, never this entry. 0 when no stop is
		// being served.
		unsigned long int servingStopId;
		// addTask while a stop is being served: the new task goes somewhere in
		// the rest of the route, never in front of the served stop, and merges
		// into it when it is for the same floor.
		Task& addTaskBehindServedStop(Task& task, int flag);

		// How much of a stop's dwell is still owed once its transfers have
		// happened: transfer + door close. This is what delays the cabin's
		// departure.
		double stopDwellTail(const Task& stop) const;
		// Door open + transfer + door close: the whole time a stop costs, which
		// is what every *later* stop in the route is pushed back by.
		double stopDwellTotal(const Task& stop) const;
		// Where the route clock starts: currentTime, or legStartTime while the
		// cabin is still sitting in a dwell (legStartTime in the future).
		double routeStartTime() const;
		// Time still owed on the leg currently in flight, measured from the
		// leg's own anchors and answered by the travel model - not recomputed
		// as a fresh leg from the cabin's current fractional position, which is
		// only equivalent for a travel model that is additive in distance.
		double legInFlightRemainder(int destination) const;
		// When the leg in flight reaches `destination`. While it is under way
		// that is the route clock plus legInFlightRemainder. Once it is over it
		// is the instant it ended, which is not "now": the cabin arrived, and its
		// doors are still opening towards an ARRIVE that has not fired. Answering
		// "now" there would restart the opening every time anything touched the
		// route.
		double firstLegArrival(int destination) const;
		// Asks the scheduling strategy to re-establish route order for where the
		// cabin is and which way it is going, and re-anchors the current leg when
		// that changes which stop comes first. Route order is relative to the
		// cabin's state, so it goes stale on its own as the cabin moves and
		// reverses - this is what keeps the stored vector matching the order the
		// strategy's own comparator implies.
		void reorderRoute();

		// The first floor the cabin can still stop at, which is where it
		// effectively IS as far as route order is concerned.
		//
		// A cabin standing at a floor can stop at that floor, so this is just
		// `position` for it. A cabin in flight cannot: it has braking distance
		// to cover, so every floor between `position` and this one will be
		// passed whether or not someone is waiting there - not by choice, but
		// because stopping needs room (ITravelTimeStrategy::minimumLegDistance,
		// Peters eq 6.103-6.112). Ordering the route against this instead of
		// against `position` is what puts such a floor in the return sweep,
		// where it belongs: the cabin carries on and comes back for it, which
		// is what a real lift does when you watch it go past your floor.
		//
		// For LinearTravelTimeStrategy this is EXACTLY `position`, because a
		// constant-velocity cabin needs no room to stop.
		double stoppingPosition() const;

		// Points the leg in flight at `newFront` instead of wherever it was
		// going. This is Peters' ChangeJourney, as distinct from his
		// StartJourney - the two are separate functions in his own simulator
		// (section 8.3.4): a leg already under way keeps its ORIGIN and its CLOCK
		// and changes only its destination, which is what section 6.6 prescribes
		// and what keeps the motion continuous. Re-anchoring the leg at the
		// cabin's current position instead would restart it from rest - a cabin
		// dropped from full rated speed to zero in an instant.
		//
		// Two call sites can change which stop comes first while a cabin is
		// moving - an insertion that overtakes the current target (addTask) and
		// a re-sort that promotes a different stop (reorderRoute) - and both go
		// through here, so that neither can reintroduce the restart.
		//
		// A cabin standing at a floor (idle, or sitting out a dwell) is not in
		// flight, so for it this IS a fresh start: it anchors the leg here.
		void retargetLegTo(const Task& newFront);

	public:
		// What a dwell strategy is told about a route entry: how many board and
		// alight there (the entry's counters) and why the cabin stops there -
		// a car call, a landing call in either direction, or both. Public and
		// static because it reads nothing but the Task, and because the door
		// hold depends on it being right, so a test pins it.
		static StopContext stopContextOf(const Task& stop);

		// `scheduling_strategy` is optional: when null, a LookSchedulingStrategy
		// is instantiated internally, making the elevator behave like a LOOK
		// elevator. Inject a custom implementation of IElevatorSchedulingStrategy
		// here to test a route-ordering policy on one cabin in isolation; to run it
		// through a full simulation, inject it into Simulator/ElevatorSystem
		// instead, which share one instance among every cabin. This fallback is the
		// same class ElevatorSystem defaults to, so a directly-constructed cabin and
		// a simulated one order routes alike.
		// `max_weight` defaults to unlimited; `capacity_strategy` defaults to
		// PercentageCapacityStrategy (see ICapacityStrategy.h).
		// `dwell_strategy` defaults to a FixedDwellTimeStrategy of all zeros, i.e.
		// the instantaneous stop. That is the default of a bare cabin, as unit
		// tests build it, and NOT the simulator's: ElevatorSystem hands every cabin
		// the handbook's centre-door preset (DwellPresets.h), because the door width
		// needs the installation's rated load - the same split as the travel
		// model's fallback below.
		// `travel_strategy` decides how long a leg takes and where the cabin
		// is partway through it. `time` is only used to build the fallback; inject
		// a strategy and it is ignored.
		//
		// The fallback here is the LINEAR model even though the simulator's
		// default is the kinematic one. That is not an oversight and not a
		// second default: the kinematic profile needs the installation's rated
		// dynamics, which are derived from how far the lift travels, and a
		// cabin built directly has no idea how tall its building is.
		// ElevatorSystem does, so that is where the kinematic default lives -
		// and every simulation goes through it. A directly-constructed
		// Elevator is a test fixture or a caller doing something deliberate,
		// and gets the model whose only parameter it actually supplied.
		Elevator(unsigned long int el_id = 0, string el_name = "", double pos = 0, double time = 1,
			shared_ptr<IElevatorSchedulingStrategy> scheduling_strategy = nullptr,
			double max_weight = std::numeric_limits<double>::infinity(),
			shared_ptr<ICapacityStrategy> capacity_strategy = nullptr,
			shared_ptr<IDwellTimeStrategy> dwell_strategy = nullptr,
			shared_ptr<ITravelTimeStrategy> travel_strategy = nullptr); 	// Elevator constructor
		
		int getId();								// Returns elevator id
		double getPosition() const;					// Returns the current position of the elevator
		unsigned int getDirection() const; 			// Returns which direction it is currently moving - UP or DOWN. IDLE if the Elevator is IDLE, and never while it is BUSY.
		unsigned int getStatus() const;				// Return the current status of the Elevator - IDLE if there are no tasks in the task queue BUSY otherwise
		const Task& getProcessingTask() const;		// Returns the task the elevator is currently performing
		
		// Add the Task task to the task Queue and return a reference to the entry that
		// now represents it. Assigns `task.stopId`: a fresh, elevator-unique id when
		// the task becomes a new route entry (also set on the caller's `task`, so the
		// returned reference carries it), or - when the task is merged into an
		// existing entry for the same floor - the existing entry's id, which is what
		// the returned reference then holds. Callers that need the stop
		// (ElevatorSystem, to tag the ARRIVE event) must read it from the RETURN value
		// of the real (flag=1) call: a dry-run (flag=0) followed by undo also
		// consumes an id, so ids are unique but not contiguous.
		//
		// Guarantees that a cabin it makes BUSY has a direction, UP or DOWN, which
		// is what IElevatorSchedulingStrategy is promised. For a parked cabin
		// given a landing call at the floor it is standing at (a leg of zero
		// length) that is the call's own direction; a car call to that floor has
		// none to lend and throws std::logic_error. A BUSY cabin left at IDLE would
		// sort the stop it stands at behind the next car call and never finish it.
		Task& addTask(Task& task, int flag);
		// Finishes the stop the caller just served and starts the next leg.
		// `finishedStopId` is the Task::getStopId of the stop being served, and the
		// front stop of the route is removed **only if it is that same stop**.
		// Identifying it by floor proximity instead (which is what 0 does) removes
		// the wrong entry whenever the route holds two stops for one floor, and that
		// is not rare: a cabin legitimately stops at floor 5 going up and at 5 again
		// coming down.
		// Passing 0 keeps the proximity rule, for callers with no stop in hand. In
		// practice that is only tests driving an Elevator directly
		// (ElevatorSystem::doTask would be the other one, but nothing calls it).
		// `holdLegDecision`: a stop's transfers are processed one at a time, and
		// each boarding passenger's car call is only registered as they board.
		// Choosing the sweep direction on the stop's *first* transfer would choose
		// it without knowing where the other boarders want to go - and price the
		// leg's energy before they are counted in the load. With `holdLegDecision`
		// true the served entry still leaves the route and still pays its dwell
		// tail, but the next leg is not chosen; the caller passes false on the
		// stop's last transfer and the decision happens then. The cabin keeps its
		// previous processingTask in between, which is safe only because every
		// transfer of a stop shares one timestamp - the window has zero duration
		// in simulated time.
		//
		// Callers with no stop in hand (finishedStopId == 0) must leave it
		// false: nothing would ever deliver the stop's remaining transfers, and
		// the cabin would stay parked with a stale leg.
		const Task& processNextTask(unsigned long int finishedStopId = 0,
			bool holdLegDecision = false);
		// True when the most recent processNextTask() both finished a stop and
		// started a new leg out of it - which is exactly when a departure
		// should be reported. False for a deferred call (a stop whose other
		// transfers have not been processed yet), for a re-confirmation that
		// removed nothing, and when the cabin parked.
		//
		// Not derived by comparing getProcessingTask().getStopId() before and
		// after: Elevator::addTask re-anchors processingTask whenever an
		// inserted task becomes the route's front, so the id can change for a
		// reason that has nothing to do with a leg starting.
		bool startedNewLeg() const;
		// Did the most recent processNextTask() change which stop the route
		// serves next - by completing one, or because the scheduling strategy
		// re-sorted and a different stop came to the front?
		//
		// Callers that derive event timestamps from the route have to re-time
		// when this is true, and only then: a stop is re-sorted away from the
		// front for real reasons (a cabin that turns UP stops serving a DOWN
		// hall call above it until the way back), and its pending event then
		// holds a timestamp computed for a position in the route it no longer
		// has. Left stale, that event would fire early, find its stop not at the
		// front, finish nothing and strand the stop. Re-timing unconditionally is
		// not an option either: the same route sum re-derived when nothing changed
		// need not land on the same bits, and the queue's (time, seq) tie-break
		// turns that into a different simulation. See
		// ElevatorSystem::advanceAndReportDeparture.
		bool routeFrontChanged() const;
		void removeTask(Task& t);					// Remove the Task t from the Task queue.
		// Estimated time to finish every task in the queue up to `task`: the
		// dispatch cost an IDispatchStrategy ranks elevators by. Not the actual
		// time anything happens - getStopArrivalTime is that.
		//
		// A double, not a float: a float would truncate the cost to a 24-bit
		// mantissa and so manufacture ties between elevators whose real costs
		// differ, and a tie is then broken by whichever one the strategy's loop
		// saw first, which is arbitrary.
		double getFinishTime(Task& task);
		// Estimated simulation time at which this elevator reaches the route stop
		// `stopId` (Task::getStopId), assuming it keeps its current route: the leg
		// in flight, then each later leg (ITravelTimeStrategy::legDuration) plus the
		// full dwell of every earlier stop, summed stop by stop. Starts from
		// max(currentTime, legStartTime), because a cabin sitting out a dwell has
		// not moved yet. Returns -1 if that stop is not (or no longer) in the
		// route. Because it is derived from the live task vector every time it is
		// asked, it is always consistent with the route.
		double getStopArrivalTime(unsigned long int stopId) const;
		// Instant the ARRIVE event of route stop `stopId` fires (and so the instant
		// of its transfers): its arrival time plus the door-open time of
		// IDwellTimeStrategy, because nobody gets in or out through a shut door.
		// Returns -1 if the stop is not in the route. Deliberately separate from
		// getStopArrivalTime: folding the door opening into the arrival would also
		// charge it to getFinishTime, i.e. to the dispatch cost of a stop the
		// elevator has not reached yet. This is what ElevatorSystem times events
		// from; with a zero dwell the two are equal.
		double getStopEventTime(unsigned long int stopId) const;
		// Simulation time at which the leg currently in flight started - or will
		// start, when the cabin is still sitting out a stop's dwell and this is in
		// the future. ElevatorSystem reports it as the real departure instant of a
		// trip.
		double getLegStartTime() const;
		
		// Which way this cabin will go when it leaves route stop `stopId`: UP,
		// DOWN, or IDLE when there is nowhere else to go.
		//
		// This is the direction indicator of a real lift, and the point of it is
		// WHEN it can be asked: before the stop is served, so before anybody
		// boards. getDirection() cannot answer that - it is the direction the
		// cabin is travelling *now*, which at a stop is the direction it arrived
		// in, and at a sweep reversal that is the opposite of where it is about to
		// go.
		//
		// Derived from the route and from nothing else: the first stop that is
		// not this one, compared against the cabin's position. No new parameter,
		// no constant, and the same rule processNextTask applies after the fact -
		// which is what makes the announcement honest rather than a second
		// opinion. A cabin whose route holds only this stop announces IDLE: it has
		// no reason to go either way, and deciding one for it would be inventing a
		// policy.
		//
		// It is NOT a promise. Between the announcement and the cabin actually
		// leaving, two things can still change the route: the car call of somebody
		// who boards here (which goes in before the leg is decided) and a stop
		// dropped by load bypass. Either can make the cabin go the other way, which
		// is what Closs rule 4 (no reversal with passengers aboard) forbids; the
		// engine does not enforce rule 4 by construction.
		unsigned int announcedDirectionAt(unsigned long int stopId) const;

		// The route as the cabin will actually drive it: the stops still to serve,
		// in service order. Read-only, and kept ordered by the injected
		// IElevatorSchedulingStrategy for the cabin's current position and
		// direction - so for the default LOOK policy the floors form one monotone
		// run in the direction of travel followed by one in the opposite
		// direction, never more. Exposed because that ordering is an observable
		// property worth asserting in tests, and because anything showing a cabin's
		// plan needs it; `tasks` itself stays private so only
		// addTask/processNextTask can change the route.
		const vector<Task>& getRoute() const;

		void showStatus();							// Displays all tasks of the elevator
		void update(Timer time);					// Updates the position,time,etc. of the elevator
		// Puts `cus` on board, adding their weight to the cabin's load.
		// Throws std::logic_error if that would take the load past
		// getMaxWeight(): a cabin carrying more than its rated load is not a
		// bad outcome to be measured, it is a physically impossible state.
		// Callers must therefore ask canBoard() first; ElevatorSystem does, and
		// turns a `false` into a passenger left on the landing with their call
		// still registered.
		void addCustomer(Customer cus);
		void removeCustomer(Customer* cus);			// Removes a customer from the customers vector
		// Takes off every passenger on board whose destination is `floor`, in
		// the order they boarded, and returns them. The reaching-time average is
		// updated before the served count grows, passenger by passenger.
		//
		// "Whose destination is this floor" is the honest rule - Rule 2 of the
		// five rules of lift traffic control says a car never passes a floor where
		// somebody wants to alight.
		//
		// Removal is by ITERATOR, deliberately, never through
		// removeCustomer(Customer*): Customer::operator== compares
		// (timestamp, atFloor, toFloor, direction) and ignores both the id and
		// the weight, so it is identity-by-value - the same trap as
		// Task::operator==. Matching by value here would work until two
		// passengers shared an arrival instant.
		//
		// Note that the cabin holds *copies* of its passengers (addCustomer
		// takes one by value), so what comes back are copies carrying the same
		// id, weight, floors and arrival time - enough for the metric hooks and
		// the snapshot, both of which only read.
		vector<Customer> dischargeAt(int floor);
		void updateAvg(double time);				// Updates the average time
		double getTime();							// Returns current time
		
		double getAvgWaitingTime();					// Returns the average waiting time of customers of this elevator
		double getIdleTime();						// Returns idle time of the elevator
		void updateReach(Customer* cus);			// Updates average reaching time

		double getCurrentLoad() const;				// Returns the sum of the weight of customers currently on board
		// How many people are on board. This, not getCurrentLoad() > 0, is
		// the answer to "is this cabin carrying anybody?": the load is a
		// running sum of non-integer weights and only comes back to zero
		// because the last alighting resets it, while a count is exact by
		// construction.
		unsigned int getPassengerCount() const;
		double getMaxWeight() const;					// Returns the maximum weight this elevator can carry
		// The load (kg) at which isBypassingCalls() turns true: what the
		// capacityStrategy answers for this cabin's rated load. Infinite for an
		// unlimited cabin. Exposed for the snapshot's metadata.
		double getBypassThresholdKg() const;
		bool isBypassingCalls() const;				// True once currentLoad crosses the capacityStrategy's threshold: the cabin is "full" as far as hall calls go, and stops answering them
		// Is there physically room for one more passenger of this weight?
		// The rated load is the hard limit: unlike isBypassingCalls(), which
		// answers a *policy* question about a load-weighing device set well below
		// capacity, this one answers whether the lift could carry them at all. An
		// unlimited cabin (the default) always says yes.
		bool canBoard(double weight) const;
		// May this cabin be given a hall call for a passenger of this weight?
		// Two conditions: it must not be bypassing, and the passenger must
		// physically fit.
		//
		// The first is the load-weighing device of Barney & Al-Sharif section
		// 10.8.3 - "a device is usually available" - a property of the
		// equipment rather than a preference of the dispatcher, which is why
		// the engine enforces it even when an IDispatchStrategy ignores it
		// (ElevatorSystem::allocateCall). It is measured on the load the
		// cabin is CARRYING, exactly as the real device is: a car at 58% still
		// answers a call and arrives at 70%, which section 14.3.5 states in so
		// many words.
		//
		// What this deliberately does NOT do is reserve a place for the
		// passengers already allocated to this cabin and not yet collected.
		// Capping the allocations at what the cabin can carry withholds
		// registered calls from every cabin, and cars then drive past floors they
		// have room for: tried, it turned a servable scenario (20 floors, 4 cabins
		// of 1200 kg, one arrival per second) from 87 s of average waiting into
		// 2416 s. That is the "refusal to make an allocation" of section 11.6.3.
		//
		// A car call (GO) is never subject to any of this: the passenger is
		// already inside.
		bool canAnswerHallCall(double weight) const;
		// The stop this cabin can no longer get out of serving: the front of
		// the route, or 0 when the route is empty. A hall call at any OTHER
		// stop can still be taken away from this cabin and given to another
		// one, which is what load bypass does; the committed one cannot,
		// because a cabin in flight has braking distance it has already
		// started spending (see stoppingPosition()). The full cabin therefore
		// still stops there - and the people waiting are refused at the open
		// door instead, which is the one case that cannot be designed away.
		unsigned long int committedStopId() const;
		// The cabin is standing at stop `stopId` with its doors open: its ARRIVE
		// has fired. From now until processNextTask finishes it (or endServing),
		// that stop is the front of the route and nothing is put in front of it -
		// reorderRoute and addTask order only the stops after it.
		//
		// This is what makes committedStopId() true by construction. The
		// scheduling strategy sorts the whole route, and some states would make
		// it sort the stop the cabin is standing at behind another one: a stop
		// whose passengers another cabin had taken (servesSweep() false both
		// ways), or two entries for the same floor whose ARRIVEs fire at the same
		// instant. processNextTask would then find another stop at the front and
		// finish nothing, the stop could be deleted as "not committed", no leg
		// would start, and every later ARRIVE of that cabin would fire at a floor
		// it was not at.
		//
		// Brings the stop to the front if the sort had moved it, but only when
		// the cabin really is at its floor: a stop whose ARRIVE fires with the
		// cabin elsewhere is a timing defect, and pinning it would hide that.
		// Returns true when the route changed (so the caller re-times the
		// cabin's pending events).
		bool beginServing(unsigned long int stopId);
		// The doors are open at stop `stopId` and it is now known who transfers:
		// from here on that stop costs `boarding` and `alighting`, not what was
		// registered on it (see StopContext). It has to be called before the
		// stop's first transfer, because that one takes the stop off the route and
		// fixes what its dwell still owes (processNextTask). Returns whether a stop
		// was found; the later stops of the route are NOT re-timed here, because
		// the first transfer takes this one off the front, which re-times them
		// from a route that no longer holds it.
		bool settleServedStop(unsigned long int stopId, int boarding, int alighting);
		// Clears the pin, whatever happened to the stop. ElevatorSystem calls it
		// at the end of every ARRIVE, so a pin can never outlive the event that
		// set it.
		void endServing();
		// Takes one hall call of `callDirection` back out of route stop
		// `stopId`, and drops the stop entirely if that leaves it serving
		// nobody. Returns true when the call was found and released.
		//
		// This is the engine's half of load bypass: a cabin that has filled up
		// gives its pending pickups back to the dispatcher instead of making stops
		// where nobody can board - "such stops would be useless and particularly
		// annoying to the passengers already in the lift" (Barney & Al-Sharif
		// 10.8.3).
		//
		// Refuses to drop the committed stop (committedStopId()), so no release
		// can ever change which stop comes next, reverse the cabin, park it, or
		// re-anchor a leg in flight. The counters of the committed stop ARE
		// still decremented, because a passenger who did not board must not be
		// charged a transfer time - it is only the route entry that stays.
		//
		// Later stops in the route move EARLIER when one is dropped (the dwell
		// of the dropped stop is no longer in the sum), so the caller has to
		// re-time that elevator's pending events afterwards.
		bool releaseHallCall(unsigned long int stopId, unsigned int callDirection);
		// Adds one more waiting passenger to route stop `stopId`, for a hall
		// call of `callDirection`, and returns that route entry - or nullptr
		// when the stop is not (or no longer) in the route.
		//
		// This is how the second person to press an already-lit button gets
		// served: a landing call is one per (floor, direction), so they do not
		// create a second route entry and the dispatcher is not consulted again -
		// they join the entry the first person's call created. The exact mirror of
		// releaseHallCall, and it finds the entry the same way, **by stopId and not
		// by value**: the route can legitimately hold two entries for one floor
		// (one in each sweep), and Task::operator== cannot tell them apart.
		//
		// Deliberately NOT routed through addTask, which would be the obvious
		// alternative and is wrong here: addTask re-sorts the route and picks
		// an insertion point from the cabin's position and direction, so it can
		// merge into the *other* entry for that floor - the one in the return
		// sweep. Nothing is inserted, nothing is re-sorted, and no stopId is
		// consumed.
		//
		// Two things it does NOT judge, both left to the caller because only
		// the caller knows them: whether that stop is still a sensible place
		// for this passenger to board (a cabin standing at the floor with its
		// doors open has already fired its ARRIVE, so joining it serves
		// nobody), and whether the cabin has room (that is canBoard, asked at
		// the door). What it does change is the stop's dwell, since it now
		// serves one more transfer - so every stop AFTER it in the route moves
		// later, and the caller has to re-time that elevator's pending events,
		// exactly as releaseHallCall requires for the opposite reason.
		//
		// Throws std::logic_error if `callDirection` is neither UP nor DOWN: a
		// hall call without a direction is not a thing a landing button can
		// produce.
		const Task* joinHallCall(unsigned long int stopId, unsigned int callDirection);
		// The travel model this cabin was built with: how long a leg of N
		// floors takes and where the cabin is partway through it. Exposed for
		// the same reason as getDwellTimeStrategy() - anything reasoning about
		// arrival times needs the same numbers the elevator itself uses - and
		// because a test comparing two travel models needs to know which one
		// a cabin got. Never null.
		const shared_ptr<ITravelTimeStrategy>& getTravelTimeStrategy() const;
		// The stop-duration policy this cabin was built with. Exposed because
		// the dwell of a stop is part of the elevator's route timing, so a
		// caller reasoning about arrival times (or a test) needs the same
		// numbers the elevator itself uses. Never null.
		const shared_ptr<IDwellTimeStrategy>& getDwellTimeStrategy() const;
		double getAvgReachingTime();				// Returns average reaching time
};

} // namespace openEGCS

#endif
