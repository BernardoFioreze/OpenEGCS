/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : ElevatorSystem.h

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

#ifndef __ELEV_SYSTEM__
#define __ELEV_SYSTEM__

#include<iostream>
#include<vector>
#include<memory>
#include<limits>
#include<map>

#include "Event.h"
#include "Elevator.h"
#include "KinematicTravelTimeStrategy.h"
#include "Timer.h"
#include "EventQueue.h"
#include "util/Matrix.h"
#include "IDispatchStrategy.h"
#include "IGroupDispatchStrategy.h"
#include "IMetricsCollector.h"
#include "ICapacityStrategy.h"
#include "IPassengerBehaviorStrategy.h"
#include "IRandomProvider.h"


using namespace std;

namespace openEGCS {

// Owns every Elevator in the building plus the shared event queue
// (`upcomingEvents`) driving the simulation. `giveEvent` is the state
// machine that reacts to each event type (CUST_CALL/RECALL/ARRIVE),
// mutating the relevant Elevator and reporting to the injected
// IDispatchStrategy/IMetricsCollector as needed - see docs/architecture.md
// for how the pluggable pieces fit together. `Simulator` owns one
// ElevatorSystem and drives the event loop; this class knows nothing about
// ISimulationLogger/snapshot output (see drainDepartures() below for how
// that stays decoupled).
class ElevatorSystem
{
	public:
		// Records the instant an elevator actually starts moving towards its
		// next stop (as opposed to the event that triggered it, which may be a
		// different customer's call, or the end of a stop's dwell - see
		// giveEvent). Lets a snapshot reader draw the real trajectory instead of
		// interpolating linearly across the whole gap between two stops.
		struct Departure {
			int elevatorId;		// 1-indexed, matches Elevator::getId()
			int fromFloor;		// Floor the elevator is leaving (its position at `time`)
			double time;		// Simulation time the departure happens at
		};

		// One passenger transfer at a stop: a boarding (ENTER), an alighting
		// (LEAVE), or a cabin that arrived and could not take somebody (PASS).
		//
		// These are records, not events, for the reason the DEPART above is one:
		// none of them has an instant of its own (all three happen at their
		// stop's ARRIVE), and none of them can be scheduled, because who boards
		// is not knowable until the cabin is standing at the floor with its doors
		// open. They are reported by the engine during giveEvent and drained by
		// the caller, exactly as departures are - which is what keeps this class
		// ignorant of ISimulationLogger.
		//
		// `type` is one of ENTER / LEAVE / PASS (Event.h). The order within one
		// stop is the physical one: everybody who gets out, then everybody who
		// gets in. With a finite cabin the order matters: boarding before
		// alighting would refuse somebody for lack of room while a passenger
		// bound for that very floor was still inside.
		struct Transfer {
			unsigned int type;				// ENTER, LEAVE or PASS
			int elevatorId;					// 1-indexed, matches Elevator::getId()
			int floor;						// Floor it happens at
			unsigned long int customerId;	// Customer::getId() of the person it happened to
			double time;					// Simulation time (the stop's event time)
		};

	private:
		vector<Elevator> elevators;												// Stores the elevators in the form of a vector
		int number;																// Stores the number of elevators
		int floors;																// Stores the number of floors
		util::Matrix<Customer> allCustomers;									// Stores the customers on all floors
		shared_ptr<IDispatchStrategy> dispatchStrategy;						// Pluggable "which elevator gets this call" policy
		// The GROUP dispatch seam: the engine's allocation path goes through
		// this, never through dispatchStrategy directly. Defaults to an
		// ImmediateGroupAdapter wrapping dispatchStrategy, which delegates
		// per-call and never reallocates. A real group strategy is opt-in by
		// injection. dispatchStrategy is kept alongside it because
		// FindBestElevator still exposes the per-call point directly (the
		// dispatch contract test drives it).
		shared_ptr<IGroupDispatchStrategy> groupDispatch;
		// The simulator's shared randomness, handed on to the group dispatcher's
		// allocate() so a metaheuristic draws from the one seeded source and
		// determinism holds. Never null (a fallback is built when none is
		// injected); the default adapter draws nothing from it.
		shared_ptr<IRandomProvider> rng;
		shared_ptr<IMetricsCollector> metrics;									// Pluggable metrics collection
		// Pluggable "does this passenger get into this car" policy.
		// One shared instance, asked per passenger - see
		// IPassengerBehaviorStrategy.h for why that is not the same kind of
		// sharing as the three installation policies.
		shared_ptr<IPassengerBehaviorStrategy> passengerBehavior;
		vector<Departure> pendingDepartures;									// Departures recorded during the most recent giveEvent() call(s), drained by drainDepartures()
		vector<Transfer> pendingTransfers;										// Boardings/alightings/refusals recorded during the most recent giveEvent() call(s), drained by drainTransfers()

		// A landing call: ONE per (floor, direction), with the people behind it
		// as a queue. This is the unit the literature's controller uses -
		// Peters' dispatcher holds m_UpLandingCalls[MAX_FLOORS] and
		// m_DownLandingCalls[MAX_FLOORS] and cancels a call when a car arrives
		// (section 8.3.5) - and it is what a landing button is: the second
		// person to press one that is already lit does not register a second
		// call, they see the light is on.
		//
		// The queue is the half that cannot be thrown away: the call belongs to
		// the floor, but the WAIT belongs to each person, measured from their
		// own first press. In arrival order, so the cabin takes people in the
		// order they turned up and so an overflowing landing cannot starve the
		// person who has been there longest.
		//
		// It is a queue of people and nothing else: who boards is resolved when
		// a cabin arrives, so there is no per-person booking to track. The
		// invariant to defend is local: a call with somebody in this queue is
		// either allocated to a cabin whose route holds `stopId`, or sitting in
		// `unallocatedCalls` (or its people hold a pending RECALL).
		//
		// `allocatedElevator`/`stopId` are the cabin coming for it and the
		// route entry that serves it, or 0/0 for a call no cabin is answering -
		// the state "landing calls should not be cancelled" (Barney & Al-Sharif
		// 10.8.3) requires somewhere to live. `registeredAt` is when the button was
		// first pressed and is what the dispatch cost sees, so a call does not
		// get younger because somebody joined it.
		struct LandingCall {
			int floor = 0;
			unsigned int direction = 0;
			vector<Customer*> waiting;
			int allocatedElevator = 0;
			unsigned long int stopId = 0;
			double registeredAt = 0;
		};

		// Every landing button in the building, indexed by callIndex(): two per
		// floor, fixed size, never grown. A 20-floor building has at most 40
		// registered calls whether 3 or 3,000 people are waiting, and the
		// dispatcher is consulted per call, not per person. It is also what keeps
		// the simulation deterministic: a vector indexed by floor has no
		// iteration order to leak, which a hash map would.
		vector<LandingCall> landingCalls;

		// Indices into `landingCalls` of the calls that are registered and
		// allocated to nobody, in registration order, so the oldest unanswered
		// call is retried first and cannot be starved by newer ones. Retried at
		// the top of every giveEvent, which is enough by construction: nothing
		// happens between two events in a discrete-event simulation, so there
		// is no instant at which a cabin frees up and a waiting call does not
		// get its chance.
		vector<int> unallocatedCalls;

		// Liveness for a PREFERENCE refusal. When a
		// passenger declines a car they physically fit into (an injected
		// IPassengerBehaviorStrategy said no), the cabin that refused them must
		// not be handed the call straight back, or at a zero dwell the
		// allocate/refuse/allocate cycle never advances the clock. The RECALL
		// branch excludes that cabin for the immediate re-call; this map carries
		// the same exclusion for a call that could only be pooled (no other cabin
		// could take it), keyed by call index -> the cabin that refused. It is
		// honoured by retryUnallocatedCalls and dropped as soon as that cabin
		// leaves the refusal floor - once it has gone it will come back emptier,
		// so re-offering to it is correct and excluding it forever would strand a
		// call in a single-cabin building (Barney & Al-Sharif 11.6.3: refusing
		// allocation is to be avoided). Empty unless a declining policy is
		// injected, so an un-configured run never touches it.
		std::map<int, int> preferenceExclusions;

		// The ARRIVE giveEvent is processing right now, as (elevator, stop), or
		// (0, 0) outside one. ensureArrivalEvent asks the queue whether a stop
		// already has its ARRIVE, and the one in hand is not in the queue any
		// more: without this, a pooled call re-offered at the top of giveEvent
		// and merged into the very stop that is arriving would get a second
		// ARRIVE, firing at the same instant with the stop already gone. Not a
		// capacity-only path: releaseUnservedCalls pools the other sweep's queue
		// at a reversal stop even with unlimited cabins.
		int arrivalInHandElevator;
		unsigned long int arrivalInHandStop;

		// Which entry of `landingCalls` a (floor, direction) is. `direction`
		// must be UP or DOWN; the floor is bounds-checked by the caller's use
		// of vector::at, so a customer outside the building throws rather than
		// corrupting a neighbouring call.
		int callIndex(int floor, unsigned int direction) const;

		// A leg a cabin is currently flying, remembered so that it can be
		// priced on ARRIVAL, with the distance it actually covered, instead of
		// on departure with the distance it intended to cover. Those are not the
		// same number: a nearer call inserted mid-flight re-anchors the route,
		// the cabin stops early, and the leg it would have been charged for
		// never happened.
		struct LegInFlight {
			bool active = false;
			double originFloor = 0;		// where the leg started
			double load = 0;			// cabin load at departure, which is the load it carries
			double departureTime = 0;	// reported as the trip's instant, as it always was
		};
		vector<LegInFlight> legsInFlight;	// indexed by elevator id - 1

		// Records that `elevatorId` has just left `originFloor`. Emits the
		// DEPART event for the snapshot straight away - that one really is
		// about the departure instant, and a trajectory reader interpolates from it
		// - but holds the energy charge until the leg completes.
		void beginLeg(int elevatorId, double originFloor, double load, double departureTime);
		// Prices the leg in flight, now that `arrivalFloor` makes its distance
		// a fact, and hands it to IMetricsCollector::onElevatorDeparted. A
		// no-op when nothing is in flight, which is the case for a cabin that
		// was parked.
		//
		// Called from beginLeg and from flushLegsInFlight, and from nowhere
		// else on purpose: the leg that was in flight ended exactly where the
		// next one starts, so those two are the only places that know an
		// arrival floor without having to reason about which event of a stop
		// fired. Closing on arrival EVENTS instead looks equivalent and is not
		// - a merged stop can fire several of them at one instant, and the
		// second would close the leg the first had just opened, booking a
		// zero-distance charge that never happened.
		void completeLeg(int elevatorId, double arrivalFloor);
		// Closes whatever is still in flight when the run ends, using each
		// cabin's final position. Without it the last leg of every elevator
		// would go uncharged, since nothing arrives after it.
		void flushLegsInFlight();

		// Makes sure route stop `stopId` of `elevatorId` has exactly one ARRIVE
		// event waiting for it, timed from the route as it is right now.
		//
		// Called after every real route insertion, and it is the ONLY place an
		// ARRIVE is created - which is what makes "one event per stop" a
		// property of the code rather than a rule to remember. The test for
		// "this stop already has one" is the event queue itself
		// (EventQueue::hasPendingForStop), so no second bookkeeping exists to
		// disagree with it: a brand-new route entry has no pending event and
		// gets one, a task merged into an existing entry finds that entry's
		// ARRIVE and adds nothing.
		//
		// Does not re-time anything: the caller owns that, because it is the one
		// that knows whether the insertion moved the rest of the route
		// (retimePendingEvents).
		void ensureArrivalEvent(int elevatorId, unsigned long int stopId);
		// Records one passenger transfer for the caller to drain - see the
		// Transfer struct. `cus` is only read.
		void recordTransfer(unsigned int type, int elevatorId, int floor, const Customer& cus, double time);
		// Which way the cabin tells the landing it is going, at the stop it has
		// just reached - and therefore which of the two queues at that floor
		// gets in. This is what makes the engine enforce Closs rule 3 (nobody
		// boards a car going against their direction), under a route policy
		// that keeps sweeping.
		//
		// The sources are asked in order, and the ORDER is the decision:
		//
		// **1. The landing call this stop was sent for, if it goes the way the
		// cabin is going.** A landing call has a direction and the dispatcher
		// already promised it a car, so a car that stops for an up-call
		// announces up. That is directional collective control, and reading the
		// route first does not terminate: a cabin standing at floor 7 for an
		// up-call whose route also holds a down-call at 3 would announce DOWN,
		// board nobody, give the up-call back to the pool, be handed it again,
		// and oscillate between the two floors for ever. The up-call at 7 is
		// served GOING UP by definition, and the car call of whoever boards there
		// is what extends the sweep - information the route cannot have before
		// the boarding.
		//
		// **2. The route** (Elevator::announcedDirectionAt), for the stop a
		// cabin makes for somebody inside it, which has no landing call of its
		// own: the people waiting there board if the cabin continues their way.
		//
		// **3. A landing call of this stop in any direction** (end of sweep).
		// When both buttons at that floor are lit and allocated to this stop (a
		// sweep reversal merged into one entry), the older registration wins -
		// the same "oldest call first" rule the pool of unallocated calls is
		// retried in.
		//
		// **4. The oldest call at this floor that has somebody waiting, whatever
		// cabin it is allocated to**, for a cabin that arrived on a car call
		// alone, emptied, and has nowhere to go. Without this step the people
		// there would wait for the cabin their call was promised to while an
		// empty one stood at their floor.
		//
		// IDLE is the answer only when nobody is waiting there at all, which is
		// a cabin that parks.
		unsigned int boardingDirectionAt(int elevatorId, unsigned long int stopId, int floor) const;
		// Everybody waiting at `floor` to go `direction`, in arrival order: the
		// queue of that landing call, whatever cabin it was allocated to.
		//
		// Boarding is decided at the door, not at allocation: a cabin that stops
		// at your floor going your way takes you whether or not it is the one
		// the dispatcher sent. Peters cancels a landing call in
		// `CancelLandingCalls(lift l[])` - plural, the car that ARRIVES cancels
		// it (section 8.3.5) - and his passengers are updated against the whole
		// lift array (section 8.3.6), so the person boards whichever car is
		// there going their way.
		//
		// A copy, not a reference: boarding empties the queue as it goes.
		vector<Customer*> waitersToBoardAt(int floor, unsigned int direction) const;
		// Gives back every landing call that is allocated to this stop and is
		// NOT going to be served by it - the other sweep's queue at a reversal
		// stop, which stays on the landing because the cabin is about to go the
		// other way.
		//
		// Two things, and both are needed. The call's allocation is released and
		// it goes back in the pool, because the stop it was promised is about to
		// leave the route and a call pointing at a stop that no longer exists is
		// a passenger nobody will come back for. And its hall calls come off the
		// stop, so the dwell is not priced for transfers that are not going to
		// happen - the route entry itself survives, because
		// Elevator::releaseHallCall never drops the stop a cabin is standing at.
		void releaseUnservedCalls(int elevatorId, unsigned long int stopId, int floor,
			unsigned int servedDirection);
		// The cabin that had been promised `call` is not the one serving it, so
		// take the stop away from it: `boarded` of its pickups have just got
		// into somebody else's car, so that many hall calls come off its route
		// entry, and if that leaves the entry serving nobody it is dropped and
		// its ARRIVE comes out of the timeline with it.
		//
		// This is Rule 1 of the five rules of lift traffic control - no stop
		// where nobody boards or alights - and `lift::RemoveLandingCall` in
		// Peters' own simulator (section 8.3.4), which is described as "called
		// by class when lift arrives at landing". Without it, every boarding into
		// a cabin other than the one the call was allocated to would leave a
		// phantom stop in that cabin's schedule.
		// `stopId` is passed in rather than read from the call, because the
		// caller has to capture it BEFORE the boarding: emptying a call's queue
		// clears the call, stopId included.
		void withdrawStopFrom(int elevatorId, unsigned long int stopId, int call, int boarded);
		// Re-times every pending ARRIVE of the elevator from its current route
		// (Elevator::getStopEventTime). Called after every route change made by
		// giveEvent: a stop's timestamp is derived from the live route, never
		// stored and patched.
		void retimePendingEvents(int elevatorId);
		// Runs Elevator::processNextTask on `elevator` and reports a departure
		// (to the IMetricsCollector and to `pendingDepartures`) only if that
		// call actually started a NEW leg. A departure is a property of the
		// *stop*, and a stop can serve several transfers: reporting once per
		// transfer would charge a leg that is travelled once several times and
		// emit duplicate DEPART records.
		// An elevator that went IDLE reports nothing here - the addTask() of its
		// next task reports that fresh departure instead.
		// `finishedStopId` is the stop being served, handed through to
		// Elevator::processNextTask so the route entry removed is that stop and
		// not merely whatever sits at the front near the cabin.
		// Returns true when the stop's dwell pushed the departure past `time`,
		// i.e. when every stop after it in the route moved and the elevator's
		// pending events must be re-timed. False means the later stops are
		// provably unchanged, and re-timing them anyway would not be free:
		// the same route sum re-derived further along a leg need not land on the
		// same bits in floating point, and EventQueue's (time, seq) tie-break
		// turns a last-bit difference into a different simulation. So this
		// condition is mandatory for determinism.
		// `holdLegDecision` comes from the caller's loop over the stop's
		// transfers, which is what knows whether more of them are still to come
		// (see Elevator::processNextTask).
		bool advanceAndReportDeparture(Elevator& elevator, double time, unsigned long int finishedStopId,
			bool holdLegDecision);

		// Puts `cus` in the queue of the landing call for their floor and
		// direction, registering that call (and consulting the dispatcher) if it
		// is not lit yet.
		//
		// This is the whole of a hall call's registration, in one place,
		// because it happens in three situations: a passenger arrives, a
		// passenger who was left behind presses again (RECALL), and the pool
		// of unanswered calls is retried. What it deliberately does NOT do is
		// the arrival bookkeeping (onCustomerArrived, allCustomers): that
		// belongs to the arrival alone, and counting it again would restart a
		// waiting time that has to keep running from the first press.
		//
		// The dispatcher is consulted at most ONCE per call, not once per
		// person: somebody joining a call that already has a cabin coming is
		// attached to that cabin's existing stop - Elevator::joinHallCall -
		// without any dispatch decision, any re-sorting, or any new route entry.
		// That is what a landing button does.
		//
		// `excludeElevator` (1-indexed, 0 = exclude nobody) is a cabin this
		// call may NOT be given to, however good the dispatcher thinks it is.
		// It exists for the cabin that has just refused somebody at its open
		// door: handing the re-call straight back to it is both physically
		// silly - the person watched that car leave without them - and the shape
		// of a livelock, since the cabin is standing right there. The capacity
		// veto already covers a refusal that happened because the cabin was
		// full; this covers a refusal for any OTHER reason, which is what
		// IPassengerBehaviorStrategy makes possible.
		//
		// Creates no event of its own: the stop's ARRIVE is created by
		// ensureArrivalEvent when the stop enters a route, and there is at most
		// one however many people are behind the call.
		void joinOrRegisterHallCall(Customer* cus, double now, int excludeElevator = 0);
		// Offers one registered call to the IDispatchStrategy and, if a cabin
		// can answer it, puts the stop in that cabin's route - with one
		// boarding counted for every waiter, so the dwell is priced for the
		// whole queue - and makes sure that stop has its ARRIVE. When no cabin
		// could take it the call keeps its queue, `allocatedElevator` stays 0 -
		// which is how the caller tells - and the caller must park it in
		// `unallocatedCalls` (deferCall).
		//
		// The veto (Elevator::canAnswerHallCall) is asked with the weight of
		// the FIRST waiter in the queue, because that is the person who would
		// board first: a queue is served in order, so a cabin that cannot take
		// the head of it cannot serve this call at all. That also makes the
		// answer independent of how the queue is stored, which matters because
		// the engine must not depend on which of several waiting people it
		// happens to look at.
		void allocateCall(int call, double now, int excludeElevator);
		// Parks a call no cabin could answer. Throws std::logic_error if
		// nothing is on board anywhere, because then no future state can answer
		// it either and the run would hang: either the passenger is heavier
		// than the cabin's rated load, or the injected IDispatchStrategy
		// declines to allocate, or an injected IPassengerBehaviorStrategy
		// declines every cabin.
		void deferCall(int call);
		// Re-offers every registered-but-unallocated call, oldest first, and
		// puts the stop of each one that now finds a cabin into that cabin's
		// route. Called at the top of giveEvent - see `unallocatedCalls` for
		// why that is the only place it needs to be.
		void retryUnallocatedCalls(double now);
		// Offers the already-allocated, non-committed landing calls to the group
		// dispatcher and applies whatever reassignment it asks for. Called at the
		// top of giveEvent, right after retryUnallocatedCalls, so the picture it
		// reads is settled and the route is final before this event's timestamps
		// are derived from it; and again at the end of the CUST_CALL and RECALL
		// branches, so a group strategy sees the whole allocated set at the
		// instant a new call arrives.
		//
		// The default IGroupDispatchStrategy (ImmediateGroupAdapter) returns each
		// call's current cabin, so nothing moves. The pool is NOT part of this -
		// retryUnallocatedCalls owns it. Committed calls are left out of the
		// offer: their stop is the front of a cabin's route and cannot be
		// released (a braking cabin cannot un-brake).
		//
		// Reassignments are applied in callIndex order (the order
		// snapshotPendingCalls yields), not in the order the strategy returned,
		// so the sequence is deterministic whatever a metaheuristic hands back.
		void reoptimizeAllocations(double now);
		// Takes `cus` out of their landing call's queue, because they have just
		// boarded. Clears the call entirely when that empties it: the button
		// goes out, and the next person to arrive at that floor and direction
		// lights a fresh one (Peters section 8.3.5 cancels a landing call when
		// a car arrives).
		void clearBoardedWaiter(Customer* cus);
		// The stop `stopId` of `elevatorId` is being served right now, so if
		// that is the stop this passenger's landing call was allocated to, the
		// allocation is over - whatever happened to the people on it. A no-op
		// when the cabin is at that floor for some other reason, which is why
		// it checks rather than assumes.
		void endAllocationAt(Customer* cus, int elevatorId, unsigned long int stopId);
		// Makes sure a call with somebody waiting on it and no cabin coming is
		// in the pool to be re-offered, exactly once. The invariant it defends
		// is the one whose violation ends a run: a call with somebody in its
		// queue, no cabin allocated and no place in the pool is a passenger
		// nothing will ever come back for.
		void ensurePooled(int call);
		// The cabin serving `call` has reached its stop (or given it up), so
		// the allocation is over whatever happened to the people on it. The
		// queue is NOT touched: whoever did not board is still waiting, and
		// their RECALL - or the pool - is what gets them a cabin again.
		void releaseAllocation(int call);
		// Is there any cabin at all that could answer a call for a passenger of
		// this weight? Asked before consulting the IDispatchStrategy, which is
		// the expensive part (it copies the whole elevator vector), and
		// outcome-identical by construction: when the answer is no, every
		// choice the strategy could make would be vetoed anyway.
		bool anyCabinCouldAnswer(double weight) const;
		// The lowest-id cabin (1-based) that can answer a call for a passenger of
		// this weight and is not `excludeElevator`, or 0 if none. A LIVENESS floor,
		// not a routing decision: allocateCall uses it only when the dispatcher
		// picks a cabin the capacity veto refuses, to redirect the call to a cabin
		// that can take it instead of pooling it, where a metaheuristic that keeps
		// naming the full cabin would strand it forever.
		int firstAnswerableCabin(double weight, int excludeElevator) const;
		// Load bypass: if `elevatorId` has filled past its load-weighing
		// threshold, takes back every pickup it has not made yet and returns
		// those passengers to `unallocatedCalls`, so the cabin drives past their
		// floors instead of stopping where nobody can board. One stop is exempt -
		// the one it is already committed to (Elevator::committedStopId) -
		// because a cabin in flight cannot un-brake; the people there are refused
		// at the open door when it arrives instead. A no-op for a cabin that is
		// not bypassing, which is every cabin of a run that configures no
		// capacity.
		void bypassPendingHallCalls(int elevatorId, double now);
		// The one predicate behind wouldRefuseBoarding and decideBoardings: would
		// this passenger refuse, or not fit, in this cabin carrying `load`? The
		// load is an argument because decideBoardings asks about boardings that
		// have not happened yet.
		bool refusesAtLoad(int elevatorId, double load, const Customer& cus, int floor, double time) const;
		// A cabin arrived at this passenger's floor, opened its doors and they
		// did not get in: counted in the metrics. The snapshot's own record of
		// it is the PASS row written in place of the ENTER that did not happen -
		// see wouldRefuseBoarding.
		//
		// The call does not go back to the pool here: the passenger presses the
		// button again, at an instant of their own - scheduleRecall - which is
		// both what a person does and what stops the engine re-offering the same
		// call at every event until it sticks.
		void recordRefusalAtDoor(Customer* cus, int elevatorId, double time);
		// Puts a RECALL event in the queue for a passenger who has just been
		// left behind by `elevatorId`: the instant comes from
		// IPassengerBehaviorStrategy::nextCallTime, clamped so it can never be
		// in the past, and the event carries `elevatorId` so the new call is
		// not handed back to the cabin that refused them.
		//
		// Must be called AFTER the cabin's next leg has been decided
		// (advanceAndReportDeparture), because the default policy answers with
		// the instant that cabin departs and that is only known once the leg
		// exists. Called with the cabin's own departure instant computed by the
		// caller for exactly that reason.
		void scheduleRecall(Customer* cus, int elevatorId, int floor, double now, double cabinDeparture);
		// The cabin that had been allocated this passenger's call filled up
		// elsewhere and gave the call back. Counted and re-offered, but
		// deliberately NOT a timeline event: nothing happens at the
		// passenger's floor, no car arrives, and emitting one would both
		// mislead a reader and, under saturation, bury the timeline in rows that
		// outnumber the passengers by orders of magnitude - see PassByReport.
		void recordReallocation(Customer* cus, int elevatorId, double time);
		// Puts a passenger's registered landing call back in the pool for the
		// dispatcher. Both of the above end here, and that is the point: a call
		// taken away from a cabin and not returned is a passenger dropped from
		// the simulation.
		void returnToDispatch(Customer* cus);

	public:
		
		EventQueue upcomingEvents;
		
		// `dispatch_strategy` MUST be provided by the caller (there is no
		// default dispatch algorithm bundled with the core library - see
		// examples/greedy_nearest_dispatch/GreedyNearestDispatchStrategy
		// for a ready-to-use example). Throws std::invalid_argument if null.
		// `max_weight`/`capacity_strategy` are applied identically to every
		// elevator (all elevators share the same capacity); default
		// `max_weight` is unlimited. `dwell_strategy` is likewise one shared
		// instance handed to every elevator - "all cabins have the same doors"
		// holds by construction, not by coincidence - and defaults to the
		// handbook's centre-opening door with normal opening, the office
		// transfer time and the doors held open 2.0 s (car call) / 3.0 s
		// (landing call), its width picked by `max_weight` (DwellPresets.h).
		// Inject makeDwellStrategy(DwellPreset::instantaneous(), ...) for a stop
		// that takes no time.
		// `travel_strategy` is the third policy shared the same way, and for
		// the strongest version of the same reason: the travel model is a
		// property of the installation, so two cabins in one shaft group
		// cannot disagree about how long a floor takes. It defaults to
		// a KinematicTravelTimeStrategy over a 3.5 m floor height and the
		// dynamics defaultDynamicsFor() derives from `no_floors`. Inject a
		// LinearTravelTimeStrategy for the constant-speed model, and inject a
		// kinematic one built from your own <Floor height> if it is not 3.5 m.
		// `speed` does not reach the default travel model at all: it is applied
		// only by the linear model.
		// `passenger_behavior` is the fourth policy and the only one that is
		// not a property of the building: it answers what the *passenger*
		// decides, and defaults to StandardPassengerBehavior, which boards
		// every car it fits into.
		// `random_provider` is the shared IRandomProvider the group dispatcher
		// draws from; defaults to a fresh RandomProvider when null, which the
		// default adapter never draws from, so determinism is unaffected. The
		// Simulator passes its own, so a GA and the arrivals share one seeded
		// source. `group_dispatch_strategy` is the group dispatch seam; defaults
		// to an ImmediateGroupAdapter wrapping `dispatch_strategy`.
		// `scheduling_strategy` is how every cabin orders its own route
		// (IElevatorSchedulingStrategy), shared by the whole group like the
		// three building policies above: one instance, so "all cabins follow
		// the same route policy" holds by construction. Defaults to a
		// LookSchedulingStrategy. The dispatcher's what-if copies
		// (FindBestElevator) carry the same instance, so a dispatch cost
		// estimate is computed under the route policy the cabin really follows.
		ElevatorSystem(int no_elevators = 1, int no_floors = 1, double speed = 1,
			shared_ptr<IDispatchStrategy> dispatch_strategy = nullptr,
			shared_ptr<IMetricsCollector> metrics_collector = nullptr,
			double max_weight = std::numeric_limits<double>::infinity(),
			shared_ptr<ICapacityStrategy> capacity_strategy = nullptr,
			shared_ptr<IDwellTimeStrategy> dwell_strategy = nullptr,
			shared_ptr<ITravelTimeStrategy> travel_strategy = nullptr,
			shared_ptr<IPassengerBehaviorStrategy> passenger_behavior = nullptr,
			shared_ptr<IRandomProvider> random_provider = nullptr,
			shared_ptr<IGroupDispatchStrategy> group_dispatch_strategy = nullptr,
			shared_ptr<IElevatorSchedulingStrategy> scheduling_strategy = nullptr);		// Default constructor
		
		int FindBestElevator(Task task);										// Delegates to the injected IDispatchStrategy, on a scratch copy of the elevators (the strategy cannot modify the real ones)
		void showSystem();														// Shows the status of all elevators
		Task& giveTask(int e, Task t);											// Gives a task to a particular elevator
		void doTask(int e);														// Processes the task of a particular elevator
		void update(Timer t);													// Updates the current time for all elevators
		// Reacts to one event: the state machine of the whole simulation. One
		// branch per event type (CUST_CALL / RECALL / ARRIVE), mutating the
		// relevant Elevator and reporting to the injected
		// IDispatchStrategy/IMetricsCollector.
		//
		// Returns nothing: every event this creates (the ARRIVE of a stop, a
		// RECALL) is pushed onto `upcomingEvents` by the engine itself, because
		// a stop's ARRIVE is created when the stop enters a route, which may
		// happen deep inside the handling of a different event.
		void giveEvent(Event ev);
		void addCustomer(int floor, Customer cus);								// Adds a customer to the elevator system
		void removeCustomer(int floor, Customer cus);							// Removes a customer from the elevator system
		void printMetrics();													// Prints the performance metrics (via injected IMetricsCollector)
		shared_ptr<IMetricsCollector> getMetricsCollector() const;				// Exposes the collector (e.g. for the simulation snapshot)
		const vector<Elevator>& getElevators() const;							// Read-only access to the elevators (e.g. for tests that check cabin state)
		// The travel model every cabin in this group shares: how long a leg
		// takes and where the cabin is partway through it. Exposed because the
		// snapshot has to record which model produced its timestamps and
		// because a caller comparing two models needs to know which one a run
		// got. Never null.
		const shared_ptr<ITravelTimeStrategy>& getTravelTimeStrategy() const;
		// The dwell every cabin in this group shares, AFTER the default has been
		// resolved for the rated load: the snapshot records it, and reading it
		// before resolution would record the wrong door for cabins up to
		// 1000 kg. Never null.
		const shared_ptr<IDwellTimeStrategy>& getDwellTimeStrategy() const;
		// The shared IRandomProvider this system resolved (the injected one, or
		// the fallback it built). Exposed so the Simulator can draw its arrivals
		// from the very same source the group dispatcher does - one seeded
		// stream for the whole run. Never null.
		const shared_ptr<IRandomProvider>& getRandomProvider() const;
		vector<Departure> drainDepartures();									// Returns and clears the departures recorded since the last call (see giveEvent)
		// Returns and clears the passenger transfers recorded since the last
		// call: the boardings, alightings and door-refusals of the stop just
		// served, in the order they happened.
		//
		// The caller is what turns them into timeline rows, the same division of
		// labour drainDepartures() has - this class does not know an
		// ISimulationLogger exists. A caller that ignores them loses the whole
		// per-passenger half of the timeline, which is why Simulator drains
		// both in the same place.
		vector<Transfer> drainTransfers();
		// Would this boarding fail - because the cabin has no room for that
		// passenger, or because the passenger declines it
		// (IPassengerBehaviorStrategy::willBoard)? Asked by the ARRIVE branch of
		// giveEvent, with the cabin standing at the floor and its doors open,
		// which is the only moment the question has an answer. The same branch
		// writes the timeline row, so the snapshot and the behaviour cannot
		// disagree.
		//
		// Both reasons produce the same PASS row: the timeline records that the
		// passenger did not get in, not why.
		//
		// Const, and deliberately independent of Elevator::update: a cabin's
		// load does not change with its position, so the answer is the same
		// before and after the clock is advanced - which is what makes it safe
		// to ask one line earlier than the branch does.
		bool wouldRefuseBoarding(int elevatorId, const Customer& cus, int floor, double time) const;
		// Who of `candidates` gets in at this stop, in queue order, each person
		// asked at the load they would really find after the ones ahead of them
		// boarded. True for each one refused. It has to be known before the first
		// transfer, because the stop's dwell is charged for the transfers that
		// really happen. Same predicate as wouldRefuseBoarding, so there is one
		// place that decides, and it asks a passenger policy exactly once per
		// candidate, in the order the boardings happen.
		vector<bool> decideBoardings(int elevatorId, const vector<Customer*>& candidates, int floor, double time) const;
		// How many registered landing calls are allocated to nobody right now:
		// the people waiting at a floor that no cabin is on its way to AND
		// whose call is sitting in the dispatcher's pool. Zero for the whole of
		// any run that configures no capacity.
		//
		// It counts CALLS, so it is bounded by two per floor however bad the
		// congestion gets. Use getUnallocatedPassengerCount() for "how much
		// demand is not being served": in a saturated run the two can differ by
		// orders of magnitude.
		int getUnallocatedCallCount() const;
		// How many PEOPLE are waiting at a landing with no cabin on its way to
		// them - the queues of every registered call that is allocated to
		// nobody. Wider than getUnallocatedCallCount() in two ways: it counts
		// people rather than buttons, and it includes the ones whose call is
		// not in the dispatcher's pool because they have just been refused at a
		// door and hold a pending RECALL instead.
		//
		// This is "demand the system has not managed to serve", the number that
		// tells a saturated run from a busy one. Zero for the whole of any run
		// that configures no capacity.
		int getUnallocatedPassengerCount() const;
	// The whole pending demand as a group dispatcher sees it: one HallCall per
	// *registered* landing call (one with somebody waiting), tagged with the
	// cabin it is allocated to now (`currentElevatorId`, 0 = the pool) and
	// whether it is `committed`. A call is committed when its stop is the FRONT
	// of that cabin's route (Elevator::committedStopId()) - the stop the cabin
	// is heading to and that Elevator::releaseHallCall refuses to drop, so it
	// must never be reassigned (a braking cabin cannot un-brake). Walked in
	// callIndex order, so no iteration order leaks into a result a GA/PSO would
	// consume. Const and side-effect-free: it is a snapshot, and the
	// reassignment it feeds is applied by the engine afterwards. A public
	// observer in the same spirit as getUnallocatedCallCount().
	std::vector<HallCall> snapshotPendingCalls() const;
	// Moves the registered landing call for (floor, direction) from the cabin it
	// is allocated to now to cabin `to`, as a unit with its whole waiting queue.
	// Returns true when the move happened. A no-op returning false when any
	// precondition fails - it is a suggestion a group dispatcher makes, not an
	// order:
	//   - the call is not registered, is pooled (allocated to nobody), is
	//     already on `to`, or `to` is out of 1..N;
	//   - the call is COMMITTED: its stop is the FRONT of its cabin's route,
	//     which Elevator::releaseHallCall refuses to drop, so moving it would
	//     leave a phantom stop in the old cabin and double-serve - a braking
	//     cabin cannot un-brake;
	//   - `to` cannot answer it (bypassing or no room - the capacity veto, asked
	//     with the head of the queue's weight, exactly as allocateCall does).
	// On success it revokes from the old cabin with releaseHallCall semantics
	// (withdrawStopFrom: drop the stop only if it now serves nobody) and inserts
	// into `to` exactly as allocateCall does, re-timing both cabins. It does NOT
	// board anyone - it only changes whose promise the call is. Exposed (like
	// FindBestElevator) so a test can drive one move directly; the engine calls
	// it from reoptimizeAllocations.
	bool reassign(int floor, unsigned int direction, int to, double now);
};

} // namespace openEGCS

#endif
