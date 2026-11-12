/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : ElevatorSystem.cpp

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
#include<cstdlib>
#include<cmath>
#include<memory>
#include<stdexcept>

#include "ElevatorSystem.h"
#include "StandardMetricsCollector.h"
#include "PercentageCapacityStrategy.h"
#include "FixedDwellTimeStrategy.h"
#include "DwellPresets.h"
#include "StandardPassengerBehavior.h"
#include "ImmediateGroupAdapter.h"
#include "RandomProvider.h"

namespace openEGCS {

using namespace std;

ElevatorSystem::ElevatorSystem(int no_elevators, int no_floors, double speed,
	shared_ptr<IDispatchStrategy> dispatch_strategy,
	shared_ptr<IMetricsCollector> metrics_collector,
	double max_weight,
	shared_ptr<ICapacityStrategy> capacity_strategy,
	shared_ptr<IDwellTimeStrategy> dwell_strategy,
	shared_ptr<ITravelTimeStrategy> travel_strategy,
	shared_ptr<IPassengerBehaviorStrategy> passenger_behavior,
	shared_ptr<IRandomProvider> random_provider,
	shared_ptr<IGroupDispatchStrategy> group_dispatch_strategy,
	shared_ptr<IElevatorSchedulingStrategy> scheduling_strategy)
{
	number = no_elevators;
	floors = no_floors;
	elevators.clear();
	// A single shared ICapacityStrategy instance is handed to every
	// elevator, so "all elevators have the same capacity policy" holds by
	// construction, not just by coincidence of using the same default class.
	shared_ptr<ICapacityStrategy> sharedCapacityStrategy =
		capacity_strategy ? capacity_strategy : make_shared<PercentageCapacityStrategy>();
	// Same reasoning for the stop-duration policy: one instance for the whole
	// group, so every cabin's doors behave alike by construction.
	//
	// The default is the handbook's centre-opening door with normal opening,
	// the office transfer time (DwellPresets.h) and the doors held open 2.0 s
	// for a car call / 3.0 s for a landing call. The default lives here and not
	// in Elevator for the same reason the kinematic travel default does
	// (below): it needs a property of the installation - the rated load, which
	// picks the door width - and a bare Elevator keeps the zero dwell of a
	// component built for a unit test.
	shared_ptr<IDwellTimeStrategy> sharedDwellStrategy =
		dwell_strategy ? dwell_strategy : makeDwellStrategy(DwellPreset{}, max_weight);
	// And for the travel model, where sharing is not just tidiness: cabins in
	// one group run in the same building, so they cannot disagree about what a
	// floor costs.
	//
	// The default is the KINEMATIC model, not the linear one, because the
	// linear one charges the same for nine one-floor hops as for one
	// nine-floor run, so the clock cannot see the cost of stopping while the
	// energy model can.
	//
	// This is also where the kinematic default has to live, rather than in
	// Elevator: the profile needs the installation's rated dynamics, which
	// defaultDynamicsFor() derives from how far the lift travels - and the
	// floor count is known here and not there. An Elevator built directly
	// still falls back to the linear model for exactly that reason; see its
	// constructor. `speed` is consequently unused by the default travel model.
	//
	// The 3.5 m floor height is the project's documented default, the same one
	// StandardMetricsCollector prices energy with. A caller configuring
	// <Floor height> must inject a strategy built from that value - main.cpp
	// does, from the same LiftDynamics it hands the collector, so the time and
	// the energy of a run cannot describe different journeys.
	shared_ptr<ITravelTimeStrategy> sharedTravelStrategy = travel_strategy
		? travel_strategy
		: make_shared<KinematicTravelTimeStrategy>(3.5,
			defaultDynamicsFor((no_floors - 1) * 3.5));
	// And for the route policy: one instance shared by the group. LOOK keeps
	// no state - everything it decides on comes in as arguments - so sharing it
	// is the same as one per cabin.
	shared_ptr<IElevatorSchedulingStrategy> sharedSchedulingStrategy = scheduling_strategy
		? scheduling_strategy
		: make_shared<LookSchedulingStrategy>();
	for (int i=1 ; i<=no_elevators ; i++)
	{
		elevators.push_back(Elevator(i, "", 0, speed, sharedSchedulingStrategy, max_weight,
			sharedCapacityStrategy, sharedDwellStrategy, sharedTravelStrategy));
	}
	allCustomers.resize(no_floors + 1);
	legsInFlight.assign(no_elevators, LegInFlight());
	arrivalInHandElevator = 0;
	arrivalInHandStop = 0;
	// Two buttons per floor, for floors 0..no_floors inclusive - the same range
	// CustomerFactory draws arrivals from. Fixed size for the whole run: the
	// number of landing calls a building can have is a property of the
	// building, not of how many people are in it.
	landingCalls.assign(2 * (no_floors + 1), LandingCall());

	if (!dispatch_strategy) {
		throw std::invalid_argument(
			"ElevatorSystem requires a non-null IDispatchStrategy. "
			"There is no default dispatch algorithm bundled with the core "
			"library anymore - see examples/greedy_nearest_dispatch for a "
			"ready-to-use example to inject.");
	}
	dispatchStrategy = dispatch_strategy;
	// The shared randomness the group dispatcher draws from. A fallback is built
	// when none is injected; the Simulator injects its own so arrivals and a GA
	// share one seeded stream. The default adapter draws nothing, so a run that
	// configures no group strategy is unaffected either way.
	rng = random_provider ? random_provider : make_shared<RandomProvider>();
	// The group dispatch seam. Default: wrap the per-call strategy in an
	// ImmediateGroupAdapter, which delegates once per call and never
	// reallocates. A real group algorithm is opt-in by injection, like
	// GreedyNearestDispatchStrategy is for the per-call point.
	groupDispatch = group_dispatch_strategy
		? group_dispatch_strategy
		: make_shared<ImmediateGroupAdapter>(dispatchStrategy);
	metrics = metrics_collector ? metrics_collector : make_shared<StandardMetricsCollector>();
	// Not handed to the Elevators, unlike the three policies above: what a
	// passenger decides is not a property of the cabin, and the one place that
	// decision is taken is wouldRefuseBoarding, which lives here. Keeping it
	// out of Elevator is also what stops a second copy of the rule appearing
	// inside the ARRIVE branch.
	passengerBehavior = passenger_behavior ? passenger_behavior : make_shared<StandardPassengerBehavior>();
}

int ElevatorSystem::FindBestElevator(Task task)
{
	// IDispatchStrategy implementations are required to leave `elevators`
	// unmodified (what-if insertions must be undone), but the engine does not
	// trust that: a strategy that fails to undo would leave ghost tasks in
	// elevators that were never chosen, stalling them at floors no event would
	// ever serve. Handing it a scratch copy makes the contract hold by
	// construction.
	vector<Elevator> scratch = elevators;
	return dispatchStrategy->selectElevator(task, scratch);
}

shared_ptr<IMetricsCollector> ElevatorSystem::getMetricsCollector() const
{
	return metrics;
}

const shared_ptr<IRandomProvider>& ElevatorSystem::getRandomProvider() const
{
	return rng;
}

void ElevatorSystem::retimePendingEvents(int elevatorId)
{
	const Elevator& elevator = elevators.at(elevatorId - 1);
	upcomingEvents.retime(elevatorId, [&elevator](const Event& e) {
		// Only events tied to a route stop (an ARRIVE) have a time derived from
		// the route; stop 0 = not tied to one, leave untouched (CUST_CALL,
		// RECALL). A stop that is no longer in the route (its task was just
		// completed at the same instant) also returns -1.
		if (e.getStopId() == 0) return -1.0;
		return elevator.getStopEventTime(e.getStopId());
	});
}

void ElevatorSystem::beginLeg(int elevatorId, double originFloor, double load, double departureTime)
{
	// The leg that was in flight ended exactly where this one starts, so this
	// is the one place that always knows its arrival floor, without any
	// reasoning about which event of a stop fired or when. Closing on arrival
	// events instead was the first attempt and was wrong: a merged stop fires
	// several events at one instant, and the second of them closed the leg the
	// first had just opened, booking a spurious zero-distance charge.
	completeLeg(elevatorId, originFloor);
	legsInFlight.at(elevatorId - 1) = LegInFlight{true, originFloor, load, departureTime};
	// The DEPART record is about the instant the cabin leaves, so it is emitted
	// here and not held back with the charge.
	pendingDepartures.push_back({elevatorId, (int)std::lround(originFloor), departureTime});
}

void ElevatorSystem::completeLeg(int elevatorId, double arrivalFloor)
{
	LegInFlight& leg = legsInFlight.at(elevatorId - 1);
	if (!leg.active) return;	// nothing in flight: a cabin that was parked
	leg.active = false;
	// Signed, because the energy model needs the direction: cabin plus load
	// works against the counterweight one way and with it the other.
	metrics->onElevatorDeparted(elevatorId, arrivalFloor - leg.originFloor,
		leg.load, leg.departureTime);
}

void ElevatorSystem::flushLegsInFlight()
{
	for (vector<Elevator>::iterator it = elevators.begin() ; it != elevators.end() ; ++it)
	{
		completeLeg(it->getId(), it->getPosition());
	}
}

bool ElevatorSystem::advanceAndReportDeparture(Elevator& elevator, double time,
	unsigned long int finishedStopId, bool holdLegDecision)
{
	// See the contract in ElevatorSystem.h. Elevator::startedNewLeg() is what
	// tells a real departure apart from another transfer of the same stop.
	// `holdLegDecision` comes from the caller's loop over that stop's
	// transfers, which is what knows whether more are coming.
	elevator.processNextTask(finishedStopId, holdLegDecision);
	// Re-time whenever the route's next stop changed, whether or not a
	// departure is reported: pending event timestamps are derived from the
	// route, so a re-sort that moves a stop away from the front leaves that
	// stop's event holding a time it no longer deserves. See
	// Elevator::routeFrontChanged().
	bool needsRetime = elevator.routeFrontChanged();
	if (elevator.getStatus() == IDLE) return needsRetime;
	if (!elevator.startedNewLeg()) return needsRetime;
	// The cabin leaves when the dwell of the stop it just served is over, which
	// processNextTask has already recorded as the new leg's start - reporting
	// `time` here would claim it left the instant the passenger boarded.
	// Equal to `time` when the dwell is zero.
	double departureTime = elevator.getLegStartTime();
	if (departureTime < time) departureTime = time;
	// Opens the leg; it is priced when the cabin arrives, by completeLeg. The
	// destination is deliberately NOT part of the charge - it is an intention,
	// and a nearer call inserted mid-flight can make the cabin stop somewhere
	// else entirely.
	beginLeg(elevator.getId(), elevator.getPosition(), elevator.getCurrentLoad(), departureTime);
	return needsRetime || departureTime > time;
}

int ElevatorSystem::callIndex(int floor, unsigned int direction) const
{
	return floor * 2 + (direction == UP ? 0 : 1);
}

std::vector<HallCall> ElevatorSystem::snapshotPendingCalls() const
{
	std::vector<HallCall> pending;
	// In callIndex order, by construction: the loop index IS the callIndex, so
	// the result has a fixed order no matter what the hot path did
	// (determinism).
	for (int call = 0 ; call < (int)landingCalls.size() ; ++call)
	{
		const LandingCall& landing = landingCalls.at(call);
		// Only a registered call - one with somebody behind it. The vector has
		// two slots per floor for the whole run; most are empty.
		if (landing.waiting.empty()) continue;
		bool committed = false;
		if (landing.allocatedElevator != 0)
		{
			const Elevator& cabin = elevators.at(landing.allocatedElevator - 1);
			// The stop is the front of its cabin's route: the one it is heading
			// to and cannot be released from. committedStopId() is 0 for an empty
			// route, so a stray allocation with no stop reads as not committed.
			committed = (landing.stopId != 0 && landing.stopId == cabin.committedStopId());
		}
		pending.push_back(HallCall{landing.floor, landing.direction, landing.registeredAt,
			landing.allocatedElevator, committed});
	}
	return pending;
}

bool ElevatorSystem::reassign(int floor, unsigned int direction, int to, double now)
{
	if (to < 1 || to > number) return false;
	int call = callIndex(floor, direction);
	LandingCall& landing = landingCalls.at(call);
	if (landing.waiting.empty()) return false;			// not registered: nobody waiting here
	int from = landing.allocatedElevator;
	if (from == 0) return false;						// pooled, not allocated: that is allocateCall's job
	if (from == to) return false;						// already there
	// Committed: the stop is the front of `from`'s route, the one it is heading
	// to and that releaseHallCall will not drop. Revoking it would decrement the
	// counters while the entry survives, and the insert into `to` would then be
	// a second cabin promised the same call - double service. So the engine does
	// not even try (the first half of §9.5's double protection).
	const Elevator& fromCabin = elevators.at(from - 1);
	if (landing.stopId != 0 && landing.stopId == fromCabin.committedStopId()) return false;
	// The capacity veto, exactly as allocateCall asks it: the head of the queue
	// is who `to` would board first, and a cabin that cannot take them cannot
	// serve this call. Asked BEFORE any mutation, so a refused move leaves
	// everything untouched.
	Elevator& toCabin = elevators.at(to - 1);
	const Customer* head = landing.waiting.front();
	if (!toCabin.canAnswerHallCall(head->getWeight())) return false;

	// --- revoke from `from` --- the whole queue's n hall calls come off the old
	// stop; withdrawStopFrom drops the stop if that leaves it serving nobody,
	// extracts its orphaned ARRIVE, and re-times `from`. landingCalls is fixed
	// size and withdrawStopFrom does not touch this entry's queue/allocation, so
	// `landing` stays valid and we update its allocation ourselves below.
	int n = (int)landing.waiting.size();
	unsigned long int oldStop = landing.stopId;
	unsigned int dir = landing.direction;
	int callFloor = landing.floor;
	double registeredAt = landing.registeredAt;
	withdrawStopFrom(from, oldStop, call, n);

	// --- insert into `to` --- the mirror of allocateCall's second half: one
	// task for the call timestamped at the first press, the rest of the queue
	// joined onto the same stop so the dwell is priced for all n transfers, one
	// ARRIVE, a fresh leg if `to` was parked, and a re-time.
	Task request(0, "", CALL, dir, callFloor, registeredAt);
	bool wasIdle = (toCabin.getStatus() == IDLE);
	Task placed = toCabin.addTask(request, 1);
	landing.allocatedElevator = to;
	landing.stopId = placed.getStopId();
	for (int w = 1 ; w < n ; ++w)
	{
		toCabin.joinHallCall(landing.stopId, dir);
	}
	ensureArrivalEvent(to, landing.stopId);
	if (wasIdle)
	{
		double departureTime = toCabin.getLegStartTime() > now ? toCabin.getLegStartTime() : now;
		beginLeg(to, toCabin.getPosition(), toCabin.getCurrentLoad(), departureTime);
	}
	retimePendingEvents(to);
	return true;
}

bool ElevatorSystem::anyCabinCouldAnswer(double weight) const
{
	for (vector<Elevator>::const_iterator it = elevators.begin() ; it != elevators.end() ; ++it)
	{
		if (it->canAnswerHallCall(weight)) return true;
	}
	return false;
}

int ElevatorSystem::firstAnswerableCabin(double weight, int excludeElevator) const
{
	for (int id = 1 ; id <= number ; ++id)
	{
		if (id == excludeElevator) continue;
		if (elevators.at(id - 1).canAnswerHallCall(weight)) return id;
	}
	return 0;
}

void ElevatorSystem::allocateCall(int call, double now, int excludeElevator)
{
	LandingCall& landing = landingCalls.at(call);
	// Whoever is first in the queue. A queue is served in order, so this is the
	// person the cabin would have to take first, and a cabin that cannot take
	// them cannot serve this call. A call is allocated as a whole or not at
	// all, because the boarding is decided when the cabin gets there.
	if (landing.waiting.empty()) return;
	// A cabin is already on its way to it: allocating again would put a SECOND
	// stop for one lit button into a route, with the whole queue counted twice
	// in its dwell and two arrivals for one call. The path that needs this is a
	// call that is in the pool AND gets allocated by somebody else arriving at
	// that floor - the pool entry survives that, and the next retry would
	// re-offer a call a cabin is already coming for.
	if (landing.allocatedElevator != 0) return;
	const Customer* head = landing.waiting.front();

	// Before consulting the dispatcher, which is the expensive part: if no
	// cabin could answer at all, every choice it could make would be vetoed
	// below, so the answer is the same and the elevator-vector copy is saved.
	if (!anyCabinCouldAnswer(head->getWeight())) return;

	// ONE task for the call, timestamped at the first press - not at this
	// passenger's arrival and not at `now`. A call does not get younger
	// because somebody joined its queue, and the dispatch cost has to see how
	// long it has been registered.
	Task request(0 , "" , CALL , landing.direction , landing.floor , landing.registeredAt);
	// Consult the GROUP dispatcher for this one call. The default
	// ImmediateGroupAdapter delegates straight to the injected IDispatchStrategy
	// on a scratch copy of the elevators, rebuilding the Task constructed just
	// above. The call is offered as a single-element batch because a new call is
	// allocated when it is registered; the whole allocated set is offered to the
	// group afterwards, by reoptimizeAllocations. `request` itself is what the
	// engine INSERTS below once an id is chosen; the adapter only reads a
	// rebuilt copy to decide.
	HallCall pending{landing.floor, landing.direction, landing.registeredAt,
		landing.allocatedElevator, false};
	vector<int> picks = groupDispatch->allocate(vector<HallCall>{pending}, elevators, *rng);
	int x = picks.empty() ? -1 : picks.front();
	// A strategy may decline outright (anything outside 1..number), which is
	// not an error: the call stays registered and is offered again later.
	if (x < 1 || x > number) return;
	// The cabin that just refused somebody on this call does not get it back,
	// even if it is the dispatcher's choice - see the header. Checked before
	// the capacity veto only because it is cheaper; the two are independent.
	if (x == excludeElevator) return;
	vector<Elevator>::iterator it = elevators.begin() + (x - 1);
	// The engine's veto. A cabin at or above its load-weighing threshold does
	// not answer landing calls - that is the device, not a preference - and one
	// that is physically full obviously cannot. Either way the call is not
	// handed to a cabin that would have to overload itself to honour it.
	//
	// This line is load-bearing for LIVENESS, not just for tidiness, which is
	// worth knowing before anyone "simplifies" it into a dispatch-strategy
	// concern: without it the test suite stops terminating. A passenger refused
	// at a full cabin's door comes back for another allocation, the same full
	// cabin is still standing at their floor, and its ARRIVE fires at that same
	// instant - refusal, allocation, refusal, with the clock never advancing.
	// The veto is what breaks that circle: the cabin that just refused somebody
	// cannot be the cabin that is given them back.
	//
	// When the veto fires, do NOT just pool the call: we already know a cabin
	// can answer it (anyCabinCouldAnswer, above), so redirect to one instead of
	// leaving it for a later retry. The retry would re-consult the same
	// dispatcher, and a metaheuristic can keep naming the same full cabin while
	// an idle, empty one stands there - the call would then sit in the pool
	// until the queue drains. Greedy never reaches here with a vetoed pick when
	// an answerable cabin exists (it skips full cabins itself). The redirect is
	// a liveness floor, not a routing choice: lowest-id answerable cabin, and
	// the cabin that just refused this passenger (excludeElevator) stays
	// excluded.
	if (!it->canAnswerHallCall(head->getWeight()))
	{
		int redirect = firstAnswerableCabin(head->getWeight(), excludeElevator);
		if (redirect == 0) return;		// only the excluded cabin could answer: keep it pooled
		x = redirect;
		it = elevators.begin() + (x - 1);
	}

	bool wasIdle = (it->getStatus() == IDLE);
	Task placed = it->addTask(request, 1);
	landing.allocatedElevator = x;
	landing.stopId = placed.getStopId();
	// addTask counted ONE boarding passenger for the route entry it created;
	// every other waiter has to be added to the same stop, or the dwell would
	// be priced for one transfer where N happen.
	//
	// By stopId, through joinHallCall, and never by calling addTask again:
	// addTask re-sorts the route and picks an insertion point, so a second call
	// for the same floor can land on the OTHER entry for it - the one in the
	// return sweep.
	for (size_t w = 1 ; w < landing.waiting.size() ; ++w)
	{
		it->joinHallCall(landing.stopId, landing.direction);
	}
	// ONE event for the whole stop, timed from the route as it actually is
	// AFTER the real insertion (not from a dry-run beforehand) and after the
	// whole queue is on it, because each joinHallCall adds a transfer to its
	// dwell and so moves it. Everybody behind this call boards at that instant
	// in queue order - which is decided when the cabin arrives, not here.
	ensureArrivalEvent(x, landing.stopId);
	if (wasIdle)
	{
		// addTask() only sets direction/status (and starts moving) when
		// the elevator was previously IDLE - a fresh departure.
		// Not the arriving customer's own time: the cabin may have gone idle at
		// a stop whose doors are still closing, in which case the trip starts
		// when they are shut. Equal with a zero dwell.
		double departureTime = it->getLegStartTime() > now ? it->getLegStartTime() : now;
		beginLeg(x, it->getPosition(), it->getCurrentLoad(), departureTime);
	}
	retimePendingEvents(x);
}

void ElevatorSystem::joinOrRegisterHallCall(Customer* cus, double now, int excludeElevator)
{
	int index = callIndex(cus->getAt(), cus->getDir());
	LandingCall& landing = landingCalls.at(index);
	landing.floor = cus->getAt();
	landing.direction = cus->getDir();

	bool alreadyQueued = false;
	for (vector<Customer*>::const_iterator w = landing.waiting.begin() ; w != landing.waiting.end() ; ++w)
	{
		if (*w == cus) { alreadyQueued = true; break; }
	}
	if (alreadyQueued && landing.allocatedElevator != 0)
	{
		// They are in the queue of a call a cabin is already coming for, so
		// there is nothing to do and specifically nothing to add to that stop's
		// dwell - they are already counted in it. This is what makes a RECALL a
		// no-op when somebody else refused at the same stop has already pressed
		// the button and re-registered the call for the whole queue.
		return;
	}
	if (!alreadyQueued)
	{
		// The button lights up with the first person to press it, and that
		// instant is the call's, not theirs: it is what the dispatch cost sees.
		if (landing.waiting.empty()) landing.registeredAt = cus->getTime();
		landing.waiting.push_back(cus);
	}

	// The button is already lit and a cabin is on its way: join that cabin's
	// stop. No dispatch decision, no new route entry, no re-sorting. One more
	// transfer at that stop, so every stop after it in the route moves later
	// and the elevator's pending events have to be re-timed.
	if (landing.allocatedElevator != 0)
	{
		vector<Elevator>::iterator it = elevators.begin() + (landing.allocatedElevator - 1);
		if (it->joinHallCall(landing.stopId, landing.direction) != NULL)
		{
			retimePendingEvents(landing.allocatedElevator);
			return;
		}
		// The stop is gone from that cabin's route, so the allocation it
		// recorded is stale - the cabin has been there and left. Fall through
		// and treat this as a call nobody is answering, which it is.
		releaseAllocation(index);
	}

	allocateCall(index, now, excludeElevator);
	if (landingCalls.at(index).allocatedElevator != 0)
	{
		preferenceExclusions.erase(index);		// a cabin is coming: nothing to carry
		return;
	}

	// Nobody could take it. The call stays registered - "landing calls should
	// not be cancelled" (Barney & Al-Sharif 10.8.3) - and goes in the pool to
	// be re-offered, unless it is already there.
	//
	// If this came from a PREFERENCE refusal, the refusing cabin rides into the
	// pool so the next retry does not hand the call straight back to it at the
	// same instant. Told apart from a CAPACITY refusal by asking whether that
	// cabin could still answer: a full one cannot, and the capacity veto already
	// keeps it from being re-offered, so only a cabin that COULD answer but the
	// passenger declined needs this.
	if (excludeElevator >= 1 && excludeElevator <= number
		&& elevators.at(excludeElevator - 1).canAnswerHallCall(landing.waiting.front()->getWeight()))
	{
		preferenceExclusions[index] = excludeElevator;
	}
	for (vector<int>::const_iterator q = unallocatedCalls.begin() ; q != unallocatedCalls.end() ; ++q)
	{
		if (*q == index) return;
	}
	deferCall(index);
}

void ElevatorSystem::deferCall(int call)
{
	// The liveness guard. A deferred call is retried every time a cabin's load
	// drops, and a load only drops when somebody alights - so if nobody is on
	// board anywhere, no future state differs from this one and the call would
	// wait forever. That is a scenario that cannot be run, not a result, so it
	// is reported as such instead of letting the event loop drain and blame
	// "a customer was dropped".
	//
	// Asked by head count, not by weight: a cabin everybody had left could
	// still hold 1e-14 kg of rounding in its load, which would count as
	// somebody aboard and keep an unrunnable scenario running.
	bool anyoneAboard = false;
	for (vector<Elevator>::const_iterator it = elevators.begin() ; it != elevators.end() ; ++it)
	{
		if (it->getPassengerCount() > 0) { anyoneAboard = true; break; }
	}
	if (!anyoneAboard)
	{
		throw std::logic_error("ElevatorSystem: no elevator can answer a landing call and none is "
			"carrying anybody, so none ever will - either the passenger weighs more than the "
			"cabin's rated load (<Elevator max weight> vs <Mean customer weight>), or the injected "
			"IDispatchStrategy declines to allocate, or an injected "
			"IPassengerBehaviorStrategy declined the only cabin that could have taken them");
	}
	unallocatedCalls.push_back(call);
}

void ElevatorSystem::retryUnallocatedCalls(double now)
{
	if (unallocatedCalls.empty()) return;		// most events, but not every one even without a capacity configured

	vector<int> stillWaiting;
	// A copy, because allocateCall can push onto unallocatedCalls through
	// bypassPendingHallCalls... it cannot today (nothing bypasses during an
	// allocation), but iterating a vector that a callee may reallocate is the
	// kind of thing that stays correct until it does not.
	vector<int> toRetry = unallocatedCalls;
	unallocatedCalls.clear();
	for (vector<int>::iterator it = toRetry.begin() ; it != toRetry.end() ; ++it)
	{
		// A preference-refused call keeps excluding the cabin that refused it only
		// while that cabin is still standing at the refusal floor - a zero-dwell
		// re-allocation to a cabin parked there would fire a fresh ARRIVE at the
		// same instant and never advance the clock. Once the cabin has left the
		// floor it will return emptier and is a candidate again, so the exclusion is
		// dropped. Empty map for any run without a declining policy, so this is a
		// no-op lookup there.
		int exclude = 0;
		std::map<int, int>::iterator ex = preferenceExclusions.find(*it);
		if (ex != preferenceExclusions.end())
		{
			int cabin = ex->second;
			int cabinFloor = static_cast<int>(std::lround(elevators.at(cabin - 1).getPosition()));
			if (cabinFloor == landingCalls.at(*it).floor) exclude = cabin;
			else preferenceExclusions.erase(ex);
		}
		allocateCall(*it, now, exclude);
		if (landingCalls.at(*it).allocatedElevator == 0) stillWaiting.push_back(*it);
		else preferenceExclusions.erase(*it);
	}
	for (vector<int>::iterator it = stillWaiting.begin() ; it != stillWaiting.end() ; ++it)
	{
		unallocatedCalls.push_back(*it);
	}
}

void ElevatorSystem::reoptimizeAllocations(double now)
{
	// Only the already-allocated, non-committed calls are reassignable. A pooled
	// call (currentElevatorId == 0) is retryUnallocatedCalls' business; a
	// committed one is the front of a cabin's route and cannot be released.
	vector<HallCall> movable;
	for (const HallCall& c : snapshotPendingCalls())
	{
		if (c.currentElevatorId != 0 && !c.committed) movable.push_back(c);
	}
	if (movable.empty()) return;

	// The group decides where each should be. Under the default adapter every
	// answer is the call's current cabin, so the loop below moves nothing and
	// the run stays byte-identical (and the adapter draws nothing from `rng`).
	vector<int> target = groupDispatch->allocate(movable, elevators, *rng);

	// Apply in the order `movable` was built - callIndex order, by
	// snapshotPendingCalls - so the sequence of moves is deterministic whatever
	// a metaheuristic returns. reassign re-checks every precondition, so a stale
	// or illegal suggestion is a silent no-op rather than a corruption.
	for (size_t i = 0 ; i < movable.size() && i < target.size() ; ++i)
	{
		if (target[i] == movable[i].currentElevatorId) continue;	// leave it where it is
		reassign(movable[i].floor, movable[i].direction, target[i], now);
	}
}

void ElevatorSystem::ensureArrivalEvent(int elevatorId, unsigned long int stopId)
{
	// Already has one: a task merged into an existing route entry rather than
	// creating a new one, so the stop it joined is already expecting its cabin.
	// Asking the queue instead of keeping a second register of which stops have
	// events is deliberate - two registers can disagree, and this one cannot.
	if (upcomingEvents.hasPendingForStop(elevatorId, stopId)) return;
	// Nor does a stop whose ARRIVE is the event being processed: it is not in
	// the queue, it is in giveEvent's hands, and the cabin is standing at that
	// stop serving whoever just joined it.
	if (elevatorId == arrivalInHandElevator && stopId == arrivalInHandStop) return;
	const Elevator& elevator = elevators.at(elevatorId - 1);
	// getStopEventTime, not getStopArrivalTime: the doors have to be open
	// before anybody transfers, and this event is the transfers. The two
	// coincide when the dwell is zero.
	double t = elevator.getStopEventTime(stopId);
	if (t < 0)
	{
		throw std::logic_error("ElevatorSystem: a stop that needs an ARRIVE is not in the elevator's route");
	}
	int floor = 0;
	const vector<Task>& route = elevator.getRoute();
	for (vector<Task>::const_iterator r = route.begin() ; r != route.end() ; ++r)
	{
		if (r->getStopId() != stopId) continue;
		floor = r->getDest();
		break;
	}
	// No customer and no direction: an arrival is not about a person, and a
	// stop can serve both sweeps at a reversal, so there is no single direction
	// to put on it either.
	Event arrival(0 , "" , ARRIVE , DEFAULT , floor , t , NULL , elevatorId);
	arrival.setStopId(stopId);
	upcomingEvents.push(arrival);
}

void ElevatorSystem::recordTransfer(unsigned int type, int elevatorId, int floor,
	const Customer& cus, double time)
{
	pendingTransfers.push_back({type, elevatorId, floor, cus.getId(), time});
}

unsigned int ElevatorSystem::boardingDirectionAt(int elevatorId, unsigned long int stopId,
	int floor) const
{
	const Elevator& cabin = elevators.at(elevatorId - 1);
	const LandingCall& up = landingCalls.at(callIndex(floor, UP));
	const LandingCall& down = landingCalls.at(callIndex(floor, DOWN));
	bool upHere = (up.allocatedElevator == elevatorId && up.stopId == stopId && !up.waiting.empty());
	bool downHere = (down.allocatedElevator == elevatorId && down.stopId == stopId
		&& !down.waiting.empty());
	unsigned int travelling = cabin.getDirection();

	// 1. A call this stop was sent for, going the way the cabin is already
	// going. The cabin collects the calls ahead of it and reverses only
	// afterwards (Closs 1970, via Barney & Al-Sharif section 10.4), and it is
	// also the only rule that terminates: reading the route first makes a cabin
	// standing at floor 7 for an up-call announce DOWN because its route holds
	// a down-call at 3, board nobody, give the up-call back, be handed it
	// again, and oscillate 7 -> 3 -> 7 for ever with no door opening.
	// An up-call is served GOING UP by definition, and the car call of whoever
	// boards is what extends the sweep - which the route cannot know before the
	// boarding.
	if (upHere && travelling == UP) return UP;
	if (downHere && travelling == DOWN) return DOWN;

	// 2. The route: a stop the cabin makes for somebody INSIDE it, where
	// whoever is waiting can get in for free. It comes before the calls that do
	// NOT match the sweep, and that order matters: a cabin travelling down
	// through a floor whose up-call was allocated to this stop must not board
	// the up queue and carry it down (a Closs rule 4 violation).
	unsigned int fromRoute = cabin.announcedDirectionAt(stopId);
	if (fromRoute != IDLE) return fromRoute;

	// 3. Nothing ahead: the cabin is at the end of its sweep or parked, so it
	// reverses for the call it came for. Both buttons lit and nothing ahead
	// means the oldest wins - the same rule the pool of unallocated calls is
	// retried in, and the only tie-break here that needs no new number.
	if (upHere && downHere) return (up.registeredAt <= down.registeredAt) ? UP : DOWN;
	if (upHere) return UP;
	if (downHere) return DOWN;

	// 4. A cabin that arrived on a car call alone, emptied and has nowhere to
	// go: the oldest call at this floor that has somebody waiting, WHOEVER it
	// is allocated to - another cabin, or none. The dispatcher is asked once per
	// lit button, so the call is usually already promised to another cabin and
	// nobody asks again; without this step the empty cabin would stand at their
	// floor while they waited for the other one. The cabin that arrives serves
	// the landing.
	//
	// Nothing else has to change for it. waitersToBoardAt already returns the
	// queue of any allocation; the boarding loop already takes the stop away
	// from the cabin that had been promised it (withdrawStopFrom); the leg out
	// is the held leg decision of any stop where somebody alights and somebody
	// boards - the cabin is not parked yet when they board, because
	// processNextTask holds the decision while transfers are left; and
	// releaseUnservedCalls finds nothing to give back, because a stop that
	// reaches this step has no call of its own with anybody behind it, or step
	// 3 would have answered. No parking policy is being decided either: with
	// nobody waiting the answer is still IDLE, and the direction comes from a
	// call somebody actually made. Oldest first, as in step 3 and in the pool.
	bool upWaiting = !up.waiting.empty();
	bool downWaiting = !down.waiting.empty();
	if (upWaiting && downWaiting) return (up.registeredAt <= down.registeredAt) ? UP : DOWN;
	if (upWaiting) return UP;
	if (downWaiting) return DOWN;
	return IDLE;
}

vector<Customer*> ElevatorSystem::waitersToBoardAt(int floor, unsigned int direction) const
{
	if (direction != UP && direction != DOWN) return vector<Customer*>();
	// Whatever cabin the call was allocated to, including none: boarding is
	// decided at the door, not at allocation.
	return landingCalls.at(callIndex(floor, direction)).waiting;
}

void ElevatorSystem::releaseUnservedCalls(int elevatorId, unsigned long int stopId, int floor,
	unsigned int servedDirection)
{
	vector<Elevator>::iterator cabin = elevators.begin() + (elevatorId - 1);
	const unsigned int directions[2] = { UP, DOWN };
	for (int d = 0 ; d < 2 ; ++d)
	{
		if (directions[d] == servedDirection) continue;
		int index = callIndex(floor, directions[d]);
		LandingCall& landing = landingCalls.at(index);
		if (landing.allocatedElevator != elevatorId) continue;
		if (landing.stopId != stopId) continue;
		// Off the stop, once per person, so its remaining dwell is not priced
		// for transfers that will not happen. The entry stays: it is the one the
		// cabin is standing at, and releaseHallCall never drops that one.
		for (size_t w = 0 ; w < landing.waiting.size() ; ++w)
		{
			cabin->releaseHallCall(stopId, landing.direction);
		}
		releaseAllocation(index);
		ensurePooled(index);
	}
}

void ElevatorSystem::withdrawStopFrom(int elevatorId, unsigned long int stopId, int call,
	int boarded)
{
	if (elevatorId < 1 || elevatorId > number) return;
	if (stopId == 0) return;
	LandingCall& landing = landingCalls.at(call);
	vector<Elevator>::iterator cabin = elevators.begin() + (elevatorId - 1);
	for (int i = 0 ; i < boarded ; ++i)
	{
		cabin->releaseHallCall(stopId, landing.direction);
	}
	// The ARRIVE of a stop that has left the route would fire at a stop that no
	// longer exists, so it comes out of the timeline with it. A stop that
	// survived - it still serves a car call, the other sweep, or the rest of
	// this queue - keeps its event, which is why this asks the route instead of
	// assuming.
	if (cabin->getStopArrivalTime(stopId) < 0)
	{
		upcomingEvents.extractPending(elevatorId,
			[stopId](const Event& e) {
				return e.getType() == ARRIVE && e.getStopId() == stopId;
			});
	}
	// Dropping a stop pulls every stop after it earlier, and releasing a hall
	// call from one that survives shortens its dwell: either way this cabin's
	// pending events are dated too late.
	retimePendingEvents(elevatorId);
}

void ElevatorSystem::clearBoardedWaiter(Customer* cus)
{
	LandingCall& landing = landingCalls.at(callIndex(cus->getAt(), cus->getDir()));
	for (vector<Customer*>::iterator w = landing.waiting.begin() ; w != landing.waiting.end() ; ++w)
	{
		if (*w != cus) continue;
		landing.waiting.erase(w);
		break;
	}
	// Nobody left at this floor going this way: the button goes out. The next
	// person to arrive lights a fresh call, with its own registration instant -
	// which is Peters cancelling the landing call when the car arrives
	// (section 8.3.5), seen from the queue's side.
	if (landing.waiting.empty())
	{
		landing.allocatedElevator = 0;
		landing.stopId = 0;
		landing.registeredAt = 0;
	}
}

void ElevatorSystem::endAllocationAt(Customer* cus, int elevatorId, unsigned long int stopId)
{
	int index = callIndex(cus->getAt(), cus->getDir());
	const LandingCall& landing = landingCalls.at(index);
	// Only if this really is the stop the call was allocated to. A cabin can be
	// standing at a floor serving a car call while a landing call there belongs
	// to somebody else's cabin, and that allocation must survive.
	if (landing.allocatedElevator != elevatorId || landing.stopId != stopId) return;
	releaseAllocation(index);
}

void ElevatorSystem::ensurePooled(int call)
{
	const LandingCall& landing = landingCalls.at(call);
	if (landing.allocatedElevator != 0) return;		// a cabin is coming: nothing to re-offer
	if (landing.waiting.empty()) return;			// nobody is waiting on it any more
	for (vector<int>::const_iterator q = unallocatedCalls.begin() ; q != unallocatedCalls.end() ; ++q)
	{
		if (*q == call) return;						// already waiting its turn
	}
	// Straight into the pool rather than through deferCall, so the liveness
	// guard there is not consulted. From load bypass that is right - the cabin
	// that just gave the call up is carrying somebody by definition. From
	// releaseUnservedCalls (the other sweep's queue at a reversal stop) the
	// cabin need not be carrying anybody. The call is re-offered at the next
	// giveEvent either way.
	unallocatedCalls.push_back(call);
}

void ElevatorSystem::releaseAllocation(int call)
{
	LandingCall& landing = landingCalls.at(call);
	landing.allocatedElevator = 0;
	landing.stopId = 0;
}

void ElevatorSystem::recordRefusalAtDoor(Customer* cus, int elevatorId, double time)
{
	// The one case the passenger actually witnesses: a cabin arrived, its doors
	// opened and it left without them. Its place in the snapshot timeline is
	// the PASS record written instead of an ENTER, which is also the only
	// record of a stop at which nobody got in or out.
	//
	// Counting is all it does. The call does not go back to the pool here: the
	// passenger presses the button again themselves, at their own instant
	// (scheduleRecall). The caller does that after the cabin's departure is
	// known, because that instant is what the default policy answers with.
	metrics->onCustomerPassedBy(*cus, elevatorId, time);
}

void ElevatorSystem::scheduleRecall(Customer* cus, int elevatorId, int floor, double now,
	double cabinDeparture)
{
	RefusalContext refusal;
	refusal.elevatorId = elevatorId;
	refusal.floor = floor;
	refusal.now = now;
	refusal.cabinDeparture = cabinDeparture;
	double when = passengerBehavior->nextCallTime(*cus, refusal);
	// Clamped, not trusted: a policy that answers with a time in the past would
	// hand the EventQueue an event that pops before the one being processed,
	// and Timer::updateTick would run the clock backwards. An injected policy
	// can make a passenger patient; it cannot make the simulation incoherent.
	if (when < now) when = now;
	// The elevator field carries the cabin that refused them, which is what the
	// RECALL branch excludes from the new allocation. The direction is their
	// own call direction: they press the same button again.
	Event again(0 , "" , RECALL , cus->getDir() , floor , when , cus , elevatorId);
	upcomingEvents.push(again);
}

bool ElevatorSystem::wouldRefuseBoarding(int elevatorId, const Customer& cus, int floor,
	double time) const
{
	if (elevatorId < 1 || elevatorId > number) return false;
	return refusesAtLoad(elevatorId, elevators.at(elevatorId - 1).getCurrentLoad(), cus, floor, time);
}

bool ElevatorSystem::refusesAtLoad(int elevatorId, double load, const Customer& cus, int floor,
	double time) const
{
	const Elevator& cabin = elevators.at(elevatorId - 1);
	// Physics first, and not only because it is cheaper: willBoard is
	// specified as "asked only after the engine has established that they
	// fit", so an implementation is entitled to assume the rated load is not
	// the reason it is being asked. Short-circuiting here is what makes that
	// contract true rather than aspirational. The same sum as Elevator::canBoard,
	// on a load that may not be the cabin's yet.
	if (load + cus.getWeight() > cabin.getMaxWeight()) return true;
	BoardingContext ctx;
	ctx.elevatorId = elevatorId;
	ctx.floor = floor;
	ctx.time = time;
	ctx.currentLoad = load;
	ctx.maxWeight = cabin.getMaxWeight();
	ctx.cabinDirection = cabin.getDirection();
	return !passengerBehavior->willBoard(cus, ctx);
}

vector<bool> ElevatorSystem::decideBoardings(int elevatorId, const vector<Customer*>& candidates,
	int floor, double time) const
{
	vector<bool> refused;
	refused.reserve(candidates.size());
	// Added up the way Elevator::addCustomer adds, in the same order, so the
	// load each person is asked at is bit for bit the one they board at.
	double load = elevators.at(elevatorId - 1).getCurrentLoad();
	for (vector<Customer*>::const_iterator c = candidates.begin() ; c != candidates.end() ; ++c)
	{
		bool refuse = refusesAtLoad(elevatorId, load, **c, floor, time);
		refused.push_back(refuse);
		if (!refuse) load += (*c)->getWeight();
	}
	return refused;
}

void ElevatorSystem::recordReallocation(Customer* cus, int elevatorId, double time)
{
	// NOT a timeline event: nothing happens at the passenger's floor when the
	// cabin that was coming for them is reassigned. They are still waiting, and
	// no car has arrived. Counted, though, because how often the engine takes
	// an allocation back is worth knowing - see PassByReport.
	// Counted per PERSON even though the call is handed back once. Putting the
	// call back in the pool is the caller's job, once per call rather than once
	// per person - see bypassPendingHallCalls.
	metrics->onHallCallReallocated(*cus, elevatorId, time);
}

void ElevatorSystem::bypassPendingHallCalls(int elevatorId, double now)
{
	vector<Elevator>::iterator elevator = elevators.begin() + (elevatorId - 1);
	if (!elevator->isBypassingCalls()) return;

	unsigned long int committed = elevator->committedStopId();
	// Every landing call allocated to this cabin is a pickup it has not made
	// yet; the passengers already inside are its car calls and are not up for
	// discussion. The committed stop is exempt - see Elevator::releaseHallCall:
	// a cabin in flight cannot un-brake, so it still stops there and the people
	// waiting are refused at the open door instead. One call hands back its
	// whole queue at once.
	bool droppedAny = false;
	for (size_t index = 0 ; index < landingCalls.size() ; ++index)
	{
		LandingCall& landing = landingCalls.at(index);
		if (landing.allocatedElevator != elevatorId) continue;
		if (landing.waiting.empty()) continue;
		if (landing.stopId == committed) continue;
		unsigned long int stopId = landing.stopId;
		// One release per person, because that is what the stop counted when
		// the queue joined it: the dwell must stop being priced for transfers
		// that are not going to happen.
		for (size_t w = 0 ; w < landing.waiting.size() ; ++w)
		{
			elevator->releaseHallCall(stopId, landing.direction);
			// Counted per PERSON even though the call is handed back once.
			recordReallocation(landing.waiting.at(w), elevatorId, now);
		}
		// The ARRIVE of a stop that has left the route would fire at a stop
		// that no longer exists, so it comes out of the timeline with it. A
		// stop that survived (it still serves a car call or the other sweep's
		// hall call) keeps its event, which is why this asks the route rather
		// than assuming.
		if (elevator->getStopArrivalTime(stopId) < 0)
		{
			upcomingEvents.extractPending(elevatorId,
				[stopId](const Event& e) {
					return e.getType() == ARRIVE && e.getStopId() == stopId;
				});
		}
		releaseAllocation(index);
		// The rule is "a call whose cabin gave it back must end up either
		// allocated or in the pool" - including a call whose allocation was
		// already over. A call left registered and in no pool is a passenger
		// nobody will come back for, and the event queue would run dry with them
		// still waiting.
		ensurePooled(index);
		droppedAny = true;
	}
	if (!droppedAny) return;
	// A dropped stop pulls every stop after it EARLIER (its dwell has left the
	// sum), so the events still pending for this cabin are now dated too late.
	retimePendingEvents(elevatorId);
}

void ElevatorSystem::showSystem()
{
	vector<Elevator>::iterator it;
	for (it = elevators.begin() ; it < elevators.end() ; it++)
	{
		it->showStatus();
	}
}

Task& ElevatorSystem::giveTask(int e, Task t)
{
	vector<Elevator>::iterator it;
	it = elevators.begin();
	it += e-1;
	return it->addTask(t, 0);
}

void ElevatorSystem::doTask(int e)
{
	vector<Elevator>::iterator it;
	it = elevators.begin();
	it += e-1;
	it->processNextTask();
}

void ElevatorSystem::update(Timer t)
{
	vector<Elevator>::iterator it;
	for (it = elevators.begin() ; it < elevators.end() ; ++it)
	{
		double before = it->getPosition();
		it->update(t);
		double after = it->getPosition();
		metrics->onElevatorMoved(it->getId(), after - before);
	}
}

void ElevatorSystem::giveEvent(Event ev)
{
	// Landing calls that no cabin could answer when they were offered get
	// their chance here, before anything else happens at this instant. This is
	// the only retry point, and it is enough: nothing happens between two
	// events in a discrete-event simulation, so there is no moment at which a
	// cabin frees up and a waiting call is not immediately re-offered. Doing it
	// first also means the route is settled before this event's own timestamps
	// are derived from it - re-offering AFTER that would leave the events this
	// function creates holding a time computed from a route that has since
	// changed. The pool fills even with no capacity configured:
	// releaseUnservedCalls pools the other sweep's queue at a reversal stop in
	// any run.
	//
	// If the event in hand is an ARRIVE, it is out of the queue now, so the
	// queue alone cannot tell ensureArrivalEvent that its stop already has one.
	// Recorded for the whole of this call and cleared on every way out of it.
	struct ArrivalInHand {
		int& elevator; unsigned long int& stop;
		~ArrivalInHand() { elevator = 0; stop = 0; }
	} inHand{arrivalInHandElevator, arrivalInHandStop};
	if (ev.getType() == ARRIVE)
	{
		arrivalInHandElevator = ev.getElevator();
		arrivalInHandStop = ev.getStopId();
	}
	retryUnallocatedCalls(ev.getTime());
	// After the pool is settled, the group dispatcher may move already-allocated,
	// non-committed calls to better cabins. A no-op under the default adapter
	// (it keeps every call where it is).
	reoptimizeAllocations(ev.getTime());
	if (ev.getType() == CUST_CALL)
	{
		Customer* arriving = ev.getCustomer();
		// Arrival bookkeeping happens exactly once, here - not inside
		// joinOrRegisterHallCall, which also runs when a passenger left behind
		// presses the button again. Counting a re-registration as an arrival
		// would restart a waiting time that has to keep running from the first
		// press.
		addCustomer(arriving->getAt(), *arriving);
		metrics->onCustomerArrived(*arriving);
		if (DEBUG) cout<<"Trace : ES -> GiveEvent() ->Event Created"<<endl;
		// Nothing is handed back to the caller: registering a call puts a stop in
		// a cabin's route, and that stop's ARRIVE is the only event there is,
		// pushed by the engine. A call no cabin could take stays registered and
		// creates nothing - a passenger waiting for a lift that has not been
		// assigned yet.
		joinOrRegisterHallCall(arriving, ev.getTime());
		// The pending set just grew, so let the group dispatcher re-optimise the
		// whole allocated set now, at the arrival instant - "each time t the set
		// of hall calls is re-allocated" (Cortes 2004). The new call was placed
		// one-at-a-time by allocateCall above; this pass folds it into the full
		// set. A no-op under the default ImmediateGroupAdapter, which keeps every
		// allocated call where it is.
		reoptimizeAllocations(ev.getTime());
		if (DEBUG) showSystem();
		return;
	}
	else if (ev.getType() == RECALL)
	{
		// The same registration as a first press, with two differences that
		// are the whole point of the event.
		//
		// First, there is NO arrival bookkeeping: onCustomerArrived fired when
		// they pressed the button the first time and their waiting time has to
		// keep running from that instant, or being left behind would reset the
		// very number that measures it. addCustomer is not repeated either -
		// they never left the floor.
		//
		// Second, the cabin that refused them is excluded from the allocation.
		// A no-op when the call they are on has already been re-registered by
		// somebody else's press at the same instant and has a cabin coming:
		// joinOrRegisterHallCall sees they are queued on an allocated call and
		// creates nothing. That is what keeps a stop that refused several
		// people from producing several allocations of the same call.
		// Unless they are not waiting any more. The cabin that refuses somebody
		// is often not the one their call was allocated to, so between the
		// refusal and the re-call ANOTHER cabin can turn up going their way and
		// take them. The queue of their own call is what says which it was: a
		// refused passenger stays in it, and boarding is the only thing that
		// takes them out (clearBoardedWaiter). Re-registering a passenger who is
		// already riding would put them back on a landing they are not standing
		// on and board them a second time - caught by the `[metrics_invariants]`
		// "nobody boards twice".
		Customer* again = ev.getCustomer();
		const LandingCall& theirCall = landingCalls.at(callIndex(again->getAt(), again->getDir()));
		bool stillWaiting = false;
		for (size_t w = 0 ; w < theirCall.waiting.size() ; ++w)
		{
			if (theirCall.waiting.at(w) == again) { stillWaiting = true; break; }
		}
		if (!stillWaiting) return;
		joinOrRegisterHallCall(again, ev.getTime(), ev.getElevator());
		// Same as the CUST_CALL branch: the set may have grown (a re-call that
		// re-registers a pooled call), so re-optimise the full allocated set now.
		// No-op under the default adapter.
		reoptimizeAllocations(ev.getTime());
		if (DEBUG) showSystem();
		return;
	}
	else if (ev.getType() == ARRIVE)
	{
		// A cabin is standing at one of its stops with the doors open. The order
		// inside this branch is the physical one (Peters' figure 8.10): everybody
		// who is getting out gets out, then everybody who is getting in gets in.
		int x = ev.getElevator();
		vector<Elevator>::iterator it = elevators.begin() + (x - 1);
		unsigned long int stopId = ev.getStopId();
		int floor = ev.getFloor();

		// The doors are open at this stop, so for as long as it is being served
		// it is the front of the cabin's route and nothing goes in front of it -
		// not the car call of somebody boarding, not a re-sort. It has to be the
		// FIRST thing the branch does: the very next step, releaseUnservedCalls,
		// deletes a stop that serves nobody unless it is the committed one, and
		// "committed" means "at the front". If the route had been sorted with this
		// stop elsewhere, the other cabins' pending events are fine but this one's
		// are not, so they are re-timed from the repaired route.
		if (it->beginServing(stopId)) retimePendingEvents(x);

		// Rule 2 of the five rules of lift traffic control: a car never passes
		// a floor where somebody wants to alight. Asked of the cabin's own load
		// (see Elevator::dischargeAt).
		vector<Customer> leaving = it->dischargeAt(floor);

		// The cabin decides and announces which way it is going, BEFORE anybody
		// boards. That order is what makes "a passenger does not get into a car
		// travelling the other way" (Rule 3 of Closs, 1970) a statement the engine
		// can enforce, and it is the order of Peters' own simulation cycle - the
		// lifts are updated and the answered calls cancelled before the passengers
		// are (figure 8.10).
		unsigned int boardingDirection = boardingDirectionAt(x, stopId, floor);
		// The other sweep's queue at a reversal stop is not served here, so its
		// call goes back now - before the stop's remaining dwell is computed
		// from it, and before the stop leaves the route carrying a promise it
		// did not keep.
		releaseUnservedCalls(x, stopId, floor, boardingDirection);

		// **Everybody waiting at this floor to go that way**, whatever cabin
		// their call was allocated to. This one line is the item: the boarding
		// is resolved by the cabin that ARRIVES, not by the cabin the
		// dispatcher picked, which is what `CancelLandingCalls(lift l[])` means
		// by taking the whole lift array (Peters section 8.3.5).
		vector<Customer*> candidates = waitersToBoardAt(floor, boardingDirection);

		// Who really gets in, decided here and not one boarding at a time inside
		// the loop below, and written on the stop before anybody moves. The first
		// transfer takes the stop off the route and fixes what its dwell still
		// owes, so anything learned after that changes nothing. Until this line
		// the stop's counters are a forecast made from who registered on it, which
		// counts neither the people refused nor the ones boarding from a queue
		// allocated to other cabins.
		vector<bool> refused = decideBoardings(x, candidates, floor, ev.getTime());
		int boarding = 0;
		for (vector<bool>::const_iterator r = refused.begin() ; r != refused.end() ; ++r)
		{
			if (!*r) ++boarding;
		}
		it->settleServedStop(stopId, boarding, static_cast<int>(leaving.size()));

		// The stop's transfers, counted up front so that each one knows whether
		// it is the last. The leg out of this stop must not be decided until it
		// is - the sweep direction and the load the leg is priced on both
		// depend on the car calls of the people still boarding.
		int total = static_cast<int>(leaving.size() + candidates.size());
		int done = 0;
		// Bookkeeping for the pickups taken from another cabin: which call, that
		// cabin, and how many of its promised passengers got into this one.
		int servedCall = -1;
		int servedBy = 0;
		unsigned long int servedStopId = 0;
		int boardedFromOther = 0;

		for (vector<Customer>::iterator c = leaving.begin() ; c != leaving.end() ; ++c)
		{
			metrics->onCustomerAlighted(*c, x, ev.getTime());
			recordTransfer(LEAVE, x, floor, *c, ev.getTime());
			++done;
			// The dwell of the stop just served delays every stop after it, so
			// the pending events have to be re-timed - but only when it really
			// did (see advanceAndReportDeparture's contract: re-deriving the
			// same route sum is not a no-op in floating point).
			if (advanceAndReportDeparture(*it, ev.getTime(), stopId, done < total))
			{
				retimePendingEvents(x);
			}
		}

		for (size_t w = 0 ; w < candidates.size() ; ++w)
		{
			Customer* cus = candidates[w];
			++done;
			bool holdLeg = (done < total);

			// The doors are open and there may be no room. This is the one
			// place load bypass cannot design the problem away:
			// bypassPendingHallCalls hands back the pickups a full cabin has
			// not made yet, but never the stop it is already committed to, and
			// two heavy passengers waiting at one stop can also fill it between
			// one boarding and the next.
			//
			// The test is the PHYSICAL one (canBoard) plus the passenger's own
			// policy (willBoard); refusing somebody who fits and wants in would
			// be the "refusal" that Barney & Al-Sharif 11.6.3 says to avoid at
			// all costs.
			if (refused[w])
			{
				recordRefusalAtDoor(cus, x, ev.getTime());
				// A cabin came, opened its doors and left without them: that is
				// what the timeline records, in the place the boarding would
				// have had.
				recordTransfer(PASS, x, floor, *cus, ev.getTime());
				// They are waiting again, and the cabin has served (or is
				// serving) the stop their call was allocated to - so that
				// allocation is over. The queue is untouched: they never left
				// the floor.
				endAllocationAt(cus, x, stopId);
				// The stop still completes - the cabin is standing at that
				// floor - so the leg decision is taken exactly as it would be
				// for a boarding, which is what keeps a refused stop from
				// stranding the cabin.
				if (advanceAndReportDeparture(*it, ev.getTime(), stopId, holdLeg))
				{
					retimePendingEvents(x);
				}
				// AFTER the leg decision, because the departure instant is what
				// the default policy answers with and it does not exist until
				// the leg does. Two cases where this is simply `now`: the cabin
				// parked, and the stop still has transfers to come, in which case
				// the leg is decided by a later one. Both read as "the person
				// presses again while the doors are still open", which is what a
				// person does - and with a zero dwell all three instants coincide.
				//
				// And only when nothing is coming for them. The cabin that refuses
				// somebody is often not the one their call was allocated to - it is
				// simply the one that turned up - so their own car may well still be
				// on its way. Pressing the button again then is both pointless (the
				// call is registered and allocated, so the press is a no-op) and
				// misleading in the timeline, which would show a re-call for somebody
				// who never lost their place.
				if (landingCalls.at(callIndex(cus->getAt(), cus->getDir())).allocatedElevator == 0)
				{
					double departure = it->getLegStartTime();
					if (departure < ev.getTime()) departure = ev.getTime();
					scheduleRecall(cus, x, floor, ev.getTime(), departure);
				}
				continue;
			}

			metrics->onCustomerBoarded(*cus, x, ev.getTime());
			recordTransfer(ENTER, x, floor, *cus, ev.getTime());
			// Which cabin had been promised this pickup, read BEFORE the
			// boarding clears it. When that is another cabin, it is now making
			// a stop for somebody who is already riding, and the stop has to
			// come off its schedule - see withdrawStopFrom, below the loop.
			int call = callIndex(cus->getAt(), cus->getDir());
			int promisedTo = landingCalls.at(call).allocatedElevator;
			if (promisedTo != 0 && promisedTo != x)
			{
				// The stop id is read HERE and remembered. Reading it after the
				// loop does not work and the failure is silent: boarding the
				// last person of a call empties its queue, and clearBoardedWaiter
				// then clears the whole call - including the stopId - so the
				// cabin that was coming would keep a stop for passengers who
				// are already riding, which is what Rule 1 of Closs forbids.
				if (servedCall == -1)
				{
					servedCall = call;
					servedBy = promisedTo;
					servedStopId = landingCalls.at(call).stopId;
				}
				boardedFromOther++;
			}
			// Out of the landing queue - they are inside now - and the call's
			// allocation is over, because the cabin it named has arrived. If
			// that was the last person waiting there, the button goes out and
			// the next arrival lights a fresh call: Peters cancels a landing
			// call when the car arrives (section 8.3.5).
			endAllocationAt(cus, x, stopId);
			clearBoardedWaiter(cus);

			// The cabin CAN already be idle here, which is worth stating
			// because it is not obvious: a stop whose entry was dropped leaves
			// a route that may empty before this transfer runs, and
			// processNextTask parks the cabin when it does.
			bool wasIdle = (it->getStatus() == IDLE);

			// The car call is registered HERE, before the leg decision, so
			// processNextTask chooses the next sweep direction knowing this
			// passenger's destination.
			Task carCall(0 , "" , GO , DEFAULT , cus->getTo() , cus->getTime());
			Task registered = it->addTask(carCall, 1);

			// Also before the leg decision, so getCurrentLoad() includes this
			// passenger when the leg leaving this stop is priced.
			Customer boarding = *cus;
			it->addCustomer(boarding);
			it->updateAvg(cus->getTime());

			// A boarding is the only thing that raises a cabin's load, so it is
			// the only moment a cabin can become "full" in the load-weighing
			// sense - and the moment it has to give back the pickups it can no
			// longer make (Barney & Al-Sharif 10.8.3). Before the arrival of this
			// passenger's own destination is derived below, because dropping a
			// stop changes the route those timestamps come from.
			bypassPendingHallCalls(x, ev.getTime());

			if (wasIdle)
			{
				// addTask() only starts a leg - and so only needs a departure
				// reported for it - when the cabin was parked. After
				// addCustomer(), so the load includes the passenger whose car
				// call just woke the cabin. advanceAndReportDeparture below
				// reports nothing in this case: addTask already made this car
				// call the leg in flight.
				double departureTime = it->getLegStartTime() > ev.getTime() ? it->getLegStartTime() : ev.getTime();
				beginLeg(x, it->getPosition(), it->getCurrentLoad(), departureTime);
			}

			// Pops the finished stop and starts the next leg, one departure per
			// leg and not per boarding passenger sharing this stop.
			advanceAndReportDeparture(*it, ev.getTime(), stopId, holdLeg);

			// The ARRIVE of this passenger's destination is created only now,
			// AFTER the stop just served has left the route. Creating it before
			// would make the route sum charge that stop's *full* dwell, door
			// opening included - and the doors opened before this boarding, so
			// it is already spent. What is still owed is the tail (transfer +
			// closing), which processNextTask has just folded into
			// legStartTime. Getting this backwards dated every boarding
			// passenger's arrival one getDoorOpenTime too late (caught by
			// test_dwell_time.cpp: 18.0 instead of 16.0).
			ensureArrivalEvent(x, registered.getStopId());

			// Unconditional, unlike the alighting loop above: the addTask
			// always changed the route, so the pending events of this elevator
			// are stale for a reason that has nothing to do with the dwell.
			retimePendingEvents(x);
		}

		// The other cabin was promised pickups that just got into this one, so
		// it must not keep a stop for them: Rule 1 of Closs, and Peters'
		// lift::RemoveLandingCall. After the transfers, because it is only now
		// known how many of them boarded here.
		if (servedCall != -1) withdrawStopFrom(servedBy, servedStopId, servedCall, boardedFromOther);

		// A stop where nobody got in and nobody got out still has to complete,
		// or the cabin stands there forever with the stop at the front of its
		// route and no event left to finish it. It is reachable: a full cabin
		// hands back the pickups it cannot make, and the one stop it may not
		// drop is the one it is already committed to.
		if (total == 0)
		{
			if (advanceAndReportDeparture(*it, ev.getTime(), stopId, false))
			{
				retimePendingEvents(x);
			}
		}
		// Normally already released, by the processNextTask that finished the
		// stop. Cleared here as well so that a pin can never outlive its ARRIVE.
		it->endServing();
		if (DEBUG) showSystem();
		return;
	}
	else if (ev.getType() == CUST_GO)
	{
		// CUST_GO is not an event: a car call is registered by the ARRIVE branch
		// as its passenger boards, before the next leg is decided. Throwing
		// rather than ignoring: a CUST_GO reaching here would register the same
		// car call twice.
		throw std::logic_error("ElevatorSystem: CUST_GO is no longer an event - "
			"the ARRIVE branch registers the car call");
	}
	else if (ev.getType() == ENTER || ev.getType() == LEAVE)
	{
		// Same reasoning as CUST_GO above. A boarding and an
		// alighting have no instant of their own - both happen at their stop's
		// ARRIVE - and neither can be scheduled, because who boards is not
		// knowable until the cabin is standing at the floor. They are records
		// now (drainTransfers), and one reaching the queue means something
		// pushed an event that cannot be processed: the stop it belongs to has
		// its own ARRIVE, which would serve the same passenger a second time.
		throw std::logic_error("ElevatorSystem: ENTER/LEAVE are no longer events - a stop is one "
			"ARRIVE and the transfers are records");
	}
	else
	{
		throw std::logic_error("ElevatorSystem::giveEvent: unknown event type");
	}
}
void ElevatorSystem::addCustomer(int floor, Customer cus)
{
	(allCustomers.at(floor)).push_back(cus);
}

void ElevatorSystem::removeCustomer(int floor, Customer cus)
{
	vector<Customer>::iterator it;
	for (it = (allCustomers.at(floor)).begin() ; it < (allCustomers.at(floor)).end() ; ++it)
	{
		if (cus == it) (allCustomers.at(floor)).erase(it);
		else continue;
	}
}

void ElevatorSystem::printMetrics()
{
	vector<Elevator>::iterator it = elevators.begin();
	// Every leg is priced when the cabin arrives, so the last leg of each
	// elevator has nothing arriving after it to close it.
	flushLegsInFlight();
	metrics->onSimulationEnd(it->getTime());
	metrics->printSummary();
}

const vector<Elevator>& ElevatorSystem::getElevators() const
{
	return elevators;
}

const shared_ptr<ITravelTimeStrategy>& ElevatorSystem::getTravelTimeStrategy() const
{
	// Every cabin got the same instance from the constructor, so cabin 0's is
	// the group's. Asserting that here rather than storing a second copy keeps
	// one owner of the fact.
	return elevators.front().getTravelTimeStrategy();
}

const shared_ptr<IDwellTimeStrategy>& ElevatorSystem::getDwellTimeStrategy() const
{
	// Same single-owner argument as getTravelTimeStrategy().
	return elevators.front().getDwellTimeStrategy();
}

vector<ElevatorSystem::Transfer> ElevatorSystem::drainTransfers()
{
	vector<Transfer> drained;
	drained.swap(pendingTransfers);
	return drained;
}

vector<ElevatorSystem::Departure> ElevatorSystem::drainDepartures()
{
	vector<Departure> result;
	result.swap(pendingDepartures);
	return result;
}

int ElevatorSystem::getUnallocatedCallCount() const
{
	return static_cast<int>(unallocatedCalls.size());
}

int ElevatorSystem::getUnallocatedPassengerCount() const
{
	// Over every registered call with no cabin, not just the ones in the pool:
	// somebody who was refused at a door is waiting with no cabin coming too,
	// and their call is out of the pool while their RECALL is pending.
	int waiting = 0;
	for (vector<LandingCall>::const_iterator c = landingCalls.begin() ; c != landingCalls.end() ; ++c)
	{
		if (c->allocatedElevator != 0) continue;
		waiting += static_cast<int>(c->waiting.size());
	}
	return waiting;
}

} // namespace openEGCS
