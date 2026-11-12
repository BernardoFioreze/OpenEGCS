# Architecture

This document explains how the engine works: what an event is, how a call
becomes a stop, how a stop is served, how time is derived, and what the engine
guarantees. The header of each class says the same things at the level of the
code; this is the map.

## The pieces

- **`Simulator`** owns one `ElevatorSystem`, generates the arrivals (through an
  `ITrafficProfile`), runs the event loop, and hands the timeline to an
  `ISimulationLogger` at the end.
- **`ElevatorSystem`** owns the cabins (`Elevator`), the landing calls, and the
  event queue. `giveEvent` is the state machine of the whole simulation.
- **`Elevator`** is one cabin: its route (an ordered list of stops, `Task`), its
  position, direction and load, and the clock arithmetic that turns the route
  into instants.
- **`EventQueue`** is a min-heap ordered by `(time, insertion sequence)`. The
  sequence number breaks ties, which is what makes runs deterministic.

Every decision that is a policy rather than mechanics is an injected interface;
[extending.md](extending.md) lists them. The engine adds no branch for a new
"kind" of strategy: a new algorithm is a new implementation of an interface.

## Events and records

There are three event types, and they are the only things in the queue:

| Event | Meaning |
|---|---|
| `CUST_CALL` | A passenger arrives at a landing and presses the button. |
| `RECALL` | A passenger who was left behind at an open door presses the button again, at an instant of their own. |
| `ARRIVE` | A cabin reaches one stop of its route. The only event derived from the route. |

Four more things appear in the snapshot's timeline but are **records**, not
events: `ENTER`, `LEAVE`, `PASS` (a passenger who did not get in) and `DEPART`
(a cabin starts moving towards its next stop). None of them has an instant of its
own and none can be scheduled - who boards is not knowable until the cabin is
standing at the floor with its doors open - so the engine reports them while it
serves an `ARRIVE`, and the `Simulator` drains them into the timeline. That keeps
`ElevatorSystem` ignorant of the logger.

Inside one instant the rows are in physical order: a stop's `ARRIVE`, its
alightings, its boardings and refusals, then the `DEPART` of the leg it leaves on.

## Landing calls

A landing call is **one per (floor, direction)**, with the people behind it as a
queue - which is what a landing button is. The second person to press a lit
button joins the call and its existing stop; the dispatcher is not consulted
again. So the dispatcher is asked once per lit button, not once per passenger,
and a call is timestamped at its **first** press: it does not get younger because
somebody joined it. The wait of each person still runs from their own press.

A call is either allocated to a cabin (whose route holds the stop that serves it)
or in the **pool** of unallocated calls. The pool is retried, oldest first, at the
top of every `giveEvent`, before anything else at that instant. Nothing happens
between two events in a discrete-event simulation, so that is enough: there is no
moment at which a cabin frees up and a waiting call does not get its chance.

Landing calls are never cancelled (Barney & Al-Sharif 10.8.3): a passenger who is
not served on the spot keeps their call, so nobody is ever lost.

## Serving a stop

When a cabin's `ARRIVE` fires, the engine serves the stop in the order of Peters'
own simulation cycle (the lifts are updated and the answered calls cancelled
before the passengers are):

1. **The stop is pinned** at the front of the route for as long as it is being
   served, so nothing sorts it away from the cabin standing at it.
2. **Everybody bound for this floor gets out** (Closs rule 2: a car never passes
   a floor where somebody wants to alight).
3. **The cabin announces its direction** (`ElevatorSystem::boardingDirectionAt`).
4. **Everybody waiting at this floor to go that way gets in**, in arrival order,
   while they fit and accept - *whatever cabin their call was allocated to*.
   Boarding is decided at the door, not at allocation: a cabin that stops at your
   floor going your way takes you.
5. **The rest are refused at the open door**: a `PASS` record, and a `RECALL`
   at the instant the cabin pulls away (the default passenger policy).
6. **The stop is taken off the cabin that had been promised it**, if another
   cabin served its passengers, so that cabin does not make an empty stop.

Who boards is decided before the first transfer, because the stop's dwell is
charged for the transfers that really happen.

### Which way the cabin announces

The announcement has four sources, asked in order:

1. a landing call of this stop going the way the cabin is already going;
2. the route (`Elevator::announcedDirectionAt`), for a stop made for somebody
   inside the cabin;
3. a landing call of this stop in any direction (end of a sweep; the older call
   wins if both are lit);
4. the oldest call at this floor that has anybody waiting, whatever cabin it was
   allocated to - for a cabin that arrived on a car call alone, emptied, and has
   nowhere to go.

`IDLE` means nobody is waiting at that floor at all. The order matters: reading
the route first does not terminate (a cabin at floor 7 for an up-call, with a
down-call at 3 in its route, would oscillate between the two floors for ever), and
reading any call before the route carries passengers the wrong way.

## Time

A stop's timestamp is **derived from the cabin's live route**, never stored and
patched. Every stop has a `stopId`, unique within its cabin, and exactly one
`ARRIVE` carrying it. After any route change, the engine recomputes the pending
`ARRIVE`s of that cabin from the route (`EventQueue::retime`).

- `Elevator::getStopArrivalTime(stopId)` sums the leg in flight, then each later
  leg (`ITravelTimeStrategy::legDuration`) and the full dwell of every earlier
  stop. It starts from `max(currentTime, legStartTime)`, because a cabin sitting
  out a dwell has not moved yet.
- `Elevator::getStopEventTime(stopId)` adds the door opening: the `ARRIVE` (and
  every transfer record of that stop) is dated when the doors are open. It is kept
  separate so the dispatch cost (`getFinishTime`) is not charged a door the cabin
  has not reached.

A stop runs:

```
arrival --door opens--> ARRIVE: LEAVE / ENTER / PASS --doors open (transfer, hold)--> door closes --> DEPART
```

The open-doors phase is `max(hold, n x tp)` with the default door controller
(`DoorHoldDwellTimeStrategy`): the doors stay open at least 2.0 s at a stop for a
car call alone and 3.0 s at any other stop (Barney & Al-Sharif 5.5.2), or longer
while `n` passengers transfer at `tp` seconds each.

One trap: re-timing when nothing changed is **not** free. The same route sum
re-derived further along a leg need not land on the same bits in floating point,
and the queue's `(time, seq)` tie-break turns a last-bit difference into a
different simulation. The engine therefore re-times only when a stop really moved
the later ones.

The invariant that ties the clock to the cabin, and is tested: **when a stop's
`ARRIVE` fires, the cabin is at that stop's floor.**

## Travel

The default travel model (`KinematicTravelTimeStrategy`) drives both the event
timestamps and the cabin's position from a jerk-limited motion profile (Peters
1998, ch. 6), the same profile the energy model integrates. Starting and stopping
dominate a leg's cost, so nine one-floor hops cost far more than one nine-floor
run (46.5 s against 22.6 s with the default dynamics). The constant-speed model
(`LinearTravelTimeStrategy`) is kept for comparison.

The kinematic model has two consequences the engine handles explicitly:

- **Duration is not additive in distance.** The time left on a leg in flight is
  asked of the model (`remainingDuration`), never re-priced as a fresh leg from
  the cabin's fractional position.
- **A cabin in flight cannot stop just anywhere.** It needs braking room
  (`minimumLegDistance`, Peters' d_min). Floors ahead of it that it can no longer
  stop at are, for route order, already behind it: the route is ordered against
  `Elevator::stoppingPosition()`, not `position`. A nearer call that becomes the
  front of the route **retargets** the leg in flight - it keeps its origin and its
  clock and changes only its destination (Peters' ChangeJourney), so the motion
  stays continuous.

A leg does not always start at a floor, because a retarget happens mid-flight.

## Route ordering

`IElevatorSchedulingStrategy` decides the order a cabin serves its stops (LOOK by
default). Route order is relative to where the cabin is and which way it is
going, so the cabin asks the strategy to re-establish it (`reorder`) before every
insertion and after every completed stop - reversing inverts the order the
stops behind the cabin must be served in.

`Elevator::addTask` merges tasks for one floor into a single route entry, so a
route entry is a **stop** and may serve several requests at once: at a sweep
reversal, an up-call and a down-call at the top floor are one stop. Ask a stop
`Task::servesSweep(direction)` rather than reading `Task::getDirection()`, which
names only the request that arrived first.

## Capacity

When a rated load is configured, capacity is enforced in three layers:

1. **Physics.** `Elevator::addCustomer` throws rather than carry more than the
   rated load, so the engine asks `Elevator::canBoard` first.
2. **Load-weighing bypass.** A cabin that crosses the bypass threshold (60% by
   default, Barney & Al-Sharif 14.3.5) gives back the pickups it has not made yet,
   so it drives past floors instead of stopping where nobody can board. The one
   stop it keeps is the one it is already braking for; the people there are
   refused at the open door.
3. **The dispatch veto.** The engine does not allocate a landing call to a cabin
   that is bypassing or has no room (`Elevator::canAnswerHallCall`), even if the
   dispatch strategy picks one. When another cabin can answer, the call is
   redirected to it rather than pooled.

The veto is load-bearing for **liveness**: without it, a refused passenger would
be re-allocated to the same full cabin standing at their floor, at the same
instant, forever.

## Passenger behaviour

`IPassengerBehaviorStrategy` holds the decisions that belong to the passenger:
whether they board the car in front of them (asked only after the engine knows
they fit), and when they press the button again after being left behind. The
default boards every car it fits into. A policy that declines a car could
livelock the run, so the engine keeps the refusing cabin out of the re-offer of
that call while it is still standing at the floor, and releases it once it
departs.

## Group dispatch

`IGroupDispatchStrategy` decides the assignment of the pending calls to cars as a
whole, and may move a call that has not been served yet. The engine offers the
allocated, non-committed calls to it at the top of every `giveEvent` and at the end
of the `CUST_CALL` and `RECALL` branches, and applies the moves it asks for in a
deterministic order (`ElevatorSystem::reassign`). A call is **committed** when its
stop is the front of its cabin's route - that cabin is braking for it - and is
never moved. The default, `ImmediateGroupAdapter`, wraps the per-call strategy and
never moves anything.

## Energy

Energy is priced per leg by an electromechanical model (load torque, inertia, DC
motor, power integrated over the motion profile; Peters 1998, ch. 7), through
`IMetricsCollector::onElevatorDeparted`. A leg is charged when it **completes**,
with the distance it actually covered - a nearer call inserted mid-flight makes a
cabin stop early, and charging the intended distance would pay for the abandoned
stretch twice. The load is captured at **departure**, and includes the passengers
who boarded at the stop the leg starts from. The hook therefore arrives one leg
behind; the last leg of each cabin is flushed at the end of the run.

## Metrics

`StandardMetricsCollector` reports waiting (arrival to boarding), travel (boarding
to alighting) and journey (arrival to alighting) as distributions: count, mean,
p50, p90 (nearest rank) and worst case. Journey is exactly waiting plus travel,
because a boarding is one instant.

- **Windows.** A run has an arrival window, up to the last arrival, and a drain
  after it in which the building only empties. Each distribution is also reported
  split into `peak` and `drain` by the instant the metric closed. Read them
  together: `peak` alone excludes exactly the passengers who waited through the
  peak and were served while draining, so it is an optimistic floor.
- **Queue.** The backlog of passengers who have arrived and not boarded: its
  maximum, and its value when arrivals ended (equal to the drain count of the
  waiting distribution, by construction).
- **Pass-bys.** Passengers refused at an open door, and calls a full cabin handed
  back to the dispatcher (counted per person).
- **Dissatisfaction.** With a configured tolerance threshold, the count, rate and
  ids of the passengers above it. It is a label applied after the fact: nobody
  gives up, and configuring a threshold changes no event. Without a threshold
  there is no verdict, which is not the same as a verdict of zero.

A metric a collector does not measure is reported as absent, never as zero.

## Determinism

The same seed and the same build produce the same snapshot, byte for byte. What
keeps it so:

- every random draw comes from the one injected `IRandomProvider`, and **the order
  of the draws is a contract** - a component that draws one more number changes
  every draw after it;
- no iteration order comes from a hash container: landing calls live in a vector
  indexed by floor, and every collection a result depends on is ordered by the
  simulation;
- ties in the event queue are broken by insertion sequence;
- re-timing is conditional (see *Time* above).

## The rules of lift traffic control

Barney & Al-Sharif (section 10.4) list the five rules of Closs (1970): no stop
where nobody boards or alights (1), never pass a floor where somebody wants to
alight (2), no boarding a car travelling against one's own direction (3), no
reversal while carrying passengers (4), car calls outrank landing calls (5).

- **Rule 2** is enforced at every stop.
- **Rule 3** is enforced by the announcement: only the queue going the announced
  way boards. It holds under a route policy that keeps sweeping, such as LOOK. The
  announcement trusts that a cabin leaves the way it arrived; under
  `examples::FcfsSchedulingStrategy`, which does not, a cabin can announce up,
  board the up passengers and leave down.
- **Rule 1** is enforced from two sides - a full cabin gives back the pickups it
  cannot make, and a stop whose passengers another cabin collected is withdrawn.
  The residue is the stop a cabin was already braking for, which it still makes.
- **Rule 4** is not enforced by construction: nothing prevents a cabin from
  reversing with somebody aboard. Under LOOK no rider is carried away from their
  destination in the reference scenarios, but that is a measurement, not a
  guarantee.
- **Rule 5** is a property of the route policy.

A dispatch strategy may assume rule 3 under LOOK, and must not assume rule 4.

## Known limitations

- A passenger policy that refuses a **parked, empty** cabin can end a run with
  "no elevator can answer a landing call": the cabin never departs, so its
  exclusion is never released. The case is in the test suite, hidden, tagged
  `[.known_issue]`. The shipped policies never trigger it.
- The command-line front end selects neither the dispatch strategy nor a peaked
  traffic profile; those are chosen through the library API.
