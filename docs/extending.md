# Extending the simulator

Every policy is an interface, injected through the constructor of `Simulator`
(or `ElevatorSystem`). To try a new algorithm, implement the interface and pass an
instance in; nothing in the engine needs to change. The header of each interface
documents its contract in full; this page is the overview.

## Where things are injected

`Simulator`'s constructor takes, in order:

```cpp
Simulator(unsigned long id, std::string name, int floors, int elevators,
          int customers, double arrivalRate, double elevatorSpeed, bool sane,
          std::shared_ptr<IRandomProvider>              randomProvider,
          std::shared_ptr<IDispatchStrategy>            dispatch,          // required
          std::shared_ptr<IMetricsCollector>            metrics,
          std::shared_ptr<ISimulationLogger>            logger,
          std::optional<uint64_t>                       seedForMetadata,
          double                                        maxWeightKg,
          std::shared_ptr<ICapacityStrategy>            capacity,
          double meanCustomerWeightKg, double stddevCustomerWeightKg,
          std::shared_ptr<IDwellTimeStrategy>           dwell,
          std::shared_ptr<ITravelTimeStrategy>          travel,
          std::shared_ptr<IPassengerBehaviorStrategy>   passengerBehavior,
          std::shared_ptr<IGroupDispatchStrategy>       groupDispatch,
          std::shared_ptr<ITrafficProfile>              trafficProfile,
          std::shared_ptr<IElevatorSchedulingStrategy>  scheduling);
```

Every pointer except `dispatch` may be null, which selects the default listed
below. `floors` is the **highest** floor, not a count: floors run `0..floors`.
`src/main.cpp` and the benches in `examples/*/bench/` are complete, working
examples of wiring a run.

Each policy is **one shared instance** for the whole group of cabins: all cabins
have the same doors, the same travel model, the same route policy. A shared policy
must therefore keep no per-cabin state.

## The interfaces

| Interface | Decides | Default |
|---|---|---|
| `IDispatchStrategy` | which cabin answers a new landing call | none - required (`examples::GreedyNearestDispatchStrategy` is the baseline) |
| `IGroupDispatchStrategy` | the assignment of all pending calls, with reassignment | `ImmediateGroupAdapter` (delegates to the per-call strategy, never reassigns) |
| `IElevatorSchedulingStrategy` | the order a cabin serves its stops | `LookSchedulingStrategy` |
| `ITrafficProfile` | when passengers arrive, and between which floors | `StationaryUniformTrafficProfile` |
| `IPassengerBehaviorStrategy` | whether a passenger boards; when they call again | `StandardPassengerBehavior` (boards whatever they fit into) |
| `IDwellTimeStrategy` | how long a stop takes | the handbook's centre-opening door with a timed hold (`DwellPresets.h`) |
| `ITravelTimeStrategy` | how long a leg takes, and where the cabin is during it | `KinematicTravelTimeStrategy` |
| `ICapacityStrategy` | the load-weighing bypass threshold | `PercentageCapacityStrategy` (60%) |
| `IMetricsCollector` | what is measured | `StandardMetricsCollector` |
| `ISimulationLogger` | what is written at the end | `JsonSimulationLogger` (`snapshot.json`) |
| `IRandomProvider` | the random numbers | `RandomProvider` (`std::mt19937_64`) |

### Dispatch: `IDispatchStrategy`

```cpp
int selectElevator(Task task, std::vector<Elevator>& elevators);
```

Return the 1-based id of the cabin that should answer the landing call in `task`
(`task.getDest()` is the floor, `task.getDirection()` the direction, and the
task's time is the call's first press). Called once per lit button.

- **Leave `elevators` unmodified.** A strategy that prices a candidate by a
  dry-run insertion (`Elevator::addTask(task, 0)`) must undo it. The engine hands
  you a scratch copy, so a mistake cannot corrupt the simulation, but a direct
  caller gets the real cabins. The greedy baseline restores each cabin whole.
- **The engine enforces capacity.** A cabin that is bypassing or has no room
  (`Elevator::isBypassingCalls()`, `Elevator::canAnswerHallCall(weight)`) is not
  given the call even if you pick it; the call goes to a cabin that can answer, or
  waits in the pool. Skipping such cabins yourself is still better dispatching.
- `Elevator::getFinishTime(task)` estimates when a cabin would finish its route
  up to `task`, under the injected travel, dwell and route models; it is the cost
  the greedy baseline ranks by. Ask it after a dry-run insertion.

### Group dispatch: `IGroupDispatchStrategy`

```cpp
std::vector<int> allocate(const std::vector<HallCall>& calls,
                          const std::vector<Elevator>& elevators,
                          IRandomProvider& rng);
```

Return one 1-based cabin id per call, in the order of `calls`. This is the shape
the optimisation and metaheuristic literature needs: decide the whole assignment
and re-optimise it.

- A `committed` call (its stop is the front of its cabin's route; that cabin is
  braking for it) must keep `currentElevatorId`; the engine ignores anything else
  for it. An id outside `1..N` leaves a non-committed call unallocated this round.
- `calls` is a **subset** of the building's calls: the new call alone (when it is
  registered), or the allocated non-committed calls (re-optimisation, which runs
  right after). Calls not handed over are read off the cabins' routes if you need
  them, and so is the current time: `allocate` receives no clock.
- **Draw every random choice from `rng`**, or from a provider of your own seeded
  from the run's seed (so variants run with one seed see the same passengers), and
  apply your choices in a deterministic order.

`examples::GahcaDispatchStrategy` and `examples::BeamurgiaDispatchStrategy` are
complete implementations.

### Route ordering: `IElevatorSchedulingStrategy`

```cpp
std::vector<Task>::iterator findInsertionPoint(std::vector<Task>& tasks,
    unsigned int direction, double position, const Task& newTask) const;
void reorder(std::vector<Task>& tasks, unsigned int direction, double position) const;
```

`findInsertionPoint` returns where a new stop goes in a route already in route
order; an iterator to a stop for the same floor means "merge into it". `reorder`
re-establishes route order for the cabin's current position and direction, and is
called before every insertion and after every completed stop - implement it if
your order depends on where the cabin is or which way it goes, and keep it
**stable** for entries that compare equal.

- `direction` is always `UP` or `DOWN`; `position` is the cabin's *stopping*
  position (the first floor it can still stop at).
- A route entry is a stop that may serve several requests: ask
  `Task::servesSweep(direction)`, not `Task::getDirection()`.
- The stop the cabin is being served at is pinned; you are handed the rest.
- Your policy is also used by the dispatcher's what-if copies, so dispatch costs
  are priced under it.
- The announcement of direction trusts that a cabin leaves the way it arrived. A
  policy that does not keep sweeping (FCFS, for instance) breaks that, and
  passengers can then be carried the wrong way.

### Traffic: `ITrafficProfile`

```cpp
double nextInterArrivalTime(double now, IRandomProvider& rng) const;
void sampleTrip(double t, IRandomProvider& rng, int maxFloors, int& origin, int& destination) const;
std::string name() const;
```

`nextInterArrivalTime` returns the delta to the next arrival; `sampleTrip` the
origin and destination (distinct, in `0..maxFloors`). Keeping both on one object
lets a profile correlate *when* with *where*. `name()` is recorded in the
snapshot's metadata. Document how many random draws each method consumes.
`examples::PeakTrafficProfile` implements a time-varying rate by thinning, and its
subclasses are the published peaks.

The number of passengers is the `customers` argument, not the profile: a profile
shapes when and where they arrive. When a profile is injected, `arrivalRate` is
ignored.

### Passenger behaviour: `IPassengerBehaviorStrategy`

```cpp
bool willBoard(const Customer& customer, const BoardingContext& cabin) const;
double nextCallTime(const Customer& customer, const RefusalContext& refusal) const;
```

`willBoard` is asked only after the engine knows the passenger fits; `false`
means they decline a car they could have entered. Answer the same way however
often you are asked. `nextCallTime` is when a passenger left behind presses the
button again (clamped to be no earlier than the refusal); the default is the
instant the car that refused them departs. The new call is never given to the
cabin that refused them. `examples::LoadingThresholdPassengerBehavior` declines a
car that would then be more than 80% full (Peters 8.3.6).

### Stop duration: `IDwellTimeStrategy`

```cpp
double getDoorOpenTime(const StopContext& stop) const;
double getTransferTime(const StopContext& stop) const;
double getDoorCloseTime(const StopContext& stop) const;
DwellDescription describe() const;
```

`StopContext` carries the passengers boarding and alighting at the stop and
whether it serves a car call and/or a landing call. The transfer records of a
stop are dated at the end of the door opening; the cabin departs after the
transfer and the door closing. `describe()` is written to the snapshot. Presets
from the handbook are built by `makeDwellStrategy` (`DwellPresets.h`).

### Travel: `ITravelTimeStrategy`

`legDuration`, `displacement`, `remainingDuration`, `minimumLegDistance`,
`name`, `describe`. The contract that carries the weight is

```
displacement(d, legDuration(d)) == d    // exactly, not to within rounding
```

because event timestamps come from sums of leg durations while the cabin's
position comes from `displacement`, and the two must agree at a leg's end. See
`ITravelTimeStrategy.h` for the others; `LinearTravelTimeStrategy` is the
simplest valid implementation.

### Capacity: `ICapacityStrategy`

```cpp
double getBypassThreshold(double maxWeight) const;
```

The load at which a cabin stops taking new landing calls. Only meaningful with a
finite `maxWeightKg`.

### Metrics: `IMetricsCollector`

Hooks for every passenger and cabin event (`onCustomerArrived`,
`onCustomerBoarded`, `onCustomerAlighted`, `onCustomerPassedBy`,
`onHallCallReallocated`, `onElevatorDeparted`, `onSimulationEnd`) and the queries
the logger writes. Optional queries return `std::optional`: return `nullopt` for
what you do not measure, and the snapshot omits it rather than writing a zero.
`onElevatorDeparted` arrives one leg behind - see `IMetricsCollector.h`.

### Output: `ISimulationLogger`, and randomness: `IRandomProvider`

A logger receives the metadata, every customer and every timeline row, and writes
whatever it likes at the end. A random provider supplies exponential, uniform
integer and normal draws; pass a seeded `RandomProvider` for reproducible runs.

## Rules a new strategy can rely on, and rules it must keep

The engine guarantees:

- a landing call is never cancelled; every created passenger reaches their
  destination before the run ends;
- no cabin carries more than its rated load;
- nobody boards against the announced direction (under a sweeping route policy);
- the same seed and build produce the same snapshot.

A strategy must:

- leave the elevators it is handed unmodified (per-call dispatch);
- draw randomness only from the provider it is given (or one seeded from the run)
  and keep its own order deterministic - no iteration over hash containers in
  anything a result depends on;
- keep no per-cabin state in a shared policy;
- not assume that a cabin never reverses with passengers aboard (Closs rule 4 is
  not enforced by construction).

[architecture.md](architecture.md) explains why each of these holds.
