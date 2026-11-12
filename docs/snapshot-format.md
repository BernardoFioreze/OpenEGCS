# The snapshot format

Every run writes one JSON file (`snapshot.json` from the CLI) with everything
needed to reconstruct and re-measure it: the scenario and its physics, every
customer, the full event timeline, and the metrics. This page describes
**schemaVersion 11**. Times are in seconds, distances in floors unless a key says
otherwise, masses in kg, energy in joules.

```json
{
  "schemaVersion": 11,
  "metadata":  { ... },
  "customers": { "1": { ... }, "2": { ... } },
  "events":    [ { ... }, ... ],
  "metrics":   { ... }
}
```

## `metadata`

| Key | Meaning |
|---|---|
| `floors` | The highest floor; floors run `0..floors`. |
| `elevators` | Number of cabins. |
| `customersToSimulate` | Passengers created in the run (all of them reach their destination). |
| `arrivalRate` | Arrivals per second of the stationary default traffic; ignored by a peaked profile. |
| `trafficProfile` | `ITrafficProfile::name()`, e.g. `"stationary-uniform"`, `"lunch-peak (Cortes 2004)"`. |
| `sane` | Whether passengers press the button for their real direction. |
| `seed` | The seed, or `null` for a non-deterministic run. |
| `startingPositions` | Each cabin's floor at time 0. |
| `travelModel` | `"kinematic"` or `"linear"`. |
| `oneFloorLegSeconds` | What the travel model charges for one floor from rest - what a time unit means in this run. |
| `speed` | The `<Elevator speed>` CLI argument as given. Ignored by the kinematic model: read `oneFloorLegSeconds` instead. |
| `travel` | The travel model's parameters: `model`, and `floorHeightM`, `ratedSpeedMs`, `accelMs2`, `jerkMs3` (kinematic) or `secondsPerFloor` (linear). Absent keys are not zero. |
| `dwell` | The resolved stop duration: `model`, `doorOpenS`, `doorCloseS`, `perPassengerS`, `hold` (`"none"`, `"timed"`, `"detector"`), and `carCallHoldS`, `landingCallHoldS`, `detectorHoldS` when a hold applies. |
| `installation` | `ratedLoadKg` and `bypassThresholdKg` (`null` for an unlimited cabin), `meanPassengerKg`, `stddevPassengerKg`. |
| `energy` | The collector's energy model: `model` (e.g. `"dc-static-converter"`), floor height, motion profile, masses and motor parameters, `regenerative`, `residualInertiaKgM2`. `{"model": "none"}` for a collector that models no energy. |

Two snapshots are comparable on time only when `travel`, `dwell`,
`installation` and `trafficProfile` match, and on energy only when `energy`
matches too.

## `customers`

Keyed by customer id (as a string):

```json
"1": { "arrivalTime": 4.69, "atFloor": 7, "toFloor": 8, "direction": "UP" }
```

`arrivalTime` is the instant of their first press of the landing button.

## `events`

The timeline, in the order things happened. Every row has the same five keys:

```json
{ "time": 4.69, "type": "CUST_CALL", "elevatorId": 0, "customerId": 1, "floor": 7 }
```

| `type` | Meaning | `elevatorId` | `customerId` |
|---|---|---|---|
| `CUST_CALL` | A passenger arrives and presses the landing button. | 0 | the passenger |
| `RECALL` | A passenger left behind at an open door presses again. | the cabin that **refused** them | the passenger |
| `ARRIVE` | A cabin reaches a stop; dated when its doors have opened. | the cabin | 0 |
| `LEAVE` | A passenger alights. | the cabin | the passenger |
| `ENTER` | A passenger boards. | the cabin | the passenger |
| `PASS` | A passenger at the stop did not get in (no room, or they declined). | the cabin | the passenger |
| `DEPART` | A cabin starts moving towards its next stop; `floor` is where it leaves from. | the cabin | 0 |

How to read it:

- `LEAVE`, `ENTER` and `PASS` carry the time of their stop's `ARRIVE`. Inside
  one instant the order is physical: `ARRIVE`, the alightings, the boardings and
  refusals, then the `DEPART` of the leg the cabin leaves on.
- Each cabin's rows alternate `DEPART`, `ARRIVE`, `DEPART`, ... - starting with a
  `DEPART` (cabins start parked). `ARRIVE` is the complete list of a cabin's
  stops, including the rare stop at which nobody got in or out (a cabin already
  braking for a stop still makes it).
- Between an `ARRIVE` and the next `DEPART` the cabin is standing at the floor
  with its doors working; between a `DEPART` and the next `ARRIVE` it is moving.
  The motion profile is in `metadata.travel`.
- A `RECALL`'s `elevatorId` is the cabin that refused the passenger. Do not feed
  it into any cabin's trajectory.
- A passenger's waiting time is their first `ENTER` minus their `arrivalTime`;
  their journey time is their `LEAVE` minus their `arrivalTime`. The metrics below
  can be recomputed from the timeline this way, and the test suite does so.

## `metrics`

| Key | Meaning |
|---|---|
| `averageWaitingTime` | Mean of arrival to boarding, over every passenger. |
| `averageTravelTime` | Mean of boarding to alighting. |
| `totalEnergyConsumption` | Joules, from the energy model in `metadata.energy`. |
| `waitingTime`, `journeyTime` | Distributions (below): arrival to boarding, and arrival to alighting. |
| `waitingTimeByWindow`, `journeyTimeByWindow` | The same, split: `peak` (closed at or before the last arrival), `drain` (after it), and `boundaryTime` (the last arrival instant). |
| `queue` | `max` (most passengers waiting at once), `atArrivalsEnd` (waiting when arrivals ended), `boundaryTime`. |
| `passedBy` | `refusedAtDoor` (one event per `PASS` row) and `reallocated` (calls a full cabin handed back, counted per person waiting on them; no timeline row). Each is `{ "events": n, "passengers": [ids] }`. |

A distribution is:

```json
{ "count": 300, "average": 52.0, "p50": 43.6, "p90": 111.4, "max": 245.5,
  "threshold": 30.0, "dissatisfiedCount": 182, "dissatisfiedRate": 0.607,
  "dissatisfiedCustomers": [4, 6, 1, ...] }
```

Quantiles are nearest-rank, so every value is a time some passenger really
experienced. `threshold` is `null` when no tolerance was configured, and then the
three `dissatisfied*` keys are **absent**: "nobody was dissatisfied" and "nobody
measured" are different claims.

Read `peak` and `drain` together. `peak` alone excludes exactly the passengers
who waited through the peak and were served while the building drained, so it is
an optimistic floor.

A key that a collector does not measure is omitted, never written as zero.

## Versions

The schema only grows: a key, once written, keeps its meaning, so a reader written
for one version keeps working on later ones. `schemaVersion` says which keys to
expect.
