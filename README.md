# OpenEGCS / liftsim

A discrete-event simulator for benchmarking **elevator group control** (dispatch)
algorithms. Every policy that decides something - which cabin answers a call,
in which order a cabin serves its stops, what traffic arrives, how long a stop
or a trip takes, what a passenger does at an open door - is an interface you can
replace, and a run with a seed is reproducible byte for byte.

- **Pluggable control.** Per-call dispatch (`IDispatchStrategy`), group dispatch
  that can reassign unserved calls (`IGroupDispatchStrategy`), and route ordering
  inside a cabin (`IElevatorSchedulingStrategy`, LOOK by default).
- **Physics from the literature.** A jerk-limited kinematic travel model and an
  electromechanical energy model (Peters 1998), doors and passenger transfer
  times from Barney & Al-Sharif's *Elevator Traffic Handbook* (2015), cabin
  capacity with load-weighing bypass.
- **Traffic.** Stationary Poisson by default; up-peak, down-peak, lunch-peak and
  stepped-mix profiles taken from the papers they reproduce.
- **Measurement.** Waiting, travel and journey time as distributions (mean,
  p50, p90, worst case), split into the arrival window and the drain, the
  backlog, energy, and an optional dissatisfaction verdict - all recomputable
  from a JSON snapshot of the full event timeline.
- **Reference algorithms.** A naive greedy baseline, THV, GAHCA (Cortes et al.
  2004), Beamurgia et al.'s modified GA (2016), Peters' green strategy (1998),
  and FCFS route ordering, with the benches that run them.

All library types live in the `openEGCS` C++ namespace.

## Building

Requires a C++17 compiler, CMake 3.16 or later, and network access on the first
configure (Catch2 and nlohmann/json are fetched by CMake; nothing is vendored).

```bash
make build      # configure + build: ./build/liftsim, the benches, ./build/tests/liftsim_tests
make test       # build + run the full test suite
make clean      # remove build/ and snapshot.json
```

Or with CMake directly:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Run the suite **serially** (no `ctest -j`): test cases that run a full simulation
write `./snapshot.json` in the same directory. Running the Catch2 binary directly
is the fastest way, and accepts tag and name filters:

```bash
./build/tests/liftsim_tests
./build/tests/liftsim_tests "[capacity]"
```

## Running

With a JSON configuration file:

```bash
./build/liftsim --config scenario.json
```

```json
{ "floors": 10, "elevators": 3, "customers": 50, "arrivalRate": 0.8, "seed": 42 }
```

`floors`, `elevators` and `customers` are required; every other key has a
default, and an unknown key is refused by name. The keys are `arrivalRate`
(arrivals per second), `seed`, `maxWeightKg` (rated load; absent = unlimited),
`meanWeightKg`, `stddevWeightKg`, `bypassFraction`, `floorHeightM`, `carMassKg`,
`counterweightMassKg`, `doorType` (`"centre"`, `"side"`, `"none"`), `doorOpening`
(`"normal"`, `"advanced"`), `transferProfile` (`"office"`, `"residential"`,
`"ratedLoad"`), `doorHold` (`"timed"`, `"detector"`, `"none"`), `doorOpenTime`,
`doorCloseTime`, `perPassengerTransferTime`, `carCallHoldTime`,
`landingCallHoldTime`, `waitingThreshold`, `journeyThreshold`, `travelModel`
(`"kinematic"`, `"linear"`) and `elevatorSpeed` (linear model only).

Or with positional arguments (`./build/liftsim` with no arguments prints them all):

```bash
./build/liftsim 10 3 50 0.8 1.0 true 42
```

That is 10 floors, 3 cabins, 50 passengers, 0.8 arrivals per second, seed 42. The
`1.0` sits in the slot of the old elevator-speed argument, which the default
kinematic travel model ignores (the run prints a warning saying so); it is kept
so the arguments after it do not shift.

`<Customers to simulate>` is the exact number of passengers created, and a run
ends only when all of them have reached their destination. One simulation time
unit is one second. The command-line front end uses the greedy baseline dispatch;
other strategies are chosen through the library API.

Each run prints a summary and writes **`snapshot.json`**: the run's metadata
(including the physics it ran with), every customer, the full event timeline and
the metrics. See [docs/snapshot-format.md](docs/snapshot-format.md).

When a rated load is configured, the CLI warns if the demand exceeds the group's
up-peak handling capacity (Barney & Al-Sharif 2015, ch. 4): such a run spends a
long drain after the last arrival, and its global averages are dominated by it.

## Writing your own algorithm

Implement the interface and inject it. A dispatch strategy answers one question,
which cabin takes a landing call:

```cpp
#include <cmath>
#include "IDispatchStrategy.h"
#include "RandomProvider.h"
#include "Simulator.h"

using namespace openEGCS;

// Sends each landing call to the cabin physically closest to it,
// skipping cabins that cannot take a new landing call.
class NearestCabin : public IDispatchStrategy {
public:
    int selectElevator(Task task, std::vector<Elevator>& elevators) override {
        int best = 1;
        double bestDistance = 1e300;
        for (Elevator& e : elevators) {
            if (e.isBypassingCalls()) continue;
            double d = std::fabs(e.getPosition() - task.getDest());
            if (d < bestDistance) { bestDistance = d; best = e.getId(); }
        }
        return best;
    }
};

int main() {
    const uint64_t seed = 42;
    Simulator sim(1, "nearest", /*floors=*/10, /*elevators=*/3, /*customers=*/50,
                  /*arrival_rate=*/0.8, /*el_speed=*/1.0, /*sanity=*/true,
                  std::make_shared<RandomProvider>(seed),
                  std::make_shared<NearestCabin>(),
                  /*metrics_collector=*/nullptr, /*simulation_logger=*/nullptr, seed);
    sim.startSimulation();   // prints a summary and writes ./snapshot.json
}
```

Link it against `liftsim_core` (and `liftsim_examples` if you use the reference
strategies), for instance by adding to `CMakeLists.txt`:

```cmake
add_executable(nearest nearest.cpp)
target_link_libraries(nearest PRIVATE liftsim_core)
```

[docs/extending.md](docs/extending.md) covers every interface, what the engine
guarantees to each, and the contracts an implementation must keep.
[docs/architecture.md](docs/architecture.md) explains how the engine works.

## Reference implementations and benches

`examples/` holds the reference strategies, built as the `liftsim_examples`
library. None of them is a default of the engine.

| Directory | What it is |
|---|---|
| `greedy_nearest_dispatch` | Greedy baseline: the cabin with the smallest estimated finish time for the new call. Naive on purpose. |
| `thv_dispatch` | THV, the baseline of Cortes et al. (2004), Appendix A. |
| `gahca_dispatch` | GAHCA, the genetic algorithm of Cortes, Larrañaga & Onieva (2004), as a group strategy. |
| `beamurgia_dispatch` | The modified genetic algorithm of Beamurgia et al. (2016), with its adjustments as switches. |
| `peters_green_dispatch` | Green Strategy No. 2 of Peters (1998), as a decorator around any dispatcher. |
| `fcfs_scheduling` | First-come-first-served route ordering, the naive counterpart of LOOK. |
| `traffic_profiles` | Up-peak, down-peak (Barney & Al-Sharif), lunch-peak (Cortes 2004) and stepped-mix (Beamurgia 2016) traffic. |
| `loading_threshold_behavior` | Passengers who refuse a car that would be too full (Peters' loading threshold). |

Four benches run complete experiments and write one snapshot per run; an
`analyze.py` next to each reads them (Python 3, standard library only). The
header of each source file says what it reproduces and how to run it.

| Executable | Experiment |
|---|---|
| `gahca_bench` | GAHCA vs THV in Cortes et al.'s saturated lunch-peak. |
| `beamurgia_bench` | Beamurgia et al.'s scenarios under each GA variant. |
| `peters_bench` | Peters' lunch-time scenario with and without his green strategy. |
| `dispatch_bench` | Every shipped controller under one common environment. |

Create the output directory first: `peters_bench` creates its own, the other
three write nothing into a directory that does not exist.

```bash
mkdir -p /tmp/out
./build/gahca_bench /tmp/out gahca-crn 1 10
./build/gahca_bench /tmp/out thv 1 10
python3 -I examples/gahca_dispatch/bench/analyze.py /tmp/out
```

What each reproduction matches, and what is validated against the literature
and what is not, is in [docs/validation.md](docs/validation.md).

## Determinism

A run with a seed is reproducible: the same seed and the same build produce the
same `snapshot.json`, byte for byte, and the test suite checks it. Every random
draw comes from one injected `IRandomProvider`, in a fixed order. Changing the
model (event timing, traffic, a policy) changes the results for a given seed by
design; what holds is that two runs of the same build agree.

## Repository layout

| Path | Contents |
|---|---|
| `include/`, `src/` | The `liftsim_core` library and the CLI (`src/main.cpp`). Headers are flat. |
| `examples/` | Reference strategies (`liftsim_examples`) and the benches. |
| `tests/` | Catch2 test suite, including oracles that recompute every reported metric from the event timeline. |
| `docs/` | Architecture, extension guide, snapshot format, validation. |

## License and credits

OpenEGCS is a modified version of [liftsim](https://github.com/nivvedan/liftsim),
written in 2011 by Nivvedan and Ankush Das at IIT Bombay. It is free software,
distributed under the GNU General Public License, version 3 or later - see
[COPYING](COPYING), [NOTICE](NOTICE) and [AUTHORS](AUTHORS).

If you use it in academic work, please cite it - see [CITATION.cff](CITATION.cff).
