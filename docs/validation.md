# Validation

What the simulator's models are checked against, what is not validated, and how
far each reproduction of a published result goes. A number that cannot be judged
by eye - a total energy, a mean waiting time - is only as good as what it was
checked against, so this page says exactly that, including the gaps.

## The models

### Kinematics

The jerk-limited motion profile (`LiftKinematics.h`) follows Peters (1998),
chapter 6, which the *Elevator Traffic Handbook* (Barney & Al-Sharif 2015,
appendix) draws its equations from. It is checked against the thesis's closed
forms - total journey time (eq. 6.36), cruise end (6.33), the Condition B range
(6.50), the displacement per slice (6.16/6.60, 6.20/6.64) - with a largest
relative residual of 1.3e-16. The short-run case (Condition C, eq. 6.81-6.85),
which the handbook leaves out, is implemented and checked for continuity with
Condition B; it is reached in the simulator because a leg retargeted mid-flight
can be shorter than a floor. Default dynamics by travel come from the handbook's
Table 5.2.

### Energy

The electromechanical model (`LiftMotorModel.h`) is Peters (1998), chapter 7.
Peters' worked example gives Et = 1.911e5 J; the model reproduces it within his
four-figure rounding (191 064 J, against a closed form of 191 062.5 J). That
example fixes the load torque and the inertia as inputs and integrates power
*signed*, so it validates eqs. 7.5-7.12, the integration, and the regenerative
case.

It does **not** validate:

- the derivation of load torque and inertia from the masses (eqs. 7.3/7.4),
  checked formula by formula and, end to end, only for order of magnitude;
- the installation masses (car 1000 kg, counterweight 1300 kg), which have no
  source - they are generic engineering figures;
- the non-regenerative drive the simulation uses by default (braking power
  dissipated, not returned): the same trip costs 2.575e5 J, 35% more;
- the lumped inertia constant (10 kg.m², borrowed from Peters' example, where it
  is the *total* inertia). With the default drive it is about 96% of the inertia,
  so a run's energy counts legs and starts more than load. Setting it to 0 keeps
  every controller on the same side of the greedy baseline but can reorder
  controllers on the same side.

Read energy comparisons with these in mind.

### Stop duration

Door opening and closing times, door widths, the minimum door hold and the
passenger transfer times are transcribed from the handbook (Table 5.3, sections
5.3.3.1, 5.5.1, 5.5.2, Table 5.7); a test compares every value with the table
cell it came from.

### Handling capacity

The up-peak handling capacity used by the CLI's saturation warning
(`TrafficCapacity.h`) is the handbook's round-trip-time calculation (ch. 4-5),
pinned to the book's worked examples; Siikonen (2000) gives the same figure.

### Traffic

The up-peak and down-peak profiles transcribe the handbook's Tables 17.2 and
17.4. The lunch-peak profile digitises Fig. 5 of Cortes et al. (2004). That
figure under-represents the traffic the paper simulated: at scale 1 it gives
about 394 trips in 130 minutes, too light to saturate two cars, while the paper
reports heavy congestion. Its absolute numbers are therefore not reproducible from
the figure; the GAHCA bench scales it by 3. The stepped-mix profile takes its
bounds and mix from Beamurgia et al. (2016); the shape of the step is not
published and is a declared substitution.

### The engine

The test suite recomputes every reported metric from the logged timeline and
requires the two to agree, checks invariants that need no expected value (every
passenger boards once and alights once, each cabin alternates `DEPART` and
`ARRIVE`, the distance charged for energy equals the distance travelled, no cabin
exceeds its rated load), and checks the timeline against independent sources (a
stop's duration against the dwell model, a leg's against the travel model) over a
grid of scenarios.

## Reproductions

Each bench rebuilds its paper's installation and runs the published comparison.
Measured with seeds 1 to 100, the same passengers for every controller of a seed:

| Experiment | Measure | Published | This simulator | Seeds with the sign |
|---|---|---|---|---|
| GAHCA vs THV (Cortes et al. 2004) | journey time | -23.3% | -26.73% (261.10 -> 191.31 s) | 100/100 |
| Beamurgia et al. 2016, C2, adjustments vs GA+S, R1 | waiting | -10.6% | -16.49% | 95/100 |
| same, R1' | waiting | -10.6% | -16.78% | 99/100 |
| same, installation at 10% handling capacity, R1 | waiting | -10.6% | -15.32% | 87/100 |
| same, 10% handling capacity, R1' | waiting | -10.6% | -17.77% | 90/100 |
| Peters 1998, green strategy vs greedy | departures | -3.2% | -1.13% ± 0.11 | 83/100 |
| same | energy | -6.2% | -1.22% ± 0.14 | 81/100 |

How to read it:

- **GAHCA vs THV** reproduces the paper's comparison in sign and in size. The
  absolute times do not match the paper's (see the lunch-peak caveat above).
- **Beamurgia** reproduces in sign, and the effect is larger than published. The
  paper does not publish its installation's physics (only "the default ones in
  Elevate"), so the bench substitutes the handbook's; with those the installation
  has 13.8% handling capacity against the paper's stated 10%, which the "10%"
  rows correct by using another row of the same handbook table. The gain comes
  from the LBI and Sd adjustments together (-18.06% alone, 97/100). Measured with
  a modified bench, a larger GA search (40 individuals x 40 iterations instead of
  the paper's 10 x 10) makes the advantage nearly disappear (-0.8%): the
  adjustments compensate for a small search.
- **Peters' green strategy** reproduces in sign and is much smaller than
  published. The thesis does not publish the demand or the traffic mix of the
  experiment; they are bench arguments.

None of the parameters was tuned to the results. Each bench's header lists its
scenario and every substitution.

### Reproducing

Build the project, then for each seed `S`:

```bash
mkdir -p out/gahca out/beam out/peters
./build/gahca_bench out/gahca thv $S $S
./build/gahca_bench out/gahca gahca-crn $S $S
for v in S S+LBI+Sd+P3; do for r in R1 R1p; do
  ./build/beamurgia_bench out/beam C2-inter $v $r $S $S
  ./build/beamurgia_bench out/beam C2-inter-hc10 $v $r $S $S
done; done
for rule in plain green; do
  ./build/peters_bench out/peters greedy $rule 4 0.3 tese $S $S
done
```

and read the results with each bench's `analyze.py`. `dispatch_bench` compares
every shipped controller under one environment; `dispatch_bench --plan` prints the
scenarios it derives from the handling capacity.
