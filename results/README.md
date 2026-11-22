# Paper results

These six experiment sets contain 14 tab-separated TXT tables, each with 112 instances in the paper's order, and 1,568 solver logs. The runs used IBM ILOG CPLEX/CP Optimizer 22.1.0 on Linux, a 600-second limit, eight threads/workers, and a relative optimality tolerance of 0.01%. Recorded times can slightly exceed the limit. CP+ times include LP preprocessing.

| Directory | Objective | Tariffs | Maximum power adjustments | Methods |
| --- | --- | --- | --- | --- |
| `costs` | Cost | (1, 2, 3) | 3 | MILP, CP, CP+ |
| `costs-moderate` | Cost | (1.5, 2, 2.5) | 3 | MILP, CP, CP+ |
| `costs-flat` | Cost | (2, 2, 2) | 3 | MILP, CP |
| `makespan` | Makespan | Not applicable | 3 | MILP, CP |
| `makespan-adjustments2` | Makespan | Not applicable | 2 | MILP, CP |
| `makespan-adjustments4` | Makespan | Not applicable | 4 | MILP, CP |

All sets enable the additional valid inequalities and bounds. Both CP variants include the duration-budget and mandatory minimum-grid-consumption strengthenings. The avoidable-overcharging strengthening is excluded. CP+ denotes CP with LP-based cost propagation. There is no separate CP+ makespan experiment because its model is identical to CP for that objective.

## Files and ordering

`results-MILP.txt`, `results-CP.txt`, and, where applicable, `results-CP-propagation.txt` contain the result rows. MILP and CP logs are named `logs/<instance>-MILP.log` and `logs/<instance>-CP.log`. Baseline CP+ logs are in `costs/logs-propagation/<instance>.log`; moderate CP+ logs are in `costs-moderate/logs/<instance>-CP+.log`. Log names are preserved from the experiments.

The instance order is size 25 followed by size 50. Within each size it is `R101`–`R112`, `C101`–`C109`, `RC101`–`RC108`, `R201`–`R211`, `C201`–`C208`, and `RC201`–`RC208`. Instances are never sorted lexicographically across these groups.

## Reproducing the settings

Use the build and solver commands in the [main README](../README.md). The default source and `instances/` reproduce the baseline settings. For a tariff experiment, change `Tariffs` in `cpp/CONSTANTS.h` to the table's tuple and rebuild. The tariffs apply to successive thirds of the horizon, using boundaries `floor(T/3)` and `floor(2T/3)`. Keep `ReferenceTariff = 1` and `DemandChargeFactor = 0.5`: the demand-charge coefficient is `0.5 * (5T/60)` in all three tariff settings.

For the adjustment experiments, use a separate copy of `instances/` and change only `MAX_ADJUSTMENTS 3` to `MAX_ADJUSTMENTS 2` or `MAX_ADJUSTMENTS 4` in every instance. Run with the chosen copy as `instances/` in the working directory. Charging windows, vehicle requirements, chargers, and grid capacities remain those in the baseline files. Keep result files from different configurations separate: the executable appends rows to files in its working directory.

## Interpreting and aggregating results

- `Objective` is the incumbent cost or makespan; `LB` is the lower bound. Makespan is the last occupied charging period, and each period lasts five minutes. Printed schedule intervals use an exclusive end.
- `Optimal` means the solver met its optimality tolerance. `Feasible` means an incumbent was found without an optimality certificate. `Unknown` means no incumbent was found before termination; it does not mean infeasible. `NA` denotes an unavailable value.
- `Gap_pct` is a percentage. Objectives, bounds, gaps, and times are printed to three decimal places, so recomputing a gap from the displayed values can differ slightly from the printed gap.
- `Feas.` counts both `Optimal` and `Feasible` rows. `Opt.` counts `Optimal` rows. Summary time is the shifted geometric mean over all 112 recorded times, including timeouts: `exp(mean(log(1 + Time_s))) - 1`, with shift one second. Round only the final aggregate.
- `Nodes` and `Cuts_in_use` are MILP statistics; `Branches` is the CP/CP+ search statistic.

The published tables and logs are preserved byte for byte from the audited results. Workbooks, intermediate server splits, and the historical weakened-model results are not part of this release.
