# EV charging scheduling

C++ implementation for *Constraint Programming for Electric Vehicle Charging Scheduling: Comparison and Integration with an Arc-Flow Mixed-Integer Program* by Daniel Yamín and Willem-Jan van Hoeve. Given fixed vehicle routes, the models schedule charging to minimize makespan or total charging cost (electricity and peak demand).

The [112 baseline instances](instances/README.txt) build on the multigraph routing benchmark of Yamín, Desaulniers, and Mendoza ([INFORMS Journal on Computing, 2025](https://doi.org/10.1287/ijoc.2023.0404)). The vehicle routes are fixed to those obtained in their adjustable-power study ([Les Cahiers du GERAD, G-2026-63](https://www.gerad.ca/en/papers/G-2026-63)).

## Methods

- **MIP:** arc-flow mixed-integer formulation.
- **CP:** interval-based constraint programming.
- **CP+:** the same CP model with additional LP-based cost propagation.

Additional valid inequalities and bounds are enabled by default.

## Build and run

Requires C++20 and IBM ILOG CPLEX Optimization Studio (CPLEX, CP Optimizer, and Concert). Run from the project root.

**Linux:** The default Studio directory is `/opt/ibm/ILOG/CPLEX_Studio221`. To use another installation, set `CPLEX_STUDIO_DIR` before building.

```sh
bash build_linux.sh
./CP-for-EVCSP C101-25 MILP costs
./CP-for-EVCSP C101-25 CP costs
./CP-for-EVCSP C101-25 CP+ costs
./CP-for-EVCSP C101-25 CP makespan
```

The command syntax is `CP-for-EVCSP <instance> [MILP|CP|CP+] [makespan|costs]`. Defaults are `MILP` and `makespan`. The instance name resolves to `instances/<instance>.txt`. Each run prints a schedule if one is found and appends a row to `results-MILP.txt`, `results-CP.txt`, or `results-CP-propagation.txt` in the working directory.

## Experiment settings and results

Solver parameters are defined in [cpp/CONSTANTS.h](cpp/CONSTANTS.h). Results from the paper are available in [results/](results/).

## License

The source code and scripts are available under the [MIT License](LICENSE).
