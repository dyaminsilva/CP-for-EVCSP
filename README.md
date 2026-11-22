# EV charging scheduling

C++ implementation for *Constraint Programming for Electric Vehicle Charging Scheduling: Comparison and Integration with an Arc-Flow Mixed-Integer Program* by Daniel Yamín and Willem-Jan van Hoeve. Given fixed vehicle routes, the models schedule charging to minimize makespan or total charging cost (electricity and peak demand).

The [112 baseline instances](instances/README.txt) build on the multigraph routing benchmark of Yamín, Desaulniers, and Mendoza ([INFORMS Journal on Computing, 2025](https://doi.org/10.1287/ijoc.2023.0404)). The vehicle routes are fixed to those obtained in their adjustable-power study ([Les Cahiers du GERAD, G-2026-63](https://www.gerad.ca/en/papers/G-2026-63)).

## Methods

- **MILP:** arc-flow mixed-integer formulation, with dual simplex at the root and deterministic parallel mode.
- **CP:** interval-based constraint programming, including the duration-budget and mandatory minimum-grid-consumption strengthenings.
- **CP+:** the same CP model with additional LP-based cost propagation.

CP and CP+ both retain the first two strengthenings; the third, excluding avoidable overcharging, is not included. CP+ adds propagation only for the cost objective, so it is identical to CP for makespan. Additional valid inequalities and bounds are enabled by default.

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

**macOS (Apple Silicon):** Run `bash build_macos.sh`. It defaults to `/Applications/CPLEX_Studio2211`; `CPLEX_STUDIO_DIR` overrides this path. Use the same executable commands as on Linux.

**Windows:** Open `CP-for-EVCSP.sln` in Visual Studio and build Release/x64. The project uses toolset v145 and defaults to Studio 22.1.1; override `CplexStudioRoot` and `CplexLibraryName` if needed. Run, for example:

```powershell
.\x64\Release\CP-for-EVCSP.exe C101-25 CP+ costs
```

The command syntax is `CP-for-EVCSP <instance> [MILP|CP|CP+] [makespan|costs]`. Defaults are `MILP` and `makespan`. The instance name resolves to `instances/<instance>.txt`. Each run prints a schedule if one is found and appends a row to `results-MILP.txt`, `results-CP.txt`, or `results-CP-propagation.txt` in the working directory. These generated files are separate from the published tables under `results/`. Keep outputs from different experiment settings in separate directories.

## Experiment settings and results

The defaults in [cpp/CONSTANTS.h](cpp/CONSTANTS.h) match the baseline experiment: **600 seconds, eight threads/workers, 0.01% relative optimality gap**, and tariffs `(1, 2, 3)`. CP+ includes LP preprocessing in its time budget. The baseline instances allow three power adjustments per vehicle.

The [published results](results/README.md) contain 14 tables and 1,568 solver logs for baseline costs and makespan, moderate and flat tariffs, and makespan with two or four adjustments. The results README describes the settings, file names, output fields, and aggregation formula. Excel workbooks are not included.

Each period lasts five minutes. Instance powers are in units of 0.6 kW. The 96 instances other than `RC*-50` use levels approximately `(3.60, 7.20, 10.79)` kW; the 16 `RC*-50` instances use approximately `(4.50, 8.99, 13.49)` kW. Use the exact values in the instance files for reproduction. The vehicle requirement is a sum of power over periods, already adjusted for the charging curve; it is not an energy value in kWh.

## License

The source code and scripts are available under the [MIT License](LICENSE).
