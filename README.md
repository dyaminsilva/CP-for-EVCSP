# EV charging scheduling

C++ implementation for *Is Constraint Programming Effective for Electric Vehicle Charging Scheduling Problems?* by Daniel Yamín and Willem-Jan van Hoeve. Given fixed vehicle routes, the models schedule charging to minimize makespan or total charging cost (electricity and peak demand).

The [112 baseline instances](E-VCSP-CGG-instances/README.txt) build on the multigraph routing benchmark of Yamín, Desaulniers, and Mendoza ([INFORMS Journal on Computing, 2025](https://doi.org/10.1287/ijoc.2023.0404)). The vehicle routes are fixed to those obtained in their adjustable-power study ([Les Cahiers du GERAD, G-2026-63](https://www.gerad.ca/en/papers/G-2026-63)).

## Build and run

Requires C++20 and IBM ILOG CPLEX Optimization Studio (CPLEX, CP Optimizer, and Concert). Run from the project root.

**Linux:** Set `studio` in `build_linux.sh` to your installation directory, then run:

```sh
bash build_linux.sh
./CP-for-EVCSP C101-25
./CP-for-EVCSP C101-25 CP costs
```

**Windows:** Open `CP-for-EVCSP.sln` in Visual Studio and build Release/x64. The project uses toolset v145 and defaults to Studio 22.1.1; override `CplexStudioRoot` and `CplexLibraryName` if needed. Run the executable from the project root, e.g. `.\x64\Release\CP-for-EVCSP.exe C101-25 CP costs`.

Usage: `CP-for-EVCSP <instance> [MILP|CP] [makespan|costs]`. The defaults are MILP and makespan. Each run prints the schedule if one is found and appends a row to `results-MILP.txt` or `results-CP.txt`. Makespan is the last occupied charging period.

The solver limits and cost parameters are in [cpp/CONSTANTS.h](cpp/CONSTANTS.h). The default limits are one hour, eight threads or workers, and a relative optimality gap of 0.01%.

## License

The source code and scripts are available under the [MIT License](LICENSE). This license does not cover the benchmark instances or IBM solver software.

The [results](results/) folder contains the baseline and strengthening experiment workbooks.
