#include "model/EVCSP.h"
#include "model/CP.h"
#include "model/MIP.h"

#include <exception>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    try {
        if (argc < 2 || argc > 4) {
            std::cerr << "Usage: CP-for-EVCSP <instance> [MILP|CP|CP+] [makespan|costs]\n"
                      << "Defaults: MILP, makespan\n"
                      << "CP: interval model; CP+: adds LP-based cost propagation for costs\n"
                      << "Example: CP-for-EVCSP C207-25 CP+ costs\n";
            return 1;
        }
        const std::string solver = argc >= 3 ? argv[2] : "MILP";
        if (solver != "MILP" && solver != "CP" && solver != "CP+") {
            std::cerr << "Solver must be MILP, CP or CP+.\n";
            return 1;
        }
        const std::string objective = argc >= 4 ? argv[3] : "makespan";
        if (objective != "makespan" && objective != "costs") {
            std::cerr << "Objective must be makespan or costs.\n";
            return 1;
        }

        // reads the instance
        model::EVCSP instance(argv[1]);
        instance.printSummary(std::cout);

        // solves the instance
        const model::ChargingResult result = solver == "MILP"
            ? model::solveMIP(instance, objective)
            : model::solveCP(instance, objective, constants::TimeLimitSeconds, true, solver == "CP+");
        model::printChargingSolution(std::cout, instance, result, solver, objective);
        model::appendChargingResult(instance, result, solver, objective);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }
}
