#include "model/EVCSP.h"
#include "model/ChargingCP.h"
#include "model/ChargingMILP.h"

#include <exception>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    try {
        if (argc < 2 || argc > 4) {
            std::cerr << "Usage: CP-for-EVCSP <instance> [MILP|CP] [makespan|costs]\n"
                      << "Defaults: MILP, makespan\n"
                      << "Example: CP-for-EVCSP C101-25 CP costs\n";
            return 1;
        }
        const std::string solver = argc >= 3 ? argv[2] : "MILP";
        if (solver != "MILP" && solver != "CP") {
            std::cerr << "Solver must be MILP or CP.\n";
            return 1;
        }
        const std::string objective = argc >= 4 ? argv[3] : "makespan";
        if (objective != "makespan" && objective != "costs") {
            std::cerr << "Objective must be makespan or costs.\n";
            return 1;
        }
        model::checkResultsFile(solver);

        // reads the instance
        model::EVCSP instance(argv[1]);
        instance.printSummary(std::cout);

        // solves the instance using the selected solver
        const model::ChargingResult result = solver == "CP"
            ? model::solveCP(instance, objective) : model::solveMILP(instance, objective);
        model::printChargingSolution(std::cout, instance, result, solver, objective);
        model::appendChargingResult(instance, result, solver, objective);
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return 1;
    }
}
