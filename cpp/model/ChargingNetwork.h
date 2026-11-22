#pragma once

#include "EVCSP.h"
#include "ChargingCommon.h"

#include <ilcplex/ilocplex.h>
#include <utility>

namespace model {

// solve the MILP, or its LP relaxation
struct ChargingNetwork {
    // tracks an EV network
    struct Route {
        int vehicleId, first, levels, sink;
        std::vector<std::pair<int, int>> arcs;
    };

    IloModel model;  // CPLEX model
	IloRangeArray sideRows; // side constraints for the MILP (different from network )
    IloNumVarArray arcVariables; // variables
    IloExpr cost; // charging cost
	IloNumVar z, peak; // peak power variable
	std::vector<std::vector<IloExpr>> powerLevel; // expressions for power level per vehicle and period (to recover the solution)
    std::vector<Route> routes; // list of charging network (one per EV)
    // tracks the solution
    ChargingResult result;

    // build the MILP model
    ChargingNetwork(IloEnv env, const EVCSP& data, const std::string& objective,
                    IloNumVar::Type arcType = ILOBOOL);
};

} // namespace model
