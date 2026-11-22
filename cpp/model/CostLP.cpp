#include "CostLP.h"
#include "ChargingNetwork.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <stdexcept>

using namespace model;

namespace {

// Dualize every retained side constraint, including the strengthening rows.
// Flow remains in the route networks, whose unit paths already imply 0 <= y <= 1.
void exportCostNetwork(IloCplex cplex, IloExpr cost, IloRangeArray rows,
                       IloNumVarArray arcs, IloNumVar z, IloNumVar peak,
                       double optimum, CostLPBound& lp) {
    const IloEnv env = cplex.getEnv();
    IloNumArray dual(env);
	cplex.getDuals(dual, rows); // retrieve duals for the side constraints
    IloExpr modified(env);
    modified += cost;
    long double constant = 0.0;
    for (IloInt i = 0; i < rows.getSize(); ++i) {
		double multiplier = dual[i]; // retrieve the dual multiplier for this side constraint
        // avoid numerical noise and check sign conventions
        if (rows[i].getLB() <= -IloInfinity) multiplier = std::min(0.0, multiplier);
        if (rows[i].getUB() >= IloInfinity) multiplier = std::max(0.0, multiplier);
        if (multiplier == 0.0) continue;
        constant += static_cast<long double>(multiplier) *
			(multiplier > 0.0 ? rows[i].getLB() : rows[i].getUB()); // adds multiplier to the constant term
		modified -= multiplier * rows[i].getExpr(); // adds the multiplier to the modified cost expression
    }
    constant += modified.getConstant();
    std::map<IloInt, long double> coefficients; // modified coefficients of all variables
    for (auto term = modified.getLinearIterator(); term.ok(); ++term)
        coefficients[term.getVar().getId()] += term.getCoef();
    // The bounded peak and makespan auxiliaries are not part of a route path.
    for (const auto variable : {z, peak}) {
        const auto coefficient = coefficients[variable.getId()];
        constant += coefficient * (coefficient >= 0.0 ? variable.getLB() : variable.getUB());
    }
    // modified arc costs
    IloInt column = 0;
    for (auto& route : lp.routes) {
        for (auto& arc : route.arcs)
            arc.cost = static_cast<double>(coefficients[arcs[column++].getId()]);
    }
	// save the constant (rouunded down to avoid issues with numerical noise)
    lp.constant = static_cast<double>(constant) -
                  constants::Precision * std::max(1.0, std::abs(optimum));

	// close the temporary expressions
    modified.end();
    dual.end();
}

} // namespace

// computes LP bound and stores dual information (i.e., modified costs)
model::CostLPBound model::costLPBound(const EVCSP& data, double timeLimit) {
    const auto start = std::chrono::steady_clock::now();
    CostLPBound lp;
    IloEnv env;
    try {
		ChargingNetwork network(env, data, "costs", ILOFLOAT); // LP relaxation
        lp.value = network.result.lowerBound;
        if (!network.result.hasSolution() && timeLimit > 0.0) {
            IloCplex cplex(network.model);
            cplex.setParam(IloCplex::Param::ClockType, 2);
            cplex.setParam(IloCplex::Param::Threads, constants::SolverThreads);
            cplex.setParam(IloCplex::Param::TimeLimit, timeLimit);
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());
            cplex.solve();
            // Use only a proven LP optimum
            if (cplex.getCplexStatus() == IloCplex::Optimal) {
                const double value = cplex.getObjValue();
                const double margin = constants::Precision * std::max(1.0, std::abs(value));
                // Keep numerical noise in the LP bound out of the CP objective.
                const double bound = std::floor((value - margin) / constants::Precision)
                                   * constants::Precision;
                lp.value = std::max(lp.value, bound);
                for (const auto& route : network.routes) {
                    lp.routes.push_back({route.vehicleId, route.first, route.levels, route.sink, {}});
                    for (const auto& [from, to] : route.arcs)
                        lp.routes.back().arcs.push_back({from, to, 0.0});
                }
                // turns the LP multipliers into arc costs
                exportCostNetwork(cplex, network.cost, network.sideRows, network.arcVariables,
                                  network.z, network.peak, value, lp);
            }
            cplex.end();
        }
        env.end();
    } catch (const IloException& ex) {
        const std::string message = ex.getMessage();
        env.end();
        throw std::runtime_error("CPLEX: " + message);
    } catch (...) {
        env.end();
        throw;
    }
    // Include LP construction, extraction and solve time in the CP budget.
    lp.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return lp;
}
