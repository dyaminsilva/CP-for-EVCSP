#include "MIP.h"
#include "ChargingNetwork.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace model;

namespace {

// use the same block recovery at termination (just for printing the solution)
std::vector<ChargingBlock> readBlocks(const EVCSP& data, const std::vector<std::vector<IloExpr>>& powerLevel, IloCplex cplex) {
    std::vector<ChargingBlock> blocks;
    for (std::size_t r = 0; r < data.vehicles.size(); ++r) {
        const auto& vehicle = data.vehicles[r];
        if (vehicle.powerRequirement == 0) continue;
        int k = 0, previousLevel = 0;
        for (int t = vehicle.firstChargingPeriod; t <= vehicle.lastChargingPeriod; ++t) {
            const int level = static_cast<int>(std::llround(cplex.getValue(powerLevel[r][t])));
            if (level > 0) {
                if (level == previousLevel) blocks.back().end = t + 1;
                else blocks.push_back({vehicle.id, ++k, level, t, t + 1});
            }
            previousLevel = level;
        }
    }
    return blocks;
}

} // namespace

// solves the MIP
ChargingResult model::solveMIP(const EVCSP& data, const std::string& objective, double timeLimit,
                                bool log) {
    const bool costs = objective == "costs";
    const int horizon = data.last_charging_period;
    IloEnv env;
    try {
        ChargingNetwork network(env, data, objective); // creates the model
        ChargingResult result = network.result;
        if (result.hasSolution()) {
            env.end();
            return result;
        }
        IloCplex cplex(network.model);

	// Added for deterministic behavior
	cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Dual);
	cplex.setParam(IloCplex::Param::Parallel, 1); // explicitly deterministic

        cplex.setParam(IloCplex::Param::ClockType, 2); // elapsed wall-clock time
        cplex.setParam(IloCplex::Param::Threads, constants::SolverThreads);
        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, constants::RelativeOptimalityGap);
        if (!log) {
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());
        }
        if (timeLimit <= 0.0) { cplex.end(); env.end(); return result; }
        cplex.setParam(IloCplex::Param::TimeLimit, timeLimit);
        const double start = cplex.getCplexTime();
        const bool found = cplex.solve();
        result.solveTime = cplex.getCplexTime() - start;
        const auto status = cplex.getStatus();
        if (!found && status == IloAlgorithm::Infeasible) result.status = ChargingStatus::Infeasible;
        // a limit without an incumbent retains Unknown
        if (status != IloAlgorithm::Infeasible && cplex.isMIP()) {
            try {
                double bound = cplex.getBestObjValue();
                if (std::isfinite(bound) && bound >= 0.0 && (costs || bound <= horizon + 1e-6)) {
                    if (!costs && std::abs(bound - std::round(bound)) <= 1e-6) bound = std::round(bound);
                    result.lowerBound = std::max(result.lowerBound, bound);
                }
            } catch (const IloCplex::Exception& ex) {
                // A limit before the initial relaxation can leave no CPLEX solution information.
                if (ex.getStatus() != CPXERR_NO_SOLN) throw;
            }
        }
        if (found) {
            result.status = status == IloAlgorithm::Optimal ? ChargingStatus::Optimal : ChargingStatus::Feasible;
            // recovers charging blocks from the solution
            result.blocks = readBlocks(data, network.powerLevel, cplex);
            for (const auto& block : result.blocks)
                result.completionTime = std::max(result.completionTime, block.end);
            result.objectiveValue = costs ? cplex.getObjValue() : std::max(0, result.completionTime - 1);
            result.lowerBound = std::min(result.lowerBound, result.objectiveValue);
        }
        result.nodes = cplex.isMIP() ? cplex.getNnodes64() : 0;
        // sum the documented cut families: these are cuts in use, not all cuts generated.
        result.cutsInUse = 0;
        for (const auto type : {IloCplex::CutCover, IloCplex::CutGubCover, IloCplex::CutFlowCover,
             IloCplex::CutClique, IloCplex::CutFrac, IloCplex::CutMir, IloCplex::CutFlowPath,
             IloCplex::CutDisj, IloCplex::CutImplBd, IloCplex::CutZeroHalf, IloCplex::CutMCF,
             IloCplex::CutLiftProj, IloCplex::CutUser, IloCplex::CutTable, IloCplex::CutSolnPool,
             IloCplex::CutLocalImplBd, IloCplex::CutBQP, IloCplex::CutRLT, IloCplex::CutBenders}) {
             result.cutsInUse += cplex.getNcuts(type);
        }
		// close CPLEX and the environment
        cplex.end();
        env.end();
        return result;
    } catch (const IloException& ex) {
        const std::string message = ex.getMessage();
        env.end();
        throw std::runtime_error("CPLEX: " + message);
    } catch (...) {
        env.end();
        throw;
    }
}
