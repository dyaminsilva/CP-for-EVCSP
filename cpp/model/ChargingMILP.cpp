#include "ChargingMILP.h"

#include <ilcplex/ilocplex.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

using namespace model;

namespace {


IloExpr costObjective(IloEnv env, const EVCSP& data,
                      const std::vector<IloExpr>& gridUsage, IloNumVar peak) {
    const double toKW = constants::PowerUnitsToKW / charging::PowerScale;
    const double hours = data.tau / 60.0;
    IloExpr cost(env);
    for (int t = 1; t <= data.last_charging_period; ++t)
        cost += data.c[t] * hours * toKW * gridUsage[t];
    cost += data.F * toKW * peak;
    return cost;
}

} // namespace

// solve the MILP
ChargingResult model::solveMILP(const EVCSP& data, const std::string& objective, double timeLimit, bool log) {
    const bool costs = objective == "costs";
    const auto points = charging::pointBounds(data);

    // tracks the solution
    ChargingResult result;
    const int horizon = data.last_charging_period;
    const int grid = charging::scaledGrid(data.maxPower);
    std::vector<int> powers(data.W.size(), 0);
    int levels = 0;
    for (std::size_t level = 1; level < data.W.size(); ++level) {
        powers[level] = charging::scaledPower(data.W[level]);
        if (powers[level] <= grid) ++levels;
    }
    const int maximum = powers[levels];
    int firstCompletionBound = 0;
    std::int64_t totalRequirement = 0;
    std::int64_t totalDuration = 0;
    int earliest = horizon;
    int lastEnd = 0;

    IloEnv env;
    try {
        // time-power expanded network
        IloModel model(env);
        IloNumVar z(env, 0.0, horizon, ILOFLOAT, "z"); // Section 4.1: for makespan minimization
		IloNumVar peak(env, 0.0, grid, ILOFLOAT, "peak"); // Nonnegative peak power for FRD charges
        std::vector<IloExpr> chargerUsage;  // charger usage constraints
        std::vector<IloExpr> gridUsage; // grid usage constraints
        std::vector<IloExpr> pointUsage; // discrete power points per period
        std::vector<std::vector<IloExpr>> powerLevel(data.vehicles.size());
        bool impossible = false;
		for (int t = 0; t <= horizon; ++t) { // one constraint per time period
            chargerUsage.emplace_back(env);
            gridUsage.emplace_back(env);
            pointUsage.emplace_back(env);
        }
		for (std::size_t r = 0; r < data.vehicles.size(); ++r) { // for each vehicle
            const auto& vehicle = data.vehicles[r];
            if (vehicle.powerRequirement == 0.0) continue;
			// recover vehicle-specific data
            const int first = vehicle.firstChargingPeriod;
            const int last = vehicle.lastChargingPeriod;
            const int requirement = charging::scaledRequirement(vehicle.powerRequirement);
            const std::int64_t window = static_cast<std::int64_t>(last) - first + 1;
            const std::int64_t minimumPeriods = maximum > 0
                ? std::max<std::int64_t>(1, (static_cast<std::int64_t>(requirement) + maximum - 1) / maximum)
                : window + 1;
			// quick check for infeasibility
            if (data.B == 0 || maximum == 0 || window <= 0 || minimumPeriods > window) {
                // Avoid an invalid network; let the engine certify the contradiction.
                IloExpr zero(env);
                model.add(zero >= 1);
                zero.end();
                impossible = true;
                result.explanation = "An EV cannot receive its required charge within its window and the available capacities.";
                break;
            }
            const int durationBound = static_cast<int>(minimumPeriods);
            firstCompletionBound = std::max(firstCompletionBound, first + durationBound);
            totalRequirement += requirement;
            totalDuration += minimumPeriods;
            earliest = std::min(earliest, first);
            lastEnd = std::max(lastEnd, last + 1);
			// number of selected source arcs, total energy delivered, number of adjustments, number of charging periods, and last charging period
            IloExpr source(env), energy(env), adjustments(env), duration(env), completion(env);
            std::vector<std::vector<IloExpr>> flow(last + 1); // outgoing flow minus incoming flow at node (t, ell)
            for (int t = 0; t <= last; ++t) powerLevel[r].emplace_back(env);
            for (int t = first; t <= last; ++t) {
                for (int ell = 0; ell <= levels; ++ell) flow[t].emplace_back(env);
            }

			// arc set A4: vehicle starts charging at time t with power level ell
            // The duration lower bound permits only these source and sink arcs.
            for (int t = first; t <= last - durationBound + 1; ++t) {
                for (int ell = 1; ell <= levels; ++ell) {
                    IloBoolVar y(env);                              // MILP-j
					source += y;  // for the one-start constraints
					flow[t][ell] -= y; // for the flow balance constraints
                }
            }

            // A1: charging ends after this occupied period.
            // MILP-d: use the sum of outgoing arcs directly for occupancy. Every outgoing time-node arc consumes its tail period, including A1.
            for (int t = first + durationBound - 1; t <= last; ++t) {
                for (int ell = 1; ell <= levels; ++ell) {
                    IloBoolVar y(env);                              // MILP-j
					flow[t][ell] += y;  //  for the flow balance constraints
					completion += t * y;    //  for the makespan minimization
					chargerUsage[t] += y;   // for the charger usage constraints
					gridUsage[t] += powers[ell] * y; // for the grid usage constraints
					energy += powers[ell] * y;  // for the energy requirement constraints
					duration += y; // for the duration requirement constraints
					powerLevel[r][t] += ell * y; // point count; also used to recover the schedule
                }
            }

            // A2/A3: continue charging, with an adjustment only when the level changes.
            for (int t = first; t < last; ++t) {
                for (int ell = 1; ell <= levels; ++ell) {
                    for (int next = 1; next <= levels; ++next) {
                        IloBoolVar y(env);                          // MILP-j
						flow[t][ell] += y;  // for the flow balance constraints
						flow[t + 1][next] -= y; // for the flow balance constraints
						if (ell != next) adjustments += y;         // A3 only; change in power level
						chargerUsage[t] += y; // for the charger usage constraints
						gridUsage[t] += powers[ell] * y; //     for the grid usage constraints
						energy += powers[ell] * y; // for the energy requirement constraints
						duration += y; // for the duration requirement constraints
						powerLevel[r][t] += ell * y; // point count; also used to recover the schedule
                    }
                }
            }

            model.add(source == 1);                                // MILP-b (select one schedule)
            IloExpr assignedPoints(env);
            for (int t = first; t <= last; ++t) {
                assignedPoints += powerLevel[r][t];
                pointUsage[t] += powerLevel[r][t];
                for (int ell = 1; ell <= levels; ++ell) {
					model.add(flow[t][ell] == 0);                   // MILP-c (flow balance)
                    flow[t][ell].end();
                }
            }
			model.add(energy >= requirement);                      // MILP-g (energy requirement)
            model.add(assignedPoints >= static_cast<IloNum>(points.required[r])); // (2a) point requirement
            assignedPoints.end();
			model.add(adjustments <= data.N);                      // MILP-h (maximum number of adjustments)
			model.add(duration >= durationBound);                  // lower bound on duration (minimum number of charging periods)
            model.add(completion <= z);                            // Section 4.1: makespan extension
            source.end();
            energy.end();
            adjustments.end();
            duration.end();
            completion.end();
        }
        if (firstCompletionBound == 0 && !impossible) {
            result.status = ChargingStatus::Optimal;
            env.end();
            return result;
        }
        int completionBound = 0;
        if (!impossible) {
            // same workload bound as CP; z is the last occupied period, not the exclusive end.
            const std::int64_t capacity = std::min<std::int64_t>(grid, static_cast<std::int64_t>(data.B) * maximum);
            const std::int64_t workloadBound = earliest + std::max({(totalRequirement + capacity - 1) / capacity,
                (totalDuration + data.B - 1) / data.B, (points.total + points.capacity - 1) / points.capacity});
            completionBound = std::max(firstCompletionBound, static_cast<int>(workloadBound));
            result.lowerBound = costs ? 0.0 : completionBound - 1;
            if (workloadBound > lastEnd) {
                IloExpr zero(env);
                model.add(zero >= 1);
                zero.end();
                result.explanation = "Total charging workload exceeds what the grid/chargers can deliver within the horizon.";
            } else {
                z.setLB(completionBound - 1); // (4) makespan lower bound
            }
        }
        for (int t = 1; t <= horizon; ++t) {
			model.add(chargerUsage[t] <= data.B);                   // MILP-e (charger usage)
			model.add(gridUsage[t] <= grid);                        // MILP-f (grid usage)
            model.add(pointUsage[t] <= static_cast<IloNum>(points.capacity)); // (2a) point capacity
            if (costs)
                model.add(gridUsage[t] <= peak);                   // MILP-i
        }
        if (costs) {
            IloExpr cost = costObjective(env, data, gridUsage, peak);
            model.add(cost >= points.cost);                        // (3) cost lower bound
            model.add(IloMinimize(env, cost));                      // MILP-a
            result.lowerBound = points.cost;
            cost.end();
        } else {
            model.add(IloMinimize(env, z));                         // Section 4.1: makespan objective
        }
        for (int t = 1; t <= horizon; ++t) {
            chargerUsage[t].end();
            gridUsage[t].end();
            pointUsage[t].end();
        }
        IloCplex cplex(model);
        cplex.setParam(IloCplex::Param::ClockType, 2); // elapsed wall-clock time
        cplex.setParam(IloCplex::Param::TimeLimit, timeLimit);
        cplex.setParam(IloCplex::Param::Threads, constants::SolverThreads);
        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, constants::RelativeOptimalityGap);
        if (!log) {
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());
        }
        const double start = cplex.getCplexTime();
        const bool found = cplex.solve();
        result.solveTime = cplex.getCplexTime() - start;
        result.nodes = cplex.isMIP() ? cplex.getNnodes64() : 0;
        // Sum the documented cut families: these are cuts in use, not all cuts generated.
        result.cutsInUse = 0;
        for (const auto type : {IloCplex::CutCover, IloCplex::CutGubCover, IloCplex::CutFlowCover,
             IloCplex::CutClique, IloCplex::CutFrac, IloCplex::CutMir, IloCplex::CutFlowPath,
             IloCplex::CutDisj, IloCplex::CutImplBd, IloCplex::CutZeroHalf, IloCplex::CutMCF,
             IloCplex::CutLiftProj, IloCplex::CutUser, IloCplex::CutTable, IloCplex::CutSolnPool,
             IloCplex::CutLocalImplBd, IloCplex::CutBQP, IloCplex::CutRLT, IloCplex::CutBenders}) {
            if (cplex.isMIP()) result.cutsInUse += cplex.getNcuts(type);
        }
        const auto status = cplex.getStatus();
        if (!found && status == IloAlgorithm::Infeasible) result.status = ChargingStatus::Infeasible;
        // A limit without an incumbent retains Unknown, never Infeasible.
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
            for (std::size_t r = 0; r < data.vehicles.size(); ++r) {
                const auto& vehicle = data.vehicles[r];
                if (vehicle.powerRequirement == 0.0) continue;
                int k = 0;
                int previousLevel = 0;
                for (int t = vehicle.firstChargingPeriod; t <= vehicle.lastChargingPeriod; ++t) {
                    const int level = static_cast<int>(std::llround(cplex.getValue(powerLevel[r][t])));
                    if (level > 0) {
                        if (level == previousLevel) result.blocks.back().end = t + 1;
                        else result.blocks.push_back({vehicle.id, ++k, level, t, t + 1});
                        result.completionTime = std::max(result.completionTime, t + 1);
                    }
                    previousLevel = level;
                }
            }
            result.objectiveValue = costs ? cplex.getObjValue() : std::max(0, result.completionTime - 1);
            result.lowerBound = std::min(result.lowerBound, result.objectiveValue);
        }
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
