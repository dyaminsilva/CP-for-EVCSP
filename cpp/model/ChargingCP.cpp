#include "ChargingCP.h"

#include <ilcp/cp.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

    // container for the CP variables for one vehicle
struct Intervals {
    int vehicleId; // vehicle ID
    IloIntervalVar I; // entire charging activity
    IloIntervalVarArray B; // charging blocks 
    std::vector<IloIntervalVarArray> J; // power alternatives for each block
};

IloNumExpr costObjective(IloEnv env, const model::EVCSP& data,
                        const std::vector<Intervals>& routes,
                        const std::vector<IloInt>& w, IloIntVar peak) {
    IloNumExpr cost(env); // time-of-use cost expression
    const double toKW = constants::PowerUnitsToKW / model::charging::PowerScale;
    const double hours = data.tau / 60.0;
    const int T = data.last_charging_period;

    // baseline time-of-use cost expression
    // Group consecutive equal tariffs; their values and boundaries come from data.c.
    for (int lo = 1; lo <= T;) {
        int hi = lo + 1;
        while (hi <= T && data.c[hi] == data.c[lo]) ++hi;
        for (const auto& route : routes) {
            for (const auto& J : route.J) {
                for (IloInt ell = 0; ell < J.getSize(); ++ell) {
                    cost += data.c[lo] * hours * toKW * w[ell]
                          * IloOverlapLength(env, J[ell], lo, hi, 0);
                }
            }
        }
        lo = hi;
    }
    cost += data.F * toKW * peak;
    return cost;
}

} // namespace

namespace model {

// Solve the CP model.
ChargingResult solveCP(const EVCSP& data, const std::string& objective, double timeLimit, bool log) {
    const bool costs = objective == "costs";
    const auto points = charging::pointBounds(data);
    const IloInt gridCapacity = charging::scaledGrid(data.maxPower);
    std::vector<IloInt> w;
    for (std::size_t ell = 1; ell < data.W.size(); ++ell) {
        const IloInt power = charging::scaledPower(data.W[ell]);
        // A mode that alone exceeds the grid capacity can never be used.
        if (power > gridCapacity) break;
        w.push_back(power);
    }

    ChargingResult result;
    // Necessary workload bounds strengthen the interval formulation without restricting schedules.
    IloInt totalRequirement = 0;
    IloInt totalDuration = 0;
    int earliest = data.last_charging_period;
    int lastEnd = 0;

    IloEnv env;
    try {
        IloModel model(env);
        IloCumulFunctionExpr chargerUsage(env); // charger usage constraints
        IloCumulFunctionExpr gridUsage(env); // grid usage constraints
        IloCumulFunctionExpr pointUsage(env); // discrete power points per period
        IloIntExprArray ends(env);
        std::vector<Intervals> routes;
        routes.reserve(data.vehicles.size());
        bool impossible = false;

        for (std::size_t rIndex = 0; rIndex < data.vehicles.size(); ++rIndex) { // for each vehicle
            const auto& vehicle = data.vehicles[rIndex];
            // an EV already at its target needs neither a charger nor an interval.
            if (vehicle.powerRequirement == 0.0) continue;
            const int first = vehicle.firstChargingPeriod;
            const int endMax = vehicle.lastChargingPeriod + 1;
            const int window = endMax - first;
            const IloInt requirement = charging::scaledRequirement(vehicle.powerRequirement);
            // check for infeasibility
            if (data.B == 0 || w.empty() || window <= 0 ||
                requirement > static_cast<IloInt>(window) * w.back()) {
                // Avoid invalid interval domains; let the engine certify the contradiction.
                model.add(IloFalseConstraint(env));
                impossible = true;
                result.explanation = "An EV cannot receive its required charge within its window and the available capacities.";
                break;
            }
            const IloInt duration = std::max<IloInt>(1, (requirement + w.back() - 1) / w.back());
            const int blockCount = w.size() == 1 ? 1 : std::min(data.N + 1, window);
            totalRequirement += requirement;
            totalDuration += duration;
            earliest = std::min(earliest, first);
            lastEnd = std::max(lastEnd, endMax);

            const std::string r = "r" + std::to_string(vehicle.id);
            routes.push_back({vehicle.id, IloIntervalVar(env), IloIntervalVarArray(env), {}});
            auto& route = routes.back();
            // define the interval variable for the route
            route.I.setName(("I_" + r).c_str());
            route.I.setStartMin(first);              // CP-f: code interval endpoints are shifted by one from the paper
            route.I.setStartMax(endMax - duration);
            route.I.setEndMax(endMax);               // CP-g: exclusive end of the last period
            route.I.setSizeMin(duration);            // Minimum charging duration for this route.
            route.I.setSizeMax(window);
            IloIntExpr delivered(env);
            IloIntExpr assignedPoints(env);

            for (int k = 0; k < blockCount; ++k) { // for each block
                const std::string rk = r + "_k" + std::to_string(k + 1);
                IloIntervalVar B(env, ("B_" + rk).c_str()); // block interval variable
                B.setOptional();
                B.setStartMin(first + k);            // preceding present blocks have positive size
                B.setEndMax(endMax);
                B.setSizeMin(1);                     // (1b), directly in the interval domain
                B.setSizeMax(window - k);
                route.B.add(B);
                route.J.emplace_back(env);
                auto& J = route.J.back();

                for (std::size_t ell = 0; ell < w.size(); ++ell) {
                    const std::string name = "J_" + rk + "_l" + std::to_string(ell + 1);
                    IloIntervalVar mode(env, name.c_str()); // the J interval variable for the power alternative
                    mode.setOptional();
                    // These time/size bounds are redundant through alternative; optionality is required.
                    mode.setStartMin(first + k);
                    mode.setEndMax(endMax);
                    mode.setSizeMin(1);
                    mode.setSizeMax(window - k);
                    J.add(mode);
                    gridUsage += IloPulse(mode, w[ell]); // for the grid usage constraints
                    delivered += w[ell] * IloSizeOf(mode, 0); // for the required energy constraints
                    const IloInt level = static_cast<IloInt>(ell + 1);
                    pointUsage += IloPulse(mode, level);
                    assignedPoints += level * IloSizeOf(mode, 0);
                }
                model.add(IloAlternative(env, B, J)); // CP-c (one power alternative per block)
            }
            model.add(IloSpan(env, route.I, route.B)); // CP-b (link the route interval to its blocks)
            for (int k = 0; k + 1 < blockCount; ++k) {
                model.add(IloPresenceOf(env, route.B[k + 1]) <= IloPresenceOf(env, route.B[k])); // CP-d (block k+1 can only be present if block k is present)
                // Native precedence is inactive if either interval is absent; CP-d supplies the prefix.
                model.add(IloEndAtStart(env, route.B[k], route.B[k + 1])); // CP-e (block k+1 starts exactly when block k ends)
                for (IloInt ell = 0; ell < route.J[k].getSize(); ++ell) {
                    model.add(IloPresenceOf(env, route.J[k][ell]) +
                              IloPresenceOf(env, route.J[k + 1][ell]) <= 1); // (1a)
                }
            }
            model.add(delivered >= requirement);     // CP-j (required energy) 
            model.add(assignedPoints >= static_cast<IloInt>(points.required[rIndex])); // (2b) point requirement
            chargerUsage += IloPulse(route.I, 1);
            ends.add(IloEndOf(route.I));
        }
        if (routes.empty() && !impossible) {
            result.status = ChargingStatus::Optimal;
            result.lowerBound = 0.0;
            env.end();
            return result;
        }
        if (!impossible) {
            const IloInt capacity = std::min(gridCapacity, static_cast<IloInt>(data.B) * w.back());
            const IloInt gridBound = (totalRequirement + capacity - 1) / capacity;
            const IloInt chargerBound = (totalDuration + data.B - 1) / data.B;
            const IloInt discreteBound = static_cast<IloInt>((points.total + points.capacity - 1) / points.capacity);
            const IloInt workloadBound = earliest + std::max({gridBound, chargerBound, discreteBound});
            if (!costs) result.lowerBound = static_cast<double>(workloadBound - 1);
            if (workloadBound > lastEnd) {
                model.add(IloFalseConstraint(env));
                result.explanation = "Total charging workload exceeds what the grid/chargers can deliver within the horizon.";
            }
            model.add(chargerUsage <= data.B);           // CP-h (charger capacity)
            model.add(gridUsage <= gridCapacity);        // CP-i (grid capacity)
            model.add(pointUsage <= static_cast<IloInt>(points.capacity)); // (2b) point capacity
            const IloIntExpr makespan = IloMax(ends);
            model.add(makespan >= workloadBound); // (4) makespan lower bound
            if (costs) {
                IloIntVar peak(env, 0, gridCapacity);
                model.add(gridUsage <= peak);            // CP-k
                IloNumExpr cost = costObjective(env, data, routes, w, peak);
                model.add(cost >= points.cost); // (3) cost lower bound
                model.add(IloMinimize(env, cost)); // CP-a
                result.lowerBound = points.cost;
            } else {
                model.add(IloMinimize(env, makespan));    // Section 4.2: makespan objective (replaces CP-a)
            }
        }

        IloCP cp(model);
        cp.setParameter(IloCP::TimeLimit, timeLimit);
        cp.setParameter(IloCP::Workers, constants::SolverThreads);
        cp.setParameter(IloCP::LogVerbosity, log ? IloCP::Terse : IloCP::Quiet);
        cp.setParameter(IloCP::RelativeOptimalityTolerance, constants::RelativeOptimalityGap);
        const bool found = cp.solve();
        result.solveTime = cp.getInfo(IloCP::SolveTime);
        result.branches = cp.getInfo(IloCP::NumberOfBranches);
        const auto status = cp.getStatus();
        if (status == IloAlgorithm::Optimal) result.status = ChargingStatus::Optimal;
        else if (found) result.status = ChargingStatus::Feasible;
        else if (status == IloAlgorithm::Infeasible) result.status = ChargingStatus::Infeasible;
        // Otherwise retain Unknown: a time limit is not a proof of infeasibility.
        // A time limit can interrupt extraction before the objective reaches the engine.
        // Keep the initial bound in that case; querying an absent objective throws.
        if (cp.hasObjective()) {
            const double bound = cp.getObjBound() - (costs ? 0.0 : 1.0);
            if (std::isfinite(bound)) result.lowerBound = std::max(result.lowerBound, bound);
        }
        if (result.hasSolution()) {
            for (const auto& route : routes) {
                for (IloInt k = 0; k < route.B.getSize(); ++k) {
                    if (!cp.isPresent(route.B[k])) continue;
                    for (IloInt ell = 0; ell < route.J[k].getSize(); ++ell) {
                        if (cp.isPresent(route.J[k][ell])) {
                            const int end = static_cast<int>(cp.getEnd(route.B[k]));
                            result.blocks.push_back({route.vehicleId, static_cast<int>(k) + 1,
                                static_cast<int>(ell) + 1, static_cast<int>(cp.getStart(route.B[k])),
                                end});
                            result.completionTime = std::max(result.completionTime, end);
                            break;
                        }
                    }
                }
            }
            result.objectiveValue = costs ? cp.getObjValue() : std::max(0, result.completionTime - 1);
            result.lowerBound = std::min(result.lowerBound, result.objectiveValue);
        }
        cp.end();
        env.end();
        return result;
    } catch (const IloException& ex) {
        const std::string message = ex.getMessage();
        env.end();
        throw std::runtime_error("CP Optimizer: " + message);
    } catch (...) {
        env.end();
        throw;
    }
}

} // namespace model
