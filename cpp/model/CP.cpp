#include "CP.h"
#include "CostLP.h"
#include "CostPropagation.h"

#include <ilcp/cp.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>
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

// One representation is used for callbacks, final recovery and incoming schedules.
std::vector<model::ChargingBlock> readBlocks(IloCP cp, const std::vector<Intervals>& routes) {
    std::vector<model::ChargingBlock> blocks;
    for (const auto& route : routes) {
        for (IloInt k = 0; k < route.B.getSize(); ++k) {
            if (!cp.isPresent(route.B[k])) continue;
            for (IloInt ell = 0; ell < route.J[k].getSize(); ++ell) {
                if (cp.isPresent(route.J[k][ell])) {
                    blocks.push_back({route.vehicleId, static_cast<int>(k) + 1, static_cast<int>(ell) + 1,
                        static_cast<int>(cp.getStart(route.B[k])), static_cast<int>(cp.getEnd(route.B[k]))});
                    break;
                }
            }
        }
    }
    return blocks;
}

IloNumExpr costObjective(IloEnv env, const model::EVCSP& data,
                        const std::vector<Intervals>& routes,
                        const std::vector<IloInt>& w, IloIntVar peak) {
    IloNumExpr cost(env); // time-of-use cost expression
    const double toKW = constants::PowerUnitsToKW / constants::PowerScale;
    const double hours = data.tau / 60.0;
    const int T = data.last_charging_period;

    // baseline time-of-use cost expression
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

// solve the CP model
ChargingResult solveCP(const EVCSP& data, const std::string& objective, double timeLimit,
                       bool log, bool useCostPropagation) {
    const bool costs = objective == "costs";
    const bool strengthen = constants::Strengthen;
    const bool propagation = costs && strengthen && constants::CostPropagation && useCostPropagation;
    const auto points = strengthen ? charging::pointBounds(data) : charging::PointBounds{};
    const IloInt gridCapacity = data.maxPower;
    std::vector<IloInt> w;
    for (std::size_t ell = 1; ell < data.W.size(); ++ell) {
        const IloInt power = data.W[ell];
        // a mode that alone exceeds the grid capacity can never be used.
        if (power > gridCapacity) break;
        w.push_back(power);
    }

    ChargingResult result;
    CostLPBound lp;
    if (propagation) {
        lp = costLPBound(data, timeLimit); // solves the LP relaxation
		result.lowerBound = std::max(points.cost, lp.value); // the lower bound is given by the LP relaxation or the analytical bounds
        result.solveTime = lp.time;
        timeLimit = std::max(0.0, timeLimit - lp.time);
        if (log)
            std::cout << "Initial cost LP bound: " << result.lowerBound
                      << " (" << lp.time << " seconds of preprocessing)\n";
        if (timeLimit == 0.0) return result;
    }
    // necessary workload bounds strengthen the interval formulation without restricting schedules.
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
        IloCumulFunctionExpr mandatoryGridUsage(env); // equivalent profile with a mandatory baseline
        IloCumulFunctionExpr mandatoryPointUsage(env); // equivalent power-point profile
        IloIntExprArray ends(env);
        std::vector<Intervals> routes;
        routes.reserve(data.vehicles.size());
        bool impossible = false;

        for (std::size_t rIndex = 0; rIndex < data.vehicles.size(); ++rIndex) { // for each vehicle
            const auto& vehicle = data.vehicles[rIndex];
            // an EV already at its target needs neither a charger nor an interval.
            if (vehicle.powerRequirement == 0) continue;
            const int first = vehicle.firstChargingPeriod;
            const int endMax = vehicle.lastChargingPeriod + 1;
            const int window = endMax - first;
            const IloInt requirement = vehicle.powerRequirement;
            // check for infeasibility
            if (data.B == 0 || w.empty() || window <= 0 ||
                requirement > static_cast<IloInt>(window) * w.back()) {
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
            route.I.setSizeMin(duration);            // minimum charging duration for this route.
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
                    mode.setStartMin(first + k);
                    mode.setEndMax(endMax);
                    mode.setSizeMin(1);
                    mode.setSizeMax(window - k);
                    J.add(mode);
                    gridUsage += IloPulse(mode, w[ell]); // for the grid usage constraints
                    delivered += w[ell] * IloSizeOf(mode, 0); // for the required energy constraints
                    if (strengthen) {
                        const IloInt level = static_cast<IloInt>(ell + 1);
                        pointUsage += IloPulse(mode, level);
                        assignedPoints += level * IloSizeOf(mode, 0);
                    }
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

            // Strengthening the duration budget constraints
            // Shared by CP and CP+, for both cost and makespan objectives.
            const IloIntExpr chargingDuration = IloSizeOf(route.I);
            IloIntExpr blockDuration(env);
            IloIntExpr modeDuration(env);
            IloIntExpr aggregateEnergy(env);
            IloIntExpr energyDeficit(env);
            for (IloInt k = 0; k < route.B.getSize(); ++k)
                blockDuration += IloSizeOf(route.B[k], 0);
            for (std::size_t ell = 0; ell < w.size(); ++ell) {
                const std::string name = "h_" + r + "_l" + std::to_string(ell + 1);
                IloIntVar h(env, 0, window, name.c_str());
                IloIntExpr levelDuration(env);
                for (const auto& J : route.J)
                    levelDuration += IloSizeOf(J[ell], 0);
                model.add(h == levelDuration); // total time at this power, across all blocks
                modeDuration += h;
                aggregateEnergy += w[ell] * h;
                if (ell + 1 < w.size())
                    energyDeficit += (w.back() - w[ell]) * h;
            }
            model.add(chargingDuration == blockDuration);
            model.add(chargingDuration == modeDuration);
            // Keep the original energy constraint above; both aggregate forms share h.
            model.add(aggregateEnergy >= requirement);
            model.add(energyDeficit <= w.back() * chargingDuration - requirement);

            // Strengthen minimum grid consumption on the mandatory I_r intervals
            // Contiguous blocks cover I_r: baseline power plus the selected mode increments.
            mandatoryGridUsage += IloPulse(route.I, w.front());
            if (strengthen)
                mandatoryPointUsage += IloPulse(route.I, 1);
            for (const auto& J : route.J) {
                for (std::size_t ell = 1; ell < w.size(); ++ell) {
                    mandatoryGridUsage += IloPulse(J[ell], w[ell] - w.front());
                    if (strengthen)
                        mandatoryPointUsage += IloPulse(J[ell], static_cast<IloInt>(ell));
                }
            }

            if (strengthen)
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
            model.add(chargerUsage <= data.B);           // CP-h (charger capacity)
            model.add(gridUsage <= gridCapacity);        // CP-i (grid capacity)
            model.add(mandatoryGridUsage <= gridCapacity); // same capacity, using the mandatory baseline
            const IloIntExpr makespan = IloMax(ends);
            if (strengthen) {
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
                model.add(pointUsage <= static_cast<IloInt>(points.capacity)); // (2b) point capacity
                model.add(mandatoryPointUsage <= static_cast<IloInt>(points.capacity));
                model.add(makespan >= workloadBound); // (4) makespan lower bound
            }
            if (costs) {
                IloIntVar peak(env, 0, gridCapacity);
                model.add(gridUsage <= peak);            // CP-k
                model.add(mandatoryGridUsage <= peak);   // expose the baseline to peak-demand propagation too
                IloNumExpr cost = costObjective(env, data, routes, w, peak);
                IloNumExpr objectiveCost(env);
                objectiveCost += strengthen ? IloMax(cost, std::max(points.cost, result.lowerBound)) : cost;
				if (propagation) { // reduced cost-based propagation
                    const auto propagationStart = std::chrono::steady_clock::now(); // just the builiding time
                    std::vector<std::vector<IloIntervalVarArray>> modes;
                    for (const auto& route : routes) modes.push_back(route.J); // collects charging intervals that will be linked to the arc variables via (5)
                    // creates arc variables, path labels, and constraints (5) and (6);
                    // returns a variable representing the resulting cost bound
                    const IloNumExpr propagated = costPropagation(model, lp, modes);
                    const double propagationTime = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - propagationStart).count();
                    result.solveTime += propagationTime;
                    timeLimit = std::max(0.0, timeLimit - propagationTime);
                    objectiveCost = IloMax(objectiveCost, propagated); //
                }
                // cost objective
                model.add(IloMinimize(env, objectiveCost)); // CP-a
                result.lowerBound = std::max(result.lowerBound, points.cost);
            } else {
                model.add(IloMinimize(env, makespan));    // Section 4.2: makespan objective (replaces CP-a)
            }
        }

        if (timeLimit == 0.0) {
            env.end();
            return result;
        }
        IloCP cp(model);
        cp.setParameter(IloCP::Workers, constants::SolverThreads);
        cp.setParameter(IloCP::LogVerbosity, log ? IloCP::Terse : IloCP::Quiet);
        cp.setParameter(IloCP::RelativeOptimalityTolerance, constants::RelativeOptimalityGap);
        cp.setParameter(IloCP::TimeLimit, timeLimit);
        const bool found = cp.solve();
        result.solveTime += cp.getInfo(IloCP::SolveTime);
        result.branches = cp.getInfo(IloCP::NumberOfBranches);
        const auto status = cp.getStatus();
        if (status == IloAlgorithm::Optimal) result.status = ChargingStatus::Optimal;
        else if (found) result.status = ChargingStatus::Feasible;
        else if (status == IloAlgorithm::Infeasible) result.status = ChargingStatus::Infeasible;
        if (cp.hasObjective()) {
            // only makespan uses an exclusive end in CP.
            const double bound = cp.getObjBound() - (costs ? 0.0 : 1.0);
            if (std::isfinite(bound)) result.lowerBound = std::max(result.lowerBound, bound);
        }
        if (result.hasSolution()) { // recover the solution
            result.blocks = readBlocks(cp, routes);
            for (const auto& block : result.blocks)
                result.completionTime = std::max(result.completionTime, block.end);
            result.objectiveValue = costs ? cp.getObjValue() : std::max(0, result.completionTime - 1);
            result.lowerBound = std::min(result.lowerBound, result.objectiveValue);
        }
		// close CPLEX and the environment
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
