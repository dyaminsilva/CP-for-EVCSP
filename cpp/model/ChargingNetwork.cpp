#include "ChargingNetwork.h"

#include <algorithm>
#include <cstdint>

using namespace model;

namespace {

IloExpr costObjective(IloEnv env, const EVCSP& data,
                      const std::vector<IloExpr>& gridUsage, IloNumVar peak) {
    const double toKW = constants::PowerUnitsToKW / constants::PowerScale;
    const double hours = data.tau / 60.0;
    IloExpr cost(env);
    for (int t = 1; t <= data.last_charging_period; ++t)
        cost += data.c[t] * hours * toKW * gridUsage[t];
    cost += data.F * toKW * peak;
    return cost;
}

} // namespace

ChargingNetwork::ChargingNetwork(IloEnv env, const EVCSP& data, const std::string& objective,
                                 IloNumVar::Type arcType)
    : model(env), sideRows(env), arcVariables(env), cost(env), powerLevel(data.vehicles.size()) {
    const bool costs = objective == "costs";
    const bool strengthen = constants::Strengthen;
    const auto points = strengthen ? charging::pointBounds(data) : charging::PointBounds{};

    const int horizon = data.last_charging_period;
    const int grid = data.maxPower;
    const auto& powers = data.W;
    int levels = 0;
    for (std::size_t level = 1; level < data.W.size(); ++level) {
        if (powers[level] <= grid) ++levels;
    }
    const int maximum = powers[levels];
    int firstCompletionBound = 0;
    std::int64_t totalRequirement = 0;
    std::int64_t totalDuration = 0;
    int earliest = horizon;
    int lastEnd = 0;

    // time-power expanded network
    const auto addSide = [&](IloRange row) {
        model.add(row);
        sideRows.add(row);
    };
    z = IloNumVar(env, 0.0, horizon, ILOFLOAT, "z"); // Section 4.1: for makespan minimization
    peak = IloNumVar(env, 0.0, grid, ILOFLOAT, "peak"); // Nonnegative peak power for FRD charges
    std::vector<IloExpr> chargerUsage;  // charger usage constraints
    std::vector<IloExpr> gridUsage; // grid usage constraints
    std::vector<IloExpr> pointUsage; // discrete power points per period
    bool impossible = false;
    for (int t = 0; t <= horizon; ++t) { // one constraint per time period
        chargerUsage.emplace_back(env);
        gridUsage.emplace_back(env);
        pointUsage.emplace_back(env);
    }
    for (std::size_t r = 0; r < data.vehicles.size(); ++r) { // for each vehicle
        const auto& vehicle = data.vehicles[r];
        if (vehicle.powerRequirement == 0) continue;
        // recover vehicle-specific data
        const int first = vehicle.firstChargingPeriod;
        const int last = vehicle.lastChargingPeriod;
        const int requirement = vehicle.powerRequirement;
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
        const int sink = 1 + static_cast<int>(window) * levels;
        const auto node = [=](int t, int ell) { return 1 + (t - first) * levels + ell - 1; };
        if (arcType == ILOFLOAT) routes.push_back({vehicle.id, first, levels, sink, {}});
        const auto recordArc = [&](IloNumVar y, int from, int to) {
            if (arcType == ILOFLOAT) {
                arcVariables.add(y);
                routes.back().arcs.emplace_back(from, to);
            }
        };
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
                IloNumVar y(env, 0.0, 1.0, arcType);             // MILP-j (continuous in the LP)
                recordArc(y, 0, node(t, ell));
                source += y;  // for the one-start constraints
                flow[t][ell] -= y; // for the flow balance constraints
            }
        }

        // A1: charging ends after this occupied period.
        // MILP-d: use the sum of outgoing arcs directly for occupancy. Every outgoing time-node arc consumes its tail period, including A1.
        for (int t = first + durationBound - 1; t <= last; ++t) {
            for (int ell = 1; ell <= levels; ++ell) {
                IloNumVar y(env, 0.0, 1.0, arcType);             // MILP-j (continuous in the LP)
                recordArc(y, node(t, ell), sink);
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
                    IloNumVar y(env, 0.0, 1.0, arcType);         // MILP-j (continuous in the LP)
                    recordArc(y, node(t, ell), node(t + 1, next));
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
        for (int t = first; t <= last; ++t) {
            for (int ell = 1; ell <= levels; ++ell) {
                model.add(flow[t][ell] == 0);                   // MILP-c (flow balance)
                flow[t][ell].end();
            }
        }
        addSide(energy >= requirement);                        // MILP-g (energy requirement)
        if (strengthen) {
            IloExpr assignedPoints(env);
            for (int t = first; t <= last; ++t) {
                assignedPoints += powerLevel[r][t];
                pointUsage[t] += powerLevel[r][t];
            }
            addSide(assignedPoints >= static_cast<IloNum>(points.required[r])); // (2a) point requirement
            assignedPoints.end();
        }
        addSide(adjustments <= data.N);                        // MILP-h (maximum number of adjustments)
        addSide(duration >= durationBound);                    // lower bound on duration (minimum number of charging periods)
        addSide(completion - z <= 0);                          // Section 4.1: makespan extension
        source.end();
        energy.end();
        adjustments.end();
        duration.end();
        completion.end();
    }
    if (firstCompletionBound == 0 && !impossible) {
        result.status = ChargingStatus::Optimal;
        return;
    }
    if (strengthen && !impossible) {
        // same workload bound as CP; z is the last occupied period, not the exclusive end.
        const std::int64_t capacity = std::min<std::int64_t>(grid, static_cast<std::int64_t>(data.B) * maximum);
        const std::int64_t workloadBound = earliest + std::max({(totalRequirement + capacity - 1) / capacity,
            (totalDuration + data.B - 1) / data.B, (points.total + points.capacity - 1) / points.capacity});
        const int completionBound = std::max(firstCompletionBound, static_cast<int>(workloadBound));
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
        addSide(chargerUsage[t] <= data.B);                     // MILP-e (charger usage)
        addSide(gridUsage[t] <= grid);                          // MILP-f (grid usage)
        if (strengthen)
            addSide(pointUsage[t] <= static_cast<IloNum>(points.capacity)); // (2a) point capacity
        if (costs)
            addSide(gridUsage[t] - peak <= 0);                 // MILP-i
    }
    if (costs) {
        IloExpr expression = costObjective(env, data, gridUsage, peak);
        if (strengthen) addSide(expression >= points.cost);                          // (3) cost lower bound
        model.add(IloMinimize(env, expression));                      // MILP-a
        cost += expression;
        result.lowerBound = points.cost;
        expression.end();
    } else {
        model.add(IloMinimize(env, z));                         // Section 4.1: makespan objective
    }
    for (int t = 1; t <= horizon; ++t) {
        chargerUsage[t].end();
        gridUsage[t].end();
        pointUsage[t].end();
    }
}
