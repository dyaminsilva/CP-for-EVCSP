#include "ChargingCommon.h"
#include "EVCSP.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <stdexcept>

namespace {

 // possible termination statuses
const char* statusName(model::ChargingStatus status) {
    using Status = model::ChargingStatus;
    switch (status) {
    case Status::Optimal: return "Optimal";
    case Status::Feasible: return "Feasible";
    case Status::Infeasible: return "Infeasible";
    default: return "Unknown";
    }
}

// computes gap as a percentage
double gapPercent(const model::ChargingResult& result) {
    if (result.objectiveValue <= 0.0) return 0.0;
    return 100.0 * std::max(0.0, result.objectiveValue - result.lowerBound) / result.objectiveValue;
}

} // namespace

// computes the bounds as Section 4.3
model::charging::PointBounds model::charging::pointBounds(const EVCSP& data) {
    PointBounds bounds;
    bounds.required.resize(data.vehicles.size(), 0);
    const std::int64_t grid = data.maxPower;
    std::int64_t upperPower = 0, upperLevel = 1, lowerPower = 0, lowerLevel = 1;
    int levels = 0;
    for (std::size_t ell = 1; ell < data.W.size(); ++ell) {
        const std::int64_t power = data.W[ell];
        if (power > grid) break;
        const auto level = static_cast<std::int64_t>(ell);
        // u = max(w_l/l), v = min(w_l/l); compare ratios without floating-point rounding.
        if (levels == 0 || power * upperLevel > upperPower * level) {
            upperPower = power;
            upperLevel = level;
        }
        if (levels == 0 || power * lowerLevel < lowerPower * level) {
            lowerPower = power;
            lowerLevel = level;
        }
        ++levels;
    }
    if (levels == 0 || data.B == 0) return bounds; // existing model checks handle infeasibility
    bounds.capacity = std::min(static_cast<std::int64_t>(data.B) * levels,
                               grid * lowerLevel / lowerPower);
    std::int64_t totalRequirement = 0;
    for (std::size_t r = 0; r < data.vehicles.size(); ++r) {
        const std::int64_t requirement = data.vehicles[r].powerRequirement;
        bounds.required[r] = (requirement * upperLevel + upperPower - 1) / upperPower;
        bounds.total += bounds.required[r];
        totalRequirement += requirement;
    }
    const int T = data.last_charging_period;
    if (T > 0) {
        const double v = static_cast<double>(lowerPower) / lowerLevel;
        const double toKW = constants::PowerUnitsToKW / constants::PowerScale;
        // Price the required power sum in the cheapest periods first, up to the grid limit in each.
        std::vector<double> tariffs(data.c.begin() + 1, data.c.end());
        std::sort(tariffs.begin(), tariffs.end());
        double remaining = std::max(static_cast<double>(totalRequirement), v * bounds.total);
        double energyCost = 0.0;
        for (double tariff : tariffs) {
            const double power = std::min(remaining, static_cast<double>(grid));
            energyCost += tariff * power;
            remaining -= power;
            if (remaining == 0.0) break;
        }
        const double peakLB = std::max(static_cast<double>(totalRequirement) / T,
                                       v * ((bounds.total + T - 1) / T));
        bounds.cost = toKW * (data.tau / 60.0 * energyCost + data.F * peakLB);
    }
    return bounds;
}

// prints the solution in console
void model::printChargingSolution(std::ostream& out, const EVCSP& data,
                                  const ChargingResult& result, const std::string& solver,
                                  const std::string& objective) {
    out << std::fixed << std::setprecision(2)
        << '\n' << solver << " status: " << statusName(result.status)
        << "\nObjective: " << objective
        << "\nSolve time: " << result.solveTime << " seconds\n";

    if (!result.explanation.empty())
        out << result.explanation << '\n';

    if (result.status != ChargingStatus::Infeasible)
        out << "Lower bound: " << result.lowerBound << '\n';

    if (!result.hasSolution()) return;

    out << "Objective value: " << result.objectiveValue
        << "\nRelative gap: " << gapPercent(result) << "%\n";

    if (result.blocks.empty()) {
        out << "No charging required.\n";
        return;
    }

    out << "EV\tBlock\tLevel\tPower\tStart\tEnd\tDuration\n";
    for (const auto& block : result.blocks) {
        out << block.vehicleId << '\t' << block.k << '\t'
            << block.powerIndex << '\t' << data.W[block.powerIndex] << '\t'
            << block.start << '\t' << block.end << '\t'
            << block.end - block.start << '\n';
    }
    out << "Block intervals are [Start, End), in charging periods.\n";
}

// writes the results file header
static std::string resultsHeader(const std::string& solver) {
    std::string header = "Instance\tObjective_type\tEVs\tCharging_horizon\tObjective\tTime_s\tStatus\tLB\tGap_pct\t";
    return header + (solver == "MILP" ? "Nodes\tCuts_in_use" : "Branches");
}

void model::appendChargingResult(const EVCSP& data, const ChargingResult& result,
                                 const std::string& solver, const std::string& objective) {
    const bool milp = solver == "MILP";
    const bool propagation = solver == "CP+" && objective == "costs" &&
                             constants::Strengthen && constants::CostPropagation;
    const std::string filename = std::string("results-") + (milp ? "MILP" : "CP")
                               + (propagation ? "-propagation.txt" : ".txt");
    std::ofstream out(filename, std::ios::app | std::ios::ate);
    if (!out) throw std::runtime_error("Cannot open " + filename);
    if (out.tellp() == 0) {
        out << resultsHeader(solver) << '\n';
    }

    // Objective and bound share the same units in both models.
    out << std::fixed << std::setprecision(3) << data.instanceName << '\t' << objective << '\t'
        << data.vehicles.size() << '\t' << data.last_charging_period << '\t';
    if (result.hasSolution()) out << result.objectiveValue;
    else out << "NA";
    out << '\t' << result.solveTime << '\t'
        << statusName(result.status) << '\t';
    if (result.status == ChargingStatus::Infeasible) out << "NA";
    else out << result.lowerBound;
    out << '\t';
    if (result.hasSolution()) out << gapPercent(result);
    else out << "NA";

    if (milp) {
        out << '\t' << (result.nodes < 0 ? "NA" : std::to_string(result.nodes))
            << '\t' << (result.cutsInUse < 0 ? "NA" : std::to_string(result.cutsInUse));
    } else {
        out << '\t' << (result.branches < 0 ? "NA" : std::to_string(result.branches));
    }
    out << '\n';
    out.close();
    if (!out) throw std::runtime_error("Cannot write " + filename);
}
