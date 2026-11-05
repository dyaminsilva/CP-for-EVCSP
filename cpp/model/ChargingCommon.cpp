#include "ChargingCommon.h"
#include "EVCSP.h"

#include <fstream>
#include <iomanip>
#include <ostream>
#include <stdexcept>

namespace {

const char* statusName(model::ChargingStatus status) {
    using Status = model::ChargingStatus;
    switch (status) {
    case Status::Optimal: return "Optimal";
    case Status::Feasible: return "Feasible (optimality not proved)";
    case Status::Infeasible: return "Infeasible (proved)";
    default: return "Unknown (no solution found; infeasibility not proved)";
    }
}

double gapPercent(const model::ChargingResult& result) {
    if (result.objectiveValue <= 0.0) return 0.0;
    return 100.0 * std::max(0.0, result.objectiveValue - result.lowerBound) / result.objectiveValue;
}

} // namespace

model::charging::PointBounds model::charging::pointBounds(const EVCSP& data) {
    PointBounds bounds;
    bounds.required.resize(data.vehicles.size(), 0);
    const std::int64_t grid = scaledGrid(data.maxPower);
    std::int64_t upperPower = 0, upperLevel = 1, lowerPower = 0, lowerLevel = 1;
    int levels = 0;
    for (std::size_t ell = 1; ell < data.W.size(); ++ell) {
        const std::int64_t power = scaledPower(data.W[ell]);
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
        const std::int64_t requirement = scaledRequirement(data.vehicles[r].powerRequirement);
        bounds.required[r] = (requirement * upperLevel + upperPower - 1) / upperPower;
        bounds.total += bounds.required[r];
        totalRequirement += requirement;
    }
    const int T = data.last_charging_period;
    if (T > 0) {
        const double v = static_cast<double>(lowerPower) / lowerLevel;
        const double cMin = *std::min_element(data.c.begin() + 1, data.c.begin() + T + 1);
        const double toKW = constants::PowerUnitsToKW / PowerScale;
        // Minimum-tariff energy cost plus a lower bound on the demand charge.
        const double powerSumLB = std::max(static_cast<double>(totalRequirement), v * bounds.total);
        const double peakLB = std::max(static_cast<double>(totalRequirement) / T,
                                       v * ((bounds.total + T - 1) / T));
        bounds.cost = toKW * (data.tau / 60.0 * cMin * powerSumLB + data.F * peakLB);
    }
    return bounds;
}

void model::printChargingSolution(std::ostream& out, const EVCSP& data,
                                  const ChargingResult& result, const std::string& solver,
                                  const std::string& objective) {
    const auto flags = out.flags();
    const auto precision = out.precision();
    const bool costs = objective == "costs";
    out << '\n' << solver << " status: " << statusName(result.status)
        << "\nObjective: " << objective
        << '\n' << solver << " solve time: " << std::fixed << std::setprecision(3)
        << result.solveTime << " seconds\n";
    if (!result.explanation.empty()) out << result.explanation << '\n';
    if (result.status != ChargingStatus::Infeasible) {
        out << (costs ? "Cost bound: " : "Makespan bound: ") << result.lowerBound << '\n';
    }
    if (result.hasSolution()) {
        const int last = std::max(0, result.completionTime - 1);
        out << "Completion boundary: " << result.completionTime
            << "\nMakespan (last occupied period): " << last << '\n';
        if (costs) out << "Total charging cost: " << result.objectiveValue << '\n';
        out << "Relative gap: " << gapPercent(result) << "%\n";
        if (result.blocks.empty()) out << "No charging required.\n";
        else {
            out << " EV  Block  Level      Power  Start    End  Duration\n";
            for (const auto& block : result.blocks) {
                out << std::setw(3) << block.vehicleId << std::setw(7) << block.k
                    << std::setw(7) << block.powerIndex << std::setw(11) << std::setprecision(4)
                    << data.W[block.powerIndex] << std::setw(7) << block.start << std::setw(7) << block.end
                    << std::setw(10) << block.end - block.start << '\n';
            }
            out << "Block intervals are [Start, End), in charging periods.\n";
        }
    }
    out.flags(flags);
    out.precision(precision);
}

static std::string resultsHeader(const std::string& solver) {
    return std::string("Instance\tObjective_type\tPower_levels\tMax_adjustments\tObjective\tTime_s\tStatus\tLB\tGap_pct\t")
        + (solver == "MILP" ? "Nodes\tCuts_in_use" : "Branches");
}

void model::checkResultsFile(const std::string& solver) {
    const std::string filename = "results-" + solver + ".txt";
    std::ifstream input(filename);
    std::string header;
    if (std::getline(input, header) && header.substr(0, header.find('\r')) != resultsHeader(solver)) {
        throw std::runtime_error(filename + " has an older/different header. Move it aside or use a fresh working directory.");
    }
}

void model::appendChargingResult(const EVCSP& data, const ChargingResult& result,
                                 const std::string& solver, const std::string& objective) {
    checkResultsFile(solver);
    const bool milp = solver == "MILP";
    const std::string filename = "results-" + solver + ".txt";
    std::ofstream out(filename, std::ios::app | std::ios::ate);
    if (!out) throw std::runtime_error("Cannot open " + filename);
    if (out.tellp() == 0) {
        out << resultsHeader(solver) << '\n';
    }

    // Objective and bound share the same units in both models.
    out << std::fixed << std::setprecision(3) << data.instanceName << '\t' << objective << '\t'
        << data.W.size() - 1 << '\t' << data.N << '\t';
    if (result.hasSolution()) out << result.objectiveValue;
    else out << "NA";
    out << '\t' << result.solveTime << '\t';
    switch (result.status) {
    case ChargingStatus::Optimal: out << "Optimal"; break;
    case ChargingStatus::Feasible: out << "Feasible"; break;
    case ChargingStatus::Infeasible: out << "Infeasible"; break;
    default: out << "Unknown"; break;
    }
    out << '\t';
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
