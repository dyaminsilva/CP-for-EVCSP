#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace model {

class EVCSP;

enum class ChargingStatus { Optimal, Feasible, Infeasible, Unknown }; // solution status

// tracks a charging block
struct ChargingBlock {
    int vehicleId;
    int k;                         // block number, starting at 1
    int powerIndex;                // original index in EVCSP::W
    int start;
    int end;                       // exclusive; occupied periods are start..end-1
};

// stores the result of a charging solver
struct ChargingResult {
    ChargingStatus status = ChargingStatus::Unknown;
    int completionTime = 0;        // exclusive completion boundary; 0 if no charging is needed
    double objectiveValue = 0.0;   // makespan or total cost
    double lowerBound = 0.0;       // same units as objectiveValue
    double solveTime = 0.0;
    // solver statistics; -1 means unavailable (for example, a return before solve).
    std::int64_t nodes = -1;
    std::int64_t cutsInUse = -1;
    std::int64_t branches = -1;
    std::string explanation;
    std::vector<ChargingBlock> blocks;

    bool hasSolution() const {
        return status == ChargingStatus::Optimal || status == ChargingStatus::Feasible;
    }
};

// prints a solution
void printChargingSolution(std::ostream& out, const EVCSP& data,
                           const ChargingResult& result, const std::string& solver,
                           const std::string& objective = "makespan");

// appends one row to the selected solver's results file in the working directory.
void appendChargingResult(const EVCSP& data, const ChargingResult& result,
                          const std::string& solver, const std::string& objective = "makespan");

namespace charging {

// Integer points per route and per period, shared by both formulations.
struct PointBounds {
    std::vector<std::int64_t> required;
    std::int64_t total = 0;
    std::int64_t capacity = 0;
    double cost = 0.0;
};

PointBounds pointBounds(const EVCSP& data);

} // namespace charging
} // namespace model
