#pragma once

#include "EVCSP.h"
#include "ChargingCommon.h"

namespace model {

// MILP formulation for the EVCSP
ChargingResult solveMILP(const EVCSP& data,
                         const std::string& objective = "makespan",
                         double timeLimit = constants::TimeLimitSeconds,
                         bool log = true);

} // namespace model
