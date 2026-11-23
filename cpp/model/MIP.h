#pragma once

#include "EVCSP.h"
#include "ChargingCommon.h"

namespace model {

// MIP formulation for the EVCSP
ChargingResult solveMIP(const EVCSP& data,
                         const std::string& objective = "makespan",
                         double timeLimit = constants::TimeLimitSeconds,
                         bool log = true);

} // namespace model
