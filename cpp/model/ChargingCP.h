#pragma once

#include "EVCSP.h"
#include "ChargingCommon.h"

namespace model {

//CP formulation for the EVCSP
ChargingResult solveCP(const EVCSP& data,
                       const std::string& objective = "makespan",
                       double timeLimit = constants::TimeLimitSeconds,
                       bool log = true);

} // namespace model
