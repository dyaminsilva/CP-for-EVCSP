#pragma once

#include "CostLP.h"
#include <ilcp/cp.h>

namespace model {

// Modes are grouped by route, then charging block, in the LP network's order.
IloNumExpr costPropagation(IloModel model, const CostLPBound& lp,
                          const std::vector<std::vector<IloIntervalVarArray>>& modes);

} // namespace model
