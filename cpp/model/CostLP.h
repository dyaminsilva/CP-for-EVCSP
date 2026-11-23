#pragma once

#include "EVCSP.h"

namespace model {

	// an arc with its modified cost (from the dualized LP)
struct CostArc {
    int from, to;
    double cost; // modified cost with fixed LP multipliers.
};

 // EV charging network
struct CostRoute {
    int vehicleId, first, levels, sink; // source is node 0; time-power nodes precede sink.
    std::vector<CostArc> arcs;
};

struct CostLPBound {
    double value = 0.0; //initial LP bound
	double time = 0.0; // LP optimization time; excludes model construction and extraction
    double constant = 0.0; // fixed contribution from dualizing constraints
    std::vector<CostRoute> routes; // modified networks for all EVs
};

// performs the LP computation and returns the modified networks for all EVs
CostLPBound costLPBound(const EVCSP& data, double timeLimit);

} // namespace model
