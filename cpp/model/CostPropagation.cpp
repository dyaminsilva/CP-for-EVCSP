#include "CostPropagation.h"

#include <cmath>
#include <vector>

namespace {

 // for one EV, it stores whether each arc is used and the modified costs from source to v and frrom v to sink
struct Path {
    IloBoolVarArray y;
    IloIntExprArray forward, backward;
};

// describes the cost of following one arc (label + weight or Omega if it cannot be reached); weigth is the reduced cost
IloIntExpr extend(IloModel model, IloBoolVar y, IloIntExpr label, IloInt weight, IloInt omega) {
    const IloEnv env = model.getEnv();
	IloIntVar next(env, -omega + weight, omega + weight); // variable with lower bound -omega + weight and upper bound omega + weight
    model.add(next == label + weight); // adds a constraint
    IloIntVarArray choices(env); // array of integer CP variables
    // choice[0] is fixed at Omega
	// choice[1] is fixed at label + weight
    choices.add(IloIntVar(env, omega, omega));
    choices.add(next);
    return IloElement(choices, (y == 1) && (label < omega));
}

} // namespace

IloNumExpr model::costPropagation(IloModel model, const CostLPBound& lp,
                                  const std::vector<std::vector<IloIntervalVarArray>>& modes) {
    const IloEnv env = model.getEnv();
    if (lp.routes.empty()) return IloNumExpr(env, 0.0);
    // Quantize costs with a compensated tolerance to preserve the lower bound.
    const double scale = 1.0 / constants::Precision;
    constexpr double roundingTolerance = 1e-6; // in scaled cost units

    // Compensate for the upward rounding allowance on every selected path.
    // A path uses one source arc and at most one outgoing arc per time period.
    double roundingAllowance = 0.0;
    for (const auto& route : lp.routes) {
        const int maxPathArcs = (route.sink - 1) / route.levels + 1;
        roundingAllowance += maxPathArcs * roundingTolerance;
    }
    const IloInt constant = static_cast<IloInt>(
        std::floor(lp.constant * scale - roundingAllowance));
	IloIntVar bound(env, 0, IloIntMax); // variable for the cost bound (lower bound on the objective)
    std::vector<Path> paths; // sortes EV arc variables and path labels
    std::vector<std::vector<IloInt>> weights(lp.routes.size()); // scaled modified cost
    paths.reserve(lp.routes.size());
    IloIntExpr total(env, constant); // starts at the scaled constant K
    for (std::size_t r = 0; r < lp.routes.size(); ++r) {
		// for each EV, create an arc-variable array and two label arrays (forward and backward)
        const auto& route = lp.routes[r];
        paths.push_back({IloBoolVarArray(env), IloIntExprArray(env, route.sink + 1),
                        IloIntExprArray(env, route.sink + 1)});
        auto& path = paths.back();
        std::vector<std::vector<int>> incoming(route.sink + 1), outgoing(route.sink + 1);
        IloInt omega = 1;
        for (std::size_t a = 0; a < route.arcs.size(); ++a) {
            // for each arc, create its selection variable
            const auto& arc = route.arcs[a];
            path.y.add(IloBoolVar(env));
            incoming[arc.to].push_back(static_cast<int>(a));
            outgoing[arc.from].push_back(static_cast<int>(a));
            weights[r].push_back(static_cast<IloInt>(
                std::floor(arc.cost * scale + roundingTolerance)));
            omega += std::abs(weights[r].back()); // value to detect unreachable nodes
        }
        // unit flow selects one charging path, using exactly the MILP's arcs.
        for (int v = 0; v <= route.sink; ++v) {
            IloIntExpr in(env), out(env);
            for (int a : incoming[v]) in += path.y[a];
            for (int a : outgoing[v]) out += path.y[a];
            model.add(out - in == (v == 0 ? 1 : v == route.sink ? -1 : 0)); // flow constraints
            if (v > 0 && v < route.sink) {
                const int t = route.first + (v - 1) / route.levels;
                const int ell = (v - 1) % route.levels;
                IloIntExpr occupied(env);
                for (const auto& J : modes[r])
                    occupied += IloOverlapLength(env, J[ell], t, t + 1, 0);
                model.add(out == occupied); // links the path to the charging intervals
            }
        }
        // node numbers are topological: source, time-power nodes, then sink.
        path.forward[0] = IloIntExpr(env, 0); // cost of reaching the source is zero
        for (int v = 1; v <= route.sink; ++v) {
            IloIntExprArray candidates(env);
            for (int a : incoming[v]) {
                const auto& arc = route.arcs[a];
                candidates.add(extend(model, path.y[a], path.forward[arc.from], weights[r][a], omega));
            }
            path.forward[v] = candidates.getSize() ? IloIntExpr(IloMin(candidates)) : IloIntExpr(env, omega); // take the minimum (DP-style constraints)
        }
		path.backward[route.sink] = IloIntExpr(env, 0); // cost of reaching the sink is zero
        for (int v = route.sink - 1; v >= 0; --v) {
            IloIntExprArray candidates(env);
            for (int a : outgoing[v]) {
                const auto& arc = route.arcs[a];
                candidates.add(extend(model, path.y[a], path.backward[arc.to], weights[r][a], omega));
            }
			path.backward[v] = candidates.getSize() ? IloIntExpr(IloMin(candidates)) : IloIntExpr(env, omega); // take the minimum (DP-style constraints)
        }
        model.add(path.forward[route.sink] < omega); // the path must reach the sink
        total += path.forward[route.sink]; // add this EV path cost to the expression total (which started with the constant K)
    }
    model.add(bound >= total); // global modified-path cost bound (i.e., the bound must be at least K pluss all EV path costs); see (6)

    // bound conditional on selecting each arc; see (7)
    for (std::size_t r = 0; r < lp.routes.size(); ++r) {
        IloIntExpr other(env, constant);
        for (std::size_t s = 0; s < lp.routes.size(); ++s)
            if (s != r) other += paths[s].forward[lp.routes[s].sink];
        for (std::size_t a = 0; a < lp.routes[r].arcs.size(); ++a) {
            const auto& arc = lp.routes[r].arcs[a];
            model.add(IloIfThen(env, paths[r].y[static_cast<IloInt>(a)] == 1,
                bound >= other + paths[r].forward[arc.from] + weights[r][a] + paths[r].backward[arc.to]));
        }
    }
    return bound / scale; // converts back to the original cost units
}
