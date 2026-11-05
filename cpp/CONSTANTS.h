#pragma once

#include <array>

namespace constants {

// period duration is used only by the optional cost objective.
// makespan inputs come entirely from E-VCSP-CGG-instances.
inline constexpr int ChargingPeriodMinutes = 5;
inline constexpr double Precision = 1e-5;

// solver time limit in seconds.
inline constexpr double TimeLimitSeconds = 3600.0;
inline constexpr int SolverThreads = 8; // CPLEX threads and CP Optimizer workers.
inline constexpr double RelativeOptimalityGap = 1e-4; // 0.01%; other solver tolerances use defaults.

// TOU tariffs per kWh and the fraction of the horizon used for the demand charge.
inline constexpr std::array<double, 3> Tariffs = {1.0, 2.0, 3.0};
inline constexpr double DemandChargeFactor = 0.5;

// conversion used only by the optional cost objective.
inline constexpr double PowerUnitsToKW = 0.6; // 1 unit = 0.6 kW

} // namespace constants
