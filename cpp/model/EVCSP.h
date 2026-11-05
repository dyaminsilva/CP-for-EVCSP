#pragma once

#include "../CONSTANTS.h"

#include <filesystem>
#include <iosfwd>
#include <string>
#include <vector>

namespace model {

// instance data for charging a fixed set of EVs
class EVCSP {
public:
    struct Vehicle {
        int id = 0;
        int firstChargingPeriod = 1;
        int lastChargingPeriod = 0;      // inclusive; already restricted by the depot horizon
        double powerRequirement = 0.0;   // minimum sum of power across charging periods
        int minimumChargingPeriods = 0;  // lower bound on the charging duration

        bool hasEnoughChargingTime() const;
    };

    // instance bookkeeping
    const std::string instanceName;
    std::filesystem::path dataDirectory;

    int B = 0;                          // available chargers
    int tau = constants::ChargingPeriodMinutes; // time discretization unit
    int N = 0;                          // maximum number of power adjustments
    int last_charging_period = 0;        // inclusive horizon, in periods
    double maxPower = 0.0;               // grid capacity
    std::vector<double> W;               // index 0 is the zero-power sentinel
    std::vector<double> c;               // TOU tariff per kWh, indexed by period
    double F = 0.0;                      // demand charge per kW
    std::vector<Vehicle> vehicles;       // set of EV routes

    // methods
    explicit EVCSP(std::string instanceName);
    void printSummary(std::ostream& out) const;

private:
    void loadInstance();
    void setChargingCosts();
};

} // namespace model
