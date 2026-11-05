#include "EVCSP.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

double number(const std::string& text, const std::string& field) {
    std::istringstream input(text);
    double value = 0.0;
    require(input >> value && input.eof() && std::isfinite(value), "Invalid " + field + ": " + text);
    return value;
}

// ignore blank lines and # comments.
std::string nextDataLine(std::istream& input) {
    std::string line;
    while (std::getline(input, line)) {
        if (line.starts_with("\xEF\xBB\xBF")) line.erase(0, 3);
        const auto comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        if (line.find_first_not_of(" \t\r\n") != std::string::npos) return line;
    }
    require(!input.bad(), "Error reading text file.");
    return {};
}

void readHeader(std::istream& input, const std::string& expected, auto& value) {
    std::istringstream row(nextDataLine(input));
    std::string key, extra;
    require(row >> key >> value && key == expected && std::isfinite(value) && !(row >> extra),
            "Expected '" + expected + " <value>'.");
}

} // namespace

using namespace model;

// check whether the vehicle has enough charging time (based on bounds)
bool EVCSP::Vehicle::hasEnoughChargingTime() const {
    return minimumChargingPeriods == 0 ||
           (lastChargingPeriod >= firstChargingPeriod &&
            minimumChargingPeriods <= lastChargingPeriod - firstChargingPeriod + 1);
}

// read the instance without modifying its parameters.
EVCSP::EVCSP(std::string name)
    : instanceName(std::move(name)), dataDirectory("instances") {
    loadInstance();
    setChargingCosts();
}

// all makespan inputs are in one TXT file
void EVCSP::loadInstance() {
    const auto path = dataDirectory / (instanceName + ".txt");
    try {
        std::ifstream input(path);
        require(static_cast<bool>(input), "Cannot open instance file.");
        readHeader(input, "HORIZON", last_charging_period);
        readHeader(input, "CHARGERS", B);
        readHeader(input, "GRID_CAPACITY", maxPower);
        readHeader(input, "MAX_ADJUSTMENTS", N);
        require(last_charging_period > 0 && B > 0 && maxPower >= 0 && N >= 0,
                "Invalid horizon, charger count, grid capacity or adjustment limit.");

        std::istringstream powers(nextDataLine(input));
        std::string key, token;
        powers >> key;
        require(key == "POWER_LEVELS", "Expected POWER_LEVELS followed by positive powers.");
        W = {0.0}; // index 0 is the zero-power sentinel
        while (powers >> token) {
            const double power = number(token, "power level");
            require(power > W.back(), "Power levels must be positive and strictly increasing.");
            W.push_back(power);
        }
        require(W.size() > 1, "At least one positive power level is required.");
        int expected;
        readHeader(input, "VEHICLES", expected);
        require(expected >= 0, "VEHICLES must be nonnegative.");
        std::set<int> ids;
        for (auto line = nextDataLine(input); !line.empty(); line = nextDataLine(input)) {
            std::istringstream row(line);
            std::string extra;
            Vehicle vehicle;
            require(row >> vehicle.id >> vehicle.firstChargingPeriod >> vehicle.lastChargingPeriod >>
                    vehicle.powerRequirement && !(row >> extra),
                    "Expected id first_period last_period required_power: " + line);
            require(vehicle.id > 0 && ids.insert(vehicle.id).second && vehicle.firstChargingPeriod >= 1 &&
                    vehicle.lastChargingPeriod >= 0 && vehicle.lastChargingPeriod <= last_charging_period &&
                    std::isfinite(vehicle.powerRequirement) && vehicle.powerRequirement >= 0,
                    "Invalid vehicle ID, charging window or power requirement: " + line);
            // Short/empty charging windows are allowed: the solvers certify infeasibility.
            const double duration = std::ceil(vehicle.powerRequirement / W.back() - constants::Precision);
            vehicle.minimumChargingPeriods = static_cast<int>(std::max(0.0, duration));
            vehicles.push_back(vehicle);
        }
        require(vehicles.size() == static_cast<std::size_t>(expected),
                "VEHICLES does not match the number of vehicle rows.");
    } catch (const std::exception& ex) {
        throw std::runtime_error(path.string() + ": " + ex.what());
    }
}

// set the tariff profile and demand charge here for cost sensitivity experiments.
void EVCSP::setChargingCosts() {
    const int T = last_charging_period;
    c.assign(T + 1, 0.0);
    for (int t = 1; t <= T; ++t) {
        const int band = t <= T / 3 ? 0 : t <= 2 * T / 3 ? 1 : 2;
        c[t] = constants::Tariffs[band];
    }
    const double cMin = *std::min_element(constants::Tariffs.begin(), constants::Tariffs.end());
    F = constants::DemandChargeFactor * cMin * (tau * T / 60.0);
}

void EVCSP::printSummary(std::ostream& out) const {
    out << "Instance: " << instanceName << '\n'
        << "Instance file: " << (dataDirectory / (instanceName + ".txt")).string() << '\n'
        << "EVs: " << vehicles.size() << "\nChargers: " << B
        << "\nHorizon: " << last_charging_period << " periods\n"
        << "Maximum power adjustments: " << N << '\n'
        << "Positive power levels: " << W.size() - 1 << '\n'
        << "Power levels:";
    for (std::size_t p = 1; p < W.size(); ++p) out << ' ' << W[p];
    out << "\n\n EV  First  Last   Power sum  Min periods\n";
    int shortWindows = 0;
    out << std::fixed << std::setprecision(3);
    for (const auto& vehicle : vehicles) {
        out << std::setw(3) << vehicle.id
            << std::setw(7) << vehicle.firstChargingPeriod << std::setw(6) << vehicle.lastChargingPeriod
            << std::setw(12) << vehicle.powerRequirement << std::setw(13) << vehicle.minimumChargingPeriods << '\n';
        if (!vehicle.hasEnoughChargingTime()) ++shortWindows;
    }
    if (shortWindows > 0) {
        out << "Individual charging windows too short even at maximum power: " << shortWindows << '\n';
    }
}
