#include "EVCSP.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

// truncation to work with integer values
int scaledPower(const std::string& text) {
    const auto dot = text.find('.');
    require(!text.empty() && text.find_first_not_of("0123456789.") == std::string::npos &&
            dot == text.rfind('.') && (dot == std::string::npos || text.size() - dot - 1 <= 4),
            "Expected a nonnegative decimal with at most four decimal places: " + text);
    // ignore only floating-point representation noise at an integer boundary.
    const double scaled = std::round(std::stod(text) * constants::PowerScale);
    require(scaled >= 0 && scaled <= std::numeric_limits<int>::max(), "Power value is out of range: " + text);
    return static_cast<int>(scaled);
}

// ignore blank lines and # comments
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
    require(row >> key >> value && key == expected && !(row >> extra),
            "Expected '" + expected + " <value>'.");
}

} // namespace

using namespace model;

// read the instance without modifying its parameters
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
        std::string grid;
        readHeader(input, "GRID_CAPACITY", grid);
        readHeader(input, "MAX_ADJUSTMENTS", N);
        require(last_charging_period > 0 && B > 0 && N >= 0,
                "Invalid horizon, charger count, grid capacity or adjustment limit.");

        maxPower = scaledPower(grid);

        std::istringstream powers(nextDataLine(input));
        std::string key, token;
        powers >> key;
        require(key == "POWER_LEVELS", "Expected POWER_LEVELS followed by positive powers.");
        W = {0}; // index 0 is the zero-power sentinel
        while (powers >> token) {
            const int scaled = scaledPower(token);
            require(scaled > W.back(), "Power levels must be positive and strictly increasing.");
            W.push_back(scaled);
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
            std::string requirement;
            require(row >> vehicle.id >> vehicle.firstChargingPeriod >> vehicle.lastChargingPeriod >>
                    requirement && !(row >> extra),
                    "Expected id first_period last_period required_power: " + line);
            require(vehicle.id > 0 && ids.insert(vehicle.id).second && vehicle.firstChargingPeriod >= 1 &&
                    vehicle.lastChargingPeriod >= 0 && vehicle.lastChargingPeriod <= last_charging_period,
                    "Invalid vehicle ID, charging window or power requirement: " + line);
            // Short/empty charging windows are allowed: the solvers certify infeasibility
            vehicle.powerRequirement = scaledPower(requirement);
            vehicle.minimumChargingPeriods = static_cast<int>(
                (static_cast<long long>(vehicle.powerRequirement) + W.back() - 1) / W.back());
            vehicles.push_back(vehicle);
        }
        require(vehicles.size() == static_cast<std::size_t>(expected),
                "VEHICLES does not match the number of vehicle rows.");
    } catch (const std::exception& ex) {
        throw std::runtime_error(path.string() + ": " + ex.what());
    }
}

// set the tariff profile and demand charge here for cost sensitivity experiments
void EVCSP::setChargingCosts() {
    const int T = last_charging_period;
    c.assign(T + 1, 0.0);
    for (int t = 1; t <= T; ++t) {
        const int band = t <= T / 3 ? 0 : t <= 2 * T / 3 ? 1 : 2;
        c[t] = constants::Tariffs[band];
    }
    F = constants::DemandChargeFactor * constants::ReferenceTariff * (tau * T / 60.0);
}

// print summary of the instance data
void EVCSP::printSummary(std::ostream& out) const {
    out << std::fixed << std::setprecision(2);
    out << "Instance: " << instanceName << '\n'
        << "Instance file: " << (dataDirectory / (instanceName + ".txt")).string() << '\n'
        << "EVs: " << vehicles.size() << "\nChargers: " << B
        << "\nHorizon: " << last_charging_period << " periods\n"
        << "Maximum power adjustments: " << N << '\n'
        << "Positive power levels: " << W.size() - 1 << '\n'
        << "Power levels:";
    for (std::size_t p = 1; p < W.size(); ++p) out << ' ' << W[p];
    out << "\n\nEV\tFirst\tLast\tPower sum\tMin periods\n";
    for (const auto& vehicle : vehicles) {
        out << vehicle.id << '\t' << vehicle.firstChargingPeriod << '\t' << vehicle.lastChargingPeriod
            << '\t' << vehicle.powerRequirement << '\t' << vehicle.minimumChargingPeriods << '\n';
    }
}
