#pragma once

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>

namespace ardurover_nav {

struct Waypoint {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

inline double yaw_from_quat(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

struct RPY {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

inline RPY rpy_from_quat(const geometry_msgs::msg::Quaternion& q) {
    const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    const double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(1.5707963267948966, sinp) : std::asin(sinp);
    return {std::atan2(sinr, cosr), pitch, yaw_from_quat(q)};
}

// Map an angle in radians to (-pi, pi]. Use on heading error so
// 179 deg vs -179 deg is a small turn, not a full spin.
inline double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

inline std::vector<Waypoint> load_path(const std::string& file) {
    std::ifstream in(file);
    if (!in) {
        throw std::runtime_error("Failed to open path file: " + file);
    }

    std::vector<Waypoint> waypoints;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        Waypoint waypoint;
        if (!(ss >> waypoint.x >> waypoint.y >> waypoint.yaw)) {
            throw std::runtime_error("Invalid path line: " + line);
        }
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

inline void save_path(const std::string& file, const std::vector<Waypoint>& waypoints) {
    std::ofstream out(file);
    if (!out) {
        throw std::runtime_error("Failed to write path file: " + file);
    }

    out << "# x y yaw\n";
    out << std::fixed << std::setprecision(6);
    for (const auto& waypoint : waypoints) {
        out << waypoint.x << " " << waypoint.y << " " << waypoint.yaw << "\n";
    }
}

}  // namespace ardurover_nav
