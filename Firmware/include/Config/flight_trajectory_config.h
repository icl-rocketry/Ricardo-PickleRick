#pragma once

#include <array>
#include <cstddef>

namespace FlightTrajectoryConfig {

struct WaypointNed {
    float north_m;
    float east_m;
    float down_m;
};

struct Leg {
    WaypointNed start_ned_m;
    WaypointNed finish_ned_m;
    float duration_s;
    float accel_fraction;
};

// Waypoints are [north, east, down] metres in NED.
// With origin at launch, negative down commands height above launch.

inline constexpr std::array<Leg, 4> Legs{{ 
    {{0.0f, 0.0f, 0.0f},  {0.0f, 0.0f, -0.7f}, 2.5f, 0.1f},//start, stop, duration, accel fraction
    {{0.0f, 0.0f, -0.7f}, {0.0f, 0.0f, -0.7f}, 5.0f, 0.2f}, //wait
    {{0.0f, 0.0f, -0.7f}, {0.0f, 1.0f, -0.7f}, 5.0f, 0.2f}, //move
    {{0.0f, 1.0f, -0.7f}, {0.0f, 1.0f, -0.7f}, 5.0f, 0.2f}  //wait
}};

inline constexpr std::size_t LegCount = Legs.size();

} // namespace FlightTrajectoryConfig
