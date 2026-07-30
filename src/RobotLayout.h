#pragma once

#include <cmath>

#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChConstants.h"

namespace amd_uw {

// Concentric site layout. Every rank owns one ray from the shared site centre,
// and everything belonging to that rank sits on it:
//
//        centre --- work circle (30) --- builder (40) --- collector (50) --->  rocks
//
// so a collector starts just outside its own builder, drives radially outward to
// fetch rocks, and comes back inward to the builder it feeds. Ranks are spread
// evenly around the circle, which also means each rank gets its own distinct
// heading -- the previous layout hard-coded two headings (330 deg / 60 deg) and
// silently gave every rank past the second the same one as rank 2.
inline constexpr double site_center_x = 0.0;
inline constexpr double site_center_y = 0.0;
inline constexpr double work_circle_radius = 30.0;
inline constexpr double builder_path_radius = 40.0;
inline constexpr double robot_start_radius = 50.0;

// Angle of the ray owned by one rank. The rank's builder and its collector share
// this angle, which is what puts the collector on the line joining the site
// centre to its builder.
inline double RankRayAngleRad(int rank_index, int num_ranks) {
    if (num_ranks <= 0)
        return 0.0;
    return chrono::CH_2PI * static_cast<double>(rank_index) / static_cast<double>(num_ranks);
}

inline chrono::ChVector3d PointOnSiteRay(int rank_index, int num_ranks, double radius) {
    const double angle = RankRayAngleRad(rank_index, num_ranks);
    return chrono::ChVector3d(site_center_x + radius * std::cos(angle),
                              site_center_y + radius * std::sin(angle), 0.0);
}

inline chrono::ChVector3d InitialGroundPositionForRobot(int robot_index, int num_robots) {
    return PointOnSiteRay(robot_index, num_robots, robot_start_radius);
}

// Radially outward, away from the site: the rock line runs into open field
// instead of back across the build area and its own builder.
inline double InitialHeadingRadForRobot(int robot_index, int num_robots) {
    return RankRayAngleRad(robot_index, num_robots);
}

inline chrono::ChVector3d RockLineForwardForRobot(int robot_index, int num_robots) {
    const double heading = InitialHeadingRadForRobot(robot_index, num_robots);
    return chrono::ChVector3d(std::cos(heading), std::sin(heading), 0.0);
}

inline chrono::ChVector3d GroundPositionAlongRockLine(int robot_index, int num_robots, double distance) {
    return InitialGroundPositionForRobot(robot_index, num_robots) +
           RockLineForwardForRobot(robot_index, num_robots) * distance;
}

inline chrono::ChVector3d BuilderOrbitGroundPosition(int builder_index, int num_builders) {
    return PointOnSiteRay(builder_index, num_builders, builder_path_radius);
}

inline double BuilderOrbitHeadingRad(int builder_index, int num_builders) {
    // Counter-clockwise tangent to the orbit.
    return RankRayAngleRad(builder_index, num_builders) + 0.5 * chrono::CH_PI;
}

}  // namespace amd_uw
