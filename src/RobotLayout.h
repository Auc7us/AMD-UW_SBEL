#pragma once

#include "chrono/core/ChVector3.h"

namespace amd_uw {

inline chrono::ChVector3d InitialGroundPositionForRobot(int robot_index, int num_robots, double start_spacing) {
    return chrono::ChVector3d(0.0, (robot_index - 0.5 * (num_robots - 1)) * start_spacing, 0.0);
}

inline double InitialHeadingDegForRobot(int robot_index) {
    return (robot_index == 0) ? 330.0 : 60.0;
}

}  // namespace amd_uw
