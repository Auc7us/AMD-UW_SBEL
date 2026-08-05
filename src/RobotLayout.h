#pragma once

#include <array>
#include <cmath>

#include "chrono/assets/ChColor.h"
#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChConstants.h"

namespace amd_uw {

// Concentric site layout. Every rank owns one ray from the shared site centre,
// and everything belonging to that rank sits on it:
//
//        centre --- work circle (30) --- builder (35) --- collector (40) --->  rocks
//
// so a collector starts just outside its own builder, drives radially outward to
// fetch rocks, and comes back inward to the builder it feeds. Ranks are spread
// evenly around the circle, which also means each rank gets its own distinct
// heading -- the previous layout hard-coded two headings (330 deg / 60 deg) and
// silently gave every rank past the second the same one as rank 2.
inline constexpr double site_center_x = 0.0;
inline constexpr double site_center_y = 0.0;
inline constexpr double work_circle_radius = 30.0;
inline constexpr double builder_path_radius = 35.0;
inline constexpr double robot_start_radius = 40.0;

// Half-width of the radial band around the collector circle that counts as "at the
// drop point" -- so a 2 * this metre band, concentric with the collector circle.
// The collector's job is to put rocks down near its builder, not on a surveyed
// spot: demanding a 1.5 m circle made rovers orbit a point they were already
// standing beside, because pure pursuit cannot converge on a target inside its own
// turning radius. Kept here rather than in the controller so it stays tied to the
// radius it is a band around.
inline constexpr double drop_band_half_width = 2.0;

// Harvest cycles. Each time a collector finishes a load and dumps it, its whole
// lane -- rock line, drop point, and the builder it feeds -- rotates one step
// counter-clockwise about the site centre, and a fresh set of rocks is spawned on
// the new line. Everything below therefore takes a cycle index, and every position
// for a rank derives from the single angle RankRayAngleRad(rank, N, cycle). Nothing
// else needs to know a cycle happened.
//
// Note 360/30 = 12, so after 12 cycles a rank's lane lands on the lane another rank
// started from. Ranks are separate ChSystems and never interact physically, so this
// only means dumped piles overlap in the aggregated view.
inline constexpr double cycle_rotation_rad = 30.0 * chrono::CH_DEG_TO_RAD;

// One colour per rank, so a collector and the builder it feeds read as a matched pair
// and the ranks can be told apart in a wide shot. Everything belonging to rank i is
// painted RankColor(i).
//
// The first four are the requested set. Beyond that the hue advances by the golden
// angle: successive values never cluster, so scaling past four ranks needs no edit
// here and rank 12 stays as distinguishable as rank 2.
inline chrono::ChColor RankColor(int rank_index) {
    const std::array<chrono::ChColor, 4> requested = {{
        chrono::ChColor(0.80f, 0.13f, 0.13f),  // rank 1: red
        chrono::ChColor(0.15f, 0.65f, 0.22f),  // rank 2: green
        chrono::ChColor(0.95f, 0.58f, 0.08f),  // rank 3: orange
        chrono::ChColor(0.17f, 0.40f, 0.88f),  // rank 4: blue
    }};
    const int index = (rank_index < 0) ? 0 : rank_index;
    if (index < static_cast<int>(requested.size()))
        return requested[index];

    // Golden-angle hue walk for rank 5 up, at fixed saturation/value so they read as
    // the same family as the four above.
    constexpr double golden_angle_deg = 137.507764;
    const double hue = std::fmod(20.0 + golden_angle_deg * (index - 3), 360.0);
    const double s = 0.78;
    const double v = 0.88;
    const double c = v * s;
    const double h6 = hue / 60.0;
    const double x = c * (1.0 - std::fabs(std::fmod(h6, 2.0) - 1.0));
    const double m = v - c;
    double r = m, g = m, b = m;
    switch (static_cast<int>(h6) % 6) {
        case 0: r += c; g += x; break;
        case 1: r += x; g += c; break;
        case 2: g += c; b += x; break;
        case 3: g += x; b += c; break;
        case 4: r += x; b += c; break;
        default: r += c; b += x; break;
    }
    return chrono::ChColor(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b));
}

// Every manipulator, every rank, every view. Arms are deliberately NOT per-rank
// coloured -- that lives on the hull and trailer bed; colouring the arms too made each
// machine one flat block with no readable structure.
//
// Set explicitly, not left to mesh defaults: the arm OBJs ship without MTL files, and
// VSG's fallback and the sensor's (Kd 0.5) differ, so an arm looked like two different
// greys in the two views.
inline chrono::ChColor ArmGrey() {
    return chrono::ChColor(0.62f, 0.62f, 0.62f);
}

// Trailer dump-bed geometry, shared because the bed exists twice: the real dynamic tub
// on the owning rank, and a visual-only copy on the sensor rank (SynTrailerBedAgent).
// Both take the extents from here, or the copy would draw rocks resting in mid-air
// over a bed of the wrong size. Masses, materials and joints are not shared.
inline constexpr double trailer_bed_floor_x = 1.0;  // along the trailer
inline constexpr double trailer_bed_floor_y = 1.2;  // across it
inline constexpr double trailer_bed_wall_height = 0.15;
inline constexpr double trailer_bed_thickness = 0.03;

// One box of the tub: full extents, and its centre in the owning body's own frame.
struct TrailerBedBox {
    chrono::ChVector3d size;
    chrono::ChVector3d center;
};

// Floor, front (+x) and both side (+/-y) walls. The rear (-x) is open, closed by the
// hinged tailgate (a separate body so it can swing) -- see TrailerTailgateBox.
inline std::array<TrailerBedBox, 4> TrailerBedBoxes() {
    const double ex = trailer_bed_floor_x;
    const double ey = trailer_bed_floor_y;
    const double h = trailer_bed_wall_height;
    const double t = trailer_bed_thickness;
    return {{
        {chrono::ChVector3d(ex, ey, t), chrono::ChVector3d(0.0, 0.0, 0.0)},           // floor
        {chrono::ChVector3d(t, ey, h), chrono::ChVector3d(ex / 2.0, 0.0, h / 2.0)},   // +x front wall
        {chrono::ChVector3d(ex, t, h), chrono::ChVector3d(0.0, ey / 2.0, h / 2.0)},   // +y left wall
        {chrono::ChVector3d(ex, t, h), chrono::ChVector3d(0.0, -ey / 2.0, h / 2.0)},  // -y right wall
    }};
}

inline TrailerBedBox TrailerTailgateBox() {
    return {chrono::ChVector3d(trailer_bed_thickness, trailer_bed_floor_y, trailer_bed_wall_height),
            chrono::ChVector3d(0.0, 0.0, 0.0)};
}

// Angle of the ray owned by one rank. The rank's builder and its collector share
// this angle, which is what puts the collector on the line joining the site
// centre to its builder.
inline double RankRayAngleRad(int rank_index, int num_ranks, int cycle = 0) {
    if (num_ranks <= 0)
        return 0.0;
    return chrono::CH_2PI * static_cast<double>(rank_index) / static_cast<double>(num_ranks) +
           static_cast<double>(cycle) * cycle_rotation_rad;
}

inline chrono::ChVector3d PointOnSiteRay(int rank_index, int num_ranks, double radius, int cycle = 0) {
    const double angle = RankRayAngleRad(rank_index, num_ranks, cycle);
    return chrono::ChVector3d(site_center_x + radius * std::cos(angle),
                              site_center_y + radius * std::sin(angle), 0.0);
}

inline chrono::ChVector3d InitialGroundPositionForRobot(int robot_index, int num_robots, int cycle = 0) {
    return PointOnSiteRay(robot_index, num_robots, robot_start_radius, cycle);
}

// Radially outward, away from the site: the rock line runs into open field
// instead of back across the build area and its own builder.
inline double InitialHeadingRadForRobot(int robot_index, int num_robots, int cycle = 0) {
    return RankRayAngleRad(robot_index, num_robots, cycle);
}

inline chrono::ChVector3d RockLineForwardForRobot(int robot_index, int num_robots, int cycle = 0) {
    const double heading = InitialHeadingRadForRobot(robot_index, num_robots, cycle);
    return chrono::ChVector3d(std::cos(heading), std::sin(heading), 0.0);
}

inline chrono::ChVector3d GroundPositionAlongRockLine(int robot_index, int num_robots, double distance,
                                                      int cycle = 0) {
    return InitialGroundPositionForRobot(robot_index, num_robots, cycle) +
           RockLineForwardForRobot(robot_index, num_robots, cycle) * distance;
}

inline chrono::ChVector3d BuilderOrbitGroundPosition(int builder_index, int num_builders, int cycle = 0) {
    return PointOnSiteRay(builder_index, num_builders, builder_path_radius, cycle);
}

inline double BuilderOrbitHeadingRad(int builder_index, int num_builders, int cycle = 0) {
    // Counter-clockwise tangent to the orbit.
    return RankRayAngleRad(builder_index, num_builders, cycle) + 0.5 * chrono::CH_PI;
}

}  // namespace amd_uw
