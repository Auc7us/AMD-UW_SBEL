#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "chrono/assets/ChColor.h"
#include "chrono/core/ChVector3.h"
#include "chrono/utils/ChConstants.h"

namespace amd_uw {

// Concentric site layout. Every rank owns one ray from the shared site centre,
// and everything belonging to that rank sits on it:
//
//        centre --- work circle (30) --- builder (33) --- collector (37) --->  rocks
//
// so a collector starts just outside its own builder, drives radially outward to
// fetch rocks, and comes back inward to the builder it feeds. Ranks are spread
// evenly around the circle, which also means each rank gets its own distinct
// heading -- the previous layout hard-coded two headings (330 deg / 60 deg) and
// silently gave every rank past the second the same one as rank 2.
//
// 33/37 diverged repeatedly before the builder was seated on a fitted terrain plane
// (see SeatBuilderOnTerrainPlane in main.cpp): 16-rank runs died at t=8.90 and t=5.78,
// and rank 1 at t=13.35, always with the builder's idler carrier rotating about the
// vertical -- which its prismatic joint forbids -- followed by the rover's arm weld
// tearing off as the weakest constraint sharing the rank's solver.
//
// The radius was never the cause. The builder was placed level from a SINGLE height
// probe at its own centre, so on ground that tilts 2-9 degrees the shoes at one end
// started up to 0.8 m into the terrain. 33 m simply landed on a worse spot than 35 m.
// With plane seating the worst-case error drops to 0.02-0.07 m and 33/37 runs clean.
//
// The gaps are set by the builder's arm. At geometry_scale 2 its links are a2=2.540,
// a3=2.286, a4=0.715 m, so full extension from the arm base is 5.54 m, but the band
// proven in service is the LRV's measured 1.39-2.22 m scaled by 2, i.e. 2.78-4.44 m.
// Parked tangentially the builder swings theta1 sideways, so that band IS its radial
// reach:
//
//   work gap  = 33 - 30 = 3.0 m   (inboard, places rocks on the work circle)
//   drop gap  = 37 - 33 = 4.0 m   (outboard, picks up from the collector's pile)
//
// Both are inside 2.78-4.44 m, so the builder serves the build site and the drop pile
// from one lane without leaving it. 35/40 put both gaps at 5.0 m -- outside the band,
// so the builder had to reposition for every pick and place.
//
// They cannot shrink much further: the hull is 2.686 m wide, so at 33 m the tracks
// occupy 31.66-34.34 m and the drop band starts at 37 - drop_band_half_width = 35.0 m,
// leaving 0.66 m between the builder's outboard track and the nearest rock a collector
// may drop. 30/32/35 makes that NEGATIVE (-0.34 m) and drops the work gap to 2.0 m,
// inside the folded-arm limit.
inline constexpr double site_center_x = 0.0;
inline constexpr double site_center_y = 0.0;
inline constexpr double work_circle_radius = 30.0;
inline constexpr double builder_path_radius = 33.0;
inline constexpr double robot_start_radius = 37.0;

// Where the rover is PLACED at t=0, which is deliberately NOT its drop point.
//
// A rover spawns heading radially outward, so its trailer points inward -- straight at
// its own builder, in the same ChSystem. The rig reaches 2.41 m behind the rover's
// chassis origin (its rear connector defaults to (0,0,0); the trailer's "Front
// Connector Location" is 1.9 m with a body box spanning +/-0.51), and the builder's
// hull is 2.686 m wide, so at 33 m it occupies 31.66-34.34 m. Spawning the rover on the
// 37 m drop ring would leave 0.25 m between the trailer and the builder's tracks, both
// dropped from vehicle_start_clearance onto sloping ground. At 42 m that becomes 5.25 m,
// and the rover simply starts a little way along the outbound leg it was going to drive
// anyway.
//
// Only the initial placement reads this. The drop point, the rock line and the terrain
// extensions all stay on robot_start_radius, so the harvest geometry is untouched.
// Pushing the BUILDER outward instead cannot work: the rover rig occupies 34.59-38.99 m
// radially, so the first clear outboard slot is past the drop point and the builder
// would have to drive through the parked rover to reach its lane.
inline constexpr double robot_spawn_radius = 42.0;

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
// The tub is ASYMMETRIC across the trailer because it discharges to the left (+y).
//
// The trailer's wheels sit at y = +/-0.5 with a 0.4089 m tire radius, so the left tire
// spans y = 0.35..0.65 and its top is at z = +0.159 in the trailer chassis frame -- only
// 21 mm below the old bed's lip at +0.18. A symmetric 1.2 m tub therefore discharges
// straight onto its own wheel. Tilting harder makes it WORSE, not better: the horizontal
// reach of a lip a distance d from the tilt axis is d*cos(theta), so at the 55 deg dump
// angle a lip at 0.6 m ends up at 0.34 m -- further inboard, right over the tire.
//
// So the discharge side is extended to 0.8 m, clear of the tire's 0.65 m outer edge, and
// the tilt axis is put ON that lip (see InitializeTrailerBed) so it does not move inboard
// as the tub rotates. The right side is unchanged.
inline constexpr double trailer_bed_right_y = 0.6;      // -y wall, unchanged
inline constexpr double trailer_bed_discharge_y = 0.8;  // +y open lip, outboard of the tire
inline constexpr double trailer_bed_floor_y = trailer_bed_right_y + trailer_bed_discharge_y;
// Centre of the tub in the trailer frame: offset left by the extra overhang.
inline constexpr double trailer_bed_center_y = 0.5 * (trailer_bed_discharge_y - trailer_bed_right_y);
inline constexpr double trailer_bed_wall_height = 0.15;
inline constexpr double trailer_bed_thickness = 0.03;

// One box of the tub: full extents, and its centre in the owning body's own frame.
struct TrailerBedBox {
    chrono::ChVector3d size;
    chrono::ChVector3d center;
};

// Floor, both end (+/-x) walls and the right (-y) wall. The LEFT (+y) side is open,
// closed by the hinged tailgate -- see TrailerTailgateBox. The trailer used to discharge
// rearward (-x open, tailgate across the back); it now discharges to the left so the
// collector can pour while running tangentially past its drop point instead of turning
// around next to its own builder.
inline std::array<TrailerBedBox, 4> TrailerBedBoxes() {
    const double ex = trailer_bed_floor_x;
    const double ey = trailer_bed_floor_y;
    const double cy = trailer_bed_center_y;
    const double h = trailer_bed_wall_height;
    const double t = trailer_bed_thickness;
    return {{
        {chrono::ChVector3d(ex, ey, t), chrono::ChVector3d(0.0, cy, 0.0)},           // floor
        {chrono::ChVector3d(t, ey, h), chrono::ChVector3d(ex / 2.0, cy, h / 2.0)},   // +x front wall
        {chrono::ChVector3d(t, ey, h), chrono::ChVector3d(-ex / 2.0, cy, h / 2.0)},  // -x rear wall
        {chrono::ChVector3d(ex, t, h),
         chrono::ChVector3d(0.0, -trailer_bed_right_y, h / 2.0)},                    // -y right wall
    }};
}

// The hinged left wall, in ITS OWN body frame. It runs along the trailer (ex long) rather
// than across it, because it now closes the long open side rather than the short back.
inline TrailerBedBox TrailerTailgateBox() {
    return {chrono::ChVector3d(trailer_bed_floor_x, trailer_bed_thickness, trailer_bed_wall_height),
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

// The rank's drop point: where the collector parks to unload, and the origin of its
// rock line. Also the initial home published to the drive controller.
inline chrono::ChVector3d InitialGroundPositionForRobot(int robot_index, int num_robots, int cycle = 0) {
    return PointOnSiteRay(robot_index, num_robots, robot_start_radius, cycle);
}

// Where the rover is placed at t=0 -- further out on the same ray, clear of the
// builder. See robot_spawn_radius.
inline chrono::ChVector3d InitialSpawnPositionForRobot(int robot_index, int num_robots, int cycle = 0) {
    return PointOnSiteRay(robot_index, num_robots, robot_spawn_radius, cycle);
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

// ---------------------------------------------------------------------------
// The builder's own build plan.
//
// The builder runs this on its OWN schedule -- it is not triggered by, gated on, or
// indexed by the collector's harvest cycle. It walks counter-clockwise along its lane
// laying a course of rocks on the work circle, one slot at a time, taking each rock
// from a feedstock heap sitting outboard of its lane.
//
// EVERYTHING here is derived from one integer, the wall slot index k, so the arm, the
// drive station and the feedstock all agree by construction rather than by tuning:
//
//   wall slot k   -> work_circle_radius,      angle ray + k*slot_pitch_rad
//   station k     -> builder_path_radius,     angle ray + k*slot_pitch_rad + arm_lead
//   heap for k    -> builder_pile_radius,     angle of the station at the heap's centre
//
// arm_lead exists because the arm is mounted 2.5 m BACK along a hull parked tangentially,
// so the arm base sits at hypot(33, 2.5) = 33.095 m, trailing the hull's own station angle
// by atan(2.5/33). Adding it to the station means the ARM BASE, not the hull origin, ends
// up radially opposite the slot it is serving -- which is what makes both the reach to the
// work circle (3.095 m) and the reach to the heap (3.5 m) land mid-band every time.
// ---------------------------------------------------------------------------

// Arc between neighbouring rocks in the laid course, measured on the work circle. About
// 2x a 0.2-scale rock's width, so the course reads as laid stones with a gap rather than
// a fused kerb -- and it makes each build step 0.9/30*33 = 0.99 m of lane, which is the
// "move a tiny bit and put the next one down beside it" the wall is built by.
inline constexpr double wall_slot_pitch_m = 0.9;
inline constexpr double wall_slot_pitch_rad = wall_slot_pitch_m / work_circle_radius;

// Where the feedstock heaps sit: 3.5 m radially outboard of the arm base, mid-way through
// the 2.78-4.44 m band the arm proved in service.
inline constexpr double builder_pile_radius = 36.6;

// One heap serves this many consecutive slots. The builder creeps ~1 m per slot, so over
// four of them the heap goes from 1.5 m ahead to 1.5 m behind the arm base -- reach
// hypot(3.5, 1.5) = 3.81 m at worst, still inside the band. Then it moves to the next heap.
inline constexpr int wall_slots_per_pile = 4;

// How many heaps to lay out per builder, before the sector cap below applies.
inline constexpr int builder_pile_count = 6;

// Trailing angle from the hull's station to its arm base. See the block comment.
inline constexpr double builder_arm_mount_back_m = 2.5;
inline double BuilderArmLeadRad() {
    return std::atan2(builder_arm_mount_back_m, builder_path_radius);
}

// Total slots this builder may lay. Capped so a rank's course never runs into the next
// rank's sector: with N builders each owns 2*pi/N of lane, and 0.7 of it leaves room for
// the machine itself (2.686 m wide) at either end.
inline int BuilderWallSlotCount(int num_builders) {
    const int desired = builder_pile_count * wall_slots_per_pile;
    if (num_builders <= 1)
        return desired;
    const double sector_rad = chrono::CH_2PI / static_cast<double>(num_builders);
    const int cap = static_cast<int>(0.7 * sector_rad / wall_slot_pitch_rad);
    return std::max(wall_slots_per_pile, std::min(desired, cap));
}

// Ground position of wall slot k, z left at 0 for the caller to probe against terrain.
inline chrono::ChVector3d BuilderWallSlotPosition(int builder_index, int num_builders, int slot) {
    const double angle = RankRayAngleRad(builder_index, num_builders) + slot * wall_slot_pitch_rad;
    return chrono::ChVector3d(site_center_x + work_circle_radius * std::cos(angle),
                              site_center_y + work_circle_radius * std::sin(angle), 0.0);
}

// Orbit angle the hull must hold for its arm base to face wall slot k. This is what
// replaces the harvest-cycle station: it advances one step per rock LAID, by this
// builder, with no reference to what its collector is doing.
inline double BuilderStationAngleRad(int builder_index, int num_builders, int slot) {
    return RankRayAngleRad(builder_index, num_builders) + slot * wall_slot_pitch_rad + BuilderArmLeadRad();
}

// Where the ARM BASE sits when the hull is on station for slot k: on the lane radius but
// at the slot's own angle, because the hull leads by arm_lead precisely so that this
// cancels. Both the wall slot and the heap are placed relative to THIS point, not to the
// hull, since it is the origin of the IK frame.
inline constexpr double builder_arm_base_radius_approx = 33.0946;  // hypot(33, 2.5)

// Centre of the feedstock heap serving slots [pile*P, pile*P + P), placed at the middle
// of that run so it is never more than ~1.5 m of lane from the arm base working it.
//
// NO arm_lead here. The heap must be radially outboard of the ARM BASE, and the arm base
// already sits at the slot's own angle -- adding the lead again shifts the heap a further
// 0.0757 rad counter-clockwise, which at radius 36.6 is 2.77 m of arc. Measured live off
// /builder_1/pick_target with the lead added: the heap was 5.62 m from the arm base,
// outside the 5.2 m guard, so no rock was ever reachable. Without it: 3.84 m at the
// extremes of the run, 3.50 m at its centre, both mid-band.
inline chrono::ChVector3d BuilderPileCenter(int builder_index, int num_builders, int pile) {
    const double slot = pile * wall_slots_per_pile + 0.5 * (wall_slots_per_pile - 1);
    const double angle = RankRayAngleRad(builder_index, num_builders) + slot * wall_slot_pitch_rad;
    return chrono::ChVector3d(site_center_x + builder_pile_radius * std::cos(angle),
                              site_center_y + builder_pile_radius * std::sin(angle), 0.0);
}

}  // namespace amd_uw
