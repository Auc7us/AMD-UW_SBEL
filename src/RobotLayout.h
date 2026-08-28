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
// SCALED UP from 30/33/37. The RADIAL OFFSETS between the rings are unchanged, and
// deliberately so: they are set by machine dimensions, not by site size.
//
//   wall -> builder lane   3.0 m   the arm's own geometry. The arm base rides the lane
//                                  at hypot(lane, 2.5) and must reach its slot on the
//                                  wall; scaling this would break the reach the whole
//                                  build depends on.
//   lane -> collector ring 4.0 m   clearance between a 2.686 m hull and a 1.49 m rover.
//                                  Vehicles do not get bigger with the site.
//   ring -> rover spawn    5.0 m   staging, out of the drop band.
//
// What growing the circle buys is ARC per rank, which is what the builders need in order
// to rotate as they build: at 16 ranks the wall sector goes from 11.78 m to 19.63 m, and
// the course from 8.25 m to 13.74 m -- from one arm-station to nearly two.
//
// NOTE the terrain. The levelled pad is r <= 45 m (make_graded_pad.py --pad-radius) and
// the site now reaches r = 62 m at the spawn ring, where the measured height spread is
// 0.50 m rms against 0.23 m inside the pad. Regrade before running this:
//     python3 tools/make_graded_pad.py --pad-radius 65 --taper-radius 130
inline constexpr double work_circle_radius = 50.0;
inline constexpr double builder_path_radius = 53.0;
inline constexpr double robot_start_radius = 57.0;

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
inline constexpr double robot_spawn_radius = 62.0;

// Half-width of the radial band around the collector circle that counts as "at the
// drop point" -- so a 2 * this metre band, concentric with the collector circle.
// The collector's job is to put rocks down near its builder, not on a surveyed
// spot: demanding a 1.5 m circle made rovers orbit a point they were already
// standing beside, because pure pursuit cannot converge on a target inside its own
// turning radius. Kept here rather than in the controller so it stays tied to the
// radius it is a band around.
inline constexpr double drop_band_half_width = 2.0;

// Arc between neighbouring rocks in the laid course, measured on the work circle. About
// 2x a 0.2-scale rock's width, so the course reads as laid stones with a gap rather than
// a fused kerb -- and it makes each build step 0.9/30*33 = 0.99 m of lane, which is the
// "move a tiny bit and put the next one down beside it" the wall is built by.
//
// Declared up here, ahead of the harvest cycle, because the harvest lane is measured in
// these slots -- see HarvestDropSlot.
// 0.5 m centre to centre, not 0.9. A rock is 0.244 m across, so this leaves a 0.26 m
// gap and the course reads as a wall being built rather than a dotted line of markers.
// 0.9 was a training spacing.
//
// Everything downstream is derived, so this is the only number to change: slots per rank
// (BuilderWallSlotCount), the station's advance per rock, the seed pile layout, and the
// collector's per-cycle lane offset all scale off wall_slot_pitch_rad. The one place it is
// NOT derived is slot_pitch_rad in pure_pursuit_controller.py, which mirrors it by hand.
inline constexpr double wall_slot_pitch_m = 0.5;
inline constexpr double wall_slot_pitch_rad = wall_slot_pitch_m / work_circle_radius;

// How many rocks the builder starts with, heaped beside its lane, and how many its
// collector brings back per round trip.
//
// The seed heap exists only to give the builder something to do during the collector's
// first outbound leg, which is minutes of sim long. After that the builder eats what the
// collector delivers, and nothing else.
inline constexpr int builder_seed_rock_count = 6;

// Rocks on one rank's rock line, and therefore in one collector load. It was a fixed 2
// on every rank; it is now 2-6, drawn per rank, so the site is not fifteen copies of the
// same round trip.
//
// This is THE source of truth for that count and every consumer has to come through it,
// because the number is read in three places that must agree exactly or the run breaks
// in ways that do not look like a rock count:
//
//   * RockField spawns this many bodies on the owning rank;
//   * SynRockAgent sizes the sensor rank's zombie pool from it -- undersize it and the
//     tail of a rank's rocks is simply invisible in the global camera;
//   * HarvestDropSlot advances the collector's lane by one load's worth of WALL SLOTS
//     per cycle, which is what keeps each delivered pile within the builder's arm reach
//     of the station it is already driving to.
//
// So it is a pure function of the rank index, computed identically in every process,
// rather than anything carried in RockFieldConfig -- rank 0 has to be able to work out
// rank 7's count without being told.
inline constexpr int min_rocks_per_rank = 2;
inline constexpr int max_rocks_per_rank = 6;

// >0 pins every rank to that count. Set once from --rocks_per_rank, before anything
// reads the layout; every rank parses the same command line, so they stay in agreement.
inline int g_rocks_per_rank_override = 0;
inline void SetRocksPerRankOverride(int count) {
    g_rocks_per_rank_override = count;
}

// Fixed seed for the draw. Change it to reshuffle every rank's whole sequence of loads;
// keep it and the run is bit-identical, which is what makes a recording reproducible.
inline constexpr unsigned int harvest_rock_count_seed = 0x20260817u;

// The count now varies PER CYCLE as well as per rank, so a rank does not bring the same
// load forever: rank 1 might bring 2, then 5, then 3. It is still a pure function --
// hashed from (rank, cycle, seed), not drawn from a stream -- for the same reason it
// always was. Rank 0 has to be able to work out what rank 7 will bring on cycle 4
// without being told, and a std::mt19937 would have to be advanced in the same order in
// every process to manage that.
//
// CYCLE IS PART OF THE HASH, not an index into a sequence, so any cycle can be evaluated
// directly. HarvestDropSlot needs exactly that: it sums every earlier cycle's load to
// find where the wall has reached, and it must be able to do so from a standing start.
inline int RocksPerRank(int rank_index, int cycle) {
    if (g_rocks_per_rank_override > 0)
        return g_rocks_per_rank_override;
    // Integer hash (splitmix32 finalizer), not <random>: this has to give the same
    // answer in every MPI process on every machine, and a std::mt19937 stream would
    // have to be advanced in the same order everywhere to do that.
    unsigned int x = 0x9E3779B9u * static_cast<unsigned int>((rank_index < 0 ? 0 : rank_index) + 1)
                   + 0x85EBCA6Bu * static_cast<unsigned int>((cycle < 0 ? 0 : cycle) + 1)
                   + harvest_rock_count_seed;
    x ^= x >> 16;
    x *= 0x7FEB352Du;
    x ^= x >> 15;
    x *= 0x846CA68Bu;
    x ^= x >> 16;
    constexpr unsigned int span = max_rocks_per_rank - min_rocks_per_rank + 1;
    return min_rocks_per_rank + static_cast<int>(x % span);
}

// Harvest cycles. Each time a collector finishes a load and dumps it, its whole lane --
// rock line, drop point, and the rocks it will fetch next -- steps counter-clockwise
// about the site centre, and a fresh set of rocks is spawned on the new line.
//
// The step is measured in WALL SLOTS, not in a round number of degrees, and that is the
// point of this whole arrangement: the drop point lands exactly where the builder's wall
// will have reached by the time that load arrives. Cycle c drops at slot
// seed + c*load, and the builder is at slot seed + c*load once it has laid the seed and
// every load before this one. So the pile is always within the arm's reach of the station
// the builder is already driving to, and the builder clears each pile off the collector
// circle before the collector comes back to that stretch of it.
//
// It was a flat 30 deg, which at 0.03 rad/slot is 17 slots -- a load of two rocks every
// 17 slots leaves a wall that is 88% gaps, and puts every pile 15 m of arc from the
// builder that is supposed to eat it.
//
// Both take the rank, because a load is now 2-6 rocks depending on whose it is, and the
// step is one slot per rock DELIVERED. A rank whose collector brings six rocks walks its
// lane six slots per cycle; one that brings two walks two. Using a single site-wide load
// size here would put the fast ranks' piles well ahead of the wall their builder has
// actually reached.
// The most any cycle can bring. Anything SIZED rather than placed has to use this and
// not a particular cycle's draw -- a pool built for a 2-rock cycle overflows on a 6-rock
// one, and the overflow is silent.
inline int MaxRocksPerCycle() {
    return g_rocks_per_rank_override > 0 ? g_rocks_per_rank_override : max_rocks_per_rank;
}

inline int HarvestDropSlot(int rank_index, int cycle) {
    // SUM of every earlier load, not cycle * a constant. The whole point of measuring the
    // step in wall slots is that the pile lands where the builder's wall will actually
    // have reached, and the wall has reached seed + (every rock delivered so far). With a
    // per-cycle load size, multiplying by this cycle's draw would put a 6-rock rank's
    // fourth pile 24 slots out when its wall is only at 14.
    int slot = builder_seed_rock_count;
    for (int c = 0; c < (cycle < 0 ? 0 : cycle); ++c)
        slot += RocksPerRank(rank_index, c);
    return slot;
}

// Where that load is actually PUT, as a slot number: the MIDDLE of the run of slots it
// will be eaten over, not its first slot.
//
// This is the same rule BuilderPileCenter already uses for the seed heap, and for the
// same reason -- the builder has to reach the pile from either end of the run. It only
// became load-bearing when a load stopped being two rocks. A load is tipped out in one
// place and the builder then creeps one slot per rock laid, so with L rocks the far end
// of the run is (L-1) slots of lane away from the pile. Measured at L=6: the arm base
// ended up 6.54 m from the drop point against a 5.2 m envelope, and the reach audit said
// so at t=0 -- the builder would have parked at its slot and refused the pile beside it.
// Centring halves that lever arm: +/-2.5 slots gives 4.70 m, inside the envelope, and it
// also tightens the old L=2 case from 4.04 m to 3.94 m.
inline double HarvestLoadCenterSlot(int rank_index, int cycle) {
    return HarvestDropSlot(rank_index, cycle) + 0.5 * (RocksPerRank(rank_index, cycle) - 1);
}
inline double HarvestLaneOffsetRad(int rank_index, int cycle) {
    return HarvestLoadCenterSlot(rank_index, cycle) * wall_slot_pitch_rad;
}

// Distance from the tractor's chassis origin BACK to the trailer's rear pour lip, along
// the rig. The rocks land here, not under the tractor, and getting that wrong is the
// difference between a pile the builder can reach and one it cannot.
//
// Geometry: the trailer's front connector sits 1.9 m behind the tractor origin, and the
// tub's pour lip is trailer_bed_half_x = 0.5 m behind the trailer chassis origin.
//
// Measured before this existed: rank 4's cycle-0 drop point is slot 6 = 280.31 deg, its
// rover parked at ~276.1 deg, and the two rocks froze at 269.4 and 270.3 deg -- roughly
// 10 deg, or 6.5 m of arc, clockwise of the slot they were meant for, which put them
// outside the arm's 5 m envelope from a builder standing at slot 5. Two terms made that
// up: the rover stops the instant it is inside drop_arc_tolerance_m, so it parked 2.7-2.9 m
// short (its own logs), and then poured 2.4 m further back again.
inline constexpr double trailer_pour_offset_m = 2.4;

// Where the ROVER must park for its pour lip to land on the drop point: the same arc,
// advanced by the length of its own rig. Counter-clockwise, because that is the direction
// it runs the last stretch of the collector circle (see the tangential run-in).
inline double RoverParkOffsetRad() {
    return trailer_pour_offset_m / robot_start_radius;
}

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
// The tub is SYMMETRIC across the trailer and discharges REARWARD, over the -x lip.
//
// It spent a while discharging to the left instead, which forced the tub off-centre: the
// trailer's wheels sit at y = +/-0.5 with a 0.4089 m tire radius, so the left tire spans
// y = 0.35..0.65 and its top is only 21 mm below the bed lip -- a symmetric tub pouring
// sideways empties onto its own wheel, and the discharge side had to be pushed out to
// 0.8 m to clear it. Rear discharge has no such conflict (nothing is behind the tailgate
// but ground), so the tub goes back to sitting on the trailer centreline, which is also
// where RosArmBridge's 4x4 placement grid has always aimed.
inline constexpr double trailer_bed_half_y = 0.6;
inline constexpr double trailer_bed_floor_y = 2.0 * trailer_bed_half_y;
inline constexpr double trailer_bed_center_y = 0.0;
// Half-length along the trailer; the rear lip (-x) is the pour line and the tilt axis.
inline constexpr double trailer_bed_half_x = 0.5 * trailer_bed_floor_x;
// DOUBLED from 0.15. Rocks are aimed at the bed centre with a measured |rock-place| error
// of 0.42-0.56 m against a 0.6 m half-width, and they arrive from a 0.47 m release height,
// so a near-miss lands on the bed and then has to be KEPT there while the collector drives
// a 358 m ring back to its builder. A 0.15 m wall is under one rock radius (~0.23 m rocks
// at rock_mesh_scale 0.2) and is climbable by a rock that lands rolling.
//
// Nothing else needs adjusting for this: TrailerBedBoxes centres each wall at h/2 and
// TrailerTailgateBox takes the same height, so the gate still closes the rear exactly, and
// the sensor rank's visual-only copy of the bed is built from these same constants
// (SynAgents.cpp). The bed's inertia picks up 4.6% (mass is a fixed 30 kg). Clearance for
// the arm is unaffected -- release is 0.47 m above the floor, so 0.17 m still clears the
// taller wall -- and the dump is unaffected because the load slides out over the REAR lip,
// which is the gated end, not over a side wall.
inline constexpr double trailer_bed_wall_height = 0.30;
inline constexpr double trailer_bed_thickness = 0.03;

// One box of the tub: full extents, and its centre in the owning body's own frame.
struct TrailerBedBox {
    chrono::ChVector3d size;
    chrono::ChVector3d center;
};

// Floor, the +x front wall and both (+/-y) side walls. The REAR (-x) side is open,
// closed by the hinged tailgate -- see TrailerTailgateBox.
inline std::array<TrailerBedBox, 4> TrailerBedBoxes() {
    const double ex = trailer_bed_floor_x;
    const double ey = trailer_bed_floor_y;
    const double h = trailer_bed_wall_height;
    const double t = trailer_bed_thickness;
    return {{
        {chrono::ChVector3d(ex, ey, t), chrono::ChVector3d(0.0, 0.0, 0.0)},                    // floor
        {chrono::ChVector3d(t, ey, h), chrono::ChVector3d(trailer_bed_half_x, 0.0, h / 2.0)},  // +x front
        {chrono::ChVector3d(ex, t, h), chrono::ChVector3d(0.0, trailer_bed_half_y, h / 2.0)},  // +y side
        {chrono::ChVector3d(ex, t, h), chrono::ChVector3d(0.0, -trailer_bed_half_y, h / 2.0)}, // -y side
    }};
}

// The hinged rear wall, in ITS OWN body frame. It runs ACROSS the trailer (ey wide),
// closing the short open end.
inline TrailerBedBox TrailerTailgateBox() {
    return {chrono::ChVector3d(trailer_bed_thickness, trailer_bed_floor_y, trailer_bed_wall_height),
            chrono::ChVector3d(0.0, 0.0, 0.0)};
}

// Angle of the ray owned by one rank -- the BASE ray, which never moves. The builder's
// wall starts on it (slot 0) and grows counter-clockwise from there.
//
// This deliberately takes no cycle argument any more. The builder's course is fixed
// site geometry; only the COLLECTOR's lane walks, and it walks along the builder's own
// slots rather than in a step of its own -- see HarvestLaneAngleRad. Passing a cycle
// here was how the builder's whole plan used to get dragged along with the harvest.
inline double RankRayAngleRad(int rank_index, int num_ranks) {
    if (num_ranks <= 0)
        return 0.0;
    return chrono::CH_2PI * static_cast<double>(rank_index) / static_cast<double>(num_ranks);
}

// Angle of the COLLECTOR's lane on a given harvest cycle: the base ray, advanced by the
// wall slots the builder will have laid by the time that load lands. See HarvestDropSlot.
inline double HarvestLaneAngleRad(int rank_index, int num_ranks, int cycle) {
    return RankRayAngleRad(rank_index, num_ranks) + HarvestLaneOffsetRad(rank_index, cycle);
}

inline chrono::ChVector3d PointOnSiteRay(int rank_index, int num_ranks, double radius) {
    const double angle = RankRayAngleRad(rank_index, num_ranks);
    return chrono::ChVector3d(site_center_x + radius * std::cos(angle),
                              site_center_y + radius * std::sin(angle), 0.0);
}

inline chrono::ChVector3d PointOnHarvestLane(int rank_index, int num_ranks, double radius, int cycle) {
    const double angle = HarvestLaneAngleRad(rank_index, num_ranks, cycle);
    return chrono::ChVector3d(site_center_x + radius * std::cos(angle),
                              site_center_y + radius * std::sin(angle), 0.0);
}

// The rank's DROP POINT: where the load has to end up, on the harvest lane, beside the
// stretch of wall the builder is about to build. This is the pour line, not the tractor.
inline chrono::ChVector3d HarvestDropPoint(int robot_index, int num_robots, int cycle = 0) {
    return PointOnHarvestLane(robot_index, num_robots, robot_start_radius, cycle);
}

// Where the collector PARKS to put its load on that point, and the origin of its rock
// line: the drop point advanced by the length of its own rig, so the pour lip -- which
// trails 2.4 m behind the tractor -- lands on the drop point rather than 2.4 m short of
// it. This is what gets published as homePos. See trailer_pour_offset_m.
inline chrono::ChVector3d InitialGroundPositionForRobot(int robot_index, int num_robots, int cycle = 0) {
    const double angle =
        HarvestLaneAngleRad(robot_index, num_robots, cycle) + RoverParkOffsetRad();
    return chrono::ChVector3d(site_center_x + robot_start_radius * std::cos(angle),
                              site_center_y + robot_start_radius * std::sin(angle), 0.0);
}

// Where the rover is placed at t=0 -- further out on its own cycle-0 lane, clear of the
// builder. See robot_spawn_radius.
inline chrono::ChVector3d InitialSpawnPositionForRobot(int robot_index, int num_robots, int cycle = 0) {
    return PointOnHarvestLane(robot_index, num_robots, robot_spawn_radius, cycle);
}

// Radially outward, away from the site: the rock line runs into open field
// instead of back across the build area and its own builder.
inline double InitialHeadingRadForRobot(int robot_index, int num_robots, int cycle = 0) {
    return HarvestLaneAngleRad(robot_index, num_robots, cycle);
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

inline chrono::ChVector3d BuilderOrbitGroundPosition(int builder_index, int num_builders) {
    return PointOnSiteRay(builder_index, num_builders, builder_path_radius);
}

inline double BuilderOrbitHeadingRad(int builder_index, int num_builders) {
    // Counter-clockwise tangent to the orbit.
    return RankRayAngleRad(builder_index, num_builders) + 0.5 * chrono::CH_PI;
}

// ---------------------------------------------------------------------------
// The builder's own build plan.
//
// The builder runs this on its OWN schedule -- it is not triggered by, gated on, or
// indexed by the collector's harvest cycle. It walks counter-clockwise along its lane
// laying a course of rocks on the work circle, one slot at a time.
//
// EVERYTHING here is derived from one integer, the wall slot index k, so the arm, the
// drive station and the feedstock all agree by construction rather than by tuning:
//
//   wall slot k   -> work_circle_radius,      angle ray + k*slot_pitch_rad
//   station k     -> builder_path_radius,     angle ray + k*slot_pitch_rad + arm_lead
//   seed heap     -> builder_pile_radius,     angle of the station at its centre
//   load c        -> robot_start_radius,      angle of slot HarvestDropSlot(c)
//
// The last two are the SOURCES of rock, and there is only one of the first: a single
// seed heap of builder_seed_rock_count rocks. After that the builder eats what its
// collector drops on the collector circle, which HarvestDropSlot puts at the slot the
// builder will have reached. So both sources land in the same place relative to the
// arm base -- radially outboard of the station it is already driving to -- and neither
// needs the arm to leave its lane.
//
// arm_lead exists because the arm is mounted 2.5 m BACK along a hull parked tangentially,
// so the arm base sits at hypot(33, 2.5) = 33.095 m, trailing the hull's own station angle
// by atan(2.5/33). Adding it to the station means the ARM BASE, not the hull origin, ends
// up radially opposite the slot it is serving -- which is what makes both the reach to the
// work circle (3.095 m) and the reach to the heap (3.5 m) land mid-band every time.
// ---------------------------------------------------------------------------

// wall_slot_pitch_m / wall_slot_pitch_rad are declared with the harvest cycle above,
// because the harvest lane is measured in these slots.

// Where the seed heap sits: 3.5 m radially outboard of the arm base, mid-way through
// the 2.78-4.44 m band the arm proved in service. Close to robot_start_radius (37.0) on
// purpose -- the heap is standing in for a delivered load, so it should sit where one does.
inline constexpr double builder_pile_radius = 56.6;

// The seed heap serves the first builder_seed_rock_count slots and is centred on the
// middle of that run, so the builder reaches it from either end. The builder creeps
// ~1 m of lane per slot, so across six slots the heap goes from 2.5 slots ahead of the
// arm base to 2.5 behind: reach hypot(3.5, 2.7) = 4.4 m at the extremes, inside the
// 5.2 m guard, mid-band at the centre.
inline constexpr int wall_slots_per_pile = builder_seed_rock_count;
inline constexpr int builder_pile_count = 1;

// Trailing angle from the hull's station to its arm base. See the block comment.
inline constexpr double builder_arm_mount_back_m = 2.5;
inline double BuilderArmLeadRad() {
    return std::atan2(builder_arm_mount_back_m, builder_path_radius);
}

// Total slots this builder may lay. No longer tied to how many heaps were laid out --
// the feedstock is a stream now, not a fixed larder -- so this is purely the sector cap:
// with N builders each owns 2*pi/N of lane, and 0.7 of it leaves room for the machine
// itself (2.686 m wide) at either end. A rank's course must never run into the next
// rank's sector.
inline int BuilderWallSlotCount(int num_builders) {
    // Enough wall for a very long run; the cap below is what actually binds for N > 1.
    constexpr int desired = 200;
    if (num_builders <= 1)
        return desired;
    const double sector_rad = chrono::CH_2PI / static_cast<double>(num_builders);
    const int cap = static_cast<int>(0.7 * sector_rad / wall_slot_pitch_rad);
    return std::max(builder_seed_rock_count, std::min(desired, cap));
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
inline constexpr double builder_arm_base_radius_approx = 53.0589;  // hypot(53, 2.5)

// Centre of the seed heap serving slots [pile*P, pile*P + P), placed at the middle
// of that run so it is never more than ~2.7 m of lane from the arm base working it.
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

// ---------------------------------------------------------------------------
// SCM active-domain sizing
// ---------------------------------------------------------------------------
//
// An active domain does one job: it selects which grid nodes fire a ray this step.
// Soil is only computed where a domain covers it, so a domain must cover its body's
// contact footprint -- but every square metre it covers BEYOND that footprint is paid
// for on every step of the run, forever, and at 2 cm spacing a square metre is 2500
// nodes.
//
// The cost is not the ray. It is that each node's ray origin needs its current height,
// and SCMLoader::GetHeight() resolves that through m_grid_map.find() -- one random hash
// probe per node per step. m_grid_map holds every node ever deformed, so it GROWS: a
// 13.7 h two-rover run at 2 cm went 52.8k -> 891k entries (5 MB -> 85 MB), and once it
// outgrew L3 every probe became two DRAM round trips. Measured on that run: wall/sim
// climbed 70 -> 120 cumulative, which for linear growth means the instantaneous rate
// roughly tripled. 194k nodes/step x ~150 ns of added probe latency is ~29 ms/step,
// and at a 5e-4 s step that is ~58 wall-seconds per simulated second of pure overhead.
//
// So domains are sized to the footprint plus a stated margin, and nothing more. The
// margin only has to absorb geometry, not attitude: UpdateActiveDomain projects the
// eight corners of the ROTATED OOBB and takes their axis-aligned hull, so pitch and
// roll can only ever EXPAND the covered region. A box sized in the body frame cannot
// uncover its own contact patch.

// Rocks measure at most 0.263 x 0.284 x 0.227 m (read off a run's object manifest), so
// 0.6 m square leaves >= 0.16 m of margin on the widest one.
//
// Was 1.0 x 1.0 x 1.0 centred (0, 0, 0.3). Unrotated that is 2500 nodes against 900;
// under free rotation the projected hull is bounded by the box diagonal, so 7500 against
// 2700. Twelve rock domains were 90k of the 194k nodes cast per step on one rank -- the
// single largest term, larger than the eight wheels and the whole builder combined.
//
// The z extent is what reaches the soil, so it is kept only just deep enough: the box
// spans 0.2 m below the rock reference frame, exactly as the 1 m cube did.
inline const chrono::ChVector3d scm_rock_domain_dims(0.6, 0.6, 0.6);
inline const chrono::ChVector3d scm_rock_domain_center(0.0, 0.0, 0.1);

// Margin added around the measured track-shoe footprint when building a tracked vehicle's
// running-gear domain. Shoe positions give only the track centre lines, so this must exceed
// half an M113 single-pin shoe's width (~0.19 m); 0.35 m clears that with 0.16 m to spare
// outboard and adds 0.7 m of run-out fore and aft. Measured result: 5.167 x 2.859 m, which
// is 36933 nodes at 2 cm spacing against 84375 for the 7.5 x 4.5 m box it replaces, and
// still wider than the vehicle's widest shape (2.686 m) so nothing is uncovered.
inline constexpr double scm_track_domain_margin = 0.35;

// How far a track domain reaches below and above the shoe loop. The domain only has to
// contain the ray segments fired from the soil under it, and those span the surface by
// SCM's own test offsets; 1 m each way is far more than either needs.
inline constexpr double scm_track_domain_z_pad = 1.0;

}  // namespace amd_uw
