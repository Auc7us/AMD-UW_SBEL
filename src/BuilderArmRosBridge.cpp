#include "BuilderArmRosBridge.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>

#include "LrvArm.h"
#include "RobotLayout.h"

namespace amd_uw {

namespace {

// Vertical aim of the grab, relative to the rock's centre of mass. CALIBRATED, not
// derived -- the value is negative, which is not what reasoning about the arm predicted.
//
// The horizontal aim is already excellent: |gripper-rockREF|xy comes out at 0.006-0.017 m
// across all four builders. So when a grab fails with pad_force exactly 0 and the jaws
// shut to 0.0998 m on a 0.30 m rock, the whole miss is vertical, and the reported
// finger-to-rock distance at closure measures it directly. Two settings, 4 builders:
//
//     grab_z_offset = +0.05  ->  finger-rock 0.19-0.20 m at closure  (some grabs succeed)
//     grab_z_offset = +0.22  ->  finger-rock 0.28-0.34 m at closure  (none succeed)
//
// Raising the aim 0.17 m moved the gripper 0.15 m FURTHER from the rock, so the jaws were
// already above it, and the sag story that motivated +0.22 was simply wrong. The slope is
// near unity, so distance ~ |offset + 0.14| and the miss vanishes around -0.14. Backing
// off slightly from that leaves margin on both sides, since rocks in a heap differ in
// height by more than the residual error.
constexpr double grab_z_offset = -0.13;
// Release height above the terrain at the wall slot. The rock is let go from rest, so
// this is a short drop that lets it seat itself on the regolith rather than being
// pressed into it by the fingers.
constexpr double place_z_clearance = 0.20;

// Both at 50 Hz of sim time, matching the rover's bridge. The consumer is a 10 Hz Python
// node doing a blocking IK solve; publishing every 5e-4 s step is what buried it.
constexpr double build_publish_period = 0.02;

// Envelope a feedstock rock must be inside, measured from the arm's IK frame origin.
// LrvArm's own grab guard is [1.0, 2.6] m * geometry_scale 2.0 = [2.0, 5.2] m; this is
// that band pulled in slightly at the top so a rock is not offered at the exact radius
// where the solve is about to be refused. Nothing outside it is a candidate, which is
// also what keeps the builder from reaching for rocks still out on the collector's rock
// line 20 m away -- the reach test is the only filter needed.
constexpr double feedstock_reach_min = 2.0;
constexpr double feedstock_reach_max = 5.0;

// How long a parked builder may sit with an unlaid slot and nothing in reach before it
// says so. Not an error -- waiting for the collector is the normal state between loads --
// but a builder that waits forever is the failure this whole arrangement risks, so it
// has to be visible in the log rather than inferred from the wall not growing.
constexpr double starved_report_period = 30.0;

// How many times the arm may be sent after the same rock before it is written off. See
// the failure branch in Synchronize for why one attempt is not enough.
constexpr int max_grab_attempts = 3;

// FETCH. How far past feedstock_reach_max a rock may lie and still be worth moving the
// station for, and how far the station may be moved to get it.
//
// Deliveries do not land in the envelope, and it is the GEOMETRY that says why rather
// than any particular set of radii: the drop point sits on the collector ring while the
// arm base rides the builder orbit, so a load lands some way further OUT and some way
// ALONG. The radial part is inside reach on its own; the arc is what pushes the total
// over. So cancel the arc by sliding the station along to meet the rock, which costs a
// short creep and touches no site geometry.
//
// Two consequences, both of which have bitten:
//
// A SLIDE CAN ONLY CANCEL ARC. When a rock sits almost straight out from the arm base --
// same bearing, just further from the site centre -- the best available slide is zero,
// and sliding is simply not the fix. See the zero-slide guard where this is used: an
// attempt that moves nothing must not be charged against the slot, or the builder spends
// its single try accomplishing nothing and then waits forever. That is exactly what
// stranded one builder for 875 s with ten rocks lying 0.09 m outside the envelope.
//
// THE LIMIT IS METRES OF ARC, NOT RADIANS. It used to be a radian constant tuned when the
// orbit was smaller, and rescaling the site silently widened it -- the same angle went
// from about 3.3 m of arc to about 5.3 m, far enough to swing the builder's own wall slot
// out of reach behind it. Arc length stays honest when the radius changes; radians do not.
constexpr double fetch_reach_margin = 2.5;         // m past feedstock_reach_max
constexpr double station_fetch_max_arc_m = 3.3;    // m of arc along the builder orbit
// Below this the slide is not worth taking and, more importantly, must not be counted as
// the slot's one attempt.
constexpr double station_fetch_min_arc_m = 0.05;

// STARVED SLOT. How long a parked builder may hold a slot with nothing in reach before it
// gives up on that slot and moves to the next.
//
// Slot skips are acceptable by design -- the wall is filled by whoever reaches it, and a
// course with gaps is finished by a later pass -- whereas a builder frozen on one slot
// fills nothing for the rest of the run. This is the backstop for everything the fetch
// cannot fix. Two builders of run_20260829_060122 held their slots for 1065 s and 875 s
// with feedstock 0.17 m and 0.09 m outside the envelope and no move available that would
// have closed it. Long enough that a normal collector round trip never trips it.
constexpr double starved_slot_timeout = 240.0;  // s

// WHY A SLOT IS SKIPPED MATTERS, NOT JUST HOW LONG IT HAS WAITED.
//
// A bare timer cannot tell "the collector is out on a long haul and will be back" from
// "the collector is never coming back", and those want opposite answers: the first should
// wait however long it takes, the second should give up on the slot and keep building
// with whatever is already in reach. starved_slot_timeout on its own gets the first case
// wrong, skipping good slots simply because a harvest run was slow.
//
// So elapsed time is necessary but not sufficient. Past starved_slot_timeout, a slot is
// released only when something says the wait is futile:
//
//   1. the collector has not MOVED in collector_stall_timeout -- crashed, stranded, or
//      wedged, as rank 10 of run_20260830_073816 was for the last 300 s of its life;
//   2. a delivery actually landed while this slot was starved and STILL nothing came into
//      the 2.0-5.0 m window -- the collector is working fine and its drops are simply not
//      reachable, which is builder 2 of that run, parked beside a rock at 1.67 m;
//   3. starved_slot_backstop, the catch-all for a cause not enumerated here.
//
// A collector that is visibly driving and has not delivered yet matches none of these,
// and the builder waits, which is the point.
constexpr double collector_stall_timeout = 120.0;  // s without moving = not coming back
constexpr double collector_stall_move_m = 1.0;     // m; under this is "has not moved"
constexpr double starved_slot_backstop = 900.0;    // s

// HULL THAT CANNOT LEAVE ITS SLOT = FAILED RUN.
//
// Skipping slots is normal and healthy; a builder that cannot physically drive is not.
// The distinction is whether the hull is being TOLD to move: parked at a station waiting
// on feedstock is the expected state and must never trip this, however long it lasts, so
// the clock only runs while m_hull_parked is false -- that is, while the orbit controller
// is actively driving this builder to its next station.
//
// Measured on the arm base rather than the hull: the bridge is not handed a hull pose, and
// the arm is rigidly mounted, so its base moves with the machine. The same rigid mount is
// what makes RosArmBridge's DRIFT check meaningful.
constexpr double hull_stuck_timeout = 300.0;  // s commanded to drive, having not moved
constexpr double hull_stuck_move_m = 0.5;     // m

// COLLECTOR KEEP-OUT. Do not START a pick while this rank's collector is beside the arm.
//
// The arm swings a 2x-scaled boom through the space a rover parks in to pour, and it has
// no idea the rover is there: on run_20260829_060122 a builder closed on a rock during
// its own collector's drop and put the machine on its roof nine seconds later. Gating the
// START is enough -- a cycle already in progress finishes, because aborting mid-grab
// drops a carried rock, and between cycles the arm rests at LrvArm's stow pose, which is
// folded in over its own hull rather than extended over the pour.
//
// The timeout matters as much as the radius. A collector that has STOPPED beside its
// builder must not gate it for the rest of the run; that would re-create, from the other
// side, the same deadlock starved_slot_timeout exists to break.
constexpr double collector_keepout_m = 4.0;
constexpr double collector_keepout_max_s = 45.0;
constexpr double collector_gate_report_period = 15.0;

// STRANDED FEEDSTOCK. How far past a rock the builder must be before that rock is deleted,
// and where it goes.
//
// The orbit is one-way and it has to stay that way. Sixteen ranks share this lane, so a
// builder that doubles back to retrieve something drives into the station behind it, and
// the demo is a demo of ranks NOT colliding. A rock the builder has passed is therefore
// not merely inconvenient to reach, it is unreachable forever -- 15 deg is 8.6 m of arc at
// this radius against a 5.2 m arm -- so leaving it lying there only puts an object in the
// render that the machines visibly ignore.
//
// 15 deg also puts the deletion well behind the builder before it happens, so nothing
// vanishes in shot. The grace period is there for the same reason: part of the seed heap
// starts behind the arm base, and clearing it at t=0 would be a handful of rocks popping
// out of existence on the first frame.
//
// The stash is the same trick SynAgents uses for unused zombie rocks -- fixed, collision
// off, parked 5 km down. It costs nothing in the solver and it cannot be seen.
constexpr double stranded_clear_rad = 15.0 * chrono::CH_PI / 180.0;
constexpr double stranded_clear_grace = 30.0;  // s of sim before any rock is cleared
const chrono::ChVector3d stranded_stash_position(0.0, 0.0, -5000.0);

// How often a builder prints what it is holding, waiting for, and standing on. A 3 h run
// is unreadable without it and unanswerable afterwards.
constexpr double builder_status_period = 30.0;

// How long a parked builder may sit unable to reach its OWN WALL SLOT before the station
// is advanced past it.
//
// This closes a real deadlock, measured: builder 3 parked with its hull 0.93 m off the
// 33 m lane -- inside the orbit controller's take band, so it correctly reported holding
// station -- but yawed 16.3 deg off tangential. The arm mounts 2.5 m BACK along the hull,
// so that heading error swings the arm base 2.5*sin(16.3) = 0.70 m radially inward, to
// r = 31.47. Its wall slot at r = 30 was then 1.47 m away, under LrvArm's 2.0 m minimum,
// and the arm controller refused it at 10 Hz indefinitely. The hull was parked, so nothing
// drove; the controller was satisfied, so nothing re-acquired. It would have sat there for
// the rest of the run.
//
// The controller closes its loop on hull radius and orbit angle and never on hull HEADING,
// which is the term the arm base is most sensitive to. Fixing that properly means giving
// the drive law a heading target; this is the backstop that makes the failure recoverable
// instead of terminal, and it belongs here because the bridge is the only place that knows
// both the true arm base pose and where the slot is.
//
// Advancing the station is what breaks it: the builder has to DRIVE about a metre, which
// re-orients the hull along its lane, and the next slot is judged from the new pose. It
// costs a gap in the wall, which is logged and is honest about what happened.
constexpr double unservable_slot_timeout = 20.0;

// How close a feedstock rock must be to the position a pick/place command was solved for
// to count as the rock that command meant. Neighbouring rocks in the seed heap sit 0.30 m
// of clear air apart -- roughly 0.6 m centre to centre -- so this is comfortably tighter
// than the gap it has to tell apart, and comfortably looser than the few millimetres a
// fixed rock can move between the publish and the command arriving.
constexpr double command_rock_match_tol = 0.25;

void EnsureRosInitialized() {
    if (rclcpp::ok())
        return;

    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
}

std::string TopicForBuilder(int builder_id, const std::string& suffix) {
    return "/builder_" + std::to_string(builder_id) + "/" + suffix;
}

}  // namespace

BuilderArmRosBridge::BuilderArmRosBridge(int builder_id,
                                         LrvArm& arm,
                                         std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> seed_rocks,
                                         std::vector<chrono::ChVector3d> wall_slots)
    : m_builder_id(builder_id),
      m_arm(arm),
      m_feedstock(std::move(seed_rocks)),
      m_wall_slots(std::move(wall_slots)) {
    EnsureRosInitialized();

    m_feedstock.erase(std::remove(m_feedstock.begin(), m_feedstock.end(), nullptr), m_feedstock.end());

    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_node = rclcpp::Node::make_shared("chrono_builder_" + std::to_string(m_builder_id) + "_arm");
    m_state_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "arm_state"), 10);
    m_base_pose_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "arm_base_pose"), 10);
    m_pick_target_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "pick_target"), 10);
    m_place_target_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "place_target"), 10);
    m_status_pub = m_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "arm_status"), 10);
    m_command_sub = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(
        TopicForBuilder(m_builder_id, "arm_cmd"),
        10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnCommand(msg); });
    m_executor->add_node(m_node);

    RCLCPP_INFO(m_node->get_logger(),
                "Builder arm bridge ready: %zu wall slots to lay, %zu seed rocks to start from "
                "(the rest arrive by collector); subscribing %s, publishing %s / %s / %s",
                m_wall_slots.size(), m_feedstock.size(),
                TopicForBuilder(m_builder_id, "arm_cmd").c_str(),
                TopicForBuilder(m_builder_id, "pick_target").c_str(),
                TopicForBuilder(m_builder_id, "place_target").c_str(),
                TopicForBuilder(m_builder_id, "arm_status").c_str());
}

BuilderArmRosBridge::~BuilderArmRosBridge() {
    if (m_node && m_executor)
        m_executor->remove_node(m_node);
}

// Take in whatever the collector has delivered, then pick the rock to work next.
//
// "FURTHEST un-consumed rock inside the envelope" is the whole selection rule -- see
// SelectFeedstock for why furthest and not nearest. It has to
// be a search rather than an index because the second source of rock is a load tipped
// out of a moving trailer: those rocks land where they land, in whatever order they
// leave the bed, and nothing upstream can promise which one ends up closest. The seed
// heap could have kept its slot-indexed mapping, but running both sources through the
// same rule means there is only one behaviour to reason about, and the changeover from
// heap to delivery needs no handling at all -- the heap simply stops being the nearest
// thing once it is empty.
// Delete what the builder has driven past, and say what it is doing while it does it.
//
// Direction of travel is taken from the WALL, not from a parameter: the slots are laid in
// order, so the bearing from slot 0 to slot 1 is the way this builder goes round. That
// makes the test correct whichever way the orbit is configured, with nothing to keep in
// sync with the controller's counter_clockwise flag.
void BuilderArmRosBridge::ClearStrandedFeedstock(double time) {
    if (time < stranded_clear_grace || m_wall_slots.size() < 2)
        return;

    const auto bearing_of = [](const chrono::ChVector3d& p) {
        return std::atan2(p.y() - site_center_y, p.x() - site_center_x);
    };
    const auto wrap = [](double a) {
        while (a > chrono::CH_PI)
            a -= chrono::CH_2PI;
        while (a < -chrono::CH_PI)
            a += chrono::CH_2PI;
        return a;
    };
    const double travel = wrap(bearing_of(m_wall_slots[1]) - bearing_of(m_wall_slots[0])) >= 0.0 ? 1.0 : -1.0;
    const auto base = m_arm.GetIkFramePos();
    const double base_bearing = bearing_of(base);

    for (auto& rock : m_feedstock) {
        if (!rock || m_consumed.count(rock.get()))
            continue;
        const double past = travel * wrap(base_bearing - bearing_of(rock->GetPos()));
        if (past < stranded_clear_rad)
            continue;
        // Booked before it is stashed, so no later selection can offer a rock that is now
        // 5 km underground.
        m_consumed.insert(rock.get());
        ++m_stranded_count;
        const double reach = (rock->GetPos() - base).Length();
        RCLCPP_WARN(m_node->get_logger(),
                    "t=%.2f clearing %s: %.1f deg behind the arm base (%.2f m away), so it can "
                    "never be laid on a one-way orbit. %d cleared so far.",
                    time, rock->GetName().c_str(), past * 180.0 / chrono::CH_PI, reach,
                    m_stranded_count);
        rock->SetFixed(true);
        rock->EnableCollision(false);
        rock->SetFrameRefToAbs(chrono::ChFrame<>(stranded_stash_position, chrono::QUNIT));
    }

    if (m_last_status_report < 0.0 || time - m_last_status_report >= builder_status_period) {
        m_last_status_report = time;
        size_t spare = 0, in_reach = 0;
        double nearest = std::numeric_limits<double>::max();
        for (const auto& rock : m_feedstock) {
            if (!rock || m_consumed.count(rock.get()))
                continue;
            ++spare;
            const double d = (rock->GetPos() - base).Length();
            nearest = std::min(nearest, d);
            if (d >= feedstock_reach_min && d <= feedstock_reach_max)
                ++in_reach;
        }
        const double place_reach =
            BuildComplete() ? 0.0 : (m_wall_slots[m_placed_count] - base).Length();
        // "nearest 0.00 m" read as a measurement when it was a sentinel for "there are
        // none", which is the opposite of what it looks like at a glance.
        char nearest_txt[32] = "n/a";
        if (spare > 0)
            std::snprintf(nearest_txt, sizeof(nearest_txt), "%.2f m", nearest);
        RCLCPP_INFO(m_node->get_logger(),
                    "t=%.2f status: slot %d/%zu parked=%d arm_busy=%d | feedstock %zu known, "
                    "%zu spare, %zu in reach, nearest %s, %d cleared | slot is %.2f m away | "
                    "base r=%.2f m, fetch offset %+.2f deg",
                    time, m_placed_count, m_wall_slots.size(), m_hull_parked ? 1 : 0,
                    m_arm.IsBusy() ? 1 : 0, m_feedstock.size(), spare, in_reach,
                    nearest_txt, m_stranded_count, place_reach,
                    std::hypot(base.x() - site_center_x, base.y() - site_center_y),
                    m_station_fetch_offset * 180.0 / chrono::CH_PI);
    }
}

void BuilderArmRosBridge::UpdateFeedstock(double time) {
    // Ask the collector at the publish rate, not at the 2 kHz step rate: the query
    // allocates and copies a vector, and a load cannot land twice in 20 ms of sim.
    // The reach search below still runs every step, so the rock on offer tracks the arm
    // base as the hull settles.
    if (m_delivered_source && (m_last_feed_refresh < 0.0 || time - m_last_feed_refresh >= build_publish_period)) {
        m_last_feed_refresh = time;
        for (auto& rock : m_delivered_source()) {
            if (!rock)
                continue;
            if (std::find(m_feedstock.begin(), m_feedstock.end(), rock) == m_feedstock.end()) {
                m_feedstock.push_back(rock);
                // A delivered rock is frozen where it stopped rolling (RobotRig::
                // GetDeliveredRocks), so this distance is final: it is the verdict on the
                // collector's drop, one line per rock, whether or not the builder is
                // starving at the time. Reported here because nothing else in the loop
                // can: the pick log only ever names rocks that WERE in reach, so a drop
                // that lands short is silent apart from a wall that stops growing.
                const double d = (rock->GetPos() - m_arm.GetIkFramePos()).Length();
                RCLCPP_INFO(m_node->get_logger(),
                            "t=%.2f delivered rock [%s] came to rest %.2f m from the arm base "
                            "-- %s the %.1f-%.1f m envelope.",
                            time, rock->GetName().c_str(), d,
                            (d >= feedstock_reach_min && d <= feedstock_reach_max) ? "INSIDE"
                                                                                  : "OUTSIDE",
                            feedstock_reach_min, feedstock_reach_max);
            }
        }
    }

    m_selected.reset();

    // FURTHEST in reach first, not nearest.
    //
    // Reachability is perishable at the far edge and permanent at the near edge. The
    // builder works a station, then advances along its orbit, and the heap it is eating
    // from does not move -- so a rock at 4.9 m of a 2.0-5.0 m envelope is the first thing
    // to fall outside it, while a rock at 2.5 m stays available for several stations. Take
    // the nearest first and the far ones are simply lost: never grabbed, never laid, and
    // the wall goes short while usable rock sits on the ground.
    //
    // This costs nothing in grab quality, which was the obvious worry. Measured over 30
    // builder grabs across four builds, reach does not predict the lateral grab error at
    // all: correlation -0.087, and the grabs that missed badly sat at a median reach of
    // 4.211 m against 4.197 m for the good ones -- indistinguishable. The arm is no worse
    // near full extension than it is folded up.
    const auto base = m_arm.GetIkFramePos();
    double best = -1.0;
    for (const auto& rock : m_feedstock) {
        if (!rock || m_consumed.count(rock.get()))
            continue;
        const auto pos = rock->GetPos();
        if (!std::isfinite(pos.x()) || !std::isfinite(pos.y()) || !std::isfinite(pos.z()))
            continue;
        const double d = (pos - base).Length();
        if (d < feedstock_reach_min || d > feedstock_reach_max)
            continue;
        if (d > best) {
            best = d;
            m_selected = rock;
        }
    }
}

std::shared_ptr<chrono::ChBodyAuxRef> BuilderArmRosBridge::FindFeedstockNear(double x, double y) const {
    std::shared_ptr<chrono::ChBodyAuxRef> best;
    double best_d2 = command_rock_match_tol * command_rock_match_tol;
    for (const auto& rock : m_feedstock) {
        if (!rock || m_consumed.count(rock.get()))
            continue;
        const auto pos = rock->GetPos();
        const double dx = pos.x() - x;
        const double dy = pos.y() - y;
        const double d2 = dx * dx + dy * dy;
        if (d2 <= best_d2) {
            best_d2 = d2;
            best = rock;
        }
    }
    return best;
}

int BuilderArmRosBridge::ReadySlot() const {
    if (!m_hull_parked || m_arm.IsBusy())
        return -1;
    if (m_placed_count >= static_cast<int>(m_wall_slots.size()))
        return -1;
    if (!m_selected)
        return -1;
    return m_placed_count;
}

void BuilderArmRosBridge::Synchronize(double time, bool apply_commands) {
    m_last_time = time;
    m_executor->spin_some();
    // Before anything reads ReadySlot(): both the command path and the publish path
    // depend on which rock is on offer, and they must agree within a step.
    if (!m_arm.IsBusy())
        UpdateFeedstock(time);
    // After the reach search, so a rock the arm has just been sent for is already booked
    // and cannot be stashed out from under it.
    ClearStrandedFeedstock(time);

    std::optional<DirectCommand> direct;
    std::optional<PickPlaceCommand> pick_place;
    {
        std::lock_guard<std::mutex> lock(m_command_mutex);
        direct = m_pending_direct;
        pick_place = m_pending_pick_place;
        m_pending_direct.reset();
        m_pending_pick_place.reset();
    }

    if (!apply_commands) {
        // Settle window: drop whatever arrived; the arm keeps the pose it already has.
        PublishBuildTopics(time);
        PublishStateThrottled(time);
        return;
    }

    if (direct) {
        m_arm.SetJointTargets(direct->theta);
        m_arm.SetFingerClosure(direct->finger_closure);
    }

    if (pick_place && !m_arm.IsBusy() && pick_place->command_seq > m_last_started_seq) {
        const int slot = ReadySlot();
        // Accept only a command for the slot actually being worked. A stale or duplicate
        // command would otherwise re-pick a rock already laid -- which, because that rock
        // was re-fixed on the wall, means welding the gripper to the wall.
        // Identify the rock by the POSITION the command was solved for, not by re-running
        // the selection. The controller solved its IK against the pick_target this bridge
        // published; m_selected is recomputed every step against a base that is still
        // settling, so two nearly-equidistant heap rocks can swap between the publish and
        // the command arriving. Handing the arm a rock 0.6 m from the one the joint angles
        // were solved for is a miss by construction. The command already carries that
        // position -- data[2..3] -- so use it.
        const auto commanded = FindFeedstockNear(pick_place->rock_x, pick_place->rock_y);
        if (slot >= 0 && pick_place->target_index == slot && commanded) {
            m_last_started_seq = pick_place->command_seq;
            m_active_slot = slot;
            m_active_rock = commanded;
            // Booked now, not on completion: the moment the arm starts for this rock,
            // no later selection may offer it again, whatever happens to the grab.
            m_consumed.insert(m_active_rock.get());
            const auto rock_pos = m_active_rock->GetPos();
            const chrono::ChVector3d grab_target(rock_pos.x(), rock_pos.y(), rock_pos.z() + grab_z_offset);
            const chrono::ChVector3d place_target = m_wall_slots[slot];
            RCLCPP_INFO(m_node->get_logger(),
                        "build step %d/%zu: rock=(%.3f, %.3f, %.3f) -> slot=(%.3f, %.3f, %.3f)",
                        slot + 1, m_wall_slots.size(), grab_target.x(), grab_target.y(), grab_target.z(),
                        place_target.x(), place_target.y(), place_target.z());
            m_arm.StartPickPlace(pick_place->command_seq, slot, m_active_rock, grab_target, place_target, time,
                                 &pick_place->grab_theta, &pick_place->place_theta);
        } else if (slot >= 0 && pick_place->target_index == slot && !commanded) {
            RCLCPP_WARN(m_node->get_logger(),
                        "Ignoring pick/place for slot %d; no un-consumed rock within %.2f m of the "
                        "commanded (%.3f, %.3f). The pile moved or the command is stale.",
                        slot, command_rock_match_tol, pick_place->rock_x, pick_place->rock_y);
        } else if (pick_place->target_index != slot) {
            RCLCPP_WARN(m_node->get_logger(),
                        "Ignoring pick/place for slot %d; the builder is %s.",
                        pick_place->target_index,
                        slot < 0 ? (m_hull_parked ? "waiting for a rock in reach" : "still driving to station")
                                 : "working a different slot");
        }
    }

    // LrvArm::Update is issued once per step by BuilderRig, after this returns, so the
    // status read below is one 5e-4 s step old. That is deliberate -- two Update calls
    // in a step would advance the slew twice -- and a step of lag on booking a
    // completion changes nothing.

    // Book the completion once, on the transition. A laid rock is FIXED again: it is now
    // part of the wall, so it must not be nudged by the next rock landing beside it, and
    // it costs the solver nothing sitting there for the rest of the run. This is also
    // what guarantees the invariant the heap starts with -- at most one rock in this
    // rank's build is a dynamic body at any moment.
    const auto status = m_arm.GetStatus();
    if (m_active_slot >= 0 && status.command_seq == m_last_started_seq && m_settled_seq != m_last_started_seq &&
        (status.state == 2 || status.state == 3)) {
        m_settled_seq = m_last_started_seq;
        auto rock = m_active_rock;
        // Fixed again either way. On success it is part of the wall, so it must not be
        // nudged by the next rock landing beside it. On failure it is back in the pile,
        // where the invariant is the same: nothing moves but what the gripper holds.
        if (rock) {
            rock->SetFixed(true);
            rock->EnableCollision(true);
        }
        if (status.state == 2 && status.success) {
            m_placed_count = m_active_slot + 1;
            // The fetch nudge belonged to the slot just filled. Carry it into the next
            // slot and the builder parks off its own wall slot.
            m_station_fetch_offset = 0.0;
            // Sim time, not the log's wall stamp: this runs ~30x slower than real time and
            // the ratio moves with rank count, so wall stamps cannot be compared between
            // builders or between runs. Every timing question about this cycle is asked
            // of this number.
            // The rock's NAME, not just the count. Rock bodies are named at construction
            // (seed_rock_b* for the starting heap, harvest_rock_r*_c* for anything a
            // collector delivered), and the count alone cannot answer the only question
            // that matters about the loop: was this stone delivered, or was it one the
            // builder started with? A stranded seed rock also makes "rock 7 or later"
            // useless as a proxy, because the builder lays whatever it can reach next --
            // a delivery can arrive as rock 6.
            RCLCPP_INFO(m_node->get_logger(),
                        "t=%.2f laid rock %d of %zu [%s]; station advances to slot %d.",
                        time, m_placed_count, m_wall_slots.size(),
                        rock ? rock->GetName().c_str() : "unnamed", m_placed_count);
        } else {
            // BOUNDED RETRY, not a write-off on the first miss.
            //
            // Writing the rock off immediately was chosen to stop a builder spending a
            // whole run on one unreachable stone, and that risk is real -- but the cost of
            // the cure is worse, because booking happens the moment the arm STARTS and
            // nothing ever un-books it. One failed grab retires the rock permanently.
            // Measured on run_20260825_221818: rank 1 laid 5 of its 6 seed rocks and then
            // sat still for the remaining 490 s of sim with its sixth rock lying 4.73 m
            // from the arm base -- inside the 2.0-5.0 m envelope, unlaid, and unofferable.
            // Furthest-first makes that worse by aiming the first attempt at exactly the
            // rock most likely to be missed.
            //
            // So count attempts per rock and give it back to the pool until the count runs
            // out. The slot is not skipped on a retry either: the same slot is still owed a
            // rock, and skipping it would leave a hole in the wall for a grab that is about
            // to be tried again.
            const int tries = rock ? ++m_grab_attempts[rock.get()] : max_grab_attempts;
            if (tries < max_grab_attempts) {
                if (rock)
                    m_consumed.erase(rock.get());
                RCLCPP_WARN(m_node->get_logger(),
                            "t=%.2f slot %d failed (error_code=%d) on attempt %d of %d; "
                            "returning that rock to the pile and retrying the same slot.",
                            time, m_active_slot, status.error_code, tries, max_grab_attempts);
            } else {
                m_placed_count = m_active_slot + 1;
                m_station_fetch_offset = 0.0;
                RCLCPP_WARN(m_node->get_logger(),
                            "t=%.2f slot %d failed (error_code=%d) on attempt %d of %d; writing "
                            "that rock off and moving to slot %d.",
                            time, m_active_slot, status.error_code, tries, max_grab_attempts,
                            m_placed_count);
            }
        }
        // This rock's state is ours now. Tell the arm to let go of its reference, or the
        // next StartPickPlace's RemoveRockLock() would unfix the stone we just laid.
        m_arm.ForgetTargetRock();
        m_active_slot = -1;
        m_active_rock.reset();
    }

    if (!m_reported_complete && BuildComplete()) {
        m_reported_complete = true;
        RCLCPP_INFO(m_node->get_logger(), "course complete: %zu rocks laid on the work circle.",
                    m_wall_slots.size());
    }

    PublishBuildTopics(time);
    PublishStateThrottled(time);
}

void BuilderArmRosBridge::OnCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!std::all_of(msg->data.begin(), msg->data.end(), [](double value) { return std::isfinite(value); })) {
        RCLCPP_WARN(m_node->get_logger(), "Ignoring arm_cmd containing a non-finite value.");
        return;
    }

    if (msg->data.size() == 5) {
        DirectCommand command;
        command.theta = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};
        // Matches LrvArm's physical finger travel used by the reference model.
        command.finger_closure = std::clamp(msg->data[4], 0.0, 0.05);
        std::lock_guard<std::mutex> lock(m_command_mutex);
        m_pending_direct = command;
        return;
    }

    if (msg->data.size() >= 12) {
        PickPlaceCommand command;
        command.command_seq = msg->data[0];
        command.target_index = static_cast<int>(std::llround(msg->data[1]));
        // data[2..3] are the rock's world x/y: the position the controller solved this
        // command's joint angles against, and therefore what identifies WHICH rock it
        // means. The rock's exact pose is still read off the body at StartPickPlace.
        command.rock_x = msg->data[2];
        command.rock_y = msg->data[3];
        command.grab_theta = {msg->data[4], msg->data[5], msg->data[6], msg->data[7]};
        command.place_theta = {msg->data[8], msg->data[9], msg->data[10], msg->data[11]};
        std::lock_guard<std::mutex> lock(m_command_mutex);
        m_pending_pick_place = command;
        return;
    }

    RCLCPP_WARN(m_node->get_logger(),
                "Ignoring arm_cmd of %zu elements; expected 5 ([theta1..4, finger_closure_m]) or "
                "12 ([seq, target_index, rock_x, rock_y, grab_theta1..4, place_theta1..4]).",
                msg->data.size());
}

void BuilderArmRosBridge::PublishBuildTopics(double time) {
    if (m_last_build_pub_time >= 0.0 && time - m_last_build_pub_time < build_publish_period)
        return;
    m_last_build_pub_time = time;

    const auto pos = m_arm.GetIkFramePos();
    const auto rot = m_arm.GetIkFrameRot();
    std_msgs::msg::Float64MultiArray base;
    base.data = {pos.x(), pos.y(), pos.z(), rot.e0(), rot.e1(), rot.e2(), rot.e3()};
    m_base_pose_pub->publish(base);

    // ready=0 means "do not solve": the hull is still driving to station, the arm is
    // already working, the course is finished, or nothing is in reach. The x/y/z are
    // still sent so a stalled builder can be diagnosed from the topic instead of the log.
    int slot = ReadySlot();

    // COLLECTOR KEEP-OUT. Hold the arm while this rank's rover is beside it -- see the
    // constants block. Only the START is gated: a cycle already running finishes, and
    // between cycles the arm is folded at LrvArm's stow pose rather than reaching out
    // over the pour, so holding here leaves it somewhere safe by construction.
    if (m_collector_probe) {
        const auto gate_base = m_arm.GetIkFramePos();
        double nearest = std::numeric_limits<double>::max();
        for (const auto& pt : m_collector_probe())
            nearest = std::min(nearest, (pt - gate_base).Length());
        if (nearest <= collector_keepout_m) {
            if (m_arm_gate_since < 0.0)
                m_arm_gate_since = time;
            const double held = time - m_arm_gate_since;
            const bool release = held >= collector_keepout_max_s;
            if (!release)
                slot = -1;
            if (time - m_last_gate_report >= collector_gate_report_period) {
                m_last_gate_report = time;
                if (release)
                    RCLCPP_WARN(m_node->get_logger(),
                                "t=%.2f collector has been %.2f m from the arm base for %.0f s; "
                                "releasing the keep-out rather than let a machine that has "
                                "stopped freeze this builder for the rest of the run.",
                                time, nearest, held);
                else
                    RCLCPP_INFO(m_node->get_logger(),
                                "t=%.2f holding the arm: collector %.2f m from the arm base "
                                "(keep-out %.1f m), held %.0f s.",
                                time, nearest, collector_keepout_m, held);
            }
        } else {
            m_arm_gate_since = -1.0;
        }
    }

    const int slot_for_geometry = std::min(std::max(m_placed_count, 0),
                                           static_cast<int>(m_wall_slots.size()) - 1);
    std_msgs::msg::Float64MultiArray pick;
    if (slot_for_geometry >= 0 && m_selected) {
        const auto rock_pos = m_selected->GetPos();
        pick.data = {slot >= 0 ? 1.0 : 0.0, static_cast<double>(slot_for_geometry), rock_pos.x(), rock_pos.y(),
                     rock_pos.z() + grab_z_offset};
    } else {
        pick.data = {0.0, -1.0, 0.0, 0.0, 0.0};
    }
    m_pick_target_pub->publish(pick);

    // Parked but unable to reach the slot it is here to fill: advance past it rather than
    // stare at it forever. See unservable_slot_timeout for the pose that produces this.
    if (m_hull_parked && !m_arm.IsBusy() && !BuildComplete()) {
        const double place_reach = (m_wall_slots[m_placed_count] - pos).Length();
        const bool servable = place_reach >= feedstock_reach_min && place_reach <= feedstock_reach_max;
        if (!servable) {
            if (m_unservable_since < 0.0)
                m_unservable_since = time;
            else if (time - m_unservable_since >= unservable_slot_timeout) {
                RCLCPP_WARN(m_node->get_logger(),
                            "t=%.2f slot %d is %.2f m from the arm base (need %.1f-%.1f m) and has "
                            "been for %.0f s; skipping it so the hull has to move.",
                            time, m_placed_count, place_reach, feedstock_reach_min,
                            feedstock_reach_max, time - m_unservable_since);
                m_placed_count += 1;
                m_unservable_since = -1.0;
            }
        } else {
            m_unservable_since = -1.0;
        }
    } else {
        m_unservable_since = -1.0;
    }

    // Starvation belongs to the SLOT, not to the hull's instantaneous parked flag. A
    // builder waiting on feedstock jitters in and out of parked -- builder 10 of
    // run_20260830_073816 flipped a dozen times over the 270 s it sat on slot 11 -- and
    // clearing the clock on each flip meant starved_slot_timeout below could never
    // mature, in precisely the case it was written for. So the clock is reset by the two
    // things that actually mean this slot is no longer starved: the slot advancing, or
    // the arm getting hold of a rock. Not by the hull twitching.
    if (m_starved_slot != m_placed_count) {
        m_starved_slot = m_placed_count;
        m_starved_since = -1.0;
    }
    if (m_arm.IsBusy() || m_selected)
        m_starved_since = -1.0;

    // Is this hull moving when it is being driven? Only while NOT parked: a builder
    // holding station for feedstock is doing its job, not failing.
    {
        const auto here = m_arm.GetIkFramePos();
        if (!m_hull_seen) {
            m_hull_seen = true;
            m_hull_last_pos = here;
            m_hull_last_move = time;
        } else if ((here - m_hull_last_pos).Length() >= hull_stuck_move_m) {
            m_hull_last_pos = here;
            m_hull_last_move = time;
        }
        if (m_hull_parked) {
            m_hull_last_move = time;  // parked on purpose; the clock does not run
        } else if (!m_hull_stuck && time - m_hull_last_move >= hull_stuck_timeout) {
            m_hull_stuck = true;
            RCLCPP_ERROR(m_node->get_logger(),
                         "t=%.2f HULL STUCK: driven towards slot %d for %.0f s without moving "
                         "%.2f m. This builder cannot leave its station, so the run has failed. "
                         "(A builder merely WAITING on its collector never reaches here.)",
                         time, m_placed_count, time - m_hull_last_move, hull_stuck_move_m);
        }
    }

    // Is this rank's collector moving? Distinguishes a long haul from a dead machine.
    if (m_collector_probe) {
        const auto pts = m_collector_probe();
        if (!pts.empty()) {
            if (!m_collector_seen) {
                m_collector_seen = true;
                m_collector_last_pos = pts.front();
                m_collector_last_move = time;
            } else if ((pts.front() - m_collector_last_pos).Length() >= collector_stall_move_m) {
                m_collector_last_pos = pts.front();
                m_collector_last_move = time;
            }
        }
    }

    // Parked, still owing the wall a rock, and nothing in reach: the builder is waiting
    // on its collector. Normal between loads, fatal if it never ends, so it is reported
    // rather than left to be inferred from a wall that stopped growing.
    if (m_hull_parked && !m_arm.IsBusy() && !m_selected && !BuildComplete()) {
        if (m_starved_since < 0.0) {
            m_starved_since = time;
            // Baseline for "a load arrived and still did not help". Counted at the
            // instant the wait starts, so a later rise means a real delivery landed.
            m_spare_at_starve = 0;
            for (const auto& rock : m_feedstock)
                if (rock && !m_consumed.count(rock.get()))
                    ++m_spare_at_starve;
        }
        if (time - m_last_starved_report >= starved_report_period) {
            m_last_starved_report = time;
            size_t spare = 0;
            // How far off the nearest one is, not just that there is no rock in reach:
            // "none within 2.0-5.0 m" cannot tell a pile dropped a metre short from a
            // stranded seed rock 8 m away, and those need opposite fixes.
            double nearest = std::numeric_limits<double>::max();
            const auto arm_base = m_arm.GetIkFramePos();
            for (const auto& rock : m_feedstock) {
                if (rock && !m_consumed.count(rock.get())) {
                    ++spare;
                    nearest = std::min(nearest, (rock->GetPos() - arm_base).Length());
                }
            }
            char nearest_txt[64] = "n/a";
            if (spare > 0)
                std::snprintf(nearest_txt, sizeof(nearest_txt), "%.2f m", nearest);
            RCLCPP_INFO(m_node->get_logger(),
                        "waiting at slot %d for %.0f s: %zu rock(s) known, nearest %s, none within "
                        "%.1f-%.1f m of the arm base.",
                        m_placed_count, time - m_starved_since, spare, nearest_txt,
                        feedstock_reach_min, feedstock_reach_max);
        }

        // Starved with a rock lying just out of reach: slide the station along to meet it
        // rather than wait for a delivery that has already arrived. Taken once per slot --
        // see m_fetch_offset_slot -- because the rock stops being a near miss the instant
        // the nudge works, and recomputing then would undo it.
        if (m_fetch_offset_slot != m_placed_count) {
            const auto arm_base = m_arm.GetIkFramePos();
            const chrono::ChBodyAuxRef* best = nullptr;
            double best_d = std::numeric_limits<double>::max();
            for (const auto& rock : m_feedstock) {
                if (!rock || m_consumed.count(rock.get()))
                    continue;
                const double d = (rock->GetPos() - arm_base).Length();
                if (d <= feedstock_reach_max || d > feedstock_reach_max + fetch_reach_margin)
                    continue;
                if (d < best_d) {
                    best_d = d;
                    best = rock.get();
                }
            }
            if (best) {
                // Only the ARC is cancelled. The station is an orbit angle, so this moves
                // the arm base along its lane towards the rock's bearing and leaves the
                // radius -- which is already inside reach -- alone.
                const double rock_bearing =
                    std::atan2(best->GetPos().y() - site_center_y, best->GetPos().x() - site_center_x);
                const double base_bearing =
                    std::atan2(arm_base.y() - site_center_y, arm_base.x() - site_center_x);
                double delta = rock_bearing - base_bearing;
                while (delta > chrono::CH_PI)
                    delta -= chrono::CH_2PI;
                while (delta < -chrono::CH_PI)
                    delta += chrono::CH_2PI;
                // Clamp in ARC, not angle, so the limit keeps its meaning if the site is
                // ever rescaled again.
                const double base_radius = std::hypot(arm_base.x() - site_center_x,
                                                      arm_base.y() - site_center_y);
                const double want = std::clamp(delta,
                                               -station_fetch_max_arc_m / std::max(1.0, base_radius),
                                               station_fetch_max_arc_m / std::max(1.0, base_radius));
                const double arc = std::abs(want) * base_radius;
                if (arc < station_fetch_min_arc_m) {
                    // Purely radial miss. Sliding along the orbit cannot close it, so this
                    // does NOT count as the slot's attempt -- leaving m_fetch_offset_slot
                    // alone lets a later delivery that IS offset along the arc still be
                    // fetched. If none ever is, starved_slot_timeout releases the slot.
                    // Rate-limited: this branch is re-evaluated every step, and the rock
                    // is not moving, so the verdict is the same until a delivery changes
                    // it. Unthrottled it produced 955 identical lines in 62 s of sim.
                    if (time - m_last_zero_slide_report >= starved_report_period) {
                        m_last_zero_slide_report = time;
                        RCLCPP_INFO(m_node->get_logger(),
                                    "t=%.2f rock at %.2f m is %.2f m past the %.1f m limit but lies "
                                    "almost straight out (%.3f m of arc to gain); sliding cannot "
                                    "reach it, so the slot keeps its fetch.",
                                    time, best_d, best_d - feedstock_reach_max, feedstock_reach_max, arc);
                    }
                } else {
                    m_station_fetch_offset = want;
                    m_fetch_offset_slot = m_placed_count;
                    RCLCPP_INFO(m_node->get_logger(),
                                "t=%.2f fetching: rock at %.2f m is %.2f m past the %.1f m limit; "
                                "sliding the station %+.1f deg (%.2f m of arc) to reach it.",
                                time, best_d, best_d - feedstock_reach_max, feedstock_reach_max,
                                m_station_fetch_offset * 180.0 / chrono::CH_PI, arc);
                }
            }
        }

        // Nothing has come into reach for a long time and no slide is going to change
        // that: abandon the slot and move on. See starved_slot_timeout.
        if (m_starved_since >= 0.0 && time - m_starved_since >= starved_slot_timeout
            && !BuildComplete()) {
            // How many unconsumed rocks this builder knows about right now. A rise since
            // starvation began means a delivery landed and still did not reach us.
            size_t spare_now = 0;
            for (const auto& rock : m_feedstock)
                if (rock && !m_consumed.count(rock.get()))
                    ++spare_now;

            const bool collector_stalled =
                m_collector_seen && time - m_collector_last_move >= collector_stall_timeout;
            const bool delivery_missed = spare_now > m_spare_at_starve;
            const bool backstop = time - m_starved_since >= starved_slot_backstop;

            if (collector_stalled || delivery_missed || backstop) {
                RCLCPP_WARN(m_node->get_logger(),
                            "t=%.2f slot %d has had nothing within %.1f-%.1f m of the arm base "
                            "for %.0f s and %s; abandoning it and moving to slot %d. The wall "
                            "keeps the gap.",
                            time, m_placed_count, feedstock_reach_min, feedstock_reach_max,
                            time - m_starved_since,
                            collector_stalled ? "its collector has stopped moving"
                            : delivery_missed ? "a load arrived and still landed out of reach"
                                              : "the backstop expired",
                            m_placed_count + 1);
                m_placed_count += 1;
                m_starved_since = -1.0;
                m_fetch_offset_slot = -1;
                m_station_fetch_offset = 0.0;
            }
        }
    }

    std_msgs::msg::Float64MultiArray place;
    if (slot_for_geometry >= 0) {
        const auto& p = m_wall_slots[slot_for_geometry];
        place.data = {p.x(), p.y(), p.z()};
    } else {
        place.data = {0.0, 0.0, 0.0};
    }
    m_place_target_pub->publish(place);

    // Rocks this builder still owns and has not laid. NOT filtered by reach: the whole
    // point of sending it is to tell the collector when a fresh load is needed, and the
    // in-reach count is zero exactly when the builder is starving -- which is too late to
    // start driving. m_feedstock only grows, so this is (pool - consumed).
    int usable_rocks = 0;
    for (const auto& rock : m_feedstock) {
        if (rock && !m_consumed.count(rock.get()))
            ++usable_rocks;
    }

    // Angle of the wall slot being worked, about the site centre. Sent as an angle rather
    // than left for the collector to rebuild from ray + slot * pitch + arm_lead: the lead
    // exists precisely because the hull and the slot are NOT at the same angle, so a
    // consumer reconstructing it has three chances to get the convention wrong. Clamped
    // at the end of the course so a finished builder reports its last slot instead of
    // running off the vector.
    double slot_angle = 0.0;
    if (!m_wall_slots.empty()) {
        const size_t slot_index =
            std::min(static_cast<size_t>(std::max(0, m_placed_count)), m_wall_slots.size() - 1);
        const auto& slot = m_wall_slots[slot_index];
        slot_angle = std::atan2(slot.y() - site_center_y, slot.x() - site_center_x);
    }

    const auto status = m_arm.GetStatus();
    std_msgs::msg::Float64MultiArray status_msg;
    status_msg.data = {
        status.command_seq,
        static_cast<double>(status.state),
        static_cast<double>(status.target_index),
        status.success ? 1.0 : 0.0,
        static_cast<double>(status.error_code),
        // Simulation time. The arm advances on the sim clock, so that is the only clock
        // a supervising controller's deadline can sensibly use -- wall time is ~20x
        // faster here and the ratio moves with rank count and machine.
        m_last_time,
        // APPENDED, so anything reading indices 0-5 is unaffected. These three are for
        // the collector's return leg: where this builder is consuming, how much it has
        // left, and therefore where the next load has to land.
        static_cast<double>(m_placed_count),
        static_cast<double>(usable_rocks),
        slot_angle,
    };
    m_status_pub->publish(status_msg);
}

// See the note in BuilderVehicleRosBridge::Synchronize: publishing arm_state at the
// 2 kHz step rate was part of what saturated Fast-DDS.
void BuilderArmRosBridge::PublishStateThrottled(double time) {
    constexpr double publish_period = 1.0 / 200.0;
    if (m_last_publish_time >= 0.0 && time - m_last_publish_time < publish_period)
        return;
    m_last_publish_time = time;
    PublishState();
}

void BuilderArmRosBridge::PublishState() {
    const auto state = m_arm.GetActuatorSnapshot();
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        state.joint_angles[0],
        state.joint_angles[1],
        state.joint_angles[2],
        state.joint_angles[3],
        state.finger_positions[0],
        state.finger_positions[1],
        state.end_effector_position.x(),
        state.end_effector_position.y(),
        state.end_effector_position.z(),
    };
    m_state_pub->publish(msg);
}

}  // namespace amd_uw
