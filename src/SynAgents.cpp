#include "SynAgents.h"

#include <array>
#include <iostream>
#include <stdexcept>
#include <utility>

#include "MaterialUtils.h"
#include "RobotLayout.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChTypes.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono_synchrono/flatbuffer/message/SynApproachMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMessageUtils.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"

namespace amd_uw {

namespace {

// Harvest cycles' worth of zombie rocks rank 0 pre-creates per rank. They cannot be
// created on demand (see InitializeZombie), and each one costs a body on the sensor
// rank, so this is sized to the longest run seen (5 cycles) plus headroom.
constexpr int zombie_rock_cycle_capacity = 12;

// Where a zombie waits until a message places it. Below all terrain, so it is out of
// every view. Not (0,0,0): that is the site centre, where an un-updated zombie would
// look like a placement bug rather than a missing message.
const chrono::ChVector3d zombie_stash_position(0.0, 0.0, -5000.0);

// Shape-name substrings picking one part out of a stock zombie assembly. No running
// gear mesh contains either, so both matches are unambiguous.
const char* const tracked_hull_shape_name = "Chassis";
const char* const trailer_chassis_shape_name = "trailer_chassis";

}  // namespace

AmdTrackedVehicleAgent::AmdTrackedVehicleAgent(chrono::vehicle::ChTrackedVehicle* vehicle,
                                               const std::string& filename)
    : chrono::synchrono::SynTrackedVehicleAgent(vehicle, filename) {}

void AmdTrackedVehicleAgent::InitializeZombie(chrono::ChSystem* system) {
    const size_t bodies_before = system->GetBodies().size();

    chrono::synchrono::SynTrackedVehicleAgent::InitializeZombie(system);

    // The bodies that appeared are the ones the base just built. See the class
    // comment for why they are found this way.
    const auto& bodies = system->GetBodies();
    const chrono::ChColor color = RankColor(m_agent_key.GetNodeID() - 1);
    int hull_shapes_painted = 0;
    for (size_t i = bodies_before; i < bodies.size(); ++i)
        hull_shapes_painted += ApplyColorToVisualShapes(bodies[i], color, tracked_hull_shape_name);

    // Report only the miss: a filter that matches nothing is silent otherwise, and
    // an unpainted hull looks like a colour bug rather than a lookup failure.
    if (hull_shapes_painted == 0) {
        std::cout << "[AmdTrackedVehicleAgent] builder " << m_agent_key.GetNodeID() << ": no shape named '"
                  << tracked_hull_shape_name << "' among " << (bodies.size() - bodies_before)
                  << " zombie bodies -- hull will NOT be rank-coloured.\n";
    }
}

SynTrailerAgent::SynTrailerAgent(std::shared_ptr<chrono::vehicle::WheeledTrailer> trailer)
    : chrono::synchrono::SynWheeledVehicleAgent(nullptr), m_trailer(trailer) {}

void SynTrailerAgent::InitializeZombie(chrono::ChSystem* system) {
    const size_t bodies_before = system->GetBodies().size();

    chrono::synchrono::SynWheeledVehicleAgent::InitializeZombie(system);

    // HIDE the plate rather than colour it: the owning rank does not draw it either
    // (InitializeTrailer sets chassis visualization to NONE). trailer_chassis.obj is
    // an 8-vertex box with an empty .mtl and 5 normals for 6 faces, which is why it
    // showed up as a dark slab in the sensor view and nowhere else.
    const auto& bodies = system->GetBodies();
    int hidden = 0;
    for (size_t i = bodies_before; i < bodies.size(); ++i)
        hidden += SetVisualShapesVisible(bodies[i], false, trailer_chassis_shape_name);

    if (hidden == 0) {
        std::cout << "[SynTrailerAgent] trailer " << m_agent_key.GetNodeID() << ": no shape named '"
                  << trailer_chassis_shape_name << "' among " << (bodies.size() - bodies_before)
                  << " zombie bodies -- the chassis plate will still be drawn.\n";
    }
}

void SynTrailerAgent::Update() {
    if (!m_trailer)
        return;

    auto chassis_abs = m_trailer->GetChassis()->GetBody()->GetFrameRefToAbs();
    chrono::synchrono::SynPose chassis(chassis_abs.GetPos(), chassis_abs.GetRot());
    chassis.GetFrame().SetPosDt(chassis_abs.GetPosDt());
    chassis.GetFrame().SetPosDt2(chassis_abs.GetPosDt2());
    chassis.GetFrame().SetRotDt(chassis_abs.GetRotDt());
    chassis.GetFrame().SetRotDt2(chassis_abs.GetRotDt2());

    std::vector<chrono::synchrono::SynPose> wheels;
    for (auto& axle : m_trailer->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto state = wheel->GetState();
            auto wheel_abs = wheel->GetSpindle()->GetFrameRefToAbs();
            chrono::synchrono::SynPose frame(state.pos, state.rot);
            frame.GetFrame().SetPosDt(wheel_abs.GetPosDt());
            frame.GetFrame().SetPosDt2(wheel_abs.GetPosDt2());
            frame.GetFrame().SetRotDt(wheel_abs.GetRotDt());
            frame.GetFrame().SetRotDt2(wheel_abs.GetRotDt2());
            wheels.emplace_back(frame);
        }
    }

    const double time = m_trailer->GetChassis()->GetBody()->GetSystem()->GetChTime();
    m_state->SetState(time, chassis, wheels);
}

SynRockAgent::SynRockAgent(const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>* rocks,
                           std::string chrono_data_path,
                           bool visualize_zombies,
                           RockFieldConfig config)
    : chrono::synchrono::SynAgent(),
      m_rocks(rocks),
      m_chrono_data_path(std::move(chrono_data_path)),
      m_visualize_zombies(visualize_zombies),
      m_config(config),
      m_state(chrono_types::make_shared<chrono::synchrono::SynMAPMessage>()) {}

void SynRockAgent::InitializeZombie(chrono::ChSystem* system) {
    if (!m_visualize_zombies || m_agent_key.GetNodeID() <= 0)
        return;

    const std::array<std::string, 3> rock_visual_obj_files = {
        m_chrono_data_path + "robot/curiosity/rocks/rock1.obj",
        m_chrono_data_path + "robot/curiosity/rocks/rock2.obj",
        m_chrono_data_path + "robot/curiosity/rocks/rock3.obj",
    };

    auto rock_vis_mat = CreateLunarHapkeMaterial();
    std::array<std::shared_ptr<chrono::ChVisualShapeTriangleMesh>, 3> rock_vis_shapes;
    for (size_t i = 0; i < rock_visual_obj_files.size(); i++) {
        auto mesh = LoadRockMesh(rock_visual_obj_files[i], true, m_config.mesh_scale);
        rock_vis_shapes[i] = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        rock_vis_shapes[i]->SetMesh(mesh);
        rock_vis_shapes[i]->SetBackfaceCull(true);
        rock_vis_shapes[i]->AddMaterial(rock_vis_mat);
    }

    // Enough zombies for every harvest cycle, all created up front: bodies added to
    // rank 0 after startup are not picked up by the VSG scene graph or the OptiX
    // scene, since both bind their renderables at initialise time. Unused ones wait
    // below the terrain until a message places them.
    const int robot_index = m_agent_key.GetNodeID() - 1;
    const int capacity = std::max(1, m_config.rocks_per_rank) * zombie_rock_cycle_capacity;
    for (int i = 0; i < capacity; i++) {
        const int shape_index = (robot_index * m_config.rocks_per_rank + i) % static_cast<int>(rock_vis_shapes.size());
        auto rock = chrono_types::make_shared<chrono::ChBodyAuxRef>();
        rock->SetFixed(true);
        rock->EnableCollision(false);
        rock->AddVisualShape(rock_vis_shapes[shape_index]);
        rock->SetFrameRefToAbs(chrono::ChFrame<>(zombie_stash_position, chrono::QUNIT));
        system->AddBody(rock);
        m_zombie_rocks.push_back(rock);
    }
}

void SynRockAgent::SynchronizeZombie(std::shared_ptr<chrono::synchrono::SynMessage> message) {
    auto state = std::dynamic_pointer_cast<chrono::synchrono::SynMAPMessage>(message);
    if (!state || m_zombie_rocks.empty() || state->intersections.empty() || state->intersections[0].approaches.empty())
        return;

    const auto& lanes = state->intersections[0].approaches[0]->lanes;

    // Report each time this rank draws more rocks for that robot -- the only place
    // rock transmission is observable from a log.
    if (lanes.size() > m_zombies_placed) {
        m_zombies_placed = lanes.size();
        std::cout << "[SynRockAgent] robot " << m_agent_key.GetNodeID() << ": now drawing "
                  << std::min(m_zombies_placed, m_zombie_rocks.size()) << " of " << lanes.size()
                  << " rock(s) (zombie capacity " << m_zombie_rocks.size() << ")\n";
        if (lanes.size() > m_zombie_rocks.size()) {
            std::cout << "[SynRockAgent] robot " << m_agent_key.GetNodeID()
                      << ": OUT OF ZOMBIE CAPACITY -- rocks beyond " << m_zombie_rocks.size()
                      << " are invisible. Raise zombie_rock_cycle_capacity.\n";
        }
    }

    for (size_t i = 0; i < lanes.size() && i < m_zombie_rocks.size(); i++) {
        if (lanes[i].controlPoints.size() < 3)
            continue;

        const auto& p = lanes[i].controlPoints[0];
        const auto& q0q1q2 = lanes[i].controlPoints[1];
        const auto& q3 = lanes[i].controlPoints[2];
        m_zombie_rocks[i]->SetFrameRefToAbs(
            chrono::ChFrame<>(p, chrono::ChQuaternion<>(q0q1q2.x(), q0q1q2.y(), q0q1q2.z(), q3.x())));
    }
}

void SynRockAgent::Update() {
    if (!m_rocks || m_rocks->empty())
        return;

    m_state = chrono_types::make_shared<chrono::synchrono::SynMAPMessage>(m_agent_key,
                                                                          chrono::synchrono::AgentKey());
    m_state->time = (*m_rocks)[0]->GetSystem()->GetChTime();

    chrono::synchrono::Intersection rock_intersection;
    auto rock_approach = chrono_types::make_shared<chrono::synchrono::SynApproachMessage>(
        m_agent_key, chrono::synchrono::AgentKey());
    rock_approach->time = m_state->time;

    for (const auto& rock : *m_rocks) {
        const auto frame = rock->GetFrameRefToAbs();
        const auto p = frame.GetPos();
        const auto q = frame.GetRot();
        rock_approach->lanes.emplace_back(
            0.0, std::vector<chrono::ChVector3d>{
                     chrono::ChVector3d(p.x(), p.y(), p.z()),
                     chrono::ChVector3d(q.e0(), q.e1(), q.e2()),
                     chrono::ChVector3d(q.e3(), 0.0, 0.0),
                 });
    }

    rock_intersection.approaches.push_back(rock_approach);
    m_state->intersections.push_back(rock_intersection);
}

void SynRockAgent::GatherMessages(chrono::synchrono::SynMessageList& messages) {
    if (m_rocks && !m_rocks->empty())
        messages.push_back(m_state);
}

void SynRockAgent::GatherDescriptionMessages(chrono::synchrono::SynMessageList& messages) {}

void SynRockAgent::SetKey(chrono::synchrono::AgentKey agent_key) {
    m_agent_key = agent_key;
    m_state->SetSourceKey(agent_key);
}

SynTrailerBedAgent::SynTrailerBedAgent(std::shared_ptr<chrono::ChBody> bed,
                                       std::shared_ptr<chrono::ChBody> tailgate,
                                       bool visualize_zombies)
    : chrono::synchrono::SynAgent(),
      m_bed(std::move(bed)),
      m_tailgate(std::move(tailgate)),
      m_visualize_zombies(visualize_zombies),
      m_state(chrono_types::make_shared<chrono::synchrono::SynMAPMessage>()) {}

void SynTrailerBedAgent::InitializeZombie(chrono::ChSystem* system) {
    if (!m_visualize_zombies || m_agent_key.GetNodeID() <= 0)
        return;

    const chrono::ChColor color = RankColor(m_agent_key.GetNodeID() - 1);

    // Same boxes as the real tub (shared geometry in RobotLayout), so a rock resting
    // on the real floor is drawn resting on this one. Visual only.
    auto make_body = [&](const std::string& name) {
        auto body = chrono_types::make_shared<chrono::ChBody>();
        body->SetName(name);
        body->SetFixed(true);
        body->EnableCollision(false);
        // Out of sight until the first pose arrives.
        body->SetPos(zombie_stash_position);
        return body;
    };
    auto add_box = [&](const std::shared_ptr<chrono::ChBody>& body, const TrailerBedBox& box) {
        auto visual = chrono_types::make_shared<chrono::ChVisualShapeBox>(box.size.x(), box.size.y(), box.size.z());
        visual->SetColor(color);
        body->AddVisualShape(visual, chrono::ChFramed(box.center, chrono::QUNIT));
    };

    const std::string suffix = std::to_string(m_agent_key.GetNodeID());
    auto bed = make_body("trailer_bed_zombie_" + suffix);
    for (const auto& box : TrailerBedBoxes())
        add_box(bed, box);
    system->AddBody(bed);
    m_zombie_bodies.push_back(bed);

    auto gate = make_body("trailer_tailgate_zombie_" + suffix);
    add_box(gate, TrailerTailgateBox());
    system->AddBody(gate);
    m_zombie_bodies.push_back(gate);

}

void SynTrailerBedAgent::SynchronizeZombie(std::shared_ptr<chrono::synchrono::SynMessage> message) {
    auto state = std::dynamic_pointer_cast<chrono::synchrono::SynMAPMessage>(message);
    if (!state || m_zombie_bodies.empty() || state->intersections.empty() ||
        state->intersections[0].approaches.empty())
        return;

    const auto& lanes = state->intersections[0].approaches[0]->lanes;
    for (size_t i = 0; i < lanes.size() && i < m_zombie_bodies.size(); ++i) {
        if (lanes[i].controlPoints.size() < 3)
            continue;

        const auto& p = lanes[i].controlPoints[0];
        const auto& q0q1q2 = lanes[i].controlPoints[1];
        const auto& q3 = lanes[i].controlPoints[2];
        m_zombie_bodies[i]->SetPos(p);
        m_zombie_bodies[i]->SetRot(chrono::ChQuaternion<>(q0q1q2.x(), q0q1q2.y(), q0q1q2.z(), q3.x()));
    }
}

void SynTrailerBedAgent::Update() {
    if (!m_bed)
        return;

    m_state = chrono_types::make_shared<chrono::synchrono::SynMAPMessage>(m_agent_key,
                                                                          chrono::synchrono::AgentKey());
    m_state->time = m_bed->GetSystem()->GetChTime();

    chrono::synchrono::Intersection bed_intersection;
    auto bed_approach = chrono_types::make_shared<chrono::synchrono::SynApproachMessage>(
        m_agent_key, chrono::synchrono::AgentKey());
    bed_approach->time = m_state->time;

    // Order must match m_zombie_bodies: bed, then tailgate.
    const std::array<std::shared_ptr<chrono::ChBody>, 2> bodies = {m_bed, m_tailgate};
    for (const auto& body : bodies) {
        if (!body)
            continue;
        const auto& frame = body->GetFrameRefToAbs();
        const auto p = frame.GetPos();
        const auto q = frame.GetRot();
        bed_approach->lanes.emplace_back(
            0.0, std::vector<chrono::ChVector3d>{
                     chrono::ChVector3d(p.x(), p.y(), p.z()),
                     chrono::ChVector3d(q.e0(), q.e1(), q.e2()),
                     chrono::ChVector3d(q.e3(), 0.0, 0.0),
                 });
    }

    bed_intersection.approaches.push_back(bed_approach);
    m_state->intersections.push_back(bed_intersection);
}

void SynTrailerBedAgent::GatherMessages(chrono::synchrono::SynMessageList& messages) {
    if (m_bed)
        messages.push_back(m_state);
}

void SynTrailerBedAgent::GatherDescriptionMessages(chrono::synchrono::SynMessageList& messages) {}

void SynTrailerBedAgent::SetKey(chrono::synchrono::AgentKey agent_key) {
    m_agent_key = agent_key;
    m_state->SetSourceKey(agent_key);
}

SynArmAgent::SynArmAgent(
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> arm_bodies,
    std::string amd_uw_data_path,
    std::string shapes_relative_dir,
    double geometry_scale,
    bool visualize_zombies)
    : chrono::synchrono::SynAgent(),
      m_arm_bodies(std::move(arm_bodies)),
      m_amd_uw_data_path(std::move(amd_uw_data_path)),
      m_shapes_relative_dir(std::move(shapes_relative_dir)),
      m_geometry_scale(geometry_scale),
      m_visualize_zombies(visualize_zombies),
      m_state(chrono_types::make_shared<chrono::synchrono::SynMAPMessage>()) {}

void SynArmAgent::InitializeZombie(chrono::ChSystem* system) {
    if (!m_visualize_zombies || m_agent_key.GetNodeID() <= 0)
        return;

    // Ordering matches LrvArm::GetBodies(). First six take the geometry scale; the
    // two fingers stay 1x, as LrvArm::CreateBody does on the owning rank.
    const std::array<std::string, 8> mesh_files = {
        "body_1_1.obj",
        "body_2_1.obj",
        "body_3_1.obj",
        "body_4_1.obj",
        "body_5_1.obj",
        "body_6_1.obj",
        "body_7_1.obj",
        "body_7_1.obj",
    };
    const std::string shapes_dir = m_amd_uw_data_path + m_shapes_relative_dir;

    for (size_t i = 0; i < mesh_files.size(); ++i) {
        auto mesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(
            shapes_dir + mesh_files[i], true, true);
        if (!mesh)
            throw std::runtime_error("Cannot load rank-0 arm mesh: " +
                                     shapes_dir + mesh_files[i]);
        if (i < 6 && m_geometry_scale != 1.0) {
            mesh->Transform(
                chrono::VNULL,
                chrono::ChMatrix33<>(m_geometry_scale));
        }

        auto visual =
            chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        visual->SetMesh(mesh);
        visual->SetName("Arm_Zombie_" + std::to_string(m_agent_key.GetNodeID()) + "_" +
                        std::to_string(m_agent_key.GetAgentID()) + "_" + std::to_string(i));
        visual->SetMutable(false);
        // Arms are grey, not per-rank coloured -- the rank colour lives on the hull
        // and trailer bed. See ArmGrey.
        visual->SetColor(ArmGrey());

        auto body = chrono_types::make_shared<chrono::ChBodyAuxRef>();
        body->SetName("arm_zombie_" + std::to_string(m_agent_key.GetNodeID()) + "_" +
                      std::to_string(m_agent_key.GetAgentID()) + "_" + std::to_string(i));
        body->SetFixed(true);
        body->EnableCollision(false);
        body->AddVisualShape(visual);
        system->AddBody(body);
        m_zombie_arm_bodies.push_back(body);
    }

}

void SynArmAgent::SynchronizeZombie(
    std::shared_ptr<chrono::synchrono::SynMessage> message) {
    auto state =
        std::dynamic_pointer_cast<chrono::synchrono::SynMAPMessage>(message);
    if (!state || m_zombie_arm_bodies.empty() ||
        state->intersections.empty() ||
        state->intersections[0].approaches.empty())
        return;

    const auto& lanes = state->intersections[0].approaches[0]->lanes;
    for (size_t i = 0;
         i < lanes.size() && i < m_zombie_arm_bodies.size(); ++i) {
        if (lanes[i].controlPoints.size() < 3)
            continue;

        const auto& p = lanes[i].controlPoints[0];
        const auto& q0q1q2 = lanes[i].controlPoints[1];
        const auto& q3 = lanes[i].controlPoints[2];
        m_zombie_arm_bodies[i]->SetFrameRefToAbs(chrono::ChFrame<>(
            p, chrono::ChQuaternion<>(q0q1q2.x(), q0q1q2.y(),
                                      q0q1q2.z(), q3.x())));
    }
}

void SynArmAgent::Update() {
    if (m_arm_bodies.empty())
        return;

    m_state =
        chrono_types::make_shared<chrono::synchrono::SynMAPMessage>(
            m_agent_key, chrono::synchrono::AgentKey());
    m_state->time = m_arm_bodies[0]->GetSystem()->GetChTime();

    chrono::synchrono::Intersection arm_intersection;
    auto arm_approach =
        chrono_types::make_shared<chrono::synchrono::SynApproachMessage>(
            m_agent_key, chrono::synchrono::AgentKey());
    arm_approach->time = m_state->time;

    for (const auto& body : m_arm_bodies) {
        const auto frame = body->GetFrameRefToAbs();
        const auto p = frame.GetPos();
        const auto q = frame.GetRot();
        arm_approach->lanes.emplace_back(
            0.0,
            std::vector<chrono::ChVector3d>{
                chrono::ChVector3d(p.x(), p.y(), p.z()),
                chrono::ChVector3d(q.e0(), q.e1(), q.e2()),
                chrono::ChVector3d(q.e3(), 0.0, 0.0),
            });
    }

    arm_intersection.approaches.push_back(arm_approach);
    m_state->intersections.push_back(arm_intersection);
}

void SynArmAgent::GatherMessages(
    chrono::synchrono::SynMessageList& messages) {
    if (!m_arm_bodies.empty())
        messages.push_back(m_state);
}

void SynArmAgent::GatherDescriptionMessages(
    chrono::synchrono::SynMessageList& messages) {}

void SynArmAgent::SetKey(
    chrono::synchrono::AgentKey agent_key) {
    m_agent_key = agent_key;
    m_state->SetSourceKey(agent_key);
}

}  // namespace amd_uw
