#include "SynAgents.h"

#include <array>
#include <utility>

#include "MaterialUtils.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/core/ChTypes.h"
#include "chrono_synchrono/flatbuffer/message/SynApproachMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMessageUtils.h"
#include "chrono_synchrono/flatbuffer/message/SynWheeledVehicleMessage.h"

namespace amd_uw {

SynTrailerAgent::SynTrailerAgent(std::shared_ptr<chrono::vehicle::WheeledTrailer> trailer)
    : chrono::synchrono::SynWheeledVehicleAgent(nullptr), m_trailer(trailer) {}

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

SynRockAgent::SynRockAgent(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> rocks,
                           std::string chrono_data_path,
                           bool visualize_zombies,
                           RockFieldConfig config)
    : chrono::synchrono::SynAgent(),
      m_rocks(std::move(rocks)),
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

    const int robot_index = m_agent_key.GetNodeID() - 1;
    for (int i = 0; i < m_config.rocks_per_rank; i++) {
        const int shape_index = (robot_index * m_config.rocks_per_rank + i) % static_cast<int>(rock_vis_shapes.size());
        auto rock = chrono_types::make_shared<chrono::ChBodyAuxRef>();
        rock->SetFixed(true);
        rock->EnableCollision(false);
        rock->AddVisualShape(rock_vis_shapes[shape_index]);
        system->AddBody(rock);
        m_zombie_rocks.push_back(rock);
    }
}

void SynRockAgent::SynchronizeZombie(std::shared_ptr<chrono::synchrono::SynMessage> message) {
    auto state = std::dynamic_pointer_cast<chrono::synchrono::SynMAPMessage>(message);
    if (!state || m_zombie_rocks.empty() || state->intersections.empty() || state->intersections[0].approaches.empty())
        return;

    const auto& lanes = state->intersections[0].approaches[0]->lanes;
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
    if (m_rocks.empty())
        return;

    m_state = chrono_types::make_shared<chrono::synchrono::SynMAPMessage>(m_agent_key,
                                                                          chrono::synchrono::AgentKey());
    m_state->time = m_rocks[0]->GetSystem()->GetChTime();

    chrono::synchrono::Intersection rock_intersection;
    auto rock_approach = chrono_types::make_shared<chrono::synchrono::SynApproachMessage>(
        m_agent_key, chrono::synchrono::AgentKey());
    rock_approach->time = m_state->time;

    for (const auto& rock : m_rocks) {
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
    if (!m_rocks.empty())
        messages.push_back(m_state);
}

void SynRockAgent::GatherDescriptionMessages(chrono::synchrono::SynMessageList& messages) {}

void SynRockAgent::SetKey(chrono::synchrono::AgentKey agent_key) {
    m_agent_key = agent_key;
    m_state->SetSourceKey(agent_key);
}

}  // namespace amd_uw
