#pragma once

#include <memory>
#include <string>
#include <vector>

#include "RockField.h"

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono_synchrono/agent/SynAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

namespace amd_uw {

class SynTrailerAgent : public chrono::synchrono::SynWheeledVehicleAgent {
  public:
    explicit SynTrailerAgent(std::shared_ptr<chrono::vehicle::WheeledTrailer> trailer = nullptr);

    void Update() override;

  private:
    std::shared_ptr<chrono::vehicle::WheeledTrailer> m_trailer;
};

// Broadcasts this rank's rocks so the sensor/visualization rank can draw them.
//
// Both halves of this used to be frozen at the initial rock count, which made
// every rock spawned by a later harvest cycle invisible everywhere -- present and
// fully simulated in its owning rank, drawn nowhere. The rovers kept picking up
// rocks nobody could see, which made every other fault impossible to judge by eye.
class SynRockAgent : public chrono::synchrono::SynAgent {
  public:
    // `rocks` is the owning rig's LIVE rock vector, not a copy. It has to be:
    // RobotRig::StartNextHarvestCycle appends to that vector on every dump, and a
    // by-value snapshot taken here at startup never grew, so Update() only ever
    // transmitted the two cycle-0 rocks for the entire run. The pointee must
    // outlive this agent, which holds while the sim loop is running -- it is only
    // dereferenced from Update()/GatherMessages().
    SynRockAgent(const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>* rocks,
                 std::string chrono_data_path,
                 bool visualize_zombies,
                 RockFieldConfig config);

    void InitializeZombie(chrono::ChSystem* system) override;
    void SynchronizeZombie(std::shared_ptr<chrono::synchrono::SynMessage> message) override;
    void Update() override;
    void GatherMessages(chrono::synchrono::SynMessageList& messages) override;
    void GatherDescriptionMessages(chrono::synchrono::SynMessageList& messages) override;
    void SetKey(chrono::synchrono::AgentKey agent_key) override;

  private:
    const std::vector<std::shared_ptr<chrono::ChBodyAuxRef>>* m_rocks;
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_zombie_rocks;
    std::string m_chrono_data_path;
    bool m_visualize_zombies;
    RockFieldConfig m_config;
    // How many rocks this zombie has been told about so far, so growth can be
    // reported once per change instead of every step.
    size_t m_zombies_placed = 0;
    std::shared_ptr<chrono::synchrono::SynMAPMessage> m_state;
};

// Synchronizes the eight independently articulated builder-arm bodies. Remote
// ranks keep only message-routing zombies; rank 0 creates visual-only bodies so
// the central Chrono Sensor camera sees the arm move with its builder.
class SynBuilderArmAgent : public chrono::synchrono::SynAgent {
  public:
    SynBuilderArmAgent(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> arm_bodies,
                       std::string amd_uw_data_path,
                       bool visualize_zombies);

    void InitializeZombie(chrono::ChSystem* system) override;
    void SynchronizeZombie(std::shared_ptr<chrono::synchrono::SynMessage> message) override;
    void Update() override;
    void GatherMessages(chrono::synchrono::SynMessageList& messages) override;
    void GatherDescriptionMessages(chrono::synchrono::SynMessageList& messages) override;
    void SetKey(chrono::synchrono::AgentKey agent_key) override;

  private:
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_arm_bodies;
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_zombie_arm_bodies;
    std::string m_amd_uw_data_path;
    bool m_visualize_zombies;
    std::shared_ptr<chrono::synchrono::SynMAPMessage> m_state;
};

}  // namespace amd_uw
