#pragma once

#include <memory>
#include <string>
#include <vector>

#include "RockField.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono_synchrono/agent/SynAgent.h"
#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/flatbuffer/message/SynMAPMessage.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledTrailer.h"

namespace amd_uw {

class SynTrailerAgent : public chrono::synchrono::SynWheeledVehicleAgent {
  public:
    explicit SynTrailerAgent(std::shared_ptr<chrono::vehicle::WheeledTrailer> trailer = nullptr);

    void Update() override;

    // Hides the zombie chassis plate, by the same trick as AmdTrackedVehicleAgent.
    void InitializeZombie(chrono::ChSystem* system) override;

  private:
    std::shared_ptr<chrono::vehicle::WheeledTrailer> m_trailer;
};

// A zombie that absorbs messages and builds nothing. Registered by every rank except
// the sensor rank, whose camera is the only one that shows other ranks' machines.
//
// Registering something is required: DistributeMessages looks up each message's
// source key and logs "The intended agent is not on this node!" on a miss -- once per
// remote agent per heartbeat.
class SynQuietAgent : public chrono::synchrono::SynAgent {
  public:
    void InitializeZombie(chrono::ChSystem* system) override {}
    void SynchronizeZombie(std::shared_ptr<chrono::synchrono::SynMessage> message) override {}
    void Update() override {}
    void GatherMessages(chrono::synchrono::SynMessageList& messages) override {}
    void GatherDescriptionMessages(chrono::synchrono::SynMessageList& messages) override {}
};

// Stock tracked-vehicle agent with the zombie hull painted in the owning rank's
// colour, so a builder is identifiable in the sensor view (which renders zombies, not
// the real bodies BuilderRig colours).
//
// The base class keeps its zombie bodies private, but it must add them to OUR system:
// so snapshot the body count, let the base build, and the bodies that appeared are
// its. The hull is picked out by shape name rather than list position, so a Chrono
// reordering cannot silently paint the wrong part. Running gear is left alone.
class AmdTrackedVehicleAgent : public chrono::synchrono::SynTrackedVehicleAgent {
  public:
    AmdTrackedVehicleAgent(chrono::vehicle::ChTrackedVehicle* vehicle, const std::string& filename);

    void InitializeZombie(chrono::ChSystem* system) override;
};

// Broadcasts this rank's rocks so the sensor rank can draw them, including rocks
// spawned by later harvest cycles.
class SynRockAgent : public chrono::synchrono::SynAgent {
  public:
    // `rocks` is the rig's LIVE rock vector, not a copy -- StartNextHarvestCycle
    // appends to it on every dump. Must outlive this agent; only dereferenced from
    // Update()/GatherMessages().
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

// Broadcasts the collector's dump bed and hinged tailgate in the rank's colour. No
// stock agent carries them -- SynTrailerAgent sends only the trailer chassis and
// wheels -- so without this the sensor view has no tub, and a loaded, dumping and
// empty bed all look the same. Two bodies so the gate swings independently.
//
// Pose encoding matches SynArmAgent: one message "lane" per body, position and
// quaternion across three control points.
class SynTrailerBedAgent : public chrono::synchrono::SynAgent {
  public:
    SynTrailerBedAgent(std::shared_ptr<chrono::ChBody> bed,
                       std::shared_ptr<chrono::ChBody> tailgate,
                       bool visualize_zombies);

    void InitializeZombie(chrono::ChSystem* system) override;
    void SynchronizeZombie(std::shared_ptr<chrono::synchrono::SynMessage> message) override;
    void Update() override;
    void GatherMessages(chrono::synchrono::SynMessageList& messages) override;
    void GatherDescriptionMessages(chrono::synchrono::SynMessageList& messages) override;
    void SetKey(chrono::synchrono::AgentKey agent_key) override;

  private:
    // Owned by the rig on the collector's rank; null on a zombie.
    std::shared_ptr<chrono::ChBody> m_bed;
    std::shared_ptr<chrono::ChBody> m_tailgate;
    // Visual-only copies, message order: [0] bed, [1] gate. Plain ChBody so the visual
    // frame matches the real bodies' and the pose applies verbatim.
    std::vector<std::shared_ptr<chrono::ChBody>> m_zombie_bodies;
    bool m_visualize_zombies;
    std::shared_ptr<chrono::synchrono::SynMAPMessage> m_state;
};

// Synchronizes the eight articulated bodies of ONE arm so the sensor rank sees it
// move. Used for both manipulators: same eight body_N_1.obj meshes in LrvArm::GetBodies()
// order, differing only in directory and geometry scale (builder 2x, rover 1x) -- the
// constructor's last two arguments.
//
// Nothing else transmits a manipulator, so an arm with no agent is absent from the
// sensor view entirely.
class SynArmAgent : public chrono::synchrono::SynAgent {
  public:
    SynArmAgent(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> arm_bodies,
                std::string amd_uw_data_path,
                std::string shapes_relative_dir,
                double geometry_scale,
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
    std::string m_shapes_relative_dir;
    double m_geometry_scale;
    bool m_visualize_zombies;
    std::shared_ptr<chrono::synchrono::SynMAPMessage> m_state;
};

}  // namespace amd_uw
