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

class SynRockAgent : public chrono::synchrono::SynAgent {
  public:
    SynRockAgent(std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> rocks,
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
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_rocks;
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> m_zombie_rocks;
    std::string m_chrono_data_path;
    bool m_visualize_zombies;
    RockFieldConfig m_config;
    std::shared_ptr<chrono::synchrono::SynMAPMessage> m_state;
};

}  // namespace amd_uw
