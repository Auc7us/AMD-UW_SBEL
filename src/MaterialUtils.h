#pragma once

#include <memory>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChContactMaterial.h"

namespace amd_uw {

// Build a contact material of the type matching `method`. NSC ignores Young's
// modulus and restitution; SMC (penalty) contact uses them. Centralizing this
// keeps every contact surface in the scene consistent -- a stray NSC material
// in an SMC system silently breaks that contact pair. `young` defaults to 2e7
// so SMC surfaces are stiff enough that position-driven parts (e.g. gripper
// pads) bear on rocks instead of sinking into them.
std::shared_ptr<chrono::ChContactMaterial> MakeContactMaterial(chrono::ChContactMethod method,
                                                               float friction,
                                                               float restitution = 0.0f,
                                                               float young = 2e7f);

std::shared_ptr<chrono::ChVisualMaterial> CreateLunarHapkeMaterial();

void ApplyMaterialToVisualShapes(std::shared_ptr<chrono::ChBody> body,
                                 std::shared_ptr<chrono::ChVisualMaterial> material);

}  // namespace amd_uw
