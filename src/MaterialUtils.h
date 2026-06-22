#pragma once

#include <memory>

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/physics/ChBody.h"

namespace amd_uw {

std::shared_ptr<chrono::ChVisualMaterial> CreateLunarHapkeMaterial();

void ApplyMaterialToVisualShapes(std::shared_ptr<chrono::ChBody> body,
                                 std::shared_ptr<chrono::ChVisualMaterial> material);

}  // namespace amd_uw
