#include "MaterialUtils.h"

#include "chrono/core/ChTypes.h"
#include "chrono/utils/ChConstants.h"

namespace amd_uw {

std::shared_ptr<chrono::ChVisualMaterial> CreateLunarHapkeMaterial() {
    auto material = chrono_types::make_shared<chrono::ChVisualMaterial>();
    material->SetAmbientColor({0.0f, 0.0f, 0.0f});
    material->SetDiffuseColor({0.7f, 0.7f, 0.7f});
    material->SetSpecularColor({1.0f, 1.0f, 1.0f});
    material->SetUseSpecularWorkflow(true);
    material->SetRoughness(0.8f);
    material->SetAnisotropy(1.0f);
    material->SetBSDF(BSDFType::HAPKE);
    material->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f,
                                 23.4f * static_cast<float>(chrono::CH_PI / 180.0));
    material->SetClassID(30000);
    material->SetInstanceID(20000);
    return material;
}

void ApplyMaterialToVisualShapes(std::shared_ptr<chrono::ChBody> body,
                                 std::shared_ptr<chrono::ChVisualMaterial> material) {
    if (!body || !body->GetVisualModel())
        return;

    for (const auto& shape_instance : body->GetVisualModel()->GetShapeInstances()) {
        auto shape = shape_instance.shape;
        if (!shape)
            continue;

        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(material);
        } else {
            shape->GetMaterials()[0] = material;
        }
    }
}

}  // namespace amd_uw
