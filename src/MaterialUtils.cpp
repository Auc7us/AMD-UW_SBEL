#include "MaterialUtils.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/core/ChTypes.h"
#include "chrono/utils/ChConstants.h"

namespace amd_uw {

std::shared_ptr<chrono::ChContactMaterial> MakeContactMaterial(chrono::ChContactMethod method,
                                                               float friction,
                                                               float restitution,
                                                               float young) {
    chrono::ChContactMaterialData data;
    data.mu = friction;
    data.cr = restitution;
    data.Y = young;
    return data.CreateMaterial(method);
}

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

namespace {

// Shared matching rule for the two helpers below.
template <class Fn>
int ForEachMatchingShape(const std::shared_ptr<chrono::ChBody>& body,
                         const std::string& shape_name_filter,
                         Fn&& fn) {
    if (!body || !body->GetVisualModel())
        return 0;

    int matched = 0;
    for (const auto& shape_instance : body->GetVisualModel()->GetShapeInstances()) {
        const auto& shape = shape_instance.shape;
        if (!shape)
            continue;

        if (!shape_name_filter.empty()) {
            // Only ChVisualShapeTriangleMesh carries a name; unnamed shapes cannot
            // match and are left alone.
            auto mesh_shape = std::dynamic_pointer_cast<chrono::ChVisualShapeTriangleMesh>(shape);
            if (!mesh_shape || mesh_shape->GetName().find(shape_name_filter) == std::string::npos)
                continue;
        }

        fn(shape);
        ++matched;
    }

    return matched;
}

}  // namespace

int ApplyColorToVisualShapes(const std::shared_ptr<chrono::ChBody>& body,
                             const chrono::ChColor& color,
                             const std::string& shape_name_filter) {
    // SetColor clones the shared default material when needed, so this is safe on
    // shapes with MTL materials and on shapes with none.
    return ForEachMatchingShape(body, shape_name_filter,
                                [&](const std::shared_ptr<chrono::ChVisualShape>& shape) { shape->SetColor(color); });
}

int SetVisualShapesVisible(const std::shared_ptr<chrono::ChBody>& body,
                           bool visible,
                           const std::string& shape_name_filter) {
    return ForEachMatchingShape(
        body, shape_name_filter,
        [&](const std::shared_ptr<chrono::ChVisualShape>& shape) { shape->SetVisible(visible); });
}

}  // namespace amd_uw
