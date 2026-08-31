#pragma once

namespace chrono {
class ChSystem;
namespace vehicle {
class ChTrackedVehicle;
}
}  // namespace chrono

namespace amd_uw {

// Re-express each M113 track shoe's SOIL-FACING collision boxes as convex triangle
// meshes, so SCM's HIP ray-cast backend can see them.
//
// WHY THIS IS NEEDED AT ALL. The GPU backend consumes only
// ChCollisionShape::TRIANGLEMESH -- AppendLocalMesh in SCMTerrainRaycastGpu.cpp
// `continue`s past every other type -- and when it succeeds it REPLACES the CPU ray-cast
// loop for that whole step. So a scene where the tyres and rocks are meshes but the ~130
// track shoes are boxes does not degrade gracefully: the builder receives no soil hits at
// all and sinks. Meshing the shoes is what makes --scm_raycast_gpu safe to turn on.
//
// WHY ONLY THE SOIL-FACING BOXES. M113_TrackShoeSinglePin builds five boxes per shoe:
// bottom, top, guide horn, and two side pads. Only the bottom and the two sides reach
// below the shoe mid-plane; the top and the guide horn face INWARD and are what the road
// wheels, the sprocket and the neighbouring shoes bear against. Converting those would
// move the track's own internal contacts onto mesh geometry, and this single-pin track is
// documented in the README as fragile enough without that. They are left as boxes.
//
// WHY ONE CONVEX MESH PER BOX, not one mesh for the union. The union of three disjoint
// boxes is not convex, so a single mesh would need is_convex=false, handing Bullet a
// MOVING concave BVH mesh -- costlier and less stable than what it replaces. One convex
// mesh per box keeps Bullet on btConvexTriangleMeshShape, which behaves like the box did,
// and the GPU backend is happy either way: it walks every TRIANGLEMESH shape instance on
// the body and appends all of them.
//
// Geometry and contact material are read back off the existing shapes rather than
// hardcoded, so they cannot drift from whatever M113_TrackShoeSinglePin declares.
//
// Returns the number of box shapes converted (expect 3 per shoe).
int MeshifyTrackShoeSoilContact(chrono::vehicle::ChTrackedVehicle* vehicle, chrono::ChSystem* system);

}  // namespace amd_uw
