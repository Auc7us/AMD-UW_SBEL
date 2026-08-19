#pragma once

#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace vehicle {
class ChVehicle;
class ChTrackedVehicle;
class ChWheeledTrailer;
}  // namespace vehicle
}  // namespace chrono

namespace amd_uw {

class LrvArm;

// Absolute pose capture for offline re-rendering (Blender).
//
// Writes, per physics rank, the world pose of every MOVING body it owns -- rover
// chassis and wheels, trailer, dump bed and tailgate, both manipulators link by link,
// the builder's hull, road wheels, sprockets, idlers and every track shoe, and every
// rock -- at a fixed sample rate, plus a manifest that says what each recorded index
// IS and which mesh draws it.
//
// Three deliberate choices, because each of them is the difference between a usable
// dataset and a plausible-looking one:
//
// 1. The pose logged is the body's REFERENCE frame (ChBody::GetFrameRefToAbs), not its
//    centre of mass. That is exactly what Chrono hands its renderers
//    (ChBody::GetVisualModelFrame returns the same thing), so a mesh placed at the
//    logged pose lands where Chrono drew it. Logging GetPos() instead would silently
//    offset every ChBodyAuxRef -- which is every rock and every arm link -- by its
//    centre-of-mass offset.
//
// 2. The manifest records each body's visual shapes WITH their local frames, because a
//    Chrono body is not one mesh at the body origin: the M113 road wheel carries two
//    wheel halves at +/-y, the trailer tub is four boxes, and the rocks carry a mesh that
//    was pre-scaled and re-based on load. World transform of a shape is therefore
//    body_pose * shape_local_frame, and a consumer that assumes shape_local_frame is
//    identity draws a rover with its wheels stacked inside the hull.
//
// 3. The body set is re-scanned on every capture rather than fixed at t=0. Rocks are
//    spawned mid-run -- one fresh line per harvest cycle -- so a fixed layout would
//    silently stop at the cycle-0 rocks. New bodies get appended to the manifest at the
//    moment they first appear, with the sim time they appeared at.
//
// Everything that was already in the system when ExcludeExisting() is called -- terrain,
// the orbit rings, the centre pad, the decorative wall rocks -- is a static prop. It is
// described once, with its t=0 pose, and never sampled again.
class TrajectoryRecorder {
  public:
    // `out_dir` must already exist or be creatable. `write_static` should be true on
    // exactly one rank: the scenery is identical on all of them.
    TrajectoryRecorder(const std::string& out_dir,
                       int rank,
                       double rate_hz,
                       double step_size,
                       bool write_static);
    ~TrajectoryRecorder();

    bool IsOpen() const { return m_frames != nullptr; }

    // Free-form key/value pairs written into rank_<r>_meta.json. Call before Start().
    void AddMeta(const std::string& key, const std::string& json_value);

    // Everything in the system right now is scenery. Call after the terrain, rings, pad
    // and decorative rocks exist and BEFORE the rover, builder and rocks are built.
    void ExcludeExisting(chrono::ChSystem* system);

    // Naming. A labelled body gets <group>/<part> in the manifest; anything unlabelled
    // falls back to its Chrono name, which is why the rock bodies are now named.
    void Label(const std::shared_ptr<chrono::ChBody>& body, const std::string& group, const std::string& part);
    void LabelVehicle(chrono::vehicle::ChVehicle* vehicle, const std::string& group);
    // The tracked builder needs its own pass: ChTrackedVehicle::GetBodyList reaches the
    // hull, the sprockets, the suspension arms and every shoe, but NOT the track wheels
    // -- the road wheels, rollers and idler wheels came out unlabelled, which is to say
    // filed under "world" alongside the rocks. They are the parts that visibly spin.
    void LabelTrackedVehicle(chrono::vehicle::ChTrackedVehicle* vehicle, const std::string& group);
    void LabelTrailer(chrono::vehicle::ChWheeledTrailer* trailer, const std::string& group);
    void LabelArm(LrvArm* arm, const std::string& group);

    // Writes the meta file and the static-prop manifest. Call once, after labelling and
    // after every rank-owned body exists.
    void Start(chrono::ChSystem* system);

    // Samples if `time` has reached the next sample instant. Cheap to call every step.
    void CaptureIfDue(chrono::ChSystem* system, double time);

    long GetFrameCount() const { return m_frame_count; }
    size_t GetObjectCount() const { return m_index.size(); }

  private:
    int IndexOf(const chrono::ChBody* body, double time);
    // Serializes one body's identity and visual shapes as a single JSON object. Not
    // const: it consumes a group/part name so the next body carrying the same one gets
    // a suffix.
    std::string DescribeBody(const chrono::ChBody* body, int index, double time);

    std::string m_dir;
    int m_rank;
    double m_period;
    double m_next_sample = 0.0;
    bool m_write_static;

    std::FILE* m_frames = nullptr;
    std::FILE* m_objects = nullptr;
    std::vector<char> m_buffer;
    std::vector<std::pair<std::string, std::string>> m_meta;

    // Scenery: in the system but never sampled.
    std::unordered_map<const chrono::ChBody*, bool> m_excluded;
    // Sampled bodies, in first-seen order. The index IS the manifest key.
    std::unordered_map<const chrono::ChBody*, int> m_index;
    std::unordered_map<const chrono::ChBody*, std::pair<std::string, std::string>> m_labels;
    // group/part strings already handed out. Chrono reuses a subsystem name across
    // axles -- the rover has two bodies called "Polaris DoubleWishbone_LCA_L" -- and a
    // consumer keying on the name would silently animate one of them twice.
    std::unordered_map<std::string, int> m_name_uses;

    long m_frame_count = 0;
};

}  // namespace amd_uw
