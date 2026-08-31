#pragma once

#include <cstdio>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace chrono {
namespace vehicle {
class SCMTerrain;
}
}  // namespace chrono

namespace amd_uw {

// Deformed-ground capture for offline rendering, alongside TrajectoryRecorder's poses.
//
// Poses alone cannot be rendered honestly on deformable terrain: the wheels ride in ruts
// they cut themselves, so a renderer drawing the original heightmap floats every machine
// over its own tracks. This writes what the soil actually did.
//
// WHY IT SAMPLES GetModifiedNodes(all_nodes=TRUE) AND DIFFS, rather than taking the
// cheaper-looking all_nodes=false:
//
//   SCMLoader::m_modified_nodes -- the list all_nodes=false reads -- is CLEARED at the
//   top of every ComputeInternalForces, so it describes ONE STEP. At a 5e-4 s step that
//   is 2000 lists per simulated second, and sampling it at 10 Hz would capture 1 step in
//   200 and silently discard the other 199: ruts would come out stippled rather than
//   continuous. all_nodes=true instead walks m_grid_map -- every node touched since the
//   run began -- so diffing it against the last height we emitted yields exactly "what
//   changed since the previous sample", complete, at any sample rate.
//
// Cost is one hash-map walk per sample, not per step. m_grid_map is the accumulated
// footprint (a 33 m builder lane at 0.1 m spacing is ~17k nodes), so at 10 Hz this is
// negligible next to the ~19 wall/sim the physics costs.
//
// KEYFRAMES NEED NO FLAG. Heights are absolute, and a consumer accumulates by overwrite,
// so a frame carrying every deformed node is idempotent with the deltas around it -- just
// larger. Emitting one periodically lets a consumer seek, or recover from a dropped
// frame, without replaying from t=0. Nothing in the format has to mark it.
//
// File layout (little-endian), one per recording rank:
//
//   rank_<r>_scm.bin
//     header  8s   "AMDUWSCM"
//             u32  version (1), u32 rank
//             f64  rate_hz
//             f64  delta                    grid spacing
//             f64  plane[7]                 SCM reference frame: pos xyz + quat wxyz
//             i32  nx, ny                   grid half-counts; indices span [-nx,+nx]
//     frame   u32  0x4D435353
//             f64  time
//             u32  count
//             count * (i32 i, i32 j, f32 z) ABSOLUTE height, not a delta
//
// Node (i,j) sits at local (i*delta, j*delta, z) and world = plane_frame * that.
class ScmRecorder {
  public:
    // `terrain` must outlive this object. `delta`, `nx`, `ny` are derived the same way
    // SCMLoader::Initialize derives them (they are private there, so they cannot be read
    // back off the terrain).
    ScmRecorder(const std::string& out_dir,
                int rank,
                double rate_hz,
                double keyframe_period,
                const chrono::vehicle::SCMTerrain* terrain,
                double delta,
                int nx,
                int ny);
    ~ScmRecorder();

    bool IsOpen() const { return m_file != nullptr; }

    // Cheap to call every step; writes only when the next sample instant is reached.
    void CaptureIfDue(double time);

    long GetFrameCount() const { return m_frame_count; }
    // Distinct nodes this rank has ever reported deformed.
    std::size_t GetNodeCount() const { return m_last_level.size(); }
    long long GetBytesWritten() const { return m_bytes; }

  private:
    void WriteFrame(double time, bool keyframe);

    const chrono::vehicle::SCMTerrain* m_terrain;
    std::string m_path;
    int m_rank;
    double m_period;
    double m_keyframe_period;
    double m_next_sample = 0.0;
    double m_next_keyframe = 0.0;

    std::FILE* m_file = nullptr;
    std::vector<char> m_buffer;

    // Last height emitted per node, so a sample can report only what moved. Key packs
    // (i,j) into one 64-bit word; the grid is bounded well inside 2^31 either way.
    std::unordered_map<std::int64_t, double> m_last_level;

    long m_frame_count = 0;
    long long m_bytes = 0;
};

}  // namespace amd_uw
