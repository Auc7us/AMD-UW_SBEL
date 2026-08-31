#include "ScmRecorder.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <type_traits>

#include "chrono_vehicle/terrain/SCMTerrain.h"

namespace amd_uw {

namespace {

constexpr char kFileMagic[8] = {'A', 'M', 'D', 'U', 'W', 'S', 'C', 'M'};
constexpr std::uint32_t kFormatVersion = 1;
constexpr std::uint32_t kFrameMagic = 0x4D435353u;  // 'SSCM' on the wire

// A node counts as moved only past this, so float noise in the last emitted value cannot
// republish a node that is physically unchanged. 0.1 mm against a 0.1 m grid.
constexpr double kLevelEpsilon = 1e-4;

template <typename T>
void Append(std::vector<char>& buffer, const T& value) {
    static_assert(std::is_trivially_copyable<T>::value, "raw append needs a POD");
    const char* p = reinterpret_cast<const char*>(&value);
    buffer.insert(buffer.end(), p, p + sizeof(T));
}

inline std::int64_t PackKey(int i, int j) {
    return (static_cast<std::int64_t>(i) << 32) | static_cast<std::uint32_t>(j);
}

}  // namespace

ScmRecorder::ScmRecorder(const std::string& out_dir,
                         int rank,
                         double rate_hz,
                         double keyframe_period,
                         const chrono::vehicle::SCMTerrain* terrain,
                         double delta,
                         int nx,
                         int ny)
    : m_terrain(terrain),
      m_rank(rank),
      m_period(rate_hz > 0.0 ? 1.0 / rate_hz : 0.0),
      m_keyframe_period(keyframe_period) {
    if (!m_terrain || m_period <= 0.0) {
        std::cout << "[scm] rank " << rank << ": rate must be > 0 and terrain non-null; capture disabled.\n";
        return;
    }

    std::string dir = out_dir;
    if (!dir.empty() && dir.back() != '/')
        dir += '/';
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        std::cout << "[scm] rank " << rank << ": cannot create " << dir << " (" << ec.message()
                  << "); capture disabled.\n";
        return;
    }

    m_path = dir + "rank_" + std::to_string(rank) + "_scm.bin";
    m_file = std::fopen(m_path.c_str(), "wb");
    if (!m_file) {
        std::cout << "[scm] rank " << rank << ": cannot open " << m_path << "; capture disabled.\n";
        return;
    }

    // The SCM reference frame is the one thing a consumer cannot infer: node (i,j) is at
    // local (i*delta, j*delta) and its world position is plane_frame * that. Recorded
    // rather than assumed, even though this demo leaves it at the identity.
    const chrono::ChCoordsys<>& plane = m_terrain->GetReferenceFrame();

    m_buffer.clear();
    m_buffer.insert(m_buffer.end(), kFileMagic, kFileMagic + sizeof(kFileMagic));
    Append(m_buffer, kFormatVersion);
    Append(m_buffer, static_cast<std::uint32_t>(rank));
    Append(m_buffer, static_cast<double>(rate_hz));
    Append(m_buffer, static_cast<double>(delta));
    Append(m_buffer, static_cast<double>(plane.pos.x()));
    Append(m_buffer, static_cast<double>(plane.pos.y()));
    Append(m_buffer, static_cast<double>(plane.pos.z()));
    Append(m_buffer, static_cast<double>(plane.rot.e0()));  // w
    Append(m_buffer, static_cast<double>(plane.rot.e1()));  // x
    Append(m_buffer, static_cast<double>(plane.rot.e2()));  // y
    Append(m_buffer, static_cast<double>(plane.rot.e3()));  // z
    Append(m_buffer, static_cast<std::int32_t>(nx));
    Append(m_buffer, static_cast<std::int32_t>(ny));
    std::fwrite(m_buffer.data(), 1, m_buffer.size(), m_file);
    m_bytes += static_cast<long long>(m_buffer.size());
    std::fflush(m_file);

    std::cout << "[scm] rank " << rank << ": deformation to " << m_path << " at " << rate_hz << " Hz"
              << " (keyframe every " << keyframe_period << " s), delta=" << delta << ", grid +/-" << nx << "x"
              << ny << ", plane pos=(" << plane.pos.x() << "," << plane.pos.y() << "," << plane.pos.z()
              << ") quat=(" << plane.rot.e0() << "," << plane.rot.e1() << "," << plane.rot.e2() << ","
              << plane.rot.e3() << ").\n";
}

ScmRecorder::~ScmRecorder() {
    if (m_file) {
        std::fflush(m_file);
        std::fclose(m_file);
        std::cout << "[scm] rank " << m_rank << ": " << m_frame_count << " frames, " << m_last_level.size()
                  << " distinct deformed nodes, " << (m_bytes / 1024) << " KiB -> " << m_path << "\n";
    }
}

void ScmRecorder::CaptureIfDue(double time) {
    if (!m_file || time < m_next_sample)
        return;
    // Advance past any instants a long step may have skipped, so the schedule cannot drift.
    while (m_next_sample <= time)
        m_next_sample += m_period;

    // t=0 is always a keyframe. It is empty in practice -- nothing has been touched before
    // the first step -- but it anchors the file's time base.
    const bool keyframe = (m_frame_count == 0) || (m_keyframe_period > 0.0 && time >= m_next_keyframe);
    if (keyframe && m_keyframe_period > 0.0) {
        while (m_next_keyframe <= time)
            m_next_keyframe += m_keyframe_period;
    }
    WriteFrame(time, keyframe);
}

void ScmRecorder::WriteFrame(double time, bool keyframe) {
    // all_nodes=true: every node touched since the run began. See the header for why the
    // cheaper all_nodes=false would drop 199 of every 200 steps at this step size.
    const std::vector<chrono::vehicle::SCMTerrain::NodeLevel> nodes = m_terrain->GetModifiedNodes(true);

    m_buffer.clear();
    m_buffer.reserve(16 + nodes.size() * 12);
    Append(m_buffer, kFrameMagic);
    Append(m_buffer, time);
    Append(m_buffer, static_cast<std::uint32_t>(0));  // count, back-patched below
    const std::size_t count_at = m_buffer.size() - sizeof(std::uint32_t);

    std::uint32_t count = 0;
    for (const auto& n : nodes) {
        const int i = n.first.x();
        const int j = n.first.y();
        const double z = n.second;
        const std::int64_t key = PackKey(i, j);
        if (!keyframe) {
            auto it = m_last_level.find(key);
            if (it != m_last_level.end() && std::abs(it->second - z) < kLevelEpsilon)
                continue;  // unchanged since the last sample
        }
        m_last_level[key] = z;
        Append(m_buffer, static_cast<std::int32_t>(i));
        Append(m_buffer, static_cast<std::int32_t>(j));
        Append(m_buffer, static_cast<float>(z));
        ++count;
    }
    std::memcpy(m_buffer.data() + count_at, &count, sizeof(count));

    std::fwrite(m_buffer.data(), 1, m_buffer.size(), m_file);
    m_bytes += static_cast<long long>(m_buffer.size());
    // Flushed per frame: at this rate it costs nothing, and a run killed mid-flight (which
    // is how most of them end) then still has every frame it reported.
    std::fflush(m_file);
    ++m_frame_count;
}

}  // namespace amd_uw
