#include "TrajectoryRecorder.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>

#include "LrvArm.h"

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCapsule.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualModel.h"

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledTrailer.h"

namespace amd_uw {

namespace {

// One shared file format constant per file, so a reader can reject the wrong thing
// loudly instead of decoding garbage.
constexpr char kFileMagic[8] = {'A', 'M', 'D', 'U', 'W', 'T', 'R', 'J'};
constexpr std::uint32_t kFormatVersion = 1;
constexpr std::uint32_t kFrameMagic = 0x544A5246u;  // 'TJRF'

template <typename T>
void Append(std::vector<char>& buffer, const T& value) {
    static_assert(std::is_trivially_copyable<T>::value, "raw append needs a POD");
    const char* p = reinterpret_cast<const char*>(&value);
    buffer.insert(buffer.end(), p, p + sizeof(T));
}

std::string JsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20)
                    out += ' ';
                else
                    out += c;
        }
    }
    return out;
}

std::string Vec3Json(const chrono::ChVector3d& v) {
    std::ostringstream s;
    s.precision(9);
    s << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
    return s.str();
}

std::string QuatJson(const chrono::ChQuaternion<>& q) {
    std::ostringstream s;
    s.precision(9);
    s << "[" << q.e0() << "," << q.e1() << "," << q.e2() << "," << q.e3() << "]";
    return s.str();
}

// One visual shape instance, as the consumer needs it: what to draw, how big, and
// where it sits IN THE BODY. See the header for why the local frame matters.
std::string ShapeJson(const chrono::ChVisualShapeInstance& instance) {
    const auto& shape = instance.shape;
    const chrono::ChFramed& frame = instance.frame;
    std::ostringstream s;
    s.precision(9);
    s << "{";

    if (auto mesh = std::dynamic_pointer_cast<chrono::ChVisualShapeTriangleMesh>(shape)) {
        // The filename survives CreateFromWavefrontFile even after the mesh has been
        // transformed in place, which is what the rocks do (scale, rotate, re-base on
        // the ground plane). So "file" names the SOURCE obj and "mesh_baked" warns that
        // the vertices Chrono drew are not that file's vertices unmodified.
        const auto& trimesh = mesh->GetMesh();
        s << "\"type\":\"trimesh\",\"file\":\"" << JsonEscape(trimesh ? trimesh->GetFileName() : "") << "\"";
        s << ",\"scale\":" << Vec3Json(mesh->GetScale());
        s << ",\"shape_name\":\"" << JsonEscape(mesh->GetName()) << "\"";
        // The bounding box of the geometry AS DRAWN, in body-local coordinates before
        // this instance's frame is applied. It is here because "scale" alone is a lie
        // for anything whose mesh was transformed in memory after loading: every rock
        // reports scale [1,1,1] and is drawn at 0.2, because LoadRockMesh bakes the
        // scale into the vertices and re-bases the mesh so its bottom sits at z=0. A
        // replacement mesh has to be fitted to this box, not to the source OBJ's.
        if (trimesh) {
            const chrono::ChAABB aabb = trimesh->GetBoundingBox();
            s << ",\"aabb_min\":" << Vec3Json(aabb.min) << ",\"aabb_max\":" << Vec3Json(aabb.max);
        }
    } else if (auto model = std::dynamic_pointer_cast<chrono::ChVisualShapeModelFile>(shape)) {
        s << "\"type\":\"modelfile\",\"file\":\"" << JsonEscape(model->GetFilename()) << "\"";
        s << ",\"scale\":" << Vec3Json(model->GetScale());
    } else if (auto box = std::dynamic_pointer_cast<chrono::ChVisualShapeBox>(shape)) {
        s << "\"type\":\"box\",\"size\":" << Vec3Json(box->GetLengths());
    } else if (auto sphere = std::dynamic_pointer_cast<chrono::ChVisualShapeSphere>(shape)) {
        s << "\"type\":\"sphere\",\"radius\":" << sphere->GetRadius();
    } else if (auto cyl = std::dynamic_pointer_cast<chrono::ChVisualShapeCylinder>(shape)) {
        s << "\"type\":\"cylinder\",\"radius\":" << cyl->GetRadius() << ",\"height\":" << cyl->GetHeight();
    } else if (auto cap = std::dynamic_pointer_cast<chrono::ChVisualShapeCapsule>(shape)) {
        s << "\"type\":\"capsule\",\"radius\":" << cap->GetRadius() << ",\"height\":" << cap->GetHeight();
    } else {
        s << "\"type\":\"other\"";
    }

    const chrono::ChColor c = shape->GetColor();
    s << ",\"pos\":" << Vec3Json(frame.GetPos());
    s << ",\"rot\":" << QuatJson(frame.GetRot());
    s << ",\"color\":[" << c.R << "," << c.G << "," << c.B << "]";
    s << "}";
    return s.str();
}

}  // namespace

TrajectoryRecorder::TrajectoryRecorder(const std::string& out_dir,
                                       int rank,
                                       double rate_hz,
                                       double step_size,
                                       bool write_static)
    : m_dir(out_dir), m_rank(rank), m_period(rate_hz > 0.0 ? 1.0 / rate_hz : 0.0), m_write_static(write_static) {
    if (m_period <= 0.0) {
        std::cout << "[traj] rank " << rank << ": record rate must be > 0; recording disabled.\n";
        return;
    }
    if (!m_dir.empty() && m_dir.back() != '/')
        m_dir += '/';

    std::error_code ec;
    std::filesystem::create_directories(m_dir, ec);
    if (ec) {
        std::cout << "[traj] rank " << rank << ": cannot create " << m_dir << " (" << ec.message()
                  << "); recording disabled.\n";
        return;
    }

    const std::string frames_path = m_dir + "rank_" + std::to_string(rank) + "_frames.bin";
    const std::string objects_path = m_dir + "rank_" + std::to_string(rank) + "_objects.jsonl";
    m_frames = std::fopen(frames_path.c_str(), "wb");
    m_objects = std::fopen(objects_path.c_str(), "wb");
    if (!m_frames || !m_objects) {
        std::cout << "[traj] rank " << rank << ": cannot open " << frames_path << "; recording disabled.\n";
        if (m_frames) std::fclose(m_frames);
        if (m_objects) std::fclose(m_objects);
        m_frames = m_objects = nullptr;
        return;
    }
    // A 4 MB stdio buffer, because the alternative is a write syscall 60 times a sim
    // second on every rank at once.
    std::setvbuf(m_frames, nullptr, _IOFBF, 4u << 20);

    std::vector<char> header;
    header.insert(header.end(), kFileMagic, kFileMagic + sizeof(kFileMagic));
    Append(header, kFormatVersion);
    Append(header, static_cast<std::uint32_t>(rank));
    Append(header, rate_hz);
    Append(header, step_size);
    std::fwrite(header.data(), 1, header.size(), m_frames);

    m_buffer.reserve(1u << 16);
    std::cout << "[traj] rank " << rank << ": recording to " << frames_path << " at " << rate_hz << " Hz.\n";
}

TrajectoryRecorder::~TrajectoryRecorder() {
    if (m_frames) {
        std::fflush(m_frames);
        std::fclose(m_frames);
    }
    if (m_objects)
        std::fclose(m_objects);
}

void TrajectoryRecorder::AddMeta(const std::string& key, const std::string& json_value) {
    m_meta.emplace_back(key, json_value);
}

void TrajectoryRecorder::ExcludeExisting(chrono::ChSystem* system) {
    if (!IsOpen() || !system)
        return;
    for (const auto& body : system->GetBodies())
        m_excluded[body.get()] = true;
}

void TrajectoryRecorder::Label(const std::shared_ptr<chrono::ChBody>& body,
                               const std::string& group,
                               const std::string& part) {
    if (!IsOpen() || !body)
        return;
    m_labels[body.get()] = {group, part};
}

void TrajectoryRecorder::LabelVehicle(chrono::vehicle::ChVehicle* vehicle, const std::string& group) {
    if (!IsOpen() || !vehicle)
        return;
    // GetBodyList is the vehicle's own inventory -- chassis, subchassis, suspension
    // links, spindles, steering, and (on a tracked vehicle) sprockets, idlers, road
    // wheels, rollers and every track shoe. Using it means a Chrono model change adds
    // its new bodies to the recording by itself.
    for (const auto& body : vehicle->GetBodyList())
        Label(body, group, body ? body->GetName() : std::string());
}

void TrajectoryRecorder::LabelTrackedVehicle(chrono::vehicle::ChTrackedVehicle* vehicle, const std::string& group) {
    if (!IsOpen() || !vehicle)
        return;
    LabelVehicle(vehicle, group);
    for (int side = 0; side < 2; ++side) {
        const auto assembly = vehicle->GetTrackAssembly(static_cast<chrono::vehicle::VehicleSide>(side));
        if (!assembly)
            continue;
        if (assembly->GetSprocket())
            Label(assembly->GetSprocket()->GetGearBody(), group, assembly->GetSprocket()->GetGearBody()->GetName());
        if (assembly->GetIdler())
            Label(assembly->GetIdler()->GetWheelBody(), group, assembly->GetIdler()->GetWheelBody()->GetName());
        for (size_t i = 0; i < assembly->GetNumTrackSuspensions(); ++i) {
            const auto suspension = assembly->GetTrackSuspension(i);
            if (suspension && suspension->GetWheelBody())
                Label(suspension->GetWheelBody(), group, suspension->GetWheelBody()->GetName());
        }
        for (size_t i = 0; i < assembly->GetNumRollers(); ++i) {
            const auto roller = assembly->GetRoller(i);
            if (roller && roller->GetBody())
                Label(roller->GetBody(), group, roller->GetBody()->GetName());
        }
        for (size_t i = 0; i < assembly->GetNumTrackShoes(); ++i) {
            const auto shoe = assembly->GetTrackShoe(i);
            if (shoe && shoe->GetShoeBody())
                Label(shoe->GetShoeBody(), group, shoe->GetShoeBody()->GetName());
        }
    }
}

void TrajectoryRecorder::LabelTrailer(chrono::vehicle::ChWheeledTrailer* trailer, const std::string& group) {
    if (!IsOpen() || !trailer)
        return;
    // ChWheeledTrailer is not a ChVehicle and has no GetBodyList, so its parts are
    // walked by hand.
    if (trailer->GetChassis())
        Label(trailer->GetChassis()->GetBody(), group, "chassis");
    // Only the chassis and the spindles are reachable from here: ChPart::GetBodyList is
    // protected, so a trailer's suspension links cannot be enumerated the way a
    // ChVehicle's can. They are still recorded -- the per-frame sweep takes every body in
    // the system -- they just carry their own Chrono names instead of this group.
    int axle_index = 0;
    for (const auto& axle : trailer->GetAxles()) {
        int wheel_index = 0;
        for (const auto& wheel : axle->GetWheels()) {
            Label(wheel->GetSpindle(), group,
                  "spindle_" + std::to_string(axle_index) + "_" + std::to_string(wheel_index));
            ++wheel_index;
        }
        ++axle_index;
    }
}

void TrajectoryRecorder::LabelArm(LrvArm* arm, const std::string& group) {
    if (!IsOpen() || !arm)
        return;
    for (const auto& body : arm->GetBodies())
        Label(body, group, body ? body->GetName() : std::string());
}

std::string TrajectoryRecorder::DescribeBody(const chrono::ChBody* body, int index, double time) {
    std::string group = "world";
    std::string part = body->GetName();
    const auto label = m_labels.find(body);
    if (label != m_labels.end()) {
        group = label->second.first;
        if (!label->second.second.empty())
            part = label->second.second;
    } else if (part.rfind("harvest_rock", 0) == 0 || part.rfind("seed_rock", 0) == 0) {
        // Rocks cannot be labelled up front: a fresh set is spawned on every harvest
        // cycle, long after the recorder was handed anything. They are named at creation
        // instead, and grouped from that name here.
        group = "rock";
    }
    if (part.empty())
        part = "body_" + std::to_string(body->GetIdentifier());

    // Make group/part unique. See m_name_uses.
    const int uses = ++m_name_uses[group + "/" + part];
    if (uses > 1)
        part += "#" + std::to_string(uses);

    const chrono::ChFrame<> frame = body->GetFrameRefToAbs();
    std::ostringstream s;
    s.precision(9);
    s << "{\"index\":" << index;
    s << ",\"group\":\"" << JsonEscape(group) << "\"";
    s << ",\"part\":\"" << JsonEscape(part) << "\"";
    s << ",\"chrono_name\":\"" << JsonEscape(body->GetName()) << "\"";
    s << ",\"chrono_id\":" << body->GetIdentifier();
    s << ",\"fixed\":" << (body->IsFixed() ? "true" : "false");
    // ChBody::GetMass() is not const-qualified in Chrono; the call reads a cached value
    // and mutates nothing.
    s << ",\"mass\":" << const_cast<chrono::ChBody*>(body)->GetMass();
    s << ",\"first_time\":" << time;
    s << ",\"first_pos\":" << Vec3Json(frame.GetPos());
    s << ",\"first_rot\":" << QuatJson(frame.GetRot());
    s << ",\"shapes\":[";
    if (const auto& model = body->GetVisualModel()) {
        const auto& instances = model->GetShapeInstances();
        for (size_t i = 0; i < instances.size(); ++i) {
            if (i)
                s << ",";
            s << ShapeJson(instances[i]);
        }
    }
    s << "]}";
    return s.str();
}

int TrajectoryRecorder::IndexOf(const chrono::ChBody* body, double time) {
    const auto it = m_index.find(body);
    if (it != m_index.end())
        return it->second;

    const int index = static_cast<int>(m_index.size());
    m_index[body] = index;
    const std::string line = DescribeBody(body, index, time) + "\n";
    std::fwrite(line.data(), 1, line.size(), m_objects);
    // Flushed on every new object rather than at exit: a run is stopped by Ctrl-C or
    // by MPI_Abort, neither of which runs this destructor, and a frame file whose
    // manifest is missing its last few objects cannot be decoded at all.
    std::fflush(m_objects);
    return index;
}

void TrajectoryRecorder::Start(chrono::ChSystem* system) {
    if (!IsOpen())
        return;

    const std::string meta_path = m_dir + "rank_" + std::to_string(m_rank) + "_meta.json";
    if (std::FILE* f = std::fopen(meta_path.c_str(), "wb")) {
        std::ostringstream s;
        s.precision(9);
        s << "{\"format\":\"amd-uw-trajectory\",\"version\":" << kFormatVersion;
        s << ",\"rank\":" << m_rank;
        s << ",\"rate_hz\":" << (m_period > 0.0 ? 1.0 / m_period : 0.0);
        s << ",\"pose_convention\":\"body reference frame (ChBody::GetFrameRefToAbs), "
             "quaternion [w,x,y,z], world = pose * shape_local_frame\"";
        for (const auto& kv : m_meta)
            s << ",\"" << JsonEscape(kv.first) << "\":" << kv.second;
        s << "}\n";
        const std::string text = s.str();
        std::fwrite(text.data(), 1, text.size(), f);
        std::fclose(f);
    }

    if (!m_write_static || !system)
        return;

    // Scenery, once. Same on every rank, so only one writes it.
    const std::string static_path = m_dir + "static_props.jsonl";
    std::FILE* f = std::fopen(static_path.c_str(), "wb");
    if (!f)
        return;
    int index = 0;
    for (const auto& body : system->GetBodies()) {
        if (!m_excluded.count(body.get()))
            continue;
        const std::string line = DescribeBody(body.get(), index++, 0.0) + "\n";
        std::fwrite(line.data(), 1, line.size(), f);
    }
    std::fclose(f);
    std::cout << "[traj] rank " << m_rank << ": " << index << " static props -> " << static_path << "\n";
}

void TrajectoryRecorder::CaptureIfDue(chrono::ChSystem* system, double time) {
    if (!IsOpen() || !system || time < m_next_sample)
        return;
    // Advance to the next grid point at or after `time`, so a step that overshoots the
    // sample instant does not leave the recorder permanently one sample behind and
    // firing on every subsequent step.
    m_next_sample += m_period * std::max(1.0, std::floor((time - m_next_sample) / m_period) + 1.0);

    m_buffer.clear();
    Append(m_buffer, kFrameMagic);
    Append(m_buffer, time);
    // Placeholder; the true count is patched in below, because it is not known until
    // the scenery has been filtered out.
    const size_t count_offset = m_buffer.size();
    Append(m_buffer, static_cast<std::uint32_t>(0));

    std::uint32_t count = 0;
    for (const auto& body : system->GetBodies()) {
        const chrono::ChBody* raw = body.get();
        if (m_excluded.count(raw))
            continue;
        const chrono::ChFrame<>& frame = body->GetFrameRefToAbs();
        const chrono::ChVector3d& p = frame.GetPos();
        const chrono::ChQuaternion<>& q = frame.GetRot();
        Append(m_buffer, static_cast<std::uint32_t>(IndexOf(raw, time)));
        Append(m_buffer, static_cast<float>(p.x()));
        Append(m_buffer, static_cast<float>(p.y()));
        Append(m_buffer, static_cast<float>(p.z()));
        Append(m_buffer, static_cast<float>(q.e0()));
        Append(m_buffer, static_cast<float>(q.e1()));
        Append(m_buffer, static_cast<float>(q.e2()));
        Append(m_buffer, static_cast<float>(q.e3()));
        ++count;
    }
    std::memcpy(m_buffer.data() + count_offset, &count, sizeof(count));

    std::fwrite(m_buffer.data(), 1, m_buffer.size(), m_frames);
    ++m_frame_count;
    // Once a sim second, so a run that is killed still has a readable, recent file and
    // so progress is observable from `ls -l` while it runs.
    if (m_frame_count % static_cast<long>(std::max(1.0, 1.0 / m_period)) == 0)
        std::fflush(m_frames);
}

}  // namespace amd_uw
