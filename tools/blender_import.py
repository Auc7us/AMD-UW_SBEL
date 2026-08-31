#!/usr/bin/env python3
"""Import an AMD-UW pose recording into Blender as keyframed animation.

    blender --background --python blender_import.py -- --dataset DIR --rank 1 --out run.blend
    blender --python blender_import.py -- --dataset DIR --rank 1        # interactive

or paste into Blender's text editor and edit CONFIG at the bottom.

WHAT IT BUILDS

One Empty per recorded rigid body, keyframed at the recording's own rate, with the
body's visual shapes parented to it as static children. That mirrors how Chrono
actually draws the scene, and it is why the shapes are children rather than being
baked into the body: a Chrono body is not one mesh at its origin -- an M113 road wheel
is two wheel halves at +/-y, the trailer tub is four boxes -- so the world transform of
a shape is body_pose * shape_local_frame. Keyframing only the Empties also keeps the
keyframe count down by roughly the shape-per-body factor.

SWAPPING IN BETTER MESHES

Two ways, and they compose:

  --meshes DIR      look for each manifest mesh by BASENAME under DIR first, so a
                    directory of better OBJ/FBX/GLB files with matching names is picked
                    up with no mapping file at all.
  --mesh-map FILE   JSON, {"pattern": "/path/to/replacement.obj"}, where pattern is
                    matched against the manifest basename OR the object's group/part.
                    Later keys win, so you can set a default and override one part.

Replacements are FITTED to the recorded bounding box by default (--no-fit disables it).
That is not a nicety: Chrono bakes transforms into mesh vertices at load, so every rock
reports scale [1,1,1] while being drawn at 0.2 and re-based to sit on the ground. The
manifest therefore carries aabb_min/aabb_max -- the box the geometry was really drawn
in -- and fitting to it is what makes a replacement land at the right size and height.

Bodies that appear mid-run (rocks spawned by a later harvest cycle) are hidden until
their first frame, so nothing sits at the origin waiting to exist.

THE GROUND

The ruts are geometry, not a texture. rank_<r>_scm.bin carries every SCM node the run
deformed as (i, j, z) with ABSOLUTE heights, and the accumulated set becomes one mesh per
rank. Only nodes that actually moved are meshed -- a quad is emitted where all four of its
corner nodes were deformed -- so what lands in Blender is the track network itself: a 92 s
two-rover run at delta 0.02 churns 182 k nodes covering 73 m^2, inside a patch 512 m
square that would otherwise be 2.6 G nodes of flat nothing. A `sinkage` point attribute
carries how far each node fell below the first height ever recorded for it, which is what
a shader needs to darken a rut without a separate depth pass.

  --no-scm                 skip it
  --scm-ranks 1,2          ruts from other ranks too (default: --rank only)
  --scm-at T               accumulate to sim time T instead of to the end
  --scm-decimate N         keep every Nth node in i and j; verts fall by N^2
  --scm-animate            keyframe the ground instead of importing its final state
  --scm-key-stride N       one shape key per N recorded samples (default 10 = 1/s)

Animation is shape keys, so it costs verts x keys x 12 bytes and that number is printed
before anything is built. Topology is fixed for the whole shot -- shape keys cannot add
vertices -- so a node not yet deformed at a given key rests at the FIRST height ever
recorded for it, and the not-yet-rutted ground lies roughly flat instead of appearing out
of nowhere. Roughly, not exactly: a node is first written AFTER its first deformation, and
at 10 Hz a track crossing at 1.5 m/s moves 0.15 m between samples, so some of that node's
sinkage is already in its first sample. The whole track network is therefore faintly
pre-drawn, bounded by the total sinkage the run produced (0.16 m on the 92 s run above).
If that shows in a shot, render the static ground instead and cut on the frame you want.

static_props.jsonl is imported too, unanimated: the three rings, the centre pad, and the
rocks already laid in the wall. It uses the same shape schema as the bodies, so the only
difference is that nothing gets keyframed and recorded colours become materials.
"""

import json
import math
import os
import re
import struct
import sys

HEADER = struct.Struct("<8sIIdd")
FRAME_HEADER = struct.Struct("<IdI")
RECORD = struct.Struct("<I7f")
FILE_MAGIC = b"AMDUWTRJ"
FRAME_MAGIC = 0x544A5246


# --------------------------------------------------------------------------- reading

def load_manifest(dataset, rank):
    path = os.path.join(dataset, f"rank_{rank}_objects.jsonl")
    objects = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                objects.append(json.loads(line))
    return objects


def load_meta(dataset, rank):
    path = os.path.join(dataset, f"rank_{rank}_meta.json")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return json.load(f)


def read_frames(dataset, rank, start=None, end=None, stride=1):
    """Yields (time, {index: (pos, quat)}). Tolerates a truncated final frame."""
    path = os.path.join(dataset, f"rank_{rank}_frames.bin")
    with open(path, "rb") as f:
        head = f.read(HEADER.size)
        magic, version, file_rank, rate, step = HEADER.unpack(head)
        if magic != FILE_MAGIC:
            raise ValueError(f"{path}: not an AMD-UW recording")
        n = 0
        while True:
            fh = f.read(FRAME_HEADER.size)
            if len(fh) < FRAME_HEADER.size:
                return
            fmagic, time, count = FRAME_HEADER.unpack(fh)
            if fmagic != FRAME_MAGIC:
                raise ValueError(f"{path}: lost frame sync at t={time}")
            payload = f.read(RECORD.size * count)
            if len(payload) < RECORD.size * count:
                return  # interrupted run; the partial tail is not a frame
            if end is not None and time > end:
                return
            take = (start is None or time >= start) and (n % stride == 0)
            n += 1
            if not take:
                continue
            poses = {}
            for i in range(count):
                idx, px, py, pz, qw, qx, qy, qz = RECORD.unpack_from(payload, i * RECORD.size)
                poses[idx] = ((px, py, pz), (qw, qx, qy, qz))
            yield time, poses


SCM_HEADER = struct.Struct("<8sII" + "d" * 9 + "ii")   # magic, version, rank, rate,
                                                       # delta, plane pos+quat, nx, ny
SCM_FRAME = struct.Struct("<IdI")
SCM_NODE = struct.Struct("<iif")
SCM_MAGIC = b"AMDUWSCM"
SCM_FRAME_MAGIC = 0x4D435353


def scm_path(dataset, rank):
    return os.path.join(dataset, f"rank_{rank}_scm.bin")


def read_scm_header(path):
    """(delta, plane) -- the node pitch, and the patch frame the indices are expressed in.

    Node (i, j) sits at (i * delta, j * delta, z) in the plane frame, which is what makes
    the indices mean anything. The plane is the identity on every run recorded so far, but
    it is carried and applied rather than assumed.
    """
    with open(path, "rb") as f:
        raw = f.read(SCM_HEADER.size)
    if len(raw) < SCM_HEADER.size:
        raise ValueError(f"{path}: shorter than its header")
    v = SCM_HEADER.unpack_from(raw, 0)
    if v[0] != SCM_MAGIC:
        raise ValueError(f"{path}: bad magic {v[0]!r}")
    return v[4], v[5:12]


def scm_frames(path):
    """Yield (time, raw node block); unpack a block with SCM_NODE.iter_unpack.

    Streamed, and the block handed over raw, because this file is the big one -- 44 MB for
    92 s of two rovers, and it grows with sim time while the pose recording does not.
    Materialising every frame as tuples costs several hundred MB for a run that a caller
    is only going to fold into one dict anyway, and the two passes the animated build
    needs are cheaper as two reads than as one resident copy.
    """
    with open(path, "rb") as f:
        f.seek(SCM_HEADER.size)
        while True:
            raw = f.read(SCM_FRAME.size)
            if len(raw) < SCM_FRAME.size:
                return
            magic, time, count = SCM_FRAME.unpack_from(raw, 0)
            if magic != SCM_FRAME_MAGIC:
                raise ValueError(f"{path}: lost frame sync at t={time}")
            need = SCM_NODE.size * count
            block = f.read(need)
            if len(block) < need:
                return  # interrupted run; the partial tail is not a frame
            yield time, block


# ------------------------------------------------------------------- mesh resolution

def resolve_mesh(src_path, group_part, mesh_dir, mesh_map):
    """Pick the file to actually load for a manifest mesh reference."""
    base = os.path.basename(src_path)
    for pattern, replacement in (mesh_map or {}).items():
        if re.search(pattern, base) or re.search(pattern, group_part):
            return replacement
    if mesh_dir:
        for root, _dirs, files in os.walk(mesh_dir):
            if base in files:
                return os.path.join(root, base)
            stem = os.path.splitext(base)[0]
            for candidate in files:
                if os.path.splitext(candidate)[0] == stem:
                    return os.path.join(root, candidate)
    return src_path if os.path.exists(src_path) else None


# Same hull lift replay_run.py applies, and for the same reason: the squashed builder
# hull's deck sits at z=0.204 over the tracks while the top run of the chain reaches 0.272,
# so the shoes saw through it by up to 0.166 m. Visual only -- poses, tracks and arm are
# left exactly as recorded.
HULL_LIFT_Z = 0.20
HULL_SHAPE_NAME = "Builder_Chassis_Squashed_Z"


def lift_hull(objects, lift):
    """Raise the builder hull visual by `lift` in the chassis frame, nothing else."""
    if abs(lift) < 1e-9:
        return
    for obj in objects:
        for shape in obj.get("shapes", []):
            if shape.get("shape_name") == HULL_SHAPE_NAME:
                pos = list(shape.get("pos", [0.0, 0.0, 0.0]))
                pos[2] += lift
                shape["pos"] = pos


# ------------------------------------------------------------------------- blender

def build(dataset, rank, out_path=None, groups=None, mesh_dir=None, mesh_map=None,
          fit_aabb=True, start=None, end=None, stride=1, fps=None, props=True,
          scm=True, scm_ranks=None, scm_at=None, scm_decimate=1, scm_animate=False,
          scm_key_stride=10, hull_lift=HULL_LIFT_Z):
    import bpy
    from mathutils import Quaternion, Vector

    meta = load_meta(dataset, rank)
    objects = load_manifest(dataset, rank)
    lift_hull(objects, hull_lift)
    rate = float(meta.get("rate_hz") or 60.0)
    fps = int(round(fps or rate))

    wanted = set(groups) if groups else None
    keep = {o["index"]: o for o in objects if wanted is None or o["group"] in wanted}
    print(f"[import] rank {rank}: {len(keep)} of {len(objects)} objects, {fps} fps")

    scene = bpy.context.scene
    scene.render.fps = fps
    scene.unit_settings.system = "METRIC"

    root = bpy.data.objects.new(f"rank_{rank}", None)
    root.empty_display_size = 1.0
    scene.collection.objects.link(root)

    mesh_cache = {}

    def import_mesh(path):
        """Load a mesh file once and return a template object to copy from."""
        if path in mesh_cache:
            return mesh_cache[path]
        before = set(bpy.data.objects)
        ext = os.path.splitext(path)[1].lower()
        try:
            if ext == ".obj":
                # forward='Y', up='Z' means NO axis conversion. Chrono reads OBJ
                # verbatim and both Chrono and Blender are Z-up right-handed, so the
                # importer's default -Z/Y remap would rotate every part 90 degrees.
                if hasattr(bpy.ops.wm, "obj_import"):
                    bpy.ops.wm.obj_import(filepath=path, forward_axis="Y", up_axis="Z")
                else:
                    bpy.ops.import_scene.obj(filepath=path, axis_forward="Y", axis_up="Z")
            elif ext in (".glb", ".gltf"):
                bpy.ops.import_scene.gltf(filepath=path)
            elif ext == ".fbx":
                bpy.ops.import_scene.fbx(filepath=path)
            elif ext == ".stl":
                if hasattr(bpy.ops.wm, "stl_import"):
                    bpy.ops.wm.stl_import(filepath=path)
                else:
                    bpy.ops.import_mesh.stl(filepath=path)
            else:
                mesh_cache[path] = None
                return None
        except Exception as exc:                                  # noqa: BLE001
            print(f"[import] FAILED {path}: {exc}")
            mesh_cache[path] = None
            return None
        new = [o for o in bpy.data.objects if o not in before]
        if not new:
            mesh_cache[path] = None
            return None
        if len(new) > 1:                     # join multi-object files into one template
            bpy.ops.object.select_all(action="DESELECT")
            for o in new:
                o.select_set(True)
            bpy.context.view_layer.objects.active = new[0]
            bpy.ops.object.join()
            new = [bpy.context.view_layer.objects.active]
        template = new[0]
        for coll in list(template.users_collection):
            coll.objects.unlink(template)    # keep the template out of the scene
        mesh_cache[path] = template
        return template

    def local_bbox(obj):
        cs = [Vector(c) for c in obj.bound_box]
        lo = Vector((min(c.x for c in cs), min(c.y for c in cs), min(c.z for c in cs)))
        hi = Vector((max(c.x for c in cs), max(c.y for c in cs), max(c.z for c in cs)))
        return lo, hi

    materials = {}

    def material_for(color):
        """One material per distinct recorded colour, shared by every shape using it.

        The scenery is where this matters: the ring markers and the pad carry no mesh
        file, so colour is the only thing separating a work-circle marker from any other
        box in the scene, and there are 540 of them.
        """
        if not color:
            return None
        key = tuple(round(float(c), 4) for c in tuple(color)[:3])
        mat = materials.get(key)
        if mat is None:
            mat = bpy.data.materials.new("amduw_%.2f_%.2f_%.2f" % key)
            mat.diffuse_color = (key[0], key[1], key[2], 1.0)
            mat.use_nodes = True
            bsdf = mat.node_tree.nodes.get("Principled BSDF")
            if bsdf:
                bsdf.inputs["Base Color"].default_value = (key[0], key[1], key[2], 1.0)
            materials[key] = mat
        return mat

    def attach_shapes(obj, name, empty, solid_primitives=False):
        """Parent this body's visual shapes to its Empty, each under its own shape frame.

        solid_primitives distinguishes the two reasons a shape has no mesh file. On a body
        it means the mesh could not be loaded, and a wire box saying "something this size
        belongs here" is the useful failure. On the scenery it means the box IS the
        geometry -- the rings are 180 painted boxes each -- and drawing those as wire
        would render the site markings invisible.
        """
        for si, shape in enumerate(obj.get("shapes", [])):
            child = None
            src = shape.get("file")
            if src:
                target = resolve_mesh(src, name, mesh_dir, mesh_map)
                template = import_mesh(target) if target else None
                if template is not None:
                    child = template.copy()
                    child.data = template.data          # shared mesh data, cheap
                    scene.collection.objects.link(child)
            if child is None:
                # No mesh (primitives, or a file we could not load): a wire box of the
                # recorded size is more useful than nothing -- it shows where the part is
                # and how big, which is what a substitute has to match.
                size = shape.get("size") or [0.1, 0.1, 0.1]
                bpy.ops.mesh.primitive_cube_add(size=1.0)
                child = bpy.context.active_object
                child.scale = (size[0], size[1], size[2])
                if not solid_primitives:
                    child.display_type = "WIRE"
            child.name = f"{name}.shape{si}"
            mat = material_for(shape.get("color"))
            if mat is not None and child.type == "MESH" and not child.data.materials:
                child.data.materials.append(mat)

            if fit_aabb and shape.get("aabb_min") and child.type == "MESH":
                lo, hi = local_bbox(child)
                tgt_lo = Vector(shape["aabb_min"])
                tgt_hi = Vector(shape["aabb_max"])
                cur = hi - lo
                want = tgt_hi - tgt_lo
                s = [(want[i] / cur[i]) if abs(cur[i]) > 1e-9 else 1.0 for i in range(3)]
                child.scale = s
                centre_now = Vector([(lo[i] + hi[i]) * 0.5 * s[i] for i in range(3)])
                centre_want = (tgt_lo + tgt_hi) * 0.5
                child.location = centre_want - centre_now
            else:
                child.location = (0.0, 0.0, 0.0)

            # The shape's own frame inside the body, applied on top of any aabb fit by
            # parenting rather than by multiplying it in -- so a fitted child keeps its
            # own local offset and the parent carries the Chrono shape frame.
            holder = bpy.data.objects.new(f"{name}.frame{si}", None)
            holder.empty_display_size = 0.05
            holder.rotation_mode = "QUATERNION"
            holder.location = shape.get("pos", [0, 0, 0])
            holder.rotation_quaternion = Quaternion(shape.get("rot", [1, 0, 0, 0]))
            scene.collection.objects.link(holder)
            holder.parent = empty
            child.parent = holder

    empties = {}
    for index, obj in sorted(keep.items()):
        name = f"{obj['group']}.{obj['part']}"
        empty = bpy.data.objects.new(name, None)
        empty.empty_display_type = "ARROWS"
        empty.empty_display_size = 0.25
        empty.rotation_mode = "QUATERNION"
        empty.parent = root
        scene.collection.objects.link(empty)
        empties[index] = empty
        attach_shapes(obj, name, empty)

    # ------------------------------------------------------------------ keyframes
    first_seen = {}
    n_frames = 0
    frame_no = 0
    for time, poses in read_frames(dataset, rank, start, end, stride):
        frame_no = int(round(time * fps)) + 1
        n_frames += 1
        for index, (pos, quat) in poses.items():
            empty = empties.get(index)
            if empty is None:
                continue
            empty.location = pos
            empty.rotation_quaternion = Quaternion(quat)
            empty.keyframe_insert("location", frame=frame_no)
            empty.keyframe_insert("rotation_quaternion", frame=frame_no)
            if index not in first_seen:
                first_seen[index] = frame_no
        if n_frames % 600 == 0:
            print(f"[import] {n_frames} frames, t={time:.2f}")

    # Bodies that did not exist at t=0 stay hidden until they do.
    for index, frame in first_seen.items():
        empty = empties[index]
        if frame <= 1:
            continue
        for attr in ("hide_viewport", "hide_render"):
            setattr(empty, attr, True)
            empty.keyframe_insert(attr, frame=frame - 1)
            setattr(empty, attr, False)
            empty.keyframe_insert(attr, frame=frame)

    scene.frame_start = 1
    scene.frame_end = max(frame_no, 1)
    print(f"[import] {n_frames} frames -> blender frames 1..{scene.frame_end}")

    # -------------------------------------------------------------- static scenery
    if props:
        prop_path = os.path.join(dataset, "static_props.jsonl")
        if not os.path.exists(prop_path):
            print("[import] no static_props.jsonl; skipping scenery")
        else:
            holder = bpy.data.objects.new("static_props", None)
            holder.empty_display_size = 1.0
            scene.collection.objects.link(holder)
            n_props = 0
            with open(prop_path) as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    obj = json.loads(line)
                    name = f"{obj['group']}.{obj['part']}"
                    # Fixed bodies: placed once from the manifest's own first pose, with
                    # no keyframes at all. Anything in this file is scenery by definition
                    # -- it is here precisely because it never moves.
                    empty = bpy.data.objects.new(name, None)
                    empty.empty_display_size = 0.1
                    empty.rotation_mode = "QUATERNION"
                    empty.location = obj.get("first_pos", [0, 0, 0])
                    empty.rotation_quaternion = Quaternion(obj.get("first_rot", [1, 0, 0, 0]))
                    empty.parent = holder
                    scene.collection.objects.link(empty)
                    attach_shapes(obj, name, empty, solid_primitives=True)
                    n_props += 1
            print(f"[import] {n_props} static props")

    # ------------------------------------------------------------------ the ground
    if scm:
        build_scm(dataset, scm_ranks or [rank], scene, fps,
                  scm_at if scm_at is not None else end,
                  scm_decimate, scm_animate, scm_key_stride)

    if out_path:
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(out_path))
        print(f"[import] saved {out_path}")


def build_scm(dataset, ranks, scene, fps, until, decimate, animate, key_stride):
    """Mesh each rank's deformed ground and, optionally, keyframe it.

    Accumulation is by OVERWRITE, because the recorder writes absolute heights and only
    the nodes that changed: the last value written to a node is that node's height, and
    the periodic keyframes that restate everything so far are therefore idempotent rather
    than a case to detect.

    Two heights are kept per node, not one. `last` is what the ground ends up as and is
    what the static mesh draws; `first` is the height at the node's first RECORDED
    deformation, which stands in for the undisturbed terrain there. The recording does not
    carry the base heightmap, so their difference is the only sinkage available from this
    file alone. It reads slightly low -- whatever the node sank within its first 0.1 s
    sample is already in `first` and is invisible here -- and it is still the number a
    shader wants, because it is zero on ground nothing touched and grows with the rut.
    """
    import bpy
    from mathutils import Quaternion, Vector

    holder = None
    for rank in ranks:
        path = scm_path(dataset, rank)
        if not os.path.exists(path):
            print(f"[import] no rank_{rank}_scm.bin; no ruts for that rank")
            continue
        delta, plane = read_scm_header(path)
        pq = Quaternion((plane[3], plane[4], plane[5], plane[6]))
        ppos = Vector((plane[0], plane[1], plane[2]))
        identity = (abs(plane[0]) < 1e-9 and abs(plane[1]) < 1e-9 and abs(plane[2]) < 1e-9
                    and abs(plane[3] - 1.0) < 1e-9)

        def to_world(x, y, z):
            if identity:
                return (x, y, z)
            return tuple(ppos + pq @ Vector((x, y, z)))

        first, last = {}, {}
        n_rows = n_frames = 0
        for time, block in scm_frames(path):
            if until is not None and time > until:
                break
            n_frames += 1
            for i, j, z in SCM_NODE.iter_unpack(block):
                n_rows += 1
                if decimate > 1 and (i % decimate or j % decimate):
                    continue
                key = (i, j)
                if key not in first:
                    first[key] = z
                last[key] = z
        if not last:
            print(f"[import] rank {rank}: nothing deformed")
            continue

        # A quad only where all four of its corners were deformed. The alternative is to
        # fill the bounding box of the lane, and a rank's ruts are a long diagonal track:
        # the box around one is mostly not the track, and filling it would bury every
        # other rank's ruts under an opaque sheet -- which is exactly the failure the
        # viewer had before it started covering the touched set instead.
        nodes = sorted(last)
        at = {key: n for n, key in enumerate(nodes)}
        verts = [to_world(i * delta, j * delta, last[(i, j)]) for i, j in nodes]
        sink = [first[k] - last[k] for k in nodes]
        d = decimate
        faces = []
        for n, (i, j) in enumerate(nodes):
            b = at.get((i + d, j))
            c = at.get((i + d, j + d))
            e = at.get((i, j + d))
            if b is not None and c is not None and e is not None:
                faces.append((n, b, c, e))    # ccw seen from +z, so normals point up

        mesh = bpy.data.meshes.new(f"scm.rank{rank}")
        mesh.from_pydata(verts, [], faces)
        mesh.validate()
        mesh.update()
        attr = mesh.attributes.new("sinkage", "FLOAT", "POINT")
        attr.data.foreach_set("value", sink)
        ground = bpy.data.objects.new(f"scm.rank{rank}", mesh)
        if holder is None:
            holder = bpy.data.objects.new("scm", None)
            holder.empty_display_size = 1.0
            scene.collection.objects.link(holder)
        ground.parent = holder
        scene.collection.objects.link(ground)
        print(f"[import] rank {rank}: {len(nodes)} nodes -> {len(faces)} quads "
              f"({len(nodes) * delta * delta:.1f} m^2), from {n_rows} rows in {n_frames} "
              f"samples, sinkage 0..{max(sink):.3f} m")

        if not animate:
            continue

        # Shape keys, one per key_stride recorded samples. Every key stores a full copy of
        # the vertex buffer, so say what that costs before spending it rather than after.
        n_keys = max(1, n_frames // max(1, key_stride))
        print(f"[import] rank {rank}: ~{n_keys} shape keys x {len(nodes)} verts "
              f"= ~{n_keys * len(nodes) * 12 / 1e6:.0f} MB")
        ground.shape_key_add(name="Basis", from_mix=False)
        z_now = dict(first)
        frames_done = []
        for n, (time, block) in enumerate(scm_frames(path)):
            if until is not None and time > until:
                break
            for i, j, z in SCM_NODE.iter_unpack(block):
                if (i, j) in at:
                    z_now[(i, j)] = z
            if n % key_stride:
                continue
            flat = []
            for i, j in nodes:
                flat.extend(to_world(i * delta, j * delta, z_now[(i, j)]))
            sk = ground.shape_key_add(name=f"t{time:.2f}", from_mix=False)
            sk.data.foreach_set("co", flat)
            frames_done.append((int(round(time * fps)) + 1, sk))

        # Crossfade neighbours: with relative keys, A at (1-t) and B at t evaluates to
        # exactly (1-t)A + tB, so a linear ramp between consecutive keys IS linear
        # interpolation of the ground, not an approximation of it.
        end_frame = max(scene.frame_end, frames_done[-1][0] if frames_done else 1)
        for n, (frame, sk) in enumerate(frames_done):
            sk.value = 0.0
            if n:
                sk.keyframe_insert("value", frame=frames_done[n - 1][0])
            else:
                sk.value = 1.0
                sk.keyframe_insert("value", frame=1)
            sk.value = 1.0
            sk.keyframe_insert("value", frame=frame)
            if n + 1 < len(frames_done):
                sk.value = 0.0
                sk.keyframe_insert("value", frame=frames_done[n + 1][0])
            else:
                sk.keyframe_insert("value", frame=end_frame)
        scene.frame_end = end_frame


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    import argparse
    ap = argparse.ArgumentParser(prog="blender_import.py")
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--rank", type=int, required=True)
    ap.add_argument("--out")
    ap.add_argument("--groups", help="comma-separated subset, e.g. collector,rock")
    ap.add_argument("--meshes", help="directory of replacement meshes, matched by basename")
    ap.add_argument("--mesh-map", help="JSON file of {pattern: replacement path}")
    ap.add_argument("--no-fit", action="store_true", help="do not fit meshes to the recorded aabb")
    ap.add_argument("--hull-lift", type=float, default=HULL_LIFT_Z,
                    help="raise the builder hull mesh by this many metres so the track "
                         f"shoes stop cutting through its deck; 0 disables (default {HULL_LIFT_Z:g})")
    ap.add_argument("--start", type=float)
    ap.add_argument("--end", type=float)
    ap.add_argument("--stride", type=int, default=1, help="keyframe every Nth sample")
    ap.add_argument("--fps", type=int)
    ap.add_argument("--no-props", action="store_true",
                    help="skip static_props.jsonl (rings, pad, laid wall rocks)")
    ap.add_argument("--no-scm", action="store_true", help="skip the deformed ground")
    ap.add_argument("--scm-ranks", help="comma-separated ranks whose ruts to import "
                                        "(default: --rank alone)")
    ap.add_argument("--scm-at", type=float,
                    help="accumulate ground to this sim time (default: --end, else all)")
    ap.add_argument("--scm-decimate", type=int, default=1,
                    help="keep every Nth node in i and j; vertex count falls by N^2")
    ap.add_argument("--scm-animate", action="store_true",
                    help="keyframe the ground as shape keys instead of its final state")
    ap.add_argument("--scm-key-stride", type=int, default=10,
                    help="one shape key per N recorded samples (default 10 = 1 per second)")
    args = ap.parse_args(argv)

    mesh_map = None
    if args.mesh_map:
        with open(args.mesh_map) as f:
            mesh_map = json.load(f)

    build(args.dataset, args.rank, args.out,
          groups=args.groups.split(",") if args.groups else None,
          mesh_dir=args.meshes, mesh_map=mesh_map, fit_aabb=not args.no_fit,
          start=args.start, end=args.end, stride=args.stride, fps=args.fps,
          props=not args.no_props, scm=not args.no_scm,
          scm_ranks=[int(r) for r in args.scm_ranks.split(",")] if args.scm_ranks else None,
          scm_at=args.scm_at, scm_decimate=max(1, args.scm_decimate),
          scm_animate=args.scm_animate, scm_key_stride=max(1, args.scm_key_stride),
          hull_lift=args.hull_lift)


if __name__ == "__main__":
    main()
