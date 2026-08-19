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


# ------------------------------------------------------------------------- blender

def build(dataset, rank, out_path=None, groups=None, mesh_dir=None, mesh_map=None,
          fit_aabb=True, start=None, end=None, stride=1, fps=None):
    import bpy
    from mathutils import Quaternion, Vector

    meta = load_meta(dataset, rank)
    objects = load_manifest(dataset, rank)
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
                child.display_type = "WIRE"
            child.name = f"{name}.shape{si}"

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

    if out_path:
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(out_path))
        print(f"[import] saved {out_path}")


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
    ap.add_argument("--start", type=float)
    ap.add_argument("--end", type=float)
    ap.add_argument("--stride", type=int, default=1, help="keyframe every Nth sample")
    ap.add_argument("--fps", type=int)
    args = ap.parse_args(argv)

    mesh_map = None
    if args.mesh_map:
        with open(args.mesh_map) as f:
            mesh_map = json.load(f)

    build(args.dataset, args.rank, args.out,
          groups=args.groups.split(",") if args.groups else None,
          mesh_dir=args.meshes, mesh_map=mesh_map, fit_aabb=not args.no_fit,
          start=args.start, end=args.end, stride=args.stride, fps=args.fps)


if __name__ == "__main__":
    main()
