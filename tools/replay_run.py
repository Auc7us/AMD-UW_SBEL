#!/usr/bin/env python3
"""Real-time 3D playblast of a --record_dir recording, with the run's own meshes.

A previz viewer: the actual OBJ assets the sim rendered, played back on the wall clock,
scrubbable, orbitable, and optionally written straight to a movie. No Blender, no import
step, no scene to maintain -- the recording's object manifest names every mesh file and
its local frame, so the scene builds itself from the run.

This is the standalone high-throughput version: dynamic bodies use GPU instancing, SCM
ranks decode concurrently, and deformation meshes upload in dirty chunks.

    replay_run.py <dir>                       interactive window, real time
    replay_run.py <dir> --speed 4             4x
    replay_run.py <dir> --rank 1,2 --to 240   one sector, first 4 minutes
    replay_run.py <dir> --boxes               bounding boxes only, for speed
    replay_run.py <dir> --movie run16.mp4     off-screen render to mp4
    replay_run.py <dir> --shot look.png --at 120

Keys: space play/pause, left/right step a frame, [ ] speed.
      ijkl flies the camera the way wasd does -- i/k forward and back over the ground,
      j/l strafe, u/o up and down, up/down zoom.
      t top view, c free camera, f follow the next machine, r restart, q quit.

Meshes are loaded once per file and shared between the bodies that use them, so 15 ranks
of builders cost one hull mesh, not fifteen. Track shoes, lugs, road wheels and suspension
are drawn by default; --no-running-gear drops them (1905 of the 3472 bodies in a 15-rank
run) when frame rate matters more than looks.
"""

import argparse
import bisect
import math
import os
import re
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_trajectory import (Recording, discover_ranks, index_frames,  # noqa: E402
                             FRAME_HEADER, FRAME_MAGIC)

# Running gear: track shoes and their lugs, road wheels, sprockets, idlers, suspension
# arms, wheel spindles. Drawn by DEFAULT -- the tracks are most of what a tracked machine
# looks like, and the lugs are on the shoe mesh, so dropping them leaves a builder as a
# coloured plate sliding over the ground. It is 1905 of the 3472 bodies in a 15-rank run,
# so --no-running-gear takes them out again when frame rate matters more than looks.
RUNNING_GEAR = re.compile(
    r"TrackShoe|RoadWheel|Suspension|Idler|Sprocket|DoubleWishbone|Rack-Pinion"
    r"|spindle|axleTube|ballast",
    re.IGNORECASE,
)

# The builder hull mesh is squashed to 0.110x in z (BuilderRig::AddSquashedChassisVisual)
# so it does not bury the arm, which leaves its deck at z=0.204 over the tracks while the
# top run of the chain and the drive sprocket stand at 0.272 and 0.258. Measured over 40
# frames of run_20260827_211044 on ranks 1-8, the shoes stand up to 0.166 m proud of the
# hull surface directly above them -- they visibly saw through the deck. This lifts the
# hull VISUAL only: the physics, the tracks, the arm and every recorded pose are untouched,
# so the render disagrees with the sim by exactly this offset and nothing else. 0.20 m
# clears the worst shoe by 3.4 cm; it also sinks the arm's pedestal block (0.324..0.413)
# into the deck, so the arm reads as deck-mounted rather than on a post.
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


# Fallback colours, by group, for a body whose shapes carry none.
GROUP_COLORS = {
    "collector": "#3b82f6",
    "collector_trailer": "#22d3ee",
    "collector_arm": "#14b8a6",
    "builder": "#ef4444",
    "builder_arm": "#f472b6",
    "world": "#475569",
}


def group_color(obj):
    if obj["group"] == "rock":
        return "#94a3b8" if obj["part"].startswith("seed_rock") else "#facc15"
    return GROUP_COLORS.get(obj["group"], "#a3a3a3")


def body_bounds(obj):
    """Axis-aligned bounds of a body in ITS OWN frame, over all its visual shapes.

    Only needed for the fallback box of a body whose shapes could not be loaded at all.
    Each shape's AABB corners are pushed through that shape's local frame before the union
    is taken -- the same body_pose * shape_frame * shape_aabb chain read_trajectory.py
    --bbox validates.
    """
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for shape in obj.get("shapes", []):
        amin, amax = shape.get("aabb_min"), shape.get("aabb_max")
        if not amin or not amax:
            continue
        scale = shape.get("scale", [1.0, 1.0, 1.0])
        spos = shape.get("pos", [0.0, 0.0, 0.0])
        rot = quat_matrix(shape.get("rot", [1.0, 0.0, 0.0, 0.0]))
        for cx in (amin[0], amax[0]):
            for cy in (amin[1], amax[1]):
                for cz in (amin[2], amax[2]):
                    p = rot @ np.array([cx * scale[0], cy * scale[1], cz * scale[2]])
                    for i in range(3):
                        lo[i] = min(lo[i], p[i] + spos[i])
                        hi[i] = max(hi[i], p[i] + spos[i])
    if not all(math.isfinite(v) for v in lo + hi):
        return [-0.25] * 3, [0.25] * 3
    return lo, hi


def quat_matrix(q):
    """3x3 rotation from (w, x, y, z)."""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def local_roots(extra):
    """Directories to look for a mesh under, when its recorded path does not exist here.

    Recordings carry ABSOLUTE paths from the machine that ran the sim, so anything
    recorded on the cluster names /work1/... and resolves to nothing locally. Rather than
    demand a flag, the paths are re-rooted: they all contain a `data/` segment, and what
    follows it is stable across checkouts.
    """
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    roots = [os.path.join(here, "data"), here]
    for env in ("CHRONO_DATA_DIR", "CHRONO_DATA_PATH"):
        if os.environ.get(env):
            roots.append(os.environ[env])
    # The sibling Chrono checkout, which is where the shared robot/rock meshes live.
    for guess in ("chrono/build/data", "chrono/data", "../chrono/build/data"):
        cand = os.path.normpath(os.path.join(os.path.dirname(here), guess))
        if os.path.isdir(cand):
            roots.append(cand)
    roots.extend(extra or [])
    return [r for r in roots if os.path.isdir(r)]


def relocate(path, roots):
    """Find `path` locally, matching progressively shorter tails against each root."""
    if not path:
        return None
    if os.path.exists(path):
        return path
    parts = [p for p in path.replace("\\", "/").split("/") if p]
    # Prefer the tail after the last `data/` segment; fall back to ever-shorter tails, so
    # a mesh moved between data roots is still found by its own subtree.
    starts = [i + 1 for i, part in enumerate(parts) if part == "data"]
    starts.extend(range(len(parts) - 1, 0, -1))
    for start in starts:
        tail = os.path.join(*parts[start:])
        for root in roots:
            cand = os.path.join(root, tail)
            if os.path.exists(cand):
                return cand
    return None


class MeshCache:
    """PolyData cache, keyed twice over.

    Once per (file, scale) for the raw mesh, and once per fully-placed shape signature --
    mesh, fit box and local frame -- because that is what actually repeats. A 15-rank run
    has 1905 track shoes that are the same mesh, fitted to the same box, at the same local
    frame, differing only in body pose, and body pose lives on the actor. Rebuilding them
    one by one cost three minutes of scene build; sharing them costs one mesh.
    """

    def __init__(self, roots=None):
        self._cache = {}
        self._placed = {}
        self._resolved = {}
        self.roots = roots or []
        self.misses = set()
        self.relocated = 0

    def get(self, path, scale):
        key = (path, tuple(scale))
        if key in self._cache:
            return self._cache[key]
        import pyvista as pv

        local = self._resolved.get(path)
        if local is None:
            local = relocate(path, self.roots)
            self._resolved[path] = local or False
            if local and local != path:
                self.relocated += 1
        elif local is False:
            local = None

        mesh = None
        if local:
            try:
                mesh = pv.read(local)
                if any(abs(s - 1.0) > 1e-9 for s in scale):
                    mesh = mesh.scale(scale, inplace=False)
            except Exception as exc:  # noqa: BLE001 - one bad asset must not kill the scene
                print(f"  ! {os.path.basename(local)}: {exc}", file=sys.stderr)
                mesh = None
        elif path:
            self.misses.add(path)
        self._cache[key] = mesh
        return mesh

    def loaded(self):
        return sum(1 for v in self._cache.values() if v is not None)

    def placed(self, signature, build):
        """Memoised placed geometry. `build` is only called on a miss."""
        if signature not in self._placed:
            self._placed[signature] = build()
        return self._placed[signature]


def fit_to_aabb(mesh, amin, amax):
    """Map a mesh's own bounds onto the bounds Chrono actually DREW, per axis.

    This is the difference between a plausible-looking scene and a correct one, and the
    recorder says so in as many words: "scale" alone is a lie for anything whose mesh was
    transformed in memory after loading. Two cases in this project, both wrong by a lot
    without this:

      * every rock reports scale [1,1,1] and is drawn at 0.2, because LoadRockMesh bakes
        the scale into the vertices and re-bases the mesh so its bottom sits at z=0 --
        so the source OBJ renders FIVE TIMES too large;
      * the builder hull is drawn squashed (shape_name Builder_Chassis_Squashed_Z), a
        factor of 0.110 in z, so the source OBJ renders as a full-height M113.

    Per axis rather than uniform: that reproduces a deliberate one-axis squash exactly,
    which a uniform fit cannot. The cost is that a mesh which was ROTATED before its
    bounds were taken (the rocks) gets a little internal distortion while still filling
    precisely the right box -- invisible on a lumpy rock, and the box is what the physics
    saw.
    """
    lo, hi = mesh.bounds[0::2], mesh.bounds[1::2]
    scale = [1.0, 1.0, 1.0]
    offset = [0.0, 0.0, 0.0]
    for i in range(3):
        raw = hi[i] - lo[i]
        want = amax[i] - amin[i]
        if raw > 1e-9 and want > 1e-9:
            scale[i] = want / raw
        # Align by the box, not by the origin: the rocks were re-based on load, so their
        # source origin is nowhere near the origin Chrono drew them about.
        offset[i] = amin[i] - lo[i] * scale[i]
    m = np.diag([scale[0], scale[1], scale[2], 1.0])
    m[:3, 3] = offset
    return mesh.transform(m, inplace=False)


def shape_signature(shape, boxes_only):
    """Everything about a shape that determines its geometry, body pose excluded."""
    def r(v, n=6):
        return tuple(round(float(x), n) for x in v) if v else None

    return (
        bool(boxes_only), shape.get("type"), shape.get("file"),
        r(shape.get("scale")), r(shape.get("size")),
        round(float(shape.get("radius", 0.0)), 6), round(float(shape.get("height", 0.0)), 6),
        r(shape.get("aabb_min")), r(shape.get("aabb_max")),
        r(shape.get("pos")), r(shape.get("rot")),
    )


def shape_geometry(shape, cache, boxes_only):
    """PolyData for one visual shape, already placed in its body's frame.

    Primitives are built from the dimensions the recorder wrote -- box "size", cylinder
    "radius"/"height" -- not from a bounding box. run16 carries 92 cylinders and they were
    every one of them drawn as a 20 cm cube before this.
    """
    return cache.placed(shape_signature(shape, boxes_only),
                        lambda: _build_shape(shape, cache, boxes_only))


def _build_shape(shape, cache, boxes_only):
    import pyvista as pv

    kind = shape.get("type")
    amin = shape.get("aabb_min")
    amax = shape.get("aabb_max")
    mesh = None

    if kind == "trimesh" and not boxes_only:
        mesh = cache.get(shape.get("file", ""), shape.get("scale", [1, 1, 1]))
        if mesh is not None:
            mesh = mesh.copy(deep=False)
            if amin and amax:
                mesh = fit_to_aabb(mesh, amin, amax)

    if mesh is None:
        if kind == "box" and shape.get("size"):
            sx, sy, sz = (max(1e-4, float(v)) for v in shape["size"])
            mesh = pv.Box(bounds=(-sx / 2, sx / 2, -sy / 2, sy / 2, -sz / 2, sz / 2))
        elif kind == "sphere" and shape.get("radius"):
            mesh = pv.Sphere(radius=float(shape["radius"]), theta_resolution=16,
                             phi_resolution=16)
        elif kind in ("cylinder", "capsule") and shape.get("radius"):
            # Chrono's cylinder and capsule are both centred on the body origin with their
            # axis along local z, which is also pyvista's default direction argument.
            r = float(shape["radius"])
            h = max(1e-4, float(shape.get("height", 2.0 * r)))
            if kind == "cylinder":
                mesh = pv.Cylinder(radius=r, height=h, direction=(0, 0, 1), resolution=20)
            else:
                mesh = pv.Capsule(radius=r, cylinder_length=h, direction=(0, 0, 1))
        elif amin and amax:
            mesh = pv.Box(bounds=(amin[0], amax[0], amin[1], amax[1], amin[2], amax[2]))
        else:
            mesh = pv.Box(bounds=(-0.1, 0.1, -0.1, 0.1, -0.1, 0.1))

    rot = quat_matrix(shape.get("rot", [1, 0, 0, 0]))
    pos = np.array(shape.get("pos", [0, 0, 0]), dtype=float)
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = pos
    if not np.allclose(m, np.eye(4)):
        mesh = mesh.transform(m, inplace=False)
    return mesh


def terrain_mesh(meta, prop, decimate, keep_radius):
    """The terrain, rebuilt from the run's own heightmap and fitted to the patch Chrono drew.

    RigidTerrain builds its patch mesh in memory, so the manifest names no file for it --
    but it does record the patch's bounds, and those are ground truth. Fitting to them
    beats reconstructing the grey mapping: run16's patch spans z=-13.82..12.65 while its
    metadata declares a [-25, 25] height range, because the mapping runs over the image's
    OWN grey range, not the declared one. Guess that wrong and the whole landscape is
    scaled and everything on it appears to float or sink.
    """
    import pyvista as pv

    terr = (meta or {}).get("terrain") or {}
    name = terr.get("heightmap")
    if not name:
        return None, None
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = next((c for c in (name, os.path.join(here, "data", name)) if os.path.exists(c)), None)
    if path is None:
        print(f"  ! heightmap {name} not found, terrain skipped", file=sys.stderr)
        return None, None

    img = pv.read(path)
    arr = img.active_scalars
    if arr is None:
        return None, None
    arr = np.asarray(arr)
    if arr.ndim > 1:  # RGB bitmap: channels are equal for greyscale, any one will do
        arr = arr[:, 0]
    nx, ny = img.dimensions[0], img.dimensions[1]
    grey = arr.reshape((ny, nx)).astype(np.float64)

    shape = ((prop or {}).get("shapes") or [{}])[0]
    amin, amax = shape.get("aabb_min"), shape.get("aabb_max")
    if amin and amax:
        x0, x1, y0, y1 = amin[0], amax[0], amin[1], amax[1]
        span = grey.max() - grey.min()
        z = (amin[2] + (amax[2] - amin[2]) * (grey - grey.min()) / span if span > 0
             else np.full_like(grey, amin[2]))
    else:
        length = float(terr.get("length", nx))
        width = float(terr.get("width", ny))
        x0, x1, y0, y1 = -length / 2, length / 2, -width / 2, width / 2
        lo, hi = float(terr.get("min_height", 0.0)), float(terr.get("max_height", 1.0))
        z = lo + (hi - lo) * (grey / (65535.0 if grey.max() > 255.0 else 255.0))

    step = max(1, int(decimate))
    z = z[::step, ::step]
    rows, cols = z.shape
    x = np.linspace(x0, x1, cols)
    y = np.linspace(y0, y1, rows)
    # Optional crop, off by default. The full patch is 1024 m across against a 37 m site,
    # but the collectors drive out to 200 m on the harvest lanes -- crop to the site and
    # they leave the ground and appear to fly. It costs nothing to keep: the heightmap is
    # 256x256, so the whole patch is 65k points. (Cropping was originally here to stop the
    # patch bounds dragging the camera out; the explicit camera positions in main() do
    # that job, so it is no longer the default.)
    if keep_radius > 0:
        cx, cy = np.abs(x) <= keep_radius, np.abs(y) <= keep_radius
        if cx.any() and cy.any():
            x, y, z = x[cx], y[cy], z[np.ix_(cy, cx)]
    xx, yy = np.meshgrid(x, y)
    colour = shape.get("color")
    # Fallback matches main.cpp's ground->SetColor(0.55, 0.55, 0.52) -- an SCM run records
    # no patch, so there is no colour to read, and the previous khaki guess made the Moon
    # look like desert.
    return pv.StructuredGrid(xx, yy, z), (tuple(colour) if colour else (0.55, 0.55, 0.52))


SCM_HEADER = struct.Struct("<8sIIdd7dii")
SCM_FRAME = struct.Struct("<IdI")
SCM_NODE = struct.Struct("<iif")
SCM_FILE_MAGIC = b"AMDUWSCM"
SCM_FRAME_MAGIC = 0x4D435353


NODE_DT = np.dtype([("i", "<i4"), ("j", "<i4"), ("z", "<f4")])
assert NODE_DT.itemsize == SCM_NODE.size


def scm_index(path, chunk=16_000_000):
    """Header, per-frame offsets, and the node SET this rank ever deformed -- one pass,
    holding neither the file nor its rows.

    The original read_scm read the whole file, upcast every (i4, i4, f4) row to
    (i8, i8, f8), and handed scm_sources a list it then np.unique'd across a concatenation
    of every frame's i and j. On rank 12 of a 1500 s 15-robot run -- 8.0 GB of node
    records, 670 M rows -- that peaks near 43 GB for that ONE rank of fifteen, which is
    why a full-site replay could not load on a 64 GB box.

    Uniques are folded in bounded chunks instead, so the transient never exceeds `chunk`
    keys (128 MB at the default) and only the offsets and the node set outlive the pass.
    """
    with open(path, "rb") as f:
        head = f.read(SCM_HEADER.size)
        if len(head) < SCM_HEADER.size:
            raise ValueError(f"{path}: shorter than its header")
        fields = SCM_HEADER.unpack(head)
        magic, rate, delta = fields[0], fields[3], fields[4]
        plane, nx, ny = fields[5:12], fields[12], fields[13]
        if magic != SCM_FILE_MAGIC:
            raise ValueError(f"{path}: bad magic {magic!r}")
        offsets, times, counts = [], [], []
        running = np.empty(0, dtype=np.int64)
        pending, buffered = [], 0
        while True:
            fh = f.read(SCM_FRAME.size)
            if len(fh) < SCM_FRAME.size:
                break
            fmagic, t, count = SCM_FRAME.unpack(fh)
            if fmagic != SCM_FRAME_MAGIC:
                raise ValueError(f"{path}: lost frame sync at offset "
                                 f"{f.tell() - SCM_FRAME.size}")
            need = NODE_DT.itemsize * count
            here = f.tell()
            data = f.read(need)
            if len(data) < need:
                break  # truncated final sample, as read_scm did
            offsets.append(here)
            times.append(t)
            counts.append(count)
            if count:
                a = np.frombuffer(data, dtype=NODE_DT, count=count)
                pending.append(pack_nodes(a["i"], a["j"]))
                buffered += count
                if buffered >= chunk:
                    running = np.unique(np.concatenate([running] + pending))
                    pending, buffered = [], 0
        if pending:
            running = np.unique(np.concatenate([running] + pending))
    return (delta, plane, nx, ny, rate,
            np.array(offsets, dtype=np.int64), np.array(times, dtype=np.float64),
            np.array(counts, dtype=np.int64), running)


class ScmStream:
    """Forward-only reader over one rank_*_scm.bin, compressing as it is read.

    This carries what compress_frames used to precompute for the whole run. The recorder
    republishes every touched node in a keyframe every 5 s -- its recovery mechanism, and
    56.6% of all rows on a measured 16-rank run, not one of which changes a height. A row
    is worth drawing iff it differs from the last height written to that node, which is
    one comparison per row and keeps nothing but `last`.

    Rewinding is reset(). apply_scm already replays from t=0 on a backward seek, so the
    semantics are unchanged; it now re-reads from disk instead of re-walking a list.
    """

    def __init__(self, path, offsets, times, counts, ukey):
        self.path = path
        self.offsets, self.times, self.counts, self.ukey = offsets, times, counts, ukey
        self.f = open(path, "rb")
        self.cursor = 0
        self.last = np.full(len(ukey), np.nan, dtype=np.float32)

    def close(self):
        try:
            self.f.close()
        except OSError:
            pass

    def reset(self):
        self.cursor = 0
        self.last[:] = np.nan

    def __len__(self):
        return len(self.offsets)

    def has_next(self):
        return self.cursor < len(self.offsets)

    def peek_time(self):
        return float(self.times[self.cursor])

    def next(self):
        k = self.cursor
        self.cursor += 1
        t, count = float(self.times[k]), int(self.counts[k])
        if not count:
            return t, np.empty(0, dtype=np.int64), np.empty(0, dtype=np.float32)
        self.f.seek(int(self.offsets[k]))
        raw = self.f.read(NODE_DT.itemsize * count)
        if len(raw) < NODE_DT.itemsize * count:
            return t, np.empty(0, dtype=np.int64), np.empty(0, dtype=np.float32)
        a = np.frombuffer(raw, dtype=NODE_DT, count=count)
        nid = np.searchsorted(self.ukey, pack_nodes(a["i"], a["j"]))
        z = a["z"]
        changed = self.last[nid] != z
        self.last[nid] = z
        return t, nid[changed], z[changed]


# One body record on disk is RECORD = "<I7f" -- packed, 32 bytes. Reading a frame as a
# structured array instead of a Python loop is most of why this file exists:
# read_trajectory.read_frame unpacks one body at a time into a dict, which is 224
# iterations per frame per rank, or 151 million of them on a 15-rank 45 000-frame run.
FRAME_REC = np.dtype([("idx", "<u4"), ("pose", "<f4", 7)])
assert FRAME_REC.itemsize == 32


def read_frame_np(f, offset):
    """(time, structured array of (idx, pose)) for the frame at `offset`, or None.

    Same bytes as read_trajectory.read_frame, one np.frombuffer instead of a Python loop.
    A truncated tail returns None rather than raising, matching the original.
    """
    f.seek(offset)
    fh = f.read(FRAME_HEADER.size)
    if len(fh) < FRAME_HEADER.size:
        return None
    fmagic, t, count = FRAME_HEADER.unpack(fh)
    if fmagic != FRAME_MAGIC:
        return None
    buf = f.read(FRAME_REC.itemsize * count)
    if len(buf) < FRAME_REC.itemsize * count:
        return None
    return t, np.frombuffer(buf, dtype=FRAME_REC, count=count)


class PoseStream:
    """poses[slot] -> (bodies, 7), read from disk on demand rather than preloaded.

    The original materialised (frames, bodies, 7) float32: 4.1 GB for 45 000 frames of a
    15-rank run. Playback only ever draws the slot it is on, and index_frames already
    hands us a byte offset per frame, so the array never has to exist. A small ring of
    recent slots keeps stepping and scrubbing off re-reading the same frame.

    Bodies are matched through `scatter`, a dense recorded-index -> row lookup built once
    per rank, so applying a frame is a vectorised scatter rather than a dict probe per
    body. Indexing is (slot) or (slot, body) -- both call sites in main() still work.
    """

    def __init__(self, per_rank, targets, n_bodies, cache=24):
        self.targets, self.n_bodies, self.cache_size = targets, n_bodies, max(2, cache)
        self.reads = 0
        self._cache, self._order = {}, []
        self.parts = []
        for _rank, rec, index, _rate, keep, slot0 in per_rank:
            want = np.array([o["index"] for o in keep], dtype=np.int64)
            scatter = np.full((int(want.max()) + 1) if len(want) else 1, -1, dtype=np.int32)
            if len(want):
                scatter[want] = np.arange(len(keep), dtype=np.int32)
            self.parts.append({
                "f": open(rec.frames_path, "rb"),
                "offsets": np.array([o for o, _t in index], dtype=np.int64),
                "stamps": np.array([t for _o, t in index], dtype=np.float64),
                "scatter": scatter, "slot0": slot0,
            })

    def close(self):
        for part in self.parts:
            try:
                part["f"].close()
            except OSError:
                pass

    def _build(self, slot):
        want = float(self.targets[slot])
        row = np.full((self.n_bodies, 7), np.nan, dtype=np.float32)
        for part in self.parts:
            stamps = part["stamps"]
            # Ranks are matched by TIME, not frame number: a rank that starts a step late
            # would otherwise draw its machines out of step with everyone else's.
            i = int(np.searchsorted(stamps, want))
            if i >= len(stamps):
                i = len(stamps) - 1
            elif i > 0 and (want - stamps[i - 1]) < (stamps[i] - want):
                i -= 1
            got = read_frame_np(part["f"], int(part["offsets"][i]))
            self.reads += 1
            if got is None:
                continue
            _t, arr = got
            idx = arr["idx"].astype(np.int64)
            keep_mask = idx < len(part["scatter"])
            if not keep_mask.all():
                idx, arr = idx[keep_mask], arr[keep_mask]
            col = part["scatter"][idx]
            live = col >= 0
            if live.any():
                row[part["slot0"] + col[live]] = arr["pose"][live]
        return row

    def __len__(self):
        return len(self.targets)

    def __getitem__(self, key):
        if isinstance(key, tuple):
            rest = key[1:]
            return self[key[0]][rest if len(rest) > 1 else rest[0]]
        slot = int(key)
        row = self._cache.get(slot)
        if row is None:
            row = self._build(slot)
            self._cache[slot] = row
            self._order.append(slot)
            if len(self._order) > self.cache_size:
                self._cache.pop(self._order.pop(0), None)
        return row


def load(directory, ranks, t_from, t_to, fps, keep_all, max_frames,
         hull_lift=HULL_LIFT_Z):
    """(meta, bodies, poses, times). poses is (frames, bodies, 7) = position + quaternion."""
    meta0 = {}
    bodies = []
    per_rank = []
    for rank in ranks:
        rec = Recording(directory, rank)
        lift_hull(rec.objects, hull_lift)
        if not meta0:
            meta0 = rec.meta
        try:
            index, rate = index_frames(rec.frames_path)
        except (OSError, ValueError) as exc:
            print(f"rank {rank}: {exc}", file=sys.stderr)
            continue
        if not index:
            continue
        keep = [o for o in rec.objects if keep_all or not RUNNING_GEAR.search(o["part"])]
        per_rank.append((rank, rec, index, rate, keep, len(bodies)))
        bodies.extend(keep)
    if not per_rank:
        raise SystemExit("nothing readable in that directory")

    # Reference timeline from the rank that stops first, so no rank runs out mid-playback.
    ref = min(per_rank, key=lambda p: p[2][-1][1])
    rate = ref[3] or 60.0
    t0 = max(ref[2][0][1], t_from if t_from is not None else -math.inf)
    t1 = min(ref[2][-1][1], t_to if t_to is not None else math.inf)
    if not t1 > t0:
        raise SystemExit(f"empty time window: {t0}..{t1}")
    # fps <= 0 means "whatever the recording holds", i.e. no resampling at all.
    if fps <= 0.0:
        fps = rate or 60.0
    want = int((t1 - t0) * max(0.1, fps)) + 1
    n = max(2, min(max_frames, want))
    if want > n:
        # Poses no longer sit in memory, so --max-frames is now only a cap on TIMELINE
        # RESOLUTION: how many distinct instants playback can land on. Raising it costs
        # eight bytes a frame instead of 22 kB, so pass the full count freely.
        print(f"  ! {want} frames at {fps:g}/sim-s over {t1 - t0:.1f} s of run, but "
              f"--max-frames is {max_frames} (a cap on timeline resolution; poses stream "
              f"from disk); resampled to {n}, i.e. {n / max(1e-9, t1 - t0):.2f}/sim-s. Pass "
              f"--max-frames {want} for every recorded frame, or narrow it with --from/--to.",
              file=sys.stderr)
    targets = np.linspace(t0, t1, n)

    poses = PoseStream(per_rank, targets, len(bodies))
    return meta0, bodies, poses, targets, rate


def static_props(directory):
    """Scenery from static_props.jsonl: the patch, the three rings, the pad, the wall rocks."""
    path = os.path.join(directory, "static_props.jsonl")
    out = []
    if not os.path.exists(path):
        return out
    import json

    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def height_sampler(grid):
    """Bilinear sampler over the rebuilt terrain grid, for the rut patches' base heights.

    Nearest-neighbour would step in 4 m blocks, and the ruts being drawn are centimetres
    deep -- the base would then contribute more error than the signal it is measuring.
    """
    gx, gy, gz = grid.x[0, :, 0], grid.y[:, 0, 0], grid.z[:, :, 0]

    def sample(x, y):
        fx = np.clip(np.searchsorted(gx, x) - 1, 0, len(gx) - 2)
        fy = np.clip(np.searchsorted(gy, y) - 1, 0, len(gy) - 2)
        tx = (x - gx[fx]) / (gx[fx + 1] - gx[fx])
        ty = (y - gy[fy]) / (gy[fy + 1] - gy[fy])
        return (gz[fy, fx] * (1 - tx) * (1 - ty) + gz[fy, fx + 1] * tx * (1 - ty)
                + gz[fy + 1, fx] * (1 - tx) * ty + gz[fy + 1, fx + 1] * tx * ty)

    return sample


def rut_colormap(ground):
    """Colours for the sinkage scalar: undisturbed ground at 0, darkening into the ruts.

    The first stop MUST be the terrain's own colour. A stock colormap puts its low end at
    some bright value, which paints the whole patch -- most of which is undisturbed -- as a
    slab in a colour the ground is not, and the ruts stop being what your eye finds.
    """
    if not isinstance(ground, str):
        r, g, b = (int(255 * max(0.0, min(1.0, c))) for c in tuple(ground)[:3])
        ground = f"#{r:02x}{g:02x}{b:02x}"
    return [ground, "#7d776b", "#655e51", "#4c4539", "#332d24", "#1f1a14"]


NODE_SHIFT = 20
NODE_BIAS = 1 << (NODE_SHIFT - 1)


def pack_nodes(i, j):
    """One int64 key per (i, j), sorting by i then j. |j| must stay under 2^19, which a
    0.02 m grid reaches at 10 km -- three times the whole 1024 m patch."""
    return ((np.asarray(i, dtype=np.int64) << NODE_SHIFT)
            + (np.asarray(j, dtype=np.int64) + NODE_BIAS))


def unpack_nodes(key):
    return key >> NODE_SHIFT, (key & ((1 << NODE_SHIFT) - 1)) - NODE_BIAS


def scm_sources(directory, ranks):
    """Per rank: the node SET it ever deformed, and a lazy stream over its frames.

    One bounded pass per file (scm_index), then playback pulls frames as it needs them.
    The node set is what every later stage tiles and counts from, so it is still resolved
    up front -- it is small (4.2 M nodes on the worst rank measured, 33 MB) where the rows
    that produce it are not (670 M).
    """
    out = []
    for rank in ranks:
        path = os.path.join(directory, f"rank_{rank}_scm.bin")
        if not os.path.exists(path):
            continue
        try:
            delta, plane, _nx, _ny, rate, offs, times, counts, ukey = scm_index(path)
        except (OSError, ValueError) as exc:
            print(f"  ! {os.path.basename(path)}: {exc}", file=sys.stderr)
            continue
        if not len(offs) or not len(ukey):
            continue
        if not (abs(plane[0]) < 1e-9 and abs(plane[1]) < 1e-9 and abs(plane[2]) < 1e-9
                and abs(plane[3] - 1.0) < 1e-9):
            print(f"  ! {os.path.basename(path)}: patch plane is not the identity; rut "
                  f"heights are drawn as world z anyway", file=sys.stderr)
        ui, uj = unpack_nodes(ukey)
        out.append({"rank": rank, "delta": delta, "rate": rate,
                    "stream": ScmStream(path, offs, times, counts, ukey),
                    "ui": ui, "uj": uj, "nodes": int(len(ui))})
    return out


# compress_frames() lived here. Its two jobs -- dropping keyframe rows that change no
# height, and resolving a node to its index in the rank's own set -- are now done per
# frame inside ScmStream.next(), so neither the rows nor the compressed result is ever
# held for the whole run. See ScmStream for the measurements that justified it.


def deformed_cells(sources, gx, gy):
    """The terrain cells any rank ever deformed, as sorted keys cx * len(gy) + cy.

    THIS, and not a per-rank bounding box, is what the rut layers cover, because a rank's
    work is not shaped anything like its bounding box. Each rank harvests a line of rocks
    running radially out to 170 m, so its ruts are one long DIAGONAL lane -- and the
    axis-aligned box around a diagonal lane is mostly not the lane. Measured over a 16-rank
    run: 897 terrain cells carry deformation, and the sixteen bounding boxes around them
    span 7019, a 7.8x overdraw whose boxes overlapped each other across 2768 cell-pairs.

    That overlap was not merely wasteful. Every box was filled with an opaque coarse
    surface, so sixteen of them stacked on top of one another, and whichever drew last hid
    the fine rut tiles of every rank beneath it. Only the ranks whose lanes happened to
    fall in uncontested ground showed ruts at all; the rest picked them up part way through
    a run, at the moment their collector drove far enough out to leave the pile-up around
    the site. Cells are shared by more than one rank in 21 places out of 897, so covering
    the cells instead of the boxes removes the occlusion rather than just reducing it.
    """
    ny = len(gy)
    keys = []
    for src in sources:
        d = src["delta"]
        cx = np.clip(np.searchsorted(gx, src["ui"] * d) - 1, 0, len(gx) - 2)
        cy = np.clip(np.searchsorted(gy, src["uj"] * d) - 1, 0, ny - 2)
        keys.append(np.unique(cx.astype(np.int64) * ny + cy))
    return np.unique(np.concatenate(keys)) if keys else np.empty(0, dtype=np.int64)


def blank_terrain_cells(grid, cells, gy):
    """Hide the terrain cells the rut layers replace, so no two surfaces coincide.

    Biasing one surface over the other -- polygon offset, a millimetre lift -- only hides
    the depth-buffer tie; it leaves two lots of geometry and shading in the same place, and
    at site distances that reads as a rectangular slab hovering over the ground, which is
    exactly what it looked like. Removing the covered cells leaves one surface everywhere.
    """
    if not len(cells):
        return 0
    rows, cols = grid.dimensions[0], grid.dimensions[1]
    cx, cy = cells // len(gy), cells % len(gy)
    mask = np.zeros((rows - 1) * (cols - 1), dtype=bool)
    mask[cx * (rows - 1) + cy] = True
    grid.hide_cells(mask, inplace=True)
    return int(mask.sum())


def tile_lines(a, b, step, tol):
    """Grid lines across the terrain cell [a, b]: both edges, and every global tile
    boundary strictly inside them.

    The two grids are INCOMMENSURATE -- the terrain pitch is 1024/255 = 4.0157 m and tiles
    are 0.32 m -- so a cell edge almost never falls on a tile line. Carrying the edge as a
    line of its own is what makes the seam with the surrounding terrain exact instead of
    off by up to half a tile, and it also puts every quad wholly inside one terrain cell
    and wholly inside one tile, so a quad's centre decides both without a tie. `tol` drops
    an interior line that lands within one node of an edge, which would otherwise leave a
    sliver too thin to give its points distinct nearest nodes.
    """
    k = np.arange(int(math.ceil(a / step)), int(math.floor(b / step)) + 1)
    inner = k * step
    inner = inner[(inner > a + tol) & (inner < b - tol)]
    return np.concatenate(([a], inner, [b]))


def build_rut_layers(pl, sources, cells, gx, gy, sample_base, ground, clim,
                     coarse_m=0.16, fine_m=0.04, budget=2_000_000):
    """Two meshes for the whole scene: a coarse surface filling the cut terrain cells, and
    a fine overlay on exactly the tiles that were ever deformed.

    WHY TWO LAYERS. One decimated grid over the deformed region has to pay for the region,
    and a region is mostly undisturbed: measured over a 16-rank run, 4.13 M deformed nodes
    sat inside 897 terrain cells of 100 M nodes, about 4%. Spending a fixed point budget on
    all of it gives the ruts the coarsest stride the region will fit -- 0.36 m, at which a
    one-metre rut is 2.8 points across and reads as a hairline. So the budget goes where
    the deformation is: the ground is tiled at `coarse_m`, and a tile that any node ever
    deformed is cut out of the coarse layer and re-drawn at `fine_m`. Cost then scales with
    rut AREA rather than region area, and every rank gets the same resolution regardless of
    how far it wandered.

    Tile size is what decides how tightly the fine layer hugs a rut, so a SMALLER coarse
    cell costs FEWER fine points, not more: over the same run, coarse 0.32 m needs 2.49 M
    fine points at 0.04 m where coarse 1.0 m needs 4.26 M.

    WHY ONE MESH EACH, rather than a pair per rank. The tile grid is global -- indexed from
    node 0, not from any one rank's box -- so a tile names the same square of ground to
    every rank, and one coarse layer can know which squares the fine layer has taken,
    whoever deformed them. Two surfaces can then never occupy one place, which is what went
    wrong when each rank filled its own bounding box (see deformed_cells). It also drops
    the scene from 32 actors to 2.
    """
    if not sources or not len(cells):
        return [], None
    import pyvista as pv

    delta = sources[0]["delta"]
    T = max(1, int(round(coarse_m / delta)))  # tile edge, in nodes
    step = T * delta

    # Global tile grid. floor division, so a tile spans the same nodes either side of zero.
    tkeys = np.unique(np.concatenate(
        [pack_nodes(np.floor_divide(s["ui"], T), np.floor_divide(s["uj"], T))
         for s in sources]))

    # Fine resolution is capped by the budget, so --rut-nodes still means what it did: the
    # most points this is allowed to spend. Coarsen the OVERLAY, never the tiling -- the
    # tiling is what keeps the cost proportional to rut area.
    for _ in range(8):
        fs = max(1, int(round(fine_m / delta)))
        while T % fs:
            fs -= 1
        if (T // fs + 1) ** 2 * len(tkeys) <= budget:
            break
        fine_m *= 2.0
    n = T // fs           # fine cells along a tile edge
    P = (n + 1) * (n + 1)  # fine points per tile

    # ---- fine overlay: one patch per deformed tile -------------------------------------
    ti, tj = unpack_nodes(tkeys)
    M = len(tkeys)
    off = np.arange(0, T + 1, fs)
    NI = (ti * T)[:, None, None] + off[None, :, None]
    NJ = (tj * T)[:, None, None] + off[None, None, :]
    px = np.broadcast_to(NI * delta, (M, n + 1, n + 1)).astype(np.float64).ravel()
    py = np.broadcast_to(NJ * delta, (M, n + 1, n + 1)).astype(np.float64).ravel()
    # float32 throughout the rut meshes. VTK's mapper builds its vertex buffer in float, so
    # double points are converted on every rebuild -- an extra pass over 26.7 M values, on
    # top of storing 214 MB where 107 MB would do. At the 200 m these coordinates reach,
    # float32 resolves 0.024 mm, against ruts measured in centimetres.
    fbase = sample_base(px, py).astype(np.float32)
    idx = np.arange(P).reshape(n + 1, n + 1)
    quad = np.stack([idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:], idx[:-1, 1:]],
                    axis=-1).reshape(-1, 4)
    allq = (quad[None, :, :] + (np.arange(M) * P)[:, None, None]).reshape(-1, 4)
    # Faces in the CONSTRUCTOR, not assigned afterwards. pv.PolyData(points) with no cells
    # builds one VERTEX cell per point, and assigning .faces later leaves them there -- so
    # the overlay drew 8.9 M point glyphs on top of its own quads. That is the speckle of
    # 2 cm dots over a flat tile, and it cost half a second a frame to draw. There is no
    # legitimate reason for this mesh to carry vertex cells.
    fine = pv.PolyData(np.column_stack([px, py, fbase]).astype(np.float32),
                       np.column_stack([np.full(len(allq), 4), allq]).ravel())
    fine.verts = np.empty(0, dtype=np.int64)  # belt and braces across pyvista versions
    fine["sinkage"] = np.zeros(fine.n_points, dtype=np.float32)

    def fine_point(a, b, T=T, fs=fs, n=n, P=P, tkeys=tkeys):
        ta, tb = np.floor_divide(a, T), np.floor_divide(b, T)
        slot = np.searchsorted(tkeys, pack_nodes(ta, tb))
        u = np.clip((a - ta * T + fs // 2) // fs, 0, n)
        v = np.clip((b - tb * T + fs // 2) // fs, 0, n)
        return slot * P + u * (n + 1) + v

    # A transposed or off-by-one map smears every rut across its own tile, which is not
    # something to notice by eye. Probe it against nodes that are known to be in the set.
    pi, pj = sources[0]["ui"][:2048], sources[0]["uj"][:2048]
    probe = fine_point(pi, pj)
    dx = np.abs(fine.points[probe, 0] - pi * delta)
    dy = np.abs(fine.points[probe, 1] - pj * delta)
    tol = 0.5 * fs * delta + 1e-6
    if dx.max() > tol or dy.max() > tol:
        k = int(np.argmax(np.maximum(dx, dy)))
        raise AssertionError(
            f"fine rut map is wrong: node ({pi[k] * delta}, {pj[k] * delta}) landed at "
            f"({fine.points[probe[k], 0]}, {fine.points[probe[k], 1]})")

    # ---- coarse layer: the cut terrain cells, minus the tiles the fine layer took -------
    ny = len(gy)
    xy, faces, base = [], [], 0
    for key in cells.tolist():
        cx, cy = key // ny, key % ny
        x = tile_lines(float(gx[cx]), float(gx[cx + 1]), step, delta)
        y = tile_lines(float(gy[cy]), float(gy[cy + 1]), step, delta)
        xx, yy = np.meshgrid(x, y, indexing="ij")
        ta = np.floor(0.5 * (x[:-1] + x[1:]) / step).astype(np.int64)
        tb = np.floor(0.5 * (y[:-1] + y[1:]) / step).astype(np.int64)
        k = pack_nodes(ta[:, None], tb[None, :])
        pos = np.clip(np.searchsorted(tkeys, k), 0, M - 1)
        taken = tkeys[pos] == k
        g = np.arange(len(x) * len(y)).reshape(len(x), len(y)) + base
        q = np.stack([g[:-1, :-1], g[1:, :-1], g[1:, 1:], g[:-1, 1:]], axis=-1).reshape(-1, 4)
        faces.append(q[~taken.ravel()])
        xy.append(np.column_stack([xx.ravel(), yy.ravel()]))
        base += len(x) * len(y)

    # Adjacent cells share their common edge, so weld the duplicated points: two points in
    # one place would each need the deformation written to them, and only one of them would
    # get it.
    allxy = np.concatenate(xy)
    _, first, inv = np.unique(np.rint(allxy / 1e-6).astype(np.int64), axis=0,
                              return_index=True, return_inverse=True)
    cxy = allxy[first]
    cbase = sample_base(cxy[:, 0], cxy[:, 1]).astype(np.float32)
    cq = inv[np.concatenate(faces)]
    coarse = pv.PolyData(np.column_stack([cxy, cbase]).astype(np.float32),
                         np.column_stack([np.full(len(cq), 4), cq]).ravel())
    coarse.verts = np.empty(0, dtype=np.int64)
    coarse["sinkage"] = np.zeros(coarse.n_points, dtype=np.float32)

    # Coarse points take the height of their NEAREST node. Most sit on a tile line and so
    # on a node exactly; the ones on a terrain cell edge cannot, the grids being
    # incommensurate, and rounding puts them within half a node -- a centimetre -- of the
    # right height. Sorted once here so a frame's nodes can be matched by searchsorted.
    cnode = pack_nodes(np.rint(cxy[:, 0] / delta).astype(np.int64),
                       np.rint(cxy[:, 1] / delta).astype(np.int64))
    corder = np.argsort(cnode, kind="stable")
    csorted = cnode[corder]

    def coarse_point(a, b, csorted=csorted, corder=corder):
        pos = np.clip(np.searchsorted(csorted, pack_nodes(a, b)), 0, len(csorted) - 1)
        return corder[pos], csorted[pos] == pack_nodes(a, b)

    cmap = rut_colormap(ground)
    for mesh in (coarse, fine):
        pl.add_mesh(mesh, scalars="sinkage", cmap=cmap,
                    clim=(0.0, max(1e-3, clim)), show_scalar_bar=False, specular=0.05)

    layers = {"coarse": coarse, "fine": fine, "coarse_base": cbase,
              "fine_base": fbase, "tiles": M, "stride": T, "fine_step": fs,
              "delta": delta, "cells": len(cells)}

    # Resolve each rank's node SET to mesh points and base heights, once. compress_frames
    # already reduced playback to node ids, so a frame costs one gather per array from
    # these -- and they are 254 k long, where reading fbase[] directly walked an 8.9 M
    # array whose 214 MB the renderer evicts from cache between every frame.
    patches = []
    for src in sources:
        fp = fine_point(src["ui"], src["uj"])
        cp, hit = coarse_point(src["ui"], src["uj"])
        patches.append({"rank": src["rank"], "stream": src["stream"],
                        "time": -1.0, "rate": src["rate"], "nodes": src["nodes"],
                        "fp": fp, "fb": fbase[fp], "cp": cp, "cb": cbase[cp], "hit": hit})
    return patches, layers


def apply_scm(patches, layers, t):
    """Accumulate deformation up to sim time `t` onto both layers.

    Forward playback applies only the samples crossed since the last call. A jump backwards
    rewinds to the pristine surfaces and replays -- cheap, because the whole run is only
    tens of thousands of node writes, and correct, which incremental-only cannot be when
    scrubbing. The two meshes are shared by every rank, so one rank rewinding rewinds all
    of them: the surfaces carry no record of which rank wrote what.

    Both layers are written, not just the fine one. The coarse layer owns the points along
    the boundary of every cut tile, and leaving those flat puts a step at the seam wherever
    a rut runs off the edge of its own tile.
    """
    if layers is None:
        return
    coarse, fine = layers["coarse"], layers["fine"]
    if any(t < p["time"] for p in patches):
        for mesh, base in ((coarse, layers["coarse_base"]), (fine, layers["fine_base"])):
            mesh.points[:, 2] = base
            mesh["sinkage"][:] = 0.0
        for p in patches:
            p["stream"].reset()
            p["time"] = -1.0
    moved = False
    for p in patches:
        stream = p["stream"]
        while stream.has_next() and stream.peek_time() <= t:
            _ft, nid, zz = stream.next()
            if not len(nid):
                continue
            # Deepest last: several nodes share a point wherever a layer is coarser than
            # the node pitch, and numpy's advanced assignment keeps the last write, so
            # sorting by height descending leaves the deepest node owning the point. A rut
            # beside undisturbed ground then stays a rut instead of being filled back in.
            order = np.argsort(-zz, kind="stable")
            nid, zz = nid[order], zz[order]
            fi = p["fp"][nid]
            fine.points[:, 2][fi] = zz
            fine["sinkage"][fi] = p["fb"][nid] - zz
            h = p["hit"][nid]
            if h.any():
                cn, cz = nid[h], zz[h]
                ci = p["cp"][cn]
                coarse.points[:, 2][ci] = cz
                coarse["sinkage"][ci] = p["cb"][cn] - cz
            moved = True
        p["time"] = t
    if moved:
        coarse.Modified()
        fine.Modified()


def build_scene(pl, bodies, cache, boxes_only, meta, terrain_decimate, scenery,
                terrain_radius, directory):
    """(actors, terrain grid, terrain colour). Actors are index-aligned with `bodies`.

    The colour comes back because the rut patches need it as the low end of their sinkage
    ramp, and rebuilding the terrain just to read it costs a second heightmap decode.
    """
    import pyvista as pv

    def merged(obj):
        pieces = [shape_geometry(s, cache, boxes_only) for s in obj.get("shapes", [])]
        pieces = [p for p in pieces if p is not None and p.n_points]
        if not pieces:
            lo, hi = body_bounds(obj)
            pieces = [pv.Box(bounds=(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))]
        return pieces[0] if len(pieces) == 1 else pieces[0].merge(pieces[1:])

    actors = []
    for obj in bodies:
        # The manifest carries each shape's colour, so the playblast looks like the run
        # rather than like a debug view. Group colour only when a body has none.
        colour = next((tuple(s["color"]) for s in obj.get("shapes", []) if s.get("color")), None)
        # NO name= here. Passing one makes pyvista remove_actor() first, which scans the
        # whole actor collection by name on every add -- O(n^2), and with 3336 bodies that
        # was 11.2 million VTK collection lookups and three minutes of scene build. The
        # actor list below is the handle we actually use.
        actor = pl.add_mesh(merged(obj), color=colour or group_color(obj), smooth_shading=True,
                            specular=0.25)
        actor.SetVisibility(False)  # until a pose is applied; rocks appear mid-run
        actors.append(actor)

    if not scenery:
        return actors, None, None

    # The rings are not circles: each is 180 little boxes laid ON the terrain, following
    # its height, and the pad and the decorative wall rocks are recorded the same way.
    # Drawing them from the record is the only way they land where the run had them --
    # a synthetic flat circle at a guessed height is wrong by metres on a hillside.
    props = static_props(directory)
    drew_terrain = False  # becomes the terrain grid once one is drawn
    ground = None
    for prop in props:
        shapes = prop.get("shapes", [])
        is_patch = len(shapes) == 1 and shapes[0].get("type") == "trimesh" and not shapes[0].get("file")
        if is_patch:
            if not terrain_decimate:
                continue
            terr, colour = terrain_mesh(meta, prop, terrain_decimate, terrain_radius)
            if terr is not None:
                pl.add_mesh(terr, color=colour, smooth_shading=True, specular=0.05)
                drew_terrain, ground = terr, colour
            continue
        colour = next((tuple(s["color"]) for s in shapes if s.get("color")), "#94a3b8")
        mesh = merged(prop)
        m = np.eye(4)
        m[:3, :3] = quat_matrix(prop.get("first_rot", [1, 0, 0, 0]))
        m[:3, 3] = prop.get("first_pos", [0, 0, 0])
        if not np.allclose(m, np.eye(4)):
            mesh = mesh.transform(m, inplace=False)
        pl.add_mesh(mesh, color=colour, smooth_shading=True)

    # An SCM run records NO patch prop: the deformable terrain is not a static visual at
    # the moment ExcludeExisting() runs. Its height field is still the same image, so the
    # surface is rebuilt from the metadata instead -- with the caveat that this is the
    # terrain as it STARTED. SCM deforms, and ruts do not show here.
    if terrain_decimate and drew_terrain is False:
        terr, colour = terrain_mesh(meta, None, terrain_decimate, terrain_radius)
        if terr is not None:
            pl.add_mesh(terr, color=colour, smooth_shading=True, specular=0.05)
            drew_terrain, ground = terr, colour
    return actors, (drew_terrain if drew_terrain is not False else None), ground


def frame_matrices(actors):
    """One vtkMatrix4x4 per actor, attached once and thereafter written in place.

    `actor.user_matrix = m` looks harmless and is the single most expensive thing this
    script did. PyVista's property setter builds a fresh vtkMatrix4x4 and marshals the
    sixteen values across on every assignment: measured at 95 us per actor, so 313 ms per
    frame for 3283 bodies -- against 15 ms to actually RENDER that same scene, ruts and
    all. Playback was never GPU-bound; it was spending 95% of the frame in a setter.

    Attaching the matrix once and mutating it costs one DeepCopy of a flat 16-tuple.
    """
    import vtk

    mats = []
    for actor in actors:
        m = vtk.vtkMatrix4x4()
        actor.SetUserMatrix(m)
        mats.append(m)
    return mats


def apply_frame(actors, poses, slot, mats=None):
    """Push one frame onto the actors. Bodies that do not exist yet stay hidden.

    The quaternion -> rotation conversion is done for every body at once. Per-body it was
    5.9 ms a frame, which is small next to the setter above but free to remove: the same
    arithmetic vectorised over (N, 4) is 0.1 ms.
    """
    row = poses[slot]
    live = np.isfinite(row[:, 0])

    q = row[:, 3:7]
    n = np.sqrt((q * q).sum(axis=1))
    n[~np.isfinite(n) | (n < 1e-12)] = 1.0
    w, x, y, z = (q / n[:, None]).T
    r00 = 1 - 2 * (y * y + z * z); r01 = 2 * (x * y - w * z); r02 = 2 * (x * z + w * y)
    r10 = 2 * (x * y + w * z); r11 = 1 - 2 * (x * x + z * z); r12 = 2 * (y * z - w * x)
    r20 = 2 * (x * z - w * y); r21 = 2 * (y * z + w * x); r22 = 1 - 2 * (x * x + y * y)
    px, py, pz = row[:, 0], row[:, 1], row[:, 2]

    for i, actor in enumerate(actors):
        if not live[i]:
            if actor.GetVisibility():
                actor.SetVisibility(False)
            continue
        m = mats[i] if mats is not None else actor.GetUserMatrix()
        if m is None:  # no pre-attached matrix (older call site): fall back to the setter
            mm = np.eye(4)
            mm[:3, :3] = quat_matrix(row[i, 3:7])
            mm[:3, 3] = row[i, 0:3]
            actor.user_matrix = mm
        else:
            m.DeepCopy((r00[i], r01[i], r02[i], px[i],
                        r10[i], r11[i], r12[i], py[i],
                        r20[i], r21[i], r22[i], pz[i],
                        0.0, 0.0, 0.0, 1.0))
            actor.Modified()
        if not actor.GetVisibility():
            actor.SetVisibility(True)


def main():
    ap = argparse.ArgumentParser(prog="replay_run.py", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("directory")
    ap.add_argument("--rank", help="comma-separated ranks (default: all)")
    ap.add_argument("--fps", type=float, default=0.0,
                    help="playback frames per sim second; 0 (default) uses the recording's "
                         "own rate, so every recorded frame is played and nothing is "
                         "resampled -- at 60 Hz that is real time at 60 fps")
    ap.add_argument("--speed", type=float, default=1.0, help="sim seconds per wall second")
    ap.add_argument("--from", dest="t_from", type=float)
    ap.add_argument("--to", dest="t_to", type=float)
    ap.add_argument("--max-frames", type=int, default=3000,
                    help="cap on frames held in memory, NOT a frame rate (default 3000). "
                         "Every body's pose for every frame is resident, so a long "
                         "run over the cap is resampled onto fewer frames and plays "
                         "coarser than it was recorded; raise it to play every frame")
    ap.add_argument("--no-running-gear", action="store_true",
                    help="drop track shoes, wheels, sprockets, idlers and suspension "
                         "(1905 of 3472 bodies in a 15-rank run) for frame rate")
    ap.add_argument("--boxes", action="store_true", help="bounding boxes instead of meshes")
    ap.add_argument("--no-terrain", action="store_true")
    ap.add_argument("--terrain-decimate", type=int, default=1,
                    help="heightmap subsampling, higher is coarser (default 1, full "
                         "resolution: the map is only 256x256, so decimating flattens "
                         "exactly the relief the site cares about)")
    ap.add_argument("--no-scm", action="store_true",
                    help="ignore rank_*_scm.bin, i.e. draw the terrain undeformed")
    ap.add_argument("--scm-depth", type=float, default=0.08,
                    help="sinkage in metres that colours fully dark (default 0.08)")
    ap.add_argument("--rut-nodes", type=int, default=12_000_000,
                    help="cap on rut-overlay points; the overlay is coarsened until it "
                         "fits (default 12M)")
    ap.add_argument("--rut-fine", type=float, default=0.0,
                    help="rut overlay resolution in metres; 0 means the recording's own "
                         "SCM node pitch, i.e. no decimation of the deformation (default)")
    ap.add_argument("--rut-coarse", type=float, default=0.16,
                    help="tile size in metres for the surface AROUND the ruts. Also the "
                         "granularity at which the overlay hugs them, so a smaller tile "
                         "costs FEWER overlay points, not more (default 0.32)")
    ap.add_argument("--no-scenery", action="store_true",
                    help="skip the terrain, rings, pad and decorative wall rocks")
    ap.add_argument("--terrain-margin", type=float, default=0.0,
                    help="crop the terrain to this many metres outside the collector ring; "
                         "0 (default) keeps the whole patch, which is what the run had")
    ap.add_argument("--movie", help="render off-screen to this .mp4 and exit")
    ap.add_argument("--shot", help="write a single PNG and exit")
    ap.add_argument("--at", type=float, help="sim time for --shot (default: midpoint)")
    ap.add_argument("--window", default="1600x1000")
    ap.add_argument("--mesh-root", action="append", metavar="DIR",
                    help="extra directory to search for meshes whose recorded absolute "
                         "path does not exist here (repeatable)")
    ap.add_argument("--focus", help="frame the first body whose group/part contains this")
    ap.add_argument("--hull-lift", type=float, default=HULL_LIFT_Z,
                    help="raise the builder hull mesh by this many metres in the render "
                         "so the track shoes stop cutting through its deck; 0 disables "
                         f"(default {HULL_LIFT_Z:g})")
    ap.add_argument("--focus-dist", type=float, default=12.0,
                    help="metres of scene to frame around the focused body, so smaller is "
                         "closer (default 12, about one builder plus its arm)")
    args = ap.parse_args()

    import pyvista as pv

    ranks = ([int(r) for r in args.rank.split(",")] if args.rank
             else discover_ranks(args.directory))
    if not ranks:
        raise SystemExit(f"no rank_*_frames.bin in {args.directory}")

    meta, bodies, poses, times, rate = load(args.directory, ranks, args.t_from, args.t_to,
                                            args.fps, not args.no_running_gear, args.max_frames,
                                            args.hull_lift)
    print(f"ranks       {ranks}")
    print(f"bodies      {len(bodies)}"
          f"{' (running gear dropped)' if args.no_running_gear else ' incl. running gear'}")
    span = max(1e-9, times[-1] - times[0])
    played = (len(times) - 1) / span
    print(f"frames      {len(times)} at {played:.2f}/sim-s from {rate:g} Hz recorded, "
          f"t={times[0]:.2f}..{times[-1]:.2f} s"
          + ("  (1:1, no resampling)" if abs(played - rate) < 0.01 * rate else ""))

    off = bool(args.movie or args.shot)
    w, h = (int(v) for v in args.window.lower().split("x"))
    pl = pv.Plotter(off_screen=off, window_size=(w, h))
    pl.set_background("#0b1120", top="#1e293b")

    cache = MeshCache(local_roots(args.mesh_root))
    site_r = float(((meta or {}).get("site") or {}).get("collector_ring", 37.0))
    actors, terrain_grid, ground = build_scene(pl, bodies, cache, args.boxes, meta,
                         0 if args.no_terrain else args.terrain_decimate,
                         not args.no_scenery,
                         (site_r + args.terrain_margin) if args.terrain_margin > 0 else 0.0,
                         args.directory)
    frame_mats = frame_matrices(actors)
    if cache.misses:
        print(f"  ! {len(cache.misses)} mesh file(s) not found here, drawn as boxes "
              f"(try --mesh-root)", file=sys.stderr)
    print(f"meshes      {cache.loaded()} distinct loaded"
          + (f", {cache.relocated} re-rooted from the recording's own paths"
             if cache.relocated else ""))

    # Rut patches, on the same surface the terrain was rebuilt from so the base heights
    # they measure sinkage against are the terrain's own.
    patches, layers = [], None
    if not args.no_scm and terrain_grid is not None:
        sources = scm_sources(args.directory, ranks)
        if sources:
            gx, gy = terrain_grid.x[0, :, 0], terrain_grid.y[:, 0, 0]
            cells = deformed_cells(sources, gx, gy)
            # Layers FIRST, then cut the terrain only from under what got built. Cutting
            # first left a hole wherever a layer did not appear, and a hole in the ground
            # shows the background: the ruts read as dark blue rectangles with no relief
            # in them, which is not a subtle failure.
            patches, layers = build_rut_layers(pl, sources, cells, gx, gy,
                                               height_sampler(terrain_grid),
                                               ground or (0.55, 0.55, 0.52), args.scm_depth,
                                               args.rut_coarse,
                                               args.rut_fine or sources[0]["delta"],
                                               args.rut_nodes)
            if layers:
                cut = blank_terrain_cells(terrain_grid, cells, gy)
                step = layers["fine_step"] * layers["delta"]
                tile = layers["stride"] * layers["delta"]
                print(f"deformation {len(patches)} rank(s) from rank_*_scm.bin at "
                      f"{sources[0]['rate']:g} Hz, "
                      f"{sum(p['nodes'] for p in patches)} deformed nodes; overlay at "
                      f"{step:.3g} m on {layers['tiles']} tiles of {tile:.3g} m "
                      f"({layers['fine'].n_points} pts), surround "
                      f"{layers['coarse'].n_points} pts over {cut} terrain cell(s) cut "
                      f"out beneath them")
    if not layers and ((meta or {}).get("terrain") or {}).get("model") == "scm":
        print("deformation none recorded -- terrain drawn as it started, no ruts")

    site = (meta or {}).get("site") or {}
    ring = float(site.get("collector_ring", 37.0))
    # NOT enable_lightkit(): VTK's light kit ships a deliberately WARM key light (warmth
    # 0.6 against 0.5 neutral), which tints the neutral grey regolith olive -- the terrain
    # came out looking like desert rather than Moon. One white sun plus a dim white fill is
    # both neutral and closer to the real thing, there being no atmosphere up there to
    # scatter light into the shadows. The sun sits mid-elevation rather than at the sim's
    # near-overhead angle because relief -- ruts, rock shadows, hull edges -- is what this
    # view exists to show, and an overhead sun flattens all of it.
    pl.remove_all_lights()
    sun = pv.Light(color="white", light_type="scene light", intensity=1.05)
    sun.set_direction_angle(38.0, -55.0)
    pl.add_light(sun)
    fill = pv.Light(color="white", light_type="camera light", intensity=0.22)
    pl.add_light(fill)

    def frame_radius(radius, focal, elev_deg=21.0, azim_deg=-135.0, fill=0.88):
        """Camera that fits a circle of `radius` about `focal` into the window.

        Derived from the site, NOT from the scene bounds. Those are two different things
        and conflating them is what made the view useless: the terrain patch is 1024 m
        across, so any bounds-based fit frames a kilometre of empty regolith and renders
        the machines as specks. Fixing THAT by cropping the terrain is the tail wagging
        the dog -- the run genuinely extends to 210 m and the ground should too.

        Solved rather than guessed: for a vertical field of view `va` and window aspect
        `a`, a sphere of radius r needs distance r/(fill*tan(va/2)) to fit vertically and
        r/(fill*a*tan(va/2)) to fit horizontally, so the larger of the two fits both.
        """
        va = math.radians(pl.camera.view_angle)
        w, h = pl.window_size
        aspect = max(1e-3, w / max(1, h))
        half = math.tan(va / 2.0)
        want = radius / max(0.05, fill)
        dist = max(want / half, want / (half * aspect))
        el, az = math.radians(elev_deg), math.radians(azim_deg)
        offset = (dist * math.cos(el) * math.cos(az),
                  dist * math.cos(el) * math.sin(az),
                  dist * math.sin(el))
        return [(focal[0] + offset[0], focal[1] + offset[1], focal[2] + offset[2]),
                tuple(focal), (0, 0, 1)]

    def set_cam(view):
        """Assign a camera and clip from the VIEW DISTANCE, not the scene bounds.

        reset_camera_clipping_range() spans everything in the scene, so with a kilometre
        of terrain loaded the near plane goes far out and the far plane enormous: the depth
        buffer then has no precision left for the machines and their surfaces z-fight. The
        range wants to bracket what is being looked at.
        """
        pl.camera_position = view
        pos, focal = np.array(view[0], dtype=float), np.array(view[1], dtype=float)
        dist = float(np.linalg.norm(pos - focal))
        pl.camera.clipping_range = (max(0.05, dist * 0.01), dist * 8.0 + 1000.0)

    site_view = frame_radius(ring, (0.0, 0.0, 3.0))
    set_cam(site_view)

    focus = -1
    if args.focus:
        want = args.focus.lower()
        focus = next((i for i, b in enumerate(bodies)
                      if want in f"{b['group']}/{b['part']}".lower()), -1)
        if focus < 0:
            print(f"  ! no body matching {args.focus!r}, keeping the site view", file=sys.stderr)
        else:
            print(f"focus       {bodies[focus]['group']}/{bodies[focus]['part']}")

    state = {"slot": 0, "playing": not off, "speed": args.speed, "follow": focus}
    # Wall-clock playback cursor: `carry` is the fraction of a frame owed from the last
    # tick, so a speed below the tick rate advances smoothly instead of stalling.
    clock = {"last": None, "carry": 0.0}
    hud_actor = pl.add_text("", position="upper_left", font_size=10, color="#e2e8f0",
                            name="hud")

    def hud():
        cam = "free" if state["follow"] < 0 else bodies[state["follow"]]["part"][:26]
        hud_actor.SetText(3, f"t={times[state['slot']]:8.2f} s   "
                             f"frame {state['slot'] + 1}/{len(times)}   "
                             f"{'>' if state['playing'] else '||'} {state['speed']:g}x   "
                             f"cam:{cam}")

    def show(slot):
        state["slot"] = max(0, min(len(times) - 1, slot))
        apply_frame(actors, poses, state["slot"], frame_mats)
        if layers:
            apply_scm(patches, layers, times[state["slot"]])
        if state["follow"] >= 0:
            p = poses[state["slot"], state["follow"]]
            if np.isfinite(p[0]):
                set_cam(frame_radius(max(1.0, args.focus_dist) * 0.5,
                                     (float(p[0]), float(p[1]), float(p[2]))))
        hud()

    show(0)

    if args.shot:
        want = args.at if args.at is not None else 0.5 * (times[0] + times[-1])
        show(int(np.argmin(np.abs(times - want))))
        pl.screenshot(args.shot)
        print(f"wrote       {args.shot} at t={times[state['slot']]:.2f} s")
        return

    if args.movie:
        # Frame rate from what is actually being played, times the speed multiplier: with
        # the default fps that makes a 60 Hz recording a 60 fps real-time video.
        pl.open_movie(args.movie, framerate=max(1, int(round(played * args.speed))))
        for slot in range(len(times)):
            show(slot)
            pl.write_frame()
        pl.close()
        print(f"wrote       {args.movie} ({len(times)} frames)")
        return

    def cycle_follow():
        pool = [i for i, b in enumerate(bodies)
                if b["group"] in ("collector", "builder") and "hassis" in b["part"]]
        if not pool:
            return
        state["follow"] = (pool[0] if state["follow"] not in pool
                           else pool[(pool.index(state["follow"]) + 1) % len(pool)])
        show(state["slot"])

    def free_cam():
        state["follow"] = -1
        set_cam(frame_radius(ring, (0.0, 0.0, 3.0)))
        hud()

    def top_cam():
        state["follow"] = -1
        set_cam(frame_radius(ring, (0.0, 0.0, 0.0), elev_deg=89.9, azim_deg=-90.0))
        hud()

    def set_speed(mult):
        state["speed"] = max(0.05, min(64.0, state["speed"] * mult))
        clock["carry"] = 0.0
        hud()

    def step(n):
        """Scrub by n frames, and stop playing -- stepping into a running clock is no use.

        The carry goes with it, or the fraction of a frame owed from the last tick lands
        on top of the step the moment playback resumes.
        """
        state["playing"] = False
        clock["carry"] = 0.0
        show(state["slot"] + n)

    def fly(right=0.0, ahead=0.0, up=0.0):
        """WASD-style camera move on IJKL: eye and focal point travel together.

        Movement is in the GROUND PLANE, not along the view vector. This camera looks down
        at the site from 21 degrees, so flying along the view direction drives into the
        dirt after a few presses; gliding over the terrain and changing height separately
        is what actually gets you across a 200 m work area. u/o do the height.

        The step scales with how far the camera is from what it is looking at, so one press
        covers the same fraction of the frame whether the whole site is in view or one
        machine is. Auto-repeat does the rest when a key is held.
        """
        pos = np.array(pl.camera.position, dtype=float)
        foc = np.array(pl.camera.focal_point, dtype=float)
        view = foc - pos
        ahead_v = np.array([view[0], view[1], 0.0])
        if np.linalg.norm(ahead_v) < 1e-6:
            # Straight down (the top view): there is no forward in the ground plane, so
            # take it from which way up is on screen instead.
            cam_up = np.array(pl.camera.up, dtype=float)
            ahead_v = np.array([cam_up[0], cam_up[1], 0.0])
        norm = np.linalg.norm(ahead_v)
        if norm < 1e-9:
            return
        ahead_v /= norm
        right_v = np.array([ahead_v[1], -ahead_v[0], 0.0])  # ahead x world-up
        delta = 0.08 * max(1.0, float(np.linalg.norm(view))) * (
            right * right_v + ahead * ahead_v + up * np.array([0.0, 0.0, 1.0]))
        # Any manual move drops follow, or show() snaps the camera back on the next frame.
        state["follow"] = -1
        set_cam([tuple(pos + delta), tuple(foc + delta), tuple(pl.camera.up)])
        hud()

    pl.add_key_event("space", lambda: (state.update(playing=not state["playing"]), hud()))
    # Timeline on the arrows, camera on IJKL. They used to share: `i` framed the iso view,
    # which is what `c` does, so nothing is lost by giving the letter to the camera.
    # Up/Down stay on pyvista's own zoom, which is the one game-camera control it ships.
    pl.add_key_event("Right", lambda: step(1))
    pl.add_key_event("Left", lambda: step(-1))
    pl.add_key_event("i", lambda: fly(ahead=1.0))
    pl.add_key_event("k", lambda: fly(ahead=-1.0))
    pl.add_key_event("j", lambda: fly(right=-1.0))
    pl.add_key_event("l", lambda: fly(right=1.0))
    pl.add_key_event("u", lambda: fly(up=1.0))
    pl.add_key_event("o", lambda: fly(up=-1.0))
    pl.add_key_event("bracketright", lambda: set_speed(2.0))
    pl.add_key_event("bracketleft", lambda: set_speed(0.5))
    pl.add_key_event("r", lambda: show(0))
    pl.add_key_event("f", cycle_follow)
    pl.add_key_event("c", free_cam)
    pl.add_key_event("t", top_cam)

    # Playback runs off the WALL CLOCK, not one frame per tick. A repeating VTK timer's
    # period is fixed when it is created, so "one frame per tick" pinned the rate to
    # whatever --speed said at startup and [ ] only moved the number in the HUD. Advancing
    # by the sim time that has actually elapsed means a speed change takes effect on the
    # next tick, and a scene too heavy to keep up drops frames rather than playing in slow
    # motion, which is what you want when judging whether motion looks right.
    def tick(_step):
        now = time.perf_counter()
        was, clock["last"] = clock["last"], now
        if not state["playing"] or was is None:
            return
        # Capped: a stall -- a drag, a resize, a slow first frame -- must not fast-forward
        # the run by however long it lasted.
        clock["carry"] += min(0.5, now - was) * played * state["speed"]
        step = int(clock["carry"])
        if step:
            clock["carry"] -= step
            show((state["slot"] + step) % len(times))

    # pyvista renders on EVERY timer event whether the callback moved anything or not, so
    # the tick rate is the render rate: hold it to the frame rate being played, between 10
    # and 60 Hz. Faster speeds then advance several frames per tick rather than rendering
    # more often than a screen can show.
    interval = int(round(1000.0 / min(60.0, max(10.0, played * max(1.0, state["speed"])))))
    pl.add_timer_event(max_steps=10 ** 9, duration=interval, callback=tick)
    print("keys        space play/pause | <- -> step a frame | [ ] speed\n"
          "            ijkl fly the camera (i/k forward-back, j/l strafe) | uo up/down | "
          "up/down zoom\n"
          "            t top | c free | f follow the next machine | r restart | q quit")
    pl.show()



# High-throughput rendering backend.  Kept in this file so replay_run.py is standalone.
import collections
import concurrent.futures

base = sys.modules[__name__]
_ORIGINAL_MAIN = main

_ORIGINAL_BUILD_SCENE = base.build_scene
_ORIGINAL_BUILD_RUT_LAYERS = base.build_rut_layers

# The recording names Polaris_tire.obj because Polaris_LuggedTire.json deliberately uses
# the smooth stock tyre for visualization even though SCM collides against the lugged
# mesh.  That visual is both misleading and expensive (7,988 triangles).  The project's
# metre-scaled lugged visual is 2,644 triangles and uses the same +Y spin axis.
_DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
_LUGGED_TIRE_VISUAL = os.path.join(
    _DATA_DIR, "vehicle", "LRV", "meshes", "LRVtire_red_m.obj")


def _render_shape(shape):
    """Apply experimental visual substitutions without changing recorded physics."""
    if (shape.get("type") == "trimesh"
            and os.path.basename(shape.get("file", "")) == "Polaris_tire.obj"):
        shape = dict(shape)
        shape["file"] = _LUGGED_TIRE_VISUAL
        # main.cpp hands this same metre-scaled asset directly to the VSG/Synchrono
        # visualization path.  Drop the stock Polaris_tire AABB so fit_to_aabb() does not
        # squash the VSG mesh back into the narrower smooth-tyre dimensions.
        shape.pop("aabb_min", None)
        shape.pop("aabb_max", None)
    return shape


def _colour(obj):
    return next((tuple(s["color"]) for s in obj.get("shapes", []) if s.get("color")),
                None) or base.group_color(obj)


def _prototype_key(obj, boxes_only):
    """Hashable description of geometry, excluding body pose and per-instance colour."""
    shapes = tuple(base.shape_signature(_render_shape(s), boxes_only)
                   for s in obj.get("shapes", []))
    # A body without recorded shapes falls back to body_bounds(), so its bounds are part
    # of the prototype.  Shaped bodies are fully described by shape_signature().
    fallback = None
    if not shapes:
        lo, hi = base.body_bounds(obj)
        fallback = tuple(round(float(x), 6) for x in (*lo, *hi))
    return shapes, fallback


def _merged_geometry(obj, cache, boxes_only):
    import pyvista as pv

    pieces = [base.shape_geometry(_render_shape(s), cache, boxes_only)
              for s in obj.get("shapes", [])]
    pieces = [p for p in pieces if p is not None and p.n_points]
    if not pieces:
        lo, hi = base.body_bounds(obj)
        pieces = [pv.Box(bounds=(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))]
    mesh = pieces[0] if len(pieces) == 1 else pieces[0].merge(pieces[1:])
    # add_mesh(..., smooth_shading=True) in the original builds point normals.  Do it
    # once per prototype here rather than once per body.
    try:
        return mesh.compute_normals(cell_normals=False, point_normals=True,
                                    split_vertices=False, inplace=False)
    except (TypeError, ValueError):
        return mesh


def build_instanced_bodies(pl, bodies, cache, boxes_only):
    """Return one multi-source glyph mapper containing every dynamic body.

    vtkGlyph3DMapper accepts a table of prototype meshes and a per-point source index.
    Keeping all recorded bodies in one point cloud reduces both Python update calls and
    renderer traversal.  VTK/OpenGL still group the instances by source internally.
    """
    import pyvista as pv
    import vtk

    prototypes = collections.OrderedDict()
    source_index = np.empty(len(bodies), dtype=np.int32)
    for body_index, obj in enumerate(bodies):
        key = _prototype_key(obj, boxes_only)
        entry = prototypes.get(key)
        if entry is None:
            entry = {"object": obj, "source_id": len(prototypes)}
            prototypes[key] = entry
        source_index[body_index] = entry["source_id"]

    count = len(bodies)
    cloud = pv.PolyData(np.zeros((count, 3), dtype=np.float32))
    cloud["orientation"] = np.tile(
        np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (count, 1))
    # vtkGlyph3DMapper's mask path requires vtkBitArray and is awkward to mutate from
    # numpy.  A zero scale is an equivalent, GPU-side visibility mask.
    cloud["instance_scale"] = np.zeros(count, dtype=np.float32)
    cloud["source_index"] = source_index
    cloud["instance_color"] = np.asarray(
        [(*pv.Color(_colour(obj)).int_rgb, 255) for obj in bodies], dtype=np.uint8)

    mapper = vtk.vtkGlyph3DMapper()
    mapper.SetInputData(cloud)
    sources = []
    for entry in prototypes.values():
        source = _merged_geometry(entry["object"], cache, boxes_only)
        sources.append(source)
        mapper.SetSourceData(entry["source_id"], source)
    mapper.SetSourceIndexArray("source_index")
    mapper.SourceIndexingOn()
    mapper.SetOrientationArray("orientation")
    mapper.SetOrientationModeToQuaternion()
    mapper.OrientOn()
    mapper.SetScaleArray("instance_scale")
    mapper.SetScaleModeToScaleByMagnitude()
    mapper.SetScaleFactor(1.0)
    mapper.ScalingOn()
    mapper.SetScalarModeToUsePointFieldData()
    mapper.SelectColorArray("instance_color")
    mapper.SetColorModeToDirectScalars()
    mapper.ScalarVisibilityOn()

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetInterpolationToPhong()
    actor.GetProperty().SetSpecular(0.25)
    pl.renderer.AddActor(actor)

    return {
        "cloud": cloud,
        "points": cloud.points,
        "orientations": cloud["orientation"],
        "scales": cloud["instance_scale"],
        "sources": sources,
        "mapper": mapper,
        "actor": actor,
        "prototype_count": len(prototypes),
    }


def build_scene(pl, bodies, cache, boxes_only, meta, terrain_decimate, scenery,
                terrain_radius, directory):
    """Instanced dynamic bodies plus the original, static terrain/scenery path."""
    instances = build_instanced_bodies(pl, bodies, cache, boxes_only)
    _unused, terrain, ground = _ORIGINAL_BUILD_SCENE(
        pl, [], cache, boxes_only, meta, terrain_decimate, scenery, terrain_radius,
        directory)
    print(f"instances   {len(bodies)} bodies, {instances['prototype_count']} prototypes, "
          "1 dynamic actor")
    replaced = sum(
        os.path.basename(shape.get("file", "")) == "Polaris_tire.obj"
        for obj in bodies for shape in obj.get("shapes", []))
    if replaced:
        print(f"tire visual {replaced} stock tires replaced by LRVtire_red_m.obj")
    return instances, terrain, ground


def frame_matrices(_instances):
    """Compatibility hook for base.main(); instances do not use per-actor matrices."""
    return None


def apply_frame(instances, poses, slot, _unused=None):
    """Upload one compact position/quaternion/scale array for all dynamic bodies."""
    row = poses[slot]
    live = np.isfinite(row[:, 0])

    q = row[:, 3:7].copy()
    norm = np.sqrt((q * q).sum(axis=1))
    bad = ~np.isfinite(norm) | (norm < 1e-12)
    norm[bad] = 1.0
    q /= norm[:, None]
    q[bad] = (1.0, 0.0, 0.0, 0.0)

    # Avoid placing NaNs in mapper bounds even for zero-scaled, not-yet-live rocks.
    points = instances["points"]
    points[:] = 0.0
    points[live] = row[live, :3]
    instances["orientations"][:] = q
    instances["scales"][:] = live

    cloud = instances["cloud"]
    cloud.GetPoints().GetData().Modified()
    cloud.GetPointData().GetArray("orientation").Modified()
    cloud.GetPointData().GetArray("instance_scale").Modified()


def _remove_dataset_actor(pl, dataset):
    """Remove the actor which the original rut builder attached for `dataset`."""
    removed = 0
    for actor in list(pl.renderer.actors.values()):
        mapper = actor.GetMapper() if hasattr(actor, "GetMapper") else None
        if mapper is None:
            continue
        try:
            # PyVista's DataSetMapper retains the exact Python dataset wrapper; GetInput()
            # creates another wrapper for the same VTK object and identity/equality both
            # fail even though the underlying pointer is shared.
            same = mapper.dataset is dataset
        except (AttributeError, TypeError):
            same = False
        if same:
            pl.remove_actor(actor, reset_camera=False, render=False)
            removed += 1
    return removed


def _chunk_mesh(pl, mesh, cmap, clim, target_faces):
    """Split a large dynamic quad mesh into independently uploadable render chunks."""
    import pyvista as pv

    faces = np.asarray(mesh.regular_faces)
    chunks = []
    for first in range(0, len(faces), target_faces):
        global_faces = faces[first:first + target_faces]
        global_ids, inverse = np.unique(global_faces, return_inverse=True)
        local_faces = inverse.reshape(global_faces.shape)
        local = pv.PolyData(
            np.asarray(mesh.points[global_ids], dtype=np.float32).copy(),
            np.column_stack((np.full(len(local_faces), 4, dtype=np.int64),
                             local_faces)).ravel())
        local.verts = np.empty(0, dtype=np.int64)
        local["sinkage"] = np.asarray(mesh["sinkage"][global_ids],
                                       dtype=np.float32).copy()
        actor = pl.add_mesh(local, scalars="sinkage", cmap=cmap, clim=clim,
                            show_scalar_bar=False, specular=0.05,
                            reset_camera=False, render=False)
        chunks.append({
            "global_ids": global_ids,
            "mesh": local,
            "points": local.points,
            "sinkage": local["sinkage"],
            "actor": actor,
        })
    return chunks


def build_rut_layers(pl, sources, cells, gx, gy, sample_base, ground, clim,
                     coarse_m=0.16, fine_m=0.04, budget=2_000_000):
    """Build the exact original rut surface, then spatially batch its GPU uploads.

    The original uses two enormous mutable meshes.  A single changed SCM node marks both
    datasets modified, making VTK rebuild buffers for 5.35 million points.  This backend
    keeps the same CPU-side topology and point mapping but renders face chunks, so an SCM
    sample invalidates only chunks containing points which actually changed.
    """
    patches, layers = _ORIGINAL_BUILD_RUT_LAYERS(
        pl, sources, cells, gx, gy, sample_base, ground, clim, coarse_m, fine_m,
        budget)
    if layers is None:
        return patches, layers

    coarse, fine = layers["coarse"], layers["fine"]
    removed = _remove_dataset_actor(pl, coarse) + _remove_dataset_actor(pl, fine)
    if removed != 2:
        print(f"  ! expected to replace 2 monolithic rut actors, found {removed}",
              file=sys.stderr)

    target_faces = max(4096, int(os.environ.get("REPLAY_SCM_CHUNK_FACES", "32768")))
    cmap = base.rut_colormap(ground)
    fine_chunks = _chunk_mesh(pl, fine, cmap, (0.0, max(1e-3, clim)), target_faces)
    coarse_chunks = _chunk_mesh(pl, coarse, cmap, (0.0, max(1e-3, clim)), target_faces)

    # CPU-authoritative dynamic fields.  The large original PolyData objects retain the
    # topology used to create chunks, but are no longer connected to a renderer.
    layers.update({
        "fine_z": np.asarray(fine.points[:, 2], dtype=np.float32).copy(),
        "fine_sinkage": np.asarray(fine["sinkage"], dtype=np.float32).copy(),
        "coarse_z": np.asarray(coarse.points[:, 2], dtype=np.float32).copy(),
        "coarse_sinkage": np.asarray(coarse["sinkage"], dtype=np.float32).copy(),
        "fine_chunks": fine_chunks,
        "coarse_chunks": coarse_chunks,
        "scm_pool": concurrent.futures.ThreadPoolExecutor(
            max_workers=max(1, min(len(patches), os.cpu_count() or 4)),
            thread_name_prefix="scm-decode"),
        "dirty_chunks": collections.OrderedDict(),
        "upload_chunks_per_frame": max(
            1, int(os.environ.get("REPLAY_SCM_UPLOAD_CHUNKS", "8"))),
    })
    print(f"scm chunks  {len(fine_chunks)} fine + {len(coarse_chunks)} coarse "
          f"(up to {target_faces} faces each)")
    return patches, layers


def _consume_patch_until(item):
    """Worker-thread side of SCM playback: read, filter, and sort one rank."""
    patch, t = item
    stream = patch["stream"]
    frames = []
    while stream.has_next() and stream.peek_time() <= t:
        _frame_time, node_ids, z = stream.next()
        if not len(node_ids):
            continue
        order = np.argsort(-z, kind="stable")
        frames.append((node_ids[order], z[order]))
    return frames


def _queue_chunks(layers, kind, changed, all_points=False):
    """Put touched chunks in a deduplicating FIFO; data is copied when they drain."""
    chunks = layers[f"{kind}_chunks"]
    changed = None if all_points else np.unique(np.concatenate(changed))
    for chunk_id, chunk in enumerate(chunks):
        global_ids = chunk["global_ids"]
        touched = changed is None
        if changed is not None:
            pos = np.searchsorted(global_ids, changed)
            valid = pos < len(global_ids)
            pos = pos[valid]
            selected = changed[valid]
            touched = bool(len(pos) and np.any(global_ids[pos] == selected))
        if touched:
            layers["dirty_chunks"].setdefault((kind, chunk_id), chunk)


def _drain_chunks(layers, limit=None):
    """Upload complete dirty chunks, each from the newest CPU-authoritative values."""
    queue = layers["dirty_chunks"]
    count = len(queue) if limit is None else min(limit, len(queue))
    for _ in range(count):
        (kind, _chunk_id), chunk = queue.popitem(last=False)
        global_ids = chunk["global_ids"]
        chunk["points"][:, 2] = layers[f"{kind}_z"][global_ids]
        chunk["sinkage"][:] = layers[f"{kind}_sinkage"][global_ids]
        chunk["mesh"].GetPoints().GetData().Modified()
        chunk["mesh"].GetPointData().GetArray("sinkage").Modified()
    return count


def apply_scm(patches, layers, t):
    """Parallel SCM decode plus dirty-chunk uploads, preserving original semantics."""
    if layers is None:
        return

    previous = max((patch["time"] for patch in patches), default=-1.0)
    first = previous < 0.0
    rewound = any(t < patch["time"] for patch in patches)
    if rewound:
        layers["coarse_z"][:] = layers["coarse_base"]
        layers["fine_z"][:] = layers["fine_base"]
        layers["coarse_sinkage"][:] = 0.0
        layers["fine_sinkage"][:] = 0.0
        for patch in patches:
            patch["stream"].reset()
            patch["time"] = -1.0

    due = [patch for patch in patches
           if patch["stream"].has_next() and patch["stream"].peek_time() <= t]
    decoded = layers["scm_pool"].map(
        _consume_patch_until, ((patch, t) for patch in due))

    fine_changed = []
    coarse_changed = []
    for patch, frames in zip(due, decoded):
        for node_ids, z in frames:
            fine_ids = patch["fp"][node_ids]
            layers["fine_z"][fine_ids] = z
            layers["fine_sinkage"][fine_ids] = patch["fb"][node_ids] - z
            fine_changed.append(fine_ids)

            hit = patch["hit"][node_ids]
            if hit.any():
                coarse_nodes = node_ids[hit]
                coarse_ids = patch["cp"][coarse_nodes]
                layers["coarse_z"][coarse_ids] = z[hit]
                layers["coarse_sinkage"][coarse_ids] = patch["cb"][coarse_nodes] - z[hit]
                coarse_changed.append(coarse_ids)

    for patch in patches:
        patch["time"] = t

    if rewound or fine_changed:
        _queue_chunks(layers, "fine", fine_changed, all_points=rewound)
    if rewound or coarse_changed:
        _queue_chunks(layers, "coarse", coarse_changed, all_points=rewound)

    # An interactive 30 Hz replay has three display frames between 10 Hz SCM samples.
    # Spread buffer uploads across those frames so one terrain sample cannot monopolise
    # the render thread.  Explicit seeks and offline output are flushed immediately for
    # exact single-frame/movie results.
    offline = "--shot" in sys.argv or "--movie" in sys.argv
    jumped = previous >= 0.0 and abs(t - previous) > 0.5
    limit = None if first or rewound or jumped or offline else layers["upload_chunks_per_frame"]
    _drain_chunks(layers, limit)


# Replace the baseline implementations in this standalone module before entering its
# original CLI.  Loading, controls, cameras, and movie output remain shared code above.
base.build_scene = build_scene
base.frame_matrices = frame_matrices
base.apply_frame = apply_frame
base.build_rut_layers = build_rut_layers
base.apply_scm = apply_scm


def _fast_main():
    """Run the standalone CLI without VTK's conflicting built-in `r` shortcut."""
    import pyvista as pv

    original_show = pv.Plotter.show

    def show_without_vtk_char_shortcuts(plotter, *args, **kwargs):
        # The replay handles its documented keyboard controls through PyVista's explicit
        # KeyPressEvent callbacks.  VTK's lower-level CharEvent handler also interprets
        # `r` as reset-camera-to-all-bounds; with the kilometre-wide terrain that fires
        # after show(0) and zooms far out.  Follow mode hid it by continuously reasserting
        # its own camera.  Removing the built-in character shortcuts leaves the replay's
        # callbacks (including r, c, movement, zoom, and q) intact.
        if plotter.iren is not None and plotter.iren.interactor is not None:
            plotter.iren.interactor.RemoveObservers("CharEvent")
        return original_show(plotter, *args, **kwargs)

    pv.Plotter.show = show_without_vtk_char_shortcuts
    try:
        _ORIGINAL_MAIN()
    finally:
        pv.Plotter.show = original_show


if __name__ == "__main__":
    _fast_main()
