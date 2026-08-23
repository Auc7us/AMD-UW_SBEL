#!/usr/bin/env python3
"""Real-time 3D playblast of a --record_dir recording, with the run's own meshes.

A previz viewer: the actual OBJ assets the sim rendered, played back on the wall clock,
scrubbable, orbitable, and optionally written straight to a movie. No Blender, no import
step, no scene to maintain -- the recording's object manifest names every mesh file and
its local frame, so the scene builds itself from the run.

    replay_run.py <dir>                       interactive window, real time
    replay_run.py <dir> --speed 4             4x
    replay_run.py <dir> --rank 1,2 --to 240   one sector, first 4 minutes
    replay_run.py <dir> --boxes               bounding boxes only, for speed
    replay_run.py <dir> --movie run16.mp4     off-screen render to mp4
    replay_run.py <dir> --shot look.png --at 120

Keys: space play/pause, left/right step a frame, [ ] speed, t top view, i iso view,
      f follow the next machine, c free camera, r restart, q quit.

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

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_trajectory import Recording, discover_ranks, index_frames, read_frame  # noqa: E402

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
    return pv.StructuredGrid(xx, yy, z), (tuple(colour) if colour else "#8a8578")


SCM_HEADER = struct.Struct("<8sIIdd7dii")
SCM_FRAME = struct.Struct("<IdI")
SCM_NODE = struct.Struct("<iif")
SCM_FILE_MAGIC = b"AMDUWSCM"
SCM_FRAME_MAGIC = 0x4D435353


def read_scm(path):
    """Deformed SCM grid nodes: (delta, plane, nx, ny, [(time, i[], j[], z[])]).

    One frame per sample, carrying only the nodes modified since the last one, with
    ABSOLUTE heights -- so a consumer accumulates, and a dropped frame leaves the ground
    slightly stale rather than permanently wrong. Node (i,j) sits at (i*delta, j*delta) in
    the patch plane frame, which is what makes the indices meaningful.
    """
    buf = open(path, "rb").read()
    if len(buf) < SCM_HEADER.size:
        raise ValueError(f"{path}: shorter than its header")
    fields = SCM_HEADER.unpack_from(buf, 0)
    magic, _version, _rank, rate, delta = fields[0], fields[1], fields[2], fields[3], fields[4]
    plane, nx, ny = fields[5:12], fields[12], fields[13]
    if magic != SCM_FILE_MAGIC:
        raise ValueError(f"{path}: bad magic {magic!r}")
    off = SCM_HEADER.size
    frames = []
    while off + SCM_FRAME.size <= len(buf):
        fmagic, time, count = SCM_FRAME.unpack_from(buf, off)
        if fmagic != SCM_FRAME_MAGIC:
            raise ValueError(f"{path}: lost frame sync at offset {off}")
        off += SCM_FRAME.size
        need = SCM_NODE.size * count
        if off + need > len(buf):
            break  # truncated final sample
        chunk = np.frombuffer(buf, dtype=np.dtype([("i", "<i4"), ("j", "<i4"), ("z", "<f4")]),
                              count=count, offset=off)
        frames.append((time, chunk["i"].astype(np.int64), chunk["j"].astype(np.int64),
                       chunk["z"].astype(np.float64)))
        off += need
    return delta, plane, nx, ny, rate, frames


def load(directory, ranks, t_from, t_to, fps, keep_all, max_frames):
    """(meta, bodies, poses, times). poses is (frames, bodies, 7) = position + quaternion."""
    meta0 = {}
    bodies = []
    per_rank = []
    for rank in ranks:
        rec = Recording(directory, rank)
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
        print(f"  ! {want} frames wanted at {fps:g}/sim-s but --max-frames is {max_frames}; "
              f"playing {n}, i.e. {n / max(1e-9, t1 - t0):.2f}/sim-s", file=sys.stderr)
    targets = np.linspace(t0, t1, n)

    poses = np.full((n, len(bodies), 7), np.nan, dtype=np.float32)
    for _rank, rec, index, _rate, keep, slot0 in per_rank:
        stamps = [t for _o, t in index]
        with open(rec.frames_path, "rb") as f:
            for slot, want in enumerate(targets):
                # Ranks are matched by TIME, not frame number: a rank that starts a step
                # late would otherwise draw its machines out of step with everyone else's.
                i = bisect.bisect_left(stamps, want)
                if i >= len(stamps):
                    i = len(stamps) - 1
                elif i > 0 and (want - stamps[i - 1]) < (stamps[i] - want):
                    i -= 1
                got = read_frame(f, index[i][0])
                if got is None:
                    continue
                _t, frame = got
                for k, obj in enumerate(keep):
                    p = frame.get(obj["index"])
                    if p is not None:
                        poses[slot, slot0 + k, :] = p
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


def scm_patches(pl, directory, ranks, sample_base, ground, clim, lift, max_nodes=4_000_000):
    """One fine grid per rank covering the nodes that rank ever deformed.

    Separate from the terrain on purpose. The heightmap is 4.0157 m per vertex and SCM
    nodes are 0.1 m apart, so ruts are forty times finer than the ground mesh and cannot
    be shown on it. Each patch spans only the bounding box its rank actually touched --
    tens of thousands of points, not the 100 million a 0.1 m grid over the whole patch
    would need.

    Drawn with polygon-offset resolution so it never z-fights the terrain underneath: the
    two surfaces agree exactly wherever nothing has driven, which is most of the patch.
    """
    patches = []
    for rank in ranks:
        path = os.path.join(directory, f"rank_{rank}_scm.bin")
        if not os.path.exists(path):
            continue
        try:
            delta, plane, _nx, _ny, rate, frames = read_scm(path)
        except (OSError, ValueError) as exc:
            print(f"  ! {os.path.basename(path)}: {exc}", file=sys.stderr)
            continue
        if not frames:
            continue
        if not (abs(plane[0]) < 1e-9 and abs(plane[1]) < 1e-9 and abs(plane[2]) < 1e-9
                and abs(plane[3] - 1.0) < 1e-9):
            print(f"  ! {os.path.basename(path)}: patch plane is not the identity; rut "
                  f"heights are drawn as world z anyway", file=sys.stderr)

        all_i = np.concatenate([f[1] for f in frames])
        all_j = np.concatenate([f[2] for f in frames])
        i0, i1 = int(all_i.min()) - 2, int(all_i.max()) + 2
        j0, j1 = int(all_j.min()) - 2, int(all_j.max()) + 2
        cols, rows = i1 - i0 + 1, j1 - j0 + 1
        if rows * cols > max_nodes:
            print(f"  ! rank {rank} deformation spans {cols}x{rows} nodes, over the "
                  f"{max_nodes} cap; skipped", file=sys.stderr)
            continue

        import pyvista as pv

        x = (np.arange(i0, i1 + 1)) * delta
        y = (np.arange(j0, j1 + 1)) * delta
        xx, yy = np.meshgrid(x, y)
        base = sample_base(xx.ravel(), yy.ravel()).reshape(yy.shape)
        grid = pv.StructuredGrid(xx, yy, base + lift)
        grid["sinkage"] = np.zeros(base.size)
        actor = pl.add_mesh(grid, scalars="sinkage", cmap=rut_colormap(ground),
                            clim=(0.0, max(1e-3, clim)), show_scalar_bar=False,
                            smooth_shading=True, specular=0.05)
        # Coincident-surface handling, belt and braces. The patch and the terrain agree
        # exactly wherever nothing has driven, and a tie in the depth buffer speckles.
        # Polygon offset biases this surface towards the camera; the millimetre lift is
        # there because offset alone still speckled at site distances. Both shift the
        # patch as a whole, so rut DEPTH -- the quantity being read -- is untouched.
        try:
            actor.mapper.SetResolveCoincidentTopologyToPolygonOffset()
            actor.mapper.SetRelativeCoincidentTopologyPolygonOffsetParameters(0.0, -12.0)
        except AttributeError:
            pass
        patches.append({
            "rank": rank, "grid": grid, "base": base + lift, "true_base": base,
            "lift": lift, "i0": i0, "j0": j0,
            "shape": (rows, cols), "frames": frames, "cursor": 0, "time": -1.0,
            "rate": rate, "nodes": int(len(set(zip(all_i.tolist(), all_j.tolist())))),
        })
    return patches


def apply_scm(patches, t):
    """Accumulate deformation up to sim time `t` onto every rut patch.

    Forward playback applies only the samples crossed since the last call. A jump
    backwards rewinds to the pristine surface and replays -- cheap, because the whole run
    is only tens of thousands of node writes, and correct, which incremental-only cannot
    be when scrubbing.
    """
    for p in patches:
        if t < p["time"]:
            p["grid"].points[:, 2] = p["base"].ravel()
            p["grid"]["sinkage"][:] = 0.0
            p["cursor"] = 0
        rows, cols = p["shape"]
        z = p["grid"].points[:, 2]
        sink = p["grid"]["sinkage"]
        moved = False
        while p["cursor"] < len(p["frames"]) and p["frames"][p["cursor"]][0] <= t:
            _ft, ii, jj, zz = p["frames"][p["cursor"]]
            flat = (jj - p["j0"]) * cols + (ii - p["i0"])
            ok = (flat >= 0) & (flat < rows * cols)
            idx = flat[ok]
            z[idx] = zz[ok] + p["lift"]
            # Sinkage against the UNLIFTED surface, so the colour reads true depth.
            sink[idx] = p["true_base"].ravel()[idx] - zz[ok]
            p["cursor"] += 1
            moved = True
        p["time"] = t
        if moved:
            p["grid"].Modified()


def build_scene(pl, bodies, cache, boxes_only, meta, terrain_decimate, scenery,
                terrain_radius, directory):
    """One actor per body, index-aligned with `bodies`, plus the static scenery."""
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
        return actors

    # The rings are not circles: each is 180 little boxes laid ON the terrain, following
    # its height, and the pad and the decorative wall rocks are recorded the same way.
    # Drawing them from the record is the only way they land where the run had them --
    # a synthetic flat circle at a guessed height is wrong by metres on a hillside.
    props = static_props(directory)
    drew_terrain = False
    for prop in props:
        shapes = prop.get("shapes", [])
        is_patch = len(shapes) == 1 and shapes[0].get("type") == "trimesh" and not shapes[0].get("file")
        if is_patch:
            if not terrain_decimate:
                continue
            terr, colour = terrain_mesh(meta, prop, terrain_decimate, terrain_radius)
            if terr is not None:
                pl.add_mesh(terr, color=colour, smooth_shading=True, specular=0.05)
                drew_terrain = True
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
    if terrain_decimate and not drew_terrain:
        terr, colour = terrain_mesh(meta, None, terrain_decimate, terrain_radius)
        if terr is not None:
            pl.add_mesh(terr, color=colour, smooth_shading=True, specular=0.05)
    return actors


def apply_frame(actors, poses, slot):
    """Push one frame onto the actors. Bodies that do not exist yet stay hidden."""
    row = poses[slot]
    for i, actor in enumerate(actors):
        p = row[i]
        if not np.isfinite(p[0]):
            if actor.GetVisibility():
                actor.SetVisibility(False)
            continue
        m = np.eye(4)
        m[:3, :3] = quat_matrix(p[3:7])
        m[:3, 3] = p[0:3]
        actor.user_matrix = m
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
    ap.add_argument("--max-frames", type=int, default=3000)
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
    ap.add_argument("--scm-lift", type=float, default=0.004,
                    help="metres the rut patch is raised to beat depth-buffer ties with "
                         "the terrain; shifts the patch, not the rut depths (default 0.004)")
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
                                            args.fps, not args.no_running_gear, args.max_frames)
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
    actors = build_scene(pl, bodies, cache, args.boxes, meta,
                         0 if args.no_terrain else args.terrain_decimate,
                         not args.no_scenery,
                         (site_r + args.terrain_margin) if args.terrain_margin > 0 else 0.0,
                         args.directory)
    if cache.misses:
        print(f"  ! {len(cache.misses)} mesh file(s) not found here, drawn as boxes "
              f"(try --mesh-root)", file=sys.stderr)
    print(f"meshes      {cache.loaded()} distinct loaded"
          + (f", {cache.relocated} re-rooted from the recording's own paths"
             if cache.relocated else ""))

    # Rut patches, on the same surface the terrain was rebuilt from so the base heights
    # they measure sinkage against are the terrain's own.
    patches = []
    if not args.no_scm and not args.no_terrain:
        full, ground = terrain_mesh(meta, None, 1, 0.0)
        if full is not None:
            patches = scm_patches(pl, args.directory, ranks, height_sampler(full), ground,
                                  args.scm_depth, args.scm_lift)
    if patches:
        total = sum(p["nodes"] for p in patches)
        print(f"deformation {len(patches)} rut patch(es) from rank_*_scm.bin at "
              f"{patches[0]['rate']:g} Hz, {total} deformed nodes, "
              + " + ".join(f"{p['shape'][1]}x{p['shape'][0]}" for p in patches) + " grids")
    elif ((meta or {}).get("terrain") or {}).get("model") == "scm":
        print("deformation none recorded -- terrain drawn as it started, no ruts")

    site = (meta or {}).get("site") or {}
    ring = float(site.get("collector_ring", 37.0))
    pl.enable_lightkit()

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
        apply_frame(actors, poses, state["slot"])
        if patches:
            apply_scm(patches, times[state["slot"]])
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
        hud()

    pl.add_key_event("space", lambda: (state.update(playing=not state["playing"]), hud()))
    pl.add_key_event("Right", lambda: show(state["slot"] + 1))
    pl.add_key_event("Left", lambda: show(state["slot"] - 1))
    pl.add_key_event("bracketright", lambda: set_speed(2.0))
    pl.add_key_event("bracketleft", lambda: set_speed(0.5))
    pl.add_key_event("r", lambda: show(0))
    pl.add_key_event("f", cycle_follow)
    pl.add_key_event("c", free_cam)
    pl.add_key_event("i", free_cam)
    pl.add_key_event("t", top_cam)

    # The timer fires at the playback rate, so --speed is honoured by advancing sim time
    # per tick; a scene too heavy to keep up drops frames rather than playing in slow
    # motion, which is what you want when judging whether motion looks right.
    def tick(_step):
        if state["playing"]:
            show((state["slot"] + 1) % len(times))

    interval = max(10, int(1000.0 / max(1.0, played * state["speed"])))
    pl.add_timer_event(max_steps=10 ** 9, duration=interval, callback=tick)
    print("keys        space play/pause | <- -> step | [ ] speed | t top | i iso | "
          "f follow | c free | r restart | q quit")
    pl.show()


if __name__ == "__main__":
    main()
