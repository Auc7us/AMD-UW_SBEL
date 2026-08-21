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
of builders cost one hull mesh, not fifteen. --all-parts brings back the track shoes,
which are 1905 of the 3472 bodies in a 15-rank run and will cost you the frame rate.
"""

import argparse
import bisect
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_trajectory import Recording, discover_ranks, index_frames, read_frame  # noqa: E402

# Parts that are numerically dominant and add nothing to a previz: 63 track shoes per
# builder side is most of a recording, and no one judging whether the site works is
# looking at a road wheel. --all-parts brings them back.
NOISE = re.compile(
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


class MeshCache:
    """One PolyData per (file, scale). A 15-rank run reuses 28 meshes across 3472 bodies."""

    def __init__(self):
        self._cache = {}
        self.misses = set()

    def get(self, path, scale):
        key = (path, tuple(scale))
        if key in self._cache:
            return self._cache[key]
        import pyvista as pv

        mesh = None
        if path and os.path.exists(path):
            try:
                mesh = pv.read(path)
                if any(abs(s - 1.0) > 1e-9 for s in scale):
                    mesh = mesh.scale(scale, inplace=False)
            except Exception as exc:  # noqa: BLE001 - one bad asset must not kill the scene
                print(f"  ! {os.path.basename(path)}: {exc}", file=sys.stderr)
                mesh = None
        elif path:
            self.misses.add(path)
        self._cache[key] = mesh
        return mesh

    def loaded(self):
        return sum(1 for v in self._cache.values() if v is not None)


def shape_geometry(shape, cache, boxes_only):
    """PolyData for one visual shape, already placed in its body's frame."""
    import pyvista as pv

    amin = shape.get("aabb_min") or [-0.1] * 3
    amax = shape.get("aabb_max") or [0.1] * 3
    mesh = None
    if not boxes_only and shape.get("type") == "trimesh":
        mesh = cache.get(shape.get("file", ""), shape.get("scale", [1, 1, 1]))
    if mesh is None:
        # Primitive stand-in from the shape's own bounds. Also the --boxes path, and the
        # only thing available for cylinders and boxes, which carry no mesh file.
        mesh = pv.Box(bounds=(amin[0], amax[0], amin[1], amax[1], amin[2], amax[2]))
    else:
        mesh = mesh.copy(deep=False)

    rot = quat_matrix(shape.get("rot", [1, 0, 0, 0]))
    pos = np.array(shape.get("pos", [0, 0, 0]), dtype=float)
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = pos
    if not np.allclose(m, np.eye(4)):
        mesh = mesh.transform(m, inplace=False)
    return mesh


def terrain_mesh(meta, decimate, keep_radius):
    """Structured grid from the run's own heightmap, so height errors are visible.

    Chrono maps grey linearly onto [min_height, max_height] across a length x width patch
    centred on the origin. STB always loads 16-bit, so the normalisation is by the image's
    own dtype maximum rather than by 255 -- getting that wrong scales the whole landscape
    by 257 and everything appears to float.
    """
    import pyvista as pv

    terr = (meta or {}).get("terrain") or {}
    name = terr.get("heightmap")
    if not name:
        return None
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = None
    for candidate in (name, os.path.join(here, "data", name)):
        if os.path.exists(candidate):
            path = candidate
            break
    if path is None:
        print(f"  ! heightmap {name} not found, terrain skipped", file=sys.stderr)
        return None

    img = pv.read(path)
    arr = img.active_scalars
    if arr is None:
        return None
    arr = np.asarray(arr)
    if arr.ndim > 1:  # RGB bitmap: channels are equal for greyscale, any one will do
        arr = arr[:, 0]
    nx, ny = img.dimensions[0], img.dimensions[1]
    grid = arr.reshape((ny, nx)).astype(np.float64)
    peak = 65535.0 if grid.max() > 255.0 else 255.0
    lo = float(terr.get("min_height", 0.0))
    hi = float(terr.get("max_height", 1.0))
    z = lo + (hi - lo) * (grid / peak)

    step = max(1, int(decimate))
    z = z[::step, ::step]
    length = float(terr.get("length", nx))
    width = float(terr.get("width", ny))
    rows, cols = z.shape
    x = np.linspace(-length / 2.0, length / 2.0, cols)
    y = np.linspace(-width / 2.0, width / 2.0, rows)
    # Cropped to the site. The full patch is 1024 m across against a 37 m site, so keeping
    # it all costs frame rate for scenery nobody is looking at AND wrecks the framing:
    # every camera fit is computed over the scene bounds, so the machines end up specks.
    if keep_radius > 0:
        cx = np.abs(x) <= keep_radius
        cy = np.abs(y) <= keep_radius
        if cx.any() and cy.any():
            x, y, z = x[cx], y[cy], z[np.ix_(cy, cx)]
    xx, yy = np.meshgrid(x, y)
    return pv.StructuredGrid(xx, yy, z)


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
        keep = [o for o in rec.objects if keep_all or not NOISE.search(o["part"])]
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
    n = max(2, min(max_frames, int((t1 - t0) * max(0.1, fps)) + 1))
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


def build_scene(pl, bodies, cache, boxes_only, meta, terrain_decimate, show_rings,
                terrain_radius):
    """One actor per body, index-aligned with `bodies`."""
    import pyvista as pv

    actors = []
    for obj in bodies:
        shapes = obj.get("shapes", [])
        pieces = [shape_geometry(s, cache, boxes_only) for s in shapes]
        pieces = [p for p in pieces if p is not None and p.n_points]
        if not pieces:
            lo, hi = body_bounds(obj)
            pieces = [pv.Box(bounds=(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))]
        mesh = pieces[0] if len(pieces) == 1 else pieces[0].merge(pieces[1:])
        # The manifest carries each shape's colour, so the playblast looks like the run
        # rather than like a debug view. Group colour only when a body has none.
        colour = next((tuple(s["color"]) for s in shapes if s.get("color")), None)
        actor = pl.add_mesh(mesh, color=colour or group_color(obj), smooth_shading=True,
                            specular=0.25, name=f"b{len(actors)}")
        actor.SetVisibility(False)  # until a pose is applied; rocks appear mid-run
        actors.append(actor)

    if terrain_decimate:
        terr = terrain_mesh(meta, terrain_decimate, terrain_radius)
        if terr is not None:
            pl.add_mesh(terr, color="#8a8578", smooth_shading=True, specular=0.05,
                        name="terrain")

    if show_rings:
        site = (meta or {}).get("site") or {}
        ring_z = float(site.get("ring_z", 3.2))
        for key, colour in (("work_circle", "#eab308"), ("builder_orbit", "#22d3ee"),
                            ("collector_ring", "#22c55e")):
            r = site.get(key)
            if not r:
                continue
            ring = pv.Circle(radius=float(r), resolution=180).extract_all_edges()
            pl.add_mesh(ring.translate((0, 0, ring_z), inplace=False), color=colour,
                        line_width=2, name=f"ring_{key}")
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
    ap.add_argument("--fps", type=float, default=30.0, help="playback frames per sim second")
    ap.add_argument("--speed", type=float, default=1.0, help="sim seconds per wall second")
    ap.add_argument("--from", dest="t_from", type=float)
    ap.add_argument("--to", dest="t_to", type=float)
    ap.add_argument("--max-frames", type=int, default=3000)
    ap.add_argument("--all-parts", action="store_true", help="include track shoes and wheels")
    ap.add_argument("--boxes", action="store_true", help="bounding boxes instead of meshes")
    ap.add_argument("--no-terrain", action="store_true")
    ap.add_argument("--terrain-decimate", type=int, default=4,
                    help="heightmap subsampling, higher is coarser (default 4)")
    ap.add_argument("--no-rings", action="store_true")
    ap.add_argument("--terrain-margin", type=float, default=25.0,
                    help="metres of terrain kept outside the collector ring (default 25)")
    ap.add_argument("--movie", help="render off-screen to this .mp4 and exit")
    ap.add_argument("--shot", help="write a single PNG and exit")
    ap.add_argument("--at", type=float, help="sim time for --shot (default: midpoint)")
    ap.add_argument("--window", default="1600x1000")
    args = ap.parse_args()

    import pyvista as pv

    ranks = ([int(r) for r in args.rank.split(",")] if args.rank
             else discover_ranks(args.directory))
    if not ranks:
        raise SystemExit(f"no rank_*_frames.bin in {args.directory}")

    meta, bodies, poses, times, rate = load(args.directory, ranks, args.t_from, args.t_to,
                                            args.fps, args.all_parts, args.max_frames)
    print(f"ranks       {ranks}")
    print(f"bodies      {len(bodies)}"
          f"{'' if args.all_parts else ' (track shoes/wheels/suspension dropped)'}")
    print(f"frames      {len(times)} at {args.fps:g}/sim-s from {rate:g} Hz, "
          f"t={times[0]:.2f}..{times[-1]:.2f} s")

    off = bool(args.movie or args.shot)
    w, h = (int(v) for v in args.window.lower().split("x"))
    pl = pv.Plotter(off_screen=off, window_size=(w, h))
    pl.set_background("#0b1120", top="#1e293b")

    cache = MeshCache()
    site_r = float(((meta or {}).get("site") or {}).get("collector_ring", 37.0))
    actors = build_scene(pl, bodies, cache, args.boxes, meta,
                         0 if args.no_terrain else args.terrain_decimate,
                         not args.no_rings,
                         site_r + max(0.0, args.terrain_margin))
    if cache.misses:
        print(f"  ! {len(cache.misses)} mesh file(s) missing, drawn as boxes", file=sys.stderr)
    print(f"meshes      {cache.loaded()} distinct loaded")

    site = (meta or {}).get("site") or {}
    ring = float(site.get("collector_ring", 37.0))
    iso = [(ring * 1.55, -ring * 1.55, ring * 0.95), (0, 0, 3.0), (0, 0, 1)]
    pl.enable_lightkit()

    def set_cam(view):
        """Assign a camera and re-fit the clipping range.

        VTK computes near/far from the bounds it last saw, and the terrain patch is
        kilometres wide: leave the range alone and the machines fall outside it or z-fight.
        """
        pl.camera_position = view
        pl.reset_camera_clipping_range()

    set_cam(iso)

    state = {"slot": 0, "playing": not off, "speed": args.speed, "follow": -1}
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
        if state["follow"] >= 0:
            p = poses[state["slot"], state["follow"]]
            if np.isfinite(p[0]):
                set_cam([(float(p[0]) - 12, float(p[1]) - 12, float(p[2]) + 8),
                         (float(p[0]), float(p[1]), float(p[2])), (0, 0, 1)])
        hud()

    show(0)

    if args.shot:
        want = args.at if args.at is not None else 0.5 * (times[0] + times[-1])
        show(int(np.argmin(np.abs(times - want))))
        pl.screenshot(args.shot)
        print(f"wrote       {args.shot} at t={times[state['slot']]:.2f} s")
        return

    if args.movie:
        pl.open_movie(args.movie, framerate=max(1, int(round(args.fps * args.speed))))
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
        set_cam(iso)
        hud()

    def top_cam():
        state["follow"] = -1
        set_cam([(0, 0, ring * 2.6), (0, 0, 0), (0, 1, 0)])
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

    interval = max(10, int(1000.0 / max(1.0, args.fps * state["speed"])))
    pl.add_timer_event(max_steps=10 ** 9, duration=interval, callback=tick)
    print("keys        space play/pause | <- -> step | [ ] speed | t top | i iso | "
          "f follow | c free | r restart | q quit")
    pl.show()


if __name__ == "__main__":
    main()
