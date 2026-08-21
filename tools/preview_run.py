#!/usr/bin/env python3
"""Preview a --record_dir pose recording in a browser, without Blender.

Writes ONE self-contained HTML file: plan view of the whole site with every body drawn
as its own bounding box, a scrubbable timeline, and an elevation panel. Open it and
watch the run. The point is to answer "does this look right, or is something going
weird" in seconds, on any machine, with nothing installed -- there is no matplotlib in
this container and no display either, so a rendered-to-file interactive page beats a
plotting library.

Boxes, not meshes, on purpose. A body's manifest carries the axis-aligned bounds of each
of its visual shapes, so the box is derived from the same numbers Blender would load and
needs no mesh files present, no scaling convention, and no import step. It is also the
check that catches the failures worth catching: a machine in the wrong place, a rock
sunk into the terrain or floating over it, a drop that lands outside the builder's
reach, an arm folded through its own hull. Use tools/blender_import.py when you want it
to look good; use this when you want to know if it is correct.

Usage:
    preview_run.py <recording_dir>                       -> <dir>/preview.html
    preview_run.py <dir> -o /tmp/run16.html --fps 4
    preview_run.py <dir> --rank 1,2,3 --to 120
    preview_run.py <dir> --all-parts                     keep track shoes etc.

The defaults drop the parts that only add weight -- track shoes, road wheels,
suspension arms, wheel spindles -- because 63 shoes per builder is 2/3 of the recording
and none of it tells you whether the site is working. --all-parts keeps everything.
"""

import argparse
import bisect
import json
import math
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_trajectory import HEADER, FRAME_HEADER, RECORD, FILE_MAGIC, FRAME_MAGIC  # noqa: E402
from read_trajectory import Recording, discover_ranks  # noqa: E402

# Parts that are structurally uninteresting from above and numerically dominant.
NOISE = re.compile(
    r"TrackShoe|RoadWheel|Suspension|Idler|Sprocket|DoubleWishbone|Rack-Pinion"
    r"|spindle|axleTube|finger|ballast",
    re.IGNORECASE,
)

# Group -> colour. Collector family cool, builder family warm, rocks by provenance, so a
# delivered rock is distinguishable from a seed rock at a glance -- that distinction is
# the whole point of watching the handoff.
COLORS = {
    "collector": "#3b82f6",
    "collector_trailer": "#22d3ee",
    "collector_arm": "#14b8a6",
    "builder": "#ef4444",
    "builder_arm": "#f472b6",
    "rock_seed": "#94a3b8",
    "rock_harvest": "#facc15",
    "world": "#475569",
}


def quat_rotate(q, v):
    """Rotate v by quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def body_bounds(obj):
    """Axis-aligned bounds of a body in ITS OWN frame, over all its visual shapes.

    A body is not necessarily one shape at its own origin, so each shape's AABB corners
    are pushed through that shape's local frame before the union is taken -- the same
    body_pose * shape_frame * shape_aabb chain read_trajectory.py --bbox validates.
    """
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for shape in obj.get("shapes", []):
        amin = shape.get("aabb_min")
        amax = shape.get("aabb_max")
        if not amin or not amax:
            continue
        scale = shape.get("scale", [1.0, 1.0, 1.0])
        spos = shape.get("pos", [0.0, 0.0, 0.0])
        srot = shape.get("rot", [1.0, 0.0, 0.0, 0.0])
        for cx in (amin[0], amax[0]):
            for cy in (amin[1], amax[1]):
                for cz in (amin[2], amax[2]):
                    p = (cx * scale[0], cy * scale[1], cz * scale[2])
                    p = quat_rotate(srot, p)
                    for i in range(3):
                        v = p[i] + spos[i]
                        lo[i] = min(lo[i], v)
                        hi[i] = max(hi[i], v)
    if not all(math.isfinite(v) for v in lo + hi):
        # No usable shape: give it a small marker box so it is still visible.
        return [-0.25, -0.25, -0.25], [0.25, 0.25, 0.25]
    return lo, hi


def group_color(obj):
    if obj["group"] == "rock":
        return COLORS["rock_seed" if obj["part"].startswith("seed_rock") else "rock_harvest"]
    return COLORS.get(obj["group"], "#a3a3a3")


def index_frames(path):
    """[(offset, time)] for every frame, built by walking headers and seeking past payloads.

    Frames are NOT fixed size -- a rank's body count grows during a run as rocks are
    created -- so the Nth frame cannot be computed from a stride, only found. Walking the
    headers reads 16 bytes per frame instead of the ~6.5 kB payload, which is what makes
    previewing an 8 GB recording take a second: this touches well under 1% of the file,
    and the payloads of the frames actually kept are the only ones ever read in full.

    A truncated tail (Ctrl-C leaves a partial frame) ends the index rather than raising.
    """
    out = []
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        head = f.read(HEADER.size)
        if len(head) < HEADER.size:
            return [], 0.0
        magic, _version, _rank, rate, _step = HEADER.unpack(head)
        if magic != FILE_MAGIC:
            raise ValueError(f"{path}: bad magic {magic!r}")
        offset = HEADER.size
        while offset + FRAME_HEADER.size <= size:
            f.seek(offset)
            fh = f.read(FRAME_HEADER.size)
            if len(fh) < FRAME_HEADER.size:
                break
            fmagic, time, count = FRAME_HEADER.unpack(fh)
            if fmagic != FRAME_MAGIC:
                raise ValueError(f"{path}: lost frame sync at offset {offset}")
            payload = RECORD.size * count
            if offset + FRAME_HEADER.size + payload > size:
                break  # truncated final frame
            out.append((offset, time))
            offset += FRAME_HEADER.size + payload
    return out, rate


def read_frame(f, offset):
    """{index: (px,py,pz,qw,qx,qy,qz)} for the frame at `offset`, or None."""
    f.seek(offset)
    fh = f.read(FRAME_HEADER.size)
    if len(fh) < FRAME_HEADER.size:
        return None
    fmagic, time, count = FRAME_HEADER.unpack(fh)
    if fmagic != FRAME_MAGIC:
        return None
    buf = f.read(RECORD.size * count)
    if len(buf) < RECORD.size * count:
        return None
    poses = {}
    for i in range(count):
        idx, px, py, pz, qw, qx, qy, qz = RECORD.unpack_from(buf, i * RECORD.size)
        poses[idx] = (px, py, pz, qw, qx, qy, qz)
    return time, poses


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def collect(directory, ranks, t_from, t_to, fps, keep_all, max_frames):
    """Decimate every rank onto one shared TIME line. Returns (meta, bodies, times, frames).

    Ranks are matched by time, not by frame number: they all record at the same rate but a
    rank that starts a step later would otherwise show its machines a frame out of step
    with everyone else's, which reads as a geometry error that is not there.
    """
    bodies = []
    per_rank = []
    meta0 = {}

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
            print(f"rank {rank}: no frames, skipped", file=sys.stderr)
            continue
        keep = []
        for obj in rec.objects:
            if not keep_all and NOISE.search(obj["part"]):
                continue
            lo, hi = body_bounds(obj)
            keep.append((obj, lo, hi))
        slot0 = len(bodies)
        for obj, lo, hi in keep:
            bodies.append(
                {
                    "n": f"{obj['group']}/{obj['part']}",
                    "g": obj["group"],
                    "r": rank,
                    # Footprint half-extents AND the body-frame centre offset: a box is not
                    # centred on the body origin (a chassis origin sits at an axle), and
                    # ignoring that draws every machine half a hull off its true place.
                    "cx": round(0.5 * (lo[0] + hi[0]), 3),
                    "cy": round(0.5 * (lo[1] + hi[1]), 3),
                    "hx": round(max(0.05, 0.5 * (hi[0] - lo[0])), 3),
                    "hy": round(max(0.05, 0.5 * (hi[1] - lo[1])), 3),
                    "c": group_color(obj),
                }
            )
        per_rank.append((rank, rec, index, rate, keep, slot0))

    if not per_rank:
        raise SystemExit("nothing readable in that directory")

    # Reference timeline from the rank that stops first, so no rank runs out mid-preview.
    ref = min(per_rank, key=lambda p: p[2][-1][1])
    rate = ref[3] or 60.0
    t_start = max(ref[2][0][1], t_from if t_from is not None else -math.inf)
    t_end = min(ref[2][-1][1], t_to if t_to is not None else math.inf)
    if not t_end > t_start:
        raise SystemExit(f"empty time window: t={t_start}..{t_end}")
    n = max(2, min(max_frames, int((t_end - t_start) * max(0.1, fps)) + 1))
    targets = [t_start + (t_end - t_start) * i / (n - 1) for i in range(n)]

    n_bodies = len(bodies)
    frames = [[None] * (4 * n_bodies) for _ in targets]
    times = [round(t, 3) for t in targets]

    for rank, rec, index, _rate, keep, slot0 in per_rank:
        stamps = [t for _off, t in index]
        with open(rec.frames_path, "rb") as f:
            for slot, want in enumerate(targets):
                # Nearest recorded frame to the wanted time.
                i = bisect.bisect_left(stamps, want)
                if i >= len(stamps):
                    i = len(stamps) - 1
                elif i > 0 and (want - stamps[i - 1]) < (stamps[i] - want):
                    i -= 1
                got = read_frame(f, index[i][0])
                if got is None:
                    continue
                _time, poses = got
                row = frames[slot]
                for k, (obj, _lo, _hi) in enumerate(keep):
                    pose = poses.get(obj["index"])
                    if pose is None:
                        continue
                    px, py, pz, qw, qx, qy, qz = pose
                    if not all(math.isfinite(v) for v in (px, py, pz, qw, qx, qy, qz)):
                        continue
                    base = 4 * (slot0 + k)
                    row[base] = round(px, 2)
                    row[base + 1] = round(py, 2)
                    row[base + 2] = round(pz, 2)
                    row[base + 3] = round(yaw_of((qw, qx, qy, qz)), 3)

    return meta0, bodies, times, frames, rate


HTML = r"""<title>__TITLE__</title>
<style>
  :root { --bg:#f8fafc; --fg:#0f172a; --panel:#ffffff; --line:#cbd5e1; --dim:#64748b; }
  :root:not([data-theme="light"]) { }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) { --bg:#0b1120; --fg:#e2e8f0; --panel:#111827; --line:#334155; --dim:#94a3b8; }
  }
  :root[data-theme="dark"] { --bg:#0b1120; --fg:#e2e8f0; --panel:#111827; --line:#334155; --dim:#94a3b8; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:13px/1.45 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }
  header { padding:10px 14px; border-bottom:1px solid var(--line); }
  h1 { margin:0 0 2px; font-size:15px; font-weight:650; }
  .sub { color:var(--dim); font-size:12px; }
  .wrap { display:flex; flex-wrap:wrap; gap:12px; padding:12px; align-items:flex-start; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:10px; }
  canvas { display:block; max-width:100%; border-radius:4px; }
  .ctl { display:flex; flex-wrap:wrap; gap:10px; align-items:center; padding:8px 14px;
         border-bottom:1px solid var(--line); position:sticky; top:0; background:var(--bg); z-index:5; }
  button, select { font:inherit; color:var(--fg); background:var(--panel);
                   border:1px solid var(--line); border-radius:6px; padding:3px 9px; cursor:pointer; }
  input[type=range] { width:340px; max-width:52vw; }
  .keys { display:flex; flex-wrap:wrap; gap:8px; margin-top:8px; }
  .keys label { display:flex; gap:5px; align-items:center; font-size:12px; white-space:nowrap; }
  .sw { width:11px; height:11px; border-radius:2px; display:inline-block; }
  #tip { position:fixed; pointer-events:none; background:var(--panel); border:1px solid var(--line);
         border-radius:6px; padding:4px 7px; font-size:12px; display:none; z-index:20; }
  code { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; }
  .mono { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:12px; color:var(--dim); }
</style>
<header>
  <h1>__TITLE__</h1>
  <div class="sub" id="sub"></div>
</header>
<div class="ctl">
  <button id="play">Play</button>
  <input type="range" id="scrub" min="0" value="0">
  <span class="mono" id="clock"></span>
  <label>speed <select id="speed">
    <option>0.25</option><option>0.5</option><option selected>1</option>
    <option>2</option><option>4</option><option>8</option></select></label>
  <label><input type="checkbox" id="trails" checked> trails</label>
  <label><input type="checkbox" id="labels"> labels</label>
  <label>view <select id="view">
    <option value="site" selected>site</option>
    <option value="all">everything</option></select></label>
  <label>zoom <input type="range" id="zoom" min="20" max="200" value="100" style="width:110px"></label>
</div>
<div class="wrap">
  <div class="card">
    <canvas id="plan" width="820" height="820"></canvas>
    <div class="keys" id="keys"></div>
  </div>
  <div class="card">
    <canvas id="elev" width="380" height="300"></canvas>
    <div class="sub" style="margin-top:6px">Height against distance from the site centre.
      Rings marked; a rock far off the terrain line is sunk or floating.</div>
    <canvas id="reach" width="380" height="300" style="margin-top:12px"></canvas>
    <div class="sub" style="margin-top:6px">Distance from each builder hull to the nearest
      rock. Below the shaded band the arm cannot reach it.</div>
  </div>
</div>
<div id="tip"></div>
<script>
const D = __DATA__;
const B = D.bodies, F = D.frames, T = D.times, NB = B.length;
const site = (D.meta && D.meta.site) || {work_circle:30, builder_orbit:33, collector_ring:37};
document.getElementById('sub').textContent =
  D.subtitle + ' — ' + NB + ' bodies, ' + F.length + ' frames, t=' +
  T[0].toFixed(1) + '–' + T[T.length-1].toFixed(1) + ' s';

// ---- group visibility keys
const groups = [...new Set(B.map(b => b.g))].sort();
const vis = {}; groups.forEach(g => vis[g] = true);
const keys = document.getElementById('keys');
groups.forEach(g => {
  const c = B.find(b => b.g === g).c;
  const lab = document.createElement('label');
  lab.innerHTML = '<input type="checkbox" checked><span class="sw" style="background:'+c+'"></span>'+g;
  lab.querySelector('input').onchange = e => { vis[g] = e.target.checked; draw(); };
  keys.appendChild(lab);
});

const plan = document.getElementById('plan'), pc = plan.getContext('2d');
const elev = document.getElementById('elev'), ec = elev.getContext('2d');
const reach = document.getElementById('reach'), rc = reach.getContext('2d');
const scrub = document.getElementById('scrub'), clock = document.getElementById('clock');
scrub.max = F.length - 1;
let cur = 0, playing = false, last = 0;

function css(v) { return getComputedStyle(document.body).getPropertyValue(v).trim(); }
let allExtent = 0;
for (const row of F) {
  for (let i = 0; i < NB; i++) {
    if (row[4*i] === null) continue;
    allExtent = Math.max(allExtent, Math.abs(row[4*i]), Math.abs(row[4*i+1]));
  }
}
allExtent = Math.max(allExtent * 1.05, site.collector_ring + 14);
function extent() {
  const base = document.getElementById('view').value === 'all' ? allExtent : site.collector_ring + 14;
  return base * (100 / +document.getElementById('zoom').value);
}
function toX(x) { return plan.width  * (0.5 + 0.5 * x / extent()); }
function toY(y) { return plan.height * (0.5 - 0.5 * y / extent()); }

function drawRings() {
  // Labels staggered by bearing: stacked at one bearing they overlap into an unreadable blob.
  const rings = [[site.work_circle, 'work ' + site.work_circle, 100],
                 [site.builder_orbit, 'orbit ' + site.builder_orbit, 60],
                 [site.collector_ring, 'ring ' + site.collector_ring, 20]];
  pc.save();
  pc.strokeStyle = css('--line'); pc.fillStyle = css('--dim'); pc.setLineDash([4,4]);
  rings.forEach(([r, name, deg]) => {
    pc.beginPath();
    pc.arc(toX(0), toY(0), Math.abs(toX(r) - toX(0)), 0, 2*Math.PI);
    pc.stroke();
    const a = deg * Math.PI / 180;
    pc.setLineDash([]);
    pc.fillText(name, toX(r*Math.cos(a)) + 4, toY(r*Math.sin(a)) - 4);
    pc.setLineDash([4,4]);
  });
  pc.setLineDash([]);
  // one ray per rank, so a machine off its own sector is obvious
  const n = (D.meta && D.meta.num_robot_ranks) || 0;
  for (let i = 0; i < n; i++) {
    const a = 2*Math.PI*i/n, r0 = site.work_circle, r1 = site.collector_ring + 8;
    pc.beginPath(); pc.moveTo(toX(r0*Math.cos(a)), toY(r0*Math.sin(a)));
    pc.lineTo(toX(r1*Math.cos(a)), toY(r1*Math.sin(a))); pc.stroke();
  }
  pc.restore();
}

function box(ctx, b, x, y, yaw) {
  const cs = Math.cos(yaw), sn = Math.sin(yaw);
  const corners = [[-b.hx,-b.hy],[b.hx,-b.hy],[b.hx,b.hy],[-b.hx,b.hy]];
  ctx.beginPath();
  corners.forEach(([lx, ly], i) => {
    const ox = b.cx + lx, oy = b.cy + ly;
    const wx = x + ox*cs - oy*sn, wy = y + ox*sn + oy*cs;
    i ? ctx.lineTo(toX(wx), toY(wy)) : ctx.moveTo(toX(wx), toY(wy));
  });
  ctx.closePath();
  // heading tick: which way is forward
  const nx = x + (b.cx + b.hx)*cs, ny = y + (b.cx + b.hx)*sn;
  return [nx, ny];
}

function draw() {
  const row = F[cur];
  pc.clearRect(0,0,plan.width,plan.height);
  drawRings();

  if (document.getElementById('trails').checked) {
    pc.save(); pc.globalAlpha = 0.5; pc.lineWidth = 1;
    for (let i = 0; i < NB; i++) {
      const b = B[i];
      if (!vis[b.g] || (b.g !== 'collector' && b.g !== 'builder')) continue;
      if (!/chassis body/i.test(b.n)) continue;
      pc.strokeStyle = b.c; pc.beginPath();
      let started = false;
      for (let f = 0; f <= cur; f++) {
        const x = F[f][4*i], y = F[f][4*i+1];
        if (x === null) continue;
        started ? pc.lineTo(toX(x), toY(y)) : pc.moveTo(toX(x), toY(y));
        started = true;
      }
      pc.stroke();
    }
    pc.restore();
  }

  pc.lineWidth = 1.25;
  const showLabels = document.getElementById('labels').checked;
  for (let i = 0; i < NB; i++) {
    const b = B[i];
    if (!vis[b.g]) continue;
    const x = row[4*i], y = row[4*i+1], yaw = row[4*i+3];
    if (x === null) continue;
    pc.strokeStyle = b.c;
    pc.fillStyle = b.c + '55';
    box(pc, b, x, y, yaw);
    pc.fill(); pc.stroke();
    if (showLabels) {
      pc.fillStyle = css('--fg');
      pc.fillText(b.n.split('/')[1].slice(0, 18), toX(x) + 4, toY(y) - 4);
    }
  }
  drawElev(row);
  drawReach(row);
  clock.textContent = 't=' + T[cur].toFixed(2) + ' s  (' + (cur+1) + '/' + F.length + ')';
  scrub.value = cur;
}

function panel(ctx, cv, xlab, ylab, x0, x1, y0, y1) {
  ctx.clearRect(0,0,cv.width,cv.height);
  ctx.strokeStyle = css('--line'); ctx.fillStyle = css('--dim');
  const L = 46, R = 14;
  ctx.strokeRect(L, 8, cv.width-L-R, cv.height-38);
  ctx.textAlign = 'left';
  ctx.fillText(xlab, cv.width - R - 8 - ctx.measureText(xlab).width, cv.height-8);
  // rotated about mid-height: anchored at the top it collided with the topmost y tick
  ctx.save(); ctx.translate(12, (cv.height - 22) / 2 + ctx.measureText(ylab).width / 2);
  ctx.rotate(-Math.PI/2); ctx.fillText(ylab, 0, 0); ctx.restore();
  for (let k = 0; k <= 4; k++) {
    const fx = x0 + (x1-x0)*k/4, fy = y0 + (y1-y0)*k/4;
    const tx = fx.toFixed(0), sx = L + (cv.width-L-R)*k/4;
    // last tick right-aligned, first left-aligned, so neither runs off the frame
    ctx.fillText(tx, k === 4 ? sx - ctx.measureText(tx).width : (k === 0 ? sx : sx - 6), cv.height-24);
    ctx.fillText(fy.toFixed(1), 20 - ctx.measureText(fy.toFixed(1)).width + 8,
                 cv.height-30 - (cv.height-46)*k/4 + 4);
  }
  return {
    px: v => L + (cv.width-L-R) * (v-x0)/(x1-x0),
    py: v => (cv.height-30) - (cv.height-46) * (v-y0)/(y1-y0),
  };
}

function drawElev(row) {
  let zlo = Infinity, zhi = -Infinity;
  for (let i = 0; i < NB; i++) {
    if (!vis[B[i].g] || row[4*i] === null) continue;
    zlo = Math.min(zlo, row[4*i+2]); zhi = Math.max(zhi, row[4*i+2]);
  }
  if (!isFinite(zlo)) { ec.clearRect(0,0,elev.width,elev.height); return; }
  const pad = Math.max(0.6, (zhi-zlo)*0.15);
  const m = panel(ec, elev, 'radius (m)', 'z (m)', 0, site.collector_ring + 12, zlo-pad, zhi+pad);
  ec.strokeStyle = css('--line'); ec.setLineDash([3,3]);
  [site.work_circle, site.builder_orbit, site.collector_ring].forEach(r => {
    ec.beginPath(); ec.moveTo(m.px(r), 8); ec.lineTo(m.px(r), elev.height-30); ec.stroke();
  });
  ec.setLineDash([]);
  for (let i = 0; i < NB; i++) {
    const b = B[i];
    if (!vis[b.g] || row[4*i] === null) continue;
    const r = Math.hypot(row[4*i], row[4*i+1]);
    if (r > site.collector_ring + 12) continue;
    ec.fillStyle = b.c;
    ec.beginPath(); ec.arc(m.px(r), m.py(row[4*i+2]), b.g === 'rock' ? 3 : 2, 0, 2*Math.PI); ec.fill();
  }
}

// Nearest rock to each builder hull, over time -- the number that decides whether a
// delivered load is pickable at all. The band is the arm's feedstock envelope.
function drawReach(row) {
  const builders = [], rocks = [];
  for (let i = 0; i < NB; i++) {
    if (/chassis body/i.test(B[i].n) && B[i].g === 'builder') builders.push(i);
    if (B[i].g === 'rock') rocks.push(i);
  }
  const m = panel(rc, reach, 't (s)', 'nearest rock (m)', T[0], T[T.length-1], 0, 12);
  rc.fillStyle = 'rgba(34,197,94,0.13)';
  rc.fillRect(m.px(T[0]), m.py(D.reach_max), m.px(T[T.length-1]) - m.px(T[0]),
              m.py(D.reach_min) - m.py(D.reach_max));
  builders.forEach(bi => {
    rc.strokeStyle = B[bi].c; rc.globalAlpha = 0.85; rc.beginPath();
    let started = false;
    for (let f = 0; f <= cur; f++) {
      const fr = F[f], bx = fr[4*bi], by = fr[4*bi+1];
      if (bx === null) continue;
      let best = Infinity;
      rocks.forEach(ri => {
        if (B[ri].r !== B[bi].r || fr[4*ri] === null) return;
        best = Math.min(best, Math.hypot(fr[4*ri]-bx, fr[4*ri+1]-by));
      });
      if (!isFinite(best)) continue;
      const px = m.px(T[f]), py = m.py(Math.min(12, best));
      started ? rc.lineTo(px, py) : rc.moveTo(px, py);
      started = true;
    }
    rc.stroke(); rc.globalAlpha = 1;
  });
}

// ---- interaction
const tip = document.getElementById('tip');
plan.onmousemove = e => {
  const r = plan.getBoundingClientRect();
  const sx = (e.clientX - r.left) * plan.width / r.width;
  const sy = (e.clientY - r.top) * plan.height / r.height;
  const row = F[cur];
  let best = null, bd = 18;
  for (let i = 0; i < NB; i++) {
    if (!vis[B[i].g] || row[4*i] === null) continue;
    const d = Math.hypot(toX(row[4*i]) - sx, toY(row[4*i+1]) - sy);
    if (d < bd) { bd = d; best = i; }
  }
  if (best === null) { tip.style.display = 'none'; return; }
  const b = B[best], row4 = 4*best;
  tip.innerHTML = '<b>' + b.n + '</b><br>rank ' + b.r + ' &middot; x=' + row[row4].toFixed(2) +
    ' y=' + row[row4+1].toFixed(2) + ' z=' + row[row4+2].toFixed(2) +
    '<br>r=' + Math.hypot(row[row4], row[row4+1]).toFixed(2) + ' m from centre';
  tip.style.display = 'block';
  tip.style.left = (e.clientX + 12) + 'px';
  tip.style.top = (e.clientY + 12) + 'px';
};
plan.onmouseleave = () => tip.style.display = 'none';

scrub.oninput = () => { cur = +scrub.value; draw(); };
document.getElementById('zoom').oninput = draw;
document.getElementById('view').onchange = draw;
document.getElementById('trails').onchange = draw;
document.getElementById('labels').onchange = draw;
document.getElementById('play').onclick = () => {
  playing = !playing;
  document.getElementById('play').textContent = playing ? 'Pause' : 'Play';
  last = performance.now();
  if (playing) requestAnimationFrame(tick);
};
document.onkeydown = e => {
  if (e.key === ' ') { e.preventDefault(); document.getElementById('play').click(); }
  if (e.key === 'ArrowRight') { cur = Math.min(F.length-1, cur+1); draw(); }
  if (e.key === 'ArrowLeft')  { cur = Math.max(0, cur-1); draw(); }
};
function tick(now) {
  if (!playing) return;
  const spd = +document.getElementById('speed').value;
  if (now - last > 1000 / (D.fps * spd)) {
    last = now;
    cur = (cur + 1) % F.length;
    draw();
  }
  requestAnimationFrame(tick);
}
draw();
</script>
"""


def main():
    ap = argparse.ArgumentParser(prog="preview_run.py", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("directory")
    ap.add_argument("-o", "--out", help="output HTML (default <dir>/preview.html)")
    ap.add_argument("--rank", help="comma-separated ranks (default: all)")
    ap.add_argument("--fps", type=float, default=4.0, help="preview frames per SIM second (default 4)")
    ap.add_argument("--from", dest="t_from", type=float, help="start sim time")
    ap.add_argument("--to", dest="t_to", type=float, help="end sim time")
    ap.add_argument("--max-frames", type=int, default=600, help="cap on frames kept (default 600)")
    ap.add_argument("--all-parts", action="store_true", help="keep track shoes, wheels, suspension")
    ap.add_argument("--reach", default="2.0,5.0", help="arm feedstock envelope, min,max (default 2.0,5.0)")
    args = ap.parse_args()

    ranks = ([int(r) for r in args.rank.split(",")] if args.rank
             else discover_ranks(args.directory))
    if not ranks:
        raise SystemExit(f"no rank_*_frames.bin in {args.directory}")

    meta, bodies, times, frames, rate = collect(
        args.directory, ranks, args.t_from, args.t_to, args.fps, args.all_parts, args.max_frames
    )

    rmin, rmax = (float(v) for v in args.reach.split(","))
    name = os.path.basename(os.path.abspath(args.directory))
    data = {
        "meta": meta,
        "bodies": bodies,
        "times": times,
        "frames": frames,
        "fps": args.fps,
        "reach_min": rmin,
        "reach_max": rmax,
        "subtitle": (f"{len(ranks)} rank(s), recorded at {rate:g} Hz, "
                     f"previewed at {args.fps:g} frames per sim second"),
    }
    out = args.out or os.path.join(args.directory, "preview.html")
    html = HTML.replace("__DATA__", json.dumps(data, separators=(",", ":")))
    html = html.replace("__TITLE__", f"AMD-UW run preview — {name}")
    with open(out, "w") as f:
        f.write(html)

    size_mb = os.path.getsize(out) / 1e6
    print(f"ranks       {ranks}")
    print(f"bodies      {len(bodies)} kept"
          f"{'' if args.all_parts else ' (track shoes/wheels/suspension dropped; --all-parts keeps them)'}")
    print(f"frames      {len(frames)} at {args.fps:g}/sim-s, t={times[0]:.2f}..{times[-1]:.2f} s")
    print(f"wrote       {out}  ({size_mb:.1f} MB)")


if __name__ == "__main__":
    main()
