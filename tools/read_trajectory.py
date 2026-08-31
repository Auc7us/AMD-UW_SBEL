#!/usr/bin/env python3
"""Read (and sanity-check) the pose recordings written by --record_dir.

Layout of a recording directory, one set per physics rank:

    rank_<r>_meta.json      run parameters
    rank_<r>_objects.jsonl  one line per recorded body: index, group/part, visual shapes
    rank_<r>_frames.bin     the poses
    static_props.jsonl      scenery (terrain, rings, pad, laid rocks), written by rank 1

frames.bin is little-endian:

    header  8s magic "AMDUWTRJ", uint32 version, uint32 rank, double rate_hz, double step
    frame   uint32 0x544A5246, double time, uint32 count,
            count * (uint32 index, float px,py,pz, float qw,qx,qy,qz)

The pose is the body's REFERENCE frame, which is what Chrono hands its renderers. A
visual shape's world transform is therefore

    body_pose * shape["pos"], shape["rot"]

taken from the object manifest -- a body is not necessarily one mesh at its own origin.

Usage:
    read_trajectory.py <dir>                     summary of every rank
    read_trajectory.py <dir> --check             the above, plus validation
    read_trajectory.py <dir> --rank 1 --list     every recorded object on that rank
    read_trajectory.py <dir> --rank 1 --track collector/chassis
    read_trajectory.py <dir> --rank 1 --npz out.npz
"""

import argparse
import glob
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


class Recording:
    def __init__(self, directory, rank):
        self.rank = rank
        self.dir = directory
        self.meta = self._load_json(f"rank_{rank}_meta.json")
        self.objects = self._load_jsonl(f"rank_{rank}_objects.jsonl")
        self.frames_path = os.path.join(directory, f"rank_{rank}_frames.bin")
        self.by_index = {o["index"]: o for o in self.objects}

    def _load_json(self, name):
        path = os.path.join(self.dir, name)
        if not os.path.exists(path):
            return {}
        with open(path) as f:
            return json.load(f)

    def _load_jsonl(self, name):
        path = os.path.join(self.dir, name)
        out = []
        if not os.path.exists(path):
            return out
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    out.append(json.loads(line))
        return out

    def name(self, index):
        o = self.by_index.get(index)
        if not o:
            return f"<unknown:{index}>"
        return f"{o['group']}/{o['part']}"

    def frames(self):
        """Yields (time, [(index, (x,y,z), (w,x,y,z)), ...]) for each frame.

        Truncated tails are tolerated and reported: a run stopped with Ctrl-C leaves a
        partial final frame, and refusing to read the other 99.99% because of it would
        be useless.
        """
        with open(self.frames_path, "rb") as f:
            head = f.read(HEADER.size)
            if len(head) < HEADER.size:
                raise ValueError(f"{self.frames_path}: file shorter than its header")
            magic, version, rank, rate, step = HEADER.unpack(head)
            if magic != FILE_MAGIC:
                raise ValueError(f"{self.frames_path}: bad magic {magic!r}")
            self.version, self.file_rank, self.rate_hz, self.step_size = version, rank, rate, step

            while True:
                fh = f.read(FRAME_HEADER.size)
                if len(fh) < FRAME_HEADER.size:
                    return
                fmagic, time, count = FRAME_HEADER.unpack(fh)
                if fmagic != FRAME_MAGIC:
                    raise ValueError(f"{self.frames_path}: lost frame sync at t={time}")
                payload = f.read(RECORD.size * count)
                if len(payload) < RECORD.size * count:
                    self.truncated = True
                    return
                poses = []
                for i in range(count):
                    idx, px, py, pz, qw, qx, qy, qz = RECORD.unpack_from(payload, i * RECORD.size)
                    poses.append((idx, (px, py, pz), (qw, qx, qy, qz)))
                yield time, poses


def index_frames(path):
    """[(offset, time)] for every frame, plus the declared rate.

    Built by walking frame headers and seeking past payloads. Frames are NOT fixed size --
    a rank's body count grows during a run as rocks are created -- so the Nth frame cannot
    be computed from a stride, only found. Walking headers reads 16 bytes per frame instead
    of the ~6.5 kB payload, so indexing an 8 GB recording touches well under 1% of it and
    the payloads of the frames actually wanted are the only ones ever read in full.

    A truncated tail (Ctrl-C leaves a partial frame) ends the index rather than raising.
    """
    out = []
    size = os.path.getsize(path)
    rate = 0.0
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
    """(time, {index: (px,py,pz,qw,qx,qy,qz)}) for the frame at `offset`, or None."""
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


def discover_ranks(directory):
    ranks = []
    for path in glob.glob(os.path.join(directory, "rank_*_frames.bin")):
        m = re.search(r"rank_(\d+)_frames\.bin$", path)
        if m:
            ranks.append(int(m.group(1)))
    return sorted(ranks)


def summarize(rec, check):
    groups = {}
    for o in rec.objects:
        groups[o["group"]] = groups.get(o["group"], 0) + 1

    n_frames = 0
    t_first = t_last = None
    prev_t = None
    counts = set()
    problems = []
    max_gap = 0.0
    bad_quat = 0
    non_finite = 0
    unknown_index = 0
    rec.truncated = False

    for time, poses in rec.frames():
        n_frames += 1
        if t_first is None:
            t_first = time
        if prev_t is not None:
            gap = time - prev_t
            if gap <= 0:
                problems.append(f"time went backwards or stalled at t={time}")
            max_gap = max(max_gap, gap)
        prev_t = time
        t_last = time
        counts.add(len(poses))
        if not check:
            continue
        for idx, p, q in poses:
            if idx not in rec.by_index:
                unknown_index += 1
            if not all(math.isfinite(v) for v in p + q):
                non_finite += 1
                continue
            n = math.sqrt(sum(v * v for v in q))
            if abs(n - 1.0) > 1e-3:
                bad_quat += 1

    print(f"--- rank {rec.rank}")
    print(f"    meta        {json.dumps({k: v for k, v in rec.meta.items() if k != 'pose_convention'})}")
    print(f"    frames      {n_frames}  t={t_first}..{t_last}")
    if n_frames > 1 and t_first is not None:
        span = t_last - t_first
        print(f"    rate        {(n_frames - 1) / span:.3f} Hz measured"
              f" (declared {getattr(rec, 'rate_hz', float('nan'))}), largest gap {max_gap * 1000:.2f} ms")
    print(f"    objects     {len(rec.objects)} in manifest; bodies per frame {sorted(counts)}")
    print("    groups      " + ", ".join(f"{g}={n}" for g, n in sorted(groups.items())))
    meshes = set()
    for o in rec.objects:
        for s in o.get("shapes", []):
            if s.get("file"):
                meshes.add(s["file"])
    print(f"    meshes      {len(meshes)} distinct source files referenced")
    if rec.truncated:
        print("    NOTE        final frame is truncated (run was interrupted); ignored")
    if check:
        status = []
        status.append(f"non-finite poses: {non_finite}")
        status.append(f"non-unit quaternions: {bad_quat}")
        status.append(f"poses with no manifest entry: {unknown_index}")
        print("    check       " + "; ".join(status))
        for p in problems[:5]:
            print(f"    PROBLEM     {p}")
        ok = not problems and non_finite == 0 and bad_quat == 0 and unknown_index == 0 and n_frames > 0
        print(f"    verdict     {'OK' if ok else 'FAILED'}")
        return ok
    return True


def list_objects(rec):
    for o in sorted(rec.objects, key=lambda o: (o["group"], o["part"])):
        files = [s.get("file", "") for s in o.get("shapes", []) if s.get("file")]
        print(f"{o['index']:5d}  {o['group']}/{o['part']:<40s} shapes={len(o.get('shapes', []))} "
              f"t0={o['first_time']:.3f} {'; '.join(os.path.basename(f) for f in files)}")


def track(rec, pattern):
    wanted = [o["index"] for o in rec.objects if pattern in f"{o['group']}/{o['part']}"]
    if not wanted:
        print(f"no object matches '{pattern}'", file=sys.stderr)
        return
    print("# " + ", ".join(f"{i}:{rec.name(i)}" for i in wanted))
    print("time," + ",".join(f"{rec.name(i)}.{c}" for i in wanted for c in ("x", "y", "z", "qw", "qx", "qy", "qz")))
    want = set(wanted)
    for time, poses in rec.frames():
        row = {idx: (p, q) for idx, p, q in poses if idx in want}
        cells = []
        for i in wanted:
            if i in row:
                p, q = row[i]
                cells += [f"{v:.6f}" for v in p] + [f"{v:.6f}" for v in q]
            else:
                cells += [""] * 7
        print(f"{time:.6f}," + ",".join(cells))


def quat_to_matrix(q):
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def transform(pos, rot, v):
    m = quat_to_matrix(rot)
    return tuple(pos[i] + sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


def bbox(rec, frame_index):
    """Rebuild the scene from the files and report each group's world bounding box.

    This is the end-to-end check on the two conventions that a consumer can silently get
    wrong: that the logged pose is the body's reference frame, and that a shape sits at
    its own local frame inside the body. Get either wrong and the numbers below stop
    looking like vehicles -- wheels stacked in the hull collapse the collector's extent,
    and using the centre of mass scatters the arm links.
    """
    target = None
    for i, (time, poses) in enumerate(rec.frames()):
        if i == frame_index:
            target = (time, poses)
            break
    if target is None:
        print(f"frame {frame_index} not in file", file=sys.stderr)
        return
    time, poses = target

    groups = {}
    for idx, p, q in poses:
        obj = rec.by_index.get(idx)
        if not obj:
            continue
        corners = []
        for s in obj.get("shapes", []):
            lo, hi = s.get("aabb_min"), s.get("aabb_max")
            if lo is None or hi is None:
                continue
            for cx in (lo[0], hi[0]):
                for cy in (lo[1], hi[1]):
                    for cz in (lo[2], hi[2]):
                        # shape frame inside the body, then the body pose.
                        local = transform(s["pos"], s["rot"], (cx, cy, cz))
                        corners.append(transform(p, q, local))
        if not corners:
            continue
        g = groups.setdefault(obj["group"], [[1e18] * 3, [-1e18] * 3, 0])
        for c in corners:
            for i in range(3):
                g[0][i] = min(g[0][i], c[i])
                g[1][i] = max(g[1][i], c[i])
        g[2] += 1

    print(f"frame {frame_index} at t={time:.4f}, world bounding box per group "
          f"(from body pose * shape frame * shape aabb)")
    for name, (lo, hi, n) in sorted(groups.items()):
        size = [hi[i] - lo[i] for i in range(3)]
        print(f"  {name:<20s} bodies_with_mesh={n:4d}  "
              f"size={size[0]:7.3f} x {size[1]:7.3f} x {size[2]:6.3f} m  "
              f"z={lo[2]:8.3f}..{hi[2]:8.3f}")


def export_npz(rec, out_path):
    import numpy as np

    times = []
    rows = []
    n_obj = len(rec.objects)
    for time, poses in rec.frames():
        # Dense [n_obj, 7], NaN where a body did not exist yet. A rock spawned at cycle 3
        # genuinely has no pose before then, and filling it with zeros would put it at
        # the site centre.
        frame = np.full((max(n_obj, 1), 7), np.nan, dtype=np.float32)
        for idx, p, q in poses:
            if idx < frame.shape[0]:
                frame[idx] = (p[0], p[1], p[2], q[0], q[1], q[2], q[3])
        times.append(time)
        rows.append(frame)
    np.savez_compressed(
        out_path,
        time=np.asarray(times, dtype=np.float64),
        pose=np.stack(rows) if rows else np.zeros((0, n_obj, 7), dtype=np.float32),
        names=np.array([rec.name(i) for i in range(n_obj)]),
        manifest=np.array([json.dumps(o) for o in rec.objects]),
    )
    print(f"wrote {out_path}: {len(times)} frames x {n_obj} objects")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("directory")
    ap.add_argument("--rank", type=int, help="restrict to one rank")
    ap.add_argument("--check", action="store_true", help="validate poses (slower)")
    ap.add_argument("--list", action="store_true", help="list the recorded objects")
    ap.add_argument("--track", help="print a CSV of every object whose group/part contains this")
    ap.add_argument("--npz", help="export the rank's poses to this .npz")
    ap.add_argument("--bbox", nargs="?", type=int, const=0, default=None, metavar="FRAME",
                    help="rebuild one frame and print each group's world bounding box")
    args = ap.parse_args()

    ranks = [args.rank] if args.rank is not None else discover_ranks(args.directory)
    if not ranks:
        print(f"no recordings in {args.directory}", file=sys.stderr)
        return 1

    static = os.path.join(args.directory, "static_props.jsonl")
    if os.path.exists(static) and not (args.list or args.track or args.npz or args.bbox is not None):
        with open(static) as f:
            print(f"static props  {sum(1 for line in f if line.strip())} (terrain, rings, pad, laid rocks)")

    ok = True
    for rank in ranks:
        rec = Recording(args.directory, rank)
        if args.list:
            list_objects(rec)
        elif args.track:
            track(rec, args.track)
        elif args.npz:
            export_npz(rec, args.npz)
        elif args.bbox is not None:
            bbox(rec, args.bbox)
        else:
            ok &= summarize(rec, args.check)
    return 0 if ok else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:
        # `... | head` closes the pipe; that is not an error worth a traceback.
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        sys.exit(0)
