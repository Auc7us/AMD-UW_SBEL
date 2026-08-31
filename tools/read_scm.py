#!/usr/bin/env python3
"""Read rank_<r>_scm.bin -- the deformed-ground companion to the pose recording.

Reference decoder and validator. `--check` verifies the file parses, that frame times
advance, and that the accumulated node set is plausible; `--accumulate T` replays every
frame up to T and reports the deformed surface at that instant.

Accumulate by OVERWRITE: heights are absolute, so a frame that re-states a node simply
supersedes the earlier value. That is also why keyframes need no flag -- a frame carrying
every deformed node is idempotent with the diffs around it.

Node (i,j) sits at local (i*delta, j*delta, z); world = plane_frame * that.

  python3 tools/read_scm.py <run_dir> --check
  python3 tools/read_scm.py <run_dir> --rank 1 --accumulate 30
  python3 tools/read_scm.py <run_dir> --rank 1 --npz scm_r1.npz --accumulate 1e9
"""
import argparse, glob, os, struct, sys

MAGIC = b"AMDUWSCM"
FRAME_MAGIC = 0x4D435353
HEADER = struct.Struct("<8sII" + "d" * 9 + "ii")  # magic, ver, rank, rate, delta,
                                                 # plane[7] (pos xyz + quat wxyz), nx, ny
assert HEADER.size == 96, HEADER.size
FRAME_HEAD = struct.Struct("<IdI")            # magic, time, count
NODE = struct.Struct("<iif")


def read_header(f):
    raw = f.read(HEADER.size)
    if len(raw) < HEADER.size:
        raise ValueError("file shorter than its header")
    v = HEADER.unpack(raw)
    if v[0] != MAGIC:
        raise ValueError("bad magic %r (expected %r)" % (v[0], MAGIC))
    return {
        "version": v[1], "rank": v[2], "rate_hz": v[3], "delta": v[4],
        "plane_pos": (v[5], v[6], v[7]),
        "plane_quat_wxyz": (v[8], v[9], v[10], v[11]),
        "nx": v[12], "ny": v[13],
    }


def frames(f):
    """Yield (time, [(i, j, z), ...]). A truncated tail is reported, not crashed on."""
    while True:
        raw = f.read(FRAME_HEAD.size)
        if not raw:
            return
        if len(raw) < FRAME_HEAD.size:
            print("    NOTE   trailing %d bytes: frame header truncated (run interrupted)" % len(raw))
            return
        magic, t, count = FRAME_HEAD.unpack(raw)
        if magic != FRAME_MAGIC:
            raise ValueError("bad frame magic 0x%08X at offset %d" % (magic, f.tell() - FRAME_HEAD.size))
        body = f.read(count * NODE.size)
        if len(body) < count * NODE.size:
            print("    NOTE   frame t=%.4f truncated (%d of %d nodes); ignored"
                  % (t, len(body) // NODE.size, count))
            return
        yield t, [NODE.unpack_from(body, k * NODE.size) for k in range(count)]


def do_file(path, until, npz):
    print("--- %s" % os.path.basename(path))
    with open(path, "rb") as f:
        h = read_header(f)
        print("    header  v%d rank=%d rate=%.3f Hz delta=%.6f grid=+/-%dx%d"
              % (h["version"], h["rank"], h["rate_hz"], h["delta"], h["nx"], h["ny"]))
        print("    plane   pos=%s quat(wxyz)=%s%s"
              % (h["plane_pos"], h["plane_quat_wxyz"],
                 "  [identity]" if h["plane_pos"] == (0, 0, 0)
                 and h["plane_quat_wxyz"] == (1, 0, 0, 0) else ""))
        acc, nf, nodes_seen, prev_t, tmax = {}, 0, 0, None, 0.0
        biggest = (0, 0.0)
        for t, nodes in frames(f):
            if prev_t is not None and t < prev_t:
                print("    WARN   frame time went backwards: %.4f after %.4f" % (t, prev_t))
            prev_t, nf, tmax = t, nf + 1, t
            nodes_seen += len(nodes)
            if len(nodes) > biggest[1]:
                biggest = (t, len(nodes))
            if t <= until:
                for i, j, z in nodes:
                    acc[(i, j)] = z
        print("    frames  %d  t=0..%.4f   %d node records (%.1f per frame avg)"
              % (nf, tmax, nodes_seen, nodes_seen / max(1, nf)))
        print("    largest single frame: %d nodes at t=%.3f (a keyframe)" % (biggest[1], biggest[0]))
        if not acc:
            print("    accum   EMPTY up to t=%g -- nothing deformed" % until)
            return
        zs = [z for z in acc.values()]
        ii = [k[0] for k in acc]
        jj = [k[1] for k in acc]
        d = h["delta"]
        print("    accum   %d distinct nodes up to t=%g" % (len(acc), until))
        print("            i in [%d,%d]  j in [%d,%d]  -> local x in [%.2f,%.2f] y in [%.2f,%.2f] m"
              % (min(ii), max(ii), min(jj), max(jj),
                 min(ii) * d, max(ii) * d, min(jj) * d, max(jj) * d))
        print("            z in [%.4f,%.4f] m   footprint area = %.2f m^2"
              % (min(zs), max(zs), len(acc) * d * d))
        if any(abs(k) > h["nx"] for k in ii) or any(abs(k) > h["ny"] for k in jj):
            print("    WARN   some indices fall outside the declared grid half-counts")
        if npz:
            try:
                import numpy as np
            except ImportError:
                print("    npz    skipped: numpy not available")
                return
            np.savez_compressed(npz, i=np.array(ii, dtype=np.int32),
                                j=np.array(jj, dtype=np.int32),
                                z=np.array(zs, dtype=np.float32),
                                delta=h["delta"], plane_pos=np.array(h["plane_pos"]),
                                plane_quat_wxyz=np.array(h["plane_quat_wxyz"]),
                                nx=h["nx"], ny=h["ny"])
            print("    npz     wrote %s" % npz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--rank", type=int, default=None)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--accumulate", type=float, default=1e30,
                    help="replay frames up to this sim time (default: all)")
    ap.add_argument("--npz", default=None)
    a = ap.parse_args()
    pat = "rank_%d_scm.bin" % a.rank if a.rank is not None else "rank_*_scm.bin"
    paths = sorted(glob.glob(os.path.join(a.run_dir, pat)))
    if not paths:
        sys.exit("no %s in %s" % (pat, a.run_dir))
    for p in paths:
        do_file(p, a.accumulate, a.npz)
    return 0


sys.exit(main())
