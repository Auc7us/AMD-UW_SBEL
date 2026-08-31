#!/usr/bin/env python3
"""Generate a lugged wheel as a closed triangle mesh (Wavefront OBJ).

A smooth barrel plus N rectangular grousers running ACROSS the tread in an
ALTERNATING, INTERLOCKING pattern -- each lug spans --lug-span of the tyre width
and is flush with one edge, with consecutive lugs swapping edges, the way a real
tractor or lunar-rover tyre is cut. Written as triangles, not as a
cylinder primitive plus box primitives, on purpose: Chrono's SCM HIP ray-cast
backend consumes ONLY ChCollisionShape::TRIANGLEMESH (see AppendLocalMesh in
SCMTerrainRaycastGpu.cpp, which `continue`s past every other shape type), so a
cylinder+box build of the same shape can never reach the GPU.

Spin axis is +Y, matching a Chrono wheel spindle frame.

Lug tips sit at --radius so the effective rolling radius still matches the tyre
being replaced; the barrel is recessed by --lug-height beneath it.

Why the lugs alternate rather than spanning the full width: a full-width grouser
presents one continuous edge to the soil across the whole contact patch, which
shears a single wide slab and gives no lateral bite. Alternating half-width lugs
break that edge up and leave a set of pockets that soil can pack into, which is
what a real lug pattern is for.

--lug-span above 0.5 makes consecutive lugs OVERLAP in the middle of the tread,
which is the interlocking part and is deliberate: at 2/3 the centre third of the
tread is covered by every lug, so contact is continuous across the width at all
times while each outer third is served by alternate lugs. Below 0.5 the pattern
opens a bare central band and the tyre will tramline.

--lugs should be EVEN, or the pattern does not close around the circumference and
the last and first lug end up on the same edge.
"""
import argparse, math

def main():
    p = argparse.ArgumentParser()
    p.add_argument("-o", "--out", default="data/vehicle/LRV/meshes/Polaris_lugged_wheel_collision.obj")
    p.add_argument("--radius", type=float, default=0.4089, help="outer radius at the lug tips [m]")
    p.add_argument("--width", type=float, default=0.30, help="tyre width [m]")
    p.add_argument("--lugs", type=int, default=10)
    p.add_argument("--lug-height", type=float, default=0.020, help="radial protrusion above the barrel [m]")
    p.add_argument("--lug-arc", type=float, default=0.035, help="circumferential length of a lug at its base [m]")
    p.add_argument("--lug-span", type=float, default=2.0 / 3.0,
                   help="fraction of the tyre width one lug covers; consecutive lugs sit "
                        "against opposite edges and overlap in the middle (1.0 = the old "
                        "full-width grousers)")
    p.add_argument("--segments", type=int, default=60, help="barrel facets around (a multiple of --lugs is tidiest)")
    a = p.parse_args()

    R_tip = a.radius
    R_bar = a.radius - a.lug_height
    hw = 0.5 * a.width
    V, F = [], []

    def add_v(x, y, z):
        V.append((x, y, z))
        return len(V)            # OBJ indices are 1-based

    def quad(v1, v2, v3, v4):    # split a quad into two triangles
        F.append((v1, v2, v3)); F.append((v1, v3, v4))

    # ---- barrel: a closed tube of R_bar, capped at both sidewalls
    n = a.segments
    ring_l, ring_r = [], []
    for i in range(n):
        th = 2.0 * math.pi * i / n
        c, s = math.cos(th), math.sin(th)
        ring_l.append(add_v(R_bar * c, -hw, R_bar * s))
        ring_r.append(add_v(R_bar * c, +hw, R_bar * s))
    hub_l = add_v(0.0, -hw, 0.0)
    hub_r = add_v(0.0, +hw, 0.0)
    for i in range(n):
        j = (i + 1) % n
        quad(ring_l[i], ring_r[i], ring_r[j], ring_l[j])   # tread
        F.append((hub_l, ring_l[j], ring_l[i]))            # left sidewall
        F.append((hub_r, ring_r[i], ring_r[j]))            # right sidewall

    # ---- lugs: a box per grouser, raised radially, spanning --lug-span of the width
    #      and alternating which edge it is flush with. See the module docstring.
    half_arc = 0.5 * a.lug_arc
    span = min(max(a.lug_span, 1e-6), 1.0) * a.width
    if a.lugs % 2:
        print("warning: --lugs %d is odd, so the alternation does not close around the "
              "circumference: lug %d and lug 0 both sit against the -y edge."
              % (a.lugs, a.lugs - 1))
    for k in range(a.lugs):
        th = 2.0 * math.pi * k / a.lugs
        # Even lugs flush with the -y edge, odd lugs flush with +y. y0 < y1 always, so the
        # face winding below (which assumes "m" is the lower-y plane) still holds.
        y0, y1 = (-hw, -hw + span) if k % 2 == 0 else (hw - span, hw)
        # local frame: er points out along the radius, et along the circumference
        er = (math.cos(th), math.sin(th))
        et = (-math.sin(th), math.cos(th))
        corners = []
        for r in (R_bar, R_tip):                 # base sits inside the barrel, tip outside
            for t in (-half_arc, +half_arc):
                x = er[0] * r + et[0] * t
                z = er[1] * r + et[1] * t
                corners.append((x, z))
        # corners: 0=(base,-t) 1=(base,+t) 2=(tip,-t) 3=(tip,+t)
        idx = {}
        for ci, (x, z) in enumerate(corners):
            idx[(ci, -1)] = add_v(x, y0, z)
            idx[(ci, +1)] = add_v(x, y1, z)
        b0m, b1m, t0m, t1m = (idx[(i, -1)] for i in range(4))
        b0p, b1p, t0p, t1p = (idx[(i, +1)] for i in range(4))
        quad(t0m, t0p, t1p, t1m)   # outer face
        quad(b1m, b1p, b0p, b0m)   # inner face (buried, kept so the box is closed)
        quad(b0m, b0p, t0p, t0m)   # trailing side
        quad(b1p, b1m, t1m, t1p)   # leading side
        quad(b0p, b1p, t1p, t0p)   # +y end cap  (tread interior on an even lug)
        quad(b1m, b0m, t0m, t1m)   # -y end cap  (tread interior on an odd lug)

    with open(a.out, "w") as f:
        f.write("# lugged wheel: R_tip=%.4f R_barrel=%.4f width=%.3f lugs=%d "
                "lug_span=%.3f (alternating) lug_h=%.3f lug_arc=%.3f seg=%d\n"
                % (R_tip, R_bar, a.width, a.lugs, span / a.width, a.lug_height, a.lug_arc,
                   a.segments))
        f.write("# spin axis +Y; generated by tools/make_lugged_wheel.py\n")
        for x, y, z in V:
            f.write("v %.6f %.6f %.6f\n" % (x, y, z))
        for t in F:
            f.write("f %d %d %d\n" % t)
    print("%s: %d vertices, %d triangles (R_tip=%.4f, R_barrel=%.4f, width=%.3f, %d lugs "
          "at %.0f%% width, alternating, %.3f m overlap in the middle)"
          % (a.out, len(V), len(F), R_tip, R_bar, a.width, a.lugs,
             100.0 * span / a.width, max(0.0, 2.0 * span - a.width)))

main()
