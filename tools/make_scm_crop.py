#!/usr/bin/env python3
# One-time: crop terrain2.bmp into a per-robot SCM lane (LEN x WID m) aligned with
# each robot's driving corridor, so every robot drives its whole mission on SCM and
# each gets a genuinely different chunk of the map.
#
# Robot layout (mirrors src/RobotLayout.h):
#   spawn(i)   = (0, (i - 0.5*(N-1)) * start_spacing)
#   heading(i) = 330 deg for i==0, else 60 deg
# Rocks run along heading at 30,60,...,~450 m, so a 450 m lane covers the field.
import math
import numpy as np
from PIL import Image

SRC = "/home/chrono-user/mountdir/amd-uw/data/terrain/terrain2.bmp"
OUT_TMPL = "/home/chrono-user/mountdir/amd-uw/data/terrain/terrain_scm_r{}.bmp"

NUM_ROBOTS = 2
START_SPACING = 50.0
LEN_X, WID_Y = 450.0, 30.0        # lane: along-heading x across
CENTER_OFFSET = LEN_X / 2.0 - 5.0  # frame origin ~5 m behind spawn -> covers spawn-5 .. spawn+445

im = np.asarray(Image.open(SRC).convert("L")).astype(float)
NPX = im.shape[0]
WORLD = 1024.0                    # terrain2 spans 1024 m (256 px * 4 m)
SRC_HMIN, SRC_HMAX = -25.0, 25.0

def world_height(x, y):
    c = (x + WORLD / 2) / WORLD * (NPX - 1)
    r = (WORLD / 2 - y) / WORLD * (NPX - 1)
    c = min(max(c, 0), NPX - 1); r = min(max(r, 0), NPX - 1)
    c0, r0 = int(math.floor(c)), int(math.floor(r))
    c1, r1 = min(c0 + 1, NPX - 1), min(r0 + 1, NPX - 1)
    fc, fr = c - c0, r - r0
    g = (im[r0, c0] * (1 - fc) * (1 - fr) + im[r0, c1] * fc * (1 - fr) +
         im[r1, c0] * (1 - fc) * fr + im[r1, c1] * fc * fr)
    return g / 255.0 * (SRC_HMAX - SRC_HMIN) + SRC_HMIN

def spawn(i):
    return np.array([0.0, (i - 0.5 * (NUM_ROBOTS - 1)) * START_SPACING])

def heading_deg(i):
    return 330.0 if i == 0 else 60.0

print("// paste the scm_lanes[] table into main.cpp")
for i in range(NUM_ROBOTS):
    hdg = math.radians(heading_deg(i))
    fwd = np.array([math.cos(hdg), math.sin(hdg)])
    origin = spawn(i) + fwd * CENTER_OFFSET
    cA, sA = math.cos(hdg), math.sin(hdg)

    NX, NY = int(LEN_X), int(WID_Y)
    heights = np.zeros((NY, NX))
    for iy in range(NY):
        ly = (iy / (NY - 1) - 0.5) * WID_Y
        for ix in range(NX):
            lx = (ix / (NX - 1) - 0.5) * LEN_X
            wx = origin[0] + cA * lx - sA * ly
            wy = origin[1] + sA * lx + cA * ly
            heights[iy, ix] = world_height(wx, wy)

    hmin, hmax = float(heights.min()), float(heights.max())
    gray = np.full_like(heights, 128) if hmax - hmin < 1e-6 else (heights - hmin) / (hmax - hmin) * 255.0
    Image.fromarray(gray.round().astype(np.uint8), mode="L").save(OUT_TMPL.format(i))

    mid = heights[NY // 2, :]
    slope = math.degrees(math.atan2(mid.max() - mid.min(), LEN_X))
    print(f'  {{"terrain/terrain_scm_r{i}.bmp", {origin[0]:.3f}, {origin[1]:.3f}, '
          f'{heading_deg(i):.1f}, {hmin:.3f}, {hmax:.3f}}},  '
          f'// robot {i} (rank {i+1}); relief {hmax-hmin:.1f} m, ~{slope:.1f} deg')
