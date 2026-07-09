#!/usr/bin/env python3
# One-time: crop terrain2.bmp into a 30 m x 150 m heightmap lane aligned with
# robot 1's driving corridor, for use as the SCM terrain template.
#
# Robot 1 (index 0): spawns at world (0, -25), heading 330 deg. Rocks lie along
# that heading at 30, 60, 90, ... m. We lay a 150 m (along heading) x 30 m (wide)
# patch starting ~5 m behind the spawn, and sample the existing height field
# along it so the SCM lane reproduces the relief robot 1 actually drove over.
import math
import numpy as np
from PIL import Image

SRC = "/home/chrono-user/mountdir/amd-uw/data/terrain/terrain2.bmp"
OUT = "/home/chrono-user/mountdir/amd-uw/data/terrain/terrain_scm.bmp"

# --- source height field (matches the demo's RigidTerrain mapping) ---
im = np.asarray(Image.open(SRC).convert("L")).astype(float)  # 256x256, gray 0..255
NPX = im.shape[0]
WORLD = 1024.0                 # terrain_length = 256 * 4.0
SRC_HMIN, SRC_HMAX = -25.0, 25.0
def world_height(x, y):
    # world (x,y) in [-512,512] -> pixel (col from x, row from -y), bilinear
    c = (x + WORLD / 2) / WORLD * (NPX - 1)
    r = (WORLD / 2 - y) / WORLD * (NPX - 1)
    c = min(max(c, 0), NPX - 1); r = min(max(r, 0), NPX - 1)
    c0, r0 = int(math.floor(c)), int(math.floor(r))
    c1, r1 = min(c0 + 1, NPX - 1), min(r0 + 1, NPX - 1)
    fc, fr = c - c0, r - r0
    g = (im[r0, c0] * (1 - fc) * (1 - fr) + im[r0, c1] * fc * (1 - fr) +
         im[r1, c0] * (1 - fc) * fr + im[r1, c1] * fc * fr)
    return g / 255.0 * (SRC_HMAX - SRC_HMIN) + SRC_HMIN

# --- robot 1 corridor patch ---
LEN_X, WID_Y = 150.0, 30.0     # sizeX (along heading), sizeY (across)
spawn = np.array([0.0, -25.0])
hdg = math.radians(330.0)
fwd = np.array([math.cos(hdg), math.sin(hdg)])
# frame origin so the patch spans ~5 m behind spawn to ~145 m ahead
origin = spawn + fwd * (LEN_X / 2.0 - 5.0)
cA, sA = math.cos(hdg), math.sin(hdg)   # rotate local (lx,ly) -> world

# output image resolution (SCM resamples to its own delta; 1 px/m is plenty
# given the 4 m/px source)
NX, NY = int(LEN_X), int(WID_Y)         # 150 x 30 px
heights = np.zeros((NY, NX))
for iy in range(NY):
    ly = (iy / (NY - 1) - 0.5) * WID_Y
    for ix in range(NX):
        lx = (ix / (NX - 1) - 0.5) * LEN_X
        wx = origin[0] + cA * lx - sA * ly
        wy = origin[1] + sA * lx + cA * ly
        heights[iy, ix] = world_height(wx, wy)

hmin, hmax = float(heights.min()), float(heights.max())
# map heights -> gray 0..255 with 0->hmin, 255->hmax
if hmax - hmin < 1e-6:
    gray = np.full_like(heights, 128)
else:
    gray = (heights - hmin) / (hmax - hmin) * 255.0
Image.fromarray(gray.round().astype(np.uint8), mode="L").save(OUT)

# average slope along the lane centerline
mid = heights[NY // 2, :]
slope = math.degrees(math.atan2(mid.max() - mid.min(), LEN_X))
print(f"wrote {OUT}  ({NX}x{NY} px)")
print(f"hMin={hmin:.3f}  hMax={hmax:.3f}  relief={hmax-hmin:.2f} m")
print(f"centerline slope over 150 m ~ {slope:.1f} deg")
print("--- paste into main.cpp ---")
print(f"scm_ref_origin = ({origin[0]:.3f}, {origin[1]:.3f}, 0.0)")
print(f"scm_ref_yaw_deg = 330.0")
print(f"scm_size_x = {LEN_X}, scm_size_y = {WID_Y}")
print(f"scm_hmin = {hmin:.3f}, scm_hmax = {hmax:.3f}")
