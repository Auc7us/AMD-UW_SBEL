#!/usr/bin/env python3
# Grade a level work pad into terrain2.bmp and emit it as a 16-bit heightmap PNG.
#
# Why a pad at all: the site rings (work circle r=30, builder orbit r=33, collector ring
# r=37) sit on one broad hillside. Measured on terrain2.bmp, the r<=40 disc fits a plane at
# 5.02% grade, and the builder orbit swings 3.96 m with a 17% peak along-path grade -- more
# than the single-pin tracks want to climb while carrying a wall segment.
#
# Why not just blur: the orbit profile is dominated by its n=1 harmonic (1.61 m amplitude,
# i.e. a tilt); the roughness left after removing n<=3 is only 0.17 m rms. A low-pass filter
# has almost nothing to remove and would leave the tilt untouched. So the interior is blended
# toward a LEVEL plane; the blur only softens the target field so the pad meets the untouched
# terrain without a crease.
#
# Why 16-bit PNG out and not BMP: RigidTerrain maps gray -> [hMin,hMax] over the image's full
# range, and Chrono's STB wrapper always loads 16-bit (8-bit inputs are promoted), so an 8-bit
# BMP quantizes the +/-25 m range into 0.196 m steps. That is invisible on a 5% hillside and
# glaring on a flat pad -- the driving surface would terrace into 20 cm stairs. 16-bit gives
# 0.76 mm steps. Chrono's vendored stb_image reads 16-bit PNG natively.
#
# Pure stdlib (struct + zlib): no numpy/PIL, so it runs on the host and in the container alike.
import argparse
import math
import struct
import zlib

# Must mirror main.cpp: terrain_pixels_{x,y} * terrain_resolution_scale, and the height range
# handed to RigidTerrain::AddPatch. Chrono lays nv_x vertices across `length`, so the vertex
# pitch is length/(nv_x-1) -- 4.0157 m, NOT the 4.0 resolution scale.
WORLD_L = 256.0 * 4.0
WORLD_W = 256.0 * 4.0
HMIN, HMAX = -25.0, 25.0


def read_bmp_gray(path):
    """Return (gray[iy][ix], nx, ny) with iy=0 the TOP row, matching STB::Gray(ix, iy)."""
    d = open(path, 'rb').read()
    if d[:2] != b'BM':
        raise ValueError('not a BMP: %s' % path)
    data_off = struct.unpack_from('<I', d, 10)[0]
    hdr_size = struct.unpack_from('<I', d, 14)[0]
    nx, ny_signed = struct.unpack_from('<ii', d, 18)
    bpp = struct.unpack_from('<H', d, 28)[0]
    if bpp != 8:
        raise ValueError('expected 8-bit BMP, got %d bpp' % bpp)
    ny = abs(ny_signed)
    top_down = ny_signed < 0

    # Resolve the palette. stb loads paletted BMPs to RGB then folds to gray with its own
    # integer luma; do the same so a non-grayscale palette would still land where Chrono
    # puts it. terrain2.bmp happens to carry an identity gray ramp.
    ncol = struct.unpack_from('<I', d, 46)[0] or 256
    pal_off = 14 + hdr_size
    lut = []
    for i in range(ncol):
        b, g, r = d[pal_off + 4 * i], d[pal_off + 4 * i + 1], d[pal_off + 4 * i + 2]
        lut.append((r * 77 + g * 150 + b * 29) >> 8)

    stride = (nx + 3) // 4 * 4
    gray = [None] * ny
    for row in range(ny):
        base = data_off + row * stride
        # BMP rows run bottom-up unless the height is negative.
        iy = row if top_down else ny - 1 - row
        gray[iy] = [lut[d[base + ix]] for ix in range(nx)]
    return gray, nx, ny


def write_png16_gray(path, samples, nx, ny):
    """16-bit grayscale PNG. samples[iy][ix] in [0, 65535], iy=0 first scanline (top)."""
    raw = bytearray()
    for iy in range(ny):
        raw.append(0)  # filter type 0 (None) -- the field is smooth, filtering buys little
        for ix in range(nx):
            raw += struct.pack('>H', samples[iy][ix])

    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload +
                struct.pack('>I', zlib.crc32(tag + payload) & 0xFFFFFFFF))

    ihdr = struct.pack('>IIBBBBB', nx, ny, 16, 0, 0, 0, 0)  # bitdepth 16, colortype 0 gray
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', ihdr))
        f.write(chunk(b'IDAT', zlib.compress(bytes(raw), 9)))
        f.write(chunk(b'IEND', b''))


def gaussian_blur(field, nx, ny, sigma_px):
    """Separable Gaussian, edges clamped. Only ever applied to the pad TARGET, never to the
    terrain that survives the blend, so clamping at the image border is harmless."""
    if sigma_px <= 0.0:
        return [row[:] for row in field]
    rad = max(1, int(math.ceil(3.0 * sigma_px)))
    k = [math.exp(-0.5 * (t / sigma_px) ** 2) for t in range(-rad, rad + 1)]
    s = sum(k)
    k = [v / s for v in k]
    tmp = [[0.0] * nx for _ in range(ny)]
    for iy in range(ny):
        src = field[iy]
        for ix in range(nx):
            acc = 0.0
            for t in range(-rad, rad + 1):
                acc += k[t + rad] * src[min(nx - 1, max(0, ix + t))]
            tmp[iy][ix] = acc
    out = [[0.0] * nx for _ in range(ny)]
    for iy in range(ny):
        for ix in range(nx):
            acc = 0.0
            for t in range(-rad, rad + 1):
                acc += k[t + rad] * tmp[min(ny - 1, max(0, iy + t))][ix]
            out[iy][ix] = acc
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--src', default='data/terrain/terrain2.bmp')
    p.add_argument('--out', default='data/terrain/terrain2_graded.png')
    p.add_argument('--center', type=float, nargs=2, default=(0.0, 0.0),
                   help='pad centre in world XY; must match site_center_{x,y}')
    p.add_argument('--pad-radius', type=float, default=45.0,
                   help='fully graded out to here (covers the r=37 collector ring + margin)')
    p.add_argument('--taper-radius', type=float, default=130.0,
                   help='blend reaches zero here; terrain beyond is bit-exact untouched')
    p.add_argument('--keep', type=float, default=0.15,
                   help='fraction of local relief retained inside the pad (0 = dead flat)')
    p.add_argument('--sigma-px', type=float, default=1.5,
                   help='Gaussian sigma on the pad target, in pixels (1 px = 4.016 m)')
    p.add_argument('--level', type=float, default=None,
                   help='pad elevation [m]; default = mean source height over the pad disc, '
                        'which balances cut against fill')
    args = p.parse_args()

    if args.taper_radius <= args.pad_radius:
        p.error('--taper-radius must exceed --pad-radius')
    if not 0.0 <= args.keep <= 1.0:
        p.error('--keep must be in [0, 1]')

    gray, nx, ny = read_bmp_gray(args.src)
    dx = WORLD_L / (nx - 1)
    dy = WORLD_W / (ny - 1)
    to_z = (HMAX - HMIN) / 255.0  # source is 8-bit, so its full range is 255
    z = [[HMIN + gray[iy][ix] * to_z for ix in range(nx)] for iy in range(ny)]

    cx, cy = args.center

    def radius(ix, iy):
        return math.hypot(ix * dx - 0.5 * WORLD_L - cx, 0.5 * WORLD_W - iy * dy - cy)

    level = args.level
    if level is None:
        acc = n = 0
        for iy in range(ny):
            for ix in range(nx):
                if radius(ix, iy) <= args.pad_radius:
                    acc += z[iy][ix]
                    n += 1
        if n == 0:
            p.error('pad disc contains no pixels -- check --center/--pad-radius')
        level = acc / n

    target = gaussian_blur(z, nx, ny, args.sigma_px)

    r0, r1 = args.pad_radius, args.taper_radius
    out = [[0] * nx for _ in range(ny)]
    max_shift = 0.0
    fill_vol = 0.0
    zlo, zhi = math.inf, -math.inf
    for iy in range(ny):
        for ix in range(nx):
            r = radius(ix, iy)
            if r <= r0:
                w = 1.0
            elif r >= r1:
                w = 0.0
            else:
                # Raised cosine: C1 at both ends, so no crease where the blend starts or stops.
                w = 0.5 * (1.0 + math.cos(math.pi * (r - r0) / (r1 - r0)))
            tgt = level + args.keep * (target[iy][ix] - level)
            zn = z[iy][ix] + w * (tgt - z[iy][ix])
            shift = zn - z[iy][ix]
            max_shift = max(max_shift, abs(shift))
            fill_vol += abs(shift) * dx * dy
            zlo, zhi = min(zlo, zn), max(zhi, zn)
            if not HMIN <= zn <= HMAX:
                raise SystemExit('height %.3f m escapes the [%.1f, %.1f] range the patch is '
                                 'built with; raise the range in main.cpp' % (zn, HMIN, HMAX))
            # 16-bit output, so the range maps over 65535 -- see the file header on why.
            out[iy][ix] = int(round((zn - HMIN) * 65535.0 / (HMAX - HMIN)))

    write_png16_gray(args.out, out, nx, ny)
    print('wrote %s  (%dx%d, 16-bit gray)' % (args.out, nx, ny))
    print('  pad centre (%.1f, %.1f), graded r<=%.1f m, taper to %.1f m, keep %.0f%%, sigma %.2f px (%.1f m)'
          % (cx, cy, r0, r1, args.keep * 100.0, args.sigma_px, args.sigma_px * dx))
    print('  pad level %.3f m; output spans %.3f .. %.3f m' % (level, zlo, zhi))
    print('  max cut/fill %.2f m; earth moved %.0f m3' % (max_shift, fill_vol))
    print('  vertical quantization %.4f m/step (an 8-bit BMP would give %.3f m)'
          % ((HMAX - HMIN) / 65535.0, (HMAX - HMIN) / 255.0))


if __name__ == '__main__':
    main()
