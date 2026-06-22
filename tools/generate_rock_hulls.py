#!/usr/bin/env python3
"""Generate convex-hull OBJ collision meshes for the Curiosity rock assets."""

from __future__ import annotations

import math
import os
from pathlib import Path


EPS = 1e-8


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a):
    return math.sqrt(dot(a, a))


def scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def read_vertices(path: Path):
    vertices = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("v "):
                parts = line.split()
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return vertices


def convex_hull_2d(points):
    points = sorted(set(points))
    if len(points) <= 1:
        return points

    def orientation(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in points:
        while len(lower) >= 2 and orientation(lower[-2], lower[-1], p) <= EPS:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and orientation(upper[-2], upper[-1], p) <= EPS:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def hull_faces(vertices):
    centroid = scale(tuple(sum(p[i] for p in vertices) for i in range(3)), 1.0 / len(vertices))
    planes = {}

    for i in range(len(vertices) - 2):
        for j in range(i + 1, len(vertices) - 1):
            for k in range(j + 1, len(vertices)):
                normal = cross(sub(vertices[j], vertices[i]), sub(vertices[k], vertices[i]))
                length = norm(normal)
                if length < EPS:
                    continue

                normal = scale(normal, 1.0 / length)
                plane_d = -dot(normal, vertices[i])
                signed = [dot(normal, p) + plane_d for p in vertices]

                if not (all(d <= 1e-7 for d in signed) or all(d >= -1e-7 for d in signed)):
                    continue

                if dot(normal, sub(centroid, vertices[i])) > 0:
                    normal = scale(normal, -1.0)
                    plane_d = -plane_d

                key = tuple(round(x, 7) for x in (*normal, plane_d))
                planes[key] = (normal, plane_d)

    faces = []
    seen_faces = set()

    for normal, plane_d in planes.values():
        plane_indices = [i for i, p in enumerate(vertices) if abs(dot(normal, p) + plane_d) < 1e-6]
        if len(plane_indices) < 3:
            continue

        center = scale(tuple(sum(vertices[i][axis] for i in plane_indices) for axis in range(3)),
                       1.0 / len(plane_indices))
        reference = (1.0, 0.0, 0.0) if abs(normal[0]) < 0.9 else (0.0, 1.0, 0.0)
        u = cross(reference, normal)
        u = scale(u, 1.0 / norm(u))
        v = cross(normal, u)

        projected = []
        for idx in plane_indices:
            rel = sub(vertices[idx], center)
            projected.append((dot(rel, u), dot(rel, v), idx))

        boundary = convex_hull_2d(projected)
        boundary_indices = [p[2] for p in boundary]
        if len(boundary_indices) < 3:
            continue

        for i in range(1, len(boundary_indices) - 1):
            face = (boundary_indices[0], boundary_indices[i], boundary_indices[i + 1])
            key = tuple(sorted(face))
            if key not in seen_faces:
                seen_faces.add(key)
                faces.append(face)

    return faces


def write_obj(path: Path, vertices, faces, source: Path):
    used = sorted({idx for face in faces for idx in face})
    remap = {old: new for new, old in enumerate(used, start=1)}

    with path.open("w", encoding="utf-8") as f:
        f.write(f"# Convex hull generated from {source.name}\n")
        f.write(f"# vertices {len(used)} faces {len(faces)}\n")
        for idx in used:
            p = vertices[idx]
            f.write(f"v {p[0]:.9f} {p[1]:.9f} {p[2]:.9f}\n")
        for face in faces:
            f.write(f"f {remap[face[0]]} {remap[face[1]]} {remap[face[2]]}\n")

    return len(used), len(faces)


def main():
    repo_root = Path(__file__).resolve().parents[1]
    chrono_data = Path(os.environ.get("CHRONO_DATA_DIR", repo_root.parent / "chrono" / "data"))
    source_dir = chrono_data / "robot" / "curiosity" / "rocks"
    output_dir = repo_root / "data" / "rocks" / "curiosity_hulls"
    output_dir.mkdir(parents=True, exist_ok=True)

    for name in ("rock1", "rock2", "rock3"):
        source = source_dir / f"{name}.obj"
        vertices = read_vertices(source)
        faces = hull_faces(vertices)
        if not faces:
            raise RuntimeError(f"no hull faces generated for {source}")

        output = output_dir / f"{name}_hull.obj"
        hull_vertices, hull_faces_count = write_obj(output, vertices, faces, source)
        print(f"{source.name}: input {len(vertices)} vertices -> hull {hull_vertices} vertices, "
              f"{hull_faces_count} triangles at {output}")


if __name__ == "__main__":
    main()
