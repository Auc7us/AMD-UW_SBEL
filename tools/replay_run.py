#!/usr/bin/env python3
"""Experimental high-throughput backend for replay_run_streaming.py.

The command line and replay semantics are inherited from replay_run_streaming.py, but
dynamic bodies are submitted as vtkGlyph3DMapper instance batches instead of one VTK
actor per recorded body.  The original script is deliberately left untouched while this
backend is profiled and validated.

Every recorded body is still drawn, including track shoes, wheels, suspension, rocks,
and articulated links.  Bodies which share the same already-placed visual geometry
become instances of one prototype, with colour retained per body.  Their position,
quaternion, and live/dead scale are updated as compact numpy arrays once per frame.
"""

from __future__ import annotations

import collections
import concurrent.futures
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_run_streaming as base  # noqa: E402


_ORIGINAL_BUILD_SCENE = base.build_scene
_ORIGINAL_BUILD_RUT_LAYERS = base.build_rut_layers

# The recording names Polaris_tire.obj because Polaris_LuggedTire.json deliberately uses
# the smooth stock tyre for visualization even though SCM collides against the lugged
# mesh.  That visual is both misleading and expensive (7,988 triangles).  The project's
# metre-scaled lugged visual is 2,644 triangles and uses the same +Y spin axis.
_DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
_LUGGED_TIRE_VISUAL = os.path.join(
    _DATA_DIR, "vehicle", "LRV", "meshes", "LRVtire_red_m.obj")


def _render_shape(shape):
    """Apply experimental visual substitutions without changing recorded physics."""
    if (shape.get("type") == "trimesh"
            and os.path.basename(shape.get("file", "")) == "Polaris_tire.obj"):
        shape = dict(shape)
        shape["file"] = _LUGGED_TIRE_VISUAL
        # main.cpp hands this same metre-scaled asset directly to the VSG/Synchrono
        # visualization path.  Drop the stock Polaris_tire AABB so fit_to_aabb() does not
        # squash the VSG mesh back into the narrower smooth-tyre dimensions.
        shape.pop("aabb_min", None)
        shape.pop("aabb_max", None)
    return shape


def _colour(obj):
    return next((tuple(s["color"]) for s in obj.get("shapes", []) if s.get("color")),
                None) or base.group_color(obj)


def _prototype_key(obj, boxes_only):
    """Hashable description of geometry, excluding body pose and per-instance colour."""
    shapes = tuple(base.shape_signature(_render_shape(s), boxes_only)
                   for s in obj.get("shapes", []))
    # A body without recorded shapes falls back to body_bounds(), so its bounds are part
    # of the prototype.  Shaped bodies are fully described by shape_signature().
    fallback = None
    if not shapes:
        lo, hi = base.body_bounds(obj)
        fallback = tuple(round(float(x), 6) for x in (*lo, *hi))
    return shapes, fallback


def _merged_geometry(obj, cache, boxes_only):
    import pyvista as pv

    pieces = [base.shape_geometry(_render_shape(s), cache, boxes_only)
              for s in obj.get("shapes", [])]
    pieces = [p for p in pieces if p is not None and p.n_points]
    if not pieces:
        lo, hi = base.body_bounds(obj)
        pieces = [pv.Box(bounds=(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))]
    mesh = pieces[0] if len(pieces) == 1 else pieces[0].merge(pieces[1:])
    # add_mesh(..., smooth_shading=True) in the original builds point normals.  Do it
    # once per prototype here rather than once per body.
    try:
        return mesh.compute_normals(cell_normals=False, point_normals=True,
                                    split_vertices=False, inplace=False)
    except (TypeError, ValueError):
        return mesh


def build_instanced_bodies(pl, bodies, cache, boxes_only):
    """Return one multi-source glyph mapper containing every dynamic body.

    vtkGlyph3DMapper accepts a table of prototype meshes and a per-point source index.
    Keeping all recorded bodies in one point cloud reduces both Python update calls and
    renderer traversal.  VTK/OpenGL still group the instances by source internally.
    """
    import pyvista as pv
    import vtk

    prototypes = collections.OrderedDict()
    source_index = np.empty(len(bodies), dtype=np.int32)
    for body_index, obj in enumerate(bodies):
        key = _prototype_key(obj, boxes_only)
        entry = prototypes.get(key)
        if entry is None:
            entry = {"object": obj, "source_id": len(prototypes)}
            prototypes[key] = entry
        source_index[body_index] = entry["source_id"]

    count = len(bodies)
    cloud = pv.PolyData(np.zeros((count, 3), dtype=np.float32))
    cloud["orientation"] = np.tile(
        np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (count, 1))
    # vtkGlyph3DMapper's mask path requires vtkBitArray and is awkward to mutate from
    # numpy.  A zero scale is an equivalent, GPU-side visibility mask.
    cloud["instance_scale"] = np.zeros(count, dtype=np.float32)
    cloud["source_index"] = source_index
    cloud["instance_color"] = np.asarray(
        [(*pv.Color(_colour(obj)).int_rgb, 255) for obj in bodies], dtype=np.uint8)

    mapper = vtk.vtkGlyph3DMapper()
    mapper.SetInputData(cloud)
    sources = []
    for entry in prototypes.values():
        source = _merged_geometry(entry["object"], cache, boxes_only)
        sources.append(source)
        mapper.SetSourceData(entry["source_id"], source)
    mapper.SetSourceIndexArray("source_index")
    mapper.SourceIndexingOn()
    mapper.SetOrientationArray("orientation")
    mapper.SetOrientationModeToQuaternion()
    mapper.OrientOn()
    mapper.SetScaleArray("instance_scale")
    mapper.SetScaleModeToScaleByMagnitude()
    mapper.SetScaleFactor(1.0)
    mapper.ScalingOn()
    mapper.SetScalarModeToUsePointFieldData()
    mapper.SelectColorArray("instance_color")
    mapper.SetColorModeToDirectScalars()
    mapper.ScalarVisibilityOn()

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetInterpolationToPhong()
    actor.GetProperty().SetSpecular(0.25)
    pl.renderer.AddActor(actor)

    return {
        "cloud": cloud,
        "points": cloud.points,
        "orientations": cloud["orientation"],
        "scales": cloud["instance_scale"],
        "sources": sources,
        "mapper": mapper,
        "actor": actor,
        "prototype_count": len(prototypes),
    }


def build_scene(pl, bodies, cache, boxes_only, meta, terrain_decimate, scenery,
                terrain_radius, directory):
    """Instanced dynamic bodies plus the original, static terrain/scenery path."""
    instances = build_instanced_bodies(pl, bodies, cache, boxes_only)
    _unused, terrain, ground = _ORIGINAL_BUILD_SCENE(
        pl, [], cache, boxes_only, meta, terrain_decimate, scenery, terrain_radius,
        directory)
    print(f"instances   {len(bodies)} bodies, {instances['prototype_count']} prototypes, "
          "1 dynamic actor")
    replaced = sum(
        os.path.basename(shape.get("file", "")) == "Polaris_tire.obj"
        for obj in bodies for shape in obj.get("shapes", []))
    if replaced:
        print(f"tire visual {replaced} stock tires replaced by LRVtire_red_m.obj")
    return instances, terrain, ground


def frame_matrices(_instances):
    """Compatibility hook for base.main(); instances do not use per-actor matrices."""
    return None


def apply_frame(instances, poses, slot, _unused=None):
    """Upload one compact position/quaternion/scale array for all dynamic bodies."""
    row = poses[slot]
    live = np.isfinite(row[:, 0])

    q = row[:, 3:7].copy()
    norm = np.sqrt((q * q).sum(axis=1))
    bad = ~np.isfinite(norm) | (norm < 1e-12)
    norm[bad] = 1.0
    q /= norm[:, None]
    q[bad] = (1.0, 0.0, 0.0, 0.0)

    # Avoid placing NaNs in mapper bounds even for zero-scaled, not-yet-live rocks.
    points = instances["points"]
    points[:] = 0.0
    points[live] = row[live, :3]
    instances["orientations"][:] = q
    instances["scales"][:] = live

    cloud = instances["cloud"]
    cloud.GetPoints().GetData().Modified()
    cloud.GetPointData().GetArray("orientation").Modified()
    cloud.GetPointData().GetArray("instance_scale").Modified()


def _remove_dataset_actor(pl, dataset):
    """Remove the actor which the original rut builder attached for `dataset`."""
    removed = 0
    for actor in list(pl.renderer.actors.values()):
        mapper = actor.GetMapper() if hasattr(actor, "GetMapper") else None
        if mapper is None:
            continue
        try:
            # PyVista's DataSetMapper retains the exact Python dataset wrapper; GetInput()
            # creates another wrapper for the same VTK object and identity/equality both
            # fail even though the underlying pointer is shared.
            same = mapper.dataset is dataset
        except (AttributeError, TypeError):
            same = False
        if same:
            pl.remove_actor(actor, reset_camera=False, render=False)
            removed += 1
    return removed


def _chunk_mesh(pl, mesh, cmap, clim, target_faces):
    """Split a large dynamic quad mesh into independently uploadable render chunks."""
    import pyvista as pv

    faces = np.asarray(mesh.regular_faces)
    chunks = []
    for first in range(0, len(faces), target_faces):
        global_faces = faces[first:first + target_faces]
        global_ids, inverse = np.unique(global_faces, return_inverse=True)
        local_faces = inverse.reshape(global_faces.shape)
        local = pv.PolyData(
            np.asarray(mesh.points[global_ids], dtype=np.float32).copy(),
            np.column_stack((np.full(len(local_faces), 4, dtype=np.int64),
                             local_faces)).ravel())
        local.verts = np.empty(0, dtype=np.int64)
        local["sinkage"] = np.asarray(mesh["sinkage"][global_ids],
                                       dtype=np.float32).copy()
        actor = pl.add_mesh(local, scalars="sinkage", cmap=cmap, clim=clim,
                            show_scalar_bar=False, specular=0.05,
                            reset_camera=False, render=False)
        chunks.append({
            "global_ids": global_ids,
            "mesh": local,
            "points": local.points,
            "sinkage": local["sinkage"],
            "actor": actor,
        })
    return chunks


def build_rut_layers(pl, sources, cells, gx, gy, sample_base, ground, clim,
                     coarse_m=0.16, fine_m=0.04, budget=2_000_000):
    """Build the exact original rut surface, then spatially batch its GPU uploads.

    The original uses two enormous mutable meshes.  A single changed SCM node marks both
    datasets modified, making VTK rebuild buffers for 5.35 million points.  This backend
    keeps the same CPU-side topology and point mapping but renders face chunks, so an SCM
    sample invalidates only chunks containing points which actually changed.
    """
    patches, layers = _ORIGINAL_BUILD_RUT_LAYERS(
        pl, sources, cells, gx, gy, sample_base, ground, clim, coarse_m, fine_m,
        budget)
    if layers is None:
        return patches, layers

    coarse, fine = layers["coarse"], layers["fine"]
    removed = _remove_dataset_actor(pl, coarse) + _remove_dataset_actor(pl, fine)
    if removed != 2:
        print(f"  ! expected to replace 2 monolithic rut actors, found {removed}",
              file=sys.stderr)

    target_faces = max(4096, int(os.environ.get("REPLAY_SCM_CHUNK_FACES", "32768")))
    cmap = base.rut_colormap(ground)
    fine_chunks = _chunk_mesh(pl, fine, cmap, (0.0, max(1e-3, clim)), target_faces)
    coarse_chunks = _chunk_mesh(pl, coarse, cmap, (0.0, max(1e-3, clim)), target_faces)

    # CPU-authoritative dynamic fields.  The large original PolyData objects retain the
    # topology used to create chunks, but are no longer connected to a renderer.
    layers.update({
        "fine_z": np.asarray(fine.points[:, 2], dtype=np.float32).copy(),
        "fine_sinkage": np.asarray(fine["sinkage"], dtype=np.float32).copy(),
        "coarse_z": np.asarray(coarse.points[:, 2], dtype=np.float32).copy(),
        "coarse_sinkage": np.asarray(coarse["sinkage"], dtype=np.float32).copy(),
        "fine_chunks": fine_chunks,
        "coarse_chunks": coarse_chunks,
        "scm_pool": concurrent.futures.ThreadPoolExecutor(
            max_workers=max(1, min(len(patches), os.cpu_count() or 4)),
            thread_name_prefix="scm-decode"),
        "dirty_chunks": collections.OrderedDict(),
        "upload_chunks_per_frame": max(
            1, int(os.environ.get("REPLAY_SCM_UPLOAD_CHUNKS", "8"))),
    })
    print(f"scm chunks  {len(fine_chunks)} fine + {len(coarse_chunks)} coarse "
          f"(up to {target_faces} faces each)")
    return patches, layers


def _consume_patch_until(item):
    """Worker-thread side of SCM playback: read, filter, and sort one rank."""
    patch, t = item
    stream = patch["stream"]
    frames = []
    while stream.has_next() and stream.peek_time() <= t:
        _frame_time, node_ids, z = stream.next()
        if not len(node_ids):
            continue
        order = np.argsort(-z, kind="stable")
        frames.append((node_ids[order], z[order]))
    return frames


def _queue_chunks(layers, kind, changed, all_points=False):
    """Put touched chunks in a deduplicating FIFO; data is copied when they drain."""
    chunks = layers[f"{kind}_chunks"]
    changed = None if all_points else np.unique(np.concatenate(changed))
    for chunk_id, chunk in enumerate(chunks):
        global_ids = chunk["global_ids"]
        touched = changed is None
        if changed is not None:
            pos = np.searchsorted(global_ids, changed)
            valid = pos < len(global_ids)
            pos = pos[valid]
            selected = changed[valid]
            touched = bool(len(pos) and np.any(global_ids[pos] == selected))
        if touched:
            layers["dirty_chunks"].setdefault((kind, chunk_id), chunk)


def _drain_chunks(layers, limit=None):
    """Upload complete dirty chunks, each from the newest CPU-authoritative values."""
    queue = layers["dirty_chunks"]
    count = len(queue) if limit is None else min(limit, len(queue))
    for _ in range(count):
        (kind, _chunk_id), chunk = queue.popitem(last=False)
        global_ids = chunk["global_ids"]
        chunk["points"][:, 2] = layers[f"{kind}_z"][global_ids]
        chunk["sinkage"][:] = layers[f"{kind}_sinkage"][global_ids]
        chunk["mesh"].GetPoints().GetData().Modified()
        chunk["mesh"].GetPointData().GetArray("sinkage").Modified()
    return count


def apply_scm(patches, layers, t):
    """Parallel SCM decode plus dirty-chunk uploads, preserving original semantics."""
    if layers is None:
        return

    previous = max((patch["time"] for patch in patches), default=-1.0)
    first = previous < 0.0
    rewound = any(t < patch["time"] for patch in patches)
    if rewound:
        layers["coarse_z"][:] = layers["coarse_base"]
        layers["fine_z"][:] = layers["fine_base"]
        layers["coarse_sinkage"][:] = 0.0
        layers["fine_sinkage"][:] = 0.0
        for patch in patches:
            patch["stream"].reset()
            patch["time"] = -1.0

    due = [patch for patch in patches
           if patch["stream"].has_next() and patch["stream"].peek_time() <= t]
    decoded = layers["scm_pool"].map(
        _consume_patch_until, ((patch, t) for patch in due))

    fine_changed = []
    coarse_changed = []
    for patch, frames in zip(due, decoded):
        for node_ids, z in frames:
            fine_ids = patch["fp"][node_ids]
            layers["fine_z"][fine_ids] = z
            layers["fine_sinkage"][fine_ids] = patch["fb"][node_ids] - z
            fine_changed.append(fine_ids)

            hit = patch["hit"][node_ids]
            if hit.any():
                coarse_nodes = node_ids[hit]
                coarse_ids = patch["cp"][coarse_nodes]
                layers["coarse_z"][coarse_ids] = z[hit]
                layers["coarse_sinkage"][coarse_ids] = patch["cb"][coarse_nodes] - z[hit]
                coarse_changed.append(coarse_ids)

    for patch in patches:
        patch["time"] = t

    if rewound or fine_changed:
        _queue_chunks(layers, "fine", fine_changed, all_points=rewound)
    if rewound or coarse_changed:
        _queue_chunks(layers, "coarse", coarse_changed, all_points=rewound)

    # An interactive 30 Hz replay has three display frames between 10 Hz SCM samples.
    # Spread buffer uploads across those frames so one terrain sample cannot monopolise
    # the render thread.  Explicit seeks and offline output are flushed immediately for
    # exact single-frame/movie results.
    offline = "--shot" in sys.argv or "--movie" in sys.argv
    jumped = previous >= 0.0 and abs(t - previous) > 0.5
    limit = None if first or rewound or jumped or offline else layers["upload_chunks_per_frame"]
    _drain_chunks(layers, limit)


# Patch only this process.  The original module and file remain the reference backend,
# while its CLI, terrain reconstruction, SCM streaming, controls, movie output, and
# camera behavior are reused unchanged.
base.build_scene = build_scene
base.frame_matrices = frame_matrices
base.apply_frame = apply_frame
base.build_rut_layers = build_rut_layers
base.apply_scm = apply_scm


def main():
    """Run the inherited CLI without VTK's conflicting built-in `r` shortcut."""
    import pyvista as pv

    original_show = pv.Plotter.show

    def show_without_vtk_char_shortcuts(plotter, *args, **kwargs):
        # The replay handles its documented keyboard controls through PyVista's explicit
        # KeyPressEvent callbacks.  VTK's lower-level CharEvent handler also interprets
        # `r` as reset-camera-to-all-bounds; with the kilometre-wide terrain that fires
        # after show(0) and zooms far out.  Follow mode hid it by continuously reasserting
        # its own camera.  Removing the built-in character shortcuts leaves the replay's
        # callbacks (including r, c, movement, zoom, and q) intact.
        if plotter.iren is not None and plotter.iren.interactor is not None:
            plotter.iren.interactor.RemoveObservers("CharEvent")
        return original_show(plotter, *args, **kwargs)

    pv.Plotter.show = show_without_vtk_char_shortcuts
    try:
        base.main()
    finally:
        pv.Plotter.show = original_show


if __name__ == "__main__":
    main()
