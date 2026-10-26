#!/usr/bin/env python3
"""Measure ``feature_dense_lattice_enabled`` against the shipped (``False``)
bound, with the never-worse wrapper (``StructureGenerator._generate_feature``)
in place, on two fixtures:

  1. the real z=230mm post, sectioned from the wing STL through the same
     stand-up + chord-canonicalisation pipeline
     ``tools/m34_rotation_equivalence.py`` uses (helpers imported from
     there, not re-derived).
  2. A synthetic 8.64x8.67mm square post -- the shape class the dense
     attempt actually helps (flat far surface across widened
     thin-section probe window), used as the positive control.

Every row goes through the REAL pipeline: ``StructureGenerator.generate``,
``SlotBuilder``, the real ``shapely``-backed boolean diff, and
``TopologyValidator.validate`` -- the same real-boolean harness
``tests/unit/test_m36_feature_classification.py`` uses.

WRITES: a new result JSON (see ``--out``); no existing file is modified.

Usage:
    python tools/m36_dense_lattice_sweep.py
    python tools/m36_dense_lattice_sweep.py --stl "<path to STL>" --z 230.0
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO_ROOT, os.path.join(REPO_ROOT, "tools"), os.path.join(REPO_ROOT, "tests", "unit")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import LayerContext, StructureGenerator  # noqa: E402
from aircraft_core.mesh_section import MeshSectioner  # noqa: E402
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.spec import StructureSpec  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

from m18_mesh_probe import read_binary_stl  # noqa: E402
from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402
from m34_rotation_equivalence import (  # noqa: E402
    _detect_theta, _drop_to_bed, _mesh_bbox_center,
)
from aircraft_core.alignment import rotate_points_deg  # noqa: E402

# Real wing STL that is not part of this repository; pass --stl or set
# AIRCRAFT_WING_STL.
DEFAULT_STL = os.environ.get("AIRCRAFT_WING_STL", "")
WALL_MM = 0.42
DEFAULT_Z = 230.0
PITCHES = (3.5, 3.0, 2.5, 2.0, 1.75, 1.5, 1.25, 1.0, 0.9, 0.85, 0.8, 0.75, 0.7, 0.65, 0.6)


def _stand_up(verts):
    """As downloaded, ``Part Studio 2 - Part 1(6).stl`` lies with its span along X
    (range 249.02.484.36mm, i.e. 235.34mm -- exactly's post-stand-up Z span)
    and only ~28.5mm along raw Z (its chord-ish thickness at the root). Swap X
    and Z so the span runs vertical, matching that doc's "stood up (span
    vertical, root on the bed)" -- a fixed axis swap MEASURED against this
    specific file's own raw bounding box, not a data-fit rotation;
    ``_detect_theta`` (imported below) supplies the real, non-90 -degree chord
    rotation on top of this."""
    return [(v[2], v[1], v[0]) for v in verts]


def canonical_sectioner(stl_path: str, already_canonical: bool = False):
    """Build a ``MeshSectioner`` for ``stl_path``.

    ``already_canonical=False`` (default): applies the full stand-up +
    bed-drop + bbox-centre + chord-canonicalisation pipeline, as a raw wing
    STL requires.

    ``already_canonical=True``: skips ALL of that -- no axis swap, no
    bed-drop, no re-centring, no theta rotation -- and sections the mesh's
    own vertices exactly as stored. For an STL that was produced by hand
    already standing on its root face in the print frame (z = 0..print
    height) and already centred/axis-aligned (e.g. a squared-post variant
    built by moving only the post's own vertices onto a square in that same
    frame), applying the stand-up pipeline a second time would double-
    transform it. Returns ``theta_detected=None``, ``rotation_applied=False``
    to make that skip visible in any report built from this return value.
    """
    verts, tris = read_binary_stl(stl_path)
    if already_canonical:
        sec = MeshSectioner(verts, tris)
        return sec, None, False, {"skipped": "already_canonical=True"}
    verts = _stand_up(verts)
    verts = _drop_to_bed(verts)
    cx, cy = _mesh_bbox_center(verts)
    centered = [(v[0] - cx, v[1] - cy, v[2]) for v in verts]
    probe = MeshSectioner(centered, tris)
    theta_detected, detail = _detect_theta(probe)
    canonical_verts = centered
    rotation_applied = False
    if theta_detected is not None and abs(theta_detected) >= 1e-4:
        xy = rotate_points_deg([(v[0], v[1]) for v in centered], -theta_detected)
        canonical_verts = [(x, y, v[2]) for (x, y), v in zip(xy, centered)]
        rotation_applied = True
    sec = MeshSectioner(canonical_verts, tris)
    return sec, theta_detected, rotation_applied, detail


def _rect_points(w, h):
    hw, hh = w / 2.0, h / 2.0
    return ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh))


def run_one(ops, poly, spec: StructureSpec, layer_index=0, print_z=0.0) -> Dict[str, Any]:
    frame = LayerFrame.from_polygon(ops, poly)
    gen = StructureGenerator()
    ctx = LayerContext(layer_index=layer_index, print_z=print_z)
    result = gen.generate(frame, spec, WALL_MM, ctx, safety_offset_mm=WALL_MM)
    builder = SlotBuilder()
    built = [builder.build(i, spec.overshoot_mm) for i in result.intents]
    slots = [ops.polygon_from_points(b.points) for b in built]
    transformed = ops.diff(poly, slots)
    validator = TopologyValidator(ops, WALL_MM)
    one_loop, err = True, None
    try:
        validator.validate(
            [poly], transformed, slots, spec, layer_index, print_z,
            slot_labels=[str(i.family_name) for i in result.intents],
            family_indices=[i.family_membership() for i in result.intents],
        )
    except TopologyError as exc:
        one_loop, err = False, str(exc)
    depths = [i.depth_mm() for i in result.intents]
    fallback_traces = [
        o for o in result.omitted
        if str(o.family_name).startswith("feature_dense_fallback")
    ]
    return {
        "frame_width_mm": round(frame.width, 4),
        "frame_depth_mm": round(frame.depth, 4),
        "teeth": len(result.intents),
        "max_depth_mm": round(max(depths), 4) if depths else 0.0,
        "one_closed_loop": one_loop,
        "validate_error": err,
        "never_worse_fallback_fired": len(fallback_traces) > 0,
        "degradations": len(result.degradations),
    }


def sweep_kai_post(stl_path: str, z: float, already_canonical: bool = False) -> Dict[str, Any]:
    sec, theta_detected, rotation_applied, detail = canonical_sectioner(
        stl_path, already_canonical=already_canonical)
    loops = sec.section_at(z)
    outer = [lp for lp in loops if not lp.is_interior]
    if not outer:
        return {"error": "no outer loop at z=%.4f" % z, "theta_detail": detail}
    best = max(outer, key=lambda lp: abs(lp.signed_area))
    ops = MiterShapelyOps()
    poly = ops.polygon_from_points(best.points)

    # z_drift_mm_per_mm=100.0 forces the ORDINARY (single, implicit)
    # aerofoil family's anchor 100mm off this small feature's own chord
    # extent at ANY print_z > 0, so its one candidate is always omitted
    # (chord-extent miss) and `generate()`'s zero-intent fallback routes to
    # `_generate_feature` -- the mechanism `_generate_feature`'s module
    # docstring documents, and the spec shape used by
    # `tests/unit/test_m36_feature_classification.py` and
    # `tests/unit/test_m36_dense_feature_lattice.py`. Without it, the
    # aerofoil family's default single full-depth web at web_position=0.5
    # fits an 8.6mm square/post on its own, `_generate_feature` is never
    # reached, and the sweep would measure the wrong code path (a plain
    # 1-tooth intent that ignores feature_dense_lattice_enabled).
    base_kwargs = dict(
        gap_ratio=0.5, skin_margin_mm=0.8608, root_solid_layers=0,
        web_position=0.5, z_drift_mm_per_mm=100.0,
    )
    rows = []
    for pitch in PITCHES:
        row = {"pitch_mm": pitch}
        for dense in (False, True):
            spec = StructureSpec(
                feature_lattice_pitch_mm=pitch, feature_dense_lattice_enabled=dense,
                **base_kwargs
            )
            row["dense" if dense else "shipped"] = run_one(ops, poly, spec, print_z=z)
        row["dense_wins"] = (
            row["dense"]["teeth"] > row["shipped"]["teeth"]
            or (row["dense"]["teeth"] == row["shipped"]["teeth"]
                and row["dense"]["max_depth_mm"] > row["shipped"]["max_depth_mm"])
        )
        rows.append(row)

    any_pitch_wins = any(r["dense_wins"] for r in rows)
    return {
        "stl": os.path.basename(stl_path),
        "z_mm": z,
        "theta_detected_deg": theta_detected,
        "rotation_applied": rotation_applied,
        "pitches_swept": list(PITCHES),
        "rows": rows,
        "any_pitch_where_dense_wins": any_pitch_wins,
        "finding": (
            "dense mode wins at at least one pitch" if any_pitch_wins else
            "with the never-worse rule in place, feature_dense_lattice_enabled "
            "produces ZERO improvement over the shipped bound at ANY of the "
            "{} pitches swept on this post -- it always silently falls back "
            "to exactly the shipped result at that pitch. The only lever "
            "that still changes this post's stiffness is "
            "feature_lattice_pitch_mm itself.".format(len(PITCHES))
        ),
    }


def sweep_square_post() -> Dict[str, Any]:
    ops = MiterShapelyOps()
    poly = ops.polygon_from_points(_rect_points(8.64, 8.67))
    # z_drift_mm_per_mm=100.0 forces the ORDINARY (single, implicit)
    # aerofoil family's anchor 100mm off this small feature's own chord
    # extent at ANY print_z > 0, so its one candidate is always omitted
    # (chord-extent miss) and `generate()`'s zero-intent fallback routes to
    # `_generate_feature` -- the mechanism `_generate_feature`'s module
    # docstring documents, and the spec shape used by
    # `tests/unit/test_m36_feature_classification.py` and
    # `tests/unit/test_m36_dense_feature_lattice.py`. Without it, the
    # aerofoil family's default single full-depth web at web_position=0.5
    # fits an 8.6mm square/post on its own, `_generate_feature` is never
    # reached, and the sweep would measure the wrong code path (a plain
    # 1-tooth intent that ignores feature_dense_lattice_enabled).
    base_kwargs = dict(
        gap_ratio=0.5, skin_margin_mm=0.8608, root_solid_layers=0,
        web_position=0.5, z_drift_mm_per_mm=100.0,
    )
    rows = []
    for pitch in (2.0, 1.5, 1.0, 0.9):
        row = {"pitch_mm": pitch}
        for dense in (False, True):
            spec = StructureSpec(
                feature_lattice_pitch_mm=pitch, feature_dense_lattice_enabled=dense,
                **base_kwargs
            )
            row["dense" if dense else "shipped"] = run_one(ops, poly, spec, print_z=1.0)
        rows.append(row)
    return {"fixture": "square post 8.64x8.67mm (no chamfer)", "rows": rows}


def main(argv=None) -> int:
    if not HAVE_SHAPELY:
        raise SystemExit("shapely unavailable; refusing to report")
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default=DEFAULT_STL)
    ap.add_argument("--z", type=float, default=DEFAULT_Z)
    ap.add_argument(
        "--already-canonical", action="store_true", default=False,
        help=(
            "Skip the stand-up + bed-drop + re-centre + theta-rotation "
            "pipeline and section --stl's own vertices as stored. Use only "
            "for an STL already produced standing on its root face in the "
            "print frame and already axis-aligned/centred -- feeding a raw "
            "downloaded STL with this flag set will section it in the wrong "
            "frame. Default (unset) is byte-identical to before this flag "
            "existed."))
    ap.add_argument("--out", default=os.path.join(
        REPO_ROOT, "docs", "results", "m36_dense_lattice_sweep.json"))
    args = ap.parse_args(argv)

    out: Dict[str, Any] = {"wall_mm": WALL_MM}
    if os.path.exists(args.stl):
        out["kai_post"] = sweep_kai_post(args.stl, args.z, already_canonical=args.already_canonical)
    else:
        out["kai_post"] = {"error": "STL not found at %r -- skipped" % args.stl}
    out["square_post"] = sweep_square_post()

    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)

    print("DENSE LATTICE SWEEP")
    if "error" in out["kai_post"]:
        print("  wing post: %s" % out["kai_post"]["error"])
    else:
        print("  wing post (%s, z=%.1f): %s" % (
            out["kai_post"]["stl"], args.z, out["kai_post"]["finding"]))
    print("  wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
