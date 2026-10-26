#!/usr/bin/env python3
"""Probe whether the real boolean/``TopologyValidator`` independently agrees
with the generator-side thin-section predictor (``_generate_family``) that
full local-depth interdigitated teeth cannot fit on the real z=230mm post.

The predictor only estimates; this probe asks the validator directly,
bypassing it, by hand-building the ``WebIntent``s a full-depth attempt would
produce (same margin arithmetic as ``_generate_family``'s full-depth branch --
``local_upper/lower_worst_case``, ``corrected_skin_margin_mm``,
``FULL_DEPTH_MARGIN_HEADROOM_MM``) and handing them straight to
``SlotBuilder``/``TopologyValidator.validate`` without ever calling
``_generate_family`` (so its thin-section check never runs).

Measured result (written to the output JSON): the validator rejects the same
deep teeth the predictor rejects, by a comparable margin, so the predictor is
not merely conservative for this geometry. This is the evidence behind the
"MEASURED COUNTEREXAMPLE" comment in ``StructureGenerator._generate_feature``:
the dense attempt never wins the never-worse comparison on this post.

WRITES: a new result JSON (see ``--out``); no existing file is modified.

Usage:
    python tools/m36_d017_bypass_probe.py
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

from aircraft_core.frame import (  # noqa: E402
    LayerFrame, corrected_skin_margin_mm, estimate_rotation_deg,
    local_lower_worst_case, local_upper_worst_case,
)
from aircraft_core.generator import (  # noqa: E402
    FULL_DEPTH_MARGIN_HEADROOM_MM, WebIntent,
)
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.spec import StructureSpec  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

from m18_mesh_probe import read_binary_stl  # noqa: E402
from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402
from m34_rotation_equivalence import (  # noqa: E402
    _detect_theta, _drop_to_bed, _mesh_bbox_center,
)
from aircraft_core.alignment import rotate_points_deg  # noqa: E402
from m36_dense_lattice_sweep import _stand_up  # noqa: E402

# Real wing STL that is not part of this repository; pass --stl or set
# AIRCRAFT_WING_STL.
DEFAULT_STL = os.environ.get("AIRCRAFT_WING_STL", "")
WALL_MM = 0.42
DEFAULT_Z = 230.0
DEFAULT_PROBE_WIDTH_MM = 0.5
SKIN_MARGIN_MM = 0.8608


def canonical_polygon(stl_path: str, z: float):
    verts, tris = read_binary_stl(stl_path)
    verts = _stand_up(verts)
    verts = _drop_to_bed(verts)
    cx, cy = _mesh_bbox_center(verts)
    centered = [(v[0] - cx, v[1] - cy, v[2]) for v in verts]
    from aircraft_core.mesh_section import MeshSectioner
    probe = MeshSectioner(centered, tris)
    theta_detected, _detail = _detect_theta(probe)
    canonical_verts = centered
    if theta_detected is not None and abs(theta_detected) >= 1e-4:
        xy = rotate_points_deg([(v[0], v[1]) for v in centered], -theta_detected)
        canonical_verts = [(x, y, v[2]) for (x, y), v in zip(xy, centered)]
    sec = MeshSectioner(canonical_verts, tris)
    loops = sec.section_at(z)
    outer = [lp for lp in loops if not lp.is_interior]
    best = max(outer, key=lambda lp: abs(lp.signed_area))
    return best.points, theta_detected


def build_full_depth_intents(ops, poly, frame, pitch_mm, width_mm, skin_margin_mm):
    """Hand-built full-local-depth WebIntents at the SAME station layout
    ``_generate_feature_core``'s dense branch uses (top family at centre,
    bottom family offset by one pitch, both interleaved) -- but computed
    directly here via ``local_upper/lower_worst_case``, NEVER through
    ``_generate_family`` (so own thin-section check never runs)."""
    rotation_deg = estimate_rotation_deg(ops, poly)
    effective_margin_mm = corrected_skin_margin_mm(skin_margin_mm, rotation_deg)
    center_x = (frame.min_x + frame.max_x) / 2.0

    top_stations = [center_x - pitch_mm, center_x, center_x + pitch_mm]
    bottom_stations = [center_x - pitch_mm / 2.0, center_x + pitch_mm / 2.0,
                        center_x + 1.5 * pitch_mm]

    intents: List[WebIntent] = []
    stations_log = []
    for x in top_stations:
        y_hi = local_upper_worst_case(ops, poly, x, width_mm, DEFAULT_PROBE_WIDTH_MM)
        y_lo = local_lower_worst_case(ops, poly, x, width_mm, DEFAULT_PROBE_WIDTH_MM)
        local_thickness = y_hi - y_lo
        end_y = (y_hi - effective_margin_mm) - FULL_DEPTH_MARGIN_HEADROOM_MM
        depth = end_y - y_lo
        stations_log.append({"side": "top", "x_mm": round(x, 4),
                              "local_thickness_mm": round(local_thickness, 4),
                              "depth_mm": round(depth, 4)})
        if depth <= 0:
            continue
        intents.append(WebIntent(
            start=(x, y_hi), end=(x, end_y), width_mm=width_mm,
            family_index=0, family_name="feature_lattice_top", entry_lower=False,
        ))
    for x in bottom_stations:
        y_hi = local_upper_worst_case(ops, poly, x, width_mm, DEFAULT_PROBE_WIDTH_MM)
        y_lo = local_lower_worst_case(ops, poly, x, width_mm, DEFAULT_PROBE_WIDTH_MM)
        local_thickness = y_hi - y_lo
        # Mirror of the top-station formula: a -Y (bottom) entry starts at
        # `y_lo` and cuts UPWARD, so its far end approaches the UPPER
        # boundary minus margin -- `end_y = y_hi - margin - headroom`, and
        # the actual traversed depth is `end_y - y_lo`, NOT `y_hi - end_y`
        # (a different, larger quantity: using it would build a barely-cut,
        # near-`y_lo` slot while logging an unrelated "depth" and silently
        # pass validation by cutting almost nothing).
        end_y = (y_hi - effective_margin_mm) - FULL_DEPTH_MARGIN_HEADROOM_MM
        depth = end_y - y_lo
        stations_log.append({"side": "bottom", "x_mm": round(x, 4),
                              "local_thickness_mm": round(local_thickness, 4),
                              "depth_mm": round(depth, 4)})
        if depth <= 0:
            continue
        intents.append(WebIntent(
            start=(x, y_lo), end=(x, end_y), width_mm=width_mm,
            family_index=1, family_name="feature_lattice_bottom", entry_lower=True,
        ))
    return intents, stations_log, rotation_deg, effective_margin_mm


def main(argv=None) -> int:
    if not HAVE_SHAPELY:
        raise SystemExit("shapely unavailable; refusing to report")
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default=DEFAULT_STL)
    ap.add_argument("--z", type=float, default=DEFAULT_Z)
    ap.add_argument("--pitch", type=float, default=2.0)
    ap.add_argument("--out", default=os.path.join(
        REPO_ROOT, "docs", "results", "m36_d017_bypass_probe.json"))
    args = ap.parse_args(argv)

    if not os.path.exists(args.stl):
        raise SystemExit(
            "STL not found at %r (pass --stl or set AIRCRAFT_WING_STL)" % args.stl)

    points, theta_detected = canonical_polygon(args.stl, args.z)
    ops = MiterShapelyOps()
    poly = ops.polygon_from_points(points)
    frame = LayerFrame.from_polygon(ops, poly)

    width_mm = 0.21  # resolved slot width at gap_ratio 0.5, wall 0.42mm
    intents, stations_log, rotation_deg, effective_margin_mm = build_full_depth_intents(
        ops, poly, frame, args.pitch, width_mm, SKIN_MARGIN_MM,
    )

    spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=SKIN_MARGIN_MM,
                          root_solid_layers=0, web_position=0.5)
    builder = SlotBuilder()
    built = [builder.build(i, spec.overshoot_mm) for i in intents]
    slots = [ops.polygon_from_points(b.points) for b in built]
    transformed = ops.diff(poly, slots)
    validator = TopologyValidator(ops, WALL_MM)
    one_loop, err = True, None
    try:
        validator.validate(
            [poly], transformed, slots, spec, 0, args.z,
            slot_labels=[i.family_name for i in intents],
            family_indices=[(i.family_index,) for i in intents],
        )
    except TopologyError as exc:
        one_loop, err = False, str(exc)

    out: Dict[str, Any] = {
        "stl": os.path.basename(args.stl),
        "z_mm": args.z,
        "pitch_mm": args.pitch,
        "theta_detected_deg": theta_detected,
        "frame_width_mm": round(frame.width, 4),
        "frame_depth_mm": round(frame.depth, 4),
        "skin_margin_mm": SKIN_MARGIN_MM,
        "rotation_deg": round(rotation_deg, 4),
        "effective_margin_mm": round(effective_margin_mm, 4),
        "candidate_stations": stations_log,
        "teeth_built": len(intents),
        "d017_bypassed": True,
        "validator_one_closed_loop": one_loop,
        "validator_error": err,
        "finding": (
            "the REAL validator, with the thin-section check entirely bypassed, still "
            "REJECTS full local-depth teeth on this post -- the thin-section check was "
            "RIGHT, not merely conservative, for this geometry."
            if not one_loop else
            "the REAL validator, with the thin-section check bypassed, ACCEPTS full "
            "local-depth teeth here -- the thin-section check is over-conservative for this "
            "geometry (a genuine finding, contradicting the "
            "m36_dense_lattice_sweep.json result at this pitch if so; "
            "reconcile before citing)."
        ),
    }
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)

    print("THIN-SECTION BYPASS PROBE")
    print("  %s" % out["finding"])
    print("  wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
