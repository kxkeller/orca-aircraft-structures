#!/usr/bin/env python3
"""Pre-flight: report what a new wing will do BEFORE a 12-minute GUI slice.

One command takes a new STL and the live plugin config and reports, in
plain language, which layers would fail or lose structure and why -- using
the REAL ``aircraft_core`` pipeline (``MeshSectioner`` ->
``StructureGenerator`` -> ``SlotBuilder`` -> ``GeometryOps.diff`` ->
``TopologyValidator``), never a stub or an approximation of it. This is
the SAME sectioner class ``plugin/aircraft_structures/capability.py``
constructs in-host (``_build_mesh_sectioner``), so calling it directly here
is reuse, not a fork (``aircraft_core`` has no Orca imports and is directly
callable offline).

FOUR REPORT TIERS, in this order (each is printed and also written to
``--json`` if given):

  T1 MODEL-LEVEL: triangle count, closed/two-manifold by exact binary-float
     edge counting, bounding box, connected-shell count. Fires once per run,
     before any layer exists (the Class A input sites).
  T2 ORIENTATION: the shipped chord-axis detection
     (``aircraft_core.alignment``), run exactly the way
     ``capability.py._build_mesh_sectioner`` runs it (same span-fraction
     stations: 0.1, 0.3, 0.5 [mid, authoritative], 0.7, 0.9). THIS IS THE
     HIGHEST-VALUE LINE IN THE TOOL. Per the alignment rule in
     ``aircraft_core/alignment.py``, a blunt/flat-cut AEROFOIL section (>=
     ``AEROFOIL_MIN_HULL_VERTICES`` hull vertices) FAILS CLOSED in-host
     with ``BluntTrailingEdgeError`` -- this tool reports that fatal
     refusal before the user spends 12 minutes finding it live. A blunt section
     with FEWER hull vertices is a structural coupon and ships UNROTATED,
     silently, in-host (``ChordAxisUnavailableError``, non-fatal) -- this
     tool names that residual exposure explicitly rather than staying
     quiet about it.
  T3 PER-LAYER: for every body layer, would it REFUSE (exception + message
     + layer index + Z), DEGRADE (which structure is lost, under the
     degrade-by-default policy -- see
     ``aircraft_core.diagnostics.RunDiagnostics``, reused directly here for
     exact field parity with a real in-host report), or pass clean. Root/mid/tip span-fraction
     breakdown included.
  T4 BLIND SPOT, printed UNCONDITIONALLY, never suppressed by a clean
     result: the mesh-alignment guard (``capability.py``'s
     ``_check_mesh_alignment``, six terms) compares OUR section against the
     HOST'S OWN libslic3r slice. There is no host slice offline, so a
     refusal raised by that guard is structurally INVISIBLE to this tool.
     This tool never prints an unqualified "PASS".

TWO CONFOUNDS THIS TOOL MUST NOT BLUR:

  STAND-UP AXIS AMBIGUITY. Offline, there is no GUI placement to read, so
  ``--up-axis auto`` (default) tries all three candidate axes, sections each at
  its own mid-extent, and picks the axis on which the SHIPPED chord-axis
  detector actually finds a chord axis -- never the longest bounding-box axis
  blindly (a wrong stand-up axis produces ~90-degree planform corners, which
  reads exactly like a blunt trailing edge, and the two must not be confused).
  If zero axes resolve a chord axis, this tool falls back to the
  longest-bbox-axis heuristic and says LOW CONFIDENCE, explicitly declining to
  blame either a blunt TE or a wrong axis. If more than one axis resolves a
  chord axis, it reports the ambiguity and REFUSES to auto-select -- pass
  ``--up-axis`` explicitly. Independently, this tool always warns when the two
  longest raw bounding-box extents are within ``AMBIGUOUS_EXTENT_RATIO`` of
  each other (a real wing can measure ~215 mm chord against ~225 mm span, so
  "longest axis is the span" is not a safe assumption).

  ROOT-END AMBIGUITY. Standing up the span axis does not say which END
  sits on the bed. ``--root-end {low,high,auto}`` (default auto) compares
  cross-sectional area near each Z extreme after stand-up and places the
  larger-area end at the bed (root sections are normally the thicker,
  wider end) -- printed and stated as an assumption, not hidden, since
  ``root_solid_layers``/tip-shutdown/the root-mid-tip breakdown all invert
  if this is wrong.

FIDELITY, STATED HONESTLY: this tool does not reimplement ``capability.py``'s
spar-socket latch, multi-island partial structuring, or order-swap
classification (``_classify_declared_pair_overlaps``) -- those stay in the
plugin. ``order_swap_count`` is always reported as 0 here with this limitation
named, not silently guessed. Every other summary field (``layers_seen``,
``mutated_count``, ``failed_count``, ``shutdown_count``,
``degraded_layer_count``, ``node_merge_count`` and their first-Z companions,
``chord_axis_unavailable``) is produced by
``aircraft_core.diagnostics.RunDiagnostics`` -- the EXACT class
``capability.py`` accumulates into for a real report -- so those fields are
directly comparable to an in-host report, not a re-derived approximation.

RE-ENTRANT SECTIONS: a re-entrant section (``ReEntrantSectionError``, raised
when the chord probe crosses material more than twice, so "local thickness"
is not a well-defined pair of surfaces at any station) is caught by type and
recorded as a DEGRADED/shutdown layer, mirroring ``capability.py``'s
``except ReEntrantSectionError``. It must not fall into the generic per-layer
catch-all (``ReEntrantSectionError`` is a ``ValueError`` subclass), which
would count every re-entrant layer on a real wing as a hard FAILURE.

NEVER WEAKENS ANYTHING TO MAKE A WING PASS. A refusal found here is
reported as a refusal, not massaged.

Usage:
    python tools/preflight_wing.py WING.stl [--config PATH] [--up-axis auto]
        [--root-end auto] [--layer-height 0.2] [--first-layer-height H]
        [--wall-width-mm 0.42] [--json OUT.json]
        [--compare-report HOST_REPORT.json] [--keep-placement]

    --keep-placement: do NOT flip the part so its root is at the bed. The mesh
    is taken as PLACED (root at the bed OR at the top, e.g. the tip-face-down
    outside elevon half); --root-end low/high then means "the root is the
    low/high Z end" and only labels the placement, and the resolved root end
    (area + spar-channel signals, or the override) is reported and drives the
    spar-socket latch walk summary.

EXIT CODES (binding contract -- kept in sync with the final status block in
``main`` and the argparse epilog):

    0  clean: no refusals, no degradation (the T4 caveat still prints)
    1  refusals found -- at least one layer the host would refuse
    2  tool-level refusal -- unreadable STL, bad config, unusable input
    3  stand-up not resolved confidently -- covers BOTH the ambiguous case
       (more than one candidate axis resolves a chord axis) and the
       low-confidence fallback (no candidate axis resolves a chord axis,
       AND the chosen fallback axis's own T2 reading is fatal/unavailable
       -- ``apply_stand_up_confound``). ``result["status"]`` distinguishes
       ``STAND_UP_AMBIGUOUS`` from ``UNRESOLVED_STAND_UP``; the exit code
       does not need to.
    4  structure lost (degradation) but no refusals

Precedence when several would apply: 2 > 3 > 1 > 4 > 0.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS_DIR = os.path.join(REPO_ROOT, "tools")
for _p in (REPO_ROOT, TOOLS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from dataclasses import replace as _dc_replace  # noqa: E402

from aircraft_core import alignment  # noqa: E402
from aircraft_core.alignment import (  # noqa: E402
    AlignmentError,
    close_outer_loops,
    BluntTrailingEdgeError,
    ChordAxisUnavailableError,
    chord_line_angle_deg,
    resolve_wing_theta_deg,
    rotate_points_deg,
)
from aircraft_core.continuity import encode_rows, web_report_rows  # noqa: E402
from aircraft_core.diagnostics import LayerRecord, RunDiagnostics  # noqa: E402
from aircraft_core.frame import LayerFrame, ReEntrantSectionError  # noqa: E402
from aircraft_core.generator import (  # noqa: E402
    SPAR_STATUS_ABSENT,
    SPAR_STATUS_FIT,
    SPAR_STATUS_NOT_FIT,
    DetectedChannel,
    LayerContext,
    StructureGenerator,
)
from aircraft_core.mesh_section import MeshSectioner  # noqa: E402
from aircraft_core.root_end import (  # noqa: E402
    ROOT_END_PROBE_FRACTION,
    RootEndError,
    area_extremes,
    area_signal,
    channel_presence_from_diameters,
    channel_span_signal,
    resolve_root_end as core_resolve_root_end,
)
from aircraft_core.pinch import (  # noqa: E402
    layer_record_fields as _pinch_record_fields,
    validate_with_pinch_shutdown,
)
from aircraft_core.spar_latch import walk_latch  # noqa: E402
from aircraft_core.z_rule import ZContinuityRule  # noqa: E402
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

# Reused helpers:
from m18_mesh_probe import read_binary_stl  # noqa: E402
from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402
import structural_analysis as sa  # noqa: E402

TOOL_VERSION = "preflight_wing/1"


def _git_attribution() -> Dict:
    """Tool version/commit: the repo commit this run's code came from, plus whether the working tree was dirty at run
    time (a result produced from a dirty tree cannot be reproduced by
    checking out that commit alone). Fails soft to "unavailable" strings
    if git is not on PATH or this is not a git checkout -- an attribution
    tool must never itself crash the run it is trying to attribute."""
    import subprocess

    def _run(args):
        try:
            out = subprocess.run(
                ["git"] + args, cwd=REPO_ROOT, capture_output=True, text=True, timeout=10,
            )
            if out.returncode != 0:
                return None
            return out.stdout.strip()
        except Exception:  # noqa: BLE001
            return None

    commit = _run(["rev-parse", "HEAD"]) or "unavailable"
    status = _run(["status", "--porcelain"])
    dirty = None if status is None else bool(status)
    return {"commit": commit, "working_tree_dirty": dirty}


DEFAULT_CONFIG_PATH = os.path.join(
    REPO_ROOT, "host", "data", "orca_plugins", "config.json"
)

# A real wing can measure ~215 mm chord against ~225 mm span -- the two longest
# bbox extents within 20% of each other. Warn, not refuse: this is evidence
# the "longest axis is the span" shortcut is unsafe, not proof that it is
# wrong for THIS mesh.
AMBIGUOUS_EXTENT_RATIO = 0.80

# ROOT_END_PROBE_FRACTION (fraction of the span probed at each Z extreme by the
# root-end AREA heuristic) lives in aircraft_core.root_end (imported above)
# so the plugin and this tool share ONE copy.

# T2 span-fraction stations, EXACTLY capability.py._build_mesh_sectioner's
# own convention (plugin/aircraft_structures/capability.py:4266-4271): mid
# = 0.5 (authoritative), others sampled only to bound spread.
_MID_FRACTION = 0.5
_OTHER_FRACTIONS = (0.1, 0.3, 0.7, 0.9)


# ---------------------------------------------------------------------------
# T1: model-level checks (model-level input checks) -- fire
# once per run, before any layer exists.
# ---------------------------------------------------------------------------

def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _binary_stl_size_check(path: str) -> Tuple[bool, str]:
    """``read_binary_stl`` trusts bytes 80:84 as a triangle count with no
    other validation. Refuse to treat a file as a binary STL when its size
    does not match ``84 + 50 * count`` -- catches an ASCII STL or a
    truncated file BEFORE ``read_binary_stl`` silently misparses it.
    """
    import struct

    size = os.path.getsize(path)
    if size < 84:
        return False, "file is {} bytes, smaller than the 84-byte binary STL header".format(size)
    with open(path, "rb") as fh:
        count = struct.unpack("<I", fh.read(84)[80:84])[0]
    expected = 84 + 50 * count
    if expected != size:
        return False, (
            "file size {} bytes does not match 84 + 50*{} = {} bytes expected "
            "for a binary STL with this header's triangle count -- this is "
            "probably an ASCII STL, a truncated file, or a corrupt header, "
            "not a binary STL read_binary_stl can trust".format(size, count, expected)
        )
    return True, "{} bytes matches 84 + 50*{} triangles".format(size, count)


class UnionFind:
    def __init__(self, n: int):
        self.parent = list(range(n))

    def find(self, x: int) -> int:
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]
            x = self.parent[x]
        return x

    def union(self, a: int, b: int) -> None:
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


def t1_model_checks(verts: List[Tuple[float, float, float]],
                     tris: List[Tuple[int, int, int]]) -> Dict:
    """Triangle count, closed/two-manifold-by-exact-edge-counting, bbox,
    connected-shell count. All computed from the welded vertex/triangle
    arrays ``read_binary_stl`` already produces -- no re-reading, no
    tolerance-based welding (exact integer vertex-index edges only, matching
    the milestone doc's "exact binary-float edge counting" wording).
    """
    edge_counts: Dict[Tuple[int, int], int] = {}
    uf = UnionFind(len(verts))
    for (i, j, k) in tris:
        for a, b in ((i, j), (j, k), (k, i)):
            key = (a, b) if a < b else (b, a)
            edge_counts[key] = edge_counts.get(key, 0) + 1
            uf.union(a, b)

    boundary_edges = sum(1 for c in edge_counts.values() if c == 1)
    nonmanifold_edges = sum(1 for c in edge_counts.values() if c > 2)
    closed_two_manifold = boundary_edges == 0 and nonmanifold_edges == 0

    roots = set()
    touched = set()
    for (i, j, k) in tris:
        for v in (i, j, k):
            touched.add(v)
    for v in touched:
        roots.add(uf.find(v))
    shell_count = len(roots)

    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    bbox = {
        "min_x": min(xs), "max_x": max(xs),
        "min_y": min(ys), "max_y": max(ys),
        "min_z": min(zs), "max_z": max(zs),
    }
    extents = {
        "x": bbox["max_x"] - bbox["min_x"],
        "y": bbox["max_y"] - bbox["min_y"],
        "z": bbox["max_z"] - bbox["min_z"],
    }
    ranked = sorted(extents.items(), key=lambda kv: kv[1], reverse=True)
    ambiguous_extents = None
    if len(ranked) >= 2 and ranked[0][1] > 0:
        ratio = ranked[1][1] / ranked[0][1]
        if ratio >= AMBIGUOUS_EXTENT_RATIO:
            ambiguous_extents = {
                "longest_axis": ranked[0][0], "longest_mm": ranked[0][1],
                "second_axis": ranked[1][0], "second_mm": ranked[1][1],
                "ratio": ratio,
                "warning": (
                    "the two longest bounding-box extents are within {:.0%} "
                    "of each other ({}={:.3f} mm, {}={:.3f} mm) -- "
                    "'longest axis is the span' is NOT a safe assumption on "
                    "this geometry (precedent: one real wing "
                    "measured ~215 mm chord against ~225 mm span). Verify "
                    "the stand-up axis this tool chose against the actual "
                    "CAD design intent.".format(
                        ratio, ranked[0][0], ranked[0][1],
                        ranked[1][0], ranked[1][1],
                    )
                ),
            }

    return {
        "triangle_count": len(tris),
        "vertex_count": len(verts),
        "boundary_edge_count": boundary_edges,
        "nonmanifold_edge_count": nonmanifold_edges,
        "closed_two_manifold": closed_two_manifold,
        "shell_count": shell_count,
        "single_shell": shell_count == 1,
        "bbox_mm": bbox,
        "extents_mm": extents,
        "ambiguous_extents": ambiguous_extents,
    }


# ---------------------------------------------------------------------------
# Stand-up: axis permutation (proper rotations, determinant +1 -- never a
# mirror) that maps the chosen "up" axis onto +Z, plus root-end resolution.
# ---------------------------------------------------------------------------

def _rotate_axis_to_z(verts: Sequence[Tuple[float, float, float]], axis: str):
    if axis == "z":
        return [(x, y, z) for x, y, z in verts]
    if axis == "y":
        # Proper rotation about X by +90 deg: (x,y,z) -> (x,-z,y). Maps
        # (0,1,0) -> (0,0,1). det(+1) -- verified in the module docstring's
        # design discussion, not a reflection.
        return [(x, -z, y) for x, y, z in verts]
    if axis == "x":
        # Proper rotation about Y by -90 deg: (x,y,z) -> (-z,y,x). Maps
        # (1,0,0) -> (0,0,1). det(+1).
        return [(-z, y, x) for x, y, z in verts]
    raise ValueError("unknown axis {!r}".format(axis))


def _largest_outer_loop_points(sectioner: MeshSectioner, z: float):
    try:
        loops = sectioner.section_at(z)
    except ValueError:
        return None
    outer = [lp for lp in loops if not lp.is_interior]
    if not outer:
        return None
    return list(max(outer, key=lambda lp: abs(lp.signed_area)).points)


def probe_axis_for_chord(verts, tris, axis: str) -> Dict:
    """Section ``axis``-stood-up ``verts`` at mid-extent and run the SHIPPED
    chord-axis detector on the outer loop. Used only to CHOOSE the stand-up
    axis under ``--up-axis auto`` -- never to classify blunt-TE vs. wrong-
    axis on its own (that classification happens in ``t2_orientation``,
    after the axis is chosen)."""
    verts_c = _rotate_axis_to_z(verts, axis)
    try:
        sec = MeshSectioner(verts_c, tris)
    except Exception as exc:  # noqa: BLE001
        return {"axis": axis, "chord_found": False, "error": "{}: {}".format(type(exc).__name__, exc)}
    if sec.z_min is None or sec.z_max is None or sec.z_max <= sec.z_min:
        return {"axis": axis, "chord_found": False, "error": "no Z extent"}
    z_mid = sec.z_min + _MID_FRACTION * (sec.z_max - sec.z_min)
    pts = _largest_outer_loop_points(sec, z_mid)
    if pts is None:
        return {"axis": axis, "chord_found": False, "error": "no outer loop at mid-extent"}
    theta = chord_line_angle_deg(pts)
    hull_len = len(alignment._convex_hull(pts))
    return {
        "axis": axis,
        "chord_found": theta is not None,
        "theta_deg": theta,
        "hull_vertices": hull_len,
    }


def resolve_stand_up_axis(verts, tris, up_axis_arg: str) -> Dict:
    """Design decision 1 + the maintainer addition 2: choose which axis becomes Z.
    ``up_axis_arg`` in {"x","y","z"} is an explicit override that skips all of
    the auto logic. "auto" tries every axis, sections each at its own
    mid-extent, and prefers the axis where the SHIPPED chord-axis detector
    actually resolves a chord -- never the longest bbox axis blindly."""
    if up_axis_arg in ("x", "y", "z"):
        return {
            "mode": "explicit_override",
            "chosen_axis": up_axis_arg,
            "confidence": "explicit override ({}); auto chord-axis probing skipped".format(up_axis_arg),
            "candidates": None,
        }

    probes = {axis: probe_axis_for_chord(verts, tris, axis) for axis in ("x", "y", "z")}
    candidates = [a for a, info in probes.items() if info.get("chord_found")]

    if len(candidates) == 1:
        chosen = candidates[0]
        return {
            "mode": "auto",
            "chosen_axis": chosen,
            "confidence": (
                "auto-detected: axis {!r} is the only candidate whose "
                "mid-extent section resolves a chord axis (theta={:.4f} deg, "
                "{} hull vertices)".format(
                    chosen, probes[chosen]["theta_deg"], probes[chosen]["hull_vertices"]
                )
            ),
            "candidates": probes,
        }

    if len(candidates) == 0:
        xs = [v[0] for v in verts]
        ys = [v[1] for v in verts]
        zs = [v[2] for v in verts]
        extents = {
            "x": max(xs) - min(xs), "y": max(ys) - min(ys), "z": max(zs) - min(zs),
        }
        fallback = max(extents, key=extents.get)
        return {
            "mode": "auto_fallback_longest_bbox",
            "chosen_axis": fallback,
            "confidence": (
                "LOW CONFIDENCE: no candidate axis produced a resolvable "
                "chord axis at its own mid-extent (probes: {}). Falling "
                "back to the longest raw bounding-box axis ({!r}, {:.3f} "
                "mm). BOTH a wrong stand-up axis AND a genuinely "
                "non-aerofoil/blunt section remain open explanations for "
                "every T2/T3 result below -- this tool does not attribute "
                "it to either. Pass --up-axis explicitly if you know which "
                "CAD axis is the span.".format(probes, fallback, extents[fallback])
            ),
            "candidates": probes,
        }

    return {
        "mode": "auto_ambiguous",
        "chosen_axis": None,
        "confidence": (
            "AMBIGUOUS: {} candidate axes ({}) each resolve a chord axis at "
            "their own mid-extent -- refusing to auto-select one. Pass "
            "--up-axis explicitly.".format(len(candidates), candidates)
        ),
        "candidates": probes,
    }


def resolve_root_end(verts, root_end_arg: str) -> Dict:
    """Which end of the (already stood-up) Z axis is the root. Compares
    cross-sectional AREA near each Z extreme (via a coarse convex-hull
    shoelace estimate on the raw points at that Z band, not a full section
    -- cheap and sufficient to tell "thick root" from "thin/pointed tip")
    and places the larger-area end at the bed. ``root_end_arg`` in
    {"low","high"} is an explicit override; "auto" (default) does the
    comparison."""
    if root_end_arg in ("low", "high"):
        return {
            "mode": "explicit_override", "root_end": root_end_arg,
            "confidence": "explicit override ({})".format(root_end_arg),
        }
    m = area_extremes(verts)
    if m["span_mm"] <= 0:
        return {"mode": "degenerate", "root_end": "low", "confidence": "no Z extent; defaulting to low"}

    band = m["band_mm"]
    area_low = m["area_low_end_mm2"]
    area_high = m["area_high_end_mm2"]
    root_end = "low" if area_low >= area_high else "high"
    return {
        "mode": "auto",
        "root_end": root_end,
        "area_low_end_mm2": area_low,
        "area_high_end_mm2": area_high,
        "confidence": (
            "auto: low-end hull area {:.3f} mm^2 vs high-end {:.3f} mm^2 "
            "within {:.3f} mm of each extreme -- {} end is larger, treated "
            "as root".format(area_low, area_high, band, root_end)
        ),
    }


def apply_stand_up(verts, tris, up_axis_arg: str, root_end_arg: str,
                   keep_placement: bool = False) -> Dict:
    axis_decision = resolve_stand_up_axis(verts, tris, up_axis_arg)
    chosen_axis = axis_decision["chosen_axis"]
    result = {"axis_decision": axis_decision}
    if chosen_axis is None:
        return result

    stood = _rotate_axis_to_z(verts, chosen_axis)
    root_decision = resolve_root_end(stood, root_end_arg)
    result["root_end_decision"] = root_decision
    result["keep_placement"] = bool(keep_placement)
    if root_decision["root_end"] == "high" and not keep_placement:
        stood = [(x, y, -z) for x, y, z in stood]

    z_min = min(v[2] for v in stood)
    dropped = [(x, y, z - z_min) for x, y, z in stood] if z_min != 0.0 else stood
    result["z_min_before_drop"] = z_min
    result["verts"] = dropped
    xs = [v[0] for v in dropped]
    ys = [v[1] for v in dropped]
    zs = [v[2] for v in dropped]
    result["post_stand_up_bbox_mm"] = {
        "x": (min(xs), max(xs)), "y": (min(ys), max(ys)), "z": (min(zs), max(zs)),
    }
    result["span_mm"] = max(zs) - min(zs)
    return result


# ---------------------------------------------------------------------------
# T2: orientation -- the shipped chord-axis detector, run exactly the way
# capability.py._build_mesh_sectioner runs it.
# ---------------------------------------------------------------------------

def t2_orientation(sectioner: MeshSectioner) -> Dict:
    z_lo, z_hi = sectioner.z_min, sectioner.z_max
    out = {
        "theta_deg": 0.0, "rotation_applied": False,
        "mid_span_theta_deg": None, "station_theta_deg": [], "spread_deg": None,
        "excluded_stations": [], "chord_axis_unavailable": False,
        "fatal": False, "fatal_message": None, "error": None,
        "mid_hull_vertices": None,
        # provenance for a theta resolved from a sharp-TE band substituted for
        # a blunt mid-span station (elevon wings).
        "theta_source": None,
        "sharp_band_fracs": None,
        "sharp_band_spread_deg": None,
        "blunt_crosscheck": None,
        # provenance for a theta read from the mid-span thin trailing-edge flat
        # when no sharp band exists (None otherwise).
        "thin_te_flat": None,
    }
    if z_lo is None or z_hi is None or z_hi <= z_lo:
        out["error"] = "mesh has no Z extent to sample span stations from"
        return out

    mid_pts = _largest_outer_loop_points(sectioner, z_lo + _MID_FRACTION * (z_hi - z_lo))
    other_pts = [
        pts for pts in (
            _largest_outer_loop_points(sectioner, z_lo + f * (z_hi - z_lo))
            for f in _OTHER_FRACTIONS
        ) if pts is not None
    ]
    if mid_pts is None:
        out["error"] = "mid-span (50% of Z range) section has no outer loop"
        return out

    try:
        hull_len = len(alignment._convex_hull(mid_pts))
        out["mid_hull_vertices"] = hull_len
    except Exception:  # noqa: BLE001
        pass

    band_provenance = {}
    try:
        theta_deg, station_thetas, spread, excluded = resolve_wing_theta_deg(
            mid_pts, other_pts,
            # substitute a contiguous band of sharp-TE span stations when the
            # mid-span station is squared off (an elevon/aileron cutout) -- see
            # resolve_wing_theta_deg's own docstring. Only activates on
            # BluntTrailingEdgeError at mid-span.
            station_sampler=lambda f: _largest_outer_loop_points(
                sectioner, z_lo + f * (z_hi - z_lo)),
            provenance=band_provenance,
        )
        out["theta_deg"] = theta_deg
        out["rotation_applied"] = True
        # NOTE: carries the BAND theta, not a literal mid-span reading,
        # when out["theta_source"] == "sharp_te_band" -- no key rename.
        out["mid_span_theta_deg"] = theta_deg
        out["station_theta_deg"] = station_thetas
        out["spread_deg"] = spread
        out["theta_source"] = band_provenance.get("theta_source")
        if out["theta_source"] in ("sharp_te_band", "thin_te_flat"):
            # This run's theta came from the sharp-TE band (or the thin-flat
            # reading), so an excluded OTHER station's own blunt-TE message
            # (which says outright "The slice is refused rather than sliced
            # unrotated") would misreport a run that in fact resolved. See the
            # matching capability.py comment for the same reasoning.
            _theta_from = ("sharp-TE band" if out["theta_source"] == "sharp_te_band"
                           else "thin trailing-edge flat")
            out["excluded_stations"] = [
                {
                    "index": i,
                    "reason": r.split(". The slice is refused", 1)[0]
                    + (
                        # keep the thin-flat refusal reason on the thin-flat
                        # path only.
                        " Thin-flat reading also refused:"
                        + r.split(" Thin-flat reading also refused:", 1)[1]
                        if out["theta_source"] == "thin_te_flat"
                        and " Thin-flat reading also refused:" in r
                        else ""
                    )
                    + " (excluded from the spread check only; this run's "
                    "theta was resolved from the " + _theta_from + ", not this "
                    "station).",
                }
                for i, r in excluded
            ]
        else:
            out["excluded_stations"] = [{"index": i, "reason": r} for i, r in excluded]
        out["sharp_band_fracs"] = band_provenance.get("band_fracs")
        out["sharp_band_spread_deg"] = band_provenance.get("band_spread_deg")
        out["blunt_crosscheck"] = band_provenance.get("blunt_crosscheck")
        out["thin_te_flat"] = band_provenance.get("thin_te_flat")
    except ChordAxisUnavailableError as exc:
        # NOT fatal. A section with FEWER than
        # alignment.AEROFOIL_MIN_HULL_VERTICES hull vertices is treated as a
        # structural coupon and ships UNROTATED, silently, in-host.
        out["chord_axis_unavailable"] = True
        out["error"] = str(exc)
    except BluntTrailingEdgeError as exc:
        # FATAL in-host. An aerofoil-shaped section (>= 12 hull vertices) with
        # a squared-off TE refuses the slice.
        out["fatal"] = True
        out["fatal_message"] = (
            "in-host, this run would FAIL CLOSED with BluntTrailingEdgeError "
            "at the first body layer, and every body layer after it, with: "
            "{}".format(exc)
        )
        out["error"] = str(exc)
    except AlignmentError as exc:
        # Genuine, aerofoil-shaped section with an unresolved alignment
        # problem (ambiguous leading edge, or per-station spread beyond the
        # washout envelope) -- capability.py's self._alignment_fatal_error
        # path, fatal at the first body layer and every one after it.
        out["fatal"] = True
        out["fatal_message"] = (
            "in-host, this run would FAIL CLOSED with AlignmentError at the "
            "first body layer, and every body layer after it, with: "
            "{}".format(exc)
        )
        out["error"] = str(exc)
    return out


def apply_stand_up_confound(axis_mode: str, t2: Dict) -> Optional[str]:
    """the maintainer correction: under the longest-bbox FALLBACK stand-up
    (``axis_mode == "auto_fallback_longest_bbox"``, i.e. no candidate axis
    resolved a chord axis on its own mid-extent), a blunt-TE-shaped or
    chord-axis-unavailable reading at THIS (possibly wrong) axis must not be
    attributed to the trailing edge -- a wrong stand-up axis produces the exact
    same ~90-degree planform-corner signature T2's own detector reads as a
    blunt TE. Mutates ``t2`` in place: clears ONLY ``fatal`` (so T3 does not
    mirror a trailing-edge-specific refusal message under an unresolved axis)
    and sets ``stand_up_confound``. Deliberately does NOT clear
    ``chord_axis_unavailable`` -- the chord axis genuinely was not resolved;
    what is unknown is WHY (wrong axis vs. genuinely blunt), not WHETHER, and
    zeroing it would make the run summary read False on a wing where the axis
    is not actually known.

    Returns the caveat message to print, or ``None`` if this confound does
    not apply (``axis_mode`` was not the fallback, or T2's own read did not
    land on ``fatal``/``chord_axis_unavailable``).

    ``main`` deliberately does NOT derive ``stand_up_unresolved`` from this
    function's return value (see the call site); it derives it straight from
    ``axis_mode == "auto_fallback_longest_bbox"``. Two reasons:

    1. A fallback axis can still yield a cleanly resolved T2 theta
       (``rotation_applied=True``, neither ``fatal`` nor
       ``chord_axis_unavailable``). ``t2_orientation`` calls
       ``resolve_wing_theta_deg`` WITH a ``station_sampler``, so a mid-span
       station that raises ``BluntTrailingEdgeError`` can still resolve via
       the sharp-TE-band substitute, while ``probe_axis_for_chord`` is a bare
       mid-extent ``chord_line_angle_deg`` probe with no band search and so
       cannot find that resolution when choosing the axis.
    2. T2's early-return paths in ``t2_orientation`` (no Z extent; no outer
       loop at the mid-extent section) set ``t2["error"]`` but leave BOTH
       ``fatal`` and ``chord_axis_unavailable`` False, so the
       ``if not (t2["fatal"] or t2["chord_axis_unavailable"]): return None``
       guard below returns ``None`` (no confound, no message) for a
       fallback-axis run whose T2 read never resolved a chord at all. This
       is reachable with real geometry (a mesh with no usable mid-extent
       cross-section on any axis).

    Deriving the flag from ``axis_mode`` means no shape of T2 failure can
    leave an unconfirmed stand-up axis reporting clean.
    """
    if axis_mode != "auto_fallback_longest_bbox":
        return None
    if not (t2["fatal"] or t2["chord_axis_unavailable"]):
        return None
    t2["fatal"] = False
    t2["stand_up_confound"] = True
    return (
        "the stand-up axis itself was never confirmed (fallback to "
        "longest-bbox-axis heuristic; see STAND-UP above), and the chosen "
        "axis's mid-extent section does not resolve a chord axis either "
        "(raw detail: {}). A WRONG STAND-UP AXIS and a genuinely blunt/"
        "non-aerofoil trailing edge are BOTH open explanations -- this tool "
        "does not attribute it to either. T3 below runs UNROTATED (theta=0) "
        "under this same uncertainty.".format(t2["error"])
    )


# ---------------------------------------------------------------------------
# T3: per-layer sweep -- real generator/slot/diff/validate pipeline,
# degrade-by-default, aircraft_core.diagnostics.RunDiagnostics for exact field
# parity with a real in-host report.
# ---------------------------------------------------------------------------

def _layer_z(index: int, layer_height: float, first_layer_height: float) -> float:
    return first_layer_height + index * layer_height


def _layer_slice_z(index: int, layer_height: float, first_layer_height: float) -> float:
    """The plane the GEOMETRY actually comes from, half A LAYER HEIGHT
    below ``print_z`` (capability.py._section_plane_for's own docstring:
    "print_z is the nozzle height for the layer; slice_z is the plane the
    geometry came from, half a layer height lower ... sectioning the mesh
    at print_z samples half a layer too high, which on a tapering wing is
    a real chord error"). Sectioning at ``print_z`` instead of ``slice_z``
    was measured to move every section by up to 0.1 mm of Z on this
    project's own real wings and to misclassify the top layer's tip
    -shutdown as "beyond mesh extent" instead (see this module's own
    docstring "FIDELITY" section). Uses THIS layer's own height (the
    first layer's own ``first_layer_height``, every other layer's
    ``layer_height``), so layer 0's slice_z is ``first_layer_height / 2``
    -- the same value capability.py's own comment implies for the first
    layer, not a layer_height-sized offset applied uniformly."""
    print_z = _layer_z(index, layer_height, first_layer_height)
    this_layer_height = first_layer_height if index == 0 else layer_height
    return print_z - 0.5 * this_layer_height


def _n_layers(span_mm: float, layer_height: float, first_layer_height: float) -> int:
    if span_mm <= first_layer_height:
        return 1
    return 1 + int(math.ceil((span_mm - first_layer_height) / layer_height))


def t3_layer_sweep(
    verts, tris, spec, wall_width_mm: float,
    layer_height: float, first_layer_height: float,
    orientation: Dict,
    *,
    closing_radius_mm: Optional[float] = None,
    latch_root_mode: Optional[str] = None,
    latch_enforce: bool = False,
    host_rescues: bool = False,
    host_largest_island: bool = False,
    host_infeasible_drop: bool = False,
) -> Tuple[RunDiagnostics, Dict]:
    """The per-layer mesh replay.

    Replay-fidelity options (all default off, giving the plain sweep):

    * ``closing_radius_mm``: close the mesh's outer loops at the host's
      ``slice_closing_radius`` before picking the layer's contour, so islands
      the host bridged read as the one contour the host cut.
    * ``latch_root_mode`` (``"auto"``/``"bed"``/``"top"``): run a
      pre-pass over every layer's spar-channel status, resolve the root end,
      walk the latch root -> tip, and drop the CONFIRMED spar socket on
      every layer the walk suppresses, exactly like the host's precomputed
      latch.
    * ``host_largest_island``: when the (closed) mesh section still has
      several outer loops, structure the LARGEST one only and record the
      dropped ones in ``meta["largest_island_dropped"]``. Evidence for the
      host doing the same: the outside half's layers 147-173 have a 10 mm
      fragment loop 5 mm clear of the wing and the host's own region holds ONE
      surface (the final in-host run has no ``multi_island_layer`` at all),
      while the plain replay reports 27 multi-island shutdowns there.
    * ``host_infeasible_drop``: drop every slot that
      fails its OWN per-slot feasibility checks before the boolean
      (``TopologyValidator.infeasible_slots``), as the capability does, with a
      ``slot_infeasible`` degradation. Without it the replay FAILS the layers
      (outside 786, 848, 850) the host degrades and carries on.
    * ``host_rescues``: run the host's pair ablation and pinch
      shutdown when validation fails (the host's behaviour with the
      ``z_continuity`` rule OFF). Ignored when ``spec.z_continuity`` is set:
      the Z-continuity rule runs its own validator loop (single-web ablation)
      and does not use the pair ablation or pinch shutdown.

    ``spec.z_continuity`` True runs ``aircraft_core.z_rule.ZContinuityRule``
    in the loop (the SAME code the capability runs in-host).
    """
    theta_deg = orientation.get("theta_deg") or 0.0
    rotation_applied = bool(orientation.get("rotation_applied"))

    # Rotate the WHOLE mesh by -theta ONCE, before building the sectioner this
    # whole sweep uses -- mirrors capability.py._build_mesh_sectioner exactly
    # (it rotates ``verts`` before constructing the ``MeshSectioner`` it
    # returns, so the sectioner's vertices are already de-rotated). Theta
    # rotates only about Z (pure XY rotation), so this changes neither z_min
    # nor z_max and commutes with Z-sectioning -- but it is NOT equivalent to
    # rotating only the outer loop per layer while leaving interior (channel)
    # loops unrotated, which misplaces the detected spar channel.
    if rotation_applied and theta_deg:
        xy = rotate_points_deg([(v[0], v[1]) for v in verts], -theta_deg)
        verts = [(x, y, v[2]) for (x, y), v in zip(xy, verts)]

    sec = MeshSectioner(verts, tris)
    z_min, z_max = sec.z_min, sec.z_max
    span_mm = (z_max - z_min) if (z_min is not None and z_max is not None) else 0.0
    n_layers = _n_layers(span_mm, layer_height, first_layer_height)

    ops = MiterShapelyOps()
    generator = StructureGenerator()
    builder = SlotBuilder()
    validator = TopologyValidator(ops, safety_offset_mm=wall_width_mm)
    diagnostics = RunDiagnostics()

    root_solid_layers = int(spec.root_solid_layers or 0)
    # per-layer spar-channel status + largest interior-loop diameter, for the
    # root-end channel signal and the latch walk. Layers this sweep does not
    # section (root-solid, beyond the mesh, multi-island) stay None.
    spar_status: List[Optional[str]] = [None] * n_layers
    max_dia: List[Optional[float]] = [None] * n_layers
    layer_ms: List[Optional[float]] = [None] * n_layers
    z_rule: Optional[ZContinuityRule] = None
    if getattr(spec, "z_continuity", False):
        z_rule = ZContinuityRule.from_spec(
            spec, ramp_mm=layer_height, safety_offset_mm=wall_width_mm)
    latch_suppressed: set = set()
    latch_walk_summary = None
    meta = {
        "n_layers": n_layers, "span_mm": span_mm,
        "z_min": z_min, "z_max": z_max,
        "root_solid_layers": root_solid_layers,
        "layer_ms": layer_ms,
        "replay_fidelity": {
            "closing_radius_mm": closing_radius_mm,
            "latch_walk": latch_root_mode,
            "host_rescues": bool(host_rescues and z_rule is None),
            "host_largest_island": bool(host_largest_island),
            "host_infeasible_drop": bool(host_infeasible_drop),
            "z_continuity": z_rule is not None,
        },
        "largest_island_dropped": [],
        "spar_status": spar_status, "max_channel_diameter_mm": max_dia,
        "layer_print_z": [
            _layer_z(i, layer_height, first_layer_height) for i in range(n_layers)
        ],
        "layer_slice_z": [
            _layer_slice_z(i, layer_height, first_layer_height) for i in range(n_layers)
        ],
    }

    # T2 fatal alignment: capability.py raises the SAME TopologyError at
    # EVERY body layer once alignment_fatal_error is set
    # (capability.py:2645-2648), never just the first. Mirrored here
    # directly, without running generation at all for every layer -- both
    # faithful AND cheap.
    if orientation.get("fatal"):
        msg = orientation["fatal_message"]
        for index in range(n_layers):
            z = _layer_z(index, layer_height, first_layer_height)
            if index < root_solid_layers:
                diagnostics.record(LayerRecord(
                    layer_index=index, print_z=z, mutated=False,
                    extra={"reason": "root_solid_layers"},
                ))
                continue
            diagnostics.record(LayerRecord(
                layer_index=index, print_z=z, mutated=False, failed=True,
                message=msg, extra={"reason": "alignment_fatal"},
            ))
        return diagnostics, meta

    if latch_root_mode is not None:
        latch_walk_summary, latch_suppressed = _latch_prepass(
            sec, verts, spec, wall_width_mm, n_layers, layer_height,
            first_layer_height, z_min, z_max, latch_root_mode, latch_enforce)
        meta["latch_walk"] = latch_walk_summary

    _t_prev = None
    _i_prev = None
    for index in range(n_layers):
        _now = time.perf_counter()
        if _i_prev is not None:
            layer_ms[_i_prev] = round((_now - _t_prev) * 1000.0, 2)
        _t_prev, _i_prev = _now, index
        z = _layer_z(index, layer_height, first_layer_height)
        # capability.py._section_plane_for: the geometry comes from
        # ``slice_z`` (half A LAYER HEIGHT below ``print_z``), never
        # ``print_z`` itself -- see ``_layer_slice_z``'s own docstring.
        # ``z`` (print_z) is still what every LayerRecord/LayerContext/
        # ``validator.validate`` call below carries, exactly matching what
        # the host passes to all three at ``layer.print_z``; only the
        # SECTIONING PLANE changes.
        slice_z = _layer_slice_z(index, layer_height, first_layer_height)
        if index < root_solid_layers:
            diagnostics.record(LayerRecord(
                layer_index=index, print_z=z, mutated=False,
                extra={"reason": "root_solid_layers"},
            ))
            continue

        if slice_z <= z_min or slice_z >= z_max:
            # Legitimate degenerate cap (module docstring item 6 of
            # mesh_section.py) -- nothing to cross-section here.
            diagnostics.record(LayerRecord(
                layer_index=index, print_z=z, mutated=False,
                extra={"reason": "at_or_beyond_mesh_extent"},
            ))
            continue

        try:
            loops = sec.section_at(slice_z)
            outer = [lp for lp in loops if not lp.is_interior]
            inner = [lp for lp in loops if lp.is_interior]
            if inner:
                max_dia[index] = max(lp.equivalent_diameter_mm for lp in inner)
            if not outer:
                raise TopologyError(
                    index, z,
                    "mesh cross-section at this layer has no outer loop, so "
                    "the requested web cannot be placed."
                )
            if closing_radius_mm:
                # the host closed its slice (grow r, union, shrink r) before
                # the hook ran; apply the same closing to the mesh's outer
                # loops so a pair of islands the host bridged reads as the ONE
                # contour the host cut. Same call the capability's alignment
                # guard makes (skip_single_loop=True).
                outer = list(close_outer_loops(
                    ops, outer, float(closing_radius_mm),
                    skip_single_loop=True).outer)
                if not outer:
                    raise TopologyError(
                        index, z,
                        "closing the mesh section at slice_closing_radius="
                        "{} mm left no outer contour.".format(closing_radius_mm))
            if host_largest_island and len(outer) > 1:
                _ranked = sorted(outer, key=lambda lp: abs(lp.signed_area),
                                 reverse=True)
                if len(meta["largest_island_dropped"]) < 500:
                    meta["largest_island_dropped"].append({
                        "layer": index,
                        "kept_area_mm2": round(abs(_ranked[0].signed_area), 2),
                        "dropped_areas_mm2": [
                            round(abs(lp.signed_area), 2) for lp in _ranked[1:]],
                    })
                outer = [_ranked[0]]
            if len(outer) > 1:
                if z_rule is not None:
                    z_rule.note_skipped(index, "multi_island_layer")
                areas = sorted((abs(lp.signed_area) for lp in outer), reverse=True)
                diagnostics.record(LayerRecord(
                    layer_index=index, print_z=z, mutated=False, shutdown=True,
                    degradations=[{
                        "kind": "multi_island_layer", "x_mm": 0.0,
                        "reason": (
                            "DEGRADED -- this layer has {} disjoint material "
                            "islands (areas {}). Left unmutated rather than "
                            "structuring one and hoping.".format(
                                len(outer), ", ".join("{:.2f}".format(a) for a in areas)
                            )
                        ),
                    }],
                    extra={"reason": "multi_island_layer"},
                ))
                continue

            # The whole MESH was already rotated into canonical (chord
            # -along-X) frame once, above -- both this outer loop and every
            # interior (channel) loop from ``sec.section_at(z)`` come out
            # already canonical, exactly mirroring
            # capability.py._build_mesh_sectioner (it rotates ``verts`` before
            # constructing the sectioner it hands to every per-layer call, so
            # every downstream section is canonical by construction, never
            # re-rotated per loop). Rotating only the OUTER loop after
            # sectioning an un-rotated mesh would be equivalent for the outer
            # loop alone (theta is a pure Z-axis rotation, which commutes with
            # Z-sectioning) but wrong for channels, whose centroids would stay
            # un-rotated and make the slot builder report "slot... requested
            # but is not present in the output" on most body layers.
            best_outer = max(outer, key=lambda lp: abs(lp.signed_area))
            poly = ops.polygon_from_points(list(best_outer.points))
            frame = LayerFrame.from_polygon(ops, poly)

            channels = tuple(
                DetectedChannel(lp.centroid[0], lp.centroid[1], lp.equivalent_diameter_mm)
                for lp in inner
            )
            layer_ctx = LayerContext(layer_index=index, print_z=z, detected_channels=channels)
            spar_status[index] = generator.spar_channel_status(
                frame, spec, layer_ctx, wall_width_mm)
            if z_rule is not None:
                # A whole-layer shutdown only for a genuinely
                # re-entrant section (bare section erodes into >1 island).
                hit = z_rule.reentrant_check(ops, poly)
                if hit is not None:
                    z_rule.note_skipped(index, "genuine_reentrant")
                    z_rule._event("genuine_reentrant", layer=index,
                                  z=round(z, 4), islands=hit["islands"],
                                  areas_mm2=hit["areas_mm2"])
                    diagnostics.record(LayerRecord(
                        layer_index=index, print_z=z, mutated=False,
                        shutdown=True,
                        degradations=[{
                            "kind": "re_entrant_section", "x_mm": 0.0,
                            "reason": "DEGRADED -- no structure on this "
                                      "layer: " + hit["message"],
                        }],
                        extra={"reason": "re_entrant_section",
                               "genuine_reentrant": True},
                    ))
                    continue

            def _gen(caps, _frame=frame, _ctx=layer_ctx):
                return generator.generate(
                    _frame, spec, wall_width_mm,
                    _ctx if caps is None else _dc_replace(_ctx, depth_caps=caps),
                    safety_offset_mm=wall_width_mm,
                )

            generation = (
                z_rule.generate_layer(index, z, _gen) if z_rule is not None
                else _gen(None))
            intents = generation.intents
            omitted = generation.omitted
            layer_degradations = [
                {"kind": d.kind, "x_mm": round(d.x_mm, 4), "reason": d.reason}
                for d in generation.degradations
            ]
            if index in latch_suppressed:
                # latch walk: the rod-reachable socket ENDED nearer the root,
                # so a confirmed socket that fits again here is dropped.
                resumed = [i for i in intents
                           if i.family_name == "spar_socket"
                           and getattr(i, "spar_confirmed", False)]
                if resumed:
                    intents = [i for i in intents
                               if i.family_name != "spar_socket"]
                    layer_degradations.append({
                        "kind": "spar_socket_latched",
                        "x_mm": round(resumed[0].start[0], 4),
                        "reason": "DEGRADED -- spar socket suppressed by the "
                                  "root-end-aware latch walk.",
                    })
            if z_rule is not None:
                intents, omitted, _sup = z_rule.post_latch(index, intents, omitted)

            if not intents:
                if z_rule is not None:
                    z_rule.note_skipped(index, "no_structure")
                # degrade by default, regardless of tip_shutdown_enabled (that
                # flag is inert later -- see aircraft_core/diagnostics.py's own
                # LayerRecord.shutdown docstring).
                thinness_prefixes = ("skin margin:", "thin section:")
                all_thin = bool(omitted) and all(
                    o.reason.startswith(thinness_prefixes) for o in omitted
                )
                why = (
                    "the section has run out of material to carry any web "
                    "({} candidate(s), all thinness/skin-margin)".format(len(omitted))
                    if all_thin else
                    "every one of {} candidate web(s) was omitted".format(len(omitted))
                    if omitted else
                    "no candidate web stations were generated at all"
                )
                layer_degradations.append({
                    "kind": "pattern_all_webs_omitted", "x_mm": 0.0,
                    "reason": "DEGRADED -- no structure on this layer: {}. "
                              "Prints as plain skin.".format(why),
                })
                _rows, _omit_rows = web_report_rows(intents, omitted)
                diagnostics.record(LayerRecord(
                    layer_index=index, print_z=z, mutated=False, shutdown=True,
                    degradations=layer_degradations,
                    extra={
                        "reason": "tip_shutdown" if all_thin else "all_webs_omitted",
                        "node_merge_count": 0,
                        "web_omit": encode_rows(_omit_rows),
                    },
                ))
                continue

            built = [builder.build(i, spec.overshoot_mm) for i in intents]
            slots = [ops.polygon_from_points(b.points) for b in built]
            slot_labels = ["family {} ({!r})".format(i.family_index, i.family_name) for i in intents]
            family_indices = [i.family_membership() for i in intents]
            if host_infeasible_drop:
                _viol = validator.infeasible_slots(
                    ops, poly, slots, spec, index, z,
                    slot_labels=slot_labels, family_indices=family_indices)
                if _viol:
                    _drop = {n - 1 for n, _exc in _viol}
                    for n, exc in _viol:
                        layer_degradations.append({
                            "kind": "slot_infeasible",
                            "x_mm": round(intents[n - 1].start[0], 4),
                            "reason": "DEGRADED -- " + exc.cause,
                        })
                    _keep = [i for i in range(len(intents)) if i not in _drop]
                    intents = [intents[i] for i in _keep]
                    slots = [slots[i] for i in _keep]
                    slot_labels = [slot_labels[i] for i in _keep]
                    family_indices = [family_indices[i] for i in _keep]
                    if not intents:
                        if z_rule is not None:
                            z_rule.note_skipped(index, "no_structure")
                        layer_degradations.append({
                            "kind": "pattern_all_webs_omitted", "x_mm": 0.0,
                            "reason": "DEGRADED -- every web failed its own "
                                      "per-slot check. Prints as plain skin.",
                        })
                        diagnostics.record(LayerRecord(
                            layer_index=index, print_z=z, mutated=False,
                            shutdown=True, degradations=layer_degradations,
                            extra={"reason": "all_webs_omitted",
                                   "node_merge_count": 0},
                        ))
                        continue
            replacement = ops.diff(poly, slots)
            def _rebuild(keep):
                nonlocal intents, slots, slot_labels, family_indices, replacement
                intents = [intents[i] for i in keep]
                slots = [slots[i] for i in keep]
                slot_labels = [slot_labels[i] for i in keep]
                family_indices = [family_indices[i] for i in keep]
                replacement = ops.diff(poly, slots)

            if z_rule is not None:
                rescue = z_rule.validate_with_ablation(
                    ops, validator, [poly], replacement, intents, slots, spec,
                    index, z, slot_labels, family_indices)
                if rescue is not None:
                    _rebuild(list(rescue.kept_indices))
                    omitted = list(omitted) + list(rescue.omitted)
                    layer_degradations.append({
                        "kind": "z_ablation", "x_mm": 0.0,
                        "reason": "DEGRADED -- " + rescue.omitted[0].reason,
                    })
            elif host_rescues:
                pair_rescue, pinch = validate_with_pinch_shutdown(
                    ops, validator, [poly], replacement, intents, slots, spec,
                    index, z, slot_labels, family_indices, wall_width_mm)
                if pinch is not None:
                    _pd, _pe = _pinch_record_fields(pinch)
                    diagnostics.record(LayerRecord(
                        layer_index=index, print_z=z, mutated=False,
                        shutdown=True, degradations=_pd, extra=_pe))
                    continue
                if pair_rescue is not None:
                    _rebuild(list(pair_rescue.kept_indices))
                    omitted = list(omitted) + [pair_rescue.omitted]
                    layer_degradations.append({
                        "kind": "pair_ablation",
                        "x_mm": round(pair_rescue.station_x_mm, 4),
                        "reason": "DEGRADED -- " + pair_rescue.omitted.reason,
                    })
            else:
                validator.validate(
                    [poly], replacement, slots, spec, index, z,
                    slot_labels=slot_labels, family_indices=family_indices,
                )
            if z_rule is not None:
                z_rule.commit(index, z, intents, omitted)
            node_merge_count = sum(1 for i in intents if i.merged_family_indices)
            # the same compact per-web rows the in-host report carries
            # (aircraft_core/continuity.py), so the checker runs on a mesh
            # replay.
            web_rows, web_omit_rows = web_report_rows(intents, omitted)
            diagnostics.record(LayerRecord(
                layer_index=index, print_z=z, mutated=True,
                degradations=layer_degradations,
                extra={
                    "webs": encode_rows(web_rows),
                    "web_omit": encode_rows(web_omit_rows),
                    "node_merge_count": node_merge_count,
                    "order_swap_count": 0,  # see module docstring: not reimplemented
                    "chord_axis_unavailable": bool(orientation.get("chord_axis_unavailable")),
                },
            ))
        except ReEntrantSectionError as exc:
            # Mirrors capability.py:3371-3412 exactly: the whole SECTION is
            # re-entrant, so "local thickness" is not a well-defined pair of
            # surfaces at any station on this layer. The real host does NOT
            # fail the slice here -- it leaves the layer unmutated and records
            # a degradation, same shape as the multi-island and
            # all-webs-omitted branches above. This exception type needs its
            # own branch: it is a ``ValueError`` subclass and would otherwise
            # fall into the catch-all below and be counted as ``failed``.
            if z_rule is not None:
                z_rule.note_skipped(index, "re_entrant_section")
            diagnostics.record(LayerRecord(
                layer_index=index, print_z=z, mutated=False, shutdown=True,
                degradations=[{
                    "kind": "re_entrant_section",
                    "x_mm": 0.0,
                    "reason": (
                        "DEGRADED -- no structure on this layer: " + str(exc)
                        + " The layer prints as plain skin rather than "
                        "failing the slice."
                    ),
                }],
                extra={"reason": "re_entrant_section"},
            ))
        except TopologyError as exc:
            diagnostics.record(LayerRecord(
                layer_index=index, print_z=z, mutated=False, failed=True, message=str(exc),
            ))
            if z_rule is not None:
                z_rule.note_skipped(index, "failed")
        except Exception as exc:  # noqa: BLE001 -- report, never crash the sweep
            diagnostics.record(LayerRecord(
                layer_index=index, print_z=z, mutated=False, failed=True,
                message="UNEXPECTED (not a clean TopologyError): {}: {}".format(
                    type(exc).__name__, exc
                ),
            ))
            if z_rule is not None:
                z_rule.note_skipped(index, "failed")

    if _i_prev is not None:
        layer_ms[_i_prev] = round((time.perf_counter() - _t_prev) * 1000.0, 2)
    if z_rule is not None:
        meta["z_rule"] = z_rule.summary()
    return diagnostics, meta


def _latch_prepass(sec, verts, spec, wall_width_mm, n_layers, layer_height,
                   first_layer_height, z_min, z_max, root_mode, enforce):
    """pre-pass for the replay: every layer's spar-channel status (the
    generator's own fit check) and largest interior-loop diameter, then the
    root end and the latch walked root -> tip. Mirrors
    ``capability._resolve_root_end_and_latch`` (same sectioning plane, same
    neutral treatment of multi-island / unevaluable layers). Returns
    ``(summary, suppressed_layer_set)``."""
    ops = MiterShapelyOps()
    generator = StructureGenerator()
    statuses: List[Optional[str]] = [None] * n_layers
    max_dia: List[Optional[float]] = [None] * n_layers
    for index in range(n_layers):
        slice_z = _layer_slice_z(index, layer_height, first_layer_height)
        if z_min is None or slice_z <= z_min or slice_z >= z_max:
            continue
        try:
            loops = sec.section_at(slice_z)
            outer = [lp for lp in loops if not lp.is_interior]
            inner = [lp for lp in loops if lp.is_interior]
            if inner:
                max_dia[index] = max(lp.equivalent_diameter_mm for lp in inner)
            if len(outer) != 1:
                continue
            poly = ops.polygon_from_points(list(outer[0].points))
            frame = LayerFrame.from_polygon(ops, poly)
            channels = tuple(
                DetectedChannel(lp.centroid[0], lp.centroid[1],
                                lp.equivalent_diameter_mm)
                for lp in inner)
            statuses[index] = generator.spar_channel_status(
                frame, spec,
                LayerContext(layer_index=index,
                             print_z=_layer_z(index, layer_height,
                                              first_layer_height),
                             detected_channels=channels),
                wall_width_mm)
        except Exception:  # noqa: BLE001 -- neutral to the walk, like the host
            continue
    premeta = {
        "spar_status": statuses, "max_channel_diameter_mm": max_dia,
        "layer_print_z": [_layer_z(i, layer_height, first_layer_height)
                          for i in range(n_layers)],
        "layer_slice_z": [_layer_slice_z(i, layer_height, first_layer_height)
                          for i in range(n_layers)],
        "z_min": z_min, "z_max": z_max,
    }
    summary = root_end_and_latch_summary(verts, premeta, root_mode, enforce)
    return summary, set(summary["latch_suppressed_layers"])


def _z_runs(indices: Sequence[int], zs: Sequence[float]) -> List[Dict]:
    """Contiguous runs (by layer index) as {layer_first, layer_last,
    z_first_mm, z_last_mm, layers}."""
    runs: List[Dict] = []
    for i in sorted(indices):
        if runs and i == runs[-1]["layer_last"] + 1:
            runs[-1]["layer_last"] = i
            runs[-1]["z_last_mm"] = round(zs[i], 4)
            runs[-1]["layers"] += 1
        else:
            runs.append({"layer_first": i, "layer_last": i,
                         "z_first_mm": round(zs[i], 4),
                         "z_last_mm": round(zs[i], 4), "layers": 1})
    return runs


def root_end_and_latch_summary(verts, meta: Dict, root_end_mode: str,
                               enforce: bool) -> Dict:
    """resolved root end (area + channel signals) and the latch
    walked root -> tip over this sweep's per-layer spar statuses, with the Z
    ranges that keep / lose the rod socket.

    ``root_end_mode`` is "auto" / "bed" / "top" (the plugin's vocabulary).
    ``enforce`` False (the default flip-to-bed mode) reports the signals but
    never refuses: the part was flipped so the AREA-larger end is at the bed
    by construction. Raises ``RootEndError`` only when ``enforce`` is True
    and the two signals disagree.
    """
    statuses = meta["spar_status"]
    zs = meta["layer_print_z"]
    area = area_signal(verts)
    present = channel_presence_from_diameters(meta["max_channel_diameter_mm"])
    channel = channel_span_signal(
        present, meta["layer_slice_z"], meta["z_min"], meta["z_max"])
    try:
        decision = core_resolve_root_end(root_end_mode, area, channel)
        refused = None
    except RootEndError as exc:
        if enforce:
            raise
        refused = str(exc)
        decision = core_resolve_root_end("bed", area, channel)
    root_at_bed = decision["root_end"] == "low"
    walk = walk_latch(statuses, root_at_bed)
    fit_idx = [i for i, st in enumerate(statuses) if st == SPAR_STATUS_FIT]
    not_fit_idx = [i for i, st in enumerate(statuses) if st == SPAR_STATUS_NOT_FIT]
    absent_idx = [i for i, st in enumerate(statuses) if st == SPAR_STATUS_ABSENT]
    supp = set(walk["suppressed"])
    cut_idx = [i for i in fit_idx if i not in supp]
    return {
        "root_end": decision["root_end"], "root_at": decision["root_at"],
        "decision_source": decision["decision_source"],
        "signals_agree": decision["signals_agree"],
        "override_contradicts_signal": decision.get("override_contradicts_signal"),
        "area_signal": area, "channel_signal": channel,
        "auto_would_refuse": refused,
        "latch": {k: walk[k] for k in (
            "walk_direction", "arm_index", "suppressed_count",
            "suppressed_first", "suppressed_last", "fit_layers_before_arm")},
        "latch_suppressed_layers": list(walk["suppressed"]),
        "arm_z_mm": None if walk["arm_index"] is None else round(zs[walk["arm_index"]], 4),
        "layer_counts": {"fit": len(fit_idx), "not_fit": len(not_fit_idx),
                         "absent": len(absent_idx), "suppressed": len(supp),
                         "socket_cut": len(cut_idx)},
        "socket_cut_z_runs": _z_runs(cut_idx, zs),
        "socket_suppressed_z_runs": _z_runs(sorted(supp), zs),
        "not_fit_z_runs": _z_runs(not_fit_idx, zs),
    }


def span_band_breakdown(diagnostics: RunDiagnostics, meta: Dict) -> Dict:
    records = diagnostics.records
    body = [r for r in records if r.extra.get("reason") != "root_solid_layers"]
    if not body:
        return {"root": {}, "mid": {}, "tip": {}}
    z0 = body[0].print_z
    z1 = body[-1].print_z
    span = (z1 - z0) or 1.0
    bands = {"root": [0.0, 1.0 / 3], "mid": [1.0 / 3, 2.0 / 3], "tip": [2.0 / 3, 1.0 + 1e-9]}
    out = {name: {"mutated": 0, "degraded": 0, "shutdown": 0, "failed": 0, "layers": 0} for name in bands}
    for r in body:
        frac = (r.print_z - z0) / span
        for name, (lo, hi) in bands.items():
            if lo <= frac < hi or (name == "tip" and frac >= hi - 1e-9):
                bucket = out[name]
                bucket["layers"] += 1
                if r.failed:
                    bucket["failed"] += 1
                elif r.degradations:
                    bucket["degraded"] += 1
                elif r.shutdown:
                    bucket["shutdown"] += 1
                elif r.mutated:
                    bucket["mutated"] += 1
                break
    return out


# ---------------------------------------------------------------------------
# T4: blind spot, printed unconditionally.
# ---------------------------------------------------------------------------

T4_BLIND_SPOT_TEXT = (
    "T4 BLIND SPOT (unconditional): the mesh-alignment guard in "
    "capability.py (_check_mesh_alignment, six terms: min_x/max_x/min_y/"
    "max_y/centroid_x/centroid_y) compares OUR section against the HOST'S "
    "OWN libslic3r slice of the object as the user placed it in the GUI. There "
    "is no host slice offline -- this tool never sections anything but the "
    "raw STL under its own stand-up assumption. A layer-997-class refusal "
    "is therefore STRUCTURALLY INVISIBLE "
    "to this tool: a clean T3 result here is NOT a guarantee of a clean "
    "in-host slice. The only offline route onto this guard is replaying a "
    "recorded sidecar (tools/replay_host_sections.py), which by definition "
    "does not exist for a wing never sliced before. This tool never prints "
    "an unqualified PASS."
)


# ---------------------------------------------------------------------------
# Host-report comparison.
# ---------------------------------------------------------------------------

_COMPARE_KEYS = (
    "layers_seen", "mutated_count", "failed_count", "shutdown_count",
    "degraded_layer_count", "first_degradation_z", "first_shutdown_z",
    "node_merge_count", "first_node_merge_z",
    "order_swap_count", "first_order_swap_z", "chord_axis_unavailable",
)

# Fields in _COMPARE_KEYS this tool NEVER computes -- see the module
# docstring's "FIDELITY" section: _classify_declared_pair_overlaps lives
# only in capability.py (plugin/aircraft_structures/capability.py:2138),
# not aircraft_core, so reproducing it here would be a reimplementation,
# not reuse. A nonzero delta on these two rows is a DECLARED GAP, not a
# parity failure -- annotated explicitly in the printed table so it does
# not read as one next to eight exact matches.
_COMPARE_KEYS_NOT_IMPLEMENTED = frozenset({"order_swap_count", "first_order_swap_z"})


def compare_to_host_report(our_summary: Dict, host_report_path: str, our_theta_deg: Optional[float] = None) -> Dict:
    with open(host_report_path, "r", encoding="utf-8") as fh:
        host = json.load(fh)
    rows = []
    for key in _COMPARE_KEYS:
        ours = our_summary.get(key)
        theirs = host.get(key)
        delta = None
        if isinstance(ours, (int, float)) and isinstance(theirs, (int, float)) \
                and not isinstance(ours, bool) and not isinstance(theirs, bool):
            delta = ours - theirs
        rows.append({
            "key": key, "ours": ours, "host": theirs, "delta": delta,
            "not_implemented": key in _COMPARE_KEYS_NOT_IMPLEMENTED,
        })

    # Free placement-vs-pipeline discriminator: the theta the HOST actually
    # detected and applied on this exact geometry
    # (host_object_transform.plugin_applied.theta_deg, criterion 1) -- if this
    # matches our own T2 theta closely, T2's chord-axis detection is confirmed
    # against a real in-host run, not merely asserted to agree.
    host_theta = (
        host.get("host_object_transform", {}).get("plugin_applied", {}).get("theta_deg")
    )
    theta_row = None
    if our_theta_deg is not None and isinstance(host_theta, (int, float)):
        theta_row = {
            "key": "t2_theta_deg", "ours": our_theta_deg, "host": host_theta,
            "delta": our_theta_deg - host_theta, "not_implemented": False,
        }
        rows.append(theta_row)

    return {
        "host_report_path": host_report_path,
        "host_degradation_counts_by_kind": host.get("degradation_counts_by_kind"),
        "our_degradation_counts_by_kind": our_summary.get("degradation_counts_by_kind"),
        "rows": rows,
    }


def print_compare_table(cmp: Dict) -> None:
    print("\n=== Comparison against recorded in-host report ===")
    print("  host report: {}".format(cmp["host_report_path"]))
    print("  {:<28} {:>14} {:>14} {:>10}  ".format("field", "ours", "host", "delta"))
    for row in cmp["rows"]:
        annotation = (
            "  <-- DECLARED GAP, not a parity failure (see module docstring "
            "FIDELITY section): not reimplemented offline"
            if row.get("not_implemented") else ""
        )
        print("  {:<28} {!s:>14} {!s:>14} {!s:>10}{}".format(
            row["key"], row["ours"], row["host"], row["delta"], annotation
        ))
    print("  ours degradation kinds:", cmp["our_degradation_counts_by_kind"])
    print("  host degradation kinds:", cmp["host_degradation_counts_by_kind"])


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _load_cap_config(path: str) -> Dict:
    """Small, clearly-marked local helper: same parsing convention as
    tools/d023_arm_sweep.load_live_cap_config, parametrized by path (that
    function hardcodes LIVE_CONFIG_PATH, so it cannot be reused unmodified
    for --config)."""
    with open(path, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    entry = doc["config"][0] if isinstance(doc["config"], list) else doc["config"]
    return entry["cap_config"]


def decide_final_status(
    stand_up_unresolved: bool, failed_count: int, degraded_layer_count: int,
) -> Tuple[str, int]:
    """The run-level status string and exit code, precedence 2 > 3 > 1 > 4
    > 0 (see the module docstring's EXIT CODES section and
    ``EXIT_CODE_EPILOG`` -- all three must stay in sync; this is the one
    function that actually decides it, factored out of ``main`` so the
    decision is unit-testable without a full pipeline run). Exit 2 is
    never returned here -- every exit-2 case (unreadable STL, bad config)
    returns directly from ``main`` before this function is ever called.

    ``stand_up_unresolved`` takes precedence over everything else, including a
    clean T3 sweep -- a genuine blunt-TE wing under the longest-bbox FALLBACK
    stand-up axis must not report exit 0/CLEAN_NO_GUARANTEE merely because T3
    ran informationally (unrotated) after ``apply_stand_up_confound`` cleared
    ``fatal``. The host would refuse this wing at every body layer, so a
    clean-looking exit code would be a silent wrong answer.
    """
    if stand_up_unresolved:
        return "UNRESOLVED_STAND_UP", 3
    if failed_count:
        return "REFUSED", 1
    if degraded_layer_count:
        return "DEGRADED", 4
    return "CLEAN_NO_GUARANTEE", 0


EXIT_CODE_EPILOG = (
    "exit codes (binding -- see module docstring):\n"
    "  0  clean: no refusals, no degradation (T4 caveat still prints)\n"
    "  1  refusals found -- at least one layer the host would refuse\n"
    "  2  tool-level refusal -- unreadable STL, bad config, unusable input\n"
    "  3  stand-up not resolved confidently -- ambiguous (>1 axis resolves\n"
    "     a chord) or low-confidence fallback with an unresolved T2 read\n"
    "  4  structure lost (degradation) but no refusals\n"
    "  precedence when several would apply: 2 > 3 > 1 > 4 > 0\n"
)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, epilog=EXIT_CODE_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("stl", help="path to the wing STL to pre-flight")
    ap.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--up-axis", choices=("auto", "x", "y", "z"), default="auto")
    ap.add_argument("--root-end", choices=("auto", "low", "high"), default="auto")
    ap.add_argument("--layer-height", type=float, default=0.2)
    ap.add_argument("--first-layer-height", type=float, default=None)
    ap.add_argument("--wall-width-mm", type=float, default=0.42)
    ap.add_argument("--json", default=None)
    ap.add_argument("--compare-report", default=None)
    ap.add_argument(
        "--closing-radius-mm", type=float, default=None,
        help="Replay fidelity: close the mesh's outer loops at the "
             "host's slice_closing_radius (mm) before structuring a layer.")
    ap.add_argument(
        "--latch-walk", action="store_true",
        help="Replay fidelity: pre-pass the spar-channel statuses, walk "
             "the spar-socket latch from the resolved root end and drop the "
             "suppressed sockets in the per-layer sweep.")
    ap.add_argument(
        "--host-largest-island", action="store_true",
        help="Replay fidelity: structure only the largest outer loop of a "
             "multi-loop section (what the host's single-region slice shows "
             "on the outside half's layers 147-173).")
    ap.add_argument(
        "--host-infeasible-drop", action="store_true",
        help="Replay fidelity: drop slots that fail their own per-slot checks "
             "before the boolean (Class B degradation), as the capability does.")
    ap.add_argument(
        "--host-rescues", action="store_true",
        help="Replay fidelity: run the host's pair ablation and "
             "pinch shutdown on a failed validation (ignored when the config "
             "sets z_continuity).")
    ap.add_argument(
        "--keep-placement", action="store_true",
        help="Keep the mesh as placed (root at bed OR top); do not flip "
             "the root to the bed. Reports the resolved root end and the "
             "root-end-aware spar-socket latch.")
    args = ap.parse_args(argv)

    first_layer_height = (
        args.first_layer_height if args.first_layer_height is not None else args.layer_height
    )

    t_start = time.perf_counter()
    git_attr = _git_attribution()
    result: Dict = {
        "tool_version": TOOL_VERSION,
        "tool_commit": git_attr["commit"],
        "tool_working_tree_dirty": git_attr["working_tree_dirty"],
        # ``tool_commit`` alone names a commit that may not contain this exact
        # code (an untracked or modified tool file). The tool's own sha256
        # makes the triple (commit, dirty flag, tool_sha256) sufficient to
        # reproduce a result.
        "tool_sha256": sha256_file(os.path.abspath(__file__)),
        "stl_path": os.path.abspath(args.stl),
    }

    if not HAVE_SHAPELY:
        print("shapely is not installed; this tool refuses to report a layer sweep "
              "without a real polygon-boolean backend.", file=sys.stderr)
        return 2

    if not os.path.isfile(args.stl):
        print("STL not found: {}".format(args.stl), file=sys.stderr)
        return 2

    result["stl_sha256"] = sha256_file(args.stl)
    ok, size_msg = _binary_stl_size_check(args.stl)
    result["binary_stl_size_check"] = {"ok": ok, "detail": size_msg}
    print("=== ATTRIBUTION ===")
    print("  tool: {} commit={} dirty={} tool_sha256={}".format(
        TOOL_VERSION, result["tool_commit"], result["tool_working_tree_dirty"],
        result["tool_sha256"]))
    print("  stl: {} (sha256 {})".format(result["stl_path"], result["stl_sha256"]))
    print("  binary STL size check: {}".format(size_msg if ok else "FAILED -- " + size_msg))
    if not ok:
        print("  refusing to parse this file as a binary STL.", file=sys.stderr)
        result["status"] = "REFUSED_NOT_BINARY_STL"
        if args.json:
            with open(args.json, "w", encoding="utf-8") as fh:
                json.dump(result, fh, indent=2, default=str)
        return 2

    cap_config = _load_cap_config(args.config)
    result["config_path"] = os.path.abspath(args.config)
    # host/data/orca_plugins/ config.json is untracked and mutable -- evidence
    # citing it by path alone cites an unhashed moving target. Hash
    # the config FILE (not just the parsed cap_config dict) so a byte-for-byte
    # mismatch is detectable even if a future edit round-trips through JSON
    # identically; the full parsed cap_config is ALSO embedded below (cheap: a
    # few KB) so the exact spec used never has to be reconstructed from a path
    # that may no longer resolve to the same bytes.
    result["config_sha256"] = sha256_file(os.path.abspath(args.config))
    result["cap_config"] = cap_config
    result["assumptions"] = {
        "layer_height_mm": args.layer_height,
        "first_layer_height_mm": first_layer_height,
        "layer_height_source": "--layer-height (cap_config never carries layer height; it is a print-process setting)",
        "wall_width_mm": args.wall_width_mm,
        "wall_width_source": "--wall-width-mm (a preset value, not in cap_config)",
        "up_axis_arg": args.up_axis,
        "root_end_arg": args.root_end,
        "keep_placement": bool(args.keep_placement),
        "closing_radius_mm": args.closing_radius_mm,
        "latch_walk": bool(args.latch_walk),
        "host_rescues": bool(args.host_rescues),
        "host_largest_island": bool(args.host_largest_island),
        "host_infeasible_drop": bool(args.host_infeasible_drop),
    }
    print("  config: {} (sha256 {})".format(result["config_path"], result["config_sha256"]))
    print("  layer_height_mm={} first_layer_height_mm={} wall_width_mm={} (see assumptions block)".format(
        args.layer_height, first_layer_height, args.wall_width_mm))

    verts, tris = read_binary_stl(args.stl)

    print("\n=== T1: MODEL-LEVEL ===")
    t1 = t1_model_checks(verts, tris)
    result["t1_model"] = t1
    print("  triangles={} vertices={} shells={} closed_two_manifold={}".format(
        t1["triangle_count"], t1["vertex_count"], t1["shell_count"], t1["closed_two_manifold"]))
    if not t1["closed_two_manifold"]:
        print("  WARNING: not closed/two-manifold -- boundary_edges={} nonmanifold_edges={}".format(
            t1["boundary_edge_count"], t1["nonmanifold_edge_count"]))
    print("  raw bbox mm: x=[{min_x:.3f},{max_x:.3f}] y=[{min_y:.3f},{max_y:.3f}] "
          "z=[{min_z:.3f},{max_z:.3f}]".format(**t1["bbox_mm"]))
    if t1["ambiguous_extents"]:
        print("  WARNING: {}".format(t1["ambiguous_extents"]["warning"]))

    print("\n=== STAND-UP ===")
    stand_up = apply_stand_up(
        verts, tris, args.up_axis, args.root_end,
        keep_placement=args.keep_placement)
    result["stand_up"] = {k: v for k, v in stand_up.items() if k != "verts"}
    print("  axis decision: {}".format(stand_up["axis_decision"]["confidence"]))
    if stand_up["axis_decision"]["chosen_axis"] is None:
        result["status"] = "STAND_UP_AMBIGUOUS"
        print("  Refusing to proceed to T2/T3 with an ambiguous stand-up axis.")
        result["t4_blind_spot"] = T4_BLIND_SPOT_TEXT
        print("\n" + T4_BLIND_SPOT_TEXT)
        # Exit 3 has two distinct status strings (STAND_UP_AMBIGUOUS here,
        # UNRESOLVED_STAND_UP from decide_final_status below) -- the module
        # docstring says the STATUS LINE is what tells them apart, so this
        # early return must print the same "=== STATUS: ... (exit N; ...)"
        # line the normal end-of-run path does, not a different message.
        print("\n=== STATUS: {} (exit 3; T4 blind spot always applies -- "
              "this is NOT an in-host guarantee) ===".format(result["status"]))
        if args.json:
            with open(args.json, "w", encoding="utf-8") as fh:
                json.dump(result, fh, indent=2, default=str)
            print("\nWrote {}".format(args.json))
        return 3
    print("  root-end decision: {}".format(stand_up["root_end_decision"]["confidence"]))
    if args.keep_placement:
        print("  --keep-placement: mesh NOT flipped; root end is reported "
              "below (after T3) and only labels/drives the latch walk.")
    print("  post-stand-up bbox mm: {}".format(stand_up["post_stand_up_bbox_mm"]))
    print("  span (Z) mm: {:.4f}".format(stand_up["span_mm"]))

    stood_verts = stand_up["verts"]
    sectioner = MeshSectioner(stood_verts, tris)

    print("\n=== T2: ORIENTATION ===")
    t2 = t2_orientation(sectioner)
    result["t2_orientation"] = t2
    if t2.get("mid_hull_vertices") is not None:
        print("  mid-span hull vertices: {}".format(t2["mid_hull_vertices"]))

    axis_mode = stand_up["axis_decision"]["mode"]
    confound_message = apply_stand_up_confound(axis_mode, t2)
    # Derive ``stand_up_unresolved`` from ``axis_mode`` itself, NOT from
    # ``confound_message is not None``. The confound message exists only when
    # T2's OWN mid-span reading also came back fatal/chord_axis_unavailable
    # (``apply_stand_up_confound``'s own gate), but T2 has an error path
    # ("no outer loop at mid-extent" / "no Z extent") that sets ``t2["error"]`` without setting either flag, so a wing whose
    # stand-up axis fell back to the longest-bbox heuristic (no candidate axis
    # resolved a chord) could reach that T2 branch and print CLEAN_NO_GUARANTEE
    # / exit 0 with an axis that was never confirmed. Tying this directly to
    # ``axis_mode`` closes that path regardless of what shape T2's own failure
    # takes.
    stand_up_unresolved = axis_mode == "auto_fallback_longest_bbox"
    if confound_message:
        print("  UNRESOLVED: {}".format(confound_message))
    elif stand_up_unresolved:
        print(
            "  UNRESOLVED: stand-up axis fallback (longest-bbox heuristic; "
            "see STAND-UP above) -- no candidate axis resolved a chord on "
            "its own mid-extent section, so this axis was never confirmed. "
            "T2's own mid-span reading below is informational only under "
            "this same uncertainty (t2 error: {!r}).".format(t2.get("error"))
        )
    elif t2["fatal"]:
        print("  FATAL: {}".format(t2["fatal_message"]))
    elif t2["chord_axis_unavailable"]:
        print("  chord_axis_unavailable=True: {}".format(t2["error"]))
        print("  This wing will ship UNROTATED, SILENTLY, in-host (coupon "
              "branch). If this is actually a wing, not a coupon, structure "
              "may be built off the true chord axis and the in-host run will "
              "NOT warn.")
    elif t2["error"]:
        print("  ERROR: {}".format(t2["error"]))
    else:
        print("  theta_deg={:.4f} spread_deg={} excluded_stations={}".format(
            t2["theta_deg"], t2["spread_deg"], t2["excluded_stations"]))
        if t2.get("theta_source") == "thin_te_flat":
            _tf = t2["thin_te_flat"]
            _ms = _tf["mid_span"]
            print("  theta_source=thin_te_flat: run {} stations "
                  "({:.2f}..{:.2f}), adjacent-delta max {:.2e} deg (bound {} "
                  "deg); mid-span face/thickness {:.4f}, face/chord {:.5f}, "
                  "corner angles {}".format(
                      _tf["run_stations"], _tf["run_fracs"][0],
                      _tf["run_fracs"][-1], _tf["adjacent_delta_max_deg"],
                      _tf["adjacent_delta_bound_deg"],
                      _ms["face_over_thickness"], _ms["face_over_chord"],
                      ["{:.1f}".format(a) for a in _ms["corner_angles_deg"]]))

    try:
        spec = sa.build_spec_from_cap_config(cap_config)
    except Exception as exc:  # noqa: BLE001
        print("\nCould not build a StructureSpec from --config: {}: {}".format(type(exc).__name__, exc))
        result["status"] = "REFUSED_BAD_CONFIG"
        result["t4_blind_spot"] = T4_BLIND_SPOT_TEXT
        print("\n" + T4_BLIND_SPOT_TEXT)
        if args.json:
            with open(args.json, "w", encoding="utf-8") as fh:
                json.dump(result, fh, indent=2, default=str)
        return 2

    print("\n=== T3: PER-LAYER SWEEP ===")
    t3_start = time.perf_counter()
    if args.keep_placement:
        _latch_mode = {"auto": "auto", "low": "bed", "high": "top"}[args.root_end]
    else:
        _latch_mode = "bed"  # flipped: the root is at the bed by construction
    diagnostics, meta = t3_layer_sweep(
        stood_verts, tris, spec, args.wall_width_mm,
        args.layer_height, first_layer_height, t2,
        closing_radius_mm=args.closing_radius_mm,
        latch_root_mode=_latch_mode if args.latch_walk else None,
        latch_enforce=args.keep_placement,
        host_rescues=args.host_rescues,
        host_largest_island=args.host_largest_island,
        host_infeasible_drop=args.host_infeasible_drop,
    )
    t3_elapsed = time.perf_counter() - t3_start
    summary = diagnostics.to_dict()
    result["t3_meta"] = meta
    result["t3_summary"] = {k: v for k, v in summary.items() if k != "layers"}
    result["t3_layers"] = summary["layers"]
    bands = span_band_breakdown(diagnostics, meta)
    result["t3_span_band_breakdown"] = bands

    print("  n_layers={} root_solid_layers={}".format(meta["n_layers"], meta["root_solid_layers"]))
    print("  layers_seen={} mutated={} failed={} shutdown={} degraded={}".format(
        summary["layers_seen"], summary["mutated_count"], summary["failed_count"],
        summary["shutdown_count"], summary["degraded_layer_count"]))
    print("  degradation_counts_by_kind: {}".format(summary["degradation_counts_by_kind"]))
    print("  node_merge_count={} first_node_merge_z={}".format(
        summary["node_merge_count"], summary["first_node_merge_z"]))
    print("  order_swap_count=0 (NOT reimplemented offline -- see module docstring; "
          "always 0 here, a known gap, not a measured absence)")
    print("  root/mid/tip breakdown: {}".format(bands))
    if summary["first_failure"]:
        ff = summary["first_failure"]
        print("  FIRST FAILURE: layer {} z={:.4f}: {}".format(
            ff["layer_index"], ff["print_z"], ff["message"]))
    print("  T3 wall-clock: {:.1f}s".format(t3_elapsed))
    if t3_elapsed > 180:
        print("  NOTE: full-layer sweep exceeded ~3 minutes; this tool may be too "
              "slow to use as a quick pre-flight on this mesh.")

    # resolved root end + root-end-aware spar-socket latch summary.
    if args.keep_placement:
        mode = {"auto": "auto", "low": "bed", "high": "top"}[args.root_end]
    else:
        mode = "bed"  # flipped: the root is at the bed by construction
    try:
        re_summary = root_end_and_latch_summary(
            stood_verts, meta, mode, enforce=args.keep_placement)
    except RootEndError as exc:
        result["root_end_resolution"] = {"refused": str(exc)}
        result["status"] = "REFUSED_ROOT_END"
        print("\n=== ROOT END ===\n  REFUSED: {}".format(exc))
        print("\n=== STATUS: REFUSED_ROOT_END (exit 1) ===")
        if args.json:
            with open(args.json, "w", encoding="utf-8") as fh:
                json.dump(result, fh, indent=2, default=str)
            print("\nWrote {}".format(args.json))
        return 1
    result["root_end_resolution"] = re_summary
    print("\n=== ROOT END + SPAR-SOCKET LATCH ===")
    print("  root end: {} (root at {}), source={}, area_end={}, channel_end={}".format(
        re_summary["root_end"], re_summary["root_at"], re_summary["decision_source"],
        re_summary["area_signal"].get("end"), re_summary["channel_signal"].get("end")))
    print("  area mm^2 low/high end: {:.3f} / {:.3f}; channel Z span {} .. {} mm, "
          "gap to low/high end {} / {} mm".format(
              re_summary["area_signal"]["area_low_end_mm2"],
              re_summary["area_signal"]["area_high_end_mm2"],
              re_summary["channel_signal"]["channel_z_first_mm"],
              re_summary["channel_signal"]["channel_z_last_mm"],
              re_summary["channel_signal"]["gap_to_low_end_mm"],
              re_summary["channel_signal"]["gap_to_high_end_mm"]))
    if re_summary["auto_would_refuse"]:
        print("  NOTE (flipped mode, not enforced): in the plugin's auto mode this "
              "placement would be REFUSED: {}".format(re_summary["auto_would_refuse"]))
    print("  latch: {} arm_layer={} arm_z={} suppressed={} layers".format(
        re_summary["latch"]["walk_direction"], re_summary["latch"]["arm_index"],
        re_summary["arm_z_mm"], re_summary["latch"]["suppressed_count"]))
    print("  layer counts: {}".format(re_summary["layer_counts"]))
    print("  ROD SOCKET CUT over Z (print_z) runs: {}".format(
        [(r["z_first_mm"], r["z_last_mm"], r["layers"]) for r in re_summary["socket_cut_z_runs"]]))
    print("  socket SUPPRESSED by latch Z runs: {}".format(
        [(r["z_first_mm"], r["z_last_mm"], r["layers"]) for r in re_summary["socket_suppressed_z_runs"]]))
    print("  channel present-but-does-not-fit (not_fit) Z runs: {}".format(
        [(r["z_first_mm"], r["z_last_mm"], r["layers"]) for r in re_summary["not_fit_z_runs"]]))

    result["elapsed_s"] = time.perf_counter() - t_start
    result["t4_blind_spot"] = T4_BLIND_SPOT_TEXT
    print("\n" + T4_BLIND_SPOT_TEXT)

    if args.compare_report:
        cmp = compare_to_host_report(
            result["t3_summary"], args.compare_report, our_theta_deg=t2.get("theta_deg"),
        )
        result["host_comparison"] = cmp
        print_compare_table(cmp)

    # Exit-code scheme (precedence 2 > 3 > 1 > 4 > 0 -- see
    # ``decide_final_status`` / the module docstring Usage block / the argparse
    # epilog, all three of which must stay in sync with this block;
    # ``decide_final_status`` is the one place that actually decides it): 0
    # clean: no refusals, no degradation (T4 caveat still printed) 1 refusals
    # found (layers the host would refuse) 2 tool-level refusal (unreadable
    # STL, bad config, unusable input) -- returned earlier, never reaches this
    # block 3 stand-up not resolved confidently -- BOTH the ambiguous case (>1
    # axis resolves a chord; returned earlier, before T2/T3 even run, as
    # STAND_UP_AMBIGUOUS) AND the low-confidence fallback from
    # ``apply_stand_up_confound`` (T3 still ran and is informational; this
    # block's own precedence). The status string tells them apart; the exit
    # code does not need to. 4 structure lost (degradation) but no refusals
    result["status"], exit_code = decide_final_status(
        stand_up_unresolved, summary["failed_count"], summary["degraded_layer_count"],
    )
    print("\n=== STATUS: {} (exit {}; T4 blind spot always applies -- this "
          "is NOT an in-host guarantee) ===".format(result["status"], exit_code))

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(result, fh, indent=2, default=str)
        print("\nWrote {}".format(args.json))

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
