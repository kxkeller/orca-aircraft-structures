#!/usr/bin/env python3
"""Offline rotation-equivalence harness: real boolean backend, full layer sweep.

Exercises the whole rotation mechanism (canonicalize -> frame -> generate ->
boolean diff -> validate -> de-rotate) against real Clipper-shaped geometry,
which the ``fake_orca`` test suite does not cover.

Built on the pattern of ``tools/d023_arm_sweep.py`` (real
generator/validator/``MiterShapelyOps``, no ``fake_orca``); ``fake_orca`` is
not extended to general polygons.

WHY THIS IS A VALID EQUIVALENCE TEST, even though the mesh is rotated:

For each theta_in in the synthetic matrix, the wing fixture's MESH is
rotated in-plane ABOUT Z ONLY (``run_arm``: XY rotated, Z preserved), so a
horizontal section plane crosses exactly the same triangles at exactly the
same interpolation parameters -- re-sectioning introduces NO discretisation
change. The detector is equivariant to ~1e-14 degrees:
``theta_detected == theta_in + bias`` for every arm, where ``bias`` is the
fixture's own true chord-line reading (NOT near-zero for this fixture --
it carries real ~3 deg built-in washout, so even the theta_in=0 "baseline"
arm has ``rotation_applied: true`` at its own detected bias; do not assume
otherwise). Therefore each arm computes::

    R(-theta_in - bias) . R(theta_in) . v   =   R(-bias) . v

-- ALGEBRAICALLY IDENTICAL to the baseline's own ``R(-bias) . v``. The arms
are not slicing differently-oriented wings; they are slicing the SAME
canonical geometry, differing only by float64 round-off from composing two
rotations instead of one (~1e-16 relative, ~1e-14 mm at 150 mm chord). This
is WHY exact (to float noise) equivalence is the right expectation for
layers where no discrete decision flips, and why a flip that DOES occur
must be explained by a decision boundary sitting within ~1e-14 mm of a
threshold, not by any other mechanism -- see ``_margin_mm``.

Usage:
    python tools/m34_rotation_equivalence.py
    python tools/m34_rotation_equivalence.py --wing-stl <path> --kai
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from collections import Counter

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO_ROOT, os.path.join(REPO_ROOT, "tools"),
           os.path.join(REPO_ROOT, "tests", "integration")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aircraft_core.alignment import (  # noqa: E402
    AlignmentError, ChordAxisUnavailableError, resolve_wing_theta_deg,
    rotate_points_deg,
)
from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import (  # noqa: E402
    DetectedChannel, LayerContext, StructureGenerator,
)
from aircraft_core.mesh_section import MeshSectioner  # noqa: E402
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

from m18_mesh_probe import read_binary_stl  # noqa: E402
from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402
import structural_analysis as sa  # noqa: E402
import fake_orca  # noqa: E402

if HAVE_SHAPELY:
    from shapely.geometry import Polygon as ShapelyPolygon  # noqa: E402
    from shapely.ops import unary_union  # noqa: E402

LAYER_HEIGHT_MM = 0.2
LIVE_CONFIG_PATH = os.path.join(
    REPO_ROOT, "host", "data", "orca_plugins", "config.json")
DEFAULT_STL = os.path.join(
    REPO_ROOT, "tests", "fixtures", "wing", "naca2412_wing_r150_t90_s400_w3.stl")
# The --kai / --kai-only modes need a real wing STL that is not part of this
# repository. Supply it with --wing-stl or this environment variable; those
# modes exit with a clear message when neither is given.
WING_STL_ENV = "AIRCRAFT_WING_STL"

THETA_EPSILON_DEG = 0.2  # must match plugin/aircraft_structures/capability.py
STATION_FRACS = (0.1, 0.3, 0.7, 0.9)
WALL_MM = 0.42

# Acceptance criterion, in three clauses:
#
# 1. GEOMETRY (layers with NO flipped decision): symmetric-difference area
#    between baseline and arm (order/ring-start/vertex-count invariant)
#    must be below NONFLIP_AREA_FRACTION * wall_mm * that
#    layer's own perimeter. NONFLIP_AREA_FRACTION is 1e-6: at that
#    fraction, for a typical ~300 mm perimeter and 0.42 mm wall, the bound
#    is ~1.26e-4 mm^2 -- many orders of magnitude above the ~1e-14 mm
#    float-composition noise the equivariance identity above predicts, and
#    many orders below anything a real one-station geometry difference
#    could produce (a single flipped web changes area by wall_mm times a
#    local span, i.e. order 1 mm^2 or more) -- so this bound cannot be
#    satisfied by a real defect passing as noise, only by genuine
#    equivalence.
# 2. DECISIONS (flips): permitted ONLY if the flipped station's margin to
#    its own decision threshold (``_margin_mm``) is below
#    FLIP_MARGIN_TOLERANCE_MM (1e-6 mm, matching clause 1's noise floor).
#    A flip with a healthy (non-tiny) margin is impossible under the
#    equivariance identity and therefore signals a real bug, not chatter.
# 3. OUTCOMES: ``failed`` (TopologyError count) must be IDENTICAL across
#    every arm -- no layer may pass in one orientation and fail in
#    another. This is the clause that protects against a silent wrong result.
#
# WHY EXACT DECISION EQUALITY IS UNACHIEVABLE IN PRINCIPLE (not just
# unmeasured): the include/omit decision is a THRESHOLD comparison (``if sign *
# (margin_y - local_entry_y) >= 0``-shaped code throughout generator.py)
# applied to values that differ by float64 round-off between arms (the
# two-rotation composition above). For ANY discontinuous threshold and ANY
# nonzero (however small) perturbation, there exists a station whose true value
# sits within that perturbation of the threshold -- for a large enough
# population of stations (13521 in this sweep), the pigeonhole probability of
# at least one such near-threshold station approaches certainty, independent of
# implementation quality. The fusion window is likewise tight by construction,
# and this sweep exposes the same property in the generator's own omission
# thresholds. Demanding zero flips would be
# demanding something no floating-point implementation of a discontinuous
# decision can promise; demanding EVERY flip be provably near-threshold is the
# achievable, meaningful bar.
NONFLIP_AREA_FRACTION = 1e-6
FLIP_MARGIN_TOLERANCE_MM = 1e-6


def load_spec(path=None):
    """Build the StructureSpec from a host-style config JSON.

    ``path`` defaults to ``LIVE_CONFIG_PATH`` (the host's installed config);
    pass an explicit path, e.g. ``presets/plugin/live_host_config.json``, to
    read a different file.
    """
    if path is None:
        path = LIVE_CONFIG_PATH
    with open(path, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    entry = doc["config"][0] if isinstance(doc["config"], list) else doc["config"]
    return sa.build_spec_from_cap_config(entry["cap_config"])


def _mesh_bbox_center(verts):
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    return (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0


def _drop_to_bed(verts):
    """Translate ``verts`` so the mesh's own minimum Z sits at 0, matching the
    real host: it drops the object onto the bed before slicing, so ``print_z``
    is measured from the OBJECT'S OWN bottom, never from the STL file's raw
    coordinate origin. Without this, ``sec.z_min`` in ``_sweep_layers`` can sit
    far above 0 (24.307 mm for the 450 mm user wing after stand-up), and the
    ``z <= z_min`` filter there silently drops every layer index below that --
    both an unattributed-layer count (the bug this fixes) and a systematic
    offset between "offline layer i" and "in-host layer i" (they stop meaning
    the same station).

    Idempotent: a mesh already sitting at z_min == 0 (or within float
    noise of it) is returned unchanged, so calling this in more than one
    place (``run_arm`` AND ``run_control_pair``) is safe and necessary --
    both must see the same convention or their two sides compare layer
    index ``i`` at two different absolute Z heights.
    """
    z_min = min(v[2] for v in verts)
    if z_min == 0.0:
        return verts
    return [(x, y, z - z_min) for x, y, z in verts]


def _detect_theta(sec):
    """Exact mirror of capability.py's ``_build_mesh_sectioner`` theta
    detection: mid-span (50%) station authoritative, 4 interior stations
    bound the spread. Returns (theta_deg_or_None, detail_dict)."""
    z_lo, z_hi = sec.z_min, sec.z_max
    detail = {"error": None, "mid_span_theta_deg": None, "spread_deg": None,
              "station_theta_deg": [], "chord_axis_unavailable": False}
    if z_lo is None or z_hi is None or z_hi <= z_lo:
        detail["error"] = "mesh has no Z extent"
        return None, detail

    def outer_pts(frac):
        z = z_lo + frac * (z_hi - z_lo)
        loops = sec.section_at(z)
        outer = [lp for lp in loops if not lp.is_interior]
        if not outer:
            return None
        best = max(outer, key=lambda lp: abs(lp.signed_area))
        return list(best.points)

    mid_pts = outer_pts(0.5)
    if mid_pts is None:
        detail["error"] = "mid-span section has no outer loop"
        return None, detail
    other_pts = [p for p in (outer_pts(f) for f in STATION_FRACS) if p is not None]
    try:
        theta, stations, spread, excluded = resolve_wing_theta_deg(mid_pts, other_pts)
        detail["mid_span_theta_deg"] = theta
        detail["station_theta_deg"] = stations
        detail["spread_deg"] = spread
        detail["excluded_stations"] = excluded
        return theta, detail
    except ChordAxisUnavailableError as exc:
        detail["error"] = str(exc)
        detail["chord_axis_unavailable"] = True
        return None, detail
    except AlignmentError as exc:
        detail["error"] = str(exc)
        detail["fatal"] = True
        return None, detail


def _station_key(family_name, x_mm):
    return (str(family_name), round(x_mm, 6))


# Parse the numeric margin (mm, POSITIVE means the station
# cleared its threshold, i.e. an omission with a small POSITIVE margin was
# a near-miss) out of the three known OmittedWeb reason shapes in
# generator.py. Returns None for any reason shape not recognised --
# reported honestly as "margin: unavailable", never guessed.
_RE_SKIN_MARGIN = re.compile(
    r"skin margin: local thickness ([\d.]+) mm .*? skin_margin_mm=([\d.]+)")
_RE_THIN_SECTION = re.compile(
    r"thin section: remaining skin ([\d.]+) mm .*?\(([\d.]+) mm required")
_RE_CHORD_EXTENT = re.compile(
    r"chord extent: slot footprint \[(-?[\d.]+), (-?[\d.]+)\] mm does not "
    r"lie within the section's chord extent \[(-?[\d.]+), (-?[\d.]+)\] mm")
# The ordinary "shallower loses" opposing-entry branch ONLY (generator.py
# ~2707-2710: ``elif meeting.d_upper_mm < meeting.d_lower_mm:... elif
# meeting.d_lower_mm < meeting.d_upper_mm:... elif u_intent.family_index >=
# l_intent.family_index:...`` -- a STRICT less-than comparison with NO epsilon
# tolerance anywhere in it). Deliberately excludes the "meets the declared
# spar-hole scar" (scar-priority, not a depth comparison) and "could not be
# measured" (boundary-halo failure, treated as an unconditional conflict)
# branches, whose reason text does not match "meets family... depth... mm)
# over".
_RE_OPPOSING_ENTRY = re.compile(
    r"opposing entry: candidate at x=-?[\d.]+ mm \([+-]Y, depth ([\d.]+) mm\) "
    r"meets family .*?depth ([\d.]+) mm\) over the overlap span")


def _margin_mm(reason):
    """Returns ``(margin_mm, precision_note)``. ``margin_mm`` is ``None``
    for any reason shape not recognised -- reported honestly as
    "margin: unavailable", never guessed. For the "skin margin"/"thin
    section"/"chord extent" shapes the reason string's own printed
    numbers ARE the decision inputs, so the margin is exact (to the
    string's 4-decimal print precision). For "opposing entry" the reason
    text prints each side's depth to 4 decimals only -- the ACTUAL
    comparison (generator.py's ``d_upper_mm < d_lower_mm``) runs on full
    float64 precision this harness does not have access to without
    instrumenting generator.py. The returned margin is therefore only a 4-decimal-
    precision BOUND (|margin_mm| < 5e-5 whenever the printed depths are
    equal), not a full-precision measurement -- ``precision_note`` says so
    explicitly so a caller cannot mistake it for one.
    """
    if not reason:
        return None, None
    m = _RE_SKIN_MARGIN.search(reason)
    if m:
        return float(m.group(1)) - float(m.group(2)), "exact (decision inputs)"
    m = _RE_THIN_SECTION.search(reason)
    if m:
        return float(m.group(1)) - float(m.group(2)), "exact (decision inputs)"
    m = _RE_CHORD_EXTENT.search(reason)
    if m:
        fmin, fmax, cmin, cmax = (float(g) for g in m.groups())
        return min(fmin - cmin, cmax - fmax), "exact (decision inputs)"
    m = _RE_OPPOSING_ENTRY.search(reason)
    if m:
        loser_depth, winner_depth = float(m.group(1)), float(m.group(2))
        return (winner_depth - loser_depth,
                "BOUND ONLY: reason text prints 4 decimals; true decision "
                "runs on full float64 precision not available here (see "
                "docstring)")
    return None, None


def _sweep_layers(sec, spec, wall_mm, cap, ops, ship_rotate_deg):
    """Core per-layer sweep, shared by every arm (rotation matrix AND the
    disabled/enabled control). ``ship_rotate_deg`` is the SINGLE net rotation
    (already composed) applied to the validated replacement pieces before they
    are recorded -- 0.0 for a run with no involvement or an epsilon-gated
    no-op.

    Returns aggregate counts, per-layer shapely piece lists (for
    symmetric-difference comparison), and per-layer candidate-station
    records (for flip/margin detection).
    """
    z_min, z_max = sec.z_min, sec.z_max
    root = int(spec.root_solid_layers or 0)
    n_layers = int(round(z_max / LAYER_HEIGHT_MM))

    gen, builder = StructureGenerator(), SlotBuilder()
    validator = TopologyValidator(ops, safety_offset_mm=wall_mm)

    built_by_family = Counter()
    omitted_by_family = Counter()
    node_merge_total = 0
    order_swap_total = 0
    failures = []
    ok = 0
    # Every index in range(root, n_layers) must land in exactly one bucket: ok,
    # a failure, or one of these two skip buckets -- never silently `continue`d
    # out of every tally (the bug this replaces: 5 root + 117
    # unattributed-skipped-below-z_min layers were missing from ok+failed
    # entirely). Tracked, not just counted, so a caller can see WHICH layer/z
    # was skipped and why.
    skipped_out_of_z_range = []  # z <= z_min or z >= z_max (mesh cap/tip)
    skipped_no_outer_loop = []   # section_at returned no outer loop at all
    per_layer_shipped = {}     # layer -> [shapely Polygon, ...] (net-rotated)
    per_layer_candidates = {}  # layer -> {station_key: (status, reason)}

    for i in range(root, n_layers):
        z = i * LAYER_HEIGHT_MM
        if z <= z_min or z >= z_max:
            skipped_out_of_z_range.append({"layer": i, "z": round(z, 4)})
            continue
        try:
            loops = sec.section_at(z)
            outer = [lp for lp in loops if not lp.is_interior]
            inner = [lp for lp in loops if lp.is_interior]
            if not outer:
                skipped_no_outer_loop.append({"layer": i, "z": round(z, 4)})
                continue
            poly = ops.polygon_from_points(
                max(outer, key=lambda lp: abs(lp.signed_area)).points)
            frame = LayerFrame.from_polygon(ops, poly)

            channels = tuple(
                DetectedChannel(lp.centroid[0], lp.centroid[1],
                                 lp.equivalent_diameter_mm) for lp in inner)
            res = gen.generate(
                frame, spec, wall_mm,
                LayerContext(layer_index=i, print_z=z, detected_channels=channels),
                safety_offset_mm=wall_mm)

            candidates = {}
            for it in res.intents:
                built_by_family[str(it.family_name)] += 1
                candidates[_station_key(it.family_name, it.start[0])] = ("built", None)
            node_merge_total += sum(1 for it in res.intents if it.merged_family_indices)
            events = cap._classify_declared_pair_overlaps(
                list(res.intents), spec, frame, i, z)
            order_swap_total += len(events)
            for o in res.omitted:
                omitted_by_family[str(getattr(o, "family_name", "?"))] += 1
                candidates[_station_key(o.family_name, o.x_mm)] = ("omitted", o.reason)
            per_layer_candidates[i] = candidates

            slots = [ops.polygon_from_points(
                builder.build(it, spec.overshoot_mm).points)
                for it in res.intents]
            repl = ops.diff(poly, slots)
            validator.validate(
                [poly], repl, slots, spec, i, z,
                slot_labels=[str(it.family_name) for it in res.intents],
                family_indices=[it.family_membership() for it in res.intents])

            # Build shapely pieces (holes included via hole_points_mm) rotated
            # by the SINGLE composed ship_rotate_deg, for symmetric-difference
            # comparison rather than positional point-list zipping.
            #
            # `ok` is counted only AFTER this block succeeds: if
            # `ShapelyPolygon(...)` itself raised, the layer would otherwise
            # count as both `ok` and `failed` (caught below), breaking the
            # root+ok+failed+skipped == n_layers reconciliation.
            shipped = []
            for piece in repl:
                shell = ops.contour_points_mm(piece)
                holes = ops.hole_points_mm(piece)
                if ship_rotate_deg:
                    shell = rotate_points_deg(shell, ship_rotate_deg)
                    holes = [rotate_points_deg(h, ship_rotate_deg) for h in holes]
                shipped.append(ShapelyPolygon(shell, holes))
            per_layer_shipped[i] = shipped
            ok += 1
        except TopologyError as exc:
            failures.append({"layer": i, "z": round(z, 4), "kind": "TopologyError",
                              "msg": str(exc)})
        except Exception as exc:  # noqa: BLE001
            failures.append({"layer": i, "z": round(z, 4),
                              "kind": type(exc).__name__, "msg": str(exc)[:300]})

    # Every index in [0, n_layers) must land in exactly one bucket: the `root`
    # layers are never iterated at all (by construction, range starts at
    # `root`); everything from `root` to `n_layers` is ok, failed, or one of
    # the two tracked skip buckets. No index may vanish from every tally --
    # that silent disappearance (e.g. layers unattributed because z_min was
    # not normalized) is the defect this accounting guards against.
    n_skipped = len(skipped_out_of_z_range) + len(skipped_no_outer_loop)
    n_accounted = root + ok + len(failures) + n_skipped
    if n_accounted != n_layers:
        raise AssertionError(
            "layer reconciliation failed: root(%d) + ok(%d) + failed(%d) + "
            "skipped(%d) = %d != n_layers(%d)" % (
                root, ok, len(failures), n_skipped, n_accounted, n_layers))

    return {
        "root": root, "n_layers": n_layers, "ok": ok, "failed": len(failures),
        "failures_sample": failures[:10],
        "skipped_out_of_z_range": len(skipped_out_of_z_range),
        "skipped_out_of_z_range_sample": skipped_out_of_z_range[:10],
        "skipped_no_outer_loop": len(skipped_no_outer_loop),
        "skipped_no_outer_loop_sample": skipped_no_outer_loop[:10],
        "reconciled": n_accounted == n_layers,
        "built_by_family": dict(built_by_family),
        "omitted_by_family": dict(omitted_by_family),
        "node_merge_count": node_merge_total,
        "order_swap_count": order_swap_total,
        "per_layer_shipped": per_layer_shipped,
        "per_layer_candidates": per_layer_candidates,
    }


def run_arm(theta_in_deg, verts, tris, spec, wall_mm, cap, ops):
    """One rotation-matrix arm: inject theta_in_deg, run the full mechanism, full
    layer sweep."""
    # Drop to bed FIRST: rotation below is about Z only (Z unaffected), so
    # this must happen before or after the rotation with the same result,
    # but it must happen before `_sweep_layers` sees `sec.z_min`. See
    # `_drop_to_bed`'s own docstring for why this is not optional.
    verts = _drop_to_bed(verts)
    cx, cy = _mesh_bbox_center(verts)
    centered = [(v[0] - cx, v[1] - cy, v[2]) for v in verts]
    injected = rotate_points_deg([(v[0], v[1]) for v in centered], theta_in_deg)
    rotated_verts = [(x, y, v[2]) for (x, y), v in zip(injected, centered)]

    probe = MeshSectioner(rotated_verts, tris)
    theta_detected, detail = _detect_theta(probe)

    rotation_applied = False
    canonical_verts = rotated_verts
    if theta_detected is not None and abs(theta_detected) >= THETA_EPSILON_DEG:
        xy = rotate_points_deg([(v[0], v[1]) for v in rotated_verts], -theta_detected)
        canonical_verts = [(x, y, v[2]) for (x, y), v in zip(xy, rotated_verts)]
        rotation_applied = True
    theta_applied = theta_detected if rotation_applied else 0.0

    sec = MeshSectioner(canonical_verts, tris)
    # Single composed rotation: +theta_applied
    # (world frame) then -theta_in_deg (undo ONLY the synthetic test
    # rotation) collapses to one net angle, computed once rather than as
    # two separate rotate_points_deg calls.
    ship_rotate_deg = theta_applied - theta_in_deg
    result = _sweep_layers(sec, spec, wall_mm, cap, ops, ship_rotate_deg)
    result.update({
        "theta_in_deg": theta_in_deg,
        "theta_detected_deg": theta_detected,
        "theta_applied_deg": theta_applied,
        "rotation_applied": rotation_applied,
        "detail": detail,
    })
    return result


def run_control_pair(verts, tris, spec, wall_mm, cap, ops):
    """Control: mechanism DISABLED vs ENABLED at a
    genuinely sub-epsilon angle, same fixture. Answers "does the mechanism
    perturb geometry at all when it should be a no-op", distinct from the
    rotation-matrix arms' "does de-rotating a real rotation reproduce the
    baseline" question.

    The fixture's own bias is NOT near zero (it carries real washout), so
    "genuinely sub-epsilon" is achieved by choosing theta_in to cancel
    that bias down to ~0.05 deg, not by using theta_in=0 (which would
    still trigger rotation_applied=True, as the r150 baseline does).
    """
    # Same drop-to-bed as `run_arm` (idempotent) -- (b) below calls
    # `run_arm` on the ORIGINAL `verts`, so (a) here must apply the same
    # normalization itself or the two sides sweep layer index `i` at two
    # different absolute Z heights, which `_flips_and_geometry` would
    # misreport as `layers_missing_from_one_side`.
    verts = _drop_to_bed(verts)
    cx, cy = _mesh_bbox_center(verts)
    centered = [(v[0] - cx, v[1] - cy, v[2]) for v in verts]

    probe0 = MeshSectioner([(v[0], v[1], v[2]) for v in centered], tris)
    theta0, _detail0 = _detect_theta(probe0)
    if theta0 is None:
        raise RuntimeError("control pair: cannot detect baseline bias")
    target_subepsilon_deg = 0.05
    theta_in_for_subepsilon = target_subepsilon_deg - theta0

    # (a) MECHANISM DISABLED: no theta detection, no rotation at all --
    # plain pipeline, straight section -> generate -> diff -> validate
    # on the mesh exactly as rotated for the test (theta_in applied, same
    # as (b) would see, so both operate on an IDENTICAL mesh).
    injected = rotate_points_deg([(v[0], v[1]) for v in centered], theta_in_for_subepsilon)
    test_verts = [(x, y, v[2]) for (x, y), v in zip(injected, centered)]
    sec_disabled = MeshSectioner(test_verts, tris)
    # ship_rotate_deg = -theta_in_for_subepsilon, NOT 0.0: (b) below goes
    # through run_arm, which (correctly, for the rotation-matrix arms)
    # undoes the synthetic test rotation before recording shipped pieces
    # -- when rotation_applied is False (the epsilon-gated case here),
    # run_arm's own ship_rotate_deg is theta_applied(0.0) - theta_in ==
    # -theta_in_for_subepsilon. (a) must use the SAME convention or the
    # two sides are compared in frames rotated relative to each other by
    # theta_in_for_subepsilon, producing a spurious area/Hausdorff
    # "difference" that is purely a comparison-frame mismatch.
    disabled = _sweep_layers(sec_disabled, spec, wall_mm, cap, ops,
                              ship_rotate_deg=-theta_in_for_subepsilon)

    # (b) MECHANISM ENABLED at the sub-epsilon angle: full code path runs
    # (theta detection executes), but epsilon-gates to rotation_applied =
    # False, so canonical_verts == test_verts -- IDENTICAL input mesh to (a).
    # Any difference in the OUTPUT is caused by the mechanism itself having a
    # side effect despite not rotating, which would be a real bug.
    enabled = run_arm(theta_in_for_subepsilon, verts, tris, spec, wall_mm, cap, ops)

    return {
        "target_subepsilon_deg": target_subepsilon_deg,
        "theta_in_for_subepsilon": theta_in_for_subepsilon,
        "baseline_bias_deg": theta0,
        "disabled": disabled,
        "enabled": enabled,
    }


def _flips_and_geometry(baseline, arm, wall_mm):
    """Symmetric-difference area (order/ring-start/
    vertex-count invariant) for layers with no flipped decision, and
    explicit per-flip margins for layers that do have one. Returns
    (ok: bool, report: dict)."""
    report = {
        "count_mismatches": [],
        "flips": [],
        "worst_nonflip_area_mm2": 0.0,
        "worst_nonflip_area_layer": None,
        "worst_hausdorff_mm": 0.0,
        "worst_hausdorff_layer": None,
        "layers_missing_from_one_side": [],
    }
    ok = True

    for field in ("built_by_family", "omitted_by_family", "node_merge_count",
                  "order_swap_count", "ok", "failed"):
        if baseline[field] != arm[field]:
            report["count_mismatches"].append(
                {"field": field, "baseline": baseline[field], "arm": arm[field]})
    # `failed` disagreement is fatal on its own, always.
    if baseline["failed"] != arm["failed"]:
        ok = False

    base_layers = baseline["per_layer_shipped"]
    arm_layers = arm["per_layer_shipped"]
    base_cand = baseline["per_layer_candidates"]
    arm_cand = arm["per_layer_candidates"]
    common = sorted(set(base_layers) & set(arm_layers))
    if set(base_layers) != set(arm_layers):
        report["layers_missing_from_one_side"] = sorted(
            set(base_layers) ^ set(arm_layers))
        ok = False

    for i in common:
        bc = base_cand.get(i, {})
        ac = arm_cand.get(i, {})
        layer_flips = []
        for key in sorted(set(bc) | set(ac)):
            bstat = bc.get(key)
            astat = ac.get(key)
            if bstat is None or astat is None or bstat[0] == astat[0]:
                continue
            reason = bstat[1] if bstat[0] == "omitted" else astat[1]
            margin, precision = _margin_mm(reason)
            flip = {
                "layer": i, "family": key[0], "x_mm": key[1],
                "baseline_status": bstat[0], "arm_status": astat[0],
                "reason": reason, "margin_mm": margin,
                "margin_precision": precision,
            }
            layer_flips.append(flip)
            report["flips"].append(flip)
            # Margin gate. Two tiers, because "opposing entry" only
            # yields a 4-decimal-print BOUND, not a full-precision
            # measurement (see _margin_mm's own docstring) -- gating it at
            # the same 1e-6 mm tolerance as an EXACT margin would be a
            # false precision claim in the other direction. A bound is
            # accepted only up to the print resolution itself (5e-5 mm,
            # i.e. the printed numbers were literally equal) and flagged
            # UNCONFIRMED (not "passed") rather than silently counted as
            # equivalent to a sub-1e-6-mm exact measurement.
            if margin is None:
                ok = False  # unparsed reason shape -- cannot assess at all
            elif precision == "exact (decision inputs)":
                if abs(margin) > FLIP_MARGIN_TOLERANCE_MM:
                    ok = False
            else:  # BOUND ONLY (opposing entry)
                flip["gate"] = "unconfirmed_precision"
                if abs(margin) > 5e-5:
                    ok = False  # even the coarse bound is not near threshold

        if layer_flips:
            continue  # geometry EXPECTED to differ here; margin gate covers it

        base_union = unary_union(base_layers[i]) if base_layers[i] else None
        arm_union = unary_union(arm_layers[i]) if arm_layers[i] else None
        if base_union is None and arm_union is None:
            continue
        if base_union is None or arm_union is None:
            report["layers_missing_from_one_side"].append(i)
            ok = False
            continue
        sym_area = base_union.symmetric_difference(arm_union).area
        perimeter_mm = base_union.length
        tolerance_mm2 = NONFLIP_AREA_FRACTION * wall_mm * max(perimeter_mm, 1.0)
        if sym_area > report["worst_nonflip_area_mm2"]:
            report["worst_nonflip_area_mm2"] = sym_area
            report["worst_nonflip_area_layer"] = i
        if sym_area > tolerance_mm2:
            ok = False
        hd = base_union.hausdorff_distance(arm_union)
        if hd > report["worst_hausdorff_mm"]:
            report["worst_hausdorff_mm"] = hd
            report["worst_hausdorff_layer"] = i

    if report["count_mismatches"]:
        # built/omitted totals differing is EXPECTED exactly when flips
        # exist and must reconcile with them; still surfaced, not fatal
        # on its own (the per-flip margin gate is the real judge).
        pass
    return ok, report


def run_matrix(stl_path, label, thetas, out_path):
    if not HAVE_SHAPELY:
        raise SystemExit("shapely unavailable; refusing to report")
    spec = load_spec()
    wall_mm = WALL_MM

    fo, restore = fake_orca.install()
    try:
        namespace = fake_orca.exec_bundle(
            os.path.join(REPO_ROOT, "plugin", "aircraft_structures", "aircraft_structures.py"))
        cap = namespace["AircraftStructuresM1"]()
        ops = MiterShapelyOps()

        verts, tris = read_binary_stl(stl_path)

        arms = {}
        t0 = time.perf_counter()
        for theta_in in thetas:
            arms[theta_in] = run_arm(theta_in, verts, tris, spec, wall_mm, cap, ops)
        control = run_control_pair(verts, tris, spec, wall_mm, cap, ops)
        elapsed = time.perf_counter() - t0

        baseline = arms[0.0]
        comparisons = {}
        all_ok = True
        for theta_in in thetas:
            if theta_in == 0.0:
                continue
            ok, report = _flips_and_geometry(baseline, arms[theta_in], wall_mm)
            comparisons[theta_in] = report
            all_ok = all_ok and ok

        control_ok, control_report = _flips_and_geometry(
            control["disabled"], control["enabled"], wall_mm)
        all_ok = all_ok and control_ok

        failed_counts = {str(t): a["failed"] for t, a in arms.items()}
        failed_counts["control_disabled"] = control["disabled"]["failed"]
        failed_counts["control_enabled"] = control["enabled"]["failed"]
        all_failed_equal = len(set(failed_counts.values())) == 1

        summary = {
            "label": label,
            "stl": stl_path,
            "wall_mm": wall_mm,
            "nonflip_area_fraction": NONFLIP_AREA_FRACTION,
            "flip_margin_tolerance_mm": FLIP_MARGIN_TOLERANCE_MM,
            "elapsed_s": elapsed,
            "per_layer_time_ms": (
                1000.0 * elapsed / max(1, sum(a["ok"] + a["failed"] for a in arms.values()))
            ),
            "arms": {
                str(t): {
                    "theta_in_deg": a["theta_in_deg"],
                    "theta_detected_deg": a["theta_detected_deg"],
                    "theta_applied_deg": a["theta_applied_deg"],
                    "rotation_applied": a["rotation_applied"],
                    "ok": a["ok"], "failed": a["failed"],
                    "failures_sample": a["failures_sample"],
                    "node_merge_count": a["node_merge_count"],
                    "order_swap_count": a["order_swap_count"],
                    "built_by_family": a["built_by_family"],
                    "omitted_by_family": a["omitted_by_family"],
                    "detail": a["detail"],
                } for t, a in arms.items()
            },
            "comparisons_to_baseline": {str(k): v for k, v in comparisons.items()},
            "control_pair": {
                "target_subepsilon_deg": control["target_subepsilon_deg"],
                "theta_in_for_subepsilon": control["theta_in_for_subepsilon"],
                "baseline_bias_deg": control["baseline_bias_deg"],
                "disabled_failed": control["disabled"]["failed"],
                "enabled_failed": control["enabled"]["failed"],
                "enabled_rotation_applied": control["enabled"]["rotation_applied"],
                "enabled_theta_detected_deg": control["enabled"]["theta_detected_deg"],
                "comparison": control_report,
                "ok": control_ok,
            },
            "failed_counts_all_arms": failed_counts,
            "failed_counts_identical_across_all_arms": all_failed_equal,
            "all_arms_equivalent": all_ok and all_failed_equal,
        }
        with open(out_path, "w", encoding="utf-8") as fh:
            json.dump(summary, fh, indent=2, default=str)

        print("ROTATION EQUIVALENCE:", label)
        print("  stl:", stl_path)
        print("  elapsed: %.2f s (%.3f ms/layer)" % (elapsed, summary["per_layer_time_ms"]))
        for t in thetas:
            a = arms[t]
            print("  theta_in=%7.3f  detected=%s  applied=%7.3f  ok=%d failed=%d  "
                  "merges=%d swaps=%d" % (
                      t, "None" if a["theta_detected_deg"] is None else "%.4f" % a["theta_detected_deg"],
                      a["theta_applied_deg"], a["ok"], a["failed"],
                      a["node_merge_count"], a["order_swap_count"]))
        print("  CONTROL PAIR (sub-epsilon %.3f deg, theta_in=%.6f): "
              "disabled failed=%d, enabled failed=%d, enabled rotation_applied=%s" % (
                  control["target_subepsilon_deg"], control["theta_in_for_subepsilon"],
                  control["disabled"]["failed"], control["enabled"]["failed"],
                  control["enabled"]["rotation_applied"]))
        print("  FAILED COUNTS, ALL ARMS:", failed_counts)
        print("  FAILED COUNTS IDENTICAL ACROSS ALL ARMS:", all_failed_equal)
        print("  ALL ARMS EQUIVALENT (geometry + flip-margin + failed-count gates):", summary["all_arms_equivalent"])
        for t, rep in comparisons.items():
            if rep["flips"]:
                print("  theta_in=%s: %d flip(s):" % (t, len(rep["flips"])))
                for f in rep["flips"][:20]:
                    print("     layer=%d family=%s x=%.4f %s->%s margin_mm=%s"
                          % (f["layer"], f["family"], f["x_mm"], f["baseline_status"],
                             f["arm_status"], f["margin_mm"]))
            if not summary["all_arms_equivalent"] and (
                    rep["count_mismatches"] or rep["layers_missing_from_one_side"]
                    or rep["worst_nonflip_area_mm2"] > NONFLIP_AREA_FRACTION * WALL_MM):
                print("  ISSUE theta_in=%s: worst_nonflip_area_mm2=%.3e (layer %s), "
                      "worst_hausdorff_mm=%.3e (layer %s), missing=%s" % (
                          t, rep["worst_nonflip_area_mm2"], rep["worst_nonflip_area_layer"],
                          rep["worst_hausdorff_mm"], rep["worst_hausdorff_layer"],
                          rep["layers_missing_from_one_side"]))
        print("  wrote", out_path)
        return summary
    finally:
        restore()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--stl", default=DEFAULT_STL)
    parser.add_argument("--wing-stl", default=None,
                         help="real wing STL for --kai/--kai-only (default: "
                              "the AIRCRAFT_WING_STL environment variable)")
    parser.add_argument("--kai", action="store_true",
                         help="also run the real wing STL (--wing-stl), stood up (span -> Z)")
    parser.add_argument("--kai-only", action="store_true",
                         help="skip the r150 matrix, run only the real wing STL (--wing-stl)")
    parser.add_argument(
        "--force", action="store_true",
        help="allow --kai/--kai-only to overwrite an existing "
             "docs/results/m34_kai_wing_result.json. That file is committed "
             "and cited by name (ok/failed/n_layers figures) in "
             "other records; without --force the run refuses to touch it "
             "if it already exists, so a re-run with different numbers "
             "(e.g. after the z_min normalization fix) cannot silently "
             "rewrite cited evidence out from under the doc that quotes it.")
    args = parser.parse_args(argv)

    ok = True
    if not args.kai_only:
        thetas = [0.0, 0.3, 15.0, 45.0, 90.0, -90.0, 180.0, -63.36, 137.0]
        out1 = os.path.join(REPO_ROOT, "docs", "results", "m34_rotation_equivalence_r150.json")
        summary1 = run_matrix(args.stl, "r150 reference wing", thetas, out1)
        ok = summary1["all_arms_equivalent"]

    if args.kai or args.kai_only:
        wing_stl = args.wing_stl or os.environ.get(WING_STL_ENV)
        if not wing_stl:
            raise SystemExit(
                "--kai/--kai-only need a real wing STL: pass --wing-stl <path> "
                "or set %s" % WING_STL_ENV)
        verts, tris = read_binary_stl(wing_stl)
        stood = [(-v[2], v[1], v[0]) for v in verts]
        report = run_kai(stood, tris, force=args.force, stl_path=wing_stl)
        ok = ok and report is not None

    return 0 if ok else 1


def run_kai(stood_verts, tris, force=False, stl_path=None):
    """Run the real wing STL (stood up, span -> Z) and write the result JSON.

    ``docs/results/m34_kai_wing_result.json`` may be a committed artifact
    whose ok/failed/n_layers figures are quoted elsewhere, and a re-run can
    produce different numbers at the same path. An existing file is therefore
    not overwritten unless the caller passes ``--force``, an explicit decision
    at the call site.
    """
    if not HAVE_SHAPELY:
        raise SystemExit("shapely unavailable; refusing to report")
    spec = load_spec()
    wall_mm = WALL_MM
    out_path = os.path.join(REPO_ROOT, "docs", "results", "m34_kai_wing_result.json")
    if os.path.exists(out_path) and not force:
        raise SystemExit(
            "refusing to overwrite existing %s without --force: this file "
            "is committed and its ok/failed/n_layers figures are quoted by "
            "name elsewhere. Re-run with --force only after "
            "confirming the citing doc will be updated to match the new "
            "numbers." % out_path)
    fo, restore = fake_orca.install()
    try:
        namespace = fake_orca.exec_bundle(
            os.path.join(REPO_ROOT, "plugin", "aircraft_structures", "aircraft_structures.py"))
        cap = namespace["AircraftStructuresM1"]()
        ops = MiterShapelyOps()
        result = run_arm(0.0, stood_verts, tris, spec, wall_mm, cap, ops)
        out = {
            "stl": stl_path,
            "stand_up_transform": "rotate about Y by -90 deg: "
                                   "new_x=-old_z, new_y=old_y, new_z=old_x",
            "theta_detected_deg": result["theta_detected_deg"],
            "theta_applied_deg": result["theta_applied_deg"],
            "rotation_applied": result["rotation_applied"],
            "root": result["root"],
            "n_layers": result["n_layers"],
            "ok": result["ok"],
            "failed": result["failed"],
            "failures_sample": result["failures_sample"],
            "detail": result["detail"],
        }
        with open(out_path, "w", encoding="utf-8") as fh:
            json.dump(out, fh, indent=2, default=str)
        print("KAI'S REAL WING, STOOD UP, FULL SWEEP:")
        print("  detected theta: %.5f deg (applied %.5f, rotation_applied=%s)" % (
            result["theta_detected_deg"] or -1.0, result["theta_applied_deg"],
            result["rotation_applied"]))
        print("  layers: root=%d n_layers=%d ok=%d failed=%d" % (
            result["root"], result["n_layers"], result["ok"], result["failed"]))
        if result["failures_sample"]:
            print("  FIRST FAILURES:")
            for f in result["failures_sample"]:
                print("   ", f)
        print("  wrote", out_path)
        return out
    finally:
        restore()


if __name__ == "__main__":
    raise SystemExit(main())
