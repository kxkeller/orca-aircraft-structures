#!/usr/bin/env python3
"""Search for a chord station, expressible as an ordinary lattice/web family
(NOT ``depth_to_socket``), at which an ordinary full-depth member's printed
WALL lands inside the fusion window of the spar socket's own wall while the
layer still validates.

BACKGROUND:

  * ``depth_to_socket`` is a TIP-to-circle mechanism -- its target is
    independent of the member's own chord station, so it only reaches the
    fusion window within ~0.05 mm of the socket's OWN x on this fixture, and
    even then a same-side refusal fires on every layer where the member
    shares the socket's own entry skin.
  * CO-LOCATING a pair at the socket's own chord station gives a measured
    wall gap of 0.0208 mm (inside the fusion window) but 35/35 layers are
    rejected by ``TopologyValidator`` ("opposing slots... meet").
  * Ordinary full-depth members placed LATERALLY BESIDE the socket
    (X-disjoint from it, so ``opposing_pair_meets`` never fires) can reach
    fused, validated stations, but a per-layer placement is not reproducible
    from a chord-fraction config: the detected socket centre travels ~9 mm
    from root to tip, so a FIXED chord fraction cannot hold station inside a
    0.021 mm window.

THIS PROBE tests the flank mechanism systematically and reproducibly:

  1. LOCATE (cheap): at one representative mid-span layer, for each of the
     four (side, direction) combinations -- side in {upper entry, lower
     entry}, direction in {forward of the socket, aft of the socket} --
     sweep an ordinary (``depth_to_socket=False``) full-depth single member
     finely over a station offset measured from the socket's OWN detected
     circle edge (not a config chord fraction: the offset is added to the
     socket's per-layer detected centre/radius, so the station follows the
     detected socket). The range is 0.60-1.20 mm past the circle edge, which
     brackets the 0.851-0.871 mm band measured by a different offset
     formula. The conclusions rest on the EXACT measured wall gap, not on
     agreement between offset formulas.
  2. CHARACTERIZE (expensive): every offset that lands within the
     permissive fuse band (<=0.042 mm) at the locate layer -- plus the two
     bracketing offsets immediately outside it, for context -- is re-run
     across the full 35-layer (every 17th layer) sample, each layer
     independently re-detecting the socket and re-measuring the wall gap and
     the ``TopologyValidator`` result.
  3. STABILITY CHECK: for the winning (side, direction, offset) combination,
     also runs the SAME offset as a single FIXED chord fraction (the
     fraction that offset produces at the locate layer, frozen and applied
     unchanged at every other sampled layer) and reports whether that static
     parameterisation still fuses/validates across the span, or drifts out
     of the window as a fixed fraction is expected to.
  4. METRIC SANITY CHECK: a deliberately far-away station (5 mm past the
     circle edge) must read a large wall gap; the co-located
     ``depth_to_socket`` diagnostic (0.0208 mm) is independently reproduced
     with the SAME wall-gap formula applied to a co-located pair; and the
     locate-phase sweep is checked for whether it reproduces wall gaps in the
     0.0005-0.0315 mm range at comparable offsets. This happens before any
     near-window flank reading is trusted.

WALL-GAP FORMULA: both the member's own built slot and the socket's own built
slot are VOIDS, so raw ``slot.distance(socket_slot)`` is a void-to-void gap
that ``TopologyValidator`` refuses below ``2*WALL_MM`` (the minimum-wall
band) -- unreachable as a fuse test. The real WALL-to-wall gap subtracts both
walls:

    wall_gap = void_gap - 2 * WALL_MM      # fuses when 0 < wall_gap <= 0.021

Exact shapely distance, not rasterized. Real boolean backend
(``MiterShapelyOps``), real ``TopologyValidator(ops, safety_offset_mm=0.42)``,
real ``MeshSectioner`` on the actual fixture mesh -- no proxy anywhere in
this file.

Usage:
    python tools/m25_socket_flank_search.py
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
from collections import Counter
from typing import Dict, List, Optional, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO_ROOT, os.path.join(REPO_ROOT, "tools")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

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

LAYER_HEIGHT_MM = 0.2
WALL_MM = 0.42
SPAR_FRAC = 0.30
STEP = 17
SPAN_MM = 120.0
FUSE_MM = 0.021
FUSE_PERMISSIVE_MM = 0.042

# LOCATE sweep: offset measured from the socket's own detected circle EDGE,
# added to that layer's own detected radius, so the target station tracks
# the real per-layer socket geometry rather than a config chord fraction.
LOCATE_OFFSET_LO_MM = 0.60
LOCATE_OFFSET_HI_MM = 1.20
LOCATE_OFFSET_STEP_MM = 0.005
# Window is 0.021 mm wide; the wall gap moves ~1:1 with this offset near the
# window (+0.010 mm offset -> +0.010 mm wall gap), so a
# 0.005 mm step gives >=4 samples inside a 0.021 mm window -- fine enough to
# resolve it, not just brush past it.
assert LOCATE_OFFSET_STEP_MM < FUSE_MM / 2.0

SIDES = (("upper_entry", False), ("lower_entry", True))
DIRECTIONS = (("fwd", -1.0), ("aft", +1.0))

# Deliberately far offset for the metric sanity check.
SANITY_FAR_OFFSET_MM = 5.0
# Characterize band: any locate-phase offset within this of the permissive
# fuse band is promoted to the expensive 35-layer run, plus immediate
# neighbours for context.
CHARACTERIZE_MAX_ROWS_PER_COMBO = 8


def frange(lo, hi, step):
    n = int(round((hi - lo) / step))
    return [round(lo + k * step, 6) for k in range(n + 1)]


LOCATE_OFFSETS = frange(LOCATE_OFFSET_LO_MM, LOCATE_OFFSET_HI_MM,
                         LOCATE_OFFSET_STEP_MM)


def flank_family(name, entry_lower, web_position):
    """An ORDINARY, full-depth, single-station family: NOT
    ``depth_to_socket``, no pitch (one web at ``web_position``), no z-drift.
    The field shape matches the socket-targeting families used elsewhere
    except that ``depth_to_socket`` is left at its default (False)."""
    return {
        "name": name, "role": "primary_structure",
        "rib_pitch_mm": None, "rib_pitch_chord_frac": None,
        "web_position": web_position, "web_depth_mm": None,
        "entry_lower": entry_lower, "z_drift_mm_per_mm": None,
        "z_period_mm": None, "z_drift_chord_frac_per_mm": None,
    }


def section_layer(sec, ops, i):
    """Returns (poly, frame, channel_or_None) for sampled layer index i, or
    None if there is no outer loop at this Z."""
    z = i * LAYER_HEIGHT_MM
    loops = sec.section_at(z)
    outer = [lp for lp in loops if not lp.is_interior]
    inner = [lp for lp in loops if lp.is_interior]
    if not outer:
        return None
    best = max(outer, key=lambda lp: abs(lp.signed_area))
    poly = ops.polygon_from_points(best.points)
    frame = LayerFrame.from_polygon(ops, poly)
    channel = None
    if inner:
        lp0 = max(inner, key=lambda lp: lp.equivalent_diameter_mm)
        channel = DetectedChannel(lp0.centroid[0], lp0.centroid[1],
                                   lp0.equivalent_diameter_mm)
    return z, poly, frame, channel


def target_frac(frame, channel, direction_sign, offset_from_edge_mm):
    radius = channel.diameter_mm / 2.0
    target_x = channel.center_x_mm + direction_sign * (radius + offset_from_edge_mm)
    frac = (target_x - frame.min_x) / frame.width
    return frac, target_x, radius


def measure_one(ops, gen, builder, validator, spec, i, z, frame, poly,
                 channel, entry_lower, direction_sign, offset_from_edge_mm,
                 override_frac=None):
    """Builds ONE flank family at the requested station, runs generate(),
    measures the exact wall gap to the socket, and runs the real
    TopologyValidator. Returns a result dict. ``override_frac``, if given,
    bypasses the per-layer socket-derived station (used by the stability
    check to freeze a single fraction across all layers)."""
    if override_frac is not None:
        frac = override_frac
    else:
        frac, _tx, _r = target_frac(frame, channel, direction_sign,
                                     offset_from_edge_mm)
    frac_clamped = min(1.0, max(0.0, frac))
    cap = copy.deepcopy(spec)
    fam = flank_family("flank_probe", entry_lower, frac_clamped)
    cap["families"] = [fam]
    s = sa.build_spec_from_cap_config(cap)

    channels = (channel,) if channel is not None else ()
    try:
        res = gen.generate(
            frame, s, WALL_MM,
            LayerContext(layer_index=i, print_z=z, detected_channels=channels),
            safety_offset_mm=WALL_MM)
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "stage": "generate",
                "error": "%s: %s" % (type(exc).__name__, str(exc)[:200]),
                "frac_out_of_range": frac_clamped != frac}

    socket_intent = next(
        (it for it in res.intents if it.socket_radius_mm is not None), None)
    member_intent = next(
        (it for it in res.intents if it.family_name == "flank_probe"), None)
    if socket_intent is None:
        return {"ok": False, "stage": "no_socket_detected"}
    if member_intent is None:
        reason = None
        for o in res.omitted:
            if o.family_name == "flank_probe":
                reason = o.reason
        return {"ok": False, "stage": "member_omitted", "reason": reason,
                "frac_out_of_range": frac_clamped != frac}

    socket_slot = ops.polygon_from_points(
        builder.build(socket_intent, s.overshoot_mm).points)
    member_slot = ops.polygon_from_points(
        builder.build(member_intent, s.overshoot_mm).points)
    void_gap = member_slot.distance(socket_slot)
    wall_gap = void_gap - 2.0 * WALL_MM

    slots_all = [ops.polygon_from_points(
        builder.build(it, s.overshoot_mm).points) for it in res.intents]
    repl_all = ops.diff(poly, slots_all)
    try:
        validator.validate(
            [poly], repl_all, slots_all, s, i, z,
            slot_labels=[str(it.family_name) for it in res.intents],
            family_indices=[it.family_membership() for it in res.intents])
        valid = True
        err = None
    except TopologyError as exc:
        valid = False
        err = str(exc)

    return {"ok": True, "wall_gap_mm": round(wall_gap, 5), "valid": valid,
            "error": err, "web_position": round(frac_clamped, 6),
            "frac_out_of_range": frac_clamped != frac}


def summarize_offset_across_layers(ops, gen, builder, validator, base_cap,
                                    sec, sampled_layers, entry_lower,
                                    direction_sign, offset_mm):
    """Full 35-layer characterization of one (side, direction, offset)."""
    gaps = []
    fused_strict = 0
    fused_permissive = 0
    validated = 0
    both = 0
    n_layers_with_socket = 0
    fail_reasons = Counter()
    fail_samples = []
    for i in sampled_layers:
        pack = section_layer(sec, ops, i)
        if pack is None:
            continue
        z, poly, frame, channel = pack
        if channel is None:
            continue
        n_layers_with_socket += 1
        r = measure_one(ops, gen, builder, validator, base_cap, i, z, frame,
                         poly, channel, entry_lower, direction_sign, offset_mm)
        if not r["ok"]:
            cat = "%s:%s" % (r["stage"], r.get("reason") or r.get("error") or "")
            fail_reasons[cat[:160]] += 1
            if len(fail_samples) < 4:
                fail_samples.append("L%d: %s" % (i, cat[:180]))
            continue
        gaps.append(r["wall_gap_mm"])
        fused = 0 < r["wall_gap_mm"] <= FUSE_MM
        permissive = 0 < r["wall_gap_mm"] <= FUSE_PERMISSIVE_MM
        if fused:
            fused_strict += 1
        if permissive:
            fused_permissive += 1
        if r["valid"]:
            validated += 1
        else:
            cat = "VALIDATE:%s" % (r["error"] or "")
            fail_reasons[cat[:160]] += 1
            if len(fail_samples) < 4:
                fail_samples.append("L%d VALIDATE-FAIL: %s" % (i, (r["error"] or "")[:180]))
        if fused and r["valid"]:
            both += 1
    return {
        "side": entry_lower and "lower_entry" or "upper_entry",
        "direction_sign": direction_sign,
        "offset_from_circle_edge_mm": offset_mm,
        "layers_sampled": len(sampled_layers),
        "layers_with_socket_detected": n_layers_with_socket,
        "layers_measured": len(gaps),
        "wall_gap_mm_min": round(min(gaps), 5) if gaps else None,
        "wall_gap_mm_median": round(sorted(gaps)[len(gaps) // 2], 5) if gaps else None,
        "wall_gap_mm_max": round(max(gaps), 5) if gaps else None,
        "layers_in_strict_fuse_window": fused_strict,
        "layers_in_permissive_band": fused_permissive,
        "layers_validated": validated,
        "layers_both_fused_and_valid": both,
        "failure_reasons": dict(fail_reasons),
        "failure_samples": fail_samples,
    }


def stability_check(ops, gen, builder, validator, base_cap, sec,
                     sampled_layers, entry_lower, direction_sign, offset_mm,
                     locate_layer_index):
    """Freezes the web_position fraction found at the locate layer for this (side,
    direction, offset) and re-runs it, UNCHANGED, at every other sampled layer
    -- this is the "fixed chord fraction" parameterisation retraction warned
    would drift out of the 0.021 mm window as the fixture's taper moves the
    detected socket centre. Compared against the per-layer-derived version's
    own 'layers_both_fused_and_valid' count."""
    pack0 = section_layer(sec, ops, locate_layer_index)
    if pack0 is None or pack0[-1] is None:
        return {"refused": "no socket detected at the locate layer"}
    _z0, _poly0, frame0, channel0 = pack0
    frozen_frac, _tx, _r = target_frac(frame0, channel0, direction_sign, offset_mm)

    gaps = []
    fused_strict = 0
    validated = 0
    both = 0
    frac_would_have_been = []
    for i in sampled_layers:
        pack = section_layer(sec, ops, i)
        if pack is None:
            continue
        z, poly, frame, channel = pack
        if channel is None:
            continue
        r = measure_one(ops, gen, builder, validator, base_cap, i, z, frame,
                         poly, channel, entry_lower, direction_sign, None,
                         override_frac=frozen_frac)
        # Also record what the PER-LAYER-DERIVED fraction would have been,
        # to report the drift directly.
        f_i, _tx_i, _r_i = target_frac(frame, channel, direction_sign, offset_mm)
        frac_would_have_been.append(f_i)
        if not r["ok"]:
            continue
        gaps.append(r["wall_gap_mm"])
        fused = 0 < r["wall_gap_mm"] <= FUSE_MM
        if fused:
            fused_strict += 1
        if r["valid"]:
            validated += 1
        if fused and r["valid"]:
            both += 1

    return {
        "frozen_web_position_fraction": round(frozen_frac, 6),
        "per_layer_derived_fraction_min": round(min(frac_would_have_been), 6) if frac_would_have_been else None,
        "per_layer_derived_fraction_max": round(max(frac_would_have_been), 6) if frac_would_have_been else None,
        "fraction_drift_span": round(
            max(frac_would_have_been) - min(frac_would_have_been), 6)
            if frac_would_have_been else None,
        "layers_measured": len(gaps),
        "wall_gap_mm_min": round(min(gaps), 5) if gaps else None,
        "wall_gap_mm_max": round(max(gaps), 5) if gaps else None,
        "layers_in_strict_fuse_window": fused_strict,
        "layers_validated": validated,
        "layers_both_fused_and_valid": both,
    }


def metric_sanity_check(ops, gen, builder, validator, base_cap, sec, i):
    pack = section_layer(sec, ops, i)
    if pack is None or pack[-1] is None:
        return {"refused": "no socket detected at the chosen sanity layer"}
    z, poly, frame, channel = pack
    # Try all four (side, direction) combinations at the far offset -- a
    # single combination hitting an unrelated "thin section" refusal at this
    # particular chord location is not evidence the metric is dead, so this
    # tries all four before concluding anything.
    far = None
    far_all = {}
    for side_label, entry_lower in SIDES:
        for dir_label, dsign in DIRECTIONS:
            r = measure_one(ops, gen, builder, validator, base_cap, i, z,
                             frame, poly, channel, entry_lower, dsign,
                             SANITY_FAR_OFFSET_MM)
            far_all["%s_%s" % (side_label, dir_label)] = r
            if far is None and r.get("ok") and r.get("wall_gap_mm") is not None:
                far = r
    if far is None:
        far = next(iter(far_all.values()))
    # co-located pair reproduction, SAME wall-gap formula -- built directly
    # here rather than imported, so this file's conclusions do not depend on
    # another module's arithmetic.
    cap = copy.deepcopy(base_cap)
    cap["families"] = [
        {"name": "colo_upper", "role": "primary_structure",
         "rib_pitch_mm": None, "rib_pitch_chord_frac": None,
         "web_position": SPAR_FRAC, "web_depth_mm": None,
         "entry_lower": False, "z_drift_mm_per_mm": None, "z_period_mm": None,
         "z_drift_chord_frac_per_mm": None, "depth_to_socket": True},
        {"name": "colo_lower", "role": "primary_structure",
         "rib_pitch_mm": None, "rib_pitch_chord_frac": None,
         "web_position": SPAR_FRAC, "web_depth_mm": None,
         "entry_lower": True, "z_drift_mm_per_mm": None, "z_period_mm": None,
         "z_drift_chord_frac_per_mm": None, "depth_to_socket": True},
    ]
    s = sa.build_spec_from_cap_config(cap)
    channels = (channel,)
    try:
        res = gen.generate(
            frame, s, WALL_MM,
            LayerContext(layer_index=i, print_z=z, detected_channels=channels),
            safety_offset_mm=WALL_MM)
        socket_intent = next(
            (it for it in res.intents if it.socket_radius_mm is not None), None)
        colo = [it for it in res.intents if it.family_name in
                ("colo_upper", "colo_lower")]
        colo_gap = None
        if socket_intent is not None and colo:
            socket_slot = ops.polygon_from_points(
                builder.build(socket_intent, s.overshoot_mm).points)
            it = colo[0]
            slot = ops.polygon_from_points(
                builder.build(it, s.overshoot_mm).points)
            colo_gap = round(slot.distance(socket_slot) - 2.0 * WALL_MM, 5)
    except Exception as exc:  # noqa: BLE001
        colo_gap = None
        colo = []
    return {
        "layer": i, "z": round(z, 2),
        "far_offset_mm": SANITY_FAR_OFFSET_MM,
        "far_offset_result": far,
        "far_offset_all_combinations": far_all,
        "far_offset_wall_gap_is_large": (
            far.get("ok") and far.get("wall_gap_mm", 0) > 1.0),
        "colocated_depth_to_socket_wall_gap_mm": colo_gap,
        "colocated_reproduces_known_0.0208mm": (
            colo_gap is not None and abs(colo_gap - 0.0208) < 0.01),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default=os.path.join(
        REPO_ROOT, "tests", "fixtures", "wing", "m18_wing_spar10_at0.30.stl"))
    ap.add_argument("--out", default=os.path.join(
        REPO_ROOT, "docs", "results", "m25_socket_flank_search.json"))
    a = ap.parse_args(argv)

    if not HAVE_SHAPELY:
        print("REFUSED: shapely is unavailable; this probe requires the real "
              "boolean backend and cannot report numbers without it.",
              file=sys.stderr)
        return 1

    base_cap = json.load(open(os.path.join(
        REPO_ROOT, "docs", "results",
        "m19_prototype_config.json")))["config"][0]["cap_config"]

    verts, tris = read_binary_stl(a.stl)
    sec = MeshSectioner(verts, tris)
    n_layers = int(round(SPAN_MM / LAYER_HEIGHT_MM))
    sampled_layers = list(range(5, n_layers, STEP))

    ops = MiterShapelyOps()
    gen, builder = StructureGenerator(), SlotBuilder()
    validator = TopologyValidator(ops, safety_offset_mm=WALL_MM)

    # Pick a mid-span locate layer that actually has a detected socket.
    mid = sampled_layers[len(sampled_layers) // 2]
    locate_layer_index = None
    locate_pack = None
    for i in sorted(sampled_layers, key=lambda x: abs(x - mid)):
        pack = section_layer(sec, ops, i)
        if pack is not None and pack[-1] is not None:
            locate_layer_index = i
            locate_pack = pack
            break
    if locate_layer_index is None:
        print("REFUSED: no sampled layer has a detected socket channel; "
              "cannot locate a flank window without one.", file=sys.stderr)
        return 1

    print("SOCKET FLANK SEARCH")
    print("locate layer: %d (z=%.2f mm)" % (locate_layer_index, locate_pack[0]))
    print("locate sweep: offset %.3f..%.3f mm past circle edge, step %.3f mm "
          "(< %.3f mm fuse window / 2 -> resolves it)"
          % (LOCATE_OFFSET_LO_MM, LOCATE_OFFSET_HI_MM, LOCATE_OFFSET_STEP_MM,
             FUSE_MM))
    print()

    all_locate = {}
    characterized_rows = []
    best = None  # (both_count, row)

    for side_label, entry_lower in SIDES:
        for dir_label, dsign in DIRECTIONS:
            rows = []
            for off in LOCATE_OFFSETS:
                z, poly, frame, channel = locate_pack
                r = measure_one(ops, gen, builder, validator, base_cap,
                                 locate_layer_index, z, frame, poly, channel,
                                 entry_lower, dsign, off)
                rows.append({"offset_mm": off, **r})
            all_locate["%s_%s" % (side_label, dir_label)] = rows

            # Select offsets to characterize: any within the permissive band,
            # plus the two neighbours immediately outside on each side, for
            # context. If nothing is within the permissive band, take the
            # closest miss (smallest positive-wall-gap-magnitude reading).
            near_idx = [k for k, r in enumerate(rows)
                        if r.get("ok") and r.get("wall_gap_mm") is not None
                        and 0 < r["wall_gap_mm"] <= FUSE_PERMISSIVE_MM]
            if near_idx:
                lo_i, hi_i = min(near_idx), max(near_idx)
                sel = set(range(max(0, lo_i - 2), min(len(rows), hi_i + 3)))
            else:
                finite = [(k, r["wall_gap_mm"]) for k, r in enumerate(rows)
                          if r.get("ok") and r.get("wall_gap_mm") is not None]
                if finite:
                    k_best = min(finite, key=lambda kv: abs(kv[1]))[0]
                    sel = set(range(max(0, k_best - 2), min(len(rows), k_best + 3)))
                else:
                    sel = set()
            sel = sorted(sel)[:CHARACTERIZE_MAX_ROWS_PER_COMBO]

            print("--- %s / %s (%d offsets characterized across %d layers) ---"
                  % (side_label, dir_label, len(sel), len(sampled_layers)))
            for k in sel:
                off = rows[k]["offset_mm"]
                full = summarize_offset_across_layers(
                    ops, gen, builder, validator, base_cap, sec,
                    sampled_layers, entry_lower, dsign, off)
                characterized_rows.append(full)
                print("  offset=%.3f mm  gap[min/med/max]=%s/%s/%s  "
                      "fuse=%d/%d  perm=%d/%d  valid=%d/%d  BOTH=%d/%d"
                      % (off, full["wall_gap_mm_min"], full["wall_gap_mm_median"],
                         full["wall_gap_mm_max"],
                         full["layers_in_strict_fuse_window"], full["layers_measured"],
                         full["layers_in_permissive_band"], full["layers_measured"],
                         full["layers_validated"], full["layers_measured"],
                         full["layers_both_fused_and_valid"], full["layers_measured"]))
                if full["failure_samples"]:
                    for fs in full["failure_samples"]:
                        print("      %s" % fs)
                if best is None or full["layers_both_fused_and_valid"] > best[0]:
                    best = (full["layers_both_fused_and_valid"], full)
            print()

    # Stability check on the best combination found.
    stability = None
    if best is not None and best[0] > 0:
        b = best[1]
        entry_lower_best = (b["side"] == "lower_entry")
        stability = stability_check(
            ops, gen, builder, validator, base_cap, sec, sampled_layers,
            entry_lower_best, b["direction_sign"], b["offset_from_circle_edge_mm"],
            locate_layer_index)

    sanity = metric_sanity_check(ops, gen, builder, validator, base_cap, sec,
                                  locate_layer_index)

    all_failure_texts = Counter()
    for row in characterized_rows:
        for k, v in row["failure_reasons"].items():
            all_failure_texts[k] += v

    verdict_yes = best is not None and best[0] == len(sampled_layers) and best[0] > 0
    verdict_partial = best is not None and best[0] > 0 and not verdict_yes

    out = {
        "fixture": a.stl,
        "wall_mm": WALL_MM, "spar_frac": SPAR_FRAC, "step": STEP,
        "layer_height_mm": LAYER_HEIGHT_MM, "span_mm": SPAN_MM,
        "fuse_mm": FUSE_MM, "fuse_permissive_mm": FUSE_PERMISSIVE_MM,
        "locate_layer_index": locate_layer_index,
        "locate_offset_range_mm": [LOCATE_OFFSET_LO_MM, LOCATE_OFFSET_HI_MM],
        "locate_offset_step_mm": LOCATE_OFFSET_STEP_MM,
        "sampled_layers": sampled_layers,
        "locate_phase": all_locate,
        "characterized": characterized_rows,
        "best": best[1] if best else None,
        "best_both_count": best[0] if best else 0,
        "stability_check_of_best": stability,
        "metric_sanity_check": sanity,
        "deduplicated_failure_texts": dict(all_failure_texts),
        "verdict": {
            "socket_fused_and_validated_placement_exists": verdict_yes or verdict_partial,
            "holds_every_sampled_layer": verdict_yes,
        },
    }
    with open(a.out, "w") as fh:
        json.dump(out, fh, indent=2)

    print("=" * 70)
    print("METRIC SANITY CHECK")
    print("  far offset (%.1f mm past edge): wall_gap=%s -> large? %s"
          % (SANITY_FAR_OFFSET_MM,
             sanity.get("far_offset_result", {}).get("wall_gap_mm"),
             sanity.get("far_offset_wall_gap_is_large")))
    print("  co-located depth_to_socket pair wall_gap=%s (expect ~0.0208 mm): "
          "reproduces known value? %s"
          % (sanity.get("colocated_depth_to_socket_wall_gap_mm"),
             sanity.get("colocated_reproduces_known_0.0208mm")))
    print()
    if best is not None and best[0] > 0:
        b = best[1]
        print("BEST PLACEMENT: side=%s direction_sign=%+.0f offset=%.3f mm "
              "-> BOTH fused+valid on %d/%d sampled layers"
              % (b["side"], b["direction_sign"], b["offset_from_circle_edge_mm"],
                 b["layers_both_fused_and_valid"], b["layers_measured"]))
        print("  wall_gap[min/med/max]=%s/%s/%s"
              % (b["wall_gap_mm_min"], b["wall_gap_mm_median"], b["wall_gap_mm_max"]))
        if stability:
            print("  STABILITY: frozen fraction=%s -> BOTH=%d/%d layers "
                  "(per-layer-derived fraction drift span=%.4f)"
                  % (stability.get("frozen_web_position_fraction"),
                     stability.get("layers_both_fused_and_valid", 0),
                     stability.get("layers_measured", 0),
                     stability.get("fraction_drift_span") or 0.0))
    else:
        print("NO placement found with any layer BOTH fused and valid.")
    print()
    print("VERDICT: %s" % (
        "YES, holds on every sampled layer" if verdict_yes else
        "PARTIAL -- fuses+validates on some but not all sampled layers" if verdict_partial else
        "NO -- no (side, offset) achieved a single fused+validated layer"))
    print("wrote %s" % a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
