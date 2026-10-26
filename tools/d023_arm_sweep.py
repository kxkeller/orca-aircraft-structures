#!/usr/bin/env python3
"""Measurement: chord-relative drift vs absolute drift over the wing fixture,
using the real generator and validator.

MEASUREMENT / TOOLING ONLY. Does not modify ``aircraft_core/``, ``plugin/``,
``host/data/orca_plugins/config.json``, or any tracked preset. Reuses
``tools/structural_analysis.build_spec_from_cap_config`` and the per-layer
generator/validator loop rather than reimplementing either.

Two arms, both built in-memory from a copy of the LIVE config's cap_config
(``host/data/orca_plugins/config.json``), never by editing that file:

  ARM A (chord-relative): every family's drift field exactly as the live config has
    it -- chord-relative (``z_drift_chord_frac_per_mm``) at +/-0.016667 for
    the eight lattice/deep families, ``z_period_mm`` 64.6.
  ARM B (absolute): every family that has ``z_drift_chord_frac_per_mm``
    set in ARM A has it replaced with ``z_drift_mm_per_mm`` at the SAME SIGN,
    magnitude 1.0, and ``z_drift_chord_frac_per_mm`` nulled,
    with ``z_period_mm`` and everything else UNCHANGED.

Runs the FULL layer sweep of the given STL (root_solid_layers through the
mesh's own z_max), never a sample. Reports, per arm: node_merge_count (count of
representative WebIntents with non-empty ``merged_family_indices``) per layer
(median/worst) and in total, built/ omitted counts per family, TopologyError
failures with layer/Z/cause, the resolved local chord at the first and last
successfully-processed body layer, and the width_mm distribution of merged
(union-interval) intents as the diagnostics' own proxy for "worst-case web
thickness at a merged node" (the node's own wall is not separately tracked as a
thickness field; the union-interval width is used as the stand-in).

Usage:
    python tools/d023_arm_sweep.py
    python tools/d023_arm_sweep.py --stl tests/fixtures/wing/naca2412_wing_r150_t90_s400_w3.stl --wall 0.42
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import statistics
import sys
import time
from collections import Counter, defaultdict

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
LIVE_CONFIG_PATH = os.path.join(
    REPO_ROOT, "host", "data", "orca_plugins", "config.json")
DEFAULT_STL = os.path.join(
    REPO_ROOT, "tests", "fixtures", "wing", "naca2412_wing_r150_t90_s400_w3.stl")


def load_live_cap_config():
    with open(LIVE_CONFIG_PATH, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    entry = doc["config"][0] if isinstance(doc["config"], list) else doc["config"]
    return entry["cap_config"]


def make_arm_b_cap_config(cap_config):
    """Part A recipe, applied to an in-memory COPY only: every family with
    ``z_drift_chord_frac_per_mm`` set gets ``z_drift_mm_per_mm`` at the same
    sign, magnitude 1.0, and the chord-frac field nulled. Every other field, on
    every family, is untouched.
    """
    out = copy.deepcopy(cap_config)
    changed = []
    for fam in out.get("families", []):
        frac = fam.get("z_drift_chord_frac_per_mm")
        if frac is None:
            continue
        sign = 1.0 if frac > 0 else -1.0
        fam["z_drift_mm_per_mm"] = sign * 1.0
        fam["z_drift_chord_frac_per_mm"] = None
        changed.append(fam.get("name"))
    return out, changed


def family_tier(fam):
    """Classify a family into 'skin' / 'deep' / 'flank' / 'other' from ITS
    OWN fields at runtime -- never hardcoded index assumptions, since
    ``effective_families()`` order is whatever the config lists.
    """
    if getattr(fam, "socket_flank_offset_mm", None) is not None:
        return "flank"
    if fam.role == "skin_lattice":
        return "skin"
    if fam.role == "primary_structure" and fam.rib_pitch_chord_frac is not None:
        return "deep"
    return "other"


def sweep(spec, wall_mm, stl_path, arm_label, progress_every=200):
    ops = MiterShapelyOps()
    gen, builder = StructureGenerator(), SlotBuilder()
    validator = TopologyValidator(ops, safety_offset_mm=wall_mm)

    verts, tris = read_binary_stl(stl_path)
    sec = MeshSectioner(verts, tris)
    z_min, z_max = sec.z_min, sec.z_max
    # This loop iterates layer index from 0 (``range(root, n_layers)`` with
    # ``z = i * LAYER_HEIGHT_MM``) and skips via ``z <= z_min or z >= z_max``
    # -- the byte-identical z_min defect ``tools/m34_rotation_equivalence.py``
    # found and fixed with ``_drop_to_bed`` (see its docstring): unless the
    # mesh's own bottom is already at Z 0, this silently drops every layer
    # below the mesh's raw z_min instead of dropping the mesh onto the bed
    # first, the way the real host does before slicing. This tool is
    # currently dormant to that bug ONLY because its default fixture
    # (``naca2412_wing_r150_...stl``) happens to sit at z_min == 0.0 exactly.
    # Loud, not silent: fail here rather than let a future caller point this
    # at another mesh and get a quietly-wrong layer count. If this ever
    # fires, apply ``_drop_to_bed`` from ``tools/m34_rotation_equivalence.py``
    # here rather than reworking this loop from scratch.
    assert abs(z_min) < 1e-6, (
        "tools/d023_arm_sweep.py: mesh z_min=%r is not ~0 -- this tool's "
        "layer loop iterates from 0 without dropping the mesh to the bed "
        "first, the same z_min defect tools/m34_rotation_equivalence.py's "
        "_drop_to_bed fixes. Apply that fix here before trusting this "
        "sweep's layer count on this mesh." % (z_min,))

    root = int(spec.root_solid_layers or 0)
    # int(round(...)), not a floor-division truncation (own convention) --
    # z_max/layer_height need not be an exact integer, and a floor here
    # silently drops the fixture's own tip layer.
    n_layers = int(round(z_max / LAYER_HEIGHT_MM))

    families = spec.effective_families()
    tiers = [family_tier(f) for f in families]
    names = [f.name for f in families]

    built_by_family = Counter()
    omitted_by_family = Counter()
    omitted_reasons = Counter()
    merges_per_layer = []
    total_merge_events = 0
    merged_widths_mm = []
    failures = []
    ok = 0
    root_chord_mm = None
    tip_chord_mm = None
    root_layer_seen = None
    tip_layer_seen = None
    body_layers_processed = 0

    # Per-station recurrence, so a persistent (every-layer) cross-tier
    # approach can be told apart from a banded (some-layers-only) one.
    # Keyed by a description string; value is the COUNT OF DISTINCT LAYERS
    # the key was observed in (deduplicated per layer before updating).
    station_layer_counts = Counter()

    t0 = time.perf_counter()
    for i in range(root, n_layers):
        z = i * LAYER_HEIGHT_MM
        if z <= z_min or z >= z_max:
            continue
        try:
            loops = sec.section_at(z)
            outer = [lp for lp in loops if not lp.is_interior]
            inner = [lp for lp in loops if lp.is_interior]
            if not outer:
                continue
            poly = ops.polygon_from_points(
                max(outer, key=lambda lp: abs(lp.signed_area)).points)
            frame = LayerFrame.from_polygon(ops, poly)

            if root_chord_mm is None:
                root_chord_mm = frame.width
                root_layer_seen = i
            tip_chord_mm = frame.width
            tip_layer_seen = i

            channels = tuple(
                DetectedChannel(lp.centroid[0], lp.centroid[1],
                                 lp.equivalent_diameter_mm) for lp in inner)
            res = gen.generate(
                frame, spec, wall_mm,
                LayerContext(layer_index=i, print_z=z,
                             detected_channels=channels),
                safety_offset_mm=wall_mm)

            body_layers_processed += 1
            this_layer_keys = set()

            layer_merges = 0
            for it in res.intents:
                built_by_family[str(it.family_name)] += 1
                if it.merged_family_indices:
                    layer_merges += 1
                    total_merge_events += 1
                    merged_widths_mm.append(it.width_mm)
                    member_tiers = sorted(set(
                        tiers[idx] for idx in it.merged_family_indices))
                    member_names = tuple(sorted(
                        names[idx] for idx in it.merged_family_indices))
                    kind = "merge/cross_tier" if len(member_tiers) > 1 else "merge/same_tier"
                    frac = ((it.start[0] - frame.min_x) / frame.width
                             if frame.width else 0.0)
                    key = "{} families={} entry_lower={} x_frac~{:.2f}".format(
                        kind, member_names, it.entry_lower, round(frac, 2))
                    this_layer_keys.add(key)
            merges_per_layer.append(layer_merges)

            # True overlaps between unmerged intents with disjoint family
            # membership. The validator accepts gap<0 with disjoint membership
            # directly and never routes it through merge, so these intents
            # carry no merged_family_indices and would otherwise be invisible
            # to this sweep.
            for a_idx in range(len(res.intents)):
                for b_idx in range(a_idx + 1, len(res.intents)):
                    ia, ib = res.intents[a_idx], res.intents[b_idx]
                    if ia.entry_lower != ib.entry_lower:
                        continue
                    mem_a, mem_b = ia.family_membership(), ib.family_membership()
                    if mem_a & mem_b:
                        continue  # same-family/component -- not this check
                    lo_a, hi_a = ia.start[0] - ia.width_mm / 2.0, ia.start[0] + ia.width_mm / 2.0
                    lo_b, hi_b = ib.start[0] - ib.width_mm / 2.0, ib.start[0] + ib.width_mm / 2.0
                    overlap = min(hi_a, hi_b) - max(lo_a, lo_b)
                    if overlap <= 0.0:
                        continue  # touching/apart, not an actual overlap
                    tier_a = tiers[min(mem_a)] if mem_a else "?"
                    tier_b = tiers[min(mem_b)] if mem_b else "?"
                    kind = "d009_overlap/cross_tier" if tier_a != tier_b else "d009_overlap/same_tier"
                    frac = ((ia.start[0] - frame.min_x) / frame.width
                             if frame.width else 0.0)
                    key = "{} families=({},{}) entry_lower={} x_frac~{:.2f}".format(
                        kind, ia.family_name, ib.family_name, ia.entry_lower,
                        round(frac, 2))
                    this_layer_keys.add(key)

            for o in res.omitted:
                nm = str(getattr(o, "family_name", "?"))
                omitted_by_family[nm] += 1
                reason = (o.reason or "").split(":")[0].strip().lower()
                omitted_reasons["{} / {}".format(nm, reason)] += 1
                x_mm = getattr(o, "x_mm", None)
                if x_mm is not None and frame.width and x_mm == x_mm:  # not nan
                    frac = (x_mm - frame.min_x) / frame.width
                    key = "omit family={} reason={} x_frac~{:.2f}".format(
                        nm, reason, round(frac, 2))
                    this_layer_keys.add(key)

            for key in this_layer_keys:
                station_layer_counts[key] += 1

            slots = [ops.polygon_from_points(
                builder.build(it, spec.overshoot_mm).points)
                for it in res.intents]
            repl = ops.diff(poly, slots)
            validator.validate(
                [poly], repl, slots, spec, i, z,
                slot_labels=[str(it.family_name) for it in res.intents],
                family_indices=[it.family_membership() for it in res.intents])
            ok += 1
        except TopologyError as exc:
            failures.append({
                "layer": i, "z": round(z, 3), "kind": "TopologyError",
                "msg": str(exc),
            })
        except Exception as exc:  # noqa: BLE001
            failures.append({
                "layer": i, "z": round(z, 3),
                "kind": type(exc).__name__,
                "msg": "{}: {}".format(type(exc).__name__, str(exc)[:400]),
            })

        if progress_every and i % progress_every == 0:
            print("  [{}] layer {}/{} z={:.1f}".format(arm_label, i, n_layers, z),
                  file=sys.stderr, flush=True)

    elapsed = time.perf_counter() - t0
    body_layers_considered = body_layers_processed

    result = {
        "arm": arm_label,
        "wall_mm": wall_mm,
        "z_min": z_min,
        "z_max": z_max,
        "root_solid_layers": root,
        "layer_height_mm": LAYER_HEIGHT_MM,
        "body_layers_considered": body_layers_considered,
        "layers_validated_ok": ok,
        "failed_count": len(failures),
        "first_failures": failures[:10],
        "all_failures": failures,
        "built_by_family": dict(built_by_family),
        "omitted_by_family": dict(omitted_by_family),
        "omitted_reasons": dict(omitted_reasons),
        "node_merge_count_total": total_merge_events,
        "node_merge_count_median_per_layer": (
            statistics.median(merges_per_layer) if merges_per_layer else None),
        "node_merge_count_worst_layer": (
            max(merges_per_layer) if merges_per_layer else None),
        "merged_intent_width_mm_min": (
            min(merged_widths_mm) if merged_widths_mm else None),
        "merged_intent_width_mm_max": (
            max(merged_widths_mm) if merged_widths_mm else None),
        "merged_intent_width_mm_median": (
            statistics.median(merged_widths_mm) if merged_widths_mm else None),
        "root_chord_mm": root_chord_mm,
        "root_layer_index": root_layer_seen,
        "tip_chord_mm": tip_chord_mm,
        "tip_layer_index": tip_layer_seen,
        "suppress_reason_count": sum(
            1 for k in omitted_reasons if "suppress" in k.lower()),
        "failures_by_kind": dict(Counter(f["kind"] for f in failures)),
        # Every recurring station, with the FRACTION of body layers it
        # was observed in -- persistence is what tells a static (every
        # layer) cross-tier approach apart from a banded (some layers)
        # ordinary one. Sorted by persistence, most persistent first.
        "family_names_by_tier": {
            "skin": [n for n, t in zip(names, tiers) if t == "skin"],
            "deep": [n for n, t in zip(names, tiers) if t == "deep"],
            "flank": [n for n, t in zip(names, tiers) if t == "flank"],
        },
        "station_recurrence": sorted(
            (
                {
                    "key": key,
                    "layers": count,
                    "fraction_of_body_layers": (
                        round(count / body_layers_considered, 4)
                        if body_layers_considered else None),
                }
                for key, count in station_layer_counts.items()
            ),
            key=lambda r: r["layers"], reverse=True,
        ),
        "elapsed_s": round(elapsed, 1),
    }
    return result


def resolved_drift_rate(fam_kwargs, chord_mm):
    """The effective |z_drift_mm_per_mm| this family resolves to at
    ``chord_mm``, mirroring ``WebFamily.station_drift_mm``'s own resolution
    (absolute field wins if set; else chord-frac * chord).
    """
    if fam_kwargs.get("z_drift_mm_per_mm") is not None:
        return abs(fam_kwargs["z_drift_mm_per_mm"])
    frac = fam_kwargs.get("z_drift_chord_frac_per_mm")
    if frac is None:
        return 0.0
    return abs(frac) * chord_mm


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--wall", type=float, default=0.42)
    ap.add_argument("--stl", default=DEFAULT_STL)
    ap.add_argument("--out-prefix", default=os.path.join(
        REPO_ROOT, "docs", "results", "d023_arm_sweep"))
    a = ap.parse_args(argv)
    if not HAVE_SHAPELY:
        raise SystemExit("shapely unavailable; refusing to report")

    cap_a = load_live_cap_config()
    cap_b, changed_families = make_arm_b_cap_config(cap_a)

    spec_a = sa.build_spec_from_cap_config(cap_a)
    spec_b = sa.build_spec_from_cap_config(cap_b)

    print("ARM B changed families:", changed_families)

    result_a = sweep(spec_a, a.wall, a.stl, "A (today: chord-relative drift)")
    result_b = sweep(spec_b, a.wall, a.stl, "B (absolute drift)")

    # Plain-arithmetic chordwise-step table, at ROOT and TIP chord as
    # measured by each arm's own sweep (should agree between arms since the
    # mesh section itself is drift-independent).
    root_chord = result_a["root_chord_mm"] or result_b["root_chord_mm"]
    tip_chord = result_a["tip_chord_mm"] or result_b["tip_chord_mm"]
    step_table = []
    for label, cap in (("A", cap_a), ("B", cap_b)):
        for fam in cap.get("families", []):
            name = fam.get("name")
            if (fam.get("z_drift_mm_per_mm") is None
                    and fam.get("z_drift_chord_frac_per_mm") is None):
                continue
            for chord_label, chord_mm in (("root", root_chord), ("tip", tip_chord)):
                if chord_mm is None:
                    continue
                rate = resolved_drift_rate(fam, chord_mm)
                step_mm = rate * LAYER_HEIGHT_MM
                step_table.append({
                    "arm": label, "family": name, "chord_point": chord_label,
                    "chord_mm": round(chord_mm, 3),
                    "resolved_drift_mm_per_mm": round(rate, 6),
                    "chordwise_step_mm": round(step_mm, 6),
                    "ratio_to_0.42mm_line": round(step_mm / 0.42, 4),
                })

    out = {
        "stl": a.stl,
        "wall_mm": a.wall,
        "live_config_path": LIVE_CONFIG_PATH,
        "arm_b_changed_families": changed_families,
        "arm_a": result_a,
        "arm_b": result_b,
        "chordwise_step_table": step_table,
    }
    out_path = a.out_prefix + ".json"
    with open(out_path, "w") as fh:
        json.dump(out, fh, indent=2)

    for r in (result_a, result_b):
        print("\n=== %s ===" % r["arm"])
        print("  layers considered: %d, validated OK: %d, failed: %d"
              % (r["body_layers_considered"], r["layers_validated_ok"], r["failed_count"]))
        print("  root chord: %.3f mm (layer %s), tip chord: %.3f mm (layer %s)"
              % (r["root_chord_mm"] or -1, r["root_layer_index"],
                 r["tip_chord_mm"] or -1, r["tip_layer_index"]))
        print("  node_merge_count total=%d median/layer=%s worst/layer=%s"
              % (r["node_merge_count_total"], r["node_merge_count_median_per_layer"],
                 r["node_merge_count_worst_layer"]))
        print("  merged intent width_mm: min=%s median=%s max=%s"
              % (r["merged_intent_width_mm_min"], r["merged_intent_width_mm_median"],
                 r["merged_intent_width_mm_max"]))
        print("  built by family:", r["built_by_family"])
        print("  omitted by family:", r["omitted_by_family"])
        print("  suppress-reason omissions:", r["suppress_reason_count"])
        print("  failures by kind:", r["failures_by_kind"])
        if r["failed_count"]:
            print("  FIRST FAILURES:")
            for f in r["first_failures"]:
                print("    L%s z=%s [%s] %s" % (f["layer"], f["z"], f["kind"], f["msg"][:300]))
        print("  TOP 15 recurring stations (R7):")
        for row in r["station_recurrence"][:15]:
            print("    %5d layers (%.1f%%)  %s" % (
                row["layers"], 100.0 * (row["fraction_of_body_layers"] or 0),
                row["key"]))

    print("\nwrote %s" % out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
