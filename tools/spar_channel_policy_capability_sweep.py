#!/usr/bin/env python3
"""Re-measure m34_rotation_equivalence.run_arm's per-layer sweep with the
SHIPPED capability.py exception taxonomy instead of a bare Exception ->
'failed' bucket.

WHY THIS EXISTS: ``_sweep_layers`` in tools/m34_rotation_equivalence.py
calls ``StructureGenerator.generate`` directly and buckets ANY exception other
than ``TopologyError`` -- including ``ReEntrantSectionError``, a ``ValueError``
subclass -- as a harness "failed" layer. The REAL shipped path
(``plugin/aircraft_structures/capability.py``'s ``_transform``) catches
``ReEntrantSectionError`` specifically and DEGRADES that layer instead (left
unmutated, recorded, slice continues). Measuring ``spar_channel_policy``
through the bare-harness taxonomy would overstate "failed" by exactly the
re-entrant layers, which have nothing to do with the policy under test.

This is a STANDALONE reproduction harness, not a patch to
``m34_rotation_equivalence.py``: it monkeypatches that module's
``_sweep_layers`` with a copy carrying the corrected except clauses for the
duration of one run, then calls its real, unmodified ``run_arm`` (rotation
detection, canonical frame, ``ship_rotate_deg`` composition, the real
generated bundle, the real ``MiterShapelyOps`` boolean backend, the real
``TopologyValidator.validate`` -- all untouched).

Usage:
    python tools/spar_channel_policy_capability_sweep.py <label> <stl_path> \
        <out_path.json> [--kai] [--policy refuse|nearest_spar_station]

--kai   apply the stand-up transform (-z, y, x) before sectioning, same
        convention as m34_rotation_equivalence.run_kai.
"""
import argparse
import dataclasses
import json
import os
import sys
from collections import Counter
from pathlib import Path

REPO_ROOT = str(Path(__file__).resolve().parent.parent)
for _p in (REPO_ROOT, os.path.join(REPO_ROOT, "tools"),
           os.path.join(REPO_ROOT, "tests", "integration")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.frame import ReEntrantSectionError  # noqa: E402
from aircraft_core.generator import (  # noqa: E402
    DetectedChannel, LayerContext, StructureGenerator,
)
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402
from m18_mesh_probe import read_binary_stl  # noqa: E402
from shapely_ops import MiterShapelyOps  # noqa: E402
import fake_orca  # noqa: E402
import m34_rotation_equivalence as m34  # noqa: E402
from m34_rotation_equivalence import (  # noqa: E402
    _station_key, rotate_points_deg, ShapelyPolygon, LAYER_HEIGHT_MM,
)


def _sweep_layers_capability_taxonomy(sec, spec, wall_mm, cap, ops, ship_rotate_deg):
    """Byte-for-byte copy of ``m34_rotation_equivalence._sweep_layers``
    except for the ``ReEntrantSectionError`` branch added around the
    ``gen.generate(...)`` call -- see this module's own docstring."""
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
    degraded = []  # layers capability.py degrades rather than fails
    ok = 0
    skipped_out_of_z_range = []
    skipped_no_outer_loop = []
    per_layer_shipped = {}
    per_layer_candidates = {}

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
            try:
                res = gen.generate(
                    frame, spec, wall_mm,
                    LayerContext(layer_index=i, print_z=z, detected_channels=channels),
                    safety_offset_mm=wall_mm)
            except ReEntrantSectionError as exc:
                # Mirrors plugin/aircraft_structures/capability.py's own
                # `except ReEntrantSectionError` -- the layer is left
                # UNMUTATED and recorded, the slice continues. Counted as
                # `ok` (the slice does not fail), tagged separately as
                # degraded so it is never conflated with a genuine
                # structural success either.
                degraded.append({
                    "layer": i, "z": round(z, 4), "kind": "re_entrant_section",
                    "msg": str(exc)[:300],
                })
                ok += 1
                continue

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
                              "msg": str(exc)[:500]})
        except Exception as exc:  # noqa: BLE001
            failures.append({"layer": i, "z": round(z, 4),
                              "kind": type(exc).__name__, "msg": str(exc)[:500]})

    n_skipped = len(skipped_out_of_z_range) + len(skipped_no_outer_loop)
    n_accounted = root + ok + len(failures) + n_skipped
    if n_accounted != n_layers:
        raise AssertionError(
            "layer reconciliation failed: root(%d) + ok(%d) + failed(%d) + "
            "skipped(%d) = %d != n_layers(%d)" % (
                root, ok, len(failures), n_skipped, n_accounted, n_layers))

    return {
        "root": root, "n_layers": n_layers, "ok": ok, "failed": len(failures),
        "degraded": len(degraded), "degraded_sample": degraded,
        "failures_sample": failures[:10],
        "skipped_out_of_z_range": len(skipped_out_of_z_range),
        "skipped_no_outer_loop": len(skipped_no_outer_loop),
        "reconciled": n_accounted == n_layers,
        "built_by_family": dict(built_by_family),
        "omitted_by_family": dict(omitted_by_family),
        "node_merge_count": node_merge_total,
        "order_swap_count": order_swap_total,
    }


def run(label, stl_path, policy, kai_transform):
    m34._sweep_layers = _sweep_layers_capability_taxonomy

    verts, tris = read_binary_stl(stl_path)
    if kai_transform:
        verts = [(-v[2], v[1], v[0]) for v in verts]
    spec = m34.load_spec()
    spec = dataclasses.replace(spec, spar_channel_policy=policy)
    spec.validate(safety_offset_mm=m34.WALL_MM)

    fo, restore = fake_orca.install()
    try:
        namespace = fake_orca.exec_bundle(
            os.path.join(REPO_ROOT, "plugin", "aircraft_structures", "aircraft_structures.py"))
        cap = namespace["AircraftStructuresM1"]()
        ops = MiterShapelyOps()
        result = m34.run_arm(0.0, verts, tris, spec, m34.WALL_MM, cap, ops)
    finally:
        restore()

    n_skipped = result["skipped_out_of_z_range"] + result["skipped_no_outer_loop"]
    return {
        "label": label, "stl": stl_path, "policy": policy,
        "root": result["root"], "n_layers": result["n_layers"],
        "ok": result["ok"], "failed": result["failed"],
        "degraded": result["degraded"], "degraded_sample": result["degraded_sample"],
        "skipped_total": n_skipped, "reconciled": result["reconciled"],
        "failures_sample": result["failures_sample"],
        "built_by_family": result["built_by_family"],
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("label")
    p.add_argument("stl_path")
    p.add_argument("out_path")
    p.add_argument("--kai", action="store_true")
    p.add_argument("--policy", default="nearest_spar_station")
    args = p.parse_args()
    out = run(args.label, args.stl_path, args.policy, args.kai)
    with open(args.out_path, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=2)
    print(json.dumps(
        {k: v for k, v in out.items() if k not in ("failures_sample", "built_by_family")},
        indent=2,
    ))


if __name__ == "__main__":
    main()
