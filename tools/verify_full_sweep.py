#!/usr/bin/env python3
"""Full-layer, real-boolean verification sweep.

WHY THIS TOOL EXISTS: two cheaper checks cannot support a "verified" claim:

  1. ``tools/structural_analysis.py``'s smoke check (``check_wall_violations``
     / ``assert_cap_config_slices_cleanly``) never runs ``TopologyValidator``
     at all -- it mirrors ONE of its rules at the intent level and
     reports "N/N layers OK" on generator output alone. It cannot see an
     erosion-connectivity failure, because it never erodes anything.
  2. A dry run that SAMPLES layers on a STUB geometry backend cannot model
     real polygon-boolean erosion. It can report "0 failures" while a real
     slice fails on a skipped layer, with a failure mode (a leading-edge
     pinch under erosion) the stub backend cannot represent at all.

This tool covers both gaps in ONE run: it drives the REAL
``StructureGenerator`` -> ``SlotBuilder`` -> ``GeometryOps.diff`` ->
``TopologyValidator`` pipeline, over EVERY layer (no sampling), using a real
polygon-boolean backend (``tools/shapely_ops.MiterShapelyOps`` -- shapely,
mitre-join; empirical Clipper match, not a verified pin). It mirrors
``plugin/aircraft_structures/capability.py``'s own layer gating
(``root_solid_layers`` skip, tip-shutdown-on-all-thin-omission handling,
same ``slot_labels``/``family_indices`` construction) so "this sweep would
have caught it" is a real claim about the SAME code path the host runs, not
an approximation of it.

REFUSE TO CLAIM SUCCESS WHEN IT CANNOT ACTUALLY CHECK: if ``shapely`` is
unavailable, or any per-layer exception escapes the pipeline in a way that
is not itself a ``TopologyError`` (a backend that cannot represent some
geometry, for instance), this reports NOT-VERIFIED and says why -- it never
prints a layer count that reads as a pass when the check did not actually
run to completion. See ``SweepResult`` for the three distinct outcomes.

``shapely`` and ``tools/shapely_ops.py`` are TEST/TOOL-ONLY. This tool
imports neither into anything that ships; it is itself never imported by
``aircraft_core`` or the plugin bundle.

Usage:
    python tools/verify_full_sweep.py [--config PATH] [--fixture {coupon,airfoil,both}]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
TOOLS_DIR = os.path.join(REPO_ROOT, "tools")
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)
TESTS_UNIT_DIR = os.path.join(REPO_ROOT, "tests", "unit")
if TESTS_UNIT_DIR not in sys.path:
    sys.path.insert(0, TESTS_UNIT_DIR)

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import LayerContext, StructureGenerator  # noqa: E402
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402

DEFAULT_CONFIG_PATH = os.path.join(
    REPO_ROOT, "presets", "plugin", "skin_lattice_only_config.json"
)
RESULTS_DIR = os.path.join(REPO_ROOT, "docs", "results")

# Real host convention (capability.py's DEFAULT_SPEC / process presets): 0.20
# mm layer height, matching every in-host report and preset tracked in this
# repo. Not itself a cap_config field -- see structural_analysis.py's own
# mass-proxy note for the same assumption.
LAYER_HEIGHT_MM = 0.2

# The tracked coupon fixture (coupon_20x20x15.stl): 20x20 mm cross-section,
# 15 mm tall -> 75 layers at 0.20 mm.
COUPON_WIDTH_MM = 20.0
COUPON_HEIGHT_MM = 20.0
COUPON_SPAN_MM = 15.0


def _load_structural_analysis():
    path = os.path.join(REPO_ROOT, "tools", "structural_analysis.py")
    spec = importlib.util.spec_from_file_location(
        "structural_analysis_under_verify_full_sweep", path
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


sa = _load_structural_analysis()


@dataclass
class LayerFailure:
    layer_index: int
    print_z: float
    message: str
    unexpected: bool = False  # True if this was NOT a clean TopologyError


@dataclass
class SweepResult:
    """The honest outcome of one fixture's sweep.

    ``status`` is exactly one of:

    - ``"VERIFIED"``: every checked layer passed the real
      generate->slot->diff->validate pipeline. Exit code 0.
    - ``"FAILURES_FOUND"``: the pipeline ran to completion on every layer,
      and at least one layer genuinely failed ``TopologyValidator`` (or
      raised an unexpected exception mid-layer). This is the tool doing its
      job, not a tool failure. Exit code 1.
    - ``"NOT_VERIFIED"``: the sweep could not actually run to completion
      (shapely missing, or a structural problem with the fixture/config
      itself) -- ``reason`` says why. NEVER printed alongside a layer count
      that would read as a pass. Exit code 2.
    """

    status: str
    fixture: str
    reason: str = ""
    layers_expected: int = 0
    layers_checked: int = 0
    layers_skipped_root: int = 0
    layers_mutated: int = 0
    layers_shutdown: int = 0
    failures: List[LayerFailure] = field(default_factory=list)
    elapsed_s: float = 0.0

    def to_dict(self) -> Dict:
        return {
            "status": self.status,
            "fixture": self.fixture,
            "reason": self.reason,
            "layers_expected": self.layers_expected,
            "layers_checked": self.layers_checked,
            "layers_skipped_root": self.layers_skipped_root,
            "layers_mutated": self.layers_mutated,
            "layers_shutdown": self.layers_shutdown,
            "failure_count": len(self.failures),
            "failures": [
                {
                    "layer_index": f.layer_index,
                    "print_z": f.print_z,
                    "message": f.message,
                    "unexpected": f.unexpected,
                }
                for f in self.failures
            ],
            "elapsed_s": self.elapsed_s,
            "backend": "shapely mitre-join (empirical match, not a verified Clipper pin)",
        }


def _build_shapely_frame(ops: MiterShapelyOps, fixture: str):
    """Returns (frame, polygon, span_mm) for ``fixture`` in {"coupon", "airfoil"},
    using the real shapely backend throughout -- never the exact-arithmetic
    unit-test stub, which is documented as unable to represent this class of
    failure."""
    if fixture == "coupon":
        hw, hh = COUPON_WIDTH_MM / 2.0, COUPON_HEIGHT_MM / 2.0
        outline = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        span_mm = COUPON_SPAN_MM
    elif fixture == "airfoil":
        section = sa.build_airfoil_section("2412", 60.0, 60.0)
        outline = [tuple(p) for p in section.polygon.points]
        span_mm = section.span_mm
    else:
        raise ValueError("unknown fixture {!r}".format(fixture))
    poly = ops.polygon_from_points(outline)
    frame = LayerFrame.from_polygon(ops, poly)
    return frame, poly, span_mm


def run_full_sweep(cap_config: Dict, fixture: str, wall_width_mm: float = 0.42) -> SweepResult:
    """Full-layer real-boolean verification sweep for one fixture. Mirrors
    ``plugin/aircraft_structures/capability.py``'s ``_transform`` layer
    gating and validator wiring exactly (root_solid_layers skip, tip
    -shutdown-on-all-thin-omission, same slot_labels/family_indices
    construction) -- see this module's own docstring for why that fidelity
    is the point.
    """
    started = time.perf_counter()
    if not HAVE_SHAPELY:
        return SweepResult(
            status="NOT_VERIFIED",
            fixture=fixture,
            reason=(
                "shapely is not installed in this environment; this tool "
                "refuses to report a layer count without a real "
                "polygon-boolean backend (see tools/shapely_ops.py's "
                "own docstring)."
            ),
        )

    ops = MiterShapelyOps()
    try:
        frame, poly, span_mm = _build_shapely_frame(ops, fixture)
        spec = sa.build_spec_from_cap_config(cap_config)
    except Exception as exc:  # noqa: BLE001 - deliberate: report, do not crash
        return SweepResult(
            status="NOT_VERIFIED",
            fixture=fixture,
            reason="could not build the fixture/spec: {}: {}".format(
                type(exc).__name__, exc
            ),
        )

    n_layers = max(1, int(round(span_mm / LAYER_HEIGHT_MM)))
    generator = StructureGenerator()
    builder = SlotBuilder()
    validator = TopologyValidator(ops, safety_offset_mm=wall_width_mm)

    layers_skipped_root = 0
    layers_checked = 0
    layers_mutated = 0
    layers_shutdown = 0
    failures: List[LayerFailure] = []

    for index in range(n_layers):
        print_z = (index + 1) * LAYER_HEIGHT_MM

        # capability.py: layers below root_solid_layers stay solid,
        # unmutated, and are never handed to the generator/validator at
        # all -- mirrored here so "layers_checked" means exactly what the
        # real host would have checked, not every layer in the stack.
        if index < spec.root_solid_layers:
            layers_skipped_root += 1
            continue

        layers_checked += 1
        try:
            ctx = LayerContext(layer_index=index, print_z=print_z)
            generation = generator.generate(
                frame, spec, wall_width_mm, ctx, safety_offset_mm=wall_width_mm
            )
            intents = generation.intents
            omitted = generation.omitted

            if not intents:
                # capability.py's own tip-shutdown exception: a layer where
                # every omission is thinness-only, with the config opted in,
                # stays unmutated rather than failing the slice.
                thinness_prefixes = ("skin margin:", "thin section:")
                all_thin = bool(omitted) and all(
                    o.reason.startswith(thinness_prefixes) for o in omitted
                )
                if spec.tip_shutdown_enabled and all_thin:
                    layers_shutdown += 1
                    continue
                failures.append(
                    LayerFailure(
                        index,
                        print_z,
                        "every candidate web was omitted ({} candidate(s) "
                        "considered, none survived): {}".format(
                            len(omitted),
                            "; ".join(
                                "x={:.4f} mm ({})".format(o.x_mm, o.reason)
                                for o in omitted
                            )
                            or "no candidate stations were generated at all",
                        ),
                    )
                )
                continue

            built = [builder.build(i, spec.overshoot_mm) for i in intents]
            slots = [ops.polygon_from_points(b.points) for b in built]
            replacement = ops.diff(poly, slots)

            slot_labels = [
                "family {} ({!r})".format(i.family_index, i.family_name) for i in intents
            ]
            family_indices = [i.family_membership() for i in intents]

            validator.validate(
                [poly],
                replacement,
                slots,
                spec,
                index,
                print_z,
                slot_labels=slot_labels,
                family_indices=family_indices,
            )
            layers_mutated += 1
        except TopologyError as exc:
            failures.append(LayerFailure(index, print_z, str(exc), unexpected=False))
        except Exception as exc:  # noqa: BLE001 - deliberate: report, never crash the sweep
            failures.append(
                LayerFailure(
                    index,
                    print_z,
                    "UNEXPECTED (not a clean TopologyError): {}: {}".format(
                        type(exc).__name__, exc
                    ),
                    unexpected=True,
                )
            )

    layers_expected = n_layers - layers_skipped_root
    elapsed_s = time.perf_counter() - started

    if layers_checked != layers_expected:
        # Should be unreachable by construction (every non-skipped index
        # increments layers_checked exactly once) -- kept as an explicit
        # assertion rather than a silent trust that the loop above is
        # correct: the tool must refuse to report success when it cannot
        # actually check.
        return SweepResult(
            status="NOT_VERIFIED",
            fixture=fixture,
            reason=(
                "internal accounting mismatch: layers_checked={} but "
                "layers_expected={} (n_layers={}, layers_skipped_root={}) "
                "-- refusing to report a result that cannot be trusted.".format(
                    layers_checked, layers_expected, n_layers, layers_skipped_root
                )
            ),
            layers_expected=layers_expected,
            layers_checked=layers_checked,
            layers_skipped_root=layers_skipped_root,
            elapsed_s=elapsed_s,
        )

    status = "FAILURES_FOUND" if failures else "VERIFIED"
    return SweepResult(
        status=status,
        fixture=fixture,
        layers_expected=layers_expected,
        layers_checked=layers_checked,
        layers_skipped_root=layers_skipped_root,
        layers_mutated=layers_mutated,
        layers_shutdown=layers_shutdown,
        failures=failures,
        elapsed_s=elapsed_s,
    )


def _load_config(path: str) -> Dict:
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    return data["config"][0]["cap_config"]


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument(
        "--fixture", choices=("coupon", "airfoil", "both"), default="both"
    )
    parser.add_argument("--wall-width-mm", type=float, default=0.42)
    parser.add_argument(
        "--out",
        default=os.path.join(RESULTS_DIR, "m18_full_sweep_verification.json"),
    )
    args = parser.parse_args(argv)

    cap_config = _load_config(args.config)
    fixtures = ["coupon", "airfoil"] if args.fixture == "both" else [args.fixture]

    results = {}
    worst_exit = 0
    for fixture in fixtures:
        result = run_full_sweep(cap_config, fixture, wall_width_mm=args.wall_width_mm)
        results[fixture] = result.to_dict()
        exit_code = {"VERIFIED": 0, "FAILURES_FOUND": 1, "NOT_VERIFIED": 2}[result.status]
        worst_exit = max(worst_exit, exit_code)
        print(
            "{}: {} (checked {}/{} layers, {} mutated, {} shutdown, {} "
            "failure(s), {:.1f}s){}".format(
                fixture,
                result.status,
                result.layers_checked,
                result.layers_expected,
                result.layers_mutated,
                result.layers_shutdown,
                len(result.failures),
                result.elapsed_s,
                "" if not result.reason else " -- {}".format(result.reason),
            )
        )
        for f in result.failures[:5]:
            print("    layer {} (Z={:.2f} mm): {}".format(f.layer_index, f.print_z, f.message))
        if len(result.failures) > 5:
            print("    ... and {} more".format(len(result.failures) - 5))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(
            {"config": args.config, "wall_width_mm": args.wall_width_mm, "results": results},
            handle,
            indent=2,
        )
    print("Wrote {}".format(args.out))
    return worst_exit


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
