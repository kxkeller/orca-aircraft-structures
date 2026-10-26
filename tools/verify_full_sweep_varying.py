#!/usr/bin/env python3
"""Full-layer, real-boolean verification sweep for VARYING cross-sections.

``tools/verify_full_sweep.py`` drives the real
generate -> slot -> diff -> validate pipeline over every layer of exactly
two CONSTANT-section fixtures (the coupon, NACA 2412 c60). This script
reuses the identical per-layer pipeline and layer-gating logic (root_solid
_layers skip, tip-shutdown-on-all-thin-omission), but builds a FRESH
``LayerFrame``/polygon at every layer from the same station-outline
functions ``tools/make_fixtures.py`` uses to build the tracked taper/
sweep/twist STL fixtures -- so a tapered or twisted wing gets a genuinely
different cross-section at every Z, exactly like the real host does
(``plugin/aircraft_structures/capability.py:1173`` builds
``LayerFrame.from_polygon`` fresh inside the per-layer loop too).

Every fixture builder here is a THIN wrapper around
``tools/make_fixtures.py``'s own ``params_fn``/``build_loft_stations``
functions -- not a re-derivation -- so "this sweep uses the same geometry
as the tracked STL fixture" is a checked claim, not an assumption: taper
and sweep are geometrically EXACT at any Z (chord/offset affine in Z, see
``make_taper``/``make_sweep`` docstrings), and twist is evaluated by
calling the exact same ``rotate_deg = washout_deg * frac`` formula at the
layer's own Z (not linear-interpolated between two rotated end stations),
matching ``make_twist``'s own per-station construction.

Usage:
    python tools/verify_full_sweep_varying.py taper
    python tools/verify_full_sweep_varying.py twist
    python tools/verify_full_sweep_varying.py airfoil --naca 2415 --chord 60
    python tools/verify_full_sweep_varying.py airfoil --naca 2409 --chord 60
    python tools/verify_full_sweep_varying.py airfoil --naca 2412 --chord 200
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
TOOLS_DIR = os.path.join(REPO_ROOT, "tools")
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import LayerContext, StructureGenerator  # noqa: E402
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402

DEFAULT_CONFIG_PATH = os.path.join(
    REPO_ROOT, "presets", "plugin", "skin_lattice_only_config.json"
)
RESULTS_DIR = os.path.join(REPO_ROOT, "docs", "results")
LAYER_HEIGHT_MM = 0.2


def _load_module(name: str, relpath: str):
    path = os.path.join(REPO_ROOT, relpath)
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


sa = _load_module("structural_analysis_under_verify_full_sweep_varying", "tools/structural_analysis.py")
mf = _load_module("make_fixtures_under_verify_full_sweep_varying", "tools/make_fixtures.py")


@dataclass
class LayerFailure:
    layer_index: int
    print_z: float
    message: str
    unexpected: bool = False


@dataclass
class SweepResult:
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


def _outline_at_z_taper(m, p, t, n_points, root_chord_mm, tip_chord_mm, span_mm, z):
    def params(zz):
        frac = zz / span_mm
        chord = root_chord_mm + (tip_chord_mm - root_chord_mm) * frac
        return chord, 0.0, 0.0

    outline_unit = mf.naca4_section_outline(m, p, t, n_points)
    chord0, rot0, dx0 = params(0.0)
    root_outline = mf.build_station_outline(outline_unit, chord0, rot0, 0.0, dx0, 0.0, (0.0, 0.0))
    xmin, ymin, xmax, ymax = mf.section_bbox_2d(root_outline)
    root_shift = ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0)
    chord, rot, dx = params(z)
    return mf.build_station_outline(outline_unit, chord, rot, 0.0, dx, 0.0, root_shift)


def _outline_at_z_twist(m, p, t, n_points, chord_mm, span_mm, washout_deg, z):
    def params(zz):
        frac = zz / span_mm
        return chord_mm, washout_deg * frac, 0.0

    outline_unit = mf.naca4_section_outline(m, p, t, n_points)
    chord0, rot0, dx0 = params(0.0)
    root_outline = mf.build_station_outline(outline_unit, chord0, rot0, 0.25, dx0, 0.0, (0.0, 0.0))
    xmin, ymin, xmax, ymax = mf.section_bbox_2d(root_outline)
    root_shift = ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0)
    chord, rot, dx = params(z)
    return mf.build_station_outline(outline_unit, chord, rot, 0.25, dx, 0.0, root_shift)


def _outline_at_z_wing(m, p, t, n_points, root_chord_mm, tip_chord_mm, span_mm,
                       washout_deg, z):
    """full wing: taper AND twist together, the two slope sources at once.
    Deliberately identical in form to ``tools/make_wing.py``'s own
    ``wing_params_fn`` -- chord affine in z, rotation ``washout_deg * frac``
    evaluated at the layer's OWN z (never interpolated between built stations,
    matching ``_outline_at_z_twist``'s treatment) -- so the swept geometry is
    the tracked STL's geometry rather than an approximation of it.
    ``pivot_frac_x=0.25`` matches the builder; because chord also varies here
    that pivot traces a swept line, which is intended.
    """
    def params(zz):
        frac = zz / span_mm
        chord = root_chord_mm + (tip_chord_mm - root_chord_mm) * frac
        return chord, washout_deg * frac, 0.0

    outline_unit = mf.naca4_section_outline(m, p, t, n_points)
    chord0, rot0, dx0 = params(0.0)
    root_outline = mf.build_station_outline(outline_unit, chord0, rot0, 0.25, dx0, 0.0, (0.0, 0.0))
    xmin, ymin, xmax, ymax = mf.section_bbox_2d(root_outline)
    root_shift = ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0)
    chord, rot, dx = params(z)
    return mf.build_station_outline(outline_unit, chord, rot, 0.25, dx, 0.0, root_shift)


def _outline_constant(m, p, t, n_points, chord_mm, _z):
    outline_unit = mf.naca4_section_outline(m, p, t, n_points)
    scaled = [(x * chord_mm, y * chord_mm) for x, y in outline_unit]
    xs = [x for x, _ in scaled]
    ys = [y for _, y in scaled]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
    return [(x - cx, y - cy) for x, y in scaled]


def _build_outline_fn(fixture: str, args) -> Tuple[callable, float]:
    """Returns (outline_at_z(z) -> list[(x, y)], span_mm)."""
    n_points = args.points
    if fixture == "taper":
        m, p, t = mf.parse_naca4(args.naca)
        span_mm = args.span
        fn = lambda z: _outline_at_z_taper(  # noqa: E731
            m, p, t, n_points, args.root_chord, args.tip_chord, span_mm, z
        )
        return fn, span_mm
    if fixture == "twist":
        m, p, t = mf.parse_naca4(args.naca)
        span_mm = args.span
        fn = lambda z: _outline_at_z_twist(  # noqa: E731
            m, p, t, n_points, args.chord, span_mm, args.washout, z
        )
        return fn, span_mm
    if fixture == "wing":
        m, p, t = mf.parse_naca4(args.naca)
        span_mm = args.span
        fn = lambda z: _outline_at_z_wing(  # noqa: E731
            m, p, t, n_points, args.root_chord, args.tip_chord, span_mm,
            args.washout, z
        )
        return fn, span_mm
    if fixture == "airfoil":
        m, p, t = mf.parse_naca4(args.naca)
        span_mm = args.span
        fn = lambda z: _outline_constant(m, p, t, n_points, args.chord, z)  # noqa: E731
        return fn, span_mm
    raise ValueError("unknown fixture {!r}".format(fixture))


def _load_config(path: str) -> Dict:
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    return data["config"][0]["cap_config"]


def run_full_sweep(cap_config: Dict, fixture: str, args, wall_width_mm: float = 0.42) -> SweepResult:
    started = time.perf_counter()
    if not HAVE_SHAPELY:
        return SweepResult(
            status="NOT_VERIFIED",
            fixture=fixture,
            reason="shapely is not installed in this environment.",
        )

    ops = MiterShapelyOps()
    try:
        outline_fn, span_mm = _build_outline_fn(fixture, args)
        spec = sa.build_spec_from_cap_config(cap_config)
    except Exception as exc:  # noqa: BLE001
        return SweepResult(
            status="NOT_VERIFIED",
            fixture=fixture,
            reason="could not build the fixture/spec: {}: {}".format(type(exc).__name__, exc),
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

        if index < spec.root_solid_layers:
            layers_skipped_root += 1
            continue

        layers_checked += 1
        try:
            # A fresh polygon/frame every layer -- the real host's own
            # per-layer construction (capability.py:1173), needed here
            # because taper/twist genuinely differ layer to layer.
            outline = outline_fn(print_z)
            poly = ops.polygon_from_points(outline)
            frame = LayerFrame.from_polygon(ops, poly)

            ctx = LayerContext(layer_index=index, print_z=print_z)
            generation = generator.generate(
                frame, spec, wall_width_mm, ctx, safety_offset_mm=wall_width_mm
            )
            intents = generation.intents
            omitted = generation.omitted

            if not intents:
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
        except Exception as exc:  # noqa: BLE001
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
        return SweepResult(
            status="NOT_VERIFIED",
            fixture=fixture,
            reason="internal accounting mismatch: layers_checked={} but layers_expected={}".format(
                layers_checked, layers_expected
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


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture", choices=("taper", "twist", "airfoil", "wing"))
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--naca", default="2412")
    parser.add_argument("--chord", type=float, default=60.0)
    parser.add_argument("--root-chord", type=float, default=60.0)
    parser.add_argument("--tip-chord", type=float, default=30.0)
    parser.add_argument("--span", type=float, default=60.0)
    parser.add_argument("--washout", type=float, default=6.0)
    parser.add_argument("--points", type=int, default=120)
    parser.add_argument("--wall-width-mm", type=float, default=0.42)
    parser.add_argument("--out", default=None)
    args = parser.parse_args(argv)

    cap_config = _load_config(args.config)
    result = run_full_sweep(cap_config, args.fixture, args, wall_width_mm=args.wall_width_mm)

    print(
        "{}: {} (checked {}/{} layers, {} mutated, {} shutdown, {} "
        "failure(s), {:.1f}s){}".format(
            args.fixture,
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
    for f in result.failures[:10]:
        print("    layer {} (Z={:.2f} mm): {}".format(f.layer_index, f.print_z, f.message))
    if len(result.failures) > 10:
        print("    ... and {} more".format(len(result.failures) - 10))

    out_path = args.out or os.path.join(
        RESULTS_DIR, "m14_varying_sweep_{}.json".format(args.fixture)
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(
            {
                "config": args.config,
                "fixture": args.fixture,
                "args": vars(args),
                "result": result.to_dict(),
            },
            handle,
            indent=2,
            default=str,
        )
    print("Wrote {}".format(out_path))

    return {"VERIFIED": 0, "FAILURES_FOUND": 1, "NOT_VERIFIED": 2}[result.status]


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
