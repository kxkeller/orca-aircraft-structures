#!/usr/bin/env python3
"""Build the full wing fixture: tapered AND twisted, at a real chord.

WHY THIS EXISTS. Every tracked fixture in ``tests/fixtures/`` is 60 mm chord
and varies in exactly one way at a time -- ``make_fixtures.py`` has
``make_taper`` (chord varies, no rotation) and ``make_twist`` (rotation varies,
constant chord), and nothing that does both. A real wing panel does both at
once, and the two mechanisms are not independent: taper and twist are ONE
mechanism with two sources of far-surface slope, and correcting for global
rotation alone is right in direction but short in magnitude, because local
slope subsumes rotation rather than stacking with it. A fixture that carries only one of them cannot exercise that.

This is a SEPARATE module rather than a new subcommand inside
``make_fixtures.py`` deliberately: ``tools/verify_full_sweep_varying.py``
imports ``make_fixtures`` and rebuilds per-layer frames from its own
``params_fn``/``build_loft_stations``, so an edit there would change the
fixtures those sweeps are measured against. Everything here is a thin wrapper around
``make_fixtures``'s own helpers -- the unit outline, the station builder, the
lofter and the STL writer are all imported, never re-derived -- so "this
fixture uses the same geometry pipeline as the tracked fixtures" is a checked
claim rather than an assumption.

GEOMETRY. ``params_fn(z) -> (chord_mm, rotate_deg, dx_mm)`` with:

  chord(z)  = root + (tip - root) * z/span          affine in z (exact)
  rotate(z) = washout_deg * z/span                  NOT affine in z

Rotation through an angle is not affine in Z, exactly as ``make_twist``'s own
docstring records, so this evaluates ``params_fn`` at MANY stations rather
than lofting between two ends. ``pivot_frac_x=0.25`` rotates about the
quarter-chord point; because chord ALSO varies here, that pivot traces a
swept line down the span rather than a straight one -- which is what a real
tapered wing rotating about its quarter-chord does, and is the intended
geometry, not an artifact.

Sign convention is inherited unchanged from ``make_twist``: positive
``washout_deg`` tilts the chord line nose-down toward the tip (genuine
washout, reduced local incidence), and this module asserts that on the BUILT
geometry rather than trusting it.

Usage:
    python tools/make_wing.py
    python tools/make_wing.py --root-chord 150 --tip-chord 90 --span 400 --washout 3
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
for _p in (str(REPO_ROOT), str(TOOLS_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import make_fixtures as mf  # noqa: E402


def wing_params_fn(root_chord_mm: float, tip_chord_mm: float, span_mm: float,
                   washout_deg: float):
    """The (chord, rotation, dx) function defining a tapered+twisted wing.

    Exposed as its own function so that a verification sweep can evaluate
    the SAME formula at each layer's own Z rather than interpolating
    between built stations, which ``verify_full_sweep_varying.py`` avoids
    for twist.
    """
    def params(z: float):
        frac = z / span_mm
        chord = root_chord_mm + (tip_chord_mm - root_chord_mm) * frac
        return chord, washout_deg * frac, 0.0
    return params


def chord_at(root_chord_mm: float, tip_chord_mm: float, span_mm: float, z: float) -> float:
    return root_chord_mm + (tip_chord_mm - root_chord_mm) * (z / span_mm)


def make_wing(repo_root: Path, naca: str, root_chord_mm: float, tip_chord_mm: float,
              span_mm: float, washout_deg: float, n_points: int, n_stations: int):
    m, p, t = mf.parse_naca4(naca)
    params = wing_params_fn(root_chord_mm, tip_chord_mm, span_mm, washout_deg)

    z_stations = [span_mm * i / (n_stations - 1) for i in range(n_stations)]
    stations, outline_unit, root_shift = mf.build_loft_stations(
        m, p, t, n_points, z_stations, params, pivot_frac_x=0.25
    )
    triangles = mf.loft_triangles(stations)

    # Read the washout sign back off the BUILT geometry, exactly as
    # make_twist does: LE is outline vertex 0, TE is vertex n_points-1 by
    # naca4_section_outline's fixed ordering.
    def chord_line_angle_deg(outline_mm):
        le = outline_mm[0]
        te = outline_mm[n_points - 1]
        return math.degrees(math.atan2(te[1] - le[1], te[0] - le[0]))

    angle_root = chord_line_angle_deg(stations[0][1])
    angle_tip = chord_line_angle_deg(stations[-1][1])
    if washout_deg > 0 and not (angle_tip > angle_root):
        raise SystemExit(
            "FAIL: positive washout_deg must tilt the tip chord line nose-down "
            "(angle_tip > angle_root), got root=%.4f tip=%.4f" % (angle_root, angle_tip)
        )

    out_dir = repo_root / "tests" / "fixtures" / "wing"
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = "naca%s_wing_r%d_t%d_s%d_w%g" % (
        naca, int(root_chord_mm), int(tip_chord_mm), int(span_mm), washout_deg)
    stl_path = out_dir / (stem + ".stl")
    mf.write_binary_stl(stl_path, triangles)
    stl_sha256 = mf.sha256_of(stl_path)

    thickness_rows = mf._thickness_table(
        m, p, t, n_points,
        lambda z: chord_at(root_chord_mm, tip_chord_mm, span_mm, z),
        [0.0, span_mm * 0.25, span_mm * 0.5, span_mm * 0.75, span_mm],
    )

    manifest = {
        "fixture": "wing",
        "stem": stem,
        "stl": str(stl_path.relative_to(repo_root)).replace("\\", "/"),
        "stl_sha256": stl_sha256,
        "naca": naca,
        "root_chord_mm": root_chord_mm,
        "tip_chord_mm": tip_chord_mm,
        "span_mm": span_mm,
        "washout_deg": washout_deg,
        "n_points": n_points,
        "n_stations": n_stations,
        "pivot_frac_x": 0.25,
        "triangles": len(triangles),
        "chord_line_angle_root_deg": angle_root,
        "chord_line_angle_tip_deg": angle_tip,
        "taper_ratio": tip_chord_mm / root_chord_mm,
    }
    (out_dir / (stem + ".json")).write_text(json.dumps(manifest, indent=2))

    (out_dir / "README.md").write_text(
        "# Wing fixture: %s.stl\n\n"
        "## Description\n"
        "A tapered AND twisted NACA %s wing panel -- the first fixture in this\n"
        "repository that varies chord and rotation at the same time, and the first\n"
        "above 60 mm chord. Chord runs %g mm at the root (Z=0) to %g mm at the tip\n"
        "(Z=%g mm), linear in Z. The section simultaneously rotates about the\n"
        "quarter-chord point by %g degrees of washout, linear in Z.\n\n"
        "Because chord varies, the quarter-chord pivot traces a SWEPT line down the\n"
        "span rather than a straight one. That is intended: it is what a real\n"
        "tapered wing rotating about its quarter-chord does.\n\n"
        "## Why it exists\n"
        "D022 established that taper and twist are one mechanism with two sources of\n"
        "far-surface slope, and that correcting for global rotation alone is short in\n"
        "magnitude because local slope subsumes rotation. No single-mechanism fixture\n"
        "can exercise that. This one carries both at a realistic chord.\n\n"
        "## Built geometry, read back (not asserted)\n"
        "- chord-line angle at root: %.4f deg\n"
        "- chord-line angle at tip:  %.4f deg (positive washout tilts nose-down)\n"
        "- taper ratio: %.4f\n"
        "- triangles: %d\n"
        "- sha256: `%s`\n\n"
        "## Origin convention\n"
        "Same root-bounding-box-centring convention as the taper, sweep and twist\n"
        "fixtures (shared `build_loft_stations`). Z is spanwise, root at Z=0.\n\n"
        "## Section thickness\n%s\n"
        % (stem, naca, root_chord_mm, tip_chord_mm, span_mm, washout_deg,
           angle_root, angle_tip, tip_chord_mm / root_chord_mm, len(triangles),
           stl_sha256, mf._thickness_table_md(thickness_rows))
    )

    print("wrote %s" % stl_path)
    print("  sha256      %s" % stl_sha256)
    print("  triangles   %d" % len(triangles))
    print("  chord       %g -> %g mm over %g mm span (taper ratio %.3f)"
          % (root_chord_mm, tip_chord_mm, span_mm, tip_chord_mm / root_chord_mm))
    print("  washout     %g deg  (chord line %.4f -> %.4f deg, nose-down at tip)"
          % (washout_deg, angle_root, angle_tip))
    return manifest


def main(argv=None):
    ap = argparse.ArgumentParser(description="Build the tapered+twisted full wing fixture")
    ap.add_argument("--naca", default="2412")
    ap.add_argument("--root-chord", type=float, default=150.0)
    ap.add_argument("--tip-chord", type=float, default=90.0)
    ap.add_argument("--span", type=float, default=400.0)
    ap.add_argument("--washout", type=float, default=3.0)
    ap.add_argument("--points", type=int, default=120)
    ap.add_argument("--stations", type=int, default=41)
    a = ap.parse_args(argv)
    make_wing(REPO_ROOT, a.naca, a.root_chord, a.tip_chord, a.span,
              a.washout, a.points, a.stations)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
