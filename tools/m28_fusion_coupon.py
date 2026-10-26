#!/usr/bin/env python3
"""Fusion coupon: a hand-pullable dogbone that reproduces the actual socket/flank
joint geometry across a gap sweep, so the fusion ladder can be tested with real
print evidence.

THE JOINT BEING REPRODUCED: an ordinary member wall laterally beside a curved
socket wall, separated by a wall-to-wall gap. A validated station sits about 1.005 mm
past the socket's circle edge, giving wall gaps of 0.006-0.016 mm. The coupon
does not reproduce that offset (it depends on the socket radius and is derived
per layer in the real generator); it reproduces the GEOMETRY (curved socket
flank beside a straight member flank) at CONTROLLED gaps spanning the whole
fusion ladder, so the sensitivity of fusion to gap is visible.

GEOMETRY, per specimen (one gap value each; stdlib-only, deterministic):

    A single constant cross-section prism (extruded from Z=0 to Z=HEIGHT_MM),
    shaped like a dogbone: two rectangular grip pads at the ends, and a
    narrow "waist" between them where the ONLY connection from one pad to
    the other is a socket-flank joint at the target gap.

    Waist cross-section (2D), symmetric about y=0, centred at x=xc:
    - MEMBER FLANK (top): a straight rectangular notch cut inward from the
      top edge (y=+HALF_H), flat-bottomed at y=+W/2, spanning x in
      [xc-R, xc+R]. This is the straight member wall.
    - SOCKET FLANK (bottom): a circular arc cut inward from the bottom edge
      (y=-HALF_H), of radius R (the socket radius: 5.0 mm spar / 2 +
      0.15 mm spar_clearance_mm default, aircraft_core.config_schema), whose
      topmost point (the apex, closest approach to the member flank) sits at
      y=-W/2. This is the curved socket wall.
    - W = 2 * OUTER_WALL_LINE_WIDTH_MM + gap_mm is the SOLID ligament width
      at x=xc. OrcaSlicer's classic
      wall generator insets each surface by half a line width, so two flanks
      W apart end up with bead CENTRELINES (W - line_width) apart and bead
      EDGES (W - 2*line_width) = gap_mm apart -- i.e. building the ligament
      at exactly this width reproduces the intended bead-to-bead air gap
      without needing to model two disjoint solids (which would break the
      one-loop-per-layer vase requirement).
    - The cross-section is narrowest (=W) only at x=xc and widens away from
      it, so a hand pull test is forced to fail at the joint, not the grips.

    Each grip pad also carries a tally of small rectangular notches cut into
    its top edge -- one notch per specimen's position in the gap sweep (1 for
    the smallest gap... 7 for the largest) -- so the printed part is
    identifiable by eye/touch without needing embossed text. The notches sit
    well inside the grip pad, away from the waist and the seam under test, and
    are ordinary boundary notches (same technique as the slits) so they cannot
    introduce a second loop.

Usage:
    python tools/m28_fusion_coupon.py [--outdir <dir>]
"""

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from make_fixtures import (  # noqa: E402
    check_watertight,
    parse_binary_stl,
    polygon_area_2d,
    sha256_of,
    tri_normal,
    write_binary_stl,
)
from m22_neck_coupon import ear_clip  # noqa: E402 (reuse the ear-clip triangulator)

# ---------------------------------------------------------------------------
# Fixed geometry constants.
# ---------------------------------------------------------------------------

LINE_WIDTH_MM = 0.42          # outer_wall_line_width, pinned to the real setting
LAYER_HEIGHT_MM = 0.20
HEIGHT_MM = 18.0              # tall enough for a meaningful seam (req: 15-20 mm)

# Socket radius: spar_diameter_mm / 2 + spar_clearance_mm at their schema
# defaults (aircraft_core/config_schema.py: spar_clearance_mm default 0.15),
# for a 10 mm spar (the m18_wing_spar10 fixture used).
SPAR_DIAMETER_MM = 10.0
SPAR_CLEARANCE_MM = 0.15
SOCKET_RADIUS_MM = SPAR_DIAMETER_MM / 2.0 + SPAR_CLEARANCE_MM  # 5.15 mm

HALF_H = 9.0                  # +-9 mm => 18 mm tall grip pads, good hand grip
GRIP_LEN_MM = 15.0            # length of each grip pad along the pull axis
ARC_POINTS_PER_SIDE = 24      # arc sampling resolution (quarter circle each side)

TALLY_NOTCH_WIDTH_MM = 0.6
TALLY_NOTCH_DEPTH_MM = 1.0
TALLY_NOTCH_PITCH_MM = 1.5
TALLY_NOTCH_START_MM = 2.0    # offset from the grip pad's outer end

# The gap sweep: the measured ladder (0.021 fused / 0.042 fused-separable /
# 0.105 no-bond / 0.210 no-bond) bracketed at finer resolution, to show
# sensitivity rather than a single point. 0.210 is omitted: 0.105 is already
# past the "no bond" threshold, so a second confirmed-negative control is not
# needed.
GAPS_MM = [0.000, 0.011, 0.021, 0.032, 0.042, 0.063, 0.105]

# Measured bond anchors (gap -> observed outcome), for the report to compare
# against.
# Values between anchors are NOT measured -- that gap is exactly what this
# coupon exists to close.
D006_ANCHORS_MM = {0.021: "fused; takes real effort to pull apart",
                    0.042: "fused, but separable by hand",
                    0.105: "no bond",
                    0.210: "no bond"}


def waist_center_x():
    return GRIP_LEN_MM + SOCKET_RADIUS_MM


def total_length_mm():
    return 2.0 * GRIP_LEN_MM + 2.0 * SOCKET_RADIUS_MM


def ligament_width_mm(gap_mm):
    return 2.0 * LINE_WIDTH_MM + gap_mm


def predicted_bead_centreline_separation_mm(gap_mm):
    """W - line_width: the same arithmetic measured and confirmed against real
    G-code."""
    return ligament_width_mm(gap_mm)- LINE_WIDTH_MM


def d006_expectation(gap_mm):
    if gap_mm in D006_ANCHORS_MM:
        return D006_ANCHORS_MM[gap_mm] + " (measured anchor)"
    if gap_mm < 0.021:
        return "predicted fused (below the 0.021 mm anchor) -- UNMEASURED"
    if gap_mm < 0.042:
        return "predicted fused/separable, interpolated between the 0.021 and 0.042 mm anchors -- UNMEASURED"
    if gap_mm < 0.105:
        return "UNKNOWN -- between the 0.042 mm (separable) and 0.105 mm (no bond) anchors; no measurement exists in this range"
    return "predicted no bond (measured 0.105 mm and 0.210 mm both no-bond) -- UNMEASURED at exactly this value"


def _tally_notch_run(n_notches, x0, x1, y_top):
    """Build the top-edge boundary points from x0 to x1 (x0 < x1), at
    height y_top, with n_notches small rectangular notches cut inward,
    evenly spaced starting TALLY_NOTCH_START_MM from x0. Returns points in
    increasing-x order (caller reverses if the CCW walk needs right-to-left).

    Notches are placed well clear of both ends: the last notch's right edge
    must stay inside [x0, x1] with margin, checked by an assertion so a
    geometry change that overflows the grip pad fails loudly instead of
    silently overlapping the waist.
    """
    pts = [(x0, y_top)]
    for i in range(n_notches):
        nx0 = x0 + TALLY_NOTCH_START_MM + i * TALLY_NOTCH_PITCH_MM
        nx1 = nx0 + TALLY_NOTCH_WIDTH_MM
        assert nx1 < x1 - 1.0, (
            f"tally notch {i} (x {nx0:.2f}..{nx1:.2f}) runs into the waist "
            f"(grip pad spans {x0:.2f}..{x1:.2f}); shrink pitch/width or "
            f"lengthen the grip pad"
        )
        y_bot = y_top - TALLY_NOTCH_DEPTH_MM
        pts.append((nx0, y_top))
        pts.append((nx0, y_bot))
        pts.append((nx1, y_bot))
        pts.append((nx1, y_top))
    pts.append((x1, y_top))
    return pts


def build_cross_section_polygon(gap_mm, n_tally_notches):
    """Build the closed 2D dogbone cross-section (CCW, no repeated first/
    last point), for the given fusion gap and tally-mark count.
    """
    xc = waist_center_x()
    R = SOCKET_RADIUS_MM
    L = total_length_mm()
    W = ligament_width_mm(gap_mm)
    Wm = W / 2.0
    yc = -Wm - R

    # --- bottom boundary: left grip flat -> vertical wall down into the arc
    # start -> arc (theta -90 .. +90, i.e. xc-R,yc through the apex xc,-Wm
    # to xc+R,yc) -> vertical wall back down -> right grip flat.
    bottom = [(0.0, -HALF_H), (xc - R, -HALF_H), (xc - R, yc)]
    n = ARC_POINTS_PER_SIDE * 2
    for i in range(1, n):  # exclude the two endpoints, already placed/placed below
        theta = math.radians(-90.0 + 180.0 * i / n)
        bottom.append((xc + R * math.sin(theta), yc + R * math.cos(theta)))
    bottom.append((xc + R, yc))
    bottom.append((xc + R, -HALF_H))
    bottom.append((L, -HALF_H))

    # --- right edge, then top boundary walked right-to-left: right grip
    # flat -> vertical wall down into the member notch -> notch floor ->
    # vertical wall up -> left grip flat (with the tally notches).
    top_right_grip = [(L, HALF_H), (xc + R, HALF_H), (xc + R, Wm),
                       (xc - R, Wm), (xc - R, HALF_H)]

    left_grip_top = _tally_notch_run(n_tally_notches, 0.0, xc - R, HALF_H)
    left_grip_top_rev = list(reversed(left_grip_top))[1:]  # drop dup at xc-R

    polygon = bottom + top_right_grip + left_grip_top_rev
    return polygon, xc, R, W, Wm, yc


def expected_cross_section_area(gap_mm, n_tally_notches):
    """Analytic cross-section area, computed against the SAME polygonal
    (chorded) approximation of the arc that build_cross_section_polygon
    actually emits -- not the ideal smooth semicircle, whose area a chorded
    n-gon systematically undercuts. Using the true continuous-arc area here
    would make this self-check fail on discretisation error rather than on
    a real bug (found empirically: n=48 segments under-carves the notch by
    about 0.03 mm^2, comfortably outside a naive 1e-6 mm^2 tolerance).
    """
    xc = waist_center_x()
    R = SOCKET_RADIUS_MM
    L = total_length_mm()
    W = ligament_width_mm(gap_mm)
    Wm = W / 2.0
    yc = -Wm - R

    area = L * (2.0 * HALF_H)
    # Top (member) notch: rectangle, width 2R, height (HALF_H - Wm).
    area -= 2.0 * R * (HALF_H - Wm)
    # Bottom (socket) notch: rectangle up to yc, plus the polygonal (fan-
    # triangulated) approximation of the upper semicircle cap above yc up
    # to the apex -- n segments of a regular polygon inscribed in a circle
    # of radius R, each subtending angle (pi / n) radians.
    n = ARC_POINTS_PER_SIDE * 2
    segment_angle = math.pi / n
    fan_area = n * 0.5 * R * R * math.sin(segment_angle)
    area -= 2.0 * R * (HALF_H + yc) + fan_area
    # Tally notches: simple rectangles.
    area -= n_tally_notches * TALLY_NOTCH_WIDTH_MM * TALLY_NOTCH_DEPTH_MM
    return area


def build_specimen(gap_mm, n_tally_notches):
    polygon, xc, R, W, Wm, yc = build_cross_section_polygon(gap_mm, n_tally_notches)
    triangles = _build_triangles(polygon)
    return polygon, triangles, dict(xc=xc, R=R, W=W, Wm=Wm, yc=yc)


def _build_triangles(polygon):
    """Extrude `polygon` from Z=0 to Z=HEIGHT_MM into a watertight mesh.
    Reimplemented (not imported) because m22_neck_coupon.build_triangles
    hardcodes its own Z0/Z1 module globals; this keeps HEIGHT_MM local and
    explicit instead of monkeypatching another module's constants.
    """
    n = len(polygon)
    cap_tris_idx = ear_clip(polygon)

    bottom = [(x, y, 0.0) for x, y in polygon]
    top = [(x, y, HEIGHT_MM) for x, y in polygon]

    triangles = []
    for i, j, k in cap_tris_idx:
        v0, v1, v2 = bottom[i], bottom[k], bottom[j]
        triangles.append((tri_normal(v0, v1, v2), v0, v1, v2))
    for i, j, k in cap_tris_idx:
        v0, v1, v2 = top[i], top[j], top[k]
        triangles.append((tri_normal(v0, v1, v2), v0, v1, v2))
    for i in range(n):
        j = (i + 1) % n
        a, b = bottom[i], bottom[j]
        c, d = top[j], top[i]
        triangles.append((tri_normal(a, b, c), a, b, c))
        triangles.append((tri_normal(a, c, d), a, c, d))
    return triangles


def gap_slug(gap_mm):
    return f"{gap_mm:.3f}".replace(".", "p")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--outdir", type=Path, default=None,
                         help="Output directory (default: tests/fixtures/coupon/)")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    outdir = args.outdir if args.outdir is not None else (repo_root / "tests" / "fixtures" / "coupon")

    print(f"socket radius (spar10 default): {SOCKET_RADIUS_MM:.3f} mm")
    print(f"waist centre x: {waist_center_x():.3f} mm; total length: {total_length_mm():.3f} mm")
    print()
    header = (f"{'gap_mm':>7} {'ligament_W':>10} {'bead_centre_sep':>16} "
              f"{'file':>34}")
    print(header)

    results = []
    for idx, gap_mm in enumerate(GAPS_MM, start=1):
        polygon, triangles, meta = build_specimen(gap_mm, idx)

        # Self-check 1: watertight.
        ok, details, problems = check_watertight(triangles)
        if not ok:
            print(f"[gap {gap_mm}] watertight: FAIL -- {details}")
            for p in problems[:20]:
                print(f"    {p}")
            raise SystemExit(f"FAIL: mesh for gap={gap_mm} is not watertight")

        # Self-check 2: signed area vs analytic expectation.
        area = polygon_area_2d(polygon)
        expected_area = expected_cross_section_area(gap_mm, idx)
        assert area > 0.0, f"gap={gap_mm}: cross-section polygon must be CCW (positive area)"
        assert abs(area - expected_area) < 1e-6, (
            f"gap={gap_mm}: area mismatch: {area!r} vs expected {expected_area!r}"
        )

        out_path = outdir / f"m28_fusion_gap_{gap_slug(gap_mm)}mm.stl"
        write_binary_stl(out_path, triangles)

        # Self-check 3: round-trip triangle count.
        _header, reread = parse_binary_stl(out_path)
        assert len(reread) == len(triangles), f"gap={gap_mm}: triangle count did not round-trip"

        digest = sha256_of(out_path)
        bead_sep = predicted_bead_centreline_separation_mm(gap_mm)
        print(f"{gap_mm:>7.3f} {meta['W']:>10.4f} {bead_sep:>16.4f} {str(out_path.name):>34}")
        results.append(dict(
            gap_mm=gap_mm, ligament_W_mm=meta["W"],
            predicted_bead_centreline_sep_mm=bead_sep,
            predicted_bead_air_mm=gap_mm,  # by construction, W - 2*line_width == gap_mm
            n_tally_notches=idx, watertight_details=details,
            area_mm2=area, expected_area_mm2=expected_area,
            sha256=digest, triangles=len(triangles), path=str(out_path),
            d006_expectation=d006_expectation(gap_mm),
        ))

    print()
    print("all specimens: watertight, area-checked, round-tripped, written.")
    return results


if __name__ == "__main__":
    main()
