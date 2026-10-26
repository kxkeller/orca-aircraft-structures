#!/usr/bin/env python3
"""Neck coupon: a constant cross-section prism with eight paired notches cut
inward from the +Y and -Y edges, leaving a narrow "neck" of ligament material
of width W between two collinear opposing slits at each station.

This is a measurement fixture, not production code: it exists to be sliced
in stock OrcaSlicer Spiral Vase mode with NO plugin loaded, so that the
plugin-free baseline behaviour of the perimeter generator at a narrow neck
of material can be measured. All structure must therefore come from the STL
alone.

Block: X in [0, 90], Y in [-6, 6], Z in [0, 20] mm.

Eight stations, evenly spaced at x = 10, 20, ..., 80 mm. At each station
(x_c, g, W): a notch of width g (in X) is cut inward from y=+6 down to
y=+W/2 (the "top" slit), and a collinear notch of the same width g is cut
inward from y=-6 up to y=-W/2 (the "bottom" slit) -- so the two slit tips
are separated by exactly W mm of material, centred on y=0.

Stdlib only. Reuses write_binary_stl / check_watertight / sha256_of /
parse_binary_stl / tri_normal / polygon_area_2d from tools/make_fixtures.py
rather than reimplementing them.

Usage:
    python tools/m22_neck_coupon.py [--out <path>]
"""

import argparse
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

# Block extents (mm).
X0, X1 = 0.0, 90.0
Y0, Y1 = -6.0, 6.0
Z0, Z1 = 0.0, 20.0

# Eight stations: (x_center, slit_width_g, ligament_W).
STATIONS = [
    (10.0, 0.01, 0.42),
    (20.0, 0.01, 0.60),
    (30.0, 0.01, 0.84),
    (40.0, 0.01, 1.00),
    (50.0, 0.42, 0.42),
    (60.0, 0.42, 0.60),
    (70.0, 0.42, 0.84),
    (80.0, 0.42, 1.00),
]


def build_cross_section_polygon():
    """Build the closed 2D cross-section polygon (list of (x, y) points, no
    repeated first/last point -- the closing edge from the last point back
    to the first is implicit, matching the convention used throughout
    make_fixtures.py).

    Boundary walk: (0,-6) -> along the bottom edge in +X, detouring UP into
    each bottom notch and back down -> (90,-6) -> (90,+6) -> along the top
    edge in -X, detouring DOWN into each top notch and back up -> (0,+6) ->
    close back to (0,-6).
    """
    bottom = [(X0, Y0)]
    for x_c, g, w in STATIONS:
        x_lo, x_hi = x_c - g / 2.0, x_c + g / 2.0
        y_tip = -w / 2.0
        bottom.append((x_lo, Y0))
        bottom.append((x_lo, y_tip))
        bottom.append((x_hi, y_tip))
        bottom.append((x_hi, Y0))
    bottom.append((X1, Y0))

    top = [(X1, Y1)]
    for x_c, g, w in reversed(STATIONS):
        x_lo, x_hi = x_c - g / 2.0, x_c + g / 2.0
        y_tip = w / 2.0
        top.append((x_hi, Y1))
        top.append((x_hi, y_tip))
        top.append((x_lo, y_tip))
        top.append((x_lo, Y1))
    top.append((X0, Y1))

    return bottom + top


def _cross2(o, a, b):
    """2D cross product of (a - o) and (b - o); sign gives turn direction."""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _is_convex_vertex(a, b, c):
    """True if the interior angle at b (polygon walked a -> b -> c, CCW
    winding) is convex, i.e. a strict left turn.
    """
    ux, uy = b[0] - a[0], b[1] - a[1]
    vx, vy = c[0] - b[0], c[1] - b[1]
    return (ux * vy - uy * vx) > 0.0


def _point_in_or_on_triangle(p, a, b, c):
    d1 = _cross2(a, b, p)
    d2 = _cross2(b, c, p)
    d3 = _cross2(c, a, p)
    has_neg = d1 < 0.0 or d2 < 0.0 or d3 < 0.0
    has_pos = d1 > 0.0 or d2 > 0.0 or d3 > 0.0
    return not (has_neg and has_pos)


def ear_clip(polygon):
    """Ear-clipping triangulation of a simple CCW polygon (no holes, no
    self-intersection). Returns a list of (i, j, k) index triples into
    polygon, each triangle wound CCW (same orientation as the input
    polygon).
    """
    n = len(polygon)
    if n < 3:
        raise ValueError(f"polygon needs >= 3 vertices, got {n}")
    if polygon_area_2d(polygon) <= 0.0:
        raise ValueError("ear_clip requires a CCW (positive-area) polygon")

    idx = list(range(n))
    triangles = []
    guard_limit = n * n + 10

    guard = 0
    while len(idx) > 3:
        guard += 1
        if guard > guard_limit:
            raise RuntimeError(
                f"ear clipping did not converge after {guard} iterations "
                f"({len(idx)} vertices remaining)"
            )
        m = len(idx)
        found = False
        for i in range(m):
            i_prev = idx[(i - 1) % m]
            i_curr = idx[i]
            i_next = idx[(i + 1) % m]
            a, b, c = polygon[i_prev], polygon[i_curr], polygon[i_next]
            if not _is_convex_vertex(a, b, c):
                continue
            blocked = False
            for j in idx:
                if j in (i_prev, i_curr, i_next):
                    continue
                if _point_in_or_on_triangle(polygon[j], a, b, c):
                    blocked = True
                    break
            if blocked:
                continue
            triangles.append((i_prev, i_curr, i_next))
            del idx[i]
            found = True
            break
        if not found:
            raise RuntimeError(
                f"no ear found with {m} vertices remaining; polygon may be "
                "invalid (self-intersecting or CW)"
            )
    triangles.append((idx[0], idx[1], idx[2]))
    return triangles


def build_triangles(polygon):
    """Build the watertight mesh: two triangulated Z caps plus extruded
    side walls, for the given CCW 2D cross-section polygon extruded from
    Z0 to Z1.
    """
    n = len(polygon)
    cap_tris_idx = ear_clip(polygon)

    bottom = [(x, y, Z0) for x, y in polygon]
    top = [(x, y, Z1) for x, y in polygon]

    triangles = []

    # Bottom cap: normal -Z, so reverse the CCW (as seen from +Z) winding
    # from ear_clip into CW (as seen from +Z) / CCW as seen from -Z.
    for i, j, k in cap_tris_idx:
        v0, v1, v2 = bottom[i], bottom[k], bottom[j]
        normal = tri_normal(v0, v1, v2)
        triangles.append((normal, v0, v1, v2))

    # Top cap: normal +Z, same winding as ear_clip produced.
    for i, j, k in cap_tris_idx:
        v0, v1, v2 = top[i], top[j], top[k]
        normal = tri_normal(v0, v1, v2)
        triangles.append((normal, v0, v1, v2))

    # Side walls: one quad (2 triangles) per boundary edge. Outward normal
    # is the CCW outward-normal rotation of the edge vector (dy, -dx),
    # matching the convention used in make_fixtures.airfoil_triangles for a
    # CCW polygon.
    for i in range(n):
        j = (i + 1) % n
        a, b = bottom[i], bottom[j]
        c, d = top[j], top[i]
        n1 = tri_normal(a, b, c)
        n2 = tri_normal(a, c, d)
        triangles.append((n1, a, b, c))
        triangles.append((n2, a, c, d))

    return triangles


def expected_cross_section_area():
    """Analytic cross-section area: full 90x12 rectangle minus the area
    removed by the two collinear notches (top + bottom) at each station.
    """
    area = (X1 - X0) * (Y1 - Y0)
    for _x_c, g, w in STATIONS:
        area -= g * (Y1 - w / 2.0) * 2.0
    return area


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output STL path (default: tests/fixtures/coupon/m22_neck_coupon.stl)",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    out_path = args.out if args.out is not None else (
        repo_root / "tests" / "fixtures" / "coupon" / "m22_neck_coupon.stl"
    )

    polygon = build_cross_section_polygon()
    print(f"cross-section polygon vertices: {len(polygon)}")

    triangles = build_triangles(polygon)
    print(f"triangles: {len(triangles)}")

    # Self-check 1: watertight.
    ok, details, problems = check_watertight(triangles)
    print(f"[check 1] watertight: {ok} -- {details}")
    if not ok:
        for p in problems[:20]:
            print(f"    {p}")
        raise SystemExit("FAIL: mesh is not watertight")

    # Self-check 2: signed area matches the analytic expectation.
    area = polygon_area_2d(polygon)
    expected_area = expected_cross_section_area()
    print(
        f"[check 2] cross-section signed area: {area:.9f} mm^2 "
        f"(expected {expected_area:.9f} mm^2, CCW positive)"
    )
    assert area > 0.0, "cross-section polygon must be CCW (positive area)"
    assert abs(area - expected_area) < 1e-9, (
        f"area mismatch: {area!r} vs expected {expected_area!r}"
    )

    write_binary_stl(out_path, triangles)

    # Self-check 3: round-trip triangle count.
    _header, reread_triangles = parse_binary_stl(out_path)
    print(
        f"[check 3] round-trip triangle count: wrote {len(triangles)}, "
        f"read back {len(reread_triangles)}"
    )
    assert len(reread_triangles) == len(triangles), "triangle count did not round-trip"

    # Self-check 4: SHA-256 of the written file.
    digest = sha256_of(out_path)
    print(f"[check 4] SHA-256: {digest}")
    print(f"wrote: {out_path}")


if __name__ == "__main__":
    main()
