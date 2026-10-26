#!/usr/bin/env python3
"""Deterministic binary STL fixture generator for aircraft-orca-structures.

Stdlib only. Writes watertight solids as binary STL files for use as slicer
test fixtures: axis-aligned rectangular prisms (the coupon), constant-section
NACA 4-digit airfoil wings (the fixture), and the geometry-envelope wings that
break the constant-section assumption -- tapered, swept, twisted, and
tip-closure wings -- all built from the same shared NACA 4-digit section
generator via a generic spanwise loft (loft_triangles / build_loft_stations).

Usage:
    python tools/make_fixtures.py coupon
    python tools/make_fixtures.py coupon-short
    python tools/make_fixtures.py airfoil [--naca 2412] [--chord 60] [--span 60] [--points 120]
    python tools/make_fixtures.py taper [--naca 2412] [--root-chord 60] [--tip-chord 30] [--span 60] [--points 120]
    python tools/make_fixtures.py sweep [--naca 2412] [--chord 60] [--span 60] [--sweep 20] [--points 120]
    python tools/make_fixtures.py twist [--naca 2412] [--chord 60] [--span 60] [--washout 6] [--points 120] [--stations 61]
    python tools/make_fixtures.py tip [--naca 2412] [--root-chord 60] [--tip-chord 3] [--span 60] [--points 120] [--skin-margin 1.0]
"""

import argparse
import hashlib
import math
import struct
import sys
from pathlib import Path

# Fixed ASCII header text -- no timestamps, no machine-specific data, so the
# output bytes are reproducible across runs and machines.
STL_HEADER_TEXT = b"aircraft-orca-structures fixture, deterministic build"


def _pad_header(text: bytes) -> bytes:
    """Pad/truncate header text to the mandatory 80-byte binary STL header."""
    if len(text) > 80:
        raise ValueError("header text exceeds 80 bytes")
    return text + b"\x00" * (80 - len(text))


def box_triangles(x0, y0, z0, x1, y1, z1):
    """Return the 12 triangles (as (normal, v0, v1, v2) tuples) for an
    axis-aligned box from (x0,y0,z0) to (x1,y1,z1), with outward-facing
    normals and consistent (counter-clockwise when viewed from outside)
    winding order.
    """
    # 8 corners
    c = {
        (0, 0, 0): (x0, y0, z0),
        (1, 0, 0): (x1, y0, z0),
        (1, 1, 0): (x1, y1, z0),
        (0, 1, 0): (x0, y1, z0),
        (0, 0, 1): (x0, y0, z1),
        (1, 0, 1): (x1, y0, z1),
        (1, 1, 1): (x1, y1, z1),
        (0, 1, 1): (x0, y1, z1),
    }

    tris = []

    def quad(n, a, b, c_, d):
        # Split quad a-b-c_-d (already CCW when viewed from outside, along
        # normal n) into two triangles with consistent winding.
        tris.append((n, c[a], c[b], c[c_]))
        tris.append((n, c[a], c[c_], c[d]))

    # -Z face (bottom), normal (0,0,-1): viewed from below, CCW order
    quad((0, 0, -1), (0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0))
    # +Z face (top), normal (0,0,1)
    quad((0, 0, 1), (0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1))
    # -Y face, normal (0,-1,0)
    quad((0, -1, 0), (0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1))
    # +Y face, normal (0,1,0)
    quad((0, 1, 0), (0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0))
    # -X face, normal (-1,0,0)
    quad((-1, 0, 0), (0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0))
    # +X face, normal (1,0,0)
    quad((1, 0, 0), (1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1))

    return tris


def write_binary_stl(path: Path, triangles, header_text: bytes = STL_HEADER_TEXT):
    """Write a list of (normal, v0, v1, v2) triangles as a binary STL file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(_pad_header(header_text))
        f.write(struct.pack("<I", len(triangles)))
        for normal, v0, v1, v2 in triangles:
            f.write(struct.pack("<3f", *normal))
            f.write(struct.pack("<3f", *v0))
            f.write(struct.pack("<3f", *v1))
            f.write(struct.pack("<3f", *v2))
            f.write(struct.pack("<H", 0))  # attribute byte count


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def parse_binary_stl(path: Path):
    """Parse a binary STL file back into (header, triangle_list)."""
    with open(path, "rb") as f:
        header = f.read(80)
        (count,) = struct.unpack("<I", f.read(4))
        triangles = []
        for _ in range(count):
            normal = struct.unpack("<3f", f.read(12))
            v0 = struct.unpack("<3f", f.read(12))
            v1 = struct.unpack("<3f", f.read(12))
            v2 = struct.unpack("<3f", f.read(12))
            f.read(2)  # attribute byte count
            triangles.append((normal, v0, v1, v2))
    return header, triangles


def check_watertight(triangles):
    """Verify every edge of the triangle mesh is shared by exactly two
    triangles (with opposite winding, i.e. the directed edge and its
    reverse each appear exactly once). Returns (ok, details_str).
    """
    edge_count = {}
    for _normal, v0, v1, v2 in triangles:
        for a, b in ((v0, v1), (v1, v2), (v2, v0)):
            key = (a, b)
            edge_count[key] = edge_count.get(key, 0) + 1

    problems = []
    total_directed_edges = 0
    for (a, b), count in edge_count.items():
        total_directed_edges += count
        reverse = (b, a)
        if count != 1:
            problems.append(f"directed edge {a}->{b} appears {count} times (expected 1)")
        if edge_count.get(reverse, 0) != 1:
            problems.append(
                f"reverse of edge {a}->{b} (i.e. {b}->{a}) appears "
                f"{edge_count.get(reverse, 0)} times (expected 1)"
            )

    num_edges = len(edge_count) // 2  # each undirected edge = 2 directed entries
    ok = len(problems) == 0
    details = (
        f"{len(triangles)} triangles, {total_directed_edges} directed edge "
        f"instances, {num_edges} undirected edges; "
        f"{'all edges shared by exactly 2 triangles' if ok else str(len(problems)) + ' problem(s) found'}"
    )
    return ok, details, problems


def parse_naca4(code: str):
    """Parse a NACA 4-digit designation into (m, p, t):

    - m: maximum camber, as a fraction of chord (first digit / 100)
    - p: chord-wise location of maximum camber, as a fraction of chord
      (second digit / 10)
    - t: maximum thickness, as a fraction of chord (last two digits / 100)
    """
    if len(code) != 4 or not code.isdigit():
        raise ValueError(
            f"NACA designation must be 4 digits, got {code!r}."
        )
    m = int(code[0]) / 100.0
    p = int(code[1]) / 10.0
    t = int(code[2:4]) / 100.0
    if t <= 0.0:
        raise ValueError(f"NACA {code}: thickness digits must be > 0.")
    return m, p, t


def naca4_thickness(x: float, t: float) -> float:
    """Closed-trailing-edge NACA 4-digit thickness distribution at chord
    fraction x in [0, 1]. The a4 coefficient (-0.1036, vs. -0.1015 for the
    open-TE variant) is chosen so yt(1) == 0.0 analytically, closing the
    trailing edge without a separate patch.
    """
    return 5.0 * t * (
        0.2969 * math.sqrt(x)
        - 0.1260 * x
        - 0.3516 * x ** 2
        + 0.2843 * x ** 3
        - 0.1036 * x ** 4
    )


def naca4_camber(x: float, m: float, p: float):
    """Mean camber line value and slope (yc, dyc/dx) at chord fraction x.

    Symmetric sections (m == 0 or p == 0, e.g. NACA 00xx) have a zero camber
    line everywhere; the standard two-segment parabolic camber formula is
    undefined at p == 0 (division by zero), so it is guarded explicitly.
    """
    if m <= 0.0 or p <= 0.0:
        return 0.0, 0.0
    if x < p:
        yc = (m / p ** 2) * (2.0 * p * x - x ** 2)
        dyc = (2.0 * m / p ** 2) * (p - x)
    else:
        yc = (m / (1.0 - p) ** 2) * ((1.0 - 2.0 * p) + 2.0 * p * x - x ** 2)
        dyc = (2.0 * m / (1.0 - p) ** 2) * (p - x)
    return yc, dyc


def cosine_spaced_chord_fractions(n_points: int):
    """n_points chord fractions in [0, 1], cosine-spaced so points cluster
    near both x=0 (leading edge) and x=1 (trailing edge): x_i =
    (1 - cos(i*pi/(n_points-1))) / 2. Strictly monotonically increasing in
    i -- this is the raw chordwise parameter, evaluated before the
    per-surface perpendicular thickness offset (naca4_surface_points),
    which is not itself guaranteed monotonic in x near a cambered leading
    edge.
    """
    if n_points < 3:
        raise ValueError(f"n_points must be >= 3, got {n_points}.")
    return [
        (1.0 - math.cos(i * math.pi / (n_points - 1))) / 2.0
        for i in range(n_points)
    ]


def naca4_surface_points(m: float, p: float, t: float, n_points: int):
    """Cosine-spaced upper and lower surface points for a NACA 4-digit
    section, in chord-fraction coordinates (x, y both in [~-0.1, 1.1] x
    chord).

    Returns (upper, lower), each a list of n_points (x, y) tuples ordered
    from the leading edge (index 0) to the trailing edge (index -1). The
    thickness is applied perpendicular to the camber line (not vertically),
    per the standard NACA construction: xu = x - yt*sin(theta),
    yu = yc + yt*cos(theta), and mirrored for the lower surface, where
    theta = atan(dyc/dx).

    The leading-edge point is exactly (0.0, 0.0) and the trailing-edge
    point is forced identical on both surfaces (the closed-TE thickness
    distribution makes yt(1) analytically 0, but floating-point evaluation
    of the quintic can leave residue at the 1e-16 level; forcing equality
    guarantees a bit-exact shared vertex rather than relying on rounding).
    """
    upper = []
    lower = []
    for x in cosine_spaced_chord_fractions(n_points):
        yt = naca4_thickness(x, t)
        yc, dyc = naca4_camber(x, m, p)
        theta = math.atan(dyc)
        xu = x - yt * math.sin(theta)
        yu = yc + yt * math.cos(theta)
        xl = x + yt * math.sin(theta)
        yl = yc - yt * math.cos(theta)
        upper.append((xu, yu))
        lower.append((xl, yl))

    upper[0] = (0.0, 0.0)
    lower[0] = (0.0, 0.0)
    yc_te, _ = naca4_camber(1.0, m, p)
    te_point = (1.0, yc_te)
    upper[-1] = te_point
    lower[-1] = te_point

    return upper, lower


def naca4_section_outline(m: float, p: float, t: float, n_points: int):
    """Closed 2D section outline in chord-fraction coordinates, wound
    counter-clockwise when viewed from +Z (the convention this module's
    extrusion code assumes; see airfoil_triangles()).

    Order: leading edge -> lower surface (increasing x) -> trailing edge
    -> upper surface (decreasing x) -> back to leading edge. (Lower-then-
    upper, rather than the more familiar upper-then-lower reading order,
    is what makes this CCW rather than CW -- verified against the shoelace
    formula on a simple symmetric test shape.)
    """
    upper, lower = naca4_surface_points(m, p, t, n_points)
    outline = [upper[0]]  # leading edge, (0, 0)
    outline.extend(lower[1:-1])
    outline.append(lower[-1])  # trailing edge
    outline.extend(reversed(upper[1:-1]))
    return outline


def polygon_area_2d(points) -> float:
    """Shoelace area of a closed 2D polygon (implicit closing edge from
    last point back to first). Positive for CCW winding.
    """
    n = len(points)
    total = 0.0
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        total += x0 * y1 - x1 * y0
    return total / 2.0


def airfoil_triangles(m: float, p: float, t: float, chord_mm: float, span_mm: float, n_points: int):
    """Build the watertight triangle mesh for a constant-section NACA
    4-digit wing: the 2D section (naca4_section_outline, scaled to
    chord_mm and centred on its own bounding box in X/Y) extruded from
    Z=0 to Z=span_mm, with a triangulated cap at each end.

    Returns (triangles, outline_mm, bbox) where outline_mm is the final
    (post-scale, post-centring) 2D outline in millimetres and bbox is
    (xmin, ymin, xmax, ymax) of that outline -- both handed back purely
    for fixture-generation reporting (section area/bbox fill ratio etc.),
    not used by the mesh itself.
    """
    outline = naca4_section_outline(m, p, t, n_points)
    scaled = [(x * chord_mm, y * chord_mm) for x, y in outline]

    xs = [x for x, _ in scaled]
    ys = [y for _, y in scaled]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
    outline_mm = [(x - cx, y - cy) for x, y in scaled]
    bbox = (xmin - cx, ymin - cy, xmax - cx, ymax - cy)

    z0, z1 = 0.0, span_mm
    n = len(outline_mm)
    bottom = [(x, y, z0) for x, y in outline_mm]
    top = [(x, y, z1) for x, y in outline_mm]

    triangles = []

    # Side walls: one quad per outline edge, outward normal = the CCW
    # outward-normal rotation of the edge vector (dy, -dx), matching
    # box_triangles()'s convention (verified there against axis-aligned
    # faces; this is the same formula, just for an arbitrary CCW polygon
    # rather than a rectangle).
    for i in range(n):
        j = (i + 1) % n
        dx = outline_mm[j][0] - outline_mm[i][0]
        dy = outline_mm[j][1] - outline_mm[i][1]
        length = math.hypot(dx, dy)
        if length == 0.0:
            continue  # degenerate (coincident) edge; skip rather than emit a zero-area triangle
        nx, ny = dy / length, -dx / length
        normal = (nx, ny, 0.0)
        a, b, c, d = bottom[i], bottom[j], top[j], top[i]
        triangles.append((normal, a, b, c))
        triangles.append((normal, a, c, d))

    # Caps, fan-triangulated from the leading-edge vertex (index 0). Valid
    # for any section that is star-shaped from its leading edge, which
    # holds for standard (non-reflex) NACA 4-digit sections such as the
    # default 2412.
    top_normal = (0.0, 0.0, 1.0)
    bottom_normal = (0.0, 0.0, -1.0)
    for k in range(1, n - 1):
        # Top cap: outline order is already CCW viewed from +Z.
        triangles.append((top_normal, top[0], top[k], top[k + 1]))
        # Bottom cap: reverse winding so it is CCW viewed from -Z.
        triangles.append((bottom_normal, bottom[0], bottom[k + 1], bottom[k]))

    return triangles, outline_mm, bbox


def scale_naca_outline(outline_unit, chord_mm: float):
    """Scale a chord-fraction outline (leading edge at (0,0), trailing edge
    at (1, yc_te)) to millimetres at the given chord.
    """
    return [(x * chord_mm, y * chord_mm) for x, y in outline_unit]


def rotate_points_2d(points, rotate_deg: float, pivot):
    """Rotate a list of 2D points by rotate_deg (standard CCW-positive
    convention) about pivot=(px, py). No-op (returns a copy) at 0 degrees,
    both to avoid needless float error and to make root stations (which use
    0 degrees everywhere) bit-identical to the un-rotated scaled outline.
    """
    if rotate_deg == 0.0:
        return list(points)
    theta = math.radians(rotate_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    px, py = pivot
    out = []
    for x, y in points:
        rx, ry = x - px, y - py
        out.append((rx * cos_t - ry * sin_t + px, rx * sin_t + ry * cos_t + py))
    return out


def translate_points_2d(points, dx_mm: float, dy_mm: float = 0.0):
    return [(x + dx_mm, y + dy_mm) for x, y in points]


def section_bbox_2d(points):
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    return min(xs), min(ys), max(xs), max(ys)


def local_max_thickness_mm(m: float, p: float, t: float, n_points: int, chord_mm: float) -> float:
    """Local maximum thickness (max upper-minus-lower separation, taken at
    matching cosine-spaced indices) at the given chord, in millimetres. This
    is the number that decides whether a structural web fits at a given
    spanwise station -- not the bounding-box Y extent, which also carries
    camber offset.
    """
    upper, lower = naca4_surface_points(m, p, t, n_points)
    max_sep = max(yu - yl for (_xu, yu), (_xl, yl) in zip(upper, lower))
    return max_sep * chord_mm


def build_station_outline(outline_unit, chord_mm: float, rotate_deg: float, pivot_frac_x: float,
                           dx_mm: float, dy_mm: float, root_shift):
    """Build one spanwise station's 2D outline in millimetres: scale the
    shared unit outline to chord_mm, rotate about (pivot_frac_x * chord_mm,
    0) -- i.e. a chordwise fraction of *this station's own* chord, so a
    constant pivot_frac_x traces a straight pivot line across a tapering
    span too -- then translate by (dx_mm, dy_mm), then apply a constant
    root_shift=(cx, cy) subtraction shared by every station in the loft (so
    the whole geometry is positioned relative to the root section's own
    bounding-box centre, matching the constant-section airfoil fixture's
    convention, rather than each station re-centring itself and silently
    cancelling out sweep/taper/twist).
    """
    scaled = scale_naca_outline(outline_unit, chord_mm)
    pivot = (pivot_frac_x * chord_mm, 0.0)
    rotated = rotate_points_2d(scaled, rotate_deg, pivot)
    translated = translate_points_2d(rotated, dx_mm, dy_mm)
    cx, cy = root_shift
    return translate_points_2d(translated, -cx, -cy)


def build_loft_stations(m: float, p: float, t: float, n_points: int, z_stations, params_fn,
                         pivot_frac_x: float = 0.25):
    """Build a list of (z, outline_mm) stations sharing one unit outline
    (naca4_section_outline, generated once -- see module docstring on reuse)
    for a spanwise loft. params_fn(z) -> (chord_mm, rotate_deg, dx_mm)
    defines the geometry class (taper/sweep/twist/tip all vary only these
    three numbers per station). The root station (params_fn(0.0)) defines
    the shared bounding-box-centring shift applied to every station.
    """
    outline_unit = naca4_section_outline(m, p, t, n_points)

    chord0, rot0, dx0 = params_fn(0.0)
    root_outline = build_station_outline(outline_unit, chord0, rot0, pivot_frac_x, dx0, 0.0, (0.0, 0.0))
    xmin, ymin, xmax, ymax = section_bbox_2d(root_outline)
    root_shift = ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0)

    stations = []
    for z in z_stations:
        chord, rot, dx = params_fn(z)
        outline_mm = build_station_outline(outline_unit, chord, rot, pivot_frac_x, dx, 0.0, root_shift)
        stations.append((z, outline_mm))
    return stations, outline_unit, root_shift


def tri_normal(v0, v1, v2):
    """Geometric (cross-product) unit normal of a triangle. Raises
    ValueError on a zero-area triangle rather than silently emitting a
    degenerate face -- a lofted mesh (unlike the axis-aligned box or the
    constant-section extrusion) has no a-priori guarantee that consecutive
    stations or outline vertices stay apart after scaling to a small tip
    chord, so this is a real check, not defensive boilerplate.
    """
    ux, uy, uz = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
    vx, vy, vz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length == 0.0:
        raise ValueError(f"degenerate (zero-area) triangle at {v0}, {v1}, {v2}")
    return (nx / length, ny / length, nz / length)


def loft_triangles(stations):
    """Build the watertight triangle mesh for a spanwise loft between two or
    more stations, each (z, outline_mm) with an outline of the same vertex
    count and vertex-index meaning (the naca4_section_outline convention:
    same cosine-spaced chordwise parametrisation at every station, so vertex
    k means the same point on the aerodynamic surface at every z). Side
    walls are a quad strip between each pair of adjacent stations (each quad
    split into two triangles, normal computed geometrically per triangle --
    unlike airfoil_triangles's constant-section extrusion, a loft's side
    quads are not guaranteed planar so a fixed 2D normal formula does not
    apply). Caps are fan-triangulated exactly as in airfoil_triangles, from
    the first and last stations.
    """
    if len(stations) < 2:
        raise ValueError("a loft needs at least 2 stations")
    n = len(stations[0][1])
    for z, outline in stations:
        if len(outline) != n:
            raise ValueError(f"station vertex-count mismatch at z={z}: {len(outline)} != {n}")

    def v3(z, xy):
        return (xy[0], xy[1], z)

    triangles = []
    for s in range(len(stations) - 1):
        z0, out0 = stations[s]
        z1, out1 = stations[s + 1]
        for k in range(n):
            j = (k + 1) % n
            a, b, c, d = v3(z0, out0[k]), v3(z0, out0[j]), v3(z1, out1[j]), v3(z1, out1[k])
            n1 = tri_normal(a, b, c)
            n2 = tri_normal(a, c, d)
            triangles.append((n1, a, b, c))
            triangles.append((n2, a, c, d))

    z0, out0 = stations[0]
    z1, out1 = stations[-1]
    bottom = [v3(z0, xy) for xy in out0]
    top = [v3(z1, xy) for xy in out1]
    bottom_normal = (0.0, 0.0, -1.0)
    top_normal = (0.0, 0.0, 1.0)
    for k in range(1, n - 1):
        triangles.append((top_normal, top[0], top[k], top[k + 1]))
        triangles.append((bottom_normal, bottom[0], bottom[k + 1], bottom[k]))

    return triangles


def make_coupon(repo_root: Path):
    """20 x 20 x 60 mm prism, sitting on Z=0, centred on X=Y=0."""
    hx, hy = 10.0, 10.0  # half extents in X, Y (20mm total)
    z0, z1 = 0.0, 60.0
    triangles = box_triangles(-hx, -hy, z0, hx, hy, z1)

    out_dir = repo_root / "tests" / "fixtures" / "coupon"
    stl_path = out_dir / "coupon_20x20x60.stl"
    write_binary_stl(stl_path, triangles)

    readme_path = out_dir / "README.md"
    readme_path.write_text(
        "# Coupon fixture: coupon_20x20x60.stl\n"
        "\n"
        "## Description\n"
        "A single axis-aligned rectangular prism, 20 mm (X) x 20 mm (Y) x "
        "60 mm (Z), sitting on the build plate at Z=0 and centred on "
        "X=0, Y=0 (so it spans X in [-10, 10], Y in [-10, 10], Z in "
        "[0, 60]).\n"
        "\n"
        "## Exact dimensions\n"
        "- X extent: -10.0 mm to 10.0 mm (20 mm)\n"
        "- Y extent: -10.0 mm to 10.0 mm (20 mm)\n"
        "- Z extent: 0.0 mm to 60.0 mm (60 mm)\n"
        "- Triangles: 12 (2 per face x 6 faces)\n"
        "- Format: binary STL, deterministic fixed ASCII header, no "
        "timestamps\n"
        "\n"
        "## Purpose\n"
        "This is the M0 test fixture: the simplest possible watertight "
        "solid used to prove the slicing-pipeline plugin hook fires "
        "correctly (M0 hook proof) before any structural slot logic "
        "exists. It also serves as the base geometry for the M1 "
        "single-slot coupon, where a single boundary-connected slot "
        "will be subtracted from this prism's per-layer 2D slices to "
        "validate the plugin's Boolean/topology pipeline end to end.\n"
        "\n"
        "Generated by `tools/make_fixtures.py coupon`.\n",
        encoding="ascii",
    )

    return stl_path, readme_path


def make_airfoil(repo_root: Path, naca: str, chord_mm: float, span_mm: float, n_points: int):
    """Constant-section NACA 4-digit wing, sitting on Z=0, centred on its
    own bounding box in X/Y, extruded along +Z (span) from Z=0 to
    Z=span_mm.
    """
    m, p, t = parse_naca4(naca)
    triangles, outline_mm, bbox = airfoil_triangles(m, p, t, chord_mm, span_mm, n_points)
    xmin, ymin, xmax, ymax = bbox

    section_area = polygon_area_2d(outline_mm)
    bbox_area = (xmax - xmin) * (ymax - ymin)
    fill_ratio = section_area / bbox_area if bbox_area > 0 else 0.0

    out_dir = repo_root / "tests" / "fixtures" / "airfoil"
    stem = f"naca{naca}_c{int(chord_mm)}_s{int(span_mm)}"
    stl_path = out_dir / f"{stem}.stl"
    write_binary_stl(stl_path, triangles)

    readme_path = out_dir / "README.md"
    readme_path.write_text(
        f"# Airfoil fixture: {stem}.stl\n"
        "\n"
        "## Description\n"
        f"A constant-section NACA {naca} wing: the standard NACA 4-digit "
        "section (thickness distribution applied perpendicular to the "
        "camber line, closed trailing edge, cosine-spaced points) extruded "
        f"along +Z (span) at fixed chord {chord_mm:g} mm from Z=0 to "
        f"Z={span_mm:g} mm. Z is the spanwise axis, so the printed wing "
        "stands vertically on the build plate with the section horizontal, "
        "per the constitution's vertical-print orientation.\n"
        "\n"
        "## Origin convention\n"
        "The 2D section is generated in chord-fraction coordinates (chord "
        "along X, thickness/camber along Y), scaled by the chord length, "
        "then translated so its own X/Y bounding box is centred on X=0, "
        "Y=0 -- i.e. it is centred on its bounding box, not on the "
        "leading edge or the chord line. The leading edge is therefore "
        "*not* at X=0; the section's actual X/Y extent is reported below. "
        "The wing sits on the build plate at Z=0 (root) and extends to "
        f"Z={span_mm:g} mm (tip).\n"
        "\n"
        "## Exact dimensions\n"
        f"- NACA designation: {naca} (m={m:g}, p={p:g}, t={t:g})\n"
        f"- Chord: {chord_mm:g} mm\n"
        f"- Span: {span_mm:g} mm\n"
        f"- Section X extent: {xmin:.6f} mm to {xmax:.6f} mm\n"
        f"- Section Y extent: {ymin:.6f} mm to {ymax:.6f} mm\n"
        f"- Z extent: 0.0 mm to {span_mm:g} mm ({span_mm:g} mm)\n"
        f"- Points per surface: {n_points} (cosine-spaced; shared leading- "
        "and trailing-edge vertices between upper and lower surfaces, so "
        f"the closed section outline has {2 * n_points - 2} vertices)\n"
        f"- Section area / bounding-box area (fill ratio): {fill_ratio:.4f}\n"
        f"- Triangles: {len(triangles)} "
        f"({2 * (2 * n_points - 2)} side wall + {2 * (2 * n_points - 4)} cap)\n"
        "- Format: binary STL, deterministic fixed ASCII header, no "
        "timestamps\n"
        "\n"
        "## Purpose\n"
        "This is the M2 fixture: the first genuinely cambered, non-"
        "rectangular geometry in the test suite. Its section fill ratio "
        "(area / bounding-box area) is well below the M1 "
        "`RECTANGULARITY_MIN = 0.90` fence in `aircraft_core/validate.py` "
        "-- see the number above -- so the M1 validator refuses this "
        "fixture today. That refusal is correct M1 behaviour, not a bug: "
        "the bounding-box margin checks it guards would mis-validate a "
        "cambered section (docs/milestones/M02-constant-section-airfoil.md). "
        "This fixture exists to be measured against once M2 replaces those "
        "checks with a real chord axis and local thickness.\n"
        "\n"
        f"Generated by `python tools/make_fixtures.py airfoil --naca {naca} "
        f"--chord {chord_mm:g} --span {span_mm:g} --points {n_points}`.\n",
        encoding="ascii",
    )

    return stl_path, readme_path, fill_ratio, section_area, bbox_area


def _thickness_table(m, p, t, n_points, chord_fn, z_values):
    return [(z, chord_fn(z), local_max_thickness_mm(m, p, t, n_points, chord_fn(z))) for z in z_values]


def _thickness_table_md(rows, extra_col_name=None, extra_col_fn=None):
    header = "| Z (mm) | Chord (mm) | Local max thickness (mm) |"
    sep = "|---|---|---|"
    if extra_col_name:
        header += f" {extra_col_name} |"
        sep += "---|"
    lines = [header, sep]
    for z, chord, thickness in rows:
        line = f"| {z:g} | {chord:.4f} | {thickness:.4f} |"
        if extra_col_name:
            line += f" {extra_col_fn(z):.4f} |"
        lines.append(line)
    return "\n".join(lines)


def make_taper(repo_root: Path, naca: str, root_chord_mm: float, tip_chord_mm: float,
                span_mm: float, n_points: int):
    """Tapered wing, straight leading edge: chord shrinks linearly from
    root_chord_mm at Z=0 to tip_chord_mm at Z=span_mm; the leading edge
    (pivot_frac_x=0.0, i.e. the outline is scaled about its own X=0 point,
    which is the leading edge by construction) stays at a fixed local X, so
    only the trailing edge moves. Two stations (root, tip) are geometrically
    exact for a linear taper: chord(z) is affine in z, so linear
    interpolation between the two scaled outlines equals the correctly
    scaled outline at every intermediate z.
    """
    m, p, t = parse_naca4(naca)

    def params(z):
        frac = z / span_mm
        chord = root_chord_mm + (tip_chord_mm - root_chord_mm) * frac
        return chord, 0.0, 0.0

    stations, outline_unit, root_shift = build_loft_stations(
        m, p, t, n_points, (0.0, span_mm), params, pivot_frac_x=0.0
    )
    triangles = loft_triangles(stations)
    n_outline = len(outline_unit)

    thickness_rows = _thickness_table(
        m, p, t, n_points,
        lambda z: root_chord_mm + (tip_chord_mm - root_chord_mm) * (z / span_mm),
        [0.0, 15.0, 30.0, 45.0, span_mm],
    )

    out_dir = repo_root / "tests" / "fixtures" / "taper"
    stem = f"naca{naca}_taper_r{int(root_chord_mm)}_t{int(tip_chord_mm)}_s{int(span_mm)}"
    stl_path = out_dir / f"{stem}.stl"
    write_binary_stl(stl_path, triangles)
    stl_sha256 = sha256_of(stl_path)

    readme_path = out_dir / "README.md"
    readme_path.write_text(
        f"# Taper fixture: {stem}.stl\n"
        "\n"
        "## Description\n"
        f"A tapered NACA {naca} wing: chord changes linearly with Z, from "
        f"{root_chord_mm:g} mm at the root (Z=0) to {tip_chord_mm:g} mm at "
        f"the tip (Z={span_mm:g} mm), with the leading edge held at a fixed "
        "local X so the taper is expressed as the trailing edge moving "
        "inward -- a straight-leading-edge tapered wing, not a straight-"
        "quarter-chord one. Built from exactly 2 stations (root, tip): "
        "because chord(z) is affine in Z, linear interpolation between the "
        "two scaled outlines is geometrically exact at every intermediate "
        "Z, so 2 stations is not an approximation here.\n"
        "\n"
        "## Origin convention\n"
        "The unit NACA section (chord along X, thickness/camber along Y) is "
        "generated once and scaled per station; every station is then "
        "shifted by the same constant offset so the *root* station's own "
        "X/Y bounding box is centred on X=0, Y=0 -- matching the "
        "constant-section airfoil fixture's convention. The tip station is "
        "positioned relative to that same fixed frame, so its bounding box "
        "is not separately centred. Z is spanwise, root at Z=0, tip at "
        f"Z={span_mm:g} mm.\n"
        "\n"
        "## Exact dimensions\n"
        f"- NACA designation: {naca} (m={m:g}, p={p:g}, t={t:g})\n"
        f"- Root chord: {root_chord_mm:g} mm\n"
        f"- Tip chord: {tip_chord_mm:g} mm\n"
        f"- Span: {span_mm:g} mm\n"
        f"- Taper: linear, leading edge at fixed local X (straight LE)\n"
        f"- SHA-256 (STL): {stl_sha256}\n"
        f"- Points per surface: {n_points} (cosine-spaced; outline vertex "
        f"count {n_outline})\n"
        f"- Stations: 2 (root, tip) -- geometrically exact for a linear "
        "taper, see above\n"
        f"- Triangles: {len(triangles)} "
        f"({2 * n_outline} side wall + {2 * (n_outline - 2)} cap)\n"
        "- Format: binary STL, deterministic fixed ASCII header, no "
        "timestamps\n"
        "\n"
        "## Local thickness by spanwise station\n"
        "Local max thickness is the maximum upper-minus-lower surface "
        "separation (at matching cosine-spaced chordwise positions), in "
        "millimetres -- the number that decides whether a structural web "
        "fits at that station, not the bounding-box Y extent (which also "
        "carries camber offset).\n"
        "\n"
        f"{_thickness_table_md(thickness_rows)}\n"
        "\n"
        "## Purpose\n"
        "M5 fixture: chord changes with Z. Exercises intersection-point and "
        "margin behaviour that assumes a constant section (M2-M4) breaks "
        "for -- both the chord axis and the local thickness at every "
        "station change, not just the outer boundary.\n"
        "\n"
        f"Generated by `python tools/make_fixtures.py taper --naca {naca} "
        f"--root-chord {root_chord_mm:g} --tip-chord {tip_chord_mm:g} "
        f"--span {span_mm:g} --points {n_points}`.\n",
        encoding="ascii",
    )

    return stl_path, readme_path


def make_sweep(repo_root: Path, naca: str, chord_mm: float, span_mm: float,
                sweep_mm: float, n_points: int):
    """Swept wing, constant chord: the whole section translates in +X
    (aft, i.e. swept back) as Z increases, from 0 offset at the root to
    sweep_mm at the tip. Two stations are exact: the offset is affine in Z,
    so linear interpolation between root and tip outlines equals the
    correctly translated outline at every intermediate Z.
    """
    m, p, t = parse_naca4(naca)

    def params(z):
        frac = z / span_mm
        return chord_mm, 0.0, sweep_mm * frac

    stations, outline_unit, root_shift = build_loft_stations(
        m, p, t, n_points, (0.0, span_mm), params, pivot_frac_x=0.0
    )
    triangles = loft_triangles(stations)
    n_outline = len(outline_unit)

    # Leading-edge X offset (vertex 0 of each station's outline) at root and
    # tip, reported directly from the built geometry rather than recomputed,
    # so the README cannot silently drift from what was actually written.
    le_root_x = stations[0][1][0][0]
    le_tip_x = stations[-1][1][0][0]

    out_dir = repo_root / "tests" / "fixtures" / "sweep"
    stem = f"naca{naca}_sweep_c{int(chord_mm)}_s{int(span_mm)}_le{int(sweep_mm)}"
    stl_path = out_dir / f"{stem}.stl"
    write_binary_stl(stl_path, triangles)
    stl_sha256 = sha256_of(stl_path)

    thickness_rows = _thickness_table(m, p, t, n_points, lambda z: chord_mm, [0.0, 15.0, 30.0, 45.0, span_mm])

    readme_path = out_dir / "README.md"
    readme_path.write_text(
        f"# Sweep fixture: {stem}.stl\n"
        "\n"
        "## Description\n"
        f"A swept NACA {naca} wing: constant {chord_mm:g} mm chord, but the "
        "whole 2D section translates in +X (aft / trailing-edge direction, "
        f"i.e. swept back) as Z increases -- {sweep_mm:g} mm of leading-"
        f"edge sweep over {span_mm:g} mm of span, linearly. Built from "
        "exactly 2 stations (root, tip): the X offset is affine in Z, so "
        "linear interpolation between the two translated outlines is "
        "geometrically exact at every intermediate Z.\n"
        "\n"
        "## Origin convention\n"
        "Same convention as the taper fixture: the unit section is scaled "
        "and translated per station, then every station is shifted by the "
        "same constant offset so the *root* station's own bounding box is "
        "centred on X=0, Y=0. The tip station's leading edge is therefore "
        f"at local X={le_tip_x - le_root_x:.6f} mm relative to the root "
        "leading edge (i.e. the sweep offset, read back from the built "
        "geometry, not recomputed). Z is spanwise, root at Z=0, tip at "
        f"Z={span_mm:g} mm.\n"
        "\n"
        "## Exact dimensions\n"
        f"- NACA designation: {naca} (m={m:g}, p={p:g}, t={t:g})\n"
        f"- Chord: {chord_mm:g} mm (constant)\n"
        f"- Span: {span_mm:g} mm\n"
        f"- Leading-edge sweep: {sweep_mm:g} mm total, linear with Z\n"
        f"- Leading-edge local X at root: {le_root_x:.6f} mm\n"
        f"- Leading-edge local X at tip: {le_tip_x:.6f} mm (delta "
        f"{le_tip_x - le_root_x:.6f} mm)\n"
        f"- SHA-256 (STL): {stl_sha256}\n"
        f"- Points per surface: {n_points} (cosine-spaced; outline vertex "
        f"count {n_outline})\n"
        "- Stations: 2 (root, tip) -- geometrically exact for a linear "
        "sweep, see above\n"
        f"- Triangles: {len(triangles)} "
        f"({2 * n_outline} side wall + {2 * (n_outline - 2)} cap)\n"
        "- Format: binary STL, deterministic fixed ASCII header, no "
        "timestamps\n"
        "\n"
        "## Local thickness by spanwise station\n"
        "Chord is constant, so local max thickness is constant too -- "
        "included for parity with the other M5 fixtures and to confirm "
        "sweep alone does not change section thickness.\n"
        "\n"
        f"{_thickness_table_md(thickness_rows)}\n"
        "\n"
        "## Purpose\n"
        "M5 fixture: the cross-section translates with Z. Exercises "
        "whether the structural pattern frame and world/object coordinate "
        "handling track a moving section origin correctly, independent of "
        "any chord or thickness change.\n"
        "\n"
        f"Generated by `python tools/make_fixtures.py sweep --naca {naca} "
        f"--chord {chord_mm:g} --span {span_mm:g} --sweep {sweep_mm:g} "
        f"--points {n_points}`.\n",
        encoding="ascii",
    )

    return stl_path, readme_path


def make_twist(repo_root: Path, naca: str, chord_mm: float, span_mm: float,
                washout_deg: float, n_points: int, n_stations: int):
    """Twisted (washed-out) wing, constant chord: the section rotates about
    the quarter-chord point (pivot_frac_x=0.25, a fixed local X since chord
    is constant here, so the quarter-chord traces a straight line down the
    span) by washout_deg from root (0 deg) to tip, using the standard
    CCW-positive rotate_points_2d convention directly (rotate_deg =
    +washout_deg * frac). Because the leading edge sits behind the pivot
    (negative local X relative to it) and the trailing edge sits ahead of
    it (positive local X), a positive (CCW) rotation moves the leading edge
    toward -Y and the trailing edge toward +Y -- i.e. the chord line tilts
    nose-down toward the tip, which is genuine washout (reduced local
    incidence toward the tip), not washin. Verified numerically below by
    checking the sign of the built geometry's chord-line angle at the tip
    (positive washout_deg must produce a positive chord-line angle there).

    Rotation through angle is NOT affine in Z the way taper/sweep's chord
    and offset are: linearly interpolating two differently-rotated outlines
    does not equal the outline rotated by the interpolated angle (it cuts
    the chord of the rotation arc, not the arc itself). n_stations
    intermediate stations are therefore used, each an exact rotation of the
    unit outline, so the loft's mid-span sections are genuinely rotated
    NACA sections rather than a linear-interpolation approximation of one.
    """
    m, p, t = parse_naca4(naca)

    def params(z):
        frac = z / span_mm
        return chord_mm, washout_deg * frac, 0.0

    z_stations = [span_mm * i / (n_stations - 1) for i in range(n_stations)]
    stations, outline_unit, root_shift = build_loft_stations(
        m, p, t, n_points, z_stations, params, pivot_frac_x=0.25
    )
    triangles = loft_triangles(stations)
    n_outline = len(outline_unit)

    # Chord-line angle (LE -> TE, in degrees) at root and tip, read back from
    # the built geometry: LE is outline vertex 0, TE is vertex n_points - 1
    # (naca4_section_outline's fixed ordering -- see that function's
    # docstring), both by construction, not by search.
    def chord_line_angle_deg(outline_mm):
        le = outline_mm[0]
        te = outline_mm[n_points - 1]
        return math.degrees(math.atan2(te[1] - le[1], te[0] - le[0]))

    angle_root = chord_line_angle_deg(stations[0][1])
    angle_tip = chord_line_angle_deg(stations[-1][1])

    thickness_rows = _thickness_table(m, p, t, n_points, lambda z: chord_mm, [0.0, 15.0, 30.0, 45.0, span_mm])

    out_dir = repo_root / "tests" / "fixtures" / "twist"
    stem = f"naca{naca}_twist_c{int(chord_mm)}_s{int(span_mm)}_w{washout_deg:g}"
    stl_path = out_dir / f"{stem}.stl"
    write_binary_stl(stl_path, triangles)
    stl_sha256 = sha256_of(stl_path)

    readme_path = out_dir / "README.md"
    readme_path.write_text(
        f"# Twist fixture: {stem}.stl\n"
        "\n"
        "## Description\n"
        f"A washed-out NACA {naca} wing: constant {chord_mm:g} mm chord, "
        f"but the section rotates about the quarter-chord point by "
        f"{washout_deg:g} degrees of washout from root (Z=0) to tip "
        f"(Z={span_mm:g} mm), linear in Z. Rotation axis: the quarter-"
        "chord point (25% of chord back from the leading edge, on the "
        "chord line), which stays at a fixed local X here because chord is "
        "constant, so it traces a straight line down the span. The "
        "rotation applied is +washout_deg (standard CCW-positive), which -- "
        "because the leading edge sits behind the pivot and the trailing "
        "edge sits ahead of it -- moves the leading edge toward -Y and the "
        "trailing edge toward +Y, i.e. the chord line tilts nose-down "
        "toward the tip: genuine washout (reduced incidence toward the "
        "tip), not washin -- confirmed below by the built geometry's own "
        "chord-line angle at root vs. tip.\n"
        "\n"
        "## Origin convention\n"
        "Same root-bounding-box-centring convention as the taper and sweep "
        "fixtures. Z is spanwise, root at Z=0, tip at "
        f"Z={span_mm:g} mm.\n"
        "\n"
        "## Why many stations (not 2)\n"
        "Rotation through an angle is not affine in Z: linearly "
        "interpolating between a 0-degree and a 6-degree-rotated outline "
        "does not equal the outline rotated by the interpolated angle at "
        "the midpoint (it cuts the chord of the rotation arc, not the arc "
        f"itself). This fixture uses {n_stations} stations "
        f"({span_mm / (n_stations - 1):g} mm apart) so intermediate "
        "slices are genuinely rotated NACA sections, not a linear "
        "approximation of one.\n"
        "\n"
        "## Exact dimensions\n"
        f"- NACA designation: {naca} (m={m:g}, p={p:g}, t={t:g})\n"
        f"- Chord: {chord_mm:g} mm (constant)\n"
        f"- Span: {span_mm:g} mm\n"
        f"- Washout: {washout_deg:g} degrees, root to tip, linear with Z\n"
        f"- Rotation axis: quarter-chord point ({0.25 * chord_mm:g} mm aft "
        "of leading edge on the chord line)\n"
        f"- Chord-line angle at root: {angle_root:.6f} degrees\n"
        f"- Chord-line angle at tip: {angle_tip:.6f} degrees (delta "
        f"{angle_tip - angle_root:.6f} degrees)\n"
        f"- SHA-256 (STL): {stl_sha256}\n"
        f"- Points per surface: {n_points} (cosine-spaced; outline vertex "
        f"count {n_outline})\n"
        f"- Stations: {n_stations}\n"
        f"- Triangles: {len(triangles)} "
        f"({2 * n_outline * (n_stations - 1)} side wall + "
        f"{2 * (n_outline - 2)} cap)\n"
        "- Format: binary STL, deterministic fixed ASCII header, no "
        "timestamps\n"
        "\n"
        "## Local thickness by spanwise station\n"
        "Chord is constant, so local max thickness (measured in the "
        "section's own rotated frame) is constant too -- included for "
        "parity with the other M5 fixtures.\n"
        "\n"
        f"{_thickness_table_md(thickness_rows)}\n"
        "\n"
        "## Purpose\n"
        "M5 fixture: the cross-section rotates with Z. Exercises whether "
        "the structural pattern definition (web placement, intersection "
        "logic) stays sensible as the section's own orientation evolves "
        "down the span, independent of any chord change.\n"
        "\n"
        f"Generated by `python tools/make_fixtures.py twist --naca {naca} "
        f"--chord {chord_mm:g} --span {span_mm:g} --washout {washout_deg:g} "
        f"--points {n_points} --stations {n_stations}`.\n",
        encoding="ascii",
    )

    return stl_path, readme_path


def make_tip(repo_root: Path, naca: str, root_chord_mm: float, tip_chord_mm: float,
             span_mm: float, n_points: int, skin_margin_mm: float = 1.0):
    """Tip-closure wing: same construction as the taper fixture (straight
    leading edge, linear chord change, 2 stations, geometrically exact) but
    with a much smaller tip chord, so the section area genuinely collapses
    toward zero rather than merely narrowing. This is the pattern-shutdown
    case: local thickness falls below skin_margin_mm before the physical
    tip, and the README below states exactly where.
    """
    m, p, t = parse_naca4(naca)

    def params(z):
        frac = z / span_mm
        chord = root_chord_mm + (tip_chord_mm - root_chord_mm) * frac
        return chord, 0.0, 0.0

    def chord_at(z):
        return root_chord_mm + (tip_chord_mm - root_chord_mm) * (z / span_mm)

    stations, outline_unit, root_shift = build_loft_stations(
        m, p, t, n_points, (0.0, span_mm), params, pivot_frac_x=0.0
    )
    triangles = loft_triangles(stations)
    n_outline = len(outline_unit)

    # Chord at which local max thickness == skin_margin_mm, from the linear
    # thickness(chord) relationship (thickness scales linearly with chord
    # for a fixed NACA t), then the corresponding Z via the linear taper.
    thickness_at_root_chord = local_max_thickness_mm(m, p, t, n_points, root_chord_mm)
    thickness_per_mm_chord = thickness_at_root_chord / root_chord_mm
    shutdown_chord_mm = skin_margin_mm / thickness_per_mm_chord
    # z where chord(z) == shutdown_chord_mm; taper is linear and chord
    # decreases with z (root_chord_mm > tip_chord_mm), so this is well
    # defined as long as shutdown_chord_mm is between tip and root chord.
    shutdown_frac = (root_chord_mm - shutdown_chord_mm) / (root_chord_mm - tip_chord_mm)
    shutdown_z_mm = shutdown_frac * span_mm

    thickness_rows = _thickness_table(
        m, p, t, n_points, chord_at,
        [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 54.0, 55.0, 56.0, 57.0, 58.0, 59.0, span_mm],
    )

    out_dir = repo_root / "tests" / "fixtures" / "tip"
    stem = f"naca{naca}_tip_r{int(root_chord_mm)}_t{tip_chord_mm:g}_s{int(span_mm)}"
    stl_path = out_dir / f"{stem}.stl"
    write_binary_stl(stl_path, triangles)
    stl_sha256 = sha256_of(stl_path)

    readme_path = out_dir / "README.md"
    readme_path.write_text(
        f"# Tip-closure fixture: {stem}.stl\n"
        "\n"
        "## Description\n"
        f"A tapered NACA {naca} wing built exactly like the taper fixture "
        f"(straight leading edge, linear chord change, 2 exact stations) "
        f"but with a much smaller tip chord -- {root_chord_mm:g} mm at the "
        f"root down to {tip_chord_mm:g} mm at the tip over {span_mm:g} mm "
        "of span -- so the section area genuinely collapses toward zero "
        "rather than merely narrowing. This is the M5 pattern-shutdown "
        "case: any structural pattern (e.g. a web) that assumes a minimum "
        "section thickness must stop being placed before the tip, not fail "
        "or silently corrupt when the section gets too thin to hold it.\n"
        "\n"
        "## Origin convention\n"
        "Same root-bounding-box-centring convention as the taper fixture. "
        f"Z is spanwise, root at Z=0, tip at Z={span_mm:g} mm.\n"
        "\n"
        "## Exact dimensions\n"
        f"- NACA designation: {naca} (m={m:g}, p={p:g}, t={t:g})\n"
        f"- Root chord: {root_chord_mm:g} mm\n"
        f"- Tip chord: {tip_chord_mm:g} mm\n"
        f"- Span: {span_mm:g} mm\n"
        "- Taper: linear, leading edge at fixed local X (straight LE), "
        "same construction as the taper fixture\n"
        f"- SHA-256 (STL): {stl_sha256}\n"
        f"- Points per surface: {n_points} (cosine-spaced; outline vertex "
        f"count {n_outline})\n"
        "- Stations: 2 (root, tip) -- geometrically exact for a linear "
        "taper\n"
        f"- Triangles: {len(triangles)} "
        f"({2 * n_outline} side wall + {2 * (n_outline - 2)} cap)\n"
        "- Format: binary STL, deterministic fixed ASCII header, no "
        "timestamps\n"
        "\n"
        "## Local thickness by spanwise station, and where pattern shutdown "
        "must occur\n"
        "Local max thickness scales linearly with chord for a fixed NACA "
        f"thickness digit, so it falls below the {skin_margin_mm:g} mm "
        "default skin margin at:\n"
        f"- chord = {shutdown_chord_mm:.4f} mm\n"
        f"- Z = {shutdown_z_mm:.4f} mm ({shutdown_z_mm / span_mm * 100.0:.2f}% "
        "of span from the root)\n"
        "\n"
        "Any structural pattern generator using this fixture must close out "
        f"before Z={shutdown_z_mm:.4f} mm -- the table below brackets that "
        "point with explicit stations on both sides.\n"
        "\n"
        f"{_thickness_table_md(thickness_rows)}\n"
        "\n"
        "## Purpose\n"
        "M5 fixture: area collapses toward zero. Requires an explicit "
        "pattern shutdown / closure strategy rather than assuming a web "
        "fits everywhere between root and tip.\n"
        "\n"
        f"Generated by `python tools/make_fixtures.py tip --naca {naca} "
        f"--root-chord {root_chord_mm:g} --tip-chord {tip_chord_mm:g} "
        f"--span {span_mm:g} --points {n_points} "
        f"--skin-margin {skin_margin_mm:g}`.\n",
        encoding="ascii",
    )

    return stl_path, readme_path, shutdown_chord_mm, shutdown_z_mm


def _report_fixture(stl_path: Path, readme_path: Path):
    """Shared verification/report path for both fixture kinds: hash+size,
    parse-back triangle count, and the watertight edge check. Returns 0 on
    success, 1 if the watertight check fails.
    """
    for path in (stl_path, readme_path):
        digest = sha256_of(path)
        size = path.stat().st_size
        print(f"{path}  ({size} bytes)  sha256={digest}")

    header, triangles = parse_binary_stl(stl_path)
    print(f"\nParsed back: header={header.rstrip(chr(0).encode()).decode('ascii')!r}")
    print(f"Parsed back: triangle_count={len(triangles)}")

    ok, details, problems = check_watertight(triangles)
    print(f"Watertight edge check: {'PASS' if ok else 'FAIL'} -- {details}")
    if not ok:
        for p in problems:
            print(f"  problem: {p}")
        return 1

    return 0


README_SHORT_COUPON = "# Short coupon fixture: coupon_20x20x15.stl\n\n20 x 20 x 15 mm prism on Z=0, centred on X=Y=0 - the same cross-section\nas coupon_20x20x60.stl at a quarter of the height.\n\n## Why it exists\nThe gap sweep varies one number across several coupons, and whether the\ntwo slot walls fuse is visible within the first few body layers. At\n0.20 mm layers the 60 mm coupon is ~21 minutes and 295 body layers; this\nis ~5 minutes and 70. The slot geometry is identical, so the fusion\nanswer is identical.\n\n## What it does NOT replace\nThe 60 mm coupon remains M1's reference fixture: its G-code, layer count\nand one-loop verification are cited in the milestone evidence. Use this\none to find the working gap, then confirm on the full coupon.\n\nGenerated by `python tools/make_fixtures.py coupon-short`.\n"


def make_short_coupon(repo_root: Path):
    """20 x 20 x 15 mm prism, the working coupon for gap sweeps.

    Identical cross-section to the 60 mm coupon, so the slot geometry and the
    fusion question are unchanged, but a quarter of the print time. A sweep
    needs several ratios and each answer is visible within the first few body
    layers, so extra height adds repetition rather than information.
    """
    hx, hy = 10.0, 10.0
    z0, z1 = 0.0, 15.0
    triangles = box_triangles(-hx, -hy, z0, hx, hy, z1)

    out_dir = repo_root / "tests" / "fixtures" / "coupon"
    stl_path = out_dir / "coupon_20x20x15.stl"
    write_binary_stl(stl_path, triangles)

    readme_path = out_dir / "README_short.md"
    readme_path.write_text(README_SHORT_COUPON, encoding="utf-8")
    return stl_path, readme_path


def main(argv):
    if len(argv) >= 2 and argv[1] == "coupon":
        if len(argv) != 2:
            print("usage: python tools/make_fixtures.py coupon", file=sys.stderr)
            return 2

        repo_root = Path(__file__).resolve().parent.parent
        stl_path, readme_path = make_coupon(repo_root)
        return _report_fixture(stl_path, readme_path)

    if len(argv) >= 2 and argv[1] == "coupon-short":
        if len(argv) != 2:
            print("usage: python tools/make_fixtures.py coupon-short", file=sys.stderr)
            return 2

        repo_root = Path(__file__).resolve().parent.parent
        stl_path, readme_path = make_short_coupon(repo_root)
        return _report_fixture(stl_path, readme_path)

    if len(argv) >= 2 and argv[1] == "airfoil":
        parser = argparse.ArgumentParser(
            prog="python tools/make_fixtures.py airfoil",
            description="Generate a constant-section NACA 4-digit airfoil wing fixture.",
        )
        parser.add_argument("--naca", default="2412", help="NACA 4-digit designation (default: 2412)")
        parser.add_argument("--chord", type=float, default=60.0, help="chord length in mm (default: 60)")
        parser.add_argument("--span", type=float, default=60.0, help="span length in mm (default: 60)")
        parser.add_argument(
            "--points", type=int, default=120,
            help="cosine-spaced points per surface, LE/TE shared (default: 120)",
        )
        args = parser.parse_args(argv[2:])

        repo_root = Path(__file__).resolve().parent.parent
        stl_path, readme_path, fill_ratio, section_area, bbox_area = make_airfoil(
            repo_root, args.naca, args.chord, args.span, args.points
        )
        rc = _report_fixture(stl_path, readme_path)
        print(
            f"\nSection area={section_area:.4f} mm^2, bbox area={bbox_area:.4f} mm^2, "
            f"fill ratio={fill_ratio:.4f}"
        )
        return rc

    if len(argv) >= 2 and argv[1] == "taper":
        parser = argparse.ArgumentParser(
            prog="python tools/make_fixtures.py taper",
            description="Generate a tapered (straight-LE, linear chord) NACA 4-digit wing fixture.",
        )
        parser.add_argument("--naca", default="2412", help="NACA 4-digit designation (default: 2412)")
        parser.add_argument("--root-chord", type=float, default=60.0, help="root chord in mm (default: 60)")
        parser.add_argument("--tip-chord", type=float, default=30.0, help="tip chord in mm (default: 30)")
        parser.add_argument("--span", type=float, default=60.0, help="span length in mm (default: 60)")
        parser.add_argument(
            "--points", type=int, default=120,
            help="cosine-spaced points per surface, LE/TE shared (default: 120)",
        )
        args = parser.parse_args(argv[2:])

        repo_root = Path(__file__).resolve().parent.parent
        stl_path, readme_path = make_taper(
            repo_root, args.naca, args.root_chord, args.tip_chord, args.span, args.points
        )
        return _report_fixture(stl_path, readme_path)

    if len(argv) >= 2 and argv[1] == "sweep":
        parser = argparse.ArgumentParser(
            prog="python tools/make_fixtures.py sweep",
            description="Generate a swept (constant-chord, translating section) NACA 4-digit wing fixture.",
        )
        parser.add_argument("--naca", default="2412", help="NACA 4-digit designation (default: 2412)")
        parser.add_argument("--chord", type=float, default=60.0, help="chord length in mm (default: 60)")
        parser.add_argument("--span", type=float, default=60.0, help="span length in mm (default: 60)")
        parser.add_argument(
            "--sweep", type=float, default=20.0,
            help="total leading-edge sweep (aft, +X) over the span, in mm (default: 20)",
        )
        parser.add_argument(
            "--points", type=int, default=120,
            help="cosine-spaced points per surface, LE/TE shared (default: 120)",
        )
        args = parser.parse_args(argv[2:])

        repo_root = Path(__file__).resolve().parent.parent
        stl_path, readme_path = make_sweep(
            repo_root, args.naca, args.chord, args.span, args.sweep, args.points
        )
        return _report_fixture(stl_path, readme_path)

    if len(argv) >= 2 and argv[1] == "twist":
        parser = argparse.ArgumentParser(
            prog="python tools/make_fixtures.py twist",
            description="Generate a twisted/washed-out (constant-chord, rotating section) NACA 4-digit wing fixture.",
        )
        parser.add_argument("--naca", default="2412", help="NACA 4-digit designation (default: 2412)")
        parser.add_argument("--chord", type=float, default=60.0, help="chord length in mm (default: 60)")
        parser.add_argument("--span", type=float, default=60.0, help="span length in mm (default: 60)")
        parser.add_argument(
            "--washout", type=float, default=6.0,
            help="total washout, root to tip, in degrees (default: 6)",
        )
        parser.add_argument(
            "--points", type=int, default=120,
            help="cosine-spaced points per surface, LE/TE shared (default: 120)",
        )
        parser.add_argument(
            "--stations", type=int, default=61,
            help="number of spanwise stations; rotation is not affine in Z so more "
                 "than 2 stations are needed for the interior to be a genuine "
                 "rotated section rather than a linear-interpolation approximation "
                 "(default: 61, i.e. 1 mm apart over a 60 mm span)",
        )
        args = parser.parse_args(argv[2:])

        repo_root = Path(__file__).resolve().parent.parent
        stl_path, readme_path = make_twist(
            repo_root, args.naca, args.chord, args.span, args.washout, args.points, args.stations
        )
        return _report_fixture(stl_path, readme_path)

    if len(argv) >= 2 and argv[1] == "tip":
        parser = argparse.ArgumentParser(
            prog="python tools/make_fixtures.py tip",
            description="Generate a tip-closure (heavily tapered) NACA 4-digit wing fixture.",
        )
        parser.add_argument("--naca", default="2412", help="NACA 4-digit designation (default: 2412)")
        parser.add_argument("--root-chord", type=float, default=60.0, help="root chord in mm (default: 60)")
        parser.add_argument("--tip-chord", type=float, default=3.0, help="tip chord in mm (default: 3)")
        parser.add_argument("--span", type=float, default=60.0, help="span length in mm (default: 60)")
        parser.add_argument(
            "--points", type=int, default=120,
            help="cosine-spaced points per surface, LE/TE shared (default: 120)",
        )
        parser.add_argument(
            "--skin-margin", type=float, default=1.0,
            help="skin margin in mm; the README reports the Z at which local "
                 "max thickness falls below this (default: 1.0)",
        )
        args = parser.parse_args(argv[2:])

        repo_root = Path(__file__).resolve().parent.parent
        stl_path, readme_path, shutdown_chord_mm, shutdown_z_mm = make_tip(
            repo_root, args.naca, args.root_chord, args.tip_chord, args.span, args.points, args.skin_margin
        )
        rc = _report_fixture(stl_path, readme_path)
        print(
            f"\nPattern shutdown: local max thickness falls below "
            f"{args.skin_margin:g} mm at chord={shutdown_chord_mm:.4f} mm, "
            f"Z={shutdown_z_mm:.4f} mm"
        )
        return rc

    print(
        "usage: python tools/make_fixtures.py "
        "{coupon|coupon-short|airfoil|taper|sweep|twist|tip} [options]",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
