"""Global in-plane wing alignment: one theta, detected from the mesh, once per run.

Distinct from ``frame.estimate_rotation_deg``/``check_rotation_envelope``,
which measure and cap RESIDUAL washout (2-6 degree design range) INSIDE an
already chord-aligned frame. This module answers a different question asked
BEFORE that one is even meaningful: is the wing's chord along world X at all? A
wing whose chord sits at an arbitrary in-plane angle (mesh placement, not
aerodynamic twist) fails the 30-degree washout cap at the first body layer --
that is a misdiagnosis of an alignment problem as a washout problem.

Pure millimetre-float geometry, no ``GeometryOps``/host imports (structure
generation and validation are ordinary geometry logic in ``aircraft_core``,
with no Orca imports). The host boundary
(``plugin/aircraft_structures/capability.py``) converts to/from scaled integers
and calls ``rotate_points_deg`` -- the ONE rotation function used at BOTH call
sites (the mesh sectioner's float mm points, and the host's own per-layer slice
points converted to mm at the boundary). Two independent rotation
implementations could drift apart; using one function everywhere is what lets
``capability.py``'s ``_check_mesh_alignment`` report any disagreement as a
runtime number instead of shipping it silently.
"""

import bisect
import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]


class AlignmentError(Exception):
    """Raised when a global theta (or the leading-edge side) cannot be
    determined from the supplied geometry. Callers translate this into the
    project's standard fail-closed diagnostic (layer index / station, Z,
    cause) -- this module has no notion of a layer or a host, so it never
    constructs that diagnostic itself.

    Two failure modes exist and callers MUST treat them differently
    (rectangular structural-coupon fixtures have no chord axis at all --
    see ``ChordAxisUnavailableError``):

    - ``ChordAxisUnavailableError`` (subclass, below): the section has no
      chord axis to find in the first place -- not aerofoil-shaped at
      all (no hull area, or no genuine trailing edge anywhere). This is
      NOT a wing alignment problem; a caller MAY treat it as theta=0/no
      rotation for that station rather than failing the whole run
      (fail-closed applies to a WING whose alignment genuinely cannot be
      read, not to a shape this method was never meant to classify).
    - Plain ``AlignmentError`` (this class, everywhere else it is raised
      directly -- leading-edge ambiguous after a genuine chord axis WAS
      found, or a whole wing's per-station spread exceeded the washout
      envelope in ``resolve_wing_theta_deg``): a genuine, aerofoil-shaped
      section (or wing) with a real, unresolved alignment problem.
      Callers MUST fail closed -- never silently proceed with theta=0
      here.
    """


class BluntTrailingEdgeError(AlignmentError):
    """A genuine AEROFOIL section whose trailing edge is squared off, so
    ``chord_line_angle_deg`` cannot read its chord axis.

    FATAL, deliberately. Falling back to theta=0 here would let a wing
    misaligned by 15 degrees slice to completion with every member built 15
    degrees off the true chord axis (measured: 1495 layers ok, 0 failed). A
    confident wrong answer is worse than a refusal, so this refuses.

    Distinguished from a structural COUPON (which also has no readable
    chord axis, and must stay non-fatal) purely by hull vertex count --
    see ``AEROFOIL_MIN_HULL_VERTICES``.

    A hull-diameter fallback axis is not used: its error is camber- and
    truncation-correlated, reaching 2.18 deg, which is worse than the ~1.5 deg
    that disqualifies ``min_area_rect_angle_deg``.
    """


class ChordAxisUnavailableError(AlignmentError):
    """The section has no chord axis at all -- no hull area (degenerate/
    collinear points), or no genuine trailing edge anywhere on the hull
    (``chord_line_angle_deg`` returned ``None``: every hull vertex is
    blunter than ``TE_ANGLE_DEGENERATE_THRESHOLD_DEG``). NOT a wing
    alignment problem -- see ``AlignmentError``'s own docstring for the
    distinction callers must make.

    KNOWN, ACCEPTED LIMIT (measured; see
    ``tests/unit/test_alignment.py``'s ``TruncatedTrailingEdgeTests``): a wing
    with a SQUARED-OFF/TRUNCATED trailing edge -- extremely common on printed
    wings, often required for a thin printed section to have a manufacturable,
    non-feathered TE at all -- presents TE corners near 90 degrees (measured
    93.85-94.19 deg across truncation levels 85%-99.9% of chord), which is
    ABOVE ``TE_ANGLE_DEGENERATE_THRESHOLD_DEG`` (60 deg) and therefore takes
    THIS branch: theta=0, no rotation, non-fatal. That is a genuine wing taking
    the "not a wing" branch -- a deliberately accepted residual exposure.

    The exposure is bounded: a genuine wing left unrotated by this branch is
    not left unrotated unconditionally. If its TRUE misalignment is 30 degrees
    or more, the independent ``frame.estimate_rotation_deg``/
    ``check_rotation_envelope`` cap (``DEFAULT_MAX_ROTATION_DEG = 30``) still
    fails the slice closed, per layer. The remaining exposure is a
    truncated-TE wing misaligned by LESS than 30 degrees, which is sliced
    with the washout-envelope correction applied at whatever residual angle
    the section carries.
    """


def rotate_points_deg(
    points: Sequence[Point], theta_deg: float, center: Point = (0.0, 0.0)
) -> List[Point]:
    """Rotate ``points`` (mm) by ``theta_deg`` (degrees, CCW positive) about
    ``center`` (mm). Pure float rotation -- the SINGLE implementation every
    call site must use for a global alignment rotation (see module
    docstring). Not used for washout/residual-angle
    corrections, which stay in ``frame.py``.
    """
    if not points:
        return []
    theta = math.radians(theta_deg)
    c, s = math.cos(theta), math.sin(theta)
    cx, cy = center
    out = []
    for x, y in points:
        dx, dy = x - cx, y - cy
        out.append((cx + dx * c - dy * s, cy + dx * s + dy * c))
    return out


def _convex_hull(points: Sequence[Point]) -> List[Point]:
    """Andrew's monotone chain convex hull, CCW, no repeated first/last
    point. Deduplicates exact-coincident points (harmless for a fine mesh
    section; guards the rotating-calipers loop below against a zero-length
    edge from a duplicated vertex)."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: List[Point] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: List[Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def min_area_rect_angle_deg(points: Sequence[Point]) -> Optional[float]:
    """Angle (degrees, normalized to ``(-90, 90]``) of the long axis of the
    minimum-area bounding rectangle enclosing ``points``, measured from
    world X. Rotating calipers over the convex hull: O(n log n) for the
    hull, O(n) for the caliper sweep (one candidate orientation per hull
    edge, the well-known result that the min-area rectangle is always
    flush with a hull edge).

    Returns ``None`` if ``points`` do not enclose an area (fewer than 3
    hull vertices -- collinear or coincident points): the caller must
    treat that as "theta undetermined", not silently fall back to 0.

    **DO NOT use this as a chord-axis estimator. It is CAMBER-BIASED;
    ``chord_line_angle_deg`` is the chord-axis estimator.** A cambered
    aerofoil's minimum-area rectangle sits flush with the near-flat lower
    surface, not the leading-edge-to-trailing-edge chord line, so it reads a
    real, non-noise angle at TRUE zero rotation. Measured against the mesh
    sectioner on the reference fixture at its documented zero-rotation root
    station
    (``tests/fixtures/wing/naca2412_wing_r150_t90_s400_w3.json``:
    ``chord_line_angle_root_deg: 0.0``)::

        f=0.0001  min_area_rect=1.5274 deg   (true angle: 0.0 deg)
        f=0.5000  min_area_rect=3.0271 deg   (true washout: 1.5 deg)
        f=0.9000  min_area_rect=4.2271 deg   (true washout: 2.7 deg)

    and against a real wing STL that the chord-line method measures as aligned
    to within 0.02 deg but this function reads as a near-constant ~1.42-1.43
    deg across all seven sampled span stations -- the camber bias, not a real
    rotation. ~1.5 deg is well above the alignment epsilon and would apply a
    spurious global de-rotation to an already-aligned wing.

    It remains valid for what it measures -- a bounding-rectangle
    extent/orientation query, e.g. bed-fit or bounding-box reasoning -- but
    not as a stand-in for the aerodynamic/print chord line, and it is not
    ``resolve_theta_deg``'s axis source.
    """
    hull = _convex_hull(points)
    n = len(hull)
    if n < 3:
        return None
    best_area = None
    best_angle = 0.0
    best_dims = None
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        edge_angle = math.atan2(y2 - y1, x2 - x1)
        c, s = math.cos(-edge_angle), math.sin(-edge_angle)
        xs = [x * c - y * s for x, y in hull]
        ys = [x * s + y * c for x, y in hull]
        w = max(xs) - min(xs)
        h = max(ys) - min(ys)
        area = w * h
        if best_area is None or area < best_area - 1e-9:
            best_area = area
            best_angle = edge_angle
            best_dims = (w, h)
    if best_dims is None:
        return None
    w, h = best_dims
    angle_deg = math.degrees(best_angle)
    # The long axis is whichever of (w, h) is larger; if it is the
    # perpendicular one, rotate the candidate angle 90 degrees to describe
    # the LONG axis specifically (chord, not thickness).
    if w < h:
        angle_deg += 90.0
    while angle_deg <= -90.0:
        angle_deg += 180.0
    while angle_deg > 90.0:
        angle_deg -= 180.0
    return angle_deg


# Sharpest hull-vertex interior angle (degrees) above which a station is
# treated as having NO genuine trailing edge.
#
# Three measured clusters:
#
#     real aerofoil TE (NACA 2412 mesh fixtures, mid-span,
#     measured via the MeshSectioner)                      16.50-16.55 deg
#     a real STL, genuine-TE stations                      11.5-11.6 deg
#     an axis-aligned RECTANGLE (a plain coupon; every
#     corner is a right angle by construction)             90.0 deg  exactly
#     a real STL, blunt flat-cut root stations             173.9-174.0 deg
#
# A rectangular coupon's 90 deg corner is NOT "> 90 deg", so a 90.0 deg
# threshold would let ``chord_line_angle_deg`` treat a plain rectangular
# fixture as having a "genuine trailing edge" and compute a spurious DIAGONAL
# axis angle for it (atan2 between two arbitrary opposite corners) --
# silently applying a wrong global rotation to a structural coupon that has
# no chord axis at all (see ``tests/unit/test_alignment.py``'s
# ``ChordLineAngleTests``/degenerate coverage).
#
# 60.0 deg separates all three clusters with real margin on both sides of
# the boundary that matters (the rectangle): 43.5 deg of headroom above the
# real-TE cluster (16.5 to 60), 30 deg below the rectangle (60 to 90), and
# 114 deg below the degenerate-flat-cut cluster (60 to 174).
TE_ANGLE_DEGENERATE_THRESHOLD_DEG = 60.0

# Fraction of the hull-diameter length used to cluster points at each end
# before taking their centroid, for a clustered-endpoint hull-diameter axis.
# Not referenced by the current detectors. 2% is wide
# enough to capture both corners of a trailing edge truncated at 90% chord (the
# corpus fixture) and narrow enough not to reach back into the section's curved
# flanks, where including points would drag the endpoint off the true extremum.
# Not swept -- one value, one fixture family; widening the corpus should
# re-measure it.
HULL_DIAMETER_END_CLUSTER_FRACTION = 0.02

# minimum convex-hull vertex count for a section to be treated as an AEROFOIL
# rather than a polygonal structural coupon. An aerofoil section is a
# discretised CURVE and reports many hull vertices; a coupon reports a handful.
# Measured:
#
#   rect 62x10 (any rotation)..............  4 hull vertices
#   pre-compensated box section........  6
#   truncated-TE NACA 2412 (corpus).... 300
#
# 12 sits at 2x the largest coupon and far below any aerofoil measured:
# the MINIMUM over 9 stations across all 16 wing fixtures on disk is 198.
#
# What it decides: a blunt/flat-cut section with MANY hull vertices is a real
# wing whose trailing edge is squared off -- that FAILS THE SLICE CLOSED with
# an actionable message, because slicing it unrotated silently builds every
# member off-axis. A blunt section with FEW hull vertices is a structural
# coupon, which takes the non-fatal path (theta=0, unrotated) because a coupon
# has no chord axis to get wrong.
#
# NOT VALIDATED against a coarsely tessellated user mesh. A real wing whose STL
# is decimated below 12 hull vertices per section would be misclassified as a
# coupon and silently unrotated -- the defect this threshold exists to
# prevent.
AEROFOIL_MIN_HULL_VERTICES = 12


def _sharpest_hull_vertex(hull: Sequence[Point]) -> Optional[Tuple[int, float]]:
    """Index and interior angle (degrees) of the hull vertex with the
    smallest interior angle -- the trailing-edge candidate for
    ``chord_line_angle_deg``. Interior angle is the angle between the two
    edges meeting at that vertex (0 deg = a folded-back spike, 180 deg = no
    corner at all, a straight run of hull points).

    Returns ``None`` if no hull vertex has two well-defined (non-zero-
    length) neighbouring edges (degenerate hull).
    """
    n = len(hull)
    best_i = None
    best_angle = None
    for i in range(n):
        a = hull[(i - 1) % n]
        b = hull[i]
        c = hull[(i + 1) % n]
        v1 = (a[0] - b[0], a[1] - b[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        d1 = math.hypot(*v1)
        d2 = math.hypot(*v2)
        if d1 < 1e-9 or d2 < 1e-9:
            continue
        cosang = (v1[0] * v2[0] + v1[1] * v2[1]) / (d1 * d2)
        cosang = max(-1.0, min(1.0, cosang))
        angle = math.degrees(math.acos(cosang))
        if best_angle is None or angle < best_angle:
            best_angle = angle
            best_i = i
    if best_i is None:
        return None
    return best_i, best_angle


def chord_line_angle_deg(
    points: Sequence[Point],
    te_angle_threshold_deg: float = TE_ANGLE_DEGENERATE_THRESHOLD_DEG,
) -> Optional[float]:
    """Directed chord-line angle (degrees, ``(-180, 180]``, from world X) from
    LEADING edge to TRAILING edge -- ``resolve_theta_deg``'s axis source.
    (``min_area_rect_angle_deg`` is camber-biased; see that function's own
    docstring for the measurement that disqualifies it.)

    Method: trailing edge (TE) is the convex-hull vertex with the smallest
    interior angle (the sharpest corner -- a real aerofoil's TE is a real,
    sharp or near-sharp point; ``_sharpest_hull_vertex``). Leading edge
    (LE) is the hull point farthest from TE. The angle is
    ``atan2(TE.y - LE.y, TE.x - LE.x)``, i.e. already DIRECTED from LE to
    TE -- unlike ``min_area_rect_angle_deg``, no separate mod-180 direction
    step is needed from this function alone (``resolve_theta_deg`` still
    runs the independent ``leading_edge_side`` thickness check afterward as
    a cross-check that proves the two signals agree -- not because this
    angle is undirected).

    Returns ``None`` (station UNUSABLE) when:

    - fewer than 3 hull vertices (no area), or
    - the sharpest hull-vertex interior angle exceeds
      ``te_angle_threshold_deg`` -- no genuine trailing edge exists (a
      blunt, flat-cut station, e.g. a wing root cut square against a
      fuselage; measured on a real wing STL at 174 deg vs. 11.5 deg for a
      real TE -- see ``TE_ANGLE_DEGENERATE_THRESHOLD_DEG``'s own comment
      for the exact numbers), or
    - LE and TE coincide (degenerate).

    Callers MUST NOT guess an angle from a degenerate station; see
    ``resolve_theta_deg`` (fails closed) and ``resolve_wing_theta_deg``
    (excludes a degenerate non-authoritative station from the spread
    check, but still fails closed if the MID-SPAN station itself is
    degenerate).
    """
    hull = _convex_hull(points)
    if len(hull) < 3:
        return None
    found = _sharpest_hull_vertex(hull)
    if found is None:
        return None
    te_idx, te_angle = found
    if te_angle > te_angle_threshold_deg:
        return None
    te = hull[te_idx]
    le = max(hull, key=lambda p: (p[0] - te[0]) ** 2 + (p[1] - te[1]) ** 2)
    if le == te:
        return None
    return math.degrees(math.atan2(te[1] - le[1], te[0] - le[0]))


def leading_edge_side(
    points: Sequence[Point],
    station_count: int = 41,
    symmetry_tolerance: float = 0.08,
    peak_margin: float = 0.04,
) -> Optional[str]:
    """Which end of the X extent holds the leading edge, for ``points``
    already in a chord-along-X frame (the axis resolved by
    ``min_area_rect_angle_deg``, direction not yet resolved).

    Method: bin the section into ``station_count`` chord
    stations and measure local thickness (max Y - min Y of points in each
    bin) at each station. A real aerofoil's thickness distribution is
    asymmetric about mid-chord (thickest well forward, tapering to a sharp
    or near-sharp trailing edge); which SIDE of mid-chord carries the
    overall thickness peak decides the leading edge.

    Two independent checks, both must pass, or the caller MUST fail closed
    (returns ``None``, never guesses):

    1. **Symmetry check.** Sum of ``|thickness[i] - thickness[mirror(i)]|``
       across all stations, normalized by total thickness. A flat plate or
       any section symmetric about mid-chord scores near 0 here regardless
       of where its (numerically arbitrary) single peak bin happens to
       land. Comparing only the two half-maxima is NOT a reliable
       discriminator: on a real tapering aerofoil the aft half's own local
       max can sit only a few percent below the true (forward) peak, simply
       because thickness is continuous, which gives false ambiguous verdicts
       on real aerofoil shapes.
    2. **Peak-margin check.** The station carrying the global thickness
       peak must not sit within ``peak_margin`` of mid-chord (0.5) --
       otherwise "which half is it in" is not answerable within this
       method's own station resolution.

    Returns ``"min_x"`` (peak forward of mid-chord -- LE already at
    min_x, no flip needed), ``"max_x"`` (peak aft of mid-chord -- the
    section is really 180 degrees around; apply the flip), or ``None``
    (flat plate, section symmetric about mid-chord, or peak too close to
    mid-chord to call).
    """
    if len(points) < 3:
        return None
    xs = [p[0] for p in points]
    min_x, max_x = min(xs), max(xs)
    span = max_x - min_x
    if span <= 0:
        return None
    bins: List[List[float]] = [[] for _ in range(station_count)]
    for x, y in points:
        idx = int((x - min_x) / span * (station_count - 1))
        idx = max(0, min(station_count - 1, idx))
        bins[idx].append(y)
    thickness = [
        (max(b) - min(b)) if len(b) >= 2 else 0.0 for b in bins
    ]
    max_t = max(thickness)
    if max_t <= 0.0:
        return None

    mirrored = thickness[::-1]
    asym = sum(abs(a - b) for a, b in zip(thickness, mirrored))
    norm = sum(thickness) + sum(mirrored)
    symmetry_ratio = (asym / norm) if norm > 0.0 else 0.0
    if symmetry_ratio < symmetry_tolerance:
        return None

    peak_idx = thickness.index(max_t)
    peak_frac = peak_idx / (station_count - 1)
    if abs(peak_frac - 0.5) < peak_margin:
        return None
    return "min_x" if peak_frac < 0.5 else "max_x"


def resolve_theta_deg(
    points: Sequence[Point],
    te_angle_threshold_deg: float = TE_ANGLE_DEGENERATE_THRESHOLD_DEG,
) -> float:
    """One theta (degrees, normalized to ``(-180, 180]``) such that rotating
    ``points`` by ``-theta`` (``rotate_points_deg(points, -theta)``) puts
    the wing's chord along +X with the leading edge at min_x -- the single
    global rotation applied to the whole wing.

    Axis source: ``chord_line_angle_deg`` (TE = sharpest hull vertex, LE =
    farthest hull point from it), NOT ``min_area_rect_angle_deg``
    (camber-biased -- see that function's docstring). The chord-line angle
    already comes out DIRECTED (LE to TE), but this function still runs the
    independent ``leading_edge_side`` thickness check on the de-rotated points
    and applies a +180 correction if it disagrees -- two independent signals
    that must agree, not a second identical computation.

    Raises ``AlignmentError`` when:

    - the section has no hull area, or
    - the station has no genuine trailing edge (``chord_line_angle_deg``
      returns ``None``, e.g. a blunt root cut with no
      sharp TE at all), or
    - the leading-edge thickness check is ambiguous, or disagrees with the
      chord-line's own LE/TE assignment in a way that cannot be resolved
      by a single 180 degree correction (the thickness check itself
      returns ``None`` after applying the chord-line candidate).

    Callers must fail the slice closed, never guess.
    """
    axis_angle = chord_line_angle_deg(points, te_angle_threshold_deg)
    if axis_angle is None:
        hull = _convex_hull(points)
        if len(hull) < 3:
            raise ChordAxisUnavailableError(
                "cannot determine a chord axis: the section has no area "
                "(collinear or coincident points)"
            )
        found = _sharpest_hull_vertex(hull)
        if found is None:
            raise ChordAxisUnavailableError(
                "cannot determine a trailing edge: no hull vertex has two "
                "well-defined edges (degenerate hull)"
            )
        # split the blunt/flat-cut case by what the section actually IS. A real
        # wing with a squared-off TE fails the slice CLOSED -- shipping it
        # unrotated is a silent wrong answer, which is strictly worse than
        # refusing. A structural coupon takes the non-fatal path: it has no
        # chord axis to get wrong.
        if len(hull) >= AEROFOIL_MIN_HULL_VERTICES:
            raise BluntTrailingEdgeError(
                "no sharp trailing edge was found, so this wing's chord "
                "axis cannot be read from a sharp corner: the sharpest "
                "hull-vertex interior angle is {:.4f} deg, above the "
                "{:.4f} deg threshold for a real corner (that vertex is "
                "not necessarily the trailing edge: on a section with a "
                "squared-off trailing edge it can be the leading-edge "
                "nose), on a section with {} hull vertices (a real "
                "aerofoil, not a structural coupon). The slice is refused "
                "rather than sliced unrotated because an unrotated wing "
                "builds every internal member off the true chord axis "
                "while reporting complete success -- measured at 15 "
                "degrees of misalignment: 1495 layers ok, 0 failed, all "
                "structure wrong. Rotating the model in the Orca GUI will "
                "NOT help: a squared-off trailing edge reads the same "
                "sharpest-hull-vertex angle at every rotation, so this "
                "station itself cannot be fixed by re-orienting the "
                "mesh.".format(
                    found[1], te_angle_threshold_deg, len(hull))
            )
        # Below the aerofoil vertex threshold: a polygonal structural coupon.
        # Non-fatal: it has no chord axis to get wrong, so refusing would
        # reject every rectangular fixture for no safety gain.
        raise ChordAxisUnavailableError(
            "no genuine trailing edge: sharpest hull-vertex interior "
            "angle is {:.4f} deg, above the {:.4f} deg threshold for a "
            "real corner, on a section with only {} hull vertices -- "
            "below the {} that would make it a discretised aerofoil "
            "CURVE, so this is a plain structural coupon (a rectangle has "
            "4), not a wing with a squared-off trailing edge. Treated as "
            "UNUSABLE (theta=0, unrotated) rather than guessed, exactly "
            "as the non-fatal chord-axis case.".format(
                found[1], te_angle_threshold_deg, len(hull),
                AEROFOIL_MIN_HULL_VERTICES)
        )
    derotated = rotate_points_deg(points, -axis_angle)
    le = leading_edge_side(derotated)
    if le is None:
        raise AlignmentError(
            "leading edge is ambiguous after de-rotation to {:.4f} deg "
            "about the {}: forward-half and aft-half peak thickness are "
            "too close to call (flat plate, or a section symmetric about "
            "mid-chord)".format(axis_angle, "chord-line axis (TE = sharpest "
            "hull vertex, LE = farthest hull point from it)")
        )
    theta = axis_angle if le == "min_x" else axis_angle + 180.0
    while theta <= -180.0:
        theta += 360.0
    while theta > 180.0:
        theta -= 360.0
    return theta


def _circular_spread_deg(thetas: Sequence[float]) -> float:
    """Max pairwise circular distance (degrees, in [0, 180]) among
    ``thetas``. Plain max-min is wrong across the +-180 seam; this wraps
    each pairwise difference into [-180, 180] first."""
    if len(thetas) < 2:
        return 0.0
    worst = 0.0
    for i in range(len(thetas)):
        for j in range(i + 1, len(thetas)):
            d = (thetas[i] - thetas[j] + 180.0) % 360.0 - 180.0
            worst = max(worst, abs(d))
    return worst


# --- Sharp-trailing-edge band substitute for a blunt mid-span ---
#
# A wing with an elevon cutout squares off the trailing edge at mid-span, so
# ``resolve_theta_deg`` fails there with ``BluntTrailingEdgeError`` even though
# the outboard ~11% of span still has a genuine sharp TE. Rather than fail the
# whole wing, a caller MAY supply ``station_sampler`` (see
# ``resolve_wing_theta_deg``): when supplied and the mid-span station
# specifically raises ``BluntTrailingEdgeError`` (never for
# ``ChordAxisUnavailableError`` or a plain ``AlignmentError``), a dense band of
# interior span fractions is sampled and the longest contiguous run of
# stations with a genuinely resolvable chord axis is used to read theta
# instead, subject to the guards below. A wing whose mid-span resolves never
# uses this path: it only substitutes a station when mid-span is unreadable
# AND a caller opts in.

# Dense span-fraction sweep used to search for a sharp-TE band, one station per
# percent of span, excluding the very root/tip percent (0.02..0.98 inclusive,
# step 0.01 -- 97 stations). On the elevon wing the sharp band spans fractions
# 0.89-0.98 within this sweep (10 stations); a sweep over the full 0..1 range
# would also catch 0.99, but the outermost root/tip percent on each end is
# excluded deliberately. 10 is well above SHARP_BAND_MIN_STATIONS.
SHARP_BAND_FRAC_START = 0.02
SHARP_BAND_FRAC_END = 0.98
SHARP_BAND_FRAC_STEP = 0.01

# Minimum contiguous run length (stations) required before a sharp-TE band
# is trusted as a chord-axis substitute for a blunt mid-span. Measured on
# the elevon wing: the real sharp band is 10 stations long (0.89..0.98);
# 5 is half that, with real margin, while still ruling out a 1-2 station
# fluke reading between two blunt regions.
SHARP_BAND_MIN_STATIONS = 5

# Maximum circular spread (degrees) allowed across the accepted run. The
# elevon's real band agrees to within 0.00002 deg (a nearly untwisted tip
# region) -- 1.0 deg leaves three orders of magnitude of headroom for a
# genuinely untwisted or lightly washed-out sharp-TE region while still
# rejecting a run straddling real per-station drift.
SHARP_BAND_MAX_SPREAD_DEG = 1.0

# Maximum allowed deviation (degrees, mod 180) between the sharp-band theta and
# the convex-hull-diameter (farthest hull point pair) axis of each BLUNT
# cross-check station (mid-span plus the 0.1/0.3/0.7/0.9 stations,
# filtered to those with >= AEROFOIL_MIN_HULL_VERTICES hull vertices). Measured
# on the elevon wing's own blunt stations: 3.09-3.11 deg -- comfortably under
# this threshold, but not by a wide margin; a future corpus with a shallower
# elevon cutout should re-measure before trusting this threshold unchanged.
BLUNT_CROSSCHECK_MAX_DEG = 5.0


def _sharp_band_fracs() -> List[float]:
    n = int(round(
        (SHARP_BAND_FRAC_END - SHARP_BAND_FRAC_START) / SHARP_BAND_FRAC_STEP
    )) + 1
    return [
        round(SHARP_BAND_FRAC_START + i * SHARP_BAND_FRAC_STEP, 2)
        for i in range(n)
    ]


def _hull_diameter_angle_mod180_deg(hull: Sequence[Point]) -> float:
    """Angle (degrees, mod 180) of the farthest pair of hull points (the
    convex-hull diameter) -- a coarse, direction-agnostic gross-orientation
    signal used only as a cross-check, NOT a chord-axis estimator (see
    ``BluntTrailingEdgeError``'s docstring: a clustered-endpoint version of
    this idea has up to 2.18 deg of camber/truncation-correlated error).
    O(n^2) over hull
    vertices only, which is a small fraction of the section's raw points."""
    best_d2 = -1.0
    best_pair = (hull[0], hull[0])
    n = len(hull)
    for i in range(n):
        xi, yi = hull[i]
        for j in range(i + 1, n):
            xj, yj = hull[j]
            d2 = (xi - xj) ** 2 + (yi - yj) ** 2
            if d2 > best_d2:
                best_d2 = d2
                best_pair = (hull[i], hull[j])
    (x1, y1), (x2, y2) = best_pair
    return math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0


def _mod180_deviation_deg(a_deg: float, b_deg: float) -> float:
    """Smallest angular distance between ``a_deg`` and ``b_deg`` when both
    are considered mod 180 (an axis, not a direction)."""
    return abs(((a_deg - b_deg) + 90.0) % 180.0 - 90.0)


class _NoSharpBandError(BluntTrailingEdgeError):
    """Internal: ``_resolve_sharp_band_theta`` found NO sharp-TE band at all (no
    contiguous run of resolving stations, or the longest run is shorter than
    ``SHARP_BAND_MIN_STATIONS``). A subclass of ``BluntTrailingEdgeError`` with
    the identical message, so every caller that catches or tests for the public
    class sees exactly what it saw; ``resolve_wing_theta_deg`` alone
    distinguishes it, because only "no band exists" may fall through to the
    thin-flat reading. A band that EXISTS and then fails a guard raises the
    plain class and stays a refusal."""


def _resolve_sharp_band_theta(
    mid_span_points: Sequence[Point],
    other_station_points: Sequence[Sequence[Point]],
    station_sampler: Callable[[float], Optional[Sequence[Point]]],
    mid_span_exc: BluntTrailingEdgeError,
) -> Tuple[float, Dict]:
    """Search ``station_sampler`` over the dense sweep for the longest
    contiguous run of stations with a genuinely resolvable chord axis, and
    accept it as theta's source in place of a blunt mid-span station,
    subject to the guards documented at the constants above. Raises
    ``BluntTrailingEdgeError`` (never returns) if no band clears every
    guard -- see the module docstring section above this function.
    """
    fracs = _sharp_band_fracs()
    theta_by_frac: Dict[float, float] = {}
    resolved_flags: List[bool] = []
    for frac in fracs:
        pts = station_sampler(frac)
        if pts is None:
            resolved_flags.append(False)
            continue
        try:
            theta_by_frac[frac] = resolve_theta_deg(pts)
            resolved_flags.append(True)
        except AlignmentError:
            resolved_flags.append(False)

    runs: List[List[float]] = []
    current: List[float] = []
    for frac, ok in zip(fracs, resolved_flags):
        if ok:
            current.append(frac)
        else:
            if current:
                runs.append(current)
            current = []
    if current:
        runs.append(current)

    def _fail(reason: str, best_run: Optional[List[float]] = None,
              band_spread: Optional[float] = None,
              crosscheck: Optional[List[Dict]] = None,
              no_band: bool = False) -> None:
        raise (_NoSharpBandError if no_band else BluntTrailingEdgeError)(
            "{} No part of this wing's span has a sharp trailing edge the "
            "plugin can read its chord from; a squared-off TE reads the "
            "same at every rotation, so rotating the model will not "
            "help. {} (searched span fractions {:.2f}..{:.2f} in steps of "
            "{:.2f}; longest usable run found: {}; run spread: {}; "
            "blunt cross-check: {}).".format(
                mid_span_exc,
                reason,
                SHARP_BAND_FRAC_START, SHARP_BAND_FRAC_END, SHARP_BAND_FRAC_STEP,
                best_run if best_run is not None else "none",
                "{:.5f} deg".format(band_spread) if band_spread is not None else "n/a",
                crosscheck if crosscheck is not None else "not reached",
            )
        )

    if not runs:
        _fail("No contiguous run of resolvable stations exists at all.",
              no_band=True)

    best_len = max(len(r) for r in runs)
    candidates = [r for r in runs if len(r) == best_len]
    best_run = min(candidates, key=lambda r: min(abs(f - 0.5) for f in r))

    if len(best_run) < SHARP_BAND_MIN_STATIONS:
        _fail(
            "The longest sharp-TE run found is only {} station(s) long, "
            "below the required minimum of {}.".format(
                len(best_run), SHARP_BAND_MIN_STATIONS),
            best_run=best_run, no_band=True,
        )

    run_thetas = [theta_by_frac[f] for f in best_run]
    band_spread = _circular_spread_deg(run_thetas)
    if band_spread > SHARP_BAND_MAX_SPREAD_DEG:
        _fail(
            "The longest sharp-TE run ({} stations) disagrees by {:.5f} "
            "deg, above the {:.4f} deg spread threshold.".format(
                len(best_run), band_spread, SHARP_BAND_MAX_SPREAD_DEG),
            best_run=best_run, band_spread=band_spread,
        )

    theta = theta_by_frac[min(best_run, key=lambda f: abs(f - 0.5))]

    crosscheck: List[Dict] = []
    for pts in [mid_span_points] + list(other_station_points):
        hull = _convex_hull(pts)
        if len(hull) < AEROFOIL_MIN_HULL_VERTICES:
            continue
        derotated = rotate_points_deg(pts, -theta)
        le = leading_edge_side(derotated)
        diam_angle = _hull_diameter_angle_mod180_deg(hull)
        deviation = _mod180_deviation_deg(diam_angle, theta)
        crosscheck.append({
            "hull_vertices": len(hull),
            "leading_edge_side": le,
            "hull_diameter_deviation_deg": deviation,
        })
        if le != "min_x":
            _fail(
                "A blunt cross-check station's leading-edge side disagrees "
                "with the sharp-TE band's theta after de-rotation "
                "(got {!r}, expected 'min_x').".format(le),
                best_run=best_run, band_spread=band_spread, crosscheck=crosscheck,
            )
        if deviation > BLUNT_CROSSCHECK_MAX_DEG:
            _fail(
                "A blunt cross-check station's hull-diameter axis "
                "disagrees with the sharp-TE band's theta by {:.4f} deg, "
                "above the {:.4f} deg cross-check threshold.".format(
                    deviation, BLUNT_CROSSCHECK_MAX_DEG),
                best_run=best_run, band_spread=band_spread, crosscheck=crosscheck,
            )

    return theta, {
        "band_fracs": best_run,
        "band_spread_deg": band_spread,
        "blunt_crosscheck": crosscheck,
    }


# --- Thin trailing-edge flat as a third chord-axis source ---
#
# The ``wing_custom`` example squares the trailing edge off with a
# perpendicular flat 0.13% of the chord long (0.021 of the section thickness)
# along the whole span. ``resolve_theta_deg`` refuses it (every hull vertex is
# blunter than 60 deg: its sharpest, 75.4 deg, is the LEADING-edge nose, not a
# trailing edge), and no sharp band exists. A hull-diameter axis is not used
# because its error is camber- and truncation-correlated, up to 2.18 deg. The
# same mechanism applies to the face-midpoint axis read here, so the reading
# is accepted ONLY where its measured error is small: face/thickness <=
# THIN_FLAT_MAX_FACE_OVER_THICKNESS. Thicker flats (a cutout, a 5% truncation)
# are refused.
#
# Reachability (``resolve_wing_theta_deg``): mid-span sharp resolve first, then
# the sharp band, and the thin-flat reading ONLY when no sharp band exists
# (``_NoSharpBandError``). Never from the coupon path, never without a
# ``station_sampler``.

# Hull vertices with an interior angle below this are "corners". Measured:
# every trailing-edge face corner on the verification grid reads 80.4-125.1
# deg (wing_custom 93.9, W04 96.7-103.1, W11 100.0-100.6), so 150 sits 24.9
# deg above the widest; the smoothest tessellated surface vertices read
# 167.5 (W04) and 173.2 (wing_custom) at their sharpest, 17.5 deg above.
THIN_FLAT_STRONG_CORNER_DEG = 150.0

# Hull vertices strictly between two corners may sit this far (as a fraction of
# the corner-to-corner distance) from their chord and the path still counts as
# one straight face: a tessellator may split a flat into several collinear
# hull vertices. wing_custom's face carries two such vertices at 1.1e-4 of the
# face length (18x inside the bound); its curved lower and upper surfaces,
# corner to corner, sit >= 0.0298 off their chord (15x outside it). The
# closest curved-surface-with-intermediates arc seen anywhere on the grid is a
# short leading-edge facet at 0.0012; it can only add a second candidate, and
# a second candidate refuses (see ``_thin_flat_station``), so the bound errs
# toward refusal.
THIN_FLAT_STRAIGHT_DEV_FRAC = 0.002

# The "wing-like window": both corner angles of the one candidate face must lie
# inside it. Lower bound 80: wing_custom's leading-edge nose reads 75.2-75.4
# (4.6 deg below, so it can never be a face corner); the lowest accepted face
# corner on the grid is 80.4, and a -15 deg slanted cut, whose corner reads
# 75.5-77.7, is therefore REFUSED rather than accepted (conservative,
# reported). Upper bound 130: the highest accepted corner is 125.1 (+15 deg
# slant, 6% camber), 4.9 deg inside; a rectangle with bevelled corners reads
# 135, 5 deg outside.
THIN_FLAT_CORNER_MIN_DEG = 80.0
THIN_FLAT_CORNER_MAX_DEG = 130.0

# Face length / chord. wing_custom 0.00126 at every station (7.9x inside). The
# face/thickness bound below, on the thickest section in the grid (18%), admits
# 0.0045 (2.2x inside); a 25%-thick section would reach 0.0063 (1.6x). The
# nearest non-aerofoil that survives the corner rules, a rectangle whose long
# side is read as the "face", measured 1.94 (194x outside).
THIN_FLAT_MAX_FACE_OVER_CHORD = 0.01

# Face length / maximum thickness (HULL thickness perpendicular to the axis; on
# a concave section that over-reads thickness and so lowers the ratio, the
# permissive direction, accepted because the grid below used this very
# definition, concave and reflexed sections included). This is the accuracy
# gate: the face-midpoint axis is camber- and truncation-correlated, so it is
# accepted only while that error stays under about 0.2 deg. Worst |error|
# over the verification grid (10416 sections, NACA 4-digit t 9-18%, p
# 0.3-0.5, m 0-6%, reflexed, cusped, designed-blunt, perpendicular and +-15
# deg cuts), by face/thickness: <= 0.025 -> 0.172 deg; 0.025-0.030 -> 0.206;
# 0.030-0.035 -> 0.230; <= 0.05 -> 0.346 (cusped TE, 6% camber peaking at 50%
# chord). So 0.025 is the largest round value under the criterion, with
# 0.028 deg to spare. wing_custom measures 0.02101 at every station: inside
# by 0.0040 (16% of the bound; ratio 1.19x). It is constant along the span,
# so the margin does not erode station to station. The corpus wings W04
# (0.0253-0.0405, mid-span 0.0311) and W11 (0.0291-0.0831 outside the notch)
# sit above the bound and are refused; the bound was not chosen to include
# or exclude either.
THIN_FLAT_MAX_FACE_OVER_THICKNESS = 0.025

# Largest adjacent-station |delta theta| (deg) inside the run of thin-flat
# stations (0.02..0.98 step 0.01). Measured legitimate drift (adjacent-station
# |delta theta|): wing_custom 0.0, W04 design washout 0.040, W11 0.050, W16
# (8 deg washout) 0.080 at most. Measured step: W11's elevon notch jumps by
# 2.15 and 2.25 deg at its two edges. 0.4 sits 5.0x above the largest drift and 5.4x below
# the smaller step. In production the notch stations are refused earlier
# (face/chord 0.126, leading edge ambiguous), so this guard is a backstop
# whose only worked example is that measured step.
THIN_FLAT_MAX_ADJACENT_DELTA_DEG = 0.4


def _hull_interior_angles(hull: Sequence[Point]) -> List[float]:
    """Interior angle (degrees) at every hull vertex; 180 where a neighbouring
    edge has zero length."""
    n = len(hull)
    out: List[float] = []
    for i in range(n):
        a, b, c = hull[i - 1], hull[i], hull[(i + 1) % n]
        v1 = (a[0] - b[0], a[1] - b[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        d1 = math.hypot(*v1)
        d2 = math.hypot(*v2)
        if d1 < 1e-9 or d2 < 1e-9:
            out.append(180.0)
            continue
        cosang = (v1[0] * v2[0] + v1[1] * v2[1]) / (d1 * d2)
        out.append(math.degrees(math.acos(max(-1.0, min(1.0, cosang)))))
    return out


def _hull_max_thickness(hull: Sequence[Point]) -> float:
    """Largest extent of the convex ``hull`` along Y at any X (``hull`` already
    rotated so the chord is along +X). Exact for a convex polygon: the
    extent is piecewise linear in X, so its maximum is at a vertex X. O(n).
    This is the HULL thickness: it equals the section's thickness for a convex
    section. On a CONCAVE section (reflexed or under-cambered surface) it
    over-reads the thickness by at most the concavity depth, so it LOWERS
    face/thickness: the permissive direction. That is acceptable because the
    verification grid (concave-underside and reflexed sections included) was
    measured with this same definition through this same function, so the
    error bound in ``THIN_FLAT_MAX_FACE_OVER_THICKNESS`` already includes it."""
    n = len(hull)
    i0 = min(range(n), key=lambda k: (hull[k][0], hull[k][1]))
    x_lo = hull[i0][0]
    x_hi = max(p[0] for p in hull)
    tol = 1e-9 * max(x_hi - x_lo, 1e-12)

    def _chain(step: int) -> List[Point]:
        chain = [hull[i0]]
        k = (i0 + step) % n
        while k != i0 and hull[k][0] >= chain[-1][0] - tol:
            chain.append(hull[k])
            k = (k + step) % n
        return chain

    lower, upper = _chain(1), _chain(-1)
    lxs = [p[0] for p in lower]
    uxs = [p[0] for p in upper]

    def _y_at(chain: List[Point], xs: List[float], x: float) -> float:
        j = bisect.bisect_right(xs, x)
        j = max(1, min(len(chain) - 1, j))
        (xa, ya), (xb, yb) = chain[j - 1], chain[j]
        if xb - xa < 1e-12:
            return ya
        return ya + (yb - ya) * (x - xa) / (xb - xa)

    best = 0.0
    for p in hull:
        x = p[0]
        if x <= x_lo + tol or x >= x_hi - tol:
            continue
        best = max(best, abs(_y_at(upper, uxs, x) - _y_at(lower, lxs, x)))
    return best


def _thin_flat_station(
    points: Sequence[Point],
) -> Tuple[Optional[Dict], Optional[Dict]]:
    """Read the chord axis of ONE section from its thin trailing-edge flat.

    Returns ``(info, None)`` when every per-station guard holds, else
    ``(None, refusal)`` with ``refusal = {"guard", "value", "bound",
    "detail"}`` naming the first guard that failed and what it measured.
    Never raises on geometry (callers decide what a refusal means).

    The section's TE face is defined exactly, and any ambiguity refuses:

    1. the hull has >= ``AEROFOIL_MIN_HULL_VERTICES`` vertices and at least
       two corners (interior angle < ``THIN_FLAT_STRONG_CORNER_DEG``). There
       is deliberately NO upper bound on the corner count: the exactly-one
       rule in step 3 already refuses every polygon measured, so a cap would
       only refuse coarsely tessellated leading edges;
    2. ALL pairs of cyclically consecutive corners are enumerated (both arcs
       when there are exactly two); a pair is a face candidate when the hull
       path between its corners is straight
       (``THIN_FLAT_STRAIGHT_DEV_FRAC``);
    3. exactly ONE candidate has both corner angles inside
       [``THIN_FLAT_CORNER_MIN_DEG``, ``THIN_FLAT_CORNER_MAX_DEG``]. Zero or
       several refuse; the best one is never picked;
    4. the axis runs from the leading edge (the hull vertex farthest from the
       face midpoint) to the face midpoint; face/chord and face/thickness
       must be under their bounds;
    5. ``leading_edge_side`` on the de-rotated points must say "min_x": None
       (ambiguous) refuses as ``leading_edge_side``; "max_x" (thickness peaks
       on the flat's own side, so the flat is at the thick end) refuses as
       ``face_at_thick_end``. No +180 correction is applied here.
    """
    def _refuse(guard, value, bound, detail):
        return None, {"guard": guard, "value": value, "bound": bound,
                      "detail": detail}

    hull = _convex_hull(points)
    n = len(hull)
    if n < AEROFOIL_MIN_HULL_VERTICES:
        return _refuse("hull_vertices", n, AEROFOIL_MIN_HULL_VERTICES,
                       "{} hull vertices, below the {} of a discretised "
                       "aerofoil".format(n, AEROFOIL_MIN_HULL_VERTICES))
    ang = _hull_interior_angles(hull)
    strong = [i for i in range(n) if ang[i] < THIN_FLAT_STRONG_CORNER_DEG]
    if len(strong) < 2:
        return _refuse(
            "strong_corners", len(strong), ">= 2",
            "{} hull corner(s) below {:.0f} deg; a trailing-edge flat needs "
            "two".format(len(strong), THIN_FLAT_STRONG_CORNER_DEG))

    candidates: List[Tuple[int, int]] = []
    m = len(strong)
    for t in range(m):
        i, j = strong[t], strong[(t + 1) % m]
        d = math.hypot(hull[j][0] - hull[i][0], hull[j][1] - hull[i][1])
        if i == j or d < 1e-9:
            continue
        straight = True
        k = (i + 1) % n
        while k != j:
            p = hull[k]
            dev = abs((hull[j][0] - hull[i][0]) * (hull[i][1] - p[1])
                      - (hull[i][0] - p[0]) * (hull[j][1] - hull[i][1])) / d
            if dev > THIN_FLAT_STRAIGHT_DEV_FRAC * d:
                straight = False
                break
            k = (k + 1) % n
        if straight:
            candidates.append((i, j))
    in_window = [
        (i, j) for i, j in candidates
        if THIN_FLAT_CORNER_MIN_DEG <= ang[i] <= THIN_FLAT_CORNER_MAX_DEG
        and THIN_FLAT_CORNER_MIN_DEG <= ang[j] <= THIN_FLAT_CORNER_MAX_DEG
    ]
    if len(in_window) != 1:
        return _refuse(
            "te_face_candidates", len(in_window), "exactly 1",
            "{} straight corner-to-corner face(s) with both corner angles in "
            "[{:.0f}, {:.0f}] deg ({} straight face(s) before the angle "
            "window; corner angles {})".format(
                len(in_window), THIN_FLAT_CORNER_MIN_DEG,
                THIN_FLAT_CORNER_MAX_DEG, len(candidates),
                ", ".join("{:.1f}".format(ang[k]) for k in strong)))

    i, j = in_window[0]
    pi, pj = hull[i], hull[j]
    mid = ((pi[0] + pj[0]) / 2.0, (pi[1] + pj[1]) / 2.0)
    le = max(hull, key=lambda p: (p[0] - mid[0]) ** 2 + (p[1] - mid[1]) ** 2)
    face = math.hypot(pj[0] - pi[0], pj[1] - pi[1])
    chord = math.hypot(le[0] - mid[0], le[1] - mid[1])
    if chord < 1e-9:
        return _refuse("chord_length", chord, "> 0", "degenerate chord")
    axis = math.degrees(math.atan2(mid[1] - le[1], mid[0] - le[0]))
    face_over_chord = face / chord
    if face_over_chord > THIN_FLAT_MAX_FACE_OVER_CHORD:
        return _refuse(
            "face_over_chord", face_over_chord, THIN_FLAT_MAX_FACE_OVER_CHORD,
            "the trailing-edge face is {:.4f} of the chord".format(
                face_over_chord))
    thickness = _hull_max_thickness(rotate_points_deg(hull, -axis))
    if thickness <= 0.0:
        return _refuse("thickness", thickness, "> 0", "degenerate thickness")
    face_over_thickness = face / thickness
    if face_over_thickness > THIN_FLAT_MAX_FACE_OVER_THICKNESS:
        return _refuse(
            "face_over_thickness", face_over_thickness,
            THIN_FLAT_MAX_FACE_OVER_THICKNESS,
            "the trailing-edge face is {:.4f} of the section thickness; "
            "this axis reading is only trusted for a thin flat".format(
                face_over_thickness))
    side = leading_edge_side(rotate_points_deg(points, -axis))
    if side is None:
        return _refuse("leading_edge_side", None, "min_x or max_x",
                       "the leading edge is ambiguous after de-rotation to "
                       "{:.4f} deg".format(axis))
    if side == "max_x":
        # The section's thickness peaks on the face's own side of mid-chord:
        # the one qualifying flat sits at the THICK end, so it is not a
        # trailing-edge flat and the axis direction would be a guess. Refuse;
        # never flip by 180 deg as ``resolve_theta_deg`` does on the sharp path.
        # The axis is directed leading edge -> face, so on every grid case and
        # corpus wing ``side`` was "min_x".
        return _refuse(
            "face_at_thick_end", side, "min_x",
            "the thickness peaks on the same side of mid-chord as the flat "
            "(leading_edge_side is {!r} after de-rotation to {:.4f} deg), so "
            "the flat is at the thick end, not a trailing edge".format(
                side, axis))
    theta = axis
    return {
        "theta_deg": theta,
        "axis_deg": axis,
        "le_side": side,
        "hull_vertices": n,
        "strong_corners": m,
        "corner_angles_deg": [ang[i], ang[j]],
        "face_mm": face,
        "chord_mm": chord,
        "thickness_mm": thickness,
        "face_over_chord": face_over_chord,
        "face_over_thickness": face_over_thickness,
    }, None


def _resolve_thin_flat_theta(
    mid_span_points: Sequence[Point],
    other_station_points: Sequence[Sequence[Point]],
    station_sampler: Callable[[float], Optional[Sequence[Point]]],
    no_band_exc: BluntTrailingEdgeError,
) -> Tuple[float, Dict]:
    """Wing-level thin-flat reading, reached only when no sharp band exists.

    theta is the MID-SPAN station's own thin-flat reading (one station, never
    averaged), accepted only if every wing-level guard holds:

    - mid-span itself resolves by thin flat;
    - it sits in a contiguous run of thin-flat-resolving dense stations
      (0.02..0.98 step 0.01) at least ``SHARP_BAND_MIN_STATIONS`` long;
    - adjacent-station |delta theta| inside that run is at most
      ``THIN_FLAT_MAX_ADJACENT_DELTA_DEG``;
    - the standard stations' leading-edge side and hull-diameter axis agree
      with theta, exactly as for the sharp band (``BLUNT_CROSSCHECK_MAX_DEG``).

    The 8 deg spread check runs afterwards in ``resolve_wing_theta_deg`` as on
    every path. Raises ``BluntTrailingEdgeError`` naming the guard, its value
    and its bound.
    """
    def _fail(guard: str, value, bound, detail: str) -> None:
        if isinstance(value, float):
            value_text = "{:.5f}".format(value)
        else:
            value_text = str(value)
        raise BluntTrailingEdgeError(
            "{} A thin-trailing-edge-flat reading (a short flat, at most "
            "{:g} of the section thickness) was also tried and refused: "
            "{} guard, measured {} against a bound of {}. {}".format(
                no_band_exc, THIN_FLAT_MAX_FACE_OVER_THICKNESS, guard,
                value_text, bound, detail))

    fracs = _sharp_band_fracs()
    mid_idx = min(range(len(fracs)), key=lambda k: abs(fracs[k] - 0.5))
    results: List[Optional[Dict]] = []
    mid_refusal: Optional[Dict] = None
    for k, frac in enumerate(fracs):
        pts = mid_span_points if k == mid_idx else station_sampler(frac)
        if pts is None:
            results.append(None)
            continue
        info, refusal = _thin_flat_station(pts)
        results.append(info)
        if k == mid_idx:
            mid_refusal = refusal
    mid_info = results[mid_idx]
    if mid_info is None:
        r = mid_refusal or {"guard": "mid_span_section", "value": "missing",
                            "bound": "present", "detail": ""}
        _fail("mid-span station / " + r["guard"], r["value"], r["bound"],
              "At mid-span: " + r["detail"] + ".")

    lo = hi = mid_idx
    while lo > 0 and results[lo - 1] is not None:
        lo -= 1
    while hi < len(fracs) - 1 and results[hi + 1] is not None:
        hi += 1
    run = list(range(lo, hi + 1))
    if len(run) < SHARP_BAND_MIN_STATIONS:
        _fail("thin-flat run length", len(run),
              ">= {}".format(SHARP_BAND_MIN_STATIONS),
              "Only {} contiguous station(s) around mid-span read a thin "
              "flat.".format(len(run)))
    run_thetas = [results[k]["theta_deg"] for k in run]
    deltas = [
        abs((run_thetas[k + 1] - run_thetas[k] + 180.0) % 360.0 - 180.0)
        for k in range(len(run_thetas) - 1)
    ]
    delta_max = max(deltas) if deltas else 0.0
    if delta_max > THIN_FLAT_MAX_ADJACENT_DELTA_DEG:
        at = fracs[run[deltas.index(delta_max)]]
        _fail("thin-flat adjacent-station delta", delta_max,
              "{:.2f} deg".format(THIN_FLAT_MAX_ADJACENT_DELTA_DEG),
              "Theta steps by that much between span fractions {:.2f} and "
              "{:.2f}; a washed-out wing drifts smoothly.".format(
                  at, at + SHARP_BAND_FRAC_STEP))

    theta = mid_info["theta_deg"]
    crosscheck: List[Dict] = []
    for pts in [mid_span_points] + list(other_station_points):
        hull = _convex_hull(pts)
        if len(hull) < AEROFOIL_MIN_HULL_VERTICES:
            continue
        derotated = rotate_points_deg(pts, -theta)
        le = leading_edge_side(derotated)
        deviation = _mod180_deviation_deg(
            _hull_diameter_angle_mod180_deg(hull), theta)
        crosscheck.append({
            "hull_vertices": len(hull),
            "leading_edge_side": le,
            "hull_diameter_deviation_deg": deviation,
        })
        if le != "min_x":
            _fail("thin-flat standard-station leading-edge side", repr(le),
                  "'min_x'",
                  "A standard cross-check station's leading-edge side "
                  "disagrees with the thin-flat theta after de-rotation.")
        if deviation > BLUNT_CROSSCHECK_MAX_DEG:
            _fail("thin-flat standard-station hull-diameter", deviation,
                  "{:.1f} deg".format(BLUNT_CROSSCHECK_MAX_DEG),
                  "A standard cross-check station's hull-diameter axis "
                  "disagrees with the thin-flat theta.")

    return theta, {
        "thin_te_flat": {
            "run_fracs": [fracs[k] for k in run],
            "run_stations": len(run),
            "adjacent_delta_max_deg": delta_max,
            "adjacent_delta_bound_deg": THIN_FLAT_MAX_ADJACENT_DELTA_DEG,
            "no_sharp_band": (
                "no contiguous run of resolvable stations"
                if "No contiguous run" in str(no_band_exc)
                else "longest sharp run below the required minimum"),
            "mid_span": dict(mid_info),
        },
        "blunt_crosscheck": crosscheck,
    }


def resolve_wing_theta_deg(
    mid_span_points: Sequence[Point],
    other_station_points: Sequence[Sequence[Point]] = (),
    spread_threshold_deg: float = 8.0,
    station_sampler: Optional[Callable[[float], Optional[Sequence[Point]]]] = None,
    provenance: Optional[Dict] = None,
) -> Tuple[float, List[float], float, List[Tuple[int, str]]]:
    """Resolve the ONE global theta for a whole wing. The operative angle is
    measured at a single representative station, the mid-span mesh
    cross-section ``mid_span_points``, and is NOT averaged or blended across
    the span.

    ``other_station_points`` (interior, e.g. 10-90% span) are sampled ONLY
    to check that per-station chord angle does not vary beyond
    ``spread_threshold_deg``. A real washout wing's per-station theta DRIFTS
    WITH SPAN BY DESIGN (measured on the reference fixture
    ``tests/fixtures/wing/naca2412_wing_r150_t90_s400_w3.stl``: theta rises
    ~0.6 deg per 0.2 of span fraction, an intentional twist, not
    misalignment noise). Averaging or taking the median across stations
    would blend that per-layer washout into the GLOBAL rotation and leave
    the wrong RESIDUAL for the per-layer washout handling in ``frame.py``
    (``estimate_rotation_deg``/``check_rotation_envelope``) to correct. The
    mid-span station alone is the global placement angle; everything else is
    residual, handled in ``frame.py``.

    A station with NO genuine trailing edge (a blunt, flat-cut station --
    ``chord_line_angle_deg``/``resolve_theta_deg`` raises ``AlignmentError``
    for it) is handled differently depending on which station it is:

    - the MID-SPAN station: this function PROPAGATES whatever
      ``resolve_theta_deg`` raises for it, unmodified -- it never guesses a
      global rotation from a degenerate section. That propagation fails
      closed only for plain ``AlignmentError``; a
      ``ChordAxisUnavailableError`` mid-span station propagates the same
      way OUT OF THIS FUNCTION, but its real callers (``capability.py``)
      catch it specifically and treat it as NON-fatal (theta stays 0, the
      run continues -- see ``ChordAxisUnavailableError``'s own docstring).
      This function does not itself decide fatal-vs-non-fatal; it only
      decides whether to substitute a sharp-TE band. When the mid-span
      station raises ``BluntTrailingEdgeError`` AND a caller supplies
      ``station_sampler``, a substitute band is searched (see that
      parameter's documentation below), subject to its own guards; the
      error still propagates (fails closed) if no band clears them.
    - any OTHER (spread-check-only) station: EXCLUDED from the spread
      check and reported in ``excluded_stations``, rather than failing the
      whole run. A root station cut flat against a fuselage is a
      legitimate, expected shape (measured on a real wing STL: root stations
      at 2%/10% span read a 174 deg sharpest-hull-vertex angle, no real TE
      at all), not a defect in the mesh; it simply cannot vote on the
      spread check.

    Returns ``(theta_deg, all_station_theta_deg, spread_deg,
    excluded_stations)``. ``all_station_theta_deg`` is
    ``[mid_span_theta] + [theta for each USABLE entry of
    other_station_points]`` (mid-span first, always present).
    ``excluded_stations`` is ``[(index_into_other_station_points,
    reason_str), ...]`` for stations that raised ``AlignmentError`` and
    were excluded. ``spread_deg`` is the worst pairwise circular distance
    among the USABLE thetas only.

    Raises ``AlignmentError`` if the mid-span theta cannot be resolved, or
    if ``spread_deg`` (over the usable stations) exceeds
    ``spread_threshold_deg`` -- design washout is 2-6 degrees; a spread
    beyond that (default threshold 8 degrees, one interval past the top of
    the design range) means real per-station twist wider than the washout
    envelope, not noise around one consistent placement angle, and a
    whole-wing de-rotation is not a valid fix for it.

    ``station_sampler`` (optional; ``span_frac -> points at that span
    fraction, or None``) lets a caller opt into a SHARP-TRAILING-EDGE-BAND
    substitute for a blunt mid-span station -- for a wing with an elevon
    cutout that squares off the trailing edge at mid-span while the outboard
    ~11% of span keeps a genuine sharp TE. This ONLY activates when:

    - ``station_sampler`` is supplied, AND
    - the MID-SPAN station specifically raises ``BluntTrailingEdgeError``
      (never ``ChordAxisUnavailableError``, so the coupon path is
      unaffected, and never a plain ``AlignmentError``, which is not this
      failure mode at all).

    On activation, ``_resolve_sharp_band_theta`` searches a dense span sweep
    (``SHARP_BAND_FRAC_START``.``SHARP_BAND_FRAC_END``, step
    ``SHARP_BAND_FRAC_STEP``) for the longest contiguous run of stations with a
    genuinely resolvable chord axis, and substitutes the theta of the run's
    station nearest span 0.5 (one station, never averaged) -- subject to guards documented at the module-level constants
    (``SHARP_BAND_MIN_STATIONS``, ``SHARP_BAND_MAX_SPREAD_DEG``,
    ``BLUNT_CROSSCHECK_MAX_DEG``). If sampler is ``None``, or mid-span resolves
    normally, or mid-span raises anything other than
    ``BluntTrailingEdgeError``, only the mid-span theta is used. If the band
    search fails any guard, this function still raises ``BluntTrailingEdgeError`` -- the wing is still refused, with no
    rotate-the-model advice (a squared-off TE reads the same at every
    rotation).

    ``provenance`` (optional; mutated in place, never read) records how
    theta was derived: ``{"theta_source": "mid_span" | "sharp_te_band" |
    "thin_te_flat", "band_fracs": [...] | None, "band_spread_deg": float |
    None, "blunt_crosscheck": [...] | None, "thin_te_flat": {...} | None}``.

    When the mid-span station raises ``BluntTrailingEdgeError``, the sampler is
    supplied, and NO sharp band exists (no run at all, or the longest run is
    shorter than ``SHARP_BAND_MIN_STATIONS``), a third source is tried: the
    mid-span station's own thin trailing-edge flat
    (``_resolve_thin_flat_theta``; ``theta_source == "thin_te_flat"``). A sharp
    band that exists and then fails a guard is still a refusal. On the
    thin-flat path a standard station that the sharp reading refuses may vote
    through its own thin flat; on every other path it is excluded.
    """
    provenance_out: Dict = {
        "theta_source": "mid_span",
        "band_fracs": None,
        "band_spread_deg": None,
        "blunt_crosscheck": None,
        "thin_te_flat": None,
    }
    try:
        theta = resolve_theta_deg(mid_span_points)  # authoritative: propagates/fails closed
    except BluntTrailingEdgeError as exc:
        if station_sampler is None:
            raise
        try:
            theta, band_info = _resolve_sharp_band_theta(
                mid_span_points, other_station_points, station_sampler, exc)
            provenance_out["theta_source"] = "sharp_te_band"
            provenance_out.update(band_info)
        except _NoSharpBandError as no_band:
            # only when NO sharp band exists. A band that exists and then fails
            # a guard raised the plain class above and propagates.
            theta, flat_info = _resolve_thin_flat_theta(
                mid_span_points, other_station_points, station_sampler,
                no_band)
            provenance_out["theta_source"] = "thin_te_flat"
            provenance_out.update(flat_info)
    thetas = [theta]
    excluded_stations: List[Tuple[int, str]] = []
    for i, pts in enumerate(other_station_points):
        try:
            thetas.append(resolve_theta_deg(pts))
        except AlignmentError as exc:
            if (provenance_out["theta_source"] == "thin_te_flat"
                    and isinstance(exc, BluntTrailingEdgeError)):
                # thin-flat path ONLY: a standard station the sharp reading
                # refused may still vote through its own thin flat. Every other
                # path excludes it.
                info, refusal = _thin_flat_station(pts)
                if info is not None:
                    thetas.append(info["theta_deg"])
                    continue
                excluded_stations.append((i, "{} Thin-flat reading also "
                    "refused: {} guard, measured {} against {}.".format(
                        exc, refusal["guard"], refusal["value"],
                        refusal["bound"])))
                continue
            excluded_stations.append((i, str(exc)))
    spread = _circular_spread_deg(thetas)
    if spread > spread_threshold_deg:
        raise AlignmentError(
            "per-station chord angle disagrees by {:.4f} deg across {} "
            "usable station(s) (stations: {}), above the {:.4f} deg spread "
            "threshold. This is real washout wider than this project's "
            "design range (2-6 deg), not a single mesh-placement angle -- "
            "auto-rotation is not a valid fix; rotate the model in the "
            "Orca GUI so its chord is closer to constant, or reduce "
            "washout in the source CAD.".format(
                spread, len(thetas),
                ", ".join("{:.4f}".format(t) for t in thetas),
                spread_threshold_deg,
            )
        )
    if provenance is not None:
        provenance.update(provenance_out)
    return theta, thetas, spread, excluded_stations


# ---------------------------------------------------------------------------
# Mesh-alignment guard decision, kept separate so it can be replayed offline.
# ---------------------------------------------------------------------------

BBOX_TERMS: Tuple[str, str, str, str] = ("min_x", "max_x", "min_y", "max_y")
CENTROID_TERMS: Tuple[str, str] = ("centroid_x", "centroid_y")


def decide_alignment(deltas, effective, near_top_extremum, tip_alignment_skip_enabled):
    """Pure re-implementation of ``capability.py``'s ``_check_mesh_alignment``
    clauses 2-4 -- the four-way branch that turns six per-term deltas (and
    their per-term effective tolerances) into pass / skip / one of two distinct
    refusal shapes. Extracted so ``tools/replay_host_sections.py`` can
    reproduce the exact decision offline, from a recorded sidecar, without
    importing ``capability.py`` (which requires the ``orca`` host bundle to
    even parse).

    No message formatting, no ``TopologyError``, no layer index/Z -- those
    stay in ``capability.py``, which has the context (index, print_z, the
    sensitivity-note suffix, ``band_mm``/``z_hi``) to build an actionable
    message. This function owns only the boolean decision, so a change to
    the decision logic cannot land in one call site without the other.

    ``deltas``/``effective``: dicts keyed by the six terms
    (``BBOX_TERMS + CENTROID_TERMS``), absolute mm.

    Returns a dict:
        {"outcome": "pass"} -- every term is within its effective tolerance.
        {"outcome": "raise_bbox", "worst_term", "worst_delta",
         "effective_tolerance"} -- Clause 2: an unconditional bbox-term
         refusal, regardless of extremum or ``tip_alignment_skip_enabled``.
        {"outcome": "raise_centroid_off_top", ...} -- Clause 4: a centroid
         term is over tolerance and this layer is NOT near the mesh's own
         top extremum -- evaluated at full strictness either way.
        {"outcome": "raise_centroid_near_top_skip_disabled", ...} --
         Clause 3: near the top extremum, only a centroid term disagrees,
         but ``tip_alignment_skip_enabled`` is False.
        {"outcome": "skip", ...} -- near the top extremum, only a centroid
         term disagrees, and ``tip_alignment_skip_enabled`` is True: the
         caller must leave the layer unmutated rather than fail the slice.
    """
    bbox_over = [t for t in BBOX_TERMS if deltas[t] > effective[t]]
    if bbox_over:
        worst = max(bbox_over, key=deltas.get)
        return {
            "outcome": "raise_bbox",
            "worst_term": worst,
            "worst_delta": deltas[worst],
            "effective_tolerance": effective[worst],
        }

    centroid_over = [t for t in CENTROID_TERMS if deltas[t] > effective[t]]
    if not centroid_over:
        return {"outcome": "pass"}

    worst = max(centroid_over, key=deltas.get)
    result = {
        "worst_term": worst,
        "worst_delta": deltas[worst],
        "effective_tolerance": effective[worst],
    }
    if not near_top_extremum:
        result["outcome"] = "raise_centroid_off_top"
        return result
    if not tip_alignment_skip_enabled:
        result["outcome"] = "raise_centroid_near_top_skip_disabled"
        return result
    result["outcome"] = "skip"
    return result


# ---------------------------------------------------------------------------
# Emulation of the host's ``slice_closing_radius`` closing on the mesh side.
# ---------------------------------------------------------------------------
#
# The host closes every slice BEFORE the posSlice hook: libslic3r's
# ``make_expolygons`` (TriangleMeshSlicer.cpp:1738-1832, called at :2031;
# radius set at PrintObjectSlice.cpp:139) computes
# ``offset2_ex(union_ex(loops), +r, -r)`` -- grow by r, then shrink by r, a
# closing -- with the ClipperUtils defaults ``jtMiter``, miter limit 3
# (ClipperUtils.hpp:19,27). One call over the whole slice, so two islands
# closer than 2r merge into one contour. CAVEAT: those citations are from the
# exp-orca 2.3.2 source; the pinned nightly (550e234a) source is not on disk.
#
# The mesh sectioner does not close, so a pair of islands the host bridged
# (layer 1098: a 0.0218 mm neck) reads as ONE contour on the host side and TWO
# loops on the mesh side, and the guard compared the host's merged outline to
# the larger island alone. ``close_outer_loops`` applies the same closing to
# the mesh's outer loops so the comparison is like-for-like. Holes are not part of it: the guard compares OUTER contours only,
# and a hole cannot change an outer contour under a closing.
#
# Clipper offsets each polygon independently and unions the results, so
# "grow each outer loop, union, shrink" IS the host's operation for the
# disjoint outer loops a section yields.

CLOSING_RADIUS_KEY = "slice_closing_radius"


class ClosingRadiusError(AlignmentError):
    """The live ``slice_closing_radius`` is missing or unparseable. Fail
    closed: it must never default to 0.049 or to no closing."""


def parse_closing_radius_mm(raw) -> float:
    """``slice_closing_radius`` from the captured print config, in mm.
    Missing/unparseable/negative/non-finite raises ``ClosingRadiusError``
    naming the key."""
    try:
        if raw is None or isinstance(raw, bool):
            raise ValueError("missing")
        value = float(raw)
    except (TypeError, ValueError):
        raise ClosingRadiusError(
            "print config key {!r} is missing or unparseable (got {!r}); it "
            "is required to emulate the host's slice closing in the mesh "
            "alignment check, and is never defaulted.".format(
                CLOSING_RADIUS_KEY, raw
            )
        )
    if not math.isfinite(value) or value < 0.0:
        raise ClosingRadiusError(
            "print config key {!r} resolved to {!r}; expected a finite "
            "non-negative radius in mm.".format(CLOSING_RADIUS_KEY, raw)
        )
    return value


def _ring_signed_area(points) -> float:
    n = len(points)
    area = 0.0
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return area / 2.0


class ClosedLoop:
    """Duck-typed stand-in for ``mesh_section.Loop`` for a closed outer
    contour: the attributes the alignment guard reads."""

    is_interior = False

    def __init__(self, points):
        self.points = tuple((float(x), float(y)) for x, y in points)
        n = len(self.points)
        self.signed_area = _ring_signed_area(self.points)
        if abs(self.signed_area) < 1e-12:
            self.centroid = (
                sum(p[0] for p in self.points) / n,
                sum(p[1] for p in self.points) / n,
            )
        else:
            cx = cy = 0.0
            for i in range(n):
                x1, y1 = self.points[i]
                x2, y2 = self.points[(i + 1) % n]
                cross = x1 * y2 - x2 * y1
                cx += (x1 + x2) * cross
                cy += (y1 + y2) * cross
            f = 1.0 / (6.0 * self.signed_area)
            self.centroid = (cx * f, cy * f)
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        self.bbox = (min(xs), min(ys), max(xs), max(ys))
        self.equivalent_diameter_mm = 2.0 * math.sqrt(
            abs(self.signed_area) / math.pi
        )


class ClosedSection:
    """Result of ``close_outer_loops``."""

    def __init__(self, radius_mm, outer, loops_before, area_before_mm2, applied=True):
        self.radius_mm = radius_mm
        self.applied = applied
        self.outer = outer
        self.loops_before = loops_before
        self.loops_after = len(outer)
        self.area_before_mm2 = area_before_mm2
        self.area_after_mm2 = sum(abs(_ring_signed_area(lp.points)) for lp in outer)

    def as_dict(self):
        return {
            "radius_mm": self.radius_mm,
            "applied": self.applied,
            "loops_before": self.loops_before,
            "loops_after": self.loops_after,
            "area_before_mm2": round(self.area_before_mm2, 4),
            "area_after_mm2": round(self.area_after_mm2, 4),
        }


def close_outer_loops(ops, outer_loops, radius_mm: float,
                      skip_single_loop: bool = False) -> ClosedSection:
    """Closing (grow +r, union, shrink -r; mitre joins via ``ops``) over the
    mesh section's OUTER loops. ``outer_loops``: objects with ``.points`` (mm
    ring) -- the non-interior ``mesh_section.Loop`` list. Returns the closed
    outer contours as ``ClosedLoop`` in ``ClosedSection.outer``.

    ``radius_mm == 0`` performs no closing (the loops pass through).

    ``skip_single_loop=True`` (what the plugin passes): a section with exactly
    ONE outer loop is returned untouched (``applied=False``, the original loop
    objects). Closing a lone loop cannot change loop count and, measured over
    4341 compared layers of four real in-host runs, moved no alignment
    term by more than 3e-6 mm; only a merge of separate loops can matter.
    Skipping it also spares the boolean backend a per-layer offset pair.
    ``False`` (the default) closes every section."""
    outer_loops = list(outer_loops)
    area_before = sum(abs(_ring_signed_area(lp.points)) for lp in outer_loops)
    if not outer_loops:
        return ClosedSection(radius_mm, [], 0, 0.0)
    if skip_single_loop and len(outer_loops) == 1:
        return ClosedSection(
            radius_mm, list(outer_loops), 1, area_before, applied=False)
    if radius_mm <= 0.0:
        return ClosedSection(
            radius_mm,
            [ClosedLoop(lp.points) for lp in outer_loops],
            len(outer_loops),
            area_before,
        )
    grown = []
    for lp in outer_loops:
        pts = list(lp.points)
        if _ring_signed_area(pts) < 0.0:
            pts.reverse()
        grown.extend(ops.offset(ops.polygon_from_points(pts), radius_mm))
    merged = grown if len(grown) <= 1 else list(ops.union(grown))
    closed = []
    for poly in merged:
        closed.extend(ops.offset(poly, -radius_mm))
    contours = []
    for poly in closed:
        pts = ops.contour_points_mm(poly)
        if len(pts) >= 3:
            contours.append(ClosedLoop(pts))
    return ClosedSection(radius_mm, contours, len(outer_loops), area_before)


def loop_count_mismatch_note(loops_before: int, loops_after: int, radius_mm: float) -> str:
    """Suffix for the guard's refusal when the host reports ONE outer contour
    but the mesh section still has several after emulating the host's closing.
    Empty when the mesh section is a single loop."""
    if loops_after <= 1:
        return ""
    return (
        " Loop-count mismatch: the host reports 1 outer contour for this "
        "layer, but the mesh section still has {} outer loops after "
        "emulating slice_closing_radius={} mm ({} before closing); the guard "
        "compares the largest.".format(loops_after, radius_mm, loops_before)
    )


def alignment_terms(loop) -> Dict[str, float]:
    """The six comparison terms of one outer loop-like object."""
    mnx, mny, mxx, mxy = loop.bbox
    cx, cy = loop.centroid
    return {
        "min_x": mnx, "max_x": mxx, "min_y": mny, "max_y": mxy,
        "centroid_x": cx, "centroid_y": cy,
    }
