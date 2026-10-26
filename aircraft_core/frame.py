"""``LayerFrame``: a normalized, axis-convention view of one sliced layer.

Isolates "where is the web" (this module) from "how is it cut" (``slot.py``).
The chord axis convention is fixed: **X is chord, Y is thickness**. A section
whose chord is not along X is rotated into this frame first (see
``alignment.py``); this module does not attempt an arbitrary chord direction.

LOCAL SURFACES. The +Y/-Y extremes are not read off the polygon's bounding
box: that is exact for a rectangle but wrong for a cambered airfoil, whose
global ``min_y`` is its single lowest point (usually near the trailing edge),
not the local lower surface under a web placed elsewhere along the chord.
``local_span`` measures the LOCAL upper/lower surface at one chord station by
intersecting the section with a thin axis-aligned strip there.
``web_centerline`` always goes through ``local_span`` -- a rectangle's local
span equals its bounding-box span (the strip clipped to a convex rectangle
returns exactly the rectangle's own Y extent), so one code path is correct for
both the coupon and a cambered section; there is no separate bounding-box
fallback to keep in sync.

WORST CASE ACROSS A SPAN. A single-station ``local_span`` probe at the web
centreline is not enough when the thing being measured must hold across the
slot's WHOLE WIDTH. If placement used only the centreline station while
``validate.py`` re-measured the slot at the removed region's actual X centre,
the two stations could differ (the removed region's true bounding box need not
be exactly symmetric about the centreline once a real Clipper backend is
involved); on a curved lower surface the two readings disagree, and a
legitimately generated slot could fail its own validator.
``local_lower_worst_case``/``local_upper_worst_case`` close this by measuring
the worst case across the slot's full width in BOTH places, over the same
span, so placement and validation measure the same thing and cannot disagree.

ROTATION ENVELOPE. ``local_span`` probes along WORLD Y. On a section rotated by
``theta`` about its own chord axis (washout), the true thickness is measured
PERPENDICULAR to the chord, not vertically, so a world-Y probe overstates
thickness by ``1/cos(theta)`` -- and the error is OPTIMISTIC: the code believes
more skin survives than actually does. (Station placement from the bounding box
stays accurate: 0.004 mm at 6 degrees on the twist fixture.)

``estimate_rotation_deg`` recovers ``theta`` cheaply from the section's own
geometry (see its docstring for the method and its measured accuracy).
``LayerFrame.rotation_deg`` computes it once per layer at
``from_polygon()``. Two things then happen with that angle:

1. Below ``DEFAULT_MAX_ROTATION_DEG`` (see that constant's own comment for
   the chosen value and why), ``corrected_skin_margin_mm`` inflates the
   Y-space margin requirement by ``1/cos(theta)`` -- the algebraic inverse
   of the overstatement, so a check written against the corrected margin is
   honest about the TRUE (chord-perpendicular) skin actually required,
   without needing a second, rotated probe direction. This is applied to
   ``web_centerline`` here and, independently, to ``generator.py``'s own
   margin arithmetic (both measure worst-case Y-extents the same way, so
   both need the same correction to keep from disagreeing with each other).
2. Above the threshold, ``check_rotation_envelope`` fails closed rather
   than trust the correction at an angle it was never validated for -- see
   that function's docstring for why 1/cos(theta) alone is not enough
   reassurance there.

Rotating the probe strip itself to run parallel to the true chord is not
done: it is a larger rework than the cheaper correction above, which already
covers the realistic washout range (2-6 degrees) and well beyond it. It would
be the correct fix if a wing ever needs washout beyond what the threshold
allows.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .ops import GeometryOps

# Width of the measurement strip ``web_centerline`` probes the section with,
# in millimetres. Thin enough to approximate a single chord station (the
# strip's own width is not part of what "local thickness at a station"
# means), wide enough that a scaled-integer backend does not round it to a
# degenerate zero-width clip (see OrcaGeometryOps._to_scaled: at the normal
# 1e-6 mm/unit scaling factor this is still 10 000 scaled units).
DEFAULT_PROBE_WIDTH_MM = 0.01

# X padding, in millimetres, applied beyond a dilated far-cap's own extent when
# ``far_cap_mitre_clearance_mm`` scans the outer contour for the nearest
# far-side edge. The mitre corner effect that function measures is local to the
# cap's own corners (the reference failure's true nearest edge sits within 1.2
# mm of the cap's own corner — see that function's own docstring); this is
# generous well beyond that while still bounding the scan to a small local
# window rather than the whole contour.
_MITRE_CLEARANCE_WINDOW_PAD_MM = 5.0

# Tolerance for floating-point boundary/margin comparisons, in millimetres.
# Defined here (not in ``validate.py``) so ``opposing_pair_meets`` below and
# ``validate.py``'s invariant checks share exactly one constant rather than
# two that agree today and could silently drift apart -- the same
# single-source-of-truth discipline this module applies to
# ``local_lower_worst_case``/``local_upper_worst_case``. ``validate.py``
# imports this name rather than redefining it.
EPSILON_MM = 1e-6

# Millimetre tolerance for grouping "tied" extreme-X contour points when
# estimating rotation (``estimate_rotation_deg``). Deliberately larger than
# any realistic scaled-integer rounding gap (SCALING_FACTOR as coarse as
# 1e-5 mm/unit on large beds) but far smaller than any real geometric
# feature this codebase's fixtures exercise, mirroring the scale of the
# other module-level epsilons in this file/generator.py.
_ROTATION_TIE_EPSILON_MM = 1e-4

# Below this angle (degrees, either sign), ``corrected_skin_margin_mm``
# applies NO correction at all — the raw ``skin_margin_mm`` is used
# unchanged.
#
# WHY A DEADZONE, NOT "always correct by the estimated angle": the O(n)
# extreme-X estimator below is not exact on a real (non-rectangular,
# blunt-leading-edge) airfoil section even at TRUE zero rotation. Measured
# on the NACA 2412 fixture (chord 60 mm, 120 points/surface — the geometry
# ``tests/unit/test_local_geometry.py`` and ``tests/unit/test_m5_fixtures.py``
# use): the estimator reports
# -0.1343 degrees at zero actual rotation, because the true leading-edge
# vertex is not quite the outline's own X-minimum (a neighbouring vertex, a
# fraction of a mm further forward due to the leading edge's curvature,
# edges it out — real airfoil geometry, not an estimator bug; see
# ``tools/make_fixtures.py``'s ``naca4_surface_points`` and
# ``test_m5_fixtures.py``'s ``ShippedFixtureTests.test_twist_root_and_tip
# _rotation``, which documents and bounds the SAME bias independently via
# an exhaustive O(n^2) farthest-pair check used only in that test).
# Without a deadzone, that bias alone would apply a (tiny but nonzero)
# correction to every constant-section fixture — coupon, airfoil, taper,
# sweep, tip — none of which are actually rotated, breaking the
# exact-equality regression tests those fixtures have.
#
# 1.0 degree is comfortably (roughly 4x-7x, across 0-30 degrees of actual
# rotation) above the measured -0.1343..-0.6778 degree bias range, so a
# genuinely unrotated section's own estimator noise can never cross it, while
# still being far below the realistic washout range (2-6 degrees) this
# correction exists to serve.
_ROTATION_DEADZONE_DEG = 1.0

# Upper bound (degrees, either sign) on a section's own rotation that
# ``LayerFrame``'s margin measurements accept before failing closed — see
# ``check_rotation_envelope``.
#
# THE MEASUREMENT THIS IS CHOSEN AGAINST: local_span's world-Y probe
# overstates true chord-perpendicular thickness by 1/cos(theta), tabulated on
# the NACA 2412 twist fixture (7.1986 mm max thickness):
#
#     6 deg    0.0397 mm overstated   (4.0% of a 1.0 mm margin)
#    10 deg    0.1111 mm              (11.1%)
#    20 deg    0.4620 mm              (46.2%)
#    30 deg    1.1136 mm              (111% -- exceeds the whole margin)
#
# ``corrected_skin_margin_mm`` below removes that overstatement algebraically
# for a FLAT surface tilted by theta -- exact for a flat plate, an
# APPROXIMATION for a curved NACA surface, whose residual error is not
# bounded past the angles measured.
# 30.0 degrees is the largest angle measured (the table above); past it a
# slot could cut through a surface the validator believes is intact, even
# before correction. Using it as the backstop means the correction is only
# trusted at angles with direct evidence; an unmeasured, more extreme washout
# fails closed instead of silently extrapolating the correction.
#
# This is NOT chosen because the probe geometry itself breaks down here --
# checked empirically against the NACA 2412 outline, the
# single vertical measurement strip local_span uses stays a single
# connected interval (no re-entrant multi-crossing) even at 80 degrees of
# rotation, so ``local_span``'s own ValueError-on-re-entrant safeguard is
# not what this threshold is protecting; it is a scope limit on how far the
# cos(theta) APPROXIMATION has been validated, not a hard geometric wall.
DEFAULT_MAX_ROTATION_DEG = 30.0

# Tolerance (degrees) applied to the ``DEFAULT_MAX_ROTATION_DEG`` comparison
# in ``check_rotation_envelope``, mirroring this module's other epsilons: a
# section rotated EXACTLY to the threshold must not be spuriously refused by
# floating-point noise in the O(n) estimate.
_ROTATION_ENVELOPE_EPSILON_DEG = 1e-6


# Non-aerofoil FEATURE regions (a joiner post, boss, tab...) get a wholly
# different placement rule than an aerofoil section, so the first thing
# ``StructureGenerator.generate`` does is classify which one it has.
AEROFOIL_REGION = "aerofoil"
FEATURE_REGION = "feature"

# MEASURED (tools/make_fixtures.py's analytic NACA4-series outlines,
# 120 points/surface, plus synthetic feature shapes):
#
#   shape                    aspect   fill
#   NACA 2412 (the default fixture)           8.23   0.67
#   NACA 4415 (fattest aerofoil in the corpus) 6.43   0.66
#   NACA 2404 (thin)                          21.39   0.58
#   NACA 0006 (symmetric, thin)               16.67   0.68
#   8.64mm square post (worked example)  1.00   1.00
#   round boss, any radius                     1.00   0.785 (pi/4, exact)
#   40x7mm thin tab                            5.71   1.00
#
# ``aspect`` is max(width, depth) / min(width, depth) (orientation
# -agnostic: chord is not guaranteed to be the long bbox axis for an
# arbitrary feature the way it is for a canonical-frame aerofoil).
# ``fill`` is area / (width * depth) -- the fraction of its own bounding
# box the section occupies, aka solidity.
#
# ASPECT RATIO ALONE CANNOT SEPARATE THE FOUR SHAPE CLASSES: a thin tab (aspect
# 5-8) overlaps NACA 4415's aerofoil floor of 6.43. A single aspect threshold
# that admits the tab as FEATURE also admits NACA 4415 as FEATURE -- which
# would change a real wing fixture -- and one that excludes NACA 4415 also
# excludes the tab. FILL RATIO is the second signal that
# resolves this: every measured aerofoil section sits at 0.58-0.68 regardless
# of its aspect ratio (a thin curved sliver is necessarily empty of its own
# bounding box), while a tab, post, or any other rectangle-like feature is
# exactly 1.0 and a circular boss is exactly pi/4 (0.785) -- both comfortably
# clear of the aerofoil ceiling.
#
# ``DEFAULT_FEATURE_ASPECT_RATIO_MAX = 3.0``: roughly the geometric
# midpoint between a square (1.0) and NACA 4415's measured floor (6.43),
# leaving better than 2x headroom on the aerofoil side. Catches a squat
# feature (post, boss) regardless of its fill ratio.
# ``DEFAULT_FEATURE_FILL_RATIO_MIN = 0.85``: between the round boss's
# exact 0.785 and a rectangular tab's exact 1.0, with a wide (0.17)
# margin above the highest measured aerofoil fill (0.68). Catches an
# ELONGATED feature (a thin tab) the aspect test alone would misclassify
# as an aerofoil.
#
# A region is FEATURE iff EITHER test fires; AEROFOIL only if neither
# does. NOT RESOLVED by either signal: a low-aspect, low-fill shape near
# the aspect threshold (an elongated ellipse at aspect ~3, fill a
# constant pi/4 regardless of its aspect ratio) — see
# ``classify_region``'s own docstring.
DEFAULT_FEATURE_ASPECT_RATIO_MAX = 3.0
DEFAULT_FEATURE_FILL_RATIO_MIN = 0.85


def classify_region(
    frame: "LayerFrame",
    aspect_ratio_max: float = DEFAULT_FEATURE_ASPECT_RATIO_MAX,
    fill_ratio_min: float = DEFAULT_FEATURE_FILL_RATIO_MIN,
) -> str:
    """Classify a whole layer cross-section (``frame``) as
    ``AEROFOIL_REGION`` or ``FEATURE_REGION`` from measured geometry alone
    -- see the module-level constants above this function for the measured
    separation the defaults rest on.

    A region is ``FEATURE_REGION`` when EITHER its aspect ratio
    (``max(width, depth) / min(width, depth)``) is at or below
    ``aspect_ratio_max`` (a squat feature: post, boss...) OR its fill
    ratio (``area / (width * depth)``, aka solidity) is at or above
    ``fill_ratio_min`` (an elongated but solid feature: a tab). Otherwise
    ``AEROFOIL_REGION``.

    KNOWN UNRESOLVED BOUNDARY CASE, stated rather than hidden: a shape with
    BOTH a low aspect ratio near ``aspect_ratio_max`` AND a fill ratio near
    ``fill_ratio_min`` from below -- e.g. an elongated ellipse, whose fill
    ratio is the constant pi/4 (~0.785) at ANY aspect ratio -- is genuinely
    ambiguous by this measure alone, and this function resolves it purely on
    the aspect test. Whether that resolution is "right" depends on what the
    shape actually is, which geometry alone cannot say (recognising a feature's PURPOSE is out
    of scope).

    Raises ``ValueError`` if ``frame.width`` or ``frame.depth`` is zero
    -- a degenerate section has no meaningful aspect or fill ratio, the
    same fail-closed posture ``estimate_rotation_deg`` already takes for
    a degenerate polygon, rather than dividing by zero or silently
    picking a class.
    """
    width, depth = frame.width, frame.depth
    if width <= 0.0 or depth <= 0.0:
        raise ValueError(
            "cannot classify a degenerate section for AEROFOIL/FEATURE: "
            "width={!r}, depth={!r}.".format(width, depth)
        )
    aspect = max(width, depth) / min(width, depth)
    area = frame.ops.area(frame.polygon)
    fill = area / (width * depth)
    if aspect <= aspect_ratio_max or fill >= fill_ratio_min:
        return FEATURE_REGION
    return AEROFOIL_REGION


# Perpendicular (surface-normal) web entry, threshold-gated and clamped.
#
# Below this off-normal error (degrees), a web's entry stays exactly vertical
# -- no tilt is even computed, let alone applied. Measurement on the NACA 2412
# puts the structurally useful mid-chord (0.25-0.45c) under 4 degrees and the
# LE/outboard extremes at 21.3/10.9 degrees; 5.0 sits above the mid-chord
# noise floor and below both extremes, so the tilted path is exercised only
# near the LE/TE, and the mid-chord surface (every fixture built at zero
# tilt) is untouched by construction -- see ``estimate_local_tilt_deg``'s
# docstring for the measurement this threshold is compared against.
DEFAULT_TILT_THRESHOLD_DEG = 5.0

# Millimetre half-spacing between the two ``local_span`` probes
# ``estimate_local_tilt_deg`` finite-differences to estimate the local
# surface slope. Small enough to approximate a single station's tangent (not
# average over a large arc of a curved leading edge), large enough that a
# scaled-integer backend does not round the two probes to the same station
# -- the same "not degenerate, not too coarse" tradeoff
# ``DEFAULT_PROBE_WIDTH_MM`` documents for the probe strip itself,
# one order of magnitude coarser here because this measures a SLOPE (a
# difference of two readings), which is far more sensitive to probe-width
# quantization noise than a single worst-case reading is.
DEFAULT_TILT_PROBE_DELTA_MM = 0.25


def estimate_rotation_deg(ops: GeometryOps, polygon) -> float:
    """Cheap, per-layer estimate of a section's rotation from its own chord.

    METHOD: read the polygon's own outer-contour vertices
    (``ops.contour_points_mm``) and take the line between the point(s) at
    the section's minimum X and the point(s) at its maximum X as a proxy
    for the chord line (the leading/trailing edge are the extremes along the
    bbox): a wing section's chord (tens of mm)
    vastly exceeds its thickness (a few mm), so among ALL vertex pairs the
    one separated the most in X is, for any realistic washout, at or very
    near the true leading-to-trailing-edge line, without needing a full
    O(n^2) all-pairs diameter search or a convex-hull-plus-rotating
    -calipers construction. When several vertices tie (within
    ``_ROTATION_TIE_EPSILON_MM``) for the same extreme X — e.g. every
    corner of an axis-aligned rectangle — this AVERAGES their Y rather than
    picking one arbitrarily by iteration order, which is what makes the
    method return an EXACT 0.0 for a rectangle regardless of point order
    (the average of two symmetric corners' Y always cancels), rather than a
    spurious +/-45 degrees from an unlucky tie-break.

    COST: two linear scans of the contour's point list (min/max X, then the
    tied-group averages) — O(n) in vertex count, no sort, no pairwise
    comparison. On the largest fixture (~240 points per NACA 2412 layer)
    this is negligible next to the per-station geometry queries
    ``local_span`` performs every layer.

    MEASURED ACCURACY (against the NACA 2412 fixture,
    tools/make_fixtures.py, chord 60 mm, 120 points/surface — see
    ``_ROTATION_DEADZONE_DEG``'s comment and
    ``tests/unit/test_m5_fixtures.py``'s independent O(n^2) cross-check):

        true washout   estimate    error
             0 deg      -0.1343    -0.1343 deg
             6 deg       5.8657    -0.1343 deg
            10 deg       9.7309    -0.2691 deg
            20 deg      19.5954    -0.4046 deg
            30 deg      29.3222    -0.6778 deg

    The error is a small, slowly-growing UNDER-estimate (never an
    over-estimate in this survey), driven by the leading edge's own
    curvature rather than any bug in the method — see
    ``_ROTATION_DEADZONE_DEG``. Accurate to well under half a degree across
    the realistic washout range (2-6 degrees) and to under a degree at the
    30-degree backstop.

    Raises ``ValueError`` if the polygon has fewer than 2 contour points
    (degenerate; nothing to estimate a chord line from).
    """
    points = ops.contour_points_mm(polygon)
    if len(points) < 2:
        raise ValueError(
            "cannot estimate rotation: polygon has {} contour point(s), "
            "need at least 2.".format(len(points))
        )
    min_x = min(p[0] for p in points)
    max_x = max(p[0] for p in points)
    min_ys = [p[1] for p in points if abs(p[0] - min_x) <= _ROTATION_TIE_EPSILON_MM]
    max_ys = [p[1] for p in points if abs(p[0] - max_x) <= _ROTATION_TIE_EPSILON_MM]
    min_pt = (min_x, sum(min_ys) / len(min_ys))
    max_pt = (max_x, sum(max_ys) / len(max_ys))
    dx = max_pt[0] - min_pt[0]
    dy = max_pt[1] - min_pt[1]
    if dx == 0.0 and dy == 0.0:
        return 0.0
    return math.degrees(math.atan2(dy, dx))


def check_rotation_envelope(rotation_deg: float, max_rotation_deg: float) -> None:
    """Fail closed if ``rotation_deg`` exceeds ``max_rotation_deg``.

    See ``DEFAULT_MAX_ROTATION_DEG`` for the default threshold and its
    justification. The message states the measured angle, the threshold,
    and WHY: the thickness reading is optimistic by ``1/cos(theta)`` past
    this point, so margins built from it can no longer be trusted, and the
    ``corrected_skin_margin_mm`` approximation this module applies below
    the threshold has not been validated past it.
    """
    if abs(rotation_deg) > max_rotation_deg + _ROTATION_ENVELOPE_EPSILON_DEG:
        raise ValueError(
            "section rotation {:.4f} degrees exceeds the configured maximum "
            "of {:.4f} degrees. local_span measures thickness along world "
            "Y, which overstates the true chord-perpendicular thickness by "
            "1/cos(theta) on a rotated section -- an OPTIMISTIC error: the "
            "code would believe more skin survives here than actually "
            "does. Below the threshold this module corrects for that "
            "overstatement (corrected_skin_margin_mm); past it, that "
            "correction is unvalidated (see DEFAULT_MAX_ROTATION_DEG's "
            "docstring for the measured table it is based on), so margins "
            "can no longer be trusted and this fails closed instead of "
            "silently extrapolating.".format(rotation_deg, max_rotation_deg)
        )


def corrected_skin_margin_mm(skin_margin_mm: float, rotation_deg: float) -> float:
    """The Y-space margin that guarantees ``skin_margin_mm`` of TRUE
    (chord-perpendicular) skin, given the section's own ``rotation_deg``.

    local_span's world-Y reading overstates true thickness by
    ``1/cos(theta)`` (see the module docstring and
    ``DEFAULT_MAX_ROTATION_DEG``'s measured table), i.e.
    ``measured = true / cos(theta)``. To guarantee ``true >=
    skin_margin_mm``, the measured Y-distance must therefore be at least
    ``skin_margin_mm / cos(theta)`` -- algebraically the same statement,
    solved for the quantity callers actually compare against (a Y-extent),
    so no caller needs to convert a Y-reading to a "true" thickness itself;
    it just needs a bigger, honest margin to compare that Y-reading
    against.

    Applies ``_ROTATION_DEADZONE_DEG``: below that angle, returns
    ``skin_margin_mm`` UNCHANGED -- see that constant's comment for why
    (the estimator's own small bias on a genuinely unrotated section must
    never perturb the exact-equality-tested unrotated fixtures).

    Does not itself bound ``rotation_deg`` -- call ``check_rotation_envelope``
    first; this function trusts its caller to have done so.
    """
    if abs(rotation_deg) < _ROTATION_DEADZONE_DEG:
        return skin_margin_mm
    theta_rad = math.radians(rotation_deg)
    cos_theta = math.cos(theta_rad)
    if cos_theta <= 0.0:
        raise ValueError(
            "rotation_deg={!r} is at or past 90 degrees; cos(theta) is not "
            "positive, so no finite Y-space margin can guarantee "
            "skin_margin_mm of true thickness here. This should already "
            "have failed closed via check_rotation_envelope (its default "
            "threshold is well under 90 degrees) -- reaching this branch "
            "means a caller invoked this function without that "
            "check.".format(rotation_deg)
        )
    return skin_margin_mm / cos_theta


def local_span(
    ops: GeometryOps,
    polygon,
    x_mm: float,
    probe_width_mm: float,
    hole_side: Optional[str] = None,
) -> Tuple[float, float]:
    """Local (y_min, y_max) of ``polygon`` at chord station ``x_mm``.

    Intersects ``polygon`` with a thin axis-aligned strip centred on
    ``x_mm`` and spanning the full Y range (built via
    ``ops.polygon_from_points`` and ``ops.intersection``), then takes the
    bounds of the result.

    Raises ``ValueError`` if the probe finds no material (the station is off
    the section entirely). A multi-piece result (the probe crosses the
    section's material more than once) is handled as follows:

    - If every gap between consecutive pieces is accounted for by a
      DECLARED interior hole (``ops.hole_bounds(polygon)``) whose own X
      footprint covers this probe's X range and whose Y extent contains the
      gap (within ``EPSILON_MM``), this is a legitimate hole crossing, not
      a defect. What happens next depends on ``hole_side``:

        - ``hole_side is None`` (the default): still raises ``ValueError``,
          but with a message that NAMES the hole rather than calling the
          section "re-entrant" (which would be misleading for this case).
          This is the behaviour for every call site that does not handle a
          hole-crossing station (the ``validate.py`` invariant-3/4 sites;
          ``opposing_pair_meets`` never reaches this branch because it goes
          through ``local_solid_thickness_worst_case``, which always passes
          an explicit ``hole_side`` via ``local_upper_worst_case``/``local_
          lower_worst_case``): fail closed, with an honest reason.
        - ``hole_side == "upper"`` or ``"lower"``: returns the SELECTED
          piece's own ``(y_min, y_max)`` — the piece whose Y range is
          entirely above the hole ("upper") or entirely below it
          ("lower"). This is deliberately the piece itself, not the full
          material envelope across every piece: the piece's INNER bound
          (the hole-facing one) is the hole's own near boundary, which a
          scar-placement primitive needs and an envelope answer would
          destroy. ``local_upper_worst_case``/``local_lower_worst_case``
          only read the piece's OUTER bound (the true exterior surface,
          unaffected by an interior hole), so this is correct for both
          without either needing the other's half of the pair.

      Only ONE hole per gap is matched (the design is scoped to one hole per
      layer); a probe spanning more than one hole, or more than two pieces,
      is not resolved here and falls through to the re-entrant/named-hole
      failure below.

    - Any gap NOT accounted for by a declared hole means the probe found a
      genuine re-entrant section — where "the" local upper/lower surface is
      not a well-defined pair of points without picking which piece is
      load-bearing. Degrading and continuing is not allowed, so this fails
      closed with the "re-entrant section" message regardless of
      ``hole_side``.

    A single-piece probe never calls ``ops.hole_bounds``, so a hole-free
    section's sequence of ``GeometryOps`` calls is the same as without hole
    awareness.
    """
    if probe_width_mm <= 0:
        raise ValueError(
            "probe_width_mm must be > 0, got {!r}.".format(probe_width_mm)
        )
    if hole_side not in (None, "upper", "lower"):
        raise ValueError(
            "hole_side must be None, 'upper', or 'lower', got "
            "{!r}.".format(hole_side)
        )
    min_x, min_y, max_x, max_y = ops.bounds(polygon)
    # Generous padding so the strip's Y range envelops the whole section
    # regardless of station — local_span only wants the intersection's
    # extent in Y, not a tight clip.
    pad = (max_y - min_y) + 1.0
    half_w = probe_width_mm / 2.0
    strip_min_x = x_mm - half_w
    strip_max_x = x_mm + half_w
    strip_points = (
        (strip_min_x, min_y - pad),
        (strip_max_x, min_y - pad),
        (strip_max_x, max_y + pad),
        (strip_min_x, max_y + pad),
    )
    strip = ops.polygon_from_points(strip_points)
    pieces = ops.intersection(polygon, strip)

    if not pieces:
        raise ValueError(
            "no material at chord station x={:.4f} mm (probe width {:.4f} "
            "mm) — the section does not reach this far along the chord "
            "axis.".format(x_mm, probe_width_mm)
        )
    if len(pieces) == 1:
        _, piece_min_y, _, piece_max_y = ops.bounds(pieces[0])
        return piece_min_y, piece_max_y

    # Multi-piece: sort by Y so consecutive pieces have a well-defined gap
    # between them, and check whether every gap is explained by a declared
    # hole before doing anything else with the re-entrant guard.
    piece_bounds = sorted(
        (ops.bounds(piece) for piece in pieces), key=lambda b: b[1]
    )
    holes = ops.hole_bounds(polygon)
    explaining_hole = None
    all_gaps_explained = True
    for (_, _, _, lower_max_y), (_, upper_min_y, _, _) in zip(
        piece_bounds, piece_bounds[1:]
    ):
        gap_explained = False
        for hole in holes:
            hole_min_x, hole_min_y, hole_max_x, hole_max_y = hole
            if (
                hole_min_x - EPSILON_MM <= strip_min_x
                and hole_max_x + EPSILON_MM >= strip_max_x
                and hole_min_y - EPSILON_MM <= lower_max_y
                and hole_max_y + EPSILON_MM >= upper_min_y
            ):
                gap_explained = True
                explaining_hole = hole
                break
        if not gap_explained:
            all_gaps_explained = False
            break

    if not all_gaps_explained or len(piece_bounds) != 2:
        raise ReEntrantSectionError(
            "chord station x={:.4f} mm crosses the section's material {} "
            "separate times (probe width {:.4f} mm); this is a re-entrant "
            "section and local thickness is not a well-defined single pair "
            "of surfaces here. Failing closed rather than guessing which "
            "piece is load-bearing.".format(x_mm, len(pieces), probe_width_mm)
        )

    lower_min_x, lower_min_y, lower_max_x, lower_max_y = piece_bounds[0]
    upper_min_x, upper_min_y, upper_max_x, upper_max_y = piece_bounds[1]
    lower_piece = (lower_min_y, lower_max_y)
    upper_piece = (upper_min_y, upper_max_y)
    if hole_side is None:
        hole_min_x, hole_min_y, hole_max_x, hole_max_y = explaining_hole
        raise ValueError(
            "chord station x={:.4f} mm crosses a DECLARED interior hole "
            "(hole bounds x=[{:.4f}, {:.4f}] y=[{:.4f}, {:.4f}] mm), not a "
            "defect -- but local_span was not asked to resolve which side "
            "of the hole to report (hole_side=None), so it fails closed "
            "rather than guessing. Pass hole_side='upper' or 'lower' if "
            "this call site knows which exterior surface it "
            "needs.".format(
                x_mm, hole_min_x, hole_max_x, hole_min_y, hole_max_y
            )
        )
    return upper_piece if hole_side == "upper" else lower_piece


def estimate_local_tilt_deg(
    ops: GeometryOps,
    polygon,
    x_mm: float,
    upper: bool,
    probe_width_mm: float = DEFAULT_PROBE_WIDTH_MM,
    delta_mm: float = DEFAULT_TILT_PROBE_DELTA_MM,
) -> Optional[float]:
    """Off-normal error, in degrees, of a VERTICAL cut entering the local UPPER
    (``upper=True``) or LOWER (``upper=False``) surface at ``x_mm`` -- the
    angle between world-Y (the vertical entry direction) and the true local
    surface normal there (finite-differencing two ``local_span`` calls
    rather than adding a new ``GeometryOps`` primitive).

    METHOD: probes ``local_span`` at ``x_mm - delta_mm/2`` and
    ``x_mm + delta_mm/2``, reads the chosen surface's Y at each (index 1 for
    upper, 0 for lower), and returns ``degrees(atan(slope))`` where
    ``slope = dy/dx`` across the two probes -- the tangent's own angle from
    horizontal, which equals a vertical cut's angle from the true normal (a
    cut along the normal makes the same angle with vertical that the tangent
    makes with horizontal; the two angles are complementary by construction
    of "perpendicular").

    SIGN carries the tilt DIRECTION (positive slope tilts a corrected entry
    toward +X), not just its magnitude -- a caller wanting the reported
    error magnitude (e.g. against ``DEFAULT_TILT_THRESHOLD_DEG``) takes
    ``abs()`` itself; this function does not, so the sign survives for
    placement.

    Returns ``None`` (never raises) if either probe fails -- off the section,
    or a re-entrant/multi-piece station. This is a MEASUREMENT primitive for an
    approximation that is licensed to fall back to vertical; an unmeasurable
    slope must fail SAFE (no tilt applied) rather than propagate a hard
    failure into a caller that would otherwise have built a perfectly good
    vertical web there.

    MEASURED against the NACA 2412 c60 fixture: see
    ``tests/unit/test_perpendicular_entry.py`` for the per-station comparison
    against the tabulated tilt at each chord fraction -- this function's
    numbers reproduce it to within the finite-difference residual (bounded by
    the surface's own curvature times ``delta_mm``, negligible at this
    fixture's chord scale).
    """
    half = delta_mm / 2.0
    try:
        y_lo = local_span(ops, polygon, x_mm - half, probe_width_mm)
        y_hi = local_span(ops, polygon, x_mm + half, probe_width_mm)
    except ValueError:
        return None
    idx = 1 if upper else 0
    slope = (y_hi[idx] - y_lo[idx]) / delta_mm
    return math.degrees(math.atan(slope))


# Number of stations ``local_lower_worst_case``/``local_upper_worst_case``
# sample across a slot-width span, including both edges and the centre
# (see this module's "worst case across a span" note above).
# Odd so the centre station is sampled exactly, not interpolated near it.
# Chosen empirically against the NACA 2412 fixture
# (tests/unit/test_local_geometry.py): on the highest-curvature region
# exercised (near the leading edge, where the surface's slope is
# steepest), successive doublings of this count change the reported worst
# case by well under 1e-4 mm for a typical slot width (a few tenths of a
# millimetre) — three orders of magnitude below EPSILON_MM's neighbourhood
# in ``validate.py`` and the skin margins in play (tenths to a few mm), so
# residual sampling error is negligible at this density.
DEFAULT_WORST_CASE_SAMPLES = 9


def _sample_stations(x_center_mm: float, span_mm: float, samples: int):
    """``samples`` evenly spaced X stations covering
    ``[x_center_mm - span_mm/2, x_center_mm + span_mm/2]``, endpoints
    included."""
    if span_mm <= 0:
        raise ValueError("span_mm must be > 0, got {!r}.".format(span_mm))
    if samples < 3:
        raise ValueError(
            "samples must be >= 3 (both edges and the centre at minimum), "
            "got {!r}.".format(samples)
        )
    half = span_mm / 2.0
    step = span_mm / (samples - 1)
    return [x_center_mm - half + i * step for i in range(samples)]


def local_lower_worst_case(
    ops: GeometryOps,
    polygon,
    x_center_mm: float,
    span_mm: float,
    probe_width_mm: float = DEFAULT_PROBE_WIDTH_MM,
    samples: int = DEFAULT_WORST_CASE_SAMPLES,
    hole_aware: bool = False,
) -> float:
    """Worst-case (highest) LOCAL lower-surface Y across a span of the chord.

    ``hole_aware`` (default ``False``): when ``True``, a sampled station that crosses a DECLARED interior hole
    resolves to the piece BELOW the hole (``local_span``'s
    ``hole_side="lower"``) instead of raising. This is safe specifically
    because this function only ever reads a piece's OUTER (here: lowest) bound,
    which is the true exterior lower surface — unaffected by a hole sitting
    above it — never the hole-facing inner bound. A station outside every
    declared hole's X footprint, or a genuinely re-entrant one, is unaffected
    by this flag and behaves exactly as when ``hole_aware`` is ``False``. Left
    ``False`` by default because ``validate.py``'s invariants 3/4 are not
    hole-safe — their fail-closed behaviour at a hole-crossing station must
    stay as it is. ``opposing_pair_meets`` does not call this function
    directly: it calls ``local_solid_thickness_worst_case``, which calls this
    function WITH ``hole_aware=True`` internally and applies its own hole-void
    correction on top — see that function's docstring and
    ``opposing_pair_meets``'s own for the full reasoning.

    A single-point probe (``local_span`` at one station) is the wrong
    measurement for anything that must hold across a SLOT'S WHOLE WIDTH, not
    merely at one arbitrary station inside it — on a curved lower surface
    the remaining skin at the two edges of a slot can differ measurably from
    the skin at its centreline (placing a slot's far end from the
    centreline alone, while ``validate.py`` re-measures at the removed
    region's actual X centre — a different station on real Clipper-backed
    geometry — would find less skin than the generator had allowed for).

    Samples ``ops.bounds``-and-``local_span`` at ``samples`` evenly spaced
    stations across ``[x_center_mm - span_mm/2, x_center_mm + span_mm/2]``
    (at minimum both edges and the centre) and returns the HIGHEST local
    lower-surface Y found among them — the worst case for remaining skin,
    since remaining skin shrinks as the local lower surface rises.

    Why sampling rather than an exact interval extremum: the geometry
    backend (``GeometryOps``) exposes no operation that answers "the
    extremum of a surface over an X interval" directly — only pointwise
    queries via ``local_span``'s thin-strip probe — and the section is smooth
    over a slot-width span (no discontinuities), so a dense enough sample
    set approximates the true extremum closely. This is a deliberate
    approximation, not an exact bound: the reported value is the maximum of
    the SAMPLED stations only, so if the true worst station falls strictly
    between two samples, this function can under-estimate the true
    worst-case surface height there (and so over-estimate the skin actually
    available at that unsampled point) — the residual shrinks as ``samples``
    grows, and is negligible at the default density; see
    ``DEFAULT_WORST_CASE_SAMPLES``'s own comment for the measured bound on
    this fixture. Callers with an unusually sharp local feature (a station
    spacing narrower than the surface's own radius of curvature) should pass
    a larger ``samples`` explicitly.

    Raises ``ValueError`` (via ``span_mm``/``samples`` validation or
    ``local_span`` itself) exactly as ``local_span`` does — no
    degrade-and-continue.
    """
    stations = _sample_stations(x_center_mm, span_mm, samples)
    hole_side = "lower" if hole_aware else None
    worst = None
    for x in stations:
        y_min, _y_max = local_span(ops, polygon, x, probe_width_mm, hole_side=hole_side)
        if worst is None or y_min > worst:
            worst = y_min
    return worst


class ReEntrantSectionError(ValueError):
    """The chord probe crossed the section's material more than twice, so "local
    thickness" is not a well-defined pair of surfaces at that station.

    A distinct TYPE, subclassing ValueError so an ``except ValueError`` still
    catches it, so that the capability can degrade a layer on THIS condition
    specifically without widening into a catch-all. Measured on a panel at
    layer 1125, Z=225.0 -- 0.74 mm below the tip, where the section pinches
    into two lobes.

    Why degrading is safe here and not a weakening: the whole SECTION is
    re-entrant, so there is no station at which the wing vocabulary can be
    applied. Leaving the layer unmutated ships the host's own geometry
    untouched, as the zero-intent path does for any layer that cannot carry
    structure. It is not a case of dropping one uncertain feature and
    keeping others.

    Do NOT adopt this type elsewhere: it grants a degrade path past a
    fail-closed check, which is the error class the pitch-collapse narrowing
    was introduced to prevent.
    """


def local_upper_worst_case(
    ops: GeometryOps,
    polygon,
    x_center_mm: float,
    span_mm: float,
    probe_width_mm: float = DEFAULT_PROBE_WIDTH_MM,
    samples: int = DEFAULT_WORST_CASE_SAMPLES,
    hole_aware: bool = False,
) -> float:
    """Worst-case (lowest) LOCAL upper-surface Y across a span of the chord.

    For invariant 3 — the slot must reach the exterior boundary.

    ``hole_aware`` (default ``False``): the mirror of ``local_lower_
    worst_case``'s flag — see its docstring for the full reasoning. When
    ``True``, a station crossing a declared hole resolves to the piece ABOVE
    the hole (``local_span``'s ``hole_side="upper"``), reading that piece's
    outer (highest) bound — the true exterior upper surface, unaffected by a
    hole below it.

    OPEN QUESTION, not settled here. Arguably the demanding
    station is the one where the skin sits HIGHEST: a cut reaching only the
    lowest point of the upper surface could leave skin intact where the
    surface rises, which on a steeply curved leading edge is a real failure
    mode if the overshoot (0.05 mm by default) is smaller than the surface's
    rise across the slot width (0.21 mm at the default ratio).

    Taking the highest is not done: every test backend models the cut with a
    FLAT cap, so a removed region can never reach a curved surface's high
    point there, and the stricter rule fails legitimate slots by ~0.02 mm for
    reasons that are artefacts of the stub rather than of the geometry. Real
    Clipper cuts follow the true outline, so only an in-host slice can settle
    it. Until then this is the weaker, evidence-backed form.

    See ``local_lower_worst_case`` for the sampling count and the
    sampling-vs-exact-extremum rationale. Residual error direction here:
    sampling can under-estimate the true highest upper surface if the
    extremum falls between stations, i.e. it can be slightly optimistic
    about the cut having reached the skin, by a negligible amount at the
    default sample density.
    """
    stations = _sample_stations(x_center_mm, span_mm, samples)
    hole_side = "upper" if hole_aware else None
    worst = None
    for x in stations:
        _y_min, y_max = local_span(ops, polygon, x, probe_width_mm, hole_side=hole_side)
        if worst is None or y_max < worst:
            worst = y_max
    return worst


def estimate_far_surface_slope_deg(
    ops: GeometryOps,
    polygon,
    x_min_mm: float,
    x_max_mm: float,
    upper: bool,
    probe_width_mm: float = DEFAULT_PROBE_WIDTH_MM,
) -> Optional[float]:
    """Secant slope, in degrees, of the local UPPER/LOWER surface between
    ``x_min_mm`` and ``x_max_mm``.

    Used by the thin-section halo check, which compares a world-Y extent
    against a threshold that assumes the far surface is roughly constant
    across the probed window. That assumption fails whenever the surface is
    genuinely SLOPED across the window, for two distinct causes that this one
    measurement covers: a rotated/twisted section's own global rotation
    (``frame.rotation_deg``, which a flat far surface reproduces almost
    exactly as a WORLD slope), and a section's own intrinsic curvature
    approaching a chord extremity (any NACA section's trailing edge closes to
    zero thickness over a far longer chordwise span than the leading edge's
    blunt curvature, so a trigger of "only within one candidate-depth of the
    true extremity" does not fire there even though the boundary is already
    measurably sloped well before that point). Measuring the LOCAL slope
    directly subsumes the global-rotation case rather than needing to be
    combined with it; it is a different, larger quantity than
    ``frame.rotation_deg``.

    METHOD: two single-point ``local_span`` probes, one at each end of the
    window (NOT a worst-case-over-span scan like ``local_lower_worst_case``
    /``local_upper_worst_case`` -- this measures the boundary's TREND
    across the window, not its extremum), reading the chosen surface's Y
    at each (``y_max`` for ``upper=True``, ``y_min`` for ``upper=False``)
    and returning ``degrees(atan(slope))`` where ``slope = dy / dx``.

    Returns ``None`` (never raises) if either edge probe fails -- off the
    section, or a re-entrant/multi-piece station -- or if the window is
    degenerate (``x_max_mm <= x_min_mm``). A caller must treat ``None`` as "no
    correction available" and fall back to something known safe (the caller
    falls back to ``frame.rotation_deg``) -- this is a best-effort
    measurement, licensed to under-correct, never a fail-closed path of its
    own.
    """
    if x_max_mm <= x_min_mm:
        return None
    try:
        y_min_a, y_max_a = local_span(ops, polygon, x_min_mm, probe_width_mm)
        y_min_b, y_max_b = local_span(ops, polygon, x_max_mm, probe_width_mm)
    except ValueError:
        return None
    y_a = y_max_a if upper else y_min_a
    y_b = y_max_b if upper else y_min_b
    slope = (y_b - y_a) / (x_max_mm - x_min_mm)
    return math.degrees(math.atan(slope))


def _point_segment_distance_mm(
    p: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]
) -> float:
    """Euclidean distance from point ``p`` to segment ``a``-``b``, in mm."""
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def _segments_intersect_mm(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    p3: Tuple[float, float],
    p4: Tuple[float, float],
) -> bool:
    """Whether open segments ``p1``-``p2`` and ``p3``-``p4`` intersect
    (including touching/collinear-overlap cases), by standard orientation
    tests — needed so ``_segment_distance_mm`` reports an exact 0.0 when two
    segments cross, which the "min of the four endpoint-to-segment
    distances" formula alone does not guarantee for a proper crossing."""

    def ccw(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    d1 = ccw(p3, p4, p1)
    d2 = ccw(p3, p4, p2)
    d3 = ccw(p1, p2, p3)
    d4 = ccw(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 != 0 and d2 != 0:
        return True

    def on_segment(a, q, b):
        return (
            min(a[0], b[0]) - 1e-9 <= q[0] <= max(a[0], b[0]) + 1e-9
            and min(a[1], b[1]) - 1e-9 <= q[1] <= max(a[1], b[1]) + 1e-9
        )

    if d1 == 0 and on_segment(p3, p1, p4):
        return True
    if d2 == 0 and on_segment(p3, p2, p4):
        return True
    if d3 == 0 and on_segment(p1, p3, p2):
        return True
    if d4 == 0 and on_segment(p1, p4, p2):
        return True
    return False


def _segment_distance_mm(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    p3: Tuple[float, float],
    p4: Tuple[float, float],
) -> float:
    """Exact minimum distance between segments ``p1``-``p2`` and
    ``p3``-``p4``, in mm: 0.0 if they intersect, else the minimum of the
    four endpoint-to-opposite-segment distances (the standard, exact
    closed-form result for two straight segments)."""
    if _segments_intersect_mm(p1, p2, p3, p4):
        return 0.0
    return min(
        _point_segment_distance_mm(p1, p3, p4),
        _point_segment_distance_mm(p2, p3, p4),
        _point_segment_distance_mm(p3, p1, p2),
        _point_segment_distance_mm(p4, p1, p2),
    )


def _clip_segment_to_x_window_mm(
    p: Tuple[float, float],
    q: Tuple[float, float],
    window_min_x: float,
    window_max_x: float,
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Clip segment ``p``-``q`` to the vertical slab
    ``window_min_x <= x <= window_max_x``, returning the clipped endpoints
    (linearly interpolated where an original endpoint falls outside the
    window) or ``None`` if the segment does not overlap the window at all.
    A vertical segment (``p[0] == q[0]``) is returned unclipped whenever
    its single X value lies in the window (clipping in X cannot narrow a
    segment whose X is already a single point)."""
    x1, y1 = p
    x2, y2 = q
    seg_min_x = x1 if x1 < x2 else x2
    seg_max_x = x1 if x1 > x2 else x2
    if seg_max_x < window_min_x or seg_min_x > window_max_x:
        return None
    if x1 == x2:
        return p, q
    # Parametrize p -> q as t in [0, 1]; clip t to where x(t) is in-window.
    t_lo, t_hi = 0.0, 1.0
    dx = x2 - x1
    for bound, is_lower in ((window_min_x, True), (window_max_x, False)):
        t_bound = (bound - x1) / dx
        if dx > 0:
            if is_lower:
                t_lo = max(t_lo, t_bound)
            else:
                t_hi = min(t_hi, t_bound)
        else:
            if is_lower:
                t_hi = min(t_hi, t_bound)
            else:
                t_lo = max(t_lo, t_bound)
    if t_lo > t_hi:
        return None
    cp = (x1 + t_lo * dx, y1 + t_lo * (y2 - y1))
    cq = (x1 + t_hi * dx, y1 + t_hi * (y2 - y1))
    return cp, cq


def _chord_extreme_indices(points: List[Tuple[float, float]]) -> Tuple[int, int]:
    """Indices of the contour's chord extremes: the minimum-X and
    maximum-X vertices.

    TIE HANDLING: if several vertices share the exact same extreme X
    (e.g. every fixture built from an axis-aligned rectangle, where two
    corners share ``min_x`` and the other two share ``max_x``), this picks
    the FIRST such vertex in the contour's own iteration order (index
    ``0..n-1``) -- the same tie-break Python's own ``min``/``max`` apply
    when compared by a key, so this is not a bespoke rule invented here.
    Deterministic, independent of any property of the tied points other
    than their position in the point list, and stable across repeated runs
    on the same section (the point list's order is itself deterministic --
    ``ops.contour_points_mm``'s own "native order" contract).

    THIS IS NOT A TIE-BREAK DEFECT. Worked through on a rectangle ``[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]``:
    ``min_idx=0``, ``max_idx=1``, so arc A is the bottom edge alone and
    arc B is right wall + top edge + left wall. Classification is correct
    (A lower, B upper) but B carries BOTH end walls. Try to fix it with a
    smarter rule -- break min-X ties by lowest Y and max-X ties by highest
    Y -- and you get one wall per chain instead of two, never zero.

    **No split can exclude an end wall, because an end wall is an edge and
    must belong to one arc or the other.** That is a structural property
    of any chain decomposition of a section with blunt ends, not a
    weakness of this particular rule.

    The consequence matters for reading the code: the X window in
    ``far_cap_mitre_clearance_mm`` is NOT a residual propping up an
    incomplete topological method. The two filters answer DIFFERENT
    questions and both are necessary -- chain membership excludes
    NEAR-SURFACE edges, the X window excludes DISTANT END WALLS.

    And where a web genuinely sits within the window of a blunt end, the
    end wall enters scope and is measured against. That is behaviour, not
    a bug: an erosion near a blunt trailing edge really can neck against
    that wall.
    """
    n = len(points)
    min_idx = min(range(n), key=lambda i: points[i][0])
    max_idx = max(range(n), key=lambda i: points[i][0])
    return min_idx, max_idx


def _arc_indices(start: int, end: int, n: int) -> List[int]:
    """Indices of a closed ``n``-vertex contour walking forward (increasing
    index, modulo ``n``) from ``start`` to ``end`` inclusive of both
    endpoints."""
    idxs = [start]
    i = start
    while i != end:
        i = (i + 1) % n
        idxs.append(i)
    return idxs


def _split_upper_lower_chains(
    points: List[Tuple[float, float]],
) -> Tuple[
    List[Tuple[Tuple[float, float], Tuple[float, float]]],
    List[Tuple[Tuple[float, float], Tuple[float, float]]],
]:
    """Split a closed contour into its UPPER and LOWER chains at the chord
    extremes (the min-X and max-X vertices), and classify which chain is
    which.

    CHAIN MEMBERSHIP, NOT SLOPE. The min-X and max-X vertices
    (``_chord_extreme_indices``) split the closed contour into exactly two arcs
    -- walk the vertex indices forward from one extreme to the other, and
    forward again from that extreme back to the first. Which arc an edge
    belongs to is a TOPOLOGICAL fact about the contour (which side of the
    chord-extreme split its two endpoints fall on), independent of how steep
    that edge happens to be. Inferring "is this a far-surface edge" from the
    edge's own orientation (``abs(dy) >= abs(dx)`` excluded as a presumed
    chord-extent end wall) is unsound: the leading-edge region of a real
    aerofoil can have a near-vertical surface tangent with no rotation at
    all, so a genuine far-surface edge there is steeper than 45 degrees and
    would be wrongly excluded, overstating the measured clearance in exactly
    the dangerous direction (see ``far_cap_mitre_clearance_mm``'s docstring).

    Each of the two arcs INCLUDES both chord-extreme vertices as its own
    endpoints (they are shared), so together the two arcs' edges partition
    the full edge set of the contour exactly once each: an ``n``-vertex
    closed contour has ``n`` edges, and the two arcs have
    ``len(arc) - 1`` edges apiece; since the arcs' vertex counts sum to
    ``n + 2`` (both endpoints counted twice), their edge counts sum to
    exactly ``n``.

    CLASSIFICATION: whichever arc's INTERIOR vertices (excluding the two
    shared chord-extreme endpoints, whose Y is ambiguous by construction --
    they sit AT the chord, not clearly above or below it) have the higher
    mean Y is the UPPER chain; the other is the LOWER chain. Mean, not a
    single sample, so one noisy/local vertex cannot flip the whole arc's
    classification. Robust to a chain with very few interior vertices: an
    arc with ZERO interior vertices (a direct edge straight from one
    chord extreme to the other, no vertex between them) is given the
    midpoint Y of the two chord-extreme vertices themselves as its
    representative -- exactly the value the OTHER arc's own classification
    is compared against, so the empty arc always ends up on the opposite
    side from whichever real surface the non-empty arc's vertices describe.
    (At most one of the two arcs can be a direct edge with zero interior
    vertices for any polygon with 3 or more vertices, since two zero
    -interior arcs would require exactly 2 total vertices.)

    Raises ``ValueError`` if the two arcs' representative Y values are
    exactly equal -- a degenerate, perfectly top/bottom-symmetric contour
    this heuristic cannot break the tie for. Not expected on any real
    mesh-derived section this codebase measures (every fixture has a
    cambered or at least non-degenerate surface); failing closed here is
    preferable to guessing.

    Returns ``(upper_edges, lower_edges)``, each a list of
    ``(p, q)`` point-pairs (one per edge in that chain, in the contour's
    own point precision -- no resampling).
    """
    n = len(points)
    min_idx, max_idx = _chord_extreme_indices(points)
    chain_a_idx = _arc_indices(min_idx, max_idx, n)
    chain_b_idx = _arc_indices(max_idx, min_idx, n)
    chord_mid_y = (points[min_idx][1] + points[max_idx][1]) / 2.0

    def representative_y(chain_idx: List[int]) -> float:
        interior = chain_idx[1:-1]
        if not interior:
            return chord_mid_y
        return sum(points[i][1] for i in interior) / len(interior)

    rep_a = representative_y(chain_a_idx)
    rep_b = representative_y(chain_b_idx)
    if rep_a == rep_b:
        raise ValueError(
            "cannot classify the contour's upper/lower chains: both arcs "
            "of the chord-extreme split have the same representative Y "
            "({:.6f} mm) -- a degenerate, top/bottom-symmetric contour "
            "this heuristic cannot break the tie for.".format(rep_a)
        )
    upper_idx, lower_idx = (
        (chain_a_idx, chain_b_idx) if rep_a > rep_b else (chain_b_idx, chain_a_idx)
    )

    def edges_for(
        idx_list: List[int],
    ) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        return [
            (points[idx_list[i]], points[idx_list[i + 1]])
            for i in range(len(idx_list) - 1)
        ]

    return edges_for(upper_idx), edges_for(lower_idx)


def far_cap_mitre_clearance_mm(
    ops: GeometryOps,
    polygon,
    x_center_mm: float,
    width_mm: float,
    cap_y_mm: float,
    dir_to_far: float,
    safety_offset_mm: float,
) -> float:
    """True clearance from a candidate web's DILATED far-cap corner to the
    section's own far-side outer contour, in mm.

    Measures the quantity behind an island-disconnect failure:
    ``validate.py``'s erosion check (``TopologyValidator``) measures whether
    the layer survives an erosion of ``safety_offset_mm`` with MITRE joins
    (``join_style=2, mitre_limit=3.0`` — empirical match to the real host), and
    a candidate web's own VERTICAL remaining-skin measurement systematically
    under-counts how much that erosion costs at the web's own far-cap corners.
    This function measures the same quantity the erosion check effectively
    measures, without eroding the whole layer.

    METHOD. The web's own slot is, at this call site, always an axis-aligned
    rectangle (vertical centerline, ``width_mm`` wide, square far cap at
    ``cap_y_mm``) — never the keyhole/socket shape (that branch never reaches
    this check). Dilating an axis-aligned rectangle outward by
    ``r=safety_offset_mm`` with a MITRE join at a 90-degree corner is EXACT and
    needs no boolean call: because the mitre ratio at a 90-degree corner is
    sqrt(2) (~1.414), well under the repo's mitre_limit of 3.0, the two edges'
    offset lines meet exactly where simple axis-aligned growth would put them —
    no clipping, no rounding. This matches the real ``MiterShapelyOps``
    backend: ``ops.offset(slot_poly, safety_offset_mm)`` reproduces this
    rectangle's corners to full float precision. It also matches the true
    erosion neck on a reference failure: the UNDILATED far-cap segment's
    distance to the far surface (0.860293 mm) equals the independently
    measured "true perpendicular clearance", and the DILATED segment's
    distance (0.419958 mm, just under ``safety_offset_mm``) is consistent with
    the measured erosion neck (0.839959 mm, just under
    ``2 * safety_offset_mm``) to within the reference bisection's own
    tolerance. So this function only needs the far cap's own dilated segment
    — the two long side edges of the dilated rectangle are farther from the
    far surface than the cap and never win the minimum.

    The far surface is read from ``polygon``'s own ORIGINAL outer contour
    (``ops.contour_points_mm`` — never called on the slot itself, per
    ``ops.py``'s contract) at EXACT vertex precision, not a sampled
    approximation: the reference failure's margin against the erosion
    threshold is ~4e-5 mm, well below what re-sampling the surface via
    ``local_span`` stations can resolve, since a mesh-derived section is
    piecewise-linear and a probe grid generally does not land exactly on its
    vertices.

    Candidate contour edges are selected by CHAIN MEMBERSHIP, not by their own
    orientation or Y position (``_split_upper_lower_chains``): the outer
    contour is split at its chord extremes (min-X/max-X vertices) into an UPPER
    and a LOWER chain, and only the chain on the FAR side (per ``dir_to_far``)
    is scanned at all. Two proxy filters are not used, because both are
    UNSOUND: excluding an edge whose "rise dominates its run" (presumed
    chord-extent end wall), and requiring at least one endpoint on the far side
    of ``cap_y_mm``. A genuine far-surface edge near the leading edge of a real
    aerofoil can be steeper than 45 degrees with no rotation at all, so the
    orientation filter could skip a nearer, genuinely-far edge and accept an
    over-stated clearance (see ``_split_upper_lower_chains``'s docstring). The
    Y-side filter fails in the other direction: on the tip-closure fixture's
    Z=54.2 mm station (``tests/integration/test_tip_shutdown.py``) the
    "nearest far edge" it found sat on the UPPER (near/entry) chain, not the
    LOWER (far) chain the candidate actually entered from, understating the
    true clearance by 27% (0.4134 mm measured vs. 0.5665 mm true) and wrongly
    omitting a perfectly good candidate. Both tests are proxies for "which
    surface does this edge belong to", and a proxy can be wrong in either
    direction while the topological fact it approximates cannot.

    SUBSUMPTION HOLDS WHENEVER THE CHORD EXTREMES ARE UNIQUE VERTICES --
    true for every aerofoil section this codebase measures (a real LE/TE
    apex is a single point). It is NOT unconditional: on a contour with a
    TIED extreme -- e.g. an axis-aligned rectangle coupon, whose two
    left-hand corners share ``min_x`` -- ``_chord_extreme_indices``'s
    first-index tie-break can put BOTH end walls in the same chain as one
    long surface edge (see that function's own docstring). In that case
    chain membership alone would not exclude an end wall from the far
    chain; only the local X-window clip below does (an end wall of a coupon
    far wider than the local window is clipped away) -- a locality bound, not
    a topological guarantee. No fixture in the suite exercises a tied-extreme
    contour through this function, so this is a documented gap, not a
    reproduced failure; a caller with such a contour should not assume
    subsumption holds and should re-verify locally.

    The scan is also clipped to a local X window
    (``_MITRE_CLEARANCE_WINDOW_PAD_MM`` beyond the dilated cap's own
    extent), which is orthogonal to the edge-selection question above: it is
    a locality optimisation (the mitre effect this function measures is local
    to the cap's own corners), not a substitute for knowing which surface an
    edge belongs to.

    ``dir_to_far`` is ``+1.0`` if the far surface is reached by increasing
    Y from the cap (entry from the lower surface, so the far chain is
    UPPER) or ``-1.0`` if reached by decreasing Y (entry from the upper
    surface, so the far chain is LOWER) — the generator's own ``-sign``
    convention (see ``StructureGenerator``'s per-family loop).

    Raises ``ValueError`` if no far-side contour edge is found within the
    window (the section does not extend past the cap on the far side at
    all, within reach) — never degrades to a fabricated large clearance.
    """
    if width_mm <= 0:
        raise ValueError("width_mm must be > 0, got {!r}.".format(width_mm))
    if safety_offset_mm <= 0:
        raise ValueError(
            "safety_offset_mm must be > 0, got {!r}.".format(safety_offset_mm)
        )
    if dir_to_far not in (1.0, -1.0):
        raise ValueError(
            "dir_to_far must be 1.0 or -1.0, got {!r}.".format(dir_to_far)
        )

    half_w = width_mm / 2.0
    r = safety_offset_mm
    cap_far_y = cap_y_mm + dir_to_far * r
    seg_a = (x_center_mm - half_w - r, cap_far_y)
    seg_b = (x_center_mm + half_w + r, cap_far_y)
    window_min_x = seg_a[0] - _MITRE_CLEARANCE_WINDOW_PAD_MM
    window_max_x = seg_b[0] + _MITRE_CLEARANCE_WINDOW_PAD_MM

    points = ops.contour_points_mm(polygon)
    n = len(points)
    if n < 3:
        raise ValueError(
            "polygon's outer contour has fewer than 3 points ({!r}); cannot "
            "split it into upper/lower chains or measure far-cap "
            "clearance.".format(n)
        )
    upper_edges, lower_edges = _split_upper_lower_chains(points)
    far_edges = upper_edges if dir_to_far > 0.0 else lower_edges

    best = None
    for p, q in far_edges:
        # CLIP to the X window (a long straight edge on a coarse-vertex
        # shape -- e.g. a plain rectangle coupon's bottom edge, corner-to
        # -corner -- can extend well beyond the window on one or both ends
        # without either raw endpoint landing inside it; clipping keeps
        # only the locally-relevant portion, which is all this function
        # needs since the mitre effect it measures is local to the cap's
        # own corners).
        clipped = _clip_segment_to_x_window_mm(p, q, window_min_x, window_max_x)
        if clipped is None:
            continue
        cp, cq = clipped
        d = _segment_distance_mm(seg_a, seg_b, cp, cq)
        if best is None or d < best:
            best = d
    if best is None:
        raise ValueError(
            "no far-side contour segment found beyond cap_y_mm={:.4f} mm "
            "within x=[{:.4f}, {:.4f}] mm (dir_to_far={:.1f}) -- the "
            "section's own outer contour does not extend past this web's "
            "far cap on that side, within reach.".format(
                cap_y_mm, window_min_x, window_max_x, dir_to_far
            )
        )
    return best


# cos(45 deg) -- the "roughly anti-parallel, within 45 deg" cone half-angle
# used by ``opposing_face_lateral_clearance_mm`` to decide whether a contour
# edge's own material-side normal opposes a candidate web's side wall at
# all. Two anti-parallel unit vectors have dot product -1; a dot product at
# or below -cos(45 deg) means the angle between them is at or above
# 180-45=135 deg from PARALLEL, i.e. within 45 deg of exactly anti-parallel.
_OPPOSING_FACE_COS_45 = math.cos(math.radians(45.0))


def opposing_face_lateral_clearance_mm(
    ops: GeometryOps,
    polygon,
    x_center_mm: float,
    width_mm: float,
    y_a_mm: float,
    y_b_mm: float,
    entry_lower: bool,
    safety_offset_mm: float,
) -> float:
    """True (mitre-dilated) LATERAL clearance from either of a candidate
    web's own SIDE WALLS to an OPPOSING section feature -- e.g. an open
    notch or re-entrant pocket whose own wall runs roughly parallel and
    close to the candidate, in mm.

    WHY THIS EXISTS, separate from ``far_cap_mitre_clearance_mm`` above. That
    function measures clearance from a web's FAR (deep) cap to the OPPOSITE
    surface of the section; "the two long side edges of the dilated rectangle
    are farther from the far surface than the cap and never win the minimum"
    is true only for that far/opposite-surface comparison. It does not cover a
    web whose SIDE WALL sits close to a DIFFERENT nearby feature on the SAME
    side of the section (e.g. a notch cut into the same surface the web enters
    from). Measured on an in-host failure at layer 786: a lattice web's right
    wall sat 0.84275 mm (mitre-exact segment-to-segment) from an open notch's
    left wall, comfortably "thick enough" by the vertical and far-cap checks,
    yet the validator's mitre erosion severed a knife-edge island there
    because the true geometry is a converging wedge, not a parallel corridor.
    Only a LATERAL measurement catches it.

    METHOD. For EACH of the web's two side walls (at ``x_center_mm -+
    width_mm/2``), build that wall's own MITRE-DILATED face: shift it
    outward (away from the web, into the material) by ``safety_offset_mm``,
    and extend it ``safety_offset_mm`` past each end of
    ``[y_a_mm, y_b_mm]`` (the web's own depth range, direction-agnostic).
    Dilating an axis-aligned rectangle this way, in BOTH the lateral and
    depth directions simultaneously, is EXACT for a 90-degree mitre join --
    the same fact ``far_cap_mitre_clearance_mm`` relies on for its own
    corners (see that function's docstring).

    Then scan the section's own outer contour for edges that OPPOSE this
    wall: edges whose own material-side normal (the interior-pointing
    normal of the CCW-wound contour -- rotate the edge's direction vector
    +90 degrees) is roughly ANTI-PARALLEL (within 45 degrees,
    ``_OPPOSING_FACE_COS_45``) to the wall's own material-side normal (which
    points straight out of the web, `+X` for the right wall or `-X` for the
    left), AND which lie on the wall's own material side (same side the
    wall's material-normal points into) -- an ENTRY-SKIN edge is nearly
    PERPENDICULAR to a vertical side wall's normal (dot product near 0, not
    near -1) and is excluded by the angle test alone, no separate
    entry-window bookkeeping needed.

    RESTRICTED TO THE ENTRY-SIDE CHAIN (``entry_lower``), via the SAME
    ``_split_upper_lower_chains`` chord-extreme split ``far_cap_mitre_
    clearance_mm`` uses -- NEVER the far/opposite chain. Without this
    restriction, a web entering very close to a finely tessellated leading
    edge finds a FAR-chain edge (the opposite surface, wrapping tightly around
    the LE apex) whose local tangent -- purely from LE curvature, not from any
    genuine lateral feature -- momentarily satisfies the anti-parallel test,
    returning a spurious near-zero "clearance" and wrongly omitting a
    perfectly safe web. This is the same unsoundness of an orientation-only
    edge filter that ``far_cap_mitre_clearance_mm`` avoids with chain
    membership (see its "CHAIN MEMBERSHIP, NOT SLOPE" note). A genuine lateral
    hazard (an open notch or pocket) is cut INTO the same surface the web
    enters from, so it is topologically part of the ENTRY chain, never the far
    one; the far chain's clearance is ``far_cap_mitre_clearance_mm``'s job and
    is not double-counted here. A hazard that exists purely on the far chain,
    away from the entry side, is NOT caught by this predicate -- a documented
    residual gap.

    The minimum true (mitre-dilated) distance from the dilated face to any
    qualifying entry-chain edge, across BOTH walls, is returned.

    Returns ``float("inf")`` (never raises) when no contour edge opposes
    either wall at all -- unlike ``far_cap_mitre_clearance_mm``, which can
    rely on a section always having a far side, MOST candidate webs have no
    nearby
    opposing lateral feature whatsoever, and that is the ordinary,
    overwhelmingly common case: this predicate must never manufacture a
    spurious defect (or a spurious ValueError) out of "there happens to be
    nothing nearby to measure against". This includes the case where
    ``_split_upper_lower_chains`` itself cannot classify the contour (a
    degenerate, top/bottom-symmetric section) -- caught and treated the
    same as "nothing to oppose", never propagated as a new failure mode.
    """
    if width_mm <= 0:
        raise ValueError("width_mm must be > 0, got {!r}.".format(width_mm))
    if safety_offset_mm <= 0:
        raise ValueError(
            "safety_offset_mm must be > 0, got {!r}.".format(safety_offset_mm)
        )

    half_w = width_mm / 2.0
    r = safety_offset_mm
    y_lo = min(y_a_mm, y_b_mm) - r
    y_hi = max(y_a_mm, y_b_mm) + r

    points = ops.contour_points_mm(polygon)
    n = len(points)
    if n < 3:
        raise ValueError(
            "polygon's outer contour has fewer than 3 points ({!r}); cannot "
            "measure opposing-face lateral clearance.".format(n)
        )

    try:
        upper_edges, lower_edges = _split_upper_lower_chains(points)
    except ValueError:
        return float("inf")
    entry_edges = lower_edges if entry_lower else upper_edges
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)

    best = None
    for wall_nx, wall_x in ((1.0, x_center_mm + half_w), (-1.0, x_center_mm - half_w)):
        dilated_x = wall_x + wall_nx * r
        face_a = (dilated_x, y_lo)
        face_b = (dilated_x, y_hi)
        for p, q in entry_edges:
            # A contour edge wholly on the global chord boundary is the
            # section's leading/trailing END wall, not an opposing face of
            # a re-entrant same-side feature.  Treating it as one wrongly
            # rejects otherwise-valid edge stations on blunt coupons: the
            # material simply ends there, so this edge cannot bound the
            # entry-side wedge/island this predicate exists to prevent.
            # Interior notch walls (the layer-786 mechanism) cannot satisfy
            # this exclusion because their X is strictly inside the global
            # chord extent.
            if (p[0] == min_x and q[0] == min_x) or (
                p[0] == max_x and q[0] == max_x
            ):
                continue
            ex, ey = q[0] - p[0], q[1] - p[1]
            length = math.hypot(ex, ey)
            if length < 1e-9:
                continue
            # Material-side (interior) normal for a CCW-wound contour:
            # rotate the edge's own direction vector +90 degrees (left of
            # the direction of travel is interior, the standard CCW
            # convention the mesh generators use -- see e.g.
            # make_fixtures.airfoil_triangles's outward-normal comment for the
            # mirror-image (clockwise, exterior) case).
            nx = -ey / length
            if nx * wall_nx > -_OPPOSING_FACE_COS_45:
                continue  # not anti-parallel within 45 deg -- e.g. entry skin
            mid_x = (p[0] + q[0]) / 2.0
            if wall_nx > 0.0 and mid_x <= wall_x:
                continue  # not on the wall's own material side
            if wall_nx < 0.0 and mid_x >= wall_x:
                continue
            d = _segment_distance_mm(face_a, face_b, p, q)
            if best is None or d < best:
                best = d
    return best if best is not None else float("inf")


@dataclass(frozen=True)
class SolidThicknessWorstCase:
    """Result of ``local_solid_thickness_worst_case``.

    ``solid_mm`` is the worst-case (minimum) SOLID material thickness across
    the sampled span -- see that function's docstring for exactly how it is
    derived. ``hole_extent_subtracted_mm`` is the total hole void this
    function subtracted to get there (``0.0`` when no declared hole
    overlapped the span in X). A caller carries the second field so it can
    apply its OWN meeting-condition correction for a hole-split span
    (``opposing_pair_meets``'s ``min_wall_mm`` discount -- see its
    docstring) without re-deriving the hole-overlap test.
    """

    solid_mm: float
    hole_extent_subtracted_mm: float


def local_solid_thickness_worst_case(
    ops: GeometryOps,
    polygon,
    x_center_mm: float,
    span_mm: float,
    probe_width_mm: float = DEFAULT_PROBE_WIDTH_MM,
    samples: int = DEFAULT_WORST_CASE_SAMPLES,
) -> SolidThicknessWorstCase:
    """Worst-case SOLID material thickness across a span of the chord -- exterior
    surface to exterior surface, MINUS whatever declared-hole extent falls
    inside the span. This is the primitive ``opposing_pair_meets`` needs to be
    hole-safe.

    ``t_upper - t_lower`` (which this function reduces to exactly whenever no
    hole overlaps the span -- see below) is a valid conservative LOWER BOUND on
    solid thickness only when the material between the two exterior surfaces is
    solid. At a station crossing a declared hole it is not: the hole's own void
    sits inside that span, and counting it as solid would be an OVERESTIMATE --
    exactly backwards for a quantity whose whole purpose is a conservative
    lower bound. This function corrects for that directly:

        solid_mm = full_span_mm - hole_extent_mm

    **``full_span_mm``** is ``local_upper_worst_case(..., hole_aware=True)
    - local_lower_worst_case(..., hole_aware=True)``: independent extremes
    across the span, not a per-station difference (see those two functions'
    docstrings for why that composition is itself a provable lower bound),
    hole-aware so it reads the true EXTERIOR surface on each side even at a
    station whose probe crosses the hole, instead of raising. On a span with no
    declared hole this is identical to the plain composition (``hole_aware`` is
    a no-op there).

    **``hole_extent_mm``** is the SUM of ``ops.hole_bounds(polygon)`` bbox
    heights (``max_y - min_y``) for every declared hole whose own X range
    overlaps the probed span (the sampled stations' X range, widened by
    ``probe_width_mm / 2`` on each side to match what each station's own probe
    actually touches -- deliberately widened rather than tight, since missing a
    hole that only grazes the span's edge would be the OPTIMISTIC direction). A
    hole entirely inside the span, straddling either end, or just clipping the
    widened edge all contribute their FULL bbox height, not a fraction of it --
    this deliberately over-subtracts (over-estimates the void, under-estimates
    the solid) for a hole that only partially overlaps the span in X, which is
    the safe direction: a conservative bound errs toward reporting less solid
    material, never more. A hole that does not overlap the span in X at all
    contributes nothing, and ``full_span_mm``'s hole-aware calls never reach
    the multi-piece branch, so the return value is the plain composition
    (``ops.hole_bounds`` is still called once). Using the bounding box (rather
    than the hole's true outline) is also conservative for a round hole: the
    bbox height equals the hole's true diameter at its own Y-centre but
    OVERSTATES the void everywhere else along X, so subtracting it never
    under-counts the hole.

    **The two-band case** -- a span crossing exactly one declared hole
    splits the solid material into two disjoint bands (above and below the
    hole). ``solid_mm`` as computed above is their SUM
    (``full_span_mm - hole_extent_mm`` telescopes to exactly
    ``band_above + band_below`` once each side is read from the piece
    outside the hole), not the larger band, the smaller, or an arbitrary
    choice: it is the TRUE total solid material in that vertical column, the
    quantity every "thickness across a span" measurement in this module means
    (see ``local_lower_worst_case``/``local_upper_worst_case``: "worst case
    for remaining skin"). Reporting the smaller band alone would be
    optimistic about how much material a caller could safely still remove from
    ONE side once the OTHER side already has its own budget; reporting the
    larger band alone would discard real material a caller is entitled to
    count. **However, the SUM is not by itself a sufficient conservative bound
    for a caller (like ``opposing_pair_meets``) whose formula assumes the
    surviving wall is ONE contiguous strip**: two sub-``min_wall_mm`` slivers
    (one in each band) can each individually fail to hold a bead, and a caller
    summing their two depths against the raw SUM misses that failure (worked
    example: bands of 5 mm each, ``min_wall_mm=0.5``, both cuts remove 4.7 mm
    -- each band's own 0.3 mm survivor is sub-bead, a genuine sever, but
    ``4.7 + 4.7 + 0.5 = 9.9 < 10 = SUM`` would read as safe).
    ``hole_extent_subtracted_mm`` is carried on the result so a caller with
    that shape of formula can apply its OWN correction (see
    ``opposing_pair_meets``'s ``min_wall_mm`` discount) rather than this
    function baking in a correction that belongs to one caller's arithmetic,
    not to "solid thickness" as a general quantity.

    **Multiple holes.** The design is scoped to one hole per layer, but this
    function stays SAFE rather than undefined if that does not hold:

    - Two or more holes side by side in X, each overlapping the span but no
      single sampled station crossing more than one of them: every
      overlapping hole's bbox height is summed into ``hole_extent_mm``
      unconditionally (not deduplicated, not capped) -- conservative at
      every station, since a station only ever loses the ONE hole it
      actually crosses, and summing every candidate can only subtract MORE
      than that station's own true loss, never less.
    - Two or more holes stacked in Y so that a single probed station would
      cross more than one of them (3+ pieces at that station): this is not
      resolved by summing bbox heights at all -- ``local_span``'s multi-piece
      guard (used internally by ``full_span_mm``'s hole-aware calls) only
      matches "exactly one declared hole explains exactly one gap between
      exactly two pieces" and raises ``ValueError`` for anything else, as it
      does for a genuinely re-entrant section. That propagates through this
      function unmodified -- fail closed, not a silent undercount.

    Raises ``ValueError`` exactly as ``local_upper_worst_case``/``local_
    lower_worst_case`` do (span/samples validation, no material, a
    genuinely re-entrant station, or a station crossing more holes than
    ``local_span`` resolves) -- no degrade-and-continue.
    """
    full_span_mm = local_upper_worst_case(
        ops, polygon, x_center_mm, span_mm, probe_width_mm, samples, hole_aware=True
    ) - local_lower_worst_case(
        ops, polygon, x_center_mm, span_mm, probe_width_mm, samples, hole_aware=True
    )

    half_span = span_mm / 2.0
    half_probe = probe_width_mm / 2.0
    span_min_x = x_center_mm - half_span - half_probe
    span_max_x = x_center_mm + half_span + half_probe

    hole_extent_mm = 0.0
    for hole_min_x, hole_min_y, hole_max_x, hole_max_y in ops.hole_bounds(polygon):
        if hole_max_x + EPSILON_MM >= span_min_x and hole_min_x - EPSILON_MM <= span_max_x:
            hole_extent_mm += hole_max_y - hole_min_y

    return SolidThicknessWorstCase(
        solid_mm=full_span_mm - hole_extent_mm,
        hole_extent_subtracted_mm=hole_extent_mm,
    )


@dataclass(frozen=True)
class OpposingPairMeeting:
    """Detail of an opposing-entry (+Y/-Y) meeting-condition evaluation that came
    back UNSAFE -- see ``opposing_pair_meets``. Carries exactly the numbers a
    caller's own failure/omission message needs, so a ``validate.py``
    ``TopologyError`` and a ``generator.py`` ``OmittedWeb`` reason are both
    worded from the SAME measured numbers rather than each re-deriving them
    independently."""

    overlap_min_x: float
    overlap_max_x: float
    t_local_conservative: float
    d_upper_mm: float
    d_lower_mm: float
    required_mm: float


def opposing_pair_meets(
    ops: GeometryOps,
    polygon,
    upper_min_x: float,
    upper_max_x: float,
    upper_depth_mm: float,
    lower_min_x: float,
    lower_max_x: float,
    lower_depth_mm: float,
    safety_offset_mm: float,
    min_wall_mm: float,
    probe_width_mm: float = DEFAULT_PROBE_WIDTH_MM,
) -> Optional[OpposingPairMeeting]:
    """The extended no-sever proof's opposing-entry meeting condition
    (opposing (+Y/-Y) entry), as ONE shared predicate. ``validate.py``'s
    cross-edge all-pairs check and ``generator.py``'s opposing-station omission
    check both call this directly rather than each carrying its own copy of
    proof points 1/3/4 below, so the two measurements of the same quantity
    cannot drift apart (the same hazard as measuring a margin at one station
    in placement and another in validation).

    Returns ``None`` when the pair is SAFE: either their haloed (widened by
    ``safety_offset_mm`` on every side) X footprints do not overlap at all
    (proof point 1 -- no shared span, so the meeting condition has nothing
    to be evaluated over regardless of depth), or they interact but clear
    the meeting condition with a positive wall remaining.

    Returns an ``OpposingPairMeeting`` when the pair MEETS (unsafe):

        d_upper_mm + d_lower_mm + min_wall_mm >= t_local_conservative - EPSILON_MM

    deliberately failing closed on EXACT equality (proof point 4 -- the
    OPPOSITE epsilon bias from ``validate.py``'s same-edge collision check,
    because at equality the wall between the two opposing cuts is already
    zero width before the safety offset even runs, not a thin positive
    one).

    HOLE-AWARE. ``t_local_conservative`` is measured via
    ``local_solid_thickness_worst_case`` -- see that function's docstring for
    the full derivation. The plain ``t_upper - t_lower`` composition is a
    provable LOWER BOUND on true thickness only when the material between the
    two exterior surfaces is solid; at a station crossing a declared hole it is
    not, and counting the hole's void as solid would be an OVERESTIMATE,
    exactly backwards for a check whose whole purpose is a conservative lower
    bound. ``local_solid_thickness_worst_case`` corrects for this directly
    (exterior-to-exterior span minus whatever hole extent falls inside it) and
    equals the plain composition whenever no declared hole overlaps the overlap
    span, so a hole-free opposing-entry check is unaffected.

    When a hole DID overlap the span, ``local_solid_thickness_worst_case``'s
    docstring names a gap in a naive sum: the surviving material splits into
    two bands (above and below the hole), and this function's
    meeting-condition arithmetic assumes one contiguous wall. Two
    sub-``min_wall_mm`` slivers -- one in each band -- can each individually
    fail to hold a bead without the raw sum ever detecting it (see that
    docstring's worked example). This function closes that gap with a one-term
    discount: whenever ``local_solid_thickness_worst_case`` reports a nonzero
    ``hole_extent_subtracted_mm`` (a hole genuinely overlapped the span),
    ``t_local_conservative`` is reduced by ``min_wall_mm`` on top of the sum.
    This is TIGHT for a scalar check: the true meeting condition at a hole
    station is "both bands are individually consumed to below ``min_wall_mm``"
    (``d_upper_mm >= band_above - min_wall_mm`` AND ``d_lower_mm >= band_below
    - min_wall_mm``), which algebraically implies ``d_upper_mm + d_lower_mm >=
    (band_above + band_below) - 2 * min_wall_mm`` -- adding ``min_wall_mm`` to
    both sides recovers exactly this function's ``required_mm >=
    t_local_conservative - EPSILON_MM`` shape with ``t_local_conservative = sum
    - min_wall_mm``. It over-flags (rejects an actually-safe pair) only when
    the ENTIRE surviving wall happens to live in one band and the other band is
    already fully spent -- bounded by ``min_wall_mm`` of accepted
    over-conservatism, the safe direction.

    ``t_local_conservative`` is measured over the INTERSECTION of the two
    haloed X ranges (the "overlap span"), via
    ``local_solid_thickness_worst_case`` (proof point 3 -- a provable LOWER
    BOUND on the true solid thickness anywhere in that span, not merely a
    plausible estimate at one station). Propagates ``ValueError`` exactly as
    ``local_upper_worst_case``/``local_lower_worst_case`` do if the overlap
    span's local geometry cannot be measured at all (e.g. a genuinely
    re-entrant station, or a station crossing more than one declared hole) -- a
    caller that needs a softer failure (a best-effort diagnostic, never a
    placement or validation decision) must catch it itself; this function does
    not degrade-and-continue on its own.

    Depths (``upper_depth_mm``/``lower_depth_mm``) and footprints are
    supplied by the CALLER, never measured here, because the two callers
    measure them from different things by construction: ``validate.py``
    measures the ACTUALLY REMOVED region's own bounding box (the only honest
    post-boolean reading available to an independent check), while
    ``generator.py`` measures its own REQUESTED placement before any boolean
    runs (the generator performs no boolean cut itself). Sharing this formula
    does not erase the difference between an intended footprint and an
    actually-cut one; it guarantees the ARITHMETIC each caller applies to
    whatever footprint it honestly has cannot diverge from the other's.
    """
    halo_u_min_x = upper_min_x - safety_offset_mm
    halo_u_max_x = upper_max_x + safety_offset_mm
    halo_l_min_x = lower_min_x - safety_offset_mm
    halo_l_max_x = lower_max_x + safety_offset_mm
    overlap_min_x = max(halo_u_min_x, halo_l_min_x)
    overlap_max_x = min(halo_u_max_x, halo_l_max_x)
    overlap_span_mm = overlap_max_x - overlap_min_x
    if overlap_span_mm <= 0:
        return None

    overlap_center_x = (overlap_min_x + overlap_max_x) / 2.0
    solid = local_solid_thickness_worst_case(
        ops, polygon, overlap_center_x, overlap_span_mm, probe_width_mm
    )
    t_local_conservative = solid.solid_mm
    if solid.hole_extent_subtracted_mm > 0.0:
        # A declared hole overlapped the span: the surviving material is
        # two bands, not one contiguous wall -- see this function's own
        # docstring and local_solid_thickness_worst_case's worked example
        # for why the raw sum alone is not a safe meeting-condition bound.
        t_local_conservative -= min_wall_mm

    required_mm = upper_depth_mm + lower_depth_mm + min_wall_mm
    if required_mm >= t_local_conservative - EPSILON_MM:
        return OpposingPairMeeting(
            overlap_min_x=overlap_min_x,
            overlap_max_x=overlap_max_x,
            t_local_conservative=t_local_conservative,
            d_upper_mm=upper_depth_mm,
            d_lower_mm=lower_depth_mm,
            required_mm=required_mm,
        )
    return None


@dataclass(frozen=True)
class LayerFrame:
    """Reference geometry for one layer cross-section.

    ``min_x``/``min_y``/``max_x``/``max_y`` are the polygon's axis-aligned
    bounding box in millimetres, used for the chord axis (X): a
    constant-section wing's true chord extent equals its bounding-box X
    extent, leading edge to trailing edge, so no local measurement is
    needed there (unlike Y — see the module docstring).

    ``ops``/``polygon`` are kept so ``web_centerline`` can call
    ``local_span`` (and ``local_lower_worst_case``/``local_upper_worst_case``)
    without a caller needing to pass them through — this is what lets
    ``StructureGenerator.generate()`` call
    ``frame.web_centerline(web_position, skin_margin_mm, slot_width_mm)``
    without also threading a ``GeometryOps``, even though the answer
    depends on a live geometry query.

    ``rotation_deg`` is this section's own estimated rotation about the chord
    axis — see ``estimate_rotation_deg``. Computed once, here, at construction,
    so every caller measuring margins against this frame (``web_centerline``,
    ``StructureGenerator``) uses the SAME angle rather than each re-estimating
    it (and potentially disagreeing, the same discipline this module applies
    to worst-case span sampling).
    """

    min_x: float
    min_y: float
    max_x: float
    max_y: float
    ops: GeometryOps = field(compare=False, repr=False)
    polygon: object = field(compare=False, repr=False)
    rotation_deg: float = 0.0

    @classmethod
    def from_polygon(cls, ops: GeometryOps, polygon) -> "LayerFrame":
        """Build a frame from one polygon's bounding box via ``ops.bounds``, and
        its estimated rotation via ``estimate_rotation_deg``."""
        min_x, min_y, max_x, max_y = ops.bounds(polygon)
        rotation_deg = estimate_rotation_deg(ops, polygon)
        return cls(
            min_x=min_x,
            min_y=min_y,
            max_x=max_x,
            max_y=max_y,
            ops=ops,
            polygon=polygon,
            rotation_deg=rotation_deg,
        )

    @property
    def width(self) -> float:
        """Extent along the chord axis (X), in millimetres."""
        return self.max_x - self.min_x

    @property
    def depth(self) -> float:
        """Bounding-box extent along the thickness axis (Y), in millimetres.

        This is the section's overall Y extent, NOT the local thickness at
        any particular chord station — see ``local_span`` for that.
        """
        return self.max_y - self.min_y

    def web_x(self, web_position: float) -> float:
        """X coordinate of the web centerline for a given chord fraction.

        ``web_position`` is a fraction in [0, 1] across the chord axis;
        0.0 is the -X edge, 1.0 is the +X edge, 0.5 is the coupon centerline.
        """
        return self.min_x + web_position * self.width

    def web_centerline(
        self,
        web_position: float,
        skin_margin_mm: float,
        slot_width_mm: float,
        probe_width_mm: float = DEFAULT_PROBE_WIDTH_MM,
        max_rotation_deg: float = DEFAULT_MAX_ROTATION_DEG,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Endpoints of the web centerline for ``web_position``.

        First checks this frame's own ``rotation_deg`` against
        ``max_rotation_deg`` (``check_rotation_envelope`` — raises past it,
        naming the measured angle, the threshold, and why), then uses
        ``corrected_skin_margin_mm`` in place of ``skin_margin_mm`` raw, so the
        margin this function guarantees is the TRUE (chord-perpendicular)
        margin, not the world-Y one local_span measures — see the module
        docstring's "ROTATION ENVELOPE" note. A rotation within the deadzone
        (``_ROTATION_DEADZONE_DEG``) applies no correction, so an unrotated
        section's ``margin_point`` uses the raw margin.

        Returns ``(boundary_point, margin_point)``, both measured against
        the WORST CASE of the LOCAL surfaces across the slot's own width
        (``local_lower_worst_case``/``local_upper_worst_case``), not a
        single-station probe at the centreline and not the section's global
        bounding-box extremes:

        - ``boundary_point`` sits at the LOWEST LOCAL +Y exterior boundary
          found across ``[x - slot_width_mm/2, x + slot_width_mm/2]`` — the
          slot's cut must start at least this high everywhere across its
          width so it opens the skin rather than leaving an interior void
          anywhere along that width.
        - ``margin_point`` sits ``skin_margin_mm`` above the HIGHEST LOCAL -Y
          skin found across that same span — the worst case for remaining
          skin, so the margin holds everywhere across the slot's width, not
          only at its centreline.

        ``slot_width_mm`` MUST be the actual resolved slot width (the same
        value ``SlotBuilder`` will build the rectangle with — see
        ``StructureGenerator.generate``'s call site) so the span this
        function measures over is the span the slot will actually occupy;
        this is what keeps ``validate.py`` — which measures the same worst
        case over the removed region's own actual X span — unable to
        disagree with this placement.

        Raises ``ValueError`` if ``skin_margin_mm`` would leave no room for
        the slot at all (margin point beyond or at the boundary point) — a
        misconfigured spec should fail loudly here rather than produce a
        zero- or negative-length centerline that ``slot.py`` would then
        silently degrade. Also raises (via ``local_span``) if any sampled
        chord station across the slot's width is off the section, or
        re-entrant.
        """
        check_rotation_envelope(self.rotation_deg, max_rotation_deg)
        effective_margin_mm = corrected_skin_margin_mm(skin_margin_mm, self.rotation_deg)

        x = self.web_x(web_position)
        y_max = local_upper_worst_case(
            self.ops, self.polygon, x, slot_width_mm, probe_width_mm
        )
        y_min = local_lower_worst_case(
            self.ops, self.polygon, x, slot_width_mm, probe_width_mm
        )
        boundary_point = (x, y_max)
        margin_point = (x, y_min + effective_margin_mm)
        if margin_point[1] >= boundary_point[1]:
            raise ValueError(
                "skin_margin_mm={!r} leaves no room for a web centerline at "
                "chord station x={!r} mm (local thickness {!r} mm: boundary "
                "at y={!r}, margin point at y={!r}, effective margin after "
                "the section's own {:.4f} degree rotation correction: "
                "{!r} mm).".format(
                    skin_margin_mm,
                    x,
                    y_max - y_min,
                    boundary_point[1],
                    margin_point[1],
                    self.rotation_deg,
                    effective_margin_mm,
                )
            )
        return boundary_point, margin_point
