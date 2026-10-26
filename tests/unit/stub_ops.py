"""Exact-arithmetic ``GeometryOps`` stub — AXIS-ALIGNED RECTANGLES ONLY.

This stub exists to test ``aircraft_core``'s logic in isolation from a real
geometry kernel. It is NOT a geometry kernel and must never be treated like
one:

- It understands exactly three shapes, all axis-aligned and built from
  millimetre floats with no tolerance beyond ``EPSILON_MM`` below:

  - ``Rect``: a plain rectangle.
  - ``NotchedRect``: a rectangle with one rectangular notch removed from its
    top (+Y) edge, not reaching the bottom edge and not reaching the left or
    right edges — the "U-shape" the design describes. Represented
    directly by its outer rectangle and its notch rectangle, NOT by an
    explicit point list; ``diff``/``bounds``/``area``/``contour_count`` are
    computed analytically from those two rectangles, not by any general
    polygon algorithm.
  - ``HoledRect``: a rectangle with one fully-interior rectangular hole
    (touches none of the four outer edges) — used only to exercise the
    "enclosed hole" validator failure.
  - ``MultiHoledRect``: the third-step analogue of ``HoledRect`` for
    MORE THAN ONE interior hole — used only to exercise ``frame.local_
    solid_thickness_worst_case``'s documented multi-hole behaviour (see its
    own docstring). Not wired into ``diff``/``offset``, since nothing else
    in this codebase needs more than one hole per layer.
  - ``PolygonShape``: an arbitrary simple polygon as a list of ``(x, y)``
    mm points — the stand-in for a real (e.g. airfoil) section, added for
    local-thickness measurement. Unlike the other three shapes it is
    not axis-aligned or convex by construction; it is whatever points the
    test hands it.
  - ``MultiNotchedRect``: the analogue of ``NotchedRect`` for MORE THAN
    ONE web on a rectangular coupon — a rectangle with several rectangular
    notches removed from its top (+Y) edge, none reaching the bottom edge
    or the left/right edges, and none overlapping another in X (not
    re-validated here; callers must construct sane fixtures). Used only to
    exercise ``TopologyValidator``'s per-slot loop and its collision
    check against a constructed "already correctly cut" transformed shape —
    like ``NotchedRect``, it is not built by any general Boolean
    subtraction here. ``StubOps.offset`` on this shape is DELIBERATELY
    NARROW (see ``_offset_multi_notched_rect``'s own docstring): it only
    answers "the shared base bridge beneath every notch clearly survives
    erosion, so the whole shape stays one connected island", assuming every
    notch shares the SAME bridge depth (true for a flat rectangular coupon
    under a uniform ``skin_margin_mm`` — every unit test that erodes a
    ``MultiNotchedRect`` only needs that case, since the SEPARATE
    pairwise collision check is what catches an undersized wall BETWEEN two notches
    before erosion is ever reached; see ``aircraft_core/validate.py``'s own
    reasoning for why that split exists). The bridge-fails/severed-island
    case for more than one notch is not modelled; extend deliberately if a
    future test needs it.
  - ``TwoSidedNotchedRect``: the analogue of ``NotchedRect`` for a pair
    of OPPOSING (+Y and -Y) webs on a rectangular coupon — a rectangle with
    one notch removed from the +Y edge and one from the -Y edge, both
    sharing the same X range (partial X-footprint overlap between the two
    notches is NOT modelled by this shape; see its own docstring). Used only
    to exercise the geometric TRUTH of the no-sever proof's "meeting
    condition" directly (piece count under erosion), independent of
    ``TopologyValidator`` — which does
    not implement an opposing-entry check at all (every invariant 3/4
    check still assumes +Y-only entry; see ``docs/geometry_invariants.md``).
  - ``NotchedPolygon``: the analogue of ``NotchedRect`` — an arbitrary
    ``PolygonShape`` (e.g. a real airfoil section) minus one rectangular
    notch, used ONLY to exercise ``TopologyValidator`` against a
    non-rectangular ORIGINAL without reimplementing general
    polygon-minus-rectangle Boolean subtraction (a real geometry-kernel
    problem this stub deliberately does not attempt). Represented directly
    by its outer ``PolygonShape`` and its notch ``Rect``, exactly like
    ``NotchedRect``; see its own docstring for what is, and is not, exact
    about it.
  - ``PinchedNotchedPolygon``: ``NotchedPolygon`` extended with an
    independent thin "waist" elsewhere in the ORIGINAL material, unrelated
    to any slot — added to prove that ``TopologyValidator``'s whole-layer
    erosion check (the LAST-resort catch-all after invariants 3/4 and the
    per-pair collision/opposing checks) is not dead code once
    ``StructureSpec.validate(safety_offset_mm=...)`` guards
    ``skin_margin_mm``: a single-sided cut's own bridge is PROVABLY safe
    once ``skin_margin_mm > 2 * safety_offset_mm`` holds (see
    ``docs/geometry_invariants.md``'s FRAGILITY WARNING), so the only
    remaining geometry-driven disconnection under a fully valid spec is one
    no PER-SLOT invariant ever measures — a thin neck in the original
    section itself, away from any web. ``offset()`` on this shape checks
    the notch's own bridge exactly as ``NotchedPolygon`` does, AND
    independently the waist's stored thickness against the same erosion
    width; if the waist does not survive it returns two placeholder pieces
    (only the COUNT is load-bearing, same convention as every other shape
    here — see ``_offset_pinched_notched_polygon``'s own docstring).

- ``diff()`` implements exactly the handful of rectangle-minus-rectangle
  cases tests need (no overlap, full vertical split, top-edge notch,
  interior hole) and asserts loudly if handed a combination it does not
  recognize. It is not a general Boolean engine: it cannot union, cannot
  handle rotated or non-axis-aligned input, and does not clip a clip that
  only partially overlaps the subject in more exotic ways than those four.

- ``offset()`` erodes/grows a ``Rect`` exactly. For ``NotchedRect`` it does
  NOT attempt to compute the true eroded polygon (that is a real
  non-convex-offsetting problem); it instead determines, exactly, whether
  every load-bearing width in the shape (each arm beside the notch, and the
  bridge below it) survives a uniform erosion of ``-delta`` on every
  material-to-air boundary (i.e. shrinks by ``2 * |delta|`` per constrained
  dimension), and returns 0, 1, or 2 pieces:

    * the outer envelope is consumed -> ``[]``;
    * the bridge below the notch is consumed while both arms survive ->
      TWO disjoint ``Rect`` arms, the severed-island case
      ``TopologyValidator``'s erosion check exists to catch;
    * the arms are consumed while the bridge holds -> one piece (see the
      KNOWN VALIDATOR GAP note at that branch: sub-bead-width arms still
      read as one island and pass);
    * otherwise -> one approximately-shrunk ``NotchedRect``.

  This is exact for the *count* of resulting polygons, which is all the
  safety-offset check consumes — it is not exact for the resulting
  polygon's geometry.

- ``intersection()`` implements ``subject`` clipped to ``clip`` for a
  convex, axis-aligned rectangular ``clip`` only (``NotImplementedError``
  otherwise) — see its own docstring for exactly what it guarantees and
  why (single-piece results are exact via Sutherland-Hodgman; multi-piece
  results are exact for bounds()/area() at the probe, not for outline
  shape). ``HoledRect`` is ALSO now a recognized ``subject`` (see
  ``_intersection_holed_rect``'s own docstring) — the one shape besides
  ``Rect``/``PolygonShape`` this method accepts, dispatched before
  ``_as_point_list`` (which still does not, and is not asked to, recognize
  it).

- ``polygon_from_points()`` returns a ``PolygonShape`` wrapping the given
  points verbatim — used to build the thin measurement strips
  ``frame.local_span`` clips sections with.

- Every call is logged to ``self.calls`` (method name + arg summary) so
  tests can assert the ops object was, or was not, invoked at all.

If a test needs a shape or operation this stub doesn't cover, extend the
stub deliberately and re-document the limit — do not silently generalize it
into something that looks like a real geometry kernel.
"""

from dataclasses import dataclass, field
from typing import List, Tuple

from aircraft_core.frame import DEFAULT_PROBE_WIDTH_MM, local_span
from aircraft_core.ops import GeometryOps

EPSILON_MM = 1e-9


@dataclass(frozen=True)
class Rect:
    """An axis-aligned rectangle: the stub's only "simple" polygon shape."""

    min_x: float
    min_y: float
    max_x: float
    max_y: float

    def area(self) -> float:
        return max(0.0, self.max_x - self.min_x) * max(0.0, self.max_y - self.min_y)

    def bounds(self) -> Tuple[float, float, float, float]:
        return (self.min_x, self.min_y, self.max_x, self.max_y)


@dataclass(frozen=True)
class NotchedRect:
    """``outer`` minus ``notch``, where ``notch`` touches only the +Y edge.

    Requires (not re-validated here, callers must construct sane fixtures):
    ``notch.max_y == outer.max_y`` (touches the top edge), and
    ``outer.min_x < notch.min_x`` and ``notch.max_x < outer.max_x`` and
    ``outer.min_y < notch.min_y`` (strictly inside on the other three sides).
    """

    outer: Rect
    notch: Rect

    def area(self) -> float:
        return self.outer.area() - self.notch.area()

    def bounds(self) -> Tuple[float, float, float, float]:
        return self.outer.bounds()

    def left_arm(self) -> float:
        return self.notch.min_x - self.outer.min_x

    def right_arm(self) -> float:
        return self.outer.max_x - self.notch.max_x

    def bottom_bridge(self) -> float:
        return self.notch.min_y - self.outer.min_y


@dataclass(frozen=True)
class MultiNotchedRect:
    """``outer`` minus several notches, all touching the +Y edge — the stand-in
    for several parallel webs on a rectangular coupon. See module docstring for
    exactly what this shape does and does not model."""

    outer: Rect
    notches: Tuple[Rect, ...]

    def area(self) -> float:
        return self.outer.area() - sum(n.area() for n in self.notches)

    def bounds(self) -> Tuple[float, float, float, float]:
        return self.outer.bounds()


@dataclass(frozen=True)
class TwoSidedNotchedRect:
    """``outer`` minus TWO notches, one from the +Y edge and one from the -Y edge
    — the no-sever-proof stand-in for a pair of OPPOSING webs on a rectangular
    coupon (``docs/geometry_invariants.md``, "Extended no-sever proof —
    opposing (+Y/-Y) entry").

    Requires (not re-validated here, callers must construct sane fixtures,
    same convention as ``NotchedRect``): ``notch_top`` touches only the +Y
    edge, ``notch_bottom`` touches only the -Y edge, BOTH SHARE THE SAME X
    RANGE (``notch_top.min_x == notch_bottom.min_x`` and
    ``notch_top.max_x == notch_bottom.max_x``) — this stub does not model
    partial X-footprint overlap between the two notches; see the module
    docstring's "TwoSidedNotchedRect" note and
    ``docs/geometry_invariants.md`` for why the partial-overlap case is
    exercised at the ``frame.py`` span-measurement level instead, not
    through this shape.
    """

    outer: Rect
    notch_top: Rect
    notch_bottom: Rect

    def area(self) -> float:
        return self.outer.area() - self.notch_top.area() - self.notch_bottom.area()

    def bounds(self) -> Tuple[float, float, float, float]:
        return self.outer.bounds()

    def left_arm(self) -> float:
        return self.notch_top.min_x - self.outer.min_x

    def right_arm(self) -> float:
        return self.outer.max_x - self.notch_top.max_x

    def middle_wall(self) -> float:
        """The wall of surviving material BETWEEN the two opposing notches
        — the "meeting condition" quantity: ``notch_top`` reaches DOWN to
        ``notch_top.min_y`` and ``notch_bottom`` reaches UP to
        ``notch_bottom.max_y``; the material between them is what the
        no-sever proof's ``d_upper + d_lower + min_wall < t_local`` condition
        is about."""
        return self.notch_top.min_y - self.notch_bottom.max_y


@dataclass(frozen=True)
class PolygonShape:
    """An arbitrary simple (non-self-intersecting) polygon, as a CCW-or-CW
    ring of ``(x, y)`` mm points. This is the stub's stand-in for a real
    airfoil section — unlike ``Rect``/``NotchedRect``, it makes no
    assumption about axis-alignment or convexity of the SUBJECT.
    """

    points: Tuple[Tuple[float, float], ...]

    def area(self) -> float:
        return abs(_signed_area(self.points))

    def bounds(self) -> Tuple[float, float, float, float]:
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        return (min(xs), min(ys), max(xs), max(ys))


@dataclass(frozen=True)
class NotchedPolygon:
    """``outer`` (a ``PolygonShape``) minus ``notch`` (a ``Rect``).

    Requires (not re-validated here, callers must construct sane fixtures,
    same convention as ``NotchedRect``): ``notch`` lies entirely within
    ``outer``'s material and touches only the exterior boundary the slot is
    meant to open from.

    ``area()`` is exact for the same reason ``NotchedRect.area()`` is:
    simple subtraction, valid because the notch is assumed non-overlapping
    with anything outside ``outer``'s material. ``bounds()`` is ``outer``'s
    own bounding box — unaffected by removing an edge-touching notch.
    """

    outer: PolygonShape
    notch: Rect

    def area(self) -> float:
        return self.outer.area() - self.notch.area()

    def bounds(self) -> Tuple[float, float, float, float]:
        return self.outer.bounds()


@dataclass(frozen=True)
class PinchedNotchedPolygon:
    """``NotchedPolygon`` plus one independent thin "waist" elsewhere in
    ``outer``, unrelated to ``notch``. See the module docstring's own entry
    for why this exists (proving the whole-layer erosion catch-all still
    fires under a fully valid ``skin_margin_mm`` for a cause no per-slot
    invariant measures).

    ``waist_x_mm`` is the chord station of the pinch; ``waist_thickness_mm``
    is a DECLARED (not point-list-derived) material thickness there — a
    synthetic knob, the same convention ``NotchedRect``/``NotchedPolygon``
    use for their own exact geometry parameters rather than deriving
    everything from raw points. Callers must construct ``outer``'s point
    list so that ``waist_thickness_mm`` is honest (this is not
    re-validated here, same convention as every other shape in this
    module).

    ``area()``/``bounds()`` behave exactly like ``NotchedPolygon``'s own —
    the waist does not change either (it is a thickness annotation, not a
    hole).
    """

    outer: PolygonShape
    notch: Rect
    waist_x_mm: float
    waist_thickness_mm: float

    def area(self) -> float:
        return self.outer.area() - self.notch.area()

    def bounds(self) -> Tuple[float, float, float, float]:
        return self.outer.bounds()


@dataclass(frozen=True)
class HoledRect:
    """``outer`` minus ``hole``, where ``hole`` touches none of the four edges."""

    outer: Rect
    hole: Rect

    def area(self) -> float:
        return self.outer.area() - self.hole.area()

    def bounds(self) -> Tuple[float, float, float, float]:
        return self.outer.bounds()


@dataclass(frozen=True)
class MultiHoledRect:
    """``outer`` minus MULTIPLE fully-interior holes — third step multi-hole test
    fixture, generalizing ``HoledRect`` to more than one hole. Used only to
    exercise ``frame.local_solid_thickness_worst_case``'s documented multi-hole
    behaviour (holes side by side in X are summed; holes stacked so a single
    probe crosses more than one fall through to ``local_span``'s own existing
    multi-piece guard and raise) — nothing else in this codebase needs more
    than one hole per layer (own design proof stays scoped to one), so this
    shape is not wired into ``diff``/``offset`` the way ``HoledRect`` is.
    """

    outer: Rect
    holes: Tuple[Rect, ...]

    def area(self) -> float:
        return self.outer.area() - sum(h.area() for h in self.holes)

    def bounds(self) -> Tuple[float, float, float, float]:
        return self.outer.bounds()


@dataclass(frozen=True)
class ScarredHoledRect:
    """``outer`` minus ``hole``, minus a vertical scar of ``scar_width_mm``
    connecting the hole's near boundary to whichever exterior edge
    ``entry_lower`` names (``False``: +Y/top; ``True``: -Y/bottom), centred in
    X on ``hole``. The "already correctly scarred transformed layer" stand-in —
    used ONLY to exercise ``TopologyValidator``'s whole-layer erosion check
    (``validate.py``) against scar geometry, and to demonstrate
    ``contour_count() == 1`` for a correctly bridged layer (whether invariant 2
    needs to change at all —'s Step 4 section: it does not).

    Requires (not re-validated here, callers must construct sane fixtures,
    same convention as every other shape in this module):
    ``scar_width_mm < hole``'s own width, and the scar's own far end lies
    strictly between the hole's near and far boundaries.

    THE CONNECTIVITY MODEL ``offset()`` USES, derived once here rather than
    re-derived per call: because the scar's own Y-extent runs from past the
    hole's near boundary all the way past the entered exterior edge, it
    fully PERFORATES the material directly between the hole and that edge,
    within the scar's own X width -- there is no thin "bridge" left there
    to model (unlike ``NotchedRect``'s notch, which stops short of the far
    edge). What remains is exactly TWO fragments of "cap" material, one on
    each side of the scar, each ``(hole_width_mm - scar_width_mm) / 2``
    wide, each spanning the full height between the hole and the entered
    edge, each already touching (zero gap, pre-erosion) its own side arm
    (the full-height material strictly outside the hole's own X range,
    which no cut in this shape ever reaches). The only question erosion
    can answer is whether each fragment's own connecting width survives —
    modelled with the SAME "a wall survives erosion iff its own width
    exceeds ``2 * |delta_mm|``" convention this stub already uses
    throughout (``NotchedRect``'s own arm-survival check; this project's
    own ``min_wall_mm = 2 * safety_offset_mm`` convention), not a full 2D
    corner-erosion simulation. A fragment that fails this becomes a
    separate, disconnected sliver — this stub's SEVERED-ISLAND convention
    (return more than one piece; only the COUNT is load-bearing, same as
    ``NotchedRect``'s own severed case).
    """

    outer: Rect
    hole: Rect
    scar_width_mm: float
    entry_lower: bool = False
    # DECLARED (not point-list-derived) area the scar itself removes BEYOND
    # the hole's own area -- same "declared knob" convention
    # ``PinchedNotchedPolygon.waist_thickness_mm`` already uses for a
    # synthetic parameter this stub does not derive from raw points.
    # Callers must supply the scar's own true non-overlapping area (its
    # width times the span from the hole's near boundary to the entered
    # exterior edge) so ``area()``/invariant 6's "something was removed"
    # check stays honest -- without this, a scar whose own footprint does
    # not overlap the hole's Y-range at all (the ordinary case) would
    # silently vanish from the reported area.
    scar_extra_area_mm2: float = 0.0

    def area(self) -> float:
        return self.outer.area() - self.hole.area() - self.scar_extra_area_mm2

    def bounds(self) -> Tuple[float, float, float, float]:
        return self.outer.bounds()

    def fragment_width_mm(self) -> float:
        hole_width_mm = self.hole.max_x - self.hole.min_x
        return (hole_width_mm - self.scar_width_mm) / 2.0


def _signed_area(points) -> float:
    """Shoelace signed area (positive for CCW winding)."""
    n = len(points)
    total = 0.0
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        total += x0 * y1 - x1 * y0
    return total / 2.0


def _as_point_list(polygon):
    """Normalize a stub polygon to a plain ``(x, y)`` point tuple list.

    Only ``Rect`` and ``PolygonShape`` are recognized as ``intersection``
    subjects/clips — ``NotchedRect``/``HoledRect`` are only ever used as
    already-transformed (post-cut) shapes in validator tests and are not
    needed here.
    """
    if isinstance(polygon, PolygonShape):
        return list(polygon.points)
    if isinstance(polygon, Rect):
        return [
            (polygon.min_x, polygon.min_y),
            (polygon.max_x, polygon.min_y),
            (polygon.max_x, polygon.max_y),
            (polygon.min_x, polygon.max_y),
        ]
    raise NotImplementedError(
        "StubOps.intersection does not support {!r} as a subject/clip — "
        "only Rect and PolygonShape are recognized. See module "
        "docstring.".format(type(polygon))
    )


def _is_convex(points) -> bool:
    """Whether ``points`` (a simple polygon ring) is convex.

    Consecutive turns must all be the same sign (allowing collinear/zero
    cross products); mixed signs mean a reflex vertex.
    """
    n = len(points)
    if n < 3:
        return False
    positive = negative = False
    for i in range(n):
        ax, ay = points[i]
        bx, by = points[(i + 1) % n]
        cx, cy = points[(i + 2) % n]
        cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx)
        if cross > EPSILON_MM:
            positive = True
        elif cross < -EPSILON_MM:
            negative = True
        if positive and negative:
            return False
    return True


def _is_axis_aligned_rect(points) -> bool:
    """Whether ``points`` forms an axis-aligned rectangle (any 4-point
    ordering — CCW or CW, starting anywhere)."""
    if len(points) != 4:
        return False
    xs = sorted({round(p[0], 9) for p in points})
    ys = sorted({round(p[1], 9) for p in points})
    if len(xs) != 2 or len(ys) != 2:
        return False
    expected = {(xs[0], ys[0]), (xs[1], ys[0]), (xs[1], ys[1]), (xs[0], ys[1])}
    got = {(round(p[0], 9), round(p[1], 9)) for p in points}
    return got == expected


def _sh_is_inside(p, a, b) -> bool:
    """Half-plane test for Sutherland-Hodgman: is ``p`` on the interior
    side of directed clip edge ``a -> b`` (clip wound CCW)?"""
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= -EPSILON_MM


def _sh_edge_intersection(p1, p2, a, b):
    """Intersection of segment ``p1 -> p2`` with the infinite line through
    ``a -> b``."""
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = a
    x4, y4 = b
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-12:
        return p2  # parallel; degenerate, no real crossing
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))


def _sutherland_hodgman(subject_points, clip_points):
    """Classic Sutherland-Hodgman polygon clip: ``subject_points`` clipped
    to the convex region bounded by ``clip_points``.

    EXACT, but only when ``subject`` intersected with ``clip`` is a single
    connected region — see ``StubOps.intersection``'s docstring for why
    that precondition is checked before this is ever called on a subject
    that might not satisfy it.
    """
    if _signed_area(clip_points) < 0:
        clip_points = list(reversed(clip_points))
    output = list(subject_points)
    n = len(clip_points)
    for i in range(n):
        a = clip_points[i]
        b = clip_points[(i + 1) % n]
        if not output:
            break
        input_list = output
        output = []
        prev = input_list[-1]
        prev_inside = _sh_is_inside(prev, a, b)
        for curr in input_list:
            curr_inside = _sh_is_inside(curr, a, b)
            if curr_inside:
                if not prev_inside:
                    output.append(_sh_edge_intersection(prev, curr, a, b))
                output.append(curr)
            elif prev_inside:
                output.append(_sh_edge_intersection(prev, curr, a, b))
            prev, prev_inside = curr, curr_inside
    return output


def _vertical_scanline_intervals(points, x_probe):
    """Y-intervals where the simple polygon ``points`` has material at
    ``x = x_probe``, via the standard ray-casting rule: collect the Y of
    every edge crossing, sort, and pair consecutive crossings into filled
    intervals ``[y[0], y[1]], [y[2], y[3]], ...``.

    Raises ``ValueError`` if the probe line grazes a vertex in a way that
    leaves an odd crossing count — callers are expected to choose
    non-degenerate stations, same as any ray-casting point-in-polygon test.

    Uses a CLOSED interval test (``min(x0, x1) <= x_probe <= max(x0, x1)``),
    not the half-open convention textbook point-in-polygon ray casting
    normally uses. Half-open would silently drop a probe that lands exactly
    on a polygon's own extreme X — which ``web_position=0.0``/``1.0`` do by
    construction (``web_x()`` returns ``min_x``/``max_x`` exactly) — because
    a vertex shared by two edges only gets counted from one of them. Closed
    intervals count it from both adjacent (non-vertical) edges instead,
    which pairs up correctly for the axis-aligned rectangles this stub
    supports; a genuinely ambiguous tangent graze elsewhere still surfaces
    via the odd-crossing-count check below.
    """
    ys = []
    n = len(points)
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        if x0 == x1:
            continue  # vertical edge: parallel to the probe, no crossing
        lo, hi = (x0, x1) if x0 <= x1 else (x1, x0)
        if lo <= x_probe <= hi:
            t = (x_probe - x0) / (x1 - x0)
            ys.append(y0 + t * (y1 - y0))
    ys.sort()
    if len(ys) % 2 != 0:
        raise ValueError(
            "odd number of crossings ({}) at x={!r}; probe line grazes a "
            "vertex exactly — choose a different station.".format(len(ys), x_probe)
        )
    return [(ys[i], ys[i + 1]) for i in range(0, len(ys), 2)]


def _as_rect_bounds(polygon) -> Tuple[float, float, float, float]:
    """Normalize any stub polygon or raw point list to a bounding box.

    Accepts ``Rect``/``NotchedRect``/``HoledRect`` (via their ``.bounds()``)
    and a raw iterable of ``(x, y)`` points (e.g. ``SlotPolygon.points`` from
    ``aircraft_core.slot``) by taking its exact bounding box — exact only
    because every point list this stub is ever handed is, by construction of
    the tests, an axis-aligned rectangle's four corners.
    """
    if hasattr(polygon, "bounds"):
        return polygon.bounds()
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return (min(xs), min(ys), max(xs), max(ys))


class StubOps(GeometryOps):
    """See module docstring: exact, axis-aligned-rectangle-only, test-only."""

    def __init__(self):
        self.calls: List[Tuple[str, tuple]] = []

    # -- GeometryOps ------------------------------------------------------

    def diff(self, subject, clips):
        self.calls.append(("diff", (subject, tuple(clips))))
        result = [subject]
        for clip in clips:
            next_result = []
            for piece in result:
                next_result.extend(self._diff_one(piece, clip))
            result = next_result
        return result

    def offset(self, polygon, delta_mm):
        self.calls.append(("offset", (polygon, delta_mm)))
        if isinstance(polygon, Rect):
            return self._offset_rect(polygon, delta_mm)
        if isinstance(polygon, NotchedRect):
            return self._offset_notched_rect(polygon, delta_mm)
        if isinstance(polygon, NotchedPolygon):
            return self._offset_notched_polygon(polygon, delta_mm)
        if isinstance(polygon, PinchedNotchedPolygon):
            return self._offset_pinched_notched_polygon(polygon, delta_mm)
        if isinstance(polygon, MultiNotchedRect):
            return self._offset_multi_notched_rect(polygon, delta_mm)
        if isinstance(polygon, TwoSidedNotchedRect):
            return self._offset_two_sided_notched_rect(polygon, delta_mm)
        if isinstance(polygon, ScarredHoledRect):
            return self._offset_scarred_holed_rect(polygon, delta_mm)
        # a generic DUCK-TYPED fallback for any object exposing
        # ``outer``/``notch`` attributes that are both ``Rect`` — e.g. a
        # test-local single-notch dataclass whose notch touches the -Y edge
        # rather than the +Y edge ``NotchedRect`` assumes. Not an isinstance
        # dispatch (a test module cannot make its own locally-defined class
        # match one of this module's classes), and deliberately narrow: it only
        # recognizes the exact ``outer``/``notch`` shape, mirroring
        # ``NotchedRect``'s own erosion logic via a Y-reflection when the notch
        # touches the bottom edge instead of the top (see
        # ``_offset_generic_single_notch_rect``).
        if hasattr(polygon, "outer") and hasattr(polygon, "notch"):
            outer = getattr(polygon, "outer")
            notch = getattr(polygon, "notch")
            if isinstance(outer, Rect) and isinstance(notch, Rect):
                return self._offset_generic_single_notch_rect(outer, notch, delta_mm)
        raise NotImplementedError(
            "StubOps.offset does not support {!r} — see module docstring "
            "for this stub's documented limits.".format(type(polygon))
        )

    def area(self, polygon):
        self.calls.append(("area", (polygon,)))
        return polygon.area()

    def is_valid(self, polygon):
        self.calls.append(("is_valid", (polygon,)))
        return polygon.area() > EPSILON_MM

    def bounds(self, polygon):
        self.calls.append(("bounds", (polygon,)))
        return _as_rect_bounds(polygon)

    def contour_count(self, polygon):
        self.calls.append(("contour_count", (polygon,)))
        if isinstance(polygon, HoledRect):
            return 2
        # ScarredHoledRect is deliberately NOT listed here: it is the "already
        # correctly scarred" transformed layer, and its own topological claim
        # (b) is that a correctly bridged hole is no longer an interior hole at
        # all — falling through to the default ``1`` here is not an omission,
        # it IS the assertion this stub encodes.
        return 1

    def polygon_from_points(self, points_mm):
        self.calls.append(("polygon_from_points", (tuple(points_mm),)))
        return PolygonShape(tuple(points_mm))

    def contour_points_mm(self, polygon):
        """See ``GeometryOps.contour_points_mm``. Only ever called on an ORIGINAL
        (pre-cut) per-layer section — a ``Rect`` (the coupon), a
        ``PolygonShape`` (an airfoil section), or a ``HoledRect`` (a section
        carrying a declared interior hole) — so those are the shapes this stub
        needs to support; anything else raises loudly, matching ``offset()``'s
        documented pattern for unsupported shapes. ``HoledRect`` returns its
        OUTER ring only, mirroring ``OrcaGeometryOps.contour_points_mm``'s own
        real behaviour (it reads ``polygon.contour.points``, never
        ``polygon.holes`` —b: frame construction succeeds, silently ignoring
        the hole, which is exactly the point of ``LayerFrame.from_polygon``
        being buildable from a holed original at all).
        """
        self.calls.append(("contour_points_mm", (polygon,)))
        if isinstance(polygon, Rect):
            return [
                (polygon.min_x, polygon.min_y),
                (polygon.max_x, polygon.min_y),
                (polygon.max_x, polygon.max_y),
                (polygon.min_x, polygon.max_y),
            ]
        if isinstance(polygon, PolygonShape):
            return list(polygon.points)
        if isinstance(polygon, HoledRect):
            return self.contour_points_mm(polygon.outer)
        raise NotImplementedError(
            "StubOps.contour_points_mm does not support {!r} — see module "
            "docstring for this stub's documented limits.".format(type(polygon))
        )

    def hole_bounds(self, polygon):
        """See ``GeometryOps.hole_bounds``. Only ``HoledRect`` (this stub's one
        holed shape) carries an interior hole; every other recognized shape has
        none, so this returns ``[]`` for them (hole-free behaviour is unchanged).
        """
        self.calls.append(("hole_bounds", (polygon,)))
        if isinstance(polygon, HoledRect):
            return [polygon.hole.bounds()]
        if isinstance(polygon, MultiHoledRect):
            return [h.bounds() for h in polygon.holes]
        return []

    def hole_points_mm(self, polygon):
        """See ``GeometryOps.hole_points_mm``. Mirrors ``hole_bounds()``
        above: only ``HoledRect`` has a hole to report.
        """
        self.calls.append(("hole_points_mm", (polygon,)))
        if isinstance(polygon, HoledRect):
            h = polygon.hole
            return [
                [
                    (h.min_x, h.min_y),
                    (h.max_x, h.min_y),
                    (h.max_x, h.max_y),
                    (h.min_x, h.max_y),
                ]
            ]
        return []

    def intersection(self, subject, clip):
        """``subject`` intersected with ``clip`` — see module docstring.

        HARD CONSTRAINT: ``clip`` must be a convex, axis-aligned rectangle
        (``NotImplementedError`` otherwise) — every real caller
        (``frame.local_span``) only ever clips with a measurement strip
        built by ``polygon_from_points``, which is exactly that shape.
        ``subject`` may be any recognized shape, convex or not
        (Sutherland-Hodgman clipping against a convex window is exact for
        an arbitrary — even concave — subject, PROVIDED the true
        intersection is a single connected region).

        That proviso matters: when ``subject`` intersected with ``clip`` is
        genuinely DISCONNECTED (a re-entrant probe station), naive single-ring
        Sutherland-Hodgman does not fail loudly — it silently stitches the
        pieces into one self-intersecting "bowtie" ring whose bounds/area are
        simply WRONG (verified by construction: an I-beam section probed
        between its flanges, outside the web, gives a shoelace area of 1.5
        against a true 2.0, and a Y bounds of the *entire* flange-to-flange
        span instead of the two separate flange intervals — exactly the kind of
        silent mis-measurement exists to eliminate). So before trusting
        Sutherland-Hodgman's single-ring output, this method first does an
        independent, cheap check — a vertical-line scan through the clip's own
        X midpoint (``_vertical_scanline_intervals``) — for how many disjoint Y
        intervals of material actually exist there:

        - exactly one interval: the fast/common path (typical airfoil
          probes). Runs Sutherland-Hodgman for an exact clipped polygon and
          returns it as the sole result.
        - zero intervals: returns ``[]`` (no material at this station).
        - more than one interval: returns one axis-aligned rectangle per
          interval, spanning the clip's own X range. This is NOT a general
          polygon-clipping result (the true piece shape is not
          reconstructed) — it is exact for area/bounds() purposes at this
          probe, which is all ``frame.local_span`` ever reads from a
          returned piece. Sufficient for a test stub whose only consumer of
          a multi-piece result needs to know "more than one" and each
          piece's Y extent, not its outline.
        """
        self.calls.append(("intersection", (subject, clip)))
        if isinstance(subject, HoledRect):
            return self._intersection_holed_rect(subject, clip)
        if isinstance(subject, MultiHoledRect):
            return self._intersection_multi_holed_rect(subject, clip)
        subject_points = _as_point_list(subject)
        clip_points = _as_point_list(clip)
        if not _is_convex(clip_points):
            raise NotImplementedError(
                "StubOps.intersection only supports a convex clip region "
                "(see module docstring); got a non-convex clip with points "
                "{!r}.".format(clip_points)
            )
        if not _is_axis_aligned_rect(clip_points):
            raise NotImplementedError(
                "StubOps.intersection only supports an axis-aligned "
                "rectangular clip (the measurement-strip use case); got "
                "{!r}.".format(clip_points)
            )

        clip_x0, clip_y0, clip_x1, clip_y1 = _as_rect_bounds(PolygonShape(tuple(clip_points)))
        x_probe = (clip_x0 + clip_x1) / 2.0
        intervals = _vertical_scanline_intervals(subject_points, x_probe)

        clipped_intervals = []
        for y_lo, y_hi in intervals:
            lo = max(y_lo, clip_y0)
            hi = min(y_hi, clip_y1)
            if hi - lo > EPSILON_MM:
                clipped_intervals.append((lo, hi))

        if not clipped_intervals:
            return []

        if len(clipped_intervals) == 1:
            exact = _sutherland_hodgman(subject_points, clip_points)
            if len(exact) < 3:
                return []
            return [PolygonShape(tuple(exact))]

        return [
            PolygonShape(
                (
                    (clip_x0, y_lo),
                    (clip_x1, y_lo),
                    (clip_x1, y_hi),
                    (clip_x0, y_hi),
                )
            )
            for y_lo, y_hi in clipped_intervals
        ]

    def _intersection_holed_rect(self, subject: "HoledRect", clip):
        """``HoledRect`` (``outer`` minus one fully-interior ``hole``) clipped to
        an axis-aligned rectangular ``clip``. ``hole_bounds()``/``hole_points_mm()``
        give a holed shape visibility, and this method lets ``intersection()``
        (what ``frame.local_span`` probes with) accept a holed polygon as a
        SUBJECT (``_as_point_list`` recognizes only ``Rect``/``PolygonShape`` —
        see that function's own docstring).

        Same convex-axis-aligned-rectangle ``clip`` constraint as the
        general ``intersection()`` above (this is exactly the shape
        ``local_span``'s own measurement strip always is). Exact for
        bounds()/area() at the probe, same guarantee as the general
        method's own multi-piece branch:

        - ``clip``'s own X midpoint (the probe station) falls STRICTLY
          inside ``hole``'s own X range: the vertical column at this X has
          material from ``outer.min_y`` up to ``hole.min_y``, a void, then
          material again from ``hole.max_y`` up to ``outer.max_y`` — TWO
          pieces, clipped to ``clip``'s own Y range exactly like the
          general method's multi-interval branch.
        - otherwise: the hole's X range does not reach this station at all,
          so the column is ordinary, unbroken ``outer`` material — ONE
          piece, an exact rectangle intersection of ``outer`` and ``clip``.
        """
        clip_points = _as_point_list(clip)
        if not _is_convex(clip_points):
            raise NotImplementedError(
                "StubOps.intersection only supports a convex clip region "
                "(see module docstring); got a non-convex clip with points "
                "{!r}.".format(clip_points)
            )
        if not _is_axis_aligned_rect(clip_points):
            raise NotImplementedError(
                "StubOps.intersection only supports an axis-aligned "
                "rectangular clip (the measurement-strip use case); got "
                "{!r}.".format(clip_points)
            )

        clip_x0, clip_y0, clip_x1, clip_y1 = _as_rect_bounds(
            PolygonShape(tuple(clip_points))
        )
        x_probe = (clip_x0 + clip_x1) / 2.0
        outer = subject.outer
        hole = subject.hole

        if hole.min_x + EPSILON_MM < x_probe < hole.max_x - EPSILON_MM:
            intervals = [(outer.min_y, hole.min_y), (hole.max_y, outer.max_y)]
        else:
            intervals = [(outer.min_y, outer.max_y)]

        clipped_intervals = []
        for y_lo, y_hi in intervals:
            lo = max(y_lo, clip_y0)
            hi = min(y_hi, clip_y1)
            if hi - lo > EPSILON_MM:
                clipped_intervals.append((lo, hi))

        if not clipped_intervals:
            return []

        lo_x = max(outer.min_x, clip_x0)
        hi_x = min(outer.max_x, clip_x1)
        return [
            PolygonShape(
                ((lo_x, y_lo), (hi_x, y_lo), (hi_x, y_hi), (lo_x, y_hi))
            )
            for y_lo, y_hi in clipped_intervals
        ]

    def _intersection_multi_holed_rect(self, subject: "MultiHoledRect", clip):
        """``MultiHoledRect`` clipped to an axis-aligned rectangular ``clip`` —
        the third-step multi-hole analogue of ``_intersection_holed_rect``.
        Finds every hole whose X range STRICTLY contains the probe's own X
        midpoint (same convention as the single-hole method), sorts them by Y,
        and threads the material intervals between them — so a probe crossing
        zero, one, or (for a deliberately stacked fixture) more than one active
        hole all fall out of the SAME general logic, rather than special-casing
        the count. Exact for bounds()/area() at the probe, same guarantee as
        the single-hole method.
        """
        clip_points = _as_point_list(clip)
        if not _is_convex(clip_points):
            raise NotImplementedError(
                "StubOps.intersection only supports a convex clip region "
                "(see module docstring); got a non-convex clip with points "
                "{!r}.".format(clip_points)
            )
        if not _is_axis_aligned_rect(clip_points):
            raise NotImplementedError(
                "StubOps.intersection only supports an axis-aligned "
                "rectangular clip (the measurement-strip use case); got "
                "{!r}.".format(clip_points)
            )

        clip_x0, clip_y0, clip_x1, clip_y1 = _as_rect_bounds(
            PolygonShape(tuple(clip_points))
        )
        x_probe = (clip_x0 + clip_x1) / 2.0
        outer = subject.outer

        active_holes = sorted(
            (
                h
                for h in subject.holes
                if h.min_x + EPSILON_MM < x_probe < h.max_x - EPSILON_MM
            ),
            key=lambda h: h.min_y,
        )

        intervals = []
        cursor = outer.min_y
        for h in active_holes:
            if h.min_y > cursor:
                intervals.append((cursor, h.min_y))
            cursor = max(cursor, h.max_y)
        if cursor < outer.max_y:
            intervals.append((cursor, outer.max_y))

        clipped_intervals = []
        for y_lo, y_hi in intervals:
            lo = max(y_lo, clip_y0)
            hi = min(y_hi, clip_y1)
            if hi - lo > EPSILON_MM:
                clipped_intervals.append((lo, hi))

        if not clipped_intervals:
            return []

        lo_x = max(outer.min_x, clip_x0)
        hi_x = min(outer.max_x, clip_x1)
        return [
            PolygonShape(
                ((lo_x, y_lo), (hi_x, y_lo), (hi_x, y_hi), (lo_x, y_hi))
            )
            for y_lo, y_hi in clipped_intervals
        ]

    # -- internals ----------------------------------------------------------

    def _diff_one(self, subject, clip):
        """``subject`` minus ``clip`` (any recognized shape).

        A ``NotchedRect``/``NotchedPolygon``/``HoledRect`` clip represents "the
        transformed layer" in validator tests: subject minus it is exactly the
        notch/hole region that was carved out, since by construction ``subject
        == clip.outer``. This is the ONLY case that accepts a non-``Rect``
        ``subject`` (e.g. an airfoil ``PolygonShape``, non-rectangular-original
        tests) — everything below this still requires a plain ``Rect`` subject,
        per the module docstring.
        """
        if isinstance(clip, (NotchedRect, NotchedPolygon)):
            if clip.outer != subject:
                raise NotImplementedError(
                    "StubOps.diff(subject, [{}]) requires clip.outer == "
                    "subject; got outer={!r} subject={!r}.".format(
                        type(clip).__name__, clip.outer, subject
                    )
                )
            return [clip.notch]
        if isinstance(clip, HoledRect):
            if clip.outer != subject:
                raise NotImplementedError(
                    "StubOps.diff(subject, [HoledRect]) requires "
                    "clip.outer == subject; got outer={!r} subject={!r}.".format(
                        clip.outer, subject
                    )
                )
            return [clip.hole]

        if not isinstance(subject, Rect):
            raise NotImplementedError(
                "StubOps.diff only subtracts from a plain Rect subject (for "
                "clips other than NotchedRect/NotchedPolygon/HoledRect); got "
                "{!r}. See module docstring.".format(type(subject))
            )

        clip_min_x, clip_min_y, clip_max_x, clip_max_y = _as_rect_bounds(clip)
        s = subject

        # No overlap at all -> unchanged.
        if (
            clip_max_x <= s.min_x + EPSILON_MM
            or clip_min_x >= s.max_x - EPSILON_MM
            or clip_max_y <= s.min_y + EPSILON_MM
            or clip_min_y >= s.max_y - EPSILON_MM
        ):
            return [s]

        touches_left = clip_min_x <= s.min_x + EPSILON_MM
        touches_right = clip_max_x >= s.max_x - EPSILON_MM
        touches_top = clip_max_y >= s.max_y - EPSILON_MM
        touches_bottom = clip_min_y <= s.min_y + EPSILON_MM

        # Full vertical split: clip spans the whole depth, strictly inside in x.
        if touches_top and touches_bottom and not touches_left and not touches_right:
            left = Rect(s.min_x, s.min_y, clip_min_x, s.max_y)
            right = Rect(clip_max_x, s.min_y, s.max_x, s.max_y)
            return [left, right]

        # Top-edge notch: touches the top edge only, strictly inside on the
        # other three sides.
        if (
            touches_top
            and not touches_bottom
            and not touches_left
            and not touches_right
        ):
            notch = Rect(clip_min_x, clip_min_y, clip_max_x, s.max_y)
            return [NotchedRect(outer=s, notch=notch)]

        # Fully interior clip: touches no edge -> enclosed hole.
        if not (touches_top or touches_bottom or touches_left or touches_right):
            hole = Rect(clip_min_x, clip_min_y, clip_max_x, clip_max_y)
            return [HoledRect(outer=s, hole=hole)]

        raise NotImplementedError(
            "StubOps.diff does not recognize this subject/clip combination "
            "(subject={!r}, clip bounds=({!r},{!r},{!r},{!r})) — extend the "
            "stub deliberately if a new test needs it.".format(
                s, clip_min_x, clip_min_y, clip_max_x, clip_max_y
            )
        )

    def _offset_rect(self, rect, delta_mm):
        new = Rect(
            rect.min_x - delta_mm,
            rect.min_y - delta_mm,
            rect.max_x + delta_mm,
            rect.max_y + delta_mm,
        )
        if new.max_x - new.min_x <= EPSILON_MM or new.max_y - new.min_y <= EPSILON_MM:
            return []
        return [new]

    def _offset_notched_rect(self, shape, delta_mm):
        # Erosion (delta_mm < 0) narrows every material-to-air boundary pair
        # by 2 * |delta_mm|: the two arms beside the notch, the bridge below
        # it, and the outer envelope itself. If any survives, return one
        # approximately-shrunk NotchedRect; the exact resulting shape is not
        # load-bearing (see module docstring), only the count is. Growth
        # (delta_mm > 0) is not needed by any test and is not supported.
        if delta_mm > 0:
            raise NotImplementedError(
                "StubOps.offset(NotchedRect, positive delta) is not needed by "
                "these tests and is not implemented."
            )
        shrink = -2.0 * delta_mm  # total width lost per constrained dimension

        outer_w = shape.outer.max_x - shape.outer.min_x
        outer_h = shape.outer.max_y - shape.outer.min_y

        # The envelope itself is gone: nothing survives, whatever the arms do.
        if outer_w - shrink <= EPSILON_MM or outer_h - shrink <= EPSILON_MM:
            return []

        arms_survive = (
            shape.left_arm() - shrink > EPSILON_MM
            and shape.right_arm() - shrink > EPSILON_MM
        )
        bridge_survives = shape.bottom_bridge() - shrink > EPSILON_MM

        if not bridge_survives:
            # The bridge under the slot is consumed. If the arms still have
            # material they become two disjoint pieces — the severed-island
            # case the validator's erosion test exists to catch. Modelling this
            # as "nothing survives" (the previous behaviour) made the split
            # branch unreachable and would have hidden a real regression.
            if not arms_survive:
                return []
            d_arm = -delta_mm
            left = Rect(
                shape.outer.min_x + d_arm,
                shape.notch.min_y,
                shape.notch.min_x - d_arm,
                shape.outer.max_y,
            )
            right = Rect(
                shape.notch.max_x + d_arm,
                shape.notch.min_y,
                shape.outer.max_x - d_arm,
                shape.outer.max_y,
            )
            return [left, right]

        if not arms_survive:
            # Arms consumed but the bridge holds: one piece remains. NOTE this
            # is a KNOWN VALIDATOR GAP, not a stub artefact — the erosion test
            # sees one island and passes, even though the slot walls are now
            # thinner than a bead and Orca would render them poorly or drop
            # them. Fine for the coupon (arms ~9.8 mm wide); revisit when a
            # section is narrow enough for it to matter.
            return [
                Rect(
                    shape.outer.min_x + -delta_mm,
                    shape.outer.min_y + -delta_mm,
                    shape.outer.max_x - -delta_mm,
                    shape.notch.min_y,
                )
            ]

        d = -delta_mm  # positive erosion distance
        new_outer = Rect(
            shape.outer.min_x + d,
            shape.outer.min_y + d,
            shape.outer.max_x - d,
            shape.outer.max_y,  # top edge is open at the notch; unaffected here
        )
        new_notch = Rect(
            shape.notch.min_x - d,
            shape.notch.min_y - d,
            shape.notch.max_x + d,
            shape.outer.max_y,
        )
        return [NotchedRect(outer=new_outer, notch=new_notch)]

    def _offset_generic_single_notch_rect(self, outer: Rect, notch: Rect, delta_mm):
        """erosion piece-count for an ``outer``/``notch`` pair whose notch touches
        either the +Y OR the -Y edge, auto-detected from the notch's own
        position relative to ``outer`` (never a parameter — the same "derive
        the side from the geometry" discipline ``validate.py``'s invariant 3/4
        side detection uses).

        A +Y-touching notch is handed straight to ``_offset_notched_rect``
        unchanged. A -Y-touching notch is handled by REFLECTING both
        rectangles vertically about ``outer``'s own Y midpoint (turning the
        bottom notch into a top notch), running the existing, already
        -exercised ``_offset_notched_rect`` logic, then reflecting the
        result back — reusing the tested erosion arithmetic exactly rather
        than re-deriving a mirrored version of it. This is exact for piece
        COUNT (all this stub's callers ever consume — see the module
        docstring), by the same reflection-symmetry argument that makes it
        exact for the top-notch case.
        """
        touches_top = notch.max_y >= outer.max_y - EPSILON_MM
        touches_bottom = notch.min_y <= outer.min_y + EPSILON_MM
        if touches_top and not touches_bottom:
            return self._offset_notched_rect(NotchedRect(outer=outer, notch=notch), delta_mm)
        if touches_bottom and not touches_top:
            mid_y = outer.min_y + outer.max_y  # reflection: y' = mid_y - y
            reflected_outer = Rect(
                outer.min_x, mid_y - outer.max_y, outer.max_x, mid_y - outer.min_y
            )
            reflected_notch = Rect(
                notch.min_x, mid_y - notch.max_y, notch.max_x, mid_y - notch.min_y
            )
            pieces = self._offset_notched_rect(
                NotchedRect(outer=reflected_outer, notch=reflected_notch), delta_mm
            )
            result = []
            for piece in pieces:
                if isinstance(piece, NotchedRect):
                    po, pn = piece.outer, piece.notch
                    result.append(
                        NotchedRect(
                            outer=Rect(
                                po.min_x, mid_y - po.max_y, po.max_x, mid_y - po.min_y
                            ),
                            notch=Rect(
                                pn.min_x, mid_y - pn.max_y, pn.max_x, mid_y - pn.min_y
                            ),
                        )
                    )
                else:
                    result.append(
                        Rect(
                            piece.min_x,
                            mid_y - piece.max_y,
                            piece.max_x,
                            mid_y - piece.min_y,
                        )
                    )
            return result
        raise NotImplementedError(
            "StubOps.offset(<outer/notch duck-typed shape>, ...): notch "
            "bounds ({!r}) touch neither cleanly the top nor the bottom "
            "edge of outer ({!r}) — this fallback only supports a notch "
            "touching exactly one of the two.".format(notch, outer)
        )

    def _offset_multi_notched_rect(self, shape: "MultiNotchedRect", delta_mm):
        """Connectivity-only erosion signal for several parallel notches.

        DELIBERATELY NARROW — see the module docstring's ``MultiNotchedRect``
        entry: only answers "the whole envelope survives AND the shared base
        bridge beneath every notch (assumed the same depth for every notch)
        clearly survives erosion", in which case the material stays one
        connected island via that base strip regardless of how thin the
        walls between individual notches are — this is exactly why a
        separate collision check exists (aircraft_core/validate.py), not a
        gap in this stub's erosion model. The bridge-fails/severed-island
        case for more than one notch is out of scope for this stub; extend
        deliberately (mirroring ``_offset_notched_rect``'s severed-island
        branch) if a future test needs it.
        """
        if delta_mm > 0:
            raise NotImplementedError(
                "StubOps.offset(MultiNotchedRect, positive delta) is not "
                "needed by any test and is not implemented."
            )
        shrink = -2.0 * delta_mm
        outer_w = shape.outer.max_x - shape.outer.min_x
        outer_h = shape.outer.max_y - shape.outer.min_y
        if outer_w - shrink <= EPSILON_MM or outer_h - shrink <= EPSILON_MM:
            return []

        if not shape.notches:
            return [shape]

        bridge_depths = {round(n.min_y, 9) for n in shape.notches}
        if len(bridge_depths) != 1:
            raise NotImplementedError(
                "StubOps.offset(MultiNotchedRect, ...): notches at different "
                "bridge depths are not supported by this stub's uniform-depth "
                "assumption. See module docstring."
            )
        bridge = next(iter(shape.notches)).min_y - shape.outer.min_y
        if bridge - shrink > EPSILON_MM:
            return [shape]

        raise NotImplementedError(
            "StubOps.offset(MultiNotchedRect, delta={!r}) mm: the shared "
            "base bridge ({!r} mm) does not clearly survive erosion. This "
            "stub only answers the 'bridge clearly survives, still one "
            "connected island' case for more than one notch; the "
            "severed/consumed case is exercised via NotchedRect fixtures "
            "instead (single notch). See module docstring.".format(
                delta_mm, bridge
            )
        )

    def _offset_two_sided_notched_rect(self, shape: "TwoSidedNotchedRect", delta_mm):
        """Erosion piece-count for a pair of OPPOSING (+Y/-Y) notches sharing an X
        range — the no-sever proof's "meeting condition", stated as geometry
        rather than assumed.

        Mirrors ``_offset_notched_rect`` exactly, with the "bridge" replaced by
        ``shape.middle_wall()`` — the material between the two opposing
        notches, rather than the material below a single top notch. If that
        wall does not survive erosion, the shape severs into a LEFT and a RIGHT
        column, each spanning the outer rectangle's FULL Y height (unlike the
        single-notch case, where a severed arm still has the notch's own Y
        range cut into its OTHER side — here neither notch reaches into the
        arm's X range at all, since both notches share the same X band by
        construction), which is exactly the sever the proof's meeting condition
        warns about.
        """
        if delta_mm > 0:
            raise NotImplementedError(
                "StubOps.offset(TwoSidedNotchedRect, positive delta) is not "
                "needed by any test and is not implemented."
            )
        shrink = -2.0 * delta_mm

        outer_w = shape.outer.max_x - shape.outer.min_x
        outer_h = shape.outer.max_y - shape.outer.min_y
        if outer_w - shrink <= EPSILON_MM or outer_h - shrink <= EPSILON_MM:
            return []

        arms_survive = (
            shape.left_arm() - shrink > EPSILON_MM
            and shape.right_arm() - shrink > EPSILON_MM
        )
        wall_survives = shape.middle_wall() - shrink > EPSILON_MM

        d = -delta_mm  # positive erosion distance

        if not wall_survives:
            # The wall between the two opposing notches is consumed. If the
            # arms still have material they become two disjoint columns —
            # the sever this proof exists to characterize.
            if not arms_survive:
                return []
            left = Rect(
                shape.outer.min_x + d,
                shape.outer.min_y + d,
                shape.notch_top.min_x - d,
                shape.outer.max_y - d,
            )
            right = Rect(
                shape.notch_top.max_x + d,
                shape.outer.min_y + d,
                shape.outer.max_x - d,
                shape.outer.max_y - d,
            )
            return [left, right]

        if not arms_survive:
            # Arms consumed but the middle wall holds: one piece remains —
            # same KNOWN VALIDATOR GAP note as ``_offset_notched_rect``
            # (sub-bead-width arms still read as one island).
            return [
                Rect(
                    shape.outer.min_x + d,
                    shape.notch_bottom.max_y,
                    shape.outer.max_x - d,
                    shape.notch_top.min_y,
                )
            ]

        new_outer = Rect(
            shape.outer.min_x + d,
            shape.outer.min_y,  # bottom edge is open at notch_bottom; unaffected here
            shape.outer.max_x - d,
            shape.outer.max_y,  # top edge is open at notch_top; unaffected here
        )
        new_notch_top = Rect(
            shape.notch_top.min_x - d,
            shape.notch_top.min_y - d,
            shape.notch_top.max_x + d,
            shape.outer.max_y,
        )
        new_notch_bottom = Rect(
            shape.notch_bottom.min_x - d,
            shape.outer.min_y,
            shape.notch_bottom.max_x + d,
            shape.notch_bottom.max_y + d,
        )
        return [
            TwoSidedNotchedRect(
                outer=new_outer, notch_top=new_notch_top, notch_bottom=new_notch_bottom
            )
        ]

    def _offset_scarred_holed_rect(self, shape: "ScarredHoledRect", delta_mm):
        """See ``ScarredHoledRect``'s own docstring for the connectivity
        model this implements. Only the resulting PIECE COUNT is exact
        (the same convention every severed-island case in this stub already
        uses) — the returned pieces themselves are placeholders, not a
        reconstructed outline.
        """
        fragment_width_mm = shape.fragment_width_mm()
        survives = fragment_width_mm - 2.0 * abs(delta_mm) > EPSILON_MM
        if survives:
            return [shape.outer]
        # Severed: each of the two cap fragments beside the scar detaches
        # from the rest of the material — two placeholder pieces, count
        # only, exactly ``NotchedRect``'s own severed-island convention.
        return [shape.outer, shape.outer]

    def _offset_notched_polygon(self, shape: "NotchedPolygon", delta_mm):
        """Connectivity-only erosion signal for a ``NotchedPolygon``.

        Deliberately NOT a general polygon-erosion implementation (the
        module docstring's boundary on this stub). It answers exactly the
        one question ``TopologyValidator``'s connectivity check consumes —
        does the material below the notch clearly survive a uniform erosion
        of ``|delta_mm|`` — by re-measuring the LOCAL bridge width beneath
        the notch (via ``aircraft_core.frame.local_span``, the same function
        the validator itself uses for invariants 3/4) at the notch's own
        chord station, and comparing it against the erosion's total width
        loss (``2 * |delta_mm|``, same convention as ``_offset_notched_rect``).

        - bridge clearly survives -> returns ``[shape]`` unchanged: this is
          exact for the COUNT (one surviving, connected island), not for the
          resulting polygon's true eroded outline.
        - otherwise -> ``NotImplementedError``. Reproducing the severed
          -island / fully-consumed cases for an arbitrary polygon is a real
          erosion problem this stub does not attempt; those cases are
          already exercised via ``NotchedRect`` fixtures (coupon).
        """
        if delta_mm > 0:
            raise NotImplementedError(
                "StubOps.offset(NotchedPolygon, positive delta) is not "
                "needed by any test and is not implemented."
            )
        shrink = -2.0 * delta_mm
        notch = shape.notch
        x_station = (notch.min_x + notch.max_x) / 2.0
        local_y_min, _local_y_max = local_span(
            self, shape.outer, x_station, DEFAULT_PROBE_WIDTH_MM
        )
        bridge = notch.min_y - local_y_min
        if bridge - shrink <= EPSILON_MM:
            raise NotImplementedError(
                "StubOps.offset(NotchedPolygon, delta={!r}) mm: the bridge "
                "beneath the notch ({!r} mm) does not clearly survive "
                "erosion. This stub only answers the 'clearly survives, "
                "still one connected island' case for a non-rectangular "
                "outer polygon; severed/consumed cases are exercised via "
                "NotchedRect fixtures instead. See module "
                "docstring.".format(delta_mm, bridge)
            )
        return [shape]

    def _offset_pinched_notched_polygon(self, shape: "PinchedNotchedPolygon", delta_mm):
        """Connectivity-only erosion signal for a ``PinchedNotchedPolygon``.

        Two INDEPENDENT checks, in order, both against the same erosion
        width (``2 * |delta_mm|``):

        1. the notch's own bridge, exactly as ``_offset_notched_polygon``
           checks it (re-measured via ``local_span`` at the notch's own
           chord station) — if it does not clearly survive, this raises
           ``NotImplementedError`` exactly as that method does; this stub
           still does not attempt a severed-island reconstruction for a
           notch-driven cause on a non-rectangular outer.
        2. the DECLARED ``waist_thickness_mm`` at ``waist_x_mm`` — unrelated
           to the notch. If it does not clearly survive, the material
           genuinely splits regardless of the notch, so this returns TWO
           placeholder pieces (the two lobes either side of the waist) —
           unlike the notch-driven case, this IS the severed-island
           reconstruction the erosion catch-all test needs, and it is exact
           here only because a straight vertical split at a single
           declared X station needs no general polygon-clipping
           implementation. Only the piece COUNT is load-bearing for
           ``TopologyValidator``'s check (``len(eroded) > 1``); the
           placeholder pieces' own bounds are a cheap, honest split of
           ``outer``'s bounding box at ``waist_x_mm``, not a reconstruction
           of ``outer``'s true eroded outline.
        """
        if delta_mm > 0:
            raise NotImplementedError(
                "StubOps.offset(PinchedNotchedPolygon, positive delta) is "
                "not needed by any test and is not implemented."
            )
        shrink = -2.0 * delta_mm

        notch = shape.notch
        x_station = (notch.min_x + notch.max_x) / 2.0
        local_y_min, _local_y_max = local_span(
            self, shape.outer, x_station, DEFAULT_PROBE_WIDTH_MM
        )
        bridge = notch.min_y - local_y_min
        if bridge - shrink <= EPSILON_MM:
            raise NotImplementedError(
                "StubOps.offset(PinchedNotchedPolygon, delta={!r}) mm: the "
                "bridge beneath the notch ({!r} mm) does not clearly "
                "survive erosion. See _offset_notched_polygon's identical "
                "limit.".format(delta_mm, bridge)
            )

        if shape.waist_thickness_mm - shrink <= EPSILON_MM:
            min_x, min_y, max_x, max_y = shape.outer.bounds()
            left = Rect(min_x, min_y, shape.waist_x_mm, max_y)
            right = Rect(shape.waist_x_mm, min_y, max_x, max_y)
            return [left, right]

        return [shape]
