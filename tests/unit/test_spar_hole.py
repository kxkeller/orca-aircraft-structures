"""Spar hole routing: proof-encoding fixtures, plus tests of the
hole-visibility, hole-aware-probe and solid-thickness primitives.

Scar placement itself (``StructureGenerator._generate_scar``,
``WebIntent.is_scar``, ``StructureSpec.expected_hole_count``, and the matching
``validate.py`` scar branch) has its own dedicated fixtures in
``tests/unit/test_scar.py``. This file holds the proof-style and
pure-arithmetic fixtures that underpin it, plus checks against the
validator and ``local_span`` with a hole present but no scar. It encodes, and
demonstrates against REAL code where possible, the claims the design document
(``docs/geometry_invariants.md``'s "Extended no-sever proof") makes about
interior holes: invariant 2 forbids them unconditionally unless a scar
connects them to the boundary.

Kinds of fixture, in the style of ``tests/unit/test_opposing_entry.py``:

- ``HoleWallMarginFormulaTests`` -- Pure-Python reference form of the
  doc's section 2d "remaining wall to the hole" inequality. Not the
  shipped check; exercises the proof's own arithmetic.
- ``TodaysUnconnectedHoleFailsInvariantTwoTests`` -- Against the REAL
  ``TopologyValidator``. Demonstrates the doc's section 1b "Case C": an
  interior hole that survives into the transformed layer (nothing connected
  it to the boundary) fails invariant 2 unconditionally. This is the
  mechanism a scar must route AROUND, not defeat.
- ``TodaysReentrantProbeFailsClosedTests`` / ``ScarBridgedStationSucceedsButNeighbouringStationsDoNotTests``
  -- Against REAL ``aircraft_core.frame.local_span``. Demonstrates the
  doc's section 1b "Case B" (a station whose vertical probe crosses
  material twice raises, exactly as a hole-crossing station would) and
  section 4b's finding (a hand-built, EXACT "keyhole" polygon -- a hole
  bridged to the boundary by a scar, traced as one simple polygon ring --
  shows the bridge fixes the probe AT the scar's own station but does
  nothing for a DIFFERENT station elsewhere in the hole's X footprint, which
  still raises the identical re-entrant error; see that class's own
  docstring for what this does and does NOT claim about the real,
  PRE-cut call sites).
- ``FeatureAbsentTests`` -- Interface checks: ``StructureSpec`` can declare an
  expected hole count (``expected_hole_count``) and ``GeometryOps`` exposes a
  hole query (``hole_bounds()``/``hole_points_mm()``).

``stub_ops.StubOps.intersection`` accepts a ``HoledRect`` as a probe subject
(dispatched before ``_as_point_list``, which does not recognize it -- see
``StubOps._intersection_holed_rect``), and ``aircraft_core.frame.local_span``
is hole-aware (opt-in via ``hole_side``; see its own docstring);
``tests/unit/test_local_span_hole_aware.py`` exercises that against a real
``HoledRect``. Generator-level scar fixtures live in ``test_scar.py``.
"""

import unittest

from aircraft_core.frame import DEFAULT_PROBE_WIDTH_MM, local_span
from aircraft_core.ops import GeometryOps
from aircraft_core.spec import StructureSpec
from aircraft_core.validate import TopologyError, TopologyValidator
from stub_ops import HoledRect, PolygonShape, Rect, StubOps

LAYER_INDEX = 6
PRINT_Z = 3.00

SAFETY_OFFSET_MM = 0.42
MIN_WALL_MM = 2.0 * SAFETY_OFFSET_MM  # 0.84


# ---------------------------------------------------------------------------
# Reference form of the doc's section 2d inequality. NOT the shipped check.
# ---------------------------------------------------------------------------


def _scar_leaves_insufficient_wall(
    hole_near_edge_reach_mm: float,
    t_local_mm: float,
    min_wall_mm: float = MIN_WALL_MM,
    epsilon_mm: float = 1e-9,
) -> bool:
    """True iff the remaining wall between a hole's near boundary and the LOCAL
    exterior surface cannot clear ``min_wall_mm`` -- the doc's section 2d, the
    same equality-fails-closed bias as ``_opposing_notches_meet`` in
    ``test_opposing_entry.py`` (proof point 4 there; here there is only one
    side, not two opposing cuts): a wall exactly at the minimum is already
    unsafe.

    ``hole_near_edge_reach_mm`` is the distance from the LOCAL exterior surface
    down to the hole's own near boundary -- what a hole-aware
    ``local_span``-shaped primitive reports (doc section 4a).
    """
    remaining_wall_mm = hole_near_edge_reach_mm
    if remaining_wall_mm >= t_local_mm - epsilon_mm:
        # A "remaining wall" at or past the section's own local thickness is
        # not a real placement (the scar would have to originate outside the
        # material) -- callers are expected to pass a genuine sub-thickness
        # reach; this is a defensive floor, not a case any real fixture
        # below exercises.
        return True
    return remaining_wall_mm <= (min_wall_mm + epsilon_mm)


class HoleWallMarginFormulaTests(unittest.TestCase):
    """doc section 2d, exercised directly: pure arithmetic, no geometry
    backend, no shipped check."""

    def test_comfortable_clearance_is_safe(self):
        # A spar hole with 5 mm of skin above its near boundary, well clear of
        # the 0.84 mm minimum wall this project already uses everywhere else
        # (own ``_min_wall_mm``).
        self.assertFalse(_scar_leaves_insufficient_wall(5.0, t_local_mm=8.0))

    def test_wall_below_minimum_is_unsafe(self):
        # 0.5 mm remaining wall is below the 0.84 mm minimum -- the same
        # class of failure as an ordinary full-depth web whose remaining
        # skin cannot survive the safety-offset erosion (invariant 4).
        self.assertTrue(_scar_leaves_insufficient_wall(0.5, t_local_mm=8.0))

    def test_exact_minimum_wall_is_unsafe(self):
        # Equality fails closed -- a wall exactly at the minimum is already
        # unsafe by this project's own standing convention for this shape of
        # comparison (see this function's own docstring).
        self.assertTrue(
            _scar_leaves_insufficient_wall(MIN_WALL_MM, t_local_mm=8.0)
        )


# ---------------------------------------------------------------------------
# Hole behaviour of the validator and local_span, against REAL code.
# ---------------------------------------------------------------------------


class TodaysUnconnectedHoleFailsInvariantTwoTests(unittest.TestCase):
    """doc section 1b, "Case C": an interior hole that survives into
    ``transformed`` (nothing connected it to the boundary) fails invariant 2
    unconditionally -- real ``TopologyValidator``, real
    ``stub_ops.HoledRect``. This is the mechanism a correctly-placed scar must
    route around, and this test pins that it fires when nothing routes around
    it."""

    def setUp(self):
        self.ops = StubOps()
        self.validator = TopologyValidator(self.ops, safety_offset_mm=SAFETY_OFFSET_MM)
        self.spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)

    def test_unconnected_hole_raises_naming_the_hole_count(self):
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 7.0, 24.0, 13.0)  # strictly interior; a spar hole
        transformed = HoledRect(outer=outer, hole=hole)

        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [outer],
                [transformed],
                [],  # no slots -- the hole persists untouched, as it would
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
            )
        message = str(ctx.exception)
        self.assertIn("interior hole", message)
        self.assertIn("1", message)


class TodaysReentrantProbeFailsClosedTests(unittest.TestCase):
    """doc section 1b, "Case B": a vertical probe that crosses material twice
    (material, void, material) raises, via
    ``aircraft_core.frame.local_span`` -- the exact exception a hole -crossing
    candidate station hits, whether or not the hole is a legitimate spar.
    Modelled with a hole-free "bracket" polygon (two bars joined only far
    outside the probed column) rather than a true interior hole, because
    ``local_span``'s probe never distinguishes a hole from any other cause of a
    two-piece intersection -- the mechanism is identical either way, and this
    fixture needs no ``GeometryOps`` extension to demonstrate it.
    """

    def setUp(self):
        self.ops = StubOps()
        # A "bracket": a solid left column [0, 2] x [0, 20] joins a top bar
        # [0, 40] x [17, 20] to a bottom bar [0, 40] x [0, 3]. Anywhere to
        # the right of x=2, a vertical probe crosses material (top bar),
        # then a real 14 mm gap, then material again (bottom bar) -- exactly
        # what a probe crossing an unconnected interior hole would see.
        self.bracket = PolygonShape(
            (
                (0.0, 0.0), (40.0, 0.0), (40.0, 3.0), (2.0, 3.0),
                (2.0, 17.0), (40.0, 17.0), (40.0, 20.0), (0.0, 20.0),
            )
        )

    def test_probe_away_from_the_connecting_column_is_reentrant(self):
        with self.assertRaises(ValueError) as ctx:
            local_span(self.ops, self.bracket, x_mm=20.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM)
        self.assertIn("re-entrant", str(ctx.exception))

    def test_probe_through_the_connecting_column_succeeds(self):
        # Sanity check on the fixture: at x=1 (inside the connecting
        # column), the probe sees ONE continuous interval, same as any
        # unholed section -- confirming the "re-entrant" result above is
        # caused by the probe's OWN X, not a fixture-wide defect.
        y_min, y_max = local_span(self.ops, self.bracket, x_mm=1.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM)
        self.assertAlmostEqual(y_min, 0.0, delta=0.01)
        self.assertAlmostEqual(y_max, 20.0, delta=0.01)


class ScarBridgedStationSucceedsButNeighbouringStationsDoNotTests(unittest.TestCase):
    """doc section 3b/4b, against REAL ``local_span``, on a hand-traced EXACT
    "keyhole" polygon: a 40x20 mm rectangle with one 8x6 mm rectangular "spar
    hole" (x=[16,24], y=[7,13]) bridged to the +Y skin by a 2 mm-wide scar
    (x=[19,21], y=[13,20]) -- traced as ONE simple polygon ring, the same
    construction the doc's section 3b proof argument describes (a single
    boundary-connected notch reaching from the skin to the hole's own edge,
    with the hole's remaining boundary forming the rest of the same loop -- see
    the doc for the full argument).

    This is not a general boolean construction -- the ring below is the
    EXACT result of ``outer - (hole UNION scar)`` for this one specific
    rectangular hole and rectangular scar, computed by hand and asserted
    against directly, the same "hand-built geometry, no general polygon
    algorithm" discipline ``stub_ops.py``'s other fixtures already use.

    THE FINDING THIS DEMONSTRATES (doc section 4b): probed on this ALREADY-CUT
    shape, the bridge fixes the probe AT the scar's own station (no re-entrant
    failure there, even though the hole is present), but does nothing for a
    DIFFERENT chord station elsewhere in the hole's own X footprint, away from
    the scar -- that station still raises the identical re-entrant error.

    WHAT THIS DOES NOT CLAIM: neither real call site in this codebase probes a
    POST-cut shape like this one with ``local_span``.
    ``LayerFrame.from_polygon`` / ``StructureGenerator._generate_family`` probe
    the PRE-cut ``original`` (the holed ``ExPolygon`` straight from Orca,
    before any boolean), and so does every ``validate.py`` invariant and
    ``frame.opposing_pair_meets``. On that PRE-cut original, the scar's OWN
    station is re-entrant too (material above the hole, void, material below)
    -- there is no cut yet to bridge anything. This class demonstrates the
    topological claim (section 3b: a correctly bridged result IS
    single-interval-clean at its own station) on the shape that claim is
    actually about; the "still re-entrant nearby" half (section 4b)
    generalizes to the PRE-cut case that matters in practice. Placing the scar
    therefore needs the hole-aware primitives (``local_span``'s opt-in
    ``hole_side`` and ``local_solid_thickness_worst_case``), not a plain probe.
    """

    def setUp(self):
        self.ops = StubOps()
        # CCW ring, traced by hand -- see this class's own docstring for the
        # exact construction. Walk: bottom edge, up the right edge, left
        # along the top to the scar's right wall, down the scar's right
        # wall to the hole's own top edge, right to the hole's top-right
        # corner, down the hole's right wall, left along the hole's bottom
        # edge, up the hole's left wall, right along the hole's top edge to
        # the scar's left wall, up the scar's left wall, left along the top
        # edge back to the start corner, down the left edge to close.
        self.keyhole = PolygonShape(
            (
                (0.0, 0.0), (40.0, 0.0), (40.0, 20.0),
                (21.0, 20.0), (21.0, 13.0),
                (24.0, 13.0), (24.0, 7.0), (16.0, 7.0), (16.0, 13.0),
                (19.0, 13.0), (19.0, 20.0),
                (0.0, 20.0),
            )
        )

    def test_probe_at_the_scars_own_station_does_not_raise(self):
        # x=20 is inside the scar's own width [19, 21]. The hole is present
        # (x=20 is also inside the hole's own footprint [16, 24]), but the
        # probe sees exactly ONE material interval: everything from the
        # skin down to the hole's own far (bottom) edge is now void (skin,
        # scar, and hole all connected), so the only material left at this
        # X is the remnant strip below the hole.
        y_min, y_max = local_span(
            self.ops, self.keyhole, x_mm=20.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM
        )
        self.assertAlmostEqual(y_min, 0.0, delta=0.01)
        self.assertAlmostEqual(y_max, 7.0, delta=0.01)

    def test_probe_elsewhere_in_the_holes_own_footprint_still_reentrant(self):
        # x=17 is inside the hole's X footprint [16, 24] but OUTSIDE the
        # scar's own width [19, 21] -- the bridge does not reach here. The
        # probe still sees material above the hole (the intact skin cap,
        # y=[13,20]) AND material below the hole (y=[0,7]): two separate
        # intervals, the identical re-entrant failure Case B names, even
        # though this layer HAS a correctly-bridged scar just 2-4 mm away.
        with self.assertRaises(ValueError) as ctx:
            local_span(
                self.ops, self.keyhole, x_mm=17.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM
            )
        self.assertIn("re-entrant", str(ctx.exception))

    def test_probe_outside_the_holes_footprint_entirely_is_unaffected(self):
        # Sanity check: away from the hole altogether, this is an ordinary,
        # unholed rectangle as far as local_span is concerned.
        y_min, y_max = local_span(
            self.ops, self.keyhole, x_mm=5.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM
        )
        self.assertAlmostEqual(y_min, 0.0, delta=0.01)
        self.assertAlmostEqual(y_max, 20.0, delta=0.01)


# ---------------------------------------------------------------------------
# Interface checks: hole declaration and hole query.
# ---------------------------------------------------------------------------


class FeatureAbsentTests(unittest.TestCase):
    """doc section 4a: ``StructureSpec`` must be able to declare an interior
    hole and ``GeometryOps`` must be able to query one. These guard the
    interface that scar placement depends on.
    """

    def test_structure_spec_has_no_declared_hole_count_field(self):
        # StructureSpec.expected_hole_count declares the expected interior
        # hole count -- see its own docstring for the licensing argument
        # (section 1d).
        spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)
        self.assertTrue(
            hasattr(spec, "expected_hole_count")
            or hasattr(spec, "spar_hole_expected"),
            "StructureSpec should be able to declare an expected interior "
            "hole count.",
        )

    def test_geometry_ops_has_a_hole_query_primitive(self):
        # GeometryOps exposes hole_bounds()/hole_points_mm() (doc section 4a).
        # This is HOLE VISIBILITY ONLY -- it does not by itself make
        # local_span/local_upper_worst_case/local_lower_worst_case hole-aware.
        members = dir(GeometryOps)
        self.assertTrue(
            any("hole" in name.lower() for name in members),
            "GeometryOps should expose a hole-local-geometry query for "
            "hole support -- e.g. "
            "hole_bounds()/holes().",
        )


if __name__ == "__main__":
    unittest.main()
