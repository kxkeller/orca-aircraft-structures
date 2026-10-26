"""Hole visibility through the ``GeometryOps`` abstraction.

Not the scar mechanism. This only tests that
``GeometryOps`` can report how many interior holes a polygon has and where they
are, via ``hole_bounds``/``hole_points_mm``, and that a hole-free polygon's
existing behaviour (empty lists) is unchanged.

Exercises the test stub (``stub_ops.StubOps``) only -- the real Orca adapter
(``plugin/aircraft_structures/capability.py``'s ``OrcaGeometryOps``) has no
test-only seam to exercise outside a running host or the fake host
integration harness (``tests/integration/fake_orca.py``, exercised by
``tests/integration/test_airfoil_capability.py``-style tests).
"""

import unittest

from stub_ops import HoledRect, PolygonShape, Rect, StubOps


class HoleFreeShapesReportNoHolesTests(unittest.TestCase):
    """Existing hole-free behaviour is unchanged: every non-``HoledRect``
    shape this stub supports has zero holes."""

    def setUp(self):
        self.ops = StubOps()

    def test_rect_has_no_holes(self):
        rect = Rect(0.0, 0.0, 40.0, 20.0)
        self.assertEqual(self.ops.hole_bounds(rect), [])
        self.assertEqual(self.ops.hole_points_mm(rect), [])

    def test_polygon_shape_has_no_holes(self):
        poly = PolygonShape(((0.0, 0.0), (40.0, 0.0), (20.0, 20.0)))
        self.assertEqual(self.ops.hole_bounds(poly), [])
        self.assertEqual(self.ops.hole_points_mm(poly), [])


class HoledRectExposesOneHoleTests(unittest.TestCase):
    """A ``HoledRect`` reports exactly one hole, with correct bounds and
    points."""

    def setUp(self):
        self.ops = StubOps()
        self.outer = Rect(0.0, 0.0, 40.0, 20.0)
        self.hole = Rect(16.0, 7.0, 24.0, 13.0)
        self.holed = HoledRect(outer=self.outer, hole=self.hole)

    def test_hole_count_is_one(self):
        self.assertEqual(len(self.ops.hole_bounds(self.holed)), 1)
        self.assertEqual(len(self.ops.hole_points_mm(self.holed)), 1)

    def test_hole_bounds_match_the_hole_rect(self):
        [bounds] = self.ops.hole_bounds(self.holed)
        self.assertEqual(bounds, (16.0, 7.0, 24.0, 13.0))

    def test_hole_points_trace_the_hole_rect(self):
        [points] = self.ops.hole_points_mm(self.holed)
        self.assertEqual(
            set(points),
            {(16.0, 7.0), (24.0, 7.0), (24.0, 13.0), (16.0, 13.0)},
        )

    def test_outer_bounds_are_unaffected_by_the_hole(self):
        # bounds() still reads the OUTER contour only -- hole visibility is
        # additive, not a change to the existing outer-contour query.
        self.assertEqual(self.ops.bounds(self.holed), (0.0, 0.0, 40.0, 20.0))

    def test_contour_count_still_reports_two(self):
        # Unchanged: hole_bounds/hole_points_mm are a NEW way to see WHERE
        # the hole is; contour_count already reported THAT one exists.
        self.assertEqual(self.ops.contour_count(self.holed), 2)


class HoledRectAsAnIntersectionSubjectTests(unittest.TestCase):
    """second step: ``HoledRect`` is now a recognized ``intersection`` SUBJECT
    (``StubOps._intersection_holed_rect``), not merely a queryable shape --
    this is what unblocks ``frame.local_span`` (the primitive every
    generator/validator local-thickness measurement is built on) from running
    against a holed polygon at all. See
    ``tests/unit/test_local_span_hole_aware.py`` for the ``local_span``-level
    behaviour this enables.
    """

    def setUp(self):
        self.ops = StubOps()
        self.outer = Rect(0.0, 0.0, 40.0, 20.0)
        self.hole = Rect(16.0, 7.0, 24.0, 13.0)
        self.holed = HoledRect(outer=self.outer, hole=self.hole)

    def _strip(self, x_mm, half_width=0.05):
        return self.ops.polygon_from_points(
            (
                (x_mm - half_width, -100.0),
                (x_mm + half_width, -100.0),
                (x_mm + half_width, 100.0),
                (x_mm - half_width, 100.0),
            )
        )

    def test_station_inside_the_holes_x_footprint_returns_two_pieces(self):
        pieces = self.ops.intersection(self.holed, self._strip(20.0))
        self.assertEqual(len(pieces), 2)
        bounds = sorted(self.ops.bounds(p) for p in pieces)
        self.assertAlmostEqual(bounds[0][1], 0.0, delta=0.01)   # lower piece min_y
        self.assertAlmostEqual(bounds[0][3], 7.0, delta=0.01)   # lower piece max_y
        self.assertAlmostEqual(bounds[1][1], 13.0, delta=0.01)  # upper piece min_y
        self.assertAlmostEqual(bounds[1][3], 20.0, delta=0.01)  # upper piece max_y

    def test_station_outside_the_holes_x_footprint_returns_one_piece(self):
        pieces = self.ops.intersection(self.holed, self._strip(5.0))
        self.assertEqual(len(pieces), 1)
        min_x, min_y, max_x, max_y = self.ops.bounds(pieces[0])
        self.assertAlmostEqual(min_y, 0.0, delta=0.01)
        self.assertAlmostEqual(max_y, 20.0, delta=0.01)


if __name__ == "__main__":
    unittest.main()
