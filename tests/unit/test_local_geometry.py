"""Tests for a real chord axis and LOCAL
thickness measurement in ``aircraft_core.frame`` (``local_span`` /
``web_centerline``).

These tests fail against a bounding-box-based measurement. The NACA
2412 section here is built by reusing ``tools/make_fixtures.py``'s own
point-generation code (``airfoil_triangles``), matching the pattern
``test_airfoil_geometry.py`` already uses to load that script as a module —
there is deliberately no second airfoil implementation in this file.
"""
import importlib.util
import unittest
from pathlib import Path

from aircraft_core.frame import (
    DEFAULT_PROBE_WIDTH_MM,
    LayerFrame,
    local_lower_worst_case,
    local_span,
    local_upper_worst_case,
)
from aircraft_core.generator import (
    FULL_DEPTH_MARGIN_HEADROOM_MM,
    LayerContext,
    StructureGenerator,
)
from aircraft_core.spec import StructureSpec
from stub_ops import PolygonShape, Rect, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location("make_fixtures_under_test", MAKE_FIXTURES_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mf = _load_make_fixtures_module()


def _naca2412_polygon():
    """The fixture's exact 2D section (chord 60 mm, span irrelevant here, 120
    cosine-spaced points per surface — same parameters as
    tests/fixtures/airfoil/naca2412_c60_s60.stl)."""
    m, p, t = mf.parse_naca4("2412")
    _triangles, outline_mm, bbox = mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm)), bbox


class LocalSpanAirfoilTests(unittest.TestCase):
    """local_span() measured against the real, non-rectangular NACA 2412
    section — the geometry this whole task exists to handle correctly.
    """

    def setUp(self):
        self.ops = StubOps()
        self.polygon, self.bbox = _naca2412_polygon()
        self.xmin, self.ymin, self.xmax, self.ymax = self.bbox
        self.bbox_depth = self.ymax - self.ymin

    def test_local_depth_is_narrower_than_bbox_depth_away_from_the_thickest_station(self):
        # Near the leading and trailing edges the section is much thinner
        # than its overall bounding-box depth (which is set by the single
        # thickest station elsewhere along the chord).
        for frac in (0.1, 0.9):
            x = self.xmin + frac * (self.xmax - self.xmin)
            y_min, y_max = local_span(self.ops, self.polygon, x, DEFAULT_PROBE_WIDTH_MM)
            local_depth = y_max - y_min
            self.assertLess(
                local_depth,
                self.bbox_depth,
                "chord fraction {}: local depth {} mm should be well under "
                "the bbox depth {} mm".format(frac, local_depth, self.bbox_depth),
            )

    def test_max_local_thickness_matches_naca_specification(self):
        # NACA 2412's "12" means max thickness = 12% of chord = 7.2 mm at a
        # 60 mm chord. Scan a dense set of stations and confirm the largest
        # local thickness found lands close to that value (not merely
        # "close to the bbox depth", which would pass trivially without
        # local measurement actually working).
        n = 200
        best_depth = 0.0
        for i in range(1, n):
            x = self.xmin + (self.xmax - self.xmin) * i / n
            y_min, y_max = local_span(self.ops, self.polygon, x, DEFAULT_PROBE_WIDTH_MM)
            best_depth = max(best_depth, y_max - y_min)
        self.assertAlmostEqual(best_depth, 7.2, delta=0.05)

    def test_local_span_at_several_stations_differs_from_bbox_span(self):
        # Documents the actual size of the error a bbox formula makes:
        # measured local (y_min, y_max) at several chord stations, versus
        # the section's single global bbox (min_y, max_y).
        for frac in (0.1, 0.3, 0.5, 0.7, 0.9):
            x = self.xmin + frac * (self.xmax - self.xmin)
            y_min, y_max = local_span(self.ops, self.polygon, x, DEFAULT_PROBE_WIDTH_MM)
            self.assertNotAlmostEqual(y_min, self.ymin, places=2)
            self.assertNotAlmostEqual(y_max, self.ymax, places=2)


class WebCenterlineAirfoilTests(unittest.TestCase):
    """web_centerline() on the airfoil: the margin point must land above the
    LOCAL lower surface, where a bbox formula would place it
    below the material.
    """

    def setUp(self):
        self.ops = StubOps()
        self.polygon, self.bbox = _naca2412_polygon()
        self.frame = LayerFrame.from_polygon(self.ops, self.polygon)
        self.skin_margin_mm = 1.0

    def test_margin_point_is_above_the_local_lower_surface(self):
        # web_position=0.7 is a station where a bounding-box margin
        # formula fails: the global min_y sits well below the local lower surface here.
        web_position = 0.7
        slot_width_mm = 0.4
        x = self.frame.web_x(web_position)
        local_y_min, _local_y_max = local_span(
            self.ops, self.polygon, x, DEFAULT_PROBE_WIDTH_MM
        )
        _boundary, margin = self.frame.web_centerline(
            web_position, self.skin_margin_mm, slot_width_mm
        )
        self.assertGreaterEqual(
            margin[1],
            local_y_min,
            "margin point y={} must sit at/above the local lower surface "
            "y={}".format(margin[1], local_y_min),
        )
        # It must actually be skin_margin_mm above the WORST-CASE local
        # lower surface across the slot's width, not merely above the
        # centreline point probe -- see local_lower_worst_case.
        worst_case_y_min = local_lower_worst_case(
            self.ops, self.polygon, x, slot_width_mm, DEFAULT_PROBE_WIDTH_MM
        )
        self.assertAlmostEqual(
            margin[1], worst_case_y_min + self.skin_margin_mm, places=9
        )

    def test_old_bbox_formula_would_have_undershot_the_local_lower_surface(self):
        # The bounding-box formula (min_y + skin_margin, using the section's
        # single global minimum) would place the margin point BELOW the
        # local material at this chord station.
        web_position = 0.7
        x = self.frame.web_x(web_position)
        local_y_min, _local_y_max = local_span(
            self.ops, self.polygon, x, DEFAULT_PROBE_WIDTH_MM
        )
        old_bbox_margin_y = self.frame.min_y + self.skin_margin_mm
        self.assertLess(
            old_bbox_margin_y,
            local_y_min,
            "old bbox-derived margin y={} was expected to fall below the "
            "local lower surface y={} at this station -- if it doesn't, "
            "this test no longer documents the bug it exists to "
            "prevent.".format(old_bbox_margin_y, local_y_min),
        )

    def test_generator_produces_a_locally_correct_intent_on_the_airfoil(self):
        # End-to-end through StructureGenerator, same call shape the
        # capability uses, confirming the whole path (not just local_span
        # in isolation) uses local geometry.
        spec = StructureSpec(gap_ratio=1.0, skin_margin_mm=1.0, web_position=0.7)
        outer_wall_mm = 0.4
        intent = StructureGenerator().generate(
            self.frame, spec, outer_wall_mm, LayerContext(layer_index=0, print_z=0.0)
        ).intents[0]
        x = self.frame.web_x(0.7)
        slot_width_mm = spec.resolve_slot_gap(outer_wall_mm)
        worst_case_y_max = local_upper_worst_case(
            self.ops, self.polygon, x, slot_width_mm, DEFAULT_PROBE_WIDTH_MM
        )
        worst_case_y_min = local_lower_worst_case(
            self.ops, self.polygon, x, slot_width_mm, DEFAULT_PROBE_WIDTH_MM
        )
        self.assertAlmostEqual(intent.start[1], worst_case_y_max, places=9)
        # Full-depth far end: skin_margin_mm plus the derived placement
        # headroom (FULL_DEPTH_MARGIN_HEADROOM_MM) -- see that constant's
        # docstring in aircraft_core/generator.py.
        self.assertAlmostEqual(
            intent.end[1],
            worst_case_y_min + spec.skin_margin_mm + FULL_DEPTH_MARGIN_HEADROOM_MM,
            places=9,
        )


class WorstCaseAcrossSlotWidthTests(unittest.TestCase):
    """The core mechanism of the bug fix, proven with real numbers on the
    real NACA 2412 section: the worst-case local lower surface across a
    slot-width span is strictly HIGHER (worse for remaining skin) than a
    single point probe at the span's centreline, near the leading edge
    where curvature is highest. This is exactly the disagreement that made
    the in-host slice fail at layer 5 -- a centreline-only
    probe under-measures how high the lower surface reaches elsewhere
    across the slot's own width.
    """

    def setUp(self):
        self.ops = StubOps()
        self.polygon, self.bbox = _naca2412_polygon()
        self.xmin, self.ymin, self.xmax, self.ymax = self.bbox

    def test_worst_case_lower_surface_exceeds_centreline_point_probe_near_the_leading_edge(
        self,
    ):
        # 0.5 mm aft of the leading edge on a 60 mm chord -- close enough to
        # the tip that the lower surface's slope is steep over even a small
        # span, matching the in-host bug's high-curvature station near the
        # nose (reported at layer 5, x~=0.16 mm on a much smaller wing
        # section; this fixture's chord and web position are unrelated but
        # demonstrate the same mechanism).
        x_center = self.xmin + 0.5
        slot_width_mm = 0.6  # ~1.5x a typical 0.4 mm outer wall line width.

        center_y_min, _center_y_max = local_span(
            self.ops, self.polygon, x_center, DEFAULT_PROBE_WIDTH_MM
        )
        worst_case_y_min = local_lower_worst_case(
            self.ops, self.polygon, x_center, slot_width_mm, DEFAULT_PROBE_WIDTH_MM
        )

        self.assertGreater(
            worst_case_y_min,
            center_y_min,
            "worst-case lower surface y={} across a {} mm span should "
            "exceed the centreline point probe y={} on a curved "
            "surface".format(worst_case_y_min, slot_width_mm, center_y_min),
        )
        # The actual measured numbers on this fixture, so the magnitude of
        # the local-versus-centreline effect is visible, not just its sign.
        self.assertAlmostEqual(center_y_min, -1.9165081341, places=6)
        self.assertAlmostEqual(worst_case_y_min, -1.6071330792, places=6)
        difference_mm = worst_case_y_min - center_y_min
        self.assertAlmostEqual(difference_mm, 0.3093750550, places=6)


class LocalSpanRectangleRegressionTests(unittest.TestCase):
    """A rectangle's local span must equal its bbox span at every station -- no
    regression for the coupon, and confirmation that one code path
    (local-measurement) suffices for both shapes.
    """

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)

    def test_local_span_equals_bbox_span_at_several_stations(self):
        for x in (0.5, 5.0, 10.0, 15.0, 19.5):
            y_min, y_max = local_span(self.ops, self.rect, x, DEFAULT_PROBE_WIDTH_MM)
            self.assertAlmostEqual(y_min, 0.0, places=9)
            self.assertAlmostEqual(y_max, 20.0, places=9)

    def test_web_centerline_matches_the_pre_m2_bbox_formula(self):
        frame = LayerFrame.from_polygon(self.ops, self.rect)
        boundary, margin = frame.web_centerline(0.5, skin_margin_mm=1.0, slot_width_mm=5.0)
        self.assertEqual(boundary, (10.0, 20.0))
        self.assertEqual(margin, (10.0, 1.0))

    def test_worst_case_equals_point_probe_on_a_rectangle(self):
        # Zero slope everywhere -> the worst case across ANY span must equal
        # a plain point probe at the span's centre. This is the regression
        # guard for the coupon: the worst-case-across-a-span machinery
        # must not change behaviour on flat geometry.
        x_center = 10.0
        for span_mm in (0.4, 1.0, 5.0, 15.0):
            center_y_min, center_y_max = local_span(
                self.ops, self.rect, x_center, DEFAULT_PROBE_WIDTH_MM
            )
            worst_y_min = local_lower_worst_case(
                self.ops, self.rect, x_center, span_mm, DEFAULT_PROBE_WIDTH_MM
            )
            worst_y_max = local_upper_worst_case(
                self.ops, self.rect, x_center, span_mm, DEFAULT_PROBE_WIDTH_MM
            )
            self.assertEqual(worst_y_min, center_y_min)
            self.assertEqual(worst_y_max, center_y_max)


class LocalSpanReentrantTests(unittest.TestCase):
    """A re-entrant section (an I-beam / H cross-section) must fail
    closed at a chord station where the probe line crosses the material
    more than once -- never guess which piece is load-bearing.
    """

    def setUp(self):
        self.ops = StubOps()
        # I-beam: bottom flange y in [0, 2] x in [0, 10]; top flange
        # y in [8, 10] x in [0, 10]; web (the only x in [4, 6]) connects
        # them. At x=1 (outside the web's x-range) the vertical probe
        # crosses material twice: the bottom flange, then (after an empty
        # gap where the web isn't present) the top flange.
        self.ibeam = PolygonShape(
            (
                (0, 0), (10, 0), (10, 2), (6, 2), (6, 8), (10, 8),
                (10, 10), (0, 10), (0, 8), (4, 8), (4, 2), (0, 2),
            )
        )

    def test_reentrant_station_raises(self):
        with self.assertRaises(ValueError) as ctx:
            local_span(self.ops, self.ibeam, 1.0, 0.5)
        message = str(ctx.exception)
        self.assertIn("re-entrant", message)
        self.assertIn("2 separate times", message)

    def test_station_through_the_web_does_not_raise(self):
        # Sanity check on the fixture itself: a station through the web
        # (x=5, inside [4, 6]) crosses the material exactly once (full
        # height), so it must NOT raise.
        y_min, y_max = local_span(self.ops, self.ibeam, 5.0, 0.5)
        self.assertAlmostEqual(y_min, 0.0, places=9)
        self.assertAlmostEqual(y_max, 10.0, places=9)


if __name__ == "__main__":
    unittest.main()
