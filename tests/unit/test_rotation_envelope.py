"""Tests for the rotation envelope.

Covers:
    - estimate_rotation_deg()'s accuracy on an unrotated NACA 2412 section
      and on a section rotated to match the twist fixture's tip (6 degrees
      washout about the quarter chord).
    - A section rotated past the threshold fails closed, naming the
      measured angle and the threshold.
    - A section within the envelope slices normally.
    - corrected_skin_margin_mm() matches the true chord-perpendicular
      margin requirement to a stated tolerance, and the generator's own
      margin arithmetic uses the corrected figure.
    - No regression on the unrotated coupon/airfoil fixtures already
      covered elsewhere (test_frame.py, test_local_geometry.py,
      test_generator.py) -- this file adds explicit rotation-specific
      assertions rather than re-testing that machinery.
"""
import importlib.util
import math
import unittest
from pathlib import Path

from aircraft_core.frame import (
    DEFAULT_MAX_ROTATION_DEG,
    LayerFrame,
    check_rotation_envelope,
    corrected_skin_margin_mm,
    estimate_rotation_deg,
)
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.spec import StructureSpec
from stub_ops import PolygonShape, Rect, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_rotation", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mf = _load_make_fixtures_module()

NACA = "2412"
CHORD_MM = 60.0
N_POINTS = 120


def _naca2412_polygon():
    """Same unrotated section test_local_geometry.py/test_generator.py use."""
    m, p, t = mf.parse_naca4(NACA)
    _triangles, outline_mm, bbox = mf.airfoil_triangles(m, p, t, CHORD_MM, CHORD_MM, N_POINTS)
    return PolygonShape(tuple(outline_mm)), bbox


def _rotated_naca2412_polygon(washout_deg):
    """A NACA 2412 section rotated by ``washout_deg`` about the quarter
    chord -- the exact convention tests/fixtures/twist/ uses (rotation axis
    at 25% of chord back from the leading edge)."""
    m, p, t = mf.parse_naca4(NACA)
    outline_unit = mf.naca4_section_outline(m, p, t, N_POINTS)
    points = mf.build_station_outline(
        outline_unit, CHORD_MM, washout_deg, 0.25, 0.0, 0.0, (0.0, 0.0)
    )
    return PolygonShape(tuple(points))


class RotationEstimationTests(unittest.TestCase):
    """estimate_rotation_deg()'s measured accuracy."""

    def test_rect_coupon_is_exactly_zero(self):
        # An axis-aligned rectangle's four corners tie for min/max X two at
        # a time; averaging the tied Y values (rather than picking one by
        # iteration order) is what makes this exact, not merely close.
        ops = StubOps()
        rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.assertEqual(estimate_rotation_deg(ops, rect), 0.0)

    def test_unrotated_naca2412_is_approximately_zero(self):
        # Real (non-rectangular, blunt-leading-edge) airfoil geometry: the
        # extreme-X estimator carries a small, documented bias here (see
        # frame.py's _ROTATION_DEADZONE_DEG docstring) rather than landing
        # on an exact 0.0. Measured: -0.1343 degrees on this exact fixture.
        ops = StubOps()
        polygon, _bbox = _naca2412_polygon()
        estimate = estimate_rotation_deg(ops, polygon)
        self.assertLess(abs(estimate), 0.5, "unrotated NACA 2412 estimate should be ~0 degrees")
        self.assertAlmostEqual(estimate, -0.1343, places=3)

    def test_twist_fixture_tip_rotation_is_approximately_six_degrees(self):
        # The twist fixture's tip station: 6 degrees washout about the
        # quarter chord (tests/fixtures/twist/README.md). Measured
        # estimate: 5.8657 degrees, error -0.1343 degrees -- the same
        # systematic bias as the unrotated case above (it does not grow
        # meaningfully over this range; see DEFAULT_MAX_ROTATION_DEG's
        # docstring for the fuller table up to 30 degrees).
        ops = StubOps()
        polygon = _rotated_naca2412_polygon(6.0)
        estimate = estimate_rotation_deg(ops, polygon)
        self.assertAlmostEqual(estimate, 6.0, delta=0.5)
        self.assertAlmostEqual(estimate, 5.8657, places=3)


class CorrectedThicknessMatchesGroundTruthTests(unittest.TestCase):
    """The cos(theta) correction against PHYSICAL ground truth, not just its
    own algebra: tests/fixtures/twist/README.md states the twist fixture's
    local max thickness, measured in the section's OWN rotated frame
    (i.e. the true chord-perpendicular thickness, constant along the span
    because chord is constant), as 7.1986 mm. A rotated section's raw
    world-Y reading overstates that; multiplying by cos(the estimated
    rotation) should recover it, to a tolerance wide enough to absorb the
    estimator's own small bias (see RotationEstimationTests) plus the
    station-scan's own granularity (same pattern as
    test_local_geometry.py's test_max_local_thickness_matches_naca
    _specification).
    """

    def test_six_degree_section_corrected_thickness_matches_the_readme_value(self):
        ops = StubOps()
        polygon = _rotated_naca2412_polygon(6.0)
        xs = [pt[0] for pt in polygon.points]
        xmin, xmax = min(xs), max(xs)

        from aircraft_core.frame import local_span, DEFAULT_PROBE_WIDTH_MM

        n = 400
        raw_max_thickness = 0.0
        for i in range(1, n):
            x = xmin + (xmax - xmin) * i / n
            y_min, y_max = local_span(ops, polygon, x, DEFAULT_PROBE_WIDTH_MM)
            raw_max_thickness = max(raw_max_thickness, y_max - y_min)

        # The raw (uncorrected) reading measurably overstates the true 7.1986
        # mm -- this is the bug exists to bound, asserted explicitly so this
        # test would fail loudly if the correction below were a no-op.
        self.assertGreater(raw_max_thickness, 7.1986 + 0.01)

        estimate = estimate_rotation_deg(ops, polygon)
        corrected_max_thickness = raw_max_thickness * math.cos(math.radians(estimate))

        # Tolerance: 0.05 mm absorbs the estimator's own ~0.13 degree bias
        # (RotationEstimationTests) plus this scan's station granularity.
        # Measured residual on this exact fixture: ~0.025 mm.
        self.assertAlmostEqual(corrected_max_thickness, 7.1986, delta=0.05)


class RotationEnvelopeTests(unittest.TestCase):
    """check_rotation_envelope(): fails closed past the threshold, passes
    within it."""

    def test_within_threshold_does_not_raise(self):
        check_rotation_envelope(6.0, DEFAULT_MAX_ROTATION_DEG)

    def test_past_threshold_raises_naming_angle_and_threshold(self):
        with self.assertRaises(ValueError) as ctx:
            check_rotation_envelope(45.0, 30.0)
        message = str(ctx.exception)
        self.assertIn("45", message)
        self.assertIn("30", message)
        self.assertIn("1/cos", message)

    def test_exactly_at_threshold_does_not_raise(self):
        # Floating-point-noise tolerance: a section rotated to EXACTLY the
        # threshold must not be spuriously refused.
        check_rotation_envelope(30.0, 30.0)

    def test_generator_fails_closed_past_the_threshold(self):
        ops = StubOps()
        polygon = _rotated_naca2412_polygon(45.0)
        frame = LayerFrame.from_polygon(ops, polygon)
        spec = StructureSpec(gap_ratio=1.0, skin_margin_mm=1.0, web_position=0.5)
        with self.assertRaises(ValueError) as ctx:
            StructureGenerator().generate(
                frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
            )
        message = str(ctx.exception)
        self.assertIn("30", message)  # the default threshold
        self.assertIn("degrees", message)

    def test_generator_slices_normally_within_the_envelope(self):
        # A realistic washout (6 degrees) must slice through without
        # raising and without needing an explicit max_rotation_deg override.
        ops = StubOps()
        polygon = _rotated_naca2412_polygon(6.0)
        frame = LayerFrame.from_polygon(ops, polygon)
        spec = StructureSpec(gap_ratio=1.0, skin_margin_mm=1.0, web_position=0.5)
        result = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        )
        self.assertEqual(len(result.intents), 1)

    def test_configurable_threshold_via_spec(self):
        # spec.max_rotation_deg overrides the module default.
        ops = StubOps()
        polygon = _rotated_naca2412_polygon(10.0)
        frame = LayerFrame.from_polygon(ops, polygon)
        spec = StructureSpec(
            gap_ratio=1.0, skin_margin_mm=1.0, web_position=0.5, max_rotation_deg=5.0
        )
        with self.assertRaises(ValueError) as ctx:
            StructureGenerator().generate(
                frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
            )
        self.assertIn("5", str(ctx.exception))


class CorrectedMarginTests(unittest.TestCase):
    """corrected_skin_margin_mm(): the cos(theta) correction."""

    def test_within_deadzone_is_unchanged(self):
        self.assertEqual(corrected_skin_margin_mm(1.0, 0.0), 1.0)
        self.assertEqual(corrected_skin_margin_mm(1.0, 0.5), 1.0)

    def test_six_degrees_matches_the_measured_overstatement(self):
        # Milestone table: at 6 degrees washout, local_span overstates
        # thickness by 1/cos(6deg). The corrected margin must therefore be
        # skin_margin_mm / cos(6deg) exactly (to floating-point precision).
        margin = corrected_skin_margin_mm(1.0, 6.0)
        self.assertAlmostEqual(margin, 1.0 / math.cos(math.radians(6.0)), places=12)
        # As an absolute figure: ~1.0055 mm for a 1.0 mm request.
        self.assertAlmostEqual(margin, 1.00551, places=5)

    def test_thirty_degrees_matches_the_measured_overstatement(self):
        margin = corrected_skin_margin_mm(1.0, 30.0)
        self.assertAlmostEqual(margin, 1.0 / math.cos(math.radians(30.0)), places=12)

    def test_negative_rotation_corrects_the_same_as_positive(self):
        self.assertAlmostEqual(
            corrected_skin_margin_mm(1.0, -6.0), corrected_skin_margin_mm(1.0, 6.0), places=12
        )

    def test_generator_applies_the_corrected_margin_on_a_rotated_section(self):
        # End-to-end: a station whose UNCORRECTED margin just barely fits
        # must be omitted once rotation-corrected -- the exact "optimistic
        # thickness reading" failure mode the correction removes.
        ops = StubOps()
        polygon = _rotated_naca2412_polygon(6.0)
        frame = LayerFrame.from_polygon(ops, polygon)

        # Find a skin_margin_mm that fits the RAW (uncorrected) reading at
        # web_position=0.5 but not the corrected one, by bisecting between
        # a margin that is known to build a web and one that is known not
        # to, using the corrected generator itself as the source of truth
        # for "no web" and a hand check against the raw local thickness for
        # "would have fit raw".
        from aircraft_core.frame import local_lower_worst_case, local_upper_worst_case, DEFAULT_PROBE_WIDTH_MM

        outer_wall_mm = 0.42
        web_position = 0.5
        x = frame.web_x(web_position)
        slot_width_mm = 1.0 * outer_wall_mm  # gap_ratio=1.0
        y_max = local_upper_worst_case(ops, polygon, x, slot_width_mm, DEFAULT_PROBE_WIDTH_MM)
        y_min = local_lower_worst_case(ops, polygon, x, slot_width_mm, DEFAULT_PROBE_WIDTH_MM)
        raw_thickness = y_max - y_min

        theta = math.radians(frame.rotation_deg)
        cos_theta = math.cos(theta)
        # Choose a margin strictly between raw_thickness (fails even
        # uncorrected) and raw_thickness * cos_theta (fits raw, fails
        # corrected): the corrected midpoint.
        margin_mm = raw_thickness * (cos_theta + 1.0) / 2.0
        self.assertLess(margin_mm, raw_thickness)  # sanity: fits the raw check
        self.assertGreater(margin_mm * (1.0 / cos_theta), raw_thickness)  # fails corrected

        spec = StructureSpec(
            gap_ratio=1.0, skin_margin_mm=margin_mm, web_position=web_position
        )
        result = StructureGenerator().generate(
            frame, spec, outer_wall_mm, LayerContext(layer_index=0, print_z=0.0)
        )
        self.assertEqual(result.intents, [])
        self.assertEqual(len(result.omitted), 1)
        self.assertTrue(result.omitted[0].reason.startswith("skin margin"))


class UnrotatedRegressionTests(unittest.TestCase):
    """Every unrotated fixture family (coupon, airfoil) must behave EXACTLY as
    without the correction -- corrected_skin_margin_mm must be a no-op within the
    deadzone, so intent geometry is bit-identical."""

    def test_rect_coupon_intent_unchanged(self):
        ops = StubOps()
        rect = Rect(0.0, 0.0, 20.0, 20.0)
        frame = LayerFrame.from_polygon(ops, rect)
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5)
        intent = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        ).intents[0]
        self.assertEqual(intent.start, (10.0, 20.0))
        self.assertEqual(intent.end[0], 10.0)

    def test_unrotated_airfoil_margin_point_unchanged(self):
        ops = StubOps()
        polygon, _bbox = _naca2412_polygon()
        frame = LayerFrame.from_polygon(ops, polygon)
        boundary, margin = frame.web_centerline(0.7, skin_margin_mm=1.0, slot_width_mm=0.4)
        from aircraft_core.frame import local_lower_worst_case, DEFAULT_PROBE_WIDTH_MM

        x = frame.web_x(0.7)
        worst_case_y_min = local_lower_worst_case(ops, polygon, x, 0.4, DEFAULT_PROBE_WIDTH_MM)
        self.assertAlmostEqual(margin[1], worst_case_y_min + 1.0, places=9)


if __name__ == "__main__":
    unittest.main()
