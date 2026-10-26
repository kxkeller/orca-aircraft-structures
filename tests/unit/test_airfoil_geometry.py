"""Tests for the NACA 4-digit airfoil geometry in tools/make_fixtures.py (the
first action).

Loaded via importlib, matching test_bundle.py's pattern for exercising a
tools/ script as a module without adding it to the package layout.
"""
import importlib.util
import math
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location("make_fixtures_under_test", MAKE_FIXTURES_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mf = _load_make_fixtures_module()


class Naca4ParseTests(unittest.TestCase):
    def test_parses_2412(self):
        m, p, t = mf.parse_naca4("2412")
        self.assertAlmostEqual(m, 0.02)
        self.assertAlmostEqual(p, 0.4)
        self.assertAlmostEqual(t, 0.12)

    def test_rejects_wrong_length(self):
        with self.assertRaises(ValueError):
            mf.parse_naca4("241")

    def test_rejects_non_digits(self):
        with self.assertRaises(ValueError):
            mf.parse_naca4("241x")

    def test_rejects_zero_thickness(self):
        with self.assertRaises(ValueError):
            mf.parse_naca4("2400")


class ClosedTrailingEdgeTests(unittest.TestCase):
    def test_thickness_is_zero_at_trailing_edge(self):
        # Closed-TE coefficients (a4 = -0.1036) make yt(1) == 0 analytically.
        self.assertAlmostEqual(mf.naca4_thickness(1.0, 0.12), 0.0, places=12)

    def test_upper_and_lower_share_the_trailing_edge_point(self):
        upper, lower = mf.naca4_surface_points(0.02, 0.4, 0.12, 50)
        self.assertEqual(upper[-1], lower[-1])

    def test_upper_and_lower_share_the_leading_edge_point(self):
        upper, lower = mf.naca4_surface_points(0.02, 0.4, 0.12, 50)
        self.assertEqual(upper[0], lower[0])
        self.assertEqual(upper[0], (0.0, 0.0))

    def test_outline_has_no_duplicate_adjacent_points(self):
        # A degenerate (zero-length) edge would indicate a non-closing TE
        # or a spacing bug producing coincident points.
        outline = mf.naca4_section_outline(0.02, 0.4, 0.12, 50)
        n = len(outline)
        for i in range(n):
            a = outline[i]
            b = outline[(i + 1) % n]
            self.assertNotEqual(a, b, f"degenerate edge at outline index {i}")


class ChordLengthTests(unittest.TestCase):
    def test_scaled_outline_spans_the_requested_chord(self):
        chord_mm = 60.0
        triangles, outline_mm, bbox = mf.airfoil_triangles(0.02, 0.4, 0.12, chord_mm, 60.0, 80)
        xmin, _ymin, xmax, _ymax = bbox
        self.assertAlmostEqual(xmax - xmin, chord_mm, places=3)

    def test_chord_scales_linearly(self):
        _t1, _o1, bbox1 = mf.airfoil_triangles(0.02, 0.4, 0.12, 60.0, 60.0, 80)
        _t2, _o2, bbox2 = mf.airfoil_triangles(0.02, 0.4, 0.12, 120.0, 60.0, 80)
        span1 = bbox1[2] - bbox1[0]
        span2 = bbox2[2] - bbox2[0]
        self.assertAlmostEqual(span2, 2.0 * span1, places=3)


class CamberTests(unittest.TestCase):
    def test_cambered_section_upper_and_lower_are_not_mirror_images(self):
        # For a cambered section (m > 0), the upper and lower surfaces are
        # not reflections of each other through y=0 -- if they were, the
        # camber line would be doing nothing.
        upper, lower = mf.naca4_surface_points(0.02, 0.4, 0.12, 60)
        mismatches = 0
        for (xu, yu), (xl, yl) in zip(upper[1:-1], lower[1:-1]):
            if abs(yu + yl) > 1e-6:
                mismatches += 1
        self.assertGreater(
            mismatches, 0,
            "cambered section's upper/lower surfaces are mirror images "
            "through y=0 -- camber is not being applied",
        )

    def test_symmetric_section_upper_and_lower_are_mirror_images(self):
        # NACA 00xx: m=0, p=0 -- camber line is zero everywhere, so upper
        # and lower surfaces must be exact reflections through y=0.
        upper, lower = mf.naca4_surface_points(0.0, 0.0, 0.12, 60)
        for (xu, yu), (xl, yl) in zip(upper, lower):
            self.assertAlmostEqual(xu, xl, places=12)
            self.assertAlmostEqual(yu, -yl, places=12)

    def test_camber_line_nonzero_away_from_le_and_te(self):
        yc, _dyc = mf.naca4_camber(0.4, 0.02, 0.4)  # at max-camber station
        self.assertGreater(yc, 0.0)


class CosineSpacingTests(unittest.TestCase):
    def test_chord_fraction_parameter_is_monotonically_increasing(self):
        # This is the raw chordwise parameter the cosine spacing controls.
        # (The *offset* upper/lower surface x, after the perpendicular
        # thickness/camber offset, is not guaranteed monotonic right at a
        # cambered leading edge -- that is real airfoil geometry, not a
        # spacing bug, so it is deliberately not asserted here.)
        xs = mf.cosine_spaced_chord_fractions(100)
        self.assertEqual(xs[0], 0.0)
        self.assertAlmostEqual(xs[-1], 1.0, places=12)
        for i in range(1, len(xs)):
            self.assertGreater(xs[i], xs[i - 1], f"x not increasing at index {i}")

    def test_leading_edge_region_is_denser_than_a_uniform_spacing_would_be(self):
        # Cosine spacing packs points near x=0 and x=1. The first interval
        # should be much shorter than a uniform 1/(n-1) step would give.
        n = 100
        xs = mf.cosine_spaced_chord_fractions(n)
        first_step = xs[1] - xs[0]
        uniform_step = 1.0 / (n - 1)
        self.assertLess(first_step, uniform_step / 4.0)

    def test_rejects_too_few_points(self):
        with self.assertRaises(ValueError):
            mf.cosine_spaced_chord_fractions(2)


class WatertightMeshTests(unittest.TestCase):
    def test_airfoil_mesh_is_watertight(self):
        triangles, _outline, _bbox = mf.airfoil_triangles(0.02, 0.4, 0.12, 60.0, 60.0, 40)
        ok, details, problems = mf.check_watertight(triangles)
        self.assertTrue(ok, f"{details}; problems: {problems}")

    def test_triangle_count_matches_side_plus_cap_formula(self):
        n_points = 40
        triangles, outline, _bbox = mf.airfoil_triangles(0.02, 0.4, 0.12, 60.0, 60.0, n_points)
        n_outline = len(outline)
        self.assertEqual(n_outline, 2 * n_points - 2)
        expected = 2 * n_outline + 2 * (n_outline - 2)  # side quads (2 tris) + two fan caps
        self.assertEqual(len(triangles), expected)

    def test_section_fill_ratio_is_well_below_rectangularity_min(self):
        # This is the fill ratio the README documents; the
        # RECTANGULARITY_MIN=0.90 fence is expected to refuse it; assert it lands
        # in the documented ~0.65-0.70 band, not merely "below 0.90", so a
        # silent change to the geometry construction is caught.
        _triangles, outline, bbox = mf.airfoil_triangles(0.02, 0.4, 0.12, 60.0, 60.0, 120)
        area = mf.polygon_area_2d(outline)
        xmin, ymin, xmax, ymax = bbox
        bbox_area = (xmax - xmin) * (ymax - ymin)
        fill_ratio = area / bbox_area
        self.assertGreater(fill_ratio, 0.60)
        self.assertLess(fill_ratio, 0.75)


if __name__ == "__main__":
    unittest.main()
