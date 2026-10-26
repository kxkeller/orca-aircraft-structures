"""The thin-section halo check's threshold is corrected by the LOCAL
far-surface slope across the candidate's own ordinary halo window, not the
section's global ``frame.rotation_deg``.

Covers:
    - ``estimate_far_surface_slope_deg`` (aircraft_core/frame.py): a
      known-slope trapezoid pins the secant-slope arithmetic directly; an
      off-section probe and a degenerate (zero-width) window both return
      ``None`` rather than raising.
    - Generator-level regression: the exact taper layer-21/slot-45 station (a
      genuinely FLAT, near-zero-rotation tapered section whose trailing edge
      is nonetheless measurably sloped, well outside the near-extremity
      widening trigger) is OMITTED as "thin section". Without the slope
      correction it is built, passing the halo check by ~0.00135 mm before
      disconnecting the real boolean cut by ~0.00065 mm.
    - A mid-chord control at the same spec proves the fix is slope
      -scoped, not a blanket change — mirrors
      ``test_thin_section.py``'s ``NearLeadingEdgeConvergenceRegressionTests``
      control.
"""
import importlib.util
import math
import unittest
from pathlib import Path

from aircraft_core.frame import (
    LayerFrame,
    estimate_far_surface_slope_deg,
)
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.spec import StructureSpec
from stub_ops import PolygonShape, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_far_surface_slope", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mf = _load_make_fixtures_module()

NACA = "2412"
N_POINTS = 120
ROOT_CHORD_MM = 60.0
TIP_CHORD_MM = 30.0
SPAN_MM = 60.0


def _tapered_polygon_at(z_mm: float):
    """Reproduces ``tools/verify_full_sweep_varying.py``'s own
    ``_outline_at_z_taper`` construction exactly (pivot_frac_x=0.0,
    rotate_deg=0.0, dx_mm=0.0 throughout for a pure taper), but via
    ``tools/make_fixtures.py``'s lower-level functions directly, the same
    way ``test_thin_section.py``/``test_perpendicular_entry.py`` build
    their own fixtures from that module rather than the CLI wrapper.
    """
    m, p, t = mf.parse_naca4(NACA)
    outline_unit = mf.naca4_section_outline(m, p, t, N_POINTS)

    def chord_at(zz):
        frac = zz / SPAN_MM
        return ROOT_CHORD_MM + (TIP_CHORD_MM - ROOT_CHORD_MM) * frac

    root_scaled = mf.scale_naca_outline(outline_unit, chord_at(0.0))
    rxmin, rymin, rxmax, rymax = mf.section_bbox_2d(root_scaled)
    root_shift = ((rxmin + rxmax) / 2.0, (rymin + rymax) / 2.0)

    scaled = mf.scale_naca_outline(outline_unit, chord_at(z_mm))
    cx, cy = root_shift
    outline_mm = [(x - cx, y - cy) for x, y in scaled]
    return PolygonShape(tuple(outline_mm))


class EstimateFarSurfaceSlopeDegTests(unittest.TestCase):
    """Pure-function tests against a hand-built, exactly-known-slope shape."""

    def setUp(self):
        self.ops = StubOps()

    def test_known_slope_trapezoid(self):
        # Lower boundary is the straight line y = -0.2 * x from x=0 to
        # x=10 (slope -0.2, i.e. -11.3099 degrees); upper boundary flat at
        # y=5. A simple, convex, CCW-wound polygon StubOps' Sutherland
        # -Hodgman clipping handles exactly.
        polygon = PolygonShape(
            ((0.0, 0.0), (10.0, -2.0), (10.0, 5.0), (0.0, 5.0))
        )
        got = estimate_far_surface_slope_deg(
            self.ops, polygon, 2.0, 8.0, upper=False
        )
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got, math.degrees(math.atan(-0.2)), places=6)

    def test_known_slope_trapezoid_upper_surface(self):
        # Same shape, mirrored: upper boundary slopes, lower is flat.
        polygon = PolygonShape(
            ((0.0, -5.0), (10.0, -5.0), (10.0, 2.0), (0.0, 0.0))
        )
        got = estimate_far_surface_slope_deg(
            self.ops, polygon, 2.0, 8.0, upper=True
        )
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got, math.degrees(math.atan(0.2)), places=6)

    def test_flat_surface_is_zero_slope(self):
        polygon = PolygonShape(((0.0, -5.0), (10.0, -5.0), (10.0, 5.0), (0.0, 5.0)))
        got = estimate_far_surface_slope_deg(
            self.ops, polygon, 2.0, 8.0, upper=False
        )
        self.assertAlmostEqual(got, 0.0, places=9)

    def test_returns_none_off_section(self):
        polygon = PolygonShape(((0.0, -5.0), (10.0, -5.0), (10.0, 5.0), (0.0, 5.0)))
        got = estimate_far_surface_slope_deg(
            self.ops, polygon, 20.0, 30.0, upper=False
        )
        self.assertIsNone(got)

    def test_degenerate_window_returns_none(self):
        polygon = PolygonShape(((0.0, -5.0), (10.0, -5.0), (10.0, 5.0), (0.0, 5.0)))
        got = estimate_far_surface_slope_deg(
            self.ops, polygon, 5.0, 5.0, upper=False
        )
        self.assertIsNone(got)


class TaperedTrailingEdgeThinSectionRegressionTests(unittest.TestCase):
    """Traced failure: tapered r60/t30 span60, layer 21 (Z=4.40 mm), slot 45
    (family 0, x=18.8296 mm, an unmerged 0.021 mm -wide +Y candidate). With the
    raw threshold the thin-section halo check passes this station by
    ~0.00135 mm; the real boolean cut disconnects it by ~0.00065 mm.
    Reproduced here at the GENERATOR level (StubOps, no boolean backend) — the
    required-threshold arithmetic, not a re-run of the real erosion (see
    ``tests/unit/test_erosion_near_leading_edge.py`` for the shapely-backed
    counterpart for the analogous leading-edge case).
    """

    Z_MM = 4.4
    TARGET_X_MM = 18.829558443379895
    SKIN_MARGIN_MM = 0.8608  # shipped two-sided default.
    WEB_DEPTH_MM = 1.5  # stiffener family depth.
    OUTER_WALL_MM = 0.42
    GAP_RATIO = 0.05  # matches the shipped skin_lattice_only preset.

    def setUp(self):
        self.ops = StubOps()
        self.polygon = _tapered_polygon_at(self.Z_MM)
        self.frame = LayerFrame.from_polygon(self.ops, self.polygon)
        self.web_position = (
            self.TARGET_X_MM - self.frame.min_x
        ) / self.frame.width
        self.layer = LayerContext(layer_index=21, print_z=self.Z_MM)

    def _spec(self, web_position):
        return StructureSpec(
            gap_ratio=self.GAP_RATIO,
            skin_margin_mm=self.SKIN_MARGIN_MM,
            web_position=web_position,
            web_depth_mm=self.WEB_DEPTH_MM,
        )

    def test_near_trailing_edge_candidate_is_omitted_for_thinness(self):
        spec = self._spec(self.web_position)
        result = StructureGenerator().generate(
            self.frame,
            spec,
            self.OUTER_WALL_MM,
            self.layer,
            safety_offset_mm=self.OUTER_WALL_MM,
        )
        self.assertEqual(result.intents, [])
        self.assertEqual(len(result.omitted), 1)
        omission = result.omitted[0]
        self.assertTrue(
            omission.reason.startswith("thin section:"),
            "expected a 'thin section:' omission, got: {!r}".format(omission.reason),
        )
        self.assertAlmostEqual(omission.x_mm, self.TARGET_X_MM, places=6)

    def test_local_slope_measured_matches_the_traced_value(self):
        half_span = self.GAP_RATIO * self.OUTER_WALL_MM / 2.0 + self.OUTER_WALL_MM
        slope_deg = estimate_far_surface_slope_deg(
            self.ops,
            self.polygon,
            self.TARGET_X_MM - half_span,
            self.TARGET_X_MM + half_span,
            upper=False,
        )
        self.assertIsNotNone(slope_deg)
        # Traced value at this exact station: 4.01 degrees.
        self.assertAlmostEqual(slope_deg, 4.01, delta=0.05)

    def test_mid_chord_candidate_unaffected(self):
        """Same spec, a mid-chord station far from either chord extremity: proves
        the slope correction is scoped, not a blanket change to every
        candidate — the same control as the leading-edge regression test."""
        result = StructureGenerator().generate(
            self.frame,
            self._spec(0.5),
            self.OUTER_WALL_MM,
            self.layer,
            safety_offset_mm=self.OUTER_WALL_MM,
        )
        self.assertEqual(len(result.intents), 1)
        self.assertEqual(result.omitted, [])


if __name__ == "__main__":
    unittest.main()
