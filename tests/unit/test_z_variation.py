"""Z-dependent web placement.

Covers the two levels this feature touches:

- ``WebFamily.station_drift_mm`` (``aircraft_core/spec.py``): the pure
  formula, tested in isolation against hand-computed values.
- ``StructureGenerator.generate`` (``aircraft_core/generator.py``): the
  formula applied to a real frame, including the interaction with the
  existing omission machinery as a drifted station leaves the chord extent
  or a too-thin station.

Same fixture-loading pattern as ``test_generator.py`` — no second airfoil
implementation here.
"""
import importlib.util
import unittest
from pathlib import Path

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.spec import SpecError, StructureSpec, WebFamily
from stub_ops import PolygonShape, Rect, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_z_variation", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_mf = _load_make_fixtures_module()


def _naca2412_polygon():
    """Same 60 mm chord / 60 mm span NACA 2412 section as
    ``test_generator.py``'s ``_naca2412_polygon()``."""
    m, p, t = _mf.parse_naca4("2412")
    _triangles, outline_mm, bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm)), bbox


class StationDriftFormulaTests(unittest.TestCase):
    """``WebFamily.station_drift_mm`` in isolation, against hand-computed
    values -- independent of any frame/generator plumbing."""

    def test_no_variation_is_always_zero(self):
        family = WebFamily(name="flat")
        for z in (0.0, 1.0, 12.3, 1000.0):
            self.assertEqual(family.station_drift_mm(z), 0.0)

    def test_linear_drift_matches_formula(self):
        family = WebFamily(name="linear", z_drift_mm_per_mm=0.5)
        self.assertAlmostEqual(family.station_drift_mm(0.0), 0.0)
        self.assertAlmostEqual(family.station_drift_mm(10.0), 5.0)
        self.assertAlmostEqual(family.station_drift_mm(20.0), 10.0)
        # Unbounded: a negative rate marches the other way, unclamped.
        family_neg = WebFamily(name="linear-neg", z_drift_mm_per_mm=-0.25)
        self.assertAlmostEqual(family_neg.station_drift_mm(40.0), -10.0)

    def test_zigzag_amplitude_and_reversal(self):
        # drift = 0.5 mm/mm, period 40 mm -> amplitude = 0.5*40/4 = 5.0 mm.
        family = WebFamily(name="zigzag", z_drift_mm_per_mm=0.5, z_period_mm=40.0)
        # print_z=0 sits at the low end of the swing, not at zero -- see
        # station_drift_mm's own docstring.
        self.assertAlmostEqual(family.station_drift_mm(0.0), -5.0)
        # Rising edge: slope +0.5 mm/mm for t in [0, 20).
        self.assertAlmostEqual(family.station_drift_mm(10.0), 0.0)
        self.assertAlmostEqual(family.station_drift_mm(20.0), 5.0)
        # Falling edge: slope -0.5 mm/mm for t in [20, 40).
        self.assertAlmostEqual(family.station_drift_mm(30.0), 0.0)
        # Wraps back to the low end at the period boundary.
        self.assertAlmostEqual(family.station_drift_mm(40.0), -5.0)

    def test_zigzag_reverses_direction_at_half_period_boundary(self):
        family = WebFamily(name="zigzag", z_drift_mm_per_mm=0.5, z_period_mm=40.0)
        eps = 1e-3
        rising_slope = (
            family.station_drift_mm(20.0 - eps) - family.station_drift_mm(20.0 - 2 * eps)
        )
        falling_slope = (
            family.station_drift_mm(20.0 + 2 * eps) - family.station_drift_mm(20.0 + eps)
        )
        self.assertGreater(rising_slope, 0.0)
        self.assertLess(falling_slope, 0.0)

    def test_zigzag_stays_bounded_over_many_periods(self):
        # The whole point of the zigzag over unbounded linear drift: the
        # station never walks off, checked over 3+ full periods at a dense
        # sample of Z values.
        family = WebFamily(name="zigzag", z_drift_mm_per_mm=0.5, z_period_mm=40.0)
        amplitude = 0.5 * 40.0 / 4.0
        z = 0.0
        while z <= 3 * 40.0 + 1e-9:
            drift = family.station_drift_mm(z)
            self.assertGreaterEqual(drift, -amplitude - 1e-9)
            self.assertLessEqual(drift, amplitude + 1e-9)
            z += 0.5

    def test_determinism_direct_from_z_not_accumulated(self):
        family = WebFamily(name="zigzag", z_drift_mm_per_mm=0.3, z_period_mm=17.0)
        # Same Z queried in any order, any number of times, is identical --
        # this only holds if the formula is a pure function of print_z.
        z_values = [0.0, 5.0, 17.0, 5.0, 100.7, 0.0, 5.0]
        drifts = [family.station_drift_mm(z) for z in z_values]
        self.assertEqual(drifts[1], drifts[3])
        self.assertEqual(drifts[1], drifts[6])
        self.assertEqual(drifts[0], drifts[5])
        # Re-querying from a fresh WebFamily instance agrees too.
        fresh = WebFamily(name="zigzag", z_drift_mm_per_mm=0.3, z_period_mm=17.0)
        for z in z_values:
            self.assertEqual(family.station_drift_mm(z), fresh.station_drift_mm(z))


class SpecValidationTests(unittest.TestCase):
    def test_period_without_drift_is_error_on_family(self):
        family = WebFamily(name="bad", z_period_mm=10.0)
        with self.assertRaises(SpecError) as ctx:
            family.validate(0)
        self.assertIn("z_period_mm", str(ctx.exception))

    def test_negative_or_zero_period_is_error_on_family(self):
        for bad in (0.0, -5.0):
            family = WebFamily(name="bad", z_drift_mm_per_mm=1.0, z_period_mm=bad)
            with self.assertRaises(SpecError):
                family.validate(0)

    def test_period_without_drift_is_error_on_spec(self):
        spec = StructureSpec(gap_ratio=0.5, z_period_mm=10.0)
        with self.assertRaises(SpecError) as ctx:
            spec.validate()
        self.assertIn("z_period_mm", str(ctx.exception))

    def test_valid_drift_only_passes(self):
        StructureSpec(gap_ratio=0.5, z_drift_mm_per_mm=0.5).validate()

    def test_valid_drift_and_period_passes(self):
        StructureSpec(gap_ratio=0.5, z_drift_mm_per_mm=0.5, z_period_mm=40.0).validate()


class GeneratorNoVariationTests(unittest.TestCase):
    """With Z-variation unset, output must be byte-identical to behaviour
    regardless of which layer/Z is asked for."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def test_result_identical_across_layer_index_and_z(self):
        spec = StructureSpec(
            gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5, rib_pitch_mm=5.0
        )
        gen = StructureGenerator()
        baseline = gen.generate(
            self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        )
        for index, z in ((1, 0.2), (50, 10.0), (300, 60.0)):
            result = gen.generate(
                self.frame, spec, 0.42, LayerContext(layer_index=index, print_z=z)
            )
            self.assertEqual(
                [i.start for i in result.intents], [i.start for i in baseline.intents]
            )
            self.assertEqual(
                [o.x_mm for o in result.omitted], [o.x_mm for o in baseline.omitted]
            )


class GeneratorDriftTests(unittest.TestCase):
    """Linear drift moves the station by the expected amount between two
    known Z values, on the coupon and on the real NACA 2412."""

    def test_drift_on_coupon_between_two_z(self):
        ops = StubOps()
        rect = Rect(0.0, 0.0, 60.0, 20.0)
        frame = LayerFrame.from_polygon(ops, rect)
        spec = StructureSpec(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            web_position=0.5,
            z_drift_mm_per_mm=0.1,
        )
        gen = StructureGenerator()
        low = gen.generate(frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        high = gen.generate(frame, spec, 0.42, LayerContext(layer_index=100, print_z=20.0))
        self.assertAlmostEqual(low.intents[0].start[0], 30.0)
        self.assertAlmostEqual(high.intents[0].start[0], 32.0)
        self.assertAlmostEqual(high.intents[0].start[0] - low.intents[0].start[0], 2.0)

    def test_drift_on_real_naca2412_between_two_z(self):
        ops = StubOps()
        polygon, _bbox = _naca2412_polygon()
        frame = LayerFrame.from_polygon(ops, polygon)
        spec = StructureSpec(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            web_position=0.5,
            z_drift_mm_per_mm=0.05,
        )
        gen = StructureGenerator()
        low = gen.generate(frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        high = gen.generate(frame, spec, 0.42, LayerContext(layer_index=150, print_z=30.0))
        self.assertEqual(len(low.intents), 1)
        self.assertEqual(len(high.intents), 1)
        self.assertAlmostEqual(
            high.intents[0].start[0] - low.intents[0].start[0], 1.5, places=6
        )


class GeneratorZigzagTests(unittest.TestCase):
    """Zigzag over the generator: direction reverses at the period
    boundary, and stations stay within the chord over many periods."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 60.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)
        # amplitude = 2.0*20/4 = 10.0 mm, well within the 60 mm chord
        # (anchor at x=30, so the swing covers [20, 40]).
        self.spec = StructureSpec(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            web_position=0.5,
            z_drift_mm_per_mm=2.0,
            z_period_mm=20.0,
        )

    def test_direction_reverses_at_period_boundary(self):
        gen = StructureGenerator()
        xs = {}
        for z in (0.0, 5.0, 10.0, 15.0, 20.0):
            result = gen.generate(
                self.frame, self.spec, 0.42, LayerContext(layer_index=0, print_z=z)
            )
            xs[z] = result.intents[0].start[0]
        rising = xs[5.0] - xs[0.0]
        falling = xs[15.0] - xs[10.0]
        self.assertGreater(rising, 0.0)
        self.assertLess(falling, 0.0)
        # Wraps back to the same station one full period later.
        self.assertAlmostEqual(xs[20.0], xs[0.0])

    def test_stations_stay_within_chord_over_three_periods(self):
        gen = StructureGenerator()
        z = 0.0
        max_z = 3 * 20.0
        while z <= max_z + 1e-9:
            result = gen.generate(
                self.frame, self.spec, 0.42, LayerContext(layer_index=0, print_z=z)
            )
            self.assertEqual(
                len(result.omitted),
                0,
                "station drifted outside the chord extent at print_z={}: {}".format(
                    z, result.omitted
                ),
            )
            x = result.intents[0].start[0]
            self.assertGreaterEqual(x, self.frame.min_x)
            self.assertLessEqual(x, self.frame.max_x)
            z += 1.0


class GeneratorDriftOmissionTests(unittest.TestCase):
    """Omission stays deterministic and recorded (never silently dropped)
    when drift takes a web off-chord or into a too-thin station."""

    def test_drift_off_chord_extent_is_recorded(self):
        # 20 mm rect; a single web (rib_pitch_mm=None -> only the anchor
        # itself is a candidate -- see StructureGenerator's anchoring
        # rule), web_position=0.5 so the undrifted anchor is x=10.0, dead
        # centre. gap_ratio=1.5 against outer_wall=0.42 -> width_mm=0.63,
        # half=0.315.
        #
        # At print_z=0.0 the drift is zero: the anchor sits at x=10.0, well
        # inside [0, 20], and is produced normally -- nothing about this
        # family looks unusual yet.
        #
        # At print_z=1.0 the drift is 9.7 mm: the anchor sits at x=19.7,
        # still inside the chord extent itself, but its FOOTPRINT
        # [19.385, 20.015] pokes 0.015 mm past max_x=20.0 -- the exact
        # "chord extent" case StructureGenerator's OMISSION POLICY
        # documents (a candidate whose centre survives marching but whose
        # width does not fit). This is the case where a web
        # that drifts off the chord extent is omitted deterministically
        # and recorded: same spec, same anchor,
        # different Z, and only Z decided whether this family's one web
        # survives.
        ops = StubOps()
        rect = Rect(0.0, 0.0, 20.0, 20.0)
        frame = LayerFrame.from_polygon(ops, rect)
        spec = StructureSpec(
            gap_ratio=1.5,
            skin_margin_mm=1.0,
            web_position=0.5,
            z_drift_mm_per_mm=9.7,
        )
        gen = StructureGenerator()

        at_root = gen.generate(frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        self.assertEqual(len(at_root.intents), 1)
        self.assertAlmostEqual(at_root.intents[0].start[0], 10.0)
        self.assertEqual(at_root.omitted, [])

        # at print_z=1.0 the DEFAULT family's own single candidate is still
        # omitted for exactly the reason this test names (below) -- but this
        # 20mm square Rect fixture (fill ratio 1.0) also reaches the
        # zero-intent feature fallback, which legitimately contributes its own
        # intent/omissions on a region this comfortably sized. Assert on the
        # DEFAULT family's own omission, not the whole -layer intent count.
        drifted = gen.generate(frame, spec, 0.42, LayerContext(layer_index=5, print_z=1.0))
        default_omitted = [o for o in drifted.omitted if o.family_name == "default"]
        self.assertEqual(len(default_omitted), 1)
        self.assertEqual(
            [i for i in drifted.intents if i.family_name == "default"], []
        )
        self.assertAlmostEqual(default_omitted[0].x_mm, 19.7)
        self.assertTrue(default_omitted[0].reason.startswith("chord extent"), default_omitted[0].reason)

    def test_drift_into_thin_station_on_real_airfoil_is_recorded(self):
        # On the NACA 2412 fixture, web_position=0.5 is comfortably thick,
        # but a large drift can walk the station out toward the thin
        # trailing edge, where MultiWebAirfoilGeneratorTests already
        # establishes a "skin margin" omission at web_position~0.95.
        ops = StubOps()
        polygon, _bbox = _naca2412_polygon()
        frame = LayerFrame.from_polygon(ops, polygon)
        anchor_x = frame.web_x(0.5)
        trailing_x = frame.web_x(0.95)
        drift_needed = trailing_x - anchor_x
        spec = StructureSpec(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            web_position=0.5,
            z_drift_mm_per_mm=drift_needed,  # drift(print_z=1.0) == drift_needed
        )
        gen = StructureGenerator()
        result = gen.generate(frame, spec, 0.42, LayerContext(layer_index=0, print_z=1.0))
        self.assertEqual(result.intents, [])
        self.assertEqual(len(result.omitted), 1)
        self.assertTrue(result.omitted[0].reason.startswith("skin margin"))
        self.assertAlmostEqual(result.omitted[0].x_mm, trailing_x, places=6)


if __name__ == "__main__":
    unittest.main()
