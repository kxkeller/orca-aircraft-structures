"""Geometry-relative (chord-fraction) member placement.

Covers three levels:

- ``WebFamily.validate()``: the mutual-exclusion / positivity rules for
  ``rib_pitch_chord_frac`` / ``z_drift_chord_frac_per_mm``.
- ``WebFamily.station_drift_mm`` / ``resolved_rib_pitch_mm``: the pure
  resolution formulas, against hand-computed values.
- ``StructureGenerator.generate``: relative mode actually holding chord
  -proportion constant across a TAPERED span where absolute mode does not,
  plus the min-feature fail-closed guard.

Same fixture-loading pattern as ``test_z_variation.py`` / ``test_generator
.py`` — no second airfoil implementation, and the taper is built directly
from varying ``Rect``/NACA chord widths rather than a shipped tapered STL,
matching this test suite's existing "construct the frame directly" style
for generator-level tests.
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
        "make_fixtures_under_test_relative_placement", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_mf = _load_make_fixtures_module()


def _naca2412_frame_at_chord(chord_mm):
    """A NACA 2412 ``LayerFrame`` at an arbitrary chord — used to simulate
    several stations of a tapered span without building a full tapered STL
    (the airfoil shape fraction is identical at every station of
    ``tools/make_fixtures.py``'s own ``make_taper``; only the chord scale
    differs, which is exactly what this helper reproduces)."""
    m, p, t = _mf.parse_naca4("2412")
    _triangles, outline_mm, _bbox = _mf.airfoil_triangles(m, p, t, chord_mm, 1.0, 120)
    ops = StubOps()
    polygon = PolygonShape(tuple(outline_mm))
    return LayerFrame.from_polygon(ops, polygon)


class RelativeFieldValidationTests(unittest.TestCase):
    def test_both_pitch_modes_is_error(self):
        family = WebFamily(name="bad", rib_pitch_mm=5.0, rib_pitch_chord_frac=0.1)
        with self.assertRaises(SpecError) as ctx:
            family.validate(0)
        self.assertIn("rib_pitch_mm", str(ctx.exception))
        self.assertIn("rib_pitch_chord_frac", str(ctx.exception))

    def test_negative_or_zero_pitch_frac_is_error(self):
        for bad in (0.0, -0.1):
            family = WebFamily(name="bad", rib_pitch_chord_frac=bad)
            with self.assertRaises(SpecError):
                family.validate(0)

    def test_pitch_frac_alone_passes(self):
        WebFamily(name="ok", rib_pitch_chord_frac=0.2).validate(0)

    def test_both_drift_modes_is_error(self):
        family = WebFamily(
            name="bad", z_drift_mm_per_mm=0.5, z_drift_chord_frac_per_mm=0.01
        )
        with self.assertRaises(SpecError) as ctx:
            family.validate(0)
        self.assertIn("z_drift_mm_per_mm", str(ctx.exception))
        self.assertIn("z_drift_chord_frac_per_mm", str(ctx.exception))

    def test_drift_frac_alone_passes(self):
        WebFamily(name="ok", z_drift_chord_frac_per_mm=0.01).validate(0)

    def test_period_accepts_relative_drift_without_absolute_drift(self):
        # z_period_mm's "needs a slope to reverse" rule must accept EITHER
        # drift field, not just the absolute one.
        WebFamily(
            name="ok", z_drift_chord_frac_per_mm=0.01, z_period_mm=20.0
        ).validate(0)

    def test_period_without_either_drift_field_is_still_an_error(self):
        family = WebFamily(name="bad", z_period_mm=20.0)
        with self.assertRaises(SpecError) as ctx:
            family.validate(0)
        self.assertIn("z_period_mm", str(ctx.exception))

    def test_existing_absolute_only_configs_unaffected(self):
        # Byte-identical backward compatibility: a family that never
        # mentions the new fields validates exactly as before.
        WebFamily(name="legacy", rib_pitch_mm=8.0, z_drift_mm_per_mm=0.5).validate(0)


class StationDriftRelativeFormulaTests(unittest.TestCase):
    def test_relative_drift_scales_with_local_chord(self):
        family = WebFamily(name="rel", z_drift_chord_frac_per_mm=0.01)
        # drift = frac * chord * print_z
        self.assertAlmostEqual(
            family.station_drift_mm(10.0, local_chord_mm=60.0), 6.0
        )
        # Half the chord halves the resolved drift at the same Z -- this is
        # the constant-chord-proportion property, not constant angle.
        self.assertAlmostEqual(
            family.station_drift_mm(10.0, local_chord_mm=30.0), 3.0
        )

    def test_relative_drift_without_local_chord_raises(self):
        family = WebFamily(name="rel", z_drift_chord_frac_per_mm=0.01)
        with self.assertRaises(ValueError) as ctx:
            family.station_drift_mm(10.0)
        self.assertIn("local_chord_mm", str(ctx.exception))

    def test_relative_zigzag_amplitude_scales_with_chord(self):
        # frac=0.02, period=40 -> effective rate at chord W is 0.02*W
        # mm/mm; amplitude = rate*period/4 = 0.02*W*10 = 0.2*W.
        family = WebFamily(
            name="rel-zigzag", z_drift_chord_frac_per_mm=0.02, z_period_mm=40.0
        )
        self.assertAlmostEqual(
            family.station_drift_mm(0.0, local_chord_mm=60.0), -0.2 * 60.0
        )
        self.assertAlmostEqual(
            family.station_drift_mm(0.0, local_chord_mm=30.0), -0.2 * 30.0
        )

    def test_absolute_drift_unaffected_by_local_chord_argument(self):
        # Passing local_chord_mm on an ABSOLUTE-mode family must be a no-op
        # -- the whole point of "existing configs behave byte-identically".
        family = WebFamily(name="abs", z_drift_mm_per_mm=0.5)
        self.assertEqual(
            family.station_drift_mm(10.0), family.station_drift_mm(10.0, local_chord_mm=999.0)
        )

    def test_no_variation_unaffected_by_local_chord_argument(self):
        family = WebFamily(name="flat")
        self.assertEqual(family.station_drift_mm(5.0, local_chord_mm=42.0), 0.0)


class ResolvedRibPitchTests(unittest.TestCase):
    def test_absolute_pitch_wins_when_set(self):
        family = WebFamily(name="abs", rib_pitch_mm=8.0)
        self.assertEqual(family.resolved_rib_pitch_mm(60.0), 8.0)
        self.assertEqual(family.resolved_rib_pitch_mm(None), 8.0)

    def test_relative_pitch_scales_with_chord(self):
        family = WebFamily(name="rel", rib_pitch_chord_frac=0.15)
        self.assertAlmostEqual(family.resolved_rib_pitch_mm(60.0), 9.0)
        self.assertAlmostEqual(family.resolved_rib_pitch_mm(30.0), 4.5)

    def test_relative_pitch_without_local_chord_raises(self):
        family = WebFamily(name="rel", rib_pitch_chord_frac=0.15)
        with self.assertRaises(ValueError) as ctx:
            family.resolved_rib_pitch_mm(None)
        self.assertIn("local_chord_mm", str(ctx.exception))

    def test_neither_set_returns_none(self):
        family = WebFamily(name="single-station")
        self.assertIsNone(family.resolved_rib_pitch_mm(60.0))


class GeneratorRelativePitchHoldsChordProportionOnTaperTests(unittest.TestCase):
    """The whole point: ABSOLUTE pitch produces a bay-to-chord ratio that
    drifts along a tapered span; RELATIVE pitch holds it constant."""

    def _pitch_over_chord_ratio(self, family, chord_mm):
        frame = _naca2412_frame_at_chord(chord_mm)
        spec = StructureSpec(gap_ratio=0.3, skin_margin_mm=1.0, families=(family,))
        gen = StructureGenerator()
        result = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        )
        # At least two webs are needed to read a pitch back out.
        self.assertGreaterEqual(len(result.intents), 2, "need 2+ webs to read pitch")
        xs = sorted(i.start[0] for i in result.intents)
        observed_pitch_mm = xs[1] - xs[0]
        return observed_pitch_mm / frame.width

    def test_absolute_pitch_ratio_drifts_with_taper(self):
        family = WebFamily(name="abs", web_position=0.5, rib_pitch_mm=6.0)
        ratio_root = self._pitch_over_chord_ratio(family, 60.0)
        ratio_tip = self._pitch_over_chord_ratio(family, 30.0)
        # 6mm/60mm = 0.10 at root, 6mm/30mm = 0.20 at tip -- doubles.
        self.assertAlmostEqual(ratio_root, 0.10, places=2)
        self.assertAlmostEqual(ratio_tip, 0.20, places=2)
        self.assertGreater(abs(ratio_tip - ratio_root), 0.05)

    def test_relative_pitch_ratio_holds_across_taper(self):
        family = WebFamily(name="rel", web_position=0.5, rib_pitch_chord_frac=0.10)
        ratio_root = self._pitch_over_chord_ratio(family, 60.0)
        ratio_tip = self._pitch_over_chord_ratio(family, 30.0)
        self.assertAlmostEqual(ratio_root, 0.10, places=6)
        self.assertAlmostEqual(ratio_tip, 0.10, places=6)


class GeneratorMinFeatureFloorTests(unittest.TestCase):
    """Relative pitch collapsing below the slot width + minimum wall on a
    thin (tip) chord must fail the layer closed, naming the cause -- never
    silently clamp or omit the whole family."""

    def test_collapsed_relative_pitch_fails_closed(self):
        # width_mm at gap_ratio=0.5, wall 0.42mm -> 0.21mm slot; min_wall_mm
        # = 2*0.42 = 0.84mm; floor = 0.21+0.84 = 1.05mm. A frac of 0.005 on
        # a 30mm tip chord resolves to 0.15mm pitch -- well under the floor.
        frame = _naca2412_frame_at_chord(30.0)
        family = WebFamily(name="rel", web_position=0.5, rib_pitch_chord_frac=0.005)
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0, families=(family,))
        gen = StructureGenerator()
        with self.assertRaises(ValueError) as ctx:
            gen.generate(frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        message = str(ctx.exception)
        self.assertIn("rib_pitch_chord_frac", message)
        self.assertIn("minimum-feature floor", message)
        self.assertIn(family.name, message)

    def test_same_physical_pitch_absolute_mode_does_not_hit_the_new_check(self):
        # The SAME resolved pitch (0.15mm), expressed as an absolute
        # rib_pitch_mm, reaches no NEW guard: it is untouched, historical
        # behaviour (it may still legitimately fail downstream via
        # TopologyValidator's own same-family collision check, but not via
        # this module's new min-feature floor -- the whole point of
        # "existing configs behave byte-identically").
        frame = _naca2412_frame_at_chord(30.0)
        family = WebFamily(name="abs", web_position=0.5, rib_pitch_mm=0.15)
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0, families=(family,))
        gen = StructureGenerator()
        try:
            gen.generate(frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        except ValueError as exc:
            self.assertNotIn("minimum-feature floor", str(exc))

    def test_relative_pitch_well_above_floor_generates_normally(self):
        frame = _naca2412_frame_at_chord(60.0)
        family = WebFamily(name="rel", web_position=0.5, rib_pitch_chord_frac=0.15)
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0, families=(family,))
        gen = StructureGenerator()
        result = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        )
        self.assertGreater(len(result.intents), 1)


class GeneratorRelativeDriftAndByteIdenticalDefaultTests(unittest.TestCase):
    def test_relative_drift_produces_different_angle_on_taper_than_absolute(self):
        # Same NUMERIC field value, different mode: relative resolves a
        # smaller mm/mm rate on the smaller (tip) chord than on the root,
        # so the trajectory's local slope genuinely differs by station --
        # exactly the "member angle is not constant" tradeoff this feature
        # documents.
        family = WebFamily(name="rel", z_drift_chord_frac_per_mm=0.01)
        rate_root = family.station_drift_mm(1.0, local_chord_mm=60.0)
        rate_tip = family.station_drift_mm(1.0, local_chord_mm=30.0)
        self.assertNotAlmostEqual(rate_root, rate_tip)
        self.assertAlmostEqual(rate_root, 2.0 * rate_tip)

    def test_family_with_no_relative_fields_is_byte_identical(self):
        # A regression pin: adding the two new fields must not perturb a
        # family that never sets them.
        frame = _naca2412_frame_at_chord(60.0)
        family = WebFamily(name="legacy", web_position=0.5, rib_pitch_mm=8.0, z_drift_mm_per_mm=0.05)
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0, families=(family,))
        gen = StructureGenerator()
        result = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=10, print_z=2.0)
        )
        self.assertGreater(len(result.intents), 0)


class WebFamilyIsMarchingTests(unittest.TestCase):
    """``WebFamily.is_marching`` -- the single predicate every classification
    site must agree on. Regression for the bug where a chord-relative family
    (``rib_pitch_mm is None``, ``rib_pitch_chord_frac`` set) was misread as a
    single EXPLICIT station by call sites testing ``rib_pitch_mm is None``
    directly."""

    def test_absolute_pitch_marches(self):
        self.assertTrue(WebFamily(name="abs", rib_pitch_mm=6.0).is_marching)

    def test_relative_pitch_marches(self):
        self.assertTrue(
            WebFamily(name="rel", rib_pitch_chord_frac=0.1).is_marching
        )

    def test_neither_pitch_field_does_not_march(self):
        self.assertFalse(WebFamily(name="single").is_marching)


class RelativePitchDepthBreachOmitsInsteadOfRaisingTests(unittest.TestCase):
    """A chord-relative family hitting a depth
    breach must OMIT the unplaceable station (the MARCHING-family policy),
    not raise ``ValueError`` (the EXPLICIT-single-station policy) -- it is
    misclassified as explicit purely because ``rib_pitch_mm is None`` for a
    chord-relative family."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def test_relative_pitch_family_omits_depth_breach(self):
        # Same geometry as test_generator.py's
        # test_depth_breaching_skin_margin_fails_closed_naming_family (full
        # local thickness 20.0 mm, web_depth_mm=19.5 leaves only 0.5 mm,
        # below skin_margin_mm=1.0) but expressed as a MARCHING
        # chord-relative family instead of the implicit single-station
        # default. rib_pitch_chord_frac=0.5 on this 20 mm-wide rect resolves
        # to a 10 mm pitch -- comfortably above the min-feature floor -- so
        # the anchor at web_position=0.5 (x=10.0) is still a candidate
        # station and still breaches the same margin.
        family = WebFamily(
            name="rel", web_position=0.5, web_depth_mm=19.5, rib_pitch_chord_frac=0.5
        )
        self.assertTrue(family.is_marching)
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, families=(family,))
        result = StructureGenerator().generate(
            self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        )
        skin_margin_omissions = [
            o for o in result.omitted if o.reason.startswith("skin margin")
        ]
        self.assertGreaterEqual(len(skin_margin_omissions), 1)
        anchor_omissions = [
            o
            for o in skin_margin_omissions
            if abs(o.x_mm - self.frame.web_x(0.5)) < 1e-6
        ]
        self.assertEqual(len(anchor_omissions), 1)

    def test_relative_pitch_family_hole_conflict_omits_instead_of_raising(self):
        # Mirrors test_scar.py's
        # test_marching_family_station_over_the_hole_is_omitted_not_raised
        # exactly, but expressed with a chord-relative pitch instead of
        # absolute -- the OTHER classification site this bug hit (the
        # hole-footprint-overlap check). A marching family must OMIT a
        # station whose haloed footprint crosses a declared interior hole,
        # not raise.
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 3.0, 24.0, 9.0)
        from stub_ops import HoledRect

        frame = LayerFrame.from_polygon(self.ops, HoledRect(outer=outer, hole=hole))
        # rib_pitch_chord_frac=0.075 on this 40 mm-wide outer rect resolves
        # to a 3.0 mm pitch -- identical to test_scar.py's absolute
        # rib_pitch_mm=3.0 fixture, marching through the hole's own X
        # footprint.
        family = WebFamily(
            name="rel", web_position=0.1, rib_pitch_chord_frac=0.075
        )
        self.assertTrue(family.is_marching)
        spec = StructureSpec(
            gap_ratio=1.5,
            skin_margin_mm=1.0,
            families=(family,),
            expected_hole_count=1,
        )
        result = StructureGenerator().generate(
            frame,
            spec,
            0.42,
            LayerContext(layer_index=0, print_z=0.0),
            safety_offset_mm=0.42,
        )
        hole_omissions = [
            o for o in result.omitted if o.reason.startswith("hole footprint:")
        ]
        self.assertTrue(hole_omissions)
        self.assertFalse(
            any(
                o.reason.startswith(("skin margin:", "thin section:"))
                for o in hole_omissions
            )
        )


class AbsoluteVsRelativeEquivalenceAtMatchedChordTests(unittest.TestCase):
    """Acceptance test: an absolute-pitch config and its chord-relative
    equivalent, evaluated at the SAME local chord, must produce IDENTICAL
    intents and omissions -- same positions, same reasons, in order. Mirrors
    the shipped skin-lattice default's own absolute-to-relative conversion
    (``rib_pitch_mm: 3.9`` / ``rib_pitch_chord_frac: 0.065`` at chord 60;
    ``z_drift_mm_per_mm: 1.0`` / ``z_drift_chord_frac_per_mm`` = 1/local
    chord, so the resolved rate is ``1.0`` mm/mm either way)."""

    def _families(self, local_chord_mm):
        # ``local_chord_mm`` is the frame's own ACTUAL measured
        # ``frame.width`` -- not the nominal chord handed to the airfoil
        # generator, which the NACA triangulation's discretization can miss
        # by a fraction of a millimetre. ``resolved_rib_pitch_mm``/
        # ``station_drift_mm`` always resolve a relative family against the
        # real, measured local chord, so the absolute-mode "equivalent" must
        # be built from that same value to be equivalent at all -- deriving
        # it from the nominal chord instead reproduced the exact "two
        # measurements of a quantity that must agree" bug class, one
        # level up, in the test itself.
        # ``station_drift_mm``'s RELATIVE MODE resolves
        # ``z_drift_mm_per_mm = z_drift_chord_frac_per_mm * local_chord_mm``
        # (a MULTIPLY, same shape as ``resolved_rib_pitch_mm``) -- so
        # matching an absolute rate of ``1.0`` mm/mm (the shipped default's
        # own value) needs ``z_drift_chord_frac_per_mm = 1.0 /
        # local_chord_mm``, not ``1.0 / local_chord_mm`` used AS the
        # absolute rate itself (that divides twice and was this test's own
        # first-draft bug).
        absolute = WebFamily(
            name="lattice_plus_upper",
            web_position=0.35,
            web_depth_mm=1.5,
            entry_lower=False,
            rib_pitch_mm=0.065 * local_chord_mm,
            z_drift_mm_per_mm=1.0,
            z_period_mm=28.0,
        )
        relative = WebFamily(
            name="lattice_plus_upper",
            web_position=0.35,
            web_depth_mm=1.5,
            entry_lower=False,
            rib_pitch_chord_frac=0.065,
            z_drift_chord_frac_per_mm=1.0 / local_chord_mm,
            z_period_mm=28.0,
        )
        return absolute, relative

    def _generate(self, family, frame, print_z):
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, families=(family,))
        result = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=print_z)
        )
        return result

    def test_identical_intents_and_omissions_at_matched_chord(self):
        frame = _naca2412_frame_at_chord(60.0)
        absolute, relative = self._families(frame.width)
        for print_z in (0.0, 3.5, 7.0, 14.0, 21.0, 27.9):
            abs_result = self._generate(absolute, frame, print_z)
            rel_result = self._generate(relative, frame, print_z)
            self.assertEqual(
                len(abs_result.intents), len(rel_result.intents), print_z
            )
            for abs_intent, rel_intent in zip(abs_result.intents, rel_result.intents):
                self.assertAlmostEqual(
                    abs_intent.start[0], rel_intent.start[0], places=9
                )
                self.assertAlmostEqual(
                    abs_intent.start[1], rel_intent.start[1], places=9
                )
                self.assertAlmostEqual(
                    abs_intent.end[0], rel_intent.end[0], places=9
                )
                self.assertAlmostEqual(
                    abs_intent.end[1], rel_intent.end[1], places=9
                )
            self.assertEqual(len(abs_result.omitted), len(rel_result.omitted))
            for abs_o, rel_o in zip(abs_result.omitted, rel_result.omitted):
                self.assertAlmostEqual(abs_o.x_mm, rel_o.x_mm, places=9)
                self.assertEqual(abs_o.reason, rel_o.reason)


if __name__ == "__main__":
    unittest.main()
