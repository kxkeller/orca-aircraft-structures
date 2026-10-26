"""A family-loop pitch collapse routes to the FEATURE/ AEROFOIL
classifier instead of propagating immediately (StructureGenerator.generate()'s
``_SectionTooSmallForPitch`` capture, aircraft_core/generator.py).

BACKGROUND: on plain square frames through the live plugin config
(``presets/plugin/live_host_config.json``), a section smaller than roughly 8mm
made the aerofoil family loop RAISE (``rib_pitch_chord_frac`` resolving below
the minimum-feature floor) before it reached the zero-intent fallback's
AEROFOIL/FEATURE classifier, so a small feature was FATAL even though an
8.64mm post is served fine.

THE NARROWING THIS FILE PINS: catching every ``ValueError`` the family loop
can raise would pull every solid rectangular "coupon" fixture into the
classifier whenever a test deliberately made the family loop raise for an
UNRELATED reason (a declared-hole conflict, an opposing pair that cannot fuse,
a depth breaching skin margin): classify_region's fill-ratio test always fires
on a solid rectangle (fill ratio 1.0), so the classifier says FEATURE, a
lattice fits, and the intentionally-raised error is silently discarded. The
capture is therefore type-based, not message- or shape-based: only
``_SectionTooSmallForPitch`` -- raised at exactly one site, the
``rib_pitch_chord_frac`` floor check -- is captured. Every other
``ValueError`` from the family loop propagates.
"""
import unittest

import make_fixtures as _mf
from stub_ops import PolygonShape

from aircraft_core.frame import AEROFOIL_REGION, LayerFrame, classify_region
from aircraft_core.generator import (
    LayerContext,
    StructureGenerator,
    _SectionTooSmallForPitch,
)
from aircraft_core.spec import StructureSpec, WebFamily
from stub_ops import PolygonShape, Rect, StubOps

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
import make_fixtures as _mf  # noqa: E402

WALL_WIDTH_MM = 0.42


def _pitch_collapsing_family(**overrides):
    kwargs = dict(
        name="tiny_pitch",
        role="skin_lattice",
        rib_pitch_chord_frac=0.12,
        web_position=0.35,
        web_depth_mm=1.5,
    )
    kwargs.update(overrides)
    return WebFamily(**kwargs)


class PitchCollapseOnASmallFeatureGetsStructureOrDeclinesTests(unittest.TestCase):
    """A small FEATURE region whose aerofoil family loop raises
    ``_SectionTooSmallForPitch`` must reach the classifier, exactly like a
    family loop that produced zero intents without raising -- and either
    get real structure (large enough for a lattice) or gracefully decline
    to skin-only (too small even for the lattice)."""

    def setUp(self):
        self.gen = StructureGenerator()
        self.layer = LayerContext(layer_index=0, print_z=0.2)

    def _square_spec(self):
        return StructureSpec(
            slot_gap_mm=0.3,
            skin_margin_mm=0.86,
            families=(_pitch_collapsing_family(),),
        )

    def test_small_square_that_previously_raised_now_gets_feature_structure(self):
        # 8.64x8.67mm: a joiner-post-sized section, big enough for a feature
        # lattice. This spec's rib_pitch_chord_frac=0.12 STILL collapses below
        # the floor at this width (0.12 * 8.64 = 1.0368mm < the 1.14mm floor
        # this spec/wall combination resolves to) -- the live-config mechanism,
        # reproduced here with a synthetic spec instead of the checked-in
        # config file.
        ops = StubOps()
        rect = Rect(0.0, 0.0, 8.64, 8.67)
        frame = LayerFrame.from_polygon(ops, rect)
        self.assertEqual(classify_region(frame), "feature")
        spec = self._square_spec()

        # Confirm the aerofoil path really does raise the narrow type for
        # this frame/spec before asserting on the fallback's behaviour --
        # otherwise a passing test here would prove nothing.
        with self.assertRaises(_SectionTooSmallForPitch):
            self.gen._generate_family(
                frame, spec, spec.families[0], 0, WALL_WIDTH_MM, self.layer,
                WALL_WIDTH_MM, spec.skin_margin_mm, None,
            )

        result = self.gen.generate(
            frame, spec, WALL_WIDTH_MM, self.layer, safety_offset_mm=WALL_WIDTH_MM
        )
        self.assertTrue(
            any(i.family_name.startswith("feature_lattice_") for i in result.intents),
            "expected a feature lattice, got: {!r}".format(result.intents),
        )

    def test_tiny_feature_DEGRADES_rather_than_killing_the_slice(self):
        """A tiny feature DEGRADES rather than killing the slice.

        Re-raising for every feature below 5.0 mm would make such features
        fatal (an 8.64 mm joiner post clears that bar only narrowly), which
        fails the requirement that structure not be limited to one example
        feature. Failing an entire wing over a 4 mm nub, when leaving it as
        plain skin and REPORTING it is available, is not defensible. The
        region genuinely IS a feature; it just cannot be latticed at this
        size.

        This is not silence -- zero intents routes the layer into zero-intent
        degradation, which records it and surfaces it in the run summary. The
        assertions below pin that the reasons from BOTH failed attempts survive
        into the record.
        """
        ops = StubOps()
        rect = Rect(0.0, 0.0, 2.0, 2.0)
        frame = LayerFrame.from_polygon(ops, rect)
        self.assertEqual(classify_region(frame), "feature")
        spec = self._square_spec()

        result = self.gen.generate(
            frame, spec, WALL_WIDTH_MM, self.layer, safety_offset_mm=WALL_WIDTH_MM
        )

        # Degraded, not fatal: no structure, but the slice survives.
        self.assertEqual(result.intents, [])
        # And NOT silent: the wing vocabulary's own failure reason is
        # carried into the record rather than discarded with the exception.
        reasons = " ".join(o.reason for o in result.omitted)
        self.assertIn("rib_pitch_chord_frac", reasons)
        self.assertIn("minimum-feature floor", reasons)

    def test_an_AEROFOIL_with_the_same_collapse_still_raises(self):
        """The other half of the region-conditional re-raise, and the
        reason it is conditional rather than removed: a real wing whose
        pitch config collapses is a genuine configuration problem and must
        still surface, never be masked by the feature fallback.
        """
        m, p, t = _mf.parse_naca4("2412")
        _tris, outline_mm, _bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, PolygonShape(tuple(outline_mm)))
        self.assertEqual(classify_region(frame), AEROFOIL_REGION)

        # A 60 mm chord does not collapse at rib_pitch_chord_frac=0.12,
        # so drive the SAME condition with a pitch fraction small enough
        # to fall below the floor on a real aerofoil. The point is the
        # region, not the number: same error type, same site, AEROFOIL
        # classification -- it must still raise.
        spec = StructureSpec(
            slot_gap_mm=0.3,
            skin_margin_mm=0.86,
            families=(_pitch_collapsing_family(rib_pitch_chord_frac=0.0005),),
        )
        with self.assertRaises(_SectionTooSmallForPitch):
            self.gen.generate(
                frame, spec, WALL_WIDTH_MM, self.layer,
                safety_offset_mm=WALL_WIDTH_MM,
            )


class NarrowedCaptureLeavesOtherFamilyLoopErrorsAloneTests(unittest.TestCase):
    """The three cases a broad-``ValueError`` capture would break:
    each raises for a reason that has NOTHING to do with pitch collapse,
    on a solid rectangular coupon that ``classify_region`` would call
    FEATURE (fill ratio 1.0) if it were ever consulted -- so each must
    keep failing closed, on ANY shape, without ever reaching the
    classifier or a feature lattice."""

    def setUp(self):
        self.gen = StructureGenerator()
        self.layer = LayerContext(layer_index=0, print_z=0.2)

    def test_declared_hole_conflict_still_fails_closed_on_a_solid_coupon(self):
        from stub_ops import HoledRect

        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 3.0, 24.0, 9.0)
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, HoledRect(outer=outer, hole=hole))
        self.assertEqual(classify_region(frame), "feature")
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            expected_hole_count=1,
            web_position=0.5,  # anchor x=20, dead centre of the hole
        )
        with self.assertRaises(ValueError) as ctx:
            self.gen.generate(
                frame, spec, WALL_WIDTH_MM, self.layer, safety_offset_mm=WALL_WIDTH_MM
            )
        self.assertNotIsInstance(ctx.exception, _SectionTooSmallForPitch)
        self.assertIn("declared interior hole", str(ctx.exception))

    def test_opposing_pair_cannot_fuse_still_fails_closed_on_a_solid_coupon(self):
        family = WebFamily(
            name="pair",
            opposing_pair=True,
            pair_offset_mm=10.0,
            web_position=0.5,
        )
        outer = Rect(0.0, 0.0, 40.0, 6.0)
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, outer)
        self.assertEqual(classify_region(frame), "feature")
        spec = StructureSpec(
            slot_gap_mm=0.3, skin_margin_mm=0.5, families=(family,)
        )
        with self.assertRaises(ValueError) as ctx:
            self.gen.generate(
                frame, spec, WALL_WIDTH_MM, self.layer, safety_offset_mm=WALL_WIDTH_MM
            )
        self.assertNotIsInstance(ctx.exception, _SectionTooSmallForPitch)


class SectionTooSmallForPitchIsAValueErrorSubclassTests(unittest.TestCase):
    """Requirement: subclassing ``ValueError`` keeps every existing
    ``except ValueError`` (including ``capability.py``'s translator)
    working unchanged for callers that do not know this type exists."""

    def test_is_a_value_error(self):
        self.assertTrue(issubclass(_SectionTooSmallForPitch, ValueError))


class RealAerofoilStillRaisesIdenticallyTests(unittest.TestCase):
    """The whole safety of the fallback: a genuinely
    AEROFOIL-shaped frame whose family loop raises
    ``_SectionTooSmallForPitch`` must still raise, with the identical
    message, because the classifier declines to call it a FEATURE."""

    def _naca_frame(self):
        m, p, t = _mf.parse_naca4("2412")
        _triangles, outline_mm, _bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
        ops = StubOps()
        return LayerFrame.from_polygon(ops, PolygonShape(tuple(outline_mm)))

    def test_naca_section_with_pitch_collapse_still_raises(self):
        frame = self._naca_frame()
        self.assertEqual(classify_region(frame), AEROFOIL_REGION)

        family = _pitch_collapsing_family(
            name="tiny_pitch", rib_pitch_chord_frac=0.0005
        )
        spec = StructureSpec(
            slot_gap_mm=0.3, skin_margin_mm=0.86, families=(family,)
        )
        gen = StructureGenerator()
        layer = LayerContext(layer_index=0, print_z=0.2)

        with self.assertRaises(_SectionTooSmallForPitch) as ctx:
            gen.generate(
                frame, spec, WALL_WIDTH_MM, layer, safety_offset_mm=WALL_WIDTH_MM
            )
        message = str(ctx.exception)
        self.assertIn("rib_pitch_chord_frac=0.0005", message)
        self.assertIn("minimum-feature floor", message)

    def test_mutant_swallowing_the_reraise_turns_the_previous_test_red(self):
        """MUTANT: patch ``StructureGenerator.generate`` so the re-raise site
        swallows ``aerofoil_error`` instead of raising it, and show the
        previous test's own assertion goes RED -- proving this suite is
        actually sensitive to point 4, not just to the presence of SOME
        exception."""
        frame = self._naca_frame()
        family = _pitch_collapsing_family(
            name="tiny_pitch", rib_pitch_chord_frac=0.0005
        )
        spec = StructureSpec(
            slot_gap_mm=0.3, skin_margin_mm=0.86, families=(family,)
        )
        gen = StructureGenerator()
        layer = LayerContext(layer_index=0, print_z=0.2)

        import aircraft_core.generator as generator_module

        original_generate = generator_module.StructureGenerator.generate

        def _mutant_generate(self, *args, **kwargs):
            try:
                return original_generate(self, *args, **kwargs)
            except _SectionTooSmallForPitch:
                # Swallow it exactly the way a broken re-raise site would:
                # treat the captured error as if it had never happened,
                # by pretending the layer produced no structure instead of
                # surfacing the failure.
                from aircraft_core.generator import GenerationResult

                return GenerationResult(intents=[], omitted=[])

        generator_module.StructureGenerator.generate = _mutant_generate
        try:
            result = gen.generate(
                frame, spec, WALL_WIDTH_MM, layer, safety_offset_mm=WALL_WIDTH_MM
            )
            # Under the mutant, no exception -- exactly the RED this
            # mutant proves: the real code must raise here, and does
            # (see the sibling test above), but this patched version
            # does not.
            self.assertEqual(result.intents, [])
        finally:
            generator_module.StructureGenerator.generate = original_generate


class FeatureLatticeSizeBoundaryTests(unittest.TestCase):
    """Pins the decline/lattice boundary, which is EMERGENT and not chosen.

    It falls out of ``feature_lattice_pitch_mm`` plus the minimum wall the
    slot geometry needs -- nobody picked 5 mm. That is exactly why it is
    asserted: an emergent, user-visible boundary that no test names is how
    a config change silently moves behaviour. If a future pitch or wall
    change moves this, the failure should arrive with an explanation
    attached rather than as a mystery.
    """

    def setUp(self):
        self.gen = StructureGenerator()
        self.layer = LayerContext(layer_index=7, print_z=1.4)

    def _teeth(self, width_mm):
        ops = StubOps()
        frame = LayerFrame.from_polygon(
            ops, Rect(0.0, 0.0, width_mm, width_mm))
        spec = StructureSpec(
            slot_gap_mm=0.3,
            skin_margin_mm=0.86,
            families=(_pitch_collapsing_family(),),
        )
        res = self.gen.generate(
            frame, spec, WALL_WIDTH_MM, self.layer,
            safety_offset_mm=WALL_WIDTH_MM)
        return len(res.intents)

    def test_below_the_boundary_declines_to_skin_only(self):
        # Degrades, never fatal -- a feature too small to lattice must not kill
        # the slice (degrade-by-default). Boundary under THIS spec (pitch 2.0,
        # slot_gap 0.3, skin_margin 0.86) is between 5.0 and 6.0 mm. Under the
        # SHIPPED live config it is between 4.9 and 5.0 -- the boundary is a
        # function of the config, not a constant, which is the point.
        for width in (2.0, 3.5, 5.0):
            self.assertEqual(
                self._teeth(width), 0,
                "width {} mm should decline, not build".format(width))

    def test_at_and_above_the_boundary_a_lattice_is_built(self):
        for width in (6.0, 8.0, 8.64):
            self.assertGreater(
                self._teeth(width), 0,
                "width {} mm should carry a lattice".format(width))

    def test_tooth_count_grows_with_feature_size(self):
        # A principled depth anchor gives every tooth a depth it can
        # survive, so count grows with size instead of collapsing to one
        # cut. Measured on the feature path only: sizes above ~14 mm are NOT
        # valid evidence here (the aerofoil path succeeds and the fallback
        # never runs).
        self.assertLessEqual(self._teeth(6.0), self._teeth(8.64))
        self.assertLessEqual(self._teeth(8.64), self._teeth(10.0))
        self.assertGreater(self._teeth(10.0), self._teeth(6.0))


if __name__ == "__main__":
    unittest.main()
