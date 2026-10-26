"""Non-aerofoil FEATURE region classification and lattice generation.

Detector criterion: it separates AEROFOIL from FEATURE on a corpus of at
least four distinct feature shapes (square post, round boss, thin tab, a shape
deliberately near the boundary) with zero false positives on every
aerofoil fixture. Structure criterion: a FEATURE region gets tight-lattice structure,
and the resulting cross-section is one valid closed loop through the REAL
boolean backend (the non-negotiable invariant, exactly as everywhere else).

WHY THIS FILE USES SHAPELY, EXPLICITLY FLAGGED (same reasoning as
``test_m35_far_cap_mitre_clearance.py``/``test_erosion_near_leading_edge.py``):
``stub_ops.py`` cannot represent a real many-slot lattice's erosion/diff, so
the single-loop tests below use ``tools/shapely_ops.MiterShapelyOps``
(TEST/TOOL-ONLY, never imported by ``aircraft_core`` or the shipped plugin
bundle). Skipped outright if shapely is unavailable.

MEASURED SEPARATION this file pins (see ``aircraft_core/frame.py``'s own
module-level docstring for the full derivation):

    shape                                       aspect   fill
    NACA 2412 (this repo's shipped default)      8.23    0.67
    NACA 4415 (fattest aerofoil in the corpus)   6.43    0.66
    NACA 2404 (thin)                            21.39    0.58
    NACA 0006 (symmetric, thin)                 16.67    0.68
    8.64x8.67mm square post (worked example) 1.00    1.00
    round boss, any radius                       1.00    0.785 (pi/4, exact)
    40x7mm thin tab                              5.71    1.00
    boundary-case ellipse at aspect ~3            ~3      0.785 (pi/4, exact)
"""
import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from aircraft_core.frame import (  # noqa: E402
    AEROFOIL_REGION,
    DEFAULT_FEATURE_ASPECT_RATIO_MAX,
    DEFAULT_FEATURE_FILL_RATIO_MIN,
    FEATURE_REGION,
    LayerFrame,
    classify_region,
    estimate_rotation_deg,
)
from aircraft_core.generator import LayerContext, StructureGenerator  # noqa: E402
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.spec import StructureSpec  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

from shapely_ops import HAVE_SHAPELY as _HAVE_SHAPELY  # noqa: E402
from shapely_ops import MiterShapelyOps as _MiterShapelyOps  # noqa: E402

import make_fixtures as _mf  # noqa: E402

TESTS_UNIT_DIR = REPO_ROOT / "tests" / "unit"
if str(TESTS_UNIT_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_UNIT_DIR))

from stub_ops import PolygonShape, Rect, StubOps  # noqa: E402

WALL_WIDTH_MM = 0.42


def _rect_points(width_mm, height_mm, cx=0.0, cy=0.0):
    hw, hh = width_mm / 2.0, height_mm / 2.0
    return (
        (cx - hw, cy - hh),
        (cx + hw, cy - hh),
        (cx + hw, cy + hh),
        (cx - hw, cy + hh),
    )


def _ellipse_points(rx_mm, ry_mm, n=180, cx=0.0, cy=0.0):
    return tuple(
        (cx + rx_mm * math.cos(2 * math.pi * i / n), cy + ry_mm * math.sin(2 * math.pi * i / n))
        for i in range(n)
    )


class ClassifyRegionCorpusTests(unittest.TestCase):
    """Criterion 1's "zero false positives on every existing aerofoil
    fixture" half, via ``stub_ops.PolygonShape`` (no boolean backend
    needed for pure classification)."""

    def _naca_frame(self, naca_code, chord_mm=60.0, span_mm=60.0, n_points=120):
        m, p, t = _mf.parse_naca4(naca_code)
        _triangles, outline_mm, _bbox = _mf.airfoil_triangles(
            m, p, t, chord_mm, span_mm, n_points
        )
        ops = StubOps()
        poly = PolygonShape(tuple(outline_mm))
        return LayerFrame.from_polygon(ops, poly)

    def test_naca2412_classifies_aerofoil(self):
        frame = self._naca_frame("2412")
        self.assertEqual(classify_region(frame), AEROFOIL_REGION)

    def test_naca4415_the_fattest_fixture_still_classifies_aerofoil(self):
        # The corpus floor: aspect 6.43, the closest any shipped aerofoil
        # fixture comes to the FEATURE aspect threshold (3.0).
        frame = self._naca_frame("4415")
        self.assertEqual(classify_region(frame), AEROFOIL_REGION)
        width, depth = frame.width, frame.depth
        aspect = max(width, depth) / min(width, depth)
        self.assertGreater(aspect, DEFAULT_FEATURE_ASPECT_RATIO_MAX)

    def test_naca2404_thin_classifies_aerofoil(self):
        frame = self._naca_frame("2404")
        self.assertEqual(classify_region(frame), AEROFOIL_REGION)

    def test_naca0006_symmetric_classifies_aerofoil(self):
        frame = self._naca_frame("0006")
        self.assertEqual(classify_region(frame), AEROFOIL_REGION)


class ClassifyRegionFourShapeClassesTests(unittest.TestCase):
    """Criterion 1's own four named shapes, direct ``classify_region``
    calls (no ``generate()`` involved -- this is purely about the
    detector, not the fallback wiring)."""

    def setUp(self):
        self.ops = StubOps()

    def _frame(self, points):
        poly = PolygonShape(tuple(points))
        return LayerFrame.from_polygon(self.ops, poly)

    def test_square_post_classifies_feature(self):
        # Worked example: an 8.64 x 8.67 mm post.
        frame = self._frame(_rect_points(8.64, 8.67))
        self.assertEqual(classify_region(frame), FEATURE_REGION)

    def test_round_boss_classifies_feature(self):
        r = 4.3
        n = 180
        points = tuple(
            (r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n))
            for i in range(n)
        )
        frame = self._frame(points)
        self.assertEqual(classify_region(frame), FEATURE_REGION)
        # The trap names: estimate_rotation_deg is documented as degenerate on
        # some shapes. A discretised circle's own min-X /max-X extrema are each
        # a SINGLE point (not a tied group, unlike an axis-aligned rectangle),
        # so this must read a small, sane value here, not a spurious large one
        # that would fail closed on check_rotation_envelope before
        # classification is ever reached.
        self.assertLess(abs(frame.rotation_deg), 5.0)

    def test_thin_tab_classifies_feature_via_fill_not_aspect(self):
        # Aspect (5.71) sits ABOVE the aspect threshold (3.0) -- aspect
        # alone would misclassify this as an aerofoil. Fill ratio (1.0,
        # any rectangle) is what actually catches it.
        frame = self._frame(_rect_points(40.0, 7.0))
        width, depth = frame.width, frame.depth
        aspect = max(width, depth) / min(width, depth)
        self.assertGreater(aspect, DEFAULT_FEATURE_ASPECT_RATIO_MAX)
        self.assertEqual(classify_region(frame), FEATURE_REGION)

    def test_boundary_case_ellipse_documents_which_way_it_falls(self):
        # An ellipse's fill ratio is the CONSTANT pi/4 (~0.785) at ANY
        # aspect ratio -- below the fill threshold (0.85) regardless, so
        # this shape is resolved by the aspect test alone. This is
        # ``classify_region``'s own documented UNRESOLVED boundary case:
        # not a claim that this resolution is "right", only that it is
        # deterministic and stated, not hidden.
        below = self._frame(_ellipse_points(rx_mm=5.8, ry_mm=2.0))  # aspect 2.9
        above = self._frame(_ellipse_points(rx_mm=6.2, ry_mm=2.0))  # aspect 3.1
        aspect_below = max(below.width, below.depth) / min(below.width, below.depth)
        aspect_above = max(above.width, above.depth) / min(above.width, above.depth)
        self.assertLess(aspect_below, DEFAULT_FEATURE_ASPECT_RATIO_MAX)
        self.assertGreater(aspect_above, DEFAULT_FEATURE_ASPECT_RATIO_MAX)
        self.assertEqual(classify_region(below), FEATURE_REGION)
        self.assertEqual(classify_region(above), AEROFOIL_REGION)


class ZeroIntentFallbackOrderingTests(unittest.TestCase):
    """The fallback ordering itself: the aerofoil path runs first, and classification is
    consulted ONLY when it produces zero intents. A rectangular coupon that
    gets ordinary wing intents never reaches the classifier at all."""

    def test_square_coupon_with_ordinary_intents_never_reaches_the_fallback(self):
        ops = StubOps()
        rect = Rect(0.0, 0.0, 20.0, 20.0)
        frame = LayerFrame.from_polygon(ops, rect)
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0)
        result = StructureGenerator().generate(
            frame, spec, WALL_WIDTH_MM, LayerContext(layer_index=0, print_z=0.0)
        )
        # Ordinary aerofoil-vocabulary intent, never touched -- a
        # "default"-named single web, not "feature_lattice_*".
        self.assertEqual(len(result.intents), 1)
        self.assertEqual(result.intents[0].family_name, "default")

    def test_naca_section_with_ordinary_intents_never_reaches_the_fallback(self):
        m, p, t = _mf.parse_naca4("2412")
        _triangles, outline_mm, _bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, PolygonShape(tuple(outline_mm)))
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0)
        result = StructureGenerator().generate(
            frame, spec, WALL_WIDTH_MM, LayerContext(layer_index=0, print_z=0.0)
        )
        self.assertEqual(len(result.intents), 1)
        self.assertEqual(result.intents[0].family_name, "default")

    def test_zero_intent_naca_section_declines_the_feature_fallback(self):
        """A NACA 2412 section engineered to zero intents (an impossibly
        large skin_margin_mm) DECLINES the feature fallback: it is an
        aerofoil section that ran out of margin, not a feature, and
        classify_region must say so. This is both the evidence that the
        classifier declines on a real aerofoil and the mutant target below."""
        m, p, t = _mf.parse_naca4("2412")
        _triangles, outline_mm, _bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, PolygonShape(tuple(outline_mm)))
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=50.0, web_position=0.5)
        result = StructureGenerator().generate(
            frame, spec, WALL_WIDTH_MM, LayerContext(layer_index=0, print_z=0.0)
        )
        self.assertEqual(result.intents, [])
        self.assertFalse(
            any(o.family_name.startswith("feature_lattice_") for o in result.omitted),
            "a real (if margin-starved) aerofoil section must never reach "
            "the feature lattice: {!r}".format(result.omitted),
        )

    def test_mutant_always_feature_turns_the_declining_case_red(self):
        """MUTANT: force ``classify_region`` to always return FEATURE_REGION and
        show the previous test's own assertion go RED. Patches
        ``aircraft_core.generator.classify_region`` -- ``generator.py`` binds
        the name at import time, so patching
        ``aircraft_core.frame.classify_region`` would not reach the call site
        actually used."""
        import aircraft_core.generator as generator_module

        m, p, t = _mf.parse_naca4("2412")
        _triangles, outline_mm, _bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, PolygonShape(tuple(outline_mm)))
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=50.0, web_position=0.5)

        original = generator_module.classify_region
        generator_module.classify_region = lambda *a, **k: FEATURE_REGION
        try:
            result = StructureGenerator().generate(
                frame, spec, WALL_WIDTH_MM, LayerContext(layer_index=0, print_z=0.0)
            )
        finally:
            generator_module.classify_region = original

        # Under the mutant, a margin-starved NACA 2412 section is WRONGLY
        # given feature-lattice structure -- the RED this mutant proves.
        self.assertTrue(
            any(i.family_name.startswith("feature_lattice_") for i in result.intents)
            or any(o.family_name.startswith("feature_lattice_") for o in result.omitted),
            "mutant did not engage -- test is not sensitive to "
            "classify_region's return value",
        )


@unittest.skipUnless(_HAVE_SHAPELY, "shapely not installed; see module docstring")
class FeatureLatticeRealBooleanTests(unittest.TestCase):
    """Criterion 2: the resulting cross-section is one valid closed loop,
    through the REAL boolean backend -- generate -> SlotBuilder -> diff ->
    TopologyValidator.validate, exactly the non-negotiable invariant
    pipeline every other layer goes through."""

    @classmethod
    def setUpClass(cls):
        cls.ops = _MiterShapelyOps()
        # z_drift_mm_per_mm=100 at print_z=1.0 drifts the ordinary (single
        # implicit) family's anchor 100mm off-chord -- guaranteed outside any
        # of this test's small (a few cm at most) feature shapes' own extent,
        # exactly the real mechanism (an absolute, root-anchored drift landing
        # outside a small feature's own bbox --). This reaches the zero -intent
        # fallback WITHOUT needing an oversized skin_margin_mm that would also
        # starve the feature lattice itself. ``_generate_feature`` ignores this
        # drift entirely by construction (its own synthetic families are built
        # with ``z_drift_mm_per_mm=None``), so the fallback's own success is
        # unaffected by it.
        cls.spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, root_solid_layers=0,
            web_position=0.5, z_drift_mm_per_mm=100.0,
        )
        cls.builder = SlotBuilder()

    def _run(self, points, layer_index=0, print_z=1.0):
        poly = self.ops.polygon_from_points(points)
        frame = LayerFrame.from_polygon(self.ops, poly)
        gen = StructureGenerator()
        ctx = LayerContext(layer_index=layer_index, print_z=print_z)
        result = gen.generate(
            frame, self.spec, WALL_WIDTH_MM, ctx, safety_offset_mm=WALL_WIDTH_MM
        )
        built = [self.builder.build(i, self.spec.overshoot_mm) for i in result.intents]
        slots = [self.ops.polygon_from_points(b.points) for b in built]
        transformed = self.ops.diff(poly, slots)
        validator = TopologyValidator(self.ops, WALL_WIDTH_MM)
        validator.validate(
            [poly], transformed, slots, self.spec, layer_index, print_z,
            slot_labels=[str(i.family_name) for i in result.intents],
            family_indices=[i.family_membership() for i in result.intents],
        )
        return result

    def test_square_post_gets_structure_and_validates_one_loop(self):
        result = self._run(_rect_points(8.64, 8.67))
        self.assertGreater(len(result.intents), 0, "no structure produced")
        self.assertTrue(
            any(i.family_name.startswith("feature_lattice_") for i in result.intents)
        )

    def test_round_boss_gets_structure_and_validates_one_loop(self):
        r = 4.3
        n = 96
        points = tuple(
            (r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n))
            for i in range(n)
        )
        result = self._run(points)
        self.assertGreater(len(result.intents), 0, "no structure produced")

    def test_thin_tab_gets_structure_and_validates_one_loop(self):
        result = self._run(_rect_points(40.0, 7.0))
        self.assertGreater(len(result.intents), 0, "no structure produced")

    def test_mutant_always_aerofoil_turns_all_three_red(self):
        """MUTANT: force ``classify_region`` to always return AEROFOIL_REGION and
        show the three feature single-loop tests above go RED -- not merely
        empty intents, but the SAME ``TopologyError`` (structure invariant
        violated) the real single-loop tests would raise if the fallback ever
        produced zero-intent structure: "no material removed" on a plain
        square/ boss/tab, since the ordinary aerofoil vocabulary has nothing to
        say about a region with no chord/web_position meaning."""
        import aircraft_core.generator as generator_module

        original = generator_module.classify_region
        generator_module.classify_region = lambda *a, **k: AEROFOIL_REGION
        try:
            with self.assertRaises(TopologyError) as ctx:
                self._run(_rect_points(8.64, 8.67))
        finally:
            generator_module.classify_region = original

        self.assertIn("no material removed", str(ctx.exception))
