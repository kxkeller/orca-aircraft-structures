"""Does the generate->merge->validate chain work at
``outer_wall_line_width=0.60`` mm (arm C)?

``safety_offset_mm`` and the minimum-wall/merge-band arithmetic are all
derived from the resolved outer wall line width, so raising it from 0.42 to
0.60 mm rescales ``min_wall_mm = 2 * safety_offset_mm`` from 0.84 mm to
1.20 mm everywhere that constant is used -- the merge band AND
``TopologyValidator``'s own ``skin_margin_mm > 2 * outer_wall_line_width``
precondition (``validate.py``'s ``TopologyValidator`` class docstring;
``spec.py``'s ``skin_margin_mm`` docstring states the same precondition).

THE FINDING (see each test's own docstring for the mechanism): the merge band
itself scales exactly as documented -- the underlying arithmetic is correct at
0.60 mm, not merely at 0.42 mm. But arm C pairs ``outer_wall_line_width=0.60``
with ``skin_margin_mm=1.0`` (copied unchanged from arms A/B, which use
0.42 mm), and ``1.0 < 2 * 0.60 = 1.20``, so arm C's config violates its own
precondition. This is not a generator bug: it is the cost of the skin
thickness lever that must be flagged rather than resolved by the generator.

``StructureGenerator.generate()`` calls ``spec.validate(safety_offset_mm=...)``
BEFORE placing any web, so a ``skin_margin_mm`` that does not exceed
``2 * safety_offset_mm`` raises ``aircraft_core.spec.SpecError`` immediately
from ``generate()`` itself, never reaching ``SlotBuilder``, the boolean diff,
or ``TopologyValidator``. The
``ArmCPreconditionBreaksOnThe*Tests`` ``test_arm_c_wall_width_with_arm_c_margin_fails_closed``
methods assert that failure. The two control tests (baseline-clean,
corrected-margin-clean) do not violate the precondition and exercise the full
chain.
"""
import unittest

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator, WebIntent
from aircraft_core.slot import SlotBuilder
from aircraft_core.spec import SpecError, StructureSpec, WebFamily
from aircraft_core.validate import TopologyError, TopologyValidator
from stub_ops import NotchedPolygon, PolygonShape, Rect, StubOps

import importlib.util
from pathlib import Path

# ``_merge_cross_family_nodes``/``_merge_node_component`` REQUIRE a
# ``LayerContext`` -- the merge-halo breach raises a ``TopologyError`` that
# must name a real layer index and Z, and an optional ctx defaulting to
# ``-1``/``0.0`` could emit a diagnostic naming the wrong layer. These direct-call tests do not exercise that raise, so any
# concrete context serves; it is named rather than inlined so it is obvious
# these values are inert here.
_MERGE_TEST_CTX = LayerContext(layer_index=0, print_z=0.0)


REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_thick_skin_path", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_mf = _load_make_fixtures_module()


def _naca2412_polygon():
    """Same 60 mm chord NACA 2412 section every other test file in this
    directory builds from ``tools/make_fixtures.py`` -- no second airfoil
    implementation here."""
    m, p, t = _mf.parse_naca4("2412")
    _triangles, outline_mm, bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm)), bbox


def _rect_from_points(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return Rect(min(xs), min(ys), max(xs), max(ys))


ARM_C_LINE_WIDTH_MM = 0.60  # arm C
BASELINE_LINE_WIDTH_MM = 0.42  # arms A/B, and every other fixture's default
ARM_C_SKIN_MARGIN_MM = 1.0  # copied unchanged from arms A/B in the pre-registered config


# ---------------------------------------------------------------------------
# 1. The merge band itself scales with wall width exactly as documented.
# ---------------------------------------------------------------------------


class MergeBandScalesWithWallWidthTests(unittest.TestCase):
    """``min_wall_mm = 2 * safety_offset_mm`` — pure arithmetic, no geometry
    backend needed. A fixed 1.0 mm gap between two cross-family footprints must
    NOT merge at the baseline 0.42 mm wall (``min_wall_mm=0.84``, and 0.84 <
    1.0) and MUST merge once the wall is raised to 0.60 mm
    (``min_wall_mm=1.20``, and 1.20 > 1.0) — the identical physical gap, judged
    only by the wall-width-derived band. This is the merge band's own scaling
    claim (``aircraft_core/generator.py``'s ``_merge_cross_family_nodes``
    docstring: "the same ``_min_wall_mm`` ``TopologyValidator`` already
    enforces, called ``min_wall_mm`` here too"), pinned directly against 0.60
    mm rather than merely asserted.
    """

    def _two_intents_with_gap(self, gap_mm):
        # Two 0.021 mm-wide footprints (near-zero gap width, the same value and
        # ``test_node_merge_scope.py`` use), separated by exactly ``gap_mm``
        # between their near edges.
        width_mm = 0.021
        fwd_x = 0.0
        aft_x = fwd_x + width_mm / 2.0 + gap_mm + width_mm / 2.0
        fwd = WebIntent(
            start=(fwd_x, 3.0),
            end=(fwd_x, -2.0),
            width_mm=width_mm,
            family_index=0,
            family_name="fwd",
        )
        aft = WebIntent(
            start=(aft_x, 3.0),
            end=(aft_x, -2.0),
            width_mm=width_mm,
            family_index=1,
            family_name="aft",
        )
        return fwd, aft

    def test_gap_stays_separate_at_baseline_wall_width(self):
        fwd, aft = self._two_intents_with_gap(1.0)
        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [fwd, aft], safety_offset_mm=BASELINE_LINE_WIDTH_MM, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(len(kept), 2, "1.0 mm gap must clear the 0.84 mm band")
        self.assertEqual(omitted, [])

    def test_same_gap_merges_at_arm_c_wall_width(self):
        fwd, aft = self._two_intents_with_gap(1.0)
        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [fwd, aft], safety_offset_mm=ARM_C_LINE_WIDTH_MM, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(
            len(kept),
            1,
            "the identical 1.0 mm gap must merge once min_wall_mm reaches "
            "1.20 mm at the 0.60 mm arm C wall width",
        )
        self.assertEqual(kept[0].merged_family_indices, (0, 1))
        self.assertEqual(omitted, [])

    def test_band_edge_is_exactly_twice_the_wall_width(self):
        # A gap just inside 2*0.60=1.20 mm merges; a gap just outside does
        # not -- pins the band edge itself, not just two sample points
        # either side of it.
        gen = StructureGenerator()
        fwd, aft = self._two_intents_with_gap(1.199)
        kept, _ = gen._merge_cross_family_nodes(
            [fwd, aft], safety_offset_mm=ARM_C_LINE_WIDTH_MM, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(len(kept), 1)

        fwd, aft = self._two_intents_with_gap(1.201)
        kept, _ = gen._merge_cross_family_nodes(
            [fwd, aft], safety_offset_mm=ARM_C_LINE_WIDTH_MM, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(len(kept), 2)


# ---------------------------------------------------------------------------
# 2. Arm C's pre-registered config, as written, fails closed at 0.60 mm --
#    the real finding.
# ---------------------------------------------------------------------------


class ArmCPreconditionBreaksOnTheCouponTests(unittest.TestCase):
    """The full generate -> build -> diff -> validate chain, at
    ``outer_wall_line_width=0.60`` mm and arm C's own
    ``skin_margin_mm=1.0`` (unchanged from the 0.42 mm arms), on the
    rectangular coupon.

    Not a generator bug: ``StructureSpec``'s own documented precondition
    (``skin_margin_mm > 2 * outer_wall_line_width`` — see its class docstring)
    is violated by this exact config (``1.0 < 1.20``).

    This precondition is enforced by
    ``StructureSpec.validate(safety_offset_mm=...)``, wired into
    ``StructureGenerator.generate()`` -- so a violating config fails
    BEFORE any full-depth web is even placed. The chain this class's name
    promises (generate -> build -> diff
    -> validate) is therefore only exercised by the two CONTROL tests below;
    the failing test stops at generate().
    """

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)
        self.gen = StructureGenerator()
        self.builder = SlotBuilder()

    def _run_one_web(self, wall_width_mm, skin_margin_mm, gap_ratio=0.5):
        spec = StructureSpec(gap_ratio=gap_ratio, skin_margin_mm=skin_margin_mm)
        layer = LayerContext(layer_index=5, print_z=1.20)
        result = self.gen.generate(
            self.frame, spec, wall_width_mm, layer, safety_offset_mm=wall_width_mm
        )
        self.assertEqual(len(result.intents), 1, "expected exactly one web")
        built = self.builder.build(result.intents[0], spec.overshoot_mm)
        slot = _rect_from_points(built.points)
        transformed = self.ops.diff(self.rect, [slot])
        self.assertEqual(len(transformed), 1)

        validator = TopologyValidator(self.ops, safety_offset_mm=wall_width_mm)
        validator.validate(
            [self.rect],
            transformed,
            [slot],
            spec,
            layer.layer_index,
            layer.print_z,
        )

    def test_baseline_wall_width_and_margin_slices_clean(self):
        # Control: the SAME chain, at the baseline 0.42 mm wall and the same
        # 1.0 mm margin (1.0 > 0.84), passes -- confirming the chain itself
        # is sound and the failure below is specifically about the
        # wall-width/margin pairing, not this test's own construction.
        self._run_one_web(BASELINE_LINE_WIDTH_MM, ARM_C_SKIN_MARGIN_MM)

    def test_arm_c_wall_width_with_arm_c_margin_fails_closed(self):
        # The guard fires from StructureGenerator.generate() itself, BEFORE
        # any web is placed: a SpecError, not an erosion-check TopologyError
        # several steps later. The finding is that arm C's pre-registered
        # config violates its own precondition.
        with self.assertRaises(SpecError) as ctx:
            self._run_one_web(ARM_C_LINE_WIDTH_MM, ARM_C_SKIN_MARGIN_MM)
        message = str(ctx.exception)
        self.assertIn("skin_margin_mm=1.0", message)
        self.assertIn("2 * safety_offset_mm=0.6", message)

    def test_arm_c_wall_width_with_corrected_margin_slices_clean(self):
        # Proves the underlying 0.60 mm line-width MECHANISM works once the
        # precondition is honoured -- this is not "0.60 mm is broken", it is
        # "arm C's pre-registered skin_margin_mm needs raising to at least
        # 2 * 0.60 = 1.20 mm before it can run". 1.3 mm chosen with a small
        # margin above the exact threshold, not tuned to any other number.
        self._run_one_web(ARM_C_LINE_WIDTH_MM, skin_margin_mm=1.3)


class ArmCPreconditionBreaksOnTheAirfoilTests(unittest.TestCase):
    """Same finding, on the real NACA 2412 section -- arm C's actual target
    fixture (: "the airfoil fixture... enclosed area and curvature are the
    point"), not just the rectangular coupon.
    """

    def setUp(self):
        self.ops = StubOps()
        self.polygon, _bbox = _naca2412_polygon()
        self.frame = LayerFrame.from_polygon(self.ops, self.polygon)
        self.gen = StructureGenerator()
        self.builder = SlotBuilder()

    def _run_one_web(self, wall_width_mm, skin_margin_mm, web_position=0.4):
        # web_position=0.4 mirrors arm C's own "fwd" family anchor.
        spec = StructureSpec(
            gap_ratio=0.05, skin_margin_mm=skin_margin_mm, web_position=web_position
        )
        layer = LayerContext(layer_index=0, print_z=0.20)
        result = self.gen.generate(
            self.frame, spec, wall_width_mm, layer, safety_offset_mm=wall_width_mm
        )
        self.assertEqual(len(result.intents), 1, "expected exactly one web")
        built = self.builder.build(result.intents[0], spec.overshoot_mm)
        notch = _rect_from_points(built.points)
        transformed_list = [NotchedPolygon(outer=self.polygon, notch=notch)]

        validator = TopologyValidator(self.ops, safety_offset_mm=wall_width_mm)
        validator.validate(
            [self.polygon],
            transformed_list,
            [notch],
            spec,
            layer.layer_index,
            layer.print_z,
        )

    def test_baseline_wall_width_and_margin_slices_clean(self):
        self._run_one_web(BASELINE_LINE_WIDTH_MM, ARM_C_SKIN_MARGIN_MM)

    def test_arm_c_wall_width_with_arm_c_margin_fails_closed(self):
        # ``stub_ops.py``'s ``NotchedPolygon`` erosion path cannot complete
        # the arithmetic to raise a real ``TopologyError`` on a
        # non-rectangular outer polygon, but that does not matter here: the
        # guard fires from ``StructureGenerator.generate()`` itself, before
        # any web is placed, before ``SlotBuilder`` runs, and before
        # ``NotchedPolygon`` or ``TopologyValidator`` are reached -- so this
        # fixture gets the real, complete ``SpecError``, identically to the
        # coupon sibling test.
        with self.assertRaises(SpecError) as ctx:
            self._run_one_web(ARM_C_LINE_WIDTH_MM, ARM_C_SKIN_MARGIN_MM)
        message = str(ctx.exception)
        self.assertIn("skin_margin_mm=1.0", message)
        self.assertIn("2 * safety_offset_mm=0.6", message)

    def test_arm_c_wall_width_with_corrected_margin_slices_clean(self):
        self._run_one_web(ARM_C_LINE_WIDTH_MM, skin_margin_mm=1.3)


if __name__ == "__main__":
    unittest.main()
