"""Scar placement. Exercises the feature code that the proof extension
(``docs/geometry_invariants.md``) licenses:

- ``StructureSpec.expected_hole_count`` — the declared-hole licence.
- ``StructureGenerator._generate_scar`` — where the scar is placed, and
  every fail-closed guard around it.
- ``StructureGenerator.generate()`` wiring: default (0) byte-identical,
  marching-family hole avoidance, explicit-station hole collision.
- ``WebIntent.is_scar`` and its two exclusions:
  ``_merge_cross_family_nodes`` (never merged) and
  ``_omit_opposing_conflicts`` (never the loser).
- ``TopologyValidator``'s scar-aware invariant-4 branch, and the two
  existing checks (same-edge collision, invariant 2 unchanged) proven
  to need NO scar-specific code.

Distinct from ``test_spar_hole.py``, which is the proof/design fixture file and
is left as-is (see its own updated module docstring).
"""

import unittest

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import (
    LayerContext,
    StructureGenerator,
    WebIntent,
)
from aircraft_core.spec import SpecError, StructureSpec
from aircraft_core.validate import TopologyError, TopologyValidator
from stub_ops import HoledRect, MultiNotchedRect, Rect, ScarredHoledRect, StubOps

# the merge helpers REQUIRE a LayerContext (see generator.py's merge-halo
# raise). These direct-call tests never reach that raise, so any concrete
# context serves; named rather than inlined to make that inertness obvious.
_MERGE_TEST_CTX = LayerContext(layer_index=0, print_z=0.0)


LAYER_INDEX = 9
PRINT_Z = 4.50
SAFETY_OFFSET_MM = 0.42
MIN_WALL_MM = 2.0 * SAFETY_OFFSET_MM  # 0.84
# 0.0 keeps this direct call testing pure scar-never-loses mechanics against
# the hand-picked depths below -- reach_depth_mm(0.0) is byte-identical to
# depth_mm().
OVERSHOOT_MM = 0.0


# ---------------------------------------------------------------------------
# StructureSpec.expected_hole_count
# ---------------------------------------------------------------------------


class ExpectedHoleCountSpecTests(unittest.TestCase):
    def test_default_is_zero(self):
        spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)
        self.assertEqual(spec.expected_hole_count, 0)
        spec.validate()  # must not raise -- shipped default unaffected

    def test_negative_rejected(self):
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, expected_hole_count=-1
        )
        with self.assertRaises(SpecError):
            spec.validate()

    def test_more_than_one_rejected(self):
        # The proof (section 3c) is scoped to exactly one hole per layer -- N>1
        # is an unproven extension and must not be silently accepted.
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, expected_hole_count=2
        )
        with self.assertRaises(SpecError) as ctx:
            spec.validate()
        self.assertIn("1", str(ctx.exception))

    def test_one_is_accepted(self):
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, expected_hole_count=1
        )
        spec.validate()  # must not raise


# ---------------------------------------------------------------------------
# StructureGenerator._generate_scar — placement and its fail-closed guards
# ---------------------------------------------------------------------------


class GenerateScarTests(unittest.TestCase):
    """Direct fixtures against ``_generate_scar`` -- hole nearer -Y so the
    direction rule, wall check, overshoot placement and far-boundary guard
    are all exercised against real, non-trivial numbers."""

    def setUp(self):
        self.ops = StubOps()
        self.gen = StructureGenerator()

    def _frame(self, outer, hole=None):
        polygon = outer if hole is None else HoledRect(outer=outer, hole=hole)
        return LayerFrame.from_polygon(self.ops, polygon)

    def test_declared_hole_scarred_toward_nearer_skin(self):
        # hole nearer -Y (dist_lower=3, dist_upper=11) -> scar enters -Y.
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 3.0, 24.0, 9.0)
        frame = self._frame(outer, hole)
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, expected_hole_count=1
        )
        intent = self.gen._generate_scar(frame, spec, 0.42, 1.0)
        self.assertTrue(intent.is_scar)
        self.assertTrue(intent.entry_lower)
        self.assertEqual(intent.width_mm, 0.6)
        self.assertAlmostEqual(intent.start[0], 20.0)
        self.assertAlmostEqual(intent.start[1], 0.0)  # true -Y skin
        self.assertAlmostEqual(intent.end[1], 3.05)  # hole near (3.0) + overshoot

    def test_tie_breaks_toward_plus_y(self):
        # hole vertically centred -> dist_upper == dist_lower exactly.
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 7.0, 24.0, 13.0)
        frame = self._frame(outer, hole)
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, expected_hole_count=1
        )
        intent = self.gen._generate_scar(frame, spec, 0.42, 1.0)
        self.assertFalse(intent.entry_lower)  # tie -> +Y
        self.assertAlmostEqual(intent.start[1], 20.0)
        self.assertAlmostEqual(intent.end[1], 13.0 - 0.05)

    def test_declared_count_mismatch_fails_closed(self):
        outer = Rect(0.0, 0.0, 40.0, 20.0)  # no hole at all
        frame = self._frame(outer)
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, expected_hole_count=1
        )
        with self.assertRaises(ValueError) as ctx:
            self.gen._generate_scar(frame, spec, 0.42, 1.0)
        self.assertIn("mismatch", str(ctx.exception))

    def test_hole_too_near_skin_fails_closed_naming_the_wall(self):
        # dist_lower = 0.5, below skin_margin_mm=1.0 -- no omission path
        # exists for a declared hole (section 2d).
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 0.5, 24.0, 6.5)
        frame = self._frame(outer, hole)
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, expected_hole_count=1
        )
        with self.assertRaises(ValueError) as ctx:
            self.gen._generate_scar(frame, spec, 0.42, 1.0)
        message = str(ctx.exception)
        self.assertIn("wall", message)
        self.assertIn("0.5", message)

    def test_hole_too_short_for_overshoot_fails_closed(self):
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 6.0, 24.0, 6.02)  # height 0.02mm < overshoot 0.05mm
        frame = self._frame(outer, hole)
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, expected_hole_count=1
        )
        with self.assertRaises(ValueError) as ctx:
            self.gen._generate_scar(frame, spec, 0.42, 1.0)
        self.assertIn("overshoot_mm", str(ctx.exception))

    def test_scar_too_wide_relative_to_hole_fails_closed(self):
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(19.75, 3.0, 20.25, 9.0)  # width 0.5mm < scar width 0.6mm
        frame = self._frame(outer, hole)
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, expected_hole_count=1
        )
        with self.assertRaises(ValueError) as ctx:
            self.gen._generate_scar(frame, spec, 0.42, 1.0)
        self.assertIn("narrower", str(ctx.exception))

    def test_scar_footprint_outside_chord_extent_fails_closed(self):
        outer = Rect(0.0, 0.0, 18.0, 20.0)
        hole = Rect(14.0, 3.0, 17.5, 9.0)
        frame = self._frame(outer, hole)
        spec = StructureSpec(
            slot_gap_mm=5.0, skin_margin_mm=1.0, expected_hole_count=1
        )
        with self.assertRaises(ValueError) as ctx:
            self.gen._generate_scar(frame, spec, 0.42, 1.0)
        self.assertIn("chord extent", str(ctx.exception))


# ---------------------------------------------------------------------------
# StructureGenerator.generate() wiring
# ---------------------------------------------------------------------------


class GenerateWiringTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.gen = StructureGenerator()
        self.layer = LayerContext(layer_index=LAYER_INDEX, print_z=PRINT_Z)

    def test_default_spec_never_calls_hole_bounds(self):
        # Hole-free layer, shipped default (expected_hole_count=0): must be
        # byte-identical to a layer with no scar machinery -- zero ops.hole_bounds calls,
        # zero scar intents.
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        frame = LayerFrame.from_polygon(self.ops, outer)
        spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)
        self.assertEqual(spec.expected_hole_count, 0)
        result = self.gen.generate(
            frame, spec, 0.42, self.layer, safety_offset_mm=0.42
        )
        self.assertEqual(len(result.intents), 1)
        self.assertFalse(result.intents[0].is_scar)
        self.assertNotIn(
            "hole_bounds", [name for name, _args in self.ops.calls]
        )

    def test_declared_hole_produces_a_scar_alongside_the_ordinary_web(self):
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 3.0, 24.0, 9.0)
        frame = LayerFrame.from_polygon(self.ops, HoledRect(outer=outer, hole=hole))
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            expected_hole_count=1,
            web_position=0.1,  # x=4, far from the hole at x=20
        )
        result = self.gen.generate(
            frame, spec, 0.42, self.layer, safety_offset_mm=0.42
        )
        scars = [i for i in result.intents if i.is_scar]
        ordinary = [i for i in result.intents if not i.is_scar]
        self.assertEqual(len(scars), 1)
        self.assertEqual(len(ordinary), 1)
        self.assertAlmostEqual(scars[0].start[0], 20.0)
        self.assertAlmostEqual(ordinary[0].start[0], 4.0)

    def test_marching_family_station_over_the_hole_is_omitted_not_raised(self):
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 3.0, 24.0, 9.0)
        frame = LayerFrame.from_polygon(self.ops, HoledRect(outer=outer, hole=hole))
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            expected_hole_count=1,
            web_position=0.1,
            rib_pitch_mm=3.0,  # marches through the hole's own X footprint
        )
        result = self.gen.generate(
            frame, spec, 0.42, self.layer, safety_offset_mm=0.42
        )
        hole_omissions = [
            o for o in result.omitted if o.reason.startswith("hole footprint:")
        ]
        self.assertTrue(hole_omissions)
        # Not a shutdown-eligible thinness reason.
        self.assertFalse(
            any(
                o.reason.startswith(("skin margin:", "thin section:"))
                for o in hole_omissions
            )
        )
        # No surviving ordinary web landed inside the haloed hole footprint.
        for i in result.intents:
            if i.is_scar:
                continue
            self.assertTrue(i.start[0] < 16.0 - 0.42 or i.start[0] > 24.0 + 0.42)

    def test_explicit_single_station_over_the_hole_raises(self):
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 3.0, 24.0, 9.0)
        frame = LayerFrame.from_polygon(self.ops, HoledRect(outer=outer, hole=hole))
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            expected_hole_count=1,
            web_position=0.5,  # anchor x=20, dead centre of the hole
        )
        with self.assertRaises(ValueError) as ctx:
            self.gen.generate(frame, spec, 0.42, self.layer, safety_offset_mm=0.42)
        self.assertIn("declared interior hole", str(ctx.exception))


# ---------------------------------------------------------------------------
# WebIntent.is_scar — merge exclusion
# ---------------------------------------------------------------------------


class ScarNeverMergedTests(unittest.TestCase):
    """Same cross-family gap band (0.4 mm, inside the merge window) tested twice:
    once with two ordinary webs (control -- SHOULD merge, proving the fixture
    is really in the merge band), once with one side marked ``is_scar`` (SHOULD
    NOT merge)."""

    def setUp(self):
        self.gen = StructureGenerator()

    def _pair(self, b_is_scar):
        a = WebIntent(
            start=(5.0, 10.0),
            end=(5.0, 2.0),
            width_mm=0.6,
            family_index=0,
            family_name="a",
        )
        b = WebIntent(
            start=(6.0, 10.0),
            end=(6.0, 2.0),
            width_mm=0.6,
            family_index=1,
            family_name="b",
            is_scar=b_is_scar,
        )
        return [a, b]

    def test_control_ordinary_cross_family_pair_merges(self):
        kept, omitted = self.gen._merge_cross_family_nodes(
            self._pair(False), SAFETY_OFFSET_MM, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(omitted, [])
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0].merged_family_indices, (0, 1))

    def test_scar_pair_never_merges(self):
        kept, omitted = self.gen._merge_cross_family_nodes(
            self._pair(True), SAFETY_OFFSET_MM, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(omitted, [])
        self.assertEqual(len(kept), 2)
        self.assertTrue(any(i.is_scar for i in kept))
        self.assertTrue(all(not i.merged_family_indices for i in kept))


# ---------------------------------------------------------------------------
# WebIntent.is_scar — opposing-entry: the scar never loses
# ---------------------------------------------------------------------------


class ScarNeverOmittedByOpposingConflictTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.gen = StructureGenerator()

    def test_scar_survives_an_opposing_conflict_regardless_of_relative_depth(self):
        # Flat 20x20 coupon: t_local=20 everywhere. Scar (+Y, depth 19) and
        # an ordinary -Y web (depth 18) fully overlap in X and meet
        # (19+18+0.84 = 37.84 >= 20) -- an ordinary "shallower loses" tie
        # -break would keep the scar anyway here (19 > 18), so also prove
        # the REVERSE depth relationship still keeps the scar.
        outer = Rect(0.0, 0.0, 20.0, 20.0)
        frame = LayerFrame.from_polygon(self.ops, outer)
        scar = WebIntent(
            start=(10.0, 20.0),
            end=(10.0, 1.0),  # depth 19 -- deeper than the web below
            width_mm=0.6,
            family_index=-1,
            family_name="spar_scar",
            entry_lower=False,
            is_scar=True,
        )
        web = WebIntent(
            start=(10.0, 0.0),
            end=(10.0, 19.5),  # depth 19.5 -- DEEPER than the scar
            width_mm=0.6,
            family_index=0,
            family_name="stiffener",
            entry_lower=True,
        )
        survivors, omitted = self.gen._omit_opposing_conflicts(
            [scar, web], frame, SAFETY_OFFSET_MM, OVERSHOOT_MM
        )
        self.assertEqual(len(survivors), 1)
        self.assertTrue(survivors[0].is_scar)
        self.assertEqual(len(omitted), 1)
        self.assertIn("scar", omitted[0].reason)


# ---------------------------------------------------------------------------
# TopologyValidator — scar-aware invariant 4, and the checks that need NO
# scar-specific code at all.
# ---------------------------------------------------------------------------


class ValidatorScarTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.validator = TopologyValidator(self.ops, safety_offset_mm=SAFETY_OFFSET_MM)

    def test_correctly_scarred_layer_validates_with_one_circuit(self):
        # Same hole as test_spar_hole.py's own keyhole fixture (tied
        # direction -> +Y), scar width 2mm, entering from +Y.
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 7.0, 24.0, 13.0)
        original = HoledRect(outer=outer, hole=hole)
        scar_slot = Rect(19.0, 13.0, 21.0, 20.0)
        transformed = ScarredHoledRect(
            outer=outer,
            hole=hole,
            scar_width_mm=2.0,
            entry_lower=False,
            scar_extra_area_mm2=2.0 * (20.0 - 13.0),
        )
        spec = StructureSpec(
            slot_gap_mm=2.0, skin_margin_mm=1.0, expected_hole_count=1
        )
        # Must not raise -- invariant 2 passes AS WRITTEN (contour_count==1
        # for the correctly bridged transformed shape), and the new
        # scar-shaped invariant-4 branch confirms the connection.
        self.validator.validate(
            [original], [transformed], [scar_slot], spec, LAYER_INDEX, PRINT_Z
        )

    def test_undeclared_hole_still_fails_invariant_two(self):
        # spec.expected_hole_count defaults to 0 -- the scar machinery is never
        # even consulted; the plain invariant-2 check applies unchanged.
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 7.0, 24.0, 13.0)
        transformed = HoledRect(outer=outer, hole=hole)
        spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [outer], [transformed], [], spec, LAYER_INDEX, PRINT_Z
            )
        self.assertIn("interior hole", str(ctx.exception))

    def test_scar_undershooting_the_hole_fails_closed(self):
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 7.0, 24.0, 13.0)
        original = HoledRect(outer=outer, hole=hole)
        # Far end at y=14: still ABOVE the hole's own near boundary
        # (13.0) -- never actually reaches the hole.
        scar_slot = Rect(19.0, 14.0, 21.0, 20.0)
        transformed = ScarredHoledRect(
            outer=outer, hole=hole, scar_width_mm=2.0, entry_lower=False,
            scar_extra_area_mm2=2.0 * (20.0 - 14.0),
        )
        spec = StructureSpec(
            slot_gap_mm=2.0, skin_margin_mm=1.0, expected_hole_count=1
        )
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [original], [transformed], [scar_slot], spec, LAYER_INDEX, PRINT_Z
            )
        self.assertIn("does not reach the declared hole", str(ctx.exception))

    def test_scar_overshooting_past_the_far_boundary_fails_closed(self):
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        hole = Rect(16.0, 7.0, 24.0, 13.0)
        original = HoledRect(outer=outer, hole=hole)
        # Far end at y=6: PAST the hole's own far boundary (7.0) -- a
        # second, unintended full cut through material beyond the hole.
        scar_slot = Rect(19.0, 6.0, 21.0, 20.0)
        transformed = ScarredHoledRect(
            outer=outer, hole=hole, scar_width_mm=2.0, entry_lower=False,
            scar_extra_area_mm2=2.0 * (20.0 - 6.0),
        )
        spec = StructureSpec(
            slot_gap_mm=2.0, skin_margin_mm=1.0, expected_hole_count=1
        )
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [original], [transformed], [scar_slot], spec, LAYER_INDEX, PRINT_Z
            )
        self.assertIn("overshoots past the declared hole", str(ctx.exception))

    def test_scar_leaving_insufficient_wall_fails_closed(self):
        outer = Rect(0.0, 0.0, 40.0, 20.0)
        # Hole 0.5mm from the +Y skin -- below the 1.0mm margin.
        hole = Rect(16.0, 7.0, 24.0, 19.5)
        original = HoledRect(outer=outer, hole=hole)
        # Far end at y=19.0 -- comfortably WITHIN the hole's own Y range
        # [7, 19.5], so the reach checks pass; only the wall (exterior
        # skin at y=20 to the hole's near boundary at y=19.5 = 0.5mm,
        # below the 1.0mm margin) fails.
        scar_slot = Rect(19.0, 19.0, 21.0, 20.0)
        transformed = ScarredHoledRect(
            outer=outer, hole=hole, scar_width_mm=2.0, entry_lower=False,
            scar_extra_area_mm2=2.0 * (20.0 - 19.0),
        )
        spec = StructureSpec(
            slot_gap_mm=2.0, skin_margin_mm=1.0, expected_hole_count=1
        )
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [original], [transformed], [scar_slot], spec, LAYER_INDEX, PRINT_Z
            )
        self.assertIn("wall", str(ctx.exception))

    def test_scar_colliding_with_a_web_is_handled_by_the_existing_M3_check(self):
        # No hole in play at all (expected_hole_count defaults to 0) -- this is
        # deliberately the UNCHANGED same-edge collision check, reused
        # verbatim: a "scar" slot and an ordinary web slot placed 0.1mm apart
        # (below the 0.84mm minimum wall) fail exactly the way any two ordinary
        # slots this close already do. Same numbers as test_validate.py's own
        # collision fixture.
        original = Rect(0.0, 0.0, 20.0, 20.0)
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)  # stands in for the scar's slot
        notch_b = Rect(5.4, 1.0, 6.0, 20.0)  # an ordinary web's slot
        transformed = MultiNotchedRect(outer=original, notches=(notch_a, notch_b))
        spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [original],
                [transformed],
                [notch_a, notch_b],
                spec,
                LAYER_INDEX,
                PRINT_Z,
                slot_labels=["scar", "stiffener"],
            )
        message = str(ctx.exception)
        self.assertIn("0.1000 mm", message)
        self.assertIn("0.84", message)


if __name__ == "__main__":
    unittest.main()
