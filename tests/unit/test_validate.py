import importlib.util
import unittest
from pathlib import Path

from aircraft_core.frame import DEFAULT_PROBE_WIDTH_MM, LayerFrame, local_span
from aircraft_core.spec import StructureSpec
from aircraft_core.validate import TopologyError, TopologyValidator
from stub_ops import (
    HoledRect,
    MultiNotchedRect,
    NotchedPolygon,
    NotchedRect,
    PolygonShape,
    Rect,
    StubOps,
)

LAYER_INDEX = 183
PRINT_Z = 36.6  # -> "Z=36.60 mm" in the message, per docs/m1_design.md.

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    # Same pattern as test_local_geometry.py: load tools/make_fixtures.py's
    # own point-generation code rather than keeping a second airfoil
    # implementation in this file.
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_validate", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_mf = _load_make_fixtures_module()


def _naca2412_polygon():
    """The fixture's exact 2D section -- same parameters as
    tests/fixtures/airfoil/naca2412_c60_s60.stl and test_local_geometry.py."""
    m, p, t = _mf.parse_naca4("2412")
    _triangles, outline_mm, bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm)), bbox


class TopologyValidatorTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.validator = TopologyValidator(self.ops, safety_offset_mm=0.42)
        self.original = Rect(0.0, 0.0, 20.0, 20.0)
        self.spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)

    def _good_notched_rect(self):
        # Slot from the top edge, 0.6 mm wide, stopping exactly at the
        # 1.0 mm margin: notch spans y in [1.0, 20.0], x in [9.7, 10.3].
        notch = Rect(9.7, 1.0, 10.3, 20.0)
        return NotchedRect(outer=self.original, notch=notch)

    def test_passes_on_good_u_shape(self):
        transformed = self._good_notched_rect()
        # Should not raise.
        self.validator.validate(
            [self.original],
            [transformed],
            [transformed.notch],
            self.spec,
            LAYER_INDEX,
            PRINT_Z,
        )

    def test_removed_region_is_measured_via_intersection_not_double_diff(self):
        # THE FIX THIS TASK CLOSES: an in-host NACA 2412 slice found
        # ``ops.diff(original, [transformed])`` reconstructing "removed" as
        # x=[-0.1050, 0.4204] mm (0.5254 mm wide, 0.3154 mm too far right)
        # where the true, requested cut -- ``ops.intersection(original,
        # slot)`` -- was exact at x=[-0.1050, 0.1050] mm (0.21 mm wide, as
        # built). See validate.py's ``TopologyValidator.validate()``
        # docstring for the full numbers.
        #
        # StubOps cannot reproduce that divergence: its ``diff()`` for a
        # ``NotchedRect``/``NotchedPolygon`` clip is DEFINED to return
        # ``[clip.notch]`` exactly (see stub_ops.py's ``_diff_one``, the
        # ``isinstance(clip, (NotchedRect, NotchedPolygon))`` branch) --
        # the double-difference and the direct intersection are
        # mathematically forced to agree on this exact-arithmetic,
        # axis-aligned-rectangle stub, unlike real Clipper-backed geometry
        # on a curved section. So this test does not (and cannot honestly)
        # assert a numeric divergence; instead it proves, via the ops call
        # log, that the validator NEVER derives ``removed`` the
        # double-difference way in the first place -- it always measures
        # the requested slot directly against the original material.
        transformed = self._good_notched_rect()
        slot = transformed.notch
        self.validator.validate(
            [self.original], [transformed], [slot], self.spec, LAYER_INDEX, PRINT_Z
        )

        diff_calls = [call for call in self.ops.calls if call[0] == "diff"]
        self.assertEqual(
            diff_calls,
            [],
            "TopologyValidator must never call ops.diff() to derive the "
            "removed region -- it must measure ops.intersection(original, "
            "slot) directly.",
        )

        intersection_calls = [
            call for call in self.ops.calls if call[0] == "intersection"
        ]
        self.assertIn(
            ("intersection", (self.original, slot)),
            intersection_calls,
            "TopologyValidator must call ops.intersection(original, slot) "
            "with the actual requested slot polygon.",
        )

    def test_two_islands_raises_and_message_format(self):
        # Requested slot spans the full depth -- a clean vertical split, the
        # clip that would have produced left/right via StubOps._diff_one's
        # "full vertical split" branch. This fails at invariant 1 (island
        # count), before slot_polygons is ever read for the removed-region
        # measurement, so its exact value is not load-bearing here.
        slot = Rect(9.7, 0.0, 10.3, 20.0)
        left = Rect(0.0, 0.0, 9.7, 20.0)
        right = Rect(10.3, 0.0, 20.0, 20.0)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original], [left, right], [slot], self.spec, LAYER_INDEX, PRINT_Z
            )
        err = ctx.exception
        self.assertEqual(err.layer_index, LAYER_INDEX)
        self.assertEqual(err.print_z, PRINT_Z)
        message = str(err)
        self.assertTrue(
            message.startswith(
                "Aircraft structure invalid at layer 183 (Z=36.60 mm):"
            ),
            message,
        )
        self.assertIn("2 material islands", message)

    def test_enclosed_hole_raises(self):
        hole = Rect(9.0, 9.0, 11.0, 11.0)  # touches no outer edge
        transformed = HoledRect(outer=self.original, hole=hole)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original], [transformed], [hole], self.spec, LAYER_INDEX, PRINT_Z
            )
        message = str(ctx.exception)
        self.assertIn("interior hole", message)
        self.assertIn("layer 183", message)
        self.assertIn("Z=36.60 mm", message)

    def test_margin_violated_raises(self):
        # Cut stops at y=0.5, less than the required 1.0 mm margin.
        notch = Rect(9.7, 0.5, 10.3, 20.0)
        transformed = NotchedRect(outer=self.original, notch=notch)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original], [transformed], [notch], self.spec, LAYER_INDEX, PRINT_Z
            )
        message = str(ctx.exception)
        self.assertIn("skin", message)
        self.assertIn("required 1.0000 mm margin", message)

    def test_web_missing_raises(self):
        # Transformed identical to original: nothing was removed, so no
        # slot was actually requested either -- fires at invariant 6's area
        # check, before slot_polygons is read.
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original], [self.original], [], self.spec, LAYER_INDEX, PRINT_Z
            )
        message = str(ctx.exception)
        self.assertIn("not present in the output", message)
        self.assertIn("layer 183", message)
        self.assertIn("Z=36.60 mm", message)

    def test_disconnect_after_safety_offset_raises(self):
        # Isolate the erosion test from the margin test. With the default
        # margin (1.0 mm) the margin check fires first for any bridge thin
        # enough to erode away — which is the precondition
        # skin_margin_mm > 2 * safety_offset in action. Use a deliberately
        # small margin so the bridge can be thinner than the erosion width
        # while still satisfying invariant 4.
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=0.2)
        # Bridge under the slot is 0.3 mm; eroding by 0.42 mm consumes it
        # (2 * 0.42 = 0.84 mm across) while the ~9 mm arms survive -> two
        # disjoint islands, even though raw polygon topology reports one.
        notch = Rect(9.0, 0.3, 11.0, 20.0)
        transformed = NotchedRect(outer=self.original, notch=notch)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original], [transformed], [notch], spec, LAYER_INDEX, PRINT_Z
            )
        message = str(ctx.exception)
        self.assertIn("disconnects material island after 0.42 mm safety offset", message)
        self.assertIn("layer 183", message)

    def test_section_thinner_than_safety_offset_reports_collapse_not_severing(self):
        # Distinct diagnosis: the whole section is thinner than the safety
        # offset, so erosion leaves nothing at all. Reporting that as a severed
        # island would misdiagnose a collapsing tip section.
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=0.05)
        thin = Rect(0.0, 0.0, 20.0, 0.3)
        notch = Rect(9.0, 0.1, 11.0, 0.3)
        transformed = NotchedRect(outer=thin, notch=notch)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [thin], [transformed], [notch], spec, LAYER_INDEX, PRINT_Z
            )
        message = str(ctx.exception)
        self.assertIn("entirely thinner than the 0.42 mm safety offset", message)
        self.assertNotIn("disconnects material island", message)


    def test_degenerate_bounding_box_is_refused(self):
        flat = Rect(0.0, 5.0, 20.0, 5.0)  # zero height -> zero-area bbox
        # Fires on original_polygons alone, before slot_polygons is read.
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate([flat], [flat], [], self.spec, LAYER_INDEX, PRINT_Z)
        self.assertIn("degenerate bounding box", str(ctx.exception))

    def test_invalid_polygon_raises(self):
        # Zero-width notch degenerate shape reported invalid by is_valid()
        # via area() == 0 is awkward to construct directly here; instead
        # feed a zero-area Rect as the sole transformed polygon. Fires at
        # invariant 5 (is_valid on transformed), before slot_polygons is
        # read.
        degenerate = Rect(5.0, 5.0, 5.0, 5.0)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original], [degenerate], [], self.spec, LAYER_INDEX, PRINT_Z
            )
        self.assertIn("validity check", str(ctx.exception))


class TopologyValidatorMultiSlotTests(unittest.TestCase):
    """the validator must handle N > 1 slots — per-slot invariants 3/4/6 each
    naming which slot failed, plus a new collision check between adjacent
    slots. Rectangular coupon, mirroring ``TopologyValidatorTests`` one level
    up."""

    def setUp(self):
        self.ops = StubOps()
        self.validator = TopologyValidator(self.ops, safety_offset_mm=0.42)
        self.original = Rect(0.0, 0.0, 20.0, 20.0)
        self.spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)

    def test_passes_with_two_well_separated_slots(self):
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)
        notch_b = Rect(14.7, 1.0, 15.3, 20.0)
        transformed = MultiNotchedRect(outer=self.original, notches=(notch_a, notch_b))
        # Should not raise.
        self.validator.validate(
            [self.original],
            [transformed],
            [notch_a, notch_b],
            self.spec,
            LAYER_INDEX,
            PRINT_Z,
        )

    def test_passes_with_three_well_separated_slots(self):
        notch_a = Rect(2.7, 1.0, 3.3, 20.0)
        notch_b = Rect(9.7, 1.0, 10.3, 20.0)
        notch_c = Rect(16.7, 1.0, 17.3, 20.0)
        transformed = MultiNotchedRect(
            outer=self.original, notches=(notch_a, notch_b, notch_c)
        )
        self.validator.validate(
            [self.original],
            [transformed],
            [notch_a, notch_b, notch_c],
            self.spec,
            LAYER_INDEX,
            PRINT_Z,
        )

    def test_second_slot_failing_margin_names_slot_two(self):
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)  # good
        notch_b = Rect(14.7, 0.5, 15.3, 20.0)  # margin 0.5mm, below the 1.0mm required
        transformed = MultiNotchedRect(outer=self.original, notches=(notch_a, notch_b))
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [notch_a, notch_b],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
            )
        message = str(ctx.exception)
        self.assertIn("slot 2", message)
        self.assertIn("skin", message)

    def test_second_slot_not_reaching_boundary_names_slot_two(self):
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)  # good, reaches the top edge
        notch_b = Rect(14.7, 1.0, 15.3, 15.0)  # short of the top edge
        transformed = MultiNotchedRect(outer=self.original, notches=(notch_a, notch_b))
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [notch_a, notch_b],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
            )
        message = str(ctx.exception)
        self.assertIn("slot 2", message)
        self.assertIn("exterior boundary", message)

    def test_third_slot_missing_from_output_names_slot_three(self):
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)
        notch_b = Rect(14.7, 1.0, 15.3, 20.0)
        far_outside = Rect(25.0, 0.0, 26.0, 20.0)  # entirely off the section
        transformed = MultiNotchedRect(outer=self.original, notches=(notch_a, notch_b))
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [notch_a, notch_b, far_outside],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
            )
        message = str(ctx.exception)
        self.assertIn("slot 3", message)
        self.assertIn("not present in the output", message)

    def test_slots_too_close_together_raise_collision_naming_both(self):
        # notch_a's right edge is at x=5.3; notch_b starts at x=5.4 -- a
        # 0.1 mm wall, well below the required 2*0.42=0.84 mm minimum.
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)
        notch_b = Rect(5.4, 1.0, 6.0, 20.0)
        transformed = MultiNotchedRect(outer=self.original, notches=(notch_a, notch_b))
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [notch_a, notch_b],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
            )
        message = str(ctx.exception)
        self.assertIn("slots 1 and 2", message)
        self.assertIn("0.1000 mm", message)
        self.assertIn("0.84", message)

    def test_slots_exactly_at_minimum_wall_pass(self):
        # Gap exactly equal to the 0.84 mm minimum (2 * 0.42 mm safety
        # offset) must pass -- the collision check must not be stricter
        # than its own documented threshold.
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)
        notch_b = Rect(6.14, 1.0, 6.74, 20.0)  # gap = 6.14 - 5.3 = 0.84
        transformed = MultiNotchedRect(outer=self.original, notches=(notch_a, notch_b))
        self.validator.validate(
            [self.original],
            [transformed],
            [notch_a, notch_b],
            self.spec,
            LAYER_INDEX,
            PRINT_Z,
        )

    def test_slot_labels_annotate_a_failure_message(self):
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)
        notch_b = Rect(5.4, 1.0, 6.0, 20.0)  # too close, as above
        transformed = MultiNotchedRect(outer=self.original, notches=(notch_a, notch_b))
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [notch_a, notch_b],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
                slot_labels=["family 0 ('shallow skin')", "family 1 ('deep rib')"],
            )
        message = str(ctx.exception)
        self.assertIn("slots 1 (family 0 ('shallow skin')) and 2 (family 1 ('deep rib'))", message)

    def test_omitting_slot_labels_reproduces_unlabelled_message(self):
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)
        notch_b = Rect(5.4, 1.0, 6.0, 20.0)
        transformed = MultiNotchedRect(outer=self.original, notches=(notch_a, notch_b))
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [notch_a, notch_b],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
            )
        self.assertIn("slots 1 and 2", str(ctx.exception))

    def test_cross_family_collision_shallow_and_deep_at_same_station(self):
        # A shallow skin web (stops well short of the far surface) and a
        # deep rib (runs the full depth) placed at the SAME chord station
        # -- exactly the case web depth/families newly makes possible. The
        # collision check has no notion of "family"; it only ever measures
        # the slots it is handed, so two overlapping footprints from
        # DIFFERENT families collide exactly as two from the same family
        # would.
        deep_rib = Rect(9.7, 1.0, 10.3, 20.0)  # full depth: y in [1.0, 20.0]
        shallow_skin = Rect(9.7, 15.0, 10.3, 20.0)  # shallow: y in [15.0, 20.0]
        transformed = MultiNotchedRect(outer=self.original, notches=(deep_rib, shallow_skin))
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [deep_rib, shallow_skin],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
                slot_labels=["family 0 ('deep rib')", "family 1 ('shallow skin')"],
            )
        message = str(ctx.exception)
        self.assertIn("deep rib", message)
        self.assertIn("shallow skin", message)


class TopologyValidatorAirfoilTests(unittest.TestCase):
    """Invariants 3 and 4 measured against LOCAL geometry, on the real NACA 2412
    section, so bounding-box margin checks cannot give a false negative (see
    docs/geometry_invariants.md).

    ``TopologyValidator`` never sees a ``LayerFrame``/``web_position``
    directly; ``NotchedPolygon`` (stub_ops.py) lets these tests hand it a
    real, non-rectangular ORIGINAL and a hand-placed notch without
    reimplementing general polygon Boolean subtraction.
    """

    def setUp(self):
        self.ops = StubOps()
        self.polygon, self.bbox = _naca2412_polygon()
        self.frame = LayerFrame.from_polygon(self.ops, self.polygon)
        self.skin_margin_mm = 1.0
        self.web_position = 0.7
        # safety_offset_mm well under skin_margin_mm/2, per
        # TopologyValidator's own documented precondition.
        self.validator = TopologyValidator(self.ops, safety_offset_mm=0.1)
        self.spec = StructureSpec(
            gap_ratio=1.0,
            skin_margin_mm=self.skin_margin_mm,
            web_position=self.web_position,
        )

    def _local_span_at_web(self):
        x = self.frame.web_x(self.web_position)
        return x, local_span(self.ops, self.polygon, x, DEFAULT_PROBE_WIDTH_MM)

    def test_slot_undershooting_local_lower_surface_fails_invariant_4_where_old_bbox_would_pass(
        self,
    ):
        # A bounding-box invariant 4 would compute
        #     remaining_skin = removed_min_y - original_min_y   (bbox min_y)
        # A slot placed exactly at the section's bbox-derived margin point
        # satisfies that formula EXACTLY (remaining_skin == skin_margin_mm,
        # asserted below) and would PASS. At 0.7 chord on this cambered
        # airfoil, that bbox-derived point sits below the LOCAL lower surface
        # -- i.e. the slot has already cut through the local skin -- which the
        # local-geometry invariant 4 must catch instead.
        x, (local_y_min, local_y_max) = self._local_span_at_web()
        old_bbox_margin_y = self.frame.min_y + self.skin_margin_mm

        # Precondition on the fixture: if this stops holding, the fixture no
        # longer demonstrates the bug and this test would pass vacuously.
        self.assertLess(
            old_bbox_margin_y,
            local_y_min,
            "fixture no longer demonstrates the bbox-vs-local gap at 0.7 "
            "chord -- this test would no longer document the bug it exists "
            "to prevent.",
        )
        old_bbox_remaining_skin = old_bbox_margin_y - self.bbox[1]
        self.assertAlmostEqual(
            old_bbox_remaining_skin, self.skin_margin_mm, places=9
        )  # -> the bbox formula's PASS.

        notch = Rect(x - 0.15, old_bbox_margin_y, x + 0.15, local_y_max)
        transformed = NotchedPolygon(outer=self.polygon, notch=notch)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.polygon], [transformed], [notch], self.spec, LAYER_INDEX, PRINT_Z
            )
        err = ctx.exception
        self.assertEqual(err.layer_index, LAYER_INDEX)
        self.assertEqual(err.print_z, PRINT_Z)
        message = str(err)
        self.assertIn("layer {}".format(LAYER_INDEX), message)
        self.assertIn("Z={:.2f} mm".format(PRINT_Z), message)
        self.assertIn("skin", message)

        new_remaining_skin = old_bbox_margin_y - local_y_min
        self.assertLess(new_remaining_skin, self.skin_margin_mm)
        self.assertLess(
            new_remaining_skin,
            0.0,
            "the slot has cut past the local lower surface entirely -- "
            "remaining skin should read negative.",
        )

    def test_correctly_placed_slot_passes_all_invariants(self):
        # THE ROUND TRIP THAT FAILED IN-HOST: a slot placed by
        # web_centerline() (the generator's own placement logic), then
        # validated by TopologyValidator (the validator's own independent
        # measurement), on the real cambered NACA 2412 section. Both
        # measure the worst case across the same slot-width span and must
        # agree by construction; measuring different chord stations could
        # disagree by more than the skin margin.
        x = self.frame.web_x(self.web_position)
        slot_width_mm = 0.3
        boundary_point, margin_point = self.frame.web_centerline(
            self.web_position, self.skin_margin_mm, slot_width_mm
        )
        half_w = slot_width_mm / 2.0
        notch = Rect(x - half_w, margin_point[1], x + half_w, boundary_point[1])
        transformed = NotchedPolygon(outer=self.polygon, notch=notch)
        # Should not raise.
        self.validator.validate(
            [self.polygon], [transformed], [notch], self.spec, LAYER_INDEX, PRINT_Z
        )

    def test_round_trip_survives_a_removed_region_off_centre_from_the_web(self):
        # Mirrors what actually happened in-host: a real Clipper boolean's
        # removed region need not be exactly centred on the web centreline
        # the way a hand-built symmetric notch fixture is (see
        # tests/integration/fake_orca.py's KNOWN DIVERGENCES note on why its
        # own fake diff_ex cannot reproduce this asymmetry). Simulate that
        # by validating against a notch whose actual X span is SHIFTED
        # inside the placed slot's footprint, rather than centred on it, and
        # confirm the worst-case-over-span fix still passes: the worst case
        # over the shifted (smaller) span can only be less extreme than the
        # worst case over the full placed span web_centerline already
        # guarded against.
        x = self.frame.web_x(self.web_position)
        slot_width_mm = 0.3
        boundary_point, margin_point = self.frame.web_centerline(
            self.web_position, self.skin_margin_mm, slot_width_mm
        )
        half_w = slot_width_mm / 2.0
        # Shifted 0.05 mm toward +X and narrowed slightly, but still fully
        # inside [x - half_w, x + half_w] -- exactly the kind of disagreement
        # a centre-point probe would re-measure at a different, unguarded
        # station.
        shifted_min_x = x - half_w + 0.05
        shifted_max_x = x + half_w
        notch = Rect(shifted_min_x, margin_point[1], shifted_max_x, boundary_point[1])
        transformed = NotchedPolygon(outer=self.polygon, notch=notch)
        # ``notch`` is what was actually passed as the cut clip (same
        # convention as capability.py's ``slots``, which is what was really
        # handed to ``ops.diff``) -- not the wider, un-shifted
        # ``[x - half_w, x + half_w]`` footprint ``web_centerline`` placed.
        # Should not raise.
        self.validator.validate(
            [self.polygon], [transformed], [notch], self.spec, LAYER_INDEX, PRINT_Z
        )

    def test_slot_not_reaching_local_upper_surface_fails_invariant_3(self):
        x, (local_y_min, local_y_max) = self._local_span_at_web()
        short_of_boundary = local_y_max - 0.5
        notch = Rect(
            x - 0.15, local_y_min + self.skin_margin_mm, x + 0.15, short_of_boundary
        )
        transformed = NotchedPolygon(outer=self.polygon, notch=notch)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.polygon], [transformed], [notch], self.spec, LAYER_INDEX, PRINT_Z
            )
        message = str(ctx.exception)
        self.assertIn("exterior boundary", message)
        self.assertIn("layer {}".format(LAYER_INDEX), message)
        self.assertIn("Z={:.2f} mm".format(PRINT_Z), message)


if __name__ == "__main__":
    unittest.main()
