"""Extended no-sever proof for OPPOSING (+Y/-Y) entry: boundary fixtures.

See ``docs/geometry_invariants.md``, "Extended no-sever proof — opposing
(+Y/-Y) entry", for the full argument these fixtures encode.

The classes fall into explicitly labelled kinds, matching the proof's own
structure:

- ``MeetingConditionFormulaTests`` — A small pure-Python reference
  form of the proof's arithmetic (``_opposing_notches_meet`` etc. below).
  This is NOT the shipped check; it exists so the proof's own inequalities can
  be exercised directly, independent of any validator or generator.
- ``TwoSidedNotchedRectErosionTests`` — Geometric TRUTH: builds
  ``stub_ops.TwoSidedNotchedRect`` fixtures and erodes them via
  ``StubOps.offset`` directly (piece count), proving the meeting condition's
  CONSEQUENCE (severing) is real geometry, not merely an arithmetic
  assertion — the same "validator-generic, hand-built geometry" pattern
  ``test_junction.py`` uses for its sever/hole fixtures.
- ``PartialOverlapSpanDiscriminationTests`` — Exercises
  ``aircraft_core.frame``'s worst-case-over-a-span machinery
  directly on a wedge section, showing a single-station probe at either
  notch's centre would wrongly pass a case the overlap-span worst case
  correctly fails.
- ``OpposingEntryValidatorFeatureAbsentTests`` — Calls the REAL
  ``TopologyValidator.validate()`` with a -Y-entry slot and asserts the
  end-state the proof requires: a comfortably-clearing pair validates, and a
  meeting pair fails closed NAMING the opposing relationship.
"""

import unittest
from dataclasses import dataclass

from aircraft_core.frame import (
    DEFAULT_PROBE_WIDTH_MM,
    local_lower_worst_case,
    local_span,
    local_upper_worst_case,
)
from aircraft_core.spec import StructureSpec
from aircraft_core.validate import TopologyError, TopologyValidator
from stub_ops import PolygonShape, Rect, StubOps, TwoSidedNotchedRect

LAYER_INDEX = 12
PRINT_Z = 1.20

# Matches this project's shipped defaults, and test_junction.py's own
# MIN_WALL_MM convention (2 * 0.42 mm safety offset).
SAFETY_OFFSET_MM = 0.42
MIN_WALL_MM = 2.0 * SAFETY_OFFSET_MM  # 0.84

# skin_margin_mm = 2 * outer_wall_line_width_mm + 0.021, with
# outer_wall_line_width_mm == safety_offset_mm in this project's convention
# (validate.py's own docstring: safety_offset_mm IS "the resolved outer wall
# width"). 2 * 0.42 + 0.021 = 0.861.
SKIN_MARGIN_MM = 2.0 * SAFETY_OFFSET_MM + 0.021  # 0.861


# ---------------------------------------------------------------------------
# Reference form of the proof's arithmetic (docs/geometry_invariants.md).
# NOT the shipped check -- exists to exercise the proof directly.
# ---------------------------------------------------------------------------


def _opposing_notches_meet(
    d_upper_mm: float,
    d_lower_mm: float,
    t_local_mm: float,
    min_wall_mm: float = MIN_WALL_MM,
    epsilon_mm: float = 1e-9,
) -> bool:
    """True iff two opposing notches of depth ``d_upper_mm``/``d_lower_mm`` at a
    station of local thickness ``t_local_mm`` MEET (unsafe) — proof point 4:
    equality FAILS (fails closed), the reverse of the collision check's ``gap >= min_wall -
    EPSILON_MM: continue`` bias, because at equality the wall is already
    zero-width before any safety erosion runs.
    """
    return (d_upper_mm + d_lower_mm + min_wall_mm) >= (t_local_mm - epsilon_mm)


def _haloed_footprints_interact(
    top_min_x: float,
    top_max_x: float,
    bottom_min_x: float,
    bottom_max_x: float,
    safety_offset_mm: float = SAFETY_OFFSET_MM,
) -> bool:
    """True iff a +Y notch's X footprint and a -Y notch's X footprint, each
    widened by ``safety_offset_mm`` on every side (the halo, proof point 1),
    overlap at all -- equivalently, the raw (unhaloed) gap between them is <
    ``2 * safety_offset_mm``. Two notches that do NOT interact under this test
    cannot meet regardless of depth: there is no shared span for the meeting
    condition to be evaluated over.
    """
    gap = max(bottom_min_x - top_max_x, top_min_x - bottom_max_x)
    return gap < 2.0 * safety_offset_mm


class MeetingConditionFormulaTests(unittest.TestCase):
    """Proof arithmetic, exercised directly. GREEN — pure functions, no
    geometry backend."""

    def test_comfortably_clearing_pair_is_safe(self):
        # own worked example: 1.5 mm stiffeners either side of an 8 mm section.
        # 1.5 + 1.5 + 0.84 = 3.84 << 8.
        self.assertFalse(_opposing_notches_meet(1.5, 1.5, t_local_mm=8.0))

    def test_exact_meeting_condition_is_unsafe(self):
        # d_upper + d_lower + min_wall == t_local exactly: a wall that erodes
        # to precisely zero width. Proof point 4: equality FAILS, the opposite
        # epsilon bias from own collision check.
        d_upper = 4.0
        t_local_mm = 10.0
        d_lower_mm = t_local_mm - MIN_WALL_MM - d_upper  # exact equality
        self.assertTrue(
            _opposing_notches_meet(d_upper, d_lower_mm, t_local_mm=t_local_mm)
        )

    def test_overlapping_pair_is_unsafe(self):
        # d_upper + d_lower already exceeds t_local outright (the notches'
        # own Y-ranges would intersect before any erosion is even applied).
        self.assertTrue(_opposing_notches_meet(6.0, 6.0, t_local_mm=10.0))

    def test_full_depth_web_opposed_by_any_realistic_web_is_unsafe(self):
        # Proof point 5: a full-depth web's far end sits skin_margin_mm short
        # of the far surface, so d_upper = t_local - skin_margin_mm at every
        # overlapping station. The safe bound on the OPPOSING depth is d_lower
        # < skin_margin_mm - min_wall = 0.021 mm at shipped defaults -- below
        # any web this project would ever configure (own example is 1.5 mm).
        t_local_mm = 8.0
        d_upper_full_depth_mm = t_local_mm - SKIN_MARGIN_MM
        safe_bound_mm = SKIN_MARGIN_MM - MIN_WALL_MM
        self.assertAlmostEqual(safe_bound_mm, 0.021, places=6)

        for d_lower_mm in (1.5, 0.5, 0.05, 0.022):
            with self.subTest(d_lower_mm=d_lower_mm):
                self.assertGreater(d_lower_mm, safe_bound_mm)
                self.assertTrue(
                    _opposing_notches_meet(
                        d_upper_full_depth_mm, d_lower_mm, t_local_mm=t_local_mm
                    )
                )

        # Below the bound (0.021 mm), a purely hypothetical, unprintably
        # shallow web WOULD be safe by the formula -- confirming the bound
        # is exactly where the formula says it is, not merely "very small".
        self.assertFalse(
            _opposing_notches_meet(
                d_upper_full_depth_mm, 0.01, t_local_mm=t_local_mm
            )
        )

    def test_non_overlapping_opposing_footprints_do_not_interact(self):
        # Proof: two opposing notches whose HALOED footprints do not
        # overlap in X cannot meet, regardless of depth -- there is no
        # shared span for the meeting condition to apply to. This is the
        # "no two cuts can meet" premise holding trivially, not a
        # coincidence of the depths chosen.
        top = (0.0, 2.0)   # +Y notch spans x in [0, 2]
        bottom = (5.0, 7.0)  # -Y notch spans x in [5, 7]
        self.assertFalse(
            _haloed_footprints_interact(top[0], top[1], bottom[0], bottom[1])
        )
        # Even a full-depth pair at these X ranges is therefore valid: no
        # opposing check needs to run at all, because they never share a
        # station.
        self.assertFalse(
            _opposing_notches_meet(100.0, 100.0, t_local_mm=1.0)
            and _haloed_footprints_interact(top[0], top[1], bottom[0], bottom[1])
        )

    def test_footprints_within_min_wall_but_not_touching_still_interact(self):
        # Gap of exactly MIN_WALL_MM - epsilon between raw footprints: the halo
        # widening (safety_offset_mm each side) makes them touch, so the
        # meeting condition must still be evaluated -- this is the same
        # "near-tangent" hazard characterizes for same-edge pairs, generalized
        # to opposite edges.
        top_max_x = 5.0
        bottom_min_x = top_max_x + (MIN_WALL_MM - 0.01)
        self.assertTrue(
            _haloed_footprints_interact(0.0, top_max_x, bottom_min_x, bottom_min_x + 2.0)
        )


class PartialOverlapSpanDiscriminationTests(unittest.TestCase):
    """Proof point 1 and 3: partial X-footprint overlap must be evaluated
    over the OVERLAP SPAN via ``frame.py``'s existing worst-case-over-a-span
    machinery, not at a single station -- a single-station probe at either
    notch's own centreline can wrongly pass a case the span-based worst case
    correctly fails. GREEN: exercises real ``aircraft_core.frame`` code
    against a hand-built wedge section, no new feature involved.
    """

    def setUp(self):
        self.ops = StubOps()
        # A trapezoid: flat top at y=5, straight sloped bottom y = x - 3,
        # over x in [0, 7]. Thickness decreases from 8 mm (x=0) to 1 mm
        # (x=7) -- monotonic, so the worst station in any span is always
        # one of its two edges, and DEFAULT_WORST_CASE_SAMPLES samples both
        # edges by construction (see frame.py's _sample_stations).
        self.wedge = PolygonShape(
            ((0.0, -3.0), (7.0, 4.0), (7.0, 5.0), (0.0, 5.0))
        )

    def test_thickness_decreases_across_the_span_as_designed(self):
        # Sanity check on the fixture itself before using it to discriminate.
        for x, expected_thickness in ((4.0, 4.0), (5.0, 3.0), (6.0, 2.0)):
            y_min, y_max = local_span(self.ops, self.wedge, x, DEFAULT_PROBE_WIDTH_MM)
            self.assertAlmostEqual(y_max - y_min, expected_thickness, delta=0.01)

    def test_single_station_probe_wrongly_passes_where_span_worst_case_fails(self):
        # Overlap span [4, 6] (e.g. two opposing notches whose haloed
        # footprints intersect there). A probe at the span's OWN centre
        # (x=5, thickness 3.0 mm) is what a naive "check at the overlap
        # centre" implementation would use. The true worst station across
        # the span is its thin edge (x=6, thickness 2.0 mm) -- exactly what
        # local_upper_worst_case/local_lower_worst_case, sampling the whole
        # span including both edges, correctly find.
        d_upper_mm = 1.0
        d_lower_mm = 1.0

        # Single-station probe at the overlap span's centre.
        y_min_center, y_max_center = local_span(
            self.ops, self.wedge, 5.0, DEFAULT_PROBE_WIDTH_MM
        )
        t_local_single_station = y_max_center - y_min_center
        self.assertAlmostEqual(t_local_single_station, 3.0, delta=0.01)
        naive_verdict_safe = not _opposing_notches_meet(
            d_upper_mm, d_lower_mm, t_local_mm=t_local_single_station
        )
        self.assertTrue(
            naive_verdict_safe,
            "the single-station probe was expected to (wrongly) call this safe",
        )

        # Span-worst-case measurement, the proof's own prescription: worst
        # case is a conservative LOWER BOUND on true thickness everywhere in
        # the span (docs/geometry_invariants.md point 3's proof-by-bound).
        span_upper = local_upper_worst_case(
            self.ops, self.wedge, x_center_mm=5.0, span_mm=2.0
        )
        span_lower = local_lower_worst_case(
            self.ops, self.wedge, x_center_mm=5.0, span_mm=2.0
        )
        t_local_span_worst = span_upper - span_lower
        self.assertAlmostEqual(t_local_span_worst, 2.0, delta=0.01)
        span_verdict_safe = not _opposing_notches_meet(
            d_upper_mm, d_lower_mm, t_local_mm=t_local_span_worst
        )
        self.assertFalse(
            span_verdict_safe,
            "the overlap-span worst case was expected to correctly fail this",
        )

        # The discrimination itself: the two measurements disagree, and the
        # span-based one is the conservative (correct) one.
        self.assertNotEqual(naive_verdict_safe, span_verdict_safe)
        self.assertLess(t_local_span_worst, t_local_single_station)


class TwoSidedNotchedRectErosionTests(unittest.TestCase):
    """Geometric TRUTH of the meeting condition's CONSEQUENCE (severing), via
    ``StubOps.offset`` on a hand-built ``TwoSidedNotchedRect`` -- the same
    "validator-generic geometry, not validate()" pattern ``test_junction.py``
    uses for own sever fixtures. GREEN.
    """

    def setUp(self):
        self.ops = StubOps()

    def test_comfortably_clearing_pair_survives_erosion_as_one_piece(self):
        # own worked example, reproduced as real geometry: 1.5 mm stiffeners
        # either side of an 8 mm section, 20 mm wide.
        outer = Rect(0.0, 0.0, 20.0, 8.0)
        shape = TwoSidedNotchedRect(
            outer=outer,
            notch_top=Rect(8.0, 6.5, 12.0, 8.0),     # depth 1.5 mm from +Y
            notch_bottom=Rect(8.0, 0.0, 12.0, 1.5),  # depth 1.5 mm from -Y
        )
        self.assertAlmostEqual(shape.middle_wall(), 5.0, places=6)
        pieces = self.ops.offset(shape, -SAFETY_OFFSET_MM)
        self.assertEqual(len(pieces), 1)
        self.assertIsInstance(pieces[0], TwoSidedNotchedRect)

    def test_exact_meeting_condition_severs_into_two_pieces(self):
        # middle_wall() == MIN_WALL_MM exactly: after erosion by
        # safety_offset_mm on each side (shrink == MIN_WALL_MM), the wall
        # is exactly zero -- proof point 4, equality fails closed.
        outer = Rect(0.0, 0.0, 20.0, 20.0)
        d_upper = d_lower = (20.0 - MIN_WALL_MM) / 2.0
        shape = TwoSidedNotchedRect(
            outer=outer,
            notch_top=Rect(8.0, 20.0 - d_upper, 12.0, 20.0),
            notch_bottom=Rect(8.0, 0.0, 12.0, d_lower),
        )
        self.assertAlmostEqual(shape.middle_wall(), MIN_WALL_MM, places=9)
        pieces = self.ops.offset(shape, -SAFETY_OFFSET_MM)
        self.assertEqual(
            len(pieces),
            2,
            "a wall exactly at MIN_WALL_MM must fail closed (sever), not "
            "survive at the boundary the way the wall-gap check would",
        )
        for piece in pieces:
            self.assertIsInstance(piece, Rect)

    def test_overlapping_pair_severs_before_any_erosion_is_even_applied(self):
        # notch_top and notch_bottom's Y-ranges literally intersect
        # (middle_wall() < 0): the section is already cut through at full
        # resolution, with no erosion needed to expose it. area() is not
        # meaningful for this fixture (the overlap region would be double
        # -subtracted) and is deliberately not called here -- only offset()
        # piece count is exercised, which is what this fixture exists to
        # prove.
        outer = Rect(0.0, 0.0, 20.0, 20.0)
        shape = TwoSidedNotchedRect(
            outer=outer,
            notch_top=Rect(8.0, 9.5, 12.0, 20.0),     # depth 10.5
            notch_bottom=Rect(8.0, 0.0, 12.0, 10.0),  # depth 10.0
        )
        self.assertLess(shape.middle_wall(), 0.0)
        pieces = self.ops.offset(shape, 0.0)  # no erosion delta needed
        self.assertEqual(len(pieces), 2)

    def test_full_depth_web_opposed_by_any_realistic_web_severs(self):
        # Proof point 5, as real geometry: an 8 mm section, a full-depth +Y web
        # (far end skin_margin_mm from the -Y surface) opposed by a 1.5 mm -Y
        # web -- own worked "skin stiffener" depth, and nowhere near the 0.021
        # mm bound that would clear.
        t_local_mm = 8.0
        d_upper_full_depth_mm = t_local_mm - SKIN_MARGIN_MM
        d_lower_mm = 1.5
        outer = Rect(0.0, 0.0, 20.0, t_local_mm)
        shape = TwoSidedNotchedRect(
            outer=outer,
            notch_top=Rect(8.0, t_local_mm - d_upper_full_depth_mm, 12.0, t_local_mm),
            notch_bottom=Rect(8.0, 0.0, 12.0, d_lower_mm),
        )
        self.assertLess(
            shape.middle_wall(),
            0.0,
            "a full-depth web opposed by any realistic web should already "
            "be a literal Y-overlap, not merely a thin surviving wall",
        )
        pieces = self.ops.offset(shape, 0.0)
        self.assertEqual(len(pieces), 2)


class OpposingEntryValidatorFeatureAbsentTests(unittest.TestCase):
    """These assert the proof's required end-state against the REAL
    ``TopologyValidator.validate()``: a -Y-entry slot must be accepted when it
    clears, and a meeting opposing pair must fail closed with a message that
    names the opposing relationship (the right outcome for the right cause).
    Do not loosen the assertions to accept a failure that merely happens to
    raise.
    """

    def setUp(self):
        self.ops = StubOps()
        self.validator = TopologyValidator(self.ops, safety_offset_mm=SAFETY_OFFSET_MM)
        self.spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=SKIN_MARGIN_MM)

    def test_a_comfortably_clearing_minus_y_slot_should_validate_desired_end_state(self):
        # By every metric this slot is fine: 1.5 mm deep on a 20 mm section,
        # nowhere near the far (+Y) skin. Required: validate() returns
        # without raising. A validator that assumed +Y-only entry
        # unconditionally would reject any -Y-entry slot outright, however
        # safely it clears the far surface.
        original = Rect(0.0, 0.0, 20.0, 20.0)
        slot = Rect(8.0, 0.0, 12.0, 1.5)  # -Y entry, depth 1.5 mm

        @dataclass(frozen=True)
        class _BottomNotchedRect:
            outer: Rect
            notch: Rect

            def area(self):
                return self.outer.area() - self.notch.area()

            def bounds(self):
                return self.outer.bounds()

        transformed = _BottomNotchedRect(outer=original, notch=slot)

        try:
            self.validator.validate(
                [original],
                [transformed],
                [slot],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
            )
        except TopologyError as exc:
            self.fail(
                "Desired end state: a comfortably-clearing -Y slot "
                "should validate cleanly. Today's TopologyValidator raises "
                "instead, for a reason unrelated to the opposing-entry proof (invariant "
                "3 assumes +Y-only entry unconditionally): {}".format(exc)
            )

    def test_the_exact_meeting_condition_pair_should_fail_naming_the_opposing_pair(self):
        # Required: FAIL closed, with a message that names the opposing pair
        # / meeting condition (a collision check generalized cross-edge) --
        # the way the same-edge collision-check messages name "slots N and M"
        # and "different families". A TopologyError raised for some other
        # cause (e.g. on the -Y slot alone, before the two slots' depths are
        # compared) would be the right outcome for the wrong reason, so this
        # asserts the CAUSE, not merely that some exception is raised.
        t_local_mm = 8.0
        original = Rect(0.0, 0.0, 20.0, t_local_mm)
        d_upper_full_depth_mm = t_local_mm - SKIN_MARGIN_MM
        slot_top = Rect(8.0, t_local_mm - d_upper_full_depth_mm, 12.0, t_local_mm)
        slot_bottom = Rect(8.0, 0.0, 12.0, 1.5)

        @dataclass(frozen=True)
        class _TwoSlotRemoved:
            outer: Rect
            notch_a: Rect
            notch_b: Rect

            def area(self):
                return (
                    self.outer.area() - self.notch_a.area() - self.notch_b.area()
                )

            def bounds(self):
                return self.outer.bounds()

        transformed = _TwoSlotRemoved(
            outer=original, notch_a=slot_top, notch_b=slot_bottom
        )

        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [original],
                [transformed],
                [slot_top, slot_bottom],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
                slot_labels=["family 0 ('internal')", "family 1 ('skin')"],
            )
        message = str(ctx.exception).lower()
        self.assertTrue(
            "opposing" in message or "meet" in message,
            "Desired end state: the failure should name the opposing "
            "pair / meeting condition, the way the cross-family message "
            "names 'different families'. Today's actual message is: "
            "{}".format(ctx.exception),
        )


if __name__ == "__main__":
    unittest.main()
