"""``frame.local_span`` is hole-aware, opt-in.

See ``aircraft_core.frame.local_span``'s own docstring for the full
contract this pins:

- a single-piece probe is unaffected (``ops.hole_bounds`` is never even
  called);
- a multi-piece probe whose only gap is explained by a DECLARED interior
  hole (``ops.hole_bounds``) fails closed with a message that NAMES the
  hole, unless a caller explicitly asks ``hole_side="upper"``/``"lower"``,
  in which case it returns that piece's own bounds;
- a multi-piece probe with an unexplained gap still raises the
  "re-entrant section" message, regardless of ``hole_side``, so the
  re-entrant-section guard is not weakened.

Also exercises ``local_upper_worst_case``/``local_lower_worst_case``'s
``hole_aware`` opt-in.

Composing two hole-aware probes directly is not safe for
``opposing_pair_meets``'s composite thickness formula, so it uses
``frame.local_solid_thickness_worst_case`` (a solid-thickness primitive,
exterior-to-exterior minus hole void); see
``tests/unit/test_local_solid_thickness.py`` for that primitive's coverage.
The class below pins that ``opposing_pair_meets`` returns a correct
conservative answer at a hole-crossing station instead of raising.
"""

import unittest

from aircraft_core.frame import (
    DEFAULT_PROBE_WIDTH_MM,
    local_lower_worst_case,
    local_span,
    local_upper_worst_case,
    opposing_pair_meets,
)
from stub_ops import HoledRect, PolygonShape, Rect, StubOps


class HoleFreeProbeIsByteIdenticalTests(unittest.TestCase):
    """A hole-free single-piece probe is unaffected by
    hole-awareness: same return value, and ``ops.hole_bounds`` is never called (the
    multi-piece branch that would call it is never reached)."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 40.0, 20.0)

    def test_return_value_unchanged(self):
        y_min, y_max = local_span(self.ops, self.rect, x_mm=20.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM)
        self.assertAlmostEqual(y_min, 0.0)
        self.assertAlmostEqual(y_max, 20.0)

    def test_hole_bounds_never_called_on_single_piece_probe(self):
        local_span(self.ops, self.rect, x_mm=20.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM)
        called_names = [name for name, _args in self.ops.calls]
        self.assertNotIn("hole_bounds", called_names)

    def test_hole_side_is_a_no_op_on_a_single_piece_probe(self):
        # Passing hole_side on an ordinary station changes nothing --
        # local_lower_worst_case/local_upper_worst_case must be able to
        # mix hole-crossing and hole-free stations in one worst-case scan.
        for side in (None, "upper", "lower"):
            y_min, y_max = local_span(
                self.ops, self.rect, x_mm=20.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM,
                hole_side=side,
            )
            self.assertAlmostEqual(y_min, 0.0)
            self.assertAlmostEqual(y_max, 20.0)


class HoleCrossingStationTests(unittest.TestCase):
    """A probe that crosses a DECLARED hole (``HoledRect``) -- the fixture
    the previous step's ``hole_bounds()`` made queryable and this step's
    ``StubOps._intersection_holed_rect`` made probeable at all."""

    def setUp(self):
        self.ops = StubOps()
        self.outer = Rect(0.0, 0.0, 40.0, 20.0)
        self.hole = Rect(16.0, 7.0, 24.0, 13.0)  # strictly interior spar hole
        self.holed = HoledRect(outer=self.outer, hole=self.hole)

    def test_default_hole_side_none_raises_naming_the_hole(self):
        with self.assertRaises(ValueError) as ctx:
            local_span(self.ops, self.holed, x_mm=20.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM)
        message = str(ctx.exception)
        self.assertIn("DECLARED interior hole", message)
        self.assertIn("hole_side", message)
        # Must NOT be worded as a re-entrant defect -- that message is reserved
        # for an UNEXPLAINED multi-piece probe (it would be misleading here).
        self.assertNotIn("re-entrant", message)

    def test_hole_side_upper_returns_the_piece_above_the_hole(self):
        y_min, y_max = local_span(
            self.ops, self.holed, x_mm=20.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM,
            hole_side="upper",
        )
        # The piece strictly above the hole: from the hole's own top edge
        # (13.0) up to the outer skin (20.0).
        self.assertAlmostEqual(y_min, 13.0, delta=0.01)
        self.assertAlmostEqual(y_max, 20.0, delta=0.01)

    def test_hole_side_lower_returns_the_piece_below_the_hole(self):
        y_min, y_max = local_span(
            self.ops, self.holed, x_mm=20.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM,
            hole_side="lower",
        )
        # The piece strictly below the hole: from the outer skin (0.0) up
        # to the hole's own bottom edge (7.0).
        self.assertAlmostEqual(y_min, 0.0, delta=0.01)
        self.assertAlmostEqual(y_max, 7.0, delta=0.01)

    def test_station_outside_the_holes_x_footprint_is_unaffected(self):
        # x=5 never touches the hole's own X range [16, 24] -- ordinary
        # single-piece probe, hole_side irrelevant.
        y_min, y_max = local_span(
            self.ops, self.holed, x_mm=5.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM,
        )
        self.assertAlmostEqual(y_min, 0.0, delta=0.01)
        self.assertAlmostEqual(y_max, 20.0, delta=0.01)


class GenuineReentrantSectionStillRaisesTests(unittest.TestCase):
    """The defect guard is UNWEAKENED: a multi-piece probe whose gap is NOT
    explained by any declared hole still raises the original "re-entrant
    section" message, even when a caller asks for a ``hole_side`` -- exactly
    the proof the task asked for, not merely a preserved code path."""

    def setUp(self):
        self.ops = StubOps()
        # Same "bracket" fixture as test_spar_hole.py's own re-entrant
        # class: a solid column joins a top bar to a bottom bar, no
        # interior hole anywhere -- hole_bounds() on a PolygonShape is [].
        self.bracket = PolygonShape(
            (
                (0.0, 0.0), (40.0, 0.0), (40.0, 3.0), (2.0, 3.0),
                (2.0, 17.0), (40.0, 17.0), (40.0, 20.0), (0.0, 20.0),
            )
        )

    def test_raises_reentrant_with_hole_side_none(self):
        with self.assertRaises(ValueError) as ctx:
            local_span(self.ops, self.bracket, x_mm=20.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM)
        self.assertIn("re-entrant", str(ctx.exception))

    def test_raises_reentrant_even_when_hole_side_is_requested(self):
        # A caller asking for a side does not entitle it to an answer when
        # there is no declared hole to explain the gap -- this is the
        # defect guard, not a convenience default.
        for side in ("upper", "lower"):
            with self.assertRaises(ValueError) as ctx:
                local_span(
                    self.ops, self.bracket, x_mm=20.0, probe_width_mm=DEFAULT_PROBE_WIDTH_MM,
                    hole_side=side,
                )
            self.assertIn("re-entrant", str(ctx.exception))


class WorstCaseFunctionsHoleAwareOptInTests(unittest.TestCase):
    """``local_upper_worst_case``/``local_lower_worst_case``'s own
    ``hole_aware`` flag: default ``False`` keeps the fail-closed
    behaviour byte-for-byte; ``True`` resolves a hole-crossing station to
    the correct EXTERIOR surface, including a span that mixes hole-crossing
    and hole-free stations."""

    def setUp(self):
        self.ops = StubOps()
        self.outer = Rect(0.0, 0.0, 40.0, 20.0)
        self.hole = Rect(16.0, 7.0, 24.0, 13.0)
        self.holed = HoledRect(outer=self.outer, hole=self.hole)

    def test_default_hole_aware_false_raises_on_a_holed_station(self):
        with self.assertRaises(ValueError):
            local_upper_worst_case(self.ops, self.holed, x_center_mm=20.0, span_mm=2.0)
        with self.assertRaises(ValueError):
            local_lower_worst_case(self.ops, self.holed, x_center_mm=20.0, span_mm=2.0)

    def test_hole_aware_true_reports_the_true_exterior_upper_surface(self):
        # A narrow span entirely inside the hole's X footprint: every
        # sampled station crosses the hole, so the worst-case upper surface
        # must come from the piece ABOVE the hole (y=20 exterior skin, flat
        # here so every station reports the same value).
        y_max = local_upper_worst_case(
            self.ops, self.holed, x_center_mm=20.0, span_mm=2.0, hole_aware=True,
        )
        self.assertAlmostEqual(y_max, 20.0, delta=0.01)

    def test_hole_aware_true_reports_the_true_exterior_lower_surface(self):
        y_min = local_lower_worst_case(
            self.ops, self.holed, x_center_mm=20.0, span_mm=2.0, hole_aware=True,
        )
        self.assertAlmostEqual(y_min, 0.0, delta=0.01)

    def test_hole_aware_true_handles_a_span_mixing_hole_and_hole_free_stations(self):
        # Span [15, 25] straddles the hole's own X range [16, 24] --
        # DEFAULT_WORST_CASE_SAMPLES=9 stations include some inside the
        # hole's footprint and some outside it. Every station's exterior
        # surface is the same flat y=0/y=20 skin here, so the worst case is
        # unaffected either way -- what this test actually pins is that no
        # station in the mix raises.
        y_max = local_upper_worst_case(
            self.ops, self.holed, x_center_mm=20.0, span_mm=10.0, hole_aware=True,
        )
        y_min = local_lower_worst_case(
            self.ops, self.holed, x_center_mm=20.0, span_mm=10.0, hole_aware=True,
        )
        self.assertAlmostEqual(y_max, 20.0, delta=0.01)
        self.assertAlmostEqual(y_min, 0.0, delta=0.01)


class OpposingPairMeetsIsHoleAwareTests(unittest.TestCase):
    """third step: ``opposing_pair_meets`` now returns a correct conservative
    answer at a hole-crossing overlap station, via ``frame.
    local_solid_thickness_worst_case``, instead of raising.

    Fixture: ``outer`` 40x20, ``hole`` x=[16,24] y=[7,13] (height 6). The
    overlap span here (upper/lower footprints [18,22], haloed by
    ``safety_offset_mm=0.42``) is [17.58, 22.42], entirely inside the hole's
    X footprint, so every sampled station crosses the hole -- worst-case
    exterior surfaces are the flat outer skin, y_max=20.0 / y_min=0.0, so
    ``full_span_mm = 20.0``, ``hole_extent_subtracted_mm = 6.0`` (the hole's
    own bbox height), and ``solid_mm = 20.0 - 6.0 = 14.0`` before the
    two-band ``min_wall_mm`` discount described in ``opposing_pair_meets``'s
    own docstring.
    """

    def setUp(self):
        self.ops = StubOps()
        self.outer = Rect(0.0, 0.0, 40.0, 20.0)
        self.hole = Rect(16.0, 7.0, 24.0, 13.0)
        self.holed = HoledRect(outer=self.outer, hole=self.hole)

    def test_safely_clear_pair_returns_none_no_raise(self):
        # required_mm = 2 + 2 + 0.84 = 4.84, well under
        # t_local_conservative = 14.0 - 0.84 = 13.16 -- safe, and critically
        # does NOT raise the way it did before this step.
        meeting = opposing_pair_meets(
            self.ops,
            self.holed,
            upper_min_x=18.0, upper_max_x=22.0, upper_depth_mm=2.0,
            lower_min_x=18.0, lower_max_x=22.0, lower_depth_mm=2.0,
            safety_offset_mm=0.42, min_wall_mm=0.84,
        )
        self.assertIsNone(meeting)

    def test_two_band_discount_is_the_difference_between_safe_and_unsafe(self):
        # required_mm = 6.5 + 6.5 + 0.84 = 13.84.
        # Raw sum (no discount) = 14.0 -> 13.84 < 14.0 -> would read SAFE.
        # With the min_wall_mm discount, t_local_conservative = 13.16 ->
        # 13.84 >= 13.16 -> MEETS (unsafe). This is the exact demonstration
        # that the discount changes the outcome, not merely the number --
        # see local_solid_thickness_worst_case's own worked example for the
        # two-sliver reasoning this pins.
        meeting = opposing_pair_meets(
            self.ops,
            self.holed,
            upper_min_x=18.0, upper_max_x=22.0, upper_depth_mm=6.5,
            lower_min_x=18.0, lower_max_x=22.0, lower_depth_mm=6.5,
            safety_offset_mm=0.42, min_wall_mm=0.84,
        )
        self.assertIsNotNone(meeting)
        self.assertAlmostEqual(meeting.t_local_conservative, 13.16, delta=0.01)
        self.assertAlmostEqual(meeting.required_mm, 13.84, delta=0.01)

    def test_deeply_meeting_pair_at_a_hole_station_is_flagged_unsafe(self):
        meeting = opposing_pair_meets(
            self.ops,
            self.holed,
            upper_min_x=18.0, upper_max_x=22.0, upper_depth_mm=7.0,
            lower_min_x=18.0, lower_max_x=22.0, lower_depth_mm=7.0,
            safety_offset_mm=0.42, min_wall_mm=0.84,
        )
        self.assertIsNotNone(meeting)
        self.assertAlmostEqual(meeting.t_local_conservative, 13.16, delta=0.01)

    def test_no_hole_at_the_overlap_station_is_unaffected(self):
        # x=5 overlap, nowhere near the hole's X footprint [16,24] -- must
        # behave exactly like the plain composition (no discount).
        meeting = opposing_pair_meets(
            self.ops,
            self.holed,
            upper_min_x=3.0, upper_max_x=7.0, upper_depth_mm=2.0,
            lower_min_x=3.0, lower_max_x=7.0, lower_depth_mm=2.0,
            safety_offset_mm=0.42, min_wall_mm=0.84,
        )
        self.assertIsNone(meeting)


if __name__ == "__main__":
    unittest.main()
