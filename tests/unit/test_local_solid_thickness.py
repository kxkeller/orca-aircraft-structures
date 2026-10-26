"""``frame.local_solid_thickness_worst_case``.

See that function's own docstring (``aircraft_core/frame.py``) for the full
contract this pins:

- exterior-to-exterior span minus whatever declared-hole extent falls
  inside it;
- a hole entirely inside the span, straddling either end, or not
  intersecting at all -- each handled, documented below;
- the two-band case (a span crossing one hole) reports the SUM of both
  bands as ``solid_mm``, and carries ``hole_extent_subtracted_mm`` so a
  caller with a "one contiguous wall" formula (``opposing_pair_meets``) can
  apply its own correction;
- multiple holes: side-by-side in X are summed conservatively; holes
  stacked so a single probe crosses more than one fail closed via
  ``local_span``'s own existing multi-piece guard;
- byte-identical to the plain ``t_upper - t_lower`` composition whenever no
  declared hole overlaps the span.

``tests/unit/test_local_span_hole_aware.py`` covers ``opposing_pair_meets``'s
own hole-station behaviour (the wiring of this primitive into that
caller, including the two-band ``min_wall_mm`` discount).
"""

import unittest

from aircraft_core.frame import (
    DEFAULT_PROBE_WIDTH_MM,
    local_lower_worst_case,
    local_solid_thickness_worst_case,
    local_upper_worst_case,
)
from stub_ops import HoledRect, MultiHoledRect, Rect, StubOps


class NoHoleTests(unittest.TestCase):
    """A span with no declared hole anywhere near it: byte-identical to the
    plain ``t_upper - t_lower`` composition."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 40.0, 20.0)

    def test_solid_equals_plain_span(self):
        result = local_solid_thickness_worst_case(
            self.ops, self.rect, x_center_mm=20.0, span_mm=4.0
        )
        expected_upper = local_upper_worst_case(
            self.ops, self.rect, x_center_mm=20.0, span_mm=4.0
        )
        expected_lower = local_lower_worst_case(
            self.ops, self.rect, x_center_mm=20.0, span_mm=4.0
        )
        self.assertAlmostEqual(result.solid_mm, expected_upper - expected_lower)
        self.assertEqual(result.hole_extent_subtracted_mm, 0.0)

    def test_holed_shape_but_hole_outside_the_span_is_unaffected(self):
        # Hole is real, but its X footprint [16, 24] never overlaps a span
        # centred at x=5.
        holed = HoledRect(outer=self.rect, hole=Rect(16.0, 7.0, 24.0, 13.0))
        result = local_solid_thickness_worst_case(
            self.ops, holed, x_center_mm=5.0, span_mm=2.0
        )
        self.assertAlmostEqual(result.solid_mm, 20.0, delta=0.01)
        self.assertEqual(result.hole_extent_subtracted_mm, 0.0)


class HoleEntirelyInsideSpanTests(unittest.TestCase):
    """The hole's X footprint is entirely inside the sampled span -- every
    station crosses it, so the exterior surfaces read from the piece
    outside the hole, and the hole's own bbox height is subtracted."""

    def setUp(self):
        self.ops = StubOps()
        self.outer = Rect(0.0, 0.0, 40.0, 20.0)
        self.hole = Rect(16.0, 7.0, 24.0, 13.0)  # height 6
        self.holed = HoledRect(outer=self.outer, hole=self.hole)

    def test_solid_is_full_span_minus_hole_height(self):
        # Narrow span [19, 21], well inside the hole's [16, 24] footprint.
        result = local_solid_thickness_worst_case(
            self.ops, self.holed, x_center_mm=20.0, span_mm=2.0
        )
        # full_span = 20.0 (exterior skin, flat) ; hole height = 6.0.
        self.assertAlmostEqual(result.solid_mm, 14.0, delta=0.01)
        self.assertAlmostEqual(result.hole_extent_subtracted_mm, 6.0, delta=0.01)

    def test_two_band_sum_not_larger_or_smaller_band(self):
        # Band above the hole: 13 -> 20 = 7 mm. Band below: 0 -> 7 = 7 mm.
        # Sum = 14 mm -- neither the larger band (7) nor the smaller (7)
        # alone (degenerate equal-band fixture; see the asymmetric fixture
        # below for a case where they differ and sum still applies).
        result = local_solid_thickness_worst_case(
            self.ops, self.holed, x_center_mm=20.0, span_mm=2.0
        )
        self.assertAlmostEqual(result.solid_mm, 14.0, delta=0.01)

    def test_asymmetric_bands_still_sum(self):
        # Off-centre hole: band_above = 30 - 25 = 5, band_below = 4 - 0 = 4.
        outer = Rect(0.0, 0.0, 40.0, 30.0)
        hole = Rect(16.0, 4.0, 24.0, 25.0)
        holed = HoledRect(outer=outer, hole=hole)
        result = local_solid_thickness_worst_case(
            self.ops, holed, x_center_mm=20.0, span_mm=2.0
        )
        # full_span = 30.0, hole height = 21.0 -> solid = 9.0 = 5 + 4.
        self.assertAlmostEqual(result.solid_mm, 9.0, delta=0.01)
        self.assertAlmostEqual(result.hole_extent_subtracted_mm, 21.0, delta=0.01)


class HoleStraddlingSpanEdgeTests(unittest.TestCase):
    """The hole's X footprint overlaps only one end of the sampled span --
    not every station crosses it. Handled by
    ``local_upper_worst_case``/``local_lower_worst_case``'s own
    documented ability to mix hole-crossing and hole-free stations
    (``hole_aware=True``), and the hole is still counted as overlapping the
    span (so its FULL bbox height is subtracted -- deliberately
    conservative, see the primitive's own docstring)."""

    def setUp(self):
        self.ops = StubOps()
        self.outer = Rect(0.0, 0.0, 40.0, 20.0)
        self.hole = Rect(16.0, 7.0, 24.0, 13.0)  # X footprint [16, 24]
        self.holed = HoledRect(outer=self.outer, hole=self.hole)

    def test_hole_straddling_the_low_x_end_of_the_span(self):
        # Span [15, 25] straddles the hole's low edge (16) -- some sampled
        # stations (< 16) are hole-free, some (>= 16) cross the hole.
        result = local_solid_thickness_worst_case(
            self.ops, self.holed, x_center_mm=20.0, span_mm=10.0
        )
        # Flat exterior skin everywhere -> full_span = 20.0 regardless of
        # which stations cross the hole. The hole's X range [16,24]
        # overlaps the widened span window, so its full height (6.0) is
        # still subtracted.
        self.assertAlmostEqual(result.solid_mm, 14.0, delta=0.01)
        self.assertAlmostEqual(result.hole_extent_subtracted_mm, 6.0, delta=0.01)

    def test_hole_straddling_the_high_x_end_of_the_span(self):
        # Span [20, 30] straddles the hole's high edge (24).
        result = local_solid_thickness_worst_case(
            self.ops, self.holed, x_center_mm=25.0, span_mm=10.0
        )
        self.assertAlmostEqual(result.solid_mm, 14.0, delta=0.01)
        self.assertAlmostEqual(result.hole_extent_subtracted_mm, 6.0, delta=0.01)

    def test_hole_counted_via_widened_probe_window_even_when_no_station_lands_inside_it(self):
        # Highest sampled station is x=15.997, strictly short of the hole's
        # own min_x=16 -- no station's probe CENTRE crosses the hole. But
        # this primitive's own overlap test widens the span by
        # probe_width_mm / 2 (0.005 mm at the default) on each side before
        # comparing against the hole's X range, so the widened window
        # [.., 16.002] still overlaps hole.min_x=16 -- the documented
        # "deliberately widened rather than tight" conservative choice
        # (over-subtract rather than risk missing a hole that only grazes
        # the span's edge).
        result = local_solid_thickness_worst_case(
            self.ops, self.holed, x_center_mm=10.997, span_mm=10.0,
            probe_width_mm=DEFAULT_PROBE_WIDTH_MM,
        )
        # No sampled station actually crosses the hole (flat exterior skin
        # either way), so full_span is still the plain 20.0 -- but the
        # widened-window overlap test still subtracts the hole's full
        # height, exactly as the primitive's docstring says it will.
        self.assertAlmostEqual(result.solid_mm, 14.0, delta=0.01)
        self.assertAlmostEqual(result.hole_extent_subtracted_mm, 6.0, delta=0.01)


class MultipleHolesTests(unittest.TestCase):
    """Two or more declared holes on one layer -- outside the supported
    design (one hole per layer), but must stay SAFE, not undefined.
    """

    def setUp(self):
        self.ops = StubOps()
        self.outer = Rect(0.0, 0.0, 40.0, 20.0)

    def test_side_by_side_holes_are_summed_conservatively(self):
        # Two holes at different X ranges, both overlapping a wide span --
        # no single station crosses both (they don't share X), but the
        # primitive sums BOTH bbox heights unconditionally per its own
        # documented conservative-summing rule.
        hole_a = Rect(5.0, 7.0, 10.0, 13.0)   # height 6, X [5,10]
        hole_b = Rect(25.0, 8.0, 30.0, 12.0)  # height 4, X [25,30]
        shape = MultiHoledRect(outer=self.outer, holes=(hole_a, hole_b))
        result = local_solid_thickness_worst_case(
            self.ops, shape, x_center_mm=17.5, span_mm=25.0,
        )
        # full_span = 20.0 (flat exterior skin); both holes overlap the
        # widened span window -> hole_extent = 6.0 + 4.0 = 10.0.
        self.assertAlmostEqual(result.hole_extent_subtracted_mm, 10.0, delta=0.01)
        self.assertAlmostEqual(result.solid_mm, 10.0, delta=0.01)

    def test_holes_stacked_in_one_column_fails_closed(self):
        # Two holes sharing the SAME X range but different Y -- a probe at
        # that station crosses material 3 times. local_span's own
        # multi-piece guard (len(piece_bounds) != 2) refuses this exactly
        # as it refuses a genuinely re-entrant section -- this primitive
        # does not special-case it, it inherits the guard for free.
        hole_a = Rect(16.0, 3.0, 24.0, 7.0)
        hole_b = Rect(16.0, 13.0, 24.0, 17.0)
        shape = MultiHoledRect(outer=self.outer, holes=(hole_a, hole_b))
        with self.assertRaises(ValueError) as ctx:
            local_solid_thickness_worst_case(
                self.ops, shape, x_center_mm=20.0, span_mm=2.0,
            )
        self.assertIn("re-entrant", str(ctx.exception))


class SamplingAndValidationTests(unittest.TestCase):
    """Basic argument validation and re-entrant propagation, matching
    ``local_upper_worst_case``/``local_lower_worst_case``'s own contract."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 40.0, 20.0)

    def test_span_must_be_positive(self):
        with self.assertRaises(ValueError):
            local_solid_thickness_worst_case(
                self.ops, self.rect, x_center_mm=20.0, span_mm=0.0
            )

    def test_no_material_raises(self):
        with self.assertRaises(ValueError):
            local_solid_thickness_worst_case(
                self.ops, self.rect, x_center_mm=100.0, span_mm=2.0
            )


if __name__ == "__main__":
    unittest.main()
