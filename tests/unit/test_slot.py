import unittest

from aircraft_core.generator import WebIntent
from aircraft_core.slot import SlotBuilder


class SlotBuilderTests(unittest.TestCase):
    def setUp(self):
        # Matches a 20x20 coupon, web_position=0.5, skin_margin=1.0.
        self.intent = WebIntent(start=(10.0, 20.0), end=(10.0, 1.0), width_mm=0.6)
        self.overshoot_mm = 0.1
        self.slot = SlotBuilder().build(self.intent, self.overshoot_mm)

    def test_width_equals_resolved_gap(self):
        xs = sorted(p[0] for p in self.slot.points)
        # Two distinct x values (left side, right side), width between them.
        left, right = xs[0], xs[-1]
        self.assertAlmostEqual(right - left, self.intent.width_mm)

    def test_overshoots_the_boundary(self):
        ys = [p[1] for p in self.slot.points]
        self.assertAlmostEqual(max(ys), self.intent.start[1] + self.overshoot_mm)
        self.assertGreater(max(ys), self.intent.start[1])

    def test_far_end_stops_exactly_at_margin(self):
        ys = [p[1] for p in self.slot.points]
        self.assertAlmostEqual(min(ys), self.intent.end[1])

    def test_far_end_is_not_extended(self):
        # No overshoot at the margin end — the far end is a square cap
        # exactly at intent.end, never past it.
        ys = [p[1] for p in self.slot.points]
        self.assertGreaterEqual(min(ys), self.intent.end[1] - 1e-9)

    def test_rectangle_centered_on_centerline_x(self):
        xs = [p[0] for p in self.slot.points]
        center = (min(xs) + max(xs)) / 2.0
        self.assertAlmostEqual(center, self.intent.start[0])

    def test_zero_overshoot_rejected(self):
        with self.assertRaises(ValueError):
            SlotBuilder().build(self.intent, 0.0)

    def test_zero_length_centerline_rejected(self):
        degenerate = WebIntent(start=(5.0, 5.0), end=(5.0, 5.0), width_mm=0.6)
        with self.assertRaises(ValueError):
            SlotBuilder().build(degenerate, 0.1)


if __name__ == "__main__":
    unittest.main()
