import unittest

from aircraft_core.frame import LayerFrame
from stub_ops import Rect, StubOps


class FrameTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        # 20x20 mm coupon cross-section, matching the fixture used.
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def test_bounds_and_dimensions(self):
        self.assertEqual(self.frame.width, 20.0)
        self.assertEqual(self.frame.depth, 20.0)

    def test_web_position_zero_with_nonzero_width_is_off_the_chord(self):
        # A REAL (nonzero-width) slot centred exactly at the chord's own
        # leading tip necessarily samples stations past x=0, off the
        # section entirely -- there is no room for it there regardless of
        # how narrow it is. web_centerline samples the slot's actual width, not
        # just the centreline point (x=0 itself, which IS on the section), so
        # it catches this geometric impossibility rather than silently
        # placing a slot whose edge is not actually resolvable.
        with self.assertRaises(ValueError):
            self.frame.web_centerline(0.0, skin_margin_mm=1.0, slot_width_mm=0.6)

    def test_web_position_near_zero_with_room_for_the_slot_width(self):
        boundary, margin = self.frame.web_centerline(0.02, skin_margin_mm=1.0, slot_width_mm=0.6)
        self.assertEqual(boundary, (0.4, 20.0))
        self.assertEqual(margin, (0.4, 1.0))

    def test_web_position_half(self):
        boundary, margin = self.frame.web_centerline(0.5, skin_margin_mm=1.0, slot_width_mm=0.6)
        self.assertEqual(boundary, (10.0, 20.0))
        self.assertEqual(margin, (10.0, 1.0))

    def test_web_position_one_with_nonzero_width_is_off_the_chord(self):
        # Mirror of the leading-tip case above, at the trailing tip.
        with self.assertRaises(ValueError):
            self.frame.web_centerline(1.0, skin_margin_mm=1.0, slot_width_mm=0.6)

    def test_web_position_near_one_with_room_for_the_slot_width(self):
        boundary, margin = self.frame.web_centerline(0.98, skin_margin_mm=1.0, slot_width_mm=0.6)
        self.assertEqual(boundary, (19.6, 20.0))
        self.assertEqual(margin, (19.6, 1.0))

    def test_margin_subtraction_is_from_the_far_skin(self):
        # depth 20, margin 3 -> the far end sits 3 mm above the -Y edge,
        # i.e. at y = min_y + margin = 3.0, NOT at y = max_y - margin's
        # complement measured some other way.
        _, margin = self.frame.web_centerline(0.5, skin_margin_mm=3.0, slot_width_mm=0.6)
        self.assertEqual(margin[1], 3.0)

    def test_margin_leaving_no_room_raises(self):
        with self.assertRaises(ValueError):
            self.frame.web_centerline(0.5, skin_margin_mm=20.0, slot_width_mm=0.6)

    def test_worst_case_equals_point_probe_on_a_rectangle(self):
        # Zero slope everywhere on a rectangle: the worst case across the
        # slot's width must equal a plain point probe at the centreline, so the
        # worst-case machinery does not change results for a plain coupon.
        boundary, margin = self.frame.web_centerline(0.5, skin_margin_mm=1.0, slot_width_mm=5.0)
        self.assertEqual(boundary, (10.0, 20.0))
        self.assertEqual(margin, (10.0, 1.0))


if __name__ == "__main__":
    unittest.main()
