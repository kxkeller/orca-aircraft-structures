"""Fidelity tests for ``fake_orca.ExPolygon.intersection_ex`` against the
strip-probe contract ``aircraft_core.frame.local_span`` actually exercises
(via ``OrcaGeometryOps.intersection`` -> ``ExPolygon.intersection_ex``,
mirroring the real ``PluginHostGeometry.cpp:211-213`` binding).

These tests exercise the fake's internal shape kernel directly
(``fake_orca._Rect``/``_Notched``/``ExPolygon._wrap``) rather than going
through the generated bundle, so each strip-probe case can be checked
exactly — including the two genuinely-disjoint-pieces case the real host's
caller (``local_span``) is written to raise on rather than average.

Nothing here modifies the code under test — it only reads fake_orca.py.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fake_orca  # noqa: E402


def rect(min_x, min_y, max_x, max_y):
    return fake_orca.ExPolygon._wrap(fake_orca._Rect(min_x, min_y, max_x, max_y))


def notched(outer_bounds, notch_bounds):
    outer = fake_orca._Rect(*outer_bounds)
    notch = fake_orca._Rect(*notch_bounds)
    return fake_orca.ExPolygon._wrap(fake_orca._Notched(outer=outer, notch=notch))


class RectIntersectionTests(unittest.TestCase):
    """``rect ∩ strip`` — the trivial base case."""

    def test_disjoint_strip_yields_empty_list(self):
        subject = rect(0, 0, 10, 10)
        strip = rect(20, -5, 21, 15)
        self.assertEqual(subject.intersection_ex(strip), [])

    def test_overlapping_strip_yields_one_rect_clipped_to_the_overlap(self):
        subject = rect(0, 0, 10, 10)
        strip = rect(4, -5, 6, 15)
        pieces = subject.intersection_ex(strip)
        self.assertEqual(len(pieces), 1)
        self.assertEqual(fake_orca.describe(pieces[0]), {"kind": "rect", "bounds": (4, 0, 6, 10)})


class NotchedIntersectionTests(unittest.TestCase):
    """``notched ∩ strip`` — the shape ``local_span`` actually probes a
    slotted section with. Outer = (0,0,20,20), notch = (8,10,12,20): a
    top-touching notch from x=8..12, y=10..20, leaving a bridge of height 10
    below it and two 8-mm-wide arms on either side."""

    OUTER = (0, 0, 20, 20)
    NOTCH = (8, 10, 12, 20)

    def make(self):
        return notched(self.OUTER, self.NOTCH)

    def test_strip_entirely_outside_the_outer_bounds_is_empty(self):
        subject = self.make()
        strip = rect(30, -5, 31, 25)
        self.assertEqual(subject.intersection_ex(strip), [])

    def test_strip_entirely_in_the_left_arm_is_one_rect_full_height(self):
        subject = self.make()
        strip = rect(2, -5, 4, 25)
        pieces = subject.intersection_ex(strip)
        self.assertEqual(len(pieces), 1)
        self.assertEqual(fake_orca.describe(pieces[0]), {"kind": "rect", "bounds": (2, 0, 4, 20)})

    def test_strip_entirely_in_the_right_arm_is_one_rect_full_height(self):
        subject = self.make()
        strip = rect(15, -5, 17, 25)
        pieces = subject.intersection_ex(strip)
        self.assertEqual(len(pieces), 1)
        self.assertEqual(fake_orca.describe(pieces[0]), {"kind": "rect", "bounds": (15, 0, 17, 20)})

    def test_strip_entirely_inside_the_notch_column_yields_only_the_bridge(self):
        # Strip's X range is fully within [8, 12] (the notch column) and its
        # Y range envelops the whole section (as local_span's padded probe
        # always does) -- above the bridge is air, so only the bridge
        # material (y in [0, 10]) survives.
        subject = self.make()
        strip = rect(9, -5, 11, 25)
        pieces = subject.intersection_ex(strip)
        self.assertEqual(len(pieces), 1)
        self.assertEqual(fake_orca.describe(pieces[0]), {"kind": "rect", "bounds": (9, 0, 11, 10)})

    def test_strip_spanning_left_arm_and_bridge_is_one_connected_l_shape(self):
        # Strip's X range [6, 10] crosses into the left arm (x<8) and the
        # notch column (x>=8) but stops short of the right arm (x<12).
        subject = self.make()
        strip = rect(6, -5, 10, 25)
        pieces = subject.intersection_ex(strip)
        self.assertEqual(len(pieces), 1)
        described = fake_orca.describe(pieces[0])
        self.assertEqual(described["kind"], "notched")
        # Combined outer spans the clipped X range at full (0, 20) height;
        # the notch cavity above the bridge is only over the [8, 10] column.
        self.assertEqual(described["outer"], (6, 0, 10, 20))
        self.assertEqual(described["notch"], (8, 10, 10, 20))

    def test_strip_spanning_right_arm_and_bridge_is_one_connected_l_shape(self):
        subject = self.make()
        strip = rect(10, -5, 14, 25)
        pieces = subject.intersection_ex(strip)
        self.assertEqual(len(pieces), 1)
        described = fake_orca.describe(pieces[0])
        self.assertEqual(described["kind"], "notched")
        self.assertEqual(described["outer"], (10, 0, 14, 20))
        self.assertEqual(described["notch"], (10, 10, 12, 20))

    def test_strip_spanning_both_arms_and_bridge_is_one_connected_u_shape(self):
        # Strip wide enough to include material on both sides of the notch
        # AND the bridge: still one connected piece.
        subject = self.make()
        strip = rect(2, -5, 18, 25)
        pieces = subject.intersection_ex(strip)
        self.assertEqual(len(pieces), 1)
        described = fake_orca.describe(pieces[0])
        self.assertEqual(described["kind"], "notched")
        self.assertEqual(described["outer"], (2, 0, 18, 20))
        self.assertEqual(described["notch"], (8, 10, 12, 20))

    def test_strip_spanning_both_arms_but_missing_the_bridge_is_two_disjoint_pieces(self):
        # Strip's Y range starts above the bridge line (y=10..25), so within
        # the notch column there is no material at all to connect the two
        # arms -- they are genuinely disjoint.
        subject = self.make()
        strip = rect(2, 12, 18, 25)
        pieces = subject.intersection_ex(strip)
        self.assertEqual(len(pieces), 2)
        described = sorted((fake_orca.describe(p) for p in pieces), key=lambda d: d["bounds"])
        self.assertEqual(
            described,
            [
                {"kind": "rect", "bounds": (2, 12, 8, 20)},
                {"kind": "rect", "bounds": (12, 12, 18, 20)},
            ],
        )

    def test_bridge_material_case_matches_local_span_expectations(self):
        """Sanity check mirroring what ``local_span`` itself would compute:
        bounds() of the single surviving piece gives the local (y_min, y_max)
        at this chord station."""
        subject = self.make()
        strip = rect(9, -100, 11, 100)  # local_span-style, hugely padded in Y
        pieces = subject.intersection_ex(strip)
        self.assertEqual(len(pieces), 1)
        bounds = fake_orca.describe(pieces[0])["bounds"]
        y_min, y_max = bounds[1], bounds[3]
        self.assertEqual((y_min, y_max), (0, 10))


class UnsupportedShapeTests(unittest.TestCase):
    """A notched CLIP (as opposed to a notched subject) is outside this
    fake's documented scope and must raise loudly, not guess."""

    def test_notched_clip_raises_not_implemented(self):
        subject = rect(0, 0, 20, 20)
        clip = notched((0, 0, 20, 20), (8, 10, 12, 20))
        with self.assertRaises(NotImplementedError):
            subject.intersection_ex(clip)


if __name__ == "__main__":
    unittest.main()
