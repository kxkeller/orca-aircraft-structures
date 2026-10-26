"""The spar-socket latch as a walk over precomputed per-layer statuses
(``aircraft_core.spar_latch``) plus the generator's own ``spar_channel_status``
classifier.

The regression this pins: on the tip-down outside elevon half the online,
bottom-up latch met the channel's TIP-side end first and suppressed the rod
socket over the whole rod-reachable spar (78 ``spar_socket`` then 833
``spar_socket_latched``).
"""
import unittest

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import (
    SPAR_STATUS_ABSENT as A,
    SPAR_STATUS_FIT as F,
    SPAR_STATUS_NOT_FIT as N,
    DetectedChannel,
    LayerContext,
    StructureGenerator,
)
from aircraft_core.spar_latch import walk_latch, walk_order
from aircraft_core.spec import StructureSpec

from stub_ops import Rect, StubOps


class WalkOrderTests(unittest.TestCase):
    def test_bed_root_walks_up_top_root_walks_down(self):
        self.assertEqual(walk_order(4, True), [0, 1, 2, 3])
        self.assertEqual(walk_order(4, False), [3, 2, 1, 0])


class WalkLatchTests(unittest.TestCase):
    def test_tip_down_thin_tip_end_arms_late_and_suppresses_nothing(self):
        # bed = tip. Layers 0-1: no channel yet; 2-4: section too thin;
        # 5-11: fits (toward the root at the top). The rod-reachable spar
        # is 5-11 and must keep its socket.
        st = [A, A, N, N, N, F, F, F, F, F, F, F]
        w = walk_latch(st, root_at_bed=False)
        self.assertEqual(w["arm_index"], 4)
        self.assertEqual(w["suppressed"], [])
        self.assertEqual(w["fit_layers_before_arm"], 7)

    def test_the_same_statuses_read_bed_first_reproduce_the_defect(self):
        # Reading the same statuses bed-first reproduces the defect: the
        # online latch meets the thin end first, arms, and suppresses the
        # whole spar.
        st = [A, A, N, N, N, F, F, F, F, F, F, F]
        w = walk_latch(st, root_at_bed=True)
        self.assertEqual(w["arm_index"], 2)
        self.assertEqual(w["suppressed"], [5, 6, 7, 8, 9, 10, 11])

    def test_root_at_bed_mirror_image_is_correct(self):
        st = [F, F, F, F, F, F, F, N, N, N, A, A]  # root at bed, thin tip end
        w = walk_latch(st, root_at_bed=True)
        self.assertEqual(w["arm_index"], 7)
        self.assertEqual(w["suppressed"], [])

    def test_resume_after_an_end_is_suppressed_tipward_of_it_top_root(self):
        #        tip(bed)                                 root(top)
        st = [F, F, F, N, F, F, F, F]
        w = walk_latch(st, root_at_bed=False)
        self.assertEqual(w["arm_index"], 3)
        self.assertEqual(w["suppressed"], [0, 1, 2])
        self.assertEqual(w["suppressed_first"], 0)
        self.assertEqual(w["suppressed_last"], 2)

    def test_absent_never_arms_and_leaves_the_latch_alone(self):
        # a model-defined gap is not a rod stop.
        st = [F, F, A, A, F, F]
        for root_at_bed in (True, False):
            w = walk_latch(st, root_at_bed)
            self.assertIsNone(w["arm_index"])
            self.assertEqual(w["suppressed"], [])
        st = [F, N, A, F, F]
        w = walk_latch(st, True)
        self.assertEqual(w["suppressed"], [3, 4])  # absent layer is not a "fit"

    def test_neutral_none_layers_neither_arm_nor_resume(self):
        st = [None, F, None, N, None, F]
        self.assertEqual(walk_latch(st, True)["suppressed"], [5])
        self.assertEqual(walk_latch(st, False)["suppressed"], [1])

    def test_never_armed_run_suppresses_nothing(self):
        self.assertEqual(walk_latch([F] * 5, True)["suppressed"], [])
        self.assertIsNone(walk_latch([F] * 5, False)["arm_index"])


class SparChannelStatusTests(unittest.TestCase):
    """``StructureGenerator.spar_channel_status`` -- the generator's OWN fit
    check, not a copy."""

    def _status(self, height_mm, channels, **spec_kw):
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, Rect(0.0, 0.0, 40.0, height_mm))
        kw = dict(slot_gap_mm=0.6, skin_margin_mm=1.0, spar_clearance_mm=0.15,
                  spar_neck_width_mm=1.0)
        kw.update(spec_kw)
        spec = StructureSpec(**kw)
        layer = LayerContext(layer_index=3, print_z=1.0, detected_channels=tuple(channels))
        return StructureGenerator().spar_channel_status(frame, spec, layer, 0.42)

    CH = DetectedChannel(center_x_mm=20.0, center_y_mm=6.0, diameter_mm=4.0)

    def test_fit_needs_diameter_clearance_and_skin_margins(self):
        # needed = 2*(2.0+0.15) + 2*1.0 = 6.3
        self.assertEqual(self._status(12.0, [self.CH]), F)
        self.assertEqual(self._status(6.4, [DetectedChannel(20.0, 3.2, 4.0)]), F)
        self.assertEqual(self._status(6.2, [DetectedChannel(20.0, 3.1, 4.0)]), N)

    def test_no_channel_is_absent(self):
        self.assertEqual(self._status(12.0, []), A)

    def test_several_channels_under_refuse_are_absent_not_armed(self):
        two = [self.CH, DetectedChannel(30.0, 6.0, 3.0)]
        self.assertEqual(self._status(12.0, two), A)

    def test_unconfirmed_nearest_station_pick_never_arms(self):
        # Even a pick that cannot fit reads absent: its degradation kind is
        # spar_socket_unconfirmed, which never arms the latch.
        two = [DetectedChannel(20.0, 3.1, 4.0), DetectedChannel(30.0, 3.1, 3.0)]
        self.assertEqual(
            self._status(6.2, two, spar_channel_policy="nearest_spar_station"), A)

    def test_confirmed_channel_too_small_for_a_keyhole_is_absent(self):
        tiny = DetectedChannel(20.0, 6.0, 0.4)  # generate() would refuse it
        self.assertEqual(
            self._status(12.0, [tiny], spar_neck_width_mm=2.0), A)


if __name__ == "__main__":
    unittest.main()
