"""Root-end resolution (``aircraft_core.root_end``).

The outside (tip) half of an elevon wing can print TIP
FACE DOWN, so the plugin cannot assume the root is at the bed. These tests use
small synthetic tapered meshes (vertex clouds are all the AREA signal reads)
and synthetic per-layer channel data, on both placements.
"""
import unittest

from aircraft_core.root_end import (
    AREA_DECISIVE_RATIO,
    CHANNEL_GAP_DECISIVE_FRACTION,
    RootEndError,
    area_extremes,
    area_signal,
    channel_presence_from_diameters,
    channel_span_signal,
    resolve_root_end,
)


def _frustum(root_z_is_low, span=100.0, root_w=40.0, tip_w=8.0, depth=6.0):
    """Vertex cloud of a straight-tapered rectangular frustum, root wide,
    tip narrow. ``root_z_is_low`` True = root at Z=0 (root on the bed)."""
    verts = []
    n = 51
    for i in range(n):
        z = span * i / (n - 1)
        frac = z / span
        w = root_w + (tip_w - root_w) * (frac if root_z_is_low else 1.0 - frac)
        for sx in (-1, 1):
            for sy in (-1, 1):
                verts.append((sx * w / 2.0, sy * depth / 2.0, z))
    return verts


def _layer_axis(span=100.0, n=500):
    return [span * (i + 0.5) / n for i in range(n)]


def _channel_flags(zs, lo, hi):
    return [lo <= z <= hi for z in zs]


class AreaSignalTests(unittest.TestCase):
    def test_root_at_bed_reads_low(self):
        sig = area_signal(_frustum(root_z_is_low=True))
        self.assertEqual(sig["end"], "low")
        self.assertGreater(sig["area_low_end_mm2"], sig["area_high_end_mm2"])

    def test_root_at_top_reads_high(self):
        sig = area_signal(_frustum(root_z_is_low=False))
        self.assertEqual(sig["end"], "high")
        self.assertGreater(sig["area_high_end_mm2"], sig["area_low_end_mm2"])

    def test_constant_section_is_indeterminate_not_a_guess(self):
        sig = area_signal(_frustum(True, root_w=20.0, tip_w=20.0))
        self.assertIsNone(sig["end"])

    def test_just_under_the_decisive_ratio_is_indeterminate(self):
        w_hi = 20.0
        w_lo = w_hi / (AREA_DECISIVE_RATIO * 0.98)
        sig = area_signal(_frustum(True, root_w=w_lo, tip_w=w_hi))
        self.assertIsNone(sig["end"])

    def test_extremes_match_the_preflight_heuristic_inputs(self):
        m = area_extremes(_frustum(True))
        self.assertEqual(m["z_min"], 0.0)
        self.assertAlmostEqual(m["band_mm"], 2.0)  # 2% of 100 mm


class ChannelSignalTests(unittest.TestCase):
    """Elevon wing numbers: 249.5 mm span, channel 15.2-199.6 mm from the
    root, i.e. abutting the root end, 50 mm short of the tip."""

    def test_channel_abutting_low_end(self):
        zs = _layer_axis(249.5)
        sig = channel_span_signal(_channel_flags(zs, 15.2, 199.6), zs, 0.0, 249.5)
        self.assertEqual(sig["end"], "low")
        self.assertAlmostEqual(sig["gap_to_low_end_mm"], 15.2, delta=0.6)

    def test_same_channel_seen_tip_down_abuts_the_high_end(self):
        zs = _layer_axis(249.5)
        flags = _channel_flags(zs, 249.5 - 199.6, 249.5 - 15.2)
        sig = channel_span_signal(flags, zs, 0.0, 249.5)
        self.assertEqual(sig["end"], "high")

    def test_centred_channel_is_indeterminate(self):
        zs = _layer_axis(100.0)
        sig = channel_span_signal(_channel_flags(zs, 20.0, 80.0), zs, 0.0, 100.0)
        self.assertIsNone(sig["end"])

    def test_no_channel_is_indeterminate(self):
        zs = _layer_axis(100.0)
        sig = channel_span_signal([False] * len(zs), zs, 0.0, 100.0)
        self.assertIsNone(sig["end"])
        self.assertEqual(sig["channel_layers"], 0)

    def test_alignment_pins_do_not_vote_as_the_spar(self):
        # 3 mm pins at both ends of a 10.5 mm spar channel that abuts the
        # low end: by raw presence the "channel" would span everything and
        # be indeterminate; by diameter it is the spar.
        zs = _layer_axis(100.0)
        dias = []
        for z in zs:
            if z < 3.0 or z > 97.0:
                dias.append(3.0)
            elif z < 70.0:
                dias.append(10.5)
            else:
                dias.append(None)
        sig = channel_span_signal(channel_presence_from_diameters(dias), zs, 0.0, 100.0)
        self.assertEqual(sig["end"], "low")
        self.assertLess(sig["channel_z_last_mm"], 71.0)

    def test_fraction_constant_is_the_documented_value(self):
        self.assertEqual(CHANNEL_GAP_DECISIVE_FRACTION, 0.05)


class ResolutionTests(unittest.TestCase):
    def _signals(self, root_low, channel_low):
        zs = _layer_axis(100.0)
        area = area_signal(_frustum(root_low))
        if channel_low:
            flags = _channel_flags(zs, 6.0, 70.0)
        else:
            flags = _channel_flags(zs, 30.0, 94.0)
        return area, channel_span_signal(flags, zs, 0.0, 100.0)

    def test_auto_agree_root_at_bed(self):
        d = resolve_root_end("auto", *self._signals(True, True))
        self.assertEqual((d["root_end"], d["root_at"]), ("low", "bed"))
        self.assertEqual(d["decision_source"], "auto_area_and_channel_agree")
        self.assertTrue(d["signals_agree"])

    def test_auto_agree_root_at_top_the_tip_down_placement(self):
        d = resolve_root_end("auto", *self._signals(False, False))
        self.assertEqual((d["root_end"], d["root_at"]), ("high", "top"))
        self.assertEqual(d["decision_source"], "auto_area_and_channel_agree")

    def test_auto_disagreement_fails_closed_naming_both_signals(self):
        area, chan = self._signals(True, False)  # area: low; channel: high
        with self.assertRaises(RootEndError) as cm:
            resolve_root_end("auto", area, chan)
        msg = str(cm.exception)
        self.assertIn("AREA says the root is the low Z end", msg)
        self.assertIn("CHANNEL says the root is the high Z end", msg)
        self.assertIn("root_end", msg)
        self.assertIn('"bed"', msg)
        self.assertIn('"top"', msg)

    def test_override_resolves_a_disagreement_and_is_flagged(self):
        area, chan = self._signals(True, False)
        d = resolve_root_end("top", area, chan)
        self.assertEqual(d["root_end"], "high")
        self.assertEqual(d["decision_source"], "config_override")
        self.assertEqual(d["override_contradicts_signal"], ["area"])
        d = resolve_root_end("bed", area, chan)
        self.assertEqual(d["root_end"], "low")
        self.assertEqual(d["override_contradicts_signal"], ["channel"])

    def test_auto_one_signal_decides(self):
        zs = _layer_axis(100.0)
        none_chan = channel_span_signal([False] * len(zs), zs, 0.0, 100.0)
        d = resolve_root_end("auto", area_signal(_frustum(False)), none_chan)
        self.assertEqual((d["root_end"], d["decision_source"]),
                         ("high", "auto_area_only"))
        flat = area_signal(_frustum(True, root_w=20.0, tip_w=20.0))
        chan_hi = channel_span_signal(_channel_flags(zs, 30.0, 94.0), zs, 0.0, 100.0)
        d = resolve_root_end("auto", flat, chan_hi)
        self.assertEqual((d["root_end"], d["decision_source"]),
                         ("high", "auto_channel_only"))

    def test_auto_no_signal_keeps_the_historical_bed_default(self):
        zs = _layer_axis(100.0)
        flat = area_signal(_frustum(True, root_w=20.0, tip_w=20.0))
        none_chan = channel_span_signal([False] * len(zs), zs, 0.0, 100.0)
        d = resolve_root_end("auto", flat, none_chan)
        self.assertEqual((d["root_end"], d["root_at"]), ("low", "bed"))
        self.assertEqual(d["decision_source"], "default_bed_no_signal")

    def test_unknown_mode_is_refused(self):
        with self.assertRaises(RootEndError):
            resolve_root_end("tip", {"end": None}, {"end": None})


if __name__ == "__main__":
    unittest.main()
