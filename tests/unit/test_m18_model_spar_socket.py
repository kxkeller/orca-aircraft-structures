"""The MODEL is the source of truth for spar-socket geometry; config only
dictates policy.

Covers the precedence ``StructureGenerator.generate`` implements between
a ``LayerContext.detected_channels`` entry (the model) and
``StructureSpec.spar_diameter_mm``/``spar_station`` (config, a debugging
fallback only, gated on ``allow_config_spar_fallback``):

- ``DetectedChannelDrivesSocketTests``: a single detected channel drives the
  socket's centre and radius, not ``spar_station``/``spar_diameter_mm``.
- ``MovingTheChannelMovesTheSocketTests``: the core determinism
  property -- shifting the detected centroid shifts the socket by exactly
  the same amount.
- ``ChangingDiameterChangesRadiusTests``: the socket radius tracks the
  detected diameter (plus policy clearance), not any config value.
- ``TwoChannelsFailClosedTests``: more than one detected channel is a POLICY
  question the model alone cannot answer -- ``TopologyError``, naming the
  layer and every candidate channel.
- ``NoChannelNoFallbackFailsClosedTests``: no channel detected, a config
  spar declared, ``allow_config_spar_fallback`` left at its default
  (``False``) -- fails closed naming the flag.
- ``NoChannelFallbackReproducesM17Tests``: the same case with the fallback
  explicitly enabled reproduces the config-only socket, byte-for-byte,
  and records the fact on ``GenerationResult.spar_fallback_used``.
- ``DetectedChannelBeatsConfigTests``: a channel AND config spar values both
  present -- the config values are demonstrably not consulted (a config
  diameter that could never fit this section does not raise, because it is
  never used).
"""
import unittest

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import DetectedChannel, LayerContext, StructureGenerator
from aircraft_core.spec import StructureSpec
from aircraft_core.validate import TopologyError

from stub_ops import Rect, StubOps


def _coupon_frame():
    ops = StubOps()
    rect = Rect(0.0, 0.0, 20.0, 20.0)
    return LayerFrame.from_polygon(ops, rect)


def _socket_intent(result):
    sockets = [i for i in result.intents if i.family_name == "spar_socket"]
    assert len(sockets) == 1, "expected exactly one spar_socket intent"
    return sockets[0]


class DetectedChannelDrivesSocketTests(unittest.TestCase):
    def test_socket_centre_and_radius_come_from_the_detected_channel(self):
        frame = _coupon_frame()
        channel = DetectedChannel(center_x_mm=8.0, center_y_mm=11.0, diameter_mm=2.0)
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
            # Deliberately unrelated config values -- must be ignored.
            spar_diameter_mm=None,
            spar_station=0.9,
        )
        layer = LayerContext(layer_index=0, print_z=0.0, detected_channels=(channel,))
        result = StructureGenerator().generate(frame, spec, 0.42, layer)
        socket = _socket_intent(result)

        self.assertEqual(socket.end, (8.0, 11.0))
        self.assertAlmostEqual(socket.socket_radius_mm, 2.0 / 2.0 + 0.15)
        self.assertFalse(result.spar_fallback_used)


class MovingTheChannelMovesTheSocketTests(unittest.TestCase):
    def test_shifting_the_centroid_shifts_the_socket_by_the_same_amount(self):
        frame = _coupon_frame()
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0,
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )

        channel_a = DetectedChannel(center_x_mm=8.0, center_y_mm=11.0, diameter_mm=2.0)
        channel_b = DetectedChannel(center_x_mm=11.0, center_y_mm=13.0, diameter_mm=2.0)

        result_a = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0, detected_channels=(channel_a,))
        )
        result_b = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0, detected_channels=(channel_b,))
        )

        socket_a = _socket_intent(result_a)
        socket_b = _socket_intent(result_b)

        dx = socket_b.end[0] - socket_a.end[0]
        dy = socket_b.end[1] - socket_a.end[1]
        self.assertAlmostEqual(dx, channel_b.center_x_mm - channel_a.center_x_mm)
        self.assertAlmostEqual(dy, channel_b.center_y_mm - channel_a.center_y_mm)


class ChangingDiameterChangesRadiusTests(unittest.TestCase):
    def test_radius_tracks_the_detected_diameter_plus_clearance(self):
        frame = _coupon_frame()
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0,
            spar_clearance_mm=0.2, spar_neck_width_mm=1.0,
        )

        small = DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=2.0)
        large = DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=4.0)

        r_small = _socket_intent(
            StructureGenerator().generate(
                frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0, detected_channels=(small,))
            )
        ).socket_radius_mm
        r_large = _socket_intent(
            StructureGenerator().generate(
                frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0, detected_channels=(large,))
            )
        ).socket_radius_mm

        self.assertAlmostEqual(r_small, 2.0 / 2.0 + 0.2)
        self.assertAlmostEqual(r_large, 4.0 / 2.0 + 0.2)


class TwoChannelsFailClosedTests(unittest.TestCase):
    def test_two_detected_channels_raise_naming_both_and_the_layer(self):
        frame = _coupon_frame()
        spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)
        channel_0 = DetectedChannel(center_x_mm=5.0, center_y_mm=10.0, diameter_mm=2.0)
        channel_1 = DetectedChannel(center_x_mm=15.0, center_y_mm=10.0, diameter_mm=3.0)
        layer = LayerContext(
            layer_index=12, print_z=2.4, detected_channels=(channel_0, channel_1)
        )

        with self.assertRaises(TopologyError) as ctx:
            StructureGenerator().generate(frame, spec, 0.42, layer)

        err = ctx.exception
        self.assertEqual(err.layer_index, 12)
        self.assertAlmostEqual(err.print_z, 2.4)
        msg = str(err)
        self.assertIn("layer 12", msg)
        self.assertIn("5.0000", msg)
        self.assertIn("15.0000", msg)
        self.assertIn("2.0000", msg)
        self.assertIn("3.0000", msg)
        self.assertIn("POLICY", msg)


class NoChannelNoFallbackFailsClosedTests(unittest.TestCase):
    def test_config_spar_without_fallback_flag_fails_closed(self):
        frame = _coupon_frame()
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            spar_diameter_mm=2.0,
            spar_station=0.5,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
            # allow_config_spar_fallback left at its default (False).
        )
        layer = LayerContext(layer_index=3, print_z=0.6)

        with self.assertRaises(TopologyError) as ctx:
            StructureGenerator().generate(frame, spec, 0.42, layer)

        err = ctx.exception
        self.assertEqual(err.layer_index, 3)
        self.assertIn("allow_config_spar_fallback", str(err))


class NoChannelFallbackReproducesM17Tests(unittest.TestCase):
    def test_fallback_enabled_reproduces_the_m17_config_only_socket(self):
        frame = _coupon_frame()
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            spar_diameter_mm=2.0,
            spar_station=0.5,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
            allow_config_spar_fallback=True,
        )
        layer = LayerContext(layer_index=0, print_z=0.0)

        result = StructureGenerator().generate(frame, spec, 0.42, layer)
        socket = _socket_intent(result)

        # Pinned concrete geometric value: x = min_x + spar_station * width
        # = 0.0 + 0.5 * 20.0 = 10.0; radius = 1.0 + 0.15 = 1.15; the 20x20
        # coupon centres the socket at y = 10.0.
        self.assertAlmostEqual(socket.end[0], 10.0)
        self.assertAlmostEqual(socket.end[1], 10.0)
        self.assertAlmostEqual(socket.socket_radius_mm, 1.15)
        self.assertTrue(result.spar_fallback_used)


class DetectedChannelBeatsConfigTests(unittest.TestCase):
    def test_detected_channel_wins_even_with_config_values_present(self):
        frame = _coupon_frame()
        # A config spar this large could never fit the 20x20 coupon (needed
        # depth = 2*(25.0+0.15) + 2*1.0 = 52.3 mm > 20 mm) -- if the config
        # path were consulted at all, generation would fail closed. It must
        # not be: the detected channel takes precedence, and the config
        # values are never touched.
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            spar_diameter_mm=50.0,
            spar_station=0.5,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
            allow_config_spar_fallback=False,
        )
        channel = DetectedChannel(center_x_mm=8.0, center_y_mm=11.0, diameter_mm=2.0)
        layer = LayerContext(layer_index=0, print_z=0.0, detected_channels=(channel,))

        result = StructureGenerator().generate(frame, spec, 0.42, layer)
        socket = _socket_intent(result)

        self.assertEqual(socket.end, (8.0, 11.0))
        self.assertAlmostEqual(socket.socket_radius_mm, 2.0 / 2.0 + 0.15)
        self.assertFalse(result.spar_fallback_used)


if __name__ == "__main__":
    unittest.main()
