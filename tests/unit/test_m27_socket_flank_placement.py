"""Pins ``WebFamily.socket_flank_offset_mm``/``socket_flank_direction``, which
derive a family's SINGLE chord station from THIS layer's detected carbon-spar
socket instead of ``web_position``/``rib_pitch_chord_frac``. The placement
formula (shared with ``tools/m25_socket_flank_search.py``) is
``x = channel.center_x_mm + sign * (radius + offset)`` where ``radius = channel.diameter_mm / 2.0`` -- the model's own bare
channel radius, NOT the clearance-inflated ``socket.socket_radius_mm`` -- and
``sign`` is -1 for "forward" and +1 for "aft".

``gap_ratio=0.05`` below matches the base config's probe: a wide slot
would push the offset needed to clear the socket-footprint exclusion band
(which DOES use the clearance-inflated radius, ``spar_clearance_mm`` default
0.15, plus ``2 * safety_offset_mm``) past the 1.005 mm offset measured
for this geometry, so the narrow gap is load-bearing for these numbers, not
cosmetic.

Both fields are real ``WebFamily`` fields, so every test here constructs a real
``WebFamily(socket_flank_offset_mm=..., socket_flank_direction=...)`` directly.
"""
import unittest

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import (
    DetectedChannel,
    LayerContext,
    StructureGenerator,
    WebIntent,
)
from aircraft_core.spec import SpecError, StructureSpec, WebFamily

from stub_ops import Rect, StubOps


class SocketFlankPlacementTests(unittest.TestCase):
    """20x20 mm coupon, a single ``flank`` family, upper entry
    (``entry_lower=False``, the default)."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def _generate(self, spec, channels=(), layer_index=0, print_z=0.0):
        return StructureGenerator().generate(
            self.frame, spec, 0.42,
            LayerContext(
                layer_index=layer_index, print_z=print_z,
                detected_channels=channels,
            ),
        )

    def _flank_family(self, offset_mm, direction, entry_lower=False):
        return WebFamily(
            name="flank", entry_lower=entry_lower,
            socket_flank_offset_mm=offset_mm,
            socket_flank_direction=direction,
        )

    def _spec(self, families):
        return StructureSpec(gap_ratio=0.05, skin_margin_mm=1.0, families=families)

    # -- (a) lands at the expected distance from the detected circle edge --

    def test_a_station_lands_at_the_expected_distance_past_the_circle_edge_aft(self):
        """Channel centred at x=10, diameter=1.7 -> r=0.85. offset=1.005 mm
        aft (toward +X/trailing edge) -> expected station
        10 + (0.85 + 1.005) = 11.855 mm exactly."""
        channel = DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7)
        spec = self._spec((self._flank_family(1.005, "aft"),))
        result = self._generate(spec, channels=(channel,))
        flank = [i for i in result.intents if i.family_name == "flank"]
        self.assertEqual(len(flank), 1)
        self.assertAlmostEqual(flank[0].start[0], 11.855, places=6)
        self.assertAlmostEqual(flank[0].end[0], 11.855, places=6)

    def test_a_station_lands_at_the_expected_distance_past_the_circle_edge_forward(self):
        """Same channel, offset=1.005 mm forward (toward -X/leading edge) ->
        expected station 10 - (0.85 + 1.005) = 8.145 mm exactly."""
        channel = DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7)
        spec = self._spec((self._flank_family(1.005, "forward"),))
        result = self._generate(spec, channels=(channel,))
        flank = [i for i in result.intents if i.family_name == "flank"]
        self.assertEqual(len(flank), 1)
        self.assertAlmostEqual(flank[0].start[0], 8.145, places=6)

    # -- (b) same family, different socket positions -> different, tracked
    #    stations: the regression that proves this is model-derived --

    def test_b_same_family_tracks_two_different_socket_positions(self):
        """The SAME family (frozen web_position=0.5 default, unread) run
        against two layers whose detected sockets sit at different X must
        produce two DIFFERENT stations, each equal to that layer's own
        channel formula -- not the same station twice (which would mean the
        placement had silently fallen back to a fixed fraction)."""
        family = self._flank_family(1.005, "aft")
        spec = self._spec((family,))
        result_a = self._generate(
            spec,
            channels=(DetectedChannel(center_x_mm=6.0, center_y_mm=10.0, diameter_mm=1.7),),
            layer_index=0, print_z=0.0,
        )
        result_b = self._generate(
            spec,
            channels=(DetectedChannel(center_x_mm=11.0, center_y_mm=10.0, diameter_mm=1.7),),
            layer_index=1, print_z=1.0,
        )
        x_a = [i.start[0] for i in result_a.intents if i.family_name == "flank"][0]
        x_b = [i.start[0] for i in result_b.intents if i.family_name == "flank"][0]
        self.assertAlmostEqual(x_a, 6.0 + 1.855, places=6)
        self.assertAlmostEqual(x_b, 11.0 + 1.855, places=6)
        self.assertNotAlmostEqual(x_a, x_b, places=6)
        # And the two stations differ by exactly the channel centres' own
        # separation -- the station TRACKS the socket, not some unrelated
        # per-layer drift.
        self.assertAlmostEqual(x_b - x_a, 5.0, places=6)

    def test_b2_station_tracks_the_channels_own_radius_not_just_its_centre(self):
        """: every OTHER test in this file uses diameter_mm=1.7, so a formula with
        the 0.85 mm radius HARD -CODED (instead of read from
        ``channel.diameter_mm``) would pass all of them, including test (b)
        above, which only proves the station tracks ``center_x_mm`` -- never
        ``diameter_mm``. Same centre (x=10), two different diameters (1.7 mm,
        the fixture's own spar; 10.0 mm, closer to a real carbon tube) -- the
        station must move by EXACTLY the two radii's own difference, (10.0 -
        1.7) / 2 = 4.15 mm, nothing else changing."""
        family = self._flank_family(1.005, "aft")
        spec = self._spec((family,))
        result_thin = self._generate(
            spec,
            channels=(DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7),),
        )
        result_thick = self._generate(
            spec,
            channels=(DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=10.0),),
        )
        x_thin = [i.start[0] for i in result_thin.intents if i.family_name == "flank"][0]
        x_thick = [i.start[0] for i in result_thick.intents if i.family_name == "flank"][0]
        self.assertAlmostEqual(x_thin, 11.855, places=6)
        self.assertAlmostEqual(x_thick, 16.005, places=6)
        self.assertAlmostEqual(x_thick - x_thin, 4.15, places=6)

    # -- (c) fail closed when no channel is detected --

    def test_c_no_channel_detected_is_omitted_not_a_chord_fraction_fallback(self):
        """No socket this layer -> the family is omitted, loudly, naming the
        cause, never silently placed at web_position or any other chord
        fraction."""
        family = self._flank_family(1.005, "aft")
        spec = self._spec((family,))
        result = self._generate(spec, channels=())
        self.assertEqual(
            [i for i in result.intents if i.family_name == "flank"], []
        )
        omitted = [o for o in result.omitted if o.family_name == "flank"]
        self.assertEqual(len(omitted), 1)
        self.assertIn("socket_flank", omitted[0].reason)
        self.assertIn("no socket channel was detected", omitted[0].reason)

    # -- (d) feature off (unset/0) leaves existing behaviour byte-identical --

    def test_d_offset_unset_is_byte_identical_to_an_ordinary_family(self):
        channel = DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7)
        ordinary = WebFamily(name="flank", entry_lower=False, web_position=0.5)
        unset = WebFamily(name="flank", entry_lower=False, web_position=0.5,
                           socket_flank_offset_mm=None,
                           socket_flank_direction=None)
        result_ordinary = self._generate(self._spec((ordinary,)), channels=(channel,))
        result_unset = self._generate(self._spec((unset,)), channels=(channel,))
        self.assertEqual(result_ordinary.intents, result_unset.intents)
        self.assertEqual(result_ordinary.omitted, result_unset.omitted)

    def test_d_offset_zero_is_byte_identical_to_an_ordinary_family(self):
        """Explicit 0 (as opposed to None) is documented as ALSO meaning
        "feature off" -- pinned separately since it takes the
        ``> 0.0`` branch of the activation check, not the ``is not None``
        one."""
        channel = DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7)
        ordinary = WebFamily(name="flank", entry_lower=False, web_position=0.5)
        zeroed = WebFamily(name="flank", entry_lower=False, web_position=0.5,
                            socket_flank_offset_mm=0.0)
        result_ordinary = self._generate(self._spec((ordinary,)), channels=(channel,))
        result_zeroed = self._generate(self._spec((zeroed,)), channels=(channel,))
        self.assertEqual(result_ordinary.intents, result_zeroed.intents)
        self.assertEqual(result_ordinary.omitted, result_zeroed.omitted)

    # -- entry side still comes from the existing field, not a new one --

    def test_e_entry_side_still_comes_from_entry_lower(self):
        channel = DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7)
        spec = self._spec((self._flank_family(1.005, "aft", entry_lower=True),))
        result = self._generate(spec, channels=(channel,))
        flank = [i for i in result.intents if i.family_name == "flank"]
        self.assertEqual(len(flank), 1)
        self.assertTrue(flank[0].entry_lower)
        self.assertAlmostEqual(flank[0].start[1], 0.0, places=6)

    # -- validate() cross-field rules --

    def test_f_negative_offset_is_rejected(self):
        family = WebFamily(name="flank", socket_flank_offset_mm=-1.0,
                            socket_flank_direction="aft")
        with self.assertRaises(SpecError):
            family.validate(0)

    def test_f_active_offset_without_a_valid_direction_is_rejected(self):
        family = WebFamily(name="flank", socket_flank_offset_mm=1.005,
                            socket_flank_direction=None)
        with self.assertRaises(SpecError):
            family.validate(0)
        family_bad = WebFamily(name="flank", socket_flank_offset_mm=1.005,
                                socket_flank_direction="sideways")
        with self.assertRaises(SpecError):
            family_bad.validate(0)

    def test_f_direction_set_without_an_active_offset_is_rejected(self):
        family = WebFamily(name="flank", socket_flank_offset_mm=None,
                            socket_flank_direction="aft")
        with self.assertRaises(SpecError):
            family.validate(0)
        family_zero = WebFamily(name="flank", socket_flank_offset_mm=0.0,
                                 socket_flank_direction="aft")
        with self.assertRaises(SpecError):
            family_zero.validate(0)

    def test_f_active_offset_with_rib_pitch_mm_is_rejected(self):
        """, PREFERRED FIX: an active offset combined with pitch would still march
        ``_candidate_stations`` across the whole chord from the socket-derived
        anchor (``generator.py``'s own call is unconditional on pitch), landing
        extra stations that can fall inside the socket's own footprint
        exclusion and vanish silently -- exactly the "SINGLE station" promise
        the field's own docstring/schema description makes. Refused at
        validate() instead of left to disagree with the docs."""
        family = WebFamily(name="flank", socket_flank_offset_mm=1.005,
                            socket_flank_direction="aft", rib_pitch_mm=5.0)
        with self.assertRaises(SpecError):
            family.validate(0)

    def test_f_active_offset_with_rib_pitch_chord_frac_is_rejected(self):
        family = WebFamily(name="flank", socket_flank_offset_mm=1.005,
                            socket_flank_direction="aft",
                            rib_pitch_chord_frac=0.2)
        with self.assertRaises(SpecError):
            family.validate(0)

    def test_f_active_offset_alone_still_validates(self):
        """Control for the two tests above: the offset/direction pair alone
        (no pitch) must NOT be rejected by the new guard."""
        family = WebFamily(name="flank", socket_flank_offset_mm=1.005,
                            socket_flank_direction="aft")
        family.validate(0)  # must not raise

    # -- (g) the operating point is tight, by design -- pin it directly --

    def test_g_the_measured_1_005mm_offset_clears_the_socket_footprint_exclusion(self):
        """: at the reference parameters (gap_ratio=0.05, spar_clearance_mm=0.15
        [StructureSpec default], safety_offset_mm/wall=0.42 -- the exact
        numbers measured), 1.005 mm past the bare channel edge clears the
        socket-footprint exclusion band by only ~0.0045 mm. That margin is the
        design, not slack -- pin it directly so a change to any of these
        constants (or to the exclusion band's own formula) is caught here
        rather than surfacing as an unexplained member vanishing from a real
        multi-family layer."""
        channel = DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7)
        spec = self._spec((self._flank_family(1.005, "aft"),))
        result = self._generate(spec, channels=(channel,))
        flank = [i for i in result.intents if i.family_name == "flank"]
        self.assertEqual(len(flank), 1,
                          "1.005 mm must clear the socket-footprint "
                          "exclusion at the reference parameters")
        self.assertEqual(
            [o for o in result.omitted if o.family_name == "flank"], [],
        )

    # -- fail-open edge: a caller that bypasses validate() must still not
    #    silently default an invalid direction to "aft" --

    def test_h_generate_family_raises_on_invalid_direction_bypassing_validate(self):
        """: the prior version's ``direction_sign = -1.0 if... == "forward" else
        1.0`` treated ``None`` (or any typo) as "aft" silently.
        ``WebFamily.validate()`` catches this through the public API, but
        ``_generate_family`` is also called directly by other tests in this
        codebase (see ``test_m19_depth_to_socket.py``'s own precedent), so the
        method itself must not have a fail-open default. Constructed via
        ``object.__setattr__`` to plant an invalid direction on an
        otherwise-valid family without going through ``validate()`` at all."""
        channel = DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7)
        family = WebFamily(name="flank", socket_flank_offset_mm=1.005,
                            socket_flank_direction="aft")
        object.__setattr__(family, "socket_flank_direction", None)
        layer = LayerContext(layer_index=0, print_z=0.0,
                              detected_channels=(channel,))
        spec = StructureSpec(gap_ratio=0.05, skin_margin_mm=1.0)
        # The socket_flank branch requires a CONFIRMED spar socket
        # (``WebIntent.spar_confirmed``) before it reaches the direction
        # validation this test targets -- a ``None`` socket is refused first
        # (as "not a CONFIRMED spar"), which is a different
        # code path than the one under test here. A minimal confirmed socket
        # matching the channel keeps this test exercising the invalid-direction
        # raise specifically.
        socket = WebIntent(
            start=(10.0, 20.0), end=(10.0, 10.0), width_mm=1.0,
            family_index=-2, family_name="spar_socket",
            socket_radius_mm=0.85, spar_confirmed=True,
        )
        with self.assertRaises(ValueError):
            StructureGenerator()._generate_family(
                self.frame, spec, family, 0, 0.42, layer, 0.42, 1.0, socket,
            )

    def test_g_control_1_000mm_offset_is_still_inside_the_exclusion_band(self):
        """Companion control: 1.000 mm (5 microns less) is STILL caught by
        the exclusion at these exact parameters -- proves the pin above is
        actually testing the tight boundary, not a wide-open one."""
        channel = DetectedChannel(center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7)
        spec = self._spec((self._flank_family(1.000, "aft"),))
        result = self._generate(spec, channels=(channel,))
        flank = [i for i in result.intents if i.family_name == "flank"]
        self.assertEqual(flank, [])
        omitted = [o for o in result.omitted if o.family_name == "flank"]
        self.assertEqual(len(omitted), 1)
        self.assertIn("spar socket footprint", omitted[0].reason)


if __name__ == "__main__":
    unittest.main()
