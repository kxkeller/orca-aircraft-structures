"""Pins ``depth_to_socket``, which derives a lattice member's depth from
THIS layer's detected carbon-spar socket instead of a fixed
``web_depth_mm``, so a brace can reach and fuse to the socket regardless of
how the socket-to-skin clearance varies along the span (a fixed depth falls
short at one end of the wing and nowhere else: e.g. 3.890 mm at z=2 versus
2.180 mm at z=118, a 1.71 mm range).

``StructureGenerator`` reads the flag defensively via ``getattr(family,
"depth_to_socket", False)``, and the tests here set the attribute directly on
a frozen ``WebFamily`` instance with ``object.__setattr__``. Every assertion
is written against the generator's behaviour, not against how the flag gets
onto the family.
"""
import unittest

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import DetectedChannel, LayerContext, StructureGenerator
from aircraft_core.spec import StructureSpec, WebFamily

from stub_ops import Rect, StubOps


def _depth_to_socket(family: WebFamily) -> WebFamily:
    """Stand-in for the not-yet-existing ``WebFamily(depth_to_socket=True)``
    -- see this module's own docstring."""
    object.__setattr__(family, "depth_to_socket", True)
    return family


class DepthToSocketTests(unittest.TestCase):
    """20x20 mm coupon, a single ``brace`` family entering from the LOWER
    skin (``entry_lower=True``, so ``start_y=0``), referencing a
    model-detected socket via ``LayerContext.detected_channels``."""

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

    def _brace_family(self):
        return _depth_to_socket(
            WebFamily(name="brace", web_position=0.5, entry_lower=True)
        )

    def test_a_depth_equals_the_gap_to_the_socket_circle_less_skin_margin(self):
        """Channel centred at y=10, diameter 1.7 mm -> r = 0.85 + 0.15 =
        1.0 mm, circle bottom (the -Y-entry near boundary) at y=9.0. Gap
        from the lower skin (y=0) to that boundary is 9.0 mm; less
        skin_margin_mm=1.0 -> depth 8.0 mm exactly."""
        channel = DetectedChannel(
            center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7
        )
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(self._brace_family(),),
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )
        result = self._generate(spec, channels=(channel,))
        brace = [i for i in result.intents if i.family_name == "brace"]
        self.assertEqual(len(brace), 1)
        self.assertAlmostEqual(brace[0].start[1], 0.0, places=6)
        self.assertAlmostEqual(brace[0].end[1], 8.0, places=6)

    def test_b_depth_follows_the_sockets_own_centre_and_diameter(self):
        """own determinism property, restated for the derived depth:
        moving the SAME channel's centre up by 4 mm (10 -> 14) with the same
        diameter must move the computed depth up by exactly 4 mm.

        Both centres (10 and 14) keep ``_generate_spar_socket_from_channel``
        choosing the UPPER entry (``enter_lower`` is false whenever
        ``cy >= 10`` on this 20 mm-tall coupon), i.e. the OPPOSITE side from
        this test's lower-entry brace, so the socket's neck never runs
        through the brace's own half of the section -- a centre below 10
        would flip the socket to a lower entry and put the brace physically
        inside the neck, which pins nothing about depth-following and is a
        degenerate fixture in its own right."""
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(self._brace_family(),),
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )
        moved_spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(self._brace_family(),),
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )
        base = self._generate(
            spec,
            channels=(DetectedChannel(
                center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7
            ),),
        )
        moved = self._generate(
            moved_spec,
            channels=(DetectedChannel(
                center_x_mm=10.0, center_y_mm=14.0, diameter_mm=1.7
            ),),
        )
        base_end_y = [i.end[1] for i in base.intents if i.family_name == "brace"][0]
        moved_end_y = [i.end[1] for i in moved.intents if i.family_name == "brace"][0]
        self.assertAlmostEqual(moved_end_y - base_end_y, 4.0, places=6)

    def test_c_two_layers_with_different_clearance_produce_different_depths(self):
        """The fixed-depth failure this feature exists to fix, pinned
        directly: two layers (distinct ``LayerContext``s, as at different Z)
        with different detected sockets must NOT produce the same depth.
        Both centres keep the socket on the upper entry, same reasoning as
        test (b)."""
        spec_a = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(self._brace_family(),),
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )
        spec_b = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(self._brace_family(),),
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )
        result_a = self._generate(
            spec_a,
            channels=(DetectedChannel(
                center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7
            ),),
            layer_index=0, print_z=0.0,
        )
        result_b = self._generate(
            spec_b,
            channels=(DetectedChannel(
                center_x_mm=10.0, center_y_mm=14.0, diameter_mm=1.7
            ),),
            layer_index=1, print_z=1.0,
        )
        depth_a = [i.end[1] for i in result_a.intents if i.family_name == "brace"][0]
        depth_b = [i.end[1] for i in result_b.intents if i.family_name == "brace"][0]
        self.assertNotAlmostEqual(depth_a, depth_b, places=6)

    def test_d_no_socket_detected_is_omitted_not_built_at_web_depth_mm(self):
        """A stray ``web_depth_mm`` set alongside the flag must never be used
        as a silent fallback when there is nothing to measure the socket
        -relative depth against."""
        family = self._brace_family()
        object.__setattr__(family, "web_depth_mm", 3.0)
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(family,),
        )
        result = self._generate(spec, channels=())
        self.assertEqual(
            [i for i in result.intents if i.family_name == "brace"], []
        )
        omitted = [o for o in result.omitted if o.family_name == "brace"]
        self.assertEqual(len(omitted), 1)
        self.assertIn("depth_to_socket", omitted[0].reason)
        self.assertNotIn("web_depth_mm=3", omitted[0].reason)

    def test_e_same_side_entry_is_refused_before_any_depth_is_computed(self):
        """A socket sitting close to the brace's entry skin is ALWAYS on that
        same skin, so the same-side refusal catches it first.

        The "no positive depth" reason is not reachable here:
        ``_generate_spar_socket`` enters whichever skin the channel is
        NEARER, so a socket close to the brace's entry skin necessarily has its
        own neck on that skin, and a brace there would lie inside the neck and
        vanish in the boolean -- a silent no-op, which is the worse failure and
        is refused first.

        The "no positive depth" branch is consequently unreachable through the
        public API: for OPPOSITE-side entry the remaining gap is at least
        ``skin_margin_mm`` whenever the socket fits at all, because the
        socket's own fit check already demands
        ``2r + 2 * skin_margin <= section_depth``. It is kept as defence in
        depth, not as live behaviour."""
        channel = DetectedChannel(
            center_x_mm=10.0, center_y_mm=1.0, diameter_mm=1.0
        )
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(self._brace_family(),),
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )
        result = self._generate(spec, channels=(channel,))
        self.assertEqual(
            [i for i in result.intents if i.family_name == "brace"], []
        )
        omitted = [o for o in result.omitted if o.family_name == "brace"]
        self.assertEqual(len(omitted), 1)
        self.assertIn("depth_to_socket", omitted[0].reason)
        self.assertIn("same side", omitted[0].reason)
        self.assertIn("vanish in the boolean", omitted[0].reason)

    def test_f_a_socket_referenced_member_is_not_omitted_by_the_footprint_exclusion(self):
        """Pins the exemption's own arithmetic and that ``references_socket``
        is actually set -- NOT a regression test of the exemption's effect:
        ``_generate_family`` alone never runs the exclusion loop (that code
        is inline in ``generate()``, after the family loop), so this test
        would pass unchanged even with the exemption deleted from
        ``generate()``. See ``test_f_regression_...`` below for the test
        that actually exercises (and fails without) the exemption.

        ``StructureSpec.validate`` requires ``skin_margin_mm > 2 *
        safety_offset_mm`` STRICTLY (a 0.021 mm cushion),
        which means a ``depth_to_socket`` member's tip -- placed exactly
        ``skin_margin_mm`` short of the circle -- can never sit EXACTLY on the
        exclusion band's own edge (``cy - r - min_wall``) through the public
        ``generate()`` entry point; there is always at least a hair of
        daylight. So this test drives ``_generate_family`` directly (bypassing
        ``spec.validate()``, the same isolation ``test_thin_section.py``
        already uses for the same reason) with ``skin_margin_mm`` set to
        EXACTLY ``2 * safety_offset_mm`` -- forcing the tip onto the band's
        edge precisely -- and then applies the exclusion band's own inequality
        directly (not the real ``generate()`` code path) to confirm the fixture
        actually reaches the boundary it claims to (``my_hi >= y_lo``)."""
        safety_offset_mm = 0.42
        min_wall_mm = 2.0 * safety_offset_mm
        channel = DetectedChannel(
            center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7
        )
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=min_wall_mm,
            families=(self._brace_family(),),
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )
        layer = LayerContext(layer_index=0, print_z=0.0, detected_channels=(channel,))
        # this returns (intent_or_None, degradation_or_None).
        socket, socket_degradation = (
            StructureGenerator()._generate_spar_socket_from_channel(
                self.frame, spec, layer, channel,
                # Unambiguous: exactly one channel, no disambiguation.
                spar_confirmed=True,
            )
        )
        self.assertIsNone(socket_degradation)
        self.assertIsNotNone(socket)
        family = spec.effective_families()[0]
        intents, omitted = StructureGenerator()._generate_family(
            self.frame,
            spec,
            family,
            0,
            safety_offset_mm,
            layer,
            safety_offset_mm,
            min_wall_mm,
            socket,
        )
        brace = [i for i in intents if i.family_name == "brace"]
        self.assertEqual(len(brace), 1)
        self.assertTrue(brace[0].references_socket)
        self.assertEqual(omitted, [])

        # The exclusion band's own inequality (mirrors StructureGenerator
        # .generate()'s socket-footprint-exclusion block exactly), applied
        # directly to prove this specific member would have been caught
        # without the exemption.
        cy, r = socket.end[1], socket.socket_radius_mm
        y_lo = min(socket.start[1], cy - r) - min_wall_mm
        my_hi = max(brace[0].start[1], brace[0].end[1])
        self.assertGreaterEqual(
            my_hi, y_lo,
            "test setup failed to reach the band edge -- this member would "
            "not have been caught by the exclusion even without the "
            "exemption, so it does not actually test the exemption",
        )

    def test_f_regression_exemption_reaches_generate(self):
        """The test that actually exercises the exemption inside
        ``generate()``'s own exclusion code, unlike (f) above.

        Bypasses ONLY ``StructureSpec.validate``'s cushion check (via
        ``unittest.mock.patch.object``, passing ``safety_offset_mm=None``
        through to the real ``validate`` so every OTHER field-level rule still
        runs) so ``skin_margin_mm`` can equal ``2 * safety_offset_mm`` exactly
        and the brace's tip lands precisely on the exclusion band's edge, THEN
        calls the real, public ``StructureGenerator.generate()`` -- the actual
        code path that carries the exemption. Without the
        ``if getattr(i, "references_socket"...)`` exemption block in
        ``generate()`` this test fails: the brace is omitted with a
        "spar socket footprint" reason."""
        from unittest import mock

        safety_offset_mm = 0.42
        min_wall_mm = 2.0 * safety_offset_mm
        channel = DetectedChannel(
            center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7
        )
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=min_wall_mm,
            families=(self._brace_family(),),
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )
        orig_validate = StructureSpec.validate
        with mock.patch.object(
            StructureSpec,
            "validate",
            lambda self, safety_offset_mm=None: orig_validate(
                self, safety_offset_mm=None
            ),
        ):
            result = self._generate(
                spec, channels=(channel,),
            )
        brace = [i for i in result.intents if i.family_name == "brace"]
        self.assertEqual(len(brace), 1)
        self.assertEqual(
            [o for o in result.omitted if o.family_name == "brace"], []
        )

    def test_f_control_a_socket_referenced_member_is_not_omitted_by_the_footprint_exclusion(self):
        """Companion to (f) above at realistic, valid-config numbers (goes
        through the full public ``generate()``, unlike (f)): confirms the
        ordinary case also survives, even though -- per (f)'s own docstring
        -- the public API's minimum cushion means this specific band test
        was never actually going to catch it. Exists so a future change
        that widens the exclusion band (e.g. rounds ``min_wall`` up) has a
        realistic-numbers regression test, not only the engineered edge
        case."""
        channel = DetectedChannel(
            center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7
        )
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(self._brace_family(),),
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )
        result = self._generate(spec, channels=(channel,))
        brace = [i for i in result.intents if i.family_name == "brace"]
        self.assertEqual(len(brace), 1)
        self.assertTrue(brace[0].references_socket)
        self.assertEqual(
            [o for o in result.omitted if o.family_name == "brace"], []
        )

    def test_g_a_family_without_the_flag_is_unaffected(self):
        """The default (``depth_to_socket`` absent/False) reaches ``web_depth_mm``
        exactly as a family without the flag always does -- pins one concrete case against a
        detected socket at the same station, so a regression that made the flag
        "sticky" (fire for every family, not just flagged ones) would be caught
        here."""
        ordinary = WebFamily(
            name="ordinary", web_position=0.5, web_depth_mm=2.0, entry_lower=True
        )
        channel = DetectedChannel(
            center_x_mm=10.0, center_y_mm=10.0, diameter_mm=1.7
        )
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(ordinary,),
            spar_clearance_mm=0.15, spar_neck_width_mm=1.0,
        )
        result = self._generate(spec, channels=(channel,))
        built = [i for i in result.intents if i.family_name == "ordinary"]
        self.assertEqual(len(built), 1)
        self.assertAlmostEqual(built[0].end[1], 2.0, places=6)
        self.assertFalse(built[0].references_socket)


if __name__ == "__main__":
    unittest.main()
