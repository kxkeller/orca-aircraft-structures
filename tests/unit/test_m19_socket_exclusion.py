"""Pins that the spar-socket footprint exclusion in
``StructureGenerator.generate`` is two-dimensional.

The exclusion band (the block guarded by ``if socket is not None:``, search
"spar socket footprint" in ``generator.py``) cannot test chordwise (X) overlap
alone. The socket's X footprint (``2 * socket_radius_mm``) is much wider than a
lattice web, so any station overlap is a collision for the socket's CIRCLE,
but the socket is a keyhole: a NECK also runs from the entry skin to the
circle centre, and a member can share the neck's X station while sitting
entirely on the far side of the circle from the neck -- e.g. a short member
entering from the skin OPPOSITE the socket's own entry, too shallow to reach
anywhere near the circle. An X-only test would omit such a member anyway,
making an entire chord band unusable for any lower-skin-to-spar coupling
member.

The exclusion therefore ANDs the X test with a Y test against the keyhole's
bounding box: ``[min(start_y, cy - r), max(start_y, cy + r)]``, both inflated
by the same ``min_wall`` the X test uses. Using the NECK's full run (not just
the circle) is required for correctness: a member on the SAME side as the
neck, within the neck's reach but outside the circle, still collides and must
still be omitted -- exercised by ``SparSocketFootprintExclusionTests`` in
``test_m17_spar_socket.py``.
"""
import unittest

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import DetectedChannel, LayerContext, StructureGenerator
from aircraft_core.spec import StructureSpec, WebFamily

from stub_ops import Rect, StubOps


class TwoDimensionalSocketExclusionTests(unittest.TestCase):
    """20x20 mm coupon, config-declared 2.0 mm spar at station 0.5 (fallback
    path): socket centred at x=10, y=10, radius 1.15 mm, entering from the
    UPPER skin (``+Y``, ``entry_lower=False`` -- the fallback midpoint always
    resolves this way). The socket's own extent is therefore x in [8.85, 11.15]
    (neck+circle X band) and its Y bounding box is [8.85, 20.0] (circle bottom
    up through the neck to the top entry skin).
    """

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def _generate(self, spec):
        return StructureGenerator().generate(
            self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        )

    def _spar_spec(self, family):
        return StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            families=(family,),
            spar_diameter_mm=2.0,
            spar_station=0.5,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
            allow_config_spar_fallback=True,
        )

    def test_a_short_member_below_the_socket_is_built_not_omitted(self):
        """The defect case: a short lower-skin-entry member (depth 2.0 mm,
        entering at y=0) sits entirely below the socket's circle bottom
        (8.85 mm, less ``min_wall``), which is also below the neck (the
        neck is on the far, UPPER side here) -- it cannot collide, and must
        be built."""
        family = WebFamily(
            name="test", web_position=0.5, web_depth_mm=2.0, entry_lower=True
        )
        result = self._generate(self._spar_spec(family))
        built_xs = {round(i.start[0], 4) for i in result.intents if i.family_name == "test"}
        omitted_xs = {round(o.x_mm, 4) for o in result.omitted if o.family_name == "test"}
        self.assertIn(10.0, built_xs)
        self.assertNotIn(10.0, omitted_xs)

    def test_b_a_member_whose_depth_reaches_the_socket_is_still_omitted(self):
        """Same station, but deep enough (9.0 mm from the lower skin) to
        reach into the inflated circle band (bottom at 10 - 1.15 - 0.84 =
        8.01 mm) -- still a real collision, still omitted, same reason."""
        family = WebFamily(
            name="test", web_position=0.5, web_depth_mm=9.0, entry_lower=True
        )
        result = self._generate(self._spar_spec(family))
        built_xs = {round(i.start[0], 4) for i in result.intents if i.family_name == "test"}
        omitted = [o for o in result.omitted if o.family_name == "test"]
        omitted_xs = {round(o.x_mm, 4) for o in omitted}
        self.assertIn(10.0, omitted_xs)
        self.assertNotIn(10.0, built_xs)
        self.assertIn("spar socket footprint", omitted[0].reason)

    def test_c_same_depth_but_chordwise_clear_is_built(self):
        """Pins that the X test still gates independently: pitch-4
        neighbours of the same deep (9.0 mm) family, clear of the socket's
        X band, are built even though the Y test alone would not exclude
        them."""
        family = WebFamily(
            name="test", web_position=0.5, web_depth_mm=9.0, entry_lower=True,
            rib_pitch_mm=4.0,
        )
        result = self._generate(self._spar_spec(family))
        built_xs = {round(i.start[0], 4) for i in result.intents if i.family_name == "test"}
        self.assertIn(6.0, built_xs)
        self.assertIn(14.0, built_xs)

    def test_d_a_member_above_a_lower_entry_socket_is_built(self):
        """Symmetric case to (a): a MODEL-detected channel low in the
        section (centre y=4.0, diameter 1.7 mm, so r = 1.7/2 + 0.15 =
        1.0 mm), so the socket enters from the LOWER skin and its neck
        runs from y=0 up to y=4. The inflated Y band is therefore
        [-0.84, 5.84] (circle top 5.0, plus min_wall 0.84). A short
        (3.0 mm) UPPER-skin member at the same station sits at y=[17, 20],
        entirely above that band and above the neck's own reach (the neck
        is on the far, LOWER side here), so it cannot collide and must be
        built."""
        channel = DetectedChannel(
            center_x_mm=10.0, center_y_mm=4.0, diameter_mm=2.0 - 2 * 0.15
        )
        family = WebFamily(
            name="test", web_position=0.5, web_depth_mm=3.0, entry_lower=False
        )
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            families=(family,),
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
        )
        ctx = LayerContext(layer_index=0, print_z=0.0, detected_channels=(channel,))
        result = StructureGenerator().generate(self.frame, spec, 0.42, ctx)
        built_xs = {round(i.start[0], 4) for i in result.intents if i.family_name == "test"}
        omitted_xs = {round(o.x_mm, 4) for o in result.omitted if o.family_name == "test"}
        self.assertIn(10.0, built_xs)
        self.assertNotIn(10.0, omitted_xs)


if __name__ == "__main__":
    unittest.main()
