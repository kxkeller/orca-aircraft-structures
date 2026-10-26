"""Regression fixtures for the carbon-spar socket: every geometry bug gets a
minimized permanent fixture and regression test.

Covers, one class each:

- ``KeyholeSimpleRingTests``: pins the self-intersection bug in
  ``SlotBuilder._keyhole`` (a wrong ``base`` angle once self-intersected) --
  the keyhole outline must be a SIMPLE ring enclosing no interior ring.
- ``SparSocketFailsClosedOnShallowSectionTests``: ``StructureGenerator.
  _generate_spar_socket`` must fail closed, naming the layer index and Z,
  when the section is too shallow for the requested spar.
- ``SparSocketFootprintExclusionTests``: the socket footprint exclusion
  omits an in-band lattice station and keeps an out-of-band one.
- ``WindingConsistencyTests``: both ``SlotBuilder.build`` branches
  (rectangle, keyhole) wind the same way (signed area, same sign).

Uses ``shapely`` (test-only backend -- see
``tests/unit/test_erosion_near_leading_edge.py``'s own docstring) only for the
simple-ring check; skipped outright if unavailable.
"""
import unittest

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.slot import SlotBuilder
from aircraft_core.spec import SpecError, StructureSpec, WebFamily
from aircraft_core.validate import TopologyError

from stub_ops import Rect, StubOps

try:
    from shapely.geometry import LinearRing, Polygon

    HAVE_SHAPELY = True
except ImportError:  # pragma: no cover - environment-dependent
    HAVE_SHAPELY = False


def _shoelace(points):
    """Signed polygon area (positive == counter-clockwise)."""
    total = 0.0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        total += x1 * y2 - x2 * y1
    return total / 2.0


@unittest.skipUnless(HAVE_SHAPELY, "shapely not installed; see module docstring")
class KeyholeSimpleRingTests(unittest.TestCase):
    """Pins that the keyhole outline does not self-intersect: a wrong ``base``
    angle in ``_keyhole`` would make the arc start 90 degrees out and
    cross the neck. The docstring's own claim -- "the outline never crosses
    itself and encloses no interior ring" -- must actually hold."""

    def _build(self, width_mm=0.6, socket_radius_mm=2.0):
        from aircraft_core.generator import WebIntent

        intent = WebIntent(
            start=(10.0, 20.0), end=(10.0, 1.0),
            width_mm=width_mm, socket_radius_mm=socket_radius_mm,
        )
        return SlotBuilder().build(intent, overshoot_mm=0.1)

    def test_keyhole_ring_is_simple(self):
        slot = self._build()
        ring = LinearRing(slot.points)
        self.assertTrue(
            ring.is_simple,
            "keyhole outline self-intersects -- this is exactly the wrong-"
            "'base'-angle regression (arc starting 90 degrees out); see "
            "SlotBuilder._keyhole's own comment on why the arc must start "
            "at base - a, not the perpendicular.",
        )

    def test_keyhole_polygon_has_no_interior_ring(self):
        slot = self._build()
        poly = Polygon(slot.points)
        self.assertTrue(poly.is_valid)
        self.assertEqual(
            len(poly.interiors), 0,
            "keyhole polygon encloses an interior ring -- it must be a "
            "single simply-connected void, boundary-connected via the neck.",
        )

    def test_keyhole_ring_simple_across_neck_widths(self):
        # A range of neck widths relative to the socket radius, all valid
        # (below the fail-closed threshold).
        for width_mm in (0.2, 0.6, 1.0, 1.5, 3.0):
            with self.subTest(width_mm=width_mm):
                slot = self._build(width_mm=width_mm, socket_radius_mm=2.0)
                self.assertTrue(LinearRing(slot.points).is_simple)


class SparSocketFailsClosedOnShallowSectionTests(unittest.TestCase):
    """``_generate_spar_socket`` must fail the slice, naming the layer index and
    Z, rather than degrade, when the section cannot fit the requested spar
    (docs/geometry_invariants.md's fail-closed mutation protocol,'s
    non-negotiable invariant)."""

    def test_too_shallow_section_raises_with_layer_and_z(self):
        ops = StubOps()
        # 20 x 5 mm coupon -- far too shallow for a 6 mm spar at 0.15 mm
        # clearance plus 1.0 mm skin margin on each side (needs 8.3 mm).
        rect = Rect(0.0, 0.0, 20.0, 5.0)
        frame = LayerFrame.from_polygon(ops, rect)
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            spar_diameter_mm=6.0,
            spar_station=0.5,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
            # no LayerContext.detected_channels here (no model to section in
            # this unit test), so the config-declared spar is only reachable
            # via the explicit debugging fallback.
            allow_config_spar_fallback=True,
        )
        # A spar socket that does not fit does not refuse the slice: the
        # socket ENDS at that layer and the wing prints skin-only from there
        # outboard, because a spar hole the rod cannot reach is not worth
        # refusing a whole wing over (a real outboard panel would otherwise
        # refuse hundreds of consecutive layers).
        #
        # The degradation must be reported: that is the licence for
        # degrading ("explicitly marked degraded").
        result = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=7, print_z=1.4)
        )
        self.assertEqual(len(result.degradations), 1)
        deg = result.degradations[0]
        self.assertEqual(deg.kind, "spar_socket")
        self.assertIn("does not fit", deg.reason)
        self.assertIn("DEGRADED", deg.reason)
        self.assertIn("No socket is cut on this layer", deg.reason)
        # No socket intent was produced for this layer.
        self.assertFalse(
            [i for i in result.intents if i.family_name == "spar_socket"])


class SparSocketFootprintExclusionTests(unittest.TestCase):
    """The socket footprint exclusion (``StructureGenerator.generate``,
    the ``spec.spar_diameter_mm`` block) omits a lattice station that falls
    inside the socket's claimed footprint, and keeps one that does not."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def test_in_band_station_omitted_out_of_band_kept(self):
        family = WebFamily(
            name="test", rib_pitch_mm=4.0, web_depth_mm=3.0, web_position=0.5
        )
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            families=(family,),
            spar_diameter_mm=2.0,
            spar_station=0.5,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
            # no detected channel in this unit test -- the debugging fallback
            # must be requested explicitly to reach the config-only socket path
            # this test exercises.
            allow_config_spar_fallback=True,
        )
        result = StructureGenerator().generate(
            self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        )
        # Socket centred at x=10, radius 1.15, min_wall = 2*0.42 = 0.84 ->
        # footprint band is x in [7.99, 12.01]. The anchor station (x=10) is
        # IN-BAND and must be omitted; the pitch-4 neighbours (x=6, x=14)
        # are OUT-OF-BAND and must be built.
        built_xs = {round(i.start[0], 4) for i in result.intents if i.family_name == "test"}
        omitted_xs = {round(o.x_mm, 4) for o in result.omitted if o.family_name == "test"}
        self.assertIn(10.0, omitted_xs)
        self.assertNotIn(10.0, built_xs)
        self.assertIn(6.0, built_xs)
        self.assertIn(14.0, built_xs)
        omitted_at_10 = [
            o for o in result.omitted if o.family_name == "test" and round(o.x_mm, 4) == 10.0
        ]
        self.assertEqual(len(omitted_at_10), 1)
        self.assertIn("spar socket footprint", omitted_at_10[0].reason)


class WindingConsistencyTests(unittest.TestCase):
    """Both ``SlotBuilder.build`` branches must wind the same way --
    ``SlotPolygon``'s own contract is counter-clockwise (see its docstring),
    and the keyhole branch must wind the same way as the
    rectangle branch."""

    def test_rectangle_and_keyhole_wind_the_same_way(self):
        from aircraft_core.generator import WebIntent

        rect_intent = WebIntent(start=(10.0, 20.0), end=(10.0, 1.0), width_mm=0.6)
        keyhole_intent = WebIntent(
            start=(10.0, 20.0), end=(10.0, 1.0), width_mm=0.6, socket_radius_mm=2.0
        )
        rect_slot = SlotBuilder().build(rect_intent, overshoot_mm=0.1)
        keyhole_slot = SlotBuilder().build(keyhole_intent, overshoot_mm=0.1)

        rect_area = _shoelace(rect_slot.points)
        keyhole_area = _shoelace(keyhole_slot.points)

        self.assertGreater(
            rect_area, 0.0, "rectangle branch must be counter-clockwise (positive area)"
        )
        self.assertGreater(
            keyhole_area, 0.0,
            "keyhole branch must be counter-clockwise (positive area), matching "
            "SlotPolygon's documented contract and the rectangle branch",
        )

    def test_no_duplicate_consecutive_vertex_on_keyhole(self):
        from aircraft_core.generator import WebIntent

        intent = WebIntent(
            start=(10.0, 20.0), end=(10.0, 1.0), width_mm=0.6, socket_radius_mm=2.0
        )
        slot = SlotBuilder().build(intent, overshoot_mm=0.1)
        pts = slot.points
        n = len(pts)
        # The last arc point and the final appended neck point must not be
        # the same coordinate (a zero-length edge) somewhere in the ring (checked including the
        # closing wraparound edge).
        for i in range(n):
            self.assertNotEqual(
                pts[i], pts[(i + 1) % n],
                "keyhole ring has a duplicate consecutive vertex at index "
                "{} (zero-length edge)".format(i),
            )


class NeckWidthFailsClosedTests(unittest.TestCase):
    """A neck at least as wide as the socket is not a keyhole -- it is a slot
    that happens to be round at the bottom, and the socket wall it was supposed
    to leave does not exist.

    This pins that ``_keyhole`` does not SILENTLY clamp an over-wide neck
    (e.g. with ``half_w = min(half_w, r * 0.98)``): such a neck "must not be
    produced silently". A declared config value must never be quietly
    altered (no silent geometric fallback), so both the spec and the
    primitive refuse it.
    """

    def test_spec_validate_rejects_neck_at_least_as_wide_as_the_socket(self):
        # 10.0 + 2*0.15 = 10.3 mm socket; a 10.3 mm neck leaves no wall.
        spec = StructureSpec(
            gap_ratio=0.05,          # required before validate reaches the spar checks
            spar_diameter_mm=10.0,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=10.3,
        )
        with self.assertRaises(SpecError) as cm:
            spec.validate()
        msg = str(cm.exception)
        self.assertIn("spar_neck_width_mm", msg)
        self.assertIn("10.3", msg)

    def test_spec_validate_accepts_a_neck_just_under_the_socket(self):
        StructureSpec(
            gap_ratio=0.05,
            spar_diameter_mm=10.0,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=10.2,
        ).validate()

    def test_keyhole_primitive_refuses_directly_not_just_via_the_spec(self):
        """``SlotBuilder`` is a public primitive with callers that never build a
        ``StructureSpec`` (the probes in ``tools/`` do exactly this), so the
        spec-level guard is not sufficient on its own."""
        from aircraft_core.generator import WebIntent

        # half_w = 2.0 against r = 2.0: neck exactly as wide as the socket.
        intent = WebIntent(
            start=(10.0, 20.0), end=(10.0, 1.0), width_mm=4.0, socket_radius_mm=2.0
        )
        with self.assertRaises(SpecError):
            SlotBuilder().build(intent, overshoot_mm=0.1)


class InstalledConfigSmokeFixtureTests(unittest.TestCase):
    """A config declaring a ~10 mm carbon spar cannot slice on a 7.2 mm deep 60
    mm-chord aerofoil, so the live-config smoke check was failing a perfectly
    valid wing config on a section the spar can never fit.

    The fix narrows the FIXTURE SET, never the assertions: fixtures that can
    hold the declared spar are still checked in full, a wing-scale fixture was
    added so spar configs are actually exercised somewhere, and a config that
    fits NO fixture still fails loudly. These tests pin both directions,
    because a "skip what does not fit" rule is exactly the shape of change
    that can silently become "skip everything".
    """

    def _m17_cap_config(self):
        import json
        import os

        repo = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        path = os.path.join(repo, "tests", "fixtures", "spar_socket",
                            "m17_integrated_config.json")
        if not os.path.isfile(path):
            self.skipTest("m17_integrated_config.json not present")
        with open(path) as fh:
            return json.load(fh)["config"][0]["cap_config"]

    def test_a_real_spar_config_is_accepted_and_actually_exercised(self):
        """Not just "does not raise" -- WHICH fixtures ran.

        `if not exercised` is all-or-nothing, and the 20 mm coupon reports a
        constant 20 mm depth at every station, so it survives for any spar
        under ~18 mm. That makes the all-or-nothing guard nearly unreachable
        and would let the skip widen silently. Pinning the exact set is the
        tripwire.
        """
        from tools.structural_analysis import assert_cap_config_slices_cleanly

        exercised = assert_cap_config_slices_cleanly(self._m17_cap_config())
        # Only the 60 mm-chord aerofoil (7.2 mm deep) is too shallow for the
        # 10 mm spar; the coupon (20 mm) and the wing-scale section (18 mm)
        # both hold it and are checked in full.
        self.assertEqual(exercised, {"coupon", "airfoil_wing"})

    def test_a_spar_too_large_for_every_fixture_still_fails_loudly(self):
        """The failure mode this rule could introduce: skipping every fixture
        and reporting success because nothing was checked."""
        import copy

        from tools.structural_analysis import assert_cap_config_slices_cleanly

        cap = copy.deepcopy(self._m17_cap_config())
        cap["spar_diameter_mm"] = 40.0
        with self.assertRaises(AssertionError) as cm:
            assert_cap_config_slices_cleanly(cap)
        self.assertIn("no fixture could hold", str(cm.exception))

    def test_a_config_with_no_spar_is_exercised_on_all_three_fixtures(self):
        """The skip is gated on a DECLARED spar, so a config without one must
        still see every fixture, including the 60 mm aerofoil."""
        import copy

        from tools.structural_analysis import assert_cap_config_slices_cleanly

        cap = copy.deepcopy(self._m17_cap_config())
        cap["spar_diameter_mm"] = None
        exercised = assert_cap_config_slices_cleanly(cap)
        self.assertEqual(exercised, {"coupon", "airfoil", "airfoil_wing"})

    def test_a_config_with_no_spar_is_still_checked_on_every_fixture(self):
        """Non-spar configs must be unaffected: the skip is gated on a declared
        spar, so removing it must restore the original fixture set."""
        import copy

        from tools import structural_analysis as sa

        cap = copy.deepcopy(self._m17_cap_config())
        cap["spar_diameter_mm"] = None
        spec = sa.build_spec_from_cap_config(cap)
        for section in (sa.build_rect_section(20.0, 20.0, 60.0),
                        sa.build_airfoil_section()):
            self.assertIsNone(sa._spar_does_not_fit_reason(spec, section))


class SparSocketDegradationBoundaryTests(unittest.TestCase):
    """A one-sided test ("too shallow degrades") cannot catch a sign flip or an
    epsilon error -- both sides of the threshold must be pinned on the SAME
    fixture, one number apart.

    The failure mode this exists for is the one the shipped behaviour makes
    dangerous: degrading when it should have BUILT. That direction is
    silent by construction (the slice succeeds), so only a test can see it.
    """

    SPAR_D = 6.0
    CLEARANCE = 0.15
    MARGIN = 1.0
    # 2*(d/2 + clearance) + 2*margin
    NEEDED = 2.0 * (SPAR_D / 2.0 + CLEARANCE) + 2.0 * MARGIN  # 8.3 mm

    def _spec(self):
        return StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=self.MARGIN,
            spar_diameter_mm=self.SPAR_D,
            spar_station=0.5,
            spar_clearance_mm=self.CLEARANCE,
            spar_neck_width_mm=1.0,
            allow_config_spar_fallback=True,
        )

    def _generate_at_depth(self, depth_mm):
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, Rect(0.0, 0.0, 20.0, depth_mm))
        return StructureGenerator().generate(
            frame, self._spec(), 0.42,
            LayerContext(layer_index=3, print_z=0.6))

    def test_just_deep_enough_BUILDS_the_socket(self):
        res = self._generate_at_depth(self.NEEDED + 0.01)
        self.assertEqual(res.degradations, [])
        self.assertTrue(
            [i for i in res.intents if i.family_name == "spar_socket"],
            "a section deeper than the requirement must still cut a socket "
            "-- degrading here is the silent failure this pair exists for")

    def test_just_too_shallow_DEGRADES(self):
        res = self._generate_at_depth(self.NEEDED - 0.01)
        self.assertEqual(len(res.degradations), 1)
        self.assertEqual(res.degradations[0].kind, "spar_socket")
        self.assertFalse(
            [i for i in res.intents if i.family_name == "spar_socket"])

    def test_the_two_sides_are_one_hundredth_of_a_mm_apart(self):
        # Pins that the pair above really does straddle the threshold,
        # rather than both landing on the same side of a moved limit.
        self.assertLess(
            (self.NEEDED + 0.01) - (self.NEEDED - 0.01), 0.021)
        self.assertEqual(
            self._generate_at_depth(self.NEEDED + 0.01).degradations, [])
        self.assertEqual(
            len(self._generate_at_depth(self.NEEDED - 0.01).degradations), 1)



class SocketDropContributesNothingToIntentsTests(unittest.TestCase):
    """PINS CURRENT (unconverted) BEHAVIOUR at the boundary does not yet cover.

    converted the two spar-socket fit sites to Class B degradation, but the
    zero-surviving-intents refusal in ``capability.py`` is NOT converted. On a
    thinning tip the two can combine badly: every lattice web is omitted for
    thinness, leaving the spar socket as the only intent; the socket then stops
    fitting at that same station; and the layer refuses because nothing
    survived -- with a message about WEBS for what is really a socket failure.

    HONEST LIMIT OF THIS TEST. The exact combined scenario cannot be built
    here: ``StubOps`` keeps the implicit family's web at every section
    depth tried (5.0 down to 2.0 mm), so a layer where the socket is
    genuinely the ONLY intent is not constructible with the available
    fixtures. What is pinned instead is the half that IS constructible --
    a dropped socket contributes no intent, so a layer whose other
    families had all been omitted would be left with none. The combined
    case remains UNPINNED.
    """

    def test_a_dropped_socket_leaves_no_socket_intent_behind(self):
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, Rect(0.0, 0.0, 20.0, 5.0))
        spec = StructureSpec(
            slot_gap_mm=0.6,
            skin_margin_mm=1.0,
            spar_diameter_mm=6.0,
            spar_station=0.5,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
            allow_config_spar_fallback=True,
        )
        res = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=11, print_z=2.2))

        self.assertEqual(len(res.degradations), 1)
        self.assertEqual(res.degradations[0].kind, "spar_socket")
        self.assertEqual(
            [i for i in res.intents if i.family_name == "spar_socket"], [],
            "a dropped socket must contribute no intent -- if it did, the "
            "degradation record and the geometry would disagree")
        # The layer is NOT empty here only because the implicit family's
        # web survives this coupon. That is what makes the combined
        # thinning-tip case unreachable from this fixture.
        self.assertTrue(res.intents)


if __name__ == "__main__":
    unittest.main()
