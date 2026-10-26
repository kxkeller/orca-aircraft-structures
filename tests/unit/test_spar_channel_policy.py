"""Regression fixtures for ``StructureSpec.spar_channel_policy`` (the
"more than one detected channel at a single layer" policy question).

Motivating case: a wing tip with two Ø3.0mm alignment-rod bores (never
overlapping the real spar channel, which ends well before the bores begin).
Mesh-sectioning hands both to ``generate()`` as
``LayerContext.detected_channels`` with no way to tell which one (if either) is
the spar, so the default policy fails closed on every layer where both coexist,
and the policy field is how a user answers the question the error asks. See
``StructureSpec.spar_channel_policy``'s own docstring for the full design and
``StructureGenerator.generate``'s docstring, clause (b), for the precedence.

One class each:

- ``RefusePolicyByteIdenticalTests``: the DEFAULT ("refuse") raises
  the ``TopologyError``, unaffected by this field's mere existence.
- ``NearestSparStationPolicyTests``: "nearest_spar_station" picks the
  channel nearest ``spar_station`` as the spar, sockets the other
  detected channel as a plain bore, and the whole layer -- both sockets,
  plus the section's own ordinary lattice web -- collapses to ONE closed
  loop under the REAL boolean backend and the REAL ``TopologyValidator``,
  never a stub approximation of either.
- ``ThreeChannelPolicyTests``: pins the chosen behaviour with three
  detected channels (one spar, two bores), rather than leaving it
  unspecified.
- ``ConfigSchemaValidationTests``: an unrecognized policy string is a
  ``SpecError`` naming the accepted values.
"""
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import (  # noqa: E402
    DetectedChannel,
    LayerContext,
    StructureGenerator,
)
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.spec import SpecError, StructureSpec, WebFamily  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

from shapely_ops import HAVE_SHAPELY as _HAVE_SHAPELY  # noqa: E402
from shapely_ops import MiterShapelyOps as _MiterShapelyOps  # noqa: E402

TESTS_UNIT_DIR = REPO_ROOT / "tests" / "unit"
if str(TESTS_UNIT_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_UNIT_DIR))

WALL_WIDTH_MM = 0.42

# A 40 x 10 mm coupon, wide enough to carry two Ø3.0mm channels roughly as far
# apart (34 mm) as real alignment bores (36.04 mm), deep enough (10 mm) that
# every socket (radius 1.65 mm at these numbers, needing 2*1.65 + 2*0.6 = 4.5
# mm of local depth) comfortably fits without itself being the subject of this
# fixture.
_RECT_POINTS = [(0.0, 0.0), (40.0, 0.0), (40.0, 10.0), (0.0, 10.0)]


class RefusePolicyByteIdenticalTests(unittest.TestCase):
    """The default policy ("refuse") raises a ``TopologyError``
    naming the layer, the detected channels, and the "this
    is never guessed" refusal -- checked here by substring, deliberately not
    by exact string equality.

    The message also carries a sentence naming ``spar_channel_policy`` as the
    way out (see ``test_spar_channel_policy_refuse_message.py``); every
    substring pinned below is independent of that sentence."""

    def setUp(self):
        from stub_ops import Rect, StubOps

        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 40.0, 10.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)
        self.channels = (
            DetectedChannel(center_x_mm=3.0, center_y_mm=5.0, diameter_mm=3.0),
            DetectedChannel(center_x_mm=37.0, center_y_mm=5.0, diameter_mm=3.0),
        )

    def _generate(self, spec):
        ctx = LayerContext(
            layer_index=1076, print_z=215.2, detected_channels=self.channels
        )
        return StructureGenerator().generate(self.frame, spec, WALL_WIDTH_MM, ctx)

    def test_default_policy_refuses_exactly_as_before(self):
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0)
        self.assertEqual(spec.spar_channel_policy, "refuse")
        with self.assertRaises(TopologyError) as ctx:
            self._generate(spec)
        msg = str(ctx.exception)
        self.assertIn("2 spar channels detected in the model", msg)
        self.assertIn(
            "which channel is the spar is a POLICY question the model "
            "alone cannot answer",
            msg,
        )
        self.assertIn("the plugin must be told explicitly", msg)

    def test_explicit_refuse_is_identical_to_the_default(self):
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, spar_channel_policy="refuse"
        )
        with self.assertRaises(TopologyError) as ctx:
            self._generate(spec)
        self.assertIn("2 spar channels detected in the model", str(ctx.exception))


@unittest.skipUnless(_HAVE_SHAPELY, "shapely not installed")
class NearestSparStationPolicyTests(unittest.TestCase):
    """"nearest_spar_station": one channel becomes the spar, the other a
    plain bore -- both socketed, the whole layer verified as ONE closed
    loop by the real boolean backend and the real ``TopologyValidator``."""

    @classmethod
    def setUpClass(cls):
        cls.ops = _MiterShapelyOps()
        cls.builder = SlotBuilder()

    def setUp(self):
        self.poly = self.ops.polygon_from_points(_RECT_POINTS)
        self.frame = LayerFrame.from_polygon(self.ops, self.poly)
        # spar_station=0.1 -> x = 0 + 0.1*40 = 4.0 mm, nearest the channel
        # centred at x=3.0 (distance 1.0) rather than the one at x=37.0
        # (distance 33.0) -- deliberately not equidistant or symmetric, so
        # a bug that always picked channel 0 (or the largest, or the last)
        # could not accidentally pass.
        self.channels = (
            DetectedChannel(center_x_mm=3.0, center_y_mm=5.0, diameter_mm=3.0),
            DetectedChannel(center_x_mm=37.0, center_y_mm=5.0, diameter_mm=3.0),
        )

    def _spec(self, **overrides):
        kwargs = dict(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            web_position=0.5,
            spar_channel_policy="nearest_spar_station",
            spar_station=0.1,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
        )
        kwargs.update(overrides)
        return StructureSpec(**kwargs)

    def _run(self, spec, channels):
        ctx = LayerContext(
            layer_index=1076, print_z=215.2, detected_channels=channels
        )
        result = StructureGenerator().generate(
            self.frame, spec, WALL_WIDTH_MM, ctx, safety_offset_mm=WALL_WIDTH_MM
        )
        built = [self.builder.build(i, spec.overshoot_mm) for i in result.intents]
        slots = [self.ops.polygon_from_points(b.points) for b in built]
        transformed = self.ops.diff(self.poly, slots)
        validator = TopologyValidator(self.ops, safety_offset_mm=WALL_WIDTH_MM)
        validator.validate(
            [self.poly], transformed, slots, spec, 1076, 215.2,
            slot_labels=[str(i.family_name) for i in result.intents],
            family_indices=[i.family_membership() for i in result.intents],
        )
        return result, transformed

    def test_nearer_channel_is_the_spar_farther_is_a_bore(self):
        spec = self._spec()
        result, _transformed = self._run(spec, self.channels)

        spar_intents = [i for i in result.intents if i.family_name == "spar_socket"]
        bore_intents = [i for i in result.intents if i.family_name == "bore_socket"]
        self.assertEqual(len(spar_intents), 1, "expected exactly one spar socket")
        self.assertEqual(len(bore_intents), 1, "expected exactly one bore socket")
        self.assertAlmostEqual(spar_intents[0].start[0], 3.0, places=6)
        self.assertAlmostEqual(bore_intents[0].start[0], 37.0, places=6)
        self.assertFalse(result.degradations)

    def test_bore_socket_radius_matches_detected_diameter_plus_clearance(self):
        spec = self._spec()
        result, _transformed = self._run(spec, self.channels)
        bore = next(i for i in result.intents if i.family_name == "bore_socket")
        self.assertAlmostEqual(bore.socket_radius_mm, 3.0 / 2.0 + 0.15, places=6)

    def test_result_is_exactly_one_closed_loop(self):
        """The non-negotiable invariant, stated directly: after both
        sockets are cut, the layer is still ONE piece with no interior
        hole left unaccounted for -- ``TopologyValidator.validate`` above
        already enforces this (it would have raised otherwise), so this
        only pins the shape of the surviving geometry as an independent
        check on the same claim."""
        spec = self._spec()
        _result, transformed = self._run(spec, self.channels)
        self.assertEqual(len(transformed), 1, "expected a single piece")
        interiors = self.ops.hole_points_mm(transformed[0])
        self.assertEqual(
            interiors, [],
            "a surviving interior hole means a bore or the spar was not "
            "fully socketed through to the boundary",
        )

    def test_swapping_which_channel_is_nearer_swaps_the_role(self):
        """Same two channels, spar_station moved to the OTHER end --
        confirms the choice tracks spar_station, not channel order or
        list position."""
        spec = self._spec(spar_station=0.9)
        result, _transformed = self._run(spec, self.channels)
        spar_intents = [i for i in result.intents if i.family_name == "spar_socket"]
        bore_intents = [i for i in result.intents if i.family_name == "bore_socket"]
        self.assertAlmostEqual(spar_intents[0].start[0], 37.0, places=6)
        self.assertAlmostEqual(bore_intents[0].start[0], 3.0, places=6)

    def test_bore_that_does_not_fit_degrades_not_fails(self):
        """A bore too close to the section's own thin edge (here, forced
        by an oversized declared diameter) DEGRADES (dropped, reported)
        rather than failing the whole slice -- exactly the spar socket's
        own fail-soft behaviour, applied to a bore."""
        oversized_channels = (
            DetectedChannel(center_x_mm=3.0, center_y_mm=5.0, diameter_mm=3.0),
            DetectedChannel(center_x_mm=20.0, center_y_mm=5.0, diameter_mm=9.0),
        )
        spec = self._spec()
        result, _transformed = self._run(spec, oversized_channels)
        bore_intents = [i for i in result.intents if i.family_name == "bore_socket"]
        self.assertEqual(bore_intents, [])
        kinds = [d.kind for d in result.degradations]
        self.assertIn("bore_socket", kinds)
        # A dropped bore must never resemble a dropped spar socket to any
        # downstream consumer keying on kind (capability.py's latch arms on
        # exactly this string) -- checked against the WHOLE degradations list,
        # not a pre-filtered one, so this cannot pass merely because the filter
        # removed the only entry.
        self.assertNotIn("spar_socket", kinds)

    def test_bore_degradation_never_uses_the_spar_socket_kind_or_family_name(self):
        spec = self._spec()
        result, _transformed = self._run(spec, self.channels)
        bore_intents = [i for i in result.intents if i.family_name == "bore_socket"]
        self.assertTrue(bore_intents, "expected at least one bore_socket intent")
        for i in bore_intents:
            # family_index=-7 is the bore socket's own sentinel -- -1 is
            # the scar, -2 is the spar socket (see
            # StructureGenerator._generate_bore_socket_from_channel's own
            # docstring for the full registry and why -7, not -3).
            self.assertEqual(i.family_index, -7)
        for d in result.degradations:
            if "bore" in d.reason.lower():
                self.assertNotEqual(d.kind, "spar_socket")


@unittest.skipUnless(_HAVE_SHAPELY, "shapely not installed")
class ThreeChannelPolicyTests(unittest.TestCase):
    """Three detected channels on one layer: one spar (nearest
    ``spar_station``), two plain bores. Pinned explicit behaviour --
    every non-spar channel is socketed, none silently dropped."""

    @classmethod
    def setUpClass(cls):
        cls.ops = _MiterShapelyOps()
        cls.builder = SlotBuilder()

    def test_two_bores_both_get_sockets(self):
        rect_points = [(0.0, 0.0), (60.0, 0.0), (60.0, 10.0), (0.0, 10.0)]
        poly = self.ops.polygon_from_points(rect_points)
        frame = LayerFrame.from_polygon(self.ops, poly)
        channels = (
            DetectedChannel(center_x_mm=3.0, center_y_mm=5.0, diameter_mm=3.0),
            DetectedChannel(center_x_mm=30.0, center_y_mm=5.0, diameter_mm=3.0),
            DetectedChannel(center_x_mm=57.0, center_y_mm=5.0, diameter_mm=3.0),
        )
        spec = StructureSpec(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            web_position=0.5,
            spar_channel_policy="nearest_spar_station",
            spar_station=0.05,  # x=3.0 -> nearest channel 0
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
        )
        ctx = LayerContext(layer_index=0, print_z=1.0, detected_channels=channels)
        result = StructureGenerator().generate(
            frame, spec, WALL_WIDTH_MM, ctx, safety_offset_mm=WALL_WIDTH_MM
        )
        built = [self.builder.build(i, spec.overshoot_mm) for i in result.intents]
        slots = [self.ops.polygon_from_points(b.points) for b in built]
        transformed = self.ops.diff(poly, slots)
        validator = TopologyValidator(self.ops, safety_offset_mm=WALL_WIDTH_MM)
        validator.validate(
            [poly], transformed, slots, spec, 0, 1.0,
            slot_labels=[str(i.family_name) for i in result.intents],
            family_indices=[i.family_membership() for i in result.intents],
        )

        spar_intents = [i for i in result.intents if i.family_name == "spar_socket"]
        bore_intents = [i for i in result.intents if i.family_name == "bore_socket"]
        self.assertEqual(len(spar_intents), 1)
        self.assertEqual(len(bore_intents), 2)
        self.assertAlmostEqual(spar_intents[0].start[0], 3.0, places=6)
        self.assertCountEqual(
            [round(i.start[0], 4) for i in bore_intents], [30.0, 57.0]
        )


class SocketFlankRefusesAnUnconfirmedSparTests(unittest.TestCase):
    """A ``socket_flank_offset_mm``
    family (``generator.py``'s ``_generate_family``, the
    ``socket_flank_active`` branch) must build NOTHING when the resolved socket
    is not a CONFIRMED spar (``WebIntent.spar_confirmed`` -- see that field's
    own docstring) -- i.e. whenever
    ``spar_channel_policy="nearest_spar_station"`` picked the spar by
    disambiguating among two or more detected channels, regardless of which
    channel won.

    Sizing against "the chosen spar channel" would be wrong: the "chosen spar"
    in a multi-channel layer is a policy GUESS good enough to cut a socket,
    never a confirmed fact good enough to anchor structure to (a wing with
    alignment-rod bores exposes this).

    Deliberately keeps both channel orderings (spar nearer index 0, spar
    nearer index 1) so a bug that only refused when the spar happened to
    land at a particular index could not accidentally pass.
    """

    def setUp(self):
        from stub_ops import Rect, StubOps

        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 40.0, 10.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def _spec(self, spar_station):
        flank_family = WebFamily(
            name="flank", entry_lower=False,
            socket_flank_offset_mm=1.005, socket_flank_direction="forward",
        )
        return StructureSpec(
            gap_ratio=0.05,
            skin_margin_mm=1.0,
            spar_channel_policy="nearest_spar_station",
            spar_station=spar_station,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
            families=(flank_family,),
        )

    def test_flank_builds_nothing_when_spar_is_channel_index_1(self):
        bore_at_index_0 = DetectedChannel(
            center_x_mm=3.0, center_y_mm=5.0, diameter_mm=3.0
        )
        spar_at_index_1 = DetectedChannel(
            center_x_mm=37.0, center_y_mm=5.0, diameter_mm=3.0
        )
        channels = (bore_at_index_0, spar_at_index_1)
        spec = self._spec(spar_station=0.9)  # x=36.0 -> nearest channels[1]
        ctx = LayerContext(layer_index=0, print_z=0.0, detected_channels=channels)
        result = StructureGenerator().generate(
            self.frame, spec, 0.42, ctx, safety_offset_mm=0.42
        )

        # The socket is still CUT (topology) -- narrowing/selection still applies --
        # but is not confirmed.
        spar_intents = [i for i in result.intents if i.family_name == "spar_socket"]
        self.assertEqual(len(spar_intents), 1)
        self.assertAlmostEqual(spar_intents[0].start[0], 37.0, places=6)
        self.assertFalse(spar_intents[0].spar_confirmed)

        flank = [i for i in result.intents if i.family_name == "flank"]
        self.assertEqual(
            flank, [], "flank must not anchor to an unconfirmed spar socket"
        )
        flank_omissions = [o for o in result.omitted if o.family_name == "flank"]
        self.assertEqual(len(flank_omissions), 1)
        self.assertIn("not a CONFIRMED spar", flank_omissions[0].reason)

    def test_flank_builds_nothing_when_spar_is_channel_index_0(self):
        spar_at_index_0 = DetectedChannel(
            center_x_mm=3.0, center_y_mm=5.0, diameter_mm=3.0
        )
        bore_at_index_1 = DetectedChannel(
            center_x_mm=37.0, center_y_mm=5.0, diameter_mm=3.0
        )
        channels = (spar_at_index_0, bore_at_index_1)
        spec = self._spec(spar_station=0.1)  # x=4.0 -> nearest channels[0]
        ctx = LayerContext(layer_index=0, print_z=0.0, detected_channels=channels)
        result = StructureGenerator().generate(
            self.frame, spec, 0.42, ctx, safety_offset_mm=0.42
        )

        spar_intents = [i for i in result.intents if i.family_name == "spar_socket"]
        self.assertEqual(len(spar_intents), 1)
        self.assertAlmostEqual(spar_intents[0].start[0], 3.0, places=6)
        self.assertFalse(spar_intents[0].spar_confirmed)

        flank = [i for i in result.intents if i.family_name == "flank"]
        self.assertEqual(
            flank, [], "flank must not anchor to an unconfirmed spar socket"
        )


class DepthToSocketRefusesAnUnconfirmedSparTests(unittest.TestCase):
    """A ``depth_to_socket=True`` family
    (``generator.py``'s ``_generate_family``) must build NOTHING when the
    ``socket`` WebIntent ``generate()`` resolved for this layer is not a
    CONFIRMED spar (``WebIntent.spar_confirmed``) -- even though a socket
    exists and even though it was chosen (correctly, by ``spar_station``
    proximity) from among several detected channels.

    Sizing a member against "the chosen spar channel" whenever
    ``nearest_spar_station`` picked one would be wrong for the same reason as
    ``socket_flank_offset_mm`` (see the class above). No shipped family sets
    ``depth_to_socket=True``, but the code path shares the identical exposure,
    so it is pinned regardless of whether it currently fires in practice.
    """

    def setUp(self):
        from stub_ops import Rect, StubOps

        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 40.0, 10.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def test_depth_to_socket_member_builds_nothing_against_an_unconfirmed_spar(self):
        bore_at_index_0 = DetectedChannel(
            center_x_mm=3.0, center_y_mm=5.0, diameter_mm=1.0,
        )  # r = 1.0/2 + 0.15 = 0.65
        spar_at_index_1 = DetectedChannel(
            center_x_mm=37.0, center_y_mm=5.0, diameter_mm=3.0,
        )  # r = 3.0/2 + 0.15 = 1.65
        channels = (bore_at_index_0, spar_at_index_1)

        brace_family = WebFamily(
            name="brace", web_position=0.5, entry_lower=True,
            depth_to_socket=True,
        )
        spec = StructureSpec(
            gap_ratio=0.05,
            skin_margin_mm=1.0,
            spar_channel_policy="nearest_spar_station",
            spar_station=0.9,  # x = 36.0 mm -> nearest channels[1] (37.0)
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.0,
            families=(brace_family,),
        )
        ctx = LayerContext(layer_index=0, print_z=0.0, detected_channels=channels)
        result = StructureGenerator().generate(
            self.frame, spec, 0.42, ctx, safety_offset_mm=0.42
        )

        # Confirms the setup: channels[1] (x=37.0, diameter 3.0) is the
        # chosen spar -- still CUT (topology), but
        # not confirmed.
        spar_intents = [i for i in result.intents if i.family_name == "spar_socket"]
        self.assertEqual(len(spar_intents), 1)
        self.assertAlmostEqual(spar_intents[0].start[0], 37.0, places=6)
        self.assertAlmostEqual(spar_intents[0].socket_radius_mm, 1.65, places=6)
        self.assertFalse(spar_intents[0].spar_confirmed)

        brace = [i for i in result.intents if i.family_name == "brace"]
        self.assertEqual(
            brace, [],
            "depth_to_socket must not size a member against an "
            "unconfirmed spar socket",
        )
        brace_omissions = [o for o in result.omitted if o.family_name == "brace"]
        self.assertEqual(len(brace_omissions), 1)
        self.assertIn("not a CONFIRMED spar", brace_omissions[0].reason)


class ConfigSchemaValidationTests(unittest.TestCase):
    def test_unrecognized_policy_string_is_a_spec_error(self):
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0,
            spar_channel_policy="nearest_channel_diameter",
        )
        with self.assertRaises(SpecError) as ctx:
            spec.validate()
        msg = str(ctx.exception)
        self.assertIn("spar_channel_policy", msg)
        self.assertIn("nearest_channel_diameter", msg)
        self.assertIn("refuse", msg)
        self.assertIn("nearest_spar_station", msg)


if __name__ == "__main__":
    unittest.main()
