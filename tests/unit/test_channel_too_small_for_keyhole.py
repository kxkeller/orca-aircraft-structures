"""Regression: a detected channel too small to carry a keyhole socket must DEGRADE
(dropped, reported) through the existing ``Degradation`` machinery, never raise
``SpecError`` out of ``SlotBuilder._keyhole`` -- UNLESS that channel is the
CONFIRMED spar, in which case it must refuse the layer GRACEFULLY with a clean
``TopologyError`` rather than either degrading silently or raising the raw,
uncaught ``SpecError``.

Motivating case: an elevon wing has two servo pockets (~1.1 mm bores,
z 11.22 mm mid-thickness) that ``mesh_section`` hands to ``generate()``.
Both are detected ALONGSIDE the real carbon-spar channel on the same layer
(e.g. layers 293/Z=58.80 mm and 351/Z=70.40 mm record a ``bore_socket``
degradation, not ``spar_socket``), so these pockets reach ``generate()``
through the ``spar_channel_policy="nearest_spar_station"`` multi-candidate
branch as plain bores, never as the sole/confirmed channel on their layer.
The single-channel branch in ``generate()`` must check that a pocket is big
enough to carry a keyhole before building one: otherwise
``SlotBuilder._keyhole`` (slot.py) raises ``SpecError`` -- "keyhole neck
half-width 0.6000 mm >= socket radius 0.5538 mm" -- an uncaught exception,
not a clean per-layer refusal.

Incidental voids such as servo pockets may be left hollow: those layers lose
structure but still slice. That does not apply to the spar.
``require_primary_structure`` -- the mechanism that would otherwise catch
"the wing has no spar" -- does not fire for a socket dropped by this guard,
so a CONFIRMED spar channel too small for its keyhole cannot be left to
degrade silently; it refuses instead.
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
    WebIntent,
)
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.spec import (  # noqa: E402
    SpecError,
    StructureSpec,
    WebFamily,
)
from aircraft_core.validate import TopologyError  # noqa: E402

TESTS_UNIT_DIR = REPO_ROOT / "tests" / "unit"
if str(TESTS_UNIT_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_UNIT_DIR))

WALL_WIDTH_MM = 0.42

# Same tiny channel numbers throughout this file (0.8 mm diameter at x=20.0,
# y=5.0 -- r=0.4+0.15=0.55 mm < half_w 0.6 mm, the identical half_w >= r
# condition SlotBuilder._keyhole raises SpecError for, mirroring real
# 0.5538 mm/0.4107 mm sockets) so every test's assertions about the
# reported numbers stay comparable.
TINY_CHANNEL = DetectedChannel(
    center_x_mm=20.0, center_y_mm=5.0, diameter_mm=0.8,
)
# A well-fitting channel used as "the other one" wherever a test needs two
# detected channels to exercise spar_channel_policy="nearest_spar_station"
# -- large enough (3.0 mm) that r=1.5+0.15=1.65 mm comfortably clears
# half_w=0.6 mm and the depth fit on the 10 mm-tall test rectangle.
FIT_CHANNEL = DetectedChannel(
    center_x_mm=6.0, center_y_mm=5.0, diameter_mm=3.0,
)


class ChannelTooSmallForKeyholeDegradesTests(unittest.TestCase):
    """The real pipeline (``StructureGenerator.generate``, no mocks) on two
    detected channels under ``spar_channel_policy="nearest_spar_station"``,
    where ``FIT_CHANNEL`` wins the station contest (becomes the spar) and
    ``TINY_CHANNEL`` is left a plain bore too small for its keyhole: no
    ``bore_socket`` intent is built for it, exactly one ``Degradation`` is
    recorded naming the channel's centre and diameter and the reason, and
    ``generate`` returns normally (the spar itself socketed fine)."""

    def setUp(self):
        from stub_ops import Rect, StubOps

        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 40.0, 10.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def _spec(self):
        return StructureSpec(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.2,  # half_w = 0.6 mm, matching the real config
            spar_channel_policy="nearest_spar_station",
            # spar_station=0.15 -> x = 0 + 0.15*40 = 6.0 mm, exactly
            # FIT_CHANNEL's centre (distance 0) vs TINY_CHANNEL's 14.0 mm
            # -- FIT_CHANNEL unambiguously wins, TINY_CHANNEL is the bore.
            spar_station=0.15,
        )

    def test_undersized_bore_degrades_not_raises(self):
        ctx = LayerContext(
            layer_index=997, print_z=58.8,
            detected_channels=(FIT_CHANNEL, TINY_CHANNEL),
        )
        spec = self._spec()

        result = StructureGenerator().generate(
            self.frame, spec, WALL_WIDTH_MM, ctx,
            safety_offset_mm=WALL_WIDTH_MM,
        )

        # Drive every built intent through the real ``SlotBuilder`` too --
        # ``generate()`` alone never calls ``_keyhole``; the host would hit
        # this exact SpecError only once the intent becomes a polygon
        # (``tools/preflight_wing.py``'s own path). This is where the
        # defect actually surfaced.
        builder = SlotBuilder()
        for intent in result.intents:
            builder.build(intent, spec.overshoot_mm)

        bore_intents = [
            i for i in result.intents if i.family_name == "bore_socket"
        ]
        self.assertEqual(
            bore_intents, [],
            "an undersized bore must not build a keyhole socket",
        )
        spar_intents = [
            i for i in result.intents if i.family_name == "spar_socket"
        ]
        self.assertEqual(
            len(spar_intents), 1,
            "the well-fitting channel must still be socketed as the spar",
        )
        self.assertEqual(len(result.degradations), 1)
        d = result.degradations[0]
        self.assertEqual(d.kind, "bore_socket")
        self.assertIn("20.0000", d.reason, "centre x must be named")
        self.assertIn("5.0000", d.reason, "centre y must be named")
        self.assertIn("0.8000", d.reason, "diameter must be named")
        self.assertIn("too small", d.reason.lower())

    def test_keyhole_backstop_in_slot_py_is_untouched(self):
        """The ``SpecError`` raise in ``SlotBuilder._keyhole`` still fires
        for a genuine out-of-spec ``WebIntent`` built directly (bypassing
        ``generator.py``'s guard) -- it is a real backstop, though not
        reachable from the ordinary detected-channel path."""
        intent = WebIntent(
            start=(20.0, 0.0), end=(20.0, 5.0), width_mm=1.2,
            family_index=-2, family_name="spar_socket", entry_lower=True,
            socket_radius_mm=0.55,
        )
        with self.assertRaises(SpecError):
            SlotBuilder().build(intent, 1.0)


class ConfirmedSparChannelTooSmallRefusesTests(unittest.TestCase):
    """A channel that is the CONFIRMED spar (the single channel detected
    on its layer -- ``generate()``'s unambiguous single-channel branch,
    ``spar_confirmed=True``) must refuse the whole layer with a clean
    ``TopologyError`` when it is too small for its keyhole, not degrade
    silently. Dropping the confirmed spar would print a wing with no spar
    socket and no other mechanism catches it (``require_primary_structure`` is
    measured inert against this drop -- see this module's own docstring)."""

    def setUp(self):
        from stub_ops import Rect, StubOps

        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 40.0, 10.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def _spec(self):
        return StructureSpec(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.2,  # half_w = 0.6 mm, matching the real config
        )

    def test_confirmed_undersized_spar_raises_topology_error(self):
        # Exactly one detected channel -> generate()'s unambiguous
        # single-channel branch -> spar_confirmed=True.
        ctx = LayerContext(
            layer_index=997, print_z=58.8,
            detected_channels=(TINY_CHANNEL,),
        )
        spec = self._spec()

        with self.assertRaises(TopologyError) as cm:
            StructureGenerator().generate(
                self.frame, spec, WALL_WIDTH_MM, ctx,
                safety_offset_mm=WALL_WIDTH_MM,
            )

        err = cm.exception
        self.assertEqual(err.layer_index, 997)
        self.assertAlmostEqual(err.print_z, 58.8, places=6)
        msg = str(err)
        self.assertIn("20.0000", msg, "centre x must be named")
        self.assertIn("5.0000", msg, "centre y must be named")
        self.assertIn("0.8000", msg, "diameter must be named")
        self.assertIn("0.5500", msg, "computed socket radius must be named")
        self.assertIn("0.6000", msg, "neck half-width must be named")
        self.assertIn("CONFIRMED", msg)

    def test_this_is_not_a_specerror(self):
        """The clean refusal must never let the raw ``SpecError`` through
        -- that was the original defect (surfaced as "UNEXPECTED (not a
        clean TopologyError)")."""
        ctx = LayerContext(
            layer_index=997, print_z=58.8,
            detected_channels=(TINY_CHANNEL,),
        )
        spec = self._spec()

        try:
            StructureGenerator().generate(
                self.frame, spec, WALL_WIDTH_MM, ctx,
                safety_offset_mm=WALL_WIDTH_MM,
            )
        except SpecError:
            self.fail(
                "a confirmed undersized spar channel must raise "
                "TopologyError, never the raw SpecError"
            )
        except TopologyError:
            pass


# ---------------------------------------------------------------------------
#: a dropped-by-this-guard socket is a
# ``socket=None`` return, same shape as "no channel detected at all" or
# "the config-fallback socket did not fit". Two family kinds read
# ``layer.detected_channels``/the resolved ``socket`` directly rather than
# through the generator's own single-socket branch --
# ``depth_to_socket`` and ``socket_flank_offset_mm`` (spec.py:1096-1102's own
# note: a bore is protected from these because it is narrowed out of
# ``detected_channels`` before the family loop, but that narrowing does not
# apply when the DROPPED socket is the spar's own -- the channel stays in
# ``detected_channels`` while no socket was cut). says a socket-flank
# family fails closed and omits rather than falling back to a chord
# fraction; these two tests establish that BY TEST for a channel dropped by
# THIS guard specifically, not merely by reading the family loop's ``socket
# is None`` branches.
#
# The dropped channel here must be the UNCONFIRMED spar pick (not a bore),
# so it is the one ``generate()`` narrows ``layer.detected_channels`` down
# to (generator.py:1422) BEFORE the family loop runs -- a dropped bore is
# narrowed OUT and never reaches these families at all, defeating the
# point of this test. TINY_CHANNEL is therefore made the nearest-to-
# spar_station pick here (spar_station tuned to its own x=20.0), and
# FIT_CHANNEL becomes the bore instead -- the mirror image of
# ChannelTooSmallForKeyholeDegradesTests above.
# ---------------------------------------------------------------------------


def _depth_to_socket(family: WebFamily) -> WebFamily:
    """Stand-in for ``WebFamily(depth_to_socket=True)`` -- mirrors
    ``test_m19_depth_to_socket.py``'s own helper of the same name."""
    object.__setattr__(family, "depth_to_socket", True)
    return family


def _socket_flank(family: WebFamily, offset_mm=1.0, direction="aft") -> WebFamily:
    """Stand-in for ``WebFamily(socket_flank_offset_mm=..., socket_flank_
    direction=...)`` -- same attribute-poking shape as ``_depth_to_socket``
    above, for the sibling socket-relative-placement feature."""
    object.__setattr__(family, "socket_flank_offset_mm", offset_mm)
    object.__setattr__(family, "socket_flank_direction", direction)
    return family


class ChannelTooSmallForKeyholeDownstreamFamilyTests(unittest.TestCase):
    """The undersized ``TINY_CHANNEL``, this time picked as the UNCONFIRMED
    spar candidate (so it stays in ``layer.detected_channels`` after
    narrowing even though its own socket was dropped), with a SECOND family
    present that references the dropped socket."""

    def setUp(self):
        from stub_ops import Rect, StubOps

        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 40.0, 10.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def _spec(self, family):
        return StructureSpec(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            spar_clearance_mm=0.15,
            spar_neck_width_mm=1.2,  # half_w = 0.6 mm, matching the real config
            spar_channel_policy="nearest_spar_station",
            # spar_station=0.5 -> x = 0 + 0.5*40 = 20.0 mm, exactly
            # TINY_CHANNEL's centre (distance 0) vs FIT_CHANNEL's 14.0 mm
            # -- TINY_CHANNEL wins the contest and is the (unconfirmed)
            # spar pick, FIT_CHANNEL is the bore.
            spar_station=0.5,
            families=(family,),
        )

    def _generate(self, family):
        ctx = LayerContext(
            layer_index=997, print_z=58.8,
            detected_channels=(TINY_CHANNEL, FIT_CHANNEL),
        )
        return StructureGenerator().generate(
            self.frame, self._spec(family), WALL_WIDTH_MM, ctx,
            safety_offset_mm=WALL_WIDTH_MM,
        )

    def test_depth_to_socket_family_omits_rather_than_raises_or_falls_back(self):
        family = _depth_to_socket(
            WebFamily(name="brace", web_position=0.5, entry_lower=True)
        )
        result = self._generate(family)

        self.assertEqual(
            [i.family_name for i in result.intents if i.family_name == "brace"],
            [],
            "a depth_to_socket family must not build any member once its "
            "socket was dropped by the too-small-for-a-keyhole guard",
        )
        brace_omissions = [
            o for o in result.omitted if o.family_name == "brace"
        ]
        self.assertTrue(
            brace_omissions,
            "the depth_to_socket family must be OMITTED, loudly, not "
            "silently skipped",
        )
        self.assertIn("depth_to_socket", brace_omissions[0].reason)
        self.assertIn(
            "spar_socket_unconfirmed",
            [d.kind for d in result.degradations],
            "the channel's own socket must still be reported as dropped",
        )

    def test_socket_flank_family_omits_rather_than_raises_or_falls_back(self):
        family = _socket_flank(
            WebFamily(name="flank", web_position=0.5, entry_lower=True)
        )
        result = self._generate(family)

        self.assertEqual(
            [i.family_name for i in result.intents if i.family_name == "flank"],
            [],
            "a socket_flank_offset_mm family must not build any member "
            "once its socket was dropped by the too-small-for-a-keyhole "
            "guard",
        )
        flank_omissions = [
            o for o in result.omitted if o.family_name == "flank"
        ]
        self.assertTrue(
            flank_omissions,
            "the socket_flank family must be OMITTED, loudly, not "
            "silently skipped",
        )
        self.assertIn("socket_flank", flank_omissions[0].reason)


if __name__ == "__main__":
    unittest.main()
