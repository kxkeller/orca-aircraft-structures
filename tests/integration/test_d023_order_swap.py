"""Bundle-level (fake-host) coverage for order-swap reporting, via
``AircraftStructuresM1._classify_declared_pair_overlaps`` -- a thin edge
wrapper around ``aircraft_core.generator.classify_declared_pair_overlaps``,
which holds the actual classification logic -- and
``RunDiagnostics.order_swap_count``/``order_swap_events``. See
``tests/unit/test_order_swap_classification.py`` for the core-level tests,
including the ones that pin the classifier's march-index inversion against
stations a REAL ``StructureGenerator`` actually emits (this module uses
hand-built ``WebIntent``s, which suffices to confirm the WRAPPER delegates
correctly through the fake-host bundle, not to re-prove the inversion
itself).

WHY THIS EXISTS: the same-edge collision rule (``aircraft_core/validate.py``)
accepts ANY disjoint-membership, measurably-overlapping slot pair as an
"intended junction" without telling a designed same-family +/- diamond apex
(both sides land on the SAME march index) apart from a genuine order swap
between DIFFERENT march stations of that same declared pair. This module
drives the classifier directly against hand-built ``WebIntent``s -- exact
station algebra, no sampling, no fake-host slice needed.

Runs the checked-in GENERATED BUNDLE (never ``capability.py`` directly), the
same discipline as ``test_m1_capability.py`` -- see that module's docstring
for why.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "unit"))

import fake_orca  # noqa: E402
from aircraft_core.frame import LayerFrame  # noqa: E402
from stub_ops import StubOps, Rect  # noqa: E402


class OrderSwapClassificationTests(unittest.TestCase):
    """Drives ``AircraftStructuresM1._classify_declared_pair_overlaps`` directly,
    against a hand-built declared diamond pair (same
    ``web_position``/``rib_pitch_mm``/``z_period_mm``, equal-magnitude
    opposite-sign ``z_drift_mm_per_mm``) on a plain 20 mm rectangular frame,
    with the algebra worked out by hand below:

    ``web_position=0.5``, ``rib_pitch_mm=3.0``, ``z_drift_mm_per_mm=+/-1.0``,
    ``z_period_mm=10.0`` -> amplitude ``A = 1.0 * 10 / 4 = 2.5``. The apex
    (both families' k=0 station) crosses every half period (``2*A=5 > 0``
    trivially, since the anchor is shared). The k=0/k=+1 cross-family pair
    (this pitch is small enough that ``2*A=5 > R=3``) genuinely swaps at
    ``z=4.0`` (found by direct search over the triangle wave, matching the
    closed form in ``aircraft_core/spec.py``'s own docstring).
    """

    def setUp(self):
        self._fake_orca, restore = fake_orca.install()
        self.addCleanup(restore)
        self._tmp_dir = os.path.dirname(os.path.abspath(__file__))
        self.namespace = fake_orca.exec_bundle(
            os.path.join(self._tmp_dir, "aircraft_structures.py")
        )
        self.cap = self.namespace["AircraftStructuresM1"]()
        self.WebFamily = self.namespace["WebFamily"]
        self.StructureSpec = self.namespace["StructureSpec"]
        self.WebIntent = self.namespace["WebIntent"]

        self.ops = StubOps()
        self.frame = LayerFrame.from_polygon(self.ops, Rect(-10.0, 0.0, 10.0, 20.0))
        self.fam_plus = self.WebFamily(
            name="plus", web_position=0.5, rib_pitch_mm=3.0,
            z_drift_mm_per_mm=1.0, z_period_mm=10.0,
        )
        self.fam_minus = self.WebFamily(
            name="minus", web_position=0.5, rib_pitch_mm=3.0,
            z_drift_mm_per_mm=-1.0, z_period_mm=10.0,
        )
        self.spec = self.StructureSpec(families=(self.fam_plus, self.fam_minus))
        self.anchor = self.frame.web_x(0.5)

    def _intent(self, x, family_index, family_name, width_mm=0.5):
        return self.WebIntent(
            start=(x, 0.0), end=(x, -5.0), width_mm=width_mm,
            family_index=family_index, family_name=family_name,
        )

    def test_designed_apex_is_not_counted_as_a_swap(self):
        # z=2.5: the triangle wave's own zero crossing (both families' drift is
        # exactly 0 at this Z, by construction -- see
        # WebFamily.station_drift_mm's docstring), so both k=0 stations
        # coincide exactly. This is the intended diamond-apex touch.
        z_apex = 2.5
        d_plus = self.fam_plus.station_drift_mm(z_apex, local_chord_mm=20.0)
        d_minus = self.fam_minus.station_drift_mm(z_apex, local_chord_mm=20.0)
        self.assertAlmostEqual(d_plus, 0.0)
        self.assertAlmostEqual(d_minus, 0.0)

        ia = self._intent(self.anchor + d_plus, 0, "plus")
        ib = self._intent(self.anchor + d_minus, 1, "minus")
        intents = [ia, ib]
        intents_before = list(intents)

        events = self.cap._classify_declared_pair_overlaps(
            intents, self.spec, self.frame, layer_index=0, print_z=z_apex
        )

        self.assertEqual(events, [], "the designed apex must not be counted as a swap")
        # Geometry (the intents list itself) is untouched by classification.
        self.assertEqual(intents, intents_before)

    def test_genuine_order_swap_between_adjacent_march_stations_is_counted(self):
        # z=4.0: found by direct search over the triangle wave (see the class
        # docstring) -- plus's own k=0 station and minus's own k=+1 station
        # coincide exactly here, a genuine order swap between two DIFFERENT
        # march indices of the declared pair (not the k=0/k=0 apex).
        z_swap = 4.0
        d_plus = self.fam_plus.station_drift_mm(z_swap, local_chord_mm=20.0)
        d_minus = self.fam_minus.station_drift_mm(z_swap, local_chord_mm=20.0)

        x_plus_k0 = self.anchor + d_plus
        x_minus_k1 = self.anchor + 3.0 + d_minus
        self.assertAlmostEqual(x_plus_k0, x_minus_k1, places=6,
                                msg="test setup: the two stations must coincide at z=4.0")

        ia = self._intent(x_plus_k0, 0, "plus")
        ib = self._intent(x_minus_k1, 1, "minus")
        intents = [ia, ib]
        intents_before = list(intents)

        events = self.cap._classify_declared_pair_overlaps(
            intents, self.spec, self.frame, layer_index=1, print_z=z_swap
        )

        self.assertEqual(len(events), 1, "a genuine adjacent-march-index overlap must be counted")
        event = events[0]
        self.assertEqual(event["layer_index"], 1)
        self.assertEqual(event["print_z"], z_swap)
        self.assertEqual(event["families"], ["minus", "plus"])
        self.assertEqual({event["march_index_a"], event["march_index_b"]}, {0, 1})
        # Geometry (the intents list itself) is untouched by classification.
        self.assertEqual(intents, intents_before)

    def test_cross_tier_overlap_is_not_classified_at_all(self):
        """A skin/deep pair (different pitch, never a declared diamond pair) is
        deliberately OUT OF SCOPE -- see ``_classify_declared_pair_overlaps``'s
        own docstring. Directly overlapping footprints with disjoint membership
        but mismatched pitch must return no events, not a false "apex" or false
        "swap".
        """
        fam_skin = self.WebFamily(
            name="skin", web_position=0.5, rib_pitch_mm=3.0,
            z_drift_mm_per_mm=1.0, z_period_mm=10.0, role="skin_lattice",
        )
        fam_deep = self.WebFamily(
            name="deep", web_position=0.5, rib_pitch_mm=7.0,
            z_drift_mm_per_mm=1.0, z_period_mm=10.0, role="primary_structure",
        )
        spec = self.StructureSpec(families=(fam_skin, fam_deep))
        ia = self._intent(0.0, 0, "skin")
        ib = self._intent(0.1, 1, "deep")  # overlaps ia's 0.5mm-wide footprint

        events = self.cap._classify_declared_pair_overlaps(
            [ia, ib], spec, self.frame, layer_index=0, print_z=0.0
        )
        self.assertEqual(events, [])

    def test_socket_flank_pair_is_not_classified_and_does_not_raise(self):
        """The socket-flank families (``flank_upper_fwd``/``flank_lower_aft``)
        leave ``rib_pitch_mm``/``rib_pitch_chord_frac`` both unset
        (``resolved_rib_pitch_mm`` -> ``None``) and share the class-default
        ``web_position=0.5`` and ``z_period_mm=None``.

        ``web_position`` is not ``None`` for these families, but
        ``WebFamily`` has no ``__post_init__``: the ``0.0 <= web_position <=
        1.0`` range check lives only in ``WebFamily.validate()``, which a
        caller CAN skip. So ``web_position=None`` is a reachable state for a
        caller that bypasses ``validate()``, and the classifier's
        ``web_position is None`` guard is live defensive code. This test
        exercises the reachable path for the socket-flank families:
        ``z_period_mm=None`` on both sides passes the classifier's equality
        check trivially (``None == None``); the classifier must still reach
        its ``pitch_a is None`` guard (both pitch fields unset) and return
        unclassified, rather than raising, before computing a drift rate or
        station anchor. See ``tests/unit/test_order_swap_classification.py``
        for the core-level tests of this logic.
        """
        fam_fwd = self.WebFamily(
            name="flank_upper_fwd", role="primary_structure",
            socket_flank_offset_mm=1.0, socket_flank_direction="forward",
        )
        fam_aft = self.WebFamily(
            name="flank_lower_aft", role="primary_structure", entry_lower=True,
            socket_flank_offset_mm=1.0, socket_flank_direction="aft",
        )
        self.assertEqual(fam_fwd.web_position, 0.5)
        self.assertIsNone(fam_fwd.z_period_mm)
        self.assertIsNone(fam_fwd.rib_pitch_mm)
        self.assertIsNone(fam_fwd.rib_pitch_chord_frac)
        spec = self.StructureSpec(families=(fam_fwd, fam_aft))

        ia = self._intent(0.0, 0, "flank_upper_fwd")
        # entry_lower differs on the real pair, but the classifier's
        # entry_lower guard (same-edge only) is checked first -- use the
        # SAME entry_lower here so this test actually exercises the
        # pitch/None guards downstream, not the earlier edge guard.
        ib_same_entry = self.WebIntent(
            start=(0.1, 0.0), end=(0.1, -5.0), width_mm=0.5,
            family_index=1, family_name="flank_lower_aft", entry_lower=False,
        )

        events = self.cap._classify_declared_pair_overlaps(
            [ia, ib_same_entry], spec, self.frame, layer_index=0, print_z=0.0
        )
        self.assertEqual(events, [])

    def test_non_overlapping_pair_is_not_classified(self):
        """A declared pair whose footprints do not actually overlap (the
        classifier's precondition) must not be classified either way."""
        ia = self._intent(0.0, 0, "plus")
        ib = self._intent(5.0, 1, "minus")  # far apart, 0.5mm-wide footprints
        events = self.cap._classify_declared_pair_overlaps(
            [ia, ib], self.spec, self.frame, layer_index=0, print_z=0.0
        )
        self.assertEqual(events, [])


if __name__ == "__main__":
    unittest.main()
