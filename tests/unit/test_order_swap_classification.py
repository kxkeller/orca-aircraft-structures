"""Core-level coverage for ``aircraft_core.generator.classify_declared_pair_overlaps``.

The classifier lives in ``aircraft_core/generator.py``, beside
``StructureGenerator._candidate_stations`` -- the anchoring formula it
inverts. This module pins the ``k = round((start_x - anchor_x)/pitch)``
inversion against stations ``StructureGenerator`` ACTUALLY EMITS, not against
a hand-recomputed formula, which would reproduce the same assumption on both
sides and prove nothing.

``test_k_inversion_pins_against_actual_generator_stations`` below does NOT
recompute ``x = anchor_x + k * pitch`` anywhere. It runs the real
``StructureGenerator`` (which builds that formula's FORWARD direction, via
an incrementing integer loop -- see ``_candidate_stations``), and derives
its ground-truth ``k`` for each real, generator-emitted station purely by
COUNTING POSITIONS in the sorted list relative to the anchor's own
position -- an independent check of the same claim (forward
multiplication vs. reverse division), not a second copy of it.

``DeclaredPairClassificationAgainstRealGeneratorOutputTests`` goes one step
further: builds a real declared diamond pair
(``StructureSpec(families=(fam_plus, fam_minus))``), calls
``StructureGenerator().generate(...)`` for real (never hand-builds a
``WebIntent``), and classifies the REAL result at both the triangle wave's own
zero-crossing (the designed apex -- see ``WebFamily.station_drift_mm``) and at
the z the two families' stations are independently known (by direct search over
the triangle wave, matching ``aircraft_core/spec.py``'s own closed form -- not
this module's inversion) to coincide one march station apart.
"""

import unittest

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import (
    LayerContext,
    StructureGenerator,
    classify_declared_pair_overlaps,
)
from aircraft_core.spec import StructureSpec, WebFamily
from stub_ops import Rect, StubOps


class KInversionPinnedAgainstRealGeneratorOutputTests(unittest.TestCase):
    def test_k_inversion_pins_against_actual_generator_stations(self):
        ops = StubOps()
        # Wide frame so several march stations exist on both sides of the
        # anchor -- a single-station frame would not exercise k != 0.
        frame = LayerFrame.from_polygon(ops, Rect(0.0, 0.0, 40.0, 20.0))
        fam = WebFamily(name="solo", rib_pitch_mm=3.0, web_position=0.5)
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, families=(fam,))
        ctx = LayerContext(layer_index=0, print_z=0.0)
        result = StructureGenerator().generate(frame, spec, 0.42, ctx)

        anchor_x = frame.web_x(0.5)
        xs = sorted(intent.start[0] for intent in result.intents)
        self.assertGreater(len(xs), 3, "test setup: need multiple march stations")
        self.assertIn(anchor_x, xs, "test setup: the anchor itself must be a station (k=0)")

        # Ground truth: _candidate_stations marches outward from the anchor
        # ONE INTEGER STEP AT A TIME in each direction (see its own
        # while-loop, `k += 1` / `k -= 1`), so the sorted list's own index
        # offset from the anchor's index IS the march index by
        # construction of the loop -- independent of the division this
        # test exists to check.
        anchor_pos = xs.index(anchor_x)
        for pos, x in enumerate(xs):
            expected_k = pos - anchor_pos
            recovered_k = round((x - anchor_x) / fam.rib_pitch_mm)
            self.assertEqual(
                recovered_k, expected_k,
                "k inversion disagreed with the generator's own march order "
                "at station x={!r} (list position {}, anchor at position {})"
                .format(x, pos, anchor_pos),
            )


class DeclaredPairClassificationAgainstRealGeneratorOutputTests(unittest.TestCase):
    """Same declared-pair shape as
    ``tests/integration/test_d023_order_swap.py`` (web_position=0.5,
    rib_pitch_mm=3.0, z_drift_mm_per_mm=+/-1.0, z_period_mm=10.0 ->
    amplitude A=2.5), but every intent classified here comes from a REAL
    ``StructureGenerator().generate(...)`` call, never hand-built.
    """

    def setUp(self):
        self.ops = StubOps()
        self.frame = LayerFrame.from_polygon(self.ops, Rect(0.0, 0.0, 40.0, 20.0))
        self.fam_plus = WebFamily(
            name="plus", web_position=0.5, rib_pitch_mm=3.0,
            z_drift_mm_per_mm=1.0, z_period_mm=10.0,
        )
        self.fam_minus = WebFamily(
            name="minus", web_position=0.5, rib_pitch_mm=3.0,
            z_drift_mm_per_mm=-1.0, z_period_mm=10.0,
        )
        self.spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, families=(self.fam_plus, self.fam_minus))

    def _generate(self, print_z, layer_index):
        ctx = LayerContext(layer_index=layer_index, print_z=print_z)
        return StructureGenerator().generate(self.frame, self.spec, 0.42, ctx)

    def test_apex_z_produces_no_swap_events_from_real_generator_output(self):
        # z=2.5: both families' drift is independently known to be exactly
        # 0 here (WebFamily.station_drift_mm's own triangle-wave zero
        # crossing) -- so every one of plus's real generator stations
        # coincides with the corresponding minus station at the SAME march
        # index. No swap should be reported anywhere in the real output.
        z_apex = 2.5
        d_plus = self.fam_plus.station_drift_mm(z_apex, local_chord_mm=self.frame.width)
        d_minus = self.fam_minus.station_drift_mm(z_apex, local_chord_mm=self.frame.width)
        self.assertAlmostEqual(d_plus, 0.0)
        self.assertAlmostEqual(d_minus, 0.0)

        result = self._generate(z_apex, layer_index=0)
        events = classify_declared_pair_overlaps(
            result.intents, self.spec, self.frame, layer_index=0, print_z=z_apex
        )
        self.assertEqual(events, [])

    def test_swap_z_produces_swap_events_from_real_generator_output(self):
        # z=4.0: found by direct search over the triangle wave (matching
        # aircraft_core/spec.py's own closed form) -- plus's own k=N
        # station and minus's own k=N+1 station coincide exactly here, for
        # every N, since the two families share one pitch and the offset
        # between their anchors is constant at this z. This is a genuine
        # order swap between adjacent march indices, not the apex.
        z_swap = 4.0
        result = self._generate(z_swap, layer_index=1)
        events = classify_declared_pair_overlaps(
            result.intents, self.spec, self.frame, layer_index=1, print_z=z_swap
        )
        self.assertGreater(len(events), 0, "expected at least one genuine order swap")
        for event in events:
            self.assertEqual(event["families"], ["minus", "plus"])
            self.assertNotEqual(event["march_index_a"], event["march_index_b"])
            self.assertEqual(
                abs(event["march_index_a"] - event["march_index_b"]), 1,
                "this declared pair's constant anchor offset at z=4.0 should "
                "only ever pair adjacent march indices",
            )


if __name__ == "__main__":
    unittest.main()
