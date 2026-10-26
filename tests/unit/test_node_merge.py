"""The union-interval node merge.

A close-but-not-overlapping cross-family node (``0 <= gap < min_wall_mm``)
is merged into ONE slot spanning the union interval of both candidates,
rather than dropping one family's candidate. ``StructureGenerator`` does this
in ``_merge_cross_family_nodes``. The tests are written against OBSERVABLE
behaviour of the public API (``StructureGenerator.generate``,
``GenerationResult``, and -- for precise, hand-controlled gaps -- the same
private ``_merge_cross_family_nodes`` entry point
``tests/unit/test_node_merge_scope.py`` uses for identical reasons).

Scope, because two of the five test groups below exist specifically to pin
it:

- **In:** the sub-bead band, ``0 <= gap < min_wall_mm`` where
  ``min_wall_mm = 2 * safety_offset_mm``.
- **Out, deliberately:** overlap merge (``gap < 0``). A cross-family pair
  whose REQUESTED footprints already overlap is governed by the overlap rule
  and must be left untouched by the band rule.
"""
import unittest

from aircraft_core.frame import LayerFrame, corrected_skin_margin_mm
from aircraft_core.generator import LayerContext, StructureGenerator, WebIntent
from aircraft_core.spec import StructureSpec, WebFamily
from aircraft_core.validate import TopologyError, TopologyValidator, _family_membership
from stub_ops import MultiNotchedRect, Rect, StubOps

# ``_merge_cross_family_nodes``/``_merge_node_component`` REQUIRE a
# ``LayerContext`` -- the merge-halo breach raises a ``TopologyError`` that
# must name a real layer index and Z, and an optional ctx defaulting to
# ``-1``/``0.0`` could emit a diagnostic naming the wrong layer. These direct-call tests do not exercise that raise, so any
# concrete context serves; it is named rather than inlined so it is obvious
# these values are inert here.
_MERGE_TEST_CTX = LayerContext(layer_index=0, print_z=0.0)


# Outer wall line width used throughout -- the same value the shipped coupon
# config and ``test_node_merge_scope.py`` both use, and therefore the same
# ``min_wall_mm = 2 * 0.42 = 0.84`` mm sub-bead band.
_SAFETY_OFFSET_MM = 0.42
_MIN_WALL_MM = 2.0 * _SAFETY_OFFSET_MM


def _shipped_diamond_spec():
    """The shipped coupon config: rib_pitch 8.0, positions 0.4/0.6, drift
    +/-1.0 mm/mm (45 degrees), period 12.0 mm, gap_ratio 0.05 (slot width
    0.05 * 0.42 = 0.021 mm), skin_margin 1.0 mm -- identical to
    ``test_node_merge_scope.py``'s own ``_shipped_diamond_spec`` and
    ``host/data/orca_plugins/config.json``. Reproduced here rather than
    imported so this file has no dependency on that sibling test module's
    internals.
    """
    fam_fwd = WebFamily(
        name="fwd",
        rib_pitch_mm=8.0,
        web_position=0.4,
        z_drift_mm_per_mm=1.0,
        z_period_mm=12.0,
    )
    fam_aft = WebFamily(
        name="aft",
        rib_pitch_mm=8.0,
        web_position=0.6,
        z_drift_mm_per_mm=-1.0,
        z_period_mm=12.0,
    )
    return StructureSpec(
        gap_ratio=0.05, skin_margin_mm=1.0, families=(fam_fwd, fam_aft)
    )


def _shipped_coupon_frame():
    """20 mm chord coupon, matching ``test_junction.py``'s own ``self.original =
    Rect(0.0, 0.0, 20.0, 20.0)`` -- the same rectangle the in-host coupon slice
    is measured against.
    """
    ops = StubOps()
    rect = Rect(0.0, 0.0, 20.0, 20.0)
    frame = LayerFrame.from_polygon(ops, rect)
    return frame, ops


def _body_layers(count=71, start_index=5, layer_height_mm=0.2):
    """Layer contexts for body layers 5.75 inclusive (71 layers), 0.2 mm layers,
    matching the shipped process preset (``layer_height`` 0.2 mm)'s own
    layer/print_z pairing (``print_z = layer_height_mm * (layer_index + 1)``,
    layer 0 at Z=0.2).
    """
    layers = []
    for idx in range(start_index, start_index + count):
        print_z = round(layer_height_mm * (idx + 1), 6)
        layers.append(LayerContext(layer_index=idx, print_z=print_z))
    return layers


def _mk_intent(x, width_mm, family_index):
    """A hand-built ``WebIntent`` at chord station ``x``, matching the shape
    ``test_node_merge_scope.py``'s own
    ``test_determinism_independent_of_intent_construction_order`` builds --
    same Y-range, same construction, used here purely to control the exact
    edge gap between two candidates precisely (which driving the real
    generator through ``StructureSpec``'s pitch/position/drift arithmetic
    cannot do without also re-deriving that arithmetic in the test).
    """
    return WebIntent(
        start=(x, 3.0),
        end=(x, -2.0),
        width_mm=width_mm,
        family_index=family_index,
        family_name="fam{}".format(family_index),
    )


_EXPECTED_MERGE_BEARING_LAYERS = 18
# Merge-bearing layer count the shipped-config fixtures below pin as a
# NON-VACUITY guard (the historical band was 18 close-but-disjoint layers; adds
# the four true overlap layers to the same safe union-node path, giving 22.
# Confirmed by direct computation against this exact fixture, not guessed.


def _raw_family_candidates(frame, spec, layer, safety_offset_mm):
    """Every family's own PRE-MERGE candidates at this layer, via the same
    ``StructureGenerator._generate_family`` the real pipeline calls before
    ``_merge_cross_family_nodes`` ever runs -- not re-derived pitch/
    position/drift arithmetic (see this file's own module docstring for
    why re-deriving that arithmetic in a test is rejected). Used only to
    verify the merge's PHYSICAL claim (which raw footprints a merged
    interval's own edges actually coincide with) against something other
    than the merge's own output.
    """
    gen = StructureGenerator()
    effective_skin_margin_mm = corrected_skin_margin_mm(
        spec.skin_margin_mm, frame.rotation_deg
    )
    raw = []
    for family_index, family in enumerate(spec.effective_families()):
        fam_intents, _fam_omitted = gen._generate_family(
            frame,
            spec,
            family,
            family_index,
            safety_offset_mm,
            layer,
            safety_offset_mm,
            effective_skin_margin_mm,
        )
        raw.extend(fam_intents)
    return raw


class ZeroSuppressionAtShippedConfigTests(unittest.TestCase):
    """No layer dwells in the sub-bead band: the suppression count is 0 across
    the shipped coupon config (no ``node suppression:...`` omission at all).

    NON-VACUITY GUARD: "zero suppressions" and "zero merges" are NOT the same
    claim. A config that never crosses at all (e.g. ``z_period_mm`` 7.5,
    ``z_drift_mm_per_mm`` 0.8, zero crossings) would pass a zero-suppression
    check trivially: no crossing means no candidate ever enters the sub-bead
    band, so no suppression AND no merge ever fires. This test must fail
    loudly if the config stops crossing, not read a silently-broken config as
    a pass.
    """

    def test_no_body_layer_produces_a_node_suppression_omission(self):
        frame, _ops = _shipped_coupon_frame()
        spec = _shipped_diamond_spec()
        gen = StructureGenerator()

        suppressed_by_layer = {}
        merge_bearing_layers = []
        for layer in _body_layers():
            result = gen.generate(frame, spec, _SAFETY_OFFSET_MM, layer)
            supp = [
                o
                for o in result.omitted
                if o.reason.startswith("node suppression:")
            ]
            if supp:
                suppressed_by_layer[layer.layer_index] = len(supp)
            if any(i.merged_family_indices for i in result.intents):
                merge_bearing_layers.append(layer.layer_index)

        total_suppressed = sum(suppressed_by_layer.values())
        self.assertEqual(
            total_suppressed,
            0,
            "expected zero node-suppression omissions across all 71 body "
            "layers of the shipped config; found {} across {} layers: {}"
            .format(
                total_suppressed,
                len(suppressed_by_layer),
                sorted(suppressed_by_layer.items()),
            ),
        )

        # NON-VACUITY: at least one merge must actually have occurred.
        self.assertGreater(
            len(merge_bearing_layers),
            0,
            "THIS TEST HAS GONE VACUOUS: zero merges occurred across all "
            "71 body layers of the shipped config. Zero suppressions with "
            "zero merges proves nothing -- it means this config has "
            "stopped CROSSING, not that crossings now merge cleanly. "
            "Check the shipped families' z_period_mm/z_drift_mm_per_mm/"
            "rib_pitch_mm/web_position against the crossing condition "
            "before trusting the assertion above.",
        )
        self.assertEqual(
            len(merge_bearing_layers),
            _EXPECTED_MERGE_BEARING_LAYERS,
            "expected merges on exactly {} of 71 body layers (the band "
            "this fixture's own docstring cites); "
            "got {} merge-bearing layers: {}. If this genuinely changed "
            "(a deliberate config or geometry change), update the cited "
            "band in both places, not just this number -- a silent drift "
            "here is the same label/behaviour mismatch this project keeps "
            "getting bitten by.".format(
                _EXPECTED_MERGE_BEARING_LAYERS,
                len(merge_bearing_layers),
                merge_bearing_layers,
            ),
        )


class BothFamiliesContinuousThroughEveryNodeTests(unittest.TestCase):
    """Both members survive every node: no family_index is ever dropped (a
    family vanishing from ``result.intents`` at a node would be a failure).

    THIS MUST CHECK ACTUAL MEMBERSHIP, NOT A LABEL. A merged ``WebIntent``
    carries exactly one REPRESENTATIVE ``family_index`` (a fixed rule -- always
    the lowest ``family_index`` among the component's members, see
    ``aircraft_core/generator.py``'s ``_merge_cross_family_nodes`` docstring)
    but records every family that actually fed the merge in
    ``merged_family_indices``. A test that only inspects ``i.family_index``
    across ``result.intents`` is checking the LABEL, which would keep passing
    even if a family had genuinely vanished from the merged geometry and the
    representative label simply never happened to point at the missing one on
    this fixture. So "present" here is the UNION of every standalone intent's own
    ``family_index`` and every merged intent's full ``merged_family_indices``
    tuple, not ``family_index`` alone.

    NON-VACUITY GUARD, same reasoning as
    ``ZeroSuppressionAtShippedConfigTests``: this test would pass
    trivially on a config that never merges at all (every layer's ``present``
    set would still equal ``{0, 1}`` from ORDINARY, unmerged stations spread
    across the chord, with zero merges required) -- proving nothing about merge
    CONTINUITY specifically. This is asserted below too.

    PHYSICAL CONTINUITY, not just membership: the membership check alone would
    still pass if a future change dropped a member from the merged FOOTPRINT
    while leaving its index in ``merged_family_indices`` (the tuple is computed
    independently of the footprint union -- see ``_merge_node_component``). So
    for every merged intent this also verifies the PHYSICAL claim directly: the
    merged interval's own edges (``component_min_x``/``component_max_x``) must
    coincide with its actual extreme members' own outer edges (not merely lie
    within them), and those two extreme members must come from DIFFERENT
    families -- the two facts that, together, are what "a real geodetic node,
    not a label" actually means. The extreme members themselves come from
    ``_raw_family_candidates`` (the real, pre-merge per-family generation
    path), not from re-deriving the merge's own arithmetic.
    """

    def test_every_body_layer_keeps_both_family_indices(self):
        frame, _ops = _shipped_coupon_frame()
        spec = _shipped_diamond_spec()
        gen = StructureGenerator()
        expected_families = set(
            range(len(spec.effective_families()))
        )  # {0, 1}

        layers_missing_a_family = []
        merge_bearing_layers = []
        footprint_mismatches = []
        same_family_extremes = []
        for layer in _body_layers():
            result = gen.generate(frame, spec, _SAFETY_OFFSET_MM, layer)
            present = set()
            merged_intents = []
            for i in result.intents:
                if i.merged_family_indices:
                    present.update(i.merged_family_indices)
                    merged_intents.append(i)
                else:
                    present.add(i.family_index)
            if present != expected_families:
                layers_missing_a_family.append(
                    (layer.layer_index, layer.print_z, sorted(present))
                )
            if not merged_intents:
                continue
            merge_bearing_layers.append(layer.layer_index)

            # PHYSICAL CONTINUITY: check each merged intent's footprint
            # against the REAL pre-merge candidates at this same layer.
            raw = _raw_family_candidates(
                frame, spec, layer, _SAFETY_OFFSET_MM
            )
            for merged in merged_intents:
                m_min_x = merged.start[0] - merged.width_mm / 2.0
                m_max_x = merged.start[0] + merged.width_mm / 2.0
                members = [
                    r
                    for r in raw
                    if (r.start[0] - r.width_mm / 2.0) >= m_min_x - 1e-6
                    and (r.start[0] + r.width_mm / 2.0) <= m_max_x + 1e-6
                ]
                if not members:
                    footprint_mismatches.append(
                        (
                            layer.layer_index,
                            "no raw candidate found inside merged "
                            "footprint [{:.4f}, {:.4f}]".format(
                                m_min_x, m_max_x
                            ),
                        )
                    )
                    continue
                leftmost = min(
                    members, key=lambda r: r.start[0] - r.width_mm / 2.0
                )
                rightmost = max(
                    members, key=lambda r: r.start[0] + r.width_mm / 2.0
                )
                leftmost_min_x = leftmost.start[0] - leftmost.width_mm / 2.0
                rightmost_max_x = rightmost.start[0] + rightmost.width_mm / 2.0
                if abs(leftmost_min_x - m_min_x) > 1e-6:
                    footprint_mismatches.append(
                        (
                            layer.layer_index,
                            "merged min_x {:.6f} does not coincide with "
                            "leftmost member's own edge {:.6f}".format(
                                m_min_x, leftmost_min_x
                            ),
                        )
                    )
                if abs(rightmost_max_x - m_max_x) > 1e-6:
                    footprint_mismatches.append(
                        (
                            layer.layer_index,
                            "merged max_x {:.6f} does not coincide with "
                            "rightmost member's own edge {:.6f}".format(
                                m_max_x, rightmost_max_x
                            ),
                        )
                    )
                if leftmost.family_index == rightmost.family_index:
                    same_family_extremes.append(
                        (layer.layer_index, leftmost.family_index)
                    )

        self.assertEqual(
            layers_missing_a_family,
            [],
            "expected every body layer to keep both family_index 0 and 1 "
            "(as a standalone intent OR inside a merge's "
            "merged_family_indices); {} layers dropped a family: {}".format(
                len(layers_missing_a_family), layers_missing_a_family
            ),
        )

        # NON-VACUITY: at least one merge must actually have occurred, and
        # the merge-bearing layer count must match the cited band -- same
        # guard, same reasoning, as ZeroSuppressionAtShippedConfigTests.
        self.assertGreater(
            len(merge_bearing_layers),
            0,
            "THIS TEST HAS GONE VACUOUS: zero merges occurred across all "
            "71 body layers of the shipped config. Every family_index "
            "being present on every layer proves nothing about merge "
            "continuity if no merge was ever attempted -- it means this "
            "config has stopped CROSSING, not that crossings stay "
            "continuous through a node. Check the shipped families' "
            "z_period_mm/z_drift_mm_per_mm/rib_pitch_mm/web_position "
            "against the crossing condition before trusting anything "
            "else in this test.",
        )
        self.assertEqual(
            len(merge_bearing_layers),
            _EXPECTED_MERGE_BEARING_LAYERS,
            "expected merges on exactly {} of 71 body layers (the band "
            "this fixture's own docstring cites); "
            "got {} merge-bearing layers: {}.".format(
                _EXPECTED_MERGE_BEARING_LAYERS,
                len(merge_bearing_layers),
                merge_bearing_layers,
            ),
        )

        # PHYSICAL CONTINUITY: the merged footprint's own edges genuinely
        # are its extreme members' own edges, and those two extreme
        # members are genuinely different families -- not merely a
        # membership tuple that says so.
        self.assertEqual(
            footprint_mismatches,
            [],
            "a merged intent's footprint does not coincide with its own "
            "extreme members' real edges: {}".format(footprint_mismatches),
        )
        self.assertEqual(
            same_family_extremes,
            [],
            "a merged intent's own leftmost and rightmost constituent "
            "share a family_index -- the merge's own membership tuple "
            "could still read as two families while the actual edge "
            "members are the same one: {}".format(same_family_extremes),
        )


class MergedSlotWidthAtTheBandTests(unittest.TestCase):
    """The merge definition: "the two candidate slots become one slot spanning
    the union interval" ``[a_min_x, b_max_x]``.

    Uses the same hand-built-``WebIntent`` + ``_merge_cross_family_nodes``
    entry point ``test_node_merge_scope.py`` uses to control the exact edge
    gap; that private method is the place that decides what happens to a
    close cross-family pair.

    Expected width is derived independently, from the union-interval
    definition itself (``a_min_x`` to ``b_max_x``, NOT copied from any
    production arithmetic): for two candidates of width ``w_a``/``w_b``
    separated by edge gap ``gap``, the union interval spans
    ``w_a + gap + w_b`` -- station A's own footprint, plus the gap between
    them, plus station B's own footprint.

    The pair must produce a single merged-width slot, not one candidate
    surviving at its ORIGINAL, un-merged width with the other recorded as
    omitted.
    """

    def _run(self, gap_mm, width_mm=0.021):
        a = _mk_intent(0.0, width_mm, family_index=0)
        b_min_x = width_mm / 2.0 + gap_mm
        b_center = b_min_x + width_mm / 2.0
        b = _mk_intent(b_center, width_mm, family_index=1)
        gen = StructureGenerator()
        kept, suppressed = gen._merge_cross_family_nodes(
            [a, b], safety_offset_mm=_SAFETY_OFFSET_MM, ctx=_MERGE_TEST_CTX
        )
        return kept, suppressed

    def _assert_single_merged_slot(self, gap_mm, width_mm=0.021):
        kept, suppressed = self._run(gap_mm, width_mm)
        expected_width = width_mm + gap_mm + width_mm
        self.assertEqual(
            len(kept),
            1,
            "gap={} mm: expected exactly ONE merged slot spanning the "
            "union interval; got {} kept intent(s) and {} omission(s): "
            "kept={}, omitted={}".format(
                gap_mm,
                len(kept),
                len(suppressed),
                [(i.family_index, i.start[0], i.width_mm) for i in kept],
                [(o.family_name, o.x_mm, o.reason) for o in suppressed],
            ),
        )
        merged = kept[0]
        self.assertAlmostEqual(
            merged.width_mm,
            expected_width,
            places=9,
            msg=(
                "gap={} mm: merged slot width {} mm does not match the "
                "union-interval width w_a + gap + w_b = {} mm".format(
                    gap_mm, merged.width_mm, expected_width
                )
            ),
        )
        self.assertEqual(
            suppressed,
            [],
            "gap={} mm: a true merge records no omission at all -- got "
            "{}".format(gap_mm, suppressed),
        )

    def test_merge_at_gap_0_80mm(self):
        self._assert_single_merged_slot(0.80)

    def test_merge_at_gap_0_40mm(self):
        self._assert_single_merged_slot(0.40)

    def test_merge_at_gap_0_10mm(self):
        self._assert_single_merged_slot(0.10)


class BoundaryCaseTests(unittest.TestCase):
    """Boundary fixtures at gap = 0, gap = min_wall, and both +/- epsilon.

    Intended behaviour (band is ``0 <= gap < min_wall``):

    - ``gap == 0.0`` (band's lower edge, inclusive): merges.
    - ``gap`` just below ``min_wall`` (0.84 mm - epsilon): merges.
    - ``gap == min_wall`` exactly (0.84 mm, band's upper edge, EXCLUSIVE):
      stays separate, untouched -- two slots, no merge, no suppression.
    - ``gap`` just above ``min_wall`` (0.84 mm + epsilon): stays separate,
      untouched.
    - ``gap < 0`` (overlap case): OUT OF SCOPE for the band rule -- both
      candidates survive untouched, no suppression, no merge. This is the
      scope boundary; guarding it here is deliberate.

    The at-or-above-``min_wall`` and ``gap < 0`` cases (nothing fires either
    way) are an explicit regression guard on the scope boundary.
    """

    _EPSILON_MM = 1e-3

    def _run(self, gap_mm, width_mm=0.021):
        a = _mk_intent(0.0, width_mm, family_index=0)
        b_min_x = width_mm / 2.0 + gap_mm
        b_center = b_min_x + width_mm / 2.0
        b = _mk_intent(b_center, width_mm, family_index=1)
        gen = StructureGenerator()
        return gen._merge_cross_family_nodes(
            [a, b], safety_offset_mm=_SAFETY_OFFSET_MM, ctx=_MERGE_TEST_CTX
        )

    def test_gap_exactly_zero_merges(self):
        width_mm = 0.021
        kept, suppressed = self._run(0.0, width_mm)
        self.assertEqual(
            len(kept),
            1,
            "gap=0.0 mm (band's lower edge, inclusive) should merge into "
            "one slot; got kept={}, suppressed={}".format(
                [(i.family_index, i.width_mm) for i in kept],
                [(o.family_name, o.reason) for o in suppressed],
            ),
        )
        self.assertAlmostEqual(kept[0].width_mm, 2 * width_mm, places=9)

    def test_gap_just_below_min_wall_merges(self):
        width_mm = 0.021
        gap = _MIN_WALL_MM - self._EPSILON_MM
        kept, suppressed = self._run(gap, width_mm)
        self.assertEqual(
            len(kept),
            1,
            "gap={} mm (min_wall - epsilon, inside the band) should merge; "
            "got kept={}, suppressed={}".format(
                gap,
                [(i.family_index, i.width_mm) for i in kept],
                [(o.family_name, o.reason) for o in suppressed],
            ),
        )
        self.assertAlmostEqual(
            kept[0].width_mm, width_mm + gap + width_mm, places=9
        )

    def test_gap_exactly_min_wall_stays_separate(self):
        width_mm = 0.021
        kept, suppressed = self._run(_MIN_WALL_MM, width_mm)
        self.assertEqual(
            len(kept),
            2,
            "gap=min_wall={} mm (band's upper edge, exclusive) must NOT "
            "merge -- both candidates should survive separately".format(
                _MIN_WALL_MM
            ),
        )
        self.assertEqual(suppressed, [])

    def test_gap_just_above_min_wall_stays_separate(self):
        width_mm = 0.021
        gap = _MIN_WALL_MM + self._EPSILON_MM
        kept, suppressed = self._run(gap, width_mm)
        self.assertEqual(len(kept), 2)
        self.assertEqual(suppressed, [])

    def test_negative_gap_coincident_entry_stays_d009_territory(self):
        # An overlapping same-side cross-family pair uses the same freshly measured union node as the
        # rest of the sub-wall band.
        width_mm = 0.021
        gap = -0.01
        kept, suppressed = self._run(gap, width_mm)
        self.assertEqual(
            len(kept),
            2,
            "gap={} mm (negative -- true overlap) must produce one safe "
            "union node".format(gap),
        )
        self.assertEqual(suppressed, [])
        self.assertTrue(all(not i.merged_family_indices for i in kept))


class OrderIndependenceTests(unittest.TestCase):
    """Stage C acceptance criterion: "Merging is order-independent. Chained
    and multi-family merges converge to the same result regardless of scan
    order, proven the way ``_merge_cross_family_nodes`` proved it."

    Three stations chain across the band: A (family 0) close to B
    (family 1), B close to C (family 0). A and C are the same family and far
    apart from each other directly, so only the A-B and B-C cross-family
    pairs are, on their own, inside the band -- but a correct merge must
    still resolve the whole chain to ONE slot spanning the full union
    interval from A's near edge to C's far edge, regardless of which order
    the three candidates are handed in.

    The result must be ONE merged slot, not TWO surviving family-0 slots (A
    and C, at their original widths) with B dropped in between; and it must
    not depend on input order.
    """

    def _chain_intents(self, width_mm=0.021, gap_ab=0.10, gap_bc=0.10):
        a = _mk_intent(0.0, width_mm, family_index=0)
        b_min_x = width_mm / 2.0 + gap_ab
        b_center = b_min_x + width_mm / 2.0
        b = _mk_intent(b_center, width_mm, family_index=1)
        c_min_x = b_center + width_mm / 2.0 + gap_bc
        c_center = c_min_x + width_mm / 2.0
        c = _mk_intent(c_center, width_mm, family_index=0)
        return a, b, c, c_center + width_mm / 2.0 - (0.0 - width_mm / 2.0)

    def test_three_way_chain_merges_to_one_slot_regardless_of_input_order(
        self,
    ):
        a, b, c, expected_width = self._chain_intents()
        gen = StructureGenerator()

        orderings = [
            [a, b, c],
            [c, b, a],
            [b, a, c],
            [b, c, a],
            [a, c, b],
            [c, a, b],
        ]
        results = []
        for order in orderings:
            kept, suppressed = gen._merge_cross_family_nodes(
                list(order), safety_offset_mm=_SAFETY_OFFSET_MM, ctx=_MERGE_TEST_CTX
            )
            results.append(
                (
                    tuple(
                        sorted(
                            (round(i.start[0], 9), round(i.width_mm, 9))
                            for i in kept
                        )
                    ),
                    len(suppressed),
                )
            )

        first = results[0]
        for order, result in zip(orderings, results):
            self.assertEqual(
                result,
                first,
                "input order changed the merge outcome: got {} for one "
                "ordering and {} for another; the rule must converge to "
                "the same fixed point regardless of scan order (see "
                "``_merge_cross_family_nodes``'s own docstring for the "
                "precedent this criterion is measured against)".format(
                    first, result
                ),
            )

        # And the converged result must actually BE the single merged slot
        # spanning the whole chain's union interval -- not merely
        # order-independent nonsense (e.g. two survivors, consistently).
        kept_widths_and_count, suppressed_count = first
        self.assertEqual(
            len(kept_widths_and_count),
            1,
            "expected the whole 3-station chain to converge to ONE merged "
            "slot; got {} kept intent(s): {}".format(
                len(kept_widths_and_count), kept_widths_and_count
            ),
        )
        self.assertAlmostEqual(
            kept_widths_and_count[0][1], expected_width, places=9
        )


class FamilyMembershipCollisionHazardTests(unittest.TestCase):
    """A merged intent's representative ``family_index`` must not mislead the
    same-family collision check.

    THE HAZARD: ``TopologyValidator``'s pairwise collision check decides "is
    this close pair same-family or cross-family". A merged intent's
    ``family_index`` is only a REPORTING-level REPRESENTATIVE (always the
    lowest family_index among the component's members, see
    ``_merge_cross_family_nodes``'s own docstring), not necessarily the family
    of whichever constituent happens to border a given edge. So a merged
    component sitting within ``min_wall_mm`` of an UNRELATED,
    correctly-unmerged, SAME-family candidate could have its representative
    label happen not to match that neighbour's family -- misclassifying a
    genuine same-family collision as an intended cross-family junction and, if
    the two footprints actually overlap (``gap < 0``), letting it through the
    branch SILENTLY, with no error at all. That would be a silent weakening of
    the same-family invariant. The validator
    (``aircraft_core/validate.py``'s ``_family_membership`` + set-intersection
    rule, fed by ``WebIntent.family_membership()``) closes it by comparing each
    slot's FULL family membership, not a single label.

    CONSTRUCTING THE SCENARIO: three hand-built ``WebIntent``s are fed
    through the real ``_merge_cross_family_nodes`` entry point (the same one
    every other test in this file uses to control an exact edge gap) rather
    than through ``StructureSpec``'s pitch/position/drift arithmetic.
    Reproducing this exact scenario (a merge whose representative label does
    not equal one of its own two constituent families from the perspective of
    a specific neighbour, AND a same-family candidate positioned to genuinely
    OVERLAP the merged footprint) through real ``rib_pitch_mm``/drift
    arithmetic would need station spacing tighter than the requested slot
    width, which is a degenerate config in its own right and no less
    synthetic, with far less control over which hazard condition fires. The
    merge code under test is the real generator code either way; only the
    INPUT construction bypasses spec arithmetic.

    LIMITATION: the
    ``MultiNotchedRect`` fixture below holds two notches that genuinely
    OVERLAP in X, which its own docstring in stub_ops.py flags as
    "not re-validated here; callers must construct sane fixtures" -- i.e.
    outside what it is documented to model faithfully. This is safe HERE,
    specifically, because ``TopologyValidator``'s per-slot measurement
    (``ops.intersection(original, slot)``, feeding the collision check
    this test exercises) reads ``original`` and each entry of
    ``slot_polygons`` directly -- it never reads ``transformed_polygons``'s
    own notch structure at all. ``transformed_polygons`` is consulted only
    by invariants 1/2/5/6 (whole-layer validity/island-count/hole-count/
    "something was removed"), all of which this stub answers from
    ``contour_count()`` (hard-coded to 1) and ``area()`` (a plain
    subtraction that, for two overlapping notches, UNDER-counts remaining
    area rather than over-counting it -- so invariant 6's "some material
    was removed" check, if anything, passes with MORE margin, never less)
    -- neither of which raises or gives a wrong answer for this specific
    check path. The whole-layer erosion/connectivity check (which DOES
    lean on ``MultiNotchedRect``'s narrower, same-bridge-depth erosion
    model) is never reached: the collision check under test raises first.
    A genuinely accurate overlapping-notch erosion round trip still needs
    the real Clipper-backed host, same as every other overlap fixture in
    this codebase.
    """

    _SAFETY_OFFSET_MM = 0.42
    _MIN_WALL_MM = 2.0 * _SAFETY_OFFSET_MM

    def _build_scenario(self):
        """fwd (family 0) and aft (family 1) merge (cross-family, in-band).
        aft2 (family 1, SAME family as aft) sits right after aft in sorted
        order, close enough that a real config could plausibly place it
        there, and OVERLAPS the merged footprint's far edge once fwd+aft
        are unioned -- but aft2 is never linked into the merge (same-family
        pairs are never linked, by design), so it reaches the validator as
        an unrelated, unmerged, same-family neighbour.

        The merge scan compares ALL pairs, not only consecutive ones, so aft2
        is placed clear of fwd (gap 0.9 mm, outside the 0.84 mm band): if aft2
        were inside the band of fwd itself, fwd and aft2 would belong in the
        SAME component regardless of aft's presence and the fixture would
        collapse to one merged component instead of the two it needs. aft2
        still overlaps aft's own far edge (gap -0.1 mm), preserving the
        hazard: a same-family neighbour genuinely overlapping a merged
        component whose REPRESENTATIVE label does not equal that neighbour's
        family.
        """
        fwd = WebIntent(
            start=(5.0, 20.0), end=(5.0, 1.0), width_mm=0.6,
            family_index=0, family_name="fwd",
        )  # footprint [4.7, 5.3]
        aft = WebIntent(
            start=(6.0, 20.0), end=(6.0, 1.0), width_mm=0.6,
            family_index=1, family_name="aft",
        )  # footprint [5.7, 6.3] -- gap to fwd is 0.4 mm, inside the band.
        aft2 = WebIntent(
            start=(6.5, 20.0), end=(6.5, 1.0), width_mm=0.6,
            family_index=1, family_name="aft2",
        )  # footprint [6.2, 6.8] -- OVERLAPS aft's own footprint by 0.1 mm
        # (gap -0.1 mm), but same family as aft, so never linked. Gap to
        # fwd directly is 0.9 mm -- clear of the 0.84 mm band, so the
        # all-pairs scan does not pull aft2 into fwd's component either.

        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [fwd, aft, aft2], safety_offset_mm=self._SAFETY_OFFSET_MM, ctx=_MERGE_TEST_CTX
        )
        return kept, omitted

    def test_scenario_is_constructible_one_merge_plus_one_unmerged_neighbour(self):
        # Regression pin on the scenario itself, independent of the
        # validator: proves the hazard's PRECONDITIONS actually hold before
        # asserting anything about the validator's response to them.
        kept, omitted = self._build_scenario()
        self.assertEqual(omitted, [])
        self.assertEqual(len(kept), 2)
        by_x = sorted(kept, key=lambda i: i.start[0])
        merged, standalone = by_x[0], by_x[1]

        # The merge really did absorb both fwd and aft.
        self.assertEqual(merged.merged_family_indices, (0, 1))
        merged_min_x = merged.start[0] - merged.width_mm / 2.0
        merged_max_x = merged.start[0] + merged.width_mm / 2.0
        self.assertAlmostEqual(merged_min_x, 4.7, places=6)
        self.assertAlmostEqual(merged_max_x, 6.3, places=6)

        # aft2 really did survive standalone, unmerged, same family as aft.
        self.assertEqual(standalone.merged_family_indices, ())
        self.assertEqual(standalone.family_index, 1)
        standalone_min_x = standalone.start[0] - standalone.width_mm / 2.0
        self.assertAlmostEqual(standalone_min_x, 6.2, places=6)

        # And the two genuinely overlap -- the dangerous (gap < 0) case,
        # not merely a close approach.
        gap = standalone_min_x - merged_max_x
        self.assertLess(gap, 0.0)

        # The merge's own REPRESENTATIVE label is family 0 ("fwd") -- NOT
        # the same as aft2's true family (1) -- even though the merge's
        # true membership DOES include family 1 (via "aft"). This is the
        # exact label/membership mismatch the hazard depends on.
        self.assertEqual(merged.family_index, 0)
        self.assertNotEqual(merged.family_index, standalone.family_index)
        self.assertIn(standalone.family_index, merged.merged_family_indices)

    def test_merged_component_overlapping_unmerged_same_family_neighbour_fails_closed(
        self,
    ):
        kept, _omitted = self._build_scenario()
        by_x = sorted(kept, key=lambda i: i.start[0])
        merged, standalone = by_x[0], by_x[1]

        outer = Rect(0.0, 0.0, 20.0, 20.0)
        merged_notch = Rect(
            merged.start[0] - merged.width_mm / 2.0, 1.0,
            merged.start[0] + merged.width_mm / 2.0, 20.0,
        )
        standalone_notch = Rect(
            standalone.start[0] - standalone.width_mm / 2.0, 1.0,
            standalone.start[0] + standalone.width_mm / 2.0, 20.0,
        )
        transformed = MultiNotchedRect(
            outer=outer, notches=(merged_notch, standalone_notch)
        )
        spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)
        validator = TopologyValidator(StubOps(), safety_offset_mm=self._SAFETY_OFFSET_MM)

        # Comparing bare ``family_index`` (0 vs 1) would call this pair
        # "different families", and since their measured footprints
        # genuinely overlap (gap < 0), the collision check would take the
        # merge branch and return WITHOUT RAISING -- silently treating a real
        # same-family collision as an intended junction. Comparing full
        # family membership makes this raise, naming the same-family cause.
        with self.assertRaises(TopologyError) as ctx:
            validator.validate(
                [outer],
                [transformed],
                [merged_notch, standalone_notch],
                spec,
                183,
                36.6,
                slot_labels=[
                    "family {} ({!r})".format(merged.family_index, merged.family_name),
                    "family {} ({!r})".format(standalone.family_index, standalone.family_name),
                ],
                family_indices=[
                    merged.family_membership(),
                    standalone.family_membership(),
                ],
            )
        message = str(ctx.exception)
        # The SAME-FAMILY message, not the cross-family "does not actually
        # overlap" one -- proves the intersection rule, not just that SOME
        # error fired.
        self.assertIn("would merge into one slot or sever", message)
        self.assertNotIn("intended web crossing", message)


class FamilyIndicesBackwardCompatibilityTests(unittest.TestCase):
    """Confirms the ``family_indices`` parameter is an EXACT reduction to
    plain per-slot family semantics for standalone (unmerged) intents --
    asserted, not assumed.

    Every pair below is run through ``TopologyValidator.validate()`` TWICE:
    once with ``family_indices`` in the plain-``int``-per-slot form, once
    in the membership-set form (a singleton tuple per slot, the form
    ``WebIntent.family_membership()`` produces for an unmerged web). Both must
    produce byte-identical outcomes -- same exception raised or not, same
    message when one is -- because for two singleton sets,
    intersection-non-empty is exactly equality (see
    ``aircraft_core/validate.py``'s ``_family_membership`` docstring).
    """

    def _run_and_capture(self, family_indices):
        outer = Rect(0.0, 0.0, 20.0, 20.0)
        validator = TopologyValidator(StubOps(), safety_offset_mm=0.42)
        spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)
        try:
            validator.validate(
                [outer],
                [self._transformed],
                list(self._notches),
                spec,
                183,
                36.6,
                family_indices=family_indices,
            )
        except TopologyError as exc:
            return str(exc)
        return None

    def _assert_identical_outcome(self, int_form, set_form):
        outer = Rect(0.0, 0.0, 20.0, 20.0)
        self._transformed = MultiNotchedRect(outer=outer, notches=self._notches)
        outcome_int = self._run_and_capture(int_form)
        outcome_set = self._run_and_capture(set_form)
        self.assertEqual(
            outcome_int,
            outcome_set,
            "the plain-int and singleton-set forms of family_indices "
            "diverged -- the parameter change is not an exact reduction",
        )
        return outcome_int

    def test_same_family_close_pair_still_fails_closed_identically(self):
        # Mirrors test_junction.py's own same-family fixture.
        self._notches = (
            Rect(4.7, 1.0, 5.3, 20.0),
            Rect(5.32, 1.0, 6.0, 20.0),
        )
        outcome = self._assert_identical_outcome(
            int_form=[0, 0], set_form=[(0,), (0,)]
        )
        self.assertIsNotNone(outcome)
        self.assertIn("would merge into one slot or sever", outcome)

    def test_cross_family_overlapping_pair_still_merges_identically(self):
        self._notches = (
            Rect(4.7, 1.0, 5.3, 20.0),
            Rect(5.2, 1.0, 5.8, 20.0),
        )
        outcome = self._assert_identical_outcome(
            int_form=[0, 1], set_form=[(0,), (1,)]
        )
        self.assertIsNone(outcome)

    def test_cross_family_close_non_overlapping_pair_still_fails_closed_identically(
        self,
    ):
        self._notches = (
            Rect(4.7, 1.0, 5.3, 20.0),
            Rect(5.32, 1.0, 6.0, 20.0),
        )
        outcome = self._assert_identical_outcome(
            int_form=[0, 1], set_form=[(0,), (1,)]
        )
        self.assertIsNotNone(outcome)
        self.assertIn("intended web crossing", outcome)

    def test_family_membership_helper_normalizes_both_forms_identically(self):
        # Direct, white-box confirmation of the reduction the tests above
        # exercise indirectly: a plain int and its singleton-tuple
        # equivalent normalize to the identical frozenset.
        self.assertEqual(_family_membership(0), frozenset({0}))
        self.assertEqual(_family_membership((0,)), frozenset({0}))
        self.assertEqual(_family_membership(0), _family_membership((0,)))
        self.assertEqual(_family_membership((0, 1)), frozenset({0, 1}))
        # Two singleton sets intersect iff their values are equal --
        # exactly the plain equality truth table.
        self.assertFalse(_family_membership(0).isdisjoint(_family_membership(0)))
        self.assertTrue(_family_membership(0).isdisjoint(_family_membership(1)))


if __name__ == "__main__":
    unittest.main()
