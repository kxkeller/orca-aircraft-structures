"""The union-interval node merge, exercised through GENERATOR fixtures.

Instead of dropping one family's candidate at a close-but-not-overlapping
cross-family node, ``StructureGenerator`` merges both into ONE ``WebIntent``
spanning the union of their footprints. See ``aircraft_core/generator.py``'s
module docstring for the derivation and ``_merge_cross_family_nodes``'s own
docstring for the union-find mechanics and the representative-family
reporting choice (a fixed rule: always the lowest family_index).

The real-world scenarios pinned here are the 7-simultaneous-node NACA 2412
failure, the deterministic-regardless-of-order proof, and the
same-family/overlap scope guards.

SPLIT FROM ``tests/unit/test_node_merge.py``, kept deliberately rather than
folded together: that file is the primary SPEC for the merge rule itself (the
union-interval definition, the boundary cases, order-independence), built from
hand-controlled, precise-gap fixtures. This file is the GENERATOR-level,
REAL-FIXTURE regression coverage: the real NACA 2412 7-node-row layer, the
whole-layer validator round trip on a merged-node layer, and the
determinism/scope guards. It mirrors the split ``test_junction.py`` draws
between its ``JunctionValidatorTests`` (hand-built) and
``DiamondGeneratorTests`` (real-fixture) classes; keeping them apart makes it
clear which failure means "the rule is wrong" versus "the real config
regressed".
"""
import importlib.util
import unittest
from pathlib import Path

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator, WebIntent
from aircraft_core.slot import SlotBuilder
from aircraft_core.spec import StructureSpec, WebFamily
from aircraft_core.validate import TopologyValidator
from stub_ops import MultiNotchedRect, PolygonShape, Rect, StubOps

# ``_merge_cross_family_nodes``/``_merge_node_component`` REQUIRE a
# ``LayerContext`` -- the merge-halo breach raises a ``TopologyError`` that
# must name a real layer index and Z, and an optional ctx defaulting to
# ``-1``/``0.0`` could emit a diagnostic naming the wrong layer. These direct-call tests do not exercise that raise, so any
# concrete context serves; it is named rather than inlined so it is obvious
# these values are inert here.
_MERGE_TEST_CTX = LayerContext(layer_index=0, print_z=0.0)


REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_node_merge_scope", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_mf = _load_make_fixtures_module()


def _naca2412_polygon():
    """Same 60 mm chord / 60 mm span NACA 2412 section as test_junction.py's own
    ``_naca2412_polygon()`` -- the exact fixture was measured against."""
    m, p, t = _mf.parse_naca4("2412")
    _triangles, outline_mm, bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm)), bbox


def _rect_from_points(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return Rect(min(xs), min(ys), max(xs), max(ys))


# The shipped coupon numbers, applied to the airfoil -- exactly the config's
# "Finding -- 45 degrees and overlap-only merging are mutually exclusive" were
# both measured against (rib_pitch 8.0, positions 0.4/0.6, drift +/-1.0, period
# 12, gap_ratio 0.05, wall 0.42 mm -- host/data/orca_plugins/config.json).
def _shipped_diamond_spec():
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


class NodeRowAndSubBeadTests(unittest.TestCase):
    """The real failure this decision was forced by, reproduced through the
    actual generator, not a hand-picked fixture.

    : layer 5, Z=1.20 mm on the crossing NACA 2412 coupon, every one of 7
    fwd/aft station pairs (both families share rib_pitch 8.0, so they are
    translates -- every fwd station crosses an aft station on the same layer)
    leaves 0.3783 mm of wall -- POSITIVE, not overlapping, below the 0.84 mm
    minimum. Under alone this failed the whole slice closed. Under it
    suppressed one family's candidate at each node. Under it MERGES both into
    one slot per node, at every one of the 7 nodes.
    """

    def setUp(self):
        self.ops = StubOps()
        self.polygon, _bbox = _naca2412_polygon()
        self.frame = LayerFrame.from_polygon(self.ops, self.polygon)
        self.spec = _shipped_diamond_spec()
        self.gen = StructureGenerator()
        self.layer = LayerContext(layer_index=5, print_z=1.20)

    def test_seven_simultaneous_sub_bead_crossings_merge_not_fail(self):
        result = self.gen.generate(self.frame, self.spec, 0.42, self.layer)

        # Every one of the 7 node pairs merges into exactly one WebIntent
        # spanning the union of its two constituents' footprints -- no omission
        # at all ("no family is dropped"). Computed directly
        # against this exact fixture: 7
        # merged intents, 0 omitted, width 0.4203 mm each (0.021 mm + 0.3783 mm
        # gap + 0.021 mm -- the same 0.3783 mm gap itself pins).
        self.assertEqual(len(result.intents), 7)
        self.assertEqual(result.omitted, [])
        for intent in result.intents:
            self.assertEqual(
                intent.merged_family_indices,
                (0, 1),
                "every node here is a two-family merge",
            )
            self.assertAlmostEqual(intent.width_mm, 0.4203, places=3)

        # No suppression reason exists: node handling merges rather than drops,
        # as the empty omitted list above shows.
        self.assertFalse(
            any(o.reason.startswith("node suppression:") for o in result.omitted)
        )

    def test_whole_layer_invariants_hold_on_a_merged_node_layer(self):
        # HONEST GAP: StubOps cannot diff/erode 7 curved-boundary notches on
        # the real NACA polygon at once (its analytic backends only model a
        # rectangular coupon for more-than-one-notch, and a single notch for a
        # curved polygon -- see stub_ops.py's module docstring). So this
        # whole-layer invariant check runs the same shipped config on the
        # rectangular coupon, where StubOps' MultiNotchedRect CAN diff and
        # erode an arbitrary number of same-bridge-depth notches exactly -- the
        # same "validator-generic, honest gap" split test_junction.py documents
        # for multi-notch curved-boundary round trips. A full curved-boundary
        # round trip needs the real Clipper-backed host, not this test double.
        ops = StubOps()
        rect = Rect(0.0, 0.0, 60.0, 20.0)
        frame = LayerFrame.from_polygon(ops, rect)
        gen = StructureGenerator()
        result = gen.generate(frame, self.spec, 0.42, self.layer)
        self.assertGreater(len(result.intents), 0)
        # A true merge never produces an omission.
        self.assertEqual(result.omitted, [])
        merged = [i for i in result.intents if i.merged_family_indices]
        self.assertGreater(
            len(merged),
            0,
            "this fixture is expected to exercise at least one merge",
        )

        builder = SlotBuilder()
        built = [builder.build(i, self.spec.overshoot_mm) for i in result.intents]
        notches = tuple(_rect_from_points(b.points) for b in built)
        transformed = MultiNotchedRect(outer=rect, notches=notches)

        validator = TopologyValidator(ops, safety_offset_mm=0.42)
        slot_labels = [
            "family {} ({!r})".format(i.family_index, i.family_name)
            for i in result.intents
        ]
        family_indices = [i.family_index for i in result.intents]
        # Should not raise: one island, no holes, despite several merged
        # nodes on the same layer.
        validator.validate(
            [rect],
            [transformed],
            list(notches),
            self.spec,
            5,
            1.20,
            slot_labels=slot_labels,
            family_indices=family_indices,
        )


class NodeMergeScopeTests(unittest.TestCase):
    """Determinism and scope of the merge.

    There is no tie-break between the two families: both are merged into one
    intent (see ``_merge_cross_family_nodes``'s own docstring for the
    reporting-only, fixed-rule representative choice -- always the lowest
    family_index). The scope guards are same-family proximity and
    cross-family overlap.
    """

    def setUp(self):
        self.ops = StubOps()
        # A generous rectangular coupon: wide and tall enough that margin/
        # thin-section omission never interferes with the merge checks
        # under test.
        self.rect = Rect(0.0, 0.0, 60.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)
        self.gen = StructureGenerator()

    def _crossing_spec(self):
        fam_fwd = WebFamily(name="fwd", web_position=0.4, z_drift_mm_per_mm=0.1)
        fam_aft = WebFamily(name="aft", web_position=0.6, z_drift_mm_per_mm=-0.1)
        return StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(fam_fwd, fam_aft)
        )

    def _near_node_layer(self):
        # z_cross for this family pair is Z=60.0 (see test_junction.py's
        # DiamondGeneratorTests._crossing_z for the derivation); 5.0 mm
        # short of it sits inside the merge band -- see test_junction.py's
        # ``test_coupon_two_families_cross_at_the_expected_z`` for the band
        # arithmetic pinned against this exact fixture.
        return LayerContext(layer_index=0, print_z=55.0)

    def test_determinism_repeated_calls_same_config_same_layer(self):
        spec = self._crossing_spec()
        layer = self._near_node_layer()

        results = [
            self.gen.generate(self.frame, spec, 0.42, layer) for _ in range(5)
        ]
        first = results[0]
        first_kept = [
            (i.family_name, round(i.start[0], 9), round(i.width_mm, 9))
            for i in first.intents
        ]
        for other in results[1:]:
            kept = [
                (i.family_name, round(i.start[0], 9), round(i.width_mm, 9))
                for i in other.intents
            ]
            self.assertEqual(kept, first_kept)
            self.assertEqual(other.omitted, [])

    def test_determinism_independent_of_intent_construction_order(self):
        # Direct, white-box confirmation of the merge's own claim: feeding
        # ``_merge_cross_family_nodes`` the identical two candidates in
        # either insertion order must yield the identical merged outcome --
        # the union interval is a pure function of each candidate's own
        # position, never of which order the pair was discovered or
        # appended in.
        fwd = WebIntent(
            start=(-23.80035, 3.0),
            end=(-23.80035, -2.0),
            width_mm=0.021,
            family_index=0,
            family_name="fwd",
        )
        aft = WebIntent(
            start=(-24.19965, 3.0),
            end=(-24.19965, -2.0),
            width_mm=0.021,
            family_index=1,
            family_name="aft",
        )
        kept_fwd_first, omitted_fwd_first = self.gen._merge_cross_family_nodes(
            [fwd, aft], safety_offset_mm=0.42, ctx=_MERGE_TEST_CTX
        )
        kept_aft_first, omitted_aft_first = self.gen._merge_cross_family_nodes(
            [aft, fwd], safety_offset_mm=0.42, ctx=_MERGE_TEST_CTX
        )

        self.assertEqual(
            [
                (i.family_name, round(i.start[0], 9), round(i.width_mm, 9))
                for i in kept_fwd_first
            ],
            [
                (i.family_name, round(i.start[0], 9), round(i.width_mm, 9))
                for i in kept_aft_first
            ],
        )
        self.assertEqual(omitted_fwd_first, [])
        self.assertEqual(omitted_aft_first, [])
        # A true merge, not two survivors: exactly one WebIntent, carrying both
        # family indices, spanning the union interval (both original 0.021 mm
        # footprints plus the 0.3783 mm gap between them --'s own "Why this was
        # forced" note pins that exact gap on this exact fixture).
        self.assertEqual(len(kept_fwd_first), 1)
        self.assertEqual(kept_fwd_first[0].merged_family_indices, (0, 1))
        self.assertAlmostEqual(kept_fwd_first[0].width_mm, 0.4203, places=3)

    def test_merge_does_not_fire_for_same_family_proximity(self):
        # A single family, packed far tighter than the minimum wall could ever
        # tolerate -- an placement mistake, not a node. Merging must never fire
        # for it (a component is never formed from a same-family pair, however
        # close); only TopologyValidator's own pre-existing collision check
        # (unaffected) is the arbiter of a same-family close pair, and this
        # test does not reach it -- it only proves the GENERATOR never links
        # same-family candidates.
        fam = WebFamily(name="solo", rib_pitch_mm=0.5, web_position=0.5)
        spec = StructureSpec(slot_gap_mm=0.3, skin_margin_mm=1.0, families=(fam,))
        result = self.gen.generate(
            self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.2)
        )
        # Every omission here is an ordinary chord-extent miss at the
        # coupon's own edges (a single family packed to its own pitch runs
        # off the end of a 60 mm chord) -- never a merge concern, and never
        # a "node suppression: ..." reason.
        self.assertTrue(
            all(o.reason.startswith("chord extent:") for o in result.omitted)
        )
        self.assertTrue(all(not i.merged_family_indices for i in result.intents))
        # Several same-family stations really were generated (packed at
        # 0.5 mm pitch on a 60 mm-wide coupon) -- this is not a vacuous
        # "nothing to merge" pass.
        self.assertGreater(len(result.intents), 1)

    def test_coincident_entry_overlap_remains_unmerged(self):
        # a true overlap uses the same union-interval merge as a close node.
        # Independent depths cut an island from elevon layer 926 even though
        # classified the overlap as a valid junction.
        spec = self._crossing_spec()
        z_cross = 60.0
        result = self.gen.generate(
            self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=z_cross)
        )
        self.assertEqual(len(result.intents), 2)
        self.assertEqual(result.omitted, [])
        self.assertTrue(all(not i.merged_family_indices for i in result.intents))

    def test_merged_intent_carries_both_family_indices_and_no_omission(self):
        spec = self._crossing_spec()
        layer = self._near_node_layer()
        result = self.gen.generate(self.frame, spec, 0.42, layer)
        self.assertEqual(result.omitted, [])
        self.assertEqual(len(result.intents), 1)
        merged = result.intents[0]
        self.assertEqual(merged.merged_family_indices, (0, 1))
        # Width is the union-interval width: both original 0.6 mm footprints
        # plus the gap between them.
        self.assertGreater(merged.width_mm, 0.6)


if __name__ == "__main__":
    unittest.main()
