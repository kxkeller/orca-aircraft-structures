"""Crossing web families / junction topology.

Two cross-family slots are merged into one only when their MEASURED
footprints actually overlap in X; matching requested depth is not the
criterion, because equal requested depth cannot apply to a non-prismatic
(real) wing. Near-tangent pairs (positive gap below the minimum wall) fail
closed. The labelled tests below pin both sides of that rule.

Two levels, mirroring test_z_variation.py's split:

- ``TopologyValidator``-level fixtures (``JunctionValidatorTests``): hand
  -built slot/transformed geometry exercising the validator's merge-vs-fail
  decision and its message attribution directly, independent of whether the
  generator can produce the input.
- ``StructureGenerator``-level fixtures (``DiamondGeneratorTests``): two
  opposite-drift families really do converge to the same chord station at a
  closed-form Z, on both the coupon and the real NACA 2412 (a diamond
  lattice is two families drifting in opposite directions).
"""
import importlib.util
import unittest
from pathlib import Path

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.slot import SlotBuilder
from aircraft_core.spec import StructureSpec, WebFamily
from aircraft_core.validate import TopologyError, TopologyValidator
from stub_ops import HoledRect, MultiNotchedRect, NotchedRect, PolygonShape, Rect, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"

LAYER_INDEX = 183
PRINT_Z = 36.6

MIN_WALL_MM = 0.84  # 2 * 0.42 mm safety offset, matching test_validate.py.


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_junction", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_mf = _load_make_fixtures_module()


def _naca2412_polygon():
    m, p, t = _mf.parse_naca4("2412")
    _triangles, outline_mm, bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm)), bbox


class JunctionValidatorTests(unittest.TestCase):
    """``TopologyValidator``-level junction fixtures, on the rectangular
    coupon (mirrors ``TopologyValidatorMultiSlotTests`` in
    test_validate.py)."""

    def setUp(self):
        self.ops = StubOps()
        self.validator = TopologyValidator(self.ops, safety_offset_mm=0.42)
        self.original = Rect(0.0, 0.0, 20.0, 20.0)
        self.spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)

    # -- crossing near the skin / overlapping, equal depth: MERGE -----------

    def test_overlapping_equal_depth_cross_family_slots_merge_and_pass(self):
        # notch_a's right edge (5.3) is PAST notch_b's left edge (5.2): a
        # genuine overlap, not merely a too-close pair -- the "crossing"
        # case itself, not just near-tangent. Both slots share the same
        # Y-range [1.0, 20.0] (same depth policy), so their union is
        # exactly the rectangle [4.7, 6.0] x [1.0, 20.0] -- the ONE case
        # junction rule merges instead of failing closed.
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)
        notch_b = Rect(5.2, 1.0, 6.0, 20.0)
        transformed = NotchedRect(outer=self.original, notch=Rect(4.7, 1.0, 6.0, 20.0))
        # Should not raise.
        self.validator.validate(
            [self.original],
            [transformed],
            [notch_a, notch_b],
            self.spec,
            LAYER_INDEX,
            PRINT_Z,
            slot_labels=["family 0 ('fwd')", "family 1 ('aft')"],
            family_indices=[0, 1],
        )

    def test_near_tangent_equal_depth_cross_family_slots_fail_closed(self):
        # Two crossing-family slots are merged only when their footprints
        # actually overlap in X (gap < 0); matching requested depth is not
        # sufficient. Here the footprints only approach (0.02 mm apart, well
        # under the 0.84 mm minimum wall) without overlapping. Full-depth webs
        # on a non-prismatic section (any real wing) resolve to the LOCAL
        # surface at each chord station, so equal REQUESTED depth says nothing
        # about whether two crossing webs' footprints meet. A near-tangent
        # pair like this is exactly the sub-bead hazard: merging on proximity
        # alone would bless a wall thinner than one printable extrusion bead,
        # which the whole-layer erosion check cannot catch (a pillar between
        # two top-entry slots stays attached to the material below). So the
        # validator fails closed regardless of matching depth.
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)
        notch_b = Rect(5.32, 1.0, 6.0, 20.0)
        transformed = NotchedRect(outer=self.original, notch=Rect(4.7, 1.0, 6.0, 20.0))
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [notch_a, notch_b],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
                slot_labels=["family 0 ('fwd')", "family 1 ('aft')"],
                family_indices=[0, 1],
            )
        message = str(ctx.exception)
        self.assertIn("fwd", message)
        self.assertIn("aft", message)
        self.assertIn("different families", message)
        self.assertIn("0.0200 mm", message)
        self.assertIn("matching depths is no longer enough to merge", message)

    # -- mid-depth crossing with DIFFERING depth: fail closed, name why -----

    def test_close_differing_depth_cross_family_slots_fail_closed_naming_junction(self):
        # A deep rib and a shallow skin web approaching each other -- the
        # crossing families' union is not a plane rectangle (a stepped
        # shape the validator does not attempt to validate), so the collision
        # check fails closed, with a message that explains this is a
        # recognized junction shape, not the plain "too close" mistake.
        deep = Rect(4.7, 1.0, 5.3, 20.0)  # full depth
        shallow = Rect(5.32, 15.0, 6.0, 20.0)  # shallow: y in [15.0, 20.0]
        transformed = MultiNotchedRect(outer=self.original, notches=(deep, shallow))
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [deep, shallow],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
                slot_labels=["family 0 ('deep rib')", "family 1 ('shallow skin')"],
                family_indices=[0, 1],
            )
        message = str(ctx.exception)
        self.assertIn("deep rib", message)
        self.assertIn("shallow skin", message)
        self.assertIn("different families", message)
        self.assertIn("depths", message)
        self.assertIn("0.0200 mm", message)

    # -- overlapping, DIFFERING depth (synthetic): MERGE ---------------

    def test_overlapping_differing_depth_cross_family_slots_merge_and_pass(self):
        # The merge condition is overlap, not matching
        # depth. Same "deep rib" / "shallow skin" pair as the fail-closed
        # test just above, but shifted so the shallow web's footprint sits
        # INSIDE the deep rib's X span instead of merely approaching it --
        # a genuine overlap. The shallow notch is also fully contained in
        # the deep notch's Y-range, so the true union is EXACTLY the deep
        # notch: this keeps the fixture exact for this stub (see
        # ``StubOps``'s module docstring on ``MultiNotchedRect``/
        # ``NotchedRect``) rather than approximating a stepped shape.
        # Differing depth does not matter here: the pair merges because the
        # footprints overlap.
        deep = Rect(4.7, 1.0, 5.3, 20.0)  # full depth, touches top
        shallow = Rect(4.9, 15.0, 5.1, 20.0)  # shallow, X range INSIDE deep's
        transformed = NotchedRect(outer=self.original, notch=deep)
        # Should not raise.
        self.validator.validate(
            [self.original],
            [transformed],
            [deep, shallow],
            self.spec,
            LAYER_INDEX,
            PRINT_Z,
            slot_labels=["family 0 ('deep rib')", "family 1 ('shallow skin')"],
            family_indices=[0, 1],
        )

    # -- the airfoil failure geometry, minimized
    # (differing depth, overlapping): MERGE
    # ----------------------------------------------------

    def test_realistic_airfoil_overlapping_differing_depth_slots_merge_and_pass(self):
        # Minimized from a real failure geometry: layer 5, Z = 1.20 mm on the crossing NACA airfoil coupon. That
        # layer's fwd-family station 0 and aft-family station 7 (``intents``,
        # ``slot_bounds_mm``) are only NEAR-tangent (0.3783 mm apart -- see
        # ``test_airfoil_sub_bead_near_tangent_gap_suppressed_per_d010_single_slot_passes``
        # below, which pins that exact case), not overlapping. This fixture
        # takes the SAME real per-family BOTTOM depth values (the fwd/aft
        # Y-ranges genuinely differ station to station on a cambered section,
        # which is the whole point) and moves the aft slot to where it actually
        # overlaps fwd in X -- exhibiting the case the overlap-merge rule exists for: a real,
        # differing-depth, genuinely overlapping cross-family pair. (This
        # stub's ``NotchedRect``/``MultiNotchedRect`` only model a single
        # moving boundary per notch -- see ``StubOps``'s module docstring -- so
        # both slots are pinned to the SAME top, matching this stub's
        # flat-topped coupon convention, and the real per-family depth
        # difference is carried entirely by the BOTTOM (``min_y``), which is
        # where the real data's difference is largest anyway; the real
        # airfoil's TOP also differs by a much smaller amount from local
        # curvature over the same 0.02 mm of X, which a rectangular coupon
        # cannot represent -- see the "sever"/ "hole" fixtures' own HONEST GAP
        # notes for the same limitation.) The aft footprint is placed fully
        # inside the (deeper) fwd footprint in both X and Y, so the true union
        # is exactly the fwd notch -- exact for this stub, not an approximation
        # of a stepped shape.
        fwd = Rect(-23.81085, -2.37907, -23.78985, 2.317064)  # real fwd station-0 depth
        aft = Rect(-23.80500, -2.339273, -23.79500, 2.317064)  # real aft station-7 bottom, pinned to fwd's top
        outer = Rect(-25.0, -5.0, -22.0, 2.317064)
        transformed = NotchedRect(outer=outer, notch=fwd)
        # Should not raise.
        self.validator.validate(
            [outer],
            [transformed],
            [fwd, aft],
            self.spec,
            5,
            1.20,
            slot_labels=["family 0 ('fwd')", "family 1 ('aft')"],
            family_indices=[0, 1],
        )

    # -- the sub-bead near-tangent case from the airfoil failure:
    # only the surviving slot reaches the validator
    # --------------------------------------------

    def test_airfoil_sub_bead_near_tangent_gap_suppressed_per_d010_single_slot_passes(self):
        # Cross-family node handling upstream of the boolean cut
        # (``StructureGenerator._merge_cross_family_nodes``) resolves this
        # exact pair (fwd station 0 and aft station 0, layer 5, Z = 1.20 mm,
        # 0.3783 mm of wall -- positive, not overlapping, below the 0.84 mm
        # minimum) into a single slot, so the validator does not see the pair
        # together on the real pipeline. The
        # validator's own fail-closed behaviour for a close pair is covered
        # for callers that bypass the generator's merge pass, and for
        # same-family close pairs, by
        # ``test_close_same_family_slots_still_fail_even_with_family_indices_given``
        # above and by generator-level coverage in
        # tests/unit/test_node_merge_scope.py.
        #
        # This test pins, at the validator level: handing the validator only
        # the single resulting slot satisfies every whole-layer invariant (one
        # island, no holes, margin, boundary, erosion) on its own. The
        # remaining single-slot layer must be a genuinely valid layer, on the
        # real geometry the original failure was measured from.
        fwd = Rect(-23.81085, -2.37907, -23.789849999999998, 3.0)
        outer = Rect(-25.0, -5.0, -23.0, 3.0)
        transformed = NotchedRect(outer=outer, notch=fwd)
        # Should not raise: leaves exactly one slot at this node, and that slot
        # alone is a valid layer.
        self.validator.validate(
            [outer],
            [transformed],
            [fwd],
            self.spec,
            5,
            1.20,
            slot_labels=["family 0 ('fwd')"],
            family_indices=[0],
        )

    # -- seven simultaneous cross-family crossings on
    # one layer (node row) -- whole-layer invariants hold
    # ------------------------------

    def test_node_row_seven_simultaneous_crossings_whole_layer_invariants_hold(self):
        # HONEST GAP addressed: an isolated pair does not represent the real
        # failure geometry. Both families share rib_pitch 8.0 mm, so they are
        # translates of each other along X -- EVERY fwd station crosses an aft
        # station on the same layer, 7 simultaneous crossings across the chord
        # at Z = 1.20 mm (see ``geometry_at_failure['intents']``: 7 fwd + 7
        # aft, evenly spaced 8.0 mm apart). This fixture reproduces that
        # multiplicity: 7 stations, each an overlapping differing-depth
        # cross-family pair (as in the merge fixture above), 8.0 mm apart on
        # the same layer, and asserts the WHOLE-layer invariants (one island,
        # no holes) still hold with 7 simultaneous junctions instead of 1.
        # (Depth is kept uniform ACROSS stations here, unlike the real data,
        # only because this stub's ``MultiNotchedRect`` erosion model requires
        # one shared bridge depth across all its notches at once -- see
        # ``StubOps``'s module docstring; the per-station depth variation
        # itself is already covered exactly by the single-pair fixtures above.)
        rib_pitch = 8.0
        centers = [-24.0 + i * rib_pitch for i in range(7)]
        fwd_notches = tuple(Rect(cx - 0.02, -2.4, cx + 0.02, 3.0) for cx in centers)
        aft_notches = tuple(Rect(cx - 0.01, -2.2, cx + 0.01, 3.0) for cx in centers)
        outer = Rect(-30.0, -5.0, 30.0, 3.0)
        transformed = MultiNotchedRect(outer=outer, notches=fwd_notches)
        slot_polygons = list(fwd_notches) + list(aft_notches)
        family_indices = [0] * 7 + [1] * 7
        slot_labels = ["family 0 ('fwd') station {}".format(i) for i in range(7)] + [
            "family 1 ('aft') station {}".format(i) for i in range(7)
        ]
        # Should not raise: 7 simultaneous overlapping cross-family
        # junctions, whole layer stays one connected, hole-free island.
        self.validator.validate(
            [outer],
            [transformed],
            slot_polygons,
            self.spec,
            5,
            1.20,
            slot_labels=slot_labels,
            family_indices=family_indices,
        )

    # -- same family, close: still the plain mistake, unchanged ----------

    def test_close_same_family_slots_still_fail_even_with_family_indices_given(self):
        # Passing family_indices does not, by itself, enable merging -- only a
        # genuinely DIFFERENT family index does. A same-family close pair is
        # exactly the placement mistake it always was.
        notch_a = Rect(4.7, 1.0, 5.3, 20.0)
        notch_b = Rect(5.32, 1.0, 6.0, 20.0)
        transformed = MultiNotchedRect(outer=self.original, notches=(notch_a, notch_b))
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [notch_a, notch_b],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
                slot_labels=["family 0 ('fwd')", "family 0 ('fwd')"],
                family_indices=[0, 0],
            )
        message = str(ctx.exception)
        self.assertIn("slots 1", message)
        self.assertIn("would merge into one slot or sever", message)

    # -- sever: hand-built, validator-generic (see class docstring) ---------

    def test_two_webs_whose_union_would_sever_the_section_fails_closed(self):
        # HONEST GAP: this architecture's generator only ever produces
        # STRAIGHT, single -surface-entry (top-edge) vertical slots. Two such
        # slots sharing the top edge can never sever the section into two
        # islands -- any point below both slots' far ends stays connected
        # across the full width. This fixture is therefore NOT reachable from
        # any two crossing families this generator can produce; it proves the
        # VALIDATOR's own (pre-existing, generic) invariant-1 check still names
        # the junction correctly if two cuts were ever handed to it that did
        # disconnect the material -- e.g. a future direct-path backend, or a
        # hand-built adversarial input.
        slot_a = Rect(9.7, 0.0, 10.0, 20.0)
        slot_b = Rect(10.0, 0.0, 10.3, 20.0)
        left = Rect(0.0, 0.0, 9.7, 20.0)
        right = Rect(10.3, 0.0, 20.0, 20.0)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [left, right],
                [slot_a, slot_b],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
                slot_labels=["family 0 ('fwd')", "family 1 ('aft')"],
                family_indices=[0, 1],
            )
        message = str(ctx.exception)
        self.assertIn("2 material islands", message)
        # Failures are attributed by ablation (removing each slot in turn), not
        # by a "nearest requested pair by X" heuristic. Here each slot is a FULL-HEIGHT cut that severs the
        # section entirely on its own, so removing either ONE individually does
        # not restore a single island -- the honest outcome is "not localised
        # to a single slot", which names no slot/family at all rather than
        # guess.
        self.assertIn("Cause not localised to any single slot", message)

    # -- hole: hand-built, validator-generic (see class docstring) ----------

    def test_two_webs_whose_union_would_enclose_a_hole_fails_closed(self):
        # HONEST GAP, same section as above, one level further: an enclosed
        # hole needs the removed region to form a CYCLE that never touches
        # the exterior boundary. Two boundary-connected straight cuts
        # cannot form a cycle at all (each is a simple arc from the
        # boundary back to itself); enclosing a hole needs at least THREE
        # cuts (or one bent one) -- topologically impossible from exactly
        # two crossing families in this architecture. As with the sever
        # fixture above, this proves the validator's own (pre-existing)
        # invariant-2 check still names the junction if handed geometry
        # that does enclose a hole.
        slot_a = Rect(9.0, 9.0, 10.0, 11.0)
        slot_b = Rect(10.0, 9.0, 11.0, 11.0)
        hole = Rect(9.0, 9.0, 11.0, 11.0)
        transformed = HoledRect(outer=self.original, hole=hole)
        with self.assertRaises(TopologyError) as ctx:
            self.validator.validate(
                [self.original],
                [transformed],
                [slot_a, slot_b],
                self.spec,
                LAYER_INDEX,
                PRINT_Z,
                slot_labels=["family 0 ('fwd')", "family 1 ('aft')"],
                family_indices=[0, 1],
            )
        message = str(ctx.exception)
        self.assertIn("interior hole", message)
        # each slot is itself a fully-interior, boundary-disconnected cut (its
        # own isolated hole), so removing either ONE alone still leaves one
        # hole -- ablation honestly reports "not localised", naming no
        # slot/family it did not actually prove is the cause.
        self.assertIn("Cause not localised to any single slot", message)


class DiamondGeneratorTests(unittest.TestCase):
    """Two opposite-drift families really do converge to a common chord
    station at a closed-form Z, on the coupon and the real NACA 2412 -- the
    generator side of "a diamond lattice is two families drifting in
    opposite directions"."""

    def _crossing_z(self, frame, pos_a, drift_a, pos_b, drift_b):
        xa0 = frame.web_x(pos_a)
        xb0 = frame.web_x(pos_b)
        return (xb0 - xa0) / (drift_a - drift_b)

    def test_coupon_two_families_cross_at_the_expected_z(self):
        # Crossing-order arithmetic is sampled at +/-10.0 mm, clear of the
        # cross-family merge band. A cross-family pair closer than
        # ``2 * safety_offset_mm`` (0.84 mm here) merges both families into
        # one WebIntent spanning the union of their footprints (both
        # families are present in ``merged_family_indices`` rather than as two
        # separate intents). At this fixture's shallow drift (0.1 mm/mm, slot
        # width 0.6 mm) the band is
        # Delta_z in [width / (2*drift), (width + min_wall) / (2*drift))
        # = [3.0, 7.2) mm either side of the crossing, so +/-5.0 mm samples
        # land INSIDE the band and are pinned separately below.
        ops = StubOps()
        rect = Rect(0.0, 0.0, 60.0, 20.0)
        frame = LayerFrame.from_polygon(ops, rect)
        fam_fwd = WebFamily(name="fwd", web_position=0.4, z_drift_mm_per_mm=0.1)
        fam_aft = WebFamily(name="aft", web_position=0.6, z_drift_mm_per_mm=-0.1)
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(fam_fwd, fam_aft)
        )
        z_cross = self._crossing_z(frame, 0.4, 0.1, 0.6, -0.1)
        self.assertAlmostEqual(z_cross, 60.0)

        gen = StructureGenerator()
        # -- order-check assertions at +/-10.0 mm: clear of the [3.0, 7.2) mm
        # band with margin (gap at 10.0 mm is 1.4 mm, versus the 0.84 mm
        # threshold), so both families are present as separate intents.
        before = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=z_cross - 10.0)
        )
        at_cross = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=z_cross)
        )
        after = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=z_cross + 10.0)
        )

        x_before = {i.family_name: i.start[0] for i in before.intents}
        x_cross = {i.family_name: i.start[0] for i in at_cross.intents}
        x_after = {i.family_name: i.start[0] for i in after.intents}

        # Before crossing: fwd is still left of aft.
        self.assertLess(x_before["fwd"], x_before["aft"])
        # At the crossing Z: coincident (within floating-point tolerance).
        self.assertAlmostEqual(x_cross["fwd"], x_cross["aft"], places=6)
        # After crossing: fwd has swept past aft -- a genuine crossing, not
        # merely an approach.
        self.assertGreater(x_after["fwd"], x_after["aft"])

        # -- band assertion, AT +/-5.0 mm: the pair merges into ONE WebIntent
        # spanning the union of both footprints -- no omission, both family
        # indices present via ``merged_family_indices``, geometry not a label
        # (see tests/unit/test_node_merge.py's
        # ``BothFamiliesContinuousThroughEveryNodeTests`` for why the label
        # alone is not sufficient evidence). Same fixture as the order-check
        # above: it pins both the crossing-order math (at +/-10.0 mm) and the
        # merge band (at +/-5.0 mm).
        near_before = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=z_cross - 5.0)
        )
        near_after = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=z_cross + 5.0)
        )
        for near in (near_before, near_after):
            # No omission at all -- a true merge drops nothing ("no family is
            # dropped").
            self.assertEqual(near.omitted, [])
            # Exactly one WebIntent at this node, spanning both families.
            self.assertEqual(len(near.intents), 1)
            merged = near.intents[0]
            self.assertEqual(merged.merged_family_indices, (0, 1))
            # Width is the union-interval width -- strictly wider than
            # either family's own requested 0.6 mm slot on its own.
            self.assertGreater(merged.width_mm, 0.6)

    def test_coupon_junction_at_crossing_z_satisfies_union_invariants(self):
        # The full round trip: the generator's OWN output at the crossing
        # Z, built into real slot polygons by SlotBuilder, validated by
        # TopologyValidator with the merge rule -- not a hand-picked
        # fixture. Both families are full-depth (web_depth_mm unset) at the
        # SAME web entry surface, so their (start, end) Y-ranges coincide
        # exactly at any shared station, which is exactly the equal-depth
        # case the junction rule merges.
        ops = StubOps()
        rect = Rect(0.0, 0.0, 60.0, 20.0)
        frame = LayerFrame.from_polygon(ops, rect)
        fam_fwd = WebFamily(name="fwd", web_position=0.4, z_drift_mm_per_mm=0.1)
        fam_aft = WebFamily(name="aft", web_position=0.6, z_drift_mm_per_mm=-0.1)
        spec = StructureSpec(
            slot_gap_mm=0.6, skin_margin_mm=1.0, families=(fam_fwd, fam_aft)
        )
        z_cross = self._crossing_z(frame, 0.4, 0.1, 0.6, -0.1)

        gen = StructureGenerator()
        result = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=600, print_z=z_cross)
        )
        self.assertEqual(len(result.intents), 2)
        self.assertEqual(result.omitted, [])

        builder = SlotBuilder()
        built = [builder.build(intent, spec.overshoot_mm) for intent in result.intents]
        slots = [Rect(*_rect_bounds_from_points(b.points)) for b in built]

        # Both slots are (numerically) identical -- true coincidence at the
        # crossing Z -- so their union is exactly the wider of the two
        # (here, identical) rectangles.
        xs = sorted((s.min_x, s.max_x) for s in slots)
        union_min_x = min(s.min_x for s in slots)
        union_max_x = max(s.max_x for s in slots)
        union_min_y = min(s.min_y for s in slots)
        union_max_y = max(s.max_y for s in slots)
        transformed = NotchedRect(
            outer=rect,
            notch=Rect(union_min_x, union_min_y, union_max_x, union_max_y),
        )

        validator = TopologyValidator(ops, safety_offset_mm=0.42)
        slot_labels = [
            "family {} ({!r})".format(i.family_index, i.family_name)
            for i in result.intents
        ]
        family_indices = [i.family_index for i in result.intents]
        # Should not raise: an intended equal-depth junction.
        validator.validate(
            [rect],
            [transformed],
            slots,
            spec,
            600,
            z_cross,
            slot_labels=slot_labels,
            family_indices=family_indices,
        )

    def test_naca2412_two_families_cross_at_the_expected_z(self):
        # HONEST GAP: this proves the GENERATOR side (two opposite-drift
        # families really do converge on the real cambered section) but
        # stops short of a full validate() round trip. StubOps/NotchedRect
        # can only represent a merged junction exactly when the union of
        # the two removed regions is itself a plane rectangle, which holds
        # for the rectangular coupon (see the test above) but not in
        # general for a curved NACA 2412 boundary. Exercising the merge
        # rule against the real curved boundary needs the real Clipper
        # backend (host), not this test double.
        #
        # Cross-family pairs closer than the minimum wall MERGE into one
        # WebIntent instead of staying as two separate intents; both families
        # are present inside ``merged_family_indices``. At this fixture's
        # drift (0.05 mm/mm) and resolved width (gap_ratio 0.5 * 0.42 mm
        # wall = 0.21 mm), the band is
        # Delta_z in [width / (2*drift), (width + min_wall) / (2*drift))
        # = [2.1, 10.5) mm either side of the crossing, so +/-5.0 mm lands
        # INSIDE the band. The order check uses +/-12.0 mm, which was
        # computed to clear the band on BOTH sides of this NACA 2412
        # crossing with real margin at the gap threshold (0.84 mm). Another
        # fixture's margin does not transfer: re-check the band math against
        # its own drift/width/z_cross.
        ops = StubOps()
        polygon, _bbox = _naca2412_polygon()
        frame = LayerFrame.from_polygon(ops, polygon)
        fam_fwd = WebFamily(name="fwd", web_position=0.45, z_drift_mm_per_mm=0.05)
        fam_aft = WebFamily(name="aft", web_position=0.55, z_drift_mm_per_mm=-0.05)
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, families=(fam_fwd, fam_aft)
        )
        z_cross = self._crossing_z(frame, 0.45, 0.05, 0.55, -0.05)
        self.assertGreater(z_cross, 0.0)

        gen = StructureGenerator()
        # -- order-check assertions at +/-12.0 mm: clear of the
        # [2.1, 10.5) mm band (crossing-order arithmetic on the real cambered
        # section).
        before = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=max(z_cross - 12.0, 0.0))
        )
        after = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=z_cross + 12.0)
        )
        x_before = {i.family_name: i.start[0] for i in before.intents}
        x_after = {i.family_name: i.start[0] for i in after.intents}
        self.assertLess(x_before["fwd"], x_before["aft"])
        self.assertGreaterEqual(x_after["fwd"], x_after["aft"])

        # -- band assertion, AT +/-5.0 mm: both families merge into ONE
        # WebIntent spanning the union of their footprints -- no omission,
        # both family indices present via ``merged_family_indices``, not
        # merely a label (see tests/unit/test_node_merge.py's
        # ``BothFamiliesContinuousThroughEveryNodeTests``). Pins the merge on
        # the real NACA 2412 section, not just the rectangular coupon above.
        near_before = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=max(z_cross - 5.0, 0.0))
        )
        near_after = gen.generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=z_cross + 5.0)
        )
        for near in (near_before, near_after):
            # No omission at all -- a true merge drops nothing ("no family is
            # dropped").
            self.assertEqual(near.omitted, [])
            # Exactly one WebIntent at this node, spanning both families.
            self.assertEqual(len(near.intents), 1)
            merged = near.intents[0]
            self.assertEqual(merged.merged_family_indices, (0, 1))
            # Width is the union-interval width -- strictly wider than
            # either family's own requested 0.21 mm slot (gap_ratio 0.5 *
            # 0.42 mm wall) on its own.
            self.assertGreater(merged.width_mm, 0.21)


def _rect_bounds_from_points(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


if __name__ == "__main__":
    unittest.main()
