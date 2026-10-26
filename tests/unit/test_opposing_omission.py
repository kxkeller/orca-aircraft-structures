"""Generator-side avoidance of the tapered two-sided phase collision.

See ``aircraft_core/generator.py``'s
``StructureGenerator._omit_opposing_conflicts`` docstring for the behaviour
this pins.

THE COLLISION, reproduced directly (no camber needed -- a plain rectangular
``LayerFrame`` at a chosen WIDTH stands in for "this wing's chord at some Z",
a tapered wing's LayerFrame stack would reproduce it too): a full-depth spar anchors by
chord FRACTION (``frame.web_x``); a marching stiffener family's other
stations sit at a FIXED MM OFFSET from its OWN anchor
(``_candidate_stations``). Two different anchor fractions plus a fixed
march pitch coincide EXACTLY at one width -- this file picks that width on
purpose, rather than sweeping a real taper, so the fixture is a minimized,
closed-form collision rather than a search.

Four groups:

- ``OmitOpposingConflictsDirectTests`` -- the private
  ``_omit_opposing_conflicts`` helper, exercised directly on hand-built
  ``WebIntent``s (mirrors ``test_node_merge.py``'s own precedent for pinning
  private placement arithmetic without driving the full
  ``StructureSpec``/family machinery).
- ``GeneratorOmitsOnTaperedPhaseCollisionTests`` -- the full
  ``StructureGenerator.generate()`` entry point, on the closed-form
  colliding width, PLUS the anti-drift proof: the geometry the generator
  declined to build is independently shown to be exactly what
  ``TopologyValidator`` would have rejected, by constructing that
  (pre-omission) geometry from the SAME two per-family calls and handing it
  to the real validator.
- ``GeneratorDoesNotOmitNonCollidingStationsTests`` -- a comfortably
  -separated two-sided config produces zero "opposing entry:" omissions
  (the "not a blunt instrument" guard).
- ``ShippedDefaultUnaffectedTests`` -- the shipped default (no ``families``,
  single implicit +Y-only family) never reaches the opposing check at all.
"""
import unittest

from aircraft_core.frame import LayerFrame, opposing_pair_meets
from aircraft_core.generator import LayerContext, StructureGenerator, WebIntent
from aircraft_core.slot import SlotBuilder
from aircraft_core.spec import StructureSpec, WebFamily
from aircraft_core.validate import TopologyError, TopologyValidator
from stub_ops import NotchedRect, Rect, StubOps, TwoSidedNotchedRect

# Matches this project's shipped defaults and test_opposing_entry.py's own
# convention.
SAFETY_OFFSET_MM = 0.42
# 0.0 keeps these direct calls testing pure omission mechanics (tie-breaks,
# restart, which side loses) against exactly the hand -picked depths below,
# unaffected by the reach_depth_mm(overshoot_mm) correction --
# reach_depth_mm(0.0) is byte-identical to depth_mm().
OVERSHOOT_MM = 0.0
MIN_WALL_MM = 2.0 * SAFETY_OFFSET_MM  # 0.84
SKIN_MARGIN_MM = 2.0 * SAFETY_OFFSET_MM + 0.021  # 0.861


def _rect_from_points(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return Rect(min(xs), min(ys), max(xs), max(ys))


class OmitOpposingConflictsDirectTests(unittest.TestCase):
    """``StructureGenerator._omit_opposing_conflicts``, called directly on
    hand-built ``WebIntent``s -- pins the arithmetic without driving the
    full anchoring/marching machinery."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 8.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)
        self.gen = StructureGenerator()

    def _full_depth_upper(self, x):
        # entry at the +Y skin (y=8), far end skin_margin_mm short of -Y.
        return WebIntent(
            start=(x, 8.0),
            end=(x, SKIN_MARGIN_MM),
            width_mm=0.6,
            family_index=0,
            family_name="spar",
            entry_lower=False,
        )

    def _shallow_lower(self, x, depth_mm=1.5, family_index=1, name="stiffener"):
        return WebIntent(
            start=(x, 0.0),
            end=(x, depth_mm),
            width_mm=0.6,
            family_index=family_index,
            family_name=name,
            entry_lower=True,
        )

    def test_colliding_pair_omits_the_shallower_and_keeps_the_deeper(self):
        spar = self._full_depth_upper(10.0)
        stiffener = self._shallow_lower(10.0)  # identical X: certain overlap
        kept, omitted = self.gen._omit_opposing_conflicts(
            [spar, stiffener], self.frame, SAFETY_OFFSET_MM, OVERSHOOT_MM
        )
        self.assertEqual(kept, [spar])
        self.assertEqual(len(omitted), 1)
        self.assertTrue(omitted[0].reason.startswith("opposing entry:"))
        self.assertEqual(omitted[0].family_name, "stiffener")
        self.assertAlmostEqual(omitted[0].x_mm, 10.0)

    def test_non_colliding_pair_is_untouched(self):
        spar = self._full_depth_upper(2.0)
        stiffener = self._shallow_lower(17.0)  # far apart, well clear
        kept, omitted = self.gen._omit_opposing_conflicts(
            [spar, stiffener], self.frame, SAFETY_OFFSET_MM, OVERSHOOT_MM
        )
        self.assertEqual(kept, [spar, stiffener])
        self.assertEqual(omitted, [])

    def test_same_side_pair_is_never_touched(self):
        # Two +Y candidates: this method only ever looks at cross-edge pairs; a
        # same-side pair is own collision check's job, not this one's,
        # regardless of how close they are.
        a = self._full_depth_upper(10.0)
        b = self._full_depth_upper(10.0)
        kept, omitted = self.gen._omit_opposing_conflicts(
            [a, b], self.frame, SAFETY_OFFSET_MM, OVERSHOOT_MM
        )
        self.assertEqual(kept, [a, b])
        self.assertEqual(omitted, [])

    def test_shallower_vs_shallower_tie_break_by_family_index(self):
        # Equal depth on both sides: the tie-break (larger family_index
        # loses) must be deterministic, not order-dependent.
        upper = self._shallow_lower(10.0, depth_mm=3.9, family_index=0, name="a")
        # Flip entry to +Y for this one, same depth as "lower" test-helper
        # produces below for the -Y side.
        upper = WebIntent(
            start=(10.0, 8.0),
            end=(10.0, 8.0 - 3.9),
            width_mm=0.6,
            family_index=0,
            family_name="a",
            entry_lower=False,
        )
        lower = self._shallow_lower(10.0, depth_mm=3.9, family_index=5, name="b")
        kept, omitted = self.gen._omit_opposing_conflicts(
            [upper, lower], self.frame, SAFETY_OFFSET_MM, OVERSHOOT_MM
        )
        # Equal depth (3.9 == 3.9): larger family_index (5, "b") loses.
        self.assertEqual(kept, [upper])
        self.assertEqual(len(omitted), 1)
        self.assertEqual(omitted[0].family_name, "b")

    def test_result_agrees_with_the_shared_predicate_directly(self):
        # Anti-drift sanity: the helper's own drop/keep decision must be
        # exactly what a direct call to the shared predicate says, for the
        # SAME inputs, no re-derivation.
        spar = self._full_depth_upper(10.0)
        stiffener = self._shallow_lower(10.0)
        meeting = opposing_pair_meets(
            self.frame.ops,
            self.frame.polygon,
            10.0 - 0.3,
            10.0 + 0.3,
            8.0 - SKIN_MARGIN_MM,
            10.0 - 0.3,
            10.0 + 0.3,
            1.5,
            SAFETY_OFFSET_MM,
            MIN_WALL_MM,
        )
        self.assertIsNotNone(meeting)
        kept, omitted = self.gen._omit_opposing_conflicts(
            [spar, stiffener], self.frame, SAFETY_OFFSET_MM, OVERSHOOT_MM
        )
        self.assertEqual(len(omitted), 1)

    def test_boundary_halo_measurement_failure_omits_instead_of_crashing(self):
        # Regression: found by a tapered-fixture verification sweep. A candidate near the very edge of
        # the chord, widened by the halo (``safety_offset_mm`` on every
        # side), can push the OVERLAP SPAN's own probe stations past
        # ``frame.min_x`` -- ``opposing_pair_meets`` propagates
        # ``ValueError`` exactly as ``local_upper_worst_case``/
        # ``local_lower_worst_case`` do (by design -- see that function's
        # own docstring), and ``TopologyValidator`` treats that as an
        # unconditional fail-closed ``TopologyError``. Left unhandled here,
        # the SAME exception would abort ``generate()`` entirely -- one
        # near-tip station taking down the whole layer, exactly what this
        # task exists to prevent. It must instead be treated
        # conservatively as a conflict and omitted, never silently treated
        # as safe.
        rect = Rect(0.0, 0.0, 20.0, 8.0)
        frame = LayerFrame.from_polygon(StubOps(), rect)
        upper = WebIntent(
            start=(0.05, 8.0),
            end=(0.05, SKIN_MARGIN_MM),
            width_mm=0.6,
            family_index=0,
            family_name="spar",
            entry_lower=False,
        )
        lower = WebIntent(
            start=(0.05, 0.0),
            end=(0.05, 1.5),
            width_mm=0.6,
            family_index=1,
            family_name="stiffener",
            entry_lower=True,
        )
        # Confirms the fixture actually reproduces the measurement failure
        # (not merely an ordinary meeting-condition conflict) before
        # asserting the helper's own handling of it.
        with self.assertRaises(ValueError):
            opposing_pair_meets(
                frame.ops,
                frame.polygon,
                -0.25,
                0.35,
                8.0 - SKIN_MARGIN_MM,
                -0.25,
                0.35,
                1.5,
                SAFETY_OFFSET_MM,
                MIN_WALL_MM,
            )

        kept, omitted = self.gen._omit_opposing_conflicts(
            [upper, lower], frame, SAFETY_OFFSET_MM, OVERSHOOT_MM
        )
        self.assertEqual(kept, [upper])
        self.assertEqual(len(omitted), 1)
        self.assertTrue(omitted[0].reason.startswith("opposing entry:"))
        self.assertIn("could not be measured", omitted[0].reason)
        self.assertEqual(omitted[0].family_name, "stiffener")


class ReachDepthOvershootRegressionTests(unittest.TestCase):
    """-- pins the fix directly: ``WebIntent. reach_depth_mm(overshoot_mm)`` (not
    ``depth_mm()``) is what ``_omit_opposing_conflicts`` must compare against,
    because ``SlotBuilder`` actually cuts ``overshoot_mm`` past the entry
    boundary. Station numbers below are the exact real-world station traced by
    hand (taper fixture, layer 55, Z=11.20 mm): local thickness 3.8753 mm,
    ``web_depth_mm=1.5``, ``overshoot_mm=0.05``. A threshold based on ``depth_mm()`` alone
    (``1.5 + 1.5 + 0.84 = 3.84``) calls this pair SAFE; the real
    cut's threshold (``1.55 + 1.55 + 0.84 = 3.94``) is what
    ``TopologyValidator`` actually enforces -- 3.8753 mm sits inside that gap.
    """

    OVERSHOOT_MM_REAL = 0.05
    T_LOCAL_MM = 3.8753

    def setUp(self):
        self.ops = StubOps()
        # A flat rectangle whose height reproduces the traced station's own
        # local thickness exactly, centred on Y=0 the way LayerFrame's own
        # worst-case probes are (see StubOps/Rect's own convention).
        half_t = self.T_LOCAL_MM / 2.0
        self.rect = Rect(0.0, -half_t, 20.0, half_t)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)
        self.gen = StructureGenerator()

    def _pair(self):
        upper = WebIntent(
            start=(10.0, self.T_LOCAL_MM / 2.0),
            end=(10.0, self.T_LOCAL_MM / 2.0 - 1.5),
            width_mm=0.621,
            family_index=0,
            family_name="lattice_plus_upper",
            entry_lower=False,
        )
        lower = WebIntent(
            start=(10.0, -self.T_LOCAL_MM / 2.0),
            end=(10.0, -self.T_LOCAL_MM / 2.0 + 1.5),
            width_mm=0.621,
            family_index=2,
            family_name="lattice_plus_lower",
            entry_lower=True,
        )
        return upper, lower

    def test_reach_depth_mm_is_depth_mm_plus_overshoot_for_an_untilted_intent(self):
        upper, _ = self._pair()
        self.assertEqual(upper.depth_mm(), 1.5)
        self.assertAlmostEqual(
            upper.reach_depth_mm(self.OVERSHOOT_MM_REAL), 1.5 + self.OVERSHOOT_MM_REAL
        )

    def test_reach_depth_mm_at_zero_overshoot_is_byte_identical_to_depth_mm(self):
        upper, _ = self._pair()
        self.assertEqual(upper.reach_depth_mm(0.0), upper.depth_mm())

    def test_pre_d021_threshold_would_have_called_this_pair_safe(self):
        # Reproduces the depth_mm()-only arithmetic directly against
        # opposing_pair_meets, to pin exactly what the omission guards against.
        upper, lower = self._pair()
        old_meeting = opposing_pair_meets(
            self.ops,
            self.rect,
            *upper.footprint_x(),
            upper.depth_mm(),
            *lower.footprint_x(),
            lower.depth_mm(),
            SAFETY_OFFSET_MM,
            MIN_WALL_MM,
        )
        self.assertIsNone(
            old_meeting,
            "sanity check on the fixture: the earlier (depth_mm()-only) "
            "threshold must call this pair safe, or this test is not "
            "reproducing the real bug.",
        )

    def test_generator_now_omits_a_pair_the_old_depth_would_have_missed(self):
        upper, lower = self._pair()
        kept, omitted = self.gen._omit_opposing_conflicts(
            [upper, lower], self.frame, SAFETY_OFFSET_MM, self.OVERSHOOT_MM_REAL
        )
        self.assertEqual(len(kept), 1, "one side of the pair must be omitted")
        self.assertEqual(len(omitted), 1)
        self.assertTrue(omitted[0].reason.startswith("opposing entry:"))

    def test_zero_overshoot_reproduces_the_pre_d021_no_op(self):
        # Confirms the omission is genuinely gated on overshoot_mm, not a
        # threshold change in disguise: at overshoot_mm=0.0 the pair is
        # still judged safe and nothing is omitted.
        upper, lower = self._pair()
        kept, omitted = self.gen._omit_opposing_conflicts(
            [upper, lower], self.frame, SAFETY_OFFSET_MM, 0.0
        )
        self.assertEqual(kept, [upper, lower])
        self.assertEqual(omitted, [])


class SharedPredicateBoundaryTests(unittest.TestCase):
    """The exact-equality boundary of ``opposing_pair_meets`` itself (proof point
    4, docs/geometry_invariants.md): equality FAILS CLOSED, the opposite
    epsilon bias from the same-edge collision check."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 8.0)  # t_local = 8.0 everywhere

    def test_exact_equality_meets_fails_closed(self):
        d_lower = 1.5
        d_upper = 8.0 - MIN_WALL_MM - d_lower  # sum reconstructs to 8.0
        meeting = opposing_pair_meets(
            self.ops,
            self.rect,
            9.7,
            10.3,
            d_upper,
            9.7,
            10.3,
            d_lower,
            SAFETY_OFFSET_MM,
            MIN_WALL_MM,
        )
        self.assertIsNotNone(
            meeting,
            "equality (d_upper + d_lower + min_wall == t_local) must FAIL "
            "CLOSED -- the wall is already zero width before the safety "
            "offset even runs.",
        )

    def test_comfortably_clear_pair_is_safe(self):
        meeting = opposing_pair_meets(
            self.ops,
            self.rect,
            9.7,
            10.3,
            1.0,
            9.7,
            10.3,
            1.0,
            SAFETY_OFFSET_MM,
            MIN_WALL_MM,
        )
        self.assertIsNone(meeting)

    def test_haloed_footprints_not_overlapping_is_safe_regardless_of_depth(self):
        # Full-depth-vs-full-depth would ordinarily meet (proof point 5),
        # but these footprints are far enough apart that even their haloed
        # ranges never touch -- no shared span, no meeting condition.
        meeting = opposing_pair_meets(
            self.ops,
            self.rect,
            0.0,
            1.0,
            7.0,
            15.0,
            16.0,
            7.0,
            SAFETY_OFFSET_MM,
            MIN_WALL_MM,
        )
        self.assertIsNone(meeting)


def _spar_and_stiffener_spec(spar_position, stiffener_position, rib_pitch_mm=3.9):
    spar = WebFamily(
        name="spar_front",
        web_position=spar_position,
        slot_gap_mm=0.6,
    )
    stiffener = WebFamily(
        name="stiffener_lower",
        web_position=stiffener_position,
        rib_pitch_mm=rib_pitch_mm,
        web_depth_mm=1.5,
        entry_lower=True,
        slot_gap_mm=0.6,
    )
    return StructureSpec(
        slot_gap_mm=0.6,
        skin_margin_mm=SKIN_MARGIN_MM,
        families=(spar, stiffener),
    )


class GeneratorOmitsOnTaperedPhaseCollisionTests(unittest.TestCase):
    """Full ``StructureGenerator.generate()``, at the closed-form width
    where ``spar_front`` (anchored at chord fraction 0.25) and
    ``stiffener_lower`` (anchored at 0.20, marching at 3.9 mm pitch) land on
    the EXACT same X -- the tapered-wing phase collision, reproduced without
    a real taper sweep (see this module's docstring)."""

    def setUp(self):
        self.ops = StubOps()
        # 0.20*W + 1*3.9 == 0.25*W  =>  W = 3.9 / 0.05 = 78.0
        self.width_mm = 78.0
        self.rect = Rect(0.0, 0.0, self.width_mm, 8.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)
        self.spec = _spar_and_stiffener_spec(0.25, 0.20)
        self.gen = StructureGenerator()
        self.layer = LayerContext(layer_index=10, print_z=2.0)

    def test_collision_x_is_exactly_shared_by_both_families(self):
        spar_x = self.frame.web_x(0.25)
        stiffener_x = self.frame.web_x(0.20) + 1 * 3.9
        self.assertAlmostEqual(spar_x, 19.5)
        self.assertAlmostEqual(stiffener_x, 19.5)
        self.assertAlmostEqual(spar_x, stiffener_x)

    def test_colliding_station_is_omitted_and_slice_succeeds(self):
        result = self.gen.generate(
            self.frame,
            self.spec,
            SAFETY_OFFSET_MM,
            self.layer,
            safety_offset_mm=SAFETY_OFFSET_MM,
        )

        # The spar survives, exactly once, at the collision X.
        spar_intents = [i for i in result.intents if i.family_name == "spar_front"]
        self.assertEqual(len(spar_intents), 1)
        self.assertAlmostEqual(spar_intents[0].start[0], 19.5)

        # The colliding stiffener station (x=19.5) is omitted, not shifted
        # -- named against the spar it collided with.
        opposing_omissions = [
            o for o in result.omitted if o.reason.startswith("opposing entry:")
        ]
        self.assertEqual(len(opposing_omissions), 1)
        self.assertEqual(opposing_omissions[0].family_name, "stiffener_lower")
        self.assertAlmostEqual(opposing_omissions[0].x_mm, 19.5)
        self.assertIn("spar_front", opposing_omissions[0].reason)

        # NOT a blunt instrument: other stiffener_lower stations, away from
        # the collision, still survive as ordinary intents.
        surviving_stiffener_xs = [
            i.start[0] for i in result.intents if i.family_name == "stiffener_lower"
        ]
        self.assertTrue(
            any(abs(x - 19.5) > 1.0 for x in surviving_stiffener_xs),
            "expected at least one non-colliding stiffener_lower station to "
            "survive; got x values {!r}".format(surviving_stiffener_xs),
        )
        # No OTHER stiffener_lower omission reason exists at the collision
        # X -- the opposing-entry reason is the only cause recorded there.
        other_reasons_at_collision = [
            o.reason
            for o in result.omitted
            if o.family_name == "stiffener_lower" and abs(o.x_mm - 19.5) < 1e-6
        ]
        self.assertEqual(
            other_reasons_at_collision, [opposing_omissions[0].reason]
        )

        # The slice actually SUCCEEDS at the collision station: with
        # the opposing stiffener gone, only the spar's own +Y notch remains
        # there, and that validates cleanly (mirrors
        # test_thick_skin_path.py's own single-web generate-build-diff
        # -validate chain; whole-layer multi-station geometry is already
        # covered by the pre-existing multi-web generator tests and is not
        # this fixture's concern -- StubOps only models one notch per edge
        # at a time).
        builder = SlotBuilder()
        spar_slot = _rect_from_points(
            builder.build(spar_intents[0], self.spec.overshoot_mm).points
        )
        transformed = self.ops.diff(self.rect, [spar_slot])
        self.assertEqual(len(transformed), 1)

        validator = TopologyValidator(self.ops, safety_offset_mm=SAFETY_OFFSET_MM)
        try:
            validator.validate(
                [self.rect],
                transformed,
                [spar_slot],
                self.spec,
                self.layer.layer_index,
                self.layer.print_z,
            )
        except TopologyError as exc:
            self.fail(
                "post-fix geometry at the collision station (opposing "
                "stiffener omitted) should validate cleanly; got: "
                "{}".format(exc)
            )

    def test_anti_drift_the_omitted_geometry_is_exactly_what_the_validator_would_reject(
        self,
    ):
        # Reconstruct the PRE-OMISSION geometry directly, bypassing
        # ``_omit_opposing_conflicts`` -- the same "call the private
        # per-family helper directly" pattern test_node_merge.py already
        # uses to pin behaviour without driving the whole pipeline.
        effective_skin_margin_mm = self.spec.skin_margin_mm
        spar_family, stiffener_family = self.spec.effective_families()
        spar_intents, spar_omitted = self.gen._generate_family(
            self.frame,
            self.spec,
            spar_family,
            0,
            SAFETY_OFFSET_MM,
            self.layer,
            SAFETY_OFFSET_MM,
            effective_skin_margin_mm,
        )
        stiffener_intents, stiffener_omitted = self.gen._generate_family(
            self.frame,
            self.spec,
            stiffener_family,
            1,
            SAFETY_OFFSET_MM,
            self.layer,
            SAFETY_OFFSET_MM,
            effective_skin_margin_mm,
        )
        self.assertEqual(spar_omitted, [])
        # Some near-boundary stiffener stations may legitimately fall just
        # outside the chord extent (an ordinary, unrelated omission reason)
        # -- only the ABSENCE of any "opposing entry" reason matters here,
        # since this call bypasses ``_omit_opposing_conflicts`` entirely.
        self.assertTrue(
            all(not o.reason.startswith("opposing entry:") for o in stiffener_omitted)
        )
        self.assertEqual(len(spar_intents), 1)

        colliding_stiffener = [
            i for i in stiffener_intents if abs(i.start[0] - 19.5) < 1e-6
        ]
        self.assertEqual(len(colliding_stiffener), 1)
        colliding_stiffener = colliding_stiffener[0]
        spar_intent = spar_intents[0]

        builder = SlotBuilder()
        spar_slot = _rect_from_points(
            builder.build(spar_intent, self.spec.overshoot_mm).points
        )
        stiffener_slot = _rect_from_points(
            builder.build(colliding_stiffener, self.spec.overshoot_mm).points
        )
        # Both centered on the same X with the same width -- the exact
        # collision this fixture was built to produce.
        self.assertAlmostEqual(spar_slot.min_x, stiffener_slot.min_x)
        self.assertAlmostEqual(spar_slot.max_x, stiffener_slot.max_x)

        transformed = TwoSidedNotchedRect(
            outer=self.rect, notch_top=spar_slot, notch_bottom=stiffener_slot
        )
        validator = TopologyValidator(self.ops, safety_offset_mm=SAFETY_OFFSET_MM)
        with self.assertRaises(TopologyError) as ctx:
            validator.validate(
                [self.rect],
                [transformed],
                [spar_slot, stiffener_slot],
                self.spec,
                self.layer.layer_index,
                self.layer.print_z,
                slot_labels=["family 0 ('spar_front')", "family 1 ('stiffener_lower')"],
            )
        message = str(ctx.exception).lower()
        self.assertTrue(
            "opposing" in message and "meet" in message,
            "expected the validator's own opposing-entry meeting-condition "
            "failure, got: {}".format(ctx.exception),
        )


class GeneratorDoesNotOmitNonCollidingStationsTests(unittest.TestCase):
    """A comfortably-separated two-sided config must not have ANY station
    swept up by the opposing-entry check -- the "not a blunt instrument"
    guard."""

    def test_far_apart_anchors_produce_no_opposing_omissions(self):
        ops = StubOps()
        # spar at 0.25, stiffener anchored at 0.75 with a WIDE pitch -- no
        # marched station ever lands near 0.25 * width for any width this
        # test uses.
        rect = Rect(0.0, 0.0, 60.0, 8.0)
        frame = LayerFrame.from_polygon(ops, rect)
        spec = _spar_and_stiffener_spec(0.25, 0.75, rib_pitch_mm=20.0)
        gen = StructureGenerator()
        layer = LayerContext(layer_index=0, print_z=0.2)
        result = gen.generate(
            frame, spec, SAFETY_OFFSET_MM, layer, safety_offset_mm=SAFETY_OFFSET_MM
        )
        opposing_omissions = [
            o for o in result.omitted if o.reason.startswith("opposing entry:")
        ]
        self.assertEqual(opposing_omissions, [])
        family_names = {i.family_name for i in result.intents}
        self.assertEqual(family_names, {"spar_front", "stiffener_lower"})


class ShippedDefaultUnaffectedTests(unittest.TestCase):
    """The shipped default has no lower-surface family at all (``families``
    empty, the single implicit family is always ``entry_lower=False``) --
    the opposing-entry check must never even run."""

    def test_default_spec_produces_no_opposing_omissions(self):
        ops = StubOps()
        rect = Rect(-10.0, -10.0, 10.0, 10.0)
        frame = LayerFrame.from_polygon(ops, rect)
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0)
        gen = StructureGenerator()
        layer = LayerContext(layer_index=0, print_z=0.2)
        result = gen.generate(
            frame, spec, 0.42, layer, safety_offset_mm=0.42
        )
        self.assertEqual(result.omitted, [])
        self.assertEqual(len(result.intents), 1)

    def test_omit_opposing_conflicts_is_a_no_op_with_no_lower_side(self):
        ops = StubOps()
        rect = Rect(0.0, 0.0, 20.0, 8.0)
        frame = LayerFrame.from_polygon(ops, rect)
        gen = StructureGenerator()
        intents = [
            WebIntent(
                start=(5.0, 8.0),
                end=(5.0, SKIN_MARGIN_MM),
                width_mm=0.6,
                family_index=0,
                family_name="default",
                entry_lower=False,
            )
        ]
        kept, omitted = gen._omit_opposing_conflicts(intents, frame, SAFETY_OFFSET_MM, OVERSHOOT_MM)
        self.assertEqual(kept, intents)
        self.assertEqual(omitted, [])


if __name__ == "__main__":
    unittest.main()
