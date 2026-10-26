"""``feature_dense_lattice_enabled`` (``aircraft_core/spec.py``
``StructureSpec``, default ``False``).

The flag lets a feature's lattice take as much depth as the part can
genuinely take. The shipped bound cannot reach that on ANY feature shape (it
is bounded by ``frame.width`` and by ``frame.depth / 2``, which forbids a
tooth from ever passing the section's own midline even though opposing teeth
are laterally offset and cannot collide there).

WHAT THIS FLAG ACTUALLY DOES (measured, not assumed -- see
``StructureGenerator._generate_feature``'s own "MEASURED COUNTEREXAMPLE"
comment in the dense branch): removes the ``frame.width``/``frame.depth/2``
caps and hands each candidate station to the SAME full-depth
(``web_depth_mm=None``) per-station measurement the aerofoil path already
uses, keeping the SAME far-side margin (``effective_skin_margin_mm``, i.e.
``skin_margin_mm`` corrected for rotation) the non-dense branch already
uses -- NOT a smaller feature-specific margin, which is unimplementable:
``TopologyValidator.validate``
independently re-derives and enforces ``skin_margin_mm``-based remaining
material on every slot regardless of what margin the generator used to
choose the depth (aircraft_core/validate.py invariant 4, ~line 1272 and its
-Y mirror), so a smaller margin only builds teeth the validator then
rejects. Reproduced directly below (``SmallerMarginIsRejectedByValidator
Tests``).

This genuinely helps a feature whose far surface is roughly FLAT across widened
thin-section probe window (a square/rectangular post:
``SquarePostGetsDeeperTeethTests`` below). It does NOT help a real
z=230mm joiner post: that section is chamfered/rounded (fill ratio ~0.78), and
extremity-widening correctly predicts (confirmed independently by
``tools/m36_d017_bypass_probe.py``, a hand-built bypass probe straight to the
real validator; see the generator's own comment for the numbers) that full local depth there leaves negative remaining
skin near the chamfer. This is recorded here, not routed around: a
chamfered/round feature is exactly the shape this flag cannot help without
either a smaller margin (shown unavailable above) or a genuinely per-station
depth bound (a construction change -- one ``WebFamily`` per candidate station).

NEVER-WORSE WRAPPER (following the measured 3->0
regression above): ``StructureGenerator._generate_feature`` computes BOTH
the dense (``_generate_feature_core(dense=True)``) and shipped
(``dense=False``) results and keeps whichever has more teeth, ties broken
by greater max depth -- see ``_feature_result_is_better``'s own docstring.
So the PUBLIC ``_generate_feature`` entry point never regresses a feature
below its shipped result; the raw, un-wrapped dense attempt (which DOES
still lose teeth on a chamfer, by itself) is only reachable by calling
``_generate_feature_core(..., dense=True)`` directly, which
``ChamferedPostRawDenseAttemptStillZeroTeethTests`` below does explicitly
so that measured fact stays pinned even though the wrapper hides it
from every ordinary caller. Swept across pitches 0.6mm-3.5mm on the real
z=230mm post (``tools/m36_dense_lattice_sweep.py``, offline, not a unit test),
the dense attempt never wins that comparison at ANY pitch tried, so with the
never-worse rule in place, ``feature_dense_lattice_enabled`` contributes
NOTHING extra on that post at any pitch -- the only lever that
changes its stiffness is ``feature_lattice_pitch_mm`` itself.
"""
import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import (  # noqa: E402
    LayerContext,
    StructureGenerator,
)
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.spec import StructureSpec  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

from shapely_ops import HAVE_SHAPELY as _HAVE_SHAPELY  # noqa: E402
from shapely_ops import MiterShapelyOps as _MiterShapelyOps  # noqa: E402

import make_fixtures as _mf  # noqa: E402

TESTS_UNIT_DIR = REPO_ROOT / "tests" / "unit"
if str(TESTS_UNIT_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_UNIT_DIR))

from stub_ops import PolygonShape, Rect, StubOps  # noqa: E402

WALL_WIDTH_MM = 0.42


def _rect_points(width_mm, height_mm, cx=0.0, cy=0.0):
    hw, hh = width_mm / 2.0, height_mm / 2.0
    return (
        (cx - hw, cy - hh),
        (cx + hw, cy - hh),
        (cx + hw, cy + hh),
        (cx - hw, cy + hh),
    )


class CandidateStationInterleaveTests(unittest.TestCase):
    """Opposing teeth cannot collide.
    Pinned directly against ``StructureGenerator._candidate_stations`` --
    the SAME method both the dense and non-dense branches call -- rather
    than inferred from a built result, since it is a property of the
    station arithmetic itself, true whether or not any station happens to
    survive the margin/thin-section gates.

    ``_generate_feature`` anchors ``top_family`` at ``frame``'s own centre
    (``top_pos = 0.5``) marching at ``2 * pitch_mm``, and ``bottom_family``
    exactly ``pitch_mm`` further along X, marching at the same
    ``2 * pitch_mm`` -- so top stations are ``center + 2*pitch*k`` and
    bottom stations are ``center + pitch + 2*pitch*k`` for integer k. Any
    equality ``center + 2*pitch*k1 == center + pitch + 2*pitch*k2`` implies
    ``pitch = 2*pitch*(k1 - k2)``, i.e. ``2*(k1-k2) = 1`` for integer
    ``k1, k2`` -- impossible, so the two sets are disjoint for ANY pitch,
    not merely the ones this file happens to sample.
    """

    def test_top_and_bottom_stations_never_share_an_x(self):
        gen = StructureGenerator()
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, PolygonShape(_rect_points(40.0, 40.0)))
        for pitch in (0.5, 0.9, 1.0, 1.5, 2.0, 3.0, 3.9):
            center_x = (frame.min_x + frame.max_x) / 2.0
            top_stations = gen._candidate_stations(frame, 2.0 * pitch, center_x)
            bottom_anchor = center_x + pitch
            bottom_stations = gen._candidate_stations(frame, 2.0 * pitch, bottom_anchor)
            shared = set(round(x, 9) for x in top_stations) & set(
                round(x, 9) for x in bottom_stations
            )
            self.assertEqual(
                shared, set(),
                "pitch={!r}: top/bottom stations share an X -- opposing "
                "teeth could land on the same station".format(pitch),
            )
            # And every top/bottom pair is at least `pitch` apart -- not
            # merely "not exactly equal" (float noise could otherwise hide
            # a near-collision).
            for tx in top_stations:
                for bx in bottom_stations:
                    self.assertGreaterEqual(
                        abs(tx - bx), pitch - 1e-9,
                        "pitch={!r}: top x={!r} and bottom x={!r} are "
                        "closer than one pitch".format(pitch, tx, bx),
                    )


class FlagFalseIsByteIdenticalTests(unittest.TestCase):
    """Criterion (1): the default (``feature_dense_lattice_enabled=False``) must
    reproduce the shipped behaviour exactly -- same intents (start/
    end/width/family), same omissions, on an existing feature fixture, whether
    or not the flag is even passed (its default IS False)."""

    def _run(self, points, dense):
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, PolygonShape(points))
        gen = StructureGenerator()
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, root_solid_layers=0,
            web_position=0.5, z_drift_mm_per_mm=100.0,
            feature_dense_lattice_enabled=dense,
        )
        ctx = LayerContext(layer_index=0, print_z=1.0)
        return gen.generate(frame, spec, WALL_WIDTH_MM, ctx, safety_offset_mm=WALL_WIDTH_MM)

    def _intent_tuple(self, i):
        return (i.start, i.end, i.width_mm, i.family_name, i.entry_lower)

    def _omitted_tuple(self, o):
        return (round(o.x_mm, 6) if o.x_mm == o.x_mm else "nan", o.family_name, o.reason)

    def test_square_post_unset_flag_matches_explicit_false(self):
        default_spec_result = self._run(_rect_points(8.64, 8.67), dense=False)
        explicit_default = StructureGenerator()
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, PolygonShape(_rect_points(8.64, 8.67)))
        spec_no_flag = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, root_solid_layers=0,
            web_position=0.5, z_drift_mm_per_mm=100.0,
        )  # feature_dense_lattice_enabled not passed -- must equal False
        self.assertFalse(spec_no_flag.feature_dense_lattice_enabled)
        ctx = LayerContext(layer_index=0, print_z=1.0)
        no_flag_result = explicit_default.generate(
            frame, spec_no_flag, WALL_WIDTH_MM, ctx, safety_offset_mm=WALL_WIDTH_MM
        )
        self.assertEqual(
            [self._intent_tuple(i) for i in default_spec_result.intents],
            [self._intent_tuple(i) for i in no_flag_result.intents],
        )
        self.assertEqual(
            [self._omitted_tuple(o) for o in default_spec_result.omitted],
            [self._omitted_tuple(o) for o in no_flag_result.omitted],
        )

    def test_square_post_flag_false_matches_shipped_five_teeth(self):
        # Pinned against the shipped envelope: shipped default (pitch 2.0)
        # builds 3 teeth on the real chamfered post but 5 on a true 8.64x8.67
        # square (flat far surface, no chamfer). This test pins the square's
        # own shipped count/depth so a future change to the FALSE branch cannot
        # silently drift.
        result = self._run(_rect_points(8.64, 8.67), dense=False)
        self.assertEqual(len(result.intents), 5)
        for i in result.intents:
            self.assertAlmostEqual(i.depth_mm(), 1.7053, places=3)


@unittest.skipUnless(_HAVE_SHAPELY, "shapely not installed")
class SquarePostGetsDeeperTeethTests(unittest.TestCase):
    """Pinned on a square post rather than the real
    (chamfered/round) z=230mm section -- see this file's module docstring
    and ``StructureGenerator._generate_feature``'s own "MEASURED
    COUNTEREXAMPLE" comment for why the flag REGRESSES (3 teeth -> 0) on
    the real post and cannot be honestly pinned as an improvement there.
    The square is the shape class this flag DOES help (flat far surface
    across widened probe window), through the REAL boolean backend,
    exactly like ``FeatureLatticeRealBooleanTests`` in
    ``test_m36_feature_classification.py``."""

    @classmethod
    def setUpClass(cls):
        cls.ops = _MiterShapelyOps()
        cls.builder = SlotBuilder()

    def _run(self, points, dense, pitch=2.0):
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, root_solid_layers=0,
            web_position=0.5, z_drift_mm_per_mm=100.0,
            feature_dense_lattice_enabled=dense,
            feature_lattice_pitch_mm=pitch,
        )
        poly = self.ops.polygon_from_points(points)
        frame = LayerFrame.from_polygon(self.ops, poly)
        gen = StructureGenerator()
        ctx = LayerContext(layer_index=0, print_z=1.0)
        result = gen.generate(frame, spec, WALL_WIDTH_MM, ctx, safety_offset_mm=WALL_WIDTH_MM)
        built = [self.builder.build(i, spec.overshoot_mm) for i in result.intents]
        slots = [self.ops.polygon_from_points(b.points) for b in built]
        transformed = self.ops.diff(poly, slots)
        validator = TopologyValidator(self.ops, WALL_WIDTH_MM)
        validator.validate(
            [poly], transformed, slots, spec, 0, 1.0,
            slot_labels=[str(i.family_name) for i in result.intents],
            family_indices=[i.family_membership() for i in result.intents],
        )
        return result

    def test_square_post_dense_mode_is_strictly_deeper_and_one_loop(self):
        # skin_margin_mm=1.0 (this class's own `_run`, above) -- pinned exactly
        # so a future change to this fixture or the margin used here is visible
        # as a failing assertion rather than a silent drift. NOT the same
        # skin_margin_mm as ``FlagFalseIsByteIdenticalTests`` (also 1.0,
        # matches) or the dense-branch measurements against the real
        # post reported offline (0.8608mm, ~7.7-7.8mm depending on section),
        # which use a DIFFERENT margin and so land on a different (but
        # consistent) depth figure -- these are the same physical quantity
        # (local_thickness - skin_margin_mm - FULL_DEPTH_MARGIN_HEADROOM_MM)
        # evaluated at different margins, not a discrepancy.
        points = _rect_points(8.64, 8.67)
        shallow = self._run(points, dense=False)
        deep = self._run(points, dense=True)

        self.assertGreater(len(shallow.intents), 0)
        self.assertGreater(len(deep.intents), 0)
        shallow_depth = max(i.depth_mm() for i in shallow.intents)
        deep_depth = max(i.depth_mm() for i in deep.intents)
        self.assertAlmostEqual(
            shallow_depth, 1.7053, places=3,
            msg="shipped depth drifted from the pinned value",
        )
        self.assertAlmostEqual(
            deep_depth, 7.6698, places=3,
            msg="dense depth (at skin_margin_mm=1.0) drifted from the "
            "pinned value -- if this changed, re-measure and update the "
            "report this test's docstring cites",
        )
        self.assertGreater(
            deep_depth, shallow_depth,
            "dense mode did not deepen the square post's teeth",
        )
        # Both already validated as one closed loop inside `_run` (via
        # `validator.validate`, which raises on failure).


def _octagon_points(w, h, chamfer_frac=0.35):
    # An octagon (not a true chamfer) is enough to reproduce the mechanism: a
    # far surface that CONVERGES toward the chord extremities, which is what
    # extremity-widening probes for. Shared by both classes below so the "raw
    # attempt" and "wrapped, never-worse" numbers are measured on the identical
    # fixture.
    hw, hh = w / 2.0, h / 2.0
    cx, cy = hw * chamfer_frac, hh * chamfer_frac
    return (
        (-hw + cx, -hh), (hw - cx, -hh), (hw, -hh + cy), (hw, hh - cy),
        (hw - cx, hh), (-hw + cx, hh), (-hw, hh - cy), (-hw, -hh + cy),
    )


@unittest.skipUnless(_HAVE_SHAPELY, "shapely not installed")
class ChamferedPostRawDenseAttemptStillZeroTeethTests(unittest.TestCase):
    """THE COUNTEREXAMPLE ITSELF, pinned so it cannot silently disappear from the
    test suite even though the never-worse wrapper (tested below,
    ``ChamferedPostDenseModeNeverRegressesTests``) hides it from every
    ordinary caller of ``_generate_feature``: the RAW, un-wrapped dense attempt
    (``_generate_feature_core(..., dense=True)``, called directly, bypassing
    ``_generate_feature``'s own comparison) still loses every tooth on a
    chamfered/octagonal post -- thin-section check correctly rejects full local
    depth near the chamfer. This measured fact is WHY the never-worse wrapper
    exists; if this test ever starts failing (the raw attempt starts building
    teeth), the wrapper's own "never worse" guarantee would need re-examining,
    not just this test's fixture."""

    @classmethod
    def setUpClass(cls):
        cls.ops = _MiterShapelyOps()
        cls.builder = SlotBuilder()

    def _run_core(self, points, dense, pitch=2.0):
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=0.8608, root_solid_layers=0,
            web_position=0.5, z_drift_mm_per_mm=100.0,
            feature_dense_lattice_enabled=dense,
            feature_lattice_pitch_mm=pitch,
        )
        poly = self.ops.polygon_from_points(points)
        frame = LayerFrame.from_polygon(self.ops, poly)
        gen = StructureGenerator()
        ctx = LayerContext(layer_index=0, print_z=1.0)
        # Direct call to the un-wrapped core, bypassing `_generate_feature`'s
        # own dense-vs-shipped comparison -- this is deliberately NOT the
        # public entry point every other test in this file uses.
        return gen._generate_feature_core(
            frame, spec, WALL_WIDTH_MM, ctx,
            safety_offset_mm=WALL_WIDTH_MM, max_rotation_deg=90.0,
            dense=dense,
        )

    def test_raw_dense_attempt_loses_all_teeth_on_chamfered_post(self):
        points = _octagon_points(8.64, 8.67)
        shallow = self._run_core(points, dense=False)
        deep = self._run_core(points, dense=True)
        self.assertGreater(
            len(shallow.intents), 0,
            "fixture regression: shipped branch should still build teeth "
            "on a chamfered octagon",
        )
        self.assertEqual(
            len(deep.intents), 0,
            "expected the RAW dense attempt to lose every tooth on a "
            "chamfered post (thin-section) -- if this now builds "
            "teeth, the measured counterexample motivating the never"
            "-worse wrapper has changed and the report/comments "
            "referencing it need updating too",
        )
        self.assertTrue(
            any("thin section" in o.reason for o in deep.omitted
                if "feature_lattice" in str(o.family_name)),
            "expected the dense-mode omissions to be thin-section, "
            "not some other cause",
        )


@unittest.skipUnless(_HAVE_SHAPELY, "shapely not installed")
class ChamferedPostDenseModeNeverRegressesTests(unittest.TestCase):
    """THE FIX: through the PUBLIC ``_generate_feature`` entry point (what
    ``generate()`` actually calls), enabling the flag on the same
    chamfered/octagonal post above must produce EXACTLY the shipped
    (``dense=False``) result -- not zero teeth -- because
    ``_feature_result_is_better`` silently falls back to the shipped
    attempt whenever the dense one is not strictly better. No
    ``Degradation`` is recorded for this fallback: relative to the shipped
    default, nothing was lost."""

    @classmethod
    def setUpClass(cls):
        cls.ops = _MiterShapelyOps()

    def _run(self, points, dense, pitch=2.0):
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=0.8608, root_solid_layers=0,
            web_position=0.5, z_drift_mm_per_mm=100.0,
            feature_dense_lattice_enabled=dense,
            feature_lattice_pitch_mm=pitch,
        )
        poly = self.ops.polygon_from_points(points)
        frame = LayerFrame.from_polygon(self.ops, poly)
        gen = StructureGenerator()
        ctx = LayerContext(layer_index=0, print_z=1.0)
        return gen.generate(frame, spec, WALL_WIDTH_MM, ctx, safety_offset_mm=WALL_WIDTH_MM)

    def test_chamfered_post_dense_flag_matches_shipped_exactly(self):
        points = _octagon_points(8.64, 8.67)
        shipped = self._run(points, dense=False)
        wrapped = self._run(points, dense=True)

        def tup(i):
            return (i.start, i.end, i.width_mm, i.family_name, i.entry_lower)

        self.assertGreater(len(shipped.intents), 0)
        self.assertEqual(
            [tup(i) for i in wrapped.intents], [tup(i) for i in shipped.intents],
            "the never-worse wrapper should reproduce the shipped result "
            "byte-for-byte on a shape where the dense attempt loses",
        )
        self.assertEqual(len(wrapped.degradations), 0)
        self.assertEqual(len(shipped.degradations), 0)

    def test_fallback_leaves_a_trace_naming_both_attempts(self):
        # : own revisit trigger reads "revisit when the never-worse rule
        # fires", which requires the fallback to be OBSERVABLE. The wrapper
        # must not discard the losing dense attempt's diagnostics wholesale --
        # it carries them into the WINNING (shipped) result's `omitted` list,
        # tagged `family_name="feature_dense_fallback..."`, WITHOUT creating a
        # `Degradation` (still asserted above: `degradations` stays empty).
        points = _octagon_points(8.64, 8.67)
        wrapped = self._run(points, dense=True)

        fallback_entries = [
            o for o in wrapped.omitted
            if str(o.family_name).startswith("feature_dense_fallback")
        ]
        self.assertGreater(
            len(fallback_entries), 0,
            "expected the never-worse fallback to leave a trace in "
            "`omitted` when it fires",
        )
        summary = [
            o for o in fallback_entries
            if o.family_name == "feature_dense_fallback"
        ]
        self.assertEqual(len(summary), 1, "expected exactly one summary entry")
        self.assertIn("dense", summary[0].reason.lower())
        self.assertIn("shipped", summary[0].reason.lower())
        self.assertIn("teeth", summary[0].reason.lower())
        # The per-station omissions from the discarded dense attempt are
        # also present, each retagged rather than silently dropped.
        per_station = [
            o for o in fallback_entries
            if o.family_name != "feature_dense_fallback"
        ]
        self.assertGreater(len(per_station), 0)
        for o in per_station:
            self.assertTrue(o.family_name.startswith("feature_dense_fallback/"))
            self.assertIn("[discarded dense attempt]", o.reason)


class SmallerMarginIsRejectedByValidatorTests(unittest.TestCase):
    """Records WHY the dense branch does not use a smaller, feature-specific
    margin: building teeth at
    ``outer_wall_line_width_mm + safety_offset_mm`` (~0.84mm at this
    fixture's wall width) instead of ``skin_margin_mm`` (1.0mm here)
    produces teeth ``TopologyValidator`` immediately rejects, because its
    own invariant 4 re-derives and enforces ``skin_margin_mm`` regardless
    of what the generator used. Hand-builds the SAME slot geometry
    ``_generate_family``'s full-depth branch would if a smaller margin were
    threaded through (which the shipped code deliberately does NOT do --
    see ``StructureGenerator._generate_feature``'s dense-branch comment),
    to keep this evidence pinned independent of that code path."""

    @unittest.skipUnless(_HAVE_SHAPELY, "shapely not installed")
    def test_full_depth_at_smaller_margin_fails_validate(self):
        from aircraft_core.generator import WebIntent

        ops = _MiterShapelyOps()
        builder = SlotBuilder()
        points = _rect_points(8.64, 8.67)
        poly = ops.polygon_from_points(points)
        frame = LayerFrame.from_polygon(ops, poly)

        skin_margin_mm = 1.0
        smaller_margin_mm = WALL_WIDTH_MM + WALL_WIDTH_MM  # outer wall + safety offset
        self.assertLess(smaller_margin_mm, skin_margin_mm)

        width_mm = 0.21
        x = (frame.min_x + frame.max_x) / 2.0
        y_hi = frame.max_y
        y_lo = frame.min_y
        end_y = y_hi - ((y_hi - y_lo) - smaller_margin_mm)
        intent = WebIntent(
            start=(x, y_hi), end=(x, end_y), width_mm=width_mm,
            family_index=0, family_name="feature_lattice_top",
            entry_lower=False,
        )
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=skin_margin_mm, root_solid_layers=0,
            web_position=0.5,
        )
        built = builder.build(intent, spec.overshoot_mm)
        slot = ops.polygon_from_points(built.points)
        transformed = ops.diff(poly, [slot])
        validator = TopologyValidator(ops, WALL_WIDTH_MM)
        with self.assertRaises(TopologyError) as ctx:
            validator.validate(
                [poly], transformed, [slot], spec, 0, 1.0,
                slot_labels=["feature_lattice_top"], family_indices=[(0,)],
            )
        self.assertIn("margin", str(ctx.exception))


class AerofoilNoRegressionTests(unittest.TestCase):
    """Criterion (4): a pure aerofoil section is unaffected by this flag in
    either state -- classify_region never reports FEATURE for a real
    aerofoil, so ``_generate_feature`` (and this flag) is never reached."""

    def _naca_frame(self, naca_code="2412", chord_mm=60.0, span_mm=60.0):
        m, p, t = _mf.parse_naca4(naca_code)
        _tri, outline_mm, _bbox = _mf.airfoil_triangles(m, p, t, chord_mm, span_mm, 120)
        ops = StubOps()
        poly = PolygonShape(tuple(outline_mm))
        return ops, LayerFrame.from_polygon(ops, poly)

    def _spec(self, dense):
        return StructureSpec(
            gap_ratio=0.5, skin_margin_mm=0.86, root_solid_layers=0,
            web_position=0.5, rib_pitch_mm=10.0, web_depth_mm=3.0,
            feature_dense_lattice_enabled=dense,
        )

    def test_naca_wing_intents_identical_both_flag_states(self):
        ops, frame = self._naca_frame()
        gen = StructureGenerator()
        ctx = LayerContext(layer_index=0, print_z=1.0)
        r_false = gen.generate(frame, self._spec(False), WALL_WIDTH_MM, ctx,
                                safety_offset_mm=WALL_WIDTH_MM)
        r_true = gen.generate(frame, self._spec(True), WALL_WIDTH_MM, ctx,
                               safety_offset_mm=WALL_WIDTH_MM)
        self.assertGreater(len(r_false.intents), 0, "aerofoil fixture produced no structure")

        def tup(i):
            return (i.start, i.end, i.width_mm, i.family_name, i.entry_lower)

        self.assertEqual(
            [tup(i) for i in r_false.intents], [tup(i) for i in r_true.intents]
        )
        self.assertEqual(len(r_false.omitted), len(r_true.omitted))


if __name__ == "__main__":
    unittest.main()
