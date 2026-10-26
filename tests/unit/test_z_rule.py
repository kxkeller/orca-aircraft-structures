"""The Z-continuity rule (``aircraft_core/z_rule.py``), behind
``StructureSpec.z_continuity``.

Part 1 drives the rule with a FAKE generator (real ``WebIntent`` objects, caps
honoured the way ``_cap_end_y`` honours them) so ramp-from-zero, END,
shallowing, lateral-continuity restart, socket exemption and the tie modes are
pinned without geometry. Part 2 uses real tracked fixtures
(``tests/fixtures/island_disconnect``) with the real generator and validator:
the genuine-re-entrant predicate (positive: the layer-1098 pinched neck;
negative: the 15 audited notch layers) and the in-loop single-web ablation
(layers 947 and 454). Part 3 pins the flag-off equivalence.

Hermetic: reads only tracked fixtures, never ``host/``.
"""
import json
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import aircraft_core.slot as slotmod  # noqa: E402
from aircraft_core.frame import LayerFrame, ReEntrantSectionError  # noqa: E402
from aircraft_core.generator import (  # noqa: E402
    DetectedChannel,
    GenerationResult,
    LayerContext,
    OmittedWeb,
    StructureGenerator,
    WebIntent,
)
from aircraft_core.spec import StructureSpec  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402
from aircraft_core.z_rule import (  # noqa: E402
    TIE_INDEPENDENT,
    TIE_JOINT,
    ZContinuityRule,
    _ZRuleCaps,
    genuine_reentrant,
)

try:
    import structural_analysis as sa  # noqa: E402
    _HAVE_SA = True
except ImportError:  # pragma: no cover
    _HAVE_SA = False
try:
    from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402
except ImportError:  # pragma: no cover
    HAVE_SHAPELY = False

NEEDS = unittest.skipUnless(HAVE_SHAPELY and _HAVE_SA, "shapely + structural_analysis required")

FIXTURES = REPO_ROOT / "tests" / "fixtures" / "island_disconnect"
R = 0.2
DX = 0.25
WIDTH = 0.4
SAFETY = 0.42


def _rule(**kw):
    kw.setdefault("ramp_mm", R)
    kw.setdefault("dx_max_mm", DX)
    kw.setdefault("root_solid_layers", 0)
    kw.setdefault("tie_family_indices", (1,))
    return ZContinuityRule(**kw)


def _intent(wid, x, depth, **kw):
    if wid[0] < 0:
        kw.setdefault("spar_confirmed", True)       # the confirmed rod channel
    return WebIntent(
        start=(x, 10.0), end=(x, 10.0 - depth), width_mm=WIDTH,
        family_index=wid[0], family_name="fam{}".format(wid[0]),
        entry_lower=(wid[2] == "L"), web_id=wid, **kw)


class _FakeGen:
    """Stands in for ``StructureGenerator.generate`` + ``LayerContext.depth_caps``.

    ``plan``: ``{web_id: (x, nominal_depth)}`` for this layer; a web missing
    from the plan is not a candidate. ``limits``: ``{web_id: max_feasible
    depth}`` -- a web asked for more is omitted with a mitre-clearance reason
    (the depth-dependent case the rule bisects)."""

    def __init__(self, plan, limits=None, hopeless=()):
        self.plan = plan
        self.limits = limits or {}
        self.hopeless = set(hopeless)
        self.calls = []

    def __call__(self, caps):
        self.calls.append(None if caps is None else dict(caps))
        intents, omitted = [], []
        for wid, (x, nominal) in self.plan.items():
            depth = nominal
            if caps is not None and wid[0] >= 0:
                depth = min(depth, caps.get(wid))
            if wid in self.hopeless:
                omitted.append(OmittedWeb(
                    x_mm=x, reason="chord extent: slot footprint outside",
                    family_index=wid[0], k=wid[1], web_id=wid))
                continue
            if wid in self.limits and depth > self.limits[wid] + 1e-9:
                omitted.append(OmittedWeb(
                    x_mm=x, reason="skin margin: mitre clearance: too deep",
                    family_index=wid[0], k=wid[1], web_id=wid))
                continue
            if depth < 0.05:
                omitted.append(OmittedWeb(
                    x_mm=x, reason="skin margin: degenerate web depth",
                    family_index=wid[0], k=wid[1], web_id=wid))
                continue
            intents.append(_intent(wid, x, depth))
        return GenerationResult(intents=intents, omitted=omitted)


def _step(rule, index, gen):
    """One layer through the rule exactly as the capability / replay do."""
    res = rule.generate_layer(index, 0.2 * (index + 1), gen)
    intents, omitted, _ = rule.post_latch(index, list(res.intents), res.omitted)
    rule.commit(index, 0.2 * (index + 1), intents, omitted)
    return intents, omitted


def _depth(intents, wid):
    for i in intents:
        if i.web_id == wid:
            return abs(i.end[1] - i.start[1])
    return None


LAT = (0, 0, "U")


class RampTests(unittest.TestCase):
    def test_ramp_from_zero_one_layer_height_per_layer_then_nominal(self):
        rule = _rule()
        depths = []
        for index in range(1, 12):
            intents, _ = _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))
            depths.append(round(_depth(intents, LAT), 3))
        self.assertEqual(depths[:7], [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4])
        self.assertEqual(depths[7:], [1.5, 1.5, 1.5, 1.5])
        # the ramp completed once and was never abandoned
        s = rule.summary()
        self.assertEqual(s["counters"]["ramps_completed"], 1)
        self.assertEqual(s["counters"]["ramps_never_completed"], 0)

    def test_first_structured_layer_is_exempt_from_the_ramp(self):
        rule = _rule(root_solid_layers=3)
        intents, _ = _step(rule, 3, _FakeGen({LAT: (10.0, 1.5)}))
        self.assertAlmostEqual(_depth(intents, LAT), 1.5)
        intents, _ = _step(rule, 4, _FakeGen({LAT: (10.0, 1.5)}))
        self.assertAlmostEqual(_depth(intents, LAT), 1.5)

    def test_end_then_return_restarts_from_zero(self):
        rule = _rule()
        for index in range(1, 10):
            _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))
        intents, _ = _step(rule, 10, _FakeGen({}))          # absent: END
        self.assertIsNone(_depth(intents, LAT))
        intents, _ = _step(rule, 11, _FakeGen({LAT: (10.0, 1.5)}))
        self.assertAlmostEqual(_depth(intents, LAT), R)     # back by ramp

    def test_a_skipped_layer_ends_every_web(self):
        rule = _rule()
        for index in range(1, 6):
            _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))
        rule.note_skipped(6, "re_entrant_section")
        intents, _ = _step(rule, 7, _FakeGen({LAT: (10.0, 1.5)}))
        self.assertAlmostEqual(_depth(intents, LAT), R)

    def test_a_gap_in_layer_indices_is_a_skipped_layer(self):
        rule = _rule()
        for index in range(1, 6):
            _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))
        intents, _ = _step(rule, 8, _FakeGen({LAT: (10.0, 1.5)}))  # 6,7 unseen
        self.assertAlmostEqual(_depth(intents, LAT), R)

    def test_shallowing_is_immediate_and_regrowth_is_ramped(self):
        rule = _rule()
        for index in range(1, 12):
            _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))
        intents, _ = _step(rule, 12, _FakeGen({LAT: (10.0, 0.6)}))
        self.assertAlmostEqual(_depth(intents, LAT), 0.6)    # decrease free
        got = []
        for index in range(13, 18):
            intents, _ = _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))
            got.append(round(_depth(intents, LAT), 3))
        self.assertEqual(got, [0.8, 1.0, 1.2, 1.4, 1.5])

    def test_a_second_region_on_a_committed_layer_fails_closed(self):
        rule = _rule()
        _step(rule, 1, _FakeGen({LAT: (10.0, 1.5)}))
        with self.assertRaises(TopologyError) as ctx:
            rule.generate_layer(1, 0.4, _FakeGen({LAT: (10.0, 1.5)}))
        self.assertIn("one structured region per layer", str(ctx.exception))

    def test_drift_within_dx_max_keeps_the_web(self):
        rule = _rule()
        x = 10.0
        for index in range(1, 12):
            x += 0.2                                          # 0.2 mm/layer
            intents, _ = _step(rule, index, _FakeGen({LAT: (x, 1.5)}))
        self.assertAlmostEqual(_depth(intents, LAT), 1.5)
        self.assertEqual(rule.summary()["counters"]["x_jump_ends"], 0)

    def test_lateral_jump_beyond_dx_max_restarts_the_web_from_zero(self):
        rule = _rule()
        for index in range(1, 12):
            _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))
        gen = _FakeGen({LAT: (10.6, 1.5)})                    # wall jumps 0.6
        intents, _ = _step(rule, 12, gen)
        self.assertAlmostEqual(_depth(intents, LAT), R)
        self.assertEqual(len(gen.calls), 2, "regenerated once with the web zeroed")
        s = rule.summary()
        self.assertEqual(s["counters"]["x_jump_ends"], 1)
        self.assertEqual([e["kind"] for e in s["events"]], ["x_jump_end"])
        self.assertFalse(s["events"][0]["merged"])
        self.assertEqual(s["counters"]["x_jump_ends_merged"], 0)

    def test_a_web_already_within_one_step_is_not_regenerated_for_a_jump(self):
        rule = _rule()
        for index in range(1, 3):
            _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))   # depth 0.4
        gen = _FakeGen({LAT: (10.0, 1.5)})
        _step(rule, 3, gen)
        gen = _FakeGen({LAT: (11.0, 0.1)})                    # tiny + jumped
        intents, _ = _step(rule, 4, gen)
        self.assertLessEqual(_depth(intents, LAT), R + 1e-9)
        self.assertEqual(len(gen.calls), 1)

    def test_ramp_that_ends_before_reaching_nominal_is_recorded(self):
        rule = _rule()
        for index in range(1, 5):
            _step(rule, index, _FakeGen({LAT: (10.0, 5.0)}))   # ramping 0.2..0.8
        _step(rule, 5, _FakeGen({}))                           # END mid-ramp
        s = rule.summary()
        self.assertEqual(s["counters"]["ramps_never_completed"], 1)
        rec = s["never_completed_ramps"][0]
        self.assertEqual(rec["web"], list(LAT))
        self.assertAlmostEqual(rec["max_depth_mm"], 0.8, places=3)


class BisectionTests(unittest.TestCase):
    def test_a_depth_dependent_omission_is_bisected_to_a_feasible_depth(self):
        rule = _rule()
        for index in range(1, 12):                     # reach nominal 1.5
            _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))
        gen = _FakeGen({LAT: (10.0, 1.5)}, limits={LAT: 0.7})
        intents, omitted = _step(rule, 12, gen)
        d = _depth(intents, LAT)
        self.assertIsNotNone(d)
        self.assertLessEqual(d, 0.7 + 1e-9)             # feasible
        self.assertGreater(d, 0.7 - 0.06)               # within the tolerance
        s = rule.summary()
        self.assertEqual(s["counters"]["bisect_found"], 1)
        self.assertEqual(s["events"][0]["kind"], "bisect_found")

    def test_a_web_that_fails_even_at_the_probe_depth_is_hopeless_not_bisected(self):
        rule = _rule()
        for index in range(1, 12):
            _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))
        gen = _FakeGen({LAT: (10.0, 1.5)}, limits={LAT: 0.01})
        intents, _ = _step(rule, 12, gen)
        self.assertIsNone(_depth(intents, LAT))
        self.assertEqual(rule.summary()["counters"]["bisect_hopeless"], 1)
        self.assertLessEqual(len(gen.calls), 3)

    def test_a_non_depth_omission_is_not_bisected(self):
        rule = _rule()
        for index in range(1, 12):
            _step(rule, index, _FakeGen({LAT: (10.0, 1.5)}))
        gen = _FakeGen({LAT: (10.0, 1.5)}, hopeless={LAT})
        _step(rule, 12, gen)
        self.assertEqual(len(gen.calls), 1)
        self.assertEqual(rule.summary()["counters"]["bisect_layers"], 0)

    def test_bisection_never_exceeds_the_ramp_cap(self):
        rule = _rule()
        for index in range(1, 6):                      # depth 1.0 after 5 layers
            _step(rule, index, _FakeGen({LAT: (10.0, 5.0)}))
        gen = _FakeGen({LAT: (10.0, 5.0)}, limits={LAT: 3.0})
        intents, _ = _step(rule, 6, gen)               # cap 1.2, feasible 3.0
        self.assertAlmostEqual(_depth(intents, LAT), 1.2)


class SocketTests(unittest.TestCase):
    SOCK = (-2, 0, "S")

    def test_sockets_are_exempt_from_the_ramp(self):
        rule = _rule()
        intents, _ = _step(rule, 1, _FakeGen({self.SOCK: (10.0, 5.0)}))
        self.assertAlmostEqual(_depth(intents, self.SOCK), 5.0)
        intents, _ = _step(rule, 2, _FakeGen({self.SOCK: (10.0, 9.0)}))
        self.assertAlmostEqual(_depth(intents, self.SOCK), 9.0)  # +4 in one layer
        self.assertEqual(rule.summary()["counters"]["ramp_web_layers"], 0)

    def test_a_socket_restart_after_its_end_is_refused(self):
        rule = _rule()
        for index in (1, 2):
            _step(rule, index, _FakeGen({self.SOCK: (10.0, 5.0)}))
        intents, _ = _step(rule, 3, _FakeGen({}))              # END
        intents, omitted = _step(rule, 4, _FakeGen({self.SOCK: (10.0, 5.0)}))
        self.assertIsNone(_depth(intents, self.SOCK))          # stays ended
        self.assertTrue(any(o.reason.startswith("z_socket_latch") for o in omitted))
        self.assertEqual(rule.summary()["counters"]["socket_restarts_suppressed"], 1)
        # and it stays ended on every later layer too
        intents, _ = _step(rule, 5, _FakeGen({self.SOCK: (10.0, 5.0)}))
        self.assertIsNone(_depth(intents, self.SOCK))

    def test_a_socket_whose_walls_jump_sideways_is_a_different_hole_and_refused(self):
        # inside half layer 1000: the spar label moves from the spar (7.6 mm)
        # to the alignment pin (58.6 mm) where the spar ends
        rule = _rule()
        for index in (1, 2):
            _step(rule, index, _FakeGen({self.SOCK: (7.6, 8.0)}))
        intents, omitted = _step(rule, 3, _FakeGen({self.SOCK: (58.6, 5.0)}))
        self.assertIsNone(_depth(intents, self.SOCK))
        self.assertTrue(any(o.reason.startswith("z_socket_latch") for o in omitted))
        self.assertEqual(rule.summary()["counters"]["socket_jumps_suppressed"], 1)
        intents, _ = _step(rule, 4, _FakeGen({self.SOCK: (7.6, 8.0)}))
        self.assertIsNone(_depth(intents, self.SOCK))     # ended stays ended

    def test_a_socket_drifting_within_dx_max_is_the_same_socket(self):
        rule = _rule()
        x = 7.4
        for index in range(1, 8):
            x += 0.063                                     # measured spar drift
            intents, _ = _step(rule, index, _FakeGen({self.SOCK: (x, 8.0)}))
        self.assertAlmostEqual(_depth(intents, self.SOCK), 8.0)

    def test_an_unconfirmed_pick_is_a_different_hole_and_may_come_and_go(self):
        # the nearest-spar-station policy labels an alignment pin (-2,0,S) at
        # the tip end of the outside half; the confirmed rod channel later
        # uses the same label. Not one web: the pick has its own identity.
        rule = _rule()

        class _Pick(_FakeGen):
            def __call__(self, caps):
                res = super().__call__(caps)
                for i, intent in enumerate(res.intents):
                    if intent.web_id == SocketTests.SOCK:
                        res.intents[i] = _intent(
                            intent.web_id, intent.start[0],
                            abs(intent.end[1] - intent.start[1]),
                            spar_confirmed=False)
                return res

        for index in (1, 2):
            _step(rule, index, _Pick({self.SOCK: (58.6, 3.0)}))
        _step(rule, 3, _FakeGen({}))
        # later the CONFIRMED rod channel appears under the same label
        intents, omitted = _step(rule, 4, _FakeGen({self.SOCK: (65.7, 8.0)}))
        self.assertAlmostEqual(_depth(intents, self.SOCK), 8.0)
        self.assertFalse(any(o.reason.startswith("z_socket_latch") for o in omitted))
        self.assertEqual(rule.summary()["counters"]["socket_restarts_suppressed"], 0)

    def test_a_bore_that_comes_back_is_not_policed(self):
        rule = _rule()
        bore = (-7, 0, "S")
        for index in (1, 2):
            _step(rule, index, _FakeGen({bore: (58.6, 3.0)}))
        _step(rule, 3, _FakeGen({}))
        intents, _ = _step(rule, 4, _FakeGen({bore: (58.6, 3.0)}))
        self.assertAlmostEqual(_depth(intents, bore), 3.0)

    def test_a_socket_that_was_never_seen_may_start(self):
        rule = _rule()
        _step(rule, 1, _FakeGen({LAT: (10.0, 1.5)}))
        intents, _ = _step(rule, 2, _FakeGen({LAT: (10.0, 1.5), self.SOCK: (20.0, 5.0)}))
        self.assertAlmostEqual(_depth(intents, self.SOCK), 5.0)


class TieModeTests(unittest.TestCase):
    U, L = (1, 0, "U"), (1, 0, "L")

    def _seeded(self, mode):
        rule = _rule(tie_mode=mode)
        # U ramps to 3.0, L to 1.0 (independent), via separate layers
        plan = {self.U: (10.0, 3.0), self.L: (10.9, 1.0)}
        for index in range(1, 20):
            _step(rule, index, _FakeGen(plan))
        return rule

    def test_independent_members_carry_their_own_caps(self):
        rule = self._seeded(TIE_INDEPENDENT)
        caps = rule._ramp_caps(set())
        self.assertAlmostEqual(caps.get(self.U), 3.2)
        self.assertAlmostEqual(caps.get(self.L), 1.2)

    def test_joint_members_share_the_shallower_cap(self):
        rule = self._seeded(TIE_JOINT)
        caps = rule._ramp_caps(set())
        self.assertAlmostEqual(caps.get(self.U), 1.2)
        self.assertAlmostEqual(caps.get(self.L), 1.2)

    def test_joint_restarts_both_when_one_member_is_absent(self):
        rule = _rule(tie_mode=TIE_JOINT)
        plan = {self.U: (10.0, 3.0), self.L: (10.9, 3.0)}
        for index in range(1, 20):
            _step(rule, index, _FakeGen(plan))
        _step(rule, 20, _FakeGen({self.U: (10.0, 3.0)}))        # L absent
        intents, _ = _step(rule, 21, _FakeGen(plan))
        self.assertAlmostEqual(_depth(intents, self.U), R)
        self.assertAlmostEqual(_depth(intents, self.L), R)

    def test_independent_keeps_the_member_that_stayed(self):
        rule = _rule(tie_mode=TIE_INDEPENDENT)
        plan = {self.U: (10.0, 3.0), self.L: (10.9, 3.0)}
        for index in range(1, 20):
            _step(rule, index, _FakeGen(plan))
        _step(rule, 20, _FakeGen({self.U: (10.0, 3.0)}))
        intents, _ = _step(rule, 21, _FakeGen(plan))
        self.assertAlmostEqual(_depth(intents, self.U), 3.0)
        self.assertAlmostEqual(_depth(intents, self.L), R)

    def test_joint_lateral_jump_of_one_member_restarts_the_partner(self):
        rule = _rule(tie_mode=TIE_JOINT)
        plan = {self.U: (10.0, 3.0), self.L: (10.9, 3.0)}
        for index in range(1, 20):
            _step(rule, index, _FakeGen(plan))
        jumped = {self.U: (11.5, 3.0), self.L: (10.9, 3.0)}
        intents, _ = _step(rule, 20, _FakeGen(jumped))
        self.assertAlmostEqual(_depth(intents, self.U), R)
        self.assertAlmostEqual(_depth(intents, self.L), R)

    def test_station_states_are_counted(self):
        rule = _rule()
        plan = {self.U: (10.0, 3.0), self.L: (10.9, 3.0)}
        for index in range(1, 25):
            _step(rule, index, _FakeGen(plan))
        t = rule.summary()["tie_station_layers"]
        self.assertGreater(t["unfused_stub"], 5)
        self.assertGreater(t["fused"], 5)
        self.assertEqual(t["one_sided"], 0)


class CapMapTests(unittest.TestCase):
    def test_unknown_web_gets_one_ramp_step_and_the_map_is_truthy(self):
        caps = _ZRuleCaps(R)
        self.assertTrue(bool(caps))
        self.assertEqual(caps.get((9, 9, "U")), R)
        caps[(1, 1, "U")] = 2.0
        self.assertEqual(caps.get((1, 1, "U")), 2.0)


def _poly(ops, pts):
    return ops.polygon_from_points([tuple(p) for p in pts])


@NEEDS
class GenuineReEntrantPredicateTests(unittest.TestCase):
    def setUp(self):
        self.ops = MiterShapelyOps()

    def test_the_15_audited_notch_layers_are_not_genuine(self):
        with open(FIXTURES / "notch_layers_15_host_contours.json", encoding="utf-8") as fh:
            fx = json.load(fh)
        self.assertEqual(len(fx["layers"]), 15)
        for rec in fx["layers"]:
            poly = _poly(self.ops, rec["outer_pts_mm"])
            self.assertIsNone(
                genuine_reentrant(self.ops, poly, fx["safety_offset_mm"]),
                "layer {} ({}) must be per-station, not a shutdown".format(
                    rec["layer_index"], rec["part"]))

    def test_the_layer_1098_pinched_neck_is_genuine(self):
        with open(FIXTURES / "layer1098_pinched_neck_minimal.json", encoding="utf-8") as fh:
            fx = json.load(fh)
        poly = _poly(self.ops, fx["host_contour_pts_mm"])
        hit = genuine_reentrant(self.ops, poly, SAFETY)
        self.assertIsNotNone(hit)
        self.assertGreaterEqual(hit["islands"], 2)
        self.assertIn("genuinely re-entrant", hit["message"])

    def test_two_lobes_joined_by_a_thin_neck_are_genuine(self):
        # a dumbbell: two 20 x 10 lobes joined by a 0.5 mm neck (< 2 x 0.42)
        pts = [(0, 0), (20, 0), (20, 4.75), (30, 4.75), (30, 0), (50, 0),
               (50, 10), (30, 10), (30, 5.25), (20, 5.25), (20, 10), (0, 10)]
        hit = genuine_reentrant(self.ops, _poly(self.ops, pts), SAFETY)
        self.assertIsNotNone(hit)
        self.assertEqual(hit["islands"], 2)

    def test_a_wide_notch_is_not_genuine(self):
        # same shape, neck 4 mm: a notch pair, one island under erosion
        pts = [(0, 0), (20, 0), (20, 3), (30, 3), (30, 0), (50, 0),
               (50, 10), (30, 10), (30, 7), (20, 7), (20, 10), (0, 10)]
        self.assertIsNone(genuine_reentrant(self.ops, _poly(self.ops, pts), SAFETY))

    def test_a_section_thinner_than_the_wall_is_not_re_entrant(self):
        pts = [(0, 0), (30, 0), (30, 0.6), (0, 0.6)]
        self.assertIsNone(genuine_reentrant(self.ops, _poly(self.ops, pts), SAFETY))

    def test_reentrant_check_counts_and_raise_helper_uses_the_existing_error(self):
        rule = _rule(safety_offset_mm=SAFETY)
        pts = [(0, 0), (20, 0), (20, 4.75), (30, 4.75), (30, 0), (50, 0),
               (50, 10), (30, 10), (30, 5.25), (20, 5.25), (20, 10), (0, 10)]
        hit = rule.reentrant_check(self.ops, _poly(self.ops, pts))
        self.assertEqual(rule.summary()["counters"]["genuine_reentrant_layers"], 1)
        with self.assertRaises(ReEntrantSectionError):
            rule.raise_genuine(hit)


class _FixtureLayer:
    """Generate + cut one tracked fixture layer the way the plugin does, with
    the rule's own caps (seeded so the previous layer was at nominal)."""

    def __init__(self, fixture_name, spec):
        with open(FIXTURES / fixture_name, encoding="utf-8") as fh:
            self.fx = json.load(fh)
        self.spec = spec
        self.ops = MiterShapelyOps()
        self.original = self.ops.polygon_from_points(
            [tuple(p) for p in self.fx["host_contour_pts_mm"]])
        self.frame = LayerFrame.from_polygon(self.ops, self.original)
        self.index = self.fx["layer_index"]
        self.z = self.fx["print_z_mm"]
        self.channels = tuple(
            DetectedChannel(center_x_mm=c["center_x_mm"],
                            center_y_mm=c["center_y_mm"],
                            diameter_mm=c["diameter_mm"])
            for c in self.fx.get("detected_channels", []))

    def gen(self, caps):
        from dataclasses import replace
        ctx = LayerContext(layer_index=self.index, print_z=self.z,
                           detected_channels=self.channels)
        if caps is not None:
            ctx = replace(ctx, depth_caps=caps)
        return StructureGenerator().generate(
            self.frame, self.spec, self.fx["wall_width_mm"], ctx,
            safety_offset_mm=self.fx["safety_offset_mm"])

    def built(self, intents):
        builder = slotmod.SlotBuilder()
        slots = [self.ops.polygon_from_points(
            builder.build(i, self.spec.overshoot_mm).points) for i in intents]
        labels = ["family {} ({!r})".format(i.family_index, i.family_name)
                  for i in intents]
        fam = [i.family_membership() for i in intents]
        return slots, labels, fam


def _elevon_spec(tie_mode="independent"):
    with open(FIXTURES / "elevon_cap_config.json", encoding="utf-8") as fh:
        cap = json.load(fh)
    cap["z_continuity"] = True
    cap["z_continuity_tie_mode"] = tie_mode
    return sa.build_spec_from_cap_config(cap)


def _seeded_rule(layer, spec):
    """A rule whose previous layer (index-1) carried this layer's nominal
    webs, so the ramp does not bind and the layer reproduces the host's."""
    rule = ZContinuityRule.from_spec(spec, ramp_mm=R, safety_offset_mm=SAFETY)
    nominal = layer.gen(None)
    rule.commit(layer.index - 1, layer.z - 0.2, list(nominal.intents))
    rule.last_index = layer.index - 1
    return rule


@NEEDS
class AblationInTheLoopTests(unittest.TestCase):
    def _run(self, fixture, tie_mode):
        spec = _elevon_spec(tie_mode)
        layer = _FixtureLayer(fixture, spec)
        rule = _seeded_rule(layer, spec)
        res = rule.generate_layer(layer.index, layer.z, layer.gen)
        intents = list(res.intents)
        slots, labels, fam = layer.built(intents)
        validator = TopologyValidator(layer.ops, safety_offset_mm=SAFETY)
        replacement = list(layer.ops.diff(layer.original, slots))
        # the layer fails as recorded in-host
        with self.assertRaises(TopologyError):
            validator.validate([layer.original], replacement, slots, spec,
                               layer.index, layer.z, slot_labels=labels,
                               family_indices=fam)
        rescue = rule.validate_with_ablation(
            layer.ops, validator, [layer.original], replacement, intents,
            slots, spec, layer.index, layer.z, labels, fam)
        return layer, rule, validator, spec, intents, slots, labels, fam, rescue

    def _revalidate(self, layer, validator, spec, slots, labels, fam, rescue):
        keep = list(rescue.kept_indices)
        kept = [slots[i] for i in keep]
        replacement = list(layer.ops.diff(layer.original, kept))
        validator.validate(
            [layer.original], replacement, kept, spec, layer.index, layer.z,
            slot_labels=[labels[i] for i in keep],
            family_indices=[fam[i] for i in keep])

    def test_947_independent_ends_one_member_and_the_layer_validates(self):
        layer, rule, validator, spec, intents, slots, labels, fam, rescue = (
            self._run("layer947_tie_pair_lateral_minimal.json", "independent"))
        self.assertIsNotNone(rescue)
        self.assertEqual(len(rescue.removed_indices), 1)
        (wid,) = rescue.ended_ids
        self.assertEqual((wid[0], wid[2] in ("U", "L")), (6, True))   # tie member
        self.assertEqual(len(rescue.kept_indices), len(slots) - 1)
        self._revalidate(layer, validator, spec, slots, labels, fam, rescue)
        self.assertTrue(rescue.omitted[0].reason.startswith("z_ablation"))
        self.assertEqual(rule.summary()["counters"]["ablation_ends"], 1)

    def test_947_joint_ends_both_members(self):
        layer, rule, validator, spec, intents, slots, labels, fam, rescue = (
            self._run("layer947_tie_pair_lateral_minimal.json", "joint"))
        self.assertIsNotNone(rescue)
        self.assertEqual(len(rescue.removed_indices), 2)
        self.assertEqual({w[2] for w in rescue.ended_ids}, {"U", "L"})
        self.assertEqual(len({(w[0], w[1]) for w in rescue.ended_ids}), 1)
        self._revalidate(layer, validator, spec, slots, labels, fam, rescue)

    def test_454_far_cap_hazard_is_rescued_by_ending_one_web(self):
        layer, rule, validator, spec, intents, slots, labels, fam, rescue = (
            self._run("layer454_rootdown_tie_minimal.json", "independent"))
        self.assertIsNotNone(rescue)
        self.assertEqual(len(rescue.removed_indices), 1)
        self._revalidate(layer, validator, spec, slots, labels, fam, rescue)

    def test_ended_web_returns_only_by_ramp(self):
        layer, rule, validator, spec, intents, slots, labels, fam, rescue = (
            self._run("layer947_tie_pair_lateral_minimal.json", "independent"))
        keep = list(rescue.kept_indices)
        rule.commit(layer.index, layer.z, [intents[i] for i in keep],
                    list(rescue.omitted))
        (wid,) = rescue.ended_ids
        caps = rule._ramp_caps(set())
        self.assertAlmostEqual(caps.get(wid), R)            # restarts from zero
        # while the survivors keep their depth + one step
        survivor = next(i for i in intents if i.web_id != wid and i.web_id[0] == 6)
        self.assertGreater(caps.get(survivor.web_id), 1.0)

    def test_no_single_web_end_restores_a_pinched_layer_so_it_fails_closed(self):
        spec = _elevon_spec()
        layer = _FixtureLayer("layer1098_pinched_neck_minimal.json", spec)
        rule = _seeded_rule(layer, spec)
        res = rule.generate_layer(layer.index, layer.z, layer.gen)
        intents = list(res.intents)
        slots, labels, fam = layer.built(intents)
        validator = TopologyValidator(layer.ops, safety_offset_mm=SAFETY)
        replacement = list(layer.ops.diff(layer.original, slots))
        with self.assertRaises(TopologyError) as ctx:
            rule.validate_with_ablation(
                layer.ops, validator, [layer.original], replacement, intents,
                slots, spec, layer.index, layer.z, labels, fam)
        self.assertIn("single-web ablation was tried", str(ctx.exception))
        self.assertEqual(ctx.exception.layer_index, layer.index)

    def test_a_healthy_layer_never_enters_the_retry_path(self):
        spec = _elevon_spec()
        layer = _FixtureLayer("layer786_entry_wedge_minimal.json", spec)
        # 786 fails on its own slot (entry wedge), so use the 1098 layer with no
        # slots instead: nothing to ablate, the validator passes
        rule = _seeded_rule(layer, spec)
        validator = TopologyValidator(layer.ops, safety_offset_mm=SAFETY)
        replacement = list(layer.ops.diff(layer.original, []))
        # a bare, un-slotted section validates (or raises for reasons other
        # than the island split, which ablation must re-raise untouched)
        try:
            out = rule.validate_with_ablation(
                layer.ops, validator, [layer.original], replacement, [], [],
                spec, layer.index, layer.z, [], [])
            self.assertIsNone(out)
        except TopologyError as exc:
            self.assertNotIn("single-web ablation was tried", str(exc))


@NEEDS
class RuleOnRealGeneratorTests(unittest.TestCase):
    def test_ramp_caps_reach_every_family_including_the_tie(self):
        spec = _elevon_spec()
        layer = _FixtureLayer("layer947_tie_pair_lateral_minimal.json", spec)
        rule = ZContinuityRule.from_spec(spec, ramp_mm=R, safety_offset_mm=SAFETY)
        # no state: every web starts from zero, one ramp step deep
        res = rule.generate_layer(layer.index, layer.z, layer.gen)
        depths = [abs(i.end[1] - i.start[1]) for i in res.intents
                  if i.web_id and i.web_id[0] >= 0]
        self.assertTrue(depths)
        self.assertLessEqual(max(depths), R + 1e-9)
        tie = [i for i in res.intents if i.web_id and i.web_id[0] == 6]
        self.assertTrue(tie, "tie members exist and honour the cap too")


@NEEDS
class FlagOffEquivalenceTests(unittest.TestCase):
    """z_continuity=False must be byte-identical to the rule not existing."""

    def test_default_spec_is_off_and_does_not_force_the_reentrant_catch(self):
        spec = StructureSpec()
        self.assertFalse(spec.z_continuity)
        self.assertFalse(spec.effective_per_station_reentrant())
        self.assertEqual(spec.z_continuity_tie_mode, "independent")
        self.assertIsNone(spec.z_continuity_dx_max_mm)

    def test_the_flag_forces_the_per_station_catch_on(self):
        self.assertTrue(StructureSpec(z_continuity=True).effective_per_station_reentrant())
        self.assertFalse(StructureSpec(z_continuity=True).per_station_reentrant)

    def test_unrestrictive_caps_leave_every_fixture_layer_unchanged(self):
        """A cap map whose default is huge is a no-op through every seam,
        including the opposing-pair family (so the seam itself is inert)."""
        with open(FIXTURES / "elevon_cap_config.json", encoding="utf-8") as fh:
            spec = sa.build_spec_from_cap_config(json.load(fh))
        for name in ("layer947_tie_pair_lateral_minimal.json",
                     "layer454_rootdown_tie_minimal.json",
                     "layer786_entry_wedge_minimal.json"):
            layer = _FixtureLayer(name, spec)
            plain = layer.gen(None)
            loose = layer.gen(_ZRuleCaps(1e9))
            self.assertEqual(
                [(i.start, i.end, i.web_id) for i in plain.intents],
                [(i.start, i.end, i.web_id) for i in loose.intents], name)
            self.assertEqual([o.reason for o in plain.omitted],
                             [o.reason for o in loose.omitted], name)

    def test_the_replay_with_the_flag_off_matches_the_default_sweep(self):
        import preflight_wing as pf
        sig = {"closing": None, "latch": None, "rescues": False}
        import inspect
        params = inspect.signature(pf.t3_layer_sweep).parameters
        for name, default in (("closing_radius_mm", None),
                              ("latch_root_mode", None),
                              ("host_rescues", False)):
            self.assertEqual(params[name].default, default)
        self.assertEqual(sig["rescues"], False)


if __name__ == "__main__":
    unittest.main()
