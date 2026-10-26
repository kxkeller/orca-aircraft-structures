"""Wall-matching variant: ``ZContinuityRule(wall_matching=True)``, spec key
``z_continuity_wall_matching``, and the checker's ``wall_matching`` mode
(``aircraft_core/continuity.py``).

Pure tests: a FAKE generator that honours depth caps (like
``tests/unit/test_z_rule.py``) and hand-built per-layer rows. They pin the
model: a printed slot wall is supported by the deepest previous wall of the
SAME orientation (left/right edge) and SAME entry side within ``dx_max``;
a slot may be at most ``min(support over its two walls) + r`` deep; a wall
with no match at all is a restart. Hermetic: no mesh, no host/.
"""
import unittest

from aircraft_core.continuity import (
    VIOLATION_X_STEP,
    check_continuity,
    wall_support,
)
from aircraft_core.generator import GenerationResult, WebIntent
from aircraft_core.spec import SpecError, StructureSpec
from aircraft_core.z_rule import ZContinuityRule

R = 0.2
DX = 0.25
WIDTH = 0.4


def _rule(wall_matching=True, **kw):
    kw.setdefault("ramp_mm", R)
    kw.setdefault("dx_max_mm", DX)
    kw.setdefault("root_solid_layers", 0)
    kw.setdefault("tie_family_indices", ())
    return ZContinuityRule(wall_matching=wall_matching, **kw)


class _Gen:
    """``slots``: list of ``(ids, centre_x, width, nominal_depth)``. A slot with
    several ids is a merged slot (one intent carrying all of them); its depth
    is the minimum of its constituents' caps."""

    def __init__(self, slots):
        self.slots = slots
        self.calls = 0

    def __call__(self, caps):
        self.calls += 1
        intents = []
        for ids, x, width, nominal in self.slots:
            depth = nominal
            if caps is not None:
                depth = min(depth, min(caps.get(w) for w in ids))
            kw = {}
            if len(ids) > 1:
                kw = dict(merged_web_ids=tuple(ids),
                          merged_web_xs=tuple(x for _ in ids))
            intents.append(WebIntent(
                start=(x, 10.0), end=(x, 10.0 - depth), width_mm=width,
                family_index=ids[0][0], family_name="fam{}".format(ids[0][0]),
                entry_lower=(ids[0][2] == "L"), web_id=ids[0], **kw))
        return GenerationResult(intents=intents, omitted=[])


def _step(rule, index, gen):
    res = rule.generate_layer(index, 0.2 * (index + 1), gen)
    intents, omitted, _ = rule.post_latch(index, list(res.intents), res.omitted)
    rule.commit(index, 0.2 * (index + 1), intents, omitted)
    return {w: abs(i.end[1] - i.start[1]) for i in intents
            for w in (list(i.merged_web_ids) or [i.web_id])}


A = (0, 0, "U")
B = (0, 1, "U")
C = (0, 2, "U")


def _grow(rule, slots, layers):
    """Run ``layers`` identical layers and return the last depth map."""
    out = None
    for index in range(1, layers + 1):
        out = _step(rule, index, _Gen(slots))
    return out


class MergeAndSplitTests(unittest.TestCase):
    A_B = [([A], 10.0, WIDTH, 1.5), ([B], 14.0, WIDTH, 1.5)]
    MERGED = [([A, B], 12.0, 4.4, 1.5)]            # walls 9.8 .. 14.2

    def _deep_then(self, rule, then_slots):
        _grow(rule, self.A_B, 12)                    # both at nominal 1.5
        return _step(rule, 13, _Gen(then_slots))

    def test_merge_keeps_depth(self):
        rule = _rule()
        got = self._deep_then(rule, self.MERGED)
        self.assertAlmostEqual(got[A], 1.5)
        self.assertAlmostEqual(got[B], 1.5)
        c = rule.summary()["counters"]
        self.assertEqual(c["x_jump_ends"], 0)
        self.assertEqual(c["wall_limited_webs"], 0)

    def test_phase2_rule_restarts_both_constituents_at_a_merge(self):
        rule = _rule(wall_matching=False)
        got = self._deep_then(rule, self.MERGED)
        self.assertAlmostEqual(got[A], R)
        self.assertAlmostEqual(got[B], R)
        c = rule.summary()["counters"]
        self.assertEqual(c["x_jump_ends"], 2)
        self.assertEqual(c["x_jump_ends_merged"], 2)

    def test_split_inner_walls_ramp_from_r_growing_by_r(self):
        rule = _rule()
        self._deep_then(rule, self.MERGED)
        for index in range(14, 17):
            _step(rule, index, _Gen(self.MERGED))
        got = []
        for index in range(17, 22):
            d = _step(rule, index, _Gen(self.A_B))
            got.append((round(d[A], 3), round(d[B], 3)))
        # first layer after the split: the NEW inner wall (A's right, B's
        # left) has nothing below -> r; then +r per layer
        self.assertEqual(got, [(0.2, 0.2), (0.4, 0.4), (0.6, 0.6),
                               (0.8, 0.8), (1.0, 1.0)])
        c = rule.summary()["counters"]
        self.assertEqual(c["x_jump_ends"], 2)
        self.assertEqual(c["x_jump_ends_merged"], 2)

    def test_split_outer_walls_alone_would_not_have_restarted_the_web(self):
        # a split whose new web keeps BOTH walls (it sat on top of one
        # constituent) is not a restart
        rule = _rule()
        self._deep_then(rule, self.MERGED)
        got = _step(rule, 14, _Gen([([A], 12.0, 4.4, 1.5)]))   # same slot, one id
        self.assertAlmostEqual(got[A], 1.5)
        self.assertEqual(rule.summary()["counters"]["x_jump_ends"], 0)


class JumpAndSupportDepthTests(unittest.TestCase):
    def test_a_true_jump_still_restarts_and_regenerates_once(self):
        rule = _rule()
        _grow(rule, [([A], 10.0, WIDTH, 1.5)], 12)
        gen = _Gen([([A], 10.6, WIDTH, 1.5)])              # nothing within dx
        got = _step(rule, 13, gen)
        self.assertAlmostEqual(got[A], R)
        self.assertEqual(gen.calls, 2)
        c = rule.summary()["counters"]
        self.assertEqual(c["x_jump_ends"], 1)
        self.assertEqual(c["x_jump_ends_merged"], 0)

    def test_drift_within_dx_max_keeps_the_web(self):
        rule = _rule()
        x = 10.0
        for index in range(1, 14):
            x += 0.2
            got = _step(rule, index, _Gen([([A], x, WIDTH, 1.5)]))
        self.assertAlmostEqual(got[A], 1.5)
        self.assertEqual(rule.summary()["counters"]["x_jump_ends"], 0)

    def test_depth_is_the_matched_walls_depth_plus_r(self):
        # X (deep) jumps onto where a SHALLOW web Y stood: supported depth is
        # Y's depth, not X's own, and not zero
        X, Y = (0, 0, "U"), (0, 1, "U")
        rule = _rule()
        for index in range(1, 13):
            _step(rule, index, _Gen([([X], 10.0, WIDTH, 3.0),
                                     ([Y], 20.0, WIDTH, 0.6)]))
        got = _step(rule, 13, _Gen([([X], 20.0, WIDTH, 3.0)]))   # Y gone
        self.assertAlmostEqual(got[X], 0.6 + R)
        # the web is a restart (its own walls found no match) only when NO
        # wall is within dx_max; here both match Y's, so it is just limited
        c = rule.summary()["counters"]
        self.assertEqual(c["x_jump_ends"], 0)
        self.assertEqual(c["wall_limited_webs"], 1)

    def test_the_weaker_wall_sets_the_depth(self):
        # left wall matches a deep wall, right wall a shallow one
        X, P, Q = (0, 0, "U"), (0, 1, "U"), (0, 2, "U")
        rule = _rule()
        for index in range(1, 14):
            _step(rule, index, _Gen([([X], 5.0, WIDTH, 3.0),
                                     ([P], 20.0, WIDTH, 3.0),
                                     ([Q], 20.4, WIDTH, 0.6)]))
        # X jumps to a slot whose left edge 19.8 matches P.left 19.8 (deep) and
        # whose right edge 20.6 matches Q.right 20.6 (depth 0.6)
        got = _step(rule, 14, _Gen([([X], 20.2, 0.8, 3.0)]))
        self.assertAlmostEqual(got[X], 0.6 + R)

    def test_orientation_is_respected(self):
        # C's LEFT wall lands on A's RIGHT wall and C's RIGHT wall on B's LEFT
        # wall: same X, wrong orientation -> no support at all
        a, b, c = (0, 0, "U"), (0, 1, "U"), (0, 2, "U")
        rule = _rule()
        for index in range(1, 13):
            _step(rule, index, _Gen([([a], 10.0, WIDTH, 1.5),     # 9.8 .. 10.2
                                     ([b], 10.8, WIDTH, 1.5),     # 10.6 .. 11.0
                                     ([c], 30.0, WIDTH, 1.5)]))
        got = _step(rule, 13, _Gen([([c], 10.4, WIDTH, 1.5)]))   # 10.2 .. 10.6
        self.assertAlmostEqual(got[c], R)
        self.assertEqual(rule.summary()["counters"]["x_jump_ends"], 1)

    def test_entry_side_is_respected(self):
        up, low = (0, 0, "U"), (2, 0, "L")
        rule = _rule()
        for index in range(1, 13):
            _step(rule, index, _Gen([([up], 10.0, WIDTH, 1.5),
                                     ([low], 30.0, WIDTH, 1.5)]))
        got = _step(rule, 13, _Gen([([low], 10.0, WIDTH, 1.5)]))
        self.assertAlmostEqual(got[low], R)

    def test_flag_false_is_the_phase2_rule_on_a_scripted_run(self):
        script = [
            [([A], 10.0, WIDTH, 1.5), ([B], 14.0, WIDTH, 1.5)],
            [([A, B], 12.0, 4.4, 1.5)],
            [([A], 10.0, WIDTH, 1.5), ([B], 14.0, WIDTH, 1.5)],
            [([A], 10.6, WIDTH, 1.5), ([B], 14.0, WIDTH, 1.5)],
        ]
        off = _rule(wall_matching=False)
        legacy = ZContinuityRule(ramp_mm=R, dx_max_mm=DX, root_solid_layers=0,
                                 tie_family_indices=())   # the old signature
        for index in range(1, 40):
            slots = script[min(index // 10, 3)]
            self.assertEqual(_step(off, index, _Gen(slots)),
                             _step(legacy, index, _Gen(slots)))
        self.assertEqual(off.summary()["counters"], legacy.summary()["counters"])
        self.assertGreater(off.summary()["counters"]["x_jump_ends"], 0)


class SpecTests(unittest.TestCase):
    def test_default_is_true_and_from_spec_carries_it(self):
        spec = StructureSpec(z_continuity=True)
        self.assertTrue(spec.z_continuity_wall_matching)
        rule = ZContinuityRule.from_spec(spec, ramp_mm=R, safety_offset_mm=0.42)
        self.assertTrue(rule.wall_matching)
        spec = StructureSpec(z_continuity=True, z_continuity_wall_matching=False)
        rule = ZContinuityRule.from_spec(spec, ramp_mm=R, safety_offset_mm=0.42)
        self.assertFalse(rule.wall_matching)

    def test_non_bool_is_refused(self):
        with self.assertRaises(SpecError):
            StructureSpec(z_continuity_wall_matching="yes").validate()


def _row(fi, k, side, depth, left, right, merged_x=None):
    x = round(0.5 * (left + right), 3)
    return [fi, k, side, depth, x, merged_x, left, right]


def _layers(per_layer):
    return [{"layer_index": i, "print_z": 0.2 * (i + 1), "webs": rows,
             "web_omit": []} for i, rows in enumerate(per_layer)]


def _check(per_layer, wall_matching, exempt=None):
    exempt = {0: "root_solid"} if exempt is None else exempt   # layer 0 = bed
    return check_continuity(_layers(per_layer), ramp_mm=R, dx_max_mm=DX,
                            exempt=exempt, wall_matching=wall_matching)


def _xsteps(result):
    return [v for v in result["violations"] if v["type"] == VIOLATION_X_STEP]


class CheckerTests(unittest.TestCase):
    def test_wall_support_helper(self):
        best, nearest = wall_support([(1.0, 2.0), (1.1, 3.0)], 1.05, 0.1)
        self.assertEqual(best, 3.0)                  # the deepest in reach
        self.assertAlmostEqual(abs(nearest), 0.05)
        self.assertEqual(wall_support([(1.0, 2.0)], 2.0, 0.1)[0], None)
        self.assertEqual(wall_support([], 2.0, 0.1), (None, None))

    def _merge_history(self):
        a_b = [_row(0, 0, "U", 1.5, 9.8, 10.2), _row(0, 1, "U", 1.5, 13.8, 14.2)]
        merged = [_row(0, 0, "U", 1.5, 9.8, 14.2, 12.0),
                  _row(0, 1, "U", 1.5, 9.8, 14.2, 12.0)]
        return [a_b, a_b, merged]

    def test_a_merge_is_a_violation_only_in_the_strict_mode(self):
        hist = self._merge_history()
        strict = _check(hist, wall_matching=False)
        wall = _check(hist, wall_matching=True)
        self.assertGreater(len(_xsteps(strict)), 0)
        self.assertEqual(wall["violation_total"], 0, wall["violations"])
        self.assertTrue(wall["wall_matching"])
        self.assertFalse(strict["wall_matching"])

    def test_a_split_that_keeps_the_depth_is_a_violation_in_both_modes(self):
        a_b = [_row(0, 0, "U", 1.5, 9.8, 10.2), _row(0, 1, "U", 1.5, 13.8, 14.2)]
        merged = [_row(0, 0, "U", 1.5, 9.8, 14.2, 12.0),
                  _row(0, 1, "U", 1.5, 9.8, 14.2, 12.0)]
        hist = [a_b, merged, merged, a_b]
        wall = _check(hist, wall_matching=True)
        found = _xsteps(wall)
        self.assertGreater(len(found), 0)
        self.assertEqual(found[0]["reason"], "no_matching_wall")
        self.assertEqual(found[0]["layer"], 3)

    def test_a_split_that_ramps_passes(self):
        a_b_deep = [_row(0, 0, "U", 1.5, 9.8, 10.2), _row(0, 1, "U", 1.5, 13.8, 14.2)]
        merged = [_row(0, 0, "U", 1.5, 9.8, 14.2, 12.0),
                  _row(0, 1, "U", 1.5, 9.8, 14.2, 12.0)]
        ramp = lambda d: [_row(0, 0, "U", d, 9.8, 10.2), _row(0, 1, "U", d, 13.8, 14.2)]
        hist = [a_b_deep, merged, merged, ramp(0.2), ramp(0.4), ramp(0.6)]
        self.assertEqual(_check(hist, wall_matching=True)["violation_total"], 0)

    def test_a_true_jump_is_a_violation_in_both_modes(self):
        hist = [[_row(0, 0, "U", 1.5, 9.8, 10.2)]] * 2 + [
            [_row(0, 0, "U", 1.5, 10.4, 10.8)]]
        self.assertGreater(len(_xsteps(_check(hist, wall_matching=False))), 0)
        self.assertGreater(len(_xsteps(_check(hist, wall_matching=True))), 0)

    def test_a_matched_but_too_shallow_wall_is_a_violation(self):
        # X (deep) lands on shallow Y's walls: depth 3.0 > Y's 0.6 + r
        hist = [[_row(0, 0, "U", 3.0, 9.8, 10.2), _row(0, 1, "U", 0.6, 19.8, 20.2)]] * 2 + [
            [_row(0, 0, "U", 3.0, 19.8, 20.2)]]
        found = _xsteps(_check(hist, wall_matching=True))
        self.assertEqual(len(found), 1)
        self.assertEqual(found[0]["reason"], "matched_wall_too_shallow")
        # at Y's depth + r it is fine
        ok = hist[:2] + [[_row(0, 0, "U", 0.8, 19.8, 20.2)]]
        self.assertEqual(_check(ok, wall_matching=True)["violation_total"], 0)

    def test_orientation_and_entry_side_are_respected(self):
        # c's left edge sits on a's right edge and c's right edge on b's left
        # edge: wrong orientation both ways
        base = [_row(0, 0, "U", 1.5, 9.8, 10.2), _row(0, 1, "U", 1.5, 10.6, 11.0),
                _row(0, 2, "U", 1.5, 29.8, 30.2)]
        hist = [base, base, [_row(0, 2, "U", 1.5, 10.2, 10.6)]]
        self.assertEqual(len(_xsteps(_check(hist, wall_matching=True))), 1)
        # the same edges on the OTHER entry side do not support a U slot either
        base = [_row(0, 0, "L", 1.5, 9.8, 10.2), _row(0, 2, "U", 1.5, 29.8, 30.2)]
        hist = [base, base, [_row(0, 2, "U", 1.5, 9.8, 10.2)]]
        self.assertEqual(len(_xsteps(_check(hist, wall_matching=True))), 1)

    def test_a_ramped_start_after_a_jump_is_not_a_violation(self):
        hist = [[_row(0, 0, "U", 1.5, 9.8, 10.2)]] * 2 + [
            [_row(0, 0, "U", 0.2, 10.6, 11.0)]]
        self.assertEqual(_check(hist, wall_matching=True)["violation_total"], 0)

    def test_phase1_rows_without_walls_use_the_old_centre_check(self):
        rows = lambda x: [[0, 0, "U", 1.5, x]]
        hist = [rows(10.0), rows(10.0), rows(11.0)]
        self.assertGreater(len(_xsteps(_check(hist, wall_matching=True))), 0)


if __name__ == "__main__":
    unittest.main()
