"""Web identity on intents/omissions, the LayerContext depth cap and the
``per_station_reentrant`` flag.

These fields are inert by default: ``depth_caps=None`` and
``per_station_reentrant=False`` must be byte-identical to those fields not
existing; these tests pin that as well as the behaviour when they are set.
"""
import unittest

from aircraft_core.continuity import check_continuity, web_report_rows
from aircraft_core.frame import LayerFrame, ReEntrantSectionError
from aircraft_core.generator import (
    LayerContext,
    OmittedWeb,
    StructureGenerator,
    WebIntent,
)
from aircraft_core.spec import StructureSpec, WebFamily
from stub_ops import PolygonShape, Rect, StubOps

WALL = 0.42


def _generate(frame, spec, z=0.0, index=0, **ctx_kw):
    return StructureGenerator().generate(
        frame, spec, WALL, LayerContext(layer_index=index, print_z=z, **ctx_kw)
    )


def _bracket():
    """A C-section: a solid column x in [0, 2] joins two bars, so any station
    with x > 2 crosses the material twice (a re-entrant section)."""
    return PolygonShape((
        (0.0, 0.0), (40.0, 0.0), (40.0, 3.0), (2.0, 3.0),
        (2.0, 17.0), (40.0, 17.0), (40.0, 20.0), (0.0, 20.0),
    ))


class WebIdentityStampTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.frame = LayerFrame.from_polygon(self.ops, Rect(0.0, 0.0, 60.0, 20.0))

    def test_lattice_intents_carry_family_k_side(self):
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0,
            families=(
                WebFamily(name="up", rib_pitch_mm=12.0, web_position=0.5,
                          web_depth_mm=1.5),
                WebFamily(name="lo", rib_pitch_mm=12.0, web_position=0.5,
                          web_depth_mm=1.5, entry_lower=True),
            ),
        )
        result = _generate(self.frame, spec)
        by_id = {i.web_id: i for i in result.intents}
        self.assertEqual(len(by_id), len(result.intents), "ids unique per layer")
        # anchor 30, pitch 12 -> stations 6,18,30,42,54 = k -2..2
        for k in (-2, -1, 0, 1, 2):
            self.assertIn((0, k, "U"), by_id)
            self.assertIn((1, k, "L"), by_id)
            self.assertAlmostEqual(by_id[(0, k, "U")].start[0], 30.0 + 12.0 * k)

    def test_candidate_stations_unchanged_and_k_consistent(self):
        gen = StructureGenerator()
        plain = gen._candidate_stations(self.frame, 7.0, 30.0)
        with_k = gen._candidate_stations_k(self.frame, 7.0, 30.0)
        self.assertEqual(plain, [x for _k, x in with_k])
        for k, x in with_k:
            self.assertEqual(x, 30.0 + k * 7.0)
        self.assertEqual(gen._candidate_stations_k(self.frame, None, 12.5), [(0, 12.5)])
        self.assertEqual(gen._candidate_stations(self.frame, None, 12.5), [12.5])

    def test_omissions_carry_the_identity_of_their_station(self):
        # stations at x=0 and x=60 are out of chord extent
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0,
            families=(WebFamily(name="up", rib_pitch_mm=30.0, web_position=0.5,
                                web_depth_mm=1.5),),
        )
        result = _generate(self.frame, spec)
        self.assertTrue(result.omitted)
        for rec in result.omitted:
            self.assertIsNotNone(rec.web_id, rec)
            self.assertEqual(rec.web_id[0], 0)
            self.assertEqual(rec.web_id[2], "U")
            self.assertEqual(rec.k, rec.web_id[1])
            self.assertAlmostEqual(rec.x_mm, 30.0 + 30.0 * rec.k)

    def test_identity_is_not_part_of_equality(self):
        a = WebIntent(start=(1.0, 2.0), end=(1.0, 1.0), width_mm=0.4)
        b = WebIntent(start=(1.0, 2.0), end=(1.0, 1.0), width_mm=0.4,
                      web_id=(0, 3, "U"))
        self.assertEqual(a, b)
        self.assertEqual(hash(a), hash(b))
        self.assertEqual(
            OmittedWeb(x_mm=1.0, reason="r"),
            OmittedWeb(x_mm=1.0, reason="r", k=2, web_id=(0, 2, "U")),
        )

    def test_opposing_pair_members_are_upper_and_lower_with_station_k(self):
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0,
            families=(WebFamily(name="tie", web_position=0.5, opposing_pair=True),),
        )
        result = _generate(self.frame, spec)
        ids = sorted(i.web_id for i in result.intents)
        self.assertEqual(ids, [(0, 0, "L"), (0, 0, "U")])

    def test_merge_carries_constituent_ids_and_own_x(self):
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0)
        gen = StructureGenerator()
        a = WebIntent(start=(10.0, 20.0), end=(10.0, 18.5), width_mm=0.5,
                      family_index=0, web_id=(0, 4, "U"))
        b = WebIntent(start=(10.8, 20.0), end=(10.8, 18.5), width_mm=0.5,
                      family_index=1, web_id=(1, 7, "U"))
        kept, omitted = gen._merge_cross_family_nodes(
            [a, b], WALL, frame=self.frame, spec=spec,
            effective_skin_margin_mm=1.0, ctx=LayerContext(0, 0.0))
        self.assertEqual(omitted, [])
        self.assertEqual(len(kept), 1)
        merged = kept[0]
        self.assertEqual(merged.web_id, (0, 4, "U"))
        self.assertEqual(set(merged.merged_web_ids), {(0, 4, "U"), (1, 7, "U")})
        self.assertEqual(sorted(merged.merged_web_xs), [10.0, 10.8])
        rows, _ = web_report_rows(kept, [])
        self.assertEqual({tuple(r[:3]) for r in rows}, {(0, 4, "U"), (1, 7, "U")})
        self.assertEqual({r[3] for r in rows}, {rows[0][3]})  # one merged depth


class DepthCapTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.frame = LayerFrame.from_polygon(self.ops, Rect(0.0, 0.0, 60.0, 20.0))
        self.lattice = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0,
            families=(WebFamily(name="up", rib_pitch_mm=12.0, web_position=0.5,
                                web_depth_mm=1.5),),
        )
        self.full = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0,
            families=(WebFamily(name="spar", web_position=0.5),),
        )

    @staticmethod
    def _depth(intent):
        return abs(intent.end[1] - intent.start[1])

    def test_none_caps_are_identical_to_no_caps(self):
        plain = _generate(self.frame, self.lattice)
        capped_none = _generate(self.frame, self.lattice, depth_caps=None)
        capped_empty = _generate(self.frame, self.lattice, depth_caps={})
        for other in (capped_none, capped_empty):
            self.assertEqual(plain.intents, other.intents)
            self.assertEqual(plain.omitted, other.omitted)
            self.assertEqual([i.web_id for i in plain.intents],
                             [i.web_id for i in other.intents])

    def test_cap_shallows_only_the_named_lattice_web(self):
        plain = _generate(self.frame, self.lattice)
        capped = _generate(self.frame, self.lattice,
                           depth_caps={(0, 0, "U"): 0.6})
        before = {i.web_id: i for i in plain.intents}
        after = {i.web_id: i for i in capped.intents}
        self.assertEqual(set(before), set(after))
        self.assertAlmostEqual(self._depth(before[(0, 0, "U")]), 1.5)
        self.assertAlmostEqual(self._depth(after[(0, 0, "U")]), 0.6)
        for wid in before:
            if wid != (0, 0, "U"):
                self.assertEqual(before[wid], after[wid])

    def test_cap_deeper_than_nominal_never_deepens(self):
        capped = _generate(self.frame, self.lattice,
                           depth_caps={(0, 0, "U"): 9.0})
        plain = _generate(self.frame, self.lattice)
        self.assertEqual(plain.intents, capped.intents)

    def test_cap_shallows_a_full_depth_web(self):
        plain = _generate(self.frame, self.full)
        capped = _generate(self.frame, self.full, depth_caps={(0, 0, "U"): 2.0})
        self.assertGreater(self._depth(plain.intents[0]), 2.0)
        self.assertAlmostEqual(self._depth(capped.intents[0]), 2.0)

    def test_cap_below_the_degenerate_guard_omits_with_the_existing_reason(self):
        capped = _generate(self.frame, self.lattice,
                           depth_caps={(0, 0, "U"): 0.0})
        self.assertNotIn((0, 0, "U"), [i.web_id for i in capped.intents])
        rec = [o for o in capped.omitted if o.web_id == (0, 0, "U")]
        self.assertEqual(len(rec), 1)
        self.assertIn("degenerate web depth", rec[0].reason)


class PerStationReentrantTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.frame = LayerFrame.from_polygon(self.ops, _bracket())

    def _spec(self, flag):
        return StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, per_station_reentrant=flag,
            families=(WebFamily(name="a", rib_pitch_mm=7.0, web_position=0.025,
                                web_depth_mm=1.5),),
        )

    def test_default_is_off_and_the_layer_still_raises(self):
        self.assertFalse(StructureSpec().per_station_reentrant)
        with self.assertRaises(ReEntrantSectionError):
            _generate(self.frame, self._spec(False))

    def test_flag_on_omits_only_the_reentrant_stations(self):
        result = _generate(self.frame, self._spec(True))
        self.assertEqual([i.web_id for i in result.intents], [(0, 0, "U")])
        self.assertAlmostEqual(result.intents[0].start[0], 1.0)
        reasons = {o.web_id: o.reason for o in result.omitted}
        self.assertEqual(sorted(reasons), [(0, k, "U") for k in range(1, 6)])
        for reason in reasons.values():
            self.assertTrue(reason.startswith("re_entrant_station:"), reason)

    def test_flag_on_changes_nothing_when_no_station_is_reentrant(self):
        rect = LayerFrame.from_polygon(self.ops, Rect(0.0, 0.0, 60.0, 20.0))
        fams = (WebFamily(name="a", rib_pitch_mm=9.0, web_position=0.4,
                          web_depth_mm=1.5, z_drift_mm_per_mm=0.5, z_period_mm=40.0),
                WebFamily(name="b", rib_pitch_mm=9.0, web_position=0.4,
                          web_depth_mm=1.5, entry_lower=True,
                          z_drift_mm_per_mm=-0.5, z_period_mm=40.0))
        off = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0, families=fams)
        on = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0, families=fams,
                           per_station_reentrant=True)
        for z in (0.0, 3.3, 9.9, 17.0, 31.0):
            a = _generate(rect, off, z=z)
            b = _generate(rect, on, z=z)
            self.assertEqual(a.intents, b.intents)
            self.assertEqual(a.omitted, b.omitted)
            self.assertEqual(a.degradations.__len__(), b.degradations.__len__())

    def test_merge_over_a_notch_omits_both_constituents_only_with_the_flag(self):
        a = WebIntent(start=(1.0, 20.0), end=(1.0, 18.5), width_mm=0.5,
                      family_index=0, web_id=(0, 0, "U"))
        b = WebIntent(start=(1.9, 20.0), end=(1.9, 18.5), width_mm=0.5,
                      family_index=1, web_id=(1, 0, "U"))
        gen = StructureGenerator()
        ctx = LayerContext(0, 0.0)
        with self.assertRaises(ReEntrantSectionError):
            gen._merge_cross_family_nodes(
                [a, b], WALL, frame=self.frame, spec=self._spec(False),
                effective_skin_margin_mm=1.0, ctx=ctx)
        kept, omitted = gen._merge_cross_family_nodes(
            [a, b], WALL, frame=self.frame, spec=self._spec(True),
            effective_skin_margin_mm=1.0, ctx=ctx)
        self.assertEqual(kept, [])
        self.assertEqual(sorted(o.web_id for o in omitted), [(0, 0, "U"), (1, 0, "U")])
        for o in omitted:
            self.assertTrue(o.reason.startswith("re_entrant_station:"))


class IdentityStabilityOnTaperedDriftingWingTests(unittest.TestCase):
    """The identity ``(family, k, side)`` must follow ONE physical web as the
    comb drifts 0.2 mm/layer, the chord tapers and stations enter/leave at the
    chord ends: a web present on two adjacent layers moves by about the drift
    only. Position-in-the-list identity would fail this at every count change.
    """

    N = 170
    LAYER_H = 0.2

    @classmethod
    def setUpClass(cls):
        self = cls  # built once: 170 stub-geometry layers are slow
        self.ops = StubOps()
        fam = lambda name, rate, lower: WebFamily(  # noqa: E731
            name=name, rib_pitch_chord_frac=0.12, web_position=0.35,
            web_depth_mm=1.5, entry_lower=lower,
            z_drift_mm_per_mm=rate, z_period_mm=64.6)
        self.spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, root_solid_layers=0,
            families=(fam("p_up", 1.0, False), fam("m_up", -1.0, False),
                      fam("p_lo", 1.0, True), fam("m_lo", -1.0, True)),
        )
        self.records = []
        self.count_changes = 0
        prev_n = None
        self.layers_ids = []
        gen = StructureGenerator()
        for i in range(self.N):
            z = 0.2 + self.LAYER_H * i
            width = 70.0 - 0.05 * i  # taper 70 -> 59 mm
            frame = LayerFrame.from_polygon(self.ops, Rect(0.0, 0.0, width, 20.0))
            res = gen.generate(frame, self.spec, WALL,
                               LayerContext(layer_index=i, print_z=z))
            rows, omit = web_report_rows(res.intents, res.omitted)
            self.records.append({"layer_index": i, "print_z": z,
                                 "webs": rows, "web_omit": omit})
            self.layers_ids.append({tuple(r[:3]): r for r in rows})
            n = len(res.intents)
            if prev_n is not None and n != prev_n:
                self.count_changes += 1
            prev_n = n

    def test_ids_unique_within_every_layer(self):
        for rec in self.records:
            ids = [tuple(r[:3]) for r in rec["webs"]]
            self.assertEqual(len(ids), len(set(ids)), rec["layer_index"])

    def test_the_station_count_does_change_over_the_run(self):
        # Otherwise the stability check below proves nothing.
        self.assertGreater(self.count_changes, 3)

    def test_a_web_on_adjacent_layers_moves_only_by_the_drift(self):
        checked = 0
        for prev, cur in zip(self.layers_ids, self.layers_ids[1:]):
            for wid in set(prev) & set(cur):
                dx = abs(cur[wid][4] - prev[wid][4])
                # 0.2 drift + taper shift of the anchor (0.35 * 0.05) and of
                # k*pitch (|k| <= 8, 0.12 * 0.05): well under 0.35
                self.assertLess(dx, 0.35, (wid, dx))
                checked += 1
        self.assertGreater(checked, 1000)

    def test_a_web_keeps_its_station_through_a_neighbours_entry_and_exit(self):
        # Pick a layer where a family's station set changes and require the
        # surviving stations to keep their k labels (and sit where the k
        # arithmetic says), instead of being renumbered by position.
        found = False
        for prev, cur in zip(self.layers_ids, self.layers_ids[1:]):
            if set(prev) != set(cur):
                found = True
                for wid in set(prev) & set(cur):
                    self.assertLess(abs(cur[wid][4] - prev[wid][4]), 0.35)
        self.assertTrue(found)

    def test_checker_on_steady_lattice(self):
        result = check_continuity(
            self.records, ramp_mm=self.LAYER_H, dx_max_mm=0.5 * WALL,
            exempt={0: "root_solid"}, violation_cap=100000)
        counts = result["violation_counts"]
        # Constant 1.5 mm lattice depth: no increase.
        self.assertEqual(counts["increase"], 0)
        # MEASURED, not hoped: the nominal drift is 0.2 mm/layer and the
        # chord taper makes the comb breathe (k * 0.12 * dW per layer on top),
        # so outer stations step 0.2-0.25 mm/layer against dx_max = 0.21.
        # Those are drift, not jumps: every flagged step stays below 0.35.
        for v in result["violations"]:
            if v["type"] == "x_step" and not v["merged"]:
                self.assertLess(abs(v["dx"]), 0.35, v)
        # Stations that enter at the chord end appear at FULL depth -- exactly
        # the unramped starts exists to remove; they must be visible.
        self.assertGreater(counts["unramped_start"], 0)


if __name__ == "__main__":
    unittest.main()
