"""The web-continuity checker and the per-web report rows
(``aircraft_core/continuity.py``).

Pure tests over hand-built per-layer records: no geometry, no host. The
checker is REPORT-ONLY -- it counts and lists, it never raises.
"""
import math
import unittest

from aircraft_core.continuity import (
    EXEMPT_FINAL_TOP,
    EXEMPT_ROOT_SOLID,
    VIOLATION_BLINK,
    VIOLATION_INCREASE,
    VIOLATION_SOCKET_RESTART,
    VIOLATION_UNRAMPED_START,
    VIOLATION_X_STEP,
    check_continuity,
    decode_rows,
    default_exemptions,
    encode_rows,
    layers_from_report,
    omission_code,
    web_report_rows,
)
from aircraft_core.generator import OmittedWeb, WebIntent

R = 0.2          # ramp step = layer height
DX_MAX = 0.21    # half of a 0.42 mm outer wall


def _layers(n, webs_by_layer, omit_by_layer=None, z0=0.2):
    omit_by_layer = omit_by_layer or {}
    return [
        {
            "layer_index": i,
            "print_z": round(z0 + 0.2 * i, 4),
            "webs": [list(r) for r in webs_by_layer.get(i, [])],
            "web_omit": [list(r) for r in omit_by_layer.get(i, [])],
        }
        for i in range(n)
    ]


def _run(layers, exempt=None, **kw):
    return check_continuity(layers, ramp_mm=R, dx_max_mm=DX_MAX, exempt=exempt, **kw)


def _types(result):
    return {k: v for k, v in result["violation_counts"].items() if v}


class RampedStartAndEndTests(unittest.TestCase):
    def test_ramped_start_then_steady_passes(self):
        # 0.2, 0.4, ... 1.4 then steady 1.5: every step <= r.
        depths = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.5, 1.5, 1.5]
        webs = {i: [(0, 3, "U", d, 10.0)] for i, d in enumerate(depths)}
        result = _run(_layers(len(depths), webs))
        self.assertEqual(result["violation_total"], 0, result["violations"])

    def test_abrupt_end_is_allowed(self):
        # web (0,3,U) ends after layer 4; another web keeps the run going
        webs = {i: [(1, 0, "U", 1.5, 40.0)] for i in range(12)}
        for i in range(5):
            webs[i] = webs[i] + [(0, 3, "U", 1.5, 10.0)]
        result = _run(_layers(12, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(result["violation_total"], 0, result["violations"])
        self.assertEqual(result["by_family"]["0"]["ends"], 1)

    def test_start_at_full_depth_is_flagged_unramped(self):
        webs = {i: [(0, 1, "U", 1.5, 10.0)] for i in range(4, 9)}
        result = _run(_layers(9, webs))
        self.assertEqual(_types(result), {VIOLATION_UNRAMPED_START: 1})
        v = result["violations"][0]
        self.assertEqual(v["web"], [0, 1, "U"])
        self.assertEqual(v["layer"], 4)

    def test_increase_over_one_layer_height_flagged(self):
        webs = {0: [(0, 1, "U", 0.2, 10.0)], 1: [(0, 1, "U", 0.4, 10.0)],
                2: [(0, 1, "U", 1.0, 10.0)], 3: [(0, 1, "U", 1.0, 10.0)]}
        result = _run(_layers(4, webs))
        self.assertEqual(_types(result), {VIOLATION_INCREASE: 1})
        self.assertEqual(result["violations"][0]["layer"], 2)

    def test_decrease_is_never_a_violation(self):
        webs = {0: [(0, 1, "U", 1.5, 10.0)], 1: [(0, 1, "U", 0.1, 10.0)],
                2: [(0, 1, "U", 0.1, 10.0)]}
        result = _run(_layers(3, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(result["violation_total"], 0)


class BlinkTests(unittest.TestCase):
    def test_blink_flagged_with_gap_and_reason(self):
        webs = {i: [(0, 2, "L", 1.5, 20.0)] for i in (0, 1, 2, 5, 6)}
        omit = {3: [(0, 2, "L", 20.0, "mitre_clearance")],
                4: [(0, 2, "L", 20.0, "mitre_clearance")]}
        result = _run(_layers(7, webs, omit), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(_types(result), {VIOLATION_BLINK: 1})
        v = result["violations"][0]
        self.assertEqual(v["layer"], 5)
        self.assertEqual(v["gap_layers"], 2)
        self.assertEqual(v["gap_reasons"], ["mitre_clearance"])

    def test_return_by_ramp_after_gap_is_not_a_blink(self):
        webs = {0: [(0, 2, "L", 1.5, 20.0)], 1: [(0, 2, "L", 1.5, 20.0)],
                4: [(0, 2, "L", 0.2, 20.0)], 5: [(0, 2, "L", 0.4, 20.0)]}
        result = _run(_layers(6, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(result["violation_total"], 0, result["violations"])

    def test_pair_omission_attributed_to_both_members(self):
        webs = {i: [(2, 0, "U", 5.0, 30.0), (2, 0, "L", 5.0, 30.8)] for i in (0, 1, 4)}
        omit = {2: [(2, 0, "P", 30.4, "opposing_pair")],
                3: [(2, 0, "P", 30.4, "opposing_pair")]}
        result = _run(_layers(5, webs, omit), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(_types(result), {VIOLATION_BLINK: 2})
        for v in result["violations"]:
            self.assertEqual(v["gap_reasons"], ["opposing_pair"])


class LateralDriftTests(unittest.TestCase):
    def test_drift_of_point_two_mm_per_layer_passes(self):
        webs = {i: [(0, 0, "U", 1.5, 10.0 + 0.2 * i)] for i in range(200)}
        result = _run(_layers(200, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(result["violation_total"], 0, result["violations"])

    def test_x_jump_with_depth_carried_is_flagged(self):
        webs = {i: [(0, 0, "U", 1.5, 10.0)] for i in range(4)}
        webs.update({i: [(0, 0, "U", 1.5, 11.0)] for i in range(4, 8)})
        result = _run(_layers(8, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(_types(result), {VIOLATION_X_STEP: 1})
        v = result["violations"][0]
        self.assertAlmostEqual(v["dx"], 1.0)
        self.assertEqual(v["layer"], 4)

    def test_x_jump_across_an_absence_is_a_blink_not_an_x_step(self):
        webs = {0: [(0, 0, "U", 1.5, 10.0)], 1: [(0, 0, "U", 1.5, 10.0)],
                3: [(0, 0, "U", 0.2, 12.0)]}
        result = _run(_layers(4, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(result["violation_total"], 0, result["violations"])

    def test_dx_max_and_ramp_are_recorded(self):
        result = _run(_layers(1, {}))
        self.assertEqual(result["ramp_mm"], R)
        self.assertEqual(result["dx_max_mm"], DX_MAX)


class MergeAndSplitTests(unittest.TestCase):
    def test_merge_with_small_wall_move_passes_and_credits_both(self):
        # Two webs approach; on layer 3 they merge: merged printed X is the
        # midpoint (0.1 mm from each), depth is the shallower one.
        webs = {
            0: [(0, 0, "U", 1.5, 9.8), (1, 0, "U", 1.5, 10.2)],
            1: [(0, 0, "U", 1.5, 9.9), (1, 0, "U", 1.5, 10.1)],
            2: [(0, 0, "U", 1.5, 10.0), (1, 0, "U", 1.5, 10.0)],
            3: [(0, 0, "U", 1.2, 10.1, 10.0), (1, 0, "U", 1.2, 9.9, 10.0)],
            4: [(0, 0, "U", 1.2, 10.2, 10.0), (1, 0, "U", 1.2, 9.8, 10.0)],
        }
        result = _run(_layers(5, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(result["violation_total"], 0, result["violations"])

    def test_merge_that_moves_the_printed_wall_is_flagged(self):
        webs = {
            0: [(0, 0, "U", 1.5, 9.0), (1, 0, "U", 1.5, 11.0)],
            1: [(0, 0, "U", 1.5, 9.1), (1, 0, "U", 1.5, 10.9)],
            # merged printed X = 10.0: web 0's wall moved 0.9, web 1's 0.9
            2: [(0, 0, "U", 1.5, 9.2, 10.0), (1, 0, "U", 1.5, 10.8, 10.0)],
        }
        result = _run(_layers(3, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(_types(result), {VIOLATION_X_STEP: 2})
        self.assertTrue(all(v["merged"] for v in result["violations"]))

    def test_split_mirrors_merge(self):
        webs = {
            0: [(0, 0, "U", 1.2, 10.1, 10.0), (1, 0, "U", 1.2, 9.9, 10.0)],
            1: [(0, 0, "U", 1.2, 10.2, 10.0), (1, 0, "U", 1.2, 9.8, 10.0)],
            # split: each web back at its own X, 1.0 mm from the merged wall
            2: [(0, 0, "U", 1.2, 11.0), (1, 0, "U", 1.2, 9.0)],
        }
        result = _run(_layers(3, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(_types(result), {VIOLATION_X_STEP: 2})


class SocketTests(unittest.TestCase):
    def test_socket_restart_after_end_is_flagged(self):
        webs = {i: [(-2, 0, "S", 9.0, 30.0)] for i in (0, 1, 2, 6, 7)}
        result = _run(_layers(8, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(_types(result), {VIOLATION_SOCKET_RESTART: 1})
        self.assertEqual(result["violations"][0]["gap_layers"], 3)

    def test_socket_is_exempt_from_the_ramp_rules_only(self):
        # Starts at full depth, then jumps 5 mm: neither is flagged for a
        # socket; but an X jump still is.
        webs = {0: [(-2, 0, "S", 4.0, 30.0)], 1: [(-2, 0, "S", 9.0, 30.0)],
                2: [(-2, 0, "S", 9.0, 31.0)]}
        result = _run(_layers(3, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(_types(result), {VIOLATION_X_STEP: 1})

    def test_single_socket_end_is_clean(self):
        webs = {i: [(-2, 0, "S", 9.0, 30.0)] for i in range(5)}
        webs.update({i: [(0, 0, "U", 0.2, 5.0)] for i in range(5, 9)})
        result = _run(_layers(9, webs), exempt={0: EXEMPT_ROOT_SOLID})
        self.assertEqual(result["violation_total"], 0, result["violations"])


class ExemptionTests(unittest.TestCase):
    def test_violation_on_exempt_layer_is_counted_not_listed(self):
        webs = {i: [(0, 1, "U", 1.5, 10.0)] for i in range(5, 9)}
        layers = _layers(9, webs)
        result = _run(layers, exempt={5: EXEMPT_ROOT_SOLID})
        self.assertEqual(result["violation_total"], 0)
        self.assertEqual(result["exempted_counts"][VIOLATION_UNRAMPED_START], 1)
        self.assertEqual(result["exempted_by_band"], {EXEMPT_ROOT_SOLID: 1})

    def test_default_exemptions_cover_root_band_and_trailing_shutdown(self):
        layers = _layers(10, {i: [(0, 0, "U", 0.2, 1.0)] for i in range(2, 7)})
        for i in (7, 8, 9):
            layers[i]["reason"] = "host_tip_region_empty"
        exempt = default_exemptions(layers, root_solid_layers=2)
        self.assertEqual({k for k, v in exempt.items() if v == EXEMPT_ROOT_SOLID},
                         {0, 1, 2})
        self.assertEqual({k for k, v in exempt.items() if v == EXEMPT_FINAL_TOP},
                         {7, 8, 9})

    def test_violation_cap_truncates_the_list_not_the_count(self):
        webs = {}
        for i in range(0, 60, 2):
            webs[i] = [(0, 0, "U", 1.5, 10.0)]
        result = _run(_layers(60, webs), exempt={0: EXEMPT_ROOT_SOLID},
                      violation_cap=5)
        self.assertEqual(len(result["violations"]), 5)
        self.assertTrue(result["violations_truncated"])
        self.assertEqual(result["violation_total"], 29)  # 29 blinks after layer 0


class ReportExtractionTests(unittest.TestCase):
    def test_encode_decode_round_trip(self):
        rows = [[0, -3, "U", 1.5, 10.123], [1, 2, "L", 0.2, -4.5, 3.25],
                [-2, 0, "S", 9.012, 30.0]]
        text = encode_rows(rows)
        self.assertEqual(text, "0,-3,U,1.5,10.123;1,2,L,0.2,-4.5,3.25;-2,0,S,9.012,30.0")
        self.assertEqual(decode_rows(text, "webs"), rows)
        omit = [[0, 1, "U", 4.0, "mitre_clearance"], [4, 0, "U", None, "socket_flank"]]
        self.assertEqual(decode_rows(encode_rows(omit), "omit"), omit)
        self.assertEqual(decode_rows("", "webs"), [])
        self.assertEqual(decode_rows(rows, "webs"), rows)  # lists pass through

    def test_layers_from_report_decodes_string_rows(self):
        report = {"layers": [
            {"layer_index": 3, "print_z": 0.8,
             "extra": {"webs": "0,1,U,1.5,10.0;0,2,U,1.5,16.0",
                       "web_omit": "1,0,L,2.0,thin_section"}}]}
        layer = layers_from_report(report)[0]
        self.assertEqual(layer["webs"], [[0, 1, "U", 1.5, 10.0], [0, 2, "U", 1.5, 16.0]])
        self.assertEqual(layer["web_omit"], [[1, 0, "L", 2.0, "thin_section"]])

    def test_layers_from_report_reads_extra(self):
        report = {"layers": [
            {"layer_index": 3, "print_z": 0.8, "shutdown": False,
             "extra": {"webs": [[0, 1, "U", 1.5, 10.0]], "web_omit": [], "reason": None}},
            {"layer_index": 4, "print_z": 1.0, "shutdown": True,
             "extra": {"reason": "tip_shutdown"}},
        ]}
        layers = layers_from_report(report)
        self.assertEqual(layers[0]["webs"], [[0, 1, "U", 1.5, 10.0]])
        self.assertEqual(layers[1]["webs"], [])
        self.assertEqual(layers[1]["reason"], "tip_shutdown")


class WebRowsTests(unittest.TestCase):
    def test_present_rows_carry_executed_depth_and_station_x(self):
        intent = WebIntent(start=(10.0, 5.0), end=(10.0, 3.5), width_mm=0.4,
                           family_index=2, web_id=(2, -3, "U"))
        rows, omit = web_report_rows([intent], [])
        # phase 2: ``[fi, k, side, depth, x, merged_x, left, right]``
        self.assertEqual(rows, [[2, -3, "U", 1.5, 10.0, None, 9.8, 10.2]])
        self.assertEqual(omit, [])

    def test_merged_intent_credits_every_constituent_with_merged_depth(self):
        merged = WebIntent(
            start=(10.0, 5.0), end=(10.0, 3.8), width_mm=1.0, family_index=0,
            merged_family_indices=(0, 1), web_id=(0, 1, "U"),
            merged_web_ids=((0, 1, "U"), (1, 2, "U")),
            merged_web_xs=(9.8, 10.3),
        )
        rows, _ = web_report_rows([merged], [])
        # the printed slot is the merged one: walls 9.5 / 10.5 on both rows
        self.assertEqual(rows, [[0, 1, "U", 1.2, 9.8, 10.0, 9.5, 10.5],
                                [1, 2, "U", 1.2, 10.3, 10.0, 9.5, 10.5]])

    def test_omission_rows_use_code_and_null_for_nan_x(self):
        omitted = [
            OmittedWeb(x_mm=4.0, reason="skin margin: mitre clearance: x",
                       family_index=1, k=2, web_id=(1, 2, "L")),
            OmittedWeb(x_mm=float("nan"), reason="socket_flank: no channel",
                       family_index=4, k=0, web_id=(4, 0, "U")),
            OmittedWeb(x_mm=7.0, reason="opposing pair: too thin",
                       family_index=6, web_id=(6, 0, "P")),
        ]
        _rows, omit = web_report_rows([], omitted)
        self.assertEqual(omit[0], [1, 2, "L", 4.0, "mitre_clearance"])
        self.assertEqual(omit[1], [4, 0, "U", None, "socket_flank"])
        self.assertEqual(omit[2], [6, 0, "P", 7.0, "opposing_pair"])

    def test_intent_without_identity_falls_back_to_family_and_side(self):
        intent = WebIntent(start=(1.0, 2.0), end=(1.0, 1.0), width_mm=0.4,
                           family_index=3, entry_lower=True)
        rows, _ = web_report_rows([intent], [])
        self.assertEqual(rows, [[3, -1, "L", 1.0, 1.0, None, 0.8, 1.2]])

    def test_rows_are_json_safe(self):
        import json
        intent = WebIntent(start=(1.0, 2.0), end=(1.0, 1.0), width_mm=0.4,
                           web_id=(0, 0, "U"))
        omitted = [OmittedWeb(x_mm=float("nan"), reason="x", web_id=(0, 1, "U"))]
        rows, omit = web_report_rows([intent], omitted)
        json.dumps({"webs": rows, "web_omit": omit}, allow_nan=False)


class OmissionCodeTests(unittest.TestCase):
    def test_known_prefixes(self):
        cases = {
            "chord extent: slot footprint": "chord_extent",
            "skin margin: local thickness": "skin_margin",
            "skin margin: mitre clearance: merged node": "mitre_clearance",
            "skin margin: degenerate web depth": "degenerate_depth",
            "thin section: halo": "thin_section",
            "opposing entry: candidate": "opposing_entry",
            "role yield: lattice": "role_yield",
            "re_entrant_station: the section": "re_entrant_station",
            "spar socket footprint: station": "socket_footprint",
        }
        for reason, code in cases.items():
            self.assertEqual(omission_code(reason), code, reason)

    def test_unknown_reason_gets_a_slug(self):
        self.assertEqual(omission_code("Some Thing: detail"), "some_thing")
        self.assertEqual(omission_code(""), "unspecified")


if __name__ == "__main__":
    unittest.main()


def _w(fi, k, side, depth, left, right, merged_x=None, own_x=None):
    """A phase-2 row: ``[fi, k, side, depth, own_x, merged_x, left, right]``."""
    centre = 0.5 * (left + right)
    return (fi, k, side, depth, centre if own_x is None else own_x, merged_x,
            left, right)


class SocketIdentityTests(unittest.TestCase):
    """Only the CONFIRMED spar socket is restart-policed; an unconfirmed pick
    and a bore are different holes sharing a label."""

    def _gap(self, wid):
        webs = {0: [(wid[0], wid[1], wid[2], 5.0, 7.0)],
                1: [], 2: [(wid[0], wid[1], wid[2], 5.0, 58.0)]}
        return _run(_layers(3, webs), exempt={0: "root_solid"})

    def test_confirmed_spar_restart_is_flagged(self):
        self.assertEqual(_types(self._gap((-2, 0, "S"))),
                         {VIOLATION_SOCKET_RESTART: 1})

    def test_unconfirmed_pick_and_bore_restarts_are_not(self):
        self.assertEqual(_types(self._gap((-2, 0, "N"))), {})
        self.assertEqual(_types(self._gap((-7, 0, "S"))), {})

    def test_the_report_rows_key_an_unconfirmed_pick_apart(self):
        pick = WebIntent(start=(5.0, 3.0), end=(5.0, 0.0), width_mm=1.2,
                         family_index=-2, family_name="spar_socket",
                         web_id=(-2, 0, "S"), spar_confirmed=False)
        spar = WebIntent(start=(9.0, 3.0), end=(9.0, 0.0), width_mm=1.2,
                         family_index=-2, family_name="spar_socket",
                         web_id=(-2, 0, "S"), spar_confirmed=True)
        rows, _ = web_report_rows([pick], [])
        self.assertEqual(rows[0][:3], [-2, 0, "N"])
        rows, _ = web_report_rows([spar], [])
        self.assertEqual(rows[0][:3], [-2, 0, "S"])


class SideWallCheckerTests(unittest.TestCase):
    """(checker correction A): the two SIDE WALLS of the printed slot are
    measured, not the centre."""

    def test_wall_step_within_dx_max_passes(self):
        webs = {i: [_w(0, 0, "U", 0.2 * (i + 1), 10.0 + 0.2 * i, 10.4 + 0.2 * i)]
                for i in range(6)}
        self.assertEqual(_run(_layers(6, webs))["violation_total"], 0)

    def test_one_wall_jumping_is_flagged_even_if_the_centre_barely_moves(self):
        # a merged slot widens to the right: centre moves 0.15 (under dx_max),
        # the right wall moves 0.5
        webs = {0: [_w(0, 0, "U", 0.2, 10.0, 10.4)],
                1: [_w(0, 0, "U", 0.4, 10.0, 10.9, merged_x=10.45)]}
        result = _run(_layers(2, webs))
        self.assertEqual(_types(result), {VIOLATION_X_STEP: 1})
        v = result["violations"][0]
        self.assertAlmostEqual(abs(v["dx"]), 0.5, places=3)
        self.assertTrue(v["merged"])

    def test_phase_one_rows_still_fall_back_to_the_centre(self):
        webs = {0: [(0, 0, "U", 0.2, 10.0)], 1: [(0, 0, "U", 0.4, 10.5)]}
        self.assertEqual(_types(_run(_layers(2, webs))), {VIOLATION_X_STEP: 1})

    def test_side_wall_step_distribution_is_reported_by_merge_involvement(self):
        webs = {
            0: [_w(0, 0, "U", 0.2, 10.0, 10.4), _w(0, 1, "U", 0.2, 20.0, 20.4)],
            1: [_w(0, 0, "U", 0.4, 10.1, 10.5),
                _w(0, 1, "U", 0.4, 20.0, 20.9, merged_x=20.45)],
        }
        steps = _run(_layers(2, webs))["side_wall_steps"]
        self.assertEqual(steps["non_merge"]["count"], 1)
        self.assertAlmostEqual(steps["non_merge"]["max"], 0.1, places=3)
        self.assertEqual(steps["merge"]["count"], 1)
        self.assertAlmostEqual(steps["merge"]["max"], 0.5, places=3)

    def test_rows_with_walls_round_trip_through_the_compact_encoding(self):
        rows = [[0, 1, "U", 1.2, 9.8, 10.0, 9.5, 10.5],
                [2, -3, "L", 1.5, 10.0, None, 9.8, 10.2],
                [3, 0, "U", 1.0, 5.0]]
        self.assertEqual(decode_rows(encode_rows(rows), "webs"), rows)
