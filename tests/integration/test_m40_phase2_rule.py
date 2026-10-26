"""Against the GENERATED BUNDLE through the fake Orca host: the Z-continuity rule
(``z_continuity``) is wired into ``_transform``, reports its own block, leaves
the continuity checker clean, and with the flag off changes nothing about what
is cut."""

import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from aircraft_core.continuity import DEFAULT_DX_MAX_MM, decode_rows  # noqa: E402
from test_m1_capability import M1IntegrationTestCase  # noqa: E402

BASE = {"root_solid_layers": 2}


class RuleInTransformTests(M1IntegrationTestCase):
    N_LAYERS = 14

    def _run(self, cap_config):
        self.arm()
        obj = self.make_object(n_layers=self.N_LAYERS, width_mm=40.0, depth_mm=20.0)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(cap_config)
        result = cap.execute(ctx)
        return result, obj

    def _ok(self, cap_config):
        result, obj = self._run(cap_config)
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success,
                         getattr(result, "message", ""))
        return self.read_report(), obj

    def test_flag_off_reports_no_rule_block_and_the_default_threshold(self):
        report, _ = self._ok(BASE)
        self.assertIsNone(report["z_continuity"])
        self.assertAlmostEqual(report["continuity"]["dx_max_mm"], DEFAULT_DX_MAX_MM)
        self.assertFalse(report["continuity"]["z_continuity_active"])
        for rec in report["layers"]:
            self.assertNotIn("z_ablation", rec.get("extra", {}))

    def test_flag_on_runs_the_rule_and_reports_it(self):
        report, _ = self._ok(dict(BASE, z_continuity=True))
        block = report["z_continuity"]
        self.assertIsNotNone(block)
        self.assertEqual(block["tie_mode"], "independent")
        self.assertAlmostEqual(block["ramp_mm"], 0.2)    # FULL_CONFIG layer_height
        self.assertAlmostEqual(block["dx_max_mm"], DEFAULT_DX_MAX_MM)
        self.assertEqual(block["counters"]["layers_committed"], self.N_LAYERS - 2)
        self.assertTrue(report["continuity"]["z_continuity_active"])
        json.dumps(block, allow_nan=False)

    def test_flag_on_cuts_the_same_constant_section_as_flag_off(self):
        # A constant section needs no ramp (the first structured layer is
        # exempt and the depth never changes): both runs cut identical webs.
        off, _ = self._ok(BASE)
        self.setUp()                       # fresh fake host and report dir
        on, _ = self._ok(dict(BASE, z_continuity=True))
        self.assertEqual(off["mutated_count"], on["mutated_count"])
        for a, b in zip(off["layers"], on["layers"]):
            if a.get("mutated"):
                self.assertEqual(a["extra"]["webs"], b["extra"]["webs"])
                self.assertEqual(a["extra"]["slots"], b["extra"]["slots"])

    def test_flag_on_keeps_the_continuity_checker_clean(self):
        report, _ = self._ok(dict(BASE, z_continuity=True))
        block = report["continuity"]
        self.assertEqual(block["violation_total"], 0, block["violations"])

    def test_the_tie_mode_is_configurable_and_validated(self):
        report, _ = self._ok(dict(BASE, z_continuity=True,
                                  z_continuity_tie_mode="joint"))
        self.assertEqual(report["z_continuity"]["tie_mode"], "joint")
        result, _ = self._run(dict(BASE, z_continuity=True,
                                   z_continuity_tie_mode="sideways"))
        self.assertNotEqual(result.status, self._fake_orca.PluginResult.Success)

    def test_an_explicit_threshold_reaches_the_rule_and_the_checker(self):
        report, _ = self._ok(dict(BASE, z_continuity=True,
                                  z_continuity_dx_max_mm=0.5))
        self.assertAlmostEqual(report["z_continuity"]["dx_max_mm"], 0.5)
        self.assertAlmostEqual(report["continuity"]["dx_max_mm"], 0.5)

    def test_a_rescue_from_the_validator_loop_is_applied_and_reported(self):
        """The ablation branch of ``_transform``: the fake host cannot make a
        real island split, so the rule's validator step is patched to return
        a rescue that ENDs the layer's only web; everything the capability
        does with a rescue (drop the slot, record the omission and the
        degradation, commit the FINAL intents) is then exercised."""
        ns = self.namespace
        Rule, Rescue, Omitted = (
            ns["ZContinuityRule"], ns["WebAblationRescue"], ns["OmittedWeb"])
        original = Rule.validate_with_ablation

        def fake(rule, ops, validator, originals, replacement, intents, slots,
                 spec, index, print_z, slot_labels, family_indices):
            if index != 5:
                return original(rule, ops, validator, originals, replacement,
                                intents, slots, spec, index, print_z,
                                slot_labels, family_indices)
            wid = intents[0].web_id
            return Rescue(
                layer_index=index, print_z=print_z, removed_indices=(0,),
                kept_indices=(), ended_ids=(wid,),
                omitted=(Omitted(x_mm=intents[0].start[0],
                                 reason="z_ablation: test", family_index=wid[0],
                                 family_name="x", k=wid[1], web_id=wid),),
                tried=1, elapsed_ms=0.0, reason="z_ablation")

        Rule.validate_with_ablation = fake
        try:
            report, _ = self._ok(dict(BASE, z_continuity=True))
        finally:
            Rule.validate_with_ablation = original
        layer5 = report["layers"][5]
        self.assertEqual(layer5["extra"]["z_ablation"]["tried"], 1)
        self.assertEqual(layer5["extra"]["slots"], 0)
        self.assertTrue(any(d["kind"] == "z_ablation" for d in layer5["degradations"]))
        self.assertEqual(decode_rows(layer5["extra"]["webs"], "webs"), [])
        self.assertIn("z_ablation", layer5["extra"]["web_omit"])
        # the ended web restarts by ramp on the next layer, from the rule
        rows6 = decode_rows(report["layers"][6]["extra"]["webs"], "webs")
        self.assertEqual(len(rows6), 1)
        self.assertLessEqual(rows6[0][3], 0.2 + 1e-6)
        self.assertEqual(report["z_continuity"]["counters"]["layers_committed"],
                         self.N_LAYERS - 2)
        self.assertEqual(report["continuity"]["violation_total"], 0,
                         report["continuity"]["violations"])


if __name__ == "__main__":
    unittest.main()
