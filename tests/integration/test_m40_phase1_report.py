"""Against the GENERATED BUNDLE through the fake Orca host: the per-layer web
records and the report-only ``continuity`` block land in the run report, and
neither changes what is cut."""

import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from aircraft_core.continuity import DEFAULT_DX_MAX_MM, decode_rows  # noqa: E402
from test_m1_capability import M1IntegrationTestCase  # noqa: E402

# One web per layer: the fake host's boolean backend cannot subtract a second
# slot from an already-notched section (fake_orca.py, DOCUMENTED SCOPE).
FAMILY_CONFIG = {"root_solid_layers": 2}


class ContinuityBlockInReportTests(M1IntegrationTestCase):
    N_LAYERS = 14

    def _run(self, cap_config):
        self.arm()
        obj = self.make_object(n_layers=self.N_LAYERS, width_mm=40.0, depth_mm=20.0)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(cap_config)
        result = cap.execute(ctx)
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success,
                         getattr(result, "message", ""))
        return self.read_report()

    def test_per_layer_web_rows_and_continuity_block(self):
        report = self._run(FAMILY_CONFIG)
        mutated = [r for r in report["layers"] if r.get("mutated")]
        self.assertTrue(mutated)
        for rec in mutated:
            extra = rec["extra"]
            self.assertIn("webs", extra)
            self.assertIn("web_omit", extra)
            self.assertIsInstance(extra["webs"], str)
            rows = decode_rows(extra["webs"], "webs")
            self.assertEqual(len(rows), extra["slots"])
            for row in rows:
                self.assertIn(row[2], ("U", "L", "S"))
                self.assertGreater(row[3], 0.0)
        block = report["continuity"]
        self.assertNotIn("error", block)
        self.assertEqual(block["mode"], "report_only")
        self.assertEqual(block["root_solid_layers"], 2)
        self.assertGreater(block["ramp_mm"], 0.0)
        # The lateral threshold is measured on the slot side walls
        # and defaults to the derived DEFAULT_DX_MAX_MM
        self.assertAlmostEqual(block["dx_max_mm"], DEFAULT_DX_MAX_MM)
        self.assertIn("ramp_source", block)
        for key in ("violation_counts", "violations", "by_family", "worst_webs",
                    "exempted_counts"):
            self.assertIn(key, block)
        json.dumps(report["continuity"], allow_nan=False)

    def test_checker_is_report_only_the_slice_still_succeeds(self):
        # A fresh bed-to-top coupon starts its webs at full depth on the
        # first structured layer: that is exempt (root band); whatever else
        # the checker finds, the slice result and the cut layers are the same.
        report = self._run(FAMILY_CONFIG)
        self.assertEqual(report["failed_count"], 0)
        self.assertEqual(report["mutated_count"], self.N_LAYERS - 2)


if __name__ == "__main__":
    unittest.main()
