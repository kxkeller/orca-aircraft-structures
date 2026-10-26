"""``tools/replay_host_sections.py`` against a REAL sidecar written by a fake
in-host run (offline replay of recorded host geometry).

Nothing here modifies the code under test -- it only reads and runs it.
"""

import io
import json
import os
import sys
import contextlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import fake_orca  # noqa: E402

from test_m18_mesh_transform import PerLayerAlignmentDiagnosticsTests  # noqa: E402
from tools import replay_host_sections as replay_tool  # noqa: E402


class ReplayHostSectionsTests(PerLayerAlignmentDiagnosticsTests):
    """``_run`` is overridden to opt IN to the sidecar (default False, per
    ``aircraft_core/spec.py``'s ``record_host_geometry_sidecar``), since
    every test in this file reads it."""

    def _run(self, mismatch_layer_index, dy_mm, tip_alignment_skip_enabled=False):
        obj = self._build_object(mismatch_layer_index, dy_mm)
        return self.run_capability(
            obj,
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "tip_alignment_skip_enabled": tip_alignment_skip_enabled,
                "record_host_geometry_sidecar": True,
            },
        ) + (obj,)

    def _sidecar_path(self):
        return replay_tool.sidecar_path_from_report(
            __import__("pathlib").Path(self.report_path)
        )

    def test_replay_reproduces_a_passing_run(self):
        result, _cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        entries = replay_tool.load_sidecar(self._sidecar_path())
        summary = replay_tool.replay(entries)
        self.assertEqual(summary["disagreements"], [])
        self.assertIsNone(summary["first_failure_layer"])
        self.assertEqual(summary["guard_ran_layers"], self.N_LAYERS)

    def test_replay_reproduces_a_failing_run_at_the_same_layer(self):
        target_layer = 3
        result, _cap, _obj = self._run(
            mismatch_layer_index=target_layer, dy_mm=0.03
        )
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        entries = replay_tool.load_sidecar(self._sidecar_path())
        summary = replay_tool.replay(entries)
        self.assertEqual(summary["disagreements"], [])
        self.assertEqual(summary["first_failure_layer"], target_layer)

    def test_cli_main_exit_code_and_output(self):
        result, _cap, _obj = self._run(mismatch_layer_index=2, dy_mm=0.03)
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = replay_tool.main([str(self._sidecar_path())])
        self.assertEqual(code, 0)  # zero DISAGREEMENTS, not "the slice passed"
        self.assertIn("FAILS at layer 2", buf.getvalue())

    def test_cli_report_flag_derives_the_sidecar_path(self):
        result, _cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            code = replay_tool.main(["--report", self.report_path])
        self.assertEqual(code, 0)
        self.assertIn("PASSES", buf.getvalue())

    def test_a_manufactured_disagreement_is_reported_nonzero(self):
        """A sidecar entry whose recorded ``decision`` does not match what
        ``decide_alignment`` would compute from its own recorded
        deltas/tolerances IS reported as a disagreement -- proving the
        comparison is not vacuously always-agree."""
        entries = [{
            "layer_index": 0,
            "guard_ran": True,
            "deltas_mm": {
                "min_x": 0.0, "max_x": 0.0, "min_y": 0.0, "max_y": 0.03,
                "centroid_x": 0.0, "centroid_y": 0.0,
            },
            "effective_tolerance_mm": {k: 0.02 for k in (
                "min_x", "max_x", "min_y", "max_y",
                "centroid_x", "centroid_y")},
            "near_top_extremum": False,
            "tip_alignment_skip_enabled": False,
            # Wrong on purpose: the recorded deltas/tolerances above are a
            # clear bbox-clause refusal, not a pass.
            "decision": {"outcome": "pass"},
        }]
        summary = replay_tool.replay(entries)
        self.assertEqual(len(summary["disagreements"]), 1)
        self.assertEqual(
            summary["disagreements"][0]["replayed"]["outcome"], "raise_bbox")

    def test_a_layer_where_the_guard_never_ran_is_skipped_not_a_disagreement(self):
        entries = [{
            "layer_index": 0,
            "guard_ran": False,
            "deltas_mm": None,
            "effective_tolerance_mm": None,
            "near_top_extremum": None,
            "tip_alignment_skip_enabled": False,
            "decision": None,
        }]
        summary = replay_tool.replay(entries)
        self.assertEqual(summary["guard_ran_layers"], 0)
        self.assertEqual(summary["disagreements"], [])
