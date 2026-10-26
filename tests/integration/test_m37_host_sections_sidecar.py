"""The ``m1_host_sections.json.gz`` sidecar.

Reuses ``PerLayerAlignmentDiagnosticsTests``'s fixture (constant-section
cambered mesh, a precisely controlled Y-shift confined to one layer) from
``test_m18_mesh_transform.py`` rather than rebuilding it -- the sidecar is
recorded from the SAME per-layer state (``self._last_loops``, the host's own
region surfaces, ``_check_mesh_alignment``'s deltas/decision) that fixture
already exercises byte-precisely.

Nothing here modifies the code under test -- it only reads it.
"""

import gzip
import json
import os
import sys
import unittest.mock as mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import fake_orca  # noqa: E402

from test_m18_mesh_transform import (  # noqa: E402
    LAYER_HEIGHT_MM,
    PerLayerAlignmentDiagnosticsTests,
)


def _sidecar_path(test_case):
    return os.path.join(
        os.path.dirname(test_case.report_path), "m1_host_sections.json.gz"
    )


def _read_sidecar_payload(test_case):
    with gzip.open(_sidecar_path(test_case), "rb") as handle:
        return json.loads(handle.read().decode("utf-8"))


def _read_sidecar(test_case):
    """The per-layer entry list. The sidecar's own top level is
    ``{"run_id":..., "layers": [...]}`` (so the file's run_id can be checked
    against the report's ``host_sections_sidecar.run_id``); most tests only
    need the layer list, which is what this helper returns."""
    return _read_sidecar_payload(test_case)["layers"]


def _read_report(test_case):
    with open(test_case.report_path, "r", encoding="utf-8") as handle:
        return json.load(handle)


class HostSectionsSidecarTests(PerLayerAlignmentDiagnosticsTests):
    """Inherits ``_build_object``/``N_LAYERS``/``TOP_Z`` from the fixture;
    ``_run`` is overridden to opt IN to the sidecar (default False, per
    ``aircraft_core/spec.py``'s ``record_host_geometry_sidecar``), since
    every test in this file is specifically about the sidecar."""

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

    def test_sidecar_present_on_success_default_flag(self):
        """Opted in explicitly (see class docstring: the field's own
        default is False): a clean run writes a sidecar with one
        contiguous, guard-ran entry per layer."""
        result, _cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message
        )
        self.assertTrue(os.path.exists(_sidecar_path(self)))
        entries = _read_sidecar(self)
        self.assertEqual(len(entries), self.N_LAYERS)
        self.assertEqual(
            sorted(e["layer_index"] for e in entries), list(range(self.N_LAYERS))
        )
        for e in entries:
            self.assertTrue(e["guard_ran"])
            self.assertIsNotNone(e["deltas_mm"])
            self.assertIsNotNone(e["decision"])
            self.assertEqual(e["decision"]["outcome"], "pass")
            self.assertGreater(len(e["host_regions"]), 0)
            outer = e["host_regions"][0]["surfaces"][0]["outer_pts_mm"]
            self.assertGreater(len(outer), 0)
            self.assertGreater(len(e["mesh_section_loops"]), 0)

    def test_sidecar_present_on_failure_and_matches_the_failing_layer(self):
        """The sidecar is written on the ``TopologyError`` path too (not
        only on success) -- and the failing layer's own entry carries its
        OWN shifted host contour and a ``raise_bbox`` decision, not a
        neighbouring layer's."""
        mismatch_layer_index = 3
        dy_mm = 0.03
        result, _cap, _obj = self._run(
            mismatch_layer_index=mismatch_layer_index, dy_mm=dy_mm
        )
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        self.assertTrue(os.path.exists(_sidecar_path(self)))
        entries = _read_sidecar(self)
        by_index = {e["layer_index"]: e for e in entries}
        # Layers before the mismatch succeeded and guard_ran; the mismatch
        # layer itself raised, but ITS OWN sidecar entry was still written
        # (guard_ran True: the guard reached its decision before the
        # caller's raise propagated).
        failing_entry = by_index[mismatch_layer_index]
        self.assertTrue(failing_entry["guard_ran"])
        self.assertEqual(failing_entry["decision"]["outcome"], "raise_bbox")
        self.assertGreater(failing_entry["deltas_mm"]["min_y"], 0.02)

        passing_entry = by_index[mismatch_layer_index - 1]
        self.assertTrue(passing_entry["guard_ran"])
        self.assertEqual(passing_entry["decision"]["outcome"], "pass")

        # The two entries' host contours differ by dy_mm on min_y -- the
        # failing layer's own geometry, not the passing layer's.
        def _min_y(entry):
            pts = entry["host_regions"][0]["surfaces"][0]["outer_pts_mm"]
            return min(p[1] for p in pts)

        self.assertAlmostEqual(
            _min_y(failing_entry) - _min_y(passing_entry), dy_mm, places=3
        )

    def test_sidecar_resets_between_runs_on_the_same_instance(self):
        """A second run on the same (host-reused) capability instance must
        not accumulate the first run's entries under its own report."""
        result_a, cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        self.assertEqual(result_a.status, self._fake_orca.PluginResult.Success)
        entries_a = _read_sidecar(self)
        self.assertEqual(len(entries_a), self.N_LAYERS)

        # Reuse the SAME capability instance (mirrors
        # test_geometry_at_failure_does_not_carry_across_runs's own
        # cross-run reuse pattern) for a run with a mismatch on layer 1.
        obj_b = self._build_object(mismatch_layer_index=1, dy_mm=0.03)
        ctx_b = self.make_ctx(obj_b)
        cap.get_config = lambda: json.dumps({
            "root_solid_layers": 0, "skin_margin_mm": 1.0,
            "record_host_geometry_sidecar": True,
        })
        result_b = cap.execute(ctx_b)
        self.assertEqual(result_b.status, self._fake_orca.PluginResult.FatalError)
        entries_b = _read_sidecar(self)
        # Run B fails at layer 1 (bbox clause), so only layers 0-1 are ever
        # reached -- NOT run A's 5 entries, and not run A's 5 plus run B's
        # 2 accumulated together. Both facts (right count, right content)
        # are needed to rule out "reset" vs. "coincidentally overwritten
        # with the same length".
        self.assertEqual(len(entries_b), 2)
        self.assertEqual([e["layer_index"] for e in entries_b], [0, 1])
        self.assertEqual(entries_b[1]["decision"]["outcome"], "raise_bbox")

    def test_flag_off_writes_no_sidecar(self):
        obj = self._build_object(mismatch_layer_index=-1, dy_mm=0.0)
        result, _cap = self.run_capability(
            obj,
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "record_host_geometry_sidecar": False,
            },
        )
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertFalse(os.path.exists(_sidecar_path(self)))
        # The ordinary report is unaffected by the flag.
        report = _read_report(self)
        self.assertIn("elapsed_ms", report)
        sidecar_info = report["host_sections_sidecar"]
        self.assertFalse(sidecar_info["enabled"])
        self.assertFalse(sidecar_info["written"])
        self.assertIsNone(sidecar_info["error"])
        self.assertFalse(sidecar_info["stale_archived"]["archived"])
        self.assertIsNone(sidecar_info["stale_archived"]["error"])

    def test_report_carries_host_sections_sidecar_info_on_success(self):
        result, _cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        report = _read_report(self)
        info = report["host_sections_sidecar"]
        self.assertTrue(info["enabled"])
        self.assertTrue(info["written"])
        self.assertIsNone(info["error"])
        self.assertTrue(info["path"].endswith("m1_host_sections.json.gz"))
        self.assertIsNotNone(info["run_id"])

    def test_run_id_matches_between_report_and_sidecar(self):
        result, _cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        report = _read_report(self)
        sidecar_payload = _read_sidecar_payload(self)
        self.assertIsNotNone(report["host_sections_sidecar"]["run_id"])
        self.assertEqual(
            report["host_sections_sidecar"]["run_id"],
            sidecar_payload["run_id"],
        )

    def test_run_id_differs_across_runs_on_the_same_instance(self):
        result_a, cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        self.assertEqual(result_a.status, self._fake_orca.PluginResult.Success)
        run_id_a = _read_report(self)["host_sections_sidecar"]["run_id"]

        obj_b = self._build_object(mismatch_layer_index=-1, dy_mm=0.0)
        ctx_b = self.make_ctx(obj_b)
        cap.get_config = lambda: json.dumps({
            "root_solid_layers": 0, "skin_margin_mm": 1.0,
            "record_host_geometry_sidecar": True,
        })
        result_b = cap.execute(ctx_b)
        self.assertEqual(result_b.status, self._fake_orca.PluginResult.Success)
        run_id_b = _read_report(self)["host_sections_sidecar"]["run_id"]

        self.assertNotEqual(run_id_a, run_id_b)

    def test_gzip_failure_is_reported_as_written_false_with_error(self):
        with mock.patch("gzip.open", side_effect=OSError("disk full")):
            result, _cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        # Diagnostics must never fail the slice itself.
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertFalse(os.path.exists(_sidecar_path(self)))
        report = _read_report(self)
        info = report["host_sections_sidecar"]
        self.assertTrue(info["enabled"])
        self.assertFalse(info["written"])
        self.assertIsNotNone(info["error"])
        self.assertIn("disk full", info["error"])

    def test_stale_sidecar_is_archived_aside_when_flag_turned_off(self):
        # Run A: flag on, sidecar written.
        result_a, cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        self.assertEqual(result_a.status, self._fake_orca.PluginResult.Success)
        self.assertTrue(os.path.exists(_sidecar_path(self)))
        run_a_payload = _read_sidecar_payload(self)

        # Run B: same instance, flag off -- the stale file from run A must not
        # survive under run B's own report, but it is renamed aside rather than
        # destroyed.
        obj_b = self._build_object(mismatch_layer_index=-1, dy_mm=0.0)
        ctx_b = self.make_ctx(obj_b)
        cap.get_config = lambda: json.dumps({
            "root_solid_layers": 0, "skin_margin_mm": 1.0,
            "record_host_geometry_sidecar": False,
        })
        result_b = cap.execute(ctx_b)
        self.assertEqual(result_b.status, self._fake_orca.PluginResult.Success)

        # Nothing remains at the canonical path...
        self.assertFalse(os.path.exists(_sidecar_path(self)))
        report_b = _read_report(self)
        info_b = report_b["host_sections_sidecar"]
        self.assertFalse(info_b["enabled"])
        self.assertFalse(info_b["written"])
        stale_archived = info_b["stale_archived"]
        self.assertTrue(stale_archived["archived"])
        self.assertIsNone(stale_archived["error"])
        archived_path = stale_archived["archived_path"]
        self.assertIsNotNone(archived_path)

        # ...but run A's own recorded geometry is NOT gone: it survives,
        # byte-for-byte, under the archive name. An assertion that only
        # checked the canonical path is empty would also pass for a
        # destructive (``os.remove``) code -- this is the assertion that
        # actually distinguishes rename-aside from delete.
        self.assertTrue(os.path.exists(archived_path))
        with gzip.open(archived_path, "rb") as handle:
            archived_payload = json.loads(handle.read().decode("utf-8"))
        self.assertEqual(archived_payload, run_a_payload)
