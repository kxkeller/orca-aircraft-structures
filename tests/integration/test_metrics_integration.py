"""Integration tests for transform-time metrics against the GENERATED BUNDLE,
run through the fake Orca host (see fake_orca.py's module docstring for scope
and documented divergences). Mirrors test_m1_capability.py's harness.
"""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fake_orca  # noqa: E402

from test_m1_capability import (  # noqa: E402
    FULL_CONFIG,
    M1IntegrationTestCase,
    _same_family_proximity_config,
)


class MetricsEnabledByDefaultTests(M1IntegrationTestCase):
    N_LAYERS = 20
    ROOT_LAYERS = 5

    def test_performance_block_present_with_sane_values(self):
        self.arm()
        obj = self.make_object(n_layers=self.N_LAYERS, width_mm=20.0, depth_mm=20.0)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()

        result = cap.execute(ctx)
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)

        report = self.read_report()
        self.assertIn("performance", report)
        perf = report["performance"]
        self.assertTrue(perf["enabled"])

        mutated_expected = self.N_LAYERS - self.ROOT_LAYERS  # 15

        # layers_measured tracks mutated (transformed) layers, not root
        # layers or layers_seen -- root layers have no phases to measure.
        self.assertEqual(perf["layers_measured"], mutated_expected)
        self.assertEqual(report["mutated_count"], mutated_expected)

        # Every phase's own record count matches layers_measured.
        for phase_name, stats in perf["phases"].items():
            self.assertEqual(stats["count"], mutated_expected, phase_name)
            self.assertIsNotNone(stats["median_ms"], phase_name)
            self.assertIsNotNone(stats["p95_ms"], phase_name)
            self.assertIsNotNone(stats["worst_layer_index"], phase_name)
            self.assertGreaterEqual(stats["total_ms"], 0.0, phase_name)

        # One slot per mutated layer under the default spec (single web).
        self.assertEqual(perf["slots"]["total"], mutated_expected)
        self.assertEqual(perf["slots"]["per_layer_max"], 1)

        # At least one boolean op (the diff subtracting the slot) per layer.
        self.assertGreaterEqual(perf["boolean_ops"]["total"], mutated_expected)
        self.assertGreaterEqual(perf["boolean_ops"]["by_method"]["diff"], mutated_expected)

        # Vertex counts: the fake host's rectangle/notch shapes always
        # report their outline (unlike the airfoil fixture's cambered
        # sections), so both before/after should be fully available here.
        v = perf["vertices"]
        self.assertEqual(v["layers_with_vertex_before"], mutated_expected)
        self.assertEqual(v["layers_with_vertex_after"], mutated_expected)
        self.assertEqual(v["layers_missing_vertex_after"], 0)
        self.assertEqual(v["max_vertex_count_before"], 4)  # plain rectangle
        self.assertEqual(v["max_vertex_count_after"], 8)  # one top notch (U-shape)

        self.assertIsNone(perf["fraction_of_total_slicing_time"])
        self.assertIn("elapsed_ms", perf["fraction_of_total_slicing_time_note"])

        self.assertEqual(len(perf["layers"]), mutated_expected)

    def test_success_message_and_slice_still_succeed_with_metrics_on(self):
        self.arm()
        obj = self.make_object(n_layers=self.N_LAYERS)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()

        result = cap.execute(ctx)

        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        for i in range(self.ROOT_LAYERS, self.N_LAYERS):
            self.assertEqual(obj.layers()[i].make_slices_calls, 1)


class MetricsDisabledTests(M1IntegrationTestCase):
    def test_disabled_via_config_omits_performance_block_entirely(self):
        self.arm()
        obj = self.make_object(n_layers=10)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"metrics_enabled": False})

        result = cap.execute(ctx)
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)

        report = self.read_report()
        self.assertNotIn("performance", report)


class MetricsOnFailurePathTests(M1IntegrationTestCase):
    """Partial metrics must still be written on a fail-closed run (a failing
    run is exactly when that data matters).

    The failure is a genuine per-layer ``TopologyError``, produced by
    ``_same_family_proximity_config`` (a same-family marched-pitch collision
    on the airfoil fixture). A config such as ``skin_margin_mm=0.1`` that
    disconnects the bridge at layer 0 would not do: it is rejected earlier by
    ``StructureSpec.validate(safety_offset_mm=...)`` at config-build time,
    before any layer or metrics object exists, so there would be no partial
    run to inspect. ``skin_margin_mm=1.0`` here clears that precondition. See
    the helper's docstring for why the airfoil fixture, not the coupon.
    """

    def test_partial_metrics_present_on_topology_error(self):
        self.arm()
        obj, _bbox = fake_orca.make_airfoil_object("wing", 5, 0.2)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap_config, _rib_pitch_mm = _same_family_proximity_config()
        cap.get_config = lambda: json.dumps(cap_config)

        result = cap.execute(ctx)
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)

        report = self.read_report()
        self.assertIn("performance", report)
        # Layer 0 itself failed before a LayerMetrics record could be built
        # for it (this fixture has no Z drift, so every layer's pattern --
        # and its collision -- is identical, and layer 0 is walked first),
        # so no layer was successfully measured -- this must be reported as
        # zero, not omitted or guessed.
        self.assertEqual(report["performance"]["layers_measured"], 0)


if __name__ == "__main__":
    unittest.main()
