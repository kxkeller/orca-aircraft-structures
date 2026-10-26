"""Capability-level (fake host) wiring of ``slice_closing_radius``: the live
value is required by the mesh-alignment guard and fails closed, naming the
key, when missing or unparseable; a single-outer-loop section skips the
closing and records that it did."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import fake_orca  # noqa: E402
from test_m18_mesh_transform import (  # noqa: E402
    FULL_CONFIG,
    LAYER_HEIGHT_MM,
    N_LAYERS,
    M18MeshTransformTestCase,
)


class ClosingRadiusConfigTests(M18MeshTransformTestCase):
    def _aligned_object(self):
        width, depth = 60.0, 10.0
        top_z = LAYER_HEIGHT_MM * N_LAYERS + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(0.0, 0.0, 0.0, width, depth, top_z)
        model = fake_orca.ModelObject(
            volumes=[fake_orca.ModelVolume(mesh)],
            instances=[fake_orca.ModelInstance()],
        )
        return fake_orca.make_rect_object(
            1, N_LAYERS, width, depth, LAYER_HEIGHT_MM,
            min_x_mm=-width / 2.0, min_y_mm=-depth / 2.0,
            model_object=model,
        )

    def _run(self, host_config):
        return self.run_capability(
            self._aligned_object(),
            {"root_solid_layers": 0, "skin_margin_mm": 1.0},
            host_config=host_config,
        )

    def test_single_loop_section_skips_closing_and_records_it(self):
        result, cap = self._run(None)
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success, result.message)
        closing = cap._mesh_alignment["closing"]
        self.assertEqual(closing["radius_mm"], 0.0049)
        self.assertGreater(closing["layers_checked"], 0)
        self.assertEqual(closing["loop_count_changed_layers"], 0)
        self.assertEqual(closing["term_changed_over_tolerance_layers"], 0)

    def test_missing_radius_fails_closed_naming_the_key(self):
        config = dict(FULL_CONFIG)
        del config["slice_closing_radius"]
        result, _cap = self._run(config)
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        self.assertIn("slice_closing_radius", result.message)

    def test_unparseable_radius_fails_closed_naming_the_key(self):
        config = dict(FULL_CONFIG)
        config["slice_closing_radius"] = "not-a-number"
        result, _cap = self._run(config)
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        self.assertIn("slice_closing_radius", result.message)


if __name__ == "__main__":
    unittest.main()
