"""Capability seam: a host region that is empty
right at the mesh's own top Z, where the plugin's own (higher- precision) mesh
section independently agrees there is a knife-edge sliver or nothing at all,
degrades (shutdown, unmutated) instead of raising -- ``capability.py``'s
``_transform``, the ``if not originals:`` fork.

FIXTURE DESIGN: a tapered RECTANGULAR prism (root 20x20 mm, tapering linearly
in WIDTH only to 0.3 mm at the tip, depth constant 20 mm) -- never an aerofoil
outline. A rectangle has 4 convex-hull vertices at every station, well under
``AEROFOIL_MIN_HULL_VERTICES`` (12), so ``resolve_theta_deg`` raises the
non-fatal ``ChordAxisUnavailableError`` (coupon path) at every sampled station
and theta stays 0 for the whole run -- ``_rotate_expolygon`` is therefore never
invoked (its ``if theta_deg:`` guard is skipped entirely), sidestepping
fake_orca's own documented limitation that its ``ExPolygon`` raw-points
constructor only accepts a 4-point AXIS-ALIGNED rectangle (a rotated one, even
at 4 points, is no longer axis-aligned and would still raise
``NotImplementedError`` -- see ``test_m34_global_alignment.py``'s
``RotationDetectedAndAppliedTests`` docstring for the mechanism this
sidesteps). This keeps the fixture fully exercisable end-to-end through
``cap.execute(ctx)``, unlike the capability-seam test (which had to stop at
``_build_mesh_sectioner`` directly).

Two host layers get an EMPTY ``SurfaceCollection`` (no surfaces at all):
the TOP layer (print_z made to land exactly at the mesh's own z_max, so it
is "within one layer_height" of it by construction) and a MID-SPAN layer
(width there is well above the printable floor and far from z_max, so
neither the near-top nor the mesh-agreement condition holds). Every other layer keeps
its ordinary, non-empty rectangular region.
"""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import fake_orca  # noqa: E402

LAYER_HEIGHT_MM = 1.0
N_LAYERS = 6  # print_z 1.0 .. 6.0, mesh z_max = 6.0 mm exactly
ROOT_WIDTH_MM = 20.0
TIP_WIDTH_MM = 0.3  # well below 2 * 0.42 mm wall width (0.84 mm)
DEPTH_MM = 20.0
WALL_WIDTH_MM = 0.42

CAP_CONFIG = {"root_solid_layers": 0, "skin_margin_mm": 1.0}
FULL_CONFIG_OVERRIDES = {
    "layer_height": LAYER_HEIGHT_MM,
    "initial_layer_print_height": LAYER_HEIGHT_MM,
    "outer_wall_line_width": WALL_WIDTH_MM,
    "inner_wall_line_width": WALL_WIDTH_MM,
    "line_width": WALL_WIDTH_MM,
    "nozzle_diameter": 0.4,
    "wall_generator": "classic",
    "wall_loops": 1,
    "spiral_mode": True,
    "top_shell_layers": 0,
    "bottom_shell_layers": 0,
    "sparse_infill_density": 0,
    "enable_support": False,
    "slice_closing_radius": 0.0049,
}


_TAPER_Z_MM = (N_LAYERS - 1) * LAYER_HEIGHT_MM  # 5.0 mm: root up to here

# An ABOVE-floor tip width, to build a fixture that isolates the
# mesh-agreement condition -- an empty host region at the top layer (the
# near-top condition holds, same as the below-floor fixture) whose mesh
# section is NOT below the printable floor (2 * 0.42 mm = 0.84 mm) must still
# raise, because the plugin's own mesh section disagrees that there is nothing
# to build here.
ABOVE_FLOOR_TIP_WIDTH_MM = 5.0


def _width_at_z(z_mm, tip_width_mm=TIP_WIDTH_MM):
    # Piecewise CONSTANT, not linear: root width for the whole root segment,
    # tip width for the whole final layer's segment. A layer's own mesh
    # section is sampled at its SLICE_Z (print_z - 0.5*layer_height, see
    # capability.py's ``_section_plane_for``), not print_z -- a linear taper
    # would put the last layer's slice_z only halfway down the taper,
    # nowhere near the printable floor this fixture needs at the tip. This
    # keeps every layer's own slice_z reading either the full root width or
    # the full tip width, with no fixture-only fudging of the underlying
    # capability.py logic being tested.
    return tip_width_mm if z_mm > _TAPER_Z_MM else ROOT_WIDTH_MM


def _rect_outline_mm(width_mm, depth_mm):
    return [
        (-width_mm / 2.0, -depth_mm / 2.0),
        (width_mm / 2.0, -depth_mm / 2.0),
        (width_mm / 2.0, depth_mm / 2.0),
        (-width_mm / 2.0, depth_mm / 2.0),
    ]


class M39HostTipRegionEmptyTests(unittest.TestCase):
    def setUp(self):
        self._fake_orca, restore = fake_orca.install()
        self.addCleanup(restore)
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.entry_file = os.path.join(self._tmp.name, "aircraft_structures.py")
        self.namespace = fake_orca.exec_bundle(self.entry_file)
        self.report_path = os.path.join(self._tmp.name, "m1", "m1_report.json")

    def _build_object(self, empty_layer_indices, tip_width_mm=TIP_WIDTH_MM):
        # Mesh: a sharp (near-instantaneous) transition at _TAPER_Z_MM from
        # the root width to the tip width -- see ``_width_at_z``'s own
        # docstring for why piecewise-constant, not linear. A tiny
        # (0.001 mm) gap between the two transition stations avoids a
        # literal zero-height wall quad (degenerate/zero-area triangle)
        # while being negligible against every layer height here.
        top_z = N_LAYERS * LAYER_HEIGHT_MM
        stations = [
            (0.0, _rect_outline_mm(ROOT_WIDTH_MM, DEPTH_MM)),
            (_TAPER_Z_MM - 0.001, _rect_outline_mm(ROOT_WIDTH_MM, DEPTH_MM)),
            (_TAPER_Z_MM, _rect_outline_mm(tip_width_mm, DEPTH_MM)),
            (top_z, _rect_outline_mm(tip_width_mm, DEPTH_MM)),
        ]
        mesh = fake_orca.loft_mesh_mm(stations)

        layers = []
        for i in range(N_LAYERS):
            print_z = round(LAYER_HEIGHT_MM * (i + 1), 6)
            slice_z = round(print_z - LAYER_HEIGHT_MM / 2.0, 6)
            if i in empty_layer_indices:
                region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([]))
            else:
                # Match the mesh's OWN cross-section at slice_z (the plane
                # the plugin's mesh sectioner actually samples for this
                # layer), not print_z -- see ``_width_at_z``'s docstring.
                width = _width_at_z(slice_z, tip_width_mm)
                outline_mm = _rect_outline_mm(width, DEPTH_MM)
                scaled_pts = tuple(
                    (fake_orca.to_scaled(x), fake_orca.to_scaled(y))
                    for x, y in outline_mm
                )
                expoly = fake_orca.ExPolygon._wrap(fake_orca._Polygon(pts=scaled_pts))
                surface = fake_orca.Surface(fake_orca.SurfaceType.stInternal, expoly)
                region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([surface]))
            layers.append(fake_orca.Layer(print_z, [region], slice_z=slice_z))

        return fake_orca.PrintObject(
            "tapered_rect", layers, model_object=fake_orca.identity_model_object(mesh)
        )

    def run_capability(self, obj):
        with open(os.path.join(self._tmp.name, "m1_enabled"), "w", encoding="utf-8"):
            pass
        Step = self._fake_orca.slicing.Step
        config = dict(FULL_CONFIG_OVERRIDES)
        ctx = self._fake_orca.slicing.SlicingPipelineContext(
            step=Step.posSlice, obj=obj, config=config, cancelled_fn=None,
        )
        cap = self.namespace["AircraftStructuresM1"]()
        cap.get_config = lambda: __import__("json").dumps(CAP_CONFIG)
        result = cap.execute(ctx)
        return result, cap

    def read_report(self):
        with open(self.report_path, "r", encoding="utf-8") as handle:
            return json.load(handle)

    def test_top_empty_region_degrades_instead_of_raising(self):
        # Top layer index N_LAYERS - 1: print_z == N_LAYERS * LAYER_HEIGHT_MM
        # == the mesh's own z_max exactly.
        top_index = N_LAYERS - 1
        obj = self._build_object(empty_layer_indices={top_index})
        result, cap = self.run_capability(obj)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)

        report = self.read_report()
        records = [lr for lr in report["layers"] if lr["layer_index"] == top_index]
        self.assertEqual(len(records), 1, "expected exactly one layer record for the top layer")
        record = records[0]
        self.assertFalse(record["mutated"])
        self.assertTrue(record["shutdown"])
        kinds = [d["kind"] for d in record.get("degradations", [])]
        self.assertIn("host_tip_region_empty", kinds)

    def test_top_nonempty_below_floor_region_degrades_before_alignment(self):
        top_index = N_LAYERS - 1
        obj = self._build_object(empty_layer_indices=set())
        result, cap = self.run_capability(obj)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)

        report = self.read_report()
        records = [lr for lr in report["layers"] if lr["layer_index"] == top_index]
        self.assertEqual(len(records), 1)
        record = records[0]
        self.assertFalse(record["mutated"])
        self.assertTrue(record["shutdown"])
        kinds = [d["kind"] for d in record.get("degradations", [])]
        self.assertIn("host_tip_below_printable_floor", kinds)

    def test_mid_span_empty_region_still_raises(self):
        # Mid-span layer: width there (~10.1 mm) is far above the printable
        # floor and far from the mesh's own z_max, so NEITHER the near-top
        # nor the mesh-agreement condition holds -- an empty host region here
        # must still raise.
        mid_index = N_LAYERS // 2
        obj = self._build_object(empty_layer_indices={mid_index})
        result, cap = self.run_capability(obj)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("region has no sliced surfaces", result.message)

    def test_top_empty_region_above_floor_tip_still_raises(self):
        # The mesh-agreement condition in isolation. The empty region is at
        # the TOP layer (the near-top condition -- within one layer_height of
        # the mesh's own z_max -- holds, exactly as in the degrading test
        # above), but the tip width here (5 mm) is well
        # ABOVE the printable floor (2 * 0.42 mm = 0.84 mm). The plugin's own
        # mesh section independently disagrees that there is nothing to build,
        # so this must still raise: an empty host region degrades only when it
        # is near the top AND the mesh agrees there is nothing there, not for
        # any empty region near the top.
        self.assertGreater(ABOVE_FLOOR_TIP_WIDTH_MM, 2.0 * WALL_WIDTH_MM)
        top_index = N_LAYERS - 1
        obj = self._build_object(
            empty_layer_indices={top_index}, tip_width_mm=ABOVE_FLOOR_TIP_WIDTH_MM
        )
        result, cap = self.run_capability(obj)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("region has no sliced surfaces", result.message)


if __name__ == "__main__":
    unittest.main()
