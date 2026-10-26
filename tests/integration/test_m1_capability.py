"""Integration tests for the GENERATED BUNDLE, run against a fake Orca host.

WHY THIS EXISTS: OrcaSlicer's CLI does not initialize the plugin subsystem, so
nothing can drive the real host automatically. This module execs the
checked-in, generated ``plugin/aircraft_structures/aircraft_structures.py``
bundle (never ``capability.py`` directly — the bundle is what ships) against
``fake_orca.py``'s fake ``orca`` module, so the capability's own control flow —
arming, step gating, layer walk, root-layer skip, slot generation, validation,
fail-closed error paths, cancellation, diagnostics — runs under plain
``unittest``.

See ``fake_orca.py``'s module docstring for the fake host's documented
geometry scope (axis-aligned rectangles and single top-notch U-shapes only)
and its documented divergences from real Orca semantics. Everything
downstream of ``slices.set()`` (perimeter generation, Spiral Vase, the
physical print) is still out of reach of this harness and remains verified
only by a human-run GUI slice.

Nothing here modifies the code under test — it only reads it.
"""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "unit"))

import fake_orca  # noqa: E402
# The bundle is what runs (see module docstring), but the EXPECTED notch
# geometry below is easiest to state honestly by reading the same constant
# the generator itself uses, rather than hard-coding its numeric value a
# second time here.
from aircraft_core.generator import FULL_DEPTH_MARGIN_HEADROOM_MM  # noqa: E402
from aircraft_core.frame import LayerFrame  # noqa: E402

# Reused from test_airfoil_capability.py rather than duplicated: an
# independent GeometryOps adapter over fake_orca's orca.host primitives,
# used ONLY to derive a same-family "just too close" rib_pitch_mm honestly
# (never to drive the capability itself).
from test_airfoil_capability import (  # noqa: E402
    _IndependentHostOps,
    OUTER_WALL_MM as AIRFOIL_OUTER_WALL_MM,
)
# ChannelTooSmallForKeyholeLatchTests only: a plain rectangular coupon,
# exact-arithmetic stub (never used to drive the capability itself).
from stub_ops import StubOps, Rect  # noqa: E402


FULL_CONFIG = {
    "layer_height": 0.2,
    "initial_layer_print_height": 0.2,
    "outer_wall_line_width": 0.42,
    "inner_wall_line_width": 0.42,
    "line_width": 0.42,
    "nozzle_diameter": 0.4,
    "wall_generator": "classic",
    "wall_loops": 1,
    "spiral_mode": True,
    "top_shell_layers": 0,
    # Mirrors the real preset, which pins this to match root_solid_layers.
    # Spiral perimeters start at layer.id() >= bottom_shell_layers, so a
    # smaller value would put the first slotted layer over hollow rings — the
    # capability refuses that, and so should this fixture rather than papering
    # over it.
    "bottom_shell_layers": 5,
    "sparse_infill_density": 0,
    "enable_support": False,
    "slice_closing_radius": 0.0049,
}


def _same_family_proximity_config(gap_ratio=0.5, anchor_position=0.5, target_gap_mm=0.3):
    """A same-family, marched-pitch config whose adjacent stations are
    ``target_gap_mm`` apart -- close enough to fail TopologyValidator's
    collision check, reached through the REAL capability control flow.

    Built on the NACA 2412 airfoil fixture, not the rectangular coupon:
    fake_orca's rectangle kernel (``_Notched``) supports at most ONE notch
    per section (``ExPolygon.diff_ex``'s "subject must be a plain rectangle
    or polygon" branch, tests/integration/fake_orca.py), so a marching
    family -- which always produces more than one candidate station on a
    section this wide -- cannot be represented on the coupon at all. The
    airfoil's polygon kernel (``_PolygonNotched``/``_PolygonMultiNotched``)
    supports any number of non-overlapping notches (see
    ``_add_polygon_notch``), which is what
    ``tests/integration/test_airfoil_capability.py``'s
    ``SameFamilyProximityIntegrationTests`` already relies on for the
    identical scenario -- this reuses that derivation, not a second one.

    Returns ``(cap_config_dict, rib_pitch_mm)``.
    """
    ops = _IndependentHostOps()
    probe_obj, _bbox = fake_orca.make_airfoil_object("probe", 1, 0.2)
    original = probe_obj.layers()[0].regions()[0].slices.surfaces[0].expolygon
    frame = LayerFrame.from_polygon(ops, original)
    width_mm = gap_ratio * AIRFOIL_OUTER_WALL_MM
    x_anchor = frame.web_x(anchor_position)
    x_next = x_anchor + width_mm / 2.0 + width_mm / 2.0 + target_gap_mm
    rib_pitch_mm = x_next - x_anchor
    return (
        {
            "root_solid_layers": 0,
            "skin_margin_mm": 1.0,
            "gap_ratio": gap_ratio,
            "web_position": anchor_position,
            "rib_pitch_mm": rib_pitch_mm,
        },
        rib_pitch_mm,
    )


class M1IntegrationTestCase(unittest.TestCase):
    """Common fixture plumbing: fresh fake orca + tmp plugin dir per test."""

    def setUp(self):
        self._fake_orca, restore = fake_orca.install()
        self.addCleanup(restore)

        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.plugin_dir = self._tmp.name
        self.fake_entry_file = os.path.join(self.plugin_dir, "aircraft_structures.py")

        self.namespace = fake_orca.exec_bundle(self.fake_entry_file)
        self.report_path = os.path.join(self.plugin_dir, "m1", "m1_report.json")

    # -- helpers ----------------------------------------------------------

    def arm(self):
        with open(os.path.join(self.plugin_dir, "m1_enabled"), "w", encoding="utf-8"):
            pass

    def new_capability(self):
        cls = self.namespace["AircraftStructuresM1"]
        return cls()

    def make_object(self, n_layers, width_mm=20.0, depth_mm=20.0, layer_height_mm=0.2, object_id="coupon"):
        return fake_orca.make_rect_object(object_id, n_layers, width_mm, depth_mm, layer_height_mm)

    def make_ctx(self, obj, step=None, config=None, cancelled_fn=None):
        Step = self._fake_orca.slicing.Step
        return self._fake_orca.slicing.SlicingPipelineContext(
            step=step if step is not None else Step.posSlice,
            obj=obj,
            config=dict(FULL_CONFIG if config is None else config),
            cancelled_fn=cancelled_fn,
        )

    def read_report(self):
        with open(self.report_path, "r", encoding="utf-8") as handle:
            return json.load(handle)

    def surfaces_of(self, layer):
        return layer.regions()[0].slices.surfaces

    def assert_layer_untouched(self, layer, expected_area_mm2):
        self.assertEqual(layer.make_slices_calls, 0)
        surfaces = self.surfaces_of(layer)
        self.assertEqual(len(surfaces), 1)
        desc = fake_orca.describe(surfaces[0].expolygon)
        self.assertEqual(desc["kind"], "rect")
        area_mm2 = surfaces[0].expolygon.area() * (fake_orca.SCALING_FACTOR ** 2)
        self.assertAlmostEqual(area_mm2, expected_area_mm2, places=6)


# ---------------------------------------------------------------------------
# 1. Not armed
# ---------------------------------------------------------------------------


class NotArmedTests(M1IntegrationTestCase):
    def test_not_armed_is_success_and_mutates_nothing(self):
        obj = self.make_object(n_layers=10)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success)
        self.assertIn("m1_enabled", result.message)
        self.assertIn("not armed", result.message)

        for layer in obj.layers():
            self.assert_layer_untouched(layer, expected_area_mm2=400.0)

        self.assertFalse(os.path.exists(self.report_path))


# ---------------------------------------------------------------------------
# 2. Armed happy path
# ---------------------------------------------------------------------------


class ArmedHappyPathTests(M1IntegrationTestCase):
    N_LAYERS = 300
    ROOT_LAYERS = 5  # DEFAULT_SPEC["root_solid_layers"]
    GAP_RATIO = 0.5  # DEFAULT_SPEC["gap_ratio"]
    OUTER_WALL_MM = 0.42
    SKIN_MARGIN_MM = 1.0  # DEFAULT_SPEC["skin_margin_mm"]
    WEB_POSITION = 0.5

    def test_root_layers_untouched_and_body_layers_get_one_slot_each(self):
        self.arm()
        obj = self.make_object(n_layers=self.N_LAYERS, width_mm=20.0, depth_mm=20.0)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success)
        self.assertIn("{} layers seen".format(self.N_LAYERS), result.message)
        mutated_expected = self.N_LAYERS - self.ROOT_LAYERS
        self.assertIn("{} slotted".format(mutated_expected), result.message)
        self.assertIn("{} left solid".format(self.ROOT_LAYERS), result.message)

        layers = obj.layers()

        for i in range(self.ROOT_LAYERS):
            self.assert_layer_untouched(layers[i], expected_area_mm2=400.0)

        # make_rect_object's default (auto-matching-mesh) coupon is centred
        # at (0, 0) -- capability.py's XY-centring fix -- so a 20x20 coupon
        # spans [-10, 10] x [-10, 10], not [0, 20] x [0, 20]. web_x_mm and
        # the Y bounds below are computed relative to that rect's own
        # min corner rather than assuming it starts at (0, 0).
        rect_min_x_mm, rect_min_y_mm = -10.0, -10.0
        slot_width_mm = self.GAP_RATIO * self.OUTER_WALL_MM
        web_x_mm = rect_min_x_mm + 20.0 * 0.5  # width 20mm, web_position 0.5
        expected_notch = (
            fake_orca.to_scaled(web_x_mm - slot_width_mm / 2.0),
            # Full-depth far end: skin_margin_mm plus the derived placement
            # headroom -- see FULL_DEPTH_MARGIN_HEADROOM_MM's docstring in
            # aircraft_core/generator.py.
            fake_orca.to_scaled(
                rect_min_y_mm + self.SKIN_MARGIN_MM + FULL_DEPTH_MARGIN_HEADROOM_MM
            ),
            fake_orca.to_scaled(web_x_mm + slot_width_mm / 2.0),
            fake_orca.to_scaled(rect_min_y_mm + 20.0),
        )

        for i in range(self.ROOT_LAYERS, self.N_LAYERS):
            layer = layers[i]
            self.assertEqual(layer.make_slices_calls, 1, "layer {} make_slices calls".format(i))
            surfaces = self.surfaces_of(layer)
            self.assertEqual(len(surfaces), 1, "layer {} surface count".format(i))
            surface = surfaces[0]
            self.assertEqual(surface.surface_type, self._fake_orca.host.SurfaceType.stInternal)

            expoly = surface.expolygon
            self.assertEqual(expoly.num_contours(), 1, "layer {} holes".format(i))
            self.assertTrue(expoly.is_valid())

            desc = fake_orca.describe(expoly)
            self.assertEqual(desc["kind"], "notched", "layer {} shape".format(i))
            self.assertEqual(desc["notch"], expected_notch, "layer {} notch bounds".format(i))


# ---------------------------------------------------------------------------
# 3. Fail closed on disconnection
# ---------------------------------------------------------------------------


class FailClosedDisconnectionTests(M1IntegrationTestCase):
    """skin_margin_mm well below 2x the safety offset (outer_wall_line_width)
    starves the bridge below the slot.

    ``StructureSpec.validate(safety_offset_mm=...)`` runs at config-build
    time, in ``capability.py``'s ``_build_spec`` -- BEFORE any layer is
    walked -- so this misconfiguration (0.1 mm margin against a 0.42 mm
    wall) is a ``SpecError`` naming the margin/wall-width pairing directly,
    surfacing as ``PluginResult.FatalError`` with a "configuration invalid"
    message and no layer/Z (there is no layer to name -- the failure
    precedes the layer walk). That gives a clearer message, earlier, naming
    the actual cause rather than a disconnected-island symptom several
    layers later.
    """

    SKIN_MARGIN_MM = 0.1
    OUTER_WALL_MM = 0.42

    def test_thin_bridge_raises_fatal_error_with_layer_and_z(self):
        self.arm()
        obj = self.make_object(n_layers=5, width_mm=20.0, depth_mm=20.0, layer_height_mm=0.2)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {"root_solid_layers": 0, "skin_margin_mm": self.SKIN_MARGIN_MM}
        )

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)

        expected_message = (
            "aircraft structures configuration invalid: "
            "skin_margin_mm={!r} does not exceed 2 * safety_offset_mm={!r} "
            "({!r} mm minimum wall).".format(
                self.SKIN_MARGIN_MM, self.OUTER_WALL_MM, 2.0 * self.OUTER_WALL_MM
            )
        )
        self.assertTrue(
            result.message.startswith(expected_message),
            "expected the message to start with {!r}, got {!r}".format(
                expected_message, result.message
            ),
        )

        # Fail closed: the guard fires at config-build time, before the
        # layer walk starts at all, so nothing is mutated and no report is
        # written (there is no per-layer diagnostics data to write yet).
        layer0 = obj.layers()[0]
        self.assertEqual(layer0.make_slices_calls, 0)
        surfaces = self.surfaces_of(layer0)
        self.assertEqual(len(surfaces), 1)
        self.assertEqual(fake_orca.describe(surfaces[0].expolygon)["kind"], "rect")

        for layer in obj.layers()[1:]:
            self.assertEqual(layer.make_slices_calls, 0)
            self.assertEqual(fake_orca.describe(self.surfaces_of(layer)[0].expolygon)["kind"], "rect")

        self.assertFalse(os.path.exists(self.report_path))


# ---------------------------------------------------------------------------
# 4. Fail closed on bad config
# ---------------------------------------------------------------------------


class FailClosedBadConfigTests(M1IntegrationTestCase):
    def test_both_gap_modes_set_is_fatal_and_mutates_nothing(self):
        self.arm()
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"slot_gap_mm": 1.0, "gap_ratio": 0.5})

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("aircraft structures configuration invalid", result.message)
        self.assertIn("exactly one gap mode", result.message)

        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 0)
            self.assertEqual(fake_orca.describe(self.surfaces_of(layer)[0].expolygon)["kind"], "rect")

    def test_malformed_json_is_fatal_and_mutates_nothing(self):
        self.arm()
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: "{not valid json"

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("aircraft structures configuration invalid", result.message)
        self.assertIn("not valid JSON", result.message)

        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 0)
            self.assertEqual(fake_orca.describe(self.surfaces_of(layer)[0].expolygon)["kind"], "rect")

    def test_family_with_unrecognized_key_is_fatal_and_mutates_nothing(self):
        # A stored "families" entry with an unknown key is a configuration
        # mistake, not something to guess past -- capability._build_families
        # must refuse it loudly rather than silently ignoring the key.
        self.arm()
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {"families": [{"name": "typo", "rib_pitc_mm": 5.0}]}
        )

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("aircraft structures configuration invalid", result.message)
        self.assertIn("unrecognized key", result.message)

        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 0)
            self.assertEqual(fake_orca.describe(self.surfaces_of(layer)[0].expolygon)["kind"], "rect")

    def test_family_web_depth_breach_is_fatal_naming_layer_and_family(self):
        # A depth-limited family whose requested depth leaves less than
        # skin_margin_mm before the far surface must fail closed, naming the
        # layer, Z and family -- not silently deepen toward (or past) the far
        # surface. The rect coupon is 20mm deep (``make_object`` default); a
        # 19.5mm requested depth against a 1.0mm skin_margin_mm leaves only
        # 0.5mm.
        self.arm()
        obj = self.make_object(n_layers=1)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "families": [
                    {"name": "too deep", "web_depth_mm": 19.5, "gap_ratio": 0.5}
                ],
            }
        )

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("Aircraft structure invalid at layer 0", result.message)
        self.assertIn("family 0", result.message)
        self.assertIn("too deep", result.message)

        layer0 = obj.layers()[0]
        self.assertEqual(layer0.make_slices_calls, 0)
        self.assertEqual(fake_orca.describe(self.surfaces_of(layer0)[0].expolygon)["kind"], "rect")


# ---------------------------------------------------------------------------
# 4b. Unknown top-level config key: warn loudly, not fail closed
# ---------------------------------------------------------------------------
#
# A top-level cap_config key is round-tripped by the host itself (Config tab
# save, self.get_config()/self.set_config()) -- unlike a "families" entry,
# which is unambiguously this plugin's own user-authored data (see
# capability.py's _unknown_top_level_key_warning docstring for the full
# reasoning and what evidence was and was not available). So an unknown
# top-level key is surfaced loudly (host-log line + diagnostics JSON) but
# does NOT fail the slice, unlike the sibling family-key check above.


class UnknownTopLevelConfigKeyTests(M1IntegrationTestCase):
    def test_near_miss_key_warns_with_did_you_mean_and_still_slices(self):
        self.arm()
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"sking_margin_mm": 1.0})

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success)
        self.assertIn("WARNING", result.message)
        self.assertIn("sking_margin_mm", result.message)
        self.assertIn("did you mean", result.message)
        self.assertIn("skin_margin_mm", result.message)
        self.assertIn("config.json", result.message)

        # Not applied (the default is used instead), and never a reason to
        # skip mutation: every body layer still gets its default single web.
        report = self.read_report()
        self.assertEqual(report["unknown_top_level_config_keys"], ["sking_margin_mm"])
        for i in range(5, 8):  # DEFAULT_SPEC["root_solid_layers"] == 5
            layer = obj.layers()[i]
            self.assertEqual(layer.make_slices_calls, 1, "layer {} make_slices calls".format(i))

    def test_wrong_scope_key_at_top_level_warns_and_still_slices(self):
        # entry_lower is a real key -- just a family-scoped one -- so this
        # exercises unknown_top_level_key_message's "wrong scope" branch,
        # not its did-you-mean branch.
        self.arm()
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"entry_lower": True})

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success)
        self.assertIn("WARNING", result.message)
        self.assertIn("entry_lower", result.message)
        self.assertIn("family-scoped key", result.message)

        report = self.read_report()
        self.assertEqual(report["unknown_top_level_config_keys"], ["entry_lower"])

    def test_known_config_has_no_warning_and_empty_report_list(self):
        self.arm()
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"skin_margin_mm": 1.0})

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success)
        self.assertNotIn("unrecognized top-level key", result.message)

        report = self.read_report()
        self.assertEqual(report["unknown_top_level_config_keys"], [])


# ---------------------------------------------------------------------------
# 4b2. Stored config vs. running config: "two things that must agree, only
# one updated"
# ---------------------------------------------------------------------------
#
# WHAT THIS PINS: OrcaSlicer reads the plugin's stored config once, at app
# start. An edit to ``host/data/orca_plugins/config.json`` made while the host
# keeps running does not reach ``self.get_config()`` until the host restarts,
# so a run can use a stale in-memory copy (e.g. ``sensitivity_enabled: False``
# while the file says ``mesh_alignment_sensitivity_enabled: true``) with no
# indication the two disagree. The plugin reports this as a WARNING only
# (capability.py's ``_compute_config_drift``/``_config_drift_warning_suffix``
# docstrings): the host's in-memory copy stays authoritative for the run either
# way; the plugin's only job is to say the two disagree and that a restart is
# needed.
#
# These tests need the REAL host layout -- ``config.json`` living one level
# ABOVE the plugin's own folder (``_plugin_dir()``'s
# ``os.path.dirname(base)``) -- which ``M1IntegrationTestCase``'s flat
# ``self.plugin_dir`` (the entry file sitting directly in its own tempdir,
# with no room for a sibling file) does not provide. ``setUp`` below
# reproduces the nested layout instead of reusing the base class's.


class ConfigDriftWarningTests(M1IntegrationTestCase):
    def setUp(self):  # noqa: D401 - deliberately not calling super().setUp()
        self._fake_orca, restore = fake_orca.install()
        self.addCleanup(restore)

        self._container = tempfile.TemporaryDirectory()
        self.addCleanup(self._container.cleanup)
        # Named after the entry file, like a real install
        # (``data_dir()/orca_plugins/<entry_file_name>/``), so
        # ``os.path.dirname(_plugin_dir())`` -- where ``config.json`` is
        # read from -- lands in ``self._container.name``, a directory this
        # test owns and cleans up, not the shared system temp root.
        self.plugin_dir = os.path.join(self._container.name, "aircraft_structures.py")
        os.makedirs(self.plugin_dir)
        self.fake_entry_file = os.path.join(self.plugin_dir, "aircraft_structures.py")
        self.namespace = fake_orca.exec_bundle(self.fake_entry_file)
        self.report_path = os.path.join(self.plugin_dir, "m1", "m1_report.json")
        self.config_json_path = os.path.join(self._container.name, "config.json")

    def _write_disk_config(self, cap_config, capability="Aircraft Structures"):
        # The real wrapper shape ``tools/config.py``'s ``_entry_for``/
        # ``_cap_config_for`` read and ``presets/plugin/live_host_config.json``
        # captures verbatim -- NOT a flat cap_config dict. Diffing against
        # the wrong shape manufactures drift on every key, every run (see capability.py's ``_compute_config_drift`` docstring).
        doc = {
            "config": [
                {
                    "cap_config": cap_config,
                    "capability": capability,
                    "capability_type": "slicing-pipeline",
                    "plugin_key": "aircraft_structures",
                    "plugin_version": "0.1.0",
                }
            ]
        }
        with open(self.config_json_path, "w", encoding="utf-8") as fh:
            json.dump(doc, fh)

    def test_matching_config_has_no_warning(self):
        self.arm()
        self._write_disk_config({"skin_margin_mm": 1.0})
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"skin_margin_mm": 1.0})

        result = cap.execute(ctx)

        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertNotIn("config differs", result.message)
        report = self.read_report()
        self.assertEqual(report["config_drift"], [])

    def test_differing_value_warns_naming_key_both_values_and_restart(self):
        self.arm()
        self._write_disk_config({"skin_margin_mm": 2.0})
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"skin_margin_mm": 1.0})

        result = cap.execute(ctx)

        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertIn("WARNING", result.message)
        self.assertIn("config differs from config.json on disk", result.message)
        self.assertIn("skin_margin_mm", result.message)
        self.assertIn("1.0", result.message)
        self.assertIn("2.0", result.message)
        self.assertIn("restart", result.message.lower())

        report = self.read_report()
        self.assertEqual(len(report["config_drift"]), 1)
        entry = report["config_drift"][0]
        self.assertEqual(entry["key"], "skin_margin_mm")
        self.assertEqual(entry["host_value"], 1.0)
        self.assertEqual(entry["disk_value"], 2.0)

    def test_key_added_on_disk_but_absent_from_running_host_config_warns(self):
        # A field newly added to config.json on disk
        # (mesh_alignment_sensitivity_enabled) that the already-running
        # host's in-memory cap_config has never seen at all.
        self.arm()
        self._write_disk_config({"mesh_alignment_sensitivity_enabled": True})
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({})

        result = cap.execute(ctx)

        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertIn("WARNING", result.message)
        self.assertIn("mesh_alignment_sensitivity_enabled", result.message)
        self.assertIn("<absent>", result.message)

        report = self.read_report()
        self.assertEqual(len(report["config_drift"]), 1)
        entry = report["config_drift"][0]
        self.assertEqual(entry["key"], "mesh_alignment_sensitivity_enabled")
        self.assertEqual(entry["host_value"], "<absent>")
        self.assertEqual(entry["disk_value"], True)

    def test_flat_cap_config_at_top_level_is_silent(self):
        # A flat cap_config dict at the top level
        # (NOT the real {"config": [{"cap_config": ...}]} wrapper) must not
        # be diffed against as if it WERE a cap_config -- that manufactures
        # spurious drift on every key, every run. Silent, not loud.
        self.arm()
        with open(self.config_json_path, "w", encoding="utf-8") as fh:
            json.dump({"skin_margin_mm": 1.0}, fh)
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"skin_margin_mm": 1.0})

        result = cap.execute(ctx)

        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertNotIn("config differs", result.message)
        report = self.read_report()
        self.assertEqual(report["config_drift"], [])

    def test_wrapper_with_no_matching_capability_entry_is_silent(self):
        self.arm()
        self._write_disk_config({"skin_margin_mm": 2.0}, capability="Some Other Plugin")
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"skin_margin_mm": 1.0})

        result = cap.execute(ctx)

        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertNotIn("config differs", result.message)
        report = self.read_report()
        self.assertEqual(report["config_drift"], [])

    def test_missing_config_file_is_silent(self):
        self.arm()
        # No config.json written at all -- the plugin may run where that
        # file does not exist.
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"skin_margin_mm": 1.0})

        result = cap.execute(ctx)

        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertNotIn("config differs", result.message)
        report = self.read_report()
        self.assertEqual(report["config_drift"], [])

    def test_unreadable_config_file_is_silent(self):
        self.arm()
        # A directory where config.json should be: open() raises, and the
        # comparison must swallow that rather than crash the run.
        os.makedirs(self.config_json_path)
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"skin_margin_mm": 1.0})

        result = cap.execute(ctx)

        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertNotIn("config differs", result.message)
        report = self.read_report()
        self.assertEqual(report["config_drift"], [])

    def test_malformed_config_file_is_silent(self):
        self.arm()
        with open(self.config_json_path, "w", encoding="utf-8") as fh:
            fh.write("{not valid json")
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"skin_margin_mm": 1.0})

        result = cap.execute(ctx)

        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertNotIn("config differs", result.message)
        report = self.read_report()
        self.assertEqual(report["config_drift"], [])

    def test_warning_does_not_change_slice_outcome(self):
        self.arm()
        self._write_disk_config({"skin_margin_mm": 2.0})
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"skin_margin_mm": 1.0})

        result = cap.execute(ctx)

        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)
        self.assertIn("WARNING", result.message)  # drift IS reported...
        mutated_expected = 8 - 5  # DEFAULT_SPEC["root_solid_layers"] == 5
        # ...but every body layer is still slotted exactly as it would be
        # with no drift at all (compare ArmedHappyPathTests).
        self.assertIn("{} slotted".format(mutated_expected), result.message)
        for i in range(5, 8):
            layer = obj.layers()[i]
            self.assertEqual(layer.make_slices_calls, 1, "layer {} make_slices calls".format(i))


# ---------------------------------------------------------------------------
# 4c. feature_dense_lattice_enabled reaches the built StructureSpec
# ---------------------------------------------------------------------------
#
# WHAT THIS PINS: aircraft_core/config_schema.py's SPEC_FIELDS/DEFAULT_SPEC
# recognise "feature_dense_lattice_enabled" as a key (so it never warns as
# unknown), but that only helps if capability.py's StructureSpec(...) call in
# _build_spec actually READS ``values["feature_dense_lattice_enabled"]``;
# otherwise a stored config that sets it True would silently produce a spec
# with the DATACLASS default (False). Unit tests that construct
# ``StructureSpec`` directly bypass ``_build_spec`` and cannot see that. This
# class drives a stored config dict through the SAME
# ``AircraftStructuresM1._build_spec`` the real host calls, and asserts the
# flag lands on the returned spec.


class FeatureDenseLatticeEnabledConfigTests(M1IntegrationTestCase):
    def test_true_in_stored_config_reaches_the_built_spec(self):
        obj = self.make_object(n_layers=1)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"feature_dense_lattice_enabled": True})

        spec, _resolved, _metrics_enabled, _unknown = cap._build_spec(ctx)

        self.assertTrue(
            spec.feature_dense_lattice_enabled,
            "feature_dense_lattice_enabled=True in the stored config did "
            "not reach the built StructureSpec -- _build_spec's own "
            "StructureSpec(...) call must pass "
            "feature_dense_lattice_enabled=bool(values["
            "\"feature_dense_lattice_enabled\"]).",
        )

    def test_false_default_reaches_the_built_spec(self):
        obj = self.make_object(n_layers=1)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({})

        spec, _resolved, _metrics_enabled, _unknown = cap._build_spec(ctx)

        self.assertFalse(spec.feature_dense_lattice_enabled)

    def test_end_to_end_slice_with_flag_true_still_succeeds(self):
        # Not just the spec field: a full arm -> execute pass with the flag
        # set, on a plain coupon (never classified FEATURE, so this proves
        # the flag reaching the spec does not itself perturb an ordinary
        # aerofoil/coupon slice -- see AerofoilNoRegressionTests in
        # tests/unit/test_m36_dense_feature_lattice.py for the unit-level
        # version of this same claim).
        self.arm()
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"feature_dense_lattice_enabled": True})

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success)


# ---------------------------------------------------------------------------
# 4d. spar_channel_policy reaches the built StructureSpec (same inert-field
# check as 4c, driven through the same real path)
# ---------------------------------------------------------------------------
#
# capability.py's DEFAULT_SPEC/config_schema.py's SPEC_FIELDS declaring a key
# does not mean _build_spec's own StructureSpec(...) call actually reads it
# (the gap 4c checks for feature_dense_lattice_enabled). This class drives
# the same stored-config-through-_build_spec path for spar_channel_policy.


class SparChannelPolicyConfigTests(M1IntegrationTestCase):
    def test_nearest_spar_station_in_stored_config_reaches_the_built_spec(self):
        obj = self.make_object(n_layers=1)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {"spar_channel_policy": "nearest_spar_station"}
        )

        spec, _resolved, _metrics_enabled, _unknown = cap._build_spec(ctx)

        self.assertEqual(
            spec.spar_channel_policy,
            "nearest_spar_station",
            "spar_channel_policy=\"nearest_spar_station\" in the stored "
            "config did not reach the built StructureSpec -- _build_spec's "
            "own StructureSpec(...) call must pass "
            "spar_channel_policy=str(values[\"spar_channel_policy\"]).",
        )

    def test_refuse_default_reaches_the_built_spec(self):
        obj = self.make_object(n_layers=1)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({})

        spec, _resolved, _metrics_enabled, _unknown = cap._build_spec(ctx)

        self.assertEqual(spec.spar_channel_policy, "refuse")

    def test_end_to_end_slice_with_policy_set_still_succeeds(self):
        # Not just the spec field: a full arm -> execute pass with the
        # policy set, on a plain coupon (never carries more than one
        # detected channel), proves the policy reaching the spec does not
        # itself perturb an ordinary slice with no channels at all.
        self.arm()
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {"spar_channel_policy": "nearest_spar_station"}
        )

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success)


# ---------------------------------------------------------------------------
# 4e. The spar-socket latch must never couple to a bore socket
# ---------------------------------------------------------------------------
#
# A bore socket has its own family_name ("bore_socket", never "spar_socket")
# and its own degradation kind (see aircraft_core/generator.py's
# _generate_bore_socket_from_channel docstring) because capability.py's
# _apply_spar_socket_latch keys on the literal string "spar_socket" in both
# directions: it ARMS on a "spar_socket" degradation, and it SUPPRESSES any
# resumed "spar_socket" intent once armed. This class exercises that absence
# of coupling for a bore by calling the real method directly (it is a method
# precisely so it can be tested directly).


class SparSocketLatchBoreCouplingTests(M1IntegrationTestCase):
    def _intent(self, WebIntent, x, family_name, family_index,
                spar_confirmed=False):
        return WebIntent(
            start=(x, 0.0), end=(x, 5.0), width_mm=1.0,
            family_index=family_index, family_name=family_name,
            socket_radius_mm=1.5, spar_confirmed=spar_confirmed,
        )

    def test_bore_socket_degradation_does_not_arm_the_latch(self):
        cap = self.new_capability()
        cap._spar_socket_ended = False  # normally set at the run boundary
        WebIntent = self.namespace["WebIntent"]
        bore = self._intent(WebIntent, 10.0, "bore_socket", -7)

        intents, degradations = cap._apply_spar_socket_latch(
            [bore],
            [{"kind": "bore_socket", "x_mm": 10.0, "reason": "irrelevant"}],
        )

        self.assertFalse(
            cap._spar_socket_ended,
            "a bore_socket degradation armed the spar-socket latch -- "
            "only a 'spar_socket'-kind degradation may.",
        )
        self.assertEqual(intents, [bore])
        self.assertEqual(len(degradations), 1)

    def test_an_already_armed_latch_does_not_suppress_a_bore_socket(self):
        cap = self.new_capability()
        # Simulate the REAL spar's own socket having ended earlier in this
        # run (e.g. the section thinned past it, well before this layer).
        cap._spar_socket_ended = True
        WebIntent = self.namespace["WebIntent"]
        bore = self._intent(WebIntent, 10.0, "bore_socket", -7)

        intents, degradations = cap._apply_spar_socket_latch([bore], [])

        self.assertEqual(
            intents, [bore],
            "an armed spar-socket latch suppressed a bore socket -- "
            "its own 'resumed' filter must key on family_name == "
            "'spar_socket' only, never 'bore_socket'.",
        )
        self.assertEqual(degradations, [])

    def test_control_an_already_armed_latch_still_suppresses_a_resumed_spar_socket(self):
        # Confirms the latch mechanism itself is genuinely armed in this test
        # (not merely inert) -- the two tests above pass because the coupling
        # is absent, not because the latch never does anything.
        # ``spar_confirmed=True`` is explicit and load-bearing here: this is
        # the ONE case the latch must still catch, a genuinely CONFIRMED spar
        # resuming after it ended elsewhere.
        cap = self.new_capability()
        cap._spar_socket_ended = True
        WebIntent = self.namespace["WebIntent"]
        spar = self._intent(
            WebIntent, 10.0, "spar_socket", -2, spar_confirmed=True)

        intents, degradations = cap._apply_spar_socket_latch([spar], [])

        self.assertEqual(intents, [])
        self.assertEqual(len(degradations), 1)
        self.assertEqual(degradations[0]["kind"], "spar_socket_latched")

    def test_an_armed_latch_does_not_suppress_an_unconfirmed_spar_socket(self):
        """A socket that carries ``family_name== "spar_socket"`` but is
        NOT confirmed (``spar_confirmed=False`` -- chosen by
        ``spar_channel_policy="nearest_spar_station"`` disambiguating among 2+
        candidates, e.g. an alignment bore that won the "nearest to
        spar_station" contest after the real spar already ended) must pass
        straight through an already-armed latch, exactly like a ``bore_socket``
        does -- see
        ``test_an_already_armed_latch_does_not_suppress_a_bore_socket`` above
        for the bore's own version of this same guarantee."""
        cap = self.new_capability()
        cap._spar_socket_ended = True
        WebIntent = self.namespace["WebIntent"]
        unconfirmed_spar = self._intent(
            WebIntent, 10.0, "spar_socket", -2, spar_confirmed=False)

        intents, degradations = cap._apply_spar_socket_latch(
            [unconfirmed_spar], [])

        self.assertEqual(
            intents, [unconfirmed_spar],
            "an armed spar-socket latch suppressed an UNCONFIRMED "
            "spar_socket intent -- its 'resumed' filter must also require "
            "spar_confirmed=True, never family_name alone.",
        )
        self.assertEqual(degradations, [])

    def test_an_unconfirmed_spar_socket_degradation_does_not_arm_the_latch(self):
        """A fit-check failure for an UNCONFIRMED spar candidate reports
        under its own kind (``aircraft_core.generator``'s
        ``_build_channel_socket``, ``"spar_socket_unconfirmed"``), never the
        literal ``"spar_socket"`` the latch arms on -- mirrors
        ``test_bore_socket_degradation_does_not_arm_the_latch`` above."""
        cap = self.new_capability()
        cap._spar_socket_ended = False
        WebIntent = self.namespace["WebIntent"]
        unconfirmed_spar = self._intent(
            WebIntent, 10.0, "spar_socket", -2, spar_confirmed=False)

        intents, degradations = cap._apply_spar_socket_latch(
            [unconfirmed_spar],
            [{
                "kind": "spar_socket_unconfirmed", "x_mm": 10.0,
                "reason": "irrelevant",
            }],
        )

        self.assertFalse(
            cap._spar_socket_ended,
            "a 'spar_socket_unconfirmed' degradation armed the "
            "spar-socket latch -- only a 'spar_socket'-kind degradation "
            "may.",
        )
        self.assertEqual(intents, [unconfirmed_spar])
        self.assertEqual(len(degradations), 1)


# ---------------------------------------------------------------------------
# 4f. A CONFIRMED spar socket dropped by depth-unfit (generator.py
# _build_channel_socket, the "section too shallow here" branch) must arm the
# SAME latch a config-fallback socket drop does.
# ---------------------------------------------------------------------------
#
# This class drives the depth-unfit branch (``depth < needed``) of the helper,
# which degrades a CONFIRMED channel under the literal "spar_socket" kind. The
# SIBLING "channel too small for a keyhole" guard (half_w >= r) cannot be used
# for this: it refuses the whole layer with a TopologyError rather than
# degrading (see ConfirmedSparChannelTooSmallRefusesTests in
# tests/unit/test_channel_too_small_for_keyhole.py), so it never reaches the
# latch. The depth-unfit branch is also the one seen in practice on a real
# wing (49 confirmed depth-unfit degradations at Z 189.4-199.0).
#
# The test exercises the end-to-end claim through the shipped bundle's real
# generator, not just the latch method in isolation (see SparSocketLatchTests
# in test_m18_mesh_transform.py for latch-method-only coverage, including its
# generic ``test_a_dropped_socket_engages_the_latch``).


class ChannelTooSmallForKeyholeLatchTests(M1IntegrationTestCase):
    def test_confirmed_depth_unfit_socket_arms_the_d034_latch(self):
        LayerFrame = self.namespace["LayerFrame"]
        DetectedChannel = self.namespace["DetectedChannel"]
        LayerContext = self.namespace["LayerContext"]
        StructureSpec = self.namespace["StructureSpec"]
        StructureGenerator = self.namespace["StructureGenerator"]

        ops = StubOps()
        # Shallow rectangle (5 mm tall): the channel below fits its
        # keyhole (r=1.65 mm > half_w=0.6 mm, clears the OTHER guard) but
        # needed=2r+2*skin_margin=5.3 mm > 5.0 mm available depth -- the
        # depth-unfit branch, not the keyhole-too-small one.
        rect = Rect(0.0, 0.0, 40.0, 5.0)
        frame = LayerFrame.from_polygon(ops, rect)

        channel = DetectedChannel(
            center_x_mm=20.0, center_y_mm=2.5, diameter_mm=3.0,
        )
        ctx = LayerContext(
            layer_index=989, print_z=198.8, detected_channels=(channel,),
        )
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, spar_clearance_mm=0.15,
            spar_neck_width_mm=1.2,
        )

        result = StructureGenerator().generate(
            frame, spec, 0.42, ctx, safety_offset_mm=0.42,
        )
        self.assertEqual(
            [d.kind for d in result.degradations], ["spar_socket"],
            "the single-channel branch is unambiguous and must report "
            "under the literal 'spar_socket' kind the latch arms on",
        )
        self.assertIn(
            "does not fit", result.degradations[0].reason,
            "must be the depth-unfit branch, not the keyhole-too-small "
            "guard (which now refuses rather than degrading when "
            "confirmed)",
        )

        layer_degradations = [
            {"kind": d.kind, "x_mm": round(d.x_mm, 4), "reason": d.reason}
            for d in result.degradations
        ]
        cap = self.new_capability()
        cap._spar_socket_ended = False  # normally set at the run boundary

        intents, degradations = cap._apply_spar_socket_latch(
            result.intents, layer_degradations,
        )

        self.assertTrue(
            cap._spar_socket_ended,
            "a confirmed spar socket dropped by depth-unfit did not arm "
            "the run-level latch",
        )
        self.assertEqual(degradations, layer_degradations)


# ---------------------------------------------------------------------------
# 5. Cancellation
# ---------------------------------------------------------------------------


class CancellationTests(M1IntegrationTestCase):
    def test_cancellation_partway_returns_success_not_failure(self):
        self.arm()
        n_layers = 50
        obj = self.make_object(n_layers=n_layers)

        # ctx.cancelled() is called once per layer iteration, in order.
        # Layers 0-4 are root (untouched), 5-9 get mutated (5 calls to
        # cancelled() so far all False), the 11th call (layer index 10)
        # returns True and must abort before layer 10 is touched at all.
        calls = {"n": 0}

        def cancelled_fn():
            calls["n"] += 1
            return calls["n"] > 10

        ctx = self.make_ctx(obj, cancelled_fn=cancelled_fn)
        cap = self.new_capability()

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success)
        self.assertEqual(result.message, "slicing cancelled")

        layers = obj.layers()
        for i in range(5):
            self.assert_layer_untouched(layers[i], expected_area_mm2=400.0)
        for i in range(5, 10):
            self.assertEqual(layers[i].make_slices_calls, 1)
            self.assertEqual(fake_orca.describe(self.surfaces_of(layers[i])[0].expolygon)["kind"], "notched")
        for i in range(10, n_layers):
            self.assertEqual(layers[i].make_slices_calls, 0)
            self.assertEqual(fake_orca.describe(self.surfaces_of(layers[i])[0].expolygon)["kind"], "rect")

        # Cancellation must not raise, and must not be reported via the
        # normal end-of-run diagnostics write (the capability's
        # `except CanceledSlice` branch never calls _write_diagnostics).
        self.assertFalse(os.path.exists(self.report_path))


# ---------------------------------------------------------------------------
# 6. Diagnostics
# ---------------------------------------------------------------------------


class DiagnosticsTests(M1IntegrationTestCase):
    def test_successful_run_report_matches_real_layer_and_mutation_counts(self):
        self.arm()
        n_layers = 20
        obj = self.make_object(n_layers=n_layers)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()

        result = cap.execute(ctx)
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success)

        self.assertTrue(os.path.exists(self.report_path))
        report = self.read_report()

        self.assertEqual(report["layers_seen"], n_layers)  # root layers included
        self.assertEqual(report["mutated_count"], n_layers - 5)
        self.assertEqual(report["failed_count"], 0)
        self.assertIsNone(report["first_failure"])
        self.assertEqual(len(report["layers"]), n_layers)
        self.assertEqual(report["root_layers"], 5)
        self.assertEqual(report["orca_version"], ctx.orca_version)
        self.assertIn("elapsed_ms", report)
        self.assertIn("config", report)
        self.assertEqual(report["config"]["outer_wall_line_width"], 0.42)

    def test_failing_run_report_records_first_failure_with_context(self):
        """A failing run's on-disk report lets a human locate which layer
        failed without re-reading the ExecutionResult message: layers_seen
        and a structured first_failure record, the same shape a successful
        run's report has.

        See capability.py's `execute`, the `except TopologyError` branch: it
        builds the on-disk report from the same `RunDiagnostics` instance
        used on the success path (via `diagnostics.record(...)` before
        `diagnostics.to_dict()`), so `layers_seen`, `mutated_count`, and a
        structured `first_failure` record are present exactly as on a
        successful run, in addition to the free-text `error` message
        embedding the layer index and Z.

        The fixture must reach `_transform`'s `except TopologyError` branch,
        so it uses ``_same_family_proximity_config`` (a same-family
        marched-pitch collision on the airfoil fixture; see that helper's
        docstring for why not the coupon). A config like
        ``skin_margin_mm=0.1`` is not usable: the
        ``skin_margin_mm > 2 * safety_offset_mm`` guard raises ``SpecError``
        at config-build time, before any layer is walked, and writes NO
        report (see ``FailClosedDisconnectionTests`` for that path).
        ``skin_margin_mm=1.0`` here clears ``2 * 0.42 = 0.84`` comfortably.
        """
        self.arm()
        obj, _bbox = fake_orca.make_airfoil_object("wing", 5, 0.2)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap_config, _rib_pitch_mm = _same_family_proximity_config()
        cap.get_config = lambda: json.dumps(cap_config)

        result = cap.execute(ctx)
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)

        self.assertTrue(os.path.exists(self.report_path))
        report = self.read_report()

        # Context is present as free text in "error":
        self.assertIn("layer 0", report.get("error", ""))

        # Structured fields a human/automation would want from the JSON
        # report directly, without parsing "error".
        self.assertIn("layers_seen", report)
        self.assertIn("first_failure", report)
        self.assertEqual(report.get("first_failure", {}).get("layer_index"), 0)

    def test_geometry_at_failure_does_not_carry_across_runs(self):
        """``_last_geometry`` and ``_root_layers`` are plain instance
        attributes, only ever (re)written from inside the per-layer SUCCESS
        path of ``_transform``. If the host reuses one capability instance
        across slices, a run that fails before its own first successful layer
        must not let its failure report describe a PRIOR run's geometry on
        this same instance.

        Run A: the plain rectangular coupon, which succeeds and populates
        ``_last_geometry``/``_root_layers`` from its own (very different)
        frame bounds. Run B: a single body layer whose region has NO sliced
        surfaces at all -- capability.py's own "region has no sliced
        surfaces" ``TopologyError`` (raised well before ``LayerFrame`` is
        even built, let alone ``_last_geometry`` assigned; see
        ``_transform``'s per-region loop) -- on the SAME capability
        instance. This is a genuine "fails before this run's own first
        successful layer" case, not merely a later-stage validator collision
        that still reaches the ``_last_geometry`` assignment for its own
        layer first. Run B's report must show this run's own (absent)
        geometry, not run A's.
        """
        self.arm()
        cap = self.new_capability()

        # Run A: succeeds, populates _last_geometry/_root_layers from the
        # rectangular coupon's own geometry.
        obj_a = self.make_object(n_layers=20)
        ctx_a = self.make_ctx(obj_a)
        result_a = cap.execute(ctx_a)
        self.assertEqual(result_a.status, self._fake_orca.PluginResult.Success)
        report_a = self.read_report()
        root_layers_a = report_a.get("root_layers")
        self.assertEqual(root_layers_a, 5, "sanity: run A actually captured its own root_layers")

        # Run B: same instance, one body layer whose region has no sliced
        # surfaces -- fails before this run ever reaches the per-layer
        # success path that would (re)populate _last_geometry/_root_layers.
        empty_region = self._fake_orca.host.LayerRegion(
            self._fake_orca.host.SurfaceCollection([])
        )
        layer_b = self._fake_orca.host.Layer(print_z=0.2, regions=[empty_region], slice_z=0.1)
        obj_b = self._fake_orca.host.PrintObject("empty_region_wing", [layer_b])
        ctx_b = self.make_ctx(obj_b)
        cap.get_config = lambda: json.dumps({"root_solid_layers": 0})

        result_b = cap.execute(ctx_b)
        self.assertEqual(result_b.status, self._fake_orca.PluginResult.FatalError)
        report_b = self.read_report()

        # requires a readable model mesh (module docstring); this object has
        # none, so the failure fires even before the per-region "no sliced
        # surfaces" check -- still well before any per-layer success path that
        # would (re)populate _last_geometry/_root_layers.
        self.assertIn("model mesh could not be read", report_b.get("error", ""))
        # Run B never reached a successful layer, so it must NOT report run A's
        # stale success-path geometry. It is not required to be None either:
        # ``_transform`` writes a fresh, minimally-complete per-layer stub for
        # THIS run's own layer 0 before the fatal-mesh-error raise fires
        # (which happens INSIDE the layer loop, not before it). The
        # correctness bar is that the stub names run B's OWN layer 0, not
        # run A's, and every unknown field is explicit ``None``/``False``
        # rather than a leftover from run A's completed geometry.
        geometry_b = report_b.get("geometry_at_failure")
        self.assertIsNotNone(geometry_b)
        self.assertEqual(geometry_b.get("layer_index"), 0)
        self.assertFalse(geometry_b.get("complete"))
        self.assertIsNone(geometry_b.get("host_contour_pts_mm"))
        self.assertIsNone(geometry_b.get("our_section_pts_mm"))
        # None of run A's own fields (e.g. its "frame_bounds", present only
        # on a fully-enriched/"complete" geometry) leaked across.
        self.assertNotIn("frame_bounds", geometry_b)
        self.assertEqual(report_b.get("root_layers"), 0)
        # ...and specifically not run A's own root_layers value bleeding
        # across the shared instance.
        self.assertNotEqual(report_b.get("root_layers"), root_layers_a)

    def test_geometry_and_mesh_alignment_do_not_carry_across_runs_via_generic_handler(self):
        """Same regression as ``test_geometry_at_failure_does_not_carry_across_runs``,
        but for ``execute``'s OTHER report-writing path: the generic
        ``except Exception`` handler (capability.py, after the typed
        ``TopologyError``/``SpecError``/``CanceledSlice`` branches), which
        fires for an UNEXPECTED exception.

        That handler reads all three of ``_last_geometry``, ``_root_layers``,
        AND ``_mesh_alignment`` via ``getattr(self, ..., None/0)``. Run B
        here raises its unexpected exception from inside ``_build_spec`` --
        BEFORE ``_transform`` is ever entered -- so ``_mesh_alignment``'s
        own normally-unconditional assignment near the top of ``_transform``
        never happens this run either; only the reset at the top of
        ``execute`` can make this run's report show its own (absent) state
        instead of run A's.
        """
        self.arm()
        cap = self.new_capability()

        # Run A: succeeds, populates _last_geometry/_root_layers/
        # _mesh_alignment from the rectangular coupon's own geometry.
        obj_a = self.make_object(n_layers=20)
        ctx_a = self.make_ctx(obj_a)
        result_a = cap.execute(ctx_a)
        self.assertEqual(result_a.status, self._fake_orca.PluginResult.Success)
        report_a = self.read_report()
        self.assertEqual(report_a.get("root_layers"), 5, "sanity: run A captured its own root_layers")
        self.assertIsNotNone(report_a.get("mesh_alignment"), "sanity: run A captured its own mesh_alignment")

        # Run B: same instance, _build_spec raises an UNEXPECTED exception
        # (a bare RuntimeError -- not SpecError, not TopologyError, not
        # CanceledSlice) so it lands in the generic `except Exception`
        # handler, not one of the typed branches.
        obj_b = self.make_object(n_layers=5)
        ctx_b = self.make_ctx(obj_b)

        def _boom(_ctx):
            raise RuntimeError("boom: unexpected failure in _build_spec")

        cap._build_spec = _boom

        result_b = cap.execute(ctx_b)
        self.assertEqual(result_b.status, self._fake_orca.PluginResult.FatalError)
        self.assertIn("boom", result_b.message)
        report_b = self.read_report()

        self.assertTrue(report_b.get("unexpected_exception"))
        self.assertIsNone(report_b.get("geometry_at_failure"))
        self.assertEqual(report_b.get("root_layers"), 0)
        self.assertIsNone(report_b.get("mesh_alignment"))

    def test_run_level_raise_after_the_per_layer_loop_reports_the_last_layers_own_stub(self):
        """A raise that happens AFTER the per-layer loop finishes -- at run
        level, not tied to any specific layer's own processing -- must still
        report ``geometry_at_failure`` as the LAST layer's own honest stub (its
        own layer_index, ``complete: False``), never as though that stub were
        itself the cause of the failure.

        Fixture: every layer is a root-solid layer (``root_solid_layers``
        == ``n_layers``), so each layer only ever reaches the Stage-1 stub
        write (own layer_index, ``complete: False``, no host contour, no
        mesh section) before ``continue`` -- no mesh, no channel detection,
        no per-region processing at all. ``_transform``'s ONLY statement
        after the per-layer loop, ``self._root_layers = min(spec.
        root_solid_layers, layers_seen)`` (capability.py), is made to raise
        by handing it a ``root_solid_layers`` value that behaves like a
        normal int for every IN-LOOP comparison (``index <
        spec.root_solid_layers``, called once per layer) but raises on the
        (N+1)th comparison -- exactly the one ``min()`` performs after the
        loop. This isolates a genuinely run-level failure: nothing about
        any single layer's own geometry caused it.
        """
        n_layers = 5

        class _PoisonedAfterLoopComparisons:
            """Compares normally for exactly ``boom_after`` calls (one per
            layer, in the loop's own ``index < root_solid_layers`` check),
            then raises on the next comparison -- the ``min()`` call that
            runs once, after the loop, in ``_transform``'s own tail."""

            def __init__(self, real_value, boom_after):
                self._real = real_value
                self._count = 0
                self._boom_after = boom_after

            def __gt__(self, other):
                # Both ``index < self`` (reflected as ``self.__gt__(index)``)
                # and ``min()``'s internal ``layers_seen < self`` (also
                # reflected the same way, since ``other`` is a plain int)
                # route through this one method -- see this class's own
                # docstring for why counting calls, not distinguishing
                # caller, is what isolates "after the loop".
                self._count += 1
                if self._count > self._boom_after:
                    raise TypeError(
                        "boom: poisoned root_solid_layers comparison "
                        "(post-loop, call {})".format(self._count)
                    )
                return self._real > other

            def __repr__(self):
                return repr(self._real)

        obj = self.make_object(n_layers=n_layers)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        self.arm()

        original_build_spec = cap._build_spec

        def _poisoned_build_spec(build_ctx):
            spec, resolved, metrics_enabled, unknown_config_keys = (
                original_build_spec(build_ctx)
            )
            spec.root_solid_layers = _PoisonedAfterLoopComparisons(
                spec.root_solid_layers, boom_after=n_layers
            )
            return spec, resolved, metrics_enabled, unknown_config_keys

        cap._build_spec = _poisoned_build_spec
        cap.get_config = lambda: json.dumps({
            "root_solid_layers": n_layers, "skin_margin_mm": 1.0,
        })

        result = cap.execute(ctx)

        # Lands in the GENERIC exception handler (a bare TypeError, not
        # TopologyError/SpecError/CanceledSlice) -- proving this is a
        # run-level defect, not a per-layer geometry failure.
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        self.assertIn("boom", result.message)
        report = self.read_report()
        self.assertTrue(report.get("unexpected_exception"))

        geo = report.get("geometry_at_failure")
        self.assertIsNotNone(geo)
        # The LAST layer processed (index n_layers - 1) -- its own stub,
        # never a different layer's, and never claiming to BE the failure.
        self.assertEqual(geo["layer_index"], n_layers - 1)
        self.assertFalse(geo.get("complete"))
        self.assertIsNone(geo.get("host_contour_pts_mm"))
        self.assertIsNone(geo.get("our_section_pts_mm"))
        # root_layers could not be computed (the raise happened inside that
        # very statement) -- the sentinel (0), not a stale prior run's value
        # and not a guess.
        self.assertEqual(report.get("root_layers"), 0)


# ---------------------------------------------------------------------------
# 7. Non-posSlice steps
# ---------------------------------------------------------------------------


class NonPosSliceStepTests(M1IntegrationTestCase):
    def test_every_other_step_mutates_nothing_and_succeeds(self):
        """execute() returns success() unconditionally for every step other
        than posSlice (capability.py: ``if ctx.step != Step.posSlice: return
        orca.ExecutionResult.success()``), before ever touching ctx.object.
        Covers all 12 non-posSlice SlicingPipelineStepPlugin values,
        including the three real Orca fires with ctx.object None
        (psWipeTower, psSkirtBrim, psGCodePostProcess —
        SlicingPipelinePluginCapability.cpp:25-33): those are exercised here
        with obj=None, matching what the real pipeline actually hands the
        capability at those steps."""
        self.arm()
        obj = self.make_object(n_layers=5)
        cap = self.new_capability()
        Step = self._fake_orca.slicing.Step
        PluginResult = self._fake_orca.PluginResult

        self.assertEqual(len(fake_orca.NON_SLICE_STEPS), 12)

        for step in fake_orca.NON_SLICE_STEPS:
            step_obj = None if step in fake_orca.OBJECT_NONE_STEPS else obj
            ctx = self.make_ctx(step_obj, step=step)
            result = cap.execute(ctx)
            self.assertEqual(result.status, PluginResult.Success, "step={}".format(step))
            self.assertEqual(result.message, "", "step={}".format(step))

        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 0)
            self.assertEqual(fake_orca.describe(self.surfaces_of(layer)[0].expolygon)["kind"], "rect")

        self.assertFalse(os.path.exists(self.report_path))


if __name__ == "__main__":
    unittest.main()


class ShellAlignmentPreconditionTests(M1IntegrationTestCase):
    """The web root must not start above the last solid bottom shell layer.

    Spiral perimeters begin at ``layer.id() >= bottom_shell_layers``
    (LayerRegion.cpp:80-83, GCode.cpp:4539-4550). If the solid root outlasts
    them, the first slotted layer lays both web walls across a hollow one-wall
    ring: an unsupported drooping extrusion right above the base. That is a
    print hazard, so it must fail closed rather than warn.
    """

    def test_root_deeper_than_bottom_shell_is_fatal_and_mutates_nothing(self):
        self.arm()
        obj = self.make_object(n_layers=8)
        ctx = self.make_ctx(obj, config=dict(FULL_CONFIG, bottom_shell_layers=2))
        cap = self.new_capability()

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("root_solid_layers=5", result.message)
        self.assertIn("bottom_shell_layers=2", result.message)

        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 0)
            self.assertEqual(
                fake_orca.describe(self.surfaces_of(layer)[0].expolygon)["kind"], "rect"
            )
