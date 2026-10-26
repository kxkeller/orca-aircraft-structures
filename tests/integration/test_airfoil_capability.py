"""Integration tests for the generated bundle over a cambered
airfoil section, run against the fake Orca host.

WHY THIS EXISTS: ``test_m1_capability.py`` drives the SHIPPED bundle only
over rectangular and single-notched sections. These tests exercise the
capability's end-to-end control flow (frame -> generator -> slot -> diff ->
validate -> set -> make_slices) against genuine camber, which
``aircraft_core``'s own unit tests only reach through a hand-built
``stub_ops.NotchedPolygon`` in ``tests/unit/test_validate.py``'s
``TopologyValidatorAirfoilTests`` (driving ``TopologyValidator`` directly,
never through the capability's slot generation or ``orca.host``-shaped
calls).

This module execs the checked-in, generated
``plugin/aircraft_structures/aircraft_structures.py`` bundle — never
``capability.py`` directly — against ``fake_orca.py``'s fake ``orca``
module (see that file's module docstring "DOCUMENTED SCOPE" for its
``_Polygon``/``_PolygonNotched`` shape kernel and "KNOWN DIVERGENCES"
(12)-(15) for exactly what is, and is not, faithful about it), over a layer
stack of real NACA 2412 sections built from the SAME
``tools/make_fixtures.py`` code ``test_local_geometry.py`` and
``test_validate.py`` use — no second airfoil implementation here.

Nothing here modifies the code under test — it only reads it.
"""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fake_orca  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)

from aircraft_core.frame import LayerFrame, local_span, DEFAULT_PROBE_WIDTH_MM  # noqa: E402
from aircraft_core.ops import GeometryOps  # noqa: E402


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
    "bottom_shell_layers": 0,
    "sparse_infill_density": 0,
    "enable_support": False,
    "slice_closing_radius": 0.0049,
}

OUTER_WALL_MM = FULL_CONFIG["outer_wall_line_width"]


class _IndependentHostOps(GeometryOps):
    """A second, independent ``GeometryOps`` adapter over ``fake_orca``'s
    ``orca.host`` primitives, used ONLY by these tests to cross-check the
    capability's own output against ``aircraft_core.frame.local_span`` —
    never used to drive the capability itself (that is always the real
    generated bundle, exec'd via ``fake_orca.exec_bundle``). This is a
    deliberately separate, minimal copy of ``OrcaGeometryOps``'s conversion
    logic (plugin/aircraft_structures/capability.py) so a test assertion
    never depends on the very adapter code path under test.
    """

    def __init__(self):
        self._f = 1.0 / fake_orca.SCALING_FACTOR

    def _to_scaled(self, mm):
        return int(round(mm * self._f))

    def polygon_from_points(self, points_mm):
        contour = fake_orca.Polygon()
        for x, y in points_mm:
            contour.append(fake_orca.Point(self._to_scaled(x), self._to_scaled(y)))
        return fake_orca.ExPolygon(contour, [])

    def diff(self, subject, clips):
        current = [subject]
        for clip in clips:
            nxt = []
            for piece in current:
                nxt.extend(piece.diff_ex(clip))
            current = nxt
            if not current:
                break
        return current

    def offset(self, polygon, delta_mm):
        return polygon.offset(self._to_scaled(delta_mm))

    def intersection(self, subject, clip):
        return subject.intersection_ex(clip)

    def area(self, polygon):
        return polygon.area() / (self._f * self._f)

    def is_valid(self, polygon):
        return polygon.is_valid()

    def bounds(self, polygon):
        points = polygon.contour.points
        xs = [pt.x for pt in points]
        ys = [pt.y for pt in points]
        return (
            min(xs) / self._f,
            min(ys) / self._f,
            max(xs) / self._f,
            max(ys) / self._f,
        )

    def contour_count(self, polygon):
        return polygon.num_contours()

    def contour_points_mm(self, polygon):
        points = polygon.contour.points
        return [(pt.x / self._f, pt.y / self._f) for pt in points]

    def hole_bounds(self, polygon):
        bounds = []
        for hole in polygon.holes:
            points = hole.points
            xs = [pt.x for pt in points]
            ys = [pt.y for pt in points]
            bounds.append(
                (min(xs) / self._f, min(ys) / self._f, max(xs) / self._f, max(ys) / self._f)
            )
        return bounds

    def hole_points_mm(self, polygon):
        return [
            [(pt.x / self._f, pt.y / self._f) for pt in hole.points]
            for hole in polygon.holes
        ]


class AirfoilIntegrationTestCase(unittest.TestCase):
    """Common fixture plumbing, mirroring ``test_m1_capability.py``'s
    ``M1IntegrationTestCase``."""

    def setUp(self):
        self._fake_orca, restore = fake_orca.install()
        self.addCleanup(restore)

        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.plugin_dir = self._tmp.name
        self.fake_entry_file = os.path.join(self.plugin_dir, "aircraft_structures.py")

        self.namespace = fake_orca.exec_bundle(self.fake_entry_file)
        self.report_path = os.path.join(self.plugin_dir, "m1", "m1_report.json")

    def arm(self):
        with open(os.path.join(self.plugin_dir, "m1_enabled"), "w", encoding="utf-8"):
            pass

    def new_capability(self):
        cls = self.namespace["AircraftStructuresM1"]
        return cls()

    def make_airfoil_object(self, n_layers, layer_height_mm=0.2, object_id="wing"):
        obj, _bbox = fake_orca.make_airfoil_object(object_id, n_layers, layer_height_mm)
        return obj

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


# ---------------------------------------------------------------------------
# 1 & 2. Armed happy path over the real NACA 2412 stack, and the
#        boundary-connectedness of the cut on a cambered section.
# ---------------------------------------------------------------------------


class ArmedAirfoilHappyPathTests(AirfoilIntegrationTestCase):
    """A sensible spec (``web_position=0.7``, the same station used by
    ``tests/unit/test_validate.py``'s ``TopologyValidatorAirfoilTests``)
    slots every body layer of a genuinely cambered constant-section stack.
    """

    N_LAYERS = 6
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5
    WEB_POSITION = 0.7

    def setUp(self):
        super().setUp()
        self.arm()
        self.obj = self.make_airfoil_object(n_layers=self.N_LAYERS)
        ctx = self.make_ctx(self.obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "gap_ratio": self.GAP_RATIO,
                "web_position": self.WEB_POSITION,
            }
        )
        self.result = cap.execute(ctx)

    def test_every_body_layer_slotted_once_and_result_is_success(self):
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(self.result.status, PluginResult.Success, self.result.message)
        self.assertIn("{} layers seen".format(self.N_LAYERS), self.result.message)
        self.assertIn("{} slotted".format(self.N_LAYERS), self.result.message)
        self.assertIn("0 left solid", self.result.message)

        for layer in self.obj.layers():
            self.assertEqual(layer.make_slices_calls, 1)
            surfaces = self.surfaces_of(layer)
            self.assertEqual(len(surfaces), 1)
            surface = surfaces[0]
            self.assertEqual(surface.surface_type, self._fake_orca.host.SurfaceType.stInternal)

            expoly = surface.expolygon
            self.assertTrue(expoly.is_valid())
            self.assertEqual(expoly.num_contours(), 1)
            desc = fake_orca.describe(expoly)
            self.assertEqual(desc["kind"], "polygon_notched")

    def test_report_counts_are_internally_consistent(self):
        self.assertTrue(os.path.exists(self.report_path))
        report = self.read_report()
        self.assertEqual(report["layers_seen"], self.N_LAYERS)
        self.assertEqual(report["mutated_count"], self.N_LAYERS)
        self.assertEqual(report["failed_count"], 0)
        self.assertIsNone(report["first_failure"])
        self.assertEqual(len(report["layers"]), self.N_LAYERS)
        self.assertEqual(report["root_layers"], 0)

    def test_cut_reaches_the_local_upper_surface_on_the_cambered_section(self):
        """The whole point: the slot must open the LOCAL skin at the web's own
        chord station, not merely stay inside the section's global bounding
        box. Verified two ways against an INDEPENDENT measurement
        (``aircraft_core.frame.local_span`` over a second,
        separately-instantiated ``GeometryOps`` adapter, never the one the
        capability itself used):

        1. the removed notch's own max_y is at least the local upper
           surface height at its chord station (the cut reached the true
           boundary, not merely "close enough" against a flat bbox
           number); and
        2. that local upper-surface height is well BELOW the section's
           global bounding-box max_y at 0.7 chord — proving this is a
           genuinely local measurement, not a bounding-box formula (at
           0.7 chord the bbox maximum and the true local surface disagree
           by a fraction of a millimetre).
        """
        ops = _IndependentHostOps()
        layer = self.obj.layers()[0]
        # The pre-cut original is gone after mutation; rebuild an identical,
        # untouched one purely for this independent measurement.
        fresh_obj = self.make_airfoil_object(n_layers=1)
        original = self.surfaces_of(fresh_obj.layers()[0])[0].expolygon
        frame = LayerFrame.from_polygon(ops, original)
        x = frame.web_x(self.WEB_POSITION)
        local_y_min, local_y_max = local_span(ops, original, x, DEFAULT_PROBE_WIDTH_MM)

        self.assertLess(
            local_y_max,
            frame.max_y - 0.01,
            "fixture no longer demonstrates the local-vs-bbox gap at 0.7 "
            "chord -- this test would no longer prove the cut is measured "
            "locally.",
        )

        surface = self.surfaces_of(layer)[0]
        desc = fake_orca.describe(surface.expolygon)
        notch_min_x, notch_min_y, notch_max_x, notch_max_y = desc["notch"]
        notch_max_y_mm = notch_max_y * fake_orca.SCALING_FACTOR

        self.assertGreaterEqual(
            notch_max_y_mm,
            local_y_max - 1e-6,
            "the cut's own top edge ({!r} mm) falls short of the "
            "independently-measured local upper surface ({!r} mm) at the "
            "web's chord station -- the slot did not open the local "
            "skin.".format(notch_max_y_mm, local_y_max),
        )


# ---------------------------------------------------------------------------
# 2b. Multiple parallel webs over the real airfoil stack.
# ---------------------------------------------------------------------------


class MultiWebAirfoilIntegrationTests(AirfoilIntegrationTestCase):
    """A multi-web slice over the airfoil stack
    succeeds through the SHIPPED bundle, every body layer gets N slots,
    ``make_slices()`` is called exactly once per mutated layer, and the
    diagnostics report's counts are internally consistent.

    ``rib_pitch_mm=10.0`` anchored at ``web_position=0.5`` on this 60 mm
    chord NACA 2412 fixture places 5 webs (x = -20, -10, 0, 10, 20 mm) with
    2 candidates omitted at the chord extent (x = +/-30 mm) — see
    ``tests/unit/test_generator.py``'s
    ``MultiWebAirfoilGeneratorTests.test_multiple_webs_across_the_real_chord``
    for the same figures established directly against ``aircraft_core``,
    independent of this harness's fake geometry kernel.
    """

    N_LAYERS = 4
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5
    WEB_POSITION = 0.5
    RIB_PITCH_MM = 10.0
    EXPECTED_WEB_COUNT = 5

    def setUp(self):
        super().setUp()
        self.arm()
        self.obj = self.make_airfoil_object(n_layers=self.N_LAYERS)
        ctx = self.make_ctx(self.obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "gap_ratio": self.GAP_RATIO,
                "web_position": self.WEB_POSITION,
                "rib_pitch_mm": self.RIB_PITCH_MM,
            }
        )
        self.result = cap.execute(ctx)

    def test_every_body_layer_gets_n_slots_as_one_circuit(self):
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(self.result.status, PluginResult.Success, self.result.message)
        self.assertIn("{} layers seen".format(self.N_LAYERS), self.result.message)
        self.assertIn("{} slotted".format(self.N_LAYERS), self.result.message)

        for layer in self.obj.layers():
            # make_slices() called exactly once per mutated layer -- never
            # once per slot.
            self.assertEqual(layer.make_slices_calls, 1)

            surfaces = self.surfaces_of(layer)
            self.assertEqual(
                len(surfaces),
                1,
                "N webs must still leave the body layer as ONE boundary "
                "-connected island (no slot crossings, no accidental body "
                "split) -- got {} surfaces.".format(len(surfaces)),
            )
            surface = surfaces[0]
            self.assertEqual(surface.surface_type, self._fake_orca.host.SurfaceType.stInternal)

            expoly = surface.expolygon
            self.assertTrue(expoly.is_valid())
            self.assertEqual(expoly.num_contours(), 1)
            desc = fake_orca.describe(expoly)
            self.assertEqual(desc["kind"], "polygon_multi_notched")
            self.assertEqual(len(desc["notches"]), self.EXPECTED_WEB_COUNT)

    def test_report_counts_are_internally_consistent(self):
        self.assertTrue(os.path.exists(self.report_path))
        report = self.read_report()
        self.assertEqual(report["layers_seen"], self.N_LAYERS)
        self.assertEqual(report["mutated_count"], self.N_LAYERS)
        self.assertEqual(report["failed_count"], 0)
        self.assertIsNone(report["first_failure"])
        self.assertEqual(len(report["layers"]), self.N_LAYERS)

        for layer_record in report["layers"]:
            self.assertTrue(layer_record["mutated"])
            self.assertEqual(layer_record["extra"]["slots"], self.EXPECTED_WEB_COUNT)
            # 2 candidates (the +/-30 mm chord-extent stations) omitted per
            # layer, on every layer of this constant-section stack -- see
            # this class's own docstring for the independently-established
            # figures.
            self.assertEqual(layer_record["extra"]["omitted"], 2)


# ---------------------------------------------------------------------------
# 2b-ii. Z-dependent web placement over the airfoil stack.
# ---------------------------------------------------------------------------


class ZVariationAirfoilIntegrationTests(AirfoilIntegrationTestCase):
    """A Z-varying slice over the airfoil stack
    succeeds through the SHIPPED bundle, every body layer gets its slot,
    ``make_slices()`` is called exactly once per mutated layer, and per-layer
    stations DIFFER between a low and a high layer -- proving the pattern
    actually varies with height, not merely that it still runs.

    ``z_drift_mm_per_mm=2.0`` with no ``rib_pitch_mm`` (single web per
    layer) on this 60 mm chord NACA 2412 fixture, over 6 layers at the
    harness's default 0.2 mm layer height (``print_z`` = 0.2 .. 1.2 mm):
    every layer's single candidate survives (no omissions -- independently
    established directly against ``aircraft_core`` with no fake-host
    geometry involved, mirroring ``MultiWebAirfoilIntegrationTests``'s own
    docstring), and the station drifts by exactly
    ``z_drift_mm_per_mm * delta_print_z`` between any two layers.
    """

    N_LAYERS = 6
    LAYER_HEIGHT_MM = 0.2
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5
    WEB_POSITION = 0.5
    Z_DRIFT_MM_PER_MM = 2.0

    def setUp(self):
        super().setUp()
        self.arm()
        self.obj = self.make_airfoil_object(
            n_layers=self.N_LAYERS, layer_height_mm=self.LAYER_HEIGHT_MM
        )
        ctx = self.make_ctx(self.obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "gap_ratio": self.GAP_RATIO,
                "web_position": self.WEB_POSITION,
                "z_drift_mm_per_mm": self.Z_DRIFT_MM_PER_MM,
            }
        )
        self.result = cap.execute(ctx)

    def _notch_center_x_mm(self, layer):
        surface = self.surfaces_of(layer)[0]
        desc = fake_orca.describe(surface.expolygon)
        self.assertEqual(desc["kind"], "polygon_notched")
        min_x, _min_y, max_x, _max_y = desc["notch"]
        return (min_x + max_x) / 2.0 * fake_orca.SCALING_FACTOR

    def test_every_body_layer_slotted_once_and_result_is_success(self):
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(self.result.status, PluginResult.Success, self.result.message)
        self.assertIn("{} layers seen".format(self.N_LAYERS), self.result.message)
        self.assertIn("{} slotted".format(self.N_LAYERS), self.result.message)
        self.assertIn("0 left solid", self.result.message)

        for layer in self.obj.layers():
            self.assertEqual(layer.make_slices_calls, 1)
            surfaces = self.surfaces_of(layer)
            self.assertEqual(len(surfaces), 1)
            surface = surfaces[0]
            self.assertEqual(surface.surface_type, self._fake_orca.host.SurfaceType.stInternal)
            expoly = surface.expolygon
            self.assertTrue(expoly.is_valid())
            self.assertEqual(expoly.num_contours(), 1)

    def test_report_counts_are_internally_consistent(self):
        self.assertTrue(os.path.exists(self.report_path))
        report = self.read_report()
        self.assertEqual(report["layers_seen"], self.N_LAYERS)
        self.assertEqual(report["mutated_count"], self.N_LAYERS)
        self.assertEqual(report["failed_count"], 0)
        self.assertIsNone(report["first_failure"])
        self.assertEqual(len(report["layers"]), self.N_LAYERS)
        for layer_record in report["layers"]:
            self.assertTrue(layer_record["mutated"])
            self.assertEqual(layer_record["extra"]["slots"], 1)
            self.assertEqual(layer_record["extra"]["omitted"], 0)

    def test_performance_block_still_populates_under_z_variation(self):
        # Z-variation adds a per-family drift computation ahead of the
        # station arithmetic; this confirms the performance block still
        # populates (metrics_enabled defaults True -- see DEFAULT_SPEC in
        # capability.py).
        report = self.read_report()
        performance = report["performance"]
        self.assertIsInstance(
            performance,
            dict,
            "performance block did not populate: {!r}".format(performance),
        )
        self.assertEqual(performance["layers_measured"], self.N_LAYERS)
        self.assertEqual(performance["slots"]["total"], self.N_LAYERS)
        self.assertEqual(len(performance["layers"]), self.N_LAYERS)

    def test_stations_differ_between_low_and_high_layers(self):
        layers = self.obj.layers()
        low_layer = layers[0]
        high_layer = layers[-1]

        low_x_mm = self._notch_center_x_mm(low_layer)
        high_x_mm = self._notch_center_x_mm(high_layer)

        self.assertNotAlmostEqual(
            low_x_mm,
            high_x_mm,
            places=3,
            msg="the web station must vary with Z -- got the same station "
            "at both the lowest and highest layer.",
        )

        expected_delta_mm = self.Z_DRIFT_MM_PER_MM * (
            high_layer.print_z - low_layer.print_z
        )
        self.assertAlmostEqual(
            high_x_mm - low_x_mm, expected_delta_mm, places=2
        )

        # Monotonic across every layer in between too, at this constant
        # positive drift rate -- not merely "different at the two ends".
        xs = [self._notch_center_x_mm(layer) for layer in layers]
        self.assertEqual(xs, sorted(xs))


# ---------------------------------------------------------------------------
# 2c. Web depth + families: a two-tier structure over the airfoil stack.
# ---------------------------------------------------------------------------


class TwoTierFamilyIntegrationTests(AirfoilIntegrationTestCase):
    """A two-tier structure (shallow skin webs + deep ribs, two DIFFERENT
    families with different pitch, depth and phase) succeeds through the
    SHIPPED bundle over the real NACA 2412 stack: every body layer gets
    both families' slots, ``make_slices()`` is called exactly once per
    mutated layer, and the diagnostics report's per-family counts are
    internally consistent.

    Figures independently established directly against ``aircraft_core``
    (no fake-host geometry involved) in
    ``tests/unit/test_generator.py``-style analysis: on this 60 mm chord
    fixture, a "shallow skin" family (0.5 mm deep, 4 mm pitch, anchored at
    the centreline) produces 14 webs with 1 omitted (the +28 mm station is
    too thin locally for even the 1.0 mm skin_margin_mm, let alone the
    extra 0.5 mm depth), and a "deep rib" family (full depth, 12 mm pitch,
    phase-shifted so no station of either family collides with the other --
    the point of web depth/families) produces 5
    webs. The phase offset is not arbitrary in TWO ways: (1) pitches and
    phases that put a shallow and a deep station at (or near) the same
    chord station are exactly the cross-family collision
    ``test_validate.py``'s
    ``test_cross_family_collision_shallow_and_deep_at_same_station`` covers
    directly, and (2) this fake host's simplified single-point (not
    worst-case-across-span) local-surface probe near the chord extremes
    diverges from ``aircraft_core``'s own worst-case measurement on this
    fixture's highest-curvature region (see ``fake_orca.py``'s "KNOWN
    DIVERGENCES") -- so deep-family stations are additionally kept a couple
    of millimetres clear of the chord extremes here, a fake-host-harness
    constraint, not a product one (the real Clipper-backed host has no such
    divergence; see ``validate.py``'s own note on why the real removed
    region, not a single-station probe, is what invariants 3/4 measure
    against). This fixture is deliberately chosen to be the SUCCEEDING
    case.
    """

    N_LAYERS = 3
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5
    SHALLOW_PITCH_MM = 4.0
    SHALLOW_DEPTH_MM = 0.5
    SHALLOW_POSITION = 0.5
    DEEP_PITCH_MM = 12.0
    DEEP_POSITION = 0.52
    EXPECTED_SHALLOW_COUNT = 14
    EXPECTED_DEEP_COUNT = 5
    EXPECTED_OMITTED = 1  # the shallow family's +28 mm station only

    def setUp(self):
        super().setUp()
        self.arm()
        self.obj = self.make_airfoil_object(n_layers=self.N_LAYERS)
        ctx = self.make_ctx(self.obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "gap_ratio": self.GAP_RATIO,
                "families": [
                    {
                        "name": "shallow skin",
                        "rib_pitch_mm": self.SHALLOW_PITCH_MM,
                        "web_depth_mm": self.SHALLOW_DEPTH_MM,
                        "web_position": self.SHALLOW_POSITION,
                        "gap_ratio": self.GAP_RATIO,
                    },
                    {
                        "name": "deep rib",
                        "rib_pitch_mm": self.DEEP_PITCH_MM,
                        "web_position": self.DEEP_POSITION,
                        "gap_ratio": self.GAP_RATIO,
                    },
                ],
            }
        )
        self.result = cap.execute(ctx)

    def test_every_body_layer_gets_both_families_slots(self):
        PluginResult = self._fake_orca.PluginResult
        expected_total = self.EXPECTED_SHALLOW_COUNT + self.EXPECTED_DEEP_COUNT
        self.assertEqual(self.result.status, PluginResult.Success, self.result.message)
        self.assertIn("{} layers seen".format(self.N_LAYERS), self.result.message)
        self.assertIn("{} slotted".format(self.N_LAYERS), self.result.message)

        for layer in self.obj.layers():
            # make_slices() called exactly once per mutated layer, no
            # matter how many families/slots it carries.
            self.assertEqual(layer.make_slices_calls, 1)

            surfaces = self.surfaces_of(layer)
            self.assertEqual(
                len(surfaces),
                1,
                "two families must still leave the body layer as ONE "
                "boundary-connected island -- got {} surfaces.".format(
                    len(surfaces)
                ),
            )
            surface = surfaces[0]
            self.assertEqual(
                surface.surface_type, self._fake_orca.host.SurfaceType.stInternal
            )

            expoly = surface.expolygon
            self.assertTrue(expoly.is_valid())
            self.assertEqual(expoly.num_contours(), 1)
            desc = fake_orca.describe(expoly)
            self.assertEqual(desc["kind"], "polygon_multi_notched")
            self.assertEqual(len(desc["notches"]), expected_total)

    def test_report_counts_are_internally_consistent_per_family(self):
        self.assertTrue(os.path.exists(self.report_path))
        report = self.read_report()
        self.assertEqual(report["layers_seen"], self.N_LAYERS)
        self.assertEqual(report["mutated_count"], self.N_LAYERS)
        self.assertEqual(report["failed_count"], 0)
        self.assertIsNone(report["first_failure"])
        self.assertEqual(len(report["layers"]), self.N_LAYERS)

        expected_total = self.EXPECTED_SHALLOW_COUNT + self.EXPECTED_DEEP_COUNT
        for layer_record in report["layers"]:
            self.assertTrue(layer_record["mutated"])
            extra = layer_record["extra"]
            self.assertEqual(extra["slots"], expected_total)
            self.assertEqual(extra["omitted"], self.EXPECTED_OMITTED)

            families = extra["families"]
            shallow_key = next(k for k in families if k.endswith("shallow skin"))
            deep_key = next(k for k in families if k.endswith("deep rib"))
            self.assertEqual(families[shallow_key]["slots"], self.EXPECTED_SHALLOW_COUNT)
            self.assertEqual(families[shallow_key]["omitted"], self.EXPECTED_OMITTED)
            self.assertEqual(families[deep_key]["slots"], self.EXPECTED_DEEP_COUNT)
            self.assertEqual(families[deep_key]["omitted"], 0)

            # Every families_breakdown entry the REAL bundle produces
            # (through fake_orca, not a hand-built RunDiagnostics extra)
            # carries a "role". Neither family here sets one, so both resolve
            # to the untagged default (skin_lattice), never inferred from
            # web_depth_mm (the "deep rib" family is full-depth and must NOT
            # silently read as primary_structure).
            self.assertEqual(families[shallow_key]["role"], "skin_lattice")
            self.assertEqual(families[deep_key]["role"], "skin_lattice")


# ---------------------------------------------------------------------------
# 2d. A crossing (differing-depth) junction merges into one web through the
# SHIPPED bundle.
# ---------------------------------------------------------------------------


class DiamondJunctionIntegrationTests(AirfoilIntegrationTestCase):
    """Two families placed close enough together that their footprints come
    within the required minimum wall (``2 * outer_wall_line_width``) of
    each other, with DIFFERING depths (a full-depth rib and a shallow skin
    web) and non-overlapping footprints. Run through the real capability
    control flow (frame -> generator -> slot -> diff -> validate -> set ->
    make_slices), over the SHIPPED bundle and a real NACA 2412 section, not
    a hand-built validator fixture.

    Such a close cross-family pair MERGES: ``StructureGenerator`` builds ONE
    ``WebIntent`` spanning the union of both families' footprints, at the
    shallower depth ("shallow skin"'s 0.5 mm, not "deep rib"'s full depth),
    and both families survive, merged, rather than one being dropped. This
    fixture uses a 0.3 mm gap (< 0.84 mm minimum wall, differing depths, not
    overlapping). See ``aircraft_core/generator.py``'s
    ``_merge_cross_family_nodes`` for the mechanism.

    The "fails closed naming both families" diagnostic still needs an
    integration-level guard, because a regression that stops the diagnostic
    naming families would otherwise go uncaught at this level. That guard
    lives in ``SameFamilyProximityIntegrationTests`` below, on a case the
    merge never touches (same-family proximity: the merge's component-linking
    step only compares DIFFERENT family indices; see
    ``_merge_cross_family_nodes``'s "if a.family_index == b.family_index:
    continue"). That sibling test does not reproduce the "different
    families" wording (a same-family failure produces the plain collision
    message); see its docstring for why a cross-family case that also fails
    closed end to end cannot be constructed here.

    FAKE-HOST LIMIT: this fake host's polygon kernel (``fake_orca.py``) can
    only build TWO notches on the same section when their X ranges do NOT
    overlap (see that module's KNOWN DIVERGENCES (17)); a truly OVERLAPPING
    junction on the curved airfoil boundary needs the real Clipper-backed
    host. This fixture therefore keeps the two footprints close but
    non-overlapping (a near-tangent approach), which is enough to trip the
    merge band without asking this fake to build geometry it cannot
    represent (a merged intent is still exactly ONE slot handed to the same
    diff/validate path a single-family web uses). The equal-depth merge
    case is exercised end to end on the rectangular coupon instead (see
    ``tests/unit/test_junction.py``'s ``DiamondGeneratorTests``), for the
    same reason.
    """

    N_LAYERS = 1
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5
    DEEP_POSITION = 0.5
    TARGET_GAP_MM = 0.3  # < 0.84 mm (2 * 0.42 mm outer wall), > 0.

    def setUp(self):
        super().setUp()
        # Independently derive a shallow-family position that places its
        # footprint TARGET_GAP_MM from the deep family's -- not a number
        # pulled out of the air, and not dependent on the capability under
        # test.
        ops = _IndependentHostOps()
        probe_obj = self.make_airfoil_object(n_layers=1)
        original = self.surfaces_of(probe_obj.layers()[0])[0].expolygon
        frame = LayerFrame.from_polygon(ops, original)
        width_mm = self.GAP_RATIO * OUTER_WALL_MM
        x_deep = frame.web_x(self.DEEP_POSITION)
        x_shallow = x_deep + width_mm / 2.0 + width_mm / 2.0 + self.TARGET_GAP_MM
        self.shallow_position = (x_shallow - frame.min_x) / frame.width
        self.actual_gap_mm = (x_shallow - width_mm / 2.0) - (x_deep + width_mm / 2.0)
        self.assertLess(self.actual_gap_mm, 0.84)
        self.assertGreater(self.actual_gap_mm, 0.0)

        self.arm()
        self.obj = self.make_airfoil_object(n_layers=self.N_LAYERS)
        ctx = self.make_ctx(self.obj)
        self.cap = self.new_capability()
        self.cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "gap_ratio": self.GAP_RATIO,
                "families": [
                    {
                        "name": "deep rib",
                        "web_position": self.DEEP_POSITION,
                        "gap_ratio": self.GAP_RATIO,
                    },
                    {
                        "name": "shallow skin",
                        "web_depth_mm": 0.5,
                        "web_position": self.shallow_position,
                        "gap_ratio": self.GAP_RATIO,
                    },
                ],
            }
        )
        self.result = self.cap.execute(ctx)

    def test_close_differing_depth_junction_merges_both_survive_per_d012(self):
        # The close differing-depth pair merges into one WebIntent (neither
        # family is dropped). See this class's docstring.
        PluginResult = self._fake_orca.PluginResult
        # Still no FatalError -- the layer slices clean too.
        self.assertEqual(self.result.status, PluginResult.Success, self.result.message)

        layer = self.obj.layers()[0]
        self.assertEqual(layer.make_slices_calls, 1)
        surfaces = self.surfaces_of(layer)
        self.assertEqual(len(surfaces), 1)

        # A true merge is RECORDED, not silent -- both at the per-layer
        # diagnostic capture (``cap._last_geometry``, the same structure the
        # failure path's ``geometry_at_failure`` uses) and in the written run
        # report's aggregate count. But a merge produces NO omission (no
        # family is dropped) -- the evidence here is the merged WebIntent's
        # own ``merged_family_indices``, not an OmittedWeb entry.
        geometry = self.cap._last_geometry
        self.assertIsNotNone(geometry)
        omitted_reasons = {
            (o["family_name"], o["reason"]) for o in geometry["omitted"]
        }
        self.assertEqual(
            omitted_reasons,
            set(),
            "a true merge must not produce any omission",
        )
        self.assertFalse(
            any(
                reason.startswith("node suppression:")
                for _name, reason in omitted_reasons
            )
        )

        # Exactly one web actually reached the boolean cut: the merged
        # intent, carrying both family indices via merged_family_indices.
        self.assertEqual(len(geometry["intents"]), 1)
        merged = geometry["intents"][0]
        self.assertEqual(sorted(merged["merged_family_indices"]), [0, 1])
        # Width is the union-interval width -- strictly wider than either
        # family's own requested slot on its own (gap_ratio 0.5 * 0.42 mm
        # wall = 0.21 mm per family).
        self.assertGreater(merged["width_mm"], 0.21)

        report = self.read_report()
        self.assertEqual(report["node_merge_count"], 1)
        self.assertEqual(report["first_node_merge_z"], layer.print_z)


class SameFamilyProximityIntegrationTests(AirfoilIntegrationTestCase):
    """Integration-level "fails closed, naming the offending pair" guard for
    a same-family collision, which the cross-family merge in
    ``DiamondJunctionIntegrationTests`` does not rescue.

    WHY SAME-FAMILY PROXIMITY, AND NOT A CROSS-FAMILY CASE:
    ``_merge_cross_family_nodes`` skips any pair sharing a ``family_index``,
    so same-family proximity is a plain placement mistake that still reaches
    ``TopologyValidator``'s collision check and fails closed. A cross-family
    case that still fails closed end to end cannot be constructed with
    confidence: the merge re-scans after every merge until no adjacent
    cross-family pair remains inside the unprintable band, so it resolves
    every cross-family sub-minimum-wall pair the GENERATOR's requested-
    footprint check can see. The one seam that could still produce an
    unresolved cross-family failure -- the generator merging on REQUESTED
    footprints while the validator re-measures MEASURED (post-cut)
    footprints, which can disagree at the band edge on a curved boundary --
    is not constructible with either test double (see
    ``tests/unit/test_node_merge_scope.py``'s module docstring):
    ``stub_ops.py``'s notch shapes and ``fake_orca.py``'s kernel both store
    the REQUESTED X bounds verbatim (never reclipped in X). A cross-family
    case would need the real Clipper-backed host.

    This test therefore does NOT exercise the "different families" message
    wording; a same-family failure produces the plain collision message
    (``test_close_same_family_slots_still_fail_even_with_family_indices_given``
    pins that message's exact text at the validator level). What this test
    adds is the same failure reached through the real capability control
    flow end to end (frame -> generator -> slot -> diff -> validate), over
    the shipped bundle and a real NACA 2412 section, proving the merge
    leaves a same-family collision alone.
    """

    N_LAYERS = 1
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5
    ANCHOR_POSITION = 0.5
    TARGET_GAP_MM = 0.3  # < 0.84 mm (2 * 0.42 mm outer wall), > 0.

    def setUp(self):
        super().setUp()
        # Independently derive a rib_pitch that places this ONE family's
        # own next marched station TARGET_GAP_MM from its anchor -- same
        # measurement discipline as DiamondJunctionIntegrationTests, just
        # within a single family instead of across two.
        ops = _IndependentHostOps()
        probe_obj = self.make_airfoil_object(n_layers=1)
        original = self.surfaces_of(probe_obj.layers()[0])[0].expolygon
        frame = LayerFrame.from_polygon(ops, original)
        width_mm = self.GAP_RATIO * OUTER_WALL_MM
        x_anchor = frame.web_x(self.ANCHOR_POSITION)
        x_next = x_anchor + width_mm / 2.0 + width_mm / 2.0 + self.TARGET_GAP_MM
        self.rib_pitch_mm = x_next - x_anchor
        self.actual_gap_mm = (x_next - width_mm / 2.0) - (x_anchor + width_mm / 2.0)
        self.assertLess(self.actual_gap_mm, 0.84)
        self.assertGreater(self.actual_gap_mm, 0.0)

        self.arm()
        self.obj = self.make_airfoil_object(n_layers=self.N_LAYERS)
        ctx = self.make_ctx(self.obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "gap_ratio": self.GAP_RATIO,
                "web_position": self.ANCHOR_POSITION,
                "rib_pitch_mm": self.rib_pitch_mm,
            }
        )
        self.result = cap.execute(ctx)

    def test_close_same_family_slots_still_fail_closed_through_real_capability(self):
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(self.result.status, PluginResult.FatalError, self.result.message)
        self.assertIn("Aircraft structure invalid at layer 0", self.result.message)
        # The plain collision message -- NOT the cross-family "different
        # families" wording, and NOT merged (same family_index on both
        # sides).
        self.assertIn("would merge into one slot or sever", self.result.message)
        self.assertNotIn("different families", self.result.message)

        # Fail closed regardless: the layer's surfaces are untouched.
        layer = self.obj.layers()[0]
        self.assertEqual(layer.make_slices_calls, 0)
        surfaces = self.surfaces_of(layer)
        self.assertEqual(len(surfaces), 1)
        self.assertEqual(fake_orca.describe(surfaces[0].expolygon)["kind"], "polygon")


# ---------------------------------------------------------------------------
# 3. Fail closed near the thin trailing edge.
# ---------------------------------------------------------------------------


class ThinTrailingEdgeTests(AirfoilIntegrationTestCase):
    """A web placed close to the trailing edge (``web_position=0.95``): at
    that chord station this NACA 2412 section is about 0.85 mm thick (see
    ``test_thin_trailing_edge_web_now_DEGRADES`` for the measurement), well
    under the 1.0 mm ``skin_margin_mm`` this test uses -- there is not enough
    local material for the requested margin to fit at all.

    ``StructureGenerator.generate()`` measures each candidate station itself
    and, when the margin cannot fit, records a deterministic ``OmittedWeb``
    (see ``aircraft_core/generator.py``'s "OMISSION POLICY") rather than
    raising. With no ``rib_pitch_mm`` set, ``web_position=0.95`` is the ONLY
    candidate station, so zero webs survive generation. ``capability.py``
    then DEGRADES that layer: it is left unmutated and reported, and the run
    still succeeds, because a section that ran out of material is not a
    geometric anomaly.
    """

    N_LAYERS = 1
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5
    WEB_POSITION = 0.95

    def test_thin_trailing_edge_web_now_DEGRADES(self):
        # Independent measurement of the true local thickness at this chord
        # station, confirming the fixture actually demonstrates a section
        # too thin for the requested margin -- not asserting against a
        # number pulled out of the air.
        ops = _IndependentHostOps()
        probe_obj = self.make_airfoil_object(n_layers=1)
        original = self.surfaces_of(probe_obj.layers()[0])[0].expolygon
        frame = LayerFrame.from_polygon(ops, original)
        x = frame.web_x(self.WEB_POSITION)
        local_y_min, local_y_max = local_span(ops, original, x, DEFAULT_PROBE_WIDTH_MM)
        local_thickness = local_y_max - local_y_min
        self.assertLess(
            local_thickness,
            self.SKIN_MARGIN_MM,
            "fixture no longer demonstrates a trailing-edge station thinner "
            "than skin_margin_mm -- this test would no longer exercise the "
            "thin-section failure path.",
        )

        self.arm()
        obj = self.make_airfoil_object(n_layers=self.N_LAYERS)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "gap_ratio": self.GAP_RATIO,
                "web_position": self.WEB_POSITION,
            }
        )

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        # A layer whose every candidate web was omitted is DEGRADED (left
        # unmutated, reported) rather than refusing the slice. The thin
        # trailing edge is the canonical case -- a section that ran out of
        # material, not a geometric anomaly.
        self.assertEqual(result.status, PluginResult.Success, result.message)
        self.assertIn("DEGRADED", result.message)
        # What the message DOES contain: it is legible -- it names the
        # omission reason (the "skin margin" category, from
        # aircraft_core/generator.py) and the chord station.
        self.assertIn("pattern_all_webs_omitted", result.message)
        # Cause detail lives on the layer's degradation record, not in the
        # summary line (which carries count and kind).
        reasons = " ".join(
            d["reason"]
            for lr in self.read_report()["layers"]
            for d in lr.get("degradations", [])
        )
        self.assertTrue(
            "skin margin" in reasons or "thinness" in reasons, reasons)
        self.assertIn(
            "skin_margin_mm={:.4f}".format(self.SKIN_MARGIN_MM), reasons
        )
        self.assertIn("x=", reasons)

        # Traceability to a layer is carried by the degradation RECORD: the
        # layer index and Z are on the LayerRecord that carries it, and the
        # summary names the first degraded Z. (A degraded layer is not a
        # TopologyError, so there is no "Aircraft structure invalid at layer
        # N" prefix.)
        first = [
            lr for lr in self.read_report()["layers"] if lr.get("degradations")
        ]
        self.assertEqual(len(first), 1)
        self.assertEqual(first[0]["layer_index"], 0)
        self.assertAlmostEqual(first[0]["print_z"], 0.20, places=4)
        self.assertIn("Z=0.20 mm", result.message)

        # Unmutated regardless: the layer's surfaces are untouched.
        layer = obj.layers()[0]
        self.assertEqual(layer.make_slices_calls, 0)
        surfaces = self.surfaces_of(layer)
        self.assertEqual(len(surfaces), 1)
        self.assertEqual(fake_orca.describe(surfaces[0].expolygon)["kind"], "polygon")

        # A diagnostics report is still written, and it is still exactly where
        # the per-layer context lives -- but this run SUCCEEDS, so there is no
        # ``error`` key and no first_failure. The equivalent record is the
        # degraded layer.
        self.assertTrue(os.path.exists(self.report_path))
        report = self.read_report()
        self.assertNotIn("error", report)
        self.assertIsNone(report["first_failure"])
        self.assertEqual(report["degraded_layer_count"], 1)
        degraded = [
            lr for lr in report["layers"] if lr.get("degradations")
        ]
        self.assertEqual(len(degraded), 1)
        self.assertEqual(degraded[0]["layer_index"], 0)
        self.assertFalse(degraded[0]["failed"])
        self.assertFalse(degraded[0]["mutated"])


# ---------------------------------------------------------------------------
# 4. Not armed.
# ---------------------------------------------------------------------------


class NotArmedAirfoilTests(AirfoilIntegrationTestCase):
    def test_not_armed_is_success_and_mutates_nothing(self):
        obj = self.make_airfoil_object(n_layers=4)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success)
        self.assertIn("m1_enabled", result.message)
        self.assertIn("not armed", result.message)

        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 0)
            surfaces = self.surfaces_of(layer)
            self.assertEqual(len(surfaces), 1)
            self.assertEqual(fake_orca.describe(surfaces[0].expolygon)["kind"], "polygon")

        self.assertFalse(os.path.exists(self.report_path))


# ---------------------------------------------------------------------------
# 5. Hole visibility through fake_orca's ExPolygon, read via
#    _IndependentHostOps (the same reading pattern OrcaGeometryOps.
#    hole_bounds/hole_points_mm uses against the real host: .holes ->
#    .points -> scale-divide). This does not go through the generated
#    bundle/capability at all (no boolean op ever produces a hole; see
#    fake_orca.py's divergence (3)) -- it only proves a holed ExPolygon can
#    be CONSTRUCTED and its hole geometry READ, the same two things
#    capability.py's own OrcaGeometryOps.hole_bounds/hole_points_mm would do
#    against a real holed section.
# ---------------------------------------------------------------------------


def _rect_polygon(min_x_mm, min_y_mm, max_x_mm, max_y_mm, scale):
    contour = fake_orca.Polygon()
    for x_mm, y_mm in (
        (min_x_mm, min_y_mm),
        (max_x_mm, min_y_mm),
        (max_x_mm, max_y_mm),
        (min_x_mm, max_y_mm),
    ):
        contour.append(
            fake_orca.Point(int(round(x_mm * scale)), int(round(y_mm * scale)))
        )
    return contour


class FakeHoledExPolygonTests(unittest.TestCase):
    """fake_orca's ExPolygon can represent one rectangular interior hole,
    exercised directly with no bundle/capability involved."""

    def setUp(self):
        self.scale = 1.0 / fake_orca.SCALING_FACTOR
        self.ops = _IndependentHostOps()

    def test_hole_free_expolygon_reports_no_holes(self):
        outer = _rect_polygon(0.0, 0.0, 40.0, 20.0, self.scale)
        expoly = fake_orca.ExPolygon(outer, [])

        self.assertEqual(expoly.num_contours(), 1)
        self.assertEqual(self.ops.hole_bounds(expoly), [])
        self.assertEqual(self.ops.hole_points_mm(expoly), [])

    def test_holed_expolygon_reports_one_hole_with_correct_bounds(self):
        outer = _rect_polygon(0.0, 0.0, 40.0, 20.0, self.scale)
        hole = _rect_polygon(16.0, 7.0, 24.0, 13.0, self.scale)
        expoly = fake_orca.ExPolygon(outer, [hole])

        self.assertEqual(expoly.num_contours(), 2)

        bounds = self.ops.hole_bounds(expoly)
        self.assertEqual(len(bounds), 1)
        min_x, min_y, max_x, max_y = bounds[0]
        self.assertAlmostEqual(min_x, 16.0, places=6)
        self.assertAlmostEqual(min_y, 7.0, places=6)
        self.assertAlmostEqual(max_x, 24.0, places=6)
        self.assertAlmostEqual(max_y, 13.0, places=6)

        points = self.ops.hole_points_mm(expoly)
        self.assertEqual(len(points), 1)
        self.assertEqual(
            {(round(x, 6), round(y, 6)) for x, y in points[0]},
            {(16.0, 7.0), (24.0, 7.0), (24.0, 13.0), (16.0, 13.0)},
        )

    def test_outer_bounds_unaffected_by_the_hole(self):
        outer = _rect_polygon(0.0, 0.0, 40.0, 20.0, self.scale)
        hole = _rect_polygon(16.0, 7.0, 24.0, 13.0, self.scale)
        expoly = fake_orca.ExPolygon(outer, [hole])

        min_x, min_y, max_x, max_y = self.ops.bounds(expoly)
        self.assertAlmostEqual(min_x, 0.0, places=6)
        self.assertAlmostEqual(min_y, 0.0, places=6)
        self.assertAlmostEqual(max_x, 40.0, places=6)
        self.assertAlmostEqual(max_y, 20.0, places=6)


if __name__ == "__main__":
    unittest.main()
