"""Integration tests for pattern shutdown, run against the SHIPPED generated
bundle (plugin/aircraft_structures/aircraft_structures.py) exec'd through
fake_orca.py's fake Orca host — never capability.py directly, per the same
discipline test_m1_capability.py / test_airfoil_capability.py use.

Covers:
  1. Shutdown enabled + every omission on a layer is a thinness reason ->
     success, layer left unmutated, recorded as a shutdown layer.
  2. Shutdown does not mask a genuine validator-level disconnection
     (a DIFFERENT failure mode than "every candidate omitted") — still
     fails closed with the SAME message as when shutdown is disabled.
  3. Shutdown does not mask a MIXED-reason all-omitted layer (chord extent
     + skin margin) — still fails closed, even with shutdown enabled.
  4. The tip-closure fixture (tests/fixtures/tip/README.md): a linearly
     tapering NACA 2412 wing, root chord 60 mm down to tip chord 3 mm over
     60 mm of span. With shutdown enabled: structure at the root, shutdown
     near the tip, first shutdown Z close to the analytically predicted
     54.3842 mm. With shutdown disabled: the same stack fails closed.

Nothing here modifies the code under test — it only reads it. The tapered
fixture comes from ``fake_orca.make_tapered_naca_object``, which reuses
``tools/make_fixtures.py``'s own ``build_loft_stations`` — the same code
``make_tip`` itself calls — so there is no second taper implementation.
"""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fake_orca  # noqa: E402


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
LAYER_HEIGHT_MM = FULL_CONFIG["layer_height"]

# Predicted analytically, tests/fixtures/tip/README.md: local max thickness
# crosses skin_margin_mm=1.0 at chord=8.3350 mm, Z=54.3842 mm (90.64% of a
# 60 mm span).
PREDICTED_SHUTDOWN_Z_MM = 54.3842


class TipShutdownTestCase(unittest.TestCase):
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

    def make_ctx(self, obj, config=None):
        Step = self._fake_orca.slicing.Step
        return self._fake_orca.slicing.SlicingPipelineContext(
            step=Step.posSlice,
            obj=obj,
            config=dict(FULL_CONFIG if config is None else config),
            cancelled_fn=None,
        )

    def read_report(self):
        with open(self.report_path, "r", encoding="utf-8") as handle:
            return json.load(handle)

    def surfaces_of(self, layer):
        return layer.regions()[0].slices.surfaces


# ---------------------------------------------------------------------------
# 1. All-thin omissions: shutdown enabled succeeds, and disabled degrades the
#    layer the same way (the flag is inert for this case).
# ---------------------------------------------------------------------------


class AllThinShutdownTests(TipShutdownTestCase):
    """Reuses test_airfoil_capability.py's ThinTrailingEdgeTests fixture
    exactly (web_position=0.95 on a constant 60 mm chord NACA 2412 — about
    0.85 mm local thickness, below the 1.0 mm skin_margin_mm requested):
    that scenario is the "every candidate omitted for a thinness reason"
    case (skin margin), here run with shutdown enabled."""

    N_LAYERS = 3
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5
    WEB_POSITION = 0.95

    def _config(self, tip_shutdown_enabled):
        return json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "gap_ratio": self.GAP_RATIO,
                "web_position": self.WEB_POSITION,
                "tip_shutdown_enabled": tip_shutdown_enabled,
            }
        )

    def test_shutdown_enabled_succeeds_and_leaves_layers_unmutated(self):
        self.arm()
        obj, _bbox = fake_orca.make_airfoil_object("wing", self.N_LAYERS, LAYER_HEIGHT_MM)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: self._config(tip_shutdown_enabled=True)

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)
        self.assertIn("shut down", result.message)

        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 0)
            surfaces = self.surfaces_of(layer)
            self.assertEqual(len(surfaces), 1)
            # Untouched: still the plain, un-notched section.
            self.assertEqual(fake_orca.describe(surfaces[0].expolygon)["kind"], "polygon")

        report = self.read_report()
        self.assertEqual(report["shutdown_count"], self.N_LAYERS)
        self.assertEqual(report["failed_count"], 0)
        self.assertIsNone(report["first_failure"])
        self.assertAlmostEqual(report["first_shutdown_z"], LAYER_HEIGHT_MM, places=6)
        first_layer_record = report["layers"][0]
        self.assertTrue(first_layer_record["shutdown"])
        self.assertFalse(first_layer_record["mutated"])
        self.assertFalse(first_layer_record["failed"])
        self.assertEqual(first_layer_record["extra"]["reason"], "tip_shutdown")
        self.assertIn("skin margin:", first_layer_record["extra"]["omitted"][0]["reason"])

    def test_shutdown_disabled_now_DEGRADES_the_flag_is_inert(self):
        """Same fixture and omission as the shutdown-enabled test, shutdown
        OFF: the layer is still DEGRADED (not refused), so the flag does not
        change the outcome."""
        self.arm()
        obj, _bbox = fake_orca.make_airfoil_object("wing", self.N_LAYERS, LAYER_HEIGHT_MM)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: self._config(tip_shutdown_enabled=False)

        result = cap.execute(ctx)

        # A layer with no surviving web is DEGRADED by default, whatever the
        # cause and whatever this flag says, so the flag does not change this
        # outcome and there is no hint to give. The layer prints as plain skin
        # and says so. (Refusing here would also be wrong at a thinning tip:
        # lattice omitted for thinness plus a dropped spar socket leaves zero
        # intents, and a refusal message about WEBS would misreport a socket
        # failure.)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)
        self.assertIn("DEGRADED", result.message)

        # Still unmutated -- degrading here means "no structure on this
        # layer", not "structure cut badly".
        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 0)


# ---------------------------------------------------------------------------
# 2. Shutdown must not mask a genuine validator-level disconnection.
# ---------------------------------------------------------------------------


class ShutdownDoesNotMaskDisconnectionTests(TipShutdownTestCase):
    """test_m1_capability.py's FailClosedDisconnectionTests scenario
    (skin_margin_mm=0.1, well below 2x the safety offset).

    ``StructureSpec.validate(safety_offset_mm=...)`` fires from
    ``capability.py``'s ``_build_spec``, before any layer is walked, so this
    misconfiguration is a ``SpecError`` at config-build time, before a web is
    ever placed. The test pins that enabling shutdown must not mask this
    failure: ``tip_shutdown_enabled`` only affects the "every candidate
    omitted for thinness" path inside the per-layer walk, which this
    misconfiguration never reaches, so the assertion below (identical to
    ``FailClosedDisconnectionTests``' own) shows shutdown changes nothing
    about it.
    """

    SKIN_MARGIN_MM = 0.1
    OUTER_WALL_MM = 0.42

    def test_thin_bridge_still_raises_fatal_error_with_shutdown_enabled(self):
        self.arm()
        obj = fake_orca.make_rect_object("coupon", 1, 20.0, 20.0, LAYER_HEIGHT_MM)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "tip_shutdown_enabled": True,
            }
        )

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        # STAYS FATAL, deliberately. A validator-level disconnection means the
        # boolean produced something the pre-checks said it would not (a bug in
        # our own geometry), so it must refuse rather than degrade. Only
        # unplaceable-feature refusals are converted to degradations.
        self.assertEqual(result.status, PluginResult.FatalError)
        layer0 = obj.layers()[0]
        expected_message = (
            "aircraft structures configuration invalid: "
            "skin_margin_mm={!r} does not exceed 2 * safety_offset_mm={!r} "
            "({!r} mm minimum wall).".format(
                self.SKIN_MARGIN_MM, self.OUTER_WALL_MM, 2.0 * self.OUTER_WALL_MM
            )
        )
        # Byte-identical to FailClosedDisconnectionTests' own assertion —
        # shutdown being enabled changes NOTHING about this message.
        self.assertTrue(result.message.startswith(expected_message))
        self.assertEqual(layer0.make_slices_calls, 0)


# ---------------------------------------------------------------------------
# 3. Shutdown must not mask a MIXED-reason all-omitted layer.
# ---------------------------------------------------------------------------


class ShutdownDoesNotMaskMixedReasonsTests(TipShutdownTestCase):
    """Two families, both omitted, for DIFFERENT reasons: one right at the
    trailing edge tip (web_position=1.0 -> "chord extent", a positional
    miss, never a thinness finding) and one just short of it
    (web_position=0.95 -> "skin margin", per ThinTrailingEdgeTests). Every
    candidate is still omitted (zero intents), but NOT every omission is a
    thinness reason, so shutdown must not apply even though it is
    enabled."""

    N_LAYERS = 1
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5

    def test_mixed_cause_omissions_also_DEGRADE_and_name_every_cause(self):
        self.arm()
        obj, _bbox = fake_orca.make_airfoil_object("wing", self.N_LAYERS, LAYER_HEIGHT_MM)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "tip_shutdown_enabled": True,
                "families": [
                    {"name": "at_te_tip", "gap_ratio": self.GAP_RATIO, "web_position": 1.0},
                    {"name": "near_te", "gap_ratio": self.GAP_RATIO, "web_position": 0.95},
                ],
            }
        )

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)
        self.assertIn("DEGRADED", result.message)
        self.assertIn("pattern_all_webs_omitted", result.message)
        # The per-omission causes are not in the SUMMARY line -- it
        # carries the count and the kind. The detail lives on the layer's
        # own degradation record, which is where a reader chasing a
        # specific station will look.
        reasons = " ".join(
            d["reason"]
            for lr in self.read_report()["layers"]
            for d in lr.get("degradations", [])
        )
        self.assertIn("chord extent", reasons)
        self.assertIn("skin margin", reasons)
        # The all-thin hint must NOT appear -- this is not a thinness-only
        # omission set, so shutdown was never eligible regardless of the
        # flag.
        self.assertNotIn("tip_shutdown_enabled=true", result.message)

        layer0 = obj.layers()[0]
        self.assertEqual(layer0.make_slices_calls, 0)
        report = self.read_report()
        # this one-layer fixture's single layer is DEGRADED (left unmutated,
        # reported) rather than refused. Mixed-cause omissions degrade like any
        # other unplaceable feature; the flag distinguishes only the CAUSES
        # recorded, not the outcome.
        self.assertEqual(report["shutdown_count"], 1)
        self.assertEqual(report["degraded_layer_count"], 1)
        self.assertEqual(report["failed_count"], 0)


# ---------------------------------------------------------------------------
# 4. The tip-closure fixture: structure at the root, shutdown near the
#    tip, first shutdown Z close to the analytically predicted 54.3842 mm.
# ---------------------------------------------------------------------------


class TipFixtureShutdownTests(TipShutdownTestCase):
    """tests/fixtures/tip/README.md's geometry (root chord 60 mm, tip
    chord 3 mm, 60 mm span, NACA 2412), built per-layer via
    fake_orca.make_tapered_naca_object -- the same taper math
    tools/make_fixtures.py's own make_tip() uses for the shipped STL, just
    consumed per print-layer here instead of as a 3D loft mesh.

    N_LAYERS=290 (Z up to 58.0 mm) rather than the full 300 (Z=60 mm): the last
    ~2 mm of chord (tip_chord=3 mm down to 0) pushes the fake host's
    Sutherland-Hodgman-based geometry stub into shapes this integration
    harness's DOCUMENTED SCOPE does not model (fake_orca.py's module
    docstring) -- unrelated to the capability, and well past the region this
    test needs to bracket (54.3842 mm).

    ``web_position=0.3`` because that is where ``PREDICTED_SHUTDOWN_Z_MM``
    (54.3842 mm) was analytically derived (``tests/fixtures/tip/README.md``).
    At ``web_position=0.5`` a full-depth web placed with zero headroom fails
    at layer 101 (Z=20.40 mm) because the generator's and validator's
    independent worst-case re-measurements disagree by a fraction of a
    micron; the generator avoids this with
    ``aircraft_core/generator.py``'s ``FULL_DEPTH_MARGIN_HEADROOM_MM``, and
    ``FullDepthPlacementHeadroomTests`` below pins that case directly.
    """

    N_LAYERS = 290
    SPAN_MM = 60.0
    ROOT_CHORD_MM = 60.0
    TIP_CHORD_MM = 3.0
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5
    WEB_POSITION = 0.3

    def _make_object(self):
        return fake_orca.make_tapered_naca_object(
            "wing",
            self.N_LAYERS,
            LAYER_HEIGHT_MM,
            self.ROOT_CHORD_MM,
            self.TIP_CHORD_MM,
            self.SPAN_MM,
        )

    def _config(self, tip_shutdown_enabled):
        return json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "gap_ratio": self.GAP_RATIO,
                "web_position": self.WEB_POSITION,
                "tip_shutdown_enabled": tip_shutdown_enabled,
            }
        )

    def test_shutdown_enabled_root_structure_and_shutdown_near_predicted_z(self):
        self.arm()
        obj = self._make_object()
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: self._config(tip_shutdown_enabled=True)

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)

        # Structure at the root: the very first (thickest) layer is
        # mutated, not shut down.
        root_layer = obj.layers()[0]
        self.assertEqual(root_layer.make_slices_calls, 1)

        report = self.read_report()
        self.assertGreater(report["mutated_count"], 0)
        self.assertGreater(report["shutdown_count"], 0)
        self.assertIsNone(report["first_failure"])

        first_shutdown_z = report["first_shutdown_z"]
        self.assertIsNotNone(first_shutdown_z)
        # The analytic value is kept as the reference. That prediction
        # models THICKNESS only -- the Z at which the section can no longer
        # accommodate a web plus its margins. The degenerate-web guard adds
        # a SECOND, independent criterion: a web whose own depth is below
        # overshoot_mm is a stub. It bites slightly earlier, moving first
        # shutdown 54.3842 -> 54.0 mm. The tolerance widens to two layer
        # heights so the gap between the two criteria stays visible instead
        # of being absorbed into a new baseline.
        self.assertLess(
            abs(first_shutdown_z - PREDICTED_SHUTDOWN_Z_MM),
            2 * LAYER_HEIGHT_MM,
            "first shutdown Z {} mm is more than one layer height from the "
            "analytically predicted {} mm".format(
                first_shutdown_z, PREDICTED_SHUTDOWN_Z_MM
            ),
        )

        # Every shutdown layer really is a thinness-only omission, and
        # every layer at or after the first shutdown stays shut down (the
        # section only gets thinner toward the tip on this fixture) --
        # i.e. shutdown is not scattered/inconsistent.
        shutdown_zs = [r["print_z"] for r in report["layers"] if r["shutdown"]]
        self.assertEqual(len(shutdown_zs), report["shutdown_count"])
        self.assertEqual(min(shutdown_zs), first_shutdown_z)

    def test_shutdown_disabled_gives_the_SAME_degraded_result(self):
        self.arm()
        obj = self._make_object()
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: self._config(tip_shutdown_enabled=False)

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)
        self.assertIn("DEGRADED", result.message)
        self.assertIn("pattern_all_webs_omitted", result.message)

        report = self.read_report()
        # The flag does not gate the OUTCOME: these tip layers are degraded
        # (unmutated, reported) whether or not tip_shutdown_enabled is set.
        #
        # Layer count and range: 21 layers shut down. 19 of them (Z=[54.4,
        # 58.0] mm) are those whose candidates fail on thickness. The Z=54.2
        # mm station must NOT add a layer: ``far_cap_mitre_clearance_mm``
        # measures clearance against the topologically-far chain only (no
        # per-edge Y/orientation heuristic), so for the sole candidate there
        # it uses the true lower-chain edge (0.5665 mm, above the 0.42 mm
        # safety offset). A per-edge filter can pick an edge on the UPPER
        # (near/entry) chain instead (``_split_upper_lower_chains``) -- the
        # near surface's wraparound near a chord extremity, triggered by this
        # fixture's thin, small-chord (~8.7 mm) tip region -- understating the
        # clearance (0.4134 mm) and wrongly omitting the candidate. The other
        # 2 layers come from the degenerate-web guard: their only surviving
        # candidates are stubs shallower than overshoot_mm (mostly the
        # builder's entry allowance, no structure), and not placing a member
        # that is not a member is the correct outcome.
        self.assertEqual(report["shutdown_count"], 21)
        self.assertEqual(report["degraded_layer_count"], 21)
        # There is no failure -- the tip degrades. The PREDICTED Z is checked
        # against the first DEGRADATION, i.e. where the section stops carrying
        # a pattern.
        self.assertIsNone(report["first_failure"])
        self.assertIsNotNone(report["first_degradation_z"])
        # Same two-layer shift as the sibling assertion above: the
        # analytic PREDICTED_SHUTDOWN_Z_MM models thickness only, and the
        # degenerate-web guard adds a second criterion that bites earlier.
        # Analytic value kept as the reference; tolerance widened rather
        # than the prediction re-derived.
        self.assertLess(
            abs(report["first_degradation_z"] - PREDICTED_SHUTDOWN_Z_MM),
            2 * LAYER_HEIGHT_MM,
        )


# ---------------------------------------------------------------------------
# 5. The exact reproduction: a full-depth web at web_position=0.5 on the
#    tip-taper fixture, layer 101 (Z=20.40 mm), must succeed directly
#    (FULL_DEPTH_MARGIN_HEADROOM_MM, aircraft_core/generator.py).
# ---------------------------------------------------------------------------


class FullDepthPlacementHeadroomTests(TipShutdownTestCase):
    """Direct reproduction of the zero-headroom full-depth placement case:
    web_position=0.5 on the SAME tip-taper fixture TipFixtureShutdownTests
    uses, at layer 101 (Z=20.40 mm, chord 40.6 mm). A placement leaving
    EXACTLY skin_margin_mm of remaining skin by construction would be
    contradicted by validate.py's independent worst-case re-measurement
    over the actually-removed region's own span, by a fraction of a micron,
    enough to fail invariant 4 on a section that is genuinely still thick,
    far from the tip. ``FULL_DEPTH_MARGIN_HEADROOM_MM`` prevents this.

    N_LAYERS=105 (Z up to 21.0 mm) rather than the fixture's usual 290: at
    web_position=0.5 this fake host's Sutherland-Hodgman-based geometry
    stub hits an UNRELATED probe-grazes-a-vertex limitation near the tip
    (well past Z=20.40 mm) that TipFixtureShutdownTests' own
    web_position=0.3 does not — this test only needs to bracket layer 101,
    not the whole span, so it stops comfortably short of that unrelated
    limitation rather than working around it.
    """

    N_LAYERS = 105
    SPAN_MM = 60.0
    ROOT_CHORD_MM = 60.0
    TIP_CHORD_MM = 3.0
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5
    WEB_POSITION = 0.5
    TARGET_LAYER_INDEX = 101
    TARGET_PRINT_Z_MM = 20.4

    def _make_object(self):
        return fake_orca.make_tapered_naca_object(
            "wing",
            self.N_LAYERS,
            LAYER_HEIGHT_MM,
            self.ROOT_CHORD_MM,
            self.TIP_CHORD_MM,
            self.SPAN_MM,
        )

    def test_web_position_half_at_layer_101_succeeds(self):
        self.arm()
        obj = self._make_object()
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "gap_ratio": self.GAP_RATIO,
                "web_position": self.WEB_POSITION,
                "tip_shutdown_enabled": False,
            }
        )

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)

        report = self.read_report()
        self.assertIsNone(report["first_failure"])

        target_layer = obj.layers()[self.TARGET_LAYER_INDEX]
        self.assertAlmostEqual(target_layer.print_z, self.TARGET_PRINT_Z_MM, places=6)
        self.assertEqual(target_layer.make_slices_calls, 1)

        target_record = report["layers"][self.TARGET_LAYER_INDEX]
        self.assertFalse(target_record["failed"])
        self.assertTrue(target_record["mutated"])


class ShutdownCauseIsNotMislabelledTests(TipShutdownTestCase):
    """The summary must name the RIGHT cause for a shutdown layer.

    The pattern-shutdown count is taken from each layer's own record, not
    derived by subtraction (``shutdown_count - tip_alignment_skip_count``):
    shutdown records come from more than two call sites, so a subtraction
    would report a layer degraded because NO CANDIDATE WAS EVER GENERATED as a
    thinness "pattern shutdown" -- a different and wrong cause.

    The all-thin fixtures cannot catch that, since the label comes out right
    for them either way. This test uses a MIXED-cause omission set, which is
    not thinness, and pins that the summary does not call it a pattern
    shutdown.
    """

    N_LAYERS = 1
    SKIN_MARGIN_MM = 1.0
    GAP_RATIO = 0.5

    def test_a_non_thinness_shutdown_is_not_called_a_pattern_shutdown(self):
        self.arm()
        obj, _bbox = fake_orca.make_airfoil_object(
            "wing", self.N_LAYERS, LAYER_HEIGHT_MM)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": self.SKIN_MARGIN_MM,
                "tip_shutdown_enabled": True,
                "families": [
                    {"name": "at_te_tip", "gap_ratio": self.GAP_RATIO,
                     "web_position": 1.0},
                    {"name": "near_te", "gap_ratio": self.GAP_RATIO,
                     "web_position": 0.95},
                ],
            }
        )

        result = cap.execute(ctx)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)

        report = self.read_report()
        reasons = {
            (lr.get("extra") or {}).get("reason")
            for lr in report["layers"] if lr.get("shutdown")
        }
        self.assertEqual(
            reasons, {"all_webs_omitted"},
            "a mixed-cause omission set is NOT a thinness shutdown")
        # The label must follow the record, not a subtraction.
        self.assertNotIn("pattern shutdown", result.message)


if __name__ == "__main__":
    unittest.main()
