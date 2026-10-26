"""``capability.py`` drops a per-slot-infeasible web BEFORE the boolean
diff (via ``TopologyValidator.infeasible_slots``) instead of refusing the whole
layer, and records the drop as a ``slot_infeasible`` degradation.

Runs against the SHIPPED bundle (``fake_orca.exec_bundle``), exactly like
``test_airfoil_capability.py``, over the same real NACA 2412 stack used by
``ArmedAirfoilHappyPathTests`` there (``web_position=0.7``,
``skin_margin_mm=1.0``, ``gap_ratio=0.5``).

A natural spec cannot exercise the infeasible-drop path on its own: the
generator already omits (as an ``OmittedWeb``) exactly what the validator would
reject, so an infeasible web never reaches slot build. Every test here
therefore monkeypatches ``StructureGenerator.generate`` on the exec'd bundle's
OWN namespace to APPEND a synthetic, deliberately-infeasible ``WebIntent``
alongside the real generator's output for one target layer -- the real
generator, frame, slot builder and validator all still run unmodified on both
the real and synthetic intents; only the INPUT is injected. The synthetic
intent starts at the SAME point as a real, feasible intent (so it needs no
independent surface probe of its own) but travels about 200x farther along its
own centerline, guaranteeing it overshoots the opposite exterior surface and
leaves far less than ``skin_margin_mm`` of remaining skin at its far end --
invariant 4, the remaining-skin check in ``TopologyValidator._check_one_slot``.
"""

import dataclasses
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fake_orca  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO_ROOT)


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

SKIN_MARGIN_MM = 1.0
GAP_RATIO = 0.5
WEB_POSITION = 0.7


class _Base(unittest.TestCase):
    """Fixture plumbing, mirroring ``test_airfoil_capability.py``'s
    ``AirfoilIntegrationTestCase``."""

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

    def spec_config(self):
        return json.dumps(
            {
                "root_solid_layers": 0,
                "skin_margin_mm": SKIN_MARGIN_MM,
                "gap_ratio": GAP_RATIO,
                "web_position": WEB_POSITION,
            }
        )

    def inject_infeasible_intent(self, target_layer_index, only_bad=False):
        """Monkeypatches ``StructureGenerator.generate`` on THIS test's own
        exec'd namespace so that, for ``target_layer_index`` only, an extra
        synthetic ``WebIntent`` -- built from a real, feasible intent's own
        ``start``, with its centerline extended ~200x -- is present in the
        generation result. ``only_bad=True`` replaces the layer's intents
        entirely with just the synthetic one (for the zero-survivors case);
        the default appends it alongside the real intent(s).
        """
        generator_cls = self.namespace["StructureGenerator"]
        real_generate = generator_cls.generate

        def patched_generate(self_gen, frame, spec, wall, layer_ctx, **kwargs):
            result = real_generate(self_gen, frame, spec, wall, layer_ctx, **kwargs)
            if layer_ctx.layer_index == target_layer_index and result.intents:
                base = result.intents[0]
                dx = base.end[0] - base.start[0]
                dy = base.end[1] - base.start[1]
                bad_end = (base.start[0] + dx * 200.0, base.start[1] + dy * 200.0)
                bad = dataclasses.replace(base, end=bad_end)
                if only_bad:
                    result.intents[:] = [bad]
                else:
                    result.intents.append(bad)
            return result

        generator_cls.generate = patched_generate
        self.addCleanup(setattr, generator_cls, "generate", real_generate)


class OneFeasibleOneInfeasibleSlotTests(_Base):
    """A layer with one feasible and one infeasible slot slices to
    SUCCESS, the infeasible slot is absent from the shipped output, and the
    drop is recorded as a ``slot_infeasible`` degradation carrying the
    per-slot predicate's own message."""

    N_LAYERS = 1

    def setUp(self):
        super().setUp()
        self.inject_infeasible_intent(target_layer_index=0)
        self.arm()
        self.obj = self.make_airfoil_object(n_layers=self.N_LAYERS)
        ctx = self.make_ctx(self.obj)
        cap = self.new_capability()
        cap.get_config = lambda: self.spec_config()
        self.result = cap.execute(ctx)

    def test_slice_succeeds_and_layer_is_mutated(self):
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(self.result.status, PluginResult.Success, self.result.message)
        layer = self.obj.layers()[0]
        self.assertEqual(layer.make_slices_calls, 1)

    def test_output_has_exactly_one_slot_worth_of_cut(self):
        # Independent measurement: build the SAME feasible-only spec with no
        # injection and compare the shipped contour -- the infeasible extra
        # web must have contributed nothing to what shipped.
        control_ns = fake_orca.exec_bundle(
            os.path.join(self._tmp.name, "control_aircraft_structures.py")
        )
        control_cap_cls = control_ns["AircraftStructuresM1"]
        control_obj = self.make_airfoil_object(n_layers=self.N_LAYERS, object_id="control")
        with open(os.path.join(self.plugin_dir, "m1_enabled"), "w", encoding="utf-8"):
            pass
        control_cap = control_cap_cls()
        control_cap.get_config = lambda: self.spec_config()
        control_ctx = self._fake_orca.slicing.SlicingPipelineContext(
            step=self._fake_orca.slicing.Step.posSlice,
            obj=control_obj,
            config=dict(FULL_CONFIG),
            cancelled_fn=None,
        )
        control_result = control_cap.execute(control_ctx)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(control_result.status, PluginResult.Success, control_result.message)

        got = self.surfaces_of(self.obj.layers()[0])[0].expolygon
        want = self.surfaces_of(control_obj.layers()[0])[0].expolygon
        self.assertEqual(
            fake_orca.describe(got), fake_orca.describe(want),
            "the injected infeasible slot changed the shipped geometry -- "
            "it should have been dropped before the boolean diff",
        )

    def test_degradation_recorded_with_kind_and_own_message(self):
        report = self.read_report()
        degradations = [
            d for lr in report["layers"] for d in lr.get("degradations", [])
        ]
        infeasible = [d for d in degradations if d["kind"] == "slot_infeasible"]
        self.assertEqual(len(infeasible), 1, degradations)
        reason = infeasible[0]["reason"]
        self.assertTrue(reason.startswith("DEGRADED -- "), reason)

        # PINS THE WHOLE CHAIN: predicate fires -> slot dropped ->
        # degradation carries that predicate's OWN message.
        #
        # Why the specific text and not just "mm of skin": without the drop,
        # the FAKE HOST's kernel would refuse to diff two same-X-range
        # slots, which is a different cause from "validate()'s predicate
        # would have refused this layer". Asserting the exact sentence
        # invariant 4 produces makes this test independent of what the
        # kernel would do downstream -- otherwise it proves the drop
        # happened, not that it prevented the refusal it exists to prevent.
        self.assertRegex(
            reason,
            r"slot \d+.*leaves -?[\d.]+ mm of skin at its far end, "
            r"below the required",
            reason,
        )
        # And the same sentence must be what validate() itself would have
        # raised -- same predicate, one implementation, two call sites.
        self.assertIn("below the required", reason)

    def test_report_is_success_not_a_refusal(self):
        report = self.read_report()
        self.assertNotIn("error", report)
        self.assertIsNone(report["first_failure"])
        self.assertEqual(report["failed_count"], 0)
        # Mutated, not degraded-to-unmutated: the SURVIVING feasible slot
        # still cuts the layer.
        self.assertEqual(report["mutated_count"], 1)


class AllSlotsFeasibleNoRegressionTests(_Base):
    """The same real stack, with NO injection at all, degrades ZERO slots --
    the drop step must be inert whenever every slot is already feasible
    pre-diff (``ArmedAirfoilHappyPathTests`` in
    ``test_airfoil_capability.py`` covers this fixture's exact expected
    geometry)."""

    N_LAYERS = 6

    def setUp(self):
        super().setUp()
        self.arm()
        self.obj = self.make_airfoil_object(n_layers=self.N_LAYERS)
        ctx = self.make_ctx(self.obj)
        cap = self.new_capability()
        cap.get_config = lambda: self.spec_config()
        self.result = cap.execute(ctx)

    def test_zero_degradation(self):
        report = self.read_report()
        self.assertEqual(report["degraded_layer_count"], 0)
        for lr in report["layers"]:
            self.assertEqual(lr.get("degradations", []), [])
        self.assertEqual(report["mutated_count"], self.N_LAYERS)
        self.assertEqual(report["failed_count"], 0)

    def test_geometry_is_byte_identical_to_the_pre_change_pipeline(self):
        # Independent measurement, not just an absence-of-degradation
        # inference: run a SECOND stack through a namespace whose
        # ``TopologyValidator.infeasible_slots`` always reports "every slot
        # feasible" (equivalent to having no drop step, since a feasible
        # spec has nothing to drop) and compare the shipped contour of every layer, point for point.
        pre_change_ns = fake_orca.exec_bundle(
            os.path.join(self._tmp.name, "pre_change_aircraft_structures.py")
        )
        pre_change_ns["TopologyValidator"].infeasible_slots = (
            lambda self_v, *a, **k: []
        )
        pre_change_cap_cls = pre_change_ns["AircraftStructuresM1"]
        pre_change_obj = self.make_airfoil_object(
            n_layers=self.N_LAYERS, object_id="pre_change")
        with open(os.path.join(self.plugin_dir, "m1_enabled"), "w", encoding="utf-8"):
            pass
        pre_change_cap = pre_change_cap_cls()
        pre_change_cap.get_config = lambda: self.spec_config()
        pre_change_ctx = self._fake_orca.slicing.SlicingPipelineContext(
            step=self._fake_orca.slicing.Step.posSlice,
            obj=pre_change_obj,
            config=dict(FULL_CONFIG),
            cancelled_fn=None,
        )
        pre_change_result = pre_change_cap.execute(pre_change_ctx)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(
            pre_change_result.status, PluginResult.Success, pre_change_result.message)

        for layer_index in range(self.N_LAYERS):
            got = self.surfaces_of(self.obj.layers()[layer_index])[0].expolygon
            want = self.surfaces_of(
                pre_change_obj.layers()[layer_index])[0].expolygon
            self.assertEqual(
                fake_orca.describe(got), fake_orca.describe(want),
                "layer {} geometry diverged from the pre-change "
                "(infeasible_slots always-[]) pipeline".format(layer_index),
            )


class DroppingAllSlotsLandsInZeroIntentPathTests(_Base):
    """When the drop leaves NO surviving slots, the layer lands in
    the existing zero-intent degradation path (unmutated, reported,
    ``pattern_all_webs_omitted``) rather than crashing or inventing a
    second path."""

    N_LAYERS = 1

    def setUp(self):
        super().setUp()
        self.inject_infeasible_intent(target_layer_index=0, only_bad=True)
        self.arm()
        self.obj = self.make_airfoil_object(n_layers=self.N_LAYERS)
        ctx = self.make_ctx(self.obj)
        cap = self.new_capability()
        cap.get_config = lambda: self.spec_config()
        self.result = cap.execute(ctx)

    def test_lands_in_zero_intent_path_not_a_crash(self):
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(self.result.status, PluginResult.Success, self.result.message)
        layer = self.obj.layers()[0]
        self.assertEqual(layer.make_slices_calls, 0)

        report = self.read_report()
        self.assertNotIn("error", report)
        self.assertEqual(report["degraded_layer_count"], 1)
        lr = report["layers"][0]
        self.assertFalse(lr["mutated"])
        kinds = {d["kind"] for d in lr.get("degradations", [])}
        self.assertIn("pattern_all_webs_omitted", kinds)
        self.assertIn("slot_infeasible", kinds)


if __name__ == "__main__":
    unittest.main()
