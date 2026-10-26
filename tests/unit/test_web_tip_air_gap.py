"""Web-tip air gap: the tip-fusion finding made a first-class, reported quantity.

Covers:

- ``RunDiagnostics.web_tip_air_gap_mm`` arithmetic at two different line
  widths (0.42 and 0.60 mm), including that
  ``FULL_DEPTH_MARGIN_HEADROOM_MM`` is folded into the real placed gap, not
  the naive ``skin_margin_mm - 2 * line_width`` formula.
- ``RunDiagnostics.web_tip_bond_class``'s boundary classification,
  including exact-threshold values.
- ``RunDiagnostics.full_depth_web_count`` excludes depth-limited webs.
- ``capability.py``'s host-log warning fragment fires/does not fire at the
  documented margin values.
- Every pre-existing ``structure_summary``/``to_dict`` field is unaffected
  by these additions (additive-only guard, matching
  ``test_diagnostics.py``'s own convention).

Loads ``capability.py`` directly via ``importlib`` under a MINIMAL stub
``orca`` module installed in ``sys.modules`` -- just enough surface for the
two module-level ``orca.*`` references (the ``AircraftStructuresM1`` class
base and the ``@orca.plugin`` decorator on ``AircraftStructures``) to
import without error. This is deliberately NOT ``tests/integration``'s
``fake_orca.py`` (a much larger fake host used to exec the generated
bundle end-to-end): ``_tip_gap_warning_suffix`` and
``_is_full_depth_intent`` need no host geometry at all, only a
``RunDiagnostics`` instance and plain data, so a minimal stub keeps this
suite fast and dependency-free while still exercising the real
``capability.py`` source (never a re-implementation of its logic).
"""
import importlib.util
import sys
import types
import unittest
from pathlib import Path

from aircraft_core.diagnostics import RunDiagnostics, _classify_web_tip_bond
from aircraft_core.generator import FULL_DEPTH_MARGIN_HEADROOM_MM

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CAPABILITY_PATH = REPO_ROOT / "plugin" / "aircraft_structures" / "capability.py"
CORE_DIR = REPO_ROOT / "aircraft_core"
BUILD_PLUGIN_PATH = REPO_ROOT / "tools" / "build_plugin.py"


def _load_build_plugin_module():
    spec = importlib.util.spec_from_file_location(
        "build_plugin_under_test_web_tip_air_gap", BUILD_PLUGIN_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _install_minimal_orca_stub():
    """Install just enough of ``orca`` for ``capability.py`` to import.

    Only the two module-level ``orca.*`` touches
    (``class AircraftStructuresM1(orca.slicing.SlicingPipelineCapabilityBase)``
    and ``@orca.plugin`` / ``orca.base`` on ``AircraftStructures``) need to
    resolve at import time; everything else in ``capability.py`` that
    touches ``orca`` is inside a function/method body and is never called
    by these tests.
    """
    if "orca" in sys.modules:
        return
    stub = types.ModuleType("orca")
    stub.slicing = types.SimpleNamespace(
        SlicingPipelineCapabilityBase=object,
        Step=types.SimpleNamespace(posSlice="posSlice"),
        unscale=lambda coord: 1e-6,
    )
    stub.base = object
    stub.plugin = lambda cls: cls
    stub.register_capability = lambda cls: None
    stub.ExecutionResult = types.SimpleNamespace(
        success=lambda *a, **k: None, failure=lambda *a, **k: None
    )
    stub.PluginResult = types.SimpleNamespace(
        FatalError="FatalError", RecoverableError="RecoverableError"
    )
    stub.host = types.SimpleNamespace()
    sys.modules["orca"] = stub


def _load_capability_module():
    """Exec the FLATTENED bundle (``tools/build_plugin.py``'s own
    ``build_bundle``), not ``capability.py`` in isolation: ``capability.py``
    is written to be concatenated after ``aircraft_core``'s modules
    (``GeometryOps``, ``StructureGenerator``, etc. are supplied by that
    concatenation, per every ``# noqa: F821 - supplied by the bundle``
    comment throughout the file) and does not import them itself -- see
    ``tools/build_plugin.py``'s own module docstring. Building the SAME
    bundle the real plugin ships (rather than writing a second, parallel
    assembly here) is exactly what keeps this test from silently drifting
    from the real installable artifact.
    """
    _install_minimal_orca_stub()
    build_plugin = _load_build_plugin_module()
    bundle_text = build_plugin.build_bundle(CORE_DIR, CAPABILITY_PATH)
    module = types.ModuleType("capability_bundle_under_test_web_tip_air_gap")
    code = compile(bundle_text, str(CAPABILITY_PATH), "exec")
    exec(code, module.__dict__)  # noqa: S102 - trusted, repo-local source
    return module


_cap = _load_capability_module()


class _FakeFamily:
    def __init__(self, web_depth_mm=None):
        self.web_depth_mm = web_depth_mm


class _FakeIntent:
    def __init__(self, family_index, merged_family_indices=()):
        self.family_index = family_index
        self.merged_family_indices = merged_family_indices


class WebTipAirGapArithmeticTests(unittest.TestCase):
    """``web_tip_air_gap_mm`` at 0.42 mm and 0.60 mm line widths."""

    def test_shipped_default_0_42mm_beads(self):
        #'s own headline
        # figure: skin_margin_mm 1.0 at 0.42 mm beads -> 0.160 mm gap.
        diag = RunDiagnostics()
        diag.set_skin_margin_mm(1.0)
        diag.set_skin_line_width_mm(0.42)
        expected = 1.0 + FULL_DEPTH_MARGIN_HEADROOM_MM - 2.0 * 0.42
        self.assertAlmostEqual(diag.web_tip_air_gap_mm, expected)
        self.assertAlmostEqual(diag.web_tip_air_gap_mm, 0.1602, places=4)
        self.assertEqual(diag.web_tip_bond_class, "none")

    def test_0_60mm_beads(self):
        diag = RunDiagnostics()
        diag.set_skin_margin_mm(1.221)
        diag.set_skin_line_width_mm(0.60)
        expected = 1.221 + FULL_DEPTH_MARGIN_HEADROOM_MM - 2.0 * 0.60
        self.assertAlmostEqual(diag.web_tip_air_gap_mm, expected)

    def test_real_gap_includes_headroom_and_is_larger_than_naive_formula(self):
        # The naive formula (skin_margin_mm - 2 * line_width) is what's table
        # computed. generator.py's ACTUAL placement adds
        # FULL_DEPTH_MARGIN_HEADROOM_MM on top of skin_margin_mm, so the REAL
        # gap is larger by exactly that amount. This must be measured from the
        # real placement, not assumed --'s own repeated lesson about the
        # optimistic-direction bug class.
        diag = RunDiagnostics()
        diag.set_skin_margin_mm(0.861)
        diag.set_skin_line_width_mm(0.42)
        naive = 0.861 - 2.0 * 0.42
        real = diag.web_tip_air_gap_mm
        self.assertGreater(real, naive)
        self.assertAlmostEqual(real - naive, FULL_DEPTH_MARGIN_HEADROOM_MM)

    def test_none_when_margin_not_recorded(self):
        diag = RunDiagnostics()
        diag.set_skin_line_width_mm(0.42)
        self.assertIsNone(diag.web_tip_air_gap_mm)
        self.assertIsNone(diag.web_tip_bond_class)

    def test_none_when_line_width_not_recorded(self):
        diag = RunDiagnostics()
        diag.set_skin_margin_mm(1.0)
        self.assertIsNone(diag.web_tip_air_gap_mm)
        self.assertIsNone(diag.web_tip_bond_class)


class WebTipBondClassBoundaryTests(unittest.TestCase):
    """MEASURED table, boundary-exact."""

    def test_fused_at_and_below_0_021(self):
        self.assertEqual(_classify_web_tip_bond(0.0), "fused")
        self.assertEqual(_classify_web_tip_bond(0.021), "fused")
        # Beads that actually collide (negative "gap") are at least as
        # fused as the hard-fuse point -- not a separate case.
        self.assertEqual(_classify_web_tip_bond(-0.240), "fused")

    def test_weak_strictly_above_0_021_through_0_042(self):
        self.assertEqual(_classify_web_tip_bond(0.0210001), "weak")
        self.assertEqual(_classify_web_tip_bond(0.042), "weak")

    def test_marginal_strictly_above_0_042_below_0_105(self):
        self.assertEqual(_classify_web_tip_bond(0.0420001), "marginal")
        self.assertEqual(_classify_web_tip_bond(0.104999), "marginal")

    def test_none_at_and_above_0_105(self):
        self.assertEqual(_classify_web_tip_bond(0.105), "none")
        self.assertEqual(_classify_web_tip_bond(0.360), "none")

    def test_real_m11_arm_gap_at_exact_float_artifact_classifies_fused(self):
        # Floating-point boundary artifact: arms A/B/C's skin_margin_mm
        # (0.8608) is tuned to land the real web_tip_air_gap_mm EXACTLY on the
        # 0.021 mm hard-fuse point. float64 computes this as 0.021000000000000019
        # (~2e-17 mm above _BOND_FUSED_MAX_MM), which an exact <= comparison
        # misclassified as "weak" -- flipping whether the -Y skin reads as
        # restrained. This must classify as "fused", computed the same way the
        # real diagnostics arithmetic does (not a bare literal).
        diag = RunDiagnostics()
        diag.set_skin_margin_mm(0.8608)
        diag.set_skin_line_width_mm(0.42)
        gap = diag.web_tip_air_gap_mm
        self.assertAlmostEqual(gap, 0.021, places=9)
        self.assertEqual(_classify_web_tip_bond(gap), "fused")
        self.assertEqual(diag.web_tip_bond_class, "fused")

    def test_100um_above_threshold_is_a_real_difference_and_stays_weak(self):
        # The tolerance must not swallow a physically meaningful gap: 100
        # micrometres above the fused threshold (0.0211 mm) is five orders
        # of magnitude larger than _BOND_CLASS_EPSILON_MM (1e-9 mm) and
        # must still classify as "weak", not "fused".
        self.assertEqual(_classify_web_tip_bond(0.0211), "weak")

    def test_100um_below_none_threshold_is_a_real_difference_and_stays_marginal(self):
        # Same guard at the marginal/none boundary, in the direction that
        # matters there: a gap genuinely, non-noise below 0.105 mm (by 100
        # micrometres) must still classify as "marginal", not "none".
        self.assertEqual(_classify_web_tip_bond(0.1049), "marginal")

    def test_research_paper_table_values(self):
        # 's own table, read back through the classifier (naive gaps, no
        # headroom, since that table predates this module and used the simple
        # formula).
        cases = {
            0.000: "fused",
            0.021: "fused",
            0.042: "weak",
            0.105: "none",
            0.160: "none",
            0.360: "none",
        }
        for gap, expected in cases.items():
            with self.subTest(gap=gap):
                self.assertEqual(_classify_web_tip_bond(gap), expected)


class FullDepthWebCountingTests(unittest.TestCase):
    """``RunDiagnostics.full_depth_web_count`` / ``record_structure_layer``
    accumulation, and ``capability.py``'s ``_is_full_depth_intent``
    classification of individual ``WebIntent``-shaped objects."""

    def test_full_depth_web_counted(self):
        diag = RunDiagnostics()
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=80.0,
            web_depths_mm=[6.0],
            entry_x_mm=[10.0],
            full_depth_web_count=1,
        )
        self.assertEqual(diag.full_depth_web_count, 1)
        self.assertEqual(diag.structure_summary()["full_depth_web_count"], 1)

    def test_depth_limited_web_not_counted(self):
        diag = RunDiagnostics()
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=80.0,
            web_depths_mm=[2.0],
            entry_x_mm=[10.0],
            full_depth_web_count=0,
        )
        self.assertEqual(diag.full_depth_web_count, 0)
        self.assertEqual(diag.structure_summary()["full_depth_web_count"], 0)

    def test_default_full_depth_web_count_is_zero_and_accumulates(self):
        # Pre-existing call sites that omit the new kwarg entirely (e.g.
        # every test in test_diagnostics.py's StructureSummaryTests) must
        # keep working unchanged.
        diag = RunDiagnostics()
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=80.0,
            web_depths_mm=[6.0],
            entry_x_mm=[10.0],
        )
        self.assertEqual(diag.full_depth_web_count, 0)
        diag.record_structure_layer(
            layer_index=1,
            print_z=0.4,
            skin_length_mm=80.0,
            web_depths_mm=[6.0, 2.0],
            entry_x_mm=[10.0, 20.0],
            full_depth_web_count=1,
        )
        self.assertEqual(diag.full_depth_web_count, 1)

    def test_capability_classifies_single_family_full_depth_intent(self):
        families = [_FakeFamily(web_depth_mm=None)]
        intent = _FakeIntent(family_index=0)
        cap = _cap.AircraftStructuresM1
        self.assertTrue(cap._is_full_depth_intent(None, intent, families))

    def test_capability_classifies_single_family_depth_limited_intent(self):
        families = [_FakeFamily(web_depth_mm=2.0)]
        intent = _FakeIntent(family_index=0)
        cap = _cap.AircraftStructuresM1
        self.assertFalse(cap._is_full_depth_intent(None, intent, families))

    def test_capability_merged_intent_full_depth_only_if_all_members_are(self):
        families = [_FakeFamily(web_depth_mm=None), _FakeFamily(web_depth_mm=None)]
        intent = _FakeIntent(family_index=0, merged_family_indices=(0, 1))
        cap = _cap.AircraftStructuresM1
        self.assertTrue(cap._is_full_depth_intent(None, intent, families))

    def test_capability_merged_intent_with_depth_limited_member_excluded(self):
        # Conservative undercounting, by design: never claim full-depth
        # (and therefore never raise the tip-bond warning) for a merge
        # that MIGHT not actually reach the far skin -- see
        # ``_is_full_depth_intent``'s own docstring.
        families = [_FakeFamily(web_depth_mm=None), _FakeFamily(web_depth_mm=1.5)]
        intent = _FakeIntent(family_index=0, merged_family_indices=(0, 1))
        cap = _cap.AircraftStructuresM1
        self.assertFalse(cap._is_full_depth_intent(None, intent, families))


class TipGapWarningSuffixTests(unittest.TestCase):
    """``capability.AircraftStructuresM1._tip_gap_warning_suffix`` -- the host-log
    result-line fragment (a field in an unread report is not
    loud)."""

    def _suffix(self, diag):
        cap = _cap.AircraftStructuresM1
        return cap._tip_gap_warning_suffix(None, diag)

    def test_fires_at_shipped_1_0mm_margin_0_42mm_beads(self):
        diag = RunDiagnostics()
        diag.set_skin_margin_mm(1.0)
        diag.set_skin_line_width_mm(0.42)
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=80.0,
            web_depths_mm=[6.0],
            entry_x_mm=[10.0],
            full_depth_web_count=1,
        )
        suffix = self._suffix(diag)
        self.assertIn("WARNING", suffix)
        self.assertIn("none", suffix)
        self.assertIn("1 full-depth web", suffix)

    def test_does_not_fire_at_0_861mm_margin_0_42mm_beads(self):
        # §3.4's candidate hard-fuse window edge -- gap 0.021 + headroom,
        # "weak", not "marginal"/"none", so the warning must NOT fire here.
        diag = RunDiagnostics()
        diag.set_skin_margin_mm(0.861)
        diag.set_skin_line_width_mm(0.42)
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=80.0,
            web_depths_mm=[6.0],
            entry_x_mm=[10.0],
            full_depth_web_count=1,
        )
        self.assertEqual(diag.web_tip_bond_class, "weak")
        self.assertEqual(self._suffix(diag), "")

    def test_does_not_fire_at_1_221mm_margin_0_60mm_beads(self):
        diag = RunDiagnostics()
        diag.set_skin_margin_mm(1.221)
        diag.set_skin_line_width_mm(0.60)
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=80.0,
            web_depths_mm=[6.0],
            entry_x_mm=[10.0],
            full_depth_web_count=1,
        )
        self.assertEqual(diag.web_tip_bond_class, "weak")
        self.assertEqual(self._suffix(diag), "")

    def test_does_not_fire_when_no_full_depth_webs_present(self):
        # bond class "none" but every web is depth-limited: no defect to
        # warn about.
        diag = RunDiagnostics()
        diag.set_skin_margin_mm(1.0)
        diag.set_skin_line_width_mm(0.42)
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=80.0,
            web_depths_mm=[2.0],
            entry_x_mm=[10.0],
            full_depth_web_count=0,
        )
        self.assertEqual(diag.web_tip_bond_class, "none")
        self.assertEqual(diag.full_depth_web_count, 0)
        self.assertEqual(self._suffix(diag), "")

    def test_does_not_fire_when_bond_class_unknown(self):
        diag = RunDiagnostics()  # nothing recorded at all
        self.assertEqual(self._suffix(diag), "")

    def test_fires_at_marginal_class_too(self):
        diag = RunDiagnostics()
        diag.set_skin_margin_mm(0.882)
        diag.set_skin_line_width_mm(0.42)
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=80.0,
            web_depths_mm=[6.0],
            entry_x_mm=[10.0],
            full_depth_web_count=3,
        )
        self.assertEqual(diag.web_tip_bond_class, "marginal")
        suffix = self._suffix(diag)
        self.assertIn("WARNING", suffix)
        self.assertIn("marginal", suffix)
        self.assertIn("3 full-depth web", suffix)


class PreExistingFieldsUnaffectedTests(unittest.TestCase):
    """Additive-only guard, matching test_diagnostics.py's own convention:
    every pre-existing ``structure_summary``/``to_dict`` field keeps its
    value and presence."""

    def test_structure_summary_pre_existing_fields_unchanged(self):
        diag = RunDiagnostics()
        diag.set_skin_line_width_mm(0.6)
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=122.5,
            web_depths_mm=[3.0] * 14,
            entry_x_mm=[float(k) for k in range(14)],
            full_depth_web_count=14,
        )
        summary = diag.structure_summary()
        self.assertEqual(summary["skin_line_width_mm"], 0.6)
        self.assertEqual(summary["total_web_count"], 14)
        self.assertAlmostEqual(summary["web_mass_fraction_est"], (2.0 * 3.0 * 14) / (2.0 * 3.0 * 14 + 122.5))
        self.assertEqual(summary["worst_skin_bay_mm"], 1.0)
        # New fields present alongside the old ones.
        self.assertIn("web_tip_air_gap_mm", summary)
        self.assertIn("web_tip_bond_class", summary)
        self.assertIn("full_depth_web_count", summary)
        self.assertEqual(summary["full_depth_web_count"], 14)

    def test_to_dict_top_level_fields_unchanged(self):
        from aircraft_core.diagnostics import LayerRecord

        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=True))
        diag.record(
            LayerRecord(layer_index=1, print_z=0.4, failed=True, message="boom")
        )
        as_dict = diag.to_dict()
        self.assertEqual(as_dict["layers_seen"], 2)
        self.assertEqual(as_dict["mutated_count"], 1)
        self.assertEqual(as_dict["failed_count"], 1)
        self.assertIn("boom", as_dict["first_failure"]["message"])
        self.assertEqual(as_dict["node_merge_count"], 0)
        self.assertIsNone(as_dict["first_shutdown_z"])
        self.assertIsNone(as_dict["first_node_merge_z"])
        self.assertEqual(len(as_dict["layers"]), 2)
        # structure_summary is nested and untouched at this level.
        self.assertIn("structure_summary", as_dict)


if __name__ == "__main__":
    unittest.main()
