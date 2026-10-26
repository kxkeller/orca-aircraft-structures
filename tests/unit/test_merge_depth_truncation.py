"""Merged-node depth-truncation visibility.

``_merge_node_component`` (aircraft_core/generator.py) resolves a merged node
to the SHALLOWER of two Y candidates. When the merged constituents request very
different depths -- e.g. a shallow stiffener family (``web_depth_mm`` a few mm)
crossing a full-depth spar-web family -- the deep member is truncated to the
shallow one's depth at every crossing, and without this reporting
``node_merge_count`` only says a merge happened, not that a web was
gutted.

This is diagnostics/visibility ONLY -- it does not change what geometry gets
cut. Covers:

- ``RunDiagnostics.record_node_merge_truncation`` / ``worst_depth_
  truncation_ratio`` / ``truncated_merge_count`` accumulation (a same-depth
  merge reports ratio 1.0 and does not count as truncated; a mixed
  shallow/full-depth merge reports the real, much lower ratio and DOES
  count; the 0.75 threshold boundary on both sides; no merges at all reports
  cleanly, ``None``/``0``, never a ZeroDivisionError).
- ``capability.py``'s ``_node_merge_truncation_ratio`` -- the two-families-
  both-depth-limited case (exact, no geometry needed) and a real, generator
  -produced both-full-depth merge on a prismatic (rectangular) frame, which
  is the anti-drift pin against ``generator.py``'s own candidate-(b) full
  -depth formula -- expected ratio ~1.0.
- ``capability.py``'s ``_merge_truncation_warning_suffix`` -- the host-log
  result-line fragment (a field in an unread report is not loud),
  fires/does not fire, and names the count/worst-ratio/layer/Z.
- Every pre-existing ``structure_summary``/``to_dict`` field is unaffected
  (additive-only guard, matching ``test_diagnostics.py``'s/``test_web_tip_
  air_gap.py``'s own convention).

Loads ``capability.py`` via the same minimal-``orca``-stub + real-bundle
harness ``test_web_tip_air_gap.py`` already uses, for the same reason: this
exercises the REAL ``capability.py`` source (never a re-implementation of
its logic), not a hand test double.
"""
import importlib.util
import sys
import types
import unittest
from pathlib import Path

from aircraft_core.diagnostics import (
    MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO,
    RunDiagnostics,
)
from aircraft_core.frame import LayerFrame, corrected_skin_margin_mm
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.spec import StructureSpec, WebFamily

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CAPABILITY_PATH = REPO_ROOT / "plugin" / "aircraft_structures" / "capability.py"
CORE_DIR = REPO_ROOT / "aircraft_core"
BUILD_PLUGIN_PATH = REPO_ROOT / "tools" / "build_plugin.py"

sys.path.insert(0, str(Path(__file__).resolve().parent))
from stub_ops import Rect, StubOps  # noqa: E402


def _load_build_plugin_module():
    spec = importlib.util.spec_from_file_location(
        "build_plugin_under_test_merge_depth_truncation", BUILD_PLUGIN_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _install_minimal_orca_stub():
    """See ``test_web_tip_air_gap.py``'s identical helper for the exact
    surface this stub covers and why it is enough."""
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
    """Exec the FLATTENED bundle -- see ``test_web_tip_air_gap.py``'s
    identical helper's docstring for why."""
    _install_minimal_orca_stub()
    build_plugin = _load_build_plugin_module()
    bundle_text = build_plugin.build_bundle(CORE_DIR, CAPABILITY_PATH)
    module = types.ModuleType("capability_bundle_under_test_merge_depth_truncation")
    code = compile(bundle_text, str(CAPABILITY_PATH), "exec")
    exec(code, module.__dict__)  # noqa: S102 - trusted, repo-local source
    return module


_cap = _load_capability_module()


class _FakeFamily:
    def __init__(self, web_depth_mm=None):
        self.web_depth_mm = web_depth_mm


class _FakeIntent:
    def __init__(self, start, end, width_mm, merged_family_indices=()):
        self.start = start
        self.end = end
        self.width_mm = width_mm
        self.merged_family_indices = merged_family_indices


# -- RunDiagnostics accumulation ----------------------------------------------


class RunDiagnosticsTruncationTests(unittest.TestCase):
    def test_no_merges_reports_cleanly(self):
        diag = RunDiagnostics()
        self.assertIsNone(diag.worst_depth_truncation_ratio)
        self.assertIsNone(diag.worst_depth_truncation_layer_index)
        self.assertIsNone(diag.worst_depth_truncation_print_z)
        self.assertEqual(diag.truncated_merge_count, 0)
        summary = diag.structure_summary()
        self.assertIsNone(summary["worst_depth_truncation_ratio"])
        self.assertIsNone(summary["worst_depth_truncation_layer_index"])
        self.assertIsNone(summary["worst_depth_truncation_print_z"])
        self.assertEqual(summary["truncated_merge_count"], 0)
        self.assertEqual(
            summary["merge_depth_truncation_threshold_ratio"],
            MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO,
        )

    def test_same_depth_merge_reports_ratio_1_and_does_not_warn(self):
        diag = RunDiagnostics()
        diag.record_node_merge_truncation(layer_index=3, print_z=0.8, ratio=1.0)
        self.assertEqual(diag.worst_depth_truncation_ratio, 1.0)
        self.assertEqual(diag.worst_depth_truncation_layer_index, 3)
        self.assertEqual(diag.worst_depth_truncation_print_z, 0.8)
        self.assertEqual(diag.truncated_merge_count, 0)

    def test_shallow_plus_full_depth_merge_reports_real_ratio_and_warns(self):
        # 1.5 mm shallow stiffener crossing an (effectively) 8 mm full
        # -depth spar web -> ratio 1.5 / 8 = 0.1875, well below threshold.
        diag = RunDiagnostics()
        diag.record_node_merge_truncation(layer_index=10, print_z=2.2, ratio=0.1875)
        self.assertAlmostEqual(diag.worst_depth_truncation_ratio, 0.1875)
        self.assertEqual(diag.worst_depth_truncation_layer_index, 10)
        self.assertEqual(diag.worst_depth_truncation_print_z, 2.2)
        self.assertEqual(diag.truncated_merge_count, 1)

    def test_worst_tracks_the_lowest_ratio_across_layers(self):
        diag = RunDiagnostics()
        diag.record_node_merge_truncation(layer_index=1, print_z=0.2, ratio=0.9)
        diag.record_node_merge_truncation(layer_index=2, print_z=0.4, ratio=0.3)
        diag.record_node_merge_truncation(layer_index=3, print_z=0.6, ratio=0.6)
        self.assertAlmostEqual(diag.worst_depth_truncation_ratio, 0.3)
        self.assertEqual(diag.worst_depth_truncation_layer_index, 2)
        self.assertEqual(diag.worst_depth_truncation_print_z, 0.4)
        # 0.3 and 0.6 are both below the 0.75 threshold; 0.9 is not.
        self.assertEqual(diag.truncated_merge_count, 2)

    def test_threshold_boundary_exactly_on_threshold_not_counted(self):
        diag = RunDiagnostics()
        diag.record_node_merge_truncation(
            layer_index=0, print_z=0.2, ratio=MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO
        )
        self.assertEqual(diag.truncated_merge_count, 0)

    def test_threshold_boundary_just_below_threshold_counted(self):
        diag = RunDiagnostics()
        diag.record_node_merge_truncation(
            layer_index=0,
            print_z=0.2,
            ratio=MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO - 1e-6,
        )
        self.assertEqual(diag.truncated_merge_count, 1)

    def test_threshold_boundary_just_above_threshold_not_counted(self):
        diag = RunDiagnostics()
        diag.record_node_merge_truncation(
            layer_index=0,
            print_z=0.2,
            ratio=MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO + 1e-6,
        )
        self.assertEqual(diag.truncated_merge_count, 0)

    def test_multiple_truncated_merges_all_counted(self):
        diag = RunDiagnostics()
        diag.record_node_merge_truncation(layer_index=0, print_z=0.2, ratio=0.5)
        diag.record_node_merge_truncation(layer_index=1, print_z=0.4, ratio=0.2)
        diag.record_node_merge_truncation(layer_index=2, print_z=0.6, ratio=1.0)
        self.assertEqual(diag.truncated_merge_count, 2)


# -- capability.py's ratio computation ----------------------------------------


class CapabilityTruncationRatioTests(unittest.TestCase):
    """``_node_merge_truncation_ratio`` -- the frame-independent
    (both-depth-limited) case needs no geometry at all; the full-depth case
    is exercised via a real generator-produced merge below."""

    def _ratio(self, intent, families, frame=None, effective_skin_margin_mm=None):
        cap = _cap.AircraftStructuresM1()
        return cap._node_merge_truncation_ratio(
            intent, families, frame, effective_skin_margin_mm
        )

    def test_non_merged_intent_returns_none(self):
        families = [_FakeFamily(web_depth_mm=None)]
        intent = _FakeIntent(start=(10.0, 3.0), end=(10.0, -2.0), width_mm=1.0)
        self.assertIsNone(self._ratio(intent, families))

    def test_two_depth_limited_families_exact_ratio(self):
        # Both requested depths are exact spec constants -- no geometry
        # needed at all. Realised (merged) depth is the SHALLOWER
        # constituent's own depth (1.5 mm,); deepest requested is
        # the 8.0 mm family -> ratio 1.5 / 8.0.
        families = [_FakeFamily(web_depth_mm=1.5), _FakeFamily(web_depth_mm=8.0)]
        intent = _FakeIntent(
            start=(10.0, 3.0), end=(10.0, 1.5), width_mm=1.0,
            merged_family_indices=(0, 1),
        )
        ratio = self._ratio(intent, families)
        self.assertAlmostEqual(ratio, 1.5 / 8.0)
        self.assertLess(ratio, MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO)

    def test_two_depth_limited_families_same_depth_ratio_is_1(self):
        families = [_FakeFamily(web_depth_mm=4.0), _FakeFamily(web_depth_mm=4.0)]
        intent = _FakeIntent(
            start=(10.0, 3.0), end=(10.0, -1.0), width_mm=1.0,
            merged_family_indices=(0, 1),
        )
        self.assertAlmostEqual(self._ratio(intent, families), 1.0)

    def test_deepest_requested_of_three_way_merge(self):
        # Three-way merge (union-find can chain more than two): deepest
        # requested is the MAX across every constituent, not merely the
        # first/last.
        families = [
            _FakeFamily(web_depth_mm=1.5),
            _FakeFamily(web_depth_mm=6.0),
            _FakeFamily(web_depth_mm=1.5),
        ]
        intent = _FakeIntent(
            start=(10.0, 3.0), end=(10.0, 1.5), width_mm=1.0,
            merged_family_indices=(0, 1, 2),
        )
        self.assertAlmostEqual(self._ratio(intent, families), 1.5 / 6.0)

    def test_geometry_unreachable_full_depth_without_frame_returns_none(self):
        # BOTH constituents are full-depth, so there is no exact,
        # spec-only fallback to fall back on -- a full-depth constituent's
        # requested depth needs a frame to re-measure; without one this
        # must say "not computable" rather than approximate silently.
        families = [_FakeFamily(web_depth_mm=None), _FakeFamily(web_depth_mm=None)]
        intent = _FakeIntent(
            start=(10.0, 3.0), end=(10.0, 1.5), width_mm=1.0,
            merged_family_indices=(0, 1),
        )
        self.assertIsNone(
            self._ratio(intent, families, frame=None, effective_skin_margin_mm=1.0)
        )

    def test_geometry_unreachable_for_one_full_depth_member_still_uses_other(self):
        # When one merged constituent is depth-limited (an exact, spec
        # -only number) and another is full-depth but unreachable (no
        # frame given here), the deepest-requested figure conservatively
        # falls back to what IS known rather than reporting nothing at
        # all -- never overstating the loss, matching this project's
        # standing "never report the optimistic number" posture in the
        # OTHER direction (never silently claim MORE loss than provable
        # either).
        families = [_FakeFamily(web_depth_mm=None), _FakeFamily(web_depth_mm=1.5)]
        intent = _FakeIntent(
            start=(10.0, 3.0), end=(10.0, 1.5), width_mm=1.0,
            merged_family_indices=(0, 1),
        )
        ratio = self._ratio(intent, families, frame=None, effective_skin_margin_mm=1.0)
        self.assertAlmostEqual(ratio, 1.0)


class PrismaticFullDepthMergeAntiDriftPinTest(unittest.TestCase):
    """The anti-drift pin: drive the REAL ``StructureGenerator`` on a prismatic
    (rectangular) frame with two full-depth families forced into a merge, then
    feed the resulting real merged ``WebIntent`` through capability's own ratio
    calculation. Because both constituents are full-depth on a flat top/ bottom
    surface, ``_merge_node_component``'s candidate (a) and (b) coincide (no
    curvature to differ over), so the expected ratio is 1.0 to within
    floating-point noise. If this test ever fails, the two formulas
    (``generator.py``'s candidate (b) and capability's own re-measurement) have
    drifted apart -- fix the DRIFT, not the tolerance.
    """

    def test_both_full_depth_merge_ratio_is_approximately_1(self):
        ops = StubOps()
        rect = Rect(0.0, 0.0, 20.0, 20.0)
        frame = LayerFrame.from_polygon(ops, rect)

        fam_a = WebFamily(
            name="a", rib_pitch_mm=None, web_position=0.495, gap_ratio=0.05
        )
        fam_b = WebFamily(
            name="b", rib_pitch_mm=None, web_position=0.505, gap_ratio=0.05
        )
        spec = StructureSpec(
            gap_ratio=0.05, skin_margin_mm=1.0, families=(fam_a, fam_b)
        )
        spec.validate()
        safety_offset_mm = 0.42
        layer = LayerContext(layer_index=5, print_z=1.2)

        gen = StructureGenerator()
        generation = gen.generate(
            frame, spec, safety_offset_mm, layer, safety_offset_mm=safety_offset_mm
        )
        merged = [i for i in generation.intents if i.merged_family_indices]
        self.assertEqual(
            len(merged),
            1,
            "fixture must actually produce one merged node -- got intents "
            "{}".format(
                [(i.family_index, i.start, i.width_mm, i.merged_family_indices)
                 for i in generation.intents]
            ),
        )
        merged_intent = merged[0]

        effective_families = spec.effective_families()
        effective_skin_margin_mm = corrected_skin_margin_mm(
            spec.skin_margin_mm, frame.rotation_deg
        )
        cap = _cap.AircraftStructuresM1()
        ratio = cap._node_merge_truncation_ratio(
            merged_intent, effective_families, frame, effective_skin_margin_mm
        )
        self.assertIsNotNone(ratio)
        self.assertAlmostEqual(ratio, 1.0, places=6)


class SameRoleMergeStillTruncatesAfterRoleYieldTest(unittest.TestCase):
    """task ("shallow lattice member crossing a full-depth primary member"): the
    fix (``StructureGenerator._yield_lattice_to_primary``) narrows merge scope
    for a cross-ROLE (``skin_lattice`` vs ``primary_structure``) pair
    specifically -- it must NOT make this truncation warning path unreachable
    for the case it does not cover.

    Two ``skin_lattice`` families (the SAME role, so the role-yield pre-pass
    never engages), one full-depth and one shallow (1.5 mm), forced into a
    merge on a rectangular frame: the merge still happens (same-role pairs are
    untouched by the role-yield pre-pass, see ``_yield_lattice_to_primary``'s
    own docstring), the merged node is still genuinely truncated (ratio well
    below 0.75), and ``RunDiagnostics.record_node_merge_truncation`` still
    counts it -- the exact mechanism
    ``MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO``'s own module comment describes
    as the defect the reporting exists to surface.
    """

    def test_shallow_lattice_crossing_full_depth_lattice_still_truncates(self):
        ops = StubOps()
        rect = Rect(0.0, 0.0, 20.0, 20.0)
        frame = LayerFrame.from_polygon(ops, rect)

        fam_full = WebFamily(
            name="lattice_full", rib_pitch_mm=None, web_position=0.495, gap_ratio=0.05,
        )
        fam_shallow = WebFamily(
            name="lattice_shallow", rib_pitch_mm=None, web_position=0.505,
            gap_ratio=0.05, web_depth_mm=1.5,
        )
        spec = StructureSpec(
            gap_ratio=0.05, skin_margin_mm=1.0, families=(fam_full, fam_shallow)
        )
        spec.validate()
        safety_offset_mm = 0.42
        layer = LayerContext(layer_index=5, print_z=1.2)

        gen = StructureGenerator()
        generation = gen.generate(
            frame, spec, safety_offset_mm, layer, safety_offset_mm=safety_offset_mm
        )
        merged = [i for i in generation.intents if i.merged_family_indices]
        self.assertEqual(
            len(merged), 1,
            "same-role pair must still merge (untouched by the role-yield "
            "pre-pass) -- got intents {}".format(
                [(i.family_index, i.start, i.width_mm, i.merged_family_indices)
                 for i in generation.intents]
            ),
        )
        merged_intent = merged[0]

        effective_families = spec.effective_families()
        effective_skin_margin_mm = corrected_skin_margin_mm(
            spec.skin_margin_mm, frame.rotation_deg
        )
        cap = _cap.AircraftStructuresM1()
        ratio = cap._node_merge_truncation_ratio(
            merged_intent, effective_families, frame, effective_skin_margin_mm
        )
        self.assertIsNotNone(ratio)
        self.assertLess(
            ratio, MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO,
            "a full-depth lattice merged with a 1.5 mm shallow lattice on "
            "a 20 mm-tall rectangle must land well below the truncation "
            "threshold -- got ratio {}".format(ratio),
        )

        diag = RunDiagnostics()
        diag.record_node_merge_truncation(layer_index=5, print_z=1.2, ratio=ratio)
        self.assertEqual(diag.truncated_merge_count, 1)


# -- capability.py's host-log warning suffix ----------------------------------


class TruncationWarningSuffixTests(unittest.TestCase):
    def _suffix(self, diag):
        cap = _cap.AircraftStructuresM1
        return cap._merge_truncation_warning_suffix(None, diag)

    def test_does_not_fire_with_no_truncated_merges(self):
        diag = RunDiagnostics()
        diag.record_node_merge_truncation(layer_index=0, print_z=0.2, ratio=1.0)
        self.assertEqual(self._suffix(diag), "")

    def test_does_not_fire_with_no_merges_at_all(self):
        diag = RunDiagnostics()
        self.assertEqual(self._suffix(diag), "")

    def test_fires_with_truncated_merge_and_names_the_consequence(self):
        diag = RunDiagnostics()
        diag.record_node_merge_truncation(layer_index=12, print_z=2.6, ratio=0.1875)
        suffix = self._suffix(diag)
        self.assertIn("WARNING", suffix)
        self.assertIn("1", suffix)
        self.assertIn("0.188", suffix)  # worst ratio, rendered to 3dp
        self.assertIn("12", suffix)
        self.assertIn("2.6", suffix)
        # Word the consequence, not merely name the event.
        self.assertIn("shallower", suffix)

    def test_fires_with_worst_ratio_and_count_across_several_merges(self):
        diag = RunDiagnostics()
        diag.record_node_merge_truncation(layer_index=1, print_z=0.2, ratio=0.5)
        diag.record_node_merge_truncation(layer_index=2, print_z=0.4, ratio=0.1)
        suffix = self._suffix(diag)
        self.assertIn("2", suffix)
        self.assertIn("0.100", suffix)


# -- additive-only guard -------------------------------------------------------


class PreExistingFieldsUnaffectedTests(unittest.TestCase):
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
        self.assertEqual(summary["full_depth_web_count"], 14)
        self.assertIn("worst_depth_truncation_ratio", summary)
        self.assertIn("truncated_merge_count", summary)

    def test_to_dict_top_level_fields_unchanged(self):
        from aircraft_core.diagnostics import LayerRecord

        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=True))
        as_dict = diag.to_dict()
        self.assertEqual(as_dict["layers_seen"], 1)
        self.assertEqual(as_dict["mutated_count"], 1)
        self.assertEqual(as_dict["node_merge_count"], 0)
        self.assertIsNone(as_dict["first_node_merge_z"])
        self.assertIn("structure_summary", as_dict)


if __name__ == "__main__":
    unittest.main()
