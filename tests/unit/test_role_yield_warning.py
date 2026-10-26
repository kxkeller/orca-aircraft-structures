"""Host-log aggregate warning for role-yield omissions.

``_yield_lattice_to_primary`` (aircraft_core/generator.py)
omits a ``skin_lattice`` candidate rather than let it merge with, and truncate,
a ``primary_structure`` candidate at the same crossing -- see that method's own
docstring. Those omissions carry the "role yield:..." reason prefix and would otherwise land only in the per-layer/per-family diagnostics JSON,
never on the host-log result line (a field in an unread report is not loud).

Covers:

- ``RunDiagnostics.role_yield_family_ratios`` / ``role_yield_omission_count``
  aggregation (mirrors ``opposing_entry_family_ratios``'s/
  ``opposing_entry_omission_count``'s own source and "absent, not 0/0"
  convention).
- ``capability.py``'s ``_role_yield_warning_suffix`` -- the host-log
  result-line fragment: does not fire below
  ``diagnostics.ROLE_YIELD_OMISSION_WARNING_RATIO``, fires above it, and
  names the CONSEQUENCE (a lattice station dropped at a primary crossing so
  the primary keeps full depth), not merely that a count is nonzero.
- A config with zero role-yield omissions produces no warning at all (the
  common case: most families never touch a primary member).
- Every pre-existing ``to_dict``/``role_totals`` field is unaffected
  (additive-only guard, matching ``test_merge_depth_truncation.py``'s own
  convention).

Loads ``capability.py`` via the same minimal-``orca``-stub + real-bundle
harness ``test_merge_depth_truncation.py``/``test_web_tip_air_gap.py``
already use, for the same reason: this exercises the REAL ``capability.py``
source (never a re-implementation of its logic), not a hand test double.
"""
import importlib.util
import sys
import types
import unittest
from pathlib import Path

from aircraft_core.diagnostics import (
    ROLE_YIELD_OMISSION_WARNING_RATIO,
    LayerRecord,
    RunDiagnostics,
)

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CAPABILITY_PATH = REPO_ROOT / "plugin" / "aircraft_structures" / "capability.py"
CORE_DIR = REPO_ROOT / "aircraft_core"
BUILD_PLUGIN_PATH = REPO_ROOT / "tools" / "build_plugin.py"


def _load_build_plugin_module():
    spec = importlib.util.spec_from_file_location(
        "build_plugin_under_test_role_yield_warning", BUILD_PLUGIN_PATH
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
    module = types.ModuleType("capability_bundle_under_test_role_yield_warning")
    code = compile(bundle_text, str(CAPABILITY_PATH), "exec")
    exec(code, module.__dict__)  # noqa: S102 - trusted, repo-local source
    return module


_cap = _load_capability_module()


def _families_entry(
    slots=0,
    merged_slots=0,
    omitted=0,
    opposing_entry_omitted=0,
    role_yield_omitted=0,
    role="skin_lattice",
):
    return {
        "slots": slots,
        "merged_slots": merged_slots,
        "omitted": omitted,
        "opposing_entry_omitted": opposing_entry_omitted,
        "role_yield_omitted": role_yield_omitted,
        "role": role,
    }


# -- RunDiagnostics aggregation -----------------------------------------------


class RoleYieldAggregationTests(unittest.TestCase):
    def test_family_ratio_aggregated_across_layers(self):
        diag = RunDiagnostics()
        diag.record(
            LayerRecord(
                layer_index=0,
                print_z=0.2,
                mutated=True,
                extra={
                    "families": {
                        "1 lattice_plus_upper": _families_entry(
                            slots=9, omitted=1, role_yield_omitted=1
                        ),
                        "0 spar_front": _families_entry(
                            slots=1, role="primary_structure"
                        ),
                    },
                    "role_yield_omission_count": 1,
                },
            )
        )
        diag.record(
            LayerRecord(
                layer_index=1,
                print_z=0.4,
                mutated=True,
                extra={
                    "families": {
                        "1 lattice_plus_upper": _families_entry(
                            slots=8, omitted=2, role_yield_omitted=2
                        ),
                    },
                    "role_yield_omission_count": 2,
                },
            )
        )

        self.assertEqual(diag.role_yield_omission_count, 3)
        ratios = diag.role_yield_family_ratios()
        self.assertIn("1 lattice_plus_upper", ratios)
        r = ratios["1 lattice_plus_upper"]
        # slots+omitted summed: (9+1) + (8+2) = 20; role_yield_omitted: 1+2=3.
        self.assertEqual(r.role_yield_omitted, 3)
        self.assertEqual(r.total_candidates, 20)
        self.assertAlmostEqual(r.ratio, 3.0 / 20.0)
        # spar_front never carries a role-yield omission -- absent from the
        # numerator but still reachable via its own (zero) ratio entry.
        self.assertEqual(ratios["0 spar_front"].role_yield_omitted, 0)

    def test_family_absent_when_never_recorded(self):
        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=True))
        self.assertEqual(diag.role_yield_family_ratios(), {})
        self.assertEqual(diag.role_yield_omission_count, 0)

    def test_role_totals_carry_role_yield_omitted(self):
        diag = RunDiagnostics()
        diag.record(
            LayerRecord(
                layer_index=0,
                print_z=0.2,
                mutated=True,
                extra={
                    "families": {
                        "1 lattice_plus_upper": _families_entry(
                            slots=9, omitted=1, role_yield_omitted=1
                        ),
                    }
                },
            )
        )
        totals = diag.role_totals()
        self.assertEqual(totals["skin_lattice"].role_yield_omitted, 1)

    def test_to_dict_carries_role_yield_fields(self):
        diag = RunDiagnostics()
        diag.record(
            LayerRecord(
                layer_index=0,
                print_z=0.2,
                mutated=True,
                extra={
                    "families": {
                        "1 lattice_plus_upper": _families_entry(
                            slots=9, omitted=1, role_yield_omitted=1
                        ),
                    },
                    "role_yield_omission_count": 1,
                },
            )
        )
        as_dict = diag.to_dict()
        self.assertEqual(as_dict["role_yield_omission_count"], 1)
        self.assertEqual(
            as_dict["role_yield_family_ratios"]["1 lattice_plus_upper"][
                "role_yield_omitted"
            ],
            1,
        )
        self.assertEqual(
            as_dict["role_yield_omission_warning_ratio"],
            ROLE_YIELD_OMISSION_WARNING_RATIO,
        )
        self.assertEqual(
            as_dict["role_totals"]["skin_lattice"]["role_yield_omitted"], 1
        )


# -- capability.py host-log warning suffix ------------------------------------


class RoleYieldWarningSuffixTests(unittest.TestCase):
    def _suffix(self, diag):
        cap = _cap.AircraftStructuresM1
        return cap._role_yield_warning_suffix(None, diag)

    def _diag_with_ratio(self, role_yield_omitted, total_candidates, key="1 lattice_plus_upper"):
        diag = RunDiagnostics()
        omitted = role_yield_omitted
        slots = total_candidates - omitted
        diag.record(
            LayerRecord(
                layer_index=0,
                print_z=0.2,
                mutated=True,
                extra={
                    "families": {
                        key: _families_entry(
                            slots=slots, omitted=omitted, role_yield_omitted=omitted
                        ),
                    },
                    "role_yield_omission_count": omitted,
                },
            )
        )
        return diag

    def test_does_not_fire_with_no_role_yield_omissions(self):
        diag = RunDiagnostics()
        diag.record(
            LayerRecord(
                layer_index=0,
                print_z=0.2,
                mutated=True,
                extra={
                    "families": {
                        "1 lattice_plus_upper": _families_entry(slots=10),
                    }
                },
            )
        )
        self.assertEqual(self._suffix(diag), "")

    def test_does_not_fire_below_threshold(self):
        # 1/20 = 0.05, well below ROLE_YIELD_OMISSION_WARNING_RATIO -- the
        # anisogrid config's own measured worst case (~0.018) sits here too.
        diag = self._diag_with_ratio(1, 20)
        self.assertEqual(self._suffix(diag), "")

    def test_does_not_fire_just_below_threshold_boundary(self):
        # 499/1000 = 0.499, just under the 0.5 threshold.
        diag = self._diag_with_ratio(499, 1000)
        self.assertEqual(self._suffix(diag), "")

    def test_fires_at_or_above_threshold(self):
        # Exactly half a family's candidates lost to role yield.
        diag = self._diag_with_ratio(5, 10)
        suffix = self._suffix(diag)
        self.assertNotEqual(suffix, "")
        self.assertIn("WARNING", suffix)
        self.assertIn("1 lattice_plus_upper", suffix)
        self.assertIn("5", suffix)
        self.assertIn("10", suffix)
        self.assertIn("50%", suffix)

    def test_wording_names_the_consequence_not_merely_the_event(self):
        diag = self._diag_with_ratio(9, 10)
        suffix = self._suffix(diag)
        # The consequence: the primary member keeps full depth because the
        # lattice yielded -- not merely "a role-yield omission occurred".
        self.assertIn("full depth", suffix)
        self.assertIn("primary", suffix)
        self.assertIn("lattice", suffix)

    def test_worst_family_reported_when_several_qualify(self):
        diag = RunDiagnostics()
        diag.record(
            LayerRecord(
                layer_index=0,
                print_z=0.2,
                mutated=True,
                extra={
                    "families": {
                        "1 lattice_plus_upper": _families_entry(
                            slots=4, omitted=6, role_yield_omitted=6
                        ),  # 6/10 = 0.6
                        "2 lattice_minus_upper": _families_entry(
                            slots=1, omitted=9, role_yield_omitted=9
                        ),  # 9/10 = 0.9 -- worst
                    }
                },
            )
        )
        suffix = self._suffix(diag)
        self.assertIn("2 lattice_minus_upper", suffix)
        self.assertIn("90%", suffix)

    def test_anisogrid_measured_ratio_does_not_fire(self):
        """Regression pin for a measured case: the shipped
        anisogrid preset's worst-affected lattice family
        (``lattice_plus_upper``/``lattice_minus_upper``) loses 85 of 4618
        candidate stations across the real 300-layer NACA 2412 fixture --
        ratio ~0.0184, two orders of magnitude below
        ``ROLE_YIELD_OMISSION_WARNING_RATIO``. A config this project already
        considers correctly configured must not trip this warning."""
        diag = self._diag_with_ratio(85, 4618)
        self.assertEqual(self._suffix(diag), "")


class PreExistingFieldsUnaffectedTests(unittest.TestCase):
    def test_role_totals_pre_existing_fields_unchanged(self):
        diag = RunDiagnostics()
        diag.record(
            LayerRecord(
                layer_index=0,
                print_z=0.2,
                mutated=True,
                extra={
                    "families": {
                        "0 spar_front": _families_entry(
                            slots=1, role="primary_structure"
                        ),
                    }
                },
            )
        )
        totals = diag.role_totals()
        self.assertEqual(totals["primary_structure"].slots, 1)
        self.assertEqual(totals["primary_structure"].omitted, 0)
        self.assertEqual(totals["primary_structure"].opposing_entry_omitted, 0)
        self.assertEqual(totals["primary_structure"].role_yield_omitted, 0)

    def test_to_dict_top_level_fields_unchanged(self):
        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=True))
        as_dict = diag.to_dict()
        self.assertEqual(as_dict["layers_seen"], 1)
        self.assertEqual(as_dict["mutated_count"], 1)
        self.assertEqual(as_dict["node_merge_count"], 0)
        self.assertIsNone(as_dict["first_node_merge_z"])
        self.assertIn("structure_summary", as_dict)
        self.assertEqual(as_dict["role_yield_omission_count"], 0)
        self.assertEqual(as_dict["role_yield_family_ratios"], {})


if __name__ == "__main__":
    unittest.main()
