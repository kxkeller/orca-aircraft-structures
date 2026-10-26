import json
import unittest

from aircraft_core.diagnostics import LayerRecord, RunDiagnostics


class DiagnosticsTests(unittest.TestCase):
    def test_counts_mutated_and_failed(self):
        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=False))
        diag.record(LayerRecord(layer_index=1, print_z=0.4, mutated=True))
        diag.record(LayerRecord(layer_index=2, print_z=0.6, mutated=True))
        diag.record(
            LayerRecord(
                layer_index=3, print_z=0.8, mutated=False, failed=True, message="boom"
            )
        )

        self.assertEqual(diag.mutated_count, 2)
        self.assertEqual(diag.failed_count, 1)
        self.assertEqual(len(diag.records), 4)

    def test_first_failure_captured_with_full_context(self):
        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=True))
        diag.record(
            LayerRecord(
                layer_index=1,
                print_z=0.4,
                failed=True,
                message="Aircraft structure invalid at layer 1 (Z=0.40 mm): boom.",
                extra={"cause": "boom"},
            )
        )
        diag.record(
            LayerRecord(
                layer_index=2,
                print_z=0.6,
                failed=True,
                message="a later, irrelevant failure",
            )
        )

        first = diag.first_failure
        self.assertIsNotNone(first)
        self.assertEqual(first.layer_index, 1)
        self.assertEqual(first.print_z, 0.4)
        self.assertIn("boom", first.message)
        self.assertEqual(first.extra, {"cause": "boom"})
        # The second failure must not overwrite the first.
        self.assertEqual(diag.failed_count, 2)

    def test_serialization_round_trips_through_json(self):
        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=True))
        diag.record(
            LayerRecord(
                layer_index=1,
                print_z=0.4,
                failed=True,
                message="boom",
                extra={"slot_number": 1},
            )
        )

        as_dict = diag.to_dict()
        # JSON-ready: round-trips without error and preserves structure.
        round_tripped = json.loads(json.dumps(as_dict))
        self.assertEqual(round_tripped, as_dict)
        self.assertEqual(round_tripped["layers_seen"], 2)
        self.assertEqual(round_tripped["mutated_count"], 1)
        self.assertEqual(round_tripped["failed_count"], 1)
        self.assertEqual(round_tripped["first_failure"]["layer_index"], 1)
        self.assertEqual(len(round_tripped["layers"]), 2)

    def test_no_failure_yields_none(self):
        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=True))
        self.assertIsNone(diag.first_failure)
        self.assertEqual(diag.to_dict()["first_failure"], None)

    def test_node_merge_count_aggregates_across_layers(self):
        # node_merge_count is aggregated the same way shutdown_count is (see
        # plugin/aircraft_structures/capability.py's per-layer
        # ``node_merge_count`` extra key): summed across layers, with the
        # first nonzero Z captured.
        diag = RunDiagnostics()
        diag.record(
            LayerRecord(
                layer_index=0,
                print_z=0.2,
                mutated=True,
                extra={"node_merge_count": 0},
            )
        )
        diag.record(
            LayerRecord(
                layer_index=1,
                print_z=0.4,
                mutated=True,
                extra={"node_merge_count": 2},
            )
        )
        diag.record(
            LayerRecord(
                layer_index=2,
                print_z=0.6,
                mutated=True,
                extra={"node_merge_count": 7},
            )
        )

        self.assertEqual(diag.node_merge_count, 9)
        self.assertEqual(diag.first_node_merge_z, 0.4)
        as_dict = diag.to_dict()
        self.assertEqual(as_dict["node_merge_count"], 9)
        self.assertEqual(as_dict["first_node_merge_z"], 0.4)

    def test_node_merge_count_zero_when_absent(self):
        # A layer that never sets the key (e.g. a shutdown or root-solid layer,
        # or any run that omits the key) contributes 0, not an error --
        # ``extra`` is caller-defined and this module does not impose a schema
        # on it.
        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=True))
        diag.record(
            LayerRecord(
                layer_index=1,
                print_z=0.4,
                mutated=False,
                extra={"reason": "root_solid_layers"},
            )
        )
        self.assertEqual(diag.node_merge_count, 0)
        self.assertIsNone(diag.first_node_merge_z)
        self.assertEqual(diag.to_dict()["node_merge_count"], 0)
        self.assertIsNone(diag.to_dict()["first_node_merge_z"])


class RoleTotalsTests(unittest.TestCase):
    """Role attribution: per-role slot/omission
    totals aggregated from each layer's own ``extra["families"]``
    breakdown, mirroring ``opposing_entry_family_ratios``'s source and
    "absent, not 0/0" convention -- see
    ``RunDiagnostics.role_totals``'s own docstring."""

    def test_aggregates_slots_and_omissions_per_role(self):
        diag = RunDiagnostics()
        diag.record(
            LayerRecord(
                layer_index=0,
                print_z=0.2,
                mutated=True,
                extra={
                    "families": {
                        "0 spar_front": {
                            "slots": 1,
                            "merged_slots": 0,
                            "omitted": 0,
                            "opposing_entry_omitted": 0,
                            "role": "primary_structure",
                        },
                        "1 diag_plus": {
                            "slots": 2,
                            "merged_slots": 1,
                            "omitted": 1,
                            "opposing_entry_omitted": 1,
                            "role": "skin_lattice",
                        },
                    }
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
                        "0 spar_front": {
                            "slots": 1,
                            "merged_slots": 0,
                            "omitted": 0,
                            "opposing_entry_omitted": 0,
                            "role": "primary_structure",
                        },
                    }
                },
            )
        )

        totals = diag.role_totals()
        self.assertEqual(totals["primary_structure"].slots, 2)
        self.assertEqual(totals["primary_structure"].omitted, 0)
        self.assertEqual(totals["skin_lattice"].slots, 2)
        self.assertEqual(totals["skin_lattice"].merged_slots, 1)
        self.assertEqual(totals["skin_lattice"].omitted, 1)
        self.assertEqual(totals["skin_lattice"].opposing_entry_omitted, 1)

    def test_role_absent_when_no_families_recorded(self):
        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=True))
        self.assertEqual(diag.role_totals(), {})

    def test_role_totals_reach_to_dict(self):
        diag = RunDiagnostics()
        diag.record(
            LayerRecord(
                layer_index=0,
                print_z=0.2,
                mutated=True,
                extra={
                    "families": {
                        "0 a": {
                            "slots": 3,
                            "merged_slots": 0,
                            "omitted": 0,
                            "opposing_entry_omitted": 0,
                            "role": "skin_lattice",
                        },
                    }
                },
            )
        )
        as_dict = diag.to_dict()
        self.assertEqual(as_dict["role_totals"]["skin_lattice"]["slots"], 3)


class StructureSummaryTests(unittest.TestCase):
    """: ``RunDiagnostics``'s structural-measurement accumulators, tested at the
    ``RunDiagnostics`` level directly -- the capability's own wiring
    (``_contour_perimeter_mm``/``_intent_depth_mm`` feeding these calls) is
    exercised end to end via the real bundle in
    ``tests/integration/test_airfoil_capability.py``-style harnesses, but the
    accumulation arithmetic itself is independent of any Orca adapter and
    belongs here.
    """

    def test_empty_run_reports_none_not_zero(self):
        diag = RunDiagnostics()
        summary = diag.structure_summary()
        self.assertIsNone(summary["skin_line_width_mm"])
        self.assertIsNone(summary["web_mass_fraction_est"])
        self.assertIsNone(summary["worst_skin_bay_mm"])
        self.assertIsNone(summary["worst_skin_bay_layer_index"])
        self.assertIsNone(summary["worst_skin_bay_print_z"])
        self.assertEqual(summary["total_web_count"], 0)
        self.assertIsNone(summary["mean_web_spacing_mm"])

    def test_skin_line_width_recorded_once(self):
        diag = RunDiagnostics()
        diag.set_skin_line_width_mm(0.6)
        self.assertEqual(diag.structure_summary()["skin_line_width_mm"], 0.6)

    def test_single_web_layer_has_no_bay_but_counts_the_web(self):
        # A layer with exactly one web has no "between adjacent entry
        # points" bay to measure (there is no second entry point) -- this
        # must not be conflated with "no data at all" (None) vs "a bay of
        # zero width", so it stays absent from the bay accumulators while
        # still counting toward total_web_count.
        diag = RunDiagnostics()
        diag.set_skin_line_width_mm(0.42)
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=80.0,
            web_depths_mm=[6.0],
            entry_x_mm=[10.0],
        )
        summary = diag.structure_summary()
        self.assertEqual(summary["total_web_count"], 1)
        self.assertIsNone(summary["worst_skin_bay_mm"])
        self.assertIsNone(summary["mean_web_spacing_mm"])
        # web_mass_fraction_est: 2*6.0 = 12.0 mm of web length against 80.0
        # mm of skin length -> 12 / 92.
        self.assertAlmostEqual(summary["web_mass_fraction_est"], 12.0 / 92.0)

    def test_worst_bay_and_mean_spacing_across_multiple_layers(self):
        diag = RunDiagnostics()
        diag.set_skin_line_width_mm(0.42)
        # Layer 0: three webs at x = 0, 4, 20 -> bays of 4.0 and 16.0 mm.
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=100.0,
            web_depths_mm=[2.0, 2.0, 2.0],
            entry_x_mm=[0.0, 4.0, 20.0],
        )
        # Layer 1: two webs at x = 0, 10 -> one bay of 10.0 mm.
        diag.record_structure_layer(
            layer_index=1,
            print_z=0.4,
            skin_length_mm=100.0,
            web_depths_mm=[2.0, 2.0],
            entry_x_mm=[0.0, 10.0],
        )
        summary = diag.structure_summary()
        self.assertEqual(summary["total_web_count"], 5)
        # Worst bay is the 16.0 mm gap on layer 0.
        self.assertEqual(summary["worst_skin_bay_mm"], 16.0)
        self.assertEqual(summary["worst_skin_bay_layer_index"], 0)
        self.assertEqual(summary["worst_skin_bay_print_z"], 0.2)
        # Mean of [4.0, 16.0, 10.0].
        self.assertAlmostEqual(summary["mean_web_spacing_mm"], 30.0 / 3.0)

    def test_entry_points_are_sorted_before_measuring_bays(self):
        # Entry points need not arrive in ascending X order (e.g. two
        # families interleaved differently on different layers) -- the bay
        # measurement must sort first, not assume caller order.
        diag = RunDiagnostics()
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=50.0,
            web_depths_mm=[1.0, 1.0, 1.0],
            entry_x_mm=[10.0, 0.0, 5.0],
        )
        summary = diag.structure_summary()
        self.assertEqual(summary["worst_skin_bay_mm"], 5.0)
        self.assertAlmostEqual(summary["mean_web_spacing_mm"], 5.0)

    def test_json_round_trip_via_to_dict(self):
        diag = RunDiagnostics()
        diag.set_skin_line_width_mm(0.6)
        diag.record_structure_layer(
            layer_index=0,
            print_z=0.2,
            skin_length_mm=122.5,
            web_depths_mm=[3.0] * 14,
            entry_x_mm=[float(k) for k in range(14)],
        )
        as_dict = diag.to_dict()
        round_tripped = json.loads(json.dumps(as_dict))
        self.assertEqual(round_tripped, as_dict)
        self.assertIn("structure_summary", round_tripped)
        self.assertEqual(round_tripped["structure_summary"]["total_web_count"], 14)
        self.assertEqual(
            round_tripped["structure_summary"]["skin_line_width_mm"], 0.6
        )

    def test_existing_report_fields_unaffected(self):
        # Additive-only guard: adding structure_summary must not perturb any
        # pre-existing field's value or presence.
        diag = RunDiagnostics()
        diag.record(LayerRecord(layer_index=0, print_z=0.2, mutated=True))
        diag.record(
            LayerRecord(
                layer_index=1, print_z=0.4, failed=True, message="boom"
            )
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


if __name__ == "__main__":
    unittest.main()
