"""Unit tests for aircraft_core/metrics.py: aggregation and the counting
GeometryOps wrapper, in isolation from any host or fake host."""

import unittest

from aircraft_core.metrics import CountingGeometryOps, LayerMetrics, PerformanceMetrics
from stub_ops import Rect, StubOps


class PerformanceMetricsAggregationTests(unittest.TestCase):
    def _record(self, index, z, gen_s, slot_s, diag_s, val_s, slots=1, boolean=None,
                vbefore=None, vafter=None):
        return LayerMetrics(
            layer_index=index,
            print_z=z,
            generation_s=gen_s,
            slot_build_and_boolean_s=slot_s,
            diagnostics_capture_s=diag_s,
            validation_s=val_s,
            slot_count=slots,
            boolean_ops=boolean or {},
            vertex_count_before=vbefore,
            vertex_count_after=vafter,
        )

    def test_known_set_median_p95_worst(self):
        # Five layers with generation times (ms): 1, 2, 3, 4, 100.
        metrics = PerformanceMetrics()
        gen_times_ms = [1.0, 2.0, 3.0, 4.0, 100.0]
        for i, ms in enumerate(gen_times_ms):
            metrics.record_layer(self._record(i, i * 0.2, ms / 1000.0, 0.0, 0.0, 0.0))

        report = metrics.to_dict()
        gen = report["phases"]["generation_ms"]

        self.assertEqual(gen["count"], 5)
        self.assertAlmostEqual(gen["total_ms"], sum(gen_times_ms), places=6)
        self.assertAlmostEqual(gen["median_ms"], 3.0, places=6)
        # Nearest-rank p95 of 5 sorted values: ceil(0.95*5)=5th value (index 4) = 100.0
        self.assertAlmostEqual(gen["p95_ms"], 100.0, places=6)
        self.assertAlmostEqual(gen["worst_ms"], 100.0, places=6)
        self.assertEqual(gen["worst_layer_index"], 4)
        self.assertAlmostEqual(gen["worst_print_z"], 0.8, places=6)

    def test_empty_metrics_reports_zero_cost_shape(self):
        metrics = PerformanceMetrics()
        report = metrics.to_dict()

        self.assertEqual(report["layers_measured"], 0)
        self.assertEqual(report["total_measured_ms"], 0.0)
        self.assertIsNone(report["fraction_of_total_slicing_time"])
        for phase in report["phases"].values():
            self.assertEqual(phase["count"], 0)
            self.assertIsNone(phase["median_ms"])
            self.assertIsNone(phase["p95_ms"])
            self.assertIsNone(phase["worst_layer_index"])
        self.assertEqual(report["slots"]["total"], 0)
        self.assertEqual(report["boolean_ops"]["total"], 0)
        self.assertEqual(report["vertices"]["layers_with_vertex_before"], 0)
        self.assertEqual(report["layers"], [])

    def test_slot_and_boolean_totals(self):
        metrics = PerformanceMetrics()
        metrics.record_layer(
            self._record(0, 0.2, 0.001, 0.002, 0.0003, 0.0004, slots=2,
                          boolean={"diff": 2, "intersection": 1, "offset": 1})
        )
        metrics.record_layer(
            self._record(1, 0.4, 0.001, 0.002, 0.0003, 0.0004, slots=3,
                          boolean={"diff": 3, "intersection": 0, "offset": 1})
        )

        report = metrics.to_dict()
        self.assertEqual(report["slots"]["total"], 5)
        self.assertEqual(report["slots"]["per_layer_median"], 2.5)
        self.assertEqual(report["slots"]["per_layer_max"], 3)

        self.assertEqual(report["boolean_ops"]["total"], 8)
        self.assertEqual(
            report["boolean_ops"]["by_method"],
            {"diff": 5, "intersection": 1, "offset": 2},
        )
        self.assertEqual(report["boolean_ops"]["per_layer_max"], 4)

    def test_vertex_counts_skip_none_but_report_availability(self):
        metrics = PerformanceMetrics()
        metrics.record_layer(self._record(0, 0.2, 0, 0, 0, 0, vbefore=4, vafter=8))
        metrics.record_layer(self._record(1, 0.4, 0, 0, 0, 0, vbefore=4, vafter=None))

        report = metrics.to_dict()
        v = report["vertices"]
        self.assertEqual(v["layers_with_vertex_before"], 2)
        self.assertEqual(v["layers_with_vertex_after"], 1)
        self.assertEqual(v["layers_missing_vertex_after"], 1)
        self.assertEqual(v["max_vertex_count_before"], 4)
        self.assertEqual(v["max_vertex_count_after"], 8)
        self.assertEqual(v["total_vertex_count_before"], 8)
        self.assertEqual(v["total_vertex_count_after"], 8)

    def test_json_round_trips(self):
        import json

        metrics = PerformanceMetrics()
        metrics.record_layer(self._record(0, 0.2, 0.001, 0.002, 0.0001, 0.0002, slots=1,
                                           boolean={"diff": 1}, vbefore=4, vafter=8))
        report = metrics.to_dict()
        round_tripped = json.loads(json.dumps(report))
        self.assertEqual(round_tripped, report)

    def test_fraction_of_total_slicing_time_is_none_with_a_note(self):
        # The task is explicit: this cannot be computed from inside the
        # plugin. Assert the field is present, None, and explained rather
        # than silently absent or a fabricated number.
        report = PerformanceMetrics().to_dict()
        self.assertIn("fraction_of_total_slicing_time", report)
        self.assertIsNone(report["fraction_of_total_slicing_time"])
        self.assertIn("elapsed_ms", report["fraction_of_total_slicing_time_note"])


class CountingGeometryOpsTests(unittest.TestCase):
    def test_counts_diff_intersection_offset_and_forwards_result(self):
        inner = StubOps()
        wrapped = CountingGeometryOps(inner)

        a = Rect(0.0, 0.0, 10.0, 10.0)
        b = Rect(2.0, 2.0, 8.0, 8.0)

        wrapped.diff(a, [b])
        wrapped.diff(a, [b])
        wrapped.intersection(a, b)
        wrapped.offset(b, -0.1)

        self.assertEqual(wrapped.snapshot(), {"diff": 2, "intersection": 1, "offset": 1})

    def test_non_boolean_methods_are_not_counted(self):
        inner = StubOps()
        wrapped = CountingGeometryOps(inner)
        rect = Rect(0.0, 0.0, 10.0, 10.0)

        wrapped.area(rect)
        wrapped.is_valid(rect)
        wrapped.bounds(rect)
        wrapped.contour_count(rect)
        wrapped.polygon_from_points([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])

        self.assertEqual(wrapped.snapshot(), {"diff": 0, "intersection": 0, "offset": 0})

    def test_wrapper_result_matches_unwrapped_call(self):
        inner = StubOps()
        wrapped = CountingGeometryOps(inner)
        a = Rect(0.0, 0.0, 10.0, 10.0)
        b = Rect(2.0, 2.0, 8.0, 8.0)

        direct = inner.intersection(a, b)
        wrapped_result = wrapped.intersection(a, b)
        self.assertEqual(direct, wrapped_result)

    def test_union_is_forwarded_uncounted(self):
        # In-host regression (outside elevon root-down layer 356): the wrapper
        # inherited GeometryOps.union's fail-closed NotImplementedError instead
        # of forwarding to the host backend.
        class _Inner:
            def union(self, polygons):
                return ["merged", len(polygons)]

        wrapped = CountingGeometryOps(_Inner())
        self.assertEqual(wrapped.union([1, 2]), ["merged", 2])
        self.assertEqual(wrapped.snapshot(), {"diff": 0, "intersection": 0, "offset": 0})

    def test_wrapper_overrides_every_public_geometry_ops_method(self):
        # Any method added to GeometryOps (abstract or not) must be forwarded
        # explicitly, or the metrics-enabled host path silently diverges
        # from the uninstrumented one.
        from aircraft_core.ops import GeometryOps

        public = {n for n in vars(GeometryOps) if not n.startswith("_")}
        missing = sorted(n for n in public if n not in vars(CountingGeometryOps))
        self.assertEqual(missing, [])


if __name__ == "__main__":
    unittest.main()
