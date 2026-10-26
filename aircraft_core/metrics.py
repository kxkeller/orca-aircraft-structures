"""Transform-time performance metrics: aggregation, not measurement policy.

Transform cost is linear in the number of webs (2.1 s / 1 web -> 9.9 s / 5
webs on the NACA 2412 fixture) and would multiply with a tighter-pitch
diamond lattice, so six things are recorded:

    total aircraft-transform time      ms and fraction of total slicing time
    per-layer transform time           median, p95, worst layer + index
    slot primitive count               per layer and total
    boolean operation count            per layer and total
    vertex count before/after          detect runaway complexity
    validation time                    separate from geometry generation

This module owns the aggregation (median/p95/worst, totals, the counting
``GeometryOps`` wrapper) and has no Orca dependency, the same separation
``diagnostics.py`` keeps: the host adapter (``plugin/aircraft_structures/
capability.py``) is responsible for calling ``time.perf_counter()`` around
its own phases and handing this module plain numbers.

WHAT THIS MODULE DOES NOT AND CANNOT DO: "fraction of total slicing time" needs
the host's own end-to-end slice duration, which is not visible from inside a
single plugin capability's ``execute()`` call. ``PerformanceMetrics.to_dict()``
reports the plugin's own total honestly and leaves
``fraction_of_total_slicing_time`` explicitly ``None`` with a note, rather than
inventing a number this module cannot support.

PHASE SPLIT: the natural split is three phases (generation, slot
building plus boolean subtraction, validation). This module and its caller use
four: the capability's own per-layer diagnostic capture (``_last_geometry`` in
``capability.py``) does real Clipper work of its own (bounds/area/ intersection
calls solely for the debug report) and would otherwise be silently folded into
"boolean subtraction" cost that profiling actually cares about. Isolating it
keeps the profile honest about what is geometry generation and what is
bookkeeping.

VERTEX COUNTING: measured every mutated layer by default (``sample_stride``
1), not sampled, because the underlying read (``len(polygon.contour.points)``)
is O(vertex count) and vertex counts here are small (rectangle coupons: 4-8;
the NACA 2412 fixture: on the order of 100). If a future fixture makes this
measurably expensive, raise ``sample_stride`` and this module's aggregation
already reports how many layers actually contributed a vertex count
(``layers_with_vertex_before`` / ``_after``) so a sampled run is legible as
sampled rather than silently looking like a full count. The host adapter
guards vertex-after collection per polygon piece (a backend may legitimately
refuse to reconstruct the exact outline of a cut, non-rectangular section —
see ``fake_orca.py``'s ``_PolygonNotched.points()``); an unavailable count is
recorded as ``None`` for that layer, never guessed.

BOOLEAN OPERATION COUNTING: ``CountingGeometryOps`` counts *calls into the
ops object* — one increment per ``diff``/``intersection``/``offset``
invocation — not the number of underlying Clipper operations those calls may
perform internally. ``OrcaGeometryOps.diff`` in particular loops over each
surviving piece for each clip internally (a multi-slot layer can call the
real Clipper ``diff_ex`` several times inside one ``ops.diff(...)`` call);
this wrapper cannot see inside that loop without per-operation
instrumentation, so its "boolean operation count" is a call-count lower
bound, not a Clipper-call-exact count.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence
import math
import statistics

from .ops import GeometryOps

BOOLEAN_METHODS = ("diff", "intersection", "offset")


class CountingGeometryOps(GeometryOps):
    """Thin call-counting wrapper around any ``GeometryOps``.

    Forwards every call unchanged to ``inner`` and increments a per-method
    counter first. Installed only when metrics are enabled, so the
    uninstrumented hot path (``OrcaGeometryOps`` called directly) is
    completely unaffected — this wrapper adds one dict increment per call
    when it IS installed, nothing else.
    """

    def __init__(self, inner):
        self._inner = inner
        self.counts: Dict[str, int] = {name: 0 for name in BOOLEAN_METHODS}

    def snapshot(self) -> Dict[str, int]:
        """A copy of the running per-method call totals, for before/after diffing."""
        return dict(self.counts)

    # -- counted methods --------------------------------------------------

    def diff(self, subject, clips):
        self.counts["diff"] += 1
        return self._inner.diff(subject, clips)

    def intersection(self, subject, clip):
        self.counts["intersection"] += 1
        return self._inner.intersection(subject, clip)

    def offset(self, polygon, delta_mm):
        self.counts["offset"] += 1
        return self._inner.offset(polygon, delta_mm)

    # -- passthroughs, not counted as "boolean operations" -----------------
    # area/is_valid/bounds/contour_count/polygon_from_points/
    # contour_points_mm are queries and construction, not Boolean
    # subtraction/intersection/offset.

    def area(self, polygon):
        return self._inner.area(polygon)

    def is_valid(self, polygon):
        return self._inner.is_valid(polygon)

    def bounds(self, polygon):
        return self._inner.bounds(polygon)

    def contour_count(self, polygon):
        return self._inner.contour_count(polygon)

    def polygon_from_points(self, points_mm):
        return self._inner.polygon_from_points(points_mm)

    def polygon_from_points_and_holes(self, points_mm, holes_mm):
        # Not part of the ``GeometryOps`` ABC (only ``OrcaGeometryOps``
        # implements it -- the alignment rotation reconstruction is a
        # host-boundary-only concern, never exercised by the offline
        # stub/shapely backends). Forwarded so the alignment rotation works
        # when metrics are enabled and ``ops`` is this wrapper, not
        # ``OrcaGeometryOps`` directly; otherwise it would raise
        # ``AttributeError``.
        return self._inner.polygon_from_points_and_holes(points_mm, holes_mm)

    def contour_points_mm(self, polygon):
        return self._inner.contour_points_mm(polygon)

    def hole_bounds(self, polygon):
        return self._inner.hole_bounds(polygon)

    def hole_points_mm(self, polygon):
        return self._inner.hole_points_mm(polygon)

    def union(self, polygons):
        # The alignment guard's closing emulation calls ``union``. Not abstract
        # on ``GeometryOps``, so without this forward the base class's
        # fail-closed NotImplementedError would fire on the first multi-loop
        # layer. Uncounted, like the other passthroughs, so BOOLEAN_METHODS and
        # the report's per-method counts keep their schema.
        return self._inner.union(polygons)


@dataclass
class LayerMetrics:
    """One layer's timing/counting breakdown, in seconds and raw counts.

    ``vertex_count_before``/``vertex_count_after`` are ``None`` when not
    measured (metrics disabled for that layer; the current caller measures
    every mutated layer) or unavailable (the backend refused to reconstruct a
    cut section's exact outline). ``None`` always means "no honest number for
    this layer", not zero.
    """

    layer_index: int
    print_z: float
    generation_s: float = 0.0
    slot_build_and_boolean_s: float = 0.0
    diagnostics_capture_s: float = 0.0
    validation_s: float = 0.0
    slot_count: int = 0
    boolean_ops: Dict[str, int] = field(default_factory=dict)
    vertex_count_before: Optional[int] = None
    vertex_count_after: Optional[int] = None

    @property
    def total_s(self) -> float:
        return (
            self.generation_s
            + self.slot_build_and_boolean_s
            + self.diagnostics_capture_s
            + self.validation_s
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "layer_index": self.layer_index,
            "print_z": self.print_z,
            "generation_ms": round(self.generation_s * 1000.0, 6),
            "slot_build_and_boolean_ms": round(self.slot_build_and_boolean_s * 1000.0, 6),
            "diagnostics_capture_ms": round(self.diagnostics_capture_s * 1000.0, 6),
            "validation_ms": round(self.validation_s * 1000.0, 6),
            "total_ms": round(self.total_s * 1000.0, 6),
            "slot_count": self.slot_count,
            "boolean_ops": dict(self.boolean_ops),
            "vertex_count_before": self.vertex_count_before,
            "vertex_count_after": self.vertex_count_after,
        }


_PHASES = (
    ("generation", "generation_s"),
    ("slot_build_and_boolean", "slot_build_and_boolean_s"),
    ("diagnostics_capture", "diagnostics_capture_s"),
    ("validation", "validation_s"),
)


def _percentile_nearest_rank(sorted_values: Sequence[float], fraction: float) -> Optional[float]:
    """Nearest-rank percentile: the smallest value V such that at least
    ``fraction`` of the (sorted, ascending) values are <= V. Simple, stable,
    and does not interpolate between samples (unlike some percentile
    conventions) — chosen because "worst layer" already reports the true
    maximum separately, so p95 here only needs to be a reasonable, documented
    cutoff, not an interpolated estimate."""
    if not sorted_values:
        return None
    n = len(sorted_values)
    idx = max(0, min(n - 1, math.ceil(fraction * n) - 1))
    return sorted_values[idx]


def _phase_stats(triples) -> Dict[str, Any]:
    """``triples``: sequence of ``(seconds, layer_index, print_z)``."""
    if not triples:
        return {
            "count": 0,
            "total_ms": 0.0,
            "median_ms": None,
            "p95_ms": None,
            "worst_ms": None,
            "worst_layer_index": None,
            "worst_print_z": None,
        }
    ms_values = [t[0] * 1000.0 for t in triples]
    sorted_ms = sorted(ms_values)
    worst = max(triples, key=lambda t: t[0])
    return {
        "count": len(triples),
        "total_ms": round(sum(ms_values), 6),
        "median_ms": round(statistics.median(ms_values), 6),
        "p95_ms": round(_percentile_nearest_rank(sorted_ms, 0.95), 6),
        "worst_ms": round(worst[0] * 1000.0, 6),
        "worst_layer_index": worst[1],
        "worst_print_z": worst[2],
    }


class PerformanceMetrics:
    """Accumulates ``LayerMetrics`` across one slice and aggregates them.

    Mirrors ``RunDiagnostics``'s shape and intent (``diagnostics.py``): a
    recorder, not a controller. The caller decides which layers to measure
    and never has to — this class does not gate on whether metrics are
    "enabled"; that decision lives entirely in the host adapter (no
    ``PerformanceMetrics`` instance is created at all when metrics are
    disabled, so the disabled case is zero-cost by construction rather than
    an internal branch here).
    """

    def __init__(self, vertex_sample_stride: int = 1) -> None:
        self._records: List[LayerMetrics] = []
        self.vertex_sample_stride = vertex_sample_stride

    def record_layer(self, layer_metrics: LayerMetrics) -> None:
        self._records.append(layer_metrics)

    @property
    def records(self) -> List[LayerMetrics]:
        return list(self._records)

    def to_dict(self) -> Dict[str, Any]:
        records = self._records
        phases = {}
        for name, attr in _PHASES:
            triples = [(getattr(r, attr), r.layer_index, r.print_z) for r in records]
            phases[name + "_ms"] = _phase_stats(triples)

        total_triples = [(r.total_s, r.layer_index, r.print_z) for r in records]
        total_stats = _phase_stats(total_triples)

        slot_counts = [r.slot_count for r in records]
        boolean_totals = {name: 0 for name in BOOLEAN_METHODS}
        per_layer_boolean_totals = []
        for r in records:
            layer_total = 0
            for name in BOOLEAN_METHODS:
                n = r.boolean_ops.get(name, 0)
                boolean_totals[name] += n
                layer_total += n
            per_layer_boolean_totals.append(layer_total)

        vertices_before = [r.vertex_count_before for r in records if r.vertex_count_before is not None]
        vertices_after = [r.vertex_count_after for r in records if r.vertex_count_after is not None]

        return {
            "enabled": True,
            "layers_measured": len(records),
            "total_measured_ms": round(sum(r.total_s for r in records) * 1000.0, 6),
            "fraction_of_total_slicing_time": None,
            "fraction_of_total_slicing_time_note": (
                "Not measurable from inside this plugin capability; it sees "
                "only its own execute() call, not the host's total slice "
                "duration. Compute this externally as this report's own "
                "elapsed_ms (the capability's total, in the enclosing "
                "diagnostics report) divided by the host log's slice "
                "duration for the same run."
            ),
            "phases": phases,
            "total": total_stats,
            "slots": {
                "total": sum(slot_counts),
                "per_layer_median": (
                    round(statistics.median(slot_counts), 6) if slot_counts else None
                ),
                "per_layer_max": max(slot_counts) if slot_counts else None,
            },
            "boolean_ops": {
                "total": sum(boolean_totals.values()),
                "by_method": boolean_totals,
                "per_layer_median": (
                    round(statistics.median(per_layer_boolean_totals), 6)
                    if per_layer_boolean_totals
                    else None
                ),
                "per_layer_max": max(per_layer_boolean_totals) if per_layer_boolean_totals else None,
                "note": (
                    "Call-count into the GeometryOps wrapper, not a count of "
                    "underlying Clipper operations; see module docstring "
                    "'BOOLEAN OPERATION COUNTING'."
                ),
            },
            "vertices": {
                "sample_stride": self.vertex_sample_stride,
                "layers_with_vertex_before": len(vertices_before),
                "layers_with_vertex_after": len(vertices_after),
                "layers_missing_vertex_after": len(records) - len(vertices_after),
                "max_vertex_count_before": max(vertices_before) if vertices_before else None,
                "max_vertex_count_after": max(vertices_after) if vertices_after else None,
                "total_vertex_count_before": sum(vertices_before) if vertices_before else None,
                "total_vertex_count_after": sum(vertices_after) if vertices_after else None,
            },
            "layers": [r.to_dict() for r in records],
        }
