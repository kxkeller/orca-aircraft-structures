"""Per-layer decision records and whole-run diagnostics.

Mirrors the shape of the probe's JSON report (``plugin/aircraft_structures
/aircraft_structures.py``'s ``report`` dict) closely enough that the capability
can extend the same reporting convention, but lives in ``aircraft_core`` so it
has no Orca dependency and is directly unit-testable.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

# The web-tip air gap (below) must use the SAME headroom the generator actually
# places a full-depth web's far end with, not a re-derived or assumed value --
# see ``generator.py``'s own comment on this constant and
# ``_generate_family``'s ``end_y = margin_y + FULL_DEPTH_MARGIN_HEADROOM_MM``.
# Importing it (rather than hardcoding a copy) keeps the two from drifting
# apart.
# ``tools/build_plugin.py`` strips this as an intra-package import at bundle
# time -- ``generator.py`` is already flattened into the same namespace earlier
# in ``CORE_MODULE_ORDER``, so the constant is simply in scope there.
from .generator import FULL_DEPTH_MARGIN_HEADROOM_MM

# MEASURED bond table, PLA at 210 C on a 0.42 mm outer wall, Elegoo Centauri
# Carbon 0.4 nozzle -- printed coupon results, NOT modelled or simulated
# values. They are specific to that temperature/material/nozzle combination
# and may not transfer to a different one:
#
#   gap mm | result
#   0.210  | no bond; a card slides into the slit
#   0.105  | no bond; visually identical to 0.210
#   0.042  | fused, but separable by hand
#   0.021  | fused; takes real effort to pull apart
#
# The boundaries below read that table as a step function of gap distance: <=
# 0.021 "fused" (hard-fuse point), <= 0.042 "weak" (separable-by-hand point), <
# 0.105 "marginal" (between the weak point and no-bond point, unmeasured
# directly but bounded by it on both sides), >= 0.105 "none" (the no-bond point
# and everything wider). These are NOT a physical model of bond strength as a
# function of gap -- four measured points, not a curve -- so "marginal" in
# particular is an interpolation between two measured points, not itself
# measured.
_BOND_FUSED_MAX_MM = 0.021
_BOND_WEAK_MAX_MM = 0.042
_BOND_MARGINAL_MAX_MM = 0.105

# Boundary-classification tolerance, in millimetres, applied ONLY at the
# threshold comparisons in ``_classify_web_tip_bond`` below.
#
# WHY THIS EXISTS: a ``skin_margin_mm`` of 0.8608 mm is chosen to place
# ``web_tip_air_gap_mm`` EXACTLY on the hard-fuse point (0.021 mm). The real
# arithmetic -- ``0.8608 + FULL_DEPTH_MARGIN_HEADROOM_MM - 2 * 0.42`` --
# computes in float64 as ``0.021000000000000019``, i.e. ~2e-17 mm ABOVE
# ``_BOND_FUSED_MAX_MM``. An exact ``<=`` comparison would classify a config
# tuned to be "fused" as "weak" instead, flipping whether the -Y skin is read
# as restrained in every downstream structural figure that depends on the
# bond class.
#
# 1e-9 mm (one nanometre) is the tolerance used here because it sits between
# the two things it must never be confused with: it is roughly eight orders of
# magnitude ABOVE the float64 rounding noise this artifact is made of (~1e-17
# mm at these magnitudes), so a larger rounding error cannot slip past it; and
# it is many orders of magnitude BELOW any physically or manufacturing-
# meaningful gap difference (the coupon measurements are tens of micrometres
# apart), so it can never mask a real, intended difference between two
# configs. A gap 100 um above a threshold (0.0001 mm = 1e-4 mm) is five orders
# of magnitude larger than this tolerance and must still, and does, classify
# into the worse band -- see
# ``tests/unit/test_web_tip_air_gap.py``'s
# ``test_100_micrometres_above_threshold_still_classifies_worse``.
_BOND_CLASS_EPSILON_MM = 1e-9

# Node-merge depth truncation threshold: a merged node's TRUNCATION
# RATIO is the merged slot's realised depth divided by its DEEPEST
# CONSTITUENT's own requested depth (see ``capability.py``'s
# ``_node_merge_truncation_ratio`` for exactly how "requested depth" is
# measured for a full-depth constituent, whose depth is not a fixed config
# number). A ratio of 1.0 means nothing was lost. Even an ALL-full-depth merge
# re-measured over the wider union interval can differ from either
# constituent's own narrow-span reading by curvature alone (the ~0.002 mm scale
# on the NACA 2412 fixture); this "re-measurement wobble" is sub-1% on every
# config seen so far.
#
# WHY 0.75, AND NOT A TIGHTER OR LOOSER NUMBER: the threshold's job is to
# separate that wobble population from a genuine depth-family mismatch (e.g. a
# 1.5 mm shallow-stiffener family crossing an 8 mm full-depth spar web
# truncates to a ratio of 1.5 / 8 = 0.1875). 0.75 sits an order of magnitude
# above the wobble noise floor (sub-1%) and comfortably below any real
# cross-family depth mismatch in the shipped configs -- a quarter of the
# deepest constituent's requested depth lost is already a real, actionable
# strength reduction (merging already halves the web thickness at every node;
# losing a quarter of one member's depth on top of that is not something a
# design should absorb silently). It is a chosen policy threshold, not a
# measured one (unlike the bond table), and is named here so it can be
# revisited against print/structural evidence.
MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO = 0.75

# Opposing-entry omission threshold (aircraft_core/generator.py's
# ``_omit_opposing_conflicts``): the FRACTION of a family's own candidate
# stations (built + omitted, for that family, summed across the whole run) that
# were omitted specifically for the "opposing entry:..." reason, above which
# the run-summary warning fires (see ``capability.py``'s
# ``_opposing_entry_warning_suffix``). A single opposing-entry omission is
# already visible in the diagnostics JSON and the per-layer ``omitted`` count
# (no requested feature is silently dropped), so this threshold is not a
# visibility gate -- it is what separates "one tapered station lost
# its opposing stiffener, unremarkable" from "this family's own anchor/pitch
# choice is losing so many stations to opposing collisions that the design
# itself, not one station, needs revisiting". 0.5 (half or more of a family's
# own candidates lost to this one cause) is a chosen policy threshold, not a
# measured one -- the same posture as
# ``MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO`` above.
OPPOSING_ENTRY_OMISSION_WARNING_RATIO = 0.5

# Role-yield omission threshold (aircraft_core/generator.py's
# ``_yield_lattice_to_primary``): the FRACTION of a
# ``skin_lattice`` family's own candidate stations (built + omitted, for
# that family, summed across the whole run) that were omitted specifically
# for the "role yield: ..." reason, above which the run-summary warning
# fires (see ``capability.py``'s ``_role_yield_warning_suffix``).
#
# THIS THRESHOLD'S SHAPE DIFFERS FROM ``OPPOSING_ENTRY_OMISSION_WARNING_
# RATIO`` ABOVE IN ONE IMPORTANT WAY, even though the number chosen is the
# same: a role-yield omission is not a defect signal at count > 0 the way an
# opposing-entry omission is. Every anisogrid-style config that combines a
# lattice with a primary member is EXPECTED to yield some lattice stations
# at primary crossings -- that is the purpose of ``_yield_lattice_to_primary``
# (stations are omitted so the primary member's own depth is never truncated
# by a merge). A warning
# gated on a bare nonzero count would therefore fire on every correctly
# -configured part combining the two roles and be worthless noise on the
# host-log line by the second slice. What actually indicates a problem is
# not that some stations yielded, but that a LARGE FRACTION of one lattice
# family's own candidates did -- evidence that family's placement (pitch,
# anchor, or depth) puts it in near-constant conflict with a primary member
# along its own run, i.e. the lattice is barely present at all where it
# crosses that member, not merely giving way at a handful of intersections.
#
# 0.5 (half or more of a family's own candidates lost to this one cause) is
# chosen to match ``OPPOSING_ENTRY_OMISSION_WARNING_RATIO``'s own number and
# reasoning -- both are chosen policy thresholds, not measured ones.
# Measured against the shipped anisogrid preset
# (``presets/plugin/anisogrid_lattice_config.json``) over its full 300-layer
# NACA 2412 fixture: the worst-affected lattice families (``lattice_plus_
# upper``/``lattice_minus_upper``) lose 85 of 4618 candidate stations each, a
# ratio of ~0.018 -- two orders of magnitude below this threshold, so a
# correctly configured part does not fire it.
ROLE_YIELD_OMISSION_WARNING_RATIO = 0.5


def _classify_web_tip_bond(gap_mm: float) -> str:
    """Classify a web-tip air gap against the measured bond table.

    See the module-level comment above this function for the table and the
    boundary reasoning. Boundaries are inclusive on the lower (better-bond)
    side, matching the printed values: exactly 0.021 mm is "fused", exactly
    0.042 mm is "weak", exactly 0.105 mm is "none" (measured 0.105 as no bond,
    not as the edge of marginal).

    Each comparison below is nudged by ``_BOND_CLASS_EPSILON_MM`` so that a gap
    float64 computes as a hair off an exact threshold still lands in the class
    THAT THRESHOLD ITSELF belongs to, per the table above -- not always "add
    epsilon to the upper bound": the direction depends on which class the exact
    threshold value belongs to.

    - 0.021 mm and 0.042 mm both belong to the band BELOW them ("fused" and
      "weak" respectively, inclusive-<= readings), so a
      computed gap a hair ABOVE either (e.g. the 0.021 mm hard-fuse
      point, computed as ``0.021000000000000019`` -- see the module-level
      constant's comment) must still fall into that lower band: both comparisons widen their upper bound
      OUTWARD by epsilon (``<= threshold + epsilon``).
    - 0.105 mm belongs to "none" (measured 0.105 mm itself as no
      bond), not to "marginal" -- the opposite direction from the two
      thresholds above. So a computed gap a hair BELOW 0.105 mm (which
      would otherwise wrongly fall into "marginal") must still fall into
      "none": the marginal comparison's upper bound is narrowed INWARD by
      epsilon (``< threshold - epsilon``), so only gaps genuinely,
      non-noise below 0.105 mm land in "marginal".

    Either way, the effect is the same: a value indistinguishable from an exact
    threshold (within ``_BOND_CLASS_EPSILON_MM``) is classified exactly as that
    threshold value itself is classified in the measured table, regardless of
    which side of it float64 happened to round to.
    """
    if gap_mm <= _BOND_FUSED_MAX_MM + _BOND_CLASS_EPSILON_MM:
        return "fused"
    if gap_mm <= _BOND_WEAK_MAX_MM + _BOND_CLASS_EPSILON_MM:
        return "weak"
    if gap_mm < _BOND_MARGINAL_MAX_MM - _BOND_CLASS_EPSILON_MM:
        return "marginal"
    return "none"


@dataclass(frozen=True)
class RoleTotals:
    """One structural role's (``skin_lattice`` / ``primary_structure``)
    aggregate slot/omission tally for the whole run — see
    ``RunDiagnostics.role_totals``.

    Mirrors ``OpposingEntryFamilyRatio``'s shape (plain accumulated counts,
    no per-layer detail) for the same reason: this is a run-wide summary a
    caller reads once, not a per-layer record.
    """

    slots: int
    merged_slots: int
    omitted: int
    opposing_entry_omitted: int
    role_yield_omitted: int


@dataclass(frozen=True)
class OpposingEntryFamilyRatio:
    """One family's opposing-entry omission tally for the whole run — see
    ``RunDiagnostics.opposing_entry_family_ratios``."""

    opposing_entry_omitted: int
    total_candidates: int

    @property
    def ratio(self) -> float:
        """Fraction of this family's own candidate stations lost to the
        "opposing entry: ..." reason this run. ``total_candidates`` is
        guaranteed > 0 by ``opposing_entry_family_ratios`` (a family with no
        candidates is simply absent from that dict, never a 0/0 entry
        here)."""
        return self.opposing_entry_omitted / self.total_candidates


@dataclass(frozen=True)
class RoleYieldFamilyRatio:
    """One (``skin_lattice``) family's role-yield omission tally for the
    whole run — see ``RunDiagnostics.role_yield_family_ratios``. Mirrors
    ``OpposingEntryFamilyRatio``'s shape exactly, for the same reason: a
    run-wide per-family summary a caller reads once."""

    role_yield_omitted: int
    total_candidates: int

    @property
    def ratio(self) -> float:
        """Fraction of this family's own candidate stations lost to the
        "role yield: ..." reason this run. ``total_candidates`` is
        guaranteed > 0 by ``role_yield_family_ratios`` (a family with no
        candidates is simply absent from that dict, never a 0/0 entry
        here)."""
        return self.role_yield_omitted / self.total_candidates


@dataclass
class LayerRecord:
    """One layer's outcome: mutated, failed, or left alone, with context.

    ``extra`` carries anything caller-specific (areas, slot count, etc.) and
    must itself already be JSON-serializable — this module does not impose a
    schema on it.
    """

    layer_index: int
    print_z: float
    mutated: bool = False
    failed: bool = False
    message: str = ""
    extra: Dict[str, Any] = field(default_factory=dict)
    # True when this layer was left unmutated on purpose
    # because every candidate web was omitted for a THINNESS reason (see
    # aircraft_core/generator.py's OmittedWeb.reason docstring) and
    # StructureSpec.tip_shutdown_enabled is True — never set together with
    # ``failed`` (a shutdown layer is a recorded, deliberate non-failure,
    # distinct from both an ordinary mutated body layer and a fatal
    # TopologyError).
    shutdown: bool = False
    # Structural features DROPPED on this layer so it could still be built,
    # each as {"kind", "x_mm", "reason"}. A mutated layer with a non-empty list
    # here PRINTED, but not with everything that was asked for. Empty on every
    # ordinary layer.
    #
    # This is the reporting half of degradation and it is not optional:
    # degrading is allowed only when the layer is explicitly marked degraded. A
    # dropped feature that does not reach the report is a silent fallback.
    degradations: List[dict] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        """Plain, JSON-ready dict."""
        return {
            "layer_index": self.layer_index,
            "print_z": self.print_z,
            "mutated": self.mutated,
            "failed": self.failed,
            "shutdown": self.shutdown,
            "degradations": list(self.degradations),
            "message": self.message,
            "extra": dict(self.extra),
        }


class RunDiagnostics:
    """Accumulates ``LayerRecord``s across one slice and tracks the first failure.

    The fail-closed policy means a run normally stops at its first failure,
    but this class does not enforce that itself — it is a
    recorder, not a controller. It captures whichever failure is reported
    first *to it*, in call order, regardless of whether the caller keeps
    going.
    """

    def __init__(self) -> None:
        self._records: List[LayerRecord] = []
        self._first_failure: Optional[LayerRecord] = None
        # mirrors ``_first_failure`` for shutdown layers — captures whichever
        # shutdown is reported first *to it*, in call order, same "recorder,
        # not a controller" contract as the failure tracking above.
        self._first_shutdown: Optional[LayerRecord] = None
        # Accumulators feeding ``structure_summary`` below. Populated by
        # ``set_skin_line_width_mm``/``record_structure_layer``, which the
        # capability calls once per resolved config and once per
        # successfully-generated (region, layer) respectively — see those
        # methods' own docstrings for exactly what each accumulates and why
        # this stays a recorder, not a controller, same as the rest of this
        # class.
        self._skin_line_width_mm: Optional[float] = None
        self._total_web_length_mm: float = 0.0
        self._total_skin_length_mm: float = 0.0
        self._total_web_count: int = 0
        self._bay_gaps_mm: List[float] = []
        self._worst_skin_bay_mm: Optional[float] = None
        self._worst_skin_bay_layer_index: Optional[int] = None
        self._worst_skin_bay_print_z: Optional[float] = None
        # Web-tip air gap: ``spec.skin_margin_mm`` as REQUESTED, set once
        # per run alongside ``_skin_line_width_mm`` -- see
        # ``set_skin_margin_mm``'s own docstring for why this is the
        # user-facing requested value rather than any per-layer, rotation
        # -corrected reading.
        self._skin_margin_mm: Optional[float] = None
        # How many of this run's surviving webs are FULL-DEPTH (their far
        # end is placed against the -Y skin rather than stopped short by a
        # requested ``web_depth_mm``) -- the population the tip-gap finding
        # actually applies to. See ``record_structure_layer``'s
        # ``full_depth_web_count`` parameter docstring for exactly how a
        # web is classified.
        self._full_depth_web_count: int = 0
        # Node merges are a per-layer COUNT (a layer can carry several merged
        # nodes at once), not a whole-layer boolean like ``shutdown``, so
        # they are derived from ``extra`` in ``node_merge_count``/
        # ``first_node_merge_z`` below rather than tracked as a
        # ``LayerRecord`` field. Every node merges, and a merge is recorded as
        # present via a ``WebIntent.merged_family_indices`` tuple, never as an
        # omission, so there is no node-suppression count.
        # Depth-truncation visibility: tracks the WORST (lowest) truncation
        # ratio seen across every merged node this run recorded via
        # ``record_node_merge_truncation`` -- see that method's and
        # ``worst_depth_truncation_ratio``'s docstrings.
        self._worst_depth_truncation_ratio: Optional[float] = None
        self._worst_depth_truncation_layer_index: Optional[int] = None
        self._worst_depth_truncation_print_z: Optional[float] = None
        self._truncated_merge_count: int = 0

    def record(self, layer_record: LayerRecord) -> None:
        """Append one layer's outcome. Captures the first failure, once."""
        self._records.append(layer_record)
        if layer_record.failed and self._first_failure is None:
            self._first_failure = layer_record
        if layer_record.shutdown and self._first_shutdown is None:
            self._first_shutdown = layer_record

    @property
    def records(self) -> List[LayerRecord]:
        return list(self._records)

    @property
    def mutated_count(self) -> int:
        return sum(1 for r in self._records if r.mutated)

    @property
    def failed_count(self) -> int:
        return sum(1 for r in self._records if r.failed)

    @property
    def shutdown_count(self) -> int:
        return sum(1 for r in self._records if r.shutdown)

    @property
    def degraded_layer_count(self) -> int:
        """Layers that PRINTED but not with everything that was asked for.
        Counted separately from ``shutdown_count`` (a layer left wholly
        unmutated) and from ``failed_count`` (a layer that refused): a degraded
        layer produced structure, just less of it.

        DO NOT QUOTE THIS AS "layers that lost structure". It counts layers
        carrying ANY degradation, including ``spar_socket_latched``
        suppressions -- layers where the section could perfectly well have
        held a socket, and the socket was withheld because it had already
        ended further inboard. The loss happened elsewhere. Read
        ``degradation_counts_by_kind`` for the breakdown."""
        return sum(1 for r in self._records if r.degradations)

    @property
    def degradation_counts_by_kind(self) -> Dict[str, int]:
        """How many layers dropped each kind of feature. Kind-level, not
        layer-level, so a reader can tell "the spar socket ended 300 layers
        early" from "300 unrelated things went wrong"."""
        out: Dict[str, int] = {}
        for r in self._records:
            for d in r.degradations:
                k = str(d.get("kind", "?"))
                out[k] = out.get(k, 0) + 1
        return out

    @property
    def pair_ablation_rescue_layers(self) -> List[Dict[str, Any]]:
        """Layers that failed the island-split check and were rescued by removing
        ONE opposing-pair station. Each entry names the layer index, Z, family,
        station, the members the validator's ablation implicated, and the extra
        elapsed ms the retry cost. These layers are also degraded (kind
        pair_ablation)."""
        out: List[Dict[str, Any]] = []
        for r in self._records:
            rec = (r.extra or {}).get("pair_ablation")
            if rec:
                out.append({
                    "layer_index": r.layer_index,
                    "print_z": r.print_z,
                    "family_index": rec.get("family_index"),
                    "family_name": rec.get("family_name"),
                    "station_x_mm": rec.get("station_x_mm"),
                    "implicated_members": rec.get("implicated_members"),
                    "elapsed_ms": rec.get("elapsed_ms"),
                })
        return out

    @property
    def pair_ablation_rescue_count(self) -> int:
        return len(self.pair_ablation_rescue_layers)

    @property
    def pinched_section_layers(self) -> List[Dict[str, Any]]:
        """Layers shut down (left as plain skin, unmutated) because the wing's own
        bare cross-section splits under the wall-safety erosion. Each entry
        carries the layer index, Z, piece count/areas, the minimum neck erosion
        radius and the recorded reason."""
        out: List[Dict[str, Any]] = []
        for r in self._records:
            rec = (r.extra or {}).get("pinched_section")
            if rec:
                out.append(dict(rec))
        return out

    @property
    def pinched_section_count(self) -> int:
        return len(self.pinched_section_layers)

    @property
    def first_degradation(self) -> Optional[LayerRecord]:
        for r in self._records:
            if r.degradations:
                return r
        return None

    @property
    def first_failure(self) -> Optional[LayerRecord]:
        return self._first_failure

    @property
    def first_shutdown(self) -> Optional[LayerRecord]:
        return self._first_shutdown

    @property
    def node_merge_count(self) -> int:
        """Total node MERGES across the whole run, summed from each layer's own
        ``extra["node_merge_count"]`` (the capability populates this by
        counting ``WebIntent`` entries whose ``merged_family_indices`` is
        non-empty). A layer that never set this key contributes 0. Same "recorded loudly,
        aggregated in the run report" posture as ``shutdown_count`` above — a
        merge is a real, visible geometric event (one wider slot instead of
        two), not merely an absence of omission, and this is what makes it show
        up in the run report without opening a per-layer ``LayerRecord``."""
        return sum(r.extra.get("node_merge_count", 0) for r in self._records)

    @property
    def first_node_merge_z(self) -> Optional[float]:
        """Z height of the first layer (in record/call order) whose
        ``node_merge_count`` is nonzero, or ``None`` if none occurred —
        mirrors ``first_shutdown``'s role for ``shutdown_count``."""
        for r in self._records:
            if r.extra.get("node_merge_count", 0) > 0:
                return r.print_z
        return None

    @property
    def order_swap_count(self) -> int:
        """Total GENUINE order-swap events across the whole run, summed
        from each layer's own ``extra["order_swap_count"]`` (the capability
        populates this via ``_classify_declared_pair_overlaps`` — a
        disjoint-membership, measurably-overlapping pair of a declared diamond
        +/- family pair, landing on DIFFERENT march indices, which the
        same-edge collision rule accepts as an "intended junction" without
        distinguishing it from the designed same-index apex). Same "recorder,
        not a controller" posture as ``node_merge_count`` above — reporting
        only; it does not change the geometry ``validate()``/``_transform``
        produce for the layer it is recorded on. A layer that never set this
        key contributes 0."""
        return sum(r.extra.get("order_swap_count", 0) for r in self._records)

    @property
    def order_swap_events(self) -> List[dict]:
        """Every recorded order-swap event, in layer order, each carrying
        enough to locate it (``layer_index``, ``print_z``, ``families``,
        ``chord_station_mm``, ``entry_lower``) — see
        ``_classify_declared_pair_overlaps``'s own docstring for the event
        shape. Empty if none occurred."""
        events: List[dict] = []
        for r in self._records:
            events.extend(r.extra.get("order_swap_events", ()))
        return events

    @property
    def first_order_swap_z(self) -> Optional[float]:
        """Z height of the first layer (in record/call order) whose
        ``order_swap_count`` is nonzero, or ``None`` if none occurred —
        mirrors ``first_node_merge_z``'s role for ``node_merge_count``."""
        for r in self._records:
            if r.extra.get("order_swap_count", 0) > 0:
                return r.print_z
        return None

    @property
    def chord_axis_unavailable_layer_count(self) -> int:
        """Total body layers processed while this run's GLOBAL alignment classification was
        ``ChordAxisUnavailableError`` -- no chord axis exists at all (no hull
        area, or no genuine trailing edge anywhere on the mid-span station), so
        theta was treated as 0/no-rotation, non-fatally, rather than failing
        the slice. Correct for a non-aerofoil structural coupon (the plugin is
        supposed to handle those); WRONG, silently, for a genuine wing whose
        mid-span section happens to be degenerate (e.g. a flat-cut root reused
        as the sampled station).

        Summed from each layer's own ``extra["chord_axis_unavailable"]``
        (the capability sets this True for every mutated layer for the
        whole run when ``_build_mesh_sectioner`` classified the run this
        way), same "recorder, not a controller" posture as
        ``node_merge_count``/``order_swap_count`` above. This field
        mitigates the residual risk of the non-fatal classification: a silent
        classification with no report field would hide a degenerate mid-span
        station. A nonzero count here, on a model that was SUPPOSED to be a wing,
        is the signal that the mid-span (or fallback) station could not be
        read and needs attention -- see
        ``chord_axis_unavailable_first_z``."""
        return sum(
            1 for r in self._records
            if r.mutated and r.extra.get("chord_axis_unavailable", False)
        )

    @property
    def chord_axis_unavailable(self) -> bool:
        """Top-level flag: ``True`` iff
        ``chord_axis_unavailable_layer_count > 0`` for this run. Reported
        alongside the count (not instead of it), in the same style as
        ``order_swap_count``."""
        return self.chord_axis_unavailable_layer_count > 0

    @property
    def chord_axis_unavailable_first_z(self) -> Optional[float]:
        """Z height of the first layer (in record/call order) whose
        ``extra["chord_axis_unavailable"]`` is set, or ``None`` if none
        occurred -- mirrors ``first_node_merge_z``'s role for
        ``node_merge_count``."""
        for r in self._records:
            if r.mutated and r.extra.get("chord_axis_unavailable", False):
                return r.print_z
        return None

    @property
    def opposing_entry_omission_count(self) -> int:
        """Total candidate stations omitted for the "opposing entry:..." reason
        (``aircraft_core/generator.py``'s ``_omit_opposing_conflicts``) across
        the whole run, summed from each layer's own
        ``extra["opposing_entry_omission_count"]`` — the same "recorder, not a
        controller, sums a per-layer count the capability already computed"
        shape as ``node_merge_count`` above. A layer that never set this key
        contributes 0."""
        return sum(
            r.extra.get("opposing_entry_omission_count", 0) for r in self._records
        )

    @property
    def role_yield_omission_count(self) -> int:
        """Total candidate stations omitted for the "role yield: ..."
        reason (``aircraft_core/generator.py``'s
        ``_yield_lattice_to_primary``) across the whole
        run, summed from each layer's own
        ``extra["role_yield_omission_count"]`` — the same "recorder, not a
        controller, sums a per-layer count the capability already computed"
        shape as ``opposing_entry_omission_count`` above. A layer that
        never set this key contributes 0."""
        return sum(
            r.extra.get("role_yield_omission_count", 0) for r in self._records
        )

    def role_yield_family_ratios(self) -> Dict[str, "RoleYieldFamilyRatio"]:
        """Per-family role-yield omission ratio, aggregated across the
        WHOLE RUN from each layer's own ``extra["families"]`` breakdown —
        the exact same source and shape ``opposing_entry_family_ratios``
        already reads, substituting ``"role_yield_omitted"`` for
        ``"opposing_entry_omitted"``.

        A family's DENOMINATOR here is every candidate station THAT FAMILY
        produced this run, survived or not (``slots + omitted``, summed
        across every layer) — not merely role-yield omissions, so the ratio
        reads "what fraction of this family's own output was lost to this
        one cause", matching ``opposing_entry_family_ratios``'s own
        docstring.

        Returns a plain dict keyed by the same family key
        ``capability.py``'s own ``_family_key`` uses. A family with zero
        candidates this run is simply absent, not reported as a 0/0 ratio —
        same convention as ``opposing_entry_family_ratios``. In practice
        only ``skin_lattice`` families ever carry a nonzero numerator here
        (``_yield_lattice_to_primary`` always yields the lattice side of a
        cross-role pair, never the ``primary_structure`` side), but this
        method does not assume that — it reads whatever the breakdown
        recorded.
        """
        totals: Dict[str, List[int]] = {}
        for r in self._records:
            families = r.extra.get("families")
            if not families:
                continue
            for key, entry in families.items():
                bucket = totals.setdefault(key, [0, 0])
                bucket[0] += entry.get("role_yield_omitted", 0)
                bucket[1] += entry.get("slots", 0) + entry.get("omitted", 0)
        return {
            key: RoleYieldFamilyRatio(
                role_yield_omitted=role_yield_count,
                total_candidates=total,
            )
            for key, (role_yield_count, total) in totals.items()
            if total > 0
        }

    def opposing_entry_family_ratios(self) -> Dict[str, "OpposingEntryFamilyRatio"]:
        """Per-family opposing-entry omission ratio, aggregated across the
        WHOLE RUN from each layer's own ``extra["families"]`` breakdown
        (``capability.py`` already builds one ``families_breakdown`` dict
        per layer for the "families" diagnostics field; this re-sums its
        ``"slots"``/``"omitted"``/``"opposing_entry_omitted"`` keys rather
        than re-deriving them a second way, as ``truncated_merge_count``
        does with ``_node_merge_truncation_ratio``'s numbers).

        A family's DENOMINATOR here is every candidate station THAT FAMILY
        produced this run, survived or not (``slots + omitted``, summed
        across every layer) — not merely opposing-entry omissions, so the
        ratio genuinely reads "what fraction of this family's own output
        was lost to this one cause", not "opposing-entry omissions as a
        fraction of opposing-entry omissions".

        Returns a plain dict keyed by the SAME family key
        ``capability.py``'s own ``_family_key`` already uses (``"{index}
        {name}"``) — a family with zero candidates this run (never reached,
        e.g. every layer failed before generation) is simply absent, not
        reported as a 0/0 ratio.
        """
        totals: Dict[str, List[int]] = {}
        for r in self._records:
            families = r.extra.get("families")
            if not families:
                continue
            for key, entry in families.items():
                bucket = totals.setdefault(key, [0, 0])
                bucket[0] += entry.get("opposing_entry_omitted", 0)
                bucket[1] += entry.get("slots", 0) + entry.get("omitted", 0)
        return {
            key: OpposingEntryFamilyRatio(
                opposing_entry_omitted=opposing_count,
                total_candidates=total,
            )
            for key, (opposing_count, total) in totals.items()
            if total > 0
        }

    def role_totals(self) -> Dict[str, "RoleTotals"]:
        """Per-ROLE (``skin_lattice`` / ``primary_structure``) slot/omission
        totals, aggregated across the WHOLE RUN from each layer's own
        ``extra["families"]`` breakdown -- the same source and shape
        ``opposing_entry_family_ratios`` already reads (``capability.py``
        stamps a ``"role"`` key onto every ``families_breakdown`` entry, one
        per family, using that family's own DECLARED
        ``aircraft_core.spec.WebFamily.role`` -- never inferred here).

        A family that never appeared this run (e.g. every layer failed
        before generation) contributes nothing; a role with zero families
        present is simply absent from the returned dict, matching
        ``opposing_entry_family_ratios``'s own "absent, not a 0/0 entry"
        convention. Two families sharing a role sum together, so this
        answers "how much of this run's structure was skin_lattice versus
        primary_structure", not "which family".
        """
        totals: Dict[str, List[int]] = {}
        for r in self._records:
            families = r.extra.get("families")
            if not families:
                continue
            for entry in families.values():
                role = entry.get("role")
                if role is None:
                    continue
                bucket = totals.setdefault(role, [0, 0, 0, 0, 0])
                bucket[0] += entry.get("slots", 0)
                bucket[1] += entry.get("merged_slots", 0)
                bucket[2] += entry.get("omitted", 0)
                bucket[3] += entry.get("opposing_entry_omitted", 0)
                bucket[4] += entry.get("role_yield_omitted", 0)
        return {
            role: RoleTotals(
                slots=slots,
                merged_slots=merged_slots,
                omitted=omitted,
                opposing_entry_omitted=opposing_entry_omitted,
                role_yield_omitted=role_yield_omitted,
            )
            for role, (
                slots,
                merged_slots,
                omitted,
                opposing_entry_omitted,
                role_yield_omitted,
            ) in totals.items()
        }

    def record_node_merge_truncation(
        self, layer_index: int, print_z: float, ratio: float
    ) -> None:
        """Record one merged node's TRUNCATION RATIO: the merged slot's realised
        depth divided by its deepest constituent's own requested depth (see
        ``MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO``'s module-level comment for
        the definition and the threshold's justification, and
        ``capability.py``'s ``_node_merge_truncation_ratio`` for exactly how
        ``ratio`` is computed — this method only accumulates an
        already-computed number, the same "recorder, not a controller" split as
        the rest of this class).

        Called once per merged ``WebIntent`` whose ratio could actually be
        computed. A merged node whose ratio was NOT computable (e.g. a
        full-depth constituent's local geometry re-measurement itself
        raised) is simply never passed here — this method does not guess a
        substitute value, matching every other field in this class.

        Tracks the WORST (lowest) ratio seen across the whole run, and
        counts every merge whose ratio falls below
        ``MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO`` — a STRICT ``<``
        comparison, so a merge landing EXACTLY on the threshold does not
        count as truncated (see ``truncated_merge_count``'s own docstring
        for why the boundary is drawn this way).
        """
        if (
            self._worst_depth_truncation_ratio is None
            or ratio < self._worst_depth_truncation_ratio
        ):
            self._worst_depth_truncation_ratio = ratio
            self._worst_depth_truncation_layer_index = layer_index
            self._worst_depth_truncation_print_z = print_z
        if ratio < MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO:
            self._truncated_merge_count += 1

    @property
    def worst_depth_truncation_ratio(self) -> Optional[float]:
        """The lowest merge truncation ratio recorded this run, or ``None`` if no
        merge's ratio was ever recorded (either no merge occurred, or every
        merge's ratio was uncomputable — see ``record_node_merge_truncation``).
        1.0 means nothing was lost; e.g. 0.2 means the deepest constituent kept
        a fifth of its requested depth at that node."""
        return self._worst_depth_truncation_ratio

    @property
    def worst_depth_truncation_layer_index(self) -> Optional[int]:
        """0-based body-layer index of the layer carrying
        ``worst_depth_truncation_ratio``, or ``None`` under the same
        condition as that property."""
        return self._worst_depth_truncation_layer_index

    @property
    def worst_depth_truncation_print_z(self) -> Optional[float]:
        """Z height (mm) of the layer carrying
        ``worst_depth_truncation_ratio``, or ``None`` under the same
        condition as that property."""
        return self._worst_depth_truncation_print_z

    @property
    def truncated_merge_count(self) -> int:
        """Count of merges (across the whole run) whose truncation ratio fell
        BELOW ``MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO`` — see that constant's
        module-level comment for what the threshold separates and why 0.75 was
        chosen. A merge whose ratio lands EXACTLY on the threshold is not
        counted (strict ``<``); always an int, never ``None`` — 0 is a truthful
        "no merge lost a quarter or more of its deepest constituent's depth",
        not a missing-data marker, matching ``full_depth_web_count``'s own
        convention."""
        return self._truncated_merge_count

    # -- structural summary -------------------------------------------

    def set_skin_line_width_mm(self, width_mm: float) -> None:
        """Record the resolved outer wall line width actually in force for
        this run — the ``skin_line_width_mm`` field of ``structure_summary``.

        Called once per run (the capability resolves this once, before the
        layer walk begins), not per layer — a later call simply overwrites
        an earlier one, which is harmless since the value is constant for
        the whole run in practice (``outer_wall_line_width`` is a single
        resolved process setting, not something that varies layer to
        layer).
        """
        self._skin_line_width_mm = width_mm

    def set_skin_margin_mm(self, margin_mm: float) -> None:
        """Record ``spec.skin_margin_mm`` -- the REQUESTED skin margin, as
        the user configured it -- for the web-tip air gap calculation.

        Called once per run, same calling convention as
        ``set_skin_line_width_mm`` (the capability resolves this once, before
        the layer walk begins). Deliberately the raw ``spec.skin_margin_mm``,
        not any per-layer ``effective_skin_margin_mm`` (rotation-corrected
        reading -- see ``generator.py``'s ``_generate_family`` docstring) —
        this mirrors that same docstring's own convention that "diagnostic
        MESSAGES still cite ``spec.skin_margin_mm`` (the user's actual request)
        for readability". A rotated section's TRUE placement may differ layer
        to layer; this field is the one constant the user actually typed in,
        and is what a reported gap should be read against. See
        ``web_tip_air_gap_mm`` for how this is used.
        """
        self._skin_margin_mm = margin_mm

    def record_structure_layer(
        self,
        layer_index: int,
        print_z: float,
        skin_length_mm: float,
        web_depths_mm: List[float],
        entry_x_mm: List[float],
        full_depth_web_count: int = 0,
        entry_lower_flags: Optional[List[bool]] = None,
    ) -> None:
        """Accumulate one (layer, region)'s contribution to the structural
        summary. Called once per successfully-generated region — a region that
        fails closed or shuts down (tip shutdown) contributes nothing, which is
        correct: there is no wall geometry to measure for either.

        THE APPROXIMATION (stated here, not just in ``structure_summary``'s
        docstring, because it governs what these arguments must mean): this is
        a proxy for wall LENGTH at constant line width, not a true mass. A web
        is modelled as contributing ``2 * depth`` of wall length — the
        perimeter detours in along one wall, around the end cap, and back out
        the other — and the skin contributes the layer's own outer-perimeter length, read
        from the UN-slotted contour. Neither term accounts for line width,
        extrusion volume, or the perimeter length actually lost to the slot's
        own footprint (the "skin" reading double-counts the short span the slot
        cuts through); at constant line width, wall length is directly
        proportional to extruded mass, so the ratio between the two terms is
        the quantity ``web_mass_fraction_est`` reports, not either term's
        absolute value.

        Args:
            layer_index: 0-based body-layer index, for the worst-bay
                location report.
            print_z: This layer's Z height in millimetres, same role.
            skin_length_mm: Perimeter length, in millimetres, of this
                region's UN-slotted (pre-mutation) outer contour.
            web_depths_mm: This region's per-web depth (centerline
                entry-to-far-end distance), one entry per physical slot
                actually built here — i.e. one per surviving ``WebIntent``,
                already collapsed for a merge (a merged node is one
                physical slot, and contributes one depth here, exactly as
                it is one physical slot everywhere else this project counts
                slots).
            entry_x_mm: This region's per-web entry-point X coordinate
                (chord station where each web meets the exterior boundary),
                same order and count as ``web_depths_mm``. Adjacent (sorted)
                entries bound one unsupported skin bay each; a region with
                fewer than two webs contributes no bay measurement at all
                (there is no "between" with 0 or 1 entry points), which is
                correct, not a gap in the accounting.
            full_depth_web_count: How many of THIS region's surviving webs
                are FULL-DEPTH -- i.e. their far end is placed against the
                -Y skin by ``generator.py``'s
                ``end_y = margin_y + FULL_DEPTH_MARGIN_HEADROOM_MM``
                formula, rather than stopped short by a requested
                ``web_depth_mm``. This is the population the web-tip air
                gap (``web_tip_air_gap_mm``/``web_tip_bond_class``) applies
                to: a depth-limited web does not reach the far skin BY
                REQUEST, so it is never a tip-bond defect and must not be
                counted here. Default 0.
        """
        web_length_mm = sum(2.0 * depth for depth in web_depths_mm)
        self._total_web_length_mm += web_length_mm
        self._total_skin_length_mm += skin_length_mm
        self._total_web_count += len(web_depths_mm)
        self._full_depth_web_count += full_depth_web_count

        # Bays are measured WITHIN one entry surface, never across both.
        #
        # An unsupported skin bay is a span of ONE skin between two webs that
        # enter THAT skin; a web entering the opposite surface does not
        # shorten it. Pooling both surfaces' stations into one list would
        # report roughly HALF the true per-surface bay whenever the two
        # surfaces' stations interleave -- and since the flat-plate critical
        # stress goes as ``1/b^2``, a 2x understated bay is a ~4x OVERSTATED
        # ``sigma_cr``, in the unsafe direction. When every family is
        # co-located in X the pooled list is effectively one surface's list
        # repeated and the adjacent differences come out right regardless.
        #
        # ``entry_lower_flags`` is optional; when it is absent (or its length
        # does not match ``entry_x_mm``) the pooled scan runs, because a
        # caller that cannot say which surface a station entered cannot
        # support the per-surface reading.
        #
        # KNOWN LIMITATION: this measures chord
        # DX, while the buckling formula it feeds wants skin ARCLENGTH along
        # the surface. Near the leading edge arclength is several times the
        # chord DX, so the bay stays understated and ``sigma_cr`` overstated
        # exactly where curvature is highest. Correcting that needs the
        # layer POLYGON, which this module does not have and should not
        # acquire -- ``RunDiagnostics`` is an accumulator, not a geometry
        # consumer. ``tools/size_wing.py`` computes the arclength bay itself
        # for exactly this reason. Anyone quoting this number for a
        # structural claim must account for that residual.
        if entry_lower_flags is not None and len(entry_lower_flags) == len(entry_x_mm):
            groups = (
                [x for x, lower in zip(entry_x_mm, entry_lower_flags) if not lower],
                [x for x, lower in zip(entry_x_mm, entry_lower_flags) if lower],
            )
        else:
            groups = (list(entry_x_mm),)

        for group in groups:
            xs = sorted(group)
            for lo, hi in zip(xs, xs[1:]):
                gap = hi - lo
                self._bay_gaps_mm.append(gap)
                if self._worst_skin_bay_mm is None or gap > self._worst_skin_bay_mm:
                    self._worst_skin_bay_mm = gap
                    self._worst_skin_bay_layer_index = layer_index
                    self._worst_skin_bay_print_z = print_z

    @property
    def web_mass_fraction_est(self) -> Optional[float]:
        """Estimated fraction of total accumulated wall LENGTH (not mass —
        see ``record_structure_layer``'s docstring) that is web rather than
        skin, summed over every mutated layer/region this run has recorded.
        ``None`` if nothing has been recorded yet (division by zero would
        otherwise silently report 0.0, which is a different claim than "no
        data")."""
        total = self._total_web_length_mm + self._total_skin_length_mm
        if total <= 0:
            return None
        return self._total_web_length_mm / total

    @property
    def mean_web_spacing_mm(self) -> Optional[float]:
        """Mean of every adjacent-entry-point bay gap recorded across the
        whole run (not the worst-case single gap — see
        ``worst_skin_bay_mm`` for that). ``None`` if no layer/region ever
        had two or more webs to measure a gap between."""
        if not self._bay_gaps_mm:
            return None
        return sum(self._bay_gaps_mm) / len(self._bay_gaps_mm)

    @property
    def full_depth_web_count(self) -> int:
        """Total number of surviving webs, across the whole run, whose far
        end reaches the -Y skin (see ``record_structure_layer``'s
        ``full_depth_web_count`` parameter docstring). Always an int, never
        ``None`` — 0 is a legitimate, truthful answer for a part with no
        full-depth webs (e.g. every family configured shallow), distinct
        from "not measured yet"."""
        return self._full_depth_web_count

    @property
    def web_tip_air_gap_mm(self) -> Optional[float]:
        """The AIR GAP, in millimetres, between a full-depth web's tip bead and
        the -Y skin's own bead — the quantity the measured bond table is
        indexed by.

        ``None`` if either ``set_skin_margin_mm`` or
        ``set_skin_line_width_mm`` has not yet been called this run — this
        property never guesses past missing data, matching every other
        field in ``structure_summary``.

        THE ARITHMETIC, and why it is NOT the naive ``skin_margin_mm - 2 *
        line_width``: ``generator.py``'s ``_generate_family`` places a
        full-depth web's far end at ``end_y = y_min + effective_skin_margin_mm
        + FULL_DEPTH_MARGIN_HEADROOM_MM`` — i.e. it adds
        ``FULL_DEPTH_MARGIN_HEADROOM_MM`` ON TOP OF the requested margin (see
        that constant's own comment in ``generator.py`` for why a bare margin
        can legitimately fail ``validate.py``'s independent re-measurement even
        on a genuinely thick section). That headroom pushes the web tip further
        from the skin, not closer, so the REAL gap is ``skin_margin_mm +
        FULL_DEPTH_MARGIN_HEADROOM_MM - 2 * line_width``, which is LARGER than
        the naive formula by ``FULL_DEPTH_MARGIN_HEADROOM_MM`` (2e-4 mm — negligible against the 0.021 mm
        fuse point, but computed from the actual placed geometry rather than
        assumed, to avoid reporting an optimistic number). The constant is
        imported from ``generator.py`` rather than re-derived, so the two can
        never drift (see the import comment above
        ``FULL_DEPTH_MARGIN_HEADROOM_MM``).

        Uses ``skin_margin_mm`` and ``skin_line_width_mm`` exactly as
        recorded — see ``set_skin_margin_mm``'s docstring for why that is
        the REQUESTED value, not a per-layer rotation-corrected reading.
        """
        if self._skin_margin_mm is None or self._skin_line_width_mm is None:
            return None
        return (
            self._skin_margin_mm
            + FULL_DEPTH_MARGIN_HEADROOM_MM
            - 2.0 * self._skin_line_width_mm
        )

    @property
    def web_tip_bond_class(self) -> Optional[str]:
        """``web_tip_air_gap_mm`` classified against the MEASURED bond table
        (see the module-level comment above ``_classify_web_tip_bond`` for the
        table and thresholds). ``None`` under the same missing-data condition
        as ``web_tip_air_gap_mm``.

        These thresholds are printed-coupon measurements at PLA 210 C on a
        0.42 mm outer wall — NOT modelled, simulated, or derived from any
        material datasheet — and are specific to that
        temperature/material/nozzle combination; they may not transfer as-is
        to a different one.
        """
        gap = self.web_tip_air_gap_mm
        if gap is None:
            return None
        return _classify_web_tip_bond(gap)

    def structure_summary(self) -> Dict[str, Any]:
        """Plain, JSON-ready dict of this run's structural measurements: skin line
        width actually in force, an estimated web mass fraction (wall-length
        proxy, see ``record_structure_layer``'s docstring), the worst (largest)
        unsupported skin bay seen anywhere in the run and where, the total
        physical web count, the mean bay spacing, and the web-tip air gap /
        bond classification / full-depth web count (see ``web_tip_air_gap_mm``,
        ``web_tip_bond_class``, and ``full_depth_web_count`` for what each
        means and why), and the merged-node depth-truncation figures
        (``worst_depth_truncation_ratio`` and its layer/Z, and
        ``truncated_merge_count`` — see those properties' own docstrings and
        ``MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO``'s module-level comment).
        Every numeric field is ``None`` if nothing was ever recorded for it,
        rather than a misleading 0 — this method never guesses past missing
        data. ``full_depth_web_count`` and ``truncated_merge_count`` are the
        exceptions, matching ``total_web_count``: 0 is a truthful count, not a
        missing-data marker."""
        return {
            "skin_line_width_mm": self._skin_line_width_mm,
            "web_mass_fraction_est": self.web_mass_fraction_est,
            "worst_skin_bay_mm": self._worst_skin_bay_mm,
            "worst_skin_bay_layer_index": self._worst_skin_bay_layer_index,
            "worst_skin_bay_print_z": self._worst_skin_bay_print_z,
            "total_web_count": self._total_web_count,
            "mean_web_spacing_mm": self.mean_web_spacing_mm,
            "web_tip_air_gap_mm": self.web_tip_air_gap_mm,
            "web_tip_bond_class": self.web_tip_bond_class,
            "full_depth_web_count": self.full_depth_web_count,
            "worst_depth_truncation_ratio": self.worst_depth_truncation_ratio,
            "worst_depth_truncation_layer_index": self.worst_depth_truncation_layer_index,
            "worst_depth_truncation_print_z": self.worst_depth_truncation_print_z,
            "truncated_merge_count": self.truncated_merge_count,
            "merge_depth_truncation_threshold_ratio": MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO,
        }

    def to_dict(self) -> Dict[str, Any]:
        """Plain, JSON-ready dict summarizing the whole run."""
        return {
            "layers_seen": len(self._records),
            "mutated_count": self.mutated_count,
            "failed_count": self.failed_count,
            "shutdown_count": self.shutdown_count,
            "degraded_layer_count": self.degraded_layer_count,
            "degradation_counts_by_kind": self.degradation_counts_by_kind,
            "pair_ablation_rescue_count": self.pair_ablation_rescue_count,
            "pair_ablation_rescue_layers": self.pair_ablation_rescue_layers,
            "pinched_section_count": self.pinched_section_count,
            "pinched_section_layers": self.pinched_section_layers,
            "first_degradation_z": (
                self.first_degradation.print_z
                if self.first_degradation is not None else None
            ),
            "node_merge_count": self.node_merge_count,
            "order_swap_count": self.order_swap_count,
            "order_swap_events": self.order_swap_events,
            "chord_axis_unavailable": self.chord_axis_unavailable,
            "chord_axis_unavailable_layer_count": self.chord_axis_unavailable_layer_count,
            "chord_axis_unavailable_first_z": self.chord_axis_unavailable_first_z,
            "opposing_entry_omission_count": self.opposing_entry_omission_count,
            "opposing_entry_family_ratios": {
                key: {
                    "opposing_entry_omitted": v.opposing_entry_omitted,
                    "total_candidates": v.total_candidates,
                    "ratio": v.ratio,
                }
                for key, v in self.opposing_entry_family_ratios().items()
            },
            "role_yield_omission_count": self.role_yield_omission_count,
            "role_yield_family_ratios": {
                key: {
                    "role_yield_omitted": v.role_yield_omitted,
                    "total_candidates": v.total_candidates,
                    "ratio": v.ratio,
                }
                for key, v in self.role_yield_family_ratios().items()
            },
            "role_yield_omission_warning_ratio": ROLE_YIELD_OMISSION_WARNING_RATIO,
            "role_totals": {
                role: {
                    "slots": v.slots,
                    "merged_slots": v.merged_slots,
                    "omitted": v.omitted,
                    "opposing_entry_omitted": v.opposing_entry_omitted,
                    "role_yield_omitted": v.role_yield_omitted,
                }
                for role, v in self.role_totals().items()
            },
            "first_failure": (
                self._first_failure.to_dict() if self._first_failure else None
            ),
            "first_shutdown_z": (
                self._first_shutdown.print_z if self._first_shutdown else None
            ),
            "first_node_merge_z": self.first_node_merge_z,
            "first_order_swap_z": self.first_order_swap_z,
            "structure_summary": self.structure_summary(),
            "layers": [r.to_dict() for r in self._records],
        }
