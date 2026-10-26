"""``TopologyValidator``: enforces the six invariants.

The invariants, on a layer's cut result:

1. exactly one material island;
2. zero interior holes;
3. every slot reaches an exterior boundary (the cut opens the skin);
4. every slot leaves at least the skin margin at its far end;
5. every output polygon is valid;
6. every requested web is actually present (material was removed).

Polygon topology is not toolpath topology: this validator proves the
*pre-perimeter* geometry is sound. It cannot and does not prove the downstream
perimeter/Spiral-Vase behaviour is correct.

Fail-closed: the first violated invariant raises ``TopologyError`` with
layer index, Z height, and cause, formatted as:

    Aircraft structure invalid at layer 183 (Z=36.60 mm):
    slot disconnects material island after 0.42 mm safety offset.

MULTIPLE WEBS. Invariants 1, 2 and 5 (island count, holes, validity) and the
connectivity erosion check are whole-layer checks, unaffected by how many slots
were requested. Invariants 3, 4 and 6 (boundary reached, margin held, web
actually present) run PER SLOT, in a loop over ``slot_polygons`` -- a failure
names which slot (1-indexed in request order) as well as the layer and Z. A
separate collision check verifies slots do not merge or overlap: two webs
placed closer together than their combined width plus a minimum wall would
either fuse into one fat slot or sever the section between them, and
polygon-topology invariant 1 alone cannot always catch this (see ``validate()``'s
"collision" section below for why).

CROSSING FAMILIES / JUNCTIONS. Invariants 1, 2 and the erosion-connectivity
check are union-aware (they run on ``transformed_polygons``/``transformed``,
the whole layer's own cut result, never a per-slot re-derivation). Their
failure messages attempt to attribute a cause when the layer carries more than
one slot (``_ablation_attribution``), by actually removing each slot in turn
rather than guessing from X position (a positional "nearest candidate
junction" heuristic can name a confidently wrong, innocent pair). The collision
check treats a close/overlapping pair from DIFFERENT families as an intended
junction and allows it to merge rather than failing closed; see ``validate()``'s
``family_indices`` parameter for the rule. The merge condition is "the two
slots' MEASURED footprints actually overlap in X" (edge-to-edge gap < 0),
regardless of depth. Comparing requested depths would only hold on a prismatic
section: on an airfoil, full-depth webs resolve to the LOCAL surface at each
chord station, so crossing webs inherently differ in depth. A cross-family pair
that merely approaches (gap >= 0 but below ``2 * safety_offset_mm``) fails
closed even when depths match -- merging on proximity alone would bless a
sub-bead wall that the whole-layer erosion check cannot catch.

ROTATION-AWARE MARGIN. Invariant 4's margin comparison uses
``frame.corrected_skin_margin_mm(spec.skin_margin_mm, rotation_deg)``, which
inflates the required Y-space margin by ``1/cos(theta)`` on a rotated section
(the same correction ``frame.py``/``generator.py`` apply when placing). The
rotation is derived HERE, from the ``original`` polygon the call is handed
(``frame.estimate_rotation_deg``) -- never accepted as a parameter from the
capability or the generator, or this stops being an independent check: it
exists to catch the generator being wrong, not to ride on the generator's own
caution. Reusing frame.py's functions, rather than a second implementation of
the correction, keeps both sides on the same arithmetic. Within
``frame._ROTATION_DEADZONE_DEG`` (1.0 degree) the correction is a no-op.

FAMILY LABELS. ``slot_polygons`` is a flat list, so a shallow skin web and a
deep rib from two DIFFERENT families collide exactly as two webs from the same
family would (the collision check has no notion of "family" beyond the optional
``family_indices``). The optional ``slot_labels`` parameter lets the caller
(capability) pass one label per slot so a per-slot failure message says which
family a web came from, without this module importing ``WebFamily`` or knowing
families exist. Omitting it leaves the messages unlabelled.
"""

from typing import FrozenSet, Optional, Sequence, Union

from .frame import (
    DEFAULT_PROBE_WIDTH_MM,
    EPSILON_MM,
    corrected_skin_margin_mm,
    estimate_rotation_deg,
    local_lower_worst_case,
    local_upper_worst_case,
    opposing_pair_meets,
)
from .ops import GeometryOps
from .spec import StructureSpec

# EPSILON_MM (tolerance for floating-point boundary/margin comparisons, in
# millimetres) is defined in ``aircraft_core.frame`` and imported above, so
# this module and ``frame.opposing_pair_meets`` (shared with ``generator.py``'s
# opposing-station omission check) cannot carry two copies of the same
# tolerance that drift apart. It is re-exported under this name for callers
# and tests importing ``validate.EPSILON_MM``.

# One ``family_indices`` entry, as accepted by ``TopologyValidator.validate``.
# A plain ``int`` means exactly one producing family per slot; an iterable of
# ``int`` is the merged-intent convention (every family the merge component
# absorbed) -- see ``_family_membership`` below for why the two forms are
# interchangeable at the call site.
FamilyIndicesEntry = Union[int, Sequence[int]]


def _family_membership(entry: FamilyIndicesEntry) -> FrozenSet[int]:
    """Normalizes one ``family_indices`` entry to its full family
    MEMBERSHIP set.

    The pairwise collision check below uses a MEMBERSHIP-set INTERSECTION
    test rather than a per-slot family_index EQUALITY comparison (see also
    ``aircraft_core/generator.py``'s ``WebIntent.family_membership``). A
    standalone (unmerged) web belongs to exactly one family, so a plain
    ``int`` is accepted and wrapped into a singleton set here -- which makes
    the intersection rule reduce EXACTLY to an equality check for two ordinary
    slots: ``frozenset({a}).isdisjoint(frozenset({b}))`` is ``False`` iff
    ``a == b`` (see ``tests/unit/test_node_merge.py``'s
    ``FamilyIndicesBackwardCompatibilityTests`` for the assertion that this
    reduction is exact).

    A MERGED intent, by contrast, belongs to EVERY family its component
    absorbed -- its own ``family_index`` is only a
    REPORTING-level representative (a FIXED rule, always the lowest
    family_index among the component's members -- see
    ``_merge_cross_family_nodes``'s own docstring), not necessarily the family
    of whichever constituent actually borders a given edge. Using that single
    representative for a SAFETY decision here would let a labelling choice that
    was never meant to describe edge-adjacency classify a genuine same-family
    collision as cross-family and allow it through the overlap branch -- a
    silent weakening of the invariant this validator exists to enforce. Passing
    the merge's FULL ``merged_family_indices`` tuple here instead (an iterable,
    not a plain ``int``) closes that gap: any entry that is not itself an
    ``int`` is treated as an iterable of every family the slot actually belongs
    to.
    """
    if isinstance(entry, int):
        return frozenset((entry,))
    return frozenset(entry)


# Bound on how many requested slots a whole-layer failure's ablation
# attribution (below) will re-diff/re-erode against. The failure path is
# already aborting the slice, so re-cutting the layer once per candidate slot
# is acceptable cost up to this count -- chosen to clear a measured 47-slot
# layer. Above this bound, ablation is skipped and the message says so
# explicitly rather than silently doing nothing.
_ABLATION_MAX_SLOTS = 64


def _ablation_attribution(ops, original_polygons, slot_polygons, slot_labels, check_ok):
    """Names the requested slot(s) whose INDIVIDUAL removal would fix a
    whole-layer failure (island count / holes / erosion connectivity), by
    actually re-cutting the layer with each candidate held out in turn --
    an empirical measurement, not a positional guess.

    Attribution is empirical rather than positional. Sorting REQUESTED slots
    by X and naming the closest CONSECUTIVE pair has no notion of entry side,
    depth, or whether that pair could plausibly interact at all; on a real
    in-host failure such a heuristic named an innocent +Y/-Y pair that the
    shared ``frame.opposing_pair_meets`` predicate had already proven safe,
    because a skin-lattice config anchors its upper/lower families at identical
    X by construction — those two ALWAYS sort adjacent with a tiny X gap,
    regardless of whether either is anywhere near the real cause. A confidently
    wrong name is worse than an honest "cause not localised", so this function
    either names a slot PROVEN (by actually removing it and re-checking) to be
    the/a cause, or says plainly that it does not know.

    ``check_ok(transformed_polygons)`` is the SAME invariant test the
    caller's own whole-layer check applies (island count == 1, or holes ==
    0, or erosion survives as one piece) — passed in by each call site
    rather than reimplemented here, so this function can never drift from
    the invariant it is attributing a cause for.

    Diagnostics-only: this NEVER raises and NEVER changes which
    ``TopologyError`` fires or its layer/Z/cause framing — only the
    attribution text appended to it. Any
    backend that cannot re-diff a particular remaining slot set (e.g.
    ``tests/unit/stub_ops.StubOps``, deliberately narrow — see its own
    module docstring) degrades that ONE candidate to "inconclusive" rather
    than raising past this function or reporting a false negative.

    Bounded to ``_ABLATION_MAX_SLOTS`` requested slots — see that
    constant's own comment for the cost/coverage bound chosen. Also
    declines (with an honest reason) when ``original_polygons`` is not
    exactly one region, since a re-diff needs exactly one subject polygon
    to cut against.
    """
    n = len(slot_polygons)
    if n == 0:
        return " Layer carried 0 requested slots; cause not localised to any slot."
    if len(original_polygons) != 1:
        return (
            " Cause not localised to a single slot (layer carried {} "
            "requested slot(s); ablation needs exactly one original region "
            "to re-cut against, got {}).".format(n, len(original_polygons))
        )
    if n > _ABLATION_MAX_SLOTS:
        return (
            " Cause not localised to a single slot (layer carried {} "
            "requested slot(s), above the {}-slot ablation bound; "
            "re-cutting the layer once per candidate was skipped as too "
            "costly).".format(n, _ABLATION_MAX_SLOTS)
        )

    original = original_polygons[0]

    def _label(idx):
        if slot_labels is None:
            return "slot {}".format(idx + 1)
        return "slot {} ({})".format(idx + 1, slot_labels[idx])

    culprits = []
    for i in range(n):
        remaining = slot_polygons[:i] + slot_polygons[i + 1 :]
        try:
            transformed = ops.diff(original, remaining)
            ok = check_ok(transformed)
        except Exception:
            # Ablation is a diagnostic convenience, never a proof: a
            # backend that cannot re-diff this particular remaining set
            # must not turn into a false "not the cause" claim, and must
            # never propagate past this function (see this function's own
            # docstring).
            continue
        if ok:
            culprits.append(i)

    if culprits:
        names = ", ".join(_label(i) for i in culprits)
        return (
            " Ablation localises the cause to {}: removing {} alone (every "
            "other requested slot unchanged) restores this layer to a "
            "valid single connected island.".format(
                names, "it" if len(culprits) == 1 else "them"
            )
        )
    return (
        " Cause not localised to any single slot among the {} requested "
        "(removing each one individually, in turn, did not restore "
        "validity — this is not a single-slot mistake this attribution can "
        "name).".format(n)
    )


class MeasurementUnavailableError(Exception):
    """Marker: the check could not be EVALUATED, as distinct from a feature
    that could not be PLACED.

    A failed measurement is a case where the plugin cannot trust its own
    inputs, so dropping structure would not be a safe fallback -- it would be
    acting on geometry that may not describe the part. A measurement that threw
    is exactly that: it is unknown whether the slot is placeable, only that
    finding out failed.

    Dropping the slot would also HIDE the measurement failure, which is a bug
    signal worth keeping visible. ``infeasible_slots`` re-raises anything
    carrying this marker instead of collecting it, so it stays fatal on both
    paths.
    """


# The exact cause prefix of the whole-layer erosion-connectivity failure.
# Shared with pair_ablation so the retry keys on the SAME string the validator
# emits, never a second copy that could drift.
ISLAND_SPLIT_CAUSE_MARKER = "slot disconnects material island after"


class TopologyError(Exception):
    """Raised when a layer's transformed geometry violates an invariant.

    Attributes:
        layer_index: Index of the offending layer.
        print_z: Z height of the offending layer, in millimetres.
    """

    def __init__(self, layer_index: int, print_z: float, cause: str):
        self.layer_index = layer_index
        self.print_z = print_z
        self.cause = cause
        message = "Aircraft structure invalid at layer {} (Z={:.2f} mm): {}".format(
            layer_index, print_z, cause
        )
        super().__init__(message)


class _MeasurementFailed(TopologyError, MeasurementUnavailableError):
    """A ``TopologyError`` that is ALSO a measurement failure, so an
    ``except TopologyError`` still catches it while ``infeasible_slots`` can
    single it out and re-raise."""


class TopologyValidator:
    """Checks one layer's transformed geometry against the invariants.

    ``safety_offset_mm`` is the erosion distance used to detect a slot that
    technically leaves the material as "one polygon" but with a neck too
    thin to survive perimeter offsetting in practice.

    The concrete reason to erode by the resolved outer wall width: under
    spiral mode Orca's own perimeter generator calls
    ``keep_largest_contour_only()`` (PerimeterGenerator.cpp:1613-1623) and
    SILENTLY DISCARDS part of a layer whose external-perimeter erosion
    (``ext_perimeter_width/2``, about half this value) splits the shape.
    Eroding by the full width is therefore deliberately about 2x stricter
    than Orca's silent-drop threshold, so any layer that passes validation
    survives that path with margin.

    A consequence worth stating: this implies a precondition
    ``skin_margin_mm > 2 * outer_wall_line_width``. Below that, the material
    left at the slot's far end cannot survive the erosion and every layer
    fails — the fix is a LARGER margin, not a smaller one.
    """

    def __init__(self, ops: GeometryOps, safety_offset_mm: float):
        if safety_offset_mm <= 0:
            raise ValueError(
                "safety_offset_mm must be > 0, got {!r}.".format(safety_offset_mm)
            )
        self._ops = ops
        self._safety_offset_mm = safety_offset_mm
        # The minimum surviving wall between two adjacent slots' measured
        # footprints, in millimetres. Twice the safety offset, the same "2x
        # stricter than Orca's own silent-drop threshold" margin the
        # connectivity erosion check already relies on (see this class's own
        # docstring) — a wall narrower than this cannot be trusted to survive
        # the SAME erosion the connectivity check applies to the whole layer,
        # so requiring it up front keeps the two checks consistent rather than
        # contradicting each other on borderline geometry.
        self._min_wall_mm = 2.0 * safety_offset_mm

    def validate(
        self,
        original_polygons: Sequence,
        transformed_polygons: Sequence,
        slot_polygons: Sequence,
        spec: StructureSpec,
        layer_index: int,
        print_z: float,
        slot_labels: Optional[Sequence[str]] = None,
        family_indices: Optional[Sequence[FamilyIndicesEntry]] = None,
    ) -> None:
        """Raise ``TopologyError`` on the first violated invariant, else return.

        ``slot_labels``, if given, must be the same length as
        ``slot_polygons`` and is used ONLY to annotate failure messages
        (e.g. which family a colliding/failing slot came from) — it has no
        effect on which invariants are checked or how. ``None`` (the
        default) leaves the messages unlabelled.

        ``family_indices`` (see ``_family_membership`` above), if given, must
        be the same length as ``slot_polygons``. Each entry is that slot's
        full family MEMBERSHIP — either a plain ``int`` (exactly one producing
        family, same ``WebIntent.family_index`` order as ``slot_labels``) or an iterable
        of ``int`` (merged-intent convention: every family a merge
        component absorbed, ``WebIntent.merged_family_indices`` — see
        ``WebIntent.family_membership`` for the accessor callers should use
        to build this list). This is the ONLY thing that changes which
        invariant fires: the collision check below (two slots closer
        together than ``2 * safety_offset_mm``) treats a close/overlapping
        pair as an INTENDED junction — allowed to merge rather than failing
        closed — exactly when (a) the two slots' family MEMBERSHIP SETS are
        DISJOINT (share no family at all — see below for why this is
        membership, not a single label) AND (b) their MEASURED footprints
        actually overlap in X (edge-to-edge gap < 0),
        regardless of whether their requested depths (Y-ranges) match. See
        the module docstring's "CROSSING FAMILIES / JUNCTIONS" note for why a
        same-family close pair, or a cross-family pair that only approaches
        without overlapping, fails closed. ``None`` (the default) treats
        every close pair as same-family, so none is allowed to merge.

        WHY MEMBERSHIP-SET INTERSECTION, NOT LABEL EQUALITY: after component
        merge, any cross-family pair genuinely inside the sub-bead band has
        ALREADY been merged into one component upstream
        (``StructureGenerator._merge_cross_family_nodes``) — so a pair that is
        STILL this close by the time it reaches the validator is either a
        same-family placement mistake or a true overlap, never a legitimate
        un-merged cross-family approach. A merged slot's own ``family_index``
        is only a reporting-level representative of its component (a fixed
        rule, always the lowest family_index among the component's members —
        see ``aircraft_core/generator.py``'s ``_merge_cross_family_nodes``
        docstring), not necessarily which constituent happens to border which
        edge, so comparing that single label against a neighbour's label could
        classify a genuine same-family collision as cross-family purely because
        the representative label did not happen to match — silently weakening
        this validator's own invariant. Comparing full MEMBERSHIP SETS instead
        is well-defined for merged intents and strictly conservative: two slots
        are "same family" (must fail closed, no branch) IFF they share ANY
        family at all, which is exactly the hazard case — sharing a family with
        a neighbour that close is the mistake, regardless of which family a
        merge happens to report as its label. For two ordinary (unmerged) slots
        this reduces EXACTLY to a label-equality check — see
        ``_family_membership``'s own docstring.

        ``original_polygons`` and ``transformed_polygons`` are single-element
        sequences (one input region, one output region) but the signature
        accepts a sequence to stay honest about what ``diff`` can return in
        general.

        ``slot_polygons`` is the requested cut: the exact slot polygon(s)
        the capability built and actually passed to ``ops.diff(original,
        slots)`` to produce ``transformed_polygons``. The removed region is
        measured DIRECTLY as
        ``union(ops.intersection(original, slot) for slot in slot_polygons)``
        rather than inferred from the output alone via
        ``ops.diff(original, [transformed])``.

        This is a measurement, not a claim: it does not trust the
        capability's assertion that ``transformed_polygons`` was produced by
        cutting ``slot_polygons`` out of ``original_polygons`` — it
        independently re-derives what that cut removes, with the same
        boolean backend the capability itself used, and invariants 3/4/6
        below measure against that re-derivation, not against the
        capability's say-so.

        WHY NOT ``ops.diff(original, [transformed])`` (a double-difference
        round trip, since ``transformed`` is itself ``original`` minus the
        slot): it looks MORE independent, because it never re-touches
        ``slot_polygons`` at all — but it is not reliable, and someone will
        be tempted to restore it for exactly that reason. An in-host NACA
        2412 slice found the two disagreeing outright: the capability built
        a 0.21 mm wide vertical slot at web station x=0.0 (slot bbox
        x=[-0.1050, 0.1050], y=[-2.1012, 3.2782]), and Clipper's own
        INTERSECTION of the section with that slot was exact —
        x=[-0.1050, 0.1050], y=[-2.1012, 3.2435] (top clipped to the local
        upper surface), 0.21 mm wide, matching the request. But
        ``ops.diff(original, [transformed])`` reconstructed "removed" as
        x=[-0.1050, 0.4204]: 0.5254 mm wide, 0.3154 mm too far right —
        same height as the true intersection, wrong width and wrong
        station. The double-difference is not a measurement of the
        requested cut; it is whatever Clipper's difference operator
        happens to return when asked to re-derive S from A and (A - S)
        with S itself discarded, and on real (non-degenerate) geometry
        that reconstruction is not guaranteed to reproduce S. Do not
        restore this form to make the validator "more independent" — it
        measures the wrong region.

        If ``slot_polygons`` is empty, or its intersection with the
        original material is empty, that is the existing "requested web is
        not present in the output" failure (invariant 6, below), not a
        separate code path.
        """
        ops = self._ops

        # A degenerate original bounding box (zero area) means the section
        # itself is degenerate — no local measurement is meaningful against
        # it, so this is still checked up front rather than left to surface
        # as an opaque local_span failure further down.
        for polygon in original_polygons:
            min_x, min_y, max_x, max_y = ops.bounds(polygon)
            bbox_area = (max_x - min_x) * (max_y - min_y)
            if bbox_area <= 0:
                raise TopologyError(
                    layer_index, print_z, "section has a degenerate bounding box."
                )

        # Validate slot_labels/family_indices up front — before invariants 1/2
        # below, which can fire before the per-slot loop further down ever
        # runs. This does not change WHICH invariant fires or its ordering; it
        # lets a whole-layer failure message attempt ablation attribution (see
        # ``_ablation_attribution``'s docstring).
        if slot_labels is not None and len(slot_labels) != len(slot_polygons):
            raise ValueError(
                "slot_labels has {} entries but slot_polygons has {}; they "
                "must be the same length (one label per requested "
                "slot).".format(len(slot_labels), len(slot_polygons))
            )
        if family_indices is not None and len(family_indices) != len(slot_polygons):
            raise ValueError(
                "family_indices has {} entries but slot_polygons has {}; "
                "they must be the same length (one family index per "
                "requested slot).".format(len(family_indices), len(slot_polygons))
            )

        # Invariant 5: all output polygons valid. Checked first — every other
        # check trusts the geometry enough to call bounds()/area() on it.
        for polygon in transformed_polygons:
            if not ops.is_valid(polygon):
                raise TopologyError(
                    layer_index,
                    print_z,
                    "transformed polygon failed validity check (is_valid() == False).",
                )

        # Invariant 1: exactly one material island. Union-aware by
        # construction: ``transformed_polygons`` is the WHOLE layer's own
        # cut result (every slot already subtracted), never re-derived
        # per-slot, so this already checks the UNION of every family's webs,
        # not one slot at a time; the attribution is appended below.
        if len(transformed_polygons) != 1:
            raise TopologyError(
                layer_index,
                print_z,
                "produced {} material islands, expected 1.".format(
                    len(transformed_polygons)
                )
                + _ablation_attribution(
                    ops,
                    original_polygons,
                    slot_polygons,
                    slot_labels,
                    check_ok=lambda t: len(t) == 1,
                ),
            )
        transformed = transformed_polygons[0]

        # Invariant 2: zero interior holes. Union-aware for the same reason
        # invariant 1 is: ``ops.contour_count(transformed)`` counts holes in
        # the WHOLE cut layer.
        holes = ops.contour_count(transformed) - 1
        if holes != 0:
            raise TopologyError(
                layer_index,
                print_z,
                "material has {} interior hole(s), expected 0.".format(holes)
                + _ablation_attribution(
                    ops,
                    original_polygons,
                    slot_polygons,
                    slot_labels,
                    check_ok=lambda t: len(t) == 1 and ops.contour_count(t[0]) - 1 == 0,
                ),
            )

        if len(original_polygons) != 1:
            raise TopologyError(
                layer_index,
                print_z,
                "expected exactly 1 original polygon to validate against, got {}.".format(
                    len(original_polygons)
                ),
            )
        original = original_polygons[0]

        # This validator's own margin re-check must not be optimistic on a
        # rotated section (see the module docstring's "ROTATION-AWARE MARGIN"
        # note). Rotation is derived HERE, independently,
        # from the ORIGINAL polygon this call was actually handed -- never
        # accepted as a parameter from the capability or the generator, which
        # is what keeps this an independent check rather than one that merely
        # trusts the generator's own rotation estimate. Reuses frame.py's
        # estimate_rotation_deg/corrected_skin_margin_mm (never a second
        # implementation of either -- see the module docstring) so this uses
        # the same deadzone/correction arithmetic the generator itself applies. Computed once per layer, like generator.py's own
        # once-per-layer call, so every slot on this layer is checked against
        # the same effective margin.
        rotation_deg = estimate_rotation_deg(ops, original)
        effective_skin_margin_mm = corrected_skin_margin_mm(
            spec.skin_margin_mm, rotation_deg
        )

        # Invariant 6 (whole-layer form): SOME material was removed at all.
        # Checked before the per-slot loop below, since a per-slot
        # intersection measurement is meaningless if nothing was removed —
        # and this is the only check that still fires correctly when
        # ``slot_polygons`` is empty (no slots were requested at all), which
        # the per-slot loop below, having nothing to iterate, cannot detect
        # on its own.
        original_area = ops.area(original)
        transformed_area = ops.area(transformed)
        # Area comparison, so the tolerance is an area: EPSILON_MM is a length.
        if transformed_area >= original_area - EPSILON_MM * EPSILON_MM:
            raise TopologyError(
                layer_index,
                print_z,
                "the requested structure is not present in the output "
                "(no material removed).",
            )

        # Per-slot measurement. Each requested cut is measured directly
        # against the original material — see the method docstring above for
        # why not ``ops.diff(original, [transformed])``. Each slot is measured,
        # and invariants 3/4/6 applied, independently, in request order
        # (1-indexed): a single bounding box spanning two separate webs'
        # removed regions would silently merge their measurements.
        # (slot_labels/family_indices length is validated above, before
        # invariants 1/2, so it is not re-checked here.)

        def _slot_label(slot_index: int) -> str:
            """"slot N", or "slot N (<label>)" if the caller supplied
            per-slot labels (e.g. family names) — purely cosmetic, does not
            change which invariant fires or when."""
            base = "slot {}".format(slot_index)
            if slot_labels is None:
                return base
            return "{} ({})".format(base, slot_labels[slot_index - 1])

        per_slot_bounds = self._check_each_slot(
            ops,
            original,
            transformed_polygons,
            slot_polygons,
            spec,
            layer_index,
            print_z,
            effective_skin_margin_mm,
            rotation_deg,
            _slot_label,
        )

        # Collision check: two webs placed too close together produce either
        # one fat merged slot or a severed section between them — neither is
        # the requested pattern. Polygon-topology invariant 1 (above) cannot
        # always catch this: both slots hang from the SAME top boundary, so
        # even a badly undersized wall between them leaves the material one
        # connected island at full resolution (the severing only shows up once
        # the perimeter's own wall width erodes it away — precisely the class
        # of neck the erosion check below exists to catch, but that check runs
        # on the WHOLE transformed layer and reports only "more than one
        # island", not which pair of slots caused it). So the pairwise gap
        # between adjacent slots' MEASURED footprints (not their requested
        # footprints — the same "measure what was actually cut" discipline as
        # invariants 3/4 above) is checked directly here, against
        # ``self._min_wall_mm`` (2x the safety offset — see ``__init__``'s
        # docstring for why that specific multiple).
        #
        # JUNCTION RULE (overlap-triggered general-polygon merge): a pair this
        # close is allowed to MERGE, instead of failing closed, exactly when
        # BOTH (a) the two slots' family MEMBERSHIP SETS are DISJOINT — a
        # set-intersection test rather than a single-label equality test; see
        # ``_family_membership``'s and ``validate()``'s docstrings ("WHY
        # MEMBERSHIP-SET INTERSECTION") for why a merged intent's single
        # representative label is not safe to compare directly — AND (b) the
        # two slots' MEASURED footprints actually OVERLAP in X (``gap < 0``).
        # Requested depths (Y-ranges) are not compared: on a non-prismatic
        # section (any real wing) full-depth webs resolve to the LOCAL surface
        # at each chord station, so two crossing webs inherently have differing
        # depths and an equal-depth premise would block every real crossing.
        # The merge branch does not compute or validate the union itself: it
        # defers to invariants 1/2 and the erosion check below, which run on
        # ``transformed``, the actual diffed polygon, regardless of shape. A
        # cross-family pair this close that does NOT overlap (0 <= gap <
        # min_wall) is deliberately NOT merged — merging on mere proximity
        # would bless a wall thinner than one printable extrusion bead
        # (sub-bead hazard: a pillar between two top-entry slots stays
        # attached to the material below, so the erosion check below never
        # disconnects it and never catches it) — so it fails closed, with a
        # message that says why, distinct from the plain collision message.
        # SAME-EDGE ONLY. The adjacent-sorted-by-X scan below is only valid
        # within one entry side: "compare adjacent neighbours" is a valid
        # shortcut for same-edge slots (every cut shares one edge, so only
        # immediate neighbours can be the closest pair), but it does not hold
        # once two edges are in play: a wide merged +Y node can overlap a -Y
        # notch that is not its nearest neighbour in a naive X-sort across ALL
        # slots. So this loop runs ONCE PER SIDE, on slots that share an entry
        # side only; the cross-edge (opposing) case is a different geometric
        # question, handled by the ALL-PAIRS check below instead.
        upper_bounds = [b for b in per_slot_bounds if b[5] == "+Y"]
        lower_bounds = [b for b in per_slot_bounds if b[5] == "-Y"]
        for per_side_bounds in (upper_bounds, lower_bounds):
            if len(per_side_bounds) <= 1:
                continue
            by_x = sorted(per_side_bounds, key=lambda b: b[1])
            for (idx_a, _a_min_x, _a_min_y, a_max_x, _a_max_y, _a_side), (
                idx_b,
                b_min_x,
                _b_min_y,
                _b_max_x,
                _b_max_y,
                _b_side,
            ) in zip(by_x, by_x[1:]):
                gap = b_min_x - a_max_x
                if gap >= self._min_wall_mm - EPSILON_MM:
                    continue

                same_family = family_indices is None or not _family_membership(
                    family_indices[idx_a - 1]
                ).isdisjoint(_family_membership(family_indices[idx_b - 1]))
                if not same_family:
                    overlaps = gap < 0
                    if overlaps:
                        # Intended junction: footprints actually overlap, so
                        # there is no plane wall between them to validate —
                        # invariant 1/2/erosion below (already union-aware) are
                        # the actual arbiters of the resulting (possibly
                        # stepped) union shape — no separate check is needed
                        # here.
                        continue

                label_a = (
                    idx_a if slot_labels is None else "{} ({})".format(idx_a, slot_labels[idx_a - 1])
                )
                label_b = (
                    idx_b if slot_labels is None else "{} ({})".format(idx_b, slot_labels[idx_b - 1])
                )
                if not same_family:
                    raise TopologyError(
                        layer_index,
                        print_z,
                        "slots {} and {} leave only {:.4f} mm of wall between "
                        "them, below the required {:.4f} mm minimum (2x the "
                        "{:.2f} mm safety offset) — this looks like an "
                        "intended web crossing (different families), but "
                        "their footprints do not actually overlap, and under "
                        "the overlap rule, matching depths is no longer enough to merge "
                        "on its own. A {:.4f} mm wall below one printable "
                        "extrusion bead cannot print. Cross-family slots "
                        "must either overlap outright (route the crossing "
                        "through the same X span, regardless of depth) or "
                        "clear the {:.4f} mm minimum wall.".format(
                            label_a,
                            label_b,
                            gap,
                            self._min_wall_mm,
                            self._safety_offset_mm,
                            gap,
                            self._min_wall_mm,
                        ),
                    )
                raise TopologyError(
                    layer_index,
                    print_z,
                    "slots {} and {} leave only {:.4f} mm of wall between "
                    "them, below the required {:.4f} mm minimum (2x the "
                    "{:.2f} mm safety offset) — they would merge into one "
                    "slot or sever the material between them.".format(
                        label_a, label_b, gap, self._min_wall_mm, self._safety_offset_mm
                    ),
                )

        # Opposing-entry check (the extended no-sever proof for opposing +Y/-Y
        # entry): a DIFFERENT geometric question from the same-edge check
        # above — whether a +Y cut and a -Y cut can reach far enough toward
        # each other to leave no wall between them. Runs AFTER the same-edge
        # check (and after any same-edge merge the generator already resolved
        # before handing slots to this validator — merge happens upstream in
        # generator.py, so ``slot_polygons`` here is already post-merge).
        # ALL-PAIRS: unlike the same-edge scan above, adjacency-in-X is not a
        # valid shortcut once two edges are in play (a wide merged +Y node can
        # overlap a non-adjacent -Y notch).
        for idx_u, u_min_x, u_min_y, u_max_x, u_max_y, _u_side in upper_bounds:
            for idx_l, l_min_x, l_min_y, l_max_x, l_max_y, _l_side in lower_bounds:
                # Depth of each notch's own cut, from its own entry surface —
                # the same "MERGED region's own actual span and far end"
                # convention proof point 6 uses for a merged node (which is
                # exactly what ``upper_bounds``/``lower_bounds`` already
                # contain, post-merge).
                d_upper = u_max_y - u_min_y
                d_lower = l_max_y - l_min_y

                # The whole proof (points 1, 3, 4) is ONE shared predicate,
                # ``frame.opposing_pair_meets`` — also called by
                # ``generator.py``'s opposing-station omission check, so the
                # generator's own placement decision and this independent
                # re-check can never apply a different formula to the same
                # geometric question. See that function's own docstring for the
                # full proof-point-by-proof-point mapping.
                try:
                    meeting = opposing_pair_meets(
                        ops,
                        original,
                        u_min_x,
                        u_max_x,
                        d_upper,
                        l_min_x,
                        l_max_x,
                        d_lower,
                        self._safety_offset_mm,
                        self._min_wall_mm,
                        DEFAULT_PROBE_WIDTH_MM,
                    )
                except ValueError as exc:
                    # Message-only halo bounds (not the meeting-condition
                    # arithmetic itself, which stays entirely inside the
                    # shared predicate) so this failure can still name the
                    # overlap span the measurement was attempted over.
                    halo_min_x = max(
                        u_min_x - self._safety_offset_mm,
                        l_min_x - self._safety_offset_mm,
                    )
                    halo_max_x = min(
                        u_max_x + self._safety_offset_mm,
                        l_max_x + self._safety_offset_mm,
                    )
                    raise TopologyError(
                        layer_index,
                        print_z,
                        "opposing pair {} (+Y) and {} (-Y): local geometry "
                        "could not be measured across the overlap span "
                        "x=[{:.4f}, {:.4f}] mm: {}".format(
                            _slot_label(idx_u),
                            _slot_label(idx_l),
                            halo_min_x,
                            halo_max_x,
                            exc,
                        ),
                    )
                if meeting is not None:
                    raise TopologyError(
                        layer_index,
                        print_z,
                        "opposing slots {} (+Y, depth {:.4f} mm) and {} "
                        "(-Y, depth {:.4f} mm) meet: over the overlap span "
                        "x=[{:.4f}, {:.4f}] mm the local thickness is "
                        "{:.4f} mm, but the two opposing cuts plus the "
                        "{:.4f} mm minimum wall (2x the {:.2f} mm safety "
                        "offset) require {:.4f} mm — these opposing cuts "
                        "leave no wall between them.".format(
                            _slot_label(idx_u),
                            meeting.d_upper_mm,
                            _slot_label(idx_l),
                            meeting.d_lower_mm,
                            meeting.overlap_min_x,
                            meeting.overlap_max_x,
                            meeting.t_local_conservative,
                            self._min_wall_mm,
                            self._safety_offset_mm,
                            meeting.required_mm,
                        ),
                    )

        # Connectivity: erode by the safety offset and confirm the material is
        # still one connected island. This is what actually catches a slot that
        # has cut fully through — polygon topology alone (invariant 1, above)
        # can miss a neck too thin to survive perimeter offsetting in
        # practice. Whole-layer: it operates on ``transformed`` regardless of
        # how many slots produced it.
        eroded = ops.offset(transformed, -self._safety_offset_mm)
        if len(eroded) > 1:
            raise TopologyError(
                layer_index,
                print_z,
                ISLAND_SPLIT_CAUSE_MARKER
                + " {:.2f} mm safety offset.".format(self._safety_offset_mm)
                + _ablation_attribution(
                    ops,
                    original_polygons,
                    slot_polygons,
                    slot_labels,
                    check_ok=lambda t: len(t) == 1
                    and len(ops.offset(t[0], -self._safety_offset_mm)) == 1,
                ),
            )
        if not eroded:
            # Distinct diagnosis: nothing survived the erosion at all, i.e. the
            # whole section is thinner than the safety offset. Reporting this
            # as a severed island would misdiagnose a collapsing tip.
            raise TopologyError(
                layer_index,
                print_z,
                "section is entirely thinner than the {:.2f} mm safety offset, "
                "so no material survives erosion; the section cannot carry a "
                "web here.".format(self._safety_offset_mm)
                + _ablation_attribution(
                    ops,
                    original_polygons,
                    slot_polygons,
                    slot_labels,
                    check_ok=lambda t: len(t) == 1
                    and len(ops.offset(t[0], -self._safety_offset_mm)) >= 1,
                ),
            )

    def _check_each_slot(
        self,
        ops,
        original,
        transformed_polygons,
        slot_polygons,
        spec,
        layer_index,
        print_z,
        effective_skin_margin_mm,
        rotation_deg,
        _slot_label,
        on_violation=None,
    ):
        """Runs the PER-SLOT checks (invariants 3, 4 and 6) over every entry in
        ``slot_polygons``, in request order, and returns the per-slot
        measurement list the caller's post-loop checks (same-edge collision,
        opposing-pair) consume.

        Parameters: ``original``/``transformed_polygons`` are the WHOLE
        layer's own before/after polygons (only used here for direct
        measurement and for failure-message text — see ``validate()``'s
        docstring for why a double-difference of ``transformed_polygons`` must
        never become a predicate input), ``slot_polygons`` is the requested
        cut, ``effective_skin_margin_mm``/``rotation_deg`` are this layer's
        already-corrected values, and ``_slot_label`` is the SAME closure
        ``validate()`` builds and reuses after this method returns, so a slot's
        label can never read differently before and after the per-slot loop.

        ``on_violation``, if given, is called as ``on_violation(slot_index,
        exc)`` for every slot whose own per-slot checks raise
        ``TopologyError``, and the scan CONTINUES to the next slot instead of
        propagating — this is the reporting mode ``infeasible_slots()`` uses.
        The default (``None``) is the gating mode: the first ``TopologyError``
        propagates immediately and the loop never continues past it.

        Returns ``per_slot_bounds``: a list of ``(slot_index, removed_min_x,
        removed_min_y, removed_max_x, removed_max_y, entry_side)`` tuples, one
        per requested slot, in request order — the only state the loop built
        that ``validate()``'s code after the loop (the same-edge collision scan
        and the opposing-pair scan) still depends on. In the ``on_violation``
        mode, a slot that failed contributes no entry to this list (there is
        nothing valid to report for it).
        """
        per_slot_bounds = []
        for i, slot in enumerate(slot_polygons):
            slot_index = i + 1
            if on_violation is None:
                bounds = self._check_one_slot(
                    ops,
                    original,
                    transformed_polygons,
                    slot,
                    slot_index,
                    spec,
                    layer_index,
                    print_z,
                    effective_skin_margin_mm,
                    rotation_deg,
                    _slot_label,
                )
                per_slot_bounds.append(bounds)
            else:
                try:
                    bounds = self._check_one_slot(
                        ops,
                        original,
                        transformed_polygons,
                        slot,
                        slot_index,
                        spec,
                        layer_index,
                        print_z,
                        effective_skin_margin_mm,
                        rotation_deg,
                        _slot_label,
                    )
                except MeasurementUnavailableError:
                    # The check could not be EVALUATED, so it is unknown
                    # whether this slot is placeable. Dropping it would act
                    # on geometry that failed to be measured AND would hide
                    # the measurement failure itself. Stays fatal on both
                    # paths -- re-raised even in collect mode.
                    raise
                except TopologyError as exc:
                    on_violation(slot_index, exc)
                    continue
                per_slot_bounds.append(bounds)

        return per_slot_bounds

    def _check_one_slot(
        self,
        ops,
        original,
        transformed_polygons,
        slot,
        slot_index,
        spec,
        layer_index,
        print_z,
        effective_skin_margin_mm,
        rotation_deg,
        _slot_label,
    ):
        """Runs the per-slot checks (invariants 3, 4 and 6) for ONE requested slot
        and returns its ``per_slot_bounds`` tuple.

        See ``_check_each_slot``'s docstring for what each parameter means.
        Raises ``TopologyError`` on a violation; ``_check_each_slot`` decides
        whether that propagates immediately or is collected via
        ``on_violation``.
        """
        removed = ops.intersection(original, slot)
        if not removed:
            raise TopologyError(
                layer_index,
                print_z,
                "{} was requested but is not present in the output "
                "(no material removed).".format(_slot_label(slot_index)),
            )
        # Per-piece bounds, kept for diagnostics: a slot's intersection
        # with the material can legitimately return several pieces, and
        # aggregating their bbox silently moves the measurement station
        # off the slot.
        piece_bounds = [ops.bounds(piece) for piece in removed]
        removed_min_x, removed_min_y, removed_max_x, removed_max_y = piece_bounds[0]
        for extra in removed[1:]:
            r_min_x, r_min_y, r_max_x, r_max_y = ops.bounds(extra)
            removed_min_x = min(removed_min_x, r_min_x)
            removed_min_y = min(removed_min_y, r_min_y)
            removed_max_x = max(removed_max_x, r_max_x)
            removed_max_y = max(removed_max_y, r_max_y)

        # Chord station and span for the local surface measurements
        # below. The removed region's own X extent is the honest
        # source: the validator is only ever handed the
        # original/transformed polygons and spec, not the frame that
        # placed the web, so deriving the station from what the slot
        # actually removed (rather than threading web_position through
        # as a new parameter) keeps this check honest about what it
        # knows.
        #
        # The worst case is measured across the removed region's FULL X
        # span (not a point probe at its centre): a single-station probe at
        # the centre of the removed region does not, in general, land on
        # the same chord station ``web_centerline`` used to PLACE the
        # slot's far end — on real Clipper-backed geometry the removed
        # region's bounding box is not exactly symmetric about the web
        # centreline the way a hand-built rectangular notch fixture is.
        # Measuring the worst case over the same span the slot actually
        # occupies is what makes placement and validation agree by
        # construction rather than by coincidence (see frame.py's
        # module docstring, "WORST CASE ACROSS A SPAN").
        slot_x = (removed_min_x + removed_max_x) / 2.0
        slot_span_mm = removed_max_x - removed_min_x

        # Independently determine whether THIS slot is a candidate
        # scar -- never trusted from the generator/capability, derived
        # here from the geometry itself (the same independence
        # discipline as the rotation re-derivation above). Gated on
        # spec.expected_hole_count > 0 so a default (0) layer never calls
        # ops.hole_bounds at all, and so an UNDECLARED
        # hole (expected_hole_count still 0) never becomes accidentally
        # tolerated here just because a slot's span happens to cross
        # it: that would silently weaken this validator exactly where
        # StructureSpec.expected_hole_count's docstring says a
        # declaration is the only safe licence. A hole "explains" this
        # slot when the hole's own X footprint overlaps the removed
        # region's X footprint -- the same test frame.local_span uses
        # internally to accept a gap as a declared-hole crossing.
        scar_hole = None
        if spec.expected_hole_count > 0:
            for hb in ops.hole_bounds(original):
                hb_min_x, hb_min_y, hb_max_x, hb_max_y = hb
                if (
                    hb_max_x + EPSILON_MM >= removed_min_x
                    and hb_min_x - EPSILON_MM <= removed_max_x
                ):
                    scar_hole = hb
                    break

        try:
            local_max_y = local_upper_worst_case(
                ops,
                original,
                slot_x,
                slot_span_mm,
                DEFAULT_PROBE_WIDTH_MM,
                hole_aware=scar_hole is not None,
            )
            local_min_y = local_lower_worst_case(
                ops,
                original,
                slot_x,
                slot_span_mm,
                DEFAULT_PROBE_WIDTH_MM,
                hole_aware=scar_hole is not None,
            )
        except ValueError as exc:
            # A measurement failure, not an omission -- see
            # MeasurementUnavailableError. The slot may be perfectly
            # placeable; finding out failed. The pre-diff drop path must not
            # swallow this.
            raise _MeasurementFailed(
                layer_index,
                print_z,
                "{} local geometry could not be measured across chord "
                "span x=[{:.4f}, {:.4f}] mm: {}".format(
                    _slot_label(slot_index), removed_min_x, removed_max_x, exc
                ),
            )

        # Invariant 3 (SIDE-AWARE): the slot touches an exterior boundary.
        # The entry surface is derived FROM THE REMOVED GEOMETRY ITSELF —
        # never from a parameter, the same independence discipline as the
        # rotation re-derivation, and for the same reason: two measurements of
        # one quantity can drift apart. A slot that reaches its LOCAL UPPER
        # boundary is a +Y entry; this is DELIBERATELY the FIRST branch
        # checked, and takes priority whenever it holds, regardless of whether
        # the slot ALSO happens to sit close to the lower boundary. Only when a
        # slot does NOT reach the upper boundary is the LOWER boundary
        # consulted, so a genuine -Y entry (which never reaches upper) gets
        # its own mirrored check instead of being rejected as an incomplete
        # +Y cut. A slot that reaches NEITHER boundary fails with "does not
        # touch the exterior boundary" — this is not a proximity/
        # nearest-boundary heuristic (which could misclassify a genuinely
        # broken +Y cut that happens to sit nearer the bottom than the top).
        touches_upper = removed_max_y >= local_max_y - EPSILON_MM
        touches_lower = removed_min_y <= local_min_y + EPSILON_MM
        if touches_upper:
            entry_side = "+Y"
        elif touches_lower:
            entry_side = "-Y"
        else:
            entry_side = None

        if entry_side is None:
            raise TopologyError(
                layer_index,
                print_z,
                "{} does not touch the exterior boundary "
                "(reaches y={:.4f} mm, local boundary at x={:.4f} mm is "
                "at y={:.4f} mm).".format(
                    _slot_label(slot_index), removed_max_y, slot_x, local_max_y
                ),
            )

        if entry_side == "+Y" and scar_hole is not None:
            # A third, scar-shaped invariant-4 check — the ordinary +Y
            # branch below measures remaining skin against the far EXTERIOR
            # surface, which is the wrong quantity for a scar: its far end
            # deliberately terminates INSIDE the declared hole's own void,
            # never against solid material. Two things independently re-verify
            # what ``_generate_scar`` intended to place:
            hole_min_x, hole_min_y, hole_max_x, hole_max_y = scar_hole
            if removed_min_y > hole_max_y + EPSILON_MM:
                raise TopologyError(
                    layer_index,
                    print_z,
                    "{} does not reach the declared hole's own near "
                    "(top) boundary at y={:.4f} mm — its far end sits "
                    "at y={:.4f} mm, still above the hole, so the hole "
                    "remains disconnected from this cut.".format(
                        _slot_label(slot_index), hole_max_y, removed_min_y
                    ),
                )
            if removed_min_y < hole_min_y - EPSILON_MM:
                raise TopologyError(
                    layer_index,
                    print_z,
                    "{} overshoots past the declared hole's own far "
                    "(bottom) boundary at y={:.4f} mm — its far end "
                    "sits at y={:.4f} mm, below the hole entirely. "
                    "This is not a hole connector any more; it is a "
                    "second, unintended full cut through material "
                    "beyond the hole.".format(
                        _slot_label(slot_index), hole_min_y, removed_min_y
                    ),
                )
            remaining_wall = local_max_y - hole_max_y
            if remaining_wall < effective_skin_margin_mm - EPSILON_MM:
                raise TopologyError(
                    layer_index,
                    print_z,
                    "{} (scar) leaves {:.4f} mm of wall between the "
                    "declared hole's near boundary (y={:.4f} mm) and "
                    "the worst-case local upper exterior surface "
                    "(y={:.4f} mm), below the required {:.4f} mm "
                    "margin (effective {:.4f} mm after this section's "
                    "own {:.4f} degree rotation correction).".format(
                        _slot_label(slot_index),
                        remaining_wall,
                        hole_max_y,
                        local_max_y,
                        spec.skin_margin_mm,
                        effective_skin_margin_mm,
                        rotation_deg,
                    ),
                )
        elif entry_side == "-Y" and scar_hole is not None:
            # Mirror of the +Y scar branch above.
            hole_min_x, hole_min_y, hole_max_x, hole_max_y = scar_hole
            if removed_max_y < hole_min_y - EPSILON_MM:
                raise TopologyError(
                    layer_index,
                    print_z,
                    "{} does not reach the declared hole's own near "
                    "(bottom) boundary at y={:.4f} mm — its far end "
                    "sits at y={:.4f} mm, still below the hole, so the "
                    "hole remains disconnected from this cut.".format(
                        _slot_label(slot_index), hole_min_y, removed_max_y
                    ),
                )
            if removed_max_y > hole_max_y + EPSILON_MM:
                raise TopologyError(
                    layer_index,
                    print_z,
                    "{} overshoots past the declared hole's own far "
                    "(top) boundary at y={:.4f} mm — its far end sits "
                    "at y={:.4f} mm, above the hole entirely. This is "
                    "not a hole connector any more; it is a second, "
                    "unintended full cut through material beyond the "
                    "hole.".format(
                        _slot_label(slot_index), hole_max_y, removed_max_y
                    ),
                )
            remaining_wall = hole_min_y - local_min_y
            if remaining_wall < effective_skin_margin_mm - EPSILON_MM:
                raise TopologyError(
                    layer_index,
                    print_z,
                    "{} (scar) leaves {:.4f} mm of wall between the "
                    "declared hole's near boundary (y={:.4f} mm) and "
                    "the worst-case local lower exterior surface "
                    "(y={:.4f} mm), below the required {:.4f} mm "
                    "margin (effective {:.4f} mm after this section's "
                    "own {:.4f} degree rotation correction).".format(
                        _slot_label(slot_index),
                        remaining_wall,
                        hole_min_y,
                        local_min_y,
                        spec.skin_margin_mm,
                        effective_skin_margin_mm,
                        rotation_deg,
                    ),
                )
        elif entry_side == "+Y":
            # Invariant 4: remaining skin at the slot end >= skin_margin_mm,
            # measured against the WORST-CASE (highest) LOCAL lower surface
            # across the slot's own chord span — not a single station at the
            # removed region's centre (an in-host NACA 2412 slice found the
            # generator and this validator disagreeing by more than the
            # required margin when each measured at a different single
            # station; see frame.py's module docstring, "WORST CASE ACROSS A
            # SPAN"), and not the section's global bounding-box min_y, which
            # on a cambered airfoil over-estimates the remaining skin under the
            # slot.
            remaining_skin = removed_min_y - local_min_y
            if remaining_skin < effective_skin_margin_mm - EPSILON_MM:
                raise TopologyError(
                    layer_index,
                    print_z,
                    "{} leaves {:.4f} mm of skin at its far end, below "
                    "the required {:.4f} mm margin (effective {:.4f} mm "
                    "after this section's own {:.4f} degree rotation "
                    "correction). Removed pieces: {}. "
                    "Measured over the removed span x=[{:.4f}, {:.4f}] "
                    "(centre {:.4f}, width {:.4f} mm): slot end y={:.4f}, "
                    "worst-case local lower surface y={:.4f}.".format(
                        _slot_label(slot_index),
                        remaining_skin,
                        spec.skin_margin_mm,
                        effective_skin_margin_mm,
                        rotation_deg,
                        "; ".join(
                            "x[{0:.4f},{2:.4f}] y[{1:.4f},{3:.4f}]".format(*b)
                            for b in piece_bounds
                        )
                        + " | transformed: "
                        + "; ".join(
                            "x[{0:.4f},{2:.4f}] y[{1:.4f},{3:.4f}]".format(
                                *ops.bounds(t)
                            )
                            for t in transformed_polygons
                        ),
                        removed_min_x,
                        removed_max_x,
                        slot_x,
                        removed_max_x - removed_min_x,
                        removed_min_y,
                        local_min_y,
                    ),
                )
        else:
            # Invariant 4, MIRRORED for a -Y entry: remaining skin at the
            # slot's far (upper) end >= skin_margin_mm, measured against the
            # WORST-CASE (lowest) LOCAL upper surface across the slot's own
            # chord span — the exact mirror of the +Y branch above, reusing the
            # same worst-case-over-a-span primitives (never a second
            # implementation of the measurement discipline).
            remaining_skin = local_max_y - removed_max_y
            if remaining_skin < effective_skin_margin_mm - EPSILON_MM:
                raise TopologyError(
                    layer_index,
                    print_z,
                    "{} leaves {:.4f} mm of skin at its far end, below "
                    "the required {:.4f} mm margin (effective {:.4f} mm "
                    "after this section's own {:.4f} degree rotation "
                    "correction). Removed pieces: {}. "
                    "Measured over the removed span x=[{:.4f}, {:.4f}] "
                    "(centre {:.4f}, width {:.4f} mm): slot end y={:.4f}, "
                    "worst-case local upper surface y={:.4f}. Entry "
                    "side: -Y.".format(
                        _slot_label(slot_index),
                        remaining_skin,
                        spec.skin_margin_mm,
                        effective_skin_margin_mm,
                        rotation_deg,
                        "; ".join(
                            "x[{0:.4f},{2:.4f}] y[{1:.4f},{3:.4f}]".format(*b)
                            for b in piece_bounds
                        )
                        + " | transformed: "
                        + "; ".join(
                            "x[{0:.4f},{2:.4f}] y[{1:.4f},{3:.4f}]".format(
                                *ops.bounds(t)
                            )
                            for t in transformed_polygons
                        ),
                        removed_min_x,
                        removed_max_x,
                        slot_x,
                        removed_max_x - removed_min_x,
                        removed_max_y,
                        local_max_y,
                    ),
                )

        return (
            slot_index,
            removed_min_x,
            removed_min_y,
            removed_max_x,
            removed_max_y,
            entry_side,
        )

    def infeasible_slots(
        self,
        ops,
        original,
        slot_polygons,
        spec,
        layer_index,
        print_z,
        slot_labels=None,
        family_indices=None,
        rotation_deg=None,
    ):
        """Report every slot in ``slot_polygons`` that fails its OWN per-slot
        feasibility checks (the predicates in ``_check_one_slot``: invariants 3,
        4 and 6 — the same predicates ``validate()`` runs, never duplicated).

        Returns a list of ``(slot_index, TopologyError)`` pairs, one per
        infeasible slot (1-indexed, same numbering ``validate()`` uses), in
        request order. A slot that passes contributes nothing. This is a
        REPORTING mode, not a gate: it never raises on a slot's own
        violation, it collects every one and keeps scanning (via
        ``_check_each_slot``'s ``on_violation`` callback).

        Deliberately does NOT run the pairwise same-edge / opposing-pair scans
        or the whole-layer invariants (1/2/5 and the connectivity
        erosion) that ``validate()`` runs after its per-slot loop — this method
        answers a strictly narrower question, "is this slot individually
        placeable", which is what a pre-diff, per-slot drop decision needs
        (intended use: dropping an individually infeasible slot BEFORE
        ``ops.diff``, upstream of this validator's own post-diff gate).

        ``original`` (a single polygon, not a sequence) and ``slot_polygons``
        are ordinary parameters; this method holds no caller state. The
        generator's pre-diff call and this validator's own post-diff gate must
        keep measuring genuinely different things (intended footprint vs.
        actually-removed geometry), and are never merged into one question by
        sharing state here.

        Callable WITHOUT a real ``transformed_polygons`` — that parameter is
        passed as ``()`` to ``_check_one_slot`` internally, since every
        per-slot predicate measures ``original``/``slot_polygons`` directly
        and ``transformed_polygons`` is used ONLY inside failure-message
        text (the "Removed pieces: ... | transformed: ..." trailer on the
        two skin-margin messages), which iterates it with
        ``"; ".join(... for t in transformed_polygons)`` — empty on an empty
        sequence, never an error. Do not pass ``None`` there (bare ``None`` is
        not iterable and WOULD raise inside that ``.join()`` if a skin-margin
        violation's message is ever built) — this method always supplies
        ``()``.

        ``rotation_deg``, if given, is used as-is (the caller's own
        independently-derived value); ``None`` (the default) derives it the
        same way ``validate()`` does, via ``estimate_rotation_deg(ops,
        original)`` — never accepted as a trusted pass-through by default,
        for the same reason ``validate()``'s own docstring gives.

        ``slot_labels``/``family_indices`` are accepted for signature
        symmetry with ``validate()`` and for message annotation
        (``slot_labels``) exactly as ``validate()`` uses them; note
        ``family_indices`` has no effect here — it only matters to the
        pairwise same-edge collision check, which this method does not run.
        """
        if slot_labels is not None and len(slot_labels) != len(slot_polygons):
            raise ValueError(
                "slot_labels has {} entries but slot_polygons has {}; they "
                "must be the same length (one label per requested "
                "slot).".format(len(slot_labels), len(slot_polygons))
            )
        if family_indices is not None and len(family_indices) != len(slot_polygons):
            raise ValueError(
                "family_indices has {} entries but slot_polygons has {}; "
                "they must be the same length (one family index per "
                "requested slot).".format(len(family_indices), len(slot_polygons))
            )

        if rotation_deg is None:
            rotation_deg = estimate_rotation_deg(ops, original)
        effective_skin_margin_mm = corrected_skin_margin_mm(
            spec.skin_margin_mm, rotation_deg
        )

        def _slot_label(slot_index: int) -> str:
            base = "slot {}".format(slot_index)
            if slot_labels is None:
                return base
            return "{} ({})".format(base, slot_labels[slot_index - 1])

        violations = []

        def _on_violation(slot_index, exc):
            violations.append((slot_index, exc))

        self._check_each_slot(
            ops,
            original,
            (),
            slot_polygons,
            spec,
            layer_index,
            print_z,
            effective_skin_margin_mm,
            rotation_deg,
            _slot_label,
            on_violation=_on_violation,
        )

        return violations
