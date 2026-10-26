"""Post-validation opposing-pair ablation.

An opposing pair (``WebFamily.opposing_pair=True``) can be individually
placeable and still tip a layer's whole-layer erosion check into an island
split, through geometry no pre-placement predicate measures (measured cases:
a lateral hazard at layer 947 and a far-cap hazard against a reflex vertex at
layer 454). A lateral pre-check catches the first and misses the second; the
validator's own criterion is the only check with no tuned threshold.

So the pair is judged by the validator itself, lazily: only AFTER
``TopologyValidator.validate`` fails with the island-split cause does this
module try, one pair station at a time (both members, in family-index then
station order), removing that station and re-validating the WHOLE layer with
the same ``safety_offset_mm``. The first single-station removal that yields a
fully valid layer is accepted and recorded as an ``OmittedWeb`` plus a
``PairAblationRescue``; stations are never combined and non-pair slots are
never removed. Any other failure kind is re-raised untouched and no ablation
is attempted. No Orca imports: the plugin only calls
``validate_with_pair_ablation``.
"""

import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .generator import MITRE_OMISSION_MARKER, OmittedWeb
from .validate import ISLAND_SPLIT_CAUSE_MARKER, TopologyError


@dataclass(frozen=True)
class PairStation:
    """One pair station: the indices (into the parallel intent/slot lists)
    of its member slots. A station normally has two members; it has one when
    the other was already dropped before validation."""

    family_index: int
    family_name: str
    x_mm: float
    member_indices: Tuple[int, ...]


@dataclass(frozen=True)
class PairAblationRescue:
    layer_index: int
    print_z: float
    family_index: int
    family_name: str
    station_x_mm: float
    removed_indices: Tuple[int, ...]
    kept_indices: Tuple[int, ...]
    implicated: Tuple[str, ...]
    safety_offset_mm: float
    elapsed_ms: float
    omitted: OmittedWeb

    def as_dict(self) -> dict:
        return {
            "layer_index": self.layer_index,
            "print_z": self.print_z,
            "family_index": self.family_index,
            "family_name": self.family_name,
            "station_x_mm": round(self.station_x_mm, 4),
            "removed_slot_numbers": [i + 1 for i in self.removed_indices],
            "implicated_members": list(self.implicated),
            "safety_offset_mm": self.safety_offset_mm,
            "elapsed_ms": round(self.elapsed_ms, 3),
            "reason": self.omitted.reason,
        }


def is_island_split_error(error: BaseException) -> bool:
    """True only for the whole-layer erosion-connectivity failure."""
    return (
        isinstance(error, TopologyError)
        and ISLAND_SPLIT_CAUSE_MARKER in getattr(error, "cause", "")
    )


def find_pair_stations(intents: Sequence, spec) -> List[PairStation]:
    """Pair stations present on this layer, in deterministic order (family
    index, then intent order = ascending station).

    Only intents of a family declared ``opposing_pair=True`` whose own
    ``family_name`` matches that family (feature-lattice intents carry a local
    family index that is not an index into ``effective_families()``) and that
    are not merged nodes. Members are grouped the way the generator emits them:
    an upper (+Y entry) immediately followed by a lower (-Y entry) of the same
    family is one station; a member whose partner was dropped earlier is a
    station of its own.
    """
    families = spec.effective_families()
    eligible = []
    for idx, intent in enumerate(intents):
        fam = intent.family_index
        if not (0 <= fam < len(families)):
            continue
        family = families[fam]
        if not family.opposing_pair or intent.family_name != family.name:
            continue
        if getattr(intent, "merged_family_indices", None):
            continue
        eligible.append(idx)

    stations: List[PairStation] = []
    pos = 0
    while pos < len(eligible):
        i = eligible[pos]
        a = intents[i]
        if pos + 1 < len(eligible):
            j = eligible[pos + 1]
            b = intents[j]
            if (
                j == i + 1
                and b.family_index == a.family_index
                and not a.entry_lower
                and b.entry_lower
            ):
                stations.append(
                    PairStation(
                        a.family_index,
                        a.family_name,
                        (a.start[0] + b.start[0]) / 2.0,
                        (i, j),
                    )
                )
                pos += 2
                continue
        stations.append(
            PairStation(a.family_index, a.family_name, a.start[0], (i,))
        )
        pos += 1
    stations.sort(key=lambda s: (s.family_index, s.x_mm))
    return stations


def _layer_is_one_island(ops, original, kept_slots, safety_offset_mm) -> bool:
    """The validator's own connectivity criterion (island count 1 and the
    erosion by ``safety_offset_mm`` stays one piece)."""
    transformed = ops.diff(original, kept_slots)
    return (
        len(transformed) == 1
        and len(ops.offset(transformed[0], -safety_offset_mm)) == 1
    )


def attempt_pair_ablation(
    error: TopologyError,
    ops,
    validator,
    originals,
    intents: Sequence,
    slots: Sequence,
    spec,
    layer_index: int,
    print_z: float,
    slot_labels,
    family_indices,
    safety_offset_mm: float,
) -> Optional[PairAblationRescue]:
    """Try to rescue the layer ``error`` refused. Returns the rescue, or
    raises (the original error untouched when ablation does not apply, the
    original cause plus an appended note when it was tried and failed)."""
    if not is_island_split_error(error):
        raise error
    stations = find_pair_stations(intents, spec)
    if not stations or len(originals) != 1:
        raise error

    t0 = time.perf_counter()
    original = originals[0]
    tried = 0
    for station in stations:
        removed = set(station.member_indices)
        keep = [i for i in range(len(slots)) if i not in removed]
        kept_slots = [slots[i] for i in keep]
        tried += 1
        try:
            if not _layer_is_one_island(ops, original, kept_slots, safety_offset_mm):
                continue
        except Exception:
            # A backend that cannot re-diff this set is "not rescued", never
            # a crash and never a false rescue.
            continue
        replacement = list(ops.diff(original, kept_slots))
        try:
            validator.validate(
                originals,
                replacement,
                kept_slots,
                spec,
                layer_index,
                print_z,
                slot_labels=(
                    [slot_labels[i] for i in keep] if slot_labels is not None else None
                ),
                family_indices=(
                    [family_indices[i] for i in keep]
                    if family_indices is not None else None
                ),
            )
        except TopologyError:
            continue

        implicated = []
        for i in station.member_indices:
            others = [slots[k] for k in range(len(slots)) if k != i]
            try:
                if _layer_is_one_island(ops, original, others, safety_offset_mm):
                    implicated.append(
                        "slot {} ({})".format(i + 1, slot_labels[i])
                        if slot_labels is not None
                        else "slot {}".format(i + 1)
                    )
            except Exception:
                pass
        if not implicated:
            implicated_text = (
                "neither member alone (only the whole pair's removal restores it)"
            )
        else:
            implicated_text = ", ".join(implicated)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        reason = (
            "{marker} opposing pair family {fam!r} at station x={x:.4f} mm, "
            "layer {idx} (Z={z:.2f} mm) -- pair ablation: the layer split "
            "into more than one island at safety_offset_mm={so:.2f}; the "
            "validator's ablation implicated {impl}; pair removal restored "
            "one island at safety_offset_mm={so:.2f}. The whole pair (both "
            "members) is omitted and this layer is marked degraded.".format(
                marker=MITRE_OMISSION_MARKER,
                fam=station.family_name,
                x=station.x_mm,
                idx=layer_index,
                z=print_z,
                so=safety_offset_mm,
                impl=implicated_text,
            )
        )
        omitted = OmittedWeb(
            x_mm=station.x_mm,
            reason=reason,
            family_index=station.family_index,
            family_name=station.family_name,
        )
        return PairAblationRescue(
            layer_index=layer_index,
            print_z=print_z,
            family_index=station.family_index,
            family_name=station.family_name,
            station_x_mm=station.x_mm,
            removed_indices=tuple(sorted(removed)),
            kept_indices=tuple(keep),
            implicated=tuple(implicated),
            safety_offset_mm=safety_offset_mm,
            elapsed_ms=elapsed_ms,
            omitted=omitted,
        )

    raise TopologyError(
        layer_index,
        print_z,
        error.cause
        + " Pair ablation was tried ({} pair station(s), one at a time, "
        "whole layer re-validated at safety_offset_mm={:.2f}) and did not "
        "restore the layer.".format(tried, safety_offset_mm),
    ) from error


def validate_with_pair_ablation(
    ops,
    validator,
    originals,
    replacement,
    intents: Sequence,
    slots: Sequence,
    spec,
    layer_index: int,
    print_z: float,
    slot_labels,
    family_indices,
    safety_offset_mm: float,
) -> Optional[PairAblationRescue]:
    """``validator.validate`` with the lazy retry. Returns ``None`` when the layer
    validated as it stood (the retry path is never entered), a
    ``PairAblationRescue`` when a single pair station's removal restored it
    (the caller must then drop ``rescue.removed_indices`` from its parallel
    lists and rebuild ``replacement``), or raises ``TopologyError``."""
    try:
        validator.validate(
            originals,
            replacement,
            slots,
            spec,
            layer_index,
            print_z,
            slot_labels=slot_labels,
            family_indices=family_indices,
        )
        return None
    except TopologyError as exc:
        return attempt_pair_ablation(
            exc,
            ops,
            validator,
            originals,
            intents,
            slots,
            spec,
            layer_index,
            print_z,
            slot_labels,
            family_indices,
            safety_offset_mm,
        )
