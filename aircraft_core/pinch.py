"""Pinched-section shutdown.

A layer can fail the validator's island-split check (``slot disconnects
material island after... safety offset``) with NO slot to blame: the wing's OWN
cross-section is pinched thinner than the wall-safety erosion, so the bare host
contour, with no slots cut at all, already erodes into more than one piece.
A measured case is layer 1098 of a root-down elevon wing: the host closes a
0.022 mm gap (``slice_closing_radius``) into a neck that any erosion of 0.1 mm
or more severs. No web placement can help such a layer, and pair ablation
cannot either.

That ONE layer is printed as plain skin with no internal structure, recorded
through the shutdown mechanism (``LayerRecord(mutated=False,
shutdown=True...)``). It is not transformed: it is left exactly as the host
sliced it and reported.

The check is lazy. It runs only after ``TopologyValidator.validate`` (and pair
ablation) has already failed with the island-split cause, so healthy layers
never pay for it, and it fires only when the bare contour itself splits; if it
does not split, the caller re-raises the original error unchanged. Same
``ops.offset`` and the same erosion distance as the validator.

No Orca imports: the plugin only calls ``pinched_section_after_failure``.
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .pair_ablation import PairAblationRescue, validate_with_pair_ablation
from .validate import ISLAND_SPLIT_CAUSE_MARKER, TopologyError

PINCHED_SECTION_KIND = "pinched_section"

# Bisection resolution for the minimum neck erosion radius, mm.
_NECK_RADIUS_TOLERANCE_MM = 0.0005


@dataclass(frozen=True)
class PinchedSection:
    layer_index: int
    print_z: float
    safety_offset_mm: float
    piece_count: int
    piece_areas_mm2: Tuple[float, ...]
    min_neck_erosion_radius_mm: float
    reason: str

    def as_dict(self) -> dict:
        return {
            "layer_index": self.layer_index,
            "print_z": round(self.print_z, 5),
            "safety_offset_mm": self.safety_offset_mm,
            "piece_count": self.piece_count,
            "piece_areas_mm2": [round(a, 3) for a in self.piece_areas_mm2],
            "min_neck_erosion_radius_mm": round(self.min_neck_erosion_radius_mm, 4),
            "reason": self.reason,
        }


def _is_island_split_error(error: BaseException) -> bool:
    return (
        isinstance(error, TopologyError)
        and ISLAND_SPLIT_CAUSE_MARKER in getattr(error, "cause", "")
    )


def _eroded_pieces(ops, polygon, radius_mm: float) -> List:
    return list(ops.offset(polygon, -radius_mm))


def min_neck_erosion_radius_mm(ops, polygon, safety_offset_mm: float) -> float:
    """Smallest erosion radius, within ``_NECK_RADIUS_TOLERANCE_MM``, at
    which ``polygon`` is no longer one piece (bisection between an erosion
    known to leave one piece and ``safety_offset_mm``, known to split it)."""
    lo, hi = 0.0, safety_offset_mm
    while hi - lo > _NECK_RADIUS_TOLERANCE_MM:
        mid = 0.5 * (lo + hi)
        if len(_eroded_pieces(ops, polygon, mid)) > 1:
            hi = mid
        else:
            lo = mid
    return hi


def check_pinched_section(
    ops,
    originals: Sequence,
    safety_offset_mm: float,
    layer_index: int,
    print_z: float,
) -> Optional[PinchedSection]:
    """``PinchedSection`` when the host reports exactly ONE outer contour and
    that bare contour, with no slots, erodes by ``safety_offset_mm`` into more
    than one piece; ``None`` otherwise."""
    if len(originals) != 1:
        return None
    original = originals[0]
    pieces = _eroded_pieces(ops, original, safety_offset_mm)
    if len(pieces) <= 1:
        return None
    areas = tuple(sorted((abs(ops.area(p)) for p in pieces), reverse=True))
    neck = min_neck_erosion_radius_mm(ops, original, safety_offset_mm)
    reason = (
        "section pinched below wall-safety erosion ({so:.2f} mm): bare host "
        "contour (no slots) splits into {n} pieces ({areas} mm2), first "
        "splitting at an erosion radius of about {neck:.3f} mm -- layer {idx} "
        "(Z={z:.2f} mm) printed as plain skin.".format(
            so=safety_offset_mm,
            n=len(pieces),
            areas=", ".join("{:.1f}".format(a) for a in areas),
            neck=neck,
            idx=layer_index,
            z=print_z,
        )
    )
    return PinchedSection(
        layer_index=layer_index,
        print_z=print_z,
        safety_offset_mm=safety_offset_mm,
        piece_count=len(pieces),
        piece_areas_mm2=areas,
        min_neck_erosion_radius_mm=neck,
        reason=reason,
    )


def pinched_section_after_failure(
    error: BaseException,
    ops,
    originals: Sequence,
    safety_offset_mm: float,
    layer_index: int,
    print_z: float,
) -> Optional[PinchedSection]:
    """Called with the ``TopologyError`` validation (and ablation) raised. Returns
    ``None`` -- the caller must re-raise ``error`` untouched -- unless the
    error is the island-split cause AND the bare contour itself splits."""
    if not _is_island_split_error(error):
        return None
    return check_pinched_section(
        ops, originals, safety_offset_mm, layer_index, print_z
    )


def validate_with_pinch_shutdown(
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
) -> Tuple[Optional[PairAblationRescue], Optional[PinchedSection]]:
    """``validate_with_pair_ablation`` plus the lazy pinch check.

    Returns ``(pair_rescue, None)`` when the layer validated (``pair_rescue``
    is ``None`` when it validated as it stood, or rescue), and ``(None,
    pinch)`` when validation failed with the island-split cause, pair ablation
    could not rescue it, and the bare contour itself splits under the safety
    erosion. Every other failure re-raises the original ``TopologyError``
    untouched. The pinch check is never entered for a layer that validates."""
    try:
        rescue = validate_with_pair_ablation(
            ops, validator, originals, replacement, intents, slots, spec,
            layer_index, print_z, slot_labels, family_indices,
            safety_offset_mm,
        )
    except TopologyError as error:
        pinch = pinched_section_after_failure(
            error, ops, originals, safety_offset_mm, layer_index, print_z
        )
        if pinch is None:
            raise
        return None, pinch
    return rescue, None


def layer_record_fields(pinch: PinchedSection) -> Tuple[List[dict], dict]:
    """``(degradations, extra)`` for the shutdown ``LayerRecord`` of a pinched
    layer (the shutdown shape: ``mutated=False, shutdown=True``)."""
    degradations = [{
        "kind": PINCHED_SECTION_KIND,
        "x_mm": 0.0,
        "reason": "DEGRADED -- " + pinch.reason,
    }]
    extra = {
        "reason": PINCHED_SECTION_KIND,
        PINCHED_SECTION_KIND: pinch.as_dict(),
    }
    return degradations, extra
