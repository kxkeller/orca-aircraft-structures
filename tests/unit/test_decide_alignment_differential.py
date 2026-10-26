"""Differential test: ``aircraft_core.alignment.decide_alignment`` against the
INLINE decision logic it was extracted from (``capability.py``'s
``_check_mesh_alignment``, clauses 2-4), copied below.

The extraction is meant to be behaviour-preserving. Hand-picked fixtures
exercise specific cases; this file instead compares the two implementations
directly over an exhaustive discretised grid PLUS randomised fuzzing, so a
behavioural drift the hand-picked fixtures happen not to reach still has a
chance to be caught.

The reference function below is a structural copy of the inline boolean
logic (variable names, branch order, and comparisons unchanged) with only
the RETURN SHAPE changed, to match ``decide_alignment``'s documented outcome
vocabulary (``pass``/``raise_bbox``/``raise_centroid_off_top``/
``raise_centroid_near_top_skip_disabled``/``skip``) instead of the
raise/return-None/return-skip_record calling convention the inline version
used at its call site. The mapping from one calling convention to the other
is a bookkeeping label, not a behavioural difference, and is spelled out at
each return site below.


The source method is ``_check_mesh_alignment`` in ``capability.py``
(clauses 2-4).
"""
import random
import unittest

from aircraft_core.alignment import decide_alignment

TERMS = ("min_x", "max_x", "min_y", "max_y", "centroid_x", "centroid_y")
BBOX_TERMS = ("min_x", "max_x", "min_y", "max_y")
CENTROID_TERMS = ("centroid_x", "centroid_y")

TOL = 0.02
EPS = 1e-6
GRID_VALUES = (0.0, TOL - EPS, TOL, TOL + EPS, TOL * 25.0)  # "large" = 25x


def _reference_decide_alignment(deltas, effective, near_top_extremum,
                                 tip_alignment_skip_enabled):
    """Structural copy of the pre-f98aade inline logic (``git show
    f98aade^:plugin/aircraft_structures/capability.py`` lines
    4766-4854, ``_check_mesh_alignment`` clauses 2-4). Only the RETURN
    SHAPE is translated (raise/None/skip_record -> the same outcome dict
    vocabulary ``decide_alignment`` documents), so this function can be
    compared directly against it.
    """
    bbox_terms = BBOX_TERMS
    bbox_over = [t for t in bbox_terms if deltas[t] > effective[t]]
    if bbox_over:
        worst_over_bbox_term = max(bbox_over, key=deltas.get)
        worst_over_bbox_delta = deltas[worst_over_bbox_term]
        # Pre-f98aade: `raise TopologyError(...)` naming worst_over_bbox_term
        # / worst_over_bbox_delta / effective[worst_over_bbox_term].
        return {
            "outcome": "raise_bbox",
            "worst_term": worst_over_bbox_term,
            "worst_delta": worst_over_bbox_delta,
            "effective_tolerance": effective[worst_over_bbox_term],
        }

    centroid_terms = CENTROID_TERMS
    centroid_over = [t for t in centroid_terms if deltas[t] > effective[t]]
    if not centroid_over:
        # Pre-f98aade: `return None` (caller proceeds normally).
        return {"outcome": "pass"}
    worst_over_centroid_term = max(centroid_over, key=deltas.get)
    worst_over_centroid_delta = deltas[worst_over_centroid_term]

    if not near_top_extremum:
        # Pre-f98aade: Clause 4, `raise TopologyError(...)`.
        return {
            "outcome": "raise_centroid_off_top",
            "worst_term": worst_over_centroid_term,
            "worst_delta": worst_over_centroid_delta,
            "effective_tolerance": effective[worst_over_centroid_term],
        }

    if not tip_alignment_skip_enabled:
        # Pre-f98aade: Clause 3, `raise TopologyError(...)`.
        return {
            "outcome": "raise_centroid_near_top_skip_disabled",
            "worst_term": worst_over_centroid_term,
            "worst_delta": worst_over_centroid_delta,
            "effective_tolerance": effective[worst_over_centroid_term],
        }

    # Pre-f98aade: opted-in skip, `return skip_record` (layer/print_z/
    # band_mm/deltas_mm -- the layer-context fields live in capability.py,
    # not in this boolean core; the decision-relevant fields are worst_term/
    # worst_delta/effective_tolerance, identical to the other three branches).
    return {
        "outcome": "skip",
        "worst_term": worst_over_centroid_term,
        "worst_delta": worst_over_centroid_delta,
        "effective_tolerance": effective[worst_over_centroid_term],
    }


def _effective_variants():
    """Two variants standing in for "sensitivity widening off/on": OFF is every
    term at the base tolerance; ON widens every term the inline code could ever
    widen (all six -- the inline code only widens a term already over base, but
    effective is an INPUT to both functions here, not something either
    recomputes, so exercising a widened value on every term is the more
    thorough of the two readings and a superset of "only the over-base terms
    are widened")."""
    off = {t: TOL for t in TERMS}
    on = {t: TOL * 2.0 for t in TERMS}
    return (off, on)


class ExhaustiveGridDifferentialTests(unittest.TestCase):
    """Full Cartesian grid: each of the six terms independently takes each of the
    five values in ``GRID_VALUES`` (0, tol-eps, tol, tol+eps, a large value),
    crossed with near-top band in/out, tip_alignment_skip_enabled true/false,
    and the two effective-tolerance variants standing in for sensitivity
    widening off/on. ``5**6 * 2 * 2 * 2 = 125000`` cases.
    """

    def test_full_grid_agrees(self):
        mismatches = []
        count = 0
        effective_variants = _effective_variants()
        for min_x in GRID_VALUES:
            for max_x in GRID_VALUES:
                for min_y in GRID_VALUES:
                    for max_y in GRID_VALUES:
                        for centroid_x in GRID_VALUES:
                            for centroid_y in GRID_VALUES:
                                deltas = {
                                    "min_x": min_x, "max_x": max_x,
                                    "min_y": min_y, "max_y": max_y,
                                    "centroid_x": centroid_x,
                                    "centroid_y": centroid_y,
                                }
                                for effective in effective_variants:
                                    for near_top in (True, False):
                                        for skip_enabled in (True, False):
                                            count += 1
                                            got = decide_alignment(
                                                deltas, effective, near_top,
                                                skip_enabled,
                                            )
                                            want = _reference_decide_alignment(
                                                deltas, effective, near_top,
                                                skip_enabled,
                                            )
                                            if got != want:
                                                mismatches.append(
                                                    (dict(deltas),
                                                     dict(effective),
                                                     near_top, skip_enabled,
                                                     got, want)
                                                )
                                                if len(mismatches) >= 10:
                                                    raise AssertionError(
                                                        "10+ mismatches; "
                                                        "first 10: {}".format(
                                                            mismatches)
                                                    )
        self.assertEqual(count, 5 ** 6 * 2 * 2 * 2)
        self.assertEqual(mismatches, [])


class RandomFuzzDifferentialTests(unittest.TestCase):
    """Randomised deltas/effective tolerances/flags, beyond the discretised
    grid above -- a fixed seed for reproducibility."""

    def test_random_fuzz_agrees(self):
        rng = random.Random(20260914)
        mismatches = []
        N = 20000
        for _ in range(N):
            deltas = {t: rng.uniform(0.0, TOL * 30.0) for t in TERMS}
            effective = {
                t: rng.uniform(TOL, TOL * 5.0) for t in TERMS
            }
            near_top = rng.choice([True, False])
            skip_enabled = rng.choice([True, False])
            got = decide_alignment(deltas, effective, near_top, skip_enabled)
            want = _reference_decide_alignment(
                deltas, effective, near_top, skip_enabled)
            if got != want:
                mismatches.append(
                    (deltas, effective, near_top, skip_enabled, got, want))
                if len(mismatches) >= 10:
                    break
        self.assertEqual(mismatches, [], "first mismatches: {}".format(
            mismatches[:10]))


if __name__ == "__main__":
    unittest.main()
