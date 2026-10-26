"""The merge scan compares ALL pairs, not only consecutive ones: regression
coverage.

THE BUG CLASS (seen on a real in-host slice, ``presets/plugin/
anisogrid_lattice_config.json`` on ``tests/fixtures/coupon/
coupon_20x20x15.stl``, layer 5, Z=1.20 mm): ``_merge_cross_family_nodes`` and
its ``_yield_lattice_to_primary`` pre-pass must not compare only CONSECUTIVE
pairs in the footprint-sorted candidate list. That would assume every skipped
comparison was skipped because its gap already cleared ``min_wall_mm`` -- the
one case where the gap to whatever sits outside a component is identical to
what an already-consulted neighbour had. The ``family_index``/
``entry_lower``/``is_scar`` guards are CATEGORICAL, not gap-based: they reject
a pair no matter how close its gap is. A categorically-rejected candidate
sitting BETWEEN two candidates that genuinely belong in the same merge/yield
band would hide the real pair from ever being compared at all.

On the anisogrid config this arises systematically: the
``lattice_plus_upper``/``lattice_minus_upper`` pair (same role, same
``entry_lower``) is separated by ``lattice_minus_lower`` -- a THIRD family
anchored at the IDENTICAL X by construction (shared ``web_position``, opposite
``z_drift_mm_per_mm``) but on the OPPOSITE surface (``entry_lower`` differs
from both) -- which always sorts between them and is skipped against both
neighbours for the ``entry_lower`` guard, never for a gap reason. If the two
``+Y`` candidates were never compared directly, merge would not fire and the
layer would fail with a sub-``min_wall_mm`` "leaves only 0.0790 mm of wall"
``TopologyError``.

This module pins the mechanism directly (minimized ``WebIntent``s through the
real ``_merge_cross_family_nodes`` entry point, same precise-gap technique
``tests/unit/test_node_merge.py`` uses), independent of the anisogrid
config's own numbers. Two scenarios: (1) a masked SAME-ROLE
(lattice-vs-lattice) pair must still merge -- the existing role-yield tests
(``tests/unit/test_role_yield.py``) only construct a CROSS-role pair; (2) a
masked CROSS-role pair must still yield (a stray "restore merging" change
would truncate a primary member's depth exactly where role-yield exists to
stop that).
"""
import unittest

from aircraft_core.generator import LayerContext, StructureGenerator, WebIntent
from aircraft_core.spec import StructureSpec, WebFamily

# the merge helpers REQUIRE a LayerContext (see generator.py's merge-halo
# raise). These direct-call tests never reach that raise, so any concrete
# context serves; named rather than inlined to make that inertness obvious.
_MERGE_TEST_CTX = LayerContext(layer_index=0, print_z=0.0)


_SAFETY_OFFSET_MM = 0.42
_MIN_WALL_MM = 2.0 * _SAFETY_OFFSET_MM


class SameRoleCrossFamilyMaskingTests(unittest.TestCase):
    """(1) A same-role (skin_lattice vs skin_lattice) cross-family pair
    masked by an intervening, opposite-``entry_lower`` third candidate must
    still merge -- the exact mechanism of the host failure this module's
    docstring describes.

    ``A`` (family 0, +Y) and ``C`` (family 2, +Y) sit 0.08 mm apart edge
    -to-edge -- inside the 0.84 mm band, so says they must merge. ``B`` (family
    1, -Y) is TIED with ``A`` at the identical X (mirroring the anisogrid
    config's shared-anchor construction) and therefore sorts between ``A`` and
    ``C`` in the footprint-sorted scan. ``B`` can never link to either
    neighbour (``entry_lower`` differs from both), but that categorical
    rejection must not prevent ``A``/``C`` from being compared to EACH OTHER.
    """

    def test_masked_pair_merges_despite_intervening_opposite_side_candidate(self):
        a = WebIntent(
            start=(0.0, 10.0), end=(0.0, 8.5), width_mm=0.02,
            family_index=0, family_name="lattice_plus", entry_lower=False,
        )  # footprint [-0.01, 0.01]
        b = WebIntent(
            start=(0.0, -10.0), end=(0.0, -8.5), width_mm=0.02,
            family_index=1, family_name="lattice_masker", entry_lower=True,
        )  # SAME X as a, opposite surface -- always sorts between a and c.
        c = WebIntent(
            start=(0.1, 10.0), end=(0.1, 8.5), width_mm=0.02,
            family_index=2, family_name="lattice_minus", entry_lower=False,
        )  # footprint [0.09, 0.11] -- edge gap to a is 0.08 mm (< 0.84 mm).

        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [a, b, c], safety_offset_mm=_SAFETY_OFFSET_MM, ctx=_MERGE_TEST_CTX
        )

        self.assertEqual(omitted, [])
        # b never links to anything (opposite entry_lower from both
        # neighbours) -- exactly one standalone intent for it, plus one
        # MERGED intent absorbing a and c.
        self.assertEqual(len(kept), 2)
        by_membership = {frozenset(w.family_membership()): w for w in kept}
        self.assertIn(frozenset({1}), by_membership, "b must survive standalone")
        merged_key = frozenset({0, 2})
        self.assertIn(
            merged_key, by_membership,
            "a (family 0) and c (family 2) must merge despite b sorting "
            "between them -- this is the masking bug.",
        )
        merged = by_membership[merged_key]
        self.assertEqual(merged.merged_family_indices, (0, 2))
        merged_min_x = merged.start[0] - merged.width_mm / 2.0
        merged_max_x = merged.start[0] + merged.width_mm / 2.0
        self.assertAlmostEqual(merged_min_x, -0.01, places=6)
        self.assertAlmostEqual(merged_max_x, 0.11, places=6)


class RoleYieldSurvivesMaskingTests(unittest.TestCase):
    """(2) must keep working even when the cross-role pair it needs to see is
    masked by the SAME kind of intervening, opposite-``entry_lower`` candidate
    -- proving the ordinary merge scan and ``_yield_lattice_to_primary`` both
    compare all pairs (the two scans must use IDENTICAL linking conditions:
    "two measurements that must agree").

    ``P`` (``primary_structure``, +Y) and ``L`` (``skin_lattice``, +Y) sit 0.08
    mm apart -- inside the band, cross-role, so ``L`` must yield (be
    omitted), not merge. ``M`` (``skin_lattice``, -Y) is TIED with ``P`` at the
    identical X and sorts between them, exactly as ``B`` did in
    ``SameRoleCrossFamilyMaskingTests`` above.
    """

    def test_masked_pair_still_yields_and_primary_is_untouched(self):
        spec = StructureSpec(
            gap_ratio=0.05,
            skin_margin_mm=1.0,
            families=[
                WebFamily(name="spar_aft", role="primary_structure", rib_pitch_mm=None, web_position=0.5),
                WebFamily(name="lattice_masker", role="skin_lattice", rib_pitch_mm=None, web_position=0.5, entry_lower=True),
                WebFamily(name="lattice_plus_upper", role="skin_lattice", rib_pitch_mm=None, web_position=0.5),
            ],
        )
        p = WebIntent(
            start=(0.0, 10.0), end=(0.0, -9.0), width_mm=0.02,
            family_index=0, family_name="spar_aft", entry_lower=False,
        )  # full-depth primary, footprint [-0.01, 0.01]
        m = WebIntent(
            start=(0.0, -10.0), end=(0.0, -8.5), width_mm=0.02,
            family_index=1, family_name="lattice_masker", entry_lower=True,
        )  # SAME X as p, opposite surface -- always sorts between p and l.
        l = WebIntent(
            start=(0.1, 10.0), end=(0.1, 8.5), width_mm=0.02,
            family_index=2, family_name="lattice_plus_upper", entry_lower=False,
        )  # footprint [0.09, 0.11] -- edge gap to p is 0.08 mm (< 0.84 mm).

        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [p, m, l], safety_offset_mm=_SAFETY_OFFSET_MM, spec=spec, ctx=_MERGE_TEST_CTX
        )

        self.assertEqual(len(omitted), 1)
        self.assertEqual(omitted[0].family_name, "lattice_plus_upper")
        self.assertTrue(omitted[0].reason.startswith("role yield:"))

        # The primary keeps its own original footprint and depth,
        # byte-identical -- whole point -- and is never merged with the masking
        # lattice candidate on the opposite surface either.
        self.assertEqual(len(kept), 2)
        by_name = {w.family_name: w for w in kept}
        self.assertIn("spar_aft", by_name)
        self.assertEqual(by_name["spar_aft"].merged_family_indices, ())
        self.assertEqual(by_name["spar_aft"].start, p.start)
        self.assertEqual(by_name["spar_aft"].end, p.end)
        self.assertIn("lattice_masker", by_name)
        self.assertEqual(by_name["lattice_masker"].merged_family_indices, ())


if __name__ == "__main__":
    unittest.main()
