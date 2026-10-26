"""Regression test for the tie-gate floating-point knife-edge bug
in ``tools/weight_stiffness_sweep.py``'s ``_layer_tie_metrics``.

``_generate_opposing_pair_family`` (``aircraft_core/generator.py``) builds an
opposing pair's default centre spacing so its wall-to-wall gap equals
``FUSE_THRESHOLD_MM`` (the 0.021 mm hard-fuse point) EXACTLY, in exact
arithmetic. In floating point, that equality is not exact: measured directly
on a real wing (row H2, layer 455 vs 470), the computed
``wall_gap`` was ``0.021000000000001573`` on one layer and
``0.02099999999999802`` on the next -- both ~1e-15 mm from 0.021, pure rounding
noise in ``anchor_x -+ offset_mm/2`` whose sign happened to trend smoothly with
span position. A bare ``wall_gap <= FUSE_THRESHOLD_MM`` comparison flipped on
this noise, undercounting a real, geometrically-identical tie on 630 of 998
layers (368/998 found instead of 998/998) -- not a real per-layer geometric
effect, since the pair's own width/offset/wall_mm are layer-independent family
constants.

The gate therefore adds ``_TIE_GATE_EPSILON_MM = 1e-9`` (~1e6x the observed
float noise, ~1e7x smaller than any real geometric quantity this module
measures) to both the ``fin_fin`` and ``fin_socket_fin`` wall-gap comparisons.

This test pins that behaviour with the EXACT floating-point values measured on
the real wing (not synthetic round numbers), on a minimal synthetic layer (a
single rectangle, no real mesh/generator/host), so it runs in milliseconds and
never depends on a real STL fixture. With ``_TIE_GATE_EPSILON_MM = 0.0`` the
layer-455 case below fails.
"""

import math
import os
import sys
import unittest
from types import SimpleNamespace

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.join(_THIS_DIR, "..", "..")
for _p in (_THIS_DIR, REPO_ROOT, os.path.join(REPO_ROOT, "tools")):
    _p = os.path.abspath(_p)
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import WebIntent  # noqa: E402
from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402

if HAVE_SHAPELY:
    import weight_stiffness_sweep as w  # noqa: E402


def _make_pair(gap_mm, width_mm=0.0):
    """Two full-depth WebIntents, one upper one lower entry, at x=0.0 and
    x=gap_mm, both zero-width -- so ``_layer_tie_metrics``'s own
    ``void_gap``/``wall_gap`` computation (with ``wall_mm=0.0`` passed to
    the function under test) reduces to exactly ``gap_mm``, no other
    arithmetic in between. This isolates the comparison itself from the
    real geometry that produced the original floating-point noise, while
    reusing the EXACT noisy float values measured on the real wing."""
    upper = WebIntent(start=(0.0, 10.0), end=(0.0, -10.0), width_mm=width_mm,
                       family_index=0, family_name="upper", entry_lower=False)
    lower = WebIntent(start=(gap_mm, -10.0), end=(gap_mm, 10.0), width_mm=width_mm,
                       family_index=0, family_name="lower", entry_lower=True)
    return upper, lower


@unittest.skipUnless(HAVE_SHAPELY, "shapely not installed; tools/weight_stiffness_sweep.py requires it")
class TieGateFuseEpsilonTest(unittest.TestCase):
    def setUp(self):
        self.ops = MiterShapelyOps()
        self.poly = self.ops.polygon_from_points(
            [(-50, -20), (50, -20), (50, 20), (-50, 20)])
        self.frame = LayerFrame.from_polygon(self.ops, self.poly)
        self.families = [SimpleNamespace(web_depth_mm=None)]

    def _tie_found(self, gap_mm):
        upper, lower = _make_pair(gap_mm)
        res = SimpleNamespace(intents=[upper, lower], omitted=[])
        record = w._layer_tie_metrics(
            res, self.families, self.frame, wall_mm=0.0, ops=self.ops,
            poly=self.poly, skin_margin_mm=1.0, layer_index=0, z=0.0)
        kinds = [iv[2] for iv in record["tie_intervals"]]
        return "fin_fin" in kinds

    def test_float_noise_just_above_threshold_counts_as_a_tie(self):
        # Measured on row H2, layer 455 -- wall_gap = 0.021000000000001573,
        # i.e. FUSE_THRESHOLD_MM plus ~1.57e-15 mm of rounding noise. This
        # is the exact case a bare ``<=`` comparison wrongly excludes.
        gap = 0.021000000000001573
        self.assertGreater(gap, w.FUSE_THRESHOLD_MM,
                            "fixture must reproduce the just-ABOVE-threshold case")
        self.assertTrue(
            self._tie_found(gap),
            "a wall_gap {} mm from FUSE_THRESHOLD_MM by only {} mm (pure "
            "float noise) must still count as a fused tie".format(
                gap, gap - w.FUSE_THRESHOLD_MM))

    def test_float_noise_just_below_threshold_counts_as_a_tie(self):
        # Measured on row H2, layer 470 -- wall_gap = 0.02099999999999802,
        # i.e. FUSE_THRESHOLD_MM minus ~1.98e-15 mm. This side of the noise
        # passes even a bare ``<=``; pinned here so
        # a future change to the comparison direction cannot silently flip
        # it too.
        gap = 0.02099999999999802
        self.assertLess(gap, w.FUSE_THRESHOLD_MM,
                         "fixture must reproduce the just-BELOW-threshold case")
        self.assertTrue(self._tie_found(gap))

    def test_a_real_non_fusing_gap_is_still_excluded(self):
        # FUSE_THRESHOLD_MM + 1e-6 mm is seven orders of magnitude larger
        # than the observed ~1e-15 mm float noise -- a genuinely wider,
        # non-fusing gap. The epsilon must not swallow this.
        gap = w.FUSE_THRESHOLD_MM + 1e-6
        self.assertFalse(
            self._tie_found(gap),
            "a wall_gap {} mm above FUSE_THRESHOLD_MM is a real gap, not "
            "float noise, and must NOT be counted as fused".format(
                gap - w.FUSE_THRESHOLD_MM))

    def test_epsilon_constant_is_between_noise_and_real_gaps(self):
        # Documents the sizing choice this test suite depends on: the
        # epsilon must be larger than the observed ~1e-15 mm noise and
        # smaller than any real geometric gap this module would need to
        # exclude (1e-6 mm, the smallest "real" gap exercised above).
        noise_scale = abs(0.021000000000001573 - w.FUSE_THRESHOLD_MM)
        self.assertGreater(w._TIE_GATE_EPSILON_MM, noise_scale * 10)
        self.assertLess(w._TIE_GATE_EPSILON_MM, 1e-6 / 10)


if __name__ == "__main__":
    unittest.main()
