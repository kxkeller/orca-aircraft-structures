"""Regression: an unsupported skin bay is measured WITHIN one entry surface.

THE INVARIANT. ``RunDiagnostics.record_structure_layer`` measures a bay within
ONE entry surface. Pooling +Y-entry and -Y-entry stations into one list and
taking adjacent differences would be wrong: a bay is a span of ONE skin between
two webs that enter THAT skin, and a web entering the opposite surface does not
shorten it.

Whenever the two surfaces' stations interleave, a pooled scan reports roughly
HALF the true per-surface bay. The flat-plate critical stress goes as
``1/b^2``, so a 2x understated bay is a ~4x OVERSTATED ``sigma_cr`` -- in the
unsafe direction, on the one number the structural summary exists to report.

A pooled scan is easy to miss: when every family is co-located in X the pooled
list is effectively one surface's list and adjacent differences come out right
by coincidence. Offsetting the surfaces (e.g. by half a pitch) exposes it.

KNOWN LIMITATION, NOT guarded by this test: the bay is measured as
chord DX where the buckling formula wants skin ARCLENGTH. That needs the layer
polygon, which ``RunDiagnostics`` does not have. See the method's own comment.
"""
from __future__ import annotations

import unittest

from aircraft_core.diagnostics import RunDiagnostics


class SkinBayIsPerSurfaceTests(unittest.TestCase):
    def _bay(self, d):
        """The bay as the structural summary reports it -- read through the
        public surface, not a private attribute, so this test pins the number
        callers actually receive."""
        return d.structure_summary()["worst_skin_bay_mm"]

    def _diag(self):
        d = RunDiagnostics()
        d.set_skin_line_width_mm(0.42)
        return d

    def test_interleaved_surfaces_do_not_shorten_each_others_bays(self):
        """Upper webs at 0 and 10; lower webs at 5 and 15.

        Each surface has a 10 mm bay. Pooled, the sorted list is
        0, 5, 10, 15 and every adjacent gap is 5 mm -- half the truth.
        """
        d = self._diag()
        d.record_structure_layer(
            layer_index=7,
            print_z=1.4,
            skin_length_mm=100.0,
            web_depths_mm=[1.5, 1.5, 1.5, 1.5],
            entry_x_mm=[0.0, 10.0, 5.0, 15.0],
            entry_lower_flags=[False, False, True, True],
        )
        self.assertAlmostEqual(self._bay(d), 10.0, places=9)

    def test_pooled_scan_is_what_the_old_behaviour_reported(self):
        """The same stations WITHOUT surface flags still report 5 mm.

        This is not an endorsement of that answer -- it pins the
        backward-compatible path, so a caller that cannot say which surface a
        station entered keeps its exact previous behaviour rather than
        silently getting a different number.
        """
        d = self._diag()
        d.record_structure_layer(
            layer_index=7,
            print_z=1.4,
            skin_length_mm=100.0,
            web_depths_mm=[1.5, 1.5, 1.5, 1.5],
            entry_x_mm=[0.0, 10.0, 5.0, 15.0],
        )
        self.assertAlmostEqual(self._bay(d), 5.0, places=9)

    def test_co_located_surfaces_are_unchanged_by_the_fix(self):
        """Every tracked config co-locates the surfaces. With flags or without,
        the answer must be identical there -- which is why this defect never
        produced a visible symptom."""
        args = dict(
            layer_index=3,
            print_z=0.6,
            skin_length_mm=80.0,
            web_depths_mm=[1.5, 1.5, 1.5, 1.5],
            entry_x_mm=[0.0, 0.0, 8.0, 8.0],
        )
        pooled = self._diag()
        pooled.record_structure_layer(**args)
        split = self._diag()
        split.record_structure_layer(
            entry_lower_flags=[False, True, False, True], **args
        )
        self.assertAlmostEqual(self._bay(pooled), 8.0, places=9)
        self.assertAlmostEqual(self._bay(split), 8.0, places=9)

    def test_a_surface_with_one_web_contributes_no_bay(self):
        """There is no 'between' with fewer than two entries on a surface.
        The lone lower web must not pair with an upper one."""
        d = self._diag()
        d.record_structure_layer(
            layer_index=1,
            print_z=0.2,
            skin_length_mm=50.0,
            web_depths_mm=[1.5, 1.5, 1.5],
            entry_x_mm=[0.0, 12.0, 6.0],
            entry_lower_flags=[False, False, True],
        )
        self.assertAlmostEqual(self._bay(d), 12.0, places=9)

    def test_mismatched_flag_length_falls_back_rather_than_guessing(self):
        """A caller that supplies the wrong number of flags gets the old
        pooled behaviour, not a partially-applied split. Guessing which
        stations the flags referred to is exactly how a wrong number that
        looks right gets produced."""
        d = self._diag()
        d.record_structure_layer(
            layer_index=2,
            print_z=0.4,
            skin_length_mm=50.0,
            web_depths_mm=[1.5, 1.5, 1.5, 1.5],
            entry_x_mm=[0.0, 10.0, 5.0, 15.0],
            entry_lower_flags=[False, True],
        )
        self.assertAlmostEqual(self._bay(d), 5.0, places=9)


if __name__ == "__main__":
    unittest.main()
