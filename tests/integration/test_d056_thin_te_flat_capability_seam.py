"""Capability-seam test: drive ``capability._build_mesh_sectioner`` (the function
``execute()`` calls before the per-layer loop; see the module docstring of
``test_m38_sharp_band_capability_seam`` for why not ``execute()`` itself)
through ``fake_orca`` with a thin-trailing-edge-flat wing at an injected
rotation, and assert the returned alignment report took the thin-flat path,
carries its provenance, and does not trip the fatal flag.

It runs the GENERATED BUNDLE (``plugin/aircraft_structures/aircraft_structures.py``),
so it fails if ``tools/build_plugin.py`` was not re-run after the
``aircraft_core.alignment`` change, and if ``capability.py`` does not pass the
sampler or does not copy the thin-flat provenance key into the report.

Section generators are the seam test's (the project's own NACA formulas,
mesh-safe single-occurrence rings); a TE truncated at 99.5% of chord is a
0.99%-of-thickness flat, well inside the thin-flat bound.
"""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import fake_orca  # noqa: E402
from aircraft_core.alignment import rotate_points_deg  # noqa: E402
from test_m38_sharp_band_capability_seam import (  # noqa: E402
    _blunt_outline_mm,
    _sharp_outline_mm,
)

_SPAN_MM = 100.0
_THIN_TRUNC = 0.995     # face/thickness ~0.012 (bound 0.025)
_THICK_TRUNC = 0.96     # face/thickness 0.11: refused by the sharp reading and the thin flat


class CapabilityThinFlatSeamTests(unittest.TestCase):
    def setUp(self):
        self._fake_orca, restore = fake_orca.install()
        self.addCleanup(restore)
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.entry_file = os.path.join(self._tmp.name, "aircraft_structures.py")
        self.namespace = fake_orca.exec_bundle(self.entry_file)

    def _alignment(self, stations_mm, theta_inj_deg):
        stations = [(z, rotate_points_deg(ring, theta_inj_deg)) for z, ring in stations_mm]
        mesh = fake_orca.loft_mesh_mm(stations)
        obj = fake_orca.PrintObject(
            "thin_flat_like", [], model_object=fake_orca.identity_model_object(mesh))
        Step = self._fake_orca.slicing.Step
        ctx = self._fake_orca.slicing.SlicingPipelineContext(
            step=Step.posSlice, obj=obj, config={}, cancelled_fn=None)
        cap = self.namespace["AircraftStructuresM1"]()
        result = cap._build_mesh_sectioner(ctx)
        self.assertIsNotNone(result, "cap._build_mesh_sectioner returned None")
        return result[2]["alignment"], cap

    def test_the_fixture_really_is_a_thin_flat_before_the_seam_test(self):
        from aircraft_core import alignment as AL
        from aircraft_core.alignment import BluntTrailingEdgeError, resolve_theta_deg
        thin = _blunt_outline_mm(_THIN_TRUNC)
        with self.assertRaises(BluntTrailingEdgeError):
            resolve_theta_deg(thin)                   # per-section verdict: refused
        info, ref = AL._thin_flat_station(thin)       # thin-flat reading
        self.assertIsNone(ref)
        self.assertLessEqual(info["face_over_thickness"], AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS)
        info, ref = AL._thin_flat_station(_blunt_outline_mm(_THICK_TRUNC))
        self.assertIsNone(info)

    def test_theta_source_is_thin_te_flat_and_theta_matches_injection(self):
        thin = _blunt_outline_mm(_THIN_TRUNC)
        for theta_inj in (20.0, -15.0, 90.0):
            with self.subTest(theta_inj=theta_inj):
                alignment, cap = self._alignment(
                    [(0.0, thin), (_SPAN_MM, thin)], theta_inj)
                self.assertIsNone(alignment["error"])
                self.assertIsNone(getattr(cap, "_alignment_fatal_error", None))
                self.assertTrue(alignment["rotation_applied"])
                self.assertEqual(alignment["theta_source"], "thin_te_flat")
                self.assertAlmostEqual(alignment["theta_deg"], theta_inj, delta=0.2)
                tf = alignment["thin_te_flat"]
                self.assertGreaterEqual(tf["run_stations"], 5)
                self.assertLessEqual(tf["mid_span"]["face_over_thickness"], 0.025)
                self.assertLess(tf["adjacent_delta_max_deg"], 0.4)
                self.assertIsNotNone(alignment["blunt_crosscheck"])
                self.assertIsNone(alignment["sharp_band_fracs"])
                self.assertEqual(alignment["excluded_stations"], [])

    def test_an_excluded_standard_station_keeps_the_thin_flat_refusal_reason(self):
        # The 0.9 standard station is a thick flat (refused by the sharp
        # reading AND by the thin flat); mid-span and a long run around it are
        # thin flats. The report's reason for that station must keep the
        # thin-flat guard, value and bound, and not repeat the raw
        # BluntTrailingEdgeError refusal sentence.
        thin = _blunt_outline_mm(_THIN_TRUNC)
        thick = _blunt_outline_mm(_THICK_TRUNC)
        alignment, _cap = self._alignment(
            [(0.0, thin), (80.0, thin), (81.0, thick), (_SPAN_MM, thick)], 10.0)
        self.assertEqual(alignment["theta_source"], "thin_te_flat")
        reasons = [e["reason"] for e in alignment["excluded_stations"]]
        self.assertEqual(len(reasons), 1)
        self.assertIn("Thin-flat reading also refused: face_over_chord guard, measured", reasons[0])
        self.assertIn("excluded from the spread check only", reasons[0])
        self.assertIn("thin trailing-edge flat", reasons[0])
        self.assertNotIn("The slice is refused", reasons[0])

    def test_a_sharp_band_outboard_still_takes_precedence_at_the_seam(self):
        thin = _blunt_outline_mm(_THIN_TRUNC)
        sharp = _sharp_outline_mm()
        alignment, _cap = self._alignment(
            [(0.0, thin), (85.0, thin), (86.0, sharp), (_SPAN_MM, sharp)], 20.0)
        self.assertIsNone(alignment["error"])
        self.assertEqual(alignment["theta_source"], "sharp_te_band")
        self.assertIsNone(alignment["thin_te_flat"])
        self.assertAlmostEqual(alignment["theta_deg"], 20.0, delta=0.5)

    def test_a_thicker_flat_is_still_refused_and_the_message_names_the_guard(self):
        thick = _blunt_outline_mm(_THICK_TRUNC)
        alignment, cap = self._alignment([(0.0, thick), (_SPAN_MM, thick)], 20.0)
        self.assertIsNone(alignment["thin_te_flat"])
        self.assertIsNotNone(alignment["error"])
        self.assertIn("A thin-trailing-edge-flat reading", alignment["error"])
        self.assertIn("face_over_chord", alignment["error"])
        self.assertIn("will NOT help", alignment["error"])
        fatal = getattr(cap, "_alignment_fatal_error", None)
        self.assertTrue(fatal)
        self.assertIn("squared-off trailing edge, not a rotation problem", fatal)
        self.assertEqual(alignment["theta_deg"], 0.0)
        self.assertFalse(alignment["rotation_applied"])


if __name__ == "__main__":
    unittest.main()
