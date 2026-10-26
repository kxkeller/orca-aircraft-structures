"""Capability-seam test: drive ``capability._build_mesh_sectioner`` (NOT the full
``execute()`` pipeline -- see below) through ``fake_orca`` with a
blunt-mid/sharp-tip mesh at an injected rotation, and assert the returned
alignment report actually took the sharp-TE-band substitution path.

WHY ``_build_mesh_sectioner`` DIRECTLY, NOT ``cap.execute(ctx)``. This
project's own ``RotationDetectedAndAppliedTests`` (``test_m34_global_
alignment.py``) is entirely ``@unittest.skip``ped for exactly this reason:
fake_orca's ``ExPolygon`` raw-points constructor (``_classify_rect``)
accepts ONLY a 4-point axis-aligned rectangle. Once a non-zero, non-90-
degree theta is detected, the real per-layer mutation loop calls
``_rotate_expolygon``, which reconstructs the rotated HOST SLICE polygon via
``ops.polygon_from_points_and_holes`` -> ``orca.host.ExPolygon(contour, [])``
-- and a rotated N>4-point aerofoil polygon (or even a rotated 4-point
rectangle at a non-multiple-of-90 angle) is not representable by this fake's
rectangle-only geometry kernel, raising ``NotImplementedError`` before
``execute()`` can reach ``PluginResult.Success``. That skip's own docstring
already establishes that the MESH-SECTIONER-LEVEL theta detection (before
any host-slice polygon reconstruction) is independently exercised and does
work; this test is that same seam, extended to the sharp-band path.
``_build_mesh_sectioner`` populates ``self._mesh_alignment`` (BEFORE the
per-layer mutation loop even starts, see ``execute()``) and touches no host
Surface/ExPolygon at all -- reading ``ctx.object.model_object()`` and
producing a plain dict -- so it is reachable through fake_orca regardless of
this limitation.

THE MESH FIXTURE. ``loft_mesh_mm`` (``tools/make_fixtures.loft_triangles``)
builds side-wall triangles as a per-edge quad between each pair of
CONSECUTIVE stations, and requires every ring to have no adjacent-duplicate
vertex (a repeated point makes two of a wall quad's three triangle vertices
identical, a zero-area triangle ``tri_normal`` rejects outright). Neither
``tests/unit/test_alignment.py``'s own synthetic NACA-like ring builders
(which duplicate BOTH the leading edge and, for a closed/sharp trailing
edge, the trailing edge, as two adjacent identical points -- fine for a
convex-hull-only geometry test, fatal for a real mesh loft) nor a naive
"open the TE by epsilon" tweak work here: measured directly (this test's own
development), ANY non-zero separation between the upper/lower trailing-edge
points reads a ~90-98 degree sharpest-hull-vertex angle regardless of how
small the separation is (the interior-angle discontinuity IS the mechanism
``TE_ANGLE_DEGENERATE_THRESHOLD_DEG``'s own docstring documents: a closed
single point measures the true converging cusp angle, a split TE of ANY gap
measures the angle against the intervening cut edge instead) -- so there is
no continuous knob from "sharp" to "blunt" via gap size, only a genuine
merged-vs-open topology choice. This module therefore builds its own
mesh-safe outlines directly from ``make_fixtures.naca4_thickness``/
``naca4_camber`` (the project's real formula, no second implementation),
assembled the same single-occurrence way ``naca4_section_outline`` uses (one
leading-edge point, trailing edge merged to a single point for "sharp", left
as two distinct points for "blunt/truncated"), with vertex counts matched
between the two variants (one interior upper point is dropped from the
blunt ring to compensate for its extra, explicit trailing-edge corner) so
``loft_mesh_mm`` accepts both as loft stations of one mesh.
"""

import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import fake_orca  # noqa: E402
from aircraft_core.alignment import rotate_points_deg  # noqa: E402

_MF = fake_orca._load_make_fixtures_module()
_M, _P, _T = _MF.parse_naca4("2412")
_N_POINTS = 80

# Span layout: blunt (truncated, elevon-like) from 0% to 85% of span, sharp
# from 86% to 100% -- deliberately abrupt (two adjacent, IDENTICAL stations
# on each side of the transition) so every sampled fraction in the
# 0.02..0.98 sweep reads either the pure blunt or the pure sharp shape,
# never a blend -- mirroring the abrupt-band synthetic fixtures in
# ``tests/unit/test_alignment.py``.
_SPAN_MM = 100.0
_BLUNT_TO_Z_MM = 85.0
_SHARP_FROM_Z_MM = 86.0
_CHORD_MM = 60.0
_TRUNCATE_FRAC = 0.96


def _naca_surface_points(truncate_frac, n_points=_N_POINTS):
    """Upper/lower surface points (chord-fraction coords), leading edge
    forced to exactly (0, 0). Reuses ``make_fixtures``'s own thickness/
    camber formulas (no second NACA implementation) but does NOT force the
    trailing edge closed the way ``naca4_surface_points`` does -- that
    forcing is exactly what this module's own outline assembly needs to
    control per-variant (see module docstring)."""
    upper, lower = [], []
    for i in range(n_points):
        x = truncate_frac * i / (n_points - 1)
        yt = _MF.naca4_thickness(x, _T)
        yc, dyc = _MF.naca4_camber(x, _M, _P)
        theta = math.atan(dyc)
        upper.append((x - yt * math.sin(theta), yc + yt * math.cos(theta)))
        lower.append((x + yt * math.sin(theta), yc - yt * math.cos(theta)))
    upper[0] = (0.0, 0.0)
    lower[0] = (0.0, 0.0)
    return upper, lower


def _sharp_outline_mm(chord_mm=_CHORD_MM, n_points=_N_POINTS):
    """A genuinely sharp (closed, single-point) trailing edge, matching
    ``make_fixtures.naca4_section_outline``'s own single-occurrence
    convention: one leading-edge point, one merged trailing-edge point,
    NO adjacent duplicate anywhere in the ring."""
    upper, lower = _naca_surface_points(1.0, n_points)
    te = (1.0, _MF.naca4_camber(1.0, _M, _P)[0])
    upper[-1] = te
    lower[-1] = te
    outline = [upper[0]]
    outline.extend(lower[1:-1])
    outline.append(lower[-1])
    outline.extend(reversed(upper[1:-1]))
    return [(x * chord_mm, y * chord_mm) for x, y in outline]


def _blunt_outline_mm(truncate_frac=_TRUNCATE_FRAC, chord_mm=_CHORD_MM, n_points=_N_POINTS):
    """A genuinely blunt (open, two-corner) trailing edge, cut short at
    ``truncate_frac`` of chord -- the elevon-cutout stand-in. Same vertex
    COUNT as ``_sharp_outline_mm`` (one interior upper point is dropped to
    compensate for the extra, explicit trailing-edge corner this variant
    carries), which ``loft_mesh_mm`` requires of every station."""
    upper, lower = _naca_surface_points(truncate_frac, n_points)
    outline = [upper[0]]
    outline.extend(lower[1:])            # includes the open lower-TE corner
    outline.append(upper[-1])            # the open upper-TE corner
    outline.extend(reversed(upper[1:-2]))  # drop one interior point to match count
    return [(x * chord_mm, y * chord_mm) for x, y in outline]


class CapabilitySharpBandSeamTests(unittest.TestCase):
    """Prove the sharp-TE-band substitution (``resolve_wing_theta_deg``'s
    ``station_sampler`` path) is actually WIRED at the ``capability.py`` seam,
    not just correct in the pure ``aircraft_core.alignment`` unit tests."""

    def setUp(self):
        self._fake_orca, restore = fake_orca.install()
        self.addCleanup(restore)
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.entry_file = os.path.join(self._tmp.name, "aircraft_structures.py")
        self.namespace = fake_orca.exec_bundle(self.entry_file)

    def _build_elevon_like_object(self, theta_inj_deg):
        blunt_mm = rotate_points_deg(_blunt_outline_mm(), theta_inj_deg)
        sharp_mm = rotate_points_deg(_sharp_outline_mm(), theta_inj_deg)
        stations = [
            (0.0, blunt_mm),
            (_BLUNT_TO_Z_MM, blunt_mm),
            (_SHARP_FROM_Z_MM, sharp_mm),
            (_SPAN_MM, sharp_mm),
        ]
        mesh = fake_orca.loft_mesh_mm(stations)
        return fake_orca.PrintObject(
            "elevon_like", [], model_object=fake_orca.identity_model_object(mesh)
        )

    def _sectioner_bounds(self, theta_inj_deg):
        obj = self._build_elevon_like_object(theta_inj_deg)
        Step = self._fake_orca.slicing.Step
        ctx = self._fake_orca.slicing.SlicingPipelineContext(
            step=Step.posSlice, obj=obj, config={}, cancelled_fn=None,
        )
        cap = self.namespace["AircraftStructuresM1"]()
        result = cap._build_mesh_sectioner(ctx)
        self.assertIsNotNone(
            result,
            "cap._build_mesh_sectioner returned None; "
            "_host_object_transform={!r}".format(cap._host_object_transform),
        )
        _sectioner, _z_off, bounds = result
        return bounds["alignment"]

    def test_blunt_mid_span_fixture_really_is_blunt_before_the_seam_test(self):
        # Sanity check on the fixture itself, independent of capability.py:
        # mid-span (the 0/85 blunt shape) must raise BluntTrailingEdgeError
        # and the tip (sharp shape) must resolve, or this whole fixture
        # proves nothing about the seam being tested.
        from aircraft_core.alignment import (
            BluntTrailingEdgeError, resolve_theta_deg,
        )
        with self.assertRaises(BluntTrailingEdgeError):
            resolve_theta_deg(_blunt_outline_mm())
        resolve_theta_deg(_sharp_outline_mm())  # must not raise

    def test_theta_source_is_sharp_te_band_and_theta_matches_injection(self):
        for theta_inj in (20.0, -15.0):
            with self.subTest(theta_inj=theta_inj):
                alignment = self._sectioner_bounds(theta_inj)
                self.assertIsNone(alignment["error"])
                self.assertTrue(alignment["rotation_applied"])
                self.assertEqual(alignment["theta_source"], "sharp_te_band")
                self.assertAlmostEqual(alignment["theta_deg"], theta_inj, delta=0.5)
                self.assertGreaterEqual(len(alignment["sharp_band_fracs"]), 5)
                self.assertLess(alignment["sharp_band_spread_deg"], 1.0)
                # On the band path, an excluded OTHER station's blunt-TE
                # reason must carry the shortened annotation, never the
                # "The slice is refused..." sentence from the raw
                # BluntTrailingEdgeError message (the run in fact succeeded via
                # the band, so that sentence would misreport it). The
                # 0.1/0.3/0.7 stations are blunt in this fixture and are
                # expected to be excluded.
                excluded = alignment["excluded_stations"]
                self.assertGreater(len(excluded), 0)
                for entry in excluded:
                    self.assertNotIn("The slice is refused", entry["reason"])
                    self.assertIn("sharp-TE band", entry["reason"])


if __name__ == "__main__":
    unittest.main()
