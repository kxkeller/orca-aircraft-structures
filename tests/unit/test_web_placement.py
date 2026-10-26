"""Tests for aircraft_core/web_placement.py -- the geometry-aware primary web
placement policy.

Fixtures follow test_generator.py's own pattern: the NACA
2412 c=60mm s=60mm section built directly from tools/make_fixtures.py's point
generation (no STL round trip), plus a lofted taper built the same way
make_taper.py itself builds its STL (tools.make_fixtures.build_loft_stations),
so the taper-stability check exercises the SAME geometry the shipped taper
fixture is made from without needing to read an STL file.
"""

import importlib.util
import unittest
from pathlib import Path

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext
from aircraft_core.spec import ROLE_PRIMARY_STRUCTURE, StructureSpec
from aircraft_core.web_placement import (
    AftWebPolicy,
    MainWebPolicy,
    PlacementError,
    deepest_station_frac,
    select_primary_webs,
)
from stub_ops import PolygonShape, Rect, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_web_placement", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_mf = _load_make_fixtures_module()


def _naca2412_polygon():
    m, p, t = _mf.parse_naca4("2412")
    _triangles, outline_mm, _bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm))


# own shipped skin margin -- the tightest margin this project's no-sever proof
# tolerates (2 * safety_offset_mm + 0.021 mm cushion at the 0.42 mm outer wall
# used throughout this test module).
SKIN_MARGIN_MM = 0.8608
OUTER_WALL_MM = 0.42


class MainWebSelectionTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.frame = LayerFrame.from_polygon(self.ops, _naca2412_polygon())
        self.frames = [(LayerContext(layer_index=0, print_z=0.0), self.frame)]
        self.spec = StructureSpec(gap_ratio=0.05, skin_margin_mm=SKIN_MARGIN_MM)

    def test_selects_naca2412_thickest_point_near_30_percent(self):
        # NACA 2412's own tabulated max-thickness station is ~29.8% chord --
        # this is a geometry fact about the airfoil, not a project constant.
        report = select_primary_webs(
            self.spec, self.frames, OUTER_WALL_MM, main_policy=MainWebPolicy()
        )
        self.assertAlmostEqual(report.main.station_frac, 0.30, places=6)
        self.assertGreater(report.main.local_thickness_mm, 7.0)
        self.assertEqual(report.main.family.role, ROLE_PRIMARY_STRUCTURE)
        self.assertIsNone(report.main.family.rib_pitch_mm)
        self.assertIsNone(report.main.family.z_drift_mm_per_mm)
        self.assertIsNone(report.main.family.z_drift_chord_frac_per_mm)

    def test_local_thickness_beats_the_hard_coded_25_percent_station(self):
        # The shipped default's spar_front sits at 0.25c. The policy's own
        # chosen 0.30c station must be at least as thick there, or the
        # "geometry-aware" claim is empty.
        report = select_primary_webs(
            self.spec, self.frames, OUTER_WALL_MM, main_policy=MainWebPolicy()
        )
        width_mm = report.main.family.resolve_gap(self.spec, OUTER_WALL_MM)
        from aircraft_core.frame import local_lower_worst_case, local_upper_worst_case

        x25 = self.frame.web_x(0.25)
        thickness_25 = local_upper_worst_case(
            self.ops, self.frame.polygon, x25, width_mm
        ) - local_lower_worst_case(self.ops, self.frame.polygon, x25, width_mm)
        self.assertGreaterEqual(report.main.local_thickness_mm, thickness_25)

    def test_no_qualifying_station_fails_closed(self):
        with self.assertRaises(PlacementError) as ctx:
            select_primary_webs(
                self.spec,
                self.frames,
                OUTER_WALL_MM,
                main_policy=MainWebPolicy(chord_window=(0.90, 0.98)),
            )
        message = str(ctx.exception)
        self.assertIn("no chord station", message)
        self.assertIn("0.9000", message)
        self.assertIn("failed", message)

    def test_malformed_window_raises_plain_value_error(self):
        with self.assertRaises(ValueError):
            select_primary_webs(
                self.spec,
                self.frames,
                OUTER_WALL_MM,
                main_policy=MainWebPolicy(chord_window=(0.5, 0.1)),
            )

    def test_empty_frames_raises(self):
        with self.assertRaises(ValueError):
            select_primary_webs(self.spec, [], OUTER_WALL_MM)


class AftWebSelectionTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.frame = LayerFrame.from_polygon(self.ops, _naca2412_polygon())
        self.frames = [(LayerContext(layer_index=0, print_z=0.0), self.frame)]
        self.spec = StructureSpec(gap_ratio=0.05, skin_margin_mm=SKIN_MARGIN_MM)

    def test_aft_web_is_none_when_not_requested(self):
        report = select_primary_webs(
            self.spec, self.frames, OUTER_WALL_MM, main_policy=MainWebPolicy()
        )
        self.assertIsNone(report.aft)

    def test_aft_web_selects_a_station_within_the_manufacturable_band(self):
        # own record: a full-depth spar at skin_margin_mm=0.8608 survives only
        # ~16-46% chord on this fixture. The conventional 55-70% rear-spar band
        # is expected to be EMPTY here -- the aft policy must still find
        # *something* just past the main web's separation floor, not fail, and
        # it must not exceed ~0.46-0.47.
        report = select_primary_webs(
            self.spec,
            self.frames,
            OUTER_WALL_MM,
            main_policy=MainWebPolicy(),
            aft_policy=AftWebPolicy(),
        )
        self.assertIsNotNone(report.aft)
        self.assertGreater(report.aft.station_frac, report.main.station_frac)
        self.assertLess(report.aft.station_frac, 0.50)
        self.assertEqual(report.aft.family.name, "aft")

    def test_conventional_55_to_70_percent_band_does_not_qualify_here(self):
        # The conventional 55-70% guidance band is infeasible on this fixture at the shipped skin
        # margin -- restricting the aft search to exactly that band must fail
        # closed, not silently return something else.
        with self.assertRaises(PlacementError):
            select_primary_webs(
                self.spec,
                self.frames,
                OUTER_WALL_MM,
                main_policy=MainWebPolicy(),
                aft_policy=AftWebPolicy(search_window=(0.55, 0.70)),
            )

    def test_aft_web_respects_minimum_separation_from_main(self):
        report = select_primary_webs(
            self.spec,
            self.frames,
            OUTER_WALL_MM,
            main_policy=MainWebPolicy(),
            aft_policy=AftWebPolicy(min_separation_frac=0.1),
        )
        self.assertGreaterEqual(
            report.aft.station_frac - report.main.station_frac, 0.1 - 1e-9
        )


class TaperStabilityTests(unittest.TestCase):
    """own drift concern, answered directly: does the chosen main station wander
    along a tapering span? Builds the SAME lofted geometry
    tools/make_fixtures.py's make_taper() builds into an STL, without needing
    the STL round trip."""

    def setUp(self):
        self.ops = StubOps()
        self.m, self.p, self.t = _mf.parse_naca4("2412")
        self.root_chord_mm = 60.0
        self.tip_chord_mm = 30.0
        self.span_mm = 60.0

    def _frames(self, n_stations=13):
        step = self.span_mm / (n_stations - 1)
        z_stations = [i * step for i in range(n_stations)]

        def params(z):
            frac = z / self.span_mm
            chord = self.root_chord_mm + (self.tip_chord_mm - self.root_chord_mm) * frac
            return chord, 0.0, 0.0

        stations, _outline_unit, _root_shift = _mf.build_loft_stations(
            self.m, self.p, self.t, 120, z_stations, params, pivot_frac_x=0.0
        )
        frames = []
        for z, outline_mm in stations:
            frame = LayerFrame.from_polygon(self.ops, PolygonShape(tuple(outline_mm)))
            frames.append(
                (LayerContext(layer_index=int(round(z / step)), print_z=z), frame)
            )
        return frames

    def test_root_frame_selection_matches_full_stride(self):
        frames = self._frames()
        spec = StructureSpec(gap_ratio=0.05, skin_margin_mm=SKIN_MARGIN_MM)
        report = select_primary_webs(
            spec, frames, OUTER_WALL_MM, main_policy=MainWebPolicy(), layer_stride=1
        )
        self.assertAlmostEqual(report.main.station_frac, 0.30, places=6)

    def test_per_layer_argmax_does_not_wander_along_a_linear_taper(self):
        # NACA thickness-fraction is chord-scale invariant, so a linear
        # taper (constant m/p/t, only chord scaling) should not move the
        # per-layer argmax at all -- this is the measurement backing the
        # "per-part, not per-layer" design decision's stability claim.
        frames = self._frames()
        width_mm = 0.05 * OUTER_WALL_MM * 10  # arbitrary fixed probe width
        fracs = []
        for _layer, frame in frames:
            best = deepest_station_frac(frame, MainWebPolicy().chord_window, width_mm)
            self.assertIsNotNone(best)
            fracs.append(best[0])
        self.assertEqual(len(set(round(f, 6) for f in fracs)), 1)

    def test_layer_stride_still_finds_a_feasible_station_at_the_tip(self):
        frames = self._frames()
        spec = StructureSpec(gap_ratio=0.05, skin_margin_mm=SKIN_MARGIN_MM)
        # A coarse stride must still probe the LAST frame (the tip) --
        # _sample_frames always appends it regardless of stride.
        report = select_primary_webs(
            spec, frames, OUTER_WALL_MM, main_policy=MainWebPolicy(), layer_stride=5
        )
        self.assertEqual(report.main.layers_probed, len(frames[::5]) + (
            0 if frames[::5][-1] is frames[-1] else 1
        ))


class RectCouponTests(unittest.TestCase):
    """The rectangle coupon: every chord station has identical thickness, so the
    policy must still pick a definite, in-window station (ties broken by
    grid/scan order) rather than erroring on a flat objective."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)
        self.frames = [(LayerContext(layer_index=0, print_z=0.0), self.frame)]

    def test_flat_objective_still_selects_a_station_in_window(self):
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0)
        report = select_primary_webs(
            spec, self.frames, 0.42, main_policy=MainWebPolicy(chord_window=(0.15, 0.40))
        )
        self.assertGreaterEqual(report.main.station_frac, 0.15 - 1e-9)
        self.assertLessEqual(report.main.station_frac, 0.40 + 1e-9)


if __name__ == "__main__":
    unittest.main()
