"""Tests for the geometry-envelope fixtures (taper, sweep, twist, tip) in
tools/make_fixtures.py.

Loaded via importlib, matching test_airfoil_geometry.py's pattern for
exercising a tools/ script as a module without adding it to the package
layout. All fixture-writing calls in this file pass a temporary directory as
repo_root -- never the real tests/fixtures/ tree -- both because the
determinism checks need two independent generations anyway, and because
writing to the real fixtures tree from a test would race with concurrent fixture
generation.
"""
import importlib.util
import math
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location("make_fixtures_under_test_m5", MAKE_FIXTURES_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mf = _load_make_fixtures_module()

NACA = "2412"
CHORD_MM = 60.0
SPAN_MM = 60.0
N_POINTS = 40  # smaller than the fixtures' own 120 -- fast tests, same formulas
N_STATIONS_TWIST = 21


def _tmp_repo_root(tmpdir_str):
    return Path(tmpdir_str)


class TaperFixtureTests(unittest.TestCase):
    def test_watertight(self):
        with tempfile.TemporaryDirectory() as d:
            stl_path, _readme = mf.make_taper(_tmp_repo_root(d), NACA, 60.0, 30.0, SPAN_MM, N_POINTS)
            _header, triangles = mf.parse_binary_stl(stl_path)
            ok, details, problems = mf.check_watertight(triangles)
            self.assertTrue(ok, f"{details}; problems: {problems}")

    def test_triangle_count_formula(self):
        with tempfile.TemporaryDirectory() as d:
            stl_path, _readme = mf.make_taper(_tmp_repo_root(d), NACA, 60.0, 30.0, SPAN_MM, N_POINTS)
            _header, triangles = mf.parse_binary_stl(stl_path)
            n_outline = 2 * N_POINTS - 2
            # 2 stations: (S - 1) = 1 side-wall ring + 2 caps.
            expected = 2 * n_outline * 1 + 2 * (n_outline - 2)
            self.assertEqual(len(triangles), expected)

    def test_deterministic_across_two_generations(self):
        with tempfile.TemporaryDirectory() as d1, tempfile.TemporaryDirectory() as d2:
            stl_path1, _r1 = mf.make_taper(_tmp_repo_root(d1), NACA, 60.0, 30.0, SPAN_MM, N_POINTS)
            stl_path2, _r2 = mf.make_taper(_tmp_repo_root(d2), NACA, 60.0, 30.0, SPAN_MM, N_POINTS)
            self.assertEqual(stl_path1.read_bytes(), stl_path2.read_bytes())

    def test_root_and_tip_chord_match_spec(self):
        m, p, t = mf.parse_naca4(NACA)
        root_chord_mm, tip_chord_mm = 60.0, 30.0

        def params(z):
            frac = z / SPAN_MM
            chord = root_chord_mm + (tip_chord_mm - root_chord_mm) * frac
            return chord, 0.0, 0.0

        stations, _outline_unit, _root_shift = mf.build_loft_stations(
            m, p, t, N_POINTS, (0.0, SPAN_MM), params, pivot_frac_x=0.0
        )
        for (z, outline), expected_chord in ((stations[0], root_chord_mm), (stations[1], tip_chord_mm)):
            xmin, _ymin, xmax, _ymax = mf.section_bbox_2d(outline)
            self.assertAlmostEqual(xmax - xmin, expected_chord, places=3, msg=f"chord at z={z}")


class SweepFixtureTests(unittest.TestCase):
    def test_watertight(self):
        with tempfile.TemporaryDirectory() as d:
            stl_path, _readme = mf.make_sweep(_tmp_repo_root(d), NACA, CHORD_MM, SPAN_MM, 20.0, N_POINTS)
            _header, triangles = mf.parse_binary_stl(stl_path)
            ok, details, problems = mf.check_watertight(triangles)
            self.assertTrue(ok, f"{details}; problems: {problems}")

    def test_triangle_count_formula(self):
        with tempfile.TemporaryDirectory() as d:
            stl_path, _readme = mf.make_sweep(_tmp_repo_root(d), NACA, CHORD_MM, SPAN_MM, 20.0, N_POINTS)
            _header, triangles = mf.parse_binary_stl(stl_path)
            n_outline = 2 * N_POINTS - 2
            expected = 2 * n_outline * 1 + 2 * (n_outline - 2)
            self.assertEqual(len(triangles), expected)

    def test_deterministic_across_two_generations(self):
        with tempfile.TemporaryDirectory() as d1, tempfile.TemporaryDirectory() as d2:
            stl_path1, _r1 = mf.make_sweep(_tmp_repo_root(d1), NACA, CHORD_MM, SPAN_MM, 20.0, N_POINTS)
            stl_path2, _r2 = mf.make_sweep(_tmp_repo_root(d2), NACA, CHORD_MM, SPAN_MM, 20.0, N_POINTS)
            self.assertEqual(stl_path1.read_bytes(), stl_path2.read_bytes())

    def test_root_and_tip_le_x_offset_match_spec(self):
        m, p, t = mf.parse_naca4(NACA)
        sweep_mm = 20.0

        def params(z):
            frac = z / SPAN_MM
            return CHORD_MM, 0.0, sweep_mm * frac

        stations, _outline_unit, _root_shift = mf.build_loft_stations(
            m, p, t, N_POINTS, (0.0, SPAN_MM), params, pivot_frac_x=0.0
        )
        le_root_x = stations[0][1][0][0]
        le_tip_x = stations[1][1][0][0]
        self.assertAlmostEqual(le_tip_x - le_root_x, sweep_mm, places=6)


class TwistFixtureTests(unittest.TestCase):
    def test_watertight(self):
        with tempfile.TemporaryDirectory() as d:
            stl_path, _readme = mf.make_twist(
                _tmp_repo_root(d), NACA, CHORD_MM, SPAN_MM, 6.0, N_POINTS, N_STATIONS_TWIST
            )
            _header, triangles = mf.parse_binary_stl(stl_path)
            ok, details, problems = mf.check_watertight(triangles)
            self.assertTrue(ok, f"{details}; problems: {problems}")

    def test_triangle_count_formula(self):
        with tempfile.TemporaryDirectory() as d:
            stl_path, _readme = mf.make_twist(
                _tmp_repo_root(d), NACA, CHORD_MM, SPAN_MM, 6.0, N_POINTS, N_STATIONS_TWIST
            )
            _header, triangles = mf.parse_binary_stl(stl_path)
            n_outline = 2 * N_POINTS - 2
            expected = 2 * n_outline * (N_STATIONS_TWIST - 1) + 2 * (n_outline - 2)
            self.assertEqual(len(triangles), expected)

    def test_deterministic_across_two_generations(self):
        with tempfile.TemporaryDirectory() as d1, tempfile.TemporaryDirectory() as d2:
            stl_path1, _r1 = mf.make_twist(
                _tmp_repo_root(d1), NACA, CHORD_MM, SPAN_MM, 6.0, N_POINTS, N_STATIONS_TWIST
            )
            stl_path2, _r2 = mf.make_twist(
                _tmp_repo_root(d2), NACA, CHORD_MM, SPAN_MM, 6.0, N_POINTS, N_STATIONS_TWIST
            )
            self.assertEqual(stl_path1.read_bytes(), stl_path2.read_bytes())

    def test_root_and_tip_rotation_match_spec(self):
        m, p, t = mf.parse_naca4(NACA)
        washout_deg = 6.0

        def params(z):
            frac = z / SPAN_MM
            return CHORD_MM, washout_deg * frac, 0.0

        z_stations = [SPAN_MM * i / (N_STATIONS_TWIST - 1) for i in range(N_STATIONS_TWIST)]
        stations, _outline_unit, _root_shift = mf.build_loft_stations(
            m, p, t, N_POINTS, z_stations, params, pivot_frac_x=0.25
        )

        def chord_line_angle_deg(outline_mm):
            le = outline_mm[0]
            te = outline_mm[N_POINTS - 1]
            return math.degrees(math.atan2(te[1] - le[1], te[0] - le[0]))

        angle_root = chord_line_angle_deg(stations[0][1])
        angle_tip = chord_line_angle_deg(stations[-1][1])
        self.assertAlmostEqual(angle_root, 0.0, places=6)
        self.assertAlmostEqual(angle_tip, washout_deg, places=6)


class TipFixtureTests(unittest.TestCase):
    def test_watertight(self):
        with tempfile.TemporaryDirectory() as d:
            stl_path, _readme, _sc, _sz = mf.make_tip(
                _tmp_repo_root(d), NACA, 60.0, 3.0, SPAN_MM, N_POINTS
            )
            _header, triangles = mf.parse_binary_stl(stl_path)
            ok, details, problems = mf.check_watertight(triangles)
            self.assertTrue(ok, f"{details}; problems: {problems}")

    def test_triangle_count_formula(self):
        with tempfile.TemporaryDirectory() as d:
            stl_path, _readme, _sc, _sz = mf.make_tip(
                _tmp_repo_root(d), NACA, 60.0, 3.0, SPAN_MM, N_POINTS
            )
            _header, triangles = mf.parse_binary_stl(stl_path)
            n_outline = 2 * N_POINTS - 2
            expected = 2 * n_outline * 1 + 2 * (n_outline - 2)
            self.assertEqual(len(triangles), expected)

    def test_deterministic_across_two_generations(self):
        with tempfile.TemporaryDirectory() as d1, tempfile.TemporaryDirectory() as d2:
            stl_path1, _r1, _sc1, _sz1 = mf.make_tip(_tmp_repo_root(d1), NACA, 60.0, 3.0, SPAN_MM, N_POINTS)
            stl_path2, _r2, _sc2, _sz2 = mf.make_tip(_tmp_repo_root(d2), NACA, 60.0, 3.0, SPAN_MM, N_POINTS)
            self.assertEqual(stl_path1.read_bytes(), stl_path2.read_bytes())

    def test_local_thickness_decreases_monotonically_toward_tip(self):
        m, p, t = mf.parse_naca4(NACA)
        root_chord_mm, tip_chord_mm = 60.0, 3.0
        z_values = [i * 6.0 for i in range(11)]  # 0, 6, 12, ..., 60
        thicknesses = [
            mf.local_max_thickness_mm(
                m, p, t, N_POINTS, root_chord_mm + (tip_chord_mm - root_chord_mm) * (z / SPAN_MM)
            )
            for z in z_values
        ]
        for i in range(1, len(thicknesses)):
            self.assertLess(
                thicknesses[i], thicknesses[i - 1],
                f"local thickness did not decrease from z={z_values[i-1]} to z={z_values[i]}",
            )

    def test_local_thickness_falls_below_skin_margin_before_the_tip(self):
        m, p, t = mf.parse_naca4(NACA)
        root_chord_mm, tip_chord_mm = 60.0, 3.0
        skin_margin_mm = 1.0

        thickness_at_root = mf.local_max_thickness_mm(m, p, t, N_POINTS, root_chord_mm)
        self.assertGreater(thickness_at_root, skin_margin_mm)

        thickness_at_tip = mf.local_max_thickness_mm(m, p, t, N_POINTS, tip_chord_mm)
        self.assertLess(thickness_at_tip, skin_margin_mm)

        # Find the first sampled z at which thickness drops below the margin;
        # it must be strictly before the physical tip (z < SPAN_MM).
        crossing_z = None
        for i in range(1, 601):
            z = SPAN_MM * i / 600.0
            chord = root_chord_mm + (tip_chord_mm - root_chord_mm) * (z / SPAN_MM)
            thickness = mf.local_max_thickness_mm(m, p, t, N_POINTS, chord)
            if thickness < skin_margin_mm:
                crossing_z = z
                break
        self.assertIsNotNone(crossing_z, "thickness never dropped below the skin margin")
        self.assertLess(crossing_z, SPAN_MM)


FIXTURES_DIR = REPO_ROOT / "tests" / "fixtures"


def _single_shipped_stl(subdir):
    matches = list((FIXTURES_DIR / subdir).glob("*.stl"))
    if len(matches) != 1:
        raise AssertionError(f"expected exactly one .stl in {subdir}, found {matches}")
    return matches[0]


def _unique_vertices_at_z(triangles, z, tol=1e-4):
    seen = set()
    for _normal, v0, v1, v2 in triangles:
        for v in (v0, v1, v2):
            if abs(v[2] - z) < tol:
                seen.add((v[0], v[1]))
    return seen


def _farthest_pair(points):
    points = list(points)
    best = (0.0, None, None)
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            ax, ay = points[i]
            bx, by = points[j]
            d = math.hypot(bx - ax, by - ay)
            if d > best[0]:
                best = (d, points[i], points[j])
    return best[1], best[2]


class ShippedFixtureTests(unittest.TestCase):
    """Read-only checks against the STL files actually committed to
    tests/fixtures/, independent of tools/make_fixtures.py's internal params_fn
    closures -- so a sign or formula regression in the generator that a
    unit test re-deriving the same formula would miss (because it would
    carry the same bug) is still caught here, against the shipped artifact.
    Read-only: never writes into tests/fixtures/, so it does not race with
    concurrent fixture generation elsewhere in the repo.
    """

    def test_taper_watertight_and_triangle_count(self):
        stl_path = _single_shipped_stl("taper")
        _header, triangles = mf.parse_binary_stl(stl_path)
        ok, details, problems = mf.check_watertight(triangles)
        self.assertTrue(ok, f"{details}; problems: {problems}")
        self.assertEqual(len(triangles), 948)

    def test_taper_root_and_tip_chord(self):
        stl_path = _single_shipped_stl("taper")
        _header, triangles = mf.parse_binary_stl(stl_path)
        zs = sorted({v[2] for _n, v0, v1, v2 in triangles for v in (v0, v1, v2)})
        z_root, z_tip = zs[0], zs[-1]
        root_pts = _unique_vertices_at_z(triangles, z_root)
        tip_pts = _unique_vertices_at_z(triangles, z_tip)
        root_x = [x for x, _y in root_pts]
        tip_x = [x for x, _y in tip_pts]
        self.assertAlmostEqual(max(root_x) - min(root_x), 60.0, places=1)
        self.assertAlmostEqual(max(tip_x) - min(tip_x), 30.0, places=1)

    def test_sweep_watertight_and_triangle_count(self):
        stl_path = _single_shipped_stl("sweep")
        _header, triangles = mf.parse_binary_stl(stl_path)
        ok, details, problems = mf.check_watertight(triangles)
        self.assertTrue(ok, f"{details}; problems: {problems}")
        self.assertEqual(len(triangles), 948)

    def test_sweep_root_and_tip_le_offset(self):
        stl_path = _single_shipped_stl("sweep")
        _header, triangles = mf.parse_binary_stl(stl_path)
        zs = sorted({v[2] for _n, v0, v1, v2 in triangles for v in (v0, v1, v2)})
        z_root, z_tip = zs[0], zs[-1]
        root_pts = _unique_vertices_at_z(triangles, z_root)
        tip_pts = _unique_vertices_at_z(triangles, z_tip)
        # Leading edge = leftmost point in each z-plane. Under a pure
        # translation (sweep is a rigid X shift, no rotation or scale), the
        # leftmost point maps to the leftmost point, so this is exact
        # regardless of which outline vertex happens to be leftmost.
        min_x_root = min(x for x, _y in root_pts)
        min_x_tip = min(x for x, _y in tip_pts)
        self.assertAlmostEqual(min_x_tip - min_x_root, 20.0, places=1)

    def test_twist_watertight_and_triangle_count(self):
        stl_path = _single_shipped_stl("twist")
        _header, triangles = mf.parse_binary_stl(stl_path)
        ok, details, problems = mf.check_watertight(triangles)
        self.assertTrue(ok, f"{details}; problems: {problems}")
        self.assertEqual(len(triangles), 29032)

    def test_twist_root_and_tip_rotation(self):
        # The farthest-apart-pair proxy for the chord line has a small
        # systematic bias (~0.13 degrees here): the true leading-edge
        # vertex is not quite the outline's own X-minimum (a neighbouring
        # vertex, a fraction of a mm further forward due to the leading-
        # edge curvature, edges it out -- see build_loft_stations/
        # naca4_surface_points; this is real airfoil geometry, not a mesh
        # bug). That bias is close to constant under the rigid rotation
        # applied here, so it cancels in the root-to-tip delta almost
        # exactly while still biasing each absolute angle individually.
        # Assert the delta tightly (the number the spec actually pins down)
        # and each absolute angle loosely (sanity: not off by some
        # multiple of the intended washout).
        stl_path = _single_shipped_stl("twist")
        _header, triangles = mf.parse_binary_stl(stl_path)
        zs = sorted({v[2] for _n, v0, v1, v2 in triangles for v in (v0, v1, v2)})
        z_root, z_tip = zs[0], zs[-1]

        def chord_line_angle_deg(z):
            pts = _unique_vertices_at_z(triangles, z)
            a, b = _farthest_pair(pts)
            # The farthest-apart pair in a thin airfoil section is its
            # leading-to-trailing-edge diameter; orient LE->TE by X.
            le, te = (a, b) if a[0] < b[0] else (b, a)
            return math.degrees(math.atan2(te[1] - le[1], te[0] - le[0]))

        angle_root = chord_line_angle_deg(z_root)
        angle_tip = chord_line_angle_deg(z_tip)
        self.assertAlmostEqual(angle_tip - angle_root, 6.0, places=2)
        self.assertLess(abs(angle_root), 0.5)
        self.assertLess(abs(angle_tip - 6.0), 0.5)

    def test_tip_watertight_and_triangle_count(self):
        stl_path = _single_shipped_stl("tip")
        _header, triangles = mf.parse_binary_stl(stl_path)
        ok, details, problems = mf.check_watertight(triangles)
        self.assertTrue(ok, f"{details}; problems: {problems}")
        self.assertEqual(len(triangles), 948)

    def test_tip_root_and_tip_chord(self):
        stl_path = _single_shipped_stl("tip")
        _header, triangles = mf.parse_binary_stl(stl_path)
        zs = sorted({v[2] for _n, v0, v1, v2 in triangles for v in (v0, v1, v2)})
        z_root, z_tip = zs[0], zs[-1]
        root_pts = _unique_vertices_at_z(triangles, z_root)
        tip_pts = _unique_vertices_at_z(triangles, z_tip)
        root_x = [x for x, _y in root_pts]
        tip_x = [x for x, _y in tip_pts]
        self.assertAlmostEqual(max(root_x) - min(root_x), 60.0, places=1)
        self.assertAlmostEqual(max(tip_x) - min(tip_x), 3.0, places=1)


if __name__ == "__main__":
    unittest.main()
