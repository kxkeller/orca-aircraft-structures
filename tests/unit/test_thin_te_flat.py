"""The thin trailing-edge flat as a third chord-axis source.

Every geometric claim here goes through the PRODUCTION code in
``aircraft_core.alignment`` (``_thin_flat_station``,
``resolve_wing_theta_deg``); nothing is imported from the measurement scripts.
Section generators are local and mirror (vertical thickness offset on the NACA
camber line, uniform-x sampling, true chord axis exactly 0 deg, LE at the
origin and the camber-line end at (chord, 0)).

Test map:
  injected misalignment........ InjectedMisalignmentTests
  per-station refusals......... StationGuardTests, NonWingShapeTests
  reachability... ReachabilityTests
  wing-level guards 4a-4d...... WingGuardTests
  truth grid / threshold....... TruthGridTests, TruncationLevelTests
  baseline results unchanged... UnchangedResultsTests
"""

import json
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", ".."))

from test_alignment import (  # noqa: E402
    _blunt_ellipse_points,
    _naca_like_points,
    _truncated_te_naca_points,
)
from aircraft_core import alignment as AL  # noqa: E402
from aircraft_core.alignment import (  # noqa: E402
    AEROFOIL_MIN_HULL_VERTICES,
    BluntTrailingEdgeError,
    ChordAxisUnavailableError,
    AlignmentError,
    SHARP_BAND_MIN_STATIONS,
    resolve_theta_deg,
    resolve_wing_theta_deg,
    rotate_points_deg,
)

FIXTURE = os.path.join(
    HERE, "..", "fixtures", "thin_te_flat", "wing_custom_midspan.json")


# --- section generators ------------------------------------------------------

def _camber(xc, m, p):
    if m == 0:
        return 0.0
    if xc < p:
        return (m / (p * p)) * (2 * p * xc - xc * xc)
    return (m / ((1 - p) ** 2)) * ((1 - 2 * p) + 2 * p * xc - xc * xc)


def _half_t(xc, t, coef):
    return 5 * t * (0.2969 * math.sqrt(xc) - 0.1260 * xc - 0.3516 * xc ** 2
                    + 0.2843 * xc ** 3 + coef * xc ** 4)


def _x_for_ratio(ratio, coef):
    """Truncation x where face / max thickness == ratio (t-independent)."""
    hmax = max(_half_t(i / 2000.0, 1.0, coef) for i in range(2001))
    lo, hi = 0.5, 1.0
    f = lambda x: 2 * _half_t(x, 1.0, coef) - ratio * 2 * hmax
    if f(hi) > 0:
        return None
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if f(mid) > 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _section(m=0.02, p=0.4, t=0.12, ratio=None, tilt=0.0, n=150,
             coef=-0.1015, chord=60.0):
    """NACA-like section, open TE (coef -0.1015: face/thickness 0.0210 at full
    chord) or cut at the x where face/thickness == ``ratio``; ``tilt`` tilts
    the cut face from perpendicular. coef -0.1036 gives a cusped TE."""
    xt = 1.0 if ratio is None else _x_for_ratio(ratio, coef)
    if xt is None:
        return None
    tn = math.tan(math.radians(tilt))

    def cut_x(sign):
        if tn == 0:
            return xt
        f = lambda x: x - tn * (_camber(x, m, p) + sign * _half_t(x, t, coef)) - xt
        lo, hi = 0.05, 1.2
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if f(mid) > 0:
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)

    xu, xl = cut_x(+1), cut_x(-1)
    pts = []
    for i in range(n + 1):
        xc = xu * i / n
        pts.append((chord * xc, chord * (_camber(xc, m, p) + _half_t(xc, t, coef))))
    for i in range(n, -1, -1):
        xc = xl * i / n
        pts.append((chord * xc, chord * (_camber(xc, m, p) - _half_t(xc, t, coef))))
    return pts


THIN = _section()                       # face/thickness 0.0210 (wing_custom's value)
THICK = _section(ratio=0.11)            # face/thickness 0.11, face/chord 0.013: refused
THICK_FT = _section(ratio=0.04)         # face/thickness 0.04 (accuracy gate), face/chord 0.005
SHARP = _naca_like_points()             # genuine cusp (existing test generator)


def _load_fixture():
    with open(FIXTURE, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _angle_err(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _stations(sampler):
    return sampler(0.5), [
        s for s in (sampler(f) for f in (0.1, 0.3, 0.7, 0.9)) if s is not None]


def _constant(section, theta=0.0):
    pts = rotate_points_deg(section, theta)
    return lambda frac: list(pts) if 0.02 <= frac <= 0.98 else None


def _split(first, rest, split, theta=0.0, extra=None, flip_frac=None):
    """Sampler: ``first`` section for frac <= split, ``rest`` after."""
    def sampler(frac):
        if frac < 0.02 or frac > 0.98:
            return None
        add = extra(frac) if extra else 0.0
        flip = 180.0 if (flip_frac is not None and abs(frac - flip_frac) < 1e-9) else 0.0
        base = first if frac <= split else rest
        return rotate_points_deg(base, theta + add + flip)
    return sampler


# --- injected misalignment ---------------------------------------------------

class InjectedMisalignmentTests(unittest.TestCase):
    ANGLES = (0.0, 5.0, 15.0, 29.0, 45.0, 90.0, 180.0)

    def _check(self, section, tol):
        for ang in self.ANGLES:
            with self.subTest(angle=ang):
                sampler = _constant(section, ang)
                mid, others = _stations(sampler)
                prov = {}
                theta, thetas, spread, excluded = resolve_wing_theta_deg(
                    mid, others, station_sampler=sampler, provenance=prov)
                self.assertEqual(prov["theta_source"], "thin_te_flat")
                self.assertLessEqual(_angle_err(theta, ang), tol)
                self.assertEqual(excluded, [])
                self.assertEqual(len(thetas), 5)

    def test_synthetic_section_recovers_every_injected_angle(self):
        self._check(THIN, 0.05)

    def test_wing_custom_midspan_fixture_recovers_every_injected_angle(self):
        # The minimized fixture of a real wing: the sharpest hull vertex
        # (75.4 deg) is the LEADING edge, the TE is a 0.112 mm flat.
        fx = _load_fixture()
        pts = [tuple(p) for p in fx["points_xy_mm"]]
        self.assertGreater(
            min(AL._hull_interior_angles(AL._convex_hull(pts))),
            AL.TE_ANGLE_DEGENERATE_THRESHOLD_DEG)
        with self.assertRaises(BluntTrailingEdgeError):
            resolve_theta_deg(pts)         # the premise: the primary method refuses
        self._check(pts, 1e-4)

    def test_fixture_numbers_are_the_measured_ones(self):
        fx = _load_fixture()
        pts = [tuple(p) for p in fx["points_xy_mm"]]
        info, ref = AL._thin_flat_station(pts)
        self.assertIsNone(ref)
        exp = fx["expected"]
        self.assertEqual(info["hull_vertices"], exp["hull_vertices"])
        self.assertAlmostEqual(info["face_over_thickness"], exp["face_over_thickness"], places=5)
        self.assertAlmostEqual(info["face_over_chord"], exp["face_over_chord"], places=6)
        self.assertLess(abs(info["axis_deg"]), exp["axis_deg_abs_max"])
        self.assertEqual(info["le_side"], "min_x")
        # The 75.4 deg nose is never a face corner.
        for a in info["corner_angles_deg"]:
            self.assertGreater(a, 90.0)
            self.assertLess(a, 100.0)

    def test_provenance_carries_run_deltas_face_metrics_and_corners(self):
        sampler = _constant(THIN, 20.0)
        mid, others = _stations(sampler)
        prov = {}
        resolve_wing_theta_deg(mid, others, station_sampler=sampler, provenance=prov)
        tf = prov["thin_te_flat"]
        self.assertEqual(tf["run_stations"], 97)
        self.assertEqual(tf["run_fracs"][0], 0.02)
        self.assertEqual(tf["run_fracs"][-1], 0.98)
        self.assertLess(tf["adjacent_delta_max_deg"], 1e-6)
        self.assertEqual(tf["adjacent_delta_bound_deg"], AL.THIN_FLAT_MAX_ADJACENT_DELTA_DEG)
        ms = tf["mid_span"]
        for key in ("face_over_thickness", "face_over_chord", "corner_angles_deg",
                    "face_mm", "chord_mm", "thickness_mm", "strong_corners"):
            self.assertIn(key, ms)
        self.assertIsNone(prov["band_fracs"])
        self.assertEqual(len(prov["blunt_crosscheck"]), 5)


# --- per-station guards ------------------------------------------------------

class StationGuardTests(unittest.TestCase):
    def _refusal(self, pts):
        info, ref = AL._thin_flat_station(pts)
        self.assertIsNone(info)
        return ref

    def test_face_over_thickness_bound_refuses_a_thicker_flat(self):
        ref = self._refusal(THICK_FT)
        self.assertEqual(ref["guard"], "face_over_thickness")
        self.assertGreater(ref["value"], AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS)
        self.assertEqual(ref["bound"], AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS)

    def test_face_over_chord_bound_refuses_when_the_flat_is_long_against_the_chord(self):
        # The synthetic THIN section reads face/chord 0.00252 and resolves; with
        # the bound just below that value the same section must refuse, naming
        # the guard. (No real thin-flat shape fails this bound alone: the
        # thickness gate and the corner rules refuse every one measured.)
        saved = AL.THIN_FLAT_MAX_FACE_OVER_CHORD
        try:
            AL.THIN_FLAT_MAX_FACE_OVER_CHORD = 0.002
            ref = self._refusal(THIN)
            self.assertEqual(ref["guard"], "face_over_chord")
            self.assertAlmostEqual(ref["value"], 0.00252, places=5)
        finally:
            AL.THIN_FLAT_MAX_FACE_OVER_CHORD = saved
        self.assertIsNotNone(AL._thin_flat_station(THIN)[0])
        # and a rectangle whose long side would be the "face" (1.94 of chord)
        # is refused long before: by the corner window (135 deg arcs) or here.
        pts = _plank(length=50.0, height=40.0, round_end=True, samples=40)
        ref = self._refusal(pts)
        self.assertEqual(ref["guard"], "face_over_chord")
        self.assertGreater(ref["value"], AL.THIN_FLAT_MAX_FACE_OVER_CHORD)

    def test_fewer_than_twelve_hull_vertices_is_not_an_aerofoil(self):
        rect = [(-20.0, -5.0), (20.0, -5.0), (20.0, 5.0), (-20.0, 5.0)]
        ref = self._refusal(rect)
        self.assertEqual(ref["guard"], "hull_vertices")
        self.assertLess(ref["value"], AEROFOIL_MIN_HULL_VERTICES)

    def test_sharp_trailing_edge_has_no_flat_to_read(self):
        ref = self._refusal(SHARP)
        self.assertEqual(ref["guard"], "te_face_candidates")
        self.assertEqual(ref["value"], 0)

    def test_ambiguous_leading_edge_refuses(self):
        # A symmetric plank cut flat at BOTH ends fails the exactly-one rule
        # first; to reach the leading-edge-side check use a symmetric lens
        # with one flat: thickness peaks at mid-chord, so no side wins.
        pts = _lens_with_flat_end()
        info, ref = AL._thin_flat_station(pts)
        self.assertIsNone(info)
        self.assertEqual(ref["guard"], "leading_edge_side")

    def test_a_flat_at_the_thick_end_is_not_a_trailing_edge_and_refuses(self):
        # Every other guard passes (one straight face, corners ~101 deg,
        # face/thickness 0.020, face/chord 0.0024), and the thickness peaks at
        # 70% chord on the flat's side: leading_edge_side says "max_x". The
        # station refuses by name with its value; it is NOT flipped by 180 deg.
        pts = _thick_end_flat()
        self.assertEqual(AL.leading_edge_side(pts), "max_x")
        info, ref = AL._thin_flat_station(pts)
        self.assertIsNone(info)
        self.assertEqual(ref["guard"], "face_at_thick_end")
        self.assertEqual(ref["value"], "max_x")
        self.assertIn("thick end", ref["detail"])
        # at the wing level the message names the guard and its value
        sampler = _constant(pts)
        mid, others = _stations(sampler)
        with self.assertRaises(BluntTrailingEdgeError) as ctx:
            resolve_wing_theta_deg(mid, others, station_sampler=sampler)
        self.assertIn("mid-span station / face_at_thick_end", str(ctx.exception))
        self.assertIn("measured max_x against a bound of min_x", str(ctx.exception))

    def test_flat_at_both_ends_refuses(self):
        pts = _plank(length=60.0, height=4.0, bulge=0.4, samples=12)
        ref = self._refusal(pts)
        self.assertEqual(ref["guard"], "te_face_candidates")
        self.assertEqual(ref["value"], 2)

    def test_ambiguous_face_refuses_even_on_an_aerofoil(self):
        # A genuine thin-flat aerofoil whose nose is ALSO cut flat: two
        # straight corner-to-corner faces inside the window. The best one is
        # never picked.
        pts = [p for p in THIN if p[0] >= 0.6]          # nose cut at 1% of chord
        ref = self._refusal(pts)
        self.assertEqual(ref["guard"], "te_face_candidates")
        self.assertEqual(ref["value"], 2)

    def test_leading_edge_nose_is_never_paired_with_a_trailing_corner(self):
        # A flat-bottomed section with a coarse nose: the lower hull path from
        # the nose (80-90 deg) to the lower TE corner is straight. The pair
        # (nose, TE corner) would have face/chord ~ 1; it is not accepted.
        pts = _flat_bottom_coarse_nose()
        info, ref = AL._thin_flat_station(pts)
        self.assertIsNone(info)
        self.assertEqual(ref["guard"], "te_face_candidates")

    def test_a_cut_whose_corner_falls_below_the_window_is_refused(self):
        # A -15 deg slanted cut reads a 79.4 deg face corner: 0.6 deg under the
        # 80 deg lower bound, so it is refused rather than accepted (and the
        # +15 deg cut, whose corners read 86.8 / 109.4, resolves).
        low = _section(0.02, 0.4, 0.12, 0.02, -15.0, 150, -0.1036)
        ref = self._refusal(low)
        self.assertEqual(ref["guard"], "te_face_candidates")
        self.assertEqual(ref["value"], 0)
        ang = sorted(a for a in AL._hull_interior_angles(AL._convex_hull(low))
                     if a < AL.THIN_FLAT_STRONG_CORNER_DEG)
        self.assertLess(ang[0], 80.0)
        high = _section(0.02, 0.4, 0.12, 0.02, 15.0, 150, -0.1036)
        info, ref = AL._thin_flat_station(high)
        self.assertIsNone(ref)
        self.assertGreaterEqual(min(info["corner_angles_deg"]), AL.THIN_FLAT_CORNER_MIN_DEG)

    def test_a_curved_surface_between_two_corners_is_not_a_face(self):
        # THIN carries three corners (94.2, 101.6 and a 129.1 deg tessellation
        # vertex of the nose). Only the straight pair is a face; the curved
        # upper/lower surfaces between the others are not.
        info, ref = AL._thin_flat_station(THIN)
        self.assertIsNone(ref)
        self.assertEqual(info["strong_corners"], 3)
        self.assertEqual([round(a, 1) for a in sorted(info["corner_angles_deg"])], [94.2, 101.6])

    def test_collinear_split_face_counts_as_one_straight_face(self):
        # The tessellator may split the flat into collinear hull vertices
        # (wing_custom's face has two). Insert two on THIN's TE face.
        pts = list(THIN)
        i = len(pts) // 2
        a, b = pts[i - 1], pts[i]
        spliced = pts[:i] + [(a[0] + (b[0] - a[0]) / 3.0, a[1] + (b[1] - a[1]) / 3.0),
                             (a[0] + 2 * (b[0] - a[0]) / 3.0, a[1] + 2 * (b[1] - a[1]) / 3.0)] + pts[i:]
        info, ref = AL._thin_flat_station(spliced)
        self.assertIsNone(ref)
        self.assertAlmostEqual(info["face_mm"], math.hypot(a[0] - b[0], a[1] - b[1]), places=9)

    def test_hull_thickness_matches_a_brute_force_scan(self):
        for pts in (THIN, _section(m=0.06, t=0.18), [tuple(p) for p in _load_fixture()["points_xy_mm"]]):
            hull = AL._convex_hull(pts)
            got = AL._hull_max_thickness(hull)
            xs = [p[0] for p in hull]
            lo, hi = min(xs), max(xs)
            best = 0.0
            n = len(hull)
            for k in range(1, 2000):
                x = lo + (hi - lo) * k / 2000.0
                ys = []
                for i in range(n):
                    (xa, ya), (xb, yb) = hull[i], hull[(i + 1) % n]
                    if (xa <= x < xb) or (xb <= x < xa):
                        ys.append(ya + (yb - ya) * (x - xa) / (xb - xa))
                if len(ys) >= 2:
                    best = max(best, max(ys) - min(ys))
            self.assertAlmostEqual(got / best, 1.0, delta=2e-3)


def _plank(length=60.0, height=4.0, bulge=0.0, samples=12, round_end=False):
    """Plank with FLAT ends (90 deg corners); long sides bulged by ``bulge`` so
    the hull has >= 12 vertices. ``round_end``: left end is a semicircle."""
    pts = []
    for i in range(samples + 1):
        x = -length / 2 + length * i / samples
        pts.append((x, height / 2 + bulge * (1 - (2 * x / length) ** 2)))
    for i in range(samples, -1, -1):
        x = -length / 2 + length * i / samples
        pts.append((x, -height / 2 - bulge * (1 - (2 * x / length) ** 2)))
    if round_end:
        r = height / 2
        pts = [p for p in pts if p[0] >= -length / 2 + r - 1e-9]
        for k in range(1, 24):
            a = math.pi / 2 + math.pi * k / 24
            pts.append((-length / 2 + r + r * math.cos(a), r * math.sin(a)))
    return pts


def _thick_end_flat(chord=60.0, n=120):
    """Blunt (sqrt) nose, thickness PEAKING AT 70% of the chord, then a straight
    taper to a 0.2%-of-chord flat (corners ~101 deg): the one qualifying flat is
    at the thick end. face/thickness 0.020, face/chord 0.0024, two corners
    only, so every other guard passes; only the leading-edge-side check can
    refuse it."""
    top = []
    for i in range(n + 1):
        x = (1 - math.cos(math.pi * i / n)) / 2
        h = 0.06 * math.sqrt(x / 0.7) if x <= 0.7 else 0.06 - (0.06 - 0.0012) * (x - 0.7) / 0.3
        top.append((chord * x, chord * h))
    return top + [(x, -y) for x, y in reversed(top)]


def _lens_with_flat_end():
    """Symmetric sine lens: a sharp point at x=0 and a 0.157 mm flat at 99.5%
    of the length. One unambiguous flat, but thickness peaks at mid-chord, so
    the leading edge cannot be told from the trailing edge."""
    n = 60
    top = [(0.995 * 50.0 * i / n, 5.0 * math.sin(math.pi * 0.995 * i / n)) for i in range(n + 1)]
    bot = [(x, -y) for x, y in reversed(top)]
    return top + bot


def _flat_bottom_coarse_nose():
    """Flat lower surface (one straight hull edge nose -> lower TE corner), a
    coarse 83.8 deg nose and a 0.9 mm perpendicular TE flat: the nose and the
    lower TE corner are consecutive corners with a straight path between them,
    both inside the angle window. Two candidates -> refuse."""
    pts = [(0.0, 0.0), (0.3, 2.5), (4.0, 3.6)]
    for i in range(1, 41):
        u = i / 40.0
        x = 4.0 + 56.0 * u
        pts.append((x, 3.6 * (1.0 - u ** 1.5) + 0.45 * u ** 1.5))
    pts.append((60.0, -0.45))
    return pts


# --- non-wing shapes -----------------------------------------------------------

def _circle(r=20.0, n=360):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def _ellipse(a=40.0, b=10.0, n=360):
    return [(a * math.cos(2 * math.pi * k / n), b * math.sin(2 * math.pi * k / n)) for k in range(n)]


def _stadium(length=80.0, width=20.0, arc_n=60):
    r = width / 2
    c = length / 2 - r
    pts = [(c + r * math.cos(-math.pi / 2 + math.pi * k / arc_n), r * math.sin(-math.pi / 2 + math.pi * k / arc_n))
           for k in range(arc_n + 1)]
    pts += [(-c + r * math.cos(math.pi / 2 + math.pi * k / arc_n), r * math.sin(math.pi / 2 + math.pi * k / arc_n))
            for k in range(arc_n + 1)]
    return pts


def _rect_with_arcs(w=80.0, h=10.0, ch=0.5, per_corner=8):
    pts = []
    for cx, cy, a0 in [(w / 2, h / 2, 0.0), (-w / 2, h / 2, 90.0), (-w / 2, -h / 2, 180.0), (w / 2, -h / 2, 270.0)]:
        ox = cx - (1 if cx > 0 else -1) * ch
        oy = cy - (1 if cy > 0 else -1) * ch
        for k in range(per_corner):
            a = math.radians(a0) + (math.pi / 2) * (k / (per_corner - 1))
            pts.append((ox + ch * math.cos(a), oy + ch * math.sin(a)))
    return pts


def _non_wing_shapes():
    """Every >= 12-hull-vertex non-wing shape, plus polygons that carry four or
    more corners (the shapes a corner-count cap would have to stop)."""
    shapes = {
        "circle r20": _circle(),
        "ellipse 4:1": _ellipse(),
        "ellipse 4:1 rot30": rotate_points_deg(_ellipse(), 30.0),
        "stadium 80x20": _stadium(80.0, 20.0),
        "stadium 80x10": _stadium(80.0, 10.0),
        "stadium 80x10 rot30": rotate_points_deg(_stadium(80.0, 10.0), 30.0),
        "rect 80x10, 8-pt corner arcs": _rect_with_arcs(per_corner=8),
        "rect 80x10, 3-pt corner arcs (12 hull)": _rect_with_arcs(per_corner=3),
        # >= 4 strong corners
        "plank 60x4 flat both ends, bulged sides": _plank(60.0, 4.0, 0.4, 12),
        "plank 60x4 flat both ends, 0.1% bulge": _plank(60.0, 4.0, 0.06, 12),
        "plank 60x4 one flat, one round end": _plank(60.0, 4.0, 0.4, 12, round_end=True),
        "plank 200x1 one flat, one round end": _plank(200.0, 1.0, 0.1, 30, round_end=True),
        "plank 60x4 rot 17": rotate_points_deg(_plank(60.0, 4.0, 0.4, 12), 17.0),
        "regular 13-gon": [(10 * math.cos(2 * math.pi * k / 13), 10 * math.sin(2 * math.pi * k / 13)) for k in range(13)],
        "plate 80x20, 5 mm 4-point corner arcs": _rect_with_arcs(80.0, 20.0, 5.0, 4),
    }
    return shapes


class NonWingShapeTests(unittest.TestCase):
    def test_every_non_wing_shape_is_refused_at_the_station(self):
        for name, pts in _non_wing_shapes().items():
            with self.subTest(shape=name):
                info, ref = AL._thin_flat_station(pts)
                self.assertIsNone(info, msg="{} resolved".format(name))
                self.assertIsNotNone(ref)

    def test_every_non_wing_shape_is_refused_at_the_wing_level(self):
        for name, pts in _non_wing_shapes().items():
            hull = AL._convex_hull(pts)
            if len(hull) < AEROFOIL_MIN_HULL_VERTICES:
                continue         # a coupon: ChordAxisUnavailableError, never reaches the sampler
            with self.subTest(shape=name):
                sampler = _constant(pts)
                mid, others = _stations(sampler)
                with self.assertRaises(BluntTrailingEdgeError):
                    resolve_wing_theta_deg(mid, others, station_sampler=sampler)

    def test_shapes_with_four_or_more_corners_are_exercised_and_all_refuse(self):
        # adopts NO cap on the number of corners: the exactly-one rule and the
        # angle window already refuse every polygon below. Prove the claim is
        # exercised, i.e. that several of the shapes really carry >= 4 corners
        # (a cap of 3 would be the only thing that could matter).
        many = 0
        for name, pts in _non_wing_shapes().items():
            ang = AL._hull_interior_angles(AL._convex_hull(pts))
            if sum(1 for a in ang if a < AL.THIN_FLAT_STRONG_CORNER_DEG) >= 4:
                many += 1
                info, _ = AL._thin_flat_station(pts)
                self.assertIsNone(info, msg=name)
        self.assertGreaterEqual(many, 4)

    def test_coupons_stay_non_fatal_and_never_reach_the_sampler(self):
        rect = [(-20.0, -5.0), (20.0, -5.0), (20.0, 5.0), (-20.0, 5.0)]

        def poison(frac):
            raise AssertionError("station_sampler must not be called for a coupon")

        for pts in (rect, rotate_points_deg(rect, 30.0)):
            with self.assertRaises(ChordAxisUnavailableError) as ctx:
                resolve_wing_theta_deg(pts, (), station_sampler=poison)
            self.assertNotIsInstance(ctx.exception, BluntTrailingEdgeError)


# --- reachability -------------------------------------------------

class ReachabilityTests(unittest.TestCase):
    """Sharp mid-span -> sharp band -> thin flat ONLY when no sharp band exists. A
    band that exists and then fails a guard is still a refusal, and the thin
    flat is never even attempted (the message proves it)."""

    def _run(self, sampler):
        mid, others = _stations(sampler)
        prov = {}
        out = resolve_wing_theta_deg(mid, others, station_sampler=sampler, provenance=prov)
        return out, prov

    def _refused(self, sampler):
        mid, others = _stations(sampler)
        with self.assertRaises(BluntTrailingEdgeError) as ctx:
            resolve_wing_theta_deg(mid, others, station_sampler=sampler)
        return str(ctx.exception)

    def test_a_sharp_band_still_wins_when_both_would_resolve(self):
        # Mid-span is a THIN flat (thin flat would resolve it) but a sharp band
        # exists outboard: the source is used, exactly as.
        (theta, _t, _s, _e), prov = self._run(_split(THIN, SHARP, 0.85, theta=15.0))
        self.assertEqual(prov["theta_source"], "sharp_te_band")
        self.assertIsNone(prov["thin_te_flat"])
        self.assertAlmostEqual(theta, 15.0, delta=1e-6)

    def test_band_found_but_failing_spread_still_refuses(self):
        sampler = _split(THIN, SHARP, 0.85, extra=lambda f: (f - 0.85) * 100.0)
        msg = self._refused(sampler)
        self.assertIn("spread threshold", msg)
        self.assertNotIn("thin-trailing-edge-flat", msg)

    def test_band_found_but_failing_leading_edge_side_still_refuses(self):
        msg = self._refused(_split(THIN, SHARP, 0.85, flip_frac=0.3))
        self.assertIn("leading-edge", msg)
        self.assertNotIn("thin-trailing-edge-flat", msg)

    def test_band_found_but_failing_hull_diameter_still_refuses(self):
        def sampler(frac):
            if frac < 0.02 or frac > 0.98:
                return None
            if frac <= 0.85:
                return rotate_points_deg(THIN, 10.0)
            return list(SHARP)
        msg = self._refused(sampler)
        self.assertIn("hull-diameter", msg)
        self.assertNotIn("thin-trailing-edge-flat", msg)

    def test_a_short_sharp_run_is_no_band_and_falls_through(self):
        # Only 4 sharp stations (0.95..0.98) < SHARP_BAND_MIN_STATIONS: there
        # is no band, so the thin flat is tried and resolves.
        def sampler(frac):
            if frac < 0.02 or frac > 0.98:
                return None
            return rotate_points_deg(SHARP if frac >= 0.95 else THIN, 5.0)
        (theta, _t, _s, _e), prov = self._run(sampler)
        self.assertEqual(prov["theta_source"], "thin_te_flat")
        self.assertIn("longest sharp run below", prov["thin_te_flat"]["no_sharp_band"])
        self.assertAlmostEqual(theta, 5.0, delta=0.05)

    def test_no_sharp_station_anywhere_is_no_band_and_falls_through(self):
        (theta, _t, _s, _e), prov = self._run(_constant(THIN, 12.0))
        self.assertEqual(prov["theta_source"], "thin_te_flat")
        self.assertIn("no contiguous run", prov["thin_te_flat"]["no_sharp_band"])
        self.assertAlmostEqual(theta, 12.0, delta=0.05)

    def test_without_a_sampler_nothing_changes(self):
        mid = rotate_points_deg(THIN, 15.0)
        with self.assertRaises(BluntTrailingEdgeError) as ctx:
            resolve_wing_theta_deg(mid, [mid])
        with self.assertRaises(BluntTrailingEdgeError) as direct:
            resolve_theta_deg(mid)
        self.assertEqual(str(ctx.exception), str(direct.exception))

    def test_a_coupon_never_reaches_the_thin_flat(self):
        rect = [(-20.0, -0.3), (20.0, -0.3), (20.0, 0.3), (-20.0, 0.3)]

        def poison(frac):
            raise AssertionError("sampler called for a coupon mid-span")
        with self.assertRaises(ChordAxisUnavailableError):
            resolve_wing_theta_deg(rect, (), station_sampler=poison)

    def test_a_sharp_mid_span_never_calls_the_sampler(self):
        def poison(frac):
            raise AssertionError("sampler called although mid-span resolved")
        mid = rotate_points_deg(SHARP, 3.0)
        theta, thetas, spread, excluded = resolve_wing_theta_deg(
            mid, [mid], station_sampler=poison)
        self.assertAlmostEqual(theta, 3.0, delta=1e-6)

    def test_a_plain_alignment_error_never_calls_the_sampler(self):
        # Sharp but symmetric: a real chord axis, an ambiguous leading edge.
        lens = [(50 * i / 100.0, 5 * math.sin(math.pi * i / 100.0)) for i in range(0, 101)]
        lens += [(50 * i / 100.0, -5 * math.sin(math.pi * i / 100.0)) for i in range(99, 0, -1)]

        def poison(frac):
            raise AssertionError("sampler called for a non-blunt AlignmentError")
        with self.assertRaises(AlignmentError) as ctx:
            resolve_wing_theta_deg(lens, (), station_sampler=poison)
        self.assertNotIsInstance(ctx.exception, BluntTrailingEdgeError)


# --- wing-level guards 4a-4d ---------------------------------------------------

class WingGuardTests(unittest.TestCase):
    def _refused(self, sampler, others=None):
        mid = sampler(0.5)
        if others is None:
            others = [s for s in (sampler(f) for f in (0.1, 0.3, 0.7, 0.9)) if s is not None]
        with self.assertRaises(BluntTrailingEdgeError) as ctx:
            resolve_wing_theta_deg(mid, others, station_sampler=sampler)
        return str(ctx.exception)

    def test_4a_run_around_mid_span_shorter_than_the_minimum_refuses(self):
        def sampler(frac):
            if frac < 0.02 or frac > 0.98:
                return None
            return list(THIN if 0.48 <= frac <= 0.51 else THICK)   # 4 stations
        self.assertLess(4, SHARP_BAND_MIN_STATIONS)
        msg = self._refused(sampler)
        self.assertIn("thin-flat run length", msg)
        self.assertIn("measured 4 against a bound of >= 5", msg)

    def test_4a_mid_span_must_itself_read_a_thin_flat(self):
        def sampler(frac):
            if frac < 0.02 or frac > 0.98:
                return None
            return list(THICK_FT if abs(frac - 0.5) < 1e-9 else THIN)
        msg = self._refused(sampler)
        self.assertIn("mid-span station / face_over_thickness", msg)

    def test_4a_a_run_of_exactly_the_minimum_resolves(self):
        def sampler(frac):
            if frac < 0.02 or frac > 0.98:
                return None
            return list(THIN if 0.48 <= frac <= 0.52 else THICK)   # 5 stations
        mid = sampler(0.5)
        prov = {}
        # standard stations are THICK -> excluded; mid-span alone sets theta
        theta, thetas, _s, excluded = resolve_wing_theta_deg(
            mid, [sampler(0.1)], station_sampler=sampler, provenance=prov)
        self.assertEqual(prov["thin_te_flat"]["run_stations"], 5)

    def test_4b_an_adjacent_station_step_refuses(self):
        # A 1 deg step between 0.60 and 0.61: a wing with a structural step,
        # not a smooth washout.
        sampler = _split(THIN, THIN, 1.0, extra=lambda f: 1.0 if f >= 0.605 else 0.0)
        msg = self._refused(sampler)
        self.assertIn("thin-flat adjacent-station delta", msg)
        self.assertIn("against a bound of {:.2f} deg".format(AL.THIN_FLAT_MAX_ADJACENT_DELTA_DEG), msg)

    def test_4b_a_smooth_washout_drift_is_accepted(self):
        # W04's design washout is 0.040 deg per station; W16's 8 deg is 0.080.
        sampler = _split(THIN, THIN, 1.0, extra=lambda f: 8.0 * f)
        mid, others = _stations(sampler)
        prov = {}
        theta, _t, spread, _e = resolve_wing_theta_deg(
            mid, others, station_sampler=sampler, provenance=prov)
        self.assertAlmostEqual(theta, 4.0, delta=0.1)
        self.assertLess(prov["thin_te_flat"]["adjacent_delta_max_deg"], 0.1)

    def test_4c_standard_station_leading_edge_flip_refuses(self):
        sampler = _constant(THIN)
        mid, others = _stations(sampler)
        others[1] = rotate_points_deg(others[1], 180.0)
        msg = self._refused(sampler, others=others)
        self.assertIn("thin-flat standard-station leading-edge side", msg)

    def test_4c_standard_station_hull_diameter_disagreement_refuses(self):
        sampler = _constant(THIN)
        mid, others = _stations(sampler)
        others[2] = rotate_points_deg(others[2], 10.0)
        msg = self._refused(sampler, others=others)
        self.assertIn("thin-flat standard-station hull-diameter", msg)

    def test_4c_the_spread_check_still_runs_after_the_thin_flat_reading(self):
        # 4.5 deg is inside the 5 deg hull-diameter cross-check, so it passes
        # that and is then reported through the unchanged spread check.
        sampler = _constant(THIN)
        mid, others = _stations(sampler)
        others[0] = rotate_points_deg(others[0], 4.5)
        theta, thetas, spread, excluded = resolve_wing_theta_deg(
            mid, others, station_sampler=sampler)
        self.assertAlmostEqual(spread, 4.5, delta=0.05)
        self.assertEqual(excluded, [])

    def test_4d_standard_stations_vote_through_the_thin_flat_only_on_that_path(self):
        # thin-flat path: the four standard stations resolve and vote.
        sampler = _constant(THIN, 7.0)
        mid, others = _stations(sampler)
        _t, thetas, _s, excluded = resolve_wing_theta_deg(
            mid, others, station_sampler=sampler)
        self.assertEqual(len(thetas), 5)
        self.assertEqual(excluded, [])
        # every other path excludes a thin-flat standard station exactly as:
        # sharp mid-span...
        sharp_mid = rotate_points_deg(SHARP, 7.0)
        _t, thetas, _s, excluded = resolve_wing_theta_deg(sharp_mid, [others[0], sharp_mid])
        self.assertEqual([i for i, _ in excluded], [0])
        self.assertEqual(len(thetas), 2)
        # ...and the sharp-band path
        sampler2 = _split(THICK, SHARP, 0.85, theta=7.0)
        mid2, others2 = _stations(sampler2)
        others2 = [rotate_points_deg(THIN, 7.0)] + others2
        prov = {}
        _t, thetas, _s, excluded = resolve_wing_theta_deg(
            mid2, others2, station_sampler=sampler2, provenance=prov)
        self.assertEqual(prov["theta_source"], "sharp_te_band")
        self.assertIn(0, [i for i, _ in excluded])

    def test_refusal_message_states_what_was_measured_and_which_guard_refused(self):
        sampler = _constant(THICK_FT)
        msg = self._refused(sampler)
        self.assertIn("no sharp trailing edge was found", msg)
        self.assertIn("not necessarily the trailing edge", msg)
        self.assertNotIn("this wing's trailing edge is squared off", msg)
        self.assertIn("A thin-trailing-edge-flat reading", msg)
        self.assertIn("mid-span station / face_over_thickness", msg)
        self.assertIn("against a bound of {}".format(AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS), msg)
        self.assertIn("at most 0.025 of the section thickness", msg)
        self.assertIn("will NOT help", msg)
        self.assertIn("No contiguous run", msg)       # the half is preserved


# --- truth grid / threshold -----------------------------------------------------

class TruthGridTests(unittest.TestCase):
    """A reduced version of (10416 sections). Truth is the pre-truncation chord
    line: 0 deg."""

    ACCURACY_DEG = 0.2

    def test_worst_error_inside_the_bound_is_under_the_accuracy_criterion(self):
        worst = 0.0
        resolved = 0
        for coef in (-0.1015, -0.1036):            # open and cusped TE
            for tilt in (0.0, 15.0, -15.0):
                for m in (0.0, 0.04, 0.06):
                    for p in (0.3, 0.5):
                        for t in (0.09, 0.18):
                            for ratio in ((None, 0.024) if coef == -0.1015 else (0.024, 0.0245)):
                                pts = _section(m, p, t, ratio, tilt, 150, coef)
                                if pts is None:
                                    continue
                                info, _ = AL._thin_flat_station(pts)
                                if info is None:
                                    continue
                                resolved += 1
                                self.assertLessEqual(
                                    info["face_over_thickness"], AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS)
                                err = abs((info["theta_deg"] + 180.0) % 360.0 - 180.0)
                                worst = max(worst, err)
        self.assertGreater(resolved, 60)
        self.assertLess(worst, self.ACCURACY_DEG)

    def test_the_worst_case_family_is_near_the_criterion_so_the_bound_cannot_be_raised(self):
        # 6% camber peaking at 50% chord, cusped TE, 9% thick, cut at 0.050:
        # the grid's worst case. Above the bound it exceeds the criterion.
        pts = _section(0.06, 0.5, 0.09, 0.05, 0.0, 300, -0.1036)
        info, ref = AL._thin_flat_station(pts)
        self.assertIsNone(info)
        self.assertEqual(ref["guard"], "face_over_thickness")
        saved = AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS
        try:
            AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS = 0.06
            info, _ = AL._thin_flat_station(pts)
            self.assertGreater(abs(info["theta_deg"]), self.ACCURACY_DEG)
        finally:
            AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS = saved

    def test_designed_blunt_sections_read_exactly(self):
        for m in (0.0, 0.04, 0.06):
            pts = []
            n = 150
            for i in range(n + 1):
                xc = i / n
                yt = _half_t(xc, 0.12, -0.1036) + xc * 0.0025 / 2
                pts.append((60 * xc, 60 * (_camber(xc, m, 0.4) + yt)))
            for i in range(n, -1, -1):
                xc = i / n
                yt = _half_t(xc, 0.12, -0.1036) + xc * 0.0025 / 2
                pts.append((60 * xc, 60 * (_camber(xc, m, 0.4) - yt)))
            info, ref = AL._thin_flat_station(pts)
            self.assertIsNone(ref, msg="m={}".format(m))
            self.assertAlmostEqual(info["theta_deg"], 0.0, places=6)

    def test_constants_are_the_measured_values(self):
        self.assertEqual(AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS, 0.025)
        self.assertEqual(AL.THIN_FLAT_MAX_FACE_OVER_CHORD, 0.01)
        self.assertEqual(AL.THIN_FLAT_CORNER_MIN_DEG, 80.0)
        self.assertEqual(AL.THIN_FLAT_CORNER_MAX_DEG, 130.0)
        self.assertEqual(AL.THIN_FLAT_STRONG_CORNER_DEG, 150.0)
        self.assertEqual(AL.THIN_FLAT_STRAIGHT_DEV_FRAC, 0.002)
        self.assertEqual(AL.THIN_FLAT_MAX_ADJACENT_DELTA_DEG, 0.4)
        self.assertFalse(hasattr(AL, "THIN_FLAT_MAX_STRONG_CORNERS"))
        # wing_custom (0.02101) must stay inside the accuracy gate
        self.assertLess(0.02102, AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS)


class TruncationLevelTests(unittest.TestCase):
    """The existing generator behind table. ``resolve_theta_deg`` still refuses at
    every level (``test_alignment.TruncatedTrailingEdgeTests``); this is the
    WHOLE-WING behaviour: a level resolves iff its face/thickness is inside the
    bound."""

    LEVELS = (0.999, 0.99, 0.98, 0.96, 0.95, 0.90, 0.85)

    def test_wing_level_verdict_follows_the_measured_face_over_thickness(self):
        verdicts = {}
        for level in self.LEVELS:
            pts = _truncated_te_naca_points(truncate_frac=level)
            with self.assertRaises(BluntTrailingEdgeError):
                resolve_theta_deg(pts)          # per-section: unchanged, still refuses
            # face / thickness measured independently of the detector: the
            # TE face is the pair of points at the truncation, the thickness
            # the hull's maximum extent.
            mid = len(pts) // 2
            face = math.hypot(pts[mid - 1][0] - pts[mid][0], pts[mid - 1][1] - pts[mid][1])
            ratio = face / AL._hull_max_thickness(AL._convex_hull(pts))
            sampler = _constant(pts)
            m, others = _stations(sampler)
            try:
                theta, *_ = resolve_wing_theta_deg(m, others, station_sampler=sampler)
                verdicts[level] = ("resolves", ratio, theta)
            except BluntTrailingEdgeError:
                verdicts[level] = ("refuses", ratio, None)
        for level, (verdict, ratio, theta) in verdicts.items():
            with self.subTest(level=level, face_over_thickness=round(ratio, 4)):
                if ratio <= AL.THIN_FLAT_MAX_FACE_OVER_THICKNESS:
                    self.assertEqual(verdict, "resolves")
                    self.assertLess(abs(theta), 0.2)
                else:
                    self.assertEqual(verdict, "refuses")
        # Measured 0.0233 (inside), 0.0442, 0.0672, 0.1123, 0.1344, 0.2412, 0.3420.
        self.assertEqual(verdicts[0.999][0], "resolves")
        for level in (0.99, 0.98, 0.96, 0.95, 0.90, 0.85):
            self.assertEqual(verdicts[level][0], "refuses", msg=level)


# --- baseline results -----------------------------------------------------------

class UnchangedResultsTests(unittest.TestCase):
    """Literal baseline results for these inputs (theta, per-station thetas,
    spread). Only the excluded-station REASON TEXT is asserted separately (the
    refusal wording)."""

    def test_sharp_mid_span_with_a_blunt_standard_station(self):
        pts = _naca_like_points()
        mid = rotate_points_deg(pts, 1.0)
        others = [rotate_points_deg(pts, 1.0), _blunt_ellipse_points(), rotate_points_deg(pts, 1.5)]
        theta, thetas, spread, excluded = resolve_wing_theta_deg(mid, others)
        self.assertEqual(theta, 1.0)
        self.assertEqual(thetas, [1.0, 1.0, 1.5000000000000002])
        self.assertEqual(spread, 0.5)
        self.assertEqual([i for i, _ in excluded], [1])

    def test_sharp_mid_span_with_washout_stations(self):
        pts = _naca_like_points()
        mid = rotate_points_deg(pts, 1.0)
        others = [rotate_points_deg(pts, 1.0 + 0.7 * k) for k in range(4)]
        theta, thetas, spread, excluded = resolve_wing_theta_deg(mid, others)
        self.assertEqual(theta, 1.0)
        self.assertEqual(thetas, [1.0, 1.0, 1.7, 2.3999999999999995, 3.1])
        self.assertEqual(spread, 2.0999999999999943)
        self.assertEqual(excluded, [])

    def test_sharp_band_path(self):
        from test_alignment import _elevon_like_sampler
        sampler = _elevon_like_sampler(theta_inj=15.0)
        mid = sampler(0.5)
        others = [sampler(f) for f in (0.1, 0.3, 0.7, 0.9)]
        prov = {}
        theta, thetas, spread, excluded = resolve_wing_theta_deg(
            mid, others, station_sampler=sampler, provenance=prov)
        self.assertEqual(theta, 14.999999999999998)
        self.assertEqual(thetas, [14.999999999999998, 14.999999999999998])
        self.assertEqual(spread, 0.0)
        self.assertEqual([i for i, _ in excluded], [0, 1, 2])
        self.assertEqual(prov["theta_source"], "sharp_te_band")
        self.assertEqual((prov["band_fracs"][0], prov["band_fracs"][-1]), (0.86, 0.98))
        self.assertEqual(prov["band_spread_deg"], 0.0)
        self.assertIsNone(prov["thin_te_flat"])

    def test_resolve_theta_deg_is_untouched_for_every_existing_shape(self):
        # Same exception CLASS and the same sharp-reading numbers as before.
        self.assertAlmostEqual(resolve_theta_deg(rotate_points_deg(SHARP, 33.0)), 33.0, delta=1e-9)
        for pts in (THIN, THICK, _truncated_te_naca_points()):
            with self.assertRaises(BluntTrailingEdgeError):
                resolve_theta_deg(pts)
        rect = [(-20.0, -5.0), (20.0, -5.0), (20.0, 5.0), (-20.0, 5.0)]
        with self.assertRaises(ChordAxisUnavailableError):
            resolve_theta_deg(rect)


if __name__ == "__main__":
    unittest.main()
