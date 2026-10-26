import math
import unittest

from aircraft_core.alignment import (
    _convex_hull,
    AlignmentError,
    ChordAxisUnavailableError,
    TE_ANGLE_DEGENERATE_THRESHOLD_DEG,
    chord_line_angle_deg,
    decide_alignment,
    leading_edge_side,
    min_area_rect_angle_deg,
    AEROFOIL_MIN_HULL_VERTICES,
    BluntTrailingEdgeError,
    SHARP_BAND_MIN_STATIONS,
    SHARP_BAND_MAX_SPREAD_DEG,
    BLUNT_CROSSCHECK_MAX_DEG,
    resolve_theta_deg,
    resolve_wing_theta_deg,
    rotate_points_deg,
)


def _naca_camber(xc, m=0.02, p=0.4):
    """NACA-2412-like mean camber line offset (fraction of chord), so this
    fixture is not left-right symmetric (a purely symmetric section makes
    the minimum-area rectangle's long-axis candidate a near-exact tie
    between two hull edges, which is a fixture artifact -- see this
    module's own discovery that ``min_area_rect_angle_deg`` is
    camber-biased on a REAL cambered aerofoil; a symmetric test fixture
    cannot exercise that at all)."""
    if xc < p:
        return (m / (p * p)) * (2 * p * xc - xc * xc)
    return (m / ((1 - p) ** 2)) * ((1 - 2 * p) + 2 * p * xc - xc * xc)


def _naca_like_points(chord=60.0, thickness_frac=0.12, n=80):
    """A crude but genuinely aerofoil-shaped (thickest well forward of
    mid-chord, blunt LE, sharp TE, cambered) closed ring, chord along +X,
    LE at x=0. Good enough to exercise axis + LE detection without pulling
    in the real NACA fixture generator. The camber term is a simple
    additive offset of the half-thickness distribution (not a proper
    surface-normal camber construction) -- adequate for this module's
    tests, not for anything claiming aerodynamic fidelity.
    """
    def half_thickness(xc):
        # -0.1036 (not the open-TE -0.1015) so yt(1) == 0 exactly: the upper
        # and lower surfaces meet at a single, genuinely sharp cusp at x=chord,
        # matching a real (closed) trailing edge. The open-TE coefficient left
        # a small residual TE thickness that, on this coarse (80-point)
        # synthetic fixture, was blunt enough to trip the degenerate-TE guard
        # (measured interior angle ~94 deg, just over the 90 deg threshold) --
        # a fixture resolution artifact, not something the real NACA/mesh
        # fixtures in tests/fixtures/wing/ exhibit (those are true closed-TE
        # meshes with hundreds of points per section).
        return 5 * thickness_frac * chord * (
            0.2969 * math.sqrt(xc) - 0.1260 * xc - 0.3516 * xc ** 2
            + 0.2843 * xc ** 3 - 0.1036 * xc ** 4
        )

    pts = []
    for i in range(n + 1):
        x = chord * i / n
        xc = x / chord
        yt = 0.0 if i == n else half_thickness(xc)
        yc = _naca_camber(xc) * chord
        pts.append((x, yc + yt))
    # lower surface, TE to LE, mirrored
    for i in range(n, -1, -1):
        x = chord * i / n
        xc = x / chord
        yt = 0.0 if i == n else half_thickness(xc)
        yc = _naca_camber(xc) * chord
        pts.append((x, yc - yt))
    return pts


def _truncated_te_naca_points(chord=60.0, thickness_frac=0.12,
                               truncate_frac=0.96, n=80):
    """A genuinely aerofoil-shaped, cambered section with a SQUARED-OFF /
    TRUNCATED trailing edge (chord cut off at ``truncate_frac`` of full
    chord, no closing taper to a point) -- extremely common on printed
    wings, often required for a thin printed section to have a
    manufacturable (non-feathered) TE at all. Uses the OPEN-TE thickness
    coefficient (-0.1015, not the closed-TE -0.1036 ``_naca_like_points``
    uses to force a sharp cusp), so upper and lower surfaces do NOT meet
    at the truncation -- the hull closes across a near-vertical face
    instead, exactly like a real blunt TE.
    """
    def half_thickness(xc):
        return 5 * thickness_frac * chord * (
            0.2969 * math.sqrt(xc) - 0.1260 * xc - 0.3516 * xc ** 2
            + 0.2843 * xc ** 3 - 0.1015 * xc ** 4
        )

    pts = []
    for i in range(n + 1):
        xc = truncate_frac * i / n
        x = chord * xc
        yt = half_thickness(xc)
        yc = _naca_camber(xc) * chord
        pts.append((x, yc + yt))
    for i in range(n, -1, -1):
        xc = truncate_frac * i / n
        x = chord * xc
        yt = half_thickness(xc)
        yc = _naca_camber(xc) * chord
        pts.append((x, yc - yt))
    return pts


def _blunt_ellipse_points(a=30.0, b=5.0, n=300):
    """A smooth, rounded closed ring with NO sharp corner anywhere -- measured
    directly (this module's own development): sharpest hull vertex reads ~172.8
    deg, well above ``TE_ANGLE_DEGENERATE_THRESHOLD_DEG`` (90 deg). Stands in
    for a real blunt/flat-cut station for the tests."""
    return [
        (30.0 + a * math.cos(t), b * math.sin(t))
        for t in [i * 2 * math.pi / n for i in range(n)]
    ]


class RotatePointsTests(unittest.TestCase):
    def test_identity_at_zero(self):
        pts = [(1.0, 2.0), (3.0, -4.0)]
        self.assertEqual(rotate_points_deg(pts, 0.0), pts)

    def test_90_degrees_about_origin(self):
        out = rotate_points_deg([(1.0, 0.0)], 90.0)
        self.assertAlmostEqual(out[0][0], 0.0, places=9)
        self.assertAlmostEqual(out[0][1], 1.0, places=9)

    def test_about_nonzero_center(self):
        out = rotate_points_deg([(1.0, 0.0)], 180.0, center=(1.0, 0.0))
        self.assertAlmostEqual(out[0][0], 1.0, places=9)
        self.assertAlmostEqual(out[0][1], 0.0, places=9)

    def test_empty(self):
        self.assertEqual(rotate_points_deg([], 30.0), [])


class MinAreaRectAngleTests(unittest.TestCase):
    """``min_area_rect_angle_deg`` is retained (it is a legitimate
    bounding-rectangle-extent tool), but it is disqualified as a chord-axis
    estimator (camber bias, ~1.5 deg on the reference fixture
    and on a real wing -- see its own docstring). These tests pin what
    it actually computes; they do NOT claim it is bias-free for a cambered
    aerofoil."""

    def test_axis_aligned_rectangle_is_zero(self):
        rect = [(0.0, 0.0), (20.0, 0.0), (20.0, 5.0), (0.0, 5.0)]
        self.assertAlmostEqual(min_area_rect_angle_deg(rect), 0.0, places=6)

    def test_rotated_rectangle_recovers_angle(self):
        rect = [(0.0, 0.0), (20.0, 0.0), (20.0, 5.0), (0.0, 5.0)]
        for theta in (0.3, 15.0, 45.0, -63.36, 89.0):
            rotated = rotate_points_deg(rect, theta)
            got = min_area_rect_angle_deg(rotated)
            # Axis is only recoverable mod 180; normalize both to (-90, 90].
            expected = theta
            while expected <= -90.0:
                expected += 180.0
            while expected > 90.0:
                expected -= 180.0
            diff = abs(got - expected)
            diff = min(diff, 180.0 - diff)
            self.assertLess(diff, 1e-3, msg="theta={}".format(theta))

    def test_degenerate_returns_none(self):
        self.assertIsNone(min_area_rect_angle_deg([(0.0, 0.0), (1.0, 1.0)]))
        self.assertIsNone(min_area_rect_angle_deg([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]))

    def test_is_camber_biased_on_a_cambered_aerofoil(self):
        # The disqualifying measurement, reproduced as a permanent
        # regression: a cambered section at TRUE zero rotation must NOT
        # read ~0 from this function (if it ever does, this function has
        # changed and resolve_theta_deg's decision to avoid it should be
        # revisited).
        pts = _naca_like_points()
        angle = min_area_rect_angle_deg(pts)
        self.assertGreater(abs(angle), 1.0)


class ChordLineAngleTests(unittest.TestCase):
    """amendment 1's replacement axis source."""

    def test_aligned_cambered_wing_is_near_zero(self):
        # Unlike min_area_rect_angle_deg, the chord-line axis is close to
        # bias-free on a cambered section -- this is exactly why it
        # replaced the minimum-area rectangle.
        pts = _naca_like_points()
        angle = chord_line_angle_deg(pts)
        self.assertIsNotNone(angle)
        self.assertLess(abs(angle), 1.0)

    def test_rotated_wing_recovers_the_angle(self):
        pts = _naca_like_points()
        for theta in (0.3, 15.0, -63.36):
            rotated = rotate_points_deg(pts, theta)
            angle = chord_line_angle_deg(rotated)
            self.assertIsNotNone(angle)
            self.assertLess(abs(angle - theta), 0.5, msg="theta={}".format(theta))

    def test_blunt_station_is_degenerate(self):
        # no genuine trailing edge -> None, not a guess.
        pts = _blunt_ellipse_points()
        self.assertIsNone(chord_line_angle_deg(pts))

    def test_degenerate_returns_none(self):
        self.assertIsNone(chord_line_angle_deg([(0.0, 0.0), (1.0, 1.0)]))


class TruncatedTrailingEdgeTests(unittest.TestCase):
    """: a squared-off/truncated TE is an extremely common REAL wing shape (often
    required for a printable, non-feathered TE), not a synthetic edge case --
    unlike ``_blunt_ellipse_points`` above, this fixture is genuinely
    aerofoil-shaped everywhere except the TE itself. This class records,
    empirically, which branch a real wing of this shape actually takes -- "do
    not ship this discrimination with the case unexercised."

    MEASURED: every tested truncation level (85% to 99.9% chord) reads a
    sharpest hull-vertex angle of 93.85-94.19 deg -- consistently ABOVE
    ``TE_ANGLE_DEGENERATE_THRESHOLD_DEG`` (60 deg), so this shape reliably
    takes the ``ChordAxisUnavailableError`` branch (theta=0, no rotation,
    non-fatal), not the fail-closed ``AlignmentError`` branch. This is a
    KNOWN, ACCEPTED limit -- see ``ChordAxisUnavailableError``'s own
    docstring for the backstop reasoning (a
    truncated-TE wing misaligned by 30 degrees or more still fails closed
    at the pre-existing ``estimate_rotation_deg``/``check_rotation_envelope``
    cap; the residual exposure is bounded to misalignment under 30
    degrees, which degrades to the pre-existing behaviour, not a new
    hazard).
    """

    def test_truncated_te_still_defeats_the_PRIMARY_axis_method(self):
        # chord_line_angle_deg needs a sharp corner and a truncated TE has
        # none; the fallback exists for exactly this case.
        self.assertIsNone(chord_line_angle_deg(_truncated_te_naca_points()))

    def test_truncated_te_now_FAILS_CLOSED(self):
        # A non-fatal fallback is not acceptable here: theta would fall back
        # to 0 and a truncated-TE wing misaligned by 15 degrees would slice to
        # completion (every layer ok) with every member built 15 degrees off
        # the true chord axis. Resolution must raise instead.
        with self.assertRaises(BluntTrailingEdgeError):
            resolve_theta_deg(_truncated_te_naca_points())

    def test_it_fails_closed_at_every_truncation_level(self):
        for truncate_frac in (0.999, 0.99, 0.98, 0.96, 0.95, 0.90, 0.85):
            with self.assertRaises(BluntTrailingEdgeError,
                                   msg="truncate_frac={}".format(truncate_frac)):
                resolve_theta_deg(
                    _truncated_te_naca_points(truncate_frac=truncate_frac))

    def test_truncated_te_sharpest_angle_is_consistently_above_threshold(self):
        # Empirical sweep, not a single point -- the whole point of
        # "do not ship this discrimination with the case unexercised".
        from aircraft_core.alignment import _convex_hull, _sharpest_hull_vertex
        for truncate_frac in (0.999, 0.99, 0.98, 0.96, 0.95, 0.90, 0.85):
            pts = _truncated_te_naca_points(truncate_frac=truncate_frac)
            hull = _convex_hull(pts)
            _idx, angle = _sharpest_hull_vertex(hull)
            self.assertGreater(
                angle, TE_ANGLE_DEGENERATE_THRESHOLD_DEG,
                msg="truncate_frac={}".format(truncate_frac))


class LeadingEdgeSideTests(unittest.TestCase):
    def test_forward_loaded_aerofoil_is_min_x(self):
        pts = _naca_like_points()
        self.assertEqual(leading_edge_side(pts), "min_x")

    def test_flipped_aerofoil_is_max_x(self):
        pts = _naca_like_points()
        flipped = [(-x, y) for x, y in pts]
        self.assertEqual(leading_edge_side(flipped), "max_x")

    def test_symmetric_plate_is_ambiguous(self):
        # A rectangle: thickness is constant along X, forward/aft peaks
        # are identical -> ambiguous.
        rect = [(0.0, 0.0), (20.0, 0.0), (20.0, 5.0), (0.0, 5.0)]
        self.assertIsNone(leading_edge_side(rect))

    def test_ellipse_is_ambiguous(self):
        # 300 points, not 60: a real mesh/slice cross-section is far denser
        # than this synthetic fixture, and the station-binning method needs
        # enough points per bin to avoid a sparse-sampling artifact (an
        # empty bin next to a full one manufactures false asymmetry) --
        # this is a fixture-density requirement, not a leading-edge-
        # detection tolerance.
        pts = _blunt_ellipse_points()
        self.assertIsNone(leading_edge_side(pts))


class ResolveThetaTests(unittest.TestCase):
    def test_already_aligned_wing_is_near_zero(self):
        # With the chord-line axis, an aligned cambered wing reads near zero --
        # the OPPOSITE of the disqualified min_area_rect_angle_deg, which read
        # ~1.5 deg here. This is the property design point C's epsilon gate
        # depends on.
        pts = _naca_like_points()
        theta = resolve_theta_deg(pts)
        self.assertLess(abs(theta), 1.0)

    def test_synthetic_rotation_matrix(self):
        # The CANONICAL frame this fixture resolves to must be the SAME
        # frame regardless of what in-plane angle it started at -- this is
        # the property the pipeline actually depends on (matrix item 1:
        # "the de-rotated pipeline output MATCHES the theta=0 run").
        base = _naca_like_points()
        base_theta = resolve_theta_deg(base)
        self.assertLess(abs(base_theta), 1.0)
        canonical_base = rotate_points_deg(base, -base_theta)
        for theta_in in (0.0, 0.3, 15.0, 45.0, 90.0, -90.0, 180.0, -63.36, 137.0):
            rotated = rotate_points_deg(base, theta_in)
            theta_out = resolve_theta_deg(rotated)
            canonical_rotated = rotate_points_deg(rotated, -theta_out)
            for (bx, by), (dx, dy) in zip(canonical_base, canonical_rotated):
                self.assertLess(math.hypot(bx - dx, by - dy), 0.1,
                                 msg="theta_in={}".format(theta_in))

    def test_degenerate_raises(self):
        with self.assertRaises(AlignmentError):
            resolve_theta_deg([(0.0, 0.0), (1.0, 1.0)])

    def test_symmetric_plate_raises(self):
        rect = [(0.0, 0.0), (20.0, 0.0), (20.0, 5.0), (0.0, 5.0)]
        with self.assertRaises(AlignmentError):
            resolve_theta_deg(rect)

    def test_blunt_station_raises(self):
        # no genuine TE -> fail closed, not a guess.
        with self.assertRaises(AlignmentError):
            resolve_theta_deg(_blunt_ellipse_points())

    def test_threshold_is_the_documented_default(self):
        self.assertEqual(TE_ANGLE_DEGENERATE_THRESHOLD_DEG, 60.0)

    def test_rectangle_is_degenerate_not_a_spurious_diagonal_axis(self):
        # THE regression this threshold exists to prevent: a plain
        # rectangular coupon (every corner exactly 90 deg) is NOT an
        # aerofoil and must not be treated as having a chord axis at all
        # -- an initial 90.0 deg threshold let this slip through as
        # "usable" and would have computed a spurious diagonal axis angle
        # for every rectangular fixture in this repository.
        rect = [(0.0, 0.0), (20.0, 0.0), (20.0, 5.0), (0.0, 5.0)]
        self.assertIsNone(chord_line_angle_deg(rect))
        with self.assertRaises(AlignmentError):
            resolve_theta_deg(rect)


class ResolveWingThetaTests(unittest.TestCase):
    def test_consistent_stations_agree(self):
        pts = _naca_like_points()
        mid = rotate_points_deg(pts, 1.4)
        others = [rotate_points_deg(pts, 1.4) for _ in range(4)]
        theta, all_thetas, spread, excluded = resolve_wing_theta_deg(mid, others)
        self.assertLess(spread, 1e-6)
        self.assertAlmostEqual(theta, all_thetas[0], places=6)
        self.assertEqual(excluded, [])

    def test_mid_span_alone_is_the_operative_theta_even_with_real_washout(self):
        # Real washout: per-station theta drifts 0..5 deg across the span,
        # same order of magnitude measured on this project's own r150
        # reference fixture. The MID-SPAN station's own theta must be what
        # comes back, not an average/median blended across the drift.
        pts = _naca_like_points()
        mid = rotate_points_deg(pts, 2.5)
        others = [rotate_points_deg(pts, i) for i in (0.0, 1.0, 4.0, 5.0)]
        theta, all_thetas, spread, excluded = resolve_wing_theta_deg(mid, others)
        self.assertAlmostEqual(theta, resolve_theta_deg(mid), places=9)
        self.assertLessEqual(spread, 8.0)
        self.assertEqual(excluded, [])

    def test_excess_spread_raises(self):
        pts = _naca_like_points()
        mid = rotate_points_deg(pts, 0.0)
        others = [rotate_points_deg(pts, 20.0)]
        with self.assertRaises(AlignmentError):
            resolve_wing_theta_deg(mid, others)

    def test_degenerate_mid_span_fails_closed(self):
        # the AUTHORITATIVE station has no fallback.
        mid = _blunt_ellipse_points()
        others = [rotate_points_deg(_naca_like_points(), 1.0)]
        with self.assertRaises(AlignmentError):
            resolve_wing_theta_deg(mid, others)

    def test_degenerate_other_station_is_excluded_not_fatal(self):
        # a non-authoritative degenerate station (e.g. a root cut flat against
        # a fuselage) is excluded from the spread check, not treated as a
        # run-ending failure.
        pts = _naca_like_points()
        mid = rotate_points_deg(pts, 1.0)
        blunt = _blunt_ellipse_points()
        others = [rotate_points_deg(pts, 1.0), blunt, rotate_points_deg(pts, 1.5)]
        theta, all_thetas, spread, excluded = resolve_wing_theta_deg(mid, others)
        self.assertAlmostEqual(theta, resolve_theta_deg(mid), places=9)
        self.assertEqual(len(all_thetas), 3)  # mid + 2 usable others (blunt excluded)
        self.assertEqual([i for i, _ in excluded], [1])
        self.assertLessEqual(spread, 8.0)


class BluntTrailingEdgeFailsClosedTests(unittest.TestCase):
    """A real wing with a squared-off TE FAILS THE SLICE CLOSED; a structural
    coupon, which also has no readable chord axis, stays NON-fatal. The
    discriminator is hull vertex count alone.

    Why fatal is the right answer here: shipping such a wing unrotated is a
    measured silent wrong answer -- 15 degrees of misalignment gave 1495 layers
    ok, 0 failed, all structure built off-axis. A hull-diameter fallback axis
    was built and measured, and REJECTED: its error is camber- and
    truncation-correlated up to 2.18 deg, worse than the ~1.5 deg that
    disqualified ``min_area_rect_angle_deg``.
    """

    def test_a_blunt_TE_AEROFOIL_fails_the_slice_closed(self):
        pts = _truncated_te_naca_points()
        self.assertGreaterEqual(
            len(_convex_hull(pts)), AEROFOIL_MIN_HULL_VERTICES)
        with self.assertRaises(BluntTrailingEdgeError):
            resolve_theta_deg(pts)

    def test_the_message_tells_the_user_what_to_actually_do(self):
        # The message must not advise "rotate the model in the Orca GUI": a
        # squared-off TE reads the same sharpest-hull-vertex angle at every
        # rotation, so that advice is unactionable. The message says so
        # explicitly instead.
        with self.assertRaises(BluntTrailingEdgeError) as ctx:
            resolve_theta_deg(_truncated_te_naca_points())
        msg = str(ctx.exception)
        self.assertNotIn("Rotate the model", msg)
        self.assertIn("will NOT help", msg)

    def test_a_COUPON_stays_non_fatal(self):
        # The blast-radius property. ChordAxisUnavailableError is the
        # non-fatal branch; making a coupon fatal broke 27 integration
        # tests, because every fake_orca fixture is a rectangle.
        for w, h in ((40.0, 10.0), (100.0, 20.0), (10.0, 40.0), (20.0, 20.0)):
            for rot in (0.0, 30.0):
                pts = [(-w / 2, -h / 2), (w / 2, -h / 2),
                       (w / 2, h / 2), (-w / 2, h / 2)]
                if rot:
                    pts = rotate_points_deg(pts, rot)
                self.assertLess(
                    len(_convex_hull(pts)), AEROFOIL_MIN_HULL_VERTICES)
                with self.assertRaises(ChordAxisUnavailableError,
                                       msg="{}x{} @{}".format(w, h, rot)):
                    resolve_theta_deg(pts)

    def test_a_coupon_does_NOT_raise_the_fatal_blunt_TE_error(self):
        # Pins the branch, not just the base class: BluntTrailingEdgeError
        # is a subclass of AlignmentError, so asserting the base class
        # would not catch a coupon being routed down the fatal path.
        rect = [(-20.0, -5.0), (20.0, -5.0), (20.0, 5.0), (-20.0, 5.0)]
        with self.assertRaises(ChordAxisUnavailableError) as ctx:
            resolve_theta_deg(rect)
        self.assertNotIsInstance(ctx.exception, BluntTrailingEdgeError)

    def test_sharp_TE_sections_are_unaffected(self):
        pts = _naca_like_points()
        self.assertIsNotNone(chord_line_angle_deg(pts))
        self.assertAlmostEqual(
            resolve_theta_deg(pts), chord_line_angle_deg(pts), delta=1e-9)


# SHARP_BAND_LO/HI mark the synthetic elevon's TE-truncated/sharp boundary --
# stations at frac <= 0.85 are TE-truncated (blunt); stations at frac > 0.85
# keep a genuine sharp TE, mirroring a real elevon wing (root to 89% span
# truncated, outboard ~11% sharp), scaled down to the 0.02.0.98 sweep this
# module's sampler is searched over.
_SHARP_BAND_SPLIT_FRAC = 0.85


def _elevon_like_sampler(theta_inj=0.0, per_frac_extra_deg=None,
                          lo=0.02, hi=0.98, flip_frac=None):
    """Builds a ``station_sampler`` (``span_frac -> points at that fraction, or
    None``) standing in for a wing whose TE is truncated (blunt) from ``lo`` to
    ``_SHARP_BAND_SPLIT_FRAC`` and genuinely sharp from there to ``hi`` -- the
    same shape as a real elevon wing. The whole wing carries an
    injected rigid misalignment ``theta_inj`` (the danger case this test class
    exists to close: a blunt wing misaligned must NOT silently come out theta
    0).

    ``per_frac_extra_deg(frac) -> float``, if given, adds an ADDITIONAL
    per-station rotation on top of ``theta_inj`` -- used to build a sharp
    band whose own internal spread exceeds ``SHARP_BAND_MAX_SPREAD_DEG``.

    ``flip_frac``, if given, adds a 180 deg flip (LE/TE reversal) to the
    single BLUNT cross-check station at that fraction only -- used to
    exercise the leading-edge cross-check refusal in isolation.
    """
    def sampler(frac):
        if frac < lo or frac > hi:
            return None
        extra = per_frac_extra_deg(frac) if per_frac_extra_deg else 0.0
        flip = 180.0 if (flip_frac is not None and abs(frac - flip_frac) < 1e-9) else 0.0
        if frac <= _SHARP_BAND_SPLIT_FRAC:
            pts = _truncated_te_naca_points()
        else:
            pts = _naca_like_points()
        return rotate_points_deg(pts, theta_inj + extra + flip)
    return sampler


class SharpTrailingEdgeBandTests(unittest.TestCase):
    """An elevon wing can square off the TE at mid-span but keeps a genuine
    sharp TE outboard. ``station_sampler`` lets a caller substitute a
    contiguous band of sharp-TE stations for a blunt mid-span station, subject
    to guards -- see ``_resolve_sharp_band_theta``'s own docstring.
    """

    def _mid_and_other(self, sampler):
        mid = sampler(0.5)
        others = [sampler(f) for f in (0.1, 0.3, 0.7, 0.9)]
        others = [o for o in others if o is not None]
        return mid, others

    def test_resolves_to_injected_misalignment_0_15_29_deg(self):
        # The danger case this whole feature exists to close: a blunt wing
        # misaligned by a real angle must NOT come out theta 0.
        for theta_inj in (0.0, 15.0, 29.0):
            with self.subTest(theta_inj=theta_inj):
                sampler = _elevon_like_sampler(theta_inj=theta_inj)
                mid, others = self._mid_and_other(sampler)
                with self.assertRaises(BluntTrailingEdgeError):
                    resolve_theta_deg(mid)  # sanity: mid-span really is blunt
                provenance = {}
                theta, thetas, spread, excluded = resolve_wing_theta_deg(
                    mid, others, station_sampler=sampler, provenance=provenance)
                self.assertAlmostEqual(theta, theta_inj, delta=1e-6)
                self.assertEqual(provenance["theta_source"], "sharp_te_band")
                self.assertGreaterEqual(
                    len(provenance["band_fracs"]), SHARP_BAND_MIN_STATIONS)
                self.assertIsNotNone(provenance["blunt_crosscheck"])

    def test_fully_blunt_wing_still_raises(self):
        # No sharp station anywhere in the sweep -- the band search must
        # still fail closed, not guess.
        def all_blunt_sampler(frac):
            pts = _truncated_te_naca_points()
            return rotate_points_deg(pts, 10.0)

        mid = all_blunt_sampler(0.5)
        others = [all_blunt_sampler(f) for f in (0.1, 0.3, 0.7, 0.9)]
        with self.assertRaises(BluntTrailingEdgeError) as ctx:
            resolve_wing_theta_deg(
                mid, others, station_sampler=all_blunt_sampler)
        # Discriminates "no band anywhere" from the other guard failures
        # (short band, excess spread, cross-check disagreement) below.
        self.assertIn("No contiguous run", str(ctx.exception))

    def test_short_band_refuses(self):
        # Only 4 sharp stations (frac 0.95..0.98) -- below
        # SHARP_BAND_MIN_STATIONS (5).
        def short_band_sampler(frac):
            if frac < 0.02 or frac > 0.98:
                return None
            if frac >= 0.95:
                pts = _naca_like_points()
            else:
                pts = _truncated_te_naca_points()
            return rotate_points_deg(pts, 5.0)

        mid, others = self._mid_and_other(short_band_sampler)
        with self.assertRaises(BluntTrailingEdgeError) as ctx:
            resolve_wing_theta_deg(
                mid, others, station_sampler=short_band_sampler)
        # Discriminates "band found but too short" from "no band at all".
        self.assertIn("below the required minimum", str(ctx.exception))

    def test_band_with_excess_internal_spread_refuses(self):
        # The sharp band is long enough (13 stations), but its own
        # per-station theta drifts far more than SHARP_BAND_MAX_SPREAD_DEG
        # across the run -- not a single consistent placement angle.
        def drifting_extra(frac):
            return (frac - _SHARP_BAND_SPLIT_FRAC) * 100.0  # >> 1 deg/step

        sampler = _elevon_like_sampler(
            theta_inj=0.0, per_frac_extra_deg=drifting_extra)
        mid, others = self._mid_and_other(sampler)
        with self.assertRaises(BluntTrailingEdgeError) as ctx:
            resolve_wing_theta_deg(mid, others, station_sampler=sampler)
        # "spread threshold", not the bare word "spread" -- discriminates
        # this guard from the min-stations/no-band/cross-check failures,
        # none of whose messages contain "spread threshold".
        self.assertIn("spread threshold", str(ctx.exception))

    def test_blunt_station_le_disagreement_refuses(self):
        # A blunt cross-check station (frac 0.3) is flipped 180 deg
        # relative to the sharp band's own theta -- the independent
        # leading-edge-side signal must catch this even though the
        # hull-diameter axis (mod 180) would not.
        sampler = _elevon_like_sampler(theta_inj=0.0, flip_frac=0.3)
        mid, others = self._mid_and_other(sampler)
        with self.assertRaises(BluntTrailingEdgeError) as ctx:
            resolve_wing_theta_deg(mid, others, station_sampler=sampler)
        self.assertIn("leading-edge", str(ctx.exception))

    def test_blunt_station_hull_diameter_disagreement_refuses(self):
        # : exercise the hull-diameter guard (alignment.py ~793-800) in
        # ISOLATION from the leading-edge guard. The blunt cross-check stations
        # are rigidly rotated ~10 deg relative to the sharp band (not 180 deg,
        # so leading_edge_side still reads "min_x" after de-rotation -- the LE
        # guard does not fire), the band's own internal spread stays 0 (all
        # sharp stations share one rotation), so only the mod-180 hull-diameter
        # axis check can be the one that trips.
        offset_deg = 10.0

        def sampler(frac):
            if frac < 0.02 or frac > 0.98:
                return None
            if frac <= _SHARP_BAND_SPLIT_FRAC:
                pts = _truncated_te_naca_points()
                return rotate_points_deg(pts, offset_deg)
            pts = _naca_like_points()
            return rotate_points_deg(pts, 0.0)

        mid, others = self._mid_and_other(sampler)
        with self.assertRaises(BluntTrailingEdgeError) as ctx:
            resolve_wing_theta_deg(mid, others, station_sampler=sampler)
        self.assertIn("hull-diameter", str(ctx.exception))

    def test_coupon_path_is_byte_for_byte_unchanged(self):
        # design point 1: ChordAxisUnavailableError (the coupon path) must
        # never enter the sharp-band search, sampler or not.
        rect = [(-20.0, -5.0), (20.0, -5.0), (20.0, 5.0), (-20.0, 5.0)]

        def poison_sampler(frac):
            raise AssertionError(
                "station_sampler must never be called for a "
                "ChordAxisUnavailableError mid-span station")

        with self.assertRaises(ChordAxisUnavailableError):
            resolve_wing_theta_deg(rect, (), station_sampler=poison_sampler)

    def test_sampler_none_keeps_old_behaviour(self):
        sampler = _elevon_like_sampler(theta_inj=15.0)
        mid, others = self._mid_and_other(sampler)
        with self.assertRaises(BluntTrailingEdgeError):
            resolve_wing_theta_deg(mid, others)  # no station_sampler at all

    def test_cross_check_constants_are_the_measured_values(self):
        # Pins the cross-check constants, so a future edit to them is a
        # deliberate change, not a silent drift.
        self.assertEqual(SHARP_BAND_MIN_STATIONS, 5)
        self.assertEqual(SHARP_BAND_MAX_SPREAD_DEG, 1.0)
        self.assertEqual(BLUNT_CROSSCHECK_MAX_DEG, 5.0)


class DecideAlignmentTests(unittest.TestCase):
    """``decide_alignment`` is a pure extraction of ``capability.py``'s
    ``_check_mesh_alignment`` clauses 2-4, reused unchanged by
    ``tools/replay_host_sections.py`` for offline replay. These
    pin the four outcomes and the term-priority within each clause; the in-host
    integration tests in ``test_m18_mesh_transform.py`` pin the same behaviour
    through the real message-formatting call site."""

    ZERO = {
        "min_x": 0.0, "max_x": 0.0, "min_y": 0.0, "max_y": 0.0,
        "centroid_x": 0.0, "centroid_y": 0.0,
    }
    TOL = {k: 0.02 for k in ZERO}

    def test_all_within_tolerance_passes(self):
        deltas = dict(self.ZERO, min_x=0.01, centroid_y=0.019)
        result = decide_alignment(deltas, self.TOL, True, True)
        self.assertEqual(result, {"outcome": "pass"})

    def test_bbox_over_raises_unconditionally_regardless_of_top_or_skip_flag(self):
        deltas = dict(self.ZERO, max_y=0.03)
        for near_top in (True, False):
            for skip_enabled in (True, False):
                result = decide_alignment(deltas, self.TOL, near_top, skip_enabled)
                self.assertEqual(result["outcome"], "raise_bbox")
                self.assertEqual(result["worst_term"], "max_y")
                self.assertAlmostEqual(result["worst_delta"], 0.03)
                self.assertAlmostEqual(result["effective_tolerance"], 0.02)

    def test_bbox_over_picks_the_worst_of_multiple_over_terms(self):
        deltas = dict(self.ZERO, min_x=0.021, max_y=0.05)
        result = decide_alignment(deltas, self.TOL, True, True)
        self.assertEqual(result["outcome"], "raise_bbox")
        self.assertEqual(result["worst_term"], "max_y")

    def test_centroid_over_off_top_extremum_raises_regardless_of_skip_flag(self):
        deltas = dict(self.ZERO, centroid_x=0.03)
        for skip_enabled in (True, False):
            result = decide_alignment(deltas, self.TOL, False, skip_enabled)
            self.assertEqual(result["outcome"], "raise_centroid_off_top")
            self.assertEqual(result["worst_term"], "centroid_x")

    def test_centroid_over_near_top_with_skip_disabled_raises(self):
        deltas = dict(self.ZERO, centroid_y=0.03)
        result = decide_alignment(deltas, self.TOL, True, False)
        self.assertEqual(
            result["outcome"], "raise_centroid_near_top_skip_disabled")
        self.assertEqual(result["worst_term"], "centroid_y")

    def test_centroid_over_near_top_with_skip_enabled_skips(self):
        deltas = dict(self.ZERO, centroid_y=0.03)
        result = decide_alignment(deltas, self.TOL, True, True)
        self.assertEqual(result["outcome"], "skip")
        self.assertEqual(result["worst_term"], "centroid_y")
        self.assertAlmostEqual(result["worst_delta"], 0.03)

    def test_effective_tolerance_per_term_not_a_single_shared_value(self):
        # sensitivity widening is per-term: a term whose OWN effective
        # tolerance is widened past its delta must not fire, even though
        # another term's un-widened tolerance is exceeded.
        deltas = dict(self.ZERO, min_x=0.03, max_x=0.03)
        effective = dict(self.TOL, min_x=0.05)  # widened past its own delta
        result = decide_alignment(deltas, effective, True, True)
        self.assertEqual(result["outcome"], "raise_bbox")
        self.assertEqual(result["worst_term"], "max_x")
        self.assertAlmostEqual(result["effective_tolerance"], 0.02)


if __name__ == "__main__":
    unittest.main()


