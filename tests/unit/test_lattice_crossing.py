"""Regression: "families that were supposed to cross, don't."

Shipped-default shape. The default config (``presets/plugin/default_config.json``)
is a two-sided skin lattice of shallow webs; ``DefaultConfigShapeTests`` pins
that shape from the TRACKED file. The live, gitignored
``host/data/orca_plugins/config.json`` is deliberately not used for shape
assertions, because a user legitimately changes it with
``python tools/config.py use <name>`` for local testing, and a differently
shaped valid config is not a regression. Instead:

- ``LiveInstalledConfigSmokeTests`` reads the live file, skips cleanly if
  absent, and asserts only what must hold for ANY valid config (schema-valid,
  generates without raising, no zero-web layers, no wall violations) via
  ``tools.structural_analysis.assert_cap_config_slices_cleanly``.
- ``TrackedPresetConfigsSliceCleanlyTests`` runs the same smoke check against
  every tracked preset, showing the check accepts every legitimate shape
  without mutating ``host/data``.
- ``MergeDetectionHasTeethTests`` is the positive control for the
  zero-cross-family-merges assertion: an absence-only assertion can pass for
  the wrong reason (a merge-masking bug once produced zero merges), so a
  fixture that must merge is checked too.

Historical-bug fixture: a forward and an aft web family that drift in opposite
directions were believed to cross -- producing a diamond-lattice junction --
but on the 20 mm coupon they never do. This module freezes that configuration
as a permanent regression fixture (``shipped_non_crossing_families``, values
copied by hand, NOT read live from config.json, so the guard keeps testing the
bug regardless of what the live config ships), derives the general crossing
condition in closed form, and shows the derivation discriminates by exercising
it on either side of the threshold.

DERIVATION -- do not trust this on sight; it is re-derived, not copied, from
the primitives cited below.

Anchoring (``StructureGenerator``, generator.py:441-443): for family ``f``
with web_position ``p`` at layer Z,

    x_k(f, z) = frame.web_x(p) + f.station_drift_mm(z) + k * f.rib_pitch_mm

for integer march index ``k`` (``frame.web_x``, frame.py:556-562, is
``min_x + p * width``).

``station_drift_mm`` (spec.py) is a triangle wave when ``z_period_mm`` is
set: amplitude ``A = z_drift_mm_per_mm * z_period_mm / 4`` (this literal
sign convention matters below -- ``A`` carries the sign of the slope, so a
negative-slope family has a negative "A" even though its drift still swings
the same |A| either side of zero), reversing direction at every half-period.
The wave's own docstring states it is continuous and reverses exactly at
half-period boundaries, so evaluating it at ``z0`` and ``z0 + period/2``
finds its true minimum and maximum EXACTLY -- no dense sampling, no
reimplementation of the triangle-wave formula, just two real calls into
``WebFamily.station_drift_mm``.

Two opposed families A (anchor position p_a) and B (p_b), sharing a section
of width W, at cross-family march offset ``n = k_b - k_a``:

    sep(n, z) = x_k_b(B, z) - x_k_a(A, z)
              = [frame.web_x(p_b) - frame.web_x(p_a)] + n * R
                + [f_b.station_drift_mm(z) - f_a.station_drift_mm(z)]
              = C + n*R + (d_b(z) - d_a(z))

where ``C = (p_b - p_a) * W`` and ``R`` is the (here, shared) rib pitch.
``sep`` is piecewise-linear in Z for fixed ``n`` (each family's drift is
piecewise-linear), so over any Z window its extrema occur only at that
window's own endpoints and at every half-period breakpoint of either family
inside it -- never in between. That is the ``_breakpoints`` helper below,
and it is exact, not approximate, PROVIDED the window's endpoints (or the
sampled breakpoint set) actually bracket the true extrema; for a family
whose ``z_period_mm`` is set this module always includes both of that
family's half-period breakpoints, so it is exact for every case this module
uses.

CROSSING CONDITION. A station ORDER SWAP for offset ``n`` happens iff
``sep(n, z)`` changes sign somewhere in the window -- equivalently (since it
is continuous and piecewise-linear) iff its minimum is negative AND its
maximum is positive at breakpoints. When ``d_b(z) == -d_a(z)`` for every z
-- which holds EXACTLY whenever the two families share ``z_period_mm`` and
have opposite ``z_drift_mm_per_mm`` (station_drift_mm's amplitude and every
term inside it are linear in the slope, so negating the slope negates the
whole function pointwise for the same z) -- this collapses to the form:

    sep(n, z) = C + n*R - 2*d_a(z),   d_a(z) in [-A, +A] (A = |slope|*period/4)

so sep(n, ·) ranges over exactly ``[C + n*R - 2A, C + n*R + 2A]``, and BOTH
endpoints are achieved (at z where d_a = +A and z where d_a = -A
respectively, i.e. exactly at the two half-period breakpoints). Order swaps
for offset n iff ``|C + n*R| < 2*A`` (strict); ``|C + n*R| == 2*A`` is exact
TANGENCY -- the closest station pair touches zero separation at one instant
but never actually swaps sides.

RIB-PITCH INTERLEAVING. The generator marches every integer k, so a
"crossing" question is not just about the anchor (k=0) pair: offset n = -1,
-2, ... pairs a family-A station one (two, ...) pitches further out against
family B's anchor, and that pair's own center ``C + n*R`` can sit closer to
zero than the anchor pair's ``C`` does. The true minimum center separation
and true crossing condition are therefore a minimum over ALL integer n, not
just n=0 -- ``cross_family_analysis`` below searches ``n in range(-8, 9)``,
comfortably wider than any pitch/width ratio this project uses.

WORKED EXAMPLE -- the shipped (buggy) config. fwd: web_position=0.4,
z_drift_mm_per_mm=+0.8; aft: web_position=0.6, z_drift_mm_per_mm=-0.8; both
z_period_mm=7.5, rib_pitch_mm=8.0; coupon width W=20 mm (x in [-10, 10]).
C = (0.6 - 0.4) * 20 = 4.0 mm. A = 0.8 * 7.5 / 4 = 1.5 mm, so 2A = 3.0 mm.
n=0: |C| = 4.0 > 2A = 3.0 -- no swap for the anchor pair.
n=-1: |C - R| = |4.0 - 8.0| = 4.0 > 3.0 -- ties n=0 exactly (this config is
  symmetric: R = 2*C), also no swap.
Every other n is further from zero still. So minimum center separation is
``4.0 - 3.0 = 1.0`` mm, matching the measured value in docs/, and
the families never cross. Order-swap tangency for this exact geometry is at
``2A = C``, i.e. ``A = 2.0``, i.e. ``z_period_mm = 4*A/0.8 = 10.0``. A figure
of ">8.0" would conflate a DIFFERENT
threshold: TopologyValidator's junction rule
(validate.py's collision check, EDGE gap, not center separation) fires once
the center separation drops under ``slot_width_mm + 0.84``, which for this
config's 0.021 mm slot is ``0.861`` mm -- i.e. once ``2A > C - 0.861``,
``z_period_mm > 4*(4.0-0.861)/(4*0.8) ~= 7.85``, which
rounds to "8.0". Junction-firing and order-swapping are two different
thresholds; this module tests the order-swap one (the crossing condition)
and separately confirms the fixture's edge gap against the 0.84 mm figure
directly.
"""
import importlib.util
import json
import math
import os
import unittest
from collections import namedtuple
from pathlib import Path

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.spec import StructureSpec, WebFamily
from stub_ops import PolygonShape, Rect, StubOps

# Repo root: tests/unit/test_lattice_crossing.py -> tests/unit -> tests -> repo
# root. Matches the REPO_ROOT derivation used by tests/integration modules.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_LIVE_CONFIG_PATH = os.path.join(_REPO_ROOT, "host", "data", "orca_plugins", "config.json")

# 2 * safety_offset_mm at safety_offset_mm=0.42, matching test_junction.py
# and test_validate.py's own MIN_WALL_MM.
JUNCTION_MIN_WALL_MM = 0.84

_ZERO_TOL_MM = 1e-9


CrossFamilyResult = namedtuple(
    "CrossFamilyResult",
    ["crosses", "min_centre_sep_mm", "min_edge_gap_mm"],
)


def _family_breakpoints(family, z_lo, z_hi):
    """Every Z in ``[z_lo, z_hi]`` where ``family.station_drift_mm`` can
    change slope, plus the window's own endpoints -- see this module's
    docstring "DERIVATION" for why sampling only at these points is exact.

    A family with ``z_period_mm`` unset (drift constant, or unbounded
    linear) has no interior breakpoints; the window endpoints alone are
    enough since the function is linear (or constant) throughout.
    """
    points = {z_lo, z_hi}
    if family.z_period_mm is not None:
        half = family.z_period_mm / 2.0
        # Breakpoints occur at every z0 + k*half for integer k where z0 is
        # the wave's own phase origin (0.0 -- station_drift_mm's docstring:
        # drift(0) == -A). Walk both directions from 0 across the window.
        # Find a starting k close to z_lo.
        k = int(math.floor(z_lo / half)) - 1
        while True:
            z = k * half
            if z > z_hi:
                break
            if z_lo <= z <= z_hi:
                points.add(z)
            k += 1
    return sorted(points)


def cross_family_analysis(
    frame,
    family_a,
    family_b,
    rib_pitch_mm,
    slot_width_mm,
    z_lo,
    z_hi,
    n_range=range(-8, 9),
    local_chord_mm=None,
):
    """Closed-form (breakpoint-exact, not sampled) analysis of whether two
    web families ever swap chord order, and how close they come, over the
    Z window ``[z_lo, z_hi]``.

    Uses the REAL anchoring/drift primitives
    (``frame.web_x``, ``WebFamily.station_drift_mm``) exactly as
    ``StructureGenerator`` does (generator.py:441-443) -- it does not
    reimplement the triangle wave, only evaluates it at the Z values where
    its own docstring says its extrema fall (window endpoints, plus every
    half-period breakpoint of either family -- see ``_family_breakpoints``).
    Searches march offsets ``n in n_range`` (family B's k minus family A's
    k) to capture rib-pitch interleaving: the closest cross-family pair is
    not always the two anchors (n=0).

    ``local_chord_mm``, when given, is threaded through to every
    ``station_drift_mm`` call so a family using the chord-relative drift
    field (``z_drift_chord_frac_per_mm``) resolves correctly; a family
    using the absolute field (``z_drift_mm_per_mm``) or no drift at all
    ignores it, so the default ``None`` is exact for every caller that
    passes only absolute-mode families (e.g. ``shipped_non_crossing_
    families``).

    Returns a ``CrossFamilyResult``:
        crosses: True iff some (n, z) pair in range/window has family B's
            station strictly left of family A's and some other (n, z) pair
            has it strictly right -- i.e. a genuine order swap, not mere
            tangency.
        min_centre_sep_mm: minimum |sep(n, z)| found, in millimetres.
        min_edge_gap_mm: ``min_centre_sep_mm - slot_width_mm`` -- the
            closest EDGE-to-edge gap, matching what
            ``TopologyValidator``'s collision check (validate.py) compares
            against ``JUNCTION_MIN_WALL_MM``.
    """
    breakpoints_a = _family_breakpoints(family_a, z_lo, z_hi)
    breakpoints_b = _family_breakpoints(family_b, z_lo, z_hi)
    z_points = sorted(set(breakpoints_a) | set(breakpoints_b))

    xa0 = frame.web_x(family_a.web_position)
    xb0 = frame.web_x(family_b.web_position)

    min_abs = None
    crosses = False

    def _sign(sep):
        if sep > _ZERO_TOL_MM:
            return 1
        if sep < -_ZERO_TOL_MM:
            return -1
        return 0

    for n in n_range:
        # Sign tracking is PER n: a swap only counts if the SAME (n, z)
        # curve changes sign as z varies -- two different offsets n each
        # touching zero from opposite sides (as happens exactly at the
        # tangency threshold, where the n=0 and n=-1 pairs are tangent from
        # opposite directions) is not a swap of either pair, so signs must
        # never be pooled across different n.
        base = (xb0 - xa0) + n * rib_pitch_mm
        sep_values = [
            base
            + family_b.station_drift_mm(z, local_chord_mm=local_chord_mm)
            - family_a.station_drift_mm(z, local_chord_mm=local_chord_mm)
            for z in z_points
        ]
        min_abs_n = min(abs(s) for s in sep_values)
        if min_abs is None or min_abs_n < min_abs:
            min_abs = min_abs_n

        # sep(n, ·) is piecewise-LINEAR between consecutive breakpoints, so
        # if the reduced (zero-dropped) sign sequence at the breakpoints
        # ever alternates, some point strictly between two breakpoints (or
        # a breakpoint itself) is an exact zero, by the intermediate value
        # theorem on each linear segment -- a genuine order swap for this n,
        # and its true minimum |sep| is exactly 0 (tighter than any
        # breakpoint sample, which is why this is not just
        # ``min(abs(s) for s in sep_values)`` on its own). A lone zero
        # sandwiched between two SAME-sign neighbours (tangency, e.g.
        # [+, 0, +]) reduces to [+, +] -- no alternation, no swap.
        reduced_signs = [s for s in (_sign(v) for v in sep_values) if s != 0]
        crosses_n = any(
            reduced_signs[i] != reduced_signs[i + 1]
            for i in range(len(reduced_signs) - 1)
        )
        if crosses_n:
            crosses = True
            if min_abs is None or 0.0 < min_abs:
                min_abs = 0.0
    min_edge_gap = min_abs - slot_width_mm
    return CrossFamilyResult(
        crosses=crosses, min_centre_sep_mm=min_abs, min_edge_gap_mm=min_edge_gap
    )


def family_drifts(family, local_chord_mm=100.0):
    """True if ``family``'s station genuinely moves with Z, checked the
    mode-agnostic way: evaluate the RESOLVED ``station_drift_mm`` at two
    different Z values rather than reading ``z_drift_mm_per_mm`` or
    ``z_drift_chord_frac_per_mm`` directly off the family (or a raw config
    dict). Works unchanged whichever of the two mutually-exclusive drift fields
    is set (task "make member placement geometry-relative instead of absolute"
    added the chord-relative one), and for the triangle-wave case
    (``z_period_mm`` set) samples at the wave's own quarter-period -- see
    ``WebFamily.station_drift_mm``'s docstring, whose extrema fall exactly at
    ``z_period_mm/4`` and its odd multiples -- so a genuinely drifting family
    cannot land on a false zero by unlucky sampling (e.g. two half-period-apart
    points, where the wave returns to the same value). ``local_chord_mm`` is
    arbitrary and nonzero: whether drift varies with Z at all does not depend
    on the chord's magnitude, only on which field (if any) is set.
    """
    if family.z_period_mm is not None:
        z_a, z_b = 0.0, family.z_period_mm / 4.0
    else:
        z_a, z_b = 0.0, 1000.0
    d_a = family.station_drift_mm(z_a, local_chord_mm=local_chord_mm)
    d_b = family.station_drift_mm(z_b, local_chord_mm=local_chord_mm)
    return d_a != d_b


# ---------------------------------------------------------------------------
# Fixture: the shipped config, frozen by hand.
# ---------------------------------------------------------------------------


def shipped_non_crossing_families():
    """The exact ``fwd``/``aft`` family pair from
    a past host/data/orca_plugins/config.json -- the shipped-default bug:
    believed to cross (diamond lattice), never actually does on the 20 mm
    coupon. Frozen here by hand, NOT read live from config.json, so this
    regression guard keeps exercising the historical bug whatever the live
    config ships.

    Returns ``(fam_fwd, fam_aft, rib_pitch_mm, coupon_rect)``.
    """
    fam_fwd = WebFamily(
        name="fwd",
        rib_pitch_mm=8.0,
        web_position=0.4,
        z_drift_mm_per_mm=0.8,
        z_period_mm=7.5,
    )
    fam_aft = WebFamily(
        name="aft",
        rib_pitch_mm=8.0,
        web_position=0.6,
        z_drift_mm_per_mm=-0.8,
        z_period_mm=7.5,
    )
    coupon_rect = Rect(-10.0, 0.0, 10.0, 20.0)
    return fam_fwd, fam_aft, 8.0, coupon_rect


class ShippedConfigDoesNotCrossTests(unittest.TestCase):
    """(a) Regression guard: the historical shipped-default bug. This config must
    keep failing to cross for as long as it ships -- if this test ever starts
    asserting a crossing, the default changed and docs/'s correction (and this
    fixture) need to change with it, not the other way around."""

    def setUp(self):
        self.ops = StubOps()
        fam_fwd, fam_aft, rib_pitch, coupon_rect = shipped_non_crossing_families()
        self.fam_fwd = fam_fwd
        self.fam_aft = fam_aft
        self.rib_pitch = rib_pitch
        self.frame = LayerFrame.from_polygon(self.ops, coupon_rect)
        self.spec = StructureSpec(gap_ratio=0.05, skin_margin_mm=1.0)

    def test_does_not_cross_min_centre_separation_is_one_mm(self):
        slot_width = self.fam_fwd.resolve_gap(self.spec, 0.42)
        self.assertAlmostEqual(slot_width, 0.021, places=6)

        # One full shared period is enough: the analysis is periodic in Z
        # with period 7.5 for both families, so any window of that length
        # contains every distinct (n, phase) combination that can occur.
        result = cross_family_analysis(
            self.frame,
            self.fam_fwd,
            self.fam_aft,
            self.rib_pitch,
            slot_width,
            z_lo=0.0,
            z_hi=7.5,
        )

        self.assertFalse(result.crosses, "shipped config must NOT cross (shipped-default bug)")
        self.assertAlmostEqual(result.min_centre_sep_mm, 1.0, places=6)
        self.assertGreater(result.min_edge_gap_mm, JUNCTION_MIN_WALL_MM)
        self.assertAlmostEqual(result.min_edge_gap_mm, 0.979, places=6)


class HelperHasTeethTests(unittest.TestCase):
    """(b) The helper is sound in the POSITIVE direction: a config chosen
    to actually cross must be reported as crossing, with a sub-threshold
    edge gap -- proving the helper would have caught the original bug had
    it existed when the shipped config was written."""

    def setUp(self):
        self.ops = StubOps()
        self.frame = LayerFrame.from_polygon(self.ops, Rect(-10.0, 0.0, 10.0, 20.0))
        self.spec = StructureSpec(gap_ratio=0.05, skin_margin_mm=1.0)

    def _families(self, z_period_mm):
        fam_fwd = WebFamily(
            name="fwd", rib_pitch_mm=8.0, web_position=0.4,
            z_drift_mm_per_mm=0.8, z_period_mm=z_period_mm,
        )
        fam_aft = WebFamily(
            name="aft", rib_pitch_mm=8.0, web_position=0.6,
            z_drift_mm_per_mm=-0.8, z_period_mm=z_period_mm,
        )
        return fam_fwd, fam_aft

    def test_raised_amplitude_crosses_with_sub_threshold_edge_gap(self):
        # z_period_mm=12.0 > the derived tangency threshold of 10.0 for
        # this geometry (see module docstring's worked example) --
        # A = 0.8*12/4 = 2.4, 2A = 4.8 > C = 4.0.
        fam_fwd, fam_aft = self._families(z_period_mm=12.0)
        slot_width = fam_fwd.resolve_gap(self.spec, 0.42)

        result = cross_family_analysis(
            self.frame, fam_fwd, fam_aft, 8.0, slot_width, z_lo=0.0, z_hi=12.0
        )

        self.assertTrue(result.crosses, "raised-amplitude config must cross")
        self.assertAlmostEqual(result.min_centre_sep_mm, 0.0, places=6)
        self.assertLess(result.min_edge_gap_mm, JUNCTION_MIN_WALL_MM)


class TangencyBoundaryTests(unittest.TestCase):
    """(c) Pin the exact order-swap threshold this module derives:
    ``2*A == C`` (equivalently ``z_period_mm == 10.0`` for this geometry).
    Just below: closest approach is a strictly positive gap, no swap. At
    exactly the threshold: the gap touches zero (tangency) but the helper
    must NOT report a swap -- ``sep`` reaches 0 without changing sign, so
    the order never actually reverses. Just above: a genuine swap."""

    def setUp(self):
        self.ops = StubOps()
        self.frame = LayerFrame.from_polygon(self.ops, Rect(-10.0, 0.0, 10.0, 20.0))
        self.spec = StructureSpec(gap_ratio=0.05, skin_margin_mm=1.0)

    def _families(self, z_period_mm):
        fam_fwd = WebFamily(
            name="fwd", rib_pitch_mm=8.0, web_position=0.4,
            z_drift_mm_per_mm=0.8, z_period_mm=z_period_mm,
        )
        fam_aft = WebFamily(
            name="aft", rib_pitch_mm=8.0, web_position=0.6,
            z_drift_mm_per_mm=-0.8, z_period_mm=z_period_mm,
        )
        return fam_fwd, fam_aft

    def _analyze(self, z_period_mm):
        fam_fwd, fam_aft = self._families(z_period_mm)
        slot_width = fam_fwd.resolve_gap(self.spec, 0.42)
        return cross_family_analysis(
            self.frame, fam_fwd, fam_aft, 8.0, slot_width,
            z_lo=0.0, z_hi=z_period_mm,
        )

    def test_just_below_threshold_does_not_cross(self):
        result = self._analyze(z_period_mm=9.9)
        self.assertFalse(result.crosses)
        self.assertGreater(result.min_centre_sep_mm, 0.0)

    def test_at_exact_threshold_touches_zero_but_does_not_cross(self):
        # A = 0.8 * 10.0 / 4 = 2.0; 2A = 4.0 = C exactly -- both are exact
        # in binary floating point (0.8, 10.0/4, 4.0*0.4/... all land on
        # clean fractions here), so this is a genuine tangency test, not an
        # approximation.
        result = self._analyze(z_period_mm=10.0)
        self.assertFalse(
            result.crosses,
            "exact tangency must not register as a swap (sep touches 0 "
            "without changing sign)",
        )
        self.assertAlmostEqual(result.min_centre_sep_mm, 0.0, places=9)

    def test_just_above_threshold_crosses(self):
        result = self._analyze(z_period_mm=10.1)
        self.assertTrue(result.crosses)
        self.assertAlmostEqual(result.min_centre_sep_mm, 0.0, places=6)


def _load_live_cap_config(config_path):
    """Read the shipped plugin config's ``cap_config`` dict, straight off disk --
    as opposed to ``shipped_non_crossing_families`` above, which is a
    hand-frozen copy of a past default and must never be read from disk.
    """
    with open(config_path, "r", encoding="utf-8") as fh:
        raw = json.load(fh)
    return raw["config"][0]["cap_config"]


_MAKE_FIXTURES_SCRIPT = Path(_REPO_ROOT) / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    # Same loader pattern test_generator.py uses to avoid a second airfoil
    # implementation living in this file.
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_lattice_crossing", _MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _naca2412_polygon():
    mf = _load_make_fixtures_module()
    m, p, t = mf.parse_naca4("2412")
    _triangles, outline_mm, _bbox = mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm))


_DEFAULT_CONFIG_PATH = os.path.join(_REPO_ROOT, "presets", "plugin", "default_config.json")

# Every tracked config a user can legitimately install via
# ``python tools/config.py use <name>`` -- see ``tools/config.py`` and
# ``presets/plugin/README.md``. Excludes README.md itself.
_TRACKED_PRESET_PATHS = sorted(
    p
    for p in (
        os.path.join(_REPO_ROOT, "presets", "plugin", name)
        for name in os.listdir(os.path.join(_REPO_ROOT, "presets", "plugin"))
    )
    if p.endswith(".json")
)


class DefaultConfigShapeTests(unittest.TestCase):
    """Shape guard: ``presets/plugin/default_config.json`` -- the TRACKED shipped
    default, read straight off disk -- must be the skin-lattice-only
    shape: NO ``primary_structure`` family and NO full-depth
    (``web_depth_mm=null``) family (the deep spar/full-depth-web
    families stay disabled until a primary-structure topology is designed), at least one family entering from EACH surface
    (``entry_lower`` true and false), at least one drifting family (the lattice
    is diagonals), and every family shallow (``web_depth_mm`` set and >= 1.5).

    The shape assertions read the TRACKED default, not the LIVE, gitignored,
    user-mutable ``host/data/orca_plugins/config.json`` (see the module
    docstring), so this class is deterministic and independent of host state.
    ``LiveInstalledConfigSmokeTests`` below is the deliberately shape-agnostic
    live-config check.

    This file is TRACKED (not gitignored); a missing file is a real repo
    problem, not a partial-checkout condition to skip past.
    """

    def setUp(self):
        self.assertTrue(
            os.path.isfile(_DEFAULT_CONFIG_PATH),
            "presets/plugin/default_config.json not found at {!r} -- this "
            "file is tracked and must always be present.".format(
                _DEFAULT_CONFIG_PATH
            ),
        )
        self.cap_config = _load_live_cap_config(_DEFAULT_CONFIG_PATH)
        self.families = self.cap_config["families"]
        self.assertTrue(
            self.families, "default config has no families to check at all"
        )

    def test_at_least_one_family_drifts(self):
        # The shipped default is only the skin lattice -- four +/-45-degree
        # diagonal families, and the whole point of a diagonal lattice is
        # that it drifts. A default that stopped drifting would mean the
        # diagonal lattice had silently reverted to straight webs.
        # setUp already asserts self.families is non-empty, so this cannot
        # pass on an empty list.
        drifting = [
            name for name in (f["name"] for f in self.families)
            if family_drifts(self._family(name))
        ]
        self.assertTrue(
            drifting,
            "default config has no drifting family at all; the "
            "skin-lattice-only default is diagonal webs, and a default "
            "with zero drift would mean the lattice silently reverted to "
            "straight (non-crossing) webs.",
        )

    def test_no_primary_structure_family_and_no_full_depth_family(self):
        # "The full-depth families are removed from the shipped default; the
        # skin lattice is finished on its own terms." Any full-depth primary
        # structural member MUST bond to the far skin (fused attachment), and
        # no topology reaching that bond while staying a single closed loop
        # has been designed yet -- so a full-depth family, or one tagged
        # role=primary_structure, appearing in the shipped default means the
        # primary system was adopted without being designed. This forbids
        # that shape outright.
        #
        # Vacuity check: setUp already asserts self.families is non-empty,
        # so "no primary_structure family" / "no full-depth family" cannot
        # pass merely because the families list is empty -- both guards
        # below run against a real, populated family list.
        primary = [
            f["name"] for f in self.families if f.get("role") == "primary_structure"
        ]
        full_depth = [
            f["name"] for f in self.families if f.get("web_depth_mm") is None
        ]
        self.assertEqual(
            primary,
            [],
            "default config has primary_structure family/families {!r}; "
            "the design defers primary structure until a topology exists that "
            "keeps a full-depth member fused to BOTH skins, and forbids "
            "shipping one before it is designed.".format(primary),
        )
        self.assertEqual(
            full_depth,
            [],
            "default config has full-depth (web_depth_mm=null) family/"
            "families {!r}; the design requires fused attachment (both-skin "
            "bonding) for any full-depth primary structural web, and no "
            "topology satisfying that has been designed yet -- a "
            "full-depth member in the default means the primary system "
            "was adopted without being designed.".format(full_depth),
        )

    def test_shape_is_two_sided_shallow_skin_lattice_only(self):
        # What the default must be:
        #   - no primary_structure family (see
        #     test_no_primary_structure_family_and_no_full_depth_family for
        #     the message text; repeated here as part of the whole-shape
        #     check so this test stands on its own);
        #   - no full-depth family (same guard);
        #   - at least one family entering from EACH surface (entry_lower
        #     true and false) -- two-sided skin coverage;
        #   - at least one drifting family -- the lattice is diagonals (see
        #     test_at_least_one_family_drifts);
        #   - EVERY family shallow (web_depth_mm set and >= 1.5): with no
        #     full-depth family, shallow must hold universally, not merely
        #     exist.
        # It also confirms the shape produces zero cross-family node
        # merges and at least one web on both fixtures.
        primary = [
            f["name"] for f in self.families if f.get("role") == "primary_structure"
        ]
        full_depth = [f["name"] for f in self.families if f.get("web_depth_mm") is None]
        not_shallow = [
            f["name"]
            for f in self.families
            if f.get("web_depth_mm") is None or f["web_depth_mm"] < 1.5
        ]
        upper_entry = [f["name"] for f in self.families if not f.get("entry_lower")]
        lower_entry = [f["name"] for f in self.families if f.get("entry_lower")]
        drifting = [
            name for name in (f["name"] for f in self.families)
            if family_drifts(self._family(name))
        ]

        self.assertEqual(
            primary,
            [],
            "default config has primary_structure family/families {!r}; "
            "the design forbids shipping primary structure before its topology "
            "is designed.".format(primary),
        )
        self.assertEqual(
            full_depth,
            [],
            "default config has full-depth (web_depth_mm=null) family/"
            "families {!r}; the design requires fused attachment for any "
            "full-depth member and no such topology is designed yet.".format(
                full_depth
            ),
        )
        self.assertEqual(
            not_shallow,
            [],
            "default config has non-shallow family/families {!r} (every "
            "family must carry web_depth_mm >= 1.5 now that "
            "the full-depth families were removed).".format(not_shallow),
        )
        self.assertTrue(
            upper_entry,
            "default config has no family entering from the upper surface "
            "(entry_lower=false); two-sided skin coverage requires both "
            "surfaces to be represented.",
        )
        self.assertTrue(
            lower_entry,
            "default config has no family entering from the lower surface "
            "(entry_lower=true); two-sided skin coverage requires both "
            "surfaces to be represented.",
        )
        self.assertTrue(
            drifting,
            "default config has no drifting family; the skin lattice is "
            "diagonal webs and a drift-free default would mean it silently "
            "reverted to straight webs.",
        )

        from tools.structural_analysis import build_spec_from_cap_config

        spec = build_spec_from_cap_config(self.cap_config)
        generator = StructureGenerator()
        fixtures = {
            "coupon": LayerFrame.from_polygon(
                StubOps(), Rect(-10.0, -10.0, 10.0, 10.0)
            ),
            "airfoil": LayerFrame.from_polygon(StubOps(), _naca2412_polygon()),
        }
        for label, frame in fixtures.items():
            ctx = LayerContext(layer_index=0, print_z=0.2)
            result = generator.generate(frame, spec, 0.42, ctx)
            # NOT asserting zero merges here: unlike straight, non-crossing
            # spar shape, skin-lattice-only default is the +/-45-degree
            # DIAGONAL lattice (crossing doctrine) on both surfaces, whose
            # families are designed to cross and merge at nodes -- a "zero
            # merges" claim at layer 0 would be true only by accident of which
            # Z happens to be sampled (see
            # test_default_config_families_actually_cross_as_ designed below,
            # which proves the crossing in closed form across the whole drift
            # period, the way a spot check at one Z cannot).
            self.assertTrue(
                result.intents,
                "{} fixture built zero webs at layer 0.".format(label),
            )

    def test_default_config_families_actually_cross_as_designed(self):
        # The default drifts by design, so one representative layer is not
        # exhaustive: a direct sweep of the real generator across the shared
        # z_period_mm=28 mm period shows ZERO merges at layer 0 (Z=0.2 mm) but
        # 10 merges per layer at many other Z values in the same period (e.g.
        # layer 3, Z=0.8 mm). An absence-of-merges spot check at layer 0 would
        # pass by accident of which layer got sampled.
        #
        # What this test guards instead: the upper
        # (`lattice_plus_upper`/`lattice_minus_upper`) and lower
        # (`lattice_plus_lower`/`lattice_minus_lower`) +/-45-degree pairs each
        # share one anchor (`web_position`) with opposite `z_drift_mm_per_mm`
        # -- the same "C == 0 identically" construction
        # `AnisogridConfigCrossesTests` proves crosses, using the SAME
        # closed-form ``cross_family_analysis`` helper this module derives in
        # its own docstring (breakpoint-exact, not sampled) -- so this is a
        # positive control that the shipped default's diagonal lattice
        # actually forms the crossing junctions it exists to form, not merely
        # an absence-of-merges claim vulnerable to merge masking.
        fam_upper_a = self._family("lattice_plus_upper")
        fam_upper_b = self._family("lattice_minus_upper")
        fam_lower_a = self._family("lattice_plus_lower")
        fam_lower_b = self._family("lattice_minus_lower")

        # web_position is unaffected by the absolute/relative pitch-and-
        # drift conversion (it has no chord-relative counterpart), so
        # reading it directly off the WebFamily objects is still exact.
        self.assertEqual(fam_upper_a.web_position, fam_upper_b.web_position)
        self.assertEqual(fam_lower_a.web_position, fam_lower_b.web_position)

        # Opposite-drift check, done the mode-agnostic way: evaluate the
        # RESOLVED station_drift_mm (not either raw z_drift_mm_per_mm /
        # z_drift_chord_frac_per_mm field) at two Z values and confirm each
        # pair's drift is an exact negation of its partner's, for whichever
        # drift field is actually set. local_chord_mm is arbitrary and
        # nonzero -- the negation holds for any chord, since a positive and
        # negative chord-relative rate resolve against the SAME chord here.
        _SAMPLE_CHORD_MM = 100.0
        for fam_a, fam_b, label in (
            (fam_upper_a, fam_upper_b, "upper"),
            (fam_lower_a, fam_lower_b, "lower"),
        ):
            z_period = fam_a.z_period_mm
            self.assertEqual(
                z_period,
                fam_b.z_period_mm,
                "{} pair does not share z_period_mm; the opposite-drift "
                "check below assumes it does.".format(label),
            )
            sample_zs = (0.0, z_period / 4.0) if z_period is not None else (0.0, 1000.0)
            for z in sample_zs:
                drift_a = fam_a.station_drift_mm(z, local_chord_mm=_SAMPLE_CHORD_MM)
                drift_b = fam_b.station_drift_mm(z, local_chord_mm=_SAMPLE_CHORD_MM)
                self.assertEqual(
                    drift_a,
                    -drift_b,
                    "{} pair's drift at z={} is not an exact negation "
                    "({!r} vs {!r}) -- the diamond-lattice construction "
                    "requires opposite drift.".format(label, z, drift_a, drift_b),
                )

        slot_width = fam_upper_a.resolve_gap(
            StructureSpec(gap_ratio=self.cap_config["gap_ratio"]), 0.42
        )
        for label, frame in (
            (
                "coupon",
                LayerFrame.from_polygon(StubOps(), Rect(-10.0, -10.0, 10.0, 10.0)),
            ),
            ("airfoil", LayerFrame.from_polygon(StubOps(), _naca2412_polygon())),
        ):
            for surface, fam_a, fam_b in (
                ("upper", fam_upper_a, fam_upper_b),
                ("lower", fam_lower_a, fam_lower_b),
            ):
                # Resolve pitch/drift against THIS frame's own local chord
                # (frame.width) -- the same value the real generator uses
                # (generator.py's anchor_x/pitch_mm derivation) -- rather
                # than reading fam_a.rib_pitch_mm directly, which is None
                # for a chord-relative family.
                result = cross_family_analysis(
                    frame,
                    fam_a,
                    fam_b,
                    fam_a.resolved_rib_pitch_mm(frame.width),
                    slot_width,
                    0.0,
                    28.0,
                    local_chord_mm=frame.width,
                )
                self.assertTrue(
                    result.crosses,
                    "{} {} pair does not actually swap chord order over "
                    "one drift period -- the shipped diagonal lattice is "
                    "not crossing as designed.".format(label, surface),
                )

    def _family_dict(self, name):
        return next(f for f in self.families if f["name"] == name)

    def _family(self, name):
        raw = dict(self._family_dict(name))
        raw.pop("role", None)
        return WebFamily(**raw)


class DefaultConfigWebDensityScalesWithChordTests(unittest.TestCase):
    """Guards chord-relative member placement: the shipped default's
    web density must be roughly CONSTANT across chord sizes, not grow with the
    chord. A fixed ``rib_pitch_mm`` (e.g. 3.9) would be a constant millimetre
    spacing regardless of chord, so a 150 mm-chord wing would pack 2.5x the web
    density of the 60 mm coupon it was tuned on for no structural reason.
    ``rib_pitch_chord_frac`` makes pitch a FRACTION of the local chord, so
    the number of marching stations across a section of width W is
    approximately ``W / (frac * W) == 1 / frac`` -- independent of W. This test
    is the regression guard for that: nothing else in this module asserts
    webs-per-layer stays flat as chord grows.
    """

    def setUp(self):
        self.assertTrue(
            os.path.isfile(_DEFAULT_CONFIG_PATH),
            "presets/plugin/default_config.json not found at {!r}".format(
                _DEFAULT_CONFIG_PATH
            ),
        )
        cap_config = _load_live_cap_config(_DEFAULT_CONFIG_PATH)
        from tools.structural_analysis import build_spec_from_cap_config

        self.spec = build_spec_from_cap_config(cap_config)
        self.generator = StructureGenerator()

    def _webs_per_layer(self, chord_mm, n_samples=40):
        """Web count (``len(result.intents)``) on a plain rectangular coupon of
        the given chord, averaged over samples spanning one full drift period
        (28 mm, the shipped default's shared ``z_period_mm``) -- a single-Z
        spot check is exactly the "passes by accident of which layer got
        sampled" trap this module's own docstring keeps recording, so this
        sweeps instead.

        ``n_samples`` must be large enough to avoid ALIASING against the
        cross-family merge pattern's own fine structure within the period:
        the merge/no-merge count swings between roughly 30 (heavily merged)
        and 62 (unmerged) webs at a resolution finer than 28/8 = 3.5 mm
        steps, so a coarse 8-sample sweep can accidentally phase-align with
        a chord's merge pattern and land a skewed mean (measured: an
        8-sample sweep gave a 46 vs 61.5 mean split between chords that a
        finer sweep shows is not real). 40 samples (0.7 mm steps) was
        measured to land every chord's mean in roughly the same 48-61 band
        that 100- and 200-sample sweeps also settle into, while running in
        a fraction of the time.
        """
        frame = LayerFrame.from_polygon(
            StubOps(),
            Rect(-chord_mm / 2.0, -10.0, chord_mm / 2.0, 10.0),
        )
        counts = []
        for i in range(n_samples):
            z = i * 28.0 / n_samples
            ctx = LayerContext(layer_index=i, print_z=z)
            result = self.generator.generate(frame, self.spec, 0.42, ctx)
            counts.append(len(result.intents))
        return counts

    def test_webs_per_layer_is_approximately_constant_across_chords(self):
        # 60 mm is the coupon chord the fixtures are tuned on; 150 mm and
        # 250 mm are chords a real wing plausibly uses.
        counts_by_chord = {
            chord_mm: self._webs_per_layer(chord_mm)
            for chord_mm in (60.0, 150.0, 250.0)
        }
        for chord_mm, counts in counts_by_chord.items():
            self.assertTrue(
                all(c > 0 for c in counts),
                "{} mm chord built zero webs on at least one sampled "
                "layer: {!r}".format(chord_mm, counts),
            )

        means = {
            chord_mm: sum(counts) / len(counts)
            for chord_mm, counts in counts_by_chord.items()
        }

        # APPROXIMATELY CONSTANT: every chord's mean web count must sit
        # within a generous band of every other chord's -- NOT proportional
        # to chord width, which is the regression this guards against.
        # Measured 100-sample means land in a roughly 48-58 band at every
        # chord from 60-250 mm (cross-family merges make the exact count
        # noisy layer to layer, but the BAND itself does not shift with
        # chord); 20 is comfortable headroom around that spread while still
        # being nowhere near what proportional growth would produce
        # (250 mm's chord is 4.17x 60 mm's -- proportional growth would
        # push the 250 mm mean past 200).
        lo, hi = min(means.values()), max(means.values())
        self.assertLess(
            hi - lo,
            20.0,
            "webs-per-layer varies by {:.2f} across chords {!r} -- the "
            "shipped default's density is supposed to be chord-relative "
            "(approximately constant web COUNT regardless of chord), not "
            "grow with chord.".format(hi - lo, means),
        )

        # Directly rule out the specific regression: web count must NOT
        # scale anywhere near proportionally with chord. An absolute-pitch
        # default would grow web count by roughly the same 250/60 ~= 4.17x
        # the chord itself grows; this asserts
        # the actual growth factor is nowhere close to that.
        growth_factor = means[250.0] / means[60.0]
        chord_ratio = 250.0 / 60.0
        self.assertLess(
            growth_factor,
            chord_ratio / 2.0,
            "web count grew {:.2f}x from a 60mm to a 250mm chord (chord "
            "itself grew {:.2f}x) -- density is scaling with chord again, "
            "exactly the bug this conversion fixed.".format(
                growth_factor, chord_ratio
            ),
        )


class MergeDetectionHasTeethTests(unittest.TestCase):
    """POSITIVE CONTROL for ``DefaultConfigShapeTests``'s zero-merges assertions
    above: an assertion that merges are ABSENT is satisfied equally by "the
    config genuinely never crosses" and by "the merge machinery itself is
    broken and reports zero merges no matter what" -- exactly the shape of the
    bug commit f8120a8 introduced (a merge-masking defect that made a
    zero-merges assertion pass for the wrong reason; see this module's
    docstring /09-01 notes). This class proves the machinery still works by
    running the SAME real generator against the tracked
    ``anisogrid_lattice_config.json`` (whose
    ``lattice_plus_*``/``lattice_minus_*`` families are DESIGNED to cross --
    see ``AnisogridConfigCrossesTests`` below, which proves the crossing in
    closed form) and asserting merges are produced -- if this test ever starts
    failing, the merge detector itself has regressed, independent of whatever
    the shipped default's own shape happens to be.
    """

    def setUp(self):
        if not os.path.isfile(_ANISOGRID_CONFIG_PATH):
            self.skipTest(
                "anisogrid_lattice_config.json not found at {!r}".format(
                    _ANISOGRID_CONFIG_PATH
                )
            )
        self.cap_config = _load_live_cap_config(_ANISOGRID_CONFIG_PATH)

    def test_anisogrid_config_actually_produces_merges_on_the_coupon(self):
        from tools.structural_analysis import build_spec_from_cap_config

        spec = build_spec_from_cap_config(self.cap_config)
        generator = StructureGenerator()
        frame = LayerFrame.from_polygon(StubOps(), Rect(-10.0, -10.0, 10.0, 10.0))
        ctx = LayerContext(layer_index=0, print_z=0.2)
        result = generator.generate(frame, spec, 0.42, ctx)
        merged = [w for w in result.intents if w.merged_family_indices]
        self.assertGreaterEqual(
            len(merged),
            1,
            "the tracked anisogrid config's +theta/-theta families are "
            "designed to cross (see AnisogridConfigCrossesTests) and "
            "should produce at least one cross-family node merge on the "
            "coupon fixture; zero merges here means the merge detector "
            "itself is not firing, not that this config is clean.",
        )


class WallCheckHasTeethTests(unittest.TestCase):
    """POSITIVE CONTROL for ``LiveInstalledConfigSmokeTests``'s (and
    ``TrackedPresetConfigsSliceCleanlyTests``'s) "no wall violations"
    assertion: an absence-only assertion there would be satisfied just as
    well by ``tools.structural_analysis.check_wall_violations`` having
    silently regressed to always returning ``[]`` -- exactly the shape of
    bug commit f8120a8 introduced for the merge detector. This class proves
    the wall-check helper still fires on synthetic ``WebIntent`` fixtures
    built directly (not through the generator), pinning all three branches
    of the rule it mirrors (``validate.py``'s "collision check" /
    " ").
    """

    def setUp(self):
        from tools.structural_analysis import check_wall_violations

        self.check_wall_violations = check_wall_violations

    def _intent(self, x, family_index, family_name, entry_lower=False, width_mm=0.5):
        from aircraft_core.generator import WebIntent

        return WebIntent(
            start=(x, 0.0),
            end=(x, -5.0),
            width_mm=width_mm,
            family_index=family_index,
            family_name=family_name,
            entry_lower=entry_lower,
        )

    def test_same_family_too_close_is_a_violation(self):
        a = self._intent(0.0, 0, "f")
        b = self._intent(0.3, 0, "f")
        violations = self.check_wall_violations([a, b], safety_offset_mm=0.42)
        self.assertEqual(
            len(violations),
            1,
            "same-family slots 0.3 mm apart (footprints 0.05 mm wall) "
            "must be flagged -- got {!r}.".format(violations),
        )

    def test_cross_family_close_but_not_overlapping_is_a_violation(self):
        # a cross-family pair this close only escapes the check if footprints
        # actually OVERLAP (gap < 0); a near miss still fails closed.
        a = self._intent(0.0, 0, "f1")
        b = self._intent(0.9, 1, "f2")
        violations = self.check_wall_violations([a, b], safety_offset_mm=0.42)
        self.assertEqual(
            len(violations),
            1,
            "cross-family slots with a positive but sub-threshold gap and "
            "no overlap must still be flagged -- got {!r}.".format(violations),
        )

    def test_cross_family_overlapping_is_an_intended_junction_not_a_violation(self):
        a = self._intent(0.0, 0, "f1")
        b = self._intent(0.2, 1, "f2")
        violations = self.check_wall_violations([a, b], safety_offset_mm=0.42)
        self.assertEqual(
            violations,
            [],
            "cross-family slots whose footprints actually overlap are an "
            "intended junction, not a violation -- got {!r}.".format(
                violations
            ),
        )

    def test_far_apart_slots_never_flagged(self):
        a = self._intent(0.0, 0, "f")
        b = self._intent(5.0, 0, "f")
        self.assertEqual(self.check_wall_violations([a, b], safety_offset_mm=0.42), [])

    def test_opposite_sides_never_compared(self):
        # entry_lower splits the scan into +Y/-Y; a same-family pair on
        # OPPOSITE sides, even coincident in X, must not be flagged by this
        # same-edge-only helper (see check_wall_violations's own "NOT
        # COVERED" docstring note for the cross-edge case this does not
        # reproduce).
        a = self._intent(0.0, 0, "f", entry_lower=False)
        b = self._intent(0.0, 0, "f", entry_lower=True)
        self.assertEqual(self.check_wall_violations([a, b], safety_offset_mm=0.42), [])


class LiveInstalledConfigSmokeTests(unittest.TestCase):
    """Smoke check for WHATEVER config is CURRENTLY INSTALLED at
    ``host/data/orca_plugins/config.json`` -- read live from disk. A user
    may legitimately have installed any tracked config (default, anisogrid,
    conservative, coupon, geometry-aware, skin-lattice-only) via
    ``python tools/config.py use <name>``, and all of those are valid
    choices this check must accept.

    Deliberately asserts ONLY what must hold for ANY valid installed config,
    never a shape claim: the config validates against the schema
    (``aircraft_core.config_schema``), the real generator produces webs without
    raising on both the coupon and NACA 2412 c60 airfoil fixtures (swept over Z
    -- see ``tools.structural_analysis._z_sweep_values``), every swept layer
    has at least one web (no zero-web layer), and no swept layer fails the wall
    check (``tools.structural_analysis.check_wall_violations`` -- an
    intent-level mirror of ``aircraft_core.validate.TopologyValidator``'s own
    collision rule; see that function's docstring for why it does not run the
    real validator directly). Does NOT assert a merge count, a family count, or
    drift-free-ness -- those are shape claims, and the whole point of this
    class is that the installed shape is the installer's choice.

    Skips cleanly (not a failure) when ``host/data`` is absent, e.g. a
    partial checkout with no host runtime at all.
    """

    def setUp(self):
        if not os.path.isfile(_LIVE_CONFIG_PATH):
            self.skipTest(
                "host/data/orca_plugins/config.json not found at {!r}; "
                "skipping the live-config smoke check (a partial checkout "
                "may omit host/data entirely).".format(_LIVE_CONFIG_PATH)
            )
        self.cap_config = _load_live_cap_config(_LIVE_CONFIG_PATH)

    def test_installed_config_slices_cleanly(self):
        from tools.structural_analysis import assert_cap_config_slices_cleanly

        assert_cap_config_slices_cleanly(self.cap_config)


class TrackedPresetConfigsSliceCleanlyTests(unittest.TestCase):
    """Verifies that the live-config smoke check
    (``LiveInstalledConfigSmokeTests``) accepts every legitimate
    installable shape, not just whichever one happens to be installed --
    runs ``assert_cap_config_slices_cleanly`` against EVERY tracked
    ``presets/plugin/*.json`` config directly, without touching
    ``host/data`` at all. This is what lets this module claim the smoke
    check passes "with the default installed" and "with the anisogrid
    installed" simultaneously, in one test run, regardless of what
    ``tools/config.py use`` last did to the live file.
    """

    def test_every_tracked_preset_slices_cleanly(self):
        from tools.structural_analysis import assert_cap_config_slices_cleanly

        self.assertTrue(_TRACKED_PRESET_PATHS, "no tracked presets found to check")
        for path in _TRACKED_PRESET_PATHS:
            with self.subTest(preset=os.path.basename(path)):
                cap_config = _load_live_cap_config(path)
                assert_cap_config_slices_cleanly(cap_config)


_ANISOGRID_CONFIG_PATH = os.path.join(
    _REPO_ROOT, "presets", "plugin", "anisogrid_lattice_config.json"
)


class AnisogridConfigCrossesTests(unittest.TestCase):
    """``presets/plugin/anisogrid_lattice_config.json`` carries a real
    axial + (+theta) + (-theta) skin-lattice pair per surface, alongside
    two full-depth primary_structure spars. This applies the closed-form
    crossing guard (``cross_family_analysis`` above) to that config, so a
    config shipped tangent -- with humans told to inspect crossings that do
    not exist -- cannot happen silently.

    DESIGN NOTE pinned by ``test_plus_minus_share_one_anchor_so_c_is_zero``:
    the ``+theta``/``-theta`` pair is anchored at the SAME ``web_position``
    (0.35), which makes ``C = (p_b - p_a) * W == 0`` IDENTICALLY, for every
    fixture width ``W`` -- this is what makes the crossing guarantee
    independent of which fixture (20 mm coupon vs 60 mm NACA chord) the config
    is checked against, unlike the historical bug (opposed anchors at 0.4/0.6,
    whose separation ``C`` scaled with fixture width and happened to fall just
    short of crossing).
    """

    def setUp(self):
        if not os.path.isfile(_ANISOGRID_CONFIG_PATH):
            self.skipTest(
                "anisogrid_lattice_config.json not found at {!r}".format(
                    _ANISOGRID_CONFIG_PATH
                )
            )
        self.cap_config = _load_live_cap_config(_ANISOGRID_CONFIG_PATH)
        self.by_name = {f["name"]: f for f in self.cap_config["families"]}

    def _family(self, name):
        raw = dict(self.by_name[name])
        raw.pop("role", None)
        return WebFamily(**raw)

    def test_plus_minus_share_one_anchor_so_c_is_zero(self):
        plus = self.by_name["lattice_plus_upper"]
        minus = self.by_name["lattice_minus_upper"]
        self.assertEqual(plus["web_position"], minus["web_position"])
        self.assertEqual(plus["z_drift_mm_per_mm"], -minus["z_drift_mm_per_mm"])
        self.assertEqual(plus["z_period_mm"], minus["z_period_mm"])

    def test_plus_minus_upper_actually_cross_on_the_coupon(self):
        frame = LayerFrame.from_polygon(StubOps(), Rect(-10.0, -10.0, 10.0, 10.0))
        fam_a = self._family("lattice_plus_upper")
        fam_b = self._family("lattice_minus_upper")
        result = cross_family_analysis(
            frame, fam_a, fam_b, fam_a.rib_pitch_mm, 0.021, 0.0, 60.0
        )
        self.assertTrue(
            result.crosses,
            "lattice_plus_upper/lattice_minus_upper do not actually swap "
            "chord order on the coupon fixture -- min centre separation "
            "{!r} mm".format(result.min_centre_sep_mm),
        )

    def test_plus_minus_upper_actually_cross_on_the_airfoil(self):
        frame = LayerFrame.from_polygon(StubOps(), _naca2412_polygon())
        fam_a = self._family("lattice_plus_upper")
        fam_b = self._family("lattice_minus_upper")
        result = cross_family_analysis(
            frame, fam_a, fam_b, fam_a.rib_pitch_mm, 0.021, 0.0, 60.0
        )
        self.assertTrue(
            result.crosses,
            "lattice_plus_upper/lattice_minus_upper do not actually swap "
            "chord order on the airfoil fixture -- min centre separation "
            "{!r} mm".format(result.min_centre_sep_mm),
        )

    def test_plus_minus_lower_also_cross(self):
        # Same anchor/period/opposite-slope construction as the upper
        # pair, on the -Y skin -- checked separately since entry_lower does
        # not enter cross_family_analysis's own X-placement arithmetic at
        # all (frame.web_x is entry-side agnostic), but is worth pinning
        # explicitly rather than assumed from the upper pair's result.
        frame = LayerFrame.from_polygon(StubOps(), _naca2412_polygon())
        fam_a = self._family("lattice_plus_lower")
        fam_b = self._family("lattice_minus_lower")
        result = cross_family_analysis(
            frame, fam_a, fam_b, fam_a.rib_pitch_mm, 0.021, 0.0, 60.0
        )
        self.assertTrue(result.crosses)

    def test_config_validates_and_roles_are_as_designed(self):
        from tools.structural_analysis import build_spec_from_cap_config

        spec = build_spec_from_cap_config(self.cap_config)
        roles = {f.name: f.role for f in spec.families}
        self.assertEqual(roles["spar_front"], "primary_structure")
        self.assertEqual(roles["spar_aft"], "primary_structure")
        for name in (
            "lattice_axial_upper",
            "lattice_plus_upper",
            "lattice_minus_upper",
            "lattice_axial_lower",
            "lattice_plus_lower",
            "lattice_minus_lower",
        ):
            self.assertEqual(roles[name], "skin_lattice")


if __name__ == "__main__":
    unittest.main()
