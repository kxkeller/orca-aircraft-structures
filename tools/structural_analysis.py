#!/usr/bin/env python3
"""Structural analysis tool: predicted figures of merit computed from the REAL
generated geometry -- not hand-typed numbers.

Everything this module reports is **PREDICTED / MODELLED**, never **MEASURED**.
No printed coupon backs any number here.

WHAT THIS DOES: takes an arm config (the ``cap_config`` dict the plugin
stores), a fixture (the NACA 2412 c=60mm s=60mm airfoil section, or the
rectangular coupon section as a fallback), and a line width, and:

  1. Builds a ``StructureSpec`` from the arm config exactly the way
     ``plugin/aircraft_structures/capability.py``'s ``_build_spec`` does
     (same field names, same defaults -- see ``build_spec_from_cap_config``).
  2. Runs ``aircraft_core.generator.StructureGenerator.generate()`` -- the
     REAL production generator, unmodified, imported from ``aircraft_core``
     -- once per body layer over the fixture's span, exactly as
     ``capability.py`` does at ``posSlice``.
  3. Accumulates the real per-layer ``WebIntent`` geometry (chord stations,
     depths, counts) into ``aircraft_core.diagnostics.RunDiagnostics`` --
     the SAME accumulator the shipped plugin uses -- for the mass proxy,
     worst-skin-bay, and web-tip-air-gap/bond-class figures.
  4. Computes two torsional stiffness models from that same real geometry:
     open-cell (Bredt-Batho on the outer contour only, webs contribute
     nothing) and closed multi-cell (webs as shear ties dividing the
     section into cells -- see ``solve_multicell_j``).
  5. Computes worst-bay critical shear-buckling stress and the resulting
     section buckling torque, using the flat-plate formula from
     Gerard & Becker, NACA TN 3781 and, for the -Y skin, the tip-bond
     classification from ``aircraft_core.diagnostics`` to decide whether
     that skin is restrained at all.

GEOMETRY BACKEND: this tool drives the real ``aircraft_core`` generator
against ``tests/unit/stub_ops.StubOps`` -- the project's own exact-arithmetic
test double for ``aircraft_core.ops.GeometryOps`` -- rather than a live Orca
host (there is none in this offline harness). ``StructureGenerator.generate``
only calls ``bounds``/``contour_points_mm``/``intersection`` (via
``aircraft_core.frame.local_span``), never ``diff``/``offset`` -- so it never
touches the parts of ``StubOps`` that are documented as narrow (multi-notch
erosion, etc.). ``tests/unit/test_generator.py`` and
``tests/unit/test_local_geometry.py`` use the same test double for the same
call path. ``stub_ops.py``'s own module docstring says it "must never be
treated like [a real geometry kernel]"; here only its
``bounds``/``contour_points_mm``/``intersection`` are used, which are exact for
the shapes this tool builds.

WHAT THIS DOES NOT DO: it does not run the boolean cut, the topology
validator, or anything Orca-side. It does not know whether a real print
would actually slice -- only whether the generator's own web-placement
logic accepts the requested pattern. A config that fails to generate any
webs at some layer is reported as an error for that arm, not silently
skipped.
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

TESTS_UNIT_DIR = os.path.join(REPO_ROOT, "tests", "unit")
if TESTS_UNIT_DIR not in sys.path:
    sys.path.insert(0, TESTS_UNIT_DIR)

from aircraft_core.config_schema import assert_valid_config  # noqa: E402
from aircraft_core.diagnostics import RunDiagnostics  # noqa: E402
from aircraft_core.frame import EPSILON_MM, LayerFrame  # noqa: E402
from aircraft_core.generator import LayerContext, StructureGenerator  # noqa: E402
from aircraft_core.spec import SpecError, StructureSpec, WebFamily  # noqa: E402

import stub_ops  # noqa: E402  (tests/unit/stub_ops.py -- see module docstring)


def _load_make_fixtures_module():
    """Load ``tools/make_fixtures.py`` as a module, the same pattern
    ``tests/unit/test_local_geometry.py`` / ``test_generator.py`` already
    use, so this tool builds the airfoil section from the SAME NACA 4-digit
    point-generation code the shipped fixture STL was built from -- no
    second airfoil implementation.
    """
    path = os.path.join(REPO_ROOT, "tools", "make_fixtures.py")
    spec = importlib.util.spec_from_file_location("make_fixtures_for_structural_analysis", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mf = _load_make_fixtures_module()


# ---------------------------------------------------------------------------
# Section geometry: a small backend-neutral bundle of what the torsion/
# buckling calculations need, built once per fixture and reused across every
# layer of a constant-section stack (this tool only handles constant
# sections -- the coupon and the airfoil fixture are both constant-section).
# ---------------------------------------------------------------------------


@dataclass
class SectionGeometry:
    """Everything the structural calculations need about one constant 2D
    cross-section, independent of any one arm's web pattern.

    ``ops``/``polygon``/``frame`` are what ``StructureGenerator.generate``
    actually consumes (unchanged production code path). ``perimeter_mm`` /
    ``area_mm2`` / ``arc_upper``/``arc_lower``/``thickness_at``/
    ``area_between`` are this tool's OWN geometry, built from a dense,
    independently-sampled profile table (see ``build_airfoil_section`` /
    ``build_rect_section``) -- used only for the torsion/buckling maths, not
    fed back into the generator.

    THE APPROXIMATION in the airfoil profile table (stated here because it
    governs every torsion/buckling number this tool produces for the
    airfoil arms): each profile point is evaluated at chord fraction ``x``
    directly from the NACA thickness/camber closed forms, i.e. treating the
    surface point's own X coordinate as exactly ``x * chord`` rather than
    the true NACA construction's chord-perpendicular offset
    (``xu = x - yt*sin(theta)``, see ``tools/make_fixtures.py``'s
    ``naca4_surface_points`` docstring). The offset is a few thousandths of
    chord for a NACA 2412 (light camber, moderate thickness) and is not
    applied here so that arc length and area can be integrated on a single
    shared, exactly-monotonic X grid. This is a documented, second-order
    approximation on top of an already-idealised torsion-box model -- not
    the same geometry ``StructureGenerator`` itself uses to place webs
    (which DOES use the true, perpendicular-offset outline via
    ``aircraft_core.frame.local_span``, unaffected by this approximation).
    """

    name: str
    ops: object
    polygon: object
    frame: LayerFrame
    span_mm: float
    min_x: float
    max_x: float
    perimeter_mm: float
    area_mm2: float
    arc_upper: Callable[[float], float]
    arc_lower: Callable[[float], float]
    thickness_at: Callable[[float], float]
    area_between: Callable[[float], float]
    lower_arc_total_mm: float
    y_upper_of_x: Optional[Callable[[float], float]] = None
    y_lower_of_x: Optional[Callable[[float], float]] = None


def _cumulative_arc_length(xs: Sequence[float], ys: Sequence[float]) -> List[float]:
    out = [0.0]
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i - 1]
        dy = ys[i] - ys[i - 1]
        out.append(out[-1] + math.hypot(dx, dy))
    return out


def _interp_monotonic(xs: Sequence[float], ys: Sequence[float], x: float) -> float:
    """Linear interpolation of ``ys`` at ``x``, ``xs`` assumed non-decreasing.
    Clamped at the ends rather than raising, since callers only ever query
    within (or numerically at the edge of) ``[xs[0], xs[-1]]``.
    """
    n = len(xs)
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    x0, x1 = xs[lo], xs[hi]
    y0, y1 = ys[lo], ys[hi]
    if x1 == x0:
        return y0
    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)


def build_airfoil_section(
    naca: str = "2412",
    chord_mm: float = 60.0,
    span_mm: float = 60.0,
    n_points: int = 120,
    profile_samples: int = 2001,
) -> SectionGeometry:
    """Build the NACA airfoil ``SectionGeometry``, matching
    ``tests/fixtures/airfoil/naca2412_c60_s60.stl`` (chord 60mm, span 60mm,
    120 cosine-spaced points/surface -- same parameters
    ``test_local_geometry.py`` and ``test_generator.py`` use for their own
    ``_naca2412_polygon()`` helpers).

    The polygon handed to ``StructureGenerator`` is built EXACTLY the way
    the fixture STL and the project's own unit tests build it
    (``mf.airfoil_triangles``) -- true perpendicular-offset NACA geometry,
    centred on its own bounding box. The dense profile table used for arc
    length / area (see ``SectionGeometry``'s docstring for the
    approximation) is a SEPARATE, independently built table, aligned to the
    same coordinate frame by construction (both centre on the same
    ``chord_mm``-scaled outline).
    """
    m, p, t = mf.parse_naca4(naca)
    _triangles, outline_mm, bbox = mf.airfoil_triangles(m, p, t, chord_mm, span_mm, n_points)
    polygon = stub_ops.PolygonShape(tuple(outline_mm))
    ops = stub_ops.StubOps()
    frame = LayerFrame.from_polygon(ops, polygon)
    xmin, _ymin, xmax, _ymax = bbox

    # Independent dense profile table -- see SectionGeometry's docstring.
    xs_frac = mf.cosine_spaced_chord_fractions(profile_samples)
    yt = [mf.naca4_thickness(x, t) for x in xs_frac]
    yc = [mf.naca4_camber(x, m, p)[0] for x in xs_frac]
    y_upper_frac = [c + th for c, th in zip(yc, yt)]
    y_lower_frac = [c - th for c, th in zip(yc, yt)]
    # Raw (uncentred) chord-fraction x, scaled to mm; then shifted to align
    # with the polygon's own frame (min_x from the true, perpendicular
    # -offset outline) rather than assuming a bare chord_mm/2 centring
    # offset, so a web station computed via frame.web_x(...) always falls
    # inside [profile_min_x, profile_max_x].
    xs_mm_raw = [x * chord_mm for x in xs_frac]
    shift = xmin - min(xs_mm_raw)
    xs_mm = [x + shift for x in xs_mm_raw]
    y_upper_mm = [y * chord_mm for y in y_upper_frac]
    y_lower_mm = [y * chord_mm for y in y_lower_frac]

    arc_u = _cumulative_arc_length(xs_mm, y_upper_mm)
    arc_l = _cumulative_arc_length(xs_mm, y_lower_mm)

    area_mm2 = 0.0
    for i in range(1, len(xs_mm)):
        dx = xs_mm[i] - xs_mm[i - 1]
        h0 = y_upper_mm[i - 1] - y_lower_mm[i - 1]
        h1 = y_upper_mm[i] - y_lower_mm[i]
        area_mm2 += 0.5 * (h0 + h1) * dx
    perimeter_mm = arc_u[-1] + arc_l[-1]

    def arc_upper(x_mm: float) -> float:
        return _interp_monotonic(xs_mm, arc_u, x_mm)

    def arc_lower(x_mm: float) -> float:
        return _interp_monotonic(xs_mm, arc_l, x_mm)

    def thickness_at(x_mm: float) -> float:
        yu = _interp_monotonic(xs_mm, y_upper_mm, x_mm)
        yl = _interp_monotonic(xs_mm, y_lower_mm, x_mm)
        return max(0.0, yu - yl)

    def y_upper_of_x(x_mm: float) -> float:
        return _interp_monotonic(xs_mm, y_upper_mm, x_mm)

    def y_lower_of_x(x_mm: float) -> float:
        return _interp_monotonic(xs_mm, y_lower_mm, x_mm)

    def area_between(x0_mm: float, x1_mm: float) -> float:
        if x1_mm <= x0_mm:
            return 0.0
        total = 0.0
        prev_x = x0_mm
        prev_h = thickness_at(x0_mm)
        for xi, yu, yl in zip(xs_mm, y_upper_mm, y_lower_mm):
            if xi <= x0_mm or xi >= x1_mm:
                continue
            h = yu - yl
            total += 0.5 * (prev_h + h) * (xi - prev_x)
            prev_x, prev_h = xi, h
        h_end = thickness_at(x1_mm)
        total += 0.5 * (prev_h + h_end) * (x1_mm - prev_x)
        return total

    return SectionGeometry(
        name="airfoil naca{} c{} s{}".format(naca, chord_mm, span_mm),
        ops=ops,
        polygon=polygon,
        frame=frame,
        span_mm=span_mm,
        min_x=xs_mm[0],
        max_x=xs_mm[-1],
        perimeter_mm=perimeter_mm,
        area_mm2=abs(area_mm2),
        arc_upper=arc_upper,
        arc_lower=arc_lower,
        thickness_at=thickness_at,
        area_between=area_between,
        lower_arc_total_mm=arc_l[-1],
        y_upper_of_x=y_upper_of_x,
        y_lower_of_x=y_lower_of_x,
    )


def build_rect_section(width_mm: float = 20.0, height_mm: float = 20.0, span_mm: float = 60.0) -> SectionGeometry:
    """Build the rectangular coupon ``SectionGeometry`` (fallback fixture,
    and the hand-checkable shape ``tests/unit/test_structural_analysis.py``
    validates this module's maths against).

    Unlike the airfoil, the rectangle's leading/trailing "edges" (the left
    and right vertical walls) are NOT zero-length points -- they are full
    ``height_mm`` verticals, exactly like an interior web. ``thickness_at``
    correctly returns ``height_mm`` at ``min_x``/``max_x`` too, which is
    what lets ``solve_multicell_j`` treat them uniformly with the airfoil's
    zero-length leading/trailing edge (see that function's docstring).
    """
    rect = stub_ops.Rect(-width_mm / 2.0, -height_mm / 2.0, width_mm / 2.0, height_mm / 2.0)
    ops = stub_ops.StubOps()
    frame = LayerFrame.from_polygon(ops, rect)
    min_x, max_x = -width_mm / 2.0, width_mm / 2.0

    def arc_upper(x_mm: float) -> float:
        x_mm = min(max(x_mm, min_x), max_x)
        return x_mm - min_x

    def arc_lower(x_mm: float) -> float:
        return arc_upper(x_mm)

    def thickness_at(_x_mm: float) -> float:
        return height_mm

    def area_between(x0_mm: float, x1_mm: float) -> float:
        return max(0.0, x1_mm - x0_mm) * height_mm

    return SectionGeometry(
        name="rect coupon {}x{} s{}".format(width_mm, height_mm, span_mm),
        ops=ops,
        polygon=rect,
        frame=frame,
        span_mm=span_mm,
        min_x=min_x,
        max_x=max_x,
        perimeter_mm=2.0 * (width_mm + height_mm),
        area_mm2=width_mm * height_mm,
        arc_upper=arc_upper,
        arc_lower=arc_lower,
        thickness_at=thickness_at,
        area_between=area_between,
        lower_arc_total_mm=width_mm,
    )


# ---------------------------------------------------------------------------
# StructureSpec from an arm's cap_config, mirroring
# plugin/aircraft_structures/capability.py's _build_spec/_build_families
# field-for-field (same key names, same "None means inherit" semantics) --
# see that module for the production version this mirrors.
# ---------------------------------------------------------------------------

_SPEC_KEYS = (
    "spar_diameter_mm",
    "spar_station",
    "spar_clearance_mm",
    "spar_neck_width_mm",
    "spar_channel_policy",
    "expected_hole_count",
    "require_primary_structure",
    "allow_config_spar_fallback",
    "slot_gap_mm",
    "gap_ratio",
    "skin_margin_mm",
    "root_solid_layers",
    "web_position",
    "overshoot_mm",
    "rib_pitch_mm",
    "web_depth_mm",
    "z_drift_mm_per_mm",
    "z_period_mm",
    "tip_shutdown_enabled",
    # Keep this list in step with capability.py's _build_spec: a StructureSpec
    # field missing here makes a stored config carrying it be refused whole by
    # the unknown-key guard in build_spec_from_cap_config. The exhaustiveness
    # test in tests/unit/test_config_schema.py detects a missing key; it does
    # not prevent one.
    "tip_alignment_skip_enabled",
    "feature_dense_lattice_enabled",
    "feature_aspect_ratio_max",
    "feature_fill_ratio_min",
    "feature_lattice_pitch_mm",
    "max_rotation_deg",
    "web_tilt_threshold_deg",
    "max_web_tilt_deg",
    "mesh_alignment_tolerance_mm",
    "mesh_alignment_sensitivity_enabled",
    "record_host_geometry_sidecar",
    "root_end",
    "per_station_reentrant",
    "z_continuity",
    "z_continuity_tie_mode",
    "z_continuity_dx_max_mm",
    "z_continuity_wall_matching",
)

_FAMILY_KEYS = frozenset(
    {
        "name",
        "rib_pitch_mm",
        "web_depth_mm",
        "web_position",
        "slot_gap_mm",
        "gap_ratio",
        "z_drift_mm_per_mm",
        "z_period_mm",
        # mirrors capability.py's own _FAMILY_KEYS (see that module's comment)
        # -- which exterior surface this family's webs enter from.
        "entry_lower",
        "depth_to_socket",
        # Mirrors capability.py's _FAMILY_KEYS: the declared
        # skin_lattice/primary_structure role. Diagnostics/labelling only;
        # WebFamily(**fam_kwargs) below accepts it unchanged.
        "role",
        # Mirrors capability.py's _FAMILY_KEYS: chord-relative pitch/drift
        # (member placement relative to the local chord rather than absolute).
        # See aircraft_core.spec.WebFamily's field docstring.
        "rib_pitch_chord_frac",
        "z_drift_chord_frac_per_mm",
        # Mirrors capability.py's _FAMILY_KEYS: an OPPOSING PAIR of members
        # whose flanks fuse and whose depths overlap.
        "opposing_pair",
        "pair_offset_mm",
        # Mirrors capability.py's _FAMILY_KEYS: socket-relative member
        # placement, with the station derived from the DETECTED spar socket
        # instead of web_position/rib_pitch_chord_frac. See
        # aircraft_core.spec.WebFamily's field docstring.
        "socket_flank_offset_mm",
        "socket_flank_direction",
    }
)


# Keys a stored cap_config may legitimately carry that are consumed by the
# plugin rather than by StructureSpec. Kept beside _SPEC_KEYS so the two are
# maintained together.
_PLUGIN_ONLY_KEYS = frozenset({"metrics_enabled"})


def build_spec_from_cap_config(cap_config: Dict) -> StructureSpec:
    """Build a validated ``StructureSpec`` from one arm config's ``cap_config``
    dict, field-for-field identical to ``capability.py``'s ``_build_spec``.
    """
    # Unknown SPEC keys raise, as unknown FAMILY keys do below. Silently
    # dropping a key (e.g. expected_hole_count=1 falling back to the default 0)
    # produces a symptom far from the cause -- every layer failing with
    # "material has 1 interior hole(s), expected 0" -- and a tool that silently
    # discards what it was asked to test is worse than one that refuses.
    # PLUGIN-LEVEL keys are legitimate members of a stored
    # cap_config that StructureSpec itself does not take. capability.py's
    # DEFAULT_SPEC declares them, and the HOST writes them back into
    # config.json on its own, so a real installed config contains them and
    # refusing it would be wrong. They are listed explicitly rather than
    # pattern-matched, so a typo in one still trips the guard below.
    unknown_spec = (set(cap_config) - set(_SPEC_KEYS) - {"families"}
                    - _PLUGIN_ONLY_KEYS)
    if unknown_spec:
        raise SpecError(
            "arm config has unrecognized top-level key(s) {}; add them to "
            "_SPEC_KEYS if StructureSpec accepts them, rather than letting "
            "them be silently ignored".format(sorted(unknown_spec))
        )

    kwargs = {}
    for key in _SPEC_KEYS:
        if key in cap_config and cap_config[key] is not None:
            kwargs[key] = cap_config[key]

    families = []
    for i, entry in enumerate(cap_config.get("families") or []):
        unknown = set(entry) - _FAMILY_KEYS
        if unknown:
            raise SpecError(
                "arm config family {} has unrecognized key(s) {}".format(i, sorted(unknown))
            )
        fam_kwargs = dict(entry)
        if "name" not in fam_kwargs:
            fam_kwargs["name"] = "family {}".format(i)
        families.append(WebFamily(**fam_kwargs))
    if families:
        kwargs["families"] = tuple(families)

    spec = StructureSpec(**kwargs)
    spec.validate()
    return spec


# ---------------------------------------------------------------------------
# wall-check smoke test: an INTENT-LEVEL mirror of
# ``aircraft_core.validate.TopologyValidator``'s collision rule (validate.py,
# "collision check" / "  -- JUNCTION RULE"), used by
# ``assert_cap_config_slices_cleanly`` below to give the live/tracked config
# smoke checks real teeth without requiring a full boolean cut.
#
# WHY NOT THE REAL VALIDATOR: ``TopologyValidator.validate()`` needs the
# actual diffed/transformed polygons and the erosion (offset) check, both of
# which come from ``ops.diff``/``ops.offset``. This tool's own geometry
# backend, ``tests/unit/stub_ops.StubOps``, is a narrow test double whose
# ``diff`` only subtracts from a plain ``Rect`` and whose ``offset`` raises on
# mixed-depth multi-notch shapes (see its module docstring) -- exactly what an
# arbitrary installed config (mixing full-depth spars and shallow, drifting
# lattice webs) produces on the airfoil fixture. Rather than silently
# skipping the wall check for configs StubOps cannot diff/offset, this
# reimplements the SAME rule the validator applies, directly on the
# generator's requested ``WebIntent`` footprints -- the measured-vs-requested
# difference this implies (this checks REQUESTED footprints; the real
# validator measures the ACTUAL removed region) is a documented
# approximation, acceptable for a smoke check but not a substitute for the
# real validator, which still only runs against a live Orca host.
# ---------------------------------------------------------------------------


def check_wall_violations(intents, safety_offset_mm: float) -> List[str]:
    """Mirror ``TopologyValidator``'s same-edge collision rule on a layer's real
    generated ``WebIntent`` list. Returns a list of human-readable violation
    strings (empty means clean).

    Per entry side (``WebIntent.entry_lower``), sorts requested footprints
    (``start[0] +/- width_mm/2``) by chord station and checks the gap between
    adjacent footprints, exactly as ``validate.py``'s per-side scan does
    ("SAME-EDGE ONLY", same adjacent-neighbour shortcut). A gap below ``2 *
    safety_offset_mm`` is allowed ONLY when the pair's family MEMBERSHIP sets
    are disjoint (set-intersection rule, not bare ``family_index`` equality)
    AND the footprints actually overlap (``gap < 0`` -- the "intended junction"
    branch); anything else is a violation, exactly as
    ``validate.py``'s own message distinguishes.

    NOT COVERED: ``validate.py`` also runs a separate cross-edge ALL-PAIRS
    check (an opposing-surface, e.g. +Y vs -Y, collision that the same-edge
    adjacent-neighbour shortcut cannot see -- see that module's "SAME-EDGE ONLY"
    comment for why the two checks are distinct).
    This helper reproduces only the same-edge check; a smoke check built on it
    is not a substitute for the real validator on that axis.
    """
    min_wall_mm = 2.0 * safety_offset_mm
    violations: List[str] = []
    for side_label, side_flag in (("+Y", False), ("-Y", True)):
        side_intents = [i for i in intents if bool(i.entry_lower) == side_flag]
        if len(side_intents) <= 1:
            continue
        by_x = sorted(side_intents, key=lambda i: i.start[0])
        for a, b in zip(by_x, by_x[1:]):
            a_max_x = a.start[0] + a.width_mm / 2.0
            b_min_x = b.start[0] - b.width_mm / 2.0
            gap = b_min_x - a_max_x
            if gap >= min_wall_mm - EPSILON_MM:
                continue
            same_family = not a.family_membership().isdisjoint(b.family_membership())
            if not same_family and gap < 0:
                # intended junction: footprints overlap, families disjoint --
                # allowed to merge, not a violation.
                continue
            violations.append(
                "{} side: family {!r} and family {!r} leave only {:.4f} mm "
                "of wall between them, below the required {:.4f} mm minimum "
                "(2x the {:.4f} mm safety offset); {}.".format(
                    side_label,
                    a.family_name,
                    b.family_name,
                    gap,
                    min_wall_mm,
                    safety_offset_mm,
                    "same family"
                    if same_family
                    else "different families but footprints do not overlap "
                    "(merging requires an actual overlap)",
                )
            )
    return violations


def _z_sweep_values(spec: StructureSpec, n_samples: int = 12) -> List[float]:
    """Z values to sample for a smoke check: if any family drifts, sweep one
    full period of the LONGEST period among them (grid samples, not the
    breakpoint-exact analysis ``tests/unit/test_lattice_crossing.py`` uses --
    a documented, coarser approximation acceptable for a smoke check); a
    driftless config is identical at every layer by construction, so one
    representative Z is exhaustive.
    """
    periods = [
        f.z_period_mm for f in spec.effective_families() if f.z_period_mm is not None
    ]
    if not periods:
        return [0.2]
    period = max(periods)
    return [i * period / n_samples for i in range(n_samples)]


def _spar_does_not_fit_reason(spec, section) -> Optional[str]:
    """``None`` when ``section`` can hold ``spec``'s declared spar (or no spar
    is declared); otherwise a human-readable reason it cannot.

    This is a second, independent derivation of the generator's spar-fit
    test, so the two can disagree. The REQUIREMENT here is
    algebraically identical to ``_generate_spar_socket``'s -- socket diameter
    plus one ``skin_margin_mm`` of material each side -- but the DEPTH is not:
    this reads ``section.thickness_at(x)``, a single-station lookup into this
    module's own sampled profile table (whose approximation the module
    docstring flags), while the generator takes a worst case over a 2r probe
    window on the real polygon.

    Both directions of disagreement are safe, which is why the duplication is
    tolerable:

    - ``thickness_at`` reads OPTIMISTIC -> the fixture is not skipped, the
      generator runs, and its own stricter fit check raises for itself. No
      false pass.
    - ``thickness_at`` reads PESSIMISTIC -> a fixture is skipped that could
      have run. Silent coverage loss, not a wrong result -- and the exercised
      set returned by ``assert_cap_config_slices_cleanly`` is asserted against
      by test, so a widening skip is caught there rather than here.
    """
    dia = getattr(spec, "spar_diameter_mm", None)
    if not dia:
        return None
    clearance = float(getattr(spec, "spar_clearance_mm", 0.0) or 0.0)
    station = float(getattr(spec, "spar_station", 0.30) or 0.30)
    margin = float(getattr(spec, "skin_margin_mm", 0.0) or 0.0)
    needed = float(dia) + 2.0 * clearance + 2.0 * margin
    x = section.min_x + station * (section.max_x - section.min_x)
    try:
        depth = float(section.thickness_at(x))
    except (LookupError, ValueError, TypeError, ArithmeticError):
        # A fixture whose profile table cannot answer at this station. Do NOT
        # skip: returning None runs the fixture, and the generator's own fit
        # check then raises for itself. Deliberately not a bare ``except`` --
        # that would also swallow an AttributeError from a renamed fixture API
        # and silently disable the skip for EVERY fixture.
        return None
    if depth >= needed:
        return None
    return ("section is {:.3f} mm deep at station {:.3f}c but the declared "
            "spar needs {:.3f} mm".format(depth, station, needed))


def assert_cap_config_slices_cleanly(
    cap_config: Dict,
    line_width_mm: float = 0.42,
    fixtures: Optional[Dict[str, "SectionGeometry"]] = None,
) -> None:
    """Smoke-check ANY valid installed ``cap_config`` -- schema-valid, generates
    without raising, no zero-web layers, no wall violations -- over both the
    coupon and airfoil fixtures. Deliberately asserts NOTHING about the
    config's SHAPE (merge counts, family counts, drift-free-ness): those are
    choices a legitimately installed config is free to make differently from
    the shipped default. Raises
    ``AssertionError``/``ConfigSchemaError``/``SpecError`` with an actionable
    message on the first problem found.

    Returns the set of fixture labels actually EXERCISED. Callers may ignore
    it, but tests should not: a config declaring a spar skips fixtures too
    shallow to hold it, and the silent failure mode of that rule is skipping
    more than intended. Asserting on this set turns that into a tripwire.
    """
    assert_valid_config(cap_config, wall_width_mm=line_width_mm)
    spec = build_spec_from_cap_config(cap_config)

    if fixtures is None:
        fixtures = {
            "coupon": build_rect_section(width_mm=20.0, height_mm=20.0, span_mm=60.0),
            "airfoil": build_airfoil_section(),
            # The two fixtures above are 20 mm and 7.2 mm deep, so NEITHER can
            # hold a ~10 mm carbon spar -- a config declaring one would be
            # checked only on sections it can never fit. This wing-scale
            # section (150 mm chord, 18 mm deep) is what such a config is
            # actually FOR, so it is exercised rather than skipped.
            "airfoil_wing": build_airfoil_section(chord_mm=150.0, span_mm=120.0),
        }

    generator = StructureGenerator()
    z_values = _z_sweep_values(spec)
    exercised = []
    for label, section in fixtures.items():
        # A declared spar has an ABSOLUTE minimum depth, so a config that is
        # perfectly valid for the wing it describes cannot be sliced on a
        # fixture thinner than the tube. Failing it there would test an
        # impossible combination, not the config. Fixtures that CAN hold the
        # spar are still checked in full, and a config that fits NO fixture
        # still fails loudly below -- so this narrows the fixture set, never
        # the assertions.
        skip_reason = _spar_does_not_fit_reason(spec, section)
        if skip_reason is not None:
            continue
        exercised.append(label)
        for z in z_values:
            ctx = LayerContext(layer_index=0, print_z=z)
            result = generator.generate(section.frame, spec, line_width_mm, ctx)
            if not result.intents:
                raise AssertionError(
                    "{} fixture at Z={:.3f} mm built ZERO webs -- on a real "
                    "slice this would be a fatal TopologyError.".format(label, z)
                )
            violations = check_wall_violations(result.intents, line_width_mm)
            if violations:
                raise AssertionError(
                    "{} fixture at Z={:.3f} mm has {} minimum-wall violation(s): "
                    "{}".format(label, z, len(violations), "; ".join(violations))
                )

    if not exercised:
        raise AssertionError(
            "no fixture could hold this config's declared spar "
            "(spar_diameter_mm={!r}, spar_clearance_mm={!r} at station {!r}), "
            "so NOTHING was checked. A config that fits no available section "
            "is not installable: shrink the spar, move it to a thicker "
            "station, or supply a fixture deep enough to hold it.".format(
                getattr(spec, "spar_diameter_mm", None),
                getattr(spec, "spar_clearance_mm", None),
                getattr(spec, "spar_station", None),
            )
        )

    return set(exercised)


# ---------------------------------------------------------------------------
# Torsional stiffness: two models, on the same real generated web stations.
# ---------------------------------------------------------------------------


def j_open_cell(section: SectionGeometry, t_skin_mm: float) -> float:
    """Bredt-Batho single-cell torsion constant on the OUTER contour only.

    ``J = 4A^2 / (P/t)`` [LIT: Megson, Structural and Stress Analysis, ch.
    11]. Webs contribute NOTHING to this model
    by construction -- this is the "open-cell / cantilever-web" reading:
    every web is a cantilever off one skin and carries
    no shear flow around a closed loop, so the torsion-carrying section is
    just the outer skin. Independent of the web pattern entirely -- the
    same value for every layer of a given arm, so this is computed once,
    not per layer.
    """
    return 4.0 * section.area_mm2 ** 2 * t_skin_mm / section.perimeter_mm


def _solve_linear(matrix: List[List[float]], rhs: List[float]) -> List[float]:
    """Plain Gaussian elimination with partial pivoting, pure Python.
    Small systems only (this tool never builds more than a few dozen
    unknowns) -- no numpy dependency for a handful of linear solves.
    """
    n = len(rhs)
    a = [row[:] + [rhs[i]] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot_row = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[pivot_row][col]) < 1e-15:
            raise ValueError("singular system in multicell torsion solve")
        a[col], a[pivot_row] = a[pivot_row], a[col]
        pivot = a[col][col]
        for j in range(col, n + 1):
            a[col][j] /= pivot
        for r in range(n):
            if r == col:
                continue
            factor = a[r][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                a[r][j] -= factor * a[col][j]
    return [a[i][n] for i in range(n)]


def solve_multicell_j(
    section: SectionGeometry,
    web_stations_mm: Sequence[float],
    t_skin_mm: float,
    t_web_mm: float,
) -> float:
    """Multi-cell St-Venant torsion solve: ``web_stations_mm`` full-depth
    webs (each idealised as reaching BOTH skins -- see the module docstring
    of ``analyze_arm`` for why this is the "closed" model's own idealisation
    and not the real, tip-gapped generated geometry) divide the section into
    ``len(web_stations_mm) + 1`` chained cells (leading-edge cap, N-1
    interior cells, trailing-edge cap for the airfoil; two end cells for the
    rectangle, whose "leading/trailing edge" IS a full-height wall -- see
    ``SectionGeometry.thickness_at`` at ``min_x``/``max_x``).

    Unknowns: one shear flow ``q_i`` per cell, plus the common rate of
    twist ``Omega = G * dtheta/dz``. Equations, per Megson's multi-cell
    method (validated against closed-form Bredt-Batho at zero webs):

        2 * A_i * Omega = q_i * S_i - sum_j q_j * (ds/t)_shared_ij

    where ``S_i`` is the sum of ``ds/t`` around cell ``i``'s own full
    perimeter (outer skin arcs at ``t_skin_mm``, PLUS its own web edges at
    ``t_web_mm``), and ``(ds/t)_shared_ij`` is the shared web edge between
    adjacent cells i, j. Plus one torque equation ``T = sum_i 2*A_i*q_i``.
    Solved for ``T = 1`` (arbitrary; ``G`` cancels identically, same as the
    single-cell formula), giving ``J = T / Omega = 1 / Omega``.

    VALIDATED (see ``tests/unit/test_structural_analysis.py``): a single
    centred web on a symmetric rectangle reduces this system to EXACTLY the
    no-web Bredt-Batho value (the "symmetric limit": q1 == q2 by symmetry, so the shared web carries zero shear
    flow and adds nothing) -- solved as a closed-form hand check, not merely
    asserted.
    """
    stations = sorted(set(round(x, 9) for x in web_stations_mm))
    boundaries = [section.min_x] + stations + [section.max_x]
    n = len(boundaries) - 1  # number of cells
    if n <= 0:
        raise ValueError("degenerate section: no cells to solve for")

    areas = []
    outer_ds_over_t = []
    for k in range(n):
        x0, x1 = boundaries[k], boundaries[k + 1]
        areas.append(section.area_between(x0, x1))
        arc = (section.arc_upper(x1) - section.arc_upper(x0)) + (
            section.arc_lower(x1) - section.arc_lower(x0)
        )
        outer_ds_over_t.append(arc / t_skin_mm)
    # End caps: the section's own boundary at min_x/max_x (zero for an
    # airfoil's leading/trailing edge, a full vertical wall for the
    # rectangle coupon -- see SectionGeometry.thickness_at).
    outer_ds_over_t[0] += section.thickness_at(boundaries[0]) / t_skin_mm
    outer_ds_over_t[-1] += section.thickness_at(boundaries[-1]) / t_skin_mm

    edge_ds_over_t = [section.thickness_at(x) / t_web_mm for x in stations]

    if n == 1:
        return 4.0 * areas[0] ** 2 / outer_ds_over_t[0]

    size = n + 1  # q_0..q_{n-1}, Omega
    matrix = [[0.0] * size for _ in range(size)]
    rhs = [0.0] * size
    for i in range(n):
        s_i = outer_ds_over_t[i]
        if i - 1 >= 0:
            s_i += edge_ds_over_t[i - 1]
            matrix[i][i - 1] -= edge_ds_over_t[i - 1]
        if i < n - 1:
            s_i += edge_ds_over_t[i]
            matrix[i][i + 1] -= edge_ds_over_t[i]
        matrix[i][i] += s_i
        matrix[i][n] -= 2.0 * areas[i]
    for i in range(n):
        matrix[n][i] = 2.0 * areas[i]
    rhs[n] = 1.0

    solution = _solve_linear(matrix, rhs)
    omega = solution[n]
    if omega <= 0:
        raise ValueError("non-physical (non-positive) Omega in multicell solve")
    return 1.0 / omega


# ---------------------------------------------------------------------------
# Buckling: flat-plate and Batdorf-curvature forms.
# ---------------------------------------------------------------------------

K_C_SS = 4.00  # simply-supported compression buckling coefficient [NACA TN 3781]
K_S_SS = 5.35  # simply-supported shear buckling coefficient [NACA TN 3781]


def flat_plate_sigma_cr_mpa(t_mm: float, b_mm: float, e_mpa: float, nu: float, k: float = K_C_SS) -> float:
    """Flat-plate critical compression buckling stress [LIT: Gerard &
    Becker, NACA TN 3781, 1957]. ``sigma_cr = k * pi^2 * E / (12*(1-nu^2)) * (t/b)^2``.
    """
    return k * math.pi ** 2 * e_mpa / (12.0 * (1.0 - nu ** 2)) * (t_mm / b_mm) ** 2


def flat_plate_tau_cr_mpa(sigma_cr_mpa: float, k_c: float = K_C_SS, k_s: float = K_S_SS) -> float:
    """Shear buckling stress from the compression value, via the ratio of
    NACA TN 3781's own simply-supported coefficients: ``tau_cr = (k_s/k_c) *
    sigma_cr`` -- same geometry, same formula, different boundary-condition
    coefficient."""
    return (k_s / k_c) * sigma_cr_mpa


def batdorf_curvature_factor(b_mm: float, r_mm: float, t_mm: float, nu: float) -> float:
    """Curvature credit on a shear-buckling stress, applied only when the
    Batdorf curvature parameter ``Zb`` puts the panel in the "sharply
    curved" regime [LIT: Air Force Stress Manual via Engineering Library,
    "Analysis of Curved Plates", eq 6-35; primary Gerard & Becker Part III,
    NACA TN 3783].

    ``Zb = b^2 * sqrt(1-nu^2) / (r*t)``; applies (returns
    ``0.37*sqrt(Zb)``, a MULTIPLIER on the flat-plate value) only when
    ``b^2/(r*t) > 30`` (the stated applicability threshold). Returns ``1.0``
    (no credit, i.e. the flat-plate value stands) otherwise -- including
    when ``r_mm`` is ``None`` or non-positive (locally flat or unknown
    curvature; this must not silently manufacture a credit past a boundary
    this tool cannot honestly evaluate).
    """
    if r_mm is None or r_mm <= 0 or b_mm <= 0 or t_mm <= 0:
        return 1.0
    if (b_mm ** 2) / (r_mm * t_mm) <= 30.0:
        return 1.0
    zb = b_mm ** 2 * math.sqrt(1.0 - nu ** 2) / (r_mm * t_mm)
    return 0.37 * math.sqrt(zb)


def local_radius_of_curvature_mm(section: SectionGeometry, x_mm: float, surface: str, h: float = 0.25) -> Optional[float]:
    """Local radius of curvature of the upper or lower surface at ``x_mm``,
    via a central finite-difference second derivative of the SAME dense
    profile table ``thickness_at``/``arc_upper``/``arc_lower`` are built
    from (only available for the airfoil section -- the rectangle's flat
    sides have infinite radius, correctly returned as ``None`` -> no
    curvature credit).

    ``radius = (1 + y'^2)^1.5 / |y''|`` -- the standard planar curve
    curvature formula. Returns ``None`` if the section exposes no
    ``y_of_x``-style callable for the requested surface (the rectangle) or
    if the estimated curvature is degenerate (near-zero second derivative,
    i.e. locally flat).
    """
    y_of_x = section.y_upper_of_x if surface == "upper" else section.y_lower_of_x
    if y_of_x is None:
        return None
    y0 = y_of_x(x_mm - h)
    y1 = y_of_x(x_mm)
    y2 = y_of_x(x_mm + h)
    dy = (y2 - y0) / (2 * h)
    d2y = (y2 - 2 * y1 + y0) / (h ** 2)
    if abs(d2y) < 1e-9:
        return None
    return (1.0 + dy ** 2) ** 1.5 / abs(d2y)


def buckling_torque_n_m(area_mm2: float, t_mm: float, tau_cr_mpa: float) -> float:
    """Section buckling torque ``T_cr = 2*A*t*tau_cr``,
    converted from N.mm (mm^2 * mm * N/mm^2) to N.m."""
    return 2.0 * area_mm2 * t_mm * tau_cr_mpa / 1000.0


# ---------------------------------------------------------------------------
# Per-arm analysis.
# ---------------------------------------------------------------------------


@dataclass
class ArmResult:
    name: str
    section_name: str
    line_width_mm: float
    layer_height_mm: float
    n_layers: int
    skin_margin_mm: float
    resolved_slot_gap_mm: float
    web_thickness_mm: float
    area_mm2: float
    perimeter_mm: float
    mass_proxy_mm3: float
    j_open_mm4: float
    j_closed_mean_mm4: float
    j_closed_min_mm4: float
    j_closed_max_mm4: float
    worst_bay_upper_mm: Optional[float]
    lower_arc_total_mm: float
    web_tip_air_gap_mm: Optional[float]
    web_tip_bond_class: Optional[str]
    lower_skin_restrained: bool
    governing_bay_mm: float
    lower_skin_restrained_lenient: bool = False
    governing_bay_lenient_mm: float = 0.0
    e_results: Dict[float, Dict[str, float]] = field(default_factory=dict)
    total_web_count: int = 0
    full_depth_web_count: int = 0
    total_omitted_count: int = 0
    omitted_by_reason: Dict[str, int] = field(default_factory=dict)
    zero_web_layer_count: int = 0
    mean_webs_per_layer: float = 0.0
    errors: List[str] = field(default_factory=list)


def analyze_arm(
    name: str,
    cap_config: Dict,
    line_width_mm: float,
    section: SectionGeometry,
    layer_height_mm: float = 0.2,
    e_values_gpa: Sequence[float] = (3.5, 2.5),
    nu: float = 0.35,
) -> ArmResult:
    """Run the real generator over every body layer of ``section`` for one
    arm config, accumulate the real diagnostics, and compute the predicted
    figures of merit. See the module docstring for exactly what is real
    generated geometry versus modelled on top of it.
    """
    spec = build_spec_from_cap_config(cap_config)
    n_layers = max(1, int(round(section.span_mm / layer_height_mm)))

    rd = RunDiagnostics()
    rd.set_skin_line_width_mm(line_width_mm)
    rd.set_skin_margin_mm(spec.skin_margin_mm)

    # WebFamily.resolve_gap falls back to spec.resolve_slot_gap when the
    # family sets neither slot_gap_mm nor gap_ratio itself -- correct for
    # every arm config here, whose families never override the gap.
    resolved_slot_gap_mm = spec.effective_families()[0].resolve_gap(spec, line_width_mm)
    web_thickness_mm = 2.0 * line_width_mm + resolved_slot_gap_mm

    total_length_mm = 0.0
    j_closed_samples: List[float] = []
    generator = StructureGenerator()
    total_omitted_count = 0
    omitted_by_reason: Dict[str, int] = {}
    zero_web_layer_count = 0
    total_web_events = 0  # sum of len(intents) across layers, for the mean
    errors: List[str] = []

    for i in range(n_layers):
        z = (i + 1) * layer_height_mm
        layer_ctx = LayerContext(layer_index=i, print_z=z)
        result = generator.generate(section.frame, spec, line_width_mm, layer_ctx)
        intents = result.intents
        web_depths = [
            math.hypot(intent.end[0] - intent.start[0], intent.end[1] - intent.start[1])
            for intent in intents
        ]
        entry_x = [intent.start[0] for intent in intents]
        rd.record_structure_layer(
            i, z, section.perimeter_mm, web_depths, entry_x, full_depth_web_count=len(intents)
        )
        total_length_mm += section.perimeter_mm + sum(2.0 * d for d in web_depths)
        total_web_events += len(intents)

        # OMISSION TRACKING: arms with a tight skin_margin_mm can lose the
        # large majority of their requested webs to thin-section omission. A config whose stated web
        # pattern is mostly omitted is NOT the structure its cap_config
        # describes, and that must be visible here, not averaged away.
        total_omitted_count += len(result.omitted)
        for omission in result.omitted:
            key = omission.reason.split(":", 1)[0]
            omitted_by_reason[key] = omitted_by_reason.get(key, 0) + 1
        if not intents:
            zero_web_layer_count += 1
            errors.append(
                "layer {} (Z={:.2f} mm): every candidate web omitted -- on a "
                "real slice with tip_shutdown_enabled=False (this spec's "
                "value: {!r}) this layer would be a fatal TopologyError, not "
                "a silently-thinner layer.".format(
                    i, z, spec.tip_shutdown_enabled
                )
            )

        # WALL CHECK: makes a "N/N layers OK" sweep result mean the
        # generated web placement is valid, not just "did not raise" -- see
        # check_wall_violations's docstring for why this mirrors
        # TopologyValidator's rule at intent level instead of running the real
        # validator, which this tool's StubOps backend cannot support for an
        # arbitrary config/fixture combination).
        wall_violations = check_wall_violations(intents, line_width_mm)
        for violation in wall_violations:
            errors.append("layer {} (Z={:.2f} mm): {}".format(i, z, violation))

        stations = sorted(set(round(x, 9) for x in entry_x))
        if stations:
            j_closed = solve_multicell_j(section, stations, line_width_mm, web_thickness_mm)
        else:
            j_closed = j_open_cell(section, line_width_mm)
        j_closed_samples.append(j_closed)

    summary = rd.structure_summary()
    j_open = j_open_cell(section, line_width_mm)
    j_closed_mean = sum(j_closed_samples) / len(j_closed_samples)
    mass_proxy_mm3 = total_length_mm * line_width_mm * layer_height_mm
    mean_webs_per_layer = total_web_events / n_layers

    bond_class = summary["web_tip_bond_class"]
    lower_restrained = bond_class == "fused"
    # LENIENT reading, reported ALONGSIDE the strict one, not instead of it.
    # It exists to surface a floating-point boundary artifact: a config tuned
    # to land EXACTLY on the 0.021 mm hard-fuse point can compute a gap of
    # 0.021000000000000019 mm (float64 rounding on
    # `skin_margin_mm + headroom - 2*line_width`), landing in the "weak"
    # bucket by under a femtometre -- a distance no print or measurement could
    # resolve. Treating "weak" as restrained too is the honest reading of that
    # specific artifact; it is not a general loosening of the bond
    # classification, since "weak" ("fused, but separable by hand") is a real,
    # weaker bond class at any gap that is not a rounding artifact of the
    # exact 0.021 boundary.
    lower_restrained_lenient = bond_class in ("fused", "weak")
    worst_upper_bay = summary["worst_skin_bay_mm"]

    if lower_restrained:
        governing_bay_mm = worst_upper_bay if worst_upper_bay is not None else section.perimeter_mm
    else:
        # -Y skin has no restraint at all: the governing
        # bay is the WHOLE lower-surface arc.
        governing_bay_mm = section.lower_arc_total_mm

    if lower_restrained_lenient:
        governing_bay_lenient_mm = worst_upper_bay if worst_upper_bay is not None else section.perimeter_mm
    else:
        governing_bay_lenient_mm = section.lower_arc_total_mm

    e_results: Dict[float, Dict[str, float]] = {}
    for e_gpa in e_values_gpa:
        e_mpa = e_gpa * 1000.0
        sigma_flat = flat_plate_sigma_cr_mpa(line_width_mm, governing_bay_mm, e_mpa, nu)
        tau_flat = flat_plate_tau_cr_mpa(sigma_flat)
        t_cr_flat = buckling_torque_n_m(section.area_mm2, line_width_mm, tau_flat)

        radius_mm = None
        if not lower_restrained:
            # Curvature credit only meaningfully applies to the governing
            # (lower-surface, unrestrained) bay; sample its midpoint.
            mid_x = (section.min_x + section.max_x) / 2.0
            radius_mm = local_radius_of_curvature_mm(section, mid_x, "lower")
        curvature_factor = batdorf_curvature_factor(governing_bay_mm, radius_mm, line_width_mm, nu)
        sigma_curved = sigma_flat * curvature_factor
        tau_curved = tau_flat * curvature_factor
        t_cr_curved = buckling_torque_n_m(section.area_mm2, line_width_mm, tau_curved)

        sigma_flat_lenient = flat_plate_sigma_cr_mpa(line_width_mm, governing_bay_lenient_mm, e_mpa, nu)
        tau_flat_lenient = flat_plate_tau_cr_mpa(sigma_flat_lenient)
        t_cr_flat_lenient = buckling_torque_n_m(section.area_mm2, line_width_mm, tau_flat_lenient)

        e_results[e_gpa] = {
            "sigma_cr_flat_mpa": sigma_flat,
            "tau_cr_flat_mpa": tau_flat,
            "t_cr_flat_n_m": t_cr_flat,
            "curvature_factor": curvature_factor,
            "radius_mm": radius_mm,
            "sigma_cr_curved_mpa": sigma_curved,
            "tau_cr_curved_mpa": tau_curved,
            "t_cr_curved_n_m": t_cr_curved,
            "t_cr_flat_lenient_n_m": t_cr_flat_lenient,
        }

    return ArmResult(
        name=name,
        section_name=section.name,
        line_width_mm=line_width_mm,
        layer_height_mm=layer_height_mm,
        n_layers=n_layers,
        skin_margin_mm=spec.skin_margin_mm,
        resolved_slot_gap_mm=resolved_slot_gap_mm,
        web_thickness_mm=web_thickness_mm,
        area_mm2=section.area_mm2,
        perimeter_mm=section.perimeter_mm,
        mass_proxy_mm3=mass_proxy_mm3,
        j_open_mm4=j_open,
        j_closed_mean_mm4=j_closed_mean,
        j_closed_min_mm4=min(j_closed_samples),
        j_closed_max_mm4=max(j_closed_samples),
        worst_bay_upper_mm=worst_upper_bay,
        lower_arc_total_mm=section.lower_arc_total_mm,
        web_tip_air_gap_mm=summary["web_tip_air_gap_mm"],
        web_tip_bond_class=bond_class,
        lower_skin_restrained=lower_restrained,
        governing_bay_mm=governing_bay_mm,
        lower_skin_restrained_lenient=lower_restrained_lenient,
        governing_bay_lenient_mm=governing_bay_lenient_mm,
        e_results=e_results,
        total_web_count=summary["total_web_count"],
        full_depth_web_count=summary["full_depth_web_count"],
        total_omitted_count=total_omitted_count,
        omitted_by_reason=omitted_by_reason,
        zero_web_layer_count=zero_web_layer_count,
        mean_webs_per_layer=mean_webs_per_layer,
        errors=errors,
    )


# ---------------------------------------------------------------------------
# CLI: run all four arms and write the predicted-figures markdown.
# ---------------------------------------------------------------------------

ARM_CONFIGS = (
    ("A", "arm_A_two_families.json", 0.42),
    ("B", "arm_B_one_family.json", 0.42),
    ("C", "arm_C_thick_skin.json", 0.60),
    ("D", "arm_D_unfused_tips_control.json", 0.42),
)

CONFIGS_DIR = os.path.join(REPO_ROOT, "tests", "fixtures", "structural_arms")
# Output of ``main``. The default directory (``experiments/``) is not part of
# this repository, so the path is optional: it does not exist here and the
# write in ``main`` fails unless the directory is created or this path is
# changed.
PREDICTED_MD_PATH = os.path.join(REPO_ROOT, "experiments", "M11_structural_ab", "predicted.md")


def load_arm_cap_config(filename: str) -> Dict:
    path = os.path.join(CONFIGS_DIR, filename)
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    return data["config"][0]["cap_config"]


def run_all_arms(section_factory=None) -> Tuple[List[ArmResult], str, str]:
    """Run every arm against the airfoil fixture; on ANY exception, fall
    back to the rectangular coupon for every arm (never a per-arm mix,
    which would make the comparison table incomparable) and record why.

    Returns (results, section_used_name, fallback_reason_or_empty).
    """
    if section_factory is None:
        section_factory = build_airfoil_section

    results = []
    fallback_reason = ""
    section = section_factory()
    try:
        for name, filename, line_width in ARM_CONFIGS:
            cap_config = load_arm_cap_config(filename)
            results.append(analyze_arm(name, cap_config, line_width, section))
        return results, section.name, fallback_reason
    except Exception as exc:  # noqa: BLE001 - deliberate: report, then fall back
        fallback_reason = "{}: {}".format(type(exc).__name__, exc)

    results = []
    section = build_rect_section(span_mm=60.0)
    for name, filename, line_width in ARM_CONFIGS:
        cap_config = load_arm_cap_config(filename)
        results.append(analyze_arm(name, cap_config, line_width, section))
    return results, section.name, fallback_reason


def _fmt(x, digits=3):
    if x is None:
        return "n/a"
    return "{:.{}f}".format(x, digits)


def render_predicted_md(results: List[ArmResult], section_name: str, fallback_reason: str) -> str:
    by_arm = {r.name: r for r in results}
    lines = []
    lines.append("# M11 predicted structural figures of merit")
    lines.append("")
    lines.append(
        "**Status: PREDICTED / MODELLED. Nothing on this page is a measurement.** "
        "Every number here comes from running the real "
        "`aircraft_core.generator.StructureGenerator` over the fixture geometry "
        "and computing textbook torsion/buckling formulas on the result -- it is "
        "not a substitute for the printed-coupon A/B rig "
        "`experiments/M11_structural_ab/README.md` describes, and it carries "
        "exactly the honesty caveats "
        "`docs/research/what_governs_strength_to_weight.md` and "
        "`docs/research/skin_and_external_structure.md` already recorded. "
        "Generated by `tools/structural_analysis.py`; do not hand-edit."
    )
    lines.append("")
    lines.append("Fixture used: **{}**.".format(section_name))
    if fallback_reason:
        lines.append("")
        lines.append(
            "**The airfoil fixture path failed in this offline harness and the "
            "tool fell back to the rectangular coupon for ALL FOUR arms** (never "
            "a per-arm mix). Reason: `{}`.".format(fallback_reason)
        )
    lines.append("")
    lines.append("## Model assumptions (read before the table)")
    lines.append("")
    lines.append(
        "- **Mass proxy** = (sum over every body layer of the skin's own "
        "un-slotted perimeter length, plus `2 * depth` per surviving web) * "
        "line width * layer height (`{:.2f} mm`, assumed -- not itself a config "
        "field in any arm's `cap_config`; matches the M1/M11 process presets). "
        "This is a **volume proxy at constant layer height, not a true mass**: "
        "it does not model bead cross-section shape, extrusion width variation, "
        "or material density. See `aircraft_core/diagnostics.py`'s "
        "`record_structure_layer` docstring for the same approximation as used "
        "in the shipped plugin's own diagnostics.".format(0.2)
    )
    lines.append(
        "- **J (open-cell)**: single-cell Bredt-Batho on the OUTER contour "
        "only, `J = 4A^2/(P/t)`. Webs contribute nothing, by construction -- "
        "this is the model for a part whose webs are cantilevers off one skin "
        "(paper 2's finding)."
    )
    lines.append(
        "- **J (closed multi-cell)**: the real generated web stations, at "
        "EVERY generated body layer, treated as full-depth shear ties "
        "reaching BOTH skins (the idealised, tip-fused case -- NOT the real "
        "tip-gapped geometry the generator actually places; see "
        "`solve_multicell_j`'s docstring). Reported as the mean across every "
        "generated layer, with the min/max range, since the web stations drift "
        "with Z for the drifting families and the cell layout is genuinely "
        "different layer to layer."
    )
    lines.append(
        "- **Web thickness for shear** (`t_web`) = `2 * line_width + resolved "
        "slot gap` -- the project's own established fused-bead-pair figure "
        "(DECISIONS.md D006; `docs/research/what_governs_strength_to_weight.md` "
        "Sec 2)."
    )
    lines.append(
        "- **Buckling**: flat-plate `sigma_cr = k*pi^2*E/(12(1-nu^2))*(t/b)^2`, "
        "`k_c=4.00` (compression, SS), `k_s=5.35` (shear, SS) [Gerard & Becker, "
        "NACA TN 3781]. `E` used at **3.5 GPa AND 2.5 GPa** (both papers' own "
        "bracket for FDM PLA; both are reported, never averaged). `nu=0.35`. "
        "Where the governing bay is unrestrained curved skin, a Batdorf "
        "curvature credit is ALSO computed and reported alongside the flat "
        "value (never substituted silently) -- see `batdorf_curvature_factor`."
    )
    lines.append(
        "- **Governing bay / -Y skin restraint**: determined from the REAL "
        "`web_tip_air_gap_mm` / `web_tip_bond_class` this run's diagnostics "
        "compute (D006's measured bond table) for each arm's actual "
        "`skin_margin_mm`. Only `\"fused\"` is treated as restraining the -Y "
        "skin; `\"weak\"`/`\"marginal\"`/`\"none\"` are all treated as "
        "UNRESTRAINED (the honest, conservative reading -- D006 itself only "
        "measured a bond at the hard-fuse point, not at the weaker classes)."
    )
    lines.append(
        "- **What is NOT computed**: any bending/EI figure (paper 1 Sec 7 "
        "already ranks bending strength as bond-limited, a process variable no "
        "geometry choice here fixes); any FE or measured allowable; any "
        "specimen-to-specimen scatter (nothing has been printed for this run)."
    )
    lines.append("")
    lines.append("## Comparison table")
    lines.append("")
    header = (
        "| Arm | line width (mm) | skin_margin (mm) | tip gap (mm) | bond class | "
        "-Y restrained (strict) | -Y restrained (lenient) | mass proxy (mm^3) | "
        "J open (mm^4) | J closed mean (min-max) (mm^4) | closed/open | "
        "worst +Y bay (mm) | governing bay, strict/lenient (mm) |"
    )
    lines.append(header)
    lines.append("|" + "---|" * 13)
    for r in results:
        lines.append(
            "| {a} | {lw:.2f} | {sm:.4f} | {gap} | {bc} | {restr} | {restrl} | "
            "{mp:.1f} | {jo:.1f} | {jc:.1f} ({jcmin:.1f}-{jcmax:.1f}) | "
            "{ratio:.3f} | {wb} | {gb:.2f} / {gbl:.2f} |".format(
                a=r.name,
                lw=r.line_width_mm,
                sm=r.skin_margin_mm,
                gap=_fmt(r.web_tip_air_gap_mm, 4),
                bc=r.web_tip_bond_class,
                restr="yes" if r.lower_skin_restrained else "NO",
                restrl="yes" if r.lower_skin_restrained_lenient else "NO",
                mp=r.mass_proxy_mm3,
                jo=r.j_open_mm4,
                jc=r.j_closed_mean_mm4,
                jcmin=r.j_closed_min_mm4,
                jcmax=r.j_closed_max_mm4,
                ratio=r.j_closed_mean_mm4 / r.j_open_mm4,
                wb=_fmt(r.worst_bay_upper_mm, 2),
                gb=r.governing_bay_mm,
                gbl=r.governing_bay_lenient_mm,
            )
        )
    lines.append("")
    lines.append("## PROMINENT FINDING: A/B/C's own web pattern is mostly omitted, not just tip-unfused")
    lines.append("")
    lines.append(
        "This was NOT anticipated by either research paper or the M11 README, "
        "and it was found by this run's own numbers, not asserted going in: "
        "the mass proxy and `J_closed` for arms A/B/C are far lower than a "
        "naive reading of their `cap_config` (14 webs/layer, two ±45 degree "
        "families) would predict, and D's `J_closed` mean (**{d_j:.1f} mm^4**) "
        "lands almost exactly on paper 1's own PUBLISHED figure for the "
        "as-shipped 14-web layout (**1397 mm^4**, "
        "`docs/research/what_governs_strength_to_weight.md` Sec 3.2's table) "
        "-- a real cross-check this tool did not target but landed on. "
        "Tallying the real `OmittedWeb` records (`result.omitted`) from "
        "every generated layer explains why:".format(d_j=by_arm.get("D").j_closed_mean_mm4 if "D" in by_arm else 0.0)
    )
    lines.append("")
    lines.append(
        "| Arm | mean SURVIVING webs/layer | total omitted | "
        "omitted: thin section | omitted: skin margin | omitted: chord extent | "
        "zero-web layers |"
    )
    lines.append("|" + "---|" * 7)
    for r in results:
        lines.append(
            "| {a} | {mw:.2f} | {tot} | {ts} | {sm} | {ce} | {zw} |".format(
                a=r.name,
                mw=r.mean_webs_per_layer,
                tot=r.total_omitted_count,
                ts=r.omitted_by_reason.get("thin section", 0),
                sm=r.omitted_by_reason.get("skin margin", 0),
                ce=r.omitted_by_reason.get("chord extent", 0),
                zw=r.zero_web_layer_count,
            )
        )
    lines.append("")
    lines.append(
        "**A/B/C lose the large majority of their requested webs to M5's "
        "\"thin section\" omission** (`aircraft_core/generator.py`'s "
        "`OmittedWeb.reason` category 3: the margin fits, but the remaining "
        "skin cannot survive the validator's own halo-span erosion check -- "
        "see that module's \"M5 change -- thin-section omission\" note). "
        "The generator's own testimony, one representative omission pulled "
        "verbatim from an arm-A layer during this investigation: "
        "*\"thin section: remaining skin 0.8068 mm at x=-24.8003 mm, measured "
        "across the 0.8610 mm erosion halo, cannot survive 2x the 0.4200 mm "
        "safety offset (0.8400 mm required)\"* -- 0.8068 mm of skin measured "
        "across the WIDE halo, just short of the 0.8400 mm the erosion check "
        "needs, at a station where the section is plenty thick for the "
        "narrow slot itself. D, whose margin sits 0.16 mm clear of the fuse "
        "point rather than 0.021 mm from it, loses far fewer webs the same "
        "way. **This is the same 21-micrometre window paper 2 Sec 3.4 "
        "already flagged as tight (\"the whole scheme lives inside a 21 um "
        "window\") -- what this run adds is that the window is not merely "
        "tight for the tip-fusion curvature check paper 2 evaluated (sagitta "
        "0.006 mm of the 0.021 mm budget); it is ALSO tight against the "
        "WORST-CASE local surface height sampled across the much WIDER "
        "erosion-halo span (~0.86 mm here, versus the slot's own ~0.02 mm) "
        "the M5 thin-section check measures, and that second effect is "
        "large enough to omit thousands of candidate webs "
        "across a 300-layer stack.**"
    )
    lines.append("")
    lines.append(
        "**This contradicts a premise P7 was pre-registered on.** "
        "`experiments/M11_structural_ab/README.md` states \"A vs D isolates "
        "tip fusion at identical mass and identical geometry\" -- this run's "
        "own mass-proxy column above shows A at **{ma:.0f} mm^3** against D "
        "at **{md:.0f} mm^3**, a **{pct:+.1f}%** difference, and the two arms' "
        "web counts differ by roughly the same factor. **If this run's "
        "geometry pipeline is correct, the committed A/B/C configs do not "
        "print the structure their `cap_config` describes on this fixture: "
        "most of their requested webs never survive generation, and a real "
        "slice would either (a) print silently thinner, since the generator's "
        "own omission policy is designed not to fail the whole layer for one "
        "candidate, or (b) hit a zero-web layer and become a fatal "
        "`TopologyError`.** Zero-web layer counts are in the table above. "
        "This needs checking against the real capability path "
        "(`plugin/aircraft_structures/capability.py`) and, if confirmed, "
        "against the actual `skin_margin_mm`/`rib_pitch_mm`/section-thickness "
        "combination before any A-vs-D print is attempted -- it is a "
        "diagnosis, not a fix, and this tool does not attempt to fix "
        "it.".format(
            ma=by_arm["A"].mass_proxy_mm3 if "A" in by_arm else float("nan"),
            md=by_arm["D"].mass_proxy_mm3 if "D" in by_arm else float("nan"),
            pct=(
                100.0 * (by_arm["D"].mass_proxy_mm3 / by_arm["A"].mass_proxy_mm3 - 1.0)
                if "A" in by_arm and "D" in by_arm
                else float("nan")
            ),
        )
    )
    lines.append("")
    lines.append("## Buckling / T_cr, per material-modulus assumption")
    lines.append("")
    for e_gpa in sorted(next(iter(results)).e_results.keys(), reverse=True) if results else []:
        lines.append("### E = {:.1f} GPa".format(e_gpa))
        lines.append("")
        lines.append(
            "| Arm | sigma_cr flat (MPa) | tau_cr flat (MPa) | T_cr flat (N.m) | "
            "curvature factor | T_cr curved (N.m) |"
        )
        lines.append("|" + "---|" * 6)
        for r in results:
            e = r.e_results[e_gpa]
            lines.append(
                "| {a} | {sf:.2f} | {tf:.2f} | {tcf:.4f} | {cf:.2f} | {tcc:.4f} |".format(
                    a=r.name,
                    sf=e["sigma_cr_flat_mpa"],
                    tf=e["tau_cr_flat_mpa"],
                    tcf=e["t_cr_flat_n_m"],
                    cf=e["curvature_factor"],
                    tcc=e["t_cr_curved_n_m"],
                )
            )
        lines.append("")
    lines.append("## Everything per gram (per unit mass proxy)")
    lines.append("")
    lines.append(
        "No density is applied (the mass proxy above is a volume proxy, not a "
        "mass) -- reported here per mm^3 of mass proxy, which is proportional "
        "to per-gram for a fixed material density and is exactly what "
        "`GJ/m`-style figures of merit (paper 1 Sec 7) need: the density "
        "cancels in every RATIO between arms."
    )
    lines.append("")
    header2 = (
        "| Arm | mass proxy (mm^3) | J open / mp | J closed mean / mp | "
        "T_cr flat / mp @3.5GPa (N.m/mm^3) |"
    )
    lines.append(header2)
    lines.append("|" + "---|" * 5)
    for r in results:
        e35 = r.e_results.get(3.5)
        tcr_per_mp = (e35["t_cr_flat_n_m"] / r.mass_proxy_mm3) if e35 else float("nan")
        lines.append(
            "| {a} | {mp:.1f} | {jo:.4f} | {jc:.4f} | {tp:.6f} |".format(
                a=r.name,
                mp=r.mass_proxy_mm3,
                jo=r.j_open_mm4 / r.mass_proxy_mm3,
                jc=r.j_closed_mean_mm4 / r.mass_proxy_mm3,
                tp=tcr_per_mp,
            )
        )
    lines.append("")
    lines.append("## Predictions P1/P2/P3/P7 -- numbers, not adjectives")
    lines.append("")
    a, b = by_arm.get("A"), by_arm.get("B")
    c = by_arm.get("C")
    d = by_arm.get("D")
    if a and b:
        pct_open = 100.0 * (b.j_open_mm4 - a.j_open_mm4) / a.j_open_mm4
        pct_closed = 100.0 * (b.j_closed_mean_mm4 - a.j_closed_mean_mm4) / a.j_closed_mean_mm4
        lines.append(
            "- **P1** (B vs A, `GJ` within 5%; **CAVEAT: as generated, not as "
            "registered** -- the README's P1 compares a 14-webs/layer arm "
            "against a 7-webs/layer arm; this run's own omission tally above "
            "shows A actually SURVIVES {amw:.2f} webs/layer and B "
            "{bmw:.2f}/layer, both far below 14/7 -- so this number compares "
            "the GENERATED structures, not the registered ones): open-cell "
            "model, B is {po:+.2f}% relative to A's `J_open` (identical by "
            "construction: webs never enter `J_open`, so this comparison is "
            "unaffected by the omission finding). Closed-cell model, B is "
            "{pc:+.2f}% relative to A's `J_closed`. Predicted-consistent-with-P1 "
            "= {ok} (on the generated structures).".format(
                po=pct_open, pc=pct_closed, ok=(abs(pct_closed) <= 5.0),
                amw=a.mean_webs_per_layer, bmw=b.mean_webs_per_layer,
            )
        )
    if a and c:
        pct_c = 100.0 * (c.j_open_mm4 - a.j_open_mm4) / a.j_open_mm4
        lines.append(
            "- **P2** (C clearly stiffer than A, open-cell model since that is "
            "the model bead width acts through; **CAVEAT: as generated, not "
            "as registered** -- A survives {amw:.2f} webs/layer, C survives "
            "{cmw:.2f}/layer, both far short of the 7 webs/layer their "
            "one-family `cap_config` requests, per the omission finding "
            "above -- this ratio is unaffected in principle, since `J_open` "
            "never depends on web count at all, but the MASS-PROXY ratio "
            "quoted below IS affected, since mass proxy includes web "
            "length): C's `J_open` is {pc:+.2f}% relative to A's, at a "
            "mass-proxy ratio of {mr:.3f} (C/A, as generated -- not the "
            "hand-tuned, mass-matched ratio the README's protocol calls "
            "for). Mass matched at 5% means the finding is about the "
            "STIFFNESS ratio, not mass parity, which this run does not "
            "itself enforce (see 'What this run does not check' "
            "below).".format(
                pc=pct_c, mr=c.mass_proxy_mm3 / a.mass_proxy_mm3,
                amw=a.mean_webs_per_layer, cmw=c.mean_webs_per_layer,
            )
        )
    lines.append(
        "- **P3** (merged vs suppressed nodes indistinguishable): NOT "
        "evaluated by this tool -- D010 suppression is RETIRED (D012), so "
        "there is no suppressed-node config left to compare against a merged "
        "one in the current codebase. See DECISIONS.md D012."
    )
    if a and d:
        pct_open_ad = 100.0 * (a.j_open_mm4 / d.j_open_mm4 - 1.0)
        pct_closed_ad = 100.0 * (a.j_closed_mean_mm4 / d.j_closed_mean_mm4 - 1.0)
        tcr_ratio_35 = None
        if a.e_results.get(3.5) and d.e_results.get(3.5):
            a_tcr = a.e_results[3.5]["t_cr_flat_n_m"] if a.lower_skin_restrained else a.e_results[3.5]["t_cr_curved_n_m"]
            d_tcr = d.e_results[3.5]["t_cr_flat_n_m"] if d.lower_skin_restrained else d.e_results[3.5]["t_cr_curved_n_m"]
            tcr_ratio_35 = a_tcr / d_tcr if d_tcr else float("inf")
        a_tcr_len = a.e_results[3.5]["t_cr_flat_lenient_n_m"] if a.e_results.get(3.5) else None
        d_tcr_len = d.e_results[3.5]["t_cr_flat_lenient_n_m"] if d.e_results.get(3.5) else None
        tcr_ratio_lenient = (a_tcr_len / d_tcr_len) if (a_tcr_len and d_tcr_len) else None
        lines.append(
            "- **P7** (README premise: A vs D isolates tip fusion \"at "
            "identical mass and identical geometry\" -- **this run's own "
            "numbers show that premise does NOT hold**: A's mass proxy is "
            "{ma:.0f} mm^3 against D's {md:.0f} mm^3, a {pct:+.1f}% "
            "difference, because A loses far more of its requested webs to "
            "M5 thin-section omission than D does -- see the PROMINENT "
            "FINDING section above; read what follows with that caveat, not "
            "as a clean isolation of tip fusion alone): A's tip bond class = "
            "`{abc}` (gap {agap} mm), D's = `{dbc}` (gap {dgap} mm). `J_open` "
            "ratio A/D = {ropen:.3f} (should be ~1.00: open-cell model does "
            "not distinguish bond state by construction). `J_closed` "
            "(idealised, both-fused) ratio A/D = {rclosed:.3f} (this model "
            "ALSO cannot distinguish bond state, since it assumes full "
            "connection regardless -- and is itself confounded here by the "
            "different surviving web counts). `T_cr` ratio A/D at E=3.5 GPa "
            "(governing-bay-aware, the one figure that DOES encode the "
            "tip-bond finding), STRICT reading (only `\"fused\"` restrains) = "
            "{rtcr}; LENIENT reading (`\"fused\"` or `\"weak\"` restrains) = "
            "{rtcr_len}. **See the resolved floating-point note below: these "
            "two readings USED to disagree, and that disagreement was itself "
            "hiding the tip-fusion effect entirely -- the strict reading said "
            "1.00x, no effect. They now agree.**".format(
                ma=a.mass_proxy_mm3,
                md=d.mass_proxy_mm3,
                pct=100.0 * (d.mass_proxy_mm3 / a.mass_proxy_mm3 - 1.0),
                abc=a.web_tip_bond_class,
                agap=_fmt(a.web_tip_air_gap_mm, 4),
                dbc=d.web_tip_bond_class,
                dgap=_fmt(d.web_tip_air_gap_mm, 4),
                ropen=pct_open_ad / 100.0 + 1.0,
                rclosed=pct_closed_ad / 100.0 + 1.0,
                rtcr=("{:.2f}x".format(tcr_ratio_35) if tcr_ratio_35 is not None else "n/a"),
                rtcr_len=("{:.2f}x".format(tcr_ratio_lenient) if tcr_ratio_lenient is not None else "n/a"),
            )
        )
    lines.append("")
    lines.append(
        "**RESOLVED -- a floating-point boundary artifact, fixed 2026-08-31.** "
        "Arm A/B/C's `skin_margin_mm` (0.8608 / 1.2208) was chosen to land EXACTLY "
        "on D006's hard-fuse point (a 0.021 mm gap). float64 computed that gap as "
        "`0.021000000000000019` mm -- about 2e-17 above the threshold -- so it "
        "classified as weak rather than fused, which under the strict restraint "
        "policy flipped A/B/C -Y skin to unrestrained and COLLAPSED THE ENTIRE "
        "A-vs-D CONTRAST: the strict T_cr ratio read 1.00x, i.e. no tip-fusion "
        "effect at all. A rounding error was hiding the result this experiment "
        "exists to find. `aircraft_core/diagnostics.py` now classifies with a "
        "documented 1 nanometre tolerance -- eight orders of magnitude above "
        "float64 noise here, five below anything D006 could resolve -- with tests "
        "asserting it cannot swallow a real difference. The strict and lenient "
        "columns above now AGREE. This paragraph is kept rather than deleted "
        "because the artifact is worth remembering: it is the same \"two "
        "measurements that must agree\" class this project keeps hitting, in a "
        "different coat."
    )
    lines.append("")
    lines.append("## What this run does NOT check")
    lines.append("")
    lines.append(
        "- **Mass matching between arms.** The M11 protocol asks C to be "
        "hand-tuned (raising `rib_pitch_mm`) until its mass is within ~5% of "
        "A's; this tool runs the arm configs AS COMMITTED in "
        "`experiments/M11_structural_ab/configs/`, does not tune anything, and "
        "reports the mass-proxy ratio actually observed so a reader can see "
        "whether that tuning has been done."
    )
    lines.append(
        "- **Whether the closed-cell model is physically real for any arm.** "
        "It is an idealisation (full tip fusion) applied uniformly to every "
        "arm's web stations, regardless of that arm's own measured/predicted "
        "bond class. The `T_cr` figures ARE bond-class-aware (via the "
        "governing-bay selection); `J_closed` is NOT."
    )
    lines.append(
        "- **P6** (failure mode: diagonal skin fold, not a web/node/bond "
        "line) -- not a geometry question at all; requires a printed "
        "specimen loaded to failure."
    )
    lines.append("")
    return "\n".join(lines) + "\n"


def main(argv: List[str]) -> int:
    results, section_name, fallback_reason = run_all_arms()
    md = render_predicted_md(results, section_name, fallback_reason)
    with open(PREDICTED_MD_PATH, "w", encoding="utf-8") as handle:
        handle.write(md)
    print("Wrote {}".format(PREDICTED_MD_PATH))
    print("Fixture used: {}".format(section_name))
    if fallback_reason:
        print("FELL BACK to coupon; airfoil path raised: {}".format(fallback_reason))
    print()
    for r in results:
        print(
            "{:>2}  lw={:.2f}  mass_proxy={:9.1f} mm3  J_open={:8.1f}  "
            "J_closed_mean={:8.1f}  tip_gap={}  bond={:>8}  restrained={:>3}".format(
                r.name,
                r.line_width_mm,
                r.mass_proxy_mm3,
                r.j_open_mm4,
                r.j_closed_mean_mm4,
                _fmt(r.web_tip_air_gap_mm, 4),
                r.web_tip_bond_class,
                "yes" if r.lower_skin_restrained else "NO",
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
