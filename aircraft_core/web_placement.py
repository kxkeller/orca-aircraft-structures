"""Geometry-aware PRIMARY web placement policy.

A fixed chord fraction for a full-depth spar is a manufacturing constraint,
not a structural choice: a full-depth spar survives the thin-section check only
between roughly 16% and 46% chord on a NACA 2412 at ``skin_margin_mm=0.8608``
(the thinnest margin the no-sever proof tolerates), and a rear spar
conventionally wants 55-70% chord. A full-depth member should only count as
part of the primary spar system when it is intentionally placed and sized for
the global load path, so this module selects the chord station from the
section's own geometry instead of scanning for where the check happens to pass.

THIS MODULE DOES NOT REPLACE THE FEASIBILITY TEST. It calls the SAME
``StructureGenerator.generate()`` path every ordinary web goes through
(skin-margin omission, the thin-section erosion halo, the opposing-entry proof,
rotation-envelope correction) via a single-station probe family, so the
generator's own primitives are reused rather than reimplemented. See
``_probe_station``.

DESIGN POINTS

1. WHAT "STRUCTURALLY DEEP" MEANS, OPERATIONALLY.

   The objective is the section's own local thickness (worst-case upper
   surface minus worst-case lower surface across the resolved slot width --
   the SAME ``local_upper_worst_case``/``local_lower_worst_case`` pair every
   web placement uses, so the objective and the feasibility test measure the
   identical geometry rather than two things that could disagree), maximized
   SUBJECT TO a caller-supplied chord window (``MainWebPolicy.chord_window``,
   default (0.15, 0.40)).

   The search is windowed rather than an unconstrained argmax: a full-depth
   member's structural value is flange separation for a wing-box beam
   (bending stiffness scales with the SQUARE of that separation), so maximum
   local thickness is the right raw quantity to chase. But an unconstrained
   argmax has no defence against an airfoil whose thickest point sits
   somewhere structurally inconvenient (reflexed/cambered sections, a
   strangely blunt leading edge, or a probe artefact at the chord extremes
   where ``local_span``'s strip can catch only a sliver of material). The
   window is a policy field, not a hard constant: it defaults to the
   quarter-chord / maximum-thickness region (~15-40% chord) and a caller can
   widen or move it.

   Section-height-weighted objectives (penalizing stations near the window
   edges, or weighting by a beam-theory flange-efficiency curve) are not
   used: they introduce a second free parameter (the weighting curve) with
   no measurement behind it. Raw worst-case thickness within an explicit,
   inspectable window is the smallest defensible objective.

2. PER-PART, NOT PER-LAYER.

   A member whose chord station moves layer to layer is not a spar, it is a
   helix (the Z-drift fields of ``WebFamily`` model that deliberately). A
   spar is a straight, continuous member by definition; searching for a
   "best" station independently on every layer would produce a wandering
   trajectory with no straight-line printable realization. So selection is
   decided ONCE, produces a single chord fraction, and that fraction is
   carried by an ordinary ``WebFamily`` (``rib_pitch_mm=None``, no drift
   fields set) like every other single-station primary family.

   REPRESENTATIVE SECTION: the objective (deepest station) is measured on
   the FIRST frame the caller supplies (``objective_layer_index=0`` by
   default) -- on a root-up print this is the root section, which is both
   the section with the largest chord (so absolute-mm feasibility margins
   are most forgiving there) and, on a real wing, the station carrying the
   largest bending moment, which is where flange separation matters most.
   FEASIBILITY, unlike the objective, is checked at EVERY sampled layer
   (subject to ``layer_stride`` -- see below): a station only survives if
   the full-depth check the generator applies passes everywhere it was
   checked, not merely at the root. This asymmetry -- objective at one
   representative section, feasibility swept across the whole part -- keeps
   a single chosen fraction both structurally motivated AND buildable along
   a taper, without requiring every layer to agree on where "deepest" is.
   The argmax constrained to a 0.15-0.40 window is reasonably stable along a
   straight-leading-edge linear taper, because a NACA section's
   thickest-point fraction does not depend on chord scale, only on the
   (m, p, t) parameters, which taper does not change.

   ``layer_stride`` is a cost control (feasibility testing calls the real
   generator once per candidate station per sampled layer); it always
   includes the first and last supplied frame regardless of stride, so the
   tip (where a tapered wing's full-depth check is most likely to fail first)
   is never skipped.

3. THE AFT WEB'S OBJECTIVE IS DIFFERENT, AND IT IS OPT-IN.

   The aft web goes as far aft as practical while retaining useful section
   depth, not at maximum local thickness. "Retaining useful section depth"
   is enforced by the feasibility oracle itself: the skin-margin /
   thin-section checks a too-thin station fails are what "useful section
   depth" means operationally here, and a SEPARATE thickness-ratio threshold
   would be a second free parameter with no measurement behind it (the same
   objection as to a weighted main-web objective above). So the aft search
   ranks candidates by chord fraction DESCENDING (rightmost first) within its
   own window and returns the first one that clears the SAME full-depth
   feasibility test the main web does, at least ``min_separation_frac`` aft
   of the chosen main station.

   ``aft_policy=None`` (the default) skips aft-web selection entirely,
   mirroring the optional shape of ``StructureSpec.families`` and the
   fail-closed doctrine: a caller who did not ask for an aft web gets none; a
   caller who DID ask and gets no qualifying station gets ``PlacementError``,
   never a silently omitted member (see point 4).

4. FAILURE: NO STATION QUALIFIES.

   ``PlacementError`` (a ``ValueError`` subclass) names the window searched,
   how many candidate stations were probed, how many sampled layers each was
   checked against, and a breakdown of WHY every candidate failed (aggregated
   from the exact ``OmittedWeb.reason`` strings / ``ValueError`` messages the
   generator itself produced -- never a new, independently worded diagnosis).
   A wing with no main web is not a wing; this module never degrades to
   "skip the family" or "fall back to the window's centre regardless of fit."
"""

from dataclasses import dataclass, replace
from typing import List, Optional, Sequence, Tuple

from .frame import LayerFrame, local_lower_worst_case, local_upper_worst_case
from .generator import LayerContext, StructureGenerator, WebIntent
from .spec import ROLE_PRIMARY_STRUCTURE, StructureSpec, WebFamily


class PlacementError(ValueError):
    """No chord station satisfies a ``MainWebPolicy``/``AftWebPolicy``
    search -- see this module's docstring, point 4, for why this is a fatal
    error rather than a silently-omitted family."""


@dataclass(frozen=True)
class MainWebPolicy:
    """Search parameters for the geometry-aware MAIN full-depth web.

    ``chord_window``: (min, max) chord fraction the search is restricted to
        -- see this module's docstring, point 1, for why windowed rather
        than unconstrained. Both values must lie in [0, 1] with
        ``min < max``.
    ``station_step_frac``: chord-fraction spacing of the search grid across
        the window (0.01 = roughly 1% of chord). Finer values cost more
        feasibility probes for a more precise objective peak; coarser
        values are cheaper. This is a SEARCH resolution, not a placement
        tolerance -- the winning station is exactly one of the grid points,
        not interpolated.
    ``entry_lower``: which exterior surface the resulting family enters
        from -- mirrors ``WebFamily.entry_lower`` exactly (``False``, the
        default, is the +Y/upper entry every existing primary family uses).
    ``name``: the produced ``WebFamily.name``.
    """

    chord_window: Tuple[float, float] = (0.15, 0.40)
    station_step_frac: float = 0.01
    entry_lower: bool = False
    name: str = "main"


@dataclass(frozen=True)
class AftWebPolicy:
    """Search parameters for the OPTIONAL aft full-depth web -- see this
    module's docstring, point 3, for why its objective (rightmost feasible)
    differs from the main web's (deepest feasible).

    ``search_window``: (min, max) chord fraction the search is restricted
        to before the main-web separation floor (below) is applied. Default
        (0.45, 0.95) starts just aft of the main window's own upper edge
        and stops short of the trailing edge, where ``local_span`` probes
        become numerically marginal on most airfoils.
    ``min_separation_frac``: minimum chord-fraction gap this search enforces
        aft of the ALREADY-CHOSEN main station, regardless of what
        ``search_window`` alone would allow -- two full-depth webs placed
        on top of each other are not a two-web structure.
    ``station_step_frac``/``entry_lower``/``name``: see ``MainWebPolicy``.
    """

    search_window: Tuple[float, float] = (0.45, 0.95)
    min_separation_frac: float = 0.05
    station_step_frac: float = 0.01
    entry_lower: bool = False
    name: str = "aft"


@dataclass(frozen=True)
class WebSelection:
    """One resolved primary web: the finished, ready-to-ship ``WebFamily``
    plus the measurements that justify it."""

    family: WebFamily
    station_frac: float
    local_thickness_mm: float
    stations_probed: int
    layers_probed: int


@dataclass(frozen=True)
class PlacementReport:
    """Result of ``select_primary_webs``: the main selection, always
    present, and the aft selection, present only when ``aft_policy`` was
    given."""

    main: WebSelection
    aft: Optional[WebSelection]


def _station_grid(lo: float, hi: float, step: float) -> List[float]:
    if step <= 0:
        raise ValueError("station_step_frac must be > 0, got {!r}.".format(step))
    if not lo < hi:
        raise ValueError(
            "window must satisfy min < max, got ({!r}, {!r}).".format(lo, hi)
        )
    n = max(1, round((hi - lo) / step))
    return [lo + i * (hi - lo) / n for i in range(n + 1)]


def _sample_frames(
    frames: Sequence[Tuple[LayerContext, LayerFrame]], stride: int
) -> List[Tuple[LayerContext, LayerFrame]]:
    if stride < 1:
        raise ValueError("layer_stride must be >= 1, got {!r}.".format(stride))
    sampled = list(frames[::stride])
    last = frames[-1]
    if not sampled or sampled[-1] is not last:
        sampled.append(last)
    return sampled


def _probe_station(
    generator: StructureGenerator,
    spec: StructureSpec,
    family: WebFamily,
    frame: LayerFrame,
    layer: LayerContext,
    outer_wall_line_width_mm: float,
    safety_offset_mm: Optional[float],
    max_rotation_deg: Optional[float],
) -> Tuple[Optional[WebIntent], Optional[str]]:
    """Feasibility oracle: does a single-station full-depth family at
    ``family.web_position`` survive ``StructureGenerator.generate()`` on this
    one layer? Reuses the REAL generator -- never a reimplementation of the
    skin-margin/thin-section/rotation checks -- via a
    probe spec carrying exactly this one family, so every other field of
    ``spec`` (skin_margin_mm, expected_hole_count, max_rotation_deg...) still
    applies exactly as it would for a real slice.

    Returns ``(intent, None)`` on success, or ``(None, reason)`` on
    failure, where ``reason`` is either the exact ``OmittedWeb.reason`` the
    generator itself produced, or the ``str()`` of a ``ValueError`` the
    generator raised for this single explicit station (e.g. a hole-footprint
    conflict, or a rotation-envelope breach) -- never an independently
    -worded diagnosis of what went wrong.
    """
    probe_spec = replace(spec, families=(family,))
    try:
        result = generator.generate(
            frame,
            probe_spec,
            outer_wall_line_width_mm,
            layer,
            safety_offset_mm=safety_offset_mm,
            max_rotation_deg=max_rotation_deg,
        )
    except ValueError as exc:
        return None, str(exc)
    if result.intents:
        return result.intents[0], None
    if result.omitted:
        return None, result.omitted[0].reason
    return None, "generator produced neither an intent nor an omission (unexpected)"


def _categorize_reason(reason: Optional[str]) -> str:
    if reason is None:
        return "unknown"
    if reason.startswith("skin margin"):
        return "skin margin"
    if reason.startswith("thin section"):
        return "thin section"
    if reason.startswith("chord extent"):
        return "chord extent"
    if reason.startswith("hole footprint"):
        return "hole footprint"
    return "other: {}".format(reason)


def _select_web(
    *,
    spec: StructureSpec,
    generator: StructureGenerator,
    sampled_frames: Sequence[Tuple[LayerContext, LayerFrame]],
    root_frame: LayerFrame,
    outer_wall_line_width_mm: float,
    family_name: str,
    window: Tuple[float, float],
    step_frac: float,
    entry_lower: bool,
    safety_offset_mm: Optional[float],
    max_rotation_deg: Optional[float],
    rank_by: str,
) -> WebSelection:
    lo, hi = window
    if not (0.0 <= lo and hi <= 1.0):
        raise ValueError(
            "window {!r} must lie within [0, 1].".format(window)
        )
    probe_template = WebFamily(
        name=family_name,
        web_position=0.5,
        rib_pitch_mm=None,
        web_depth_mm=None,
        entry_lower=entry_lower,
        role=ROLE_PRIMARY_STRUCTURE,
    )
    width_mm = probe_template.resolve_gap(spec, outer_wall_line_width_mm)

    fracs = _station_grid(lo, hi, step_frac)
    candidates: List[Tuple[float, float]] = []
    for frac in fracs:
        x = root_frame.web_x(frac)
        try:
            thickness = local_upper_worst_case(
                root_frame.ops, root_frame.polygon, x, width_mm
            ) - local_lower_worst_case(
                root_frame.ops, root_frame.polygon, x, width_mm
            )
        except ValueError:
            continue
        candidates.append((frac, thickness))

    if not candidates:
        raise PlacementError(
            "family {!r}: no chord station in window [{:.4f}, {:.4f}] has "
            "any measurable material at the root section (every probe in "
            "the window missed the section entirely).".format(
                family_name, lo, hi
            )
        )

    if rank_by == "deepest":
        candidates.sort(key=lambda c: c[1], reverse=True)
    elif rank_by == "aftmost":
        candidates.sort(key=lambda c: c[0], reverse=True)
    else:
        raise ValueError("rank_by must be 'deepest' or 'aftmost', got {!r}.".format(rank_by))

    failures: List[Tuple[float, Optional[str]]] = []
    for frac, thickness in candidates:
        family = replace(probe_template, web_position=frac)
        ok = True
        reason = None
        for layer, frame in sampled_frames:
            intent, station_reason = _probe_station(
                generator,
                spec,
                family,
                frame,
                layer,
                outer_wall_line_width_mm,
                safety_offset_mm,
                max_rotation_deg,
            )
            if intent is None:
                ok = False
                reason = station_reason
                break
        if ok:
            return WebSelection(
                family=family,
                station_frac=frac,
                local_thickness_mm=thickness,
                stations_probed=len(candidates),
                layers_probed=len(sampled_frames),
            )
        failures.append((frac, reason))

    categories: dict = {}
    for _frac, reason in failures:
        cat = _categorize_reason(reason)
        categories[cat] = categories.get(cat, 0) + 1
    breakdown = ", ".join(
        "{}: {}".format(cat, count) for cat, count in sorted(categories.items())
    )
    example_frac, example_reason = failures[0]
    raise PlacementError(
        "family {!r}: no chord station in window [{:.4f}, {:.4f}] survives "
        "the full-depth feasibility check on all {} sampled layer(s) -- "
        "{} candidate station(s) probed, every one failed. Failure "
        "breakdown by first-failing-layer reason category: {}. Example: "
        "station x_frac={:.4f} failed with {!r}.".format(
            family_name,
            lo,
            hi,
            len(sampled_frames),
            len(candidates),
            breakdown,
            example_frac,
            example_reason,
        )
    )


def select_primary_webs(
    spec: StructureSpec,
    frames: Sequence[Tuple[LayerContext, LayerFrame]],
    outer_wall_line_width_mm: float,
    *,
    main_policy: MainWebPolicy = MainWebPolicy(),
    aft_policy: Optional[AftWebPolicy] = None,
    safety_offset_mm: Optional[float] = None,
    max_rotation_deg: Optional[float] = None,
    layer_stride: int = 1,
    objective_layer_index: int = 0,
) -> PlacementReport:
    """Select the main (and, optionally, aft) primary full-depth web
    station(s) for one PART -- see this module's docstring for the full
    policy and its justification.

    ``frames`` must be ``(LayerContext, LayerFrame)`` pairs for every body
    layer of the part, in root-to-tip Z order (``frames[0]`` is treated as
    the representative "root" section for the objective -- see
    ``objective_layer_index`` to override). Must be non-empty.

    ``layer_stride`` subsamples ``frames`` for the (expensive) feasibility
    sweep -- every ``stride``-th frame, ALWAYS including the last frame
    regardless of stride, so a tapering tip is never skipped. ``1`` (the
    default) checks every layer.

    Raises ``PlacementError`` if no station qualifies for the main web, or
    (only when ``aft_policy`` is given) for the aft web -- see this module's
    docstring, point 4. Raises plain ``ValueError`` for malformed inputs
    (empty ``frames``, non-positive ``layer_stride``, an inverted window).
    """
    if not frames:
        raise ValueError("select_primary_webs: frames must be non-empty.")
    sampled = _sample_frames(frames, layer_stride)
    generator = StructureGenerator()
    _root_layer, root_frame = frames[objective_layer_index]

    main_selection = _select_web(
        spec=spec,
        generator=generator,
        sampled_frames=sampled,
        root_frame=root_frame,
        outer_wall_line_width_mm=outer_wall_line_width_mm,
        family_name=main_policy.name,
        window=main_policy.chord_window,
        step_frac=main_policy.station_step_frac,
        entry_lower=main_policy.entry_lower,
        safety_offset_mm=safety_offset_mm,
        max_rotation_deg=max_rotation_deg,
        rank_by="deepest",
    )

    aft_selection = None
    if aft_policy is not None:
        lo, hi = aft_policy.search_window
        floor = main_selection.station_frac + aft_policy.min_separation_frac
        effective_lo = max(lo, floor)
        if effective_lo >= hi:
            raise PlacementError(
                "family {!r}: aft search window [{:.4f}, {:.4f}] leaves no "
                "room once the main web's station {:.4f} plus "
                "min_separation_frac={:.4f} (floor {:.4f}) is "
                "applied.".format(
                    aft_policy.name,
                    lo,
                    hi,
                    main_selection.station_frac,
                    aft_policy.min_separation_frac,
                    effective_lo,
                )
            )
        aft_selection = _select_web(
            spec=spec,
            generator=generator,
            sampled_frames=sampled,
            root_frame=root_frame,
            outer_wall_line_width_mm=outer_wall_line_width_mm,
            family_name=aft_policy.name,
            window=(effective_lo, hi),
            step_frac=aft_policy.station_step_frac,
            entry_lower=aft_policy.entry_lower,
            safety_offset_mm=safety_offset_mm,
            max_rotation_deg=max_rotation_deg,
            rank_by="aftmost",
        )

    return PlacementReport(main=main_selection, aft=aft_selection)


def deepest_station_frac(
    frame: LayerFrame,
    window: Tuple[float, float],
    width_mm: float,
    step_frac: float = 0.01,
) -> Optional[Tuple[float, float]]:
    """Verification helper: the raw (unconstrained-by-feasibility) argmax
    local-thickness station within ``window`` at a SINGLE layer's own
    frame, or ``None`` if no grid point in the window has any measurable
    material. Not used by ``select_primary_webs`` itself (which measures
    only the representative root frame for its objective -- see this
    module's docstring, point 2) -- this exists so a caller can measure how
    much the per-layer argmax WANDERS along a span, as evidence for the
    per-part-not-per-layer design decision.
    """
    lo, hi = window
    best = None
    for frac in _station_grid(lo, hi, step_frac):
        x = frame.web_x(frac)
        try:
            thickness = local_upper_worst_case(
                frame.ops, frame.polygon, x, width_mm
            ) - local_lower_worst_case(frame.ops, frame.polygon, x, width_mm)
        except ValueError:
            continue
        if best is None or thickness > best[1]:
            best = (frac, thickness)
    return best
