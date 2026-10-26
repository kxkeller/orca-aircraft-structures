"""Weight-vs-stiffness sweep on a real wing STL (set via AIRCRAFT_WING_STL),
through the real offline pipeline: MeshSectioner -> StructureGenerator ->
SlotBuilder -> real shapely diff -> TopologyValidator, exactly the sequence
capability.py runs at posSlice, minus the Orca host itself.

ANALYSIS ONLY. Does not modify tools/, plugin/, aircraft_core/, or
host/data/orca_plugins/config.json. Every config variant is built with
dataclasses.replace() in memory.

Config variation is applied to a copy of the LIVE config
(tools/m34_rotation_equivalence.load_spec(), which reads
host/data/orca_plugins/config.json -- the same file the shipped plugin
reads) -- never to the file itself.

Mass estimate: this harness has the REAL diffed replacement polygons per
layer (repl, from ops.diff(poly, slots)), so it computes an HONEST cut-path
length directly from that geometry -- not the RunDiagnostics 2*depth proxy
alone (which is also recorded, as web_mass_fraction_est, since that is the
standard project figure and the two should be compared). Per layer:
    skin_perimeter_mm  = perimeter of the un-slotted section polygon
    replacement_perimeter_mm = sum of perimeter (contour + holes) of every
                                piece in repl
    web_path_mm = replacement_perimeter_mm - skin_perimeter_mm
This is the extra wall LENGTH the slots add to the toolpath (each slot cuts
a detour in, around, and back out of the skin loop -- same physical picture
as diagnostics.py's own docstring, but measured from the actual boolean
result instead of assumed as 2*depth per web). Mass:
    layer_mass_g = (skin_perimeter_mm + web_path_mm) * wall_mm * layer_height_mm
                   * PLA_DENSITY_G_PER_MM3
Summed over all ok layers. Web-only mass uses web_path_mm alone. This
still assumes a single-line-width spiral wall (true for spiral vase, wall_loops=1,
the shipped config) and ignores the small length actually displaced by
the slot's own footprint on the skin loop (same second-order effect
diagnostics.py's docstring names).
"""
import argparse
import json
import math
import os
import re
import sys
import time
from collections import Counter
from dataclasses import replace

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO_ROOT, os.path.join(REPO_ROOT, "tools"),
           os.path.join(REPO_ROOT, "tests", "integration")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aircraft_core.diagnostics import RunDiagnostics  # noqa: E402
from aircraft_core.frame import (  # noqa: E402
    LayerFrame,
    corrected_skin_margin_mm,
    local_lower_worst_case,
    local_upper_worst_case,
)
from aircraft_core.generator import DetectedChannel, LayerContext, StructureGenerator  # noqa: E402
from aircraft_core.mesh_section import MeshSectioner  # noqa: E402
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.spec import ROLE_PRIMARY_STRUCTURE, WebFamily  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

from m18_mesh_probe import read_binary_stl  # noqa: E402
from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402
import fake_orca  # noqa: E402
import m34_rotation_equivalence as m34  # noqa: E402

# The sweep needs a real wing STL that is not part of this repository; supply
# its path in this environment variable.
WING_STL_ENV = "AIRCRAFT_WING_STL"


def _wing_stl_path():
    path = os.environ.get(WING_STL_ENV)
    if not path:
        raise SystemExit(
            "this sweep needs a real wing STL: set %s to its path" % WING_STL_ENV)
    return path

LAYER_HEIGHT_MM = m34.LAYER_HEIGHT_MM
WALL_MM = m34.WALL_MM
PLA_DENSITY_G_PER_MM3 = 1.24e-3  # generic PLA, ~1.24 g/cm^3 -- stated assumption

# Measured bond ladder: a 0.021 mm modelled wall-to-wall gap prints fused
# ("takes real effort to pull apart"); 0.042 mm is separable by hand; 0.105 mm
# has no bond. 0.021 mm is the "genuinely fused" gate for a skin-to-skin tie
# -- NOT the ~1.0 mm skin_margin_mm standoff a lone full-depth web stops short
# of the opposite skin by (that standoff is Tier 1 below, "stiffened skin", not
# a connection).
FUSE_THRESHOLD_MM = 0.021

# ``_generate_opposing_pair_family``'s DEFAULT ``pair_offset_mm`` is
# constructed so ``wall_gap`` equals ``FUSE_THRESHOLD_MM`` EXACTLY in exact
# arithmetic (``offset_mm = 2*outer_wall_line_width_mm + slot_width +
# FUSE_THRESHOLD_MM``, so ``wall_gap = offset_mm - slot_width - 2*wall_mm ==
# FUSE_THRESHOLD_MM``). The FLOATING-POINT ``wall_gap``
# straddles ``0.021`` by ~1e-15 mm depending on the layer's own anchor_x
# magnitude (rounding noise in ``anchor_x -+ offset_mm/2``), so a bare ``<=``
# comparison against the exact constant would flip essentially at random from
# one layer to the next. That is not a real geometric effect: the true pair's
# wall_gap is a family-level constant, independent of layer, so every layer
# where both members are built must report the identical tie. 1e-9 mm is ~1e6x the observed ~1e-15 mm noise and ~1e7x
# smaller than any real geometric quantity this module measures, so it swallows
# the float noise without softening the gate for any genuinely non-fusing
# candidate.
_TIE_GATE_EPSILON_MM = 1e-9


def load_spec_from_path(config_path):
    """Build a StructureSpec from an arbitrary cap_config JSON file, the
    same shape m34.load_spec() reads from the live path -- factored out so
    a sweep row can use a NON-live config without touching m34's own hardcoded LIVE_CONFIG_PATH or load_spec()."""
    with open(config_path, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    entry = doc["config"][0] if isinstance(doc["config"], list) else doc["config"]
    return m34.sa.build_spec_from_cap_config(entry["cap_config"])

# Variant of m34._RE_OPPOSING_ENTRY that ALSO captures each side's entry
# surface ("+Y"/"-Y"), which m34's own regex discards -- needed to answer "is
# it the lower skin that loses more often" directly from the reason string.
# Matches the SAME "ordinary
# shallower loses" branch only (generator.py's non-scar-priority "opposing
# entry: ... meets family ...'s candidate ..." text) -- excludes the scar
# -priority and "could not be measured" branches, exactly as m34's does.
_RE_OPPOSING_ENTRY_SIDED = re.compile(
    r"opposing entry: candidate at x=(-?[\d.]+) mm \(([+-]Y), depth ([\d.]+) mm\) "
    r"meets family .*?candidate at x=(-?[\d.]+) mm \(([+-]Y), depth ([\d.]+) mm\) "
    r"over the overlap span")


def kai_transform(verts):
    """Stand-up convention (-z, y, x), then drop-to-bed -- the same two steps
    as m34's real-wing mode and m34.run_arm's own _drop_to_bed call, applied in
    that order (rotation is about Y and does not touch the drop-to-bed
    normalization's assumptions)."""
    rotated = [(-v[2], v[1], v[0]) for v in verts]
    return m34._drop_to_bed(rotated)


def build_spec_variant(base_spec, axis, value):
    """axis in {"lattice_pitch", "lattice_depth", "deep_pitch", "drop_families",
    "combo", None}.
    None (or value None) returns base_spec unchanged (the shipped baseline).
    "drop_families": value is a tuple of family names to REMOVE from
    base_spec.families (name match on str(f.name)).
    "combo": value is a list of (axis, value) pairs applied in sequence to
    base_spec, so e.g. a pitch change and a family drop can be combined in
    one row."""
    if axis is None:
        return base_spec
    if axis == "combo":
        spec = base_spec
        for sub_axis, sub_value in value:
            spec = build_spec_variant(spec, sub_axis, sub_value)
        return spec
    if axis == "drop_families":
        names_to_drop = set(value)
        new_families = tuple(
            f for f in base_spec.families if str(f.name) not in names_to_drop)
        return replace(base_spec, families=new_families)
    if axis == "skin_margin_mm":
        # Harness-only override of the SPEC-level (not family-level)
        # skin_margin_mm, applied in memory only -- never touches
        # presets/plugin/live_host_config.json or
        # host/data/orca_plugins/config.json.
        return replace(base_spec, skin_margin_mm=value)
    if axis == "add_families":
        # opposing_pair rows: value is a tuple of WebFamily
        # instances to APPEND, unchanged, to base_spec.families -- e.g. an
        # ``opposing_pair=True`` family built directly by the caller. Never
        # mutates or reorders the existing families, so every prior axis's
        # own family-matching-by-name-prefix logic below is unaffected.
        return replace(base_spec, families=tuple(base_spec.families) + tuple(value))
    new_families = []
    for f in base_spec.families:
        name = str(f.name)
        if axis == "lattice_pitch" and name.startswith("lattice_"):
            f = replace(f, rib_pitch_chord_frac=value)
        elif axis == "lattice_depth" and name.startswith("lattice_"):
            f = replace(f, web_depth_mm=value)
        elif axis == "deep_pitch" and name.startswith("deep_"):
            f = replace(f, rib_pitch_chord_frac=value)
        new_families.append(f)
    return replace(base_spec, families=tuple(new_families))


def sweep_one_config(label, spec, verts, tris, wall_mm, cap, ops):
    """One config's full layer sweep. Returns a result dict. Mirrors
    m34._sweep_layers's control flow (root/skip/ok/failed accounting) but
    additionally: (a) feeds a fresh RunDiagnostics via record_structure_layer
    with the SAME arguments capability.py itself passes, and (b) computes the
    honest perimeter-based mass estimate described in the module docstring.
    """
    sec = MeshSectioner(verts, tris)
    z_min, z_max = sec.z_min, sec.z_max
    root = int(spec.root_solid_layers or 0)
    n_layers = int(round(z_max / LAYER_HEIGHT_MM))

    gen, builder = StructureGenerator(), SlotBuilder()
    validator = TopologyValidator(ops, safety_offset_mm=wall_mm)
    diag = RunDiagnostics()

    built_by_role = Counter()
    omitted_by_role = Counter()
    opposing_entry_events = []  # (winner_depth, loser_depth) mm, winner always deeper/equal
    failures = []
    ok = 0
    skipped_out_of_z_range = 0
    skipped_no_outer_loop = 0

    skin_mass_g = 0.0
    web_mass_g = 0.0
    # Per-surface worst bay: re-implements
    # diagnostics.py's own per-surface grouping (entry_lower splits upper
    #/lower) locally, since RunDiagnostics.worst_skin_bay_mm only exposes
    # the single global worst across both surfaces.
    worst_bay_upper_mm = None
    worst_bay_lower_mm = None
    built_count_upper = 0
    built_count_lower = 0

    # Per-layer tie-gap records, one per
    # ok layer -- see ``_layer_tie_metrics``'s own docstring for the metric.
    tie_layer_records = []

    family_roles_by_index = {i: f.role for i, f in enumerate(spec.effective_families())}

    t0 = time.perf_counter()
    for i in range(root, n_layers):
        z = i * LAYER_HEIGHT_MM
        if z <= z_min or z >= z_max:
            skipped_out_of_z_range += 1
            continue
        try:
            loops = sec.section_at(z)
            outer = [lp for lp in loops if not lp.is_interior]
            inner = [lp for lp in loops if lp.is_interior]
            if not outer:
                skipped_no_outer_loop += 1
                continue
            poly = ops.polygon_from_points(
                max(outer, key=lambda lp: abs(lp.signed_area)).points)
            frame = LayerFrame.from_polygon(ops, poly)
            channels = tuple(
                DetectedChannel(lp.centroid[0], lp.centroid[1],
                                 lp.equivalent_diameter_mm) for lp in inner)
            res = gen.generate(
                frame, spec, wall_mm,
                LayerContext(layer_index=i, print_z=z, detected_channels=channels),
                safety_offset_mm=wall_mm)

            for it in res.intents:
                for fam_idx in (it.merged_family_indices or (it.family_index,)):
                    built_by_role[family_roles_by_index.get(fam_idx, "?")] += 1
            for o in res.omitted:
                omitted_by_role[family_roles_by_index.get(o.family_index, "?")] += 1
                m = _RE_OPPOSING_ENTRY_SIDED.search(o.reason)
                if m:
                    loser_side, loser_depth = m.group(2), float(m.group(3))
                    winner_side, winner_depth = m.group(5), float(m.group(6))
                    opposing_entry_events.append(
                        (winner_depth, loser_depth, loser_side, winner_side))

            slots = [ops.polygon_from_points(
                builder.build(it, spec.overshoot_mm).points)
                for it in res.intents]
            repl = ops.diff(poly, slots)
            validator.validate(
                [poly], repl, slots, spec, i, z,
                slot_labels=[str(it.family_name) for it in res.intents],
                family_indices=[it.family_membership() for it in res.intents])

            # --- honest perimeter-based mass (module docstring) ---
            skin_perimeter_mm = _contour_perimeter_mm(ops, poly)
            repl_perimeter_mm = sum(_piece_perimeter_mm(ops, piece) for piece in repl)
            web_path_mm = max(0.0, repl_perimeter_mm - skin_perimeter_mm)
            skin_mass_g += skin_perimeter_mm * wall_mm * LAYER_HEIGHT_MM * PLA_DENSITY_G_PER_MM3
            web_mass_g += web_path_mm * wall_mm * LAYER_HEIGHT_MM * PLA_DENSITY_G_PER_MM3

            # --- RunDiagnostics, same call shape as capability.py ---
            effective_families = spec.effective_families()
            diag.record_structure_layer(
                layer_index=i,
                print_z=z,
                skin_length_mm=skin_perimeter_mm,
                web_depths_mm=[_intent_depth_mm(it) for it in res.intents],
                entry_x_mm=[it.start[0] for it in res.intents],
                entry_lower_flags=[bool(it.entry_lower) for it in res.intents],
                full_depth_web_count=sum(
                    1 for it in res.intents
                    if _is_full_depth_intent(it, effective_families)
                ),
            )

            # --- tie-gap metric ---
            effective_skin_margin_mm = corrected_skin_margin_mm(
                spec.skin_margin_mm, frame.rotation_deg)
            tie_layer_records.append(_layer_tie_metrics(
                res, effective_families, frame, wall_mm, ops, poly,
                effective_skin_margin_mm, i, z))

            upper_xs = sorted(it.start[0] for it in res.intents if not it.entry_lower)
            lower_xs = sorted(it.start[0] for it in res.intents if it.entry_lower)
            built_count_upper += len(upper_xs)
            built_count_lower += len(lower_xs)
            for lo, hi in zip(upper_xs, upper_xs[1:]):
                gap = hi - lo
                if worst_bay_upper_mm is None or gap > worst_bay_upper_mm:
                    worst_bay_upper_mm = gap
            for lo, hi in zip(lower_xs, lower_xs[1:]):
                gap = hi - lo
                if worst_bay_lower_mm is None or gap > worst_bay_lower_mm:
                    worst_bay_lower_mm = gap

            ok += 1
        except TopologyError as exc:
            failures.append({"layer": i, "z": round(z, 4), "kind": "TopologyError",
                              "msg": str(exc)[:300]})
        except Exception as exc:  # noqa: BLE001
            failures.append({"layer": i, "z": round(z, 4),
                              "kind": type(exc).__name__, "msg": str(exc)[:300]})
    elapsed = time.perf_counter() - t0

    n_skipped = skipped_out_of_z_range + skipped_no_outer_loop
    n_accounted = root + ok + len(failures) + n_skipped
    reconciled = (n_accounted == n_layers)

    tie_summary = _summarize_tie_layer_records(tie_layer_records)

    return {
        "label": label,
        "n_layers": n_layers,
        "root": root,
        "ok": ok,
        "failed": len(failures),
        "failures_sample": failures[:10],
        "skipped_out_of_z_range": skipped_out_of_z_range,
        "skipped_no_outer_loop": skipped_no_outer_loop,
        "reconciled": reconciled,
        "built_by_role": dict(built_by_role),
        "omitted_by_role": dict(omitted_by_role),
        "total_built": sum(built_by_role.values()),
        "total_omitted": sum(omitted_by_role.values()),
        "opposing_entry_event_count": len(opposing_entry_events),
        "opposing_entry_events": opposing_entry_events,
        "web_mass_fraction_est": diag.web_mass_fraction_est,
        "worst_skin_bay_mm": diag._worst_skin_bay_mm,
        "worst_skin_bay_layer_index": diag._worst_skin_bay_layer_index,
        "worst_bay_upper_mm": worst_bay_upper_mm,
        "worst_bay_lower_mm": worst_bay_lower_mm,
        "built_count_upper": built_count_upper,
        "built_count_lower": built_count_lower,
        "tie_gap_metrics": tie_summary,
        "tie_layer_records": tie_layer_records,
        "mean_web_spacing_mm": diag.mean_web_spacing_mm,
        "skin_mass_g_est": skin_mass_g,
        "web_mass_g_est": web_mass_g,
        "total_mass_g_est": skin_mass_g + web_mass_g,
        "web_mass_fraction_perimeter_est": (
            web_mass_g / (skin_mass_g + web_mass_g) if (skin_mass_g + web_mass_g) > 0 else None
        ),
        "elapsed_s": elapsed,
    }


def _contour_perimeter_mm(ops, polygon):
    points = ops.contour_points_mm(polygon)
    if len(points) < 2:
        return 0.0
    total = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def _piece_perimeter_mm(ops, piece):
    total = _contour_perimeter_mm(ops, piece)
    for hole in ops.hole_points_mm(piece):
        if len(hole) < 2:
            continue
        for (x0, y0), (x1, y1) in zip(hole, hole[1:] + hole[:1]):
            total += math.hypot(x1 - x0, y1 - y0)
    return total


def _intent_depth_mm(intent):
    (x0, y0), (x1, y1) = intent.start, intent.end
    return math.hypot(x1 - x0, y1 - y0)


def _is_full_depth_intent(intent, families):
    member_idxs = intent.merged_family_indices or (intent.family_index,)
    for idx in member_idxs:
        if idx < 0 or idx >= len(families):
            return False
        if families[idx].web_depth_mm is not None:
            return False
    return True


def _layer_tie_metrics(res, effective_families, frame, wall_mm, ops, poly,
                        skin_margin_mm, layer_index, z):
    """One layer's skin-to-skin tie geometry, in two tiers. Reads ONLY ``res.intents`` (already-built webs) plus a handful of
    ``local_*_worst_ case`` probes; never guesses connectivity from family
    names.

    Tier 1 -- "full-depth fin": any built intent whose full member set has
    every ``web_depth_mm is None`` (``_is_full_depth_intent`` -- this is a real
    geometric consequence of that flag, not a name match: it makes
    ``StructureGenerator`` place the far end at ``local_far_worst_case +
    skin_margin_mm`` (+ 2e-4 mm headroom), i.e. ``skin_margin_mm`` short of the
    OPPOSITE skin -- see ``FULL_DEPTH_MARGIN_HEADROOM_MM``'s docstring). This
    is "stiffened skin" -- NOT a connection to the opposite skin by itself.

    Tier 2 -- "fused tie": a pair of Tier-1 fins (opposite ``entry_lower``,
    fin-to-fin) OR a Tier-1 fin plus the spar-socket keyhole's own boundary
    -connected circle wall (fin-to-socket), whose WALL-TO-WALL gap is
    <= FUSE_THRESHOLD_MM (measured hard-fuse point), the predicted-fused
    test. A fin-to-socket chain only counts
    when BOTH an upper-entry and a lower-entry fin each independently fuse
    to the circle (the circle wall is one continuous printed loop around
    the void, so two independent fuses to it complete a skin-to-skin
    printed path: upper skin -> fin -> circle wall -> fin -> lower skin).
    This is PREDICTED from geometry and the measured bond ladder -- not
    measured from a real print.
    """
    full_depth = [it for it in res.intents
                  if _is_full_depth_intent(it, effective_families)]
    uppers = [it for it in full_depth if not it.entry_lower]
    lowers = [it for it in full_depth if it.entry_lower]
    socket_it = next((it for it in res.intents
                       if it.socket_radius_mm is not None), None)

    tier1_records = []
    for it in full_depth:
        x = it.start[0]
        try:
            if it.entry_lower:
                far_y = local_upper_worst_case(
                    ops, poly, x, it.width_mm, hole_aware=True)
                near_y = local_lower_worst_case(
                    ops, poly, x, it.width_mm, hole_aware=True)
            else:
                far_y = local_lower_worst_case(
                    ops, poly, x, it.width_mm, hole_aware=True)
                near_y = local_upper_worst_case(
                    ops, poly, x, it.width_mm, hole_aware=True)
        except ValueError:
            continue
        thickness_mm = abs(near_y - far_y)
        depth_mm = _intent_depth_mm(it)
        residual_mm = thickness_mm - skin_margin_mm - depth_mm
        tier1_records.append({"x_mm": x, "depth_mm": depth_mm,
                               "thickness_mm": thickness_mm,
                               "residual_mm": residual_mm})

    tie_intervals = []  # list of (x_min, x_max, kind, detail)

    # fin-to-fin
    for u in uppers:
        for lo in lowers:
            xu, xl = u.start[0], lo.start[0]
            void_gap = abs(xu - xl) - u.width_mm / 2.0 - lo.width_mm / 2.0
            # Prune bound: a genuine opposing_pair's DEFAULT offset is
            # ``2 * outer_wall_line_width_mm + slot_width + FUSE_THRESHOLD_MM``
            # (generator.py's ``_generate_opposing_pair_family``), i.e. a
            # void_gap of roughly ``2 * wall_mm + FUSE_THRESHOLD_MM``. A bound
            # of ``5 * FUSE_THRESHOLD_MM + wall_mm`` would prune every real
            # pair candidate before its wall_gap was computed. 2 mm of
            # headroom above ``2 * wall_mm`` covers that plus a wider slot
            # width, while still cheaply excluding fins that are
            # chord-fractions apart.
            if void_gap > 2.0 * wall_mm + 2.0:
                continue  # far apart; not a candidate pair, skip the probe
            wall_gap = void_gap - 2.0 * wall_mm
            if wall_gap <= FUSE_THRESHOLD_MM + _TIE_GATE_EPSILON_MM:
                tie_intervals.append((min(xu, xl), max(xu, xl), "fin_fin",
                                       {"wall_gap_mm": wall_gap}))

    # fin-to-socket-to-fin chain
    if socket_it is not None:
        cx, cy = socket_it.end
        r = socket_it.socket_radius_mm
        fused_upper = None
        fused_lower = None
        for u in uppers:
            void_gap = abs(u.start[0] - cx) - u.width_mm / 2.0 - r
            wall_gap = void_gap - 2.0 * wall_mm
            if wall_gap <= FUSE_THRESHOLD_MM + _TIE_GATE_EPSILON_MM and (
                    fused_upper is None or wall_gap < fused_upper[1]):
                fused_upper = (u, wall_gap)
        for lo in lowers:
            void_gap = abs(lo.start[0] - cx) - lo.width_mm / 2.0 - r
            wall_gap = void_gap - 2.0 * wall_mm
            if wall_gap <= FUSE_THRESHOLD_MM + _TIE_GATE_EPSILON_MM and (
                    fused_lower is None or wall_gap < fused_lower[1]):
                fused_lower = (lo, wall_gap)
        if fused_upper is not None and fused_lower is not None:
            xs = [fused_upper[0].start[0], fused_lower[0].start[0],
                  cx - r, cx + r]
            tie_intervals.append((min(xs), max(xs), "fin_socket_fin",
                                   {"wall_gap_upper_mm": fused_upper[1],
                                    "wall_gap_lower_mm": fused_lower[1]}))

    tie_intervals.sort(key=lambda t: t[0])
    le_x, te_x = frame.min_x, frame.max_x
    chord_mm = max(te_x - le_x, 1e-9)

    # gaps between LE, each tie's x_min (approaching from the near side),
    # and TE -- i.e. the "largest CHORDWISE gap between consecutive
    # skin-to-skin ties, with LE/TE as boundaries".
    # A tie occupies [x_min, x_max]; the gap to its neighbour is measured
    # from that neighbour's OWN near edge, so two overlapping ties never
    # report a negative gap.
    edges = [le_x]
    for x_min, x_max, _kind, _detail in tie_intervals:
        edges.append(x_min)
        edges.append(x_max)
    edges.append(te_x)
    gaps = []
    for a, b in zip(edges[::2], edges[1::2]):
        gaps.append(max(0.0, b - a))
    worst_gap_mm = max(gaps) if gaps else (te_x - le_x)
    worst_gap_idx = gaps.index(worst_gap_mm) if gaps else 0
    worst_gap_mid_x = (edges[2 * worst_gap_idx] + edges[2 * worst_gap_idx + 1]) / 2.0 \
        if gaps else (le_x + te_x) / 2.0
    worst_gap_chord_frac = (worst_gap_mid_x - le_x) / chord_mm

    # aft-of-spar gap: from the spar tie group's aft edge (the
    # fin_socket_fin interval if present, else the bare socket circle's own
    # footprint if a socket was built at all, else undefined) to the next
    # tie after it, or TE.
    spar_group = next((iv for iv in tie_intervals if iv[2] == "fin_socket_fin"),
                       None)
    if spar_group is not None:
        spar_aft_x = spar_group[1]
    elif socket_it is not None:
        spar_aft_x = socket_it.end[0] + socket_it.socket_radius_mm
    else:
        spar_aft_x = None

    aft_gap_mm = None
    if spar_aft_x is not None:
        next_tie_x = next((iv[0] for iv in tie_intervals if iv[0] > spar_aft_x),
                           te_x)
        aft_gap_mm = max(0.0, next_tie_x - spar_aft_x)

    return {
        "layer_index": layer_index,
        "z": z,
        "tier1_count": len(full_depth),
        "tier1_residuals_mm": [r["residual_mm"] for r in tier1_records],
        "tier2_tie_count": len(tie_intervals),
        "tie_intervals": [(round(a, 4), round(b, 4), k, d) for a, b, k, d in tie_intervals],
        "worst_gap_mm": worst_gap_mm,
        "worst_gap_mid_x_mm": worst_gap_mid_x,
        "worst_gap_chord_frac": worst_gap_chord_frac,
        "spar_aft_x_mm": spar_aft_x,
        "aft_gap_mm": aft_gap_mm,
    }


def _median(values):
    if not values:
        return None
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0


def _summarize_tie_layer_records(records):
    """Row-level rollup of ``_layer_tie_metrics`` per-layer records: worst
    chordwise gap (with its layer/Z/chord-fraction location), the
    span-wise median of the per-layer worst gap, and the same two figures
    for the aft-of-spar gap specifically. Also rolls up
    Tier-1 residual (how far short of the opposite skin a lone full-depth
    fin stops) and the Tier-2 fused-tie count per layer, so a reader can
    see how many "ties" and how much "just stiffened skin" each row built."""
    if not records:
        return None

    worst_rec = max(records, key=lambda r: r["worst_gap_mm"])
    aft_records = [r for r in records if r["aft_gap_mm"] is not None]
    worst_aft_rec = (max(aft_records, key=lambda r: r["aft_gap_mm"])
                      if aft_records else None)

    all_residuals = [res for r in records for res in r["tier1_residuals_mm"]]
    tier2_counts = [r["tier2_tie_count"] for r in records]

    # Verify the Tier-2 gate rather than only asserting it: kind counts and the
    # fin-socket wall gaps actually computed, checked against the measured
    # 0.006-0.016 mm band; and the Z range of zero-tie layers, since "same tie
    # count" claims must be checked layer by layer.
    kind_counts = Counter()
    wall_gap_upper = []
    wall_gap_lower = []
    wall_gap_fin_fin = []
    for r in records:
        for _a, _b, kind, detail in r["tie_intervals"]:
            kind_counts[kind] += 1
            if kind == "fin_socket_fin":
                wall_gap_upper.append(detail["wall_gap_upper_mm"])
                wall_gap_lower.append(detail["wall_gap_lower_mm"])
            elif kind == "fin_fin":
                wall_gap_fin_fin.append(detail["wall_gap_mm"])

    def _stats(vals):
        return ({"min": min(vals), "median": _median(vals), "max": max(vals),
                 "n": len(vals)} if vals else None)

    zero_tie_layers = [{"layer_index": r["layer_index"], "z": r["z"]}
                        for r in records if r["tier2_tie_count"] == 0]

    return {
        "tie_kind_counts": dict(kind_counts),
        "fin_socket_fin_wall_gap_mm": {
            "upper": _stats(wall_gap_upper),
            "lower": _stats(wall_gap_lower),
        },
        "fin_fin_wall_gap_mm": _stats(wall_gap_fin_fin),
        "zero_tie_layer_count": len(zero_tie_layers),
        "zero_tie_layers_z_range": (
            [min(r["z"] for r in zero_tie_layers),
             max(r["z"] for r in zero_tie_layers)]
            if zero_tie_layers else None),
        "zero_tie_layers": zero_tie_layers,
        "n_layers_measured": len(records),
        "worst_gap_mm": worst_rec["worst_gap_mm"],
        "worst_gap_layer_index": worst_rec["layer_index"],
        "worst_gap_z": worst_rec["z"],
        "worst_gap_chord_frac": worst_rec["worst_gap_chord_frac"],
        "median_worst_gap_mm": _median([r["worst_gap_mm"] for r in records]),
        "aft_gap_n_layers_with_spar": len(aft_records),
        "worst_aft_gap_mm": (worst_aft_rec["aft_gap_mm"]
                              if worst_aft_rec else None),
        "worst_aft_gap_layer_index": (worst_aft_rec["layer_index"]
                                       if worst_aft_rec else None),
        "worst_aft_gap_z": worst_aft_rec["z"] if worst_aft_rec else None,
        "median_aft_gap_mm": _median([r["aft_gap_mm"] for r in aft_records]),
        "tier1_full_depth_residual_mm": {
            "worst_mm": max(all_residuals) if all_residuals else None,
            "median_mm": _median(all_residuals),
            "n": len(all_residuals),
        },
        "tier2_tie_count_per_layer": {
            "median": _median(tier2_counts),
            "max": max(tier2_counts) if tier2_counts else None,
            "n_layers_with_zero_ties": sum(1 for c in tier2_counts if c == 0),
        },
    }


def run_grid(grid, out_path):
    if not HAVE_SHAPELY:
        raise SystemExit("shapely unavailable; refusing to report")

    verts_raw, tris = read_binary_stl(_wing_stl_path())
    verts = kai_transform(verts_raw)

    # canonicalization -- mirrors m34.run_arm(theta_in_deg=0.0) exactly
    # (bbox-center, detect theta, rotate to canonical frame if beyond epsilon).
    # Done ONCE here (not per config) since theta detection reads only the
    # mesh, never spec/family config, and every arm below must section the SAME
    # canonical mesh capability.py itself would slice.
    cx, cy = m34._mesh_bbox_center(verts)
    centered = [(v[0] - cx, v[1] - cy, v[2]) for v in verts]
    probe = MeshSectioner(centered, tris)
    theta_detected, theta_detail = m34._detect_theta(probe)
    canonical_verts = centered
    rotation_applied = False
    if theta_detected is not None and abs(theta_detected) >= m34.THETA_EPSILON_DEG:
        xy = m34.rotate_points_deg([(v[0], v[1]) for v in centered], -theta_detected)
        canonical_verts = [(x, y, v[2]) for (x, y), v in zip(xy, centered)]
        rotation_applied = True
    print(f"theta_detected={theta_detected} rotation_applied={rotation_applied}")
    verts = canonical_verts

    fo, restore = fake_orca.install()
    try:
        namespace = fake_orca.exec_bundle(
            os.path.join(REPO_ROOT, "plugin", "aircraft_structures", "aircraft_structures.py"))
        cap = namespace["AircraftStructuresM1"]()
        ops = MiterShapelyOps()
        base_spec = m34.load_spec()

        # print the live config once, for the record
        fam_dump = [(f.name, f.role, f.rib_pitch_chord_frac, f.web_depth_mm)
                    for f in base_spec.families]
        print("LIVE CONFIG FAMILIES:", fam_dump)

        results = []
        for label, axis, value in grid:
            print(f"--- running {label} (axis={axis} value={value}) ---")
            try:
                spec = build_spec_variant(base_spec, axis, value)
                r = sweep_one_config(label, spec, verts, tris, WALL_MM, cap, ops)
                r["axis"] = axis
                r["value"] = value
                print(f"  ok={r['ok']} failed={r['failed']} built={r['total_built']} "
                      f"omitted={r['total_omitted']} web_mass_fraction_est={r['web_mass_fraction_est']} "
                      f"worst_skin_bay_mm={r['worst_skin_bay_mm']} "
                      f"total_mass_g_est={r['total_mass_g_est']:.3f} "
                      f"elapsed={r['elapsed_s']:.1f}s")
            except Exception as exc:  # noqa: BLE001
                # A row-level (not per-layer) raise -- e.g. a spec/validator
                # rejection outside the per-layer try in sweep_one_config.
                # Record verbatim and continue with the rest of the grid.
                r = {
                    "label": label,
                    "axis": axis,
                    "value": value,
                    "ok": False,
                    "failed": True,
                    "row_exception": {
                        "kind": type(exc).__name__,
                        "msg": str(exc),
                    },
                }
                print(f"  ROW RAISED: {type(exc).__name__}: {exc}")
            results.append(r)
            # write incrementally so a session end doesn't lose completed arms
            with open(out_path, "w", encoding="utf-8") as fh:
                json.dump({
                    "live_config_families": fam_dump,
                    "theta_detected_deg": theta_detected,
                    "rotation_applied": rotation_applied,
                    "results": results,
                }, fh, indent=2, default=str)
    finally:
        restore()

    return results


PRE_CONFIG_D_PATH = os.path.join(
    REPO_ROOT, "docs", "results", "pre_config_d_live_host_config_20260913.json")


def run_grid_with_bases(grid, out_path, base_paths):
    """Like ``run_grid``, but each grid row names WHICH base config to
    start from (grid entries are ``(label, base_key, axis, value)``,
    ``base_key`` an entry in ``base_paths``, e.g. ``{"live": <live config
    path>, "pre_d": PRE_CONFIG_D_PATH}``) instead of always using the live
    config, for rows that need a different starting point (e.g. the
    pre-config-D deep-web families); ``run_grid``/``GRID`` above are
    independent of it. Mesh loading/canonicalization is copied from
    ``run_grid`` rather than shared, so ``run_grid`` is unaffected by changes
    here.
    """
    if not HAVE_SHAPELY:
        raise SystemExit("shapely unavailable; refusing to report")

    verts_raw, tris = read_binary_stl(_wing_stl_path())
    verts = kai_transform(verts_raw)

    cx, cy = m34._mesh_bbox_center(verts)
    centered = [(v[0] - cx, v[1] - cy, v[2]) for v in verts]
    probe = MeshSectioner(centered, tris)
    theta_detected, theta_detail = m34._detect_theta(probe)
    canonical_verts = centered
    rotation_applied = False
    if theta_detected is not None and abs(theta_detected) >= m34.THETA_EPSILON_DEG:
        xy = m34.rotate_points_deg([(v[0], v[1]) for v in centered], -theta_detected)
        canonical_verts = [(x, y, v[2]) for (x, y), v in zip(xy, centered)]
        rotation_applied = True
    print(f"theta_detected={theta_detected} rotation_applied={rotation_applied}")
    verts = canonical_verts

    fo, restore = fake_orca.install()
    try:
        namespace = fake_orca.exec_bundle(
            os.path.join(REPO_ROOT, "plugin", "aircraft_structures", "aircraft_structures.py"))
        cap = namespace["AircraftStructuresM1"]()
        ops = MiterShapelyOps()

        base_specs = {}
        fam_dumps = {}
        for key, path in base_paths.items():
            spec = (m34.load_spec() if key == "live" else load_spec_from_path(path))
            base_specs[key] = spec
            fam_dumps[key] = [
                (f.name, f.role, f.rib_pitch_chord_frac, f.web_depth_mm)
                for f in spec.families]
            print(f"BASE '{key}' FAMILIES ({path}):", fam_dumps[key])

        results = []
        for label, base_key, axis, value in grid:
            print(f"--- running {label} (base={base_key} axis={axis} value={value}) ---")
            try:
                spec = build_spec_variant(base_specs[base_key], axis, value)
                r = sweep_one_config(label, spec, verts, tris, WALL_MM, cap, ops)
                r["axis"] = axis
                r["value"] = value
                r["base_key"] = base_key
                print(f"  ok={r['ok']} failed={r['failed']} built={r['total_built']} "
                      f"omitted={r['total_omitted']} "
                      f"total_mass_g_est={r['total_mass_g_est']:.3f} "
                      f"worst_gap_mm={r['tie_gap_metrics']['worst_gap_mm'] if r['tie_gap_metrics'] else None} "
                      f"elapsed={r['elapsed_s']:.1f}s")
            except Exception as exc:  # noqa: BLE001
                r = {
                    "label": label,
                    "base_key": base_key,
                    "axis": axis,
                    "value": value,
                    "ok": False,
                    "failed": True,
                    "row_exception": {
                        "kind": type(exc).__name__,
                        "msg": str(exc),
                    },
                }
                print(f"  ROW RAISED: {type(exc).__name__}: {exc}")
            results.append(r)
            with open(out_path, "w", encoding="utf-8") as fh:
                json.dump({
                    "base_config_families": fam_dumps,
                    "base_config_paths": base_paths,
                    "theta_detected_deg": theta_detected,
                    "rotation_applied": rotation_applied,
                    "fuse_threshold_mm": FUSE_THRESHOLD_MM,
                    "results": results,
                }, fh, indent=2, default=str)
    finally:
        restore()

    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--out", default=os.path.join(
        os.path.dirname(__file__), "weight_stiffness_sweep_out.json"))
    p.add_argument("--only", default=None,
                   help="comma-separated label substrings to restrict the grid to")
    p.add_argument("--bg-rows", action="store_true",
                   help="Warp-tie sweep rows B-G "
                        "(pre-config-D deep-web base) instead of the "
                        "original GRID below; a wholly separate code path "
                        "via run_grid_with_bases, so the default invocation "
                        "and --only are unaffected.")
    p.add_argument("--pair-rows", action="store_true",
                   help="Warp-tie sweep rows "
                        "H-K (opposing_pair families added on top of the "
                        "live config-D base, i.e. row A) instead of the "
                        "original GRID below; a wholly separate code path, "
                        "so the default invocation, --only and --bg-rows "
                        "are unaffected.")
    p.add_argument("--web-depth-rows", action="store_true",
                   help="Tie-plus-web-depth rows L1/L2/M1/M2 "
                        "(lattice web_depth 1.0/1.5 on top of the live-config "
                        "margin-1.0 base A2, and again with the H2 0.60c "
                        "opposing_pair tie added) instead of the original "
                        "GRID below; a wholly separate code path via "
                        "run_grid_with_bases, so the default invocation, "
                        "--only, --bg-rows and --pair-rows are unaffected. "
                        "A2/H2 themselves are NOT re-run here (reuse "
                        "previously recorded H2/A2 results).")
    args = p.parse_args()

    if args.pair_rows:
        def _tie_family(name, web_position):
            return WebFamily(name=name, role=ROLE_PRIMARY_STRUCTURE,
                              web_position=web_position, opposing_pair=True)

        PAIR_GRID = [
            ("H_pair_0.60c", "live", "add_families",
             (_tie_family("tie_H_0.60c", 0.60),)),
            ("I_pair_0.50c_0.70c", "live", "add_families",
             (_tie_family("tie_I_0.50c", 0.50),
              _tie_family("tie_I_0.70c", 0.70))),
            ("J_pair_0.45c_0.60c_0.75c", "live", "add_families",
             (_tie_family("tie_J_0.45c", 0.45),
              _tie_family("tie_J_0.60c", 0.60),
              _tie_family("tie_J_0.75c", 0.75))),
            ("K_pair_0.60c_latticeDepth0.75", "live", "combo",
             [("add_families", (_tie_family("tie_K_0.60c", 0.60),)),
              ("lattice_depth", 0.75)]),
        ]
        if args.only:
            wanted = args.only.split(",")
            PAIR_GRID = [g for g in PAIR_GRID if any(w in g[0] for w in wanted)]
        base_paths = {"live": m34.LIVE_CONFIG_PATH, "pre_d": PRE_CONFIG_D_PATH}
        run_grid_with_bases(PAIR_GRID, args.out, base_paths)
        raise SystemExit(0)

    if args.web_depth_rows:
        def _tie_family_wd(name, web_position):
            return WebFamily(name=name, role=ROLE_PRIMARY_STRUCTURE,
                              web_position=web_position, opposing_pair=True)

        # Same tie shape as the pair rows (add_families + skin_margin_mm
        # 1.0). The family name is not read by any geometry: placement and
        # pairing in generator.py's _generate_opposing_pair_family key off
        # role/web_position/opposing_pair, not name.
        WEB_DEPTH_GRID = [
            ("L1_A2_latticeDepth1.0", "live", "combo",
             [("skin_margin_mm", 1.0), ("lattice_depth", 1.0)]),
            ("L2_A2_latticeDepth1.5", "live", "combo",
             [("skin_margin_mm", 1.0), ("lattice_depth", 1.5)]),
            ("M1_H2_latticeDepth1.0", "live", "combo",
             [("add_families", (_tie_family_wd("tie_M_0.60c", 0.60),)),
              ("skin_margin_mm", 1.0), ("lattice_depth", 1.0)]),
            ("M2_H2_latticeDepth1.5", "live", "combo",
             [("add_families", (_tie_family_wd("tie_M_0.60c", 0.60),)),
              ("skin_margin_mm", 1.0), ("lattice_depth", 1.5)]),
        ]
        if args.only:
            wanted = args.only.split(",")
            WEB_DEPTH_GRID = [g for g in WEB_DEPTH_GRID if any(w in g[0] for w in wanted)]
        base_paths = {"live": m34.LIVE_CONFIG_PATH, "pre_d": PRE_CONFIG_D_PATH}
        run_grid_with_bases(WEB_DEPTH_GRID, args.out, base_paths)
        raise SystemExit(0)

    if args.bg_rows:
        BG_GRID = [
            ("B_deep_pitch0.287", "pre_d", "lattice_depth", 0.50),
            ("C_deep_pitch0.40", "pre_d", "combo",
             [("lattice_depth", 0.50), ("deep_pitch", 0.40)]),
            ("D_deep_pitch0.55", "pre_d", "combo",
             [("lattice_depth", 0.50), ("deep_pitch", 0.55)]),
            ("E_deep_plusU_minusL_pitch0.287", "pre_d", "combo",
             [("lattice_depth", 0.50),
              ("drop_families", ("deep_minus_upper", "deep_plus_lower"))]),
            ("F_deep_plusU_minusL_pitch0.40", "pre_d", "combo",
             [("lattice_depth", 0.50), ("deep_pitch", 0.40),
              ("drop_families", ("deep_minus_upper", "deep_plus_lower"))]),
            ("G_deep_pitch0.287_latticeDepth0.75", "pre_d", "lattice_depth", 0.75),
        ]
        if args.only:
            wanted = args.only.split(",")
            BG_GRID = [g for g in BG_GRID if any(w in g[0] for w in wanted)]
        base_paths = {"live": m34.LIVE_CONFIG_PATH, "pre_d": PRE_CONFIG_D_PATH}
        run_grid_with_bases(BG_GRID, args.out, base_paths)
        raise SystemExit(0)

    # "shipped" rows (lattice_pitch 0.12, lattice_depth 1.5, deep_pitch
    # 0.287) are NOT re-run: they are bit-for-bit the same config as
    # "baseline" (axis=None uses the live config unchanged), so the table
    # reuses the single "baseline" run for all three shipped cells rather
    # than spending ~3 redundant sweeps re-deriving numbers already computed.
    GRID = [
        ("baseline_D", None, None),
        ("pitch_0.16", "lattice_pitch", 0.16),
        ("pitch_0.20", "lattice_pitch", 0.20),
        ("pitch_0.24", "lattice_pitch", 0.24),
        ("depth_0.60", "lattice_depth", 0.60),
        ("depth_0.50", "lattice_depth", 0.50),
        ("no_flank_upper_fwd", "drop_families", ("flank_upper_fwd",)),
        ("no_flank_lower_aft", "drop_families", ("flank_lower_aft",)),
        ("no_flanks", "drop_families", ("flank_upper_fwd", "flank_lower_aft")),
        ("pitch_0.16_no_flanks", "combo",
         [("lattice_pitch", 0.16),
          ("drop_families", ("flank_upper_fwd", "flank_lower_aft"))]),
        ("pitch_0.20_depth_0.60", "combo",
         [("lattice_pitch", 0.20), ("lattice_depth", 0.60)]),
    ]
    if args.only:
        wanted = args.only.split(",")
        GRID = [g for g in GRID if any(w in g[0] for w in wanted)]

    run_grid(GRID, args.out)
