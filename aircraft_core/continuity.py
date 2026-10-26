"""web continuity: per-layer web records and a pure checker.

No Orca imports, no geometry:
this module only reads the per-web records a slice (or
a mesh replay) wrote into its per-layer report, so the same code runs in-host
at the end of every slice, offline on a saved report, and in
``tools/continuity_census.py``.

THE RECORDS. For every layer, ``web_report_rows(intents, omitted)`` returns

* ``webs``: one row per web actually cut (written to the report as one
  string per layer, see ``encode_rows``), ``[family_index, k, side, depth_mm,
  x_mm]`` (and a sixth element, the printed merged X, when the web was folded
  into a merge). ``depth_mm`` is the EXECUTED depth, ``|end_y -
  start_y|`` of the slot actually built, so a merge credits its merged
  (shallowest) depth to every constituent. ``x_mm`` is the web's own station X
  in the frame the generator uses (canonical chord-along-X, not rotated back
  to world). A web absent from the layer has no row: absence is depth 0.
  Rows written by the current code are always eight elements ``[fi, k, side,
  depth, x, merged_x | None, left, right]`` where ``left``/``right`` are the X
  of the printed slot's two SIDE WALLS (``start_x -/+ width/2`` of the slot
  actually cut: the merged slot's edges for a node). Short rows (5 or 6
  elements, no walls) still decode and are checked on the centre X.
* ``web_omit``: one row per web the generator considered and dropped,
  ``[family_index, k, side, x_mm, code]`` where ``code`` is a short slug of the
  omission reason (``omission_code``). A whole omitted opposing pair has side
  ``P``.

Sides: ``U``/``L`` entry skin, ``S`` socket (one identity per socket, however
its entry side flips), ``P`` pair (omissions only).

THE CHECKER. ``check_continuity`` reports five violation types, per web
identity ``(family_index, k, side)`` over Z:

* ``blink``: back after at least one absent layer at depth > ``ramp_mm``.
* ``unramped_start``: the web's FIRST appearance at depth > ``ramp_mm``.
  ``blink`` is "reappearance after absence"; a web that never existed before
  is the degenerate case and is reported under its own name so the two can
  be told apart.
* ``increase``: depth rose by more than ``ramp_mm`` between adjacent layers.
* ``x_step``: EITHER side wall of the web's printed slot moved by more than
  ``dx_max_mm`` between adjacent layers with depth carried over, i.e. the web
  is deeper than a fresh ramped start may be (``depth > ramp_mm``); at depth
  <= ``ramp_mm`` a lateral jump IS the "END of the old web plus a new
  ramped START". The walls are measured, not the centre, so a merged slot
  whose far edge jumps is caught even when its centre barely moves; a row
  without walls falls back to the centre X.
  WALL-MATCHING MODE (``wall_matching=True``): a printed
  slot wall is supported by ANY printed wall of the same orientation (left
  edge / right edge of a slot) and the same entry side on the previous layer
  that lies within ``dx_max_mm`` -- not only by the same web's own previous
  wall. A web deeper than a fresh ramped start (``depth > ramp_mm``) is an
  ``x_step`` violation only when one of its two walls has NO such matching
  wall below, or when the best matching wall is shallower than ``depth -
  ramp_mm`` (the overlapped portion cannot carry a deeper slot). This is the
  physical statement; the strict per-web mode is the default.
* ``socket_restart``: the CONFIRMED spar socket ``(-2, 0, "S")`` present again
  after an END, at any depth. Sockets cannot ramp, so every
  socket (``family_index < 0``) is EXEMPT from the three ramp rules; only the
  confirmed spar's restart is policed (a blind pocket the rod cannot
  enter). An unconfirmed pick (side ``N``) and a bore are different holes that
  share a label, and are not restart-policed (see ``SIDE_SOCKET_UNCONFIRMED``).

EXEMPTIONS are passed in (``exempt``: ``{layer_index: name}``) and every
violation that lands on an exempt layer is counted under ``exempted`` instead
of ``violations``. ``default_exemptions`` builds the two exempt bands:
the first structured layer at the bed (``root_solid``) and the trailing
shutdown run at the top (``final_top``).

This module only reports: nothing here raises on a violation.
"""

from typing import Dict, Iterable, List, Optional, Sequence, Tuple

CONTINUITY_VERSION = 3

# The default lateral-continuity threshold, mm. DERIVATION, measured on both
# elevon halves in print orientation with the strict per-web rule, over every
# web that carried depth over (depth > r on the later layer) and was not
# merged on either layer:
#
#   * LITERAL "max observed non-merge side-wall step + 0.05 mm" = 11.525 +
#     0.05 = 11.575 mm. That figure is a layer-1003 inside-half chord
#     discontinuity, not drift, and a threshold of 11.6 mm makes the lateral
#     rule vacuous (it would keep a wall that moves 1.2 mm per layer, ~10 deg
#     from horizontal, as "the same web"). NOT USED.
#   * DRIFT-POPULATION figure (used): the steps form a dense population up to
#     0.284 mm (0.2 mm/layer z-drift plus chord taper; one outlier at 0.297),
#     then a gap of 0.028 mm, then a sparse swept-region population from
#     0.325 mm up (layers 612-659 inside: ~20 webs sliding 0.33-1.2 mm per
#     layer). The largest step below the first gap wider than 0.025 mm above
#     0.2 mm is 0.297 mm; + 0.05 mm margin = 0.347 mm.
#
# The literal figure is only a sensitivity run, not the default.
DEFAULT_DX_MAX_MM = 0.347
DX_MAX_DERIVATION = {
    "rule": "largest non-merge carried side-wall step below the first gap "
            "> 0.025 mm above 0.2 mm, plus 0.05 mm margin",
    "literal_rule_value_mm": 11.575,
    "drift_population_max_mm": 0.297,
    "margin_mm": 0.05,
    "value_mm": 0.347,
}

# Depth/X are rounded to 0.001 mm in the rows; two roundings can differ from
# the true step by up to 0.001 mm, so comparisons carry a small tolerance.
DEPTH_TOLERANCE_MM = 0.002
X_TOLERANCE_MM = 0.002
DEFAULT_VIOLATION_CAP = 200

VIOLATION_BLINK = "blink"
VIOLATION_UNRAMPED_START = "unramped_start"
VIOLATION_INCREASE = "increase"
VIOLATION_X_STEP = "x_step"
VIOLATION_SOCKET_RESTART = "socket_restart"
VIOLATION_TYPES = (
    VIOLATION_BLINK,
    VIOLATION_UNRAMPED_START,
    VIOLATION_INCREASE,
    VIOLATION_X_STEP,
    VIOLATION_SOCKET_RESTART,
)

EXEMPT_ROOT_SOLID = "root_solid"
EXEMPT_FINAL_TOP = "final_top"

# ``extra["reason"]`` values of a layer left unmutated at the top of the part.
_TOP_SHUTDOWN_REASONS = frozenset({
    "host_tip_region_empty",
    "host_tip_below_printable_floor",
    "tip_alignment_skip",
    "tip_shutdown",
    "all_webs_omitted",
})

# Ordered (prefix, code): the first match wins.
_OMISSION_PREFIXES = (
    ("re_entrant_station", "re_entrant_station"),
    ("skin margin: mitre clearance", "mitre_clearance"),
    ("skin margin: degenerate", "degenerate_depth"),
    ("skin margin", "skin_margin"),
    ("thin section", "thin_section"),
    ("chord extent", "chord_extent"),
    ("opposing entry", "opposing_entry"),
    ("opposing pair", "opposing_pair"),
    ("role yield", "role_yield"),
    ("hole footprint", "hole_footprint"),
    ("depth_to_socket", "depth_to_socket"),
    ("socket_flank", "socket_flank"),
    ("spar socket footprint", "socket_footprint"),
    ("bore socket footprint", "bore_footprint"),
    ("pair ablation", "pair_ablation"),
    ("D051", "pair_ablation"),
    ("z_ablation", "z_ablation"),
    ("z_x_jump", "z_x_jump"),
    ("z_socket_latch", "z_socket_latch"),
)


def omission_code(reason: str) -> str:
    """A short, stable slug for an omission reason string (for the compact
    ``web_omit`` rows). Unknown reasons keep a slug of their own first words."""
    text = str(reason or "")
    for prefix, code in _OMISSION_PREFIXES:
        if text.startswith(prefix):
            return code
    head = text.split(":", 1)[0].strip().lower()
    slug = "".join(c if c.isalnum() else "_" for c in head).strip("_")
    return slug[:24] or "unspecified"


def _round_or_none(value: float, places: int = 3):
    try:
        if value != value:  # NaN
            return None
        return round(float(value), places)
    except (TypeError, ValueError):
        return None


def _side_of(intent) -> str:
    return "L" if getattr(intent, "entry_lower", False) else "U"


# Socket identity. The generator labels WHICHEVER channel the
# nearest-spar-station policy picks as ``(-2, 0, "S")``: the confirmed rod
# channel, but also an UNCONFIRMED pick (two channels at the tip end of the
# outside half: the spar and an alignment pin) and, where the spar has ended, a
# lone pin that is itself read as the spar. Treating those as one web made a
# different hole read as a "restart" of the rod channel (outside half: 993
# layers refused) or as a 51 mm "jump" (inside half layer 1000). So the report
# and the rule give an unconfirmed pick its own side letter, ``N``. Only the
# CONFIRMED spar socket ``(-2, 0, "S")`` is subject to "ends stay ended" (is
# about the rod); bores are independent holes whose ordinal is not a stable
# identity (the generator's own comment) and are not restart-policed.
SIDE_SOCKET_CONFIRMED = "S"
SIDE_SOCKET_UNCONFIRMED = "N"


def is_confirmed_spar_id(wid) -> bool:
    return wid[0] == -2 and wid[2] == SIDE_SOCKET_CONFIRMED


def intent_web_ids(intent) -> Tuple[tuple, ...]:
    """Every web identity an intent carries, as the report and the Z rule both key
    them (a merge carries all of its constituents; an unconfirmed spar-socket
    pick has side ``N``)."""
    merged = tuple(getattr(intent, "merged_web_ids", ()) or ())
    if merged:
        return merged
    wid = getattr(intent, "web_id", None)
    if wid is None:
        return ()
    if (wid[0] == -2 and wid[2] == SIDE_SOCKET_CONFIRMED
            and not getattr(intent, "spar_confirmed", True)):
        wid = (wid[0], wid[1], SIDE_SOCKET_UNCONFIRMED)
    return (wid,)


def web_report_rows(intents: Sequence, omitted: Sequence) -> Tuple[List[list], List[list]]:
    """The compact per-layer web records (see the module docstring).

    ``intents`` / ``omitted`` are the FINAL lists for the layer (after the
    merge, opposing pass, socket latch, slot-infeasible drop and ablation),
    i.e. exactly what was cut. An intent with no ``web_id`` (feature lattice,
    direct construction) is reported under ``(family_index, -1, side)``."""
    present: List[list] = []
    for intent in intents:
        depth = _round_or_none(abs(intent.end[1] - intent.start[1]))
        printed_x = _round_or_none(intent.start[0])
        half_w = 0.5 * float(getattr(intent, "width_mm", 0.0) or 0.0)
        left = _round_or_none(intent.start[0] - half_w)
        right = _round_or_none(intent.start[0] + half_w)
        ids = tuple(getattr(intent, "merged_web_ids", ()) or ())
        xs = tuple(getattr(intent, "merged_web_xs", ()) or ())
        if ids and len(ids) == len(xs):
            for wid, own_x in zip(ids, xs):
                present.append([wid[0], wid[1], wid[2], depth,
                                _round_or_none(own_x), printed_x, left, right])
            continue
        own = intent_web_ids(intent)
        wid = own[0] if own else None
        if wid is None:
            wid = (intent.family_index, -1, _side_of(intent))
        present.append([wid[0], wid[1], wid[2], depth, printed_x, None,
                        left, right])
    dropped: List[list] = []
    for rec in omitted:
        ids = tuple(getattr(rec, "merged_web_ids", ()) or ())
        if not ids:
            wid = getattr(rec, "web_id", None)
            if wid is None:
                wid = (rec.family_index, -1, "?")
            ids = (wid,)
        code = omission_code(rec.reason)
        for wid in ids:
            dropped.append([wid[0], wid[1], wid[2], _round_or_none(rec.x_mm), code])
    return present, dropped


def _fmt(value) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return repr(value)
    return str(value)


def encode_rows(rows: Sequence[Sequence]) -> str:
    """One compact string for a layer's rows: ``fi,k,side,depth,x[,mx[,left,
    right]]`` per web (an absent merged X is an empty field), ``;``-separated. The run report is written with ``indent=2``, which
    puts every element of a nested list on its own line; a row list would
    multiply the report several times over, a string is one line per layer."""
    return ";".join(",".join(_fmt(v) for v in row) for row in rows)


def decode_rows(text, kind: str = "webs") -> List[list]:
    """Inverse of ``encode_rows``. ``kind`` is ``"webs"`` (``[fi, k, side,
    depth, x]`` plus optional printed X) or ``"omit"`` (``[fi, k, side, x,
    code]``). A list passes through unchanged, so callers can hand it either
    form."""
    if isinstance(text, (list, tuple)):
        return [list(r) for r in text]
    if not text:
        return []

    def _num(s):
        return None if s == "" else float(s)

    rows: List[list] = []
    for chunk in str(text).split(";"):
        parts = chunk.split(",")
        if kind == "omit":
            rows.append([int(parts[0]), int(parts[1]), parts[2],
                         _num(parts[3]), parts[4]])
        else:
            row = [int(parts[0]), int(parts[1]), parts[2],
                   _num(parts[3]), _num(parts[4])]
            if len(parts) > 5:
                row.append(_num(parts[5]))
            if len(parts) > 7:
                row.append(_num(parts[6]))
                row.append(_num(parts[7]))
            rows.append(row)
    return rows


# --------------------------------------------------------------------------
# Checker
# --------------------------------------------------------------------------

def default_exemptions(
    layers: Sequence[dict], root_solid_layers: int
) -> Dict[int, str]:
    """The two exempt bands.

    ``root_solid``: layer indices ``<= root_solid_layers`` -- the solid bed
    layers and the FIRST structured layer, which cannot ramp over a solid
    layer below it. ``final_top``: the trailing run of layers a top-of-part
    shutdown left unmutated (``extra.reason`` in ``host_tip_*``,
    ``tip_alignment_skip``, ``tip_shutdown``, ``all_webs_omitted``)."""
    exempt: Dict[int, str] = {}
    for index in range(0, max(0, int(root_solid_layers)) + 1):
        exempt[index] = EXEMPT_ROOT_SOLID
    ordered = sorted(layers, key=lambda rec: rec["layer_index"])
    for rec in reversed(ordered):
        reason = (rec.get("extra") or {}).get("reason") if "extra" in rec else rec.get("reason")
        if reason in _TOP_SHUTDOWN_REASONS or (
            not rec.get("webs") and rec.get("shutdown")
        ):
            exempt.setdefault(rec["layer_index"], EXEMPT_FINAL_TOP)
        else:
            break
    return exempt


def layers_from_report(report: dict) -> List[dict]:
    """Per-layer dicts for ``check_continuity`` from a run report (the shape
    ``RunDiagnostics.to_dict()['layers']`` / the preflight ``t3_layers`` use)."""
    out = []
    for rec in report.get("layers", []) or []:
        extra = rec.get("extra") or {}
        out.append({
            "layer_index": rec["layer_index"],
            "print_z": rec.get("print_z"),
            "webs": decode_rows(extra.get("webs") or "", "webs"),
            "web_omit": decode_rows(extra.get("web_omit") or "", "omit"),
            "shutdown": bool(rec.get("shutdown")),
            "reason": extra.get("reason"),
        })
    return out


def _is_socket(family_index: int) -> bool:
    return family_index < 0


WALL_LEFT = "L"
WALL_RIGHT = "R"


def wall_support(
    walls: Sequence[Tuple[float, float]], x: float, dx_max_mm: float,
) -> Tuple[Optional[float], Optional[float]]:
    """Support for one printed wall at ``x`` from a previous-layer wall list
    ``[(x_i, depth_i), ...]`` of the SAME orientation and entry side. Returns
    ``(best_depth, nearest_dx)``: the deepest wall within ``dx_max_mm``
    (``None`` when no wall is that close), and the signed offset of the
    nearest wall of any distance (``None`` for an empty list)."""
    best: Optional[float] = None
    nearest: Optional[float] = None
    for wx, wd in walls:
        off = x - wx
        if nearest is None or abs(off) < abs(nearest):
            nearest = off
        if abs(off) <= dx_max_mm and (best is None or wd > best):
            best = wd
    return best, nearest


def check_continuity(
    layers: Iterable[dict],
    *,
    ramp_mm: float,
    dx_max_mm: float,
    exempt: Optional[Dict[int, str]] = None,
    family_names: Optional[Dict[int, str]] = None,
    violation_cap: int = DEFAULT_VIOLATION_CAP,
    depth_tolerance_mm: float = DEPTH_TOLERANCE_MM,
    x_tolerance_mm: float = X_TOLERANCE_MM,
    police_all_sockets: bool = False,
    wall_matching: bool = False,
) -> dict:
    """Check web continuity over Z. Pure; never raises on a violation.

    ``wall_matching`` selects the wall-matching x_step rule (see the module
    docstring); False is the strict per-web rule.

    ``police_all_sockets`` restart-polices every ``family_index < 0``
    identity; the default polices only the confirmed spar socket.

    ``layers``: dicts with ``layer_index``, ``print_z``, ``webs``,
    ``web_omit`` (see ``layers_from_report``). Layers absent from the input
    count as absent for every web. Returns a JSON-friendly dict (see the
    module docstring for the rules)."""
    exempt = exempt or {}
    family_names = family_names or {}
    layer_list = sorted(layers, key=lambda rec: rec["layer_index"])
    z_by_layer: Dict[int, Optional[float]] = {
        rec["layer_index"]: rec.get("print_z") for rec in layer_list
    }

    # series[web] = {layer_index: (depth, own_x, printed_x, merged, left,
    # right)}; ``left``/``right`` are the printed slot's side-wall X (None for
    # a row without walls, which is then checked on the centre X).
    series: Dict[tuple, Dict[int, tuple]] = {}
    omit_codes: Dict[tuple, Dict[int, str]] = {}
    for rec in layer_list:
        index = rec["layer_index"]
        for row in rec.get("webs") or []:
            fi, k, side, depth, own_x = row[0], row[1], row[2], row[3], row[4]
            merged = len(row) > 5 and row[5] is not None
            printed_x = row[5] if merged else own_x
            left = row[6] if len(row) > 7 else None
            right = row[7] if len(row) > 7 else None
            slot = series.setdefault((fi, k, side), {})
            previous = slot.get(index)
            if previous is None or (depth is not None and depth < previous[0]):
                slot[index] = (depth, own_x, printed_x, merged, left, right)
        for row in rec.get("web_omit") or []:
            fi, k, side, _x, code = row[0], row[1], row[2], row[3], row[4]
            omit_codes.setdefault((fi, k, side), {})[index] = code
            if side == "P":
                # A whole pair: attribute to both members.
                omit_codes.setdefault((fi, k, "U"), {})[index] = code
                omit_codes.setdefault((fi, k, "L"), {})[index] = code

    # Wall index for the wall-matching mode: per layer, per (entry side,
    # orientation), every printed non-socket wall as (x, depth). Built from the
    # same rows (a merged slot's constituents repeat its walls, harmlessly).
    walls_at: Dict[int, Dict[tuple, List[Tuple[float, float]]]] = {}
    if wall_matching:
        for web, pts in series.items():
            if _is_socket(web[0]):
                continue
            for idx, (dpt, _ox, _px, _m, lft, rgt) in pts.items():
                if lft is None or rgt is None or dpt is None:
                    continue
                layer_walls = walls_at.setdefault(idx, {})
                layer_walls.setdefault((web[2], WALL_LEFT), []).append((lft, dpt))
                layer_walls.setdefault((web[2], WALL_RIGHT), []).append((rgt, dpt))

    violations: List[dict] = []
    counts = {name: 0 for name in VIOLATION_TYPES}
    exempted = {name: 0 for name in VIOLATION_TYPES}
    exempted_by_band: Dict[str, int] = {}
    per_web: Dict[tuple, dict] = {}
    per_family: Dict[int, dict] = {}
    truncated = False
    # Every adjacent-layer side-wall step of a web that carried depth over
    # (both layers deeper than zero), split by whether a merge was involved on
    # either layer. This is the distribution ``dx_max`` is derived from (see
    # ``DEFAULT_DX_MAX_MM``).
    step_bins = (("<=0.10", 0.10), ("0.10-0.21", 0.21), ("0.21-0.30", 0.30),
                 ("0.30-0.50", 0.50), ("0.50-1.0", 1.0), (">1.0", float("inf")))
    step_stats = {
        key: {"count": 0, "max": 0.0, "bins": {name: 0 for name, _ in step_bins}}
        for key in ("non_merge", "merge")
    }

    def _family_entry(fi: int) -> dict:
        entry = per_family.get(fi)
        if entry is None:
            entry = per_family[fi] = {
                "name": family_names.get(fi),
                "webs": 0, "web_layers": 0, "runs": 0, "ends": 0,
                "violations": {name: 0 for name in VIOLATION_TYPES},
            }
        return entry

    def _flag(web, vtype, layer, **detail):
        nonlocal truncated
        band = exempt.get(layer)
        info = per_web[web]
        if band is not None:
            exempted[vtype] += 1
            exempted_by_band[band] = exempted_by_band.get(band, 0) + 1
            return
        counts[vtype] += 1
        info["violations"][vtype] = info["violations"].get(vtype, 0) + 1
        z = z_by_layer.get(layer)
        if z is not None:
            info["z_first"] = z if info["z_first"] is None else min(info["z_first"], z)
            info["z_last"] = z if info["z_last"] is None else max(info["z_last"], z)
        _family_entry(web[0])["violations"][vtype] += 1
        if len(violations) < violation_cap:
            record = {
                "type": vtype, "web": list(web), "layer": layer,
                "z": None if z is None else round(z, 4),
            }
            record.update(detail)
            violations.append(record)
        else:
            truncated = True

    for web in sorted(series, key=lambda w: (w[0], w[1], str(w[2]))):
        fi = web[0]
        socket = _is_socket(fi)
        points = series[web]
        indices = sorted(points)
        per_web[web] = {
            "layers": len(indices), "runs": 0, "ends": 0,
            "violations": {}, "z_first": None, "z_last": None,
        }
        fam = _family_entry(fi)
        fam["webs"] += 1
        fam["web_layers"] += len(indices)
        info = per_web[web]
        prev_index = None
        for index in indices:
            depth, own_x, printed_x, merged, left, right = points[index]
            if prev_index is None:
                info["runs"] += 1
                fam["runs"] += 1
                if not socket and depth is not None and depth > ramp_mm + depth_tolerance_mm:
                    _flag(web, VIOLATION_UNRAMPED_START, index,
                          depth=depth, x=own_x, ramp_mm=ramp_mm)
            else:
                (prev_depth, _prev_own, prev_printed, _prev_merged,
                 prev_left, prev_right) = points[prev_index]
                gap = index - prev_index - 1
                if gap > 0:
                    info["runs"] += 1
                    fam["runs"] += 1
                    info["ends"] += 1
                    fam["ends"] += 1
                    reasons = sorted({
                        omit_codes.get(web, {}).get(j)
                        for j in range(prev_index + 1, index)
                        if omit_codes.get(web, {}).get(j)
                    })
                    if socket:
                        if police_all_sockets or is_confirmed_spar_id(web):
                            _flag(web, VIOLATION_SOCKET_RESTART, index,
                                  depth=depth, x=own_x, gap_layers=gap,
                                  gap_reasons=reasons)
                    elif depth is not None and depth > ramp_mm + depth_tolerance_mm:
                        _flag(web, VIOLATION_BLINK, index, depth=depth, x=own_x,
                              gap_layers=gap, gap_reasons=reasons,
                              ramp_mm=ramp_mm)
                else:
                    if (not socket and depth is not None and prev_depth is not None
                            and depth - prev_depth > ramp_mm + depth_tolerance_mm):
                        _flag(web, VIOLATION_INCREASE, index, depth=depth,
                              prev_depth=prev_depth, x=own_x, ramp_mm=ramp_mm)
                    step = None
                    signed = None
                    # "Depth carried over" = the web is deeper than a fresh
                    # ramped start may be (> r): a lateral jump at depth <= r
                    # is an "END of the old web plus a new ramped START", not
                    # a violation.
                    if (depth and prev_depth
                            and depth > ramp_mm + depth_tolerance_mm):
                        if (left is not None and right is not None
                                and prev_left is not None
                                and prev_right is not None):
                            dl, dr = left - prev_left, right - prev_right
                            signed = dl if abs(dl) >= abs(dr) else dr
                            step = abs(signed)
                        elif printed_x is not None and prev_printed is not None:
                            signed = printed_x - prev_printed
                            step = abs(signed)
                    if step is not None:
                        bucket = step_stats[
                            "merge" if (merged or _prev_merged) else "non_merge"]
                        bucket["count"] += 1
                        bucket["max"] = max(bucket["max"], step)
                        for name, hi in step_bins:
                            if step <= hi:
                                bucket["bins"][name] += 1
                                break
                    if (wall_matching and not socket and depth is not None
                            and prev_depth is not None and left is not None
                            and right is not None
                            and depth > ramp_mm + depth_tolerance_mm
                            and prev_depth > 0):
                        below = walls_at.get(prev_index, {})
                        for wx, orient in ((left, WALL_LEFT), (right, WALL_RIGHT)):
                            best, nearest = wall_support(
                                below.get((web[2], orient), ()), wx,
                                dx_max_mm + x_tolerance_mm)
                            if best is None or (
                                    best + ramp_mm + depth_tolerance_mm < depth):
                                _flag(web, VIOLATION_X_STEP, index, depth=depth,
                                      prev_depth=prev_depth, x=printed_x,
                                      prev_x=prev_printed,
                                      dx=None if nearest is None else round(nearest, 4),
                                      dx_max_mm=dx_max_mm,
                                      merged=bool(merged or _prev_merged),
                                      wall="left" if orient == WALL_LEFT else "right",
                                      support_depth=best,
                                      reason=("no_matching_wall" if best is None
                                              else "matched_wall_too_shallow"))
                                break
                    elif step is not None and step > dx_max_mm + x_tolerance_mm:
                        _flag(web, VIOLATION_X_STEP, index, depth=depth,
                              prev_depth=prev_depth, x=printed_x,
                              prev_x=prev_printed, dx=round(signed, 4),
                              dx_max_mm=dx_max_mm,
                              merged=bool(merged or _prev_merged))
            prev_index = index
        # A web present on the last layer of the run did not END; an END is
        # only counted when a later layer exists without it.
        if indices and any(
            rec["layer_index"] > indices[-1] for rec in layer_list
        ):
            info["ends"] += 1
            fam["ends"] += 1

    total = sum(counts.values())
    worst = sorted(
        (
            {
                "web": list(web), "violations": sum(info["violations"].values()),
                "by_type": dict(info["violations"]),
                "z_first": None if info["z_first"] is None else round(info["z_first"], 4),
                "z_last": None if info["z_last"] is None else round(info["z_last"], 4),
                "runs": info["runs"], "layers": info["layers"],
            }
            for web, info in per_web.items()
            if info["violations"]
        ),
        key=lambda item: (-item["violations"], item["web"]),
    )[:10]
    return {
        "version": CONTINUITY_VERSION,
        "mode": "report_only",
        "wall_matching": bool(wall_matching),
        "ramp_mm": ramp_mm,
        "dx_max_mm": dx_max_mm,
        "depth_tolerance_mm": depth_tolerance_mm,
        "x_tolerance_mm": x_tolerance_mm,
        "layers_checked": len(layer_list),
        "webs_tracked": len(series),
        "violation_total": total,
        "violation_counts": counts,
        "violations": violations,
        "violations_truncated": truncated,
        "violation_cap": violation_cap,
        "side_wall_steps": {
            key: {"count": v["count"], "max": round(v["max"], 4),
                  "bins": v["bins"]}
            for key, v in step_stats.items()
        },
        "exempted_counts": exempted,
        "exempted_by_band": exempted_by_band,
        "exempt_layer_count": len(exempt),
        "by_family": {str(fi): per_family[fi] for fi in sorted(per_family)},
        "worst_webs": worst,
    }
