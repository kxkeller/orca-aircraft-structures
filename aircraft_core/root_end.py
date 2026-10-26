"""Which end of the placed mesh is the wing ROOT.

A wing may print ROOT ON THE BED (layer 0 is the root) or TIP FACE DOWN, with
the root/joint face at the top (as the outboard half of an elevon wing does).
The plugin has to be told -- or has to work out -- which Z extreme is the
root instead of assuming it.

Two independent signals, both derived from the PLACED mesh alone (no host
slice, no config):

  AREA    the convex-hull area of the vertices within
          ``ROOT_END_PROBE_FRACTION`` of each Z extreme. A root is the
          thick/wide end; a tip is the thin/pointed one. This is the same
          heuristic ``tools/preflight_wing.py`` uses (the preflight tool
          re-exports this module's copy).
  CHANNEL the span of Z over which the spar channel is present. The rod
          enters from the root, so the channel abuts the root end and stops
          short of the tip. (Measured on the elevon wing: the channel runs
          15.2-199.6 mm of a 249.5 mm span, i.e. 15 mm from the root face and
          50 mm from the tip.)

Decision rule, deliberately conservative:

  * An explicit override ("bed" / "top") always wins. Both signals are still
    computed and reported so the override is auditable.
  * If both signals are decisive and DISAGREE the answer is refused
    (``RootEndError``) naming both signals -- never guessed. That is the
    fail-closed style: a wrong root end puts the rod socket on the wrong
    side of the wing.
  * A signal is "indeterminate" (None) when its two ends are too close to
    call. If exactly one signal is decisive it decides; if neither is, the
    root is assumed to be at the bed and the decision source says so
    ("default_bed_no_signal"). Constant-section coupons and symmetric test
    prisms land here.

Nothing in this module imports Orca or any other aircraft_core module except
``alignment`` (for the convex hull); it is plain float geometry.
"""

from typing import Dict, List, Optional, Sequence, Tuple

from .alignment import _convex_hull

# Fraction of the Z span, measured in from each extreme, whose vertices feed
# the AREA signal. Same value as tools/preflight_wing.py.
ROOT_END_PROBE_FRACTION = 0.02

# AREA is decisive only when the larger end is at least this many times the
# smaller. A tapered wing is far beyond it (elevon halves: >2x); a constant
# section prism is exactly 1.0.
AREA_DECISIVE_RATIO = 1.10

# CHANNEL is decisive only when the two end gaps differ by at least this
# fraction of the span (elevon wing: (50 - 15) / 249.5 = 14%).
CHANNEL_GAP_DECISIVE_FRACTION = 0.05

# Accepted config values. "bed" = low Z (layer 0), "top" = high Z.
ROOT_END_AUTO = "auto"
ROOT_END_BED = "bed"
ROOT_END_TOP = "top"
VALID_ROOT_END_VALUES = (ROOT_END_AUTO, ROOT_END_BED, ROOT_END_TOP)

END_LOW = "low"
END_HIGH = "high"


class RootEndError(ValueError):
    """The root end could not be resolved without guessing."""


def band_hull_area(verts: Sequence[Tuple[float, float, float]], lo: float, hi: float) -> float:
    """Convex-hull (shoelace) area of the XY projection of the vertices
    whose Z lies in ``[lo, hi]``. Coarse by design: cheap and sufficient to
    tell a thick root from a thin/pointed tip."""
    pts = [(v[0], v[1]) for v in verts if lo <= v[2] <= hi]
    if len(pts) < 3:
        return 0.0
    hull = _convex_hull(pts)
    if len(hull) < 3:
        return 0.0
    area = 0.0
    n = len(hull)
    for i in range(n):
        x1, y1 = hull[i]
        x2, y2 = hull[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return abs(area) / 2.0


def area_extremes(verts: Sequence[Tuple[float, float, float]]) -> Dict:
    """Raw AREA measurements at both Z extremes (no decision)."""
    zs = [v[2] for v in verts]
    z_min, z_max = min(zs), max(zs)
    span = z_max - z_min
    if span <= 0:
        return {"z_min": z_min, "z_max": z_max, "span_mm": span, "band_mm": None,
                "area_low_end_mm2": 0.0, "area_high_end_mm2": 0.0}
    band = max(span * ROOT_END_PROBE_FRACTION, 1e-6)
    return {
        "z_min": z_min, "z_max": z_max, "span_mm": span, "band_mm": band,
        "area_low_end_mm2": band_hull_area(verts, z_min, z_min + band),
        "area_high_end_mm2": band_hull_area(verts, z_max - band, z_max),
    }


def area_signal(verts: Sequence[Tuple[float, float, float]]) -> Dict:
    """AREA signal: which end is the larger one.

    ``end`` is ``"low"``/``"high"`` when the larger end is at least
    ``AREA_DECISIVE_RATIO`` times the smaller, else ``None`` (indeterminate).
    """
    m = area_extremes(verts)
    a_lo, a_hi = m["area_low_end_mm2"], m["area_high_end_mm2"]
    big, small = max(a_lo, a_hi), min(a_lo, a_hi)
    if m["band_mm"] is None or big <= 0.0:
        end = None
    elif small <= 0.0 or big / small >= AREA_DECISIVE_RATIO:
        end = END_LOW if a_lo > a_hi else END_HIGH
    else:
        end = None
    m["end"] = end
    m["decisive_ratio"] = AREA_DECISIVE_RATIO
    return m


# A layer counts as "the spar channel is present" only when its largest
# interior loop is at least this fraction of the largest interior loop seen
# anywhere on the part. Keeps a 3 mm alignment pin (present at the tip and at
# the root) from voting as the 10.5 mm spar channel.
CHANNEL_PRESENT_DIAMETER_FRACTION = 0.5


def channel_presence_from_diameters(
    max_diameters: Sequence[Optional[float]],
) -> List[Optional[bool]]:
    """Per-layer "spar channel present" flags from each layer's largest
    interior-loop diameter (None = no interior loop / layer not sectioned)."""
    known = [d for d in max_diameters if d]
    if not known:
        return [None if d is None else False for d in max_diameters]
    threshold = CHANNEL_PRESENT_DIAMETER_FRACTION * max(known)
    return [False if not d else d >= threshold for d in max_diameters]


def channel_span_signal(
    present: Sequence[Optional[bool]],
    layer_z: Sequence[float],
    z_min: float,
    z_max: float,
) -> Dict:
    """CHANNEL signal: which extreme the channel abuts.

    ``present[i]`` is True/False/None (None = layer not evaluated) for the
    layer whose section plane is ``layer_z[i]``. The gap from the channel
    span to each Z extreme is measured in mm; the root end is the end with
    the SMALLER gap. Indeterminate (``end`` None) when there is no channel
    at all, or the gaps differ by less than
    ``CHANNEL_GAP_DECISIVE_FRACTION`` of the span.
    """
    zs = [z for z, p in zip(layer_z, present) if p]
    span = z_max - z_min
    out = {
        "channel_layers": len(zs), "channel_z_first_mm": None,
        "channel_z_last_mm": None, "gap_to_low_end_mm": None,
        "gap_to_high_end_mm": None, "end": None,
        "decisive_fraction": CHANNEL_GAP_DECISIVE_FRACTION,
    }
    if not zs or span <= 0:
        return out
    first, last = min(zs), max(zs)
    gap_lo = max(0.0, first - z_min)
    gap_hi = max(0.0, z_max - last)
    out.update({
        "channel_z_first_mm": first, "channel_z_last_mm": last,
        "gap_to_low_end_mm": gap_lo, "gap_to_high_end_mm": gap_hi,
    })
    if abs(gap_lo - gap_hi) >= CHANNEL_GAP_DECISIVE_FRACTION * span:
        out["end"] = END_LOW if gap_lo < gap_hi else END_HIGH
    return out


def _to_end(mode: str) -> str:
    return END_LOW if mode == ROOT_END_BED else END_HIGH


def resolve_root_end(
    mode: str,
    area: Dict,
    channel: Dict,
) -> Dict:
    """Combine the two signals into a decision.

    Returns a dict with ``root_end`` ("low"/"high"), ``root_at`` ("bed"/
    "top"), ``decision_source`` and both signals verbatim. Raises
    ``RootEndError`` in auto mode when both signals are decisive and
    disagree.
    """
    if mode not in VALID_ROOT_END_VALUES:
        raise RootEndError(
            "root_end={!r} is not one of {!r}.".format(mode, VALID_ROOT_END_VALUES)
        )
    a_end, c_end = area.get("end"), channel.get("end")
    base = {
        "mode": mode, "area_signal": area, "channel_signal": channel,
        "area_end": a_end, "channel_end": c_end,
        "override_contradicts_signal": [],
    }
    if mode != ROOT_END_AUTO:
        end = _to_end(mode)
        base.update({"root_end": end, "decision_source": "config_override",
                     "signals_agree": (None if None in (a_end, c_end)
                                       else a_end == c_end == end)})
        # An override that contradicts a decisive signal is honoured but
        # flagged: the report must show it, the user asked for it.
        base["override_contradicts_signal"] = [
            n for n, e in (("area", a_end), ("channel", c_end))
            if e is not None and e != end
        ]
    elif a_end is not None and c_end is not None:
        if a_end != c_end:
            raise RootEndError(
                "the root end cannot be resolved: the two signals disagree. "
                "AREA says the root is the {a} Z end (hull area {alo:.3f} mm^2 "
                "at the low end vs {ahi:.3f} mm^2 at the high end, larger end "
                "is the root); CHANNEL says the root is the {c} Z end (the spar "
                "channel spans Z {f:.3f}-{l:.3f} mm, {glo:.3f} mm from the low "
                "end and {ghi:.3f} mm from the high end; the rod enters at the "
                "root, so the channel abuts it). Refusing to guess -- a wrong "
                "root end puts the spar socket on the wrong side of the wing. "
                "Set root_end to \"bed\" (root at layer 0) or \"top\" (root at "
                "the highest layer) in the plugin config.".format(
                    a=a_end, alo=area["area_low_end_mm2"],
                    ahi=area["area_high_end_mm2"], c=c_end,
                    f=channel["channel_z_first_mm"],
                    l=channel["channel_z_last_mm"],
                    glo=channel["gap_to_low_end_mm"],
                    ghi=channel["gap_to_high_end_mm"],
                )
            )
        base.update({"root_end": a_end, "decision_source": "auto_area_and_channel_agree",
                     "signals_agree": True})
    elif a_end is not None:
        base.update({"root_end": a_end, "decision_source": "auto_area_only",
                     "signals_agree": None})
    elif c_end is not None:
        base.update({"root_end": c_end, "decision_source": "auto_channel_only",
                     "signals_agree": None})
    else:
        base.update({"root_end": END_LOW, "decision_source": "default_bed_no_signal",
                     "signals_agree": None})
    base["root_at"] = ROOT_END_BED if base["root_end"] == END_LOW else ROOT_END_TOP
    return base
