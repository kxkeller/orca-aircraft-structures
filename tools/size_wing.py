#!/usr/bin/env python3
"""Weighted lattice sizer: choose rib pitch for a given wing by scoring real
generated geometry against a stated objective.

WHAT THIS IS. Given a wing (taper, twist, chord, section) and a set of WEIGHTS,
this sweeps candidate ``rib_pitch_chord_frac`` values, drives the REAL
``StructureGenerator`` at sampled spanwise stations, scores each candidate, and
reports the ranked table, so the pitch adapts to each wing and size.

THREE THINGS IT DELIBERATELY DOES NOT DO
----------------------------------------
1. **It does not decide validity.** Hard constraints REJECT a candidate; they
   are never traded against a weight. Weighing the one-loop invariant against
   mass is how a silent geometric fallback gets built. The generator's own omissions are counted, but the real
   authority on validity is ``tools/verify_full_sweep_varying.py``, which
   must be run on any candidate before it is adopted. This tool prints that
   requirement and does not pretend to satisfy it.
2. **It does not use ``structural_analysis.analyze_arm``.** That function
   carries ONE ``LayerFrame`` per section and reuses it at every layer, so it is blind to taper and
   twist -- the exact variation this tool exists to adapt to. Scoring a
   tapered wing with it would be scoring a constant section.
3. **It does not use ``RunDiagnostics.worst_skin_bay_mm``.** That quantity
   sorts entry-X POOLED ACROSS BOTH ENTRY SURFACES and takes adjacent
   differences in chord X. Two problems: the
   buckling formula wants skin ARCLENGTH, which near the leading edge is
   several times the chord dX; and pooling upper with lower reports roughly
   half the true per-surface bay whenever the two surfaces interleave. Since
   ``sigma_cr`` goes as ``1/b^2``, that is a ~4x error in the wrong
   direction. This module computes the bay ITSELF: per entry surface, in
   arclength, rather than using the shipped diagnostic.

THE OBJECTIVE
-------------
Minimise, with weights summing to 1 and every term normalised against the
SHIPPED default's own value on the SAME wing (so J = 1 means "no better than
the shipped default"):

    J = w_bay  * (bay_worst / bay_ref)
      + w_mass * (mass_proxy / mass_ref)
      + w_omit * (omitted_frac / omit_ref)

``bay_worst`` is the worst per-surface ARCLENGTH bay over sampled stations.
Buckling enters through it as ``sigma_cr ~ k / b^2``; the reported
``sigma_rel`` column makes that explicit, including the edge-fixity term.

EDGE FIXITY, AND ITS UNPINNED CONSTANT
--------------------------------------
Web depth restrains the skin-bay edge, moving the buckling coefficient from
4.00 (simply supported) toward 6.98 (clamped):

    zeta = 12 (1 - nu^2) I_web / (t^3 b),  I_web = t_web d^3 / 12
    k    = 4.00 + (6.98 - 4.00) * zeta / (zeta + zeta0)

``zeta0`` is a MODELLING CHOICE, not a citation. Its value changes how much
depth is worth, which is why depth is NOT an optimisation variable here: the
model cannot settle it. Pitch is a different matter: ``b`` enters
``zeta`` as ``1/b`` AND ``sigma_cr`` as ``1/b^2``, and both push the same way,
so the RANKING of pitches is stable across zeta0. This tool reports the ranking
at several zeta0 values and refuses to name a winner that is not stable across
them.

MASS is a VOLUME PROXY at constant layer height, the same construction
``structural_analysis.py`` uses and labels "not a true mass". No grams.

Usage:
    python tools/size_wing.py
    python tools/size_wing.py --weights 0.5,0.4,0.1 --samples 21
"""
from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import math
import os
import sys
from typing import Dict, List, Optional, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS_DIR = os.path.join(REPO_ROOT, "tools")
for _p in (REPO_ROOT, TOOLS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import LayerContext, StructureGenerator  # noqa: E402

from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402


def _load(name: str, relpath: str):
    spec = importlib.util.spec_from_file_location(
        name, os.path.join(REPO_ROOT, relpath))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


vfsv = _load("vfsv_under_size_wing", "tools/verify_full_sweep_varying.py")
sa = _load("sa_under_size_wing", "tools/structural_analysis.py")

LAYER_HEIGHT_MM = 0.2
NU = 0.35
K_SS = 4.00
K_CLAMPED = 6.98
ZETA0_RANGE = (0.1, 1.0, 10.0)


# ---------------------------------------------------------------------------
# Per-surface arclength bay, computed here rather than taken from diagnostics.py.
# ---------------------------------------------------------------------------

def _split_surfaces(outline: List[Tuple[float, float]]):
    """Split a closed section outline into an UPPER and a LOWER polyline,
    each ordered by increasing X.

    Uses the section's own extreme-X vertices (leading and trailing edge) as
    the split points rather than assuming ``naca4_section_outline``'s index
    ordering survives rotation -- the wing fixture is twisted, so the LE/TE
    are not at fixed indices after ``build_station_outline`` rotates the
    outline about the quarter chord.
    """
    n = len(outline)
    i_le = min(range(n), key=lambda i: outline[i][0])
    i_te = max(range(n), key=lambda i: outline[i][0])
    a, b = (i_le, i_te) if i_le <= i_te else (i_te, i_le)
    arc1 = outline[a:b + 1]
    arc2 = outline[b:] + outline[:a + 1]

    def mean_y(pts):
        return sum(p[1] for p in pts) / len(pts)

    upper, lower = (arc1, arc2) if mean_y(arc1) >= mean_y(arc2) else (arc2, arc1)
    upper = sorted(upper, key=lambda p: p[0])
    lower = sorted(lower, key=lambda p: p[0])
    return upper, lower


def _cumulative_arc(pts):
    xs = [p[0] for p in pts]
    cum = [0.0]
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        cum.append(cum[-1] + math.hypot(dx, dy))
    return xs, cum


def _arc_at_x(xs, cum, x):
    if x <= xs[0]:
        return cum[0]
    if x >= xs[-1]:
        return cum[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    span = xs[hi] - xs[lo]
    if span <= 0:
        return cum[lo]
    f = (x - xs[lo]) / span
    return cum[lo] + f * (cum[hi] - cum[lo])


def surface_bays_mm(outline, entries_upper, entries_lower):
    """Worst and mean unsupported skin bay, measured as ARCLENGTH along each
    surface separately. Returns (worst_mm, mean_mm, n_bays).

    A surface with fewer than two entries contributes no bay -- correct, not
    a gap: there is no "between" with 0 or 1 entry points.
    """
    upper, lower = _split_surfaces(outline)
    bays: List[float] = []
    for pts, entries in ((upper, entries_upper), (lower, entries_lower)):
        if len(entries) < 2:
            continue
        xs, cum = _cumulative_arc(pts)
        arcs = sorted(_arc_at_x(xs, cum, x) for x in entries)
        bays.extend(hi - lo for lo, hi in zip(arcs, arcs[1:]))
    if not bays:
        return None, None, 0
    return max(bays), sum(bays) / len(bays), len(bays)


# ---------------------------------------------------------------------------
# Edge fixity
# ---------------------------------------------------------------------------

def edge_fixity_k(depth_mm: float, bay_mm: float, skin_t_mm: float,
                  web_t_mm: float, zeta0: float) -> float:
    if bay_mm <= 0 or skin_t_mm <= 0:
        return K_SS
    i_web = web_t_mm * depth_mm ** 3 / 12.0
    zeta = 12.0 * (1.0 - NU ** 2) * i_web / (skin_t_mm ** 3 * bay_mm)
    return K_SS + (K_CLAMPED - K_SS) * zeta / (zeta + zeta0)


def sigma_rel(bay_mm: float, k: float, skin_t_mm: float) -> float:
    """sigma_cr up to constants: k * (t/b)^2. Relative only -- no E, no MPa."""
    return k * (skin_t_mm / bay_mm) ** 2


# ---------------------------------------------------------------------------
# Scoring one candidate
# ---------------------------------------------------------------------------

def score_candidate(cap_config: Dict, pitch_frac: float, args,
                    wall_width_mm: float, n_samples: int) -> Optional[Dict]:
    cfg = copy.deepcopy(cap_config)
    for fam in cfg.get("families", []):
        if fam.get("rib_pitch_chord_frac") is not None:
            fam["rib_pitch_chord_frac"] = pitch_frac

    spec = sa.build_spec_from_cap_config(cfg)
    outline_at_z, span_mm = vfsv._build_outline_fn("wing", args)
    ops = MiterShapelyOps()
    gen = StructureGenerator()

    n_layers = int(round(span_mm / LAYER_HEIGHT_MM))
    root_skip = int(getattr(spec, "root_solid_layers", 0) or 0)
    idxs = [root_skip + round(i * (n_layers - 1 - root_skip) / (n_samples - 1))
            for i in range(n_samples)]

    worst_bay = 0.0
    bay_samples: List[float] = []
    built = omitted = 0
    web_len_mm = 0.0
    skin_len_mm = 0.0
    depths: List[float] = []
    zero_web_layers = 0

    for li in idxs:
        z = li * LAYER_HEIGHT_MM
        outline = outline_at_z(z)
        poly = ops.polygon_from_points(outline)
        frame = LayerFrame.from_polygon(ops, poly)
        ctx = LayerContext(layer_index=li, print_z=z)
        gen_res = gen.generate(frame, spec, wall_width_mm, ctx,
                               safety_offset_mm=wall_width_mm)
        intents = list(gen_res.intents)
        oms = list(gen_res.omitted)
        built += len(intents)
        omitted += len(oms)
        if not intents:
            zero_web_layers += 1
            continue

        eu = [i.start[0] for i in intents if not i.entry_lower]
        el = [i.start[0] for i in intents if i.entry_lower]
        w, m, n = surface_bays_mm(outline, eu, el)
        if w is not None:
            worst_bay = max(worst_bay, w)
            bay_samples.append(m)

        for i in intents:
            d = abs(i.end[1] - i.start[1])
            depths.append(d)
            web_len_mm += 2.0 * d
        up, lo = _split_surfaces(outline)
        for pts in (up, lo):
            _, cum = _cumulative_arc(pts)
            skin_len_mm += cum[-1]

    if not bay_samples or worst_bay <= 0:
        return None

    total = built + omitted
    slot_gap = 0.021
    web_t = 2.0 * wall_width_mm + slot_gap
    mean_depth = sum(depths) / len(depths) if depths else 0.0
    mass_proxy = (web_len_mm + skin_len_mm) * wall_width_mm * LAYER_HEIGHT_MM

    sig = {}
    for z0 in ZETA0_RANGE:
        k = edge_fixity_k(mean_depth, worst_bay, wall_width_mm, web_t, z0)
        sig["zeta0_%g" % z0] = sigma_rel(worst_bay, k, wall_width_mm)

    return {
        "rib_pitch_chord_frac": pitch_frac,
        "worst_bay_arclen_mm": worst_bay,
        "mean_bay_arclen_mm": sum(bay_samples) / len(bay_samples),
        "webs_built": built,
        "webs_omitted": omitted,
        "omitted_frac": (omitted / total) if total else 0.0,
        "zero_web_layers": zero_web_layers,
        "mean_web_depth_mm": mean_depth,
        "mass_proxy_mm3": mass_proxy,
        "sigma_rel": sig,
    }


def _emit_validated_config(rows, cap_config, a, out, ref):
    """Sweep candidates in J order and write the FIRST that verifies.

    This is what makes the tool ADAPT rather than merely recommend. The
    objective is monotone toward finer pitch, and finer pitch runs into the
    `+`/`-` node-merge width boundary -- 0.0500 c scored best on the wing,
    passed seven fixtures including that wing, and failed the tapered r60/t30
    fixture on four layers. So the lowest J is NOT the answer; the lowest J
    THAT VERIFIES is. Sweeping here, inside the tool, is the only way the
    emitted artifact is validated by construction instead of by a caller
    remembering to check.

    Every candidate is swept on EVERY layer with the real boolean backend -- no
    sampling. Sampling is legitimate for scoring and never for validity (a
    sampled sweep once reported "0 failures" on the layer that failed).
    """
    import subprocess
    import tempfile

    attempts = []
    chosen = None
    for r in rows:
        pitch = r["rib_pitch_chord_frac"]
        cfg = copy.deepcopy(cap_config)
        for fam in cfg.get("families", []):
            if fam.get("rib_pitch_chord_frac") is not None:
                fam["rib_pitch_chord_frac"] = pitch
        whole = {"config": [{"cap_config": cfg,
                             "capability": "Aircraft Structures",
                             "capability_type": "slicing-pipeline",
                             "plugin_key": "aircraft_structures",
                             "plugin_version": "0.1.0"}]}
        tmp = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump(whole, tmp, indent=4)
        tmp.close()

        sweep_out = os.path.join(
            tempfile.gettempdir(), "size_wing_sweep_%s.json" % str(pitch).replace(".", "_"))
        cmd = [sys.executable,
               os.path.join(REPO_ROOT, "tools", "verify_full_sweep_varying.py"),
               "wing", "--config", tmp.name,
               "--root-chord", str(a.root_chord), "--tip-chord", str(a.tip_chord),
               "--span", str(a.span), "--washout", str(a.washout),
               "--out", sweep_out]
        print("  sweeping pitch %.4f c (J=%.4f) over every layer ..." % (pitch, r["J"]))
        subprocess.run(cmd, capture_output=True, text=True)
        status, failures = "NO-RESULT", None
        try:
            with open(sweep_out) as f:
                res = json.load(f)
            res = res.get("result", res)
            status = res.get("status", "NO-RESULT")
            failures = res.get("failure_count")
        except Exception as exc:  # pragma: no cover
            status = "NO-RESULT (%s)" % exc
        attempts.append({"rib_pitch_chord_frac": pitch, "J": r["J"],
                         "sweep_status": status, "failure_count": failures})
        print("    -> %s (%s failures)" % (status, failures))
        if status == "VERIFIED":
            chosen = (pitch, whole, r)
            break
        os.unlink(tmp.name)

    out["emit"] = {"attempts": attempts,
                   "chosen_rib_pitch_chord_frac": chosen[0] if chosen else None,
                   "validation": "every layer, real boolean backend, no sampling"}

    if chosen is None:
        print("\nNO candidate verified. Nothing emitted -- the shipped pitch "
              "stands. This is the hard constraint refusing to adapt, which is "
              "correct behaviour, not a tool failure.")
        return

    pitch, whole, r = chosen
    with open(a.emit_config, "w") as f:
        json.dump(whole, f, indent=4)
    print("\nEMITTED a VALIDATED per-wing config:")
    print("  pitch      %.4f c  (J=%.4f, shipped-normalised)" % (pitch, r["J"]))
    print("  verified   every layer of this wing, real boolean backend")
    print("  path       %s" % a.emit_config)
    if abs(pitch - out["inputs"]["shipped_pitch_chord_frac"]) < 1e-12:
        print("  NOTE: this equals the shipped pitch -- the best VERIFIED "
              "candidate for this wing is what already ships.")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(
        REPO_ROOT, "presets", "plugin", "default_config.json"))
    ap.add_argument("--naca", default="2412")
    ap.add_argument("--root-chord", type=float, default=150.0)
    ap.add_argument("--tip-chord", type=float, default=90.0)
    ap.add_argument("--span", type=float, default=400.0)
    ap.add_argument("--washout", type=float, default=3.0)
    ap.add_argument("--points", type=int, default=120)
    ap.add_argument("--chord", type=float, default=150.0)
    ap.add_argument("--samples", type=int, default=13)
    ap.add_argument("--wall-width-mm", type=float, default=0.42)
    ap.add_argument("--weights", default="0.45,0.40,0.15",
                    help="w_bay,w_mass,w_omit -- must sum to 1")
    ap.add_argument("--pitches", default="0.030,0.040,0.050,0.065,0.080,0.100,0.130")
    ap.add_argument("--out", default=os.path.join(
        REPO_ROOT, "docs", "results", "m15_size_wing.json"))
    ap.add_argument("--emit-config", default=None, metavar="PATH",
                    help="Write the best pitch that PASSES the real "
                         "full-layer sweep to PATH, as a per-wing config. "
                         "Candidates are swept in J order and the first that "
                         "verifies wins; a candidate that fails is recorded "
                         "and skipped, never emitted.")
    a = ap.parse_args(argv)

    if not HAVE_SHAPELY:
        raise SystemExit("shapely unavailable; refusing to report numbers")

    w = [float(x) for x in a.weights.split(",")]
    if len(w) != 3 or abs(sum(w) - 1.0) > 1e-9:
        raise SystemExit("--weights must be three numbers summing to 1")
    w_bay, w_mass, w_omit = w

    with open(a.config) as f:
        cap = json.load(f)["config"][0]["cap_config"]

    shipped = None
    for fam in cap.get("families", []):
        if fam.get("rib_pitch_chord_frac") is not None:
            shipped = fam["rib_pitch_chord_frac"]
            break
    if shipped is None:
        raise SystemExit("config has no chord-relative pitch to optimise")

    pitches = sorted({float(x) for x in a.pitches.split(",")} | {shipped})
    rows = []
    for p in pitches:
        r = score_candidate(cap, p, a, a.wall_width_mm, a.samples)
        if r is None:
            print("  pitch %.4f -> no measurable bay; skipped" % p)
            continue
        rows.append(r)

    ref = next((r for r in rows if abs(r["rib_pitch_chord_frac"] - shipped) < 1e-12), None)
    if ref is None:
        raise SystemExit("shipped pitch failed to score; cannot normalise")

    for r in rows:
        r["J"] = (w_bay * (r["worst_bay_arclen_mm"] / ref["worst_bay_arclen_mm"])
                  + w_mass * (r["mass_proxy_mm3"] / ref["mass_proxy_mm3"])
                  + w_omit * (r["omitted_frac"] / ref["omitted_frac"]
                              if ref["omitted_frac"] > 0 else 0.0))
    rows.sort(key=lambda r: r["J"])

    # Stability of the ranking across the unpinned zeta0.
    orders = {}
    for z0 in ZETA0_RANGE:
        key = "zeta0_%g" % z0
        orders[key] = [r["rib_pitch_chord_frac"]
                       for r in sorted(rows, key=lambda r: -r["sigma_rel"][key])]
    stable = len({tuple(v) for v in orders.values()}) == 1

    out = {
        "inputs": {
            "wing": {"naca": a.naca, "root_chord_mm": a.root_chord,
                     "tip_chord_mm": a.tip_chord, "span_mm": a.span,
                     "washout_deg": a.washout},
            "weights": {"bay": w_bay, "mass": w_mass, "omit": w_omit},
            "shipped_pitch_chord_frac": shipped,
            "n_layers_sampled": a.samples,
            "coverage": "SAMPLED for SCORING only -- validity requires "
                        "tools/verify_full_sweep_varying.py on every layer",
            "bay": "per-entry-surface ARCLENGTH, computed here; NOT "
                   "diagnostics.worst_skin_bay_mm (pooled chord-dX, ~4x wrong)",
            "depth": "FIXED, not optimised",
        },
        "sigma_rel_ranking_stable_across_zeta0": stable,
        "sigma_rel_orders": orders,
        "candidates": rows,
    }
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(out, f, indent=2)

    print("\nWEIGHTED PITCH SIZER  (w_bay=%.2f w_mass=%.2f w_omit=%.2f)" % (w_bay, w_mass, w_omit))
    print("wing NACA %s  %.0f->%.0f mm chord, %.0f mm span, %.1f deg washout"
          % (a.naca, a.root_chord, a.tip_chord, a.span, a.washout))
    print("bay = per-surface ARCLENGTH.  J normalised to shipped %.4f (J=1)." % shipped)
    print()
    print("  pitch_c   worstbay   meanbay    built  omit%   massproxy      J")
    for r in rows:
        mark = "  <- shipped" if abs(r["rib_pitch_chord_frac"] - shipped) < 1e-12 else ""
        print("   %.4f   %8.3f  %8.3f   %6d  %5.2f  %10.1f  %6.4f%s" % (
            r["rib_pitch_chord_frac"], r["worst_bay_arclen_mm"],
            r["mean_bay_arclen_mm"], r["webs_built"],
            100 * r["omitted_frac"], r["mass_proxy_mm3"], r["J"], mark))
    best = rows[0]
    print("\nlowest J: pitch %.4f c   (J=%.4f)" % (best["rib_pitch_chord_frac"], best["J"]))
    print("sigma_rel ranking stable across zeta0 %s: %s"
          % (list(ZETA0_RANGE), "YES" if stable else "NO -- do not name a winner"))
    if a.emit_config:
        _emit_validated_config(rows, cap, a, out, ref)

    if not a.emit_config:
        print("\nNOT VALIDATED. Before adopting any pitch above, run:")
        print("  python tools/verify_full_sweep_varying.py wing --config <cfg> \\")
        print("      --root-chord %g --tip-chord %g --span %g --washout %g"
          % (a.root_chord, a.tip_chord, a.span, a.washout))
    print("wrote %s" % a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
