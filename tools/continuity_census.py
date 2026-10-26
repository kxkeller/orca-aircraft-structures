#!/usr/bin/env python3
"""The web-continuity census of a PLACED part, offline.

Replays a placed STL through the existing mesh pre-flight (``tools/
preflight_wing.py``: ``MeshSectioner -> StructureGenerator -> SlotBuilder ->
diff -> TopologyValidator``, ``--keep-placement``), which records the same
per-web rows the in-host report carries (``aircraft_core/continuity.py``), and
runs the continuity checker over them. The result is the baseline count of
blinks per web (``per_station_reentrant`` off unless asked).

Usage:
    python tools/continuity_census.py PLACED.stl --out-dir DIR [--label NAME]
        [--config PATH] [--up-axis z] [--layer-height 0.2]
        [--wall-width-mm 0.42] [--per-station-reentrant]
        [--from-preflight PREFLIGHT.json]

Writes ``DIR/<label>_census.json`` (summary, per-family, per-web table, whole-
layer gaps, known differences), ``DIR/<label>_violations.json`` (every
violation, uncapped) and ``DIR/<label>_summary.txt``. ``--from-preflight``
re-analyses a saved preflight JSON without re-running the sweep.

KNOWN MESH-vs-HOST DIFFERENCES (stated in every output, because they decide how
far the baseline can be read as "what the host did"):

* The sweep sections the ORIGINAL mesh, not the host's libslic3r slice; in the
  body the two outlines agree to ~0.002 mm (median 0.0005, p99 0.002 in the
  in-host alignment records), but the tip layers disagree more.
* No slice-closing emulation: preflight predicted 27 ``multi_island_layer``
  shutdowns (outside half, layers 147-173) that the host did not have.
* No pair ablation, no pinch check: layers the host rescued
  (947, 454-458) fail the sweep, and those failed layers carry no web rows,
  so every web crossing one reads as absent there. Blinks whose gap contains a
  mesh-replay failed or multi-island layer are counted separately
  (``blinks_adjacent_to_replay_artefact``).
* No latch walk in the per-layer context (the latch is only summarised),
  so socket/flank on-off follows the raw per-layer channel fit.
* Host-only tip shutdowns (``host_tip_region_empty``, ``tip_alignment_skip``)
  need the host region and are not reproduced.
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import json
import os
import sys
from typing import Dict, List, Optional

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS_DIR = os.path.join(REPO_ROOT, "tools")
for _p in (REPO_ROOT, TOOLS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aircraft_core.continuity import (  # noqa: E402
    DEFAULT_DX_MAX_MM,
    VIOLATION_BLINK,
    check_continuity,
    default_exemptions,
    layers_from_report,
)


TOOL_VERSION = "continuity_census/1"

KNOWN_DIFFERENCES = [
    "Mesh replay sections the original mesh, not the host slice (body outlines agree to ~0.002 mm; tip layers more).",
    "No slice-closing emulation (preflight over-predicts multi_island_layer, e.g. outside layers 147-173).",
    "No pair ablation / pinch shutdown: host-rescued layers fail the replay and carry no web rows (read as absent).",
    "No latch walk in the per-layer context: socket/flank on-off follows the raw per-layer fit.",
    "Host-only tip shutdowns (host_tip_*, tip_alignment_skip) are not reproduced.",
    "The existing in-host final reports carry per-family COUNTS only; they have no per-web records to run the checker on.",
]


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _z_runs(layer_indices: List[int], z_of: Dict[int, float]) -> List[dict]:
    runs: List[dict] = []
    for idx in sorted(layer_indices):
        if runs and idx == runs[-1]["layer_last"] + 1:
            runs[-1]["layer_last"] = idx
            runs[-1]["z_last"] = z_of.get(idx)
            runs[-1]["layers"] += 1
        else:
            runs.append({"layer_first": idx, "layer_last": idx,
                         "z_first": z_of.get(idx), "z_last": z_of.get(idx),
                         "layers": 1})
    return runs


def analyse(preflight: dict, *, label: str, per_station_reentrant: bool,
            dx_max_mm: float = DEFAULT_DX_MAX_MM):
    """``(census_doc, violations)`` for one preflight result (pure; no I/O).
    ``violations`` is the full, uncapped checker list (blinks annotated with
    whether their gap contains a mesh-replay artefact layer)."""
    meta = preflight.get("t3_meta") or {}
    summary = preflight.get("t3_summary") or {}
    layers = layers_from_report({"layers": preflight.get("t3_layers") or []})
    z_of = {rec["layer_index"]: rec["print_z"] for rec in layers}
    root_layers = int(meta.get("root_solid_layers") or 0)
    assumptions = preflight.get("assumptions") or {}
    ramp_mm = float(assumptions.get("layer_height_mm") or 0.2)
    wall_mm = float(assumptions.get("wall_width_mm") or 0.42)
    cap_config = preflight.get("cap_config") or {}
    family_names = {
        i: fam.get("name") for i, fam in enumerate(cap_config.get("families") or [])
    }
    family_names.setdefault(-2, "spar_socket")
    family_names.setdefault(-7, "bore_socket")

    exempt = default_exemptions(layers, root_layers)
    result = check_continuity(
        layers, ramp_mm=ramp_mm, dx_max_mm=dx_max_mm, exempt=exempt,
        family_names=family_names, violation_cap=10 ** 9,
    )

    raw_layers = preflight.get("t3_layers") or []
    failed = {r["layer_index"]: (r.get("message") or "")[:160]
              for r in raw_layers if r.get("failed")}
    multi_island = {r["layer_index"] for r in raw_layers
                    if (r.get("extra") or {}).get("reason") == "multi_island_layer"}
    shutdown_reason = {r["layer_index"]: (r.get("extra") or {}).get("reason")
                       for r in raw_layers if r.get("shutdown")}
    artefact_layers = set(failed) | multi_island

    # Blinks and socket restarts, split by whether the gap contains a
    # mesh-replay artefact layer (a replay-failed or multi-island layer).
    blinks_artefact = blinks_genuine = 0
    restarts_artefact = restarts_genuine = 0
    blink_reason_counts: Dict[str, int] = {}
    blink_gap_hist = {"1": 0, "2-4": 0, "5-20": 0, ">20": 0}
    blink_events_by_web: Dict[tuple, List[dict]] = {}
    for v in result["violations"]:
        if v["type"] not in (VIOLATION_BLINK, "socket_restart"):
            continue
        gap = range(v["layer"] - v["gap_layers"], v["layer"])
        v["adjacent_to_replay_artefact"] = any(j in artefact_layers for j in gap)
        v["gap_layer_reasons"] = sorted({
            str(shutdown_reason.get(j) or ("replay_failed" if j in failed else "no_structure"))
            for j in gap
        })
        if v["type"] == "socket_restart":
            if v["adjacent_to_replay_artefact"]:
                restarts_artefact += 1
            else:
                restarts_genuine += 1
            continue
        if v["adjacent_to_replay_artefact"]:
            blinks_artefact += 1
        else:
            blinks_genuine += 1
        # What kept the web out: its own omission code(s) during the gap, or
        # nothing recorded (the station was not a candidate -- it had drifted
        # off the chord extent, or the whole layer was unstructured).
        codes = v.get("gap_reasons") or ["not_a_candidate"]
        for code in codes:
            blink_reason_counts[code] = blink_reason_counts.get(code, 0) + 1
        g = v["gap_layers"]
        blink_gap_hist["1" if g == 1 else "2-4" if g <= 4 else "5-20" if g <= 20 else ">20"] += 1
        blink_events_by_web.setdefault(tuple(v["web"]), []).append(
            {"layer": v["layer"], "z": v["z"], "gap_layers": g,
             "reasons": codes, "replay_artefact": v["adjacent_to_replay_artefact"]})

    # Per-web table (compact): id, family name, layers present, runs, ends,
    # violations by type, z range present.
    presence: Dict[tuple, List[int]] = {}
    for rec in layers:
        for row in rec["webs"]:
            presence.setdefault((row[0], row[1], row[2]), []).append(rec["layer_index"])
    by_web_viol: Dict[tuple, dict] = {}
    for v in result["violations"]:
        by_web_viol.setdefault(tuple(v["web"]), {}).setdefault(v["type"], 0)
        by_web_viol[tuple(v["web"])][v["type"]] += 1
    per_web = []
    for web in sorted(presence, key=lambda w: (w[0], w[1], str(w[2]))):
        idx = presence[web]
        runs = _z_runs(idx, z_of)
        per_web.append({
            "web": list(web), "family": family_names.get(web[0]),
            "layers": len(idx), "runs": len(runs),
            "z_first": z_of.get(idx[0]), "z_last": z_of.get(idx[-1]),
            "violations": by_web_viol.get(web, {}),
        })

    # Whole-layer gaps: runs of layers with no web rows between structured layers.
    structured = sorted(rec["layer_index"] for rec in layers if rec["webs"])
    whole_layer_gaps = []
    if structured:
        have = set(structured)
        gap_layers = [i for i in range(structured[0], structured[-1] + 1) if i not in have]
        for run in _z_runs(gap_layers, z_of):
            reasons = sorted({
                str(shutdown_reason.get(j) or ("replay_failed" if j in failed else "no_structure"))
                for j in range(run["layer_first"], run["layer_last"] + 1)
            })
            run["reasons"] = reasons
            run["replay_artefact"] = any(
                j in artefact_layers
                for j in range(run["layer_first"], run["layer_last"] + 1))
            whole_layer_gaps.append(run)

    # How big are the X steps the checker flags? (dx_max is half a wall, the
    # nominal drift is 0.2 mm/layer and the chord taper makes the comb breathe,
    # so drift alone can read as an x_step on outer stations.)
    x_bins = [("<=0.25", 0.25), ("0.25-0.30", 0.30), ("0.30-0.50", 0.50),
              ("0.50-1.0", 1.0), (">1.0", float("inf"))]
    x_hist = {name: 0 for name, _ in x_bins}
    x_hist_merged = {name: 0 for name, _ in x_bins}
    for v in result["violations"]:
        if v["type"] != "x_step":
            continue
        for name, hi in x_bins:
            if abs(v["dx"]) <= hi:
                x_hist[name] += 1
                if v.get("merged"):
                    x_hist_merged[name] += 1
                break

    # Report-size cost of the per-web rows, measured in the report's own
    # serialization (json.dump indent=2, sort_keys=True).
    def _size(rows):
        return len(json.dumps(rows, indent=2, sort_keys=True, default=str))
    stripped = []
    for rec in raw_layers:
        rec2 = dict(rec)
        extra2 = dict(rec2.get("extra") or {})
        extra2.pop("webs", None)
        extra2.pop("web_omit", None)
        rec2["extra"] = extra2
        stripped.append(rec2)
    size_with, size_without = _size(raw_layers), _size(stripped)

    counts = result["violation_counts"]
    fam_summary = {}
    for fi, entry in result["by_family"].items():
        fam_summary[fi] = {
            "name": entry["name"], "webs": entry["webs"],
            "web_layers": entry["web_layers"], "runs": entry["runs"],
            "ends": entry["ends"], "violations": entry["violations"],
        }

    return {
        "tool": TOOL_VERSION,
        "label": label,
        "stl_path": preflight.get("stl_path"),
        "stl_sha256": preflight.get("stl_sha256"),
        "config_sha256": preflight.get("config_sha256"),
        "per_station_reentrant": per_station_reentrant,
        "ramp_mm": ramp_mm,
        "dx_max_mm": dx_max_mm,
        "side_wall_steps": result["side_wall_steps"],
        "outer_wall_line_width_mm": wall_mm,
        "root_solid_layers": root_layers,
        "preflight_status": preflight.get("status"),
        "layers_seen": summary.get("layers_seen"),
        "mutated_count": summary.get("mutated_count"),
        "failed_count": summary.get("failed_count"),
        "shutdown_count": summary.get("shutdown_count"),
        "degradation_counts_by_kind": summary.get("degradation_counts_by_kind"),
        "mesh_replay_failed_layers": [
            {"layer": i, "z": z_of.get(i), "message": m} for i, m in sorted(failed.items())],
        "violation_counts": counts,
        "violation_total": result["violation_total"],
        "x_step_abs_dx_histogram": x_hist,
        "x_step_abs_dx_histogram_merged_only": x_hist_merged,
        "web_rows_report_bytes_indent2": size_with - size_without,
        "layers_report_bytes_with_rows_indent2": size_with,
        "layers_report_bytes_without_rows_indent2": size_without,
        "blinks_genuine": blinks_genuine,
        "blinks_adjacent_to_replay_artefact": blinks_artefact,
        "socket_restarts_genuine": restarts_genuine,
        "socket_restarts_adjacent_to_replay_artefact": restarts_artefact,
        "blink_gap_reason_counts": dict(sorted(blink_reason_counts.items(), key=lambda kv: -kv[1])),
        "blink_gap_length_histogram": blink_gap_hist,
        "worst_webs_by_blink": [
            {"web": list(w), "family": family_names.get(w[0]), "blinks": len(ev),
             "z_first": ev[0]["z"], "z_last": ev[-1]["z"], "events": ev[:8]}
            for w, ev in sorted(blink_events_by_web.items(),
                                key=lambda kv: (-len(kv[1]), kv[0]))[:10]
        ],
        "exempted_counts": result["exempted_counts"],
        "exempted_by_band": result["exempted_by_band"],
        "layers_checked": result["layers_checked"],
        "webs_tracked": result["webs_tracked"],
        "by_family": fam_summary,
        "worst_webs": result["worst_webs"],
        "whole_layer_gaps": whole_layer_gaps,
        "per_web": per_web,
        "known_differences": KNOWN_DIFFERENCES,
    }, result["violations"]


def _summary_text(doc: dict) -> str:
    lines = []
    lines.append("continuity census: {} ({})".format(doc["label"], doc["stl_path"]))
    lines.append("  stl sha256 {}  config sha256 {}".format(doc["stl_sha256"], doc["config_sha256"]))
    lines.append("  per_station_reentrant={}  ramp r={} mm  dx_max={} mm (side walls; aircraft_core.continuity.DX_MAX_DERIVATION)  root_solid_layers={}".format(
        doc["per_station_reentrant"], doc["ramp_mm"], doc["dx_max_mm"],
        doc["root_solid_layers"]))
    if doc.get("side_wall_steps"):
        lines.append("  side-wall steps of webs that carried depth: {}".format(
            json.dumps(doc["side_wall_steps"])))
    lines.append("  mesh replay: status={} seen={} mutated={} failed={} shutdown={} kinds={}".format(
        doc["preflight_status"], doc["layers_seen"], doc["mutated_count"],
        doc["failed_count"], doc["shutdown_count"], doc["degradation_counts_by_kind"]))
    lines.append("  webs tracked {}  layers {}".format(doc["webs_tracked"], doc["layers_checked"]))
    lines.append("  VIOLATIONS {}: {}".format(doc["violation_total"], doc["violation_counts"]))
    lines.append("    blinks genuine {} / adjacent to a replay failed|multi-island layer {}; socket restarts genuine {} / artefact {}".format(
        doc["blinks_genuine"], doc["blinks_adjacent_to_replay_artefact"],
        doc["socket_restarts_genuine"], doc["socket_restarts_adjacent_to_replay_artefact"]))
    lines.append("    blink gap reasons {}  gap lengths {}".format(
        doc["blink_gap_reason_counts"], doc["blink_gap_length_histogram"]))
    lines.append("    exempted {} {}".format(doc["exempted_counts"], doc["exempted_by_band"]))
    lines.append("    x_step |dx| histogram (all) {}  (merged-only) {}".format(
        doc["x_step_abs_dx_histogram"], doc["x_step_abs_dx_histogram_merged_only"]))
    lines.append("    web rows add {} bytes to the layer records ({} -> {}, json indent=2)".format(
        doc["web_rows_report_bytes_indent2"], doc["layers_report_bytes_without_rows_indent2"],
        doc["layers_report_bytes_with_rows_indent2"]))
    lines.append("  by family:")
    for fi, e in doc["by_family"].items():
        lines.append("    {:>3} {:<22} webs={:<3} web_layers={:<6} runs={:<4} ends={:<4} viol={}".format(
            fi, str(e["name"]), e["webs"], e["web_layers"], e["runs"], e["ends"],
            {k: v for k, v in e["violations"].items() if v}))
    lines.append("  worst webs by blink:")
    for w in doc["worst_webs_by_blink"]:
        lines.append("    {} {} blinks={} z {}..{}".format(
            w["web"], w["family"], w["blinks"], w["z_first"], w["z_last"]))
    lines.append("  worst webs (all violation types):")
    for w in doc["worst_webs"]:
        lines.append("    {} n={} {} z {}..{} runs={} layers={}".format(
            w["web"], w["violations"], w["by_type"], w["z_first"], w["z_last"],
            w["runs"], w["layers"]))
    lines.append("  whole-layer structure gaps (mid-part, structure above and below): {}".format(
        len(doc["whole_layer_gaps"])))
    for g in doc["whole_layer_gaps"]:
        lines.append("    layers {}-{} z {}..{} ({} layers) reasons={}{}".format(
            g["layer_first"], g["layer_last"], g["z_first"], g["z_last"], g["layers"],
            g["reasons"], "  [replay artefact]" if g["replay_artefact"] else ""))
    lines.append("  mesh-replay failed layers: {}".format(
        [f["layer"] for f in doc["mesh_replay_failed_layers"]]))
    lines.append("  known mesh-vs-host differences:")
    for d in doc["known_differences"]:
        lines.append("    - " + d)
    return "\n".join(lines) + "\n"


def run_preflight(stl: str, args, cfg_path: str, json_path: str) -> dict:
    import preflight_wing
    argv = [stl, "--config", cfg_path, "--up-axis", args.up_axis,
            "--layer-height", str(args.layer_height),
            "--wall-width-mm", str(args.wall_width_mm),
            "--json", json_path]
    if args.keep_placement:
        argv.append("--keep-placement")
    if args.root_end != "auto":
        argv += ["--root-end", args.root_end]
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = preflight_wing.main(argv)
    with open(json_path, "r", encoding="utf-8") as fh:
        doc = json.load(fh)
    doc["_preflight_exit_code"] = rc
    return doc


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stl", nargs="?")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--label", default=None)
    ap.add_argument("--config", default=None)
    ap.add_argument("--up-axis", choices=("auto", "x", "y", "z"), default="z")
    ap.add_argument("--root-end", choices=("auto", "low", "high"), default="auto")
    ap.add_argument("--layer-height", type=float, default=0.2)
    ap.add_argument("--wall-width-mm", type=float, default=0.42)
    ap.add_argument("--no-keep-placement", dest="keep_placement", action="store_false")
    ap.add_argument("--per-station-reentrant", action="store_true",
                    help="run with spec.per_station_reentrant=True (default: today's behaviour, off)")
    ap.add_argument("--dx-max-mm", type=float, default=DEFAULT_DX_MAX_MM,
                    help="lateral-continuity threshold on the slot side walls")
    ap.add_argument("--from-preflight", default=None,
                    help="analyse a saved preflight JSON instead of re-running the sweep")
    args = ap.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    label = args.label or (os.path.splitext(os.path.basename(args.stl or args.from_preflight))[0])

    if args.from_preflight:
        with open(args.from_preflight, "r", encoding="utf-8") as fh:
            preflight = json.load(fh)
    else:
        if not args.stl:
            ap.error("an STL (or --from-preflight) is required")
        import preflight_wing
        cfg_path = args.config or preflight_wing.DEFAULT_CONFIG_PATH
        if args.per_station_reentrant:
            with open(cfg_path, "r", encoding="utf-8") as fh:
                cfg_doc = json.load(fh)
            entry = cfg_doc["config"][0] if isinstance(cfg_doc["config"], list) else cfg_doc["config"]
            entry["cap_config"]["per_station_reentrant"] = True
            cfg_path = os.path.join(args.out_dir, label + "_config_per_station.json")
            with open(cfg_path, "w", encoding="utf-8") as fh:
                json.dump(cfg_doc, fh, indent=2)
        pf_path = os.path.join(args.out_dir, label + "_preflight_raw.json")
        preflight = run_preflight(args.stl, args, cfg_path, pf_path)
        # The raw preflight JSON is large and fully re-derivable; keep it out
        # of the committed folder unless asked.
        if os.environ.get("CENSUS_KEEP_RAW") != "1":
            try:
                os.remove(pf_path)
            except OSError:
                pass

    doc, violations = analyse(
        preflight, label=label, per_station_reentrant=bool(args.per_station_reentrant),
        dx_max_mm=args.dx_max_mm)
    with open(os.path.join(args.out_dir, label + "_violations.json"), "w", encoding="utf-8") as fh:
        json.dump({"label": label, "violation_total": len(violations),
                   "violations": violations}, fh, separators=(",", ":"))
    out_main = os.path.join(args.out_dir, label + "_census.json")
    with open(out_main, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, indent=1)
    text = _summary_text(doc)
    with open(os.path.join(args.out_dir, label + "_summary.txt"), "w", encoding="utf-8") as fh:
        fh.write(text)
    print(text)
    print("wrote", out_main)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
