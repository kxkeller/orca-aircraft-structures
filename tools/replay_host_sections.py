"""Offline replay of the mesh-alignment guard's decision from a recorded
``m1_host_sections.json.gz`` sidecar.

The sidecar is written by ``plugin/aircraft_structures/capability.py``
(``AircraftStructuresM1._write_diagnostics``) next to ``m1_report.json``
whenever ``StructureSpec.record_host_geometry_sidecar`` is true (the default).
For every sectioned layer it records the host's own slice region polygons, our
mesh section's guard loops, the six deltas, the per-term EFFECTIVE tolerance
actually used that layer (sensitivity widening already applied -- a replay
cannot redo the two extra ``section_at`` probes that produced it, so it must be
recorded, not recomputed), and the decision ``_check_mesh_alignment`` reached.

This tool re-runs ``aircraft_core.alignment.decide_alignment`` -- the SAME
pure function ``capability.py`` calls in-host (see that module's docstring
above ``decide_alignment``) -- against each layer's recorded deltas/
effective-tolerance/near-top-extremum/tip-alignment-skip-enabled, and reports
whether it reproduces the in-host run's own recorded decision. It never
re-derives deltas from the recorded polygons (that would be a DIFFERENT,
useful check -- "did our polygon-comparison code regress" -- which this tool
deliberately does not attempt; see "Not attempted" below).

This is a plain offline tool: no ``orca`` import, importable and runnable
with only ``aircraft_core`` and the Python standard library.

Usage::

    python tools/replay_host_sections.py PATH/TO/m1_host_sections.json.gz
    python tools/replay_host_sections.py --report PATH/TO/m1_report.json

The ``--report`` form derives the sidecar path the same way capability.py
derives it (same directory, filename ``m1_host_sections.json.gz``).

Not attempted here (documented, not silently absent):

- Re-deriving the six deltas from the recorded host/mesh polygons. The
  sidecar records ``deltas_mm``/``effective_tolerance_mm`` as CAPABILITY.PY
  computed them; replaying the polygon geometry itself (bbox/centroid
  extraction) would double as a regression test for that extraction code,
  which is a different, legitimate tool this one does not attempt to be.
- Any graded local-disagreement scoring. This tool replays the binary guard
  only.
"""
from __future__ import annotations

import argparse
import gzip
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from aircraft_core.alignment import decide_alignment  # noqa: E402


SIDECAR_FILENAME = "m1_host_sections.json.gz"


def sidecar_path_from_report(report_path: Path) -> Path:
    """Mirrors ``capability.py._write_diagnostics``'s own derivation: the sidecar
    sits in the SAME directory as ``m1_report.json``, under the fixed filename
    ``m1_host_sections.json.gz`` -- never derived from ``APPDATA`` or any other
    root, matching the write side."""
    return report_path.parent / SIDECAR_FILENAME


def load_sidecar(path: Path) -> List[Dict[str, Any]]:
    """Returns the per-layer entry list. The on-disk sidecar is wrapped as
    ``{"run_id":..., "layers": [...]}`` so its own ``run_id`` can be checked
    against the report's ``host_sections_sidecar.run_id``
    (``run_id_from_sidecar`` below); this function returns just the layer
    list."""
    with gzip.open(path, "rb") as handle:
        payload = json.loads(handle.read().decode("utf-8"))
    if isinstance(payload, dict) and "layers" in payload:
        return payload["layers"]
    # Also accept a bare list of layer entries.
    return payload


def run_id_from_sidecar(path: Path) -> Optional[str]:
    with gzip.open(path, "rb") as handle:
        payload = json.loads(handle.read().decode("utf-8"))
    if isinstance(payload, dict):
        return payload.get("run_id")
    return None


def replay_layer(entry: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Returns the replayed decision dict for one sidecar entry, or ``None``
    if the guard never ran for this layer (``guard_ran: False`` -- a
    root-solid layer, or a layer whose raise happened before the guard's
    own decision point; see ``capability.py``'s own comment on
    ``_start_sidecar_entry`` for why those layers are still present, with
    an honest ``guard_ran: False``, rather than a gap in the sequence)."""
    if not entry.get("guard_ran"):
        return None
    return decide_alignment(
        entry["deltas_mm"],
        entry["effective_tolerance_mm"],
        entry["near_top_extremum"],
        entry["tip_alignment_skip_enabled"],
    )


def replay(entries: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Replays every entry and compares against the RECORDED decision
    (what the in-host run's own ``_check_mesh_alignment`` reached that
    layer, per its own sidecar entry). Returns a summary dict:

        {
            "total_layers": int,
            "guard_ran_layers": int,
            "agreements": int,
            "disagreements": [ {layer_index, recorded, replayed}, ... ],
            "first_failure_layer": int | None,
        }

    A disagreement means this tool's re-run of TODAY's decision logic does
    not match what the in-host run's OWN copy of that logic decided at the
    time -- which can only happen if the two are running different code
    (a version drift between the sidecar's provenance and the currently
    checked-out ``aircraft_core.alignment``), since both read the same
    recorded deltas/tolerances. Zero disagreements is the expected, and
    only interesting, outcome for a sidecar recorded by the code currently
    checked out.
    """
    disagreements = []
    agreements = 0
    guard_ran = 0
    first_failure_layer = None
    for entry in entries:
        replayed = replay_layer(entry)
        if replayed is None:
            continue
        guard_ran += 1
        recorded = entry.get("decision")
        if replayed == recorded:
            agreements += 1
        else:
            disagreements.append({
                "layer_index": entry["layer_index"],
                "recorded": recorded,
                "replayed": replayed,
            })
        if (
            first_failure_layer is None
            and replayed is not None
            and replayed["outcome"].startswith("raise")
        ):
            first_failure_layer = entry["layer_index"]
    return {
        "total_layers": len(entries),
        "guard_ran_layers": guard_ran,
        "agreements": agreements,
        "disagreements": disagreements,
        "first_failure_layer": first_failure_layer,
    }


def _format_summary(summary: Dict[str, Any]) -> str:
    lines = [
        "layers: {} total, {} guard-ran, {} agree, {} disagree".format(
            summary["total_layers"], summary["guard_ran_layers"],
            summary["agreements"], len(summary["disagreements"]),
        ),
    ]
    if summary["first_failure_layer"] is not None:
        lines.append(
            "replayed outcome: FAILS at layer {}".format(
                summary["first_failure_layer"])
        )
    else:
        lines.append("replayed outcome: PASSES (no layer refused)")
    for d in summary["disagreements"]:
        lines.append(
            "  DISAGREEMENT layer {}: recorded={!r} replayed={!r}".format(
                d["layer_index"], d["recorded"], d["replayed"])
        )
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "sidecar", nargs="?", default=None,
        help="Path to m1_host_sections.json.gz")
    group.add_argument(
        "--report", default=None,
        help="Path to m1_report.json; the sidecar is derived from its "
             "directory")
    args = parser.parse_args(argv)

    if args.report is not None:
        path = sidecar_path_from_report(Path(args.report))
    else:
        path = Path(args.sidecar)

    if not path.exists():
        print("sidecar not found: {}".format(path), file=sys.stderr)
        return 2

    entries = load_sidecar(path)
    summary = replay(entries)
    print(_format_summary(summary))
    return 1 if summary["disagreements"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
