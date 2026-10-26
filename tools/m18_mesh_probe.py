#!/usr/bin/env python3
"""Validate MeshSectioner against REAL wing meshes, and time it.

WHY THIS EXISTS SEPARATELY FROM THE UNIT TESTS. `tests/unit/test_mesh_section.py`
builds its meshes analytically -- boxes and prisms -- which is right for pinning
the algorithm but says nothing about a 39k-facet tapered, twisted wing with a
swept spar channel. Two things need checking against the real article:

  1. that the detected interior loop IS the spar channel, at the position and
     diameter the CAD was told to make, and
  2. what it COSTS. Runtime is measured rather than assumed. A wing slice is
     ~600 layers, so a per-layer cost that looks trivial in isolation is
     multiplied by 600 and lands on the critical path of every slice.

The expected hole positions are not hardcoded: they are derived from the
chord at each station, so this stays a real check rather than a restatement of
whatever the code currently produces.

Usage:
    python tools/m18_mesh_probe.py
    python tools/m18_mesh_probe.py --stl tests/fixtures/wing/m18_wing_spar10_at0.45.stl
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time
from typing import Dict, List, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO_ROOT, os.path.join(REPO_ROOT, "tools")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aircraft_core.mesh_section import MeshSectioner  # noqa: E402

LAYER_HEIGHT_MM = 0.2


def read_binary_stl(path: str) -> Tuple[List[Tuple[float, float, float]],
                                        List[Tuple[int, int, int]]]:
    """Vertices and triangles from a binary STL.

    STL has no vertex sharing, so this welds identical coordinates back
    together -- MeshSectioner assembles loops by matching segment endpoints,
    and unwelded duplicates would produce endpoints that are numerically equal
    but arrived at independently.
    """
    with open(path, "rb") as fh:
        data = fh.read()
    count = struct.unpack("<I", data[80:84])[0]
    verts: List[Tuple[float, float, float]] = []
    index: Dict[Tuple[float, float, float], int] = {}
    tris: List[Tuple[int, int, int]] = []
    off = 84
    for _ in range(count):
        vals = struct.unpack("<12f", data[off:off + 48])
        off += 50
        ids = []
        for k in (3, 6, 9):
            key = (vals[k], vals[k + 1], vals[k + 2])
            i = index.get(key)
            if i is None:
                i = len(verts)
                index[key] = i
                verts.append(key)
            ids.append(i)
        tris.append(tuple(ids))
    return verts, tris


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stl", default=os.path.join(
        REPO_ROOT, "tests", "fixtures", "wing", "m18_wing_spar10_at0.30.stl"))
    ap.add_argument("--root-chord", type=float, default=150.0)
    ap.add_argument("--tip-chord", type=float, default=120.0)
    ap.add_argument("--span", type=float, default=120.0)
    ap.add_argument("--expect-frac", type=float, default=0.30)
    ap.add_argument("--expect-dia", type=float, default=10.0)
    ap.add_argument("--out", default=os.path.join(
        REPO_ROOT, "docs", "results", "m18_mesh_probe.json"))
    a = ap.parse_args(argv)

    t0 = time.perf_counter()
    verts, tris = read_binary_stl(a.stl)
    t_read = time.perf_counter() - t0

    t0 = time.perf_counter()
    sec = MeshSectioner(verts, tris)
    t_build = time.perf_counter() - t0

    n_layers = int(round(a.span / LAYER_HEIGHT_MM))
    rows = []
    worst_dx = 0.0
    worst_dd = 0.0
    interior_seen = 0
    t0 = time.perf_counter()
    failures = []
    for idx in range(1, n_layers):
        z = idx * LAYER_HEIGHT_MM
        try:
            loops = sec.section_at(z)
        except Exception as exc:  # noqa: BLE001
            failures.append({"layer": idx, "z": round(z, 3),
                             "error": "{}: {}".format(type(exc).__name__, exc)[:150]})
            continue
        outer = [lp for lp in loops if not lp.is_interior]
        inner = [lp for lp in loops if lp.is_interior]
        if not outer:
            failures.append({"layer": idx, "z": round(z, 3),
                             "error": "no outer loop"})
            continue
        if not inner:
            continue
        interior_seen += 1
        o = outer[0]
        chord = o.bbox[2] - o.bbox[0]
        expect_x = o.bbox[0] + a.expect_frac * chord
        h = inner[0]
        dx = abs(h.centroid[0] - expect_x)
        dd = abs(h.equivalent_diameter_mm - a.expect_dia)
        worst_dx = max(worst_dx, dx)
        worst_dd = max(worst_dd, dd)
        if idx % 120 == 0:
            rows.append({"layer": idx, "z": round(z, 2),
                         "chord_mm": round(chord, 3),
                         "hole_cx": round(h.centroid[0], 4),
                         "expected_cx": round(expect_x, 4),
                         "dx": round(dx, 4),
                         "hole_dia": round(h.equivalent_diameter_mm, 4),
                         "n_interior": len(inner)})
    t_sweep = time.perf_counter() - t0

    out = {
        "stl": os.path.relpath(a.stl, REPO_ROOT),
        "triangles": sec.triangle_count,
        "vertices": len(verts),
        "layers_swept": n_layers - 1,
        "layers_with_interior_loop": interior_seen,
        "expected": {"frac": a.expect_frac, "dia_mm": a.expect_dia},
        "worst_centroid_error_mm": round(worst_dx, 4),
        "worst_diameter_error_mm": round(worst_dd, 4),
        "failures": failures[:10],
        "failure_count": len(failures),
        "samples": rows,
        "timing_s": {
            "read_stl": round(t_read, 4),
            "build_index": round(t_build, 4),
            "sweep_all_layers": round(t_sweep, 4),
            "per_layer_ms": round(1000.0 * t_sweep / max(n_layers - 1, 1), 4),
        },
    }
    with open(a.out, "w") as fh:
        json.dump(out, fh, indent=2)

    print("MESH PROBE -- %s" % os.path.basename(a.stl))
    print("  %d triangles, %d welded vertices" % (sec.triangle_count, len(verts)))
    print("  interior loop found on %d/%d layers"
          % (interior_seen, n_layers - 1))
    print("  worst centroid error vs expected %.3fc : %.4f mm"
          % (a.expect_frac, worst_dx))
    print("  worst diameter error vs %.1f mm       : %.4f mm"
          % (a.expect_dia, worst_dd))
    if failures:
        print("  FAILURES: %d, first: %s" % (len(failures), failures[0]))
    print("  timing: read %.3f s, index %.3f s, sweep %.3f s (%.3f ms/layer)"
          % (t_read, t_build, t_sweep, out["timing_s"]["per_layer_ms"]))
    for r in rows:
        print("    layer %4d z=%6.2f chord %7.3f  cx %9.4f (want %9.4f, d=%.4f)"
              "  dia %.4f  n_int=%d"
              % (r["layer"], r["z"], r["chord_mm"], r["hole_cx"],
                 r["expected_cx"], r["dx"], r["hole_dia"], r["n_interior"]))
    print("wrote %s" % a.out)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
