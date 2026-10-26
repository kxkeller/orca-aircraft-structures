#!/usr/bin/env python3
"""Build the wing fixture with a genuinely BORED root-to-tip spar channel.

WHY THIS IS BUILT FROM PARTS. Lofting the wing (which fan-caps its own root and
tip) and adding a CAPPED cylinder inside it produces a SEALED internal cavity,
not a bored channel: the wing's end cap and the cylinder's end cap sit
coincident at each end, which reads as a non-manifold surface at the top and
bottom and leaves the spar with nowhere to enter.

A root-to-tip channel needs three things:

  1. the wing's SIDE walls only -- not its end caps;
  2. end caps rebuilt as ANNULI, outer section minus the spar circle, so the
     channel opens at root and tip;
  3. the cylinder OPEN at both ends, wound so its normals face the axis (away
     from the material).

The spar axis follows each station's own 0.30c mid-depth point, so the channel
stays inside a tapering, twisting wing instead of walking out of it.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for _p in (str(REPO_ROOT), str(REPO_ROOT / "tools")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import make_fixtures as mf  # noqa: E402
from make_wing import wing_params_fn  # noqa: E402


def _norm(a, b, c):
    u = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
    v = (c[0]-a[0], c[1]-a[1], c[2]-a[2])
    n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
    L = math.sqrt(sum(q*q for q in n)) or 1.0
    return (n[0]/L, n[1]/L, n[2]/L)


def T(a, b, c):
    return (_norm(a, b, c), a, b, c)


def build(naca, root, tip, span, washout, npts, nsta, spar_d, spar_frac, out_dir):
    m, p, t = mf.parse_naca4(naca)
    params = wing_params_fn(root, tip, span, washout)
    z_st = [span * i / (nsta - 1) for i in range(nsta)]
    stations, _unit, _shift = mf.build_loft_stations(
        m, p, t, npts, z_st, params, pivot_frac_x=0.25)

    # spar axis: each station's own 0.30c mid-depth point
    axis = []
    for z, outl in stations:
        xs = [q[0] for q in outl]
        cx = min(xs) + spar_frac * (max(xs) - min(xs))
        band = (max(xs) - min(xs)) * 0.02
        ys = [q[1] for q in outl if abs(q[0] - cx) < band]
        axis.append((cx, (min(ys) + max(ys)) / 2.0, z))

    n_out = len(stations[0][1])
    r = spar_d / 2.0

    def ring(i):
        """Spar ring at station i, with one point per OUTLINE point placed at
        that outline point's own ANGLE about the spar centre.

        Index-matching an outline to a uniform-angle circle does NOT work: the
        outline runs LE->TE with variable
        point spacing while a circle is uniform in angle, so zipping k-to-k
        folds the cap triangles across one another. Volume still integrates
        correctly under the divergence theorem when the winding is consistent,
        which is exactly why ``--info`` reported manifold=yes, parts=1 and the
        right volume on a visibly broken mesh.

        Matching by angle makes every cap quad a proper radial trapezoid and
        cannot self-cross, provided the section is star-shaped about the spar
        centre -- true for an aerofoil about a point near 0.30c mid-depth.
        """
        cx, cy, z = axis[i]
        outl = stations[i][1]
        pts = []
        for (ox, oy) in outl:
            th = math.atan2(oy - cy, ox - cx)
            pts.append((cx + r*math.cos(th), cy + r*math.sin(th), z))
        return pts

    def v3(z, xy):
        return (xy[0], xy[1], z)

    tris = []

    # 1. wing SIDE walls only (loft_triangles would also cap the ends)
    for s in range(len(stations) - 1):
        z0, o0 = stations[s]
        z1, o1 = stations[s + 1]
        for k in range(n_out):
            j = (k + 1) % n_out
            a, b = v3(z0, o0[k]), v3(z0, o0[j])
            c, d = v3(z1, o1[k]), v3(z1, o1[j])
            tris.append(T(a, b, c))
            tris.append(T(b, d, c))

    # 2. cylinder side wall, OPEN at both ends, wound to face the axis
    for s in range(len(axis) - 1):
        a, b = ring(s), ring(s + 1)
        for k in range(n_out):
            j = (k + 1) % n_out
            tris.append(T(a[k], a[j], b[k]))
            tris.append(T(a[j], b[j], b[k]))

    # 3. end caps as ANNULI so the channel opens at root and tip
    for idx, is_root in ((0, True), (len(stations) - 1, False)):
        z, outl = stations[idx]
        rg = ring(idx)
        for k in range(n_out):
            j = (k + 1) % n_out
            o_k, o_j = v3(z, outl[k]), v3(z, outl[j])
            r_k, r_j = rg[k], rg[j]
            if is_root:
                tris.append(T(o_k, r_k, o_j))
                tris.append(T(o_j, r_k, r_j))
            else:
                tris.append(T(o_k, o_j, r_k))
                tris.append(T(o_j, r_j, r_k))

    out_dir.mkdir(parents=True, exist_ok=True)
    stem = "naca%s_wing_r%d_t%d_s%d_w%g_spar%g" % (
        naca, int(root), int(tip), int(span), washout, spar_d)
    path = out_dir / (stem + ".stl")
    mf.write_binary_stl(path, tris)
    print("wrote %s" % path)
    print("  triangles %d" % len(tris))
    print("  sha256    %s" % mf.sha256_of(path))
    print("  spar      %.1f mm at %.2fc, root-to-tip, OPEN at both ends" % (spar_d, spar_frac))
    return path


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--naca", default="2412")
    ap.add_argument("--root-chord", type=float, default=150.0)
    ap.add_argument("--tip-chord", type=float, default=120.0)
    ap.add_argument("--span", type=float, default=120.0)
    ap.add_argument("--washout", type=float, default=1.5)
    ap.add_argument("--points", type=int, default=120)
    ap.add_argument("--stations", type=int, default=41)
    ap.add_argument("--spar-dia", type=float, default=10.0)
    ap.add_argument("--spar-frac", type=float, default=0.30)
    a = ap.parse_args(argv)
    build(a.naca, a.root_chord, a.tip_chord, a.span, a.washout,
          a.points, a.stations, a.spar_dia, a.spar_frac,
          REPO_ROOT / "tests" / "fixtures" / "wing")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
