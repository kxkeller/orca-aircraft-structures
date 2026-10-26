"""Measure what Spiral Vase actually emits at a narrow neck.

The analysis answers a question the TopologyValidator cannot: when two collinear
opposing cuts leave a ligament of material narrower than 2 x line_width, does
OrcaSlicer still emit ONE closed loop that passes through the neck, or does the
section fall apart / the slit vanish?

The fixture is tools/m22_neck_coupon.py's prism: a 90 x 12 x 20 mm block with
eight stations, each a pair of collinear slits cut inward from opposite faces,
sweeping ligament width against slit width.

This reads G-code only. It never writes G-code and never touches a printer.

What it reports, per station:
    detour_upper / detour_lower   how far the toolpath reaches into the notch
    slit_present                  whether the path enters the notch at all
    neck_gap                      closest approach of the upper and lower
                                  toolpath around the ligament

IMPORTANT: a result here licenses a TOPOLOGY claim only -- that the loop exists
and the neck survives as toolpath. It says nothing about whether the neck is
strong, and nothing about fusion. Both remain gated on print evidence.

Usage:
    python tools/m22_neck_analyse.py <file.gcode> [--json out.json]
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

# The coupon, as built by tools/m22_neck_coupon.py. Kept here so the analysis
# fails loudly if it is ever pointed at a different model.
BLOCK_X_MM = 90.0
BLOCK_Y_MM = 12.0            # -6 .. +6
STATIONS = [
    # (x_centre, slit_width_g, ligament_W)
    (10.0, 0.01, 0.42),
    (20.0, 0.01, 0.60),
    (30.0, 0.01, 0.84),
    (40.0, 0.01, 1.00),
    (50.0, 0.42, 0.42),
    (60.0, 0.42, 0.60),
    (70.0, 0.42, 0.84),
    (80.0, 0.42, 1.00),
]

LAYER_CHANGE = re.compile(r"^;LAYER_CHANGE\b")
Z_COMMENT = re.compile(r"^;Z:\s*([0-9.]+)")
END_OF_PRINT = re.compile(r"^;\s*(filament end gcode|TYPE:Custom)", re.IGNORECASE)
WORD = re.compile(r"([XYZEF])(-?\d*\.?\d+)")


class Layer:
    __slots__ = ("index", "z", "points", "travels", "extrusions")

    def __init__(self, index, z):
        self.index = index
        self.z = z
        self.points = []      # ordered (x, y) of EXTRUDING moves
        self.travels = 0
        self.extrusions = 0


def parse_layers(path):
    """Extract per-layer ordered extrusion points.

    Modal G-code: X/Y/E persist across lines, so the current position must be
    tracked rather than read per-line. E is relative in this profile, but the
    sign test below works for absolute E too because we compare against the
    previous value.
    """
    layers = []
    cur = None
    x = y = z = 0.0
    e_prev = 0.0
    relative_e = True
    stopped = False

    with open(path, "r", errors="ignore") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            if END_OF_PRINT.match(line):
                stopped = True
            if line.startswith(";"):
                if LAYER_CHANGE.match(line):
                    cur = Layer(len(layers), z)
                    layers.append(cur)
                    # The START custom block also carries ";TYPE:Custom", so the
                    # end-of-print guard must be armed per layer, not once.
                    stopped = False
                m = Z_COMMENT.match(line)
                if m and cur is not None:
                    cur.z = float(m.group(1))
                continue
            if line.startswith("M83"):
                relative_e = True
                continue
            if line.startswith("M82"):
                relative_e = False
                continue
            if line.startswith("G92"):
                if "E" in line:
                    e_prev = 0.0
                continue
            if not (line.startswith("G1") or line.startswith("G0")):
                continue

            words = dict((w[0], float(w[1])) for w in WORD.findall(line.split(";")[0]))
            nx = words.get("X", x)
            ny = words.get("Y", y)
            nz = words.get("Z", z)
            moved = (nx != x) or (ny != y)

            if "E" in words:
                e = words["E"]
                de = e if relative_e else (e - e_prev)
                e_prev = e if not relative_e else e_prev
            else:
                de = 0.0

            if cur is not None and not stopped and moved:
                if de > 0:
                    cur.extrusions += 1
                    if not cur.points:
                        cur.points.append((x, y))
                    cur.points.append((nx, ny))
                else:
                    cur.travels += 1

            x, y, z = nx, ny, nz

    return layers


def body_layers(layers, root_layers=5):
    """Layers that should be pure spiral: past the solid base, with geometry."""
    return [l for l in layers[root_layers:] if len(l.points) > 20]


def frame_from_points(points):
    """Recover the model->bed transform from the toolpath bounding box.

    The outer wall centreline is inset by half a line width from the true
    surface, so the path bbox is the model bbox shrunk by line_width on each
    axis. Solving both axes gives the inset, which is checked against the
    expected 0.42 mm line width.
    """
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    px0, px1 = min(xs), max(xs)
    py0, py1 = min(ys), max(ys)
    inset_x = (BLOCK_X_MM - (px1 - px0)) / 2.0
    inset_y = (BLOCK_Y_MM - (py1 - py0)) / 2.0
    # model x=0 maps to px0 - inset_x ; model y=-6 maps to py0 - inset_y
    return {
        "px0": px0,
        "px1": px1,
        "x0": px0 - inset_x,
        "y_mid": (py0 + py1) / 2.0,
        "inset_x": inset_x,
        "inset_y": inset_y,
        "path_w": px1 - px0,
        "path_h": py1 - py0,
    }


def analyse_layer(layer, frame, line_width=0.42):
    """Per-station behaviour of the toolpath at each neck.

    Two independent measurements, because either alone is ambiguous:

    * Y reach -- how far in from each face the path travels. This says whether
      a slit exists, but it is IDENTICAL for a 0.01 mm and a 0.42 mm slit, so
      it cannot confirm the hairline survived as a real two-flank path.
    * X extent -- the chordwise width of the detour, which should equal
      `slit_width + line_width` (the two flank beads, each inset half a line
      width into the material). This is what distinguishes the two slit widths.

    `neck_gap_mm` is the distance between the two toolpath CENTRELINES across
    the ligament, NOT between bead edges. Beads are `line_width` wide, so the
    material/air relation is `air = neck_gap - line_width`: negative means the
    two beads interpenetrate.
    """
    out = []
    half = BLOCK_Y_MM / 2.0
    wall = half - frame["inset_y"]
    for (xc, g, w) in STATIONS:
        bed_xc = frame["x0"] + xc
        window = max(g, line_width) / 2.0 + 0.45
        near = [p for p in layer.points if abs(p[0] - bed_xc) <= window]
        rec = {
            "x_mm": xc,
            "slit_width_mm": g,
            "ligament_mm": w,
            "ligament_x_line_width": round(w / line_width, 4),
            "points_near": len(near),
            # The raw points, so the primary evidence survives in the JSON even
            # though the .gcode itself is gitignored project-wide.
            "points": [[round(px - frame["x0"], 4), round(py - frame["y_mid"], 4)]
                       for (px, py) in sorted(near)],
        }
        if not near:
            rec.update(upper_reach_mm=None, lower_reach_mm=None, x_extent_mm=None,
                       slit_present=False, neck_gap_mm=None, bead_air_mm=None,
                       note="no path vertices here: the wall runs straight "
                            "past, so the slit was erased before slicing")
            out.append(rec)
            continue

        ys = [p[1] - frame["y_mid"] for p in near]
        xs = [p[0] - frame["x0"] for p in near]

        # NOTE: >= / <= not > / <. A ligament of exactly 1.0x line_width puts
        # BOTH U-turns at y == 0.0, and strict comparisons would drop those
        # points into neither set, making a fully-collapsed neck look like an
        # absent slit.
        upper = [v for v in ys if v >= 0]
        lower = [v for v in ys if v <= 0]
        up_min = min(upper) if upper else wall
        lo_min = min(abs(v) for v in lower) if lower else wall
        upper_reach = wall - up_min
        lower_reach = wall - lo_min

        interior = [x for (x, y) in zip(xs, ys) if abs(y) < wall - 0.15]
        x_extent = round(max(interior) - min(interior), 4) if len(interior) > 1 else 0.0
        neck_gap = up_min + lo_min

        rec.update(
            upper_reach_mm=round(upper_reach, 4),
            lower_reach_mm=round(lower_reach, 4),
            x_extent_mm=x_extent,
            x_extent_expected_mm=round(g + line_width, 4),
            slit_present=bool(upper_reach > 0.15 or lower_reach > 0.15),
            neck_gap_mm=round(neck_gap, 4),
            # Positive = air between the two beads. Zero = exactly tangent.
            # Negative = the beads interpenetrate (over-extrusion at the neck).
            bead_air_mm=round(neck_gap - line_width, 4),
        )
        out.append(rec)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("gcode")
    ap.add_argument("--json")
    ap.add_argument("--root", type=int, default=5)
    ap.add_argument("--x0", type=float,
                    help="Pin the bed X of model x=0 instead of deriving it from the "
                         "footprint. Needed when a cut has SEVERED part of the coupon, "
                         "which shrinks the footprint and makes the derived frame wrong.")
    a = ap.parse_args(argv)

    layers = parse_layers(a.gcode)
    body = body_layers(layers, a.root)
    if not body:
        print("NO BODY LAYERS PARSED - refusing to report numbers", file=sys.stderr)
        return 2

    allpts = [p for l in body for p in l.points]
    frame = frame_from_points(allpts)
    severed_mm = 0.0
    if a.x0 is not None:
        # A severed coupon has a short footprint, so the derived origin is wrong.
        # Pinning it lets the stations still be indexed, and the shortfall is
        # itself the finding worth reporting.
        severed_mm = round(BLOCK_X_MM - 2 * 0.21 - frame["path_w"], 4)
        frame["x0"] = a.x0
        frame["inset_x"] = 0.21
        frame["severed_mm"] = severed_mm
        # Which END was lost: compare the observed path limits against where an
        # intact coupon's path would sit for this pinned origin.
        exp0, exp1 = a.x0 + 0.21, a.x0 + BLOCK_X_MM - 0.21
        frame["lost_low_end_mm"] = round(frame["px0"] - exp0, 4)
        frame["lost_high_end_mm"] = round(exp1 - frame["px1"], 4)

    # --- self-check, because a mis-anchored parse silently produces wrong
    # numbers. If the recovered footprint is not the coupon, every
    # station index below would be meaningless.
    problems = []
    if a.x0 is not None and severed_mm > 0.01:
        print(f"*** SEVERED: footprint is {severed_mm:.4f} mm short of the model. "
              f"Material was silently lost. ***")
        print(f"    lost at low-x end: {frame['lost_low_end_mm']:.4f} mm, "
              f"high-x end: {frame['lost_high_end_mm']:.4f} mm "
              f"(path x {frame['px0']:.3f}..{frame['px1']:.3f})")
    if a.x0 is None and not (0.15 < frame["inset_x"] < 0.35):
        problems.append(f"x inset {frame['inset_x']:.4f} mm is not ~half a 0.42 line width")
    if not (0.15 < frame["inset_y"] < 0.35):
        problems.append(f"y inset {frame['inset_y']:.4f} mm is not ~half a 0.42 line width")
    if a.x0 is None and abs(frame["path_w"] - (BLOCK_X_MM - 2 * frame["inset_x"])) > 1e-6:
        problems.append("width inconsistent")

    mid = body[len(body) // 2]
    print(f"file            {a.gcode}")
    print(f"layers parsed   {len(layers)}  (body {len(body)})")
    print(f"path footprint  {frame['path_w']:.4f} x {frame['path_h']:.4f} mm"
          f"   inset x {frame['inset_x']:.4f}  y {frame['inset_y']:.4f}")
    if problems:
        print("SELF-CHECK FAILED:")
        for p in problems:
            print("  -", p)
        print("Refusing to report per-station numbers.")
        return 2
    print("self-check      OK (footprint matches the coupon inset by ~line_width/2)")

    travels = sum(l.travels for l in body)
    print(f"body travels    {travels}   (Spiral Vase should emit 0)")
    print(f"mid layer       index {mid.index}  z={mid.z}  extrusions={mid.extrusions}")
    print()
    print(f"{'x':>5} {'slit':>6} {'ligament':>9} {'xW':>6} "
          f"{'y_reach':>8} {'x_ext':>7} {'x_exp':>7} {'slit?':>6} "
          f"{'gap_cl':>7} {'bead_air':>9}")
    rows = analyse_layer(mid, frame)
    for r in rows:
        xw = r["ligament_mm"] / 0.42
        print(f"{r['x_mm']:>5.0f} {r['slit_width_mm']:>6.2f} {r['ligament_mm']:>9.2f} "
              f"{xw:>5.2f}x "
              f"{_f(r['upper_reach_mm']):>8} {_f(r.get('x_extent_mm')):>7} "
              f"{_f(r.get('x_extent_expected_mm')):>7} "
              f"{str(r['slit_present']):>6} {_f(r['neck_gap_mm']):>7} "
              f"{_f(r.get('bead_air_mm')):>9}")

    if a.json:
        Path(a.json).write_text(json.dumps({
            "file": a.gcode,
            "layers": len(layers),
            "body_layers": len(body),
            "body_travels": travels,
            "frame": frame,
            "severed_mm": severed_mm,
            "mid_layer": {"index": mid.index, "z": mid.z,
                          "extrusions": mid.extrusions},
            "stations": rows,
        }, indent=2), encoding="ascii")
        print(f"\nwrote {a.json}")
    return 0


def _f(v):
    return "-" if v is None else f"{v:.4f}"


if __name__ == "__main__":
    raise SystemExit(main())
