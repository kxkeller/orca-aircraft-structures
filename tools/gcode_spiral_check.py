"""Check an exported G-code file against the one-continuous-loop objective.

The acceptance criteria include "no unintended body-layer travel/retraction": the
perimeter must run around the skin, in along one slot wall, around the end cap,
back out the other wall, and continue - as ONE circuit per body layer. A travel
move inside the body loop means the slicer treated the web as separate
geometry, which is the failure this whole architecture exists to avoid.

This reads the file only; it never writes and never touches a printer.

Usage:
    python tools/gcode_spiral_check.py <file.gcode> [--json out.json] [--root 5]

What it reports, per layer and in summary:
    extrusions      G1 moves with a positive E delta
    travels         G0/G1 moves with XY movement and no extrusion
    retractions     negative E delta (E-only moves, or explicit G10)
    z_hops          Z increases on a travel move
    xy_extent       min/max X and Y of extruding moves, for the slot's footprint

Interpretation, for the coupon:
    - Body layers should show 0 travels and 0 retractions. Spiral Vase emits one
      continuous circuit, so any travel is a real finding.
    - The first `--root` layers are the solid base and legitimately contain
      travels; they are reported separately and excluded from the verdict.
    - Layer count should match the model height / layer height.

--expect-structure (structure-presence check, practical half)
    Continuity alone cannot catch a plain-vase export: a cancelled or refused
    plugin hook followed by a non-invalidating setting change makes Orca
    export an ordinary Spiral Vase perimeter, which is ALSO one continuous
    circuit with zero travels and retractions -- it passes the check above
    trivially. This mode is a second, independent discriminator: per body
    layer, how far does the toolpath detour INWARD from that layer's own
    convex hull? A plain vase traces (approximately) its hull, so every
    point reads ~0. A structured print's toolpath dives into slot walls and
    webs, so a large fraction of points read several mm inward.

    Calibrated against kai_wing_20260913.gcode (a known-structured print) and a
    synthetic negative control built from that same file by replacing each
    layer's path with a traversal of that layer's own convex hull -- see
    tests/unit/test_gcode_structure_check.py for the measured separation and
    the generator. This proves structure is PRESENT in the G-code. It does not
    prove the structure is correct, bonded, or matches the intended config --
    that is a separate, physical question.
"""

import argparse
import json
import re
import sys
from pathlib import Path

# Orca emits ";LAYER_CHANGE" followed by ";Z:<height>". Both are matched so a
# file that only carries one of them still parses.
LAYER_CHANGE = re.compile(r"^;LAYER_CHANGE\b")
# End-of-print custom G-code (wipe, filament end, machine end block) legitimately
# retracts and travels. Counting it as a broken body loop would mislabel every
# successful spiral print as a failure, so move statistics stop being collected
# once this is seen.
#
# ";TYPE:Custom" is not unique to the true end of print: Orca's start G-code
# also emits it (in the purge/wipe block, before the first ";LAYER_CHANGE").
# Treating that occurrence as the end of print would discard every later move
# while ;LAYER_CHANGE markers kept creating empty layer records, producing a
# confident "PASS, 0 travels, 0 retractions" over data that was never
# analyzed. parse() therefore honours this match only after at least one
# ;LAYER_CHANGE has been seen, so a start-gcode custom block cannot trigger it.
END_OF_PRINT = re.compile(r"^;\s*(filament end gcode|TYPE:Custom)", re.IGNORECASE)
Z_COMMENT = re.compile(r"^;Z:\s*([0-9.]+)")
WORD = re.compile(r"([XYZEF])(-?\d*\.?\d+)")


class LayerStats:
    __slots__ = (
        "index", "z", "extrusions", "travels", "retractions", "z_hops",
        "min_x", "max_x", "min_y", "max_y", "extruded_mm", "points",
    )

    def __init__(self, index, z, capture_points=False):
        self.index = index
        self.z = z
        self.extrusions = 0
        self.travels = 0
        self.retractions = 0
        self.z_hops = 0
        self.min_x = self.min_y = float("inf")
        self.max_x = self.max_y = float("-inf")
        self.extruded_mm = 0.0
        # Ordered (x, y) of every extruding move, in toolpath order. Only
        # populated when the caller asks (--expect-structure); left as None
        # otherwise so the default continuity check's memory footprint is
        # unchanged.
        self.points = [] if capture_points else None

    def note_point(self, x, y):
        if x is not None:
            self.min_x = min(self.min_x, x)
            self.max_x = max(self.max_x, x)
        if y is not None:
            self.min_y = min(self.min_y, y)
            self.max_y = max(self.max_y, y)

    def to_dict(self):
        finite = self.min_x != float("inf")
        return {
            "layer": self.index,
            "z": round(self.z, 3) if self.z is not None else None,
            "extrusions": self.extrusions,
            "travels": self.travels,
            "retractions": self.retractions,
            "z_hops": self.z_hops,
            "extruded_mm": round(self.extruded_mm, 3),
            "x_mm": [round(self.min_x, 3), round(self.max_x, 3)] if finite else None,
            "y_mm": [round(self.min_y, 3), round(self.max_y, 3)] if finite else None,
        }


def parse(path, capture_points=False):
    """Parse the file into per-layer stats. Handles absolute and relative E.

    capture_points=True additionally records each layer's ordered extruding
    (x, y) points (LayerStats.points), for --expect-structure. Default is
    False so the plain continuity check's behaviour and memory use are
    unchanged.
    """
    layers = []
    current = None
    end_of_print = False
    x = y = z = None
    e = 0.0
    relative_e = False
    absolute_xyz = True

    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for raw in handle:
            line = raw.strip()
            if not line:
                continue

            if line.startswith(";"):
                # Gated on having seen at least one ;LAYER_CHANGE: Orca's own
                # start G-code emits ";TYPE:Custom" too (purge/wipe block,
                # before the first layer), and without this gate that would
                # set end_of_print True before any body move is parsed --
                # silently discarding every subsequent move in the file. See
                # the END_OF_PRINT comment above.
                if layers and END_OF_PRINT.match(line):
                    end_of_print = True
                    continue
                if LAYER_CHANGE.match(line):
                    current = LayerStats(len(layers), None, capture_points=capture_points)
                    layers.append(current)
                else:
                    match = Z_COMMENT.match(line)
                    if match and current is not None and current.z is None:
                        current.z = float(match.group(1))
                continue

            if end_of_print:
                continue

            code = line.split(None, 1)[0].upper()

            # Extrusion and positioning modes must be tracked, not assumed:
            # a negative E delta means nothing if E is relative and the move is
            # simply a normal extrusion.
            if code == "M83":
                relative_e = True
                continue
            if code == "M82":
                relative_e = False
                continue
            if code == "G91":
                absolute_xyz = False
                continue
            if code == "G90":
                absolute_xyz = True
                continue
            if code == "G92":
                for letter, value in WORD.findall(line.upper()):
                    if letter == "E":
                        e = float(value)
                continue
            if code == "G10":
                if current is not None:
                    current.retractions += 1
                continue
            if code not in ("G0", "G1"):
                continue

            words = dict(
                (letter, float(value)) for letter, value in WORD.findall(line.upper())
            )
            new_x = words.get("X")
            new_y = words.get("Y")
            new_z = words.get("Z")
            e_word = words.get("E")

            if absolute_xyz:
                target_x = new_x if new_x is not None else x
                target_y = new_y if new_y is not None else y
                target_z = new_z if new_z is not None else z
            else:
                target_x = (x or 0.0) + (new_x or 0.0)
                target_y = (y or 0.0) + (new_y or 0.0)
                target_z = (z or 0.0) + (new_z or 0.0)

            moved_xy = (target_x != x and target_x is not None) or (
                target_y != y and target_y is not None
            )

            if e_word is None:
                delta_e = 0.0
            elif relative_e:
                delta_e = e_word
                e += e_word
            else:
                delta_e = e_word - e
                e = e_word

            if current is not None:
                if delta_e > 0 and moved_xy:
                    current.extrusions += 1
                    current.extruded_mm += delta_e
                    current.note_point(target_x, target_y)
                    if current.points is not None:
                        current.points.append((target_x, target_y))
                elif delta_e < 0:
                    current.retractions += 1
                elif moved_xy:
                    current.travels += 1
                    if target_z is not None and z is not None and target_z > z:
                        current.z_hops += 1

            x, y = target_x, target_y
            if target_z is not None:
                z = target_z

    return layers


# --- --expect-structure: hazard structure-presence check -------------
#
# Constants below are measured, not guessed.
# tests/unit/test_gcode_structure_check.py has the negative-control generator
# (hull_traversal_points) and the calibration assertions against the real
# print. Summary of the distributions, including which layer subset each
# min/median/max is taken over:
#
#   Real structured print (a 1000-body-layer wing export):
#     fraction of points >STRUCTURAL_POINT_MM inward, per STRUCTURED layer
#     (997 of 1000; the 3 excluded are the wing-tip solid-cap layers, which
#     read 0.0 -- see below):
#       min 0.220, median 0.243, max 0.462
#
#   Synthetic plain-vase negative control (same file, every body layer's
#   path replaced by a traversal of that layer's own convex hull -- so every
#   emitted point lies exactly on its own hull by construction), full file:
#     fraction of points >STRUCTURAL_POINT_MM inward, per layer:
#       min 0.0, median 0.0, max 0.0 (all 1000 layers, exactly 0 -- the
#       ~1e-13 figure sometimes quoted for this construction is float noise
#       in the raw per-POINT hull_inward_distances() output, not in this
#       rounded per-LAYER fraction)
#
#   Overlap (why whole-window coverage is required, not optional): the real
#   print's 3 wing-tip solid-cap layers (1002-1004, z=200.6-201.0) read
#   fraction 0.0 -- identical, per-layer, to every layer of the synthetic
#   plain-vase control. A per-layer-only check cannot tell a legitimate tip
#   cap from lost structure; MIN_STRUCTURED_COVERAGE below is what catches
#   the difference (a few fringe layers vs. a sustained loss).
#
# Separation is >4x between the real structured-layer minimum (0.220) and
# LAYER_STRUCTURED_FRACTION, and effectively infinite against the synthetic
# control's exact 0.0. LAYER_STRUCTURED_FRACTION is set inside that gap.
STRUCTURAL_POINT_MM = 3.0
LAYER_STRUCTURED_FRACTION = 0.05
MIN_POINTS_FOR_HULL = 4
# A long run of unstructured layers -- e.g. a mid-print plugin cancellation
# that leaves every layer above it a plain vase -- must fail even though it
# is technically a single "trailing" gap and even though some structured
# layers exist near the root. Coverage over the whole body window is the
# guard: the real print measured 997/1000 = 0.997; a print that lost
# structure at layer 500 of 1000 would measure ~0.50. 0.90 sits with wide
# margin below the real measurement and far above anything a genuine
# fringe effect (root transition, tip cap) should ever cost.
MIN_STRUCTURED_COVERAGE = 0.90
# Floor on how much of the body window was even analyzable
# (usable_layer_count / body_layer_count). Below this, too few layers
# survived hull_inward_distances() (too-few-points / degenerate hull) for
# structured_coverage to mean anything -- e.g. a truncated export or a parse
# gone wrong could leave a handful of usable, genuinely-structured layers
# computing a "PASS" coverage over almost no data. Neither the real print nor
# the synthetic control has ANY inconclusive layer (both measure 1000/1000 =
# 1.000 usable), so there is no measured
# failure-mode distribution to sit this against; 0.5 is a deliberately
# conservative floor chosen far below both baselines, not a calibrated
# boundary. This floor does NOT catch a truncated export that simply has
# FEWER body layers (at most the one cut-off layer goes degenerate) --
# layer count vs. expected model height remains a separate reader check.
MIN_USABLE_FRACTION = 0.5


def hull_inward_distances(points):
    """Per-point inward distance from the convex hull of `points` (mm).

    points: sequence of (x, y) in the same units as the G-code (mm).
    Returns a numpy array of length len(points); ~0 for points on the hull
    boundary, positive and growing with how far a point sits inside the
    hull. Raises ValueError if there are too few points or they are
    degenerate (collinear / zero area) -- the caller must treat that layer
    as inconclusive, not as a failure.

    Imports numpy/scipy lazily so the default continuity check (no
    --expect-structure) has no scipy/numpy dependency.
    """
    import numpy as np
    from scipy.spatial import ConvexHull, QhullError

    if len(points) < MIN_POINTS_FOR_HULL:
        raise ValueError("too few points ({}) for a hull".format(len(points)))
    arr = np.asarray(points, dtype=float)
    try:
        hull = ConvexHull(arr)
    except QhullError as exc:
        raise ValueError("degenerate hull: {}".format(exc))
    # hull.equations rows are [a, b, c] with a*x + b*y + c <= 0 for points
    # inside; the signed distance to that facet's line is a*x+b*y+c. The
    # inward distance from the boundary is the negative of the LEAST
    # negative (i.e. closest, max) facet value.
    facet_values = arr @ hull.equations[:, :2].T + hull.equations[:, 2]
    inward = -np.max(facet_values, axis=1)
    return inward


def analyze_structure(layers, root, drop_final):
    """Per-body-layer structure-presence analysis for --expect-structure.

    Returns a dict with per-layer records plus a summary: structured-layer
    count, first/last structured layer, and any leading/trailing/interior
    gaps of unstructured layers within the body window.
    """
    body = [layer for layer in layers if layer.index >= root]
    final = body[-1] if body and drop_final else None
    if final is not None:
        body = body[:-1]

    records = []
    for layer in body:
        pts = layer.points or []
        n = len(pts)
        record = {
            "layer": layer.index,
            "z": round(layer.z, 3) if layer.z is not None else None,
            "points": n,
            "gt_structural_mm": None,
            "max_inward_mm": None,
            "structured": False,
            "inconclusive": False,
        }
        try:
            inward = hull_inward_distances(pts)
        except ValueError as exc:
            record["inconclusive"] = True
            record["reason"] = str(exc)
            records.append(record)
            continue
        gt_structural = int((inward > STRUCTURAL_POINT_MM).sum())
        record["gt_structural_mm"] = gt_structural
        record["max_inward_mm"] = round(float(inward.max()), 3)
        record["fraction_structural"] = round(gt_structural / n, 3) if n else 0.0
        record["structured"] = (n > 0) and (
            (gt_structural / n) >= LAYER_STRUCTURED_FRACTION
        )
        records.append(record)

    # Inconclusive layers (too few points / degenerate hull) are evidence of
    # neither structure nor its absence, so they are dropped before judging
    # coverage or runs rather than silently counting as "unstructured" (that
    # would both understate coverage and fabricate gaps that were never
    # measured).
    usable = [r for r in records if not r["inconclusive"]]
    structured_indices = [r["layer"] for r in usable if r["structured"]]
    coverage = (len(structured_indices) / len(usable)) if usable else 0.0
    summary = {
        "root_layers": root,
        "final_excluded": final.index if final is not None else None,
        "body_layer_count": len(records),
        "usable_layer_count": len(usable),
        "structured_layer_count": len(structured_indices),
        "structured_coverage": round(coverage, 3),
        "first_structured_layer": min(structured_indices) if structured_indices else None,
        "last_structured_layer": max(structured_indices) if structured_indices else None,
    }

    # Gaps: runs of consecutive non-structured USABLE layers, classified as
    # leading (before the first structured layer), trailing (after the
    # last), or interior (structured on both sides). All are reported --
    # a trailing gap at a wingtip solid cap is expected, but this check
    # does not know that; it reports what it measured (including its
    # length as a share of the body) and leaves the physical judgment to
    # the reader. A long trailing/leading gap still fails the overall
    # verdict via structured_coverage below -- gap "kind" alone must not
    # be read as a safety classification.
    gaps = []
    if structured_indices:
        first_idx = summary["first_structured_layer"]
        last_idx = summary["last_structured_layer"]
        run_start = None
        prev_layer = None
        for r in usable:
            unstructured = not r["structured"]
            if unstructured and run_start is None:
                run_start = r["layer"]
            if not unstructured and run_start is not None:
                gaps.append((run_start, prev_layer))
                run_start = None
            prev_layer = r["layer"]
        if run_start is not None:
            gaps.append((run_start, usable[-1]["layer"]))
        for start, end in gaps:
            if end < first_idx:
                kind = "leading"
            elif start > last_idx:
                kind = "trailing"
            else:
                kind = "interior"
            summary.setdefault("gaps", []).append(
                {"kind": kind, "first_layer": start, "last_layer": end,
                 "length": end - start + 1}
            )
    summary.setdefault("gaps", [])

    return {"summary": summary, "layers": records}


def structure_verdict(summary):
    """PASS / WARN / FAIL / INCONCLUSIVE for a --expect-structure summary
    dict (pure, no I/O -- kept separate from print_structure_report so the
    verdict logic itself is directly unit-testable).

    INCONCLUSIVE is checked FIRST, before everything else, including the
    "0 structured layers" FAIL case: if too few body layers were even
    analyzable (see MIN_USABLE_FRACTION), structured_coverage is computed
    over almost no data and must not be read as a confident FAIL (or,
    worse, a confident PASS if the handful that survived happen to be
    structured) -- that is exactly the "confident verdict over data it
    never analyzed" failure mode this check exists to avoid. A body window
    with zero layers at all (e.g. --root swallows everything) is the same
    hazard by construction and is also INCONCLUSIVE.

    Coverage is checked before gap "kind": a long leading/trailing run --
    e.g. structure lost at layer 500 of 1000 after a mid-print plugin
    cancellation -- must fail even though it is technically a single
    trailing gap next to some structured root layers. Gap kind alone is
    not a safety classification; see the module docstring for the
    measured real-print (0.997) vs synthetic-plain-vase (0.0) coverage
    this threshold (MIN_STRUCTURED_COVERAGE) sits between.
    """
    body_count = summary["body_layer_count"]
    usable_count = summary["usable_layer_count"]
    if body_count == 0 or (usable_count / body_count) < MIN_USABLE_FRACTION:
        return "INCONCLUSIVE"
    if summary["structured_layer_count"] == 0:
        return "FAIL"
    if summary["structured_coverage"] < MIN_STRUCTURED_COVERAGE:
        return "FAIL"
    if any(g["kind"] == "interior" for g in summary["gaps"]):
        return "WARN"
    return "PASS"


def print_structure_report(analysis):
    summary = analysis["summary"]
    print("\n--- --expect-structure: structure-presence check ---")
    print("body layers analyzed: {} ({} usable, {} inconclusive)".format(
        summary["body_layer_count"], summary["usable_layer_count"],
        summary["body_layer_count"] - summary["usable_layer_count"]))
    print("structured layers:    {} / {} usable ({:.1%} coverage)".format(
        summary["structured_layer_count"], summary["usable_layer_count"],
        summary["structured_coverage"]))
    if summary["first_structured_layer"] is not None:
        print("first structured layer: {}".format(summary["first_structured_layer"]))
        print("last structured layer:  {}".format(summary["last_structured_layer"]))
    else:
        print("first/last structured layer: none")

    usable = max(summary["usable_layer_count"], 1)
    if summary["gaps"]:
        print("\nUNSTRUCTURED RUNS (candidate plain-vase regions):")
        for gap in summary["gaps"]:
            print("  {:<9} layers {}-{} ({} layer(s), {:.1%} of usable body)".format(
                gap["kind"] + ":", gap["first_layer"], gap["last_layer"], gap["length"],
                gap["length"] / usable))
    else:
        print("no unstructured runs within the body window.")

    inconclusive = [l for l in analysis["layers"] if l["inconclusive"]]
    if inconclusive:
        print("\n{} layer(s) inconclusive (too few points / degenerate hull) -- "
              "excluded from coverage and gap analysis, not counted as either "
              "pass or fail:".format(len(inconclusive)))
        for l in inconclusive[:6]:
            print("  layer {:>4} z={}: {}".format(l["layer"], l["z"], l["reason"]))

    verdict = structure_verdict(summary)
    if verdict == "INCONCLUSIVE":
        usable_fraction = (
            (summary["usable_layer_count"] / summary["body_layer_count"])
            if summary["body_layer_count"] else 0.0
        )
        print(
            "\nVERDICT: INCONCLUSIVE - only {} / {} body layers ({:.1%}) were "
            "analyzable (too few points / degenerate hull on the rest), below "
            "the {:.0%} floor. structured_coverage would be computed over "
            "almost no data, so it is not reported as PASS or FAIL -- this "
            "usually means a truncated export or a parse gone wrong upstream "
            "of this check, not a plain-vase print."
            .format(
                summary["usable_layer_count"], summary["body_layer_count"],
                usable_fraction, MIN_USABLE_FRACTION))
    elif verdict == "FAIL" and summary["structured_layer_count"] == 0:
        print(
            "\nVERDICT: FAIL - no body layer shows inward toolpath excursion "
            "above the structural threshold ({} mm on >= {:.0%} of its points). "
            "This G-code is indistinguishable from a plain-vase perimeter."
            .format(STRUCTURAL_POINT_MM, LAYER_STRUCTURED_FRACTION))
    elif verdict == "FAIL":
        print(
            "\nVERDICT: FAIL - only {:.1%} of usable body layers show "
            "structural inward excursion (threshold {:.0%}). A large share "
            "of this print reads as plain-vase, even though some layers are "
            "structured -- see the unstructured run(s) above for where."
            .format(summary["structured_coverage"], MIN_STRUCTURED_COVERAGE))
    elif verdict == "WARN":
        print(
            "\nVERDICT: WARN - structure is present and coverage is high, "
            "but not continuous; see the interior unstructured run(s) "
            "above. Structure may have been lost and then recovered "
            "partway through the print.")
    else:
        print(
            "\nVERDICT: PASS - inward structural excursion is present across "
            "the body window (leading/trailing gaps, if any, are only at the "
            "root or tip fringe).")

    print(
        "\nBOUNDS: this proves structure is PRESENT in the G-code toolpath. "
        "It does NOT prove the structure is correct, bonded, or matches the "
        "intended config -- that requires the physical/print-side checks."
    )
    return verdict


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gcode", type=Path)
    parser.add_argument("--root", type=int, default=5,
                        help="solid root layers to exclude from the verdict")
    parser.add_argument("--json", type=Path, default=None)
    parser.add_argument("--show", type=int, default=6,
                        help="how many offending layers to list")
    parser.add_argument("--include-last", action="store_true",
                        help="include the final layer in the verdict; by default "
                             "it is excluded because it carries the print-ending "
                             "retract, wipe and Z-lift")
    parser.add_argument("--expect-structure", action="store_true",
                        help="also run the structure-presence check: "
                             "per body layer, how far the toolpath detours inward "
                             "from that layer's own convex hull. Opt-in; does not "
                             "change the continuity check above.")
    args = parser.parse_args(argv)

    if not args.gcode.is_file():
        print("not found: {}".format(args.gcode), file=sys.stderr)
        return 2

    layers = parse(args.gcode, capture_points=args.expect_structure)
    if not layers:
        print("no ;LAYER_CHANGE markers found - is this an Orca G-code file?",
              file=sys.stderr)
        return 2

    body = [layer for layer in layers if layer.index >= args.root]
    # The last layer always ends the print: retract, wipe, lift. Those are not
    # a broken body loop, so they are reported separately rather than failing
    # every successful spiral print.
    final = body[-1] if body and not args.include_last else None
    if final is not None:
        body = body[:-1]
    dirty = [layer for layer in body if layer.travels or layer.retractions]

    print("file:          {}".format(args.gcode))
    print("layers:        {} ({} body, {} root excluded)".format(
        len(layers), len(body), args.root))
    print("body travels:      {}".format(sum(l.travels for l in body)))
    print("body retractions:  {}".format(sum(l.retractions for l in body)))
    print("body z-hops:       {}".format(sum(l.z_hops for l in body)))

    if body:
        widths = [
            (l.max_x - l.min_x, l.max_y - l.min_y)
            for l in body if l.min_x != float("inf")
        ]
        if widths:
            print("body xy extent:    {:.2f} x {:.2f} mm (first body layer)".format(
                *widths[0]))

    if dirty:
        print("\nLAYERS WITH TRAVEL OR RETRACTION INSIDE THE BODY:")
        for layer in dirty[: args.show]:
            print("  layer {:>4} z={:<7} travels={:<4} retractions={}".format(
                layer.index,
                layer.z if layer.z is not None else "?",
                layer.travels,
                layer.retractions))
        if len(dirty) > args.show:
            print("  ... and {} more".format(len(dirty) - args.show))
        print("\nVERDICT: FAIL - the body loop is not continuous.")
    else:
        print("\nVERDICT: PASS - every body layer is one continuous circuit "
              "with no travel or retraction.")

    if args.json:
        args.json.write_text(
            json.dumps(
                {
                    "file": str(args.gcode),
                    "layer_count": len(layers),
                    "root_layers": args.root,
                    "body_travels": sum(l.travels for l in body),
                    "body_retractions": sum(l.retractions for l in body),
                    "body_z_hops": sum(l.z_hops for l in body),
                    "dirty_layers": [l.index for l in dirty],
                    "layers": [l.to_dict() for l in layers],
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        print("json: {}".format(args.json))

    continuity_ok = not dirty
    structure_ok = True
    structure_analysis = None
    if args.expect_structure:
        structure_analysis = analyze_structure(
            layers, args.root, drop_final=not args.include_last
        )
        verdict = print_structure_report(structure_analysis)
        # Both FAIL and WARN return nonzero: an interior gap is the hazard
        # occurring mid-print (structure lost then recovered), not a cosmetic
        # issue, and this is an offline pre-flight -- silence on a WARN would
        # let it through unnoticed.
        structure_ok = verdict == "PASS"
        if args.json:
            data = json.loads(args.json.read_text(encoding="utf-8"))
            data["expect_structure"] = structure_analysis
            args.json.write_text(json.dumps(data, indent=2), encoding="utf-8")

    return 0 if (continuity_ok and structure_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
