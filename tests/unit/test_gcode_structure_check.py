"""Tests for tools/gcode_spiral_check.py's --expect-structure mode.

Hazard: a cancelled/refused plugin hook
followed by a non-invalidating setting change makes Orca export a plain Spiral
Vase perimeter -- status 0, one continuous circuit, zero travels/retractions --
which the pre-existing continuity check in this module cannot distinguish from
a genuinely structured slice. --expect-structure adds a second, independent
discriminator: per body layer, how far the toolpath detours INWARD from that
layer's own convex hull. A plain vase traces (approximately) its own hull, so
every point reads ~0 inward. A structured print's toolpath dives into slot
walls/webs, so a large fraction of points read several mm inward.

Three tiers of coverage:
  1. Unit tests of hull_inward_distances() and analyze_structure() against
     small, hand-built point sets (fast, no I/O).
  2. CLI round-trip tests against small synthetic G-code files built with
     write_gcode() below (fast, temp files).
  3. Calibration against the real known-structured print
     (~1006 layers, ~12 MB) and a negative control built from a sample of
     that same file's layers by hull_traversal_points() -- replacing each
     sampled layer's path with a traversal of ITS OWN convex hull, so every
     emitted point lies exactly on that layer's hull by construction. This
     tier is skipped if the (large, tracked) fixture file is missing.

Measured separation (full 1000-body-layer run; see
tools/gcode_spiral_check.py's constant block for the same numbers and
the full method):
  real structured print:   fraction of points >3mm inward, per STRUCTURED
                            layer (997 of 1000; the 3 excluded are the
                            wing-tip solid-cap layers, which read exactly
                            0.0 -- identical to every synthetic-control
                            layer below, which is WHY whole-window coverage
                            is required, not just a per-layer threshold):
                            min 0.220, median 0.243, max 0.462
  synthetic hull-traversal negative control (same file, same method as
  hull_traversal_points() below, run over all 1000 layers):
                            min 0.0, median 0.0, max 0.0 (exactly, for the
                            rounded per-layer fraction; the raw per-point
                            hull_inward_distances() output has ~1e-13 float
                            noise, but it never clears STRUCTURAL_POINT_MM)
LAYER_STRUCTURED_FRACTION (0.05) sits inside that gap.

This module also carries TestEndOfPrintMarker, regression tests for the
END_OF_PRINT marker fix (tools/gcode_spiral_check.py) even though that bug
is unrelated to structure detection: both share the property that this tool
must never report a confident PASS over G-code it did not actually analyze
-- structure_verdict()'s INCONCLUSIVE floor (see TestAnalyzeStructure) is
that same property applied to too-few-usable-layers, END_OF_PRINT's
layers-seen gate is that same property applied to a start-gcode custom
block that would otherwise silently zero out the rest of the file's move
stats while still reporting a clean "PASS, 0 travels, 0 retractions".
"""
import io
import math
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import gcode_spiral_check as gsc  # noqa: E402

try:
    import numpy as np  # noqa: F401
    from scipy.spatial import ConvexHull
    HAVE_SCIPY = True
except ImportError:  # pragma: no cover - scipy is a project dependency elsewhere
    HAVE_SCIPY = False

REAL_GCODE = (
    REPO_ROOT / "docs" / "results" / "print_20260913_kai_wing_petg"
    / "kai_wing_20260913.gcode"
)


# --- shared fixtures --------------------------------------------------------

def hull_traversal_points(points):
    """The synthetic plain-vase negative control's core transform.

    Replace an ordered list of extruding (x, y) points with a traversal of
    THEIR OWN convex hull (scipy ConvexHull, CCW vertex order), closing the
    loop back to the first vertex. Every returned point lies exactly on the
    hull of the input by construction, so a working structure-presence check
    must read ~0 inward excursion for all of them. This is how both the
    small in-file negative controls and the full 1000-layer synthetic
    plain-vase file used for timing/calibration were built.
    """
    arr = np.asarray(points, dtype=float)
    hull = ConvexHull(arr)
    verts = arr[hull.vertices]
    loop = [tuple(p) for p in verts]
    loop.append(loop[0])
    return loop


def slotted_layer_points(total=100, deep_fraction=0.3, radius=10.0, depth=6.0):
    """A roughly circular perimeter with a cluster of points pulled inward
    by `depth`, standing in for a slot/web wall detour. Not a realistic
    airfoil section -- just enough geometry to exercise analyze_structure's
    per-point/per-layer classification deterministically.
    """
    n_deep = int(total * deep_fraction)
    n_hull = total - n_deep
    pts = []
    for i in range(n_hull):
        theta = 2 * math.pi * i / n_hull
        pts.append((radius * math.cos(theta), radius * math.sin(theta)))
    r_deep = radius - depth
    for i in range(n_deep):
        theta = 0.03 * i  # cluster near angle 0, inside the hull
        pts.append((r_deep * math.cos(theta), r_deep * math.sin(theta)))
    return pts


def write_gcode(path, layers):
    """layers: list of (z, [(x, y)...]) -> a minimal but valid Orca-style G-code
    file (G90 absolute XYZ, relative E) that gsc.parse() accepts.

    Every move -- including each layer's first point -- carries a positive E,
    so the file reads as one continuous extruding circuit (Spiral Vase-like:
    zero travels/retractions in the body) regardless of which points/shape
    layers_fn supplies. That isolates --expect-structure's geometric
    discriminator in CLI round-trip tests from the pre-existing continuity
    check, which is exercised separately in tests/unit for the unmodified
    default mode.
    """
    # No end-of-print retract/wipe block is appended: these fixtures don't
    # need one, since the default final-layer exclusion already drops it
    # before the verdict. END_OF_PRINT's own repair (the stray backspace
    # byte, and gating on "at least one layer seen") is covered separately
    # in TestEndOfPrintMarker below.
    lines = ["G90", "M83", "G92 E0"]
    prev_z = None
    for z, pts in layers:
        lines.append(";LAYER_CHANGE")
        lines.append(";Z:{}".format(z))
        if not pts:
            continue
        if z != prev_z:
            lines.append("G1 Z{:.3f} F1200".format(z))
            prev_z = z
        for point in pts:
            lines.append("G1 X{:.3f} Y{:.3f} E0.02000 F1200".format(*point))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def make_layer(index, z, points):
    layer = gsc.LayerStats(index, z, capture_points=True)
    layer.points = points
    return layer


# --- tier 1: hull_inward_distances / analyze_structure ----------------------

@unittest.skipUnless(HAVE_SCIPY, "scipy is required for --expect-structure")
class TestHullInwardDistances(unittest.TestCase):
    def test_square_center_point_reads_its_own_margin(self):
        # 10x10 square, center point is exactly 5mm from every edge.
        pts = [(0, 0), (10, 0), (10, 10), (0, 10), (5, 5)]
        d = gsc.hull_inward_distances(pts)
        self.assertAlmostEqual(d[4], 5.0, places=6)
        for i in range(4):
            self.assertAlmostEqual(d[i], 0.0, places=6)

    def test_hull_traversal_control_reads_zero(self):
        pts = slotted_layer_points()
        control = hull_traversal_points(pts)
        d = gsc.hull_inward_distances(control)
        self.assertTrue(all(abs(v) < 1e-6 for v in d), "max={}".format(max(abs(v) for v in d)))

    def test_too_few_points_raises(self):
        with self.assertRaises(ValueError):
            gsc.hull_inward_distances([(0, 0), (1, 1)])

    def test_degenerate_collinear_points_raises(self):
        with self.assertRaises(ValueError):
            gsc.hull_inward_distances([(0, 0), (1, 0), (2, 0), (3, 0)])


@unittest.skipUnless(HAVE_SCIPY, "scipy is required for --expect-structure")
class TestAnalyzeStructure(unittest.TestCase):
    def test_structured_layers_all_pass(self):
        layers = [make_layer(i, float(i), slotted_layer_points()) for i in range(10)]
        analysis = gsc.analyze_structure(layers, root=0, drop_final=False)
        summary = analysis["summary"]
        self.assertEqual(summary["structured_layer_count"], 10)
        self.assertEqual(summary["first_structured_layer"], 0)
        self.assertEqual(summary["last_structured_layer"], 9)
        self.assertEqual(summary["gaps"], [])

    def test_vase_layers_all_fail(self):
        vase_pts = hull_traversal_points(slotted_layer_points())
        layers = [make_layer(i, float(i), vase_pts) for i in range(10)]
        analysis = gsc.analyze_structure(layers, root=0, drop_final=False)
        summary = analysis["summary"]
        self.assertEqual(summary["structured_layer_count"], 0)
        self.assertIsNone(summary["first_structured_layer"])
        self.assertEqual(summary["gaps"], [])  # no structured layer -> no gap window to report

    def test_interior_gap_is_visible_and_warns(self):
        # structure present at root, lost in the middle, present again -- this
        # is exactly the "root structured, outboard plain vase" partial failure
        # describes. Coverage stays high (6/8 = 0.75 is below
        # MIN_STRUCTURED_COVERAGE too, so use a longer structured run either
        # side to isolate the WARN path from the coverage FAIL path).
        structured = slotted_layer_points()
        vase = hull_traversal_points(structured)
        sequence = [structured] * 20 + [vase] * 2 + [structured] * 20
        layers = [make_layer(i, float(i), pts) for i, pts in enumerate(sequence)]
        analysis = gsc.analyze_structure(layers, root=0, drop_final=False)
        gaps = analysis["summary"]["gaps"]
        self.assertEqual(len(gaps), 1)
        self.assertEqual(gaps[0]["kind"], "interior")
        self.assertEqual(gaps[0]["first_layer"], 20)
        self.assertEqual(gaps[0]["last_layer"], 21)
        self.assertEqual(gaps[0]["length"], 2)
        self.assertGreaterEqual(analysis["summary"]["structured_coverage"], gsc.MIN_STRUCTURED_COVERAGE)
        self.assertEqual(gsc.structure_verdict(analysis["summary"]), "WARN")

    def test_small_trailing_gap_passes_tip_cap_case(self):
        # Pins the real-print shape: a short trailing run of unstructured
        # layers at the very end (wing tip solid cap) with high overall
        # coverage must still PASS, not just be labelled "trailing".
        structured = slotted_layer_points()
        vase = hull_traversal_points(structured)
        sequence = [structured] * 97 + [vase] * 3  # 97/100 = 0.97 coverage
        layers = [make_layer(i, float(i), pts) for i, pts in enumerate(sequence)]
        analysis = gsc.analyze_structure(layers, root=0, drop_final=False)
        gaps = analysis["summary"]["gaps"]
        self.assertEqual(len(gaps), 1)
        self.assertEqual(gaps[0]["kind"], "trailing")
        self.assertEqual(gsc.structure_verdict(analysis["summary"]), "PASS")

    def test_long_trailing_gap_fails_on_coverage_not_just_kind(self):
        # The exact hazard named in the task: a plugin cancellation partway
        # through the print leaves everything above that layer a plain
        # vase. That is technically one long "trailing" gap next to
        # structured root layers -- gap kind alone must not pass it.
        structured = slotted_layer_points()
        vase = hull_traversal_points(structured)
        sequence = [structured] * 100 + [vase] * 100  # lost structure at the midpoint
        layers = [make_layer(i, float(i), pts) for i, pts in enumerate(sequence)]
        analysis = gsc.analyze_structure(layers, root=0, drop_final=False)
        gaps = analysis["summary"]["gaps"]
        self.assertEqual(len(gaps), 1)
        self.assertEqual(gaps[0]["kind"], "trailing")
        self.assertEqual(gaps[0]["length"], 100)
        self.assertAlmostEqual(analysis["summary"]["structured_coverage"], 0.5, places=2)
        self.assertLess(analysis["summary"]["structured_coverage"], gsc.MIN_STRUCTURED_COVERAGE)
        self.assertEqual(gsc.structure_verdict(analysis["summary"]), "FAIL")

    def test_mostly_inconclusive_window_is_not_pass(self):
        # The hazard this test pins: a small number M of usable, structured
        # layers sitting among a large N of inconclusive (too-few-points /
        # degenerate-hull) layers must NOT compute a confident PASS over
        # the handful that happened to survive. Modelled on
        # test_long_trailing_gap_fails_on_coverage_not_just_kind above --
        # same "don't let a narrow technical pass hide a broad failure to
        # analyze" shape, applied to usable-fraction instead of coverage.
        structured = slotted_layer_points()
        degenerate = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]  # < MIN_POINTS_FOR_HULL
        sequence = [structured] * 5 + [degenerate] * 95  # 5 usable / 100 body
        layers = [make_layer(i, float(i), pts) for i, pts in enumerate(sequence)]
        analysis = gsc.analyze_structure(layers, root=0, drop_final=False)
        summary = analysis["summary"]
        self.assertEqual(summary["usable_layer_count"], 5)
        self.assertEqual(summary["body_layer_count"], 100)
        self.assertEqual(summary["structured_layer_count"], 5)
        # Coverage alone (5/5 usable = 1.0) would read as a confident PASS;
        # the usable-fraction floor must override that.
        self.assertEqual(summary["structured_coverage"], 1.0)
        self.assertNotEqual(gsc.structure_verdict(summary), "PASS")
        self.assertEqual(gsc.structure_verdict(summary), "INCONCLUSIVE")

    def test_empty_body_window_is_inconclusive_not_zero_division(self):
        analysis = gsc.analyze_structure([], root=0, drop_final=False)
        summary = analysis["summary"]
        self.assertEqual(summary["body_layer_count"], 0)
        self.assertEqual(gsc.structure_verdict(summary), "INCONCLUSIVE")

    def test_root_and_final_excluded_as_in_continuity_check(self):
        structured = slotted_layer_points()
        layers = [make_layer(i, float(i), structured) for i in range(8)]
        analysis = gsc.analyze_structure(layers, root=3, drop_final=True)
        summary = analysis["summary"]
        self.assertEqual(summary["root_layers"], 3)
        self.assertEqual(summary["body_layer_count"], 4)  # 8 - 3 root - 1 final
        self.assertEqual(summary["final_excluded"], 7)


# --- tier 2: CLI round trip on small synthetic files -------------------------

class TestExpectStructureCLI(unittest.TestCase):
    def _write(self, tmpdir, name, n_layers, points_fn):
        path = Path(tmpdir) / name
        layers = [(float(i), points_fn()) for i in range(n_layers)]
        write_gcode(path, layers)
        return path

    @unittest.skipUnless(HAVE_SCIPY, "scipy is required for --expect-structure")
    def test_structured_fixture_passes(self):
        # One extra trailing layer so the tool's default final-layer
        # exclusion has something to drop, same as real usage.
        with tempfile.TemporaryDirectory() as td:
            path = self._write(td, "structured.gcode", 9, slotted_layer_points)
            rc = gsc.main([str(path), "--root", "0", "--expect-structure"])
            self.assertEqual(rc, 0)

    @unittest.skipUnless(HAVE_SCIPY, "scipy is required for --expect-structure")
    def test_vase_fixture_fails(self):
        with tempfile.TemporaryDirectory() as td:
            path = self._write(
                td, "vase.gcode", 9,
                lambda: hull_traversal_points(slotted_layer_points()),
            )
            rc = gsc.main([str(path), "--root", "0", "--expect-structure"])
            self.assertEqual(rc, 1)

    @unittest.skipUnless(HAVE_SCIPY, "scipy is required for --expect-structure")
    def test_mid_print_structure_loss_fails_via_cli(self):
        # End-to-end version of the coverage regression test: structure
        # present for the first half of the print, gone for the second --
        # this is what a mid-print plugin cancellation followed by a
        # non-invalidating setting change looks like on disk.
        structured_pts = slotted_layer_points()
        vase_pts = hull_traversal_points(structured_pts)
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "mixed.gcode"
            layers = (
                [(float(i), structured_pts) for i in range(20)]
                + [(float(i), vase_pts) for i in range(20, 41)]  # extra layer for final-drop
            )
            write_gcode(path, layers)
            rc = gsc.main([str(path), "--root", "0", "--expect-structure"])
            self.assertEqual(rc, 1)

    def test_flag_is_opt_in_default_output_unchanged(self):
        # Without --expect-structure the report section must not appear at all,
        # and the continuity verdict/return code must match what the tool has
        # always produced for a clean structured file.
        with tempfile.TemporaryDirectory() as td:
            path = self._write(td, "structured.gcode", 9, slotted_layer_points)
            out = io.StringIO()
            with redirect_stdout(out):
                rc = gsc.main([str(path), "--root", "0"])
            text = out.getvalue()
            self.assertEqual(rc, 0)
            self.assertIn("VERDICT: PASS - every body layer is one continuous circuit", text)
            self.assertNotIn("structure-presence check", text)
            self.assertNotIn("--expect-structure", text)

    @unittest.skipUnless(HAVE_SCIPY, "scipy is required for --expect-structure")
    def test_mostly_degenerate_fixture_is_nonzero_exit_not_pass(self):
        # CLI-level version of test_mostly_inconclusive_window_is_not_pass:
        # a handful of structured layers among many too-few-point
        # (degenerate) layers must return nonzero, not 0 (PASS), proving
        # the INCONCLUSIVE verdict is actually wired through main()'s exit
        # code and not just correct in the pure structure_verdict() logic.
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "mostly_degenerate.gcode"
            degenerate_pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
            layers = (
                [(float(i), slotted_layer_points()) for i in range(5)]
                + [(float(i), degenerate_pts) for i in range(5, 96)]
            )
            write_gcode(path, layers)
            rc = gsc.main([str(path), "--root", "0", "--expect-structure"])
            self.assertEqual(rc, 1)

    def test_parse_without_capture_points_leaves_points_none(self):
        with tempfile.TemporaryDirectory() as td:
            path = self._write(td, "structured.gcode", 4, slotted_layer_points)
            layers = gsc.parse(path)
            self.assertTrue(all(layer.points is None for layer in layers))


# --- END_OF_PRINT marker regression ----------------------------------

class TestEndOfPrintMarker(unittest.TestCase):
    """Regression tests for the END_OF_PRINT marker.

    The marker pattern must not carry a stray literal 0x08 (backspace) byte,
    which would make it match nothing (inert only because of the default
    --root / final-layer exclusion). Simply removing the stray byte is itself
    a trap: Orca's start G-code
    also emits ";TYPE:Custom" (in the purge/wipe block, before the first
    ;LAYER_CHANGE), so a plain repair would set end_of_print True there and
    never reset it -- silently discarding every subsequent move in the file
    while ;LAYER_CHANGE markers kept creating (empty) layer records, i.e. a
    confident "PASS, 0 travels, 0 retractions" over data that was never
    analyzed. The fix repairs the pattern AND gates it on having seen at
    least one ;LAYER_CHANGE.
    """

    def test_start_gcode_custom_block_does_not_suppress_body_moves(self):
        # A ;TYPE:Custom block BEFORE the first ;LAYER_CHANGE (start gcode)
        # must not set end_of_print -- the naive-repair trap.
        lines = [
            "G90", "M83", "G92 E0",
            ";TYPE:Custom",
            "; some start gcode purge/wipe block",
            "G1 X0 Y0 E1 F1200",
        ]
        layers = [(0.0, slotted_layer_points()), (1.0, slotted_layer_points())]
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "start_custom.gcode"
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            first_layers = gsc.parse(path)
            self.assertEqual(len(first_layers), 0)  # no ;LAYER_CHANGE yet in this snippet

        # Full round trip: a normal structured file, but with the write_gcode
        # preamble replaced so a ;TYPE:Custom line appears before the first
        # ;LAYER_CHANGE, as it does in the real print (line 44 vs the first
        # ;LAYER_CHANGE at line 130).
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "structured_with_start_custom.gcode"
            write_gcode(path, layers)
            text = path.read_text(encoding="utf-8")
            text = text.replace("G90\nM83\nG92 E0\n",
                                 "G90\nM83\nG92 E0\n;TYPE:Custom\n; start purge\n", 1)
            path.write_text(text, encoding="utf-8")
            parsed = gsc.parse(path)
            self.assertEqual(len(parsed), 2)
            total_extrusions = sum(l.extrusions for l in parsed)
            self.assertGreater(total_extrusions, 0,
                                "start-gcode ;TYPE:Custom silently suppressed body moves")

    def test_end_of_print_block_excluded_from_last_layer_stats(self):
        # A ;TYPE:Custom block AFTER the last ;LAYER_CHANGE (the real
        # end-of-print shape) must be recognised and its retract/travel
        # moves excluded from that last layer's counted stats.
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "end_custom.gcode"
            lines = [
                "G90", "M83", "G92 E0",
                ";LAYER_CHANGE", ";Z:0.2",
                "G1 X0 Y0 E1 F1200",
                "G1 X1 Y1 E1 F1200",
                ";TYPE:Custom",
                "; filament end gcode",
                "G1 E-1 F1200",  # retraction
                "G1 X50 Y50 F3000",  # travel
            ]
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            layers = gsc.parse(path)
            self.assertEqual(len(layers), 1)
            self.assertEqual(layers[0].retractions, 0)
            self.assertEqual(layers[0].travels, 0)
            self.assertEqual(layers[0].extrusions, 2)

    def test_real_print_marker_fires_at_end_not_at_start_custom_block(self):
        if not REAL_GCODE.is_file():
            self.skipTest("reference print G-code not present")
        layers = gsc.parse(REAL_GCODE)
        # Real file: ";TYPE:Custom" appears at line 44 (start gcode, before
        # the first ;LAYER_CHANGE at line 130) and again at line 386496
        # (the true end of print, right before "; filament end gcode" at
        # 386497). Only the real end should have suppressed the last
        # layer's trailing retract/travel/park moves.
        last = layers[-1]
        self.assertEqual(last.retractions, 2)
        self.assertEqual(last.travels, 0)
        self.assertGreater(last.extrusions, 0)


# --- tier 3: calibration against the real print + a real-data negative control

@unittest.skipUnless(HAVE_SCIPY, "scipy is required for --expect-structure")
@unittest.skipUnless(REAL_GCODE.is_file(), "reference print G-code not present")
class TestRealPrintCalibration(unittest.TestCase):
    """Calibration against the known-structured print  used to derive
    STRUCTURAL_POINT_MM and LAYER_STRUCTURED_FRACTION. Regenerating the full
    1000-layer synthetic negative control here would be slow for a unit test; a
    sampled subset is enough to prove the check fires on unstructured input.
    The full-file distributions (both files, both subsets) are committed,
    regenerable via -- not re-run on every `python tools/run_tests.py`.
    """

    def test_real_print_reads_structured(self):
        layers = gsc.parse(REAL_GCODE, capture_points=True)
        analysis = gsc.analyze_structure(layers, root=5, drop_final=True)
        summary = analysis["summary"]
        # Measured: 997/1000 body layers structured; only the last 3 (the
        # wing-tip solid cap, z=200.6-201.0) read as unstructured -- a
        # legitimate trailing gap, not an interior one.
        self.assertGreaterEqual(summary["structured_layer_count"], 990)
        self.assertTrue(all(g["kind"] != "interior" for g in summary["gaps"]))
        self.assertGreaterEqual(summary["structured_coverage"], gsc.MIN_STRUCTURED_COVERAGE)
        self.assertEqual(gsc.structure_verdict(summary), "PASS")

    def test_hull_traversal_control_from_real_layers_reads_unstructured(self):
        layers = gsc.parse(REAL_GCODE, capture_points=True)
        # Sample every 50th body layer rather than regenerating all ~1000 --
        # this unit test only needs to show the check FIRES on unstructured
        # input built from real geometry, not re-measure the full file.
        vase_layers = []
        for layer in layers[5:-1:50]:
            pts = layer.points or []
            if len(pts) < gsc.MIN_POINTS_FOR_HULL:
                continue
            vase_layers.append(make_layer(layer.index, layer.z, hull_traversal_points(pts)))
        self.assertGreater(len(vase_layers), 5, "sample too small to be meaningful")
        analysis = gsc.analyze_structure(vase_layers, root=0, drop_final=False)
        self.assertEqual(analysis["summary"]["structured_layer_count"], 0)


if __name__ == "__main__":
    unittest.main()
