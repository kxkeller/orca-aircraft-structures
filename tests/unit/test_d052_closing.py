"""The mesh-alignment guard's mesh side emulates the host's
``slice_closing_radius`` closing. Hermetic: tracked fixtures under
``tests/fixtures/island_disconnect`` only (never ``host/``).

Real fixture: layer 1098 of the root-down outside elevon wing. The host
bridged a 0.0218 mm neck between two islands (one outer contour); the mesh
sectioner has two outer loops. Negative controls that must still fail: a 0.5 mm
offset, a small rotation, genuinely separate islands (not falsely merged), and
a host single contour vs mesh loops far apart (loop-count message).
"""
import json
import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from aircraft_core import alignment as al  # noqa: E402

try:
    from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402
except ImportError:  # pragma: no cover
    HAVE_SHAPELY = False

NEEDS = unittest.skipUnless(HAVE_SHAPELY, "shapely required")
FX = REPO_ROOT / "tests" / "fixtures" / "island_disconnect" / "layer1098_closing_sections.json"
TOL = 0.02
R = 0.049


class _L:
    def __init__(self, pts):
        self.points = [tuple(p) for p in pts]


def _load():
    with open(FX, encoding="utf-8") as fh:
        return json.load(fh)


def _terms_delta(closed_outer, host_pts):
    best = max(closed_outer, key=lambda lp: abs(lp.signed_area))
    host = al.ClosedLoop(host_pts)
    a, b = al.alignment_terms(best), al.alignment_terms(host)
    return {k: abs(a[k] - b[k]) for k in a}


def _shift(pts, dx, dy):
    return [(x + dx, y + dy) for x, y in pts]


def _rotate(pts, deg):
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [(cx + c * (x - cx) - s * (y - cy), cy + s * (x - cx) + c * (y - cy)) for x, y in pts]


@NEEDS
class Layer1098ClosingTests(unittest.TestCase):
    def setUp(self):
        self.fx = _load()
        self.ops = MiterShapelyOps()
        self.host = [tuple(p) for p in self.fx["host_contour_pts_mm"]]

    def _outer(self, z):
        return [_L(l["points_mm"]) for l in self.fx["sections"][z] if not l["is_interior"]]

    def test_unclosed_mesh_fails_and_closed_mesh_matches_host(self):
        outer = self._outer("219.70")
        self.assertEqual(len(outer), 2)
        raw = al.close_outer_loops(self.ops, outer, 0.0)
        self.assertEqual(raw.loops_after, 2)
        self.assertGreater(max(_terms_delta(raw.outer, self.host).values()), 30.0)
        closed = al.close_outer_loops(self.ops, outer, R)
        self.assertEqual((closed.loops_before, closed.loops_after), (2, 1))
        deltas = _terms_delta(closed.outer, self.host)
        self.assertLess(max(deltas.values()), TOL, deltas)
        self.assertAlmostEqual(closed.area_after_mm2, 1165.98, delta=0.5)

    def test_planes_either_side(self):
        # 219.71: the islands are already fused (one loop). 219.69: the
        # islands are 0.088 mm apart (< 2r) but the neck is a corner-to-edge
        # wedge, and a closing only bridges where a disk of radius r cannot
        # fit ALONG the channel, so it is NOT bridged (the host would not
        # bridge it either) -- the bridge window is narrower than 2r.
        c71 = al.close_outer_loops(self.ops, self._outer("219.71"), R)
        self.assertEqual((c71.loops_before, c71.loops_after), (1, 1))
        c69 = al.close_outer_loops(self.ops, self._outer("219.69"), R)
        self.assertEqual((c69.loops_before, c69.loops_after), (2, 2))

    def test_offset_half_mm_still_fails(self):
        closed = al.close_outer_loops(self.ops, self._outer("219.70"), R)
        moved = [_L(_shift(lp.points, 0.5, 0.0)) for lp in closed.outer]
        again = al.close_outer_loops(self.ops, moved, 0.0)
        deltas = _terms_delta(again.outer, self.host)
        self.assertGreater(max(deltas.values()), TOL, deltas)
        self.assertGreater(deltas["min_x"], 0.4)

    def test_small_rotation_still_fails(self):
        closed = al.close_outer_loops(self.ops, self._outer("219.70"), R)
        rot = [_L(_rotate(lp.points, 0.3)) for lp in closed.outer]
        deltas = _terms_delta(al.close_outer_loops(self.ops, rot, 0.0).outer, self.host)
        self.assertGreater(max(deltas.values()), TOL, deltas)


@NEEDS
class NegativeControlTests(unittest.TestCase):
    def setUp(self):
        self.ops = MiterShapelyOps()

    def test_two_genuine_islands_are_not_merged(self):
        a = _L([(0, 0), (20, 0), (20, 10), (0, 10)])
        b = _L([(21.0, 0), (30, 0), (30, 10), (21.0, 10)])   # 1 mm gap >> 2r
        closed = al.close_outer_loops(self.ops, [a, b], R)
        self.assertEqual((closed.loops_before, closed.loops_after), (2, 2))
        host_two = [al.ClosedLoop(a.points), al.ClosedLoop(b.points)]
        best_h = max(host_two, key=lambda lp: abs(lp.signed_area))
        best_m = max(closed.outer, key=lambda lp: abs(lp.signed_area))
        ta, tb = al.alignment_terms(best_h), al.alignment_terms(best_m)
        self.assertLess(max(abs(ta[k] - tb[k]) for k in ta), TOL)

    def test_neck_just_over_2r_is_not_bridged(self):
        a = _L([(0, 0), (20, 0), (20, 10), (0, 10)])
        b = _L([(20.11, 0), (30, 0), (30, 10), (20.11, 10)])  # 0.11 > 2r = 0.098
        self.assertEqual(al.close_outer_loops(self.ops, [a, b], R).loops_after, 2)
        b2 = _L([(20.09, 0), (30, 0), (30, 10), (20.09, 10)])  # 0.09 < 2r
        self.assertEqual(al.close_outer_loops(self.ops, [a, b2], R).loops_after, 1)

    def test_host_single_contour_vs_far_apart_mesh_loops_fails_with_loop_note(self):
        a = _L([(0, 0), (20, 0), (20, 10), (0, 10)])
        b = _L([(60, 0), (80, 0), (80, 10), (60, 10)])
        closed = al.close_outer_loops(self.ops, [a, b], R)
        self.assertEqual(closed.loops_after, 2)
        host = [(0, 0), (20, 0), (20, 10), (0, 10)] and [(0, 0), (80, 0), (80, 10), (0, 10)]
        deltas = _terms_delta(closed.outer, host)
        eff = {k: TOL for k in deltas}
        decision = al.decide_alignment(deltas, eff, False, False)
        self.assertEqual(decision["outcome"], "raise_bbox")
        note = al.loop_count_mismatch_note(closed.loops_before, closed.loops_after, R)
        self.assertIn("Loop-count mismatch", note)
        self.assertIn("2 outer loops", note)
        self.assertEqual(al.loop_count_mismatch_note(2, 1, R), "")

    def test_single_loop_is_skipped_when_asked_and_closed_otherwise(self):
        a = _L([(0, 0), (20, 0), (20, 10), (0, 10)])
        skipped = al.close_outer_loops(self.ops, [a], R, skip_single_loop=True)
        self.assertFalse(skipped.applied)
        self.assertIs(skipped.outer[0], a)
        closed = al.close_outer_loops(self.ops, [a], R)
        self.assertTrue(closed.applied)
        self.assertEqual(closed.loops_after, 1)

    def test_zero_radius_passes_loops_through(self):
        a = _L([(0, 0), (20, 0), (20, 10), (0, 10)])
        self.assertEqual(al.close_outer_loops(self.ops, [a], 0.0).loops_after, 1)


class ClosingRadiusParsingTests(unittest.TestCase):
    def test_parses_numbers_and_strings(self):
        self.assertEqual(al.parse_closing_radius_mm(0.049), 0.049)
        self.assertEqual(al.parse_closing_radius_mm("0.049"), 0.049)
        self.assertEqual(al.parse_closing_radius_mm(0), 0.0)

    def test_missing_or_unparseable_fails_closed_naming_the_key(self):
        for bad in (None, "", "<error: boom>", "abc", float("nan"), -0.1, True):
            with self.assertRaises(al.ClosingRadiusError) as ctx:
                al.parse_closing_radius_mm(bad)
            self.assertIn("slice_closing_radius", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
