"""Island-disconnect regression with a real polygon-boolean backend, pinning
layer 1392 (Z=278.4 mm) of
``tests/fixtures/m35_corpus/naca4415_r140_t80_s300_w0.stl``.

WHY THIS FILE USES SHAPELY, EXPLICITLY FLAGGED: same reasoning as
``test_erosion_near_leading_edge.py`` — ``stub_ops.py`` cannot represent a real
many-slot lattice's erosion/diff, so this uses
``tools/shapely_ops.MiterShapelyOps`` (shared TEST/TOOL-ONLY real
polygon-boolean backend) as a real-geometry regression, never imported by
``aircraft_core`` or the shipped plugin bundle. Skipped outright if ``shapely``
is unavailable.

PINNED CONFIG, NOT THE LIVE ONE. This regression pins a SPECIFIC candidate
family (``deep_minus_lower``) at a specific layer/station. Loading the spec via
``tools/m34_rotation_equivalence.load_spec()`` reads the live, editable
``host/data/orca_plugins/config.json`` -- a live config without the ``deep_*``
families would silently make this test meaningless (nothing
named ``deep_minus_lower`` would exist to omit) rather than failing for the
reason it was written to catch. Instead this test loads
``tests/fixtures/m35_layer1392_config.json``, a copy of the tracked hand-synced
mirror ``presets/plugin/live_host_config.json`` taken when this test was
added, via the same ``structural_analysis.build_spec_from_cap_config``
path ``load_spec()`` itself uses. That is the exact spec this regression was
written against, frozen independent of any later edit to the live config.

THE FAILURE THIS PINS. Without the mitre gate, family ``deep_minus_lower``'s full-depth
candidate at this exact layer/chord-station passed every existing check (the
vertical skin-margin check, the ``FULL_DEPTH_MARGIN_HEADROOM_MM`` check, and
the thin-section halo) yet still disconnected the material after the real
``TopologyValidator`` erosion check — measured root cause: the vertical margin
measure does not model the MITRE-join corner loss the erosion actually applies
at this web's own far-cap corners (``join_style=2, mitre_limit=3.0``).
``far_cap_mitre_clearance_mm`` (``aircraft_core/frame.py``) closes that gap
with a second, additional gate. This test pins BOTH halves: the layer
validates cleanly, and the specific web that would disconnect it is omitted
with a ``skin margin:`` reason naming the mitre-dilated clearance shortfall.
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

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.frame import far_cap_mitre_clearance_mm  # noqa: E402
from aircraft_core.generator import LayerContext, StructureGenerator  # noqa: E402
from aircraft_core.mesh_section import MeshSectioner  # noqa: E402
from aircraft_core.slot import SlotBuilder  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

# Shared TEST/TOOL-ONLY real polygon-boolean backend -- see this module's own
# docstring for why this is safe to import here.
from shapely_ops import HAVE_SHAPELY as _HAVE_SHAPELY  # noqa: E402
from shapely_ops import MiterShapelyOps as _MiterShapelyOps  # noqa: E402

import m18_mesh_probe  # noqa: E402
import m34_rotation_equivalence as _m34  # noqa: E402
import structural_analysis as _sa  # noqa: E402

TESTS_UNIT_DIR = REPO_ROOT / "tests" / "unit"
if str(TESTS_UNIT_DIR) not in sys.path:
    sys.path.insert(0, str(TESTS_UNIT_DIR))

from stub_ops import StubOps  # noqa: E402

STL_PATH = REPO_ROOT / "tests" / "fixtures" / "m35_corpus" / "naca4415_r140_t80_s300_w0.stl"
# Pinned copy of the tracked hand-synced mirror
# presets/plugin/live_host_config.json taken when this regression was
# added -- see module docstring "PINNED CONFIG, NOT THE
# LIVE ONE".
PINNED_CONFIG_PATH = REPO_ROOT / "tests" / "fixtures" / "m35_layer1392_config.json"
LAYER_INDEX = 1392
WALL_WIDTH_MM = 0.42
# The exact offending station this fixture's own root-cause diagnosis measured:
# family ``deep_minus_lower``'s candidate at x~=-44.3314 mm.
OFFENDING_X_MM = -44.331402453560365


@unittest.skipUnless(_HAVE_SHAPELY, "shapely not installed; see module docstring")
class Layer1392RealBooleanRegressionTests(unittest.TestCase):
    """Full generate -> slot -> diff -> erosion pipeline, real geometry, exact
    layer/Z of the real failure."""

    @classmethod
    def setUpClass(cls):
        if not STL_PATH.exists():
            raise unittest.SkipTest("fixture STL not present: {}".format(STL_PATH))
        cls.ops = _MiterShapelyOps()
        with open(PINNED_CONFIG_PATH, "r", encoding="utf-8") as fh:
            doc = json.load(fh)
        entry = doc["config"][0] if isinstance(doc["config"], list) else doc["config"]
        cls.spec = _sa.build_spec_from_cap_config(entry["cap_config"])
        vertices, triangles = m18_mesh_probe.read_binary_stl(str(STL_PATH))
        cls.sectioner = MeshSectioner(vertices, triangles)
        cls.print_z = LAYER_INDEX * _m34.LAYER_HEIGHT_MM
        loops = cls.sectioner.section_at(cls.print_z)
        outer_loops = [loop for loop in loops if not loop.is_interior]
        outer = max(outer_loops, key=lambda loop: abs(loop.signed_area))
        cls.poly = cls.ops.polygon_from_points(outer.points)
        cls.frame = LayerFrame.from_polygon(cls.ops, cls.poly)

    def _generate(self):
        gen = StructureGenerator()
        ctx = LayerContext(layer_index=LAYER_INDEX, print_z=self.print_z)
        return gen.generate(
            self.frame,
            self.spec,
            WALL_WIDTH_MM,
            ctx,
            safety_offset_mm=WALL_WIDTH_MM,
        )

    def test_layer_validates_after_the_mitre_gate(self):
        """The exact real-boolean pipeline that would raise
        ``TopologyError`` (island-disconnect) on this layer without the mitre
        gate validates cleanly with it."""
        result = self._generate()
        builder = SlotBuilder()
        built = [builder.build(i, self.spec.overshoot_mm) for i in result.intents]
        slots = [self.ops.polygon_from_points(b.points) for b in built]
        transformed = self.ops.diff(self.poly, slots)
        validator = TopologyValidator(self.ops, WALL_WIDTH_MM)
        try:
            validator.validate(
                [self.poly],
                transformed,
                slots,
                self.spec,
                LAYER_INDEX,
                self.print_z,
                slot_labels=[str(i.family_name) for i in result.intents],
                family_indices=[i.family_membership() for i in result.intents],
            )
        except TopologyError as exc:
            self.fail(
                "layer {} should validate cleanly after the mitre gate, "
                "got: {}".format(LAYER_INDEX, exc)
            )

    def test_offending_web_is_omitted_for_mitre_clearance(self):
        """The specific ``deep_minus_lower`` candidate the root-cause
        diagnosis traced this failure to is omitted, with a ``skin
        margin:`` reason naming the mitre-dilated clearance shortfall --
        not silently dropped for some unrelated cause."""
        result = self._generate()
        matches = [
            o
            for o in result.omitted
            if o.family_name == "deep_minus_lower"
            and abs(o.x_mm - OFFENDING_X_MM) < 1e-6
        ]
        self.assertTrue(
            matches,
            "expected the deep_minus_lower candidate at x={:.4f} mm to be "
            "omitted; omitted reasons were: {!r}".format(
                OFFENDING_X_MM, [o.reason for o in result.omitted]
            ),
        )
        for o in matches:
            self.assertTrue(o.reason.startswith("skin margin:"))
            self.assertIn("mitre-dilated", o.reason)


class SteepFarSurfaceEdgeChainMembershipTests(unittest.TestCase):
    """A hand-built leading-edge-shaped polygon whose true nearest far-side edge
    is STEEPER than 45 degrees (``abs(dy) >= abs(dx)``) with no rotation at all
    -- the exact case the chain-membership scan exists for. An
    orientation filter (excluding ``abs(dy) >= abs(dx)``) would skip
    this edge and fall back to a farther, shallower one, overstating the
    clearance by more than 10x. Uses ``StubOps``/``PolygonShape`` directly (no
    shapely needed): this is a pure geometric property of
    ``far_cap_mitre_clearance_mm`` itself, not a real-boolean regression.

    GEOMETRY (mm), five vertices, deliberately NOT a NACA section so every
    coordinate here is exact and the expected distances are exact closed
    -form numbers, not something read off a mesh:

        (0, 0)    -- chord's min-X vertex ("leading edge")
        (50, -5)  -- lower chain's only interior vertex
        (100, 0)  -- chord's max-X vertex ("trailing edge")
        (60, 8)   -- upper chain interior vertex, shallow segment to TE
        (2, 6)    -- upper chain interior vertex, STEEP segment to LE:
                     dx=-2, dy=-6, abs(dy) >= abs(dx)

    The candidate cap sits at x_center=1.0 mm, just inboard of the LE,
    entering from the lower surface (``dir_to_far=+1.0``, far chain is
    UPPER). The true nearest far edge is the steep (2, 6)-(0, 0) segment;
    the farther, shallow (60, 8)-(2, 6) segment is what the old filter
    would have fallen back to.
    """

    POINTS = [
        (0.0, 0.0),
        (50.0, -5.0),
        (100.0, 0.0),
        (60.0, 8.0),
        (2.0, 6.0),
    ]

    def setUp(self):
        self.ops = StubOps()
        self.poly = self.ops.polygon_from_points(self.POINTS)

    def test_chain_method_measures_against_the_steep_edge(self):
        """The exact clearance to the STEEP (2,6)-(0,0) edge, verified by
        an independent point-to-line computation, not merely "smaller than
        the old answer"."""
        # Exact geometry: cap_far_y=0.6, cap segment x in [0.7, 1.3]. The
        # line through (0,0)-(2,6) is y=3x; solving y=0.6 gives the nearest
        # point on that infinite line at x=0.2 -- outside the cap segment's
        # own X range [0.7, 1.3], so the true minimum distance is from the
        # cap segment's near endpoint (0.7, 0.6) to that line, i.e. the
        # perpendicular distance from (0.7, 0.6) to y=3x (3x - y = 0):
        # |3*0.7 - 0.6| / sqrt(3**2 + 1**2).
        expected = abs(3.0 * 0.7 - 0.6) / math.sqrt(3.0 ** 2 + 1.0 ** 2)
        got = far_cap_mitre_clearance_mm(
            self.ops,
            self.poly,
            x_center_mm=1.0,
            width_mm=0.4,
            cap_y_mm=0.5,
            dir_to_far=1.0,
            safety_offset_mm=0.1,
        )
        self.assertAlmostEqual(got, expected, places=9)

    def test_old_orientation_filter_would_have_overstated_this(self):
        """Reproduces an orientation-filtered scan directly
        -- not by calling production code, since that filter is not
        part of ``aircraft_core.frame`` -- to demonstrate, on this exact
        fixture, that it would exclude the true nearest edge and
        fall back to a farther one, overstating the clearance by more
        than 10x. This is the failure mode the chain-membership scan
        exists to close; the previous test shows it does."""
        from aircraft_core.frame import (
            _clip_segment_to_x_window_mm,
            _segment_distance_mm,
        )

        points = self.POINTS
        n = len(points)
        x_center_mm, width_mm = 1.0, 0.4
        cap_y_mm, dir_to_far, safety_offset_mm = 0.5, 1.0, 0.1
        half_w = width_mm / 2.0
        r = safety_offset_mm
        cap_far_y = cap_y_mm + dir_to_far * r
        seg_a = (x_center_mm - half_w - r, cap_far_y)
        seg_b = (x_center_mm + half_w + r, cap_far_y)
        window_min_x = seg_a[0] - 5.0
        window_max_x = seg_b[0] + 5.0

        old_best = None
        for k in range(n):
            p, q = points[k], points[(k + 1) % n]
            dx, dy = q[0] - p[0], q[1] - p[1]
            if abs(dy) >= abs(dx):
                continue
            clipped = _clip_segment_to_x_window_mm(p, q, window_min_x, window_max_x)
            if clipped is None:
                continue
            cp, cq = clipped
            if not (
                dir_to_far * (cp[1] - cap_y_mm) > 0.0
                or dir_to_far * (cq[1] - cap_y_mm) > 0.0
            ):
                continue
            d = _segment_distance_mm(seg_a, seg_b, cp, cq)
            if old_best is None or d < old_best:
                old_best = d

        new_result = far_cap_mitre_clearance_mm(
            self.ops,
            self.poly,
            x_center_mm=x_center_mm,
            width_mm=width_mm,
            cap_y_mm=cap_y_mm,
            dir_to_far=dir_to_far,
            safety_offset_mm=safety_offset_mm,
        )
        self.assertIsNotNone(old_best)
        self.assertGreater(
            old_best,
            10.0 * new_result,
            "expected the old orientation-filtered scan to overstate the "
            "clearance by more than 10x on this fixture; got old={!r}, "
            "new={!r}".format(old_best, new_result),
        )


class RectangleCouponChainContentsTests(unittest.TestCase):
    """Pins what a rectangle coupon's chains ACTUALLY contain, so the
    structural property is fixed by a test rather than by a comment.

    No chain split can exclude an end wall -- an end wall is an edge and
    must belong to one arc or the other. On a rectangle the first-index
    tie-break puts BOTH walls in the upper chain. That is expected, and
    it is why the X window in ``far_cap_mitre_clearance_mm`` is a
    separate, necessary filter rather than a leftover: chain membership
    excludes near-surface edges, the window excludes distant end walls.
    """

    def test_a_rectangle_puts_both_end_walls_in_one_chain(self):
        from aircraft_core.frame import _split_upper_lower_chains
        x0, y0, x1, y1 = 0.0, 0.0, 20.0, 5.0
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        upper, lower = _split_upper_lower_chains(pts)

        def has_edge(chain, a, b):
            return any(
                (tuple(p), tuple(q)) in ((a, b), (b, a)) for p, q in chain
            )

        # The bottom edge is the lower chain, alone.
        self.assertTrue(has_edge(lower, (x0, y0), (x1, y0)))
        self.assertEqual(len(lower), 1, lower)
        # Both vertical end walls land in the upper chain, with the top.
        self.assertTrue(has_edge(upper, (x1, y0), (x1, y1)), upper)
        self.assertTrue(has_edge(upper, (x0, y1), (x0, y0)), upper)
        self.assertTrue(has_edge(upper, (x1, y1), (x0, y1)), upper)
        # Together the two chains partition every edge exactly once.
        self.assertEqual(len(upper) + len(lower), len(pts))



if __name__ == "__main__":
    unittest.main()
