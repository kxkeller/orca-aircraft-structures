"""An OPPOSING-FACE lateral-clearance pre-check, sibling to
``far_cap_mitre_clearance_mm``, catching a candidate web whose SIDE WALL (not
far cap) sits too close to a nearby opposing section feature (e.g. an open
notch) on the SAME side the web enters from. See
``aircraft_core.frame.opposing_face_lateral_clearance_mm``'s
own docstring for the exact predicate.

Explicitly NOT a per-web local island/drop-and-revalidate check -- a pure
GEOMETRIC pre-check against the section's own outer contour, exactly like
``far_cap_mitre_clearance_mm`` already is.
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
import make_fixtures as _mf  # noqa: E402

from stub_ops import PolygonShape, StubOps

from aircraft_core.frame import LayerFrame, opposing_face_lateral_clearance_mm
from aircraft_core.generator import (
    LayerContext,
    StructureGenerator,
    WebIntent,
)
from aircraft_core.spec import StructureSpec, WebFamily
from aircraft_core.validate import TopologyError, TopologyValidator

WALL_WIDTH_MM = 0.42
FIXTURE_PATH = (
    REPO_ROOT / "tests" / "fixtures" / "island_disconnect"
    / "layer786_entry_wedge_minimal.json"
)

CAP_CONFIG_PATH = (
    REPO_ROOT / "tests" / "fixtures" / "island_disconnect"
    / "elevon_cap_config.json"
)

try:
    import structural_analysis as sa  # noqa: E402
    _HAVE_SA = True
except ImportError:  # pragma: no cover
    _HAVE_SA = False

try:
    from shapely_ops import HAVE_SHAPELY, MiterShapelyOps
except ImportError:  # pragma: no cover
    HAVE_SHAPELY = False


def _live_spec():
    # Hermetic: tracked copy of the elevon cap_config, never host/.
    with open(CAP_CONFIG_PATH, encoding="utf-8") as fh:
        return sa.build_spec_from_cap_config(json.load(fh))


@unittest.skipUnless(HAVE_SHAPELY and _HAVE_SA, "shapely + structural_analysis required")
class Layer786RealFailureIsFixedTests(unittest.TestCase):
    """(a) The real, measured layer-786 fixture: slot 12's own candidate
    must be omitted by the new lateral check, and the resulting layer must
    validate (single connected island) -- reproducing the measured scenario."""

    def setUp(self):
        with open(FIXTURE_PATH, "r", encoding="utf-8") as fh:
            self.fixture = json.load(fh)
        self.ops = MiterShapelyOps()
        self.original = self.ops.polygon_from_points(
            [tuple(p) for p in self.fixture["host_contour_pts_mm"]]
        )
        self.frame = LayerFrame.from_polygon(self.ops, self.original)
        self.spec = _live_spec()

    def test_slot_12_web_is_omitted_and_layer_validates(self):
        gen = StructureGenerator()
        layer = LayerContext(layer_index=786, print_z=157.3999999999996)
        result = gen.generate(
            self.frame, self.spec, WALL_WIDTH_MM, layer,
            safety_offset_mm=WALL_WIDTH_MM,
        )
        # The known failing station (slot 12, 1-indexed) sits at x=-22.5395.
        omitted_xs = [round(o.x_mm, 2) for o in result.omitted]
        self.assertIn(-22.54, omitted_xs)
        lateral_reasons = [o for o in result.omitted if "LATERAL" in o.reason]
        self.assertTrue(
            any(round(o.x_mm, 2) == -22.54 for o in lateral_reasons),
            "expected slot 12's omission to cite the new LATERAL clearance check",
        )

        import aircraft_core.slot as slotmod
        builder = slotmod.SlotBuilder()
        slots = [
            self.ops.polygon_from_points(builder.build(i, self.spec.overshoot_mm).points)
            for i in result.intents
        ]
        replacement = list(self.ops.diff(self.original, slots))
        slot_labels = [
            "family {} ({!r})".format(i.family_index, i.family_name)
            for i in result.intents
        ]
        family_indices = [i.family_membership() for i in result.intents]
        validator = TopologyValidator(self.ops, safety_offset_mm=WALL_WIDTH_MM)
        try:
            validator.validate(
                [self.original], replacement, slots, self.spec, 786, 157.4,
                slot_labels=slot_labels, family_indices=family_indices,
            )
        except TopologyError as exc:
            self.fail("layer 786 still fails to validate after the fix: {}".format(exc))
        self.assertEqual(len(replacement), 1, "expected a single connected island")


@unittest.skipUnless(HAVE_SHAPELY, "shapely required for the real polygon backend")
class NearLeadingEdgeWebIsNotOmittedTests(unittest.TestCase):
    """(b) A web entering near a vertex on a finely tessellated leading
    edge must NOT be omitted by the new check -- the false-positive risk
    explicitly flagged during this predicate's own design (a smoothly
    curving LE can locally satisfy a naive "opposing normal" test purely
    from its own curvature, not from a genuine second feature)."""

    def _naca_frame(self, n_points=400):
        m, p, t = _mf.parse_naca4("2412")
        _tris, outline_mm, _bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, n_points)
        ops = MiterShapelyOps()
        polygon = ops.polygon_from_points(outline_mm)
        frame = LayerFrame.from_polygon(ops, polygon)
        return ops, frame

    def test_marching_family_near_le_is_not_omitted_for_lateral_reasons(self):
        ops, frame = self._naca_frame()
        chord = frame.max_x - frame.min_x
        # A marching lattice family (never raises on a thin station -- it omits
        # and moves on), pitched densely enough to place a candidate only ~3%
        # chord from the leading edge, comfortably closer than this project's
        # own aerofoil TE-truncation margins but still past the point where the
        # thin-section/skin-margin checks already exclude it on their own
        # (measured in this test's own development: a single explicit-depth
        # candidate this close raises/omits before ever reaching the lateral
        # check at all -- a MARCHING family is the case that actually reaches
        # it).
        family = WebFamily(
            name="dense_lattice", role="skin_lattice",
            web_position=0.05, rib_pitch_chord_frac=0.05, web_depth_mm=1.0,
        )
        spec = StructureSpec(slot_gap_mm=0.3, skin_margin_mm=0.86, families=(family,))
        gen = StructureGenerator()
        layer = LayerContext(layer_index=0, print_z=0.2)
        result = gen.generate(frame, spec, WALL_WIDTH_MM, layer, safety_offset_mm=WALL_WIDTH_MM)

        near_le_x = frame.min_x + 0.05 * chord
        lateral_omissions_near_le = [
            o for o in result.omitted
            if "LATERAL" in o.reason and abs(o.x_mm - near_le_x) < 0.5 * chord * 0.05
        ]
        self.assertEqual(
            lateral_omissions_near_le, [],
            "a web near the LE was omitted for a LATERAL reason -- likely a "
            "false positive from the LE's own curvature: {!r}".format(
                lateral_omissions_near_le
            ),
        )

    def test_clearance_function_directly_near_le_is_generous(self):
        ops, frame = self._naca_frame()
        # 3 mm from the LE on a 60 mm chord (5% chord) -- comfortably closer
        # than any real family station this project's own fixtures place,
        # per the measured 3 mm natural exclusion zone above.
        x_center = frame.min_x + 3.0
        for entry_lower in (False, True):
            clearance = opposing_face_lateral_clearance_mm(
                ops, frame.polygon, x_center, 0.021, 1.0, -0.5,
                entry_lower, WALL_WIDTH_MM,
            )
            self.assertGreater(
                clearance, WALL_WIDTH_MM,
                "entry_lower={}: near-LE clearance {} unexpectedly tight".format(
                    entry_lower, clearance
                ),
            )


@unittest.skipUnless(HAVE_SHAPELY, "shapely required for the real polygon backend")
class FilletedNotchWallBoundaryTests(unittest.TestCase):
    """(c) A filleted/curved notch wall, swept across the 45-degree
    anti-parallel cone boundary this predicate's angle test relies on."""

    def _rect_with_angled_notch_wall(self, wall_angle_deg):
        """A simple rectangular section (100 x 40 mm, centred at origin)
        with a single notch cut into the +Y (upper) edge whose LEFT wall is
        tilted ``wall_angle_deg`` off vertical (0 deg = exactly parallel to
        a lattice web's own vertical side wall, the worst case; 90 deg =
        horizontal, i.e. co-planar with the entry skin -- never anti
        -parallel to a vertical wall's normal at all)."""
        half_w, half_d = 50.0, 20.0
        notch_left_x = -10.0
        notch_right_x = 10.0
        notch_depth = 15.0
        notch_bottom_y = half_d - notch_depth
        # Tilt the notch's LEFT wall about its own bottom corner.
        angle = math.radians(wall_angle_deg)
        top_x = notch_left_x + notch_depth * math.tan(angle)
        # Build explicitly, CCW, one vertex per corner.
        points = [
            (-half_w, -half_d),
            (half_w, -half_d),
            (half_w, half_d),
            (notch_right_x, half_d),
            (notch_right_x, notch_bottom_y),
            (notch_left_x, notch_bottom_y),
            (top_x, half_d),
            (-half_w, half_d),
        ]
        ops = MiterShapelyOps()
        polygon = ops.polygon_from_points(points)
        return ops, polygon, notch_left_x

    def test_wall_within_45_degrees_of_parallel_is_caught(self):
        # 0 deg tilt: the notch's left wall is exactly vertical, exactly
        # parallel to a lattice web's own side wall -- the design case.
        # Web's own Y range (8..10) sits well within the notch's own Y
        # extent (5..20) so the dilated face's ends do not run off it.
        ops, polygon, notch_left_x = self._rect_with_angled_notch_wall(0.0)
        raw_gap = 0.7  # < 2 * WALL_WIDTH_MM (0.84) -- must be caught
        wall_x = notch_left_x - raw_gap
        x_center = wall_x - 0.021 / 2.0
        clearance = opposing_face_lateral_clearance_mm(
            ops, polygon, x_center, 0.021, 10.0, 8.0, False, WALL_WIDTH_MM,
        )
        self.assertLessEqual(clearance, WALL_WIDTH_MM)

    def test_wall_beyond_45_degrees_drops_out(self):
        # 60 deg tilt: the notch's own wall leans past the 45-degree
        # cone -- its material-side normal is not within 45 degrees
        # of anti-parallel to the web's vertical wall normal, so it must
        # NOT be treated as an opposing face (this predicate's own
        # documented boundary, not a claim that a 60-degree wall is
        # structurally safe by some other measure). Same raw gap as above.
        ops, polygon, notch_left_x = self._rect_with_angled_notch_wall(60.0)
        raw_gap = 0.7
        wall_x = notch_left_x - raw_gap
        x_center = wall_x - 0.021 / 2.0
        clearance = opposing_face_lateral_clearance_mm(
            ops, polygon, x_center, 0.021, 10.0, 8.0, False, WALL_WIDTH_MM,
        )
        # The tilted wall itself must drop out of consideration (its own
        # material-side normal not within 45 deg of anti-parallel);
        # whatever edge the scan finds instead (here, the rectangle's own
        # far side wall) is not the point of this test and is comfortably
        # clear regardless.
        self.assertGreater(clearance, WALL_WIDTH_MM)


@unittest.skipUnless(HAVE_SHAPELY, "shapely required for the real polygon backend")
class ChordEndWallTests(unittest.TestCase):
    """A blunt global chord end is not a re-entrant opposing feature."""

    def test_global_end_walls_are_excluded(self):
        ops = MiterShapelyOps()
        polygon = ops.polygon_from_points(
            [(-4.32, -4.335), (4.32, -4.335), (4.32, 4.335), (-4.32, 4.335)]
        )
        for x_center in (-4.0, 4.0):
            clearance = opposing_face_lateral_clearance_mm(
                ops, polygon, x_center, 0.21, -4.335, -2.63,
                True, WALL_WIDTH_MM,
            )
            self.assertGreater(clearance, WALL_WIDTH_MM)


if __name__ == "__main__":
    unittest.main()
