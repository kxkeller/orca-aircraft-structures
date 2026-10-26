"""A section thinner than two wall widths is not "a real aerofoil whose pitch
config collapsed" -- it is provably all skin, with no material to build a wall
or a web into. ``StructureGenerator.generate()``'s pitch-collapse fallback must
not classify such a sliver as AEROFOIL and re-raise ``_SectionTooSmallForPitch``
unconditionally, as it does for a genuine thin aerofoil (high-aspect/low-fill
section); a knife-edge sliver is not really an aerofoil section at all.

PERMANENT FIXTURE: ``tests/fixtures/tip/
elevon_layer1247_sliver_points.json`` is the REAL, measured mesh cross-section
of ``newoutsidewingelevon.stl`` at layer 1247 (print_z 249.6 mm, slice_z 249.5
mm, 9.155e-5 mm below the mesh's own z_max) -- NOT a synthetic shape. See that
file's own ``_comment`` for why it is dumped in the canonical (later-rotation)
frame rather than the raw mesh frame.
"""

import json
import os
import unittest
from pathlib import Path

import sys

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))
import make_fixtures as _mf  # noqa: E402

from stub_ops import PolygonShape, StubOps

from aircraft_core.frame import AEROFOIL_REGION, LayerFrame, classify_region
from aircraft_core.generator import (
    LayerContext,
    PRINTABLE_FLOOR_WALL_MULTIPLIER,
    StructureGenerator,
    _SectionTooSmallForPitch,
)
from aircraft_core.spec import StructureSpec, WebFamily

WALL_WIDTH_MM = 0.42
FIXTURE_PATH = REPO_ROOT / "tests" / "fixtures" / "tip" / "elevon_layer1247_sliver_points.json"


def _pitch_collapsing_family(**overrides):
    kwargs = dict(
        name="tiny_pitch",
        role="skin_lattice",
        rib_pitch_chord_frac=0.12,
        web_position=0.35,
        web_depth_mm=1.5,
    )
    kwargs.update(overrides)
    return WebFamily(**kwargs)


class ElevonTipSliverShutsDownTests(unittest.TestCase):
    """The real, measured layer-1247 sliver: a knife-edge section whose
    smaller in-plane extent is ~100x smaller than a single wall, let alone
    two -- must degrade (empty intents, a distinct degradation kind),
    never re-raise ``_SectionTooSmallForPitch`` as if it were a real
    aerofoil pitch-collapse failure.
    """

    def setUp(self):
        with open(FIXTURE_PATH, "r", encoding="utf-8") as fh:
            self.fixture = json.load(fh)
        points = tuple(tuple(p) for p in self.fixture["points_mm"])
        ops = StubOps()
        self.frame = LayerFrame.from_polygon(ops, PolygonShape(points))
        self.gen = StructureGenerator()
        self.layer = LayerContext(
            layer_index=self.fixture["layer_index"],
            print_z=self.fixture["print_z_mm"],
        )
        self.spec = StructureSpec(
            slot_gap_mm=0.3,
            skin_margin_mm=0.86,
            families=(_pitch_collapsing_family(),),
        )

    def test_fixture_reads_as_a_knife_edge_sliver_classified_aerofoil(self):
        # Sanity check on the fixture itself, so a future measurement
        # change doesn't silently stop exercising this bug.
        self.assertLess(self.frame.depth, WALL_WIDTH_MM)
        self.assertGreater(self.frame.width, 1.0)
        self.assertEqual(classify_region(self.frame), AEROFOIL_REGION)

    def test_the_aerofoil_path_really_does_raise_for_this_frame(self):
        # Confirm the failure this fixture is meant to exercise actually
        # happens on this exact frame/spec before asserting on FIX 1's own
        # behaviour -- otherwise a passing test below would prove nothing.
        with self.assertRaises(_SectionTooSmallForPitch):
            self.gen._generate_family(
                self.frame, self.spec, self.spec.families[0], 0,
                WALL_WIDTH_MM, self.layer, WALL_WIDTH_MM,
                self.spec.skin_margin_mm, None,
            )

    def test_generate_shuts_down_with_the_printable_floor_kind(self):
        result = self.gen.generate(
            self.frame, self.spec, WALL_WIDTH_MM, self.layer,
            safety_offset_mm=WALL_WIDTH_MM,
        )
        self.assertEqual(result.intents, [])
        kinds = [d.kind for d in result.degradations]
        self.assertIn("section_below_printable_floor", kinds)
        reason = next(
            d.reason for d in result.degradations
            if d.kind == "section_below_printable_floor"
        )
        # Names the measured numbers, not just a generic message.
        self.assertIn("{:.4f} mm".format(self.frame.depth), reason)
        self.assertIn("{:.4f} mm".format(self.frame.width), reason)
        floor_mm = PRINTABLE_FLOOR_WALL_MULTIPLIER * WALL_WIDTH_MM
        self.assertIn("{:.4f} mm".format(floor_mm), reason)

    def test_does_not_raise(self):
        # The whole point: a slice built on this layer must survive.
        try:
            self.gen.generate(
                self.frame, self.spec, WALL_WIDTH_MM, self.layer,
                safety_offset_mm=WALL_WIDTH_MM,
            )
        except _SectionTooSmallForPitch as exc:
            self.fail(
                "generate() re-raised _SectionTooSmallForPitch for a "
                "section below the printable floor: {}".format(exc)
            )


class ThickAerofoilWithImpossiblePitchStillRaisesTests(unittest.TestCase):
    """FIX 1's own safety boundary: a genuinely thick (well above the
    printable floor) AEROFOIL section whose pitch config collapses must
    still raise, unaffected by the new floor branch -- the floor check
    only short-circuits sections that are provably too thin to carry any
    wall, never a real aerofoil pitch-configuration problem.
    """

    def test_60mm_chord_naca_with_pitch_collapse_still_raises(self):
        m, p, t = _mf.parse_naca4("2412")
        _tris, outline_mm, _bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, PolygonShape(tuple(outline_mm)))
        self.assertEqual(classify_region(frame), AEROFOIL_REGION)
        # Comfortably above the printable floor (2 * 0.42 = 0.84 mm) --
        # a 60 mm chord NACA 2412's depth is several mm.
        self.assertGreater(frame.depth, 2.0 * WALL_WIDTH_MM)

        spec = StructureSpec(
            slot_gap_mm=0.3, skin_margin_mm=0.86,
            families=(_pitch_collapsing_family(rib_pitch_chord_frac=0.0005),),
        )
        gen = StructureGenerator()
        layer = LayerContext(layer_index=0, print_z=0.2)
        with self.assertRaises(_SectionTooSmallForPitch):
            gen.generate(
                frame, spec, WALL_WIDTH_MM, layer,
                safety_offset_mm=WALL_WIDTH_MM,
            )


if __name__ == "__main__":
    unittest.main()
