"""The whole-layer erosion catch-all is not dead code behind
``StructureSpec.validate(safety_offset_mm=...)``.

WHY THIS FILE EXISTS: ``StructureGenerator.generate()`` and
``capability.py``'s ``_build_spec`` both call ``spec.validate(
safety_offset_mm=...)`` before placing any web (see
``aircraft_core/spec.py``'s ``StructureSpec.validate`` docstring and
``docs/geometry_invariants.md``'s FRAGILITY WARNING). That guard enforces

    skin_margin_mm > 2 * safety_offset_mm  ( = min_wall_mm )

and, once it holds, a SINGLE-SIDED cut's own remaining-skin invariant
(invariant 4: ``remaining_skin >= skin_margin_mm``) ALREADY guarantees the
bridge beneath that cut survives the validator's own erosion check, on flat
(non-rotated, non-curved-at-the-cut) geometry:

    remaining_skin - 2*safety_offset_mm >= skin_margin_mm - 2*safety_offset_mm > 0

The same argument extends to the opposing-pair check (its equality
-fails-closed condition on ``d_upper + d_lower + min_wall``), so under a valid
spec, NEITHER invariant 4 nor the opposing-pair check can be the thing that
lets a disconnection through to the whole-layer erosion check UNDER A SLOT.
That could read as "the erosion catch-all is unreachable" — the guard
shadows the ONLY path the existing validator-level fixture
(``tests/unit/test_validate.py:181-216``,
``test_disconnect_after_safety_offset_raises``) uses to reach it, which needs
``skin_margin_mm=0.2`` (itself now a config the live generator/ capability path
refuses before ever building a web).

THE ANSWER, proven here: invariants 3/4 and the collision/opposing checks
are all PER-SLOT (or per-pair-of-slots) — none of them ever measures
material that is not directly beneath or beside a cut. A thin neck
elsewhere in the ORIGINAL section, entirely unrelated to any web, is
invisible to every one of those checks and is caught ONLY by the
whole-layer erosion check (``validate.py``, "erode by the safety offset and
confirm the material is still one connected island", around line 996) —
this is precisely what makes it a genuine catch-all rather than a redundant
final check. This file builds that fixture (a "dogbone" original section:
two full-height lobes joined by a 0.5 mm waist, well under
``2 * safety_offset_mm = 0.84 mm``) and drives it through the LIVE path end
to end -- ``StructureGenerator.generate()`` (where the now-wired
``spec.validate(safety_offset_mm=...)`` guard actually runs and PASSES,
``skin_margin_mm=1.0 > 0.84``), ``SlotBuilder.build()``, the boolean
cut, and ``TopologyValidator.validate()`` -- exactly the chain
``tests/unit/test_thick_skin_path.py``'s ``ArmCPreconditionBreaksOnThe*
Tests`` classes use, not a hand-built ``transformed`` shape handed directly
to the validator (which would only replicate the validator-level coverage
that already exists).

The web itself is placed well inside one lobe, far from the waist, so every
per-slot invariant sees only thick, comfortable material -- the ONLY thing
that can fail is the whole-layer erosion check tripping over the waist.
"""
import unittest

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.slot import SlotBuilder
from aircraft_core.spec import StructureSpec
from aircraft_core.validate import TopologyError, TopologyValidator
from stub_ops import PinchedNotchedPolygon, PolygonShape, Rect, StubOps

OUTER_WALL_MM = 0.42
SAFETY_OFFSET_MM = OUTER_WALL_MM  # capability.py's own convention
MIN_WALL_MM = 2.0 * SAFETY_OFFSET_MM  # 0.84 mm
SKIN_MARGIN_MM = 1.0  # > MIN_WALL_MM -- a fully valid spec per the guard

# Dogbone original section: two 15 mm-wide, 20 mm-tall lobes (x in
# [-20, -5] and [5, 20], y in [-10, 10]) joined by a 10 mm-wide waist
# (x in [-5, 5]) that is only WAIST_THICKNESS_MM tall (y in
# [-WAIST_THICKNESS_MM/2, WAIST_THICKNESS_MM/2]) -- well under MIN_WALL_MM,
# so the waist cannot survive a 2 * SAFETY_OFFSET_MM erosion, independent of
# skin_margin_mm entirely.
WAIST_THICKNESS_MM = 0.5
assert WAIST_THICKNESS_MM < MIN_WALL_MM, "fixture must not survive erosion"

_HALF_WAIST = WAIST_THICKNESS_MM / 2.0
DOGBONE_POINTS = (
    (-20.0, -10.0),
    (-20.0, 10.0),
    (-5.0, 10.0),
    (-5.0, _HALF_WAIST),
    (5.0, _HALF_WAIST),
    (5.0, 10.0),
    (20.0, 10.0),
    (20.0, -10.0),
    (5.0, -10.0),
    (5.0, -_HALF_WAIST),
    (-5.0, -_HALF_WAIST),
    (-5.0, -10.0),
)


def _rect_from_points(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return Rect(min(xs), min(ys), max(xs), max(ys))


class ErosionCatchAllFiresUnderValidSpecTests(unittest.TestCase):
    """The live generate -> build -> diff -> validate chain, at a fully
    valid ``skin_margin_mm`` (1.0 > 0.84), on a section with a thin neck
    far from the web -- proving the erosion catch-all still fires for a
    GEOMETRY-driven cause, not a margin-config-driven one."""

    def setUp(self):
        self.ops = StubOps()
        self.outer = PolygonShape(DOGBONE_POINTS)
        self.frame = LayerFrame.from_polygon(self.ops, self.outer)
        self.gen = StructureGenerator()
        self.builder = SlotBuilder()

    def test_original_section_is_genuinely_one_piece_before_any_cut(self):
        # Fixture sanity: invariant 1/2 would trivially pass on the
        # UNCUT original regardless -- this just confirms the dogbone is a
        # single simple polygon, not an accidental self-intersection or
        # two disjoint pieces already.
        min_x, min_y, max_x, max_y = self.ops.bounds(self.outer)
        self.assertEqual((min_x, min_y, max_x, max_y), (-20.0, -10.0, 20.0, 10.0))
        # Waist is thinner than 2 * SAFETY_OFFSET_MM by construction.
        self.assertLess(WAIST_THICKNESS_MM, MIN_WALL_MM)

    def test_erosion_catches_the_pinched_waist_with_a_fully_valid_spec(self):
        # web_position=0.2 -> frame.web_x(0.2) = -20 + 0.2*40 = -12 mm,
        # squarely inside the left lobe (x in [-20, -5]), far from the
        # waist (x in [-5, 5]) and from either lobe's own edges.
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=SKIN_MARGIN_MM, web_position=0.2
        )
        layer = LayerContext(layer_index=7, print_z=1.4)

        # 1. generate() -- this is where spec.validate(safety_offset_mm=...)
        # actually runs, and it must PASS: 1.0 > 0.84.
        result = self.gen.generate(
            self.frame, spec, OUTER_WALL_MM, layer, safety_offset_mm=SAFETY_OFFSET_MM
        )
        self.assertEqual(len(result.intents), 1, "expected exactly one web")
        self.assertEqual(result.omitted, [])
        intent = result.intents[0]
        self.assertAlmostEqual(intent.start[0], -12.0, places=6)

        # 2. SlotBuilder -- the same live step every other end-to-end
        # fixture in this suite uses.
        built = self.builder.build(intent, spec.overshoot_mm)
        notch = _rect_from_points(built.points)

        # 3. The boolean cut. StubOps has no general PolygonShape-minus-Rect
        # diff (see stub_ops.py's module docstring); every other
        # non-rectangular end-to-end fixture in this suite
        # (test_thick_skin_path.py's ArmCPreconditionBreaksOnTheAirfoilTests)
        # constructs the post-cut shape directly for the same documented
        # reason -- this is that same, established convention, not a
        # shortcut invented for this fixture.
        transformed = [
            PinchedNotchedPolygon(
                outer=self.outer,
                notch=notch,
                waist_x_mm=0.0,
                waist_thickness_mm=WAIST_THICKNESS_MM,
            )
        ]

        # 4. TopologyValidator -- invariants 3/4 must pass (the web sits in
        # comfortable material), and the whole-layer erosion check must be
        # the thing that fails, on the waist, not the slot.
        validator = TopologyValidator(self.ops, safety_offset_mm=SAFETY_OFFSET_MM)
        with self.assertRaises(TopologyError) as ctx:
            validator.validate(
                [self.outer],
                transformed,
                [notch],
                spec,
                layer.layer_index,
                layer.print_z,
            )
        message = str(ctx.exception)
        self.assertIn(
            "disconnects material island after {:.2f} mm safety offset".format(
                SAFETY_OFFSET_MM
            ),
            message,
        )
        self.assertIn("layer 7", message)
        # Not a margin/skin-margin message -- proves the failure is the
        # whole-layer erosion catch-all, not invariant 4 misfiring under a
        # valid margin.
        self.assertNotIn("required {:.4f} mm margin".format(SKIN_MARGIN_MM), message)

    def test_same_config_survives_erosion_once_the_waist_is_thick_enough(self):
        # Control: the identical chain, with the waist widened to clear
        # 2 * SAFETY_OFFSET_MM, must NOT raise -- confirms the failure
        # above is specifically about the waist's thickness, not an
        # artifact of this fixture's construction.
        thick_points = tuple(
            (x, y if abs(y) < 1.0 else y) for x, y in DOGBONE_POINTS
        )
        # Rebuild with a waist thick enough to survive (> MIN_WALL_MM).
        half = (MIN_WALL_MM + 0.5) / 2.0
        thick_points = (
            (-20.0, -10.0),
            (-20.0, 10.0),
            (-5.0, 10.0),
            (-5.0, half),
            (5.0, half),
            (5.0, 10.0),
            (20.0, 10.0),
            (20.0, -10.0),
            (5.0, -10.0),
            (5.0, -half),
            (-5.0, -half),
            (-5.0, -10.0),
        )
        outer = PolygonShape(thick_points)
        frame = LayerFrame.from_polygon(self.ops, outer)
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=SKIN_MARGIN_MM, web_position=0.2
        )
        layer = LayerContext(layer_index=7, print_z=1.4)
        result = self.gen.generate(
            frame, spec, OUTER_WALL_MM, layer, safety_offset_mm=SAFETY_OFFSET_MM
        )
        self.assertEqual(len(result.intents), 1)
        built = self.builder.build(result.intents[0], spec.overshoot_mm)
        notch = _rect_from_points(built.points)
        transformed = [
            PinchedNotchedPolygon(
                outer=outer,
                notch=notch,
                waist_x_mm=0.0,
                waist_thickness_mm=2.0 * half,
            )
        ]
        validator = TopologyValidator(self.ops, safety_offset_mm=SAFETY_OFFSET_MM)
        # Must not raise.
        validator.validate(
            [outer], transformed, [notch], spec, layer.layer_index, layer.print_z
        )


if __name__ == "__main__":
    unittest.main()
