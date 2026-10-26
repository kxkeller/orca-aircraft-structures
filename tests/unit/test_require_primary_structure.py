"""Regression: primary structure must not vanish from a passing slice.

THE MEASURED CASE THIS GUARDS (
). A two-spar config -- `spar_main` at 0.25c and `spar_rear` at 0.65c,
both full depth -- was swept over every layer of a 150 -> 120 mm x 120 mm wing
at the shipped 0.42 mm line width. It reported:

    VERIFIED 595/595 layers, 0 failures

with `spar_rear` OMITTED at every single station. Nothing objected, because
nothing was asked to. Every omission was individually correct -- the member did
not fit -- and the resulting layers were genuinely valid single loops. The part
simply had no rear spar in it.

That trade is right for a SKIN LATTICE: one stiffener fewer is a small
degradation, and failing the whole slice over it would be worse. It is wrong
for PRIMARY STRUCTURE, where the omission deletes a load path and the slice
still passes every check.

exists because of the inverse error -- "do not infer 'spar' from depth
alone" (see that test's docstring). This test pins the other
direction: **do not infer a spar's PRESENCE from a passing slice.**

WHAT IS DELIBERATELY NOT GUARDED HERE: the default stays OFF. Turning the guard
on unconditionally would change the behaviour of every existing config carrying
a primary_structure family, which is a decision about shipped defaults. The
third test below pins that default, so flipping it becomes a deliberate act
rather than a side effect.
"""
from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.spec import (
    ROLE_PRIMARY_STRUCTURE,
    ROLE_SKIN_LATTICE,
    StructureSpec,
    WebFamily,
)
from aircraft_core.validate import TopologyError
from stub_ops import PolygonShape, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES = REPO_ROOT / "tools" / "make_fixtures.py"


def _mf():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_require_primary", MAKE_FIXTURES)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


mf = _mf()

NACA = "2412"
N_POINTS = 240
CHORD_MM = 150.0
WALL_MM = 0.42
SKIN_MARGIN_MM = 0.8608

# 0.65c is the station recorded as unreachable and the measurement confirmed is
# unplaceable at a 0.42 mm line width. 0.25c is placeable.
REAR_SPAR_FRAC = 0.65
MAIN_SPAR_FRAC = 0.25


def _section():
    m, p, t = mf.parse_naca4(NACA)
    outline = mf.naca4_section_outline(m, p, t, N_POINTS)
    scaled = [(x * CHORD_MM, y * CHORD_MM) for x, y in outline]
    xs = [q[0] for q in scaled]
    ys = [q[1] for q in scaled]
    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    return PolygonShape(tuple((x - cx, y - cy) for x, y in scaled))


def _spec(require: bool, role: str = ROLE_PRIMARY_STRUCTURE,
          position: float = REAR_SPAR_FRAC) -> StructureSpec:
    return StructureSpec(
        gap_ratio=0.05,
        skin_margin_mm=SKIN_MARGIN_MM,
        root_solid_layers=0,
        require_primary_structure=require,
        families=(WebFamily(
            name="spar_rear",
            web_position=position,
            web_depth_mm=None,      # FULL DEPTH -- a real tie, not a stiffener
            role=role,
        ),),
    )


class RequirePrimaryStructureTests(unittest.TestCase):
    def _generate(self, spec):
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, _section())
        return StructureGenerator().generate(
            frame, spec, WALL_MM,
            LayerContext(layer_index=5, print_z=1.2),
            safety_offset_mm=WALL_MM,
        )

    def test_unplaceable_spar_is_silently_omitted_when_guard_is_off(self):
        """The measured hazard itself, pinned so the test below means
        something. Off, the generator omits the rear spar and returns a
        perfectly valid result -- no exception, no intents."""
        result = self._generate(_spec(require=False))
        self.assertEqual(result.intents, [])
        self.assertTrue(result.omitted, "expected the rear spar to be omitted")
        self.assertTrue(
            any("spar_rear" in str(o.family_name) for o in result.omitted),
            "the omission should name the family",
        )

    def test_guard_turns_that_silence_into_a_failure(self):
        with self.assertRaises(TopologyError) as ctx:
            self._generate(_spec(require=True))
        exc = ctx.exception
        # The invariant's three required facts.
        self.assertEqual(exc.layer_index, 5)
        self.assertAlmostEqual(exc.print_z, 1.2, places=6)
        msg = str(exc)
        self.assertIn("primary structure omitted", msg)
        self.assertIn("spar_rear", msg)
        # It must say what to DO, not merely that something failed -- the
        # line-width lever is the strongest one and is the whole finding.
        self.assertIn("line width", msg)

    def test_default_is_off(self):
        """Pinned deliberately. Flipping this default changes every existing
        config that carries a primary_structure family, so it must be a
        decision rather than a side effect."""
        self.assertFalse(StructureSpec().require_primary_structure)

    def test_guard_ignores_skin_lattice_omissions(self):
        """A skin-lattice member omitted at the same unplaceable station must
        NOT fail, guard on or off. Omitting a stiffener is the correct trade;
        the guard is about load paths only."""
        spec = _spec(require=True, role=ROLE_SKIN_LATTICE)
        result = self._generate(spec)          # must not raise
        self.assertEqual(result.intents, [])
        self.assertTrue(result.omitted)

    def test_a_placeable_primary_member_passes_with_the_guard_on(self):
        """0.25c is inside the feasible band, so the guard must stay quiet --
        otherwise it would be failing everything rather than failing correctly."""
        result = self._generate(_spec(require=True, position=MAIN_SPAR_FRAC))
        self.assertEqual(len(result.intents), 1)
        self.assertEqual(result.intents[0].family_name, "spar_rear")


if __name__ == "__main__":
    unittest.main()
