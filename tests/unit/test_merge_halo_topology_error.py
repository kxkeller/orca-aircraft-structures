"""Regression: the merge-halo breach must fail as a TopologyError.

WHAT BROKE, AND HOW IT WAS FOUND. The weighted pitch search
(``tools/size_wing.py``) proposed ``rib_pitch_chord_frac`` 0.0500 against the
shipped 0.0650. That candidate passed the full real-boolean sweep on seven
fixtures -- including the 1995-layer tapered/twisted wing it was tuned on --
and failed on the eighth, the tapered r60/t30 fixture, at layers 278, 280, 288
and 295.

The failure itself was correct and is not what this test guards. A finer pitch
puts stations close enough that a node merge unions a long run of them into one
very wide slot (20.87 mm at layer 278), whose erosion halo leaves 0.8334 mm of
skin against the 0.8400 mm required -- a breach by 0.0066 mm, a
hair's-breadth breach.
Refusing to merge is right.

The SHAPE of the refusal matters: it must be a ``TopologyError``, not a bare
``ValueError``.

In-host, a bare ``ValueError`` would also be handled: ``capability.py``'s
``except (ValueError, ArithmeticError)`` handler around
``generator.generate()`` translates that ``ValueError`` into a
``TopologyError`` carrying the authoritative layer index and Z, so a user
already sees a properly attributed diagnostic on the structured path. Raising
``TopologyError`` directly does not alter what a user sees.

It matters for callers that use ``aircraft_core`` DIRECTLY, which have no
such translation: ``tools/verify_full_sweep.py``,
``tools/verify_full_sweep_varying.py`` and the ``tools/size_wing.py`` search
built on them would classify this deliberate fail-closed refusal as ``UNEXPECTED
(not a clean TopologyError)`` -- i.e. as a crash -- which is how the pitch
search would report its only invariant rejection.

This test pins the exception TYPE and the layer/Z attributes. It deliberately
does NOT assert the message wording beyond the merge-specific fragments, so
that improving the prose does not break it.
"""
from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.spec import StructureSpec, WebFamily
from aircraft_core.validate import TopologyError
from stub_ops import PolygonShape, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_merge_halo_topology_error", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mf = _load_make_fixtures_module()

NACA = "2412"
N_POINTS = 120
ROOT_CHORD_MM = 60.0
TIP_CHORD_MM = 30.0
SPAN_MM = 60.0


def _tapered_polygon_at(z_mm: float):
    """Same construction as ``tools/verify_full_sweep_varying.py``'s
    ``_outline_at_z_taper`` (pure taper: pivot_frac_x=0.0, no rotation, no
    dx), built from ``make_fixtures``'s own primitives rather than
    re-derived -- matching ``test_far_surface_slope.py``'s helper.
    """
    m, p, t = mf.parse_naca4(NACA)
    outline_unit = mf.naca4_section_outline(m, p, t, N_POINTS)

    def chord_at(zz):
        return ROOT_CHORD_MM + (TIP_CHORD_MM - ROOT_CHORD_MM) * (zz / SPAN_MM)

    root_scaled = mf.scale_naca_outline(outline_unit, chord_at(0.0))
    rxmin, rymin, rxmax, rymax = mf.section_bbox_2d(root_scaled)
    cx, cy = ((rxmin + rxmax) / 2.0, (rymin + rymax) / 2.0)

    scaled = mf.scale_naca_outline(outline_unit, chord_at(z_mm))
    return PolygonShape(tuple((x - cx, y - cy) for x, y in scaled))


def _four_family_spec(pitch_chord_frac: float) -> StructureSpec:
    """The shipped default's four-family skin lattice, with the pitch as the
    only variable -- upper/lower entry, plus/minus drift, everything else at
    the shipped values."""
    families = []
    for name, entry_lower, sign in (
        ("lattice_plus_upper", False, +1.0),
        ("lattice_minus_upper", False, -1.0),
        ("lattice_plus_lower", True, +1.0),
        ("lattice_minus_lower", True, -1.0),
    ):
        families.append(
            WebFamily(
                name=name,
                web_position=0.35,
                web_depth_mm=1.5,
                entry_lower=entry_lower,
                z_period_mm=28.0,
                rib_pitch_chord_frac=pitch_chord_frac,
                z_drift_chord_frac_per_mm=sign * 0.016667,
            )
        )
    return StructureSpec(
        gap_ratio=0.05,
        skin_margin_mm=0.8608,
        root_solid_layers=5,
        families=tuple(families),
    )


class MergeHaloBreachRaisesTopologyErrorTests(unittest.TestCase):
    """The four traced layers from the rejected 0.0500 c candidate."""

    OUTER_WALL_MM = 0.42
    FAILING_LAYERS = ((278, 55.8), (280, 56.2), (288, 57.8), (295, 59.2))

    def _generate(self, layer_index: int, z_mm: float, pitch: float):
        ops = StubOps()
        polygon = _tapered_polygon_at(z_mm)
        frame = LayerFrame.from_polygon(ops, polygon)
        return StructureGenerator().generate(
            frame,
            _four_family_spec(pitch),
            self.OUTER_WALL_MM,
            LayerContext(layer_index=layer_index, print_z=z_mm),
            safety_offset_mm=self.OUTER_WALL_MM,
        )

    def test_breach_raises_topology_error_naming_layer_and_z(self):
        raised = 0
        for layer_index, z_mm in self.FAILING_LAYERS:
            try:
                self._generate(layer_index, z_mm, 0.050)
            except TopologyError as exc:
                raised += 1
                # The invariant's three required facts.
                self.assertEqual(exc.layer_index, layer_index)
                self.assertAlmostEqual(exc.print_z, z_mm, places=6)
                self.assertIn("node merge", str(exc))
                # Pin the HALO raise specifically, not merely "some merge
                # error" -- there are other merge-path failure modes.
                self.assertIn("erosion halo", str(exc))
                # Formatted header, so the diagnostic is actionable as text.
                self.assertIn(
                    "Aircraft structure invalid at layer {}".format(layer_index),
                    str(exc),
                )
            except ValueError as exc:  # pragma: no cover - the regression
                self.fail(
                    "layer {} raised a bare ValueError, the exact regression "
                    "this test exists to prevent: {}".format(layer_index, exc)
                )
        self.assertGreater(
            raised,
            0,
            "no layer reached the merge-halo breach; the fixture or the "
            "candidate pitch no longer reproduces it, so this test is no "
            "longer guarding anything and must be re-derived rather than "
            "deleted",
        )

    def test_topology_error_is_not_a_value_error(self):
        """``capability.py`` routes ``TopologyError`` to its structured
        handler and everything else to a generic one, and
        ``verify_full_sweep*.py`` classifies anything that is not a
        ``TopologyError`` as UNEXPECTED. Both behaviours depend on these
        being distinct types, not merely on the message text."""
        self.assertFalse(issubclass(TopologyError, ValueError))

    def test_shipped_pitch_does_not_breach_on_the_same_layers(self):
        """The shipped 0.0650 c must still generate cleanly here -- this is
        what makes the test above a statement about the CANDIDATE pitch
        rather than about the fixture being broken."""
        for layer_index, z_mm in self.FAILING_LAYERS:
            try:
                self._generate(layer_index, z_mm, 0.065)
            except TopologyError as exc:  # pragma: no cover
                self.fail(
                    "shipped pitch 0.0650 c now breaches at layer {}: {}".format(
                        layer_index, exc
                    )
                )


if __name__ == "__main__":
    unittest.main()
