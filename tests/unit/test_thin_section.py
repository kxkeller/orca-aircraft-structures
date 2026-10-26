"""Thin-section omission tests (aircraft_core/generator.py).

The exact failing case record: a full-depth web near the leading edge of a NACA
2412 section (x=-29.0 mm on a 60 mm chord centred at 0) passes the existing
chord-extent and skin-margin omission checks -- the local thickness there is
about 2.6 mm, comfortably more than the 1.0 mm skin_margin_mm requested -- yet
the real in-host validator's safety-offset erosion still disconnected the
material there.

These tests reuse tools/make_fixtures.py's NACA math (no second airfoil
implementation) and StubOps' exact Sutherland-Hodgman clipping (the same
backend test_generator.py and test_validate.py already trust for
non-rectangular subjects) to reproduce the analytical station the real failure
was measured at, and prove the NEW "thin section" omission catches it
deterministically -- see aircraft_core/generator.py's module docstring ("change
-- thin-section omission") for the full derivation.
"""
import importlib.util
import unittest
from pathlib import Path

from aircraft_core.frame import (
    DEFAULT_PROBE_WIDTH_MM,
    LayerFrame,
    local_lower_worst_case,
    local_upper_worst_case,
)
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.spec import StructureSpec
from stub_ops import PolygonShape, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_thin_section", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_mf = _load_make_fixtures_module()


def _naca2412_polygon():
    """Same 60 mm chord / 60 mm span NACA 2412 section as test_generator.py's /
    test_validate.py's own ``_naca2412_polygon()`` -- the failure's exact
    fixture parameters."""
    m, p, t = _mf.parse_naca4("2412")
    _triangles, outline_mm, bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm)), bbox


# Matches: the real failure's web landed at chord station x=-29.0 mm on a
# section whose leading edge sits at x~=-30.0 mm.
TARGET_X_MM = -29.0

SKIN_MARGIN_MM = 1.0  # "the 1.0 mm skin margin does fit" -- finding text.
OUTER_WALL_MM = 0.42  # matches the recorded failure's resolved wall width.
GAP_RATIO = 0.5


class ThinSectionOmissionTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.polygon, _bbox = _naca2412_polygon()
        self.frame = LayerFrame.from_polygon(self.ops, self.polygon)
        # Derive web_position from the target chord station, exactly like
        # docs' own fixtures derive placement from geometry rather than
        # hard-coding a fraction that could silently stop meaning what it
        # says if the fixture ever changes.
        self.web_position = (TARGET_X_MM - self.frame.min_x) / self.frame.width
        self.layer = LayerContext(layer_index=29, print_z=6.0)

    def _spec(self, web_position, skin_margin_mm=SKIN_MARGIN_MM):
        return StructureSpec(
            gap_ratio=GAP_RATIO, skin_margin_mm=skin_margin_mm, web_position=web_position
        )

    def test_fixture_still_demonstrates_margin_fits_but_thin(self):
        """Independent confirmation (mirrors test_airfoil_capability.py's
        ThinTrailingEdgeTests pattern): the target station's local
        thickness comfortably exceeds skin_margin_mm on its own -- this is
        NOT the "skin margin" omission, so a passing test here
        proves the thin-section check, not the skin-margin one."""
        width_mm = GAP_RATIO * OUTER_WALL_MM
        y_max = local_upper_worst_case(
            self.ops, self.polygon, TARGET_X_MM, width_mm, DEFAULT_PROBE_WIDTH_MM
        )
        y_min = local_lower_worst_case(
            self.ops, self.polygon, TARGET_X_MM, width_mm, DEFAULT_PROBE_WIDTH_MM
        )
        local_thickness = y_max - y_min
        self.assertGreater(
            local_thickness,
            SKIN_MARGIN_MM,
            "fixture no longer demonstrates a station where the margin "
            "fits on its own -- this test would no longer exercise the "
            "NEW thin-section path (it would just be the old skin-margin "
            "omission).",
        )

    def test_thin_section_web_is_omitted_not_fatal(self):
        spec = self._spec(self.web_position)
        result = StructureGenerator().generate(
            self.frame,
            spec,
            OUTER_WALL_MM,
            self.layer,
            safety_offset_mm=OUTER_WALL_MM,
        )
        self.assertEqual(result.intents, [])
        self.assertEqual(len(result.omitted), 1)
        omission = result.omitted[0]
        self.assertTrue(
            omission.reason.startswith("thin section:"),
            "expected a 'thin section:' omission, got: {!r}".format(omission.reason),
        )
        self.assertAlmostEqual(omission.x_mm, TARGET_X_MM, places=6)

    def test_control_station_at_mid_chord_builds_normally(self):
        """Same spec, a mid-chord station: proves the new check is
        station-specific, not a blanket regression -- a web comfortably
        clear of the leading edge still builds."""
        spec = self._spec(web_position=0.5)
        result = StructureGenerator().generate(
            self.frame,
            spec,
            OUTER_WALL_MM,
            self.layer,
            safety_offset_mm=OUTER_WALL_MM,
        )
        self.assertEqual(len(result.intents), 1)
        self.assertEqual(result.omitted, [])

    def test_thin_section_never_fires_below_the_scope_gate(self):
        """When skin_margin_mm < 2 * safety_offset_mm, the thin-section
        check must stay OFF entirely (see the module docstring's "SCOPE
        GATE") -- that regime is the pre-existing TopologyValidator
        misconfiguration precondition and must keep failing closed through
        the validator's own erosion message, unchanged. The web here is
        BUILT (not omitted for thinness) even though the halo reading at
        this same near-leading-edge station would otherwise be well short
        of 2 * safety_offset_mm.

        The PUBLIC ``StructureGenerator.generate()`` entry point calls
        ``spec.validate(safety_offset_mm=...)`` before placing any web, so
        ``skin_margin_mm=0.1`` at ``safety_offset_mm=0.42`` raises
        ``SpecError`` immediately and never reaches this test's subject (the
        thin-section gate's SCOPE GATE behaviour). That is a DIFFERENT
        invariant: this test is about ``_generate_family``'s own internal gate
        logic, so it drives ``_generate_family`` directly (bypassing
        ``spec.validate()``, as ``test_node_merge.py`` calls
        ``_merge_cross_family_nodes`` directly to pin an internal mechanism in
        isolation). The guard's own failure path is tested in
        ``tests/unit/test_spec.py``. This test keeps the scope-gate behaviour
        covered in case a future change decouples the gate from the guard.
        """
        spec = self._spec(self.web_position, skin_margin_mm=0.1)
        family = spec.effective_families()[0]
        intents, omitted = StructureGenerator()._generate_family(
            self.frame,
            spec,
            family,
            0,
            OUTER_WALL_MM,
            self.layer,
            OUTER_WALL_MM,
            spec.skin_margin_mm,
        )
        self.assertEqual(len(intents), 1)
        self.assertEqual(omitted, [])

    def test_halo_probe_near_the_chord_extent_is_clamped_not_fatal(self):
        """A station close enough to the leading edge that the UNCLAMPED
        erosion halo (slot width + 2 * safety_offset_mm either side) would
        reach past frame.min_x must not raise -- the clamp (and its
        try/except fallback belt-and-suspenders) documented in
        generator.py must absorb this, not surface a NEW fail-closed path.
        Verified directly: an unclamped halo probe at this exact station
        DOES raise ValueError from local_span (confirmed while developing
        this fixture), so a passing test here is proving the clamp branch
        specifically, not merely that "most stations work"."""
        target_x = -29.5
        web_position = (target_x - self.frame.min_x) / self.frame.width
        spec = self._spec(web_position)
        result = StructureGenerator().generate(
            self.frame,
            spec,
            OUTER_WALL_MM,
            self.layer,
            safety_offset_mm=OUTER_WALL_MM,
        )
        self.assertEqual(result.intents, [])
        self.assertEqual(len(result.omitted), 1)
        self.assertTrue(result.omitted[0].reason.startswith("thin section:"))

    def test_default_safety_offset_mm_falls_back_to_wall_width(self):
        """``safety_offset_mm`` is optional -- omitting it entirely must
        reproduce passing ``safety_offset_mm=outer_wall_line_width_mm``
        explicitly, since that is what every existing call site (before
        this parameter existed) effectively assumed."""
        spec = self._spec(self.web_position)
        with_kw = StructureGenerator().generate(
            self.frame, spec, OUTER_WALL_MM, self.layer, safety_offset_mm=OUTER_WALL_MM
        )
        without_kw = StructureGenerator().generate(
            self.frame, spec, OUTER_WALL_MM, self.layer
        )
        self.assertEqual(with_kw.intents, without_kw.intents)
        self.assertEqual(
            [o.reason for o in with_kw.omitted], [o.reason for o in without_kw.omitted]
        )


class NearLeadingEdgeConvergenceRegressionTests(unittest.TestCase):
    """A real in-host slice of ``presets/plugin/skin_lattice_only_config.json``
    on ``tests/fixtures/airfoil/naca2412_c60_s60.stl`` failed at layer 12
    (Z=2.60 mm). A thin-section halo check using only the ordinary probe window
    would pass this exact station by
    0.0018 mm (``remaining_halo_mm`` 0.8418 mm against an 0.84 mm threshold)
    because its probe window (slot width + 2 * safety_offset_mm, centred on the
    candidate) reached only to x=-29.431 mm -- short of where a real
    polygon-boolean backend (see
    ``tests/unit/test_erosion_near_leading_edge.py``, mitre-join offset) shows
    the layer actually pinches, x=[-29.581, -29.431] mm, on the leading-edge
    side. This class pins the behaviour that prevents it (the halo widens to reach a
    chord extremity when the ordinary window already comes within one
    candidate-depth of it) at the generator level, independent of any boolean
    backend -- see the ``aircraft_core.generator`` module docstring for the
    full derivation.
    """

    TARGET_X_MM = -29.000524968240548  # matches the real in-host failure exactly.
    SKIN_MARGIN_MM = 0.8608  # shipped two-sided default.
    WEB_DEPTH_MM = 1.5  # stiffener family depth.
    OUTER_WALL_MM = 0.42

    def setUp(self):
        self.ops = StubOps()
        self.polygon, _bbox = _naca2412_polygon()
        self.frame = LayerFrame.from_polygon(self.ops, self.polygon)
        self.web_position = (
            self.TARGET_X_MM - self.frame.min_x
        ) / self.frame.width
        self.layer = LayerContext(layer_index=12, print_z=2.6)

    def _spec(self):
        return StructureSpec(
            gap_ratio=0.05,
            skin_margin_mm=self.SKIN_MARGIN_MM,
            web_position=self.web_position,
            web_depth_mm=self.WEB_DEPTH_MM,
        )

    def test_near_apex_full_depth_candidate_is_omitted_for_thinness(self):
        """Without the widened halo this candidate would BUILD (the halo check
        passes it by 0.0018 mm) -- see this class's own docstring. It must be
        OMITTED, not fatal, and the reason must stay in the "thin
        section:"/"skin margin:" prefix class the tip_shutdown
        semantics key off (``plugin/aircraft_structures/capability.py``'s
        ``thinness_prefixes``)."""
        spec = self._spec()
        result = StructureGenerator().generate(
            self.frame,
            spec,
            self.OUTER_WALL_MM,
            self.layer,
            safety_offset_mm=self.OUTER_WALL_MM,
        )
        self.assertEqual(result.intents, [])
        self.assertEqual(len(result.omitted), 1)
        omission = result.omitted[0]
        self.assertTrue(
            omission.reason.startswith("thin section:"),
            "expected a 'thin section:' omission, got: {!r}".format(omission.reason),
        )
        self.assertAlmostEqual(omission.x_mm, self.TARGET_X_MM, places=6)

    def test_mid_chord_full_depth_candidate_unaffected(self):
        """Same spec, a mid-chord station far from any chord extremity: proves
        widening is near-extremity-scoped, not a blanket change to every
        full-depth candidate -- widening the halo by the candidate's own depth EVERYWHERE would break
        this exact shape of case."""
        web_position = 0.5
        spec = StructureSpec(
            gap_ratio=0.05,
            skin_margin_mm=self.SKIN_MARGIN_MM,
            web_position=web_position,
            web_depth_mm=self.WEB_DEPTH_MM,
        )
        result = StructureGenerator().generate(
            self.frame,
            spec,
            self.OUTER_WALL_MM,
            self.layer,
            safety_offset_mm=self.OUTER_WALL_MM,
        )
        self.assertEqual(len(result.intents), 1)
        self.assertEqual(result.omitted, [])


if __name__ == "__main__":
    unittest.main()
