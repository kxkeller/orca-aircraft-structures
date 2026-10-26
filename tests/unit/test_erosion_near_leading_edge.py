"""Real polygon-boolean regression for the layer 12 in-host failure of
``presets/plugin/skin_lattice_only_config.json`` on
``tests/fixtures/airfoil/naca2412_c60_s60.stl`` (Z=2.60 mm).

WHY THIS FILE USES SHAPELY, EXPLICITLY FLAGGED: ``tests/unit/stub_ops.py``
is deliberately exact-arithmetic but narrow — its own module docstring says
it "must never be treated like [a real geometry kernel]", and it does not
attempt general multi-notch erosion/diff for an arbitrary many-slot lattice
(only hand-built two-notch dataclasses). Diagnosing this failure required an
actual polygon-boolean backend able to run ``StructureGenerator`` ->
``SlotBuilder`` -> ``GeometryOps.diff`` -> the erosion connectivity check on
47 real candidate slots, which StubOps cannot represent. ``shapely`` (2.1.2)
is already present in this environment; it is used here as a TEST-ONLY
diagnostic/regression backend, never imported by ``aircraft_core`` or the
shipped plugin bundle (the plugin has no third-party dependency). This
module is skipped outright if ``shapely`` is ever unavailable, so it can
never become a hard requirement for the rest of the suite.

THE JOIN STYLE: an initial round-join offset reproduction did NOT split at
the real ``safety_offset_mm`` (0.42 mm) — it only pinched a sliver at
0.46 mm, a seam left open rather than closed. A mitre-join offset (mitre
limit 3.0, shapely's ``join_style=2`` — the ClipperLib default this
project's own source pin could not be re-verified against offline) DOES split at exactly 0.42 mm, at bounds
matching the in-host failure, which ablation traces to a single candidate
(x=-29.0005 mm, ~1.0 mm from the leading-edge apex, +Y, 1.5 mm deep). This
file pins that reproduction; it is an empirical match to the observed
in-host failure, not a verified statement about Clipper's own join
parameters.
"""
import importlib.util
import json
import sys
import unittest
from pathlib import Path

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.slot import SlotBuilder

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"
STRUCTURAL_ANALYSIS_SCRIPT = REPO_ROOT / "tools" / "structural_analysis.py"
CONFIG_PATH = REPO_ROOT / "presets" / "plugin" / "skin_lattice_only_config.json"

TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

# Shared TEST/TOOL-ONLY real polygon-boolean backend (see that module's own
# docstring for why this is ONE shared implementation, not a copy local to this
# file: two measurements of the same quantity must agree).
from shapely_ops import HAVE_SHAPELY as _HAVE_SHAPELY  # noqa: E402
from shapely_ops import MiterShapelyOps as _MiterShapelyOps  # noqa: E402


def _load_module(name, script_path):
    import sys

    spec = importlib.util.spec_from_file_location(name, script_path)
    module = importlib.util.module_from_spec(spec)
    # Registered in sys.modules BEFORE exec_module: structural_analysis.py
    # defines dataclasses whose field-type resolution needs
    # ``sys.modules[cls.__module__]`` to exist -- see
    # tests/unit/test_structural_analysis.py's own loader for the same fix.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@unittest.skipUnless(_HAVE_SHAPELY, "shapely not installed; see module docstring")
class LayerTwelveRealBooleanRegressionTests(unittest.TestCase):
    """Full generate -> slot -> diff -> erosion pipeline, real geometry,
    exact layer/Z/config of the real in-host failure."""

    @classmethod
    def setUpClass(cls):
        cls.sa = _load_module(
            "structural_analysis_under_test_erosion_le", STRUCTURAL_ANALYSIS_SCRIPT
        )
        cls.ops = _MiterShapelyOps()
        section = cls.sa.build_airfoil_section("2412", 60.0, 60.0)
        outline = [tuple(p) for p in section.polygon.points]
        cls.poly = cls.ops.polygon_from_points(outline)
        cls.frame = LayerFrame.from_polygon(cls.ops, cls.poly)
        cap_config = json.loads(CONFIG_PATH.read_text())["config"][0]["cap_config"]
        cls.spec = cls.sa.build_spec_from_cap_config(cap_config)
        cls.wall_width_mm = 0.42

    def _replacement(self):
        gen = StructureGenerator()
        ctx = LayerContext(layer_index=12, print_z=2.6)
        result = gen.generate(
            self.frame,
            self.spec,
            self.wall_width_mm,
            ctx,
            safety_offset_mm=self.wall_width_mm,
        )
        builder = SlotBuilder()
        built = [builder.build(i, self.spec.overshoot_mm) for i in result.intents]
        slots = [self.ops.polygon_from_points(b.points) for b in built]
        pieces = self.ops.diff(self.poly, slots)
        self.assertEqual(len(pieces), 1, "cut should still be one connected piece")
        return pieces[0], result

    def test_fixed_generator_produces_a_layer_that_survives_erosion(self):
        """The near-apex candidate is omitted for
        thinness, and the resulting single body layer survives the real erosion
        connectivity check at the layer's own safety_offset_mm (0.42 mm) with a
        real mitre-join boolean backend — the same backend that splits
        the layer into 2 pieces at this offset when the near-apex candidate is
        kept. That failing case is not re-asserted here, since reproducing it
        would require duplicating removed generator logic rather than
        exercising the shipped code."""
        rep, result = self._replacement()
        pieces = self.ops.offset(rep, -self.wall_width_mm)
        self.assertEqual(
            len(pieces),
            1,
            "layer 12 should survive the {:.2f} mm safety-offset erosion "
            "as one connected piece after the thin-section fix".format(self.wall_width_mm),
        )
        near_apex = [
            o
            for o in result.omitted
            if abs(o.x_mm - (-29.000524968240548)) < 1e-3
        ]
        self.assertTrue(
            near_apex, "expected the near-apex candidate to be omitted for thinness"
        )
        for o in near_apex:
            self.assertTrue(o.reason.startswith("thin section:"))


if __name__ == "__main__":
    unittest.main()
