"""Acceptance: the CAD MODEL determines the spar socket, with zero config edits.

THE REQUIREMENT THIS PINS. The input model must deterministically dictate the
part geometry. If the CAD contains a spar channel, its exact position,
diameter, orientation and taper/trajectory come from the model. Config is for
slicing/structural policy; it must not independently specify where modeled
aircraft features are. The regression test for that: move/change the modeled
spar channel, slice again, and prove the generated socket follows it exactly.

WHY THIS IS AN INTEGRATION TEST AND NOT A UNIT TEST. The unit tests in
``tests/unit/test_m18_model_spar_socket.py`` feed hand-built ``DetectedChannel``
values straight into the generator, which pins the generator's contract but
assumes the detection. The property that actually matters end-to-end is that a
change made in CAD propagates through mesh cross-sectioning, interior-loop
classification, channel detection and socket synthesis without anyone touching
config. That needs the real STL fixtures and the whole chain, which is what this
file exercises.

THE CONTROL THAT MAKES IT MEANINGFUL: every case below is generated from ONE
``StructureSpec``, built once, carrying NO spar geometry (``spar_diameter_mm``
is ``None``). If the socket still lands in three different places for three
different models, the model -- and only the model -- put it there.

Real fixtures are the point of this file: three defect classes in
``mesh_section`` are only reachable with real meshes, not the analytic unit
tests -- station-plane vertices, over-eager short-segment filtering, and a
silent empty section one ULP below a bucket boundary.
"""
from __future__ import annotations

import json
import os
import sys
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
for _p in (REPO_ROOT, os.path.join(REPO_ROOT, "tools")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import (  # noqa: E402
    DetectedChannel,
    LayerContext,
    StructureGenerator,
)
from aircraft_core.mesh_section import MeshSectioner  # noqa: E402

try:
    from shapely_ops import HAVE_SHAPELY, MiterShapelyOps
except ImportError:  # pragma: no cover - shapely absent
    HAVE_SHAPELY = False

FIXTURES = os.path.join(REPO_ROOT, "tests", "fixtures", "wing")

# Deliberately NOT a station plane and NOT a bucket boundary: the point here is
# the model->socket correspondence, and the degenerate-Z cases have their own
# dedicated regressions in tests/unit/test_mesh_section.py.
SECTION_Z_MM = 61.7
WALL_MM = 0.42

# The clearance is POLICY and therefore legitimately comes from config; the
# position and diameter are GEOMETRY and must not.
CLEARANCE_MM = 0.15


def _spec_without_spar_geometry():
    """One spec, shared by every case, declaring no spar geometry at all.

    ``spar_diameter_mm=None`` is the load-bearing part of this fixture: it means
    the config physically cannot be the source of the socket's position or size,
    so any correspondence observed below has to have come from the mesh.
    """
    from aircraft_core.spec import StructureSpec, WebFamily

    return StructureSpec(
        gap_ratio=0.05,
        skin_margin_mm=0.8608,
        overshoot_mm=0.05,
        root_solid_layers=5,
        spar_diameter_mm=None,
        spar_clearance_mm=CLEARANCE_MM,
        spar_neck_width_mm=1.2,
        allow_config_spar_fallback=False,
        families=(
            WebFamily(
                name="lattice_plus_upper", role="skin_lattice",
                rib_pitch_chord_frac=0.287, web_position=0.35,
                web_depth_mm=1.5, entry_lower=False, z_period_mm=64.6,
                z_drift_chord_frac_per_mm=0.016667,
            ),
        ),
    )


@unittest.skipUnless(HAVE_SHAPELY, "shapely is required for the real boolean backend")
class ModelDeterminesTheSparSocketTests(unittest.TestCase):
    """Move or resize the channel in CAD; the socket must follow, config fixed."""

    @classmethod
    def setUpClass(cls):
        cls.spec = _spec_without_spar_geometry()
        cls.ops = MiterShapelyOps()
        cls.generator = StructureGenerator()

    def _socket_for(self, stl_name):
        """Run the REAL chain: STL -> section -> detect -> generate.

        Returns ``(socket_intent, detected_channel)``.
        """
        from m18_mesh_probe import read_binary_stl

        path = os.path.join(FIXTURES, stl_name)
        if not os.path.isfile(path):
            self.skipTest("fixture {!r} not present".format(stl_name))
        verts, tris = read_binary_stl(path)
        loops = MeshSectioner(verts, tris).section_at(SECTION_Z_MM)

        outer = [lp for lp in loops if not lp.is_interior]
        inner = [lp for lp in loops if lp.is_interior]
        self.assertEqual(
            len(outer), 1,
            "expected exactly one outer boundary at Z={} in {}".format(
                SECTION_Z_MM, stl_name))
        self.assertEqual(
            len(inner), 1,
            "expected exactly one interior loop (the spar channel) at Z={} in "
            "{}; got {}".format(SECTION_Z_MM, stl_name, len(inner)))

        channel = DetectedChannel(
            center_x_mm=inner[0].centroid[0],
            center_y_mm=inner[0].centroid[1],
            diameter_mm=inner[0].equivalent_diameter_mm,
        )
        polygon = self.ops.polygon_from_points(outer[0].points)
        frame = LayerFrame.from_polygon(self.ops, polygon)
        result = self.generator.generate(
            frame, self.spec, WALL_MM,
            LayerContext(layer_index=300, print_z=SECTION_Z_MM,
                         detected_channels=(channel,)),
            safety_offset_mm=WALL_MM,
        )
        sockets = [i for i in result.intents
                   if getattr(i, "socket_radius_mm", None) is not None]
        self.assertEqual(
            len(sockets), 1,
            "expected exactly one spar socket for {}".format(stl_name))
        return sockets[0], channel

    def test_socket_sits_exactly_where_the_model_puts_the_channel(self):
        socket, channel = self._socket_for("m18_wing_spar10_at0.30.stl")
        self.assertAlmostEqual(
            socket.start[0], channel.center_x_mm, places=9,
            msg="the socket's chordwise station must BE the detected "
                "centroid, not a config station")

    def test_socket_radius_is_the_detected_diameter_plus_config_clearance(self):
        socket, channel = self._socket_for("m18_wing_spar10_at0.30.stl")
        self.assertAlmostEqual(
            socket.socket_radius_mm,
            channel.diameter_mm / 2.0 + CLEARANCE_MM, places=9,
            msg="radius must be the MODEL's diameter plus the CONFIG's "
                "clearance -- geometry from the model, policy from config")

    def test_moving_the_channel_in_cad_moves_the_socket_by_the_same_amount(self):
        """THE CORE PROPERTY. Two models differing only in
        spar station, one unchanged config, and the socket must track the model
        exactly."""
        near, near_ch = self._socket_for("m18_wing_spar10_at0.30.stl")
        far, far_ch = self._socket_for("m18_wing_spar10_at0.45.stl")

        model_shift = far_ch.center_x_mm - near_ch.center_x_mm
        socket_shift = far.start[0] - near.start[0]

        self.assertGreater(
            abs(model_shift), 15.0,
            "the two fixtures must genuinely differ, or this proves nothing")
        self.assertAlmostEqual(
            socket_shift, model_shift, places=9,
            msg="socket moved {:.6f} mm but the modelled channel moved "
                "{:.6f} mm".format(socket_shift, model_shift))

    def test_changing_the_channel_diameter_changes_only_the_radius(self):
        big, big_ch = self._socket_for("m18_wing_spar10_at0.30.stl")
        small, small_ch = self._socket_for("m18_wing_spar8_at0.30.stl")

        self.assertAlmostEqual(
            big.socket_radius_mm - small.socket_radius_mm,
            (big_ch.diameter_mm - small_ch.diameter_mm) / 2.0, places=9)
        # Same station in CAD, so the socket must not have wandered. The
        # tolerance is set by MESH FACETING, not by the code under test: both
        # channels are cut as polygons with the same vertex count, so a 10 mm
        # and an 8 mm bore have different chord lengths and their polygon
        # centroids differ from the true circle centre by slightly different
        # amounts. Measured difference is 6.0e-5 mm -- 60 nanometres, four
        # orders of magnitude below the 0.42 mm extrusion width. A tighter
        # assertion would be pinning the fixture generator's tessellation, not
        # the property this test exists to prove.
        self.assertAlmostEqual(big.start[0], small.start[0], delta=1e-3)

    def test_the_shared_config_declares_no_spar_geometry_whatsoever(self):
        """The control. If this ever gains a spar diameter, every assertion
        above stops proving that the MODEL is the source of truth."""
        self.assertIsNone(self.spec.spar_diameter_mm)
        self.assertFalse(self.spec.allow_config_spar_fallback)


@unittest.skipUnless(HAVE_SHAPELY, "shapely is required for the real boolean backend")
class InstalledConfigCarriesNoSparGeometryTests(unittest.TestCase):
    """The config actually installed for slicing must not carry spar geometry.

    Declaring `spar_diameter_mm`/`spar_station` in the live config would
    duplicate information that lives in the model (and let the two disagree).
    This guards against that: it is easy to reintroduce a config-declared spar
    while debugging and never take it out again, which would silently restore
    config-driven spar placement.
    """

    def test_live_config_has_no_spar_diameter(self):
        # The tracked mirror of the installed config is always checked; the
        # host's own installed copy is checked too when it is present.
        paths = [os.path.join(REPO_ROOT, "presets", "plugin",
                              "live_host_config.json")]
        installed = os.path.join(REPO_ROOT, "host", "data", "orca_plugins",
                                 "config.json")
        if os.path.isfile(installed):
            paths.append(installed)
        for path in paths:
            with self.subTest(config=os.path.relpath(path, REPO_ROOT)):
                with open(path) as fh:
                    cap = json.load(fh)["config"][0]["cap_config"]
                self.assertIsNone(
                    cap.get("spar_diameter_mm"),
                    "the installed config declares spar geometry; the "
                    "model must be the source of truth for spar geometry")
                self.assertFalse(
                    cap.get("allow_config_spar_fallback", False),
                    "the installed config enables the debugging fallback; that is for "
                    "debugging the socket primitive, not for normal operation")


if __name__ == "__main__":
    unittest.main()
