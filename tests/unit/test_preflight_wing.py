"""Tests for tools/preflight_wing.py (the pre-flight tool).

Fast, synthetic-geometry unit coverage for T1 (model-level checks), the
stand-up axis/root-end resolution, and the T2 fatal-alignment short-circuit in
T3, plus one small real end-to-end run (naca2412_wing_r150_t120_s120_w1.5.stl,
a tracked ~120 mm-span fixture, at a coarse --layer-height so the full pipeline
runs in seconds) verifying the tool produces a self-consistent report and never
prints an unqualified PASS. Skipped outright if shapely is unavailable,
matching every other real-pipeline test in this repo.
"""
import hashlib
import json
import os
import struct
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from shapely_ops import HAVE_SHAPELY  # noqa: E402

import preflight_wing as pw  # noqa: E402
from aircraft_core import alignment  # noqa: E402

FIXTURE_STL = REPO_ROOT / "tests" / "fixtures" / "wing" / "naca2412_wing_r150_t120_s120_w1.5.stl"
# A TRACKED, committed mirror of the live editable config
# (host/data/orca_plugins/config.json), not the live file itself --
# host/data/orca_plugins/config.json is untracked, so a test reading it
# directly would fail on a fresh checkout and could be silently gutted when
# the live config is retuned (same hazard test_m35_far_cap_mitre_clearance.py
# avoids with a frozen fixture). The mirror carries the fields
# ChannelRotationRegressionTest needs (spar_station 0.3,
# allow_config_spar_fallback False); drift from the live config does not
# break these tests, which need only A config with those two fields, not THE
# live one.
LIVE_CONFIG = REPO_ROOT / "presets" / "plugin" / "live_host_config.json"


def _write_binary_stl(path, triangles):
    """Minimal binary STL writer for synthetic T1 fixtures: ``triangles`` is
    a list of ((x,y,z) x3) tuples, normals are irrelevant to the reader
    under test."""
    with open(path, "wb") as fh:
        fh.write(b"\x00" * 80)
        fh.write(struct.pack("<I", len(triangles)))
        for tri in triangles:
            fh.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            for v in tri:
                fh.write(struct.pack("<3f", *v))
            fh.write(struct.pack("<H", 0))


def _box_triangles(x0, y0, z0, x1, y1, z1):
    """12 triangles for an axis-aligned box, consistently wound (winding
    correctness does not matter to the edge-count/manifold check under
    test, which is winding-agnostic by construction)."""
    v = {
        0: (x0, y0, z0), 1: (x1, y0, z0), 2: (x1, y1, z0), 3: (x0, y1, z0),
        4: (x0, y0, z1), 5: (x1, y0, z1), 6: (x1, y1, z1), 7: (x0, y1, z1),
    }
    faces = [
        (0, 1, 2), (0, 2, 3),  # bottom
        (4, 6, 5), (4, 7, 6),  # top
        (0, 5, 1), (0, 4, 5),  # -y
        (1, 6, 2), (1, 5, 6),  # +x
        (2, 7, 3), (2, 6, 7),  # +y
        (3, 4, 0), (3, 7, 4),  # -x
    ]
    return [(v[a], v[b], v[c]) for a, b, c in faces]


class T1ModelChecksTests(unittest.TestCase):
    def _read_via_tempfile(self, triangles):
        import tempfile
        fd, path = tempfile.mkstemp(suffix=".stl")
        os.close(fd)
        try:
            _write_binary_stl(path, triangles)
            ok, msg = pw._binary_stl_size_check(path)
            self.assertTrue(ok, msg)
            from m18_mesh_probe import read_binary_stl
            return read_binary_stl(path)
        finally:
            os.remove(path)

    def test_closed_box(self):
        verts, tris = self._read_via_tempfile(_box_triangles(0, 0, 0, 10, 10, 10))
        t1 = pw.t1_model_checks(verts, tris)
        self.assertTrue(t1["closed_two_manifold"])
        self.assertEqual(t1["boundary_edge_count"], 0)
        self.assertEqual(t1["nonmanifold_edge_count"], 0)
        self.assertEqual(t1["shell_count"], 1)
        self.assertEqual(t1["triangle_count"], 12)

    def test_box_missing_one_triangle_has_boundary_edges(self):
        tris3 = _box_triangles(0, 0, 0, 10, 10, 10)[:-1]
        verts, tris = self._read_via_tempfile(tris3)
        t1 = pw.t1_model_checks(verts, tris)
        self.assertFalse(t1["closed_two_manifold"])
        self.assertGreater(t1["boundary_edge_count"], 0)

    def test_two_disjoint_boxes_are_two_shells(self):
        tris3 = _box_triangles(0, 0, 0, 10, 10, 10) + _box_triangles(100, 100, 100, 110, 110, 110)
        verts, tris = self._read_via_tempfile(tris3)
        t1 = pw.t1_model_checks(verts, tris)
        self.assertTrue(t1["closed_two_manifold"])
        self.assertEqual(t1["shell_count"], 2)
        self.assertFalse(t1["single_shell"])

    def test_ambiguous_extents_flagged_when_within_ratio(self):
        # 10 x 9.5 x 1 box: two longest extents (10, 9.5) are within 20%.
        verts, tris = self._read_via_tempfile(_box_triangles(0, 0, 0, 10, 9.5, 1))
        t1 = pw.t1_model_checks(verts, tris)
        self.assertIsNotNone(t1["ambiguous_extents"])

    def test_binary_stl_size_check_rejects_truncated_file(self):
        import tempfile
        fd, path = tempfile.mkstemp(suffix=".stl")
        os.close(fd)
        try:
            _write_binary_stl(path, _box_triangles(0, 0, 0, 10, 10, 10))
            with open(path, "r+b") as fh:
                fh.truncate(os.path.getsize(path) - 10)
            ok, msg = pw._binary_stl_size_check(path)
            self.assertFalse(ok)
        finally:
            os.remove(path)

    def test_sha256_matches_hashlib(self):
        import tempfile
        fd, path = tempfile.mkstemp(suffix=".stl")
        os.close(fd)
        try:
            _write_binary_stl(path, _box_triangles(0, 0, 0, 10, 10, 10))
            with open(path, "rb") as fh:
                expected = hashlib.sha256(fh.read()).hexdigest()
            self.assertEqual(pw.sha256_file(path), expected)
        finally:
            os.remove(path)


class StandUpAxisTests(unittest.TestCase):
    def test_rotate_axis_to_z_is_identity_for_z(self):
        pts = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        self.assertEqual(pw._rotate_axis_to_z(pts, "z"), pts)

    def test_rotate_axis_to_z_maps_y_unit_vector_onto_z(self):
        out = pw._rotate_axis_to_z([(0.0, 1.0, 0.0)], "y")
        x, y, z = out[0]
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, 1.0)

    def test_rotate_axis_to_z_maps_x_unit_vector_onto_z(self):
        out = pw._rotate_axis_to_z([(1.0, 0.0, 0.0)], "x")
        x, y, z = out[0]
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, 1.0)

    def test_rotation_is_proper_not_a_mirror(self):
        # A proper rotation preserves the signed volume of a tetrahedron
        # built from three orthonormal basis vectors and the origin, i.e.
        # its determinant is +1, never -1 (a reflection would flip section
        # winding/handedness downstream).
        basis = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
        for axis in ("x", "y", "z"):
            out = pw._rotate_axis_to_z(basis, axis)
            m = out  # rows are the images of e1, e2, e3
            det = (
                m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
            )
            self.assertAlmostEqual(det, 1.0, places=6, msg="axis {} not a proper rotation".format(axis))

    def test_explicit_up_axis_skips_auto_probing(self):
        verts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
        tris = [(0, 1, 2)]
        decision = pw.resolve_stand_up_axis(verts, tris, "x")
        self.assertEqual(decision["mode"], "explicit_override")
        self.assertEqual(decision["chosen_axis"], "x")
        self.assertIsNone(decision["candidates"])

    def test_rectangular_coupon_falls_back_no_axis_has_a_chord(self):
        # A plain axis-aligned box (a structural coupon, not an aerofoil) has
        # no chord axis on ANY of its three candidate stand-up axes -- every
        # corner is 90 degrees by construction (own rectangle-vs-aerofoil
        # discriminator). auto stand-up must fall back to the longest-bbox-axis
        # heuristic, not silently pick one via the chord-axis probe.
        import tempfile

        fd, path = tempfile.mkstemp(suffix=".stl")
        os.close(fd)
        try:
            _write_binary_stl(path, _box_triangles(0, 0, 0, 10, 20, 5))
            from m18_mesh_probe import read_binary_stl
            verts, tris = read_binary_stl(path)
            decision = pw.resolve_stand_up_axis(verts, tris, "auto")
            self.assertEqual(decision["mode"], "auto_fallback_longest_bbox")
            self.assertEqual(decision["chosen_axis"], "y")  # longest extent (20)
        finally:
            os.remove(path)


class StandUpConfoundTests(unittest.TestCase):
    """pw.apply_stand_up_confound: the gate that stops a wrong stand-up axis from
    being reported as a blunt trailing edge."""

    def test_fallback_mode_clears_fatal_but_keeps_chord_axis_unavailable(self):
        t2 = {"fatal": True, "chord_axis_unavailable": True, "error": "synthetic"}
        msg = pw.apply_stand_up_confound("auto_fallback_longest_bbox", t2)
        self.assertIsNotNone(msg)
        self.assertFalse(t2["fatal"])
        self.assertTrue(t2["chord_axis_unavailable"])  # NOT cleared -- see docstring
        self.assertTrue(t2["stand_up_confound"])

    def test_non_fallback_mode_is_a_no_op(self):
        for mode in ("explicit_override", "auto", "auto_ambiguous"):
            t2 = {"fatal": True, "chord_axis_unavailable": False, "error": "x"}
            msg = pw.apply_stand_up_confound(mode, t2)
            self.assertIsNone(msg)
            self.assertTrue(t2["fatal"])  # untouched

    def test_fallback_mode_with_clean_t2_is_a_no_op(self):
        t2 = {"fatal": False, "chord_axis_unavailable": False, "error": None}
        msg = pw.apply_stand_up_confound("auto_fallback_longest_bbox", t2)
        self.assertIsNone(msg)

    def test_fallback_axis_never_gets_a_clean_t2_read_on_real_geometry(self):
        """A fallback axis never gets a clean T2 read (see
        ``apply_stand_up_confound``'s docstring): on a real fixture that forces
        ``auto_fallback_longest_bbox`` (no candidate axis resolves a chord),
        the ACTUAL ``t2_orientation`` read for the chosen fallback axis never
        comes back clean either -- it is the exact same
        ``chord_line_angle_deg`` call on the exact same mid-extent plane the
        fallback decision itself already failed. Demonstrated directly through
        the real (non-monkeypatched) pipeline, not asserted."""
        import tempfile

        from m18_mesh_probe import read_binary_stl, MeshSectioner

        fd, path = tempfile.mkstemp(suffix=".stl")
        os.close(fd)
        try:
            # Same rectangular-box fixture the fallback-axis-resolution test
            # above uses: no chord axis on any of x/y/z.
            _write_binary_stl(path, _box_triangles(0, 0, 0, 10, 20, 5))
            verts, tris = read_binary_stl(path)
            axis_decision = pw.resolve_stand_up_axis(verts, tris, "auto")
            self.assertEqual(axis_decision["mode"], "auto_fallback_longest_bbox")
            chosen_axis = axis_decision["chosen_axis"]

            stand_up = pw.apply_stand_up(verts, tris, "auto", "auto")
            self.assertEqual(stand_up["axis_decision"]["mode"], "auto_fallback_longest_bbox")
            self.assertEqual(stand_up["axis_decision"]["chosen_axis"], chosen_axis)

            sectioner = MeshSectioner(stand_up["verts"], tris)
            t2 = pw.t2_orientation(sectioner)
            # Never a clean resolution: either the confound-relevant flags
            # this function already checks, or T2's own separate "no
            # section" error path (established fact 2) -- but NEVER
            # ``rotation_applied`` True with no error at all.
            clean = (
                t2["rotation_applied"]
                and not t2["fatal"]
                and not t2["chord_axis_unavailable"]
                and t2["error"] is None
            )
            self.assertFalse(clean)
        finally:
            os.remove(path)


def _c_shape_prism_triangles(z0, z1):
    """A single watertight triangle prism whose XY cross-section is a
    non-convex "C"/bracket shape (a rectangle with a notch cut into its
    right edge, open to the exterior -- NOT an enclosed hole). Constant
    along Z, so every layer between ``z0`` and ``z1`` sections to the
    identical 2D polygon.

    The point of this shape: probed with a vertical chord strip anywhere
    inside the notch's X range (8..20), the strip crosses the polygon's
    material in two SEPARATE pieces (the bottom band y=[0,3] and the top
    band y=[7,10]) with a gap that is NOT explained by any interior ring
    (``ops.hole_bounds`` returns ``[]`` -- this polygon has none). That is
    exactly ``aircraft_core.frame.local_span``'s ``ReEntrantSectionError``
    trigger (frame.py:588-595), reached from
    ``StructureGenerator.generate`` while placing any family whose
    candidate stations fall in that range (here, the skin-lattice
    families' 3.9 mm rib pitch across the full 0..20 mm chord).
    """
    # Boundary, CCW: (0,0) -> (20,0) -> (20,3) -> (8,3) -> (8,7) -> (20,7)
    # -> (20,10) -> (0,10) -> close.
    boundary = [
        (0.0, 0.0), (20.0, 0.0), (20.0, 3.0), (8.0, 3.0),
        (8.0, 7.0), (20.0, 7.0), (20.0, 10.0), (0.0, 10.0),
    ]
    # Cap decomposition: the C-shape is exactly the union of three
    # axis-aligned rectangles (their internal shared edges at x=8 are not
    # part of the exterior boundary, only used to triangulate the caps).
    cap_rects = [
        (0.0, 0.0, 8.0, 10.0),   # left column, full height
        (8.0, 0.0, 20.0, 3.0),   # bottom-right band
        (8.0, 7.0, 20.0, 10.0),  # top-right band
    ]

    tris = []

    def _rect_cap(x0, y0, x1, y1, z):
        c0, c1, c2, c3 = (x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)
        tris.append((c0, c1, c2))
        tris.append((c0, c2, c3))

    for (x0, y0, x1, y1) in cap_rects:
        _rect_cap(x0, y0, x1, y1, z0)
        _rect_cap(x0, y0, x1, y1, z1)

    n = len(boundary)
    for i in range(n):
        x0, y0 = boundary[i]
        x1, y1 = boundary[(i + 1) % n]
        b0, b1 = (x0, y0, z0), (x1, y1, z0)
        t0, t1 = (x0, y0, z1), (x1, y1, z1)
        tris.append((b0, b1, t1))
        tris.append((b0, t1, t0))

    return tris


class ReEntrantSectionDegradationTests(unittest.TestCase):
    """Fidelity regression: a re-entrant
    section (``aircraft_core.frame.ReEntrantSectionError``, a ``ValueError``
    subclass) must not fall into ``t3_layer_sweep``'s generic per-layer catch-all and
    be counted as a hard FAILURE: the real host
    (``plugin/aircraft_structures/capability.py:3371-3412``) catches it BY TYPE
    and records a DEGRADED/shutdown layer instead, since the section is left
    unmutated (the host's own geometry, untouched) rather than failing the
    whole slice. Constructs the re-entrant geometry directly (a "C" -shaped
    prism) rather than depending on an untracked wing STL."""

    @unittest.skipUnless(HAVE_SHAPELY, "shapely not installed (tool-only backend)")
    def test_re_entrant_layer_is_degraded_not_failed(self):
        import tempfile
        import structural_analysis as sa

        cap_config = json.loads(
            (REPO_ROOT / "presets" / "plugin" / "skin_lattice_only_config.json").read_text()
        )["config"][0]["cap_config"]
        spec = sa.build_spec_from_cap_config(cap_config)
        root_solid_layers = int(spec.root_solid_layers or 0)

        # Tall enough (40 mm at 4 mm layers = 10 layers) that body layers
        # remain after the config's own root_solid_layers=5 are skipped.
        z0, z1 = 0.0, 40.0
        tris_pts = _c_shape_prism_triangles(z0, z1)

        fd, path = tempfile.mkstemp(suffix=".stl")
        os.close(fd)
        try:
            _write_binary_stl(path, tris_pts)
            verts, tris = pw.read_binary_stl(path)
        finally:
            os.remove(path)

        # Explicit "z" stand-up: the prism's extrusion axis IS the span
        # axis and the chord is already along X by construction, so no
        # auto-detection/rotation is needed or wanted here.
        stand_up = pw.apply_stand_up(verts, tris, "z", "auto")
        self.assertEqual(stand_up["axis_decision"]["chosen_axis"], "z")

        orientation = {
            "fatal": False, "chord_axis_unavailable": False,
            "theta_deg": 0.0, "rotation_applied": False,
        }
        diagnostics, meta = pw.t3_layer_sweep(
            stand_up["verts"], tris, spec, 0.42, 4.0, 4.0, orientation,
        )
        summary = diagnostics.to_dict()
        body_records = [
            r for r in summary["layers"]
            if r["extra"].get("reason") != "root_solid_layers"
        ]
        self.assertGreater(root_solid_layers, 0)
        self.assertGreater(len(body_records), 0)

        reentrant = [
            r for r in body_records
            if r["extra"].get("reason") == "re_entrant_section"
        ]
        self.assertGreater(
            len(reentrant), 0,
            "no layer hit the re-entrant path -- geometry/spec no longer "
            "exercises it: {}".format(summary),
        )
        for r in reentrant:
            self.assertFalse(r["failed"], r)
            self.assertTrue(r["shutdown"], r)
            self.assertFalse(r["mutated"], r)
            self.assertTrue(
                any(d["kind"] == "re_entrant_section" for d in r["degradations"]),
                r,
            )

        # The headline claim this test pins: NONE of these layers are
        # counted as failures, matching the real host's degrade-not-fail
        # behaviour for this exact exception type.
        self.assertEqual(summary["failed_count"], 0, summary)


class RootEndTests(unittest.TestCase):
    def test_larger_area_end_is_root(self):
        # A cone-like stack: big square base near z=0, tiny square near z=10.
        verts = []
        big = [(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)]
        small = [(-1, -1, 10), (1, -1, 10), (1, 1, 10), (-1, 1, 10)]
        verts = big + small
        decision = pw.resolve_root_end(verts, "auto")
        self.assertEqual(decision["root_end"], "low")

    def test_explicit_override(self):
        verts = [(0.0, 0.0, 0.0), (1.0, 1.0, 5.0)]
        decision = pw.resolve_root_end(verts, "high")
        self.assertEqual(decision["mode"], "explicit_override")
        self.assertEqual(decision["root_end"], "high")


@unittest.skipUnless(HAVE_SHAPELY, "shapely not installed (tool-only backend)")
class FatalAlignmentShortCircuitTests(unittest.TestCase):
    """T3 must mirror capability.py:2645-2648: once alignment is fatal,
    EVERY body layer refuses with the SAME message, not just the first --
    and the sweep must not need to run generation at all to prove it."""

    def test_every_body_layer_refuses_when_orientation_is_fatal(self):
        import structural_analysis as sa

        cap_config = json.loads(
            (REPO_ROOT / "presets" / "plugin" / "skin_lattice_only_config.json").read_text()
        )["config"][0]["cap_config"]
        spec = sa.build_spec_from_cap_config(cap_config)

        verts, tris = pw.read_binary_stl(str(FIXTURE_STL))
        stand_up = pw.apply_stand_up(verts, tris, "z", "auto")
        self.assertIsNotNone(stand_up["axis_decision"]["chosen_axis"])

        orientation = {
            "fatal": True,
            "fatal_message": "synthetic fatal alignment message for this test",
            "theta_deg": 0.0,
            "rotation_applied": False,
            "chord_axis_unavailable": False,
        }
        diagnostics, meta = pw.t3_layer_sweep(
            stand_up["verts"], tris, spec, 0.42, 4.0, 4.0, orientation,
        )
        summary = diagnostics.to_dict()
        body_records = [
            r for r in summary["layers"] if r["extra"].get("reason") != "root_solid_layers"
        ]
        self.assertGreater(len(body_records), 0)
        for r in body_records:
            self.assertTrue(r["failed"], r)
            self.assertEqual(r["message"], "synthetic fatal alignment message for this test")
        self.assertEqual(summary["mutated_count"], 0)
        self.assertEqual(summary["failed_count"], len(body_records))


class LayerZConventionTests(unittest.TestCase):
    """Pins the layer-count/section-plane conventions against REAL in-host
    evidence (: a real run of control_refined_placed.stl, span 200.9553 mm,
    layer_height 0.2 mm, first_layer_height 0.2 mm, reporting ``layers_seen:
    1005`` and ``first_shutdown_z: 200.99999999999713`` -- i.e. the host's own
    top layer's print_z is ABOVE the mesh's own z_max, and slices from a plane
    BELOW it). ``tools/d023_arm_sweep.py``'s own ``round(z_max/h)`` convention
    gives 1005 too on THIS span by coincidence (200.9553/0.2 = 1004.78, which
    rounds to 1005), but does not generalise -- ``ceil`` is what the host's own
    top-layer-above-z_max behaviour requires in general, which is why this tool
    uses ``ceil``, not ``round``."""

    def test_n_layers_matches_real_host_evidence(self):
        self.assertEqual(pw._n_layers(200.9553, 0.2, 0.2), 1005)

    def test_top_layer_slice_plane_sits_below_mesh_z_max_like_the_host(self):
        # Host: print_z of the last (1005th, index 1004) layer is
        # ~201.0 mm, ABOVE the mesh's own z_max (200.9553 mm) -- yet the
        # host still sections it (first_shutdown_z is recorded, not
        # skipped). slice_z, half a layer below print_z, is what actually
        # falls back inside the mesh.
        z_max = 200.9553
        slice_z = pw._layer_slice_z(1004, 0.2, 0.2)
        self.assertLess(slice_z, z_max)
        self.assertGreater(slice_z, z_max - 0.2)

    def test_first_layer_uses_its_own_first_layer_height(self):
        # Layer 0's slice plane is HALF its own first_layer_height, not
        # half the (possibly different) ordinary layer_height.
        self.assertAlmostEqual(pw._layer_slice_z(0, 0.2, 0.3), 0.15)


@unittest.skipUnless(HAVE_SHAPELY, "shapely not installed (tool-only backend)")
class ChannelRotationRegressionTest(unittest.TestCase):
    """Pins channel-centroid rotation (see the module docstring's "FIDELITY"
    section and ``tools/preflight_wing.py``'s ``t3_layer_sweep`` comment): the
    outer loop is rotated into canonical (chord-along-X) frame per layer, and
    each layer's spar-channel centroid must be rotated too. Feeding it
    UN-rotated puts the detected channel at the wrong X/Y under any
    nonzero theta and made TopologyValidator refuse body layers ("slot ...
    requested but is not present in the output"). Uses a KNOWN-spar
    fixture (m18_wing_spar10_at0.30.stl) with a synthetic 20-degree theta
    -- this fixture has no real global misalignment, but the rotation-path
    code is exercised identically to a genuinely rotated wing.

    Without the rotation this test fails with a "no material removed"
    symptom."""

    def test_rotated_channel_survives_with_theta_applied(self):
        import structural_analysis as sa

        # The live host config's ``spar_station``/``allow_config_spar_
        # fallback: False`` means the spar-socket candidate is built ONLY
        # from a DETECTED channel, never synthesised -- exactly the path
        # the real bug broke (skin_lattice_only_config.json, used by the
        # other tests in this file, has no spar family at all and would
        # not exercise it).
        cap_config = json.loads(LIVE_CONFIG.read_text())["config"][0]["cap_config"]
        spec = sa.build_spec_from_cap_config(cap_config)

        stl = REPO_ROOT / "tests" / "fixtures" / "wing" / "m18_wing_spar10_at0.30.stl"
        verts, tris = pw.read_binary_stl(str(stl))
        stand_up = pw.apply_stand_up(verts, tris, "z", "auto")
        self.assertIsNotNone(stand_up["axis_decision"]["chosen_axis"])

        orientation = {
            "fatal": False, "chord_axis_unavailable": False,
            # 20 deg: large enough that an un-rotated channel centroid (fed to
            # the generator while the outer loop is rotated) misplaces the
            # detected channel and the layer fails with a "no material
            # removed" symptom. This fixture is coarse (layer_height 8.0, 10
            # body layers) and measures clean (0 failures) at this angle with
            # the rotation applied. 30 deg is avoided: it fails at layer 9
            # (Z=80.00 mm) with a different failure class, a skin-margin
            # shortfall under the section's own rotation correction, not the
            # channel-misplacement symptom this test targets.
            "theta_deg": 20.0, "rotation_applied": True,
        }
        diagnostics, meta = pw.t3_layer_sweep(
            stand_up["verts"], tris, spec, 0.42, 8.0, 8.0, orientation,
        )
        summary = diagnostics.to_dict()
        self.assertEqual(
            summary["failed_count"], 0,
            "rotated-channel regression: {}".format(summary.get("first_failure")),
        )
        self.assertGreater(summary["mutated_count"], 0)


@unittest.skipUnless(HAVE_SHAPELY, "shapely not installed (tool-only backend)")
class EndToEndTests(unittest.TestCase):
    """One small, real end-to-end run through main() at a coarse layer height so
    the full T1-T4 pipeline runs in seconds, against a tracked fixture wing
    (real wing STLs are exercised manually)."""

    def test_runs_clean_and_never_prints_unqualified_pass(self):
        import io
        import contextlib
        import tempfile

        json_out = tempfile.mktemp(suffix=".json")
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                rc = pw.main([
                    str(FIXTURE_STL),
                    "--config", str(LIVE_CONFIG),
                    "--up-axis", "auto",
                    "--layer-height", "4.0",
                    "--json", json_out,
                ])
            out = buf.getvalue()
            self.assertIn("T4 BLIND SPOT", out)
            self.assertIn("NOT an in-host guarantee", out)
            self.assertNotRegex(out, r"STATUS:\s*PASS\b")
            self.assertIn("STATUS:", out)

            with open(json_out, "r", encoding="utf-8") as fh:
                report = json.load(fh)
            self.assertIn("t1_model", report)
            self.assertIn("t2_orientation", report)
            self.assertIn("t3_summary", report)
            self.assertEqual(report["t4_blind_spot"], pw.T4_BLIND_SPOT_TEXT)
            self.assertEqual(
                report["stl_sha256"],
                hashlib.sha256(FIXTURE_STL.read_bytes()).hexdigest(),
            )
            # Exact, not a membership check: this fixture at this
            # config/layer-height is measured clean.
            self.assertEqual(report["status"], "CLEAN_NO_GUARANTEE")
            self.assertEqual(rc, 0)
        finally:
            if os.path.exists(json_out):
                os.remove(json_out)


@unittest.skipUnless(HAVE_SHAPELY, "shapely not installed (tool-only backend)")
class ExitCodeContractTests(unittest.TestCase):
    """/1: the exit-code scheme is a user-visible contract, not an implementation
    detail -- pin each reachable code through ``main()`` itself, not just the
    helper it calls."""

    def _run_main(self, args):
        import io
        import contextlib

        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = pw.main(args)
        return rc, buf.getvalue()

    def test_exit_0_clean(self):
        import tempfile

        json_out = tempfile.mktemp(suffix=".json")
        try:
            rc, out = self._run_main([
                str(FIXTURE_STL), "--config", str(LIVE_CONFIG),
                "--up-axis", "auto", "--layer-height", "4.0", "--json", json_out,
            ])
            self.assertEqual(rc, 0)
            with open(json_out) as fh:
                report = json.load(fh)
            self.assertEqual(report["status"], "CLEAN_NO_GUARANTEE")
        finally:
            if os.path.exists(json_out):
                os.remove(json_out)

    def test_exit_2_tool_level_refusal_missing_stl(self):
        rc, out = self._run_main(["/no/such/file.stl"])
        self.assertEqual(rc, 2)

    def test_exit_3_ambiguous_stand_up(self):
        # Monkeypatch probe_axis_for_chord so TWO candidate axes both resolve
        # a chord. This is the only exercise of the auto_ambiguous stand-up
        # branch in this suite (no cheap real two-chord fixture exists).
        def fake_probe(verts, tris, axis):
            if axis in ("x", "y"):
                return {"axis": axis, "chord_found": True, "theta_deg": 1.0, "hull_vertices": 50}
            return {"axis": axis, "chord_found": False, "error": "no chord"}

        import tempfile

        json_out = tempfile.mktemp(suffix=".json")
        orig = pw.probe_axis_for_chord
        pw.probe_axis_for_chord = fake_probe
        try:
            rc, out = self._run_main([
                str(FIXTURE_STL), "--config", str(LIVE_CONFIG),
                "--up-axis", "auto", "--layer-height", "4.0", "--json", json_out,
            ])
            self.assertEqual(rc, 3)
            with open(json_out) as fh:
                report = json.load(fh)
            self.assertEqual(report["status"], "STAND_UP_AMBIGUOUS")
            # Exit 3 has two distinct status strings; the printed status line
            # is what tells them apart -- this early-return path must print it
            # too, not just the JSON.
            self.assertIn("STATUS: STAND_UP_AMBIGUOUS", out)
        finally:
            pw.probe_axis_for_chord = orig
            if os.path.exists(json_out):
                os.remove(json_out)

    def test_exit_3_unresolved_stand_up_fallback(self):
        # : the real correctness fix. No axis resolves a chord (fallback mode)
        # AND the chosen fallback axis's own T2 reading is fatal (blunt TE) --
        # this must NOT be CLEAN_NO_GUARANTEE/exit 0. A rectangular coupon (no
        # chord axis on any of the 3 candidate axes) forces the fallback branch
        # on its own; nothing further needs to be forced to exercise it, since
        # a coupon is non-fatal (ChordAxisUnavailableError, not
        # BluntTrailingEdgeError) -- so this test forces the FATAL sub-case
        # directly via monkeypatched T2, which is the case of a genuine blunt-TE wing under the fallback
        # axis.
        import tempfile

        def fake_probe(verts, tris, axis):
            return {"axis": axis, "chord_found": False, "error": "no chord"}

        def fake_t2_orientation(sectioner):
            return {
                "theta_deg": 0.0, "rotation_applied": False,
                "mid_span_theta_deg": None, "station_theta_deg": [], "spread_deg": None,
                "excluded_stations": [], "chord_axis_unavailable": False,
                "fatal": True,
                "fatal_message": "synthetic BluntTrailingEdgeError for this test",
                "error": "synthetic blunt TE", "mid_hull_vertices": 50,
            }

        json_out = tempfile.mktemp(suffix=".json")
        orig_probe = pw.probe_axis_for_chord
        orig_t2 = pw.t2_orientation
        pw.probe_axis_for_chord = fake_probe
        pw.t2_orientation = fake_t2_orientation
        try:
            rc, out = self._run_main([
                str(FIXTURE_STL), "--config", str(LIVE_CONFIG),
                "--up-axis", "auto", "--layer-height", "4.0", "--json", json_out,
            ])
            self.assertEqual(rc, 3)
            with open(json_out) as fh:
                report = json.load(fh)
            self.assertEqual(report["status"], "UNRESOLVED_STAND_UP")
            self.assertIn("UNRESOLVED:", out)
            self.assertNotIn("STATUS: CLEAN_NO_GUARANTEE", out)
        finally:
            pw.probe_axis_for_chord = orig_probe
            pw.t2_orientation = orig_t2
            if os.path.exists(json_out):
                os.remove(json_out)

    def test_exit_3_unresolved_stand_up_fallback_with_non_fatal_t2_error(self):
        # See ``apply_stand_up_confound``'s docstring: fallback axis chosen, AND
        # T2's own mid-span read hits its "no outer loop"/"no Z extent"
        # early-return path, which sets ``t2["error"]`` but leaves BOTH
        # ``fatal`` and ``chord_axis_unavailable`` at their default False.
        # ``apply_stand_up_confound`` must not return None for this shape
        # (neither flag set), or ``stand_up_unresolved`` would not be derived
        # and the run would reach CLEAN_NO_GUARANTEE / exit 0 with an axis
        # that was never confirmed. Required: exit 3 / UNRESOLVED_STAND_UP.
        import tempfile

        def fake_probe(verts, tris, axis):
            return {"axis": axis, "chord_found": False, "error": "no outer loop at mid-extent"}

        def fake_t2_orientation(sectioner):
            return {
                "theta_deg": 0.0, "rotation_applied": False,
                "mid_span_theta_deg": None, "station_theta_deg": [], "spread_deg": None,
                "excluded_stations": [], "chord_axis_unavailable": False,
                "fatal": False, "fatal_message": None,
                "error": "mid-span (50% of Z range) section has no outer loop",
                "mid_hull_vertices": None,
            }

        json_out = tempfile.mktemp(suffix=".json")
        orig_probe = pw.probe_axis_for_chord
        orig_t2 = pw.t2_orientation
        pw.probe_axis_for_chord = fake_probe
        pw.t2_orientation = fake_t2_orientation
        try:
            rc, out = self._run_main([
                str(FIXTURE_STL), "--config", str(LIVE_CONFIG),
                "--up-axis", "auto", "--layer-height", "4.0", "--json", json_out,
            ])
            self.assertEqual(rc, 3)
            with open(json_out) as fh:
                report = json.load(fh)
            self.assertEqual(report["status"], "UNRESOLVED_STAND_UP")
            self.assertIn("UNRESOLVED:", out)
            self.assertNotIn("STATUS: CLEAN_NO_GUARANTEE", out)
        finally:
            pw.probe_axis_for_chord = orig_probe
            pw.t2_orientation = orig_t2
            if os.path.exists(json_out):
                os.remove(json_out)


class DecideFinalStatusTests(unittest.TestCase):
    """: ``decide_final_status`` is the one place the
    exit-code precedence (2 > 3 > 1 > 4 > 0) is actually decided -- pinned
    directly, cheaply, rather than only through an end-to-end run that
    would need a real mesh landing exactly on each boundary (exit 4 in
    particular: a real degraded-but-not-failed sweep is only cheap to
    reach at the real 0.2 mm layer height, which is too slow for a unit
    test -- see the module's own >3-minute timing note)."""

    def test_clean(self):
        self.assertEqual(pw.decide_final_status(False, 0, 0), ("CLEAN_NO_GUARANTEE", 0))

    def test_refused(self):
        self.assertEqual(pw.decide_final_status(False, 3, 0), ("REFUSED", 1))

    def test_degraded_no_refusals(self):
        self.assertEqual(pw.decide_final_status(False, 0, 2), ("DEGRADED", 4))

    def test_stand_up_unresolved_beats_everything(self):
        # Precedence: stand_up_unresolved wins even over failures/
        # degradations that would otherwise map to 1 or 4.
        self.assertEqual(pw.decide_final_status(True, 0, 0), ("UNRESOLVED_STAND_UP", 3))
        self.assertEqual(pw.decide_final_status(True, 5, 5), ("UNRESOLVED_STAND_UP", 3))

    def test_refused_beats_degraded(self):
        self.assertEqual(pw.decide_final_status(False, 1, 1), ("REFUSED", 1))


class KeepPlacementTests(unittest.TestCase):
    """``--keep-placement`` keeps a deliberate tip-down placement and reports the
    resolved root end + the root-end-aware latch, instead of flipping the root
    to the bed (audit finding 9)."""

    @staticmethod
    def _frustum(root_low, span=12.0):
        verts = []
        for i in range(13):
            z = span * i / 12.0
            frac = z / span
            w = 20.0 + (4.0 - 20.0) * (frac if root_low else 1.0 - frac)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    verts.append((sx * w / 2.0, sy * 3.0, z))
        return verts

    def test_default_flips_a_top_root_to_the_bed_and_keep_placement_does_not(self):
        verts = self._frustum(root_low=False)
        flipped = pw.apply_stand_up(verts, [], "z", "auto")
        kept = pw.apply_stand_up(verts, [], "z", "auto", keep_placement=True)
        self.assertFalse(flipped["keep_placement"])
        self.assertTrue(kept["keep_placement"])
        # The decision is the same (root is the HIGH end) ...
        self.assertEqual(flipped["root_end_decision"]["root_end"], "high")
        self.assertEqual(kept["root_end_decision"]["root_end"], "high")
        # ... but only the default mode moves the geometry.
        self.assertEqual([v[2] for v in kept["verts"]], [v[2] for v in verts])
        self.assertNotEqual([v[2] for v in flipped["verts"]], [v[2] for v in verts])

    @staticmethod
    def _meta(statuses, dias, lh=1.0):
        n = len(statuses)
        return {
            "spar_status": statuses, "max_channel_diameter_mm": dias,
            "layer_print_z": [lh * (i + 1) for i in range(n)],
            "layer_slice_z": [lh * (i + 0.5) for i in range(n)],
            "z_min": 0.0, "z_max": lh * n,
        }

    def test_summary_tip_down_keeps_the_root_ward_socket(self):
        A, F, N = "absent", "fit", "not_fit"
        st = [A, A, N, N, N, F, F, F, F, F, F, F]
        dias = [None, None, 10.5, 10.5, 10.5] + [10.5] * 7
        meta = self._meta(st, dias)
        out = pw.root_end_and_latch_summary(
            self._frustum(root_low=False, span=12.0), meta, "auto", enforce=True)
        self.assertEqual((out["root_end"], out["root_at"]), ("high", "top"))
        self.assertEqual(out["latch"]["walk_direction"], "top_to_bed")
        self.assertEqual(out["latch"]["arm_index"], 4)
        self.assertEqual(out["latch"]["suppressed_count"], 0)
        runs = out["socket_cut_z_runs"]
        self.assertEqual([(r["z_first_mm"], r["z_last_mm"]) for r in runs], [(6.0, 12.0)])
        self.assertEqual([(r["z_first_mm"], r["z_last_mm"])
                          for r in out["not_fit_z_runs"]], [(3.0, 5.0)])

    def test_summary_root_at_bed_override_reproduces_the_pre_d054_loss(self):
        A, F, N = "absent", "fit", "not_fit"
        st = [A, A, N, N, N, F, F, F, F, F, F, F]
        meta = self._meta(st, [None, None] + [10.5] * 10)
        out = pw.root_end_and_latch_summary(
            self._frustum(root_low=False), meta, "bed", enforce=True)
        self.assertEqual(out["root_at"], "bed")
        self.assertEqual(out["socket_cut_z_runs"], [])
        self.assertEqual(out["layer_counts"]["suppressed"], 7)

    def test_summary_disagreement_refuses_only_when_enforced(self):
        A, F = "absent", "fit"
        # AREA: root at the top; CHANNEL span abuts the LOW end.
        st = [F] * 8 + [A] * 4
        meta = self._meta(st, [10.5] * 8 + [None] * 4)
        verts = self._frustum(root_low=False)
        with self.assertRaises(pw.RootEndError) as cm:
            pw.root_end_and_latch_summary(verts, meta, "auto", enforce=True)
        self.assertIn("AREA says the root is the high Z end", str(cm.exception))
        self.assertIn("CHANNEL says the root is the low Z end", str(cm.exception))
        out = pw.root_end_and_latch_summary(verts, meta, "bed", enforce=False)
        self.assertEqual(out["root_at"], "bed")
        out = pw.root_end_and_latch_summary(verts, meta, "auto", enforce=False)
        self.assertIsNotNone(out["auto_would_refuse"])

    @unittest.skipUnless(HAVE_SHAPELY, "shapely not installed (tool-only backend)")
    def test_cli_keeps_a_flipped_real_fixture_and_resolves_root_at_top(self):
        import io
        import contextlib
        import tempfile

        verts, tris = pw.read_binary_stl(str(FIXTURE_STL))
        z_hi = max(v[2] for v in verts)
        tip_down = [(x, y, z_hi - z) for x, y, z in verts]
        tmp = tempfile.mkdtemp()
        stl = os.path.join(tmp, "tip_down.stl")
        json_out = os.path.join(tmp, "out.json")
        _write_binary_stl(stl, [tuple(tip_down[i] for i in t) for t in tris])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            pw.main([stl, "--config", str(LIVE_CONFIG), "--up-axis", "z",
                     "--layer-height", "4.0", "--keep-placement",
                     "--json", json_out])
        out = buf.getvalue()
        self.assertIn("--keep-placement: mesh NOT flipped", out)
        self.assertIn("ROOT END + SPAR-SOCKET LATCH", out)
        with open(json_out, "r", encoding="utf-8") as fh:
            report = json.load(fh)
        self.assertTrue(report["stand_up"]["keep_placement"])
        self.assertEqual(report["root_end_resolution"]["root_at"], "top")
        self.assertEqual(report["root_end_resolution"]["latch"]["walk_direction"],
                         "top_to_bed")


if __name__ == "__main__":
    unittest.main()
