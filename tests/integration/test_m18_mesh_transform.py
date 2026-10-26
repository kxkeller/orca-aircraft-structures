"""Mesh-transform integration tests: pin the model -> mesh -> socket chain through
the REAL plugin path (the generated bundle, via ``capability.py``), not just
the offline ``aircraft_core`` proof
(``tests/integration/test_m18_model_determinism.py``, which drives
``MeshSectioner``/``StructureGenerator`` directly with real STL fixtures and
never touches the ``orca.host`` surface at all).

WHY THIS FILE EXISTS. The CAD model is the source of truth for spar-socket
geometry: the plugin reads the original mesh via
``ctx.object.model_object().volumes()[j].mesh()``, cross-sections it per layer,
and fails the slice if it cannot (fail CLOSED: a wing silently produced with no
spar bore would be "a missing member removes geometry without breaking
topology"). The fake host therefore has to expose a real ``model_object``;
without one every integration test would hit that fail-closed path
unconditionally. This file exercises a real, matching mesh through
non-identity placement.

THE MESH->PRINT TRANSFORM. ``capability.py`` applies the model instance's
ROTATION but not its TRANSLATION (an instance X offset once produced a
128.0000 mm bbox disagreement against the host's own slice). This file gives
the fake instance a genuinely non-zero translation AND rotation, so both the
translation exclusion and the rotation APPLICATION are exercised rather than
assumed correct. A mesh fixture with an identity volume transform and an
unrotated instance would leave every branch except plain translation-drop
unexercised.

XY-centring (``capability.py``'s ``_build_mesh_sectioner``) subtracts the mesh's
own transformed-frame XY bbox centre, which cancels ANY uniform translation by
construction -- including instance translation -- so applying or excluding
``i_off`` is numerically indistinguishable. ``InstanceTranslationIsNowANoOpTests``
therefore pins that invariance rather than a "translation must not leak"
property; see its docstring.
"""

import json
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import fake_orca  # noqa: E402
from aircraft_core.mesh_section import MeshSectioner  # noqa: E402


FULL_CONFIG = {
    "layer_height": 0.2,
    "initial_layer_print_height": 0.2,
    "outer_wall_line_width": 0.42,
    "inner_wall_line_width": 0.42,
    "line_width": 0.42,
    "nozzle_diameter": 0.4,
    "wall_generator": "classic",
    "wall_loops": 1,
    "spiral_mode": True,
    "top_shell_layers": 0,
    "bottom_shell_layers": 0,
    "sparse_infill_density": 0,
    "enable_support": False,
    "slice_closing_radius": 0.0049,
}

LAYER_HEIGHT_MM = 0.2
N_LAYERS = 10


class M18MeshTransformTestCase(unittest.TestCase):
    """Common fixture plumbing, mirroring ``test_m1_capability.py``'s
    ``M1IntegrationTestCase``."""

    def setUp(self):
        self._fake_orca, restore = fake_orca.install()
        self.addCleanup(restore)

        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.plugin_dir = self._tmp.name
        self.fake_entry_file = os.path.join(self.plugin_dir, "aircraft_structures.py")

        self.namespace = fake_orca.exec_bundle(self.fake_entry_file)
        self.report_path = os.path.join(self.plugin_dir, "m1", "m1_report.json")

    def arm(self):
        with open(os.path.join(self.plugin_dir, "m1_enabled"), "w", encoding="utf-8"):
            pass

    def new_capability(self):
        cls = self.namespace["AircraftStructuresM1"]
        return cls()

    def make_ctx(self, obj, step=None, config=None, cancelled_fn=None):
        Step = self._fake_orca.slicing.Step
        return self._fake_orca.slicing.SlicingPipelineContext(
            step=step if step is not None else Step.posSlice,
            obj=obj,
            config=dict(FULL_CONFIG if config is None else config),
            cancelled_fn=cancelled_fn,
        )

    def run_capability(self, obj, cap_config, host_config=None):
        self.arm()
        ctx = self.make_ctx(obj, config=host_config)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(cap_config)
        result = cap.execute(ctx)
        return result, cap

    # -- fixture builders ---------------------------------------------------

    def rect_object_with_instance_transform(
        self, gt_x0, gt_y0, gt_x1, gt_y1,
        n_layers=N_LAYERS, layer_height_mm=LAYER_HEIGHT_MM,
        i_off=(0.0, 0.0, 0.0), i_rot_z=0.0,
    ):
        """A ``PrintObject`` whose MODEL MESH is built in mesh-LOCAL
        coordinates, via ``fake_orca.mesh_local_from_print_xyz``, so that once
        ``capability.py`` applies its REAL forward transform (instance
        rotation applied, instance translation dropped), the mesh's
        cross-section reproduces the SAME target rectangle
        ``(gt_x0..gt_x1, gt_y0..gt_y1)`` regardless of ``i_rot_z`` --
        ``local()`` pre-compensates by the INVERSE rotation, so a correctly
        applied forward rotation exactly cancels it (this is the mechanism
        ``InstanceRotationAppliedTests`` relies on to prove rotation is
        really applied: an unapplied or wrong-signed rotation would leave
        the pre-compensation uncancelled and the section far from this
        target).

        THE HOST SLICE is that SAME target rectangle, re-centred: capability.py
        subtracts the whole mesh's OWN XY bounding-box centre, which -- for a
        plain box, by construction -- erases any translation, landing the
        section symmetric about (0, 0). This is independent of ``i_rot_z``:
        since the rotation is exactly cancelled before centring (see above),
        what centring sees is always the UNROTATED target rectangle's own bbox,
        never a rotated one. So the expected host rectangle is simply the
        target rectangle re-centred on its own bbox -- computed the same way
        regardless of ``i_rot_z``.
        """
        top_z = layer_height_mm * n_layers + layer_height_mm

        def local(pt):
            return fake_orca.mesh_local_from_print_xyz(
                pt,
                v_off=(0.0, 0.0, 0.0), v_rot_z=0.0, v_scl=(1.0, 1.0, 1.0),
                i_scl=(1.0, 1.0, 1.0), i_mir=(1.0, 1.0, 1.0), i_rot_z=i_rot_z,
            )

        corners_local = [
            local((gt_x0, gt_y0, 0.0)), local((gt_x1, gt_y0, 0.0)),
            local((gt_x1, gt_y1, 0.0)), local((gt_x0, gt_y1, 0.0)),
        ]
        lx0 = min(c[0] for c in corners_local)
        lx1 = max(c[0] for c in corners_local)
        ly0 = min(c[1] for c in corners_local)
        ly1 = max(c[1] for c in corners_local)

        mesh = fake_orca.box_mesh_mm(lx0, ly0, 0.0, lx1, ly1, top_z)
        volume = fake_orca.ModelVolume(mesh)
        instance = fake_orca.ModelInstance(offset=i_off, rotation=(0.0, 0.0, i_rot_z))
        model_object = fake_orca.ModelObject([volume], [instance])

        # The EXPECTED host rectangle: the UNROTATED target rectangle,
        # re-centred on its own bbox -- see docstring for why i_rot_z does
        # not enter this calculation at all.
        cx, cy = (gt_x0 + gt_x1) / 2.0, (gt_y0 + gt_y1) / 2.0
        host_x0, host_x1 = gt_x0 - cx, gt_x1 - cx
        host_y0, host_y1 = gt_y0 - cy, gt_y1 - cy

        return fake_orca.make_rect_object(
            "wing", n_layers, host_x1 - host_x0, host_y1 - host_y0, layer_height_mm,
            min_x_mm=host_x0, min_y_mm=host_y0, model_object=model_object,
        )


# ---------------------------------------------------------------------------
# (a) A non-zero instance TRANSLATION must not change the alignment result.
# ---------------------------------------------------------------------------


class InstanceTranslationIsNowANoOpTests(M18MeshTransformTestCase):
    """The mesh alignment result (bounds AND every ``worst_by_term`` figure) is
    IDENTICAL whether the instance translation is zero or a large, asymmetric,
    real-looking offset.

    This is a no-op by construction: XY-centring (``_build_mesh_sectioner``)
    subtracts the mesh's own transformed-frame bbox centre, which cancels ANY
    uniform translation applied before it -- including ``i_off``. So this
    guard cannot test "does translation leak through" by looking for a wrong
    answer: adding ``+ i_off[...]`` to the forward transform would NOT move
    ``worst_bbox_delta_mm`` at all. What the class pins is the invariance
    itself; a change that broke it (e.g. removing the XY-centring step
    without also keeping the translation exclusion, which WOULD make
    translation leak through) fails this test. Excluding instance translation
    is also architecturally correct on independent grounds (it matches
    ``ModelObject::raw_bounding_box()``'s ``get_matrix_no_offset()`` in the
    host's ``libslic3r/Model.cpp``).
    """

    def test_mesh_alignment_is_identical_regardless_of_instance_translation(self):
        def run_with_offset(i_off):
            obj = self.rect_object_with_instance_transform(
                2.0, 0.0, 62.0, 10.0, i_off=i_off,
            )
            instance = obj.model_object().instances()[0]
            self.assertEqual(instance.offset(), i_off)
            result, cap = self.run_capability(
                obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
            )
            self.assertEqual(
                result.status, self._fake_orca.PluginResult.Success, result.message
            )
            return cap._mesh_alignment

        zero = run_with_offset((0.0, 0.0, 0.0))
        large = run_with_offset((128.0, 7.5, -3.25))

        self.assertEqual(
            zero["mesh_bounds_after_transform"],
            large["mesh_bounds_after_transform"],
            "instance translation changed the mesh cross-section -- either "
            "the XY-centring fix regressed, or i_off leaked back into the "
            "forward transform",
        )
        self.assertEqual(zero["worst_by_term"], large["worst_by_term"])
        self.assertLess(zero["worst_bbox_delta_mm"], 1e-6)
        self.assertLess(large["worst_bbox_delta_mm"], 1e-6)

        # The box's OWN XY bbox centre is subtracted, so a plain (unrotated)
        # 60x10 box always lands symmetric about (0, 0) regardless of where it
        # was requested -- see rect_object_with_instance_transform's
        # docstring. width=60, height=10 here (gt_x1-gt_x0=60,
        # gt_y1-gt_y0=10).
        bounds = large["mesh_bounds_after_transform"]
        self.assertAlmostEqual(bounds["x"][0], -30.0, places=6)
        self.assertAlmostEqual(bounds["x"][1], 30.0, places=6)
        self.assertAlmostEqual(bounds["y"][0], -5.0, places=6)
        self.assertAlmostEqual(bounds["y"][1], 5.0, places=6)


# ---------------------------------------------------------------------------
# (b) Instance ROTATION, unlike translation, MUST be applied.
# ---------------------------------------------------------------------------


class InstanceRotationAppliedTests(M18MeshTransformTestCase):
    """A rotated instance whose mesh is built in the correspondingly rotated
    LOCAL frame must still align with the host's own (unrotated) slice --
    proving rotation is actually applied, not silently dropped the way
    translation correctly is. 90 degrees about Z (deliberately NOT 180: a
    180-degree rotation is its own inverse, so a wrong-signed rotation
    convention in the transform under test would still happen to align --
    90 degrees does not have that blind spot), on a rectangle NOT centred on
    the origin (x in [2, 62]), so an unapplied OR wrong-signed rotation
    would leave the section in the wrong place and produce a large,
    unmistakable bbox delta rather than a coincidentally small one.
    """

    def test_90deg_instance_rotation_is_applied_and_mesh_aligns(self):
        angle = math.pi / 2.0  # 90 degrees
        obj = self.rect_object_with_instance_transform(
            2.0, 0.0, 62.0, 10.0, i_off=(50.0, 20.0, 0.0), i_rot_z=angle,
        )

        instance = obj.model_object().instances()[0]
        self.assertAlmostEqual(instance.rotation()[2], angle, places=9)

        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)

        alignment = cap._mesh_alignment
        self.assertLess(
            alignment["worst_bbox_delta_mm"], 1e-6,
            "instance rotation was not correctly applied (delta {} mm)".format(
                alignment["worst_bbox_delta_mm"]
            ),
        )
        # The mesh's LOCAL coordinates are pre-rotated by the INVERSE of
        # i_rot_z (see rect_object_with_instance_transform's docstring), so
        # a correctly-applied forward rotation cancels it exactly and the
        # section reproduces the UNROTATED 60x10 target rectangle -- same
        # bounds as the unrotated case, [-30, 30] x [-5, 5], regardless of
        # i_rot_z. An unapplied or wrong-signed rotation would leave the
        # pre-compensation uncancelled and fail alignment well above the
        # 1e-6 mm assertion just above, long before reaching this line.
        bounds = alignment["mesh_bounds_after_transform"]
        self.assertAlmostEqual(bounds["x"][0], -30.0, places=6)
        self.assertAlmostEqual(bounds["x"][1], 30.0, places=6)


# ---------------------------------------------------------------------------
# (c) FAIL CLOSED when the mesh cannot be read.
# ---------------------------------------------------------------------------


class FailClosedOnUnreadableMeshTests(M18MeshTransformTestCase):
    """FAIL CLOSED, never fail-open, when the model mesh cannot be read (a
    missing member removes geometry without breaking topology). Neither branch
    below may reach a Success result with an unmutated/silently-clean slice.

    Every ``TopologyError`` raised from ``_build_mesh_sectioner`` /
    ``_channels_for_layer`` passes the real ``(layer_index, print_z, cause)``
    positional arguments (e.g. ``TopologyError(0, 0.0, "model has... ")``), so
    raising inside a fake ``model_object()`` callback produces exactly
    ``"Aircraft structure invalid at layer 0 (Z=0.20 mm): the model mesh
    could not be read..."``, naming the layer and Z.
    """

    def _run(self, model_object):
        obj = fake_orca.make_rect_object(
            "wing", 3, 20.0, 20.0, LAYER_HEIGHT_MM, model_object=model_object,
        )
        return self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        ) + (obj,)

    def test_model_object_raising_fails_closed(self):
        def boom():
            raise RuntimeError("host model_object() exploded")

        result, _cap, obj = self._run(boom)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertTrue(
            all(layer.make_slices_calls == 0 for layer in obj.layers()),
            "a layer was mutated despite the model mesh being unreadable",
        )
        # TopologyError names the layer and Z correctly (see class
        # docstring): "Aircraft structure invalid at layer 0 (Z=0.20 mm): ..."
        self.assertIn("layer 0", result.message)
        self.assertIn("Z=0.20", result.message)

    def test_volumes_empty_fails_closed(self):
        model_object = fake_orca.ModelObject([], [fake_orca.ModelInstance()])

        result, _cap, obj = self._run(model_object)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertTrue(
            all(layer.make_slices_calls == 0 for layer in obj.layers()),
            "a layer was mutated despite the model having zero part volumes",
        )
        # This branch fires before any real layer is walked, so
        # layer_index=0/print_z=0.0: "Aircraft structure invalid at layer 0
        # (Z=0.00 mm): ..." (see class docstring).
        self.assertIn("layer 0", result.message)
        self.assertIn("Z=0.00", result.message)

    def test_no_model_object_at_all_fails_closed(self):
        """A fixture with no mesh whatsoever must still fail closed with the
        same layer/Z-naming error."""
        result, _cap, obj = self._run(None)

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertTrue(
            all(layer.make_slices_calls == 0 for layer in obj.layers()),
        )


# ---------------------------------------------------------------------------
# (d)/(e) Interior-loop detection tracks the model; no interior loop is a
#         legitimate, non-fatal result.
# ---------------------------------------------------------------------------


class ChannelDetectionTracksTheModelTests(M18MeshTransformTestCase):
    """Core property, exercised through ``capability.py`` itself (not bypassing it
    the way ``test_m18_model_determinism.py`` -- deliberately, for a different
    reason -- drives ``MeshSectioner``/``StructureGenerator`` directly): the
    per-layer channel DETECTED FROM THE MESH must be exactly the modelled
    tube's own centre, and moving the tube in the fake mesh moves the detected
    channel by the same amount.

    THIS CANNOT GO ALL THE WAY THROUGH TO A CUT SOCKET on this fake: a
    socket is a many-point keyhole polygon
    (``aircraft_core/slot.py``'s ``_keyhole``), and ``fake_orca.py``'s
    ``ExPolygon`` kernel only accepts a 4-point axis-aligned rectangle from
    raw points (module docstring "DOCUMENTED SCOPE"); the fake is not a
    general polygon-Boolean engine by design. This test instead pins
    ``capability._mesh_alignment``'s own per-layer channel-detection fields
    (``first_channel``), which are populated by the REAL
    ``_channels_for_layer`` call -- the actual model -> mesh ->
    ``DetectedChannel`` path -- BEFORE the (fake-limited) slot cut is even
    attempted, so the overall ``execute()`` status is irrelevant here. The
    generator's OWN socket-centring math (``DetectedChannel`` ->
    ``WebIntent.start``) is separately pinned, end-to-end with real STL
    fixtures, by ``test_m18_model_determinism.py``.
    """

    # Centred at (0, 0): capability.py's XY-centring always subtracts the
    # mesh's own bbox centre, so a box built off-centre would not match a
    # host rect built at the same raw coordinates -- see
    # fake_orca.make_rect_object's own comment. Built pre-centred here so
    # both sides agree without a shift.
    OUTER = (-20.0, -5.0, 20.0, 5.0)
    HALF_W = 2.0
    HALF_H = 1.5
    HOLE_CY = 0.0

    def _object_with_tube(self, hole_cx, n_layers=3):
        x0, y0, x1, y1 = self.OUTER
        top_z = LAYER_HEIGHT_MM * n_layers + LAYER_HEIGHT_MM
        mesh = fake_orca.box_with_rect_channel_mesh_mm(
            x0, y0, x1, y1, 0.0, top_z,
            hole_cx - self.HALF_W, self.HOLE_CY - self.HALF_H,
            hole_cx + self.HALF_W, self.HOLE_CY + self.HALF_H,
        )
        model_object = fake_orca.identity_model_object(mesh)
        return fake_orca.make_rect_object(
            "wing", n_layers, x1 - x0, y1 - y0, LAYER_HEIGHT_MM,
            min_x_mm=x0, min_y_mm=y0, model_object=model_object,
        )

    def _channel_stats(self, hole_cx):
        obj = self._object_with_tube(hole_cx)
        _result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )
        # Status is deliberately not asserted here -- see class docstring.
        alignment = cap._mesh_alignment
        self.assertIsNotNone(
            alignment["first_channel"],
            "no interior loop was detected at all -- the mesh fixture "
            "itself is broken, not the property under test",
        )
        # channel_cx_min/max are the UNROUNDED per-run extrema
        # capability.py records (vs. first_channel's cx/cy, which are each
        # round(..., 5) -- see _channels_for_layer): prefer them wherever an
        # exact float comparison is being made. Since every layer in this
        # small, constant-tube fixture detects the identical channel, min
        # and max coincide and either is "the" channel's true X.
        self.assertAlmostEqual(
            alignment["channel_cx_min"], alignment["channel_cx_max"], places=9,
            msg="the channel's X wandered between layers on a constant-"
                "position tube -- the fixture, not the property, is broken",
        )
        return alignment

    def test_detected_channel_centre_matches_the_modelled_tube(self):
        alignment = self._channel_stats(hole_cx=-5.0)
        self.assertAlmostEqual(alignment["channel_cx_min"], -5.0, places=9)
        self.assertAlmostEqual(alignment["first_channel"]["cy"], self.HOLE_CY, places=5)

    def test_moving_the_tube_in_the_mesh_moves_the_detected_channel(self):
        near = self._channel_stats(hole_cx=-8.0)
        far = self._channel_stats(hole_cx=8.0)
        self.assertAlmostEqual(
            far["channel_cx_min"] - near["channel_cx_min"], 16.0, places=9,
        )
        self.assertAlmostEqual(
            near["first_channel"]["cy"], far["first_channel"]["cy"], places=5,
        )


class NoChannelIsALegitimateResultTests(M18MeshTransformTestCase):
    """A model with NO interior loop means a wing with no modelled spar --
    the ordinary case for most layers of most wings -- and must NOT raise.
    Only a genuinely UNREADABLE mesh (see ``FailClosedOnUnreadableMeshTests``
    above) is fail-closed; an honestly-sectioned, hole-free mesh is not."""

    def test_plain_solid_model_succeeds_with_no_socket(self):
        n_layers = 6
        obj = fake_orca.make_rect_object("wing", n_layers, 20.0, 20.0, LAYER_HEIGHT_MM)

        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)
        self.assertEqual(
            cap._mesh_alignment["interior_loop_counts"], {"0": n_layers}
        )
        self.assertIsNone(cap._mesh_alignment["first_channel"])
        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 1)


# ---------------------------------------------------------------------------
# The case an X-only alignment check provably cannot see.
# ---------------------------------------------------------------------------


class YOnlyMisalignmentIsCaughtTests(M18MeshTransformTestCase):
    """A mesh that is correct in X and wrong in Y must fail the alignment guard.

    WHY THIS EXISTS. An alignment check comparing only ``min_x``/``max_x``
    cannot see a Y-only error, and ``center_y_mm`` of the detected channel
    decides which skin the socket's neck breaks through: a Y error puts the
    neck on the wrong surface while every X measurement still agrees
    perfectly. The check must compare Y and centroid terms too. (A Y/centroid
    branch guarded by ``getattr(frame, "points", None)`` would be dead,
    because ``LayerFrame`` has no ``points`` attribute, silently degrading the
    check to X-only while the report claimed six terms were live.)

    A pure Y TRANSLATION of the whole mesh cannot build this case: XY-centring
    in capability.py subtracts the mesh's own XY bbox centre, which erases any
    uniform translation by construction (see ``_build_mesh_sectioner``'s
    docstring; the same translation is equally invisible to the host's own
    centring). So the Y-only error here is a Y-only SIZE change instead (the
    mesh's far Y edge moved, its near edge did not) -- a genuine shape
    difference, which centring cannot hide, while X stays untouched.
    """

    WIDTH_MM = 60.0
    DEPTH_MM = 10.0

    def _object_with_y_shifted_mesh(self, y_error_mm):
        """Host slice is the box's ALIGNED (``y_error_mm == 0``) centred
        cross-section, held fixed; the mesh's far Y edge grows by
        ``y_error_mm`` (a genuine height mismatch, not a translation --
        see class docstring), leaving X identical at every ``y_error_mm``.
        """
        width, depth = self.WIDTH_MM, self.DEPTH_MM
        top_z = LAYER_HEIGHT_MM * N_LAYERS + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(0.0, 0.0, 0.0,
                                     width, depth + y_error_mm, top_z)
        model = fake_orca.ModelObject(
            volumes=[fake_orca.ModelVolume(mesh)],
            instances=[fake_orca.ModelInstance()],
        )
        return fake_orca.make_rect_object(
            1, N_LAYERS, width, depth, LAYER_HEIGHT_MM,
            min_x_mm=-width / 2.0, min_y_mm=-depth / 2.0,
            model_object=model,
        )

    def test_y_shift_far_above_tolerance_fails_the_slice(self):
        # 0.5 mm is 25x the 0.02 mm tolerance and, critically, leaves both X
        # bounds exactly right -- an X-only check reports 0.0 for this.
        obj = self._object_with_y_shifted_mesh(0.5)
        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(
            result.status, PluginResult.FatalError,
            "a 0.5 mm Y misalignment between the model mesh and the host's "
            "own slice was accepted; the alignment guard is not comparing Y",
        )
        self.assertIn("disagrees with the host", result.message)

        # The X terms must be clean -- that is what makes this a Y-ONLY case,
        # and therefore what makes the test prove anything. Asserted
        # unconditionally: an `if worst:` guard would be a silent-skip shape
        # that hides the very assertions that give this test its meaning.
        worst = cap._mesh_alignment["worst_by_term"]
        self.assertLess(worst["min_x"], 1e-6)
        self.assertLess(worst["max_x"], 1e-6)

    def test_an_aligned_mesh_still_passes(self):
        """The control. Without it, a guard that rejected everything would
        pass the test above."""
        obj = self._object_with_y_shifted_mesh(0.0)
        result, _cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success,
            result.message)


# ---------------------------------------------------------------------------
# An arbitrary CAD origin must align after capability.py subtracts the mesh's
# own XY bounding-box centre -- and a genuinely wrong mesh must still fail
# closed under the same code path.
# ---------------------------------------------------------------------------


class OffCentreMeshMatchesHostAfterXYCentringFixTests(M18MeshTransformTestCase):
    """A mesh authored at an arbitrary CAD origin aligns, as in a real in-host
    failure: ``worst_by_term`` centroid_x 2.66950, centroid_y 40.94296 and
    ``mesh_bounds_after_transform`` x [-10.840, 5.501], y [-66.534, 148.420]
    -- an arbitrary CAD origin, not a rotation, scale, mirror, or
    instance-translation issue (each of those has its own test classes in
    this file). Without the mesh's own XY bbox-centring step, a mesh authored
    off-centre like this would fail the 0.02 mm alignment guard by roughly
    half its own width/height. With it, the same off-centre authoring aligns
    to floating-point precision, because centring depends only on the mesh's
    own shape, never on where it happened to be drawn.
    """

    def _object_with_off_centre_mesh(self, x_origin_mm, y_origin_mm):
        """A box mesh authored with its own local origin at an ARBITRARY, non-zero
        CAD coordinate ``(x_origin_mm, y_origin_mm)`` -- the model was never
        re-centred at (0, 0) before being loaded. No instance/volume rotation,
        scale, mirror, or translation is involved: this isolates the
        XY-centring from every other transform component."""
        width, depth = 40.0, 15.0
        top_z = LAYER_HEIGHT_MM * N_LAYERS + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(
            x_origin_mm, y_origin_mm, 0.0,
            x_origin_mm + width, y_origin_mm + depth, top_z,
        )
        model_object = fake_orca.identity_model_object(mesh)
        # The host's own slice is the box's CENTRED cross-section -- what a
        # real host reports regardless of the mesh's raw CAD origin.
        return fake_orca.make_rect_object(
            "wing", N_LAYERS, width, depth, LAYER_HEIGHT_MM,
            min_x_mm=-width / 2.0, min_y_mm=-depth / 2.0,
            model_object=model_object,
        )

    def test_arbitrary_cad_origin_aligns_after_the_fix(self):
        # Values deliberately far from (0, 0) and from each other in X vs Y,
        # mirroring the asymmetry of a real in-host report (X offset ~2.7 mm,
        # Y offset ~40.9 mm -- different magnitudes on each axis).
        obj = self._object_with_off_centre_mesh(
            x_origin_mm=-137.4, y_origin_mm=812.9,
        )
        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)

        alignment = cap._mesh_alignment
        self.assertLess(
            alignment["worst_bbox_delta_mm"], 1e-6,
            "an off-centre CAD origin leaked into the mesh cross-section "
            "(delta {} mm) -- this is the user's real failure signature".format(
                alignment["worst_bbox_delta_mm"]
            ),
        )
        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 1)

    def test_the_reproduction_above_is_not_vacuous(self):
        """Plain arithmetic on four literals (the raw mesh's authored corners
        vs. the host's centred frame), computed independently of
        ``capability.py`` entirely. It shows that the delta between this
        off-centre mesh's raw (uncentred) corners and the host's own centred
        frame exceeds the 0.02 mm tolerance, so the test above is not
        trivially passing because the case was already aligned. It does not
        replay the transform; without centring, this input is refused with
        "disagrees with the host's own slice by 820.40000 mm on centroid_y".
        """
        x_origin_mm, y_origin_mm = -137.4, 812.9
        width, depth = 40.0, 15.0
        # Without centring: raw mesh coordinates, uncentred.
        old_x0, old_x1 = x_origin_mm, x_origin_mm + width
        old_y0, old_y1 = y_origin_mm, y_origin_mm + depth
        # The host's own (centred) frame.
        host_x0, host_x1 = -width / 2.0, width / 2.0
        host_y0, host_y1 = -depth / 2.0, depth / 2.0

        worst_delta = max(
            abs(old_x0 - host_x0), abs(old_x1 - host_x1),
            abs(old_y0 - host_y0), abs(old_y1 - host_y1),
        )
        self.assertGreater(
            worst_delta, MESH_ALIGNMENT_TOLERANCE_MM_FOR_TEST,
            "the pre-fix (uncentred) transform would NOT have failed this "
            "case -- the reproduction above would be meaningless",
        )


# Mirrors capability.py's MESH_ALIGNMENT_TOLERANCE_MM (0.02 mm) -- imported
# by value, not by reference, since the fake host does not expose the
# bundle's module-level constants directly to this test file.
MESH_ALIGNMENT_TOLERANCE_MM_FOR_TEST = 0.02


class GenuinelyWrongModelStillFailsClosedTests(M18MeshTransformTestCase):
    """The XY-centring fix must not defang the guard: a mesh that is
    actually the WRONG SHAPE (not merely off-centre) -- as if the wrong
    model file were loaded entirely -- must still fail closed, since a
    size mismatch survives centring (only a pure translation is erased by
    it; see this file's other classes for why)."""

    def test_wrong_sized_mesh_still_fails_the_alignment_guard(self):
        # Host rect: 40 x 15, centred. Mesh: a DIFFERENT box, 40 x 30,
        # also centred -- so this is not a translation at all, only a size
        # mismatch, the case centring cannot and must not hide.
        width, depth = 40.0, 15.0
        wrong_depth = 30.0
        top_z = LAYER_HEIGHT_MM * N_LAYERS + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(
            -width / 2.0, -wrong_depth / 2.0, 0.0,
            width / 2.0, wrong_depth / 2.0, top_z,
        )
        model_object = fake_orca.identity_model_object(mesh)
        obj = fake_orca.make_rect_object(
            "wing", N_LAYERS, width, depth, LAYER_HEIGHT_MM,
            min_x_mm=-width / 2.0, min_y_mm=-depth / 2.0,
            model_object=model_object,
        )

        result, _cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("disagrees with the host", result.message)


# ---------------------------------------------------------------------------
# Safety argument for XY-centring, proven rather than assumed: after centring,
# a mirror (or an odd rotation) is detectable ONLY through the centroid terms,
# since a mirrored box is bbox-identical to an unmirrored one. "The guard would
# catch a mirror" must be demonstrated on a shape a mirror actually changes,
# not assumed from a symmetric box; the centroid terms are only live if
# ``frame.points`` (or an equivalent source) actually feeds them.
# ---------------------------------------------------------------------------


class MirrorDetectionRestsOnCentroidTermsTests(M18MeshTransformTestCase):
    """A box mesh cannot prove the alignment guard catches an instance
    mirror: a plain rectangle is symmetric about its own centreline, so
    mirroring it in Y produces the IDENTICAL bbox and (by symmetry) the
    identical centroid too -- a bug that silently dropped the mirror
    entirely would still pass. A CAMBERED section (a real NACA 2412
    aerofoil, not symmetric top-to-bottom) does not have that blind spot:
    mirroring it in Y moves the camber to the other side, which moves the
    area-weighted centroid_y by a real amount while leaving all four bbox
    edges UNCHANGED (top/bottom extent is the same, just which side is
    fuller) -- which is why the guard compares six terms, not four.
    """

    N_LAYERS = 5

    def _build_object(self, i_mir):
        outline_mm, _bbox = fake_orca.naca2412_outline_mm(
            chord_mm=60.0, span_mm=60.0, n_points=120
        )
        scaled_pts = tuple(
            (fake_orca.to_scaled(x), fake_orca.to_scaled(y)) for x, y in outline_mm
        )
        top_z = LAYER_HEIGHT_MM * self.N_LAYERS + LAYER_HEIGHT_MM

        layers = []
        for i in range(self.N_LAYERS):
            print_z = round(LAYER_HEIGHT_MM * (i + 1), 6)
            slice_z = round(print_z - LAYER_HEIGHT_MM / 2.0, 6)
            expoly = fake_orca.ExPolygon._wrap(fake_orca._Polygon(pts=scaled_pts))
            surface = fake_orca.Surface(fake_orca.SurfaceType.stInternal, expoly)
            region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([surface]))
            layers.append(fake_orca.Layer(print_z, [region], slice_z=slice_z))

        # Constant section (root outline == tip outline): a 2-station loft
        # is exact for every slice_z in between, same reasoning
        # make_airfoil_object's own docstring gives.
        mesh = fake_orca.loft_mesh_mm([(0.0, outline_mm), (top_z, outline_mm)])
        volume = fake_orca.ModelVolume(mesh)
        instance = fake_orca.ModelInstance(mirror=i_mir)
        model_object = fake_orca.ModelObject([volume], [instance])
        return fake_orca.PrintObject("wing", layers, model_object=model_object)

    def test_y_mirrored_camber_fails_on_centroid_y_alone(self):
        obj = self._build_object(i_mir=(1.0, -1.0, 1.0))

        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        # Specifically the alignment guard, not some other fail-closed path.
        self.assertIn("disagrees with the host", result.message)

        worst = cap._mesh_alignment["worst_by_term"]
        # All four bbox edges must be CLEAN -- a mirrored camber does not
        # move the bounding box at all, which is exactly why this is the
        # case an X/Y-bbox-only check cannot see.
        self.assertLess(worst["min_x"], 1e-6)
        self.assertLess(worst["max_x"], 1e-6)
        self.assertLess(worst["min_y"], 1e-6)
        self.assertLess(worst["max_y"], 1e-6)
        # The mirror is caught ONLY here.
        self.assertGreater(worst["centroid_y"], MESH_ALIGNMENT_TOLERANCE_MM_FOR_TEST)

    def test_unmirrored_camber_is_the_control_and_passes(self):
        """Without this, a guard that rejected every cambered section
        (mirrored or not) would pass the test above for the wrong reason."""
        obj = self._build_object(i_mir=(1.0, 1.0, 1.0))

        result, _cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message
        )


# ---------------------------------------------------------------------------
# Near-top alignment: a real in-host wing failed on its FINAL layer (1124/1125,
# mesh top Z=225.097 mm, layer print_z 224.99999999999577 mm) -- 0.04728 mm
# centroid_x disagreement against the 0.02 mm tolerance, with every bbox term
# between 5e-5 and 4.5e-3 mm. The mesh sectioned 1120/1120 layers with no
# failure and 1119 of 1125 layers had already mutated cleanly through the SAME
# transform, including this layer's own bbox terms staying inside tolerance.
# THE MECHANISM IS NOT ESTABLISHED; ``tip_alignment_skip_enabled`` is a
# bounded, OPT-IN mitigation, not a fix -- a layer this affects is left
# UNMUTATED (never accepted with a term quietly excluded from the decision)
# and the four bbox terms gate unconditionally regardless of the flag. See
# capability.py's module comment above ``THETA_EPSILON_DEG`` for the full
# argument, and ``StructureSpec.tip_alignment_skip_enabled`` for the exact
# opt-in semantics. These tests confine a CONTROLLED mismatch to exactly the
# layer nearest the mesh's own TOP Z, so the guard's near-top behaviour is
# exercised directly instead of inferred from a real wing's own geometry.
#
# FIXTURE NOTE: the mesh's top face sits HALF a layer height above the last
# layer's own print_z (``top_z = N*h + 0.5*h``), not a full layer height --
# that is the tightest position the host can actually produce
# (``print_z == object_top - 0.5h``, see the band derivation), comfortably
# inside the one-layer-height band without landing on its ``<=`` boundary.
# ---------------------------------------------------------------------------


class NearTopCentroidMismatchTests(M18MeshTransformTestCase):
    """A constant-section cambered mesh, with a host/mesh mismatch confined
    to ONE layer. Off that layer, mesh and host agree exactly -- so any
    failure or pass is attributable to that one layer's own position
    relative to the mesh's own top Z, nothing else about the fixture.
    """

    N_LAYERS = 5
    # Realistic tightest-host-case offset above the last layer's own
    # print_z (0.5 * layer height, not a full layer height -- see the
    # section comment above). Distance from the last layer (print_z 1.0 mm)
    # to the mesh top (1.1 mm) is 0.1 mm, inside the 0.2 mm band (layer_height
    # == initial_layer_print_height == 0.2 mm in FULL_CONFIG) and off its
    # boundary. Distance from the middle layer (print_z 0.6 mm) is 0.5 mm,
    # well outside the band.
    TOP_Z = LAYER_HEIGHT_MM * N_LAYERS + 0.5 * LAYER_HEIGHT_MM

    def _outline(self):
        outline_mm, _bbox = fake_orca.naca2412_outline_mm(
            chord_mm=60.0, span_mm=60.0, n_points=120
        )
        return outline_mm

    def _scaled_points(self, outline_mm):
        return tuple(
            (fake_orca.to_scaled(x), fake_orca.to_scaled(y)) for x, y in outline_mm
        )

    def _build_object(self, mismatch_layer_index):
        outline_mm = self._outline()
        # Centred-on-own-bbox construction (make_fixtures.airfoil_triangles)
        # means ymin == -ymax exactly, so negating y swaps the two bbox
        # edges onto each other -- the bbox is unchanged by this mirror,
        # exactly like ``MirrorDetectionRestsOnCentroidTermsTests`` relies
        # on, but applied here to the HOST side instead of the mesh side so
        # the mismatch can be confined to one layer.
        mirrored_mm = [(x, -y) for x, y in outline_mm]
        pts = self._scaled_points(outline_mm)
        pts_mirrored = self._scaled_points(mirrored_mm)

        layers = []
        for i in range(self.N_LAYERS):
            print_z = round(LAYER_HEIGHT_MM * (i + 1), 6)
            slice_z = round(print_z - LAYER_HEIGHT_MM / 2.0, 6)
            use_pts = pts_mirrored if i == mismatch_layer_index else pts
            expoly = fake_orca.ExPolygon._wrap(fake_orca._Polygon(pts=use_pts))
            surface = fake_orca.Surface(fake_orca.SurfaceType.stInternal, expoly)
            region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([surface]))
            layers.append(fake_orca.Layer(print_z, [region], slice_z=slice_z))

        # The MESH is the constant, UNMIRRORED section at every Z -- only
        # the host's own slice differs, and only on the targeted layer.
        mesh = fake_orca.loft_mesh_mm([(0.0, outline_mm), (self.TOP_Z, outline_mm)])
        volume = fake_orca.ModelVolume(mesh)
        instance = fake_orca.ModelInstance()
        model_object = fake_orca.ModelObject([volume], [instance])
        return fake_orca.PrintObject("wing", layers, model_object=model_object)

    def _assert_centroid_only_mismatch(self, alignment):
        self.assertGreater(
            alignment["worst_by_term"]["centroid_y"],
            MESH_ALIGNMENT_TOLERANCE_MM_FOR_TEST,
        )
        for term in ("min_x", "max_x", "min_y", "max_y"):
            self.assertLess(alignment["worst_by_term"][term], 1e-6)

    def test_flag_off_centroid_only_mismatch_at_top_layer_raises(self):
        """Default (``tip_alignment_skip_enabled`` unset/False): a
        centroid-only mismatch at the layer nearest the mesh's own top Z
        still fails the slice, and the message names the flag a user would
        need to opt into (``assertIn("tip_alignment_skip_enabled"...)``).
        """
        obj = self._build_object(mismatch_layer_index=self.N_LAYERS - 1)
        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("disagrees with the host", result.message)
        self.assertIn("tip_alignment_skip_enabled", result.message)

        self._assert_centroid_only_mismatch(cap._mesh_alignment)

    def test_flag_on_leaves_the_layer_unmutated_and_records_the_skip(self):
        """Opted in: the identical fixture succeeds, the mismatched
        (top) layer is left unmutated rather than cut from an unverified
        cross-section, and the skip is recorded loudly with all six
        deltas -- never silent."""
        obj = self._build_object(mismatch_layer_index=self.N_LAYERS - 1)
        result, cap = self.run_capability(
            obj,
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "tip_alignment_skip_enabled": True,
            },
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)
        self._assert_centroid_only_mismatch(cap._mesh_alignment)

        alignment = cap._mesh_alignment
        self.assertEqual(alignment["tip_alignment_skipped_layer_count"], 1)
        skipped = alignment["tip_alignment_skipped_layers"]
        self.assertEqual(len(skipped), 1)
        self.assertEqual(skipped[0]["layer"], self.N_LAYERS - 1)
        for term in ("min_x", "max_x", "min_y", "max_y", "centroid_x", "centroid_y"):
            self.assertIn(term, skipped[0]["deltas_mm"])

        # The report saying "mutated: False" is a claim about the DIAGNOSTIC
        # record; this is the independent claim about the HOST itself never
        # having been touched -- no slices.set()/make_slices() reached the
        # top layer, while every other layer mutated normally.
        host_layers = list(obj.layers())
        self.assertEqual(host_layers[self.N_LAYERS - 1].make_slices_calls, 0)
        for i in range(self.N_LAYERS - 1):
            self.assertEqual(host_layers[i].make_slices_calls, 1)

        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        rec = next(
            r for r in report["layers"] if r["layer_index"] == self.N_LAYERS - 1
        )
        self.assertFalse(rec["mutated"])
        self.assertFalse(rec["failed"])
        self.assertTrue(rec["shutdown"])
        self.assertEqual(rec["extra"]["reason"], "tip_alignment_skip")
        for term in ("min_x", "max_x", "min_y", "max_y", "centroid_x", "centroid_y"):
            self.assertIn(term, rec["extra"]["deltas_mm"])

    def test_the_same_mismatch_away_from_any_extremum_still_fails(self):
        """The control this class needs, flag ON: confine the IDENTICAL
        mismatch to a middle layer (index 2, distance 0.5 mm to the mesh's
        top against a 0.2 mm band) and confirm it still fails -- so the
        pass above is attributable to proximity to the top extremum, not to
        something else about this fixture or to the flag alone."""
        obj = self._build_object(mismatch_layer_index=2)
        result, _cap = self.run_capability(
            obj,
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "tip_alignment_skip_enabled": True,
            },
        )
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("disagrees with the host", result.message)

    def test_all_agree_control_passes_and_mutates(self):
        """No mismatch anywhere: an ordinary, fully-agreeing run still
        succeeds and still mutates every layer -- without this, a guard
        that skipped or rejected everything near the top would pass the
        tests above for the wrong reason."""
        obj = self._build_object(mismatch_layer_index=-1)
        result, _cap = self.run_capability(
            obj,
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "tip_alignment_skip_enabled": True,
            },
        )
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.Success, result.message)
        for layer in obj.layers():
            self.assertEqual(layer.make_slices_calls, 1)

    def test_band_prefers_layer_height_over_initial_layer_print_height(self):
        """``_resolve_extremum_band_mm`` must resolve to ``layer_height``
        alone when the two config values DIFFER, not
        ``max(layer_height, initial_layer_print_height)`` -- the band is
        consulted only at the top extremum, which ``layer_height`` governs;
        ``initial_layer_print_height`` governs the FIRST layer, at the
        root, and is only ever a fallback when ``layer_height`` itself does
        not resolve (see ``test_flag_on_with_unresolvable_band_still_
        raises`` for that path)."""
        obj = self._build_object(mismatch_layer_index=-1)
        host_config = dict(FULL_CONFIG)
        host_config["layer_height"] = 0.2
        host_config["initial_layer_print_height"] = 0.28

        result, cap = self.run_capability(
            obj,
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "tip_alignment_skip_enabled": True,
            },
            host_config=host_config,
        )
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message
        )
        self.assertEqual(cap._mesh_alignment["tip_alignment_band_mm"], 0.2)

    def test_flag_on_with_unresolvable_band_still_raises(self):
        """``_alignment_extremum_band_mm`` resolves to ``None`` when neither
        ``layer_height`` nor ``initial_layer_print_height`` is available --
        the guard must run at FULL STRICTNESS in that case, flag or no
        flag: an unresolved layer height must never make the guard MORE
        permissive."""
        obj = self._build_object(mismatch_layer_index=self.N_LAYERS - 1)
        host_config = dict(FULL_CONFIG)
        del host_config["layer_height"]
        del host_config["initial_layer_print_height"]

        result, cap = self.run_capability(
            obj,
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "tip_alignment_skip_enabled": True,
            },
            host_config=host_config,
        )
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("disagrees with the host", result.message)
        self.assertIsNone(cap._mesh_alignment["tip_alignment_band_mm"])


class NearTopBboxDisagreementStillFailsClosedTests(M18MeshTransformTestCase):
    """The top-extremum band must not become a hole: a section near the
    mesh's own top Z whose BBOX genuinely disagrees (not merely its
    centroid) must still fail, ``tip_alignment_skip_enabled`` or not. This
    is the test that stops the flag from being a general degrade-and
    -continue lever near a tip -- a real wrong-transform case does not stop
    mattering just because it happens to land on the last layer.

    This fixture's Y-extension ALSO moves the centroid (a rectangle with
    only its far edge extended is no longer symmetric about its own centre),
    so this test constrains its own claim: the failure must be traceable to
    the BBOX clause specifically, not merely to "some term disagreed" --
    verified below by asserting the centroid term is ALSO over tolerance
    (so a version that gated on the worst of all six terms would raise for
    the same INPUT, but not prove the bbox clause is the one doing the
    unconditional gating) while the message still names a bbox term and
    withholds the tip_alignment_skip_enabled opt-in wording that only the
    centroid-only clause ever prints.
    """

    N_LAYERS = 5
    # NOT 40.0/15.0: that box's mid-span (used by the mesh's own theta-axis
    # detection, unrelated to this test) hits an unrelated MeshSectioner edge
    # case ("8 segments produced 4 dangling endpoints") at that exact Z for
    # that exact box shape. Nearby box dimensions (36x14, 44x18, 50x20) all
    # section cleanly, so it is a narrow floating-point coincidence in the
    # segment assembly, not a systemic defect.
    WIDTH_MM = 36.0
    DEPTH_MM = 14.0
    TOP_Z = LAYER_HEIGHT_MM * N_LAYERS + 0.5 * LAYER_HEIGHT_MM

    def _build_object(self, mismatch_layer_index, y_error_mm=0.5):
        width, depth = self.WIDTH_MM, self.DEPTH_MM

        def rect_pts(half_w, y_lo, y_hi):
            return (
                (fake_orca.to_scaled(-half_w), fake_orca.to_scaled(y_lo)),
                (fake_orca.to_scaled(half_w), fake_orca.to_scaled(y_lo)),
                (fake_orca.to_scaled(half_w), fake_orca.to_scaled(y_hi)),
                (fake_orca.to_scaled(-half_w), fake_orca.to_scaled(y_hi)),
            )

        correct_pts = rect_pts(width / 2.0, -depth / 2.0, depth / 2.0)
        # A genuine SIZE mismatch (not a translation): the far Y edge moves
        # by ``y_error_mm``, changing max_y (a bbox-visible disagreement)
        # AND centroid_y (the shape is no longer symmetric about its own
        # centre) -- both terms disagree here, deliberately; see class
        # docstring for why the test below still constrains its own claim.
        wrong_pts = rect_pts(width / 2.0, -depth / 2.0, depth / 2.0 + y_error_mm)

        layers = []
        for i in range(self.N_LAYERS):
            print_z = round(LAYER_HEIGHT_MM * (i + 1), 6)
            slice_z = round(print_z - LAYER_HEIGHT_MM / 2.0, 6)
            use_pts = wrong_pts if i == mismatch_layer_index else correct_pts
            expoly = fake_orca.ExPolygon._wrap(fake_orca._Polygon(pts=use_pts))
            surface = fake_orca.Surface(fake_orca.SurfaceType.stInternal, expoly)
            region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([surface]))
            layers.append(fake_orca.Layer(print_z, [region], slice_z=slice_z))

        mesh = fake_orca.box_mesh_mm(
            -width / 2.0, -depth / 2.0, 0.0, width / 2.0, depth / 2.0, self.TOP_Z,
        )
        model_object = fake_orca.identity_model_object(mesh)
        return fake_orca.PrintObject("wing", layers, model_object=model_object)

    def test_bbox_disagreement_at_the_top_layer_still_fails_closed_flag_on(self):
        obj = self._build_object(mismatch_layer_index=self.N_LAYERS - 1)
        result, cap = self.run_capability(
            obj,
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "tip_alignment_skip_enabled": True,
            },
        )
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("disagrees with the host", result.message)
        self.assertIn("max_y", result.message)
        # The centroid term ALSO exceeded tolerance on this exact input, so a
        # passing raise here does not, by itself, prove the unconditional bbox
        # clause fired -- it might instead be a "worst of all six terms"
        # check that would raise for the same reason. Distinguish it by the
        # message: only the centroid-only clause ever names the opt-in flag.
        self.assertGreater(
            cap._mesh_alignment["worst_by_term"]["centroid_y"],
            MESH_ALIGNMENT_TOLERANCE_MM_FOR_TEST,
        )
        self.assertNotIn("tip_alignment_skip_enabled", result.message)

    def test_the_control_all_layers_agree_and_passes(self):
        obj = self._build_object(
            mismatch_layer_index=self.N_LAYERS - 1, y_error_mm=0.0
        )
        result, _cap = self.run_capability(
            obj,
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "tip_alignment_skip_enabled": True,
            },
        )
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message
        )


class SparSocketLatchTests(M18MeshTransformTestCase):
    """Once the spar socket has ENDED on a run, it stays ended. Drives
    ``_apply_spar_socket_latch`` directly.

    WHY DIRECTLY, and not through a sliced fixture: the latch only matters on a
    wing that thins and then THICKENS, and the fake host builds only
    constant-section prisms. Both channel fixtures are monotone, as is a real
    CAD flare, so no available end-to-end fixture can produce a resume. The
    latch matters because a message promising "the socket ENDS" must not be
    implemented as "the socket is dropped on this layer", which is a different
    thing exactly where it matters.
    """

    class _Intent(object):
        def __init__(self, family_name, x=1.0, spar_confirmed=True):
            self.family_name = family_name
            self.start = (x, 0.0)
            # ``_apply_spar_socket_latch``'s "resumed" filter also requires
            # ``spar_confirmed`` (see that method's own docstring). Most uses
            # of this mock represent an ordinary, unambiguous spar socket, so
            # the default here is ``True``.
            self.spar_confirmed = spar_confirmed

    def _socket_dropped(self):
        return [{"kind": "spar_socket", "x_mm": 1.0, "reason": "..."}]

    def test_the_latch_starts_disengaged(self):
        cap = self.new_capability()
        cap._spar_socket_ended = False
        intents = [self._Intent("spar_socket"), self._Intent("lattice")]
        kept, degs = cap._apply_spar_socket_latch(intents, [])
        self.assertEqual(kept, intents)
        self.assertEqual(degs, [])

    def test_a_dropped_socket_engages_the_latch(self):
        cap = self.new_capability()
        cap._spar_socket_ended = False
        cap._apply_spar_socket_latch([], self._socket_dropped())
        self.assertTrue(cap._spar_socket_ended)

    def test_a_socket_that_FITS_AGAIN_is_suppressed_and_reported(self):
        # THE test. Without the latch this socket would be re-cut, leaving
        # a blind interior pocket the rod cannot reach.
        cap = self.new_capability()
        cap._spar_socket_ended = True
        intents = [self._Intent("spar_socket", x=12.5),
                   self._Intent("lattice_plus_upper")]
        kept, degs = cap._apply_spar_socket_latch(intents, [])
        self.assertEqual([i.family_name for i in kept],
                         ["lattice_plus_upper"])
        self.assertEqual(len(degs), 1)
        self.assertEqual(degs[0]["kind"], "spar_socket_latched")
        self.assertEqual(degs[0]["x_mm"], 12.5)
        self.assertIn("blind interior pocket", degs[0]["reason"])

    def test_the_latch_does_not_touch_other_families(self):
        cap = self.new_capability()
        cap._spar_socket_ended = True
        intents = [self._Intent("lattice_plus_upper"),
                   self._Intent("deep_minus_lower")]
        kept, degs = cap._apply_spar_socket_latch(intents, [])
        self.assertEqual(len(kept), 2)
        self.assertEqual(degs, [], "no socket present, nothing to suppress")

    def test_the_latch_is_reset_at_the_run_boundary(self):
        # Staleness: the host REUSES a capability instance across slices, so
        # a latch left engaged would suppress the spar socket on the NEXT
        # wing, which never needed it suppressed.
        #
        # Drives ONE instance through two runs, which is what the host does;
        # setting the flag on a different instance than the one that runs
        # would pass for the wrong reason.
        obj = self.rect_object_with_instance_transform(2.0, 0.0, 62.0, 10.0)
        self.arm()
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"root_solid_layers": 0})

        cap.execute(self.make_ctx(obj))
        cap._spar_socket_ended = True          # as if this wing ended it

        cap.execute(self.make_ctx(obj))        # a second, unrelated slice
        self.assertFalse(
            cap._spar_socket_ended,
            "the latch leaked across runs: the next wing would print with "
            "its spar socket suppressed for no reason")


class MultiIslandLayerIsReportedTests(M18MeshTransformTestCase):
    """A layer with more than one disjoint island is structured on the largest
    only -- every other island prints as plain skin. That is allowed; being
    SILENT about it is not.

    Example from a real nubbed wing at Z=225.40, the transition between the
    aerofoil and the joiner post: two islands, the post (area 58.8) and a thin
    63 x 1.34 mm strip (area 53.2). The strip carried no structure and nothing
    said so.

    Driven directly against the capability's own surface list, because no
    fixture builder produces a two-island layer.
    """

    def _two_island_layer(self, obj):
        fo = fake_orca
        layer = obj.layers()[0]
        region = layer.regions()[0]
        first = region.slices.surfaces[0]
        b = fo.describe(first.expolygon)["bounds"]
        w = b[2] - b[0]
        # A second, disjoint island well clear of the first.
        shifted = fo.ExPolygon._wrap(
            fo._Rect(b[0] + 2 * w, b[1], b[2] + 2 * w, b[3]))
        region.slices.set(
            [first, self._fake_orca.host.Surface(first.surface_type, shifted)])
        return layer

    def test_a_two_island_layer_is_reported_not_silently_skipped(self):
        obj = self.rect_object_with_instance_transform(2.0, 0.0, 62.0, 10.0)
        self.arm()
        self._two_island_layer(obj)
        result, _cap = self.run_capability(obj, {"root_solid_layers": 0})

        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        kinds = [
            d["kind"]
            for lr in report["layers"] for d in lr.get("degradations", [])
        ]
        # The layer DEGRADES (unmutated, reported) rather than failing:
        # reporting alone is not enough, because the validator sees every
        # island and invariant 1 would refuse with "produced N material
        # islands" before any report is written.
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success,
            result.message)
        self.assertIn(
            "multi_island_layer", kinds,
            "a multi-island layer must degrade AND be reported. Result "
            "was: {}".format(result.message))

        reason = " ".join(
            d["reason"]
            for lr in report["layers"] for d in lr.get("degradations", [])
            if d["kind"] == "multi_island_layer"
        )
        self.assertIn("disjoint material islands", reason)
        self.assertIn("plain skin", reason)
        self.assertIn("rather than failing the slice", reason)


# ---------------------------------------------------------------------------
# Per-layer alignment diagnostics (all six deltas for EVERY sectioned layer,
# plus contour dumps for near-tolerance/failing layers). DIAGNOSTIC-ONLY --
# nothing here changes the guard, the tolerance, or slicing behaviour; only
# what the report contains.


class PerLayerAlignmentDiagnosticsTests(M18MeshTransformTestCase):
    """A constant-section cambered mesh, with a host/mesh mismatch of a
    PRECISELY controlled magnitude confined to one layer -- a uniform Y
    translation of the host slice only (mesh unchanged), which shifts
    ``min_y``, ``max_y`` and ``centroid_y`` by EXACTLY the same amount
    (``dy_mm``) and leaves every X term at exactly zero. Unlike the
    full-mirror fixture ``NearTopCentroidMismatchTests`` uses (which
    produces a large, imprecise delta good only for "clearly over
    tolerance"), this lets a test target an EXACT delta relative to
    ``MESH_ALIGNMENT_TOLERANCE_MM_FOR_TEST`` (0.02 mm): comfortably below
    half-tolerance, between half-tolerance and tolerance (a "near miss"
    that still passes), or above tolerance (fails unconditionally via the
    bbox clause, regardless of proximity to the mesh's own top Z).
    """

    N_LAYERS = 5
    TOP_Z = LAYER_HEIGHT_MM * N_LAYERS + 0.5 * LAYER_HEIGHT_MM

    def _outline(self):
        outline_mm, _bbox = fake_orca.naca2412_outline_mm(
            chord_mm=60.0, span_mm=60.0, n_points=120
        )
        return outline_mm

    def _scaled_points(self, outline_mm):
        return tuple(
            (fake_orca.to_scaled(x), fake_orca.to_scaled(y)) for x, y in outline_mm
        )

    def _build_object(self, mismatch_layer_index, dy_mm):
        outline_mm = self._outline()
        shifted_mm = [(x, y + dy_mm) for x, y in outline_mm]
        pts = self._scaled_points(outline_mm)
        pts_shifted = self._scaled_points(shifted_mm)

        layers = []
        for i in range(self.N_LAYERS):
            print_z = round(LAYER_HEIGHT_MM * (i + 1), 6)
            slice_z = round(print_z - LAYER_HEIGHT_MM / 2.0, 6)
            use_pts = pts_shifted if i == mismatch_layer_index else pts
            expoly = fake_orca.ExPolygon._wrap(fake_orca._Polygon(pts=use_pts))
            surface = fake_orca.Surface(fake_orca.SurfaceType.stInternal, expoly)
            region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([surface]))
            layers.append(fake_orca.Layer(print_z, [region], slice_z=slice_z))

        # The MESH is the constant, UNSHIFTED section at every Z -- only the
        # host's own slice differs, and only on the targeted layer.
        mesh = fake_orca.loft_mesh_mm([(0.0, outline_mm), (self.TOP_Z, outline_mm)])
        volume = fake_orca.ModelVolume(mesh)
        instance = fake_orca.ModelInstance()
        model_object = fake_orca.ModelObject([volume], [instance])
        return fake_orca.PrintObject("wing", layers, model_object=model_object)

    def _run(self, mismatch_layer_index, dy_mm, tip_alignment_skip_enabled=False):
        obj = self._build_object(mismatch_layer_index, dy_mm)
        return self.run_capability(
            obj,
            {
                "root_solid_layers": 0,
                "skin_margin_mm": 1.0,
                "tip_alignment_skip_enabled": tip_alignment_skip_enabled,
            },
        ) + (obj,)

    def test_all_layer_alignment_deltas_has_one_entry_per_sectioned_layer(self):
        """Every sectioned layer gets an entry, regardless of mismatch --
        unlike ``near_top_alignment_deltas``, which is windowed."""
        result, cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message
        )
        all_deltas = cap._mesh_alignment["all_layer_alignment_deltas"]
        self.assertEqual(len(all_deltas), self.N_LAYERS)
        self.assertEqual(
            sorted(d["layer"] for d in all_deltas), list(range(self.N_LAYERS))
        )
        for entry in all_deltas:
            for term in (
                "min_x", "max_x", "min_y", "max_y", "centroid_x", "centroid_y",
            ):
                self.assertIn(term, entry["deltas_mm"])

    def test_a_near_tolerance_layer_produces_a_contour_dump(self):
        """0.015 mm sits between half-tolerance (0.01) and tolerance (0.02):
        the layer still PASSES (no term individually exceeds tolerance) but
        is close enough that a dump is recorded."""
        target_layer = 2
        result, cap, _obj = self._run(mismatch_layer_index=target_layer, dy_mm=0.015)
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message
        )
        dumps = cap._mesh_alignment["alignment_contour_dumps"]
        self.assertEqual(len(dumps), 1)
        dump = dumps[0]
        self.assertEqual(dump["layer"], target_layer)
        self.assertFalse(dump["exceeds_tolerance"])
        self.assertAlmostEqual(dump["deltas_mm"]["centroid_y"], 0.015, places=4)
        self.assertIn("best_loop_points_mm", dump)
        self.assertIn("frame_pts_mm", dump)
        self.assertGreater(len(dump["best_loop_points_mm"]), 0)
        self.assertGreater(len(dump["frame_pts_mm"]), 0)
        self.assertIn("slice_z_used", dump)
        self.assertIn("print_z", dump)

    def test_a_comfortably_passing_layer_produces_no_dump(self):
        """0.001 mm is well under half-tolerance: no dump, on any layer."""
        result, cap, _obj = self._run(mismatch_layer_index=2, dy_mm=0.001)
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message
        )
        self.assertEqual(cap._mesh_alignment["alignment_contour_dumps"], [])

    def test_a_failing_layer_still_raises_and_its_dump_is_present(self):
        """0.03 mm mm exceeds tolerance outright (bbox clause, unconditional):
        the slice still fails, but the failing layer's own dump reaches the
        report written on the TopologyError path -- ``_write_diagnostics``
        runs BEFORE the FatalError is returned, and the dump is written
        into ``stats`` before this method's own raise."""
        target_layer = 2
        result, _cap, _obj = self._run(mismatch_layer_index=target_layer, dy_mm=0.03)
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        self.assertIn("disagrees with the host", result.message)

        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        dumps = report["mesh_alignment"]["alignment_contour_dumps"]
        matching = [d for d in dumps if d["layer"] == target_layer]
        self.assertEqual(len(matching), 1, dumps)
        self.assertTrue(matching[0]["exceeds_tolerance"])

    def test_near_top_alignment_deltas_stays_windowed_and_capped_unchanged(self):
        """Pin: ``near_top_alignment_deltas`` keeps its EXISTING windowed
        (~10x the extremum band) behaviour, unaffected by the new
        unconditional ``all_layer_alignment_deltas`` list -- a layer far
        from the mesh's own top Z (index 0, distance ~0.9 mm against a
        ~2.0 mm 10x-band window here, still inside it in this small
        fixture) is covered by both lists identically in a run this short;
        the two lists must not have started sharing state or a cap."""
        result, cap, _obj = self._run(mismatch_layer_index=-1, dy_mm=0.0)
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message
        )
        alignment = cap._mesh_alignment
        near_top = alignment["near_top_alignment_deltas"]
        all_deltas = alignment["all_layer_alignment_deltas"]
        # Both lists are populated (existing near_top behaviour untouched),
        # near_top keeps its own 40-entry cap (irrelevant at N_LAYERS==5,
        # but the field must still exist and be a plain list of dicts, not
        # repurposed into anything the new field owns).
        self.assertLessEqual(len(near_top), 40)
        self.assertGreater(len(near_top), 0)
        self.assertEqual(len(all_deltas), self.N_LAYERS)
        for entry in near_top:
            self.assertIn("distance_to_mesh_top_mm", entry)
            self.assertIn("deltas_mm", entry)

    def test_geometry_at_failure_carries_the_failing_layers_own_geometry(self):
        """``report["geometry_at_failure"]`` describes the layer named in the
        raise, not whichever EARLIER layer last completed its full
        ``self._last_geometry`` assignment. ``_check_mesh_alignment`` can
        raise ``TopologyError`` BEFORE that assignment (the one carrying
        ``frame_bounds``) is reached for the failing layer. Layers 0-2 here
        succeed and fully write ``_last_geometry``; layer 3 (index == 3,
        MISMATCH_LAYER_INDEX) is shifted 0.03 mm in Y, comfortably over the
        0.02 mm default tolerance, and fails the UNCONDITIONAL bbox clause
        (Clause 2) regardless of its distance from the mesh's own top Z.

        The dict carries a ``layer_index`` field so staleness is detectable
        by value: a stale report would show layer 2's OWN unshifted ``min_y``
        (baseline, no offset) as layer 3's ``frame_bounds[1]`` -- a different
        layer's geometry silently attributed to the failing layer.
        """
        mismatch_layer_index = 3
        dy_mm = 0.03

        # Baseline: an all-unshifted run's OWN frame min_y at any layer
        # (every layer uses the identical unshifted outline, so this is a
        # true constant, not layer-3-specific) -- the value a stale report
        # would wrongly give for layer 3's own bounds.
        baseline_result, baseline_cap, _obj = self._run(
            mismatch_layer_index=-1, dy_mm=0.0
        )
        self.assertEqual(
            baseline_result.status, self._fake_orca.PluginResult.Success,
            baseline_result.message,
        )
        baseline_min_y = min(
            pt[1] for pt in baseline_cap._last_geometry["host_contour_pts_mm"]
        )

        result, _cap, _obj = self._run(
            mismatch_layer_index=mismatch_layer_index, dy_mm=dy_mm
        )
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        self.assertIn("disagrees with the host", result.message)

        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        geo = report.get("geometry_at_failure")
        self.assertIsNotNone(geo)

        # The field that makes staleness detectable at all: every
        # ``geometry_at_failure`` must name its OWN layer.
        self.assertEqual(geo["layer_index"], mismatch_layer_index)

        # The value itself must be layer 3's OWN (shifted) host contour --
        # not layer 2's (unshifted, == baseline) -- proving this is not
        # merely a correctly-LABELLED stale value.
        failing_min_y = min(pt[1] for pt in geo["host_contour_pts_mm"])
        self.assertAlmostEqual(failing_min_y, baseline_min_y + dy_mm, places=3)
        self.assertNotAlmostEqual(failing_min_y, baseline_min_y, places=3)

        # This raise fires INSIDE ``_check_mesh_alignment``, before the
        # layer's full ``_last_geometry`` assignment (the one carrying
        # ``intents``/``slot_points_mm``) is ever reached -- so ``complete``
        # honestly reads False, but the guard-comparison contours it DID
        # reach (host contour and our own section) are both present, in
        # the layer's OWN dict, not left over from layer 2's completed one.
        self.assertFalse(geo.get("complete"))
        self.assertIsNotNone(geo.get("host_contour_pts_mm"))
        self.assertIsNotNone(geo.get("our_section_pts_mm"))


class UnrecognisedDecisionOutcomeFailsClosedTests(PerLayerAlignmentDiagnosticsTests):
    """An outcome from ``decide_alignment`` that is not one of the four
    explicitly handled must fail closed in ``_check_mesh_alignment`` (not fall
    through to the "skip this layer" branch, and not be guarded by a bare
    ``assert``, which is a silent no-op under ``python -O``). Monkeypatches the
    bundle's own ``decide_alignment`` (a bare module-level name in the exec'd
    namespace, the same "supplied by the bundle" wiring every other
    aircraft_core name in ``capability.py`` uses) to return a made-up outcome,
    and asserts the slice fails closed naming the defect, never silently
    skipping."""

    def test_unrecognised_outcome_fails_closed_not_silently_skipped(self):
        obj = self._build_object(mismatch_layer_index=-1, dy_mm=0.0)

        original = self.namespace["decide_alignment"]

        def _bogus(deltas, effective, near_top_extremum, tip_alignment_skip_enabled):
            return {"outcome": "not_a_real_outcome"}

        self.namespace["decide_alignment"] = _bogus
        try:
            result, cap = self.run_capability(
                obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0}
            )
        finally:
            self.namespace["decide_alignment"] = original

        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        self.assertIn("internal error", result.message)
        self.assertIn("not_a_real_outcome", result.message)
        # NOT the skip-layer path: that path leaves diagnostics.mutated_any
        # False and would still report Success at the run level for a
        # clean-otherwise fixture -- FatalError above already rules that
        # out, checked again here against the written report directly.
        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        self.assertIn("internal error", report.get("error", ""))


# ---------------------------------------------------------------------------
# mesh_alignment_tolerance_mm config field: the alignment tolerance is a config
# field (default 0.02 mm), so a wing whose measured mesh/host disagreement sits
# just over 0.02 mm (e.g. 0.02479 mm) has a lever to proceed. This class pins
# the default and proves the field reaches the comparison -- raising it admits
# a layer that fails at the default, lowering it rejects a layer that passes at
# the default -- through the REAL config path (capability.py._build_spec), not
# aircraft_core.spec directly, since a field never wired into _build_spec would
# still pass an aircraft_core-only test while remaining inert from a
# user-facing config.
# ---------------------------------------------------------------------------


class MeshAlignmentToleranceConfigTests(M18MeshTransformTestCase):
    """A constant-section cambered mesh with a host/mesh mismatch of a
    PRECISELY controlled magnitude (a uniform Y shift of the host slice
    only) confined to ONE layer, chosen far from the mesh's own top Z so
    the unconditional bbox clause (Clause 2) is what fires -- independent
    of ``tip_alignment_skip_enabled``/the near-top band entirely. A rigid Y
    shift moves ``min_y``, ``max_y`` and ``centroid_y`` by EXACTLY
    ``dy_mm`` and leaves every X term at exactly zero, so the worst bbox
    delta equals ``dy_mm`` precisely -- letting each test target an exact
    delta relative to whatever tolerance is configured.
    """

    N_LAYERS = 5
    TOP_Z = LAYER_HEIGHT_MM * N_LAYERS + 0.5 * LAYER_HEIGHT_MM
    # Layer 0 (print_z == LAYER_HEIGHT_MM == 0.2 mm) sits 0.9 mm from
    # TOP_Z (1.1 mm) -- far outside the default one-layer-height band
    # (0.2 mm), so near_top_extremum is False and Clause 2 (bbox, always
    # unconditional) is the one that decides pass/fail here.
    MISMATCH_LAYER_INDEX = 0

    def _outline(self):
        outline_mm, _bbox = fake_orca.naca2412_outline_mm(
            chord_mm=60.0, span_mm=60.0, n_points=120
        )
        return outline_mm

    def _scaled_points(self, outline_mm):
        return tuple(
            (fake_orca.to_scaled(x), fake_orca.to_scaled(y)) for x, y in outline_mm
        )

    def _build_object(self, dy_mm):
        outline_mm = self._outline()
        shifted_mm = [(x, y + dy_mm) for x, y in outline_mm]
        pts = self._scaled_points(outline_mm)
        pts_shifted = self._scaled_points(shifted_mm)

        layers = []
        for i in range(self.N_LAYERS):
            print_z = round(LAYER_HEIGHT_MM * (i + 1), 6)
            slice_z = round(print_z - LAYER_HEIGHT_MM / 2.0, 6)
            use_pts = pts_shifted if i == self.MISMATCH_LAYER_INDEX else pts
            expoly = fake_orca.ExPolygon._wrap(fake_orca._Polygon(pts=use_pts))
            surface = fake_orca.Surface(fake_orca.SurfaceType.stInternal, expoly)
            region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([surface]))
            layers.append(fake_orca.Layer(print_z, [region], slice_z=slice_z))

        # The MESH is the constant, UNSHIFTED section at every Z -- only
        # the host's own slice differs, and only on the targeted layer.
        mesh = fake_orca.loft_mesh_mm([(0.0, outline_mm), (self.TOP_Z, outline_mm)])
        volume = fake_orca.ModelVolume(mesh)
        instance = fake_orca.ModelInstance()
        model_object = fake_orca.ModelObject([volume], [instance])
        return fake_orca.PrintObject("wing", layers, model_object=model_object)

    def _run(self, dy_mm, mesh_alignment_tolerance_mm=None):
        obj = self._build_object(dy_mm)
        cap_config = {"root_solid_layers": 0, "skin_margin_mm": 1.0}
        if mesh_alignment_tolerance_mm is not None:
            cap_config["mesh_alignment_tolerance_mm"] = mesh_alignment_tolerance_mm
        return self.run_capability(obj, cap_config)

    def test_default_tolerance_is_0_02_mm_pinned(self):
        """A 0.021 mm mismatch (just over the 0.02 mm default) still fails
        when the config omits the key entirely -- pinning the default at
        0.02 mm."""
        result, _cap = self._run(dy_mm=0.021)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn("disagrees with the host", result.message)

        result_pass, _cap2 = self._run(dy_mm=0.019)
        self.assertEqual(
            result_pass.status, PluginResult.Success, result_pass.message
        )

    def test_raised_tolerance_admits_a_layer_that_fails_at_default(self):
        """0.025 mm exceeds the 0.02 mm default (fails) but not a
        configured 0.03 mm tolerance (passes) -- proving the configured
        value, not the module constant, is what the comparison actually
        used."""
        failing, _cap = self._run(dy_mm=0.025)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(failing.status, PluginResult.FatalError)

        passing, cap2 = self._run(
            dy_mm=0.025, mesh_alignment_tolerance_mm=0.03
        )
        self.assertEqual(passing.status, PluginResult.Success, passing.message)

        # The near-miss threshold (spec.mesh_alignment_tolerance_mm / 2.0)
        # must also move with the configured value, not stay pinned to the
        # 0.02 mm default -- proving the ``/2`` site was converted too, not
        # just the two gating comparisons above. At tolerance 0.03, the
        # near-miss threshold is 0.015 mm; 0.025 mm clears it, so layer 0
        # gets a contour dump recorded as NOT exceeding tolerance.
        dumps = cap2._mesh_alignment["alignment_contour_dumps"]
        matching = [d for d in dumps if d["layer"] == self.MISMATCH_LAYER_INDEX]
        self.assertEqual(len(matching), 1, dumps)
        self.assertFalse(matching[0]["exceeds_tolerance"])

        # At tolerance 0.10, the near-miss threshold is 0.05 mm; 0.025 mm
        # does not clear it, so no dump is recorded for layer 0 at all --
        # confirming the threshold actually tracks the configured value
        # rather than a fixed fraction of the 0.02 mm default.
        passing_wide, cap3 = self._run(
            dy_mm=0.025, mesh_alignment_tolerance_mm=0.10
        )
        self.assertEqual(
            passing_wide.status, PluginResult.Success, passing_wide.message
        )
        dumps_wide = cap3._mesh_alignment["alignment_contour_dumps"]
        matching_wide = [
            d for d in dumps_wide if d["layer"] == self.MISMATCH_LAYER_INDEX
        ]
        self.assertEqual(matching_wide, [])

    def test_lowered_tolerance_rejects_a_layer_that_passes_at_default(self):
        """0.015 mm is within the 0.02 mm default (passes) but exceeds a
        configured 0.01 mm tolerance (fails), and the failure message
        itself names the CONFIGURED tolerance (0.01000 mm), not the 0.02 mm
        default -- proving the message's own format argument was converted,
        not just the gating comparison."""
        passing, _cap = self._run(dy_mm=0.015)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(passing.status, PluginResult.Success, passing.message)

        failing, _cap2 = self._run(
            dy_mm=0.015, mesh_alignment_tolerance_mm=0.01
        )
        self.assertEqual(failing.status, PluginResult.FatalError)
        self.assertIn("disagrees with the host", failing.message)
        self.assertIn("0.01000 mm tolerance", failing.message)
        self.assertNotIn("0.02000 mm tolerance", failing.message)

    # NOTE: test_raised_tolerance_admits_a_layer_that_fails_at_default and
    # test_lowered_tolerance_rejects_a_layer_that_passes_at_default above
    # ARE the config-path-through-_build_spec proof: ``run_capability``
    # only ever sets the stored config JSON ``cap.get_config`` reads (the
    # real ``capability.py`` path from stored config through
    # ``_build_spec`` to ``StructureSpec`` to ``_check_mesh_alignment``),
    # never ``StructureSpec`` directly -- so a value that changed the
    # outcome only if a config key actually reached the comparison is
    # exactly the "inert field" class of bug this guards against.


# ---------------------------------------------------------------------------
# Sensitivity-aware mesh-alignment tolerance (off by default, real
# ``StructureSpec``/config-schema field ``mesh_alignment_sensitivity_enabled``
# -- see capability.py's own comment above ``MESH_ALIGNMENT_SENSITIVITY_K`` for
# the full derivation, the sign gate, and the absolute ceiling).
#
# The fixture builds the HOST'S contour by directly re-running
# ``aircraft_core.mesh_section.MeshSectioner`` -- the exact class
# ``capability.py`` uses -- on the same welded mesh, replicating
# ``_build_mesh_sectioner``'s own XY-centring / Z-min-normalising step, so a
# layer with NO deliberate perturbation agrees with the plugin's own mesh-side
# computation to float precision (the zero-perturbation control passes with
# the field OFF, i.e. below the 0.02 mm base tolerance on every term, not
# merely "rescued"). A hand-interpolated host outline is not used: a per-vertex
# ``lerp`` between the two bounding taper stations leaves a sign-consistent
# ~0.033 mm residual at every taper layer, which cannot be told apart from
# "the sign gate correctly denies a rigid shift". Every perturbation below is
# therefore a KNOWN, chosen quantity -- a rigid (dx, dy) shift, or a
# controlled Z-sampling offset -- not a residual of unknown origin.
class MeshAlignmentSensitivityTests(M18MeshTransformTestCase):
    LH = 0.2
    N_LAYERS = 20
    SHIFT_LAYER_INDEX = 18  # print_z 3.8 mm -- inside the closing taper.

    def _stations(self):
        outline_mm, _bbox = fake_orca.naca2412_outline_mm(
            chord_mm=300.0, span_mm=300.0, n_points=200
        )
        cx = sum(p[0] for p in outline_mm) / len(outline_mm)
        cy = sum(p[1] for p in outline_mm) / len(outline_mm)

        def scaled(scale):
            return [
                (cx + (x - cx) * scale, cy + (y - cy) * scale)
                for x, y in outline_mm
            ]

        top_z = self.LH * self.N_LAYERS + self.LH
        return [
            (0.0, outline_mm),
            (self.LH * 17, outline_mm),
            (top_z, scaled(0.02)),
        ]

    def _local_sectioner(self, stations):
        """A ``MeshSectioner`` built from the SAME welded mesh
        ``fake_orca.loft_mesh_mm`` will hand the plugin, replicating
        ``capability.py._build_mesh_sectioner``'s own XY-centring
        (subtract the whole mesh's XY bbox centre) and Z-min-normalising
        step -- so sectioning THIS object at a given Z reproduces exactly
        what the plugin's own mesh-side comparison computes at that Z, for
        this fixture's identity instance placement (no rotation, no
        offset, so nothing else in the forward transform can differ).
        """
        mesh = fake_orca.loft_mesh_mm(stations)
        verts = [mesh.vertex(i) for i in range(mesh.vertex_count())]
        tris = [tuple(mesh.triangle(i)) for i in range(mesh.facets_count())]
        x_center = (min(v[0] for v in verts) + max(v[0] for v in verts)) / 2.0
        y_center = (min(v[1] for v in verts) + max(v[1] for v in verts)) / 2.0
        verts = [(v[0] - x_center, v[1] - y_center, v[2]) for v in verts]
        z_min = min(v[2] for v in verts)
        verts = [(v[0], v[1], v[2] - z_min) for v in verts]
        return MeshSectioner(verts, tris, z_bin_mm=1.0), mesh

    def _build_object(self, dx_mm=0.0, dy_mm=0.0, z_shift_mm=0.0):
        """``dx_mm``/``dy_mm``: a RIGID shift applied to the shift layer's
        host contour only -- the transform-bug proxy. ``z_shift_mm``:
        instead of shifting the CONTOUR, sample the host's contour at
        ``slice_z - z_shift_mm`` rather than ``slice_z`` -- a direct,
        controlled reproduction of the mechanism this feature targets (the
        mesh side effectively reads ``z + epsilon_mm``; this simulates the
        host reading a plane ``z_shift_mm`` EARLIER than the mesh, in the
        same units the module comment's own derivation uses). The two are
        mutually exclusive per test; nothing stops combining them, but no
        test here does.
        """
        stations = self._stations()
        sectioner, mesh = self._local_sectioner(stations)
        layers = []
        for i in range(self.N_LAYERS):
            print_z = round(self.LH * (i + 1), 6)
            slice_z = round(print_z - self.LH / 2.0, 6)
            query_z = slice_z
            if i == self.SHIFT_LAYER_INDEX and z_shift_mm:
                query_z = slice_z - z_shift_mm
            if not (sectioner.z_min < query_z < sectioner.z_max):
                continue
            loops = sectioner.section_at(query_z)
            outer = [lp for lp in loops if not lp.is_interior]
            best = max(outer, key=lambda lp: abs(lp.signed_area))
            outline = list(best.points)
            if i == self.SHIFT_LAYER_INDEX and (dx_mm or dy_mm):
                outline = [(x + dx_mm, y + dy_mm) for x, y in outline]
            pts = tuple(
                (fake_orca.to_scaled(x), fake_orca.to_scaled(y))
                for x, y in outline
            )
            expoly = fake_orca.ExPolygon._wrap(fake_orca._Polygon(pts=pts))
            surface = fake_orca.Surface(fake_orca.SurfaceType.stInternal, expoly)
            region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([surface]))
            layers.append(fake_orca.Layer(print_z, [region], slice_z=slice_z))

        volume = fake_orca.ModelVolume(mesh)
        instance = fake_orca.ModelInstance()
        model_object = fake_orca.ModelObject([volume], [instance])
        return fake_orca.PrintObject("wing", layers, model_object=model_object)

    def _run(self, dx_mm=0.0, dy_mm=0.0, z_shift_mm=0.0, sensitivity=False):
        obj = self._build_object(dx_mm=dx_mm, dy_mm=dy_mm, z_shift_mm=z_shift_mm)
        cap_config = {"root_solid_layers": 0, "skin_margin_mm": 1.0}
        if sensitivity:
            cap_config["mesh_alignment_sensitivity_enabled"] = True
        return self.run_capability(obj, cap_config)

    def _epsilon_mm(self):
        """This fixture's own ``MeshSectioner.epsilon_mm``, read the same
        way the module comment's derivation does -- never hard-coded, so a
        change to the fixture's chord/point count cannot silently make
        these tests describe a different epsilon than the one they use."""
        sectioner, _mesh = self._local_sectioner(self._stations())
        return sectioner.epsilon_mm

    # -- the clean baseline: zero perturbation, either flag setting --------

    def test_clean_baseline_passes_regardless_of_the_flag(self):
        """With NO deliberate perturbation, the host contour (built from
        this fixture's own ``MeshSectioner``) agrees with the plugin's own
        mesh-side computation to float precision -- well under the 0.02 mm
        base tolerance -- so the slice succeeds whether the field is off
        or on. Establishes that every other test's failure/rescue is
        caused by the perturbation it introduces, not by fixture noise."""
        PluginResult = self._fake_orca.PluginResult
        result_off, _cap = self._run(sensitivity=False)
        self.assertEqual(result_off.status, PluginResult.Success, result_off.message)
        result_on, _cap2 = self._run(sensitivity=True)
        self.assertEqual(result_on.status, PluginResult.Success, result_on.message)

    # -- the legitimate rescue: a controlled Z-sampling offset --------------

    def test_a_mechanism_consistent_z_offset_is_rescued_only_when_enabled(self):
        """Simulates the mechanism directly: the host's contour is sampled ``1x
        epsilon_mm`` EARLIER than the mesh at the shift layer -- the same order
        of magnitude ``MeshSectioner._section_at_exact``'s own ``eff_z = z +
        epsilon_mm`` nudge produces between two independent slicers of a
        closing tip (see the module comment's own artifact table). With the
        field off, this over-base disagreement fails exactly as any other bbox
        disagreement would; with it on, the fixture's own measured rate (~184
        mm/mm on ``min_x``/``max_x``) and this exact epsilon widen the
        effective tolerance past the simulated disagreement, and the slice
        succeeds."""
        eps = self._epsilon_mm()
        PluginResult = self._fake_orca.PluginResult
        result_off, _cap = self._run(z_shift_mm=eps, sensitivity=False)
        self.assertEqual(result_off.status, PluginResult.FatalError, result_off.message)
        self.assertIn("disagrees with the host", result_off.message)

        result_on, cap = self._run(z_shift_mm=eps, sensitivity=True)
        self.assertEqual(result_on.status, PluginResult.Success, result_on.message)
        stats = cap._mesh_alignment
        self.assertTrue(stats["sensitivity_enabled"])
        self.assertGreater(stats["sensitivity_widened_term_count"], 0)
        adjustments = stats["sensitivity_adjustments"]
        self.assertTrue(adjustments, "expected at least one recorded adjustment")
        widened = adjustments[0]["effective_tolerance_mm"]
        self.assertIn("min_x", widened)
        self.assertGreater(widened["min_x"], 0.02)

    def test_k_equals_2_is_the_measured_boundary_not_a_round_number(self):
        """At exactly 2x the simulated epsilon offset, the widened
        tolerance (``K=2 * rate * epsilon``) sits almost exactly ON the
        resulting disagreement (``rate * 2*epsilon``) and still admits it;
        at 3x, the disagreement (``rate * 3*epsilon``) exceeds even the
        widened tolerance and the slice fails. This is the K=2 headroom
        described in the module comment made concrete: one measured
        epsilon of margin above the 1x mechanism, not an arbitrary
        multiplier."""
        eps = self._epsilon_mm()
        PluginResult = self._fake_orca.PluginResult
        result_2x, _cap = self._run(z_shift_mm=2.0 * eps, sensitivity=True)
        self.assertEqual(result_2x.status, PluginResult.Success, result_2x.message)

        result_3x, _cap2 = self._run(z_shift_mm=3.0 * eps, sensitivity=True)
        self.assertEqual(result_3x.status, PluginResult.FatalError, result_3x.message)

    # -- the safety fix: a rigid shift is never granted the mechanism's ----
    # -- relaxation, in EITHER direction ------------------------------------

    def test_a_rigid_shift_is_never_rescued_in_either_direction(self):
        """THE SAFETY PROPERTY. A rigid (dx, dy) shift moves
        ``min_x`` and ``max_x`` by the SAME signed amount, but the mechanism
        predicts opposite signs for those two terms on a closing section (see
        the module comment's sign convention) -- so a rigid shift always
        matches the mechanism's predicted sign on at most one of the two edges
        and MISMATCHES it on the other. The sign gate denies the widening on
        whichever edge mismatches, in EITHER shift direction, so 0.025 mm --
        comfortably inside the ~0.0657 mm effective tolerance a matching-sign
        term would receive at this fixture's rate -- still fails on the
        mismatched edge alone."""
        PluginResult = self._fake_orca.PluginResult
        for dx in (0.025, -0.025):
            result, _cap = self._run(dx_mm=dx, sensitivity=True)
            self.assertEqual(
                result.status, PluginResult.FatalError,
                "dx={} mm was NOT caught: {}".format(dx, result.message),
            )
            self.assertIn("disagrees with the host", result.message)

    def test_a_large_rigid_shift_still_fails_closed_with_flag_on(self):
        """A grossly wrong transform (0.2 mm rigid X-shift) still fails
        the slice with the field on -- the sign gate and the absolute
        ceiling are two independent reasons this can never be admitted,
        not just a near-miss that happened not to clear a threshold."""
        result, _cap = self._run(dx_mm=0.2, sensitivity=True)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError, result.message)
        self.assertIn("disagrees with the host", result.message)

        result_off, _cap2 = self._run(dx_mm=0.2, sensitivity=False)
        self.assertEqual(result_off.status, PluginResult.FatalError, result_off.message)


class MeshAlignmentSensitivityConstantSectionControlTests(
    MeshAlignmentToleranceConfigTests
):
    """The EXISTING constant-section, rigid-Y-shift fixture
    (``MeshAlignmentToleranceConfigTests`` above) has a rate of exactly
    zero for every term at every layer -- a prismatic section does not
    change with Z at all. Re-running its own tests with the sensitivity
    marker file present must reproduce the identical verdicts: with
    rate == 0, ``effective == max(base, K * 0 * epsilon) == base`` for
    every term, so this class is a second, independent proof (distinct
    from the closing-taper fixture above, which HAS a nonzero rate) that a
    well-conditioned, non-tapering mismatch is never rescued -- only a
    genuinely fast-closing section can ever produce a wider effective
    tolerance.
    """

    def _run(self, dy_mm, mesh_alignment_tolerance_mm=None):
        obj = self._build_object(dy_mm)
        cap_config = {
            "root_solid_layers": 0,
            "skin_margin_mm": 1.0,
            "mesh_alignment_sensitivity_enabled": True,
        }
        if mesh_alignment_tolerance_mm is not None:
            cap_config["mesh_alignment_tolerance_mm"] = mesh_alignment_tolerance_mm
        return self.run_capability(obj, cap_config)


# ---------------------------------------------------------------------------
# The host's object/volume/instance transform, as the host actually exposed
# it, recorded on every run -- success or failure -- through
# ``self._host_object_transform`` / ``report["host_object_transform"]``.
# ---------------------------------------------------------------------------


class HostObjectTransformRecordedOnSuccessTests(M18MeshTransformTestCase):
    """A plain, symmetric box (so an instance rotation/mirror/scale does not
    itself need to defeat the alignment guard -- that is a separate, already
    -covered concern) exercised through the real ``execute()`` path, pinning
    that ``_build_mesh_sectioner`` records the RAW host-applied values for
    identity, rotated, mirrored (negative determinant) and scaled instances,
    plus the plugin-derived values alongside them."""

    WIDTH_MM = 20.0
    DEPTH_MM = 20.0
    N_LAYERS = 3

    def _run_with_instance(self, **inst_kwargs):
        half_w, half_d = self.WIDTH_MM / 2.0, self.DEPTH_MM / 2.0
        top_z = LAYER_HEIGHT_MM * self.N_LAYERS + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(-half_w, -half_d, 0.0, half_w, half_d, top_z)
        volume = fake_orca.ModelVolume(mesh)
        instance = fake_orca.ModelInstance(**inst_kwargs)
        model_object = fake_orca.ModelObject([volume], [instance])
        obj = fake_orca.make_rect_object(
            "wing", self.N_LAYERS, self.WIDTH_MM, self.DEPTH_MM, LAYER_HEIGHT_MM,
            min_x_mm=-half_w, min_y_mm=-half_d, model_object=model_object,
        )
        _result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )
        return cap

    def test_identity_instance_records_matching_raw_values(self):
        cap = self._run_with_instance()
        block = cap._host_object_transform
        self.assertTrue(block["read"])
        self.assertEqual(len(block["instances"]), 1)
        self.assertEqual(len(block["volumes"]), 1)
        inst_rec = block["instances"][0]
        self.assertEqual(inst_rec["offset"], (0.0, 0.0, 0.0))
        self.assertEqual(inst_rec["rotation"], (0.0, 0.0, 0.0))
        self.assertEqual(inst_rec["scaling_factor"], (1.0, 1.0, 1.0))
        self.assertEqual(inst_rec["mirror"], (1.0, 1.0, 1.0))
        self.assertEqual(inst_rec["determinant_sign"], 1)
        self.assertFalse(inst_rec["is_left_handed"])

    def test_rotated_instance_records_the_rotation(self):
        angle = math.pi / 4.0
        cap = self._run_with_instance(rotation=(0.0, 0.0, angle))
        inst_rec = cap._host_object_transform["instances"][0]
        self.assertAlmostEqual(inst_rec["rotation"][2], angle, places=9)
        self.assertEqual(inst_rec["determinant_sign"], 1)

    def test_mirrored_instance_records_negative_determinant(self):
        cap = self._run_with_instance(mirror=(1.0, -1.0, 1.0), is_left_handed=True)
        inst_rec = cap._host_object_transform["instances"][0]
        self.assertEqual(inst_rec["mirror"], (1.0, -1.0, 1.0))
        self.assertEqual(
            inst_rec["determinant_sign"], -1,
            "a mirrored instance must record a negative determinant sign",
        )
        # The host's OWN mirror flag, recorded alongside the plugin's
        # computed determinant sign -- both, not one standing in for the
        # other.
        self.assertTrue(inst_rec["is_left_handed"])

    def test_scaled_instance_records_the_scale(self):
        cap = self._run_with_instance(scaling_factor=(2.0, 1.5, 1.0))
        inst_rec = cap._host_object_transform["instances"][0]
        self.assertEqual(inst_rec["scaling_factor"], (2.0, 1.5, 1.0))
        self.assertEqual(inst_rec["determinant_sign"], 1)

    def test_object_determinant_sign_identity(self):
        """Exactly one volume, one instance, both determinant +1 ->
        object_determinant_sign is the product, +1."""
        cap = self._run_with_instance()
        self.assertEqual(cap._host_object_transform["object_determinant_sign"], 1)

    def test_object_determinant_sign_combines_volume_and_instance(self):
        """A mirrored instance (determinant -1) over an unmirrored volume
        (determinant +1) combines to -1 -- proving this is a PRODUCT of the
        two, not just a copy of the instance's own sign."""
        cap = self._run_with_instance(mirror=(1.0, -1.0, 1.0))
        inst_rec = cap._host_object_transform["instances"][0]
        vol_rec = cap._host_object_transform["volumes"][0]
        self.assertEqual(vol_rec["determinant_sign"], 1)
        self.assertEqual(inst_rec["determinant_sign"], -1)
        self.assertEqual(cap._host_object_transform["object_determinant_sign"], -1)

    def test_plugin_applied_present_alongside_raw_host_values(self):
        cap = self._run_with_instance()
        applied = cap._host_object_transform["plugin_applied"]
        self.assertIsNotNone(applied)
        self.assertIn("theta_deg", applied)
        self.assertIn("rotation_applied", applied)
        self.assertIn("rotation_matrix_2x2", applied)
        self.assertIn("z_shift_mm", applied)
        self.assertIn("xy_center_mm", applied)

    def test_report_on_disk_carries_the_same_block(self):
        """Not just an in-memory attribute -- what actually gets written for the
        user to compare against the GUI's own placement."""
        self._run_with_instance(rotation=(0.0, 0.0, math.pi / 6.0))
        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        block = report["host_object_transform"]
        self.assertTrue(block["read"])
        self.assertAlmostEqual(
            block["instances"][0]["rotation"][2], math.pi / 6.0, places=9,
        )


class HostObjectTransformPresentOnFailingRunTests(M18MeshTransformTestCase):
    """The block must be present on a run that fails, not only on success --
    a refusal is exactly the case where the recorded placement is most
    needed. Uses the volume-mirror ``TopologyError``
    (``_build_mesh_sectioner``'s "this plugin does not apply" refusal),
    which fires INSIDE ``_build_mesh_sectioner`` itself, so this specifically
    proves the record happens before that raise, not after it."""

    def test_present_and_correct_when_volume_mirror_refuses_the_run(self):
        width, depth = 20.0, 20.0
        n_layers = 3
        top_z = LAYER_HEIGHT_MM * n_layers + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(
            -width / 2.0, -depth / 2.0, 0.0, width / 2.0, depth / 2.0, top_z,
        )
        volume = fake_orca.ModelVolume(mesh, mirror=(1.0, -1.0, 1.0))
        instance = fake_orca.ModelInstance()
        model_object = fake_orca.ModelObject([volume], [instance])
        obj = fake_orca.make_rect_object(
            "wing", n_layers, width, depth, LAYER_HEIGHT_MM,
            min_x_mm=-width / 2.0, min_y_mm=-depth / 2.0,
            model_object=model_object,
        )
        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)

        block = cap._host_object_transform
        self.assertIsNotNone(block)
        self.assertTrue(block["read"])
        self.assertEqual(block["volumes"][0]["mirror"], (1.0, -1.0, 1.0))
        self.assertIsNone(
            block["plugin_applied"],
            "the run never got past the composition check, so no "
            "plugin-applied transform should be recorded",
        )

        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        self.assertIsNotNone(report["host_object_transform"])
        self.assertTrue(report["host_object_transform"]["read"])
        self.assertEqual(
            report["host_object_transform"]["volumes"][0]["mirror"],
            [1.0, -1.0, 1.0],
        )


class HostObjectTransformResetBetweenRunsTests(M18MeshTransformTestCase):
    """Staleness discipline, same as ``_mesh_alignment``/ ``_last_geometry``: a
    run that raises before ``_transform`` (and therefore before
    ``_build_mesh_sectioner``) is ever entered must not report a PRIOR run's
    ``host_object_transform`` on the same host-reused capability instance.
    The reset of ``self._host_object_transform`` in ``execute()`` is the line
    under test: without it, the SECOND run's ``cap._host_object_transform``
    would still hold the first run's rotated-instance data instead of
    ``None``.
    """

    def _make_obj(self, **inst_kwargs):
        width, depth = 20.0, 20.0
        n_layers = 3
        top_z = LAYER_HEIGHT_MM * n_layers + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(
            -width / 2.0, -depth / 2.0, 0.0, width / 2.0, depth / 2.0, top_z,
        )
        volume = fake_orca.ModelVolume(mesh)
        instance = fake_orca.ModelInstance(**inst_kwargs)
        model_object = fake_orca.ModelObject([volume], [instance])
        return fake_orca.make_rect_object(
            "wing", n_layers, width, depth, LAYER_HEIGHT_MM,
            min_x_mm=-width / 2.0, min_y_mm=-depth / 2.0,
            model_object=model_object,
        )

    def test_reset_between_runs_on_reused_capability_instance(self):
        self.arm()
        cap = self.new_capability()

        # Run 1: succeeds, with a rotated instance -- 90 degrees, so the
        # square box's bbox is unchanged and the host's own (unrotated)
        # slice still aligns, while ``rotation()`` still records a real,
        # non-default, non-zero value.
        obj1 = self._make_obj(rotation=(0.0, 0.0, math.pi / 2.0))
        cap.get_config = lambda: json.dumps(
            {"root_solid_layers": 0, "skin_margin_mm": 1.0}
        )
        result1 = cap.execute(self.make_ctx(obj1))
        self.assertEqual(
            result1.status, self._fake_orca.PluginResult.Success, result1.message,
        )
        self.assertIsNotNone(cap._host_object_transform)
        self.assertTrue(cap._host_object_transform["read"])

        # Run 2, SAME capability instance: a malformed stored config makes
        # ``_build_spec`` raise ``SpecError`` before ``_transform`` --
        # and therefore before ``_build_mesh_sectioner`` -- is ever
        # reached.
        obj2 = self._make_obj()
        cap.get_config = lambda: "not valid json{"
        result2 = cap.execute(self.make_ctx(obj2))
        self.assertEqual(result2.status, self._fake_orca.PluginResult.FatalError)

        self.assertIsNone(
            cap._host_object_transform,
            "a run that raised before _transform reported a PRIOR run's "
            "host_object_transform -- the reset is missing or was "
            "placed too late",
        )


# ---------------------------------------------------------------------------
# ``mo.volumes()`` itself raising must surface the ORIGINAL exception, not a
# misleading "model has 0 part volume(s)" message that makes a broken host call
# look like a normal, well-formed empty model.
# ---------------------------------------------------------------------------


class VolumesRaisingSurfacesTheOriginalExceptionTests(M18MeshTransformTestCase):
    def test_volumes_raising_is_not_reported_as_zero_part_volumes(self):
        def boom():
            raise RuntimeError("host volumes() exploded")

        model_object = fake_orca.ModelObject(volumes=boom, instances=[fake_orca.ModelInstance()])
        obj = fake_orca.make_rect_object(
            "wing", 3, 20.0, 20.0, LAYER_HEIGHT_MM, model_object=model_object,
        )
        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn(
            "host volumes() exploded", result.message,
            "the ORIGINAL exception must reach the failure report",
        )
        self.assertNotIn(
            "0 part volume(s)", result.message,
            "volumes() raising must not be silently reinterpreted as a "
            "normal, empty-of-parts model",
        )

        block = cap._host_object_transform
        self.assertIsNotNone(block)
        self.assertIn("host volumes() exploded", block["volumes_error"])
        self.assertFalse(
            block["read"],
            "read must be False when volumes() itself failed",
        )

        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        self.assertIn("host volumes() exploded", report["error"])
        self.assertNotIn("0 part volume(s)", report["error"])


class InstancesRaisingSurfacesTheOriginalExceptionTests(M18MeshTransformTestCase):
    """Same fix as ``VolumesRaisingSurfacesTheOriginalExceptionTests``, applied to
    ``mo.instances()``: raising there must not be silently reinterpreted as
    "model object has 0 instances"."""

    def test_instances_raising_is_not_reported_as_zero_instances(self):
        def boom():
            raise RuntimeError("host instances() exploded")

        mesh = fake_orca.box_mesh_mm(-10.0, -10.0, 0.0, 10.0, 10.0, 1.0)
        model_object = fake_orca.ModelObject(
            volumes=[fake_orca.ModelVolume(mesh)], instances=boom,
        )
        obj = fake_orca.make_rect_object(
            "wing", 3, 20.0, 20.0, LAYER_HEIGHT_MM, model_object=model_object,
        )
        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError)
        self.assertIn(
            "host instances() exploded", result.message,
            "the ORIGINAL exception must reach the failure report",
        )
        self.assertNotIn(
            "0 instances", result.message,
            "instances() raising must not be silently reinterpreted as a "
            "normal, instance-free model",
        )

        block = cap._host_object_transform
        self.assertIsNotNone(block)
        self.assertIn("host instances() exploded", block["instances_error"])
        self.assertFalse(block["read"])

        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        self.assertIn("host instances() exploded", report["error"])
        self.assertNotIn("0 instances", report["error"])


# ---------------------------------------------------------------------------
# trafo()/matrix()/mirror()/is_left_handed() each independently raising must
# not break an otherwise-normal run -- the error is recorded, and the resulting
# mutated geometry is IDENTICAL to a control run where nothing raises.
# ---------------------------------------------------------------------------


class HostObjectTransformRaisingMembersDoNotBreakTheRunTests(M18MeshTransformTestCase):
    """Covers ONLY ``trafo()``/``matrix()``/``is_left_handed()`` -- members that
    are diagnostic-only and NEVER feed geometry (see ``capability.py``'s
    ``_record_placement``/``_require_geometry_member`` docstrings). A
    ``mirror()`` that raises is NOT tolerated here: silently falling back to
    an identity mirror would slice a mirrored object with a flipped section.
    Every geometry-feeding member (offset/rotation/scaling_factor/mirror) is
    covered by ``GeometryMemberFailuresRefuseTheSliceTests`` below, which
    pins fail-CLOSED behaviour."""

    WIDTH_MM = 20.0
    DEPTH_MM = 20.0
    N_LAYERS = 3

    def _build_obj(self, volume_kwargs=None, instance_kwargs=None, trafo=None):
        half_w, half_d = self.WIDTH_MM / 2.0, self.DEPTH_MM / 2.0
        top_z = LAYER_HEIGHT_MM * self.N_LAYERS + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(-half_w, -half_d, 0.0, half_w, half_d, top_z)
        volume = fake_orca.ModelVolume(mesh, **(volume_kwargs or {}))
        instance = fake_orca.ModelInstance(**(instance_kwargs or {}))
        model_object = fake_orca.ModelObject([volume], [instance])

        min_x, min_y = fake_orca.to_scaled(-half_w), fake_orca.to_scaled(-half_d)
        max_x, max_y = fake_orca.to_scaled(half_w), fake_orca.to_scaled(half_d)
        layers = []
        for i in range(self.N_LAYERS):
            print_z = round(LAYER_HEIGHT_MM * (i + 1), 6)
            slice_z = round(print_z - LAYER_HEIGHT_MM / 2.0, 6)
            expoly = fake_orca.ExPolygon._wrap(fake_orca._Rect(min_x, min_y, max_x, max_y))
            surface = fake_orca.Surface(fake_orca.SurfaceType.stInternal, expoly)
            region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([surface]))
            layers.append(fake_orca.Layer(print_z, [region], slice_z=slice_z))
        return fake_orca.PrintObject(
            "wing", layers, model_object=model_object, trafo=trafo,
        )

    def _snapshot(self, obj):
        """Exact, comparable per-layer mutated geometry -- via
        ``fake_orca.describe`` (test-only introspection, not part of any
        real ``orca.host`` API), never a second, potentially-inconsistent
        float formula of this test's own."""
        snapshot = []
        for layer in obj.layers():
            layer_snapshot = []
            for region in layer.regions():
                for surface in region.slices.surfaces:
                    layer_snapshot.append(
                        (surface.surface_type, fake_orca.describe(surface.expolygon))
                    )
            snapshot.append(layer_snapshot)
        return snapshot

    def _run(self, **kwargs):
        obj = self._build_obj(**kwargs)
        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )
        return result, cap, obj

    def _assert_raising_member_matches_control(self, **kwargs):
        control_result, _control_cap, control_obj = self._run()
        self.assertEqual(
            control_result.status, self._fake_orca.PluginResult.Success,
            control_result.message,
        )
        control_snapshot = self._snapshot(control_obj)

        result, cap, obj = self._run(**kwargs)
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message,
        )
        self.assertEqual(
            self._snapshot(obj), control_snapshot,
            "a raising host member changed the MUTATED geometry -- it must "
            "only ever be recorded as a diagnostic, never change what gets "
            "cut",
        )
        return cap

    def test_trafo_raising_does_not_break_the_run(self):
        def boom():
            raise RuntimeError("host trafo() exploded")

        cap = self._assert_raising_member_matches_control(trafo=boom)
        block = cap._host_object_transform
        self.assertIn("host trafo() exploded", block["print_object_trafo_error"])
        self.assertIsNone(block["print_object_trafo"])

    def test_volume_matrix_raising_does_not_break_the_run(self):
        def boom():
            raise RuntimeError("host volume matrix() exploded")

        cap = self._assert_raising_member_matches_control(
            volume_kwargs={"matrix": boom},
        )
        vol_rec = cap._host_object_transform["volumes"][0]
        self.assertIn("host volume matrix() exploded", vol_rec["matrix_error"])
        self.assertIsNone(vol_rec["matrix"])

    def test_instance_matrix_raising_does_not_break_the_run(self):
        def boom():
            raise RuntimeError("host instance matrix() exploded")

        cap = self._assert_raising_member_matches_control(
            instance_kwargs={"matrix": boom},
        )
        inst_rec = cap._host_object_transform["instances"][0]
        self.assertIn("host instance matrix() exploded", inst_rec["matrix_error"])
        self.assertIsNone(inst_rec["matrix"])

    def test_is_left_handed_raising_does_not_break_the_run(self):
        def boom():
            raise RuntimeError("host is_left_handed() exploded")

        cap = self._assert_raising_member_matches_control(
            instance_kwargs={"is_left_handed": boom},
        )
        inst_rec = cap._host_object_transform["instances"][0]
        self.assertIn("host is_left_handed() exploded", inst_rec["is_left_handed_error"])
        self.assertIsNone(inst_rec["is_left_handed"])


# ---------------------------------------------------------------------------
# offset/rotation/scaling_factor/mirror FEED GEOMETRY. A failed (or absent)
# read of any of them must refuse the slice, immediately, naming the member and
# the recorded error -- never silently substitute an identity default (a
# mirrored object whose mirror() raised would otherwise slice with a flipped
# section instead of refusing).
# ---------------------------------------------------------------------------


class GeometryMemberFailuresRefuseTheSliceTests(M18MeshTransformTestCase):
    WIDTH_MM = 20.0
    DEPTH_MM = 20.0
    N_LAYERS = 3

    def _build_obj(self, volume_kwargs=None, instance_kwargs=None):
        half_w, half_d = self.WIDTH_MM / 2.0, self.DEPTH_MM / 2.0
        top_z = LAYER_HEIGHT_MM * self.N_LAYERS + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(-half_w, -half_d, 0.0, half_w, half_d, top_z)
        volume = fake_orca.ModelVolume(mesh, **(volume_kwargs or {}))
        instance = fake_orca.ModelInstance(**(instance_kwargs or {}))
        model_object = fake_orca.ModelObject([volume], [instance])

        min_x, min_y = fake_orca.to_scaled(-half_w), fake_orca.to_scaled(-half_d)
        max_x, max_y = fake_orca.to_scaled(half_w), fake_orca.to_scaled(half_d)
        layers = []
        for i in range(self.N_LAYERS):
            print_z = round(LAYER_HEIGHT_MM * (i + 1), 6)
            slice_z = round(print_z - LAYER_HEIGHT_MM / 2.0, 6)
            expoly = fake_orca.ExPolygon._wrap(fake_orca._Rect(min_x, min_y, max_x, max_y))
            surface = fake_orca.Surface(fake_orca.SurfaceType.stInternal, expoly)
            region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([surface]))
            layers.append(fake_orca.Layer(print_z, [region], slice_z=slice_z))
        return fake_orca.PrintObject("wing", layers, model_object=model_object)

    def _run(self, volume_kwargs=None, instance_kwargs=None):
        obj = self._build_obj(volume_kwargs=volume_kwargs, instance_kwargs=instance_kwargs)
        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )
        return result, cap, obj

    def _assert_refuses(self, expected_member, **kwargs):
        result, cap, _obj = self._run(**kwargs)
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(
            result.status, PluginResult.FatalError,
            "expected a refusal naming {} but got: {}".format(
                expected_member, result.message
            ),
        )
        self.assertIn(expected_member, result.message)
        self.assertIn(
            "refusing rather than guessing", result.message,
        )
        # (f): the on-disk report must carry the same error string as the
        # message the caller actually sees, in EVERY refusal case -- not
        # only some of them.
        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        self.assertIn(expected_member, report["error"])
        block = report["host_object_transform"]
        self.assertIsNotNone(block)
        return result, cap

    # -- (a)/(b): mirror() raising refuses, both instance and volume -------

    def test_instance_mirror_raising_refuses(self):
        """(a) instance mirror() raising refuses. Falling back to an identity
        mirror would silently succeed (fail-open) and slice a mirrored object
        with a flipped section."""
        def boom():
            raise RuntimeError("host instance mirror() exploded")

        result, cap = self._assert_refuses(
            "instance mirror()", instance_kwargs={"mirror": boom},
        )
        self.assertIn("RuntimeError: host instance mirror() exploded", result.message)
        inst_rec = cap._host_object_transform["instances"][0]
        self.assertIsNone(inst_rec["mirror"])
        self.assertIsNone(
            inst_rec["determinant_sign"],
            "a failed mirror read must never yield a confident sign",
        )

    def test_volume_mirror_raising_refuses(self):
        """(b) same as above, for the volume's own mirror()."""
        def boom():
            raise RuntimeError("host volume mirror() exploded")

        result, cap = self._assert_refuses(
            "volume mirror()", volume_kwargs={"mirror": boom},
        )
        self.assertIn("RuntimeError: host volume mirror() exploded", result.message)
        vol_rec = cap._host_object_transform["volumes"][0]
        self.assertIsNone(vol_rec["mirror"])
        self.assertIsNone(vol_rec["determinant_sign"])

    # -- (c): offset/rotation/scaling_factor raising refuses, naming the ---
    # -- member --------------------------------------------------------

    def test_volume_offset_raising_refuses(self):
        def boom():
            raise RuntimeError("host volume offset() exploded")

        self._assert_refuses("volume offset()", volume_kwargs={"offset": boom})

    def test_volume_rotation_raising_refuses(self):
        def boom():
            raise RuntimeError("host volume rotation() exploded")

        self._assert_refuses("volume rotation()", volume_kwargs={"rotation": boom})

    def test_volume_scaling_factor_raising_refuses(self):
        def boom():
            raise RuntimeError("host volume scaling_factor() exploded")

        self._assert_refuses(
            "volume scaling_factor()", volume_kwargs={"scaling_factor": boom},
        )

    def test_instance_rotation_raising_refuses(self):
        def boom():
            raise RuntimeError("host instance rotation() exploded")

        self._assert_refuses("instance rotation()", instance_kwargs={"rotation": boom})

    def test_instance_scaling_factor_raising_refuses(self):
        def boom():
            raise RuntimeError("host instance scaling_factor() exploded")

        self._assert_refuses(
            "instance scaling_factor()", instance_kwargs={"scaling_factor": boom},
        )

    def test_instance_offset_raising_does_not_refuse(self):
        """The control for (c): instance OFFSET is deliberately never
        required (translation stays dropped -- see
        ``_build_mesh_sectioner``'s own docstring), so a raising instance
        offset must NOT refuse the slice -- unlike every other member
        tested above. Without this, ``_assert_refuses`` alone could not
        tell a real "every member is required" bug from "the four members
        this suite happens to test are required"."""
        def boom():
            raise RuntimeError("host instance offset() exploded")

        result, _cap, _obj = self._run(instance_kwargs={"offset": boom})
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message,
        )

    # -- (d): an ABSENT member (not raising -- not present at all) --------

    def test_absent_instance_mirror_refuses(self):
        """(d) A host instance that does not expose ``mirror()`` at all
        (simulating an older/partial binding) must refuse exactly like a
        raising one -- recorded as ``"mirror_error": "absent"``, never
        silently treated as an identity mirror."""

        class _InstanceWithoutMirror:
            def offset(self):
                return (0.0, 0.0, 0.0)

            def rotation(self):
                return (0.0, 0.0, 0.0)

            def scaling_factor(self):
                return (1.0, 1.0, 1.0)

            # Deliberately no mirror(), no matrix(), no is_left_handed().

        half_w, half_d = self.WIDTH_MM / 2.0, self.DEPTH_MM / 2.0
        top_z = LAYER_HEIGHT_MM * self.N_LAYERS + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(-half_w, -half_d, 0.0, half_w, half_d, top_z)
        volume = fake_orca.ModelVolume(mesh)
        instance = _InstanceWithoutMirror()
        model_object = fake_orca.ModelObject([volume], [instance])
        obj = fake_orca.make_rect_object(
            "wing", self.N_LAYERS, self.WIDTH_MM, self.DEPTH_MM, LAYER_HEIGHT_MM,
            min_x_mm=-half_w, min_y_mm=-half_d, model_object=model_object,
        )

        result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )
        PluginResult = self._fake_orca.PluginResult
        self.assertEqual(result.status, PluginResult.FatalError, result.message)
        self.assertIn("instance mirror()", result.message)
        self.assertIn("absent", result.message)

        inst_rec = cap._host_object_transform["instances"][0]
        self.assertEqual(inst_rec["mirror_error"], "absent")
        self.assertIsNone(inst_rec["mirror"])

        with open(self.report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        self.assertIn("absent", report["error"])


# ---------------------------------------------------------------------------
# _sign_product must never return a confident sign from missing data -- a
# failed scaling_factor read must yield None too, not just a failed mirror
# read.
# ---------------------------------------------------------------------------


class MissingScalingFactorNeverYieldsAConfidentSignTests(M18MeshTransformTestCase):
    def test_volume_scaling_factor_raising_yields_no_confident_determinant_sign(self):
        def boom():
            raise RuntimeError("host volume scaling_factor() exploded")

        width, depth = 20.0, 20.0
        n_layers = 3
        top_z = LAYER_HEIGHT_MM * n_layers + LAYER_HEIGHT_MM
        mesh = fake_orca.box_mesh_mm(
            -width / 2.0, -depth / 2.0, 0.0, width / 2.0, depth / 2.0, top_z,
        )
        # Unlike ``mirror()``, capability.py does NOT fall back to a
        # default when re-reading scale downstream of this record, so this
        # run is not expected to succeed -- only that the RECORD (built
        # before that later, unprotected re-read) never claims a confident
        # determinant sign from data it never actually had.
        volume = fake_orca.ModelVolume(mesh, scaling_factor=boom)
        instance = fake_orca.ModelInstance()
        model_object = fake_orca.ModelObject([volume], [instance])
        obj = fake_orca.make_rect_object(
            "wing", n_layers, width, depth, LAYER_HEIGHT_MM,
            min_x_mm=-width / 2.0, min_y_mm=-depth / 2.0,
            model_object=model_object,
        )

        _result, cap = self.run_capability(
            obj, {"root_solid_layers": 0, "skin_margin_mm": 1.0},
        )

        block = cap._host_object_transform
        self.assertIsNotNone(block)
        vol_rec = block["volumes"][0]
        self.assertIsNone(vol_rec["scaling_factor"])
        self.assertIn("host volume scaling_factor() exploded", vol_rec["scaling_factor_error"])
        self.assertIsNone(
            vol_rec["determinant_sign"],
            "a missing scaling_factor must never yield a confident "
            "determinant sign, even though scale never carries the sign "
            "itself (see _sign_product's own docstring)",
        )
        self.assertIsNone(block["object_determinant_sign"])


if __name__ == "__main__":
    unittest.main()
