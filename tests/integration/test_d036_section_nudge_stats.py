"""Section-nudge bookkeeping in ``capability.py``'s ``_channels_for_layer``.

WHAT THIS PINS. ``sectioner.last_nudge_mm`` (the offset the mesh sectioner's
retry ladder actually applied to recover a grazed section) is recorded in the
per-run ``self._mesh_alignment`` diagnostics:

- ``applied_z = z + sectioner.last_nudge_mm`` -- the plane the section was
  ACTUALLY cut at, not the plane requested.
- ``stats["section_nudge_count"]`` -- incremented whenever a layer nudged.
- ``stats["section_nudge_first"]`` -- ``{"layer", "requested_z", "offset_mm"}``
  recorded on the FIRST nudged layer only.
- ``stats["anomalies"][...]["section_z"]`` and
  ``stats["first_channel"]["section_z"]`` -- both use ``applied_z``, not the
  requested ``z``.

APPROACH: STUB SECTIONER, NOT THE REAL FIXTURE. ``capability.py``'s
``_build_mesh_sectioner`` is monkeypatched to return a small fake sectioner
object exposing ``section_at()``/``last_nudge_mm``/``triangle_count``, driven
through the REAL host path (the checked-in bundle, via ``fake_orca``, exactly
like ``test_m1_capability.py`` and ``test_m18_mesh_transform.py``) so
``_channels_for_layer`` itself -- the method under test -- runs unmodified,
with its real caller (``_transform``'s per-layer loop) and real
``self._mesh_alignment`` dict. This tests the BOOKKEEPING only, not mesh
sectioning/retry itself -- that is ``tests/unit/test_mesh_section_retry.py``'s
job (real geometry, ``tests/fixtures/m36_grazed_vertex.stl``). Driving
that whole fixture through the host path is impractical for this check:
it would require placing a real model instance so that some layer's own
``slice_z`` lands exactly on the fixture's one known grazed plane
(168.850 mm), on top of everything ``test_m18_mesh_transform.py`` already
needs (instance transform, XY-centring, alignment-tolerance matching) --
none of which the nudge bookkeeping depends on.

The fake sectioner never returns an "outer" (non-interior) loop, only interior
ones (or none). ``_check_mesh_alignment`` (a SEPARATE, unrelated guard)
short-circuits to "nothing to check" whenever the outer-loop list it inspects
is empty (``if not outer: return None``, capability.py's
``_check_mesh_alignment``), so this stub cannot accidentally trip that guard's
own 0.02 mm tolerance -- it is simply never reached.
"""

import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import fake_orca  # noqa: E402


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
WIDTH_MM = 60.0
DEPTH_MM = 10.0

CAP_CONFIG = {"root_solid_layers": 0, "skin_margin_mm": 1.0}


class _FakeLoop:
    """Duck-typed stand-in for ``aircraft_core.mesh_section.Loop``.

    Only the attributes ``_channels_for_layer`` (and, for interior loops,
    ``DetectedChannel`` construction) actually touch: ``is_interior``,
    ``centroid``, ``equivalent_diameter_mm``, ``signed_area``. Never
    ``is_interior=False`` in this module -- see module docstring on why an
    "outer" loop is deliberately never produced.
    """

    def __init__(self, centroid, diameter_mm, area):
        self.is_interior = True
        self.centroid = centroid
        self.equivalent_diameter_mm = diameter_mm
        self.signed_area = area
        self.bbox = (
            centroid[0] - diameter_mm / 2.0, centroid[1] - diameter_mm / 2.0,
            centroid[0] + diameter_mm / 2.0, centroid[1] + diameter_mm / 2.0,
        )
        self.points = ()


class StubSectioner:
    """A ``MeshSectioner`` stand-in: ``plan`` maps a Z (rounded to 6 places,
    matching every Z this test's layers can produce) to
    ``(nudge_mm, loops)``. Unlisted Z values default to ``(0.0, [])`` -- a
    clean, channel-free section, matching a plain-skin layer.
    """

    def __init__(self, plan, triangle_count=42):
        self.triangle_count = triangle_count
        self.last_nudge_mm = 0.0
        self._plan = dict(plan)
        self.calls = []

    def section_at(self, z):
        key = round(z, 6)
        self.calls.append(key)
        nudge, loops = self._plan.get(key, (0.0, []))
        self.last_nudge_mm = nudge
        return list(loops)


class D036NudgeStatsTestCase(unittest.TestCase):
    """Common plumbing: fresh fake orca + tmp plugin dir, real bundle."""

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

    def make_ctx(self, obj, config=None):
        Step = self._fake_orca.slicing.Step
        return self._fake_orca.slicing.SlicingPipelineContext(
            step=Step.posSlice, obj=obj,
            config=dict(FULL_CONFIG if config is None else config),
        )

    def make_object(self):
        return fake_orca.make_rect_object(
            "wing", N_LAYERS, WIDTH_MM, DEPTH_MM, LAYER_HEIGHT_MM)

    def run_with_plan(self, plan, cap_config=None):
        """Arms, installs a ``StubSectioner`` built from ``plan``, and runs
        the real capability's ``execute`` once. Returns ``(result, cap)``.
        """
        self.arm()
        obj = self.make_object()
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps(
            CAP_CONFIG if cap_config is None else cap_config)

        stub = StubSectioner(plan)
        cap._build_mesh_sectioner = lambda _ctx: (
            stub, 0.0, {"x": [-WIDTH_MM / 2.0, WIDTH_MM / 2.0],
                        "y": [-DEPTH_MM / 2.0, DEPTH_MM / 2.0],
                        "z": [0.0, LAYER_HEIGHT_MM * N_LAYERS]})

        result = cap.execute(ctx)
        return result, cap

    # Every layer's own ``slice_z`` (``print_z - layer_height/2``), for
    # picking the plan's keys and the expected requested Z of a given
    # (0-based) layer index -- mirrors ``fake_orca.make_rect_object``'s own
    # formula exactly.
    @staticmethod
    def slice_z_of(index):
        print_z = round(LAYER_HEIGHT_MM * (index + 1), 6)
        return round(print_z - LAYER_HEIGHT_MM / 2.0, 6)


# ---------------------------------------------------------------------------
# 1 + 2 + 4: a nudge with a channel (first_channel), a second nudge with an
# anomaly (no channel), both on ``applied_z`` -- and only the FIRST nudge is
# latched into ``section_nudge_first``.
# ---------------------------------------------------------------------------


class TwoNudgedLayersTests(D036NudgeStatsTestCase):
    NUDGE_A_MM = 0.0015
    NUDGE_B_MM = -0.0007
    LAYER_A = 2  # gets a channel: exercises first_channel + section_nudge_first
    LAYER_B = 5  # no channel (0 interior loops): exercises an anomaly entry

    def setUp(self):
        super().setUp()
        self.z_a = self.slice_z_of(self.LAYER_A)
        self.z_b = self.slice_z_of(self.LAYER_B)
        plan = {
            # 15 mm diameter on a 10 mm-deep coupon needs more depth than the
            # section has (see ``_generate_spar_socket_from_channel``'s fit
            # check): the socket is DEGRADED (dropped, no geometry cut) rather
            # than built. That is deliberate -- it keeps this test about
            # ``_channels_for_layer``'s bookkeeping (which runs and records
            # ``first_channel`` regardless of what the generator does with the
            # channel afterwards) without needing fake_orca's rectangle-only
            # kernel to represent an actual circular spar-socket cut, which it
            # cannot (see this module's docstring).
            self.z_a: (
                self.NUDGE_A_MM,
                [_FakeLoop(centroid=(0.0, 0.0), diameter_mm=15.0, area=176.7)],
            ),
            self.z_b: (self.NUDGE_B_MM, []),
        }
        self.result, self.cap = self.run_with_plan(plan)
        self.stats = self.cap._mesh_alignment

    def test_run_succeeds(self):
        # Not the point of this module, but a Failure here would mean the
        # stats below were never reliably reached for every layer.
        self.assertEqual(
            self.result.status, self._fake_orca.PluginResult.Success,
            self.result.message,
        )

    def test_section_nudge_count_is_two(self):
        self.assertEqual(self.stats["section_nudge_count"], 2)

    def test_section_nudge_first_names_the_first_nudged_layer_only(self):
        """Assertion 1 + 4: layer/requested_z/offset_mm for LAYER_A, and
        LAYER_B (nudged too, and processed later) does not overwrite it."""
        self.assertEqual(
            self.stats["section_nudge_first"],
            {
                "layer": self.LAYER_A,
                "requested_z": round(self.z_a, 4),
                "offset_mm": self.NUDGE_A_MM,
            },
        )

    def test_first_channel_section_z_is_the_applied_plane_not_the_requested_one(self):
        """Assertion 2, the one that matters most: ``first_channel``'s
        ``section_z`` is the plane the section actually cut (requested Z
        PLUS the applied nudge), not the plane requested."""
        requested = round(self.z_a, 4)
        applied = round(self.z_a + self.NUDGE_A_MM, 4)
        self.assertNotEqual(applied, requested)
        self.assertEqual(self.stats["first_channel"]["section_z"], applied)
        self.assertNotEqual(self.stats["first_channel"]["section_z"], requested)

    def test_anomaly_entry_section_z_is_the_applied_plane_not_the_requested_one(self):
        """Assertion 2's other half: an ``anomalies`` entry (LAYER_B has
        zero interior loops, so ``len(inner) != 1`` triggers one) also uses
        ``applied_z``."""
        requested = round(self.z_b, 4)
        applied = round(self.z_b + self.NUDGE_B_MM, 4)
        self.assertNotEqual(applied, requested)
        entries = [a for a in self.stats["anomalies"] if a["layer"] == self.LAYER_B]
        self.assertEqual(len(entries), 1, self.stats["anomalies"])
        self.assertEqual(entries[0]["section_z"], applied)
        self.assertNotEqual(entries[0]["section_z"], requested)


# ---------------------------------------------------------------------------
# 3: a clean run needs no nudge at all.
# ---------------------------------------------------------------------------


class NoNudgeNeededTests(D036NudgeStatsTestCase):
    def test_clean_run_records_zero_nudges_and_no_first(self):
        result, cap = self.run_with_plan(plan={})
        self.assertEqual(
            result.status, self._fake_orca.PluginResult.Success, result.message)
        stats = cap._mesh_alignment
        self.assertEqual(stats["section_nudge_count"], 0)
        self.assertIsNone(stats["section_nudge_first"])


if __name__ == "__main__":
    unittest.main()
