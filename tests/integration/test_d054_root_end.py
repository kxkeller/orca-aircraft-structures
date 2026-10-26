"""Capability seam: root-end resolution, the root-end-aware spar-socket latch,
and the tip-end watches, for a part that prints TIP FACE DOWN.

Requirement: the outside (tip) half of the elevon wing prints tip face down
(root/joint face at the top). Everything below runs the real generated bundle
under the fake host.

What each class pins:

* ``ConfigAndReportTests``   root_end config -> report fields (explicit bed,
  explicit top, auto with no signal defaults to bed).
* ``DisagreementFailsClosedTests``   area says one end, the spar-channel span
  says the other, in auto mode: the slice fails naming both signals.
* ``PrecomputedLatchTests``   the precompute walked from the root end; the
  online per-layer latch consulting it (root at top) and, with the root at the
  bed, agreeing with ``walk_latch(root_at_bed=True)`` on random status
  sequences.
* ``TipDownPrintTopWatchTests``   watch the PRINT TOP whichever
  anatomical end is there (in-host counterexample, layer 1250).
"""

import json
import os
import random
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import fake_orca  # noqa: E402
from aircraft_core.mesh_section import Loop  # noqa: E402
from aircraft_core.spar_latch import walk_latch  # noqa: E402

LH = 1.0
WALL = 0.42
FULL_CONFIG = {
    "layer_height": LH,
    "initial_layer_print_height": LH,
    "outer_wall_line_width": WALL,
    "inner_wall_line_width": WALL,
    "line_width": WALL,
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
A, F, N = "absent", "fit", "not_fit"


class _Base(unittest.TestCase):
    def setUp(self):
        self._fake_orca, restore = fake_orca.install()
        self.addCleanup(restore)
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.entry_file = os.path.join(self._tmp.name, "aircraft_structures.py")
        self.namespace = fake_orca.exec_bundle(self.entry_file)
        self.report_path = os.path.join(self._tmp.name, "m1", "m1_report.json")

    def arm(self):
        with open(os.path.join(self._tmp.name, "m1_enabled"), "w", encoding="utf-8"):
            pass

    def ctx(self, obj):
        Step = self._fake_orca.slicing.Step
        return self._fake_orca.slicing.SlicingPipelineContext(
            step=Step.posSlice, obj=obj, config=dict(FULL_CONFIG),
            cancelled_fn=None,
        )

    def cap(self, cap_config):
        self.arm()
        cap = self.namespace["AircraftStructuresM1"]()
        cap.get_config = lambda: json.dumps(cap_config)
        return cap

    def read_report(self):
        with open(self.report_path, "r", encoding="utf-8") as fh:
            return json.load(fh)


# ---------------------------------------------------------------------------
# A scripted sectioner: an outer rectangle whose width follows ``width_at``,
# plus one interior loop (a 10.5 mm "spar channel") on layers whose status is
# not ``absent``. Duck-types MeshSectioner's read surface, incl. ``vertices``.
# ---------------------------------------------------------------------------


class ScriptedSectioner:
    def __init__(self, z_hi, width_lo, width_hi, depth, channel_layers,
                 n_layers, channel_dia=10.5):
        self.triangle_count = 12
        self.last_nudge_mm = 0.0
        self.z_min, self.z_max = 0.0, z_hi
        self._w = (width_lo, width_hi)
        self._depth = depth
        self._n = n_layers
        self._present = set(channel_layers)
        self._dia = channel_dia
        self.vertices = [
            (sx * w / 2.0, sy * depth / 2.0, z)
            for z, w in ((0.0, width_lo), (z_hi, width_hi))
            for sx in (-1, 1) for sy in (-1, 1)
        ]

    def _layer_of(self, z):
        return int(z / LH)

    def section_at(self, z):
        frac = z / self.z_max
        w = self._w[0] + (self._w[1] - self._w[0]) * frac
        hw, hd = w / 2.0, self._depth / 2.0
        pts = ((-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd))
        loops = [Loop(points=pts, signed_area=w * self._depth, is_interior=False,
                      centroid=(0.0, 0.0), bbox=(-hw, -hd, hw, hd),
                      equivalent_diameter_mm=0.0)]
        if self._layer_of(z) in self._present:
            r = self._dia / 2.0
            loops.append(Loop(points=(), signed_area=-3.14159 * r * r,
                              is_interior=True, centroid=(0.0, 0.0),
                              bbox=(-r, -r, r, r), equivalent_diameter_mm=self._dia))
        return loops


class _StubGenerator:
    """Returns scripted statuses by layer index."""

    def __init__(self, statuses):
        self._st = statuses

    def spar_channel_status(self, frame, spec, layer_ctx, wall):
        return self._st[layer_ctx.layer_index]


def _host_object(n_layers, width=20.0, depth=6.0):
    return fake_orca.make_rect_object("wing", n_layers, width, depth, LH)


class ConfigAndReportTests(_Base):
    """A real tube-through-box mesh (constant section): no area signal, no
    channel-span signal (the tube spans everything)."""

    def _tube_object(self, n_layers=6):
        # Mesh exactly as tall as the layer stack, so the channel abuts BOTH
        # ends equally (a taller mesh would make it look root-ward-low).
        top_z = LH * n_layers
        mesh = fake_orca.box_with_rect_channel_mesh_mm(
            -20.0, -5.0, 20.0, 5.0, 0.0, top_z, -2.0, -1.5, 2.0, 1.5)
        return fake_orca.make_rect_object(
            "wing", n_layers, 40.0, 10.0, LH, min_x_mm=-20.0, min_y_mm=-5.0,
            model_object=fake_orca.identity_model_object(mesh))

    def _run(self, root_end):
        cfg = {"root_solid_layers": 0, "skin_margin_mm": 1.0}
        if root_end is not None:
            cfg["root_end"] = root_end
        cap = self.cap(cfg)
        cap.execute(self.ctx(self._tube_object()))
        return cap

    def test_default_is_auto_and_a_signal_free_mesh_keeps_the_bed_default(self):
        cap = self._run(None)
        r = cap._root_end_report
        self.assertEqual(r["config"], "auto")
        self.assertEqual((r["root_end"], r["root_at"]), ("low", "bed"))
        self.assertEqual(r["decision_source"], "default_bed_no_signal")
        self.assertIsNone(cap._latch_precomputed)
        self.assertEqual(cap._spar_latch_report["mode"],
                         "online_root_at_bed_walk_is_crosscheck")
        self.assertIsNotNone(cap._spar_latch_report["precompute_ms"])

    def test_explicit_bed_skips_the_precompute_entirely(self):
        cap = self._run("bed")
        self.assertEqual(cap._root_end_report["decision_source"], "config_override")
        self.assertEqual(cap._spar_latch_report["mode"],
                         "online_root_at_bed_precompute_skipped")
        self.assertEqual(cap._spar_latch_report["layers_total"], 0)

    def test_explicit_top_selects_the_top_walk(self):
        cap = self._run("top")
        r = cap._root_end_report
        self.assertEqual((r["root_end"], r["root_at"]), ("high", "top"))
        self.assertEqual(r["decision_source"], "config_override")
        self.assertEqual(cap._spar_latch_report["mode"], "precomputed_root_at_top")
        self.assertIsNotNone(cap._latch_precomputed)

    def test_report_json_carries_both_blocks(self):
        self._run("top")
        report = self.read_report()
        self.assertEqual(report["root_end"]["root_at"], "top")
        self.assertIn("area_signal", report["root_end"])
        self.assertIn("channel_signal", report["root_end"])
        latch = report["spar_latch"]
        self.assertEqual(latch["mode"], "precomputed_root_at_top")
        self.assertIn("predicted", latch)
        self.assertIn("online", latch)
        self.assertGreater(latch["layers_total"], 0)

    def test_an_invalid_root_end_value_is_a_config_error(self):
        cap = self.cap({"root_solid_layers": 0, "root_end": "tip"})
        result = cap.execute(self.ctx(self._tube_object()))
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        self.assertIn("root_end", result.message)


class DisagreementFailsClosedTests(_Base):
    N_LAYERS = 12

    def _run(self, root_end):
        # AREA: 20 mm wide at the TOP, 4 mm at the bed -> root is the high end.
        # CHANNEL: present only on layers 1..8 -> abuts the LOW end.
        sec = ScriptedSectioner(
            z_hi=self.N_LAYERS * LH, width_lo=4.0, width_hi=20.0, depth=6.0,
            channel_layers=range(1, 9), n_layers=self.N_LAYERS)
        cap = self.cap({"root_solid_layers": 0, "root_end": root_end})
        cap._build_mesh_sectioner = lambda _ctx: (
            sec, 0.0, {"x": [-10.0, 10.0], "y": [-3.0, 3.0],
                       "z": [0.0, self.N_LAYERS * LH]})
        result = cap.execute(self.ctx(_host_object(self.N_LAYERS)))
        return result, cap

    def test_auto_refuses_naming_both_signals_and_the_remedy(self):
        result, cap = self._run("auto")
        self.assertEqual(result.status, self._fake_orca.PluginResult.FatalError)
        msg = result.message
        self.assertIn("AREA says the root is the high Z end", msg)
        self.assertIn("CHANNEL says the root is the low Z end", msg)
        self.assertIn("root_end", msg)
        self.assertIn('"bed"', msg)
        self.assertIn('"top"', msg)
        report = self.read_report()
        self.assertEqual(report["root_end"]["decision_source"],
                         "REFUSED_signals_disagree")
        self.assertEqual(report["root_end"]["area_signal"]["end"], "high")
        self.assertEqual(report["root_end"]["channel_signal"]["end"], "low")

    def test_the_override_resolves_it_and_is_recorded_as_contradicting(self):
        for value, at, contradicts in (("top", "top", ["channel"]),
                                       ("bed", "bed", ["area"])):
            _result, cap = self._run(value)
            r = cap._root_end_report
            self.assertEqual(r["root_at"], at)
            self.assertEqual(r["decision_source"], "config_override")
            self.assertEqual(r["override_contradicts_signal"], contradicts)


class PrecomputedLatchTests(_Base):
    N_LAYERS = 12

    class _Intent(object):
        def __init__(self, family_name, x=1.0, spar_confirmed=True):
            self.family_name = family_name
            self.start = (x, 0.0)
            self.spar_confirmed = spar_confirmed

    def _precompute(self, statuses, root_end="auto"):
        n = len(statuses)
        present = [i for i, s in enumerate(statuses) if s != A]
        sec = ScriptedSectioner(
            z_hi=n * LH, width_lo=4.0, width_hi=20.0, depth=6.0,
            channel_layers=present, n_layers=n)
        cap = self.cap({"root_solid_layers": 0, "root_end": root_end})
        obj = _host_object(n)
        ctx = self.ctx(obj)
        spec = cap._build_spec(ctx)[0]
        self._reset_run_state(cap)
        cap._resolve_root_end_and_latch(ctx, spec, sec, _StubGenerator(statuses), WALL)
        return cap

    @staticmethod
    def _reset_run_state(cap):
        # What ``_transform`` resets at the run boundary; needed here because
        # these tests call the precompute method directly.
        cap._root_end_report = None
        cap._spar_latch_report = None
        cap._latch_precomputed = None
        cap._latch_online_arm_index = None
        cap._latch_online_suppressed = 0
        cap._spar_socket_ended = False
        pass

    def test_tip_down_thin_tip_end_keeps_the_whole_root_ward_spar(self):
        # bed = tip: no channel at 0-1, too thin at 2-4, fits 5-11.
        st = [A, A, N, N, N, F, F, F, F, F, F, F]
        cap = self._precompute(st)
        self.assertEqual(cap._root_end_report["root_at"], "top")
        self.assertEqual(cap._root_end_report["decision_source"],
                         "auto_area_and_channel_agree")
        pred = cap._spar_latch_report["predicted"]
        self.assertEqual(pred["walk_direction"], "top_to_bed")
        self.assertEqual(pred["arm_layer"], 4)
        self.assertEqual(pred["suppressed_count"], 0)
        # ... and the online layer decision, root at top: every fitting layer
        # keeps its socket, the thin tip-side layers keep their degradation.
        for i in range(5, 12):
            kept, degs = cap._apply_spar_socket_latch(
                [self._Intent("spar_socket")], [], i)
            self.assertEqual([x.family_name for x in kept], ["spar_socket"])
            self.assertEqual(degs, [])
        for i in (2, 3, 4):
            kept, degs = cap._apply_spar_socket_latch(
                [], [{"kind": "spar_socket", "x_mm": 1.0, "reason": "."}], i)
            self.assertEqual(len(degs), 1)
        self.assertFalse(cap._spar_socket_ended,
                         "the online flag must never arm in root-at-top mode")

    def test_tip_down_resume_past_an_end_is_suppressed_tipward_only(self):
        st = [F, F, F, N, F, F, F, F, F, F, F, F]
        cap = self._precompute(st)
        self.assertEqual(cap._root_end_report["root_at"], "top")
        pred = cap._spar_latch_report["predicted"]
        self.assertEqual(pred["arm_layer"], 3)
        self.assertEqual((pred["suppressed_layer_first"],
                          pred["suppressed_layer_last"]), (0, 2))
        for i in (0, 1, 2):
            kept, degs = cap._apply_spar_socket_latch(
                [self._Intent("spar_socket"), self._Intent("lattice")], [], i)
            self.assertEqual([x.family_name for x in kept], ["lattice"])
            self.assertEqual([d["kind"] for d in degs], ["spar_socket_latched"])
            self.assertIn("ROOT end", degs[0]["reason"])
        kept, degs = cap._apply_spar_socket_latch(
            [self._Intent("spar_socket")], [], 8)
        self.assertEqual(len(kept), 1)
        self.assertEqual(degs, [])
        snap = cap._spar_latch_report_snapshot()
        self.assertEqual(snap["online"]["suppressed_layer_count"], 3)

    def test_unconfirmed_socket_is_never_suppressed_by_the_walk(self):
        cap = self._precompute([F, F, F, N, F, F, F, F, F, F, F, F])
        kept, degs = cap._apply_spar_socket_latch(
            [self._Intent("spar_socket", spar_confirmed=False)], [], 1)
        self.assertEqual(len(kept), 1)
        self.assertEqual(degs, [])

    def test_a_socket_drop_the_precompute_did_not_foresee_is_recorded_not_armed(self):
        cap = self._precompute([F] * 12, root_end="top")
        cap._apply_spar_socket_latch(
            [], [{"kind": "spar_socket", "x_mm": 1.0, "reason": "."}], 6)
        self.assertFalse(cap._spar_socket_ended)
        self.assertEqual(cap._spar_latch_report["unforeseen_socket_drops"], [6])

    def test_bed_root_precompute_is_a_crosscheck_and_leaves_the_online_latch_in_charge(self):
        st = [F, F, F, F, F, N, N, F, F, F, A, A]
        # AREA must say the root is at the LOW end for auto to pick the bed:
        n = len(st)
        sec = ScriptedSectioner(
            z_hi=n * LH, width_lo=20.0, width_hi=4.0, depth=6.0,
            channel_layers=[i for i, s in enumerate(st) if s != A], n_layers=n)
        cap = self.cap({"root_solid_layers": 0})
        ctx = self.ctx(_host_object(n))
        spec = cap._build_spec(ctx)[0]
        self._reset_run_state(cap)
        cap._resolve_root_end_and_latch(ctx, spec, sec, _StubGenerator(st), WALL)
        self.assertEqual(cap._root_end_report["root_at"], "bed")
        self.assertIsNone(cap._latch_precomputed)
        self.assertEqual(cap._spar_latch_report["predicted"]["arm_layer"], 5)
        self.assertEqual(cap._spar_latch_report["predicted"]["suppressed_layer_first"], 7)

    def test_bed_root_walk_reproduces_the_online_latch_exactly(self):
        """The REAL online ``_apply_spar_socket_latch`` (no precompute
        installed), driven in host order over random status sequences, must
        arm at, and suppress, exactly what ``walk_latch(root_at_bed=True)``
        says."""
        cap = self.cap({"root_solid_layers": 0})
        rng = random.Random(54)
        for trial in range(200):
            n = rng.randint(1, 40)
            st = [rng.choice([F, F, F, N, A]) for _ in range(n)]
            cap._spar_socket_ended = False
            cap._latch_precomputed = None
            cap._latch_online_arm_index = None
            cap._latch_online_suppressed = 0
            suppressed = []
            for i, s in enumerate(st):
                intents = [self._Intent("spar_socket")] if s == F else []
                degs = ([{"kind": "spar_socket", "x_mm": 1.0, "reason": "."}]
                        if s == N else [])
                _kept, out = cap._apply_spar_socket_latch(intents, degs, i)
                if any(d["kind"] == "spar_socket_latched" for d in out):
                    suppressed.append(i)
            walk = walk_latch(st, root_at_bed=True)
            self.assertEqual(suppressed, walk["suppressed"], (trial, st))
            self.assertEqual(cap._latch_online_arm_index, walk["arm_index"], (trial, st))
            self.assertEqual(cap._latch_online_suppressed, len(suppressed))

    def test_the_two_walk_directions_disagree_on_the_measured_defect_shape(self):
        # Shape of the in-host outside-half report, scaled down: tip-side thin
        # end first in host order.
        st = [A] * 3 + [N] * 4 + [F] * 20
        bed_first = walk_latch(st, True)["suppressed"]
        root_first = walk_latch(st, False)["suppressed"]
        self.assertEqual(len(bed_first), 20)   # the defect: rod-reachable spar lost
        self.assertEqual(root_first, [])       # root-first walk: none of it is lost


class TipDownPrintTopWatchTests(_Base):
    """The tip-end watches follow the PRINT TOP (mesh max Z), whichever
    anatomical end is there, rather than a fixed end. In-host counterexample:
    the tip-face-down outside half ended with a sliver on its last layer
    (Z 250.20) left by the 3.5 deg tilted ROOT face; that layer must shut down
    like a tip layer instead of failing the slice.

    Fixture: tip-down placement (root_end=top): a 20 mm wide body on the bed
    (the flat tip face) and a 0.3 mm sliver on the last layer (the tilted
    root face's leftover)."""

    N_LAYERS = 6
    WIDE, SLIVER, DEPTH = 20.0, 0.3, 20.0

    @staticmethod
    def _rect(w, d):
        return [(-w / 2.0, -d / 2.0), (w / 2.0, -d / 2.0),
                (w / 2.0, d / 2.0), (-w / 2.0, d / 2.0)]

    def _object(self, empty=()):
        n = self.N_LAYERS
        top_z = n * LH
        taper_z = (n - 1) * LH
        stations = [(0.0, self._rect(self.WIDE, self.DEPTH)),
                    (taper_z - 0.001, self._rect(self.WIDE, self.DEPTH)),
                    (taper_z, self._rect(self.SLIVER, self.DEPTH)),
                    (top_z, self._rect(self.SLIVER, self.DEPTH))]
        mesh = fake_orca.loft_mesh_mm(stations)
        layers = []
        for i in range(n):
            print_z = round(LH * (i + 1), 6)
            slice_z = round(print_z - LH / 2.0, 6)
            if i in empty:
                region = fake_orca.LayerRegion(fake_orca.SurfaceCollection([]))
            else:
                w = self.SLIVER if slice_z > taper_z else self.WIDE
                pts = tuple((fake_orca.to_scaled(x), fake_orca.to_scaled(y))
                            for x, y in self._rect(w, self.DEPTH))
                expoly = fake_orca.ExPolygon._wrap(fake_orca._Polygon(pts=pts))
                region = fake_orca.LayerRegion(fake_orca.SurfaceCollection(
                    [fake_orca.Surface(fake_orca.SurfaceType.stInternal, expoly)]))
            layers.append(fake_orca.Layer(print_z, [region], slice_z=slice_z))
        return fake_orca.PrintObject(
            "tip_down", layers, model_object=fake_orca.identity_model_object(mesh))

    def _run(self, empty=(), root_end="top"):
        cap = self.cap({"root_solid_layers": 0, "skin_margin_mm": 1.0,
                        "root_end": root_end})
        result = cap.execute(self.ctx(self._object(empty)))
        return result, cap

    def _top_record(self):
        return [lr for lr in self.read_report()["layers"]
                if lr["layer_index"] == self.N_LAYERS - 1][0]

    def test_tip_down_top_layer_sliver_shuts_down_as_d048_not_failure(self):
        result, cap = self._run()
        self.assertEqual(cap._root_end_report["root_at"], "top")
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success,
                         result.message)
        rec = self._top_record()
        self.assertFalse(rec["mutated"])
        self.assertIn("host_tip_below_printable_floor",
                      [d["kind"] for d in rec.get("degradations", [])])

    def test_tip_down_top_layer_empty_host_region_degrades_as_d046(self):
        result, cap = self._run(empty={self.N_LAYERS - 1})
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success,
                         result.message)
        self.assertIn("host_tip_region_empty",
                      [d["kind"] for d in self._top_record().get("degradations", [])])

    def test_root_end_does_not_move_the_print_top_watch(self):
        # Same mesh, root_end=bed: identical outcome at the print top.
        result, _cap = self._run(root_end="bed")
        self.assertEqual(result.status, self._fake_orca.PluginResult.Success,
                         result.message)
        self.assertIn("host_tip_below_printable_floor",
                      [d["kind"] for d in self._top_record().get("degradations", [])])

    def test_print_top_helper_is_the_mesh_max_z(self):
        cap = self.cap({})
        self.assertEqual(cap._print_top_z([0.0, 9.0]), 9.0)


if __name__ == "__main__":
    unittest.main()
