"""Pinched-section shutdown. Hermetic: tracked fixtures under
``tests/fixtures/island_disconnect`` only (never ``host/``).

Real fixture: layer 1098 of the root-down outside elevon wing (the host's own
contour, one outer contour, bare contour erodes at 0.42 mm into two pieces).
Negative controls: a layer whose bare contour does not split but whose slots do
(layer-786 wedge) still raises; a healthy layer never enters the pinch path.
"""
import json
import sys
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import aircraft_core.pinch as pinch  # noqa: E402
import aircraft_core.slot as slotmod  # noqa: E402
from aircraft_core.diagnostics import LayerRecord, RunDiagnostics  # noqa: E402
from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import LayerContext, StructureGenerator, WebIntent  # noqa: E402
from aircraft_core.spec import StructureSpec, WebFamily  # noqa: E402
from aircraft_core.validate import TopologyError, TopologyValidator  # noqa: E402

FIXTURES = REPO_ROOT / "tests" / "fixtures" / "island_disconnect"
SAFETY = 0.42

try:
    import structural_analysis as sa  # noqa: E402
    _HAVE_SA = True
except ImportError:  # pragma: no cover
    _HAVE_SA = False
try:
    from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402
except ImportError:  # pragma: no cover
    HAVE_SHAPELY = False

NEEDS = unittest.skipUnless(HAVE_SHAPELY and _HAVE_SA, "shapely + structural_analysis required")


def _load_1098():
    with open(FIXTURES / "layer1098_pinched_neck_minimal.json", encoding="utf-8") as fh:
        fx = json.load(fh)
    with open(FIXTURES / "elevon_cap_config.json", encoding="utf-8") as fh:
        spec = sa.build_spec_from_cap_config(json.load(fh))
    ops = MiterShapelyOps()
    original = ops.polygon_from_points([tuple(p) for p in fx["host_contour_pts_mm"]])
    return fx, spec, ops, original


def _plugin_like_run(fx, spec, ops, original):
    """Generate, cut and validate the layer the way the plugin does, through the
    wrapper."""
    frame = LayerFrame.from_polygon(ops, original)
    gen = StructureGenerator().generate(
        frame, spec, fx["wall_width_mm"],
        LayerContext(layer_index=fx["layer_index"], print_z=fx["print_z_mm"]),
        safety_offset_mm=fx["safety_offset_mm"],
    )
    builder = slotmod.SlotBuilder()
    intents = list(gen.intents)
    slots = [ops.polygon_from_points(builder.build(i, spec.overshoot_mm).points) for i in intents]
    labels = ["family {} ({!r})".format(i.family_index, i.family_name) for i in intents]
    fam = [i.family_membership() for i in intents]
    validator = TopologyValidator(ops, safety_offset_mm=SAFETY)
    replacement = list(ops.diff(original, slots))
    return pinch.validate_with_pinch_shutdown(
        ops, validator, [original], replacement, intents, slots, spec,
        fx["layer_index"], fx["print_z_mm"], labels, fam, SAFETY,
    )


@NEEDS
class Layer1098PinchTests(unittest.TestCase):
    def setUp(self):
        self.fx, self.spec, self.ops, self.original = _load_1098()

    def test_bare_contour_splits_into_wing_and_cap(self):
        p = pinch.check_pinched_section(self.ops, [self.original], SAFETY, 1098, 219.8)
        self.assertIsNotNone(p)
        self.assertEqual(p.piece_count, 2)
        self.assertAlmostEqual(p.piece_areas_mm2[0], 983.9, delta=1.0)
        self.assertAlmostEqual(p.piece_areas_mm2[1], 63.0, delta=1.0)
        # the neck is severed by any erosion of about 0.1 mm or more
        self.assertGreater(p.min_neck_erosion_radius_mm, 0.05)
        self.assertLess(p.min_neck_erosion_radius_mm, 0.15)

    def test_layer_ends_as_shutdown_with_recorded_reason(self):
        rescue, p = _plugin_like_run(self.fx, self.spec, self.ops, self.original)
        self.assertIsNone(rescue)
        self.assertIsNotNone(p)
        self.assertIn("section pinched below wall-safety erosion (0.42 mm)", p.reason)
        self.assertIn("splits into 2 pieces", p.reason)
        self.assertIn("layer 1098", p.reason)
        self.assertIn("printed as plain skin", p.reason)
        degradations, extra = pinch.layer_record_fields(p)
        diag = RunDiagnostics()
        diag.record(LayerRecord(
            layer_index=1098, print_z=self.fx["print_z_mm"], mutated=False,
            shutdown=True, degradations=degradations, extra=extra,
        ))
        report = diag.to_dict()
        self.assertEqual(report["shutdown_count"], 1)
        self.assertEqual(report["failed_count"], 0)
        self.assertEqual(report["mutated_count"], 0)
        self.assertEqual(report["pinched_section_count"], 1)
        entry = report["pinched_section_layers"][0]
        self.assertEqual(entry["layer_index"], 1098)
        self.assertEqual(entry["piece_count"], 2)
        self.assertEqual(report["degradation_counts_by_kind"], {"pinched_section": 1})

    def test_not_entered_when_host_has_more_than_one_outer_contour(self):
        self.assertIsNone(pinch.check_pinched_section(
            self.ops, [self.original, self.original], SAFETY, 1, 1.0))


@NEEDS
class NegativeControlTests(unittest.TestCase):
    def test_slot_caused_split_on_unpinched_contour_still_raises(self):
        # layer-786 wedge: the bare contour does NOT split; a slot does.
        with open(FIXTURES / "layer786_entry_wedge_minimal.json", encoding="utf-8") as fh:
            fx = json.load(fh)
        ops = MiterShapelyOps()
        original = ops.polygon_from_points([tuple(p) for p in fx["host_contour_pts_mm"]])
        self.assertIsNone(pinch.check_pinched_section(ops, [original], SAFETY, 786, fx["print_z_mm"]))
        frame = LayerFrame.from_polygon(ops, original)
        pair_family = WebFamily(name="tie_test", role="primary_structure", web_position=0.5, opposing_pair=True)
        plain_family = WebFamily(name="wedge", role="skin_lattice", web_position=0.1, web_depth_mm=1.5)
        spec = StructureSpec(skin_margin_mm=1.0, gap_ratio=0.05, families=(pair_family, plain_family))
        result = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=786, print_z=fx["print_z_mm"]),
            safety_offset_mm=SAFETY)
        builder = slotmod.SlotBuilder()
        intents = list(result.intents)
        slots = [ops.polygon_from_points(builder.build(i, spec.overshoot_mm).points) for i in intents]
        wedge = WebIntent(start=(-22.54, 7.1), end=(-22.54, 5.55), width_mm=0.021,
                          family_index=1, family_name="wedge", entry_lower=False)
        slots.append(ops.polygon_from_points([tuple(p) for p in fx["slot_points_mm"]]))
        intents.append(wedge)
        labels = ["s{}".format(k) for k in range(len(slots))]
        fam = [i.family_membership() for i in intents]
        validator = TopologyValidator(ops, safety_offset_mm=SAFETY)
        replacement = list(ops.diff(original, slots))
        with self.assertRaises(TopologyError) as ctx:
            pinch.validate_with_pinch_shutdown(
                ops, validator, [original], replacement, intents, slots, spec,
                786, fx["print_z_mm"], labels, fam, SAFETY)
        self.assertIn("slot disconnects material island", str(ctx.exception))
        self.assertIn("Pair ablation was tried", str(ctx.exception))

    def test_healthy_layer_never_enters_pinch_path(self):
        ops = MiterShapelyOps()
        original = ops.polygon_from_points([(-20.0, -10.0), (20.0, -10.0), (20.0, 10.0), (-20.0, 10.0)])
        frame = LayerFrame.from_polygon(ops, original)
        family = WebFamily(name="tie_test", role="primary_structure", web_position=0.5, opposing_pair=True)
        spec = StructureSpec(skin_margin_mm=1.0, gap_ratio=0.05, families=(family,))
        result = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=1.0), safety_offset_mm=SAFETY)
        intents = list(result.intents)
        builder = slotmod.SlotBuilder()
        slots = [ops.polygon_from_points(builder.build(i, spec.overshoot_mm).points) for i in intents]
        fam = [i.family_membership() for i in intents]
        validator = TopologyValidator(ops, safety_offset_mm=SAFETY)
        replacement = list(ops.diff(original, slots))
        with mock.patch.object(pinch, "pinched_section_after_failure") as spy_after, \
                mock.patch.object(pinch, "check_pinched_section") as spy_check:
            rescue, p = pinch.validate_with_pinch_shutdown(
                ops, validator, [original], replacement, intents, slots, spec,
                0, 1.0, ["a", "b"], fam, SAFETY)
        self.assertIsNone(rescue)
        self.assertIsNone(p)
        self.assertEqual(spy_after.call_count, 0)
        self.assertEqual(spy_check.call_count, 0)

    def test_non_island_failure_never_checks_the_bare_contour(self):
        ops = MiterShapelyOps()
        original = ops.polygon_from_points([(-20.0, -10.0), (20.0, -10.0), (20.0, 10.0), (-20.0, 10.0)])
        err = TopologyError(0, 1.0, "some other cause")
        with mock.patch.object(pinch, "check_pinched_section") as spy:
            self.assertIsNone(pinch.pinched_section_after_failure(err, ops, [original], SAFETY, 0, 1.0))
        self.assertEqual(spy.call_count, 0)


if __name__ == "__main__":
    unittest.main()
