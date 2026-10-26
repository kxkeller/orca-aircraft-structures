"""Post-validation opposing-pair ablation (rather than a lateral pre-check).
Hermetic: reads only tracked fixtures under
``tests/fixtures/island_disconnect`` (never ``host/``).

Real fixtures: layer 947 (tip-down, lateral hazard) and layer 454 (root-down,
far-cap hazard against a reflex vertex). Both split the island because of the
``tie_0.60c`` pair only. Negative controls: a split the pair does not cause
still raises (with the ablation note), a non-island failure raises with no
ablation attempted, and a healthy layer never enters the retry path.
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

import aircraft_core.pair_ablation as pa  # noqa: E402
import aircraft_core.slot as slotmod  # noqa: E402
from aircraft_core.diagnostics import LayerRecord, RunDiagnostics  # noqa: E402
from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import (  # noqa: E402
    MITRE_OMISSION_MARKER,
    DetectedChannel,
    LayerContext,
    StructureGenerator,
    WebIntent,
)
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


def _spec():
    with open(FIXTURES / "elevon_cap_config.json", encoding="utf-8") as fh:
        return sa.build_spec_from_cap_config(json.load(fh))


class _Layer:
    """Generate + cut one fixture layer the way the plugin does."""

    def __init__(self, fixture_name, spec):
        with open(FIXTURES / fixture_name, encoding="utf-8") as fh:
            self.fx = json.load(fh)
        self.spec = spec
        self.ops = MiterShapelyOps()
        self.original = self.ops.polygon_from_points(
            [tuple(p) for p in self.fx["host_contour_pts_mm"]]
        )
        assert self.original.exterior.is_ccw
        frame = LayerFrame.from_polygon(self.ops, self.original)
        channels = tuple(
            DetectedChannel(
                center_x_mm=c["center_x_mm"],
                center_y_mm=c["center_y_mm"],
                diameter_mm=c["diameter_mm"],
            )
            for c in self.fx.get("detected_channels", [])
        )
        self.index = self.fx["layer_index"]
        self.z = self.fx["print_z_mm"]
        self.result = StructureGenerator().generate(
            frame, spec, self.fx["wall_width_mm"],
            LayerContext(layer_index=self.index, print_z=self.z,
                         detected_channels=channels),
            safety_offset_mm=self.fx["safety_offset_mm"],
        )
        self.intents = self.result.intents
        builder = slotmod.SlotBuilder()
        self.slots = [
            self.ops.polygon_from_points(builder.build(i, spec.overshoot_mm).points)
            for i in self.intents
        ]
        self.labels = [
            "family {} ({!r})".format(i.family_index, i.family_name)
            for i in self.intents
        ]
        self.family_indices = [i.family_membership() for i in self.intents]
        self.validator = TopologyValidator(self.ops, safety_offset_mm=SAFETY)

    def run(self):
        replacement = list(self.ops.diff(self.original, self.slots))
        return pa.validate_with_pair_ablation(
            self.ops, self.validator, [self.original], replacement,
            self.intents, self.slots, self.spec, self.index, self.z,
            self.labels, self.family_indices, SAFETY,
        )

    def revalidate(self, rescue):
        keep = list(rescue.kept_indices)
        slots = [self.slots[i] for i in keep]
        replacement = list(self.ops.diff(self.original, slots))
        self.validator.validate(
            [self.original], replacement, slots, self.spec, self.index, self.z,
            slot_labels=[self.labels[i] for i in keep],
            family_indices=[self.family_indices[i] for i in keep],
        )
        return replacement


@NEEDS
class RealFixturesAreRescuedTests(unittest.TestCase):
    def _check(self, fixture_name, expected_slots):
        layer = _Layer(fixture_name, _spec())
        # Without the layer fails exactly as recorded in-host.
        replacement = list(layer.ops.diff(layer.original, layer.slots))
        with self.assertRaises(TopologyError) as ctx:
            layer.validator.validate(
                [layer.original], replacement, layer.slots, layer.spec,
                layer.index, layer.z, slot_labels=layer.labels,
                family_indices=layer.family_indices,
            )
        self.assertIn("slot disconnects material island", str(ctx.exception))

        rescue = layer.run()
        self.assertIsNotNone(rescue)
        self.assertEqual(rescue.family_name, "tie_0.60c")
        self.assertEqual(
            [i + 1 for i in rescue.removed_indices], expected_slots
        )
        # Whole pair removed, nothing else.
        removed = [layer.intents[i] for i in rescue.removed_indices]
        self.assertEqual({r.family_name for r in removed}, {"tie_0.60c"})
        self.assertEqual({r.entry_lower for r in removed}, {False, True})
        self.assertEqual(
            len(rescue.kept_indices), len(layer.intents) - 2
        )
        reason = rescue.omitted.reason
        self.assertTrue(reason.startswith(MITRE_OMISSION_MARKER))
        self.assertIn("layer {}".format(layer.index), reason)
        self.assertIn("Z={:.2f}".format(layer.z), reason)
        self.assertIn("tie_0.60c", reason)
        self.assertIn("x={:.4f}".format(rescue.station_x_mm), reason)
        self.assertIn(
            "pair removal restored one island at safety_offset_mm=0.42", reason
        )
        self.assertTrue(rescue.implicated)
        self.assertGreaterEqual(rescue.elapsed_ms, 0.0)
        # The complete validator accepts the ablated layer: one island.
        replacement = layer.revalidate(rescue)
        self.assertEqual(len(replacement), 1)
        return rescue

    def test_layer_947_tip_down(self):
        rescue = self._check("layer947_tie_pair_lateral_minimal.json", [21, 22])
        # Only the lower (-Y) member is the validator's culprit there.
        self.assertEqual(len(rescue.implicated), 1)

    def test_layer_454_root_down(self):
        rescue = self._check("layer454_rootdown_tie_minimal.json", [20, 21])
        # Slot 20 (upper member) alone is the culprit; slot 21 alone is not.
        self.assertEqual(len(rescue.implicated), 1)
        self.assertIn("slot 20", rescue.implicated[0])

    def test_generator_no_longer_prechecks_lateral(self):
        # withdrawn: the pair is generated at the 947 station.
        layer = _Layer("layer947_tie_pair_lateral_minimal.json", _spec())
        tie = [i for i in layer.intents if i.family_name == "tie_0.60c"]
        self.assertEqual(len(tie), 2)
        self.assertFalse(
            [o for o in layer.result.omitted
             if o.family_name == "tie_0.60c" and "LATERAL" in o.reason]
        )

    def test_report_fields(self):
        layer = _Layer("layer454_rootdown_tie_minimal.json", _spec())
        rescue = layer.run()
        diag = RunDiagnostics()
        diag.record(LayerRecord(
            layer_index=layer.index, print_z=layer.z, mutated=True,
            degradations=[{"kind": "pair_ablation", "x_mm": 0.0, "reason": "r"}],
            extra={"pair_ablation": rescue.as_dict()},
        ))
        diag.record(LayerRecord(layer_index=layer.index + 1, print_z=layer.z + 0.2,
                                mutated=True, extra={"pair_ablation": None}))
        report = diag.to_dict()
        self.assertEqual(report["pair_ablation_rescue_count"], 1)
        entry = report["pair_ablation_rescue_layers"][0]
        self.assertEqual(entry["layer_index"], 454)
        self.assertEqual(entry["family_name"], "tie_0.60c")
        self.assertIn("elapsed_ms", entry)
        self.assertEqual(report["degraded_layer_count"], 1)
        json.dumps(report)


@NEEDS
class PairStationDiscoveryTests(unittest.TestCase):
    def test_only_declared_pair_families_and_not_merged(self):
        layer = _Layer("layer454_rootdown_tie_minimal.json", _spec())
        stations = pa.find_pair_stations(layer.intents, layer.spec)
        self.assertTrue(stations)
        for st in stations:
            for i in st.member_indices:
                self.assertEqual(layer.intents[i].family_name, "tie_0.60c")
        # Deterministic order: (family index, x).
        keys = [(s.family_index, s.x_mm) for s in stations]
        self.assertEqual(keys, sorted(keys))


def _wedge_layer():
    """Layer 786's real contour: slot 12 (a NON-pair lattice slot) alone
    splits the island. A healthy pair is added elsewhere on the section."""
    with open(FIXTURES / "layer786_entry_wedge_minimal.json", encoding="utf-8") as fh:
        fx = json.load(fh)
    ops = MiterShapelyOps()
    original = ops.polygon_from_points([tuple(p) for p in fx["host_contour_pts_mm"]])
    frame = LayerFrame.from_polygon(ops, original)
    pair_family = WebFamily(
        name="tie_test", role="primary_structure", web_position=0.5,
        opposing_pair=True,
    )
    plain_family = WebFamily(
        name="wedge", role="skin_lattice", web_position=0.1, web_depth_mm=1.5,
    )
    spec = StructureSpec(
        skin_margin_mm=1.0, gap_ratio=0.05, families=(pair_family, plain_family),
    )
    result = StructureGenerator().generate(
        frame, spec, 0.42, LayerContext(layer_index=786, print_z=fx["print_z_mm"]),
        safety_offset_mm=SAFETY,
    )
    builder = slotmod.SlotBuilder()
    intents = list(result.intents)
    slots = [
        ops.polygon_from_points(builder.build(i, spec.overshoot_mm).points)
        for i in intents
    ]
    return fx, ops, original, spec, intents, slots


@NEEDS
class NegativeControlTests(unittest.TestCase):
    def test_split_not_caused_by_pair_still_raises_with_note(self):
        fx, ops, original, spec, intents, slots = _wedge_layer()
        pairs = [i for i in intents if i.family_name == "tie_test"]
        self.assertEqual(len(pairs), 2, "control needs a placed healthy pair")
        wedge = WebIntent(
            start=(-22.54, 7.1), end=(-22.54, 5.55), width_mm=0.021,
            family_index=1, family_name="wedge", entry_lower=False,
        )
        wedge_slot = ops.polygon_from_points([
            tuple(p) for p in fx["slot_points_mm"]
        ])
        intents = intents + [wedge]
        slots = slots + [wedge_slot]
        labels = ["s{}".format(k) for k in range(len(slots))]
        fam = [i.family_membership() for i in intents]
        validator = TopologyValidator(ops, safety_offset_mm=SAFETY)
        replacement = list(ops.diff(original, slots))
        with self.assertRaises(TopologyError) as ctx:
            pa.validate_with_pair_ablation(
                ops, validator, [original], replacement, intents, slots, spec,
                786, fx["print_z_mm"], labels, fam, SAFETY,
            )
        text = str(ctx.exception)
        self.assertIn("slot disconnects material island", text)
        self.assertIn("Pair ablation was tried", text)
        self.assertIn("did not restore", text)

    def test_non_island_failure_raises_without_ablation(self):
        ops = MiterShapelyOps()
        original = ops.polygon_from_points(
            [(-20.0, -10.0), (20.0, -10.0), (20.0, 10.0), (-20.0, 10.0)]
        )
        frame = LayerFrame.from_polygon(ops, original)
        family = WebFamily(name="tie_test", role="primary_structure",
                           web_position=0.5, opposing_pair=True)
        spec = StructureSpec(skin_margin_mm=1.0, gap_ratio=0.05, families=(family,))
        result = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=1.0),
            safety_offset_mm=SAFETY,
        )
        intents = list(result.intents)
        self.assertEqual(len(intents), 2)
        builder = slotmod.SlotBuilder()
        slots = [
            ops.polygon_from_points(builder.build(i, spec.overshoot_mm).points)
            for i in intents
        ]
        # A slot slicing the whole rectangle: two islands, not an erosion split.
        slots.append(ops.polygon_from_points(
            [(10.0, -11.0), (10.5, -11.0), (10.5, 11.0), (10.0, 11.0)]
        ))
        intents.append(WebIntent(
            start=(10.25, 11.0), end=(10.25, -11.0), width_mm=0.5,
            family_index=0, family_name="other", entry_lower=False,
        ))
        labels = ["a", "b", "c"]
        fam = [i.family_membership() for i in intents]
        validator = TopologyValidator(ops, safety_offset_mm=SAFETY)
        replacement = list(ops.diff(original, slots))
        with mock.patch.object(
            pa, "find_pair_stations", wraps=pa.find_pair_stations
        ) as spy:
            with self.assertRaises(TopologyError) as ctx:
                pa.validate_with_pair_ablation(
                    ops, validator, [original], replacement, intents, slots,
                    spec, 0, 1.0, labels, fam, SAFETY,
                )
        self.assertEqual(spy.call_count, 0, "ablation must not be attempted")
        self.assertNotIn("Pair ablation", str(ctx.exception))
        self.assertNotIn("slot disconnects material island", str(ctx.exception))

    def test_healthy_layer_keeps_pair_and_never_enters_retry(self):
        ops = MiterShapelyOps()
        original = ops.polygon_from_points(
            [(-20.0, -10.0), (20.0, -10.0), (20.0, 10.0), (-20.0, 10.0)]
        )
        frame = LayerFrame.from_polygon(ops, original)
        family = WebFamily(name="tie_test", role="primary_structure",
                           web_position=0.5, opposing_pair=True)
        spec = StructureSpec(skin_margin_mm=1.0, gap_ratio=0.05, families=(family,))
        result = StructureGenerator().generate(
            frame, spec, 0.42, LayerContext(layer_index=0, print_z=1.0),
            safety_offset_mm=SAFETY,
        )
        intents = list(result.intents)
        self.assertEqual(len(intents), 2)
        builder = slotmod.SlotBuilder()
        slots = [
            ops.polygon_from_points(builder.build(i, spec.overshoot_mm).points)
            for i in intents
        ]
        labels = ["a", "b"]
        fam = [i.family_membership() for i in intents]
        validator = TopologyValidator(ops, safety_offset_mm=SAFETY)
        replacement = list(ops.diff(original, slots))
        with mock.patch.object(
            pa, "attempt_pair_ablation", wraps=pa.attempt_pair_ablation
        ) as spy:
            rescue = pa.validate_with_pair_ablation(
                ops, validator, [original], replacement, intents, slots, spec,
                0, 1.0, labels, fam, SAFETY,
            )
        self.assertIsNone(rescue)
        self.assertEqual(spy.call_count, 0)


if __name__ == "__main__":
    unittest.main()
