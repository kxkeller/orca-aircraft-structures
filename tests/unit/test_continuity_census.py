"""``tools/continuity_census.py``: the pure analysis half (``analyse``) on a
hand-built preflight result. The sweep itself (``run_preflight``) is the
existing ``tools/preflight_wing.py`` and is covered there."""
import os
import sys
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))

import continuity_census  # noqa: E402
from aircraft_core.continuity import encode_rows  # noqa: E402


def _layer(i, rows=(), omit=(), **kw):
    rec = {
        "layer_index": i, "print_z": round(0.2 * (i + 1), 4),
        "mutated": bool(rows), "failed": False, "shutdown": False,
        "degradations": [], "message": None,
        "extra": {"webs": encode_rows(rows), "web_omit": encode_rows(omit)},
    }
    rec.update(kw)
    return rec


def _preflight(layers):
    return {
        "stl_path": "x.stl", "stl_sha256": "s", "config_sha256": "c",
        "status": "DEGRADED",
        "assumptions": {"layer_height_mm": 0.2, "wall_width_mm": 0.42},
        "cap_config": {"families": [{"name": "lat"}, {"name": "flank"}]},
        "t3_meta": {"root_solid_layers": 1},
        "t3_summary": {"layers_seen": len(layers), "mutated_count": 0,
                       "failed_count": 0, "shutdown_count": 0,
                       "degradation_counts_by_kind": {}},
        "t3_layers": layers,
    }


class AnalyseTests(unittest.TestCase):
    def test_blink_is_counted_with_gap_reason_and_family_name(self):
        web = [0, 2, "U", 1.5, 10.0]
        layers = [_layer(0)]
        layers += [_layer(i, [web]) for i in (1, 2, 3)]
        layers += [_layer(4, omit=[[0, 2, "U", 10.0, "mitre_clearance"]]),
                   _layer(5, omit=[[0, 2, "U", 10.0, "mitre_clearance"]])]
        layers += [_layer(i, [web]) for i in (6, 7)]
        doc, violations = continuity_census.analyse(
            _preflight(layers), label="t", per_station_reentrant=False)
        self.assertEqual(doc["violation_counts"]["blink"], 1)
        self.assertEqual(doc["blinks_genuine"], 1)
        self.assertEqual(doc["blink_gap_reason_counts"], {"mitre_clearance": 1})
        self.assertEqual(doc["blink_gap_length_histogram"]["2-4"], 1)
        self.assertEqual(doc["by_family"]["0"]["name"], "lat")
        self.assertEqual(doc["per_web"][0]["runs"], 2)
        self.assertEqual(len(violations), doc["violation_total"])
        self.assertIn("Mesh replay sections", " ".join(doc["known_differences"]))

    def test_blink_across_a_replay_failed_layer_is_labelled_artefact(self):
        web = [0, 2, "U", 1.5, 10.0]
        layers = [_layer(0), _layer(1, [web]), _layer(2, [web])]
        layers.append(_layer(3, failed=True, message="slot disconnects island"))
        layers += [_layer(4, [web]), _layer(5, [web])]
        doc, _ = continuity_census.analyse(
            _preflight(layers), label="t", per_station_reentrant=False)
        self.assertEqual(doc["blinks_adjacent_to_replay_artefact"], 1)
        self.assertEqual(doc["blinks_genuine"], 0)
        self.assertEqual([f["layer"] for f in doc["mesh_replay_failed_layers"]], [3])
        self.assertEqual(len(doc["whole_layer_gaps"]), 1)
        self.assertTrue(doc["whole_layer_gaps"][0]["replay_artefact"])

    def test_summary_text_mentions_the_key_numbers(self):
        web = [0, 2, "U", 1.5, 10.0]
        layers = [_layer(0), _layer(1, [web]), _layer(2, [web])]
        doc, _ = continuity_census.analyse(
            _preflight(layers), label="t", per_station_reentrant=False)
        text = continuity_census._summary_text(doc)
        self.assertIn("VIOLATIONS", text)
        self.assertIn("dx_max=0.347", text)


if __name__ == "__main__":
    unittest.main()
