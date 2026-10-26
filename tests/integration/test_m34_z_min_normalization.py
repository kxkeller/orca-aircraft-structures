"""Z normalisation in ``_sweep_layers`` of ``tools/m34_rotation_equivalence.py``.

The sweep must translate the mesh so its own bottom sits at Z 0 before
comparing ``sec.z_min``/``sec.z_max`` against layer heights (the real host
drops the object onto the bed before slicing -- ``print_z`` is measured from
the OBJECT'S OWN bottom, never from the STL file's coordinate origin).

Two consequences this test pins directly, on a synthetic box mesh whose raw Z
sits well above 0 (like a real 450 mm wing whose z_min after stand-up was
24.307 mm):

1. ATTRIBUTED LAYERS. Every index in ``range(root, n_layers)`` is counted
   exactly once (ok, failed or skipped). Without normalisation the
   ``z <= z_min or z >= z_max`` filter would drop every index below the raw
   z_min from every tally, and ``root + ok + failed + skipped != n_layers``
   (with root=5 and z_min=24.307 the gap would be 117 layers).
2. INDEX/Z ALIGNMENT. "Offline layer i" and "in-host layer i" refer to the
   same absolute Z height: the first processed body layer sits near
   ``root * LAYER_HEIGHT_MM`` above the mesh's own bottom (z~1.0), which is
   what the host means by layer ``root``, not near the raw z_min (z~24.4,
   layer index ~122).
"""

import os
import sys
import unittest
from collections import defaultdict
from unittest.mock import patch

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.join(_THIS_DIR, "..", "..")
# Tracked mirror of the host's installed config (equal to it; see
# tests/unit/test_config_schema.py's drift guard). Passed explicitly so the
# test does not depend on the git-ignored host/data directory.
_TRACKED_LIVE_CONFIG = os.path.join(
    REPO_ROOT, "presets", "plugin", "live_host_config.json")
for _p in (_THIS_DIR, REPO_ROOT, os.path.join(REPO_ROOT, "tools")):
    _p = os.path.abspath(_p)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fake_orca  # noqa: E402
import m34_rotation_equivalence as m34  # noqa: E402
from shapely_ops import HAVE_SHAPELY, MiterShapelyOps  # noqa: E402

# Well above 0, the same order of magnitude as the real wing z_min (24.307 mm)
# -- large enough that an un-normalised ``z <= z_min`` filter would silently
# swallow every layer index from ``root`` up to
# ``round(Z0_MM / LAYER_HEIGHT_MM)``.
Z0_MM = 24.307
SPAN_MM = 4.0
X_MM = 20.0
Y_MM = 10.0


def _box_mesh(z0=Z0_MM, span=SPAN_MM):
    """An 8-vertex, 12-triangle watertight rectangular box, extruded along
    Z from ``z0`` to ``z0 + span``. Not wing-shaped -- the point of this
    test is the layer BOOKKEEPING around ``z_min``/``z_max``, not the
    structure generator's decisions, so every layer landing in ``ok``
    (accepted with no intents) or ``failed`` is equally fine as long as
    every layer index is accounted for exactly once.
    """
    verts = [
        (0.0, 0.0, z0), (X_MM, 0.0, z0), (X_MM, Y_MM, z0), (0.0, Y_MM, z0),
        (0.0, 0.0, z0 + span), (X_MM, 0.0, z0 + span),
        (X_MM, Y_MM, z0 + span), (0.0, Y_MM, z0 + span),
    ]
    tris = [
        (0, 1, 2), (0, 2, 3),      # bottom
        (4, 6, 5), (4, 7, 6),      # top
        (0, 4, 5), (0, 5, 1),      # side y=0
        (1, 5, 6), (1, 6, 2),      # side x=X_MM
        (2, 6, 7), (2, 7, 3),      # side y=Y_MM
        (3, 7, 4), (3, 4, 0),      # side x=0
    ]
    return verts, tris


@unittest.skipUnless(HAVE_SHAPELY, "shapely unavailable")
class ZMinNormalizationTests(unittest.TestCase):
    def setUp(self):
        self._fo, restore = fake_orca.install()
        self.addCleanup(restore)
        namespace = fake_orca.exec_bundle(
            os.path.join(m34.REPO_ROOT, "plugin", "aircraft_structures",
                         "aircraft_structures.py"))
        self.cap = namespace["AircraftStructuresM1"]()
        self.ops = MiterShapelyOps()
        self.spec = m34.load_spec(_TRACKED_LIVE_CONFIG)
        self.verts, self.tris = _box_mesh()

    def _sweep(self):
        return m34.run_arm(0.0, self.verts, self.tris, self.spec,
                            m34.WALL_MM, self.cap, self.ops)

    def test_layer_buckets_reconcile_to_n_layers(self):
        """(a) No layer index vanishes: root + ok + failed + skipped ==
        n_layers, exactly. Layers dropped by an un-normalised
        ``z <= z_min`` filter would be missing from every one of these
        tallies -- this assertion is the direct check for that gap.
        """
        r = self._sweep()
        n_skipped = r["skipped_out_of_z_range"] + r["skipped_no_outer_loop"]
        accounted = r["root"] + r["ok"] + r["failed"] + n_skipped
        self.assertEqual(
            accounted, r["n_layers"],
            "layer reconciliation gap: root=%d ok=%d failed=%d skipped=%d "
            "-> accounted=%d != n_layers=%d" % (
                r["root"], r["ok"], r["failed"], n_skipped, accounted,
                r["n_layers"]))
        self.assertTrue(r["reconciled"])

    def test_first_body_layer_z_is_relative_to_mesh_own_bottom(self):
        """(b) The first swept body layer (index ``root``) must be
        processed at ``z == root * LAYER_HEIGHT_MM`` -- i.e. measured from
        the MESH'S OWN bottom (post drop-to-bed), matching what the host
        means by layer ``root``. If the mesh's raw z_min (24.307 mm here)
        were not subtracted, the first index reaching ``section_at`` would
        be near ``round(z0/LAYER_HEIGHT_MM)`` (~122), not ``root`` (5), at
        z ~24.4 mm rather than ``root * LAYER_HEIGHT_MM`` (~1.0 mm).

        This pins the Z VALUE ACTUALLY PASSED TO ``section_at`` via a spy on
        ``MeshSectioner.section_at``, rather than re-deriving
        ``first_index * LAYER_HEIGHT_MM`` (which merely restates
        ``first_index == root`` and would pass identically whether
        ``section_at`` was called at ~1.0 mm or, wrongly, at ~24.4 mm, since
        both sides would be computed from the SAME ``first_index``).

        ``run_arm`` constructs two ``MeshSectioner`` instances -- a `probe`
        used only for theta detection (queried at a handful of
        span-fraction Z's, unrelated to this bug) and `sec`, the one
        ``_sweep_layers`` actually iterates layer-by-layer. The spy records
        every ``(instance, z)`` call across both; grouping by instance
        identity and taking the largest group isolates `sec`'s calls (one
        per swept layer, far more than the probe's handful) without
        depending on internal attribute names.
        """
        calls = []
        original_section_at = m34.MeshSectioner.section_at

        def _spy(mesh_self, z):
            calls.append((id(mesh_self), z))
            return original_section_at(mesh_self, z)

        with patch.object(m34.MeshSectioner, "section_at", _spy):
            r = self._sweep()

        by_instance = defaultdict(list)
        for instance_id, z in calls:
            by_instance[instance_id].append(z)
        self.assertTrue(by_instance, "section_at was never called")
        sweep_calls = max(by_instance.values(), key=len)

        expected_z = r["root"] * m34.LAYER_HEIGHT_MM
        actual_first_z = sweep_calls[0]
        self.assertAlmostEqual(
            actual_first_z, expected_z, places=9,
            msg="first z actually passed to section_at (%r) != "
                "root * LAYER_HEIGHT_MM (%r) -- the mesh's z_min was not "
                "normalized to 0 before sweeping" % (actual_first_z, expected_z))


if __name__ == "__main__":
    unittest.main()
