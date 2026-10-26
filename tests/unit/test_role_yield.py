"""Role-yield: a shallow ``skin_lattice`` member crossing a full-depth
``primary_structure`` member OMITS the lattice station rather than merging
(and thereby truncating) the primary's own depth.

Without role-yield, a shallow lattice member crossing a full-depth primary
member drags it down to stiffener depth: on the anisogrid config, the union-merge
band (``0 <= gap < min_wall_mm``) truncates ``spar_front``/``spar_aft`` to the
lattice family's 1.5 mm depth on 102/300 and 68/300 layers of the NACA 2412
fixture. ``StructureGenerator._yield_lattice_to_primary`` (a pre-pass
inside ``_merge_cross_family_nodes``, gated on ``spec is not None``) omits the
``skin_lattice`` candidate instead of letting union-find merge it with the
``primary_structure`` candidate -- see that method's own docstring and
``_merge_cross_family_nodes``'s "WHY OMISSION, NOT 'MERGE AT THE DEEPER DEPTH'"
note for the full reasoning.

Two test classes, mirroring the existing hand-built/generator-fixture split
the ``test_node_merge.py`` / ``test_node_merge_scope.py`` pair:

- ``HandBuiltRoleYieldTests`` -- precise, hand-controlled ``WebIntent``
  pairs through the private ``_merge_cross_family_nodes`` entry point
  (same pattern ``test_node_merge.py``'s ``MergedSlotWidthAtTheBandTests``
  already uses), pinning the exact band/scope logic independent of any
  ``StructureSpec`` pitch/position/drift arithmetic.
- ``RealAnisogridFixtureRegressionTests`` -- the REAL generator driven by
  the shipped ``presets/plugin/anisogrid_lattice_config.json`` families
  over the real NACA 2412 60x60 fixture, a permanent regression pin for the
  measured defect (a sparse, deterministic layer sample rather than all 300
  body layers, to keep this file's own runtime bounded -- see that class's
  docstring for exactly which layers and why).
"""
import importlib.util
import json
import math
import unittest
from pathlib import Path

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import LayerContext, StructureGenerator, WebIntent
from aircraft_core.spec import ROLE_PRIMARY_STRUCTURE, ROLE_SKIN_LATTICE, StructureSpec, WebFamily
from stub_ops import PolygonShape, StubOps

# ``_merge_cross_family_nodes``/``_merge_node_component`` REQUIRE a
# ``LayerContext`` -- the merge-halo breach raises a ``TopologyError`` that
# must name a real layer index and Z, and an optional ctx defaulting to
# ``-1``/``0.0`` could emit a diagnostic naming the wrong layer. These direct-call tests do not exercise that raise, so any
# concrete context serves; it is named rather than inlined so it is obvious
# these values are inert here.
_MERGE_TEST_CTX = LayerContext(layer_index=0, print_z=0.0)


_SAFETY_OFFSET_MM = 0.42
_MIN_WALL_MM = 2.0 * _SAFETY_OFFSET_MM

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"
ANISOGRID_CONFIG_PATH = REPO_ROOT / "presets" / "plugin" / "anisogrid_lattice_config.json"


def _mk_intent(x, width_mm, family_index, family_name):
    """Same hand-built-``WebIntent`` shape ``test_node_merge.py``'s own
    ``_mk_intent`` uses (same Y-range, ``start``/``end`` construction) --
    duplicated here rather than imported, matching that module's own choice
    to keep this precise-gap helper local and dependency-free."""
    return WebIntent(
        start=(x, 3.0),
        end=(x, -2.0),
        width_mm=width_mm,
        family_index=family_index,
        family_name=family_name,
    )


def _two_family_spec(role_a, role_b):
    fam_a = WebFamily(name="a", role=role_a)
    fam_b = WebFamily(name="b", role=role_b)
    return StructureSpec(families=(fam_a, fam_b))


class HandBuiltRoleYieldTests(unittest.TestCase):
    """Precise-gap coverage, mirroring ``test_node_merge.py``'s
    ``MergedSlotWidthAtTheBandTests``/``BoundaryCaseTests`` pattern."""

    def _pair(self, gap_mm, width_mm=0.021, family_index_a=0, family_index_b=1,
              name_a="a", name_b="b"):
        a = _mk_intent(0.0, width_mm, family_index_a, name_a)
        b_min_x = width_mm / 2.0 + gap_mm
        b_center = b_min_x + width_mm / 2.0
        b = _mk_intent(b_center, width_mm, family_index_b, name_b)
        return a, b

    def test_cross_role_in_band_omits_lattice_keeps_primary_unchanged(self):
        """Primary-first-in-X ordering: family 0 is primary, family 1 is
        lattice, gap well inside the merge band."""
        spec = _two_family_spec(ROLE_PRIMARY_STRUCTURE, ROLE_SKIN_LATTICE)
        a, b = self._pair(gap_mm=0.10, name_a="spar", name_b="lattice")
        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [a, b], safety_offset_mm=_SAFETY_OFFSET_MM, spec=spec, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(
            kept,
            [a],
            "the primary_structure candidate must survive completely "
            "UNCHANGED (same start/end/width) -- got {}".format(kept),
        )
        self.assertEqual(len(omitted), 1)
        self.assertTrue(
            omitted[0].reason.startswith("role yield:"),
            omitted[0].reason,
        )
        self.assertEqual(omitted[0].family_name, "lattice")
        self.assertAlmostEqual(omitted[0].x_mm, b.start[0])

    def test_cross_role_in_band_lattice_first_in_x_still_omits_lattice(self):
        """Same band, but the LATTICE candidate is first in ascending-X
        order this time (family_index 0 = lattice, family_index 1 =
        primary) -- proves the victim is chosen by ROLE, never by which
        side of the pair happens to sort first."""
        spec = _two_family_spec(ROLE_SKIN_LATTICE, ROLE_PRIMARY_STRUCTURE)
        a, b = self._pair(gap_mm=0.10, name_a="lattice", name_b="spar")
        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [a, b], safety_offset_mm=_SAFETY_OFFSET_MM, spec=spec, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(kept, [b], "the primary (b, family 1) must survive unchanged")
        self.assertEqual(len(omitted), 1)
        self.assertEqual(omitted[0].family_name, "lattice")
        self.assertTrue(omitted[0].reason.startswith("role yield:"))

    def test_same_role_lattice_pair_still_merges_exactly_as_before(self):
        """A/B both ``skin_lattice`` (the default role): ordinary merge fires
        exactly as without role-yield -- no omission, one merged WebIntent
        spanning the union interval."""
        spec = _two_family_spec(ROLE_SKIN_LATTICE, ROLE_SKIN_LATTICE)
        a, b = self._pair(gap_mm=0.10)
        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [a, b], safety_offset_mm=_SAFETY_OFFSET_MM, spec=spec, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(omitted, [])
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0].merged_family_indices, (0, 1))

    def test_same_role_primary_pair_still_merges_exactly_as_before(self):
        """Two primary_structure candidates in-band: also untouched by the
        role-yield pre-pass (it only fires on a genuine role MISMATCH)."""
        spec = _two_family_spec(ROLE_PRIMARY_STRUCTURE, ROLE_PRIMARY_STRUCTURE)
        a, b = self._pair(gap_mm=0.10)
        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [a, b], safety_offset_mm=_SAFETY_OFFSET_MM, spec=spec, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(omitted, [])
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0].merged_family_indices, (0, 1))

    def test_cross_role_true_overlap_is_d009_territory_untouched(self):
        """``gap < 0`` (true overlap) is concern, not merge or this pre-pass --
        both candidates survive at their ORIGINAL footprints and depths,
        exactly as without role-yield."""
        spec = _two_family_spec(ROLE_PRIMARY_STRUCTURE, ROLE_SKIN_LATTICE)
        a, b = self._pair(gap_mm=-0.01, name_a="spar", name_b="lattice")
        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [a, b], safety_offset_mm=_SAFETY_OFFSET_MM, spec=spec, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(omitted, [])
        self.assertEqual(sorted(kept, key=lambda i: i.start[0]), sorted([a, b], key=lambda i: i.start[0]))

    def test_cross_role_clearing_min_wall_is_untouched(self):
        """A gap that already clears ``min_wall_mm`` on its own: nothing to
        merge or omit, cross-role or not."""
        spec = _two_family_spec(ROLE_PRIMARY_STRUCTURE, ROLE_SKIN_LATTICE)
        a, b = self._pair(gap_mm=_MIN_WALL_MM + 0.05, name_a="spar", name_b="lattice")
        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [a, b], safety_offset_mm=_SAFETY_OFFSET_MM, spec=spec, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(omitted, [])
        self.assertEqual(len(kept), 2)

    def test_without_spec_falls_back_to_pre_task_merge_behaviour(self):
        """Every existing direct caller of ``_merge_cross_family_nodes``
        (``test_node_merge.py``, ``test_node_merge_scope.py``) omits
        ``spec`` entirely -- this pre-pass MUST be a complete no-op in that
        case, reproducing the exact pre-task merge behaviour (role unknown
        without a spec, so a cross-role pair still merges rather than
        omitting)."""
        a, b = self._pair(gap_mm=0.10, name_a="spar", name_b="lattice")
        gen = StructureGenerator()
        kept, omitted = gen._merge_cross_family_nodes(
            [a, b], safety_offset_mm=_SAFETY_OFFSET_MM, ctx=_MERGE_TEST_CTX
        )
        self.assertEqual(omitted, [])
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0].merged_family_indices, (0, 1))


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_role_yield", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _naca2412_polygon():
    """Same 60 mm chord / 60 mm span NACA 2412 section every other real -fixture
    test file in this directory builds (``test_node_merge_scope.py``'s own
    ``_naca2412_polygon``, etc.)."""
    mf = _load_make_fixtures_module()
    m, p, t = mf.parse_naca4("2412")
    _triangles, outline_mm, _bbox = mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm))


def _anisogrid_cap_config():
    with open(ANISOGRID_CONFIG_PATH) as f:
        return json.load(f)["config"][0]["cap_config"]


def _build_spec(cap_config, family_dicts):
    families = tuple(
        WebFamily(
            name=fam["name"],
            role=fam["role"],
            rib_pitch_mm=fam["rib_pitch_mm"],
            web_position=fam["web_position"],
            web_depth_mm=fam["web_depth_mm"],
            entry_lower=fam["entry_lower"],
            z_drift_mm_per_mm=fam["z_drift_mm_per_mm"],
            z_period_mm=fam["z_period_mm"],
        )
        for fam in family_dicts
    )
    return StructureSpec(
        gap_ratio=cap_config["gap_ratio"],
        skin_margin_mm=cap_config["skin_margin_mm"],
        web_position=cap_config["web_position"],
        families=families,
    )


def _intent_depth_mm(intent):
    return math.hypot(intent.end[0] - intent.start[0], intent.end[1] - intent.start[1])


class RealAnisogridFixtureRegressionTests(unittest.TestCase):
    """Permanent regression fixture at the anisogrid config's real
    geometry, mirroring the measured defect
    (``spar_front``/``spar_aft`` truncated to the lattice family's 1.5 mm
    depth on 102/300 and 68/300 layers of the NACA 2412 60x60 fixture).

    Runs a deterministic, sparse layer sample (every 30th of the 300 body
    layers, ``0.2 mm`` layer height over the 60 mm span -- 10 layers) rather
    than the full 300, to keep the runtime bounded; the full-300-layer
    measurement gives 102/300 and 68/300, so this class exists to catch a
    REGRESSION on a fast, deterministic subset, not to re-derive that
    measurement. Both full-config and
    spars-only generation are run ONCE in ``setUpClass`` and shared by every
    test method below, rather than re-run per test.

    LIVE-CONFIG COUPLING, DELIBERATE (unlike ``test_lattice_crossing.py``,
    which hand-freezes its regression numbers): this class reads
    ``presets/plugin/anisogrid_lattice_config.json`` as shipped, not a
    hand-frozen copy, because the shipped file IS the fixture of interest
    (the anisogrid config's real geometry). The cost: if the shipped preset is ever retuned so no
    family lands in merge band against a spar,
    ``test_role_yield_omission_actually_fires_on_this_fixture`` will fail --
    correctly, since the scenario it pins would no longer exist -- and a reader
    who does not check this docstring may mistake that for a regression in the
    FIX rather than a change in the FIXTURE.
    """

    LAYER_HEIGHT_MM = 0.2
    N_LAYERS = 300
    SAMPLE_STRIDE = 30

    @classmethod
    def setUpClass(cls):
        polygon = _naca2412_polygon()
        ops = StubOps()
        frame = LayerFrame.from_polygon(ops, polygon)
        cap_config = _anisogrid_cap_config()
        all_families = cap_config["families"]
        full_spec = _build_spec(cap_config, all_families)
        spars_only_spec = _build_spec(
            cap_config, [f for f in all_families if f["role"] == ROLE_PRIMARY_STRUCTURE]
        )
        cls.role_by_index = {i: f["role"] for i, f in enumerate(all_families)}
        cls.lattice_names = [
            f["name"] for f in all_families if f["role"] == ROLE_SKIN_LATTICE
        ]

        def _generate(spec, layer_index):
            z = (layer_index + 1) * cls.LAYER_HEIGHT_MM
            layer = LayerContext(layer_index=layer_index, print_z=z)
            gen = StructureGenerator()
            return gen.generate(frame, spec, 0.42, layer, safety_offset_mm=0.42)

        sample_layers = range(0, cls.N_LAYERS, cls.SAMPLE_STRIDE)
        cls.full_results = {i: _generate(full_spec, i) for i in sample_layers}
        cls.ref_results = {i: _generate(spars_only_spec, i) for i in sample_layers}

    def test_primary_spars_survive_at_full_depth_on_every_sampled_layer(self):
        """The regression role-yield exists to prevent: on every sampled layer,
        every ``spar_front``/``spar_aft`` ``WebIntent`` in the full
        (lattice + primary) config has the SAME depth as the same family on
        the same layer of a spars-ONLY spec -- i.e. the lattice never drags
        it down, and it is never folded into a ``merged_family_indices``
        component at all."""
        failures = []
        for i, full in self.full_results.items():
            ref = self.ref_results[i]
            ref_by_name = {intent.family_name: intent for intent in ref.intents}
            for intent in full.intents:
                if intent.family_name not in ("spar_front", "spar_aft"):
                    continue
                if intent.merged_family_indices:
                    failures.append(
                        "layer {}: {} was merged ({}) -- a primary member "
                        "must never be folded into a merge".format(
                            i, intent.family_name, intent.merged_family_indices
                        )
                    )
                    continue
                ref_intent = ref_by_name.get(intent.family_name)
                if ref_intent is None:
                    failures.append(
                        "layer {}: {} present in full config but absent "
                        "from the spars-only reference".format(i, intent.family_name)
                    )
                    continue
                full_depth = _intent_depth_mm(intent)
                ref_depth = _intent_depth_mm(ref_intent)
                if abs(full_depth - ref_depth) > 1e-6:
                    failures.append(
                        "layer {}: {} depth {:.6f} mm in the full config "
                        "differs from the spars-only reference depth "
                        "{:.6f} mm -- the primary member was dragged down "
                        "by a nearby lattice station".format(
                            i, intent.family_name, full_depth, ref_depth
                        )
                    )
        self.assertEqual(failures, [], "\n".join(failures))

    def test_role_yield_omission_actually_fires_on_this_fixture(self):
        """A vacuous "primary always survives" pass could also mean the
        pre-pass never engaged at all (e.g. if the config no longer
        produces any in-band cross-role pair). Assert it DOES engage on at
        least one sampled layer, with the expected reason prefix."""
        saw_role_yield = False
        for result in self.full_results.values():
            for o in result.omitted:
                if o.reason.startswith("role yield:"):
                    saw_role_yield = True
                    self.assertIn(o.family_name, self.lattice_names)
        self.assertTrue(
            saw_role_yield,
            "expected at least one 'role yield' omission across the "
            "sampled layers of the real anisogrid config -- if this "
            "fires, either the fixture or the fix regressed",
        )

    def test_no_merged_intent_ever_spans_a_primary_and_a_lattice_family(self):
        """Directly pins the invariant role-yield exists to establish: no
        ``WebIntent.merged_family_indices`` on this fixture ever contains both
        a ``primary_structure`` family index and a ``skin_lattice`` one --
        ordinary merge stays same-role-only for this config, by
        construction."""
        for i, result in self.full_results.items():
            for intent in result.intents:
                if not intent.merged_family_indices:
                    continue
                roles = set(self.role_by_index[idx] for idx in intent.merged_family_indices)
                self.assertEqual(
                    len(roles),
                    1,
                    "layer {}: merged intent spans roles {} (families {}) "
                    "-- a cross-role merge must never occur".format(
                        i, roles, intent.merged_family_indices
                    ),
                )


if __name__ == "__main__":
    unittest.main()
