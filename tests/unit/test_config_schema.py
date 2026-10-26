"""Tests for aircraft_core/config_schema.py.

Includes a drift guard against plugin/aircraft_structures/capability.py's
DEFAULT_SPEC / _FAMILY_KEYS: since capability.py imports ``orca`` (which
does not exist outside the host), it cannot be imported directly here, so
its source is ``ast``-parsed and the DEFAULT_SPEC dict literal is
``literal_eval``-ed instead. This is what makes "single source of truth"
actually true rather than merely asserted.
"""
import ast
import unittest
from pathlib import Path
from typing import Dict, Tuple

from aircraft_core.config_schema import (
    DEFAULT_WALL_MM,
    FAMILY_DEFAULTS,
    FAMILY_FIELDS,
    FAMILY_KEYS,
    SPEC_DEFAULTS,
    SPEC_FIELDS,
    SPEC_KEYS,
    ConfigField,
    ConfigSchemaError,
    assert_valid_config,
    build_spec,
    validate_family_entry,
    validate_top_level,
)
from aircraft_core.spec import SpecError

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CAPABILITY_PATH = REPO_ROOT / "plugin" / "aircraft_structures" / "capability.py"
PRESETS_DIR = REPO_ROOT / "presets" / "plugin"


def _parse_capability_default_spec():
    tree = ast.parse(CAPABILITY_PATH.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "DEFAULT_SPEC" for t in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError("DEFAULT_SPEC assignment not found in capability.py")


def _capability_family_keys_is_bare_schema_reference():
    """True iff capability.py wires ``_FAMILY_KEYS = FAMILY_KEYS`` (a bare
    reference to this schema's own ``FAMILY_KEYS``, "supplied by the
    bundle") rather than repeating a literal key set that could drift from
    it. Structural, not value, equality: since the bundler concatenates
    config_schema.py ahead of capability.py into one namespace, a bare-name
    assignment makes drift impossible by construction rather than merely
    checked after the fact."""
    tree = ast.parse(CAPABILITY_PATH.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "_FAMILY_KEYS" for t in node.targets
        ):
            value = node.value
            return isinstance(value, ast.Name) and value.id == "FAMILY_KEYS"
    raise AssertionError("_FAMILY_KEYS assignment not found in capability.py")


def _parse_capability_mesh_alignment_tolerance_constant():
    """The bare module-level ``MESH_ALIGNMENT_TOLERANCE_MM = 0.02`` --
    kept as a plain literal (not wired to DEFAULT_SPEC by reference; see
    that dict's own comment for why) so this parses it independently and
    ``test_mesh_alignment_tolerance_default_matches_module_constant`` below
    is the thing that actually keeps the two literals from drifting apart."""
    tree = ast.parse(CAPABILITY_PATH.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "MESH_ALIGNMENT_TOLERANCE_MM"
            for t in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(
        "MESH_ALIGNMENT_TOLERANCE_MM assignment not found in capability.py"
    )


def _capability_spec_keys_is_bare_schema_reference():
    """Same check as ``_capability_family_keys_is_bare_schema_reference``,
    one scope up: True iff capability.py wires ``_SPEC_KEYS = SPEC_KEYS`` (a
    bare reference to this schema's own ``SPEC_KEYS``) rather than a literal
    key set that could drift from it -- the top-level equivalent of the
    family-key wiring, closing the same drift risk for
    ``_build_spec``'s unknown-top-level-key check."""
    tree = ast.parse(CAPABILITY_PATH.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "_SPEC_KEYS" for t in node.targets
        ):
            value = node.value
            return isinstance(value, ast.Name) and value.id == "SPEC_KEYS"
    raise AssertionError("_SPEC_KEYS assignment not found in capability.py")


class DriftGuardTests(unittest.TestCase):
    """capability.py must never silently diverge from this schema."""

    def test_default_spec_keys_match_schema(self):
        capability_default_spec = _parse_capability_default_spec()
        self.assertEqual(set(capability_default_spec), SPEC_KEYS)

    def test_default_spec_values_match_schema(self):
        capability_default_spec = _parse_capability_default_spec()
        self.assertEqual(capability_default_spec, SPEC_DEFAULTS)

    def test_mesh_alignment_tolerance_default_matches_module_constant(self):
        # DEFAULT_SPEC["mesh_alignment_tolerance_mm"] is a literal (must
        # stay ast.literal_eval-able), not a reference to
        # MESH_ALIGNMENT_TOLERANCE_MM -- this is the check that keeps the
        # two from drifting apart instead.
        module_constant = _parse_capability_mesh_alignment_tolerance_constant()
        capability_default_spec = _parse_capability_default_spec()
        self.assertEqual(
            capability_default_spec["mesh_alignment_tolerance_mm"],
            module_constant,
        )
        self.assertEqual(module_constant, 0.02)

    def test_family_keys_is_bare_reference_to_schema(self):
        self.assertTrue(
            _capability_family_keys_is_bare_schema_reference(),
            "capability.py's _FAMILY_KEYS must be wired to this schema's "
            "own FAMILY_KEYS (bare name), not a literal key list that "
            "could drift from it.",
        )

    def test_structural_analysis_family_keys_match_schema(self):
        """``tools/structural_analysis.py`` is NOT bundled -- it is a plain
        offline tool with its own literal ``_FAMILY_KEYS`` set, unlike
        capability.py's bare-name reference above, so it CAN silently drift
        from this schema (the bug class is "two
        measurements of a quantity that must agree" kept as two separate
        literals instead of one). Direct VALUE equality, not structural (ast)
        equality: unlike capability.py, structural_analysis.py has no ``orca``
        import, so it can be imported here directly."""
        from tools import structural_analysis as sa

        self.assertEqual(sa._FAMILY_KEYS, FAMILY_KEYS)

    def test_structural_analysis_spec_keys_match_schema(self):
        """Guards the second copy of the key lists, which has drifted from the
        schema repeatedly.

        ``tools/structural_analysis.py`` partitions a stored cap_config's
        top-level keys into ``_SPEC_KEYS`` (passed to ``StructureSpec``),
        ``_PLUGIN_ONLY_KEYS`` (legitimate, consumed by the plugin) and
        ``families``, and REFUSES anything else. Together those three must
        therefore be exactly this schema's ``SPEC_KEYS`` -- if the schema
        gains a key none of them knows, every stored config carrying it is
        refused wholesale by the guards that exist to check it.

        The drift is real: new schema keys (e.g. ``tip_alignment_skip_enabled``,
        ``feature_*``, ``mesh_alignment_sensitivity_enabled``,
        ``record_host_geometry_sidecar``) have been added to the tracked
        mirror without updating the tool, and each such change refused the
        stored config with a ``SpecError`` from ``build_spec_from_cap_config``,
        failing tests such as:
        ``test_lattice_crossing.LiveInstalledConfigSmokeTests.test_installed_config_slices_cleanly``,
        ``test_lattice_crossing.TrackedPresetConfigsSliceCleanlyTests.test_every_tracked_preset_slices_cleanly``,
        and both
        ``tests.integration.test_m34_z_min_normalization.ZMinNormalizationTests``
        methods (which reach it through ``tools/m34_rotation_equivalence.py``'s
        ``load_spec``, in ``setUp``). Note what is NOT in that list: the
        config-drift guards in THIS module do not call
        ``build_spec_from_cap_config`` and stayed green, and
        ``tools/m34_rotation_equivalence.py`` is a ~46-minute sweep tool that
        ``tools/run_tests.py`` never invokes, so it did not itself break.

        ``_SPEC_KEYS``'s own comments predicted each recurrence without
        preventing it -- and this test DETECTS a recurrence rather than
        preventing one.

        Asserted on the SCHEMA axis rather than ``StructureSpec``'s dataclass
        fields: a future PLUGIN-ONLY schema key -- the ``metrics_enabled``
        shape, which the host writes into config.json itself and which has no
        ``StructureSpec`` field -- would refuse the live config all over again,
        and a dataclass-field comparison would stay green through it."""
        from tools import structural_analysis as sa

        covered = set(sa._SPEC_KEYS) | sa._PLUGIN_ONLY_KEYS | {"families"}
        # Compared as a sorted symmetric difference rather than set
        # equality so the failure NAMES the drifting key: unittest
        # truncates a two-set diff of this size to "[640 chars]", which
        # is what the reader needs least.
        self.assertEqual(
            sorted(covered ^ set(SPEC_KEYS)),
            [],
            "config_schema.SPEC_KEYS and tools/structural_analysis.py's "
            "_SPEC_KEYS/_PLUGIN_ONLY_KEYS have drifted; a stored config "
            "carrying the key(s) above is refused wholesale by "
            "build_spec_from_cap_config",
        )

    def test_structural_analysis_spec_keys_are_all_structure_spec_fields(self):
        """Every ``_SPEC_KEYS`` entry must be a real ``StructureSpec``
        field, because ``build_spec_from_cap_config`` passes them as
        kwargs -- a name the dataclass does not accept is a TypeError at
        the call, not a refusal with a message.

        Kept as the one direction the schema equality above does NOT cover: the
        opposite direction, every StructureSpec field appearing in
        ``_SPEC_KEYS``, is deliberately NOT asserted, since a legitimately
        non-config field added to the dataclass later would fail it wrongly."""
        import dataclasses

        from aircraft_core.spec import StructureSpec
        from tools import structural_analysis as sa

        spec_fields = {f.name for f in dataclasses.fields(StructureSpec)}
        self.assertEqual(sorted(set(sa._SPEC_KEYS) - spec_fields), [])

    def test_spec_keys_is_bare_reference_to_schema(self):
        self.assertTrue(
            _capability_spec_keys_is_bare_schema_reference(),
            "capability.py's _SPEC_KEYS must be wired to this schema's "
            "own SPEC_KEYS (bare name), not a literal key list that could "
            "drift from it.",
        )


class LiveHostConfigMirrorDriftGuardTests(unittest.TestCase):
    """``presets/plugin/
    live_host_config.json`` is a HAND-SYNCED mirror of the gitignored
    ``host/data/orca_plugins/config.json`` (``.gitignore`` line 6 -- the live
    file is never tracked directly). A hand-synced mirror with no enforcement
    reproduces exactly the divergence it was created to fix, so this guard
    compares the two byte-for-byte (as parsed JSON, since the live file's own
    formatting is whatever the host last wrote) WHEN the live file is present,
    and skips with a stated reason when it is not -- so it enforces on the
    machine where the host has actually run and is inert elsewhere
    (CI, a fresh checkout, another contributor's machine without a host data
    directory)."""

    LIVE_CONFIG_PATH = REPO_ROOT / "host" / "data" / "orca_plugins" / "config.json"
    MIRROR_PATH = PRESETS_DIR / "live_host_config.json"

    def test_mirror_matches_live_config_when_live_config_is_present(self):
        if not self.LIVE_CONFIG_PATH.exists():
            self.skipTest(
                "{} not present on this machine (gitignored, host-runtime "
                "only) -- the mirror drift guard has nothing to compare "
                "against here and is inert by design; see this class's own "
                "docstring.".format(self.LIVE_CONFIG_PATH)
            )
        self.assertTrue(
            self.MIRROR_PATH.exists(),
            "{} is present but its tracked mirror {} is missing -- track a "
            "fresh copy (see presets/plugin/README.md's own section on "
            "this file).".format(self.LIVE_CONFIG_PATH, self.MIRROR_PATH),
        )
        import json

        with open(self.LIVE_CONFIG_PATH, "r", encoding="utf-8") as fh:
            live = json.load(fh)
        with open(self.MIRROR_PATH, "r", encoding="utf-8") as fh:
            mirror = json.load(fh)
        self.assertEqual(
            live, mirror,
            "presets/plugin/live_host_config.json has drifted from the live "
            "host config it mirrors (host/data/orca_plugins/config.json). "
            "Re-copy the live file and re-apply any config changes documented "
            "in the commit message, per presets/plugin/"
            "README.md's own instructions for keeping this mirror in sync.",
        )


class TopLevelValidationTests(unittest.TestCase):
    def test_empty_config_is_valid(self):
        self.assertEqual(validate_top_level({}), [])

    def test_unknown_key_is_error(self):
        errors = validate_top_level({"sking_margin_mm": 1.0})
        self.assertEqual(len(errors), 1)
        self.assertIn("unknown key", errors[0])

    def test_unknown_key_did_you_mean(self):
        errors = validate_top_level({"sking_margin_mm": 1.0})
        self.assertIn("skin_margin_mm", errors[0])

    def test_family_only_key_at_top_level_is_error(self):
        errors = validate_top_level({"entry_lower": True})
        self.assertEqual(len(errors), 1)
        self.assertIn("family-scoped key", errors[0])

    def test_wrong_type_is_error(self):
        errors = validate_top_level({"skin_margin_mm": "1.0"})
        self.assertEqual(len(errors), 1)
        self.assertIn("must be float", errors[0])

    def test_out_of_range_is_error(self):
        errors = validate_top_level({"web_position": 1.5})
        self.assertEqual(len(errors), 1)
        self.assertIn("must be <=", errors[0])

    def test_expected_hole_count_accepts_0_and_1_at_schema_layer(self):
        # validate_top_level never constructs a StructureSpec -- a pass here
        # is the schema layer's own opinion, not StructureSpec.validate's.
        self.assertEqual(validate_top_level({"expected_hole_count": 0}), [])
        self.assertEqual(validate_top_level({"expected_hole_count": 1}), [])

    def test_expected_hole_count_rejects_2_at_schema_layer(self):
        # Exercises validate_top_level directly, not build_spec/SpecError:
        # if ConfigField.maximum=1 were ever deleted from the schema, this
        # call would return [] and the assertion below would fail here,
        # at the schema layer, rather than surviving because
        # StructureSpec.validate still happens to catch N>1 downstream with
        # a different message (see that check's own "UNPROVEN extension"
        # docstring in aircraft_core/spec.py).
        errors = validate_top_level({"expected_hole_count": 2})
        self.assertEqual(len(errors), 1)
        self.assertIn("must be <=", errors[0])

    def test_expected_hole_count_rejects_negative_at_schema_layer(self):
        # Same distinction as above, for the lower bound: if
        # ConfigField.minimum=0 were deleted, this would return [] here even
        # though StructureSpec.validate's own "must be >= 0" check would
        # still catch it one layer down.
        errors = validate_top_level({"expected_hole_count": -1})
        self.assertEqual(len(errors), 1)
        self.assertIn("must be >=", errors[0])

    def test_null_where_not_allowed_is_error(self):
        errors = validate_top_level({"skin_margin_mm": None})
        self.assertEqual(len(errors), 1)
        self.assertIn("must not be null", errors[0])

    def test_null_where_allowed_passes(self):
        self.assertEqual(validate_top_level({"rib_pitch_mm": None}), [])

    def test_bool_field_rejects_int(self):
        errors = validate_top_level({"tip_shutdown_enabled": 1})
        self.assertEqual(len(errors), 1)
        self.assertIn("must be bool", errors[0])

    def test_families_must_be_list(self):
        errors = validate_top_level({"families": {"name": "x"}})
        self.assertEqual(len(errors), 1)
        self.assertIn("must be a list", errors[0])

    def test_valid_families_list_passes(self):
        errors = validate_top_level(
            {"families": [{"name": "spar", "rib_pitch_mm": 12.0}]}
        )
        self.assertEqual(errors, [])

    def test_config_not_a_dict_is_error(self):
        errors = validate_top_level([1, 2, 3])
        self.assertEqual(len(errors), 1)
        self.assertIn("must be an object", errors[0])


class FamilyValidationTests(unittest.TestCase):
    def test_unknown_family_key_is_error(self):
        errors = validate_family_entry({"nmae": "spar"}, 0)
        self.assertEqual(len(errors), 1)
        self.assertIn("unknown key", errors[0])
        self.assertIn("name", errors[0])

    def test_spec_only_key_in_family_is_error(self):
        errors = validate_family_entry({"skin_margin_mm": 1.0}, 0)
        self.assertEqual(len(errors), 1)
        self.assertIn("top-level-scoped key", errors[0])

    def test_entry_not_a_dict_is_error(self):
        errors = validate_family_entry("spar", 0)
        self.assertEqual(len(errors), 1)
        self.assertIn("must be an object", errors[0])

    def test_valid_entry_passes(self):
        self.assertEqual(
            validate_family_entry(
                {"name": "spar", "rib_pitch_mm": 12.0, "entry_lower": False}, 0
            ),
            [],
        )

    def test_role_key_is_schema_valid_string(self):
        # Schema layer only checks type (str) -- the skin_lattice /
        # primary_structure enum check is a cross-field WebFamily.validate()
        # rule, exercised via build_spec below.
        self.assertEqual(
            validate_family_entry({"name": "a", "role": "primary_structure"}, 0),
            [],
        )

    def test_role_wrong_type_is_schema_error(self):
        errors = validate_family_entry({"name": "a", "role": 1}, 0)
        self.assertEqual(len(errors), 1)
        self.assertIn("role", errors[0])


# Manifest of every bound SPEC_FIELDS / FAMILY_FIELDS is expected to
# declare, as (minimum, min_exclusive, maximum, max_exclusive) tuples. This
# is a PIN, same pattern as DriftGuardTests' DEFAULT_SPEC comparison: it is
# deliberately a second, independent statement of what the schema should
# say, not re-derived from the schema itself, so BoundSweepTests notices
# three distinct failure modes that a "loop over whatever SPEC_FIELDS
# currently declares" sweep cannot: a bound deleted outright (the field
# silently drops out of iteration and the loop just checks less, without
# failing -- proven empirically below), min_exclusive/max_exclusive flipped
# with the numeric bound left alone (the probe value and expected message
# both shift to match the new, weaker flag, so an enforcement-only sweep
# still passes), and a bound's numeric value edited to something else
# entirely. A field absent from this manifest whose ConfigField declares a
# bound, or a manifest entry whose field no longer declares that bound, is
# itself a mismatch this test class catches.
_EXPECTED_SPEC_BOUNDS: Dict[str, Tuple] = {
    "gap_ratio": (0.0, True, None, False),
    "slot_gap_mm": (0.0, True, None, False),
    "skin_margin_mm": (0.0, False, None, False),
    "root_solid_layers": (0, False, None, False),
    "web_position": (0.0, False, 1.0, False),
    "overshoot_mm": (0.0, True, None, False),
    "rib_pitch_mm": (0.0, True, None, False),
    "web_depth_mm": (0.0, True, None, False),
    "z_period_mm": (0.0, True, None, False),
    "max_rotation_deg": (0.0, True, None, False),
    "expected_hole_count": (0, False, 1, False),
    "web_tilt_threshold_deg": (0.0, True, None, False),
    "max_web_tilt_deg": (0.0, False, None, False),
    # the lateral-continuity threshold; None resolves to the derived default, a
    # given value must be > 0.
    "z_continuity_dx_max_mm": (0.0, True, None, False),
    # carbon-spar socket. spar_diameter_mm None disables the socket entirely,
    # so its lower bound is exclusive: a declared spar of zero diameter is a
    # config error, not "no spar".
    "spar_diameter_mm": (0.0, True, None, False),
    "spar_station": (0.0, False, 1.0, False),
    "spar_clearance_mm": (0.0, False, None, False),
    "spar_neck_width_mm": (0.0, True, None, False),
    # non-aerofoil FEATURE regions.
    "feature_aspect_ratio_max": (1.0, False, None, False),
    "feature_fill_ratio_min": (0.0, False, 1.0, False),
    "feature_lattice_pitch_mm": (0.0, True, None, False),
    # mesh-to-slice alignment guard tolerance. Exclusive lower
    # bound: a non-positive tolerance could never admit even a perfectly
    # aligned mesh.
    "mesh_alignment_tolerance_mm": (0.0, True, None, False),
}

_EXPECTED_FAMILY_BOUNDS: Dict[str, Tuple] = {
    "rib_pitch_mm": (0.0, True, None, False),
    "web_depth_mm": (0.0, True, None, False),
    "web_position": (0.0, False, 1.0, False),
    "slot_gap_mm": (0.0, True, None, False),
    "gap_ratio": (0.0, True, None, False),
    "z_period_mm": (0.0, True, None, False),
    "rib_pitch_chord_frac": (0.0, True, None, False),
    "pair_offset_mm": (0.0, True, None, False),
    "socket_flank_offset_mm": (0.0, False, None, False),
}


def _actual_bound_tuple(field: "ConfigField") -> Tuple:
    return (field.minimum, field.min_exclusive, field.maximum, field.max_exclusive)


class BoundManifestTests(unittest.TestCase):
    """The manifest itself must exactly match what SPEC_FIELDS/FAMILY_FIELDS
    currently declare -- both directions, so an added-but-unlisted bound is
    caught too, not just a deleted one."""

    def test_spec_fields_bounds_match_manifest_exactly(self):
        actual = {
            key: _actual_bound_tuple(field)
            for key, field in SPEC_FIELDS.items()
            if field.minimum is not None or field.maximum is not None
        }
        self.assertEqual(actual, _EXPECTED_SPEC_BOUNDS)

    def test_family_fields_bounds_match_manifest_exactly(self):
        actual = {
            key: _actual_bound_tuple(field)
            for key, field in FAMILY_FIELDS.items()
            if field.minimum is not None or field.maximum is not None
        }
        self.assertEqual(actual, _EXPECTED_FAMILY_BOUNDS)


class BoundSweepTests(unittest.TestCase):
    """Parametrized sweep over every SPEC_FIELDS / FAMILY_FIELDS bound
    (minimum, maximum, min_exclusive/max_exclusive): each declared bound
    must actually be enforced by the schema layer (validate_top_level /
    validate_family_entry), not merely by a downstream StructureSpec/
    WebFamily check. Before this class, only web_position's upper bound was
    pinned anywhere, and deleting any of the others (e.g. overshoot_mm's
    min_exclusive, or gap_ratio's minimum) would silently degrade schema
    validation to whatever SpecError build_spec happens to raise instead --
    one layer later, with a worse message, and nothing here failing.

    Exercises validate_top_level/validate_family_entry directly, never
    build_spec/assert_valid_config, so a deleted bound is caught at the
    schema's own opinion rather than surviving because SpecError still
    happens to catch the same out-of-range value one layer down.

    Enforcement alone (this class) is not sufficient to catch every
    deletion -- see BoundManifestTests above, which pins the bound values
    themselves independently of what the sweep discovers.
    """

    def _delta(self, field: ConfigField) -> float:
        return 1 if field.type is int else 1e-6

    def _probe_min(self, field: ConfigField):
        """Value that must be rejected for being at/under the declared
        minimum, plus the exact substring (including the discriminating
        trailing space) validate_*'s own message must contain. For
        min_exclusive fields the probe is the bound value ITSELF -- that is
        the whole point of the flag, and is exactly the distinction that
        would rot unnoticed if only "well below the minimum" were probed."""
        if field.min_exclusive:
            return field.minimum, "must be > "
        return field.minimum - self._delta(field), "must be >= "

    def _probe_max(self, field: ConfigField):
        if field.max_exclusive:
            return field.maximum, "must be < "
        return field.maximum + self._delta(field), "must be <= "

    def _iter_bounded_fields(self, fields_dict):
        """Yield (key, field) for every field declaring a bound. Raises
        loudly (rather than silently skipping) if a field declares a bound
        on a type this sweep does not know how to probe -- a sweep that
        silently covers nothing reads as coverage, which is worse than no
        sweep at all."""
        for key, field in fields_dict.items():
            if field.minimum is None and field.maximum is None:
                continue
            if field.type not in (int, float):
                raise AssertionError(
                    "field {!r} declares a bound (minimum={!r}, "
                    "maximum={!r}) but is type {} -- BoundSweepTests does "
                    "not know how to probe a non-numeric bounded field; "
                    "extend _probe_min/_probe_max before adding "
                    "this.".format(
                        key, field.minimum, field.maximum, field.type.__name__
                    )
                )
            yield key, field

    def test_sweep_is_not_vacuous(self):
        spec_bounded = list(self._iter_bounded_fields(SPEC_FIELDS))
        family_bounded = list(self._iter_bounded_fields(FAMILY_FIELDS))
        total = len(spec_bounded) + len(family_bounded)
        self.assertGreater(
            total,
            0,
            "bound sweep iterated zero fields -- either SPEC_FIELDS/"
            "FAMILY_FIELDS have no declared bounds any more (suspicious) or "
            "the sweep's own field discovery is broken.",
        )

    def _assert_bound_enforced(self, validate_fn, key, field, checked_counter):
        if field.minimum is not None:
            value, phrase = self._probe_min(field)
            errors = validate_fn(key, value)
            self.assertEqual(
                len(errors),
                1,
                "{!r}.minimum={!r} (min_exclusive={!r}) is not enforced "
                "any more: probing {!r}={!r} returned {!r} instead of "
                "exactly one error.".format(
                    key, field.minimum, field.min_exclusive, key, value, errors
                ),
            )
            self.assertIn(phrase, errors[0])
            checked_counter[0] += 1
        if field.maximum is not None:
            value, phrase = self._probe_max(field)
            errors = validate_fn(key, value)
            self.assertEqual(
                len(errors),
                1,
                "{!r}.maximum={!r} (max_exclusive={!r}) is not enforced "
                "any more: probing {!r}={!r} returned {!r} instead of "
                "exactly one error.".format(
                    key, field.maximum, field.max_exclusive, key, value, errors
                ),
            )
            self.assertIn(phrase, errors[0])
            checked_counter[0] += 1

    def test_every_spec_field_bound_is_enforced_at_schema_layer(self):
        checked = [0]
        for key, field in self._iter_bounded_fields(SPEC_FIELDS):
            self._assert_bound_enforced(
                lambda k, v: validate_top_level({k: v}), key, field, checked
            )
        self.assertGreater(checked[0], 0, "no SPEC_FIELDS bound was actually probed.")

    def test_every_family_field_bound_is_enforced_at_schema_layer(self):
        checked = [0]
        for key, field in self._iter_bounded_fields(FAMILY_FIELDS):
            self._assert_bound_enforced(
                lambda k, v: validate_family_entry({k: v}, 0), key, field, checked
            )
        self.assertGreater(
            checked[0], 0, "no FAMILY_FIELDS bound was actually probed."
        )


class BuildSpecTests(unittest.TestCase):
    def test_default_config_builds_and_validates(self):
        spec = build_spec({})
        self.assertEqual(spec.skin_margin_mm, SPEC_DEFAULTS["skin_margin_mm"])

    def test_bad_gap_mode_raises_spec_error(self):
        with self.assertRaises(SpecError):
            build_spec({"gap_ratio": 0.5, "slot_gap_mm": 0.3})

    def test_skin_margin_below_safety_floor_raises_spec_error(self):
        with self.assertRaises(SpecError):
            build_spec({"skin_margin_mm": 0.5}, wall_width_mm=0.42)

    def test_schema_error_raised_before_cross_field(self):
        with self.assertRaises(ConfigSchemaError):
            build_spec({"unknown_key": 1})

    def test_assert_valid_config_returns_spec(self):
        spec = assert_valid_config({"skin_margin_mm": 1.0})
        self.assertEqual(spec.skin_margin_mm, 1.0)

    def test_mesh_alignment_tolerance_defaults_to_0_02(self):
        spec = build_spec({})
        self.assertEqual(spec.mesh_alignment_tolerance_mm, 0.02)

    def test_mesh_alignment_tolerance_round_trips(self):
        spec = build_spec({"mesh_alignment_tolerance_mm": 0.05})
        self.assertEqual(spec.mesh_alignment_tolerance_mm, 0.05)

    def test_mesh_alignment_tolerance_zero_raises_schema_error(self):
        # Enforced at the schema layer (min_exclusive=0.0), before a
        # StructureSpec is even constructed -- see the raised
        # ConfigSchemaError, not StructureSpec.validate's SpecError.
        with self.assertRaises(ConfigSchemaError):
            build_spec({"mesh_alignment_tolerance_mm": 0.0})

    def test_family_role_round_trips(self):
        spec = build_spec(
            {"families": [{"name": "a", "role": "primary_structure"}]}
        )
        self.assertEqual(spec.families[0].role, "primary_structure")

    def test_family_role_defaults_to_skin_lattice_when_unset(self):
        spec = build_spec({"families": [{"name": "a"}]})
        self.assertEqual(spec.families[0].role, "skin_lattice")

    def test_family_invalid_role_raises_spec_error(self):
        with self.assertRaises(SpecError):
            build_spec({"families": [{"name": "a", "role": "spar"}]})

    def test_family_relative_pitch_and_drift_round_trip(self):
        # task "make member placement geometry-relative instead of absolute" --
        # the two new family keys build and validate through the same path as
        # every other family field.
        spec = build_spec(
            {
                "families": [
                    {
                        "name": "rel",
                        "rib_pitch_chord_frac": 0.1,
                        "z_drift_chord_frac_per_mm": 0.01,
                        "z_period_mm": 40.0,
                    }
                ]
            }
        )
        family = spec.families[0]
        self.assertEqual(family.rib_pitch_chord_frac, 0.1)
        self.assertEqual(family.z_drift_chord_frac_per_mm, 0.01)
        self.assertIsNone(family.rib_pitch_mm)
        self.assertIsNone(family.z_drift_mm_per_mm)

    def test_family_both_pitch_modes_raises_spec_error(self):
        with self.assertRaises(SpecError):
            build_spec(
                {
                    "families": [
                        {"name": "a", "rib_pitch_mm": 8.0, "rib_pitch_chord_frac": 0.1}
                    ]
                }
            )


class TrackedConfigsValidateCleanTests(unittest.TestCase):
    """Both shipped presets must always validate clean -- if either stops
    validating, the schema has drifted from what the plugin actually
    accepts, or the preset itself regressed."""

    def _cap_config(self, name):
        import json

        doc = json.loads((PRESETS_DIR / name).read_text(encoding="utf-8"))
        return doc["config"][0]["cap_config"]

    def test_default_config_validates_clean(self):
        build_spec(self._cap_config("default_config.json"), wall_width_mm=DEFAULT_WALL_MM)

    def test_conservative_config_validates_clean_at_shipped_wall_width(self):
        # The shipped DEFAULT is the two-sided structure (skin_margin_mm
        # 0.8608, the tight 0.0208 mm no-sever cushion) -- see
        # test_default_config_validates_clean above for that pin.
        # conservative_config.json carries the more conservative one-sided
        # margin, 1.0, for anyone who wants it. At 1.0 vs 2 * 0.42 = 0.84 the
        # cushion is 0.16 mm, not 0.0208 mm -- a materially looser margin,
        # pinned here as its own number.
        cap_config = self._cap_config("conservative_config.json")
        self.assertGreater(cap_config["skin_margin_mm"], 2 * 0.42)
        self.assertAlmostEqual(
            cap_config["skin_margin_mm"] - 2 * 0.42, 0.16, places=9
        )
        build_spec(cap_config, wall_width_mm=0.42)

    def test_anisogrid_lattice_config_validates_clean(self):
        # The axial + diagonal skin-lattice config,
        # alongside the two full-depth primary_structure spars. NOT the
        # shipped default -- see presets/plugin/README.md.
        cap_config = self._cap_config("anisogrid_lattice_config.json")
        spec = build_spec(cap_config, wall_width_mm=DEFAULT_WALL_MM)
        roles = {f.name: f.role for f in spec.families}
        self.assertEqual(roles["spar_front"], "primary_structure")
        self.assertEqual(roles["spar_aft"], "primary_structure")
        self.assertEqual(roles["lattice_plus_upper"], "skin_lattice")
        self.assertEqual(roles["lattice_minus_upper"], "skin_lattice")


if __name__ == "__main__":
    unittest.main()
