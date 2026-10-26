import unittest

from aircraft_core.spec import (
    ROLE_PRIMARY_STRUCTURE,
    ROLE_SKIN_LATTICE,
    SpecError,
    StructureSpec,
    WebFamily,
)


class SpecValidationTests(unittest.TestCase):
    def test_both_gap_modes_set_is_error(self):
        spec = StructureSpec(slot_gap_mm=0.6, gap_ratio=1.5)
        with self.assertRaises(SpecError) as ctx:
            spec.validate()
        self.assertIn("both slot_gap_mm", str(ctx.exception))

    def test_neither_gap_mode_set_is_error(self):
        spec = StructureSpec(slot_gap_mm=None, gap_ratio=None)
        with self.assertRaises(SpecError) as ctx:
            spec.validate()
        self.assertIn("neither slot_gap_mm nor gap_ratio", str(ctx.exception))

    def test_negative_skin_margin_is_error(self):
        spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=-0.1)
        with self.assertRaises(SpecError):
            spec.validate()

    def test_web_position_out_of_range_is_error(self):
        spec = StructureSpec(slot_gap_mm=0.6, web_position=1.5)
        with self.assertRaises(SpecError):
            spec.validate()

    def test_zero_overshoot_is_error(self):
        spec = StructureSpec(slot_gap_mm=0.6, overshoot_mm=0.0)
        with self.assertRaises(SpecError):
            spec.validate()

    def test_valid_explicit_gap_passes(self):
        StructureSpec(slot_gap_mm=0.6).validate()

    def test_valid_ratio_gap_passes(self):
        StructureSpec(gap_ratio=1.5).validate()

    def test_negative_web_depth_is_error(self):
        spec = StructureSpec(slot_gap_mm=0.6, web_depth_mm=-1.0)
        with self.assertRaises(SpecError):
            spec.validate()

    def test_zero_web_depth_is_error(self):
        spec = StructureSpec(slot_gap_mm=0.6, web_depth_mm=0.0)
        with self.assertRaises(SpecError):
            spec.validate()

    def test_none_web_depth_passes(self):
        StructureSpec(slot_gap_mm=0.6, web_depth_mm=None).validate()

    def test_positive_web_depth_passes(self):
        StructureSpec(slot_gap_mm=0.6, web_depth_mm=1.5).validate()

    def test_mesh_alignment_tolerance_default_is_0_02(self):
        spec = StructureSpec(slot_gap_mm=0.6)
        self.assertEqual(spec.mesh_alignment_tolerance_mm, 0.02)
        spec.validate()

    def test_zero_mesh_alignment_tolerance_is_error(self):
        spec = StructureSpec(slot_gap_mm=0.6, mesh_alignment_tolerance_mm=0.0)
        with self.assertRaises(SpecError):
            spec.validate()

    def test_negative_mesh_alignment_tolerance_is_error(self):
        spec = StructureSpec(slot_gap_mm=0.6, mesh_alignment_tolerance_mm=-0.01)
        with self.assertRaises(SpecError):
            spec.validate()

    def test_positive_mesh_alignment_tolerance_passes(self):
        StructureSpec(slot_gap_mm=0.6, mesh_alignment_tolerance_mm=0.05).validate()


class WebFamilyValidationTests(unittest.TestCase):
    def test_both_gap_modes_on_family_is_error(self):
        family = WebFamily(name="bad", slot_gap_mm=0.6, gap_ratio=1.5)
        spec = StructureSpec(slot_gap_mm=0.6, families=(family,))
        with self.assertRaises(SpecError) as ctx:
            spec.validate()
        message = str(ctx.exception)
        self.assertIn("family 0", message)
        self.assertIn("'bad'", message)

    def test_family_gap_modes_may_both_be_none(self):
        family = WebFamily(name="inherits")
        StructureSpec(slot_gap_mm=0.6, families=(family,)).validate()

    def test_negative_family_rib_pitch_is_error(self):
        family = WebFamily(name="bad", rib_pitch_mm=-1.0)
        spec = StructureSpec(slot_gap_mm=0.6, families=(family,))
        with self.assertRaises(SpecError):
            spec.validate()

    def test_negative_family_web_depth_is_error(self):
        family = WebFamily(name="bad", web_depth_mm=-1.0)
        spec = StructureSpec(slot_gap_mm=0.6, families=(family,))
        with self.assertRaises(SpecError):
            spec.validate()

    def test_family_web_position_out_of_range_is_error(self):
        family = WebFamily(name="bad", web_position=1.5)
        spec = StructureSpec(slot_gap_mm=0.6, families=(family,))
        with self.assertRaises(SpecError):
            spec.validate()

    def test_second_family_error_names_its_own_index(self):
        good = WebFamily(name="good")
        bad = WebFamily(name="bad", rib_pitch_mm=0.0)
        spec = StructureSpec(slot_gap_mm=0.6, families=(good, bad))
        with self.assertRaises(SpecError) as ctx:
            spec.validate()
        self.assertIn("family 1", str(ctx.exception))

    def test_family_resolve_gap_inherits_spec_when_unset(self):
        family = WebFamily(name="inherits")
        spec = StructureSpec(gap_ratio=1.5)
        self.assertAlmostEqual(family.resolve_gap(spec, 0.42), 0.63)

    def test_family_resolve_gap_uses_own_slot_gap_mm(self):
        family = WebFamily(name="own", slot_gap_mm=0.2)
        spec = StructureSpec(gap_ratio=1.5)
        self.assertEqual(family.resolve_gap(spec, 0.42), 0.2)

    def test_family_resolve_gap_uses_own_gap_ratio(self):
        family = WebFamily(name="own", gap_ratio=0.5)
        spec = StructureSpec(gap_ratio=1.5)
        self.assertAlmostEqual(family.resolve_gap(spec, 0.42), 0.21)

    # -- role (roles as first-class) -------------

    def test_role_defaults_to_skin_lattice(self):
        family = WebFamily(name="untagged")
        self.assertEqual(family.role, ROLE_SKIN_LATTICE)

    def test_role_may_be_set_explicitly_to_primary_structure(self):
        family = WebFamily(name="spar", role=ROLE_PRIMARY_STRUCTURE)
        spec = StructureSpec(slot_gap_mm=0.6, families=(family,))
        spec.validate()  # does not raise
        self.assertEqual(family.role, ROLE_PRIMARY_STRUCTURE)

    def test_role_is_not_inferred_from_web_depth(self):
        # A full-depth family (web_depth_mm=None) does NOT silently become
        # primary_structure -- role is explicit, never inferred from depth
        # (depth and role are independent).
        full_depth = WebFamily(name="full", web_depth_mm=None)
        shallow = WebFamily(name="shallow", web_depth_mm=1.5)
        self.assertEqual(full_depth.role, ROLE_SKIN_LATTICE)
        self.assertEqual(shallow.role, ROLE_SKIN_LATTICE)

    def test_invalid_role_is_error(self):
        family = WebFamily(name="bad", role="spar")
        spec = StructureSpec(slot_gap_mm=0.6, families=(family,))
        with self.assertRaises(SpecError) as ctx:
            spec.validate()
        message = str(ctx.exception)
        self.assertIn("family 0", message)
        self.assertIn("role", message)

    def test_role_does_not_change_geometry(self):
        # Two otherwise-identical families differing only in role produce
        # the same effective_families() field values apart from role.
        skin = WebFamily(name="a", web_position=0.3, role=ROLE_SKIN_LATTICE)
        primary = WebFamily(name="a", web_position=0.3, role=ROLE_PRIMARY_STRUCTURE)
        self.assertEqual(skin.web_position, primary.web_position)
        self.assertEqual(skin.rib_pitch_mm, primary.rib_pitch_mm)
        self.assertEqual(skin.web_depth_mm, primary.web_depth_mm)


class EffectiveFamiliesTests(unittest.TestCase):
    def test_no_families_yields_one_synthetic_family_from_top_level_fields(self):
        spec = StructureSpec(
            gap_ratio=1.5,
            web_position=0.7,
            rib_pitch_mm=10.0,
            web_depth_mm=2.0,
        )
        families = spec.effective_families()
        self.assertEqual(len(families), 1)
        family = families[0]
        self.assertEqual(family.web_position, 0.7)
        self.assertEqual(family.rib_pitch_mm, 10.0)
        self.assertEqual(family.web_depth_mm, 2.0)
        # Gap fields left unset so resolve_gap falls back to the spec's own
        # gap mode -- see effective_families' own docstring.
        self.assertIsNone(family.slot_gap_mm)
        self.assertIsNone(family.gap_ratio)
        self.assertAlmostEqual(family.resolve_gap(spec, 0.42), 0.63)

    def test_families_given_are_returned_verbatim(self):
        family_a = WebFamily(name="a")
        family_b = WebFamily(name="b")
        spec = StructureSpec(slot_gap_mm=0.6, families=(family_a, family_b))
        self.assertEqual(spec.effective_families(), (family_a, family_b))


class ResolveSlotGapTests(unittest.TestCase):
    def test_resolve_from_explicit_value_ignores_line_width(self):
        spec = StructureSpec(slot_gap_mm=0.6)
        self.assertEqual(spec.resolve_slot_gap(0.42), 0.6)
        self.assertEqual(spec.resolve_slot_gap(999.0), 0.6)

    def test_resolve_from_ratio(self):
        spec = StructureSpec(gap_ratio=1.5)
        self.assertAlmostEqual(spec.resolve_slot_gap(0.42), 0.63)

    def test_resolve_from_ratio_requires_positive_line_width(self):
        spec = StructureSpec(gap_ratio=1.5)
        with self.assertRaises(SpecError):
            spec.resolve_slot_gap(0.0)


if __name__ == "__main__":
    unittest.main()
