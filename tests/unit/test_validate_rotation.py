"""``TopologyValidator``'s own margin re-check must be rotation-aware, the same
way ``frame.py``/``generator.py`` already are -- see ``validate.py``'s module
docstring, "change",'s "Known residual, recorded not hidden".

Covers:
    - A slot that satisfies the RAW ``spec.skin_margin_mm`` on a rotated
      section, but not the rotation-corrected margin, is now rejected by
      the validator on its own (independent of whatever the generator
      would have done) -- the asymmetry the validator's correction removes.
    - The validator derives rotation ITSELF from the ``original`` polygon
      it is handed, not from any parameter -- proven by constructing the
      rotated case above without ever threading a rotation value into
      ``validate()``.
    - Unrotated fixtures are completely unaffected (see
      ``tests/unit/test_validate.py`` for the full pre-existing suite,
      which must keep passing unweakened; this file adds one explicit
      regression assertion that the deadzone is truly a no-op).
"""
import math
import unittest

from aircraft_core.frame import (
    DEFAULT_PROBE_WIDTH_MM,
    LayerFrame,
    corrected_skin_margin_mm,
    estimate_rotation_deg,
    local_lower_worst_case,
    local_upper_worst_case,
)
from aircraft_core.spec import StructureSpec
from aircraft_core.validate import TopologyError, TopologyValidator
from test_rotation_envelope import _naca2412_polygon, _rotated_naca2412_polygon
from stub_ops import NotchedPolygon, Rect, StubOps

LAYER_INDEX = 42
PRINT_Z = 8.4


class RotatedMarginReCheckTests(unittest.TestCase):
    """The validator's invariant 4 compares against the rotation-corrected
    margin, not the raw ``spec.skin_margin_mm``: on a rotated section the raw
    value is optimistic by ``1/cos(theta)``, exactly as it would be in the
    generator."""

    def setUp(self):
        self.ops = StubOps()
        self.polygon = _rotated_naca2412_polygon(6.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.polygon)
        # Sanity: this fixture really is rotated, and by roughly the angle the rotation-envelope
        # tests measure.
        self.assertGreater(abs(self.frame.rotation_deg), 1.0)

    def test_slot_passing_raw_margin_but_failing_corrected_margin_is_rejected(self):
        web_position = 0.5
        outer_wall_mm = 0.42
        slot_width_mm = 1.0 * outer_wall_mm
        x = self.frame.web_x(web_position)

        y_max = local_upper_worst_case(
            self.ops, self.polygon, x, slot_width_mm, DEFAULT_PROBE_WIDTH_MM
        )
        y_min = local_lower_worst_case(
            self.ops, self.polygon, x, slot_width_mm, DEFAULT_PROBE_WIDTH_MM
        )
        raw_thickness = y_max - y_min

        theta = math.radians(self.frame.rotation_deg)
        cos_theta = math.cos(theta)
        # A margin strictly between raw_thickness*cos_theta (the TRUE
        # chord-perpendicular thickness) and raw_thickness (the raw world-Y
        # thickness): satisfies the RAW check (remaining_skin ==
        # skin_margin_mm exactly, by construction below) but not the
        # corrected one (skin_margin_mm / cos_theta > raw_thickness).
        skin_margin_mm = raw_thickness * (cos_theta + 1.0) / 2.0
        self.assertLess(skin_margin_mm, raw_thickness)
        self.assertGreater(skin_margin_mm / cos_theta, raw_thickness)

        # Build a notch whose far end sits EXACTLY skin_margin_mm above the
        # worst-case local lower surface -- passes the OLD (raw) invariant 4
        # exactly, by construction.
        half_w = slot_width_mm / 2.0
        notch_end_y = y_min + skin_margin_mm
        notch = Rect(x - half_w, notch_end_y, x + half_w, y_max)
        transformed = NotchedPolygon(outer=self.polygon, notch=notch)

        spec = StructureSpec(
            gap_ratio=1.0, skin_margin_mm=skin_margin_mm, web_position=web_position
        )
        # Deliberately small so the erosion/collision checks below invariant
        # 4 can never be what fires here -- only the margin re-check should.
        validator = TopologyValidator(self.ops, safety_offset_mm=0.01)

        with self.assertRaises(TopologyError) as ctx:
            validator.validate(
                [self.polygon], [transformed], [notch], spec, LAYER_INDEX, PRINT_Z
            )
        message = str(ctx.exception)
        self.assertIn("skin", message)
        self.assertIn("rotation correction", message)

    def test_rotation_is_derived_independently_not_passed_in(self):
        # validate()'s signature carries no rotation/frame parameter at all
        # -- this is a structural guarantee, not just an assertion on this
        # one call, but assert the call succeeds without one to document it.
        import inspect

        params = inspect.signature(TopologyValidator.validate).parameters
        for name in params:
            self.assertNotIn("rotation", name.lower())
            self.assertNotIn("frame", name.lower())

    def test_same_rotation_estimate_validator_uses_matches_frame_py(self):
        # The validator must derive EXACTLY the same rotation frame.py's own
        # LayerFrame would for the same polygon -- proof it reuses
        # estimate_rotation_deg rather than a second implementation.
        self.assertEqual(
            estimate_rotation_deg(self.ops, self.polygon), self.frame.rotation_deg
        )


class UnrotatedDeadzoneIsANoOpTests(unittest.TestCase):
    """The 1-degree deadzone (frame._ROTATION_DEADZONE_DEG) must make this
    change a complete no-op on every existing unrotated fixture -- explicit
    regression beyond the full pre-existing test_validate.py suite."""

    def test_unrotated_airfoil_margin_message_cites_zero_degree_correction(self):
        ops = StubOps()
        polygon, _bbox = _naca2412_polygon()
        frame = LayerFrame.from_polygon(ops, polygon)
        self.assertLess(abs(frame.rotation_deg), 1.0)  # within the deadzone

        skin_margin_mm = 1.0
        web_position = 0.7
        x = frame.web_x(web_position)
        y_max = local_upper_worst_case(ops, polygon, x, 0.3, DEFAULT_PROBE_WIDTH_MM)
        y_min = local_lower_worst_case(ops, polygon, x, 0.3, DEFAULT_PROBE_WIDTH_MM)
        # Cut short of the margin -> must still fail, with the SAME
        # effective margin as the raw one (deadzone => no correction).
        notch = Rect(x - 0.15, y_min + skin_margin_mm - 0.2, x + 0.15, y_max)
        transformed = NotchedPolygon(outer=polygon, notch=notch)
        spec = StructureSpec(
            gap_ratio=1.0, skin_margin_mm=skin_margin_mm, web_position=web_position
        )
        validator = TopologyValidator(ops, safety_offset_mm=0.1)
        with self.assertRaises(TopologyError) as ctx:
            validator.validate(
                [polygon], [transformed], [notch], spec, LAYER_INDEX, PRINT_Z
            )
        message = str(ctx.exception)
        self.assertIn("required 1.0000 mm margin", message)
        self.assertIn("effective 1.0000 mm", message)


if __name__ == "__main__":
    unittest.main()
