"""Class-level regression: ``StructureGenerator``'s placement and
``TopologyValidator``'s independent re-check must never disagree.

THE PATTERN THIS GUARDS AGAINST (see ``validate.py``'s module docstring):
"two measurements of the same quantity, where one side was updated and the
other was not" -- different chord stations, the removed region inferred by
double-difference instead of measured, full-depth webs with zero placement
headroom, and the validator's own margin re-check not carrying the rotation
correction the generator has. Each is easy to miss and shows up late, on real
in-host geometry.

This test does not re-derive any of those fixes. It checks agreement from
BOTH directions, because a one-directional check cannot catch this bug class
by itself:

  - ACCEPT side (``GeneratorValidatorRotationAgreementTests.test_*_sweep``):
    builds actual webs with ``StructureGenerator`` (the real placement code)
    across a SWEEP of rotations, margins, and shapes, then feeds each
    produced web straight into ``TopologyValidator`` (the real, independent
    re-check) and asserts it never rejects what the generator just built.
    This alone is NOT sufficient: a validator that is too LENIENT (e.g. the
    bug this file's siblings fix -- the rotation correction silently
    dropped) still accepts everything a conservative generator hands it, so
    this half can pass even while the two sides disagree.
  - REJECT side (``test_*_reject_side_across_rotations``): independently
    constructs a slot that satisfies the RAW, pre-correction margin formula
    exactly -- what a validator with the correction dropped would have
    passed -- and asserts the real validator rejects it. This is the half
    that actually catches a dropped/weakened correction; without the correction in ``validate.py`` this half fails while
    the accept-side half above still passes.

If EITHER side is changed -- generator loosened, validator's correction
dropped, double-applied, or scaled differently than the generator's -- one
of these should turn red, without needing a human to notice the asymmetry
first.
"""
import importlib.util
import math
import unittest
from pathlib import Path

from aircraft_core.frame import (
    DEFAULT_PROBE_WIDTH_MM,
    LayerFrame,
    corrected_skin_margin_mm,
    local_lower_worst_case,
    local_upper_worst_case,
)
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.spec import StructureSpec
from aircraft_core.validate import TopologyError, TopologyValidator
from stub_ops import NotchedPolygon, PolygonShape, Rect, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_agreement", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mf = _load_make_fixtures_module()

NACA = "2412"
CHORD_MM = 60.0
N_POINTS = 120

# Safety offset held small and fixed across the sweep so the erosion/collision
# checks (unrelated to the rotation fix) never become the reason a case passes
# or fails -- only the margin re-check is exercised here.
SAFETY_OFFSET_MM = 0.03

# Rotations up to (not including) the 30-degree envelope threshold --
# 0 exercises the deadzone, the rest exercise the correction itself.
ROTATIONS_DEG = (0.0, 0.5, 2.0, 6.0, 10.0, 20.0, 29.0)
SKIN_MARGINS_MM = (0.3, 1.0, 2.0)


def _naca2412_polygon():
    m, p, t = mf.parse_naca4(NACA)
    _triangles, outline_mm, _bbox = mf.airfoil_triangles(m, p, t, CHORD_MM, CHORD_MM, N_POINTS)
    return PolygonShape(tuple(outline_mm))


def _rotated_naca2412_polygon(washout_deg):
    m, p, t = mf.parse_naca4(NACA)
    outline_unit = mf.naca4_section_outline(m, p, t, N_POINTS)
    points = mf.build_station_outline(
        outline_unit, CHORD_MM, washout_deg, 0.25, 0.0, 0.0, (0.0, 0.0)
    )
    return PolygonShape(tuple(points))


def _rotate_points(points, theta_deg, pivot):
    theta = math.radians(theta_deg)
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    px, py = pivot
    out = []
    for x, y in points:
        dx, dy = x - px, y - py
        out.append((px + dx * cos_t - dy * sin_t, py + dx * sin_t + dy * cos_t))
    return out


def _rotated_coupon_polygon(washout_deg):
    """An elongated rectangular coupon (30 x 4 mm, chord along X), rotated
    about its own centre -- the coupon analogue of the NACA twist fixture,
    built as a plain PolygonShape so the same rotated-geometry code path
    (frame.estimate_rotation_deg, NotchedPolygon) exercises a shape with no
    camber at all."""
    base = [(0.0, 0.0), (30.0, 0.0), (30.0, 4.0), (0.0, 4.0)]
    rotated = _rotate_points(base, washout_deg, pivot=(15.0, 2.0))
    return PolygonShape(tuple(rotated))


def _build_and_validate_all_intents(shape_name, polygon, rotation_deg, skin_margin_mm):
    """Runs the real generator on ``polygon``, then feeds every produced
    web straight to the real validator. Returns the number of webs checked
    (0 if every candidate was omitted -- not a failure, just nothing to
    cross-check at this combination)."""
    ops = StubOps()
    frame = LayerFrame.from_polygon(ops, polygon)
    outer_wall_mm = 0.42
    spec = StructureSpec(
        gap_ratio=1.0,
        skin_margin_mm=skin_margin_mm,
        web_position=0.5,
    )
    result = StructureGenerator().generate(
        frame,
        spec,
        outer_wall_mm,
        LayerContext(layer_index=0, print_z=0.0),
        safety_offset_mm=SAFETY_OFFSET_MM,
    )
    if not result.intents:
        return 0

    validator = TopologyValidator(ops, safety_offset_mm=SAFETY_OFFSET_MM)
    checked = 0
    for intent in result.intents:
        x = intent.start[0]
        half_w = intent.width_mm / 2.0
        y_top = max(intent.start[1], intent.end[1])
        y_bottom = min(intent.start[1], intent.end[1])
        notch = Rect(x - half_w, y_bottom, x + half_w, y_top)
        transformed = NotchedPolygon(outer=polygon, notch=notch)
        # Must not raise -- the generator's own placement must survive the
        # validator's independent, rotation-corrected re-check.
        validator.validate(
            [polygon],
            [transformed],
            [notch],
            spec,
            layer_index=0,
            print_z=0.0,
        )
        checked += 1
    return checked


def _assert_raw_margin_only_slot_is_rejected(test_case, polygon, rotation_deg):
    """THE REJECT-SIDE HALF of the agreement this file guards.

    The accept-side check above (every web the generator actually builds must
    validate) cannot, on its own, catch a validator whose rotation correction
    was dropped or never applied: a TOO LENIENT validator still accepts
    everything the (correctly, conservatively) placed generator output hands it
    -- the bug the validator's rotation correction prevents (see validate.py's
    module docstring). This function is the other half: it builds a
    slot that satisfies the RAW, uncorrected margin formula exactly (what a
    pre-fix validator would have passed) and asserts the validator rejects it
    -- proof the correction is actually being applied, not merely that nothing
    downstream complained.

    Only meaningful strictly above the rotation deadzone (raw and corrected
    margins are identical inside it by design -- see
    frame._ROTATION_DEADZONE_DEG) -- callers must not invoke this at
    ``rotation_deg`` inside the deadzone.
    """
    ops = StubOps()
    frame = LayerFrame.from_polygon(ops, polygon)
    outer_wall_mm = 0.42
    slot_width_mm = 1.0 * outer_wall_mm
    x = frame.web_x(0.5)

    y_max = local_upper_worst_case(ops, polygon, x, slot_width_mm, DEFAULT_PROBE_WIDTH_MM)
    y_min = local_lower_worst_case(ops, polygon, x, slot_width_mm, DEFAULT_PROBE_WIDTH_MM)
    raw_thickness = y_max - y_min

    theta = math.radians(frame.rotation_deg)
    cos_theta = math.cos(theta)
    # Strictly between the corrected-safe thickness (raw_thickness *
    # cos_theta) and raw_thickness itself: satisfies the OLD raw check
    # exactly by construction, fails the corrected one.
    skin_margin_mm = raw_thickness * (cos_theta + 1.0) / 2.0
    test_case.assertLess(skin_margin_mm, raw_thickness)
    test_case.assertGreater(skin_margin_mm / cos_theta, raw_thickness)

    half_w = slot_width_mm / 2.0
    notch_end_y = y_min + skin_margin_mm  # exactly satisfies the RAW formula
    notch = Rect(x - half_w, notch_end_y, x + half_w, y_max)
    transformed = NotchedPolygon(outer=polygon, notch=notch)

    spec = StructureSpec(gap_ratio=1.0, skin_margin_mm=skin_margin_mm, web_position=0.5)
    validator = TopologyValidator(ops, safety_offset_mm=0.01)

    with test_case.assertRaises(TopologyError):
        validator.validate([polygon], [transformed], [notch], spec, layer_index=0, print_z=0.0)


class GeneratorValidatorRotationAgreementTests(unittest.TestCase):
    """Sweeps rotation x margin x shape; every produced web must validate."""

    def test_naca2412_sweep(self):
        total_checked = 0
        for rotation_deg in ROTATIONS_DEG:
            polygon = (
                _naca2412_polygon()
                if rotation_deg == 0.0
                else _rotated_naca2412_polygon(rotation_deg)
            )
            for skin_margin_mm in SKIN_MARGINS_MM:
                with self.subTest(
                    shape="naca2412", rotation_deg=rotation_deg, skin_margin_mm=skin_margin_mm
                ):
                    total_checked += _build_and_validate_all_intents(
                        "naca2412", polygon, rotation_deg, skin_margin_mm
                    )
        # Sanity: the sweep must actually exercise SOME webs, or this test
        # would pass vacuously and catch nothing.
        self.assertGreater(total_checked, 0)

    def test_rectangular_coupon_sweep(self):
        total_checked = 0
        for rotation_deg in ROTATIONS_DEG:
            polygon = _rotated_coupon_polygon(rotation_deg)
            for skin_margin_mm in SKIN_MARGINS_MM:
                with self.subTest(
                    shape="coupon", rotation_deg=rotation_deg, skin_margin_mm=skin_margin_mm
                ):
                    total_checked += _build_and_validate_all_intents(
                        "coupon", polygon, rotation_deg, skin_margin_mm
                    )
        self.assertGreater(total_checked, 0)

    def test_naca2412_reject_side_across_rotations(self):
        # Above-deadzone rotations only -- see
        # _assert_raw_margin_only_slot_is_rejected's docstring for why.
        above_deadzone = [r for r in ROTATIONS_DEG if r >= 1.0]
        self.assertGreater(len(above_deadzone), 0)
        for rotation_deg in above_deadzone:
            with self.subTest(shape="naca2412", rotation_deg=rotation_deg):
                polygon = _rotated_naca2412_polygon(rotation_deg)
                _assert_raw_margin_only_slot_is_rejected(self, polygon, rotation_deg)

    def test_rectangular_coupon_reject_side_across_rotations(self):
        above_deadzone = [r for r in ROTATIONS_DEG if r >= 1.0]
        for rotation_deg in above_deadzone:
            with self.subTest(shape="coupon", rotation_deg=rotation_deg):
                polygon = _rotated_coupon_polygon(rotation_deg)
                _assert_raw_margin_only_slot_is_rejected(self, polygon, rotation_deg)


if __name__ == "__main__":
    unittest.main()
