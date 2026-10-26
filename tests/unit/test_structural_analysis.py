"""Tests for ``tools/structural_analysis.py``.

Loaded via ``importlib``, the same pattern every other test in this
directory already uses for ``tools/make_fixtures.py`` (see
``test_local_geometry.py``, ``test_generator.py``) -- no second copy of the
tool's own code lives here.

The centrepiece is a HAND-CHECKABLE validation of ``solve_multicell_j`` against
a rectangular box section, where the "symmetric limit" paper 1 (Sec 3.2)
describes -- "equal cells, uniform skin... the web carries zero shear flow" --
has a closed-form answer this test derives independently in its own docstring,
not merely by trusting the implementation under test.
"""

import importlib.util
import math
import os
import sys
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_STRUCTURAL_ANALYSIS_PATH = os.path.join(REPO_ROOT, "tools", "structural_analysis.py")


def _load_structural_analysis():
    spec = importlib.util.spec_from_file_location(
        "structural_analysis_under_test", _STRUCTURAL_ANALYSIS_PATH
    )
    module = importlib.util.module_from_spec(spec)
    # Registered in sys.modules BEFORE exec_module: the tool uses
    # dataclasses together with `from __future__ import annotations`, and
    # Python 3.11's dataclass machinery resolves string annotations via
    # `sys.modules[cls.__module__]` -- without this, exec_module raises
    # AttributeError deep inside the stdlib dataclasses module.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


sa = _load_structural_analysis()


class RectSectionGeometryTests(unittest.TestCase):
    """Sanity checks on the rectangle SectionGeometry builder itself --
    these numbers must be right before anything built on top of them can be
    trusted."""

    def setUp(self):
        self.w, self.h = 20.0, 10.0
        self.section = sa.build_rect_section(self.w, self.h, span_mm=60.0)

    def test_area_and_perimeter(self):
        self.assertAlmostEqual(self.section.area_mm2, self.w * self.h)
        self.assertAlmostEqual(self.section.perimeter_mm, 2.0 * (self.w + self.h))

    def test_thickness_is_constant_everywhere_including_the_ends(self):
        for x in (self.section.min_x, 0.0, self.section.max_x):
            self.assertAlmostEqual(self.section.thickness_at(x), self.h)

    def test_area_between_is_linear_in_x(self):
        x0, x1 = self.section.min_x, self.section.min_x + self.w / 2.0
        self.assertAlmostEqual(self.section.area_between(x0, x1), (self.w / 2.0) * self.h)

    def test_arc_upper_and_lower_track_x_directly(self):
        x = self.section.min_x + 3.0
        self.assertAlmostEqual(self.section.arc_upper(x), 3.0)
        self.assertAlmostEqual(self.section.arc_lower(x), 3.0)


class OpenCellBredtBathoTests(unittest.TestCase):
    """``j_open_cell`` against the textbook closed-form Bredt-Batho value,
    on both a hand-computable rectangle and the real NACA 2412 fixture,
    cross-checked against the exact figure
 Sec 3.1 already
    published for the SAME fixture ("with zero webs the solve returns
    J = 1201.4641710739309, against the closed-form Bredt-Batho
    4A^2/(P/t) = 1201.4641710739306").
    """

    def test_rectangle_hand_value(self):
        w, h, t = 20.0, 10.0, 0.42
        section = sa.build_rect_section(w, h, span_mm=60.0)
        a = w * h
        p = 2.0 * (w + h)
        expected = 4.0 * a ** 2 * t / p
        self.assertAlmostEqual(sa.j_open_cell(section, t), expected, places=9)
        # Also independently by hand: A=200, P=60, t=0.42 ->
        # J = 4*200^2*0.42/60 = 4*40000*0.42/60 = 67200/60 = 1120.0
        self.assertAlmostEqual(expected, 1120.0, places=6)

    def test_matches_the_projects_own_published_airfoil_figure(self):
        section = sa.build_airfoil_section(naca="2412", chord_mm=60.0, span_mm=60.0, n_points=120)
        t = 0.42
        j = sa.j_open_cell(section, t)
        # Paper 1's own published figure for this exact fixture is
        # 1201.4641710739309 mm^4 (A=296.2, P=122.6, per that paper's own
        # rounding). This tool's profile table uses a DIFFERENT, documented
        # approximation from the true perpendicular-offset NACA construction
        # (see SectionGeometry's docstring: it evaluates surface points at
        # raw chord fraction rather than the true offset X), so agreement is
        # expected to within about 2%, not to machine precision -- the
        # published figure used the true offset geometry, this tool's J_open
        # deliberately does not, for the reasons stated there.
        self.assertAlmostEqual(j, 1201.46, delta=0.02 * 1201.46)
        self.assertAlmostEqual(section.area_mm2, 296.2, delta=0.01 * 296.2)
        self.assertAlmostEqual(section.perimeter_mm, 122.6, delta=0.01 * 122.6)


class MulticellSymmetricLimitTests(unittest.TestCase):
    """THE hand-derivable validation case: a single web at the exact centre
    of a symmetric rectangle must contribute EXACTLY ZERO to J, matching
    paper 1's own closed-form claim (Sec 3.2): "In the symmetric limit --
    equal cells, uniform skin -- q1 = q2 EXACTLY, the web carries zero shear
    flow, and it contributes exactly nothing to J."

    Independent hand derivation (see this class's own docstring math,
    reproduced from the general 2-cell equations, not from the
    implementation under test): for a rectangle width W, height H, split by
    one centred full-depth web,

        S_0 = S_1 = (W + H) / t_skin  (each half-box's own outer arc, incl.
                                        its end wall of length H)
        edge = H / t_web              (the shared central web)

    By symmetry A_0 = A_1 and S_0 = S_1, so the compatibility equations force
    q_0 = q_1 (the coefficient of (q_0 - q_1) times a strictly positive term
    must be zero). Substituting q_0 = q_1 = q into the torque equation and
    then into either compatibility equation eliminates ``edge`` entirely --
    a web can be arbitrarily thin or arbitrarily thick and it changes NOTHING
    -- and reduces algebraically to the SAME 4A^2/(P/t) formula the whole
    (unsplit) rectangle would give. This holds regardless of t_web, which
    this test asserts directly by sweeping several very different t_web
    values and confirming J_closed is unchanged and equals J_open.
    """

    def test_centred_web_adds_nothing_regardless_of_web_thickness(self):
        w, h, t_skin = 20.0, 10.0, 0.42
        section = sa.build_rect_section(w, h, span_mm=60.0)
        j_open = sa.j_open_cell(section, t_skin)
        center_x = (section.min_x + section.max_x) / 2.0

        for t_web in (0.05, 0.42, 0.861, 5.0, 50.0):
            j_closed = sa.solve_multicell_j(section, [center_x], t_skin, t_web)
            self.assertAlmostEqual(
                j_closed,
                j_open,
                places=6,
                msg="a symmetric centred web with t_web={!r} must add exactly "
                "zero to J (paper 1's symmetric-limit claim), got J_closed="
                "{!r} vs J_open={!r}".format(t_web, j_closed, j_open),
            )

    def test_off_center_web_strictly_increases_j(self):
        # Off-centre breaks the symmetry that forces q0 == q1; the standard
        # multi-cell result (paper 1 Sec 3.2, "an interior web participates
        # in torsion only to the extent that the cells it separates want
        # DIFFERENT shear flows") predicts a strictly positive gain here,
        # not zero.
        w, h, t_skin, t_web = 20.0, 10.0, 0.42, 0.861
        section = sa.build_rect_section(w, h, span_mm=60.0)
        j_open = sa.j_open_cell(section, t_skin)
        off_center_x = section.min_x + 0.3 * w  # well off the centreline
        j_closed = sa.solve_multicell_j(section, [off_center_x], t_skin, t_web)
        self.assertGreater(j_closed, j_open)

    def test_no_webs_reduces_to_open_cell_exactly(self):
        w, h, t_skin = 20.0, 10.0, 0.42
        section = sa.build_rect_section(w, h, span_mm=60.0)
        # solve_multicell_j with an empty station list still must build a
        # single cell spanning the whole section and match j_open_cell
        # exactly (n == 1 branch).
        j_closed = sa.solve_multicell_j(section, [], t_skin, 0.861)
        j_open = sa.j_open_cell(section, t_skin)
        self.assertAlmostEqual(j_closed, j_open, places=9)

    def test_two_asymmetric_webs_hand_checked_three_cells(self):
        # A fully independent hand check with THREE cells (two webs), unequal
        # spacing, built directly from the general multi-cell equations
        # rather than by trusting solve_multicell_j's own bookkeeping.
        # Rectangle W=12, H=4, t_skin=1.0, t_web=1.0 (nice round numbers).
        # Webs at x = -2 and x = +3 (min_x=-6, max_x=+6).
        w, h, t = 12.0, 4.0, 1.0
        section = sa.build_rect_section(w, h, span_mm=60.0)
        x1, x2 = -2.0, 3.0

        # Cell 0: [-6, -2], width 4 -> area 16, outer arc = 4(top)+4(bottom)+4(end wall)=12
        # Cell 1: [-2, 3], width 5 -> area 20, outer arc = 5+5 = 10 (no end wall, interior)
        # Cell 2: [3, 6], width 3 -> area 12, outer arc = 3+3+4(end wall) = 10
        a0, a1, a2 = 16.0, 20.0, 12.0
        s0_outer, s1_outer, s2_outer = 12.0, 10.0, 10.0
        edge01 = h / t  # = 4.0
        edge12 = h / t  # = 4.0

        s0 = s0_outer + edge01
        s1 = s1_outer + edge01 + edge12
        s2 = s2_outer + edge12

        # Solve the 4x4 system [q0,q1,q2,Omega] by hand via substitution,
        # done here with plain Python arithmetic (NOT calling
        # solve_multicell_j or its internal _solve_linear), as an
        # independent check.
        # Equations:
        #  q0*s0 - q1*edge01 - 2*a0*Omega = 0
        #  q1*s1 - q0*edge01 - q2*edge12 - 2*a1*Omega = 0
        #  q2*s2 - q1*edge12 - 2*a2*Omega = 0
        #  2*a0*q0 + 2*a1*q1 + 2*a2*q2 = 1
        # Express q0, q2 in terms of q1 and Omega from eq1/eq3, substitute
        # into eq2 and the torque equation, solve the resulting 2x2 system
        # for q1 and Omega.
        # q0 = (q1*edge01 + 2*a0*Omega) / s0
        # q2 = (q1*edge12 + 2*a2*Omega) / s2
        # eq2: q1*s1 - edge01*q0 - edge12*q2 - 2*a1*Omega = 0
        #   => q1*s1 - edge01*(q1*edge01 + 2*a0*Omega)/s0
        #             - edge12*(q1*edge12 + 2*a2*Omega)/s2 - 2*a1*Omega = 0
        #   => q1*(s1 - edge01^2/s0 - edge12^2/s2)
        #      - Omega*(2*a0*edge01/s0 + 2*a2*edge12/s2 + 2*a1) = 0
        coeff_q1 = s1 - edge01 ** 2 / s0 - edge12 ** 2 / s2
        coeff_omega = 2 * a0 * edge01 / s0 + 2 * a2 * edge12 / s2 + 2 * a1
        # => q1 = Omega * coeff_omega / coeff_q1
        ratio = coeff_omega / coeff_q1  # q1 = ratio * Omega

        # torque: 2*a0*q0 + 2*a1*q1 + 2*a2*q2 = 1
        # q0 = (ratio*Omega*edge01 + 2*a0*Omega)/s0 = Omega*(ratio*edge01 + 2*a0)/s0
        # q2 = Omega*(ratio*edge12 + 2*a2)/s2
        q0_coeff = (ratio * edge01 + 2 * a0) / s0
        q2_coeff = (ratio * edge12 + 2 * a2) / s2
        # 2*a0*q0_coeff*Omega + 2*a1*ratio*Omega + 2*a2*q2_coeff*Omega = 1
        omega_coeff_total = 2 * a0 * q0_coeff + 2 * a1 * ratio + 2 * a2 * q2_coeff
        omega = 1.0 / omega_coeff_total
        j_hand = 1.0 / omega

        j_tool = sa.solve_multicell_j(section, [x1, x2], t, t)
        self.assertAlmostEqual(j_tool, j_hand, places=6)
        # And it must exceed the open-cell value (webs add real stiffness
        # here, since cells are unequal).
        j_open = sa.j_open_cell(section, t)
        self.assertGreater(j_tool, j_open)


class BucklingFormulaTests(unittest.TestCase):
    def test_shear_over_compression_ratio_matches_naca_tn3781_coefficients(self):
        t, b, e_mpa, nu = 0.42, 7.6, 3500.0, 0.35
        sigma = sa.flat_plate_sigma_cr_mpa(t, b, e_mpa, nu)
        tau = sa.flat_plate_tau_cr_mpa(sigma)
        self.assertAlmostEqual(tau / sigma, sa.K_S_SS / sa.K_C_SS, places=9)

    def test_sigma_cr_scales_as_inverse_b_squared(self):
        t, e_mpa, nu = 0.42, 3500.0, 0.35
        sigma_wide = sa.flat_plate_sigma_cr_mpa(t, 10.0, e_mpa, nu)
        sigma_narrow = sa.flat_plate_sigma_cr_mpa(t, 5.0, e_mpa, nu)
        self.assertAlmostEqual(sigma_narrow / sigma_wide, 4.0, places=6)

    def test_batdorf_factor_is_unity_when_flat_or_unknown_radius(self):
        self.assertEqual(sa.batdorf_curvature_factor(7.6, None, 0.42, 0.35), 1.0)
        self.assertEqual(sa.batdorf_curvature_factor(7.6, 100000.0, 0.42, 0.35), 1.0)

    def test_batdorf_factor_exceeds_unity_in_the_sharply_curved_regime(self):
        # b^2/(r*t) > 30 required; pick a small radius to trigger it.
        b, t, r, nu = 7.6, 0.42, 3.5, 0.35
        self.assertGreater(b ** 2 / (r * t), 30.0)
        factor = sa.batdorf_curvature_factor(b, r, t, nu)
        self.assertGreater(factor, 1.0)

    def test_buckling_torque_hand_value(self):
        area, t, tau = 100.0, 0.5, 2.0
        # T_cr = 2*A*t*tau, N.mm -> N.m: 2*100*0.5*2 = 200 N.mm = 0.2 N.m
        self.assertAlmostEqual(sa.buckling_torque_n_m(area, t, tau), 0.2, places=9)


class SpecFromCapConfigTests(unittest.TestCase):
    def test_two_family_arm_a_config_builds_two_families(self):
        cap_config = {
            "gap_ratio": 0.05,
            "skin_margin_mm": 0.8608,
            "root_solid_layers": 5,
            "overshoot_mm": 0.05,
            "families": [
                {"name": "fwd", "rib_pitch_mm": 8.0, "web_position": 0.4,
                 "z_drift_mm_per_mm": 1.0, "z_period_mm": 12.0},
                {"name": "aft", "rib_pitch_mm": 8.0, "web_position": 0.6,
                 "z_drift_mm_per_mm": -1.0, "z_period_mm": 12.0},
            ],
        }
        spec = sa.build_spec_from_cap_config(cap_config)
        self.assertEqual(len(spec.families), 2)
        self.assertAlmostEqual(spec.skin_margin_mm, 0.8608)
        self.assertEqual(spec.families[0].name, "fwd")
        self.assertEqual(spec.families[1].name, "aft")

    def test_unknown_family_key_raises(self):
        cap_config = {
            "gap_ratio": 0.05,
            "skin_margin_mm": 1.0,
            "families": [{"name": "x", "not_a_real_field": 1}],
        }
        with self.assertRaises(sa.SpecError):
            sa.build_spec_from_cap_config(cap_config)


class EndToEndArmSmokeTests(unittest.TestCase):
    """Run each real committed arm config over a SHORT airfoil stack (a few
    layers, not the full 60mm span, to keep this test fast) and confirm the
    real generator path produces sane, self-consistent output. This is the
    integration-level check that ``analyze_arm`` actually drives
    ``StructureGenerator`` correctly end to end, not merely that the
    isolated maths functions are right.
    """

    def setUp(self):
        self.section = sa.build_airfoil_section(chord_mm=60.0, span_mm=60.0, n_points=120)

    def _load(self, filename):
        return sa.load_arm_cap_config(filename)

    def test_arm_a_two_families_produces_full_depth_webs_and_positive_j(self):
        cap_config = self._load("arm_A_two_families.json")
        result = sa.analyze_arm("A", cap_config, 0.42, self.section, layer_height_mm=1.0)
        self.assertGreater(result.total_web_count, 0)
        self.assertEqual(result.total_web_count, result.full_depth_web_count)
        self.assertGreater(result.j_open_mm4, 0.0)
        self.assertGreater(result.j_closed_mean_mm4, 0.0)
        # Closed-cell (idealised fused) must be >= open-cell: webs can only
        # add stiffness in this model, never remove it.
        self.assertGreaterEqual(result.j_closed_mean_mm4, result.j_open_mm4 - 1e-6)
        self.assertIsNotNone(result.web_tip_air_gap_mm)
        self.assertIsNotNone(result.web_tip_bond_class)
        # Arm A's tight skin_margin_mm loses a large fraction of its
        # requested webs to the "thin section" omission. If a generator
        # change stops this from happening, this assertion should fail
        # loudly rather than let the finding silently stop reproducing.
        self.assertGreater(result.total_omitted_count, 0)
        self.assertIn("thin section", result.omitted_by_reason)

    def test_arm_b_one_family_is_lighter_than_arm_a(self):
        cap_a = self._load("arm_A_two_families.json")
        cap_b = self._load("arm_B_one_family.json")
        result_a = sa.analyze_arm("A", cap_a, 0.42, self.section, layer_height_mm=1.0)
        result_b = sa.analyze_arm("B", cap_b, 0.42, self.section, layer_height_mm=1.0)
        self.assertLess(result_b.mass_proxy_mm3, result_a.mass_proxy_mm3)
        # Open-cell model must be UNCHANGED (webs never enter it) --
        # exercises the tool's own P1-relevant claim.
        self.assertAlmostEqual(result_a.j_open_mm4, result_b.j_open_mm4, places=9)

    def test_arm_d_has_a_larger_web_tip_air_gap_than_arm_a(self):
        cap_a = self._load("arm_A_two_families.json")
        cap_d = self._load("arm_D_unfused_tips_control.json")
        result_a = sa.analyze_arm("A", cap_a, 0.42, self.section, layer_height_mm=1.0)
        result_d = sa.analyze_arm("D", cap_d, 0.42, self.section, layer_height_mm=1.0)
        self.assertGreater(result_d.web_tip_air_gap_mm, result_a.web_tip_air_gap_mm)
        self.assertEqual(result_d.web_tip_bond_class, "none")

    def test_arm_c_thick_skin_has_larger_j_open_than_arm_a(self):
        cap_a = self._load("arm_A_two_families.json")
        cap_c = self._load("arm_C_thick_skin.json")
        result_a = sa.analyze_arm("A", cap_a, 0.42, self.section, layer_height_mm=1.0)
        result_c = sa.analyze_arm("C", cap_c, 0.60, self.section, layer_height_mm=1.0)
        self.assertGreater(result_c.j_open_mm4, result_a.j_open_mm4)


class RunAllArmsSmokeTest(unittest.TestCase):
    """The actual CLI entry path: all four committed arms, full fixture,
    default layer height -- confirms the whole tool runs to completion
    without needing the coupon fallback (the airfoil path must not be
    "blocked" in this offline harness), and that the report renders."""

    def test_run_all_arms_and_render(self):
        results, section_name, fallback_reason = sa.run_all_arms()
        self.assertEqual(len(results), 4)
        self.assertEqual(fallback_reason, "", "airfoil fixture path unexpectedly failed: {}".format(fallback_reason))
        self.assertIn("airfoil", section_name)
        md = sa.render_predicted_md(results, section_name, fallback_reason)
        self.assertIn("PREDICTED / MODELLED", md)
        self.assertIn("Comparison table", md)


if __name__ == "__main__":
    unittest.main()
