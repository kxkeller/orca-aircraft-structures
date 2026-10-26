"""Dry-run every ratio in the gap sweep before any plastic is spent.

The sweep varies `gap_ratio` across six coupons. A ratio that makes the
capability fail closed — because the slot is too wide for the section, or the
margin no longer survives the safety erosion — is far cheaper to discover here
than at the printer, five coupons in.

This exercises the SHIPPED bundle against the fake host at each planned ratio
and asserts the slice completes with the expected slot geometry. It cannot
prove anything about fusion: two walls 0.63 mm apart in G-code either bond in
plastic or do not, and only the print answers that.
"""

import json
import os
import unittest

from tests.integration.test_m1_capability import FULL_CONFIG, M1IntegrationTestCase

# Import the SAME module object the harness uses. `test_m1_capability` puts
# tests/integration on sys.path and imports it top-level as `fake_orca`; a
# `tests.integration.fake_orca` import would be a second, distinct module whose
# shape classes never satisfy the harness's isinstance checks.
import fake_orca  # noqa: E402

# Mirrors the gap-ratio bracket planned for the physical coupon sweep.
SWEEP_RATIOS = (0.25, 0.50, 0.75, 1.00, 1.25, 1.50)

WALL_MM = 0.42          # resolved outer wall width, captured
SECTION_MM = 20.0       # coupon cross-section
LAYERS = 40             # enough to clear the 5 root layers; keeps the test quick


class SweepRatioDryRunTests(M1IntegrationTestCase):
    def _run_at(self, ratio):
        self.arm()
        obj = self.make_object(n_layers=LAYERS)
        ctx = self.make_ctx(obj)
        cap = self.new_capability()
        cap.get_config = lambda: json.dumps({"gap_ratio": ratio, "slot_gap_mm": None})
        return cap.execute(ctx), obj

    def test_every_planned_ratio_slices_without_failing_closed(self):
        PluginResult = self._fake_orca.PluginResult
        failures = []
        for ratio in SWEEP_RATIOS:
            with self.subTest(ratio=ratio):
                result, obj = self._run_at(ratio)
                if result.status != PluginResult.Success:
                    failures.append((ratio, result.message))
                    continue
                # Every body layer must still carry the slot.
                mutated = sum(
                    1 for layer in obj.layers() if layer.make_slices_calls > 0
                )
                self.assertEqual(
                    mutated,
                    LAYERS - FULL_CONFIG_ROOT,
                    "ratio {}: expected every body layer mutated".format(ratio),
                )
        self.assertEqual(
            failures,
            [],
            "these sweep ratios fail closed and would waste a coupon: {}".format(
                failures
            ),
        )

    def test_slot_width_tracks_the_ratio(self):
        """The cut's width must be gap_ratio * wall width, not a fixed value.

        This is what makes the sweep a sweep: if the slot did not scale, every
        coupon would print the same geometry and the experiment would measure
        nothing.
        """
        widths = {}
        for ratio in SWEEP_RATIOS:
            result, obj = self._run_at(ratio)
            if result.status != self._fake_orca.PluginResult.Success:
                continue
            body = [l for l in obj.layers() if l.make_slices_calls > 0][0]
            # Use the harness's own fake module: install() hands out a fresh
            # module object per test, so classes from a separate import would
            # never match its isinstance checks.
            shape = fake_orca.describe(self.surfaces_of(body)[0].expolygon)
            if shape.get("kind") == "notched":
                min_x, _, max_x, _ = (
                    shape["notch"][0], shape["notch"][1],
                    shape["notch"][2], shape["notch"][3],
                )
                # Scaled integers -> mm.
                widths[ratio] = round((max_x - min_x) * 1e-6, 4)

        self.assertTrue(widths, "no ratio produced a measurable notch")
        for ratio, width in widths.items():
            self.assertAlmostEqual(
                width,
                ratio * WALL_MM,
                places=3,
                msg="ratio {} produced a {} mm slot, expected {}".format(
                    ratio, width, ratio * WALL_MM
                ),
            )
        # And the widths must be strictly increasing across the bracket.
        ordered = [widths[r] for r in sorted(widths)]
        self.assertEqual(ordered, sorted(ordered))
        self.assertNotEqual(ordered[0], ordered[-1])


FULL_CONFIG_ROOT = FULL_CONFIG.get("bottom_shell_layers", 5)


if __name__ == "__main__":
    unittest.main()
