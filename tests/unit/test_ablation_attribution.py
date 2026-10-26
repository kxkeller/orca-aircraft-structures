"""Direct unit coverage of ``aircraft_core.validate._ablation_attribution``, which
attributes a failure to a slot by removing slots one at a time (ablation)
rather than by a positional "nearest requested pair by X" heuristic.

``tests/unit/test_junction.py`` already exercises this through
``TopologyValidator.validate()``'s own failure messages (including the
"cause not localised" case, when the exact-arithmetic stub cannot re-diff a
given remaining slot set). This file adds direct coverage of the three
outcomes: (a) ablation naming the TRUE single
culprit among several innocent slots, (b) the not-localised fallback when
no single removal helps, and (c) the bound-exceeded skip when a layer
carries more requested slots than ``_ABLATION_MAX_SLOTS``.
"""
import unittest

from aircraft_core.validate import _ABLATION_MAX_SLOTS, _ablation_attribution
from stub_ops import Rect, StubOps


class AblationNamesTrueCulpritTests(unittest.TestCase):
    """Two slots on a rectangular coupon: one harmless top notch, and one
    full-height slot that alone severs the section. ``stub_ops.StubOps.diff``
    only subtracts sequential clips from a plain ``Rect`` subject (see its
    own module docstring) — real multi-clip cascaded diff on an already-cut
    shape is deliberately narrow — so this fixture stays to exactly ONE
    slot per ablation re-diff, which is exactly what ``_ablation_attribution``
    itself does (each candidate is re-diffed against the ORIGINAL with only
    the OTHER requested slots, one clip here). Ablation must name exactly
    the severing slot — not the harmless notch, and not "not localised" —
    by actually re-cutting the layer with each one held out in turn and
    checking the SAME invariant (island count == 1) the real caller would
    check.
    """

    def setUp(self):
        self.ops = StubOps()
        self.original = Rect(0.0, 0.0, 20.0, 20.0)
        # A harmless top-edge notch, well clear of the severing slot below.
        self.notch_a = Rect(1.0, 15.0, 2.0, 20.0)
        # A full-height slot: by itself this severs the section into two
        # islands (a "full vertical split", one of the cases
        # ``stub_ops.StubOps.diff`` implements directly).
        self.sever_slot = Rect(9.7, 0.0, 10.3, 20.0)

    def test_ablation_names_only_the_severing_slot(self):
        slots = [self.notch_a, self.sever_slot]
        # Sanity: this layer really is severed when BOTH slots are cut.
        self.assertEqual(
            len(self.ops.diff(self.original, [self.sever_slot])), 2,
            "sanity: the sever slot alone already severs the section",
        )

        message = _ablation_attribution(
            self.ops,
            [self.original],
            slots,
            slot_labels=["family 0 ('a')", "family 0 ('sever')"],
            check_ok=lambda t: len(t) == 1,
        )
        self.assertIn("Ablation localises the cause to", message)
        self.assertIn("slot 2 (family 0 ('sever'))", message)
        self.assertNotIn("slot 1 (", message)


class AblationNotLocalisedTests(unittest.TestCase):
    """Two slots that EACH sever the section on their own (see
    ``test_junction.py``'s own sever/hole fixtures for the same case at the
    ``TopologyValidator`` level): removing either one individually never
    restores validity, so the honest outcome is "not localised", not a
    guess.
    """

    def test_no_single_removal_helps(self):
        ops = StubOps()
        original = Rect(0.0, 0.0, 20.0, 20.0)
        slot_a = Rect(9.7, 0.0, 10.0, 20.0)
        slot_b = Rect(10.0, 0.0, 10.3, 20.0)
        message = _ablation_attribution(
            ops,
            [original],
            [slot_a, slot_b],
            slot_labels=None,
            check_ok=lambda t: len(t) == 1,
        )
        self.assertIn("Cause not localised to any single slot among the 2 requested", message)
        self.assertNotIn("Ablation localises", message)


class AblationBoundTests(unittest.TestCase):
    """Above ``_ABLATION_MAX_SLOTS`` requested slots, ablation is skipped
    outright (bounded cost) and the message says so explicitly."""

    def test_bound_exceeded_skips_ablation_and_says_so(self):
        ops = StubOps()
        original = Rect(0.0, 0.0, 20.0, 20.0)
        # Content of the "slots" is irrelevant here -- the bound check
        # fires before any re-diff is attempted, so plain placeholders
        # (never diffed) are enough to prove ablation is skipped, not that
        # it silently fails on real geometry.
        slots = [object()] * (_ABLATION_MAX_SLOTS + 1)
        message = _ablation_attribution(
            ops,
            [original],
            slots,
            slot_labels=None,
            check_ok=lambda t: True,
        )
        self.assertIn("above the {}-slot ablation bound".format(_ABLATION_MAX_SLOTS), message)
        self.assertIn(str(_ABLATION_MAX_SLOTS + 1), message)
        self.assertNotIn("Ablation localises", message)

    def test_bound_is_at_least_the_layer_that_motivated_this_fix(self):
        # The real in-host failure that motivated ablation carried 47
        # requested slots. A bound below that would make ablation never fire on the exact
        # case it exists for.
        self.assertGreaterEqual(_ABLATION_MAX_SLOTS, 47)


if __name__ == "__main__":
    unittest.main()
