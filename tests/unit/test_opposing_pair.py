"""An OPPOSING PAIR of members whose flanks actually FUSE and whose depths
OVERLAP.

One ``WebFamily`` with ``opposing_pair=True`` emits TWO ``WebIntent``s per
station (one entering each skin), offset in X just far enough that the printed
flanks fuse (measured hard-fuse gap), with a depth derived every layer from
``local_solid_thickness_worst_case`` over the pair's own combined footprint --
see ``StructureGenerator._generate_opposing_pair_family``'s own docstring for
the full derivation and its guarantee argument.
"""
import unittest

from aircraft_core.frame import opposing_pair_meets
from aircraft_core.generator import LayerContext, StructureGenerator
from aircraft_core.spec import SpecError, StructureSpec, WebFamily
from stub_ops import Rect, StubOps

LINE_WIDTH_MM = 0.42
SAFETY_OFFSET_MM = 0.42
SLOT_GAP_MM = 0.021
# own shipped shape: > 2 * safety_offset_mm strictly (own precondition), by the
# same 0.021 mm cushion measured as hard-fuse.
SKIN_MARGIN_MM = 2.0 * SAFETY_OFFSET_MM + 0.021  # 0.861
MIN_WALL_MM = 2.0 * SAFETY_OFFSET_MM  # 0.84
FIN_THICKNESS_MM = 2.0 * LINE_WIDTH_MM + SLOT_GAP_MM  # 0.861
DEFAULT_PAIR_OFFSET_MM = FIN_THICKNESS_MM + 0.021  # 0.882


def _pair_spec(width_mm=40.0, pair_offset_mm=None, web_position=0.5):
    family = WebFamily(
        name="pair",
        web_position=web_position,
        slot_gap_mm=SLOT_GAP_MM,
        opposing_pair=True,
        pair_offset_mm=pair_offset_mm,
    )
    spec = StructureSpec(
        slot_gap_mm=SLOT_GAP_MM,
        skin_margin_mm=SKIN_MARGIN_MM,
        families=(family,),
    )
    return family, spec


def _generate(rect, spec, layer_index=0, print_z=0.2):
    ops = StubOps()
    from aircraft_core.frame import LayerFrame

    frame = LayerFrame.from_polygon(ops, rect)
    gen = StructureGenerator()
    layer = LayerContext(layer_index=layer_index, print_z=print_z)
    result = gen.generate(
        frame, spec, LINE_WIDTH_MM, layer, safety_offset_mm=SAFETY_OFFSET_MM
    )
    return frame, gen, result


class OpposingPairEmissionTests(unittest.TestCase):
    """Test (a) -- a pair is emitted as TWO intents, opposite entry sides,
    at the expected stations."""

    def test_two_intents_opposite_entry_sides_at_expected_stations(self):
        width_mm = 40.0
        _, spec = _pair_spec(width_mm=width_mm)
        rect = Rect(0.0, 0.0, width_mm, 6.0)
        frame, _gen, result = _generate(rect, spec)

        self.assertEqual(len(result.intents), 2)
        by_entry = {i.entry_lower: i for i in result.intents}
        self.assertEqual(set(by_entry), {False, True})

        anchor_x = frame.web_x(0.5)
        expected_upper_x = anchor_x - DEFAULT_PAIR_OFFSET_MM / 2.0
        expected_lower_x = anchor_x + DEFAULT_PAIR_OFFSET_MM / 2.0
        self.assertAlmostEqual(by_entry[False].start[0], expected_upper_x, places=6)
        self.assertAlmostEqual(by_entry[True].start[0], expected_lower_x, places=6)
        # Both from the SAME family.
        self.assertEqual(by_entry[False].family_index, by_entry[True].family_index)
        self.assertEqual(by_entry[False].family_name, "pair")
        self.assertEqual(by_entry[True].family_name, "pair")


class OpposingPairFlankFuseGapTests(unittest.TestCase):
    """Test (b) -- the pair's flank gap is <= 0.021 mm (fused) at the
    default offset, computed as
    ``centre_spacing - (2 * line_width + slot_gap)``."""

    def test_default_offset_flank_gap_is_the_hard_fuse_gap(self):
        width_mm = 40.0
        _, spec = _pair_spec(width_mm=width_mm)
        rect = Rect(0.0, 0.0, width_mm, 6.0)
        _frame, _gen, result = _generate(rect, spec)

        by_entry = {i.entry_lower: i for i in result.intents}
        centre_spacing = abs(by_entry[True].start[0] - by_entry[False].start[0])
        flank_gap = centre_spacing - (2.0 * LINE_WIDTH_MM + SLOT_GAP_MM)
        self.assertLessEqual(flank_gap, 0.021 + 1e-9)
        self.assertAlmostEqual(flank_gap, 0.021, places=6)


class OpposingPairDepthOverlapTests(unittest.TestCase):
    """Test (c): the two members' far ends must OVERLAP, which is the whole
    point of the pair -- material runs upper skin -> down one fin -> across the
    fused flank -> up the other -> lower skin.

    DEPTH DERIVATION. A depth of ``d = (T_worst - 2*safety_offset - eps)/2``
    would make overlap impossible: ``T_worst`` is a MINIMUM over a span
    containing both entries, so ``T_worst <= T_actual`` and

        end_y_upper - end_y_lower = T_actual - 2*d >= min_wall + eps > 0

    -- a guaranteed GAP. That formula wrongly assumes a member starts one
    ``skin_margin`` INSIDE its skin, so that overlap needs
    ``d_u + d_l > T - 2*skin_margin``. A member starts AT its surface and stops
    ``skin_margin`` short of the far one, so overlap needs ``d_u + d_l > T``.
    Each member may individually reach ``T - skin_margin`` (exactly what a
    full-depth member already does), so a pair reaches ``2T - 2*skin_margin``,
    which exceeds ``T`` whenever ``T > 2*skin_margin`` -- true of any section
    this plugin can already place a full-depth member in.

    does not constrain the pair: at the fusing offset the two haloed footprints
    are X-disjoint, so ``opposing_pair_meets`` returns ``None``.
    """
    def test_far_ends_overlap_as_literally_specified(self):
        width_mm = 40.0
        _, spec = _pair_spec(width_mm=width_mm)
        rect = Rect(0.0, 0.0, width_mm, 6.0)
        _frame, _gen, result = _generate(rect, spec)

        by_entry = {i.entry_lower: i for i in result.intents}
        end_y_upper = by_entry[False].end[1]
        end_y_lower = by_entry[True].end[1]
        # "the upper member's far end is beyond the lower member's far
        # end" -- literally, end_y_upper < end_y_lower (upper descends
        # from the top skin, lower ascends from the bottom skin).
        self.assertLess(end_y_upper, end_y_lower)


class OpposingPairNotOmittedByM12Tests(unittest.TestCase):
    """Test (d) -- the pair is NOT omitted; it satisfies the opposing-entry check
    by construction (assert no omission with reason "opposing entry").

    WHY IT PASSES: NOT because the derived depth formula makes the pair safe
    (is never even evaluated for this pair -- see below), but because the
    default offset is chosen wide enough that the two candidates' HALOED
    (widened by safety_offset_mm) X footprints are already disjoint: at the
    default offset the haloed gap is exactly ``pair_offset_mm - (2 * line_width
    + slot_gap) - 2 * safety_offset_mm = 0.021 mm`` (own hard-fuse gap, again
    -- not a coincidence: both the flank-fuse gap and this haloed-disjointness
    margin are priced off the same measured constant). ``opposing_pair_meets``
    returns ``None`` (safe) for ANY depth once footprints are X-disjoint, per
    its own docstring ("no shared span, so the meeting condition has nothing to
    be evaluated over"). Confirmed directly below.
    """

    def test_no_opposing_entry_omission_and_predicate_confirms_disjoint(self):
        width_mm = 40.0
        _, spec = _pair_spec(width_mm=width_mm)
        rect = Rect(0.0, 0.0, width_mm, 6.0)
        _frame, _gen, result = _generate(rect, spec)

        opposing_omissions = [
            o for o in result.omitted if o.reason.startswith("opposing entry")
        ]
        self.assertEqual(opposing_omissions, [])

        by_entry = {i.entry_lower: i for i in result.intents}
        upper, lower = by_entry[False], by_entry[True]
        u_min_x, u_max_x = upper.footprint_x()
        l_min_x, l_max_x = lower.footprint_x()
        meeting = opposing_pair_meets(
            _frame.ops,
            _frame.polygon,
            u_min_x,
            u_max_x,
            upper.depth_mm(),
            l_min_x,
            l_max_x,
            lower.depth_mm(),
            SAFETY_OFFSET_MM,
            MIN_WALL_MM,
        )
        self.assertIsNone(
            meeting,
            "expected opposing_pair_meets to be vacuously safe (X-disjoint "
            "haloed footprints), not merely safe by depth",
        )


class OpposingPairDifferentThicknessDifferentDepthTests(unittest.TestCase):
    """Test (e) -- two layers with DIFFERENT local thickness produce
    DIFFERENT depths (the fixed-depth failure this feature replaces)."""

    def test_thickness_change_changes_derived_depth(self):
        width_mm = 40.0
        _, spec_thick = _pair_spec(width_mm=width_mm)
        _, spec_thin = _pair_spec(width_mm=width_mm)

        rect_thick = Rect(0.0, 0.0, width_mm, 6.0)
        rect_thin = Rect(0.0, 0.0, width_mm, 3.0)

        _f1, _g1, result_thick = _generate(rect_thick, spec_thick)
        _f2, _g2, result_thin = _generate(rect_thin, spec_thin)

        depth_thick = result_thick.intents[0].depth_mm()
        depth_thin = result_thin.intents[0].depth_mm()

        self.assertNotAlmostEqual(depth_thick, depth_thin, places=3)
        # Sanity: matches the derivation directly.
        # Full depth per side: t_worst - skin_margin - epsilon. The halved
        # form this once pinned could not produce overlap at all.
        expected_thick = 6.0 - SKIN_MARGIN_MM - 1e-6
        expected_thin = 3.0 - SKIN_MARGIN_MM - 1e-6
        self.assertAlmostEqual(depth_thick, expected_thick, places=5)
        self.assertAlmostEqual(depth_thin, expected_thin, places=5)


class OpposingPairTooThinOmissionTests(unittest.TestCase):
    """Test (f) -- a too-thin section omits the pair with the stated
    reason, never a negative or degenerate depth."""

    def test_too_thin_section_is_omitted_not_built_degenerate(self):
        width_mm = 40.0
        _, spec = _pair_spec(width_mm=width_mm)
        # T_worst = 0.85 => depth = (0.85 - 0.84 - eps) / 2 ~= 0.005 mm,
        # below _MIN_USEFUL_OPPOSING_PAIR_DEPTH_MM (0.05 mm).
        rect = Rect(0.0, 0.0, width_mm, 0.85)
        _frame, _gen, result = _generate(rect, spec)

        self.assertEqual(result.intents, [])
        # this plain rectangle (fill ratio 1.0) also reaches the zero-intent
        # feature fallback, which tries and -- correctly, on material this thin
        # -- fails to place its own lattice, adding its own omissions to the
        # list rather than replacing it (see generate()'s own fallback
        # comment). Filter to this family's own omission rather than assuming
        # it is the only one recorded.
        pair_omitted = [o for o in result.omitted if o.family_name == "pair"]
        self.assertEqual(len(pair_omitted), 1)
        reason = pair_omitted[0].reason
        self.assertTrue(reason.startswith("opposing pair:"))
        self.assertIn("minimum useful opposing-pair depth", reason)

    def test_negative_derived_depth_is_also_omitted_cleanly(self):
        width_mm = 40.0
        _, spec = _pair_spec(width_mm=width_mm)
        # T_worst = 0.5 < min_wall_mm (0.84) => depth would be negative.
        rect = Rect(0.0, 0.0, width_mm, 0.5)
        _frame, _gen, result = _generate(rect, spec)

        self.assertEqual(result.intents, [])
        # see the previous test's own comment -- filter to this family's own
        # omission.
        pair_omitted = [o for o in result.omitted if o.family_name == "pair"]
        self.assertEqual(len(pair_omitted), 1)
        self.assertTrue(pair_omitted[0].reason.startswith("opposing pair:"))


class OpposingPairOverwideOffsetRefusedTests(unittest.TestCase):
    """Test (g) -- an over-wide pair_offset_mm is refused with the stated
    reason, rather than silently building a non-fusing pair."""

    def test_overwide_pair_offset_raises_value_error(self):
        width_mm = 40.0
        max_allowed = FIN_THICKNESS_MM + 0.021
        _, spec = _pair_spec(width_mm=width_mm, pair_offset_mm=max_allowed + 0.5)
        rect = Rect(0.0, 0.0, width_mm, 6.0)
        ops = StubOps()
        from aircraft_core.frame import LayerFrame

        frame = LayerFrame.from_polygon(ops, rect)
        gen = StructureGenerator()
        layer = LayerContext(layer_index=0, print_z=0.2)
        with self.assertRaises(ValueError) as ctx:
            gen.generate(
                frame, spec, LINE_WIDTH_MM, layer, safety_offset_mm=SAFETY_OFFSET_MM
            )
        self.assertIn("pair_offset_mm", str(ctx.exception))
        self.assertIn("cannot fuse", str(ctx.exception))

    def test_pair_offset_mm_must_be_positive_at_spec_level(self):
        family = WebFamily(name="pair", opposing_pair=True, pair_offset_mm=0.0)
        with self.assertRaises(SpecError):
            family.validate(0)


class OpposingPairUnaffectedFamiliesTests(unittest.TestCase):
    """Test (h) -- families without the opposing_pair flag are
    byte-identical to before (pinned against a concrete, hand-computed
    existing case: a fixed-depth, +Y-entry, single-station family on a
    uniform rectangle)."""

    def test_ordinary_family_output_unchanged(self):
        width_mm = 40.0
        family = WebFamily(
            name="ordinary", web_position=0.5, web_depth_mm=1.5, slot_gap_mm=SLOT_GAP_MM
        )
        spec = StructureSpec(
            slot_gap_mm=SLOT_GAP_MM, skin_margin_mm=SKIN_MARGIN_MM, families=(family,)
        )
        rect = Rect(0.0, 0.0, width_mm, 6.0)
        _frame, _gen, result = _generate(rect, spec)

        self.assertEqual(len(result.intents), 1)
        intent = result.intents[0]
        self.assertFalse(intent.entry_lower)
        self.assertAlmostEqual(intent.start[1], 6.0, places=9)
        self.assertAlmostEqual(intent.end[1], 4.5, places=9)
        self.assertAlmostEqual(intent.width_mm, SLOT_GAP_MM, places=9)
        self.assertEqual(result.omitted, [])


if __name__ == "__main__":
    unittest.main()
