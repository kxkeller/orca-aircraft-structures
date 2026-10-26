import importlib.util
import unittest
from pathlib import Path

from aircraft_core.frame import LayerFrame
from aircraft_core.generator import (
    FULL_DEPTH_MARGIN_HEADROOM_MM,
    GenerationResult,
    LayerContext,
    OmittedWeb,
    StructureGenerator,
    WebIntent,
)
from aircraft_core.spec import StructureSpec, WebFamily
from stub_ops import PolygonShape, Rect, StubOps

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MAKE_FIXTURES_SCRIPT = REPO_ROOT / "tools" / "make_fixtures.py"


def _load_make_fixtures_module():
    # Same pattern as test_validate.py / test_local_geometry.py: load
    # tools/make_fixtures.py's own point-generation code rather than keeping
    # a second airfoil implementation in this file.
    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_generator", MAKE_FIXTURES_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_mf = _load_make_fixtures_module()


def _naca2412_polygon():
    """The fixture's exact 2D section -- same parameters as
    tests/fixtures/airfoil/naca2412_c60_s60.stl and test_validate.py's
    _naca2412_polygon()."""
    m, p, t = _mf.parse_naca4("2412")
    _triangles, outline_mm, bbox = _mf.airfoil_triangles(m, p, t, 60.0, 60.0, 120)
    return PolygonShape(tuple(outline_mm)), bbox


class GeneratorTests(unittest.TestCase):
    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def test_exactly_one_intent_for_m1(self):
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0)
        result = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        self.assertEqual(len(result.intents), 1)
        self.assertIsInstance(result.intents[0], WebIntent)
        self.assertEqual(result.omitted, [])

    def test_intent_geometry_matches_the_frame(self):
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5)
        intent = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)).intents[0]
        self.assertEqual(intent.start, (10.0, 20.0))
        # Full-depth far end: skin_margin_mm plus the derived placement
        # headroom (FULL_DEPTH_MARGIN_HEADROOM_MM) — see
        # aircraft_core/generator.py's docstring for that constant for why
        # a bare skin_margin_mm (no headroom) is not safe against
        # validate.py's independent re-measurement.
        self.assertEqual(intent.end, (10.0, 1.0 + FULL_DEPTH_MARGIN_HEADROOM_MM))
        self.assertAlmostEqual(intent.width_mm, 0.63)

    def test_intent_width_from_explicit_gap(self):
        spec = StructureSpec(slot_gap_mm=0.6, skin_margin_mm=1.0)
        intent = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)).intents[0]
        self.assertEqual(intent.width_mm, 0.6)

    def test_full_depth_headroom_boundary_still_omits_a_razor_thin_station(self):
        # A station with room for EXACTLY skin_margin_mm plus a hair less
        # than FULL_DEPTH_MARGIN_HEADROOM_MM must still be omitted, not
        # built with end_y >= y_max (which would flip SlotBuilder's
        # direction vector). The 20 mm Rect fixture's local thickness at
        # any station is exactly 20.0 mm, so skin_margin_mm is chosen to
        # leave less than the headroom above it.
        thickness_mm = 20.0
        spec = StructureSpec(
            gap_ratio=1.5,
            skin_margin_mm=thickness_mm - (FULL_DEPTH_MARGIN_HEADROOM_MM / 2.0),
            web_position=0.5,
        )
        result = StructureGenerator().generate(
            self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        )
        # this test is about the margin/headroom boundary specifically, not
        # about whether the whole layer ends up with zero structure -- the 20mm
        # square Rect fixture (fill ratio 1.0) also satisfies classify_region's
        # FEATURE test, so the zero-intent fallback legitimately
        # contributes a separate feature-lattice intent here (a real,
        # comfortably-sized region -- unlike the razor-thin opposing-pair
        # fixtures, this one is not too thin for a lattice). Assert on the
        # DEFAULT family's own omission specifically, not on the whole-layer
        # intent count.
        default_omitted = [o for o in result.omitted if o.family_name == "default"]
        self.assertEqual(len(default_omitted), 1)
        # The candidate must not have been BUILT (end_y >= y_max would flip
        # SlotBuilder's direction vector) -- unaffected by the fallback's own
        # (separate) feature intent.
        self.assertEqual(
            [i for i in result.intents if i.family_name == "default"], []
        )
        self.assertTrue(default_omitted[0].reason.startswith("skin margin"), default_omitted[0].reason)
        self.assertIn("headroom", default_omitted[0].reason)

    def test_genuine_margin_violation_still_omitted_headroom_does_not_mask_it(self):
        # A GENUINE violation -- skin_margin_mm well beyond both the
        # available thickness AND the headroom -- must still be omitted.
        # Proves FULL_DEPTH_MARGIN_HEADROOM_MM (0.0002 mm) did not make
        # this check permissive: the deficit here (10 mm) is five orders
        # of magnitude larger than the headroom it could ever absorb.
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=30.0, web_position=0.5)
        result = StructureGenerator().generate(
            self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        )
        # see the previous test's own comment -- assert on the DEFAULT family's
        # own omission, not the whole-layer intent count (a separate
        # feature-lattice attempt legitimately contributes here too, on this
        # comfortably-sized 20mm square).
        default_omitted = [o for o in result.omitted if o.family_name == "default"]
        self.assertEqual(len(default_omitted), 1)
        self.assertEqual(
            [i for i in result.intents if i.family_name == "default"], []
        )
        self.assertTrue(default_omitted[0].reason.startswith("skin margin"), default_omitted[0].reason)

    def test_no_destructive_ops_invoked(self):
        # The frame was already built (via ops.bounds()) before this point;
        # reset the call log so this assertion is specifically about
        # generate() itself, not setUp().
        #
        # generate() -> frame.web_centerline() -> local_span() queries the
        # ops object (bounds/polygon_from_points/intersection) to measure the
        # local surfaces at the web station. What the "no Booleans" invariant
        # protects is that intent generation never performs the DESTRUCTIVE
        # cut itself, so the check is "no diff/offset call", not "no ops
        # call at all".
        self.ops.calls.clear()
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0)
        StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        destructive = [name for name, _args in self.ops.calls if name in ("diff", "offset")]
        self.assertEqual(
            destructive,
            [],
            "StructureGenerator must not perform a destructive boolean op "
            "(diff/offset) — only read-only geometry queries for local "
            "measurement. Calls seen: {!r}".format(self.ops.calls),
        )


class MultiWebGeneratorTests(unittest.TestCase):
    """multiple parallel webs, spaced by ``rib_pitch_mm``.

    Anchoring rule under test: candidate stations are
    ``web_x(web_position) + k * rib_pitch_mm`` for every integer ``k``,
    computed directly from ``k`` and marching outward from the anchor both
    ways, stopping once a candidate's own X leaves the section's chord
    extent (see ``StructureGenerator``'s docstring).
    """

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def test_multiple_webs_on_rectangular_coupon(self):
        # gap_ratio=1.5 against outer_wall=0.42 -> width_mm=0.63, half=0.315.
        # anchor=10.0 (web_position=0.5 of a 20mm-wide rect); pitch=5.0mm ->
        # candidates 0, 5, 10, 15, 20. The two extreme candidates (0 and 20)
        # have a footprint that pokes 0.315 mm past the chord extent
        # [0, 20], so they must be omitted (reason: "chord extent"); the
        # inner three (5, 10, 15) have ample margin on this 20x20 rect and
        # must all be produced.
        spec = StructureSpec(
            gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5, rib_pitch_mm=5.0
        )
        result = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        self.assertIsInstance(result, GenerationResult)

        xs = sorted(intent.start[0] for intent in result.intents)
        self.assertEqual(xs, [5.0, 10.0, 15.0])

        omitted_xs = sorted(o.x_mm for o in result.omitted)
        self.assertEqual(omitted_xs, [0.0, 20.0])
        for o in result.omitted:
            self.assertIsInstance(o, OmittedWeb)
            self.assertTrue(o.reason.startswith("chord extent"), o.reason)

    def test_anchoring_is_deterministic_and_reproducible(self):
        spec = StructureSpec(
            gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5, rib_pitch_mm=5.0
        )
        result_a = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        result_b = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        self.assertEqual(
            [i.start for i in result_a.intents], [i.start for i in result_b.intents]
        )
        self.assertEqual(
            [o.x_mm for o in result_a.omitted], [o.x_mm for o in result_b.omitted]
        )
        # Every produced station is exactly the anchor plus an integer
        # multiple of the pitch -- the formula, not an accumulated sum.
        anchor_x = self.frame.web_x(spec.web_position)
        for intent in result_a.intents:
            offset = intent.start[0] - anchor_x
            k = round(offset / spec.rib_pitch_mm)
            self.assertAlmostEqual(anchor_x + k * spec.rib_pitch_mm, intent.start[0])

    def test_single_web_when_pitch_exceeds_chord(self):
        spec = StructureSpec(
            gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5, rib_pitch_mm=100.0
        )
        result = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        self.assertEqual(len(result.intents), 1)
        self.assertAlmostEqual(result.intents[0].start[0], 10.0)
        self.assertEqual(result.omitted, [])

    def test_pitch_none_is_identical_to_single_web(self):
        spec_none = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5)
        spec_wide = StructureSpec(
            gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5, rib_pitch_mm=100.0
        )
        result_none = StructureGenerator().generate(self.frame, spec_none, 0.42, LayerContext(layer_index=0, print_z=0.0))
        result_wide = StructureGenerator().generate(self.frame, spec_wide, 0.42, LayerContext(layer_index=0, print_z=0.0))
        self.assertEqual(
            [i.start for i in result_none.intents], [i.start for i in result_wide.intents]
        )

    def test_negative_pitch_or_zero_is_rejected_by_spec(self):
        from aircraft_core.spec import SpecError

        for bad in (0.0, -1.0):
            spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, rib_pitch_mm=bad)
            with self.assertRaises(SpecError):
                spec.validate()


class MultiWebAirfoilGeneratorTests(unittest.TestCase):
    """on the real NACA 2412 section: multiple webs, with omissions recorded near
    the thin trailing edge."""

    def setUp(self):
        self.ops = StubOps()
        self.polygon, self.bbox = _naca2412_polygon()
        self.frame = LayerFrame.from_polygon(self.ops, self.polygon)

    def test_multiple_webs_across_the_real_chord(self):
        # 60 mm chord, a 10 mm pitch anchored at 0.5 chord: several webs should
        # be produced, with the ones nearest the leading/trailing edge omitted
        # for insufficient local thickness under a 1.0 mm margin.
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, web_position=0.5, rib_pitch_mm=10.0
        )
        result = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))

        self.assertGreater(
            len(result.intents), 1, "expected more than one surviving web at this pitch"
        )
        xs = [i.start[0] for i in result.intents]
        self.assertEqual(xs, sorted(xs), "intents must be in ascending chord order")

        for o in result.omitted:
            self.assertTrue(
                o.reason.startswith("chord extent") or o.reason.startswith("skin margin"),
                o.reason,
            )

        # Every candidate the anchoring rule could have produced ends up
        # either an intent or a recorded omission -- nothing vanishes
        # silently.
        anchor_x = self.frame.web_x(spec.web_position)
        candidate_count = 0
        k = 0
        while anchor_x + k * spec.rib_pitch_mm <= self.frame.max_x + 1e-6:
            if anchor_x + k * spec.rib_pitch_mm >= self.frame.min_x - 1e-6:
                candidate_count += 1
            k += 1
        k = -1
        while anchor_x + k * spec.rib_pitch_mm >= self.frame.min_x - 1e-6:
            candidate_count += 1
            k -= 1
        self.assertEqual(candidate_count, len(result.intents) + len(result.omitted))

    def test_omission_near_thin_trailing_edge_is_recorded(self):
        # web_position=0.95 alone is already too thin for a 1.0 mm margin
        # on this NACA 2412 section (see test_validate.py's
        # TopologyValidatorAirfoilTests / test_airfoil_capability.py's
        # ThinTrailingEdgeTests, which independently measure ~0.85 mm local
        # thickness there). With a wide pitch there is only this one
        # candidate, and it must be omitted -- and recorded -- not silently
        # dropped and not raised as an exception.
        spec = StructureSpec(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            web_position=0.95,
            rib_pitch_mm=1000.0,
        )
        result = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        self.assertEqual(result.intents, [])
        self.assertEqual(len(result.omitted), 1)
        self.assertTrue(result.omitted[0].reason.startswith("skin margin"))
        self.assertAlmostEqual(
            result.omitted[0].x_mm, self.frame.web_x(0.95), places=6
        )


class WebDepthGeneratorTests(unittest.TestCase):
    """Web depth: a web stops ``web_depth_mm`` in from the LOCAL entry
    surface instead of running to ``skin_margin_mm`` short of the far
    surface."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def test_depth_none_reproduces_full_depth_exactly(self):
        spec_full = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5)
        spec_none = StructureSpec(
            gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5, web_depth_mm=None
        )
        intent_full = StructureGenerator().generate(self.frame, spec_full, 0.42, LayerContext(layer_index=0, print_z=0.0)).intents[0]
        intent_none = StructureGenerator().generate(self.frame, spec_none, 0.42, LayerContext(layer_index=0, print_z=0.0)).intents[0]
        self.assertEqual(intent_full.start, intent_none.start)
        self.assertEqual(intent_full.end, intent_none.end)

    def test_shallow_web_stops_at_requested_depth_from_entry_surface(self):
        # Entry surface (boundary_point) is y=20.0 on this rect; a 5.0 mm deep
        # web must stop at y=15.0, well short of the full-depth margin point
        # (y=1.0).
        spec = StructureSpec(
            gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5, web_depth_mm=5.0
        )
        intent = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)).intents[0]
        self.assertEqual(intent.start, (10.0, 20.0))
        self.assertEqual(intent.end, (10.0, 15.0))

    def test_depth_breaching_skin_margin_fails_closed_naming_family(self):
        # Full local thickness is 20.0 mm; a depth of 19.5 mm would leave
        # only 0.5 mm before the far surface, below the required 1.0 mm
        # skin_margin_mm.
        spec = StructureSpec(
            gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5, web_depth_mm=19.5
        )
        with self.assertRaises(ValueError) as ctx:
            StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        message = str(ctx.exception)
        self.assertIn("family 0", message)
        self.assertIn("'default'", message)
        self.assertIn("web_depth_mm", message)

    def test_depth_exactly_at_margin_boundary_passes(self):
        # Depth of exactly 19.0 mm leaves exactly the 1.0 mm margin -- must
        # not be spuriously refused at the boundary (mirrors
        # test_validate.py's test_slots_exactly_at_minimum_wall_pass).
        spec = StructureSpec(
            gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5, web_depth_mm=19.0
        )
        intent = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)).intents[0]
        self.assertAlmostEqual(intent.end[1], 1.0)


class WebDepthAirfoilGeneratorTests(unittest.TestCase):
    """Web depth on the real NACA 2412 section: a shallow web follows the
    LOCAL surface, not a bounding-box approximation."""

    def setUp(self):
        self.ops = StubOps()
        self.polygon, self.bbox = _naca2412_polygon()
        self.frame = LayerFrame.from_polygon(self.ops, self.polygon)

    def test_shallow_web_on_real_airfoil_stops_at_requested_depth(self):
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, web_position=0.5, web_depth_mm=1.5
        )
        intent = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)).intents[0]
        entry_y = intent.start[1]
        self.assertAlmostEqual(entry_y - intent.end[1], 1.5, places=6)
        # A shallow web must be well short of the full-depth margin point --
        # the whole point of the feature.
        full_spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0, web_position=0.5)
        full_intent = StructureGenerator().generate(self.frame, full_spec, 0.42, LayerContext(layer_index=0, print_z=0.0)).intents[0]
        self.assertGreater(intent.end[1], full_intent.end[1])

    def test_depth_breach_on_thin_station_fails_closed(self):
        # At web_position=0.9 this section is locally ~1.61 mm thick --
        # enough to clear the 1.0 mm skin_margin_mm on its own (so the
        # station is NOT omitted), but a requested 2.0 mm depth needs
        # 2.0 + 1.0 = 3.0 mm, more than is locally available. This is the
        # "fits a full-depth/shallower web but not THIS family's depth"
        # case that must fail closed rather than silently
        # deepening toward the far surface.
        spec = StructureSpec(
            gap_ratio=0.5,
            skin_margin_mm=1.0,
            web_position=0.9,
            web_depth_mm=2.0,
        )
        with self.assertRaises(ValueError) as ctx:
            StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        self.assertIn("family 0", str(ctx.exception))

    def test_marching_shallow_family_omits_depth_breach_instead_of_raising(self):
        # Same physical breach as test_depth_breach_on_thin_station_fails_
        # closed (web_position=0.9, web_depth_mm=2.0, skin_margin_mm=1.0 --
        # locally ~1.61 mm thick, enough for skin_margin_mm alone but not
        # skin_margin_mm + web_depth_mm), except this family MARCHES
        # (rib_pitch_mm is not None) rather than naming one explicit station. A
        # marching family cannot itself steer its arithmetic sequence around a
        # locally-thin station --) -- so the breach must be recorded as an
        # omission, never a fatal error for the whole layer.
        family = WebFamily(
            name="stiffener",
            rib_pitch_mm=3.9,
            web_depth_mm=2.0,
            web_position=0.9,
        )
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0, families=(family,))
        result = StructureGenerator().generate(
            self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0)
        )
        depth_breach_omissions = [
            o
            for o in result.omitted
            if o.reason.startswith("skin margin:")
            and "web_depth_mm=2.0000" in o.reason
        ]
        self.assertTrue(
            depth_breach_omissions,
            "expected at least one depth-breach omission; got: {}".format(
                [o.reason for o in result.omitted]
            ),
        )
        for o in depth_breach_omissions:
            self.assertEqual(o.family_index, 0)
            self.assertEqual(o.family_name, "stiffener")


class WebFamilyGeneratorTests(unittest.TestCase):
    """Families: groups of webs sharing pitch, depth and phase."""

    def setUp(self):
        self.ops = StubOps()
        self.rect = Rect(0.0, 0.0, 20.0, 20.0)
        self.frame = LayerFrame.from_polygon(self.ops, self.rect)

    def test_no_families_is_single_implicit_family_tagged_default(self):
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5)
        result = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        self.assertEqual(len(result.intents), 1)
        self.assertEqual(result.intents[0].family_index, 0)
        self.assertEqual(result.intents[0].family_name, "default")

    def test_no_families_reproduces_pre_families_intents_exactly(self):
        # Backward compatibility: a spec with families=() (the default)
        # must produce byte-identical geometry to the pre-families release.
        spec_old_shape = StructureSpec(
            gap_ratio=1.5, skin_margin_mm=1.0, web_position=0.5, rib_pitch_mm=5.0
        )
        result = StructureGenerator().generate(self.frame, spec_old_shape, 0.42, LayerContext(layer_index=0, print_z=0.0))
        xs = sorted(intent.start[0] for intent in result.intents)
        self.assertEqual(xs, [5.0, 10.0, 15.0])
        omitted_xs = sorted(o.x_mm for o in result.omitted)
        self.assertEqual(omitted_xs, [0.0, 20.0])

    def test_two_families_produce_union_of_stations(self):
        shallow = WebFamily(
            name="shallow skin", rib_pitch_mm=4.0, web_depth_mm=1.5, web_position=0.5
        )
        deep = WebFamily(
            name="deep rib", rib_pitch_mm=15.0, web_depth_mm=None, web_position=0.5
        )
        spec = StructureSpec(
            gap_ratio=0.5, skin_margin_mm=1.0, families=(shallow, deep)
        )
        result = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))

        shallow_intents = [i for i in result.intents if i.family_index == 0]
        deep_intents = [i for i in result.intents if i.family_index == 1]
        self.assertTrue(shallow_intents)
        self.assertTrue(deep_intents)
        for i in shallow_intents:
            self.assertEqual(i.family_name, "shallow skin")
            self.assertAlmostEqual(i.start[1] - i.end[1], 1.5)
        for i in deep_intents:
            self.assertEqual(i.family_name, "deep rib")
            # Full depth: end sits at the margin point plus the derived
            # placement headroom, not 1.5 mm in — see
            # FULL_DEPTH_MARGIN_HEADROOM_MM's docstring.
            self.assertAlmostEqual(i.end[1], 1.0 + FULL_DEPTH_MARGIN_HEADROOM_MM)

        # Union: every station either family would produce alone is present.
        only_shallow = StructureGenerator().generate(
            self.frame,
            StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0, families=(shallow,)),
            0.42,
            LayerContext(layer_index=0, print_z=0.0),
        )
        only_deep = StructureGenerator().generate(
            self.frame,
            StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0, families=(deep,)),
            0.42,
            LayerContext(layer_index=0, print_z=0.0),
        )
        expected_xs = sorted(
            [i.start[0] for i in only_shallow.intents]
            + [i.start[0] for i in only_deep.intents]
        )
        actual_xs = sorted(i.start[0] for i in result.intents)
        self.assertEqual(actual_xs, expected_xs)

    def test_omitted_candidates_tagged_with_family(self):
        # gap_ratio=1.5 -> width 0.63mm, half 0.315mm; pitch=5.0 anchored at
        # 10.0 on a 20mm rect omits the two extreme stations (0, 20) for
        # "chord extent" -- see MultiWebGeneratorTests for the same figures
        # against a single family.
        family = WebFamily(name="only", rib_pitch_mm=5.0, web_position=0.5, gap_ratio=1.5)
        spec = StructureSpec(gap_ratio=1.5, skin_margin_mm=1.0, families=(family,))
        result = StructureGenerator().generate(self.frame, spec, 0.42, LayerContext(layer_index=0, print_z=0.0))
        self.assertEqual(len(result.omitted), 2)
        for o in result.omitted:
            self.assertEqual(o.family_index, 0)
            self.assertEqual(o.family_name, "only")


if __name__ == "__main__":
    unittest.main()
