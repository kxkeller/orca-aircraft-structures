"""Fake ``orca`` host used to exec the GENERATED BUNDLE under plain ``unittest``.

WHY THIS EXISTS ---------------- The real OrcaSlicer CLI does not initialize
the plugin subsystem, so nothing can drive
``plugin/aircraft_structures/aircraft_structures.py`` against a real host
automatically. This module fakes just enough of the ``orca`` package's surface
— ``orca.slicing``, ``orca.host``,
``orca.ExecutionResult``/``orca.PluginResult``, ``orca.plugin``/``orca.base``/
``orca.register_capability`` — that the bundle's top-level ``import orca`` and
every attribute it touches at ``execute()`` time resolve, so the capability's
own control flow (arming, step gating, layer walk, root-layer skip, spec
resolution, slot generation, validation, fail-closed error paths, diagnostics)
runs for real. It is not, and does not try to be, a reimplementation of Orca's
Clipper-backed geometry kernel.

DOCUMENTED SCOPE — READ BEFORE TRUSTING A PASSING TEST
--------------------------------------------------------
This fake's ``orca.host`` geometry layer supports **exactly four polygon
shapes**, mirroring ``tests/unit/stub_ops.py``'s documented limits one level
down (in scaled-integer space instead of millimetre floats):

- an axis-aligned rectangle (4 points, 2 distinct x's, 2 distinct y's);
- the U-shaped result of subtracting one top-edge-touching notch from a
  rectangle (the boundary-connected slot shape always produces), which
  carries zero interior holes and is topologically one ring;
- an arbitrary simple polygon (``_Polygon``, stand-in for a real
  cambered section, mirroring ``stub_ops.PolygonShape``) — unlike the other
  shapes, not axis-aligned or convex by construction, just whatever points a
  fixture builder hands it; and
- the analogue of the U-shape, ``_PolygonNotched``: an arbitrary
  ``_Polygon`` minus ONE axis-aligned rectangular clip that cuts in from the
  polygon's own LOCAL upper surface at the clip's own chord station and
  stops short of the LOCAL lower surface there — mirroring
  ``stub_ops.NotchedPolygon`` one level down, including its choice to model
  the notch's cap as flat at the local surface height rather than tracing
  the section's true curved boundary (see KNOWN DIVERGENCES below); and
- the analogue for MORE THAN ONE web, ``_PolygonMultiNotched``: an
  arbitrary ``_Polygon`` minus TWO OR MORE axis-aligned rectangular clips,
  each measured independently against the SAME original polygon (never
  against material another clip already removed) and refused outright if
  their X ranges overlap — see KNOWN DIVERGENCES (17) below.

Anything else — non-axis-aligned polygons, multiple islands, an *interior*
(enclosed) hole, a slot that fully bisects a rectangle (or a section) top-to-
bottom, a clip touching the leading/trailing chordwise extreme, a re-entrant
probe station, two OVERLAPPING notches on the same layer, growth (positive)
offset of a notched shape — is **not implemented** and raises
``NotImplementedError`` with a message naming this limit, exactly as
``stub_ops.py`` does. Do not extend this silently; if a new test needs a
shape this fake does not cover, add it deliberately and re-document the
limit here.

Because every coordinate is a scaled integer, comparisons in this module are
**exact integer comparisons** — no ``EPSILON_MM`` fudge is needed or applied
anywhere in this file (contrast ``stub_ops.py``'s ``EPSILON_MM = 1e-9``,
needed there only because it works in mm floats).

KNOWN DIVERGENCES FROM REAL ORCA — WHERE THIS FAKE COULD GIVE FALSE CONFIDENCE
--------------------------------------------------------------------------------
0. Constructor input types (NOT a divergence; listed so the contract is
   explicit). ``ExPolygon``'s constructor and the ``Polygon`` bindings mirror
   ``PluginHostGeometry.cpp``'s accepted input types: a bound
   ``orca.host.Polygon`` (built via ``Polygon()`` + ``.append(Point)``) or an
   (N,2) int64 ndarray, and nothing else. A raw list of ``Point`` objects
   raises ``ValueError`` here exactly as the real ``parse_polygon``/
   ``as_polygon`` do, so a capability that handed ``ExPolygon`` a bare
   list of ``Point`` (which the real host rejects on the first slice) fails
   in this harness too. Items 1-8 below are the fidelity gaps that remain.
1. ``unscale`` here is pinned to ``SCALING_FACTOR = 1e-6`` always. Real Orca
   switches to ``1e-5`` for beds larger than 2147 mm and reads the live
   factor (the host contract: derive the mm->coord factor at call time
   and never cache it). ``capability.py`` already does this correctly
   (``_scale_factor()`` calls ``orca.slicing.unscale(1)`` fresh each time);
   this fake cannot exercise the large-bed code path at all, so a bug that
   only appears when the factor changes mid-run would pass here undetected.
2. ``ExPolygon.is_valid()`` here only checks ``area > 0``. The real Clipper
   binding's validity check almost certainly also catches self-intersection,
   degenerate/duplicate vertices, and orientation problems that cannot occur
   in shapes this fake can even construct. A capability bug that produces a
   self-intersecting-but-positive-area polygon would look "valid" here and
   would not be caught until a real GUI slice.
3. ``num_contours()`` is 1 for every shape this fake's own operations
   (``diff_ex``/``offset``) can PRODUCE (rectangle or single notch) — no
   boolean here ever creates a hole, so invariant 2's "material has N
   interior holes" failure can still never be reached by cutting through
   this harness. A holed ``ExPolygon`` CAN be directly
   CONSTRUCTED (a single axis-aligned rectangular hole in a
   rectangular outer contour, via ``ExPolygon(contour, holes=[...])`` — see
   ``_Holed``), purely to exercise ``OrcaGeometryOps.hole_bounds``/
   ``hole_points_mm`` against this fake. That construction path is NOT
   wired into ``diff_ex``/``offset`` at all (no scar-cutting boolean
   exists), so no fixture here exercises invariant 2's hole-count failure
   via a real cut; only direct construction is supported.
4. ``orca.host.Surface(surface_type, expolygon)`` here is a faithful,
   complete copy (it only has two fields). Real Orca's binding
   (``PluginHostSlicing.cpp:100-102``) resets ``thickness``,
   ``bridge_angle``, and ``extra_perimeters`` to constructor defaults. This
   fake cannot exercise or regress against that loss because it never models
   those fields at all — a future capability change that started depending
   on them would look correct here and break silently in the real host.
5. ``Layer.make_slices()`` here only increments a per-layer call counter. It
   does **not** recompute an ``lslices`` merged-island view the way the real
   binding does (the real call re-derives lslices, the merged islands, and
   refreshes the bbox cache). Nothing downstream of ``execute()`` reads
   ``lslices`` in this harness, so this is inert here — but it means this
   harness cannot catch a bug where the capability's mutation is correct
   while ``lslices`` still describes stale pre-mutation geometry.
6. ``SurfaceCollection.set()`` here always takes the ``list[Surface]``
   overload (the one the host documentation recommends). The single-overload
   ``set(expolygons, surface_type)`` form is not modeled at all; a
   capability change that accidentally called the retagging overload would
   raise a ``TypeError`` here rather than silently retag everything the way
   it would against the real binding.
7. This fake never simulates the plugin filesystem audit hook
   (the host's audit hook for plugin file access). ``_write_diagnostics`` writes wherever
   ``__file__`` points in the tmp dir; a path-derivation bug that happened
   to still resolve under ``data_dir()`` in the real host but not here (or
   vice versa) would not be caught.
8. Slot polygons produced by ``SlotBuilder`` for this coupon convention
   (X = chord, Y = thickness, web centered on a vertical run) are always
   axis-aligned by construction, so ``polygon_from_points`` never needs to
   handle a rotated quad in these tests. A future spec (angled web,
   ``web_position`` interacting with a non-rectangular section) would need
   real geometry support this fake does not have.
9. ``ExPolygon.offset()`` on a notched (U-shaped) shape still refuses a
   growth (positive ``delta``) offset with ``NotImplementedError`` — this
   mirrors ``tests/unit/stub_ops.py``'s identical, still-present limit
   (a true growth-offset of a non-convex polygon is a real Clipper problem
   this integer rectangle kernel does not attempt to solve). Real Orca's
   Clipper-backed ``ExPolygon.offset()`` handles growth of any polygon,
   convex or not, without restriction. A capability change that relied on
   growing a notched slot polygon would not be exercised by this fake at
   all and would need a real GUI slice to catch a regression there.
10. ``_offset_notched`` (this fake's model of eroding a U-shaped slot
    cross-section) mirrors ``tests/unit/stub_ops.py``'s
    ``_offset_notched_rect`` handling of the severed-island case: when the
    erosion consumes the bridge below the notch but both side arms survive,
    it returns two disjoint pieces rather than collapsing to "nothing
    survives". This matters here specifically because
    ``TopologyValidator``'s connectivity check in ``aircraft_structures.py``
    (the "slot N disconnects material island" diagnosis) fires exactly on
    ``len(eroded) > 1`` from ``ops.offset()``. Without the two-piece
    result, notched erosion could only return 0 or 1 pieces, so
    ``FailClosedDisconnectionTests.test_thin_bridge_raises_fatal_error_with_layer_and_z``
    could never reach the ``len(eroded) > 1`` branch through this
    integration harness — every thin-bridge fixture would be misdiagnosed as
    "section is entirely thinner than the safety offset", hiding the defect
    class that test exists to catch. ``stub_ops.py`` has the same two-piece
    behaviour one level down.
11. ``ExPolygon.intersection_ex`` is exact — not approximate — for every
    subject/clip combination this fake's DOCUMENTED SCOPE admits: a plain
    rectangle or a top-notched (U-shaped) subject against a rectangle clip
    (the measurement-strip probe shape ``aircraft_core.frame.local_span``
    always builds). It is computed by closed-form interval arithmetic
    (``_intersect_rect_rect``/``_intersect_notched_rect``), including the
    disjoint-arms case (two ``ExPolygon`` returned), never by a general
    polygon-clipping algorithm. What is NOT faithful: (a) the real
    Clipper-backed ``intersection_ex`` accepts and correctly clips ANY two
    ``ExPolygon`` — non-axis-aligned, non-convex, multi-contour, with
    holes; this fake only accepts a rectangle clip against a rectangle or
    single-top-notch subject and raises ``NotImplementedError`` for
    anything else (notably a notched *clip*, or either operand being some
    other shape this fake cannot even construct), so a capability change
    that started intersecting two non-rectangle sections, or clipping with
    a non-rectangular window, would not be exercised here at all and would
    need a real GUI slice to catch a regression; (b) the real binding can
    return contours whose outline is a general polygon (e.g. a clipped
    corner), while every non-empty result this fake produces is itself
    only ever a ``_Rect`` or a ``_Notched`` — exact for area()/bounds(),
    which is all ``local_span`` ever reads, but not a general-purpose
    outline. This mirrors ``tests/unit/stub_ops.py``'s ``StubOps.intersection``
    one level down, including its documented reason for existing: naive
    single-ring clipping of a subject whose true intersection with the clip
    is disconnected does not fail loudly, it silently produces a
    self-intersecting ring with a wrong bounds box and area. This fake
    sidesteps that failure mode entirely (rather than detecting-and-guarding
    against it the way ``stub_ops.py`` does) because its shape kernel is
    restrictive enough to decompose the notched case into independent
    column bands by construction.

12. ``_Polygon``/``_PolygonNotched`` (cambered-section shapes) are
    computed by closed-form scanline/half-plane arithmetic
    (``_scanline_y_intervals``, ``_clip_polygon_to_rect_int``,
    ``_diff_polygon_rect``, ``_offset_polygon_notched``), never by a general
    polygon-Boolean engine — mirroring ``stub_ops.py``'s identical restraint
    for ``PolygonShape``/``NotchedPolygon``. What is NOT faithful: (a) the
    real Clipper-backed ``diff_ex``/``intersection_ex`` handle ANY polygon
    against ANY clip; this fake's polygon ``diff_ex`` accepts only an
    axis-aligned rectangular clip that is strictly interior in X (not
    touching the leading/trailing chordwise extreme), does not reach the
    local lower surface, and does reach (or exceed) the local upper surface
    — a clip touching a chordwise extreme, bisecting the section, landing
    short of the boundary, or crossing a re-entrant column all raise
    ``NotImplementedError`` naming the limit rather than being attempted;
    (b) ``_diff_polygon_rect``'s resulting notch is the EXACT clip/material
    overlap (computed via ``_intersect_polygon_rect``, the same machinery
    ``intersection_ex`` uses) — it traces the section's true curved boundary
    within the clip's footprint, not a flat cap at a single-point probe. A
    flat cap at the local upper-surface height (measured once, at the
    clip's own X midpoint) is NOT used, because on a sloped surface, a
    single-point scanline probe measures slightly LOWER than
    ``aircraft_core.frame.local_span``'s own finite-width strip probe (which
    can catch a marginally higher point near the strip's edge), and that
    gap — a few tenths of a micron on the NACA 2412 fixture at 0.7 chord,
    measured empirically — is large enough
    relative to ``TopologyValidator``'s ``EPSILON_MM = 1e-6`` mm tolerance
    to spuriously fail invariant 3 for a legitimately-placed slot. This is
    unlike ``stub_ops.NotchedPolygon``, whose flat-rectangle notch is
    supplied directly by the TEST as already-known-correct ground truth
    (never computed by a diff operation at all) — this fake's ``diff_ex``
    has no such luxury, since a real capability run actually calls it, so
    it needed the exact overlap instead;
    (c) ``_intersect_polygon_rect`` (this fake's ``intersection_ex`` for a
    polygon subject) mirrors ``stub_ops.StubOps.intersection``'s two-tier
    strategy exactly: a scanline probe at the clip's own X midpoint first
    counts how many disjoint Y-intervals of material exist there; exactly
    one gets an EXACT Sutherland-Hodgman clip (rounded to the nearest scaled
    integer, unavoidable for a non-axis-aligned polygon edge in integer
    coordinate space — this rounding is a real Clipper property too, not a
    shortcut unique to this fake); more than one is the DISCONNECTED case
    the case that makes naive single-ring clipping wrong, and is never stitched into one self-intersecting
    ring — each surviving Y-interval is returned as its own separate
    ``_Rect`` piece spanning the clip's X range, exact for bounds()/area()
    at this probe (all ``local_span`` ever reads from a piece) but NOT a
    general clipping result, mirroring both ``stub_ops.py``'s and this
    file's own ``_intersect_notched_rect``'s identical multi-piece
    convention.
13. ``_offset_polygon_notched`` (this fake's connectivity-only erosion model
    for a cut cambered section) answers exactly the question
    ``TopologyValidator``'s safety-offset check consumes — "does this remain
    one piece, or collapse entirely" — and nothing about the true eroded
    outline, exactly as documented for ``_offset_notched``/
    ``stub_ops._offset_notched_polygon``. It goes ONE step further than
    ``stub_ops._offset_notched_polygon``, which only ever recognizes "the
    bridge clearly survives" and raises ``NotImplementedError`` for
    anything else: this fake ALSO recognizes full LOCAL-envelope collapse
    (the section's own material width at the notch's chord station, not the
    section's global bounding-box depth — the same local-vs-bbox
    distinction the capability's local-thickness measurement exists to make) failing to
    survive erosion, returning ``[]`` and letting
    ``TopologyValidator`` raise its "section is entirely thinner than the
    safety offset" ``TopologyError``. This is needed to reach that
    branch through this integration harness at all — the SEVERED-but-not-
    fully-collapsed case (bridge fails while the local envelope still
    survives) is NOT modelled and still raises ``NotImplementedError``,
    same restraint as ``stub_ops.py``. Provably, for any spec that satisfies
    ``TopologyValidator``'s own documented precondition
    (``skin_margin_mm > 2 * outer_wall_line_width``, required for ANY layer
    to ever pass at all) and for which ``frame.web_centerline`` did not
    already refuse to place the web (local thickness > skin_margin_mm), the
    local envelope always exceeds ``skin_margin_mm`` which always exceeds
    ``2 * outer_wall_line_width`` — so this specific collapse branch can
    never actually fire for a legitimately-generated web either; a
    thinning trailing edge fails earlier, in ``frame.web_centerline``
    itself (see ``test_airfoil_capability.py``'s trailing-edge test and its
    accompanying finding about ``capability.py`` not wrapping that
    ``ValueError`` with layer/Z context).
14. ``make_airfoil_object`` builds every layer from the SAME NACA 2412
    outline (loaded once from ``tools/make_fixtures.py``, not regenerated
    per layer) — a genuinely constant-section stack, matching the
    constant-section scope of this fixture. It does not model taper, sweep, twist, or a
    thinning tip, so this harness cannot exercise or regress against
    those even once the real capability grows support for them.
15. This fake's polygon SLICE shapes (``_Rect``/``_Polygon``/etc, what
    ``LayerRegion.slices`` exposes) never carry the fixture's own Z (span)
    coordinate — like ``make_rect_object``, ``make_airfoil_object`` uses the
    SAME 2D outline for every layer's ``print_z``, so a bug that only shows
    up when the cross-section genuinely changes between layers (any
    non-constant-section geometry) is out of reach of THAT part of this
    harness by construction, not just by the current fixture's choice of
    parameters. This limit applies only to the 2D slice
    shapes, not to layers/meshes overall -- ``Layer.slice_z`` and the
    attached ``model_object``'s ``TriangleMesh`` are both genuinely
    Z-varying (``make_tapered_naca_object`` builds a real tapered loft
    mesh), so a taper bug reachable only through Z-varying MESH geometry is
    within this harness's reach; only the non-mesh 2D slice representation
    keeps this limit.
16. **This fake cannot reproduce the centreline-vs-removed-region-centre
    disagreement an in-host NACA 2412 slice found** (the bug
    that ``frame.local_lower_worst_case``/``local_upper_worst_case`` and
    ``validate.py``'s span-based re-measurement address). ``_diff_polygon_rect``
    (above) sets the removed notch to the EXACT clip/material overlap via
    ``_intersect_polygon_rect``, whose clip rectangle is, by construction,
    centred on the same X the capability placed the web at
    (``SlotBuilder.build`` — see aircraft_core/slot.py — always builds a
    slot symmetric about its centreline). Clipping a polygon against an
    X-symmetric rectangle produces a result whose own bounding box is, for
    every fixture this harness has, ALSO X-symmetric about that same
    centre — so ``validate.py``'s
    ``(removed_min_x + removed_max_x) / 2`` recovers exactly the station
    ``web_centerline`` already measured, and the two never disagree here
    even without those worst-case helpers. On real Clipper-backed geometry the removed
    region's bounding box is not guaranteed to be exactly symmetric about
    the clip rectangle's own centre (integer-coordinate rounding of a
    non-axis-aligned boundary edge, and the fact that the exact overlap
    polygon's OWN bbox is a property of where the curve happens to cross
    the clip's edges, not of the clip's symmetry) — this fake's simplified
    integer-scanline-and-Sutherland-Hodgman kernel is not known to exhibit
    that asymmetry, and no attempt is made here to force it to. This
    divergence is noted rather than faked: introducing an artificial
    asymmetry into this fake's clip kernel just to manufacture a failure
    would be testing the fake's own arithmetic, not anything about
    ``aircraft_core``. The fix is still exercised for real by
    ``tests/unit/test_local_geometry.py``'s
    ``WorstCaseAcrossSlotWidthTests`` (the worst-case-vs-point-probe
    mechanism, with real numbers on the real NACA 2412 section) and
    ``tests/unit/test_validate.py``'s
    ``test_correctly_placed_slot_passes_all_invariants`` (the placement ->
    validation round trip that failed in-host), both of which drive
    ``aircraft_core`` directly rather than through this fake.

17. ``_PolygonMultiNotched``/``_add_polygon_notch``/
    ``_offset_polygon_multi_notched`` (more than one web on a cambered
    section) extend the single-notch machinery to N notches by
    REJECTING two clips whose X ranges overlap, rather than attempting to
    represent that geometry at all. The real Clipper-backed
    ``diff_ex``/``offset`` have no such restriction — a genuinely
    overlapping pair of slot clips is valid input to them, just not to this
    fake's column-based kernel. This is not a gap this fake papers over
    silently: a well-formed capability run should never hand two
    overlapping slots to ``ops.diff`` in the first place, because
    ``aircraft_core.generator``'s anchoring rule spaces webs by
    ``rib_pitch_mm`` and ``aircraft_core.validate``'s collision check
    fails the layer closed before an undersized (let alone overlapping)
    gap between two slots would ever reach a real cut — so this fake's
    refusal exercises the "should never happen" boundary rather than a
    realistic capability behaviour this harness needs to tolerate.
    ``_offset_polygon_multi_notched`` additionally assumes the
    severed-island case (one notch's bridge failing to survive erosion
    while the section's local envelope elsewhere still does) is out of
    scope, mirroring ``_offset_polygon_notched``'s identical single-notch
    limit — not modelled for N > 1 notches either.

18. **This fake cannot cut an carbon-spar SOCKET at all.**
    ``aircraft_core.slot.SlotBuilder._keyhole`` builds a ~74-point polygon
    (a neck plus a circular arc) for any ``WebIntent`` carrying
    ``socket_radius_mm``, and ``ExPolygon``'s raw-points constructor
    (``_classify_rect``) accepts ONLY a 4-point axis-aligned rectangle (see
    "DOCUMENTED SCOPE" above) — so ``OrcaGeometryOps.polygon_from_points``
    raises ``NotImplementedError`` the moment a socket-bearing intent is
    built, before any cut is attempted. This is a scope limit of the fake (it is
    not a general polygon-Boolean engine). It becomes reachable whenever a
    channel is detected, since a detected channel is what causes
    ``StructureGenerator`` to emit a socket-bearing intent. ``tests/integration/test_m18_mesh_transform.py``'s
    channel-detection tests therefore stop at
    ``capability._mesh_alignment``'s own per-layer detection fields
    (populated by the real ``_channels_for_layer`` call BEFORE the socket
    cut is attempted) rather than asserting on a cut result; the real
    detection -> socket-centring chain, with an actual boolean cut, is
    proven separately, end-to-end, against real STL fixtures and the real
    Shapely-backed ``GeometryOps``, by
    ``test_m18_model_determinism.py``.

Nothing here reuses Orca's real coordinate scaling edge cases, real Clipper
robustness, or real perimeter/Spiral-Vase behaviour. Everything downstream of
``slices.set()`` is verified only by a human-run GUI slice and the physical
print.
"""

from dataclasses import dataclass
import math
import types
from typing import List, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Scaling
# ---------------------------------------------------------------------------

SCALING_FACTOR = 1e-6  # see module docstring divergence (1)


def unscale(v):
    return v * SCALING_FACTOR


def to_scaled(mm):
    """Millimetres -> scaled int, using the same round-half-to-even ``round``
    the real capability's ``_to_scaled`` uses. Exposed for test fixtures so
    expected geometry can be derived the same way the code under test derives
    it, instead of hand-computing a second, potentially inconsistent, float
    formula."""
    return int(round(mm / SCALING_FACTOR))


# ---------------------------------------------------------------------------
# Internal shape kernel — rectangle and single top-notch only. See module
# docstring "DOCUMENTED SCOPE".
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _Rect:
    min_x: int
    min_y: int
    max_x: int
    max_y: int

    def area(self) -> int:
        return max(0, self.max_x - self.min_x) * max(0, self.max_y - self.min_y)

    def points(self):
        return [
            (self.min_x, self.min_y),
            (self.max_x, self.min_y),
            (self.max_x, self.max_y),
            (self.min_x, self.max_y),
        ]


@dataclass(frozen=True)
class _Holed:
    """``outer`` minus ``hole``, where ``hole`` touches none of the four
    outer edges — mirrors ``stub_ops.HoledRect`` one level down.

    Lets this fake represent a holed ``ExPolygon`` (see divergence (3) in the
    module docstring). Like ``HoledRect``, this is ONLY ever constructed
    directly (never produced by ``diff_ex``/``offset`` — this fake has no
    hole-producing boolean); it exists
    purely so a holed polygon can be represented and its hole geometry queried
    through ``OrcaGeometryOps.hole_bounds``/``hole_points_mm``.
    """

    outer: _Rect
    hole: _Rect

    def area(self) -> int:
        return self.outer.area() - self.hole.area()

    def points(self):
        return self.outer.points()


@dataclass(frozen=True)
class _Notched:
    outer: _Rect
    notch: _Rect  # notch.max_y == outer.max_y (touches the +Y/top boundary)

    def area(self) -> int:
        return self.outer.area() - self.notch.area()

    def left_arm(self) -> int:
        return self.notch.min_x - self.outer.min_x

    def right_arm(self) -> int:
        return self.outer.max_x - self.notch.max_x

    def bottom_bridge(self) -> int:
        return self.notch.min_y - self.outer.min_y

    def points(self):
        o, n = self.outer, self.notch
        return [
            (o.min_x, o.min_y),
            (o.max_x, o.min_y),
            (o.max_x, o.max_y),
            (n.max_x, o.max_y),
            (n.max_x, n.min_y),
            (n.min_x, n.min_y),
            (n.min_x, o.max_y),
            (o.min_x, o.max_y),
        ]


@dataclass(frozen=True)
class _Polygon:
    """An arbitrary simple (non-self-intersecting) polygon ring of scaled-int
    ``(x, y)`` points — stand-in for a real cambered section, mirroring
    ``stub_ops.PolygonShape`` one level up. Unlike ``_Rect``/``_Notched``,
    makes no assumption about axis-alignment or convexity. Always constructed
    directly (``ExPolygon._wrap``) by a fixture builder — never parsed from raw
    points via ``_classify_rect``, which stays rectangle-only (see that
    function's own docstring)."""

    pts: Tuple[Tuple[int, int], ...]

    def area(self) -> float:
        n = len(self.pts)
        total = 0
        for i in range(n):
            x0, y0 = self.pts[i]
            x1, y1 = self.pts[(i + 1) % n]
            total += x0 * y1 - x1 * y0
        return abs(total) / 2.0

    def bounds(self) -> Tuple[int, int, int, int]:
        xs = [p[0] for p in self.pts]
        ys = [p[1] for p in self.pts]
        return (min(xs), min(ys), max(xs), max(ys))

    def points(self):
        return list(self.pts)


@dataclass(frozen=True)
class _PolygonNotched:
    """``outer`` (a ``_Polygon``) minus ``notch`` — analogue of ``_Notched``,
    mirroring ``stub_ops.NotchedPolygon``.

    ``notch`` is the EXACT clip/material overlap (a ``_Polygon`` tracing the
    section's true curved boundary within the clip's footprint, or a
    ``_Rect`` in the degenerate case that overlap happens to be
    axis-aligned) — computed via ``_intersect_polygon_rect``, the same exact
    machinery ``intersection_ex`` uses, NOT a flat rectangular cap at a
    single-point probe. See ``_diff_polygon_rect``'s docstring for why a
    flat cap was tried first and rejected (module docstring KNOWN
    DIVERGENCES (12)).
    """

    outer: _Polygon
    notch: object  # _Polygon or _Rect — the exact clip/material overlap

    def area(self) -> float:
        return self.outer.area() - self.notch.area()

    def points(self):
        raise NotImplementedError(
            "fake orca.host.ExPolygon: the exact outline of a "
            "polygon-minus-slot (cambered section minus a web slot) shape is "
            "never computed — nothing in the real capability control flow "
            "reads .contour on a cut section (bounds() is only ever read on "
            "the ORIGINAL polygon and on the removed notch rectangle, both "
            "of which have their own .points()); reproducing the true "
            "concave outline here would need the general polygon-Boolean "
            "engine this fake deliberately does not build. See module "
            "docstring 'DOCUMENTED SCOPE'."
        )


@dataclass(frozen=True)
class _PolygonMultiNotched:
    """``outer`` (a ``_Polygon``) minus TWO OR MORE notches — analogue of
    ``_PolygonNotched``, added for multiple parallel webs on a cambered
    section. Each entry of ``notches`` is the EXACT clip/material overlap for
    that slot (same convention as ``_PolygonNotched.notch``), computed
    independently against the ORIGINAL ``outer`` polygon at that slot's own
    chord station — correct because webs never overlap in X (enforced when a
    notch is added; see ``_add_polygon_notch``), so one slot's cut never
    touches material another slot's cut also claims.

    Like ``_PolygonNotched``, this is never asked for its true concave
    outline (``.points()`` still raises) — nothing in the real capability
    control flow needs it; see that class's docstring for the full
    reasoning, which applies unchanged here.
    """

    outer: _Polygon
    notches: Tuple[object, ...]  # each a _Polygon or _Rect exact overlap

    def area(self) -> float:
        return self.outer.area() - sum(n.area() for n in self.notches)

    def points(self):
        raise NotImplementedError(
            "fake orca.host.ExPolygon: the exact outline of a "
            "polygon-minus-multiple-slots shape is never computed — see "
            "_PolygonNotched.points()'s identical reasoning, unchanged for "
            "more than one slot. See module docstring 'DOCUMENTED SCOPE'."
        )


def _shape_bounds(shape):
    """Bounds for a ``_Rect`` or ``_Polygon`` — the two shapes
    ``_PolygonNotched.notch`` can hold (the exact clip/material overlap)."""
    if isinstance(shape, _Rect):
        return (shape.min_x, shape.min_y, shape.max_x, shape.max_y)
    return shape.bounds()


def _scanline_y_intervals(pts: Sequence[Tuple[int, int]], x_probe: int):
    """Y-intervals where the simple polygon ``pts`` has material at
    ``x = x_probe``, via ray-casting: collect the Y of every edge crossing
    (rounded to the nearest scaled integer — unavoidable for a
    non-axis-aligned edge in integer coordinate space, a real Clipper
    property too), sort, and pair consecutive crossings into filled
    intervals. Mirrors ``stub_ops._vertical_scanline_intervals`` exactly,
    one level down (scaled ints instead of mm floats).

    Raises ``ValueError`` if the probe grazes a vertex (odd crossing count)
    — callers must choose a non-degenerate station, same as any ray-casting
    point-in-polygon test.
    """
    ys = []
    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        if x0 == x1:
            continue
        lo, hi = (x0, x1) if x0 <= x1 else (x1, x0)
        if lo <= x_probe <= hi:
            t = (x_probe - x0) / (x1 - x0)
            ys.append(int(round(y0 + t * (y1 - y0))))
    ys.sort()
    if len(ys) % 2 != 0:
        raise ValueError(
            "fake orca.host: probe x={!r} grazes a polygon vertex (odd "
            "crossing count {}); choose a different chord station.".format(
                x_probe, len(ys)
            )
        )
    return [(ys[i], ys[i + 1]) for i in range(0, len(ys), 2)]


def _polygon_local_y_range(shape: _Polygon, x_probe: int):
    """The single (y_min, y_max) interval of ``shape`` at ``x_probe``.

    Raises ``ValueError`` if there is no material there, ``NotImplementedError``
    if the station is re-entrant (more than one interval) — this fake's
    diff_ex/offset only ever answer "the" local upper/lower surface, same
    restriction ``aircraft_core.frame.local_span`` documents.
    """
    intervals = _scanline_y_intervals(shape.pts, x_probe)
    if not intervals:
        raise ValueError(
            "fake orca.host: no material at chord station x={!r} (scaled "
            "units).".format(x_probe)
        )
    if len(intervals) > 1:
        raise NotImplementedError(
            "fake orca.host: chord station x={!r} crosses the polygon's "
            "material {} separate times; re-entrant sections are not "
            "supported by this fake's diff_ex/offset. See module docstring "
            "'DOCUMENTED SCOPE'.".format(x_probe, len(intervals))
        )
    return intervals[0]


def _compute_polygon_notch(outer: _Polygon, clip: _Rect):
    """Shared validation + exact-overlap computation for ONE clip against
    ``outer`` — the core of ``_diff_polygon_rect``, factored out so an earlier
    rule's ``_add_polygon_notch`` (a second, third... clip against the SAME
    ``outer``) can reuse it exactly rather than duplicating the validation.
    Exact ONLY for "a rectangular clip cutting in from the subject's own local
    upper surface at the clip's chord station, strictly interior in X, stopping
    short of the local lower surface". See DOCUMENTED SCOPE for what is refused
    outright.

    The returned notch is the EXACT clip/material overlap
    (``_intersect_polygon_rect``, the same machinery ``intersection_ex``
    uses), not a flat rectangular cap at a single-point probe. A flat cap
    at the local upper-surface height measured only at the clip's X
    midpoint was tried first and rejected: on a sloped section, that
    single-point probe can measure a few tenths-of-a-micron LOWER than
    ``aircraft_core.frame.local_span``'s own finite-width strip probe (which
    can catch a slightly higher point near the strip's edge where the
    boundary is sloped), making the flat-capped notch appear to fall short
    of the exterior boundary by more than ``EPSILON_MM`` and spuriously
    fail invariant 3 even for a legitimately-placed slot. The exact overlap
    does not have this problem: it traces the section's own true boundary
    inside the clip's full width, so it is at least as high as any
    finite-width probe centred in that same window could ever measure.
    """
    ox0, _oy0, ox1, _oy1 = outer.bounds()
    if clip.min_x <= ox0 or clip.max_x >= ox1:
        raise NotImplementedError(
            "fake orca.host.ExPolygon.diff_ex(polygon, rect): clip x-range "
            "({!r}, {!r}) touches or exceeds the section's own chordwise "
            "bounding-box edge ({!r}, {!r}); only a clip strictly interior "
            "in X (a web placed away from the leading/trailing tip) is "
            "supported. See module docstring 'DOCUMENTED SCOPE'.".format(
                clip.min_x, clip.max_x, ox0, ox1
            )
        )
    x_mid = (clip.min_x + clip.max_x) // 2
    local_y0, local_y1 = _polygon_local_y_range(outer, x_mid)

    touches_top = clip.max_y >= local_y1
    touches_bottom = clip.min_y <= local_y0

    if touches_bottom:
        raise NotImplementedError(
            "fake orca.host.ExPolygon.diff_ex(polygon, rect): clip reaches "
            "the LOCAL lower surface (y={!r}) at chord station x={!r} — a "
            "cut that bisects the section top-to-bottom is not supported. "
            "See module docstring 'DOCUMENTED SCOPE'.".format(local_y0, x_mid)
        )
    if not touches_top:
        raise NotImplementedError(
            "fake orca.host.ExPolygon.diff_ex(polygon, rect): clip (max_y="
            "{!r}) does not reach the LOCAL upper surface (y={!r}) at chord "
            "station x={!r} — this would leave an interior void, not a "
            "boundary-connected slot, which is not supported. See module "
            "docstring 'DOCUMENTED SCOPE'.".format(clip.max_y, local_y1, x_mid)
        )

    overlap = _intersect_polygon_rect(outer, clip)
    if not overlap:
        raise NotImplementedError(
            "fake orca.host.ExPolygon.diff_ex(polygon, rect): clip at chord "
            "station x={!r} classified as touching the exterior boundary, "
            "but its exact material overlap is empty — this should not "
            "happen for a well-formed slot; refusing rather than guessing.".format(
                x_mid
            )
        )
    if len(overlap) > 1:
        raise NotImplementedError(
            "fake orca.host.ExPolygon.diff_ex(polygon, rect): clip's exact "
            "material overlap at chord station x={!r} is DISCONNECTED ({} "
            "pieces) — a re-entrant clip footprint is not supported. See "
            "module docstring 'DOCUMENTED SCOPE'.".format(x_mid, len(overlap))
        )

    return overlap[0]


def _diff_polygon_rect(subject: _Polygon, clip: _Rect):
    """``subject`` (an untouched polygon) minus ``clip`` — the FIRST notch.
    See ``_compute_polygon_notch`` for the validation and exact-overlap
    machinery this wraps.
    """
    return [_PolygonNotched(outer=subject, notch=_compute_polygon_notch(subject, clip))]


def _add_polygon_notch(shape, clip: _Rect):
    """``shape`` (a ``_PolygonNotched`` or ``_PolygonMultiNotched`` — i.e. already
    carrying one or more notches) minus ANOTHER ``clip`` — second, third... web
    on the same cambered section.

    Each notch is measured independently against the shape's own ORIGINAL
    ``outer`` polygon at the new clip's chord station (never against
    already-notched material), via the identical ``_compute_polygon_notch``
    machinery ``_diff_polygon_rect`` uses for the first notch — correct because
    webs never overlap in X, which is enforced here before the new notch is
    even computed: a clip whose X range overlaps an EXISTING notch's X range is
    refused outright (a genuinely overlapping pair of slots is not a shape this
    fake's column-based kernel can represent; the real validator's collision
    check exists precisely so a well-formed capability run never asks this fake
    to attempt it — see aircraft_core/validate.py's collision check).
    """
    if isinstance(shape, _PolygonNotched):
        outer = shape.outer
        existing = (shape.notch,)
    elif isinstance(shape, _PolygonMultiNotched):
        outer = shape.outer
        existing = shape.notches
    else:
        raise NotImplementedError(
            "fake orca.host.ExPolygon.diff_ex: _add_polygon_notch expects a "
            "_PolygonNotched or _PolygonMultiNotched subject; got {!r}.".format(
                type(shape)
            )
        )

    for n in existing:
        n_x0, _n_y0, n_x1, _n_y1 = _shape_bounds(n)
        if not (clip.max_x <= n_x0 or clip.min_x >= n_x1):
            raise NotImplementedError(
                "fake orca.host.ExPolygon.diff_ex(polygon, rect): new clip "
                "x-range ({!r}, {!r}) overlaps an existing notch's x-range "
                "({!r}, {!r}) — two overlapping slots on the same section "
                "are not representable by this fake's column-based kernel. "
                "A well-formed capability run should never reach this: see "
                "aircraft_core/validate.py's wall-collision check. See module "
                "docstring 'DOCUMENTED SCOPE'.".format(
                    clip.min_x, clip.max_x, n_x0, n_x1
                )
            )

    new_notch = _compute_polygon_notch(outer, clip)
    all_notches = tuple(
        sorted(existing + (new_notch,), key=lambda n: _shape_bounds(n)[0])
    )
    return [_PolygonMultiNotched(outer=outer, notches=all_notches)]


def _clip_polygon_to_rect_int(points: Sequence[Tuple[int, int]], clip: _Rect):
    """Sutherland-Hodgman clip of an arbitrary polygon against an
    axis-aligned rectangle, in scaled-int coordinates. Because the clip is
    axis-aligned, each of its four edges is a trivial vertical/horizontal
    half-plane clip (no general line-line intersection needed); crossing
    points are rounded to the nearest scaled integer, exactly as a real
    integer-coordinate Boolean kernel must. EXACT for a subject whose true
    intersection with ``clip`` is a single connected region — see
    ``_intersect_polygon_rect``, which checks that precondition before
    calling this."""

    def clip_edge(pts, keep, cross):
        if not pts:
            return []
        out = []
        n = len(pts)
        for i in range(n):
            curr = pts[i]
            prev = pts[i - 1]
            curr_in = keep(curr)
            prev_in = keep(prev)
            if curr_in:
                if not prev_in:
                    out.append(cross(prev, curr))
                out.append(curr)
            elif prev_in:
                out.append(cross(prev, curr))
        return out

    def cross_x(prev, curr, x_at):
        x0, y0 = prev
        x1, y1 = curr
        if x1 == x0:
            return (x_at, y0)
        t = (x_at - x0) / (x1 - x0)
        return (x_at, int(round(y0 + t * (y1 - y0))))

    def cross_y(prev, curr, y_at):
        x0, y0 = prev
        x1, y1 = curr
        if y1 == y0:
            return (x0, y_at)
        t = (y_at - y0) / (y1 - y0)
        return (int(round(x0 + t * (x1 - x0))), y_at)

    out = list(points)
    out = clip_edge(out, lambda p: p[0] >= clip.min_x, lambda a, b: cross_x(a, b, clip.min_x))
    out = clip_edge(out, lambda p: p[0] <= clip.max_x, lambda a, b: cross_x(a, b, clip.max_x))
    out = clip_edge(out, lambda p: p[1] >= clip.min_y, lambda a, b: cross_y(a, b, clip.min_y))
    out = clip_edge(out, lambda p: p[1] <= clip.max_y, lambda a, b: cross_y(a, b, clip.max_y))
    return out


def _intersect_polygon_rect(subject: _Polygon, clip: _Rect):
    """``subject`` (an arbitrary polygon) intersected with an axis-aligned
    ``clip`` rectangle — the "section clipped to a measurement strip" case
    ``aircraft_core.frame.local_span`` always builds. Mirrors
    ``stub_ops.StubOps.intersection``'s two-tier strategy exactly (module
    docstring KNOWN DIVERGENCES (12)(c)): a scanline probe at the clip's own
    X midpoint counts disjoint Y-intervals of material there BEFORE trusting
    a single-ring clip, so a genuinely DISCONNECTED result (a re-entrant
    probe) is detected structurally and returned as separate pieces rather
    than silently stitched into one self-intersecting ring."""
    x_mid = (clip.min_x + clip.max_x) // 2
    intervals = _scanline_y_intervals(subject.pts, x_mid)

    clipped = []
    for y0, y1 in intervals:
        lo = max(y0, clip.min_y)
        hi = min(y1, clip.max_y)
        if hi > lo:
            clipped.append((lo, hi))

    if not clipped:
        return []
    if len(clipped) == 1:
        exact = _clip_polygon_to_rect_int(subject.pts, clip)
        if len(exact) < 3:
            return []
        return [_Polygon(pts=tuple(exact))]

    # DISCONNECTED: more than one separate Y-interval of material inside the
    # clip. NOT stitched into one ring — see module docstring KNOWN
    # DIVERGENCES (12)(c) for exactly what this multi-piece result is (and
    # is not) exact for.
    return [_Rect(clip.min_x, y0, clip.max_x, y1) for y0, y1 in clipped]


def _offset_polygon_notched(shape: _PolygonNotched, delta: int):
    """Connectivity-only erosion signal for a ``_PolygonNotched`` — see
    module docstring KNOWN DIVERGENCES (13) for exactly what this does and
    does not model, and why it goes one step further than
    ``stub_ops._offset_notched_polygon``."""
    if delta > 0:
        raise NotImplementedError(
            "fake orca.host.ExPolygon.offset: growing (positive delta) a "
            "polygon-minus-slot shape is not needed and is not implemented. "
            "See module docstring 'DOCUMENTED SCOPE'."
        )
    shrink = -2 * delta
    notch_min_x, notch_min_y, notch_max_x, _notch_max_y = _shape_bounds(shape.notch)
    x_mid = (notch_min_x + notch_max_x) // 2
    local_y0, local_y1 = _polygon_local_y_range(shape.outer, x_mid)
    local_thickness = local_y1 - local_y0

    # Local-envelope collapse: the section's own material width at THIS
    # chord station (not the section's global bounding-box depth) is what a
    # uniform erosion narrows first here — see KNOWN DIVERGENCES (13).
    if local_thickness - shrink <= 0:
        return []

    bridge = notch_min_y - local_y0
    if bridge - shrink <= 0:
        raise NotImplementedError(
            "fake orca.host.ExPolygon.offset(polygon-notched, delta={!r}): "
            "the bridge beneath the notch ({!r} scaled units) does not "
            "clearly survive erosion, though the section's local envelope "
            "does. Reproducing the severed-island outline for an arbitrary "
            "polygon is a real non-convex-erosion problem this fake does "
            "not attempt (mirrors tests/unit/stub_ops.py's identical limit "
            "for NotchedPolygon). See module docstring.".format(delta, bridge)
        )
    return [shape]


def _offset_polygon_multi_notched(shape: _PolygonMultiNotched, delta: int):
    """Connectivity-only erosion signal for a ``_PolygonMultiNotched`` (more than
    one web on a cambered section) — the same question
    ``_offset_polygon_notched`` answers for one notch, checked independently at
    EVERY notch's own chord station.

    Mirrors ``tests/unit/stub_ops.py``'s ``_offset_multi_notched_rect``
    reasoning one level down: if every notch's LOCAL envelope and LOCAL bridge
    (beneath that specific notch, at that specific chord station — not a
    shared/uniform assumption the way the flat-rectangle stub can get away
    with, since a cambered section's thickness genuinely varies between web
    stations) clearly survive the erosion, the whole shape stays one connected
    island, because the material beneath every notch remains part of the SAME
    lower surface regardless of how thin the material directly between two
    notches (above their shared bridge region) is — that inter-notch collapse
    is exactly what separate collision check (aircraft_core/validate.py) exists
    to catch before erosion is ever reached; this offset model is not expected
    to (and does not attempt to) catch it. If ANY notch's local envelope or
    bridge fails to clearly survive, this raises rather than guessing — the
    severed/consumed multi-notch case is out of scope for this fake, exactly as
    it is for ``_offset_polygon_notched``'s single-notch form.
    """
    if delta > 0:
        raise NotImplementedError(
            "fake orca.host.ExPolygon.offset: growing (positive delta) a "
            "polygon-minus-multiple-slots shape is not needed and is not "
            "implemented. See module docstring 'DOCUMENTED SCOPE'."
        )
    shrink = -2 * delta
    for notch in shape.notches:
        notch_min_x, notch_min_y, notch_max_x, _notch_max_y = _shape_bounds(notch)
        x_mid = (notch_min_x + notch_max_x) // 2
        local_y0, local_y1 = _polygon_local_y_range(shape.outer, x_mid)
        local_thickness = local_y1 - local_y0

        if local_thickness - shrink <= 0:
            return []

        bridge = notch_min_y - local_y0
        if bridge - shrink <= 0:
            raise NotImplementedError(
                "fake orca.host.ExPolygon.offset(polygon-multi-notched, "
                "delta={!r}): the bridge beneath a notch at chord station "
                "x={!r} ({!r} scaled units) does not clearly survive "
                "erosion, though the section's local envelope there does. "
                "Reproducing the severed-island outline for more than one "
                "notch is out of scope for this fake. See module "
                "docstring.".format(delta, x_mid, bridge)
            )
    return [shape]


def _classify_rect(pts: Sequence[Tuple[int, int]]) -> _Rect:
    """Turn a raw 4-point ring into a ``_Rect``, or raise loudly.

    Only used for polygons constructed from *raw points* — i.e.
    ``polygon_from_points`` slot clips and test-fixture layer rectangles.
    Everything this fake's own operations (``diff_ex``/``offset``) produce is
    tagged directly (``ExPolygon._wrap``), never re-classified from points.
    """
    if len(pts) != 4:
        raise NotImplementedError(
            "fake orca.host.ExPolygon only accepts a 4-point axis-aligned "
            "rectangle contour from raw points; got {} points. See "
            "fake_orca.py module docstring 'DOCUMENTED SCOPE'.".format(len(pts))
        )
    xs = sorted(set(p[0] for p in pts))
    ys = sorted(set(p[1] for p in pts))
    if len(xs) != 2 or len(ys) != 2:
        raise NotImplementedError(
            "fake orca.host.ExPolygon: 4-point contour is not an axis-aligned "
            "rectangle (x's={}, y's={}). See fake_orca.py module docstring "
            "'DOCUMENTED SCOPE'.".format(xs, ys)
        )
    return _Rect(xs[0], ys[0], xs[1], ys[1])


def _diff_rect(subject: _Rect, clip: _Rect):
    s = subject
    if (
        clip.max_x <= s.min_x
        or clip.min_x >= s.max_x
        or clip.max_y <= s.min_y
        or clip.min_y >= s.max_y
    ):
        return [s]

    touches_left = clip.min_x <= s.min_x
    touches_right = clip.max_x >= s.max_x
    touches_top = clip.max_y >= s.max_y
    touches_bottom = clip.min_y <= s.min_y

    if touches_top and touches_bottom and not touches_left and not touches_right:
        return [
            _Rect(s.min_x, s.min_y, clip.min_x, s.max_y),
            _Rect(clip.max_x, s.min_y, s.max_x, s.max_y),
        ]

    if touches_top and not touches_bottom and not touches_left and not touches_right:
        notch = _Rect(clip.min_x, clip.min_y, clip.max_x, s.max_y)
        return [_Notched(outer=s, notch=notch)]

    if not (touches_top or touches_bottom or touches_left or touches_right):
        raise NotImplementedError(
            "fake orca.host.ExPolygon.diff_ex: an interior (enclosed) hole "
            "result is not supported by this rectangle/top-notch-only fake. "
            "See fake_orca.py module docstring 'DOCUMENTED SCOPE'."
        )

    raise NotImplementedError(
        "fake orca.host.ExPolygon.diff_ex: unrecognized subject/clip "
        "combination outside the documented rectangle/top-notch cases "
        "(subject={!r}, clip=({!r},{!r},{!r},{!r})).".format(
            s, clip.min_x, clip.min_y, clip.max_x, clip.max_y
        )
    )


def _range_intersect(a0: int, a1: int, b0: int, b1: int):
    """Intersection of closed integer ranges ``[a0, a1]`` and ``[b0, b1]``,
    or ``None`` if they do not overlap on a positive-length interval."""
    lo = max(a0, b0)
    hi = min(a1, b1)
    return (lo, hi) if hi > lo else None


def _intersect_rect_rect(a: _Rect, b: _Rect):
    x = _range_intersect(a.min_x, a.max_x, b.min_x, b.max_x)
    y = _range_intersect(a.min_y, a.max_y, b.min_y, b.max_y)
    if x is None or y is None:
        return []
    return [_Rect(x[0], y[0], x[1], y[1])]


def _intersect_notched_rect(shape: _Notched, clip: _Rect):
    """``shape`` (a top-notched U) intersected with an axis-aligned ``clip``
    rectangle — the "section clipped to a measurement strip" case.

    Exact case analysis (see ``ExPolygon.intersection_ex`` docstring for why
    this cannot be done by naive single-ring clipping): decompose the
    clipped material into up to three independent column bands —
    left arm, notch/bridge column, right arm — each an exact rectangle, then
    recombine based on which bands survive the clip and whether the bridge
    band (the only thing that can connect an arm to the rest) survives:

    - no band survives -> ``[]``.
    - exactly one band survives -> that single ``_Rect``.
    - the bridge band survives alongside one or both arms -> they are
      connected through it; return one combined shape (a plain ``_Rect`` if
      the clip left no notch cavity above the bridge, otherwise a
      ``_Notched``).
    - both arm bands survive but the bridge band does not (the clip's Y
      range sits above the bridge, inside the notch cavity, or otherwise
      fails to connect them) -> the arms are genuinely disjoint; return
      both ``_Rect`` pieces.
    """
    o, n = shape.outer, shape.notch

    arm_y = _range_intersect(o.min_y, o.max_y, clip.min_y, clip.max_y)
    if arm_y is None:
        return []
    ay0, ay1 = arm_y

    left_x = _range_intersect(o.min_x, n.min_x, clip.min_x, clip.max_x)
    right_x = _range_intersect(n.max_x, o.max_x, clip.min_x, clip.max_x)
    bridge_x = _range_intersect(n.min_x, n.max_x, clip.min_x, clip.max_x)

    bridge_y = None
    if bridge_x is not None:
        bridge_y = _range_intersect(o.min_y, n.min_y, ay0, ay1)

    left_rect = _Rect(left_x[0], ay0, left_x[1], ay1) if left_x is not None else None
    right_rect = _Rect(right_x[0], ay0, right_x[1], ay1) if right_x is not None else None
    bridge_rect = (
        _Rect(bridge_x[0], bridge_y[0], bridge_x[1], bridge_y[1])
        if bridge_x is not None and bridge_y is not None
        else None
    )

    present = [r for r in (left_rect, bridge_rect, right_rect) if r is not None]
    if not present:
        return []
    if len(present) == 1:
        return [present[0]]

    if bridge_rect is None:
        # Both arms survive but nothing bridges them: genuinely disjoint.
        return [left_rect, right_rect]

    # The bridge band survives alongside at least one arm: connected. Build
    # one combined shape spanning the union of the surviving bands' X range.
    arms_present = [r for r in (left_rect, right_rect) if r is not None]
    combined_min_x = min([bridge_rect.min_x] + [r.min_x for r in arms_present])
    combined_max_x = max([bridge_rect.max_x] + [r.max_x for r in arms_present])
    combined_outer = _Rect(combined_min_x, ay0, combined_max_x, ay1)

    if bridge_rect.max_y >= ay1:
        # No notch cavity survives the clip above the bridge: the combined
        # shape is simply a solid rectangle.
        return [combined_outer]

    combined_notch = _Rect(bridge_rect.min_x, bridge_rect.max_y, bridge_rect.max_x, ay1)
    return [_Notched(outer=combined_outer, notch=combined_notch)]


def _offset_rect(rect: _Rect, delta: int):
    new = _Rect(rect.min_x - delta, rect.min_y - delta, rect.max_x + delta, rect.max_y + delta)
    if new.max_x - new.min_x <= 0 or new.max_y - new.min_y <= 0:
        return []
    return [new]


def _offset_notched(shape: _Notched, delta: int):
    # Mirrors tests/unit/stub_ops.py's _offset_notched_rect exactly (see
    # module docstring KNOWN DIVERGENCES (10)): erosion (delta < 0) narrows
    # every material-to-air boundary pair by 2 * |delta| (each arm beside the
    # notch, the bridge below it, and the outer envelope); growth is not
    # supported (KNOWN DIVERGENCES (9)).
    if delta > 0:
        raise NotImplementedError(
            "fake orca.host.ExPolygon.offset: growing (positive delta) a "
            "notched shape is not needed by these tests and is not implemented. "
            "See fake_orca.py module docstring 'DOCUMENTED SCOPE'."
        )
    shrink = -2 * delta
    outer_w = shape.outer.max_x - shape.outer.min_x
    outer_h = shape.outer.max_y - shape.outer.min_y

    # The envelope itself is gone: nothing survives, whatever the arms do.
    if outer_w - shrink <= 0 or outer_h - shrink <= 0:
        return []

    arms_survive = shape.left_arm() - shrink > 0 and shape.right_arm() - shrink > 0
    bridge_survives = shape.bottom_bridge() - shrink > 0

    if not bridge_survives:
        # The bridge under the slot is consumed. If the arms still have
        # material they become two disjoint pieces — the severed-island case
        # TopologyValidator's connectivity check (len(eroded) > 1) exists to
        # catch.
        if not arms_survive:
            return []
        d_arm = -delta
        left = _Rect(
            shape.outer.min_x + d_arm, shape.notch.min_y, shape.notch.min_x - d_arm, shape.outer.max_y
        )
        right = _Rect(
            shape.notch.max_x + d_arm, shape.notch.min_y, shape.outer.max_x - d_arm, shape.outer.max_y
        )
        return [left, right]

    if not arms_survive:
        # Arms consumed but the bridge holds: one piece remains. KNOWN
        # VALIDATOR GAP (mirrors stub_ops.py's identical note), not a fake
        # artefact: the erosion check sees one island and passes even though
        # the slot walls are thinner than a bead.
        d = -delta
        return [
            _Rect(
                shape.outer.min_x + d, shape.outer.min_y + d, shape.outer.max_x - d, shape.notch.min_y
            )
        ]

    d = -delta
    new_outer = _Rect(
        shape.outer.min_x + d, shape.outer.min_y + d, shape.outer.max_x - d, shape.outer.max_y
    )
    new_notch = _Rect(
        shape.notch.min_x - d, shape.notch.min_y - d, shape.notch.max_x + d, shape.outer.max_y
    )
    return [_Notched(outer=new_outer, notch=new_notch)]


# ---------------------------------------------------------------------------
# orca.host
# ---------------------------------------------------------------------------


class Point:
    __slots__ = ("x", "y")

    def __init__(self, x, y):
        self.x = int(x)
        self.y = int(y)

    def __iter__(self):
        yield self.x
        yield self.y

    def __repr__(self):
        return "Point({}, {})".format(self.x, self.y)


class SurfaceType:
    stTop = "stTop"
    stBottom = "stBottom"
    stBottomBridge = "stBottomBridge"
    stInternalAfterExternalBridge = "stInternalAfterExternalBridge"
    stInternal = "stInternal"
    stInternalSolid = "stInternalSolid"
    stInternalBridge = "stInternalBridge"
    stInternalVoid = "stInternalVoid"


class Polygon:
    """Fake ``orca.host.Polygon`` (PluginHostGeometry.cpp:96-143). Mirrors
    the real bound Polygon's construction surface: constructible empty,
    ``.append(Point)``, ``.points`` as live-editable Point refs, and
    ``.as_array()`` as a numpy (N,2) int64 view. This is the ONLY object
    (besides an (N,2) int64 ndarray) ``ExPolygon``'s constructor/contour
    setter/``set_holes`` accept — see ``_as_polygon_points`` below, which
    mirrors the real host's ``as_polygon``/``parse_polygon``
    (PluginHostGeometry.cpp:26-57)."""

    def __init__(self):
        self._points = []

    def append(self, point):
        if not isinstance(point, Point):
            raise TypeError("Polygon.append expects an orca.host.Point")
        self._points.append(point)

    @property
    def points(self):
        """Live references: mutating a returned Point's .x/.y mutates this
        Polygon's buffer, mirroring the real reference_internal binding."""
        return list(self._points)

    def as_array(self):
        return np.array([(p.x, p.y) for p in self._points], dtype=np.int64)

    def set_points(self, src):
        self._points = [Point(x, y) for x, y in _parse_ndarray_points(src, "Polygon.set_points")]

    def size(self):
        return len(self._points)


def _parse_ndarray_points(h, who):
    """Mirror PluginHostGeometry.cpp's ``parse_polygon``: accept ONLY an
    (N,2) int64 ndarray; anything else raises ValueError. Returns a list of
    (int, int) tuples."""
    if not isinstance(h, np.ndarray):
        raise ValueError("{}: each contour/hole must be an (N,2) int64 ndarray".format(who))
    if h.dtype != np.int64:
        raise ValueError("{}: polygon coordinates must be int64 (scaled coords)".format(who))
    if h.ndim != 2 or h.shape[1] != 2:
        raise ValueError("{}: each polygon array must have shape (N,2)".format(who))
    if h.shape[0] < 3:
        raise ValueError("{}: a polygon needs at least 3 points".format(who))
    return [(int(x), int(y)) for x, y in h]


def _as_polygon_points(h, who):
    """Mirror PluginHostGeometry.cpp's ``as_polygon``: accept a bound
    ``Polygon`` (copied) or an (N,2) int64 ndarray, and nothing else — in
    particular NOT a raw list of ``Point`` (see module docstring KNOWN
    DIVERGENCES (0))."""
    if isinstance(h, Polygon):
        return [(p.x, p.y) for p in h.points]
    return _parse_ndarray_points(h, who)


def _is_sequence_not_str(x):
    if isinstance(x, str):
        return False
    try:
        iter(x)
    except TypeError:
        return False
    return True


class _ContourView:
    """What ``ExPolygon.contour`` returns — a read-only Polygon-shaped view
    (``.points``, ``.as_array()``) over this fake's internal shape kernel,
    enough for ``capability.py``'s ``OrcaGeometryOps.bounds()``, which reads
    ``polygon.contour.points``."""

    def __init__(self, points):
        self._points = points  # list[(int, int)]

    @property
    def points(self):
        return [Point(x, y) for x, y in self._points]

    def as_array(self):
        return np.array(self._points, dtype=np.int64)


class ExPolygon:
    """Rectangle/top-notch-only fake. See module docstring 'DOCUMENTED SCOPE'.

    Construction input types are faithful to the real binding (module
    docstring KNOWN DIVERGENCES (0)): only a fake ``Polygon`` or an (N,2)
    int64 ndarray is accepted for ``contour``/each hole — anything else,
    notably a raw list of ``Point``, raises ``ValueError`` exactly as
    ``PluginHostGeometry.cpp``'s ``as_polygon``/``parse_polygon`` do."""

    def __init__(self, contour, holes=None):
        pts = _as_polygon_points(contour, "ExPolygon.contour")
        hole_point_lists = []
        if holes is not None:
            if not _is_sequence_not_str(holes):
                raise ValueError("ExPolygon: holes must be a list of Polygon or (N,2) ndarrays")
            for h in holes:
                hole_point_lists.append(_as_polygon_points(h, "ExPolygon.hole"))
        if not hole_point_lists:
            self._shape = _classify_rect(pts)
            return
        # a single axis-aligned rectangular hole in a rectangular outer
        # contour, mirroring stub_ops.HoledRect's own scope exactly (see
        # _Holed's docstring). Anything wider (a hole in a notched shape, more
        # than one hole, a non-rectangular hole) still raises, matching this
        # fake's documented "raise loudly rather than silently approximate"
        # convention.
        if len(hole_point_lists) != 1:
            raise NotImplementedError(
                "fake orca.host.ExPolygon: constructing with more than one "
                "hole is not supported. See fake_orca.py module docstring."
            )
        outer_rect = _classify_rect(pts)
        hole_rect = _classify_rect(hole_point_lists[0])
        self._shape = _Holed(outer=outer_rect, hole=hole_rect)

    @classmethod
    def _wrap(cls, shape):
        self = cls.__new__(cls)
        self._shape = shape
        return self

    @property
    def contour(self):
        return _ContourView(self._shape.points())

    @property
    def holes(self):
        """See divergence (3): only ``_Holed`` (constructed directly with a
        ``holes=`` argument) carries an interior hole; every other shape
        this fake can produce has none.
        """
        if isinstance(self._shape, _Holed):
            return [_ContourView(self._shape.hole.points())]
        return []

    def num_contours(self):
        if isinstance(self._shape, _Holed):
            return 2
        return 1

    def area(self):
        return self._shape.area()

    def is_valid(self):
        return self._shape.area() > 0  # see divergence (2)

    def offset(self, delta_scaled):
        if isinstance(self._shape, _Rect):
            return [ExPolygon._wrap(r) for r in _offset_rect(self._shape, delta_scaled)]
        if isinstance(self._shape, _Notched):
            return [ExPolygon._wrap(r) for r in _offset_notched(self._shape, delta_scaled)]
        if isinstance(self._shape, _PolygonNotched):
            return [
                ExPolygon._wrap(r) for r in _offset_polygon_notched(self._shape, delta_scaled)
            ]
        if isinstance(self._shape, _PolygonMultiNotched):
            return [
                ExPolygon._wrap(r)
                for r in _offset_polygon_multi_notched(self._shape, delta_scaled)
            ]
        raise NotImplementedError("fake orca.host.ExPolygon.offset: unsupported shape.")

    def diff_ex(self, other):
        if isinstance(self._shape, _Polygon):
            other_shape = other._shape
            if isinstance(other_shape, _PolygonNotched):
                if other_shape.outer != self._shape:
                    raise NotImplementedError(
                        "fake orca.host.ExPolygon.diff_ex(polygon, "
                        "notched-clip) requires clip.outer == subject; got "
                        "a different outer polygon."
                    )
                return [ExPolygon._wrap(other_shape.notch)]
            if isinstance(other_shape, _Rect):
                return [
                    ExPolygon._wrap(p) for p in _diff_polygon_rect(self._shape, other_shape)
                ]
            raise NotImplementedError(
                "fake orca.host.ExPolygon.diff_ex: unsupported clip shape "
                "{!r} against a polygon subject. See module docstring "
                "'DOCUMENTED SCOPE'.".format(type(other_shape))
            )
        if isinstance(self._shape, (_PolygonNotched, _PolygonMultiNotched)):
            # a SECOND (or third...) slot against a section that already
            # carries at least one notch — see ``_add_polygon_notch``'s own
            # docstring.
            other_shape = other._shape
            if isinstance(other_shape, _Rect):
                return [
                    ExPolygon._wrap(p)
                    for p in _add_polygon_notch(self._shape, other_shape)
                ]
            raise NotImplementedError(
                "fake orca.host.ExPolygon.diff_ex: unsupported clip shape "
                "{!r} against an already-notched polygon subject. See "
                "module docstring 'DOCUMENTED SCOPE'.".format(type(other_shape))
            )
        if not isinstance(self._shape, _Rect):
            raise NotImplementedError(
                "fake orca.host.ExPolygon.diff_ex: subject must be a plain "
                "rectangle or polygon; got {!r}. See fake_orca.py module "
                "docstring 'DOCUMENTED SCOPE' (mirrors stub_ops.py's "
                "identical limit).".format(type(self._shape))
            )
        other_shape = other._shape
        if isinstance(other_shape, _Notched):
            if other_shape.outer != self._shape:
                raise NotImplementedError(
                    "fake orca.host.ExPolygon.diff_ex(subject, notched-clip) "
                    "requires clip.outer == subject; got outer={!r} "
                    "subject={!r}.".format(other_shape.outer, self._shape)
                )
            return [ExPolygon._wrap(other_shape.notch)]
        if isinstance(other_shape, _Rect):
            return [ExPolygon._wrap(p) for p in _diff_rect(self._shape, other_shape)]
        raise NotImplementedError(
            "fake orca.host.ExPolygon.diff_ex: unsupported clip shape {!r}.".format(
                type(other_shape)
            )
        )

    def intersection_ex(self, other):
        """``self`` intersected with ``other``. Returns a LIST of
        ``ExPolygon`` (possibly empty, possibly several), mirroring the real
        binding (``PluginHostGeometry.cpp:211-213``, ``intersection_ex``).

        Supports exactly the two shape combinations this fake's rectangle
        kernel can represent AND the capability actually calls
        (``OrcaGeometryOps.intersection`` -> ``aircraft_core.frame.local_span``,
        which clips a section against a thin measurement strip): a plain
        rectangle subject against a rectangle clip, and a top-notched
        (U-shaped) subject against a rectangle clip. Both are computed
        exactly via ``_intersect_rect_rect``/``_intersect_notched_rect``
        above, not via any general polygon-clipping routine. Anything else
        (a notched clip, i.e. ``other`` itself notched) raises
        ``NotImplementedError`` naming the shape.

        WHY THIS IS EXACT RECTANGLE ARITHMETIC AND NOT SUTHERLAND-HODGMAN:
        ``tests/unit/stub_ops.py``'s ``StubOps.intersection`` (the analogous
        operation one level up, over float mm polygons) had to special-case
        its own naive approach: single-ring Sutherland-Hodgman clipping of a
        concave subject against a convex window is only correct when the
        true intersection is a single connected region. When the true
        result is actually two disjoint pieces (e.g. a probe strip that
        clears the bridge below a notch but lands inside both arms),
        naive single-ring clipping does not fail loudly — it silently
        stitches the two pieces into one self-intersecting "bowtie" ring
        with a wrong bounds box and a wrong area, exactly the kind of
        silent mis-measurement this whole exercise exists to prevent. This
        fake's shapes are restricted enough (axis-aligned rectangle subject
        or axis-aligned top-notch subject, axis-aligned rectangle clip) that
        no Sutherland-Hodgman-style ring walk is needed at all:
        ``_intersect_notched_rect`` decomposes the notched subject into up
        to three independent column bands (left arm / notch-bridge / right
        arm) and recombines them by exact interval arithmetic, so the
        multi-piece case is detected structurally (the bridge band is
        simply absent) rather than by a fragile post-hoc scan line probe.
        This is exact, not approximate, for every case this fake's
        DOCUMENTED SCOPE admits — see ``KNOWN DIVERGENCES`` (11) for what
        is NOT covered.
        """
        other_shape = other._shape
        if isinstance(self._shape, _Polygon) and isinstance(other_shape, _Rect):
            return [
                ExPolygon._wrap(r) for r in _intersect_polygon_rect(self._shape, other_shape)
            ]
        if isinstance(self._shape, _Rect) and isinstance(other_shape, _Rect):
            return [ExPolygon._wrap(r) for r in _intersect_rect_rect(self._shape, other_shape)]
        if isinstance(self._shape, _Notched) and isinstance(other_shape, _Rect):
            return [
                ExPolygon._wrap(r) for r in _intersect_notched_rect(self._shape, other_shape)
            ]
        raise NotImplementedError(
            "fake orca.host.ExPolygon.intersection_ex: unsupported "
            "subject/clip combination (subject={!r}, clip={!r}). See "
            "fake_orca.py module docstring 'DOCUMENTED SCOPE'.".format(
                type(self._shape), type(other_shape)
            )
        )


def describe(expolygon):
    """Test-only introspection: expose the fake's internal shape as plain
    scaled-int tuples, so tests can assert exact geometry without a second,
    potentially-inconsistent float formula. NOT part of any real orca.host
    API — do not import this from capability code."""
    shape = expolygon._shape
    if isinstance(shape, _Rect):
        return {"kind": "rect", "bounds": (shape.min_x, shape.min_y, shape.max_x, shape.max_y)}
    if isinstance(shape, _Notched):
        o, n = shape.outer, shape.notch
        return {
            "kind": "notched",
            "outer": (o.min_x, o.min_y, o.max_x, o.max_y),
            "notch": (n.min_x, n.min_y, n.max_x, n.max_y),
        }
    if isinstance(shape, _Polygon):
        return {"kind": "polygon", "bounds": shape.bounds(), "num_points": len(shape.pts)}
    if isinstance(shape, _PolygonNotched):
        return {
            "kind": "polygon_notched",
            "outer_bounds": shape.outer.bounds(),
            "notch": _shape_bounds(shape.notch),
        }
    if isinstance(shape, _PolygonMultiNotched):
        # N >= 2 notches. Kept as a distinct "kind" from "polygon_notched"
        # (rather than folding the single-notch case into a one-element list)
        # so existing tests asserting ``kind == "polygon_notched"`` for a
        # single web keep meaning exactly that, per this function's own "no
        # second, potentially inconsistent, float formula" purpose.
        return {
            "kind": "polygon_multi_notched",
            "outer_bounds": shape.outer.bounds(),
            "notches": [_shape_bounds(n) for n in shape.notches],
        }
    raise NotImplementedError("fake_orca.describe: unrecognized shape {!r}".format(type(shape)))


class Surface:
    def __init__(self, surface_type, expolygon=None):
        self.surface_type = surface_type
        self.expolygon = expolygon


class SurfaceCollection:
    def __init__(self, surfaces=None):
        self._surfaces = list(surfaces or [])

    @property
    def surfaces(self):
        return list(self._surfaces)

    def size(self):
        return len(self._surfaces)

    def set(self, surfaces):
        self._surfaces = list(surfaces)


class LayerRegion:
    def __init__(self, slices_collection):
        self._slices = slices_collection

    @property
    def slices(self):
        # Live reference,: "the returned SurfaceCollection is a live mutable
        # reference". Always the same object for this region.
        return self._slices


class Layer:
    def __init__(self, print_z, regions, slice_z=None):
        self.print_z = print_z
        # the plane the host actually sliced this layer at -- half a layer
        # height below print_z (capability.py's own ``_section_plane_for``
        # docstring). ``None`` falls back to print_z, mirroring the real host
        # attribute possibly being absent; every fixture builder below sets
        # this explicitly so the mesh/host alignment check compares apples to
        # apples (see the "MESH SUPPORT" section comment
        # below).
        self.slice_z = slice_z
        self._regions = list(regions)
        self.make_slices_calls = 0  # see divergence (5)

    def regions(self):
        return list(self._regions)

    def make_slices(self):
        self.make_slices_calls += 1


class PrintObject:
    def __init__(self, object_id, layers, model_object=None, trafo=None):
        self._id = object_id
        self._layers = list(layers)
        # backs ``ctx.object.model_object()``. One of:
        #  - a ``ModelObject`` instance (returned as-is);
        #  - ``None`` (no model at all -- the "no model object on this
        #    print object" fail-closed case); or
        #  - a zero-argument callable, so a test can simulate the host's
        #    ``model_object()`` ITSELF raising (a real failure mode this
        #    fake must be able to reproduce, not just an empty result).
        self._model_object_provider = model_object
        # backs ``ctx.object.trafo()`` (PluginHostSlicing.cpp:322). One of:
        # ``None`` (default -- capability.py records this as an absent value,
        # never an error), a 4x4 nested-list matrix supplied explicitly by a
        # test (real numpy-vs-list normalisation is capability.py's job, not
        # this fake's -- no Orca-convention arithmetic is derived here), or a
        # zero-argument callable so a test can simulate ``trafo()`` itself
        # raising (e.g. numpy unavailable in the embedded interpreter, per the
        # PEP 723 ``dependencies = []`` note in aircraft_structures.py).
        self._trafo_provider = trafo

    def id(self):
        return self._id

    def layers(self):
        return list(self._layers)

    def model_object(self):
        provider = self._model_object_provider
        if provider is None or isinstance(provider, ModelObject):
            return provider
        return provider()

    def trafo(self):
        provider = self._trafo_provider
        if callable(provider):
            return provider()
        return provider


# ---------------------------------------------------------------------------
# MESH SUPPORT: ``orca.host`` model/mesh surface.
#
# ``capability.py``'s ``_build_mesh_sectioner`` calls, exactly (the real
# source is the authority; see plugin/aircraft_structures/capability.py):
#   ctx.object.model_object()            -> ModelObject (a METHOD)
#   model_object.volumes()               -> list[ModelVolume]
#   model_object.instances()             -> list[ModelInstance]
#   volume.is_model_part()               -> bool
#   volume.mesh()                        -> TriangleMesh
#   volume.offset()/.rotation()/.scaling_factor()/.mirror()   -> 3-tuples
#   instance.offset()/.rotation()/.scaling_factor()/.mirror() -> 3-tuples
#   mesh.facets_count()/.vertex_count()  -> int
#   mesh.vertex(i)                       -> (x, y, z) tuple, PLAIN MM FLOATS
#     (mesh space is never scaled-integer, unlike orca.host.Point/ExPolygon)
#   mesh.triangle(i)                     -> (i, j, k) tuple of vertex indices
#
# ``ctx.print.model`` is NOT read anywhere in capability.py (grepped, not
# assumed) -- only ``ctx.object.model_object()`` -- so this fake does not
# model a ``Model``/``ctx.print`` surface at all; faking an unread attribute
# would be dead weight with nothing exercising it.
# ---------------------------------------------------------------------------


class TriangleMesh:
    """Fake ``orca.host`` mesh: plain vertex/triangle index arrays, matching
    ``capability.py``'s own read surface exactly. Coordinates are plain
    millimetre floats."""

    def __init__(self, vertices, triangles):
        self._vertices = list(vertices)
        self._triangles = list(triangles)

    def facets_count(self):
        return len(self._triangles)

    def vertex_count(self):
        return len(self._vertices)

    def vertex(self, i):
        return self._vertices[i]

    def triangle(self, i):
        return self._triangles[i]


class ModelVolume:
    """Fake ``orca.host`` ModelVolume: one mesh part plus its own local
    placement (offset/rotation/scaling_factor/mirror) -- exactly what
    ``capability.py._build_mesh_sectioner`` reads."""

    def __init__(
        self,
        mesh,
        offset=(0.0, 0.0, 0.0),
        rotation=(0.0, 0.0, 0.0),
        scaling_factor=(1.0, 1.0, 1.0),
        mirror=(1.0, 1.0, 1.0),
        is_part=True,
        matrix=None,
    ):
        self._mesh = mesh
        # ALL FOUR placement members -- offset/rotation/scaling_factor/mirror --
        # may be a zero-argument callable, so a test can simulate ANY of them
        # raising or being read. capability.py's geometry path REQUIRES every
        # one of these (via ``_require_geometry_member``); a raising callable
        # proves that requirement refuses the slice rather than silently
        # falling back to an identity default (which would fail open).
        self._offset = offset if callable(offset) else tuple(offset)
        self._rotation = rotation if callable(rotation) else tuple(rotation)
        self._scaling_factor = (
            scaling_factor if callable(scaling_factor) else tuple(scaling_factor)
        )
        self._mirror = mirror if callable(mirror) else tuple(mirror)
        self._is_part = is_part
        # backs ``volume.matrix()`` (PluginHostModel.cpp:46). ``None`` by
        # default -- capability.py's own record treats a None matrix as absent,
        # not an error. A test supplies an explicit 4x4 nested list (or a
        # raising callable) rather than this fake deriving one from
        # offset/rotation/scaling_factor, matching the real binding's
        # independence (the host computes its matrix from its OWN internal
        # ``Transformation``, not from the decomposed accessors). ``matrix``
        # never feeds geometry (diagnostic-only), so it keeps its own,
        # separately-resilient contract regardless of this change.
        self._matrix = matrix

    def mesh(self):
        return self._mesh

    def offset(self):
        if callable(self._offset):
            return self._offset()
        return self._offset

    def rotation(self):
        if callable(self._rotation):
            return self._rotation()
        return self._rotation

    def scaling_factor(self):
        if callable(self._scaling_factor):
            return self._scaling_factor()
        return self._scaling_factor

    def mirror(self):
        if callable(self._mirror):
            return self._mirror()
        return self._mirror

    def is_model_part(self):
        return self._is_part

    def matrix(self):
        if callable(self._matrix):
            return self._matrix()
        return self._matrix


class ModelInstance:
    """Fake ``orca.host`` ModelInstance: the placement on the plate.

    Per ``capability.py._build_mesh_sectioner``'s own documented transform,
    TRANSLATION here must never move the recovered spar geometry -- only
    rotation/scale/mirror do (Orca's slices are object-relative). Modelled
    anyway, with a non-zero default-able offset, precisely so
    ``test_m18_mesh_transform.py`` can prove that offset is correctly
    ignored.
    """

    def __init__(
        self,
        offset=(0.0, 0.0, 0.0),
        rotation=(0.0, 0.0, 0.0),
        scaling_factor=(1.0, 1.0, 1.0),
        mirror=(1.0, 1.0, 1.0),
        matrix=None,
        is_left_handed=False,
    ):
        # (REVIEWER REVISE, on top of 61e2d0b): see ``ModelVolume``'s identical
        # contract -- ALL FOUR placement members may be a zero-argument
        # callable, not just ``mirror``.
        self._offset = offset if callable(offset) else tuple(offset)
        self._rotation = rotation if callable(rotation) else tuple(rotation)
        self._scaling_factor = (
            scaling_factor if callable(scaling_factor) else tuple(scaling_factor)
        )
        self._mirror = mirror if callable(mirror) else tuple(mirror)
        # see ``ModelVolume.matrix``'s docstring -- same contract.
        self._matrix = matrix
        # backs ``instance.is_left_handed()`` (PluginHostModel.cpp:86) -- the
        # host's OWN mirror flag, distinct from the plugin's computed
        # determinant-sign-from-mirror-tuple. May also be a zero-argument
        # callable, so a test can simulate it raising.
        self._is_left_handed = is_left_handed

    def offset(self):
        if callable(self._offset):
            return self._offset()
        return self._offset

    def rotation(self):
        if callable(self._rotation):
            return self._rotation()
        return self._rotation

    def scaling_factor(self):
        if callable(self._scaling_factor):
            return self._scaling_factor()
        return self._scaling_factor

    def mirror(self):
        if callable(self._mirror):
            return self._mirror()
        return self._mirror

    def matrix(self):
        if callable(self._matrix):
            return self._matrix()
        return self._matrix

    def is_left_handed(self):
        if callable(self._is_left_handed):
            return self._is_left_handed()
        return self._is_left_handed


class ModelObject:
    """Fake ``orca.host`` ModelObject: one or more part/modifier volumes plus
    one or more plate instances.

    ``volumes``/``instances`` are each either a plain list (returned fresh,
    copied, on every call -- the ordinary case) or a zero-argument callable, so
    a test can simulate the host's OWN ``volumes()``/``instances()`` itself
    raising (proving ``_build_mesh_sectioner`` records the error and re-raises
    the ORIGINAL exception rather than silently treating a broken host call as
    "zero volumes").
    """

    def __init__(self, volumes, instances):
        self._volumes = volumes if callable(volumes) else list(volumes)
        self._instances = instances if callable(instances) else list(instances)

    def volumes(self):
        if callable(self._volumes):
            return self._volumes()
        return list(self._volumes)

    def instances(self):
        if callable(self._instances):
            return self._instances()
        return list(self._instances)


def _weld_triangle_soup(tri_tuples):
    """``(normal, v0, v1, v2)`` float triples -> welded ``(vertices,
    triangles)`` index arrays, mirroring ``tools/m18_mesh_probe.py``'s
    ``read_binary_stl`` welding -- done here directly on an in-memory
    triangle soup rather than via an STL round-trip, so
    ``tools/make_fixtures.py``'s own triangle builders (``box_triangles``,
    ``loft_triangles``) can be reused verbatim as this fake's mesh source
    (no second mesh-building implementation)."""
    verts = []
    index = {}
    tris = []
    for _normal, v0, v1, v2 in tri_tuples:
        ids = []
        for v in (v0, v1, v2):
            key = (float(v[0]), float(v[1]), float(v[2]))
            i = index.get(key)
            if i is None:
                i = len(verts)
                index[key] = i
                verts.append(key)
            ids.append(i)
        tris.append(tuple(ids))
    return verts, tris


def mesh_from_triangle_soup(tri_tuples):
    """Weld and wrap a ``(normal, v0, v1, v2)`` triangle soup as a fake
    ``TriangleMesh``."""
    verts, tris = _weld_triangle_soup(tri_tuples)
    return TriangleMesh(verts, tris)


def box_mesh_mm(x0, y0, z0, x1, y1, z1):
    """An axis-aligned box mesh, reusing ``tools/make_fixtures.box_triangles``
    verbatim (no second box-mesh implementation)."""
    mf = _load_make_fixtures_module()
    return mesh_from_triangle_soup(mf.box_triangles(x0, y0, z0, x1, y1, z1))


def loft_mesh_mm(stations):
    """A spanwise loft mesh between two or more ``(z, outline_mm)`` stations,
    reusing ``tools/make_fixtures.loft_triangles`` verbatim (no second loft
    implementation)."""
    mf = _load_make_fixtures_module()
    return mesh_from_triangle_soup(mf.loft_triangles(stations))


def identity_model_object(mesh):
    """One part volume, one instance, identity GEOMETRIC placement (no
    rotation/scale/mirror), for fixtures that do not test transform handling
    itself (see ``test_m18_mesh_transform.py`` for the dedicated non-identity
    rotation/translation cases, which build their own ``ModelObject``
    directly).

    The default instance carries a NON-ZERO offset
    ((128.0, 0.0, 0.0), the exact 128 mm regression value) rather than
    ``(0, 0, 0)``: instance translation is dropped by
    ``capability.py._build_mesh_sectioner`` (Orca's slices are
    object-relative), so this does not change the resulting geometry at
    all -- but it means EVERY test built on this default helper, not only
    the dedicated ``test_m18_mesh_transform.py`` cases, would immediately
    fail its mesh/host alignment check by ~128 mm if that translation were
    ever mistakenly reapplied. A caller that genuinely needs an all-zero
    instance builds ``ModelObject`` directly, as the dedicated transform
    tests already do.
    """
    return ModelObject(
        [ModelVolume(mesh)], [ModelInstance(offset=(128.0, 0.0, 0.0))]
    )


def _rect_corners(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def box_with_rect_channel_mesh_mm(
    outer_x0, outer_y0, outer_x1, outer_y1, z0, z1,
    hole_x0, hole_y0, hole_x1, hole_y1,
):
    """A rectangular prism with a rectangular channel bored root-to-tip, OPEN at
    both Z ends -- the fixture analogue of ``tools/make_wing_spar.py`` (side
    walls only, annulus end caps, tube open at both ends so the channel is a
    genuine bore, never a sealed interior cavity), simplified to an
    axis-aligned rectangle bore in an axis-aligned rectangle bar: every
    mesh-transform test that needs this only needs a genuinely interior loop at
    a known, movable centre, not a faithful spar cross-section.

    ``MeshSectioner`` never reads triangle winding or normals -- only vertex
    positions -- so no attempt is made here to orient faces consistently
    outward; a dummy normal is supplied to satisfy the ``(normal, v0, v1,
    v2)`` triangle-soup shape ``mesh_from_triangle_soup`` expects.
    """
    outer = _rect_corners(outer_x0, outer_y0, outer_x1, outer_y1)
    inner = _rect_corners(hole_x0, hole_y0, hole_x1, hole_y1)
    n = 4
    dummy_normal = (0.0, 0.0, 0.0)

    def v3(z, xy):
        return (xy[0], xy[1], z)

    tris = []

    def quad(a, b, c, d):
        tris.append((dummy_normal, a, b, c))
        tris.append((dummy_normal, a, c, d))

    # Outer side walls (open, no top/bottom caps here).
    for k in range(n):
        j = (k + 1) % n
        a, b = v3(z0, outer[k]), v3(z0, outer[j])
        c, d = v3(z1, outer[j]), v3(z1, outer[k])
        quad(a, b, c, d)

    # Inner (channel) side walls (open at both ends -- a bore, not a cavity).
    for k in range(n):
        j = (k + 1) % n
        a, b = v3(z0, inner[k]), v3(z0, inner[j])
        c, d = v3(z1, inner[j]), v3(z1, inner[k])
        quad(a, b, c, d)

    # End caps: the annulus between outer and inner, at z0 and z1, so the
    # channel opens at both ends instead of sealing an interior cavity.
    for z in (z0, z1):
        for k in range(n):
            j = (k + 1) % n
            o_k, o_j = v3(z, outer[k]), v3(z, outer[j])
            i_k, i_j = v3(z, inner[k]), v3(z, inner[j])
            quad(o_k, o_j, i_j, i_k)

    return mesh_from_triangle_soup(tris)


def _rotate_xy(x, y, angle_rad):
    """Rotate ``(x, y)`` by ``angle_rad`` about the Z axis -- the ONLY
    rotation axis ``mesh_local_from_print_xyz`` (below) supports; see that
    function's docstring for why restricting to Z is deliberate."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return (x * c - y * s, x * s + y * c)


def mesh_local_from_print_xyz(pt, v_off, v_rot_z, v_scl, i_scl, i_mir, i_rot_z):
    """Invert ``capability.py._build_mesh_sectioner``'s forward transform for
    ONE print-space point, so a mesh built from these local coordinates
    reproduces ``pt`` after the real transform is applied (before the final
    Z-normalisation, which callers control by choosing ``v_off``/mesh Z
    range so the whole mesh's own z_min lands at 0 -- see
    ``test_m18_mesh_transform.py`` for how that is arranged).

    Restricted to Z-AXIS-ONLY rotations for both volume and instance (the
    ``rotation()`` tuple's third component only): the real transform's
    ``rot()`` applies X, then Y, then Z Euler rotations in sequence, and
    mixing X/Y rotation into a 2D cross-section fixture is not needed by
    anything this test file pins (translation-drop and Z-axis
    rotation-application are the two behaviours under test). Restricting to
    Z keeps every mesh Z coordinate unaffected by rotation, which is what
    lets a caller choose an exact, hand-checkable mesh Z per print-space Z
    without solving a coupled 3D inverse.

    Mirrors the forward pipeline -- scale by ``v_scl``, rotate by
    ``v_rot_z``, translate by ``v_off``, scale/mirror by ``i_scl``/``i_mir``,
    rotate by ``i_rot_z`` (instance TRANSLATION omitted, exactly as
    ``_build_mesh_sectioner`` documents and deliberately never applies) --
    function-for-function, in reverse.
    """
    x, y, z = pt
    x, y = _rotate_xy(x, y, -i_rot_z)
    x, y, z = (
        x / (i_mir[0] * i_scl[0]),
        y / (i_mir[1] * i_scl[1]),
        z / (i_mir[2] * i_scl[2]),
    )
    x, y, z = x - v_off[0], y - v_off[1], z - v_off[2]
    x, y = _rotate_xy(x, y, -v_rot_z)
    x, y, z = x / v_scl[0], y / v_scl[1], z / v_scl[2]
    return (x, y, z)


# ---------------------------------------------------------------------------
# orca.slicing
# ---------------------------------------------------------------------------


class Step:
    """Mirrors ``enum SlicingPipelineStepPlugin`` exactly, pinned from
    ``SlicingPipelinePluginCapability.cpp:14-34`` (the authoritative real
    binding for this plugin-facing enum). NOT the host's internal
    ``PrintObjectStep`` — that is a different, internal C++ enum and is the
    wrong source for this surface. 13 values, in the real enum's order."""

    posSlice = "posSlice"
    posPerimeters = "posPerimeters"
    posEstimateCurledExtrusions = "posEstimateCurledExtrusions"
    posPrepareInfill = "posPrepareInfill"
    posInfill = "posInfill"
    posIroning = "posIroning"
    posContouring = "posContouring"
    posSupportMaterial = "posSupportMaterial"
    posDetectOverhangsForLift = "posDetectOverhangsForLift"
    posSimplifyPath = "posSimplifyPath"
    psWipeTower = "psWipeTower"
    psSkirtBrim = "psSkirtBrim"
    psGCodePostProcess = "psGCodePostProcess"


ALL_STEPS = [
    Step.posSlice,
    Step.posPerimeters,
    Step.posEstimateCurledExtrusions,
    Step.posPrepareInfill,
    Step.posInfill,
    Step.posIroning,
    Step.posContouring,
    Step.posSupportMaterial,
    Step.posDetectOverhangsForLift,
    Step.posSimplifyPath,
    Step.psWipeTower,
    Step.psSkirtBrim,
    Step.psGCodePostProcess,
]
assert len(ALL_STEPS) == 13, "Step must carry exactly 13 values"
NON_SLICE_STEPS = [s for s in ALL_STEPS if s != Step.posSlice]

# Steps fired with ctx.object None in the real pipeline
# (SlicingPipelinePluginCapability.cpp:25-33): psGCodePostProcess fires
# outside Print::process() entirely (ctx.print is also None there), and
# psWipeTower/psSkirtBrim are print-wide, not object-scoped.
OBJECT_NONE_STEPS = frozenset({Step.psWipeTower, Step.psSkirtBrim, Step.psGCodePostProcess})


class SlicingPipelineCapabilityBase:
    """Plain base class with a no-op ``get_config``."""

    def get_config(self):
        return None


class SlicingPipelineContext:
    """Fake ``ctx`` passed to ``execute()``.

    ``cancelled_fn`` is called once per ``ctx.cancelled()`` invocation (the
    capability calls it once per layer iteration); tests supply a predicate
    closure to simulate cancellation partway through a run.
    """

    def __init__(self, step, obj, orca_version="fake-test-1.0.0", config=None, cancelled_fn=None):
        self.step = step
        self.object = obj
        self.orca_version = orca_version
        self._config = dict(config or {})
        self._cancelled_fn = cancelled_fn or (lambda: False)
        self.cancelled_call_count = 0

    def config_value(self, key):
        return self._config[key]

    def cancelled(self):
        self.cancelled_call_count += 1
        return bool(self._cancelled_fn())


# ---------------------------------------------------------------------------
# orca.{ExecutionResult,PluginResult,base,plugin,register_capability}
# ---------------------------------------------------------------------------


class PluginResult:
    Success = "Success"
    Skipped = "Skipped"
    RecoverableError = "RecoverableError"
    FatalError = "FatalError"


class ExecutionResult:
    def __init__(self, status, message):
        self.status = status
        self.message = message

    def __repr__(self):
        return "ExecutionResult(status={!r}, message={!r})".format(self.status, self.message)

    @staticmethod
    def success(msg=""):
        return ExecutionResult(PluginResult.Success, msg)

    @staticmethod
    def failure(status, msg):
        return ExecutionResult(status, msg)


class base:
    """``orca.base`` — plain subclassable placeholder."""


def plugin(cls):
    """``orca.plugin`` — identity decorator."""
    return cls


REGISTERED_CAPABILITIES = []


def register_capability(cls):
    REGISTERED_CAPABILITIES.append(cls)
    return cls


# ---------------------------------------------------------------------------
# Assembly + install/uninstall helpers
# ---------------------------------------------------------------------------


def build_fake_orca_module():
    """Build a fresh ``types.ModuleType('orca')`` with every attribute the
    bundle's ``import orca`` + ``orca.xxx`` attribute access needs."""
    orca_mod = types.ModuleType("orca")

    slicing_mod = types.ModuleType("orca.slicing")
    slicing_mod.Step = Step
    slicing_mod.unscale = unscale
    slicing_mod.SlicingPipelineCapabilityBase = SlicingPipelineCapabilityBase
    slicing_mod.SlicingPipelineContext = SlicingPipelineContext

    host_mod = types.ModuleType("orca.host")
    host_mod.Point = Point
    host_mod.Polygon = Polygon
    host_mod.ExPolygon = ExPolygon
    host_mod.Surface = Surface
    host_mod.SurfaceType = SurfaceType
    host_mod.SurfaceCollection = SurfaceCollection
    host_mod.LayerRegion = LayerRegion
    host_mod.Layer = Layer
    host_mod.PrintObject = PrintObject

    orca_mod.slicing = slicing_mod
    orca_mod.host = host_mod
    orca_mod.ExecutionResult = ExecutionResult
    orca_mod.PluginResult = PluginResult
    orca_mod.base = base
    orca_mod.plugin = plugin
    orca_mod.register_capability = register_capability

    return orca_mod


def install():
    """Install a fresh fake ``orca`` module into ``sys.modules``.

    Returns ``(fake_module, restore_fn)``. ``restore_fn()`` must be called
    (e.g. via ``addCleanup``) to put ``sys.modules`` back exactly as it was —
    this harness must never leak a fake ``orca`` into other test modules.
    """
    import sys

    previous = sys.modules.get("orca")
    fake = build_fake_orca_module()
    sys.modules["orca"] = fake

    def restore():
        if previous is None:
            sys.modules.pop("orca", None)
        else:
            sys.modules["orca"] = previous

    return fake, restore


# ---------------------------------------------------------------------------
# Test-fixture builders (not part of the "orca" surface; harness helpers)
# ---------------------------------------------------------------------------


import os as _os

REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
BUNDLE_PATH = _os.path.join(REPO_ROOT, "plugin", "aircraft_structures", "aircraft_structures.py")


def exec_bundle(fake_file_path):
    """Read the real, checked-in GENERATED BUNDLE and ``exec`` it with
    ``__file__`` pointed at ``fake_file_path`` (inside a tmp dir the caller
    controls), so ``capability.py``'s ``_plugin_dir()`` — and therefore
    arming and diagnostics — resolve inside the tmp dir instead of the real
    installed plugin directory.

    The bundle file itself is only read, never modified.
    """
    with open(BUNDLE_PATH, "r", encoding="utf-8") as handle:
        source = handle.read()
    namespace = {"__name__": "aircraft_structures_bundle_under_test", "__file__": fake_file_path}
    exec(compile(source, fake_file_path, "exec"), namespace)
    return namespace


_USE_DEFAULT_MESH = object()  # sentinel: distinguishes "not given" from "None"


def make_rect_object(object_id, n_layers, width_mm, depth_mm, layer_height_mm, surface_type=SurfaceType.stInternal, min_x_mm=0.0, min_y_mm=0.0, model_object=_USE_DEFAULT_MESH):
    """Build a ``PrintObject`` with ``n_layers`` identical solid-rectangle layers,
    one region/one surface each — the rectangular coupon.

    also attaches a ``model_object`` -- by default (``model_object`` not given
    at all) an identity-placed box mesh whose cross-section matches this same
    rectangle at every layer's own ``slice_z``, so ``capability.py``'s
    mesh/host alignment check (a hard requirement, see
    plugin/aircraft_structures/capability.py's module docstring) has something
    to agree with by construction. Pass ``model_object`` explicitly (a
    ``ModelObject``, ``None`` for "no model at all", or a zero-arg callable --
    see ``PrintObject.model_object``'s docstring) to override this default --
    e.g. a mesh with an interior channel, a non-identity instance transform, or
    a deliberately unreadable model, per ``test_m18_mesh_transform.py``. A bare
    ``None`` is deliberately NOT the default value here (a separate sentinel
    is): ``None`` must mean "test the mesh-unavailable path", never "caller
    didn't say", or a fixture that means to build a working mesh could silently
    get no mesh instead.
    """
    # XY-centring fix (capability.py's ``_build_mesh_sectioner``): PrintObject
    # centres its slices on the transformed mesh's OWN XY bounding-box centre,
    # and that centring erases any pure translation of a symmetric mesh -- a
    # box mesh built at an arbitrary ``min_x_mm``/``min_y_mm`` will ALWAYS
    # cross-section back to ``[-width_mm/2, width_mm/2]`` x ``[-depth_mm/2,
    # depth_mm/2]`` after that centring, regardless of where its raw local
    # vertices were authored. So the DEFAULT identity mesh this helper builds
    # can only ever match a host rect that is ALSO built there -- an arbitrary
    # ``min_x_mm``/``min_y_mm`` together with the default mesh is not something
    # a real host could produce for a plain rectangular part, since real
    # object-relative slice coordinates are centred the same way. No caller in
    # this repository passes a non-default ``min_x_mm``/``min_y_mm`` together
    # with the default mesh (the only calls that do also pass an explicit
    # ``model_object`` of their own -- see ``test_m18_mesh_transform.py``), so
    # this is enforced rather than silently reinterpreted.
    if model_object is _USE_DEFAULT_MESH and (min_x_mm != 0.0 or min_y_mm != 0.0):
        raise ValueError(
            "make_rect_object: non-zero min_x_mm/min_y_mm ({}, {}) is not "
            "supported together with the default (auto-matching) mesh -- "
            "the mesh always cross-sections centred at (0, 0) after "
            "capability.py's XY-centring fix. Pass an explicit "
            "model_object built for the position you want instead (see "
            "test_m18_mesh_transform.py).".format(min_x_mm, min_y_mm)
        )

    effective_min_x_mm = -width_mm / 2.0 if model_object is _USE_DEFAULT_MESH else min_x_mm
    effective_min_y_mm = -depth_mm / 2.0 if model_object is _USE_DEFAULT_MESH else min_y_mm

    min_x = to_scaled(effective_min_x_mm)
    min_y = to_scaled(effective_min_y_mm)
    max_x = to_scaled(effective_min_x_mm + width_mm)
    max_y = to_scaled(effective_min_y_mm + depth_mm)

    layers = []
    for i in range(n_layers):
        print_z = round(layer_height_mm * (i + 1), 6)
        slice_z = round(print_z - layer_height_mm / 2.0, 6)
        expoly = ExPolygon._wrap(_Rect(min_x, min_y, max_x, max_y))
        surface = Surface(surface_type, expoly)
        region = LayerRegion(SurfaceCollection([surface]))
        layers.append(Layer(print_z, [region], slice_z=slice_z))

    if model_object is _USE_DEFAULT_MESH:
        # Mesh Z range starts at 0 (the bed) and pads above the last
        # slice_z, so every layer's own slice_z is strictly interior to the
        # mesh's Z range (MeshSectioner.section_at refuses an empty result
        # only when it is NOT at/outside the mesh's own [z_min, z_max]
        # extremes). XY position is irrelevant here -- see the comment
        # above: the capability's centring step erases it -- so the mesh is
        # built at the same effective (centred) coordinates the host rect
        # above uses, for readability, not because it changes anything.
        top_z = layer_height_mm * n_layers + layer_height_mm
        mesh = box_mesh_mm(
            effective_min_x_mm, effective_min_y_mm, 0.0,
            effective_min_x_mm + width_mm, effective_min_y_mm + depth_mm, top_z,
        )
        model_object = identity_model_object(mesh)
    return PrintObject(object_id, layers, model_object=model_object)


def _load_make_fixtures_module():
    """Load ``tools/make_fixtures.py``'s own NACA 4-digit section generation
    code — same pattern ``tests/unit/test_local_geometry.py`` and
    ``tests/unit/test_validate.py`` use, reused here rather than writing a
    second airfoil implementation. Reads the file only;
    never modifies it."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "make_fixtures_under_test_fake_orca",
        _os.path.join(REPO_ROOT, "tools", "make_fixtures.py"),
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def naca2412_outline_mm(chord_mm=60.0, span_mm=60.0, n_points=120):
    """The exact 2D section outline (mm, centred on its own bounding box) for
    NACA 2412 — same parameters as
    ``tests/fixtures/airfoil/naca2412_c60_s60.stl``,
    ``tests/unit/test_local_geometry.py``, and
    ``tests/unit/test_validate.py``'s ``_naca2412_polygon()``. Returns
    ``(outline_mm, bbox)``, both in millimetres, CCW-wound per
    ``make_fixtures.airfoil_triangles``'s own convention."""
    mf = _load_make_fixtures_module()
    m, p, t = mf.parse_naca4("2412")
    _triangles, outline_mm, bbox = mf.airfoil_triangles(m, p, t, chord_mm, span_mm, n_points)
    return outline_mm, bbox


def make_airfoil_object(
    object_id,
    n_layers,
    layer_height_mm,
    surface_type=SurfaceType.stInternal,
    chord_mm=60.0,
    span_mm=60.0,
    n_points=120,
):
    """Build a ``PrintObject`` with ``n_layers`` identical constant-section NACA
    2412 layers — the fixture's cambered-section analogue of
    ``make_rect_object``. Every layer shares the SAME 2D outline (a genuinely
    constant section — see module docstring KNOWN
    DIVERGENCES (14)).

    also attaches an identity-placed prism ``model_object`` (same outline,
    extruded root-to-tip) so the mesh/host alignment check has a real
    cross-section to agree with — see ``make_rect_object``'s identical note.
    """
    outline_mm, bbox = naca2412_outline_mm(chord_mm, span_mm, n_points)
    scaled_pts = tuple((to_scaled(x), to_scaled(y)) for x, y in outline_mm)

    layers = []
    for i in range(n_layers):
        print_z = round(layer_height_mm * (i + 1), 6)
        slice_z = round(print_z - layer_height_mm / 2.0, 6)
        expoly = ExPolygon._wrap(_Polygon(pts=scaled_pts))
        surface = Surface(surface_type, expoly)
        region = LayerRegion(SurfaceCollection([surface]))
        layers.append(Layer(print_z, [region], slice_z=slice_z))

    top_z = layer_height_mm * n_layers + layer_height_mm
    # Constant section: outline(z) is identical at z=0 and z=top_z, so a
    # 2-station loft (a prism) is exact for every slice_z in between -- no
    # per-layer station is needed.
    mesh = loft_mesh_mm([(0.0, outline_mm), (top_z, outline_mm)])
    return PrintObject(object_id, layers, model_object=identity_model_object(mesh)), bbox


def make_tapered_naca_object(
    object_id,
    n_layers,
    layer_height_mm,
    root_chord_mm,
    tip_chord_mm,
    span_mm,
    surface_type=SurfaceType.stInternal,
    n_points=120,
):
    """Build a ``PrintObject`` whose section CHANGES per layer: a linear,
    straight-leading-edge taper from ``root_chord_mm`` at Z=0 to
    ``tip_chord_mm`` at Z=``span_mm`` — the tip-closure geometry
    (``tests/fixtures/tip/README.md``, ``tools/make_fixtures.py``'s
    ``make_tip``), built here per-layer rather than as a 3D loft mesh (no STL
    import path exists in this fake host), reusing
    ``make_fixtures.build_loft_stations`` — the SAME per-station
    outline/centring code ``make_tip`` itself calls — so this is not a second
    taper implementation, just a different (2D, per-print-layer) consumer of
    it.

    ``pivot_frac_x=0.0`` and ``params(z) = (chord(z), 0.0, 0.0)`` match
    ``make_tip`` exactly: a straight leading edge, no sweep, no twist, only
    chord shrinking linearly with Z. Every layer is centred on the ROOT
    station's own bounding box (``build_loft_stations``'s ``root_shift``),
    the same convention the shipped tip fixture's README documents.

    The HOST's own per-layer outline is UNCHANGED -- still taken at each
    layer's own ``print_z``, exactly as for a mesh-less object (the mesh is
    derived from whatever section the fake host already returns; it never
    changes what the host itself returns). ``Layer.slice_z`` is recorded too (``print_z - layer_height_mm /
    2`` -- the plane ``capability.py._section_plane_for`` actually sections the
    MESH at), and the MESH is built so that its cross-section AT each layer's
    ``slice_z`` reproduces that SAME host outline: the mesh is this taper's
    ``params`` queried half a layer height HIGHER than its own local Z, i.e.
    ``mesh_section(z) == outline_from_params(z + layer_height_mm / 2)``, so
    ``mesh_section(slice_z_i) == outline_from_params(slice_z_i +
    layer_height_mm / 2) == outline_from_params(print_z_i)`` -- the exact host
    outline for that layer.

    The mesh loft REUSES the host layers' own outlines verbatim, one mesh
    station per host layer, bracketed by a true root (z=0) and an
    extrapolated tip (z=top_z) station. A two-station (root/tip) loft would
    reproduce the SHAPE of a linear taper, but it leaves a small residual
    after ``capability.py``'s XY-centring step; see the inline comment above
    ``mesh_stations``.
    """
    mf = _load_make_fixtures_module()
    m, p, t = mf.parse_naca4("2412")

    def params(z):
        frac = z / span_mm
        chord = root_chord_mm + (tip_chord_mm - root_chord_mm) * frac
        return chord, 0.0, 0.0

    print_z_values = [round(layer_height_mm * (i + 1), 6) for i in range(n_layers)]
    top_z = print_z_values[-1] + layer_height_mm
    half = layer_height_mm / 2.0

    stations, outline_unit, root_shift = mf.build_loft_stations(
        m, p, t, n_points, print_z_values, params, pivot_frac_x=0.0
    )

    layers = []
    slice_zs = []
    for print_z, (_z, outline_mm) in zip(print_z_values, stations):
        slice_z = round(print_z - half, 6)
        slice_zs.append(slice_z)
        scaled_pts = tuple((to_scaled(x), to_scaled(y)) for x, y in outline_mm)
        expoly = ExPolygon._wrap(_Polygon(pts=scaled_pts))
        surface = Surface(surface_type, expoly)
        region = LayerRegion(SurfaceCollection([surface]))
        layers.append(Layer(print_z, [region], slice_z=slice_z))

    # The mesh loft REUSES the host layers' own outlines verbatim, one mesh
    # station per host layer at that layer's own ``slice_z`` -- not a
    # separate 2-station (root/tip only) approximation queried at Z's
    # shifted by ``half``.
    #
    # WHY A 2-STATION APPROXIMATION IS NOT USED: with ``capability.py``'s
    # XY-centring, PrintObject centres its ENTIRE mesh on ONE global XY
    # bounding-box centre, computed once over every station, and applies it
    # uniformly to every layer -- correct, and exactly what this fake's own
    # per-layer host outlines already do via the single shared
    # ``root_shift`` (``build_loft_stations``'s own doc: "positioned
    # relative to the root section's own bounding-box centre ... rather
    # than each station re-centring itself"). But the 2-station mesh
    # approximation queried ``params`` at ``half``/``top_z + half`` -- NOT
    # at the root's true ``z=0`` -- so its own two stations' chords were
    # each a hair narrower than the ``chord0`` the shared ``root_shift`` was
    # actually computed from. That made the 2-station mesh's OWN global XY
    # bbox centre a hair off (0, 0); because the centring step subtracts that
    # slightly-off centre from every section, it leaves a small per-layer
    # residual (~0.0475 mm, over the 0.02 mm tolerance).
    #
    # Reusing the host's own already-built outlines (bit-identical, same
    # ``root_shift``) for every interior mesh station removes the
    # discrepancy at its source rather than re-tuning a tolerance: sectioning
    # the mesh at any layer's own ``slice_z`` lands exactly ON that
    # layer's own mesh station, which IS that layer's own host outline.
    # Bracketed by one station at ``z=0`` (the true root, ``params(0.0)`` --
    # the same station ``root_shift`` itself was derived from, so its own
    # bbox is EXACTLY centred) and one beyond the last ``slice_z`` at
    # ``z=top_z`` (extrapolated tip), so every ``slice_z`` stays strictly
    # interior to the mesh's own Z range, as ``MeshSectioner.section_at``
    # requires.
    root_chord, root_rot, root_dx = params(0.0)
    root_outline = mf.build_station_outline(
        outline_unit, root_chord, root_rot, 0.0, root_dx, 0.0, root_shift
    )
    tip_chord, tip_rot, tip_dx = params(top_z)
    tip_outline = mf.build_station_outline(
        outline_unit, tip_chord, tip_rot, 0.0, tip_dx, 0.0, root_shift
    )
    mesh_stations = (
        [(0.0, root_outline)]
        + [(sz, st[1]) for sz, st in zip(slice_zs, stations)]
        + [(top_z, tip_outline)]
    )
    mesh = loft_mesh_mm(mesh_stations)
    return PrintObject(object_id, layers, model_object=identity_model_object(mesh))
