"""``StructureGenerator``: turns a ``LayerFrame`` + ``StructureSpec`` into web intent.

Produces **intent only** -- centerline geometry and a requested width. No
destructive boolean (subtraction/offset) is performed here; the actual cut is
owned by ``slot.py`` and the capability's ``ops.diff`` call. ``WebIntent``
carries no slot-specific fields (no overshoot, no cap style), so the same
intent can be realized either as a subtractive slot or as a direct extrusion
path.

The public ``generate()`` signature takes (frame, spec, wall width) and no
``GeometryOps`` argument, but ``frame.web_centerline()`` and the local worst-case
measurements query the ``GeometryOps`` the ``LayerFrame`` was built with
(``ops.bounds``, ``ops.polygon_from_points``, ``ops.intersection``). Those are
read-only geometry queries, not the boolean subtraction the "no Booleans" claim
is about; ``test_generator.py``'s ``test_no_destructive_ops_invoked`` pins what
is guaranteed.

Placement
---------
``generate()`` requires a ``LayerContext`` (layer index + print_z) as an explicit
positional argument. It has no default so that a call site that omits it fails
with a ``TypeError`` instead of silently using "layer 0", which would collapse
every layer's Z-varying pattern onto the same station.

``generate()`` resolves ``width_mm`` (via ``spec.resolve_slot_gap``) before
measuring surfaces, because the local worst-case measurements take the slot's
footprint width rather than probing a single centreline (see ``frame.py``).

``generate()`` loops over ``spec.effective_families()``: one synthetic family
built from the spec's top-level fields when ``spec.families`` is empty, so the
single-family case goes through the same code path. Each candidate station is
generated per family, using that family's pitch, position and resolved gap.
``WebIntent``/``OmittedWeb`` carry ``family_index``/``family_name``. The final
``intents``/``omitted`` lists are the union across families, in family order;
because every family's slots end up in the same ``slot_polygons`` list, the
validator's pairwise collision check also covers webs from different families.

Each family's anchor station is shifted by ``family.station_drift_mm(print_z)``
(``spec.py``) before marching by ``rib_pitch_mm``. The whole family shifts
together, so parallel shear panels become parallel diagonal members as Z rises
instead of fanning apart. The drift is a pure function of ``print_z`` (never
accumulated layer over layer), so the same Z always yields the same stations.
With neither ``z_drift_mm_per_mm`` nor ``z_period_mm`` set, the drift is ``0.0``.
A drifted station that lands off the chord extent or in a station that is too
thin is handled by the ordinary omission machinery below; drift only moves
where a candidate sits before that machinery runs.

Candidates are measured station by station with ``local_upper_worst_case`` /
``local_lower_worst_case`` rather than ``frame.web_centerline``, which raises on
a station that cannot fit the requested margin. Raising is right for a single
mandatory web but wrong for an optional additional rib, so a station that does
not fit is recorded as an ``OmittedWeb``. See ``StructureGenerator`` for the
anchoring rule and omission policy, and ``GenerationResult`` for the return
shape (``intents`` + ``omitted``).

Depth
-----
A station that passes the skin-margin check can still be too thin for a
specific family's ``web_depth_mm``: the depth measured from the entry surface
plus the required margin may exceed the local thickness even though a shallower
web would fit. The generator never resolves this by deepening the web toward the
far surface (that would risk cutting through). What differs is whether the
unplaceable station stays loud or becomes an omission, and that follows
``family.is_marching`` (``WebFamily``'s single predicate for this split; testing
``rib_pitch_mm`` directly misclassifies a chord-relative family). It is the same
boundary ``_candidate_stations`` uses for "one named station" vs. "a marched
sequence".

- An explicit single-station family (``not family.is_marching``) named this
  chord position, so a depth breach there is the user's one request failing and
  raises ``ValueError`` naming the family and station. This module has no layer
  index or Z to name, so ``capability.py``'s ``except (ValueError,
  ArithmeticError)`` handler around ``generator.generate()`` attaches them, as
  it does for a station ``local_span`` cannot measure.
- A marching family generates its own stations arithmetically out to the chord
  extent and cannot steer around a locally thin station. A depth-limited request
  that fails for lack of local thickness is therefore recorded as an
  ``OmittedWeb`` ("skin margin"-prefixed), the same way a full-depth request
  at that station is. Otherwise a marching shallow family would be unusable on
  any tapered section whose thin band is wider than the pitch (5.86 mm,
  85.15-94.92% chord, on the NACA 2412 c60 fixture), since no phase offset can
  dodge a band wider than the spacing.

Thin-section omission
---------------------
A web can pass both the chord-extent and skin-margin checks (there is room for
``skin_margin_mm`` of remaining skin) and still be rejected by the validator's
erosion check, because "the margin fits" and "the margin survives the
safety-offset erosion" are different questions. Example: on a NACA 2412 truss
slice (layer 29, Z=6.00 mm) a full-depth web at x=-29.0 mm, 1.0 mm from the
leading edge, left exactly ``skin_margin_mm`` (1.0 mm) of skin under the slot,
more than ``2 * safety_offset_mm`` (0.84 mm) by a flat reading, yet the
whole-layer erosion disconnected the material there.

Derivation. ``validate.py``'s connectivity check erodes the whole transformed
layer by ``safety_offset_mm`` (``ops.offset(transformed, -safety_offset_mm)``),
so every material boundary, including the slot's cut edges, recedes by
``safety_offset_mm`` along its local normal, not vertically. The skin under a
slot is bounded by the slot's far edge (erodes inward) and the section's lower
surface (erodes inward), the same two-boundary reasoning behind
``TopologyValidator``'s ``_min_wall_mm = 2 * safety_offset_mm``. Reusing
``2 * safety_offset_mm`` here keeps the two checks consistent.

The flat check ``remaining_skin_mm >= 2 * safety_offset_mm`` misses a lateral
term: erosion also widens the slot's effective footprint by ``safety_offset_mm``
on each side (width ``w`` behaves like ``w + 2 * safety_offset_mm``). The
remaining skin is therefore measured across that wider "halo" span, using the
same "measure across the actual footprint" discipline as
``local_lower_worst_case`` / ``local_upper_worst_case``. On a flat or gently
curved lower surface the halo reading and the slot-span reading agree (to the
``DEFAULT_WORST_CASE_SAMPLES`` sampling residual), so nothing changes there,
including rectangular-coupon fixtures. Near the leading edge the lower surface
rises steeply within the halo, so the halo reading reports markedly less skin.

Scope gate: the check fires only when ``spec.skin_margin_mm >= 2 *
safety_offset_mm``. Below that, even an infinitely thick flat section cannot
satisfy it (halo reading equals slot-span reading, both ``skin_margin_mm``);
that is a configuration mistake (the margin is too small for the wall width, and
the fix is a larger margin). The public ``generate()`` entry point never reaches
it: ``spec.validate(safety_offset_mm=...)`` raises ``SpecError`` before any web
is placed when ``skin_margin_mm <= 2 * safety_offset_mm``. The gate is still
exercised directly against ``_generate_family``, bypassing ``spec.validate()``
(``tests/unit/test_thin_section.py``,
``test_thin_section_never_fires_below_the_scope_gate``).

Like the other worst-case-over-a-span measurements, this is a documented
approximation of the true 2D erosion, not a proof: a true worst point strictly
between two of the ``DEFAULT_WORST_CASE_SAMPLES`` samples can be missed. It is
deliberately not a re-simulation of ``ops.offset`` on the cut layer (this module
has only intent, no whole-layer geometry); it stays a closed-form,
backend-neutral inequality.

``safety_offset_mm`` is a keyword-only argument of ``generate()``, defaulting to
``outer_wall_line_width_mm``. ``capability.py`` builds both
``TopologyValidator``'s ``safety_offset_mm`` and this module's
``outer_wall_line_width_mm`` from the same resolved wall width, but they answer
different questions (erosion distance vs. slot-gap-ratio base), so a caller that
needs them to differ must say so.

Cross-family node merge
-----------------------
Suppressing one family's web at a node was tried and rejected. The 45-degree
families and the overlap-only merge rule cannot both hold at practical layer
heights, so every genuine crossing on a real airfoil lands in a band that fails
closed. Omitting one web leaves the suppressed member terminating into void,
not into solid material, and an alternating or loudly reported victim does not
fix that. Instead, nearby cross-family candidates are merged.

Where two cross-family candidates on the same surface are closer than ``2 *
safety_offset_mm`` (the validator's ``_min_wall_mm``, called ``min_wall_mm``
here too), i.e. ``gap < min_wall_mm``, the whole connected group of mutually
close candidates becomes one ``WebIntent`` spanning the union of their
footprints, ``[component_min_x, component_max_x]``, at the shallowest depth of
the group. No station moves and no family is dropped; this models a real node
where members meet and become one slot. Overlapping pairs (``gap < 0``) join
the merge only when ``spec`` is supplied and their entry Y values differ by more
than ``spec.overshoot_mm``; otherwise each overlapping candidate keeps its own
footprint and depth. Merging overlapping slots at different resolved depths
matters because leaving those depths in place cut a material island on a real
elevon slice (layer 926).

Same-family proximity is not merged: a same-family pair, however close, is never
linked and still fails closed in ``TopologyValidator``. Pairs entering from
opposite surfaces, and scar or socket candidates, are never linked either.

The merge forms connected components, not a pairwise scan with restart.
Candidates are sorted once by footprint ``min_x`` and every pair in that order
is linked when it passes those guards and its edge gap is below ``min_wall_mm``.
No re-scan is needed: a component's outer edges are its extreme members' own
original edges (a union takes min/max, it does not move them), so the gap to a
candidate just outside the component equals the gap that candidate already had
against that extreme member, which the single pass already consulted. See
``_merge_cross_family_nodes`` for the union-find mechanics and the
order-independence argument.

The scan compares every pair, not only consecutive ones. The
``family_index``/``entry_lower``/``is_scar`` guards reject a pair regardless of
gap, so a rejected candidate sitting between two candidates that belong in one
component would hide that pair from an adjacent-only scan. This occurred on a
host slice of ``presets/plugin/anisogrid_lattice_config.json`` (layer 5,
Z=1.20 mm): two same-role, same-``entry_lower`` lattice candidates were masked by
a third family anchored at the same X (shared ``web_position``, opposite
``z_drift_mm_per_mm``) on the opposite surface. Candidate counts per layer are
small, so the O(n^2) cost is immaterial, and each inner scan stops once ``gap``
clears the band (it is non-decreasing with sorted rank).
``_yield_lattice_to_primary`` has a pre-pass whose linking test must stay
identical to this one, and it also scans every pair.

The merge lives here, not in ``TopologyValidator``, because it changes which webs
are emitted (one merged intent instead of two) rather than how emitted geometry
is judged. It runs as a post-pass over every family's decided intents
(``_merge_cross_family_nodes``, called from ``generate()`` after the per-family
loop), because the check is inherently cross-family.

Far-end clearance across the merged interval: a merged slot can be much wider
than either constituent, so its far end must clear the local lower surface at
every x in the union interval. When a ``LayerFrame`` is available (the real
``generate()`` path, not the bare ``[intents]``-only calls in
``tests/unit/test_node_merge.py`` that pin the pure X-placement arithmetic), the
merged entry and far-end Y values are re-measured with
``local_upper_worst_case``/``local_lower_worst_case`` over the merged interval's
full span, the same primitive ``_generate_family`` uses for a single station.
``validate.py``'s invariant 4 already measures worst-case skin across the
removed region's full X span, so the validator side is satisfied once the merge
yields one ``WebIntent`` and one cut. A breach here (the merged interval cannot
hold ``skin_margin_mm`` across its full width although each constituent held it
across its narrower one) raises a ``TopologyError`` naming the layer index and Z,
so direct ``aircraft_core`` callers (the sweep tools) can classify it, and never
silently narrows the merge.

Halo / thin-section interaction: the halo span is ``width_mm + 2 *
safety_offset_mm``, so a merged slot's halo is wider than either constituent's.
A merge that was fine for both members can fail the thin-section erosion check
once merged. Every constituent already passed its own halo check in
``_generate_family``, so a merged-halo breach always fails closed with the same
wrapping as above instead of going through the ``OmittedWeb`` channel; omitting
the merge would silently delete both members.

Reporting: a merged ``WebIntent`` carries ``merged_family_indices``, the sorted,
deduplicated tuple of every family that fed the component (e.g. ``(0, 1)``), so a
caller can tell a merged node from an ordinary single-station web. Unmerged
intents carry the empty tuple. ``capability.py``/``diagnostics.py`` count merges
this way for ``node_merge_count``. A merge produces no ``OmittedWeb``, so no
``"node suppression:..."`` reason ever appears in ``omitted``.

Full-depth headroom
-------------------
A full-depth web's far end is placed at ``local_lower_worst_case(...) +
skin_margin_mm + FULL_DEPTH_MARGIN_HEADROOM_MM``. With zero headroom, the
generator's reading and ``validate.py``'s independent re-measurement over the
actual removed region could disagree by floating-point-scale amounts (observed:
1.0e-6 mm on the tip-taper fixture, web_position=0.5, layer 101, Z=20.40 mm) and
fail the margin on a genuinely thick section. This module cannot make its span
identical to the validator's, since it never performs the boolean cut. See
``FULL_DEPTH_MARGIN_HEADROOM_MM`` for the derivation and rejected alternatives;
this is a placement fix, not a loosened tolerance (``validate.py``'s
``EPSILON_MM`` and margin comparison are unchanged).
"""

import math
from dataclasses import dataclass, field, replace
from typing import FrozenSet, List, Mapping, NamedTuple, Optional, Tuple

from .frame import (
    AEROFOIL_REGION,
    DEFAULT_FEATURE_ASPECT_RATIO_MAX,
    DEFAULT_FEATURE_FILL_RATIO_MIN,
    DEFAULT_MAX_ROTATION_DEG,
    DEFAULT_PROBE_WIDTH_MM,
    DEFAULT_TILT_THRESHOLD_DEG,
    FEATURE_REGION,
    LayerFrame,
    ReEntrantSectionError,
    check_rotation_envelope,
    classify_region,
    corrected_skin_margin_mm,
    estimate_far_surface_slope_deg,
    far_cap_mitre_clearance_mm,
    estimate_local_tilt_deg,
    local_lower_worst_case,
    local_solid_thickness_worst_case,
    local_span,
    local_upper_worst_case,
    opposing_face_lateral_clearance_mm,
    opposing_pair_meets,
)
from .spec import (
    FUSED_FLANK_MAX_GAP_MM,
    ROLE_PRIMARY_STRUCTURE,
    ROLE_SKIN_LATTICE,
    SPAR_CHANNEL_POLICY_NEAREST_STATION,
    StructureSpec,
    WebFamily,
)
# ``TopologyError`` is raised by the merge-halo breach below. Safe at module
# scope -- ``validate`` imports ``frame``/``ops``/``spec`` but never this module,
# so there is no cycle, and ``TopologyError`` is referenced only at call time
# inside a method, so the single-file bundle's flattening order cannot matter.
from .validate import TopologyError

# Tolerance for the footprint-vs-chord-extent comparison, in millimetres.
# Deliberately tiny, mirroring validate.py's EPSILON_MM, so a station whose
# footprint lands EXACTLY on the chord extent (the single-web case, where
# web_position=0.5 on a rectangle can place the footprint exactly centred) is
# never spuriously omitted by floating-point noise.
_FOOTPRINT_EPSILON_MM = 1e-6

# Tolerance for the depth-breach comparison, in millimetres. Deliberately
# tiny, mirroring _FOOTPRINT_EPSILON_MM and validate.py's EPSILON_MM, so a
# depth that lands EXACTLY on the margin (e.g. web_depth_mm chosen to equal
# the section's own available depth) is never spuriously refused by
# floating-point noise.
_DEPTH_EPSILON_MM = 1e-6

# Tolerance for the thin-section (halo erosion) comparison, in
# millimetres. Same role and magnitude as the other module-level epsilons
# above: a station whose halo reading lands EXACTLY on
# ``2 * safety_offset_mm`` is never spuriously omitted by floating-point
# noise.
_THIN_SECTION_EPSILON_MM = 1e-6

# Stable marker inside the omission reason emitted when
# ``far_cap_mitre_clearance_mm`` rejects a candidate. Counted by the
# capability so a mitre-specific omission is visible in the run summary,
# not only in an offline sweep. The predicate can only ever ADD
# rejections, so this count is the whole cost of the mitre model -- if it
# is zero, the model cost nothing on that wing.
MITRE_OMISSION_MARKER = "skin margin: mitre clearance:"

# Tolerance for the node-merge minimum-wall comparison, in millimetres. Same
# role and magnitude as the other module-level epsilons above: a cross-family
# pair whose gap lands EXACTLY on ``2 * safety_offset_mm`` is treated as
# clearing the minimum wall (no merge), never spuriously merged by
# floating-point noise. Mirrors ``validate.py``'s own ``EPSILON_MM`` at the
# same comparison.
_NODE_SUPPRESSION_EPSILON_MM = 1e-6

# Headroom added to a FULL-DEPTH web's far end, beyond ``skin_margin_mm``
# exactly, in millimetres.
#
# WHY THE HEADROOM EXISTS: placing a full-depth web's far end at EXACTLY
# ``local_lower_worst_case(...) + spec.skin_margin_mm`` (the generator's own
# worst-case-across-the-INTENDED-span reading, plus the margin, with nothing
# left over) leaves no slack. ``validate.py`` independently
# re-measures the same quantity over the ACTUAL REMOVED REGION's own span
# (the only honest thing it can measure post-cut — see its own docstring),
# which is not, and cannot be made to be, bit-identical to the generator's
# intended span: this module never performs the boolean cut itself (the "no
# Booleans" contract at the top of this file), so it cannot know the
# post-boolean removed-region geometry at placement time. On real geometry
# an in-host slice of the tip-taper fixture (web_position=0.5, layer 101,
# Z=20.40 mm, chord 40.6 mm) found the two readings disagree by exactly
# 1.0e-6 mm — enough, with zero headroom, to fail invariant 4 on a section
# that is genuinely thick.
#
# Rejected alternatives:
#   - Make the generator measure over the SAME span the validator will use.
#     Impossible in principle, not just inconvenient: that span is a
#     property of the post-boolean cut (the removed region's own bounding
#     box), which does not exist until the boolean this module is
#     forbidden from performing has run.
#   - Make the validator measure over the generator's INTENDED span instead
#     of the removed region's actual one. Rejected because it stops being
#     an independent check — it would trust the generator's claimed
#     footprint rather than verify what was actually cut, silently passing
#     a real under- or over-cut that never reached the requested width.
#   - Loosen EPSILON_MM / the margin comparison in validate.py. Rejected
#     because that hides the disagreement rather
#     than removing it, and would blur the SAME comparison invariant 4
#     uses to catch a genuine margin violation (see
#     tests/unit/test_validate.py's margin-violation fixtures, unweakened).
#
# THE DERIVATION (three independent, bounded sources of disagreement
# between the generator's and the validator's worst-case readings, none of
# them eliminable by this module alone):
#   (i)   Cap/corner quantization: ``SlotBuilder`` hands the validator's
#         eventual removed-region boundary through a real backend, which
#         rounds every coordinate to its scaled-integer grid before any
#         boolean runs (``plugin/aircraft_structures/capability.py``'s
#         ``OrcaGeometryOps._to_scaled``). Up to 0.5 grid units of
#         rounding per coordinate, and ``SCALING_FACTOR`` is documented as
#         high as 1e-5 mm (large beds) — up to ~5e-6 mm of irreducible cap
#         displacement, already larger than EPSILON_MM (1e-6 mm) on its
#         own, on a backend this module never touches.
#   (ii)  Station-set shift: a removed-region span offset by (i) shifts
#         ``local_lower_worst_case``'s/``local_upper_worst_case``'s sampled
#         stations by a comparable amount; against the steepest local slope
#         this codebase exercises that is on the same order, ~1e-5 mm.
#   (iii) Sampled-worst-case residual: ``DEFAULT_WORST_CASE_SAMPLES``'s own
#         measured, documented ceiling of "well under 1e-4 mm" for how far
#         a discrete sample set can miss the true extremum between
#         stations, on the highest-curvature fixture this codebase
#         exercises.
#
# These three sum to a bit over 1e-4 mm; ``2e-4`` covers that sum with
# margin, and is not otherwise tuned to any single fixture's numbers.
FULL_DEPTH_MARGIN_HEADROOM_MM = 2e-4

# Measured hard-fuse gap, in millimetres: 0.021 mm on a 0.42 mm PLA
# wall was "fused; takes real effort to pull apart" in the printed test
# table. ``_generate_opposing_pair_family`` uses this as the ADDITIONAL
# clearance, beyond one fin's own thickness, that a pair's default centre
# spacing carries so the space BETWEEN the two fins' flanks also fuses,
# exactly as measured (0.88 mm centres -> 0.019 mm flank gap, fused).
_HARD_FUSE_GAP_MM = FUSED_FLANK_MAX_GAP_MM

# Minimum DERIVED per-side depth an opposing pair is built at, in millimetres.
# Below this a "pair" would be built with visually and structurally negligible
# reach -- not a topology hazard (the derivation below already guarantees
# safety at any positive depth), but not a useful member either. Chosen as one
# order of magnitude above ``_DEPTH_EPSILON_MM``'s own floating-point-noise
# scale, so a station that is merely numerically non-negative (a hair above
# zero from subtraction noise) is still refused as "not useful", not silently
# accepted.
_MIN_USEFUL_OPPOSING_PAIR_DEPTH_MM = 0.05

# A section thinner than two wall widths
# cannot contain a web between two skins at all -- this is a fixed geometric
# fact about the wall the caller already resolved
# (``outer_wall_line_width_mm``), NOT an independently-tunable threshold.
# Example from a real elevon wing STL, layer 1247 (print_z 249.6 mm,
# slice_z 249.5 mm): the mesh section there is a knife-edge sliver, extent
# 0.0064 x 1.8076 mm, area 0.00577 mm^2 -- aspect ~282 (classify_region's own
# AEROFOIL_REGION), yet the smaller extent (0.0064 mm) is over 100x SMALLER
# than even a single 0.42 mm wall, let alone two. That is not "a real aerofoil
# whose pitch config collapsed"; it is a section with no material to build a
# wall or a web into.
PRINTABLE_FLOOR_WALL_MULTIPLIER = 2.0


class DetectedChannel(NamedTuple):
    """One spar channel RECOVERED FROM THE CAD MODEL for a single layer.

    The model, not the config, is the source of truth for spar-socket
    geometry. Pure geometric data, no Orca types -- the plugin
    cross-sections the original (pre-Spiral-Vase) mesh
    (``aircraft_core.mesh_section``) and hands the resulting ``Loop``s in as
    instances of this type, one per interior loop it judges to be a spar-tube
    channel at this layer's Z. This module never sections anything itself; it
    only consumes what it is given.

    Lives HERE, not in ``spec.py``: ``StructureSpec`` is explicitly "pure
    data plus validation... nothing here touches geometry" (that module's
    own docstring) and carries POLICY (clearance, neck width, whether the
    config fallback is allowed) -- never a per-layer geometric MEASUREMENT.
    A ``DetectedChannel`` is exactly that measurement, freshly produced
    every layer, the same category of per-layer input ``LayerContext``
    itself already is (this module, not ``spec.py``, is where per-layer
    geometry meets policy) -- so it belongs beside ``LayerContext``, not
    beside ``StructureSpec``/``WebFamily``.

    Attributes:
        center_x_mm: Channel centroid X (chordwise), in millimetres, in the
            same coordinate frame as everything else this module places
            (``LayerFrame``'s own X).
        center_y_mm: Channel centroid Y, in millimetres, same frame.
        diameter_mm: Channel equivalent diameter, in millimetres (mirrors
            ``mesh_section.Loop.equivalent_diameter_mm``).
    """

    center_x_mm: float
    center_y_mm: float
    diameter_mm: float


# per-layer spar-channel status, see ``StructureGenerator.spar_channel_status``
# and ``aircraft_core.spar_latch``.
SPAR_STATUS_ABSENT = "absent"
SPAR_STATUS_FIT = "fit"
SPAR_STATUS_NOT_FIT = "not_fit"


@dataclass(frozen=True)
class LayerContext:
    """Per-layer identity threaded into ``StructureGenerator.generate()``.

    Gives the generator the layer it is working on. A small, immutable,
    REQUIRED argument, so it cannot be forgotten at a call site (see the
    module docstring's "Placement" section).

    Attributes:
        layer_index: 0-based index of this body layer, exactly as the host
            adapter enumerates ``ctx.object.layers()``
            (``plugin/aircraft_structures/capability.py``). Not consumed by
            the Z-variation formula itself (``WebFamily.station_drift_mm``
            depends on ``print_z`` only, deliberately — see that method's
            docstring), but carried through for diagnostics/error messages
            and anything that needs layer ordering rather than Z height.
        print_z: This layer's Z height in millimetres, exactly as the host
            reports it (``Layer.print_z``). ``WebFamily.z_drift_mm_per_mm``
            / ``z_period_mm`` are a pure function of this value -- i.e. of
            distance from the BED, not from the wing root. TODO: on a
            tip-face-down print the drift therefore starts at the tip; the
            run report's ``root_end.drift`` records this.
        detected_channels: Zero or more ``DetectedChannel`` entries
            recovered from the CAD model at THIS layer's Z, via mesh
            -sectioning done entirely outside this module (the plugin owns
            calling ``aircraft_core.mesh_section`` on the original mesh and
            filtering to interior loops; this module only consumes the
            result). ``()`` (the default) means no channel was detected.
            ``StructureGenerator.generate`` treats a non-empty tuple as the
            model overriding ``spec.spar_diameter_mm``/``spar_station``
            entirely -- see that method's own docstring for the full
            precedence.
    """

    layer_index: int
    print_z: float
    detected_channels: Tuple[DetectedChannel, ...] = ()
    # Optional per-web depth cap in mm, keyed by web identity
    # ``(family_index, k, side)`` (see ``WebIntent.web_id``). ``None`` (the
    # default) means no caps. A cap only ever SHALLOWS a
    # slot: it is applied where ``_generate_family`` resolves ``end_y`` (the
    # depth_to_socket, full-depth and explicit-depth seams), before every
    # per-station check, so a capped web is re-judged by the same gates as any
    # other. Not part of equality/hash/repr (it is an instruction, not layer
    # identity).
    depth_caps: Optional[Mapping[Tuple[int, int, str], float]] = field(
        default=None, compare=False, repr=False
    )


# web identity sides. ``U``/``L`` = the entry skin of a lattice, flank or tie
# member; ``S`` = a socket (one identity per family -- its entry side may
# change between layers, which must not read as a new web); ``P`` = a whole
# opposing pair (only on an OMISSION record, where both members go together).
WEB_SIDE_UPPER = "U"
WEB_SIDE_LOWER = "L"
WEB_SIDE_SOCKET = "S"
WEB_SIDE_PAIR = "P"


def web_side(entry_lower: bool) -> str:
    """identity side letter for an entry surface."""
    return WEB_SIDE_LOWER if entry_lower else WEB_SIDE_UPPER


@dataclass(frozen=True)
class WebIntent:
    """One requested internal web, as pure geometric intent.

    Attributes:
        start: Point on the exterior boundary where the web begins.
        end: Point at the far end — ``skin_margin_mm`` PLUS
            ``FULL_DEPTH_MARGIN_HEADROOM_MM`` short of the opposite skin for
            a full-depth web (``web_depth_mm is None`` — see that
            constant's docstring for why the extra headroom is required),
            or ``web_depth_mm`` in from ``start`` along the centerline for
            a depth-limited web.
        width_mm: Requested web width in millimetres (the resolved slot
            gap, for subtractive realization).
        family_index: Index (0-based, in ``spec.effective_families()``
            order) of the family that produced this web. Defaulted so
            single-family callers that construct a
            ``WebIntent`` directly can omit it. For a merged node
            (see ``merged_family_indices`` below) this is a REPRESENTATIVE
            value only, not necessarily "the" family — see
            ``_merge_cross_family_nodes``'s own docstring for how the
            representative is chosen and why.
        family_name: The producing family's ``WebFamily.name``. Same
            representative-only caveat as ``family_index`` for a merged
            node.
        merged_family_indices: Empty for an ordinary, unmerged web. For a
            union-interval merge, the sorted, deduplicated tuple of every
            family whose candidate fed the merged component — e.g.
            ``(0, 1)`` for a two-family node. This is what lets a caller
            tell a merged web apart from an ordinary one without inferring
            it from width alone (see ``capability.py``'s
            ``node_merge_count``).
        entry_lower: Which exterior surface this web enters from, the
            same meaning as ``WebFamily.entry_lower`` (``False`` is +Y,
            ``True`` is -Y). Carried onto
            the intent (not just the producing family) so
            ``_merge_cross_family_nodes`` can refuse to merge a +Y candidate
            with a -Y candidate — see that method's own docstring for why a
            same-edge merge must never cross entry sides.
        is_scar: ``True`` for the one synthetic candidate that bridges a
            declared interior hole to the exterior boundary
            (``StructureGenerator._generate_scar``), ``False`` (default)
            for an ordinary web. A
            scar's far end deliberately terminates INSIDE a void (the
            hole), never against solid material, so it must never be folded
            into union-interval merge (``_merge_cross_family_nodes``
            excludes any pair where either side is a scar, the same way it
            already excludes a same-edge merge across opposite entry sides)
            and it must never be the side omitted by the opposing-entry
            conflict check (``_omit_opposing_conflicts`` — a declared hole
            is a mesh feature, not a candidate the generator chose to
            place, so unlike an ordinary web it has no softer "omit it"
            outcome; see ``StructureSpec.expected_hole_count``'s docstring).
    """

    start: Tuple[float, float]
    end: Tuple[float, float]
    width_mm: float
    family_index: int = 0
    family_name: str = "default"
    merged_family_indices: Tuple[int, ...] = ()
    entry_lower: bool = False
    is_scar: bool = False
    # when set, SlotBuilder produces a KEYHOLE -- a neck of ``width_mm`` from
    # ``start`` down to a circular socket of this radius centred on ``end`` --
    # instead of a rectangle. Used for the carbon-spar socket, which must be
    # boundary-connected because Spiral Vase deletes interior holes before the
    # plugin runs. ``None`` for every ordinary intent.
    socket_radius_mm: Optional[float] = None
    # ``True`` iff this intent's depth was DERIVED from a detected spar socket
    # (``WebFamily.depth_to_socket``, read defensively via ``getattr``)
    # rather than from ``web_depth_mm``/full-depth placement. ``False`` for
    # every ordinary family. Exists so the socket-footprint exclusion
    # (``StructureGenerator.generate``) can exempt a member that was
    # deliberately positioned against THIS socket -- see that exclusion's own
    # comment for why an epsilon-narrow band cannot be trusted to arbitrate a
    # self-referential placement.
    references_socket: bool = False
    # ``True`` iff this socket is
    # KNOWN to be the real spar without disambiguation -- built either from the
    # ONLY detected channel on this layer, or from the config fallback
    # (``_generate_spar_socket``). ``False`` (the default)
    # whenever the socket was chosen by
    # ``spar_channel_policy="nearest_spar_station"`` DISAMBIGUATING among two
    # or more detected channels -- that pick is good enough to CUT a socket (an
    # interior loop still has to reach the boundary, or Spiral Vase breaks;
    # pure topology, no semantic claim required), but it is NOT good enough to
    # ANCHOR STRUCTURE to, because nothing establishes that the chosen channel
    # is semantically the spar rather than an alignment bore that merely won a
    # proximity contest against ``spar_station``. This is the flag
    # ``depth_to_socket``/ ``socket_flank_offset_mm`` families and
    # ``capability.py``'s spar-socket latch must all consult before treating a
    # socket as structural, rather than keying on
    # ``family_name``/``family_index`` alone (a bore-mislabelled-as-spar
    # carries the SAME ``family_name="spar_socket"``/``family_index=-2`` as a
    # genuine one). Irrelevant (left ``False``) for a bore socket
    # (``family_name= "bore_socket"``) and for every non-socket intent -- only
    # a ``family_name=="spar_socket"`` intent's value here is ever consulted.
    spar_confirmed: bool = False
    # web identity: ``(family_index, k, side)`` where ``k`` is the integer
    # lattice station index from ``_candidate_stations_k`` (``x = anchor + k *
    # pitch``) and ``side`` is ``U``/``L`` (entry skin) or ``S`` (socket). A
    # flank has ``k == 0`` (single station). ``None`` for an intent built
    # outside the family loops (feature lattice, direct test construction). A
    # merge's ``web_id`` is its representative constituent's, and
    # ``merged_web_ids`` / ``merged_web_xs`` list EVERY constituent's id and
    # own pre-merge station X (parallel tuples). Identity only: excluded from
    # equality/hash/repr so no existing comparison can see it.
    web_id: Optional[Tuple[int, int, str]] = field(
        default=None, compare=False, repr=False
    )
    merged_web_ids: Tuple[Tuple[int, int, str], ...] = field(
        default=(), compare=False, repr=False
    )
    merged_web_xs: Tuple[float, ...] = field(
        default=(), compare=False, repr=False
    )

    def is_tilted(self) -> bool:
        """``True`` iff this intent's centerline is not a plain vertical segment
        (``start[0] != end[0]``). ``_generate_family`` only
        moves ``end``'s X when the tilt correction is both enabled and accepted
        -- see that method's own comment -- so otherwise ``start[0] == end[0]``
        exactly and this is ``False``."""
        return self.start[0] != self.end[0]

    def footprint_x(self) -> Tuple[float, float]:
        """This intent's true X footprint: the requested slot's own min/max X
        across its FULL centerline (``start`` to ``end``), widened by
        ``width_mm / 2`` on each side.

        For an untilted intent (``start[0] == end[0]``) this is exactly
        ``start[0] -/+ width_mm/2``, since ``min(start_x, end_x)`` and
        ``max(start_x, end_x)`` both collapse to ``start_x``.

        For a tilted intent this is a CONSERVATIVE (never an under-count)
        bound on the true rotated-rectangle footprint, not its exact bbox:
        the true rectangle's X extent is
        ``width_mm * cos(theta) + |end_x - start_x|``, which is at most
        ``width_mm + |end_x - start_x|`` (``cos(theta) <= 1``) -- exactly
        what this returns. Erring wide, never narrow, is the safe direction
        for every caller of this method (a collision/merge/opposing-conflict
        GATE: judging two candidates as further apart than they truly are is
        the dangerous direction, judging them as closer is merely
        conservative).
        """
        half_w = self.width_mm / 2.0
        start_x, end_x = self.start[0], self.end[0]
        if start_x == end_x:
            return start_x - half_w, start_x + half_w
        return min(start_x, end_x) - half_w, max(start_x, end_x) + half_w

    def depth_mm(self) -> float:
        """True Y-reach of this intent's own slot, from its entry boundary to its
        deepest point.

        The centerline's own ``abs(end[1] - start[1])`` is exact for the
        DEPTH quantity the opposing-entry proof needs (see the comment at this
        intent's own construction site in ``_generate_family`` for why Y is
        unaffected by tilt), but MEASURED on real post-boolean geometry
        (``TopologyValidator``, which reads the actually-removed region's
        own bbox directly -- exact by construction, tilted or not) a
        tilted slot's true Y-extent is slightly MORE than the centerline
        alone: the slot polygon's WIDTH also contributes to Y once the
        centerline is not vertical (the far corners sit
        ``width_mm/2 * |dx|/length`` further from the entry boundary than
        the centerline's own endpoint -- the same perpendicular-offset
        geometry ``SlotBuilder.build`` already computes for the real
        polygon, restated here in closed form rather than re-derived via a
        ``SlotBuilder`` call).

        This matters in practice: on the NACA 2412 skin-lattice fixture at
        ``max_web_tilt_deg=12``, using the centerline-only depth would let
        ``_omit_opposing_conflicts`` judge a pair SAFE by its own
        (under-counted) depths while ``TopologyValidator`` -- reading the
        true, wider removed bbox -- correctly finds they meet (layer 12,
        x=[13.469, 14.330] mm: required 3.8911 mm against 3.8897 mm
        available). Equal to ``abs(end[1] - start[1])`` whenever
        ``start[0] == end[0]``.
        """
        dy = abs(self.end[1] - self.start[1])
        start_x, end_x = self.start[0], self.end[0]
        if start_x == end_x:
            return dy
        start_y, end_y = self.start[1], self.end[1]
        length = math.hypot(end_x - start_x, end_y - start_y)
        if length == 0.0:
            return dy
        half_w = self.width_mm / 2.0
        return dy + half_w * abs(end_x - start_x) / length

    def reach_depth_mm(self, overshoot_mm: float) -> float:
        """The Y-extent of the RAW slot polygon ``SlotBuilder.build`` actually
        produces for this intent, including the ``overshoot_mm`` extension past
        the entry boundary.

        Why this differs from ``depth_mm()``: ``SlotBuilder.build`` extends the near
        (entry-side) end of the slot rectangle past ``start`` by
        ``overshoot_mm`` "so the cut reaches cleanly through the skin"
        (that method's own docstring) -- and ``start`` is itself placed at
        a WORST-CASE (locally lowest-reach) boundary point across the
        slot's own width, so the true local surface elsewhere across that
        width commonly sits beyond ``start``. On real (curved) geometry
        this overshoot band therefore often survives ``ops.intersection``
        in full, not merely at a clipped sliver — measured directly on the
        default skin-lattice preset (tapered NACA 2412 fixture, layer 55):
        ``depth_mm()`` reports 1.5000 mm for a family with
        ``web_depth_mm=1.5``, but ``TopologyValidator`` -- reading the
        actually-removed region's own bbox -- measures 1.5500 mm, exactly
        ``depth_mm() + overshoot_mm`` (``overshoot_mm=0.05`` there).
        ``depth_mm()`` never accounts for this at all (it measures only
        ``start``-to-``end``, the REQUESTED structural depth), so feeding
        it to ``_omit_opposing_conflicts`` silently UNDER-predicts the
        depth the validator can measure by up to ``overshoot_mm`` per
        side. This method closes that gap by computing the raw slot
        polygon's own Y bbox in closed form (mirroring
        ``SlotBuilder.build``'s corner geometry exactly -- if that method's
        corner formula is revised, this one must be revised with it)
        rather than calling ``SlotBuilder`` (this module has no
        slot-builder dependency; see the module docstring).

        PROVABLY AN UPPER BOUND, not merely an empirical match:
        ``ops.intersection(original, slot)`` is, by definition of intersection,
        a subset of ``slot`` itself, so the actually-removed region's own bbox
        can never exceed the raw slot polygon's bbox this method computes.
        Using this value (instead of ``depth_mm()``) as each side's depth in
        the opposing-entry omission decision therefore makes the GENERATOR's
        own meeting-condition inputs dominate the VALIDATOR's: for the SAME
        footprint/overlap-span reasoning ``footprint_x()``'s own docstring
        already establishes ("erring wide, never narrow" is the safe direction
        here too), any pair the validator would find meets, the generator
        finds meets (and omits) first. This extends the
        shared-predicate guarantee to the INPUT level, not just the
        predicate's own arithmetic.

        Equal to ``depth_mm() + overshoot_mm`` whenever
        ``start[0] == end[0]``; a tilted intent uses the general corner
        formula so the same "no under-count" guarantee holds.
        """
        start_x, start_y = self.start
        end_x, end_y = self.end
        dx, dy_raw = start_x - end_x, start_y - end_y
        length = math.hypot(dx, dy_raw)
        if length == 0.0:
            return self.depth_mm()
        ux, uy = dx / length, dy_raw / length
        py = ux
        overshoot_y = start_y + uy * overshoot_mm
        half_w = self.width_mm / 2.0
        y_max = max(overshoot_y, end_y) + abs(py) * half_w
        y_min = min(overshoot_y, end_y) - abs(py) * half_w
        return y_max - y_min

    def family_membership(self) -> FrozenSet[int]:
        """This web's full family MEMBERSHIP set — ``{family_index}`` for an
        ordinary (unmerged) web, or the whole ``merged_family_indices`` for a
        merge.

        ``family_index`` alone is only a REPORTING-level representative for a
        merged web (see ``_merge_cross_family_nodes``'s docstring, "A
        component's merged WebIntent gets a REPRESENTATIVE" — a fixed rule,
        always the lowest ``family_index`` among the component's members), not
        necessarily the constituent that actually borders which edge. Comparing
        two webs' bare ``family_index`` values for a SAFETY decision (such as
        ``TopologyValidator``'s pairwise collision check) can therefore
        classify a genuine same-family collision as cross-family purely because
        the representative labels happened not to match — silently weakening
        the invariant. Callers building ``TopologyValidator.validate``'s
        ``family_indices`` argument should use this accessor, not
        ``family_index`` directly, so that safety decision is made against
        actual membership — see ``validate.py``'s ``_family_membership`` (the
        receiving side) for the intersection rule this feeds.
        """
        if self.merged_family_indices:
            return frozenset(self.merged_family_indices)
        return frozenset((self.family_index,))


class Degradation:
    """One structural feature DROPPED so the layer could still be built.

    Distinct from ``OmittedWeb``, which records a candidate web station that
    was never viable. A ``Degradation`` records a feature that WAS requested
    and could not be placed, where the alternative is refusing the whole slice.
    The generator places what fits and reports the rest.

    NEVER SILENT. Every instance must reach the run report; that is the
    entire licence for degrading rather than failing closed (a layer may be
    degraded only if it is explicitly marked as such).

    Attributes:
        kind: Stable machine-readable slug, e.g. ``"spar_socket"``.
        x_mm: Chord station the dropped feature would have occupied.
        reason: Human-readable cause, with the measured numbers in it.
    """

    # NOT a @dataclass, deliberately, unlike every other record in this
    # module. It sits directly above ``OmittedWeb``, and a decorator line
    # placed above it would bind to the wrong class ("OmittedWeb() takes no
    # arguments"). __slots__ plus an explicit __init__ gives
    # immutability-in-practice with no decorator to misplace. Do not turn it
    # into a dataclass without moving the class.
    __slots__ = ("kind", "x_mm", "reason")

    def __init__(self, kind, x_mm, reason):
        self.kind = kind
        self.x_mm = x_mm
        self.reason = reason

    def __repr__(self):  # pragma: no cover - diagnostic only
        return "Degradation({!r}, x={:.4f}, {!r})".format(
            self.kind, self.x_mm, self.reason)


@dataclass(frozen=True)
class OmittedWeb:
    """One candidate web station that was considered but not produced.

    Omission is deterministic and always recorded here, never silent (see
    ``StructureGenerator``'s docstring for the policy).

    Attributes:
        x_mm: Chord station (X) of the candidate web centerline.
        reason: Human-readable cause. Currently one of five categories,
            named at the start of the string so a caller can match on it
            without parsing the rest: ``"chord extent:..."`` (the slot's
            requested footprint does not lie entirely within the section's
            chord extent), ``"skin margin:..."`` (the local material at
            that station cannot accommodate ``skin_margin_mm``),
            ``"thin section:..."`` (the margin fits, but the remaining
            skin cannot survive the validator's own safety-offset erosion
            — see the module docstring's "Thin-section omission"), or
            ``"opposing entry:..."`` (this candidate
            would meet an opposing-surface web closely enough to trip the
            no-sever proof's meeting condition
            (``frame.opposing_pair_meets``); see ``_omit_opposing_conflicts``
            for the full derivation and why omission, not shifting, is the
            fix), or ``"role yield:..."`` (``_yield_lattice_to_primary`` —
            a ``skin_lattice`` candidate that would otherwise merge
            with a ``primary_structure`` candidate on the same edge is
            omitted instead, so the primary member's own depth is never
            truncated by the merge; see that method's own docstring and
            ``_merge_cross_family_nodes``'s "WHY OMISSION, NOT 'MERGE AT
            THE DEEPER DEPTH'" note). ``"skin margin:..."`` and ``"thin
            section:..."`` are
            both THICKNESS reasons — a caller deciding whether a layer is
            eligible for pattern shutdown treats them the same way
            (``"chord extent:..."``, ``"opposing entry:..."`` and
            ``"role yield:..."`` are NEITHER a positional miss nor a
            thinness finding and are never shutdown-eligible — see
            ``capability.py``'s
            ``thinness_prefixes``). A depth request that cannot be
            honoured is a DIFFERENT, fail-closed case (``ValueError``, not
            an omission) — see the module docstring. A node merge produces
            no ``OmittedWeb`` at all (see the module docstring's
            "Cross-family node merge" section and ``merged_family_indices``
            on ``WebIntent``).
        family_index: Index of the family this candidate belonged to.
        family_name: The candidate's family's ``WebFamily.name``.
    """

    x_mm: float
    reason: str
    family_index: int = 0
    family_name: str = "default"
    # the station index and web identity of the candidate this
    # omission belongs to (see ``WebIntent.web_id``); ``None`` when the
    # omission has no single web. A whole omitted opposing pair carries side
    # ``P``; an omitted merge carries the merge representative's id plus
    # every constituent in ``merged_web_ids``. Excluded from equality/hash/
    # repr: identity only.
    k: Optional[int] = field(default=None, compare=False, repr=False)
    web_id: Optional[Tuple[int, int, str]] = field(
        default=None, compare=False, repr=False
    )
    merged_web_ids: Tuple[Tuple[int, int, str], ...] = field(
        default=(), compare=False, repr=False
    )


def _per_station_reentrant(spec) -> bool:
    """The per-station re-entrant catch is on when ``per_station_reentrant`` is
    set OR ``z_continuity`` is (which forces it)."""
    return bool(
        getattr(spec, "per_station_reentrant", False)
        or getattr(spec, "z_continuity", False))


def _stamp_omitted(omitted: List["OmittedWeb"], start: int, web_id) -> None:
    """Stamp ``web_id``/``k`` onto the omissions appended since index
    ``start`` (the ones one station iteration produced). In place; never adds,
    removes or reorders a record."""
    if web_id is None:
        return
    for j in range(start, len(omitted)):
        rec = omitted[j]
        if rec.web_id is None:
            omitted[j] = replace(rec, k=web_id[1], web_id=web_id)


def _cap_end_y(local_entry_y: float, end_y: float, sign: float, cap) -> float:
    """Depth-cap seam: shallow ``end_y`` so its depth below ``local_entry_y`` is
    at most ``cap``. ``cap is None`` returns ``end_y`` untouched; a cap NEVER
    deepens."""
    if cap is None:
        return end_y
    if sign * (local_entry_y - end_y) > cap:
        return local_entry_y - sign * max(cap, 0.0)
    return end_y


@dataclass(frozen=True)
class GenerationResult:
    """Everything ``StructureGenerator.generate`` decided for one layer.

    ``intents`` is the UNION of every family's webs to actually build,
    concatenated in family order and ascending chord (X) order within each
    family; with one implicit family it is the plain ascending-X list.
    ``omitted`` is every candidate station that was considered but not
    produced, each with a reason (omission is recorded, never silent) and its
    producing family.

    A layer where every candidate was omitted returns ``intents=[]`` here
    rather than raising: this module is backend-neutral and does not know
    the layer index or Z height a fail-closed error needs to name. The capability is responsible for
    treating a zero-web result as a failure -- see
    ``StructureGenerator.generate``'s own docstring ("OMISSION POLICY") and
    ``plugin/aircraft_structures/capability.py``'s ``_transform``, which is
    where the layer index and Z are known and the fail-closed error is
    actually raised.

    spar_fallback_used: ``True`` iff this layer's spar socket (if any)
        was synthesised from ``spec.spar_diameter_mm``/``spar_station``
        because no channel was detected in the model
        (``layer.detected_channels`` was empty) AND
        ``spec.allow_config_spar_fallback`` was set to permit it. ``False``
        (the default) for every layer with no socket and every layer whose
        socket came from a detected channel (the model winning, the normal
        case). This is what makes the debugging
        fallback impossible to use silently -- see
        ``StructureSpec.allow_config_spar_fallback``'s own docstring and
        ``StructureGenerator.generate``'s precedence section.
    """

    intents: List[WebIntent]
    omitted: List[OmittedWeb]
    spar_fallback_used: bool = False
    # Features DROPPED so this layer could still be built, each carrying its
    # measured cause. Empty by default. Never silent: the capability must surface every entry in the run report
    # -- that reporting is the entire licence for degrading rather than failing
    # closed.
    degradations: List["Degradation"] = field(default_factory=list)


def _feature_result_is_better(candidate: GenerationResult, baseline: GenerationResult) -> bool:
    """The "never worse" rule: is the dense-mode feature result ``candidate`` strictly
    better than the non-dense ``baseline``? See ``StructureGenerator.
    _generate_feature`` for how the two are produced.

    More teeth built wins outright: a feature is a load path, and one more
    surviving tooth is structurally significant even if that tooth is shallow
    (the non-dense bound can produce MORE, shallower teeth than a full-depth
    attempt whose per-station thin-section gate is stricter -- see
    ``_generate_feature_core``'s "MEASURED COUNTEREXAMPLE" comment: 3 teeth
    beats 0). Only on an EXACT tie in tooth count does depth decide, via
    ``max(depth_mm() for i in intents)`` -- the single deepest tooth, not the
    sum or the mean: the question is whether going dense let the feature hold
    more load, and the deepest surviving member is the one the interdigitation
    design changes (every non-dense tooth shares one family-wide
    ``web_depth_mm``, while dense per-station depth can vary member to
    member). An empty ``intents`` list has no teeth, so tooth count alone
    handles it; when both results are empty the tie returns False, preferring
    the baseline.
    """
    n_candidate = len(candidate.intents)
    n_baseline = len(baseline.intents)
    if n_candidate != n_baseline:
        return n_candidate > n_baseline
    depth_candidate = max((i.depth_mm() for i in candidate.intents), default=0.0)
    depth_baseline = max((i.depth_mm() for i in baseline.intents), default=0.0)
    return depth_candidate > depth_baseline


class _SectionTooSmallForPitch(ValueError):
    """A family's own ``rib_pitch_chord_frac`` has resolved to a pitch
    below the minimum-feature floor at this layer's local chord (see the
    raise site in ``_generate_family``; the message is the same, only the
    type differs).

    This is deliberately its OWN type, not a bare ``ValueError``, because
    ``StructureGenerator.generate`` needs to tell it apart from every
    OTHER ``ValueError`` the family loop can raise (a declared hole
    conflict, an opposing pair that cannot fuse, a depth breaching skin
    margin...). Those other errors mean "the requested structure is
    invalid" and must keep failing closed on any shape. THIS one means
    "the section is too small for the configured wing pitch to resolve at
    all" -- which is exactly the question the AEROFOIL/FEATURE classifier
    exists to answer, so ``generate()`` routes only this type to it
    instead of propagating immediately. Subclassing ``ValueError`` (not
    ``Exception``) keeps every existing ``except ValueError`` -- including
    ``capability.py``'s translator -- working unchanged for callers that
    have not been updated to know this type exists.

    CONSTRAINT, load-bearing: this type is raised at EXACTLY ONE site --
    the ``rib_pitch_chord_frac ... below the minimum-feature floor`` check
    in ``_generate_family`` -- and ``generate()`` catches it BY TYPE to
    route the layer to the feature-region fallback.

    Adopting this type at any other raise site silently grants that site
    fallback routing PAST a fail-closed path. That is the exact class of
    error the narrowing prevents: a broad ``except ValueError`` in
    ``generate()`` would convert three genuine fail-closed invariants
    (declared interior hole, opposing pair that cannot fuse, depth breaching
    skin margin) into feature lattices on any rectangular section. If a new
    site needs similar routing, give it its own type and its own handling.
    """


class StructureGenerator:
    """Produces the list of webs for one layer: zero or more per family, spaced
    by that family's own ``rib_pitch_mm``. The union of every family in
    ``spec.effective_families()`` is what turns a single row of full-depth webs
    into a multi-tier structure.

    ANCHORING RULE (must stay deterministic and reproducible across runs and
    across machines), applied INDEPENDENTLY per family: candidate web
    stations for family ``family`` at layer Z ``layer.print_z`` are

        x_k = frame.web_x(family.web_position)
              + family.station_drift_mm(layer.print_z)
              + k * family.rib_pitch_mm

    for every integer ``k`` (``k = 0`` is the anchor itself, then ``k = 1, 2,
    3...`` marching toward +X and ``k = -1, -2, -3...`` marching toward -X),
    each computed directly from ``k`` — never by repeatedly adding
    ``rib_pitch_mm`` to a running total — so floating-point drift cannot make
    the pattern depend on how many stations were produced before it or on
    iteration order. The Z-drift term is likewise computed directly from
    ``print_z`` (``WebFamily.station_drift_mm``, never accumulated per layer),
    so the whole formula depends only on ``family`` and ``layer.print_z`` — the
    same layer always yields the same stations, regardless of construction
    order. Marching in each direction stops as
    soon as a candidate's own X falls outside ``[frame.min_x, frame.max_x]``
    (there is no reason to keep considering stations that have left the
    section's chord extent entirely — that boundary is not itself recorded as
    an omission, since it is not a discrete candidate the pattern ever named,
    merely the natural end of an unbounded arithmetic sequence).

    ``family.rib_pitch_mm is None`` (the default) gives the single-web
    behaviour for that family: only ``k = 0`` is ever a candidate.

    OMISSION POLICY — deterministic omission of individual webs, never a
    fail-the-whole-layer policy for narrowness alone (see
    ``GenerationResult`` for what happens when every candidate is omitted).
    A candidate station is recorded as an ``OmittedWeb`` instead of being
    built, for the five reasons enumerated below (a "skin margin" depth
    -breach variant among them applies only to a MARCHING family's
    depth-limited stations — an explicit single-station family's depth
    breach still raises ``ValueError`` instead — see the module docstring's
    "Depth" section for the full split):

    1. "chord extent" — the slot's requested footprint
       ``[x - width/2, x + width/2]`` does not lie entirely within the
       section's chord extent ``[frame.min_x, frame.max_x]``. A candidate
       whose CENTRE is inside the extent (so marching did not stop) but
       whose WIDTH pushes one edge past it still cannot be built cleanly.
    2. "skin margin" — the worst-case local lower surface across the slot's
       width, plus ``spec.skin_margin_mm``, would reach or exceed the
       worst-case local upper surface there. This is the same worst-case
       -across-the-span measurement ``frame.web_centerline`` uses (and
       ``validate.py`` re-measures independently), just consulted here
       BEFORE committing to build the web rather than after, so a station
       too narrow to carry a rib is omitted rather than raising.
    3. "thin section" — the margin fits, but the remaining skin
       cannot survive the validator's own safety-offset erosion, measured
       across the WIDER halo span erosion actually reaches rather than the
       slot's own requested width. See the module docstring's "Thin-section
       omission" for the full derivation, and note its scope gate: this
       never fires when ``spec.skin_margin_mm < 2 * safety_offset_mm``
       (that regime violates ``TopologyValidator``'s precondition). It does
       not reach the validator through the public ``generate()`` entry point
       at all: ``spec.validate(safety_offset_mm=...)`` raises ``SpecError``
       before any web is placed whenever the precondition is violated. The
       gate's own internal behaviour is still exercised directly in tests;
       see the module docstring.
    4. "opposing entry" (``_omit_opposing_conflicts``) — a CROSS
       -FAMILY check, run once per layer AFTER every family's own loop
       above and after ``_merge_cross_family_nodes`` (the same-edge merge
       must resolve first, because the no-sever proof's meeting condition is
       evaluated on the final footprints). A +Y candidate and a
       -Y candidate whose haloed footprints overlap in X and would trip the
       meeting condition (``frame.opposing_pair_meets`` — the SAME
       predicate ``TopologyValidator`` uses for its own independent
       cross-edge check, so the generator's decision to omit and the
       validator's decision to fail closed can never disagree about which
       stations are unsafe) are never both built: the SHALLOWER of the two
       (ties broken by the larger ``family_index`` losing) is omitted
       instead, and the deeper survives untouched. See
       ``_omit_opposing_conflicts``'s own docstring for why omission — not
       shifting the colliding station sideways — is the fix, and why
       "shallower loses" is the general policy a tapered wing's fixed
       -fraction spar vs. fixed-mm-pitch stiffener collision (the finding
       this omission exists to fix) is one instance of.
    5. "role yield" (``_yield_lattice_to_primary``, a pre-pass run inside
       ``_merge_cross_family_nodes`` before its own union-find) — a SAME
       -EDGE, cross-ROLE check: where a ``skin_lattice`` candidate and a
       ``primary_structure`` candidate would otherwise fall into the
       sub-``min_wall_mm`` merge band, the lattice candidate is omitted
       instead of merged, so the primary member's own footprint and depth
       are never touched by the merge. See that method's own docstring and
       ``_merge_cross_family_nodes``'s "WHY OMISSION, NOT 'MERGE AT THE
       DEEPER DEPTH'" note for the full reasoning — this is the SAME
       "shallower yields, deeper survives untouched" policy reason 4
       states, applied to a same-edge conflict rather than a
       cross-edge one. A same-role pair (lattice-lattice or
       primary-primary), however close, is not affected by this check and
       goes through the ordinary merge.

    A station whose local geometry cannot be measured at all — no material
    there, or a re-entrant crossing — raises ``ValueError`` from
    ``local_span`` (via ``local_lower_worst_case``/``local_upper_worst_case``)
    rather than being omitted. This is deliberate: converting THAT
    ``ValueError`` into an omission would also swallow the fail-closed cases
    for a genuinely malformed section. In practice it cannot fire for a
    well-formed constant-section wing, because station iteration is already
    bounded to the section's own chord extent.
    """

    def generate(
        self,
        frame: LayerFrame,
        spec: StructureSpec,
        outer_wall_line_width_mm: float,
        layer: LayerContext,
        *,
        safety_offset_mm: Optional[float] = None,
        max_rotation_deg: Optional[float] = None,
    ) -> GenerationResult:
        """Return the webs (and any omissions) for ``frame`` at ``layer``.

        ``spec`` is validated here (``spec.validate()``) rather than trusted
        blindly, since a caller could hand generate() a spec that was never
        explicitly validated.

        ``layer`` is REQUIRED (see ``LayerContext``'s docstring and the module
        docstring's "Placement" section) — it is what lets each family's
        stations vary with Z via ``WebFamily.station_drift_mm``.

        ``safety_offset_mm`` (keyword-only) is the erosion distance the
        thin-section omission check measures against — see the module
        docstring's "Thin-section omission" section. Defaults to
        ``outer_wall_line_width_mm`` when not given, matching
        ``capability.py``'s practice of deriving both from the same
        resolved wall width, without this module assuming the two are
        always the same constant (a caller that needs them to diverge has
        to say so explicitly).

        ``max_rotation_deg`` (keyword-only) is the rotation-envelope threshold
        — see ``frame.check_rotation_envelope`` and
        ``frame.DEFAULT_MAX_ROTATION_DEG``. Defaults to
        ``spec.max_rotation_deg`` when that is set, else
        ``frame.DEFAULT_MAX_ROTATION_DEG`` — the same "explicit argument
        overrides the spec-resolved default" shape ``safety_offset_mm`` uses.

        Checked ONCE per layer, here, against ``frame.rotation_deg`` (a
        property of the whole section, not of any one family or station) —
        raises ``ValueError`` naming the measured angle and the threshold
        before any family is generated, rather than letting each family
        rediscover the same violation independently. The corrected margin
        (``frame.corrected_skin_margin_mm``) is likewise computed once here
        and threaded into every family's own margin arithmetic below, so
        every family agrees on the same effective margin for this layer.

        Loops over ``spec.effective_families()`` and concatenates each
        family's intents/omissions, in family order — see the module
        docstring.

        SPAR-SOCKET PRECEDENCE (the model is the source of truth for
        spar-socket geometry; config dictates only policy) — evaluated once per
        layer, in this strict order (the socket is built before the family loop
        so that a ``depth_to_socket`` family can size its members against it):

        (a) ``layer.detected_channels`` non-empty -> THE MODEL WINS. The
            socket's radius is ``detected_diameter_mm / 2 +
            spec.spar_clearance_mm`` (clearance stays POLICY, from config);
            its chordwise (X) position and Y centre come from the detected
            channel's own centroid, never from ``spec.spar_station``.
            ``spec.spar_diameter_mm``/``spec.spar_station`` are not
            consulted at all in this branch — the fit check that can still
            fail closed (section too shallow) runs against the DETECTED
            diameter, not any config value.
        (b) Exactly one detected channel: proceeds as in (a). MORE than one
            is a POLICY question the model alone cannot answer, resolved by
            ``spec.spar_channel_policy`` (see that field's own docstring):
            ``"refuse"`` (the default) raises ``TopologyError`` naming the layer index,
            ``print_z``, and EVERY detected channel's centre and diameter,
            never guessing (not the largest, not the nearest to
            ``spar_station``). ``"nearest_spar_station"`` instead declares
            the channel nearest ``spar_station`` (by chordwise position,
            measured against THIS layer's own frame) the spar and proceeds
            as in (a) for it alone; every OTHER detected channel on this
            layer is a plain bore, socketed by
            ``_generate_bore_socket_from_channel`` (same keyhole geometry,
            none of the spar's placement/latch semantics — see that
            method's own docstring). SPAR CHANNEL SELECTION: whichever
            branch runs, every family below this point sees
            ``layer.detected_channels`` narrowed to AT MOST the one chosen
            spar channel, never the raw multi-channel tuple, so
            ``depth_to_socket``/``socket_flank_offset_mm`` families can
            assume "exactly 0 or 1 channels".
        (c) No channel detected AND ``spec.spar_diameter_mm`` is set: this
            is a DEBUGGING FALLBACK, not a normal input.
            ``spec.allow_config_spar_fallback is True`` builds a
            config-only socket (same geometry and fit check as the detected
            case, against the CONFIG diameter) and records the fact on
            ``GenerationResult.spar_fallback_used`` so it is never silent;
            ``False`` (the default) raises ``TopologyError`` naming
            ``allow_config_spar_fallback`` as the switch.
        (d) No channel and no ``spar_diameter_mm``: no socket.

        Everything downstream of "there is (or is not) a socket" is
        independent of which branch produced it: the socket-footprint
        exclusion of overlapping lattice members, the merge/opposing-entry
        passes' treatment of the socket as a normal (unmergeable) intent, and
        the fail-closed shallow-section check.
        """
        # A ``skin_margin_mm`` that does not clear ``2 * safety_offset_mm``
        # voids the no-sever proof's connectivity argument. That is a
        # SPEC-level configuration mistake, not a per-layer geometry finding:
        # it is true (or false) before any layer is generated, so it is caught
        # here, once, before placement begins. Left to ``TopologyValidator``'s
        # erosion check it would surface several layers later and name the
        # wrong proximate cause (a disconnected island, not the
        # margin/wall-width pairing that guaranteed one). ``safety_offset_mm``
        # is resolved to its effective value BEFORE this call so the guard
        # always has a concrete number to check, matching every downstream
        # comparison in this method. The failure is a ``SpecError`` raised
        # here, not a ``TopologyError`` from the erosion check.
        if safety_offset_mm is None:
            safety_offset_mm = outer_wall_line_width_mm
        spec.validate(safety_offset_mm=safety_offset_mm)
        if max_rotation_deg is None:
            max_rotation_deg = (
                spec.max_rotation_deg
                if spec.max_rotation_deg is not None
                else DEFAULT_MAX_ROTATION_DEG
            )

        check_rotation_envelope(frame.rotation_deg, max_rotation_deg)
        effective_skin_margin_mm = corrected_skin_margin_mm(
            spec.skin_margin_mm, frame.rotation_deg
        )

        # Synthesise the carbon-spar socket. Computed BEFORE the family loop
        # so a ``depth_to_socket`` family can size its own members against THIS
        # layer's actual socket while it is being generated, not only exclude
        # against it afterward. It is added to ``intents`` (via the post-loop
        # block below) before the merge and opposing passes, so it
        # participates in both -- excluded from being merged (categorical
        # guards in ``_merge_cross_family_nodes``) but a legitimate winner of
        # any conflict, because a socket that loses is a spar that does not fit.
        #
        # Precedence (see this method's own docstring for the full derivation):
        # the MODEL wins whenever it has an answer
        # (``layer.detected_channels``); config-declared spar geometry is a
        # debugging fallback, gated on ``spec.allow_config_spar_fallback`` and
        # recorded loudly via ``spar_fallback_used`` whenever it actually
        # fires.
        socket = None
        degradations: List[Degradation] = []
        spar_fallback_used = False
        # Populated only under spec.spar_channel_policy ==
        # SPAR_CHANNEL_POLICY_NEAREST_STATION, one entry per detected
        # channel this layer that was NOT chosen as the spar -- see this
        # method's own docstring, clause (b).
        bore_sockets: List[WebIntent] = []
        if len(layer.detected_channels) > 1:
            if spec.spar_channel_policy == SPAR_CHANNEL_POLICY_NEAREST_STATION:
                spar_x = frame.min_x + spec.spar_station * frame.width
                channels = layer.detected_channels
                spar_idx = min(
                    range(len(channels)),
                    key=lambda idx: abs(channels[idx].center_x_mm - spar_x),
                )
                spar_channel = channels[spar_idx]
                # Narrow layer.detected_channels to the CHOSEN spar channel
                # alone before anything downstream (the (a)/(b) socket
                # branch immediately below, and every family in the loop
                # further down that reads layer.detected_channels) ever
                # sees it -- see this method's docstring, "SPAR CHANNEL
                # SELECTION".
                layer = replace(layer, detected_channels=(spar_channel,))
                socket, socket_degradation = (
                    self._generate_spar_socket_from_channel(
                        frame, spec, layer, spar_channel,
                        # This pick came from disambiguating among 2+
                        # candidates by proximity to spar_station alone -- good
                        # enough to cut the socket, never confirmed as the real
                        # spar. See WebIntent.spar_confirmed's own docstring.
                        spar_confirmed=False,
                    )
                )
                if socket_degradation is not None:
                    degradations.append(socket_degradation)
                for idx, channel in enumerate(channels):
                    if idx == spar_idx:
                        continue
                    bore_socket, bore_degradation = (
                        self._generate_bore_socket_from_channel(
                            frame, spec, channel
                        )
                    )
                    if bore_degradation is not None:
                        degradations.append(bore_degradation)
                    if bore_socket is not None:
                        # a bore's identity is its detected-channel ordinal
                        # (approximate: channels carry no stable id).
                        bore_socket = replace(
                            bore_socket,
                            web_id=(bore_socket.family_index, idx,
                                    WEB_SIDE_SOCKET),
                        )
                        bore_sockets.append(bore_socket)
            else:
                raise TopologyError(
                    layer.layer_index, layer.print_z,
                    "{} spar channels detected in the model at this layer: {} "
                    "-- which channel is the spar is a POLICY question the "
                    "model alone cannot answer. This is never guessed (not the "
                    "largest channel, not the nearest to spar_station) -- the "
                    "plugin must be told explicitly which channel is the "
                    "spar. Set spar_channel_policy=\"nearest_spar_station\" "
                    "to declare that the channel nearest spar_station is the "
                    "spar and every other detected channel is a plain bore, "
                    "or leave spar_channel_policy=\"refuse\" (the default, "
                    "this message) and change the model/sectioning so at "
                    "most one channel is ever detected per layer.".format(
                        len(layer.detected_channels),
                        "; ".join(
                            "channel {} at (x={:.4f}, y={:.4f}) mm, "
                            "diameter={:.4f} mm".format(
                                i, c.center_x_mm, c.center_y_mm, c.diameter_mm
                            )
                            for i, c in enumerate(layer.detected_channels)
                        ),
                    ),
                )
        elif len(layer.detected_channels) == 1:
            socket, socket_degradation = (
                self._generate_spar_socket_from_channel(
                    frame, spec, layer, layer.detected_channels[0],
                    # Unambiguous: exactly one channel, no disambiguation,
                    # so it is confirmed as the spar.
                    spar_confirmed=True,
                )
            )
            if socket_degradation is not None:
                degradations.append(socket_degradation)
        elif spec.spar_diameter_mm:
            if not spec.allow_config_spar_fallback:
                raise TopologyError(
                    layer.layer_index, layer.print_z,
                    "no spar channel was detected in the model at this "
                    "layer, and config declares spar_diameter_mm={!r}. "
                    "Config-declared spar geometry is a DEBUGGING FALLBACK "
                    "ONLY -- the model, not config, is the source of truth "
                    "for spar-socket geometry. Set "
                    "allow_config_spar_fallback=True to use the declared "
                    "geometry deliberately, or fix the model/sectioning so "
                    "the real channel is detected.".format(
                        spec.spar_diameter_mm
                    ),
                )
            socket, socket_degradation = self._generate_spar_socket(
                frame, spec, outer_wall_line_width_mm, layer)
            if socket_degradation is not None:
                degradations.append(socket_degradation)
            spar_fallback_used = True

        intents: List[WebIntent] = []
        omitted: List[OmittedWeb] = []
        # The aerofoil family loop can RAISE, not just return zero intents,
        # when ``rib_pitch_chord_frac`` resolves to a pitch below the
        # minimum-feature floor at this frame's size (``_SectionTooSmall
        # ForPitch``, raised at that one site in ``_generate_family``). The
        # error is captured so that a small FEATURE region can still reach the
        # classifier/lattice fallback below.
        #
        # The capture is DELIBERATELY narrowed to ``_SectionTooSmallForPitch``
        # alone, not bare ``ValueError``: a declared-hole conflict, an opposing
        # pair that cannot fuse, and a depth breaching skin margin are all
        # "the requested structure is invalid", which must keep failing
        # closed on ANY shape, never routed to the FEATURE/AEROFOIL
        # classifier. Only "this section is too small for the configured
        # wing pitch to resolve at all" is the "is this even a wing
        # section?" question the classifier exists to answer. A broader catch
        # would also collide with solid rectangular "coupon" fixtures (fill
        # ratio 1.0 always classifies FEATURE per ``classify_region``) and
        # silently discard their intentionally raised errors as soon as
        # ``_generate_feature`` had room to build a lattice.
        #
        # On catch, ``intents`` is reset to empty so this falls into the
        # same ``if not intents:`` fallback path as a family loop that
        # completed but produced nothing. ``aerofoil_error`` is re-raised
        # unchanged unless the fallback below classifies this region as
        # FEATURE and actually produces a lattice; for an AEROFOIL region, or
        # when the feature fallback declines, the original error propagates
        # (see the re-raise site after that block).
        aerofoil_error: Optional[_SectionTooSmallForPitch] = None
        try:
            for family_index, family in enumerate(spec.effective_families()):
                # An opposing pair is two members whose flanks FUSE and whose
                # depths OVERLAP. The flag is read defensively via ``getattr``
                # (same shape as ``depth_to_socket`` above). An
                # opposing-pair family takes a wholly separate generation
                # path -- see
                # ``_generate_opposing_pair_family``'s own docstring for why
                # its depth is never ``family.web_depth_mm``/full-depth
                # placement.
                if getattr(family, "opposing_pair", False):
                    fam_intents, fam_omitted = self._generate_opposing_pair_family(
                        frame,
                        spec,
                        family,
                        family_index,
                        outer_wall_line_width_mm,
                        layer,
                        safety_offset_mm,
                    )
                else:
                    fam_intents, fam_omitted = self._generate_family(
                        frame,
                        spec,
                        family,
                        family_index,
                        outer_wall_line_width_mm,
                        layer,
                        safety_offset_mm,
                        effective_skin_margin_mm,
                        socket,
                    )
                intents.extend(fam_intents)
                omitted.extend(fam_omitted)
        except _SectionTooSmallForPitch as exc:
            aerofoil_error = exc
            intents = []

        if socket is not None:
            # The socket occupies a footprint 2r wide -- an order of magnitude
            # more than a lattice web -- so any member landing on it collides.
            # It claims that space the same way a DECLARED HOLE already does,
            # by omitting the members that fall inside it rather than letting
            # the pair check fail the layer. Without this the wing fails with
            # "slots ... leave only 0.4762 mm of wall between them", which is
            # true but is a placement problem, not a topology one.
            #
            # The collision this guards against is TWO-dimensional (a keyhole
            # occupying an X band and a Y band), so the test requires both X and
            # Y overlap. An X-only test would omit a short member that sits
            # entirely above or below the socket, on the side of the circle
            # away from the neck, even though it cannot physically touch the
            # socket. The Y test is against the keyhole's own bounding
            # box: the neck runs from the entry skin (``socket.start[1]``) to
            # the circle centre (``socket.end[1]``), and the circle itself
            # extends ``socket_radius_mm`` either side of the centre, so the
            # keyhole's Y extent is ``[min(start_y, cy - r), max(start_y, cy +
            # r)]`` -- using the full neck run, not just the circle, because a
            # member entering on the SAME side as the neck can still collide
            # with it well outside the circle (see
            # ``test_m17_spar_socket.py``'s
            # ``SparSocketFootprintExclusionTests`` for the pinned same-side
            # case; ``test_m19_socket_exclusion.py`` pins the opposite-side
            # relaxation). This is a placement CONVENIENCE only --
            # ``TopologyValidator`` remains the actual safety net for a real
            # wall violation; a member that genuinely collides is still
            # omitted, with the reason below.
            keep, socket_omitted = [], []
            min_wall = 2.0 * safety_offset_mm
            lo = socket.start[0] - (socket.socket_radius_mm + min_wall)
            hi = socket.start[0] + (socket.socket_radius_mm + min_wall)
            cy = socket.end[1]
            r = socket.socket_radius_mm
            local_thickness_mm = self._socket_local_thickness_mm(
                frame, socket.start[0], r, spec)
            y_lo = min(socket.start[1], cy - r) - min_wall
            y_hi = max(socket.start[1], cy + r) + min_wall
            for i in intents:
                # A ``depth_to_socket`` member (``references_socket``) is
                # positioned RELATIVE TO this exact socket -- its far tip is
                # placed exactly ``skin_margin_mm`` short of the circle by
                # construction (``_generate_family``'s depth-to-socket branch).
                # Because ``StructureSpec.validate`` requires ``skin_margin_mm
                # > 2 * safety_offset_mm`` STRICTLY (the proof rests on a
                # 0.021 mm cushion -- see that method's own docstring), the tip
                # is always at least a hair outside this band's own edge under
                # exact arithmetic (at default settings, ~0.02 mm --
                # ``skin_margin_mm`` minus ``min_wall``), so this test alone
                # would not omit it. That margin is thin enough, and
                # this method's Y reading (``local_upper_worst_
                # case``/``local_lower_worst_case``, a sampled worst-case over
                # a probe window, not an exact analytic value on a real curved
                # section) is approximate enough, that relying on it to survive
                # is not something this convenience check should ever be
                # trusted to arbitrate. The member is exempted unconditionally
                # instead. It cannot be a false exemption: it was placed to
                # clear the circle by ``skin_margin_mm``, and
                # ``TopologyValidator`` still re-measures the real,
                # post-boolean geometry independently.
                if getattr(i, "references_socket", False):
                    keep.append(i)
                    continue
                a, b = i.footprint_x()
                my_lo, my_hi = min(i.start[1], i.end[1]), max(i.start[1], i.end[1])
                # The Y-overlap test above is WEAKER than the opposing-entry
                # rule, which RAISES for the socket rather than omitting. A
                # member entering from the opposite skin can therefore clear
                # this band in Y and still be rejected a moment later, failing
                # the whole slice -- observed at layer 461 near the tip, where
                # the socket's 12.79 mm neck plus a 1.50 mm lattice web plus the
                # 0.84 mm minimum wall exceed the local thickness. The socket
                # must win that contest (a spar that does not fit is not an
                # option), so the same criterion is applied here and the other
                # member is omitted first.
                socket_depth = abs(socket.end[1] - socket.start[1]) + r
                member_depth = abs(i.end[1] - i.start[1])
                opposite_side = bool(i.entry_lower) != bool(
                    getattr(socket, "entry_lower", False))
                m12_conflict = (
                    opposite_side
                    and b >= lo and a <= hi
                    and (socket_depth + member_depth + min_wall) > local_thickness_mm
                )
                if m12_conflict or (
                        b >= lo and a <= hi and my_hi >= y_lo and my_lo <= y_hi):
                    socket_omitted.append(OmittedWeb(
                        x_mm=i.start[0],
                        reason=(
                            "spar socket footprint: station overlaps the "
                            "{:.2f} mm carbon-spar socket at x=[{:.4f}, "
                            "{:.4f}] mm (socket radius {:.3f} plus {:.4f} mm "
                            "of wall each side). The socket is the load path "
                            "to the spar; a lattice member cannot share its "
                            "space.".format(
                                2.0 * socket.socket_radius_mm, lo, hi,
                                socket.socket_radius_mm, min_wall)),
                        family_index=i.family_index,
                        family_name=i.family_name,
                        k=None if i.web_id is None else i.web_id[1],
                        web_id=i.web_id,
                        merged_web_ids=i.merged_web_ids,
                    ))
                else:
                    keep.append(i)
            intents = keep
            omitted.extend(socket_omitted)
            intents.append(socket)

        # Same footprint-exclusion discipline as the spar socket's own
        # block immediately above, applied once per plain-bore socket
        # (``spec.spar_channel_policy == "nearest_spar_station"``, this
        # method's docstring clause (b)). A bore is a different WebIntent
        # than ``socket``, with its own family_name, so it is not folded into
        # the test above, because a bore's footprint must never exclude
        # ``socket`` itself or another bore's socket, only ordinary
        # lattice members, hence the ``i.socket_radius_mm`` guard below
        # (two sockets that genuinely overlap is a distinct, real defect
        # for TopologyValidator to catch on its own terms, not something a
        # placement convenience pass should silently resolve by dropping
        # one).
        for bore_socket in bore_sockets:
            keep, bore_omitted = [], []
            min_wall = 2.0 * safety_offset_mm
            lo = bore_socket.start[0] - (bore_socket.socket_radius_mm + min_wall)
            hi = bore_socket.start[0] + (bore_socket.socket_radius_mm + min_wall)
            cy = bore_socket.end[1]
            r = bore_socket.socket_radius_mm
            local_thickness_mm = self._socket_local_thickness_mm(
                frame, bore_socket.start[0], r, spec)
            y_lo = min(bore_socket.start[1], cy - r) - min_wall
            y_hi = max(bore_socket.start[1], cy + r) + min_wall
            for i in intents:
                if getattr(i, "references_socket", False) or i.socket_radius_mm:
                    keep.append(i)
                    continue
                a, b = i.footprint_x()
                my_lo, my_hi = min(i.start[1], i.end[1]), max(i.start[1], i.end[1])
                socket_depth = abs(bore_socket.end[1] - bore_socket.start[1]) + r
                member_depth = abs(i.end[1] - i.start[1])
                opposite_side = bool(i.entry_lower) != bool(
                    getattr(bore_socket, "entry_lower", False))
                m12_conflict = (
                    opposite_side
                    and b >= lo and a <= hi
                    and (socket_depth + member_depth + min_wall) > local_thickness_mm
                )
                if m12_conflict or (
                        b >= lo and a <= hi and my_hi >= y_lo and my_lo <= y_hi):
                    bore_omitted.append(OmittedWeb(
                        x_mm=i.start[0],
                        reason=(
                            "bore socket footprint: station overlaps the "
                            "{:.2f} mm bore socket at x=[{:.4f}, "
                            "{:.4f}] mm (socket radius {:.3f} plus {:.4f} mm "
                            "of wall each side). The bore is a plain "
                            "alignment feature socketed like the spar; a "
                            "lattice member cannot share its "
                            "space.".format(2.0 * r, lo, hi, r, min_wall)),
                        family_index=i.family_index,
                        family_name=i.family_name,
                        k=None if i.web_id is None else i.web_id[1],
                        web_id=i.web_id,
                        merged_web_ids=i.merged_web_ids,
                    ))
                else:
                    keep.append(i)
            intents = keep
            omitted.extend(bore_omitted)
            intents.append(bore_socket)

        # Scar: a third kind of candidate, neither an ordinary family web nor a
        # cross-family post-pass check. One synthetic WebIntent bridges the
        # declared interior hole to the exterior boundary (see
        # ``_generate_scar``). It is generated once per layer and added to
        # ``intents`` BEFORE the merge and opposing-entry passes below so it
        # participates in both -- excluded from ever being MERGED
        # (``_merge_cross_family_nodes``'s ``is_scar`` guard) but fully
        # eligible to WIN an opposing-entry conflict
        # (``_omit_opposing_conflicts``'s scar-never-loses priority).
        # ``spec.expected_hole_count == 0`` (the default) skips this entirely.
        if spec.expected_hole_count > 0:
            intents.append(
                self._generate_scar(
                    frame,
                    spec,
                    outer_wall_line_width_mm,
                    effective_skin_margin_mm,
                )
            )

        # Node merge: a cross-family post-pass, since no single family's own
        # loop above can see another family's stations. See the module
        # docstring's "Cross-family node merge" section. ``frame``/``spec``/
        # ``effective_skin_margin_mm`` are threaded through so a merged node's
        # far end and halo can be RE-MEASURED across the merged interval's own
        # full span (the far-end-clearance and halo obligations) rather
        # than reused from either constituent's own narrower reading.
        intents, node_merge_omitted = self._merge_cross_family_nodes(
            intents,
            safety_offset_mm,
            frame=frame,
            spec=spec,
            effective_skin_margin_mm=effective_skin_margin_mm,
            ctx=layer,
        )
        omitted.extend(node_merge_omitted)

        # A node merge widens the printed slot to the union of its
        # constituents. Each constituent passed the lateral check before
        # merging, but that does not prove the wider final footprint safe.
        # Measured in-host on the elevon wing at layer 848: two safe lower
        # candidates merged to a 0.5145 mm slot whose side wall severed an
        # island. Re-check the geometry that SlotBuilder will actually use.
        if spec.skin_margin_mm >= 2.0 * safety_offset_mm:
            kept_after_lateral_merge = []
            for intent in intents:
                if (
                    not intent.merged_family_indices
                    or intent.socket_radius_mm
                    or intent.is_scar
                ):
                    kept_after_lateral_merge.append(intent)
                    continue
                lateral_clearance_mm = opposing_face_lateral_clearance_mm(
                    frame.ops,
                    frame.polygon,
                    intent.start[0],
                    intent.width_mm,
                    intent.start[1],
                    intent.end[1],
                    intent.entry_lower,
                    safety_offset_mm,
                )
                # A merged footprint can reach a neck on the opposite
                # contour chain even though each constituent entered from
                # (and was checked against) the other chain. Layer 848 is
                # the measured counterexample. Scan that chain too, but
                # only for merged slots; doing this for every ordinary web
                # reintroduces the smooth-leading-edge false positives the
                # entry-chain restriction exists to prevent.
                opposite_chain_clearance_mm = opposing_face_lateral_clearance_mm(
                    frame.ops,
                    frame.polygon,
                    intent.start[0],
                    intent.width_mm,
                    intent.start[1],
                    intent.end[1],
                    not intent.entry_lower,
                    safety_offset_mm,
                )
                lateral_clearance_mm = min(
                    lateral_clearance_mm, opposite_chain_clearance_mm
                )
                # The validator erodes the surviving material from both
                # sides of a neck. For this opposite-chain, post-merge
                # case the stable bound is therefore the full erosion
                # diameter, not overshoot_mm (layer 850 measured 0.4998 mm
                # after dilation and still split with a 0.42 mm erosion).
                merged_clearance_required_mm = 2.0 * safety_offset_mm
                if lateral_clearance_mm > merged_clearance_required_mm:
                    kept_after_lateral_merge.append(intent)
                    continue

                member_families = [
                    spec.families[index]
                    for index in intent.family_membership()
                    if 0 <= index < len(spec.families)
                ]
                if any(
                    family.web_depth_mm is not None and not family.is_marching
                    for family in member_families
                ):
                    raise ValueError(
                        "merged family node {} at chord station x={:.4f} mm "
                        "leaves only {:.4f} mm of true (mitre-dilated) "
                        "LATERAL clearance from its final widened side wall "
                        "to an opposing section feature, at or below the "
                        "required merged clearance {:.4f} mm "
                        "(2 * safety_offset_mm) -- refusing "
                        "to place a merged explicit-depth web the "
                        "validator's own mitre erosion would then sever."
                        .format(
                            intent.family_membership(),
                            intent.start[0],
                            lateral_clearance_mm,
                            merged_clearance_required_mm,
                        )
                    )
                omitted.append(
                    OmittedWeb(
                        x_mm=intent.start[0],
                        reason=(
                            "skin margin: mitre clearance: merged node {} "
                            "passes its constituents' checks but its final "
                            "{:.4f} mm-wide slot leaves only {:.4f} mm of "
                            "true (mitre-dilated) LATERAL clearance from "
                            "its widened side wall to an opposing section "
                            "feature, at or below the required merged "
                            "clearance {:.4f} mm (2 * safety_offset_mm) "
                            "-- the merged station is "
                            "omitted, never "
                            "placed.".format(
                                intent.family_membership(),
                                intent.width_mm,
                                lateral_clearance_mm,
                                merged_clearance_required_mm,
                            )
                        ),
                        family_index=intent.family_index,
                        family_name=intent.family_name,
                        k=None if intent.web_id is None else intent.web_id[1],
                        web_id=intent.web_id,
                        merged_web_ids=intent.merged_web_ids,
                    )
                )
            intents = kept_after_lateral_merge

        # Opposing-entry omission: a second cross-family post-pass, run AFTER
        # the same-edge merge above (a pre-merge constituent's own depth is
        # not the depth that will actually be printed there, so this must see
        # POST-merge geometry). See ``_omit_opposing_conflicts``'s own
        # docstring and ``StructureGenerator``'s "OMISSION POLICY" reason 4.
        intents, opposing_omitted = self._omit_opposing_conflicts(
            intents, frame, safety_offset_mm, spec.overshoot_mm
        )
        omitted.extend(opposing_omitted)

        # Guard: primary structure must not vanish quietly.
        #
        # Every omission above is legitimate -- the web could not be placed
        # safely, so it was dropped rather than forced, and the layer that
        # results is genuinely valid. That is the right trade for a skin
        # stiffener and the WRONG one for a load path. Measured: a two-spar
        # config at 0.42 mm line width verified 595/595 layers with its rear
        # spar omitted at every single station, because nothing in the
        # pipeline was asked to object.
        #
        # Opt-in via ``require_primary_structure``; see that field's docstring
        # for why the default is off.
        if spec.require_primary_structure:
            lost = [
                o for o in omitted
                if self._family_role_of(o, spec) == ROLE_PRIMARY_STRUCTURE
            ]
            if lost:
                raise TopologyError(
                    layer.layer_index,
                    layer.print_z,
                    "primary structure omitted: {} (require_primary_structure "
                    "is set, so a load-carrying member that cannot be placed "
                    "fails the slice rather than degrading silently). Each "
                    "omission below is individually correct -- the member did "
                    "not fit -- but the part would otherwise verify while "
                    "missing a load path. Widen the placement envelope (a "
                    "thinner outer wall line width is the strongest lever -- "
                    "the feasible band is set by skin_margin_mm minus 2x the "
                    "line width"
                    "), move the station, or drop the primary role if the "
                    "member was never meant to carry load.".format(
                        "; ".join(
                            "{} at x={:.4f} mm ({})".format(
                                o.family_name, o.x_mm, o.reason
                            )
                            for o in lost
                        )
                    ),
                )

        # FALLBACK, consulted ONLY when the aerofoil path just run above
        # produced NO structure at all. "Is this region even a wing section?"
        # is only a meaningful question once the wing vocabulary has
        # demonstrably failed on it. Asking it of a region the aerofoil path
        # already served could misclassify the many synthetic rectangular
        # "coupon" fixtures the unit tests use as generic stand-ins for a
        # cross-section (they always get ordinary wing intents, so they never
        # reach here). See ``classify_region``'s docstring for the aspect/fill
        # separation this checks.
        #
        # A degenerate frame (``classify_region`` raising on zero width or
        # depth) DECLINES rather than propagating a new failure mode here
        # — the ordinary zero-intent degradation below is what such a
        # layer would have reached anyway; this fallback must never be the
        # thing that turns a layer newly fatal.
        #
        # ``aerofoil_error is not None`` is OR'd in here (not just
        # ``not intents``) because ``intents`` can be non-empty AFTER
        # the family loop's own catch resets it to ``[]``: the scar
        # block above (``spec.expected_hole_count > 0``) runs
        # unconditionally and can append a scar even though the ordinary family
        # request that was supposed to produce this layer's real structure
        # never ran. Gating on ``not intents`` alone would let that scar
        # make the layer look "served" and skip straight to the final
        # ``return`` below, silently discarding a genuine, fatal aerofoil-
        # path failure. Entering on the error alone costs nothing: the FEATURE
        # branch still requires ``_generate_feature`` to actually produce
        # intents before returning early, and the re-raise below covers every
        # other outcome, so a scar-bearing aerofoil failure still surfaces as
        # the original exception.
        if aerofoil_error is not None or not intents:
            # BEFORE classify_region gets a say, rule out
            # a section that is physically too thin to carry any wall or web at
            # all -- this only matters when the aerofoil path actually RAISED
            # (``aerofoil_error is not None``); the plain "no intents, no
            # error" case already reaches the same non-fatal zero-intent return
            # at the bottom of this method regardless of ``classify_region``'s
            # answer (see the region-conditional re-raise below: it only fires
            # when ``aerofoil_error is not None``). A high-aspect, low-fill
            # knife-edge sliver reads AEROFOIL under ``classify_region`` (see
            # its own docstring), which would otherwise re-raise a "pitch
            # collapsed" error for a section that was never a real aerofoil
            # pitch problem in the first place -- see
            # ``PRINTABLE_FLOOR_WALL_MULTIPLIER``'s own comment for the
            # measured elevon-tip case.
            if aerofoil_error is not None:
                min_extent_mm = min(frame.width, frame.depth)
                printable_floor_mm = (
                    PRINTABLE_FLOOR_WALL_MULTIPLIER * outer_wall_line_width_mm
                )
                if min_extent_mm < printable_floor_mm:
                    area_mm2 = frame.ops.area(frame.polygon)
                    degradations = degradations + [
                        Degradation(
                            kind="section_below_printable_floor",
                            x_mm=(frame.min_x + frame.max_x) / 2.0,
                            reason=(
                                "DEGRADED -- no structure on this layer: "
                                "the section's smaller in-plane extent is "
                                "{:.4f} mm, below the printable floor of "
                                "{:.4f} mm ({:.1f}x the {:.4f} mm wall line "
                                "width) -- width={:.4f} mm, depth={:.4f} mm, "
                                "area={:.6f} mm^2. A section this thin is "
                                "all skin; no web can exist here regardless "
                                "of the configured pitch, so this is not a "
                                "genuine wing-vocabulary failure: {}".format(
                                    min_extent_mm, printable_floor_mm,
                                    PRINTABLE_FLOOR_WALL_MULTIPLIER,
                                    outer_wall_line_width_mm,
                                    frame.width, frame.depth, area_mm2,
                                    aerofoil_error,
                                )
                            ),
                        )
                    ]
                    return GenerationResult(
                        intents=[],
                        omitted=omitted,
                        spar_fallback_used=spar_fallback_used,
                        degradations=degradations,
                    )
            feature_aspect_ratio_max = (
                spec.feature_aspect_ratio_max
                if spec.feature_aspect_ratio_max is not None
                else DEFAULT_FEATURE_ASPECT_RATIO_MAX
            )
            feature_fill_ratio_min = (
                spec.feature_fill_ratio_min
                if spec.feature_fill_ratio_min is not None
                else DEFAULT_FEATURE_FILL_RATIO_MIN
            )
            try:
                region = classify_region(
                    frame, feature_aspect_ratio_max, feature_fill_ratio_min
                )
            except ValueError:
                region = AEROFOIL_REGION
            # NOTE: the guarantee below covers ``_generate_feature`` too,
            # not only ``classify_region``. ``_generate_feature`` DECLINES
            # (returns no intents) for a feature too small to carry a
            # lattice rather than raising -- see its own pitch guard. If it
            # ever raises again, this fallback can turn a layer newly fatal
            # and the guarantee is broken.
            if region == FEATURE_REGION:
                feature_result = self._generate_feature(
                    frame,
                    spec,
                    outer_wall_line_width_mm,
                    layer,
                    safety_offset_mm=safety_offset_mm,
                    max_rotation_deg=max_rotation_deg,
                )
                if feature_result.intents:
                    # Structure WAS produced -- this must never also be
                    # recorded as a "pattern_all_webs_omitted" degradation
                    # (the capability's own zero-intent check only ever sees
                    # ``intents``, so returning here with a non-empty list is
                    # what keeps the two from colliding).
                    #
                    # ``omitted``/``degradations`` from the aerofoil path
                    # just run above are PRESERVED, merged with the
                    # feature attempt's own -- discarding them would lose
                    # real diagnostics the aerofoil path already recorded
                    # (e.g. a spar-socket degradation, or the very
                    # omissions that explain why THAT vocabulary found
                    # nothing) even though a feature lattice separately
                    # succeeded.
                    return GenerationResult(
                        intents=feature_result.intents,
                        omitted=omitted + list(feature_result.omitted),
                        spar_fallback_used=spar_fallback_used,
                        degradations=degradations + list(feature_result.degradations),
                    )
                # FEATURE, but the lattice ALSO produced nothing (every
                # tooth was individually infeasible) — fall through to the
                # ordinary zero-intent degradation below, merging the
                # lattice attempt's own omissions in so the degradation
                # record names BOTH why the wing vocabulary failed and why
                # the lattice did too.
                omitted = omitted + list(feature_result.omitted)
            # THE RE-RAISE IS REGION-CONDITIONAL, and the distinction is
            # the whole point.
            #
            # AEROFOIL: re-raise the SAME exception object, unedited. A
            # real wing whose pitch config collapses is a genuine
            # configuration problem and must surface as such -- never masked
            # by a fallback.
            #
            # FEATURE that declined: DEGRADE instead. The region IS a feature;
            # it simply cannot get a lattice at this size. Failing an entire
            # wing over a 4 mm nub, when leaving that nub as plain skin and
            # REPORTING it is available, contradicts the degrade-by-default
            # policy for features.
            #
            # This is not silence: returning no intents routes the layer into
            # zero-intent degradation, which records it with a reason and
            # surfaces it in the run summary. The measured numbers from BOTH
            # failed attempts are already merged into ``omitted`` above, so the
            # record names why the wing vocabulary failed AND why the lattice
            # could not stand in. (Re-raising unconditionally would make every
            # small feature fatal.)
            if aerofoil_error is not None and region != FEATURE_REGION:
                raise aerofoil_error
            if aerofoil_error is not None:
                # Degrading a FEATURE after the wing path RAISED: that
                # exception is about to be discarded, and its message is
                # the only record of why the wing vocabulary failed here.
                # Carry it into the omissions so the degradation record
                # names BOTH failures. (The "already merged" statement above
                # holds only when the wing path OMITTED; when it RAISED, its
                # reason lives only in the exception.)
                omitted = omitted + [
                    OmittedWeb(
                        x_mm=(frame.min_x + frame.max_x) / 2.0,
                        reason=(
                            "skin margin: the wing vocabulary could not be "
                            "applied here -- " + str(aerofoil_error)
                        ),
                        family_index=-4,
                        family_name="aerofoil_path",
                    )
                ]

        return GenerationResult(
            intents=intents, omitted=omitted,
            spar_fallback_used=spar_fallback_used,
            degradations=degradations,
        )

    def _generate_feature(
        self,
        frame: LayerFrame,
        spec: StructureSpec,
        outer_wall_line_width_mm: float,
        layer: LayerContext,
        *,
        safety_offset_mm: float,
        max_rotation_deg: float,
    ) -> GenerationResult:
        """Feature-region lattice with a "never worse" rule.

        When ``spec.feature_dense_lattice_enabled`` is True, this computes
        BOTH candidate results -- ``_generate_feature_core(dense=True)``
        (full local-depth teeth, no global width/depth cap) and
        ``_generate_feature_core(dense=False)`` (the global-bound result) --
        and keeps whichever is BETTER: more teeth built wins outright; a tie
        in tooth count is broken by the greater achieved web depth
        (``max(i.depth_mm() for i in intents)`` -- see
        ``_feature_result_is_better`` for why max, not sum). If the dense
        attempt is not better, this falls back to the non-dense result. No
        ``Degradation`` is recorded and the zero-intent channel does not
        fire, because relative to the flag being off nothing was lost: the
        layer gets exactly the ``intents`` it would have gotten with the flag
        off. The fallback is still observable: the returned ``omitted`` list
        carries the LOSING dense attempt's own omissions forward, retagged
        ``family_name="feature_dense_fallback/..."``, plus one summary entry
        naming both attempts' tooth counts and depths. This follows the same
        "carry a discarded attempt's diagnostics into the surviving result"
        approach as ``generate()`` uses when the aerofoil path raised (see its
        "Carry it into the omissions" comment).

        Why the wrapper exists: on a real z=230 mm post the dense attempt alone
        produces 0 teeth at every pitch tried (the thin-section check
        correctly rejects full local depth on that chamfered/round section --
        see ``_generate_feature_core``'s "MEASURED COUNTEREXAMPLE" comment in
        its dense branch), while the non-dense bound still produces 3 teeth at
        1.7045 mm there. Without this wrapper, enabling the flag would make
        that post WORSE than leaving it off. On a flatter feature (an
        8.64x8.67 mm square, no chamfer) the comparison goes the other way and
        dense wins: 5 teeth at 7.8090 mm (dense) beats 5 teeth at 1.7053 mm
        (non-dense) on the tooth-count tie. Both outcomes come from the same
        comparison rule.

        Computing both attempts costs one extra ``_generate_feature_core``
        call per feature-classified layer, and only when the flag is enabled.
        This is accepted rather than predicting the winner, since the cost is
        bounded (one more geometry pass on a FEATURE region only) and
        prediction would duplicate the reasoning ``_generate_feature_core``
        already does.
        """
        if not spec.feature_dense_lattice_enabled:
            return self._generate_feature_core(
                frame, spec, outer_wall_line_width_mm, layer,
                safety_offset_mm=safety_offset_mm,
                max_rotation_deg=max_rotation_deg, dense=False,
            )

        dense_result = self._generate_feature_core(
            frame, spec, outer_wall_line_width_mm, layer,
            safety_offset_mm=safety_offset_mm,
            max_rotation_deg=max_rotation_deg, dense=True,
        )
        shipped_result = self._generate_feature_core(
            frame, spec, outer_wall_line_width_mm, layer,
            safety_offset_mm=safety_offset_mm,
            max_rotation_deg=max_rotation_deg, dense=False,
        )
        if _feature_result_is_better(dense_result, shipped_result):
            return dense_result

        # NEVER-WORSE FALLBACK, TRACED: the fallback is reported but is not a
        # degradation (relative to the flag being off nothing was lost).
        # Discarding the losing dense attempt's ``omitted`` list wholesale
        # would make the fallback invisible. As in ``generate()``'s
        # aerofoil-path-raised branch ("Degrading a FEATURE after the wing
        # path RAISED... Carry it into the omissions"), a losing attempt's
        # diagnostics are carried into the WINNING result's ``omitted`` list
        # under a distinguishable ``family_name``, never folded into
        # ``degradations`` (that channel is reserved for an actual loss of
        # structure).
        dense_teeth = len(dense_result.intents)
        dense_depth_mm = max(
            (i.depth_mm() for i in dense_result.intents), default=0.0
        )
        shipped_teeth = len(shipped_result.intents)
        shipped_depth_mm = max(
            (i.depth_mm() for i in shipped_result.intents), default=0.0
        )
        traced_omitted = list(shipped_result.omitted) + [
            OmittedWeb(
                x_mm=(frame.min_x + frame.max_x) / 2.0,
                reason=(
                    "feature_dense_lattice_enabled fallback: the dense "
                    "(full local-depth) attempt built {} teeth (max depth "
                    "{:.4f} mm); the shipped global-bound attempt built {} "
                    "teeth (max depth {:.4f} mm) and was KEPT -- "
                    "_feature_result_is_better did not prefer the dense "
                    "attempt. Not a degradation: this layer's own "
                    "structure is exactly what it would be with "
                    "feature_dense_lattice_enabled=False.".format(
                        dense_teeth, dense_depth_mm,
                        shipped_teeth, shipped_depth_mm,
                    )
                ),
                family_index=-5,
                family_name="feature_dense_fallback",
            )
        ] + [
            OmittedWeb(
                x_mm=o.x_mm,
                reason="[discarded dense attempt] " + o.reason,
                family_index=-6,
                family_name="feature_dense_fallback/" + str(o.family_name),
            )
            for o in dense_result.omitted
        ]
        return GenerationResult(
            intents=shipped_result.intents,
            omitted=traced_omitted,
            spar_fallback_used=shipped_result.spar_fallback_used,
            degradations=shipped_result.degradations,
        )

    def _generate_feature_core(
        self,
        frame: LayerFrame,
        spec: StructureSpec,
        outer_wall_line_width_mm: float,
        layer: LayerContext,
        *,
        safety_offset_mm: float,
        max_rotation_deg: float,
        dense: bool,
    ) -> GenerationResult:
        """Tight alternating-entry lattice for a non-aerofoil FEATURE region (a
        joiner post, boss, tab...), dispatched from ``generate()`` ONLY as a
        FALLBACK: the ordinary aerofoil path runs first and this is consulted
        only when that path's own ``intents`` list comes back empty. See
        ``generate()``'s fallback comment for why classifying every frame
        unconditionally is wrong: it would ask "is this a wing section?" of
        regions the aerofoil vocabulary already served, including every
        rectangular "coupon" fixture the unit tests use as a generic
        cross-section stand-in.

        Why the ordinary families are not reused as-is: an ordinary family's
        anchor is ``frame.web_x(web_position) + family.station_drift_mm(print_z)``
        — ``frame.web_x`` is local to this frame's own bbox, but the Z-drift
        term is an ABSOLUTE offset accumulated from the root, unrelated to THIS
        layer's own (possibly tiny) extent. On an 8.6 mm joiner post the
        drifted anchor lands entirely outside the post's own
        ``[min_x, max_x]``, so every candidate is omitted — not because nothing
        fits, but because nothing was ever asked to fit HERE.

        Construction: two synthetic ``WebFamily`` instances, anchored at THIS
        FRAME's own centre (never a chord fraction or Z-drift carried from
        the wing's global placement), one entering +Y and one -Y, each
        marching at ``2 * feature_lattice_pitch_mm`` and offset from each
        other by exactly ``feature_lattice_pitch_mm`` so the combined,
        interleaved sequence alternates sides every
        ``feature_lattice_pitch_mm`` — an ordinary comb/finger-joint
        lattice, sized to the region's own extent by construction
        (``frame.web_x``/marching already stop at
        ``frame.min_x``/``frame.max_x``).

        REUSE, NOT A PARALLEL PRIMITIVE: this calls the SAME
        ``_generate_family`` every ordinary family uses — the same
        per-station feasibility predicates (chord extent, skin margin,
        thin section) gate a feature member exactly as they gate an
        aerofoil one — and the SAME cross-family passes
        (``_merge_cross_family_nodes``, ``_omit_opposing_conflicts``) an
        ordinary layer runs.

        WHY ALTERNATING FULL-DEPTH TEETH DO NOT SEVER THE MATERIAL:
        consecutive stations sit ``feature_lattice_pitch_mm`` apart in X.
        ``_omit_opposing_conflicts``/``TopologyValidator`` both gate a
        +Y/-Y "meeting" only when the two candidates' HALOED footprints
        overlap in X (see ``_omit_opposing_conflicts``'s own docstring) —
        a pitch that clears the slot width plus the minimum wall means
        adjacent teeth never overlap in X, so alternating full-depth cuts
        interlace into one continuous zigzag path instead of a straight
        through-cut. This is REASONING, not a substitute for verification:
        ``TopologyValidator`` still re-measures the real, post-boolean
        geometry independently, and a layer that genuinely severs still
        fails the slice closed, exactly as everywhere else the
        non-negotiable invariant applies.

        No spar socket, no scar, no ``require_primary_structure`` guard: a
        feature region has no spar-socket or declared-interior-hole concept,
        and every member here is deliberately ``ROLE_SKIN_LATTICE`` ("semi
        structural"), never a primary-structure claim.
        """
        check_rotation_envelope(frame.rotation_deg, max_rotation_deg)
        effective_skin_margin_mm = corrected_skin_margin_mm(
            spec.skin_margin_mm, frame.rotation_deg
        )

        pitch_mm = spec.feature_lattice_pitch_mm
        if pitch_mm >= frame.width / 2.0:
            # DECLINE, never raise. This fallback's own guarantee (see
            # ``generate()``'s fallback comment) is that it can only ADD
            # structure where the aerofoil path produced none -- it must
            # never be the thing that turns a layer newly FATAL.
            #
            # A raise here would propagate out of the unguarded
            # ``_generate_feature`` call, ``capability.py``'s ``except
            # (ValueError, ArithmeticError)`` would convert it to a
            # ``TopologyError``, and the slice would fail, where declining
            # lets the layer reach zero-intent degradation and print
            # skin-only with a record.
            #
            # The population this affects is exactly the one the fallback
            # exists to serve: a feature narrower than twice the pitch (4 mm
            # at the default 2 mm pitch) is a small nub, a tapering joiner, or
            # a chamfered post's own top layers.
            #
            # Declining returns no intents, so the caller falls through to
            # degradation exactly as an unclassified region would. The measured
            # numbers travel with it.
            return GenerationResult(
                intents=[],
                omitted=[
                    OmittedWeb(
                        x_mm=(frame.min_x + frame.max_x) / 2.0,
                        reason=(
                            "skin margin: feature too small for a lattice "
                            "-- feature_lattice_pitch_mm={:.4f} mm is at or "
                            "above half this feature's own {:.4f} mm chord "
                            "extent, so the offset tooth would fall outside "
                            "the feature. Declined to plain skin rather "
                            "than failing the slice.".format(
                                pitch_mm, frame.width)
                        ),
                        family_index=-3,
                        family_name="feature_lattice",
                    )
                ],
            )

        center_x = (frame.min_x + frame.max_x) / 2.0
        top_pos = 0.5
        bottom_pos = (center_x + pitch_mm - frame.min_x) / frame.width

        # ``resolved_slot_width_mm``/``halo_half_span_mm`` are needed by
        # BOTH branches below (the ``dense=False`` branch's
        # ``width_bound_mm`` derivation, and as general slot-geometry
        # context) -- see the ``if not dense:`` branch's "NOT full
        # depth" comment for why THAT branch does not use full depth; the
        # ``dense=True`` branch's comment (further down) explains why IT does.
        resolved_slot_width_mm = spec.resolve_slot_gap(outer_wall_line_width_mm)
        halo_half_span_mm = resolved_slot_width_mm / 2.0 + safety_offset_mm

        # ``dense`` selects the dense-lattice mode (see
        # ``feature_dense_lattice_enabled`` in ``aircraft_core/spec.py``, and
        # ``_generate_feature`` for the "never worse" wrapper that decides
        # whether this method is called with ``dense=True``).
        # ``family_margin_mm`` is the value actually enforced as each tooth's
        # far-side stop; the ``dense=False`` branch below leaves it the
        # aerofoil ``effective_skin_margin_mm``. It is declared here, before
        # the branch, so the shared ``_generate_family`` call below (identical
        # in both branches) has one variable to read.
        family_margin_mm = effective_skin_margin_mm

        if not dense:
            # NOT full depth (``web_depth_mm=None``). Full depth reaches to
            # within ``skin_margin_mm`` of the OPPOSITE skin -- fine on a real
            # wing, where the local thickness is tens of mm and that margin is
            # a small fraction of it, but on a small feature (an 8.6 mm joiner
            # post) it consumes nearly the ENTIRE local thickness.
            #
            # Derived, not fit: the thin-section check's extremity-widening
            # (see the comment in ``_generate_family``) widens its probe halo to the
            # section's FULL remaining span whenever a candidate's ORDINARY
            # (un-widened) halo already sits within one requested depth of a
            # chord extremity. For this family's own anchor -- the CENTRE
            # station, distance ``frame.width / 2`` from either edge -- that
            # widening triggers whenever ``depth_used_mm >= frame.width / 2 -
            # halo_half_span_mm``, where ``halo_half_span_mm =
            # resolved_slot_width_mm / 2 + safety_offset_mm`` is exactly the
            # ordinary halo's own half-width (the same quantity
            # ``_generate_family`` computes as ``halo_span_mm / 2``). Staying
            # STRICTLY inside that bound (a 5% margin against the boundary
            # itself, not an empirical fit) means the centre station's own
            # local thickness is what gates it, never a widened reading
            # contaminated by material near the feature's own edges, which can
            # be much thinner than the centre implies. Measured, not assumed:
            # the "square" 8.64x8.67mm test post is actually
            # chamfered/octagonal -- true local thickness tapers from 8.67 mm
            # at its centre to 1.49 mm at its edges (fill ratio 0.78, not a
            # square's 1.0) -- and a depth that lets sample that edge material
            # reads "remaining skin -3.19 mm" and omits every candidate.
            #
            # Also capped at half the region's own local depth
            # (``frame.depth / 2``): never request more than a tooth reaching
            # to the section's own midline, matching the alternating/
            # interlacing design this method's own docstring describes.
            # Whichever bound is tighter wins -- a feature far wider than it
            # is deep is capped by depth, one far deeper than it is wide is
            # capped by width. A result of zero or less (the region is too
            # small for either bound to leave any depth at all) fails closed
            # below, naming both bounds, rather than silently building a
            # zero-depth (degenerate) web.
            #
            # Anchored to the OUTERMOST INTENDED STATION, not the centre.
            # The teeth sit at the centre and at +/- one pitch, so the station
            # whose margin reading is most at risk of extremity contamination
            # is the offset one -- it is ``pitch`` closer to the edge than the
            # centre is. Bounding on the centre would let the centre tooth
            # take a depth the offset teeth could not survive, so they would
            # be omitted for thinness and the "lattice" would collapse to a
            # single slot. Bounding on the outermost station gives a
            # shallower depth that EVERY tooth can take, which is what makes
            # it an alternating comb rather than one cut.
            width_bound_mm = 0.95 * (
                frame.width / 2.0 - pitch_mm - halo_half_span_mm
            )
            depth_bound_mm = frame.depth / 2.0
            feature_web_depth_mm = min(depth_bound_mm, width_bound_mm)
            if feature_web_depth_mm <= 0.0:
                # DECLINE, never raise -- same reasoning as the
                # ``pitch_mm >= frame.width / 2.0`` decline above (reached at
                # feature_lattice_pitch_mm=2.0 mm on a ~4.5 mm-wide square).
                # This method can only ADD structure where the aerofoil path
                # produced none, never turn a layer newly FATAL. A raise here
                # would escape this (otherwise unguarded) method,
                # ``capability.py`` would convert it to a ``TopologyError``,
                # and the slice would fail where a layer just outside the
                # depth-bound floor should fall through to zero-intent
                # degradation and print skin-only with a record. The measured
                # bounds travel with the decline.
                return GenerationResult(
                    intents=[],
                    omitted=[
                        OmittedWeb(
                            x_mm=(frame.min_x + frame.max_x) / 2.0,
                            reason=(
                                "depth bound: this feature's own extent "
                                "leaves no room for a lattice tooth at layer "
                                "{} (print_z={:.4f}): width-derived bound "
                                "{:.4f} mm (frame.width/2 - halo_half_span_mm, "
                                "halo half-span {:.4f} mm), depth-derived "
                                "bound {:.4f} mm (frame.depth/2) -- the "
                                "tighter of the two is <= 0. This feature is "
                                "too small for feature_lattice_pitch_mm="
                                "{:.4f} mm at the resolved slot width {:.4f} "
                                "mm. Declined to plain skin rather than "
                                "failing the slice.".format(
                                    layer.layer_index, layer.print_z,
                                    width_bound_mm, halo_half_span_mm,
                                    depth_bound_mm, pitch_mm,
                                    resolved_slot_width_mm,
                                )
                            ),
                            family_index=-3,
                            family_name="feature_lattice",
                        )
                    ],
                )
        else:
            # DENSE MODE. A feature is a load path (e.g. holding a winglet on),
            # so it should carry as much lattice as is useful. The non-dense
            # bound above cannot reach that: it is capped by ``frame.width``
            # (the wrong quantity for a round/square feature) and by
            # ``frame.depth / 2`` (which forbids a tooth from passing the
            # section's own midline, even though opposing teeth are laterally
            # offset -- see below -- and so cannot collide there).
            #
            # ``feature_web_depth_mm = None`` hands each candidate station to
            # ``_generate_family``'s FULL-DEPTH branch (``family.web_depth_mm
            # is None``, the same branch the aerofoil path uses to "reach
            # toward the local far surface, station by station"). That branch
            # calls ``local_upper_worst_case``/``local_lower_worst_case`` at
            # EACH candidate's own X, so a chamfered edge station gets less
            # depth than the thicker centre without any global width term. No
            # per-station cap is added here: the far-side stop is the margin
            # below, applied at each station's own local far surface.
            feature_web_depth_mm = None

            # Wall-protection margin. A feature-specific margin smaller than
            # the aerofoil ``skin_margin_mm`` (e.g. ``outer_wall_line_width_mm
            # + safety_offset_mm``, ~0.84 mm against ``skin_margin_mm``'s
            # 0.8608 mm) is tempting, since ``skin_margin_mm`` protects an
            # aerofoil's far exterior skin and a feature's far side is its own
            # outer wall. It does not work: ``TopologyValidator.validate``'s
            # invariant 4 (and its -Y mirror) independently re-derives
            # ``effective_skin_margin_mm = corrected_skin_margin_mm(
            # spec.skin_margin_mm, rotation_deg)`` and requires
            # ``remaining_skin/remaining_wall >= effective_skin_margin_mm`` on
            # EVERY non-scar slot, whatever margin the generator used to
            # choose the depth. A tooth built to a smaller margin leaves less
            # than ``skin_margin_mm`` of material at its far end and fails
            # that invariant after the boolean. Reproduced: on a round boss
            # r=4.3 mm with skin_margin_mm=1.0 mm, a ~0.84 mm trial margin
            # built 5 teeth up to 7.83 mm deep on the 8.64x8.67 mm square
            # fixture and the validator rejected every one ("leaves 0.8402 mm
            # of skin... below the required 1.0000 mm margin"). A topology
            # invariant is never weakened to make a fixture pass, so this
            # branch uses the same ``effective_skin_margin_mm`` as the
            # non-dense branch (``family_margin_mm`` was seeded from it above
            # and is left untouched here).
            #
            # What does change from the non-dense branch is the depth bound.
            # ``width_bound_mm`` and ``depth_bound_mm`` above are GLOBAL,
            # single-value bounds applied to every station, conservative
            # enough to survive the furthest-out (thinnest, chamfer-adjacent)
            # station. With ``web_depth_mm=None`` each station stops
            # ``effective_skin_margin_mm`` short of ITS OWN local far surface
            # and is not limited to the section's midline, so a station near
            # the thick centre reaches nearly its full local depth minus
            # margin.
            #
            # MEASURED COUNTEREXAMPLE: this helps a feature whose far surface
            # is roughly FLAT across the widened probe window (a rectangular
            # post: 5 teeth, 1.7 mm -> 7.7 mm deep, still one valid closed
            # loop). It does NOT help -- it regresses from 3 teeth to 0 at
            # every pitch tried -- on a real post section at z=230 mm whose far
            # surface is chamfered/rounded (fill ratio ~0.78). The
            # extremity-widening in ``_generate_family`` (see its comment)
            # correctly predicts that full local depth there leaves negative
            # remaining skin near the chamfer, and
            # ``tools/m36_d017_bypass_probe.py`` (which skips the generator's
            # checks and asks ``TopologyValidator`` directly) confirms the
            # prediction was right, not merely conservative: the validator's
            # independent re-measurement rejects the same deep teeth by a
            # comparable margin (0.8016 mm remaining vs. 0.8608 mm required at
            # one of the post's bottom-entry stations). The full pitch sweep of
            # dense vs. non-dense, with the never-worse wrapper, is
            # ``tools/m36_dense_lattice_sweep.py``. So on a chamfered/round
            # feature this mode cannot deliver a dense lattice without either
            # a smaller ``skin_margin_mm`` (which the validator will not
            # accept) or a per-station depth bound, which would need one
            # ``WebFamily`` per candidate station rather than two marching
            # families.

        top_family = WebFamily(
            name="feature_lattice_top",
            web_position=top_pos,
            rib_pitch_mm=2.0 * pitch_mm,
            web_depth_mm=feature_web_depth_mm,
            entry_lower=False,
            role=ROLE_SKIN_LATTICE,
        )
        bottom_family = WebFamily(
            name="feature_lattice_bottom",
            web_position=bottom_pos,
            rib_pitch_mm=2.0 * pitch_mm,
            web_depth_mm=feature_web_depth_mm,
            entry_lower=True,
            role=ROLE_SKIN_LATTICE,
        )
        top_family.validate(0)
        bottom_family.validate(1)

        # Opposing teeth cannot collide, in either mode, by construction:
        # ``top_family`` is anchored at this frame's own centre
        # (``top_pos = 0.5``) and marches at ``2 * pitch_mm``;
        # ``bottom_family`` is anchored exactly ``pitch_mm`` further
        # along X (``bottom_pos``, above) and marches at the same
        # ``2 * pitch_mm``. So the two families' candidate stations
        # interleave as ..., center-pitch, center, center+pitch,
        # center+2*pitch, ... with every +Y station at least ``pitch_mm``
        # (in fact exactly an odd multiple of it) from every -Y station --
        # never the same X. ``_generate_family``'s own footprint width is
        # ``resolved_slot_width_mm`` (well under ``pitch_mm`` by the
        # ``pitch_mm >= frame.width / 2.0`` guard above and the spec's own
        # documented pitch-floor derivation), so opposing haloed
        # footprints do not overlap in X either. This does
        # NOT rely on reasoning alone: ``_omit_opposing_conflicts``
        # below independently re-measures every +Y/-Y pair's actual footprint and
        # ``reach_depth_mm`` and omits (never silently allows) any pair
        # that WOULD meet, and ``TopologyValidator.validate`` re-measures
        # the real post-boolean geometry a second time after that -- so a
        # collision this reasoning missed still fails closed rather than
        # slicing.
        intents: List[WebIntent] = []
        omitted: List[OmittedWeb] = []
        for family_index, family in enumerate((top_family, bottom_family)):
            fam_intents, fam_omitted = self._generate_family(
                frame,
                spec,
                family,
                family_index,
                outer_wall_line_width_mm,
                layer,
                safety_offset_mm,
                family_margin_mm,
                None,
            )
            intents.extend(fam_intents)
            omitted.extend(fam_omitted)

        intents, node_merge_omitted = self._merge_cross_family_nodes(
            intents,
            safety_offset_mm,
            frame=frame,
            spec=spec,
            effective_skin_margin_mm=effective_skin_margin_mm,
            ctx=layer,
        )
        omitted.extend(node_merge_omitted)

        intents, opposing_omitted = self._omit_opposing_conflicts(
            intents, frame, safety_offset_mm, spec.overshoot_mm
        )
        omitted.extend(opposing_omitted)

        return GenerationResult(
            intents=intents, omitted=omitted,
            spar_fallback_used=False, degradations=[],
        )

    @staticmethod
    def _socket_local_thickness_mm(frame, x, r, spec):
        """Local section thickness across a socket's footprint, read for the
        socket-footprint exclusion. With ``spec.per_station_reentrant`` a
        re-entrant read returns 0.0 (the conservative direction: every
        opposite-side member in the socket's X band then counts as a
        conflict and is omitted) instead of
        propagating and shutting the whole layer down."""
        try:
            return (
                local_upper_worst_case(
                    frame.ops, frame.polygon, x, 2.0 * r,
                    DEFAULT_PROBE_WIDTH_MM)
                - local_lower_worst_case(
                    frame.ops, frame.polygon, x, 2.0 * r,
                    DEFAULT_PROBE_WIDTH_MM))
        except ReEntrantSectionError:
            if not _per_station_reentrant(spec):
                raise
            return 0.0

    def _generate_spar_socket(self, frame, spec, line_width_mm, layer):
        """One carbon-spar socket for this layer, as a keyhole WebIntent.

        The spar is EXTERNAL: a carbon tube installed after printing, carrying
        the global bending. This cuts the passage it slides into.

        Enters from whichever skin is nearer at the spar station -- the same
        "vertical entry, nearest surface" discipline as the scar, so the
        ordinary collision, erosion and opposing-entry checks apply.

        FAILS CLOSED. A socket that does not fit is a spar that cannot be
        installed, which is not a degradation to absorb quietly -- it raises
        with the layer, the Z, the section depth and what was needed.
        """
        r = spec.spar_diameter_mm / 2.0 + spec.spar_clearance_mm
        x = frame.min_x + spec.spar_station * frame.width

        y_hi = local_upper_worst_case(
            frame.ops, frame.polygon, x, 2.0 * r, DEFAULT_PROBE_WIDTH_MM)
        y_lo = local_lower_worst_case(
            frame.ops, frame.polygon, x, 2.0 * r, DEFAULT_PROBE_WIDTH_MM)
        depth = y_hi - y_lo
        needed = 2.0 * r + 2.0 * spec.skin_margin_mm

        if depth < needed:
            # The socket ENDS here rather than failing the slice -- same rule
            # as the detected-channel path below, applied to the
            # config-declared debugging fallback so the two cannot drift apart.
            return None, Degradation(
                "spar_socket", spec.spar_station * frame.max_x,
                "DEGRADED -- spar socket dropped: the carbon-spar socket "
                "does not fit at station {:.3f}c: the "
                "section is {:.3f} mm deep there but a {:.1f} mm spar with "
                "{:.2f} mm clearance needs {:.3f} mm ({:.3f} mm of socket "
                "plus {:.4f} mm of skin margin on each side). No socket is "
                "cut on this layer; the capability holds the run-level "
                "latch that keeps it ended. The tip is the binding station "
                "on a tapered wing, never the root.".format(
                    spec.spar_station, depth, spec.spar_diameter_mm,
                    spec.spar_clearance_mm, needed, 2.0 * r,
                    spec.skin_margin_mm))

        # centre the socket in the local section, then enter from the nearer
        # skin so the neck is as short as possible
        cy = 0.5 * (y_hi + y_lo)
        enter_lower = (cy - y_lo) < (y_hi - cy)

        # The neck must BREAK THROUGH the skin. ``local_*_worst_case`` returns
        # the most conservative surface position over the probe window, which
        # lies INSIDE the true surface -- starting there leaves the neck short,
        # the socket never reaches the boundary, and the validator correctly
        # reports "material has 1 interior hole(s), expected 0". Start from the
        # section's own bbox extreme instead, which is on or outside the
        # surface at every station by construction; SlotBuilder's overshoot
        # then guarantees a clean cut, exactly as it does for every other slot.
        start_y = frame.min_y if enter_lower else frame.max_y

        return WebIntent(
            start=(x, start_y),
            end=(x, cy),
            width_mm=spec.spar_neck_width_mm,
            family_index=-2,
            family_name="spar_socket",
            entry_lower=enter_lower,
            socket_radius_mm=r,
            web_id=(-2, 0, WEB_SIDE_SOCKET),
            # The config-fallback socket is never a pick among
            # several candidates -- it is the one, config-declared spar station
            # -- so it is CONFIRMED unconditionally, same as the unambiguous
            # single-detected-channel case in
            # ``_generate_spar_socket_from_channel``.
            spar_confirmed=True,
        ), None

    def _build_channel_socket(
        self, frame, spec, channel, *,
        family_name, family_index, degradation_kind, unfit_message_template,
        spar_confirmed=False, layer=None,
    ):
        """Shared geometry for ``_generate_spar_socket_from_channel`` and
        ``_generate_bore_socket_from_channel``: radius from the DETECTED
        diameter plus ``spec.spar_clearance_mm``, the fit check against the
        section's own local depth at this ``channel``'s centroid, the
        "nearer skin" entry-side choice, and the keyhole ``WebIntent``
        shape. Every one of those is IDENTICAL for a spar channel and a
        plain bore -- see each caller's own docstring for the derivation.

        What is NOT shared, and is threaded through as keyword-only parameters
        instead of being read off ``self``/hard-coded here, is exactly what
        makes a bore's socket safely distinct from the spar's THROUGH
        ``capability.py`` (not just a label difference in this module):
        ``family_name``/``family_index`` (the spar-socket latch and its
        degradation bookkeeping key on the literal string ``"spar_socket"`` --
        see ``_generate_bore_socket_from_channel``'s own docstring for why a
        bore must never share it), ``degradation_kind`` (same reason), and
        ``unfit_message_template`` (the human-readable reason differs: the
        spar's names the run-level latch explicitly, a bore's says it carries
        none).

        ``unfit_message_template`` is a ``str.format`` template consuming
        the named fields ``x``, ``depth``, ``diameter_mm``, ``clearance``,
        ``needed``, ``twice_r``, ``skin_margin`` -- see each caller's own
        template for the exact wording.

        ``spar_confirmed`` (keyword-only, defaulted to ``False`` so a
        bore's call site never has to think about it) is carried straight onto
        the built ``WebIntent`` -- see ``WebIntent.spar_confirmed``'s own
        docstring. The caller decides it, never this shared helper:
        ``_generate_spar_socket_from_channel`` is called from BOTH the
        unambiguous single-channel branch (confirmed) and the
        ``nearest_spar_station`` multi-candidate disambiguation branch (not
        confirmed) in ``generate()``, so only the caller knows which one this
        invocation is.

        ``layer`` (keyword-only, defaulted to ``None``) is needed ONLY for the
        ``spar_confirmed`` refusal below: it supplies the
        ``layer_index``/``print_z`` a ``TopologyError`` requires.
        ``_generate_spar_socket_from_channel`` forwards its own ``layer``
        argument here; ``_generate_bore_socket_from_channel`` never does,
        because a bore can never take the ``spar_confirmed`` branch that needs
        it (``degradation_kind`` is never ``"spar_socket"`` for a bore -- see
        its own docstring).
        """
        r = channel.diameter_mm / 2.0 + spec.spar_clearance_mm
        x = channel.center_x_mm
        cy = channel.center_y_mm

        # A detected channel is ordinary geometry recovered from the mesh (e.g.
        # an elevon servo pocket, ~1.1 mm diameter) -- it is
        # under no obligation to be big enough to carry a keyhole socket.
        # ``SlotBuilder._keyhole`` (slot.py) FAILS CLOSED with ``SpecError``
        # when the neck half-width does not intersect the socket circle
        # (``half_w >= r``), which is the right backstop for a genuine spec
        # mistake -- but reaching that raise from here would blow up the whole
        # slice over a bore nobody asked to be load-bearing.
        #
        # This is NOT the same situation as the config-declared path
        # (``_generate_spar_socket`` above): there, ``half_w >= r`` is already
        # guarded upstream, at spec-validation time
        # (``StructureSpec.validate`` -- algebraically the
        # identical predicate one layer earlier, on
        # ``spar_neck_width_mm``/``spar_diameter_mm``/``spar_clearance_mm``),
        # so it is unreachable there by construction; that is why the config
        # path carries no matching test here. THIS guard exists because a
        # DETECTED channel's radius comes from measured mesh geometry, which
        # ``StructureSpec.validate`` never sees -- it cannot validate a
        # diameter it has no way to read. Test the identical predicate
        # ``_keyhole`` will apply, BEFORE calling it, so a too-small channel
        # degrades (dropped, reported, layer keeps slicing) instead of raising.
        # No new threshold: this reuses the exact half_w/r relationship
        # ``_keyhole`` encodes, from the same ``spec.spar_neck_width_mm`` and
        # the same locally-computed ``r`` used a few lines below to build the
        # ``WebIntent``.
        #
        # Degrading here is right for a bore or an UNCONFIRMED spar
        # candidate -- neither is required to be load-bearing (servo pockets
        # are simply left hollow). It is NOT acceptable for the CONFIRMED
        # spar itself: that channel is the one structural member every family's
        # fit decisions (``depth_to_socket``, ``socket_flank_*``) treat as load
        # path, and ``require_primary_structure`` -- the mechanism that would
        # otherwise catch "the wing has no spar" -- does not fire
        # for this drop (a dropped socket reaches the family loop the same way
        # as any other unfit channel). So a too-small CONFIRMED spar channel
        # refuses here instead: a catchable ``TopologyError``,
        # matching every other fail-closed structural condition in this module
        # (e.g. the ambiguous-channel-count refusal above), never the raw
        # ``SpecError`` ``_keyhole`` would otherwise raise uncaught.
        half_w = spec.spar_neck_width_mm / 2.0
        if half_w >= r:
            if degradation_kind == "spar_socket" and spar_confirmed:
                raise TopologyError(
                    layer.layer_index, layer.print_z,
                    "confirmed spar channel too small to carry a keyhole "
                    "socket: the detected channel at x={x:.4f} mm, "
                    "y={y:.4f} mm (centroid from the model), diameter "
                    "{diameter_mm:.4f} mm, gives socket radius {r:.4f} mm "
                    "({clearance:.2f} mm clearance) against keyhole neck "
                    "half-width {half_w:.4f} mm (spar_neck_width_mm/2) -- "
                    "a neck this wide does not intersect a socket this "
                    "small (SlotBuilder._keyhole's own guard, slot.py). "
                    "This channel is the CONFIRMED spar (spar_confirmed="
                    "True), not an incidental bore: dropping it silently "
                    "would print a wing with no spar socket. Fix the model "
                    "geometry, spar_neck_width_mm, or spar_clearance_mm so "
                    "the spar channel can carry a keyhole.".format(
                        x=x, y=cy, diameter_mm=channel.diameter_mm,
                        clearance=spec.spar_clearance_mm, r=r,
                        half_w=half_w,
                    ),
                )
            effective_kind = degradation_kind
            if degradation_kind == "spar_socket" and not spar_confirmed:
                effective_kind = "spar_socket_unconfirmed"
            return None, Degradation(
                effective_kind, x,
                "DEGRADED -- {family} dropped: the detected channel at "
                "x={x:.4f} mm, y={y:.4f} mm (centroid from the model), "
                "diameter {diameter_mm:.4f} mm, is too small to carry a "
                "keyhole socket: {clearance:.2f} mm clearance gives socket "
                "radius {r:.4f} mm, but the keyhole neck half-width is "
                "{half_w:.4f} mm (spar_neck_width_mm/2) -- a neck this wide "
                "does not intersect a socket this small and would silently "
                "become a plain rectangle instead of a keyhole "
                "(SlotBuilder._keyhole's own guard, slot.py). No socket is "
                "cut on this layer for this channel; it is left "
                "hollow.".format(
                    family=family_name, x=x, y=cy,
                    diameter_mm=channel.diameter_mm,
                    clearance=spec.spar_clearance_mm, r=r, half_w=half_w,
                ),
            )

        try:
            y_hi = local_upper_worst_case(
                frame.ops, frame.polygon, x, 2.0 * r, DEFAULT_PROBE_WIDTH_MM)
            y_lo = local_lower_worst_case(
                frame.ops, frame.polygon, x, 2.0 * r, DEFAULT_PROBE_WIDTH_MM)
        except ReEntrantSectionError as exc:
            # With ``per_station_reentrant`` (default off), a re-entrant read
            # at THIS channel degrades this channel alone. Never for the
            # CONFIRMED spar -- that is the one load path, and dropping it (an
            # END that arms the latch) is not this flag's call; the error
            # propagates and shuts the layer down.
            if (
                not _per_station_reentrant(spec)
                or (degradation_kind == "spar_socket" and spar_confirmed)
            ):
                raise
            return None, Degradation(
                "re_entrant_station", x,
                "DEGRADED -- {} dropped: the section is re-entrant at this "
                "channel's station x={:.4f} mm, so its local depth is not "
                "measurable ({}). No socket is cut on this layer for this "
                "channel (per_station_reentrant).".format(
                    family_name, x, exc),
            )
        depth = y_hi - y_lo
        needed = 2.0 * r + 2.0 * spec.skin_margin_mm

        if depth < needed:
            # The socket ENDS here rather than failing the slice -- see
            # ``_generate_spar_socket_from_channel``'s own docstring for the
            # full "why degrade, not fail" reasoning, which applies identically
            # to a bore.
            #
            # An UNCONFIRMED spar candidate's own fit failure must
            # never arm the run-level latch (``capability.py``'s
            # ``_apply_spar_socket_latch`` keys on the literal string
            # ``"spar_socket"``) -- arming it here would mean a bore that
            # merely won the "nearest to spar_station" contest, and then failed
            # to fit, permanently suppresses the REAL confirmed spar for the
            # rest of the run if one later reappears. Reported under a distinct
            # kind (``"spar_socket_unconfirmed"``) instead, so the degradation
            # is still fully visible in the run report (never silent) but
            # cannot be mistaken for -- or drive -- the real spar's own latch.
            effective_kind = degradation_kind
            if degradation_kind == "spar_socket" and not spar_confirmed:
                effective_kind = "spar_socket_unconfirmed"
            return None, Degradation(
                effective_kind, x,
                unfit_message_template.format(
                    x=x, depth=depth, diameter_mm=channel.diameter_mm,
                    clearance=spec.spar_clearance_mm, needed=needed,
                    twice_r=2.0 * r, skin_margin=spec.skin_margin_mm,
                ),
            )

        enter_lower = (cy - y_lo) < (y_hi - cy)
        start_y = frame.min_y if enter_lower else frame.max_y

        return WebIntent(
            start=(x, start_y),
            end=(x, cy),
            width_mm=spec.spar_neck_width_mm,
            family_index=family_index,
            family_name=family_name,
            entry_lower=enter_lower,
            socket_radius_mm=r,
            spar_confirmed=spar_confirmed,
            web_id=(family_index, 0, WEB_SIDE_SOCKET),
        ), None

    # ``_build_channel_socket``'s ``unfit_message_template`` for the spar
    # case -- named fields, not positional ``{}``, so the shared helper
    # above can supply them by keyword regardless of which caller it is
    # serving.
    _SPAR_SOCKET_UNFIT_MESSAGE = (
        "DEGRADED -- spar socket dropped: the detected carbon-spar "
        "channel does not fit at x={x:.4f} mm "
        "(centroid from the model): the section is {depth:.3f} mm deep "
        "there but a {diameter_mm:.4f} mm detected channel with "
        "{clearance:.2f} mm clearance needs {needed:.3f} mm "
        "({twice_r:.3f} mm of socket plus {skin_margin:.4f} mm of skin "
        "margin on each side). No socket is cut on "
        "this layer. This module is single-layer and stateless, so "
        "it states only what it did HERE -- the capability holds "
        "the run-level latch that keeps the socket ended once it "
        "has ended (the rod must stop short, and a socket "
        "that stopped and restarted would leave a blind pocket the "
        "rod cannot enter)."
    )

    # Same shape as ``_SPAR_SOCKET_UNFIT_MESSAGE``, used only when
    # ``spar_confirmed`` is ``False`` (the ``nearest_spar_station``
    # multi-candidate disambiguation branch). Deliberately drops the
    # run-level-latch sentence: an unconfirmed candidate's fit failure never
    # arms that latch (see ``_build_channel_socket``'s own comment), so a
    # message claiming it does would be describing a mechanism this candidate
    # never triggers.
    _SPAR_SOCKET_UNCONFIRMED_UNFIT_MESSAGE = (
        "DEGRADED -- spar socket dropped: the detected channel nearest "
        "spar_station does not fit at x={x:.4f} mm "
        "(centroid from the model): the section is {depth:.3f} mm deep "
        "there but a {diameter_mm:.4f} mm detected channel with "
        "{clearance:.2f} mm clearance needs {needed:.3f} mm "
        "({twice_r:.3f} mm of socket plus {skin_margin:.4f} mm of skin "
        "margin on each side). No socket is cut on this layer. This "
        "candidate was chosen by disambiguating among two or more "
        "detected channels (spar_channel_policy=\"nearest_spar_station\"), "
        "never confirmed as the real spar -- its own fit failure carries "
        "no run-level latch of its own, the same way a plain bore's does "
        "not."
    )

    # Same shape, bore wording: no latch reference (a bore is never subject to
    # it -- see _generate_bore_socket_from_channel's docstring).
    _BORE_SOCKET_UNFIT_MESSAGE = (
        "DEGRADED -- bore socket dropped: the detected "
        "alignment-bore channel does not fit at x={x:.4f} mm "
        "(centroid from the model): the section is {depth:.3f} mm deep "
        "there but a {diameter_mm:.4f} mm detected channel with "
        "{clearance:.2f} mm clearance needs {needed:.3f} mm "
        "({twice_r:.3f} mm of socket plus {skin_margin:.4f} mm of skin "
        "margin on each side). No socket is cut "
        "on this layer for this bore -- this bore is independent "
        "of the spar socket (if any) and carries no run-level "
        "latch of its own."
    )

    def _generate_spar_socket_from_channel(
        self, frame, spec, layer, channel, *, spar_confirmed,
    ):
        """One carbon-spar socket for this layer, built from a ``DetectedChannel``
        RECOVERED FROM THE MODEL rather than from
        ``spec.spar_diameter_mm``/``spec.spar_station``.

        THE MODEL WINS: ``x`` (chordwise position) and the socket's Y centre
        both come from ``channel``'s own centroid, never from
        ``spec.spar_station`` -- unlike ``_generate_spar_socket``, this
        method never reads ``spec.spar_diameter_mm``/``spec.spar_station``
        at all. Only CLEARANCE (``spec.spar_clearance_mm``) and NECK WIDTH
        (``spec.spar_neck_width_mm``) come from the spec -- both are POLICY
        (how much slip fit the fit needs, how wide the boundary connection
        is), never a geometric measurement the model already supplied.

        The entry surface (which skin the neck breaks through) is still
        chosen by "nearer to the detected centre", the same discipline
        ``_generate_spar_socket`` uses -- only which Y the comparison is
        made against the detected centroid rather than a locally-computed
        midpoint.

        FAILS CLOSED, against the DETECTED diameter (not any config value): a
        socket that does not fit here is a spar that cannot be installed at the
        position the model actually shows, and that is exactly as fatal as the
        config-driven case -- see ``_generate_spar_socket``'s own docstring.

        Geometry lives in ``_build_channel_socket`` (shared with
        ``_generate_bore_socket_from_channel``); this method supplies only
        what makes a SPAR distinct through ``capability.py``:
        ``family_name="spar_socket"``, ``family_index=-2`` (the sentinel
        registry: -1 scar, -2 spar socket, -3 feature-lattice omissions
        (``OmittedWeb``-only, never a built ``WebIntent`` -- see
        ``_generate_feature``), -4 aerofoil-path / -5 / -6 feature-dense
        fallback sentinels (see the same call sites), -7 bore socket),
        and ``degradation_kind="spar_socket"`` -- both literal strings
        ``capability.py``'s spar-socket latch
        (``_apply_spar_socket_latch``) keys on directly.

        ``spar_confirmed`` is REQUIRED (keyword-only, no default)
        precisely because this method is called from two different places in
        ``generate()`` that must not share a default: the unambiguous
        single-detected-channel branch (``True``) and the
        ``spar_channel_policy="nearest_spar_station"`` multi-candidate
        disambiguation branch (``False`` -- the pick is good enough to cut the
        socket, not to anchor structure to it; see
        ``WebIntent.spar_confirmed``'s own docstring). A missing argument
        raises ``TypeError`` rather than silently defaulting to either value --
        fail closed on exactly the ambiguity this field exists to resolve.
        """
        return self._build_channel_socket(
            frame, spec, channel,
            family_name="spar_socket", family_index=-2,
            degradation_kind="spar_socket",
            unfit_message_template=(
                self._SPAR_SOCKET_UNFIT_MESSAGE if spar_confirmed
                else self._SPAR_SOCKET_UNCONFIRMED_UNFIT_MESSAGE
            ),
            spar_confirmed=spar_confirmed, layer=layer,
        )

    def spar_channel_status(self, frame, spec, layer, outer_wall_line_width_mm):
        """What the spar-socket latch would see on this layer, WITHOUT generating
        any other structure.

        One of ``SPAR_STATUS_ABSENT`` (no CONFIRMED spar channel here: none
        detected, or the ``nearest_spar_station`` pick, which is never
        confirmed and never arms the latch), ``SPAR_STATUS_FIT`` (a
        confirmed socket would be built) or ``SPAR_STATUS_NOT_FIT`` (a
        confirmed channel is present but the section cannot hold it -- the
        ``"spar_socket"`` degradation that arms the latch).

        It runs the SAME branch logic and the SAME
        ``_generate_spar_socket_from_channel`` fit check ``generate()``
        runs, so the precompute and the per-layer generator cannot drift.
        Layers ``generate()`` would refuse (several channels under
        ``refuse``; a confirmed channel too small for a keyhole) report
        ABSENT: they fail the real slice on their own and must not steer the
        latch.
        """
        channels = layer.detected_channels
        if len(channels) > 1:
            return SPAR_STATUS_ABSENT
        try:
            if len(channels) == 1:
                socket, degradation = self._generate_spar_socket_from_channel(
                    frame, spec, layer, channels[0], spar_confirmed=True)
            elif spec.spar_diameter_mm and spec.allow_config_spar_fallback:
                socket, degradation = self._generate_spar_socket(
                    frame, spec, outer_wall_line_width_mm, layer)
            else:
                return SPAR_STATUS_ABSENT
        except TopologyError:
            return SPAR_STATUS_ABSENT
        if degradation is not None and degradation.kind == "spar_socket":
            return SPAR_STATUS_NOT_FIT
        return SPAR_STATUS_FIT if socket is not None else SPAR_STATUS_ABSENT

    def _generate_bore_socket_from_channel(self, frame, spec, channel):
        """One plain-bore socket, built from a ``DetectedChannel`` that
        ``spec.spar_channel_policy == "nearest_spar_station"`` decided is NOT
        the spar (``generate()``'s own docstring, clause (b)) -- e.g. one of
        two Ø3mm alignment-rod bores, which coexist with each other
        but never with the real spar channel and are otherwise
        indistinguishable from a spar channel to the mesh-sectioning that feeds
        ``layer.detected_channels``.

        The geometry (radius from the DETECTED diameter plus
        ``spec.spar_clearance_mm``, keyhole entering the nearer skin,
        fit-check against the DETECTED diameter) mirrors
        ``_generate_spar_socket_from_channel`` exactly -- both call the
        same ``_build_channel_socket`` helper. ``spec.spar_clearance_mm``/
        ``spec.spar_neck_width_mm`` are reused deliberately: both are
        generic slip-fit/boundary-connection POLICY, not spar-specific, and
        there is no separate bore policy.

        A bore deliberately uses a DIFFERENT ``family_name``/``family_index``/
        ``degradation_kind`` than the spar's: those fields are load-bearing THROUGH
        ``capability.py``, not just labels here. ``_apply_spar_socket_latch``
        ("once the spar socket has ENDED on this run, keep it ended") and its
        "spar socket dropped"/"spar socket footprint" degradation-kind
        bookkeeping both key on the literal string ``"spar_socket"``; a plain
        bore reusing it would either falsely arm that run-level latch on a
        bore's own fit failure, or (worse) be silently SUPPRESSED by a latch
        the real spar armed earlier in the run, on a station where the bore
        itself fits fine. A genuinely separate ``WebIntent``
        (``family_name="bore_socket"``, ``family_index=-7``) keeps a bore
        wholly outside that machinery. -7 is a clean, otherwise-unused
        sentinel (-1/-2 are the scar and the spar socket; -3 is the
        ``feature_lattice`` omissions' sentinel; -4/-5/-6 are the
        feature-dense-lattice fallback sentinels). -3 would in practice have
        been safe too (every family-index lookup this module/``capability.py``
        performs on a socket-bearing ``WebIntent`` guards on ``idx >= 0`` --
        see ``capability.py``'s ``_family_key``/``real = [idx for idx in
        member_indices if idx >= 0]`` -- and ``feature_lattice`` never
        produces a built ``WebIntent``), but a collision that is only argued
        safe is worse than a sentinel with none. A bore is excluded from
        cross-family merge/opposing-entry the same way every socket is (via
        ``socket_radius_mm`` being set, not via its family name), but never
        touches the spar-specific latch.

        FAILS SOFT, like the spar case: a bore that does not fit at this
        station is dropped and reported as a ``Degradation`` (kind
        ``"bore_socket"``), never silently discarded and never a reason to
        fail the whole slice -- an alignment-rod bore near the tip is not
        worth refusing a whole wing over, any more than the spar socket
        itself is (see ``_generate_spar_socket_from_channel``'s own
        docstring).
        """
        return self._build_channel_socket(
            frame, spec, channel,
            family_name="bore_socket", family_index=-7,
            degradation_kind="bore_socket",
            unfit_message_template=self._BORE_SOCKET_UNFIT_MESSAGE,
        )

    @staticmethod
    def _family_role_of(omission, spec) -> str:
        """The ``role`` of the family an omission came from.

        ``OmittedWeb`` carries ``family_name``/``family_index`` for reporting
        but not the role itself, so it is looked up on the spec. Falls back to
        the skin-lattice default when the name does not resolve, which is the
        safe direction: an unrecognised family never triggers the guard.
        """
        fams = spec.effective_families()
        idx = getattr(omission, "family_index", None)
        if isinstance(idx, int) and 0 <= idx < len(fams):
            return getattr(fams[idx], "role", ROLE_SKIN_LATTICE)
        name = getattr(omission, "family_name", None)
        for f in fams:
            if f.name == name:
                return getattr(f, "role", ROLE_SKIN_LATTICE)
        return ROLE_SKIN_LATTICE

    def _yield_lattice_to_primary(
        self,
        intents: List[WebIntent],
        min_wall_mm: float,
        spec: StructureSpec,
    ) -> Tuple[List[WebIntent], List[OmittedWeb]]:
        """Pre-pass for ``_merge_cross_family_nodes``: a ``skin_lattice``
        candidate within the sub-``min_wall_mm`` merge band of a
        ``primary_structure`` candidate on the SAME entry edge is OMITTED, not
        merged — see ``_merge_cross_family_nodes``'s docstring ("Why omission,
        not 'merge at the deeper depth'") for the reasoning.

        Every linking condition below is DELIBERATELY IDENTICAL to
        ``_merge_cross_family_nodes``'s union-find linking test — same
        ``_NODE_SUPPRESSION_EPSILON_MM``, same gap arithmetic, same
        same-family/entry_lower/is_scar guards — so a cross-role pair can never
        fall through this pre-pass into the ordinary merge by the two tests
        disagreeing about where the band's edges are.

        ITERATIVE, scan-with-restart, unlike the single-pass union-find in the
        merge: omitting one lattice candidate can newly expose the primary to a
        formerly-non-adjacent lattice neighbour on the other side. The
        candidate set here is a handful of stations per layer, so a
        restart-to-fixed-point scan costs nothing material.

        A component with 2+ lattice members all crowded against one primary
        member omits EVERY lattice member that lands in-band against the
        primary or against another already-omitted lattice member's former
        neighbour — i.e. it iterates until no cross-role in-band pair remains.
        It does not touch a lattice-lattice pair's own gap (that stays ordinary
        merge concern), only pairs where one side is ``primary_structure`` and
        the other ``skin_lattice``.

        Returns ``(remaining, omitted)`` — ``remaining`` is what
        ``_merge_cross_family_nodes`` should hand to its own union-find in
        place of its original ``intents`` argument; every remaining
        candidate (including every ``primary_structure`` one) is returned
        with its ORIGINAL footprint and depth, untouched.
        """
        role_by_family_index = {
            idx: fam.role for idx, fam in enumerate(spec.effective_families())
        }

        def _role(intent: WebIntent) -> str:
            if intent.is_scar or intent.socket_radius_mm:
                # A scar has no producing family (family_index=-1) and is
                # never a role-yield participant either — mirrors the
                # is_scar guard below.
                return ROLE_SKIN_LATTICE
            return role_by_family_index.get(intent.family_index, ROLE_SKIN_LATTICE)

        remaining = list(intents)
        omitted: List[OmittedWeb] = []

        while True:
            order = sorted(
                range(len(remaining)),
                key=lambda idx: remaining[idx].footprint_x()[0],
            )
            # ALL-PAIRS scan, not CONSECUTIVE-pairs-only (see the module
            # docstring). ``order`` is sorted by left edge, but the guards
            # below (``family_index`` equality, ``entry_lower`` mismatch,
            # ``is_scar``) are CATEGORICAL: they reject a pair regardless of
            # how close its gap is, unlike the final gap-threshold check. A
            # consecutive-only scan would assume every skip is gap-based, so a
            # categorically-skipped candidate sitting between two in-band
            # candidates in ``order`` (e.g. a same-X mirror-side candidate —
            # the anisogrid config's
            # ``lattice_minus_upper``/``lattice_minus_lower`` pair, which share
            # an anchor and drift by construction) would hide the very pair
            # that should yield. ``order`` is still sorted so the inner loop
            # can stop as soon as ``gap`` — which is non-decreasing in ``b``'s
            # position once ``a`` is fixed, since footprints never move left as
            # ``order`` advances — clears the band.
            victim_pos: Optional[int] = None
            for a_rank in range(len(order)):
                i_a = order[a_rank]
                a = remaining[i_a]
                for b_rank in range(a_rank + 1, len(order)):
                    i_b = order[b_rank]
                    b = remaining[i_b]
                    # ``order`` is sorted by the same ``footprint_x()`` min_x, so
                    # the monotonicity this loop's early break relies on holds
                    # for a mix of tilted and untilted intents.
                    _a_min_x, a_max_x = a.footprint_x()
                    b_min_x, _b_max_x = b.footprint_x()
                    gap = b_min_x - a_max_x
                    if gap >= min_wall_mm - _NODE_SUPPRESSION_EPSILON_MM:
                        # Clears the band, and every later b in ``order``
                        # (further right) clears it too — stop scanning
                        # this a's partners entirely.
                        break
                    if a.family_index == b.family_index:
                        continue
                    if a.entry_lower != b.entry_lower:
                        continue
                    if a.is_scar or b.is_scar or a.socket_radius_mm or b.socket_radius_mm:
                        continue
                    role_a, role_b = _role(a), _role(b)
                    if role_a == role_b:
                        # Same-role pair (both lattice or both primary): not
                        # this pre-pass's concern, ordinary merge governs.
                        continue
                    if gap < 0:
                        # Overlap is left alone: each candidate cuts to its
                        # own requested depth independently.
                        continue
                    victim_pos = i_a if role_a == ROLE_SKIN_LATTICE else i_b
                    break
                if victim_pos is not None:
                    break
            if victim_pos is None:
                break
            victim = remaining.pop(victim_pos)
            omitted.append(
                OmittedWeb(
                    x_mm=victim.start[0],
                    reason=(
                        "role yield: {} lattice station at x={:.4f} mm "
                        "would merge with a primary_structure member "
                        "within the {:.4f} mm minimum-wall band -- omitted "
                        "rather than truncating the primary member's depth "
                        "or halving its wall at the merge (a shallow lattice member "
                        "crossing "
                        "a full-depth primary member)".format(
                            victim.family_name, victim.start[0], min_wall_mm
                        )
                    ),
                    family_index=victim.family_index,
                    family_name=victim.family_name,
                    k=None if victim.web_id is None else victim.web_id[1],
                    web_id=victim.web_id,
                )
            )

        return remaining, omitted

    def _merge_cross_family_nodes(
        self,
        intents: List[WebIntent],
        safety_offset_mm: float,
        *,
        frame: Optional[LayerFrame] = None,
        spec: Optional[StructureSpec] = None,
        effective_skin_margin_mm: Optional[float] = None,
        ctx: "LayerContext",
    ) -> Tuple[List[WebIntent], List[OmittedWeb]]:
        """Merge cross-family nodes into one slot per union interval.

        The method is named "merge" (not "suppress") because it never drops a
        candidate to resolve a node; it unions the footprints.
        ``tests/unit/test_node_merge.py`` calls it directly to pin the merge's
        pure X-placement arithmetic without driving the full ``StructureSpec``
        pitch/position/drift machinery (see that file's module docstring).

        Where two candidates from DIFFERENT families on the same entry surface
        are closer than ``2 * safety_offset_mm`` (the same ``_min_wall_mm``
        ``TopologyValidator`` enforces, called ``min_wall_mm`` here), i.e.
        ``gap < min_wall_mm``, they are MERGED into one ``WebIntent`` spanning
        the union of their footprints. Overlapping pairs (``gap < 0``) are the
        zero-gap end of the same band and are merged too when ``spec`` is given
        and their entry Y values differ by more than ``spec.overshoot_mm``
        (merging avoids leaving two independently resolved depths in place,
        which cut a material island on an elevon slice, layer 926); otherwise
        an overlapping pair keeps its original footprints. A same-family pair,
        however close, is never linked into a component: it is a plain
        placement mistake and ``TopologyValidator`` fails it closed.

        Connected-component merge: candidates are sorted ONCE by requested
        footprint (``footprint_x()[0]``), and a union-find links EVERY pair (not
        merely consecutive ones; see the module docstring) in that sorted order
        into the same component whenever they pass the guards below and their
        edge gap is below ``min_wall_mm``. Every component of size 1 is
        returned unchanged; every component of size 2+ becomes exactly one
        merged ``WebIntent`` (``_merge_node_component``). This is a SINGLE pass
        with no fixed-point restart, because merging cannot expose a new
        violating pair: the merged component's OUTER edges are its extreme
        members' own original edges (a footprint union takes the min/max of its
        members' edges, it does not move them), so the gap to whatever sits just
        outside the component is IDENTICAL to the gap that outside candidate
        already had against that same extreme member, a value the single pass
        already consulted. The result is also ORDER-INDEPENDENT by construction:
        the sort key, the adjacency test and the union-find's component
        membership are pure functions of each candidate's own
        ``start``/``width_mm``/``family_index``, never of the order candidates
        arrive in ``intents`` (see ``tests/unit/test_node_merge.py``'s
        ``OrderIndependenceTests``).

        A component's merged ``WebIntent`` gets a REPRESENTATIVE
        ``family_index``/``family_name`` (chosen by ``_merge_node_component``)
        but ALWAYS carries the full ``merged_family_indices`` tuple of every
        family that fed it. The representative is always the numerically LOWEST
        ``family_index`` among the component's members. It is a labelling
        choice only; it never changes a merged web's X, width, or Y. Alternating
        the label by component position would make
        ``set(i.family_index for i in result.intents)`` show both families even
        where the geometry contains only one, which is misleading, and a
        ``family_index`` compared as a safety label is the defect
        ``validate.py``'s ``_family_membership`` guards against. A caller that
        needs to know which families are present on a layer must inspect
        ``merged_family_indices`` (or ``WebIntent.family_membership()``), never
        ``family_index`` alone (see ``tests/unit/test_node_merge.py``'s
        ``BothFamiliesContinuousThroughEveryNodeTests``).

        Returns ``(kept, omitted)``. Merging never omits, so ``omitted`` comes
        only from the role-yield pre-pass (``_yield_lattice_to_primary``) run at
        the top of this method: a ``skin_lattice`` candidate that would
        otherwise merge with a ``primary_structure`` candidate in the same
        band is omitted instead of merged. Without that, a shallow lattice
        member crossing a full-depth primary member drags the primary down to
        stiffener depth (measured on the anisogrid config: 102/300 and 68/300
        layers on the NACA 2412 fixture had ``spar_front``/``spar_aft``
        truncated to the lattice family's 1.5 mm depth).

        Why omission, not "merge at the deeper depth": taking the deeper depth
        still runs the merge, so the primary's slot becomes the
        union-interval construction, whose node has HALVED wall thickness
        (through the band each member loses its inner wall). That compromises
        the primary at exactly the station a full-depth member exists to be
        strong at, and the re-measurement over the wider merged interval can
        trip the far-end-clearance/halo fail-closed checks near curvature
        (leading edge) where the unmerged primary alone would have passed,
        turning a printable config into a slice failure for a member that did
        not need to move. Omission leaves the primary's own ``WebIntent``
        identical to a lattice-free run: no merge, no re-measurement, no new
        fail-closed surface. It is the same policy ``_omit_opposing_conflicts``
        applies to a cross-edge conflict ("the SHALLOWER of the two... is
        omitted instead, and the deeper survives untouched"), applied here to
        a same-edge, cross-ROLE conflict.

        Scope of the pre-pass: only a ``skin_lattice``/``primary_structure``
        PAIR in the same sub-``min_wall_mm`` band that the merge would claim
        (same ``entry_lower``, neither ``is_scar``, cross-family,
        ``0 <= gap < min_wall_mm``). A same-role pair (lattice-lattice or
        primary-primary), however close, is not affected and goes through the
        ordinary merge below. True overlap (``gap < 0``) is left alone: each
        candidate cuts to its own requested depth independently, so the
        primary's full-depth cut is not touched by an overlapping lattice
        member (no crossing on the anisogrid config lands there). The pre-pass
        needs ``spec`` to look up each candidate's family's declared ``role``
        (never inferred from depth, matching ``WebFamily.role``); callers that
        omit ``spec`` (the direct-call unit tests in
        ``tests/unit/test_node_merge.py`` and ``test_node_merge_scope.py``) skip
        it.
        """
        min_wall_mm = 2.0 * safety_offset_mm

        role_yield_omitted: List[OmittedWeb] = []
        if spec is not None and len(intents) > 1:
            intents, role_yield_omitted = self._yield_lattice_to_primary(
                intents, min_wall_mm, spec
            )

        n = len(intents)
        if n <= 1:
            return list(intents), role_yield_omitted

        order = sorted(
            range(n),
            key=lambda idx: intents[idx].footprint_x()[0],
        )

        # Plain union-find (path-compressed find, unranked union — n is a
        # handful of candidates per layer, never enough to need union-by
        # -rank). ``parent[i] == i`` means ``i`` is its own component root.
        parent = list(range(n))

        def _find(i: int) -> int:
            root = i
            while parent[root] != root:
                root = parent[root]
            while parent[i] != root:
                parent[i], i = root, parent[i]
            return root

        def _union(a: int, b: int) -> None:
            ra, rb = _find(a), _find(b)
            if ra != rb:
                parent[rb] = ra

        # ALL-PAIRS scan, not CONSECUTIVE-pairs-only (see the module
        # docstring, and the matching scan in ``_yield_lattice_to_primary``).
        # A consecutive-only scan is correct ONLY if every skipped pair is
        # skipped because its gap already clears the band. The
        # ``family_index``/``entry_lower``/``is_scar`` guards below are
        # CATEGORICAL, not gap-based: they reject a pair no matter how small
        # its gap is. A categorically-rejected candidate sitting BETWEEN two
        # candidates that belong in the same band (in ``order``'s left-edge
        # sort) would hide the real pair from ever being compared. This
        # occurs on the anisogrid config's ``lattice_minus_upper`` (+Y) /
        # ``lattice_minus_lower`` (-Y) pair, which the config anchors at the
        # IDENTICAL X by construction (same ``web_position``, opposite
        # ``z_drift_mm_per_mm``): the -Y candidate always sorts between the
        # two +Y candidates that should merge (host slice, layer 5,
        # Z=1.20 mm, x=-8.9/-8.8, 0.079 mm wall). ``order`` stays sorted so
        # the inner loop can stop the moment ``gap`` — non-decreasing in
        # ``b``'s rank once ``a`` is fixed, since footprints never move left
        # as ``order`` advances — clears the band.
        for a_rank in range(len(order)):
            i_a = order[a_rank]
            a = intents[i_a]
            for b_rank in range(a_rank + 1, len(order)):
                i_b = order[b_rank]
                b = intents[i_b]
                # ``order`` is sorted by the same ``footprint_x()`` min_x, so
                # the "footprints never move left as order advances"
                # monotonicity this loop's early break relies on holds for a
                # mix of tilted and untilted intents.
                _a_min_x, a_max_x = a.footprint_x()
                b_min_x, _b_max_x = b.footprint_x()
                # Overlapping same-side cross-family slots are the zero-gap end
                # of this same node band (see the ``gap < 0`` guard below).
                gap = b_min_x - a_max_x
                if gap >= min_wall_mm - _NODE_SUPPRESSION_EPSILON_MM:
                    # Clears the minimum wall on its own — nothing to merge,
                    # and every later b in ``order`` clears it too.
                    break
                if a.family_index == b.family_index:
                    # Same-family pair: never linked, regardless of gap — it is
                    # a plain placement mistake for the validator to reject.
                    continue
                # A tilted candidate IS allowed to link into a same-edge merged
                # node: ``_merge_node_component``
                # (below) never reads a member's ``end`` at all for the merged
                # node's own placement, only ``start[0]`` (entry X, which never
                # moves -- only ``end``'s X can tilt) for the union footprint,
                # and ``abs(start[1] - end[1])`` (a Y-distance, unaffected by
                # tilt -- see ``WebIntent.footprint_x``'s own docstring and the
                # module's comment at the tilt call site) for the shallower
                # member's depth. The merged node's own ``start``/``end`` are
                # then computed FRESH, from the union footprint's re-measured
                # local surfaces -- always a plain vertical node, regardless of
                # whether any absorbed member was tilted. So a tilted member
                # simply loses its own individual tilt when merged (replaced by
                # the merge's own safe, freshly-measured placement) -- there is
                # no undefined "merged tilted geometry" to protect against.
                # Excluding tilted members from merging would break the
                # skin_lattice diamond-crossing pattern (two same-role families
                # whose stations cross in X by design and rely on this merge to
                # avoid a real sub-bead-wall collision).
                if a.entry_lower != b.entry_lower:
                    # A +Y candidate and a -Y candidate are never linked into a
                    # same-edge node, regardless of gap or family. The merge
                    # models two candidates joining into ONE WIDER notch on the SAME
                    # edge (a real geodetic node) — that construction has no
                    # meaning for two cuts entering from OPPOSITE surfaces,
                    # which is a different geometric question entirely (whether
                    # the two cuts REACH far enough toward each other to meet),
                    # governed by the cross-edge ALL-PAIRS opposing check in
                    # ``validate.py``, not by X-adjacency here. Without this
                    # guard, a close +Y/-Y pair from different families would
                    # fall through to the ordinary merge branch below and be
                    # unioned into a single ``WebIntent`` with no well-defined
                    # single far-end direction — a construction
                    # ``_merge_node_component`` was never designed to
                    # represent.
                    continue
                if a.is_scar or b.is_scar or a.socket_radius_mm or b.socket_radius_mm:
                    # a scar's far end deliberately terminates INSIDE a void
                    # (the declared hole), never against solid material —
                    # ``_merge_node_component``'s far-end arithmetic
                    # (``local_lower_worst_case(...) + skin_margin_mm`` and its
                    # mirror) assumes a far end measured against solid
                    # material, which a scar's far end is not. Merging a scar
                    # with an ordinary candidate would ask that arithmetic to
                    # represent a footprint it was never designed for — see
                    # ``WebIntent.is_scar``'s own docstring. Excluded
                    # unconditionally, the same way the ``entry_lower`` guard
                    # above excludes a cross-edge pair.
                    continue
                if gap < 0 and (
                    spec is None
                    or abs(a.start[1] - b.start[1]) <= spec.overshoot_mm
                ):
                    # Overlapping pair that does not differ in entry Y (or no
                    # ``spec`` to read ``overshoot_mm`` from): not merged;
                    # each candidate keeps its own footprint and depth.
                    continue
                _union(i_a, i_b)

        components: List[List[int]] = []
        roots_seen: dict = {}
        for i in range(n):
            root = _find(i)
            if root not in roots_seen:
                roots_seen[root] = len(components)
                components.append([])
            components[roots_seen[root]].append(i)

        # No ordering step needed here: ``_merge_node_component``'s
        # representative choice is a FIXED rule (always the lowest
        # ``family_index`` among a component's own members — see that
        # method's docstring), a pure function of the component's own
        # members, not of which order components are visited in. The
        # caller-facing order of ``kept`` is normalized below regardless
        # (``kept.sort(...)``), so nothing here depends on ``components``'
        # own iteration order either.
        kept: List[WebIntent] = []
        for member_indices in components:
            if len(member_indices) == 1:
                kept.append(intents[member_indices[0]])
                continue
            members = [intents[i] for i in member_indices]
            try:
                merged_intent = self._merge_node_component(
                    members,
                    safety_offset_mm,
                    frame,
                    spec,
                    effective_skin_margin_mm,
                    ctx,
                )
            except ReEntrantSectionError as exc:
                # With ``per_station_reentrant`` (default off): the merged
                # union interval re-measures local surfaces over a wider span
                # than either constituent, so it can hit a notch neither did.
                # Omit the constituents of THIS node only.
                if spec is None or not _per_station_reentrant(spec):
                    raise
                for m in members:
                    role_yield_omitted.append(OmittedWeb(
                        x_mm=m.start[0],
                        reason=(
                            "re_entrant_station: merged node (families {}) "
                            "re-measures a section that is re-entrant over "
                            "the union interval ({}) -- this node's "
                            "constituents are omitted, the rest of the "
                            "layer keeps its structure.".format(
                                sorted(set(x.family_index for x in members)),
                                exc)),
                        family_index=m.family_index,
                        family_name=m.family_name,
                        k=None if m.web_id is None else m.web_id[1],
                        web_id=m.web_id,
                        merged_web_ids=m.merged_web_ids,
                    ))
                continue
            kept.append(merged_intent)

        kept.sort(key=lambda i: i.start[0])
        return kept, role_yield_omitted

    def _merge_node_component(
        self,
        members: List[WebIntent],
        safety_offset_mm: float,
        frame: Optional[LayerFrame],
        spec: Optional[StructureSpec],
        effective_skin_margin_mm: Optional[float],
        ctx: "LayerContext",
    ) -> WebIntent:
        """One union-interval merge, from 2+ same-node candidates.

        The merged ``WebIntent``'s REPRESENTATIVE ``family_index``/
        ``family_name`` is a FIXED rule — always the numerically LOWEST
        ``family_index`` among ``members`` — a pure function of the
        component's own members only, never of scan/construction/component
        -iteration order. It is a reporting/labelling choice, nothing more:
        it never changes the merged web's X, width, or Y, and nothing in
        ``aircraft_core/`` or ``plugin/`` branches on WHICH family a merge
        reports as its representative — see ``WebIntent.family_membership``
        and ``validate.py``'s ``_family_membership`` for the actual
        family-membership consumers, which always use the FULL
        ``merged_family_indices``, never this representative.

        Footprint: ``[component_min_x, component_max_x]`` — the leftmost
        member's near edge to the rightmost member's far edge. ``width_mm``
        and the centre X follow directly.

        Depth: the SHALLOWEST of the members' own depths. "Shallower" could
        mean literally re-applying one member's already-recorded Y-distance,
        or comparing the two placements' actual RESULTING far-Y once both are
        re-measured over the merged interval. This implementation takes the
        second reading, as the shallower of TWO independently valid far-Y
        candidates when ``frame`` is given:
        (a) the shallower member's own recorded depth, applied below the
        interval's re-measured entry (honours a genuinely DEPTH-LIMITED
        member's hard request), and (b) the standard full-depth placement
        — ``local_lower_worst_case(...) + effective_skin_margin_mm +
        FULL_DEPTH_MARGIN_HEADROOM_MM``, re-measured over the SAME merged
        interval (the same formula ``_generate_family`` uses for a single
        full-depth station, just over the wider span). Comparing actual
        resulting far-Y rather than reusing an out-of-date depth NUMBER matters in
        practice: on the diamond-lattice preset's NACA 2412 fixture,
        re-measuring the worst-case entry over a merged interval legitimately
        drops it below either constituent's own narrow reading (real
        curvature, not a bug; see the module docstring's "Far-end clearance"
        paragraph), and subtracting a depth measured over the narrow span from that lower
        entry manufactures a spurious ~0.002 mm margin breach neither
        constituent had. Taking candidate (b) as well and keeping whichever is
        shallower avoids that breach for the common (both full-depth) case
        while still honouring a genuinely shallower depth-limited request via
        candidate (a). Both candidates, and the final choice between them,
        are checked against the same span's worst-case lower surface and
        (when in scope) the halo; both fail closed rather than silently
        narrowing or omitting the merge (see the module docstring's "Far-end
        clearance" and "Halo" paragraphs).

        Without a ``frame`` (this method's unit tests, and
        ``tests/unit/test_node_merge.py``'s direct calls into
        ``_merge_cross_family_nodes``; no geometry to re-query), the
        shallowest member's own already-measured ``(start, end)`` Y pair is
        reused verbatim, re-centred at the merged station: a best-effort
        fallback, not a proof, like every other worst-case-over-a-span
        measurement in this module when its inputs run out.
        """
        footprints = [
            (m.start[0] - m.width_mm / 2.0, m.start[0] + m.width_mm / 2.0, m)
            for m in members
        ]
        component_min_x = min(f[0] for f in footprints)
        component_max_x = max(f[1] for f in footprints)
        width_mm = component_max_x - component_min_x
        center_x = (component_min_x + component_max_x) / 2.0

        merged_family_indices = tuple(sorted(set(m.family_index for m in members)))
        # Representative family — a FIXED rule (always the lowest
        # family_index among this component's members), never alternated.
        # See this method's own docstring for why a fixed rule is correct
        # here: nothing downstream branches on which family is reported as
        # the representative, so there is nothing for alternation to buy.
        representative = min(members, key=lambda m: m.family_index)

        shallow_depth = min(abs(m.start[1] - m.end[1]) for m in members)

        # Every member of a component reaching this method is guaranteed (by
        # the caller's linking rule, see ``_merge_cross_family_nodes``'s
        # ``a.entry_lower != b.entry_lower`` guard) to share the SAME entry
        # side — a component can never mix +Y and -Y members. ``sign`` mirrors
        # every formula below exactly the way ``_generate_family``'s own
        # ``sign`` does.
        entry_lower = members[0].entry_lower
        sign = -1.0 if entry_lower else 1.0

        if frame is not None:
            if entry_lower:
                entry_y = local_lower_worst_case(
                    frame.ops, frame.polygon, center_x, width_mm, DEFAULT_PROBE_WIDTH_MM
                )
                y_far_full = local_upper_worst_case(
                    frame.ops, frame.polygon, center_x, width_mm, DEFAULT_PROBE_WIDTH_MM
                )
            else:
                entry_y = local_upper_worst_case(
                    frame.ops, frame.polygon, center_x, width_mm, DEFAULT_PROBE_WIDTH_MM
                )
                y_far_full = local_lower_worst_case(
                    frame.ops, frame.polygon, center_x, width_mm, DEFAULT_PROBE_WIDTH_MM
                )
            y_min_full = y_far_full

            # Candidate (a): the shallower member's own recorded depth,
            # applied below the freshly re-measured entry.
            far_y = entry_y - sign * shallow_depth
            # Candidate (b): the standard full-depth placement, re-measured
            # over the merged interval — see this method's own docstring
            # ("Depth: ... UNDERDETERMINED POINT") for why both are
            # compared rather than only (a).
            if effective_skin_margin_mm is not None:
                full_depth_far_y = (
                    y_far_full
                    + sign * (effective_skin_margin_mm + FULL_DEPTH_MARGIN_HEADROOM_MM)
                )
                # Shallower wins — the higher (less deep) of the two Y
                # candidates, sign-generic: for +Y "higher" means the
                # numerically larger Y; for -Y it means the numerically
                # smaller Y (closer to the -Y entry surface), so the
                # generic "shallower wins" comparison is
                # ``sign * far_y`` maximal, i.e. picking the larger of
                # ``sign * candidate``.
                far_y = far_y if sign * far_y >= sign * full_depth_far_y else full_depth_far_y

            if effective_skin_margin_mm is not None:
                # Far-end clearance: the merged slot's far end must clear the
                # local far surface across the WHOLE union interval, not
                # either constituent's own narrower span. See the module
                # docstring's "Far-end clearance" paragraph.
                remaining_skin_mm = sign * (far_y - y_min_full)
                if remaining_skin_mm < effective_skin_margin_mm - _DEPTH_EPSILON_MM:
                    raise ValueError(
                        "node merge at x=[{:.4f}, {:.4f}] mm (families {}): "
                        "the merged slot's far end would leave only {:.4f} "
                        "mm of skin measured across the FULL {:.4f} mm "
                        "union interval, below the required {:.4f} mm "
                        "skin_margin_mm — the merge cannot be placed "
                        "without breaching the far-end clearance "
                        "obligation. Refusing to "
                        "silently narrow the merged interval or deepen "
                        "the cut.".format(
                            component_min_x,
                            component_max_x,
                            merged_family_indices,
                            remaining_skin_mm,
                            width_mm,
                            effective_skin_margin_mm,
                        )
                    )

            # Halo obligation — see the module docstring's "Halo /
            # thin-section interaction" paragraph for why a breach here ALWAYS
            # fails closed rather than routing through the ordinary
            # thin-section omission channel. Same scope gate as
            # _generate_family's own check (spec.skin_margin_mm below 2x the
            # safety offset violates TopologyValidator's precondition; it is
            # not a thinness finding).
            if spec is not None and spec.skin_margin_mm >= 2.0 * safety_offset_mm:
                # Same near-extremity widening as ``_generate_family``'s copy
                # of this check — see that copy's comment for the full
                # derivation and why "widen by depth everywhere" was rejected.
                # The rule is applied separately here (not through a shared
                # helper) because the two call sites' surrounding variables
                # (``x``/``local_entry_y``/``end_y`` vs.
                # ``center_x``/``entry_y``/``far_y``) differ; the widening
                # rule itself is identical, so it must be changed in both
                # places together.
                depth_used_mm = abs(entry_y - far_y)
                halo_span_mm = width_mm + 2.0 * safety_offset_mm
                halo_min_x = center_x - halo_span_mm / 2.0
                halo_max_x = center_x + halo_span_mm / 2.0
                # Same "capture before the extremity widen" discipline as
                # ``_generate_family``'s copy — see that copy's comment for
                # the full derivation.
                ordinary_halo_min_x = halo_min_x
                ordinary_halo_max_x = halo_max_x
                if halo_min_x - frame.min_x < depth_used_mm:
                    halo_min_x = frame.min_x
                if frame.max_x - halo_max_x < depth_used_mm:
                    halo_max_x = frame.max_x
                clamped_min_x = max(halo_min_x, frame.min_x + _FOOTPRINT_EPSILON_MM)
                clamped_max_x = min(halo_max_x, frame.max_x - _FOOTPRINT_EPSILON_MM)
                clamped_span_mm = clamped_max_x - clamped_min_x
                if clamped_span_mm > 0:
                    halo_center_x = (clamped_min_x + clamped_max_x) / 2.0
                    try:
                        # mirrors entry_y/y_far_full above -- for a -Y merged
                        # component the far side's halo probe is the UPPER
                        # surface, not the lower one.
                        if entry_lower:
                            far_halo_y = local_upper_worst_case(
                                frame.ops,
                                frame.polygon,
                                halo_center_x,
                                clamped_span_mm,
                                DEFAULT_PROBE_WIDTH_MM,
                            )
                        else:
                            far_halo_y = local_lower_worst_case(
                                frame.ops,
                                frame.polygon,
                                halo_center_x,
                                clamped_span_mm,
                                DEFAULT_PROBE_WIDTH_MM,
                            )
                    except ValueError:
                        # Belt-and-suspenders, same reasoning as
                        # _generate_family's own fallback: this halo
                        # measurement is a best-effort approximation, not a
                        # proof, and must never itself become a new fail-closed
                        # path.
                        far_halo_y = y_min_full
                else:
                    far_halo_y = y_min_full
                remaining_halo_mm = sign * (far_y - far_halo_y)
                # Identical local-slope correction to ``_generate_family``'s
                # copy of this check — see that copy's comment for the full
                # derivation (why LOCAL slope, not global rotation; why the
                # ORDINARY, un-widened window). Applied separately for the
                # reason given in the widening comment above.
                far_slope_deg = estimate_far_surface_slope_deg(
                    frame.ops,
                    frame.polygon,
                    ordinary_halo_min_x,
                    ordinary_halo_max_x,
                    upper=entry_lower,
                )
                required_halo_mm = corrected_skin_margin_mm(
                    2.0 * safety_offset_mm,
                    far_slope_deg if far_slope_deg is not None else frame.rotation_deg,
                )
                if remaining_halo_mm < required_halo_mm - _THIN_SECTION_EPSILON_MM:
                    # Raise the invariant error type (``TopologyError``) directly,
                    # rather than a bare ``ValueError``.
                    #
                    # In-host, ``capability.py``'s ``except (ValueError,
                    # ArithmeticError)`` handler around ``generator.generate()``
                    # would translate a ``ValueError`` into a ``TopologyError``
                    # carrying the layer index and ``layer.print_z`` anyway.
                    # Callers that use ``aircraft_core`` DIRECTLY have no such
                    # translation: ``tools/verify_full_sweep.py``,
                    # ``tools/verify_full_sweep_varying.py`` and the
                    # ``tools/size_wing.py`` search built on them would
                    # classify a ``ValueError`` as "UNEXPECTED (not a clean
                    # TopologyError)", which reads as a crash rather than the
                    # deliberate fail-closed refusal it is. Raising the real
                    # type makes this module self-describing instead of
                    # depending on a caller to attribute it.
                    raise TopologyError(
                        ctx.layer_index,
                        ctx.print_z,
                        "node merge at x=[{:.4f}, {:.4f}] mm (families {}): "
                        "the merged slot's {:.4f} mm erosion halo leaves "
                        "only {:.4f} mm of remaining skin, below 2x the "
                        "{:.4f} mm safety offset ({:.4f} mm required, "
                        "{:.4f} mm after this station's own {} local "
                        "far-surface slope correction). "
                        "Every constituent station here already passed "
                        "its OWN halo check individually — that is the "
                        "only way it could have reached the merge at all "
                        "— so this is not an ordinary thin-section "
                        "omission: omitting the merge here would silently "
                        "delete BOTH members, which the halo obligation "
                        "forbids. Refusing to merge; "
                        "the merged slot's width, not its topology, is "
                        "what should move first.".format(
                            component_min_x,
                            component_max_x,
                            merged_family_indices,
                            clamped_span_mm,
                            remaining_halo_mm,
                            safety_offset_mm,
                            2.0 * safety_offset_mm,
                            required_halo_mm,
                            "{:.4f} degree".format(far_slope_deg)
                            if far_slope_deg is not None
                            else "{:.4f} degree (fallback: global rotation)".format(
                                frame.rotation_deg
                            ),
                        )
                    )
        else:
            shallow_member = min(members, key=lambda m: abs(m.start[1] - m.end[1]))
            entry_y = shallow_member.start[1]
            far_y = shallow_member.end[1]

        # every constituent's identity and own station X, so the report can
        # credit the merged executed depth to each constituent web.
        merged_ids: List[Tuple[int, int, str]] = []
        merged_xs: List[float] = []
        for m in members:
            if m.merged_web_ids:
                merged_ids.extend(m.merged_web_ids)
                merged_xs.extend(m.merged_web_xs)
            elif m.web_id is not None:
                merged_ids.append(m.web_id)
                merged_xs.append(m.start[0])
        return WebIntent(
            start=(center_x, entry_y),
            end=(center_x, far_y),
            width_mm=width_mm,
            family_index=representative.family_index,
            family_name=representative.family_name,
            merged_family_indices=merged_family_indices,
            entry_lower=entry_lower,
            web_id=representative.web_id,
            merged_web_ids=tuple(merged_ids),
            merged_web_xs=tuple(merged_xs),
        )

    def _omit_opposing_conflicts(
        self,
        intents: List[WebIntent],
        frame: LayerFrame,
        safety_offset_mm: float,
        overshoot_mm: float,
    ) -> Tuple[List[WebIntent], List[OmittedWeb]]:
        """Omit whichever station of a colliding +Y/-Y pair is safe to drop,
        rather than let ``TopologyValidator`` fail the WHOLE layer closed over
        one opposing-station phase collision.

        The problem: on a tapered wing the two-sided default collides, and
        only the validator notices. A full-depth spar
        family anchors by chord FRACTION (``frame.web_x``), while a marching
        stiffener family's other stations sit at a FIXED MM OFFSET from ITS OWN
        anchor. On a constant-section wing the two conventions agree
        everywhere; on a tapered wing the marched offset becomes a growing
        fraction of the shrinking chord, so the two families' relative phase
        sweeps continuously along the span and periodically brings a shallow
        stiffener station directly opposite a full-depth spar station. A
        full-depth web opposed by ANY web on the other surface fails the
        no-sever proof's meeting condition at every depth this project would
        configure, so this is not a narrow special case: it is the general
        opposing-pair inequality landing on the geometry a tapered two-sided
        default produces.

        Why omission, not shifting: nudging the colliding station sideways
        would silently change the ACTUAL pitch between it and its neighbours
        away from the pitch's own buckling-derived value, which the structure
        exists to hold. Omission removes one station and leaves every
        SURVIVING bay at its designed pitch; a caller can see what happened
        from the recorded ``OmittedWeb`` reason.

        Which side of the pair is dropped: the SHALLOWER of the two (ties —
        equal depth — broken by dropping the member with the larger
        ``family_index``, a fixed, deterministic rule, never by construction
        order). This generalizes "omit the offending stiffener, not the spar":
        a full-depth spar is deeper than any depth-limited stiffener family,
        so "shallower loses" keeps the spar in the default case, while still
        terminating deterministically on a shallow-vs-shallow collision (e.g.
        two skin-stiffener families on opposite surfaces meeting near a thin
        trailing edge) that the same proof covers.

        Same predicate as the validator, deliberately: this calls
        ``frame.opposing_pair_meets`` — the identical shared function
        ``TopologyValidator``'s cross-edge all-pairs check calls (see its
        "opposing-entry check") — rather than a second copy of the meeting
        condition. This method measures each candidate's REQUESTED footprint
        (before any boolean runs) and its
        ``WebIntent.reach_depth_mm(overshoot_mm)`` — NOT ``depth_mm()`` — while
        the validator measures the ACTUALLY REMOVED region post-boolean.
        ``depth_mm()`` (the requested centerline depth alone) would omit the
        overshoot term, and that term alone is enough to reach the validator
        on tapered/thin-airfoil fixtures. ``reach_depth_mm`` is a PROVABLE
        upper bound on what ``TopologyValidator`` can measure for the same
        slot (``ops.intersection(original, slot)`` is a subset of ``slot``;
        see that method's docstring), computed without any boolean call. What
        remains unmeasurable at generation time is the footprint-side
        residual — how much of a curved local surface the actually-removed
        region's X-bbox spans versus the requested, symmetric footprint —
        which ``footprint_x()``'s "erring wide, never narrow" bound covers
        from the other direction.

        Fixed-point (restart), not a single pass over the original list:
        dropping one station can make a conflict that existed only because of
        that station disappear — e.g. a shallow candidate loses to a deep one,
        and a SEPARATE candidate on the same side that only conflicted with the
        now-dropped shallow one need not be touched at all. A single pass that
        decided every pair's loser against the ORIGINAL set would over-omit.
        So this method finds ONE conflicting pair at a time (deterministic
        order: ascending upper index, then ascending lower index, both stable
        w.r.t. ``intents``' own order), drops ONLY that pair's loser, and
        rescans the SURVIVING set from scratch until no surviving pair
        conflicts. Unlike the node merge (which can only shrink the set of
        conflicts and needs no restart), omission is a removal, which is what
        a restart is for.

        Runs on the intent list AFTER ``_merge_cross_family_nodes`` — see the
        call site in ``generate()`` for why same-edge merging must resolve
        first.

        Boundary-halo measurement failures are treated as conflicts, not
        skipped: near the very tip of the chord, widening a near-boundary
        candidate's footprint by ``safety_offset_mm`` (the halo) can
        push the OVERLAP SPAN's own probe stations past
        ``frame.min_x``/``frame.max_x``, which ``opposing_pair_meets``
        propagates as ``ValueError`` exactly as ``local_upper_worst_case``/
        ``local_lower_worst_case`` do — the same condition
        ``TopologyValidator``'s own opposing-pair check treats as an
        unconditional fail-closed ``TopologyError``. Left unhandled here
        that would abort ``generate()`` entirely over one near-tip pair,
        defeating this method's whole purpose — so it is caught and treated
        conservatively as a conflict (never silently assumed safe), with
        the shallower candidate omitted and a reason naming the
        measurement failure instead of a thickness figure it could not
        obtain.
        """
        n = len(intents)
        if n <= 1:
            return list(intents), []

        min_wall_mm = 2.0 * safety_offset_mm

        def _footprint(intent: WebIntent) -> Tuple[float, float]:
            # WebIntent.footprint_x(): exact for an untilted intent,
            # conservative for a tilted one. See that method's docstring.
            return intent.footprint_x()

        def _depth(intent: WebIntent) -> float:
            # WebIntent.reach_depth_mm(overshoot_mm) -- a provable upper bound
            # on what TopologyValidator can measure for this slot's
            # actually-removed region (see that method's docstring), NOT
            # depth_mm() (the requested centerline depth alone, which omits the
            # overshoot_mm SlotBuilder cuts past the entry boundary). Equal to
            # depth_mm() + overshoot_mm for every untilted intent.
            return intent.reach_depth_mm(overshoot_mm)

        surviving = list(intents)
        omitted: List[OmittedWeb] = []

        while True:
            upper = [i for i in range(len(surviving)) if not surviving[i].entry_lower]
            lower = [i for i in range(len(surviving)) if surviving[i].entry_lower]
            if not upper or not lower:
                break

            conflict = None
            unmeasurable = None
            for i_u in upper:
                u_min_x, u_max_x = _footprint(surviving[i_u])
                d_upper = _depth(surviving[i_u])
                for i_l in lower:
                    l_min_x, l_max_x = _footprint(surviving[i_l])
                    d_lower = _depth(surviving[i_l])
                    # A haloed overlap span that cannot be measured at all
                    # (near the very tip of the chord, where widening a
                    # near-boundary footprint by ``safety_offset_mm`` can
                    # push the OVERLAP span's own sample stations a hair
                    # past ``frame.min_x``/``frame.max_x`` — the same
                    # boundary-halo artifact ``_generate_family``'s own
                    # thin-section check clamps against) is EXACTLY the
                    # condition ``TopologyValidator``'s opposing-pair check
                    # treats as an unconditional fail-closed TopologyError
                    # (see its own "could not be measured across the
                    # overlap span" message). Reproducing that as a crash
                    # HERE — at generation time, before any slot is even
                    # built — would defeat this whole method's purpose: one
                    # near-tip station would abort the ENTIRE layer instead
                    # of being omitted on its own. So this candidate PAIR is
                    # conservatively treated as a conflict (never as safe —
                    # "cannot prove safe" must never default to "assume
                    # safe"), and the shallower side is omitted exactly as
                    # an ordinary meeting-condition conflict would be,
                    # naming the measurement failure instead of a
                    # thickness figure it does not have.
                    try:
                        meeting = opposing_pair_meets(
                            frame.ops,
                            frame.polygon,
                            u_min_x,
                            u_max_x,
                            d_upper,
                            l_min_x,
                            l_max_x,
                            d_lower,
                            safety_offset_mm,
                            min_wall_mm,
                            DEFAULT_PROBE_WIDTH_MM,
                        )
                    except ValueError as exc:
                        unmeasurable = (i_u, i_l, exc)
                        continue
                    if meeting is not None:
                        conflict = (i_u, i_l, meeting)
                        break
                if conflict is not None:
                    break

            if conflict is None and unmeasurable is None:
                break

            if conflict is not None:
                i_u, i_l, meeting = conflict
                u_intent = surviving[i_u]
                l_intent = surviving[i_l]
                # A declared hole's scar never loses. Unlike an ordinary
                # web, a scar is not a candidate the generator chose to
                # place -- it connects a mesh feature (the hole) that
                # cannot be omitted at all (see
                # StructureSpec.expected_hole_count and WebIntent.is_scar).
                # So the scar's own depth never enters this decision:
                # whichever side is NOT the scar loses, unconditionally,
                # before the ordinary "shallower loses" tie-break below is
                # consulted. At most one side of any opposing pair can be a
                # scar (one hole per layer), so this never leaves both sides
                # exempt.
                scar_priority = False
                if u_intent.is_scar and not l_intent.is_scar:
                    loser_i, winner_i = i_l, i_u
                    scar_priority = True
                elif l_intent.is_scar and not u_intent.is_scar:
                    loser_i, winner_i = i_u, i_l
                    scar_priority = True
                # "Shallower loses" (this method's docstring); ties
                # broken by the larger family_index losing, a fixed rule
                # independent of which of i_u/i_l happens to be smaller.
                elif meeting.d_upper_mm < meeting.d_lower_mm:
                    loser_i, winner_i = i_u, i_l
                elif meeting.d_lower_mm < meeting.d_upper_mm:
                    loser_i, winner_i = i_l, i_u
                elif u_intent.family_index >= l_intent.family_index:
                    loser_i, winner_i = i_u, i_l
                else:
                    loser_i, winner_i = i_l, i_u
                loser_intent = surviving[loser_i]
                winner_intent = surviving[winner_i]

                if scar_priority:
                    # The loser here need not be shallower at all -- the winner
                    # is the SCAR, which can never be omitted (see the comment
                    # above), so the message must not
                    # claim "shallower" when depth was never the deciding
                    # factor.
                    reason = (
                        "opposing entry: candidate at x={:.4f} mm "
                        "({}, depth {:.4f} mm) meets the declared "
                        "spar-hole scar at x={:.4f} mm ({}, depth "
                        "{:.4f} mm) over the overlap span "
                        "x=[{:.4f}, {:.4f}] mm — local thickness "
                        "{:.4f} mm cannot accommodate both depths plus "
                        "the {:.4f} mm minimum wall ({:.4f} mm "
                        "required). The scar is never omitted (a "
                        "declared hole cannot be silently left "
                        "unconnected -- see StructureSpec.expected_"
                        "hole_count's own docstring), so this "
                        "candidate is omitted instead, regardless of "
                        "which side is shallower.".format(
                            loser_intent.start[0],
                            "-Y" if loser_intent.entry_lower else "+Y",
                            _depth(loser_intent),
                            winner_intent.start[0],
                            "-Y" if winner_intent.entry_lower else "+Y",
                            _depth(winner_intent),
                            meeting.overlap_min_x,
                            meeting.overlap_max_x,
                            meeting.t_local_conservative,
                            min_wall_mm,
                            meeting.required_mm,
                        )
                    )
                else:
                    reason = (
                        "opposing entry: candidate at x={:.4f} mm "
                        "({}, depth {:.4f} mm) meets family {!r}'s "
                        "candidate at x={:.4f} mm ({}, depth {:.4f} mm) "
                        "over the overlap span x=[{:.4f}, {:.4f}] mm — "
                        "local thickness {:.4f} mm cannot accommodate "
                        "both depths plus the {:.4f} mm minimum wall "
                        "({:.4f} mm required). The shallower candidate "
                        "is omitted, not shifted (see "
                        "_omit_opposing_conflicts's own docstring for "
                        "why): shifting would silently change the pitch "
                        "between surviving stations.".format(
                            loser_intent.start[0],
                            "-Y" if loser_intent.entry_lower else "+Y",
                            _depth(loser_intent),
                            winner_intent.family_name,
                            winner_intent.start[0],
                            "-Y" if winner_intent.entry_lower else "+Y",
                            _depth(winner_intent),
                            meeting.overlap_min_x,
                            meeting.overlap_max_x,
                            meeting.t_local_conservative,
                            min_wall_mm,
                            meeting.required_mm,
                        )
                    )
            else:
                i_u, i_l, exc = unmeasurable
                u_intent = surviving[i_u]
                l_intent = surviving[i_l]
                d_u, d_l = _depth(u_intent), _depth(l_intent)
                # Same scar-never-loses priority as the ordinary conflict
                # branch above -- see its comment.
                if u_intent.is_scar and not l_intent.is_scar:
                    loser_i, winner_i = i_l, i_u
                elif l_intent.is_scar and not u_intent.is_scar:
                    loser_i, winner_i = i_u, i_l
                # Same "shallower loses" tie-break — the only figures
                # available here are each candidate's own requested depth,
                # exactly as the ordinary conflict branch above uses.
                elif d_u < d_l or (d_u == d_l and u_intent.family_index >= l_intent.family_index):
                    loser_i, winner_i = i_u, i_l
                else:
                    loser_i, winner_i = i_l, i_u
                loser_intent = surviving[loser_i]
                winner_intent = surviving[winner_i]

                reason = (
                    "opposing entry: candidate at x={:.4f} mm ({}, depth "
                    "{:.4f} mm) could not be measured against family "
                    "{!r}'s candidate at x={:.4f} mm ({}, depth {:.4f} mm) "
                    "over their (haloed) overlap span: {} — treated "
                    "conservatively as a conflict (never assumed safe) "
                    "and omitted, the shallower candidate, rather than "
                    "aborting the whole layer over a boundary-halo "
                    "measurement artifact near the chord extent.".format(
                        loser_intent.start[0],
                        "-Y" if loser_intent.entry_lower else "+Y",
                        _depth(loser_intent),
                        winner_intent.family_name,
                        winner_intent.start[0],
                        "-Y" if winner_intent.entry_lower else "+Y",
                        _depth(winner_intent),
                        exc,
                    )
                )

            omitted.append(
                OmittedWeb(
                    x_mm=loser_intent.start[0],
                    reason=reason,
                    family_index=loser_intent.family_index,
                    family_name=loser_intent.family_name,
                    k=None if loser_intent.web_id is None else loser_intent.web_id[1],
                    web_id=loser_intent.web_id,
                    merged_web_ids=loser_intent.merged_web_ids,
                )
            )
            del surviving[loser_i]

        if not omitted:
            return list(intents), []
        return surviving, omitted

    def _generate_scar(
        self,
        frame: LayerFrame,
        spec: StructureSpec,
        outer_wall_line_width_mm: float,
        effective_skin_margin_mm: float,
    ) -> WebIntent:
        """One scar ``WebIntent`` bridging the declared interior hole to the
        exterior boundary. Called exactly once per layer, only when
        ``spec.expected_hole_count > 0``.

        Declared-vs-actual check, first: ``ops.hole_bounds(frame.polygon)`` is
        the geometry's own interior-hole count, independent of anything the
        caller believes. A mismatch against ``spec.expected_hole_count`` is
        itself the defect and fails closed naming both numbers. The count is
        capped at one (``StructureSpec.expected_hole_count``), so a match means
        exactly one hole.

        Chord station: the hole's own bounding-box X centre, not
        ``web_position`` or any family's anchor; a scar has no position to
        configure, it goes wherever the hole is. A straight spar tube sits at
        roughly the same relative chord position at every Z, so the scar
        produces a CONTINUOUS SEAM up the span in the ordinary case: a line of
        two thin, fused-but-not-solid walls standing in for intact skin, the
        same class of weakness as a merged node, running the whole span.
        Varying the scar's angular position by Z (a zigzag around the hole's
        circumference) would spread that weakness out, at the cost of
        abandoning the vertical-only entry constraint. That is not
        implemented.

        Width: ``spec.resolve_slot_gap`` (the spec's top-level gap policy, not
        any one family's ``resolve_gap``). The scar is not a member of any
        family, so it reuses the same measured gap physics through the
        spec-level resolver, the same gap policy as every other slot.

        Direction: vertical only, toward whichever LOCAL exterior surface is
        closer to the hole at its station, measured worst-case across the
        scar's resolved width (``local_upper_worst_case``/
        ``local_lower_worst_case`` with ``hole_aware=True`` so a hole-crossing
        probe resolves to the correct piece instead of raising). A tie (equal
        distance to both surfaces) breaks toward +Y, a fixed arbitrary choice.

        Wall check: the worst-case LOCAL exterior surface minus the hole's near
        boundary must clear ``effective_skin_margin_mm`` (the layer's
        rotation-corrected margin, the same threshold a full-depth web's
        skin-margin omission check uses). ``StructureSpec.validate`` requires
        ``skin_margin_mm > 2 * safety_offset_mm``, so this threshold already
        exceeds ``min_wall_mm`` and satisfying it also satisfies the weaker
        ``min_wall_mm`` bound. Unlike an ordinary web there is NO OMISSION PATH
        for this failure: a declared hole is a mesh feature, not a candidate
        the generator chose to place, so failing the check fails the whole
        layer closed with a ``ValueError`` naming the layer's numbers
        (``capability.py``'s ``except (ValueError, ArithmeticError)`` handler
        wraps it into a ``TopologyError`` with layer index and Z, like every
        other generation-time geometric impossibility).

        Far end: reaches PAST the hole's near boundary by
        ``spec.overshoot_mm``, the same constant and mechanism ``SlotBuilder``
        applies at a slot's exterior end, doubled to the hole side. Setting
        ``WebIntent.end`` past the boundary (rather than exactly on it) means
        ``SlotBuilder.build`` needs no special case: its "no extension at the
        far end" behaviour lands past the hole's near boundary because this
        method already placed ``end`` there. The end is guarded strictly short
        of the hole's FAR boundary (the scar must not become a second,
        unrelated full cut): the hole's bbox height must comfortably exceed
        ``overshoot_mm`` before ``end`` is computed, and the placed ``end`` is
        re-checked against the far boundary directly as a second, independent
        guard, the same discipline as ``_generate_family``'s halo clamp.

        Width-vs-hole guard: the resolved scar width must be comfortably
        narrower than the hole's bbox width. A scar as wide as the hole could
        graze or straddle both of the hole's side walls instead of passing
        cleanly through its centre, a case the no-sever argument does not
        cover. The method fails closed rather than build a scar that has not
        been shown safe.
        """
        holes = frame.ops.hole_bounds(frame.polygon)
        if len(holes) != spec.expected_hole_count:
            raise ValueError(
                "declared hole count mismatch: StructureSpec.expected_hole_"
                "count={!r} but the section's own geometry has {} interior "
                "hole(s). A mismatch between what is declared and what is "
                "actually present is itself the defect.".format(
                spec.expected_hole_count, len(holes))
            )
        hole_min_x, hole_min_y, hole_max_x, hole_max_y = holes[0]
        hole_width_mm = hole_max_x - hole_min_x
        hole_height_mm = hole_max_y - hole_min_y
        x = (hole_min_x + hole_max_x) / 2.0

        width_mm = spec.resolve_slot_gap(outer_wall_line_width_mm)
        half_w = width_mm / 2.0
        footprint_min_x = x - half_w
        footprint_max_x = x + half_w
        if (
            footprint_min_x < frame.min_x - _FOOTPRINT_EPSILON_MM
            or footprint_max_x > frame.max_x + _FOOTPRINT_EPSILON_MM
        ):
            raise ValueError(
                "declared hole at x=[{:.4f}, {:.4f}] mm: the scar's own "
                "footprint [{:.4f}, {:.4f}] mm (width {:.4f} mm) does not "
                "lie within the section's chord extent "
                "[{:.4f}, {:.4f}] mm.".format(
                    hole_min_x,
                    hole_max_x,
                    footprint_min_x,
                    footprint_max_x,
                    width_mm,
                    frame.min_x,
                    frame.max_x,
                )
            )
        if width_mm >= hole_width_mm - _FOOTPRINT_EPSILON_MM:
            raise ValueError(
                "declared hole at x=[{:.4f}, {:.4f}] mm (width {:.4f} mm): "
                "the scar's own resolved width {:.4f} mm is not "
                "comfortably narrower than the hole -- section 3c's "
                "no-hole proof requires the scar to pass cleanly through "
                "the hole's own centre, not graze or straddle its side "
                "walls.".format(
                    hole_min_x, hole_max_x, hole_width_mm, width_mm
                )
            )
        if hole_height_mm <= spec.overshoot_mm + _DEPTH_EPSILON_MM:
            raise ValueError(
                "declared hole at y=[{:.4f}, {:.4f}] mm (height {:.4f} "
                "mm): not comfortably taller than overshoot_mm={:.4f} mm "
                "-- the scar's far end cannot reach past the hole's own "
                "near boundary without also reaching (or passing) its "
                "far boundary (section 2c).".format(
                    hole_min_y, hole_max_y, hole_height_mm, spec.overshoot_mm
                )
            )

        upper_y = local_upper_worst_case(
            frame.ops, frame.polygon, x, width_mm, DEFAULT_PROBE_WIDTH_MM, hole_aware=True
        )
        lower_y = local_lower_worst_case(
            frame.ops, frame.polygon, x, width_mm, DEFAULT_PROBE_WIDTH_MM, hole_aware=True
        )
        dist_upper = upper_y - hole_max_y
        dist_lower = hole_min_y - lower_y
        # Tie -> +Y (see this method's docstring).
        entry_lower = dist_lower < dist_upper
        sign = -1.0 if entry_lower else 1.0

        entry_y = lower_y if entry_lower else upper_y
        hole_near_y = hole_min_y if entry_lower else hole_max_y
        hole_far_y = hole_max_y if entry_lower else hole_min_y

        remaining_wall_mm = sign * (entry_y - hole_near_y)
        if remaining_wall_mm < effective_skin_margin_mm - _DEPTH_EPSILON_MM:
            raise ValueError(
                "declared hole at x={:.4f} mm: the wall between the "
                "hole's near boundary (y={:.4f} mm) and the worst-case "
                "local {} exterior surface (y={:.4f} mm) is only "
                "{:.4f} mm, below the required {:.4f} mm skin_margin_mm "
                "(after this layer's own rotation correction). No "
                "omission path exists for a declared hole (section 2d) "
                "-- the section must be thickened, or the spar moved, "
                "not silently skipped.".format(
                    x,
                    hole_near_y,
                    "lower" if entry_lower else "upper",
                    entry_y,
                    remaining_wall_mm,
                    effective_skin_margin_mm,
                )
            )

        end_y = hole_near_y - sign * spec.overshoot_mm
        if sign * (hole_far_y - end_y) >= -_DEPTH_EPSILON_MM:
            # Belt-and-suspenders: the hole_height_mm guard above should
            # already make this unreachable, but this is checked directly
            # against the actual placed end, the same "never trust a
            # precondition alone" discipline _generate_family's own halo clamp
            # uses.
            raise ValueError(
                "declared hole at x={:.4f} mm: the scar's far end "
                "(y={:.4f} mm) would reach or pass the hole's own FAR "
                "boundary (y={:.4f} mm) -- this would stop being a "
                "hole-connector and become a second, unrelated full cut "
                "through the far wall (section 2c). Refusing to build "
                "it.".format(x, end_y, hole_far_y)
            )

        return WebIntent(
            start=(x, entry_y),
            end=(x, end_y),
            width_mm=width_mm,
            family_index=-1,
            family_name="spar_scar",
            entry_lower=entry_lower,
            is_scar=True,
            web_id=(-1, 0, WEB_SIDE_SOCKET),
        )

    def _generate_family(
        self,
        frame: LayerFrame,
        spec: StructureSpec,
        family: WebFamily,
        family_index: int,
        outer_wall_line_width_mm: float,
        layer: LayerContext,
        safety_offset_mm: float,
        effective_skin_margin_mm: float,
        socket: Optional[WebIntent] = None,
    ) -> Tuple[List[WebIntent], List[OmittedWeb]]:
        """One family's webs (and omissions), in ascending chord order.

        ``socket`` (optional) is THIS layer's already-resolved carbon-spar
        socket intent (``StructureGenerator.generate``'s ``socket`` local,
        computed before the family loop precisely so it can be handed in
        here), or ``None`` if no socket was detected/declared this layer.
        Consulted only when ``family.depth_to_socket`` (read defensively via
        ``getattr(family, "depth_to_socket", False)``) is set.

        ``effective_skin_margin_mm`` is ``spec.skin_margin_mm`` already
        corrected for this layer's own section rotation (``generate()``'s
        one-per-layer call to ``frame.corrected_skin_margin_mm`` — see that
        function's docstring). Every margin comparison below uses this value,
        not ``spec.skin_margin_mm`` directly, so a rotated section's margin
        checks use the TRUE (chord-perpendicular) skin required rather than
        the optimistic world-Y reading. Diagnostic MESSAGES still cite
        ``spec.skin_margin_mm`` (the configured request) for readability; only
        the comparisons use the corrected value. On an unrotated section (or
        one within the rotation deadzone) the two are identical — see
        ``frame._ROTATION_DEADZONE_DEG``.

        ``family.entry_lower`` MIRRORS every formula below through a single
        ``sign`` variable (``+1`` for +Y entry, ``-1`` for -Y):
        ``local_entry_y`` is the worst-case LOCAL surface the family's own
        entry side must touch across the slot's width
        (``local_upper_worst_case`` for +Y; ``local_lower_worst_case`` for -Y —
        the SAME primitive invariant 3's mirrored check in ``validate.py``
        uses to confirm a -Y cut reaches its own boundary), and
        ``local_far_y`` is the worst-case LOCAL surface the far end must clear
        ``skin_margin_mm`` before (swapped the same way). Every comparison and
        placement formula below is written generically in ``sign`` — see each
        step's comment for the algebra — and reduces exactly to the +Y-only
        formulas when ``sign == 1`` (``family.entry_lower`` is ``False``, the
        default).

        RELATIVE PLACEMENT: both the anchor's Z-drift and the marching pitch
        are resolved against THIS layer's own ``frame.width`` (local chord)
        when the family's relative fields (``z_drift_chord_frac_per_mm`` /
        ``rib_pitch_chord_frac``) are set — see
        ``WebFamily.station_drift_mm``/``resolved_rib_pitch_mm`` for the
        formulas and the class docstring's field block (after ``role``) for the
        constant-angle-vs-constant-chord-proportion tradeoff. Absolute-mode
        families resolve against absolute millimetres instead.
        """
        # Resolved BEFORE measuring any station: every candidate's footprint
        # and local-surface measurement depends on the actual resolved slot
        # width, not an arbitrary probe width — see the module docstring and
        # frame.web_centerline's docstring.
        width_mm = family.resolve_gap(spec, outer_wall_line_width_mm)

        # Socket-relative placement. The formula follows ``target_frac`` in
        # tools/m25_socket_flank_search.py. The fields are read defensively via
        # ``getattr`` (same shape as ``depth_to_socket``/``opposing_pair``
        # above).
        #
        # ``socket_flank_offset_mm`` unset OR 0 leaves this family's usual
        # station logic (``web_position`` + drift, below) in sole control. A
        # positive value OVERRIDES the anchor entirely: this family's station
        # is computed from THIS LAYER's own detected socket channel
        # (``layer.detected_channels``), never from ``web_position``,
        # ``rib_pitch_chord_frac``, or Z-drift. The model is the source of
        # truth for placement too: a frozen chord fraction drifts out of the
        # 0.021 mm fusion window as the taper moves the detected socket, so the
        # station must be re-measured from the model on every layer, not
        # carried as config.
        socket_flank_offset_mm = getattr(family, "socket_flank_offset_mm", None)
        socket_flank_active = (
            socket_flank_offset_mm is not None and socket_flank_offset_mm > 0.0
        )

        if socket_flank_active:
            # Exactly 0 or 1 channels ever reaches this point -- more than
            # one is either a fatal ``TopologyError`` raised earlier in
            # ``generate()`` (``spar_channel_policy="refuse"``, the
            # default) or has already been narrowed to exactly the ONE
            # channel chosen as the spar (``spar_channel_policy=
            # "nearest_spar_station"`` -- see ``generate()``'s own
            # docstring, "SPAR CHANNEL SELECTION"), before the family loop
            # runs either way.
            if not layer.detected_channels:
                # ``x_mm`` is REQUIRED (non-Optional) on ``OmittedWeb``, but
                # there is no computed station here -- the whole point of
                # this branch is that nothing was measured. Filling it with
                # ``frame.web_x(family.web_position)`` would be wrong: the
                # reason string in the SAME record says that value is never
                # read for this family, and a diagnostics reader could mistake
                # it for the real (unbuilt) station. ``nan`` is the honest
                # value: it formats fine (``"{:.4f}".format(float("nan"))`` ->
                # ``"nan"``) and cannot be mistaken for a real coordinate.
                return [], [
                    OmittedWeb(
                        x_mm=float("nan"),
                        reason=(
                            "socket_flank: family {} ({!r}) requests "
                            "placement socket_flank_offset_mm={!r} past the "
                            "DETECTED spar socket's own circle edge, but no "
                            "socket channel was detected in the model at "
                            "this layer -- there is nothing to offset "
                            "from. This never falls back to web_position "
                            "or any chord fraction: a frozen "
                            "fraction cannot hold station inside the "
                            "fusion window on a tapered wing.".format(
                                family_index, family.name,
                                socket_flank_offset_mm,
                            )
                        ),
                        family_index=family_index,
                        family_name=family.name,
                        k=0,
                        web_id=(family_index, 0, web_side(family.entry_lower)),
                    )
                ]
            # A channel being present is not enough -- it must be the
            # CONFIRMED spar (``socket.spar_confirmed``, see that field's own
            # docstring), never one merely disambiguated among several
            # candidates by ``spar_channel_policy="nearest_spar_station"``.
            # ``socket`` is this layer's already-resolved socket intent (the
            # same one ``generate()`` passed in), so this reads the SAME
            # resolution this family's anchor is about to depend on -- not a
            # second, potentially inconsistent read of
            # ``layer.detected_channels``. Omitted the same way as "no channel
            # detected": there is nothing this family may honestly anchor to,
            # whether because no channel exists or because the one that exists
            # was only a guess (on a rod-bore wing an alignment bore can win
            # the "nearest to spar_station" contest at a Z where the real spar
            # has already ended).
            if socket is None or not getattr(socket, "spar_confirmed", False):
                return [], [
                    OmittedWeb(
                        x_mm=float("nan"),
                        reason=(
                            "socket_flank: family {} ({!r}) requests "
                            "placement socket_flank_offset_mm={!r} past the "
                            "DETECTED spar socket's own circle edge, but the "
                            "socket resolved at this layer is not a "
                            "CONFIRMED spar (spar_confirmed=False -- it was "
                            "chosen by disambiguating among two or more "
                            "detected channels, never established as the "
                            "real spar rather than a bore). Refusing to "
                            "anchor structure to a guess; this never falls "
                            "back to web_position or any chord fraction "
                            "instead.".format(
                                family_index, family.name,
                                socket_flank_offset_mm,
                            )
                        ),
                        family_index=family_index,
                        family_name=family.name,
                        k=0,
                        web_id=(family_index, 0, web_side(family.entry_lower)),
                    )
                ]
            channel = layer.detected_channels[0]
            socket_flank_direction = getattr(
                family, "socket_flank_direction", None
            )
            # "forward" (toward the leading edge, -X) / "aft" (toward the
            # trailing edge, +X) -- same sign convention as the ``DIRECTIONS``
            # table in ``tools/m25_socket_flank_search.py``.
            # ``WebFamily.validate()`` refuses any other value whenever the
            # offset is active through the public API, but this method is also
            # called directly (bypassing ``validate()``) by some tests -- see
            # ``test_m19_depth_to_socket.py``. Treating any non-"forward" value
            # (including ``None``) as "aft" would be a fail-OPEN default for
            # exactly the kind of caller that bypasses validate(), so this
            # raises instead, mirroring the relative-pitch floor raise a few
            # lines below.
            if socket_flank_direction not in ("forward", "aft"):
                raise ValueError(
                    "family {} ({!r}): socket_flank_offset_mm={!r} is "
                    "active (> 0) but socket_flank_direction={!r} is not "
                    "'forward' or 'aft' -- WebFamily.validate() should have "
                    "caught this before generate() is ever reached; refusing "
                    "to guess a direction here.".format(
                        family_index, family.name, socket_flank_offset_mm,
                        socket_flank_direction,
                    )
                )
            direction_sign = -1.0 if socket_flank_direction == "forward" else 1.0
            anchor_x = channel.center_x_mm + direction_sign * (
                channel.diameter_mm / 2.0 + socket_flank_offset_mm
            )
        else:
            # The whole family's anchor shifts by this layer's drift BEFORE
            # marching by rib_pitch_mm, so every web in the family moves
            # together — see the module docstring and
            # WebFamily.station_drift_mm's docstring for the formula. Zero
            # when Z-variation is unset.
            #
            # ``local_chord_mm=frame.width`` is THIS layer's own local chord,
            # read on every call. It has no effect unless
            # ``family.z_drift_chord_frac_per_mm`` is set (see
            # ``WebFamily.station_drift_mm``'s "RELATIVE MODE" note).
            anchor_x = frame.web_x(family.web_position) + family.station_drift_mm(
                layer.print_z, local_chord_mm=frame.width
            )

        # +1 for a +Y-entry family, -1 for -Y — see this method's docstring.
        sign = -1.0 if family.entry_lower else 1.0

        # Read defensively via ``getattr`` (see this method's docstring).
        depth_to_socket = getattr(family, "depth_to_socket", False)

        # ``rib_pitch_chord_frac`` resolves against THIS layer's own local
        # chord (``frame.width``) via ``WebFamily.resolved_rib_pitch_mm``, and
        # equals ``family.rib_pitch_mm`` when the relative field is unset.
        #
        # MIN-FEATURE FLOOR (relative pitch only; an absolute pitch is not
        # checked here): a fraction of a shrinking chord can resolve to a pitch
        # smaller than the slot itself, which would place a station's own
        # neighbour inside its own footprint -- a pattern-level infeasibility,
        # not a single station's thinness. Rather than let this surface late
        # as an opaque same-family collision inside TopologyValidator, several
        # call frames away from the actual cause, it fails closed HERE, naming
        # the family, this layer's local chord, the requested fraction, the
        # resolved pitch, and the floor. The floor is ``width_mm +
        # min_wall_mm`` -- the same ``min_wall_mm = 2 * safety_offset_mm``
        # TopologyValidator uses as the minimum edge-to-edge gap between two
        # same-family slots: at exactly this pitch, two adjacent same-family
        # stations' footprints are ``width_mm`` apart edge-to-edge, i.e. the
        # smallest gap the validator does not already reject as a collision. A
        # bare ``width_mm`` floor (adjacent footprints merely not overlapping)
        # would leave a band where THIS check passes and the validator fails
        # the whole layer moments later with a message that does not name the
        # real cause (the collapsed relative pitch); two checks of one quantity
        # must agree.
        #
        # NOT a clamp, NOT a per-station omission: clamping would silently
        # substitute a DIFFERENT pitch than the one requested, and this is a
        # property of the WHOLE family's march at this layer, not one candidate
        # station's own local thinness -- there is no single ``OmittedWeb``
        # this maps onto. An opt-in "clamp to the floor and record it loudly"
        # mode (in the style of ``tip_shutdown_enabled``) would be a possible
        # extension; it is not implemented.
        pitch_mm = family.resolved_rib_pitch_mm(frame.width)
        if family.rib_pitch_chord_frac is not None and pitch_mm is not None:
            min_wall_mm = 2.0 * safety_offset_mm
            pitch_floor_mm = width_mm + min_wall_mm
            if pitch_mm < pitch_floor_mm - _FOOTPRINT_EPSILON_MM:
                raise _SectionTooSmallForPitch(
                    "family {} ({!r}): rib_pitch_chord_frac={!r} resolves "
                    "to {:.4f} mm at this layer's local chord "
                    "{:.4f} mm (frame.width), below the minimum-feature "
                    "floor {:.4f} mm (resolved slot width {:.4f} mm plus "
                    "the minimum wall 2 * safety_offset_mm={:.4f} mm) -- "
                    "the relative pitch has collapsed too far for this "
                    "family's own slot width to fit twice with daylight "
                    "between the slots. Raise rib_pitch_chord_frac, reduce "
                    "the family's slot width, or switch to absolute "
                    "rib_pitch_mm for this span.".format(
                        family_index,
                        family.name,
                        family.rib_pitch_chord_frac,
                        pitch_mm,
                        frame.width,
                        pitch_floor_mm,
                        width_mm,
                        min_wall_mm,
                    )
                )

        intents: List[WebIntent] = []
        omitted: List[OmittedWeb] = []
        # Web identity: ``(family_index, k, side)`` per station. Omissions a
        # station iteration appends are stamped with its identity at the top of
        # the NEXT iteration (and after the loop) so none of the many
        # ``continue`` exits has to carry it.
        web_side_letter = web_side(family.entry_lower)
        depth_caps = getattr(layer, "depth_caps", None)
        per_station_reentrant = _per_station_reentrant(spec)
        _stamp_from = 0
        _stamp_id = None
        for station_k, x in self._candidate_stations_k(frame, pitch_mm, anchor_x):
            _stamp_omitted(omitted, _stamp_from, _stamp_id)
            _stamp_from = len(omitted)
            _stamp_id = (family_index, station_k, web_side_letter)
            depth_cap = None if not depth_caps else depth_caps.get(_stamp_id)
            if depth_to_socket and (
                socket is None or not getattr(socket, "spar_confirmed", False)
            ):
                # A socket-referenced family with no socket to reference this
                # layer. It never silently falls back to ``web_depth_mm`` (or
                # any other fixed geometry); silent substitution would hide the
                # missing socket. It is omitted instead, loudly, per station,
                # the same way every other unplaceable candidate in this
                # method is.
                #
                # The same applies when a socket exists but is not CONFIRMED
                # the real spar (``WebIntent.spar_confirmed`` -- see its
                # docstring). A socket chosen by ``spar_channel_policy=
                # "nearest_spar_station"`` disambiguating among several
                # candidates is good enough to have been CUT (pure topology),
                # never good enough to size a member's depth against, for the
                # identical reason the socket-flank anchor above refuses it.
                if socket is None:
                    reason = (
                        "depth_to_socket: family {} ({!r}) requests a "
                        "depth derived from the detected carbon-spar "
                        "socket, but no socket was detected in the "
                        "model or declared in config at this layer -- "
                        "there is nothing to measure this member's "
                        "depth against.".format(family_index, family.name)
                    )
                else:
                    reason = (
                        "depth_to_socket: family {} ({!r}) requests a "
                        "depth derived from the detected carbon-spar "
                        "socket, but the socket resolved at this layer is "
                        "not a CONFIRMED spar (spar_confirmed=False -- it "
                        "was chosen by disambiguating among two or more "
                        "detected channels, never established as the real "
                        "spar rather than a bore) -- refusing to size a "
                        "member against a guess.".format(
                            family_index, family.name)
                    )
                omitted.append(
                    OmittedWeb(
                        x_mm=x,
                        reason=reason,
                        family_index=family_index,
                        family_name=family.name,
                    )
                )
                continue
            half_w = width_mm / 2.0
            footprint_min_x = x - half_w
            footprint_max_x = x + half_w
            if (
                footprint_min_x < frame.min_x - _FOOTPRINT_EPSILON_MM
                or footprint_max_x > frame.max_x + _FOOTPRINT_EPSILON_MM
            ):
                omitted.append(
                    OmittedWeb(
                        x_mm=x,
                        reason=(
                            "chord extent: slot footprint [{:.4f}, {:.4f}] mm "
                            "does not lie within the section's chord extent "
                            "[{:.4f}, {:.4f}] mm".format(
                                footprint_min_x,
                                footprint_max_x,
                                frame.min_x,
                                frame.max_x,
                            )
                        ),
                        family_index=family_index,
                        family_name=family.name,
                    )
                )
                continue

            if spec.expected_hole_count > 0:
                # An ORDINARY (non-scar) family candidate whose halo footprint
                # overlaps the declared hole's own X footprint cannot be
                # measured by this method's usual (hole-BLIND)
                # local_upper_worst_case/local_lower_worst_case calls below --
                # those raise the re-entrant ValueError at a hole-crossing
                # station (frame.local_span's own guard, hole_side=None here
                # since this is not the scar). Rather than make every
                # measurement in this method hole-aware (which would risk
                # silently tolerating an UNDECLARED hole anywhere a family
                # happens to probe -- the exact hazard
                # StructureSpec.expected_hole_count's docstring exists to
                # prevent), a station whose (haloed) footprint overlaps the
                # hole is excluded up front, the same "positional miss, not a
                # thinness finding" category as the chord-extent check just
                # above. Haloed by safety_offset_mm on both sides (not the bare
                # footprint) because the thin-section check further below
                # probes an equally widened span, and must never reach the hole
                # either.
                halo_min_x = footprint_min_x - safety_offset_mm
                halo_max_x = footprint_max_x + safety_offset_mm
                hole_conflict = None
                for hole_min_x, hole_min_y, hole_max_x, hole_max_y in (
                    frame.ops.hole_bounds(frame.polygon)
                ):
                    if halo_max_x > hole_min_x and halo_min_x < hole_max_x:
                        hole_conflict = (hole_min_x, hole_max_x)
                        break
                if hole_conflict is not None:
                    hc_min_x, hc_max_x = hole_conflict
                    if not family.is_marching:
                        raise ValueError(
                            "family {} ({!r}): the requested station at "
                            "chord x={:.4f} mm (haloed footprint "
                            "[{:.4f}, {:.4f}] mm) overlaps the declared "
                            "interior hole's own X footprint "
                            "[{:.4f}, {:.4f}] mm -- an ordinary web cannot "
                            "be measured or placed there. This is the "
                            "user's one explicit request failing, not an "
                            "omittable marching candidate.".format(
                                family_index,
                                family.name,
                                x,
                                halo_min_x,
                                halo_max_x,
                                hc_min_x,
                                hc_max_x,
                            )
                        )
                    omitted.append(
                        OmittedWeb(
                            x_mm=x,
                            reason=(
                                "hole footprint: haloed footprint "
                                "[{:.4f}, {:.4f}] mm overlaps the declared "
                                "interior hole's own X footprint "
                                "[{:.4f}, {:.4f}] mm -- an ordinary web "
                                "cannot cross the hole (only the scar "
                                "does; see StructureSpec.expected_hole_"
                                "count).".format(
                                    halo_min_x, halo_max_x, hc_min_x, hc_max_x
                                )
                            ),
                            family_index=family_index,
                            family_name=family.name,
                        )
                    )
                    continue

            try:
                if family.entry_lower:
                    local_entry_y = local_lower_worst_case(
                        frame.ops, frame.polygon, x, width_mm, DEFAULT_PROBE_WIDTH_MM
                    )
                    local_far_y = local_upper_worst_case(
                        frame.ops, frame.polygon, x, width_mm, DEFAULT_PROBE_WIDTH_MM
                    )
                else:
                    local_entry_y = local_upper_worst_case(
                        frame.ops, frame.polygon, x, width_mm, DEFAULT_PROBE_WIDTH_MM
                    )
                    local_far_y = local_lower_worst_case(
                        frame.ops, frame.polygon, x, width_mm, DEFAULT_PROBE_WIDTH_MM
                    )
            except ReEntrantSectionError as exc:
                # With ``per_station_reentrant`` (spec flag, default OFF), this
                # ONE station cannot be measured because the section is
                # re-entrant around it. With the flag off the exception
                # propagates and the capability shuts the whole layer down.
                if not per_station_reentrant:
                    raise
                omitted.append(
                    OmittedWeb(
                        x_mm=x,
                        reason=(
                            "re_entrant_station: the section is re-entrant "
                            "at x={:.4f} mm (slot width {:.4f} mm, probe "
                            "{:.4f} mm), so this station's entry/far "
                            "surfaces are not measurable ({}). The station "
                            "is omitted; the layer keeps its other "
                            "structure.".format(
                                x, width_mm, DEFAULT_PROBE_WIDTH_MM, exc)
                        ),
                        family_index=family_index,
                        family_name=family.name,
                    )
                )
                continue
            # True local thickness, sign-generic: for +Y (sign=1) this is
            # ``local_entry_y - local_far_y`` (upper minus lower);
            # for -Y (sign=-1) it is ``local_far_y - local_entry_y`` (upper
            # minus lower) — both reduce to the same positive quantity via
            # multiplying by ``sign``.
            thickness_mm = sign * (local_entry_y - local_far_y)
            margin_y = local_far_y + sign * effective_skin_margin_mm
            if sign * (margin_y - local_entry_y) >= 0:
                omitted.append(
                    OmittedWeb(
                        x_mm=x,
                        reason=(
                            "skin margin: local thickness {:.4f} mm at "
                            "x={:.4f} mm cannot accommodate "
                            "skin_margin_mm={:.4f} mm".format(
                                thickness_mm, x, spec.skin_margin_mm
                            )
                        ),
                        family_index=family_index,
                        family_name=family.name,
                    )
                )
                continue

            if depth_to_socket:
                # SAME-SIDE ENTRY IS A SILENT NO-OP, so it is refused here. If
                # this family enters the same skin the socket's own neck
                # enters, the brace is placed INSIDE that neck: its slot is a
                # geometric subset of the neck's slot, so after the boolean
                # there is no member at all. Nothing downstream catches it --
                # the validator sees no collision because there is no material
                # to collide over, and the footprint exclusion is deliberately
                # exempted for socket-referenced members. It does not bite the
                # reference wing only because socket always enters the upper
                # skin, which is an accident of that model rather than a
                # guarantee.
                if bool(family.entry_lower) == bool(
                        getattr(socket, "entry_lower", False)):
                    omitted.append(OmittedWeb(
                        x_mm=x,
                        reason=(
                            "depth_to_socket: family {} ({!r}) enters the "
                            "{} skin, the same side as the spar socket's own "
                            "neck, so the member would lie inside that neck "
                            "and vanish in the boolean. Enter from the "
                            "opposite skin.".format(
                                family_index, family.name,
                                "lower" if family.entry_lower else "upper")),
                        family_index=family_index,
                        family_name=family.name,
                    ))
                    continue
                # depth DERIVED from the detected socket -- never from
                # ``family.web_depth_mm`` (this branch is unconditional on the
                # flag, so a fixed ``web_depth_mm`` set alongside the flag is
                # ignored for this family: the model, not config, supplies the
                # geometry).
                #
                # ``socket`` is the SAME resolved intent
                # ``StructureGenerator.generate`` will place and exclude
                # against below -- not a re-derivation, so this member and
                # the exclusion band always agree about where the socket is.
                cy = socket.end[1]
                r = socket.socket_radius_mm
                # The circle boundary nearest the entry skin, in the
                # entry -> far direction ``sign`` already encodes (see this
                # method's docstring's ``sign`` paragraph): a +Y entry
                # (sign=+1) reaches the circle's TOP first (``cy + r``); a
                # -Y entry (sign=-1) reaches its BOTTOM first (``cy - r``)
                # -- both captured by the one expression below.
                circle_near_y = cy + sign * r
                # Target far end: ``skin_margin_mm`` back toward the entry
                # side from that boundary, so the member's tip stops SHORT
                # of the circle rather than reaching or crossing it -- the
                # same "move margin_mm toward entry from a boundary" shape
                # ``margin_y`` above uses against the far exterior skin,
                # with the circle's near edge standing in for that skin.
                target_end_y = circle_near_y + sign * effective_skin_margin_mm
                computed_depth_mm = sign * (local_entry_y - target_end_y)
                if computed_depth_mm <= 0.0:
                    gap_mm = sign * (local_entry_y - circle_near_y)
                    omitted.append(
                        OmittedWeb(
                            x_mm=x,
                            reason=(
                                "depth_to_socket: family {} ({!r}) at chord "
                                "station x={:.4f} mm has only {:.4f} mm of "
                                "gap between its entry skin and the "
                                "detected socket's circle, which does not "
                                "leave room for the required {:.4f} mm "
                                "skin_margin_mm -- there is no depth at "
                                "which this member both reaches the "
                                "socket's fusion zone and clears "
                                "skin_margin_mm.".format(
                                    family_index,
                                    family.name,
                                    x,
                                    gap_mm,
                                    spec.skin_margin_mm,
                                )
                            ),
                            family_index=family_index,
                            family_name=family.name,
                        )
                    )
                    continue
                end_y = local_entry_y - sign * computed_depth_mm
                end_y = _cap_end_y(local_entry_y, end_y, sign, depth_cap)
            elif family.web_depth_mm is None:
                # FULL-DEPTH web: place the far end with the derived
                # headroom above skin_margin_mm — see
                # FULL_DEPTH_MARGIN_HEADROOM_MM's comment for why a
                # bare margin_y (skin_margin_mm exactly, no headroom) can
                # legitimately fail validate.py's independent
                # re-measurement even on a genuinely thick section. This
                # is checked here, AFTER the plain "is there any room at
                # all" omission above, because a station can have room for
                # skin_margin_mm but not for skin_margin_mm plus this
                # headroom — that station must still be omitted (as a
                # "skin margin" reason, same category as the check above),
                # not built with end_y >= y_max, which would flip
                # SlotBuilder's direction vector and build the slot
                # upside down.
                end_y = margin_y + sign * FULL_DEPTH_MARGIN_HEADROOM_MM
                end_y = _cap_end_y(local_entry_y, end_y, sign, depth_cap)
                if sign * (end_y - local_entry_y) >= 0:
                    omitted.append(
                        OmittedWeb(
                            x_mm=x,
                            reason=(
                                "skin margin: local thickness {:.4f} mm at "
                                "x={:.4f} mm cannot accommodate "
                                "skin_margin_mm={:.4f} mm plus the "
                                "{:.4f} mm full-depth placement headroom "
                                "(FULL_DEPTH_MARGIN_HEADROOM_MM)".format(
                                    thickness_mm,
                                    x,
                                    spec.skin_margin_mm,
                                    FULL_DEPTH_MARGIN_HEADROOM_MM,
                                )
                            ),
                            family_index=family_index,
                            family_name=family.name,
                        )
                    )
                    continue
            else:
                # Depth is measured along the centerline from the LOCAL
                # entry surface (the same worst-case measurement used for the
                # margin above), not the section's bounding box, so a shallow
                # web on a curved section follows the surface it entered from.
                # A breach here (the family's requested depth would eat into
                # skin_margin_mm) is NEVER resolved by deepening the web toward
                # the far surface, which would risk a through-cut; neither
                # branch below does that.
                #
                # What differs is whether an unplaceable station stays loud
                # (raise) or is folded into the omission machinery, and that
                # split follows ``family.is_marching`` (the single predicate
                # for this boundary -- see its docstring for why testing
                # ``rib_pitch_mm`` directly misclassifies a chord-relative
                # family), the same "one explicit station" vs. "marched
                # sequence" boundary ``_candidate_stations`` uses:
                #
                # - An EXPLICIT single-station family (``not family.
                #   is_marching``) named this exact chord position; the
                #   breach is the user's one request failing, and is a
                #   ``ValueError`` naming the family and station.
                # - A MARCHING family (``family.is_marching``) generates its
                #   own stations arithmetically out to the section's chord
                #   extent and cannot itself avoid a locally-thin station.
                #   The identical physical condition (not enough local
                #   thickness for this specific request) is already an
                #   ``OmittedWeb`` for a full-depth request at the same
                #   station (the "skin margin" checks above); a depth-limited
                #   request hitting the same thinness gets the same treatment
                #   instead of a fatal error for the whole layer. The NACA 2412
                #   c60 fixture has an 85.15-94.92% chord thin band that no
                #   marched shallow pitch can dodge, so a hard fail would make
                #   every marching shallow family unusable on a tapered
                #   section.
                end_y = local_entry_y - sign * family.web_depth_mm
                end_y = _cap_end_y(local_entry_y, end_y, sign, depth_cap)
                if sign * (end_y - margin_y) < -_DEPTH_EPSILON_MM:
                    remaining_skin_mm = sign * (end_y - local_far_y)
                    available_depth_mm = sign * (local_entry_y - margin_y)
                    if not family.is_marching:
                        raise ValueError(
                            "family {} ({!r}): requested web_depth_mm={:.4f} "
                            "at chord station x={:.4f} mm would leave only "
                            "{:.4f} mm of skin at its far end, below the "
                            "required {:.4f} mm skin_margin_mm (local "
                            "thickness {:.4f} mm allows at most {:.4f} mm of "
                            "depth here) — refusing to silently deepen a "
                            "depth-limited web toward the far surface.".format(
                                family_index,
                                family.name,
                                family.web_depth_mm,
                                x,
                                remaining_skin_mm,
                                spec.skin_margin_mm,
                                thickness_mm,
                                available_depth_mm,
                            )
                        )
                    omitted.append(
                        OmittedWeb(
                            x_mm=x,
                            reason=(
                                "skin margin: local thickness {:.4f} mm at "
                                "x={:.4f} mm cannot accommodate the "
                                "requested web_depth_mm={:.4f} mm plus "
                                "skin_margin_mm={:.4f} mm (at most {:.4f} mm "
                                "of depth available here) — a marched "
                                "station this thin is omitted, never "
                                "silently deepened toward the far "
                                "surface.".format(
                                    thickness_mm,
                                    x,
                                    family.web_depth_mm,
                                    spec.skin_margin_mm,
                                    available_depth_mm,
                                )
                            ),
                            family_index=family_index,
                            family_name=family.name,
                        )
                    )
                    continue

            # Thin-section omission — see the module docstring's "Thin-section
            # omission" section for the full derivation. Scope-gated: below
            # this threshold even an infinitely thick flat section cannot pass,
            # so that regime violates TopologyValidator's precondition and must
            # stay fail-closed through the validator's own erosion check.
            #
            # The gate (whether this check runs at all) stays against the RAW
            # ``spec.skin_margin_mm``: it is a config-level sanity
            # precondition, the same at every layer, not a per-layer physical
            # reading. The ``required_halo_mm`` THRESHOLD the check compares
            # against IS corrected, via ``corrected_skin_margin_mm`` (the same
            # function ``generate()`` applies once per layer to the margin
            # check) fed the LOCAL far-surface slope
            # (``estimate_far_surface_slope_deg``, measured across this
            # candidate's own ordinary halo window), not the section's global
            # ``frame.rotation_deg``. ``remaining_halo_mm`` is a world-Y extent
            # read off ``frame``, the same class of quantity the margin check
            # corrects. The erosion itself is isotropic in true mm; it is only
            # this check's vertical-Y proxy that overstates true
            # chord-perpendicular thickness by 1/cos(theta) wherever the far
            # surface is sloped across the probed window. That slope can come
            # from the section's global rotation (a rotated flat far surface
            # has a world slope that reproduces ``frame.rotation_deg`` almost
            # exactly, so the local measurement subsumes that case) or from the
            # section's intrinsic curvature approaching a chord extremity, which
            # near a trailing edge is a genuinely LARGER quantity than the
            # global rotation. Falls back to ``frame.rotation_deg`` when the
            # local slope cannot be measured (off-section or re-entrant edge
            # probe). ``end_y`` (used just below, in ``remaining_halo_mm =
            # end_y - y_min_halo``) is itself derived from the corrected
            # ``margin_y``, so a rotated section's halo reading inherits the
            # margin correction's added conservatism even though the gate
            # condition does not.
            #
            # Near-extremity widening: when this check's ordinary window
            # already comes within one candidate-depth of a chord extremity,
            # that side is extended the REST of the way to the extremity. A
            # real in-host slice failed at layer 12 (Z=2.60 mm) of
            # ``presets/plugin/skin_lattice_only_config.json`` on the NACA 2412
            # c60 fixture at a station this check passed by 0.0018 mm
            # (``remaining_halo_mm`` 0.8418 mm against an 0.84 mm threshold): a
            # single +Y candidate at x=-29.0005 mm, ~1.0 mm from the
            # leading-edge apex, cutting 1.5 mm deep. The ordinary window
            # (``width_mm + 2 * safety_offset_mm`` wide, centred on the
            # candidate) reached only to x=-29.431 mm. An offline reproduction
            # with a real polygon-boolean backend (mitre-join offset, matching
            # Clipper's default, rather than the round-join approximation) found
            # the layer genuinely disconnects, and the pinch sits at
            # x=[-29.581, -29.431] mm, entirely OUTSIDE the ordinary window, on
            # the leading-edge side. The mechanism: this check measures the far
            # surface's WORST reading only directly under the slot (plus its
            # own erosion halo); it implicitly assumes the far boundary's local
            # Y is roughly constant across that span. That assumption fails
            # near a chord extremity, where the boundary CONVERGES: material
            # trapped between the cut's own (fixed-Y) end cap and the
            # converging boundary can pinch thin at an X some distance away
            # from the candidate's station, not under it.
            #
            # Why "reach the extremity" and not "widen by depth everywhere":
            # adding ``2 * depth_used_mm`` to the span unconditionally sweeps
            # mid-chord and full-depth (multi-mm) candidates, far from any
            # extremity, into an unrelated dip elsewhere on the section, because
            # the window is sized off their own (sometimes large) depth rather
            # than off proximity to a converging boundary. The condition below
            # triggers ONLY when the ordinary window already sits within one
            # depth of the chord extremity, i.e. near a genuine convergence, and
            # then measures the WHOLE remaining span out to that extremity (an
            # exact bound on that side, not another approximation layered on
            # one). A mid-chord or full-depth candidate whose ordinary window
            # is many mm from either extremity is unaffected. There is no
            # effect on a flat-sided fixture (the 20 mm coupon) either way: the
            # far surface's worst reading is the same constant value at every X
            # there, so widening the probed span changes nothing.
            if spec.skin_margin_mm >= 2.0 * safety_offset_mm:
                depth_used_mm = abs(local_entry_y - end_y)
                halo_span_mm = width_mm + 2.0 * safety_offset_mm
                halo_min_x = x - halo_span_mm / 2.0
                halo_max_x = x + halo_span_mm / 2.0
                # Captured BEFORE the extremity-widening reassignment just
                # below, so the slope measurement used for the threshold
                # correction reads the ORDINARY (un-widened) window: the neck
                # this check approximates forms at the slot's own corner, not
                # out at a chord extremity the widening may have pulled this
                # window toward.
                ordinary_halo_min_x = halo_min_x
                ordinary_halo_max_x = halo_max_x
                if halo_min_x - frame.min_x < depth_used_mm:
                    halo_min_x = frame.min_x
                if frame.max_x - halo_max_x < depth_used_mm:
                    halo_max_x = frame.max_x
                # Clamp the halo probe to the section's own chord extent —
                # erosion cannot reach material that was never there, and a web
                # near the tip of the chord must not turn this approximation
                # into a NEW fail-closed path (local_span raises ValueError
                # off-section) when the primary, slot-width measurement above
                # already proved the station itself is on-section and
                # margin-valid. Pulled in by _FOOTPRINT_EPSILON_MM (not clamped
                # exactly to the boundary): local_lower_worst_case reconstructs
                # its own sample stations from (center, span) via ``center -
                # span / 2 + i * step``, which for a clamp landing EXACTLY on
                # frame.min_x/max_x can round a hair PAST it in floating point
                # and re-trigger the very ValueError this clamp exists to avoid
                # — verified: an unpadded clamp at x=-29.5 on the NACA 2412
                # fixture raises exactly that. The
                # try/except below is a second, independent safety net for the
                # same reason: this is a deliberate approximation (see the
                # module docstring), not a proof, and it must never itself
                # become a new fail-closed path.
                clamped_min_x = max(halo_min_x, frame.min_x + _FOOTPRINT_EPSILON_MM)
                clamped_max_x = min(halo_max_x, frame.max_x - _FOOTPRINT_EPSILON_MM)
                clamped_span_mm = clamped_max_x - clamped_min_x
                if clamped_span_mm > 0:
                    halo_center_x = (clamped_min_x + clamped_max_x) / 2.0
                    try:
                        # the FAR side's halo reading mirrors
                        # local_entry_y/local_far_y above — for a -Y-entry
                        # family the far side is the UPPER surface, so the halo
                        # probe is local_upper_worst_case, not
                        # local_lower_worst_case.
                        if family.entry_lower:
                            far_halo_y = local_upper_worst_case(
                                frame.ops,
                                frame.polygon,
                                halo_center_x,
                                clamped_span_mm,
                                DEFAULT_PROBE_WIDTH_MM,
                            )
                        else:
                            far_halo_y = local_lower_worst_case(
                                frame.ops,
                                frame.polygon,
                                halo_center_x,
                                clamped_span_mm,
                                DEFAULT_PROBE_WIDTH_MM,
                            )
                    except ValueError:
                        # Belt-and-suspenders: the clamp above is meant to
                        # prevent this, but this measurement is a
                        # best-effort approximation of the true erosion
                        # halo, not a proof — falling back to the already
                        # -valid slot-width reading keeps this check from
                        # ever becoming a NEW fail-closed path of its own.
                        far_halo_y = local_far_y
                else:
                    # Degenerate clamp (the halo collapsed to a point at the
                    # chord extent itself) — fall back to the already-valid
                    # slot-width reading rather than skip the check.
                    far_halo_y = local_far_y
                remaining_halo_mm = sign * (end_y - far_halo_y)
                # ``remaining_halo_mm`` is, like ``margin_y`` above, a world-Y
                # extent read straight off ``frame``: the same class of quantity
                # ``corrected_skin_margin_mm`` exists to correct. The erosion is
                # isotropic in true mm; it is this check's vertical-Y proxy for
                # "material remaining" that overstates true chord-perpendicular
                # thickness by 1/cos(theta) on a sloped boundary, as the margin
                # check does for a rotated section. The correction reuses the
                # function ``generate()`` calls once per layer for the margin
                # check.
                #
                # Which slope: local, not global. The check fails at a
                # genuinely twisted section (global rotation) AND at a
                # genuinely FLAT (zero-rotation) tapered section approaching
                # its trailing edge, a slow convergence intrinsic to the
                # airfoil that the near-extremity widening above does not reach
                # (a NACA trailing edge closes to zero thickness over a far
                # longer chordwise span than the leading edge's blunt
                # curvature). A rotated section's flat far surface has a WORLD
                # slope approximately equal to ``frame.rotation_deg``
                # everywhere, so measuring the local slope directly subsumes
                # the global-rotation case (stacking the two would double-correct
                # a rotated section's flat regions). The slope is measured over
                # the ORDINARY (un-widened) window, not the extremity-widened
                # one: the neck this check approximates forms at the slot's own
                # corner, and a huge extremity-widened span would dilute a real
                # local slope into a shallow secant across mostly irrelevant
                # material.
                far_slope_deg = estimate_far_surface_slope_deg(
                    frame.ops,
                    frame.polygon,
                    ordinary_halo_min_x,
                    ordinary_halo_max_x,
                    upper=family.entry_lower,
                )
                required_halo_mm = corrected_skin_margin_mm(
                    2.0 * safety_offset_mm,
                    far_slope_deg if far_slope_deg is not None else frame.rotation_deg,
                )
                if remaining_halo_mm < required_halo_mm - _THIN_SECTION_EPSILON_MM:
                    omitted.append(
                        OmittedWeb(
                            x_mm=x,
                            reason=(
                                "thin section: remaining skin {:.4f} mm at "
                                "x={:.4f} mm, measured across the "
                                "{:.4f} mm erosion halo, cannot survive "
                                "2x the {:.4f} mm safety offset "
                                "({:.4f} mm required, {:.4f} mm after this "
                                "station's own {} degree local far-surface "
                                "slope correction) — the margin fits, but "
                                "the section is too thin here to survive "
                                "perimeter erosion.".format(
                                    remaining_halo_mm,
                                    x,
                                    clamped_span_mm,
                                    safety_offset_mm,
                                    2.0 * safety_offset_mm,
                                    required_halo_mm,
                                    "{:.4f}".format(far_slope_deg)
                                    if far_slope_deg is not None
                                    else "{:.4f} (fallback: global rotation)".format(
                                        frame.rotation_deg
                                    ),
                                )
                            ),
                            family_index=family_index,
                            family_name=family.name,
                        )
                    )
                    continue

            # Perpendicular (surface-normal) web entry, threshold-gated and
            # clamped. Off by default: ``spec.max_web_tilt_deg`` defaults to
            # ``0.0``, which short-circuits this whole block before any
            # measurement is taken (``end_x`` stays exactly ``x``).
            #
            # Design (deliberately narrow): only ``end``'s X moves.
            # ``local_entry_y``/``end_y`` (both already fully resolved and
            # margin-checked above) are UNCHANGED, so every Y-based safety
            # argument this method already relies on (the skin-margin check,
            # the full-depth headroom, the thin-section halo, and -- downstream
            # -- the opposing-entry meeting condition, whose
            # ``d_upper``/``d_lower`` read the Y-extent between ``start`` and
            # ``end`` and are therefore UNCHANGED by this tilt) survive this
            # step without re-derivation. Only the X footprint moves, and only
            # by an amount bounded by ``max_web_tilt_deg`` and independently
            # re-verified below before being accepted.
            end_x = x
            max_tilt_deg = (
                spec.max_web_tilt_deg if spec.max_web_tilt_deg is not None else 0.0
            )
            if max_tilt_deg > 0.0:
                threshold_deg = (
                    spec.web_tilt_threshold_deg
                    if spec.web_tilt_threshold_deg is not None
                    else DEFAULT_TILT_THRESHOLD_DEG
                )
                off_normal_deg = estimate_local_tilt_deg(
                    frame.ops, frame.polygon, x, upper=not family.entry_lower
                )
                if off_normal_deg is not None and abs(off_normal_deg) > threshold_deg:
                    applied_deg = math.copysign(
                        min(abs(off_normal_deg), max_tilt_deg), off_normal_deg
                    )
                    depth_mm = abs(local_entry_y - end_y)
                    candidate_end_x = x + sign * math.tan(
                        math.radians(applied_deg)
                    ) * depth_mm
                    # Verification, not trust: the linear correction above is
                    # an approximation; re-probe the ACTUAL far surface at the
                    # shifted station before accepting it, and fall back to the
                    # untilted placement (never a hard failure) if the tilt
                    # would leave inadequate clearance there. This also protects
                    # LEADING-EDGE coverage: near the LE the surface is
                    # steepest, so a perpendicular-entry correction is LARGEST
                    # there and most likely to fail this re-probe -- when it
                    # does, the station reverts to vertical, i.e. never worse
                    # than the untilted placement at that station.
                    try:
                        far_lo, far_hi = local_span(
                            frame.ops,
                            frame.polygon,
                            candidate_end_x,
                            DEFAULT_PROBE_WIDTH_MM,
                        )
                        far_at_shifted = far_hi if family.entry_lower else far_lo
                        if sign * (end_y - far_at_shifted) > 0.0:
                            end_x = candidate_end_x
                    except ValueError:
                        pass

            # Every check above (the vertical skin-margin check, the
            # FULL_DEPTH_MARGIN_HEADROOM_MM check, the thin-section halo, and
            # tilt) measures a VERTICAL or worst-case-over-a-span Y quantity.
            # None of them models the MITRE-join erosion loss ``validate.py``'s
            # connectivity check (``join_style=2, mitre_limit=3.0``) applies at
            # THIS web's own far-cap corners specifically: measured at up to
            # 0.028 mm beyond the vertical reading on the reference failure,
            # enough to flip a web that passes every check above into one the
            # validator then severs. This is the LAST gate before acceptance,
            # after every check above: a web that already failed one never
            # reaches here, so this adds an omission reason for webs that
            # survived every other check but would still be severed. Excluded
            # for a socket-referenced member (``depth_to_socket``): its slot is
            # the keyhole shape, which ``far_cap_mitre_clearance_mm``'s
            # rectangle assumption does not model. See that function's
            # docstring for the exact predicate.
            #
            # Same scope gate as the halo check above (``spec.skin_margin_mm >=
            # 2.0 * safety_offset_mm``): below that threshold even an
            # infinitely thick flat section cannot pass the validator's own
            # erosion check (see the module docstring), so that regime violates
            # TopologyValidator's precondition and is not a thinness finding
            # this generator-side approximation should pre-empt; it stays
            # fail-closed through the validator itself.
            if not depth_to_socket and spec.skin_margin_mm >= 2.0 * safety_offset_mm:
                far_mitre_clearance_mm = far_cap_mitre_clearance_mm(
                    frame.ops,
                    frame.polygon,
                    end_x,
                    width_mm,
                    end_y,
                    -sign,
                    safety_offset_mm,
                )
                if far_mitre_clearance_mm <= safety_offset_mm:
                    # Raise/omit split mirrors whichever branch above
                    # produced this candidate's ``end_y``, NOT a blanket
                    # rule: the FULL-DEPTH branch's own sibling check
                    # (``FULL_DEPTH_MARGIN_HEADROOM_MM``, above) ALWAYS
                    # omits, regardless of ``family.is_marching`` -- a
                    # full-depth family's depth is DERIVED (headroom above
                    # skin_margin_mm), never a specific user request, so a
                    # thin station failing here is exactly the same
                    # "derived placement doesn't fit here" category the
                    # thin-section halo check already treats
                    # as an ordinary omission, single-family/non-marching
                    # or not (the "family 0 ('default')" tip-shutdown
                    # regression is exactly that case -- see
                    # ``tests/integration/test_tip_shutdown.py``). Only the
                    # EXPLICIT-DEPTH branch's own sibling check (below,
                    # ``family.web_depth_mm`` not ``None``) raises for a
                    # non-marching family, because THAT depth is a specific
                    # user request failing, not a derived one.
                    if family.web_depth_mm is not None and not family.is_marching:
                        raise ValueError(
                            "family {} ({!r}): the candidate at chord "
                            "station x={:.4f} mm passes every vertical "
                            "skin-margin/thin-section check but leaves "
                            "only {:.4f} mm of true (mitre-dilated) "
                            "clearance from its own far-cap corners to the "
                            "far surface, at or below the required "
                            "safety_offset_mm={:.4f} mm — refusing to "
                            "place a web the validator's own mitre erosion "
                            "check would then sever.".format(
                                family_index,
                                family.name,
                                x,
                                far_mitre_clearance_mm,
                                safety_offset_mm,
                            )
                        )
                    omitted.append(
                        OmittedWeb(
                            x_mm=x,
                            reason=(
                                # "skin margin:" prefix retained so the
                                # all-thin classification keeps treating
                                # this as a thinness omission; the stable
                                # "mitre clearance:" marker after it is
                                # what MITRE_OMISSION_MARKER counts on, so
                                # the first time this predicate costs a
                                # real wing any structure it is visible in
                                # the run summary rather than only in a
                                # sweep tool.
                                "skin margin: mitre clearance: local "
                                "thickness {:.4f} mm at "
                                "x={:.4f} mm passes the vertical "
                                "skin-margin/thin-section checks but "
                                "leaves only {:.4f} mm of true "
                                "(mitre-dilated) clearance from the "
                                "far-cap corners to the far surface, at or "
                                "below the required safety_offset_mm="
                                "{:.4f} mm — a marched station whose far "
                                "cap corners would not survive the "
                                "validator's mitre erosion is omitted, "
                                "never placed.".format(
                                    thickness_mm,
                                    x,
                                    far_mitre_clearance_mm,
                                    safety_offset_mm,
                                )
                            ),
                            family_index=family_index,
                            family_name=family.name,
                        )
                    )
                    continue

            # Lateral clearance. A real wing (layer 786) severed a
            # knife-edge island at a web whose FAR-cap clearance (immediately
            # above) was never in question -- the true hazard was LATERAL, an
            # open notch on the SAME side the web enters, 0.84275 mm
            # (mitre-exact) from the web's own side wall, just barely "thick
            # enough" for every existing vertical/far-cap check yet severed by
            # the validator's own mitre erosion because the true geometry is a
            # converging wedge, not a parallel corridor. Same gate as the
            # far-cap check immediately above (never runs for a socket member,
            # whose slot is not the rectangle either predicate assumes; never
            # runs below the halo's own ``2 * safety_offset_mm`` skin-margin
            # precondition, for the identical reason that check documents).
            # Placed AFTER the far-cap check: a web already omitted/raised
            # above never reaches here.
            if not depth_to_socket and spec.skin_margin_mm >= 2.0 * safety_offset_mm:
                lateral_clearance_mm = opposing_face_lateral_clearance_mm(
                    frame.ops,
                    frame.polygon,
                    end_x,
                    width_mm,
                    local_entry_y,
                    end_y,
                    family.entry_lower,
                    safety_offset_mm,
                )
                if lateral_clearance_mm <= safety_offset_mm:
                    # Raise/omit split mirrors the far-cap check immediately
                    # above -- see that block's own comment for why this is
                    # keyed on ``family.web_depth_mm``/``is_marching``, not
                    # a blanket rule.
                    if family.web_depth_mm is not None and not family.is_marching:
                        raise ValueError(
                            "family {} ({!r}): the candidate at chord "
                            "station x={:.4f} mm passes every vertical "
                            "skin-margin/thin-section/far-cap check but "
                            "leaves only {:.4f} mm of true (mitre-dilated) "
                            "LATERAL clearance from its own side wall to an "
                            "opposing section feature (e.g. an open notch "
                            "on the same side), at or below the required "
                            "safety_offset_mm={:.4f} mm — refusing to "
                            "place a web the validator's own mitre erosion "
                            "check would then sever.".format(
                                family_index,
                                family.name,
                                x,
                                lateral_clearance_mm,
                                safety_offset_mm,
                            )
                        )
                    omitted.append(
                        OmittedWeb(
                            x_mm=x,
                            reason=(
                                # Same "skin margin:" + MITRE_OMISSION_MARKER
                                # prefix as the far-cap omission above, so
                                # this counts identically toward the
                                # existing all-thin classification and the
                                # capability's own ``mitre_omitted`` tally --
                                # this is the SAME predicate family (a
                                # mitre-erosion clearance check the
                                # validator's own erosion would otherwise
                                # fail), just measuring the lateral
                                # direction instead of the far cap.
                                "skin margin: mitre clearance: local "
                                "thickness {:.4f} mm at "
                                "x={:.4f} mm passes the vertical "
                                "skin-margin/thin-section/far-cap checks "
                                "but leaves only {:.4f} mm of true "
                                "(mitre-dilated) LATERAL clearance from its "
                                "own side wall to an opposing section "
                                "feature, at or below the required "
                                "safety_offset_mm={:.4f} mm — a marched "
                                "station whose side wall would not survive "
                                "the validator's mitre erosion against a "
                                "nearby opposing feature is omitted, never "
                                "placed.".format(
                                    thickness_mm,
                                    x,
                                    lateral_clearance_mm,
                                    safety_offset_mm,
                                )
                            ),
                            family_index=family_index,
                            family_name=family.name,
                        )
                    )
                    continue

            # DEGENERATE-LENGTH GUARD. A web whose own depth has collapsed
            # to near zero is not a structural member -- it is a hole. The
            # depth clamps above can drive it there on a thin trailing-edge
            # station without any of them omitting the candidate, and
            # ``SlotBuilder.build`` then extends the stub by
            # ``overshoot_mm`` past the entry, so a ~0.002 mm "web" becomes
            # a cut of overshoot length that punches straight through a
            # thin section and severs the island.
            #
            # Measured in-host on a real wing (layer 860,
            # Z=172.20): ``deep_plus_upper`` at x=93.2265 with start_y -3.0278
            # and end_y -3.0302 -- a depth of 0.0024 mm, where that layer's
            # other three ``deep_plus_upper`` members measured 7.0, 9.7 and
            # 11.8 mm. The validator caught it as "slot disconnects material
            # island", and the ablation named exactly this slot.
            #
            # The threshold is ``overshoot_mm``, not a tuned number. ``SlotBuilder.build`` extends the slot past the entry
            # boundary by exactly that much. So a web whose OWN depth is
            # smaller than the overshoot produces a cut that is mostly
            # overshoot -- it is not the feature that was requested, it is
            # the builder's entry allowance with a stub attached.
            #
            # ``safety_offset_mm`` (0.42 mm) is not used as the threshold:
            # it is 8x larger and catches legitimate short webs (on the tip
            # fixture shutdown_count goes 19 -> 37 and two-tier family tests
            # lose a slot each). A threshold that removes real structure to
            # catch a stub is the wrong trade.
            web_depth_actual_mm = abs(end_y - local_entry_y)
            if web_depth_actual_mm < spec.overshoot_mm:
                omitted.append(
                    OmittedWeb(
                        x_mm=x,
                        reason=(
                            "skin margin: degenerate web depth {:.4f} mm at "
                            "x={:.4f} mm, below the {:.4f} mm overshoot "
                            "-- the depth clamps collapsed this candidate to "
                            "a stub that contributes no structure, and the "
                            "slot builder's overshoot would turn it into a "
                            "cut through a thin section. Omitted, never "
                            "placed.".format(
                                web_depth_actual_mm, x, spec.overshoot_mm)
                        ),
                        family_index=family_index,
                        family_name=family.name,
                    )
                )
                continue

            intents.append(
                WebIntent(
                    start=(x, local_entry_y),
                    end=(end_x, end_y),
                    width_mm=width_mm,
                    family_index=family_index,
                    family_name=family.name,
                    entry_lower=family.entry_lower,
                    # Marks this member as positioned relative to ``socket`` --
                    # see ``WebIntent.references_socket`` and the
                    # socket-footprint exclusion's own comment in
                    # ``generate()`` for why it must be exempted there.
                    references_socket=depth_to_socket,
                    web_id=_stamp_id,
                )
            )

        _stamp_omitted(omitted, _stamp_from, _stamp_id)
        return intents, omitted

    def _generate_opposing_pair_family(
        self,
        frame: LayerFrame,
        spec: StructureSpec,
        family: WebFamily,
        family_index: int,
        outer_wall_line_width_mm: float,
        layer: LayerContext,
        safety_offset_mm: float,
    ) -> Tuple[List[WebIntent], List[OmittedWeb]]:
        """Emit ONE family's opposing pairs -- two ``WebIntent``s per
        station, one entering each skin, offset in X just far enough that their
        own flanks fuse, with a depth DERIVED every layer so the pair overlaps
        in Y and ties the two skins together.

        Why this is a separate method, not a branch inside
        ``_generate_family``: every other family places ONE candidate per
        station, with a depth that is either full-depth or a fixed
        ``web_depth_mm``. A pair is two coupled candidates whose depths are
        derived TOGETHER from one shared thickness measurement. Fitting that
        into the single-candidate loop (which threads ``family.entry_lower``,
        ``depth_to_socket``, and the tilt correction through one ``sign``
        variable per candidate) would either mix the two shapes or require
        restructuring that loop's contract. ``family.entry_lower``,
        ``family.web_depth_mm`` and ``family.depth_to_socket`` are never
        consulted here: a pair's entry side and depth are not configurable, and
        the family's own ``entry_lower`` is ignored.

        Placement: for each station ``x`` returned by ``_candidate_stations``
        (the same anchor/pitch/drift arithmetic every other family uses, so a
        pair family can still march or drift like an ordinary one), two
        sub-stations are placed:

            x_upper = x - pair_offset_mm / 2   (+Y entry, entry_lower=False)
            x_lower = x + pair_offset_mm / 2   (-Y entry, entry_lower=True)

        ``pair_offset_mm`` defaults to one fin's own printed thickness (``2 *
        outer_wall_line_width_mm + width_mm``) plus ``_HARD_FUSE_GAP_MM`` (the
        measured hard-fuse gap), which is the measured 0.88 mm centre spacing
        that leaves a 0.019 mm flank gap (fused) at a 0.42 mm wall. An explicit
        ``pair_offset_mm`` wider than that maximum is REFUSED here (raises
        ``ValueError`` naming the family and both figures) rather than silently
        built as a non-fusing pair, which would do nothing.

        Depth derivation: for each station, ``T_worst`` is measured via
        ``local_solid_thickness_worst_case`` over the PAIR'S OWN COMBINED
        footprint (from ``x_upper``'s left edge to ``x_lower``'s right edge),
        less ``min_wall_mm`` when the footprint spans a hole. Both members are
        built at the SAME depth:

            d_upper = d_lower = T_worst - corrected_skin_margin_mm
                                - _DEPTH_EPSILON_MM

        Each member starts AT its own surface and stops the rotation-corrected
        skin margin (less an epsilon) short of the far one, the same depth a
        full-depth member uses, so the pair reaches ``2 * T_worst - 2 * margin``.
        That exceeds ``T_worst`` whenever ``T_worst > 2 * margin``, which holds
        on any section where a full-depth member can be placed, so the two
        members overlap in Y and tie the skins together. (Halving the thickness
        instead, ``(T_worst - min_wall) / 2`` per side, would leave a gap
        because the members start at the surface, not one margin inside it.)

        At the fusing offset the two members' haloed footprints are X-disjoint,
        so ``opposing_pair_meets`` returns ``None`` for a pair's own members and
        the opposing-entry omission does not constrain the pair. The pair is
        judged by the validator after generation (see ``pair_ablation.py``),
        which removes a single pair station if the layer's erosion check splits
        an island.

        Fail-closed, never a negative or degenerate depth: a derived depth at
        or below ``_MIN_USEFUL_OPPOSING_PAIR_DEPTH_MM`` omits the WHOLE pair at
        that station (both sub-candidates), naming the family, station,
        measured ``T_worst`` and the derived depth -- never clamped to a
        smaller positive value and never split so only one side is built (a
        one-sided "pair" ties nothing).
        """
        width_mm = family.resolve_gap(spec, outer_wall_line_width_mm)
        anchor_x = frame.web_x(family.web_position) + family.station_drift_mm(
            layer.print_z, local_chord_mm=frame.width
        )
        pitch_mm = family.resolved_rib_pitch_mm(frame.width)

        fin_thickness_mm = 2.0 * outer_wall_line_width_mm + width_mm
        max_pair_offset_mm = fin_thickness_mm + _HARD_FUSE_GAP_MM
        if family.pair_offset_mm is not None:
            offset_mm = family.pair_offset_mm
            if offset_mm > max_pair_offset_mm + _FOOTPRINT_EPSILON_MM:
                flank_gap_mm = offset_mm - fin_thickness_mm
                raise ValueError(
                    "family {} ({!r}): pair_offset_mm={:.4f} mm is wider "
                    "than one fin thickness ({:.4f} mm = 2 * "
                    "outer_wall_line_width_mm + slot width) plus the "
                    "{:.4f} mm hard-fuse gap ({:.4f} mm maximum) -- "
                    "the resulting {:.4f} mm flank gap provably cannot "
                    "fuse. Refusing rather than silently building a "
                    "non-fusing pair.".format(
                        family_index,
                        family.name,
                        offset_mm,
                        fin_thickness_mm,
                        _HARD_FUSE_GAP_MM,
                        max_pair_offset_mm,
                        flank_gap_mm,
                    )
                )
        else:
            offset_mm = max_pair_offset_mm

        half_w = width_mm / 2.0
        min_wall_mm = 2.0 * safety_offset_mm

        intents: List[WebIntent] = []
        omitted: List[OmittedWeb] = []
        # Web identity: members are ``(family_index, k, "U")`` and
        # ``(..., "L")``; an omission is for the whole pair, side ``P``.
        per_station_reentrant = _per_station_reentrant(spec)
        # Optional per-member depth caps (same seam and semantics as
        # ``_generate_family``; ``None`` means no caps).
        depth_caps = getattr(layer, "depth_caps", None)
        _stamp_from = 0
        _stamp_id = None

        for station_k, x in self._candidate_stations_k(frame, pitch_mm, anchor_x):
            _stamp_omitted(omitted, _stamp_from, _stamp_id)
            _stamp_from = len(omitted)
            _stamp_id = (family_index, station_k, WEB_SIDE_PAIR)
            x_upper = x - offset_mm / 2.0
            x_lower = x + offset_mm / 2.0
            footprint_min_x = x_upper - half_w
            footprint_max_x = x_lower + half_w

            if (
                footprint_min_x < frame.min_x - _FOOTPRINT_EPSILON_MM
                or footprint_max_x > frame.max_x + _FOOTPRINT_EPSILON_MM
            ):
                omitted.append(
                    OmittedWeb(
                        x_mm=x,
                        reason=(
                            "chord extent: opposing pair's combined "
                            "footprint [{:.4f}, {:.4f}] mm does not lie "
                            "within the section's chord extent "
                            "[{:.4f}, {:.4f}] mm".format(
                                footprint_min_x,
                                footprint_max_x,
                                frame.min_x,
                                frame.max_x,
                            )
                        ),
                        family_index=family_index,
                        family_name=family.name,
                    )
                )
                continue

            if spec.expected_hole_count > 0:
                # Same convenience exclusion as the ordinary family loop above
                # -- a pair whose combined, haloed footprint overlaps a
                # declared interior hole cannot be measured by the hole-blind
                # primitives below.
                halo_min_x = footprint_min_x - safety_offset_mm
                halo_max_x = footprint_max_x + safety_offset_mm
                hole_conflict = None
                for hole_min_x, _hole_min_y, hole_max_x, _hole_max_y in (
                    frame.ops.hole_bounds(frame.polygon)
                ):
                    if halo_max_x > hole_min_x and halo_min_x < hole_max_x:
                        hole_conflict = (hole_min_x, hole_max_x)
                        break
                if hole_conflict is not None:
                    hc_min_x, hc_max_x = hole_conflict
                    omitted.append(
                        OmittedWeb(
                            x_mm=x,
                            reason=(
                                "hole footprint: opposing pair's haloed "
                                "combined footprint [{:.4f}, {:.4f}] mm "
                                "overlaps the declared interior hole's own "
                                "X footprint [{:.4f}, {:.4f}] mm".format(
                                    halo_min_x, halo_max_x, hc_min_x, hc_max_x
                                )
                            ),
                            family_index=family_index,
                            family_name=family.name,
                        )
                    )
                    continue

            try:
                worst_case = local_solid_thickness_worst_case(
                    frame.ops,
                    frame.polygon,
                    (footprint_min_x + footprint_max_x) / 2.0,
                    footprint_max_x - footprint_min_x,
                    DEFAULT_PROBE_WIDTH_MM,
                )
            except ValueError as exc:
                omitted.append(
                    OmittedWeb(
                        x_mm=x,
                        reason=(
                            "opposing pair: local thickness across the "
                            "pair's combined footprint [{:.4f}, {:.4f}] mm "
                            "could not be measured ({}) -- treated "
                            "conservatively as a station too thin for a "
                            "useful pair.".format(
                                footprint_min_x, footprint_max_x, exc
                            )
                        ),
                        family_index=family_index,
                        family_name=family.name,
                    )
                )
                continue

            t_worst = worst_case.solid_mm
            if worst_case.hole_extent_subtracted_mm > 0.0:
                t_worst -= min_wall_mm

            # FULL depth per side, not half the section.
            #
            # A halved form `(t_worst - min_wall)/2` would assume a member
            # starts one skin_margin INSIDE its skin, so that overlap needs
            # `d_u + d_l > T - 2*skin_margin`. A member actually starts AT its
            # surface and stops skin_margin short of the far one, so overlap
            # needs `d_u + d_l > T`. The halved form gives `t_worst - min_wall`,
            # which is strictly LESS than T and therefore guarantees a GAP
            # (measured at -0.868 to -0.889 mm over 510 layers, never an
            # overlap).
            #
            # Each member may individually reach `T - skin_margin` (the same
            # depth a full-depth member uses), so a pair reaches
            # `2T - 2*skin_margin`, which exceeds T whenever
            # `T > 2*skin_margin`. At the fusing offset the two haloed
            # footprints are X-disjoint, so `opposing_pair_meets` returns None
            # and does not constrain the pair.
            #
            # With this form: 1689 skin-to-skin ties, flank gap exactly
            # 0.021 mm, depth overlap 10.0-15.6 mm. The depth is reduced by the
            # ROTATION-CORRECTED margin and an epsilon: leaving exactly
            # `spec.skin_margin_mm` fails the far-end check, which compares
            # strictly AND applies this layer's own slope correction on top --
            # the same correction every other member uses via
            # `corrected_skin_margin_mm`.
            depth_mm = (t_worst
                        - corrected_skin_margin_mm(
                            spec.skin_margin_mm, frame.rotation_deg)
                        - _DEPTH_EPSILON_MM)
            if depth_mm <= _MIN_USEFUL_OPPOSING_PAIR_DEPTH_MM:
                omitted.append(
                    OmittedWeb(
                        x_mm=x,
                        reason=(
                            "opposing pair: worst-case local thickness "
                            "{:.4f} mm across [{:.4f}, {:.4f}] mm derives a "
                            "per-side depth of {:.4f} mm (({:.4f} - "
                            "{:.4f}) / 2), at or below the "
                            "{:.4f} mm minimum useful opposing-pair depth "
                            "-- the section is too thin here for a useful "
                            "pair.".format(
                                t_worst,
                                footprint_min_x,
                                footprint_max_x,
                                depth_mm,
                                t_worst - min_wall_mm,
                                min_wall_mm,
                                _MIN_USEFUL_OPPOSING_PAIR_DEPTH_MM,
                            )
                        ),
                        family_index=family_index,
                        family_name=family.name,
                    )
                )
                continue

            try:
                local_entry_upper = local_upper_worst_case(
                    frame.ops, frame.polygon, x_upper, width_mm, DEFAULT_PROBE_WIDTH_MM
                )
                local_entry_lower = local_lower_worst_case(
                    frame.ops, frame.polygon, x_lower, width_mm, DEFAULT_PROBE_WIDTH_MM
                )
            except ReEntrantSectionError as exc:
                # With ``per_station_reentrant`` (default off): the member
                # entry reads can still hit a notch the combined-footprint
                # thickness read above stepped over.
                if not per_station_reentrant:
                    raise
                omitted.append(
                    OmittedWeb(
                        x_mm=x,
                        reason=(
                            "re_entrant_station: the section is re-entrant "
                            "at this opposing pair's member entry stations "
                            "({:.4f}, {:.4f}) mm ({}). The pair is omitted; "
                            "the layer keeps its other structure.".format(
                                x_upper, x_lower, exc)
                        ),
                        family_index=family_index,
                        family_name=family.name,
                    )
                )
                continue
            end_y_upper = local_entry_upper - depth_mm
            end_y_lower = local_entry_lower + depth_mm

            # No lateral pre-check here: a pair is judged by the validator
            # itself after generation (``aircraft_core/pair_ablation.py``), not
            # by a placement predicate.

            member_ids = (
                (family_index, station_k, WEB_SIDE_UPPER),
                (family_index, station_k, WEB_SIDE_LOWER),
            )
            if depth_caps:
                # Each member honours its own cap. A member whose capped depth
                # falls below the overshoot is absent (the generator's
                # degenerate-depth rule); its partner may still be built (a
                # one-sided tie is legal).
                end_y_upper = _cap_end_y(
                    local_entry_upper, end_y_upper, 1.0,
                    depth_caps.get(member_ids[0]))
                end_y_lower = _cap_end_y(
                    local_entry_lower, end_y_lower, -1.0,
                    depth_caps.get(member_ids[1]))
            member_specs = (
                (x_upper, local_entry_upper, end_y_upper, False, member_ids[0]),
                (x_lower, local_entry_lower, end_y_lower, True, member_ids[1]),
            )
            for m_x, m_entry, m_end, m_lower, m_id in member_specs:
                m_depth = abs(m_end - m_entry)
                if depth_caps and m_depth < spec.overshoot_mm:
                    omitted.append(
                        OmittedWeb(
                            x_mm=m_x,
                            reason=(
                                "skin margin: degenerate web depth {:.4f} mm "
                                "at x={:.4f} mm, below the {:.4f} mm "
                                "overshoot -- this opposing-pair member's "
                                "depth cap leaves no usable depth. Omitted, "
                                "never placed.".format(
                                    m_depth, m_x, spec.overshoot_mm)
                            ),
                            family_index=family_index,
                            family_name=family.name,
                            k=station_k,
                            web_id=m_id,
                        )
                    )
                    continue
                intents.append(
                    WebIntent(
                        start=(m_x, m_entry),
                        end=(m_x, m_end),
                        width_mm=width_mm,
                        family_index=family_index,
                        family_name=family.name,
                        entry_lower=m_lower,
                        web_id=m_id,
                    )
                )

        _stamp_omitted(omitted, _stamp_from, _stamp_id)
        return intents, omitted

    def _candidate_stations(
        self, frame: LayerFrame, rib_pitch_mm, anchor_x: float
    ) -> List[float]:
        """Candidate X stations per the anchoring rule, ascending order.

        ``rib_pitch_mm is None`` yields exactly ``[anchor_x]`` — the single-web
        case (and, per family, the "one web, no march" case). Otherwise marches
        outward both ways from the anchor, each station computed directly from
        its integer offset (never by cumulative addition), stopping in each
        direction once a candidate's own X leaves ``[frame.min_x,
        frame.max_x]``.
        """
        return [x for _k, x in self._candidate_stations_k(frame, rib_pitch_mm, anchor_x)]

    def _candidate_stations_k(
        self, frame: LayerFrame, rib_pitch_mm, anchor_x: float
    ) -> List[Tuple[int, float]]:
        """``_candidate_stations`` with the integer lattice index kept (web
        identity): ``[(k, x)]`` ascending in ``x``, ``x = anchor_x + k *
        pitch`` computed directly from ``k``. A single-station family
        (``rib_pitch_mm is None``) is ``[(0, anchor_x)]``. The ``x`` values and
        their order are exactly those ``_candidate_stations`` always returned."""
        if rib_pitch_mm is None:
            return [(0, anchor_x)]

        pitch = rib_pitch_mm
        stations = [(0, anchor_x)]

        k = 1
        while True:
            x = anchor_x + k * pitch
            if x < frame.min_x - _FOOTPRINT_EPSILON_MM or x > frame.max_x + _FOOTPRINT_EPSILON_MM:
                break
            stations.append((k, x))
            k += 1

        k = -1
        while True:
            x = anchor_x + k * pitch
            if x < frame.min_x - _FOOTPRINT_EPSILON_MM or x > frame.max_x + _FOOTPRINT_EPSILON_MM:
                break
            stations.append((k, x))
            k -= 1

        return sorted(stations, key=lambda kx: kx[1])


# Not a tuned threshold -- just "closer to one integer march index than to
# the boundary between two" (the same kind of tie-tolerance role frame.py's
# `_ROTATION_TIE_EPSILON_MM` plays for a different quantity).
_ORDER_SWAP_K_TOLERANCE = 0.25


def classify_declared_pair_overlaps(intents, spec, frame, layer_index, print_z):
    """Classify every disjoint-membership, measurably-overlapping slot pair on
    THIS layer that the validator's same-edge rule accepts as an "intended
    junction" (``aircraft_core/validate.py``, "Intended junction: footprints
    actually overlap..."), as either the DESIGNED same-family +/- apex (both
    sides land on the SAME march index) or a GENUINE ORDER SWAP (different march
    indices).

    This lives in ``aircraft_core``, beside ``StructureGenerator.
    _candidate_stations``, and not in ``plugin/aircraft_structures/capability.py``,
    because it INVERTS that method's anchoring formula (``x = anchor_x + k *
    pitch``, and ``WebIntent.start[0]`` IS that ``x``, verbatim) to recover
    ``k``. Two independent re-derivations of the same formula in two packages,
    with nothing pinning them together, would be two measurements of one
    quantity that must agree: if ``_candidate_stations``' anchoring changed, a
    copy in the plugin would keep computing march indices against the old
    formula and silently misreport swaps as apexes or vice versa. Next to the
    formula it inverts, a change to one is caught by
    ``test_order_swap_classification.py``'s inversion-pinning test, which runs
    the REAL ``StructureGenerator`` and compares this function's recovered
    ``k`` against station positions the generator actually emitted. It also
    keeps structure generation and validation testable as ordinary geometry
    logic in ``aircraft_core``, with no Orca imports: every argument
    (``intents``, ``spec``, ``frame``, ``layer_index``, ``print_z``) is an
    ``aircraft_core`` type, and its aggregation lives in
    ``aircraft_core/diagnostics.py``
    (``order_swap_count``/``order_swap_events``/``first_order_swap_z``).
    ``plugin/aircraft_structures/capability.py`` calls this function directly
    (resolved by name once the plugin bundle is flattened into one namespace by
    ``tools/build_plugin.py``; see ``CORE_MODULE_ORDER``, which places
    ``generator.py`` before the capability source).

    Returns the list of swap events only. Apex and indeterminate events are
    classified and discarded; this is reporting only and changes nothing in
    ``intents``, ``slots``, or what ``validate()`` decides.

    Scope, narrower than "every accepted overlap":

    - ALL-PAIRS over ``intents`` (every pair, not just X-sorted neighbours).
      ``validate()``'s same-edge scan (``by_x = sorted(...)`` then
      ``zip(by_x, by_x[1:])``, see ``validate.py``) compares only ADJACENT
      slots after sorting by X, which misses a pair that has already crossed
      and separated by more than a slot width by the time the layer's scan
      runs (``tools/d023_arm_sweep.py``'s detector uses the same all-pairs
      approach and found swaps lasting 1-3 layers each). Classifying all pairs
      here is more complete for this purpose and touches none of
      ``validate()``'s control flow.
    - CROSS-TIER pairs (e.g. a skin-lattice candidate overlapping a
      deep/primary-structure candidate) are EXCLUDED; see the "declared pair"
      test below. They have no "apex" concept (two different families with
      different pitches never share one march sequence) and occur routinely
      under chord-relative configs too, so counting them would report
      thousands of "order swaps" unrelated to what this function measures.
    - Only a family pair that is STRUCTURALLY a diamond pair is eligible to be
      classified apex/swap: same ``web_position``, same resolved
      ``rib_pitch_mm`` (at THIS layer's local chord), same ``z_period_mm``, and
      equal-magnitude opposite-sign resolved drift rate. Every other
      disjoint-membership overlap returns unclassified (not counted).
    - March index is recovered EXACTLY (not by a heuristic threshold) from the
      closed-form anchoring equation ``_candidate_stations`` uses:
      ``k = round((start_x - anchor_x) / pitch)``. A residual more than
      ``_ORDER_SWAP_K_TOLERANCE`` pitches from the nearest integer (which
      should not occur for an ordinary marched candidate, but could for a
      socket-flank or keyhole family that satisfies the field-level "declared
      pair" test without marching by ``_candidate_stations``) is left
      unclassified rather than forced into either label.
    - ``web_position``/``z_period_mm`` None-guards: a family can be constructed
      with ``web_position=None`` (``WebFamily`` has no ``__post_init__``; the
      ``0.0 <= web_position <= 1.0`` range check lives in ``validate()``,
      which a caller can skip). A field that can be skipped past ``validate()``
      must still be guarded here rather than trusted. Both guards are live,
      reachable checks, not dead code; see
      ``test_order_swap_classification.py``'s
      ``test_family_with_none_web_position_does_not_raise``.
    """
    events = []
    families = spec.effective_families()
    chord_mm = frame.width
    if not chord_mm:
        return events
    for a_idx in range(len(intents)):
        ia = intents[a_idx]
        for b_idx in range(a_idx + 1, len(intents)):
            ib = intents[b_idx]
            if ia.entry_lower != ib.entry_lower:
                continue
            mem_a, mem_b = ia.family_membership(), ib.family_membership()
            if mem_a & mem_b:
                continue  # same family/component -- not this check
            lo_a = ia.start[0] - ia.width_mm / 2.0
            hi_a = ia.start[0] + ia.width_mm / 2.0
            lo_b = ib.start[0] - ib.width_mm / 2.0
            hi_b = ib.start[0] + ib.width_mm / 2.0
            if min(hi_a, hi_b) - max(lo_a, lo_b) <= 0.0:
                continue  # touching/apart -- not a measured overlap

            fam_a_idx = min(mem_a)
            fam_b_idx = min(mem_b)
            if fam_a_idx >= len(families) or fam_b_idx >= len(families):
                continue  # a socket/scar sentinel index -- not a family pair
            fam_a, fam_b = families[fam_a_idx], families[fam_b_idx]

            # A family CAN be constructed with web_position=None or
            # z_period_mm=None -- there is no __post_init__ on WebFamily, only
            # a range check inside validate(), which a caller can skip. The
            # real socket-flank families (flank_upper_fwd/flank_lower_aft) reach
            # this function with z_period_mm=None on both sides, which None ==
            # None would otherwise pass as "sharing a period" when neither
            # family oscillates at all; guarded explicitly rather than relying
            # on the downstream pitch_a/pitch_b None-check to also happen to
            # catch it.
            if fam_a.web_position is None or fam_b.web_position is None:
                continue
            if fam_a.z_period_mm is None or fam_b.z_period_mm is None:
                continue
            if fam_a.web_position != fam_b.web_position:
                continue
            if fam_a.z_period_mm != fam_b.z_period_mm:
                continue
            try:
                pitch_a = fam_a.resolved_rib_pitch_mm(chord_mm)
                pitch_b = fam_b.resolved_rib_pitch_mm(chord_mm)
            except ValueError:
                continue
            if pitch_a is None or pitch_b is None:
                continue
            if abs(pitch_a - pitch_b) > 1e-9:
                continue

            rate_a = fam_a.z_drift_mm_per_mm
            if rate_a is None and fam_a.z_drift_chord_frac_per_mm is not None:
                rate_a = fam_a.z_drift_chord_frac_per_mm * chord_mm
            rate_b = fam_b.z_drift_mm_per_mm
            if rate_b is None and fam_b.z_drift_chord_frac_per_mm is not None:
                rate_b = fam_b.z_drift_chord_frac_per_mm * chord_mm
            if rate_a is None or rate_b is None:
                continue
            if abs(abs(rate_a) - abs(rate_b)) > 1e-9 or (rate_a > 0) == (rate_b > 0):
                continue  # not equal-magnitude opposite-sign -- not a declared pair

            anchor_a = frame.web_x(fam_a.web_position) + fam_a.station_drift_mm(
                print_z, local_chord_mm=chord_mm
            )
            anchor_b = frame.web_x(fam_b.web_position) + fam_b.station_drift_mm(
                print_z, local_chord_mm=chord_mm
            )
            k_a_frac = (ia.start[0] - anchor_a) / pitch_a
            k_b_frac = (ib.start[0] - anchor_b) / pitch_b
            k_a = round(k_a_frac)
            k_b = round(k_b_frac)
            if (
                abs(k_a_frac - k_a) > _ORDER_SWAP_K_TOLERANCE
                or abs(k_b_frac - k_b) > _ORDER_SWAP_K_TOLERANCE
            ):
                continue  # indeterminate -- not forced into either label

            if k_a == k_b:
                continue  # the designed apex -- not a swap

            events.append({
                "layer_index": layer_index,
                "print_z": print_z,
                "families": sorted([fam_a.name, fam_b.name]),
                "march_index_a": k_a,
                "march_index_b": k_b,
                "chord_station_mm": round((ia.start[0] + ib.start[0]) / 2.0, 4),
                "entry_lower": bool(ia.entry_lower),
            })
    return events
