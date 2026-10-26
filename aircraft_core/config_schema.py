"""Single source of truth for the aircraft_structures plugin's stored config.

Every key the plugin will read from its stored ``cap_config`` (top-level, in
``DEFAULT_SPEC``/``CONFIG_KEYS`` terms) or from one entry of its ``families``
list (``_FAMILY_KEYS`` in ``capability.py``) is declared exactly once here,
as a ``ConfigField``: its Python type, its allowed range, whether ``null`` is
a legal value, and a one-line human description. Nothing outside this module
should hard-code that list a second time — ``tools/config.py``'s CLI,
``docs/configuration.md``'s generator, and this module's own validation all
read from ``SPEC_FIELDS``/``FAMILY_FIELDS``. ``capability.py``'s own
``_FAMILY_KEYS`` is wired to ``FAMILY_KEYS`` below (bare name, "supplied by
the bundle" — see that file) rather than repeating the key list a second
time, so a family key typo cannot silently diverge between what the plugin
accepts and what this schema thinks it accepts.

This module knows nothing about Orca, JSON files on disk, or the CLI — it is
pure schema plus validation, testable as ordinary Python (``aircraft_core/``,
no Orca imports). Cross-field rules that live on ``StructureSpec``/
``WebFamily`` (the gap-mode exclusivity, the ``z_period_mm`` needs
``z_drift_mm_per_mm`` rule, and critically ``skin_margin_mm > 2 *
safety_offset_mm``, load-bearing for the no-sever proof) are NOT
reimplemented here — ``build_spec`` constructs the real
``StructureSpec``/``WebFamily`` objects with the same coercions
``capability.py``'s ``_build_spec``/``_build_families`` use, and calls their
own ``validate()``, so this module's cross-field errors are always the same
text those objects produce. Never re-deriving the check keeps the two from
disagreeing.
"""

import difflib
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from .spec import ROLE_SKIN_LATTICE, SpecError, StructureSpec, WebFamily

# Resolved outer wall line width assumed when a caller does not supply one —
# same value and provenance as tools/set_gap_ratio.py's DEFAULT_WALL_MM (the
# resolved default on the Elegoo CC 0.4). Only used to evaluate the
# skin_margin_mm > 2 * safety_offset_mm rule offline, before any host is
# involved; the real resolved value always wins once one is available.
DEFAULT_WALL_MM = 0.42


class ConfigSchemaError(ValueError):
    """One or more keys/values in a stored config do not match the schema.

    Distinct from ``aircraft_core.spec.SpecError``: this is raised for
    structural problems (unknown key, wrong type, out of range) that must be
    caught before a ``StructureSpec``/``WebFamily`` is even constructed;
    ``SpecError`` is raised by ``build_spec`` afterwards, for cross-field
    rules those objects already own.
    """


@dataclass(frozen=True)
class ConfigField:
    """One accepted key's type, range, nullability and description."""

    type: type
    description: str
    default: Any
    none_allowed: bool = False
    minimum: Optional[float] = None
    min_exclusive: bool = False
    maximum: Optional[float] = None
    max_exclusive: bool = False

    def range_text(self) -> str:
        if self.minimum is None and self.maximum is None:
            return ""
        # An unset bound is unbounded (no actual finite value to be
        # inclusive/exclusive OF), so it always renders open, regardless of
        # the exclusive flag -- only a SET bound's own exclusivity flag
        # matters.
        lo = "(" if (self.minimum is None or self.min_exclusive) else "["
        hi = ")" if (self.maximum is None or self.max_exclusive) else "]"
        lo_val = "-inf" if self.minimum is None else self.minimum
        hi_val = "inf" if self.maximum is None else self.maximum
        return "{}{}, {}{}".format(lo, lo_val, hi_val, hi)


# --- top-level ("cap_config") keys ------------------------------------------
#
# Values/defaults here are drawn verbatim from
# plugin/aircraft_structures/capability.py's DEFAULT_SPEC (the runtime
# source of truth for what a MISSING key resolves to) — kept in sync by
# tests/unit/test_config_schema.py, which parses DEFAULT_SPEC out of
# capability.py's source without importing it (capability.py imports
# ``orca``, which does not exist outside the host) and asserts both the key
# set and every default value match.

SPEC_FIELDS: Dict[str, ConfigField] = {
    "gap_ratio": ConfigField(
        float,
        "Slot width as a multiple of the resolved outer wall line width. "
        "Exactly one of gap_ratio / slot_gap_mm must be set (the other left "
        "null).",
        default=0.5,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "slot_gap_mm": ConfigField(
        float,
        "Explicit slot width in millimetres. Exactly one of gap_ratio / "
        "slot_gap_mm must be set (the other left null).",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "skin_margin_mm": ConfigField(
        float,
        "Minimum remaining skin thickness a web must stop short of at its "
        "far end, in millimetres. Must exceed 2x the resolved outer wall "
        "line width -- load-bearing for the no-sever proof.",
        default=1.0,
        minimum=0.0,
    ),
    "root_solid_layers": ConfigField(
        int,
        "Number of body layers, from the bed (layer 0), that stay solid (no "
        "web) -- a bed-adhesion guard, not an anatomical-root guard: on a "
        "tip-face-down print (root_end='top') it protects the tip end at the "
        "bed.",
        default=5,
        minimum=0,
    ),
    "web_position": ConfigField(
        float,
        "Fraction across the chord axis (0..1) locating the web centerline "
        "when families is empty.",
        default=0.5,
        minimum=0.0,
        maximum=1.0,
    ),
    "overshoot_mm": ConfigField(
        float,
        "Distance the slot extends past the exterior boundary before "
        "subtraction, in millimetres, guaranteeing a clean cut.",
        default=0.05,
        minimum=0.0,
        min_exclusive=True,
    ),
    "rib_pitch_mm": ConfigField(
        float,
        "Spacing between webs along the chord axis, in millimetres, when "
        "families is empty. Null produces a single web at web_position.",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "web_depth_mm": ConfigField(
        float,
        "How far a web reaches in from the entry surface, in millimetres, "
        "when families is empty. Null means full depth (to skin_margin_mm "
        "short of the far surface).",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "families": ConfigField(
        list,
        "Zero or more per-family overrides, each using the family keys "
        "below. Empty (the default) reproduces the single-implicit-family "
        "behaviour built from this spec's own top-level fields.",
        default=[],
    ),
    "z_drift_mm_per_mm": ConfigField(
        float,
        "Top-level convenience for the single implicit family (families "
        "empty): chord drift per millimetre of layer Z. Null means no "
        "drift.",
        default=None,
        none_allowed=True,
    ),
    "z_period_mm": ConfigField(
        float,
        "Top-level convenience for the single implicit family: Z distance "
        "over which the drift zigzags once. Requires z_drift_mm_per_mm to "
        "also be set; null means unbounded linear drift.",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "tip_shutdown_enabled": ConfigField(
        bool,
        "SUPERSEDED and now INERT: since Class B degradation "
        "became the default, a layer whose every candidate web was omitted "
        "is left unmutated and reported whatever this flag says, so setting "
        "it changes nothing. Still ACCEPTED rather than removed, because "
        "existing configs and the tracked preset set it and rejecting a "
        "known key would be worse than accepting an inert one. Previously: "
        "leave a layer unmutated instead of failing the slice when every "
        "candidate web on it was omitted purely for thinness.",
        default=False,
    ),
    "tip_alignment_skip_enabled": ConfigField(
        bool,
        "Leave a layer unmutated instead of failing the slice when only the "
        "mesh-alignment CENTROID terms disagree, within one layer height of "
        "the mesh's own top Z. A distinct cause from tip_shutdown_enabled -- "
        "see StructureSpec.tip_alignment_skip_enabled for why they are not "
        "the same flag.",
        default=False,
    ),
    "require_primary_structure": ConfigField(
        bool,
        "Fail the slice when a family whose role is primary_structure has a "
        "station omitted, instead of degrading silently. Off by default; see "
        "StructureSpec.require_primary_structure for why.",
        default=False,
    ),
    "per_station_reentrant": ConfigField(
        bool,
        "Omit a single station whose measurement hits a "
        "re-entrant section (reason re_entrant_station) instead of shutting "
        "the whole layer down. Off by default (no behaviour change); see "
        "StructureSpec.per_station_reentrant.",
        default=False,
    ),
    "z_continuity": ConfigField(
        bool,
        "Run the Z-continuity rule (aircraft_core/z_rule.py) in "
        "the per-layer loop -- webs ramp from zero at one layer height per "
        "layer, a lateral jump of a web's slot side walls ends it, the "
        "validator runs in the loop with single-web ablation, and only a "
        "genuinely re-entrant section shuts a whole layer down. Forces "
        "per_station_reentrant on. Off by default (no behaviour change); "
        "see StructureSpec.z_continuity.",
        default=False,
    ),
    "z_continuity_tie_mode": ConfigField(
        str,
        "'independent' (default) -- the two members of an "
        "opposing-pair (tie) station ramp, END and are ablated on their own; "
        "'joint' -- both share one ramp cap and END together. Only read when "
        "z_continuity is true.",
        default="independent",
    ),
    "z_continuity_dx_max_mm": ConfigField(
        float,
        "Largest slot side-wall X step between adjacent layers "
        "that still counts as the same web. None resolves to "
        "aircraft_core.continuity.DEFAULT_DX_MAX_MM (derivation recorded "
        "there). Only read when z_continuity is true.",
        default=None, none_allowed=True, minimum=0.0, min_exclusive=True,
    ),
    "z_continuity_wall_matching": ConfigField(
        bool,
        "Wall-matching variant: a printed slot wall is supported "
        "when it lies within dx_max of ANY printed wall of the same "
        "orientation and entry side on the previous layer (so a merged slot "
        "keeps its depth and only the new inner walls of a split ramp). "
        "False = the per-web rule exactly. Only read when "
        "z_continuity is true.",
        default=True,
    ),
    "spar_diameter_mm": ConfigField(
        float, "Carbon-spar tube diameter in mm. None disables the socket.",
        default=None, none_allowed=True, minimum=0.0, min_exclusive=True,
    ),
    "spar_station": ConfigField(
        float, "Chord fraction where the carbon spar runs.",
        default=0.30, minimum=0.0, maximum=1.0,
    ),
    "spar_clearance_mm": ConfigField(
        float, "Radial slip fit added to the spar radius.",
        default=0.15, minimum=0.0,
    ),
    "spar_neck_width_mm": ConfigField(
        float, "Width of the boundary connection into the spar socket.",
        default=1.2, minimum=0.0, min_exclusive=True,
    ),
    "spar_channel_policy": ConfigField(
        str,
        "Which of MORE THAN ONE detected channel at a single layer is the "
        "spar -- e.g. a wing tip carrying alignment-rod bores alongside "
        "the spar channel. 'refuse' (default): fails the slice closed, "
        "naming every detected channel, exactly as before this field "
        "existed. 'nearest_spar_station': the channel nearest "
        "spar_station (by chordwise position) is the spar; every other "
        "detected channel on that layer is socketed as a plain bore. "
        "Choosing 'nearest_spar_station' makes spar_station load-bearing "
        "for WHICH channel is picked -- unlike the ordinary (0 or 1 "
        "channel) case, where spar_station is never consulted once a "
        "channel is detected. It still never moves or resizes the "
        "socket itself: position/diameter always come from the chosen "
        "channel's own geometry. See StructureSpec.spar_channel_policy "
        "for the full precedence.",
        default="refuse",
    ),
    "max_rotation_deg": ConfigField(
        float,
        "Upper bound, in degrees, on a section's own rotation (washout) "
        "before generation fails closed. Null uses the built-in default "
        "(aircraft_core.frame.DEFAULT_MAX_ROTATION_DEG).",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "web_tilt_threshold_deg": ConfigField(
        float,
        "Perpendicular-entry correction. Below this off-normal error, "
        "in degrees, a web's entry stays exactly vertical. Null uses the "
        "built-in default (aircraft_core.frame.DEFAULT_TILT_THRESHOLD_DEG). "
        "Has no effect while max_web_tilt_deg is 0.0 (the default).",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "max_web_tilt_deg": ConfigField(
        float,
        "Perpendicular-entry correction. Upper bound, in degrees, on "
        "how far a web's entry may tilt away from vertical toward the "
        "local surface normal. 0.0 (the default) disables the correction "
        "entirely -- every web is placed exactly as before this feature "
        "existed.",
        default=0.0,
        minimum=0.0,
    ),
    "metrics_enabled": ConfigField(
        bool,
        "Collect per-layer transform-time performance metrics.",
        default=True,
    ),
    "expected_hole_count": ConfigField(
        int,
        "How many interior holes (e.g. a spar tube) this section is "
        "declared to carry. 0 (the default) means: the "
        "generator never builds a scar, and any interior hole found is "
        "still, unconditionally, the defect invariant 2 exists to catch. "
        "Setting this to 1 declares that the section carries exactly one "
        "interior hole and licenses the generator to connect it to the "
        "exterior boundary with a scar; a hole count that disagrees with "
        "this declared number is itself a defect and fails closed. Capped "
        "at 1 -- the scarring proof is scoped to exactly one declared "
        "hole per layer.",
        default=0,
        minimum=0,
        maximum=1,
    ),
    "allow_config_spar_fallback": ConfigField(
        bool,
        "The CAD model is the source of truth for spar-socket "
        "geometry (a channel detected by mesh-sectioning always wins). "
        "False (the default) fails the slice closed when no channel is "
        "detected but spar_diameter_mm is set, naming this flag. True "
        "restores the config-only socket exactly, as a debugging "
        "fallback only -- every layer it fires on is recorded (see "
        "GenerationResult.spar_fallback_used), never silent.",
        default=False,
    ),
    "feature_aspect_ratio_max": ConfigField(
        float,
        "A layer region is classified FEATURE (non-aerofoil) rather "
        "than AEROFOIL when its bounding-box aspect ratio "
        "(max(width,depth)/min(width,depth)) is at or below this value, "
        "OR its fill ratio is at or above feature_fill_ratio_min. Null "
        "uses the built-in default "
        "(aircraft_core.frame.DEFAULT_FEATURE_ASPECT_RATIO_MAX) -- see "
        "that module for the measured separation this default rests on.",
        default=None,
        none_allowed=True,
        minimum=1.0,
    ),
    "feature_fill_ratio_min": ConfigField(
        float,
        "A layer region is classified FEATURE when its fill ratio "
        "(area / (width*depth), aka solidity) is at or above this value, "
        "regardless of aspect ratio -- catches an elongated tab an "
        "aspect-only test would misclassify as an aerofoil. Null uses "
        "the built-in default "
        "(aircraft_core.frame.DEFAULT_FEATURE_FILL_RATIO_MIN).",
        default=None,
        none_allowed=True,
        minimum=0.0,
        maximum=1.0,
    ),
    "feature_lattice_pitch_mm": ConfigField(
        float,
        "Spacing, in millimetres, between a FEATURE region's own "
        "alternating-entry lattice webs, sized to the region's OWN "
        "extent (never the globally-marched aerofoil stations). Must "
        "exceed the resolved slot width plus 2x the resolved safety "
        "offset, or stations collide.",
        default=2.0,
        minimum=0.0,
        min_exclusive=True,
    ),
    "feature_dense_lattice_enabled": ConfigField(
        bool,
        "False (the default, shipped behaviour "
        "unchanged) bounds a FEATURE region's lattice depth by "
        "frame.width/frame.depth-derived global terms that never let teeth "
        "pass the section's own midline. True additionally tries a "
        "per-station full-local-depth attempt (each tooth reaching to "
        "skin_margin_mm -- the SAME margin the shipped path already uses, "
        "not a smaller feature-specific one; TopologyValidator enforces "
        "skin_margin_mm on every slot regardless, so a smaller margin only "
        "builds teeth it then rejects) and keeps whichever of the two "
        "attempts is BETTER -- more teeth wins, a tie in tooth count is "
        "broken by greater depth -- falling back to the shipped result "
        "silently, never worse than leaving this False, when the dense "
        "attempt is not better (e.g. a chamfered/round feature, where "
        "full local depth fails the thin-section check near the taper) "
        "-- see aircraft_core.spec.StructureSpec."
        "feature_dense_lattice_enabled, StructureGenerator."
        "_generate_feature, and _feature_result_is_better for the full "
        "derivation.",
        default=False,
    ),
    "mesh_alignment_tolerance_mm": ConfigField(
        float,
        "Maximum disagreement, in millimetres, the mesh-to-slice "
        "alignment guard (capability.py._check_mesh_alignment) accepts on "
        "any bbox or centroid term before failing the slice closed. "
        "Default 0.02 mm is 15-20x the measured clean-run noise floor "
        "(~0.001 mm) and is "
        "UNCHANGED from the pre-config module constant this field "
        "replaces. This guard exists to catch a genuinely wrong "
        "mesh-to-slice transform before it places a spar socket from "
        "geometry that is not what is actually printed -- raising this "
        "value does not make a misaligned mesh safe, it trades away that "
        "protection for the ability to slice. Not a general tuning knob: "
        "a wing that needs this raised has a transform problem this field "
        "only hides.",
        default=0.02,
        minimum=0.0,
        min_exclusive=True,
    ),
    "mesh_alignment_sensitivity_enabled": ConfigField(
        bool,
        "Off by default. When true, mesh_alignment_tolerance_mm is "
        "widened PER TERM -- only for a term already over base tolerance, "
        "only when the observed disagreement's sign matches the sign a "
        "measured sub-micron plane-disagreement mechanism predicts, and "
        "capped at one layer height -- to admit a fast-closing tip's own "
        "well-conditioned sensitivity instead of a genuinely wrong "
        "transform. See StructureSpec.mesh_alignment_sensitivity_enabled "
        "and capability.py's own comment above "
        "MESH_ALIGNMENT_SENSITIVITY_K for the full derivation. A rigid "
        "mistranslation is not admitted by this widening even when it "
        "moves a high-rate term.",
        default=False,
    ),
    "root_end": ConfigField(
        str,
        "Which Z end of the placed mesh is the wing root. 'auto' "
        "(default) resolves it from the mesh -- hull area at each Z extreme "
        "corroborated by where the spar channel sits -- and FAILS THE SLICE "
        "if the two signals disagree; 'bed' says the root prints at layer 0; "
        "'top' says the part prints tip-face-down with the root at the "
        "highest layer. The spar-socket latch walks from the root end "
        "toward the tip, so this decides which end of the spar keeps its rod "
        "socket. root_solid_layers stays bed-relative either way.",
        default="auto",
    ),
    "record_host_geometry_sidecar": ConfigField(
        bool,
        "When true, capability.py records, for "
        "every sectioned layer, the host's own slice region polygons (outer "
        "+ holes, mm, per surface) and our mesh section's loops used by the "
        "mesh-alignment guard, plus that guard's six deltas/effective "
        "tolerances/decision -- as a gzip JSON sidecar next to m1_report.json "
        "(m1_host_sections.json.gz), so a guard decision can be replayed "
        "OFFLINE (tools/replay_host_sections.py) against what the host "
        "actually produced, without a live host. Written on both success "
        "and failure. Default FALSE: measured on a fake 1000-layer run, "
        "the marginal cost is ~4.1 "
        "ms/layer (~29% over the no-sidecar baseline) for ~1.8 MB "
        "gzip-compressed -- the size is small but the time cost is not, "
        "so this stays opt-in per this criterion's own instruction to "
        "default off when cost is not small. Turn on per-run for a "
        "corpus/replay entry.",
        default=False,
    ),
}

# --- per-family ("families" list entry) keys --------------------------------

FAMILY_FIELDS: Dict[str, ConfigField] = {
    "name": ConfigField(
        str,
        "Human-readable label for this family, carried into diagnostics "
        "and error messages.",
        default="default",
    ),
    "rib_pitch_mm": ConfigField(
        float,
        "Spacing between this family's webs, in millimetres. Null produces "
        "a single web for this family, at web_position.",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "web_depth_mm": ConfigField(
        float,
        "How far this family's webs reach in from the entry surface, in "
        "millimetres. Null means full depth.",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "web_position": ConfigField(
        float,
        "Fraction across the chord axis (0..1) locating this family's "
        "anchor web.",
        default=0.5,
        minimum=0.0,
        maximum=1.0,
    ),
    "slot_gap_mm": ConfigField(
        float,
        "This family's own explicit slot width in millimetres, or null to "
        "inherit the spec's gap mode.",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "gap_ratio": ConfigField(
        float,
        "This family's own slot width as a multiple of the resolved outer "
        "wall line width, or null to inherit the spec's gap mode.",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "z_drift_mm_per_mm": ConfigField(
        float,
        "This family's chord drift per millimetre of layer Z, or null for "
        "no drift.",
        default=None,
        none_allowed=True,
    ),
    "z_period_mm": ConfigField(
        float,
        "Z distance over which this family's drift zigzags once. Requires "
        "z_drift_mm_per_mm to also be set on this family; null means "
        "unbounded linear drift.",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "entry_lower": ConfigField(
        bool,
        "Which exterior surface this family's webs enter from: false "
        "(default) is the +Y skin, true is the -Y (lower) skin.",
        default=False,
    ),
    "depth_to_socket": ConfigField(
        bool,
        "Derive this family's web depth from the DETECTED spar socket "
        "rather than web_depth_mm, so the member stops one skin margin short "
        "of the socket and can fuse to it. Requires a detected channel on the "
        "layer; omits with a stated reason when there is none. The "
        "socket-to-skin clearance varies along the span, so a fixed "
        "web_depth_mm cannot track it.",
        default=False,
    ),
    "role": ConfigField(
        str,
        "The declared structural intent of this family -- 'skin_lattice' "
        "(default): a distributed shallow member stabilising the thin "
        "printed skin against local buckling. 'primary_structure': a "
        "sparse, deliberately placed member intended to carry a global "
        "load path -- an affirmative design claim, not inferred from "
        "web_depth_mm or anything else about the family. Changes ONLY "
        "diagnostics attribution, never the slot geometry itself (see "
        "aircraft_core.spec.WebFamily.role's own docstring).",
        default=ROLE_SKIN_LATTICE,
    ),
    "rib_pitch_chord_frac": ConfigField(
        float,
        "Pitch as a fraction of this layer's own local chord (frame.width), "
        "instead of millimetres. Mutually exclusive with rib_pitch_mm (set "
        "at most one). Null (default) leaves rib_pitch_mm in sole control -- "
        "byte-identical to every pre-existing config. Trades the "
        "constant-angle property of absolute rib_pitch_mm for a constant "
        "chord-proportion instead (see aircraft_core.spec.WebFamily's "
        "field docstring for the full reasoning); a resolved pitch below "
        "the slot width plus the minimum wall fails the slice closed, "
        "naming the layer's local chord and the resolved pitch.",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "z_drift_chord_frac_per_mm": ConfigField(
        float,
        "Z-drift rate as a fraction of this layer's own local chord per "
        "millimetre of Z, instead of mm/mm. Mutually exclusive with "
        "z_drift_mm_per_mm (set at most one). z_period_mm stays an "
        "absolute millimetre Z-distance regardless of which drift field is "
        "set. Null (default) leaves z_drift_mm_per_mm in sole control -- "
        "byte-identical to every pre-existing config. See "
        "aircraft_core.spec.WebFamily's field docstring for the "
        "constant-angle-vs-constant-chord-proportion tradeoff.",
        default=None,
        none_allowed=True,
    ),
    "opposing_pair": ConfigField(
        bool,
        "This ONE family emits TWO webs per station instead of one -- one "
        "entering the +Y skin, one the -Y skin, offset in X just far "
        "enough that their own flanks fuse, with a depth DERIVED every "
        "layer so the pair overlaps in Y and ties the two skins together. "
        "entry_lower/web_depth_mm/depth_to_socket are ignored for a pair "
        "family. Default false reproduces every pre-existing config "
        "unchanged.",
        default=False,
    ),
    "pair_offset_mm": ConfigField(
        float,
        "Centre-to-centre spacing, in millimetres, between an "
        "opposing_pair family's two stations. Null (default) resolves to "
        "one fin thickness (2 * outer_wall_line_width_mm + slot width) "
        "plus the measured 0.021 mm hard-fuse gap. A value wider than "
        "that maximum is refused at generation time -- the flanks provably "
        "cannot fuse.",
        default=None,
        none_allowed=True,
        minimum=0.0,
        min_exclusive=True,
    ),
    "socket_flank_offset_mm": ConfigField(
        float,
        "Distance, in millimetres, this family's single station is placed "
        "past the DETECTED spar socket's own circle edge, laterally beside "
        "it. Overrides web_position and every other station/drift field "
        "for this family; requires a detected channel on this layer and "
        "fails closed (never falls back to a chord fraction) when there is "
        "none. Null or 0 (default) leaves this family's usual station "
        "logic in sole control -- byte-identical to every pre-existing "
        "config. Requires socket_flank_direction to also be set; mutually "
        "exclusive with rib_pitch_mm/rib_pitch_chord_frac on this family "
        "(both set is a SpecError -- pitch would march additional stations "
        "off the socket-derived anchor, breaking the single-station "
        "guarantee this field name promises). The operating point is tight "
        "by design: an offset must exceed spar_clearance_mm + "
        "2*safety_offset_mm + (this family's resolved slot width)/2 to "
        "clear the socket-footprint exclusion, or the member is omitted "
        "silently (see aircraft_core.spec.WebFamily's field docstring).",
        default=None,
        none_allowed=True,
        minimum=0.0,
    ),
    "socket_flank_direction": ConfigField(
        str,
        "Which side of the detected socket's circle edge "
        "socket_flank_offset_mm is measured toward: 'forward' (toward the "
        "leading edge, -X) or 'aft' (toward the trailing edge, +X). "
        "Required when socket_flank_offset_mm is active (> 0); must be "
        "left null otherwise.",
        default=None,
        none_allowed=True,
    ),
}

SPEC_KEYS = frozenset(SPEC_FIELDS)
FAMILY_KEYS = frozenset(FAMILY_FIELDS)
SPEC_DEFAULTS: Dict[str, Any] = {k: f.default for k, f in SPEC_FIELDS.items()}
FAMILY_DEFAULTS: Dict[str, Any] = {k: f.default for k, f in FAMILY_FIELDS.items()}

# Keys that only make sense in one scope, used to produce a "you put this in
# the wrong place" message instead of a bare "unknown key" one.
_SPEC_ONLY_KEYS = SPEC_KEYS - FAMILY_KEYS
_FAMILY_ONLY_KEYS = FAMILY_KEYS - SPEC_KEYS


def _suggest(key: str, candidates) -> Optional[str]:
    matches = difflib.get_close_matches(key, list(candidates), n=1, cutoff=0.6)
    return matches[0] if matches else None


def unknown_top_level_key_message(key: str) -> str:
    if key in _FAMILY_ONLY_KEYS:
        return (
            "{!r} is a family-scoped key; it cannot be set at the top level "
            "-- set it inside a 'families' entry instead.".format(key)
        )
    suggestion = _suggest(key, SPEC_KEYS)
    if suggestion:
        return "unknown key {!r} in top-level config; did you mean {!r}?".format(
            key, suggestion
        )
    return "unknown key {!r} in top-level config; accepted keys are {}.".format(
        key, sorted(SPEC_KEYS)
    )


def unknown_family_key_message(key: str) -> str:
    if key in _SPEC_ONLY_KEYS:
        return (
            "{!r} is a top-level-scoped key; it cannot be set on a family "
            "-- set it at the top level instead.".format(key)
        )
    suggestion = _suggest(key, FAMILY_KEYS)
    if suggestion:
        return "unknown key {!r} in family config; did you mean {!r}?".format(
            key, suggestion
        )
    return "unknown key {!r} in family config; accepted keys are {}.".format(
        key, sorted(FAMILY_KEYS)
    )


def _check_type(key: str, value: Any, field: ConfigField, label: str) -> Optional[str]:
    if value is None:
        if field.none_allowed:
            return None
        return "{} {!r} must not be null ({}).".format(label, key, field.description)
    if field.type is float:
        ok = isinstance(value, (int, float)) and not isinstance(value, bool)
    elif field.type is bool:
        ok = isinstance(value, bool)
    elif field.type is int:
        ok = isinstance(value, int) and not isinstance(value, bool)
    elif field.type is list:
        ok = isinstance(value, list)
    else:
        ok = isinstance(value, field.type)
    if not ok:
        return "{} {!r} must be {}, got {} ({!r}).".format(
            label, key, field.type.__name__, type(value).__name__, value
        )
    return None


def _check_range(key: str, value: Any, field: ConfigField, label: str) -> Optional[str]:
    if value is None or isinstance(value, bool):
        return None
    if field.minimum is not None:
        if field.min_exclusive and not value > field.minimum:
            return "{} {!r} must be > {}, got {!r}.".format(label, key, field.minimum, value)
        if not field.min_exclusive and not value >= field.minimum:
            return "{} {!r} must be >= {}, got {!r}.".format(label, key, field.minimum, value)
    if field.maximum is not None:
        if field.max_exclusive and not value < field.maximum:
            return "{} {!r} must be < {}, got {!r}.".format(label, key, field.maximum, value)
        if not field.max_exclusive and not value <= field.maximum:
            return "{} {!r} must be <= {}, got {!r}.".format(label, key, field.maximum, value)
    return None


def validate_family_entry(entry: Any, index: int) -> List[str]:
    """Schema-level errors for one entry of a stored ``families`` list.

    Returns a list of human-readable problem strings (empty if the entry is
    clean at the schema level). Does not run ``WebFamily.validate()`` --
    that cross-field check happens in ``build_spec``, once a whole config
    has passed schema validation.
    """
    label = "family {}".format(index)
    if not isinstance(entry, dict):
        return [
            "{} must be an object, got {}.".format(label, type(entry).__name__)
        ]
    errors = []
    for key, value in entry.items():
        if key not in FAMILY_FIELDS:
            errors.append(
                "{}: {}".format(label, unknown_family_key_message(key))
            )
            continue
        field = FAMILY_FIELDS[key]
        msg = _check_type(key, value, field, label)
        if msg:
            errors.append(msg)
            continue
        msg = _check_range(key, value, field, label)
        if msg:
            errors.append(msg)
    return errors


def validate_top_level(config: Any) -> List[str]:
    """Schema-level errors for a whole stored ``cap_config`` dict.

    Checks unknown keys (with a did-you-mean suggestion, or a "wrong scope"
    message when the key is a real family-only key used at the top level),
    types, and ranges -- everything that must hold BEFORE a
    ``StructureSpec``/``WebFamily`` is even constructed. Returns a list of
    problem strings; empty means the config is clean at the schema level
    (cross-field rules are checked separately by ``build_spec``).
    """
    if not isinstance(config, dict):
        return ["config must be an object, got {}.".format(type(config).__name__)]
    errors = []
    for key, value in config.items():
        if key not in SPEC_FIELDS:
            errors.append(unknown_top_level_key_message(key))
            continue
        field = SPEC_FIELDS[key]
        if key == "families":
            if not isinstance(value, list):
                errors.append(
                    "top-level key 'families' must be a list, got {}.".format(
                        type(value).__name__
                    )
                )
                continue
            for i, entry in enumerate(value):
                errors.extend(validate_family_entry(entry, i))
            continue
        msg = _check_type(key, value, field, "top-level key")
        if msg:
            errors.append(msg)
            continue
        msg = _check_range(key, value, field, "top-level key")
        if msg:
            errors.append(msg)
    return errors


def _coerce_family_kwargs(entry: Dict[str, Any]) -> Dict[str, Any]:
    """Mirror capability.py's ``_build_families`` per-entry coercion
    exactly (float()/bool() on the fields that need it), so the
    ``WebFamily`` this module constructs behaves identically to the one the
    running plugin would build from the same stored entry."""
    kwargs = dict(entry)
    for numeric_key in (
        "rib_pitch_mm",
        "web_depth_mm",
        "slot_gap_mm",
        "gap_ratio",
        "z_drift_mm_per_mm",
        "z_period_mm",
        "rib_pitch_chord_frac",
        "z_drift_chord_frac_per_mm",
        "pair_offset_mm",
        "socket_flank_offset_mm",
    ):
        if kwargs.get(numeric_key) is not None:
            kwargs[numeric_key] = float(kwargs[numeric_key])
    if "web_position" in kwargs:
        kwargs["web_position"] = float(kwargs["web_position"])
    if "entry_lower" in kwargs:
        kwargs["entry_lower"] = bool(kwargs["entry_lower"])
    if "depth_to_socket" in kwargs:
        kwargs["depth_to_socket"] = bool(kwargs["depth_to_socket"])
    if "opposing_pair" in kwargs:
        kwargs["opposing_pair"] = bool(kwargs["opposing_pair"])
    return kwargs


def build_spec(config: Dict[str, Any], wall_width_mm: float = DEFAULT_WALL_MM) -> StructureSpec:
    """Build and validate a real ``StructureSpec`` from a stored config dict.

    ``config`` is overlaid on ``SPEC_DEFAULTS`` (missing keys resolve to the
    same defaults the running plugin uses), mirroring
    ``capability.py``'s ``_build_spec``/``_build_families`` coercions
    exactly, then ``StructureSpec.validate(safety_offset_mm=wall_width_mm)``
    is called -- the single place the ``skin_margin_mm > 2 *
    safety_offset_mm`` rule and every other cross-field rule lives. Raises
    ``ConfigSchemaError`` if ``validate_top_level`` finds a schema problem
    first (unknown key, wrong type, out of range) -- cross-field validation
    only ever runs on an already schema-clean config, same order
    ``capability.py`` uses (config parsed and merged, THEN
    ``StructureSpec(...)`` constructed, THEN ``.validate()`` called).
    """
    schema_errors = validate_top_level(config)
    if schema_errors:
        raise ConfigSchemaError(
            "config does not match the schema:\n  - " + "\n  - ".join(schema_errors)
        )

    values = dict(SPEC_DEFAULTS)
    values.update(config)

    families = tuple(
        WebFamily(**_coerce_family_kwargs(entry)) for entry in values["families"]
    )

    spec = StructureSpec(
        slot_gap_mm=values["slot_gap_mm"],
        gap_ratio=values["gap_ratio"],
        skin_margin_mm=float(values["skin_margin_mm"]),
        root_solid_layers=int(values["root_solid_layers"]),
        web_position=float(values["web_position"]),
        overshoot_mm=float(values["overshoot_mm"]),
        rib_pitch_mm=None if values["rib_pitch_mm"] is None else float(values["rib_pitch_mm"]),
        web_depth_mm=None if values["web_depth_mm"] is None else float(values["web_depth_mm"]),
        families=families,
        z_drift_mm_per_mm=(
            None if values["z_drift_mm_per_mm"] is None else float(values["z_drift_mm_per_mm"])
        ),
        z_period_mm=None if values["z_period_mm"] is None else float(values["z_period_mm"]),
        tip_shutdown_enabled=bool(values["tip_shutdown_enabled"]),
        tip_alignment_skip_enabled=bool(values["tip_alignment_skip_enabled"]),
        require_primary_structure=bool(values["require_primary_structure"]),
        per_station_reentrant=bool(values["per_station_reentrant"]),
        z_continuity=bool(values["z_continuity"]),
        z_continuity_tie_mode=str(values["z_continuity_tie_mode"]),
        z_continuity_dx_max_mm=(
            None if values["z_continuity_dx_max_mm"] is None
            else float(values["z_continuity_dx_max_mm"])),
        z_continuity_wall_matching=bool(values["z_continuity_wall_matching"]),
        spar_diameter_mm=values["spar_diameter_mm"],
        spar_station=float(values["spar_station"]),
        spar_clearance_mm=float(values["spar_clearance_mm"]),
        spar_neck_width_mm=float(values["spar_neck_width_mm"]),
        spar_channel_policy=str(values["spar_channel_policy"]),
        max_rotation_deg=(
            None if values["max_rotation_deg"] is None else float(values["max_rotation_deg"])
        ),
        expected_hole_count=int(values["expected_hole_count"]),
        web_tilt_threshold_deg=(
            None
            if values["web_tilt_threshold_deg"] is None
            else float(values["web_tilt_threshold_deg"])
        ),
        max_web_tilt_deg=float(values["max_web_tilt_deg"]),
        allow_config_spar_fallback=bool(values["allow_config_spar_fallback"]),
        feature_aspect_ratio_max=(
            None if values["feature_aspect_ratio_max"] is None
            else float(values["feature_aspect_ratio_max"])
        ),
        feature_fill_ratio_min=(
            None if values["feature_fill_ratio_min"] is None
            else float(values["feature_fill_ratio_min"])
        ),
        feature_lattice_pitch_mm=float(values["feature_lattice_pitch_mm"]),
        feature_dense_lattice_enabled=bool(
            values["feature_dense_lattice_enabled"]
        ),
        mesh_alignment_tolerance_mm=float(values["mesh_alignment_tolerance_mm"]),
        mesh_alignment_sensitivity_enabled=bool(
            values["mesh_alignment_sensitivity_enabled"]
        ),
        record_host_geometry_sidecar=bool(
            values["record_host_geometry_sidecar"]
        ),
        root_end=str(values["root_end"]),
    )
    spec.validate(safety_offset_mm=wall_width_mm)
    return spec


def assert_valid_config(config: Dict[str, Any], wall_width_mm: float = DEFAULT_WALL_MM) -> StructureSpec:
    """Validate ``config`` end to end (schema, then cross-field) and return
    the ``StructureSpec`` it builds. Raises ``ConfigSchemaError`` or
    ``SpecError`` (both ``ValueError`` subclasses) with an actionable
    message on the first class of problem found; callers that only want a
    yes/no plus text can ``str()`` the exception."""
    return build_spec(config, wall_width_mm=wall_width_mm)
