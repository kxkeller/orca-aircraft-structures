"""``StructureSpec``: the user-facing parameters for one structure request.

Pure data plus validation of its own fields. Nothing here touches geometry —
``frame.py`` and ``generator.py`` turn a validated spec plus a per-layer
``LayerFrame`` into web intent.
"""

from dataclasses import dataclass
from typing import Optional, Tuple


class SpecError(ValueError):
    """Raised when a ``StructureSpec`` is internally inconsistent."""


# The two structural roles a ``WebFamily`` may declare. See
# ``WebFamily.role``'s docstring for what each MEANS, not just that it
# exists.
ROLE_SKIN_LATTICE = "skin_lattice"
ROLE_PRIMARY_STRUCTURE = "primary_structure"
VALID_ROLES = (ROLE_SKIN_LATTICE, ROLE_PRIMARY_STRUCTURE)

# Measured maximum fused-flank gap. This is shared by the opposing-pair
# generator and the validator's narrowly bounded fused-pair erosion allowance;
# keeping one definition prevents those two safety decisions from drifting.
FUSED_FLANK_MAX_GAP_MM = 0.021

# The two policies ``StructureGenerator.generate`` may use to resolve MORE THAN
# ONE detected channel at a single layer (e.g. alignment-rod bores: two
# Ø3mm holes that coexist with each other but never with the real spar
# channel). "Which channel is the spar" is a policy question the model alone
# cannot answer -- see the ``StructureSpec.spar_channel_policy`` field comment
# for what each value means and why the default is refusal.
SPAR_CHANNEL_POLICY_REFUSE = "refuse"
SPAR_CHANNEL_POLICY_NEAREST_STATION = "nearest_spar_station"
VALID_SPAR_CHANNEL_POLICIES = (
    SPAR_CHANNEL_POLICY_REFUSE,
    SPAR_CHANNEL_POLICY_NEAREST_STATION,
)


@dataclass(frozen=True)
class WebFamily:
    """One group of webs sharing pitch, depth and phase.

    A family is what turns a single row of full-depth webs into a two-tier
    structure: e.g. one shallow-web family stiffening the skin at a tight
    pitch, plus one deep-web family carrying load at a wider pitch, produced
    together on every body layer.

    Each field here is the family's OWN value — a family does not silently
    inherit ``rib_pitch_mm`` / ``web_depth_mm`` / ``web_position`` from the
    enclosing ``StructureSpec``; those three are named explicitly here
    because that is exactly what varies between tiers. The gap fields are
    the one deliberate exception: leaving both ``slot_gap_mm`` and
    ``gap_ratio`` unset on a family means "use the spec's own gap mode",
    since most families are expected to share bead width and only vary
    pitch/depth/phase.

    Attributes:
        name: Human-readable label, carried into ``WebIntent``/
            ``OmittedWeb`` and diagnostics/validator messages so a two-tier
            report is legible about which family produced which web.
        rib_pitch_mm: Spacing between this family's webs, in millimetres,
            or ``None`` to produce exactly one web (at ``web_position``) for
            this family — the same meaning ``StructureSpec.rib_pitch_mm``
            has for the single implicit family.
        web_depth_mm: How far this family's webs reach in from the surface
            they enter, in millimetres, or ``None`` to run to
            ``skin_margin_mm`` from the far surface (full depth) — the same
            meaning ``StructureSpec.web_depth_mm`` has for the single
            implicit family.
        web_position: Fraction across the chord axis (0 to 1) locating this
            family's anchor web. Defaults to the coupon/section centerline.
        slot_gap_mm: This family's explicit slot width in millimetres, or
            ``None``.
        gap_ratio: This family's slot width as a multiple of the resolved
            outer wall line width, or ``None``.
        z_drift_mm_per_mm: How far this family's stations march
            along the chord (X) per millimetre of layer Z, or ``None`` for
            no Z-variation at all (every layer identical). See ``station_drift_mm`` for the exact formula
            (plain linear drift when ``z_period_mm`` is ``None``, a
            triangle-wave zigzag when it is set). Positive marches toward
            +X as Z rises; negative marches toward -X.
        z_period_mm: Z distance, in millimetres, over which the drift
            zigzags once (one full triangle-wave period) before repeating,
            or ``None`` for unbounded linear drift (the station walks off
            the chord and every station past the extent is omitted, never
            clamped). Setting this WITHOUT ``z_drift_mm_per_mm`` is a
            configuration mistake (there is no slope to reverse), so
            ``validate()`` rejects it. Must be > 0 when set.
        entry_lower: Which exterior surface this family's webs enter
            from. ``False`` (the default) enters from the +Y skin.
            ``True`` anchors this family to the -Y (lower) skin instead --
            the mirror construction the extended no-sever proof for opposing
            (+Y/-Y) entry licenses: entry/far-end measurements are the same
            ``local_upper_worst_case``/``local_lower_worst_case``
            primitives, swapped, and the placement arithmetic is mirrored
            (see ``StructureGenerator._generate_family``'s ``sign``
            variable).
        role: The STRUCTURAL INTENT this family declares, EXPLICITLY, never
            inferred from ``web_depth_mm`` or anything else about the
            family's own geometry. Shallow skin stiffeners, diagonal grid
            members, and full-depth load-carrying webs are not
            interchangeable, and a full-depth skin web is not automatically a
            well-designed spar. Two values:

            - ``"skin_lattice"`` (the default): a distributed, shallow
              member whose job is to stabilise the thin printed skin --
              shorten the unsupported skin bay, resist local
              buckling/oil-canning, spread point loads (System A).
            - ``"primary_structure"``: a sparse, deliberately placed member
              -- typically, but NOT necessarily, full depth -- intended to
              carry a global load path (System B). Setting this is an
              affirmative design claim: "I placed and sized this member on
              purpose to carry load," not merely "this member happens to
              reach the far skin."

            ``role`` changes geometry in exactly one place: a cross-family,
            same-edge merge node where one candidate is ``skin_lattice`` and
            the other ``primary_structure`` and their footprints would
            otherwise fall into the sub-``min_wall_mm`` merge band
            (``StructureGenerator._yield_lattice_to_primary``, called from
            ``_merge_cross_family_nodes``). There the ``skin_lattice``
            candidate is omitted instead of merged, so the
            ``primary_structure`` candidate's own depth is never truncated by
            a lattice member's shallower request (merging at the deeper depth
            or doing nothing are the alternatives; see that method's
            docstring for why they are not used). Every other placement
            decision (station X, depth, entry side, gap, skin margin,
            thin-section erosion, opposing-entry) is untouched by ``role``.
            Beyond that merge interaction, ``role`` affects diagnostics
            attribution (``capability.py``'s per-family/per-role report); it
            is never read by ``TopologyValidator``. Two otherwise-identical
            families that differ only in ``role`` produce identical geometry
            except at that one cross-role merge-band interaction.

            DEFAULT: ``"skin_lattice"``, not ``"primary_structure"``.
            ``primary_structure`` is the affirmative claim above; a family
            nobody explicitly tagged has not earned it, and defaulting to it
            would manufacture the unearned "this is a spar because it is full
            depth" inference: a full-depth skin web that merely reaches the
            far skin is not a designed spar. ``role`` carries no cross-field
            constraint against any other field.
        rib_pitch_chord_frac: Pitch expressed as a fraction of the LOCAL
            chord (``frame.width``, resolved fresh every layer), not
            millimetres -- mutually exclusive with ``rib_pitch_mm`` (both set
            is a ``SpecError``, same shape as ``slot_gap_mm``/``gap_ratio``).
            ``None`` (the default) leaves ``rib_pitch_mm`` in sole control.
            See the field comment below this docstring (after ``role``) for
            the constant-angle-vs-constant-chord-proportion reasoning and the
            min-feature fail-closed guard this triggers in
            ``StructureGenerator._generate_family``.
        z_drift_chord_frac_per_mm: Z-drift RATE expressed as a fraction of
            local chord per millimetre of Z, not mm/mm -- mutually exclusive
            with ``z_drift_mm_per_mm``. ``z_period_mm`` stays in absolute
            millimetres regardless of which drift field is set (it names a
            Z distance, not a chord quantity). ``None`` (the default)
            leaves ``z_drift_mm_per_mm`` in sole control. See the field
            comment below this docstring for the full reasoning.

    ``slot_gap_mm``/``gap_ratio`` may not both be set on the same family.
    Both may be left ``None`` to inherit the spec's own gap mode (see
    ``resolve_gap``).
    """

    name: str = "default"
    rib_pitch_mm: Optional[float] = None
    web_depth_mm: Optional[float] = None
    web_position: float = 0.5
    slot_gap_mm: Optional[float] = None
    gap_ratio: Optional[float] = None
    z_drift_mm_per_mm: Optional[float] = None
    z_period_mm: Optional[float] = None
    entry_lower: bool = False
    # Fields from here on are appended last: this is a frozen dataclass, and
    # positional-argument construction of a WebFamily (test fixtures,
    # tools/structural_analysis.py) must keep working.
    role: str = ROLE_SKIN_LATTICE
    # Chord-relative pitch and drift (geometry-relative member placement).
    # ``None`` on BOTH (the default) leaves ``rib_pitch_mm``/
    # ``z_drift_mm_per_mm`` in sole control; this feature is strictly opt-in.
    #
    # WHY CHORD, NOT SURFACE ARCLENGTH: arclength along the upper/lower
    # surface would be the more natural coordinate for a CURVED-surface
    # lattice, but ``LayerFrame`` carries chord (X) as its only normalized
    # section coordinate -- ``web_x``/``web_position`` are already chord
    # fractions, and every placement primitive in this module
    # (``local_span``, ``local_upper_worst_case``/``local_lower_worst_case``)
    # is a function of chord station, not arclength. A second, arclength-based
    # coordinate system is separate, unproven work; chord fraction is what the
    # codebase already has tested infrastructure for.
    #
    # WHY NOT THICKNESS: pitch and drift are CHORDWISE quantities -- they
    # describe where along the chord a station sits and how far it marches.
    # Local thickness governs DEPTH (``web_depth_mm``), which stays
    # unconditionally absolute here: a shallow lattice member's depth is a
    # bead/nozzle-scale quantity (how many perimeters deep the skin
    # stiffener reaches), not a chord- or thickness-proportional one, and a
    # full-depth member (``web_depth_mm is None``) already adapts to local
    # thickness via ``local_upper_worst_case``/``local_lower_worst_case``.
    #
    # THE CRUX -- CONSTANT ANGLE VS. CONSTANT CHORD-PROPORTION, and why both
    # cannot hold on a taper: a member's TRUE printed angle (its slope in the
    # X-Z plane, dx/dz) is what diagonal-tension / geodetic-construction
    # reasoning is about -- 45 degrees is a claim about the physical angle a
    # shear panel wants its diagonal at, independent of how wide the local
    # chord happens to be. ``z_drift_mm_per_mm`` (absolute mode, the only mode
    # reachable when this field is left ``None``) is a constant mm-per-mm
    # slope -- dx/dz is literally constant along the whole span, so absolute
    # mode IS constant-angle by construction, on a taper or not.
    # ``rib_pitch_chord_frac``/``z_drift_chord_frac_per_mm`` deliberately do
    # NOT preserve that: they resolve against THIS LAYER's own ``frame.width``
    # (the section's local chord, freshly measured every layer -- see
    # ``StructureGenerator._generate_family``), so on a tapering span the
    # resolved mm-per-mm rate SHRINKS as the chord shrinks, and the
    # trajectory's dx/dz is no longer constant -- the printed member's angle
    # changes along the span. What IS held constant instead is the
    # trajectory's CHORD FRACTION at every Z: ``x(z) = min_x(z) + W(z) *
    # (web_position + drift_frac(z) + k * pitch_frac)`` is independent of
    # ``W(z)`` in its FRACTIONAL form -- the station always sits at the same
    # proportional place across whatever chord that layer has. These are two
    # different trajectories and NEITHER is simply "more correct": a diagonal
    # shear panel wants constant angle (absolute mode, the default); a
    # lattice whose point is bay PROPORTIONS relative to the local skin
    # (spacing that stays, say, "one bay per 15% of local chord" no matter how
    # the chord tapers) wants constant chord-proportion (relative mode).
    #
    # SCALE INVARIANCE, the sharpest practical consequence: the crossing
    # condition for two opposed families (``station_drift_mm``'s "CAP ON
    # AMPLITUDE..." note) compares a centre separation ``C = (p_b - p_a) * W``
    # against an amplitude ``A`` and a pitch ``R``, all in millimetres. In
    # ABSOLUTE mode on a tapering span, ``C`` shrinks proportionally to
    # ``W(z)`` while ``A``/``R`` stay fixed, so a crossing that exists at the
    # root can vanish by the tip (or the reverse) -- the lattice drifts
    # nonsensically on swept/tapered geometry. In FULL RELATIVE mode (every
    # one of ``web_position``, ``rib_pitch_chord_frac``,
    # ``z_drift_chord_frac_per_mm`` chosen so that ``C``, ``A``, and ``R`` are
    # ALL proportional to ``W(z)``), every term of the crossing inequality
    # scales by the same factor ``W(z)``, so the inequality's TRUTH VALUE is
    # invariant along the span: a diamond that crosses at the root crosses
    # identically (same margin, in chord-fraction terms) at the tip. Mixing an
    # absolute term with a relative one (e.g. absolute pitch with relative
    # drift) does NOT get this property -- see ``rib_pitch_chord_frac``'s
    # note.
    rib_pitch_chord_frac: Optional[float] = None
    z_drift_chord_frac_per_mm: Optional[float] = None

    # Take this family's depth from the DETECTED spar socket instead of
    # web_depth_mm, so a brace reaches the carbon tube on every layer. The
    # socket-to-skin clearance shrinks 3.89 -> 2.18 mm along the reference
    # wing's span, so a config constant can only be right at one station;
    # the socket's own position is model-derived for the same reason.
    depth_to_socket: bool = False

    # Opposing pair: ``False`` (the default) is a single-entry-side family.
    # ``opposing_pair`` is strictly opt-in.
    #
    # When set, this ONE family emits TWO ``WebIntent``s per station instead
    # of one: one entering the +Y (upper) skin at ``x - pair_offset_mm / 2``,
    # one entering the -Y (lower) skin at ``x + pair_offset_mm / 2`` -- see
    # ``StructureGenerator._generate_opposing_pair_family`` for the full
    # placement and depth derivation. ``entry_lower`` is IGNORED for a pair
    # (both sides are always emitted, unconditionally) -- a family cannot ask
    # for "opposing pair, but only the lower one", since the whole point of
    # the feature is the pair.
    #
    # Depth is NEVER read from ``web_depth_mm``/full-depth placement for a pair
    # family -- both members' depth is DERIVED, every layer, from the same
    # worst-case local-thickness measurement the opposing-entry check itself
    # uses (``frame.opposing_pair_meets`` /
    # ``local_solid_thickness_worst_case``), so the pair overlaps in depth (the
    # property the whole feature exists for) while remaining safe by
    # construction rather than by a fixed ``web_depth_mm`` that cannot track a
    # tapering section (see ``_generate_opposing_pair_family``'s own docstring
    # for the guarantee argument).
    opposing_pair: bool = False
    # Centre-to-centre spacing, in millimetres, between the pair's two
    # stations. ``None`` (the default) resolves to one fin thickness (``2 *
    # outer_wall_line_width_mm + slot_gap_mm``) plus measured hard-fuse gap
    # (0.021 mm) -- the measured 0.88 mm at a 0.42 mm wall that gives a fused,
    # sub-0.021-mm flank gap. Set explicitly only to experiment; a value wider
    # than that maximum is refused at generation time (the flanks provably
    # cannot fuse), never silently built as a non-fusing pair.
    pair_offset_mm: Optional[float] = None

    # Socket-relative placement. Socket coupling is not configurable as an
    # ordinary chord-fraction family; it needs a socket-relative placement
    # mode. ``None`` or ``0`` leaves ``web_position``/drift in sole control of
    # this family's station. A positive value makes this family's SINGLE
    # station the distance, in millimetres, past the DETECTED spar socket's own
    # circle edge (``layer.detected_channels`` -- the model's own channel
    # radius, not ``spec.spar_diameter_mm`` nor the built socket's
    # clearance-inflated ``socket_radius_mm``, exactly the quantity
    # ``m25_socket_flank_search.py`` offsets from), laterally BESIDE the socket
    # rather than co-located with it -- see
    # ``StructureGenerator._generate_family`` for the exact placement formula
    # and the model-derived-every-layer requirement this implements. Measured
    # usable band on the reference fixture: 1.005-1.015 mm past the edge,
    # 35/35 sampled layers both fused (wall gap 0.006-0.016 mm, inside the
    # 0.021 mm fusing window) and validated.
    #
    # This family's entry side is still ``entry_lower`` -- there is no
    # separate field for it; only the CHORD STATION comes from the socket.
    #
    # SINGLE STATION, ENFORCED: an active offset (> 0) combined with either
    # ``rib_pitch_mm`` or ``rib_pitch_chord_frac`` is refused by
    # ``validate()`` below -- without that guard, pitch would march
    # additional stations out from the socket-derived anchor across the
    # whole chord (``StructureGenerator._generate_family`` still calls
    # ``_candidate_stations(frame, pitch_mm, anchor_x)`` unconditionally),
    # landing some of them inside the socket's own footprint exclusion,
    # where they are omitted silently. A socket-flank family is a named
    # single-station placement; a marching pattern anchored on the socket is a
    # different feature, not built.
    #
    # THE OPERATING POINT IS TIGHT, BY DESIGN: the socket-footprint exclusion
    # band (``StructureGenerator.generate()``'s post-family-loop exclusion
    # pass) keeps any candidate whose footprint reaches within
    # ``spar_clearance_mm + 2 * safety_offset_mm`` of the socket's bare
    # detected edge. For this family's own footprint (half-width
    # ``resolved_gap / 2``) to clear that band, the offset must exceed
    # ``spar_clearance_mm + 2 * safety_offset_mm + resolved_gap / 2`` --
    # measured on the reference fixture (``spar_clearance_mm=0.15``,
    # ``safety_offset_mm=0.42``, ``gap_ratio=0.05`` -> half-width ~0.0105 mm)
    # as ~0.9945 mm, against the 1.005 mm measured as fusing: a margin
    # of only ~0.0045 mm. That tightness IS the fusion gap (the 0.021 mm
    # window sits just past it) -- it is the design, not a bug -- but an
    # offset that does NOT clear this constraint is a SILENT per-family
    # omission (the socket-footprint-exclusion reason, not a loud refusal at
    # config time), indistinguishable from any other omitted candidate when
    # other families are still present on the layer.
    socket_flank_offset_mm: Optional[float] = None
    # Which side of the socket's circle edge ``socket_flank_offset_mm`` is
    # measured toward: ``"forward"`` (toward the leading edge, -X) or
    # ``"aft"`` (toward the trailing edge, +X) -- the same sign convention
    # ``m25_socket_flank_search.py``'s ``DIRECTIONS`` table uses.
    # Required (one of those two strings) whenever
    # ``socket_flank_offset_mm`` is active (positive); must be left
    # ``None`` when it is not (unset or 0) -- a direction with nothing to
    # apply it to is a configuration mistake, not something to guess past.
    socket_flank_direction: Optional[str] = None

    def validate(self, index: int) -> None:
        """Raise ``SpecError`` naming this family's index and name on the
        first violated field-level contract."""
        label = "family {} ({!r})".format(index, self.name)
        gap_set = self.slot_gap_mm is not None
        ratio_set = self.gap_ratio is not None
        if gap_set and ratio_set:
            raise SpecError(
                "{} has both slot_gap_mm={!r} and gap_ratio={!r} set; set at "
                "most one, or leave both None to inherit the spec's own gap "
                "mode.".format(label, self.slot_gap_mm, self.gap_ratio)
            )
        if gap_set and self.slot_gap_mm <= 0:
            raise SpecError(
                "{} slot_gap_mm must be > 0, got {!r}.".format(label, self.slot_gap_mm)
            )
        if ratio_set and self.gap_ratio <= 0:
            raise SpecError(
                "{} gap_ratio must be > 0, got {!r}.".format(label, self.gap_ratio)
            )
        if self.rib_pitch_mm is not None and self.rib_pitch_mm <= 0:
            raise SpecError(
                "{} rib_pitch_mm must be > 0 or None, got {!r}.".format(
                    label, self.rib_pitch_mm
                )
            )
        pitch_mm_set = self.rib_pitch_mm is not None
        pitch_frac_set = self.rib_pitch_chord_frac is not None
        if pitch_mm_set and pitch_frac_set:
            raise SpecError(
                "{} has both rib_pitch_mm={!r} and rib_pitch_chord_frac={!r} "
                "set; set at most one -- rib_pitch_mm is an absolute "
                "(constant-angle-preserving) spacing, rib_pitch_chord_frac "
                "is a fraction of the local chord (constant-proportion), "
                "and a single family cannot be both at once.".format(
                    label, self.rib_pitch_mm, self.rib_pitch_chord_frac
                )
            )
        if pitch_frac_set and self.rib_pitch_chord_frac <= 0:
            raise SpecError(
                "{} rib_pitch_chord_frac must be > 0 or None, got {!r}.".format(
                    label, self.rib_pitch_chord_frac
                )
            )
        drift_mm_set = self.z_drift_mm_per_mm is not None
        drift_frac_set = self.z_drift_chord_frac_per_mm is not None
        if drift_mm_set and drift_frac_set:
            raise SpecError(
                "{} has both z_drift_mm_per_mm={!r} and "
                "z_drift_chord_frac_per_mm={!r} set; set at most one -- "
                "z_drift_mm_per_mm is an absolute (constant-angle) drift "
                "rate, z_drift_chord_frac_per_mm is a fraction of local "
                "chord per mm of Z (constant chord-proportion), and a "
                "single family cannot be both at once.".format(
                    label, self.z_drift_mm_per_mm, self.z_drift_chord_frac_per_mm
                )
            )
        if self.web_depth_mm is not None and self.web_depth_mm <= 0:
            raise SpecError(
                "{} web_depth_mm must be > 0 or None, got {!r}.".format(
                    label, self.web_depth_mm
                )
            )
        if not 0.0 <= self.web_position <= 1.0:
            raise SpecError(
                "{} web_position must be within [0, 1] (fraction across the "
                "chord axis), got {!r}.".format(label, self.web_position)
            )
        if self.z_period_mm is not None and self.z_period_mm <= 0:
            raise SpecError(
                "{} z_period_mm must be > 0 or None, got {!r}.".format(
                    label, self.z_period_mm
                )
            )
        if self.z_period_mm is not None and not drift_mm_set and not drift_frac_set:
            raise SpecError(
                "{} sets z_period_mm={!r} without z_drift_mm_per_mm or "
                "z_drift_chord_frac_per_mm; a period needs a slope to "
                "reverse. Set one of the two drift fields, or leave "
                "z_period_mm None for unbounded linear drift.".format(
                    label, self.z_period_mm
                )
            )
        if self.pair_offset_mm is not None and self.pair_offset_mm <= 0:
            raise SpecError(
                "{} pair_offset_mm must be > 0 or None, got {!r}.".format(
                    label, self.pair_offset_mm
                )
            )
        if self.socket_flank_offset_mm is not None and self.socket_flank_offset_mm < 0:
            raise SpecError(
                "{} socket_flank_offset_mm must be >= 0 or None, got {!r}."
                .format(label, self.socket_flank_offset_mm)
            )
        socket_flank_active = (
            self.socket_flank_offset_mm is not None
            and self.socket_flank_offset_mm > 0
        )
        if socket_flank_active and self.socket_flank_direction not in (
            "forward", "aft"
        ):
            raise SpecError(
                "{} socket_flank_offset_mm={!r} is active (> 0) but "
                "socket_flank_direction={!r} is not 'forward' or 'aft' -- "
                "direction is required whenever the offset is active."
                .format(label, self.socket_flank_offset_mm,
                        self.socket_flank_direction)
            )
        if not socket_flank_active and self.socket_flank_direction is not None:
            raise SpecError(
                "{} sets socket_flank_direction={!r} without an active "
                "socket_flank_offset_mm (unset or 0) -- direction has no "
                "effect without an offset to apply it to. Set "
                "socket_flank_offset_mm > 0, or leave "
                "socket_flank_direction None.".format(
                    label, self.socket_flank_direction
                )
            )
        # ``socket_flank_offset_mm`` overrides ONLY ``anchor_x`` in
        # ``StructureGenerator._generate_family`` -- it does not disable pitch.
        # A family that also sets ``rib_pitch_mm``/``rib_pitch_chord_frac``
        # still reaches ``_candidate_stations(frame, pitch_mm, anchor_x)``,
        # which marches the WHOLE chord from that anchor at the requested
        # pitch, landing extra stations wherever the march takes them --
        # including inside the socket's own footprint exclusion, where they are
        # omitted silently (a real ``OmittedWeb``, but not what "single
        # station" promised). To keep "this family's SINGLE station" true, an
        # active offset combined with either pitch field is refused outright:
        # a socket-flank family is a named single-station placement, not a
        # marching pattern anchored on the socket.
        if socket_flank_active and (
            self.rib_pitch_mm is not None or self.rib_pitch_chord_frac is not None
        ):
            raise SpecError(
                "{} sets socket_flank_offset_mm={!r} (active) together with "
                "rib_pitch_mm={!r} / rib_pitch_chord_frac={!r} -- "
                "socket_flank_offset_mm places exactly ONE station (this "
                "family's anchor, derived from the detected socket); a "
                "pitch would then march additional stations from that "
                "anchor across the whole chord, which is not what a "
                "socket-relative family is for. Leave both pitch fields "
                "None, or use a separate ordinary family for a marching "
                "pattern.".format(
                    label, self.socket_flank_offset_mm, self.rib_pitch_mm,
                    self.rib_pitch_chord_frac,
                )
            )
        if self.role not in VALID_ROLES:
            raise SpecError(
                "{} role={!r} is not one of {} -- role must be EXPLICIT and "
                "one of the two declared structural intents (see "
                "WebFamily.role's own docstring); it is never inferred from "
                "web_depth_mm or any other field.".format(
                    label, self.role, VALID_ROLES
                )
            )

    def station_drift_mm(
        self, print_z: float, local_chord_mm: Optional[float] = None
    ) -> float:
        """This family's chord (X) drift in millimetres at layer height
        ``print_z``. Added once to the family's anchor station before
        marching by ``rib_pitch_mm`` (see ``StructureGenerator``'s
        anchoring rule) — every web in the family shifts together, which is
        what turns a row of parallel shear panels into a row of parallel
        DIAGONAL members as Z rises.

        Computed directly from ``print_z`` — never accumulated layer over layer
        — so the same Z always yields the same drift regardless of construction
        order or how many layers preceded it (determinism).

        Modes:

        - ``z_drift_mm_per_mm is None``: no variation. Returns ``0.0``
          always.

        - ``z_period_mm is None`` (drift set, no period): plain linear
          drift, ``drift = z_drift_mm_per_mm * print_z``. Unbounded: a tall
          enough part walks the station off the chord entirely, and every
          station past the extent is omitted exactly like any other
          out-of-extent candidate (see ``StructureGenerator``'s omission
          policy) — never clamped back onto the chord.
        - ``z_period_mm`` set (both fields set, enforced by ``validate()``):
          a symmetric TRIANGLE WAVE in Z, period ``z_period_mm``, amplitude
          ``A = z_drift_mm_per_mm * z_period_mm / 4`` — chosen so the
          wave's slope magnitude over each straight run is exactly
          ``z_drift_mm_per_mm``, the same rate the unbounded linear mode
          uses. Letting ``t = print_z mod z_period_mm``:

              t <  z_period_mm / 2:  drift = -A + z_drift_mm_per_mm * t
              t >= z_period_mm / 2:  drift =  A - z_drift_mm_per_mm * (t - z_period_mm / 2)

          This is continuous, reverses direction (slope sign flips) at
          every half-period boundary, and keeps the station oscillating
          within ``anchor +/- A`` forever rather than walking off the chord
          — THE property that makes a zigzag usable as a structural
          pattern rather than a one-way drift that eventually omits every
          station. Note ``drift(print_z=0) == -A``, not ``0.0``: the wave
          is symmetric about the anchor, not anchored to start at zero, so
          the bottom layer of a zigzagging family starts at the low end of
          its swing rather than at dead centre.

        CAP ON AMPLITUDE, and the CROSSING CONDITION for two opposed
        families (see tests/unit/test_lattice_crossing.py for the full
        derivation and a regression fixture): setting ``z_period_mm`` does not just bound
        the drift, it CAPS the amplitude at ``|z_drift_mm_per_mm| *
        z_period_mm / 4`` — the unbounded-linear mode's rate is only ever
        realized as an instantaneous slope, never accumulated past that cap.
        A period picked to be "generous" can therefore still be too small
        to produce the swing a designer expects; the amplitude scales
        LINEARLY with the chosen period at fixed slope, so doubling
        ``z_period_mm`` doubles the swing.

        For two families anchored at chord fractions ``p_a < p_b`` on a
        section of width ``W``, sharing ``z_period_mm`` and rib pitch ``R``,
        with EQUAL and OPPOSITE ``z_drift_mm_per_mm`` (the "diamond
        lattice" case — this is what makes the two drift waves exact
        negatives of each other, since every term of this formula is
        linear in ``z_drift_mm_per_mm``): the anchor-pair (march offset
        n=0) chord separation is ``C - 2*d(z)`` where ``C = (p_b - p_a) *
        W`` and ``d(z)`` is one family's own drift, ranging continuously
        over ``[-A, +A]``. The anchor pair's stations swap chord order
        (a true crossing, not mere approach) iff ``2*A > C`` — i.e. iff
        ``z_period_mm > 4*C / (2*|z_drift_mm_per_mm|)`` — and are exactly
        tangent (touch zero separation without swapping) at equality.

        Rib pitch changes this by INTERLEAVING: the generator marches every
        integer offset ``k`` (``StructureGenerator``'s anchoring rule), so
        the anchor pair (n=0) is not necessarily the closest cross-family
        pair. Offset ``n = k_b - k_a`` shifts the same-shaped separation
        window by ``n*R``: pair ``n``'s window is ``[C + n*R - 2*A, C +
        n*R + 2*A]``, and the TRUE crossing condition is the minimum over
        every integer ``n`` of ``|C + n*R|`` compared against ``2*A`` — a
        small rib pitch relative to ``C`` can bring an ``n != 0`` pair
        closer to zero separation than the anchors ever get, and can do so
        even when the anchor pair alone would not cross. Worked example:
        ``fwd``/``aft`` at 0.4/0.6, ``|z_drift_mm_per_mm|=0.8``,
        ``z_period_mm=7.5``, ``rib_pitch_mm=8.0`` on a 20 mm coupon has
        ``C=4.0``, ``A=1.5``; both the ``n=0`` and ``n=-1`` pairs tie at
        ``|C + n*R| = 4.0 > 2*A = 3.0`` mm, so the closest approach is exactly
        1.0 mm and the families never cross, though a config like this may be
        written believing they would.

        RELATIVE MODE (``z_drift_chord_frac_per_mm``): ``local_chord_mm``,
        when given, resolves an EFFECTIVE ``z_drift_mm_per_mm`` for this call
        as ``z_drift_chord_frac_per_mm * local_chord_mm`` and otherwise runs
        the formula above -- ``z_period_mm`` stays an absolute Z distance in
        both modes. Callers pass THIS LAYER's own local chord
        (``frame.width``, freshly measured every layer), never a
        cached/anchor-Z value, so the resolved rate varies along a tapering
        span -- see the field comment after ``role`` for why that is
        deliberate: absolute mode holds the printed member ANGLE constant;
        this mode holds the trajectory's CHORD FRACTION constant instead, and
        the two cannot both hold on a taper.

        THE CROSSING DERIVATION ABOVE ASSUMES CONSTANT ``A``/``C``/``R`` and
        does NOT apply term-by-term when this method is called in relative
        mode with a Z-varying ``local_chord_mm`` -- ``A`` (via the resolved
        rate) and, separately, any ``rib_pitch_chord_frac`` also resolved
        against the same varying chord are themselves functions of Z there,
        which the derivation's constant-coefficient algebra does not cover.
        What DOES hold, and is the point of relative mode (see the class
        docstring's "SCALE INVARIANCE" note): if EVERY term of the crossing
        inequality (``C``, ``A``, ``R``) is resolved against the SAME local
        chord at a given Z, they scale together and the inequality's truth
        value is invariant along the span -- a property this formula's
        constant-coefficient form does not need re-derived to exploit,
        precisely because ``local_chord_mm`` here is always THIS layer's own
        value, never a stale one.
        """
        z_drift_mm_per_mm = self.z_drift_mm_per_mm
        if z_drift_mm_per_mm is None and self.z_drift_chord_frac_per_mm is not None:
            if local_chord_mm is None:
                raise ValueError(
                    "family {!r} sets z_drift_chord_frac_per_mm={!r} but "
                    "station_drift_mm was called without local_chord_mm -- "
                    "relative drift cannot be resolved without this layer's "
                    "own local chord.".format(self.name, self.z_drift_chord_frac_per_mm)
                )
            z_drift_mm_per_mm = self.z_drift_chord_frac_per_mm * local_chord_mm
        if z_drift_mm_per_mm is None:
            return 0.0
        if self.z_period_mm is None:
            return z_drift_mm_per_mm * print_z
        amplitude = z_drift_mm_per_mm * self.z_period_mm / 4.0
        half_period = self.z_period_mm / 2.0
        t = print_z % self.z_period_mm
        if t < half_period:
            return -amplitude + z_drift_mm_per_mm * t
        return amplitude - z_drift_mm_per_mm * (t - half_period)

    def resolve_gap(self, spec: "StructureSpec", outer_wall_line_width_mm: float) -> float:
        """Resolve this family's slot width in millimetres.

        Uses the family's own ``slot_gap_mm``/``gap_ratio`` if either is
        set; otherwise falls back to ``spec.resolve_slot_gap`` (the spec's
        own gap mode). Call ``validate()`` first — this method does not
        re-check field consistency.
        """
        if self.slot_gap_mm is not None:
            return self.slot_gap_mm
        if self.gap_ratio is not None:
            if outer_wall_line_width_mm <= 0:
                raise SpecError(
                    "outer_wall_line_width_mm must be > 0 to resolve "
                    "family {!r} gap_ratio={!r}, got {!r}.".format(
                        self.name, self.gap_ratio, outer_wall_line_width_mm
                    )
                )
            return self.gap_ratio * outer_wall_line_width_mm
        return spec.resolve_slot_gap(outer_wall_line_width_mm)

    @property
    def is_marching(self) -> bool:
        """True if this family marches multiple stations, not one explicit.

        The single predicate for the MARCHING-vs-EXPLICIT-STATION split
        ``_candidate_stations`` implements (see ``generator.py``'s
        "EXPLICIT-STATION family" / "MARCHING family" section) and every
        other call site must agree with. A family marches when EITHER pitch
        field is set -- ``rib_pitch_mm`` (absolute) or
        ``rib_pitch_chord_frac`` (relative); ``validate()`` forbids both
        being set together, so this is never ambiguous.

        Testing ``rib_pitch_mm is not None`` directly would misclassify a
        chord-relative family: it has ``rib_pitch_mm is None`` by
        construction, so it would read as a single EXPLICIT station, and a
        depth/hole breach that a marching family would OMIT would instead
        raise a ``ValueError`` naming a "requested station" the config never
        named. This property does NOT need ``local_chord_mm`` -- whether a
        family marches is a static
        property of which field is set, not of the resolved pitch value
        (unlike ``resolved_rib_pitch_mm``, which needs the local chord to
        turn ``rib_pitch_chord_frac`` into millimetres and would raise
        without it).
        """
        return self.rib_pitch_mm is not None or self.rib_pitch_chord_frac is not None

    def resolved_rib_pitch_mm(self, local_chord_mm: Optional[float]) -> Optional[float]:
        """This family's rib pitch in millimetres, resolved for placement.

        Mirrors ``resolve_gap``'s shape: ``rib_pitch_mm`` (absolute), if
        set, wins outright. Otherwise, if ``rib_pitch_chord_frac`` is set, resolves it against
        ``local_chord_mm`` (THIS layer's own local chord -- see
        ``station_drift_mm``'s "RELATIVE MODE" note for why it must be
        fresh per layer, never cached). ``None`` if neither is set (the
        single-station family case).

        Raises ``ValueError`` if ``rib_pitch_chord_frac`` is set but
        ``local_chord_mm`` is not given -- the same "cannot resolve a
        relative quantity without this layer's own geometry" contract
        ``station_drift_mm`` enforces.
        """
        if self.rib_pitch_mm is not None:
            return self.rib_pitch_mm
        if self.rib_pitch_chord_frac is not None:
            if local_chord_mm is None:
                raise ValueError(
                    "family {!r} sets rib_pitch_chord_frac={!r} but "
                    "resolved_rib_pitch_mm was called without "
                    "local_chord_mm -- relative pitch cannot be resolved "
                    "without this layer's own local chord.".format(
                        self.name, self.rib_pitch_chord_frac
                    )
                )
            return self.rib_pitch_chord_frac * local_chord_mm
        return None


@dataclass
class StructureSpec:
    """Parameters for the fused webs a structure request produces.

    Exactly one of ``slot_gap_mm`` / ``gap_ratio`` must be set. The other stays
    ``None``; picking a default for the unset one would silently reintroduce a
    hard-coded slot width.

    Attributes:
        slot_gap_mm: Explicit slot width in millimetres, or ``None`` to
            derive it from ``gap_ratio`` at resolve time.
        gap_ratio: Slot width as a multiple of the resolved outer wall line
            width, or ``None`` if ``slot_gap_mm`` is given directly.
        skin_margin_mm: Minimum remaining skin thickness the slot must stop
            short of at its far end (millimetres). PRECONDITION:
            ``skin_margin_mm > 2 * outer_wall_line_width``. The topology
            validator erodes by the outer wall width to detect a neck too thin
            to survive perimeter offsetting, so a margin below twice that
            cannot survive its own check — every layer would fail. When the
            disconnect error fires, the fix is a LARGER margin, not a smaller
            one.
        root_solid_layers: Number of body layers, from the BED (layer 0),
            that must remain solid (no slot). This is a bed-adhesion guard
            (the first layers are where a slot would most easily lift), NOT
            an anatomical-root guard. On a part printed tip-face-down
            (``root_end="top"``) it protects the TIP end at the bed and gives
            the true root, at the top of the print, no such protection. The
            generator does not act on this field beyond carrying it: the
            capability applies it per body layer.
        web_position: Fraction across the chord axis (0 to 1) locating the web
            centerline. 0.5 is the coupon centerline. This is the anchor:
            additional webs are placed at ``web_position +/- k *
            rib_pitch_mm`` (see ``generator.py``'s anchoring rule).
        overshoot_mm: Distance the slot is extended past the exterior
            boundary before subtraction, guaranteeing a clean cut through
            the skin.
        rib_pitch_mm: Spacing between webs along the chord axis, in
            millimetres, or ``None`` for a single web (only the web at
            ``web_position`` is produced). When set, ``StructureGenerator``
            places one web at ``web_position`` and marches outward both ways at
            this pitch (see ``generator.py``'s anchoring rule).
        web_depth_mm: How far a web reaches in from the LOCAL surface it
            enters, measured along the web centerline, or ``None`` for full
            depth (the web runs to ``skin_margin_mm`` short of the far
            surface). When set, the
            web stops ``web_depth_mm`` in from the entry surface instead —
            this is what turns a full-depth shear panel into a shallow
            skin-stiffening web. Still measured against the same worst-case
            LOCAL surface ``StructureGenerator`` already uses for margins
            (not a bounding-box approximation), so a shallow web on a
            curved section follows the local surface rather than cutting to
            a fixed global depth. A requested depth that would leave less
            than ``skin_margin_mm`` before the far surface fails closed
            (``ValueError``, naming the family) rather than silently
            degrading into a through-cut — see ``StructureGenerator``.
            Only used for the single implicit family when ``families`` is
            empty; each entry of ``families`` carries its own
            ``web_depth_mm`` instead (see ``WebFamily``).
        families: Zero or more ``WebFamily`` entries, each a group of webs
            sharing pitch, depth and phase — the mechanism for a two-tier
            structure (e.g. shallow skin webs at a tight pitch plus deep
            ribs at a wider pitch, produced together on every layer).
            ``StructureGenerator`` produces the UNION of every family's
            webs. Empty (the default) means a single implicit family: one
            family built from this spec's own
            ``rib_pitch_mm``/``web_depth_mm``/``web_position`` and gap mode
            (see ``effective_families``).
        z_drift_mm_per_mm: Top-level convenience for the single implicit
            family — see ``WebFamily.z_drift_mm_per_mm``. Only used when
            ``families`` is empty; each entry of ``families`` carries its
            own value instead, like ``rib_pitch_mm``/``web_depth_mm``.
        z_period_mm: Top-level convenience for the single implicit family —
            see ``WebFamily.z_period_mm``. Only used when ``families`` is
            empty.
        tip_shutdown_enabled: Pattern shutdown. When every candidate web
            on a layer was omitted, and every one of those omissions is a
            THINNESS reason (``"skin margin:..."`` or ``"thin section:..."``
            -- see ``generator.py``'s ``OmittedWeb.reason`` docstring; never
            ``"chord extent:..."``), ``True`` leaves that layer's surfaces
            unmutated and records it as a shutdown layer instead of failing
            the whole slice. ``False`` (the default) keeps fail-closed
            behaviour: a layer with zero surviving webs is always a fatal
            ``TopologyError``, regardless of why every candidate was omitted.

            Default OFF, deliberately: this is a narrowly scoped exception
            to the fail-closed mutation protocol (no path leaves a layer
            unmutated and carries on), not a general degrade-and-continue
            lever, and opting into "the last N layers of this wing print as a
            plain shell" must be a decision a user makes on purpose, not a
            default that silently changes what a tapered/tipped wing prints.
            A layer that shuts down is still recorded loudly (per-layer in
            diagnostics, and aggregated as a shutdown count + first
            shutdown Z in the run report).
        per_station_reentrant: (default False.) True omits a single station
            whose measurement raises ``ReEntrantSectionError`` (reason
            ``re_entrant_station``) instead of shutting the whole layer down.
        tip_alignment_skip_enabled: A DIFFERENT cause from
            ``tip_shutdown_enabled`` above -- that flag is for a tip that ran
            out of MATERIAL to carry a pattern; this one is for a tip layer
            where the mesh-to-slice alignment guard's CENTROID terms disagree
            while its four bbox terms all still agree, within one layer height
            of the mesh's own TOP Z. The mechanism is not established:
            ``slice_closing_radius`` is measured-unsupported (~4.5e-9 mm
            across chords 130 -> 1 mm on this repo's boolean backend);
            ordinary sub-layer sensitivity is in tension with the size of the
            observed jump; and tip tessellation / vertex-on-plane divergence
            between this project's sectioner and libslic3r's
            ``TriangleMeshSlicer`` has not been measured. This flag is a
            bounded, opt-in MITIGATION for that unresolved case, not a fix: it
            does not touch the four bbox terms (a genuinely wrong transform
            still fails closed, tip or no tip) and only applies within one
            layer height of the mesh's top Z, never the root (the root is
            normally already excluded by ``root_solid_layers``). ``False``
            (the default) keeps fail-closed behaviour: a centroid-only
            disagreement at the tip is a fatal ``TopologyError``, worded to
            name this flag. ``True`` leaves that one layer unmutated (no
            socket cut from an unverified cross-section) and records it
            loudly instead of failing the whole slice -- never conflated with
            ``tip_shutdown_enabled``, which is recorded with a different
            ``extra["reason"]``.
        max_rotation_deg: Rotation envelope. Upper bound, in degrees
            (either sign), on a section's own rotation about the chord axis
            (washout) that ``StructureGenerator``/``LayerFrame.web_centerline``
            will accept before failing closed, or
            ``None`` to use ``aircraft_core.frame.DEFAULT_MAX_ROTATION_DEG``.
            See that constant's comment for the measured overstatement
            table (``local_span`` reads thickness along world Y, which
            overstates true chord-perpendicular thickness by
            ``1/cos(theta)`` on a rotated section) and why 30 degrees is
            the chosen default. Below this threshold, margin checks are
            corrected for the rotation (``frame.corrected_skin_margin_mm``);
            past it, generation fails closed naming the measured angle and
            the threshold rather than trust an unvalidated correction.
        expected_hole_count: How many interior holes (e.g. a spar tube) this
            section is DECLARED to carry. ``0`` (the default): ``StructureGenerator``
            never queries ``ops.hole_bounds``, never builds a scar, and
            ``TopologyValidator``'s invariant 2 and per-slot measurements apply
            unmodified -- an interior hole found in this regime is
            unconditionally the defect invariant 2 exists to catch. A positive
            value is the LICENCE that makes relaxing that invariant safe: it
            tells the generator "this section is expected to carry exactly N
            interior holes, connect each one to the exterior boundary with a
            scar" and tells the validator "a hole here is not automatically a
            defect -- verify it was actually declared and actually connected,
            instead of just refusing its presence outright". A geometry hole
            count that disagrees with this declared number is ITSELF the defect
            and fails closed naming the mismatch (see
            ``StructureGenerator._generate_scar``). Capped at ``1`` here: the
            scarring proof is scoped to exactly one hole per layer and does not
            cover N>1, so a config assuming the unproven extension is refused.
        web_tilt_threshold_deg: Below this off-normal error, in degrees, a
            web's entry stays exactly vertical -- no tilt correction is even
            computed. ``None`` (the default) uses
            ``aircraft_core.frame.DEFAULT_TILT_THRESHOLD_DEG``. Mirrors
            ``max_rotation_deg``'s "None means the built-in default" shape.
            This field alone changes nothing while ``max_web_tilt_deg`` is
            ``0.0`` (the default) -- see that field's docstring.
        max_web_tilt_deg: Upper bound, in degrees, on how far a web's entry
            may tilt away from vertical toward the local surface normal.
            ``0.0`` (the default) is a hard, structural no-op: every web is
            placed vertically regardless of ``web_tilt_threshold_deg`` -- the
            generator never even measures a local slope when this is ``0.0``
            (see ``StructureGenerator._generate_family``). A positive value
            enables the correction ONLY where the untilted off-normal error
            exceeds ``web_tilt_threshold_deg``, and even then clamps the
            APPLIED tilt to at most this many degrees -- deliberately not
            "always tilt to the true normal": a fully perpendicular entry near
            the leading edge WIDENS the web's X footprint (the surface is
            steepest, hence nearly-horizontal entry) exactly where the
            thin-section/collision checks are already tightest, so an
            unbounded correction there would cost LE coverage rather than buy
            it. The generator independently re-verifies the tilted far end
            against the actual local surface at its own (shifted) station
            before accepting it (see ``_generate_family``'s comment) and falls
            back to the vertical placement -- never a build failure -- if that
            verification fails, so a station this correction cannot help is
            never worse off than without it.
        allow_config_spar_fallback: ``False`` (the default) means a layer with
            no MODEL-detected spar channel
            (``generator.LayerContext.detected_channels``) but a declared
            ``spar_diameter_mm`` fails closed, naming this flag, instead of
            silently synthesising the socket from config alone -- config-declared
            spar geometry is a debugging fallback only, never the normal path,
            once the model can supply the real channel. Setting ``True`` allows
            the config-only socket, and every layer where it actually fires is
            recorded on ``GenerationResult.spar_fallback_used`` so it is never
            silent. Irrelevant, either way, once ``detected_channels`` is
            non-empty -- the model always wins over config when it has an
            answer (see ``StructureGenerator.generate``'s docstring for the
            full precedence).
    """

    slot_gap_mm: Optional[float] = None
    gap_ratio: Optional[float] = None
    skin_margin_mm: float = 0.6
    root_solid_layers: int = 0
    web_position: float = 0.5
    overshoot_mm: float = 0.05
    rib_pitch_mm: Optional[float] = None
    web_depth_mm: Optional[float] = None
    families: Tuple[WebFamily, ...] = ()
    z_drift_mm_per_mm: Optional[float] = None
    z_period_mm: Optional[float] = None
    tip_shutdown_enabled: bool = False
    # A distinct opt-in from ``tip_shutdown_enabled``, gating the tip-only
    # centroid-alignment skip rather than pattern shutdown. Default OFF for the
    # identical reason: this changes what a wing's last layer(s) print, and
    # must be a decision a user makes on purpose.
    tip_alignment_skip_enabled: bool = False
    # When True, a family whose ``role`` is ``primary_structure`` that gets
    # OMITTED fails the slice instead of degrading quietly.
    #
    # WHY THIS EXISTS. Omitting an unplaceable web is correct, fail-closed
    # behaviour and the resulting layer is genuinely valid -- for a SKIN
    # LATTICE, one stiffener fewer is a small degradation. For PRIMARY
    # STRUCTURE it is not: a user asks for a rear spar, does not get one, and
    # every check passes. Measured: a two-spar config at 0.42 mm line width
    # sweeps VERIFIED 595/595 with ``spar_rear`` omitted at EVERY station.
    # A spar's PRESENCE cannot be inferred from a passing slice.
    #
    # DEFAULTS FALSE, deliberately. Turning it on unconditionally would change
    # the behaviour of every config that carries a ``primary_structure``
    # family, which is a decision about shipped defaults and belongs to the
    # user. Off, this field changes nothing anywhere; on, it is the guard. Same
    # opt-in shape as ``expected_hole_count``.
    require_primary_structure: bool = False
    # When True, a ``ReEntrantSectionError`` raised while measuring ONE
    # candidate station (its entry/far surfaces, a merge re-measurement, a
    # socket-footprint thickness read) omits THAT station with reason
    # ``re_entrant_station`` and the layer keeps its other structure. ``False``
    # (the default): the exception propagates and the capability shuts the
    # whole layer down. Default off on purpose: it changes what a layer prints,
    # so enabling it is the user's decision. A genuinely re-entrant SECTION
    # (two lobes) is NOT distinguished from a notch by this flag.
    per_station_reentrant: bool = False
    # The Z-continuity rule. Default False. When True the capability / mesh
    # replay run ``aircraft_core/z_rule.py`` in the per-layer
    # loop: every web may deepen by at most one ramp step per layer from the
    # depth it actually executed on the previous layer (a web that is absent,
    # or whose slot side walls moved by more than ``z_continuity_dx_max_mm``,
    # restarts from zero), a station whose checks fail at the ramp depth is
    # bisected to the deepest depth that passes, the validator runs inside
    # the loop with single-web ablation (the culprit ENDs), and a whole layer
    # is shut down only for a genuinely re-entrant section. Forces
    # ``per_station_reentrant`` ON (see ``effective_per_station_reentrant``);
    # pair ablation and pinch are not used.
    z_continuity: bool = False
    # ``independent`` (default): the two members of an opposing-pair (tie)
    # station ramp, END and are ablated on their own. ``joint``: both members
    # share one ramp cap (the shallower of the two) and END together.
    z_continuity_tie_mode: str = "independent"
    # Largest slot side-wall X step between adjacent layers that still counts
    # as "the same web". ``None`` resolves to
    # ``aircraft_core.continuity.DEFAULT_DX_MAX_MM`` (derivation recorded there).
    z_continuity_dx_max_mm: Optional[float] = None
    # Wall-matching variant. Only read when ``z_continuity`` is True. True
    # (default): a printed slot wall is "supported" when it lies within
    # ``z_continuity_dx_max_mm`` of SOME printed wall of the same orientation
    # (left/right edge) and entry side on the previous layer, and the slot may
    # be at most ``min(over its walls: deepest such wall) + ramp`` deep; a
    # merge keeps its depth and a split's new inner walls ramp. False: the
    # strict per-web rule (each web's own two walls must stay within dx_max of
    # that web's own previous walls).
    z_continuity_wall_matching: bool = True
    # --- carbon-spar socket -------------------------------------------
    # The wing carries an EXTERNAL carbon tube installed after printing; the
    # printed structure is secondary. Spiral Vase deletes interior holes before
    # posSlice, so a channel modelled in CAD cannot reach the generator
    # directly: the socket is derived from the channel detected by sectioning
    # the mesh, or (debugging fallback) SYNTHESISED from a declared position.
    #
    # The duplication -- spar position in both CAD and config, with nothing
    # enforcing agreement -- is a real ergonomic cost and is deliberate.
    #
    # ``spar_diameter_mm`` None (the default) disables the whole feature.
    spar_diameter_mm: Optional[float] = None
    spar_station: float = 0.30          # chord fraction
    spar_clearance_mm: float = 0.15     # radial slip fit for the tube
    spar_neck_width_mm: float = 1.2     # the boundary connection
    # Which of MORE THAN ONE detected channel at a single layer is the spar --
    # e.g. a wing tip where two Ø3mm alignment-rod bores coexist with each
    # other (but never with the real spar channel) and both get picked up by
    # mesh-sectioning as "a channel". The model alone cannot answer "which
    # one is the spar"; this is deliberately a POLICY field, read only when
    # ``StructureGenerator.generate`` sees more than one channel on the same
    # layer -- irrelevant, either way, whenever 0 or 1 channels are detected.
    #
    # "refuse" (the DEFAULT): ``generate()`` raises ``TopologyError`` naming
    # every detected channel's centre and diameter, never guessing.
    #
    # "nearest_spar_station": the channel whose chordwise (X) position is
    # nearest ``spar_station`` (measured as a chord fraction against THIS
    # layer's own frame, the same station ``_generate_spar_socket`` itself
    # uses) is declared the spar; every OTHER detected channel on the same
    # layer is a plain bore.
    #
    # THIS MAKES ``spar_station`` LOAD-BEARING FOR CHANNEL SELECTION. Otherwise
    # the model is the source of truth for spar-socket geometry and
    # ``spar_station`` is never consulted once a channel is detected -- see
    # clause (a) of ``generate()``'s docstring: "``spec.spar_diameter_mm``/
    # ``spec.spar_station`` are not consulted at all in this branch". That
    # remains true for POSITION/SIZE whenever 0 or 1 channels are detected (the
    # ordinary case) and even here, once a channel is chosen, for what the
    # socket looks like (radius, centre, entry side all still come from the
    # DETECTED channel's own centroid/diameter, never from config).
    # ``spar_station`` is consulted ONLY to break the tie of WHICH detected
    # channel gets called the spar. A misconfigured ``spar_station`` under this
    # policy therefore cannot move or resize the socket, but it CAN pick the
    # wrong channel to call the spar.
    #
    # A plain bore still gets a socket -- an unsocketed interior loop is a
    # second closed loop and breaks the spiral-vase invariant just as surely as
    # an unsocketed spar channel would -- built by the same keyhole machinery
    # (``_generate_bore_socket_from_channel``, mirroring
    # ``_generate_spar_socket_from_channel``) and sharing this spec's
    # ``spar_clearance_mm``/``spar_neck_width_mm`` POLICY fields (both are
    # generic slip-fit/boundary-connection policy, not spar-specific), but
    # carrying NONE of the spar's own semantics: it is never read by a
    # ``depth_to_socket``/``socket_flank_offset_mm`` family (both read
    # ``layer.detected_channels``, which is narrowed to the CHOSEN spar channel
    # alone before the family loop runs -- see ``generate()``'s docstring),
    # and it never arms or is caught by ``capability.py``'s spar-socket latch,
    # which keys on the literal family name ``"spar_socket"`` (a bore's is
    # ``"bore_socket"``). A bore that does not fit at its station DEGRADES
    # (dropped, reported) exactly like a spar socket that does not fit -- never
    # silently discarded, never a reason to fail the whole slice.
    spar_channel_policy: str = SPAR_CHANNEL_POLICY_REFUSE
    max_rotation_deg: Optional[float] = None
    expected_hole_count: int = 0
    web_tilt_threshold_deg: Optional[float] = None
    max_web_tilt_deg: float = 0.0
    # ``StructureGenerator.generate`` prefers a spar channel DETECTED in the CAD
    # model (``LayerContext.detected_channels``, populated by the plugin's
    # mesh-sectioning -- see ``generator.DetectedChannel``) over this spec's
    # own ``spar_diameter_mm``/``spar_station``: the model is the source of
    # truth for spar-socket geometry. When no channel is detected and
    # ``spar_diameter_mm`` is set, the config-declared geometry is a DEBUGGING
    # FALLBACK ONLY, not a normal input -- using it silently would let config
    # geometry drift from the model without anyone noticing. ``False`` (the
    # default) fails the slice closed instead, naming this flag, whenever a
    # config declares a spar but the model shows no channel for it. Setting
    # this to ``True`` is an explicit decision to trust config geometry anyway
    # (e.g. while mesh-sectioning is still being wired up, or for a coupon
    # fixture that never had a modelled channel) --
    # ``GenerationResult.spar_fallback_used`` records every layer where the
    # fallback actually fired, so it can never be silent even once enabled.
    allow_config_spar_fallback: bool = False
    # --- non-aerofoil FEATURE regions ----------------------------------
    # A layer whose whole cross-section is not an aerofoil (a joiner post, a
    # boss, a tab...) gets a wholly different placement rule: a tight
    # alternating-entry lattice sized to THAT REGION'S OWN EXTENT, never the
    # globally marched aerofoil stations.
    # ``StructureGenerator.generate`` classifies via
    # ``aircraft_core.frame.classify_region`` using these two thresholds -- see
    # the comment above ``DEFAULT_FEATURE_ASPECT_RATIO_MAX`` in ``frame.py``
    # for the measured separation the defaults rest on. ``None`` (the default)
    # resolves to ``aircraft_core.frame.DEFAULT_FEATURE_ASPECT_RATIO_MAX`` /
    # ``DEFAULT_FEATURE_FILL_RATIO_MIN`` -- the same "None means use frame.py's
    # own default" shape ``max_rotation_deg`` uses, deliberately NOT a literal
    # copied here: this module is pure data with no geometry dependency, and
    # ``frame.py`` is concatenated AFTER this module in
    # ``tools/build_plugin.py``'s ``CORE_MODULE_ORDER`` -- a literal
    # cross-module default would be a plain ``NameError`` in the built bundle
    # (a dataclass field default is evaluated at class-definition time, before
    # ``frame.py``'s constants exist; ``tests/unit/test_bundle.py`` catches
    # this).
    feature_aspect_ratio_max: Optional[float] = None
    feature_fill_ratio_min: Optional[float] = None
    # Spacing, in millimetres, between a FEATURE region's own lattice webs.
    # 2.0 mm: at the default settings (gap_ratio=0.5, wall 0.42mm -> slot
    # width 0.21mm, safety_offset 0.42mm) the minimum non-colliding pitch
    # for two alternating stations is slot width + 2*safety_offset = 1.05mm
    # (the same "two same-family footprints must clear by 2*safety_offset"
    # floor TopologyValidator enforces everywhere else); 2.0mm is roughly
    # double that floor, giving 3-4 members across an 8.6mm feature while
    # staying comfortably clear of the floor across the realistic range of
    # wall widths.
    feature_lattice_pitch_mm: float = 2.0
    # False (the default): ``_generate_feature`` bounds each tooth's depth by
    # ``min(frame.width/2 -..., frame.depth/2)``, a GLOBAL bound that (a)
    # never lets a tooth pass the section's own midline and (b) binds on the
    # feature's WIDTH even for a round/square feature where width says nothing
    # about how deep a tooth can safely go. True lets a feature's
    # alternating-entry teeth INTERDIGITATE past the midline, bounded instead
    # by the LOCAL depth at each tooth's own station (the same full-depth,
    # per-station measurement the aerofoil path uses for
    # ``web_depth_mm=None``) rather than a single global value sized for the
    # section's thinnest station. The far-side margin used to place each
    # tooth's tip is ``skin_margin_mm`` (corrected for rotation), NOT a
    # feature-specific outer-wall-derived margin:
    # ``TopologyValidator.validate``'s invariant 4 independently re-derives and
    # enforces ``skin_margin_mm``-based remaining material on every slot
    # regardless of what margin the generator used to choose the depth, so a
    # smaller margin here would only build teeth the validator then rejects --
    # see ``StructureGenerator._generate_feature``'s comment, at the site of
    # the rejected attempt, for the measured failure and the reasoning.
    # Opposing teeth cannot collide when this is enabled: the two synthetic
    # families are anchored exactly one ``feature_lattice_pitch_mm`` apart and
    # march at ``2 * feature_lattice_pitch_mm``, so every +Y and every -Y
    # candidate station is laterally offset from every station of the other
    # family -- and ``_omit_opposing_conflicts``/``TopologyValidator`` still
    # independently re-check every pair before and after the boolean, as
    # everywhere else.
    feature_dense_lattice_enabled: bool = False
    # --- mesh-to-slice alignment guard --------------------------
    # ``capability.py._check_mesh_alignment`` compares the plugin's own section
    # frame against Orca's sliced cross-section on bbox extent (4 terms) and
    # centroid (2 terms) and fails the slice closed, naming the layer index, Z
    # height, and worst offending term, whenever any term disagrees by more
    # than this many millimetres -- the guard that catches a genuinely wrong
    # mesh-to-slice transform BEFORE it places a spar socket from geometry that
    # is not what is actually printed. 0.02 mm (the default) is 15-20x the
    # measured clean-run noise floor (~0.001 mm).
    #
    # Raising this value does not make a misaligned mesh safe: it trades away
    # exactly the protection above, letting a larger transform error through in
    # exchange for being able to slice at all. It is not a tuning knob for "my
    # wing is slightly off" -- a wing that needs this raised has a
    # mesh-to-slice transform problem that this field only hides, and the
    # socket it produces is placed from geometry that may not match what
    # prints. A measured case is a layer 564 disagreement of 0.02479 mm (24%
    # over the default): the field exists so that decision -- accept the
    # reduced protection to proceed, or fix the transform -- is made
    # explicitly, never by a default that changes silently.
    mesh_alignment_tolerance_mm: float = 0.02
    # Off by default. When True, ``capability.py._check_mesh_alignment``
    # widens ``mesh_alignment_tolerance_mm`` PER TERM, for a term already over
    # base tolerance, to ``min(ceiling, max(base, K * |d(term)/dz| *
    # sectioner.epsilon_mm))`` -- but ONLY when the observed disagreement's
    # sign agrees with the sign the mechanism itself predicts (see
    # ``capability.py``'s module comment above
    # ``MESH_ALIGNMENT_SENSITIVITY_K`` for the full derivation, the sign
    # convention, and why a rigid mistranslation is NOT admitted by this
    # widening even though it can move a high-rate term by a similar raw
    # magnitude). With ``False``, every comparison uses the base tolerance --
    # see ``tests/integration/test_m18_mesh_transform.py``'s
    # ``MeshAlignmentSensitivityTests`` for the pinned proof.
    #
    # This does not replace ``mesh_alignment_tolerance_mm``: it is a
    # correction for a MEASURED, understood artifact of comparing two
    # different slicers' cross-sections of a fast-closing tip (a
    # ``MeshSectioner``-vs-``libslic3r`` sub-micron plane disagreement,
    # amplified ~100x by a real closing rate) -- not a general "my wing is
    # slightly off" knob, and it never touches a term whose disagreement
    # does not match that mechanism's own predicted sign.
    mesh_alignment_sensitivity_enabled: bool = False

    # When true, capability.py writes a gzip-JSON sidecar next to
    # m1_report.json recording, for every sectioned layer, the host's own slice
    # region polygons and our mesh section's guard loops/deltas/decision, so a
    # guard decision can be replayed offline against what the host actually
    # produced (tools/replay_host_sections.py) -- see config_schema.py for the
    # key's description.
    #
    # Default FALSE. Measured on a fake 1000-layer run: ~4.1 ms/layer marginal
    # cost (14.15 -> 18.24 ms/layer, ~29% over the no-sidecar baseline) and
    # ~1.8 MB gzip-compressed for 1000 layers. The size is small; the time cost
    # is comparable in order of magnitude to the run's OWN per-layer
    # mesh-sectioning cost (~2.9 ms/layer) -- not obviously negligible on a
    # real slice -- so this stays opt-in. Turn it on per run to collect a
    # corpus/replay entry for that slice.
    record_host_geometry_sidecar: bool = False

    # Which Z end of the PLACED mesh is the wing root -- "auto" (the
    # default: resolved from the mesh, see ``aircraft_core.root_end``),
    # "bed" (root at layer 0) or "top" (root at the highest layer, i.e. the
    # part prints TIP FACE DOWN). Only the spar-socket latch consumes it
    # (it must walk root -> tip); ``root_solid_layers`` deliberately stays
    # bed-relative (a bed-adhesion guard, see its docstring).
    root_end: str = "auto"

    def validate(self, safety_offset_mm: Optional[float] = None) -> None:
        """Raise ``SpecError`` with an actionable message if inconsistent.

        Does not raise on values that are merely unusual (e.g. a huge
        ``skin_margin_mm``); geometric infeasibility is the validator's job
        (``validate.py``), not the spec's — the spec only owns its own
        field-level contract.

        ``safety_offset_mm`` (keyword-optional): when given, guards the
        FRAGILITY the extended no-sever proof's connectivity argument is
        load-bearing on:

            skin_margin_mm > 2 * safety_offset_mm

        At the default settings that is 0.861 > 0.840 mm — a 0.021 mm cushion.
        The proof's chain argument relies on a SINGLE-sided cut's own remnant
        surviving the validator's safety-offset erosion ON ITS OWN,
        independent of anything on the opposite surface — that only holds
        because of this cushion. Below or at the threshold,
        ``TopologyValidator`` would still accept a layer whose single-sided
        remnant cannot survive its own erosion check in every case, and the
        failure would surface as plastic severing at print time, not a raised
        error. ``None`` (the default) skips this check entirely, so a caller
        that never passes ``safety_offset_mm`` is unaffected. Two call sites
        pass a concrete value: ``StructureGenerator.generate()`` (every layer,
        resolved from its own ``safety_offset_mm``/``outer_wall_line_width_mm``
        arguments) and ``plugin/aircraft_structures/capability.py``'s
        ``_build_spec`` (once, at config-build time, before any layer is
        generated, from the resolved outer wall line width) — see each call
        site's comment for why both fire rather than just one.
        """
        gap_set = self.slot_gap_mm is not None
        ratio_set = self.gap_ratio is not None
        if gap_set and ratio_set:
            raise SpecError(
                "StructureSpec has both slot_gap_mm={!r} and gap_ratio={!r} set; "
                "exactly one gap mode must be chosen. Set the other to None.".format(
                    self.slot_gap_mm, self.gap_ratio
                )
            )
        if not gap_set and not ratio_set:
            raise SpecError(
                "StructureSpec has neither slot_gap_mm nor gap_ratio set; "
                "provide an explicit slot width in mm, or a ratio of the "
                "outer wall line width."
            )
        if gap_set and self.slot_gap_mm <= 0:
            raise SpecError(
                "slot_gap_mm must be > 0, got {!r}.".format(self.slot_gap_mm)
            )
        if ratio_set and self.gap_ratio <= 0:
            raise SpecError(
                "gap_ratio must be > 0, got {!r}.".format(self.gap_ratio)
            )
        if self.skin_margin_mm < 0:
            raise SpecError(
                "skin_margin_mm must be >= 0, got {!r}.".format(self.skin_margin_mm)
            )
        if self.root_solid_layers < 0:
            raise SpecError(
                "root_solid_layers must be >= 0, got {!r}.".format(
                    self.root_solid_layers
                )
            )
        if self.z_continuity_tie_mode not in ("independent", "joint"):
            raise SpecError(
                "z_continuity_tie_mode must be 'independent' or 'joint', "
                "got {!r}.".format(self.z_continuity_tie_mode)
            )
        if not isinstance(self.z_continuity_wall_matching, bool):
            raise SpecError(
                "z_continuity_wall_matching must be a bool, got {!r}.".format(
                    self.z_continuity_wall_matching)
            )
        if self.z_continuity_dx_max_mm is not None and not (
            self.z_continuity_dx_max_mm > 0
        ):
            raise SpecError(
                "z_continuity_dx_max_mm must be > 0 or None, got {!r}.".format(
                    self.z_continuity_dx_max_mm)
            )
        if not 0.0 <= self.web_position <= 1.0:
            raise SpecError(
                "web_position must be within [0, 1] (fraction across the chord "
                "axis), got {!r}.".format(self.web_position)
            )
        if self.overshoot_mm <= 0:
            raise SpecError(
                "overshoot_mm must be > 0, got {!r} — a zero overshoot risks "
                "leaving a sliver instead of a clean cut through the skin "
                "(the slot must start exactly on the "
                "boundary).".format(self.overshoot_mm)
            )
        if self.rib_pitch_mm is not None and self.rib_pitch_mm <= 0:
            raise SpecError(
                "rib_pitch_mm must be > 0 or None, got {!r}. None reproduces "
                "the single-web behaviour; a non-positive pitch cannot "
                "space webs along the chord.".format(self.rib_pitch_mm)
            )
        if self.web_depth_mm is not None and self.web_depth_mm <= 0:
            raise SpecError(
                "web_depth_mm must be > 0 or None, got {!r}. None reproduces "
                "the full-depth behaviour; a non-positive depth "
                "cannot measure how far a web reaches in from the "
                "surface.".format(self.web_depth_mm)
            )
        if self.max_rotation_deg is not None and self.max_rotation_deg <= 0:
            raise SpecError(
                "max_rotation_deg must be > 0 or None, got {!r}. None uses "
                "aircraft_core.frame.DEFAULT_MAX_ROTATION_DEG; a "
                "non-positive threshold could never admit even an "
                "unrotated section.".format(self.max_rotation_deg)
            )
        if self.web_tilt_threshold_deg is not None and self.web_tilt_threshold_deg <= 0:
            raise SpecError(
                "web_tilt_threshold_deg must be > 0 or None, got {!r}. None "
                "uses aircraft_core.frame.DEFAULT_TILT_THRESHOLD_DEG; a "
                "non-positive threshold would apply the tilt correction "
                "everywhere, including where it has not been evaluated for "
                "safety.".format(self.web_tilt_threshold_deg)
            )
        if self.max_web_tilt_deg < 0:
            raise SpecError(
                "max_web_tilt_deg must be >= 0, got {!r}. 0.0 (the default) "
                "disables the perpendicular-entry correction "
                "entirely.".format(self.max_web_tilt_deg)
            )
        if self.z_period_mm is not None and self.z_period_mm <= 0:
            raise SpecError(
                "z_period_mm must be > 0 or None, got {!r}.".format(self.z_period_mm)
            )
        if self.z_period_mm is not None and self.z_drift_mm_per_mm is None:
            raise SpecError(
                "z_period_mm={!r} is set without z_drift_mm_per_mm; a "
                "period needs a slope to reverse. Set z_drift_mm_per_mm, or "
                "leave z_period_mm None for unbounded linear drift.".format(
                    self.z_period_mm
                )
            )
        if self.expected_hole_count < 0:
            raise SpecError(
                "expected_hole_count must be >= 0, got {!r}.".format(
                    self.expected_hole_count
                )
            )
        if self.expected_hole_count > 1:
            raise SpecError(
                "expected_hole_count={!r} exceeds 1. The spar-hole "
                "scarring proof "
                "is explicitly scoped to exactly one declared "
                "interior hole per layer; N>1 is a plausible but UNPROVEN "
                "extension. Shipping a config that assumes it would be "
                "exactly the untested generalization this project "
                "forbids.".format(self.expected_hole_count)
            )
        if self.spar_diameter_mm is not None:
            min_socket_diameter = self.spar_diameter_mm + 2.0 * self.spar_clearance_mm
            if self.spar_neck_width_mm >= min_socket_diameter:
                raise SpecError(
                    "spar_neck_width_mm={!r} must be < spar_diameter_mm "
                    "({!r}) + 2*spar_clearance_mm ({!r}) = {!r} mm; a neck "
                    "this wide has no chord across the socket circle and "
                    "would silently degrade to a plain rectangle instead of "
                    "a keyhole (SlotBuilder._keyhole). Shrink "
                    "spar_neck_width_mm, or grow spar_diameter_mm / "
                    "spar_clearance_mm.".format(
                        self.spar_neck_width_mm, self.spar_diameter_mm,
                        self.spar_clearance_mm, min_socket_diameter,
                    )
                )
        if self.feature_aspect_ratio_max is not None and self.feature_aspect_ratio_max < 1.0:
            raise SpecError(
                "feature_aspect_ratio_max must be >= 1.0 or None, got {!r}. "
                "None uses aircraft_core.frame."
                "DEFAULT_FEATURE_ASPECT_RATIO_MAX; aspect is "
                "max(width,depth)/min(width,depth), which is always >= 1 "
                "-- a threshold below 1 could never admit any "
                "section.".format(self.feature_aspect_ratio_max)
            )
        if self.feature_fill_ratio_min is not None and not (
            0.0 <= self.feature_fill_ratio_min <= 1.0
        ):
            raise SpecError(
                "feature_fill_ratio_min must be within [0, 1] (a fraction "
                "of the section's own bounding box) or None, got {!r}. "
                "None uses aircraft_core.frame."
                "DEFAULT_FEATURE_FILL_RATIO_MIN.".format(
                    self.feature_fill_ratio_min
                )
            )
        if self.feature_lattice_pitch_mm <= 0:
            raise SpecError(
                "feature_lattice_pitch_mm must be > 0, got {!r}.".format(
                    self.feature_lattice_pitch_mm
                )
            )
        if self.mesh_alignment_tolerance_mm <= 0:
            raise SpecError(
                "mesh_alignment_tolerance_mm must be > 0, got {!r}. A "
                "non-positive tolerance could never admit even a perfectly "
                "aligned mesh.".format(self.mesh_alignment_tolerance_mm)
            )
        if self.spar_channel_policy not in VALID_SPAR_CHANNEL_POLICIES:
            raise SpecError(
                "spar_channel_policy={!r} is not one of {!r}. \"refuse\" "
                "(the default) fails the slice closed, naming every "
                "detected channel, whenever more than one channel is "
                "detected on the same layer; \"nearest_spar_station\" "
                "declares the channel nearest spar_station the spar and "
                "sockets every other detected channel as a plain "
                "bore.".format(
                    self.spar_channel_policy, VALID_SPAR_CHANNEL_POLICIES
                )
            )
        if self.root_end not in ("auto", "bed", "top"):
            raise SpecError(
                "root_end={!r} is not one of 'auto', 'bed', 'top'. \"auto\" "
                "(the default) resolves the root end from the placed mesh; "
                "\"bed\" says the root prints at layer 0; \"top\" says the "
                "part prints tip-face-down with the root at the highest "
                "layer.".format(self.root_end)
            )
        for index, family in enumerate(self.families):
            family.validate(index)
        if safety_offset_mm is not None:
            min_wall_mm = 2.0 * safety_offset_mm
            if self.skin_margin_mm <= min_wall_mm:
                raise SpecError(
                    "skin_margin_mm={!r} does not exceed 2 * "
                    "safety_offset_mm={!r} ({!r} mm minimum wall). The "
                    "extended no-sever proof's connectivity argument "
                    "is "
                    "load-bearing on this inequality holding STRICTLY: a "
                    "single-sided cut's own remnant only survives the "
                    "validator's own safety-offset erosion because of the "
                    "cushion above 2 * safety_offset_mm. At or below it, the "
                    "failure is not a crash — it is a layer the validator "
                    "still accepts that severs in plastic. Raise "
                    "skin_margin_mm (never lower safety_offset_mm to "
                    "compensate).".format(
                        self.skin_margin_mm, safety_offset_mm, min_wall_mm
                    )
                )

    def effective_per_station_reentrant(self) -> bool:
        """``per_station_reentrant``, forced ON by ``z_continuity``: the rule
        needs a per-station re-entrant catch so only a genuinely re-entrant
        section shuts a whole layer down."""
        return bool(self.per_station_reentrant or self.z_continuity)

    def effective_families(self) -> Tuple[WebFamily, ...]:
        """The families ``StructureGenerator`` should actually produce.

        Returns ``self.families`` verbatim if non-empty. Otherwise returns a
        single synthetic family built from this spec's own top-level
        ``rib_pitch_mm``/``web_depth_mm``/``web_position`` fields, with its
        gap fields left ``None`` so ``WebFamily.resolve_gap`` falls back to
        ``self.resolve_slot_gap``, which makes "no families given" behave as
        a single family with the spec's own parameters.
        """
        if self.families:
            return self.families
        return (
            WebFamily(
                name="default",
                rib_pitch_mm=self.rib_pitch_mm,
                web_depth_mm=self.web_depth_mm,
                web_position=self.web_position,
                z_drift_mm_per_mm=self.z_drift_mm_per_mm,
                z_period_mm=self.z_period_mm,
            ),
        )

    def resolve_slot_gap(self, outer_wall_line_width_mm: float) -> float:
        """Resolve the effective slot width in millimetres.

        ``gap_ratio`` mode multiplies the caller-supplied resolved outer wall
        line width; ``slot_gap_mm`` mode ignores the argument entirely. Call
        ``validate()`` first — this method does not re-check field
        consistency.
        """
        if self.slot_gap_mm is not None:
            return self.slot_gap_mm
        if outer_wall_line_width_mm <= 0:
            raise SpecError(
                "outer_wall_line_width_mm must be > 0 to resolve gap_ratio={!r}, "
                "got {!r}.".format(self.gap_ratio, outer_wall_line_width_mm)
            )
        return self.gap_ratio * outer_wall_line_width_mm
