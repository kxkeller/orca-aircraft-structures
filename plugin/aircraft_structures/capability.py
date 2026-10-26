# /// script
# requires-python = ">=3.11"
# dependencies = []
#
# [tool.orcaslicer.plugin]
# name = "Aircraft Structures"
# description = "Cuts boundary-connected slots for each configured member family so the perimeter detours inward and back as one loop, printing internal webs and spar sockets in vase mode."
# author = "aircraft-orca-structures"
# version = "0.1.0"
# ///
"""Aircraft Structures — capability.

For each body layer of the enabled object, subtracts boundary-connected slots
for every configured member family (skin-lattice webs, spar flanks, tie pairs)
and a socket around any detected spar channel. Each slot makes the outer
perimeter detour in along one slot wall, around the end cap, and back out the
other, so the layer still prints as a single continuous loop whose two slot
walls fuse into an internal structural web.

This is the host edge only. All geometry decisions live in ``aircraft_core``,
which knows nothing about Orca: this file adapts Orca's scaled-integer,
Clipper-backed geometry to the millimetre-and-floats ``GeometryOps`` interface
the core expects, walks the slicing graph, and converts a core failure into a
slice-stopping error.

Most failures stop the slice: a layer that cannot carry the requested
structure fails it with the layer index, Z height and cause. The exceptions
are reported in the run report rather than degrading silently:

- Default-on degradations and rescues (see LIMITATIONS.md): a structural
  feature that cannot be placed is dropped and reported (Class B degradation,
  currently applied where a spar socket does not fit); opposing-pair ablation
  drops a web of a tie pair that would make a layer fail; a pinched section
  shuts the layer down to plain skin; the host's slice-closing radius is
  emulated on the mesh section inside the mesh-to-slice guard; ``root_end``
  ``"auto"`` resolves the root end, which also sets where the spar-socket latch
  starts; and the first ``root_solid_layers`` layers (5 by default) are left
  solid.
- ``tip_alignment_skip_enabled`` (opt-in, default off): a layer within one
  layer height of the mesh top whose bbox terms agree but whose centroid does
  not is left unmutated.
- ``tip_shutdown_enabled`` is superseded and inert: a layer whose every
  candidate web was omitted is left unmutated and reported whatever the flag
  says. The key is still accepted.

No third-party dependencies: per-layer bounds are read through
``Polygon.points`` (PluginHostGeometry.cpp:118-126). Deriving bounds from
``PrintObject.bounding_box()`` instead would need no coordinates at all, but it
is the whole-object XY box (PluginHostSlicing.cpp:317-321) — correct only for a
prismatic coupon and silently wrong for any tapered or swept section.
"""

import json
import math
import os
import time
import uuid

import orca

# --- arming ------------------------------------------------------------------
#
# The capability is inert unless a marker file sits beside the installed entry
# file. Arming needs no source edit, so the committed and installed copies stay
# byte-identical.
ARM_MARKER = "m1_enabled"

# Written into the plugin's own directory, which is inside data_dir() by
# construction and therefore inside the audit hook's allowed root.
DIAGNOSTICS_DIRNAME = "m1"

# The exact header line ``tools/build_plugin.py:234`` writes into the
# generated bundle -- matched verbatim by ``_read_bundle_info`` below so a
# report can say which bundle produced it.
_BUNDLE_HASH_LINE_PREFIX = (
    "# Bundle content hash (sha256 of imports + module bodies below): "
)

# Resolved Orca settings the transformation depends on.
CONFIG_KEYS = (
    "layer_height",
    "initial_layer_print_height",
    "outer_wall_line_width",
    "inner_wall_line_width",
    "line_width",
    "nozzle_diameter",
    "wall_generator",
    "wall_loops",
    "spiral_mode",
    "top_shell_layers",
    "bottom_shell_layers",
    "sparse_infill_density",
    "enable_support",
    "slice_closing_radius",
)

def _polygon_centroid(points):
    """Area-weighted polygon centroid, matching ``mesh_section._centroid``.

    Must agree in DEFINITION with the mesh side, or the two centroids are not
    comparable: the host's slice and the mesh cross-section tessellate the same
    curve differently, so a vertex average differs between them even when the
    geometry is identical. Falls back to the vertex average for a degenerate
    ring, exactly as the mesh side does.
    """
    n = len(points)
    a2 = 0.0
    cx = 0.0
    cy = 0.0
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        cross = x1 * y2 - x2 * y1
        a2 += cross
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    if abs(a2) < 1e-12:
        return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)
    return (cx / (3.0 * a2), cy / (3.0 * a2))


# mesh/host alignment limit, SET FROM MEASUREMENT with all six comparison terms
# live. Worst residual per term over 595 layers of the tapered/twisted
# reference wing, both CAD variants:
#
#     min_x       5.2e-06      max_x       4.0e-05
#     min_y       2.4e-03      max_y       2.3e-03
#     centroid_x  3.7e-03      centroid_y  1.9e-04
#
# A tolerance measured on the X terms alone would bound X only: the worst
# term (centroid_x) is ~90x larger than the max_x residual.
#
# 0.02 mm is ~5.4x the worst measured term and 21x tighter than the 0.42 mm
# extrusion width. It is tight enough to catch a 0.0251 mm half-layer error
# (sectioning at print_z instead of slice_z), which is the kind of transform
# error the comparison exists to detect.
#
# An instance-translation error (e.g. 128 mm) is not something this tolerance
# catches: _build_mesh_sectioner subtracts the mesh's own transformed-frame XY
# bbox centre, which cancels any instance translation by construction, so
# applying or omitting it is numerically indistinguishable.
#
# The centroid terms are the noisy ones: the host's slice and the mesh
# cross-section tessellate the same curve differently, and an area-weighted
# centroid is sensitive to that. They are not here for micron precision -- they
# are here because a Z-flip or a Y-mirror moves the centroid by tens of
# millimetres, which the bbox-edge terms alone can miss entirely.
MESH_ALIGNMENT_TOLERANCE_MM = 0.02

# Diagnostic-only. Cap on how many entries ``alignment_contour_dumps`` (see
# ``_check_mesh_alignment``) may hold -- full point-list dumps are not
# cheap the way six rounded floats are, so this stays a handful, not
# "every near-miss layer". A layer that actually EXCEEDS the tolerance
# (would raise, or would opt-in-skip) is never dropped for being past this
# cap -- see ``_check_mesh_alignment``'s own comment for why.
ALIGNMENT_CONTOUR_DUMP_CAP = 5

# --- mesh-alignment SENSITIVITY-AWARE tolerance (off by default) ------
#
# ``StructureSpec.mesh_alignment_sensitivity_enabled`` (config key
# ``mesh_alignment_sensitivity_enabled``, default ``False``) gates this
# entirely. ABSENT/``False`` gives the plain fixed-tolerance
# comparison unchanged (pinned by ``MeshAlignmentSensitivityTests`` in
# ``tests/integration/test_m18_mesh_transform.py``). It is a regular
# ``StructureSpec``/``config_schema`` field, like
# ``mesh_alignment_tolerance_mm`` itself.
#
# WHAT THIS MEASURES, AND WHY. At the extreme tip of a tapering wing, the
# host's own libslic3r slice and this project's ``MeshSectioner`` can
# legitimately disagree by tens of microns on a bbox-edge term even though
# both are correct sections of the same solid, because the two effective
# cutting planes differ by a FRACTION OF A MICRON and the section is closing
# so fast with Z that a sub-micron plane offset is amplified ~100x into a
# millimetre-scale term change. A FIXED absolute tolerance cannot
# distinguish "the plane moved a float's worth" from "the transform is
# wrong" once the local rate is this high -- it is measuring plane
# disagreement, not transform disagreement, wherever ``|d(term)/dz|`` is
# large.
#
# THE MECHANISM, MEASURED IN-HOST, NOT FITTED.
# ``MeshSectioner._section_at_exact`` (aircraft_core/mesh_section.py) always
# cuts at ``eff_z = z + epsilon_mm``, never at the raw requested ``z`` -- see
# that method's own docstring, item 2. On a section closing with increasing Z,
# that predicts ``delta_signed(term) = mesh_term - host_term`` and
# ``rate_signed(term) = d(term)/dz`` (BOTH signed, not absolute) share the same
# sign, and predicts the RATIO ``delta_signed / (rate_signed * epsilon_mm)``
# should sit close to 1. Measured directly on a real in-host run of `Part
# Studio 2 - Part 9(1).stl`, ``mesh_alignment_sensitivity_ enabled`` ARMED,
# DEFAULT 0.02 mm base tolerance (``mesh_alignment.sensitivity_adjustments``),
# five consecutive layers (998-1002), ``max_x`` term, this mesh's own
# ``epsilon_mm`` = 2.3924189e-04 mm (read from the SAME run, not a different
# reference mesh):
#
#     layer   rate (mm/mm)   delta (mm)   delta/(rate*epsilon)
#     998     105.9779       0.02568      1.013
#     999      87.4474       0.02145      1.025
#     1000     94.1791       0.02196      0.975
#     1001    124.4818       0.02940      0.987
#     1002    168.5557       0.04033      1.000
#
# Five ratios spanning a 2x range of rate (87-169 mm/mm), all within 2.5% of
# 1.0. That is the ``eff_z = z + epsilon_mm`` nudge measured directly, not a
# constant fitted after the fact to make a story work.
#
# K = 2. The measured ratios above already account for ~1x epsilon_mm of
# the expected disagreement; K=2 gives one full epsilon_mm of HEADROOM
# beyond the measured mechanism -- room for libslic3r's own vertex/rounding
# behaviour at the same plane, which this project cannot inspect from
# Python and has never measured directly. K=1 would sit exactly on the
# measured mechanism with no margin for anything on the host side this
# project cannot see; K=10 (this project's OWN ``EPSILON_SAFETY_FACTOR`` for
# epsilon_mm itself) would admit an equivalent-Z disagreement an order of
# magnitude past the measured mechanism, which stops looking like "the
# plane moved a float's worth" and starts looking like a real transform
# error being waved through. K=2 is the smallest integer that clears the
# measured, signed mechanism with any margin at all.
#
# THE SIGN GATE (closes a safety hole: widening on |delta| alone would also
# relax the tolerance for a genuine rigid mistranslation). The mechanism above predicts a SIGN, not just a
# magnitude: on a section closing with increasing Z, ``max_x`` shrinks
# (``rate_signed < 0``) and the mesh -- cut at ``z + epsilon_mm``, i.e.
# further along the closing direction -- reads SMALLER than the host
# (``delta_signed = mesh - host < 0``); ``min_x`` grows toward zero
# (``rate_signed > 0``) and the mesh reads LARGER (``delta_signed > 0``).
# Both cases: ``sign(delta_signed) == sign(rate_signed)``. A RIGID
# mistranslation does not respect this: shifting the host's polygon by
# ``+dx`` moves BOTH ``min_x`` and ``max_x``'s ``delta_signed`` by the SAME
# ``-dx`` -- so it moves WITH the mechanism's predicted sign on whichever
# edge is locally shrinking-with-Z, and AGAINST it on the other. Widening
# on |delta| alone would relax the tolerance on the matching edge for a
# genuine mistranslation, not just for the closing-tip mechanism it is meant
# for. ``_measure_term_rates`` therefore returns SIGNED rates, ``_check_mesh_alignment`` computes SIGNED deltas
# alongside the existing absolute ones, and a term is only ever widened
# when ``sign(delta_signed) == sign(rate_signed)`` (a zero rate never
# widens, matching ``K * 0 * epsilon == 0 <= base`` regardless). A term
# whose sign disagrees keeps ``base_tolerance`` exactly as if sensitivity
# were off. This does not close every rigid-shift exposure by itself --
# see ``MESH_ALIGNMENT_SENSITIVITY_CEILING_FALLBACK_MM`` below for the
# second half of the fix -- but it removes the half of the pair that
# manufactures a false pass from a mechanism that does not apply.
#
# PLANE UNCERTAINTY READS ``sectioner.epsilon_mm`` ONLY, never
# ``sectioner.last_nudge_mm`` (the dangling-endpoint RETRY offset) -- a
# fail-closed reading: the retry nudge is this run's own, already-applied
# correction for a DIFFERENT problem (a grazed vertex at THIS exact plane, see
# ``mesh_section.py``'s ``_RETRY_NUDGES_MM``), not part of the mesh/host
# plane-disagreement budget this tolerance is sizing. Folding it in would let
# an unrelated, already-compensated retry silently widen the alignment
# tolerance too. ``_measure_term_rates`` also SAVES and RESTORES
# ``sectioner.last_nudge_mm`` around its own two probe ``section_at`` calls, so
# this claim is true of WRITES as well as reads: a probe that happens to need
# the dangling-endpoint retry at ``z +/- probe_delta`` must not leave
# ``last_nudge_mm`` describing a probe plane instead of the layer's own
# ``applied_z`` for any code reading it afterwards (diagnostics dumps,
# ``_channels_for_layer``'s own bookkeeping on the NEXT layer).
#
# A DIFFERENT, UNRELATED FAILURE MODE MUST NOT BE RESCUED BY THIS DESIGN.
# A second mechanism, seen on a different wing (layer 1126 of wing "(7)"):
# the tip cap there is
# topologically UNSTABLE within about three layer heights of the mesh's
# true top -- the outer contour's signed area collapses NON-MONOTONICALLY
# AND CHANGES SIGN (1077 -> 911 -> -591 -> -202 mm2 over the last four
# layers), so "the largest |area| outer loop" flips between two entirely
# different loops for a sub-micron plane change (measured: 0.247 mm of
# centroid movement from a nudge three orders of magnitude smaller than the
# smooth-taper case above). That is NOT a well-defined local rate to
# extrapolate through -- it is a discontinuity -- and measuring
# ``d(term)/dz`` by central difference across it would return an
# effectively arbitrary number, not a physical closing speed. Feeding that
# into ``K * rate * epsilon_mm`` could manufacture an enormous, meaningless
# effective tolerance and pass a layer whose cross-section is genuinely
# unverifiable -- exactly the case the separate opt-in
# (``tip_alignment_skip_enabled``) exists to handle, deliberately, by
# leaving the layer unmutated rather than by inflating a comparison
# tolerance. ``_measure_term_rates`` therefore checks that the loop
# selected at each probe plane is PLAUSIBLY THE SAME LOOP as the one the
# six-term comparison itself is already using (same sign, area within 2x)
# before trusting any slope it computes -- see that method's own docstring.
# A probe that fails this check returns ``(None, reason)`` (unmeasurable),
# which leaves the affected terms at ``base_tolerance`` -- i.e. wing (7)'s
# layer 1126 continues to raise exactly as it does today unless
# ``tip_alignment_skip_enabled`` is separately opted into -- and the
# ``reason`` is included in the TopologyError message itself, naming
# ``tip_alignment_skip_enabled`` as the instrument for exactly this case, so the failure says WHICH mechanism it
# hit rather than only "above tolerance".
#
# THE ABSOLUTE CEILING (the second half of the rigid-mistranslation
# protection). Even with the sign gate above, a rigid shift can
# still widen the ONE edge whose sign happens to match the local closing
# direction -- the sign gate alone does not bound how far a genuinely wrong
# transform could be admitted on that one term if the rate there is large
# enough. ``effective`` is therefore additionally capped at ``min(ceiling_mm,
# max(base, K * rate * epsilon))``, where ``ceiling_mm`` is the run's own
# resolved layer height (``self._alignment_extremum_band_ mm``, the SAME value
# already resolved for the top-extremum band -- see
# ``_resolve_extremum_band_mm``), falling back to
# ``MESH_ALIGNMENT_SENSITIVITY_CEILING_FALLBACK_MM`` (0.2 mm) only when that
# resolves to nothing. One layer height is ~2.5x the largest effective
# tolerance the real (9)(1) run above actually needed (0.0807 mm mm, layer
# 1002's ``max_x``) and is itself the same physical quantity ``print_z``
# increments by -- a term disagreement past a full layer height is not "the
# plane moved a float's worth" by any reading of the mechanism this feature is
# built around, and is refused regardless of how large the measured rate is.
# Every layer/term pair that hits this ceiling is counted in
# ``_mesh_alignment["sensitivity_ceiling_hit_count"]``.
#
# PROBE DELTA = 0.01 mm. Derived, not a round guess: it must clear this
# mesh's own ``epsilon_mm`` (~2.4e-4 mm on the (9)(1) run measured above) by
# a wide margin so the two probe sections are not themselves sitting in
# float/tessellation noise -- 0.01 mm is ~42x that epsilon on this mesh --
# while staying small relative to a typical 0.2 mm layer height (5%) so the
# measured central-difference slope describes the LOCAL rate at this Z
# rather than an average over a stretch where the closing rate itself
# changes (measured on the same run: the rate on ``max_x`` alone ranges
# 87-169 mm/mm over five consecutive layers, a ~2x swing in 0.8 mm of Z --
# the closing curve is not locally linear over a full layer height).
MESH_ALIGNMENT_SENSITIVITY_PROBE_DELTA_MM = 0.01
MESH_ALIGNMENT_SENSITIVITY_K = 2

# See "THE ABSOLUTE CEILING" above. Used only when
# ``self._alignment_extremum_band_mm`` itself failed to resolve to a usable
# positive number (see ``_resolve_extremum_band_mm``) -- a "some ceiling
# beats none" fallback, not a claim that 0.2 mm is the governing layer
# height for every possible config.
MESH_ALIGNMENT_SENSITIVITY_CEILING_FALLBACK_MM = 0.2

# Cap on how many per-layer sensitivity adjustment records
# ``_mesh_alignment["sensitivity_adjustments"]`` may hold -- same "a handful,
# not every layer" reasoning as ``ALIGNMENT_CONTOUR_DUMP_CAP``.
SENSITIVITY_ADJUSTMENT_DUMP_CAP = 20

# Observed case: an in-host slice of a real wing mutated 1119 of 1125 body
# layers and then failed on the FINAL layer (index 1124, print_z 224.99999999999577 mm, mesh
# top 225.09729003906264 mm -- ~0.10 mm, well under one 0.2 mm layer, below the
# mesh's own top face): "the model mesh cross-section disagrees with the host's
# own slice by 0.04728 mm on centroid_x", every bbox term between 5e-5 and
# 4.5e-3 mm.
#
# THE MECHANISM IS NOT ESTABLISHED. Three candidates, none confirmed:
#   (a) ordinary sub-layer sensitivity -- right at a span extremum, a small
#       change in the exact Z sampled changes how much of a closing/tapering
#       feature is captured, which can move an AREA-WEIGHTED centroid
#       measurably while every bbox EDGE (set by extreme vertices, not an
#       area integral) barely moves. In tension with the SIZE of the
#       observed jump (0.047 mm on one layer, ~0 on the previous 1119) --
#       plausible in direction, not confirmed in magnitude.
#   (b) ``slice_closing_radius`` (0.049 mm in the shipped preset) altering
#       the host's own closing behaviour only at a tip-scale feature --
#       MEASURED UNSUPPORTED: ~4.5e-9 mm of measured effect across chords
#       130 mm -> 1 mm on this repo's own boolean backend. Ruled out, not
#       merely unconfirmed.
#   (c) tip tessellation / vertex-on-plane divergence between this
#       project's own sectioner (``aircraft_core.mesh_section.MeshSectioner``)
#       and libslic3r's own ``TriangleMeshSlicer`` at a
#       Z that lands exactly on or near a mesh vertex -- LIVE, UNTESTED.
#       Not measured in either direction.
#
# So ``tip_alignment_skip_enabled`` is a BOUNDED, OPT-IN MITIGATION for an
# unresolved failure mode, not a fix, and it is scoped accordingly: it never
# touches the four bbox terms (a genuinely wrong transform still fails
# closed, tip or no tip),
# it applies ONLY within one layer height of the mesh's own TOP Z (never
# the root -- no supporting observation there, and ``root_solid_layers``
# normally already excludes the bottom extremum from this check), and a
# layer it applies to is SKIPPED (left unmutated, recorded loudly) rather
# than accepted with a term quietly removed from the decision -- a spar
# socket is never cut from a cross-section this guard declined to verify.
# Off by default; see ``StructureSpec.tip_alignment_skip_enabled``'s
# docstring for the opt-in and its exact semantics.
#
# Band derivation, not assumed: OrcaSlicer's own layer generator
# (``generate_object_layers``, ``exp-orca/src/libslic3r/Slicing.cpp``
# :764-797) accepts a final body layer only while its ``slice_z``
# (``print_z - 0.5 * layer_height``) is strictly below the object's top Z,
# and rejects the next candidate once ITS ``slice_z`` would reach or pass
# it. Combining both conditions bounds the reported ``print_z`` of the
# LAST layer to within HALF a layer height of the mesh's true top face:
# ``abs(print_z - object_top_z) < 0.5 * layer_height``. Comparing
# ``layer.print_z`` (not ``slice_z``) against a band of one FULL layer
# height therefore carries a built-in 2x margin over the tightest case the
# host can actually produce, rather than sitting on the edge of it -- and
# because that margin is 2x, NOT "at most one extra layer": a full-``h``
# band CAN contain TWO top layers, but only at the loose end of the
# accepted range. The last layer's own ``print_z``
# ranges over ``[top - 0.5h, top + 0.5h)``, so the PENULTIMATE layer's
# distance from ``top`` ranges over ``(0.5h, 1.5h]`` -- both land inside a
# one-``h`` band only when the last layer sits ABOVE the mesh's true top
# face (``print_z`` in ``(top, top + 0.5h)``), which puts the penultimate
# at distance ``top - 0.5h`` from ``top``, inside the band. At the TIGHT
# end (``print_z == top - 0.5h`` exactly, the tightest legal position), the
# penultimate sits a full ``1.5h`` from ``top`` and is NOT inside a one-``h``
# band -- only one top layer is band-covered there.
#
# The band is governed by ``layer_height`` alone, not
# ``max(layer_height, initial_layer_print_height)``: this constant is only
# ever consulted for the TOP extremum (see the "TOP extremum only" note at
# its call site), and ``initial_layer_print_height`` governs the FIRST
# layer, at the root -- a quantity this band has no reason to read.
# ``_resolve_extremum_band_mm`` falls back to
# ``initial_layer_print_height`` only when ``layer_height`` itself fails to
# resolve to a usable positive number, which is a "some answer beats none"
# fallback, not a claim that it governs the top.
#
# TWO CAVEATS THIS DERIVATION DOES NOT COVER, stated rather than silently
# assumed away:
#   - With a per-layer ``layer_height_profile`` (adaptive layer height),
#     the governing height at the object's own top is the LOCAL height
#     there, not the uniform ``layer_height`` this band reads -- the
#     margin degrades to roughly ``layer_height / (0.5 * max_layer_height)``
#     when the local top-layer height differs from the uniform one used
#     here.
#   - ``is_precise_z_height`` runs ``adjust_layer_series_to_align_object_
#     height`` AFTER ``generate_object_layers``'s own loop this derivation
#     is based on, which can move the accepted top layer's ``print_z``
#     again in a way this band does not account for.
#
# ACCEPTED LIMITATION: an object 2 layers tall or shorter has every body
# layer within one layer height of the top extremum, so this mitigation (if
# enabled) could skip verifying every layer of such an object.

# below this angle (degrees, either sign), the detected global chord theta
# (``aircraft_core.alignment.resolve_theta_deg``, mid-span- authoritative) is
# treated as exactly zero -- NO rotation is applied at either call site (mesh
# sectioner or host slice mutation), not a rotation by a tiny angle. This is
# what keeps an already-aligned wing's behaviour provably bit-identical to
# as an unrotated one.
#
# MEASURED, not assumed. The chord-line axis (``chord_line_angle_deg``; a
# minimum-area-rectangle angle would be biased by camber) read at
# mid-span on every airfoil-shaped fixture this plugin's own ``execute()``
# path exercises (``tools/*`` measurement, reproducible via
# ``aircraft_core.mesh_section.MeshSectioner`` + ``resolve_theta_deg``):
#
#     no built-in washout (naca2412_c60_s60, the sweep/taper/tip
#     fixtures)      -1.3429e-01 deg   (irreducible method residual --
#                                        matches frame.py's own documented
#                                        "-0.1343 degrees at zero actual
#                                        rotation" for the same NACA 2412
#                                        section, to 4 significant figures)
#     m18_wing_spar*, r150 w1.5/w3 (real washout, mid-span)
#                                       0.6157 - 1.3657 deg  (TRUE washout,
#                                        must NOT be skipped)
#     twist fixture (6 deg washout)     2.8657 deg           (TRUE washout)
#
# So the "already aligned" floor is ~0.1343 deg and the smallest genuine
# angle that must NOT be skipped is the test matrix's own 0.3 deg
# synthetic-rotation case. 0.2 deg sits strictly between them with real
# headroom on both sides: 0.2 - 0.1343 = 0.0657 deg above the floor,
# 0.3 - 0.2 = 0.1000 deg below the smallest case that must fire. (The
# theoretical max-headroom point is the midpoint, 0.2172 deg; 0.2 is used
# as a clean, easily-audited value close to it, not tuned to either edge.)
THETA_EPSILON_DEG = 0.2

DEFAULT_SPEC = {
    "gap_ratio": 0.5,
    "slot_gap_mm": None,
    "skin_margin_mm": 1.0,
    "root_solid_layers": 5,
    "web_position": 0.5,
    "overshoot_mm": 0.05,
    # None reproduces single-web behaviour exactly (see aircraft_core/spec.py's
    # StructureSpec.rib_pitch_mm docstring).
    "rib_pitch_mm": None,
    # Depth-and-families: both default to the pre-existing single
    # -full-depth-family behaviour (see StructureSpec.web_depth_mm /
    # .families docstrings). "families" is a list of dicts, each mapping
    # onto aircraft_core.spec.WebFamily's fields by name — see
    # _build_families' docstring for the exact accepted keys.
    "web_depth_mm": None,
    "families": [],
    # None gives Z-invariant webs exactly (see
    # aircraft_core/spec.py's StructureSpec.z_drift_mm_per_mm /.z_period_mm
    # docstrings and WebFamily.station_drift_mm for the exact formula).
    "z_drift_mm_per_mm": None,
    "z_period_mm": None,
    # off by default (see aircraft_core/spec.py's
    # StructureSpec.tip_shutdown_enabled docstring for why) — a layer where
    # every candidate web was omitted purely for thinness stays a fatal
    # TopologyError until a user opts in.
    "tip_shutdown_enabled": False,
    # off by default -- see aircraft_core/spec.py's
    # StructureSpec.tip_alignment_skip_enabled docstring. A distinct opt-in
    # from tip_shutdown_enabled above: this one gates leaving a layer unmutated
    # when only the mesh-alignment CENTROID terms disagree near the mesh's own
    # top Z, never a pattern-thinness shutdown.
    "tip_alignment_skip_enabled": False,
    # off by default so no existing config changes behaviour. On, a
    # primary_structure family whose station is omitted fails the slice instead
    # of vanishing quietly -- see aircraft_core/spec.py's
    # StructureSpec.require_primary_structure docstring for the measured case.
    "require_primary_structure": False,
    # off by default (no behaviour change) -- see aircraft_core/spec.py's
    # StructureSpec.per_station_reentrant docstring.
    "per_station_reentrant": False,
    # the Z-continuity rule, off by default (no behaviour change) -- see
    # aircraft_core/spec.py's StructureSpec.z_continuity.
    "z_continuity": False,
    "z_continuity_tie_mode": "independent",
    "z_continuity_dx_max_mm": None,
    "z_continuity_wall_matching": True,
    # carbon-spar socket. None disables it. Spar geometry comes from the
    # MODEL (mesh cross-section), not from here. These stay only as a DEBUGGING FALLBACK for the socket
    # primitive, gated behind allow_config_spar_fallback, which defaults off so
    # config-declared geometry can never quietly become the architecture again.
    "allow_config_spar_fallback": False,
    "spar_diameter_mm": None,
    "spar_station": 0.30,
    "spar_clearance_mm": 0.15,
    "spar_neck_width_mm": 1.2,
    # Which of MORE THAN ONE detected channel at a single layer is the
    # spar (e.g. a wing tip carrying alignment-rod bores alongside the
    # spar channel). "refuse" (the default) reproduces the pre-existing
    # TopologyError byte-for-byte -- see aircraft_core/spec.py's
    # StructureSpec.spar_channel_policy docstring for the full precedence
    # and what "nearest_spar_station" does instead.
    "spar_channel_policy": "refuse",
    # rotation envelope: None resolves to
    # aircraft_core.frame.DEFAULT_MAX_ROTATION_DEG (see
    # StructureSpec.max_rotation_deg's docstring for the measured table it is
    # based on).
    "max_rotation_deg": None,
    # transform-time metrics. Default on; the overhead is a handful of
    # time.perf_counter() calls plus a thin call
    # -counting ops wrapper — see aircraft_core/metrics.py's module
    # docstring for exactly what is and is not measured. Set false to
    # disable: no metrics object is created at all, so the disabled case
    # costs nothing beyond one dict lookup and one boolean check.
    "metrics_enabled": True,
    # 0: the generator never builds a scar and an interior hole is
    # unconditionally a defect, caught by invariant 2 (see aircraft_core/spec.py's StructureSpec.expected_hole_count
    # docstring). Capped at 1 by the schema/StructureSpec.validate(): the
    # scarring proof only covers exactly one declared hole per layer.
    "expected_hole_count": 0,
    # perpendicular-entry correction: 0.0 disables it entirely -- every web is
    # placed without tilt (see
    # aircraft_core/spec.py's StructureSpec.max_web_tilt_deg docstring).
    "web_tilt_threshold_deg": None,
    "max_web_tilt_deg": 0.0,
    # non-aerofoil FEATURE regions (a joiner post, boss, tab...) -- see
    # aircraft_core/frame.py's classify_region docstring for the measured
    # separation these thresholds rest on, and aircraft_core/spec.py's
    # StructureSpec.feature_lattice_pitch_mm docstring for the pitch default's
    # justification.
    "feature_aspect_ratio_max": None,
    "feature_fill_ratio_min": None,
    "feature_lattice_pitch_mm": 2.0,
    # False keeps the default feature-lattice depth bound -- see aircraft_core/spec.py's
    # StructureSpec.feature_dense_lattice_enabled docstring.
    "feature_dense_lattice_enabled": False,
    # mesh-to-slice alignment guard tolerance, in millimetres. 0.02 (the
    # default) equals the module constant MESH_ALIGNMENT_TOLERANCE_MM -- see aircraft_core/spec.py's
    # StructureSpec.mesh_alignment_tolerance_mm docstring for what raising it
    # actually trades away (it is not a "my wing is slightly off" tuning knob).
    # Written as a literal, not a reference to MESH_ALIGNMENT_TOLERANCE_MM
    # above, because DEFAULT_SPEC must stay ast.literal_eval-able for
    # test_config_schema.py's DriftGuardTests (a bare Name node there is not
    # literal_eval-able) -- see tests/unit/test_config_schema.py's
    # test_mesh_alignment_tolerance_default_matches_module_constant for the
    # check that keeps this literal and MESH_ALIGNMENT_TOLERANCE_MM from
    # drifting apart.
    "mesh_alignment_tolerance_mm": 0.02,
    # off by default (plain fixed tolerance) -- see
    # aircraft_core/spec.py's StructureSpec.mesh_alignment_sensitivity_enabled
    # docstring and capability.py's own comment above
    # MESH_ALIGNMENT_SENSITIVITY_K for the full derivation.
    "mesh_alignment_sensitivity_enabled": False,
    # Default False: the marginal per-layer time cost on a ~1000-layer run is
    # not small -- see aircraft_core/config_schema.py's ConfigField docstring.
    "record_host_geometry_sidecar": False,
    # "auto" resolves the root end from the placed mesh (area + spar channel
    # signals, fail closed on disagreement); "bed"/"top" override.
    "root_end": "auto",
}

# Keys accepted in one entry of the stored "families" list. Mirrors
# aircraft_core.spec.WebFamily's field names exactly, so a stored config can
# be built with ``WebFamily(**entry)`` after key validation.
#
# This is ``aircraft_core.config_schema.FAMILY_KEYS`` (bare name: no import
# statement here, same "supplied by the bundle" pattern every other
# aircraft_core name in this file uses — config_schema.py is concatenated
# ahead of this file by tools/build_plugin.py). Wired this way, rather than
# repeated as a literal set, so the schema tools/config.py and
# docs/configuration.md read from is the SAME set this capability actually
# accepts — a family key typo cannot silently diverge between what a
# user is told is valid and what the running plugin reads.
_FAMILY_KEYS = FAMILY_KEYS  # noqa: F821 - supplied by the bundle

# Same "supplied by the bundle" wiring as ``_FAMILY_KEYS`` above, one scope
# up: this is ``aircraft_core.config_schema.SPEC_KEYS`` -- the accepted
# top-level ``cap_config`` key set -- so ``_build_spec``'s unknown-key check
# below can never silently diverge from what ``tools/config.py check``
# accepts.
_SPEC_KEYS = SPEC_KEYS  # noqa: F821 - supplied by the bundle


def _plugin_dir():
    """This plugin's installed directory — the only path it may write to."""
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except Exception:
        return None


def _archive_aside(path):
    """If ``path`` already exists, rename it aside to an archive name derived
    from ITS OWN mtime in UTC (e.g. ``m1_report.20260918T224000Z.json``)
    before the caller overwrites ``path`` with this run's output -- so a
    successful run never destroys the evidence of an earlier failing one.

    Never deletes anything and never retries past a collision: if the
    mtime-derived name is already taken (two runs inside the same second,
    routine under repeated slicing), a ``-1``, ``-2``, ... suffix is
    appended until a free name is found, so archiving can never overwrite
    an earlier archive either. There is no retention cap; see
    ``_write_diagnostics``'s docstring.

    Never raises: archiving must not fail a slice any more than the write
    it guards. Returns a dict describing the outcome, folded into the
    report by the caller so a failure here is never invisible.
    """
    info = {"archived": False, "archived_path": None, "error": None}
    try:
        if not os.path.exists(path):
            return info
        stamp = time.strftime(
            "%Y%m%dT%H%M%SZ", time.gmtime(os.path.getmtime(path))
        )
        base_name = os.path.basename(path)
        directory = os.path.dirname(path)
        # Split off BOTH suffixes of a double extension (``.json.gz``) so the
        # stamp lands between the stem and the full suffix, matching the
        # maintainer's example filenames for both targets.
        if base_name.endswith(".json.gz"):
            stem, suffix = base_name[: -len(".json.gz")], ".json.gz"
        else:
            stem, suffix = os.path.splitext(base_name)
        candidate = os.path.join(directory, "{}.{}{}".format(stem, stamp, suffix))
        collision = 0
        # NOT ATOMIC, stated rather than implied: the exists-then-rename pair
        # below can race if two processes share this datadir within the same
        # second. On Windows ``os.rename`` refuses an existing target, so a
        # loser fails safe into ``error``; on POSIX it would silently overwrite
        # the archive. The host runs one slicing worker against one
        # ``--datadir``, so this is a documented residual, not a live hazard.
        while os.path.exists(candidate):
            collision += 1
            candidate = os.path.join(
                directory, "{}.{}-{}{}".format(stem, stamp, collision, suffix)
            )
        os.rename(path, candidate)
        info["archived"] = True
        info["archived_path"] = candidate
    except Exception as exc:  # noqa: BLE001
        info["error"] = "{}: {}".format(type(exc).__name__, exc)
    return info


def _archive_directory_stats(out_dir):
    """Total size and file count of everything ``_archive_aside`` has left
    behind in ``out_dir`` (an mtime stamp between the stem and extension of
    an ``m1_report``/``m1_host_sections`` name) -- NOT the two canonical,
    current-run files. There is no retention cap, so this is how unbounded
    growth stays visible in the report instead of silent (see
    ``_write_diagnostics``'s docstring note). Never raises.
    """
    info = {"total_bytes": 0, "file_count": 0, "error": None}
    canonical = {"m1_report.json", "m1_host_sections.json.gz"}
    try:
        for name in os.listdir(out_dir):
            if name in canonical:
                continue
            is_report_archive = name.startswith("m1_report.") and name.endswith(".json")
            is_sidecar_archive = name.startswith("m1_host_sections.") and name.endswith(
                ".json.gz"
            )
            if not (is_report_archive or is_sidecar_archive):
                continue
            full = os.path.join(out_dir, name)
            try:
                info["total_bytes"] += os.path.getsize(full)
                info["file_count"] += 1
            except OSError:
                continue
    except Exception as exc:  # noqa: BLE001
        info["error"] = "{}: {}".format(type(exc).__name__, exc)
    return info


def _read_bundle_info():
    """Read the RUNNING module's own generated bundle header (written by
    ``tools/build_plugin.py``'s ``_build_header``) from ``__file__`` on disk,
    and return the content-hash line it carries -- so a report always says which bundle produced it.

    Diagnostic only: never raises, never refuses a slice, and a run whose hash
    cannot be read (unbundled source under test, an unreadable file, a
    missing/malformed header line) is still a perfectly valid run -- it just
    says so, the same resilient treatment
    ``trafo()``/``matrix()``/``is_left_handed()`` get. Deliberately does NOT
    compare against the repo source -- the host has no business reading the
    repo; that comparison belongs to offline tooling.
    """
    info = {"content_hash": None, "read": False, "error": None}
    try:
        path = os.path.abspath(__file__)
        with open(path, "r", encoding="utf-8") as handle:
            for _ in range(200):
                line = handle.readline()
                if not line:
                    break
                stripped = line.rstrip("\n")
                if not stripped.strip():
                    continue  # blank lines separate the PEP 723 block,
                    # the generated header, and the code below -- keep
                    # scanning through them rather than stopping early.
                if not stripped.startswith("#"):
                    break  # reached code; the header is always above it.
                if stripped.startswith(_BUNDLE_HASH_LINE_PREFIX):
                    candidate = stripped[len(_BUNDLE_HASH_LINE_PREFIX):].strip()
                    if len(candidate) == 64 and all(
                        c in "0123456789abcdef" for c in candidate.lower()
                    ):
                        info["content_hash"] = candidate
                        info["read"] = True
                    else:
                        info["error"] = "malformed hash line: {!r}".format(candidate)
                    return info
        if info["error"] is None:
            info["error"] = "no bundle content-hash header line found"
    except Exception as exc:  # noqa: BLE001
        info["error"] = "{}: {}".format(type(exc).__name__, exc)
    return info


def _scale_factor():
    """Scaled-integer units per millimetre, read live, never cached.

    ``unscale`` reads the live SCALING_FACTOR (1e-6 normally, 1e-5 for beds
    larger than 2147 mm).
    """
    mm_per_unit = orca.slicing.unscale(1)
    if not mm_per_unit:
        raise RuntimeError("unscale(1) returned 0; cannot derive scale factor")
    return 1.0 / mm_per_unit


class OrcaGeometryOps(GeometryOps):  # noqa: F821 - supplied by the bundle
    """``GeometryOps`` backed by Orca's Clipper-backed ExPolygon bindings.

    Converts at this boundary and nowhere else: everything above speaks
    millimetres and floats, everything below speaks scaled integers.
    """

    def __init__(self):
        self._f = _scale_factor()

    # -- conversion -----------------------------------------------------------

    def _to_scaled(self, mm):
        return int(round(mm * self._f))

    def polygon_from_points(self, points_mm):
        """Build an ExPolygon from millimetre (x, y) tuples — used for slots.

        ``ExPolygon(contour, holes)`` routes its contour through
        ``as_polygon()`` -> ``parse_polygon()``, which accepts ONLY a bound
        ``orca.host.Polygon`` or an (N,2) int64 ndarray — a list of ``Point``
        raises ValueError (PluginHostGeometry.cpp:26-57, 148-165). So build a
        Polygon and append vertices to it, which needs no numpy
        (PluginHostGeometry.cpp:97, 127-129).

        The constructor shape is taken from the host source, not from a
        captured binding dump (which records attribute NAMES only, not
        constructor arity or whether an empty holes list is accepted). A
        mismatch surfaces as a TypeError or ValueError on the first body
        layer, reported as a FatalError.
        """
        contour = orca.host.Polygon()
        for x, y in points_mm:
            contour.append(orca.host.Point(self._to_scaled(x), self._to_scaled(y)))
        return orca.host.ExPolygon(contour, [])

    def polygon_from_points_and_holes(self, points_mm, holes_mm):
        """Like ``polygon_from_points``, but also rebuilds interior holes.

        Used only by ``_rotate_expolygon`` to reconstruct a rotated
        ExPolygon at the host boundary from
        ``contour_points_mm``/``hole_points_mm``. The rotation math itself
        lives in ``aircraft_core.alignment.rotate_points_deg`` (one rotation
        function for the whole plugin); this method only turns millimetre
        point rings back into host geometry, the same boundary-conversion role
        ``polygon_from_points`` plays.
        """
        contour = orca.host.Polygon()
        for x, y in points_mm:
            contour.append(orca.host.Point(self._to_scaled(x), self._to_scaled(y)))
        holes = []
        for hole_pts in holes_mm:
            h = orca.host.Polygon()
            for x, y in hole_pts:
                h.append(orca.host.Point(self._to_scaled(x), self._to_scaled(y)))
            holes.append(h)
        return orca.host.ExPolygon(contour, holes)

    # -- GeometryOps ----------------------------------------------------------

    def diff(self, subject, clips):
        """Subject minus every clip.

        ``ExPolygon.diff_ex`` takes a single other ExPolygon, so clips are
        applied in sequence; each round may split the result into several
        pieces, all of which continue through the remaining clips.
        """
        current = [subject]
        for clip in clips:
            nxt = []
            for piece in current:
                nxt.extend(piece.diff_ex(clip))
            current = nxt
            if not current:
                break
        return current

    def offset(self, polygon, delta_mm):
        return polygon.offset(self._to_scaled(delta_mm))

    def union(self, polygons):
        """Boolean union via ``ExPolygon.union_ex``. Merged pairwise to a
        fixed point, the same shape as ``diff``'s ``diff_ex`` use. The call
        shape ``a.union_ex(b) -> list[ExPolygon]`` is assumed by analogy with
        ``diff_ex``/``intersection_ex``."""
        current = list(polygons)
        changed = True
        while changed:
            changed = False
            for i in range(len(current)):
                for j in range(i + 1, len(current)):
                    merged = current[i].union_ex(current[j])
                    if len(merged) == 1:
                        current = [
                            p for k, p in enumerate(current) if k not in (i, j)
                        ] + [merged[0]]
                        changed = True
                        break
                if changed:
                    break
        return current

    def intersection(self, subject, clip):
        """Subject intersected with clip.

        ``ExPolygon.intersection_ex`` is bound directly on the host, the same
        shape as ``diff_ex`` (PluginHostGeometry.cpp:211-213). Used for local
        thickness measurement (``aircraft_core.frame.local_span``), which clips
        a section against a thin measurement strip rather than a slot.
        """
        return subject.intersection_ex(clip)

    def area(self, polygon):
        # ExPolygon.area() is in scaled units squared.
        return polygon.area() / (self._f * self._f)

    def is_valid(self, polygon):
        return polygon.is_valid()

    def bounds(self, polygon):
        """Axis-aligned bounds in mm, read through ``Polygon.points``.

        ``points`` yields Point references with ``.x`` / ``.y``
        (PluginHostGeometry.cpp:118-126), so no numpy is required. The refs are
        read immediately and never retained — structural changes invalidate
        them, same as C++ vector iterators.
        """
        points = polygon.contour.points
        if not points:
            raise ValueError("cannot take bounds of a polygon with no points")
        xs = [pt.x for pt in points]
        ys = [pt.y for pt in points]
        return (
            min(xs) / self._f,
            min(ys) / self._f,
            max(xs) / self._f,
            max(ys) / self._f,
        )

    def contour_count(self, polygon):
        return polygon.num_contours()

    def contour_points_mm(self, polygon):
        """Outer-contour vertices in mm, read the same way ``bounds()`` does.

        Same lifetime caveat as ``bounds()``: the Point refs are read
        immediately into plain floats and never retained.
        """
        points = polygon.contour.points
        return [(pt.x / self._f, pt.y / self._f) for pt in points]

    def hole_bounds(self, polygon):
        """Axis-aligned bounds in mm of each interior hole.

        ``polygon.holes`` is checked against the host source
        (``PluginHostGeometry.cpp``). The binding is::

            .def_property_readonly("holes", [](py::object self) { ... })

        so ``.holes`` is a read-only PROPERTY, not a bound method, and it
        returns a Python list of ``Polygon`` references taken from the
        underlying ``std::vector<Polygon>``. Each element is a real
        ``Polygon``, whose ``points`` is likewise a
        ``def_property_readonly`` -- so ``for hole in polygon.holes`` and
        ``hole.points`` are both correct as written here.

        For completeness, from the same source: ``contour`` is a
        ``def_property`` (property), while ``set_holes`` and
        ``num_contours`` are ``def`` (methods) -- which is why
        ``contour_count`` above calls ``polygon.num_contours()`` with
        parentheses and this method does not.

        The binding shape comes from the host source; the end-to-end path for
        a holed model is not verified against a running host.
        """
        bounds = []
        for hole in polygon.holes:
            points = hole.points
            if not points:
                raise ValueError("cannot take bounds of a hole with no points")
            xs = [pt.x for pt in points]
            ys = [pt.y for pt in points]
            bounds.append(
                (
                    min(xs) / self._f,
                    min(ys) / self._f,
                    max(xs) / self._f,
                    max(ys) / self._f,
                )
            )
        return bounds

    def hole_points_mm(self, polygon):
        """Each interior hole's vertices in mm, read the same way
        ``contour_points_mm()`` reads the outer contour.

        Same lifetime caveat: Point refs are read immediately into plain
        floats and never retained.
        """
        return [
            [(pt.x / self._f, pt.y / self._f) for pt in hole.points]
            for hole in polygon.holes
        ]


def _rotate_expolygon(ops, polygon, theta_deg, center_mm=(0.0, 0.0)):
    """Rotate a host ExPolygon by ``theta_deg`` about ``center_mm``.

    Reads contour + holes via ``ops`` (the SAME
    ``contour_points_mm``/``hole_points_mm`` accessors ``frame.py`` and
    ``_check_mesh_alignment`` use), rotates with
    ``aircraft_core.alignment.rotate_points_deg`` -- the ONE rotation function
    shared by the mesh sectioner and this host-slice call site -- and
    reconstructs via ``polygon_from_points_and_holes``. Host
    ``Polygon.rotate()`` is deliberately not used: it is a second
    (C++, radians) implementation of the same operation, so the two call
    sites could disagree.
    """
    contour_mm = ops.contour_points_mm(polygon)
    holes_mm = ops.hole_points_mm(polygon)
    r_contour = rotate_points_deg(contour_mm, theta_deg, center_mm)  # noqa: F821 - bundle
    r_holes = [
        rotate_points_deg(h, theta_deg, center_mm)  # noqa: F821 - supplied by bundle
        for h in holes_mm
    ]
    return ops.polygon_from_points_and_holes(r_contour, r_holes)


def _assert_rotation_round_trip(ops, polygon, theta_deg, center_mm=(0.0, 0.0),
                                 max_scaled_units=2):
    """Round-trip an ExPolygon through -theta then +theta and
    assert every vertex lands within ``max_scaled_units`` (scaled integer
    coords, ~1e-6 mm/unit at the normal scaling factor) of where it started.
    ASSERTED, not assumed -- fails closed with a diagnostic naming the worst
    vertex delta rather than silently trusting floating-point rotation +
    round-to-int64 to be lossless.

    Cheap enough to run once per layer (one representative polygon, not
    every polygon/every vertex on every layer): a handful of point-list
    rotations on a section with, at most, a few hundred vertices.

    Reads the scale factor via ``_scale_factor()`` (the same live
    ``orca.slicing.unscale`` read ``OrcaGeometryOps.__init__`` uses), NOT
    ``ops._f`` -- ``ops`` here may be ``CountingGeometryOps`` (metrics
    enabled), which has no such attribute; going through the free
    function works regardless of which concrete backend wraps
    ``OrcaGeometryOps``.
    """
    if abs(theta_deg) < 1e-12:
        return
    original_mm = ops.contour_points_mm(polygon)
    if not original_mm:
        return
    scale = _scale_factor()
    forward = rotate_points_deg(original_mm, -theta_deg, center_mm)  # noqa: F821
    back = rotate_points_deg(forward, theta_deg, center_mm)  # noqa: F821 - bundle
    worst_units = 0.0
    worst_mm = 0.0
    for (ox, oy), (bx, by) in zip(original_mm, back):
        d_units = max(
            abs(ox - bx) * scale,
            abs(oy - by) * scale,
        )
        if d_units > worst_units:
            worst_units = d_units
            worst_mm = max(abs(ox - bx), abs(oy - by))
    if worst_units > max_scaled_units:
        raise RuntimeError(
            "alignment rotation round-trip failed: worst vertex moved "
            "{:.6f} scaled unit(s) ({:.9f} mm) after rotating by {:.6f} deg "
            "and back, above the {} unit tolerance. The mesh-to-print "
            "rotation is not precision-safe at this angle/scale; refusing "
            "to trust a socket cut from it.".format(
                worst_units, worst_mm, theta_deg, max_scaled_units
            )
        )


class AircraftStructuresM1(orca.slicing.SlicingPipelineCapabilityBase):
    """One boundary-connected slot per body layer, validated and fail-closed."""

    def get_name(self):
        return "Aircraft Structures"

    def get_default_config(self):
        return dict(DEFAULT_SPEC)

    def execute(self, ctx):
        if not self._armed():
            if ctx.step == orca.slicing.Step.posSlice:
                return orca.ExecutionResult.success(
                    "aircraft structures not armed ({} absent); slice untouched".format(
                        ARM_MARKER
                    )
                )
            return orca.ExecutionResult.success()

        if ctx.step != orca.slicing.Step.posSlice:
            return orca.ExecutionResult.success()

        if ctx.object is None:
            return orca.ExecutionResult.failure(
                orca.PluginResult.RecoverableError,
                "posSlice fired with ctx.object None; expected an object-scoped step",
            )

        started = time.perf_counter()
        resolved = {}  # bound before the try so the error paths can report it
        # Owned here, not inside _transform: a fail-closed run is exactly when
        # the per-layer record matters most, and _transform raises rather than
        # returning it.
        diagnostics = RunDiagnostics()  # noqa: F821 - supplied by the bundle
        # Same reasoning applies to performance metrics: a failing run's
        # partial per-layer timings/counts are exactly the data a slow-then
        # -crashing layer needs, so this is owned here too and serialized on
        # both the success and TopologyError paths. Stays None (and so is
        # never serialized) when metrics are disabled or _build_spec itself
        # fails before metrics_enabled is even known.
        metrics = None
        # Set by _build_spec on success; stays [] (an unknown top-level key
        # never fails a slice on its own -- see _unknown_top_level_key_warning's
        # docstring) if _build_spec itself raises before reaching its return.
        unknown_config_keys = []
        # Reset every only-written-on-success instance attribute here, at the
        # true run boundary: execute() fires at EVERY pipeline step (ctx.step),
        # not once per slice, so this cannot sit at the top of execute() itself
        # -- but everything above this line already ran unconditionally for
        # this run (the ctx.step/ctx.object gates), and everything below --
        # _build_spec, then _transform -- can raise and reach one of the report
        # sites below (the "except" branches) that read these attributes.
        # Resetting any later (e.g. inside _transform, which a _build_spec
        # failure never reaches) would leave a _build_spec failure free to
        # report a PRIOR run's data: a run that raises before its own first
        # successful layer would leave these holding a PRIOR run's data on the
        # same host-reused capability instance.
        #
        # All three are reset together despite ``_mesh_alignment`` normally
        # being assigned unconditionally near the top of ``_transform``
        # (making it fresh for any run that reaches _transform at all): an
        # UNEXPECTED exception raised inside ``_build_spec``, before
        # ``_transform`` is ever entered, lands in the generic ``except
        # Exception`` handler below, which reads
        # ``getattr(self, "_mesh_alignment", None)`` exactly like the other
        # two -- so on that one path ``_mesh_alignment`` has the identical
        # exposure and must be reset here too, not left to _transform's own
        # assignment. ``None``/``0`` match the sentinels already read via
        # ``getattr(self, ..., None/0)`` at the failure/success report
        # sites, so an absent-this-run value is reported explicitly rather
        # than omitted or stale.
        self._last_geometry = None
        self._root_layers = 0
        # Same staleness discipline: a run that raises
        # before ``_transform`` even reads
        # ``spec.record_host_geometry_sidecar`` must not write (or, worse,
        # silently reuse) a PRIOR run's sidecar entries under this run's
        # report. ``_sidecar_enabled`` starts False so a run that never reaches
        # _transform writes no sidecar at all, rather than an empty-but-present
        # one that looks like "this run recorded zero layers" instead of "this
        # run never got far enough to record".
        self._host_sections_sidecar = []
        self._sidecar_enabled = False
        self._current_sidecar_entry = None
        # A fresh identifier for THIS run, generated unconditionally here
        # (not gated on the sidecar flag, and not left to _transform, which a
        # run can fail before reaching) so the report's own
        # "host_sections_sidecar" field always names ITS OWN run, and -- when
        # the sidecar is actually written -- the same value is embedded in the
        # sidecar file too, letting a reader match one to the other without
        # relying on wall-clock proximity or file mtimes.
        self._run_id = uuid.uuid4().hex
        self._mesh_alignment = None
        # Same staleness discipline: the
        # raw host object/volume/instance transform is read fresh inside
        # ``_build_mesh_sectioner`` on every run that reaches it, but a run
        # that raises in ``_build_spec`` -- before
        # ``_transform``/``_build_mesh_sectioner`` are ever entered -- must not
        # report a PRIOR run's placement as this run's (see
        # ``HostObjectTransformResetBetweenRunsTests``).
        self._host_object_transform = None
        # Diagnostic-only: the exact ``slice_z`` the host
        # sectioner actually used for the most recently processed layer, set
        # alongside ``_last_loops`` in ``_channels_for_layer`` and read by
        # ``_check_mesh_alignment`` for its contour dumps. Same staleness
        # discipline -- an unexpected exception raised
        # before ``_transform`` is ever entered must not let a prior run's
        # value leak into this run's report.
        self._last_applied_slice_z = None
        # Same staleness discipline: this is set only inside ``_transform``
        # (see that method), so a run
        # that raises in ``_build_spec`` before ``_transform`` is ever entered
        # would otherwise inherit a PRIOR run's band on a host-reused
        # capability instance.
        self._alignment_extremum_band_mm = None
        self._closing_radius_raw = None
        # same staleness discipline: the root-end / spar-latch reports and the
        # tip-end selector are written inside ``_transform``; a run that raises
        # before then must not report a PRIOR run's resolution.
        self._root_end_report = None
        self._spar_latch_report = None
        self._latch_precomputed = None
        self._latch_online_arm_index = None
        self._latch_online_suppressed = 0
        # Same staleness discipline: computed inside ``_build_spec`` (right
        # next to ``_raw_cap_config``, the field it's diffed against), so a run
        # that raises before ``_build_spec`` even runs must not report a PRIOR
        # run's config-drift finding as this run's.
        self._config_drift = None
        spec = None  # the continuity block reads it on every exit path
        try:
            spec, resolved, metrics_enabled, unknown_config_keys = self._build_spec(ctx)
            metrics = PerformanceMetrics() if metrics_enabled else None  # noqa: F821
            self._transform(ctx, spec, resolved, diagnostics, metrics)
        except TopologyError as exc:  # noqa: F821 - supplied by the bundle
            diagnostics.record(
                LayerRecord(  # noqa: F821 - supplied by the bundle
                    layer_index=exc.layer_index,
                    print_z=exc.print_z,
                    mutated=False,
                    failed=True,
                    message=str(exc),
                    extra={"cause": exc.cause},
                )
            )
            report = diagnostics.to_dict()
            report["error"] = str(exc)
            report["continuity"] = self._continuity_block(report, spec, resolved)
            report["z_continuity"] = self._z_rule_report_block()
            report["geometry_at_failure"] = getattr(self, "_last_geometry", None)
            report["config"] = resolved
            report["root_layers"] = getattr(self, "_root_layers", 0)
            report["unknown_top_level_config_keys"] = unknown_config_keys
            report["config_drift"] = getattr(self, "_config_drift", None) or []
            report["mesh_alignment"] = getattr(self, "_mesh_alignment", None)
            report["root_end"] = getattr(self, "_root_end_report", None)
            report["spar_latch"] = self._spar_latch_report_snapshot()
            # The raw host instance/volume transform
            # (matrix/offset/rotation/scale/mirror as the host actually exposed
            # them) plus what the plugin derived from it, on the TopologyError
            # path -- exactly the path a bad transform is most likely to be
            # discovered on.
            report["host_object_transform"] = getattr(
                self, "_host_object_transform", None)

            if metrics is not None:
                # Metrics must never fail a slice, including at this
                # serialization boundary: the geometry failure above is
                # already diagnosed and must reach the caller even if the
                # aggregation math here (new, unlike RunDiagnostics.to_dict)
                # has a bug of its own.
                try:
                    report["performance"] = metrics.to_dict()
                except Exception as metrics_exc:
                    report["performance"] = "<unavailable: {}>".format(metrics_exc)
            self._write_diagnostics(report)
            # even on a fatal topology failure, an unrecognized top-level key
            # stays visible on the message the caller actually sees (not only
            # inside the diagnostics JSON) -- the scenario where the typo
            # matters most is exactly a default silently substituted in place
            # of the intended value, then a topology failure downstream that
            # looks unrelated to the config.
            config_warning_suffix = self._unknown_top_level_key_warning(
                unknown_config_keys
            )
            config_drift_suffix = self._config_drift_warning_suffix()
            return orca.ExecutionResult.failure(
                orca.PluginResult.FatalError,
                str(exc) + config_warning_suffix + config_drift_suffix,
            )
        except SpecError as exc:  # noqa: F821 - supplied by the bundle
            return orca.ExecutionResult.failure(
                orca.PluginResult.FatalError,
                "aircraft structures configuration invalid: {}".format(exc),
            )
        except CanceledSlice:
            return orca.ExecutionResult.success("slicing cancelled")
        except Exception as exc:
            # An UNEXPECTED exception is the case where a diagnostic is most
            # needed: without one the caller sees only "aircraft structures
            # failed: ..." with no layer, no Z and no JSON report, which is
            # indistinguishable from the host's own "got other exception".
            # Everything known at this point is recorded before failing --
            # the traceback especially, since an unexpected exception is by
            # definition one whose location is not already in the message.
            import traceback
            try:
                report = diagnostics.to_dict()
                report["error"] = "{}: {}".format(type(exc).__name__, exc)
                report["unexpected_exception"] = True
                report["traceback"] = traceback.format_exc()[-4000:]
                report["geometry_at_failure"] = getattr(
                    self, "_last_geometry", None)
                report["config"] = resolved
                report["root_layers"] = getattr(self, "_root_layers", 0)
                report["unknown_top_level_config_keys"] = unknown_config_keys
                report["config_drift"] = getattr(self, "_config_drift", None) or []
                report["mesh_alignment"] = getattr(self, "_mesh_alignment", None)
                report["root_end"] = getattr(self, "_root_end_report", None)
                report["spar_latch"] = self._spar_latch_report_snapshot()
                # The raw host instance/volume transform, on the
                # unexpected-exception path -- the case with the least other
                # context, so this is the one place it matters most not to be
                # silently missing.
                report["host_object_transform"] = getattr(
                    self, "_host_object_transform", None)
                self._write_diagnostics(report)
            except Exception:  # noqa: BLE001
                # Diagnostics must never mask the original failure.
                pass
            return orca.ExecutionResult.failure(
                orca.PluginResult.FatalError,
                "aircraft structures failed: {}: {}".format(type(exc).__name__, exc),
            )

        elapsed_ms = round((time.perf_counter() - started) * 1000.0, 3)
        report = diagnostics.to_dict()
        report["elapsed_ms"] = elapsed_ms
        report["continuity"] = self._continuity_block(report, spec, resolved)
        report["z_continuity"] = self._z_rule_report_block()
        report["config"] = resolved
        report["orca_version"] = ctx.orca_version
        report["root_layers"] = getattr(self, "_root_layers", 0)
        report["unknown_top_level_config_keys"] = unknown_config_keys
        report["config_drift"] = getattr(self, "_config_drift", None) or []
        report["mesh_alignment"] = getattr(self, "_mesh_alignment", None)
        report["root_end"] = getattr(self, "_root_end_report", None)
        report["spar_latch"] = self._spar_latch_report_snapshot()
        # Recorded on the success path too, matching the placement
        # a real slice was actually run with, verifiable against the GUI's own
        # placement.
        report["host_object_transform"] = getattr(
            self, "_host_object_transform", None)
        if metrics is not None:
            # "Fraction of total slicing time" cannot be computed here — this
            # capability only sees its own execute() call, not the host's
            # total slice duration. elapsed_ms above is this run's own total;
            # divide it by the host log's slice-duration line for the
            # fraction (see aircraft_core/metrics.py's module docstring).
            # Guarded the same way the failure branch above is: the slice
            # already succeeded, and a bug in this new aggregation math must
            # not turn a good slice into a reported failure.
            try:
                report["performance"] = metrics.to_dict()
            except Exception as metrics_exc:
                report["performance"] = "<unavailable: {}>".format(metrics_exc)
        self._write_diagnostics(report)

        shutdown_suffix = ""
        if diagnostics.shutdown_count:
            # ``shutdown_count`` merges unrelated causes that share the same
            # ``LayerRecord`` shape; only this message distinguishes them.
            #
            # Counts are DERIVED FROM EACH RECORD'S OWN ``extra["reason"]``,
            # NOT by subtraction from the total. Subtracting
            # ``tip_alignment_skip_count`` would fold any further producer
            # (e.g. ``all_webs_omitted``, where NO CANDIDATE WAS EVER
            # GENERATED) into "pattern shutdown", mislabelling a layer that
            # never had material to thin as one that ran out of it.
            #
            # Counting by reason gives each producer its own bucket.
            reason_counts = {}
            for rec in diagnostics.records:
                if not rec.shutdown:
                    continue
                key = str((rec.extra or {}).get("reason", "unspecified"))
                reason_counts[key] = reason_counts.get(key, 0) + 1
            _LABELS = {
                "tip_shutdown": "pattern shutdown",
                "tip_alignment_skip": "tip-alignment skip",
                "all_webs_omitted": "no webs generated",
            }
            # ALWAYS name the cause, even when there is only one: a bare count
            # in the common single-cause case would say nothing about WHY.
            shutdown_suffix = (
                ", {} shut down (first at Z={:.2f} mm: {})".format(
                    diagnostics.shutdown_count,
                    diagnostics.first_shutdown.print_z,
                    "; ".join(
                        "{} {}".format(c, _LABELS.get(k, k))
                        for k, c in sorted(reason_counts.items())
                    ),
                )
            )
        # Degradation is flagged in THIS sentence, not only in the JSON
        # report: the user reads the host-log line. A wing that dropped its
        # spar socket for hundreds of layers and reported plain success here
        # would be silent degradation at the only level that matters.
        degraded_suffix = ""
        degraded_n = diagnostics.degraded_layer_count
        if degraded_n:
            by_kind = diagnostics.degradation_counts_by_kind
            first = diagnostics.first_degradation
            degraded_suffix = (
                ", {} layer(s) DEGRADED (first at Z={:.2f} mm: {})".format(
                    degraded_n,
                    first.print_z if first is not None else 0.0,
                    "; ".join(
                        "{} x{}".format(k, c)
                        for k, c in sorted(by_kind.items())
                    ),
                )
            )
        mitre_omitted_total = sum(
            int((r.extra or {}).get("mitre_omitted", 0) or 0)
            for r in diagnostics.records
        )
        mitre_suffix = (
            ", {} web(s) omitted for mitre clearance".format(mitre_omitted_total)
            if mitre_omitted_total else ""
        )
        tip_gap_suffix = self._tip_gap_warning_suffix(diagnostics)
        truncation_suffix = self._merge_truncation_warning_suffix(diagnostics)
        opposing_entry_suffix = self._opposing_entry_warning_suffix(diagnostics)
        role_yield_suffix = self._role_yield_warning_suffix(diagnostics)
        config_warning_suffix = self._unknown_top_level_key_warning(unknown_config_keys)
        inert_warning_suffix = self._inert_config_key_warning(
            getattr(self, "_raw_cap_config", None))
        config_drift_suffix = self._config_drift_warning_suffix()
        sensitivity_suffix = self._sensitivity_warning_suffix()
        return orca.ExecutionResult.success(
            "aircraft structures: {} layers seen, {} slotted, {} left solid, "
            "{:.1f} ms{}{}{}{}{}{}{}{}{}{}{}".format(
                report.get("layers_seen", 0),
                diagnostics.mutated_count,
                report.get("root_layers", 0),
                elapsed_ms,
                degraded_suffix,
                mitre_suffix,
                shutdown_suffix,
                tip_gap_suffix,
                truncation_suffix,
                opposing_entry_suffix,
                role_yield_suffix,
                config_warning_suffix,
                inert_warning_suffix,
                config_drift_suffix,
                sensitivity_suffix,
            )
        )

    # -- internals ------------------------------------------------------------

    def _tip_gap_warning_suffix(self, diagnostics):
        """The web-tip air gap warning fragment for the dispatcher's
        host-log result line, or ``""`` if it should not fire.

        Appended to the SAME host-log line the shutdown/merge counts use, not
        only written into the diagnostics JSON, because a field in an unread
        report is not loud.

        Fires only when BOTH: the bond class is "marginal" or "none" (i.e. the
        measured table does not support fusion at this gap), AND
        ``full_depth_web_count > 0`` (there is actually at least one full-depth
        web this applies to — a part built entirely from depth-limited/shallow
        families has no tip-bond question to raise, per
        ``RunDiagnostics.full_depth_web_count``'s own docstring). Pulled into
        its own method (rather than inlined in ``execute()``) so it is
        unit-testable directly against a constructed ``RunDiagnostics``,
        without needing a full slicing-host stack.
        """
        bond_class = diagnostics.web_tip_bond_class
        if bond_class not in ("marginal", "none") or diagnostics.full_depth_web_count <= 0:
            return ""
        return (
            ", WARNING: {} full-depth web(s), tip air gap {:.4f} mm "
            "vs -Y skin (bond class: {}) -- these webs are NOT "
            "bonding to the far skin".format(
                diagnostics.full_depth_web_count,
                diagnostics.web_tip_air_gap_mm,
                bond_class,
            )
        )

    def _sensitivity_warning_suffix(self):
        """Warning fragment for the unmeasurable/ceiling counts. They belong on
        the host-log line the user reads, not only in the JSON -- same
        reasoning as ``_tip_gap_warning_suffix`` above.
        Fires only when ``mesh_alignment_sensitivity_enabled`` was actually on
        for this run AND at least one of the three counters is nonzero; a run
        where the field is off (the default) or where every over-base term was
        cleanly measured and admitted produces no suffix at all.
        """
        stats = getattr(self, "_mesh_alignment", None)
        if not stats or not stats.get("sensitivity_enabled"):
            return ""
        widened = stats.get("sensitivity_widened_term_count", 0)
        unmeasurable = stats.get("sensitivity_unmeasurable_count", 0)
        ceiling_hits = stats.get("sensitivity_ceiling_hit_count", 0)
        if not (widened or unmeasurable or ceiling_hits):
            return ""
        return (
            ", sensitivity-aware tolerance: {} layer(s) widened, "
            "{} unmeasurable, {} ceiling-capped".format(
                widened, unmeasurable, ceiling_hits
            )
        )

    def _safe_intersection_bounds(self, ops, polygon, slots):
        """Clipper's own view of the cut, for diagnosis only. Never raises."""
        try:
            return [
                list(ops.bounds(piece))
                for sl in slots
                for piece in ops.intersection(polygon, sl)
            ]
        except Exception as exc:
            return "<unavailable: {}>".format(exc)

    def _armed(self):
        base = _plugin_dir()
        return bool(base) and os.path.exists(os.path.join(base, ARM_MARKER))

    def _measure_term_rates(self, sectioner, applied_z, base_area,
                             probe_delta_mm, closer=None):
        """SIGNED central-difference ``d(term)/dz`` for all six comparison
        terms, by re-sectioning the MESH at ``applied_z +/- probe_delta_mm``.

        Returns ``(rates, None)`` -- a dict of six SIGNED rates (mm per mm
        of Z, positive if the term grows with Z) -- or ``(None, reason)``
        when the rate cannot be trusted, ``reason`` a short human-readable
        string naming why. FAIL CLOSED: a caller that gets ``None`` back must
        leave every term at ``base_tolerance``, i.e. the plain fixed-tolerance
        comparison.

        SIGNED, not absolute: the sign-gate in ``_check_mesh_alignment`` needs
        to compare the OBSERVED disagreement's sign against the mechanism's
        PREDICTED sign, and an absolute-value rate cannot express a prediction
        at all. See this module's own comment above
        ``MESH_ALIGNMENT_SENSITIVITY_K`` for the sign convention and why it
        matters.

        Three independent reasons this returns ``(None, reason)`` rather
        than a number, all deliberate:

        1. Either probe point falls outside the mesh's own strict
           ``(z_min, z_max)`` interior (a tip/root layer close enough to
           the mesh extreme that one probe would ask ``section_at`` a
           question it cannot answer) -- no measurement attempted.
        2. Either probe section raises (dangling endpoints surviving every
           retry, or any other ``Exception``) -- a probe failure is not
           this method's problem to solve; it reports "unmeasurable", not
           a fallback value.
        3. THE LOOP-IDENTITY CHECK (see this module's own
           ``MESH_ALIGNMENT_SENSITIVITY_K`` comment, "a different,
           unrelated failure mode"): the outer loop selected at EITHER
           probe plane must be plausibly the SAME loop the six-term
           comparison itself is already using at ``applied_z`` --
           same-signed area, magnitude within a factor of 2 of
           ``base_area``. A topologically unstable tip cap (wing
           "(7)", layer 1126: signed area 911 -> -591 -> -202 mm2 over
           three layers, sign and identity both flipping) fails this
           check by construction, so its "rate" is never computed at all
           -- extrapolating a central difference across a discontinuity
           would not describe a physical closing speed.

        ``probe_delta_mm`` is deliberately NOT adaptive to
        ``sectioner.epsilon_mm`` or to the local layer height -- see the
        module comment for the fixed 0.01 mm derivation.

        Saves and restores ``sectioner.last_nudge_mm`` around its own two probe
        calls: ``section_at`` WRITES that attribute on every call, including a
        probe's, and this module's own comment above promises the sensitivity
        feature reads ``epsilon_mm`` ONLY. That holds for WRITES as well as
        reads: without the restore, a probe needing the dangling-endpoint retry
        would silently overwrite ``last_nudge_mm`` with the PROBE plane's nudge, not
        the layer's own ``applied_z`` nudge that other code (contour dumps,
        ``_channels_for_layer``'s own bookkeeping) reads afterwards.
        """
        if sectioner.z_min is None or sectioner.z_max is None:
            return None, "mesh has no Z range"
        lo_z = applied_z - probe_delta_mm
        hi_z = applied_z + probe_delta_mm
        if not (sectioner.z_min < lo_z < sectioner.z_max):
            return None, (
                "probe point z={:.4f} mm (applied_z - {} mm) falls outside "
                "the mesh's own strict Z range".format(lo_z, probe_delta_mm)
            )
        if not (sectioner.z_min < hi_z < sectioner.z_max):
            return None, (
                "probe point z={:.4f} mm (applied_z + {} mm) falls outside "
                "the mesh's own strict Z range".format(hi_z, probe_delta_mm)
            )
        saved_nudge = sectioner.last_nudge_mm
        try:
            try:
                lo_loops = sectioner.section_at(lo_z)
                hi_loops = sectioner.section_at(hi_z)
            except Exception as exc:  # noqa: BLE001 - unmeasurable, not fatal
                return None, "probe section raised {}: {}".format(
                    type(exc).__name__, str(exc)[:150])
        finally:
            sectioner.last_nudge_mm = saved_nudge

        lo_outer = [lp for lp in lo_loops if not lp.is_interior]
        hi_outer = [lp for lp in hi_loops if not lp.is_interior]
        if closer is not None:
            # probe planes see the same host closing as the base plane.
            lo_outer = closer(lo_outer) if lo_outer else []
            hi_outer = closer(hi_outer) if hi_outer else []
        if not lo_outer or not hi_outer:
            return None, "a probe plane produced no outer loop"
        lo_best = max(lo_outer, key=lambda lp: abs(lp.signed_area))
        hi_best = max(hi_outer, key=lambda lp: abs(lp.signed_area))

        # Loop-identity plausibility check (reason 3 above). base_area is
        # the SAME loop's signed_area the caller already selected for the
        # six-term comparison at applied_z.
        if base_area == 0.0:
            return None, "the base loop's own area is zero"
        for probe_area in (lo_best.signed_area, hi_best.signed_area):
            same_sign = (probe_area > 0) == (base_area > 0)
            ratio = abs(probe_area) / abs(base_area)
            if not same_sign or ratio < 0.5 or ratio > 2.0:
                return None, (
                    "the loop selected at a probe plane is not plausibly "
                    "the same loop as the one this comparison uses "
                    "(area {:.4f} mm2 vs base {:.4f} mm2) -- this is the "
                    "signature of a topologically unstable cross-section "
                    "near this Z, exactly the case "
                    "tip_alignment_skip_enabled exists to handle".format(
                        probe_area, base_area)
                )

        span = hi_z - lo_z  # == 2 * probe_delta_mm; both probes already
        # verified strictly inside the mesh's own Z range above, so this is
        # never clipped short.
        lo_mnx, lo_mny, lo_mxx, lo_mxy = lo_best.bbox
        hi_mnx, hi_mny, hi_mxx, hi_mxy = hi_best.bbox
        lo_cx, lo_cy = lo_best.centroid
        hi_cx, hi_cy = hi_best.centroid
        return {
            "min_x": (hi_mnx - lo_mnx) / span,
            "max_x": (hi_mxx - lo_mxx) / span,
            "min_y": (hi_mny - lo_mny) / span,
            "max_y": (hi_mxy - lo_mxy) / span,
            "centroid_x": (hi_cx - lo_cx) / span,
            "centroid_y": (hi_cy - lo_cy) / span,
        }, None

    def _build_spec(self, ctx):
        """Merge stored config over defaults and resolve Orca's settings."""
        resolved = {}
        for key in CONFIG_KEYS:
            try:
                resolved[key] = ctx.config_value(key)
            except Exception as exc:
                resolved[key] = "<error: {}>".format(exc)

        values = dict(DEFAULT_SPEC)
        unknown_keys = []
        try:
            stored = json.loads(self.get_config() or "{}")
            # Kept for the inert-key warning: the filtered merge below
            # discards keys that no longer do anything, so this is the only
            # place the raw user config is visible.
            self._raw_cap_config = stored if isinstance(stored, dict) else {}
            # OrcaSlicer reads the plugin's stored config once, at app start. ``self.get_config()``
            # above is that in-memory copy -- what THIS run actually
            # uses -- and it can silently disagree with
            # ``config.json`` on disk if the file was edited (or
            # rewritten by ``tools/config.py``) while the host kept
            # running. Diffed here, immediately next to
            # ``_raw_cap_config``, so it has identical staleness
            # properties to that field (see the run-boundary reset
            # above and that field's own comment).
            self._config_drift = self._compute_config_drift(self._raw_cap_config)
            if isinstance(stored, dict):
                # An unrecognized top-level key never reaches ``values`` or the
                # built ``StructureSpec`` (the same check ``_build_families``
                # applies to "families" entries via ``_FAMILY_KEYS``), but it is
                # not dropped silently: every filtered key is recorded and
                # surfaced as a loud warning by
                # ``_unknown_top_level_key_warning`` below, named individually
                # with the schema's own did-you-mean/wrong-scope message.
                unknown_keys = sorted(k for k in stored if k not in _SPEC_KEYS)
                values.update({k: v for k, v in stored.items() if k in DEFAULT_SPEC})
        except Exception:
            # A malformed stored config must not silently select defaults that
            # differ from what the user believes is set.
            raise SpecError(  # noqa: F821 - supplied by the bundle
                "stored plugin configuration is not valid JSON; fix or reset it "
                "in the Plugins dialog"
            )

        rib_pitch_mm = values["rib_pitch_mm"]
        web_depth_mm = values["web_depth_mm"]
        z_drift_mm_per_mm = values["z_drift_mm_per_mm"]
        z_period_mm = values["z_period_mm"]
        max_rotation_deg = values["max_rotation_deg"]
        web_tilt_threshold_deg = values["web_tilt_threshold_deg"]
        spec = StructureSpec(  # noqa: F821 - supplied by the bundle
            slot_gap_mm=values["slot_gap_mm"],
            gap_ratio=values["gap_ratio"],
            skin_margin_mm=float(values["skin_margin_mm"]),
            root_solid_layers=int(values["root_solid_layers"]),
            web_position=float(values["web_position"]),
            overshoot_mm=float(values["overshoot_mm"]),
            rib_pitch_mm=None if rib_pitch_mm is None else float(rib_pitch_mm),
            web_depth_mm=None if web_depth_mm is None else float(web_depth_mm),
            families=self._build_families(values["families"]),
            z_drift_mm_per_mm=(
                None if z_drift_mm_per_mm is None else float(z_drift_mm_per_mm)
            ),
            z_period_mm=None if z_period_mm is None else float(z_period_mm),
            tip_shutdown_enabled=bool(values["tip_shutdown_enabled"]),
            tip_alignment_skip_enabled=bool(values["tip_alignment_skip_enabled"]),
            require_primary_structure=bool(values["require_primary_structure"]),
            per_station_reentrant=bool(values["per_station_reentrant"]),
            z_continuity=bool(values["z_continuity"]),
            z_continuity_tie_mode=str(values["z_continuity_tie_mode"]),
            z_continuity_dx_max_mm=(
                None if values["z_continuity_dx_max_mm"] is None
                else float(values["z_continuity_dx_max_mm"])),
            z_continuity_wall_matching=bool(
                values["z_continuity_wall_matching"]),
            allow_config_spar_fallback=bool(
                values["allow_config_spar_fallback"]),
            spar_diameter_mm=values["spar_diameter_mm"],
            spar_station=float(values["spar_station"]),
            spar_clearance_mm=float(values["spar_clearance_mm"]),
            spar_neck_width_mm=float(values["spar_neck_width_mm"]),
            spar_channel_policy=str(values["spar_channel_policy"]),
            max_rotation_deg=(
                None if max_rotation_deg is None else float(max_rotation_deg)
            ),
            expected_hole_count=int(values["expected_hole_count"]),
            web_tilt_threshold_deg=(
                None if web_tilt_threshold_deg is None else float(web_tilt_threshold_deg)
            ),
            max_web_tilt_deg=float(values["max_web_tilt_deg"]),
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
            mesh_alignment_tolerance_mm=float(
                values["mesh_alignment_tolerance_mm"]
            ),
            mesh_alignment_sensitivity_enabled=bool(
                values["mesh_alignment_sensitivity_enabled"]
            ),
            record_host_geometry_sidecar=bool(
                values["record_host_geometry_sidecar"]
            ),
            root_end=str(values["root_end"]),
        )
        # Guard skin_margin_mm > 2 * safety_offset_mm here, once, at
        # config-build time -- before any layer is generated (it also fires
        # per layer inside StructureGenerator.generate(); see that method's
        # docstring). This is the earliest point the resolved outer wall line
        # width is known, so this is the earliest the guard CAN fire. A config that fails this
        # never reaches _transform()/generate() at all.
        spec.validate(safety_offset_mm=self._outer_wall_width_mm(resolved))
        self._check_shell_alignment(spec, resolved)
        metrics_enabled = bool(values["metrics_enabled"])
        return spec, resolved, metrics_enabled, unknown_keys

    # Config keys that are still ACCEPTED but no longer change any
    # behaviour, each mapped to the reason it is inert. Accepting an
    # inert key beats rejecting a known one -- existing configs and the
    # tracked preset set these -- but a silently inert setting is its own
    # kind of lie, so setting one to a non-default value says so in the
    # run summary, not only in the schema description nobody reads.
    INERT_CONFIG_KEYS = {
        "tip_shutdown_enabled": (
            False,
            "Class B degradation is the default, so a layer whose "
            "every candidate web was omitted is left unmutated and "
            "reported whatever this flag says",
        ),
    }

    def _inert_config_key_warning(self, cap_config):
        """Loud fragment naming any accepted-but-inert key set to a
        non-default value, or ``""``. Same host-log line and the same
        "a field in an unread report is not loud" reasoning as
        ``_unknown_top_level_key_warning`` below.
        """
        if not isinstance(cap_config, dict):
            return ""
        hits = []
        for key, (default, why) in sorted(self.INERT_CONFIG_KEYS.items()):
            if key in cap_config and cap_config[key] != default:
                hits.append("{}={!r} (INERT: {})".format(
                    key, cap_config[key], why))
        if not hits:
            return ""
        return ", WARNING: {} config key(s) set but no longer do anything: {}".format(
            len(hits), "; ".join(hits))

    # Sentinel for "the file/host copy of cap_config has no such key" in the
    # drift report below -- kept as a plain string (not a Python object())
    # so a drift entry is JSON-serializable as-is, no translation needed at
    # the ``report[...] = ...`` sites.
    _CONFIG_DRIFT_ABSENT = "<absent>"
    # Cap on a single value's repr() length in the host-log warning line
    # (see _config_drift_warning_suffix) -- the full values are still
    # written into report["config_drift"] (the diagnostics JSON), only the
    # one-line summary is truncated.
    _CONFIG_DRIFT_VALUE_MAX_CHARS = 80

    def _compute_config_drift(self, cap_config):
        """Diff ``cap_config`` (what the host handed THIS run via
        ``self.get_config()``) against ``host/data/orca_plugins/config.json``
        on disk, right now, at the path derived from ``_plugin_dir()``.

        OrcaSlicer reads the plugin's stored config once, at app start; an
        edit to the file afterwards (by hand, or by ``tools/config.py``)
        does not reach a running host until it restarts. The failure
        this detects: ``config.json`` holds a new value, but the run still
        uses the OLD in-memory copy and fails a layer that the new value would
        have handled, with nothing to say the two disagreed.

        The file is NOT a flat ``cap_config`` object -- it is the same
        wrapper shape ``tools/config.py`` reads/writes (``_entry_for``/
        ``_cap_config_for`` there) and the shape captured verbatim in
        ``presets/plugin/live_host_config.json``: ``{"config": [{"cap_config":
        {...}, "capability": "Aircraft Structures", ...}, ...]}``. This
        method finds the entry whose ``"capability"`` matches
        ``self.get_name()`` and diffs against ITS ``cap_config``, exactly
        the extraction ``tools/config.py``'s ``_cap_config_for``
        does -- diffing against the wrapper's own top level would compare
        ``cap_config`` (a dict) against a list of entries and manufacture
        drift on every key, every run.

        Returns a list of ``{"key", "host_value", "disk_value"}`` dicts for
        every top-level ``cap_config`` key where the two disagree (covers
        both directions: a key with a different value, and a key present in
        only one of the two -- the ``_CONFIG_DRIFT_ABSENT`` sentinel above
        marks the side that lacks it), sorted by key. Returns ``[]`` --
        silently, never raising -- whenever the comparison cannot be
        trusted: no ``cap_config`` dict, no plugin directory, no readable
        file, the file isn't a JSON object, there's no ``"config"`` list, or
        no entry in it matches this capability's own name. This is a
        diagnostic, not a source of truth: the host's in-memory copy stays
        authoritative for this run either way, so a comparison that itself
        fails must say nothing rather than approximate.
        """
        try:
            if not isinstance(cap_config, dict):
                return []
            base = _plugin_dir()
            if not base:
                return []
            config_path = os.path.join(os.path.dirname(base), "config.json")
            with open(config_path, "r", encoding="utf-8") as fh:
                doc = json.load(fh)
            if not isinstance(doc, dict):
                return []
            entries = doc.get("config")
            if not isinstance(entries, list):
                return []
            disk = None
            for entry in entries:
                if isinstance(entry, dict) and entry.get("capability") == self.get_name():
                    candidate = entry.get("cap_config")
                    if isinstance(candidate, dict):
                        disk = candidate
                    break
            if disk is None:
                return []
            missing = object()
            drift = []
            for key in sorted(set(cap_config) | set(disk)):
                host_val = cap_config.get(key, missing)
                disk_val = disk.get(key, missing)
                if host_val != disk_val:
                    drift.append({
                        "key": key,
                        "host_value": (
                            self._CONFIG_DRIFT_ABSENT if host_val is missing
                            else host_val
                        ),
                        "disk_value": (
                            self._CONFIG_DRIFT_ABSENT if disk_val is missing
                            else disk_val
                        ),
                    })
            return drift
        except Exception:
            # Absent/unreadable/malformed config.json (or anything else
            # about this comparison) must be silent, not a crash -- the
            # plugin runs fine where that file doesn't exist at all.
            return []

    def _config_drift_warning_suffix(self):
        """Loud fragment naming every ``cap_config`` key where the running (host
        in-memory) copy disagrees with ``config.json`` on disk, or ``""`` if
        there is none -- the same host-log line and reasoning as
        ``_inert_config_key_warning``/``_unknown_top_level_key_warning``
        (a field in an unread report is not loud).
        This is a WARNING only: the host's copy is what THIS run used and
        nothing here changes that -- it only tells the user a restart is needed
        for a prior edit to take effect.

        Each value is truncated to ``_CONFIG_DRIFT_VALUE_MAX_CHARS`` --
        "report the differing keys with both values", not the whole
        config: an unrecognized-key or scalar drift is a few characters, but
        a key like ``families`` carries a list of dicts that would otherwise
        put two whole family lists on the one host-log line this is meant
        to keep readable.
        """
        drift = getattr(self, "_config_drift", None)
        if not drift:
            return ""

        def _fmt(value):
            text = repr(value)
            limit = self._CONFIG_DRIFT_VALUE_MAX_CHARS
            if len(text) > limit:
                text = text[:limit] + "...<truncated>"
            return text

        detail = "; ".join(
            "{}={} (file: {})".format(
                entry["key"], _fmt(entry["host_value"]), _fmt(entry["disk_value"])
            )
            for entry in drift
        )
        return (
            ", WARNING: running config differs from config.json on disk "
            "({} key(s); restart OrcaSlicer to apply the file, this run "
            "used what was already loaded): {}".format(len(drift), detail)
        )

    def _unknown_top_level_key_warning(self, unknown_keys):
        """Loud warning fragment for one or more unrecognized top-level
        ``cap_config`` keys, or ``""`` if there are none -- appended to the
        SAME dispatcher host-log success line
        ``_tip_gap_warning_suffix``/``_merge_truncation_warning_suffix``/
        ``_opposing_entry_warning_suffix`` use (a field in an unread report is
        not loud), and also written into the diagnostics JSON via
        ``report["unknown_top_level_config_keys"]``.

        NOT fail-closed, deliberately, unlike the check ``_build_families``
        runs on a "families" list entry (which DOES raise ``SpecError``). A
        families-list entry is unambiguously this plugin's own user-authored
        data; the top-level ``cap_config`` dict is round-tripped by the host
        itself through ``self.get_config()``/``self.set_config()`` (the
        Plugins dialog's Config tab), and nothing available to this plugin
        proves the host never adds a key of its own (a version marker, UI
        state) into that same dict. The observed wrapper shape
        ``{"cap_config": ..., "capability": ..., "capability_type": ...,
        "plugin_key": ..., "plugin_version": ...}`` keeps all host
        bookkeeping as siblings of ``cap_config``, which is suggestive, not
        proof. Failing the whole slice on a key the HOST itself wrote would
        be worse than ignoring an unknown key, so this stays a
        loud, visible, non-fatal warning: every offending key, its
        did-you-mean/wrong-scope message (from
        ``config_schema.unknown_top_level_key_message``, the SAME text
        ``tools/config.py check`` -- which DOES fail closed, since the CLI
        never runs against a host-written file -- already prints), and
        where the config file lives.
        """
        if not unknown_keys:
            return ""
        base = _plugin_dir()
        config_path = (
            os.path.join(os.path.dirname(base), "config.json")
            if base
            else "<plugin directory unavailable>"
        )
        detail = "; ".join(
            unknown_top_level_key_message(k)  # noqa: F821 - supplied by the bundle
            for k in unknown_keys
        )
        return (
            ", WARNING: stored config at {} has {} unrecognized top-level "
            "key(s), IGNORED (not applied -- defaults used instead): {} -- "
            "run `python tools/config.py check` for the same diagnosis".format(
                config_path, len(unknown_keys), detail
            )
        )

    def _build_families(self, raw_families):
        """Convert the stored "families" list (plain JSON) into a tuple of
        ``WebFamily``.

        ``raw_families`` must be a list of dicts, each using only the keys
        ``WebFamily`` accepts (``_FAMILY_KEYS``) — an unknown key or a
        non-dict entry is a configuration mistake, not something to guess
        past, so it raises ``SpecError`` naming the offending entry rather
        than silently ignoring or coercing it. An empty/missing list (the
        default) yields ``()``, which is exactly
        ``StructureSpec.effective_families``'s trigger for the single
        -implicit-family fallback.
        """
        if raw_families is None:
            return ()
        if not isinstance(raw_families, list):
            raise SpecError(  # noqa: F821 - supplied by the bundle
                "stored 'families' must be a list, got {}.".format(
                    type(raw_families).__name__
                )
            )
        families = []
        for i, entry in enumerate(raw_families):
            if not isinstance(entry, dict):
                raise SpecError(  # noqa: F821 - supplied by the bundle
                    "stored 'families' entry {} must be an object, got {}.".format(
                        i, type(entry).__name__
                    )
                )
            unknown = set(entry) - _FAMILY_KEYS
            if unknown:
                raise SpecError(  # noqa: F821 - supplied by the bundle
                    "stored 'families' entry {} has unrecognized key(s) {}; "
                    "accepted keys are {}.".format(
                        i, sorted(unknown), sorted(_FAMILY_KEYS)
                    )
                )
            kwargs = dict(entry)
            if "name" not in kwargs:
                kwargs["name"] = "family {}".format(i)
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
            families.append(WebFamily(**kwargs))  # noqa: F821 - supplied by the bundle
        return tuple(families)

    def _check_shell_alignment(self, spec, resolved):
        """Refuse a config whose web root would start over hollow layers.

        Spiral perimeters begin at ``layer.id() >= bottom_shell_layers``
        (LayerRegion.cpp:80-83, GCode.cpp:4539-4550). If the solid root ends
        after that, the first slotted layer lays both web walls across a
        one-wall hollow ring — an unsupported drooping extrusion right above
        the base, and a genuine print hazard rather than a cosmetic one.
        """
        raw = resolved.get("bottom_shell_layers")
        try:
            bottom = int(float(raw))
        except (TypeError, ValueError):
            return  # unreadable; the slice may still be fine, do not guess
        if spec.root_solid_layers > bottom:
            raise SpecError(  # noqa: F821 - supplied by the bundle
                "root_solid_layers={} exceeds bottom_shell_layers={}: the first "
                "slotted layer would print its web walls across hollow spiral "
                "rings. Raise bottom_shell_layers to at least {} in the process "
                "preset, or lower root_solid_layers.".format(
                    spec.root_solid_layers, bottom, spec.root_solid_layers
                )
            )

    def _outer_wall_width_mm(self, resolved):
        raw = resolved.get("outer_wall_line_width")
        try:
            width = float(raw)
        except (TypeError, ValueError):
            raise SpecError(  # noqa: F821 - supplied by the bundle
                "could not resolve outer_wall_line_width (got {!r}); the slot "
                "gap cannot be derived from bead width without it".format(raw)
            )
        if width <= 0:
            raise SpecError(  # noqa: F821 - supplied by the bundle
                "outer_wall_line_width resolved to {!r}; expected a positive "
                "width in mm".format(width)
            )
        return width

    @staticmethod
    def _print_top_z(z_bounds):
        """Z of the PRINT TOP: the mesh's own highest Z, whichever anatomical end
        (tip or root) is there. The checks are print-top extremum checks (the
        last layers, where the slice plane crosses the final face and
        quantization artifacts live), not anatomical-tip checks. Pointing them
        at the resolved tip end instead fails a tip-face-down print at its
        last layer (a sliver of the tilted root face). The bed end never needs
        them: ``root_solid_layers`` skips the first layers."""
        return z_bounds[1]

    @staticmethod
    def _drift_reference_note(spec, root_at):
        """Record the drift reference frame. The family/spec ``z_drift_*``
        lattice drift is a pure function of ``print_z`` (distance from the
        BED), so on a tip-face-down print it runs from the tip instead of from
        the root. It is NOT keyed on distance-from-root and it does NOT raise:
        a plus/minus lattice pair (``z_drift_mm_per_mm`` +-1.0) is symmetric
        under the mirror (it changes the lattice phase, not the structure's
        presence), so a guard that raised would make a valid tip-down slice
        impossible. The fact is recorded so nobody reads the drift as
        root-relative. Keying ``LayerContext`` drift on distance-from-root
        would be needed for a root-relative drift."""
        configured = spec.z_drift_mm_per_mm is not None or any(
            getattr(f, "z_drift_mm_per_mm", None) is not None
            or getattr(f, "z_drift_chord_frac_per_mm", None) is not None
            for f in spec.families
        )
        return {
            "drift_configured": bool(configured),
            "drift_reference": "print_z (distance from the bed)",
            "drift_mirrored_vs_root_relative": bool(
                configured and root_at == "top"),
        }

    def _spar_latch_report_snapshot(self):
        """The run's ``spar_latch`` report block with the online counters
        folded in (they change while layers are processed, so they are read
        at report time rather than copied at precompute time)."""
        rep = getattr(self, "_spar_latch_report", None)
        if rep is None:
            return None
        out = dict(rep)
        out["online"] = {
            "armed_at_layer": getattr(self, "_latch_online_arm_index", None),
            "suppressed_layer_count": getattr(
                self, "_latch_online_suppressed", 0),
        }
        return out

    def _resolve_root_end_and_latch(self, ctx, spec, sectioner, generator,
                                    wall_width):
        """Resolve the root end and precompute the spar-socket latch, once, before
        layer 0.

        Reads the layer list up front (``ctx.object.layers()``), sections the
        mesh at every layer's own ``slice_z`` -- the same plane
        ``_channels_for_layer`` will use -- and records, per layer, the
        spar-channel ``fit``/``not_fit``/``absent`` status via
        ``StructureGenerator.spar_channel_status`` (the generator's own fit
        check, not a copy of it) plus the largest interior-loop diameter
        (the CHANNEL signal: a 3 mm alignment pin never outvotes a 10.5 mm
        spar). The root end then comes from ``aircraft_core.root_end``
        (area + channel, fail closed on disagreement, ``spec.root_end``
        override) and ``aircraft_core.spar_latch.walk_latch`` walks the
        statuses root -> tip.

        Consumption differs by placement, deliberately: with the root at the
        BED the online latch stays in charge (the walk is reported as a
        cross-check it can be compared against), with
        the root at the TOP the precomputed suppression set drives
        ``_apply_spar_socket_latch`` because the online latch cannot know which
        end it met first.
        """
        t0 = time.perf_counter()
        mode = spec.root_end
        # A sectioner that exposes no vertices / Z extent (a duck-typed
        # stand-in) yields an indeterminate AREA signal and evaluates no
        # layers; the default (root at the bed) then applies.
        verts = getattr(sectioner, "vertices", None) or []
        z_lo = getattr(sectioner, "z_min", None)
        z_hi = getattr(sectioner, "z_max", None)
        if z_lo is None or z_hi is None:
            z_lo = z_hi = None
        area = (
            area_signal(verts)  # noqa: F821 - supplied by the bundle
            if verts else {"end": None, "note": "no vertices exposed"}
        )
        report = {
            "config": mode, "root_end": None, "root_at": None,
            "decision_source": None, "area_signal": area,
            "channel_signal": None, "signals_agree": None,
            "override_contradicts_signal": [],
        }
        self._root_end_report = report
        latch = {
            "mode": None, "precompute_ms": None, "layers_total": 0,
            "layers_evaluated": 0, "status_counts": {},
            "evaluation_errors": {"count": 0, "first": None},
            "predicted": None, "unforeseen_socket_drops": [],
        }
        self._spar_latch_report = latch

        if mode == ROOT_END_BED:  # noqa: F821 - supplied by the bundle
            # Explicit bed: nothing to resolve, nothing to walk -- the online
            # latch is exactly right. Signals other than AREA are skipped so a
            # root-down slice pays nothing.
            decision = resolve_root_end(  # noqa: F821 - supplied by the bundle
                mode, area, {"end": None, "skipped": "root_end=bed"})
            report.update({k: decision[k] for k in (
                "root_end", "root_at", "decision_source", "signals_agree",
                "override_contradicts_signal")})
            report["channel_signal"] = decision["channel_signal"]
            latch["mode"] = "online_root_at_bed_precompute_skipped"
            latch["precompute_ms"] = round(
                (time.perf_counter() - t0) * 1000.0, 3)
            return

        layers = list(ctx.object.layers())
        n = len(layers)
        latch["layers_total"] = n
        print_zs = [float(layer.print_z) for layer in layers]
        slice_zs = [self._section_plane_for(layer)[0] for layer in layers]
        pre_ops = OrcaGeometryOps()  # noqa: F821 - private, uncounted
        statuses = [None] * n
        max_dia = [None] * n
        for index in range(n):
            # Polled sparsely: the per-layer loop below owns the one-poll-
            # per-layer contract; this pass only has to stay interruptible.
            if index % 64 == 63 and ctx.cancelled():
                raise CanceledSlice()
            z = slice_zs[index]
            if z_lo is None or z <= z_lo or z >= z_hi:
                continue
            try:
                loops = sectioner.section_at(z)
                outer = [lp for lp in loops if not lp.is_interior]
                inner = [lp for lp in loops if lp.is_interior]
                if inner:
                    max_dia[index] = max(
                        lp.equivalent_diameter_mm for lp in inner)
                if len(outer) != 1:
                    # Multi-island / empty sections degrade before
                    # generation: neutral to the latch, exactly like a layer
                    # the online latch never saw. Root-solid layers (the
                    # first ``root_solid_layers`` at the BED) are evaluated
                    # too, deliberately: on a root-at-top part they are the
                    # tip-most layers, which no walk from the root can let
                    # steer anything, and evaluating them keeps this pass
                    # independent of ``spec.root_solid_layers``.
                    continue
                poly = pre_ops.polygon_from_points(list(outer[0].points))
                frame = LayerFrame.from_polygon(pre_ops, poly)  # noqa: F821
                channels = tuple(
                    DetectedChannel(  # noqa: F821 - supplied by the bundle
                        center_x_mm=lp.centroid[0],
                        center_y_mm=lp.centroid[1],
                        diameter_mm=lp.equivalent_diameter_mm)
                    for lp in inner
                )
                statuses[index] = generator.spar_channel_status(
                    frame, spec,
                    LayerContext(  # noqa: F821 - supplied by the bundle
                        layer_index=index, print_z=print_zs[index],
                        detected_channels=channels),
                    wall_width)
            except CanceledSlice:
                raise
            except Exception as exc:  # noqa: BLE001
                # Not silent: counted and named in the report. A layer whose
                # status could not be computed is neutral to the walk; the
                # real per-layer pass will fail or degrade it on its own if
                # it is genuinely bad.
                errs = latch["evaluation_errors"]
                errs["count"] += 1
                if errs["first"] is None:
                    errs["first"] = {
                        "layer": index, "z": round(slice_zs[index], 4),
                        "error": "{}: {}".format(
                            type(exc).__name__, str(exc)[:200]),
                    }
        latch["layers_evaluated"] = sum(1 for st in statuses if st is not None)
        counts = {}
        for st in statuses:
            counts[str(st)] = counts.get(str(st), 0) + 1
        latch["status_counts"] = counts

        present = channel_presence_from_diameters(max_dia)  # noqa: F821
        channel = channel_span_signal(  # noqa: F821 - supplied by the bundle
            present, slice_zs,
            0.0 if z_lo is None else z_lo, 0.0 if z_hi is None else z_hi)
        try:
            decision = resolve_root_end(mode, area, channel)  # noqa: F821
        except RootEndError as exc:  # noqa: F821 - supplied by the bundle
            report.update({"channel_signal": channel,
                           "decision_source": "REFUSED_signals_disagree"})
            raise TopologyError(0, 0.0, str(exc))  # noqa: F821
        report.update({k: decision[k] for k in (
            "root_end", "root_at", "decision_source", "signals_agree",
            "override_contradicts_signal")})
        report["channel_signal"] = channel
        root_at_bed = decision["root_end"] == END_LOW  # noqa: F821

        walk = walk_latch(statuses, root_at_bed)  # noqa: F821
        suppressed = walk["suppressed"]
        latch["predicted"] = {
            "walk_direction": walk["walk_direction"],
            "arm_layer": walk["arm_index"],
            "arm_z_mm": (None if walk["arm_index"] is None
                         else round(slice_zs[walk["arm_index"]], 4)),
            "suppressed_count": walk["suppressed_count"],
            "suppressed_layer_first": walk["suppressed_first"],
            "suppressed_layer_last": walk["suppressed_last"],
            "suppressed_z_first_mm": (
                None if not suppressed else round(slice_zs[suppressed[0]], 4)),
            "suppressed_z_last_mm": (
                None if not suppressed else round(slice_zs[suppressed[-1]], 4)),
            "fit_layers_before_arm": walk["fit_layers_before_arm"],
        }
        if root_at_bed:
            latch["mode"] = "online_root_at_bed_walk_is_crosscheck"
        else:
            latch["mode"] = "precomputed_root_at_top"
            self._latch_precomputed = {
                "suppressed": frozenset(suppressed),
                "not_fit": frozenset(
                    i for i, st in enumerate(statuses)
                    if st == SPAR_STATUS_NOT_FIT),  # noqa: F821
                "arm_index": walk["arm_index"],
                "arm_z_mm": latch["predicted"]["arm_z_mm"],
            }
        latch["precompute_ms"] = round((time.perf_counter() - t0) * 1000.0, 3)

    def _apply_spar_socket_latch(self, intents, layer_degradations, index=None):
        """Once the spar socket has ENDED on this run, keep it ended. Returns
        ``(intents, layer_degradations)``.

        The generator is single-layer and stateless by design
        (``aircraft_core`` stays ordinary geometry logic), so it can only DROP
        the socket on each layer that cannot hold it. Without this latch a
        section that thins and then thickens would stop and RESTART the socket,
        leaving a blind interior pocket the carbon rod cannot reach. The
        socket is never shrunk to fit, because a channel the rod cannot enter
        is worse than no channel.

        This is a separate method so it can be tested directly: the fake host
        builds only constant-section prisms, so no integration fixture
        produces a thins-then-thickens wing.

        SCOPE, stated because it is a choice and not an accident: the latch
        arms on a ``spar_socket`` DEGRADATION only -- i.e. on "the socket
        stopped fitting". A model whose channel simply ENDS and whose second
        channel appears further outboard produces no degradation, never arms
        the latch, and gets a socket in both channels. That is correct ("the
        MODEL determines spar geometry; config carries policy only"): two
        channels by design are two sockets, not a resume after a loss. The
        latch exists to stop US re-cutting a socket we ourselves ended, not to
        overrule the model.

        Both the ARM check and the RESUME check are scoped
        to a CONFIRMED spar socket only (``WebIntent.spar_confirmed`` --
        see its own docstring). A socket built by
        ``spar_channel_policy="nearest_spar_station"`` disambiguating among
        two or more detected channels degrades under its own kind,
        ``"spar_socket_unconfirmed"`` (``aircraft_core.generator``'s
        ``_build_channel_socket``), never ``"spar_socket"`` -- so it cannot
        arm this latch. And an unconfirmed ``WebIntent`` (``family_name==
        "spar_socket"`` but ``spar_confirmed`` is ``False``) is never
        treated as a "resumed" spar either. Both matter for the same
        reason: an alignment bore that happens to win the "nearest to
        spar_station" contest, then either fails to fit or reappears after
        the REAL spar ended, must never arm or be caught by the real
        spar's own run-level latch.
        """
        pre = getattr(self, "_latch_precomputed", None)
        if pre is not None and index is not None:
            return self._apply_precomputed_spar_latch(
                pre, index, intents, layer_degradations)
        if any(
            d["kind"] == "spar_socket" for d in layer_degradations
        ):
            if index is not None and not self._spar_socket_ended:
                self._latch_online_arm_index = index
            self._spar_socket_ended = True
            return intents, layer_degradations
        if not self._spar_socket_ended:
            return intents, layer_degradations
        resumed = [
            i for i in intents
            if i.family_name == "spar_socket"
            and getattr(i, "spar_confirmed", False)
        ]
        if not resumed:
            return intents, layer_degradations
        if index is not None:
            self._latch_online_suppressed = (
                getattr(self, "_latch_online_suppressed", 0) + 1)
        kept = [i for i in intents if i.family_name != "spar_socket"]
        layer_degradations = list(layer_degradations) + [{
            "kind": "spar_socket_latched",
            "x_mm": round(resumed[0].start[0], 4),
            "reason": (
                "DEGRADED -- spar socket suppressed: it fits again at this "
                "layer, but the socket already ENDED further inboard. "
                "Re-cutting it would leave a blind interior pocket the "
                "carbon rod cannot reach, so it stays ended for the rest "
                "of the run."
            ),
        }]
        return kept, layer_degradations

    def _apply_precomputed_spar_latch(self, pre, index, intents,
                                      layer_degradations):
        """Root-at-TOP form of the latch: consult the walk precomputed before
        layer 0 instead of arming online.

        A layer the walk suppressed drops its CONFIRMED socket and records
        ``spar_socket_latched`` (same kind, so every consumer of the report
        keeps working). A ``spar_socket`` degradation the real generator raised
        on a layer the precompute did not see as ``not_fit`` is NOT allowed to
        arm anything (online, in this direction, it would be the tip-side end);
        it is recorded under
        ``unforeseen_socket_drops`` so a precompute/host disagreement is
        visible rather than silent.
        """
        if any(d["kind"] == "spar_socket" for d in layer_degradations):
            if index not in pre["not_fit"]:
                drops = self._spar_latch_report["unforeseen_socket_drops"]
                if len(drops) < 20:
                    drops.append(index)
            return intents, layer_degradations
        if index not in pre["suppressed"]:
            return intents, layer_degradations
        resumed = [
            i for i in intents
            if i.family_name == "spar_socket"
            and getattr(i, "spar_confirmed", False)
        ]
        if not resumed:
            return intents, layer_degradations
        self._latch_online_suppressed = (
            getattr(self, "_latch_online_suppressed", 0) + 1)
        kept = [i for i in intents if i.family_name != "spar_socket"]
        layer_degradations = list(layer_degradations) + [{
            "kind": "spar_socket_latched",
            "x_mm": round(resumed[0].start[0], 4),
            "reason": (
                "DEGRADED -- spar socket suppressed: it fits at this layer, "
                "but walking from the ROOT end (root at the top of the "
                "print) the rod-reachable channel ENDED at layer {} "
                "(Z={} mm), nearer the tip than the root. Cutting a socket "
                "past that end would leave a blind interior pocket the "
                "carbon rod cannot reach, so it is suppressed from there "
                "to the tip.".format(
                    pre["arm_index"], pre["arm_z_mm"])
            ),
        }]
        return kept, layer_degradations

    def _resolve_ramp_mm(self, resolved):
        """The Z-continuity ramp step ``r``: the LIVE ``layer_height`` (a 45
        degree ramp is one layer of depth per layer). Fails closed when it
        does not resolve -- the rule must never guess its own step."""
        try:
            value = float(resolved.get("layer_height"))
        except (TypeError, ValueError):
            value = None
        if value is None or not value > 0:
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                0, 0.0,
                "z_continuity requires the live print config key "
                "'layer_height' (the ramp step is one layer height); it is "
                "missing or non-positive ({!r}).".format(
                    resolved.get("layer_height")))
        return value

    def _z_rule_report_block(self):
        rule = getattr(self, "_z_rule", None)
        return None if rule is None else rule.summary()

    def _continuity_block(self, report, spec, resolved):
        """The web-continuity check over this run's per-layer web records,
        REPORT-ONLY. Never raises and never changes the slice: any failure here
        is itself recorded in the returned block.

        ``r`` (the ramp step) is the LIVE ``layer_height`` (45 degrees:
        one layer of depth per layer) and ``dx_max`` is half the outer wall
        line width; both are recorded with where they came from."""
        try:
            layers = layers_from_report(report)  # noqa: F821 - supplied by the bundle
            ramp_source = "resolved.layer_height"
            try:
                ramp_mm = float(resolved.get("layer_height"))
                if ramp_mm <= 0:
                    raise ValueError("non-positive layer_height")
            except (TypeError, ValueError, AttributeError):
                zs = sorted(
                    rec["print_z"] for rec in layers
                    if rec.get("print_z") is not None
                )
                diffs = sorted(b - a for a, b in zip(zs, zs[1:]) if b - a > 1e-9)
                ramp_mm = diffs[len(diffs) // 2] if diffs else 0.2
                ramp_source = "median print_z step (layer_height unresolved)"
            try:
                wall_mm = float(self._outer_wall_width_mm(resolved))
                wall_source = "resolved.outer_wall_line_width"
            except Exception:
                wall_mm = 0.42
                wall_source = "default 0.42 (outer_wall_line_width unresolved)"
            root_layers = int(spec.root_solid_layers) if spec is not None else 0
            exempt = default_exemptions(layers, root_layers)  # noqa: F821
            family_names = {}
            if spec is not None:
                family_names = {
                    idx: fam.name
                    for idx, fam in enumerate(spec.effective_families())
                }
            family_names.setdefault(-2, "spar_socket")
            family_names.setdefault(-7, "bore_socket")
            dx_max_mm = (
                DEFAULT_DX_MAX_MM  # noqa: F821 - supplied by the bundle
                if spec is None or spec.z_continuity_dx_max_mm is None
                else float(spec.z_continuity_dx_max_mm))
            block = check_continuity(  # noqa: F821 - supplied by the bundle
                layers, ramp_mm=ramp_mm, dx_max_mm=dx_max_mm,
                exempt=exempt, family_names=family_names,
                wall_matching=bool(
                    spec is not None and spec.z_continuity
                    and spec.z_continuity_wall_matching),
            )
            block["dx_max_mm"] = dx_max_mm
            block["z_continuity_active"] = bool(
                spec is not None and spec.z_continuity)
            block["ramp_source"] = ramp_source
            block["outer_wall_line_width_mm"] = wall_mm
            block["outer_wall_line_width_source"] = wall_source
            block["root_solid_layers"] = root_layers
            return block
        except Exception as exc:  # noqa: BLE001 - report-only, never fatal
            return {"error": "{}: {}".format(type(exc).__name__, exc)}

    def _resolve_extremum_band_mm(self, resolved):
        """The Z band around the mesh's own TOP within which
        ``tip_alignment_skip_enabled`` may apply -- see the module-level
        comment above ``THETA_EPSILON_DEG`` for the full derivation.
        "TOP" means the PRINT TOP (the mesh's max Z), whichever anatomical end
        is there -- not the anatomical tip.

        ``layer_height`` alone governs this band: the band is consulted
        ONLY at the top extremum (never the root), and ``layer_height`` --
        not ``initial_layer_print_height``, which governs the FIRST layer
        at the root -- is what the derivation's own ``generate_object_
        layers`` reasoning is built on. ``initial_layer_print_height`` is
        used only as a fallback when ``layer_height`` itself fails to
        resolve, so an unusual root layer height can never make this band
        wider than the top-extremum derivation supports.

        Returns ``None`` (never a fabricated default) when NEITHER value
        resolves to a usable positive number -- the caller's only correct
        response to ``None`` is to run the mesh-alignment check at full
        strictness on every term, everywhere. An unresolved layer height
        must never make the guard MORE permissive; it can only fail to make
        it selectively lenient near the top extremum, which is the safe
        (fail-closed) default.
        """
        def _positive_float(key):
            try:
                value = float(resolved.get(key))
            except (TypeError, ValueError):
                return None
            return value if value > 0 else None

        layer_height = _positive_float("layer_height")
        if layer_height is not None:
            return layer_height
        # Fallback only: ``layer_height`` itself did not resolve. Some
        # answer beats none here (see the docstring's "must never make the
        # guard MORE permissive" -- ``initial_layer_print_height`` is not
        # claimed to GOVERN the top extremum, only offered as the next-best
        # available number before giving up and returning ``None``).
        return _positive_float("initial_layer_print_height")

    def _count_vertices(self, polygons):
        """Total vertex count across ``polygons``, or ``None`` if any piece
        refuses to report its outline (e.g. a cut, non-rectangular section on
        a backend that only materializes bounds/area for such a shape — see
        ``fake_orca.py``'s ``_PolygonNotched.points()``). Never raises: an
        unavailable count is reported honestly as ``None``, not guessed."""
        total = 0
        for polygon in polygons:
            try:
                total += len(polygon.contour.points)
            except Exception:
                return None
        return total

    def _contour_perimeter_mm(self, ops, polygon):
        """Closed-contour perimeter length in mm, read the same way
        ``_count_vertices`` reads vertices — ``contour_points_mm`` gives an
        ordered ring of points with no explicit closing duplicate, so the
        last-to-first segment is added explicitly.

        Structural summary input (``RunDiagnostics.record_structure_layer``):
        this is the UN-slotted region's own outer perimeter, used as the "skin"
        side of the wall-length proxy — see that method's docstring for exactly
        what the resulting number does and does not mean. Never raises past
        what ``contour_points_mm`` itself raises; a caller with fewer than 2
        points would get a degenerate (zero) length rather than an error, which
        is acceptable here since this is diagnostics, not a geometry invariant.
        """
        points = ops.contour_points_mm(polygon)
        if len(points) < 2:
            return 0.0
        total = 0.0
        for (x0, y0), (x1, y1) in zip(points, points[1:] + points[:1]):
            total += math.hypot(x1 - x0, y1 - y0)
        return total

    def _intent_depth_mm(self, intent):
        """Straight-line distance between one ``WebIntent``'s ``start`` and
        ``end`` points, in mm — this module's approximation of "how far this
        web reaches in", used as the per-web depth input to
        ``RunDiagnostics.record_structure_layer`` (see that method's
        docstring for the ``2 * depth`` wall-length model this feeds)."""
        (x0, y0), (x1, y1) = intent.start, intent.end
        return math.hypot(x1 - x0, y1 - y0)

    def _is_full_depth_intent(self, intent, families):
        """True if EVERY family this intent belongs to is full-depth
        (``WebFamily.web_depth_mm is None``) — the web-tip air gap
        (``RunDiagnostics.web_tip_air_gap_mm``) population.

        ``families`` is ``spec.effective_families()``, already resolved:
        when ``spec.families`` is non-empty, each entry's ``web_depth_mm``
        is read VERBATIM (``StructureSpec.effective_families`` does not
        re-inherit a per-family ``None`` from the spec-level default in
        that branch), so ``None`` here always means "this family's own
        webs are placed against the far skin", never "unset, ask the
        spec". Indexed by ``intent.family_index``/
        ``intent.merged_family_indices`` exactly as ``generator.py``
        assigns them (``for family_index, family in
        enumerate(spec.effective_families())`` in ``_generate_family``).

        A merged node's ``merged_family_indices`` may span a
        full-depth family and a depth-limited one; that pairing's actual
        placed depth is resolved by ``_merge_node_component`` (candidate
        (a) vs (b), whichever is shallower — see that method's own
        docstring) and is not necessarily full depth even when one
        constituent is. Rather than re-deriving that comparison here (a
        second, independent measurement of a quantity ``generator.py``
        already computed once, which could drift out of agreement), this
        method is conservative: it counts a merged node as full-depth only when
        EVERY constituent family is itself full-depth, which is the one
        case ``_merge_node_component`` is guaranteed to resolve to the
        full-depth placement. A merge with a depth-limited constituent is
        excluded from the count even on the (unusual) layer where it
        happens to reach full depth anyway — undercounting here, never
        overcounting, and never reports the optimistic number.
        """
        member_indices = intent.merged_family_indices or (intent.family_index,)
        # SYNTHETIC members carry sentinel indices, not positions in
        # ``families``: the scar is -1, the spar socket is -2,
        # ``feature_lattice`` omissions (``OmittedWeb``-only, never a built
        # intent) use -3, feature-dense-lattice fallback sentinels use
        # -4/-5/-6, and the "more than one detected channel" policy's
        # plain-bore socket uses -7
        # (``aircraft_core.generator._generate_bore_socket_from_channel`` --
        # deliberately its own sentinel, not -3, even though every lookup here
        # guards on ``idx >= 0`` below). None of these are family webs
        # and none have a ``web_depth_mm`` to read.
        #
        # Indexing ``families`` with a sentinel is wrong in two ways: with few
        # families it raises IndexError, and with enough families Python's
        # negative indexing silently resolves it to a REAL family and reports
        # the socket's depth as that unrelated family's -- a wrong number with
        # no error, which is the worse failure.
        real = [idx for idx in member_indices if idx >= 0]
        if not real:
            return False
        return all(families[idx].web_depth_mm is None for idx in real)

    def _classify_declared_pair_overlaps(self, intents, spec, frame, layer_index, print_z):
        """Thin edge wrapper. The actual classification
        (``classify_declared_pair_overlaps``) lives in
        ``aircraft_core/generator.py``, beside
        ``StructureGenerator._candidate_stations`` -- the anchoring formula it
        inverts. A copy at the plugin edge, in a different package from the
        formula it inverts, would have nothing pinning the two together and
        could silently disagree. See that
        function's own docstring for full scope/rationale. Resolved by name
        once ``tools/build_plugin.py`` flattens ``aircraft_core`` and this
        capability into one namespace (``generator.py`` precedes the capability
        source in ``CORE_MODULE_ORDER``).
        """
        return classify_declared_pair_overlaps(intents, spec, frame, layer_index, print_z)

    def _node_merge_truncation_ratio(
        self, intent, families, frame, effective_skin_margin_mm
    ):
        """The merged-node TRUNCATION RATIO for one merged ``WebIntent`` — the
        merged slot's realised depth divided by its DEEPEST CONSTITUENT's own
        requested depth — or ``None`` if it cannot be honestly computed (see
        below). ``None`` for a non-merged intent (``merged_family_indices``
        empty): there is nothing to truncate.

        REALISED DEPTH: ``_intent_depth_mm(intent)`` — the merged slot's
        own placed depth, exactly as built.

        REQUESTED DEPTH, per constituent family in
        ``intent.merged_family_indices``:

        - A depth-limited family (``WebFamily.web_depth_mm is not None``)
          has an EXACT, config-level requested depth — no measurement
          needed, no approximation.
        - A full-depth family (``web_depth_mm is None``) has no fixed
          requested-depth NUMBER: full depth is "as deep as this station's
          local geometry allows", which ``generator.py``'s
          ``_merge_node_component`` already computes once, as its own
          candidate (b), but does not return on ``WebIntent`` (only the
          final chosen ``start``/``end`` cross the ``GenerationResult``
          boundary — see that method's own docstring). Rather than reusing
          a stale or narrower reading, this
          re-measures candidate (b) directly, over the SAME merged
          interval (``intent.start[0]``, ``intent.width_mm``) and with the
          SAME formula and constants ``_merge_node_component`` itself
          uses: ``intent.start[1]`` (the already-realised, re-measured
          entry — reused verbatim rather than re-measured a second time,
          since the merge's own entry re-measurement already accounts for
          the wider span) minus ``local_lower_worst_case(...) +
          effective_skin_margin_mm + FULL_DEPTH_MARGIN_HEADROOM_MM``. This
          is a SECOND CALL SITE for one formula, not a second, divergent
          formula — ``FULL_DEPTH_MARGIN_HEADROOM_MM`` is imported from the
          same ``generator.py`` constant ``diagnostics.py`` already
          imports for the web-tip air gap, and ``local_lower_worst_case``/
          ``DEFAULT_PROBE_WIDTH_MM`` are the same public functions/constant
          ``generator.py`` itself calls — so drift between the two call
          sites would require editing one and not the other, the same
          discipline applied to the tip-gap calculation.

        The deepest constituent's requested depth is the MAXIMUM of the
        above across ``intent.merged_family_indices``. The ratio is
        ``realised / deepest_requested``.

        Returns ``None`` (rather than a fabricated number) when EVERY
        constituent's contribution turns out to be unreachable: no
        ``frame``/``effective_skin_margin_mm`` was given AND the
        constituent needing them is full-depth, or the local
        re-measurement itself raises ``ValueError`` (mirrors
        ``_merge_node_component``'s own halo belt-and-suspenders fallback —
        a diagnostic re-measurement must never itself fail a slice, and
        must never silently substitute a wrong number either), or the
        resulting deepest-requested depth is not strictly positive (a
        degenerate placement this diagnostic cannot meaningfully rate). A
        merge with AT LEAST ONE reachable constituent (e.g. one
        depth-limited member alongside an unreachable full-depth one)
        still reports a ratio, conservatively, from whatever WAS reachable
        — this can only ever UNDERSTATE the true depth loss, never overstate
        it, matching ``_is_full_depth_intent``'s own "undercount, never
        overcount" posture.
        """
        member_indices = intent.merged_family_indices
        if not member_indices:
            return None
        # Same sentinel problem as _is_full_depth_intent: -1 (scar) and -2
        # (socket) are not positions in ``families``.
        member_indices = [idx for idx in member_indices if idx >= 0]
        if not member_indices:
            return None
        deepest_requested = None
        for idx in member_indices:
            depth_mm = families[idx].web_depth_mm
            if depth_mm is not None:
                candidate = depth_mm
            else:
                if frame is None or effective_skin_margin_mm is None:
                    # No geometry to re-measure against -- this
                    # constituent's requested depth genuinely is NOT
                    # reachable from what this call was given (see this
                    # method's own docstring). Skip only its contribution
                    # rather than fabricate a number; a caller with no
                    # full-depth constituents at all is unaffected.
                    continue
                try:
                    y_min_full = local_lower_worst_case(  # noqa: F821 - supplied by the bundle
                        frame.ops,
                        frame.polygon,
                        intent.start[0],
                        intent.width_mm,
                        DEFAULT_PROBE_WIDTH_MM,  # noqa: F821 - supplied by the bundle
                    )
                except ValueError:
                    # Belt-and-suspenders, matching _merge_node_component's
                    # own halo fallback: this re-measurement is a
                    # diagnostic convenience, not a proof, and must never
                    # fail the slice or silently corrupt the reported
                    # ratio -- skip only this constituent's contribution.
                    continue
                candidate = intent.start[1] - (
                    y_min_full
                    + effective_skin_margin_mm
                    + FULL_DEPTH_MARGIN_HEADROOM_MM  # noqa: F821 - supplied by the bundle
                )
            if deepest_requested is None or candidate > deepest_requested:
                deepest_requested = candidate
        if deepest_requested is None or deepest_requested <= 0:
            return None
        return self._intent_depth_mm(intent) / deepest_requested

    def _merge_truncation_warning_suffix(self, diagnostics):
        """The merged-node depth-truncation warning fragment for the dispatcher's
        host-log result line, or ``""`` if it should not fire — appended to
        the line the user already reads, like ``_tip_gap_warning_suffix``.

        Fires whenever ``diagnostics.truncated_merge_count > 0`` — at
        least one merge lost a quarter or more of its deepest
        constituent's requested depth (see
        ``MERGE_DEPTH_TRUNCATION_THRESHOLD_RATIO``'s module-level comment
        in ``diagnostics.py``). Worded around the actual consequence — a
        deep web reduced to a shallow one's depth at a crossing — rather
        than merely naming the event ("a merge occurred").
        """
        count = diagnostics.truncated_merge_count
        if count <= 0:
            return ""
        return (
            ", WARNING: {} node merge(s) truncated a deeper web to a "
            "shallower one's depth (worst ratio {:.3f} at layer {}, "
            "Z={:.2f} mm) -- the deep member's load path is cut short at "
            "these crossings".format(
                count,
                diagnostics.worst_depth_truncation_ratio,
                diagnostics.worst_depth_truncation_layer_index,
                diagnostics.worst_depth_truncation_print_z,
            )
        )

    def _opposing_entry_warning_suffix(self, diagnostics):
        """The opposing-entry omission warning fragment for the dispatcher's
        host-log result line, or ``""`` if it should not fire — appended to
        the line the user already reads, like
        ``_tip_gap_warning_suffix``/``_merge_truncation_warning_suffix``.

        A single opposing-entry omission is unremarkable on its own — the
        whole point of ``_omit_opposing_conflicts`` (aircraft_core/
        generator.py) is that one tapered station losing its opposing
        stiffener lets the REST of the layer slice cleanly, and that
        omission is already visible per-layer (``omitted`` count) and in
        the full diagnostics JSON. This fires only when a LARGE FRACTION of
        some family's own candidates were lost to this one cause across the
        whole run (``RunDiagnostics.opposing_entry_family_ratios``,
        ``diagnostics.OPPOSING_ENTRY_OMISSION_WARNING_RATIO``) — evidence
        that family's own anchor/pitch choice is structurally out of phase
        with an opposing family, not that one station happened to fall on a
        bad spot.

        Named per-family (not a single run-wide count) because the remedy is
        per-family: it points at WHICH family's anchoring needs revisiting.
        """
        ratios = diagnostics.opposing_entry_family_ratios()
        worst_key = None
        worst_ratio = None
        for key, r in ratios.items():
            if r.ratio >= OPPOSING_ENTRY_OMISSION_WARNING_RATIO:  # noqa: F821
                if worst_ratio is None or r.ratio > worst_ratio:
                    worst_key, worst_ratio = key, r.ratio
        if worst_key is None:
            return ""
        r = ratios[worst_key]
        return (
            ", WARNING: family {} lost {}/{} candidate station(s) "
            "({:.0%}) to opposing-entry collisions -- this family's own "
            "anchor/pitch is structurally out of phase with an opposing "
            "family on this (likely tapered) section, not merely unlucky "
            "at one station".format(
                worst_key, r.opposing_entry_omitted, r.total_candidates, r.ratio
            )
        )

    def _role_yield_warning_suffix(self, diagnostics):
        """The role-yield omission warning fragment for the dispatcher's host-log
        result line, or ``""`` if it should not fire — appended to the line
        the user already reads, like
        ``_tip_gap_warning_suffix``/``_merge_truncation_warning_suffix``/
        ``_opposing_entry_warning_suffix``.

        ``_yield_lattice_to_primary`` (aircraft_core/
        generator.py) omits a ``skin_lattice`` candidate rather than let it
        merge with, and truncate, a ``primary_structure`` candidate at the same
        crossing. Unlike opposing-entry, a role-yield omission at count > 0 is
        NOT itself a defect signal — it is the intended, healthy outcome on any
        config that combines a lattice with a primary member, and a warning
        gated on a bare nonzero count would fire on every correctly-configured
        part and be worthless noise (see
        ``diagnostics.ROLE_YIELD_OMISSION_WARNING_RATIO``'s module -level
        comment for the full reasoning). This fires only when a LARGE FRACTION
        of some lattice family's own candidates were lost to this one cause
        across the whole run (``RunDiagnostics.role_yield_family_ratios``,
        ``diagnostics.ROLE_YIELD_OMISSION_WARNING_RATIO``) — evidence that
        family is barely present where it crosses that primary member, not that
        a handful of stations gave way as designed.

        Named per-family (not a single run-wide count) for the same reason
        ``_opposing_entry_warning_suffix`` is: the fix this warning points
        at is per-family (that lattice family's own pitch/anchor/depth
        against the primary member it keeps losing to).
        """
        ratios = diagnostics.role_yield_family_ratios()
        worst_key = None
        worst_ratio = None
        for key, r in ratios.items():
            if r.ratio >= ROLE_YIELD_OMISSION_WARNING_RATIO:  # noqa: F821
                if worst_ratio is None or r.ratio > worst_ratio:
                    worst_key, worst_ratio = key, r.ratio
        if worst_key is None:
            return ""
        r = ratios[worst_key]
        return (
            ", WARNING: family {} lost {}/{} candidate station(s) "
            "({:.0%}) to role-yield at primary crossings -- half or more "
            "of this lattice family's own candidate stations were dropped "
            "where they cross primary_structure members so those members "
            "keep full depth; the lattice is barely present along its own "
            "run, not merely giving way at a handful of crossings as "
            "designed".format(
                worst_key, r.role_yield_omitted, r.total_candidates, r.ratio
            )
        )

    def _transform(self, ctx, spec, resolved, diagnostics, metrics=None):
        # Read once, here, so ``_write_diagnostics``
        # (called from ``execute()``, which has no ``spec`` in scope at that
        # point) knows whether to serialize ``self._host_sections_sidecar``
        # without needing the spec threaded through. Reset to False at the
        # run-boundary reset (top of ``execute()``) before this is ever
        # reached, so a run that raises before ``_transform`` writes no sidecar
        # rather than reusing a prior run's flag.
        self._sidecar_enabled = bool(spec.record_host_geometry_sidecar)
        ops = OrcaGeometryOps()
        counting_ops = None
        if metrics is not None:
            # Installed only when metrics are enabled, per aircraft_core
            # /metrics.py's module docstring: the uninstrumented hot path
            # (``ops`` called directly) is unaffected when this is None.
            counting_ops = CountingGeometryOps(ops)  # noqa: F821
            ops = counting_ops
        generator = StructureGenerator()  # noqa: F821 - supplied by the bundle
        builder = SlotBuilder()  # noqa: F821 - supplied by the bundle
        wall_width = self._outer_wall_width_mm(resolved)
        validator = TopologyValidator(ops, safety_offset_mm=wall_width)  # noqa: F821
        # skin_line_width_mm is a per-run constant (the resolved outer wall
        # line width in force for this whole slice), so it is recorded once
        # here rather than per layer — see
        # ``RunDiagnostics.set_skin_line_width_mm``'s docstring.
        diagnostics.set_skin_line_width_mm(wall_width)
        # Web-tip air gap input: the REQUESTED skin_margin_mm, recorded
        # once per run alongside the line width above — see
        # ``RunDiagnostics.set_skin_margin_mm``'s docstring for why this is
        # ``spec.skin_margin_mm`` and not a per-layer rotation-corrected
        # reading.
        diagnostics.set_skin_margin_mm(spec.skin_margin_mm)

        # The Z-continuity rule, behind ``spec.z_continuity`` (default False:
        # ``self._z_rule`` stays None and every hook below is skipped).
        # Rebuilt every run: its per-web state must never leak across slices
        # on a reused instance.
        self._z_rule = None
        if spec.z_continuity:
            self._z_rule = ZContinuityRule.from_spec(  # noqa: F821 - bundle
                spec, ramp_mm=self._resolve_ramp_mm(resolved),
                safety_offset_mm=wall_width)

        # Recover the ORIGINAL mesh once per run. Spiral Vase has already
        # stripped interior holes from the slices by the time we see them, so
        # the model's spar channel can only come from here. Built once because
        # a ~39k-facet wing costs ~20 ms to read and index, against ~2.9 ms per
        # layer to section.
        sectioner = None
        mesh_note = None
        # Reset every run, never carried over from a prior slice on a reused
        # capability instance (the same discipline as
        # ``_last_geometry``/``_root_layers``).
        self._alignment_theta_deg = 0.0
        self._alignment_fatal_error = None
        self._chord_axis_unavailable = False
        # Spar-socket latch. Reset HERE, at the true run boundary, with the
        # rest of the per-run state. A latch that leaked across runs on a
        # host-reused capability instance would suppress the socket on a wing
        # that never needed it suppressed.
        self._spar_socket_ended = False
        self._latch_precomputed = None
        self._latch_online_arm_index = None
        self._latch_online_suppressed = 0
        self._root_end_report = None
        self._spar_latch_report = None
        # Resolved once per run, from the SAME ``resolved`` config dict every
        # other per-run constant here comes from -- never re-read from ``ctx``
        # per layer. ``None`` means the guard runs at full strictness
        # everywhere; see ``_resolve_extremum_band_mm``'s own docstring. Also
        # reset unconditionally at the run boundary in ``execute()``; this
        # reassignment is the normal-path value once ``_transform`` is reached.
        self._alignment_extremum_band_mm = self._resolve_extremum_band_mm(resolved)
        # The LIVE ``slice_closing_radius`` from the captured print config,
        # kept raw and parsed (fail closed, never defaulted) where the guard
        # first needs it.
        self._closing_radius_raw = resolved.get(CLOSING_RADIUS_KEY)  # noqa: F821
        try:
            got = self._build_mesh_sectioner(ctx)
            if got is not None:
                sectioner, _mesh_z_off, _mesh_bounds = got
        except TopologyError:  # noqa: F821 - supplied by the bundle
            raise
        except Exception as exc:  # noqa: BLE001
            # Degrading to "no channel" here would FAIL OPEN: with a config
            # carrying no spar geometry, "no channel" means the generator
            # builds NO SOCKET, and a wing with no spar bore still satisfies
            # every topology invariant and passes gcode_spiral_check (a
            # missing member removes geometry without breaking topology).
            # The error is recorded here and raised at the first body layer,
            # where a layer index and Z exist to name.
            mesh_note = "{}: {}".format(type(exc).__name__, exc)
        self._mesh_alignment = {
            "mesh_bounds_after_transform": (
                None if sectioner is None else _mesh_bounds),
            "mesh_z_shift_applied": None if sectioner is None else _mesh_z_off,
            "sectioner_built": sectioner is not None,
            "error": mesh_note,
            "triangles": None if sectioner is None else sectioner.triangle_count,
            "layers_sectioned": 0,
            "layers_failed": 0,
            "worst_bbox_delta_mm": 0.0,
            "worst_bbox_delta_layer": None,
            "worst_bbox_delta_term": None,
            "worst_by_term": {},
            # The top-extremum band this run used (None if neither
            # layer_height nor initial_layer_print_height resolved -- see
            # ``_resolve_extremum_band_mm``): every layer where
            # ``tip_alignment_skip_enabled`` actually left a layer unmutated
            # (never a suppressed TERM -- see ``_check_mesh_alignment``, this
            # is a whole-layer skip or nothing), capped like ``anomalies``
            # below so a pathological run cannot inflate the report without
            # bound.
            "tip_alignment_band_mm": self._alignment_extremum_band_mm,
            "tip_alignment_skipped_layer_count": 0,
            "tip_alignment_skipped_first_z": None,
            "tip_alignment_skipped_layers": [],
            # how many layers this run needed ``MeshSectioner``'s
            # dangling-endpoint retry to succeed (``sectioner.last_nudge_mm``
            # != 0.0 on return), and the first occurrence -- diagnostic only,
            # deliberately NOT a ``Degradation``: a successful retry produces
            # the same cross-section by any physical measure (see
            # ``_RETRY_NUDGES_MM``'s own docstring).
            "section_nudge_count": 0,
            "section_nudge_first": None,
            # All six alignment deltas, per layer, for every
            # layer within roughly 10 layer heights of the mesh's own top Z
            # -- independent of whether the guard actually skipped or
            # raised on that layer, and independent of
            # ``tip_alignment_skip_enabled``. The band alone is one or two
            # layers and cannot distinguish a step from a trend; this is
            # what lets a single run's report settle that question.
            # ``None`` (unresolved band) records
            # nothing here -- "10 layer heights" has no meaning without a
            # layer height to multiply.
            "near_top_alignment_deltas": [],
            # Diagnostic-only. UNLIKE ``near_top_alignment_deltas`` above, this
            # is NOT windowed to the mesh's own top extremum and NOT capped:
            # one entry (six rounded floats) for EVERY sectioned layer, so a
            # layer far from the top is visible here even though it cannot be
            # seen in ``near_top_alignment_deltas`` by construction. 1175
            # layers x 6 floats is negligible next to what this report
            # already carries.
            "all_layer_alignment_deltas": [],
            # run-level summary of the mesh-side slice-closing emulation (see
            # ``_check_mesh_alignment``). ``changed_layers`` lists (capped)
            # every layer where closing changed the outer loop count or moved
            # any of the six terms by more than the alignment tolerance.
            "closing": {
                "radius_mm": None,
                "layers_checked": 0,
                "loop_count_changed_layers": 0,
                "term_changed_over_tolerance_layers": 0,
                "changed_layers": [],
            },
            # Diagnostic-only. A handful (see
            # ALIGNMENT_CONTOUR_DUMP_CAP) of full point-list dumps -- the
            # mesh's own chosen ("best") loop, the host's own frame outline,
            # and the exact Z values used -- for any layer that fails this
            # guard or comes within 2x tolerance of failing on any of the
            # six terms. Written BEFORE this method's own raises, so a
            # layer that goes on to fail the slice still has its dump in
            # the report the TopologyError handler writes (see
            # ``_check_mesh_alignment``'s own comment for the mechanism).
            "alignment_contour_dumps": [],
            "interior_loop_counts": {},
            "first_error": None,
            "anomalies": [],
            # WHERE the model put the channel, recorded per run so two
            # slices of two CAD variants can be compared directly without
            # re-deriving anything.
            "channel_cx_min": None,
            "channel_cx_max": None,
            "channel_dia_min": None,
            "channel_dia_max": None,
            "first_channel": None,
            # Sensitivity-aware tolerance (off by default --
            # ``StructureSpec.mesh_alignment_sensitivity_enabled``). Zero/empty
            # on every run where the field is False, since the probe is then
            # never attempted at all.
            "sensitivity_enabled": spec.mesh_alignment_sensitivity_enabled,
            "sensitivity_probe_count": 0,
            "sensitivity_probe_ms_total": 0.0,
            "sensitivity_unmeasurable_count": 0,
            "sensitivity_ceiling_hit_count": 0,
            "sensitivity_widened_term_count": 0,
            "sensitivity_adjustments": [],
        }

        # Root end + spar-socket latch precompute, before layer 0. Needs the
        # sectioner (a run without one fails closed at the first body layer).
        if sectioner is not None:
            self._resolve_root_end_and_latch(
                ctx, spec, sectioner, generator, wall_width)
            if self._root_end_report is not None:
                self._root_end_report["drift"] = self._drift_reference_note(
                    spec, self._root_end_report.get("root_at"))

        layers_seen = 0
        for index, layer in enumerate(ctx.object.layers()):
            if ctx.cancelled():
                raise CanceledSlice()
            layers_seen += 1

            # Write a fresh, minimally-complete ``_last_geometry`` stub for
            # THIS layer before any raise site below can fire. Without it, a
            # raise between here and the full assignment further down (e.g.
            # inside ``_check_mesh_alignment``, or "region has no sliced
            # surfaces") would leave ``self._last_geometry`` holding the LAST
            # layer that reached the full assignment -- a different layer's
            # geometry silently attributed to this layer's failure report. Every value written here is either known
            # immediately (layer_index, print_z) or explicitly
            # ``None``/``False`` rather than left over from a previous layer.
            # Enriched in place (never replaced wholesale) as more becomes
            # known further down, so a raise at ANY point after this line
            # reports at least this stub, always carrying its OWN layer_index
            # -- the field that makes staleness detectable even where full
            # enrichment could not be reached.
            self._last_geometry = {
                "layer_index": index,
                "print_z": layer.print_z,
                "complete": False,
                "host_contour_pts_mm": None,
                "our_section_pts_mm": None,
            }

            # The root region stays solid: it is the coupon's base, and the
            # first layers are where a slot would most easily lift. Recorded
            # rather than skipped silently, so layers_seen stays truthful. This
            # is a BED-adhesion guard (layer 0 upward), not an
            # anatomical-root guard -- on a tip-face-down print it protects the
            # TIP end and the true root gets no such protection.
            if index < spec.root_solid_layers:
                diagnostics.record(
                    LayerRecord(  # noqa: F821 - supplied by the bundle
                        layer_index=index,
                        print_z=layer.print_z,
                        mutated=False,
                        extra={"reason": "root_solid_layers"},
                    )
                )
                if self._sidecar_enabled:
                    # Contiguous layer_index sequence in the sidecar even
                    # for a layer the guard never reaches -- no mesh
                    # section, no host geometry, ``guard_ran: False``,
                    # explicit rather than a silent gap an offline replay
                    # could mistake for "layer never sliced".
                    self._host_sections_sidecar.append({
                        "layer_index": index,
                        "print_z": round(float(layer.print_z), 5),
                        "slice_z": None,
                        "guard_ran": False,
                        "host_regions": [],
                        "mesh_section_loops": [],
                        "deltas_mm": None,
                        "effective_tolerance_mm": None,
                        "near_top_extremum": None,
                        "tip_alignment_skip_enabled": bool(
                            spec.tip_alignment_skip_enabled),
                        "decision": None,
                        "reason": "root_solid_layers",
                    })
                continue

            mutated_any = False
            regions = list(layer.regions())
            if not regions:
                raise TopologyError(  # noqa: F821 - supplied by the bundle
                    index,
                    layer.print_z,
                    "layer has no regions, so the requested web cannot be placed.",
                )
            # A genuine (aerofoil-shaped) alignment problem detected once for
            # the whole run in ``_build_mesh_sectioner`` fails the slice HERE,
            # at the first body layer reached -- where a layer index and Z
            # exist to name, mirroring how ``mesh_note`` defers its own
            # fail-closed raise to ``_channels_for_layer`` immediately below.
            if self._alignment_fatal_error is not None:
                raise TopologyError(  # noqa: F821 - supplied by the bundle
                    index, layer.print_z, self._alignment_fatal_error
                )
            # Cross-section the model ONCE per layer, before the region loop,
            # so the per-layer result is not recomputed (and mislabelled) per
            # region.
            layer_channels = self._channels_for_layer(
                sectioner, mesh_note, index, layer)
            if self._sidecar_enabled:
                self._current_sidecar_entry = self._start_sidecar_entry(
                    index, layer, spec)
                self._host_sections_sidecar.append(self._current_sidecar_entry)
            else:
                self._current_sidecar_entry = None
            alignment_checked = False
            round_trip_checked = False

            for region in regions:
                slices = region.slices
                originals = [surface.expolygon for surface in slices.surfaces]
                types = [surface.surface_type for surface in slices.surfaces]
                # Enrich this layer's stub (written at the top of the outer
                # loop, above) as soon as the host's own contour for THIS
                # layer is known -- still this layer's own dict, carrying its
                # own layer_index.
                if originals:
                    self._last_geometry["host_contour_pts_mm"] = (
                        ops.contour_points_mm(originals[0])
                    )
                if self._current_sidecar_entry is not None:
                    # Raw host geometry, in the host's OWN (pre-de-rotation)
                    # frame -- "the host region polygons" as the host
                    # actually produced them, for every surface in this
                    # region, not just ``originals[0]``. Copied to plain
                    # floats immediately (lifetime rule: ``originals``/
                    # ``types`` are non-owning references valid only during
                    # this call).
                    self._current_sidecar_entry["host_regions"].append({
                        "surfaces": [
                            {
                                "surface_type": str(types[i]),
                                "outer_pts_mm": [
                                    [round(px, 5), round(py, 5)]
                                    for px, py in ops.contour_points_mm(
                                        originals[i])
                                ],
                                "holes_pts_mm": [
                                    [
                                        [round(px, 5), round(py, 5)]
                                        for px, py in hole
                                    ]
                                    for hole in ops.hole_points_mm(
                                        originals[i])
                                ],
                            }
                            for i in range(len(originals))
                        ]
                    })
                if not originals:
                    # A host region can legitimately be empty right at the
                    # mesh's own top Z -- a knife-edge sliver tip section
                    # (see generator.py's ``PRINTABLE_FLOOR_WALL_MULTIPLIER``
                    # comment for the measured real case, 0.0064x1.8076 mm,
                    # area 0.00577 mm^2) can fall below the host's OWN
                    # ``slice_closing_radius`` while the plugin's own
                    # (higher-precision) mesh sectioner still resolves a
                    # real, if tiny, section there. Degrading (shutdown,
                    # unmutated) instead of raising is safe ONLY when ALL
                    # THREE hold -- every other empty-region case still
                    # raises:
                    #   1. this region really is empty (this block's own
                    #      condition, above);
                    #   2. this layer's print_z is within one layer_height
                    #      (``self._alignment_extremum_band_mm``, the SAME
                    #      near-top-extremum band ``_check_mesh_alignment``
                    #      uses -- not a separate tunable) of the mesh's
                    #      own top Z;
                    #   3. the PLUGIN's own mesh section at this Z
                    #      (``self._last_loops``, set by
                    #      ``_channels_for_layer`` above, before this
                    #      region loop) INDEPENDENTLY agrees there is
                    #      nothing to build: no outer loop at all, or the
                    #      largest outer loop's own smaller in-plane extent
                    #      is below the same printable floor (2x the wall
                    #      line width) used in generator.py.
                    # The MESH-VS-HOST AGREEMENT is the evidence that makes
                    # this safe: this is not "the host said nothing, so
                    # believe it" (a silent fallback) -- it is two
                    # independent geometry sources agreeing there is
                    # nothing here, at a Z where "nothing" is geometrically
                    # plausible (the tip). A wrong/absent host section
                    # ANYWHERE ELSE (mid-span, an ordinary root/body layer)
                    # still raises, because condition 2 alone excludes it.
                    host_tip_region_empty = False
                    band_mm = self._alignment_extremum_band_mm
                    mesh_bounds = self._mesh_alignment.get(
                        "mesh_bounds_after_transform")
                    if band_mm is not None and mesh_bounds:
                        z_bounds = mesh_bounds.get("z")
                        if z_bounds and len(z_bounds) == 2:
                            # Print top, not the anatomical tip.
                            z_hi = self._print_top_z(z_bounds)
                            if abs(float(layer.print_z) - z_hi) <= band_mm:
                                mesh_loops = getattr(self, "_last_loops", None) or []
                                mesh_outer = [
                                    lp for lp in mesh_loops if not lp.is_interior
                                ]
                                if not mesh_outer:
                                    host_tip_region_empty = True
                                else:
                                    best_mesh = max(
                                        mesh_outer,
                                        key=lambda lp: abs(lp.signed_area),
                                    )
                                    mnx, mny, mxx, mxy = best_mesh.bbox
                                    mesh_min_extent = min(mxx - mnx, mxy - mny)
                                    printable_floor_mm = 2.0 * wall_width
                                    if mesh_min_extent < printable_floor_mm:
                                        host_tip_region_empty = True
                    if host_tip_region_empty:
                        diagnostics.record(
                            LayerRecord(  # noqa: F821 - supplied by the bundle
                                layer_index=index,
                                print_z=layer.print_z,
                                mutated=False,
                                shutdown=True,
                                degradations=[{
                                    "kind": "host_tip_region_empty",
                                    "x_mm": 0.0,
                                    "reason": (
                                        "DEGRADED -- no structure on this "
                                        "layer: the host's own sliced "
                                        "region is empty at print_z={:.4f} "
                                        "mm, within one layer_height "
                                        "({:.4f} mm) of the mesh's own top "
                                        "Z ({:.4f} mm), and the plugin's "
                                        "own mesh section there "
                                        "independently agrees there is "
                                        "nothing to build (no outer loop, "
                                        "or below the printable floor). "
                                        "Treated as a tip too thin to "
                                        "carry structure, not a topology "
                                        "failure.".format(
                                            float(layer.print_z), band_mm, z_hi
                                        )
                                    ),
                                }],
                                extra={"reason": "host_tip_region_empty"},
                            )
                        )
                        continue
                    # A body layer with an empty region cannot carry the
                    # requested web. Failing closed rather than skipping keeps
                    # the module docstring's claim true: no path leaves a layer
                    # unmutated and carries on.
                    raise TopologyError(  # noqa: F821 - supplied by the bundle
                        index,
                        layer.print_z,
                        "region has no sliced surfaces, so the requested web "
                        "cannot be placed.",
                    )

                # Rotate the INCOMING host slice polygons by -theta into the
                # canonical (chord-along-X) frame BEFORE frame construction,
                # generation, validation, or the boolean difference --
                # everything below runs entirely in the canonical frame; the
                # FINAL replacement polygons are rotated back by +theta after
                # the boolean difference (below), so what gets validated is
                # what ships (never "cut canonically, diff in world frame"). A
                # theta of exactly 0.0 (the epsilon-gated case, or a mesh with
                # no genuine trailing edge -- see ``_build_mesh_sectioner``)
                # makes this whole block a no-op by construction: the rotation
                # is skipped entirely so an already-aligned wing's geometry is
                # bit-identical, not merely close.
                theta_deg = self._alignment_theta_deg
                if theta_deg:
                    if not round_trip_checked:
                        _assert_rotation_round_trip(  # noqa: F821 - bundle
                            ops, originals[0], theta_deg)
                        round_trip_checked = True
                    originals = [
                        _rotate_expolygon(ops, poly, -theta_deg)  # noqa: F821
                        for poly in originals
                    ]

                counts_before = counting_ops.snapshot() if counting_ops is not None else None

                # A section too thin to carry the requested web raises a bare
                # ValueError from deep in the geometry core, which knows the
                # chord station but not which layer it is on. Translate it here,
                # where the layer index and Z are in hand: a geometric
                # impossibility must fail closed naming the layer, not surface
                # as an untraceable "aircraft structures failed".
                try:
                    t_gen0 = time.perf_counter()
                    # A layer carrying MORE THAN ONE disjoint island is
                    # DEGRADED as a whole: the layer is left unmutated and
                    # reported, never structured on one island while the others
                    # pass through. Reporting alone would not be enough: the
                    # validator receives ALL the originals and invariant 1
                    # refuses with "produced N material islands, expected 1",
                    # so the multi-island layer would fail the slice. (A harness
                    # that picks the largest island per layer would hide this,
                    # since the host hands the plugin every island.) An example
                    # is the transition between an aerofoil and a joiner post:
                    # two islands, the post and a thin strip, the strip
                    # carrying no structure.
                    #
                    # Leaving the layer unmutated is safe by construction: it
                    # is the host's own geometry, untouched, which is what the
                    # zero-intent path ships for a layer that cannot carry
                    # structure. Structuring each island independently is not
                    # attempted.
                    if len(originals) > 1:
                        _areas = sorted(
                            (ops.area(o) for o in originals), reverse=True)
                        diagnostics.record(
                            LayerRecord(  # noqa: F821 - supplied by the bundle
                                layer_index=index,
                                print_z=layer.print_z,
                                mutated=False,
                                shutdown=True,
                                degradations=[{
                                    "kind": "multi_island_layer",
                                    "x_mm": 0.0,
                                    "reason": (
                                        "DEGRADED -- this layer has {} "
                                        "disjoint material islands (areas "
                                        "{}). The plugin structures one "
                                        "region per layer, so the layer is "
                                        "left unmutated and prints as plain "
                                        "skin rather than failing the "
                                        "slice. Common at a transition "
                                        "between an aerofoil and a "
                                        "non-aerofoil feature.".format(
                                            len(originals),
                                            ", ".join("{:.2f}".format(a)
                                                      for a in _areas),
                                        )
                                    ),
                                }],
                                extra={"reason": "multi_island_layer"},
                            )
                        )
                        continue

                    frame = LayerFrame.from_polygon(ops, originals[0])  # noqa: F821
                    # The non-empty counterpart to the
                    # ``host_tip_region_empty`` path above. The host can retain
                    # a last-layer sliver that is non-empty but narrower than
                    # two wall lines; the mesh section can independently agree.
                    # Such a section is all skin and cannot carry a web, so
                    # leave it unmutated before the alignment guard compares
                    # two numerically unstable tip caps. Measured on the elevon
                    # at layer 1250: host and mesh widths 0.1850/0.1823 mm
                    # versus a 0.8400 mm floor.
                    host_min_extent = min(frame.width, frame.depth)
                    printable_floor_mm = 2.0 * wall_width
                    band_mm = self._alignment_extremum_band_mm
                    mesh_bounds = self._mesh_alignment.get(
                        "mesh_bounds_after_transform")
                    host_tip_below_floor = False
                    mesh_min_extent = None
                    z_hi = None
                    if band_mm is not None and mesh_bounds:
                        z_bounds = mesh_bounds.get("z")
                        if z_bounds and len(z_bounds) == 2:
                            # Print top, not the anatomical tip.
                            z_hi = self._print_top_z(z_bounds)
                            if abs(float(layer.print_z) - z_hi) <= band_mm:
                                mesh_outer = [
                                    lp for lp in (
                                        getattr(self, "_last_loops", None) or []
                                    )
                                    if not lp.is_interior
                                ]
                                if mesh_outer:
                                    best_mesh = max(
                                        mesh_outer,
                                        key=lambda lp: abs(lp.signed_area),
                                    )
                                    mnx, mny, mxx, mxy = best_mesh.bbox
                                    mesh_min_extent = min(mxx - mnx, mxy - mny)
                                    host_tip_below_floor = (
                                        host_min_extent < printable_floor_mm
                                        and mesh_min_extent < printable_floor_mm
                                    )
                    if host_tip_below_floor:
                        diagnostics.record(
                            LayerRecord(  # noqa: F821 - supplied by the bundle
                                layer_index=index,
                                print_z=layer.print_z,
                                mutated=False,
                                shutdown=True,
                                degradations=[{
                                    "kind": "host_tip_below_printable_floor",
                                    "x_mm": 0.0,
                                    "reason": (
                                        "DEGRADED -- no structure on this "
                                        "layer: the host and mesh tip "
                                        "sections have smaller extents "
                                        "{:.4f}/{:.4f} mm, both below the "
                                        "{:.4f} mm printable floor (two "
                                        "wall lines), within {:.4f} mm of "
                                        "the mesh top Z {:.4f} mm. The "
                                        "non-empty sliver is all skin and "
                                        "is left unmutated.".format(
                                            host_min_extent,
                                            mesh_min_extent,
                                            printable_floor_mm,
                                            band_mm,
                                            z_hi,
                                        )
                                    ),
                                }],
                                extra={
                                    "reason": "host_tip_below_printable_floor"
                                },
                            )
                        )
                        continue
                    # alignment CHECK, not an assumption. The mesh->print
                    # transform is derived from the volume and instance
                    # placement; this compares the resulting cross-section
                    # against the host's OWN slice of the same layer. A wrong
                    # transform would otherwise put the spar socket somewhere
                    # plausible but wrong, which is the worst possible failure
                    # mode. ENFORCED at spec.mesh_alignment_tolerance_mm
                    # (default MESH_ALIGNMENT_TOLERANCE_MM); see the comment
                    # above that constant for how the limit was measured.
                    if not alignment_checked:
                        skip_record = self._check_mesh_alignment(
                            index, layer, frame, spec, sectioner)
                        alignment_checked = True
                        if skip_record is not None:
                            # Opted-in tip-alignment skip: leave THIS region,
                            # and every other region on this layer, exactly as
                            # it started. No slices.set(), no make_slices() --
                            # mutated_any stays False, so the end-of-layer
                            # make_slices() guard is also skipped for this
                            # layer, mirroring the tip_shutdown
                            # "record-and-continue" shape of the
                            # omission-shutdown branch below.
                            # _last_geometry/_root_layers/_mesh_alignment's
                            # per-run "latest" attribution fields are
                            # deliberately NOT touched here -- a skipped layer
                            # must never become a later failure report's
                            # attributed geometry.
                            diagnostics.record(
                                LayerRecord(  # noqa: F821 - supplied by the bundle
                                    layer_index=index,
                                    print_z=layer.print_z,
                                    mutated=False,
                                    shutdown=True,
                                    extra={
                                        "reason": "tip_alignment_skip",
                                        "band_mm": skip_record["band_mm"],
                                        "deltas_mm": skip_record["deltas_mm"],
                                    },
                                )
                            )
                            break
                    layer_ctx = LayerContext(  # noqa: F821 - supplied by the bundle
                        layer_index=index, print_z=layer.print_z,
                        detected_channels=layer_channels,
                    )
                    z_rule = self._z_rule
                    if z_rule is not None:
                        # A whole-layer shutdown only for a
                        # genuinely re-entrant section (the bare section,
                        # eroded at the wall-safety offset, has more than one
                        # island). A notch is handled per station.
                        _hit = z_rule.reentrant_check(ops, originals[0])
                        if _hit is not None:
                            z_rule._event(
                                "genuine_reentrant", layer=index,
                                z=round(float(layer.print_z), 4),
                                islands=_hit["islands"],
                                areas_mm2=_hit["areas_mm2"])
                            raise ReEntrantSectionError(  # noqa: F821 - bundle
                                _hit["message"])

                    def _gen(caps, _frame=frame, _ctx=layer_ctx):
                        return generator.generate(
                            _frame,
                            spec,
                            wall_width,
                            _ctx if caps is None else replace(  # noqa: F821
                                _ctx, depth_caps=caps),
                            safety_offset_mm=wall_width,
                        )

                    generation = (
                        z_rule.generate_layer(index, layer.print_z, _gen)
                        if z_rule is not None else _gen(None)
                    )
                    intents = generation.intents
                    omitted = generation.omitted
                    # Never silent: every dropped feature reaches the
                    # LayerRecord below AND the run-summary sentence the user
                    # actually reads. That reporting is what justifies
                    # degrading instead of failing closed ("explicitly marked
                    # degraded").
                    #
                    # Direct attribute access, NOT getattr-with-default:
                    # ``GenerationResult.degradations`` always exists, and
                    # a silent default here would make a rename report every
                    # degraded run as clean. Same defect shape as the
                    # ``frame.points`` regression documented in
                    # ``_check_mesh_alignment``.
                    layer_degradations = [
                        {"kind": d.kind, "x_mm": round(d.x_mm, 4),
                         "reason": d.reason}
                        for d in generation.degradations
                    ]

                    intents, layer_degradations = (
                        self._apply_spar_socket_latch(
                            intents, layer_degradations, index)
                    )
                    if z_rule is not None:
                        intents, omitted, _suppressed = z_rule.post_latch(
                            index, intents, omitted)
                        if _suppressed:
                            layer_degradations = list(layer_degradations) + [{
                                "kind": "spar_socket_latched",
                                "x_mm": 0.0,
                                "reason": (
                                    "DEGRADED -- spar socket suppressed: it "
                                    "had ENDED and sockets cannot ramp, so "
                                    "it stays ended (z_continuity)."),
                            }]
                    t_gen1 = time.perf_counter()
                    built = [builder.build(i, spec.overshoot_mm) for i in intents]
                    slots = [ops.polygon_from_points(b.points) for b in built]
                    t_slot1 = time.perf_counter()

                    # One label per slot, in the same order as `slots`/
                    # `intents` (built together above), so a collision/
                    # margin/boundary failure can name which family produced
                    # the offending web — never changes which invariant fires,
                    # only the message (see aircraft_core/validate.py's
                    # `slot_labels` docstring). Built HERE, before the pre-diff
                    # drop below, so `slot_labels`/`family_indices` can be
                    # filtered in lockstep with `intents`/`built`/`slots` and
                    # then reused UNCHANGED for the post-diff `validate()` call
                    # further down -- one construction site, so a slot's label
                    # can never read differently pre-diff and post-diff.
                    slot_labels = [
                        "family {} ({!r})".format(i.family_index, i.family_name)
                        for i in intents
                    ]
                    # one family-membership SET per slot, same order as
                    # slot_labels/slots/intents — this is what lets the
                    # validator's collision check tell an intended cross-family
                    # junction from a same-family placement mistake (see
                    # aircraft_core/validate.py's ``validate()`` docstring,
                    # "family_indices"). This passes each intent's FULL
                    # membership (``WebIntent.family_membership`` —
                    # {family_index} for an ordinary web, or the whole
                    # merged_family_indices for a merge), not the bare
                    # representative ``family_index`` alone — using the
                    # representative here for what is ultimately a SAFETY
                    # decision (same-family vs. cross-family) would let an
                    # arbitrary reporting label decide whether a genuine
                    # same-family collision gets classified as an intended
                    # cross-family junction and let through. See
                    # ``WebIntent.family_membership``'s own docstring and
                    # ``aircraft_core/validate.py``'s ``_family_membership``
                    # for the receiving side.
                    family_indices = [i.family_membership() for i in intents]

                    # Drop every slot that fails its OWN per-slot feasibility
                    # checks BEFORE the boolean diff, rather than refusing the
                    # whole layer over one bad slot. ``infeasible_slots``
                    # reuses the identical per-slot predicates ``validate()``
                    # runs post-diff (one copy, never duplicated) but
                    # answers the narrower PRE-diff question "is this slot
                    # individually placeable". It does NOT replace
                    # ``validate()``'s own post-diff gate below, which keeps
                    # measuring the ACTUALLY REMOVED geometry against the
                    # SURVIVING slots and must still fail closed if a slot that
                    # passed here disagrees post-diff -- that disagreement is a
                    # suspected defect (Class C), never silently reconciled
                    # away here.
                    violations = validator.infeasible_slots(
                        ops,
                        originals[0],
                        slots,
                        spec,
                        index,
                        layer.print_z,
                        slot_labels=slot_labels,
                        family_indices=family_indices,
                    )
                    if violations:
                        drop = {slot_index - 1 for slot_index, _exc in violations}
                        for slot_index, exc in violations:
                            dropped_intent = intents[slot_index - 1]
                            # A dropped SPAR SOCKET must engage the latch, not
                            # be recorded as a generic slot. Otherwise a socket
                            # whose keyhole fails a per-slot predicate here (an
                            # epsilon disagreement between the generator's own
                            # fit check and the validator's rotation-corrected
                            # margin is enough) would be dropped on this layer
                            # and re-cut further outboard -- the blind interior
                            # pocket the rod cannot enter, which is exactly
                            # what the latch exists to prevent. The latch is keyed on
                            # the ``spar_socket`` kind, so the kind must match
                            # or the guarantee has a hole.
                            #
                            # This also requires ``spar_confirmed`` -- a
                            # socket that came from
                            # ``spar_channel_policy="nearest_spar_station"``
                            # disambiguating among 2+ candidates
                            # (``spar_confirmed=False``) must not arm the latch
                            # here either, for the identical reason
                            # ``_apply_spar_socket_latch`` itself excludes it
                            # (see that method's own docstring).
                            is_socket = (
                                dropped_intent.family_name == "spar_socket"
                                and getattr(
                                    dropped_intent, "spar_confirmed", False)
                            )
                            layer_degradations.append({
                                "kind": (
                                    "spar_socket" if is_socket
                                    else "slot_infeasible"
                                ),
                                "x_mm": round(dropped_intent.start[0], 4),
                                "reason": (
                                    "DEGRADED -- spar socket dropped: its "
                                    "keyhole slot failed a per-slot check "
                                    "before the boolean. " + exc.cause
                                    if is_socket
                                    else "DEGRADED -- " + exc.cause
                                ),
                            })
                        keep = [
                            i for i in range(len(intents)) if i not in drop
                        ]
                        intents = [intents[i] for i in keep]
                        built = [built[i] for i in keep]
                        slots = [slots[i] for i in keep]
                        slot_labels = [slot_labels[i] for i in keep]
                        family_indices = [family_indices[i] for i in keep]

                    # Exact inputs to the geometry, captured for diagnosis:
                    # Clipper can cut a slot wider and further right than the
                    # core builds offline, and the cause cannot be inferred
                    # from the removed region alone.
                    # ``layer_index``/``print_z``/``complete`` here match the
                    # stub written at the top of the layer loop -- this is
                    # the same layer's dict, fully enriched, never a
                    # replacement that could silently drift to a different
                    # layer than the stub described.
                    self._last_geometry = {
                        "layer_index": index,
                        "print_z": layer.print_z,
                        "complete": True,
                        "host_contour_pts_mm": (
                            self._last_geometry.get("host_contour_pts_mm")
                            if self._last_geometry else None
                        ),
                        "our_section_pts_mm": (
                            self._last_geometry.get("our_section_pts_mm")
                            if self._last_geometry else None
                        ),
                        "wall_width_mm": wall_width,
                        "resolved_gap_mm": spec.resolve_slot_gap(wall_width),
                        "frame_bounds": [frame.min_x, frame.min_y, frame.max_x, frame.max_y],
                        "web_x": frame.web_x(spec.web_position),
                        "intents": [
                            {
                                "start": list(i.start),
                                "end": list(i.end),
                                "width_mm": i.width_mm,
                                "family_index": i.family_index,
                                "family_name": i.family_name,
                                # non-empty exactly for a merged node — see
                                # aircraft_core/generator.py's WebIntent
                                # docstring.
                                "merged_family_indices": list(
                                    i.merged_family_indices
                                ),
                            }
                            for i in intents
                        ],
                        # every candidate station omitted during generation,
                        # and why — never dropped silently (see
                        # aircraft_core/generator.py's "Omission policy").
                        "omitted": [
                            {
                                "x_mm": o.x_mm,
                                "reason": o.reason,
                                "family_index": o.family_index,
                                "family_name": o.family_name,
                            }
                            for o in omitted
                        ],
                        "slot_points_mm": [[list(pt) for pt in b.points] for b in built],
                        "slot_bounds_mm": [
                            [
                                min(pt[0] for pt in b.points),
                                min(pt[1] for pt in b.points),
                                max(pt[0] for pt in b.points),
                                max(pt[1] for pt in b.points),
                            ]
                            for b in built
                        ],
                        "slot_bounds_via_ops": [list(ops.bounds(sl)) for sl in slots],
                        # Clipper's own view of the cut, both ways round. If
                        # A intersect S is 0.21 mm wide but A minus S loses
                        # 0.525 mm, the difference operation is the culprit
                        # rather than the slot we handed it.
                        "intersection_bounds": self._safe_intersection_bounds(
                            ops, originals[0], slots
                        ),
                    }
                    t_diag1 = time.perf_counter()
                    if not intents:
                        if self._z_rule is not None:
                            self._z_rule.note_skipped(index, "no_structure")
                        # Cause classification. It does not gate the OUTCOME
                        # (degrades regardless), only the reason string --
                        # "chord extent" is a positional miss, not evidence the
                        # section itself collapsed; see
                        # aircraft_core/generator.py's OmittedWeb.reason
                        # docstring.
                        thinness_prefixes = ("skin margin:", "thin section:")
                        all_thin = bool(omitted) and all(
                            o.reason.startswith(thinness_prefixes)
                            for o in omitted
                        )
                        # A layer with no surviving web is left UNMUTATED and
                        # recorded, whatever the cause, instead of refusing the
                        # slice (Class B degradation, default on; this is why
                        # ``tip_shutdown_enabled`` is inert).
                        #
                        # The cause does not gate the OUTCOME: a layer that
                        # cannot carry any pattern is precisely a feature that
                        # cannot be placed. The cause still gates the REASON
                        # STRING, because "the section ran out of material" and
                        # "no candidate was ever generated" are different things
                        # for a reader.
                        #
                        # This is also the site reached at a thinning tip when
                        # the lattice is omitted for thinness and the spar
                        # socket is dropped at the same station: ``intents`` is
                        # empty, and the reason must not blame the WEBS for what
                        # was really a socket failure.
                        if not omitted and layer_degradations:
                            # Distinguish "never generated" from "generated
                            # then dropped". Saying "no candidate was
                            # generated at all" when one was generated and
                            # dropped pre-diff is literally false, and the
                            # reader chasing it would look in the wrong
                            # place entirely.
                            why = (
                                "every candidate web was generated but then "
                                "dropped before the boolean as individually "
                                "unplaceable (see this layer's other "
                                "degradation records for each one)"
                            )
                        elif not omitted:
                            why = (
                                "no candidate web stations were generated "
                                "at all for this section"
                            )
                        elif all_thin:
                            # Carry the per-station detail, not just the
                            # count: every station and its measured margin is
                            # exactly what a reader needs to decide whether
                            # the tip is behaving as designed.
                            why = (
                                "the section has run out of material to "
                                "carry any web ({} candidate(s), every one "
                                "omitted for thinness or skin margin): {}"
                            ).format(
                                len(omitted),
                                "; ".join(
                                    "x={:.4f} mm ({})".format(o.x_mm, o.reason)
                                    for o in omitted
                                ),
                            )
                        else:
                            why = (
                                "every one of {} candidate web(s) was "
                                "omitted: {}"
                            ).format(
                                len(omitted),
                                "; ".join(
                                    "x={:.4f} mm ({})".format(o.x_mm, o.reason)
                                    for o in omitted
                                ),
                            )
                        layer_degradations.append({
                            "kind": "pattern_all_webs_omitted",
                            "x_mm": 0.0,
                            "reason": (
                                "DEGRADED -- no structure on this layer: "
                                + why
                                + ". The layer prints as plain skin. This "
                                "is a feature that could not be placed, "
                                "not a geometric anomaly."
                            ),
                        })
                        diagnostics.record(
                            LayerRecord(  # noqa: F821 - supplied by the bundle
                                layer_index=index,
                                print_z=layer.print_z,
                                mutated=False,
                                shutdown=True,
                                degradations=layer_degradations,
                                extra={
                                    "reason": (
                                        "tip_shutdown" if all_thin
                                        else "all_webs_omitted"
                                    ),
                                    "omitted": [
                                        {
                                            "x_mm": o.x_mm,
                                            "reason": o.reason,
                                            "family_index": o.family_index,
                                            "family_name": o.family_name,
                                        }
                                        for o in omitted
                                    ],
                                },
                            )
                        )
                        # Leaves this region's surfaces exactly as they
                        # started (never reaches slices.set()/
                        # make_slices() below) and moves on to the next
                        # region/layer -- mutated_any stays False, so the
                        # end-of-layer make_slices() guard below is also
                        # skipped for this layer.
                        continue
                except ReEntrantSectionError as exc:  # noqa: F821 - bundle
                    # The whole SECTION is re-entrant -- the chord probe
                    # crosses material more than twice, so "local thickness" is
                    # not a well-defined pair of surfaces at ANY station here.
                    # There is no place to put wing structure, so the layer is
                    # left unmutated and recorded rather than failing the
                    # slice.
                    #
                    # Safe for the same reason the multi-island case is:
                    # unmutated means the host's own geometry, untouched. This
                    # is NOT dropping one uncertain feature while keeping
                    # others, which would be a silent fallback. An example is
                    # a section 0.74 mm below the tip where it pinches into two
                    # lobes.
                    #
                    # Caught BY TYPE, never as a broad ValueError: a catch-all
                    # here would degrade genuine fail-closed conditions.
                    if getattr(self, "_z_rule", None) is not None:
                        self._z_rule.note_skipped(index, "re_entrant_section")
                    diagnostics.record(
                        LayerRecord(  # noqa: F821 - supplied by the bundle
                            layer_index=index,
                            print_z=layer.print_z,
                            mutated=False,
                            shutdown=True,
                            degradations=[{
                                "kind": "re_entrant_section",
                                "x_mm": 0.0,
                                "reason": (
                                    "DEGRADED -- no structure on this "
                                    "layer: " + str(exc) + " The layer "
                                    "prints as plain skin rather than "
                                    "failing the slice."
                                ),
                            }],
                            extra={"reason": "re_entrant_section"},
                        )
                    )
                    continue
                except (ValueError, ArithmeticError) as exc:
                    raise TopologyError(  # noqa: F821 - supplied by the bundle
                        index, layer.print_z, str(exc)
                    )

                generation_s = t_gen1 - t_gen0
                slot_build_s = t_slot1 - t_gen1
                diag_capture_s = t_diag1 - t_slot1

                replacement = []
                replacement_types = []
                _diag = getattr(self, "_last_geometry", None)
                t_bool0 = time.perf_counter()
                for polygon, surface_type in zip(originals, types):
                    for piece in ops.diff(polygon, slots):
                        replacement.append(piece)
                        replacement_types.append(surface_type)
                t_bool1 = time.perf_counter()
                slot_build_and_boolean_s = slot_build_s + (t_bool1 - t_bool0)

                t_diag2_0 = time.perf_counter()
                if _diag is not None:
                    # Diagnostics must never change behaviour: a backend may
                    # legitimately refuse bounds() on a cut section.
                    _diag["replacement_count"] = len(replacement)
                    try:
                        _diag["replacement_bounds"] = [
                            list(ops.bounds(piece)) for piece in replacement
                        ]
                    except Exception as exc:
                        _diag["replacement_bounds"] = "<unavailable: {}>".format(exc)
                    try:
                        # Areas settle it: if area(A) - area(A-S) exceeds
                        # area(A n S), material vanished that no cut accounts
                        # for. Bounds can mislead; conservation of area cannot.
                        area_a = ops.area(originals[0])
                        area_u = sum(ops.area(x) for x in replacement)
                        _diag["area_original"] = area_a
                        _diag["area_replacement"] = area_u
                        _diag["area_removed_by_difference"] = area_a - area_u
                        _diag["area_intersection"] = sum(
                            ops.area(piece)
                            for sl in slots
                            for piece in ops.intersection(originals[0], sl)
                        )
                        _diag["contours_replacement"] = [
                            ops.contour_count(x) for x in replacement
                        ]
                    except Exception as exc:
                        _diag["area_check"] = "<unavailable: {}>".format(exc)
                t_diag2_1 = time.perf_counter()
                diag_capture_s += t_diag2_1 - t_diag2_0

                # Vertex counts before/after mutation, to detect runaway
                # complexity. Guarded the same
                # way `replacement_bounds` above is: a backend may refuse to
                # reconstruct a cut, non-rectangular section's exact outline
                # (see `_count_vertices`'s docstring), so an unavailable
                # count is recorded as None rather than failing the slice.
                # Counted as diagnostics-capture time, same bucket as the
                # rest of this per-layer debug bookkeeping.
                vertex_before = None
                vertex_after = None
                if metrics is not None:
                    t_vertex0 = time.perf_counter()
                    vertex_before = self._count_vertices(originals)
                    vertex_after = self._count_vertices(replacement)
                    t_vertex1 = time.perf_counter()
                    diag_capture_s += t_vertex1 - t_vertex0

                # `slot_labels`/`family_indices` were built above, before the
                # pre-diff drop, and already reflect any dropped slot -- reused
                # here unchanged so a slot's label can never read differently
                # pre-diff and post-diff.
                t_val0 = time.perf_counter()
                # Lazy opposing-pair ablation. None on every layer that
                # validates as it stands (the retry path is never entered); a
                # rescue only when the layer failed with the island-split cause
                # and removing ONE pair station restored it. All logic lives in
                # aircraft_core/pair_ablation.py. Lazy pinch check: when the
                # wing's OWN bare cross-section,
                # with NO slots cut, already splits under the validator's
                # safety erosion, no web placement can help this layer. Both
                # are reached only after validation failed with the
                # island-split cause; healthy layers pay nothing. Any other
                # failure, or a bare contour that does not split (a slot-caused
                # split), re-raises unchanged.
                z_rescue = None
                if self._z_rule is not None:
                    # under z_continuity the validator runs in the loop with
                    # single-web ablation (the culprit ENDs); pair ablation and
                    # pinch are not used (a genuinely re-entrant section was
                    # already shut down before generation).
                    pair_rescue, pinch = None, None
                    z_rescue = self._z_rule.validate_with_ablation(
                        ops, validator, originals, replacement, intents,
                        slots, spec, index, layer.print_z, slot_labels,
                        family_indices)
                else:
                    pair_rescue, pinch = validate_with_pinch_shutdown(  # noqa: F821 - supplied by the bundle
                        ops,
                        validator,
                        originals,
                        replacement,
                        intents,
                        slots,
                        spec,
                        index,
                        layer.print_z,
                        slot_labels,
                        family_indices,
                        wall_width,
                    )
                if pinch is not None:
                    # The existing shutdown mechanism: the layer is left
                    # exactly as the host sliced it, recorded. Not a
                    # transformed layer, and not silent.
                    pinch_degradations, pinch_extra = layer_record_fields(  # noqa: F821
                        pinch)
                    diagnostics.record(
                        LayerRecord(  # noqa: F821 - supplied by the bundle
                            layer_index=index,
                            print_z=layer.print_z,
                            mutated=False,
                            shutdown=True,
                            degradations=pinch_degradations,
                            extra=pinch_extra,
                        )
                    )
                    continue
                if pair_rescue is not None:
                    keep = list(pair_rescue.kept_indices)
                    # the ablated station's web identity (the removed members
                    # are about to leave ``intents``).
                    _removed_ids = [
                        intents[i].web_id for i in pair_rescue.removed_indices
                        if intents[i].web_id is not None
                    ]
                    if _removed_ids and pair_rescue.omitted.web_id is None:
                        pair_rescue = replace(  # noqa: F821 - supplied by the bundle
                            pair_rescue,
                            omitted=replace(  # noqa: F821
                                pair_rescue.omitted,
                                k=_removed_ids[0][1],
                                web_id=(_removed_ids[0][0], _removed_ids[0][1], "P"),
                            ),
                        )
                    intents = [intents[i] for i in keep]
                    slots = [slots[i] for i in keep]
                    slot_labels = [slot_labels[i] for i in keep]
                    family_indices = [family_indices[i] for i in keep]
                    omitted = list(omitted) + [pair_rescue.omitted]
                    layer_degradations.append({
                        "kind": "pair_ablation",
                        "x_mm": round(pair_rescue.station_x_mm, 4),
                        "reason": "DEGRADED -- " + pair_rescue.omitted.reason,
                    })
                    replacement = []
                    replacement_types = []
                    for polygon, surface_type in zip(originals, types):
                        for piece in ops.diff(polygon, slots):
                            replacement.append(piece)
                            replacement_types.append(surface_type)
                if z_rescue is not None:
                    keep = list(z_rescue.kept_indices)
                    intents = [intents[i] for i in keep]
                    slots = [slots[i] for i in keep]
                    slot_labels = [slot_labels[i] for i in keep]
                    family_indices = [family_indices[i] for i in keep]
                    omitted = list(omitted) + list(z_rescue.omitted)
                    layer_degradations.append({
                        "kind": "z_ablation",
                        "x_mm": round(z_rescue.omitted[0].x_mm, 4),
                        "reason": "DEGRADED -- " + z_rescue.omitted[0].reason,
                    })
                    replacement = []
                    replacement_types = []
                    for polygon, surface_type in zip(originals, types):
                        for piece in ops.diff(polygon, slots):
                            replacement.append(piece)
                            replacement_types.append(surface_type)
                if self._z_rule is not None:
                    # The FINAL intents of this layer: this is what the next
                    # layer's ramp is measured from.
                    self._z_rule.commit(index, layer.print_z, intents, omitted)
                validation_s = time.perf_counter() - t_val0

                # Rotate the FINAL, validated replacement polygons back by
                # +theta into world frame -- AFTER validation, not before.
                # Validating the canonical geometry and shipping a
                # separately-rotated copy would validate one polygon and ship
                # another; rotating the validated pieces themselves keeps what
                # shipped identical in shape to what passed.
                if theta_deg:
                    replacement = [
                        _rotate_expolygon(ops, piece, theta_deg)  # noqa: F821
                        for piece in replacement
                    ]

                surfaces = [
                    orca.host.Surface(surface_type, piece)
                    for piece, surface_type in zip(replacement, replacement_types)
                ]
                # The UN-slotted skin length is read from originals[0] HERE,
                # before it is deleted below — see the
                # ``record_structure_layer`` call further down this loop, which
                # uses this captured value rather than originals[0] itself.
                skin_length_mm = self._contour_perimeter_mm(ops, originals[0])
                # No live Surface reference survives this call; everything the
                # loop needed was read above.
                del originals, types
                slices.set(surfaces)
                mutated_any = True

                # Per-family breakdown, alongside the aggregate counts above:
                # a two-tier part (e.g. a shallow skin family
                # plus a deep rib family) is only legible in the report if
                # each family's own slots/omissions are visible separately,
                # not merged into one number. Keyed by "index name" so two
                # families sharing a name (not forbidden) stay
                # distinguishable.
                #
                # A merged node is counted against EVERY constituent family,
                # not just the representative: bucketing a merge's single
                # "slots" count entirely under ``i.family_index`` (the
                # REPORTING-level representative — see
                # ``WebIntent.family_membership``'s own docstring) would make
                # the per-family totals depend on which family a merge happened
                # to report as its representative rather than on which families
                # were actually present. A merged intent's own ``family_name`` is also only
                # the representative's name, so every OTHER constituent's name
                # is looked up from ``spec`` directly.
                #
                # WHAT "slots" MEANS FOR A MERGED NODE, stated plainly
                # because this report is read directly: a merged node adds
                # 1 to EVERY constituent family's ``slots`` count, so ONE
                # physical slot is counted once per family it belongs to —
                # summing ``slots`` across all families does not equal the
                # total number of physical slots on the layer when any
                # merge occurred. ``merged_slots`` (a subset of ``slots``)
                # is how many of THIS family's counted slots were actually
                # merged nodes. The whole-layer physical slot count is
                # ``len(intents)``, and the per-layer ``node_merge_count``
                # below says how many of those were merges spanning more
                # than one family.
                family_names_by_index = {
                    idx: fam.name for idx, fam in enumerate(spec.effective_families())
                }
                # Role: looked up once per family, same shape as family_names_by_index
                # above, so every families_breakdown entry can be
                # attributed to the role its producing family DECLARED
                # (never inferred -- see aircraft_core.spec.WebFamily.role).
                # A scar (family_index=-1, no entry in effective_families())
                # falls back to the same default every untagged WebFamily
                # uses (ROLE_SKIN_LATTICE), rather than a bespoke label --
                # it is diagnostic labelling only, never a geometry
                # decision either way.
                family_roles_by_index = {
                    idx: fam.role for idx, fam in enumerate(spec.effective_families())
                }

                def _family_key(fam_idx, fallback_name):
                    return "{} {}".format(
                        fam_idx, family_names_by_index.get(fam_idx, fallback_name)
                    )

                def _family_role(fam_idx):
                    # Report UNKNOWN (None) rather than a confident wrong value
                    # for an index with no corresponding family.
                    #
                    # A feature-lattice intent carries family_index 0/1 from
                    # ``_generate_feature``'s own local enumeration, NOT an
                    # index into ``spec.effective_families()``. Defaulting to
                    # ROLE_SKIN_LATTICE for an unknown index would make a
                    # feature tooth's reported role read as whatever role wing
                    # family #0/#1 declares. This is a diagnostics field only;
                    # geometry and slicing are unaffected.
                    if fam_idx not in family_roles_by_index:  # noqa: F821
                        return None
                    return family_roles_by_index[fam_idx]  # noqa: F821

                # "opposing_entry_omitted" is a SUBSET of "omitted" (see
                # ``aircraft_core/generator.py``'s ``_omit_opposing_conflicts``
                # and its "opposing entry:..." reason prefix) -- broken out per
                # family here so
                # ``RunDiagnostics.opposing_entry_family_ratios`` can read
                # "what fraction of THIS family's own candidates were lost to
                # an opposing-entry collision specifically" without re-deriving
                # it from the reason string a second time anywhere else.
                families_breakdown = {}
                for i in intents:
                    member_families = i.merged_family_indices or (i.family_index,)
                    is_merged = bool(i.merged_family_indices)
                    for fam_idx in member_families:
                        key = _family_key(fam_idx, i.family_name)
                        entry = families_breakdown.setdefault(
                            key,
                            {
                                "slots": 0,
                                "merged_slots": 0,
                                "omitted": 0,
                                "opposing_entry_omitted": 0,
                                "role_yield_omitted": 0,
                                "role": _family_role(fam_idx),
                                # How many of this family's own built
                                # slots were a CONFIRMED spar
                                # (``WebIntent.spar_confirmed`` -- see that
                                # field's own docstring), so a run report can
                                # say whether the spar socket it cut was KNOWN
                                # or GUESSED, not just that one was cut.
                                # Meaningful only for the "-2 spar_socket"
                                # family (every other family's intents carry
                                # ``spar_confirmed=False`` by construction and
                                # are never consulted for it), reported
                                # generically rather than special-cased so this
                                # stays correct if a future family ever sets
                                # the flag too.
                                "spar_confirmed_slots": 0,
                            },
                        )
                        entry["slots"] += 1
                        if is_merged:
                            entry["merged_slots"] += 1
                        if getattr(i, "spar_confirmed", False):
                            entry["spar_confirmed_slots"] += 1
                for o in omitted:
                    key = _family_key(o.family_index, o.family_name)
                    entry = families_breakdown.setdefault(
                        key,
                        {
                            "slots": 0,
                            "merged_slots": 0,
                            "omitted": 0,
                            "opposing_entry_omitted": 0,
                            "role_yield_omitted": 0,
                            "role": _family_role(o.family_index),
                            "spar_confirmed_slots": 0,
                        },
                    )
                    entry["omitted"] += 1
                    if o.reason.startswith("opposing entry:"):
                        entry["opposing_entry_omitted"] += 1
                    if o.reason.startswith("role yield:"):
                        entry["role_yield_omitted"] += 1

                # total opposing-entry omissions on this layer, summed the same
                # way ``node_merge_count`` below sums a per-layer count for
                # ``RunDiagnostics`` to aggregate across the run
                # (``RunDiagnostics.opposing_entry_omission_count``).
                opposing_entry_omission_count = sum(
                    1 for o in omitted if o.reason.startswith("opposing entry:")
                )
                # Role-yield (``_yield_lattice_to_primary``):
                # total "role yield: ..." omissions on this layer, summed the
                # same way ``opposing_entry_omission_count`` above is, for
                # ``RunDiagnostics.role_yield_omission_count`` to aggregate
                # across the run.
                role_yield_omission_count = sum(
                    1 for o in omitted if o.reason.startswith("role yield:")
                )

                # A node merge is not an omission at all ("no family is
                # dropped") — it is visible on the surviving WebIntent itself,
                # via a non-empty ``merged_family_indices`` tuple (see
                # ``_merge_cross_family_nodes``'s docstring in
                # aircraft_core/generator.py). Counted
                # here so RunDiagnostics can aggregate it the same way
                # shutdown_count/first_shutdown_z are aggregated
                # (aircraft_core/diagnostics.py).
                node_merge_count = sum(
                    1 for i in intents if i.merged_family_indices
                )

                # Order-swap classification. The same-edge collision rule in
                # aircraft_core/validate.py ("Intended junction: footprints
                # actually overlap") accepts ANY disjoint-membership,
                # measurably-overlapping slot pair as a junction without
                # distinguishing intended same-family +/-
                # diamond apex (both families' march index k equal, e.g. both
                # landing on their SHARED k=0 anchor station) from a genuine
                # order swap between two DIFFERENT march stations of that same
                # declared pair (k differs, e.g. one family's k=0 anchor
                # overlapping the other's k=+1 neighbour). Computed here, from
                # ``intents`` directly, rather than inside ``validate()``
                # itself: ``validate()``'s own same-edge scan only compares
                # X-sorted ADJACENT slots (a real limitation -- see the
                # docstring on ``_classify_declared_pair_overlaps`` below), and
                # every input this classification needs (``spec``, ``frame``,
                # ``intents`` with their own
                # ``start``/``width_mm``/``family_membership()``) is already in
                # scope here, in exactly the same place ``node_merge_count``
                # above is computed the same way -- this is reporting only,
                # appended alongside it, and never changes
                # ``slots``/``slices.set(surfaces)`` (already called above) or
                # anything ``validate()`` itself decides.
                order_swap_events = self._classify_declared_pair_overlaps(
                    intents, spec, frame, index, layer.print_z
                )
                order_swap_count = len(order_swap_events)

                # Structural summary: recorded for
                # every region that actually reached this point (survived
                # generation and validation), using ``skin_length_mm`` captured
                # above, before ``originals``/``types`` were deleted. See
                # ``RunDiagnostics.record_structure_layer``'s docstring for
                # exactly what this proxy does and does not claim.
                effective_families = spec.effective_families()

                # Depth-truncation visibility: for every merged WebIntent on
                # this region, record how much of its deepest constituent's own
                # requested depth the merge actually kept — see
                # ``_node_merge_truncation_ratio``'s own docstring for the
                # definition and why a full-depth constituent's "requested
                # depth" needs its own local re-measurement. Visibility only:
                # this never changes what geometry is cut.
                effective_skin_margin_mm = corrected_skin_margin_mm(  # noqa: F821 - supplied by the bundle
                    spec.skin_margin_mm, frame.rotation_deg
                )
                for i in intents:
                    if not i.merged_family_indices:
                        continue
                    ratio = self._node_merge_truncation_ratio(
                        i, effective_families, frame, effective_skin_margin_mm
                    )
                    if ratio is not None:
                        diagnostics.record_node_merge_truncation(
                            layer_index=index, print_z=layer.print_z, ratio=ratio
                        )

                diagnostics.record_structure_layer(
                    layer_index=index,
                    print_z=layer.print_z,
                    skin_length_mm=skin_length_mm,
                    web_depths_mm=[self._intent_depth_mm(i) for i in intents],
                    entry_x_mm=[i.start[0] for i in intents],
                    # Name the entry surface so the bay is measured WITHIN one
                    # skin. See RunDiagnostics.record_structure_layer --
                    # pooling the two surfaces understates the bay (~4x on
                    # sigma_cr) once they stop being co-located.
                    entry_lower_flags=[bool(i.entry_lower) for i in intents],
                    full_depth_web_count=sum(
                        1
                        for i in intents
                        if self._is_full_depth_intent(i, effective_families)
                    ),
                )

                # Per-web executed depth / station X / omission code (compact
                # rows; aircraft_core/continuity.py documents the schema).
                # Built from the FINAL intents and omissions of this layer,
                # i.e. exactly what was cut.
                web_rows, web_omit_rows = web_report_rows(  # noqa: F821
                    intents, omitted)
                diagnostics.record(
                    LayerRecord(  # noqa: F821 - supplied by the bundle
                        layer_index=index,
                        print_z=layer.print_z,
                        mutated=True,
                        degradations=layer_degradations,
                        extra={
                            "webs": encode_rows(web_rows),  # noqa: F821
                            "web_omit": encode_rows(web_omit_rows),  # noqa: F821
                            "slots": len(slots),
                            "pieces": len(replacement),
                            "omitted": len(omitted),
                            # How many of this layer's omissions came from the
                            # mitre-clearance predicate specifically. That
                            # predicate can only ADD rejections, so this IS its
                            # whole cost; surfacing it here and in the run
                            # summary makes any structure it costs visible.
                            "mitre_omitted": sum(
                                1 for o in omitted
                                if str(getattr(o, "reason", "")).startswith(
                                    MITRE_OMISSION_MARKER)  # noqa: F821
                            ),
                            "degraded": len(layer_degradations),
                            # present only on a layer rescued by single-station
                            # pair ablation.
                            "pair_ablation": (
                                pair_rescue.as_dict()
                                if pair_rescue is not None else None
                            ),
                            # present only on a layer rescued by the
                            # Z-continuity rule's single-web ablation.
                            **({} if z_rescue is None else {"z_ablation": {
                                "removed_slot_numbers": [
                                    i + 1 for i in z_rescue.removed_indices],
                                "ended": [list(w) for w in z_rescue.ended_ids],
                                "tried": z_rescue.tried,
                                "elapsed_ms": round(z_rescue.elapsed_ms, 3),
                            }}),
                            "families": families_breakdown,
                            "node_merge_count": node_merge_count,
                            "opposing_entry_omission_count": opposing_entry_omission_count,
                            "role_yield_omission_count": role_yield_omission_count,
                            "order_swap_count": order_swap_count,
                            "order_swap_events": order_swap_events,
                            "chord_axis_unavailable": self._chord_axis_unavailable,
                        },
                    )
                )

                if metrics is not None:
                    counts_after = counting_ops.snapshot()
                    layer_boolean_ops = {
                        name: counts_after[name] - counts_before[name]
                        for name in counts_after
                    }
                    metrics.record_layer(
                        LayerMetrics(  # noqa: F821 - supplied by the bundle
                            layer_index=index,
                            print_z=layer.print_z,
                            generation_s=generation_s,
                            slot_build_and_boolean_s=slot_build_and_boolean_s,
                            diagnostics_capture_s=diag_capture_s,
                            validation_s=validation_s,
                            slot_count=len(slots),
                            boolean_ops=layer_boolean_ops,
                            vertex_count_before=vertex_before,
                            vertex_count_after=vertex_after,
                        )
                    )

            if mutated_any:
                # lslices is derived from the region slices and is read
                # downstream; re-derive it or it describes the pre-mutation
                # cross-section (PluginHostSlicing.cpp:279-283).
                layer.make_slices()

        self._root_layers = min(spec.root_solid_layers, layers_seen)
        return diagnostics

    def _build_mesh_sectioner(self, ctx):
        """The ORIGINAL mesh, in PRINT coordinates, ready to cross-section.

        WHY. Spiral Vase deletes interior holes before ``posSlice``, so a CAD
        spar channel never reaches the slices. The MESH is never touched by
        Spiral Vase and IS reachable here, so cross-sectioning it recovers the
        true channel and the model is the source of truth for spar geometry.

        THE TRANSFORM. The obvious reading -- print = rotate/scale(mesh) +
        instance_offset -- is WRONG for this purpose: the slices the plugin
        mutates are in OBJECT-relative coordinates, so instance TRANSLATION
        must NOT be applied (moving the model around the plate does not move
        its slices), while instance rotation, scale and mirror must be, since
        those do change the sliced section. This follows the host source:
        ``ModelObject::raw_bounding_box()`` uses ``get_matrix_no_offset()`` and
        ``PrintObject::PrintObject`` builds ``m_center_offset`` from it
        (``libslic3r/Model.cpp``, ``libslic3r/PrintObject.cpp``). Applying
        instance translation anyway would show up as a bbox disagreement equal
        to the instance offset. The XY-centring step below cancels any
        instance translation by construction, so a test that re-applies
        translation passes regardless; see ``TranslationDropRegressionTests``
        for what is and is not testable here.

        "Object-relative" means centred on the TRANSFORMED MESH'S OWN XY
        bounding box (what libslic3r derives the PrintObject's coordinate
        frame from), not on the raw mesh's own local origin. Omitting that
        centring is invisible for a model authored already centred at (0, 0),
        but an arbitrary CAD file is not: the alignment check then reports
        centroid deltas equal to the mesh's own XY bbox centre. See the
        in-line comment above the centring step for the arithmetic and why
        this is not the same thing as re-applying instance translation.

        So: apply the volume transform, apply instance rotation/scale/mirror,
        subtract the resulting mesh's own XY bounding-box centre, then
        normalise Z so the lowest vertex sits at 0, matching ``print_z``. The
        Z shift is derived from the mesh itself rather than from the instance
        offset, so it stays correct if the object is lifted or dropped on the
        plate.

        Nothing here is TRUSTED: the caller re-checks each layer's section
        against the host's own slice, which is what turned this error into a
        number instead of a spar socket quietly cut in the wrong place.

        Returns ``(sectioner, z_offset_mm)`` or ``None`` when the mesh cannot
        be used. Refuses rather than guesses when the object's composition
        needs a policy decision: several model parts, negative/modifier
        volumes, or several instances all change what "the" geometry means.
        """
        # Read the raw host-applied instance/volume transform
        # BEFORE any of the TopologyError policy checks below can raise -- a
        # refusal (several instances, several parts, a modifier, an unsupported
        # volume mirror) is exactly the case where the recorded placement
        # matters most, and every one of those raises lives inside this same
        # method. Copied into plain tuples/dicts immediately (the lifetime
        # rule: ``mo``/volume/ instance objects are non-owning host references
        # valid only for this ``execute()`` call and must never be stashed on
        # ``self``).
        host_transform = {
            "read": False,
            "reason": None,
            "print_object_trafo": None,
            "print_object_trafo_error": None,
            "volumes": [],
            "volumes_error": None,
            "instances": [],
            "instances_error": None,
            # Filled in at the bottom of this method, from values already
            # computed for ``bounds``/``alignment_stats`` below -- stays
            # None if a TopologyError raises first, which is itself the
            # honest record: no plugin-applied transform exists for a run
            # that never got past the composition check.
            "plugin_applied": None,
        }
        self._host_object_transform = host_transform

        obj = getattr(ctx, "object", None)
        if obj is None or not hasattr(obj, "model_object"):
            host_transform["reason"] = "ctx.object is None or lacks model_object()"
            return None

        # PrintObject.trafo() (PluginHostSlicing.cpp:322) is the single
        # value closest to what the host actually sliced with, but it
        # requires numpy in the embedded interpreter and this plugin
        # declares no dependencies (PEP 723 ``dependencies = []`` in
        # aircraft_structures.py) -- so it may not be importable there.
        # Opportunistic: record the exception rather than swallowing it,
        # never let it fail the run.
        try:
            trafo = obj.trafo()
            host_transform["print_object_trafo"] = (
                None if trafo is None else
                trafo.tolist() if hasattr(trafo, "tolist")
                else [list(row) for row in trafo]
            )
        except Exception as exc:  # noqa: BLE001
            host_transform["print_object_trafo_error"] = "{}: {}".format(
                type(exc).__name__, exc
            )

        try:
            mo = obj.model_object()
        except Exception as exc:  # noqa: BLE001
            host_transform["reason"] = "model_object() raised {}: {}".format(
                type(exc).__name__, exc
            )
            return None
        if mo is None:
            host_transform["reason"] = "model_object() returned None"
            return None

        def _sign_product(scaling_factor, mirror):
            """Determinant sign of ONE placement's linear (rotation-free)
            part -- from ``mirror`` ALONE, not from ``scaling_factor``, per the
            host source (``Transformation::get_scaling_factor``,
            ``Geometry.cpp``: ``return { std::abs(scale(0,0)),
            std::abs(scale(1,1)), std::abs(scale(2,2)) }`` -- always
            non-negative magnitudes, never sign-bearing; and
            ``Transformation::get_mirror``: ``return {
            scale(0,0)/std::abs(scale(0,0)), ... }`` -- exactly +-1 per axis,
            carrying the entire sign).

            Still requires ``scaling_factor`` to have been read -- never a
            confident sign from missing data -- because an unread scale
            could in principle be exactly zero (a degenerate,
            non-invertible transform) that this function has no way to
            rule out without having seen it; a zero scale on any axis
            reports a degenerate ``0`` rather than a sign.
            """
            if scaling_factor is None or mirror is None:
                return None
            if any(s == 0.0 for s in scaling_factor):
                return 0
            product = 1.0
            for m in mirror:
                product *= m
            if product == 0.0:
                return 0
            return 1 if product > 0 else -1

        def _record_placement(placement_obj):
            """Raw offset/rotation/scaling_factor/mirror/matrix off ONE
            ``ModelVolume`` or ``ModelInstance``, member-by-member so a
            single unreadable member does not blank the rest of the
            record.

            offset/rotation/scaling_factor/mirror FEED GEOMETRY (baked
            into mesh vertices below) -- catching their exceptions here is
            ONLY so the report can name exactly which member failed and
            why; it does NOT mean the run may still succeed. The geometry
            path (``_require_geometry_member``, called on ``vol``/``inst``
            after this) refuses the slice, immediately, the first time it
            finds one of these four ``None`` -- whether ``None`` because
            the read raised, or because the host does not expose the
            member at all (recorded as ``"absent"``, never silently
            defaulted to identity: an unknown mirror is not a proven
            IDENTITY mirror, and a silent identity substitution here would
            fail open).

            ``matrix()`` (below, separately) never feeds geometry (the
            decomposed values above are what actually gets applied -- see
            the module comment on why), so it is fully resilient: catch,
            record, and let the run continue regardless. So does ``is_left_handed()`` (recorded by the
            caller, once per instance) -- diagnostic-only, by construction,
            never capable of flipping a section.
            """
            rec = {}
            for name in ("offset", "rotation", "scaling_factor", "mirror"):
                if not hasattr(placement_obj, name):
                    rec[name] = None
                    rec["{}_error".format(name)] = "absent"
                    continue
                try:
                    rec[name] = tuple(getattr(placement_obj, name)())
                except Exception as exc:  # noqa: BLE001
                    rec[name] = None
                    rec["{}_error".format(name)] = "{}: {}".format(
                        type(exc).__name__, exc
                    )
            # ``matrix()`` NEVER feeds geometry (see this function's own
            # docstring) -- resilient by design, not merely by omission:
            # catch, record, continue, regardless of outcome.
            try:
                matrix = placement_obj.matrix()
                rec["matrix"] = (
                    None if matrix is None else
                    matrix.tolist() if hasattr(matrix, "tolist")
                    else [list(row) for row in matrix]
                )
            except Exception as exc:  # noqa: BLE001
                rec["matrix"] = None
                rec["matrix_error"] = "{}: {}".format(type(exc).__name__, exc)
            # Determinant sign from ``mirror`` alone (see ``_sign_product``'s
            # own docstring for the source proof) -- the plugin's OWN
            # computed mirror flag, independent of whatever the host's
            # ``is_left_handed()``/``matrix()`` separately report, so a
            # disagreement between them is visible rather than hidden
            # behind a single trusted source.
            rec["determinant_sign"] = _sign_product(
                rec.get("scaling_factor"), rec.get("mirror")
            )
            return rec

        def _require_geometry_member(rec, label, name):
            """The recorded value for ONE placement member that FEEDS
            GEOMETRY (offset/rotation/scaling_factor/mirror) -- fails
            closed, immediately, if it is missing for ANY reason (the read
            raised, or the host does not expose the member at all). Unlike
            ``trafo()``/``matrix()``/``is_left_handed()`` (recording-only,
            see ``_record_placement``'s own docstring), these four values
            are baked directly into the mesh vertices a few lines below,
            so silently substituting an identity default here means a
            wrong, or MIRRORED, section gets cut without the alignment
            guard necessarily catching it (a determinant flip does not
            always move the bounding box -- see
            ``MirrorDetectionRestsOnCentroidTermsTests``). ``rec`` is the
            SAME dict already attached to ``self._host_object_transform``,
            so the refusal this raises is already backed by a report that
            names the exact error.
            """
            value = rec.get(name) if rec else None
            if value is not None:
                return value
            error = (rec or {}).get("{}_error".format(name), "absent")
            aspect = "handedness" if name == "mirror" else "placement"
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                0,
                0.0,
                "{} {}() could not be read ({}); the section's {} is "
                "unknown, refusing rather than guessing."
                .format(label, name, error, aspect)
            )

        # ``mo.volumes()`` itself raising is NOT swallowed: recorded, then
        # RE-RAISED, so the ORIGINAL exception reaches the run's actual
        # failure-reporting path (mesh_note / the unexpected-exception
        # report) instead of being silently turned into a misleading
        # "model has 0 part volume(s)" TopologyError a few lines below --
        # a real failure (the host call itself broke) reported as if it
        # were a normal, well-formed empty model.
        try:
            vols = [v for v in mo.volumes()]
        except Exception as exc:  # noqa: BLE001
            host_transform["volumes_error"] = "{}: {}".format(
                type(exc).__name__, exc
            )
            raise
        volume_records = [_record_placement(v) for v in vols]
        host_transform["volumes"] = volume_records
        # Keyed by identity, not equality, so the later mirror-reuse lookup
        # (see ``v_mir``/``i_mir`` below) finds exactly the object actually
        # read, never a coincidentally-equal but distinct one.
        volume_records_by_id = {id(v): rec for v, rec in zip(vols, volume_records)}

        # ``mo.instances()`` itself raising is NOT swallowed either --
        # same reasoning as ``mo.volumes()`` above: recorded, then
        # RE-RAISED, so the ORIGINAL exception reaches the run's actual
        # failure-reporting path instead of a misleading "model object has
        # 0 instances" TopologyError a few lines below.
        try:
            insts_for_record = [i for i in mo.instances()]
        except Exception as exc:  # noqa: BLE001
            host_transform["instances_error"] = "{}: {}".format(
                type(exc).__name__, exc
            )
            raise
        instance_records = []
        instance_records_by_id = {}
        for inst in insts_for_record:
            irec = _record_placement(inst)
            # is_left_handed() (PluginHostModel.cpp:86) is the host's OWN
            # mirror flag -- recorded alongside, not instead of, the
            # plugin's decomposed determinant_sign above.
            try:
                irec["is_left_handed"] = bool(inst.is_left_handed())
            except Exception as exc:  # noqa: BLE001
                irec["is_left_handed"] = None
                irec["is_left_handed_error"] = "{}: {}".format(
                    type(exc).__name__, exc
                )
            instance_records.append(irec)
            instance_records_by_id[id(inst)] = irec
        host_transform["instances"] = instance_records

        # "read" means BOTH volumes and instances were actually read from
        # the host -- not just that the two ``try`` blocks above ran
        # without this method itself crashing. A volumes() failure never
        # reaches this line at all (re-raised above); an instances()
        # failure must not be reported as a successful read just because
        # the (now-empty) instances list built without raising.
        host_transform["read"] = (
            host_transform["volumes_error"] is None
            and host_transform["instances_error"] is None
        )

        # A single top-level handedness figure for the object AS A WHOLE
        # -- volume determinant sign x instance determinant sign -- only
        # when there is exactly one of each (the shape every downstream
        # policy check below requires anyway) and both signs are actually
        # known; ``None`` otherwise, never a guessed +1.
        host_transform["object_determinant_sign"] = None
        if len(volume_records) == 1 and len(instance_records) == 1:
            v_sign = volume_records[0].get("determinant_sign")
            i_sign = instance_records[0].get("determinant_sign")
            if v_sign is not None and i_sign is not None:
                host_transform["object_determinant_sign"] = v_sign * i_sign

        parts = [v for v in vols if getattr(v, "is_model_part", lambda: True)()]
        modifiers = [v for v in vols if v not in parts]
        if modifiers:
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                0,
                0.0,
                "model has {} modifier/negative volume(s); the plugin recovers spar "
                "geometry from the model mesh and cannot know how a modifier "
                "is meant to change it. Flatten the model, or declare the "
                "spar in config and enable allow_config_spar_fallback."
                .format(len(modifiers))
            )
        if len(parts) != 1:
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                0,
                0.0,
                "model has {} part volume(s); expected exactly 1. Which one "
                "carries the spar channel is a policy question the mesh "
                "cannot answer.".format(len(parts))
            )

        vol = parts[0]
        mesh = vol.mesh()
        n = int(mesh.facets_count())

        # Volume-local placement. ``matrix`` needs numpy (not a declared
        # dependency), so the decomposed offset/rotation/scale are used; they
        # cover everything but shear, and a shear would be caught by the
        # per-layer alignment check.
        #
        # Each of these is read EXACTLY ONCE (via ``volume_records_by_id``,
        # the SAME record already attached to ``self._host_object_transform``
        # -- never a second ``vol.offset()``/``vol.rotation()``/etc. call),
        # and ``_require_geometry_member`` refuses the slice immediately if
        # any of them is missing for any reason. Falling back to an identity
        # mirror when ``mirror()`` raised or was absent would slice a mirrored
        # object with a FLIPPED section that the guard may not catch, because
        # a determinant flip does not always move the bounding box (see
        # ``MirrorDetectionRestsOnCentroidTermsTests``). An unknown
        # placement/handedness is refused, not guessed.
        v_rec = volume_records_by_id.get(id(vol))
        v_off = _require_geometry_member(v_rec, "volume", "offset")
        v_rot = _require_geometry_member(v_rec, "volume", "rotation")
        v_scl = _require_geometry_member(v_rec, "volume", "scaling_factor")
        # The instance mirror is applied below; ignoring the VOLUME mirror
        # while honouring the instance one would be an indefensible asymmetry
        # that silently flips a section. Refuse instead: a quiet wrong answer
        # is worse than a loud unsupported one.
        v_mir = _require_geometry_member(v_rec, "volume", "mirror")
        if any(abs(m - 1.0) > 1e-9 for m in v_mir):
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                0,
                0.0,
                "model volume carries a mirror {} which this plugin does not "
                "apply; the recovered spar geometry would be flipped relative "
                "to what is printed.".format(v_mir)
            )

        # Reused from ``insts_for_record`` above -- NOT a second
        # ``mo.instances()`` call. Same non-owning-reference objects,
        # still valid for the rest of this ``execute()`` call (lifetime
        # rule); a second host call here would be redundant at best and,
        # on a host whose ``instances()`` is not perfectly idempotent,
        # a second source of truth to disagree with the first.
        insts = insts_for_record
        if len(insts) != 1:
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                0,
                0.0,
                "model object has {} instances; expected exactly 1."
                .format(len(insts))
            )
        inst = insts[0]
        # Same one-read/fail-closed contract as the volume above. Instance
        # OFFSET is deliberately never required here (translation stays
        # dropped -- see this method's own docstring), only
        # rotation/scale/mirror, which DO feed geometry.
        i_rec = instance_records_by_id.get(id(inst))
        i_rot = _require_geometry_member(i_rec, "instance", "rotation")
        i_scl = _require_geometry_member(i_rec, "instance", "scaling_factor")
        i_mir = _require_geometry_member(i_rec, "instance", "mirror")

        def rot(p, r):
            x, y, z = p
            cx, sx = math.cos(r[0]), math.sin(r[0])
            cy, sy = math.cos(r[1]), math.sin(r[1])
            cz, sz = math.cos(r[2]), math.sin(r[2])
            y, z = y * cx - z * sx, y * sx + z * cx
            x, z = x * cy + z * sy, -x * sy + z * cy
            x, y = x * cz - y * sz, x * sz + y * cz
            return (x, y, z)

        verts = []
        for i in range(int(mesh.vertex_count())):
            p = mesh.vertex(i)
            p = (p[0] * v_scl[0], p[1] * v_scl[1], p[2] * v_scl[2])
            p = rot(p, v_rot)
            p = (p[0] + v_off[0], p[1] + v_off[1], p[2] + v_off[2])
            p = (p[0] * i_mir[0] * i_scl[0],
                 p[1] * i_mir[1] * i_scl[1],
                 p[2] * i_mir[2] * i_scl[2])
            # Instance TRANSLATION deliberately omitted -- see the docstring.
            verts.append(rot(p, i_rot))

        # XY-centre on the transformed mesh's OWN bounding box. Without this
        # step the result is only correct for a model AUTHORED already centred
        # at (0, 0) in this same decomposed rotation/scale/mirror frame (as the
        # fixtures under tests/fixtures/wing/ are). libslic3r centres a PrintObject's slices on its own snug
        # XY bounding box (computed from the part meshes under the volume
        # transform and the instance's rotation/scale/mirror, with instance
        # translation excluded -- the same decomposition already built above);
        # the slices this plugin mutates are in that centred frame, not in the
        # raw mesh's own local origin.
        #
        # The signature of a missing centring step is every bbox-edge and
        # centroid term being off by exactly the mesh's own XY bbox centre
        # (e.g. a mesh with bounds x [-10.840, 5.501], y [-66.534, 148.420]
        # has centre (-2.6695, 40.943), and the reported centroid deltas match
        # it), as opposed to an unapplied instance translation.
        #
        # Adding ``+ i_off[0]``/``+ i_off[1]`` right here would be a NO-OP:
        # the XY component is cancelled by the centring below (subtracting a
        # box's own bbox centre erases any translation applied to it, by
        # construction), and any Z component is equally cancelled by the
        # ``z_min`` normalisation further down. So applying instance
        # translation is structurally indistinguishable from not applying it,
        # for ANY mesh, and the alignment guard cannot verify the choice.
        # Excluding it remains correct: it matches
        # ``ModelObject::raw_bounding_box()``'s ``get_matrix_no_offset()``
        # (``libslic3r/Model.cpp``) and ``PrintObject::PrintObject``'s
        # ``m_center_offset`` computation from it
        # (``libslic3r/PrintObject.cpp``). ``TranslationDropRegressionTests``
        # is scoped accordingly.
        #
        # This is NOT "measure the host's slice and subtract the difference":
        # the centre is computed from the mesh alone,
        # before any host slice is read, exactly mirroring what the host itself
        # does to build its own coordinate frame. A pure rigid translation of
        # the mesh relative to its own geometry is invisible to this centring
        # by construction -- but it is equally invisible to the host's own
        # centring, so that is correct, not a gap: rotation, mirror, scale and
        # Z errors still change the section's shape and the asymmetric centroid
        # position, so the per-layer alignment guard below still catches them.
        if verts:
            x_center = (
                min(v[0] for v in verts) + max(v[0] for v in verts)
            ) / 2.0
            y_center = (
                min(v[1] for v in verts) + max(v[1] for v in verts)
            ) / 2.0
        else:
            x_center = 0.0
            y_center = 0.0
        verts = [(v[0] - x_center, v[1] - y_center, v[2]) for v in verts]

        # Z-normalise: the host's layers start at print_z 0 on the bed.
        z_min = min(v[2] for v in verts) if verts else 0.0
        verts = [(v[0], v[1], v[2] - z_min) for v in verts]

        tris = [tuple(mesh.triangle(i)) for i in range(n)]

        # ONE global theta, detected from the mesh, ONCE PER RUN, computed
        # HERE -- after XY-centring/Z-normalisation, before the
        # sectioner this whole run hands out is finalised -- so every
        # downstream consumer (per-layer mesh sectioning below, and the
        # per-layer host-slice mutation in ``execute()``) works in the SAME
        # already-canonical frame. Freshly computed every call, never read off
        # ``self`` from a prior run (a stale value would leak across runs).
        #
        # Method: ``resolve_wing_theta_deg`` (mid-span station
        # authoritative; a few interior stations sampled only to bound the
        # spread -- see that function's own docstring for why NOT
        # averaging/median-blending them is the point). The underlying
        # per-station angle is the CHORD-LINE axis
        # (``chord_line_angle_deg``), not the camber-biased
        # ``min_area_rect_angle_deg``.
        alignment_stats = {
            "theta_deg": 0.0,
            "rotation_applied": False,
            "epsilon_deg": THETA_EPSILON_DEG,  # noqa: F821 - bundle constant
            "mid_span_theta_deg": None,
            "station_theta_deg": [],
            "spread_deg": None,
            "excluded_stations": [],
            "error": None,
            # provenance for a theta resolved from a sharp-TE band substituted
            # for a blunt mid-span station (elevon wings). See
            # ``resolve_wing_theta_deg``'s own docstring for the shape.
            "theta_source": None,
            "sharp_band_fracs": None,
            "sharp_band_spread_deg": None,
            "blunt_crosscheck": None,
            # provenance for a theta read from the mid-span thin trailing-edge
            # flat when no sharp band exists (None otherwise).
            "thin_te_flat": None,
        }
        theta_deg = None
        probe = MeshSectioner(verts, tris)  # noqa: F821 - supplied by the bundle
        z_lo, z_hi = probe.z_min, probe.z_max
        if z_lo is not None and z_hi is not None and z_hi > z_lo:
            def _outer_loop_points(frac):
                z = z_lo + frac * (z_hi - z_lo)
                loops = probe.section_at(z)
                outer = [lp for lp in loops if not lp.is_interior]
                if not outer:
                    return None
                best = max(outer, key=lambda lp: abs(lp.signed_area))
                return list(best.points)

            mid_pts = _outer_loop_points(0.5)
            other_pts = [
                pts for pts in (
                    _outer_loop_points(f) for f in (0.1, 0.3, 0.7, 0.9)
                ) if pts is not None
            ]
            if mid_pts is None:
                alignment_stats["error"] = (
                    "mid-span (50% of Z range) section has no outer loop; "
                    "cannot detect global chord alignment"
                )
            else:
                band_provenance = {}
                try:
                    theta_deg, station_thetas, spread, excluded = (
                        resolve_wing_theta_deg(  # noqa: F821 - bundle
                            mid_pts, other_pts,
                            # substitute a contiguous band of sharp-TE span
                            # stations when the mid-span station is squared off
                            # (an elevon/aileron cutout) -- see
                            # resolve_wing_theta_deg's own docstring. Only
                            # activates on BluntTrailingEdgeError at mid-span;
                            # every other path here is unchanged.
                            station_sampler=_outer_loop_points,
                            provenance=band_provenance,
                        )
                    )
                    # NOTE: "mid_span_theta_deg" carries the BAND theta, not
                    # a literal mid-span reading, when theta_source ==
                    # "sharp_te_band" (mid-span itself was blunt).
                    alignment_stats["mid_span_theta_deg"] = theta_deg
                    alignment_stats["station_theta_deg"] = station_thetas
                    alignment_stats["spread_deg"] = spread
                    alignment_stats["theta_source"] = band_provenance.get("theta_source")
                    if alignment_stats["theta_source"] in (
                            "sharp_te_band", "thin_te_flat"
                        ):
                        # This run's theta was resolved via the sharp-TE band
                        # (or the thin-flat source), so an excluded OTHER
                        # station's own blunt-TE message -- which says outright
                        # "The slice is refused rather than sliced unrotated"
                        # -- would misreport a run that in fact succeeded. Trim
                        # to the diagnostic prefix and annotate rather than
                        # carry the refusal sentence.
                        _theta_from = (
                            "sharp-TE band"
                            if alignment_stats["theta_source"] == "sharp_te_band"
                            else "thin trailing-edge flat"
                        )
                        alignment_stats["excluded_stations"] = [
                            {
                                "index": i,
                                "reason": reason.split(". The slice is refused", 1)[0]
                                + (
                                    # keep the thin-flat refusal reason on the
                                    # thin-flat path only.
                                    " Thin-flat reading also refused:"
                                    + reason.split(" Thin-flat reading also refused:", 1)[1]
                                    if alignment_stats["theta_source"] == "thin_te_flat"
                                    and " Thin-flat reading also refused:" in reason
                                    else ""
                                )
                                + " (excluded from the spread check only; "
                                "this run's theta was resolved from the "
                                + _theta_from + ", not this station).",
                            }
                            for i, reason in excluded
                        ]
                    else:
                        alignment_stats["excluded_stations"] = [
                            {"index": i, "reason": reason} for i, reason in excluded
                        ]
                    alignment_stats["sharp_band_fracs"] = band_provenance.get("band_fracs")
                    alignment_stats["sharp_band_spread_deg"] = band_provenance.get("band_spread_deg")
                    alignment_stats["blunt_crosscheck"] = band_provenance.get("blunt_crosscheck")
                    alignment_stats["thin_te_flat"] = band_provenance.get("thin_te_flat")
                except ChordAxisUnavailableError as exc:  # noqa: F821 - bundle
                    # NOT a wing alignment problem -- the mid-span
                    # section has no chord axis at all (no hull area, or
                    # no genuine trailing edge anywhere). Rectangular
                    # structural-coupon fixtures (every corner exactly
                    # 90 deg) take this path; they are not wings -- see
                    # ``ChordAxisUnavailableError``'s own docstring for the
                    # distinction. Soft: theta stays 0/no rotation and the run
                    # continues. This is the one case where a failing mid-span
                    # station does not fail the slice closed; it is recorded
                    # in ``alignment_stats`` and counted per layer.
                    alignment_stats["error"] = str(exc)
                    alignment_stats["chord_axis_unavailable"] = True
                    theta_deg = None
                except BluntTrailingEdgeError as exc:  # noqa: F821 - bundle
                    # The mid-span station was squared off AND either no
                    # sampler-derived sharp-TE band cleared every guard, or
                    # no sampler-eligible substitution exists at all. The underlying message (from
                    # ``resolve_theta_deg``/``_resolve_sharp_band_theta``)
                    # already states explicitly that rotating the model in the
                    # Orca GUI will NOT help -- a squared-off TE reads the same
                    # at every rotation -- so this wrapper must NOT append the
                    # generic "consider rotating" advice the plain
                    # ``AlignmentError`` branch below still gives for genuinely
                    # rotation-fixable problems. Caught BEFORE
                    # ``AlignmentError`` (its own base class) specifically to
                    # keep those two messages from contradicting each other.
                    alignment_stats["error"] = str(exc)
                    self._alignment_fatal_error = (
                        "global wing alignment could not be resolved: {} "
                        "-- this is a squared-off trailing edge, not a "
                        "rotation problem; the model needs a genuine "
                        "sharp trailing edge somewhere the plugin can "
                        "read a chord from.".format(exc)
                    )
                    theta_deg = None
                except AlignmentError as exc:  # noqa: F821 - supplied by the bundle
                    # A genuine, aerofoil-shaped section (a real chord axis
                    # WAS found) with an unresolved alignment problem
                    # (leading edge ambiguous, or per-station spread beyond
                    # the washout envelope). Fail the
                    # slice closed, naming the detected theta per station
                    # (already embedded in the exception message) and the
                    # manual rotation the user could apply in the Orca
                    # GUI. Deferred to the first body layer (where a layer
                    # index/Z exist to name), mirroring how ``mesh_note``
                    # already defers its own fail-closed raise to
                    # ``_channels_for_layer``.
                    alignment_stats["error"] = str(exc)
                    self._alignment_fatal_error = (
                        "global wing alignment could not be resolved: {} "
                        "-- if this is a real, non-worsening washout, "
                        "consider rotating the model in the Orca GUI so "
                        "its chord sits closer to a single consistent "
                        "angle; if it is a genuinely symmetric/ambiguous "
                        "section, this plugin cannot auto-detect its "
                        "leading edge and the model needs a discernible "
                        "aerofoil shape at the sampled span "
                        "stations.".format(exc)
                    )
                    theta_deg = None
        else:
            alignment_stats["error"] = "mesh has no Z extent to sample span stations from"

        rotation_applied = False
        if theta_deg is not None and abs(theta_deg) >= THETA_EPSILON_DEG:  # noqa: F821
            xy = [(v[0], v[1]) for v in verts]
            rotated_xy = rotate_points_deg(xy, -theta_deg)  # noqa: F821 - bundle
            verts = [(rx, ry, v[2]) for (rx, ry), v in zip(rotated_xy, verts)]
            rotation_applied = True
        alignment_stats["theta_deg"] = theta_deg if theta_deg is not None else 0.0
        alignment_stats["rotation_applied"] = rotation_applied
        self._alignment_theta_deg = alignment_stats["theta_deg"] if rotation_applied else 0.0
        # The run-level classification, exposed so every mutated layer's own
        # LayerRecord can carry it and RunDiagnostics can report both a
        # top-level flag AND a per-run affected-layer count
        # (chord_axis_unavailable_layer_count) -- never silent. See that
        # property's own docstring for why.
        self._chord_axis_unavailable = bool(
            alignment_stats.get("chord_axis_unavailable", False)
        )

        # Vertices are in print coordinates (and, if rotation_applied,
        # already de-rotated into the canonical chord-along-X frame), so
        # the section plane is the layer's own print_z and no separate Z
        # offset is needed. The value is still returned so the caller can
        # log what was applied.
        bounds = {
            "vertices": len(verts),
            "triangles": len(tris),
            "x": [min(v[0] for v in verts), max(v[0] for v in verts)],
            "y": [min(v[1] for v in verts), max(v[1] for v in verts)],
            "z": [min(v[2] for v in verts), max(v[2] for v in verts)],
            "xy_center_applied_mm": [x_center, y_center],
            "alignment": alignment_stats,
        }

        # What THIS plugin derived from the raw host
        # placement recorded at the top of this method -- the global alignment
        # rotation, the Z shift, and the XY centring -- so the report carries
        # both the host's own values and what the plugin did with them, side by
        # side.
        applied_theta_deg = alignment_stats["theta_deg"]
        theta_rad = math.radians(applied_theta_deg) if applied_theta_deg else 0.0
        host_transform["plugin_applied"] = {
            "theta_deg": applied_theta_deg,
            "rotation_applied": alignment_stats["rotation_applied"],
            "rotation_matrix_2x2": [
                [math.cos(theta_rad), -math.sin(theta_rad)],
                [math.sin(theta_rad), math.cos(theta_rad)],
            ],
            "z_shift_mm": float(z_min),
            "xy_center_mm": [x_center, y_center],
        }

        return (MeshSectioner(verts, tris),  # noqa: F821 - supplied by the bundle
                float(z_min), bounds)

    def _section_plane_for(self, layer):
        """The Z the HOST actually sliced this layer at.

        ``print_z`` is the nozzle height for the layer; ``slice_z`` is the
        plane the geometry came from, half a layer height lower (the probe
        records ``print_z 8.2`` against ``slice_z 8.1``). Sectioning the mesh
        at ``print_z`` samples half a layer too high, which on a tapering wing
        is a real chord error -- the fixture tapers 0.2503 mm of chord per mm
        of Z, so half a 0.2 mm layer is ~0.0125 mm per edge, the same order as
        the entire mesh/host disagreement being measured. Prefer ``slice_z``;
        fall back only if the host does not expose it.
        """
        z = getattr(layer, "slice_z", None)
        if z is None:
            return float(layer.print_z), False
        return float(z), True

    def _channels_for_layer(self, sectioner, mesh_note, index, layer):
        """Spar channels recovered from the MODEL for this layer.

        FAILS CLOSED, deliberately. Every path that cannot produce an answer
        raises rather than returning ``()``, because ``()`` means "this model
        has no spar" to the generator, and with the shipped config carrying no
        spar geometry that silently produces a wing with no bore -- which
        satisfies the one-loop invariant and passes ``gcode_spiral_check``
        while being useless (a missing member removes geometry without
        breaking topology). The only legitimate
        empty result is a section that WAS computed and contained no interior
        loop, which is simply a model with no spar channel.
        """
        if sectioner is None:
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                index,
                layer.print_z,
                "the model mesh could not be read, so the spar channel cannot "
                "be recovered from it ({}). The model must be the "
                "source of truth for spar geometry; refusing to slice a wing "
                "whose spar bore would silently be absent."
                .format(mesh_note or "no model object on this print object")
            )

        z, used_slice_z = self._section_plane_for(layer)
        stats = self._mesh_alignment
        stats["used_slice_z"] = used_slice_z
        try:
            loops = sectioner.section_at(z)
        except Exception as exc:  # noqa: BLE001
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                index,
                layer.print_z,
                "cross-sectioning the model mesh at Z={:.4f} mm failed ({}: "
                "{}). The spar channel cannot be recovered, and continuing "
                "would produce a wing with no spar bore that still passes "
                "every topology check.".format(
                    z, type(exc).__name__, str(exc)[:200])
            )

        self._last_loops = loops
        stats["layers_sectioned"] += 1

        # ``sectioner.last_nudge_mm`` is the offset ACTUALLY applied by the
        # section that just succeeded (0.0 if the unnudged plane worked) --
        # name the plane really cut, not the plane requested, in every
        # diagnostic below.
        applied_z = z + sectioner.last_nudge_mm
        # Diagnostic-only: the exact plane the sectioner just cut, for
        # ``_check_mesh_alignment``'s contour dump on THIS layer -- set here
        # and read via getattr in the sibling method, like ``_last_loops``
        # immediately above.
        self._last_applied_slice_z = applied_z
        if sectioner.last_nudge_mm != 0.0:
            stats["section_nudge_count"] += 1
            if stats["section_nudge_first"] is None:
                stats["section_nudge_first"] = {
                    "layer": index,
                    "requested_z": round(z, 4),
                    "offset_mm": sectioner.last_nudge_mm,
                }

        inner = [lp for lp in loops if lp.is_interior]
        k = str(len(inner))
        stats["interior_loop_counts"][k] = stats["interior_loop_counts"].get(k, 0) + 1
        if len(inner) != 1 and len(stats["anomalies"]) < 12:
            outer_all = [lp for lp in loops if not lp.is_interior]
            stats["anomalies"].append({
                "layer": index, "print_z": round(float(layer.print_z), 4),
                "section_z": round(applied_z, 4),
                "n_loops": len(loops), "n_outer": len(outer_all),
                "n_inner": len(inner),
                "areas": [round(lp.signed_area, 4) for lp in loops[:6]],
            })
        for lp in inner:
            cx = lp.centroid[0]
            dia = lp.equivalent_diameter_mm
            if stats["channel_cx_min"] is None or cx < stats["channel_cx_min"]:
                stats["channel_cx_min"] = cx
            if stats["channel_cx_max"] is None or cx > stats["channel_cx_max"]:
                stats["channel_cx_max"] = cx
            if stats["channel_dia_min"] is None or dia < stats["channel_dia_min"]:
                stats["channel_dia_min"] = dia
            if stats["channel_dia_max"] is None or dia > stats["channel_dia_max"]:
                stats["channel_dia_max"] = dia
            if stats["first_channel"] is None:
                stats["first_channel"] = {
                    "layer": index, "section_z": round(applied_z, 4),
                    "cx": round(cx, 5), "cy": round(lp.centroid[1], 5),
                    "diameter": round(dia, 5)}
        return tuple(
            DetectedChannel(  # noqa: F821 - supplied by the bundle
                center_x_mm=lp.centroid[0],
                center_y_mm=lp.centroid[1],
                diameter_mm=lp.equivalent_diameter_mm,
            )
            for lp in inner
        )

    def _check_mesh_alignment(self, index, layer, frame, spec, sectioner):
        """Verify the mesh section against the host's OWN slice of this layer.

        Returns a truthy dict (the skip record: layer index, print_z, band,
        all six deltas) when the caller must SKIP this layer entirely --
        leave every region unmutated, no ``slices.set()``, no
        ``make_slices()`` -- and ``None`` for the ordinary case, where the
        caller proceeds to generate and cut as normal. The result is never
        passed through instance state: the caller acts on the return value
        directly.

        This is the only guard against a transform that is subtly wrong: a bad
        rotation convention or an unapplied mirror still yields a perfectly
        plausible section, just in the wrong place, and the spar socket is then
        cut where the tube is not.

        Comparing only ``min_x``/``max_x`` is too weak: a 180-degree error
        about Z maps x -> -x, and on a section whose X extent is
        near-symmetric about the object origin that leaves both X bounds
        unchanged and reports ~0. All four bbox edges
        and the section centroid are compared -- a cambered aerofoil's
        centroid is off-centre in BOTH axes, so a Z-flip or a Y-mirror moves
        it.
        """
        loops = getattr(self, "_last_loops", None)
        if not loops:
            return None
        stats = self._mesh_alignment
        raw_outer = [lp for lp in loops if not lp.is_interior]
        if not raw_outer:
            return None
        # The host closed its slice (``slice_closing_radius``: grow r, union,
        # shrink r, mitre joins) BEFORE this hook ran; the mesh sectioner did
        # not. Apply the same closing to the mesh's outer loops so two islands
        # the host bridged read as the ONE contour the host reports. Tolerance
        # is unchanged. The radius is read from the live print config and never
        # defaulted.
        try:
            closing_radius_mm = parse_closing_radius_mm(  # noqa: F821 - bundle
                getattr(self, "_closing_radius_raw", None))
        except ClosingRadiusError as exc:  # noqa: F821 - supplied by the bundle
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                index, layer.print_z, str(exc))
        try:
            closed = close_outer_loops(  # noqa: F821 - supplied by the bundle
                frame.ops, raw_outer, closing_radius_mm,
                skip_single_loop=True)
        except NotImplementedError as exc:
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                index, layer.print_z,
                "the mesh-alignment guard could not emulate the host's "
                "slice closing (slice_closing_radius={} mm): {}".format(
                    closing_radius_mm, exc))
        if not closed.outer:
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                index, layer.print_z,
                "closing the mesh section at slice_closing_radius={} mm left "
                "no outer contour ({} outer loop(s) before closing).".format(
                    closing_radius_mm, closed.loops_before))
        outer = closed.outer
        raw_best = max(raw_outer, key=lambda lp: abs(lp.signed_area))
        best = max(outer, key=lambda lp: abs(lp.signed_area))
        mnx, mny, mxx, mxy = best.bbox

        # Enrich THIS layer's ``_last_geometry`` stub (written at the top of
        # the layer loop in ``_transform``, same ``index``) with our own
        # section's outer loop as soon as it is known -- before any raise
        # site below in this method can fire. Otherwise a failure here would
        # report whatever a PRIOR layer's full assignment last left behind.
        if getattr(self, "_last_geometry", None) is not None:
            self._last_geometry["our_section_pts_mm"] = [
                [round(px, 5), round(py, 5)] for px, py in best.points
            ]

        # The frame's own outline. No getattr-with-default here: reading a
        # nonexistent ``frame.points`` attribute (and defaulting) would make
        # the Y and centroid terms silently dead and quietly degrade the check
        # to a min_x/max_x comparison. If the outline cannot be obtained that
        # is a defect, not a branch to skip.
        frame_pts = frame.ops.contour_points_mm(frame.polygon)
        if getattr(self, "_last_geometry", None) is not None:
            self._last_geometry["host_contour_pts_mm"] = [
                [round(px, 5), round(py, 5)] for px, py in frame_pts
            ]
        if not frame_pts:
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                index,
                layer.print_z,
                "the host's own slice for this layer yielded no outline "
                "points, so the mesh cross-section cannot be checked against "
                "it. Refusing to place a spar socket from an unverified "
                "mesh-to-slice transform.",
            )

        f_cx, f_cy = _polygon_centroid(frame_pts)  # noqa: F821 - bundle helper
        m_cx, m_cy = best.centroid

        # Six terms. All four bbox edges plus the centroid, because a
        # 180-degree error about Z maps x -> -x and, on a section whose X
        # extent is near-symmetric about the object origin, leaves BOTH X
        # bounds unchanged -- an X-only check reports ~0 for it. A cambered
        # aerofoil's area-weighted centroid is off-centre in both axes, so a
        # Z-flip or a Y-mirror moves it. Both centroids are area-weighted, so
        # they are like-for-like across two differently tessellated outlines;
        # a vertex average would not be.
        # ``mesh_value - host_value`` per term, kept alongside
        # the absolute ``deltas`` below (every entry of ``deltas`` is exactly
        # ``abs(signed_deltas[term])``) so the sensitivity sign-gate can compare an OBSERVED
        # disagreement's sign against the mechanism's PREDICTED sign. See
        # the module comment above ``MESH_ALIGNMENT_SENSITIVITY_K`` for why
        # an absolute-only delta cannot support that comparison.
        signed_deltas = {
            "min_x": mnx - frame.min_x,
            "max_x": mxx - frame.max_x,
            "min_y": mny - frame.min_y,
            "max_y": mxy - frame.max_y,
            "centroid_x": m_cx - f_cx,
            "centroid_y": m_cy - f_cy,
        }
        deltas = {k: abs(v) for k, v in signed_deltas.items()}
        # Recorded on ALL SIX terms unconditionally -- ``worst_by_term``/
        # ``worst_bbox_delta_*`` stay a complete, honest record of every
        # disagreement this run ever measured, whether or not this layer
        # goes on to skip or raise below.
        worst_term_all = max(deltas, key=deltas.get)
        delta_all = deltas[worst_term_all]

        # Record loop count and areas before/after closing, and whether
        # closing moved any of the six terms by more than the tolerance
        # (closed-vs-unclosed largest loop, same terms the comparison uses).
        raw_terms = alignment_terms(raw_best)  # noqa: F821 - supplied by the bundle
        closed_terms = alignment_terms(best)  # noqa: F821 - supplied by the bundle
        term_moves = {
            k: abs(closed_terms[k] - raw_terms[k]) for k in closed_terms
        }
        closing_tol = spec.mesh_alignment_tolerance_mm
        moved = {k: v for k, v in term_moves.items() if v > closing_tol}
        closing_rec = closed.as_dict()
        closing_rec["max_term_change_mm"] = round(max(term_moves.values()), 6)
        closing_rec["terms_changed_over_tolerance"] = sorted(moved)
        closing_stats = stats["closing"]
        closing_stats["radius_mm"] = closing_radius_mm
        closing_stats["layers_checked"] += 1
        loop_changed = closed.loops_after != closed.loops_before
        if loop_changed:
            closing_stats["loop_count_changed_layers"] += 1
        if moved:
            closing_stats["term_changed_over_tolerance_layers"] += 1
        if (loop_changed or moved) and len(closing_stats["changed_layers"]) < 40:
            closing_stats["changed_layers"].append({
                "layer": index,
                "print_z": round(float(layer.print_z), 5),
                **closing_rec,
            })
        if getattr(self, "_current_sidecar_entry", None) is not None:
            self._current_sidecar_entry["closing"] = closing_rec

        if delta_all > stats["worst_bbox_delta_mm"]:
            stats["worst_bbox_delta_mm"] = delta_all
            stats["worst_bbox_delta_layer"] = index
            stats["worst_bbox_delta_term"] = worst_term_all
        stats["worst_by_term"] = {
            k: max(v, (stats.get("worst_by_term") or {}).get(k, 0.0))
            for k, v in deltas.items()
        }

        # Diagnostic-only. All six deltas for EVERY sectioned layer, not
        # windowed to the top extremum -- ``near_top_alignment_deltas``
        # below is windowed/capped; this is a second, unconditional,
        # uncapped list, because a failure far from the mesh's own top can
        # never appear in the windowed list.
        stats["all_layer_alignment_deltas"].append({
            "layer": index,
            "print_z": round(float(layer.print_z), 5),
            "deltas_mm": {k: round(v, 5) for k, v in deltas.items()},
        })

        # Diagnostic-only. A full contour dump -- the mesh's
        # own chosen ("best") loop points, the host's own frame outline
        # points, and the exact Z values used -- for any layer that FAILS
        # this guard, or comes within 2x tolerance of failing on ANY of the
        # six terms, so the polygon difference is computable exactly
        # OFFLINE from the report alone (the host's own libslic3r slice
        # cannot be reproduced outside the host). Placed here, before every raise site
        # below, so a layer that goes on to fail still has its own dump
        # written into ``stats`` before the TopologyError propagates -- the
        # dispatcher's ``except TopologyError`` branch in ``execute()``
        # serializes ``self._mesh_alignment`` (this same ``stats`` dict)
        # as-is at that point.
        exceeds_tolerance = delta_all > spec.mesh_alignment_tolerance_mm
        near_miss = delta_all >= (spec.mesh_alignment_tolerance_mm / 2.0)
        if near_miss:
            dumps = stats["alignment_contour_dumps"]
            # The cap is soft for a layer that actually EXCEEDS tolerance:
            # that is exactly the layer this dump exists to capture, and
            # losing it to a handful of earlier, merely-close near-misses
            # would defeat the point.
            if len(dumps) < ALIGNMENT_CONTOUR_DUMP_CAP or exceeds_tolerance:  # noqa: F821 - bundle constant
                applied_z = getattr(self, "_last_applied_slice_z", None)
                dumps.append({
                    "layer": index,
                    "print_z": round(float(layer.print_z), 5),
                    "slice_z_used": (
                        None if applied_z is None
                        else round(float(applied_z), 5)
                    ),
                    "deltas_mm": {k: round(v, 5) for k, v in deltas.items()},
                    "exceeds_tolerance": exceeds_tolerance,
                    "best_loop_points_mm": [
                        [round(px, 5), round(py, 5)] for px, py in best.points
                    ],
                    "frame_pts_mm": [
                        [round(px, 5), round(py, 5)] for px, py in frame_pts
                    ],
                })

        band_mm = self._alignment_extremum_band_mm
        mesh_bounds = stats.get("mesh_bounds_after_transform")
        near_top_extremum = False
        z_hi = None
        if band_mm is not None and mesh_bounds:
            z_bounds = mesh_bounds.get("z")
            if z_bounds and len(z_bounds) == 2:
                # Print top (mesh max Z), see ``_print_top_z``.
                z_hi = self._print_top_z(z_bounds)
                p_z = float(layer.print_z)
                # PRINT-TOP extremum ONLY (whichever anatomical end is there;
                # see ``_print_top_z``) -- see the module comment above
                # ``THETA_EPSILON_DEG``: with the root at the bed,
                # ``root_solid_layers`` normally already excludes the other end.
                near_top_extremum = abs(p_z - z_hi) <= band_mm

        # Every layer within ~10 layer heights of the mesh's own top Z gets
        # all six deltas recorded, independent of skip/raise/enabled -- so a
        # single run can tell a step from a trend. Capped like ``anomalies``
        # elsewhere in this file.
        if band_mm is not None and z_hi is not None:
            if abs(float(layer.print_z) - z_hi) <= 10.0 * band_mm:
                near_top_list = stats["near_top_alignment_deltas"]
                if len(near_top_list) < 40:
                    near_top_list.append({
                        "layer": index,
                        "print_z": round(float(layer.print_z), 5),
                        "distance_to_mesh_top_mm": round(
                            float(layer.print_z) - z_hi, 5),
                        "deltas_mm": {k: round(v, 5) for k, v in deltas.items()},
                    })

        bbox_terms = ("min_x", "max_x", "min_y", "max_y")
        worst_bbox_term = max(bbox_terms, key=deltas.get)
        worst_bbox_delta = deltas[worst_bbox_term]

        # --- sensitivity-aware effective tolerance (off by default) -
        #
        # ``effective[term]`` starts equal to the base tolerance for every term
        # -- so with the field FALSE, or when no term is over base, or when the
        # rate cannot be measured/trusted, or when a measured rate's sign
        # disagrees with the observed disagreement's sign, every comparison
        # below is the plain fixed-tolerance comparison (same value compared,
        # same format-string arguments, same message).
        #
        # ``sensitivity_unmeasurable_reason`` is surfaced in the raise sites
        # below whenever sensitivity is enabled but the rate could not be
        # measured for a term that is actually over base tolerance -- so a
        # failure names WHICH mechanism it hit (unmeasurable rate, sign
        # mismatch, or plain over-tolerance) instead of only "above tolerance".
        base_tol = spec.mesh_alignment_tolerance_mm
        effective = {term: base_tol for term in deltas}
        sensitivity_unmeasurable_reason = None
        # Only measured when at least one term is already over base
        # tolerance -- see the module comment above
        # ``MESH_ALIGNMENT_SENSITIVITY_K`` ("only computing it when the
        # term is ALREADY over the base tolerance, which is the only case
        # where it matters") -- so an ordinary, comfortably-passing layer
        # never pays the two extra ``section_at`` calls.
        if spec.mesh_alignment_sensitivity_enabled and any(
            v > base_tol for v in deltas.values()
        ):
            applied_z = getattr(self, "_last_applied_slice_z", None)
            t_probe0 = time.perf_counter()
            rates = None
            reason = None
            if applied_z is None:
                reason = "this layer's own applied slice_z is unknown"
            else:
                rates, reason = self._measure_term_rates(
                    sectioner, float(applied_z), best.signed_area,
                    MESH_ALIGNMENT_SENSITIVITY_PROBE_DELTA_MM,  # noqa: F821
                    closer=lambda outer_loops: close_outer_loops(  # noqa: F821
                        frame.ops, outer_loops, closing_radius_mm,
                        skip_single_loop=True).outer,
                )
            probe_cost_ms = (time.perf_counter() - t_probe0) * 1000.0
            stats["sensitivity_probe_count"] = (
                stats.get("sensitivity_probe_count", 0) + 1)
            stats["sensitivity_probe_ms_total"] = (
                stats.get("sensitivity_probe_ms_total", 0.0) + probe_cost_ms)
            if rates is None:
                # FAIL CLOSED: unmeasurable means every term stays at
                # base_tol.
                stats["sensitivity_unmeasurable_count"] = (
                    stats.get("sensitivity_unmeasurable_count", 0) + 1)
                sensitivity_unmeasurable_reason = reason
            else:
                epsilon_mm = sectioner.epsilon_mm
                # See "THE ABSOLUTE CEILING" in the module comment above
                # ``MESH_ALIGNMENT_SENSITIVITY_K``: the resolved layer height
                # (same value the top-extremum band already resolves), falling
                # back to a fixed constant only when that itself failed to
                # resolve.
                ceiling_mm = self._alignment_extremum_band_mm
                if not ceiling_mm or ceiling_mm <= 0:
                    ceiling_mm = MESH_ALIGNMENT_SENSITIVITY_CEILING_FALLBACK_MM  # noqa: F821,E501
                adjusted = {}
                ceiling_hits = []
                for term, delta in deltas.items():
                    if delta <= base_tol:
                        continue
                    rate_signed = rates[term]
                    # THE SIGN GATE (see the module comment above
                    # ``MESH_ALIGNMENT_SENSITIVITY_K``). Widen only when the OBSERVED disagreement's sign matches
                    # the sign the mechanism itself predicts. A zero rate never
                    # widens either way (0 <= base_tol already holds, so the
                    # term stays at base_tol exactly as a sign mismatch would
                    # leave it).
                    same_sign = (
                        (signed_deltas[term] > 0) == (rate_signed > 0)
                        and rate_signed != 0
                    )
                    if not same_sign:
                        continue
                    uncapped = max(
                        base_tol,
                        MESH_ALIGNMENT_SENSITIVITY_K * abs(rate_signed) * epsilon_mm,  # noqa: F821,E501
                    )
                    eff = min(ceiling_mm, uncapped)
                    effective[term] = eff
                    if uncapped > ceiling_mm:
                        ceiling_hits.append(term)
                    if eff != base_tol:
                        adjusted[term] = round(eff, 6)
                if ceiling_hits:
                    stats["sensitivity_ceiling_hit_count"] = (
                        stats.get("sensitivity_ceiling_hit_count", 0)
                        + len(ceiling_hits))
                if adjusted:
                    # Uncapped running total -- distinct from
                    # ``sensitivity_adjustments`` below, which is a bounded, "a
                    # handful of examples" dump. The run summary suffix reads
                    # THIS counter so it stays honest past the dump's own cap.
                    stats["sensitivity_widened_term_count"] = (
                        stats.get("sensitivity_widened_term_count", 0)
                        + len(adjusted))
                dumps = stats.setdefault("sensitivity_adjustments", [])
                if len(dumps) < SENSITIVITY_ADJUSTMENT_DUMP_CAP:  # noqa: F821
                    dumps.append({
                        "layer": index,
                        "print_z": round(float(layer.print_z), 5),
                        "applied_z": round(float(applied_z), 5),
                        "epsilon_mm": epsilon_mm,
                        "ceiling_mm": ceiling_mm,
                        "ceiling_hit_terms": ceiling_hits,
                        "rates_signed_mm_per_mm": {
                            k: round(v, 4) for k, v in rates.items()
                        },
                        "signed_deltas_mm": {
                            k: round(v, 5) for k, v in signed_deltas.items()
                        },
                        "deltas_mm": {
                            k: round(v, 5) for k, v in deltas.items()
                        },
                        "effective_tolerance_mm": adjusted,
                        "probe_cost_ms": round(probe_cost_ms, 4),
                    })

        # Make an unmeasurable rate LOUD. Only non-empty when sensitivity is
        # actually enabled AND at least one over-base term's rate could not be
        # measured/trusted this layer -- appended to every raise site below so
        # a failure names WHICH mechanism it hit, not only "above tolerance".
        # Explicitly names ``tip_alignment_skip_enabled`` as the instrument for
        # the topologically-unstable-tip case (wing "(7)", layer 1126), since
        # "the loop selected at a probe plane is not plausibly the same loop"
        # is exactly that signature -- see ``_measure_term_rates``'s own reason
        # strings. The host reports ONE outer contour for this region; if even
        # after emulating its closing the mesh section still has several, the
        # loop-count note names that.
        sensitivity_note_suffix = loop_count_mismatch_note(  # noqa: F821
            closed.loops_before, closed.loops_after, closing_radius_mm)
        if (
            spec.mesh_alignment_sensitivity_enabled
            and sensitivity_unmeasurable_reason is not None
        ):
            sensitivity_note_suffix += (
                " mesh_alignment_sensitivity_enabled is on, but the local "
                "sensitivity rate could not be measured for this layer "
                "({}); falling back to the base tolerance for every term. "
                "If this is a topologically unstable tip cap, "
                "tip_alignment_skip_enabled is the instrument for that "
                "case, not this one.".format(sensitivity_unmeasurable_reason)
            )

        # The boolean decision (clauses 2-4) lives in
        # ``aircraft_core.alignment.decide_alignment`` -- a pure function,
        # unit-tested there and reused unchanged by
        # ``tools/replay_host_sections.py`` for OFFLINE replay.
        # Everything below this call is message-formatting and instance-state
        # bookkeeping specific to a live run; the decision itself (which clause
        # fires, on which term, at what delta/tolerance) is computed exactly
        # once, here.
        decision = decide_alignment(  # noqa: F821 - supplied by the bundle
            deltas, effective, near_top_extremum, spec.tip_alignment_skip_enabled
        )
        # Recorded for the host-sections sidecar (see
        # ``_record_host_section_sidecar_entry``), regardless of outcome -- a
        # passing layer's decision is as replay-relevant as a failing one's.
        if getattr(self, "_last_geometry", None) is not None:
            self._last_geometry["alignment_decision"] = dict(decision)
        if getattr(self, "_current_sidecar_entry", None) is not None:
            entry = self._current_sidecar_entry
            entry["guard_ran"] = True
            entry["deltas_mm"] = {k: round(v, 5) for k, v in deltas.items()}
            entry["effective_tolerance_mm"] = {
                k: round(v, 5) for k, v in effective.items()
            }
            entry["near_top_extremum"] = bool(near_top_extremum)
            # Rounded the SAME way ``deltas_mm``/``effective_tolerance_mm``
            # above are: a replay recomputes ``decide_alignment`` from
            # THOSE rounded values, so recording ``decision`` unrounded
            # would make an honest, float-precision-only replay agreement
            # look like a disagreement (rounded input -> rounded-consistent
            # output, compared against this run's own unrounded output).
            entry["decision"] = {
                k: (round(v, 5) if isinstance(v, float) else v)
                for k, v in decision.items()
            }

        outcome = decision["outcome"]
        if outcome == "pass":
            return None

        if outcome == "raise_bbox":
            # Clause 2: the four bbox terms gate UNCONDITIONALLY, extremum
            # or not, tip_alignment_skip_enabled or not -- a genuinely
            # wrong transform still moves the bbox regardless of where on
            # the part it lands, and this is the clause that keeps that
            # case failing closed. Gated per-term against EACH term's OWN
            # effective tolerance, not against the single raw-worst term: a
            # term with a small rate can be genuinely over base while the
            # raw-worst (high-rate) term is merely sensitivity noise, and
            # testing only the raw-worst would wrongly let the small-rate
            # term's real disagreement through.
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                index,
                layer.print_z,
                ("the model mesh cross-section disagrees with the host's own "
                 "slice by {:.5f} mm on {}, above the {:.5f} mm tolerance. The "
                 "mesh-to-slice transform is wrong, so a spar socket cut from "
                 "it would be placed from geometry that does not match what is "
                 "actually being printed.".format(
                     decision["worst_delta"], decision["worst_term"],
                     decision["effective_tolerance"],
                 )) + sensitivity_note_suffix,
            )

        if outcome == "raise_centroid_off_top":
            # Clause 4: a centroid disagreement away from the
            # mesh's own top extremum is evaluated at full strictness,
            # tip_alignment_skip_enabled or not.
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                index,
                layer.print_z,
                ("the model mesh cross-section disagrees with the host's own "
                 "slice by {:.5f} mm on {}, above the {:.5f} mm tolerance. The "
                 "mesh-to-slice transform is wrong, so a spar socket cut from "
                 "it would be placed from geometry that does not match what is "
                 "actually being printed.".format(
                     decision["worst_delta"], decision["worst_term"],
                     decision["effective_tolerance"],
                 )) + sensitivity_note_suffix,
            )

        if outcome == "raise_centroid_near_top_skip_disabled":
            # Clause 3: bbox terms all agree, only a centroid term does not,
            # and this layer sits within one layer height of the mesh's own top
            # Z -- exactly the case, mechanism unestablished (see the module
            # comment above ``THETA_EPSILON_DEG``).
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                index,
                layer.print_z,
                ("the model mesh cross-section disagrees with the host's own "
                 "slice by {:.5f} mm on {}, above the {:.5f} mm tolerance. The "
                 "bbox terms all agree (worst {:.5f} mm) -- only the centroid "
                 "disagrees, and this layer sits within {:.4f} mm of the "
                 "mesh's own top Z ({:.4f} mm), which is exactly the "
                 "in-host tip failure this guard cannot yet explain. Set "
                 "tip_alignment_skip_enabled=true in the plugin config to "
                 "leave a layer like this unmutated instead of failing the "
                 "slice.".format(
                     decision["worst_delta"], decision["worst_term"],
                     decision["effective_tolerance"], worst_bbox_delta,
                     band_mm, z_hi,
                 )) + sensitivity_note_suffix,
            )

        if outcome != "skip":
            # This is a ``raise``, not an ``assert``: an assert is a silent
            # no-op under ``python -O`` and would fall through to the
            # skip-layer code below for an outcome ``decide_alignment`` never
            # documents -- a silent fallback on the one guard that must not
            # have one. An unrecognised outcome is an internal defect (version
            # drift between this dispatch and
            # ``aircraft_core.alignment.decide_alignment``'s five documented
            # outcomes), not a case to guess at: fail the slice closed, naming
            # it, rather than falling through into "skip this layer" for a
            # decision that was never actually a skip.
            raise TopologyError(  # noqa: F821 - supplied by the bundle
                index,
                layer.print_z,
                "internal error: the mesh-alignment guard's decision "
                "function returned an unrecognised outcome {!r}. This is a "
                "defect in the guard itself, not a geometry problem with "
                "this layer; refusing rather than guessing which case it "
                "was meant to be.".format(outcome),
            )
        # Opted in: skip this layer entirely. The caller must leave every
        # region unmutated -- no slices.set(), no make_slices().
        # ``_root_layers`` is untouched here (only a mutated layer
        # increments it, elsewhere). ``_last_geometry`` IS written for THIS
        # layer, earlier in this same method (the per-layer stub set at the
        # top of the layer loop in ``_transform``, enriched here with this
        # layer's own ``host_contour_pts_mm``/``our_section_pts_mm``/
        # ``alignment_decision``). Staleness safety comes from that stub
        # always carrying ITS OWN ``layer_index``/``complete: False`` rather
        # than a prior layer's data. A skipped layer's own (honest, partial)
        # stub is harmless because a skip never produces a failure report on
        # its own; if a LATER layer goes on to raise, that later layer
        # overwrites/enriches ``_last_geometry`` with its own stub first (see
        # the top of the layer loop). ``_mesh_alignment`` (``stats``, this same
        # dict) IS mutated below (``tip_alignment_skipped_layer_count`` etc.)
        # -- that is a run-wide aggregate, not a per-layer "latest"
        # attribution field. The six deltas are also recorded above
        # (``worst_by_term``/``near_top_alignment_deltas``).
        stats["tip_alignment_skipped_layer_count"] += 1
        if stats["tip_alignment_skipped_first_z"] is None:
            stats["tip_alignment_skipped_first_z"] = round(float(layer.print_z), 4)
        skip_record = {
            "layer": index,
            "print_z": round(float(layer.print_z), 4),
            "band_mm": band_mm,
            "deltas_mm": {k: round(v, 5) for k, v in deltas.items()},
        }
        if len(stats["tip_alignment_skipped_layers"]) < 20:
            stats["tip_alignment_skipped_layers"].append(skip_record)
        # Returned to the caller regardless of the cap above (a per-run
        # aggregate list, bounded so a pathological run cannot inflate the
        # report) -- the CALLER's own LayerRecord for THIS layer must carry
        # this layer's own deltas even past the 20th skip in one run.
        return skip_record

    def _start_sidecar_entry(self, index, layer, spec):
        """A fresh per-layer sidecar entry, created at the
        same point in the layer loop as the ``_last_geometry`` stub
        (right after ``_channels_for_layer`` sets ``self._last_loops``), so
        THIS layer's mesh-section loops are captured immediately, before any
        raise later in this same iteration. ``host_regions`` starts empty and
        is filled by the per-region loop immediately below this call site;
        ``guard_ran``/``deltas_mm``/etc. are filled by
        ``_check_mesh_alignment`` if and when it runs for this layer -- a layer
        that never reaches the guard (an early raise, or a multi-island skip)
        keeps ``guard_ran: False`` and empty/None guard fields, which is itself
        informative (a contiguous layer index with an honest "the guard never
        ran here" marker), not a gap in the layer index sequence.
        """
        applied_z = getattr(self, "_last_applied_slice_z", None)
        loops = getattr(self, "_last_loops", None) or []
        return {
            "layer_index": index,
            "print_z": round(float(layer.print_z), 5),
            "slice_z": None if applied_z is None else round(float(applied_z), 5),
            "guard_ran": False,
            "host_regions": [],
            "mesh_section_loops": [
                {
                    "is_interior": bool(lp.is_interior),
                    "points_mm": [
                        [round(px, 5), round(py, 5)] for px, py in lp.points
                    ],
                    "centroid_mm": [
                        round(lp.centroid[0], 5), round(lp.centroid[1], 5)
                    ],
                    "equivalent_diameter_mm": (
                        None if lp.equivalent_diameter_mm is None
                        else round(lp.equivalent_diameter_mm, 5)
                    ),
                }
                for lp in loops
            ],
            "deltas_mm": None,
            "effective_tolerance_mm": None,
            "near_top_extremum": None,
            "tip_alignment_skip_enabled": bool(spec.tip_alignment_skip_enabled),
            "decision": None,
        }

    def _write_diagnostics(self, report):
        base = _plugin_dir()
        if not base:
            return
        try:
            out_dir = os.path.join(base, DIAGNOSTICS_DIRNAME)
            os.makedirs(out_dir, exist_ok=True)
            report_path = os.path.join(out_dir, "m1_report.json")

            # Which bundle produced this run, read from this module's own
            # on-disk header. Never
            # raises; a failed read is itself a valid, reportable outcome
            # (see ``_read_bundle_info``'s docstring), so it is computed
            # directly, not behind its own try/except.
            report["bundle"] = _read_bundle_info()

            # A run must never destroy the previous run's evidence by silently
            # overwriting the fixed report/sidecar filenames (a successful
            # run would otherwise erase an earlier FAILING run's report).
            # Each target's pre-existing file, if any,
            # is renamed aside by ``_archive_aside`` BEFORE this run's own
            # write. Its own outcome is folded into the report, matching
            # ``_handle_sidecar``'s own discipline: a failure invisible from
            # the report is a defect in itself. Isolated in its own try/except
            # -- an archiving failure must not skip the report write below any
            # more than a sidecar failure does.
            try:
                report["diagnostics_archive"] = {
                    "report": _archive_aside(report_path),
                }
            except Exception as exc:  # noqa: BLE001
                report["diagnostics_archive"] = {
                    "report": {
                        "archived": False,
                        "archived_path": None,
                        "error": "{}: {}".format(type(exc).__name__, exc),
                    },
                }

            # The sidecar is handled BEFORE the report is written, and its
            # own outcome (written or not, and why) is folded into
            # ``report["host_sections_sidecar"]`` before ``json.dump`` below --
            # so the report the user actually reads NEVER silently omits
            # whether the sidecar it may reference actually exists.
            report["host_sections_sidecar"] = self._handle_sidecar(out_dir)

            # There is no retention cap on the archives ``_archive_aside``
            # leaves behind (a cap would delete evidence), so growth is
            # instead made visible here, every run, directly in the report.
            report["diagnostics_archive"]["archive_dir"] = _archive_directory_stats(
                out_dir
            )

            with open(report_path, "w", encoding="utf-8") as handle:
                json.dump(report, handle, indent=2, sort_keys=True, default=str)
        except Exception:
            # Diagnostics must never fail a slice.
            pass

    def _handle_sidecar(self, out_dir):
        """Returns the ``host_sections_sidecar`` report field
        -- never raises: any failure here is recorded IN the returned dict
        (``error``/ ``stale_archived.error``), not propagated, since
        diagnostics must never fail a slice (matching ``_write_diagnostics``'s
        own try/except, but this method's own errors are additionally made
        VISIBLE in the report rather than only swallowed).

        Written on EVERY path that reaches ``_write_diagnostics`` --
        success, TopologyError, and the generic exception handler --
        because a failing layer's own recorded geometry is exactly what an
        offline replay of a REFUSED slice needs (a success-only sidecar
        could never replay the failing layer).

        Gated on ``self._sidecar_enabled``, opt-in and False by default (see
        aircraft_core/spec.py's ``record_host_geometry_sidecar`` for the
        measured cost that default is based on). When DISABLED, any stale
        sidecar left behind by an earlier run with the flag on (same plugin
        dir, same host-reused capability instance or not) is ARCHIVED ASIDE --
        via the same ``_archive_aside`` rename-aside treatment the write path
        below and ``_write_diagnostics`` use -- rather than deleted. Nothing
        remains at the canonical ``m1_host_sections.json.gz`` path afterward,
        so a reader can never find an old sidecar next to a fresh report and
        mistake it for this run's own geometry (the same stale-attribution
        hazard the ``_last_geometry`` handling guards against), while the
        prior run's recorded geometry survives under its stamped archive
        name.
        """
        sidecar_path = os.path.join(out_dir, "m1_host_sections.json.gz")
        run_id = getattr(self, "_run_id", None)
        info = {
            "enabled": bool(getattr(self, "_sidecar_enabled", False)),
            "written": False,
            "error": None,
            "path": sidecar_path,
            "run_id": run_id,
            # Cleared from the canonical path (renamed aside, not deleted --
            # see the docstring above). Same ``_archive_aside`` return shape
            # as "archived_previous" below: the archived/archived_path/error
            # dict.
            "stale_archived": {
                "archived": False,
                "archived_path": None,
                "error": None,
            },
            # Populated only on the write path below, before the
            # write, so an existing sidecar from a previous run is archived
            # aside rather than overwritten in place.
            "archived_previous": {
                "archived": False,
                "archived_path": None,
                "error": None,
            },
        }
        if info["enabled"]:
            info["archived_previous"] = _archive_aside(sidecar_path)
            try:
                import gzip
                payload = json.dumps(
                    {
                        "run_id": run_id,
                        "layers": self._host_sections_sidecar,
                    },
                    sort_keys=True, default=str,
                ).encode("utf-8")
                with gzip.open(sidecar_path, "wb") as handle:
                    handle.write(payload)
                info["written"] = True
            except Exception as exc:  # noqa: BLE001
                info["written"] = False
                info["error"] = "{}: {}".format(type(exc).__name__, exc)
            return info
        # Disabled this run: archive aside (never delete) a stale sidecar
        # from an earlier run on this same host-reused instance/plugin dir,
        # rather than leaving it to be mistaken for this run's own geometry.
        # ``_archive_aside`` itself never raises -- it folds any failure
        # into the returned dict -- so no try/except is needed here.
        info["stale_archived"] = _archive_aside(sidecar_path)
        return info


class CanceledSlice(Exception):
    """Raised internally when the user cancels; never surfaced as an error.

    Orca converts a raised exception at this seam into a user-visible
    SlicingError (PluginHooks.cpp:90-95), so cancellation must return a
    success/skip result instead of propagating.
    """


@orca.plugin
class AircraftStructures(orca.base):
    def register_capabilities(self):
        orca.register_capability(AircraftStructuresM1)
