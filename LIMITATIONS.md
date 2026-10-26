# Limitations

This is a research-grade plugin with narrow evidence behind it. Read this before
you spend filament. Nothing here has been structurally tested, and it is not
claimed to work on arbitrary wings.

Evidence terms used throughout (defined in `TESTING.md`): *geometry validation*
(offline), *offline replay*, *fake-host integration test*, *in-host GUI slice*,
*physical print*, *structural measurement*.

## What is established, and what is not

| Level of evidence | Status |
| --- | --- |
| Geometry validation (offline, `aircraft_core` with stub or shapely booleans) | Extensive automated suite (1506 tests) |
| Offline replay (`tools/preflight_wing.py`, shapely backend) | Available; fidelity gaps listed below |
| Fake-host integration tests (`tests/integration/fake_orca.py`) | Run the real bundle through a fake `orca`; limited geometry |
| In-host GUI slicing in the tested OrcaSlicer nightly | Done on the author's machine for a few wings, including the full model in `examples/wing_custom/` (development bundle). From this snapshot: one slice of that model's root half (Z 0 to 100 mm), 2026-10-27 |
| G-code export | One, from the snapshot run of the root half (`examples/wing_custom/snapshot_run_root_half.gcode`). None for the full wing |
| Physical printing | A small number of earlier prints from earlier builds (see Printing). **None** of the shipped example |
| Structural measurement (bending, torsion, buckling, mass-versus-strength) | **None** |

No strength, stiffness, durability or weight-saving claim is made. Filament mass
and time figures anywhere in this repository are OrcaSlicer estimates or the
plugin's own model estimates, not weighed parts.

## Geometry limits

**Structure vocabulary.** The output is one wall plus spanwise members that
detour in from the skin. Chordwise ribs, enclosed cells, lightning holes and
second wall loops are not expressible (`docs/architecture.md`, section 3).

**Orientation.** The code assumes the span runs along the print axis (the wing
stands up). It reads one global in-plane rotation from the mesh and works in a
frame with chord along X and thickness along Y. Per-layer residual rotation beyond
`max_rotation_deg` (30 degrees by default) fails the slice, and a per-station
chord-angle spread wider than the washout envelope fails it too.

**Which end is the root.** `root_end` defaults to `auto`: it compares section area
at the two ends of the placed mesh and, when present, where the spar channel sits.
If the two signals disagree the slice fails; if neither is decisive the root is
assumed to be at the bed. You can force `bed` or `top`.

**Thin, flat trailing edges.** A flat trailing edge is accepted when its face is at
most 2.5% of the local section thickness and 1% of the chord (the "thin flat"
reading; the example wing's face is about 2.1% of its thickness). A section that
looks like an airfoil but has a blunter squared-off trailing edge is **refused**
(`BluntTrailingEdgeError`), because its chord axis cannot be read and guessing
risked a confident wrong rotation. A wing whose mid-span is blunt can still be
accepted if a run of stations elsewhere along the span has a genuinely sharp
trailing edge that can be used instead. Sections with fewer than 12 hull
vertices are treated as coupons and are not rotated, so a coarsely tessellated real
wing can be misread as a coupon.

**Blunt mid-span, pitch-collapsed and thin sections.**
- If the configured pitch collapses on an airfoil-shaped section (stations land
  closer than a web footprint plus its erosion halo), the slice **fails** with a
  pitch error. A fixed chord-fraction pitch is only valid near the chord it was
  derived for (see `presets/plugin/README.md`).
- A section whose smaller in-plane extent is below the printable floor is left as
  plain skin and recorded as degraded.
- A web whose station cannot survive the safety erosion is omitted and recorded;
  a section entirely thinner than the erosion distance cannot carry a web.
- A lattice station very close to a notch wall (for example near an elevon
  cutout) can make a whole layer shut down to plain skin under default settings.
  Plain layers in the middle of a part can leave the next structured layer
  printing over air. The optional Z-continuity rule exists to avoid isolated
  blinks; it is experimental (below).

**Pinched sections.** If a layer's bare cross-section already splits under the
wall-safety erosion, with no slot to blame, that one layer is printed as plain
skin and recorded (`pinched_section`).

**Mesh top.** The last layer or two near the top of the mesh can disagree with the
host's own slice (area-weighted terms are sensitive where a tapering section
closes). That fails the slice unless `tip_alignment_skip_enabled` is set, in which
case such a layer is left unmutated. The default in the schema is off; the shipped
live preset and the example config turn it on. A region that the host has emptied
at a thin tip is treated as a tip too thin to carry structure.

**Root and tip ends.** The first `root_solid_layers` layers (5 by default, counted
from the bed) are left solid, whichever end is the anatomical root. With the Z-continuity rule on, webs start
gradually, and webs may end; the final layer of a part can be shut down. A spar socket
that stops fitting ends and stays ended (a blind pocket the rod cannot reach would
be worse than none); the rest of the part prints skin-only. This snapshot contains
no record of fitting a carbon rod into a printed socket.

**Single solid body.** One solid volume is assumed. A layer with no regions, or a
region with no sliced surface (other than the tip case above), fails the slice.
Modifier volumes, multi-material parts and models with an internal Z gap are not
supported.

**Non-airfoil features** (posts, nubs) get a simpler fallback lattice or plain
skin. This path is covered by a few fixtures only.

**Print settings are assumed, not enforced.** The structure depends on spiral
vase mode, one Classic wall and no top layers. The plugin records those settings in
its report but the plugin does not check them and nothing rejects a non-spiral preset, so a wrong
preset produces a wrong print rather than an error. Arachne is untested.

## What degrades intentionally and what refuses

| Situation | Behaviour |
| --- | --- |
| A cut that was made leaves more than one island, any hole, an unreachable slot end, or less skin than the margin | **Slice fails**, naming layer index, Z and cause |
| Mesh-to-slice transform disagrees beyond tolerance, unreadable instance placement, ambiguous leading-edge side, root-end signals disagree | **Slice fails** |
| Blunt trailing edge beyond the thin-flat bound (no usable sharp band) | **Slice fails** |
| User cancels the slice | Returns success ("slicing cancelled"); **no error** is raised |
| Web cannot be placed (thin, near skin, collision) | Web omitted, recorded in the report; the layer still slices |
| Section below printable floor; tip region emptied; pinched section; spar socket stops fitting | Layer or feature left as plain skin, recorded as degraded or shutdown |
| Opposing-pair web would break a layer | Pair ablation drops that web and re-validates; recorded |

A fail-closed slice is returned to the host as a fatal plugin error carrying the
plugin's message (how the GUI displays it was not re-verified from this snapshot).
A degrading slice looks successful: **read the report** (`INSTALL.md`).

## Behaviours that are on by default and have no physical validation

The following run by default and are not behind a switch. They are exercised by
automated tests, offline replay and a few in-host slices. **None has been
physically printed or loaded, and none has been independently reviewed beyond the
author's own tests.**

- opposing-pair ablation (drop a web of a tie pair that would make a layer fail),
- pinched-section shutdown (one plain layer),
- emulation of the host's slice-closing radius on the mesh section inside the
  mesh-to-slice guard,
- root-end `auto` resolution and the root-end-aware spar-socket latch.

**`z_continuity`.** In the code the option defaults to `false`. The shipped live
preset (`presets/plugin/live_host_config.json`) and the example config
(`examples/wing_custom/plugin_config.json`) set it to `true`. It is **experimental**:
it changes how webs start, end and ramp across layers, it has no physical
validation, and it has not been signed off as a default. The continuity checker
that accompanies it is **report-only**: it runs on every slice, whatever the flag,
and never changes or fails the slice.

## Host limits

- **Nightly only.** The plugin system is not in any stable OrcaSlicer release
  through v2.4.2. This snapshot was tested with exactly one build (see
  `README.md`). The plugin API is experimental and has changed; stable releases and
  other nightlies are untested.
- `nightly-builds` is a rolling tag; the tested build is not expected to still be served.
- **CLI slicing does not load the plugin** on the tested nightly. This was checked
  on that build only, by comparing a command-line slice's log and output with a
  no-plugin slice. Do not use the CLI for this plugin.
- The graphical picker for the process-preset plugin setting did not respond when
  its "+" button was used in the tested build; the working route is a preset file
  that carries the plugin keys (`INSTALL.md`).
- The plugin reads its stored config once, at host start. Edit, then restart.
- The plugin is inert unless a marker file sits beside the installed entry file
  (`INSTALL.md`).
- After a cancelled or refused plugin run, a later non-invalidating setting change
  can make Orca export an ordinary spiral vase without structure. A plain vase is
  also one continuous loop, so a loop check alone passes it. Always inspect the
  report and run `gcode_spiral_check.py --expect-structure`. That check can miss
  shallow or full-depth slots (`TESTING.md`); it reports FAIL on the example wing,
  so there the report and the preview are the evidence.
- Plugin runtime is long: the example's 1000-layer in-host run recorded about 2300
  seconds inside the plugin.
- numpy was not available to the plugin (it declares no dependencies), so the host's
  transform matrices were not readable; the report records an error string for
  them. The plugin reads the placement from other members instead.
- The GUI steps (first install, enabling, slicing, export) were followed from this
  snapshot once, on the example's root half only, with the tested host build and
  one printer profile.

## Testing limits

- The fake host supports a handful of polygon shapes. It cannot represent
  overlapping notches on a curved section and has no entry from the lower skin at
  all. Four rotation-related integration tests are skipped as
  "BLOCKED, not fixed", because the fake cannot do rotated polygons; that logic is
  covered at unit level only.
- The continuity checker is report-only, so no automated test or run fails because
  of a continuity violation.
- Three tests that read a real print's G-code are skipped because that G-code is
  not shipped (see `TESTING.md` for the full skip list).
- The offline replay does not reimplement everything the plugin does in-host.
  `preflight_wing.py` states its own gaps, and it cannot see the host's own slice,
  so it cannot predict the mesh-to-slice alignment guard.
- Offline booleans (shapely) and the host's Clipper are different backends; only an
  in-host slice shows them agreeing on a given layer.

## Printing limits

- No physical print of the shipped example setup exists.
- The photographed prints on the public portfolio page come from earlier
  development builds. Their exact slicer settings and plugin builds are **not
  recoverable**, so they illustrate the idea and are not a reproduction of the
  example.
- One earlier PETG print from an older build with 0.5 mm-deep webs warped by a few
  millimetres, and its webs were barely distinguishable from the surface. It is not
  included here. Sustained print quality across wings, materials and printers is
  unknown.
- The slot-gap fusing observation behind `gap_ratio` comes from small printed PLA
  coupons on one printer (0.021 mm fused; 0.105 mm and wider did not bond). Whether
  full-depth web tips bond to the far skin depends on `skin_margin_mm`; with the
  example's margin of 1.0 mm the plugin's own report classes the tip bond as
  "none" (0.16 mm air gap).

## No structural measurement

No bending, torsion, buckling or fatigue test has been run on a part made with this
plugin. Descriptions of what the structure is "for" (cutting the unsupported skin
bay, tying skins together) are design intent and analytical argument. The strength
or weight of a printed wing should be treated as unknown until you measure it.
