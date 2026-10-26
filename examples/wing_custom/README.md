# Example: `wing_custom` (tapered wing, thin flat trailing edge)

A solid, untwisted, tapered wing half and the exact plugin configuration and
process preset that were used for one in-host OrcaSlicer slice of it.

**Status, stated plainly**

| Level | Status for this example |
| --- | --- |
| In-host GUI slice (development bundle, 2026-10-16) | Done once, recorded below as a summary |
| G-code export | None for this full-wing run; one from the snapshot run of the root half (below) |
| Physical print of this setup | **None** |
| OrcaSlicer time/mass estimates for this model | **None recorded** |
| Screenshot from that run | **None kept** |
| Re-run from this snapshot with the comment-cleaned bundle | Done 2026-10-27 on the **root half only** (model cut at Z = 100 mm); see "Snapshot run" |

The in-host numbers describe the **development bundle** as it was on 2026-10-16.
The bundle in this snapshot has had its comments rewritten and is therefore a
different file; see "Bundle bridge".

## Files

| File | What it is |
| --- | --- |
| `wing_custom_from_host.stl` | The model: binary STL, 111,332 triangles, SHA-256 `2dba0b95d8206fda60c5ca7acbc56c47b8573806fb35bb88ef6586ddf897a350` (5.6 MB) |
| `plugin_config.json` | The plugin config in effect for the run, byte-identical to the recorded file, SHA-256 `afadd218a903b3934618c8f33f1f129c88f689427027f9743bce7d6a207c5bad` |
| `M1 Coupon 0.20mm @Elegoo CC 0.4.json` | The process preset in effect, as recorded |
| `inhost_report_summary.json` | Selected fields copied from the run's `m1_report.json` (the 5 MB full report is not shipped) |

`plugin_config.json` is identical, as parsed JSON, to `presets/plugin/live_host_config.json`,
so `python tools/config.py use live_host_config.json` installs the same settings.
**It sets `z_continuity` to `true`, an experimental option that is off by default in
the code** (`LIMITATIONS.md`). That is how the run was made, so it is shipped as
recorded.

## The model

- Extruded taper with no twist: span 200 mm along Z, root (the larger end) at
  Z = 0, chord about 117 mm at the root to 60 mm at the tip, maximum thickness about
  7.0 to 3.6 mm. Bounding box about 167.4 x 15.8 x 200 mm.
- Trailing edge: a thin flat cut, about 0.112 mm across at mid-span (0.13% of chord,
  2.1% of section thickness), accepted by the plugin's thin-flat reading.
- Leading edge on the -X side. The model has **no spar channel**, so no spar
  socket is exercised.
- Origin: a wing created by the repository author in CAD (the STEP file is not
  included). This STL was extracted from the 3MF object that the host built from
  that STEP, already stood up and centred (root on Z = 0, XY centred). Its header is
  the plain text `wing_custom from host 3mf obj3, transformed` and contains no other
  data.

## Print orientation

Root face on the bed, span vertical (+Z), as shipped. Load the STL **without any
rotation**.

In the recorded run the host loaded the original STEP, and the object instance
carried a rotation of -90 degrees about X (scale 1, no mirror) to stand it up. The
STEP contains two wing halves; this STL was extracted from a placement that used
+90 degrees, whereas the 2026-10-16 run most likely used the mirror-image half. The
two meshes were not compared by hash. The section magnitudes, layer counts and
outcome were the same in the in-host run and in an offline replay of this STL, so
expect your slice of this STL to match the numbers below closely, but do not expect
it to be proven identical.

## Printer, material and settings

- Printer profile: Elegoo Centauri Carbon, 0.4 mm nozzle.
- Process preset: `M1 Coupon 0.20mm @Elegoo CC 0.4` (the name is historical; it
  is a general spiral-vase wing preset). It inherits
  `0.20mm Standard @Elegoo CC 0.4 nozzle` and overrides: 0.20 mm layers, spiral
  mode on, 1 wall loop, 0 top shell layers, 5 bottom shell layers, 0% sparse infill,
  outer wall 40 mm/s at 1500 mm/s^2, plus the two plugin keys
  (`slicing_pipeline_plugin`, `plugins`). The preset tracked at
  `presets/process/` additionally spells out `wall_generator: classic`,
  `outer_wall_line_width: 0.42` and `enable_support: 0`, the values the recorded
  run resolved from the inherited profile.
- Resolved by the host in the run: line width 0.42 mm, classic wall generator,
  `slice_closing_radius` 0.049 mm, supports off.
- **Filament and material: not recorded for this run.**
- Plugin settings: `plugin_config.json` (7 families: four skin-lattice families at
  0.12c pitch, 1.5 mm deep, plus two spar-flank webs and a 0.60c tie pair;
  `skin_margin_mm` 1.0, `gap_ratio` 0.05, 5 solid root layers).

## Host and bundle identifiers

| Item | Value |
| --- | --- |
| Host | OrcaSlicer Windows x64 nightly portable, asset dated 2026-08-22; zip SHA-256 `1283a196...6595a` (full value in `README.md`); the host reports itself as `2.5.0-dev` |
| Plugin bundle sha256 (development bundle, 2026-10-16) | `0ca6d9b770fa639204e3ea45006a3c6a0bcd880bf1c00fb3de54aaa18258b129` |
| Bundle content hash in the run report (`bundle.content_hash`) | `90c18f5094ac97662898ae2cac3e58349b64b3f13eea1ec199abe2ab584e0640` |

The content hash covers the imports and module bodies of the bundle, not its
header, so it is the identifier that a report carries.

### Bundle bridge

The in-host run above used the development bundle (sha256 `0ca6d9b7...`, content
hash `90c18f50...`) built from development commit `d04b47e`. This repository's
`plugin/aircraft_structures/aircraft_structures.py` is a different file:
sha256 `fb5825687dc81154fa674d0099cf6b93a1d95c3ebb70b6ddc7c35e1b5acc2b8b`, content hash
`54ea6a33ac9f29328fa09a7bb3614b31501089c00cb57e3af3df30ce21ec57c9`. The two differ
only in comments, docstrings and 52 enumerated message and description string
literals; a comparison of the parsed syntax trees (docstrings removed) shows
identical code structure, and with string values blanked the trees are equal.
The full-wing in-host run has **not** been repeated with this repository's bundle.
The root half was run with it on 2026-10-27 (content hash `54ea6a33…`); see
"Snapshot run".

## In-host report: slice of 2026-10-16, development bundle

Source: `inhost_report_summary.json`. This was an in-host GUI slice only; nothing
was exported or printed.

| Quantity | Value |
| --- | --- |
| Layers seen / mutated / failed | 1000 / 995 / 0 |
| Shutdown / degraded / pinched layers | 0 / 0 / 0 |
| Layers left solid at the root | 5 |
| Chord-axis source | `thin_te_flat` (theta about 1.5e-07 degrees; no rotation applied) |
| Root end | `auto`, decided from section area: root at the bed |
| Mesh-to-slice guard | 995 layers sectioned, 0 failed, worst bounding-box/centroid disagreement 0.0065 mm (centroid_x, layer 104) against a 0.02 mm tolerance |
| Continuity checker (report-only) | 1000 layers checked, 0 violations. It cannot fail a slice |
| Slots cut | 25,227 in total (1,990 on the primary-structure families) |
| Web omissions | 3,293 opposing-entry and 678 role-yield omissions among skin-lattice candidates; recorded, layers still sliced |
| Plugin run time | about 2,306 s |
| Reported tip-bond class for full-depth webs | `none` (0.16 mm air gap at `skin_margin_mm` 1.0) |

The summary also records the instance placement and the thin-flat measurements. The
checker line is a self-check by the same code that generated the structure, and
"0 failed" means the plugin's own invariants held in the host's geometry; neither
says anything about how the part would print or perform.

## Reproduce

1. Set up the host and plugin as in `INSTALL.md`, parts (a) to (c), including the
   arming marker. Use the exact host build in `README.md` for a like-for-like
   comparison.
2. Install the config: `python tools/config.py use live_host_config.json` (or copy
   `plugin_config.json` to `<repo>\host\data\orca_plugins\config.json`), then restart
   the host.
3. Copy `M1 Coupon 0.20mm @Elegoo CC 0.4.json` from this folder to
   `<repo>\host\data\user\default\process\`.
4. Launch with the model:
   `powershell -File tools\run_host.ps1 -Fixture examples\wing_custom\wing_custom_from_host.stl`.
   Choose the Elegoo Centauri Carbon 0.4 profile and the preset above, leave the
   model un-rotated, and slice. The plugin reported about 38 minutes for this wing in the recorded run.
5. Confirm the plugin ran (`INSTALL.md`, "Confirm the plugin actually ran"):
   `m1_report.json` should report 1000 layers seen and 995 mutated, with 0 failed.
6. Export G-code and run
   `python tools\gcode_spiral_check.py <file.gcode> --expect-structure`. Expect the
   loop check to pass and the structure-presence check to report FAIL: that check
   cannot see this wing's slots (`TESTING.md`).
7. Optional, no GUI needed: an offline replay of the same model and config,
   `python tools\preflight_wing.py examples\wing_custom\wing_custom_from_host.stl --config examples\wing_custom\plugin_config.json --up-axis z`
   (pass `--up-axis z`; the automatic choice picks the wrong axis for this
   wing). It uses a shapely backend and cannot see the host's own slice.

**Offline replay result (run while preparing this documentation, not part of the
in-host record).** The command in step 7, with `plugin_config.json` and this STL,
reported: 1000 layers seen, 995 mutated, 0 failed, 0 shutdown, 0 degraded; chord-axis
source `thin_te_flat`; root at the low end (bed) decided from section area; 1,952
node merges (the in-host run recorded 1,917; the offline tool uses a different
boolean backend and does not reimplement every in-host behaviour). Status line:
`CLEAN_NO_GUARANTEE`, exit code 0, about 989 s of sweep time on the machine used.
That agrees with the in-host layer counts above, which is all it can show.

## Snapshot run (root half, 2026-10-27)

A second in-host slice, made by following `INSTALL.md` from this snapshot: the
tested host build in a fresh data directory, a first install through the Plugins
window, the Elegoo Centauri Carbon 0.4 profile and Generic PLA, this folder's
preset and `plugin_config.json`. The author did the GUI steps; the arming marker
(an equivalent empty-file command), the preset copy and `python tools/config.py use live_host_config.json` were done
for the author with the methods in `INSTALL.md`. Before slicing, the model was cut in OrcaSlicer at
Z = 100 mm, and only the root half (Z 0 to 100 mm) was sliced. The figures are
therefore not comparable with the full-wing run above.

Files: `snapshot_run_summary.json` (selected report fields) and
`snapshot_run_root_half.gcode` (the exported G-code, SHA-256
`9f44fdc1b260f86ed17ebc079588ae3eb87fbca9899a465e49e45d6bdb3c7d1d`). It is software
output and has not been printed.

| Item | Result |
| --- | --- |
| Bundle content hash in the report | `54ea6a33…`, equal to this snapshot's bundle header |
| Config drift | none |
| Plugin outcome | 500 layers seen, 495 mutated, 5 root layers left solid, 0 failed, 0 shutdown, 0 degraded |
| Slots cut | 13,428 in total, 990 of them full-depth |
| Plugin warnings | full-depth webs end 0.16 mm short of the far skin (bond class `none`); 153 node merges cut a deeper web to a shallower one's depth (worst ratio 0.13 at Z = 91.4 mm) |
| Plugin run time | about 853 s |
| `gcode_spiral_check.py` loop check | PASS: every body layer is one continuous loop; 0 body travels, 0 retractions, 0 Z hops |
| `gcode_spiral_check.py --expect-structure` | FAIL: no point beyond the 3.0 mm threshold on any of the 495 body layers (largest 1.64 mm); the check cannot see this wing's slots (`TESTING.md`) |
| Preview | the author reported that the wing looked good; the slot detours were not separately confirmed |
| OrcaSlicer estimates (not measured) | 1 h 16 min, 13.61 g |

The worst skin bay (29.3 mm at Z = 57.0 mm, layer 284) matches the full-wing run's.
No G-code exists for the full wing.
