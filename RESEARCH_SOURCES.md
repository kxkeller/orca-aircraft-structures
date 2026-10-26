# Research sources and attribution

The sources consulted while designing and implementing this plugin: URL, licence,
and what was learned. Entries verified against live source record the verifying
commit SHA and date in the Verified column. Sources were accessed 22-23 August
2026 unless noted.

## Attribution

The underlying technique, using narrow cuts in a wing's cross-section so that a
single vase-mode perimeter is routed inward to form internal ribs and webs, builds
on approaches demonstrated by **Tom Stanton**, who used narrow cuts in CAD
geometry to generate internal wing ribs
(<https://www.youtube.com/watch?v=QJjhMan6T_E>). This plugin automates the
geometry modification during slicing instead of modelling it in CAD. Related
public vase-wing generators are listed below as prior art (R14-R17, R11).

## Source table

| ID | Source | Licence | Learned | Verified |
| --- | --- | --- | --- | --- |
| R01 | [OrcaSlicer wiki, Plugin Types](https://github.com/OrcaSlicer/OrcaSlicer/wiki/plugins_types) | AGPL-3.0 project | Slicing-pipeline capability, live graph mutation, post-processing behaviour, experimental status | wiki clone, 2026-08-23 |
| R02 | [OrcaSlicer wiki, Plugin Development](https://github.com/OrcaSlicer/OrcaSlicer/wiki/plugin_development) | AGPL-3.0 project | Plugin metadata, capability execution, configuration, host context, failure behaviour | wiki clone, 2026-08-23 |
| R03 | [src/slic3r/plugin/pluginTypes/slicingPipeline/SlicingPipelinePluginCapability.cpp](https://github.com/OrcaSlicer/OrcaSlicer/blob/main/src/slic3r/plugin/pluginTypes/slicingPipeline/SlicingPipelinePluginCapability.cpp) | AGPL-3.0 | Pipeline step enum; `psGCodePostProcess` output is not reflected in preview | blob `d2e63024` @ `550e234a`, 2026-08-23 |
| R04 | [src/slic3r/plugin/host/PluginHostSlicing.cpp](https://github.com/OrcaSlicer/OrcaSlicer/blob/main/src/slic3r/plugin/host/PluginHostSlicing.cpp) | AGPL-3.0 | Python bindings; `LayerRegion::slices` mutability at `posSlice`; extrusion tree read-only; `Layer.make_slices()` maintains derived `lslices` | blob `30e7b451` @ `550e234a`, 2026-08-23 |
| R05 | [src/slic3r/plugin/host/PluginHostGeometry.cpp](https://github.com/OrcaSlicer/OrcaSlicer/blob/main/src/slic3r/plugin/host/PluginHostGeometry.cpp) | AGPL-3.0 | `ExPolygon`/`Polygon` bindings; Orca-hosted Clipper Boolean/offset operations | blob `646dffb5` @ `550e234a`, 2026-08-23 |
| R06 | [src/libslic3r/GCode/SpiralVase.cpp](https://github.com/OrcaSlicer/OrcaSlicer/blob/main/src/libslic3r/GCode/SpiralVase.cpp) | AGPL-3.0 | Single-complete-loop expectation; cumulative-distance continuous Z | pending |
| R07 | [OrcaSlicer wiki, Wall Generator](https://github.com/OrcaSlicer/OrcaSlicer/wiki/quality_settings_wall_generator) | AGPL-3.0 project | Classic fixed line width vs Arachne dynamic width | pending |
| R08 | [OrcaSlicer wiki, Precision / slice gap closing radius](https://github.com/OrcaSlicer/OrcaSlicer/wiki/quality_settings_precision) | AGPL-3.0 project | Mesh-slicing crack closure occurs before `posSlice` | pending |
| R09 | [OrcaSlicer wiki, How to Test](https://www.orcaslicer.com/wiki/developer_reference/how_to_test) | AGPL-3.0 project | Test build flag and ctest workflow (relevant only if the host is ever built from source) | pending |
| R10 | [OrcaSlicer repository](https://github.com/OrcaSlicer/OrcaSlicer) | AGPL-3.0 | Host project and source tree | pending |
| R11 | [Deneb Systems, Vase Mode Wing](https://vmw.deneb-systems.com/) | proprietary; requirements evidence only | Public tool behaviour: one-loop goal, pitch/angle, slit, hole routing, tip and wall modes | n/a |
| R12 | [Reddit, vase mode wing generator thread](https://www.reddit.com/r/RCPlanes/comments/1vupq3e/vase_mode_wing_generator_for_3d_printed_rc_planes/) | n/a | Creator describes a per-layer 2D Boolean workflow; users report rib-gap merging/travel | n/a |
| R13 | [Deneb Systems, Terms](https://deneb-systems.com/terms/) | proprietary | Use restrictions; do not copy implementation | n/a |
| R14 | [OpenGeoDrone](https://github.com/enacuavlab/OpenGeoDrone) | GPL-3.0 | Single-contour wing, grid voids, spars, insert clearances. Descendant of R17 | 2026-09-07, commit `9c32e30bde4d00309b4ce52b9343f8ea509ff7e5` |
| R15 | Grid-Structure.scad and Grid-Void-Creator.scad (present in both R14 and R17) | GPL-3.0 | The grid is the cut set; the void creator cancels cuts to leave a thin mean-line ligament | 2026-09-07, both commits |
| R16 | Spar-Hole.scad (present in both R14 and R17) | GPL-3.0 | Spar-hole channel, side flip, clearance, retaining features; the access seam is one slice-gap wide and runs upward only | 2026-09-07, both commits |
| R17 | [Beachless/Vase-Wing](https://github.com/Beachless/Vase-Wing) | GPL-3.0 | The original vase-mode wing generator, credited by R14 as its direct inspiration | 2026-09-07, commit `3c33b1ac9747261077fbd33f7b692042e8a0c59c` |
| R18 | Hackaday coverage of 2022 vase-mode structural wings | n/a | Prior-art chronology pointer | n/a |
| R19 | [FullControl](https://github.com/FullControlXYZ/fullcontrol) | GPL-3.0 | Explicit toolpath design reference for possible future direct-path work | n/a |
| R20 | [colorFabb Compound LW-PLA datasheet](https://colorfabb.com/media/datasheets/tds/colorfabb/TDS_E%20colorFabb%20compound%20LW-PLA.pdf) | n/a | Foaming depends on temperature/speed/layer height; roughly 190-210 C activation, 210-230 C maximum | n/a |
| R21 | [U.S. Copyright Office, Computer Programs](https://www.copyright.gov/register/tx-programs.html) | n/a | Copyright covers expression, not algorithms or methods | n/a |
| R22 | [GNU GPLv3 section 13](https://www.gnu.org/licenses/gpl-3.0.html#section13) | n/a | GPLv3/AGPLv3 combination permitted | n/a |
| R23 | [GNU GPL FAQ, GPLv3/AGPLv3 compatibility](https://www.gnu.org/licenses/gpl-faq.html) | n/a | FSF explanation of the compatibility mechanism | n/a |

"Pending" means the source was read for orientation but its specific claim was not
re-verified against a pinned commit.

## Third-party code

No third-party source code is included in this repository, and no
`THIRD_PARTY_NOTICES.md` ships with it. The GPL-licensed projects (R14-R17, R19)
were read as prior art only. If code from any of them is ever reused, the exact
file and commit provenance must be recorded in the source header and in a notices
file added at that point.

**Deneb (R11, R13) is requirements evidence only.** Its public behaviour and
published discussion informed what a one-loop vase wing tool needs to do. Its
implementation was not inspected, extracted or ported, and no Deneb code is used
here.

## Verified during development

**Host availability.** The plugin system is not in any stable OrcaSlicer release.
`src/slic3r/plugin/` returns 404 at tags `v2.4.0`, `v2.4.1` and `v2.4.2`, and
exists only on `main` (first commit `ecddf3d1`, 2026-07-02, "feat: Python
Plugins"; checked through the GitHub API on 2026-08-23). `compare/v2.4.2...main`
reported `ahead_by 808, behind_by 76, diverged`: the release branch forked before
the plugin work merged. Only `main`-branch nightlies carry the plugin system; see
`README.md` and `INSTALL.md` for the exact build this snapshot was tested with.

**API pinned at `main` HEAD `550e234a37a408c7f2b8ca6dd3fe6c42f912a0eb`**
(2026-08-22T22:23:59Z), by blob SHA:

| File | Blob SHA |
| --- | --- |
| `SlicingPipelinePluginCapability.cpp` | `d2e630242dbfee49d49db357877d255a7633bb9d` |
| `PluginHostSlicing.cpp` | `30e7b45138d984f7fe89953a06f7c237d3c50fe2` |
| `PluginHostGeometry.cpp` | `646dffb5d43b9a0337a7732cde151e78da414341` |

The tested host binary's correspondence to that commit is inferred, not proven:
the binary embeds no commit string.

**Audit-hook scope** (wiki clone, 2026-08-23): only the `open` event is enforced;
`os.mkdir`, `os.remove`, `os.rename`, `os.open`, `open(path, "x")` and `shutil.*`
are unaudited. The plugin therefore derives every write path from its own
`__file__` (so it follows `--datadir`) and does not rely on the sandbox.

**ExPolygon hole bindings (R05), re-read at the pinned commit on 2026-09-01.** The
binding surface was checked directly, because a `dir()` listing cannot distinguish
a property from a bound method:

| member | binding | so it is |
| --- | --- | --- |
| `holes` | `def_property_readonly` | a **property** |
| `contour` | `def_property` | a **property** |
| `set_holes` | `def` | a **method** |
| `num_contours` | `def` | a **method** |

`holes` returns a Python list of `Polygon` references taken from the underlying
vector, and `Polygon.points` is itself a read-only property, so
`for hole in polygon.holes` and `hole.points` are both correct. This is a
source-reading result, not a behavioural one.
