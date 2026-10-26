# Testing

## Running the suite

```
python -m venv venv
venv\Scripts\python -m pip install -r requirements-dev.txt
venv\Scripts\python tools\run_tests.py          # add -v for per-test output
```

- **Python 3.11** (the suite was verified on CPython 3.11.9, Windows;
  `requirements-dev.txt` pins numpy 2.4.6, shapely 2.2.0 and scipy 1.17.1).
- `tools/run_tests.py` runs **both** suites, `tests/unit` and `tests/integration`.
  They cannot share one discovery root (the unit tests import each other as
  siblings, the integration tests are a package), so plain `unittest discover tests`
  would silently run only about half. Always use the runner.
- **Install shapely and scipy.** numpy is required (the fake host imports it at
  module level; without it collection fails). shapely and scipy are optional at
  import time, but about a hundred tests skip without them, so a run without them
  looks green and is not a meaningful pass.
- A full run takes roughly 5 to 8 minutes.
- Some tests build the plugin in a shared temporary directory, so the plugin looks
  for an optional `config.json` one level up, in `%TEMP%`. In a clean environment
  that file is absent and the lookup is harmless: a missing or unreadable file just
  means no drift warning is added. A stray `%TEMP%\config.json` could be read and
  could change those tests' output, so remove one before running the suite.
- The plugin bundle itself needs only the standard library and the host's `orca`
  module; these dependencies are for testing and the offline tools.

### Last recorded result

On the snapshot's source tree, in a fresh Python 3.11.9 virtual environment built
from `requirements-dev.txt`:

| suite | tests | result |
| --- | --- | --- |
| unit | 1212 | OK, 5 skipped |
| integration | 294 | OK, 4 skipped |
| **total** | **1506** | **0 failures, 0 errors, 9 skipped** |

The nine skips, with reasons:

| skips | reason |
| --- | --- |
| 1 | the drift guard comparing the tracked live preset with an installed host config is inert when no host config is present |
| 1 | the smoke test that slices with the installed host config skips when it is absent (the tracked presets are sliced by a separate test that always runs) |
| 3 | tests of real print G-code skip because that reference G-code is not shipped |
| 4 | rotation-detection integration tests, marked "BLOCKED, not fixed": the fake host cannot represent rotated polygons |

Counts will drift as the code changes; re-run rather than trusting this table. If
you change anything under `aircraft_core/` or `plugin/aircraft_structures/capability.py`,
rebuild the bundle first (`python tools/build_plugin.py`); a freshness test fails
when the checked-in bundle does not match its sources.

## Evidence taxonomy

The word "tested" is ambiguous in this project, so claims are labelled by the
kind of evidence behind them.

| Level | What it is | What it can show | What it cannot show |
| --- | --- | --- | --- |
| **Geometry validation (offline)** | `aircraft_core` run with a stub backend (`tests/unit/stub_ops.py`, exact arithmetic on simple shapes) or a shapely backend (`tools/shapely_ops.py`) | placement arithmetic, invariant arithmetic, boundary cases, one-loop and no-sever logic on real polygons | that Orca's Clipper behaves the same; that the perimeter generator or Spiral Vase behaves as assumed |
| **Offline replay** | `tools/preflight_wing.py` (mesh to layers to the real pipeline), `tools/replay_host_sections.py` (re-decides a recorded mesh-alignment guard from a saved sidecar) | what the pipeline would do to a given model: refusals, degradations, counts | the host's own slice, so the mesh-to-slice alignment guard is invisible to it; the spar-socket latch and a few other behaviours are only partly reproduced (the tool says so) |
| **Fake-host integration test** | the real generated bundle executed against `tests/integration/fake_orca.py` | the capability's control flow: arming, step gating, layer walk, root-layer skip, config resolution, slot generation, validation, failure paths, report writing | real geometry kernel behaviour (see below) |
| **In-host GUI slice** | a slice in the actual OrcaSlicer nightly with the plugin installed, enabled and selected | that the plugin ran against the real host, and what the real backend produced (plugin report, preview) | anything about the plastic; also it is not automated, so it is not part of the test suite |
| **Physical print** | a printed part | that something printed | strength, unless measured |
| **Structural measurement** | loading or weighing a printed part | strength, stiffness, mass | **none exists for this project** |

### What the fake host does and does not cover

`tests/integration/fake_orca.py` fakes just enough of `orca.slicing`, `orca.host`,
`orca.ExecutionResult`, the plugin decorators and the layer/region/surface objects
for the bundle to run its own control flow. It is not a reimplementation of
Orca's Clipper-backed geometry. Its geometry layer supports a small set of shapes:
axis-aligned rectangles, a rectangle with one notch cut from an edge, an arbitrary
simple polygon, and a polygon minus one or more non-overlapping rectangular
notches. Anything else raises `NotImplementedError`.

Known divergences from real Orca, which the module's docstring lists in full:
- validity is checked only as "area greater than zero";
- it cannot produce an interior hole through a boolean;
- `Layer.make_slices()` only counts calls and does not re-derive merged islands;
- `Surface` construction is a faithful copy, whereas the real binding resets some
  fields;
- the scaling factor is fixed;
- no cut from the lower skin, no overlapping notches on a curved section, and no
  rotated polygons (hence the four skipped tests above).

A pass here means the plugin's control flow works against these assumptions. It is
not evidence that a real slice works.

## Offline replay tools

All run with the same Python environment; none imports `orca`.

```
python tools/preflight_wing.py <model.stl> --config <config.json> --up-axis z [--json out.json]
```
Reports four tiers: model-level checks (manifold, bounds), orientation (chord-axis
detection as the plugin does it), per-layer refuse/degrade/clean, and a
standing note that the mesh-to-slice guard cannot be checked offline. The default
`--up-axis auto` may pick the wrong axis on some wings (it can refuse a model
whose span is not the longest bounding-box axis); pass `--up-axis` explicitly.
Exit codes: 0 clean, 1 refusals, 2 tool-level refusal, 3 stand-up not resolved,
4 structure lost but no refusals. Runs take minutes per model (about 17 minutes for the 1000-layer example wing).

```
python tools/replay_host_sections.py --report <m1_report.json>
```
Re-runs the pure mesh-alignment decision against a saved
`m1_host_sections.json.gz` sidecar and reports whether it reproduces the in-host
decision.

This tool needs data that is not in the repository: an `m1_report.json` and its
`m1_host_sections.json.gz` sidecar, both written by an in-host slice (see
INSTALL.md, "Confirm the plugin ran"; the sidecar is written when
`record_host_geometry_sidecar` is true). It was not run for this snapshot.

### Checking exported G-code

```
python tools/gcode_spiral_check.py <file.gcode> [--root N] [--json out.json] [--expect-structure]
```
- Without flags it counts, per layer, extrusions, travels, retractions and Z hops,
  and checks that each body layer is one continuous loop with no travel. The first
  `--root` layers (default 5) are the solid base and are excluded from the verdict;
  the final layer is excluded unless `--include-last`.
- `--expect-structure` adds a second check, because a plain Spiral Vase is also one
  continuous loop and passes the first check trivially. It measures how far each
  layer's toolpath dives inward from that layer's own convex hull; a structured
  print has points several millimetres inside, a plain vase has about zero. It
  proves structure is **present** in the G-code. It does not prove the structure is
  correct, bonded, or matches your config.
- **It can miss shallow or full-depth slots.** A layer counts as structured only
  when at least 5% of its path points (the G-code move end points) lie more than
  3.0 mm inside the layer's convex hull. A slot's points sit at its entry, on the
  hull, and at its tip: the slot depth in from the skin it enters, or next to the
  far skin for a full-depth web, which is again near the hull. Half the section
  thickness is an upper bound on any point's inward distance, but slot depth
  usually limits it first. On the example wing's root half (sections 5.3 to 7.1 mm
  thick; skin-lattice slots 1.5 mm deep), the largest per-layer inward distance
  over all 495 body layers was 1.64 mm, including on the 307 layers thicker than
  6 mm. The check therefore reports FAIL there although the plugin reports slots on
  495 layers; see `examples/wing_custom/README.md`.
- It reads the file only. It never writes to a printer.

Its thresholds were calibrated against one real exported print and a synthetic
plain-vase control derived from it; neither is shipped here, so the three tests
that depend on that file skip.

## What has and has not been run for this snapshot

- The automated suite: run, results above.
- The deterministic bundle build (twice, identical bytes) and the bundle freshness
  test: run.
- The OrcaSlicer GUI path (installation, activation, in-host slice, preview,
  G-code export): followed once from this snapshot on 2026-10-27, on the example
  wing's root half. Results and the exported G-code are in `examples/wing_custom/`.
  The full-wing in-host run there was made with the development bundle.
