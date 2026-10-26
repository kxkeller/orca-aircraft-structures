# Architecture

An orientation document for anyone who needs to build around the existing
pipeline without breaking it. For installation see `../INSTALL.md`; for what the
code does not do see `../LIMITATIONS.md`.

---

## 1. What it does, in one paragraph

An ordinary **solid** wing STL goes into OrcaSlicer. A Python slicing-pipeline
plugin intercepts each layer at `posSlice` and **cuts narrow slots** out of the
layer's cross-section polygon. Orca then generates a single Classic perimeter
and **Spiral Vase** turns the whole part into one continuous helical extrusion.
The result is a one-wall shell whose wall detours inward wherever a slot was cut,
and those detours print as internal structure. Nothing is *added* to the model;
structure is made entirely by **removing** material from a solid section.

## 2. The one rule everything else serves

> **Every body layer must remain exactly ONE closed perimeter loop.**

Spiral Vase can only trace a single continuous contour. Two contours (an island,
or an interior hole) breaks it. So every cut must:

- **enter from the exterior boundary** (never start inside the material),
- **never sever** the section into two pieces,
- **never enclose** a hole.

A layer on which a cut was made and the result violates this **fails the slice**
with a diagnostic naming the layer index, Z height and cause. A violation is never
quietly patched into something that looks valid.

There is a second, separate outcome that is *not* a failure: when structure cannot
be placed at all on a layer or in a region (the section is too thin for any web, a
candidate web does not fit, a spar socket stops fitting), the plugin can leave that
part as plain host skin and record it in the run report as an omission,
degradation or shutdown. That is deliberate, visible in the report, and it is not
the same thing as silently weakening the invariant. See `../LIMITATIONS.md` for the
list of what degrades and what refuses.

## 3. The physical model: what a "web" actually is

- A slot is a narrow notch cut from the skin inward. The perimeter dives in along
  one side, turns at the far end, and comes back out the other side.
- With the slot width set to about 0.02 mm (`gap_ratio` 0.05 of a 0.42 mm wall),
  those two adjacent wall traces are expected to **fuse into a single web**
  roughly one bead thick. The fusing gap was observed on printed PLA coupons by
  the author (0.021 mm fused; 0.105 mm and wider did not bond). It is a small
  coupon result with one material and one printer, not a general measurement.
- **A member is therefore a stack of per-layer webs.** It exists only as a Z
  sequence of chord positions.
- **Removing a slot does not leave solid material; it leaves void.** The part is
  one wall and nothing else; anything not traced by the perimeter is air.

### What is and is not expressible

| Want | Possible? | Why |
| --- | --- | --- |
| Spanwise members (stringers, shear webs) | **Yes** | a stack of per-layer slots runs along the print axis |
| Members at an angle | **Yes** | let the slot's chord position drift with Z |
| **Chordwise ribs** | **No** | would need a member lying inside one layer, i.e. a cut across the section, which severs it |
| Lightning holes in a member | **No** | a hole inside a member is a second loop |
| Enclosed cells, sandwich skins | **No** | second loop |
| More than one wall loop | **No** | Spiral Vase |

Printed root-down, **every member this architecture can make runs spanwise** (in
the print axis direction), possibly drifting sideways with height.

## 4. The pipeline

```
Orca slices the STL normally
        |
        v
posSlice  ->  capability.execute(ctx)          plugin/aircraft_structures/capability.py
        |
        |  once per run: read the placed mesh, section it, resolve the wing's
        |  global rotation and which end is the root (alignment.py, root_end.py,
        |  mesh_section.py)
        |
        |  for each layer, each region:
        |
        +-> LayerFrame.from_polygon()          aircraft_core/frame.py
        |     measures the real local geometry of THIS layer
        |
        +-> StructureGenerator.generate()      aircraft_core/generator.py
        |     decides WHERE webs go -> WebIntent list + OmittedWeb list
        |
        +-> SlotBuilder                        aircraft_core/slot.py
        |     turns each intent into an actual slot polygon
        |
        +-> ops.diff()                         the boolean cut
        |
        +-> TopologyValidator.validate()       aircraft_core/validate.py
        |     proves the RESULT still satisfies the invariant, or raises
        |
        +-> slices.set(...) ; layer.make_slices()
        |
        v
Orca generates ONE perimeter -> Spiral Vase -> G-code
```

Two things worth noting about that shape:

- **The generator decides, the validator checks.** They are deliberately separate
  and the validator re-measures from the actual cut geometry rather than trusting
  what the generator intended.
- **`make_slices()` is mandatory** after mutating region slices; it re-derives the
  layer's merged islands and bbox cache, which downstream Orca code reads.

The capability branches on `ctx.step`: it is invoked at every pipeline step and
acts only at `posSlice`.

## 5. Module map

**`aircraft_core/`** is pure geometry logic. **It must never import `orca`.** This
is what makes it unit-testable off-host.

| module | responsibility |
| --- | --- |
| `spec.py` | `StructureSpec`, `WebFamily`: the configuration model and its validation |
| `config_schema.py` | single source of truth for every config key, its type, range and meaning (`docs/configuration.md` is generated from it) |
| `frame.py` | measures the real section: local upper/lower surfaces, local thickness, worst-case-over-a-span probes, the opposing-pair predicate |
| `generator.py` | placement policy: where webs go, what gets omitted or degraded and why |
| `slot.py` | turns a placement into slot geometry (width, overshoot, end caps) |
| `validate.py` | the invariants; raises `TopologyError` with layer/Z/cause |
| `ops.py` | `GeometryOps`, the abstract geometry interface (offset, diff, bounds, contour, holes) |
| `alignment.py` | one global in-plane rotation per wing, detected from the mesh; leading-edge side; mesh-to-slice agreement decision |
| `mesh_section.py` | sections the *original* triangle mesh at a Z plane; Spiral Vase removes interior holes from the slices the plugin receives, so a CAD-modelled spar channel can only be recovered from the mesh |
| `root_end.py` | which Z end of the placed mesh is the wing root (`auto`, `bed`, `top`) |
| `spar_latch.py` | once a spar socket ends it stays ended, walking from the root toward the tip |
| `pair_ablation.py` | when an opposing-pair web would make a layer fail, drop that pair's web and re-validate |
| `pinch.py` | a layer whose bare section already splits under the safety erosion is printed as plain skin and recorded |
| `z_rule.py`, `continuity.py` | the optional Z-continuity rule (experimental) and the continuity checker (report-only) |
| `web_placement.py` | geometry-aware choice of primary web stations (used by a validation preset) |
| `diagnostics.py` | the per-run report: counts, omissions, degradations, warnings |
| `metrics.py` | timing/counting wrapper around `GeometryOps` |

**`plugin/aircraft_structures/`** is the Orca edge.

| file | responsibility |
| --- | --- |
| `capability.py` | the real plugin: implements `GeometryOps` against `orca.host`, drives the pipeline, writes the report |
| `aircraft_structures.py` | **generated** single-file bundle; built by `tools/build_plugin.py`. Never hand-edit it |

OrcaSlicer installs exactly one entry file per plugin, so `tools/build_plugin.py`
flattens `aircraft_core` plus `capability.py` into that one file, deterministically
(no timestamps, no absolute paths), and writes a content hash of the module bodies
into the header. Every report records the hash of the bundle that produced it.
The bundle runs on the Python standard library plus the host's `orca` module only.

**`tools/`**: `build_plugin.py` (bundle), `install_plugin.ps1` (update an existing
install), `run_host.ps1` (launch the host with an isolated data directory),
`config.py` (config CLI), `run_tests.py`, `gcode_spiral_check.py` (checks an
exported G-code for one loop per layer, no travel), `preflight_wing.py` (offline
replay of the real pipeline over a model), plus test-support and analysis tools.

## 6. Data structures you will touch

```python
WebFamily          # one "kind" of web, repeated across the part
  name
  role             # "skin_lattice" (default) or "primary_structure"
  web_position     # chord fraction 0..1: where it sits
  rib_pitch_mm     # marching pitch, or None for a single explicit station
  rib_pitch_chord_frac  # pitch as a fraction of the local chord instead
  web_depth_mm     # None = full depth; a number = shallow
  entry_lower      # False = cut from +Y skin, True = from -Y skin
  z_drift_mm_per_mm, z_period_mm   # optional drift with height (a lattice)
  opposing_pair, socket_flank_*    # tie pairs and webs beside a spar socket

StructureSpec      # the whole configuration
  families[]
  skin_margin_mm   # material kept between a web's far end and the far skin
  gap_ratio        # slot width as a fraction of line width (the fusing gap)
  root_solid_layers, overshoot_mm, expected_hole_count, root_end, ...

WebIntent          # one web on one layer: start/end points, width, family
OmittedWeb         # a web deliberately NOT built, with a machine-readable reason
GenerationResult   # intents + omissions + degradations
```

**The omission policy matters.** When a web cannot be placed (section too thin,
too near the skin, would collide), the generator **omits it with a recorded
reason** and the part still slices. When a cut that was made would break the
*invariant*, validation **fails closed** instead. Know which case you are in before
adding behaviour.

## 7. The two structural systems

**System A: skin structure.** Shallow webs, ~1.5 mm deep, distributed around the
skin at a fine pitch, on **both** the upper and lower surfaces (`entry_lower`).
The design intent is to cut the unsupported skin bay so the skin buckles at a
higher stress. This is what the default configuration produces. That intent comes
from an analytical argument; it has not been verified by structural testing.

**System B: primary structure.** Families with `role: "primary_structure"`:
full-depth webs, tie pairs (one web entering from each skin, offset so the flanks
fuse), webs placed beside a carbon-rod socket, and the socket itself. The socket is
taken from the model's own spar channel, detected by sectioning the mesh. These
exist and are exercised by offline tests and one in-host slice, but nothing here
is claimed to be a designed, load-bearing spar-and-rib box: ribs are not
expressible (section 3), and no printed part has been loaded.

## 8. Extension points

- **New placement behaviour** goes in `generator.py`. Return intents and omissions;
  never mutate geometry there.
- **New geometric measurement** goes in `frame.py`. Adding a method to
  `GeometryOps` fans out to several implementations (the abstract class, the real
  Orca adapter, the metrics wrapper, and test doubles).
- **New invariant** goes in `validate.py`, derived from the *actual cut geometry*,
  and should be written up as a short derivation first.
- **New config key** goes in `config_schema.py` **and** `capability.py`'s
  `DEFAULT_SPEC` (a drift-guard test enforces they agree), then regenerate
  `docs/configuration.md` with `python tools/config.py docs`.
- **Anything Orca-facing** goes in `capability.py` only. `aircraft_core` stays clean.

## 9. Testing tiers, and what each can actually prove

| tier | backend | proves |
| --- | --- | --- |
| unit | `tests/unit/stub_ops.py` and, for real booleans, `tools/shapely_ops.py` | placement logic, invariant arithmetic, exact boundary cases |
| integration | `tests/integration/fake_orca.py` | the real bundle through a fake host |
| offline replay | `tools/preflight_wing.py`, `tools/replay_host_sections.py` | what the real pipeline would do to a given model, with a shapely backend instead of Orca's |
| **in-host** | real OrcaSlicer, GUI slice | **the only thing that shows the real Clipper backend agrees** |

**The fake host is deliberately limited**, and its limits are documented in its own
module docstring. It cannot represent overlapping notches on a curved section and
has no entry from the lower skin at all, so nothing about two-sided structure can be
verified there. See `../TESTING.md` for the full taxonomy and counts.

## 10. Working conventions

- **Rebuild the bundle after ANY `aircraft_core` or `capability.py` change**
  (`python tools/build_plugin.py`) or the freshness test fails and the host runs
  stale code.
- **Launch the host only with an explicit `--datadir`** (`tools/run_host.ps1` does
  this). A bare launch attaches to your normal OrcaSlicer configuration.
- **Do not weaken an invariant to make a fixture pass.** If an invariant is wrong,
  document the counterexample and change the rule explicitly.
- Every geometry bug should get a minimized permanent fixture and a regression
  test.

## 11. The failure mode this codebase actually has

The recurring, expensive defect is **two measurements of one quantity that must
agree, where one side is updated and the other is not**: a validator and a
generator computing the same wall thickness slightly differently, a document
describing behaviour the code no longer has, a test asserting a label rather than
the geometry the label stands for.

The countermeasure used throughout is **one shared function called from both
places** (see `frame.opposing_pair_meets`, called by the generator to decide an
omission and by the validator to fail closed), plus tests that prove the two agree
rather than assuming it. Prefer extracting over reimplementing.
