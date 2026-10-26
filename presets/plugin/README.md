# Plugin configurations

Install one with `python tools/config.py use <name>`, then **restart the host** —
plugin instances are captured at load time, so a running OrcaSlicer keeps the
old values. See `INSTALL.md` for where the installed config lives and
`docs/configuration.md` for every key.

`python tools/config.py list` prints the table below, one line per file. It
reads each `## \`file.json\` — description` heading in this document, so keep that
heading shape when adding a config.

None of these configurations has been structurally tested. Every strength-related
statement below is a design intent or an analytical estimate, not a measurement.

## `live_host_config.json` — tracked mirror of the live host config

The configuration recorded for the in-host slice in `examples/wing_custom/`
(content-equal as parsed JSON to `examples/wing_custom/plugin_config.json`).
Seven families: four shallow `skin_lattice` families (0.12c pitch, 1.5 mm deep,
+/-45 degree drift, 64.6 mm Z period, anchored at `web_position` 0.35, two on each
skin), plus three `primary_structure` families (two spar-socket flank webs and one
opposing-pair tie at 0.60c). `skin_margin_mm` is 1.0.

**It sets `z_continuity` to `true`.** The Z-continuity rule is opt-in in the code
(the schema default is `false`) and is experimental: it has not been signed off as
a default and has no physical validation. Use `default_config.json` if you do not
want it.

The file exists so the configuration the author's host actually ran has a tracked
record. It is not meant as a general starting point, though
`python tools/config.py use live_host_config.json` works the same way as for any
other file.

**Keep it in sync by hand.** The installed config
(`<datadir>/orca_plugins/config.json`) is not tracked. If you edit it (directly,
or with `tools/config.py set` or `use`) and want this mirror to follow, copy the
file here in the same commit. A drift-guard test compares the two when an
installed config is present and is skipped when it is not.

## `default_config.json` — what ships

Four `skin_lattice` families, all 1.5 mm deep and at a chord-relative pitch of
0.065c: a +45 and a -45 degree diagonal on the upper skin and the same pair on the
lower skin (28 mm Z period). No full-depth members and no `primary_structure`
families. `skin_margin_mm` is 0.8608.

The 0.8608 mm margin places the far end of a *full-depth* web 0.021 mm short of
the opposite skin, the gap that fused in the author's PLA coupon tests (0.105 mm
and wider did not bond). At the 1.0 mm margin that gap is 0.160 mm. This
default contains no full-depth web, so the margin matters only if you add one.

The shallow webs cut the unsupported skin bay, which is the design intent for
raising skin buckling resistance per gram. That is an analytical argument; it has
not been measured on a printed part.

## `skin_lattice_only_config.json` — the skin lattice, and nothing else

Four `skin_lattice` families, 1.5 mm deep, at an absolute 3.9 mm pitch: a +45 and
a -45 degree diagonal on the upper surface and the same pair on the lower. No
axial family and no full-depth member. It isolates the skin system so it can be
judged on its own. Its generator behaviour is covered by the offline tests; it
has not been physically evaluated.

## `coupon_config.json` — for square test coupons

Two full-depth spars at 30% and 70% chord plus a shallow stiffener on each skin.
It is laid out for a rectangular test part. **Do not use it for a wing**: on a
cambered, thin-trailing-edge section the aft spar lands where the section is too
thin and is omitted.

## `anisogrid_lattice_config.json` — axial + diagonal skin lattice, for testing

Two full-depth spars (0.25c, 0.44c) plus, per skin, three shallow (1.5 mm)
`skin_lattice` families at 3.9 mm pitch: one axial family and a +45/-45 degree
pair anchored at the same `web_position` (0.35) with a shared 28 mm period. Every
family declares an explicit `role`.

It exists to exercise the node-merge and opposing-entry machinery that a
drift-free configuration never reaches, not to demonstrate an optimised
structure. The diagonals cross the spars and each other constantly. The offline
fake host has no entry from the lower skin at all, so the one-loop invariant for
the lower-skin families is only established by the geometry-level tests and an
in-host slice, not by the fake-host integration tests.

## `geometry_aware_primary_web_config.json` — main web chosen from geometry, for validation

Replaces hand-picked spar stations with stations selected by
`aircraft_core.web_placement.select_primary_webs`: the same generator
feasibility check every web goes through, ranked by local section thickness
(`spar_main`, 0.30c) and by "as far aft as still buildable" (`spar_aft`, 0.47c).
Same stiffeners, margin and gap as the older two-spar configurations.

On the test fixture the result is close to hand-scanned values, as expected,
since those were derived from the same feasibility check. The policy's value is
producing correct stations under a different airfoil, chord or margin, and
failing closed with a named cause when nothing qualifies. Validation only; not a
default.

## `conservative_config.json` — the previous one-sided default

Two families, both entering from the +Y skin: a full-depth `spar` at 12 mm pitch
and a shallow 1.5 mm `stiffener` at 3.9 mm pitch, `skin_margin_mm` 1.0.

Use it if you want more retained skin under every web and spars free to sit
anywhere along the chord. The cost is that the lower skin carries no structure
and every full-depth web tip stands 0.160 mm clear of the far skin, which the
author's coupon tests found does not bond.

## `wing_r150_t90_s400_w3_sized.json` — one wing, sized by the weighted sizer

**Per-wing. Not a default.** Emitted by `tools/size_wing.py --emit-config` for one
geometry: NACA 2412, 150 -> 90 mm chord, 400 mm span, 3 degrees washout
(`tests/fixtures/wing/naca2412_wing_r150_t90_s400_w3.stl`). It differs from
`default_config.json` in one value: `rib_pitch_chord_frac` 0.0500 instead of
0.0650.

The sizer emits the first candidate that verifies, so the file is checked against
every layer of that wing with the real boolean backend. The same 0.0500c fails the
tapered 60 -> 30 mm test fixture on a few layers, where merged slots leave slightly
less skin than the safety erosion requires. The lower bound on pitch is set by
quantities fixed by the nozzle (web footprint plus erosion halo), while a
chord-relative pitch scales with the part, so no single `rib_pitch_chord_frac` is
right for every wing.

## `rc_small_config.json` — park flyer, ~120 mm chord, pitch derived from size

## `rc_medium_config.json` — .25-.40 sport, ~175 mm chord, pitch derived from size

Four-family skin lattices whose pitch and Z period were derived from a simple
size model (chord implies weight by the square-cube law, weight implies skin
stress under an assumed 4 g elliptical lift, stress implies the largest skin bay
that does not buckle, and the pitch is half that bay):

| preset | reference chord | pitch | z_period |
| --- | --- | --- | --- |
| small | 120 mm | 0.2243c | 40.4 mm |
| medium | 175 mm | 0.1054c | 27.7 mm |

There is no `rc_large_config.json`. The derived chord-relative pitch for a 300 mm
chord (0.0359c) collapses to about 1.1 mm at the tip of a 60 -> 30 mm taper, below
the web footprint plus erosion halo, so the stations collide and the layer fails
closed. An automated test slices every tracked preset on the project fixtures and
caught this, so that preset was removed instead of shipped. The general lesson is
that required pitch scales roughly with 1/chord^2, so a chord *fraction* cannot
serve every size: use the preset whose reference chord is nearest your part, or
derive a figure for your chord.

What is judgement rather than physics: the aspect ratio of 6, the 4 g limit, and
the three-web floor below about 100 mm chord. The stress model is deliberately
conservative (it ignores the webs' own bending contribution and the curved nose),
and everything here is predicted, never measured. Nothing sized this way has been
printed or loaded.

## Spar holes

Set `expected_hole_count` to 1 to declare that the section carries a spar tube. The
generator then connects the hole to the exterior with a narrow scar so the single
perimeter can reach it, and the layer keeps one circuit. Left at 0 (the default),
any interior hole is still treated as a defect. In practice a CAD-modelled spar
channel is detected from the mesh and handled by the spar-socket logic; see
`docs/configuration.md` for the `spar_*` keys.

## Useful commands

```
python tools/config.py show                    # the live config, labelled
python tools/config.py check                   # validate it
python tools/config.py set skin_margin_mm 1.0  # change one value, validated
python tools/config.py docs                    # regenerate docs/configuration.md
```

Every key, its range and its meaning: `docs/configuration.md`, generated from the
schema so it cannot drift.
