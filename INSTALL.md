# Installation

The plugin runs inside a specific OrcaSlicer build. Setup has five parts, kept
separate on purpose: (a) get and launch the compatible host, (b) install the plugin
once through the Plugins window, (c) enable it and select it in the process preset,
(d) apply a plugin configuration, (e) later, update an existing install.

> **Verification status.** Parts (a) to (d) were followed from this snapshot on
> 2026-10-27 with the tested host, a fresh data directory, the Elegoo Centauri
> Carbon 0.4 profile and the example's preset and config, on the example wing's
> root half. The GUI steps were done in the GUI; the file steps (arming marker,
> preset copy, `tools/config.py use`) were done with the methods described here
> (the marker with an equivalent empty-file command).
> The plugin installed, armed and ran; its report showed 0 failed layers; the
> exported G-code passed the loop check and failed the structure-presence check,
> which cannot see this wing's slots (`TESTING.md`). Details in
> `examples/wing_custom/README.md`. Part (e) was exercised against scratch
> directories only.

All paths are relative to your clone of this repository (`<repo>`). Use
PowerShell on Windows.

## Compatibility

Tested host: **OrcaSlicer Windows x64 nightly, portable build**
(`OrcaSlicer_Windows_x64_nightly_portable.zip` from the rolling `nightly-builds`
release), asset dated 2026-08-22. See `README.md` for its exact identifiers. The
plugin system does not exist in any stable release through v2.4.2. Stable releases
and other nightlies are untested, and `nightly-builds` is a rolling tag; the tested
build is not expected to still be served. A current nightly may or may not work; the plugin API is
experimental.

## (a) Obtain and launch the compatible host, isolated

1. Obtain the tested zip. Its SHA-256 must be
   `1283a196b2e5ed14d05123645706acbe31b0665cfc671887db0e61ef5bf6595a`
   (227,528,542 bytes). Check with
   `Get-FileHash <zip> -Algorithm SHA256`. If you cannot obtain that exact file,
   you are running an untested configuration.
2. Extract it so the executable is at
   `<repo>\host\orca-nightly\OrcaSlicer\orca-slicer.exe`. The `host\` folder is
   git-ignored. Do not run an installer; the portable tree is never modified by
   these tools.
3. Launch it **only** with the helper script:

   ```
   powershell -File tools\run_host.ps1 -Fixture examples\wing_custom\wing_custom_from_host.stl
   ```

   What `tools/run_host.ps1` does, exactly:
   - works out `<repo>` as the parent of the `tools` folder and expects the
     executable at `<repo>\host\orca-nightly\OrcaSlicer\orca-slicer.exe` (it stops
     with an error if that file is missing);
   - creates `<repo>\host\data` if needed and refuses to continue if that folder
     resolves outside `<repo>`;
   - chooses a model to open: your `-Fixture <path>`, or by default the 20 x 20 x
     60 mm test coupon; `-Short` opens the 15 mm coupon, `-Airfoil` opens the
     constant-section airfoil fixture, `-NoFixture` opens an empty plate;
   - starts the executable with `--datadir=<repo>\host\data` and the model path;
   - checks whether the arming marker exists (part b) and prints whether the plugin
     is armed;
   - creates `<repo>\host\data\EBWebView` and sets the environment variable
     `WEBVIEW2_USER_DATA_FOLDER` to it for the launched process;
   - prints the process id and the `Stop-Process -Id <pid>` command to close it.

   It does not install the plugin, copy presets, or create the marker file.

Why both of those matter:
- **`--datadir`**: a bare launch of a portable OrcaSlicer can attach to your normal
  OrcaSlicer configuration. The script always passes an explicit data directory
  inside the repository, so your own profiles and settings are not touched. Never
  start `orca-slicer.exe` directly for this plugin.
- **The WebView2 folder**: OrcaSlicer's setup wizard, Plugins window and other
  local pages are WebView2 views, and WebView2 derives its user-data folder from the
  application identity, not from `--datadir`. If a normal OrcaSlicer install is also
  present, whichever starts second can fail to create its WebView2 environment and
  shows a **completely blank** wizard or Plugins window (the log shows
  `PrinterWebView::OnError ... wxWEBVIEW_NAV_ERR_OTHER`). Giving this host its own
  folder avoids that.

Keep the clone path free of spaces; the script passes its arguments to the
executable without quoting them, and that has not been tested with spaces.

The host starts with a fresh profile (first-run setup). The tested setup used the
**Elegoo Centauri Carbon 0.4 nozzle** printer profile that ships with the nightly.

## (b) First-time plugin install (Plugins window)

The plugin must be installed through the OrcaSlicer GUI. A plugin folder copied into
`orca_plugins` by hand is *discovered* (the log shows `Discovered plugin: ...`) but
never loaded or activated, and the Plugins list stays empty.

1. Start the host as in (a).
2. Main menu (not the File menu) > **Plugins** > the arrow beside **Browse plugins**
   > **Install local plugin**.
3. Select `<repo>\plugin\aircraft_structures\aircraft_structures.py`.
4. Tick **Activate** on both the plugin row and the capability row.

The dialog installs into a folder named after the **file, extension included**:
`<repo>\host\data\orca_plugins\aircraft_structures.py\`. It contains the plugin file
and `.install_state.json`.

5. **Create the arming marker**: an empty file named `m1_enabled` in that folder:

   ```
   New-Item -ItemType File <repo>\host\data\orca_plugins\aircraft_structures.py\m1_enabled
   ```

   Without it the plugin is inert on purpose: at `posSlice` it returns "aircraft
   structures not armed (m1_enabled absent); slice untouched" and changes nothing.
   Delete the file to disarm. The marker keeps the checked-in and installed plugin
   files byte-identical.
6. Restart the host. Plugin instances are captured at load time.

## (c) Enable it and select it in the process preset

**Activation requirements.** The plugin changes a slice only when all of these hold
(the first three are the host's, the fourth is the plugin's own):

1. It is installed via the Plugins window and activated (`.install_state.json` shows
   `enabled` true and the capability `true`). This is the only one of the four that
   is visible in the Plugins window.
2. The active **process (print) preset** lists the capability *name* in
   `slicing_pipeline_plugin`. The host checks this once per slice; empty means no
   pipeline step runs at all.
3. The preset's dynamic **`plugins`** key carries the full reference
   `plugin_key;cloud_uuid;capability_name`. The uuid is empty for a local plugin:
   `aircraft_structures;;Aircraft Structures`.
4. The arming marker `m1_enabled` exists (part b).

**"Visible" or "enabled" in the Plugins window is not sufficient evidence that the
plugin ran.** A plugin can be loaded, enabled and shown, and still never execute.

The GUI picker for (2) and (3) is Print Settings > Others > "Slicing Pipeline
Plugin" (advanced mode only), but its "+" button did not respond in the tested
build. The working route is a preset file that already carries both keys:

1. Copy `presets\process\M1 Coupon 0.20mm @Elegoo CC 0.4.json` into
   `<repo>\host\data\user\default\process\`. (For the shipped example use
   `examples\wing_custom\M1 Coupon 0.20mm @Elegoo CC 0.4.json`; same name, see
   that folder's README.) The author's installed copy also had a small `.info`
   sidecar holding only sync metadata; whether Orca needs one for a hand-copied
   preset was not tested.
2. Restart the host, choose a 0.4 mm Elegoo Centauri Carbon printer, and select the
   process preset **M1 Coupon 0.20mm @Elegoo CC 0.4**.

The preset's relevant keys are `slicing_pipeline_plugin: ["Aircraft Structures"]`,
`plugins: ["aircraft_structures;;Aircraft Structures"]`, `spiral_mode: 1`,
`wall_loops: 1`, `top_shell_layers: 0`, `sparse_infill_density: 0%`, and it
inherits `0.20mm Standard @Elegoo CC 0.4 nozzle`. For another printer, copy those
keys into a preset inheriting from yours. The plugin records these settings but
does not check them, so a preset without spiral mode simply gives a normal print.

## (d) Apply the plugin configuration

The plugin reads `<repo>\host\data\orca_plugins\config.json` **once, at host start**.
Install it after part (b) and restart the host after any change.

```
python tools\config.py list                      # tracked configs and one-line descriptions
python tools\config.py use default_config.json   # validate, then write config.json
python tools\config.py show                      # print the live config, labelled
python tools\config.py check                     # validate the live config
python tools\config.py set skin_margin_mm 1.0    # change one validated value
```

- `use` validates a file from `presets\plugin\` before writing it, and refuses to
  write outside the repository. `--datadir <dir>` (given before the subcommand)
  selects another data directory inside the repo; the default is `host\data`.
- The file format is `{"config": [{"capability": "Aircraft Structures",
  "cap_config": {...}, ...}]}`. Every key is described in `docs/configuration.md`
  (generated from the schema; unknown keys are errors in this CLI).
- The shipped live preset and the example config set `z_continuity` to `true`,
  which is **experimental** and off by default in the code. Use
  `default_config.json` if you do not want it. See `LIMITATIONS.md`.
- If the host's in-memory config differs from the file on disk (you edited it while
  Orca was running), the run report lists the difference under `config_drift`.

## (e) Update an existing install

After you change the source and rebuild the bundle (`python tools\build_plugin.py`),
refresh the installed copy:

```
powershell -File tools\install_plugin.ps1
```

- It copies `plugin\aircraft_structures\aircraft_structures.py` over the **existing**
  installed copy and prints the SHA-256 of source and destination; it fails if they
  differ.
- Default target: `<repo>\host\data`. Override with `-DataDir <dir>` (and `-RepoRoot
  <dir>`). It never defaults to `%APPDATA%` or `%LOCALAPPDATA%`, and it refuses a
  destination outside the repository root.
- It does **not** perform a first install. If `...\orca_plugins\aircraft_structures.py\`
  or its `.install_state.json` is missing, it stops with an error and creates
  nothing. Do part (b) first.
- Restart the host afterwards.

Verified against scratch directories only (empty or absent target: refused; fake
existing install: updated and hashes equal), not against a real host install.

## Confirm the plugin actually ran

After slicing in the GUI, check, in this order:

1. **The report.** The plugin writes
   `<repo>\host\data\orca_plugins\aircraft_structures.py\m1\m1_report.json` (and a
   sidecar `m1_host_sections.json.gz`) at the end of every armed run, including a
   failed run. The previous report is renamed to `m1_report.<UTC time>Z.json`
   before being replaced, so a missing or old-timestamped `m1_report.json` after you
   slice means the plugin did not run, or was not armed. In the new report look at:
   - `layers_seen` (greater than 0), `mutated_count`, `failed_count`,
     `degraded_layer_count`, `shutdown_count`;
   - `bundle.content_hash`, which must equal the "Bundle content hash" line in the
     header of the installed plugin file;
   - `config`, the resolved Orca settings (check `spiral_mode`, `wall_generator`,
     `wall_loops`), and `config_drift`.
2. **The preview.** Slots should be visible as inward detours of the single wall.
3. **The host log.** The host loads the plugin through its plugin loader, which logs
   lines such as `Discovered plugin: Aircraft Structures` and `[plugin_loader::load]`.
   When run from the GUI the host writes logs under `<repo>\host\data\log\`.
   The plugin's one-line summary also appears there, for example (root-half run of
   the example) `Slicing pipeline plugin 'Aircraft Structures' [posSlice]: aircraft
   structures: 500 layers seen, 495 slotted, 5 left solid, ...`, followed by any
   warnings.
4. **The exported G-code**, with
   `python tools\gcode_spiral_check.py <file.gcode> --expect-structure`. The first
   check (one continuous loop per layer) applies to any model. The
   structure-presence check can miss shallow or full-depth slots and reports FAIL
   on the example wing although slots are present (see `TESTING.md`). Where it
   fails, rely on the report and the preview.

## Command-line slicing does not load the plugin

On the tested nightly, `orca-slicer.exe` run from the command line slices
normally but never initialises the plugin subsystem, so a CLI slice is an ordinary
slice. This was established by reading a full debug-level log of a CLI slice, which
contained none of the plugin-loader strings that exist in the binary, and by
comparing the CLI output with a no-plugin slice. It was checked on that build only,
and the evidence files are not part of this snapshot. Use the GUI.
A re-check from this snapshot, with the plugin and its config seeded in a scratch data
directory, exited 0, but `gcode_spiral_check.py --expect-structure` found 0 of 994
body layers structured and no plugin report was written: the result is a plain vase.
