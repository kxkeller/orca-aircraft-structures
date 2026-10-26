"""Configure the aircraft_structures plugin without hand-editing JSON.

A small CLI over the stored capability config: it handles every key the schema
in aircraft_core/config_schema.py knows about, and every write is validated
first and refused on error rather than silently applied.

Subcommands:

    python tools/config.py list
        Show the tracked configs in presets/plugin/, with the one-line
        description read from presets/plugin/README.md's own headings.

    python tools/config.py show
        Print the live config at host/data/orca_plugins/config.json,
        labelled, flagging anything that differs from the tracked
        default_config.json.

    python tools/config.py use <name>
        Validate a tracked config (by filename, from `list`) and copy it
        into place. Refuses on a validation error or an out-of-repo
        destination; never writes silently.

    python tools/config.py check
        Validate the live config and report problems, actionably.

    python tools/config.py set <key> <value>
        Change one value in the live config, preserving everything else.
        <key> may be dotted to reach into a family by name:
        `families.spar.rib_pitch_mm 10.0`. <value> is parsed as JSON when
        possible (so `null`, `true`, `12.0` work as expected), otherwise
        used as a literal string. The whole resulting config is validated
        before anything is written; a bad value is refused, not applied.

    python tools/config.py docs
        Render docs/configuration.md from the schema (aircraft_core/
        config_schema.py), so the documentation cannot drift from the code.

Every subcommand accepts --datadir (default: host/data, the same default as
tools/install_plugin.ps1) to locate the live config, and
--wall (default: aircraft_core.config_schema.DEFAULT_WALL_MM, the shipped
default wall) for the
skin_margin_mm > 2 * safety_offset_mm cross-field check, which needs a
resolved outer wall line width that only a real host slice can supply.

After `use`/`set`, RESTART the host before slicing -- plugin instances are
captured at load time.
"""

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from aircraft_core.config_schema import (  # noqa: E402
    DEFAULT_WALL_MM,
    FAMILY_FIELDS,
    SPEC_DEFAULTS,
    SPEC_FIELDS,
    ConfigSchemaError,
    assert_valid_config,
    unknown_family_key_message,
    unknown_top_level_key_message,
)
from aircraft_core.spec import SpecError  # noqa: E402

PRESETS_DIR = REPO_ROOT / "presets" / "plugin"
PRESETS_README = PRESETS_DIR / "README.md"
CAPABILITY = "Aircraft Structures"


# --- shared helpers ----------------------------------------------------------


def _live_config_path(datadir: Path) -> Path:
    return datadir / "orca_plugins" / "config.json"


def _resolve_no_exist(path: Path) -> Path:
    """Resolve ``path`` to an absolute path without requiring it to exist,
    the same job install_plugin.ps1's Resolve-PathNoExist does (``Path
    .resolve()`` handles a non-existent path fine on Python 3.6+; it does
    not require ``strict=True``)."""
    return path.resolve()


def _assert_inside_repo(path: Path):
    """Refuse to touch a path outside the repository root -- mirrors
    tools/install_plugin.ps1's own StartsWith(repo_root) guard exactly."""
    repo_root_norm = str(REPO_ROOT).rstrip("\\/") + "\\"
    path_norm = str(path)
    if not path_norm.lower().startswith(repo_root_norm.lower()):
        raise SystemExit(
            "refusing to write: destination {!r} is outside repo root {!r}".format(
                str(path), str(REPO_ROOT)
            )
        )


def _load_json(path: Path, hint: str) -> dict:
    if not path.is_file():
        raise SystemExit(
            "no config at {}\n{}".format(path, hint)
        )
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SystemExit("{} is not valid JSON: {}".format(path, exc))


def _entry_for(doc: dict, source: Path) -> dict:
    for entry in doc.get("config", []):
        if entry.get("capability") == CAPABILITY:
            return entry
    raise SystemExit(
        "capability {!r} not found in {}. Present: {}".format(
            CAPABILITY, source, [e.get("capability") for e in doc.get("config", [])]
        )
    )


def _cap_config_for(doc: dict, source: Path) -> dict:
    entry = _entry_for(doc, source)
    cap_config = entry.get("cap_config")
    if not isinstance(cap_config, dict):
        raise SystemExit(
            "{} entry has no 'cap_config' object.".format(source)
        )
    return cap_config


def _format_value(value):
    return json.dumps(value)


def _validate_or_exit(cap_config: dict, wall_mm: float, source: str):
    try:
        assert_valid_config(cap_config, wall_width_mm=wall_mm)
    except (ConfigSchemaError, SpecError) as exc:
        raise SystemExit(
            "{} is INVALID:\n\n{}".format(source, exc)
        )


# --- list ---------------------------------------------------------------


def _tracked_descriptions():
    """filename -> one-line description, parsed from presets/plugin/README.md
    section headings of the form ## `file.json` — description, rather than
    hard-coded here, so a new tracked config only needs a README entry to
    show up described in `list`."""
    descriptions = {}
    if PRESETS_README.is_file():
        import re

        text = PRESETS_README.read_text(encoding="utf-8")
        for m in re.finditer(r"^## `([^`]+)`\s*[—-]\s*(.+)$", text, re.MULTILINE):
            descriptions[m.group(1)] = m.group(2).strip()
    return descriptions


def cmd_list(args):
    descriptions = _tracked_descriptions()
    files = sorted(p.name for p in PRESETS_DIR.glob("*.json"))
    if not files:
        print("no tracked configs found in {}".format(PRESETS_DIR))
        return 0
    print("tracked configs in {}:\n".format(PRESETS_DIR))
    for name in files:
        desc = descriptions.get(name, "(no description in README.md)")
        print("  {:<36} {}".format(name, desc))
    print("\nuse `python tools/config.py use <name>` to install one.")
    return 0


# --- show -----------------------------------------------------------------


def _flatten_for_diff(cap_config: dict):
    """Top-level scalar keys plus a per-family breakdown, in schema order,
    for the ``show`` labelled printout / diff."""
    top = {k: cap_config.get(k, SPEC_DEFAULTS[k]) for k in SPEC_FIELDS if k != "families"}
    families = cap_config.get("families") or []
    return top, families


def cmd_show(args):
    datadir = Path(args.datadir)
    live_path = _live_config_path(datadir)
    live_doc = _load_json(
        live_path,
        "Install and activate the plugin first, and open its Config tab "
        "once so the file exists, or run `python tools/config.py use "
        "<name>` to install a tracked one.",
    )
    live_cap = _cap_config_for(live_doc, live_path)

    default_path = PRESETS_DIR / "default_config.json"
    default_cap = {}
    if default_path.is_file():
        default_cap = _cap_config_for(
            json.loads(default_path.read_text(encoding="utf-8")), default_path
        )

    print("live config: {}\n".format(live_path))
    top, families = _flatten_for_diff(live_cap)
    default_top, default_families = _flatten_for_diff(default_cap) if default_cap else ({}, [])

    for key in SPEC_FIELDS:
        if key == "families":
            continue
        value = top[key]
        marker = ""
        if default_cap and value != default_top.get(key):
            marker = "  (default_config.json: {})".format(_format_value(default_top.get(key)))
        print("  {:<22} = {:<20}{}".format(key, _format_value(value), marker))

    print("  {:<22} = {} famil{}".format(
        "families", len(families), "y" if len(families) == 1 else "ies"
    ))
    for i, fam in enumerate(families):
        name = fam.get("name", "family {}".format(i))
        print("    [{}] {}".format(i, name))
        for key in FAMILY_FIELDS:
            if key in fam:
                print("        {:<20} = {}".format(key, _format_value(fam[key])))

    if default_cap and families != default_families:
        print(
            "\n  NOTE: families differ from default_config.json ({} vs {} entries)".format(
                len(families), len(default_families)
            )
        )
    return 0


# --- check ------------------------------------------------------------------


def cmd_check(args):
    datadir = Path(args.datadir)
    live_path = _live_config_path(datadir)
    live_doc = _load_json(
        live_path,
        "Install and activate the plugin first, and open its Config tab "
        "once so the file exists.",
    )
    live_cap = _cap_config_for(live_doc, live_path)
    try:
        assert_valid_config(live_cap, wall_width_mm=args.wall)
    except (ConfigSchemaError, SpecError) as exc:
        print("{} is INVALID (assuming outer wall line width {} mm):\n".format(
            live_path, args.wall
        ))
        print(str(exc))
        return 1
    print("{} is valid (checked against outer wall line width {} mm).".format(
        live_path, args.wall
    ))
    return 0


# --- use --------------------------------------------------------------------


def cmd_use(args):
    source = PRESETS_DIR / args.name
    if not source.is_file():
        raise SystemExit(
            "no tracked config named {!r} in {}. Run `python tools/config.py "
            "list` to see what is available.".format(args.name, PRESETS_DIR)
        )
    doc = _load_json(source, "")
    cap_config = _cap_config_for(doc, source)
    _validate_or_exit(cap_config, args.wall, str(source))

    datadir = Path(args.datadir)
    live_path = _live_config_path(datadir)
    dest = _resolve_no_exist(live_path)
    _assert_inside_repo(dest)

    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(doc, indent=4), encoding="utf-8")
    print("WROTE {}".format(dest))
    print("  from tracked config: {}".format(source))
    print("\nRESTART the host before slicing -- plugin instances are captured")
    print("at load time, so a running OrcaSlicer keeps the old value.")
    return 0


# --- set --------------------------------------------------------------------


def _parse_value(raw: str):
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return raw  # a bare word (e.g. a family name) is used as a literal string


def _find_family(families, name):
    for i, fam in enumerate(families):
        if fam.get("name") == name:
            return i, fam
    return None, None


def cmd_set(args):
    datadir = Path(args.datadir)
    live_path = _live_config_path(datadir)
    dest = _resolve_no_exist(live_path)
    _assert_inside_repo(dest)

    live_doc = _load_json(
        live_path,
        "Install and activate the plugin first, and open its Config tab "
        "once so the file exists, or run `python tools/config.py use "
        "<name>` to install a tracked one first.",
    )
    entry = _entry_for(live_doc, live_path)
    cap_config = entry.setdefault("cap_config", {})

    value = _parse_value(args.value)
    parts = args.key.split(".")

    if len(parts) == 1:
        key = parts[0]
        if key not in SPEC_FIELDS:
            raise SystemExit(unknown_top_level_key_message(key))
        old = cap_config.get(key, SPEC_DEFAULTS[key])
        cap_config[key] = value
        where = "top-level key {!r}".format(key)
    elif len(parts) == 3 and parts[0] == "families":
        _, family_name, key = parts
        if key not in FAMILY_FIELDS:
            raise SystemExit(unknown_family_key_message(key))
        families = cap_config.setdefault("families", [])
        idx, fam = _find_family(families, family_name)
        if fam is None:
            raise SystemExit(
                "no family named {!r} exists in the live config; `set` never "
                "creates a family. Existing families: {}. Edit "
                "'families' by hand (or via `use`) to add one.".format(
                    family_name, [f.get("name") for f in families]
                )
            )
        old = fam.get(key)
        fam[key] = value
        where = "family {!r} key {!r}".format(family_name, key)
    else:
        raise SystemExit(
            "unrecognized key syntax {!r}. Use a top-level key (e.g. "
            "'skin_margin_mm'), or 'families.<name>.<key>' to reach into a "
            "family by name (e.g. 'families.spar.rib_pitch_mm').".format(args.key)
        )

    _validate_or_exit(cap_config, args.wall, "resulting config")

    live_path.write_text(json.dumps(live_doc, indent=4), encoding="utf-8")
    print("WROTE {}".format(live_path))
    print("  {}: {} -> {}".format(where, _format_value(old), _format_value(value)))
    print("\nRESTART the host before slicing -- plugin instances are captured")
    print("at load time, so a running OrcaSlicer keeps the old value.")
    return 0


# --- docs ---------------------------------------------------------------


def _render_field_table(fields, heading):
    lines = ["## {}".format(heading), ""]
    lines.append("| key | type | default | range | description |")
    lines.append("|---|---|---|---|---|")
    for key, field in fields.items():
        type_text = field.type.__name__ + (" or null" if field.none_allowed else "")
        default_text = "`{}`".format(_format_value(field.default))
        range_text = field.range_text() or "--"
        lines.append(
            "| `{}` | {} | {} | {} | {} |".format(
                key, type_text, default_text, range_text, field.description
            )
        )
    lines.append("")
    return "\n".join(lines)


def render_docs() -> str:
    """Render docs/configuration.md's full text from the schema. The single
    generation point tools/config.py's own `docs` subcommand and the golden
    test in tests/unit/test_config_cli.py both call, so the checked-in file
    can never silently drift from aircraft_core/config_schema.py."""
    parts = [
        "# Aircraft Structures plugin configuration",
        "",
        "GENERATED by `python tools/config.py docs` from "
        "`aircraft_core/config_schema.py` -- do not hand-edit; edit the "
        "schema and regenerate.",
        "",
        "Every key the plugin's stored `cap_config` accepts. An unknown key "
        "is an error, not silence -- see `tools/config.py check`.",
        "",
        "This CLI fails closed on an unknown top-level key. The running "
        "plugin does not: a top-level `cap_config` key is round-tripped by "
        "the host itself, and no reachable source rules out the host ever "
        "adding one of its own, so the running plugin only warns loudly "
        "(the same message, on the slice's host-log result line and in its "
        "diagnostics JSON) and keeps using the default for that key -- see "
        "`plugin/aircraft_structures/capability.py`'s "
        "`_unknown_top_level_key_warning` docstring for the full reasoning. "
        "An unknown key inside a `families` entry IS fail-closed everywhere, "
        "including the running plugin, since that data is unambiguously "
        "this plugin's own.",
        "",
        _render_field_table(SPEC_FIELDS, "Top-level keys"),
        "Exactly one of `gap_ratio` / `slot_gap_mm` must be set (the other "
        "left `null`).",
        "",
        _render_field_table(FAMILY_FIELDS, "Family keys (`families[i]`)"),
        "Each entry of the top-level `families` list accepts only these "
        "keys. `families` empty (the default) reproduces the "
        "single-implicit-family behaviour built from the top-level fields "
        "above.",
        "",
        "## Cross-field rules",
        "",
        "- Exactly one of `gap_ratio` / `slot_gap_mm` must be set, at the "
        "top level and independently on each family (the other left "
        "`null`, or both left `null` on a family to inherit the spec's own "
        "gap mode).",
        "- `z_period_mm` requires `z_drift_mm_per_mm` to also be set, at "
        "the top level and independently on each family.",
        "- `skin_margin_mm` must exceed `2 * ` the resolved outer wall "
        "line width -- load-bearing for the no-sever proof. "
        "A config that violates this is "
        "refused before any layer is generated.",
        "",
        "## Tooling",
        "",
        "- `python tools/config.py list` -- tracked configs in "
        "`presets/plugin/`.",
        "- `python tools/config.py show` -- the live config, labelled.",
        "- `python tools/config.py use <name>` -- install a tracked "
        "config, validated first.",
        "- `python tools/config.py check` -- validate the live config.",
        "- `python tools/config.py set <key> <value>` -- change one value, "
        "validated, preserving everything else. Dotted "
        "`families.<name>.<key>` reaches into a family by name.",
        "",
    ]
    return "\n".join(parts) + "\n"


def cmd_docs(args):
    output_path = REPO_ROOT / "docs" / "configuration.md"
    text = render_docs()
    output_path.write_text(text, encoding="utf-8")
    print("WROTE {}".format(output_path))
    return 0


# --- entry point --------------------------------------------------------


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--datadir",
        default=str(REPO_ROOT / "host" / "data"),
        help="OrcaSlicer data dir holding orca_plugins/config.json (default: host/data)",
    )
    parser.add_argument(
        "--wall",
        type=float,
        default=DEFAULT_WALL_MM,
        help="resolved outer wall line width in mm, for the skin_margin_mm "
        "cross-field check (default: {})".format(DEFAULT_WALL_MM),
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="show tracked configs in presets/plugin/")
    sub.add_parser("show", help="print the live config, labelled")
    sub.add_parser("check", help="validate the live config")
    sub.add_parser("docs", help="render docs/configuration.md from the schema")

    p_use = sub.add_parser("use", help="install a tracked config")
    p_use.add_argument("name", help="filename in presets/plugin/ (see `list`)")

    p_set = sub.add_parser("set", help="change one value in the live config")
    p_set.add_argument("key", help="top-level key, or 'families.<name>.<key>'")
    p_set.add_argument("value", help="new value (JSON-parsed when possible)")

    args = parser.parse_args(argv)

    handlers = {
        "list": cmd_list,
        "show": cmd_show,
        "check": cmd_check,
        "use": cmd_use,
        "set": cmd_set,
        "docs": cmd_docs,
    }
    return handlers[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
