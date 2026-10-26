"""Deterministic bundler for the ``aircraft_structures`` Orca plugin.

OrcaSlicer installs exactly one entry file per plugin folder
(``data_dir()/orca_plugins/<entry_file>/<entry_file>``); a package directory
cannot be installed. This script flattens the multi-module ``aircraft_core``
package plus one capability source file into a single installable module, in
dependency order, with every intra-package import stripped and every
remaining (stdlib/orca) import hoisted to the top.

Usage::

    python tools/build_plugin.py [--capability PATH] [--output PATH] [--core-dir PATH]

Deterministic: identical inputs always produce byte-identical output. No
timestamps, no absolute paths embedded in the generated content, no
dict-ordering nondeterminism (hoisted imports are deduplicated via a set and
then sorted before emission).

This file only performs the bundling described above and its own output
verification (``compile()`` plus a scan for surviving intra-package imports);
it does not alter ``aircraft_core`` or the capability source.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import sys
from pathlib import Path
from typing import List, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CORE_DIR = REPO_ROOT / "aircraft_core"
DEFAULT_CAPABILITY = REPO_ROOT / "plugin" / "aircraft_structures" / "capability.py"
DEFAULT_OUTPUT = REPO_ROOT / "plugin" / "aircraft_structures" / "aircraft_structures.py"

# Dependency order: ops.py has no aircraft_core dependencies; each later
# module depends only on modules earlier in this list (see the module
# docstrings for the dependency rationale).
CORE_MODULE_ORDER = (
    "ops.py",
    # pure mm-float alignment geometry (min-area-rect axis, leading- edge side,
    # the single rotate_points_deg function every rotation call site must
    # share). No dependency on any other core module, and no other core module
    # depends on it -- only the capability itself (mesh sectioning, host slice
    # mutation) calls it directly. Placed early, alongside ops.py, on the same
    # "no internal dependency" grounds.
    "alignment.py",
    # which Z end is the wing root (area + spar-channel signals). Plain float
    # geometry; depends only on alignment.py (convex hull).
    "root_end.py",
    # pure triangle-soup geometry with no dependency on the other core modules,
    # so it can be inlined early. The capability needs it to cross-section the
    # ORIGINAL mesh, which Spiral Vase never touches, and recover the spar
    # channel the slices no longer carry.
    "mesh_section.py",
    "spec.py",
    "config_schema.py",
    "frame.py",
    "generator.py",
    "slot.py",
    "validate.py",
    # post-validation opposing-pair ablation; depends on generator.py
    # (OmittedWeb) and validate.py (TopologyError), nothing later.
    "pair_ablation.py",
    # root-end-aware spar-socket latch walk; depends on generator.py
    # (SPAR_STATUS_*).
    "spar_latch.py",
    # pinched-section shutdown; depends only on validate.py.
    "pinch.py",
    # per-web report rows and the report-only continuity checker. Pure Python,
    # no dependency on any other core module.
    "continuity.py",
    # the Z-continuity rule (flag ``z_continuity``); depends on frame.py,
    # generator.py, validate.py and continuity.py.
    "z_rule.py",
    "diagnostics.py",
    "metrics.py",
)


class BundleError(RuntimeError):
    """A condition that must abort the bundle rather than emit a partial one."""


# --- PEP 723 metadata block extraction --------------------------------------


def _extract_pep723_block(text: str, source_name: str) -> Tuple[str, str]:
    """Split ``text`` into (pep723_block_with_markers, remaining_text).

    Raises ``BundleError`` if no well-formed ``# /// script`` ... ``# ///``
    block is found — Orca will not read the plugin manifest without one, so a
    capability file missing it cannot produce a usable bundle.
    """
    lines = text.splitlines(keepends=True)
    start = None
    for i, line in enumerate(lines):
        if line.rstrip("\n") == "# /// script":
            start = i
            break
    if start is None:
        raise BundleError(
            "{} has no PEP 723 '# /// script' metadata block; Orca will not "
            "read the plugin manifest without one.".format(source_name)
        )
    end = None
    for j in range(start + 1, len(lines)):
        if lines[j].rstrip("\n") == "# ///":
            end = j
            break
    if end is None:
        raise BundleError(
            "{} has an unterminated PEP 723 block (opening '# /// script' with "
            "no closing '# ///' line).".format(source_name)
        )
    block = "".join(lines[start : end + 1])
    remaining = "".join(lines[:start] + lines[end + 1 :])
    return block, remaining


# --- import classification and stripping ------------------------------------


def _is_intrapackage_import(node: ast.AST) -> bool:
    """True for ``from .x import y`` / ``from aircraft_core.x import y`` /
    ``import aircraft_core[.x]`` — anything that only makes sense across
    package files and must be dropped once everything shares one namespace.
    """
    if isinstance(node, ast.ImportFrom):
        if node.level and node.level > 0:
            return True
        if node.module and (
            node.module == "aircraft_core" or node.module.startswith("aircraft_core.")
        ):
            return True
        return False
    if isinstance(node, ast.Import):
        return any(
            alias.name == "aircraft_core" or alias.name.startswith("aircraft_core.")
            for alias in node.names
        )
    return False


def _strip_module_level_imports(
    text: str, tree: ast.Module
) -> Tuple[str, List[ast.AST]]:
    """Remove module-level import statements from ``text``.

    Only imports that are direct children of the module (not nested inside a
    function or class body) are touched, so a lazy/local import inside a
    function is left exactly where it is. Returns the remaining source text
    and the list of surviving (non-intra-package) import nodes that still
    need to be hoisted to the top of the bundle.
    """
    import_nodes = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    remove_lines = set()
    hoist_nodes = []
    for node in import_nodes:
        for lineno in range(node.lineno, node.end_lineno + 1):
            remove_lines.add(lineno)
        if not _is_intrapackage_import(node):
            hoist_nodes.append(node)
    lines = text.splitlines(keepends=True)
    kept = [line for i, line in enumerate(lines, start=1) if i not in remove_lines]
    return "".join(kept), hoist_nodes


def _merge_hoisted_imports(hoist_nodes: List[ast.AST]) -> List[str]:
    """Merge hoisted import nodes into one canonical line per module.

    Multiple ``from X import a`` / ``from X import b`` statements across
    different source files collapse into a single ``from X import a, b``
    line instead of surviving as separate, overlapping lines. Plain
    ``import X [as Y]`` statements are deduplicated by (module, asname).
    Returns the final sorted, deduplicated list of import line strings.
    """
    plain: set = set()
    from_names: dict = {}
    for node in hoist_nodes:
        if isinstance(node, ast.Import):
            for alias in node.names:
                plain.add((alias.name, alias.asname))
        else:
            names = from_names.setdefault(node.module, set())
            for alias in node.names:
                names.add((alias.name, alias.asname))

    lines = []
    for module, asname in plain:
        lines.append(
            "import {}".format(module)
            if asname is None
            else "import {} as {}".format(module, asname)
        )
    for module, names in from_names.items():
        rendered = ", ".join(
            name if asname is None else "{} as {}".format(name, asname)
            for name, asname in sorted(names, key=lambda t: (t[0], t[1] or ""))
        )
        lines.append("from {} import {}".format(module, rendered))
    return sorted(set(lines))


# --- assembly -----------------------------------------------------------------


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _load_core_sources(core_dir: Path) -> List[Tuple[str, str]]:
    sources = []
    for name in CORE_MODULE_ORDER:
        path = core_dir / name
        if not path.is_file():
            raise BundleError(
                "required aircraft_core source is missing: {}".format(path)
            )
        sources.append((name, _read(path)))
    return sources


def _build_body_region(
    sorted_imports: List[str], labeled_bodies: List[Tuple[str, str]]
) -> str:
    chunks = []
    if sorted_imports:
        chunks.append("\n".join(sorted_imports) + "\n")
    for label, body in labeled_bodies:
        chunks.append(
            "# --- begin {} ---\n{}\n# --- end {} ---\n".format(
                label, body.strip("\n"), label
            )
        )
    return ("\n".join(chunks)).rstrip("\n") + "\n"


def _build_header(source_labels: List[str], content_hash: str) -> str:
    lines = [
        "# GENERATED by tools/build_plugin.py -- do not edit; edit the sources",
        "# and rebuild.",
        "#",
        "# Sources ({}), in dependency order: {}".format(
            len(source_labels), ", ".join(source_labels)
        ),
        "# Bundle content hash (sha256 of imports + module bodies below): {}".format(
            content_hash
        ),
    ]
    return "\n".join(lines) + "\n\n"


def build_bundle(core_dir: Path, capability_path: Path) -> str:
    """Build the flattened bundle text. Pure function; performs no writes."""
    core_dir = Path(core_dir)
    capability_path = Path(capability_path)

    if not capability_path.is_file():
        raise BundleError(
            "capability source file not found: {}. Refusing to produce a "
            "partial bundle -- create the capability file first, then "
            "rebuild.".format(capability_path)
        )

    core_sources = _load_core_sources(core_dir)
    capability_text = _read(capability_path)
    pep723_block, capability_remaining = _extract_pep723_block(
        capability_text, str(capability_path)
    )

    labeled_sources = [
        ("aircraft_core/{}".format(name), text) for name, text in core_sources
    ] + [(capability_path.name, capability_remaining)]

    all_hoist_nodes = []
    labeled_bodies = []
    for label, text in labeled_sources:
        try:
            tree = ast.parse(text, filename=label)
        except SyntaxError as exc:
            raise BundleError("{} failed to parse: {}".format(label, exc)) from exc
        body_text, hoist_nodes = _strip_module_level_imports(text, tree)
        all_hoist_nodes.extend(hoist_nodes)
        labeled_bodies.append((label, body_text))

    sorted_imports = _merge_hoisted_imports(all_hoist_nodes)
    body_region = _build_body_region(sorted_imports, labeled_bodies)
    content_hash = hashlib.sha256(body_region.encode("utf-8")).hexdigest()

    source_labels = [label for label, _ in labeled_sources]
    header = _build_header(source_labels, content_hash)

    pep723_text = pep723_block if pep723_block.endswith("\n") else pep723_block + "\n"
    bundle = pep723_text + "\n" + header + body_region
    return bundle


def _verify(bundle: str, output_path: Path) -> None:
    """Compile the bundle and check no intra-package import survived.

    Raises ``BundleError`` on either failure; the caller must not write the
    output in that case.
    """
    try:
        compile(bundle, str(output_path), "exec")
    except SyntaxError as exc:
        raise BundleError("bundled output failed to compile: {}".format(exc)) from exc

    for lineno, line in enumerate(bundle.splitlines(), start=1):
        stripped = line.strip()
        if (
            stripped.startswith("from .")
            or stripped.startswith("from aircraft_core")
            or stripped.startswith("import aircraft_core")
        ):
            raise BundleError(
                "intra-package import survived flattening at line {}: {!r}".format(
                    lineno, line
                )
            )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--capability",
        type=Path,
        default=DEFAULT_CAPABILITY,
        help="Capability source file (default: {})".format(DEFAULT_CAPABILITY),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Bundle output path (default: {})".format(DEFAULT_OUTPUT),
    )
    parser.add_argument(
        "--core-dir",
        type=Path,
        default=DEFAULT_CORE_DIR,
        help="aircraft_core package directory (default: {})".format(DEFAULT_CORE_DIR),
    )
    args = parser.parse_args(argv)

    try:
        bundle = build_bundle(args.core_dir, args.capability)
        _verify(bundle, args.output)
    except BundleError as exc:
        print("build_plugin: {}".format(exc), file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    data = bundle.encode("utf-8")
    args.output.write_bytes(data)

    digest = hashlib.sha256(data).hexdigest()
    line_count = bundle.count("\n")
    print("output: {}".format(args.output))
    print("sha256: {}".format(digest))
    print("lines: {}".format(line_count))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
