"""Tests for tools/build_plugin.py, the deterministic aircraft_core bundler.

Runs the bundler into a temporary directory only — never touches the real
``plugin/aircraft_structures/aircraft_structures.py`` entry file.
"""
import ast
import importlib.util
import sys
import tempfile
import tomllib
import types
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CORE_DIR = REPO_ROOT / "aircraft_core"
BUILD_SCRIPT = REPO_ROOT / "tools" / "build_plugin.py"

CORE_MODULE_ORDER = (
    "ops.py",
    "spec.py",
    "config_schema.py",
    "frame.py",
    "generator.py",
    "slot.py",
    "validate.py",
    "diagnostics.py",
    "metrics.py",
)

STUB_CAPABILITY_SOURCE = '''# /// script
# requires-python = ">=3.11"
# dependencies = []
#
# [tool.orcaslicer.plugin]
# name = "Aircraft Structures"
# description = "Test stub capability for the bundler."
# author = "aircraft-orca-structures"
# version = "0.1.0"
# ///
"""Stub capability used only by test_bundle.py."""

import orca


class AircraftStructuresCapability(orca.slicing.SlicingPipelineCapabilityBase):
    def get_name(self):
        return "Aircraft Structures"

    def execute(self, ctx):
        return orca.ExecutionResult.success()


@orca.plugin
class AircraftStructures(orca.base):
    def register_capabilities(self):
        orca.register_capability(AircraftStructuresCapability)
'''


def _load_build_plugin_module():
    spec = importlib.util.spec_from_file_location("build_plugin_under_test", BUILD_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _public_symbols(source_path: Path):
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    return [
        node.name
        for node in tree.body
        if isinstance(node, (ast.ClassDef, ast.FunctionDef))
        and not node.name.startswith("_")
    ]


def _install_stub_orca_module():
    """Inject a minimal stub ``orca`` module so ``import orca`` resolves.

    Returns the previous value of ``sys.modules['orca']`` (or a sentinel) so
    the caller can restore it afterwards and never pollute other tests.
    """
    stub = types.ModuleType("orca")

    def _plugin(cls):
        return cls

    stub.plugin = _plugin
    stub.base = object

    slicing = types.ModuleType("orca.slicing")

    class _SlicingPipelineCapabilityBase:
        pass

    slicing.SlicingPipelineCapabilityBase = _SlicingPipelineCapabilityBase
    stub.slicing = slicing

    class _ExecutionResult:
        @staticmethod
        def success(*args, **kwargs):
            return None

        @staticmethod
        def failure(*args, **kwargs):
            return None

    stub.ExecutionResult = _ExecutionResult

    def _register_capability(*args, **kwargs):
        return None

    stub.register_capability = _register_capability

    previous = sys.modules.get("orca")
    sys.modules["orca"] = stub
    sys.modules["orca.slicing"] = slicing
    return previous


def _restore_orca_module(previous):
    if previous is None:
        sys.modules.pop("orca", None)
    else:
        sys.modules["orca"] = previous
    sys.modules.pop("orca.slicing", None)


class BuildPluginTest(unittest.TestCase):
    def setUp(self):
        self.build_plugin = _load_build_plugin_module()
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.tmp_dir = Path(self._tmp.name)
        self.capability_path = self.tmp_dir / "capability.py"
        self.capability_path.write_text(STUB_CAPABILITY_SOURCE, encoding="utf-8")
        self.output_path = self.tmp_dir / "aircraft_structures.py"

    def _build(self):
        return self.build_plugin.build_bundle(CORE_DIR, self.capability_path)

    def test_missing_capability_raises(self):
        missing = self.tmp_dir / "does_not_exist.py"
        with self.assertRaises(self.build_plugin.BundleError):
            self.build_plugin.build_bundle(CORE_DIR, missing)

    def test_bundle_compiles(self):
        bundle = self._build()
        compile(bundle, str(self.output_path), "exec")  # raises on failure

    def test_no_intrapackage_imports_survive(self):
        bundle = self._build()
        for lineno, line in enumerate(bundle.splitlines(), start=1):
            stripped = line.strip()
            self.assertFalse(
                stripped.startswith("from .")
                or stripped.startswith("from aircraft_core")
                or stripped.startswith("import aircraft_core"),
                "intra-package import survived at line {}: {!r}".format(lineno, line),
            )

    def test_pep723_block_is_first_and_parses(self):
        bundle = self._build()
        lines = bundle.splitlines()
        self.assertEqual(lines[0], "# /// script")

        toml_lines = []
        started = False
        for line in lines:
            if line == "# /// script":
                started = True
                continue
            if line == "# ///":
                break
            if started:
                self.assertTrue(line == "#" or line.startswith("# "))
                toml_lines.append(line[2:] if line.startswith("# ") else "")
        toml_text = "\n".join(toml_lines)
        parsed = tomllib.loads(toml_text)
        self.assertIn("tool", parsed)
        self.assertIn("orcaslicer", parsed["tool"])
        self.assertIn("plugin", parsed["tool"]["orcaslicer"])

    def test_all_public_symbols_present_after_exec(self):
        bundle = self._build()
        previous_orca = _install_stub_orca_module()
        try:
            namespace = {"__name__": "aircraft_structures_bundle_under_test"}
            exec(compile(bundle, str(self.output_path), "exec"), namespace)
        finally:
            _restore_orca_module(previous_orca)

        for name in CORE_MODULE_ORDER:
            for symbol in _public_symbols(CORE_DIR / name):
                self.assertIn(
                    symbol,
                    namespace,
                    "{} from {} missing in bundled namespace".format(symbol, name),
                )

        # The capability's own public class should also have made it in.
        self.assertIn("AircraftStructures", namespace)
        self.assertIn("AircraftStructuresCapability", namespace)

    def test_deterministic_across_builds(self):
        first = self._build()
        second = self._build()
        self.assertEqual(first, second)

    def test_cli_writes_output_and_prints_summary(self):
        rc = self.build_plugin.main(
            [
                "--capability",
                str(self.capability_path),
                "--output",
                str(self.output_path),
                "--core-dir",
                str(CORE_DIR),
            ]
        )
        self.assertEqual(rc, 0)
        self.assertTrue(self.output_path.is_file())
        written = self.output_path.read_text(encoding="utf-8")
        self.assertEqual(written, self._build())

    def test_cli_fails_loudly_on_missing_capability(self):
        rc = self.build_plugin.main(
            [
                "--capability",
                str(self.tmp_dir / "nope.py"),
                "--output",
                str(self.output_path),
                "--core-dir",
                str(CORE_DIR),
            ]
        )
        self.assertNotEqual(rc, 0)
        self.assertFalse(self.output_path.exists())

    def test_real_plugin_entry_file_not_touched(self):
        real_output = REPO_ROOT / "plugin" / "aircraft_structures" / "aircraft_structures.py"
        before = real_output.read_bytes()
        self._build()
        self.build_plugin.main(
            [
                "--capability",
                str(self.capability_path),
                "--output",
                str(self.output_path),
                "--core-dir",
                str(CORE_DIR),
            ]
        )
        after = real_output.read_bytes()
        self.assertEqual(before, after)




class BundleFreshnessTests(unittest.TestCase):
    """The checked-in bundle must match what the sources build right now.

    Without this, a stale `plugin/aircraft_structures/aircraft_structures.py`
    passes every other test — the unit tier builds from a stub capability into
    a temp dir, and the integration tier execs whatever is checked in. The
    stale file is the one that actually gets installed into Orca.
    """

    def test_checked_in_bundle_matches_its_sources(self):
        repo_root = Path(__file__).resolve().parent.parent.parent
        checked_in = repo_root / "plugin" / "aircraft_structures" / "aircraft_structures.py"
        self.assertTrue(checked_in.is_file(), "bundle is missing: {}".format(checked_in))

        builder = _load_build_plugin_module()
        rebuilt = builder.build_bundle(
            repo_root / "aircraft_core",
            repo_root / "plugin" / "aircraft_structures" / "capability.py",
        )
        self.assertEqual(
            rebuilt,
            checked_in.read_text(encoding="utf-8"),
            "checked-in bundle is stale — run `python tools/build_plugin.py`",
        )


if __name__ == "__main__":
    unittest.main()
