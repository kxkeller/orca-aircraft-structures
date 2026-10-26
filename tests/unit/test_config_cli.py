"""Tests for tools/config.py.

Loads the script by path (importlib.util.spec_from_file_location), matching
tests/unit/test_bundle.py's own precedent for loading a tools/ script rather
than relying on tools/ being importable as a namespace package.

Two kinds of scratch location are used, deliberately never the real
host/data or presets/plugin/:

- an IN-REPO scratch directory (tests/_scratch_config_cli/, removed in
  tearDown) for ordinary functional tests -- `use`/`set` refuse to write
  outside the repo root, so a normal round-trip test needs a destination
  that is legitimately inside it;
- the OS temp directory (guaranteed outside the repo root) specifically for
  the outside-repo refusal tests.
"""
import importlib.util
import io
import json
import shutil
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CONFIG_SCRIPT = REPO_ROOT / "tools" / "config.py"
IN_REPO_SCRATCH = REPO_ROOT / "tests" / "_scratch_config_cli"


def _load_config_module():
    spec = importlib.util.spec_from_file_location("config_under_test", CONFIG_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


config = _load_config_module()


def _run(argv):
    buf = io.StringIO()
    try:
        with redirect_stdout(buf):
            code = config.main(argv)
    except SystemExit as exc:
        return (exc.code if isinstance(exc.code, int) else 1), buf.getvalue() + str(exc)
    return code, buf.getvalue()


class InRepoScratchTestBase(unittest.TestCase):
    """Base for tests that need a real, in-repo datadir to write into."""

    def setUp(self):
        if IN_REPO_SCRATCH.exists():
            shutil.rmtree(IN_REPO_SCRATCH)
        IN_REPO_SCRATCH.mkdir(parents=True)
        self.addCleanup(lambda: shutil.rmtree(IN_REPO_SCRATCH, ignore_errors=True))
        self.datadir = IN_REPO_SCRATCH


class OutsideRepoTestBase(unittest.TestCase):
    """Base for tests that need a datadir guaranteed OUTSIDE the repo."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.datadir = Path(self._tmp.name)
        self.assertFalse(
            str(self.datadir).lower().startswith(str(REPO_ROOT).lower()),
            "OS temp dir unexpectedly inside the repo; fix the test environment",
        )


class ListCommandTests(unittest.TestCase):
    def test_list_shows_every_tracked_config_with_a_description(self):
        # Glob presets/plugin/*.json rather than hardcoding filenames, so
        # adding/renaming a tracked config cannot silently break this test
        # the way the deleted two_sided_config.OPT-IN.json did.
        code, out = _run(["list"])
        self.assertEqual(code, 0)
        tracked_names = sorted(p.name for p in config.PRESETS_DIR.glob("*.json"))
        self.assertGreaterEqual(
            len(tracked_names), 2, "expected at least two tracked configs"
        )
        for name in tracked_names:
            self.assertIn(name, out)

        # default_config.json has a real README heading and gets its
        # description from it.
        self.assertIn("what ships", out)

        # conservative_config.json has its own README heading too, so this
        # checks the real description rather than a placeholder.
        #
        # The point of asserting a SUBSTANTIVE description rather than just
        # "some text appeared": `list` is how a user discovers what these
        # configs are, and a config whose description is a placeholder is
        # undiscoverable in the way that matters.
        self.assertIn("the previous one-sided default", out)
        self.assertNotIn("(no description in README.md)", out)


class ShowCheckNoLiveConfigTests(InRepoScratchTestBase):
    def test_show_with_no_live_config_refuses_actionably(self):
        code, out = _run(["--datadir", str(self.datadir), "show"])
        self.assertNotEqual(code, 0)
        self.assertIn("no config at", out)

    def test_check_with_no_live_config_refuses_actionably(self):
        code, out = _run(["--datadir", str(self.datadir), "check"])
        self.assertNotEqual(code, 0)
        self.assertIn("no config at", out)


class UseCommandTests(InRepoScratchTestBase):
    def test_use_installs_a_valid_tracked_config(self):
        # Read the expected value from the tracked config itself rather
        # than hardcoding it -- a hardcoded figure here pins whatever
        # skin_margin_mm happens to ship and breaks at the next default
        # change.
        default_doc = json.loads(
            (config.PRESETS_DIR / "default_config.json").read_text(encoding="utf-8")
        )
        expected_margin = default_doc["config"][0]["cap_config"]["skin_margin_mm"]

        code, out = _run(["--datadir", str(self.datadir), "use", "default_config.json"])
        self.assertEqual(code, 0)
        live_path = self.datadir / "orca_plugins" / "config.json"
        self.assertTrue(live_path.is_file())
        doc = json.loads(live_path.read_text(encoding="utf-8"))
        self.assertEqual(
            doc["config"][0]["cap_config"]["skin_margin_mm"], expected_margin
        )

    def test_use_every_other_tracked_config_installs_clean(self):
        # Every tracked config besides default_config.json (which has its
        # own dedicated test above) must install cleanly too. Looped via
        # glob rather than hardcoding a filename, so a newly added tracked
        # config is covered automatically -- the deleted
        # two_sided_config.OPT-IN.json was hardcoded here and this test
        # went on passing right up until the file it named vanished.
        other_names = sorted(
            p.name
            for p in config.PRESETS_DIR.glob("*.json")
            if p.name != "default_config.json"
        )
        self.assertGreater(
            len(other_names), 0, "expected at least one non-default tracked config"
        )
        for name in other_names:
            with self.subTest(name=name):
                code, out = _run(["--datadir", str(self.datadir), "use", name])
                self.assertEqual(code, 0, out)

    def test_use_unknown_tracked_name_refuses(self):
        code, out = _run(["--datadir", str(self.datadir), "use", "does_not_exist.json"])
        self.assertNotEqual(code, 0)
        self.assertIn("no tracked config named", out)


class OutsideRepoRefusalTests(OutsideRepoTestBase):
    def test_use_refuses_when_destination_outside_repo(self):
        code, out = _run(["--datadir", str(self.datadir), "use", "default_config.json"])
        self.assertNotEqual(code, 0)
        self.assertIn("outside repo root", out)
        self.assertFalse((self.datadir / "orca_plugins" / "config.json").exists())

    def test_set_refuses_when_destination_outside_repo(self):
        # No live config exists here at all (use is what would have been
        # refused above), so `set` must refuse on the path guard, before
        # ever trying to read a config that isn't there.
        code, out = _run(["--datadir", str(self.datadir), "set", "skin_margin_mm", "1.4"])
        self.assertNotEqual(code, 0)
        self.assertIn("outside repo root", out)


class UseRefusesInvalidPresetTests(unittest.TestCase):
    """End-to-end: point the loaded module's PRESETS_DIR at a scratch
    directory containing a deliberately invalid config, and confirm `use`
    refuses to write it -- never touches the real presets/plugin/."""

    def setUp(self):
        self._real_presets_dir = config.PRESETS_DIR
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.scratch_presets = Path(self._tmp.name) / "presets"
        self.scratch_presets.mkdir()
        config.PRESETS_DIR = self.scratch_presets
        self.addCleanup(lambda: setattr(config, "PRESETS_DIR", self._real_presets_dir))
        if IN_REPO_SCRATCH.exists():
            shutil.rmtree(IN_REPO_SCRATCH)
        IN_REPO_SCRATCH.mkdir(parents=True)
        self.addCleanup(lambda: shutil.rmtree(IN_REPO_SCRATCH, ignore_errors=True))

    def _write_preset(self, name, cap_config):
        doc = {
            "config": [
                {
                    "cap_config": cap_config,
                    "capability": "Aircraft Structures",
                    "capability_type": "slicing-pipeline",
                    "plugin_key": "aircraft_structures",
                    "plugin_version": "0.1.0",
                }
            ]
        }
        (self.scratch_presets / name).write_text(json.dumps(doc), encoding="utf-8")

    def test_use_refuses_invalid_config_and_does_not_write(self):
        self._write_preset("bad.json", {"skin_margin_mm": -1.0})
        code, out = _run(["--datadir", str(IN_REPO_SCRATCH), "use", "bad.json"])
        self.assertNotEqual(code, 0)
        self.assertIn("INVALID", out)
        self.assertFalse((IN_REPO_SCRATCH / "orca_plugins" / "config.json").is_file())

    def test_use_refuses_config_with_unknown_key(self):
        self._write_preset("typo.json", {"sking_margin_mm": 1.0})
        code, out = _run(["--datadir", str(IN_REPO_SCRATCH), "use", "typo.json"])
        self.assertNotEqual(code, 0)
        self.assertIn("did you mean", out)


class SetCommandTests(InRepoScratchTestBase):
    def setUp(self):
        super().setUp()
        code, _ = _run(["--datadir", str(self.datadir), "use", "default_config.json"])
        self.assertEqual(code, 0)
        self.live_path = self.datadir / "orca_plugins" / "config.json"
        # Read the just-installed config's own starting values rather than
        # hardcoding default_config.json numbers/family names --
        # those change whenever the default changes.
        self.before_cap_config = self._read_live()["config"][0]["cap_config"]

    def _read_live(self):
        return json.loads(self.live_path.read_text(encoding="utf-8"))

    def test_set_top_level_value(self):
        old_margin = self.before_cap_config["skin_margin_mm"]
        code, out = _run(["--datadir", str(self.datadir), "set", "skin_margin_mm", "1.4"])
        self.assertEqual(code, 0)
        doc = self._read_live()
        self.assertEqual(doc["config"][0]["cap_config"]["skin_margin_mm"], 1.4)
        self.assertIn("{} -> 1.4".format(json.dumps(old_margin)), out)

    def test_set_null_value(self):
        code, out = _run(["--datadir", str(self.datadir), "set", "rib_pitch_mm", "null"])
        self.assertEqual(code, 0)
        doc = self._read_live()
        self.assertIsNone(doc["config"][0]["cap_config"]["rib_pitch_mm"])

    def test_set_family_scoped_value(self):
        # Use whatever the first family in the tracked config is actually
        # named, instead of hardcoding a family name that may not exist in
        # the shipped default.
        #
        # Uses web_depth_mm, not rib_pitch_mm, as the scoped field under
        # test: this test is exercising the `set` command's generic
        # family-scoped-key mechanics, not pitch semantics, and
        # rib_pitch_mm has a mutually-exclusive relative counterpart
        # (rib_pitch_chord_frac) on the shipped default -- setting
        # rib_pitch_mm there leaves both fields set and the resulting
        # config INVALID (see WebFamily.validate()). web_depth_mm has no
        # such counterpart and is safe against the next mode conversion.
        family_name = self.before_cap_config["families"][0]["name"]
        code, out = _run(
            [
                "--datadir",
                str(self.datadir),
                "set",
                "families.{}.web_depth_mm".format(family_name),
                "2.0",
            ]
        )
        self.assertEqual(code, 0)
        doc = self._read_live()
        families = doc["config"][0]["cap_config"]["families"]
        fam = next(f for f in families if f["name"] == family_name)
        self.assertEqual(fam["web_depth_mm"], 2.0)

    def test_set_unknown_top_level_key_refuses(self):
        code, out = _run(["--datadir", str(self.datadir), "set", "sking_margin_mm", "1.0"])
        self.assertNotEqual(code, 0)
        self.assertIn("did you mean", out)

    def test_set_nonexistent_family_refuses_and_does_not_create_one(self):
        before = self._read_live()
        code, out = _run(
            ["--datadir", str(self.datadir), "set", "families.nope.rib_pitch_mm", "5.0"]
        )
        self.assertNotEqual(code, 0)
        self.assertIn("never creates a family", out)
        after = self._read_live()
        self.assertEqual(before, after)

    def test_set_invalid_value_refuses_and_does_not_write(self):
        before = self._read_live()
        code, out = _run(["--datadir", str(self.datadir), "set", "skin_margin_mm", "-1"])
        self.assertNotEqual(code, 0)
        self.assertIn("INVALID", out)
        after = self._read_live()
        self.assertEqual(before, after)

    def test_set_preserves_every_other_key_round_trip(self):
        before = self._read_live()
        code, _ = _run(["--datadir", str(self.datadir), "set", "web_position", "0.4"])
        self.assertEqual(code, 0)
        code, _ = _run(["--datadir", str(self.datadir), "set", "web_position", "0.5"])
        self.assertEqual(code, 0)
        after = self._read_live()
        self.assertEqual(before, after)

    def test_set_family_value_round_trip_preserves_config(self):
        # Uses web_depth_mm, not rib_pitch_mm, for the same reason as
        # test_set_family_scoped_value above: this is a round-trip check
        # of the `set` command's own mechanics, and rib_pitch_mm
        # conflicts with the default's rib_pitch_chord_frac (see
        # WebFamily.validate()).
        before = self._read_live()
        family_name = self.before_cap_config["families"][0]["name"]
        original = next(
            f
            for f in before["config"][0]["cap_config"]["families"]
            if f["name"] == family_name
        )["web_depth_mm"]
        code, _ = _run(
            [
                "--datadir",
                str(self.datadir),
                "set",
                "families.{}.web_depth_mm".format(family_name),
                "99.0",
            ]
        )
        self.assertEqual(code, 0)
        # Round-trip back to the family's OWN original value (which may be
        # null for a full-depth family), not a hardcoded literal that only
        # matches one particular family.
        code, _ = _run(
            [
                "--datadir",
                str(self.datadir),
                "set",
                "families.{}.web_depth_mm".format(family_name),
                json.dumps(original),
            ]
        )
        self.assertEqual(code, 0)
        after = self._read_live()
        self.assertEqual(before, after)


class DocsCommandTests(unittest.TestCase):
    def test_docs_render_is_deterministic(self):
        self.assertEqual(config.render_docs(), config.render_docs())

    def test_docs_render_mentions_every_schema_key(self):
        text = config.render_docs()
        for key in config.SPEC_FIELDS:
            self.assertIn("`{}`".format(key), text)
        for key in config.FAMILY_FIELDS:
            self.assertIn("`{}`".format(key), text)

    def test_checked_in_docs_matches_current_render(self):
        checked_in = (REPO_ROOT / "docs" / "configuration.md").read_text(encoding="utf-8")
        self.assertEqual(
            checked_in,
            config.render_docs(),
            "docs/configuration.md is stale -- run `python tools/config.py "
            "docs` to regenerate it from aircraft_core/config_schema.py.",
        )


if __name__ == "__main__":
    unittest.main()
