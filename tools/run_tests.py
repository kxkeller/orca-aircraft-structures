"""Run the whole test suite: unit tests and the fake-host integration tests.

One command exists because the two trees cannot share a discovery root.
``tests/unit`` is a flat directory whose modules import each other as siblings
(``from stub_ops import...``), which only works when that directory is itself
the top level. ``tests/integration`` is a package. Pointing stdlib ``unittest
discover`` at ``tests`` therefore finds only the package half, and "the tests
passed" would silently mean "only some of them ran".

Usage:
    python tools/run_tests.py [-v]
"""

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

SUITES = (
    ("unit", REPO_ROOT / "tests" / "unit", REPO_ROOT / "tests" / "unit"),
    ("integration", REPO_ROOT / "tests" / "integration", REPO_ROOT),
)


def main(argv):
    verbosity = 2 if "-v" in argv else 1
    loader = unittest.TestLoader()
    runner = unittest.TextTestRunner(verbosity=verbosity)

    total = failures = errors = 0
    for name, start_dir, top_level in SUITES:
        if not start_dir.is_dir():
            print("MISSING suite directory: {}".format(start_dir), file=sys.stderr)
            return 2

        # Two path entries are needed, for different reasons: the repo root so
        # `import aircraft_core` resolves, and the suite's own top level so
        # unit tests' sibling imports (`from stub_ops import ...`) resolve.
        added = [str(REPO_ROOT), str(top_level)]
        for entry in added:
            sys.path.insert(0, entry)
        try:
            suite = loader.discover(str(start_dir), top_level_dir=str(top_level))
            print("\n=== {} ===".format(name))
            result = runner.run(suite)
        finally:
            for entry in added:
                if entry in sys.path:
                    sys.path.remove(entry)

        total += result.testsRun
        failures += len(result.failures)
        errors += len(result.errors)

    print(
        "\nTOTAL: {} tests, {} failures, {} errors".format(total, failures, errors)
    )
    return 0 if failures == 0 and errors == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
