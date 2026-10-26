"""Bounds the behaviour-change surface of the mesh XY-centring step against every
real STL fixture in the repository -- not just the wing/ family its own tests
exercise.

WHY THIS EXISTS. ``capability.py._build_mesh_sectioner`` subtracts the
(volume+instance rotation/scale/mirror-transformed, translation-excluded)
mesh's own XY bounding-box centre before cross-sectioning -- confirmed
against the pinned OrcaSlicer source as matching
``PrintObject::PrintObject`` (``m_center_offset`` from
``ModelObject::raw_bounding_box()``, `exp-orca/src/libslic3r/PrintObject.cpp`
around lines 91-103/123) and ``ModelObject::raw_bounding_box()``
(`exp-orca/src/libslic3r/Model.cpp` around lines 1582-1596). For an
IDENTITY volume/instance transform (no rotation/scale/mirror -- the case
every fixture in this repository actually uses), that reduces to: subtract
the raw mesh's own XY bbox centre.

For a fixture whose raw mesh is ALREADY centred at (0, 0), this is a no-op --
exactly why a centring bug is invisible until an arbitrary, off-centre CAD file (a
real wing) was loaded. For a fixture that is NOT centred, the fix
genuinely MOVES that fixture's derived cross-section (and, if it were ever run
through the real host path with a ``model_object``, any detected spar
channel/socket) by its own off-centre amount. This test measures every real STL
fixture's raw XY bbox centre directly (no host, no capability.py, just the same
welded-vertex reading ``tools/m18_mesh_probe.py`` already uses) so that fact is
a recorded MEASUREMENT, not an assumption.

WHAT THIS DOES NOT ASSUME. It does NOT assume every fixture is centred: the
``wing``-family fixtures (the only ones the host-alignment path is actually
evidenced against -- and ``tests/integration/test_m18_model_determinism.py``)
are asserted centred to a tight tolerance, since the in-host evidence
depends on it. The ``coupon`` and ``sweep`` fixtures are NOT
asserted centred -- they measurably are not (see ``KNOWN_OFF_CENTRE_FIXTURES``)
-- and are instead asserted to match their KNOWN, recorded off-centre value, so
a future change to those fixtures is still caught. None of
those off-centre fixtures are used anywhere with a ``model_object`` through
``capability.py``'s real path (only via ``tools/m22_neck_analyse.py``,
``tools/m22_neck_coupon.py``, ``tools/m28_fusion_coupon.py`` -- offline
physical-print analysis that never touches ``orca.host`` or the alignment
guard); if that ever changes, this test's own failure is the signal that the
fix would move their geometry.

``tests/fixtures/m35_corpus/`` (plain NACA4 lofts plus channel-bearing
wings) is treated as PART OF THE WING FAMILY,
not filed alongside the coupon/sweep off-centre group, because it measurably IS
centred -- every file measures at (0, 0) to within 1.1e-3 mm, well inside
``WING_CENTERING_TOLERANCE_MM`` -- and it goes through the same
MeshSectioner-centring convention (``build_loft_stations``'s
root-station-bbox-centring shift, shared with ``tests/fixtures/wing/``) as the
fixtures this assertion already protects. One honest distinction from the
``wing/`` directory proper: ``m35_corpus`` is NOT evidenced through
``capability.py``'s real host path with a ``model_object`` -- it is exercised
only through the OFFLINE sweep harness (``tools/m34_rotation_equivalence.py``'s
own centring primitives, replicating ``_build_mesh_sectioner``'s
identity-transform special case directly, never through ``orca.host``), the
same offline-only status the ``twist``/``taper``/``tip``/``airfoil`` group
below already has. That does not change which bucket centring belongs in: a
wing corpus meant to exercise the sweep path must actually BE centred for that
path's own centring assumptions to mean anything, so this is recorded as a
wing-family assertion, not a bookkeeping convenience.
"""

import glob
import os
import struct
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIXTURES_DIR = os.path.join(REPO_ROOT, "tests", "fixtures")

# How centred is "centred" for the wing family: tight enough to catch a
# genuine off-centre authoring mistake, loose enough for STL float32
# round-trip noise (every wing fixture measures within ~5e-4 mm of this).
WING_CENTERING_TOLERANCE_MM = 0.01

# Fixtures measured NOT centred at (0, 0), with their measured XY centre
# recorded here so a change to them is caught rather than silently
# re-tolerated. Not a defect in these fixtures -- they were authored for
# offline coupon/neck analysis, where the mesh's own absolute placement is
# irrelevant (see this module's docstring for exactly what consumes them).
KNOWN_OFF_CENTRE_FIXTURES = {
    "coupon/m22_neck_coupon.stl": (45.0, 0.0),
    "coupon/m28_fusion_gap_0p000mm.stl": (20.15, 0.0),
    "coupon/m28_fusion_gap_0p011mm.stl": (20.15, 0.0),
    "coupon/m28_fusion_gap_0p021mm.stl": (20.15, 0.0),
    "coupon/m28_fusion_gap_0p032mm.stl": (20.15, 0.0),
    "coupon/m28_fusion_gap_0p042mm.stl": (20.15, 0.0),
    "coupon/m28_fusion_gap_0p063mm.stl": (20.15, 0.0),
    "coupon/m28_fusion_gap_0p105mm.stl": (20.15, 0.0),
    "sweep/naca2412_sweep_c60_s60_le20.stl": (10.000001, 0.0),
}
OFF_CENTRE_TOLERANCE_MM = 0.001

# EVERY remaining real STL fixture's measured XY bbox centre (mm), so nothing
# in tests/fixtures/ is left unclassified -- every real STL
# under tests/fixtures. Most measure at/near (0, 0) (the taper/tip/airfoil
# family); one -- ``twist`` -- measures a small but genuinely non-zero Y centre
# (0.29 mm), smaller than the coupon/sweep group but still real and worth a
# name rather than silent tolerance. None of these are reachable through
# capability.py's real path either (only consumed by aircraft_core unit tests
# working from the SAME analytic parameters, never by loading these STL files
# through ``_build_mesh_sectioner`` -- see
# ``tests/integration/test_m18_mesh_transform.py``, which builds its own
# synthetic meshes, and ``tests/integration/test_m18_model_determinism.py``,
# which loads real STL files but drives ``MeshSectioner`` directly, never
# ``orca.host``).
OTHER_FIXTURE_CENTERS_MM = {
    "airfoil/naca2412_c60_s60.stl": (0.0, 0.0),
    "coupon/coupon_20x20x15.stl": (0.0, 0.0),
    "coupon/coupon_20x20x60.stl": (0.0, 0.0),
    "taper/naca2412_taper_r60_t30_s60.stl": (0.0, 0.0),
    "tip/naca2412_tip_r60_t3_s60.stl": (0.0, 0.0),
    "twist/naca2412_twist_c60_s60_w6.stl": (-0.000328, 0.290152),
    # 518 triangles cut directly from a real wing mesh (see
    # ``tests/unit/test_mesh_section_retry.py::RealGrazedVertexTests``),
    # not authored at the origin like the synthetic fixtures above and not
    # one of the deliberately off-centre coupon/sweep group either (it is
    # not a coupon/neck/fusion offline-analysis fixture, and it is not
    # asserted-reachable-only-offline the way that group is -- see
    # ``KNOWN_OFF_CENTRE_FIXTURES``'s own reachability test, which this
    # fixture would fail since it IS referenced from a unit test). It is a
    # genuine SLAB -- a wing-derived cutout, not a whole wing and not a
    # coupon -- so it belongs in this catch-all "measured, not asserted
    # near-zero" group rather than either of the other two.
    "m36_grazed_vertex.stl": (-7.319232, 18.817734),
    # the slab reproducing an ACTUAL in-host layer-562 failure, cut
    # from Part Studio 2 - Part 1(6).stl in the reconstructed host frame. Same
    # group and same reasoning as the fixture above.
    "m36_kai_layer562_z112p5.stl": (78.583399, 0.022360),
    # Fuselage-mode prototype fixtures. Measured
    # directly by this file's own _read_binary_stl_xy_bbox_center -- not one of
    # the wing/coupon/sweep families and not asserted-centred, just recorded so
    # a future change to the mesh is caught. Fuselage work itself is parked
    # (automatic keep-out identification was judged too hard to trust;
    # stiffening is modelled in CAD), but that does not mean
    # these meshes are inert: no test.py file references any of the three
    # filenames below except this one (checked by grep of tests/), yet
    # fuselage_pod_8gon.stl WAS sliced in-host
    # with a real model_object through capability.py's path -- unlike the
    # coupon/sweep group above, which is asserted unreachable through that
    # path. This mesh's measured -1.04 mm Y centre is not merely a bookkeeping
    # number: the mesh-centring step (identity volume/instance transform case)
    # subtracts the raw mesh's own XY bbox centre before cross-sectioning, so
    # if/when it processes this raw mesh it moves the derived cross-section by
    # about +1.04 mm in Y relative to raw STL coordinates. An offline tool that
    # sections the raw STL directly instead (e.g.
    # tools/spar_channel_policy_capability_sweep.py, which produced via
    # read_binary_stl with no centring step) does not apply that shift unless
    # it separately replicates the centring itself.
    "fuselage_c_section_proto.stl": (-0.0000057, -1.041884),
    "fuselage_pod_8gon.stl": (0.0, -1.041884),
    "fuselage_pod_proto.stl": (0.0, -1.041884),
}
OTHER_FIXTURE_TOLERANCE_MM = 0.001

# Directories treated as the WING FAMILY (asserted centred at the origin,
# not merely recorded off-centre) -- see the module docstring's
# "m35_corpus" paragraph for why that corpus joins ``wing`` here rather than
# being filed with the coupon/sweep off-centre group.
WING_FAMILY_DIRS = ("wing", "m35_corpus")


def _wing_family_stls():
    paths = []
    for d in WING_FAMILY_DIRS:
        paths.extend(glob.glob(os.path.join(FIXTURES_DIR, d, "*.stl")))
    return sorted(paths)


def _read_binary_stl_xy_bbox_center(path):
    """The raw mesh's own XY bounding-box centre, straight from the STL
    file -- the IDENTITY-transform special case of
    ``capability.py._build_mesh_sectioner``'s centring step (no volume or
    instance rotation/scale/mirror to apply first). Deliberately a second,
    independent, minimal implementation rather than importing
    ``tools/m18_mesh_probe.read_binary_stl`` and welding vertices: welding
    is irrelevant to a bounding box (every vertex, shared or not, still
    contributes the same min/max), and a bbox-only reader is easier to
    trust by inspection for a test whose entire job is catching a
    measurement bug.
    """
    with open(path, "rb") as fh:
        data = fh.read()
    count = struct.unpack("<I", data[80:84])[0]
    x_min = x_max = y_min = y_max = None
    off = 84
    for _ in range(count):
        vals = struct.unpack("<12f", data[off:off + 48])
        off += 50
        for k in (3, 6, 9):
            x, y = vals[k], vals[k + 1]
            if x_min is None or x < x_min:
                x_min = x
            if x_max is None or x > x_max:
                x_max = x
            if y_min is None or y < y_min:
                y_min = y
            if y_max is None or y > y_max:
                y_max = y
    return (x_min + x_max) / 2.0, (y_min + y_max) / 2.0


def _all_fixture_stls():
    return sorted(
        glob.glob(os.path.join(FIXTURES_DIR, "**", "*.stl"), recursive=True)
    )


class AllFixturesMeasuredTests(unittest.TestCase):
    """Sanity: this test actually found fixtures, and every wing/coupon/
    sweep file this module talks about by name is among them -- so a future
    fixture reorganisation that silently stops collecting them fails loudly
    here rather than this test quietly asserting nothing."""

    def test_fixtures_directory_is_not_empty(self):
        paths = _all_fixture_stls()
        self.assertGreater(len(paths), 0, "no STL fixtures found under {}".format(FIXTURES_DIR))

    def test_every_known_off_centre_fixture_still_exists(self):
        found = {
            os.path.relpath(p, FIXTURES_DIR).replace(os.sep, "/")
            for p in _all_fixture_stls()
        }
        for rel in KNOWN_OFF_CENTRE_FIXTURES:
            self.assertIn(
                rel, found,
                "{} is in KNOWN_OFF_CENTRE_FIXTURES but no longer exists -- "
                "update this test".format(rel),
            )


class WingFamilyFixturesAreCentredTests(unittest.TestCase):
    """``tests/fixtures/wing/`` is the ONLY directory the host-alignment path has
    real IN-HOST evidence behind
    (``tests/integration/test_m18_model_determinism.py``) and must stay centred
    at (0, 0) to within measurement noise, since that is the premise the fix's
    own residual figures (worst term 3.7e-03 mm) were measured against. A
    regression here means those figures do not describe the fixtures they
    were measured on.

    ``tests/fixtures/m35_corpus/`` is checked by the SAME assertion (see
    ``WING_FAMILY_DIRS``) even though it lacks that in-host evidence -- it
    measurably IS centred, and centring is what this class actually checks,
    not "has an m18_inhost_*.json entry". See the module docstring's
    "m35_corpus" paragraph for the full reasoning.
    """

    def test_every_wing_family_stl_is_centred_at_origin(self):
        wing_paths = _wing_family_stls()
        self.assertGreater(len(wing_paths), 0, "no wing-family *.stl fixtures found")
        for path in wing_paths:
            with self.subTest(fixture=os.path.basename(path)):
                cx, cy = _read_binary_stl_xy_bbox_center(path)
                self.assertLess(
                    abs(cx), WING_CENTERING_TOLERANCE_MM,
                    "{}: X centre {} mm is not near 0 -- the centring fix would "
                    "MOVE this fixture's derived cross-section by that "
                    "amount".format(os.path.basename(path), cx),
                )
                self.assertLess(
                    abs(cy), WING_CENTERING_TOLERANCE_MM,
                    "{}: Y centre {} mm is not near 0 -- the centring fix would "
                    "MOVE this fixture's derived cross-section by that "
                    "amount".format(os.path.basename(path), cy),
                )


class EveryOtherFixtureIsClassifiedTests(unittest.TestCase):
    """No fixture under ``tests/fixtures/`` is left unmeasured: the wing
    family is checked for centring above, the coupon/sweep family below,
    and every other real STL (airfoil/taper/tip/twist) is pinned here,
    including ``twist``'s own small but genuine 0.29 mm Y offset -- not
    zero, not large enough to be one of the deliberately off-centre
    coupon/sweep fixtures.
    """

    def test_all_fixture_paths_are_accounted_for(self):
        accounted = set(KNOWN_OFF_CENTRE_FIXTURES) | set(OTHER_FIXTURE_CENTERS_MM)
        wing_paths = {
            os.path.relpath(p, FIXTURES_DIR).replace(os.sep, "/")
            for p in _wing_family_stls()
        }
        all_paths = {
            os.path.relpath(p, FIXTURES_DIR).replace(os.sep, "/")
            for p in _all_fixture_stls()
        }
        unaccounted = all_paths - accounted - wing_paths
        self.assertEqual(
            unaccounted, set(),
            "fixture(s) added under tests/fixtures/ with no recorded XY "
            "centre in this test file: {}".format(sorted(unaccounted)),
        )

    def test_other_fixtures_match_their_recorded_centre(self):
        for rel, (expected_cx, expected_cy) in OTHER_FIXTURE_CENTERS_MM.items():
            path = os.path.join(FIXTURES_DIR, rel.replace("/", os.sep))
            with self.subTest(fixture=rel):
                cx, cy = _read_binary_stl_xy_bbox_center(path)
                self.assertAlmostEqual(cx, expected_cx, delta=OTHER_FIXTURE_TOLERANCE_MM)
                self.assertAlmostEqual(cy, expected_cy, delta=OTHER_FIXTURE_TOLERANCE_MM)


class OffCentreFixturesAreRecordedNotHiddenTests(unittest.TestCase):
    """The coupon/sweep fixtures ARE off-centre -- this pins the measured
    value (so a change to the fixture is caught) rather than silently
    tolerating drift or asserting a false "everything is centred" claim.
    """

    def test_known_off_centre_fixtures_match_their_recorded_centre(self):
        for rel, (expected_cx, expected_cy) in KNOWN_OFF_CENTRE_FIXTURES.items():
            path = os.path.join(FIXTURES_DIR, rel.replace("/", os.sep))
            with self.subTest(fixture=rel):
                cx, cy = _read_binary_stl_xy_bbox_center(path)
                self.assertAlmostEqual(cx, expected_cx, delta=OFF_CENTRE_TOLERANCE_MM)
                self.assertAlmostEqual(cy, expected_cy, delta=OFF_CENTRE_TOLERANCE_MM)

    def test_off_centre_fixtures_are_not_reachable_through_the_host_path(self):
        """Documents, and checks as best this repository can, that the
        off-centre fixtures are consumed only by standalone offline tools
        (never through ``capability.py``'s ``_build_mesh_sectioner`` with a
        real ``model_object``) -- so the centring step does not silently change
        their behaviour anywhere it is actually exercised today. Greps
        ``tests/`` for the fixture's own filename; a hit outside the
        offline-tool set below means this claim needs re-checking, not
        silently continuing to pass.
        """
        offline_only_consumers = {
            "coupon/m22_neck_coupon.stl",
            "coupon/m28_fusion_gap_0p000mm.stl",
            "coupon/m28_fusion_gap_0p011mm.stl",
            "coupon/m28_fusion_gap_0p021mm.stl",
            "coupon/m28_fusion_gap_0p032mm.stl",
            "coupon/m28_fusion_gap_0p042mm.stl",
            "coupon/m28_fusion_gap_0p063mm.stl",
            "coupon/m28_fusion_gap_0p105mm.stl",
            "sweep/naca2412_sweep_c60_s60_le20.stl",
        }
        self.assertEqual(
            set(KNOWN_OFF_CENTRE_FIXTURES), offline_only_consumers,
            "KNOWN_OFF_CENTRE_FIXTURES and the reachability claim above have "
            "drifted apart -- update both together",
        )
        tests_dir = os.path.join(REPO_ROOT, "tests")
        this_file = os.path.abspath(__file__)
        for rel in offline_only_consumers:
            filename = os.path.basename(rel)
            hits = []
            for dirpath, _dirnames, filenames in os.walk(tests_dir):
                if "__pycache__" in dirpath:
                    continue
                for fn in filenames:
                    if not fn.endswith(".py"):
                        continue
                    full = os.path.join(dirpath, fn)
                    if os.path.abspath(full) == this_file:
                        # This module names every off-centre fixture in its
                        # own KNOWN_OFF_CENTRE_FIXTURES literal, which is not
                        # "using" them through the host path -- excluded so
                        # the check inspects every OTHER test file honestly.
                        continue
                    with open(full, "r", encoding="utf-8") as handle:
                        if filename in handle.read():
                            hits.append(os.path.relpath(full, REPO_ROOT))
            self.assertEqual(
                hits, [],
                "{} is referenced from {} -- if that reference now drives "
                "capability.py's real model-mesh path with a model_object, this "
                "fixture's off-centre geometry means the fix MOVES its "
                "derived socket; re-verify before trusting this test's "
                "silence".format(filename, hits),
            )


if __name__ == "__main__":
    unittest.main()
