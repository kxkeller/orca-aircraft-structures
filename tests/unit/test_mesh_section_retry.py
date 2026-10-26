"""Regression tests for the retry-nudge added to ``MeshSectioner.section_at`` (see
that method's docstring and ``_RETRY_NUDGES_MM`` in
``aircraft_core/mesh_section.py``).

Three properties are pinned here, each with its own fixture:

1. A plane that grazes a loft station ring closely enough to fail
   ``_section_at_exact`` with a "dangling endpoint" ``ValueError`` is
   recovered by ``section_at``'s retry.
2. A genuinely non-manifold mesh (a real hole at the query height) still
   raises through ``section_at`` -- the retry must never mask an actual
   mesh defect.
3. A ``ValueError`` whose message does not mention "dangling endpoint"
   propagates immediately, with no retry attempted.

Plus the REAL-GEOMETRY case, on a minimized fixture cut from the mesh the bug
was found on -- see ``RealGrazedVertexTests`` rule 4 ("every geometry bug earns
a minimized permanent fixture").
"""

import math
import os
import struct
import unittest

from aircraft_core.mesh_section import DanglingEndpointError, MeshSectioner


def _regular_polygon(n, cx, cy, r):
    pts = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        pts.append((cx + r * math.cos(theta), cy + r * math.sin(theta)))
    return pts


def _polygon_tube(polygon, z0, z1):
    """Vertical tube walls (no caps), matching ``test_mesh_section.py``."""
    n = len(polygon)
    bottom = [(x, y, z0) for x, y in polygon]
    top = [(x, y, z1) for x, y in polygon]
    vertices = bottom + top
    triangles = []
    for idx in range(n):
        a = idx
        b = (idx + 1) % n
        ta = a + n
        tb = b + n
        triangles.append((a, b, tb))
        triangles.append((a, tb, ta))
    return vertices, triangles


class MeshSectionerRetryRecoversGrazedVertexTests(unittest.TestCase):
    """POSITIVE case: a lofted tube whose middle station ring has been
    perturbed by float32-STL-scale per-vertex Z noise (order 1e-4 to 1e-6
    mm, the same order as ``self.epsilon_mm`` at this mesh's ~75 mm
    coordinate scale -- see module docstring item 2 in
    ``mesh_section.py``), queried at a Z that grazes that noisy ring
    closely enough to leave 2 dangling endpoints at the primary
    epsilon-nudged plane.

    This exact fixture (16-gon, these specific per-vertex Z offsets, this
    specific query Z) was found by randomized search over jittered lofted
    tubes and its failure/recovery was verified directly before being
    hardcoded here -- see the class-level assertions below, which pin
    both the ``_section_at_exact`` failure AND the ``section_at`` recovery
    so this stays a real reproduction, not an assumed one.
    """

    N = 16
    RADIUS = 75.0
    # Per-vertex Z offset for each of the 16 middle-ring vertices, station
    # nominally at z=60.0. Magnitudes are ~1e-4 to ~1e-7 mm -- comparable to
    # this mesh's derived epsilon_mm (~1.43e-4 mm at 75 mm coordinate
    # scale), i.e. exactly the STL float32-rounding regime the module
    # docstring describes, not an arbitrarily large perturbation.
    MID_RING_Z_OFFSETS = (
        60.00021453815357 - 60.0,
        60.0001158111397 - 60.0,
        59.99998230200323 - 60.0,
        59.99994298388758 - 60.0,
        59.99999471473451 - 60.0,
        60.000007510862254 - 60.0,
        60.000032444942256 - 60.0,
        59.999961966097004 - 60.0,
        59.99999921611029 - 60.0,
        59.99999059360317 - 60.0,
        59.999979147258664 - 60.0,
        59.99995087513998 - 60.0,
        60.00001327077802 - 60.0,
        59.99997031086158 - 60.0,
        60.00000883341197 - 60.0,
        59.99998562652566 - 60.0,
    )
    QUERY_Z = 60.00003833586027

    def _build_sectioner(self):
        n = self.N
        bottom = _regular_polygon(n, 0.0, 0.0, self.RADIUS)
        mid_xy = _regular_polygon(n, 0.0, 0.0, self.RADIUS)
        top = _regular_polygon(n, 0.0, 0.0, self.RADIUS)

        vertices = []
        vertices.extend((x, y, 0.0) for x, y in bottom)
        vertices.extend(
            (x, y, 60.0 + self.MID_RING_Z_OFFSETS[k])
            for k, (x, y) in enumerate(mid_xy)
        )
        vertices.extend((x, y, 120.0) for x, y in top)

        triangles = []
        for base_a, base_b in ((0, n), (n, 2 * n)):
            for idx in range(n):
                a = base_a + idx
                b = base_a + (idx + 1) % n
                ta = base_b + idx
                tb = base_b + (idx + 1) % n
                triangles.append((a, b, tb))
                triangles.append((a, tb, ta))

        return MeshSectioner(vertices, triangles, z_bin_mm=1.0)

    def test_section_at_exact_fails_with_dangling_endpoint(self):
        # Confirms the fixture genuinely reproduces the failure this
        # retry exists to recover from, rather than assuming it.
        sectioner = self._build_sectioner()
        with self.assertRaises(ValueError) as ctx:
            sectioner._section_at_exact(self.QUERY_Z)
        self.assertIn("dangling endpoint", str(ctx.exception))

    def test_section_at_recovers_via_retry(self):
        sectioner = self._build_sectioner()
        loops = sectioner.section_at(self.QUERY_Z)
        self.assertEqual(len(loops), 1)
        self.assertFalse(loops[0].is_interior)
        min_x, min_y, max_x, max_y = loops[0].bbox
        self.assertAlmostEqual(max_x - min_x, 2.0 * self.RADIUS, delta=1.0)
        self.assertAlmostEqual(max_y - min_y, 2.0 * self.RADIUS, delta=1.0)


def _box_mesh():
    """A closed 20x20x10 box, as a (vertices, triangles) pair."""
    square = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
    vertices, triangles = _polygon_tube(square, 0.0, 10.0)
    n = len(square)
    triangles.append((0, 1, 2))
    triangles.append((0, 2, 3))
    triangles.append((n, n + 2, n + 1))
    triangles.append((n, n + 3, n + 2))
    return vertices, triangles


def _closed_box_sectioner():
    vertices, triangles = _box_mesh()
    return MeshSectioner(vertices, triangles, z_bin_mm=1.0)


def _holed_box_sectioner():
    """The same box with one wall triangle deleted -- a real hole through
    the wall spanning the full 0..10 height, including z=5.0."""
    vertices, triangles = _box_mesh()
    del triangles[0]
    return MeshSectioner(vertices, triangles, z_bin_mm=1.0)


class MeshSectionerRetryDoesNotMaskRealHoleTests(unittest.TestCase):
    """NEGATIVE / most important case: a genuinely non-manifold mesh (a
    closed box with one wall triangle deleted, opening a real hole that
    spans the query height) must still raise ``ValueError`` from
    ``section_at``. A real hole leaves dangling ends at every nearby
    plane, so every retry offset fails too and the original error must
    still surface -- the retry must never paper over an actual mesh
    defect.
    """

    def test_section_at_still_raises_for_a_real_hole(self):
        square = [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)]
        vertices, triangles = _polygon_tube(square, 0.0, 10.0)
        n = len(square)
        triangles.append((0, 1, 2))
        triangles.append((0, 2, 3))
        triangles.append((n, n + 2, n + 1))
        triangles.append((n, n + 3, n + 2))
        # Delete one wall triangle: a real hole through the wall spanning
        # the full 0..10 height, including the z=5.0 query below.
        del triangles[0]

        sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)
        with self.assertRaises(ValueError) as ctx:
            sectioner.section_at(5.0)
        self.assertIn("dangling endpoint", str(ctx.exception))


class MeshSectionerRetryPassesThroughUnrelatedErrorsTests(unittest.TestCase):
    """PASS-THROUGH case: a ``ValueError`` whose message does not mention
    "dangling endpoint" is not a retry candidate and must propagate
    immediately, with ``_section_at_exact`` called exactly once.
    """

    def test_unrelated_value_error_propagates_without_retry(self):
        vertices, triangles = _polygon_tube(
            [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)], 0.0, 10.0
        )
        sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)

        call_count = [0]
        original = MeshSectioner._section_at_exact

        def _fake_section_at_exact(self, z):
            call_count[0] += 1
            raise ValueError("mesh cross-section: some unrelated failure")

        MeshSectioner._section_at_exact = _fake_section_at_exact
        try:
            with self.assertRaises(ValueError) as ctx:
                sectioner.section_at(5.0)
            self.assertNotIn("dangling endpoint", str(ctx.exception))
            self.assertIn("unrelated failure", str(ctx.exception))
            self.assertEqual(call_count[0], 1)
        finally:
            MeshSectioner._section_at_exact = original


class RetryKeysOnTypeNotMessageTests(unittest.TestCase):
    """The retry must be driven by the EXCEPTION TYPE, not message text.

    The retry keys on a typed ``DanglingEndpointError``, not a
    ``"dangling endpoint" in str(exc)`` substring match. These two tests make
    that load-bearing: a substring match would fail both.
    """

    def _box(self):
        return _closed_box_sectioner()

    def test_dangling_type_is_retried_even_with_an_unrecognisable_message(self):
        """A DanglingEndpointError whose text says nothing about dangling
        endpoints is STILL retried. A substring match would let it propagate on the
        first attempt."""
        sectioner = self._box()
        calls = []
        original = MeshSectioner._section_at_exact

        def _fake(self, z):
            calls.append(z)
            raise DanglingEndpointError("assembly failed; text says nothing")

        MeshSectioner._section_at_exact = _fake
        try:
            with self.assertRaises(DanglingEndpointError):
                sectioner.section_at(5.0)
            self.assertGreater(
                len(calls), 1,
                "expected the retry to fire on the TYPE; it did not",
            )
            self.assertEqual(calls[0], 5.0)
        finally:
            MeshSectioner._section_at_exact = original

    def test_plain_value_error_naming_dangling_endpoints_is_not_retried(self):
        """The mirror image: a plain ValueError whose text DOES contain
        'dangling endpoint' is NOT retried. A substring match would retry it."""
        sectioner = self._box()
        calls = []
        original = MeshSectioner._section_at_exact

        def _fake(self, z):
            calls.append(z)
            raise ValueError("produced 2 dangling endpoint(s) somewhere else")

        MeshSectioner._section_at_exact = _fake
        try:
            with self.assertRaises(ValueError) as ctx:
                sectioner.section_at(5.0)
            self.assertNotIsInstance(ctx.exception, DanglingEndpointError)
            self.assertEqual(
                len(calls), 1,
                "a plain ValueError must not be retried, whatever it says",
            )
        finally:
            MeshSectioner._section_at_exact = original


class AssemblyRaisesTheTypedErrorTests(unittest.TestCase):
    def test_real_hole_raises_DanglingEndpointError_specifically(self):
        """The raise site itself must use the type, or nothing above works."""
        sectioner = _holed_box_sectioner()
        with self.assertRaises(DanglingEndpointError):
            sectioner.section_at(5.0)


FIXTURES = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "fixtures"
)
GRAZED_VERTEX = "m36_grazed_vertex.stl"


def _read_binary_stl(path):
    """A binary-STL reader, deliberately local to this unit test.

    ``tools/m18_mesh_probe.py`` has one, but ``tools/`` is only on the path
    for integration tests; depending on it here would make this unit test
    fail for a reason that has nothing to do with what it asserts. Welds
    vertices on exact float32 equality, which is what the fixture stores.
    """
    with open(path, "rb") as handle:
        handle.read(80)
        (count,) = struct.unpack("<I", handle.read(4))
        index = {}
        verts = []
        tris = []
        for _ in range(count):
            data = struct.unpack("<12fH", handle.read(50))
            tri = []
            for k in (3, 6, 9):
                point = (data[k], data[k + 1], data[k + 2])
                if point not in index:
                    index[point] = len(verts)
                    verts.append(point)
                tri.append(index[point])
            tris.append(tuple(tri))
    return verts, tris
GRAZED_VERTEX_Z_MM = 168.850
KAI_L562 = "m36_kai_layer562_z112p5.stl"
KAI_L562_Z_MM = 112.5


class RealGrazedVertexTests(unittest.TestCase):
    """REAL GEOMETRY, not synthetic. The permanent minimized fixture for the bug
    was written for.

    ``m36_grazed_vertex.stl`` is the 518 triangles of ``Part Studio
    2 - Part 1(5).stl`` (in the canonical frame) whose Z range reaches within
    0.25 mm of z=168.850 -- the plane where TWO VERTICES fall within
    ``epsilon_mm`` of the effective plane (distances +2.692e-05 and -3.601e-06
    against an epsilon of 2.691e-04). Each is the lone vertex of its incident
    triangles on one side, so both crossing edges snap to it and the segment
    collapses to zero length; the contour's continuation through that vertex is
    never built.

    486 candidates -> 391 segments -> 12 dropped -> 379 -> 2 dangling.

    The self-collide drop is NOT the cause. Disable it and all 391 segments
    assemble into 9 unbalanced buckets (degrees 4,4,4,4,4,4,4,5,7) -- worse.
    The two buckets that dangle were already unbalanced at degree 7 and 5
    before the filter ran.

    The triangles are 82.9 to 89.9 degrees from parallel -- near-vertical
    walls, not faces nearly parallel to the slice plane.

    The slab reproduces the failure with the same 379 segments and
    recovers to the same two loops (52.1924 mm2 and 2168.4339 mm2) as the
    full mesh, and it survives the float32 round-trip through the STL
    file.

    The 52 mm2 loop is the external joiner feature and the 2168 mm2 loop is the
    wing: the grazed ring is on the FEATURE. That is why this surfaced and
    never on a pure-aerofoil wing.
    """

    def _sectioner(self):
        path = os.path.join(FIXTURES, GRAZED_VERTEX)
        if not os.path.isfile(path):
            self.skipTest("fixture {!r} not present".format(GRAZED_VERTEX))
        return MeshSectioner(*_read_binary_stl(path))

    def test_exact_section_fails_on_the_grazed_vertex(self):
        sectioner = self._sectioner()
        with self.assertRaises(DanglingEndpointError) as ctx:
            sectioner._section_at_exact(GRAZED_VERTEX_Z_MM)
        message = str(ctx.exception)
        self.assertIn("dangling endpoint", message)
        # Pin the exact counts so fixture drift is LOUD. If this fixture is
        # ever recut and the numbers move, the mechanism recorded in this docstring does not
        # describe it and the docs must be re-measured rather than the
        # assertion relaxed.
        self.assertIn("379 segment(s) produced 2 dangling", message)

    def test_retry_recovers_the_true_section(self):
        """The whole point: real geometry driven through the retry.

        The corpus sweep does NOT exercise this path -- it samples the
        0.2 mm layer grid and z=168.850 is not on it, which is exactly why
        the defect reached the host before it reached a test.
        """
        sectioner = self._sectioner()
        loops = sectioner.section_at(GRAZED_VERTEX_Z_MM)
        areas = sorted(abs(lp.signed_area) for lp in loops)
        self.assertEqual(len(loops), 2)
        self.assertAlmostEqual(areas[0], 52.1924, places=3)
        self.assertAlmostEqual(areas[1], 2168.4339, places=3)

    def test_the_nudge_is_at_most_one_percent_of_a_layer(self):
        """Guards the magnitude claim rests on. A future widening of the tuple to
        escape a stubborn part is how this decision would decay into the silent
        fallback it was written to avoid."""
        nudges = MeshSectioner._RETRY_NUDGES_MM
        self.assertTrue(nudges, "the retry tuple must not be empty")
        self.assertLessEqual(max(abs(dz) for dz in nudges), 0.002)
        self.assertTrue(
            any(dz > 0 for dz in nudges) and any(dz < 0 for dz in nudges),
            "nudges must probe BOTH sides of the plane: a grazed vertex is "
            "escaped in one direction and not necessarily the other",
        )


class SecondaryFailureFromANudgedAttemptDoesNotReplaceTheOriginalTests(unittest.TestCase):
    """DIRECTIVE A. A nudged attempt that raises a DIFFERENT ``ValueError``
    (e.g. ``_check_empty_result_is_legitimate`` firing at the nudged plane)
    must never replace the original ``DanglingEndpointError`` -- the caller
    must still see the ORIGINAL error, naming the Z it actually requested.
    """

    def test_caller_sees_the_original_dangling_error_not_the_secondary_one(self):
        sectioner = _closed_box_sectioner()
        requested_z = 5.0

        def _fake(self, z):
            if z == requested_z:
                raise DanglingEndpointError(
                    "original: z={:.6f} mm dangling".format(z)
                )
            # Every nudged attempt fails a DIFFERENT way.
            raise ValueError(
                "secondary: z={:.6f} mm produced ZERO loops".format(z)
            )

        original = MeshSectioner._section_at_exact
        MeshSectioner._section_at_exact = _fake
        try:
            with self.assertRaises(DanglingEndpointError) as ctx:
                sectioner.section_at(requested_z)
            message = str(ctx.exception)
            self.assertIn("original", message)
            self.assertIn("{:.6f}".format(requested_z), message)
            self.assertNotIn("secondary", message)
        finally:
            MeshSectioner._section_at_exact = original


class LastNudgeMmIsObservableTests(unittest.TestCase):
    """DIRECTIVE B.1. ``MeshSectioner.last_nudge_mm`` reflects the offset
    actually applied by the most recent successful ``section_at`` call:
    non-zero after a recovered retry, ``0.0`` after a clean unnudged
    section.
    """

    def test_last_nudge_mm_is_nonzero_after_a_recovered_retry(self):
        path = os.path.join(FIXTURES, GRAZED_VERTEX)
        if not os.path.isfile(path):
            self.skipTest("fixture {!r} not present".format(GRAZED_VERTEX))
        sectioner = MeshSectioner(*_read_binary_stl(path))
        sectioner.section_at(GRAZED_VERTEX_Z_MM)
        self.assertNotEqual(sectioner.last_nudge_mm, 0.0)

    def test_last_nudge_mm_is_zero_after_a_clean_unnudged_section(self):
        sectioner = _closed_box_sectioner()
        sectioner.section_at(5.0)
        self.assertEqual(sectioner.last_nudge_mm, 0.0)


class KaiInHostLayer562Tests(unittest.TestCase):
    """An in-host OrcaSlicer failure, reproduced offline.

    In-host report: ``Aircraft structure invalid at layer 562
    (Z=112.60 mm): cross-sectioning the model mesh at Z=112.5000 mm
    failed ... 344 segment(s) produced 2 dangling endpoint(s)``.

    A hull-based fit recovers the host's orientation exactly: the mesh is
    ``Part Studio 2 - Part 1(6).stl`` rotated so the root face lies on the bed
    with a ~3.5 degree tilt, with scale 1.000005 (i.e. none). Refined, it
    reproduces the reported bounds x=214.991957, y=16.898478,
    z=235.001160 to **1e-13 mm**.

    Sectioning that reconstruction on the host's own layer grid
    (``slice_z = i*0.2 - 0.1``) fails at exactly ONE plane out of 1175:
    **z=112.5000**, the reported one. This fixture is the 442 triangles
    within 0.25 mm of it.

    NOT a byte-identical reproduction: the host built 344 segments at this plane,
    the full reconstruction 348, and this trimmed slab 345. A bounding box
    does not pin a frame uniquely, so a small residual rotation remains.
    The Z, the failure mode and the dangling-endpoint count all match
    exactly.
    """

    def _sectioner(self):
        path = os.path.join(FIXTURES, KAI_L562)
        if not os.path.isfile(path):
            self.skipTest("fixture {!r} not present".format(KAI_L562))
        return MeshSectioner(*_read_binary_stl(path))

    def test_it_reproduces_the_in_host_failure(self):
        sectioner = self._sectioner()
        with self.assertRaises(DanglingEndpointError) as ctx:
            sectioner._section_at_exact(KAI_L562_Z_MM)
        message = str(ctx.exception)
        self.assertIn("z=112.500000 mm", message)
        self.assertIn("2 dangling endpoint(s)", message)

    def test_the_retry_recovers_it_with_the_spar_bore_intact(self):
        """The retry recovers the layer that failed the in-host slice, with the
        spar bore intact.

        The interior loop assertion matters on its own -- _channels_for_layer
        fails closed when the spar channel cannot be recovered, so a
        section that returned only the outer boundary would still break
        the slice.
        """
        sectioner = self._sectioner()
        loops = sectioner.section_at(KAI_L562_Z_MM)
        self.assertEqual(len(loops), 2)
        self.assertNotEqual(sectioner.last_nudge_mm, 0.0)
        self.assertEqual(sum(1 for lp in loops if lp.is_interior), 1)
        areas = sorted(abs(lp.signed_area) for lp in loops)
        self.assertAlmostEqual(areas[0], 60.6314, places=3)
        self.assertAlmostEqual(areas[1], 1464.1026, places=3)


if __name__ == "__main__":
    unittest.main()
