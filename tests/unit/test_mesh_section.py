import math
import unittest

from aircraft_core.mesh_section import MeshSectioner


def _polygon_tube(polygon, z0, z1):
    """Vertical tube walls (no caps) for a closed 2D polygon extruded from
    z0 to z1. Two triangles per quad side face, standard diagonal split.
    """
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


def _combine(*meshes):
    """Concatenate (vertices, triangles) meshes into one, offsetting
    triangle indices so each sub-mesh's vertices stay distinct.
    """
    vertices = []
    triangles = []
    offset = 0
    for verts, tris in meshes:
        vertices.extend(verts)
        triangles.extend((a + offset, b + offset, c + offset) for a, b, c in tris)
        offset += len(verts)
    return vertices, triangles


def _box_mesh(x0, y0, x1, y1, z0, z1):
    """A closed rectangular box: tube walls plus top and bottom caps."""
    square = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    vertices, triangles = _polygon_tube(square, z0, z1)
    n = len(square)
    triangles = list(triangles)
    triangles.append((0, 1, 2))
    triangles.append((0, 2, 3))
    triangles.append((n, n + 2, n + 1))
    triangles.append((n, n + 3, n + 2))
    return vertices, triangles


def _hollow_box_mesh(outer, inner, z0, z1):
    """A box wall plus an independent inner tube (the spar-channel stand-in),
    no caps -- caps are irrelevant to a mid-height section and this keeps
    the fixture minimal, matching the module docstring's "candidate
    triangles only" design.
    """
    return _combine(
        _polygon_tube(outer, z0, z1),
        _polygon_tube(inner, z0, z1),
    )


def _lofted_tube(stations):
    """A tapered tube built from several (z, polygon) stations, all sharing ONE
    vertex ring per station between the tube segment below it and the tube
    segment above it -- exactly the loft topology that produced the real bug (a
    mesh vertex exactly on a section plane is shared by every triangle on both
    sides of that station, and inconsistent classification/interpolation of
    that shared vertex is what dangled). Every ``polygon`` in ``stations`` must
    have the same vertex count.
    """
    n = len(stations[0][1])
    vertices = []
    station_base = []
    for z, poly in stations:
        station_base.append(len(vertices))
        vertices.extend((x, y, z) for x, y in poly)
    triangles = []
    for s in range(len(stations) - 1):
        base_a = station_base[s]
        base_b = station_base[s + 1]
        for idx in range(n):
            a = base_a + idx
            b = base_a + (idx + 1) % n
            ta = base_b + idx
            tb = base_b + (idx + 1) % n
            triangles.append((a, b, tb))
            triangles.append((a, tb, ta))
    return vertices, triangles


def _regular_polygon(n, cx, cy, r):
    pts = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        pts.append((cx + r * math.cos(theta), cy + r * math.sin(theta)))
    return pts


def _square(cx, cy, half):
    return [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
    ]


class MeshSectionerBoxTests(unittest.TestCase):
    def setUp(self):
        self.vertices, self.triangles = _box_mesh(0.0, 0.0, 20.0, 20.0, 0.0, 10.0)
        self.sectioner = MeshSectioner(self.vertices, self.triangles, z_bin_mm=1.0)

    def test_triangle_count_exposed(self):
        self.assertEqual(self.sectioner.triangle_count, len(self.triangles))

    def test_single_loop_at_mid_height(self):
        loops = self.sectioner.section_at(5.0)
        self.assertEqual(len(loops), 1)
        loop = loops[0]
        self.assertFalse(loop.is_interior)

    def test_bbox_matches_box_footprint(self):
        loop = self.sectioner.section_at(5.0)[0]
        min_x, min_y, max_x, max_y = loop.bbox
        self.assertAlmostEqual(min_x, 0.0, places=3)
        self.assertAlmostEqual(min_y, 0.0, places=3)
        self.assertAlmostEqual(max_x, 20.0, places=3)
        self.assertAlmostEqual(max_y, 20.0, places=3)


class MeshSectionerEpsilonNudgeDirectionTests(unittest.TestCase):
    """the epsilon nudge in ``_section_at_exact`` must not push the effective
    plane outside the mesh's own [z_min, z_max] range. A minimized permanent
    fixture standing in for the real elevon wing's near-zero-chord tip -- NOT a
    flat cap; the measured true section there is a knife-edge sliver, roughly
    6.4 um by 1.8 mm, area 0.00577 mm^2 (measured: z_max=249.500092 mm, sampled
    z=249.5 mm, 9.155e-05 mm below the top, epsilon_mm=2.974e-4 mm -- the
    default upward nudge overshot z_max by 2.06e-4 mm and raised a false "ZERO
    loops" ValueError for a Z that has a real, if narrow, cross-section).
    """

    def setUp(self):
        self.vertices, self.triangles = _box_mesh(0.0, 0.0, 20.0, 20.0, 0.0, 10.0)
        self.sectioner = MeshSectioner(self.vertices, self.triangles, z_bin_mm=1.0)

    def test_epsilon_mm_is_smaller_than_the_gap_used_below(self):
        # Sanity check on the fixture itself, so a future change to the
        # epsilon derivation doesn't silently stop exercising this bug: the
        # sampled Z must be CLOSER to z_max than epsilon_mm for the upward
        # nudge to overshoot at all.
        self.assertGreater(self.sectioner.epsilon_mm, 5e-6)

    def test_z_just_below_z_max_still_sections(self):
        # An upward nudge (eff_z = z + epsilon_mm) would land AT/ABOVE
        # z_max (10.0); no triangle spans above z_max, so the empty-result
        # check would raise ValueError for a z strictly inside (0.0, 10.0).
        # The nudge flips downward here instead and returns the true
        # (near-top) outline.
        z = 10.0 - 5e-6
        loops = self.sectioner.section_at(z)
        self.assertEqual(len(loops), 1)
        min_x, min_y, max_x, max_y = loops[0].bbox
        self.assertAlmostEqual(min_x, 0.0, places=3)
        self.assertAlmostEqual(max_x, 20.0, places=3)
        self.assertAlmostEqual(min_y, 0.0, places=3)
        self.assertAlmostEqual(max_y, 20.0, places=3)

    def test_z_just_above_z_min_already_sections_correctly(self):
        # The symmetric bottom-end case: nudging UPWARD near z_min moves eff_z
        # further INSIDE the mesh, away from z_min, which was already the
        # direction-correct choice -- no flip needed or applied here.
        # Regression guard: the upward nudge near z_min must stay correct.
        z = 0.0 + 5e-6
        loops = self.sectioner.section_at(z)
        self.assertEqual(len(loops), 1)
        min_x, min_y, max_x, max_y = loops[0].bbox
        self.assertAlmostEqual(min_x, 0.0, places=3)
        self.assertAlmostEqual(max_x, 20.0, places=3)
        self.assertAlmostEqual(min_y, 0.0, places=3)
        self.assertAlmostEqual(max_y, 20.0, places=3)


class MeshSectionerHoleTests(unittest.TestCase):
    """A box with a square hole through it -- the spar-channel stand-in
    this module exists for.
    """

    def setUp(self):
        self.outer = _square(10.0, 10.0, 10.0)  # 0..20 square
        self.hole_center = (10.0, 10.0)
        self.hole_half = 2.0
        self.inner = _square(
            self.hole_center[0], self.hole_center[1], self.hole_half
        )
        self.z0, self.z1 = 0.0, 10.0
        vertices, triangles = _hollow_box_mesh(
            self.outer, self.inner, self.z0, self.z1
        )
        self.sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)

    def test_two_loops_outer_and_interior_hole(self):
        loops = self.sectioner.section_at(5.0)
        self.assertEqual(len(loops), 2)
        interior = [loop for loop in loops if loop.is_interior]
        outer = [loop for loop in loops if not loop.is_interior]
        self.assertEqual(len(interior), 1)
        self.assertEqual(len(outer), 1)

    def test_interior_loop_centroid_and_diameter(self):
        loops = self.sectioner.section_at(5.0)
        interior = next(loop for loop in loops if loop.is_interior)
        self.assertAlmostEqual(interior.centroid[0], self.hole_center[0], places=3)
        self.assertAlmostEqual(interior.centroid[1], self.hole_center[1], places=3)
        expected_diameter = 2.0 * math.sqrt(
            (2.0 * self.hole_half) ** 2 / math.pi
        )
        self.assertAlmostEqual(
            interior.equivalent_diameter_mm, expected_diameter, places=3
        )

    def test_moving_the_hole_moves_the_detected_centroid_by_the_same_amount(self):
        # Core determinism property exists to guarantee: the plugin's recovered
        # channel position must track the CAD model exactly, not approximately.
        delta = (3.5, -1.25)
        moved_center = (
            self.hole_center[0] + delta[0],
            self.hole_center[1] + delta[1],
        )
        moved_inner = _square(moved_center[0], moved_center[1], self.hole_half)
        vertices, triangles = _hollow_box_mesh(
            self.outer, moved_inner, self.z0, self.z1
        )
        moved_sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)

        original = next(
            loop for loop in self.sectioner.section_at(5.0) if loop.is_interior
        )
        moved = next(
            loop for loop in moved_sectioner.section_at(5.0) if loop.is_interior
        )
        observed_delta = (
            moved.centroid[0] - original.centroid[0],
            moved.centroid[1] - original.centroid[1],
        )
        self.assertAlmostEqual(observed_delta[0], delta[0], places=3)
        self.assertAlmostEqual(observed_delta[1], delta[1], places=3)

    def test_changing_hole_size_changes_equivalent_diameter(self):
        bigger_half = 4.0
        bigger_inner = _square(
            self.hole_center[0], self.hole_center[1], bigger_half
        )
        vertices, triangles = _hollow_box_mesh(
            self.outer, bigger_inner, self.z0, self.z1
        )
        bigger_sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)

        original = next(
            loop for loop in self.sectioner.section_at(5.0) if loop.is_interior
        )
        bigger = next(
            loop for loop in bigger_sectioner.section_at(5.0) if loop.is_interior
        )
        self.assertGreater(
            bigger.equivalent_diameter_mm, original.equivalent_diameter_mm
        )
        expected_ratio = (2.0 * bigger_half) / (2.0 * self.hole_half)
        observed_ratio = (
            bigger.equivalent_diameter_mm / original.equivalent_diameter_mm
        )
        self.assertAlmostEqual(observed_ratio, expected_ratio, places=3)


class MeshSectionerDegenerateTests(unittest.TestCase):
    def test_vertex_on_section_plane_does_not_corrupt_the_loop(self):
        # Query exactly at z0, where the bottom cap is fully coplanar
        # (skipped, requirement 3) and every bottom-face side-wall vertex
        # sits exactly on the plane. The recovered loop should still match
        # the box footprint to within COPLANAR_EPSILON_MM's nudge.
        vertices, triangles = _box_mesh(0.0, 0.0, 20.0, 20.0, 0.0, 10.0)
        sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)
        loops = sectioner.section_at(0.0)
        self.assertEqual(len(loops), 1)
        min_x, min_y, max_x, max_y = loops[0].bbox
        self.assertAlmostEqual(min_x, 0.0, places=3)
        self.assertAlmostEqual(min_y, 0.0, places=3)
        self.assertAlmostEqual(max_x, 20.0, places=3)
        self.assertAlmostEqual(max_y, 20.0, places=3)

    def test_dangling_segments_raise_value_error_naming_z(self):
        # A single free-floating triangle crossing the plane produces
        # exactly one segment -- both endpoints dangling, no way to close
        # a loop.
        vertices = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (0.0, 0.0, 10.0)]
        triangles = [(0, 1, 2)]
        sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)
        with self.assertRaises(ValueError) as ctx:
            sectioner.section_at(5.0)
        self.assertIn("5", str(ctx.exception))

    def test_zero_z_bin_rejected(self):
        with self.assertRaises(ValueError):
            MeshSectioner([(0.0, 0.0, 0.0)], [], z_bin_mm=0.0)


class MeshSectionerRealMeshRegressionTests(unittest.TestCase):
    """Real-mesh regression: a lofted, tapered prism at REALISTIC (order
    100 mm, not order 1 mm) coordinate magnitude, sectioned exactly at an
    interior station plane where mesh vertices sit exactly on the query Z.

    This is the minimized version of the bug found by
    ``tools/m18_mesh_probe.py`` on a real 39032-facet wing STL: 39 of 599
    layers raised ``ValueError`` at exactly the wing's lofted station
    planes (every 3 mm). A unit-cube-scale test would not have caught this
    -- the failure is scale-dependent, since it stems from float32's
    (STL's on-disk precision) relative rounding error, which only becomes
    comparable to the module's matching tolerance at wing-sized
    coordinates. This mesh is built directly from float64 Python floats
    (no STL round-trip), so it does not, by itself, reproduce the float32
    provenance of the real bug; it instead pins the OTHER piece the
    epsilon choice depends on -- that sectioning exactly at a station plane whose vertices
    are exactly coplanar with the query Z assembles cleanly at realistic
    scale, which a too-small epsilon would fail even without float32 rounding.
    """

    def test_lofted_prism_sections_cleanly_at_interior_station_plane(self):
        # Five stations spanning a realistic ~120 mm span, at ~75 mm coordinate
        # magnitude (matching the wing fixture's scale), tapering linearly --
        # the same "shared vertex ring at every interior station" topology as
        # the real wing loft.
        stations = []
        for i, z in enumerate((0.0, 30.0, 60.0, 90.0, 120.0)):
            half = 75.0 - 15.0 * (z / 120.0)
            stations.append((z, _square(0.0, 0.0, half)))
        vertices, triangles = _lofted_tube(stations)
        sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)

        # Query exactly at an interior station (z=60.0): every vertex on
        # this ring sits exactly on the query plane and is shared by the
        # tube segment below AND the tube segment above -- the coplanar
        # case the epsilon nudge handles.
        loops = sectioner.section_at(60.0)
        self.assertEqual(len(loops), 1)
        expected_half = 75.0 - 15.0 * (60.0 / 120.0)
        min_x, min_y, max_x, max_y = loops[0].bbox
        self.assertAlmostEqual(min_x, -expected_half, delta=0.01)
        self.assertAlmostEqual(max_x, expected_half, delta=0.01)
        self.assertAlmostEqual(min_y, -expected_half, delta=0.01)
        self.assertAlmostEqual(max_y, expected_half, delta=0.01)

    def test_lofted_prism_sections_cleanly_at_every_interior_station(self):
        # Every interior station plane is a coincident-vertex case; none of
        # them should raise, matching the real fixture's "every station
        # plane failed" pattern (fixed, not merely one instance of it).
        stations = []
        for z in (0.0, 30.0, 60.0, 90.0, 120.0):
            half = 75.0 - 15.0 * (z / 120.0)
            stations.append((z, _square(0.0, 0.0, half)))
        vertices, triangles = _lofted_tube(stations)
        sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)

        for z in (30.0, 60.0, 90.0):
            loops = sectioner.section_at(z)
            self.assertEqual(len(loops), 1, "station z={}".format(z))

    def test_epsilon_derived_from_realistic_coordinate_magnitude(self):
        # A sanity check on the derivation itself, at the scale where it
        # matters: at ~75 mm coordinate magnitude the derived epsilon must
        # be comfortably above the worst-case float32 ULP at that
        # magnitude (~75 * 2**-23 ~= 8.94e-6 mm) and comfortably below any
        # real feature in this project (walls/channels >= 0.1 mm).
        vertices = [(-75.0, -75.0, 0.0), (75.0, 75.0, 120.0), (0.0, 0.0, 60.0)]
        sectioner = MeshSectioner(vertices, [(0, 1, 2)], z_bin_mm=1.0)
        float32_ulp_at_scale = 75.0 * (2.0 ** -23)
        self.assertGreater(sectioner.epsilon_mm, float32_ulp_at_scale)
        self.assertLess(sectioner.epsilon_mm, 0.01)


class MeshSectionerCylinderTests(unittest.TestCase):
    def test_ngon_hole_equivalent_diameter_approximates_true_diameter(self):
        # A 32-gon prism stands in for a round spar channel. Its polygonal
        # area is slightly less than a true circle's of the same
        # circumradius, so a small, explicitly stated tolerance is used
        # rather than an exact match.
        outer = _square(0.0, 0.0, 20.0)
        radius = 5.0
        inner = _regular_polygon(32, 0.0, 0.0, radius)
        z0, z1 = 0.0, 10.0
        vertices, triangles = _hollow_box_mesh(outer, inner, z0, z1)
        sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)

        loop = next(
            loop for loop in sectioner.section_at(5.0) if loop.is_interior
        )
        true_diameter = 2.0 * radius
        self.assertAlmostEqual(
            loop.equivalent_diameter_mm, true_diameter, delta=0.5
        )


class MeshSectionerBucketBoundaryTests(unittest.TestCase):
    """Regression for the third mesh_section bug: the z-bucket lookup used
    the caller's raw Z while the intersection math used an epsilon-nudged
    plane, so a Z landing one ULP below an integer bucket boundary (exactly
    what Orca's accumulated ``print_z`` produces -- 0.2 is not exact in
    binary float64) could select a bucket that never held the triangles
    actually straddling the nudged plane, and silently return zero loops
    for a Z deep inside the mesh. ``tools/m18_mesh_probe.py``'s three-line
    reproduction found this on the real wing fixture at z=12.0 (layer 60,
    accumulated from 0.2 steps): ``section_at(11.999999999999998)`` returned
    an empty list where 2 loops were correct.
    """

    def setUp(self):
        self.outer = _square(10.0, 10.0, 10.0)
        self.inner = _square(10.0, 10.0, 2.0)
        self.z0, self.z1 = 0.0, 10.0
        vertices, triangles = _hollow_box_mesh(
            self.outer, self.inner, self.z0, self.z1
        )
        # z_bin_mm=1.0 puts a bucket boundary at every integer Z, including
        # the z=5.0 station probed below.
        self.sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)

    def test_one_ulp_below_a_bucket_boundary_matches_the_boundary_itself(self):
        # This is THE bug: math.nextafter(5.0, -math.inf) is one float64
        # ULP below 5.0 -- far smaller than epsilon_mm -- so the nudged
        # plane for this query still lands just above 5.0, in the NEXT
        # integer bucket, while the raw Z's own bucket is the one below it.
        # A lookup keyed on the raw Z misses the triangles the nudged plane
        # actually needs.
        just_below = math.nextafter(5.0, -math.inf)
        self.assertLess(just_below, 5.0)

        at_boundary = self.sectioner.section_at(5.0)
        one_ulp_below = self.sectioner.section_at(just_below)

        self.assertEqual(len(at_boundary), len(one_ulp_below))
        self.assertEqual(len(one_ulp_below), 2)
        interior = next(lp for lp in one_ulp_below if lp.is_interior)
        self.assertAlmostEqual(interior.centroid[0], 10.0, places=3)
        self.assertAlmostEqual(interior.centroid[1], 10.0, places=3)

    def test_sweep_across_bucket_boundary_has_no_dropout(self):
        # The interior hole spans the whole box height, so every Z in this
        # sweep must find exactly 2 loops -- a dropout to 0 or 1 anywhere
        # in this range (which straddles the z=5.0 and z=6.0 bucket
        # boundaries) is the bug reappearing.
        z = 4.90
        counts = []
        while z <= 6.10:
            loops = self.sectioner.section_at(z)
            counts.append(len(loops))
            z = math.nextafter(z, math.inf)
            z += 0.02
        self.assertTrue(counts, "sweep produced no samples")
        self.assertEqual(set(counts), {2})

    def test_gap_strictly_inside_mesh_z_range_raises_value_error(self):
        # Two separate, disconnected boxes with a genuine gap between them
        # (z=5..6 has no triangles at all). The combined mesh's own z range
        # is [0, 11], so z=5.5 is strictly inside it yet has zero candidate
        # triangles crossing it -- exactly the "impossible empty result"
        # this module must refuse to return quietly, whether the cause is
        # a lookup bug or (as constructed here) a genuinely non-manifold
        # input.
        box_a = _box_mesh(0.0, 0.0, 20.0, 20.0, 0.0, 5.0)
        box_b = _box_mesh(0.0, 0.0, 20.0, 20.0, 6.0, 11.0)
        vertices, triangles = _combine(box_a, box_b)
        sectioner = MeshSectioner(vertices, triangles, z_bin_mm=1.0)

        with self.assertRaises(ValueError) as ctx:
            sectioner.section_at(5.5)
        message = str(ctx.exception)
        self.assertIn("5.5", message)
        self.assertIn("0.000000", message)
        self.assertIn("11.000000", message)

    def test_z_outside_mesh_range_returns_empty_without_raising(self):
        # Pinning the chosen (documented) behaviour: a Z beyond the mesh's
        # own span is a legitimate empty result, not a defect -- a caller
        # sweeping past the tip of a wing should not be forced to catch an
        # exception for every out-of-range layer.
        loops_above = self.sectioner.section_at(self.z1 + 5.0)
        loops_below = self.sectioner.section_at(self.z0 - 5.0)
        self.assertEqual(loops_above, [])
        self.assertEqual(loops_below, [])

    def test_z_exactly_at_mesh_extreme_does_not_raise(self):
        # Pinning the documented edge case: at z_min/z_max, this module does
        # not treat "empty" as an error. The epsilon nudge is not
        # always upward (it nudges downward for a z STRICTLY inside the mesh, close enough to z_max that the default
        # upward nudge would overshoot it -- see module docstring item 7) --
        # but at z0 (exactly z_min) and z1 (exactly z_max) neither of guards
        # fires (the z_max flip requires z < z_max, strictly), so THIS test's
        # own two probes still take the default upward nudge. For z0 that lands just inside the tube (matching
        # the interior fixture's usual two loops); for z1 it lands just past
        # the mesh's own top edge, so no triangle crosses it there (empty,
        # legitimate per item 6(b) -- a tapered mesh closing to a point exactly
        # at its extreme Z would ALSO legitimately get an empty result here;
        # either outcome must not raise).
        loops_at_z0 = self.sectioner.section_at(self.z0)
        loops_at_z1 = self.sectioner.section_at(self.z1)
        self.assertEqual(len(loops_at_z0), 2)
        self.assertEqual(loops_at_z1, [])


if __name__ == "__main__":
    unittest.main()
