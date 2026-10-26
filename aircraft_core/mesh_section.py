"""``MeshSectioner``: cross-section a triangle mesh at a Z plane, recovering
every closed loop -- outer boundary AND interior holes -- as plain polygons.

WHY THIS EXISTS. Spiral Vase deletes interior holes from the *slices* the
plugin receives at ``posSlice``, but never touches the *mesh* underneath them.
A spar channel modelled in CAD is therefore invisible in
``LayerRegion.slices`` yet still present, undamaged, in
``ctx.print.model.objects[0].volumes[0].mesh``. To make the CAD model the
single source of truth for spar geometry, the plugin re-derives those interior
loops itself, once per layer, by intersecting the mesh with the layer's Z
plane -- what a slicer's own slicing engine does internally, done here in
Python on data Orca has already discarded.

This module knows nothing about Orca, ``orca.host`` types, or the plugin
runtime. It takes plain vertex/triangle arrays (Python floats and ints, no
numpy required: bulk numpy array access on ``TriangleMesh`` throws
``ImportError`` when numpy isn't installed in the host's embedded interpreter,
but the per-element ``vertex(i)``/``triangle(i)`` accessors don't need it) and
returns plain ``Loop`` records. The caller (the plugin, at the edge) is
responsible for applying any placement transform (offset/rotation/scale) to
vertices BEFORE handing them here, and for converting ``Loop.points`` into
whatever the host Surface/ExPolygon constructor wants.

DESIGN CHOICES, AND WHY.

1. Z-BUCKETING FOR SPEED. A wing slice is ~600 layers over a mesh of ~20k
   facets. Testing every triangle against every Z is 12M triangle-plane tests
   per slice, all in Python, on the slicing worker thread. ``__init__`` bins
   every triangle once by its ``[zmin, zmax]`` span into fixed-height buckets
   keyed by ``z_bin_mm``, so ``section_at(z)`` only iterates triangles whose
   span could plausibly cross that Z -- one dict lookup per relevant bucket,
   not a scan of all triangles. ``z_bin_mm`` should be set to roughly the
   print's layer height; a coarser bin makes each bucket bigger (more
   triangles per lookup, still far fewer than "all of them") and a finer bin
   uses more memory for negligible gain, since typical layer heights
   (0.1-0.3 mm) already keep buckets small on a wing-sized mesh.

2. THE COPLANAR-VERTEX PROBLEM, AND WHY THE EPSILON MUST BE DERIVED FROM THE
   MESH'S OWN COORDINATE MAGNITUDE. A mesh vertex lying EXACTLY on the
   section plane is the classic degenerate case: naive above/below sign
   tests classify it as neither, plane-crossing edges that touch it produce
   zero-length or duplicate segments, and coplanar triangles (all three
   vertices on the plane) contribute an edge that has no business being
   part of a cross-section loop at all -- it's the plane grazing a mesh
   face, not intersecting a solid interior.

   A bare ``1e-6`` mm nudge of the query plane is enough headroom above
   float64 rounding noise on synthetic double-precision test meshes, but NOT
   for a real STL: STL stores every coordinate as IEEE-754 **binary32**
   (float32), whose relative precision is ``2**-23`` (~1.192e-7). At a wing
   coordinate of magnitude ~75 mm, one float32 ULP is ~75 * 1.192e-7 =~
   9e-6 mm -- roughly TEN TIMES that constant. A CAD loft built from
   stations spaced exactly 3 mm apart round-trips through STL with
   per-vertex Z noise on that ~9e-6 mm order: two mesh vertices that are
   conceptually on the SAME station plane can land on opposite sides of an
   epsilon smaller than the noise that put them there, a coin-flip in
   classification that produces dangling segments (measured: 39 of 599
   layers on a real 41-station wing, one failure per interior station
   plane, ``tools/m18_mesh_probe.py``).

   The fix has two parts, and both are needed:

   (a) THE PLANE-NUDGE EPSILON IS DERIVED FROM THE MESH, NOT A BARE
       CONSTANT. ``__init__`` computes ``self.epsilon_mm`` from the largest
       absolute vertex coordinate seen (``coordinate_scale``), as
       ``max(EPSILON_FLOOR_MM, coordinate_scale * FLOAT32_ULP_RELATIVE *
       EPSILON_SAFETY_FACTOR)``. ``FLOAT32_ULP_RELATIVE = 2 ** -23`` is
       binary32 machine epsilon, and ``EPSILON_SAFETY_FACTOR = 10`` makes the
       resulting epsilon an order of magnitude bigger than the worst-case
       float32 rounding step at that coordinate magnitude, so it reliably
       DOMINATES the noise rather than merely matching it. At wing scale
       (~75 mm) this lands at ~9e-5 mm; a 1e-4 mm shift in section height
       is physically irrelevant against a 0.2 mm layer, and negligible
       against every real feature here (minimum wall thickness >= 0.1 mm).
       ``EPSILON_FLOOR_MM = 1e-6`` mm is the floor for near-origin or
       sub-millimetre meshes (unit tests) where the magnitude-derived value
       would otherwise be too small to mean anything; it is unrelated to
       float32 and is not claimed to dominate float32 noise at that scale,
       only to give small meshes some tolerance at all. The SAME
       ``self.epsilon_mm`` is reused for the loop-assembly endpoint tolerance
       (item 3) and the crossing-point vertex snap (below) -- one
       physically-justified precision floor for the whole module, not three
       independently tuned numbers that could drift apart. Tightening it back
       toward ``1e-6`` is only valid for meshes with exact float64
       coordinates, never for anything that passed through an STL.

   (b) A CROSSING NEAR A VERTEX IS SNAPPED TO THAT VERTEX, NOT INTERPOLATED.
       Enlarging the epsilon alone still leaves an ill-conditioned case: an
       edge from an (effectively) on-plane vertex to a far vertex crosses
       the plane at an interpolation parameter ``t`` very close to 0 or 1,
       and although the interpolation formula is algebraically identical
       regardless of which of the two triangles sharing that edge computed
       it, a SEPARATE near-plane vertex on a different edge of the same
       station ring is a different vertex, independently perturbed by its
       own float32 rounding, and the two nearby interpolated points from two
       different incident triangles need not agree even to the enlarged
       epsilon. ``_triangle_plane_segment`` therefore checks, per crossing,
       whether the vertex at either end of the crossing edge is itself within
       ``epsilon_mm`` of the plane; if so it uses that vertex's OWN stored
       (x, y) directly instead of the interpolated point. Because vertices
       are welded (shared by index) before reaching this module, every
       triangle incident to that mesh vertex then produces the exact same
       Python floats for that crossing point -- bit-identical, not merely
       "close within a tolerance" -- which lets adjacent segments close a
       loop by construction rather than by two independent computations
       happening to land inside the matching tolerance.

       One consequence has to be handled explicitly: when a single
       triangle's PLANE crosses very close to one of ITS OWN corners, both
       of that triangle's crossing edges snap to that same corner vertex,
       collapsing the "segment" to zero length. This is the geometrically
       correct limit -- the plane is grazing a corner, not cutting a chord
       across the triangle -- and the real boundary segment through that
       vertex is produced by whichever NEIGHBOURING triangle actually
       crosses it. ``_triangle_plane_segment`` therefore drops any resulting
       zero-length (within ``epsilon_mm``) segment, mirroring the
       coplanar-triangle skip: contribute nothing rather than a spurious
       point that would upset the endpoint-degree count in loop assembly.

3. LOOP ASSEMBLY WITH A NAMED, MESH-DERIVED TOLERANCE. Segments from
   adjacent triangles that share a mesh edge normally produce
   numerically identical endpoints (both derived from linear interpolation
   between, or snapped to, the SAME shared mesh vertex/vertices), so
   endpoint matching only needs to absorb genuine float32-provenance noise.
   It therefore reuses ``self.epsilon_mm`` (see item 2(a)): real mesh
   features of interest here (walls, channels) are never anywhere near that
   size, so anything closing at this tolerance is a genuine shared vertex,
   not a coincidence.

4. FAIL CLOSED ON DANGLING ENDPOINTS. If greedy endpoint-chasing cannot walk
   every segment back to its start, the mesh's cross-section at this Z is
   not a set of clean closed rings -- either the mesh is non-manifold at
   this Z, or something upstream is wrong. Geometry is never silently
   weakened or dropped, so this raises ``ValueError`` naming the Z, the
   segment count, and the number of dangling ends, rather than returning
   whatever partial loops did close.

5. INTERIOR CLASSIFICATION VIA POINT-IN-POLYGON, NOT CENTROID CONTAINMENT.
   A polygon's centroid is only guaranteed to lie inside the polygon for
   convex shapes; a concave loop (a lattice cross-section, a keyhole slot)
   can have its centroid fall outside its own boundary, so testing "is A's
   centroid inside B" would test the wrong thing. Instead this module finds
   a point that is provably inside each loop by taking the midpoint of the
   first edge and nudging it perpendicular to that edge by a tiny fraction
   of the edge length, then falling back to a horizontal-ray probe if that
   point is ambiguous. Ray casting against every OTHER loop with that
   guaranteed-interior point then answers "is this loop nested inside that
   one" correctly regardless of concavity. The loop of greatest
   ``|signed_area|`` is always the outer boundary (any hole must be strictly
   smaller in area than the material it is cut from), so it can never be
   `is_interior`.

6. THE BUCKET LOOKUP MUST USE THE SAME PLANE THE INTERSECTION MATH USES, AND
   AN IMPOSSIBLE EMPTY RESULT MUST NEVER BE RETURNED QUIETLY. If
   ``_candidate_triangles(z)`` were called with the caller's RAW requested Z
   while the intersection used ``eff_z = z + epsilon_mm``, the two would be
   different planes: one picking candidates, the other testing them. Orca
   accumulates ``print_z`` by summing layer heights, and 0.2 is not exact in
   binary float64, so a real host slice reaches layer 60 at
   ``z = 11.999999999999998``, not ``12.0``. That value's bucket
   (``floor(11.999999999999998) == 11``) holds only triangles with
   ``zmax <= 12.0``; the epsilon-nudged plane at ``~12.00009`` sits ABOVE
   every one of them, while the triangles that actually straddle
   ``12.00009`` live in bucket 12+, never consulted. The assembler would
   correctly find zero crossings in the candidate set and ``section_at``
   would return ``[]`` for a Z deep inside a mesh spanning 0-120 mm -- which
   a plugin caller cannot tell apart from "genuinely no spar channel at this
   layer".

   Two independent parts guard against this, because either one alone leaves
   a hole:

   (a) ``_candidate_triangles`` is always called with ``eff_z`` (the same
       nudged plane ``_triangle_plane_segment`` intersects against), never
       the caller's raw ``z``. Given that, a SINGLE bucket lookup is
       provably sufficient -- no need to also consult a neighbouring
       bucket -- because bucket membership was assigned at construction
       time over the INCLUSIVE range ``floor(zmin/bin)`` .. ``floor(zmax/
       bin)`` for every triangle, and ``floor`` is monotonic: for any value
       ``v`` with ``zmin <= v <= zmax``, ``floor(v/bin)`` must fall between
       ``floor(zmin/bin)`` and ``floor(zmax/bin)`` inclusive, hence inside
       whichever single bucket that triangle was filed under for that
       range.

   (b) That proof covers THIS mechanism only -- a different bug in candidate
       selection, or a genuinely non-manifold mesh, could still hand the
       assembler zero usable segments at a Z that is nowhere near a mesh
       boundary. ``section_at`` therefore checks, whenever it is about to
       return an empty list, whether the requested ``z`` lies STRICTLY inside
       the mesh's own ``[z_min, z_max]``. If it does, an empty result is
       impossible for a closed mesh and is raised as ``ValueError`` naming
       the Z, the mesh's Z range, and the candidate-triangle count, rather
       than returned as if it were a legitimate "no interior loop here"
       answer. Exactly AT ``z_min`` or ``z_max`` an empty result is allowed
       through without raising: many solids taper to a point or a knife
       edge at their extreme span (a wingtip closing to zero chord), where
       the true cross-section is legitimately degenerate. This module does
       not distinguish that case from a flat capped end at the same Z; both
       pass the check quietly.

7. THE EPSILON NUDGE MUST NOT PUSH THE EFFECTIVE PLANE OUTSIDE THE MESH.
   The nudge in item 2(a) (``eff_z = z + epsilon_mm``) is safe everywhere
   except within one ``epsilon_mm`` of ``z_max``, where nudging upward can
   push ``eff_z`` to or past ``z_max`` itself, so no triangle spans it and
   item 6's "impossible empty result" check fires for a Z that genuinely has
   a real (if narrow) cross-section. Measured on a real elevon wing STL: the
   true section 9.155e-05 mm below the mesh's own z_max is not a flat cap
   but a knife-edge sliver -- roughly 6.4 um (chordwise) by 1.8 mm
   (spanwise-adjacent outline extent), area 0.00577 mm^2 -- because the wing
   tapers to a near-zero-chord tip rather than closing on a flat end face.
   ``epsilon_mm`` (2.974e-4 mm there) is more than 3x that 9.155e-05 mm gap,
   so the default nudge overshoots ``z_max`` by 2.06e-4 mm.
   ``_section_at_exact`` therefore nudges DOWNWARD instead whenever the
   requested ``z`` is STRICTLY LESS THAN ``z_max`` (never when
   ``z == z_max`` exactly -- that stays item 6(b)'s quiet-empty case), the
   upward nudge would reach or exceed ``z_max``, AND the downward nudge is
   itself safe (strictly above ``z_min``). Both directions use the identical
   ``epsilon_mm``, only the sign flips, so the precision reasoning of items
   2-3 is unaffected. No equivalent flip is needed near ``z_min``: nudging
   upward there moves ``eff_z`` further inside the mesh.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

Point = Tuple[float, float]
Point3 = Tuple[float, float, float]
TriIndices = Tuple[int, int, int]

# See module docstring, item 2. IEEE-754 binary32 machine epsilon
# (2**-23) -- STL's on-disk float precision -- used to size the working
# epsilon relative to the mesh's own coordinate magnitude, not as an
# absolute mm value by itself.
FLOAT32_ULP_RELATIVE = 2.0 ** -23

# See module docstring, item 2(a). The working epsilon is this many times
# the worst-case float32 rounding step at the mesh's coordinate magnitude,
# so it reliably DOMINATES that noise instead of merely matching it.
EPSILON_SAFETY_FACTOR = 10.0

# See module docstring, item 2(a). Floor for near-origin/sub-millimetre
# meshes (unit test fixtures) where the magnitude-derived epsilon would
# otherwise be too small to mean anything. Not itself a float32 claim.
EPSILON_FLOOR_MM = 1e-6


class DanglingEndpointError(ValueError):
    """Loop assembly left endpoints not shared by exactly two segments.

    Subclasses ``ValueError`` so a caller that catches ``ValueError`` around
    a section still sees it. It exists so ``section_at``'s retry can
    recognise THIS failure without matching on message text (a broad
    ``except ValueError`` would otherwise turn fail-closed invariants into
    silent fallbacks; see ``_SectionTooSmallForPitch``,
    ``ReEntrantSectionError``).

    Raised BOTH for the grazed-station-ring case described at
    ``_RETRY_NUDGES_MM`` AND for a genuinely non-manifold mesh; the two are
    indistinguishable at the raise site and are told apart by whether the
    failure survives a change of plane.
    """


@dataclass(frozen=True)
class Loop:
    """One closed ring recovered from a mesh cross-section.

    ``points`` is a closed ring, first point NOT repeated at the end,
    matching the convention of ``SlotPolygon`` (see ``slot.py``).
    """

    points: Tuple[Point, ...]
    signed_area: float
    is_interior: bool
    centroid: Point
    bbox: Tuple[float, float, float, float]
    equivalent_diameter_mm: float


def _signed_area(points: Sequence[Point]) -> float:
    """Shoelace signed area. Positive for counter-clockwise winding."""
    area = 0.0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return area / 2.0


def _centroid(points: Sequence[Point], signed_area: float) -> Point:
    """Polygon centroid (area-weighted), not the vertex average.

    Falls back to the vertex average for a degenerate (near-zero-area)
    ring, since the area-weighted formula divides by ``6 * signed_area``.
    """
    if abs(signed_area) < 1e-12:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return (sum(xs) / len(points), sum(ys) / len(points))
    cx = 0.0
    cy = 0.0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        cross = x1 * y2 - x2 * y1
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    factor = 1.0 / (6.0 * signed_area)
    return (cx * factor, cy * factor)


def _bbox(points: Sequence[Point]) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs), min(ys), max(xs), max(ys))


def _point_in_polygon(pt: Point, ring: Sequence[Point]) -> bool:
    """Standard even-odd ray-casting point-in-polygon test.

    Casts a horizontal ray from ``pt`` toward +X and counts crossings with
    the ring's edges. Correct for both convex and concave rings, which is
    the reason it is used instead of any centroid-based shortcut for
    interior/exterior classification (see module docstring item 5).
    """
    x, y = pt
    inside = False
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_at_y = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x_at_y > x:
                inside = not inside
    return inside


def _interior_point(ring: Sequence[Point]) -> Point:
    """A point provably inside ``ring``, safe for concave rings.

    The centroid of a concave ring can fall outside it (a "C" shape, a
    keyhole slot), so it cannot be trusted as a generic "is this loop inside
    that one" probe -- see module docstring item 5. Instead this nudges the
    midpoint of the first edge slightly inward, along that edge's inward
    normal, by a small fraction of the ring's own scale. Because the ring is
    a simple polygon (produced by this module's own loop assembly, never
    self-intersecting), a sufficiently small nudge from an edge midpoint
    toward the ring's interior side always lands inside -- unlike the
    centroid, this is a *local* property of one edge, not a global average
    that a concave shape can pull outside the boundary.
    """
    x1, y1 = ring[0]
    x2, y2 = ring[1]
    mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    ex, ey = x2 - x1, y2 - y1
    length = math.hypot(ex, ey)
    if length == 0:
        # Degenerate zero-length first edge; fall back to the vertex
        # average, which is still a reasonable interior guess for the
        # non-degenerate remainder of the ring.
        xs = [p[0] for p in ring]
        ys = [p[1] for p in ring]
        return (sum(xs) / len(ring), sum(ys) / len(ring))
    # Perpendicular to the edge, both candidate directions.
    nx, ny = -ey / length, ex / length
    step = max(length * 1e-3, 1e-9)
    candidate_a = (mx + nx * step, my + ny * step)
    candidate_b = (mx - nx * step, my - ny * step)
    if _point_in_polygon(candidate_a, ring):
        return candidate_a
    return candidate_b


def _key(pt: Point, tolerance_mm: float) -> Tuple[int, int]:
    """Round a point to ``tolerance_mm`` for use as a dict key.

    ``tolerance_mm`` is the mesh-derived ``MeshSectioner.epsilon_mm`` (see
    module docstring items 2(a) and 3), passed in rather than read from a
    module constant so it tracks each mesh's own coordinate magnitude.
    """
    scale = 1.0 / tolerance_mm
    return (round(pt[0] * scale), round(pt[1] * scale))


class MeshSectioner:
    """Cross-sections a triangle mesh at arbitrary Z planes.

    See the module docstring for the full rationale. Construction does the
    one-time O(triangle count) work of binning triangles by Z span;
    ``section_at`` then only visits triangles whose span could plausibly
    cross the requested Z.
    """

    def __init__(
        self,
        vertices: Sequence[Point3],
        triangles: Sequence[TriIndices],
        z_bin_mm: float = 1.0,
    ):
        if z_bin_mm <= 0:
            raise ValueError(
                "z_bin_mm must be > 0, got {!r}.".format(z_bin_mm)
            )
        self._vertices: List[Point3] = list(vertices)
        self._triangles: List[TriIndices] = list(triangles)
        self._z_bin_mm = float(z_bin_mm)
        self.triangle_count = len(self._triangles)

        # See module docstring, item 2(a): the working precision floor for
        # this mesh, derived from its own coordinate magnitude so it
        # reliably dominates float32-provenance noise (a real STL) without
        # being needlessly loose on an exact-coordinate synthetic mesh.
        coordinate_scale = 1.0
        for vx, vy, vz in self._vertices:
            coordinate_scale = max(
                coordinate_scale, abs(vx), abs(vy), abs(vz)
            )
        self.epsilon_mm = max(
            EPSILON_FLOOR_MM,
            coordinate_scale * FLOAT32_ULP_RELATIVE * EPSILON_SAFETY_FACTOR,
        )

        # Bucket index: bucket index -> list of triangle indices whose
        # [zmin, zmax] span overlaps that bucket. A triangle spanning
        # multiple buckets is listed in each one it overlaps, so
        # section_at never has to consult more than the single bucket
        # containing the query plane -- PROVIDED that bucket is derived
        # from the SAME value used here to decide membership (see
        # ``_candidate_triangles`` and module docstring item 6 for why this
        # matters).
        self._buckets: Dict[int, List[int]] = {}
        self.z_min = None
        self.z_max = None

        # the offset actually applied by the most recent SUCCESSFUL
        # ``section_at`` call, in mm. ``0.0`` means the unnudged plane worked.
        # Set on every successful return from ``section_at`` -- including the
        # unnudged path, where it is reset to ``0.0`` -- so a caller can tell a
        # retried section apart from a clean one without re-deriving it. Not a
        # property: computed once per call and cached here rather than
        # re-evaluated on access, so it can never disagree with the section it
        # describes.
        self.last_nudge_mm: float = 0.0
        for tri_idx, (i, j, k) in enumerate(self._triangles):
            zs = (self._vertices[i][2], self._vertices[j][2], self._vertices[k][2])
            zmin, zmax = min(zs), max(zs)
            if self.z_min is None or zmin < self.z_min:
                self.z_min = zmin
            if self.z_max is None or zmax > self.z_max:
                self.z_max = zmax
            bucket_lo = int(math.floor(zmin / self._z_bin_mm))
            bucket_hi = int(math.floor(zmax / self._z_bin_mm))
            for b in range(bucket_lo, bucket_hi + 1):
                self._buckets.setdefault(b, []).append(tri_idx)

    @property
    def vertices(self) -> List[Point3]:
        """The mesh's vertices (read-only by convention; not copied). The
        root-end AREA signal reads the vertices near each Z extreme straight
        off the sectioner the capability already built."""
        return self._vertices

    def _candidate_triangles(self, plane_z: float) -> List[int]:
        """Triangles whose ``[zmin, zmax]`` span could cross ``plane_z``.

        ``plane_z`` MUST be the actual value the intersection math will use
        (i.e. the epsilon-nudged plane, not the caller's raw requested Z --
        see module docstring item 6). Bucket membership was assigned at
        construction time by ``floor(vertex_z / z_bin_mm)`` for every
        triangle's zmin and zmax, inclusive of every bucket in between; for
        any value ``v`` with ``zmin <= v <= zmax``, ``floor`` is monotonic
        non-decreasing, so ``floor(v / z_bin_mm)`` always lies between
        ``floor(zmin / z_bin_mm)`` and ``floor(zmax / z_bin_mm)`` inclusive
        -- meaning the SINGLE bucket ``floor(plane_z / z_bin_mm)`` is
        provably sufficient, with no need to also consult a neighbouring
        bucket, AS LONG AS the same ``plane_z`` is used both here and in
        the actual crossing test. A mismatch between the two -- looking up
        one Z but intersecting against another -- breaks that guarantee
        silently: a triangle can straddle the true plane while sitting
        entirely in a bucket the mismatched lookup never visits.
        """
        bucket = int(math.floor(plane_z / self._z_bin_mm))
        return self._buckets.get(bucket, [])

    # Retry nudges, in order, when loop assembly leaves dangling ends.
    #
    # THE MECHANISM, MEASURED. The triangles involved are 82.9 to 89.9
    # degrees from parallel to the plane, i.e. essentially VERTICAL walls,
    # not faces nearly parallel to the slice plane.
    #
    # TWO VERTICES lie within epsilon_mm of the EFFECTIVE plane. On the
    # fixture, distances to eff_z are +2.692e-05 and -3.601e-06 against an
    # epsilon_mm of 2.691e-04. Each is the LONE vertex of its incident
    # triangles on one side of the plane, so both crossing edges snap to that
    # same vertex and ``_triangle_plane_segment`` returns a zero-length
    # segment -- the "corner clip" its docstring names. All 5 zero-length
    # segments trace to exactly these 2 vertices. The two vertices are
    # 0.1999 mm apart laterally; they are not a doubled station ring.
    #
    # The contour must pass THROUGH those vertices, and the triangles that
    # should carry it through are exactly the ones that degenerate, so the
    # continuation is never built.
    #
    # epsilon_mm nudges the plane off a vertex lying EXACTLY on it. It
    # cannot help here: the nudge IS epsilon_mm, so a vertex closer than
    # that to eff_z still snaps.
    #
    # The self-collide drop in ``_assemble_loops`` is NOT the cause. With
    # that filter disabled all 391 segments assemble into 9 unbalanced
    # buckets (degrees 4,4,4,4,4,4,4,5,7) -- worse, never 2. The two buckets
    # that dangle were already unbalanced at degree 7 and 5 before the
    # filter ran, each holding one real wall segment plus zero-length
    # segments contributing 2 uses apiece to the same bucket. The filter
    # removes those correctly; what is left is the real segment whose
    # continuation was never built.
    #
    # The vertex positions are a property of the MESH; only their EFFECT
    # is plane-local -- a vertex degenerates a triangle only while eff_z
    # sits within epsilon_mm of it. That is why moving the plane by a
    # micron fixes it and why a 0.2 mm layer grid rarely sees it. The mesh
    # itself is a legitimate tessellation: the degeneracy is this
    # sectioner's snap rule meeting an unlucky plane, so the retry returns
    # the true section of a sound mesh.
    #
    # So the discriminator is not "mesh defect vs plane accident". It is
    # DOES THE FAILURE SURVIVE A CHANGE OF PLANE: a hole large enough to
    # matter does, and still fails closed; a grazed vertex does not.
    #
    # DO NOT "fix" this by widening epsilon_mm. It is BOTH the snap band and
    # the self-collide drop band, so widening drops MORE segments. 2 dangling
    # ends become 4 at 2x and stay 4 out to 500x; at 1000x (0.269 mm, wider
    # than a nozzle) it still fails, having destroyed 178 of 379 segments.
    #
    # An in-host failure at layer 562, Z=112.60 ("344 segment(s) produced 2
    # dangling endpoint(s)") is consistent with this mechanism; that mesh was
    # not available to confirm it.
    #
    # Retrying steps the plane off the ring. It cannot mask a mesh defect
    # large enough to matter: such a hole leaves dangling ends at EVERY
    # nearby plane, so all the retries fail and the original error still
    # surfaces (asserted by test). A sub-micron unwelded sliver WOULD be
    # masked, and that is accepted -- the section returned is still a true
    # section of the mesh. Offsets stay at or below 1% of one layer height
    # (0.2 mm).
    _RETRY_NUDGES_MM = (1e-4, -1e-4, 5e-4, -5e-4, 2e-3, -2e-3)

    def section_at(self, z: float) -> List[Loop]:
        """Cross-section the mesh at height ``z``, returning every closed
        loop (outer boundary and interior holes alike).

        Retries at a few micron-scale offsets if loop assembly
        leaves dangling ends -- see ``_RETRY_NUDGES_MM``. A genuinely
        non-manifold mesh fails at every offset and still raises.

        A NUDGED attempt can fail with a DIFFERENT ``ValueError`` than
        ``DanglingEndpointError`` -- e.g. ``_check_empty_result_is_legitimate``
        raising because the nudged plane happens to land somewhere the
        original, un-nudged plane never would have. That secondary failure
        names a Z the caller never asked for and must never REPLACE the
        original ``DanglingEndpointError``, which names the Z the caller
        actually requested; it is swallowed here (tried, then moved past)
        so the retry loop keeps going, and if every offset fails the
        original error still surfaces. Note the deliberate asymmetry: the
        FIRST, un-nudged call's non-dangling ``ValueError`` is NOT caught
        here at all -- it propagates immediately, unretried.
        """
        try:
            result = self._section_at_exact(z)
            self.last_nudge_mm = 0.0
            return result
        except DanglingEndpointError as original:
            first_secondary_failure = None
            for dz in self._RETRY_NUDGES_MM:
                zz = z + dz
                if not (self.z_min < zz < self.z_max):
                    continue
                try:
                    result = self._section_at_exact(zz)
                    self.last_nudge_mm = dz
                    return result
                except DanglingEndpointError:
                    continue
                except ValueError as secondary_failure:
                    if first_secondary_failure is None:
                        first_secondary_failure = secondary_failure
                    continue
            # Every offset failed -- either by still dangling, or by some
            # other ValueError -- so re-raise the ORIGINAL error, naming
            # the Z the caller actually asked for. Chain the first
            # secondary failure (if any) purely for diagnostic visibility;
            # ``raise ... from`` does not change the exception type or
            # message the caller sees, only its __cause__.
            raise original from first_secondary_failure

    def _section_at_exact(self, z: float) -> List[Loop]:
        """``section_at`` without the retry, at exactly this plane.

        Vertices are classified against a plane nudged by ``self.epsilon_mm``
        off the requested Z (see module docstring item 2) so an
        exactly-coplanar mesh vertex never has to be handled as a special
        case in the segment math; a triangle that remains exactly coplanar
        after the nudge is skipped rather than turned into a spurious
        segment. Crossings near a vertex are snapped to that
        vertex's own coordinates (module docstring item 2(b)) so adjacent
        triangles agree by construction rather than by tolerance.
        """
        eff_z = z + self.epsilon_mm
        # Near the mesh's own z_max, nudging UPWARD (the default direction
        # above) can push eff_z to or past z_max itself -- no triangle spans
        # above z_max, so candidate selection (and every real crossing) comes
        # back empty even though the requested z is genuinely, if narrowly,
        # inside the mesh. Measured on a real elevon wing STL:
        # z_max=249.500092 mm, requested z=249.5 mm (9.155e-5 mm below z_max --
        # NOT a flat tip cap; the wing tapers to a near-zero-chord tip, so the
        # true section there is a knife-edge sliver, roughly 6.4 um by 1.8 mm,
        # area 0.00577 mm^2), epsilon_mm=2.974e-4 mm; the default
        # eff_z=249.500297 mm lands 2.06e-4 mm ABOVE z_max, producing a false
        # "ZERO loops" ValueError for a layer whose real (if tiny) section sits
        # 9e-5 mm below z_max. Nudging DOWNWARD instead keeps eff_z strictly
        # inside [z_min, z_max] and sections that knife-edge outline.
        # Guarded so this only fires when (a) the downward nudge is ALSO safe
        # (z - epsilon_mm > z_min) and (b) z is STRICTLY less than z_max --
        # z == z_max exactly is item 6(b)'s documented "quiet empty" edge case
        # (a genuine point-taper tip has zero area AT its extreme Z, and this
        # module does not try to tell that apart from a flat capped end at the
        # same exact Z). On a mesh so short that neither nudge direction is
        # safe, the upward nudge is kept (both extreme-Z edge cases return an
        # empty result quietly, never raise, so a residual upward nudge there
        # is not a new hazard). No equivalent flip is needed near z_min:
        # nudging upward there moves eff_z further INSIDE the mesh, away from
        # z_min.
        if (z < self.z_max and eff_z >= self.z_max
                and z - self.epsilon_mm > self.z_min):
            eff_z = z - self.epsilon_mm

        # THE LOOKUP MUST USE THE SAME PLANE THE INTERSECTION MATH USES.
        # See module docstring item 6: using the caller's raw ``z`` here
        # while intersecting against ``eff_z`` would let the epsilon nudge
        # push the true plane into a bucket this call never consulted,
        # handing the assembler zero segments for a Z that is genuinely
        # inside the mesh -- an impossible, silently wrong empty result. Both
        # must agree; ``_candidate_triangles``'s docstring proves a single
        # bucket, looked up this way, is sufficient.
        candidates = self._candidate_triangles(eff_z)

        segments: List[Tuple[Point, Point]] = []
        for tri_idx in candidates:
            i, j, k = self._triangles[tri_idx]
            v0, v1, v2 = self._vertices[i], self._vertices[j], self._vertices[k]
            seg = self._triangle_plane_segment(v0, v1, v2, eff_z, self.epsilon_mm)
            if seg is not None:
                segments.append(seg)

        if not segments:
            self._check_empty_result_is_legitimate(z, len(candidates))
            return []

        loops_points = self._assemble_loops(segments, z, self.epsilon_mm)

        loops: List[Loop] = []
        for pts in loops_points:
            area = _signed_area(pts)
            centroid = _centroid(pts, area)
            bbox = _bbox(pts)
            diameter = 2.0 * math.sqrt(abs(area) / math.pi) if area else 0.0
            loops.append(
                Loop(
                    points=tuple(pts),
                    signed_area=area,
                    is_interior=False,  # filled in below
                    centroid=centroid,
                    bbox=bbox,
                    equivalent_diameter_mm=diameter,
                )
            )

        # ``_assemble_loops`` always turns a non-empty segment list into at
        # least one loop (or raises on dangling ends), so ``loops`` cannot
        # be empty here -- the fail-closed check above already covers the
        # only path that can legitimately produce zero.
        loops = self._classify_interior(loops)
        return loops

    def _check_empty_result_is_legitimate(self, z: float, candidate_count: int) -> None:
        """Refuse to return an empty section for a Z strictly inside the
        mesh's own Z range (module docstring item 6(b)).

        A closed, manifold mesh has SOME cross-section at every Z strictly
        between its lowest and highest vertex -- an empty result there is
        never a valid answer, only evidence of a bug in candidate-triangle
        selection or a non-manifold/self-intersecting input. Both are
        defects to surface loudly, not silently degrade into "no spar
        channel here": a caller (the plugin) that gets an empty list here
        cannot tell "correctly no interior loop at this Z" from "the
        sectioner silently lost the geometry".

        Two cases are legitimate and return quietly instead of raising,
        because a manifold solid genuinely need not have a cross-section
        AT its own extreme Z values:
          - ``z`` at or beyond ``[z_min, z_max]`` entirely (outside the
            mesh, e.g. a caller sweeping past the tip): nothing to find.
          - ``z`` exactly equal to ``z_min`` or ``z_max``: many meshes
            taper to a point or a knife edge at their extreme span (a
            wingtip closing to zero chord), where the true cross-section
            is degenerate (zero area) and legitimately produces no usable
            loop. This module does not attempt to distinguish a genuine
            point-taper from a flat capped end at exactly z_min/z_max;
            both are treated as edge cases outside the "strictly interior"
            guarantee this check makes.
        """
        if self.z_min is None or self.z_max is None:
            return  # empty mesh; nothing to check against.
        if z <= self.z_min or z >= self.z_max:
            return
        raise ValueError(
            "mesh cross-section at z={:.6f} mm produced ZERO loops, but "
            "z lies strictly inside the mesh's own z range [{:.6f}, "
            "{:.6f}] mm ({} candidate triangle(s) were tested). A closed "
            "mesh cannot have an empty cross-section strictly inside its "
            "span; this means candidate-triangle selection missed the "
            "triangles that actually straddle this Z (or the mesh is "
            "non-manifold here). Refusing to return an empty result that "
            "a caller cannot distinguish from 'genuinely no interior "
            "loop at this layer'.".format(
                z, self.z_min, self.z_max, candidate_count
            )
        )

    @staticmethod
    def _triangle_plane_segment(v0, v1, v2, z, epsilon_mm):
        """Return the (p, q) segment where triangle (v0, v1, v2) crosses
        plane Z == z, or None if it doesn't cross (entirely above, entirely
        below, or exactly coplanar).

        Coplanar triangles are skipped deliberately: a
        triangle lying flat in the section plane contributes an edge, not a
        cross-section of a solid interior, and including it would corrupt
        loop assembly with a spurious extra segment.

        A crossing edge whose vertex at either end is itself within
        ``epsilon_mm`` of the plane is SNAPPED to that vertex's own (x, y)
        rather than interpolated (module docstring item 2(b)): near a
        vertex the interpolation parameter is ill-conditioned, and two
        different, independently-perturbed near-plane vertices on the same
        station ring are not guaranteed to interpolate to points that agree
        even to a generous tolerance. Using the vertex's own stored floats
        makes every triangle sharing that (welded) vertex index produce a
        bit-identical point, which is what lets loop assembly close by
        construction.
        """
        d0 = v0[2] - z
        d1 = v1[2] - z
        d2 = v2[2] - z

        signs = [d0 > 0, d1 > 0, d2 > 0]
        # All strictly above or all strictly below: no crossing.
        if all(s for s in signs) or all(not s for s in signs):
            # Exactly coplanar (all distances zero) also lands here since
            # `0 > 0` is False for every vertex -- treated as "all below",
            # correctly skipped.
            return None

        verts = (v0, v1, v2)
        dists = (d0, d1, d2)
        pts: List[Point] = []
        for a in range(3):
            b = (a + 1) % 3
            da, db = dists[a], dists[b]
            if (da > 0) != (db > 0):
                va, vb = verts[a], verts[b]
                if abs(da) <= epsilon_mm:
                    pts.append((va[0], va[1]))
                elif abs(db) <= epsilon_mm:
                    pts.append((vb[0], vb[1]))
                else:
                    t = da / (da - db)
                    x = va[0] + t * (vb[0] - va[0])
                    y = va[1] + t * (vb[1] - va[1])
                    pts.append((x, y))
        if len(pts) != 2:
            # A vertex sits exactly on the (nudged) plane while the
            # opposite edge still crosses it; this degenerates to 0 or 1
            # unique crossing points depending on rounding. Treat as no
            # usable segment rather than guess at a corrupted one.
            return None
        # A triangle whose plane crossing collapses to a single point (a
        # "corner clip", where both crossing edges snapped to the same
        # near-plane vertex, or -- on a finely tessellated real mesh -- two
        # independently-computed points that are simply closer together
        # than epsilon_mm) is filtered globally in ``_assemble_loops``, not
        # here: see that method's docstring for why a per-triangle distance
        # check is the wrong place to do it.
        return (pts[0], pts[1])

    @staticmethod
    def _assemble_loops(
        segments: List[Tuple[Point, Point]], z: float, epsilon_mm: float
    ) -> List[List[Point]]:
        """Chain plane-intersection segments into closed rings by matching
        endpoints within ``epsilon_mm`` (see module docstring items 2(a)
        and 3).

        Fails closed: any endpoint that cannot be matched to
        exactly one partner raises ``ValueError`` naming the Z, the segment
        count, and how many ends were left dangling, rather than returning
        whatever partial rings happened to close.

        SELF-COLLIDING SEGMENTS ARE DROPPED HERE, GLOBALLY, USING THE SAME
        QUANTIZED KEY AS EVERYTHING ELSE -- AND NOT BY A PER-TRIANGLE
        EUCLIDEAN-DISTANCE CHECK IN ``_triangle_plane_segment``. A real,
        finely tessellated mesh (measured on the wing fixture, see
        ``tools/m18_mesh_probe.py``) can legitimately contain triangle edges
        only slightly longer than the mesh-derived ``epsilon_mm``, so a plane
        crossing very close to (but not through) one of that triangle's own
        vertices can produce two DISTINCT, correctly computed points that
        nonetheless land within ``epsilon_mm`` of each other purely because the
        local tessellation is that fine. That is real (if physically
        negligible) surface detail, not a coplanar-vertex problem (module
        docstring item 2). Dropping such a segment per triangle is actively
        harmful: it does not merely remove a redundant point, it removes one
        of the two real connections a NEIGHBOURING segment relies on,
        converting a single false positive into new dangling ends elsewhere.
        A per-triangle Euclidean check raised the probe's failure count from
        39/599 layers to 596/599.

        The distinction that matters is not "is this segment short" but "do
        this segment's OWN two endpoints round to the SAME matching bucket
        that everything else in this function uses". If they do, this
        segment is a self-loop UNDER THIS FUNCTION'S OWN notion of point
        equality -- entering and leaving at a point this function cannot tell
        apart -- and removing it cannot disconnect anything meaningful under
        that same notion of equality. Any segment whose two endpoints round
        to DIFFERENT buckets survives untouched, however short it is in raw
        mm, because by this function's own equality it bridges two genuinely
        different points. Reusing the identical ``_key(..., epsilon_mm)``
        quantization for this check (rather than an independent Euclidean
        threshold) makes the two notions of "same point" agree; with it, 0/599
        layers fail on the same fixture (``tools/m18_mesh_probe.py``).
        """
        segments = [
            (p, q)
            for (p, q) in segments
            if _key(p, epsilon_mm) != _key(q, epsilon_mm)
        ]

        # Map each rounded endpoint key to the segment endpoints touching
        # it, so we can detect dangling (degree != 2) endpoints up front
        # and give an actionable diagnostic instead of an obscure KeyError
        # or an infinite loop during chaining.
        endpoint_uses: Dict[Tuple[int, int], List[int]] = {}
        for idx, (p, q) in enumerate(segments):
            endpoint_uses.setdefault(_key(p, epsilon_mm), []).append(idx)
            endpoint_uses.setdefault(_key(q, epsilon_mm), []).append(idx)

        dangling = [k for k, uses in endpoint_uses.items() if len(uses) != 2]
        if dangling:
            raise DanglingEndpointError(
                "mesh cross-section at z={:.6f} mm did not assemble into "
                "closed loops: {} segment(s) produced {} dangling "
                "endpoint(s) (each closed-loop endpoint must be shared by "
                "exactly 2 segments). This means the mesh is not manifold "
                "at this Z, or an upstream transform corrupted the "
                "geometry; refusing to return a partial or silently-closed "
                "ring.".format(z, len(segments), len(dangling))
            )

        used = [False] * len(segments)
        loops: List[List[Point]] = []

        for start_idx in range(len(segments)):
            if used[start_idx]:
                continue
            loop: List[Point] = []
            current_idx = start_idx
            # Orient the walk starting from p -> q of the seed segment.
            start_key = _key(segments[start_idx][0], epsilon_mm)
            current_point = segments[start_idx][0]
            target_key = start_key

            while True:
                p, q = segments[current_idx]
                used[current_idx] = True
                if _key(p, epsilon_mm) == _key(current_point, epsilon_mm):
                    loop.append(p)
                    next_point = q
                else:
                    loop.append(q)
                    next_point = p

                next_key = _key(next_point, epsilon_mm)
                if next_key == target_key:
                    break

                candidates = [
                    i for i in endpoint_uses[next_key] if not used[i]
                ]
                if not candidates:
                    # Endpoint counts were verified balanced above, so this
                    # should be unreachable; fail loudly rather than return
                    # a broken partial ring if it ever is.
                    raise ValueError(
                        "mesh cross-section at z={:.6f} mm: loop assembly "
                        "reached endpoint with no unused partner segment "
                        "despite balanced endpoint counts; refusing to "
                        "return a partial ring.".format(z)
                    )
                current_idx = candidates[0]
                current_point = next_point

            loops.append(loop)

        return loops

    @staticmethod
    def _classify_interior(loops: List[Loop]) -> List[Loop]:
        """Mark every loop nested inside another as ``is_interior``.

        The outer boundary is the loop of greatest ``|signed_area|`` (a hole
        can never be larger than the material it is cut from), so it is
        never interior. Every other loop is tested by point-in-polygon
        against a provably-interior probe point (see ``_interior_point`` and
        module docstring item 5), never against centroid containment, since
        a concave loop's centroid is not guaranteed to lie inside it.
        """
        if len(loops) <= 1:
            return loops

        order = sorted(
            range(len(loops)), key=lambda i: abs(loops[i].signed_area), reverse=True
        )
        outer_idx = order[0]

        probes = [_interior_point(loop.points) for loop in loops]

        result = list(loops)
        for i in range(len(loops)):
            if i == outer_idx:
                continue
            nested = False
            for j in range(len(loops)):
                if i == j:
                    continue
                if abs(loops[j].signed_area) <= abs(loops[i].signed_area):
                    continue
                if _point_in_polygon(probes[i], loops[j].points):
                    nested = True
                    break
            result[i] = Loop(
                points=loops[i].points,
                signed_area=loops[i].signed_area,
                is_interior=nested,
                centroid=loops[i].centroid,
                bbox=loops[i].bbox,
                equivalent_diameter_mm=loops[i].equivalent_diameter_mm,
            )
        return result
