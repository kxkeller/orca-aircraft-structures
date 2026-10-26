"""``GeometryOps``: the boolean/geometry backend interface ``aircraft_core`` depends on.

``aircraft_core`` works entirely in **millimetres and Python floats**. It never
sees Orca's scaled-integer coordinate space. The host adapter (in ``plugin/``)
is responsible for converting to and from scaled integers when it constructs
the concrete ``GeometryOps`` implementation that wraps ``orca.host.ExPolygon``.
Test code satisfies the same interface with an exact-arithmetic stub
(``tests/unit/stub_ops.py``) that never touches scaled integers at all.

A "polygon" passed through this interface is opaque to ``aircraft_core``: it
is whatever object the concrete backend uses to represent one region,
including its holes (i.e. it is closer to an ExPolygon than a bare ring).
``aircraft_core`` never inspects a polygon's points directly — it only calls
back into ``GeometryOps``. This is what lets the same generator/validator
code run against Orca's Clipper-backed geometry in the host and against the
rectangle-only stub in tests.
"""

from abc import ABC, abstractmethod
from typing import List, Sequence, Tuple


class GeometryOps(ABC):
    """Boolean and query operations ``aircraft_core`` needs, backend-neutral.

    All linear units are millimetres, all areas are square millimetres, and
    all coordinates are Python floats. A concrete backend that only speaks
    scaled integers converts at its own boundary, not here.
    """

    @abstractmethod
    def diff(self, subject, clips: Sequence) -> List:
        """Boolean difference: ``subject`` minus every polygon in ``clips``.

        Returns a list of resulting polygons (zero or more — the result may
        be empty, one piece, or several disjoint pieces). ``subject`` and
        entries of ``clips`` are opaque polygon objects as returned by other
        ``GeometryOps`` methods or supplied by the caller in the same
        representation.
        """
        raise NotImplementedError

    @abstractmethod
    def offset(self, polygon, delta_mm: float) -> List:
        """Offset (grow if positive, erode if negative) by ``delta_mm``.

        Returns a list of resulting polygons — erosion can split one polygon
        into several, or eliminate it entirely (empty list).
        """
        raise NotImplementedError

    @abstractmethod
    def area(self, polygon) -> float:
        """Signed or unsigned area in mm^2 (holes already netted out)."""
        raise NotImplementedError

    @abstractmethod
    def is_valid(self, polygon) -> bool:
        """Whether the backend considers ``polygon`` well-formed.

        Mirrors ``ExPolygon.is_valid()`` in the host binding.
        """
        raise NotImplementedError

    @abstractmethod
    def bounds(self, polygon) -> Tuple[float, float, float, float]:
        """Axis-aligned bounding box in mm: ``(min_x, min_y, max_x, max_y)``."""
        raise NotImplementedError

    @abstractmethod
    def contour_count(self, polygon) -> int:
        """``1 + number of interior holes`` for a single-region polygon.

        A value greater than 1 means ``polygon`` has interior holes, which
        the single-loop invariant forbids in a result.
        """
        raise NotImplementedError

    @abstractmethod
    def intersection(self, subject, clip) -> List:
        """Boolean intersection: the region ``subject`` and ``clip`` share.

        Returns a list of resulting polygons (zero or more — two disjoint
        overlaps between a non-convex ``subject`` and ``clip`` can yield
        several pieces). Used for local thickness measurement
        (``frame.local_span``). The host binds ``ExPolygon.intersection_ex``
        directly, so the host adapter only has to expose it.
        """
        raise NotImplementedError

    @abstractmethod
    def polygon_from_points(self, points_mm: Sequence[Tuple[float, float]]):
        """Build a backend polygon from a millimetre ``(x, y)`` point ring.

        Used to construct simple clip shapes (a slot rectangle, a thin
        measurement strip in ``frame.local_span``) without any caller needing
        to know the backend's native point/contour types. The host adapter
        implements it as ``OrcaGeometryOps.polygon_from_points``.
        """
        raise NotImplementedError

    @abstractmethod
    def contour_points_mm(self, polygon) -> List[Tuple[float, float]]:
        """Vertices of ``polygon``'s OUTER contour, in millimetres, in the
        backend's own native order.

        Used by the rotation-envelope check (``aircraft_core.frame.
        estimate_rotation_deg``): a rotated section's chord line is not
        recoverable from ``bounds()`` alone (a bounding box has no notion of
        which vertex sits where), so estimating rotation needs the actual
        vertices. This is the information ``bounds()`` derives its min/max
        from (``OrcaGeometryOps.bounds`` walks ``polygon.contour.points``),
        returned as points instead of collapsed to an axis-aligned box.

        Only ever called on an ORIGINAL (pre-cut) per-layer section — the
        polygon a ``LayerFrame`` is built from — never on a slot or a
        post-boolean result, so a backend need not support holes or
        multiple contours here.

        EXCEPTION: ``plugin/aircraft_structures/capability.py``'s
        ``_rotate_expolygon`` also calls this (via ``OrcaGeometryOps``, the
        only backend that needs it) on POST-BOOLEAN replacement pieces, to
        rotate the validated result back into world frame. That is a
        host-boundary-only use, outside ``aircraft_core`` itself, and goes
        through ``hole_points_mm`` too when a piece has holes — the "never on a
        post-boolean result" contract above still describes what
        ``aircraft_core`` itself relies on (e.g. ``LayerFrame``, which is
        unaffected), not every caller of this method.
        """
        raise NotImplementedError

    @abstractmethod
    def hole_bounds(self, polygon) -> List[Tuple[float, float, float, float]]:
        """Axis-aligned bounding box, in mm, of each INTERIOR hole.

        One entry per interior contour, in the backend's own native hole order
        — mirrors ``bounds()`` (which reads the OUTER contour only) so the two
        stay consistent: same ``(min_x, min_y, max_x, max_y)`` shape, just
        applied to each hole instead of the outer ring. Returns an empty list
        for a polygon with no holes (``contour_count(polygon) == 1``) — this is
        the ordinary case and must stay cheap there.

        A spar-hole scar needs to know WHERE the hole is (at minimum its
        local Y-extent) before it can be placed or its overshoot bounded. The
        host backend binds ``ExPolygon.holes``, so this method only exposes
        it through the interface.

        This method does not itself make ``local_span``/
        ``local_upper_worst_case``/``local_lower_worst_case`` hole-aware;
        ``frame.local_span`` documents its opt-in, one-declared-hole-per-gap
        contract. ``frame.local_solid_thickness_worst_case`` is built on this
        method (summing the bbox height of every hole overlapping a span) and
        ``opposing_pair_meets`` uses it, rather than composing the two
        single-sided worst-case probes directly, which would count the hole's
        void as solid. This method only lets a caller ask about a hole's
        geometry directly, independent of any probe.
        """
        raise NotImplementedError

    @abstractmethod
    def hole_points_mm(self, polygon) -> List[List[Tuple[float, float]]]:
        """Vertices of each INTERIOR hole, in millimetres, in the backend's
        own native order.

        One point list per interior contour, mirroring
        ``contour_points_mm()`` (which reads the OUTER contour only) the
        same way ``hole_bounds()`` mirrors ``bounds()``. Returns an empty
        list for a polygon with no holes.

        Same motivation and same caveat as ``hole_bounds()`` above: this
        exposes geometry only and does not make anything downstream
        hole-aware.
        """
        raise NotImplementedError

    def union(self, polygons: Sequence) -> List:
        """Boolean union of ``polygons``. Returns the merged polygons.

        Used by the closing emulation in the mesh-alignment guard. Not
        abstract: only a backend that can actually merge overlapping regions
        implements it, and a backend that cannot fails closed rather than
        returning an unmerged list that would read as "no merge happened".
        """
        raise NotImplementedError(
            "{} does not implement GeometryOps.union".format(type(self).__name__)
        )
