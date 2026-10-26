"""Shared TEST/TOOL-ONLY real polygon-boolean ``GeometryOps`` backend, using
``shapely``.

The shipped plugin bundle stays dependency-free, so `aircraft_core` and
`plugin/aircraft_structures` must NEVER import this module or `shapely`. This
module exists so that ``tests/unit/test_erosion_near_leading_edge.py`` and
``tools/verify_full_sweep.py`` share exactly ONE implementation of the
real-boolean backend, rather than two copies that can drift apart.

THE JOIN STYLE: a mitre-join offset (mitre limit 3.0, shapely's
``join_style=2``) is used because it is the empirical match to a real in-host
failure traced by direct reproduction. It is NOT a verified statement about
the pinned Orca/Clipper source's own join parameters, which could not be
re-inspected offline.

Importable even when ``shapely`` is absent: callers must check
``HAVE_SHAPELY`` themselves before constructing ``MiterShapelyOps`` (which
raises ``ImportError`` at construction time if shapely was never
importable) — this module never raises merely on import, so a caller can
report "not verified, shapely missing" instead of crashing.
"""
from __future__ import annotations

try:
    from shapely.geometry import GeometryCollection, MultiPolygon, Polygon
    from shapely.ops import unary_union

    HAVE_SHAPELY = True
except ImportError:  # pragma: no cover - environment-dependent
    HAVE_SHAPELY = False

from aircraft_core.ops import GeometryOps


def _as_pieces(geom):
    if geom is None or geom.is_empty:
        return []
    if HAVE_SHAPELY and isinstance(geom, Polygon):
        return [geom]
    if HAVE_SHAPELY and isinstance(geom, (MultiPolygon, GeometryCollection)):
        out = []
        for g in geom.geoms:
            out.extend(_as_pieces(g))
        return out
    return []


class MiterShapelyOps(GeometryOps):
    """Real polygon-boolean ``GeometryOps``, backed by shapely, mitre-join
    offset. See this module's own docstring for why this backend exists,
    why it is never imported by ``aircraft_core``/the shipped plugin, and
    why the join style is an empirical match rather than a verified pin.
    """

    def __init__(self):
        if not HAVE_SHAPELY:
            raise ImportError(
                "shapely is not installed; MiterShapelyOps cannot be constructed. "
                "This is a TEST/TOOL-ONLY backend -- see this module's "
                "own docstring."
            )

    def diff(self, subject, clips):
        if not clips:
            return [subject]
        return _as_pieces(subject.difference(unary_union(list(clips))))

    def union(self, polygons):
        return _as_pieces(unary_union(list(polygons)))

    def offset(self, polygon, delta_mm):
        return _as_pieces(
            polygon.buffer(delta_mm, join_style=2, mitre_limit=3.0, quad_segs=16)
        )

    def area(self, polygon):
        return polygon.area

    def is_valid(self, polygon):
        return polygon.is_valid

    def bounds(self, polygon):
        return tuple(polygon.bounds)

    def contour_count(self, polygon):
        return 1 + len(polygon.interiors)

    def intersection(self, subject, clip):
        return _as_pieces(subject.intersection(clip))

    def polygon_from_points(self, points_mm):
        return Polygon(list(points_mm))

    def contour_points_mm(self, polygon):
        return list(polygon.exterior.coords)[:-1]

    def hole_bounds(self, polygon):
        return [tuple(Polygon(interior).bounds) for interior in polygon.interiors]

    def hole_points_mm(self, polygon):
        return [list(interior.coords)[:-1] for interior in polygon.interiors]
