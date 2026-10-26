"""``SlotBuilder``: converts a ``WebIntent`` into the subtractive slot polygon.

A web is realised by cutting a slot and relying on bead fusion (extrusion
space, not polygon space). This module only builds the polygon(s) to subtract;
the boolean difference is performed by the caller through a
``GeometryOps.diff`` call, keeping this module free of any backend
dependency.

Cap policy: the far end (at the margin point) is a **square cut**,
perpendicular to the centerline. A rounded cap would reduce stress
concentration at the web root but is not implemented.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

from .generator import WebIntent
from .spec import SpecError

Point = Tuple[float, float]


@dataclass(frozen=True)
class SlotPolygon:
    """A slot's outline, ready to hand to ``GeometryOps.diff`` as a clip.

    ``points`` is a closed polygon ring, wound counter-clockwise, expressed
    as plain ``(x, y)`` mm tuples — the same representation a concrete
    ``GeometryOps`` backend is expected to accept for a simple rectangular
    clip shape.
    """

    points: Tuple[Point, ...]


class SlotBuilder:
    """Builds one slot polygon per ``WebIntent``."""

    def build(self, intent: WebIntent, overshoot_mm: float) -> SlotPolygon:
        """Return the slot rectangle for ``intent``.

        The rectangle is centered on the intent's centerline, ``width_mm``
        wide, extended past ``intent.start`` (the boundary point) by
        ``overshoot_mm`` so the cut reaches cleanly through the skin, and
        stopping exactly at ``intent.end`` (the margin point) with a square
        cap — no extension there, since overshooting the far end would
        violate the skin margin invariant it exists to protect.
        """
        if overshoot_mm <= 0:
            raise ValueError(
                "overshoot_mm must be > 0, got {!r}; a non-positive overshoot "
                "cannot guarantee the cut reaches through the skin "
                "(the slot must start exactly on the "
                "boundary).".format(overshoot_mm)
            )
        if intent.width_mm <= 0:
            raise ValueError(
                "WebIntent.width_mm must be > 0, got {!r}.".format(intent.width_mm)
            )

        start_x, start_y = intent.start
        end_x, end_y = intent.end

        dx, dy = start_x - end_x, start_y - end_y
        length = math.hypot(dx, dy)
        if length == 0:
            raise ValueError(
                "WebIntent has zero-length centerline (start == end == {!r}); "
                "cannot derive a slot direction.".format(intent.start)
            )
        # Unit vector pointing from the margin end toward (and past) the
        # boundary end — "outward".
        ux, uy = dx / length, dy / length
        # Perpendicular unit vector, used to lay out the slot's width.
        px, py = -uy, ux

        overshoot_x = start_x + ux * overshoot_mm
        overshoot_y = start_y + uy * overshoot_mm

        half_w = intent.width_mm / 2.0

        socket_r = getattr(intent, "socket_radius_mm", None)
        if socket_r:
            return SlotPolygon(points=self._keyhole(
                overshoot_x, overshoot_y, end_x, end_y,
                ux, uy, px, py, half_w, socket_r))

        corners = (
            (overshoot_x + px * half_w, overshoot_y + py * half_w),
            (end_x + px * half_w, end_y + py * half_w),
            (end_x - px * half_w, end_y - py * half_w),
            (overshoot_x - px * half_w, overshoot_y - py * half_w),
        )
        return SlotPolygon(points=corners)

    @staticmethod
    def _keyhole(ox, oy, cx, cy, ux, uy, px, py, half_w, r, arc_steps=72):
        """Carbon-spar socket: a neck from the skin down to a circular socket
        centred on ``intent.end``, as ONE simply-connected void that touches
        the boundary.

        WHY THIS SHAPE. Spiral Vase deletes interior holes before the plugin
        runs, so a spar channel modelled in CAD never reaches ``posSlice``
        and there is no hole to connect to the skin. A wall around the outside AND a wall around the
        spar is two loops; strict vase mode prints one. They reconcile only if
        joined -- neck plus socket gives the nozzle a single closed path:
        skin -> down the neck -> around the socket -> back up -> skin.

        The socket is therefore NOT a rescued hole. It is an ordinary
        boundary-connected cut that happens to be round at its far end, and it
        goes through exactly the same erosion and one-loop validation as every
        other slot.

        CONSTRUCTION. The neck's two sides meet the circle at +/- asin(half_w/r)
        from the neck axis. The ring is then walked the LONG way round (away
        from the neck) so the outline never crosses itself and encloses no
        interior ring. A neck half-width at or above ``r`` has no such
        intersection with the socket circle -- it would be a plain rectangle
        with a bulge, which is a different shape than requested, and this
        FAILS CLOSED rather than silently clamping to a narrower neck than
        the caller asked for. ``StructureSpec.validate`` rejects the same
        condition earlier, at spec-validation time, from
        ``spar_neck_width_mm``/``spar_diameter_mm``/``spar_clearance_mm``;
        this raise is the defensive backstop for ``_keyhole``'s other direct
        callers (it is a public primitive, reachable without going through
        ``StructureSpec`` at all).
        """
        if half_w >= r:
            raise SpecError(
                "keyhole neck half-width {:.4f} mm >= socket radius {:.4f} "
                "mm; a neck this wide does not intersect the socket circle "
                "and would silently become a plain rectangle instead of a "
                "keyhole. Reduce the neck width (WebIntent.width_mm), or "
                "increase the socket radius (WebIntent.socket_radius_mm)."
                .format(half_w, r)
            )
        a = math.asin(max(-1.0, min(1.0, half_w / r)))

        pts = [(ox + px * half_w, oy + py * half_w),
               (ox - px * half_w, oy - py * half_w)]
        # around the socket the long way, from -a to +a going the far side
        # angle, about the socket centre, of the direction back toward the
        # skin. The neck meets the circle at base -/+ a, so the arc must START
        # at base - a and sweep the LONG way round. Using the perpendicular
        # here instead puts the arc start 90 degrees out and the outline
        # crosses itself -- GEOS reports "side location conflict".
        base = math.atan2(uy, ux)
        start_th = base - a
        sweep = -(2.0 * math.pi - 2.0 * a)
        # NOTE: the arc's own first point (i == 0, th == base - a) is ALREADY
        # the neck/circle intersection on the -p side -- algebraically
        # identical to (cx + ux*r*cos(a) - px*half_w, cy + uy*r*cos(a) -
        # py*half_w), since half_w == r*sin(a) by construction of ``a``. A
        # separately appended "down the -p side of the neck" point would
        # duplicate this vertex and add a zero-length edge to every socket
        # polygon, so none is appended; likewise the loop's LAST point is
        # not duplicated on the +p side.
        for i in range(arc_steps + 1):
            th = start_th + sweep * i / arc_steps
            pts.append((cx + r * math.cos(th), cy + r * math.sin(th)))
        # WINDING: this construction order (top +p -> top -p -> down -p ->
        # around the LONG way -> up to +p) is CLOCKWISE, but ``SlotPolygon``'s
        # own contract (see its docstring) is counter-clockwise, matching the
        # rectangle branch above. Reverse once, at the end, rather than
        # re-deriving the construction in the other direction -- the shape
        # (self-intersection-free, one simple ring) is unaffected by winding
        # direction, only its sign.
        pts.reverse()
        return tuple(pts)
