"""The Z-continuity rule, behind ``StructureSpec.z_continuity``.

No Orca imports, no geometry backend of its own: it drives the real ``StructureGenerator`` and
``TopologyValidator`` through callbacks, so the in-host capability
(``plugin/aircraft_structures/capability.py::_transform``) and the mesh replay
(``tools/preflight_wing.py::t3_layer_sweep``) share THIS code, not two copies.

THE RULE (per web ``w = (family_index, k, side)``, layer ``i``, causal):

    d_w(i) = min(D_w(i), d_exec_w(i-1) + r)

* ``d_exec_w(i-1)`` is the depth ACTUALLY EXECUTED on the previous layer (after
  a merge, coupled-pass shallowing or omission), 0 when the web was absent
  or when its slot side walls moved by more than ``dx_max`` since then.
* ``r`` is the ramp step, the live layer height (45 degrees).
* ``D_w(i)`` is the deepest cut that passes the generator's per-station checks
  on this layer's section: nominal when the nominal depth passes, otherwise
  found by bisection over a depth cap with the REAL generator confirming
  (``LayerContext.depth_caps``); 0 for a station-killing check.
* The ramp cap reaches the generator through ``LayerContext.depth_caps``: the
  ``_cap_end_y`` seams in ``_generate_family`` and the opposing-pair family.
* WALL-MATCHING VARIANT (``wall_matching=True``, spec key
  ``z_continuity_wall_matching``). The "same web" test above looks at a
  web's OWN previous walls, so a merge (the inner rib walls vanish and
  each constituent seems to jump) and a swept lattice read as lateral jumps
  and restart every constituent. The variant asks the physical question
  instead: is each printed slot wall of layer ``i`` supported by SOME printed
  wall of the same orientation (left edge / right edge of a slot) and the
  same entry side (U / L) on layer ``i-1`` within ``dx_max``? The support of a
  wall is the DEEPEST such previous wall; the slot may be at most
  ``min(over its two walls: support) + r`` deep, and the per-web cap
  ``d_exec(i-1) + r`` still applies. A merged slot's outer walls match the
  constituents' outer walls (the vanished inner walls cost nothing, and a
  merged slot keeps its depth). When a merged slot splits, each new web's new
  INNER wall has nothing below it (support 0, no match within dx_max): that
  web is a RESTART, its depth capped at ``r`` on the first layer after the
  split and growing by ``r`` per layer (the inner wall matches itself on the
  next layer). With the flag off none of this code runs: the strict per-web rule applies.
* Sockets (family index < 0) are exempt from the ramp; a spar socket that would
  restart after an END is suppressed here (the latch already does the
  same in-host) and recorded.
* The first structured layer (``index <= root_solid_layers``) is exempt: it
  sits on the solid bed layers.

THE VALIDATOR IN THE LOOP: ``validate_with_ablation`` runs the full
``TopologyValidator.validate``; on the island-split cause it ablates ONE slot at
a time (cheapest structure first); the first removal that restores one island
and a fully valid layer is the culprit, which ENDs (exec depth 0, so it can only
come back by ramp). If no single removal fixes it the layer fails closed.

WHOLE-LAYER SHUTDOWN only for a genuinely re-entrant section: the bare section,
eroded at the wall-safety offset, has more than one island
(``genuine_reentrant``). A notch is handled per station.

HONEST OPTIMALITY: for a single web the rule is the pointwise-maximal feasible
schedule (feasible, not provably maximal: the bisection confirms each depth
with the real generator but the feasible set need not be downward-closed).
Across coupled webs (opposing entry, role yield, merge) it is greedy: those
passes pick their own victim, which is always the shallower, ramping web.
"""

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

from .continuity import (
    DEFAULT_DX_MAX_MM,
    WALL_LEFT,
    WALL_RIGHT,
    wall_support,
    intent_web_ids,
    is_confirmed_spar_id,
    omission_code,
)
from .frame import ReEntrantSectionError
from .generator import OmittedWeb
from .validate import ISLAND_SPLIT_CAUSE_MARKER, TopologyError

Z_RULE_VERSION = 1

# Omission codes that a shallower cut can cure (every other code is a station
# killing check or a coupled-pass loss, which bisection cannot help).
_ZR_DEPTH_DEPENDENT_CODES = frozenset({
    "mitre_clearance", "thin_section", "skin_margin",
})
_ZR_DEPTH_TOL_MM = 1e-6
_ZR_STEP_TOL_MM = 1e-6
_ZR_MAX_X_ITERATIONS = 6
_ZR_MAX_BISECT_ITERATIONS = 9
_ZR_EVENT_CAP = 400

TIE_INDEPENDENT = "independent"
TIE_JOINT = "joint"


class _ZRuleCaps(dict):
    """``depth_caps`` mapping with a default: a web the rule has no state for
    (never seen, absent last layer) may start at ``default`` (one ramp step).
    ``__bool__`` is always True so the generator's ``if not depth_caps`` test
    treats an empty map as "caps in force"."""

    def __init__(self, default: float):
        super().__init__()
        self.default = float(default)

    def __bool__(self) -> bool:
        return True

    def get(self, key, default=None):
        if key in self:
            return dict.__getitem__(self, key)
        return self.default


def _zr_ids(intent) -> Tuple[tuple, ...]:
    """Every web identity an intent carries, keyed exactly as the report
    rows are (``continuity.intent_web_ids``)."""
    return intent_web_ids(intent)


def _zr_walls(intent) -> Tuple[float, float]:
    half = 0.5 * float(getattr(intent, "width_mm", 0.0) or 0.0)
    return intent.start[0] - half, intent.start[0] + half


def _zr_depth(intent) -> float:
    return abs(intent.end[1] - intent.start[1])


def _zr_is_socket(wid) -> bool:
    return wid[0] < 0


@dataclass
class _ZRulePrev:
    depth: float
    left: float
    right: float
    merged: bool = False        # sat in a merged slot on that layer


@dataclass(frozen=True)
class WebAblationRescue:
    """Result of the in-loop single-web ablation (the Z-aware counterpart of pair ablation)."""

    layer_index: int
    print_z: float
    removed_indices: Tuple[int, ...]
    kept_indices: Tuple[int, ...]
    ended_ids: Tuple[tuple, ...]
    omitted: Tuple[OmittedWeb, ...]
    tried: int
    elapsed_ms: float
    reason: str


def genuine_reentrant(ops, polygon, safety_offset_mm: float) -> Optional[dict]:
    """Re-entrant-section predicate: ``None`` for an ordinary section (a notch
    included), else ``{"islands": n, "areas_mm2": [...], "message":...}`` when
    the bare section, eroded at ``safety_offset_mm`` with no slots cut, has
    more than one island. A section that erodes to nothing (a collapsing tip)
    is NOT re-entrant -- it is the thin-tip case the existing shutdowns own."""
    eroded = ops.offset(polygon, -float(safety_offset_mm))
    if len(eroded) <= 1:
        return None
    areas = sorted((float(ops.area(piece)) for piece in eroded), reverse=True)
    return {
        "islands": len(eroded),
        "areas_mm2": [round(a, 3) for a in areas],
        "message": (
            "genuinely re-entrant section: the bare section (no slots cut) "
            "erodes at the {:.2f} mm wall-safety offset into {} islands "
            "(areas {} mm^2), so no web placement can keep it one connected "
            "perimeter.".format(
                safety_offset_mm, len(eroded),
                ", ".join("{:.2f}".format(a) for a in areas))
        ),
    }


class ZContinuityRule:
    """Per-run state and the per-layer procedures of the rule."""

    def __init__(
        self,
        *,
        ramp_mm: float,
        dx_max_mm: float,
        tie_mode: str = TIE_INDEPENDENT,
        d_min_mm: float = 0.05,
        safety_offset_mm: float = 0.42,
        root_solid_layers: int = 0,
        tie_family_indices: Sequence[int] = (),
        family_names: Optional[Dict[int, str]] = None,
        wall_matching: bool = False,
    ):
        if not ramp_mm > 0:
            raise ValueError("ramp_mm must be > 0, got {!r}".format(ramp_mm))
        if not dx_max_mm > 0:
            raise ValueError("dx_max_mm must be > 0, got {!r}".format(dx_max_mm))
        if tie_mode not in (TIE_INDEPENDENT, TIE_JOINT):
            raise ValueError("unknown tie_mode {!r}".format(tie_mode))
        self.ramp_mm = float(ramp_mm)
        self.dx_max_mm = float(dx_max_mm)
        self.tie_mode = tie_mode
        self.wall_matching = bool(wall_matching)
        self.d_min_mm = float(d_min_mm)
        self.safety_offset_mm = float(safety_offset_mm)
        self.root_solid_layers = int(root_solid_layers)
        self.tie_families = frozenset(int(i) for i in tie_family_indices)
        self.family_names = dict(family_names or {})
        self.reset()

    @classmethod
    def from_spec(cls, spec, *, ramp_mm: float, safety_offset_mm: float):
        """Build from a ``StructureSpec`` (the same call in-host and offline)."""
        dx = spec.z_continuity_dx_max_mm
        families = spec.effective_families()
        return cls(
            ramp_mm=ramp_mm,
            dx_max_mm=DEFAULT_DX_MAX_MM if dx is None else float(dx),
            tie_mode=spec.z_continuity_tie_mode,
            d_min_mm=float(spec.overshoot_mm),
            safety_offset_mm=safety_offset_mm,
            root_solid_layers=int(spec.root_solid_layers),
            tie_family_indices=[
                i for i, fam in enumerate(families) if fam.opposing_pair],
            family_names={i: fam.name for i, fam in enumerate(families)},
            wall_matching=bool(getattr(spec, "z_continuity_wall_matching", False)),
        )

    # ------------------------------------------------------------------ state

    def reset(self) -> None:
        self.prev: Dict[tuple, _ZRulePrev] = {}
        self.last_index: Optional[int] = None
        self.ended_sockets: Set[tuple] = set()
        self.ramp_open: Dict[tuple, dict] = {}
        self._caps = None
        self._zeroed: Set[tuple] = set()
        self._pending_x_ends: List[tuple] = []
        self._jump_merged: Dict[tuple, bool] = {}
        self._layer_bisected = 0
        self._layer_gens = 0
        self._t_layer = None
        self.events: List[dict] = []
        self.counters: Dict[str, int] = {
            "layers_committed": 0, "layers_skipped": 0,
            "generator_calls": 0, "bisect_layers": 0, "bisect_found": 0,
            "bisect_hopeless": 0, "x_jump_ends": 0, "x_jump_ends_merged": 0,
            "socket_jumps_suppressed": 0, "ablation_ends": 0,
            "ablation_layers": 0, "genuine_reentrant_layers": 0,
            "socket_restarts_suppressed": 0, "ramp_web_layers": 0,
            "ramp_starts": 0, "ramps_completed": 0, "ramps_never_completed": 0,
            "deepening_ramps_never_completed": 0,
            "web_layers": 0, "ends_absent": 0,
            "wall_limited_webs": 0, "wall_cap_not_honoured": 0,
        }
        self.depth_mm_by_family: Dict[int, float] = {}
        # per-family tallies of the rule's own actions (web-layers / events)
        self.by_family: Dict[str, Dict[int, int]] = {
            "ramp_web_layers": {}, "ends_absent": {}, "x_jump_ends": {},
            "ablation_ends": {}, "bisect_found": {}}
        self.tie_layers = {"fused": 0, "unfused_stub": 0, "one_sided": 0,
                           "ended": 0}
        self.never_completed: List[dict] = []
        self.time_s = {"generate": 0.0, "validate": 0.0, "commit": 0.0}
        self.layer_ms: Dict[int, float] = {}

    def _tally(self, key: str, wid) -> None:
        bucket = self.by_family[key]
        bucket[wid[0]] = bucket.get(wid[0], 0) + 1

    def _event(self, kind: str, **detail) -> None:
        if len(self.events) < _ZR_EVENT_CAP or kind in (
                "ablation_end", "genuine_reentrant"):
            rec = {"kind": kind}
            rec.update(detail)
            self.events.append(rec)

    # --------------------------------------------------------------- per layer

    def begin_layer(self, index: int, print_z: float = 0.0) -> None:
        """Call once per structured layer before generating. A layer the
        capability skipped between the previous commit and this one (shutdown,
        multi-island, tip skip) left no structure: every web ENDed there.

        The rule tracks ONE structured region per layer: a second region on a
        layer already committed would silently mix two regions' web state, so
        it fails closed instead."""
        if self.last_index is not None and index == self.last_index and (
                self.counters["layers_committed"] > 0):
            raise TopologyError(
                index, print_z,
                "z_continuity tracks one structured region per layer; layer "
                "{} was already committed (a second region on the same layer "
                "would mix two regions' web state).".format(index))
        if self.last_index is not None and index != self.last_index + 1:
            self._clear_all("gap", index)
        self._layer_bisected = 0
        self._layer_gens = 0
        self._t_layer = time.perf_counter()

    def note_skipped(self, index: int, reason: str) -> None:
        """A layer produced no structure (whole-layer shutdown, unmutated):
        every web ENDs there, so the next layer restarts every web by ramp."""
        self.counters["layers_skipped"] += 1
        self._clear_all(reason, index)
        self.last_index = index

    def _clear_all(self, reason: str, index: int) -> None:
        for wid in list(self.ramp_open):
            self._close_ramp_never_completed(wid, index, reason)
        if self.prev:
            self.counters["ends_absent"] += len(self.prev)
        for wid in self.prev:
            if _zr_is_socket(wid):
                self.ended_sockets.add(wid)
        self.prev = {}

    def reentrant_check(self, ops, polygon) -> Optional[dict]:
        """``genuine_reentrant`` at this run's safety offset; the caller turns
        a hit into the existing whole-layer ``ReEntrantSectionError``
        shutdown."""
        hit = genuine_reentrant(ops, polygon, self.safety_offset_mm)
        if hit is not None:
            self.counters["genuine_reentrant_layers"] += 1
        return hit

    def raise_genuine(self, hit: dict) -> None:
        raise ReEntrantSectionError(hit["message"])

    # ------------------------------------------------------------------ caps

    def _exempt_layer(self, index: int) -> bool:
        return index <= self.root_solid_layers

    def _tie_group(self, wid) -> Optional[Tuple[int, int]]:
        if self.tie_mode == TIE_JOINT and wid[0] in self.tie_families and wid[2] in ("U", "L"):
            return (wid[0], wid[1])
        return None

    def _ramp_caps(self, zeroed: Set[tuple], wall_caps=None) -> _ZRuleCaps:
        caps = _ZRuleCaps(self.ramp_mm)
        for wid, prev in self.prev.items():
            if _zr_is_socket(wid) or wid in zeroed:
                continue
            caps[wid] = prev.depth + self.ramp_mm
        if wall_caps:
            for wid, allowed in wall_caps.items():
                caps[wid] = min(caps.get(wid), allowed)
        if self.tie_mode == TIE_JOINT:
            groups: Dict[Tuple[int, int], List[tuple]] = {}
            for wid in list(caps.keys()):
                grp = self._tie_group(wid)
                if grp is not None:
                    groups.setdefault(grp, []).append(wid)
            for grp, _ids in groups.items():
                members = [(grp[0], grp[1], "U"), (grp[0], grp[1], "L")]
                value = min(
                    (caps[m] if m in caps and m not in zeroed else self.ramp_mm)
                    for m in members)
                for m in members:
                    caps[m] = value
        return caps

    def _x_jump_ids(self, intents, zeroed: Set[tuple]) -> Dict[tuple, float]:
        """Webs whose slot side walls moved by more than ``dx_max`` since the
        previous layer while carrying depth. Returns ``{id: step_mm}`` and
        remembers which of them are merge-involved (in a merged slot on this
        layer or the previous one) in ``self._jump_merged``."""
        jumped: Dict[tuple, float] = {}
        for intent in intents:
            left, right = _zr_walls(intent)
            ids = _zr_ids(intent)
            for wid in ids:
                if _zr_is_socket(wid) or wid in zeroed:
                    continue
                prev = self.prev.get(wid)
                if prev is None or prev.depth <= 0.0:
                    continue
                step = max(abs(left - prev.left), abs(right - prev.right))
                if step > self.dx_max_mm + _ZR_STEP_TOL_MM:
                    jumped[wid] = step
                    # merge-involved: in a merged slot now, or was in one
                    # on the previous layer (a split)
                    self._jump_merged[wid] = len(ids) > 1 or prev.merged
        if self.tie_mode == TIE_JOINT:
            for wid in list(jumped):
                grp = self._tie_group(wid)
                if grp is not None:
                    for side in ("U", "L"):
                        partner = (grp[0], grp[1], side)
                        if partner not in zeroed:
                            jumped.setdefault(partner, jumped[wid])
        return jumped

    @staticmethod
    def _present_ids(result) -> Dict[tuple, float]:
        present: Dict[tuple, float] = {}
        for intent in result.intents:
            d = _zr_depth(intent)
            for wid in _zr_ids(intent):
                present[wid] = d
        return present

    def generate_layer(self, index: int, print_z: float, gen):
        """The rule's generation step. ``gen(depth_caps)`` must run the real
        ``StructureGenerator.generate`` with ``LayerContext.depth_caps=
        depth_caps`` and return its ``GenerationResult``. Returns the final
        ``GenerationResult`` (every depth confirmed by the real generator)."""
        self.begin_layer(index, print_z)
        t0 = time.perf_counter()
        if self._exempt_layer(index):
            self._caps = None
            self._zeroed = set()
            self._layer_gens += 1
            self.counters["generator_calls"] += 1
            res = gen(None)
            self.time_s["generate"] += time.perf_counter() - t0
            return res

        if self.wall_matching:
            res = self._generate_wall_matching(index, print_z, gen)
            self.time_s["generate"] += time.perf_counter() - t0
            return res

        zeroed: Set[tuple] = set()
        res = None
        caps = None
        for _attempt in range(_ZR_MAX_X_ITERATIONS):
            caps = self._ramp_caps(zeroed)
            res = self._call(gen, caps)
            res, caps = self._bisect(res, caps, gen, index)
            jumped = self._x_jump_ids(res.intents, zeroed)
            if not jumped:
                break
            # Regenerate only when a jumped web is actually deeper than one
            # ramp step (otherwise zeroing it cannot change the cut).
            present = self._present_ids(res)
            needs_regen = any(
                present.get(wid, 0.0) > self.ramp_mm + _ZR_STEP_TOL_MM
                for wid in jumped)
            for wid, step in jumped.items():
                self._pending_x_ends.append((wid, step))
            zeroed |= set(jumped)
            if not needs_regen:
                caps = self._ramp_caps(zeroed)
                break
        else:
            raise TopologyError(
                index, print_z,
                "z_continuity: the lateral-continuity iteration (webs whose "
                "slot side walls jumped by more than {:.3f} mm restart from "
                "zero) did not converge in {} generator passes.".format(
                    self.dx_max_mm, _ZR_MAX_X_ITERATIONS))
        self._caps = caps
        self._zeroed = zeroed
        self.time_s["generate"] += time.perf_counter() - t0
        return res

    # ------------------------------------------------ wall matching

    def _prev_walls(self) -> Dict[tuple, List[Tuple[float, float]]]:
        """Every printed non-socket wall of the previous committed layer, as
        ``{(entry side, orientation): [(x, slot depth), ...]}``."""
        out: Dict[tuple, List[Tuple[float, float]]] = {}
        for wid, st in self.prev.items():
            if _zr_is_socket(wid):
                continue
            out.setdefault((wid[2], WALL_LEFT), []).append((st.left, st.depth))
            out.setdefault((wid[2], WALL_RIGHT), []).append((st.right, st.depth))
        return out

    def _wall_limits(self, intents, walls_prev):
        """For every non-socket web of ``intents``: the deepest slot its two
        walls' support allows, ``min(support) + r`` (support = the deepest
        previous wall of the same side and orientation within ``dx_max``; 0
        when none). Returns ``(limits, restarts)``: ``limits`` has the webs
        whose current slot is deeper than that; ``restarts`` has the webs that
        carried depth on the previous layer (``prev.depth > 0``) and have a
        wall with NO matching wall below at all (``{id: (offset_mm,
        merged_involved)}``)."""
        r = self.ramp_mm
        reach = self.dx_max_mm + _ZR_STEP_TOL_MM
        limits: Dict[tuple, float] = {}
        restarts: Dict[tuple, Tuple[float, bool]] = {}
        for intent in intents:
            ids = [w for w in _zr_ids(intent) if not _zr_is_socket(w)]
            if not ids:
                continue
            left, right = _zr_walls(intent)
            depth = _zr_depth(intent)
            for wid in ids:
                support = []
                unmatched_offset = None
                for x, orient in ((left, WALL_LEFT), (right, WALL_RIGHT)):
                    best, nearest = wall_support(
                        walls_prev.get((wid[2], orient), ()), x, reach)
                    if best is None:
                        support.append(0.0)
                        off = 1e3 if nearest is None else abs(nearest)
                        unmatched_offset = (
                            off if unmatched_offset is None
                            else max(unmatched_offset, off))
                    else:
                        support.append(best)
                allowed = min(support) + r
                if depth > allowed + _ZR_DEPTH_TOL_MM:
                    limits[wid] = min(limits.get(wid, allowed), allowed)
                prev = self.prev.get(wid)
                if (unmatched_offset is not None and prev is not None
                        and prev.depth > 0.0):
                    restarts[wid] = (
                        unmatched_offset, len(_zr_ids(intent)) > 1 or prev.merged)
        return limits, restarts

    def _generate_wall_matching(self, index, print_z, gen):
        walls_prev = self._prev_walls()
        wall_caps: Dict[tuple, float] = {}
        restarted: Dict[tuple, Tuple[float, bool]] = {}
        res = None
        caps = None
        for _attempt in range(_ZR_MAX_X_ITERATIONS):
            caps = self._ramp_caps(set(restarted), wall_caps)
            res = self._call(gen, caps)
            res, caps = self._bisect(res, caps, gen, index)
            limits, restarts = self._wall_limits(res.intents, walls_prev)
            for wid, info in restarts.items():
                restarted.setdefault(wid, info)
            changed = {w: a for w, a in limits.items()
                       if a < wall_caps.get(w, float("inf")) - _ZR_DEPTH_TOL_MM}
            if not changed:
                if limits:
                    # the generator built a slot deeper than the cap it was
                    # handed: counted, and left to the continuity checker.
                    self.counters["wall_cap_not_honoured"] += len(limits)
                    self._event("wall_cap_not_honoured", layer=index,
                                webs=[list(w) for w in sorted(limits)][:20])
                break
            wall_caps.update(changed)
        else:
            raise TopologyError(
                index, print_z,
                "z_continuity (wall matching): the wall-support iteration did "
                "not converge in {} generator passes.".format(
                    _ZR_MAX_X_ITERATIONS))
        for wid in restarted:
            if wid not in wall_caps:
                caps[wid] = min(caps.get(wid), self.ramp_mm)
        self.counters["wall_limited_webs"] += len(
            [w for w in wall_caps if w not in restarted])
        for wid, (offset, merged) in restarted.items():
            self._pending_x_ends.append((wid, offset))
            self._jump_merged[wid] = merged
        self._caps = caps
        self._zeroed = set(restarted)
        return res

    def _call(self, gen, caps):
        self._layer_gens += 1
        self.counters["generator_calls"] += 1
        return gen(caps)

    # ------------------------------------------------------------- bisection

    def _bisect(self, res, caps, gen, index):
        """Find, for every web the generator omitted at its ramp cap for a
        depth-dependent reason, the deepest cap at which the REAL generator
        places it. Batched: one generator call per bisection step serves every
        pending web. Returns ``(result, caps)`` (a result of the final,
        confirmed pass)."""
        pending: Dict[tuple, List[float]] = {}
        for om in res.omitted:
            ids = tuple(getattr(om, "merged_web_ids", ()) or ())
            if not ids:
                wid = getattr(om, "web_id", None)
                ids = (wid,) if wid is not None else ()
            code = omission_code(om.reason)
            if code not in _ZR_DEPTH_DEPENDENT_CODES:
                continue
            for wid in ids:
                if _zr_is_socket(wid) or wid[2] not in ("U", "L"):
                    continue
                hi = caps.get(wid)
                if hi is None or hi <= self.d_min_mm + _ZR_DEPTH_TOL_MM:
                    continue
                pending[wid] = [0.0, float(hi)]
        if not pending:
            return res, caps
        self.counters["bisect_layers"] += 1

        def _present(result):
            return self._present_ids(result)

        # One probe pass at twice the degenerate floor: a station that fails
        # even there is hopeless (D = 0), and is dropped from the search.
        probe = 2.0 * self.d_min_mm
        trial = _ZRuleCaps(caps.default)
        trial.update(caps)
        for wid, (lo, hi) in pending.items():
            trial[wid] = min(probe, hi)
        res_probe = self._call(gen, trial)
        have = _present(res_probe)
        live: Dict[tuple, List[float]] = {}
        for wid, (lo, hi) in pending.items():
            if wid in have:
                live[wid] = [min(probe, hi), hi]
            else:
                self.counters["bisect_hopeless"] += 1
        if not live:
            return res, caps
        found: Dict[tuple, float] = {wid: iv[0] for wid, iv in live.items()}
        for _ in range(_ZR_MAX_BISECT_ITERATIONS):
            active = {w: iv for w, iv in live.items()
                      if iv[1] - iv[0] > self.d_min_mm}
            if not active:
                break
            trial = _ZRuleCaps(caps.default)
            trial.update(caps)
            for wid, iv in live.items():
                trial[wid] = found[wid]
            for wid, iv in active.items():
                trial[wid] = 0.5 * (iv[0] + iv[1])
            got = _present(self._call(gen, trial))
            for wid, iv in active.items():
                mid = trial[wid]
                if wid in got:
                    iv[0] = mid
                    found[wid] = mid
                else:
                    iv[1] = mid
        final = _ZRuleCaps(caps.default)
        final.update(caps)
        for wid, depth in found.items():
            final[wid] = depth
        res_final = self._call(gen, final)
        have = _present(res_final)
        n_found = 0
        for wid, depth in found.items():
            if wid in have:
                n_found += 1
                self._event("bisect_found", layer=index, web=list(wid),
                            depth_mm=round(depth, 3),
                            ramp_cap_mm=round(caps.get(wid), 3))
                self._tally("bisect_found", wid)
        self.counters["bisect_found"] += n_found
        self._layer_bisected = n_found
        return res_final, final

    # -------------------------------------------------- sockets (post-latch)

    def post_latch(self, index: int, intents, omitted):
        """Suppress the CONFIRMED spar socket ``(-2, 0, "S")`` when it would
        restart after its END or when its slot walls jumped sideways (a
        different hole). Sockets cannot ramp; ends stay ended. Unconfirmed
        picks and bores are different holes sharing a label and are left
        alone (``continuity.SIDE_SOCKET_UNCONFIRMED``). Returns ``(intents,
        omitted, suppressed_ids)``. Never invoked on a layer where the socket
        was never seen."""
        keep = []
        extra: List[OmittedWeb] = []
        suppressed: List[tuple] = []
        for intent in intents:
            ids = _zr_ids(intent)
            restart = [w for w in ids
                       if is_confirmed_spar_id(w) and w in self.ended_sockets]
            if not restart:
                # A socket whose slot walls jumped sideways is a DIFFERENT
                # hole under the same label (the spar socket flips to the
                # alignment pin where the spar ends: inside half layer 1000,
                # 7.6 -> 58.6 mm). The old socket ENDed; sockets cannot ramp,
                # so the new one is refused like any restart.
                left, right = _zr_walls(intent)
                for w in ids:
                    prev = self.prev.get(w)
                    if (is_confirmed_spar_id(w) and prev is not None
                            and prev.depth > 0.0
                            and max(abs(left - prev.left),
                                    abs(right - prev.right))
                            > self.dx_max_mm + _ZR_STEP_TOL_MM):
                        restart.append(w)
                        self.ended_sockets.add(w)
                        self.counters["socket_jumps_suppressed"] += 1
            if restart:
                suppressed.extend(restart)
                for wid in ids:
                    extra.append(OmittedWeb(
                        x_mm=intent.start[0],
                        reason=("z_socket_latch: this socket ENDED (earlier, or "
                                "its slot walls jumped sideways so this is a "
                                "different hole) and sockets cannot ramp, so "
                                "it stays ended (a restart would leave a "
                                "blind pocket)."),
                        family_index=intent.family_index,
                        family_name=intent.family_name,
                        k=wid[1], web_id=wid))
                self.counters["socket_restarts_suppressed"] += 1
                self._event("socket_restart_suppressed", layer=index,
                            web=[list(w) for w in restart])
                continue
            keep.append(intent)
        if suppressed:
            omitted = list(omitted) + extra
        return keep, omitted, suppressed

    # -------------------------------------------------------------- validator

    def validate_with_ablation(
        self, ops, validator, originals, replacement, intents, slots, spec,
        index: int, print_z: float, slot_labels, family_indices,
    ) -> Optional[WebAblationRescue]:
        """``validator.validate`` with in-loop single-web ablation. ``None``
        when the layer validates as it stands; a ``WebAblationRescue`` when
        ENDing one web (its whole slot) restored a valid layer; otherwise a
        ``TopologyError`` (the original unchanged unless ablation was tried)."""
        t0 = time.perf_counter()
        try:
            validator.validate(
                originals, replacement, slots, spec, index, print_z,
                slot_labels=slot_labels, family_indices=family_indices)
            self.time_s["validate"] += time.perf_counter() - t0
            return None
        except TopologyError as exc:
            self.time_s["validate"] += time.perf_counter() - t0
            if ISLAND_SPLIT_CAUSE_MARKER not in getattr(exc, "cause", ""):
                raise
            return self._ablate(
                exc, ops, validator, originals, intents, slots, spec, index,
                print_z, slot_labels, family_indices)

    def _ablate(self, error, ops, validator, originals, intents, slots, spec,
                index, print_z, slot_labels, family_indices):
        t0 = time.perf_counter()
        if len(originals) != 1 or not slots:
            raise error
        original = originals[0]
        safety = self.safety_offset_mm
        order = []
        for i, intent in enumerate(intents):
            ids = _zr_ids(intent)
            if not ids or any(_zr_is_socket(w) for w in ids):
                continue
            order.append((_zr_depth(intent), ids[0], i))
        order.sort()
        tried = 0
        for _depth, _wid, i in order:
            removed = {i}
            if self.tie_mode == TIE_JOINT:
                grp = self._tie_group(_zr_ids(intents[i])[0])
                if grp is not None:
                    for j, other in enumerate(intents):
                        oids = _zr_ids(other)
                        if oids and self._tie_group(oids[0]) == grp:
                            removed.add(j)
            keep = [j for j in range(len(slots)) if j not in removed]
            kept_slots = [slots[j] for j in keep]
            tried += 1
            try:
                pieces = ops.diff(original, kept_slots)
                if len(pieces) != 1 or len(ops.offset(pieces[0], -safety)) != 1:
                    continue
                validator.validate(
                    originals, list(pieces), kept_slots, spec, index, print_z,
                    slot_labels=([slot_labels[j] for j in keep]
                                 if slot_labels is not None else None),
                    family_indices=([family_indices[j] for j in keep]
                                    if family_indices is not None else None))
            except TopologyError:
                continue
            except Exception:  # noqa: BLE001 -- a backend that cannot re-diff
                continue
            ended: List[tuple] = []
            records: List[OmittedWeb] = []
            reason_ids = []
            for j in sorted(removed):
                intent = intents[j]
                ids = _zr_ids(intent)
                reason_ids.extend(ids)
                for wid in ids:
                    ended.append(wid)
                    records.append(OmittedWeb(
                        x_mm=intent.start[0],
                        reason=(
                            "z_ablation: layer {} (Z={:.2f} mm) split into "
                            "more than one island at safety_offset_mm={:.2f}; "
                            "ending this web alone ({}) restored one island "
                            "and a valid layer. It ENDs here and may only "
                            "return by ramp.".format(
                                index, print_z, safety,
                                slot_labels[j] if slot_labels is not None
                                else "slot {}".format(j + 1))),
                        family_index=intent.family_index,
                        family_name=intent.family_name,
                        k=wid[1], web_id=wid))
            elapsed = (time.perf_counter() - t0) * 1000.0
            self.counters["ablation_layers"] += 1
            self.counters["ablation_ends"] += len(set(ended))
            for wid in sorted(set(ended)):
                self._event("ablation_end", layer=index,
                            z=round(print_z, 4), web=list(wid))
                self._tally("ablation_ends", wid)
            return WebAblationRescue(
                layer_index=index, print_z=print_z,
                removed_indices=tuple(sorted(removed)),
                kept_indices=tuple(keep), ended_ids=tuple(sorted(set(ended))),
                omitted=tuple(records), tried=tried, elapsed_ms=elapsed,
                reason="z_ablation")
        raise TopologyError(
            index, print_z,
            error.cause
            + " Z-continuity single-web ablation was tried ({} slot(s), one at "
            "a time, whole layer re-validated at safety_offset_mm={:.2f}) and "
            "no single web's END restored the layer.".format(tried, safety)
        ) from error

    # ----------------------------------------------------------------- commit

    def _close_ramp_never_completed(self, wid, index, reason) -> None:
        info = self.ramp_open.pop(wid, None)
        if info is None:
            return
        key = ("ramps_never_completed" if info["from_zero"]
               else "deepening_ramps_never_completed")
        self.counters[key] += 1
        if len(self.never_completed) < 2000:
            self.never_completed.append({
                "web": list(wid), "start_layer": info["start"],
                "end_layer": index, "max_depth_mm": round(info["max"], 3),
                "from_zero": info["from_zero"], "end_reason": reason})

    def commit(self, index: int, print_z: float, intents, omitted=()) -> dict:
        """Record the layer actually built. Must be called with the FINAL
        intents (after latch, infeasible-slot drop and ablation)."""
        t0 = time.perf_counter()
        r = self.ramp_mm
        caps = self._caps
        cur: Dict[tuple, _ZRulePrev] = {}
        for intent in intents:
            left, right = _zr_walls(intent)
            depth = _zr_depth(intent)
            ids = _zr_ids(intent)
            for wid in ids:
                cur[wid] = _ZRulePrev(depth, left, right, len(ids) > 1)

        exempt = self._exempt_layer(index)
        limited: Set[tuple] = set()
        if caps is not None and not exempt:
            for wid, st in cur.items():
                if _zr_is_socket(wid):
                    continue
                cap = caps.get(wid)
                if cap is not None and st.depth >= cap - 1e-3:
                    limited.add(wid)

        # ENDs and ramps.
        zeroed = self._zeroed
        for wid, step in self._pending_x_ends:
            if wid in cur:
                self.counters["x_jump_ends"] += 1
                merged = bool(self._jump_merged.get(wid))
                if merged:
                    self.counters["x_jump_ends_merged"] += 1
                self._tally("x_jump_ends", wid)
                self._event("x_jump_end", layer=index, z=round(print_z, 4),
                            web=list(wid), step_mm=round(step, 4),
                            merged=merged)
        self._pending_x_ends = []
        self._jump_merged = {}
        for wid, st in self.prev.items():
            if wid not in cur:
                self.counters["ends_absent"] += 1
                self._tally("ends_absent", wid)
                if _zr_is_socket(wid):
                    self.ended_sockets.add(wid)
                if wid in self.ramp_open:
                    self._close_ramp_never_completed(wid, index, "ended")
        n_ramp = 0
        for wid, st in cur.items():
            if _zr_is_socket(wid):
                continue
            was_present = wid in self.prev and wid not in zeroed
            self.counters["web_layers"] += 1
            self.depth_mm_by_family[wid[0]] = (
                self.depth_mm_by_family.get(wid[0], 0.0) + st.depth)
            if exempt:
                continue
            if wid in limited:
                n_ramp += 1
                self.counters["ramp_web_layers"] += 1
                self._tally("ramp_web_layers", wid)
                info = self.ramp_open.get(wid)
                if info is None:
                    self.ramp_open[wid] = {
                        "start": index, "max": st.depth,
                        "from_zero": not was_present}
                    if not was_present:
                        self.counters["ramp_starts"] += 1
                else:
                    info["max"] = max(info["max"], st.depth)
            else:
                if wid in self.ramp_open:
                    self.ramp_open.pop(wid)
                    self.counters["ramps_completed"] += 1
        # An X-jump zeroing restarts a ramp: the old one never completed.
        for wid in zeroed:
            if wid in self.ramp_open and wid in cur and self.ramp_open[wid]["start"] != index:
                self._close_ramp_never_completed(wid, index, "x_jump")
                self.ramp_open[wid] = {
                    "start": index, "max": cur[wid].depth, "from_zero": True}
                self.counters["ramp_starts"] += 1

        # Tie station states (per layer, per station).
        stations: Dict[Tuple[int, int], Dict[str, tuple]] = {}
        for wid in list(cur) + [w for w in self.prev if w not in cur]:
            if wid[0] in self.tie_families and wid[2] in ("U", "L"):
                stations.setdefault((wid[0], wid[1]), {})[wid[2]] = (
                    wid in cur, wid in limited)
        for key, sides in stations.items():
            u = sides.get("U", (False, False))
            l = sides.get("L", (False, False))
            if u[0] and l[0]:
                self.tie_layers["unfused_stub" if (u[1] or l[1]) else "fused"] += 1
            elif u[0] or l[0]:
                self.tie_layers["one_sided"] += 1
            else:
                self.tie_layers["ended"] += 1

        self.prev = cur
        self.last_index = index
        self.counters["layers_committed"] += 1
        self.time_s["commit"] += time.perf_counter() - t0
        t_layer = getattr(self, "_t_layer", None)
        if t_layer is not None:
            self.layer_ms[index] = round((time.perf_counter() - t_layer) * 1000.0, 2)
        self._zeroed = set()
        return {"ramp_limited": n_ramp, "generator_calls": self._layer_gens,
                "bisected": self._layer_bisected}

    # ---------------------------------------------------------------- summary

    def summary(self) -> dict:
        # Ramps still open at the end of the run are neither completed nor
        # abandoned: they are reported separately.
        counts = dict(self.counters)
        return {
            "version": Z_RULE_VERSION,
            "ramp_mm": self.ramp_mm, "dx_max_mm": self.dx_max_mm,
            "wall_matching": self.wall_matching,
            "tie_mode": self.tie_mode, "d_min_mm": self.d_min_mm,
            "safety_offset_mm": self.safety_offset_mm,
            "counters": counts,
            "ramps_open_at_end": len(self.ramp_open),
            "depth_mm_by_family": {
                str(k): round(v, 3)
                for k, v in sorted(self.depth_mm_by_family.items())},
            "tie_station_layers": dict(self.tie_layers),
            "by_family": {
                key: {str(k): v for k, v in sorted(bucket.items())}
                for key, bucket in self.by_family.items()},
            "never_completed_ramps": list(self.never_completed),
            "events": list(self.events),
            "time_s": {k: round(v, 3) for k, v in self.time_s.items()},
        }
