"""Root-end-aware spar-socket latch.

Once the spar socket has ENDED it stays ended, because a socket that stopped
and restarted leaves a blind interior pocket the carbon rod cannot reach. The
rod enters from the ROOT, so "ended" only makes sense walking from the root
toward the tip. A latch that walked layers in host order (bottom to top) would
assume layer 0 is the root; on a tip-down print it would meet the channel's
TIP-side end first and suppress the socket over the whole rod-reachable spar
(measured on an outboard elevon half: 78 ``spar_socket`` then 833
``spar_socket_latched`` layers).

This module holds the rule itself as a pure function of per-layer statuses
(``aircraft_core.generator.SPAR_STATUS_*``), so it can be walked in EITHER
direction from a set of statuses precomputed before layer 0:

  * ``fit``      a confirmed socket is cut (no effect while un-armed)
  * ``not_fit``  a confirmed channel is present but the section cannot hold
                 it: ARMS the latch (a "spar_socket" degradation)
  * ``absent``   no confirmed channel: does NOT arm (two channels by
                 design are two sockets, not a resume after a loss) and
                 leaves the latch state alone
  * ``None``     layer not evaluated (root-solid layer, multi-island, empty
                 or re-entrant section, evaluation error): neutral

Once armed, every later ``fit`` layer (later = further from the root) is
suppressed. Walking from the bed (root at bed) is the bottom-to-top order;
``tests/unit/test_spar_latch.py`` covers the walk.
"""

from typing import Dict, List, Optional, Sequence

from .generator import SPAR_STATUS_FIT, SPAR_STATUS_NOT_FIT


def walk_order(n_layers: int, root_at_bed: bool) -> List[int]:
    """Layer indices in ROOT -> TIP order."""
    order = list(range(n_layers))
    return order if root_at_bed else order[::-1]


def walk_latch(statuses: Sequence[Optional[str]], root_at_bed: bool) -> Dict:
    """Apply arm/resume rule from the root end toward the tip.

    Returns ``{"arm_index", "suppressed", "suppressed_first", "suppressed_last",
    "fit_layers_before_arm", "walk_direction"}``. ``suppressed`` is a sorted
    list of layer indices whose confirmed socket must be dropped.
    """
    armed_at = None
    suppressed: List[int] = []
    fit_before = 0
    for idx in walk_order(len(statuses), root_at_bed):
        st = statuses[idx]
        if armed_at is None:
            if st == SPAR_STATUS_NOT_FIT:
                armed_at = idx
            elif st == SPAR_STATUS_FIT:
                fit_before += 1
        elif st == SPAR_STATUS_FIT:
            suppressed.append(idx)
    suppressed.sort()
    return {
        "walk_direction": "bed_to_top" if root_at_bed else "top_to_bed",
        "arm_index": armed_at,
        "suppressed": suppressed,
        "suppressed_count": len(suppressed),
        "suppressed_first": suppressed[0] if suppressed else None,
        "suppressed_last": suppressed[-1] if suppressed else None,
        "fit_layers_before_arm": fit_before,
    }

