"""The "refuse" ``TopologyError`` names the way out.

A default-path user hitting "2 spar channels detected... which channel is the
spar is a POLICY question the model alone cannot answer" must be told how to
answer that question: ``spar_channel_policy="nearest_spar_station"``.
Otherwise the "refuse" path is a dead end.

This test asserts ONLY that sentence. Every other substring the message must
contain is pinned in ``test_spar_channel_policy.py``'s
``RefusePolicyByteIdenticalTests``, deliberately not repeated here.
"""
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOLS_DIR = REPO_ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from aircraft_core.frame import LayerFrame  # noqa: E402
from aircraft_core.generator import (  # noqa: E402
    DetectedChannel,
    LayerContext,
    StructureGenerator,
)
from aircraft_core.spec import StructureSpec  # noqa: E402
from aircraft_core.validate import TopologyError  # noqa: E402

from stub_ops import Rect, StubOps  # noqa: E402

WALL_WIDTH_MM = 0.42


class RefuseMessageNamesSparChannelPolicyTests(unittest.TestCase):
    def test_message_names_the_field_and_both_of_its_values(self):
        ops = StubOps()
        rect = Rect(0.0, 0.0, 40.0, 10.0)
        frame = LayerFrame.from_polygon(ops, rect)
        channels = (
            DetectedChannel(center_x_mm=3.0, center_y_mm=5.0, diameter_mm=3.0),
            DetectedChannel(center_x_mm=37.0, center_y_mm=5.0, diameter_mm=3.0),
        )
        spec = StructureSpec(gap_ratio=0.5, skin_margin_mm=1.0)
        ctx = LayerContext(
            layer_index=1076, print_z=215.2, detected_channels=channels
        )

        with self.assertRaises(TopologyError) as raised:
            StructureGenerator().generate(frame, spec, WALL_WIDTH_MM, ctx)

        msg = str(raised.exception)
        self.assertIn("spar_channel_policy", msg)
        self.assertIn("nearest_spar_station", msg)
        self.assertIn("refuse", msg)


if __name__ == "__main__":
    unittest.main()
