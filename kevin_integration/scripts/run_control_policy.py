"""Starter script for running the Kevin integration learned control policy.

This is a minimal smoke-test harness. It does not connect to Isaac Sim.
You must provide your own integration code that assembles the feature vectors in the same order as training.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from kevin_integration.control_policy import ControlPolicy


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="cpu", help="torch device string, e.g. cpu, cuda, cuda:0")
    parser.add_argument("--debug", action="store_true", help="Print the full pipeline output dictionary")
    args = parser.parse_args()

    policy = ControlPolicy(
        config={
            "pipeline": {"device": args.device},
            "return_debug": args.debug,
        }
    )

    # Dummy inputs (replace with real assembled feature vectors).
    obs = {
        "aam_x_t": np.zeros((18,), dtype=np.float32),
        "id_x_t": np.zeros((19,), dtype=np.float32),
        # Optional:
        "sd_x_t": np.zeros((22,), dtype=np.float32),
        "fd_x_t": np.zeros((18,), dtype=np.float32),
    }

    action_or_debug = policy.compute_action(obs)
    print(action_or_debug)


if __name__ == "__main__":
    main()
