#!/usr/bin/env python3
"""
Unitree G1 SLAM relocation (unitree_sdk2py only).

Logic mirrors guide2/g1_slam_client.py relocation(); no import from G1SlamClient.
Edit the configuration block below, then run or import.

Usage (CLI):
    python3 g1_slam_relocation.py

Usage (library):
    from g1_slam_relocation import start_relocation
    status, data = start_relocation()
"""

from __future__ import annotations

import json
from typing import Optional, Tuple

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client

# ---------------------------------------------------------------------------
# Configuration (edit here)
# ---------------------------------------------------------------------------
MAP_PATH = "/home/unitree/test.pcd"
NETWORK_INTERFACE = "eth0"
DOMAIN_ID = 0
TIMEOUT_SEC = 10.0

# Initial pose (same defaults as G1SlamClient.relocation)
INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = 0.0
INIT_Q_X = 0.0
INIT_Q_Y = 0.0
INIT_Q_Z = 0.0
INIT_Q_W = 1.0

# From g1_slam_client.py / keyDemo.cpp
SLAM_SERVICE_NAME = "slam_operate"
SLAM_API_VERSION = "1.0.0.1"
ROBOT_API_ID_START_RELOCATION_PL = 1804

__all__ = [
    "start_relocation",
    "SlamRelocationClient",
    "MAP_PATH",
    "NETWORK_INTERFACE",
    "SLAM_SERVICE_NAME",
    "SLAM_API_VERSION",
    "ROBOT_API_ID_START_RELOCATION_PL",
]


class SlamRelocationClient(Client):
    """Minimal SLAM RPC client for relocation only (unitree_sdk2py.rpc.client.Client)."""

    def __init__(self) -> None:
        super().__init__(SLAM_SERVICE_NAME, False)

    def init(self) -> None:
        self._SetApiVerson(SLAM_API_VERSION)
        self._RegistApi(ROBOT_API_ID_START_RELOCATION_PL, 0)


def _build_relocation_parameter() -> str:
    """JSON payload for ROBOT_API_ID_START_RELOCATION_PL."""
    return json.dumps({
        "data": {
            "x": INIT_X,
            "y": INIT_Y,
            "z": INIT_Z,
            "q_x": INIT_Q_X,
            "q_y": INIT_Q_Y,
            "q_z": INIT_Q_Z,
            "q_w": INIT_Q_W,
            "address": MAP_PATH,
        }
    })


def start_relocation(init_channel: bool = True) -> Tuple[int, Optional[str]]:
    """
    Start SLAM relocation using the hard-coded configuration at the top of this file.

    Args:
        init_channel: If True, call ChannelFactoryInitialize before RPC.
            Set False when another module already initialized the channel.

    Returns:
        (status_code, response_data) — status_code 0 means success.
    """
    if init_channel:
        ChannelFactoryInitialize(DOMAIN_ID, NETWORK_INTERFACE)

    client = SlamRelocationClient()
    client.init()
    client.SetTimeout(TIMEOUT_SEC)

    status_code, data = client._Call(
        ROBOT_API_ID_START_RELOCATION_PL, _build_relocation_parameter()
    )
    return status_code, data


def main() -> int:
    print("=" * 70)
    print("G1 SLAM Relocation")
    print("=" * 70)
    print(f"  map_path:  {MAP_PATH}")
    print(f"  pose:      x={INIT_X}, y={INIT_Y}, z={INIT_Z}, "
          f"q=({INIT_Q_X}, {INIT_Q_Y}, {INIT_Q_Z}, {INIT_Q_W})")
    print(f"  network:   {NETWORK_INTERFACE}")
    print(f"  domain_id: {DOMAIN_ID}")
    print("=" * 70)

    status, data = start_relocation()

    if status == 0:
        print(f"OK: Relocation started with map {MAP_PATH}")
        if data:
            print(f"Response: {data}")
        return 0

    print(f"ERROR: Relocation failed, statusCode={status}, data={data}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
