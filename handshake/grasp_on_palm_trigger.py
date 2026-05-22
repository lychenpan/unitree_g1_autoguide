#!/usr/bin/env python3
"""
Palm-triggered grasp: when palm mean pressure > threshold, close four fingers
until each finger's tip/top/palm pressure reaches stop threshold. Thumb stays still.

Self-contained - Modbus TCP, no separate Headless_driver needed.
"""

import time

import os
import sys

import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize

try:
    from .inspire_sdkpy import inspire_sdk, inspire_hand_defaut, inspire_dds
except ImportError:
    # Run as script: python handshake/grasp_on_palm_trigger.py
    _handshake_dir = os.path.dirname(os.path.abspath(__file__))
    if _handshake_dir not in sys.path:
        sys.path.insert(0, _handshake_dir)
    from inspire_sdkpy import inspire_sdk, inspire_hand_defaut, inspire_dds

# --- Global parameters (configurable) ---
PALM_TRIGGER_THRESHOLD = 50   # Trigger when palm mean pressure > this
START_ANGLE = 900             # Initial command when function starts
TRIGGER_ANGLE = 300           # Command right after palm trigger
FINAL_ANGLE = 800             # Command after hold time
TRIGGER_HOLD_SECONDS = 3.0    # Hold time at TRIGGER_ANGLE
LOOP_DT = 0.03                # Seconds per control loop
PALM_WAIT_TIMEOUT = 5.0      # Stop and return if no palm trigger within this many seconds

# Finger index -> touch region names (tip, top, palm)
FINGER_TOUCH_REGIONS = [
    ("fingerone_tip_touch", "fingerone_top_touch", "fingerone_palm_touch"),   # 0: Pinky
    ("fingertwo_tip_touch", "fingertwo_top_touch", "fingertwo_palm_touch"),   # 1: Ring
    ("fingerthree_tip_touch", "fingerthree_top_touch", "fingerthree_palm_touch"),  # 2: Middle
    ("fingerfour_tip_touch", "fingerfour_top_touch", "fingerfour_palm_touch"),   # 3: Index
]
FINGER_NAMES = ["Pinky", "Ring", "Middle", "Index"]

# Cache for init_hand_connection (ip, network, handler, publisher, cmd)

# Cache for init_hand_connection (ip, network, handler, publisher, cmd)
_hand_connection_cache = None


def reset_hand_connection():
    """Clear the cached connection. Next init_hand_connection will perform full init."""
    global _hand_connection_cache
    _hand_connection_cache = None

# All touch regions for debug output
DEBUG_REGION_NAMES = {
    "fingerone_tip_touch": "Pinky tip",
    "fingerone_top_touch": "Pinky top",
    "fingerone_palm_touch": "Pinky palm",
    "fingertwo_tip_touch": "Ring tip",
    "fingertwo_top_touch": "Ring top",
    "fingertwo_palm_touch": "Ring palm",
    "fingerthree_tip_touch": "Middle tip",
    "fingerthree_top_touch": "Middle top",
    "fingerthree_palm_touch": "Middle palm",
    "fingerfour_tip_touch": "Index tip",
    "fingerfour_top_touch": "Index top",
    "fingerfour_palm_touch": "Index palm",
    "fingerfive_tip_touch": "Thumb tip",
    "fingerfive_top_touch": "Thumb top",
    "fingerfive_middle_touch": "Thumb middle",
    "fingerfive_palm_touch": "Thumb palm",
    "palm_touch": "Palm",
}


def _get_finger_max_pressure(touch: dict, finger_idx: int) -> float:
    """Return max pressure across tip, top, palm for the given finger (0-3)."""
    regions = FINGER_TOUCH_REGIONS[finger_idx]
    max_val = 0.0
    for var in regions:
        arr = touch.get(var)
        if arr is not None and np.size(arr) > 0:
            max_val = max(max_val, float(np.max(arr)))
    return max_val


def _print_debug_pressure(touch: dict, timestamp: str = ""):
    """Print each part's pressure (mean, max) to terminal."""
    print(f"\n--- Pressure debug [{timestamp}] ---")
    for var, name in DEBUG_REGION_NAMES.items():
        arr = touch.get(var)
        if arr is not None and np.size(arr) > 0:
            mean_val = float(np.mean(arr))
            max_val = int(np.max(arr))
            min_val = int(np.min(arr))
            print(f"  {name:18s}: mean={mean_val:7.1f}  max={max_val:5d}  min={min_val:5d}")
        else:
            print(f"  {name:18s}: (no data)")


def init_hand_connection(
    ip: str = "192.168.123.210",
    network: str = "eth0",
    device_id: int = 1,
):
    """
    Initialize hand connection: DDS, ModbusDataHandler, and control publisher.
    Returns (handler, publisher, cmd, was_cached). Only performs actual init on
    first call for given (ip, network); subsequent calls reuse cached connection.
    """
    global _hand_connection_cache

    if _hand_connection_cache is not None:
        cached_ip, cached_net, handler, publisher, cmd = _hand_connection_cache
        if cached_ip == ip and cached_net == network:
            return handler, publisher, cmd, True

    ChannelFactoryInitialize(0, network)

    handler = inspire_sdk.ModbusDataHandler(
        ip=ip,
        LR="l",
        device_id=device_id,
        use_serial=False,
        initDDS=False,
    )
    time.sleep(0.3)

    publisher = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
    publisher.Init()

    cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
    cmd.mode = 0b0001  # Angle mode

    _hand_connection_cache = (ip, network, handler, publisher, cmd)
    return handler, publisher, cmd, False


def grasp_on_palm_trigger(
    ip: str = "192.168.123.210",
    network: str = "eth0",
    palm_trigger: float = None,
    start_angle: int = None,
    trigger_angle: int = None,
    final_angle: int = None,
    trigger_hold_seconds: float = None,
    palm_wait_timeout: float = None,
    verbose: bool = True,
    debug: bool = False,
):
    """
    When called:
      - publish angle_set=[start_angle]*6 immediately
      - wait for palm trigger
      - on trigger publish angle_set=[trigger_angle]*6
      - hold trigger_hold_seconds
      - publish angle_set=[final_angle]*6
    Returns early if no palm trigger within palm_wait_timeout seconds.

    Args:
        ip: Hand IP (Modbus TCP)
        network: DDS network interface (e.g. 'eth0')
        palm_trigger: Override PALM_TRIGGER_THRESHOLD
        start_angle: Override START_ANGLE
        trigger_angle: Override TRIGGER_ANGLE
        final_angle: Override FINAL_ANGLE
        trigger_hold_seconds: Override TRIGGER_HOLD_SECONDS
        palm_wait_timeout: Max seconds to wait for palm trigger (default 2.0); None = no timeout
        verbose: Print status messages
        debug: Print each part's pressure (mean, max, min) to terminal each loop
    """
    palm_tr = palm_trigger if palm_trigger is not None else PALM_TRIGGER_THRESHOLD
    start_ang = int(start_angle if start_angle is not None else START_ANGLE)
    trigger_ang = int(trigger_angle if trigger_angle is not None else TRIGGER_ANGLE)
    final_ang = int(final_angle if final_angle is not None else FINAL_ANGLE)
    hold_s = float(trigger_hold_seconds if trigger_hold_seconds is not None else TRIGGER_HOLD_SECONDS)
    timeout = palm_wait_timeout if palm_wait_timeout is not None else PALM_WAIT_TIMEOUT

    handler, publisher, cmd, was_cached = init_hand_connection(ip=ip, network=network)
    cmd.mode = 0b0001

    if verbose:
        if was_cached:
            print(f"Using cached hand connection ({ip}).")
        else:
            print(f"Connected to left hand at {ip}.")
        print(
            f"  palm_trigger={palm_tr}, start_angle={start_ang}, "
            f"trigger_angle={trigger_ang}, final_angle={final_ang}, hold={hold_s}s"
        )
        if debug:
            print("  debug=ON (printing pressure each loop)")
        if timeout > 0:
            print(f"  palm_wait_timeout={timeout}s")

    # Step 1: publish initial angle immediately when called
    cmd.angle_set = [start_ang] * 6
    publisher.Write(cmd)
    if verbose:
        print(f"Initial command sent: angle_set={cmd.angle_set}")

    triggered = False

    if verbose:
        timeout_msg = f" (timeout {timeout}s)" if timeout > 0 else ""
        print(f"Monitoring palm... (place object on palm to trigger){timeout_msg}")

    palm_wait_start = time.perf_counter()
    try:
        while True:
            result = handler.read()
            touch = result.get("touch", {})

            if debug and touch:
                _print_debug_pressure(touch, time.strftime("%H:%M:%S"))

            if not triggered:
                if timeout > 0:
                    elapsed = time.perf_counter() - palm_wait_start
                    if elapsed > timeout:
                        if verbose:
                            print(f"\nNo palm trigger within {timeout}s. Stopping.")
                        return

                palm_arr = touch.get("palm_touch")
                if palm_arr is not None and np.size(palm_arr) > 0:
                    palm_mean = float(np.mean(palm_arr))
                    if palm_mean > palm_tr:
                        triggered = True
                        if verbose:
                            print(f"\nPalm triggered (mean={palm_mean:.1f}).")
                else:
                    palm_mean = 0.0
                if verbose and not triggered:
                    print(f"\rPalm mean: {palm_mean:.1f} / {palm_tr}  ", end="", flush=True)
            else:
                # Step 3: publish trigger angle, hold, then final angle
                cmd.angle_set = [trigger_ang] * 6
                publisher.Write(cmd)
                if verbose:
                    print(f"\nTrigger command sent: angle_set={cmd.angle_set}")
                    print(f"Holding for {hold_s:.1f}s...")
                time.sleep(max(0.0, hold_s))
                cmd.angle_set = [final_ang] * 6
                publisher.Write(cmd)
                if verbose:
                    print(f"Final command sent: angle_set={cmd.angle_set}")
                    print("Sequence complete.")
                break

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        if verbose:
            print("\nInterrupted.")


if __name__ == "__main__":
    ip = "192.168.123.210"
    grasp_on_palm_trigger(ip=ip, debug=True)
