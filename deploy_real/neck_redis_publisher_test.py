#!/usr/bin/env python3
"""
Send test neck commands to Redis for the neck_servo_controller.

Use this to validate servo motion without needing the full VR pipeline.
"""
import argparse
import json
import math
import time

import redis
from rich import print


REDIS_KEY_NECK = "action_neck_unitree_g1_with_hands"
REDIS_KEY_T = "t_action"


def deg_to_rad(deg: float) -> float:
    """Convert degrees to radians."""
    return math.radians(deg)


def publish(redis_client: redis.Redis, yaw_deg: float, pitch_deg: float) -> None:
    """Publish a single yaw/pitch command to Redis."""
    yaw_rad = deg_to_rad(yaw_deg)
    pitch_rad = deg_to_rad(pitch_deg)
    payload = [yaw_rad, pitch_rad]
    redis_client.set(REDIS_KEY_NECK, json.dumps(payload))
    redis_client.set(REDIS_KEY_T, int(time.time() * 1000))
    print(
        f"[green]Sent[/] yaw={yaw_deg:+5.1f}° ({yaw_rad:+.3f} rad), "
        f"pitch={pitch_deg:+5.1f}° ({pitch_rad:+.3f} rad)"
    )


def run_demo_sequence(redis_client: redis.Redis, interval: float, repeat: bool) -> None:
    """
    Publish a simple demo sequence of neck poses.

    Angles are moderate and within the controller's safe range.
    """
    steps = [
        (0.0, 0.0, "center"),
        (40.0, 0.0, "look right"),
        (-40.0, 0.0, "look left"),
        (0.0, 0.0, "center"),
        (0.0, 40.0, "look up"),
        (0.0, -40.0, "look down"),
        (0.0, 0.0, "back to center"),
    ]

    try:
        while True:
            for yaw_deg, pitch_deg, label in steps:
                print(f"\n[cyan]{label}[/]")
                publish(redis_client, yaw_deg, pitch_deg)
                time.sleep(interval)
            if not repeat:
                break
    except KeyboardInterrupt:
        print("\n[yellow]Interrupted, stopping demo sequence[/]")


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish neck commands to Redis for testing.")
    parser.add_argument("--redis-ip", type=str, default="localhost", help="Redis server IP")
    parser.add_argument("--yaw-deg", type=float, help="Yaw angle in degrees (right is positive)")
    parser.add_argument("--pitch-deg", type=float, help="Pitch angle in degrees (up is positive)")
    parser.add_argument(
        "--interval",
        type=float,
        default=1.5,
        help="Seconds to wait between demo sequence steps",
    )
    parser.add_argument(
        "--repeat",
        action="store_true",
        default=True,
        help="Repeat the demo sequence until interrupted",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Run the built-in demo sequence (default if no angles provided).",
    )
    args = parser.parse_args()

    redis_client = redis.Redis(host=args.redis_ip, port=6379, db=0, socket_timeout=0.1)
    redis_client.ping()
    print(f"[green]Connected[/] to Redis at {args.redis_ip}:6379")

    # If explicit angles are provided, send a single command; otherwise run the demo.
    if args.yaw_deg is not None or args.pitch_deg is not None:
        yaw = args.yaw_deg if args.yaw_deg is not None else 0.0
        pitch = args.pitch_deg if args.pitch_deg is not None else 0.0
        publish(redis_client, yaw, pitch)
        return

    run_demo_sequence(redis_client, interval=args.interval, repeat=args.repeat or args.demo)


if __name__ == "__main__":
    main()

