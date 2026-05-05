#!/usr/bin/env python3
"""OTA firmware uploader for the BalanceBot ESP-IDF web updater.

This script posts a compiled firmware .bin directly to the robot's
HTTP endpoint:
    POST http://<robot-ip>/update

Examples:
    python3 ota_flash.py --firmware build/self_balancing_robot.bin
    python3 ota_flash.py --firmware build/self_balancing_robot.bin --robot-ip 192.168.4.1
    python3 ota_flash.py --firmware build/self_balancing_robot.bin --robot-ip 192.168.4.1 --port 80
"""

from __future__ import annotations

import argparse
import http.client
import os
from pathlib import Path
import sys
import time
from typing import Optional


DEFAULT_ROBOT_IP = "192.168.4.1"
DEFAULT_PORT = 80
DEFAULT_ENDPOINT = "/update"
CHUNK_SIZE = 32 * 1024


def find_default_firmware() -> Optional[Path]:
    """Try to locate a reasonable default app binary in build/."""
    candidates = []
    build_dir = Path("build")
    if not build_dir.is_dir():
        return None

    preferred_names = [
        "self_balancing_robot.bin",
        "project.bin",
        "app.bin",
    ]
    for name in preferred_names:
        candidate = build_dir / name
        if candidate.is_file():
            return candidate

    for path in build_dir.rglob("*.bin"):
        if path.name.endswith("bootloader.bin"):
            continue
        candidates.append(path)

    if not candidates:
        return None

    candidates.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    return candidates[0]


def format_bytes(num_bytes: int) -> str:
    units = ["B", "KiB", "MiB", "GiB"]
    value = float(num_bytes)
    for unit in units:
        if value < 1024.0 or unit == units[-1]:
            if unit == "B":
                return f"{int(value)} {unit}"
            return f"{value:.2f} {unit}"
        value /= 1024.0
    return f"{num_bytes} B"


def upload_firmware(robot_ip: str, port: int, endpoint: str, firmware_path: Path) -> None:
    if not firmware_path.is_file():
        raise FileNotFoundError(f"Firmware file not found: {firmware_path}")

    file_size = firmware_path.stat().st_size
    if file_size <= 0:
        raise ValueError(f"Firmware file is empty: {firmware_path}")

    print(f"[*] Upload target: http://{robot_ip}:{port}{endpoint}")
    print(f"[*] Firmware: {firmware_path} ({format_bytes(file_size)})")

    connection = http.client.HTTPConnection(robot_ip, port=port, timeout=60)
    file_start = time.time()
    bytes_sent = 0

    try:
        connection.putrequest("POST", endpoint)
        connection.putheader("Content-Type", "application/octet-stream")
        connection.putheader("Content-Length", str(file_size))
        connection.endheaders()

        with firmware_path.open("rb") as firmware_file:
            while True:
                chunk = firmware_file.read(CHUNK_SIZE)
                if not chunk:
                    break
                connection.send(chunk)
                bytes_sent += len(chunk)

                percent = (bytes_sent / file_size) * 100.0
                elapsed = max(time.time() - file_start, 1e-6)
                rate = bytes_sent / elapsed
                sys.stdout.write(
                    f"\r[+] Uploaded {bytes_sent}/{file_size} bytes "
                    f"({percent:5.1f}%) @ {format_bytes(int(rate))}/s"
                )
                sys.stdout.flush()

        print()
        response = connection.getresponse()
        response_body = response.read().decode("utf-8", errors="replace").strip()

        print(f"[*] HTTP {response.status} {response.reason}")
        if response_body:
            print(response_body)

        if 200 <= response.status < 300:
            print("[*] OTA upload complete. The robot should reboot shortly.")
        else:
            raise RuntimeError(f"OTA upload failed with HTTP {response.status}")

    finally:
        connection.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Upload BalanceBot firmware over OTA HTTP.")
    parser.add_argument(
        "--firmware",
        type=Path,
        default=find_default_firmware(),
        help="Path to the compiled firmware .bin (default: auto-detect in build/)",
    )
    parser.add_argument(
        "--robot-ip",
        default=DEFAULT_ROBOT_IP,
        help=f"Robot IP address (default: {DEFAULT_ROBOT_IP})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help=f"Robot HTTP port (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--endpoint",
        default=DEFAULT_ENDPOINT,
        help=f"OTA endpoint path (default: {DEFAULT_ENDPOINT})",
    )

    args = parser.parse_args()

    if args.firmware is None:
        print("[!] Could not auto-detect a firmware .bin in build/. Use --firmware.")
        return 2

    try:
        upload_firmware(args.robot_ip, args.port, args.endpoint, args.firmware)
        return 0
    except KeyboardInterrupt:
        print("\n[!] Cancelled")
        return 130
    except Exception as exc:
        print(f"[!] OTA upload failed: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
