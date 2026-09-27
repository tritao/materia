#!/usr/bin/env python3
"""Exercise RobotClient -> robotd -> RKD6 against the Rust minimal device."""

import fcntl
import json
import os
from pathlib import Path
import pty
import signal
import socket
import struct
import subprocess
import tempfile
import time


ROOT = Path(__file__).resolve().parents[2]
HAXEON = ROOT / "haxeon/scripts/haxeon"
ROBOTD = ROOT / "robotkit/robotd/haxeon.json"
CLIENT = ROOT / "robotkit/tests/integration/haxeon.json"
MANIFEST = ROOT / "robotkit/device_virtual/Cargo.toml"
TARGET = ROOT / "robotkit/tests/build/device-pty-cargo"
def effective_fingerprint(base_hex, channels):
    if not channels:
        return base_hex
    mask = (1 << 64) - 1
    value = 14695981039346656037
    def mix(number):
        nonlocal value
        for byte in number.to_bytes(8, "little"):
            value = ((value ^ byte) * 1099511628211) & mask
    for byte in bytes.fromhex(base_hex): mix(byte)
    mix(0)  # no actuator layout
    mix(len(channels))
    for channel in channels:
        for byte in channel["id"].encode().ljust(48, b"\0"): mix(byte)
        safe = channel["safeValue"]
        kind = {"digital": 1, "analog": 2, "process": 3}[safe["kind"]]
        mix(kind); mix(kind); mix(int(safe.get("digital", False)))
        mix(struct.unpack("<Q", struct.pack("<d", safe.get("analog", 0.0)))[0])
        mix(struct.unpack("<Q", struct.pack("<d", safe.get("argument", 0.0)))[0])
        for byte in safe.get("command", "").encode().ljust(48, b"\0"): mix(byte)
    output = bytearray()
    for i in range(16):
        value ^= value >> 32
        value = (value * 1099511628211) & mask
        output.append((value >> ((i % 8) * 8)) & 255)
    return output.hex()
def run(*args):
    result = subprocess.run(args, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f"build failed: {' '.join(args)}\n{result.stdout[-6000:]}")


def unused_port():
    with socket.socket() as server:
        server.bind(("127.0.0.1", 0))
        return server.getsockname()[1]


def wait_for_server(process, port):
    for _ in range(200):
        if process.poll() is not None:
            raise RuntimeError("robotd exited before opening its TCP listener")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                return
        except OSError:
            time.sleep(0.05)
    raise RuntimeError("robotd did not open its TCP listener")


def nonblocking(fd):
    fcntl.fcntl(fd, fcntl.F_SETFL, fcntl.fcntl(fd, fcntl.F_GETFL) | os.O_NONBLOCK)


def main():
    run("cargo", "build", "--manifest-path", str(MANIFEST), "--target-dir", str(TARGET),
        "--bin", "robotd_pty_device")
    run(str(HAXEON), "build", "--project", str(ROBOTD))
    run(str(HAXEON), "build", "--project", str(CLIENT))
    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    control_read, control_write = os.pipe()
    nonblocking(master)
    nonblocking(control_read)
    port = unused_port()
    with tempfile.TemporaryDirectory(prefix="robotkit-device-tcp-") as temp:
        deployment_dir = Path(temp)
        fixture_dir = ROOT / "robotkit/tests/fixtures/device-deployment"
        (deployment_dir / "robot.json").write_bytes((fixture_dir / "robot.json").read_bytes())
        (deployment_dir / "layout.json").write_bytes((fixture_dir / "layout.json").read_bytes())
        (deployment_dir / "device_wire6.lock.json").write_bytes(
            (ROOT / "robotkit/schema/device_wire6.lock.json").read_bytes())
        deployment = json.loads((fixture_dir / "deployment.json").read_text())
        deployment["device"]["path"] = slave_path
        deployment["device"]["schema_lock"] = "device_wire6.lock.json"
        deployment_path = deployment_dir / "deployment.json"
        deployment_path.write_text(json.dumps(deployment))
        wrong = json.loads(json.dumps(deployment))
        wrong["device"]["fingerprint"] = "000102030405060708090a0b0c0d0e0f"
        wrong_path = deployment_dir / "wrong-fingerprint.json"
        wrong_path.write_text(json.dumps(wrong))
        rejected = subprocess.run(
            [str(HAXEON), "run", "--project", str(ROBOTD), "--", "--server",
             f"--deployment={wrong_path}"], cwd=ROOT, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
        if rejected.returncode == 0 or "fingerprint does not match" not in rejected.stdout:
            raise RuntimeError(f"robotd accepted a stale deployment fingerprint:\n{rejected.stdout}")
        device_log_path = Path(temp) / "device.log"
        server_log_path = Path(temp) / "robotd.log"
        client_log_path = Path(temp) / "client.log"
        with device_log_path.open("w+") as device_log, server_log_path.open("w+") as server_log, \
                client_log_path.open("w+") as client_log:
            device = subprocess.Popen(
                [str(TARGET / "debug/robotd_pty_device"), str(master), str(control_read),
                 effective_fingerprint(deployment["device"]["fingerprint"],
                     deployment.get("channels", []))],
                cwd=ROOT, pass_fds=(master, control_read), stdout=device_log,
                stderr=subprocess.STDOUT)
            os.close(control_read)
            os.close(master)
            server = None
            error = None
            try:
                server = subprocess.Popen(
                    [str(HAXEON), "run", "--project", str(ROBOTD), "--", "--server",
                     f"--deployment={deployment_path}", f"--port={port}",
                     "--listen=0.0.0.0"],
                    cwd=ROOT, stdout=server_log, stderr=subprocess.STDOUT,
                    start_new_session=True)
                os.close(slave)
                wait_for_server(server, port)
                subprocess.run(
                    [str(HAXEON), "run", "--project", str(CLIENT), "--",
                     "--device", f"--port={port}"], cwd=ROOT, check=True,
                    stdout=client_log, stderr=subprocess.STDOUT, timeout=60)
            except Exception as exc:
                error = exc
            finally:
                if server is not None and server.poll() is None:
                    os.killpg(server.pid, signal.SIGTERM)
                    try:
                        server.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(server.pid, signal.SIGKILL)
                        server.wait()
                os.close(control_write)
                try:
                    device.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    device.kill()
                    device.wait()
                for handle in (device_log, server_log, client_log):
                    handle.flush()
                if error is not None or device.returncode != 0:
                    for label, path in (("device", device_log_path), ("robotd", server_log_path),
                                        ("client", client_log_path)):
                        print(f"--- {label} ---\n{path.read_text()[-6000:]}")
                    if error is not None:
                        raise error
                    raise RuntimeError(f"Rust device exited with status {device.returncode}")
                print(client_log_path.read_text().splitlines()[-1])


if __name__ == "__main__":
    main()
