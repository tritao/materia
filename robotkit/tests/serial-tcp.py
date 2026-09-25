#!/usr/bin/env python3
"""Exercise RemoteRobot's TCP path through robotd and a serial PTY emulator."""

import os
import pty
import select
import socket
import struct
import subprocess
import tempfile
import threading
import time
import tty
import zlib


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
HAXEON = os.path.join(ROOT, "haxeon/scripts/haxeon")


def checksum(data):
    return zlib.crc32(data) & 0xFFFFFFFF


def frame(magic, payload):
    body = magic + struct.pack("<I", len(payload)) + payload
    return body + struct.pack("<I", checksum(body))


def write_all(descriptor, data):
    offset = 0
    while offset < len(data):
        offset += os.write(descriptor, data[offset:])


def read_commands(pending, position, velocity, effort, target_seen):
    while True:
        start = pending.find(b"RKC3")
        if start < 0:
            if len(pending) > 3:
                del pending[:-3]
            return
        if start:
            del pending[:start]
        if len(pending) < 8:
            return
        payload_size = struct.unpack_from("<I", pending, 4)[0]
        packet_size = 8 + payload_size + 4
        if len(pending) < packet_size:
            return
        packet = bytes(pending[:packet_size])
        del pending[:packet_size]
        if struct.unpack_from("<I", packet, packet_size - 4)[0] != checksum(packet[:-4]):
            continue

        payload = packet[8:-4]
        if len(payload) < 20:
            continue
        kind, _, target_count, _ = struct.unpack_from("<IQII", payload)
        if kind in (2, 3, 4):
            for joint in range(len(velocity)):
                velocity[joint] = 0.0
                effort[joint] = 0.0
            continue
        if kind != 1 or len(payload) != 20 + target_count * 16:
            continue
        for index in range(target_count):
            joint, mode, value = struct.unpack_from("<IId", payload, 20 + index * 16)
            if joint >= len(position):
                continue
            if mode == 1:
                position[joint] = value
                velocity[joint] = 0.0
                effort[joint] = 0.0
                if joint == 0 and abs(value - 0.5) < 1e-9:
                    target_seen.set()
            elif mode == 2:
                velocity[joint] = value
                effort[joint] = 0.0
            elif mode == 3:
                effort[joint] = value


def run_device(master, stopping, position, velocity, effort, target_seen):
    pending = bytearray()
    sequence = 0
    next_state = time.monotonic()
    last_update = next_state
    while not stopping.is_set():
        readable, _, _ = select.select([master], [], [], 0.002)
        if readable:
            try:
                data = os.read(master, 4096)
            except OSError:
                return
            pending.extend(data)
            read_commands(pending, position, velocity, effort, target_seen)

        now = time.monotonic()
        elapsed = now - last_update
        for joint in range(len(position)):
            position[joint] += velocity[joint] * elapsed
        last_update = now
        if now < next_state:
            continue
        sequence += 1
        timestamp = time.monotonic_ns()
        payload = struct.pack("<IQI", len(position), timestamp, 3)
        for joint in range(len(position)):
            payload += struct.pack("<ddd", position[joint], velocity[joint], effort[joint])
        sensors = [
            position.copy(),
            [0.0, 0.0, 0.0, 0.0, 0.0, 9.81],
            [10.0] * 8,
        ]
        for values in sensors:
            payload += struct.pack("<QQI", sequence, timestamp, len(values))
            payload += struct.pack("<" + "d" * len(values), *values)
        try:
            write_all(master, frame(b"RKS3", payload))
        except OSError:
            return
        next_state = now + 0.02


def read_server_log(output):
    output.flush()
    output.seek(0)
    return output.read()


def tcp_ready(port):
    connection = socket.socket()
    connection.settimeout(0.1)
    try:
        connection.connect(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        connection.close()


def run():
    master, slave = pty.openpty()
    tty.setraw(slave)
    device_path = os.ttyname(slave)

    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()

    stopping = threading.Event()
    target_seen = threading.Event()
    position = [0.0, 0.0, 0.0]
    velocity = [0.0, 0.0, 0.0]
    effort = [0.0, 0.0, 0.0]
    device_thread = threading.Thread(
        target=run_device,
        args=(master, stopping, position, velocity, effort, target_seen),
        daemon=True,
    )
    device_thread.start()

    server_args = [
        HAXEON, "run", "--project", "robotkit/robotd/haxeon.json", "--",
        "--server", "--once", "--serial=" + device_path, "--baud=115200",
        "--multi-joint", "--robot-id=42", "--port=" + str(port),
    ]
    client_args = [
        HAXEON, "run", "--project", "robotkit/tests/integration/haxeon.json", "--",
        "--port=" + str(port),
    ]

    try:
        with tempfile.TemporaryFile(mode="w+t") as server_output:
            server = subprocess.Popen(
                server_args, cwd=ROOT, stdout=server_output, stderr=subprocess.STDOUT
            )
            try:
                deadline = time.monotonic() + 60
                while time.monotonic() < deadline:
                    log = read_server_log(server_output)
                    if tcp_ready(port):
                        break
                    if server.poll() is not None:
                        raise RuntimeError("robotd exited before listening:\n" + log)
                    time.sleep(0.05)
                else:
                    raise RuntimeError("robotd did not start:\n" + read_server_log(server_output))
                # Let robotd clear the probe connection before the real client arrives.
                time.sleep(0.1)

                client = subprocess.run(
                    client_args, cwd=ROOT, capture_output=True, text=True, timeout=60
                )
                if client.returncode != 0:
                    raise RuntimeError("TCP client integration failed:\n" + client.stdout + client.stderr)
                if not target_seen.wait(1.0):
                    raise RuntimeError("robotd did not send the expected serial joint target")
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired as error:
                    raise RuntimeError("robotd did not stop after the one-shot client") from error
                if server.returncode != 0:
                    raise RuntimeError("robotd failed:\n" + read_server_log(server_output))
                print("RemoteRobot behavior and GoTo parity passed through robotd and a serial PTY emulator")
                print(client.stdout.strip())
            finally:
                if server.poll() is None:
                    server.terminate()
                    server.wait(timeout=5)
    finally:
        stopping.set()
        device_thread.join(timeout=1)
        os.close(slave)
        os.close(master)


if __name__ == "__main__":
    run()
