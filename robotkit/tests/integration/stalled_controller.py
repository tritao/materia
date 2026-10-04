#!/usr/bin/env python3
"""A raw TCP controller that keeps its lease while pausing socket reads."""

import socket
import re
from pathlib import Path
import struct
import sys
import threading
import time


PROTOCOL_VERSION = int(re.search(r"VERSION:Int = (\d+)",
    (Path(__file__).resolve().parents[2] / "haxe/robotkit/protocol/RobotFrame.hx").read_text()).group(1))


def pack(value):
    if isinstance(value, bool):
        return b"\xc3" if value else b"\xc2"
    if isinstance(value, int):
        if 0 <= value < 128:
            return bytes((value,))
        return b"\xcf" + struct.pack(">Q", value)
    if isinstance(value, float):
        return b"\xcb" + struct.pack(">d", value)
    if isinstance(value, str):
        data = value.encode()
        return (bytes((0xA0 | len(data),)) if len(data) < 32 else b"\xd9" + bytes((len(data),))) + data
    if isinstance(value, list):
        assert len(value) < 16
        return bytes((0x90 | len(value),)) + b"".join(pack(item) for item in value)
    if isinstance(value, dict):
        assert len(value) < 16
        return bytes((0x80 | len(value),)) + b"".join(pack(k) + pack(v) for k, v in value.items())
    raise TypeError(type(value))


def unpack(data):
    def item(offset):
        tag = data[offset]
        offset += 1
        if tag < 0x80:
            return tag, offset
        if tag >= 0xE0:
            return tag - 256, offset
        if 0x80 <= tag <= 0x8F or tag in (0xDE, 0xDF):
            count = tag & 15 if tag <= 0x8F else int.from_bytes(data[offset:offset + (2 if tag == 0xDE else 4)], "big")
            if tag > 0x8F:
                offset += 2 if tag == 0xDE else 4
            result = {}
            for _ in range(count):
                key, offset = item(offset)
                value, offset = item(offset)
                result[key] = value
            return result, offset
        if 0x90 <= tag <= 0x9F or tag in (0xDC, 0xDD):
            count = tag & 15 if tag <= 0x9F else int.from_bytes(data[offset:offset + (2 if tag == 0xDC else 4)], "big")
            if tag > 0x9F:
                offset += 2 if tag == 0xDC else 4
            result = []
            for _ in range(count):
                value, offset = item(offset)
                result.append(value)
            return result, offset
        if 0xA0 <= tag <= 0xBF or tag in (0xD9, 0xDA, 0xDB, 0xC4, 0xC5, 0xC6):
            if tag <= 0xBF:
                length = tag & 31
            else:
                size = {0xD9: 1, 0xDA: 2, 0xDB: 4, 0xC4: 1, 0xC5: 2, 0xC6: 4}[tag]
                length = int.from_bytes(data[offset:offset + size], "big")
                offset += size
            value = data[offset:offset + length]
            return (value if tag in (0xC4, 0xC5, 0xC6) else value.decode()), offset + length
        if tag in (0xC0, 0xC2, 0xC3):
            return {0xC0: None, 0xC2: False, 0xC3: True}[tag], offset
        if tag in (0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3):
            size = {0xCC: 1, 0xCD: 2, 0xCE: 4, 0xCF: 8, 0xD0: 1, 0xD1: 2, 0xD2: 4, 0xD3: 8}[tag]
            signed = tag >= 0xD0
            return int.from_bytes(data[offset:offset + size], "big", signed=signed), offset + size
        if tag in (0xCA, 0xCB):
            size = 4 if tag == 0xCA else 8
            return struct.unpack(">f" if size == 4 else ">d", data[offset:offset + size])[0], offset + size
        raise AssertionError(f"unsupported MessagePack tag 0x{tag:02x}")

    value, end = item(0)
    assert end == len(data), "trailing MessagePack bytes"
    return value


def read_exact(sock, size):
    chunks = []
    while size:
        chunk = sock.recv(min(size, 65536))
        if not chunk:
            raise AssertionError("controller socket closed")
        chunks.append(chunk)
        size -= len(chunk)
    return b"".join(chunks)


def read_frame(sock):
    magic, version, kind, flags, length, attachments, session, sequence, timestamp = struct.unpack(
        ">4sHHIIIQQQ", read_exact(sock, 44)
    )
    assert magic == b"RKF1" and version == PROTOCOL_VERSION and flags == 0
    payload = read_exact(sock, length)
    sizes = []
    for _ in range(attachments):
        size = struct.unpack(">I", read_exact(sock, 4))[0]
        read_exact(sock, size)
        sizes.append(size)
    return kind, session, sequence, unpack(payload), sizes


def send_frame(sock, lock, kind, payload, session=0):
    body = pack(payload)
    frame = struct.pack(">4sHHIIIQQQ", b"RKF1", PROTOCOL_VERSION, kind, 0, len(body), 0, session, 0, 0) + body
    with lock:
        sock.sendall(frame)


def main(port):
    with socket.create_connection(("127.0.0.1", port), timeout=5) as sock:
        sock.settimeout(5)
        lock = threading.Lock()
        send_frame(sock, lock, 1, {1: PROTOCOL_VERSION, 2: "stalled-bulk", 3: "", 4: "controller", 5: []})
        kind, _, _, welcome, _ = read_frame(sock)
        assert kind == 2 and welcome[5] is True, "controller lease was not granted"
        session, robot, lease = welcome[3], welcome[4], welcome[6]
        initial_state_sequence = None
        while initial_state_sequence is None:
            kind, _, sequence, value, _ = read_frame(sock)
            if kind == 5:
                assert value[10] != 2, "controller started in emergency stop"
                initial_state_sequence = sequence
        stop = threading.Event()
        heartbeat_error = []

        def heartbeats():
            while not stop.is_set():
                try:
                    send_frame(sock, lock, 16, {1: robot, 2: lease}, session)
                except OSError as error:
                    heartbeat_error.append(error)
                    return
                stop.wait(0.25)

        thread = threading.Thread(target=heartbeats, daemon=True)
        thread.start()
        try:
            # No recv calls while two 640x480 RGB camera sources publish at 50 Hz.
            time.sleep(2.0)
            send_frame(sock, lock, 7, {}, session)  # unsupported command -> Fault 404
            seen_state = seen_fault = False
            cameras = set()
            camera_ids = set()
            kinds = {}
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline and not (seen_state and seen_fault and len(camera_ids) >= 2):
                kind, frame_session, sequence, value, attachments = read_frame(sock)
                kinds[kind] = kinds.get(kind, 0) + 1
                assert frame_session == session
                if kind == 5:
                    assert value[10] != 2, "bulk traffic caused emergency stop"
                    seen_state |= sequence > initial_state_sequence
                elif kind == 11 and value[2] == 404:
                    seen_fault = True
                elif kind == 17:
                    key = (value[2], value[5])
                    assert key not in cameras, f"camera sequence repeated: {key}"
                    cameras.add(key)
                    if attachments == [640 * 480 * 3]:
                        camera_ids.add(value[2])
            assert not heartbeat_error, f"heartbeat failed: {heartbeat_error}"
            assert seen_state and seen_fault and len(camera_ids) >= 2, (
                f"recovery missing state={seen_state} fault={seen_fault} cameras={camera_ids} kinds={kinds}"
            )
            print(f"robotd stalled controller retained control; {len(cameras)} unique large camera frames received")
        finally:
            stop.set()
            thread.join(timeout=1)


if __name__ == "__main__":
    main(int(sys.argv[1]))
