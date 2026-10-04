#!/usr/bin/env python3
"""Authentication and independent observe/command/deployment grants over real TCP."""
import socket
import threading
import sys
import time
from stalled_controller import PROTOCOL_VERSION, read_frame, send_frame


def hello(sock, identity, token, role):
    send_frame(sock, threading.Lock(), 1, {
        1: PROTOCOL_VERSION, 2: "authorization-test", 3: "", 4: role, 5: [],
        6: identity, 7: token,
    })
    for _ in range(20):
        kind, session, _, value, _ = read_frame(sock)
        if kind in (2, 11):
            return kind, session, value
    raise AssertionError("No handshake result")


def main(port):
    for identity, token, role in [
        ("", "", "observer"),
        ("unknown", "robotkit-test-observer-token-0001", "observer"),
        ("test-observer", "wrong-token-000000000000", "observer"),
        ("test-observer", "robotkit-test-observer-token-0001", "controller"),
        ("test-controller", "robotkit-test-controller-token-0001", "deployment"),
    ]:
        with socket.create_connection(("127.0.0.1", port), timeout=5) as sock:
            sock.settimeout(5)
            kind, _, value = hello(sock, identity, token, role)
            assert kind == 11 and value[2] in (401, 403), (identity, role, kind, value)
    for identity, role, permissions in [
        ("test-observer", "observer", ["observe"]),
        ("test-deployer", "deployment", ["observe", "deployment"]),
    ]:
        with socket.create_connection(("127.0.0.1", port), timeout=5) as sock:
            sock.settimeout(5)
            token = "robotkit-test-" + ("observer" if role == "observer" else "deployer") + "-token-0001"
            kind, session, value = hello(sock, identity, token, role)
            assert kind == 2 and value[9] == identity and value[10] == permissions and not value[5], value
            send_frame(sock, threading.Lock(), 7, {}, session, sequence=1)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                kind, _, _, reply, _ = read_frame(sock)
                if kind == 11:
                    assert reply[2] == 403, reply
                    break
            else:
                raise AssertionError("Observe/deploy identity accepted a command")
            # An observer cannot change deployment, and even a deployer cannot supply a path.
            send_frame(sock, threading.Lock(), 20, {1: "../../unauthorized.json"}, session, sequence=2)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                kind, _, _, value, _ = read_frame(sock)
                if kind == 11:
                    assert value[2] == 403, value
                    break
            else:
                raise AssertionError("Unauthorized deployment accepted")
    print("robotd TCP authentication and deployment authorization passed")


if __name__ == "__main__":
    main(int(sys.argv[1]))
