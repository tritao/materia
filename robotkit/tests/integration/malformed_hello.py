#!/usr/bin/env python3
"""An invalid subscription field produces Fault 400 without killing robotd."""

import socket
import sys
import threading

from stalled_controller import PROTOCOL_VERSION, read_frame, send_frame


def main(port):
    with socket.create_connection(("127.0.0.1", port), timeout=5) as bad:
        bad.settimeout(5)
        send_frame(bad, threading.Lock(), 1, {
            1: PROTOCOL_VERSION, 2: "bad-hello", 3: "", 4: "observer",
            5: [{1: "camera", 2: 15}],  # maxRateHz must be a Float, not an Int.
        })
        for _ in range(10):
            kind, _, _, value, _ = read_frame(bad)
            if kind == 11:
                assert value[2] == 400, f"malformed Hello fault was {value[2]}"
                break
        else:
            raise AssertionError("malformed Hello did not receive Fault 400")

    with socket.create_connection(("127.0.0.1", port), timeout=5) as good:
        good.settimeout(5)
        send_frame(good, threading.Lock(), 1, {1: PROTOCOL_VERSION, 2: "good-hello", 3: "", 4: "observer", 5: []})
        for _ in range(10):
            kind, _, _, _, _ = read_frame(good)
            if kind == 2:
                print("robotd survived malformed Hello and accepted a current-version Hello")
                return
        raise AssertionError("robotd did not accept a current-version Hello after malformed input")


if __name__ == "__main__":
    main(int(sys.argv[1]))
