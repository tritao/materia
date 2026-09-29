#!/usr/bin/env python3
"""A non-reading observation subscriber must not impede the controller."""

import socket
import sys
import threading
import time

from stalled_controller import read_frame, send_frame


def main(port):
    with socket.create_connection(("127.0.0.1", port), timeout=5) as observer:
        observer.settimeout(5)
        # The observer requests large camera frames and detections, then stops reading.
        send_frame(observer, threading.Lock(), 1, {
            1: 1, 2: "stalled-observer", 3: "", 4: "observer",
            5: [{1: "camera", 2: 0.0}, {1: "observation", 2: 0.0}],
        })
        with socket.create_connection(("127.0.0.1", port), timeout=5) as controller:
            controller.settimeout(5)
            lock = threading.Lock()
            send_frame(controller, lock, 1, {1: 1, 2: "active-controller", 3: "", 4: "controller"})
            kind, _, _, welcome, _ = read_frame(controller)
            assert kind == 2 and welcome[5] is True
            session, robot, lease = welcome[3], welcome[4], welcome[6]
            initial = None
            while initial is None:
                kind, _, sequence, value, _ = read_frame(controller)
                if kind == 5:
                    assert value[10] != 2
                    initial = sequence
            stop = threading.Event()
            failures = []

            def heartbeat():
                while not stop.is_set():
                    try:
                        send_frame(controller, lock, 16, {1: robot, 2: lease}, session)
                    except OSError as error:
                        failures.append(error)
                        return
                    stop.wait(0.25)

            thread = threading.Thread(target=heartbeat, daemon=True)
            thread.start()
            try:
                time.sleep(2.0)
                send_frame(controller, lock, 7, {}, session)
                state = fault = detection = False
                deadline = time.monotonic() + 6
                while time.monotonic() < deadline and not (state and fault and detection):
                    kind, frame_session, sequence, value, _ = read_frame(controller)
                    assert frame_session == session
                    if kind == 5:
                        assert value[10] != 2, "stalled observer caused emergency stop"
                        state |= sequence > initial
                    elif kind == 11 and value[2] == 404:
                        fault = True
                    elif kind == 19:
                        detection = True
                assert not failures and state and fault and detection, (
                    f"controller delivery state={state} fault={fault} detection={detection} heartbeat={failures}"
                )
                print("robotd stalled observation subscriber did not affect controller")
            finally:
                stop.set()
                thread.join(timeout=1)


if __name__ == "__main__":
    main(int(sys.argv[1]))
