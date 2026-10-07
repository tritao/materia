#!/usr/bin/env python3
"""Drives the browser editor in Chrome over the DevTools protocol.

Waits until the page reports that the editor has drawn --frames frames (window.materia),
prints the page's console output, and optionally saves a screenshot. Exits non-zero if the
editor fails or does not start in time.
"""

import argparse
import base64
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../../haxeon/vendor/nativekit/tools"))
from canvas_sizing_smoke import check_canvas_sizing
from web_smoke import WebSocket, wait_for_page  # noqa: E402


class Page:
    def __init__(self, socket):
        self.socket = socket
        self.next_id = 0
        self.console = []
        # Other protocol events, oldest first, for drivers that wait on one (file choosers, downloads).
        self.events = []

    def command(self, method, params=None):
        self.next_id += 1
        self.socket.send({"id": self.next_id, "method": method, "params": params or {}})
        while True:
            kind, payload = self.socket.receive()
            if kind != 1:
                continue
            message = json.loads(payload)
            event, params_ = message.get("method"), message.get("params", {})
            if event == "Runtime.consoleAPICalled":
                text = " ".join(str(argument.get("value", argument.get("description", "")))
                                for argument in params_.get("args", []))
                self.console.append(f"[{params_.get('type')}] {text}")
            elif event is not None and event != "Runtime.exceptionThrown":
                self.events.append((event, params_))
            elif event == "Runtime.exceptionThrown":
                details = params_.get("exceptionDetails", {})
                self.console.append("[exception] " + (details.get("exception", {}).get("description")
                                                      or details.get("text", "")))
            if message.get("id") == self.next_id:
                if "error" in message:
                    raise RuntimeError(json.dumps(message["error"]))
                return message.get("result", {})

    def evaluate(self, expression):
        result = self.command("Runtime.evaluate", {"expression": expression, "returnByValue": True})
        return result.get("result", {}).get("value")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug-port", type=int, required=True)
    parser.add_argument("--page-url", required=True)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--screenshot")
    parser.add_argument("--click", action="append", default=[], metavar="X,Y",
                        help="after startup, click at page coordinates X,Y (repeatable, in order)")
    options = parser.parse_args()

    target = wait_for_page(options.debug_port, options.page_url, 30)
    page = Page(WebSocket(target["webSocketDebuggerUrl"]))
    # Guest compilation can keep the renderer busy during initial startup.
    page.socket.socket.settimeout(min(30.0, options.timeout))
    page.command("Runtime.enable")
    page.command("Page.enable")
    deadline = time.monotonic() + options.timeout
    state = {}
    while time.monotonic() < deadline:
        state = page.evaluate("JSON.stringify(window.materia || null)")
        state = (json.loads(state) if state else None) or {}
        if state.get("state") in ("failed", "stopped") or state.get("frames", 0) >= options.frames:
            break
        time.sleep(0.25)
    if state.get("state") == "running":
        check_canvas_sizing(page)
    for click in options.click if state.get("state") == "running" else []:
        x, y = (float(value) for value in click.split(","))
        for kind in ("mouseMoved", "mousePressed", "mouseReleased"):
            page.command("Input.dispatchMouseEvent", {"type": kind, "x": x, "y": y, "button": "left",
                                                      "clickCount": 0 if kind == "mouseMoved" else 1})
        frames = page.evaluate("window.materia.frames")
        while time.monotonic() < deadline and page.evaluate("window.materia.frames") < frames + 20:
            time.sleep(0.1)
    state = json.loads(page.evaluate("JSON.stringify(window.materia || null)") or "null") or {}
    if options.screenshot:
        shot = page.command("Page.captureScreenshot", {"format": "png"})
        with open(options.screenshot, "wb") as handle:
            handle.write(base64.b64decode(shot["data"]))
    for line in page.console:
        print(line)
    print("materia:", json.dumps(state))
    if state.get("state") != "running" or state.get("frames", 0) < options.frames:
        print("smoke: the editor did not reach", options.frames, "frames", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
