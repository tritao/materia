#!/usr/bin/env python3
"""Drives the browser editor through a fixed tour and checks the editor's state after each step.

Controls are found through the editor's own report (app.MainWeb.report, printed by window.materia.inspect()):
every visible widget with a style key or accessibility label, with its bounds. Clicks and typing go through
Chrome's input events, so they take the same path as a user's. Exits non-zero at the first failed check.

  app/web/test.sh --tour [--screenshot PATH] [--dump]
"""

import argparse
import base64
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from smoke import Page, WebSocket, wait_for_page  # noqa: E402


class TourFailure(Exception):
    pass


# nativekit.ui.semantics.AccessibilityRole values, by the names WAI-ARIA gives those roles.
ROLES = {"button": 1, "checkbox": 2, "radio": 3, "text": 4, "textbox": 5, "link": 6, "slider": 11, "dialog": 13,
         "menu": 14, "menuitem": 16, "tablist": 17, "tab": 18, "switch": 20, "combobox": 22, "treeitem": 31}


class Editor:
    def __init__(self, page, timeout):
        self.page = page
        self.deadline = time.monotonic() + timeout

    def frames(self):
        return self.page.evaluate("window.materia ? window.materia.frames : 0") or 0

    def status(self):
        # The page defines window.materia once its scripts load.
        status = self.page.evaluate("window.materia ? JSON.stringify({state: materia.state, error: materia.error}) : null")
        return json.loads(status) if status else {"state": "loading", "error": None}

    def settle(self, frames=12):
        """Waits for a few frames so input is handled and the result drawn."""
        start = self.frames()
        while self.frames() < start + frames:
            status = self.status()
            if status["state"] != "running":
                raise TourFailure(f"the editor is {status['state']}: {status['error']}")
            if time.monotonic() > self.deadline:
                raise TourFailure("the tour ran out of time")
            time.sleep(0.05)

    def report(self):
        mark = len(self.page.console)
        result = self.page.evaluate("window.materia.inspect()")
        if result != 0:
            raise TourFailure(f"app.MainWeb.report returned {result}")
        self.page.evaluate("0")
        for line in reversed(self.page.console[mark:]):
            prefix = line.find("materia-report ")
            if prefix >= 0:
                return json.loads(line[prefix + len("materia-report "):])
        raise TourFailure("the editor printed no report")

    def widget(self, report, label=None, key=None, role=None):
        """A visible control by accessible role and name, as a user finds it, or by style key."""
        role_id = None if role is None else ROLES[role]
        matches = [w for w in report["widgets"]
                   if (label is None or w.get("label") == label) and (key is None or w.get("key") == key)
                   and (role_id is None or w.get("role") == role_id)]
        if not matches:
            raise TourFailure(f"no visible widget with role={role!r} label={label!r} key={key!r}")
        # The innermost match is the control itself rather than a container sharing its label.
        return min(matches, key=lambda w: w["width"] * w["height"])

    def mouse(self, kind, x, y, buttons=0):
        self.page.command("Input.dispatchMouseEvent", {
            "type": kind, "x": x, "y": y, "button": "none" if kind == "mouseMoved" else "left",
            "buttons": buttons, "clickCount": 0 if kind == "mouseMoved" else 1})

    def click(self, label=None, key=None, role=None):
        target = self.widget(self.report(), label, key, role)
        if not target.get("enabled", True):
            raise TourFailure(f"widget role={role!r} label={label!r} key={key!r} is disabled")
        x, y = target["x"] + target["width"] / 2, target["y"] + target["height"] / 2
        self.mouse("mouseMoved", x, y)
        self.mouse("mousePressed", x, y, 1)
        self.mouse("mouseReleased", x, y)
        self.settle()

    def key(self, key, code, key_code, modifiers=0, text=None):
        for kind in ("keyDown", "keyUp"):
            event = {"type": kind, "key": key, "code": code, "windowsVirtualKeyCode": key_code, "modifiers": modifiers}
            if text is not None and kind == "keyDown":
                event["text"] = text
            self.page.command("Input.dispatchKeyEvent", event)
        self.settle(4)

    def type_text(self, text):
        self.page.command("Input.insertText", {"text": text})
        self.settle(4)


def check(condition, message):
    if not condition:
        raise TourFailure(message)
    print("  ok:", message)


def enabled(report, command):
    return next(entry["enabled"] for entry in report["commands"] if entry["id"] == command)


def tour(editor):
    """The steps a user takes most: start a scene, add and edit objects, undo, look in 3D."""
    print("start")
    start = editor.report()
    check(start["mode"] == "design", "the editor opens in Design mode")

    print("empty scene")
    editor.click(role="button", label="Empty scene")
    empty = editor.report()
    check(len(empty["objects"]) == 0, "an empty scene has no objects")
    check(not enabled(empty, "editor.undo"), "nothing to undo in a new scene")

    print("add rectangles")
    for count in (1, 2):
        editor.click(role="button", label="Add")
        check(any(w.get("label") == "Add rectangle" for w in editor.report()["widgets"]), "the Add menu lists rectangles")
        editor.click(role="menuitem", label="Add rectangle")
        added = editor.report()
        check(len(added["objects"]) == count, f"the scene has {count} object(s)")
        check(added["selected"] == added["objects"][-1]["id"], "the new rectangle is selected")
    check(enabled(added, "editor.undo"), "adding can be undone")

    print("undo and redo")
    editor.click(role="button", label="Undo")
    undone = editor.report()
    check(len(undone["objects"]) == 1, "undo removes the second rectangle")
    check(enabled(undone, "editor.redo"), "the removal can be redone")
    editor.click(role="button", label="Redo")
    check(len(editor.report()["objects"]) == 2, "redo restores it")

    print("inspector")
    selected = editor.report()["selected"]
    editor.click(role="textbox", label="Position X (m)")
    editor.key("a", "KeyA", 65, modifiers=2)
    editor.type_text("1.5")
    editor.key("Enter", "Enter", 13, text="\r")
    moved = next(o for o in editor.report()["objects"] if o["id"] == selected)
    check(abs(moved["x"] - 1.5) < 1e-6, "editing Position X moves the rectangle")
    check(editor.widget(editor.report(), role="textbox", label="Position X (m)")["value"] == "1.5",
          "the field shows the new position")

    print("delete")
    editor.click(role="button", label="Delete")
    check(len(editor.report()["objects"]) == 1, "delete removes the selected rectangle")

    print("unavailable kit")
    # CadKit has no browser build: the host throws haxeon.wasm.HostError, which the editor reports and survives.
    before = len(editor.report()["objects"])
    editor.click(role="button", label="Add")
    editor.click(role="menuitem", label="Add mounting plate")
    after = editor.report()
    check(len(after["objects"]) == before, "a mounting plate needs CadKit, which the browser build lacks")
    check(any("not available in this build" in line for line in after["state"]["recentLog"]),
          "the editor logs why and keeps running")

    print("3D view")
    editor.click(role="tab", label="3D")
    check(editor.report()["state"]["perspective"] is not None, "the 3D view is open")

    print("simulation")
    # The browser build has no physics engine yet; Play must be off rather than stop the editor.
    final = editor.report()
    check(not enabled(final, "sim.play"), "Play is disabled without a physics engine")
    check(not editor.widget(final, key="toolbar-sim-play")["enabled"], "the Play button shows it")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug-port", type=int, required=True)
    parser.add_argument("--page-url", required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--screenshot")
    parser.add_argument("--dump", action="store_true", help="print the first report's widgets and stop")
    options = parser.parse_args()

    target = wait_for_page(options.debug_port, options.page_url, 30)
    page = Page(WebSocket(target["webSocketDebuggerUrl"]))
    page.command("Runtime.enable")
    page.command("Page.enable")
    editor = Editor(page, options.timeout)
    while True:
        try:
            status = editor.status()
            if status["state"] == "running" and editor.frames() >= 30:
                break
        except TimeoutError:
            # The page answers late while the browser compiles the guest.
            status = {"state": "loading", "error": None}
        if status["state"] == "failed" or time.monotonic() > editor.deadline:
            print("tour: the editor did not start:", status["error"], file=sys.stderr)
            return 1
        time.sleep(0.2)
    failure = None
    try:
        if options.dump:
            report = editor.report()
            for widget in report["widgets"]:
                print(json.dumps(widget))
            print("stateError:", report["stateError"])
        else:
            tour(editor)
    except TourFailure as error:
        failure = str(error)
    if options.screenshot:
        shot = page.command("Page.captureScreenshot", {"format": "png"})
        with open(options.screenshot, "wb") as handle:
            handle.write(base64.b64decode(shot["data"]))
    for line in page.console:
        if "materia-report " not in line:
            print(line)
    if failure is not None:
        print("tour: FAILED:", failure, file=sys.stderr)
        return 1
    print("tour: passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
