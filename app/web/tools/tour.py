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
import shutil
import sys
import tempfile
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
    """The steps a user takes most: start a scene, add and edit objects, undo, look in 3D, simulate."""
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

    print("mounting plate")
    # A mounting plate is a CadKit part. A build without OCCT lacks CadKit: the host throws haxeon.wasm.HostError,
    # which the editor reports and survives.
    before = len(editor.report()["objects"])
    cadkit = "cadkit-core" not in json.loads(editor.page.evaluate("JSON.stringify(window.materia.unavailable)"))
    editor.click(role="button", label="Add")
    editor.click(role="menuitem", label="Add mounting plate")
    after = editor.report()
    if cadkit:
        check(len(after["objects"]) == before + 1, "CadKit builds a mounting plate")
        check(after["selected"] == after["objects"][-1]["id"], "the plate is selected")
    else:
        check(len(after["objects"]) == before, "a mounting plate needs CadKit, which this build lacks")
        check(any("not available in this build" in line for line in after["state"]["recentLog"]),
              "the editor logs why and keeps running")

    print("stock and workers")
    # AnimKit poses a worker; StockKit carves a stock simulation's stock, whose tool paths need CadKit.
    steps = [("Add worker", "AnimKit")] + ([("Add stock simulation", "StockKit")] if cadkit else [])
    for label, kit in steps:
        before = len(editor.report()["objects"])
        editor.click(role="button", label="Add")
        editor.click(role="menuitem", label=label)
        added = editor.report()
        check(len(added["objects"]) == before + 1, f"{label} adds an object ({kit})")
        check(added["stateError"] is None, f"the editor state is readable after {label}")

    print("3D view")
    editor.click(role="tab", label="3D")
    check(editor.report()["state"]["perspective"] is not None, "the 3D view is open")

    print("simulation")
    # SimKit, MuJoCo and RobotKit run in the page, single-threaded: the editor steps the session each frame.
    editor.click(role="button", label="Play")
    playing = editor.report()
    check(playing["simulationError"] is None, f"the simulation builds: {playing['simulationError']}")
    check(playing["simulationActive"] and playing["simulationRunning"], "Play starts the simulation")
    editor.settle(60)
    check(editor.report()["simulationRunning"], "the simulation keeps running")
    editor.click(role="button", label="Stop")
    stopped = editor.report()
    check(not stopped["simulationRunning"], "Stop ends the simulation")
    check(stopped["mode"] == "design", "and returns to Design mode")

def wait_until_running(editor):
    """Waits for the editor to start and draw, after the page loads or reloads."""
    while True:
        try:
            status = editor.status()
            if status["state"] == "running" and editor.frames() >= 30:
                return
        except TimeoutError:
            # The page answers late while the browser compiles the guest.
            status = {"state": "loading", "error": None}
        if status["state"] == "failed" or time.monotonic() > editor.deadline:
            raise TourFailure(f"the editor did not start: {status['error']}")
        time.sleep(0.2)


def wait_for(editor, condition, what):
    """Lets frames run until `condition()` holds."""
    while not condition():
        if time.monotonic() > editor.deadline:
            raise TourFailure(f"timed out waiting for {what}")
        editor.settle(4)


def documents(editor):
    """Save the scene to the user's disk and open it again, through the browser's file chooser.

    Without the File System Access API (Firefox, Safari, and this tour, which hides it) saving downloads the file and
    opening uses a file input, which the protocol can answer."""
    print("save and open")
    page = editor.page
    page.evaluate("window.showSaveFilePicker = undefined; window.showOpenFilePicker = undefined")
    downloads = tempfile.mkdtemp(prefix="materia-tour-")
    try:
        page.command("Page.setDownloadBehavior", {"behavior": "allow", "downloadPath": downloads})
        saved = editor.report()["objects"]
        editor.click(role="button", label="Save")
        target = os.path.join(downloads, "Untitled.materia.json")
        wait_for(editor, lambda: os.path.exists(target) and os.path.getsize(target) > 0, "the download")
        with open(target, encoding="utf-8") as handle:
            document = json.load(handle)
        check(document.get("format") == "materia.scene", "Save downloads the scene as a Materia document")
        check(len(document.get("objects", [])) == len(saved), f"with its {len(saved)} objects")
        check("/files/Untitled.materia.json" in editor.report()["files"], "and keeps it in the editor's storage")

        editor.click(role="button", label="New")
        wait_for(editor, lambda: len(editor.report()["objects"]) == 0, "an empty scene")
        page.command("Page.setInterceptFileChooserDialog", {"enabled": True})
        mark = len(page.events)
        editor.click(role="button", label="Open")
        wait_for(editor, lambda: any(name == "Page.fileChooserOpened" for name, _ in page.events[mark:]),
                 "the file chooser")
        chooser = next(params for name, params in page.events[mark:] if name == "Page.fileChooserOpened")
        page.command("DOM.setFileInputFiles", {"files": [target], "backendNodeId": chooser["backendNodeId"]})
        wait_for(editor, lambda: len(editor.report()["objects"]) == len(saved), "the opened scene")
        opened = editor.report()
        check(sorted(o["label"] for o in opened["objects"]) == sorted(o["label"] for o in saved),
              "Open brings the saved objects back")
        page.command("Page.setInterceptFileChooserDialog", {"enabled": False})
    finally:
        shutil.rmtree(downloads, ignore_errors=True)


def persistence(editor):
    """The editor's files (settings, workspace, documents) outlive the page: reload it and look for them."""
    print("files across a reload")
    before = editor.report()["files"]
    check(len(before) > 0, f"the editor keeps files: {', '.join(before)}")
    editor.page.command("Runtime.evaluate", {"expression": "window.materia.filesSettled()", "awaitPromise": True})
    editor.page.command("Page.reload")
    wait_until_running(editor)
    restored = editor.page.evaluate("window.materia.restoredFiles")
    check(restored >= len(before), f"the page restored {restored} file(s) from storage")
    after = editor.report()["files"]
    check(set(before) <= set(after), "they are all back after a reload")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug-port", type=int, required=True)
    parser.add_argument("--page-url", required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--screenshot")
    parser.add_argument("--dump", action="store_true", help="print the first report's widgets and stop")
    options = parser.parse_args()

    target = wait_for_page(options.debug_port, options.page_url, 30)
    socket = WebSocket(target["webSocketDebuggerUrl"])
    # The first report can take many seconds: Json.stringify's wasm-gc reflection is slow until the browser
    # optimizes it.
    socket.socket.settimeout(90)
    page = Page(socket)
    page.command("Runtime.enable")
    page.command("Page.enable")
    editor = Editor(page, options.timeout)
    try:
        wait_until_running(editor)
    except TourFailure as error:
        print("tour:", error, file=sys.stderr)
        return 1
    failure = None
    try:
        if options.dump:
            report = editor.report()
            for widget in report["widgets"]:
                print(json.dumps(widget))
            print("stateError:", report["stateError"])
        else:
            tour(editor)
            documents(editor)
            persistence(editor)
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
