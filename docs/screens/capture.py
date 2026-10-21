#!/usr/bin/env python3
"""Screenshots for the docs site, taken from the editor's browser build.

Serves a built web editor (app/build/web/site, with the examples published into it), opens each example
through `?example=ID` in headless Chrome, lets it settle, and writes WebP images to
docs/site/public/screens/: `editor-ID.webp` (the whole editor) for the hero, `ID.webp` (the 3D view) for
the rest. The images come from the real editor, so they are regenerated rather than drawn.

    docs/screens/capture.py [--site DIR] [--editor ID] ID...
"""
import argparse
import base64
import io
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "app/web/tools"))
from smoke import Page, WebSocket, wait_for_page  # noqa: E402
from tour import Editor, TourFailure  # noqa: E402
from PIL import Image  # noqa: E402

OUT = os.path.join(REPO, "docs/site/public/screens")
WIDTH, HEIGHT = 1600, 1000


def free_port():
    with socket.socket() as handle:
        handle.bind(("127.0.0.1", 0))
        return handle.getsockname()[1]


def viewport(report):
    """The 3D view's bounds: the largest visible widget whose style key names the viewport."""
    views = [w for w in report["widgets"] if "viewport" in (w.get("key") or "").lower() and w["width"] > 200]
    if not views:
        keys = sorted({w.get("key") for w in report["widgets"] if w.get("key")})
        raise RuntimeError("no 3D view among the widgets; keys: " + ", ".join(keys))
    return max(views, key=lambda w: w["width"] * w["height"])


def capture(page, editor, url, example, whole):
    page.command("Page.navigate", {"url": f"{url}?example={example}"})
    deadline = time.monotonic() + 600
    while time.monotonic() < deadline:
        if page.evaluate('window.materia?.state === "running" && !!materia.projectOpenProfile'):
            break
        if page.evaluate('window.materia?.state === "failed"'):
            raise RuntimeError("\n".join(page.console[-10:]))
        time.sleep(0.25)
    else:
        raise RuntimeError(f"{example} did not open")
    editor.settle(30)
    view = viewport(editor.report())
    centre = (view["x"] + view["width"] / 2, view["y"] + view["height"] * 0.6)

    def wheel(delta):
        editor.mouse("mouseMoved", centre[0], centre[1])
        page.command("Input.dispatchMouseEvent", {"type": "mouseWheel", "x": centre[0], "y": centre[1],
                                                   "deltaX": 0, "deltaY": delta})
        # Move the pointer off the view, or the part under it is drawn with the hover tint.
        editor.mouse("mouseMoved", 4, HEIGHT - 4)
        editor.settle(30)

    if whole:
        # The hero shows the editor as a user first sees it, framed and zoomed in a little.
        grid(page, editor, True)
        editor.click(label="Fit")
        wheel(-360)
        hero = grab(page).convert("RGB")
        hero.save(os.path.join(OUT, f"editor-{example}.webp"), quality=86)
        # The social preview (og:image): 1200 x 630 PNG, which every platform accepts.
        preview = hero.resize((1200, round(hero.height * 1200 / hero.width)), Image.LANCZOS)
        top = max(0, (preview.height - 630) // 2)
        preview.crop((0, top, 1200, top + 630)).save(os.path.join(OUT, "social.png"), optimize=True)
    # The thumbnail has no grid and sits one notch out, clear of the view's overlays; it is cropped to the machine.
    editor.click(label="Fit")
    grid(page, editor, False)
    wheel(240)
    shot = grab(page)
    scale = shot.width / WIDTH
    view = viewport(editor.report())
    # Leave out the view's own overlays along its top (camera menu, run status).
    top = view["y"] + min(160, view["height"] * 0.25)
    box = [round(value * scale) for value in (view["x"], top, view["x"] + view["width"], view["y"] + view["height"])]
    crop = machine(shot.crop(box).convert("RGB"))
    crop.resize((640, 400), Image.LANCZOS).save(os.path.join(OUT, f"{example}.webp"), quality=84)
    print(f"captured {example}", flush=True)


def machine(view):
    """The part of the 3D view the machine covers, padded and widened to 16:10.

    With the grid off the background is a smooth gradient, so pixels far from the background colour of their
    row (sampled at the view's left and right edges) are the machine.
    """
    width, height = view.size
    pixels = view.load()
    left, top, right, bottom = width, height, 0, 0
    for y in range(0, height, 2):
        background = [(a + b) // 2 for a, b in zip(pixels[2, y], pixels[width - 3, y])]
        for x in range(0, width, 2):
            if sum(abs(c - b) for c, b in zip(pixels[x, y], background)) > 60:
                left, top, right, bottom = min(left, x), min(top, y), max(right, x), max(bottom, y)
    if right <= left or bottom <= top:
        return view
    pad = 0.12 * max(right - left, bottom - top)
    left, top, right, bottom = left - pad, top - pad, right + pad, bottom + pad
    cx, cy = (left + right) / 2, (top + bottom) / 2
    w, h = right - left, bottom - top
    if w / h < 1.6:
        w = h * 1.6
    else:
        h = w / 1.6
    w, h = min(w, width), min(h, height, w / 1.6)
    w = h * 1.6
    left = min(max(0, cx - w / 2), width - w)
    top = min(max(0, cy - h / 2), height - h)
    return view.crop((round(left), round(top), round(left + w), round(top + h)))


def grid(page, editor, visible):
    """Show or hide the 3D view's grid; the editor remembers it across loads, so read its state first."""
    if editor.report()["state"].get("gridVisible") != visible:
        editor.click(label="Grid")
    if editor.report()["state"].get("gridVisible") != visible:
        raise RuntimeError(f"could not {'show' if visible else 'hide'} the grid")


def grab(page):
    return Image.open(io.BytesIO(base64.b64decode(page.command("Page.captureScreenshot", {"format": "png"})["data"])))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", default=os.path.join(REPO, "app/build/web/site"))
    parser.add_argument("--editor", action="append", default=[], help="also save the whole editor for this example")
    parser.add_argument("examples", nargs="+")
    options = parser.parse_args()
    os.makedirs(OUT, exist_ok=True)
    http_port, debug_port = free_port(), free_port()
    server = subprocess.Popen([sys.executable, "-m", "http.server", str(http_port), "--bind", "127.0.0.1",
                               "--directory", options.site], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    profile = tempfile.mkdtemp()
    url = f"http://127.0.0.1:{http_port}/index.html"
    browser = subprocess.Popen(["google-chrome", "--headless=new", "--no-sandbox", "--disable-dev-shm-usage",
                                "--enable-unsafe-swiftshader", "--use-angle=swiftshader", f"--window-size={WIDTH},{HEIGHT}",
                                "--force-device-scale-factor=1", "--hide-scrollbars", "--no-first-run",
                                f"--user-data-dir={profile}", f"--remote-debugging-port={debug_port}",
                                "--remote-allow-origins=*", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    failures = []
    try:
        page = Page(WebSocket(wait_for_page(debug_port, url, 60)["webSocketDebuggerUrl"]))
        page.socket.socket.settimeout(300)
        page.command("Runtime.enable")
        page.command("Page.enable")
        editor = Editor(page, 3600)
        for example in options.examples:
            try:
                capture(page, editor, url, example, example in options.editor)
            except (RuntimeError, TourFailure) as error:
                failures.append(f"{example}: {error}")
                print(f"failed {example}: {error}", file=sys.stderr, flush=True)
    finally:
        browser.kill()
        server.kill()
        browser.wait()
        shutil.rmtree(profile, True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
