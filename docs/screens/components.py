#!/usr/bin/env python3
"""Thumbnails for the component catalogue, rendered from its own models.

Loads every GLB in docs/site/public/components/ into the same <model-viewer> the site uses (headless Chrome,
software GL) and writes docs/site/public/components/thumbs/NAME.webp. Run after the docs-components export:

    docs/screens/components.py [NAME.glb ...]
"""
import base64
import io
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SITE = os.path.join(REPO, "docs/site")
MODELS = os.path.join(SITE, "public/components")
THUMBS = os.path.join(MODELS, "thumbs")
sys.path.insert(0, os.path.join(REPO, "app/web/tools"))
from smoke import Page, WebSocket, wait_for_page  # noqa: E402
from PIL import Image  # noqa: E402

WIDTH, HEIGHT = 640, 480
PAGE = """<!doctype html><meta charset="utf-8">
<style>html,body{margin:0;background:#e7ebf0}model-viewer{width:%dpx;height:%dpx;background:linear-gradient(#eef1f5,#dfe4ea)}</style>
<script type="module" src="/node_modules/@google/model-viewer/dist/model-viewer.min.js"></script>
<model-viewer id="viewer" shadow-intensity="0.6" exposure="1.0" camera-orbit="35deg 65deg auto" interaction-prompt="none"></model-viewer>
<script>
window.show = (src) => new Promise((resolve, reject) => {
  const viewer = document.getElementById("viewer");
  const done = () => { viewer.removeEventListener("load", done); requestAnimationFrame(() => requestAnimationFrame(() => resolve(true))); };
  viewer.addEventListener("load", done);
  viewer.addEventListener("error", (event) => reject(String(event.detail?.sourceError ?? "load error")), { once: true });
  viewer.src = src;
});
</script>
""" % (WIDTH, HEIGHT)


def free_port():
    with socket.socket() as handle:
        handle.bind(("127.0.0.1", 0))
        return handle.getsockname()[1]


def main():
    names = sys.argv[1:] or sorted(name for name in os.listdir(MODELS) if name.endswith(".glb"))
    os.makedirs(THUMBS, exist_ok=True)
    page_path = os.path.join(SITE, "public", "__component-thumbs.html")
    with open(page_path, "w", encoding="utf-8") as handle:
        handle.write(PAGE)
    http_port, debug_port = free_port(), free_port()
    # Serve the site directory, so both node_modules (model-viewer) and public/ (models) are reachable.
    server = subprocess.Popen([sys.executable, "-m", "http.server", str(http_port), "--bind", "127.0.0.1", "--directory", SITE],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    profile = tempfile.mkdtemp()
    url = f"http://127.0.0.1:{http_port}/public/__component-thumbs.html"
    browser = subprocess.Popen(["google-chrome", "--headless=new", "--no-sandbox", "--disable-dev-shm-usage",
                                "--enable-unsafe-swiftshader", "--use-angle=swiftshader", f"--window-size={WIDTH},{HEIGHT}",
                                "--force-device-scale-factor=1", "--hide-scrollbars", f"--user-data-dir={profile}",
                                f"--remote-debugging-port={debug_port}", "--remote-allow-origins=*", url],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    failures = []
    try:
        page = Page(WebSocket(wait_for_page(debug_port, url, 60)["webSocketDebuggerUrl"]))
        page.socket.socket.settimeout(120)
        page.command("Runtime.enable")
        # The window's chrome takes height from the viewport; pin it to the viewer's size.
        page.command("Emulation.setDeviceMetricsOverride", {"width": WIDTH, "height": HEIGHT, "deviceScaleFactor": 1, "mobile": False})
        deadline = time.monotonic() + 60
        while not page.evaluate("!!customElements.get('model-viewer')"):
            if time.monotonic() > deadline:
                raise RuntimeError("model-viewer did not load")
            time.sleep(0.2)
        for name in names:
            result = page.command("Runtime.evaluate", {"expression": f"window.show('/public/components/{name}')",
                                                        "awaitPromise": True, "returnByValue": True})
            if "exceptionDetails" in result:
                failures.append(name)
                print(f"failed {name}: {result['exceptionDetails'].get('text')}", file=sys.stderr, flush=True)
                continue
            shot = page.command("Page.captureScreenshot", {"format": "png",
                                                           "clip": {"x": 0, "y": 4, "width": WIDTH, "height": HEIGHT - 4, "scale": 1}})
            image = Image.open(io.BytesIO(base64.b64decode(shot["data"]))).convert("RGB").resize((320, 240), Image.LANCZOS)
            image.save(os.path.join(THUMBS, name.replace(".glb", ".webp")), quality=82)
        print(f"{len(names) - len(failures)} thumbnails", flush=True)
    finally:
        browser.kill()
        server.kill()
        browser.wait()
        shutil.rmtree(profile, True)
        os.remove(page_path)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
