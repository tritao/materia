#!/usr/bin/env python3
"""Check artifact opening, warm preparation, persistence and failed replacement in Chrome."""
import argparse
import base64
import json
import os
import time
from smoke import Page, WebSocket, wait_for_page
from tour import Editor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug-port', type=int, required=True)
    parser.add_argument('--page-url', required=True)
    parser.add_argument('--artifact', required=True)
    parser.add_argument('--timeout', type=float, default=180)
    options = parser.parse_args()
    page = Page(WebSocket(wait_for_page(options.debug_port, options.page_url, 30)['webSocketDebuggerUrl']))
    page.socket.socket.settimeout(options.timeout)
    page.command('Runtime.enable')
    page.command('Page.enable')
    deadline = time.monotonic() + options.timeout

    def wait(expression):
        while time.monotonic() < deadline:
            value = page.evaluate(expression)
            if value:
                return value
            if page.evaluate('window.materia && materia.state === "failed"'):
                raise RuntimeError('Browser failed: ' + '\n'.join(page.console[-12:]))
            time.sleep(.1)
        raise RuntimeError('Timed out: ' + expression + '\n' + '\n'.join(page.console[-15:]))

    def async_js(expression):
        result = page.command('Runtime.evaluate', {'expression': expression, 'returnByValue': True, 'awaitPromise': True})
        if result.get('exceptionDetails'):
            raise RuntimeError(str(result['exceptionDetails']))
        return result.get('result', {}).get('value')

    wait('window.materia && materia.state === "running" && materia.frames > 10')
    encoded = base64.b64encode(open(options.artifact, 'rb').read()).decode()
    page.evaluate('window.artifactBytes = Uint8Array.from(atob(' + json.dumps(encoded) + '), c => c.charCodeAt(0)); 0')
    # Hold the real worker in synchronous work at its preparation boundary so a fast
    # machine cannot finish before CDP dispatches the user's Cancel click.
    async_js("""(async () => {
      const RealWorker = window.Worker;
      let source = await (await fetch('preparation-worker.js')).text();
      for (const name of ['haxeon-host.js', 'materia_preparation.wasm'])
        source = source.replaceAll(JSON.stringify(name), JSON.stringify(new URL(name, location.href).href));
      const hold = `const originalPost = self.postMessage.bind(self);
        self.postMessage = (message, transfer) => {
          originalPost(message, transfer);
          if (message.type === 'progress' && message.phase === 'Preparing geometry and physics') {
            const until = performance.now() + 5000;
            while (performance.now() < until) {}
          }
        };`;
      const url = URL.createObjectURL(new Blob([hold, source], {type: 'application/javascript'}));
      window.Worker = function(path, options) {
        return new RealWorker(path === 'preparation-worker.js' ? url : path, options);
      };
      window.restorePreparationWorker = () => { window.Worker = RealWorker; URL.revokeObjectURL(url); };
    })()""")
    initial = Editor(page, options.timeout).report()['objects']
    async_js('materia.openArtifact(window.artifactBytes, "Fixture.mtrg")')
    wait('materia.preparation().active?.phase === "Preparing geometry and physics"')
    Editor(page, options.timeout).settle(3)
    Editor(page, options.timeout).click(key='start-cancel-load')
    wait('!materia.preparation().active')
    page.evaluate('window.restorePreparationWorker(); 0')
    assert Editor(page, options.timeout).report()['objects'] == initial, 'Cancelled preparation replaced scene'
    assert not any(p.endswith('.mtrp') for p in Editor(page, options.timeout).report()['files']), 'Cancelled preparation published cache'
    profiles = []
    expected = None
    for attempt in range(2):
        page.evaluate('delete materia.projectOpenProfile; 0')
        if attempt == 0:
            async_js('materia.openArtifact(window.artifactBytes, "Fixture.mtrg")')
        else:
            page.evaluate('window.showOpenFilePicker = undefined')
            page.command('Page.setInterceptFileChooserDialog', {'enabled': True})
            mark = len(page.events)
            Editor(page, options.timeout).click(role='button', label='Open')
            chooser = None
            while time.monotonic() < deadline and chooser is None:
                page.evaluate('0')
                chooser = next((params for name, params in page.events[mark:] if name == 'Page.fileChooserOpened'), None)
                time.sleep(.05)
            assert chooser, 'File > Open did not show the chooser'
            page.command('DOM.setFileInputFiles', {'files': [os.path.abspath(options.artifact)], 'backendNodeId': chooser['backendNodeId']})
            page.command('Page.setInterceptFileChooserDialog', {'enabled': False})
        profile = wait('materia.projectOpenProfile')
        profiles.append(profile)
        preparation = page.evaluate('materia.preparation().last')
        if attempt: assert preparation['id'] == cold_job, 'Cached open started another worker job'
        if not attempt:
            cold_job = preparation['id']
            assert preparation['framesDuringPreparation'] > 0, 'UI stopped drawing during preparation'
        state = Editor(page, options.timeout).report()
        assert state['objects'], 'Artifact has no objects'
        if expected is None:
            expected = state['objects']
        assert state['objects'] == expected, 'Reopen changed scene'
        if attempt:
            assert not any(p['phase'] == 'Preparing geometry and physics' for p in profile['loading']['phases']), 'Warm open missed cache'
        assert any(p.endswith('.mtrp') for p in state['files']), 'Prepared cache was not written'
    async_js('materia.filesSettled()')
    page.command('Page.reload')
    wait('window.materia && materia.state === "running" && materia.frames > 10')
    state = Editor(page, options.timeout).report()
    assert any(p.endswith('/Fixture.mtrg') for p in state['files'])
    # The imported bytes and cache must both have survived a fresh guest instance.
    restored = async_js('''(async () => {
      const root = await navigator.storage.getDirectory();
      async function find(dir) {
        for await (const [name, entry] of dir.entries()) {
          if (entry.kind === 'directory') { const found = await find(entry); if (found) return found; }
          else if (name === 'Fixture.mtrg') return await (await entry.getFile()).arrayBuffer();
        }
      }
      const bytes = await find(root); if (!bytes) throw Error('Imported artifact missing in OPFS');
      await materia.openArtifact(bytes, 'Fixture.mtrg'); return true;
    })()''')
    assert restored
    profile = wait('materia.projectOpenProfile')
    profiles.append(profile)
    assert not any(p['phase'] == 'Preparing geometry and physics' for p in profile['loading']['phases']), 'Reload missed persisted cache'
    assert Editor(page, options.timeout).report()['objects'] == expected
    async_js('materia.filesSettled()')
    async_js("""(async () => {
      async function corrupt(dir) {
        for await (const [name, entry] of dir.entries()) {
          if (entry.kind === 'directory') { if (await corrupt(entry)) return true; }
          else if (name.endsWith('.mtrp')) {
            const bytes = new Uint8Array(await (await entry.getFile()).arrayBuffer());
            bytes[bytes.length - 1] ^= 1;
            const out = await entry.createWritable(); await out.write(bytes); await out.close(); return true;
          }
        }
        return false;
      }
      if (!await corrupt(await navigator.storage.getDirectory())) throw Error('No prepared cache to corrupt');
    })()""")
    page.command('Page.reload')
    wait('window.materia && materia.state === "running" && materia.frames > 10')
    async_js("""(async () => {
      async function open(dir) {
        for await (const [name, entry] of dir.entries()) {
          if (entry.kind === 'directory') { if (await open(entry)) return true; }
          else if (name === 'Fixture.mtrg') {
            await materia.openArtifact(await (await entry.getFile()).arrayBuffer(), name); return true;
          }
        }
        return false;
      }
      if (!await open(await navigator.storage.getDirectory())) throw Error('No persisted artifact');
    })()""")
    repaired = wait('materia.projectOpenProfile')
    assert any(p['phase'] == 'Preparing geometry and physics' for p in repaired['loading']['phases']), 'Corrupt cache was not repaired'
    assert Editor(page, options.timeout).report()['objects'] == expected
    async_js('materia.openArtifact(new Uint8Array([1,2,3]), "Broken.mtrg")')
    time.sleep(1)
    assert Editor(page, options.timeout).report()['objects'] == expected, 'Invalid replacement lost scene'
    print(json.dumps({'objects': len(expected), 'profiles': profiles}, indent=2))
    print('Browser artifact tests passed')


if __name__ == '__main__':
    main()
