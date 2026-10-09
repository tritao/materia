#!/usr/bin/env python3
"""Exercise the actual Start page, lazy network acquisition, cancellation and offline reopening."""
import argparse
import json
import time
from smoke import Page, WebSocket, wait_for_page
from tour import Editor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug-port', type=int, required=True)
    parser.add_argument('--page-url', required=True)
    parser.add_argument('--examples', action='store_true')
    parser.add_argument('--timeout', type=float, default=240)
    options = parser.parse_args()
    page = Page(WebSocket(wait_for_page(options.debug_port, options.page_url, 30)['webSocketDebuggerUrl']))
    page.socket.socket.settimeout(options.timeout)
    page.command('Runtime.enable')
    page.command('Page.enable')
    deadline = time.monotonic() + options.timeout
    editor = Editor(page, options.timeout)
    global failure_editor
    failure_editor = editor

    def wait(expression):
        while time.monotonic() < deadline:
            value = page.evaluate(expression)
            if value:
                return value
            if 'projectOpenProfile' in expression and editor.report()['exampleFailure']:
                raise RuntimeError(editor.report()['exampleFailure'])
            if page.evaluate('window.materia?.state === "failed"'):
                raise RuntimeError('\n'.join(page.console[-15:]))
            time.sleep(.05)
        raise RuntimeError('Timed out: ' + expression + '\n' + '\n'.join(page.console[-15:]))

    def async_js(expression):
        response = page.command('Runtime.evaluate', {'expression': expression, 'returnByValue': True, 'awaitPromise': True})
        if response.get('exceptionDetails'):
            raise RuntimeError(str(response['exceptionDetails']))
        return response.get('result', {}).get('value')

    def show_start():
        state = editor.report()
        if not any(widget.get('label') == 'Search examples...' for widget in state['widgets']):
            editor.click(role='tab', label='Start')

    def scroll(amount):
        pane = max((widget for widget in editor.report()['widgets'] if widget.get('key') == 'start-scroll'),
            key=lambda widget: widget['width'] * widget['height'])
        page.command('Input.dispatchMouseEvent', {'type': 'mouseWheel', 'x': pane['x'] + pane['width'] / 2,
            'y': pane['y'] + pane['height'] / 2, 'deltaX': 0, 'deltaY': amount})
        editor.settle(6)

    def browse(value):
        # Browse the actual category filters and scrollable cards, without depending on text-input plumbing.
        show_start()
        scroll(-10000)
        category, example = {
            'Gantry picker': ('Robotics & handling', 'gantry-picker'),
            'woven seam': ('Welding', 'robot-welder-weave'),
            'Worker gallery': ('People & simulation', 'worker-gallery'),
            'Two robots with sensors': ('People & simulation', 'two-robot')
        }[value]
        editor.click(key='start-category:' + category)
        for attempt in range(30):
            state = editor.report()
            if any(widget.get('key') == 'start-open-example:' + example and widget['height'] > 25 for widget in state['widgets']):
                return
            scroll(220)
        raise RuntimeError('Example card was not reachable: ' + example)

    def open_gantry():
        browse('Gantry picker')
        editor.click(key='start-open-example:gantry-picker')

    def paused_request(mark):
        while time.monotonic() < deadline:
            page.evaluate('0')
            found = next((params for name, params in page.events[mark:] if name == 'Fetch.requestPaused'), None)
            if found:
                return found
            time.sleep(.05)
        raise RuntimeError('Example was not fetched')

    wait('window.materia?.state === "running" && materia.frames > 10')
    catalog = page.evaluate('materia.examples().catalog')
    state = editor.report()
    assert len(catalog) == 25 and state['examples'] == catalog, 'Browser and desktop catalog differ'
    assert page.evaluate('materia.examples().requests.length') == 0, 'Start page eagerly downloaded example data'
    print('catalog', flush=True)
    browse('Gantry picker')
    editor.widget(editor.report(), key='start-open-example:gantry-picker')
    initial = editor.report()['objects']

    # Hold the real fetch until the user cancels it; no source or cache should be installed.
    page.command('Fetch.enable', {'patterns': [{'urlPattern': '*/examples/objects/*.mtrg', 'requestStage': 'Request'}]})
    mark = len(page.events)
    open_gantry()
    paused_request(mark)
    scroll(-10000)
    editor.click(key='start-cancel-load')
    wait('!materia.examples().active')
    page.command('Fetch.disable')
    assert editor.report()['objects'] == initial, 'Cancelled download replaced scene'
    assert not any(path.endswith('.mtrg') for path in editor.report()['files']), 'Cancelled download published a file'

    print('cancelled', flush=True)
    # A failed request preserves the scene and allows another click to retry.
    page.command('Fetch.enable', {'patterns': [{'urlPattern': '*/examples/objects/*.mtrg', 'requestStage': 'Request'}]})
    mark = len(page.events)
    open_gantry()
    request = paused_request(mark)
    page.command('Fetch.failRequest', {'requestId': request['requestId'], 'errorReason': 'Failed'})
    wait('!materia.examples().active')
    page.command('Fetch.disable')
    assert editor.report()['objects'] == initial, 'Failed download replaced scene'
    assert editor.report()['exampleFailure'], 'Download failure was not shown'

    print('retry', flush=True)
    open_gantry()
    cold = wait('materia.projectOpenProfile')
    expected = editor.report()['objects']
    assert len(expected) > 100, 'Gantry example did not load'
    assert editor.report()['dynamicParts'], 'Published project lost dynamic parts'
    request_count = page.evaluate('materia.examples().requests.length')
    assert request_count == 3, 'Unexpected example fetches'
    page.evaluate('delete materia.projectOpenProfile; 0')
    open_gantry()
    warm = wait('materia.projectOpenProfile')
    assert page.evaluate('materia.examples().requests.length') == request_count, 'Cached example was downloaded again'
    assert editor.report()['objects'] == expected
    async_js('materia.filesSettled()')

    print('reload', flush=True)
    page.command('Page.reload')
    wait('window.materia?.state === "running" && materia.frames > 10')
    browse('Gantry picker')
    # Block every example model request. Persisted raw and prepared data must suffice after reload.
    page.command('Network.enable')
    page.command('Network.setBlockedURLs', {'urls': ['*/examples/objects/*']})
    open_gantry()
    restored = wait('materia.projectOpenProfile')
    assert page.evaluate('materia.examples().requests.length') == 0, 'Reload did not reuse stored example'
    assert editor.report()['objects'] == expected
    page.command('Network.setBlockedURLs', {'urls': []})
    print('persisted gantry', flush=True)
    # A job variant must keep the selected mission instead of falling back to the default project.
    browse('woven seam')
    page.evaluate('delete materia.projectOpenProfile; 0')
    editor.click(key='start-open-example:robot-welder-weave')
    while editor.report()['projectJob'] != 'woven-seam':
        wait('materia.frames > 10')
        if time.monotonic() >= deadline: raise RuntimeError('Welding variant did not open')
        time.sleep(.05)
    assert editor.report()['projectJob'] == 'woven-seam', 'Published welding variant lost its selected job'

    print('job variant', flush=True)
    print('compiled setup', flush=True)
    before_script = page.evaluate('materia.examples().requests.length')
    browse('Two robots with sensors')
    page.evaluate('delete materia.projectOpenProfile; 0')
    editor.click(key='start-open-example:two-robot')
    while not any(item['id'] == 'moving-obstacle' for item in editor.report()['objects']):
        if time.monotonic() >= deadline: raise RuntimeError('Compiled setup did not open')
        time.sleep(.05)
    assert any(item['id'] == 'moving-obstacle' for item in editor.report()['objects']), 'Compiled setup example did not open'
    assert page.evaluate('materia.examples().requests.length') == before_script, 'Compiled setup downloaded model data'
    # Saved examples acquire their documents and native animation assets through the same stage.
    browse('Worker gallery')
    page.evaluate('delete materia.projectOpenProfile; 0')
    editor.click(key='start-open-example:worker-gallery')
    while not editor.report()['simulationActive']:
        if time.monotonic() >= deadline: raise RuntimeError('Worker gallery did not open')
        time.sleep(.05)
    gallery = editor.report()
    assert sum(item['type'] == 'human-worker' for item in gallery['objects']) == 6, 'Worker gallery was not loaded'
    assert gallery['simulationRunning'] and not gallery['simulationError'], 'Worker example did not start'
    assert page.evaluate('Module.FS.stat("/animkit/assets/quaternius-ual/ual-work.glb").size') > 4000000, 'Native asset was not staged'
    async_js('materia.filesSettled()')
    print('reload', flush=True)
    page.command('Page.reload')
    wait('window.materia?.state === "running" && materia.frames > 10')
    browse('Worker gallery')
    page.command('Network.setBlockedURLs', {'urls': ['*/examples/objects/*']})
    editor.click(key='start-open-example:worker-gallery')
    while not editor.report()['simulationActive']:
        if time.monotonic() >= deadline: raise RuntimeError('Stored worker gallery did not open')
        time.sleep(.05)
    assert editor.report()['simulationRunning'], 'Persisted native assets did not restore'
    assert page.evaluate('materia.examples().requests.length') == 0, 'Persisted worker example was downloaded again'
    page.command('Network.setBlockedURLs', {'urls': []})

    print(json.dumps({'examples': len(catalog), 'objects': len(expected), 'profiles': [cold, warm, restored]}, indent=2))
    print('Browser lazy example tests passed')


if __name__ == '__main__':
    try:
        main()
    except Exception:
        if 'failure_editor' in globals():
            try:
                state = failure_editor.report()
                print(json.dumps({'failure': state['exampleFailure'], 'widgets': [widget for widget in state['widgets']
                    if widget.get('role') in (5, 18) or 'start-' in (widget.get('key') or '')]}, indent=2), flush=True)
            except Exception:
                pass
        raise
