#!/usr/bin/env python3
"""Run a published robot's real pick mission through the browser host."""
import argparse
import json
import time
from smoke import Page, WebSocket, wait_for_page
from tour import Editor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug-port', type=int, required=True)
    parser.add_argument('--page-url', required=True)
    parser.add_argument('--missions', action='store_true')
    parser.add_argument('--timeout', type=float, default=240)
    options = parser.parse_args()
    page = Page(WebSocket(wait_for_page(options.debug_port, options.page_url, 30)['webSocketDebuggerUrl']))
    page.socket.socket.settimeout(options.timeout)
    page.command('Runtime.enable')
    deadline = time.monotonic() + options.timeout
    editor = Editor(page, options.timeout)
    while page.evaluate('window.materia?.state') != 'running':
        if time.monotonic() >= deadline:
            raise RuntimeError('Browser startup timed out')
        time.sleep(.1)
    editor.settle()
    editor.click(key='start-category:Robotics & handling')
    editor.click(key='start-open-example:robot-arm')
    while not page.evaluate('materia.projectOpenProfile'):
        state = editor.report()
        if state['exampleFailure']:
            raise RuntimeError(state['exampleFailure'])
        if time.monotonic() >= deadline:
            raise RuntimeError('Robot arm did not open')
        time.sleep(.1)
    editor.click(role='button', label='Play')
    editor.click(role='button', label='Pause')
    assert not editor.report()['simulationRunning'], 'Pause did not stop simulation stepping'
    editor.click(role='button', label='Play')
    while time.monotonic() < deadline:
        state = editor.report()
        if state['simulationError']:
            raise RuntimeError(state['simulationError'])
        mission = state['mission']
        if mission is None:
            raise RuntimeError('Robot arm has no mission')
        if mission['failure']:
            raise RuntimeError(mission['failure'])
        if mission['loop'] > 1 or mission['phase'] == 'complete':
            break
        if page.evaluate('materia.state') != 'running':
            raise RuntimeError('\n'.join(page.console[-10:]))
        time.sleep(.2)
    else:
        raise RuntimeError('Robot mission cycle did not complete: ' + json.dumps(mission))
    assert not any('Project UI extension:' in line for line in page.console), 'Artifact was treated as a UI manifest'
    assert not any('is not available in this build' in line for line in page.console), 'Mission used a missing native function'
    print(json.dumps(mission, indent=2))
    print('Browser robot mission tests passed')


if __name__ == '__main__':
    main()
