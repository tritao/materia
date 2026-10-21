#!/usr/bin/env python3
"""Open the page as `?example=gantry-picker` (the docs' "Open in Materia" link) and check the example loads by itself."""
import argparse
import time
from smoke import Page, WebSocket, wait_for_page
from tour import Editor, TourFailure


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug-port', type=int, required=True)
    parser.add_argument('--page-url', required=True)
    parser.add_argument('--launch', action='store_true')
    parser.add_argument('--timeout', type=float, default=360)
    options = parser.parse_args()
    page = Page(WebSocket(wait_for_page(options.debug_port, options.page_url, 30)['webSocketDebuggerUrl']))
    page.socket.socket.settimeout(options.timeout)
    page.command('Runtime.enable')
    page.command('Page.enable')
    editor = Editor(page, options.timeout)
    deadline = time.monotonic() + options.timeout

    def wait(expression):
        while time.monotonic() < deadline:
            value = page.evaluate(expression)
            if value:
                return value
            failure = None
            if page.evaluate('window.materia?.state === "running"'):
                try:
                    failure = editor.report()['exampleFailure']
                except TourFailure:
                    pass  # The page runs a moment before the editor exists.
            if failure:
                raise RuntimeError(failure)
            if page.evaluate('window.materia?.state === "failed"'):
                raise RuntimeError('\n'.join(page.console[-15:]))
            time.sleep(.05)
        raise RuntimeError('Timed out: ' + expression + '\n' + '\n'.join(page.console[-15:]))

    page.command('Page.navigate', {'url': options.page_url + '?example=gantry-picker'})
    wait('window.materia?.state === "running" && materia.frames > 10')
    wait('materia.projectOpenProfile')
    state = editor.report()
    assert len(state['objects']) > 100, 'The launched example did not load'
    assert state['dynamicParts'], 'The launched example lost its dynamic parts'
    print('launched gantry-picker: %d objects' % len(state['objects']), flush=True)

    # An unknown id leaves the editor running with nothing loaded.
    page.command('Page.navigate', {'url': options.page_url + '?example=no-such-example'})
    wait('window.materia?.state === "running" && materia.frames > 10')
    editor.settle(12)
    assert not page.evaluate('materia.projectOpenProfile'), 'An unknown example id opened something'
    print('unknown id ignored', flush=True)


if __name__ == '__main__':
    main()
