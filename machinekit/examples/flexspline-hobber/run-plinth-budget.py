#!/usr/bin/env python3
"""Finish HB11 after HB-D11, preserving the earlier batch and its running cuts.

Requires frozen main.hl and inputs.json in a fresh output directory. Reuses only
unchanged independent sources; reruns wall, axial, tilt and combined. Completed
cases require successful execution, complete logs and identical fingerprints.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--previous', required=True, type=Path)
    parser.add_argument('--native', required=True, type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    directory, previous, native = (p.resolve() for p in [args.directory, args.previous, args.native])
    spec = importlib.util.spec_from_file_location('budget', Path(__file__).with_name('run-reference-budget.py'))
    budget = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(budget)
    save, load, digest = budget.save, budget.load, budget.digest
    directory.mkdir(parents=True, exist_ok=True)
    lock = (directory / 'run.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    original = load(previous / 'settings.json')
    if original is None or digest(directory / 'inputs.json') != digest(previous / 'inputs.json'):
        raise ValueError('same validated frozen force/sync inputs required')
    native_hash = digest(native / 'libstockkit_core.so')
    if native_hash != load(previous / 'execution.json')['native']:
        raise ValueError('validated native implementation must remain unchanged')
    settings = dict(original, threads=4, workers=2, faceEndpoints=True,
                    sourceIds=original['sourceIds'].copy(), supportStiffnessNPerMm=83936.78528166225)
    settings['sourceIds'].insert(settings['sourceIds'].index('table-axial') + 1, 'table-tilt')
    fingerprint = dict(binary=digest(directory / 'main.hl'), inputs=digest(directory / 'inputs.json'),
                       native=native_hash, settings=settings)
    prior = load(directory / 'execution.json')
    if prior is not None and prior != fingerprint:
        raise ValueError('execution changed; use a fresh output directory')
    save(directory / 'execution.json', fingerprint)
    save(directory / 'settings.json', settings)
    (directory / 'all-exit-status.txt').unlink(missing_ok=True)
    cases = {}
    refreshed = ['combined', 'table-tilt', 'table-axial', 'wall']
    # Only the wall support and the added table tilt change physical source input.
    # HB-D1's analytic work-frame geometry is invariant; all remaining source
    # functions, amplitudes, force/sync histories, grids and native code agree.
    for label in ['baseline', *settings['sourceIds']]:
        if label in refreshed:
            continue
        raw = budget.validate(load(previous / f'{label}.json'))
        if budget.FACE_WARNING in raw['warnings']:
            raise ValueError(f'{label} has no physical end coverage')
        raw = dict(raw, reusedEvidence=dict(path=str(previous / f'{label}.json'),
                   sha256=digest(previous / f'{label}.json'),
                   basis='unchanged independent source and analytic work-frame geometry; wall and combined rerun; table tilt added separately'))
        save(directory / f'{label}.json', raw)
        cases[label] = raw
    env = os.environ.copy()
    env.update(HOBBER_BUDGET_THREADS='4', STOCKKIT_RAKE_BATCH_DOMAINS='1')
    env.pop('HOBBER_BUDGET_LEGACY_ENDS', None)
    env['LD_LIBRARY_PATH'] = ':'.join(map(str, [native, root / '../machine-tending/app/build/host/native/cadkit',
        Path('/home/joao/dev/materia-cache/occt-f1dc4efb-release/lib'), root / 'haxeon/.tools/hashlink', root / 'haxeon/out']))
    stopping, children, mutex = threading.Event(), set(), threading.Lock()

    def stop(signum, frame):
        stopping.set()
        with mutex:
            for child in children:
                child.terminate()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    def completed_case(label):
        observed = load(directory / f'{label}-settings.json')
        for key in ['sourceIds', 'threads', 'sweepToleranceMm', 'pathToleranceMm', 'radialStepMm', 'axialStepMm', 'supportStiffnessNPerMm']:
            if observed[key] != settings[key]:
                raise ValueError(f'{label}: unexpected setting {key}')
        if 'HOBBING_ERROR_RESULT ' not in (directory / f'{label}.log').read_text():
            raise ValueError(f'{label}: missing completed-cut log')
        return label, budget.validate(load(directory / f'{label}.json'))

    def run(label):
        execution_path = directory / f'{label}-execution.json'
        record = load(execution_path)
        if record and record.get('exitCode') == 0 and record.get('fingerprint') == digest(directory / 'execution.json'):
            return completed_case(label)
        # Save interrupted/invalid evidence before retrying; no partial cut is a pass.
        stamp = time.time_ns()
        for suffix in ['.log', '.json', '-execution.json']:
            path = directory / f'{label}{suffix}'
            if path.exists():
                path.rename(directory / f'{label}-prior-{stamp}{suffix}')
        while True:
            if stopping.is_set():
                raise RuntimeError('scheduler stopped')
            available = int(next(row.split()[1] for row in Path('/proc/meminfo').read_text().splitlines()
                                 if row.startswith('MemAvailable:'))) * 1024
            if available > 4 * 1024**3:
                break
            stopping.wait(5)
        began = time.monotonic()
        with (directory / f'{label}.log').open('w') as log:
            with mutex:
                if stopping.is_set():
                    raise RuntimeError('scheduler stopped')
                child = subprocess.Popen([str(root / 'haxeon/.tools/hashlink/hl'), str(directory / 'main.hl'),
                    'reference-budget', str(directory / 'inputs.json'), str(directory), label], cwd=root,
                    env=env, stdout=log, stderr=subprocess.STDOUT)
                children.add(child)
            print(f'STARTED {label} pid={child.pid}', flush=True)
            status = child.wait()
            with mutex:
                children.remove(child)
        save(execution_path, dict(exitCode=status, seconds=time.monotonic()-began,
                                 fingerprint=digest(directory / 'execution.json')))
        if status:
            raise RuntimeError(f'{label} exited {status}')
        return completed_case(label)

    failures = []
    save(directory / 'progress.json', dict(completed=list(cases), pending=refreshed, expected=16))
    with ThreadPoolExecutor(max_workers=2) as pool:
        for future in as_completed([pool.submit(run, label) for label in refreshed]):
            try:
                label, raw = future.result()
                cases[label] = raw
                print(f'COMPLETED {label}', flush=True)
            except Exception as error:
                failures.append(str(error))
                print(f'FAILED {error}', flush=True)
            save(directory / 'progress.json', dict(completed=list(cases), pending=[k for k in refreshed if k not in cases], failures=failures, expected=16))
    if failures:
        save(directory / 'failures.json', failures)
        (directory / 'all-exit-status.txt').write_text('1\n')
        raise RuntimeError('; '.join(failures))
    result = budget.assemble(cases, settings['sourceIds'])
    result['scope'] = settings['scope']
    result['engineeringAcceptance'] = 'HB4 convergence and loaded-machine qualification remain open; ISO module extrapolation; assumed stiffness/contact and plant'
    save(directory / 'result.json', result)
    (directory / 'all-exit-status.txt').write_text('0\n')
    print(result['acceptance'], flush=True)


if __name__ == '__main__':
    main()
