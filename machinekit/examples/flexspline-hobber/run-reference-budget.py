#!/usr/bin/env python3
"""Restartable, bounded parallel fine cuts. Never alters legacy evidence.

Prepare a frozen main.hl, inputs.json and settings.json in the output directory.
All jobs retain the accepted grid/sweep/path settings; only thread count changes.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import copy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import threading

PARAMETERS = ['fp', 'Fp', 'Fr', 'Falpha', 'ffalpha', 'fHalpha', 'Fbeta', 'ffbeta', 'fHbeta']
LEAD = ['Fbeta', 'ffbeta', 'fHbeta']
FACE_WARNING = 'Face endpoints not sampled; end excess material unchecked'


def load(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def save(path, data):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def validate(raw, full=True):
    keys = PARAMETERS if full else ['fp', 'Fp', 'Fr', *LEAD]
    if raw is None or any(not isinstance(raw.get('values', {}).get(k), (float, int))
                          or not math.isfinite(raw['values'][k]) for k in keys):
        raise ValueError('incomplete/non-finite inspection')
    if full and any(not raw.get(k) for k in ['profile', 'lead', 'pitch', 'runoutMm']):
        raise ValueError('missing budget traces')
    return raw


def promote(original, lead):
    """Keep original physical profile; replace lead only after exact datum parity."""
    validate(original)
    validate(lead, False)
    if FACE_WARNING in lead['warnings']:
        raise ValueError('lead supplement does not cover both physical face ends')
    for key in ['fp', 'Fp', 'Fr']:
        if original['values'][key] != lead['values'][key]:
            raise ValueError(f'lead supplement changes interior {key}')
    for key in ['pitch', 'runoutMm', 'thicknessMm']:
        if original[key] != lead[key]:
            raise ValueError(f'lead supplement changes interior {key}')
    out = copy.deepcopy(original)
    for key in LEAD:
        out['values'][key], out['grades'][key] = lead['values'][key], lead['grades'][key]
    out['lead'], out['helixSourcePoints'] = lead['lead'], lead['helixSourcePoints']
    out['warnings'] = [w for w in original['warnings'] if w != FACE_WARNING]
    grades = list(out['grades'].values())
    out['overallGrade'] = max(grades) if all(g is not None for g in grades) else None
    out['coverageBasis'] = 'Original fine profile plus physical end-inclusive lead; interior fp/Fp/Fr, pitch, runout and thickness exactly equal; originals retained separately'
    return out


def trace(raw, channel):
    if channel in ['profile', 'lead', 'pitch']:
        return [v for row in raw[channel] for v in row]
    return [v * 1000 for v in raw['runoutMm']]


def assemble(cases, ids):
    """Same scalar deltas and pointwise criterion as gearkit.ErrorBudget."""
    baseline, combined = cases['baseline'], cases['combined']
    for raw in cases.values():
        validate(raw)
        if FACE_WARNING in raw['warnings']:
            raise ValueError('cannot assemble an end-incomplete inspection')
    dominant = {}
    for key in baseline['values']:
        largest, selected = 1e-9, 'none'
        for label in ids:
            delta = abs(cases[label]['values'][key] - baseline['values'][key])
            if delta > largest:
                largest, selected = delta, label
        dominant[key] = selected
    interactions, warnings = {}, []
    for channel in ['profile', 'lead', 'pitch', 'runout']:
        b, all_values = trace(baseline, channel), trace(combined, channel)
        individual = [trace(cases[label], channel) for label in ids]
        if not b or any(len(row) != len(b) for row in [all_values, *individual]):
            raise ValueError(f'inconsistent budget sampling: {channel}')
        residual, scale = 0., 0.
        for i, base in enumerate(b):
            deltas = [row[i] - base for row in individual]
            if not all(math.isfinite(v) for v in [base, all_values[i], *deltas]):
                raise ValueError('non-finite budget trace')
            residual = max(residual, abs(all_values[i] - base - sum(deltas)))
            scale = max(scale, sum(abs(v) for v in deltas))
        interactions[channel] = residual
        if residual > .1 + .25 * scale:
            warnings.append(f'{channel} interaction exceeds stated trace-superposition tolerance')
    additive = not warnings
    for raw in [baseline, combined]:
        for warning in raw['warnings'] + raw['extrapolation']:
            if warning not in warnings:
                warnings.append(warning)
    failures = [key for key in PARAMETERS if combined['grades'].get(key) is None or combined['grades'][key] > 6]
    return dict(additive=additive, dominant=dominant, interactionUm=interactions, warnings=warnings,
                failedClass6Parameters=failures, combined=combined,
                acceptance='extrapolated class-6 limits fit; review qualifications' if not failures else 'open: class-6 limits exceeded')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--legacy-directory', required=True, type=Path)
    parser.add_argument('--face-directory', required=True, type=Path)
    parser.add_argument('--native', required=True, type=Path)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--legacy-pid', type=int, help='stop this old all-case process only AFTER table-radial.json is complete')
    args = parser.parse_args()
    if not 1 <= args.workers <= 4 or not 1 <= args.threads <= 4:
        parser.error('bounded to 1..4 workers, 1..4 threads per cut')
    root = Path(__file__).resolve().parents[3]
    directory, legacy, face = (p.resolve() for p in [args.directory, args.legacy_directory, args.face_directory])
    directory.mkdir(parents=True, exist_ok=True)
    lock = (directory / 'parallel.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    available = int(next(line.split()[1] for line in Path('/proc/meminfo').read_text().splitlines() if line.startswith('MemAvailable:'))) * 1024
    if available < args.workers * 900 * 1024**2 + 3 * 1024**3:
        raise RuntimeError('insufficient available memory for workers plus 3 GiB headroom')
    settings = load(directory / 'settings.json')
    if settings is None:
        raise ValueError('prepare settings.json first')
    for key, expected in [('sweepToleranceMm', .000025), ('pathToleranceMm', .0000015625), ('radialStepMm', .0015), ('axialStepMm', .04)]:
        if settings[key] != expected:
            raise ValueError(f'unvalidated setting {key}')
    old_settings = load(legacy / 'settings.json')
    for key in ['moduleMm', 'teeth', 'faceWidthMm', 'sweepToleranceMm', 'pathToleranceMm', 'radialStepMm', 'axialStepMm', 'windows', 'sourceIds']:
        if settings[key] != old_settings[key]:
            raise ValueError(f'legacy settings differ: {key}')
    if digest(args.native / 'libstockkit_core.so') not in (legacy / 'provenance.txt').read_text():
        raise ValueError('legacy native implementation differs')
    if digest(directory / 'inputs.json') != digest(legacy / 'inputs.json'):
        raise ValueError('legacy inputs differ')
    if settings['threads'] != args.threads or settings.get('workers') != args.workers:
        raise ValueError('execution worker/thread settings differ from saved settings')
    fingerprint = dict(binary=digest(directory / 'main.hl'), inputs=digest(directory / 'inputs.json'),
                       native=digest(args.native / 'libstockkit_core.so'), threads=args.threads,
                       settings=settings, schema=1)
    existing = load(directory / 'execution.json')
    if existing is not None and existing != fingerprint:
        raise ValueError('frozen inputs/binary/settings changed; use a separate evidence directory')
    save(directory / 'execution.json', fingerprint)
    (directory / 'all-exit-status.txt').unlink(missing_ok=True)
    ids = settings['sourceIds']
    env = os.environ.copy()
    env['HOBBER_BUDGET_THREADS'] = str(args.threads)
    env.pop('HOBBER_BUDGET_LEGACY_ENDS', None)
    env['LD_LIBRARY_PATH'] = ':'.join(map(str, [args.native.resolve(), root / '../machine-tending/app/build/host/native/cadkit',
        Path('/home/joao/dev/materia-cache/occt-f1dc4efb-release/lib'), root / 'haxeon/.tools/hashlink', root / 'haxeon/out']))

    stopping, children, child_lock = threading.Event(), set(), threading.Lock()

    def stop_children(signum, frame):
        stopping.set()
        with child_lock:
            for child in list(children):
                child.terminate()

    signal.signal(signal.SIGTERM, stop_children)
    signal.signal(signal.SIGINT, stop_children)

    def cut(mode):
        started = time.monotonic()
        with (directory / f'{mode}.log').open('w') as log:
            child = subprocess.Popen([str(root / 'haxeon/.tools/hashlink/hl'), str(directory / 'main.hl'),
                'reference-budget', str(directory / 'inputs.json'), str(directory), mode], cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
            with child_lock:
                children.add(child)
                if stopping.is_set():
                    child.terminate()
            returncode = child.wait()
            with child_lock:
                children.remove(child)
        save(directory / f'{mode}-execution.json', dict(exitCode=returncode, wallSeconds=time.monotonic()-started,
                                                       threads=args.threads, executionFingerprint=digest(directory / 'execution.json')))
        if returncode:
            raise RuntimeError(f'{mode} exited {returncode}; see its log')

    def wait_json(path):
        while True:
            if stopping.is_set():
                raise RuntimeError('scheduler stopped')
            raw = load(path)
            if raw is not None:
                return raw
            for status_name in ['all-exit-status.txt', 'lead-ends-exit-status.txt']:
                status = path.parent / status_name
                if status.exists() and int(status.read_text().strip()) != 0:
                    raise RuntimeError(f'required evidence unavailable after failed/stopped run: {path}')
            # This is a background scheduler, not an interactive tool wait.
            time.sleep(5)

    def run(label):
        if stopping.is_set():
            raise RuntimeError('scheduler stopped')
        target = directory / f'{label}.json'
        saved = load(target)
        if saved is not None:
            validate(saved)
            if FACE_WARNING in saved['warnings']:
                raise ValueError(f'{label}: saved result lacks face ends')
            print(f'REUSED {label}', flush=True)
            return label, saved
        if label in ['baseline', 'mandrel', 'table-radial']:
            original = wait_json(legacy / f'{label}.json')
            if label == 'table-radial' and args.legacy_pid:
                # Verify identity before signalling; PID reuse must not kill unrelated work.
                try:
                    command = Path(f'/proc/{args.legacy_pid}/cmdline').read_bytes().split(b'\0')
                    if str(legacy / 'main.hl').encode() in command and command[-2] == b'all':
                        os.kill(args.legacy_pid, signal.SIGTERM)
                except (ProcessLookupError, FileNotFoundError):
                    pass
            if label == 'baseline':
                lead = wait_json(face / 'face-end-baseline.json')
                basis = str(face / 'face-end-baseline.json')
            else:
                lead_path = directory / f'lead-{label}.json'
                if load(lead_path) is None:
                    cut(f'lead-{label}')
                lead = load(lead_path)
                basis = str(lead_path)
            measured = promote(original, lead)
            measured['reusedEvidence'] = dict(profile=str(legacy / f'{label}.json'), profileSha256=digest(legacy / f'{label}.json'),
                                             lead=basis, leadSha256=digest(Path(basis)), originalThreads=8, leadThreads=8 if label == 'baseline' else args.threads)
            save(target, measured)
        else:
            cut(label)
            measured = validate(load(target))
        print(f'COMPLETED {label}', flush=True)
        return label, measured

    cases, errors = {}, []
    # Reuse baseline and let the currently engaged table case finish. Other
    # independent cases occupy the remaining slots; no serial full-cut restart.
    labels = ['table-radial', 'mandrel', 'combined', *[k for k in ids if k not in ['mandrel', 'table-radial']]]
    with ThreadPoolExecutor(max_workers=args.workers) as executor, ThreadPoolExecutor(max_workers=1) as promotion:
        futures = [executor.submit(run, label) for label in labels] + [promotion.submit(run, "baseline")]
        for future in as_completed(futures):
            try:
                label, measured = future.result()
            except Exception as error:
                errors.append(str(error))
                save(directory / 'failures.json', errors)
                print(f'FAILED {error}', flush=True)
                continue
            cases[label] = measured
            save(directory / 'progress.json', dict(completed=list(cases), expected=len(labels)+1, workers=args.workers, threadsPerCut=args.threads))
    if errors:
        (directory / 'all-exit-status.txt').write_text('1\n')
        raise RuntimeError('; '.join(errors))
    try:
        result = assemble(cases, ids)
    except Exception:
        (directory / 'all-exit-status.txt').write_text('1\n')
        raise
    result['scope'] = settings['scope']
    result['execution'] = dict(workers=args.workers, threadsPerCut=args.threads, faceEndpoints=True,
                               settingsUnchanged=True, provenance='execution.json and per-case execution/settings files')
    save(directory / 'result.json', result)
    (directory / 'all-exit-status.txt').write_text('0\n')
    print('ALL_COMPLETED', flush=True)


if __name__ == '__main__':
    main()
