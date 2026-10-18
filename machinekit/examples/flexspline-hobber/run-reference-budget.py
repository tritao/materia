#!/usr/bin/env python3
"""Restartable, bounded parallel fine cuts. Never alters legacy evidence.

Prepare a frozen main.hl, inputs.json and settings.json in the output directory.
Grid/sweep/path settings remain pinned. Native/bytecode upgrades require
hash-bound exact-parity and paired speed evidence.
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
    parser.add_argument('--upgrade-validation', type=Path, help='exact parity and paired speed evidence for a native-library upgrade')
    parser.add_argument('--adopt-running', type=Path, help='existing native cuts to let finish under their original library')
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
    if available < 3 * 1024**3:
        raise RuntimeError('insufficient available memory for 3 GiB headroom')
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
    proof_path = args.upgrade_validation or directory / 'native-upgrade-validation.json'
    proof = load(proof_path)
    native_hash = digest(args.native / 'libstockkit_core.so')
    binary = Path(proof['newBinaryPath']).resolve() if proof else directory / 'main.hl'
    if proof is not None:
        if (proof.get('method') != 'paired streaming domains' or proof.get('newBinarySha256') != digest(binary)
                or proof.get('oldBinarySha256') != digest(directory / 'main.hl')
                or proof.get('newNativeSha256') != native_hash or proof.get('passed') is not True
                or proof.get('referenceStockHashIdentical') is not True
                or proof.get('pairedExactEndpointsAndProvenance') is not True
                or proof.get('nativeTestsPassed') is not True or proof.get('speedup', 0) < 1.1):
            raise ValueError('native upgrade lacks exact parity and paired speed validation')
    worker_memory = 900 * 1024**2
    legacy_native = proof['oldNativeSha256'] if proof else native_hash
    if legacy_native not in (legacy / 'provenance.txt').read_text():
        raise ValueError('legacy native implementation differs')
    if digest(directory / 'inputs.json') != digest(legacy / 'inputs.json'):
        raise ValueError('legacy inputs differ')
    if settings['threads'] != args.threads or settings.get('workers') != args.workers:
        raise ValueError('execution worker/thread settings differ from saved settings')
    fingerprint = dict(binary=digest(binary), inputs=digest(directory / 'inputs.json'),
                       native=digest(args.native / 'libstockkit_core.so'), threads=args.threads,
                       settings=settings, schema=1)
    if proof:
        fingerprint.update(sharedGeometry=True, batchDomains=True, upgradeValidationSha256=digest(proof_path))
    adoption_path = args.adopt_running or directory / 'adopt-running.json'
    adopted = load(adoption_path) or {}
    for mode, row in adopted.items():
        if row.get('nativeSha256') not in [native_hash, legacy_native]:
            raise ValueError(f'{mode}: unknown native library in adoption manifest')
    existing = load(directory / 'execution.json')
    if existing is not None and existing != fingerprint:
        before = {k: v for k, v in existing.items() if k not in ['native', 'binary', 'sharedGeometry', 'batchDomains', 'upgradeValidationSha256']}
        after = {k: v for k, v in fingerprint.items() if k not in ['native', 'binary', 'sharedGeometry', 'batchDomains', 'upgradeValidationSha256']}
        if not proof or before != after or existing['native'] != proof['oldNativeSha256'] or existing['binary'] != proof['oldBinarySha256']:
            raise ValueError('frozen inputs/binary/settings changed; use a separate evidence directory')
        save(directory / ('execution-before-' + existing['native'][:12] + '.json'), existing)
    save(directory / 'execution.json', fingerprint)
    (directory / 'all-exit-status.txt').unlink(missing_ok=True)
    ids = settings['sourceIds']
    env = os.environ.copy()
    env['HOBBER_BUDGET_THREADS'] = str(args.threads)
    env.pop('HOBBER_BUDGET_LEGACY_ENDS', None)
    env['STOCKKIT_RAKE_BATCH_DOMAINS'] = '1' if proof else '0'
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
        if mode in adopted:
            row = adopted[mode]
            pid = int(row['pid'])
            print(f'ADOPTING {mode} pid={pid}', flush=True)
            while not stopping.is_set():
                try:
                    command = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
                except FileNotFoundError:
                    break
                if len(command) < 2 or not any(path in command for path in [str(directory / 'main.hl').encode(),str(binary).encode()]) or mode.encode() != command[-2]:
                    break  # Exited/reaped, or PID reused: never signal it.
                time.sleep(5)
            if stopping.is_set():
                raise RuntimeError('scheduler stopped; adopted native cut left running')
            validate(load(directory / f'{mode}.json'), not mode.startswith('lead-'))
            results = [json.loads(line.split(' ', 1)[1]) for line in (directory / f'{mode}.log').read_text().splitlines()
                       if line.startswith('HOBBING_ERROR_RESULT ')]
            if not results:
                raise RuntimeError(f'{mode}: adopted process has no completed-cut log')
            prior = load(directory / f'{mode}-execution.json') or {}
            save(directory / f'{mode}-execution.json', dict(prior, exitCode=prior.get('exitCode'), completedCut=True, adopted=True,
                 wallSeconds=results[-1]['wallSeconds'], threads=results[-1]['threads'], nativeSha256=row['nativeSha256'],
                 sharedGeometry=row.get('sharedGeometry', False), acceptanceBasis='complete physical measurement and successful cut log; old launcher replaced, OS exit status unavailable'))
            print(f'ADOPTED_COMPLETED {mode}', flush=True)
            return
        started = time.monotonic()
        with (directory / f'{mode}.log').open('w') as log:
            while True:
                with child_lock:
                    if stopping.is_set():
                        raise RuntimeError('scheduler stopped')
                    available = int(next(line.split()[1] for line in Path('/proc/meminfo').read_text().splitlines() if line.startswith('MemAvailable:'))) * 1024
                    pending = 0
                    for running in children:
                        try:
                            rss = int(Path(f'/proc/{running.pid}/statm').read_text().split()[1]) * os.sysconf('SC_PAGE_SIZE')
                        except FileNotFoundError:
                            rss = 0
                        pending += max(0, worker_memory - rss)
                    if available >= pending + worker_memory + 3 * 1024**3:
                        child = subprocess.Popen([str(root / 'haxeon/.tools/hashlink/hl'), str(binary),
                            'reference-budget', str(directory / 'inputs.json'), str(directory), mode], cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
                        children.add(child)
                        break
                stopping.wait(5)
            returncode = child.wait()
            with child_lock:
                children.remove(child)
        save(directory / f'{mode}-execution.json', dict(exitCode=returncode, wallSeconds=time.monotonic()-started,
                                                       threads=args.threads, nativeSha256=native_hash, sharedGeometry=bool(proof), executionFingerprint=digest(directory / 'execution.json')))
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
                    if len(command) >= 2 and str(legacy / 'main.hl').encode() in command and command[-2] == b'all':
                        os.kill(args.legacy_pid, signal.SIGTERM)
                        save(legacy / 'replacement.json', dict(status='deliberately stopped after table-radial', pid=args.legacy_pid, replacement=str(directory)))
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
                               settingsUnchanged=True, sharedGeometryForNewCuts=bool(proof),
                               provenance='execution.json and per-case execution/settings files; adopted cuts retain their original native hash')
    save(directory / 'result.json', result)
    (directory / 'all-exit-status.txt').write_text('0\n')
    print('ALL_COMPLETED', flush=True)


if __name__ == '__main__':
    main()
