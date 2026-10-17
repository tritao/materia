#!/usr/bin/env python3
"""Prepare fixed HB10 force history and explicitly assumed HB7 servo observations."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force-directory', required=True)
    parser.add_argument('--output-directory', required=True)
    parser.add_argument('--sync-exporter', required=True)
    args = parser.parse_args()
    force_directory = Path(args.force_directory)
    output = Path(args.output_directory)
    output.mkdir(parents=True, exist_ok=True)
    force = json.loads((force_directory / 'loads.json').read_text())
    if not (force['referenceForceAccepted'] and force['forceToleranceValidated']):
        raise ValueError('Accepted full-reference HB10 loads required')
    # These are the installed reference B encoder counts. Kept in the manifest
    # and checked against the assembly by the Haxe consumer.
    master_counts = 1048576
    rpm = force['tableRpm'] * force['teeth']
    subprocess.run([args.sync_exporter, str(output / 'sync.csv'), str(rpm),
                    str(force['seconds']), str(master_counts), str(force['teeth'])], check=True)
    with (output / 'sync.csv').open() as source:
        sync = [[float(row[k]) for k in ['time_s', 'predicted_error_rad', 'no_prediction_error_rad']]
                for row in csv.DictReader(source)]
    predicted = max(abs(row[1]) for row in sync)
    disabled = max(abs(row[2]) for row in sync)
    if predicted > math.pi / 648000 or abs(disabled - rpm*2*math.pi/60/force['teeth']*.001) > 1e-6:
        raise ValueError('HB7 servo observation sanity check failed')
    events = {}
    with (force_directory / 'passes.csv').open() as source:
        for row in csv.DictReader(source):
            for key, sign in [('start_s', 1), ('end_s', -1)]:
                t = float(row[key])
                events[t] = events.get(t, 0.0) + sign*float(row['radial_N'])
    # Replace each discontinuous jump by a centred linear ramp. Its equal
    # signed areas before/after the event preserve the load-time integral.
    # A ramp occupies at most 10% of either neighbouring interval. Coalesce
    # sub-nanosecond event roundoff before constructing strictly ordered knots.
    grouped = []
    for t in sorted(events):
        if grouped and t-grouped[-1][0] < 1e-10:
            grouped[-1][1] += events[t]
        else:
            grouped.append([t, events[t]])
    radial = [[0.0, max(0.0, grouped[0][1])]]
    running = grouped[0][1]
    for i in range(1, len(grouped)-1):
        t, delta = grouped[i]
        previous = max(0.0, running)
        running += delta
        if not 0 < t < force['seconds']:
            continue
        epsilon = .1*min(t-grouped[i-1][0], grouped[i+1][0]-t)
        radial.extend([[t-epsilon, previous], [t+epsilon, max(0.0, running)]])
    radial.append([force['seconds'], 0.0])
    if any(a[0] >= b[0] for a, b in zip(radial, radial[1:])):
        raise ValueError('Force ramp knots must strictly increase')
    interpolated_mean = sum((b[0]-a[0])*(a[1]+b[1])/2 for a, b in zip(radial, radial[1:]))/force['seconds']
    mean_error = abs(interpolated_mean/force['meanRadialN']-1)
    peak_error = abs(max(row[1] for row in radial)/force['radialN']-1)
    if max(mean_error, peak_error) > .001:
        raise ValueError('Continuous force interpolation changes mean or peak by more than 0.1%')
    payload = dict(force=force, radial=radial, sync=sync, masterCounts=master_counts,
                   forceInterpolation='centred linear ramps at radial window jumps, at most 10% of neighbouring interval; event roundoff coalesced below 1e-10 s; integral and peak checked within 0.1%; frozen baseline loads, no force/compliance feedback',
                   interpolatedMeanRadialN=interpolated_mean,interpolationMeanRelativeError=mean_error,interpolationPeakRelativeError=peak_error,
                   syncBasis='HB7 native ElectronicGear at 4 kHz, installed 20-bit B encoder, assumed 1 ms transport latency; assumed 200 Hz critically damped ideal plant, one-second warm-up; independently regenerated prediction enabled/disabled; not loaded full-machine observations',
                   syncPeakArcsec=predicted*648000/math.pi,
                   noPredictionPeakArcsec=disabled*648000/math.pi)
    (output / 'inputs.json').write_text(json.dumps(payload, separators=(',', ':'))+'\n')
    print(json.dumps({key: payload[key] for key in ['syncBasis', 'syncPeakArcsec', 'noPredictionPeakArcsec', 'forceInterpolation']}))


if __name__ == '__main__':
    main()
