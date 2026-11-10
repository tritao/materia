#!/usr/bin/env python3
"""Evaluate representative bands from a completed full history; never skip stock preparation."""
import argparse
import csv
import json
from pathlib import Path


def stats(events, start, end):
    force = 0.0
    time = start
    integral = 0.0
    peak = 0.0
    for epoch, delta in events:
        if epoch <= start:
            force += delta
            continue
        if epoch >= end:
            break
        integral += force * (epoch - time)
        peak = max(peak, force)
        time, force = epoch, force + delta
    integral += force * (end - time)
    return {'meanRadialN': integral / (end - start), 'peakRadialN': max(peak, force),
            'durationSeconds': end - start}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    report = json.loads(Path(args.report).read_text())
    deltas = {}
    with Path(report['historyCsv']).open() as source:
        for row in csv.DictReader(source):
            radial = float(row['radial_N'])
            for time, delta in [(float(row['start_s']), radial), (float(row['end_s']), -radial)]:
                deltas[time] = deltas.get(time, 0.0) + delta
    events = sorted(deltas.items())
    speed = report['feedMmPerWorkRevolution'] * report['tableRpm'] / 60
    entry_end = -report['zStartMm'] / speed
    exit_start = (report['faceWidthMm'] - report['zStartMm']) / speed
    duration = report['seconds']
    middle_duration = min(2 * 60 / report['tableRpm'], exit_start - entry_end)
    center = (entry_end + exit_start) / 2
    bands = {'entry': stats(events, 0, entry_end),
             'middleTwoTurns': stats(events, center - middle_duration / 2, center + middle_duration / 2),
             'exit': stats(events, exit_start, duration)}
    weighted_mean = (bands['entry']['meanRadialN'] * entry_end
                     + bands['middleTwoTurns']['meanRadialN'] * (exit_start - entry_end)
                     + bands['exit']['meanRadialN'] * (duration - exit_start)) / duration
    peak = max(band['peakRadialN'] for band in bands.values())
    mean_change = abs(weighted_mean / report['meanRadialN'] - 1)
    peak_change = abs(peak / report['radialN'] - 1)
    result = {'bands': bands, 'weightedMeanRadialN': weighted_mean, 'peakRadialN': peak,
              'relativeMeanChange': mean_change, 'relativePeakChange': peak_change,
              'passesOnePercent': mean_change < 0.01 and peak_change < 0.01,
              'scope': 'Offline statistics from fully machined history. This does not validate starting each band from virgin stock or reusing a seed across changed error sources; no shortened physical sweep is enabled.'}
    Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
