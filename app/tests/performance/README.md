# Editor performance gate

From the repository root, run:

```sh
python3 app/tools/check-editor-performance.py
```

The command builds the current app and headless benchmark, runs three 500-cycle
Hierarchy/Sensors interaction captures without a display server, and measures
30 seconds of desktop idle time. It needs `DISPLAY` or `WAYLAND_DISPLAY` for
the idle phase. In headless CI, use `--headless-only` to run the interaction
and retention checks. Use `--skip-build` only when the app and benchmark
binaries already contain the source being tested.

Each interaction run verifies the inspector rename, frame p95, RSS growth
after the first 100 cycles, and the retained listener, widget resource, style,
state, and key path counts. The RSS check allows up to 32 MiB total growth
after warmup and rejects two consecutive 100-cycle windows above 8 MiB each.
The frame p95 limit is 30 ms. Desktop idle checks ignore the first five
seconds and reject consecutive five-second windows with over one CPU second
or 16 MiB RSS growth. These limits were set against three 500-cycle captures
and several desktop idle captures on 2026-09-23; the interaction captures
showed 11.7–12.2 ms p95 and 8.1–8.9 MiB RSS growth after warmup.

The gate writes all raw captures under `app/build/profiles/gate-*`. To inspect
existing captures without launching the app, pass one or more `--capture`
paths and optionally `--idle-capture` with the matching `--idle-seconds`.
The JSON summaries show the measured values and any failed checks.

For a heap snapshot, run `python3 app/tools/profile-editor.py --scenario
tab-inspector --cycles 100 --heap-dump`. The capture contains `heap.dump` and
the exact `headless-profile.hl` bytecode used for that run. Inspect the pair
with `python3 haxeon/scripts/inspect-hl-heap.py
<capture>/headless-profile.hl <capture>/heap.dump`; the shared tool writes
`heap-report.txt` beside the dump.
