# Rack-to-table worker demo

The app and the headless app acceptance test run the same MuJoCo session: a worker fetches the part from rack slot B3, carries it along the facility lane to the assembly table, and places it while a robot arm cycles nearby. `app/tests/src/tests/WorkerDemoTests.hx` asserts that the part settles within 1 cm of its table target, the rack and table zones are occupied in sequence, and the robot separation signal varies during the cycle.

Run the command-line scenario from the worktree root after building the app:

```bash
./app/run-built.sh --snapshot --worker-demo=rack-to-table --worker-demo-step=1000
```

The JSON output includes job status, part position, occupied zones, and current separation. `--worker-demo-step` advances a chosen number of fixed ticks before reporting or launching the UI. The captured headless screenshots correspond to tick 300 (at the rack), tick 480 (carrying), and tick 700 (placed). To reproduce one under Xvfb:

```bash
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s '-screen 0 1600x1000x24' \
  ./app/run-built.sh --reset-workspace \
  --capture-dir=humankit/sim/tests/screenshots/capture --frames=12 \
  --worker-demo=rack-to-table --worker-demo-step=480
```

The generated `frame.png` is the screenshot; the capture directory also contains diagnostics. The three checked-in PNGs were captured with this command at their named ticks.
