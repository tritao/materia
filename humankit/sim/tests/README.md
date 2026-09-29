# Rack-to-table worker demo

The app and the headless app acceptance test run the same MuJoCo session: a worker fetches the part from rack slot B3, carries it through a waypoint and a turn for more than 2.5 m to the assembly table, and releases it directly from the hand while the example document's robot motion track cycles a jointed arm nearby. `app/tests/src/tests/WorkerDemoTests.hx` asserts that the part settles within 2 cm of its table target, checks for jumps during delivery near the table and confirms that this check ran, verifies the worker finishes outside the rack, and checks that robot link pose and separation vary during the cycle. There is no target correction after release.

Run the command-line scenario from the worktree root after building the app:

```bash
./app/run-built.sh --snapshot --worker-demo=rack-to-table --worker-demo-step=1000
```

The JSON output includes job status, part position, occupied zones, and current separation. Separation approximates the robot link's collision sphere against each worker capsule. `--worker-demo-step` advances a chosen number of fixed ticks before reporting or launching the UI. To capture a frame under Xvfb:

```bash
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s '-screen 0 1600x1000x24' \
  ./app/run-built.sh --reset-workspace \
  --capture-dir=humankit/sim/tests/build/screenshots/capture --frames=12 \
  --worker-demo=rack-to-table --worker-demo-step=480
```

The generated `frame.png` is the screenshot; the capture directory also contains diagnostics. Captures are build output and are not committed.
