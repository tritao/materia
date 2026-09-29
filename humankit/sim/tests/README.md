# Rack-to-table worker demo

The app and the headless app acceptance test run the same MuJoCo session: a worker fetches the part from rack slot B3, carries it along the facility lane to the assembly table, and releases it directly from the hand while a jointed robot arm cycles nearby. `app/tests/src/tests/WorkerDemoTests.hx` asserts that the part settles within 25 cm of its table target, does not jump during delivery near the table, the rack and table zones are occupied in sequence, and the robot link pose and separation signal vary during the cycle. The placement tolerance includes wrist IK error and grasp offset; there is no target correction after release.

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
