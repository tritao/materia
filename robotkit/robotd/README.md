# robotd

`robotd` is Materia's headless robotics application. It owns the Haxeon
robot/document model, compilation and orchestration, while hosting RobotKit
runtime/control and endpoint adapters behind the versioned protocol exposed to
editor clients.

The executable builds a small Haxeon robot document into a bulk RobotKit
runtime blueprint. By default it uses the SimKit-backed endpoint; `--serial`
selects the POSIX serial endpoint while preserving the same runtime and
NativeKit TCP process boundary. The server owns one deployed runtime, a unique
session for each connection, one controller lease, and any number of read-only
observers; it does not become a multi-robot world:

Run the current skeleton with:

```sh
../../haxeon/scripts/haxeon run --project haxeon.json

# Start the authoritative robot process
../../haxeon/scripts/haxeon run --project haxeon.json -- --server

# Host the compiled robot model through a serial device
../../haxeon/scripts/haxeon run --project haxeon.json -- \
  --server --serial=/dev/serial/by-id/robot-controller --baud=115200

# Optionally host a Haxeon behavior inside robotd
../../haxeon/scripts/haxeon run --project haxeon.json -- \
  --server --behavior=oscillate

# Native-only runtime smoke test
../../haxeon/scripts/haxeon run --project haxeon.json -- --in-memory
```

The protocol and world TCP clients are integration tests rather than robotd
runtime modes. Run them through `../tests/world-tcp.sh`.

Run the model-driven `MobileBase`, `GoTo`, sensor, and recording parity checks
against a pseudo-terminal device emulator to exercise the serial server path
without motor hardware:

```sh
python3 ../tests/serial-tcp.py
```

Before connecting actual hardware, implement the device side of
[`../runtime/SERIAL_PROTOCOL.md`](../runtime/SERIAL_PROTOCOL.md). The device
must enforce a local actuator watchdog; a disconnected cable cannot receive a
host emergency-stop frame.

The same runtime command/controller boundary is used by simulation and by the
first native physical endpoint (`SerialRobotEndpoint`). Sensor state is carried
as `SensorFrameMsg` values with sensor identity, frame ID, sequence, source
timestamp, and receive timestamp.
