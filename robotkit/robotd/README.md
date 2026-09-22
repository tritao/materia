# robotd

`robotd` is Materia's headless robotics application. It owns the Haxeon
robot/document model, compilation and orchestration, while hosting RobotKit
runtime/control and endpoint adapters behind the versioned protocol exposed to
editor clients.

The executable builds a small Haxeon robot document into a bulk RobotKit
runtime blueprint and runs it through the SimKit-backed endpoint. The normal
process-boundary path is the NativeKit TCP server/client on loopback. The
server owns one deployed runtime, a unique session for each connection, one
controller lease, and any number of read-only observers; it does not become a
multi-robot world:

Run the current skeleton with:

```sh
../../haxeon/scripts/haxeon run --project haxeon.json

# Start the authoritative robot process
../../haxeon/scripts/haxeon run --project haxeon.json -- --server

# Optionally host a Haxeon behavior inside robotd
../../haxeon/scripts/haxeon run --project haxeon.json -- \
  --server --behavior=oscillate

# Native-only runtime smoke test
../../haxeon/scripts/haxeon run --project haxeon.json -- --in-memory
```

The protocol and world TCP clients are integration tests rather than robotd
runtime modes. Run them through `../tests/world-tcp.sh`.

The same runtime command/controller boundary is used by simulation and by the
first native physical endpoint (`SerialRobotEndpoint`). Sensor state is carried
as `SensorFrameMsg` values with sensor identity, frame ID, sequence, source
timestamp, and receive timestamp.
