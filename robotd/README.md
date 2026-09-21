# robotd

`robotd` is Materia's headless robotics application. It owns the Haxeon
robot/document model, compilation and orchestration, while hosting RobotKit
runtime/control and endpoint adapters behind the versioned protocol exposed to
editor clients.

The executable builds a small Haxeon robot document into a bulk RobotKit
runtime blueprint and runs it through the SimKit-backed endpoint. The normal
process-boundary path is the NativeKit TCP server/client on loopback:

Run the current skeleton with:

```sh
../haxeon/scripts/haxeon run --project haxeon.json

# Start the authoritative robot process
../haxeon/scripts/haxeon run --project haxeon.json -- --server

# Optionally host a Haxeon behavior inside robotd
../haxeon/scripts/haxeon run --project haxeon.json -- \
  --server --behavior=oscillate

# In another terminal, exercise the protocol client
../haxeon/scripts/haxeon run --project haxeon.json -- --client

# Native-only runtime smoke test
../haxeon/scripts/haxeon run --project haxeon.json -- --in-memory
```
