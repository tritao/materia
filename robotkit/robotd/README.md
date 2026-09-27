# robotd

`robotd` is Materia's headless robotics application. It owns the Haxeon
robot/document model, compilation and orchestration, while hosting RobotKit
runtime/control and endpoint adapters behind the versioned protocol exposed to
editor clients.

The executable builds a small Haxeon robot document into a bulk RobotKit
runtime blueprint. By default it uses the SimKit-backed endpoint; `--deployment`
selects the POSIX serial endpoint while preserving the same runtime and
NativeKit TCP process boundary. The server owns one deployed runtime, a unique
session for each connection, one controller lease, and any number of read-only
observers; it does not become a multi-robot world:

`robotd` has one control owner. Without `--behavior`, the first valid remote
controller takes ownership. With `--behavior`, local behavior owns control and
remote connections are observers. Controller disconnect submits an emergency
stop and clears ownership; a new controller must explicitly reset safety.
Client sequence numbers are checked per session, while `robotd` assigns a
separate 64-bit sequence to the runtime command stream.

Run the current skeleton with:

```sh
../../haxeon/scripts/haxeon run --project haxeon.json

# Start the authoritative robot process
../../haxeon/scripts/haxeon run --project haxeon.json -- --server

# Bind to a robot LAN interface when the network is isolated
../../haxeon/scripts/haxeon run --project haxeon.json -- \
  --server --listen=192.168.10.20

# Host a deployed robot through a serial device
../../haxeon/scripts/haxeon run --project haxeon.json -- \
  --server --deployment=/etc/robotkit/deployment.json --listen=192.168.10.20

# Optionally host a Haxeon behavior inside robotd
../../haxeon/scripts/haxeon run --project haxeon.json -- \
  --server --behavior=oscillate

# Native-only runtime smoke test
../../haxeon/scripts/haxeon run --project haxeon.json -- --in-memory
```

The TCP listener defaults to `127.0.0.1`. `--listen` accepts an explicit IPv4
address, including `0.0.0.0` to bind all interfaces. The RobotKit TCP protocol
does not authenticate clients; use an isolated robot network or SSH tunnel.

Serial hosting requires a deployment JSON file. It refers to the canonical
semantic robot model and gives the UART path, baud, `f32` target error budget,
and layout fingerprint. Its device section points to a device layout and the
matching schema lock. The layout maps ordered RKD6 actuator channels to joint IDs in the
model. `robotd` loads the complete `RobotModel`, recomputes the fingerprint from
the exact layout bytes and schema lock, and checks the channel order before
opening the UART. `RobotModelCodec` reads the versioned v4 model artifact;
the device fingerprint still covers only the physical layout and wire schema.

See [`../tests/fixtures/device-deployment/deployment.json`](../tests/fixtures/device-deployment/deployment.json)
and its referenced [`robot.json`](../tests/fixtures/device-deployment/robot.json)
for the PTY fixture. Its calibration and fingerprint are test values; a physical
deployment needs measured values. Generate the deployed fingerprint from the
exact layout bytes with `python3 ../tools/device_fingerprint.py layout.json
--schema-lock device_wire6.lock.json --rust firmware_fingerprint.rs`, then put
the printed hex value in `deployment.json` and compile the Rust constant into
the MCU firmware. Include that exact schema lock file in the deployment directory.

The protocol and world TCP clients are integration tests rather than robotd
runtime modes. Run them through `../tests/world-tcp.sh`.

The complete RobotClient → robotd → RKD6 → Rust device path runs without
hardware through `python3 ../tests/device-tcp.py`.

[`../runtime/DEVICE_PROTOCOL.md`](../runtime/DEVICE_PROTOCOL.md) describes the
RKD6 device contract. The Rust scheduled core handles queued segments, path
clock, HOLD, STOP and link-loss stopping. The Nucleo adapter drives two
in-memory position setpoints. A physical board layer still needs motor pins,
feedback and verified stop outputs; a disconnected cable cannot receive a
host emergency-stop frame.

The same runtime command/controller boundary is used by simulation and the
physical endpoint (`DeviceSerialEndpoint`). Sensor data stays outside RKD6
control acknowledgements.
