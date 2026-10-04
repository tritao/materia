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
../../haxeon/scripts/haxeon run --project haxeon.json -- --server --auth=/etc/robotkit/authorization.json

# Bind to a robot LAN interface when the network is isolated
../../haxeon/scripts/haxeon run --project haxeon.json -- \
  --server --auth=/etc/robotkit/authorization.json --listen=192.168.10.20

# Host a deployed robot through a serial device
../../haxeon/scripts/haxeon run --project haxeon.json -- \
  --server --auth=/etc/robotkit/authorization.json --deployment=/etc/robotkit/deployment.json --listen=192.168.10.20

# Optionally host a Haxeon behavior inside robotd
../../haxeon/scripts/haxeon run --project haxeon.json -- \
  --server --auth=/etc/robotkit/authorization.json --behavior=oscillate

# Native-only runtime smoke test
../../haxeon/scripts/haxeon run --project haxeon.json -- --in-memory
```

The TCP listener defaults to `127.0.0.1`. `--listen` accepts an explicit IPv4
address, including `0.0.0.0` to bind all interfaces. Every TCP host requires
`--auth=FILE`; missing configuration stops the host before opening its listener.
RKF1 version 3 authenticates each connection and returns its identity and grants
in Welcome. There is no anonymous or loopback exception.

The authorization file uses schema version 1:

```json
{
  "schemaVersion": 1,
  "identities": [
    {"id": "operator", "tokenSha256": "<64 lowercase hexadecimal SHA-256 digits>",
     "permissions": ["observe", "command"]}
  ],
  "deployments": {"bench": "deployment.json"}
}
```

Configure a distinct secret token of at least 16 characters per identity and
store only its SHA-256 digest on the host. `observe`, `command`, and `deployment`
are independent grants. Clients provide `ClientCredentials`; the editor CLI
reads `ROBOTKIT_IDENTITY` and `ROBOTKIT_TOKEN`. Tokens travel over the TCP channel,
so use a protected network or an encrypted tunnel when confidentiality is needed.
Test identities in `tests/fixtures/authorization.json` are public test data.

A controller receives small control state and numeric encoder/IMU frames.
`RemoteRobot` opens a separately authenticated stream connection for LiDAR,
camera, and inference subscriptions; bulk queues cannot occupy its control socket.
An authenticated deployment client can call `requestDeployment(name)` only with
the deployment grant. Names resolve through this file; clients cannot provide paths.
The host requires no active local or remote controller, validates the selected
record, closes the current host, and starts the new deployment. A successful
change disconnects clients, which must reconnect and authenticate again.

Serial hosting requires a deployment JSON file (schema v6). It refers to the
canonical semantic robot model and gives the UART path, baud, `f32` target
error budget, the `controller` id of the board it is for and a layout. The
layout (schema v1) wires ordered RKD6 channels to model actuators: each channel
names its actuator and wiring `direction`. Driver microsteps are stated on the
model actuator; layouts that state them are rejected.
The motor's full steps, transmission and driver pulse-rate ceiling come from the model; `robotd` joins them in
`robotkit.device.DeviceBinding` into each channel's joint, ratio, steps per unit
and rate ceiling, and plans on the model with those ceilings. A stepper with no
channel, or a channel with no stepper, is refused before the UART opens. When
the session opens the board must report the deployment's controller id and
acknowledge the same configuration digest; see
[`../runtime/DEVICE_PROTOCOL.md`](../runtime/DEVICE_PROTOCOL.md). A v3 or v4
file, which named a compiled fingerprint, is rejected with what to change.

`robotd identify DEVICE_PATH BAUD` prints the controller id of the board on a
port, for the deployment's `device.controller`.

See [`../tests/fixtures/device-deployment/deployment.json`](../tests/fixtures/device-deployment/deployment.json)
and its referenced [`robot.json`](../tests/fixtures/device-deployment/robot.json)
for the PTY fixture. Its calibration and controller id are test values; a physical
deployment needs measured values and the board's real id. The bench deployment
in [`../deployment/bench-nucleo-g474re`](../deployment/bench-nucleo-g474re/README.md)
carries a placeholder id until `robotd identify` reads the board's.

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
