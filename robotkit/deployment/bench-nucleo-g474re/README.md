# Bench deployment: NUCLEO-G474RE

`deployment.json` names the board it is for in `device.controller`. The value
in this file, `baadf00dbaadf00dbaadf00dbaadf00d`, is a placeholder: no real
board has this id, so a session against any board is refused until it is
replaced. Read the real id from the board on the bench with

```sh
robotd identify /dev/robotkit-mcu 921600
```

and put the printed 32 hex digits in `device.controller`. The id is the
STM32G4's factory-programmed unique ID, so it is not compiled into firmware
and the firmware image is the same for every board of this type.

`layout.json` wires the board's channels to the model's actuators, with each
driver's direction and microstepping; the motors' full steps come from
`robot.json`. See `robotkit/runtime/DEVICE_PROTOCOL.md`.

## G13 switch input assignment — hardware unverified

The minimal bench firmware assigns input channel 0 to **PB0** and channel 1
to **PB1**. Both are configured as GPIO inputs with internal pull-ups. For the
active-low bench setup, each dry contact closes to board ground; the deployment
input entry must explicitly set `active_high: false` and name the model switch
and its actuator. USART1 remains PC4 TX / PC5 RX. Identify PB0/PB1 on the exact
board schematic before connecting; no connector-number assignment is asserted.

These assignments and the GPIO/edge-capture path have **not been verified on
hardware**. No motors are connected or driven by this firmware. It still
advertises profile 2, two virtual actuator setpoints and degree-1 motion. Input
edges are polled in the DWT source clock and State6 reports closing/opening
counts and captured ticks; captured step counts remain zero because this bench
adapter has no physical step generator. It is not a physical router homing
controller and does not accept scoped motor holds or counter calibration.

At the Phase C gate, compile with `cargo check --offline` from the firmware
directory. That checks code, not electrical polarity, contact timing or wiring.
