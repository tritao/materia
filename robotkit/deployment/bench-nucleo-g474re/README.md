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
