# ADR 001: Shared toolpath format

Status: Accepted

## Context

CncKit, CamKit, MotionKit and StockKit need a common representation of machining paths. CncKit currently emits machine coordinates while CamSetup emits work coordinates. Controller details and motion execution are mixed into the current CNC IR.

## Decision

1. `toolpathkit` has no dependencies. It owns the toolpath IR, tools, setups, and travel envelope checks. It imports no MotionKit, CadKit, or G-code types.
2. Moves use the part's work coordinates and are grouped by setup. `SetSetup(id)` changes the active setup. Machine-coordinate moves such as G53 and G28 carry an explicit flag. The motion adapter transforms setup coordinates into machine coordinates. G54–G59 mapping exists only at the G-code boundary.
3. Cutter compensation stays in CncKit. CncKit resolves G41/G42 before emitting shared operations; compensation operations do not enter the shared format.
4. The format carries typed spindle and coolant state. The execution adapter selects controller channel names.
5. There are no compatibility aliases. Every consumer is in this repository, so each rename updates all call sites in the same commit.

## Consequences

The G-code interpreter and CAM producers share geometry and provenance without depending on motion execution. Controller mapping remains in CncKit, and execution policy remains in the motion adapter.

## Addendum: setup placement and machine travel

`ToolpathProgram` carries its ordered setups alongside operations and tools. The first
setup is active until a `SetSetup` operation selects another known setup. A setup's
required work origin places work coordinates in machine space; its stock, fixtures,
and safe Z are optional planning data. A measured or probed origin is represented
by a program setup at that measured position.

`MachineBinding` owns the physical frame, axes, speed, tolerances, and optional
`TravelEnvelope`. A binding intersects logical axis travel with model joint limits
and supplies that envelope to compilation and lowering. It never owns setup
positions. `CncController` owns only G-code dialect, G54–G59 offsets, G28/G30
homes, H/D mappings, and the controller tool library. The compile call supplies
the start position.
