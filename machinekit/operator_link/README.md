# Operator link

The path between an operator device (the jog pendant) and the motion core, and between the host
and the cabinet controller about that device. The spec of the pendant itself is
`../PENDANT_PLAN.md`; the phases are in `../CONTROL_CABINET_PLAN.md` (CC0 to CC1).

## Pieces

| Where | What |
|---|---|
| `schema/operator.wire.idl` | The records and enums, in the same `.wire.idl` system as RKD6, with a compatibility lock. |
| `src/operator_wire.rs` | Generated `no_std` codec. Enforces the schema's ranges, masks, reserved bytes, enum values and text rules. |
| `../haxe/src/machinekit/operator/wire/` | Generated Haxe records and named enum constants. |
| `src/lib.rs` | The one cross-field check (label index by kind) and the decode entry points. |
| `src/authority.rs` | Pendant jog authority: sequence, lease, release-before-resume, wheel gating, hold-to-run, refusals. |
| `src/host_link.rs` | What the host allows (`OperatorState`); stale means not allowed. |
| `src/profile.rs` | The compiled machine profile and its digest. |
| `src/operator_frame.rs` | The `OPR1` frame for host to controller messages (CRC from the RKD6 core). |
| `../haxe/src/machinekit/operator/` | `OperatorProfile` (the model and its codec), `PendantCompiler`, `OperatorFrame`. |
| `../cabinet_node/` | `OperatorService`: runs the above on the cabinet node and gates executing jogs through the core's `JogGate`. |
| `vectors/` | Byte vectors made by the Haxe compiler; the Rust tests decode them. |

## Design rules

- Safety is hard wired and separate. Nothing here is a safety function.
- The controller evaluates machine facts (`OperatorCondition`); the host sends only intent
  (`OperatorState.jog_allowed`). Missing or stale means no jog.
- The host plans jog segments (`purpose = 1`) and the core executes them. The service only decides
  whether an executing jog may continue, and stops it with the ordinary `StopReason::Stop`.
- Programs and homing are never gated here. A running program blocks the pendant.
- One definition of the wire format: the schema. One implementation of the rules: `authority.rs`.
  The Haxe side compiles profiles and builds frames; it does not re-implement the rules.
- The lock is frozen from the first hardware build; until then breaking changes regenerate it.

## Commands

```text
python3 -m tools.wire generate --config machinekit/operator_link.json   # from the repo root
machinekit/scripts/test-operator-link      # schema check, wire-tool tests, Rust tests, MCU build
PENDANT_VECTORS=write ...                  # refresh vectors/ after reviewing a compiler change
```

`scripts/test-haxeon` runs the first script before the Haxe suites. The Haxe side of the link is
`OperatorProfileTests` and `PendantCompilerTests` in the smoke suite.
