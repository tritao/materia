# Shared Materia wire tool: proposed directory and API

This is the design for the wire-foundation milestone. `tools/wire` compiles a
small `.nkw` schema into typed *payload* codecs and reference artifacts. It does
not define transport frames, sessions, CRCs, watchdogs, or device behavior.
SensorKit keeps its HMPK transport; RobotKit defines its own control transport.

## Repository layout

```text
tools/wire/
  DESIGN.md                    this design
  PROFILE.md                   normative .nkw syntax and compatibility rules
  __init__.py
  __main__.py                  `python -m tools.wire ...`
  model.py                     schema AST and normalized schema types
  parser.py                    lexer and syntax only
  validate.py                  semantic checks and lockfile evolution
  render.py                    deterministic output planning and --check
  backends/
    haxe.py                    @:wire records and strict payload wrappers
    cpp.py                     host types and payload/packed codecs
    rust.py                    no_std borrowed payload and packed codecs
    markdown.py                field/layout reference
    lock.py                    normalized compatibility lock
    vectors.py                 canonical payload vectors
  tests/                       tool-level parser, evolution, backend tests

sensorkit/schema/sensor_wire.nkw
sensorkit/schema/sensor_wire.lock.json
sensorkit/schema/sensor_wire_examples.json
sensorkit/wire.json             paths, names, enabled backends, profile
sensorkit/sensor_io/...         existing generated output paths

robotkit/schema/robot_wire.nkw  added with the v5 milestone
robotkit/wire.json              RobotKit names, limits, output paths
```

The current `sensorkit/tools/wire/{model,parser,validate}.py` can move with
small edits. Split the current `generate.py` by output backend. SensorKit's
schema, examples, lockfile, generated code, and fixture paths stay in their
current locations during extraction, minimizing churn and preserving consumers.
The package has no SensorKit or RobotKit imports.

## Driver and backend API

`wire.json` is a checked-in, per-consumer build manifest. It contains a schema
path, lock path, optional examples path, enabled backend names, language names
(Haxe package, C++ namespace, Rust module/crate), output paths, and explicit
resource limits. All paths resolve relative to the manifest. Neither the
parser nor a backend computes paths from its own source location.

```text
python -m tools.wire validate --config sensorkit/wire.json
python -m tools.wire generate --config sensorkit/wire.json
python -m tools.wire check    --config sensorkit/wire.json
```

The driver parses once, validates once, then passes an immutable normalized
schema plus backend-specific options to each backend. A backend returns a map
of relative output paths to bytes; it does not write files. The driver checks
for overlapping output paths, deterministic ordering, stale files, and output
root escapes. `check` is read-only and fails on a missing lock or stale output.
`generate` validates against the existing lock before writing any output;
changing a published wire contract is never an implicit regeneration step.
The lock backend writes the normalized schema only after validation succeeds.

Backend functions should stay simple:

```python
def render(schema: NormalizedSchema, options: BackendOptions) -> dict[Path, bytes]: ...
```

Keep example values separate from schema declarations. Vector generation takes
the validated schema and examples, producing canonical **payload** vectors.
A SensorKit-specific test adapter may wrap those payloads in HMPK and continue
to emit the existing complete-frame fixture. RobotKit's v5 frame vectors belong
to its transport tests, not the shared schema compiler.

## `.nkw` profile and compatibility

`PROFILE.md` should first document the syntax already implemented: numeric
constants, fixed-value integer enums, numbered MessagePack record fields,
`bytes`, signed and unsigned scalars, and explicit-endian fixed packed structs.
Canonical records use sorted positive integer field IDs. Readers skip unknown
fields, reject duplicate IDs and trailing data, and enforce declared ranges,
constant values, required fields, and configured resource bounds. The duplicate
rule is the `.nkw` strict profile: Haxeon's general `@:wire` decoder currently
uses last-value-wins, so generated Haxe entry points must perform their existing
strict key check before invoking Haxeon decoding. The tool does not redefine
Haxeon's general record behavior.

The existing SensorKit lock means **published message shapes are frozen**:
adding a field to an existing message is currently rejected, even if another
codec could skip it. Published field IDs and meanings are permanent; enum
members may be added, but old names/values, constants, and packed layouts may
not change. Keep that rule for SensorKit. If RobotKit needs optional additions
to existing messages, define and test that evolution policy *before* publishing
its first lock; never quietly loosen a published schema's policy. Unknown enum
values need an explicit per-message decision: reject for safety/control enums,
or preserve as raw numeric values only where forward compatibility is safe.

The present validator restricts message `u64` and some constants because Haxe
`Int64` cannot represent their full ranges. Preserve that restriction until
the Haxe backend has a tested unsigned representation. Packed structs already
support either endianness; a `bytes` field does not automatically imply a
packed type or count. Add a narrow annotation such as a bounded packed-array
reference only when RobotKit's schema needs it, and validate exact byte count
as `count * sizeof(element)` with overflow and maximum-count checks. Explicit
bounds must apply to each variable payload and to total message size.

`PROFILE.md` must also state float behavior (including where nonfinite values
are forbidden), maximum map entries, nesting and skip limits, array lengths,
and error behavior. These limits must agree across Haxe, C++, and Rust. Do not
add arbitrary nested records, dynamic trees, services, or RPC concepts.

## Generated codec boundary

Each language backend exposes typed `encode_payload`/`decode_payload` and
packed-element functions. No shared API accepts or returns a framed packet.
Haxe continues to use `@:wire` plus Haxeon MessagePack, with generated strict
validation wrappers. C++ retains typed structs and explicit span-based codecs.
The Rust backend targets `#![no_std]`, uses no allocator, `Vec`, `String`,
reflection, or generic MessagePack value tree, and accepts caller-owned output
buffers. Decoding borrows `bytes` fields from the input slice. A proposed Rust
surface is:

```rust
pub fn encode_command(value: &Command<'_>, out: &mut [u8]) -> Result<usize, EncodeError>;
pub fn decode_command(input: &[u8]) -> Result<Command<'_>, DecodeError>;
pub fn decode_joint_target(input: &[u8]) -> Result<JointTarget, DecodeError>;
```

The Rust decoder must bound map traversal and unknown-field skipping, reject
duplicate/invalid fields, and never allocate. Lifetime-bound borrowed bytes
make it possible for an RTIC task to decode from a fixed receive buffer.
RobotKit's later MCU crate owns incremental framing, CRC, sessions, sequences,
watchdog, and safety; the Rust backend owns only payload and packed codecs.

## Extraction sequence and acceptance

1. Move parser/model/validator to `tools/wire`, add the manifest driver, and
   preserve the normalized SensorKit lock byte-for-byte. Keep a temporary
   import shim only if an existing caller needs it.
2. Split Haxe, C++, Markdown, lock, and vector rendering into backends. Make
   `sensorkit/wire.json` reproduce every currently checked-in generated file
   byte-for-byte, including the existing HMPK fixture through its adapter.
3. Write `PROFILE.md` and tests for evolution, malformed records, bounds, and
   exact output paths. Add the Rust backend with `cargo test` on a host and a
   `no_std` target check; compare canonical SensorKit payload vectors across
   Haxe, C++, and Rust.
4. Point SensorKit CMake checks at `python -m tools.wire check --config
   sensorkit/wire.json`. Remove the old generator after all consumers use the
   shared driver.

The extraction is complete when SensorKit's generated artifacts and existing
tests are unchanged, the shared driver has no SensorKit paths or names in its
core/backends, and a second schema can select its own names, limits, and output
roots without changing compiler code.
