# `.wire.idl` profile

This document describes the small language currently parsed by
`tools/wire/parser.py` and validated by `tools/wire/validate.py`. The file
extension is `.wire.idl`; the grammar is the former `.nkw` grammar.

## Declarations

```text
const VERSION : u8 = 1;
enum Mode : u8 { idle = 0 active = 1 }
message Sample { 1 version : u8 constant = VERSION 2 data : bytes }
packed JointTarget endian little { joint : u16 mode : u8 flags : u8 value : f32 }
```

The language has numeric constants; fixed numeric enums; numbered MessagePack
messages; and fixed packed structs. Semicolons are optional. `#` and `//`
start line comments. There are no imports, nested records, dynamic fields,
strings, services, or RPC declarations. A schema must contain at least one
message or packed struct.

Scalar types are `u8`, `u16`, `u32`, `u64`, `i8`, `i16`, `i32`, `i64`, `f32`,
and `f64`, with the usual bit-width ranges. Message fields may also use
`bytes` and an enum. `u64` message fields are currently forbidden because
Haxe `Int64` cannot represent the full unsigned range; use `i64 nonnegative`
when appropriate. Fixed arrays (`type[count]`) are supported only inside
packed structs, with `1 <= count <= 1,000,000`. Each packed struct declares
`endian little` or `endian big`; its size is the sum of its fields with no
padding.

## Packed field constraints

A packed field may carry one optional constraint after its type. Constraints are part of the
compatibility lock (an existing field cannot gain, lose or change one) and the generated Rust and
C++ codecs enforce them on both encode and decode. Schemas without constraints generate exactly
the same output and lock as before.

```text
position : u8 range 0..11      # integer types; applies to each element of an array
flags    : u8 mask 0x03        # unsigned; bits outside the mask must be zero
reserved : u8[2] zero          # integer types; must be zero
text     : u8[12] text         # u8 arrays; printable ASCII to the first NUL, only NULs after it
mode     : Mode                # an enum type (scalar only): the stored value must be a member
```

Hex integers (`0x3F`) are accepted wherever an integer is. A field named `range`, `mask`, `zero`
or `text` still parses, since a modifier is never followed by `:`. Rust adds
`Error::Constraint(&'static str)` naming `Struct.field` and a `check()` method; C++ adds
`valid()`, and `decode`/`encode` return false for a violation. Rules involving two fields stay in
the consumer.

## Message encoding

Messages are MessagePack maps with positive integer field IDs sorted for
canonical encoding. All declared fields are required by the generated strict
codec. A declared `constant = value` must match on decode. The optional
`nonnegative` modifier applies to signed integer fields. Strict decoders
skip unknown field IDs, reject duplicate IDs and missing required IDs, and
reject trailing bytes. Haxeon's general `@:wire` decoder has different
missing/duplicate behavior; use the generated strict wrapper for this profile.
MessagePack encodes values only: HMPK and RobotKit framing are outside this
language.

The current tool does not declare per-field byte limits for messages or nonfinite-float
policies. Those must be validated by the consuming protocol until added to
the language. SensorKit's Haxeon reader applies its own container and nesting
limits; do not assume those limits are an `.wire.idl` promise.

## Compatibility lock

The normalized lock fixes every published constant type/value, enum storage
type and existing member value, message field list, and packed layout. New
enum members, messages, and packed structs are allowed. Existing message
shapes cannot gain fields under the current policy; publish a new message
instead. Existing field IDs, enum values, and packed offsets must never be
reassigned. Lock validation happens before generation.
