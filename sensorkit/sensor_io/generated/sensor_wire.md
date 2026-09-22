# SensorKit wire schema

Generated from `schema/sensor_wire.nkw`. MessagePack fields use integer map keys; the HMPK envelope is version 1.

Integer ranges follow the declared primitive. Haxe `u32` fields use `Int64` so the full unsigned 32-bit range is representable; message fields cannot use `u64` because Haxe `Int64` cannot represent its full range. `i64 nonnegative` fields match Haxe `Int64` and restrict the protocol value to `0..2^63-1`. Use the generated `SensorWireCodec` entry points in Haxe to enforce required fields, duplicates, constants, and integer ranges.

## MessageType

| Name | Value |
| --- | ---: |
| `camera_frame` | 1 |
| `depth_frame` | 2 |
| `segmentation_frame` | 3 |
| `imu_sample` | 4 |
| `lidar_scan` | 5 |

## PixelFormat

| Name | Value |
| --- | ---: |
| `rgba8` | 1 |
| `r32f_le` | 2 |
| `r32u_le` | 3 |
| `u64_le` | 4 |

## PackedFrameMessage

| ID | Field | Type | Rule |
| ---: | --- | --- | --- |
| 1 | `schema_version` | `u8` | constant `1` |
| 2 | `message_type` | `MessageType` | required |
| 3 | `sensor` | `i64` | required, non-negative |
| 4 | `sequence` | `i64` | required, non-negative |
| 5 | `capture_time` | `f64` | required |
| 6 | `delivery_time` | `f64` | required |
| 7 | `frame` | `i64` | required, non-negative |
| 8 | `width` | `u32` | required |
| 9 | `height` | `u32` | required |
| 10 | `stride` | `u32` | required |
| 11 | `pixel_format` | `PixelFormat` | required |
| 12 | `data` | `bytes` | required |

## ImuSampleMessage

| ID | Field | Type | Rule |
| ---: | --- | --- | --- |
| 1 | `schema_version` | `u8` | constant `1` |
| 2 | `message_type` | `MessageType` | constant `4` |
| 3 | `sensor` | `i64` | required, non-negative |
| 4 | `sequence` | `i64` | required, non-negative |
| 5 | `capture_time` | `f64` | required |
| 6 | `delivery_time` | `f64` | required |
| 7 | `frame` | `i64` | required, non-negative |
| 8 | `data` | `bytes` | required |

## LidarScanMessage

| ID | Field | Type | Rule |
| ---: | --- | --- | --- |
| 1 | `schema_version` | `u8` | constant `1` |
| 2 | `message_type` | `MessageType` | constant `5` |
| 3 | `sensor` | `i64` | required, non-negative |
| 4 | `sequence` | `i64` | required, non-negative |
| 5 | `capture_time` | `f64` | required |
| 6 | `delivery_time` | `f64` | required |
| 7 | `frame` | `i64` | required, non-negative |
| 8 | `horizontal_count` | `u32` | required |
| 9 | `vertical_count` | `u32` | required |
| 10 | `return_stride` | `u32` | constant `12` |
| 11 | `data` | `bytes` | required |

## ImuSampleData (packed)

Endianness: `little`; size: **192 bytes**.

| Field | Type | Size |
| --- | --- | ---: |
| `angular_velocity` | `f64[3]` | 24 |
| `linear_acceleration` | `f64[3]` | 24 |
| `angular_velocity_covariance` | `f64[9]` | 72 |
| `linear_acceleration_covariance` | `f64[9]` | 72 |

## LidarReturn (packed)

Endianness: `little`; size: **12 bytes**.

| Field | Type | Size |
| --- | --- | ---: |
| `range` | `f32` | 4 |
| `intensity` | `f32` | 4 |
| `hit` | `u8` | 1 |
| `reserved` | `u8[3]` | 3 |

## Compatibility

Published message shapes and packed layouts are immutable. Existing field IDs, types, and constants cannot change. Create a new message and message type when a shape must change. Existing enum values cannot change; new enum values and message types may be added. Unknown MessagePack fields are skipped, and duplicate field IDs are rejected.
