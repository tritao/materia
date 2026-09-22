# SensorKit wire schema

Generated from `schema/sensor_wire.nkw`. MessagePack fields use integer map keys; the HMPK envelope is version 1.

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
| 3 | `sensor` | `u64` | required |
| 4 | `sequence` | `u64` | required |
| 5 | `capture_time` | `f64` | required |
| 6 | `delivery_time` | `f64` | required |
| 7 | `frame` | `u64` | required |
| 8 | `width` | `u32` | required |
| 9 | `height` | `u32` | required |
| 10 | `stride` | `u32` | required |
| 11 | `pixel_format` | `PixelFormat` | required |
| 12 | `data` | `bytes` | required |

Reserved IDs: `13..31`.

## ImuSampleMessage

| ID | Field | Type | Rule |
| ---: | --- | --- | --- |
| 1 | `schema_version` | `u8` | constant `1` |
| 2 | `message_type` | `MessageType` | constant `4` |
| 3 | `sensor` | `u64` | required |
| 4 | `sequence` | `u64` | required |
| 5 | `capture_time` | `f64` | required |
| 6 | `delivery_time` | `f64` | required |
| 7 | `frame` | `u64` | required |
| 8 | `data` | `bytes` | required |

Reserved IDs: `9..31`.

## LidarScanMessage

| ID | Field | Type | Rule |
| ---: | --- | --- | --- |
| 1 | `schema_version` | `u8` | constant `1` |
| 2 | `message_type` | `MessageType` | constant `5` |
| 3 | `sensor` | `u64` | required |
| 4 | `sequence` | `u64` | required |
| 5 | `capture_time` | `f64` | required |
| 6 | `delivery_time` | `f64` | required |
| 7 | `frame` | `u64` | required |
| 8 | `horizontal_count` | `u32` | required |
| 9 | `vertical_count` | `u32` | required |
| 10 | `return_stride` | `u32` | constant `12` |
| 11 | `data` | `bytes` | required |

Reserved IDs: `12..31`.

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

Existing fields, IDs, types, constants, and packed layouts are immutable. New enum values and new message types can be added. Adding a field to an existing message is rejected; define a new message when the wire shape changes.
