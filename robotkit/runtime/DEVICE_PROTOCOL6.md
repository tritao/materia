# RobotKit scheduled device protocol (RKD6)

RKD6 is a separate protocol from [RKD5](DEVICE_PROTOCOL.md). Its four-byte
sync marker is `RKD6`. A frame is marker (4), message type (1), reserved zero
(1), little-endian payload length (2), payload, then little-endian CRC-32/IEEE
(4), calculated over every preceding byte. The largest current payload is a
64-actuator segment (36 + 64 × 25 = 1,636 bytes), so the frame maximum is
1,648 bytes. A receiver rejects unknown types, wrong lengths, a nonzero
reserved byte, CRC mismatch, or a malformed segment. Message id 16 is reserved
for EVENT and cannot be sent until its payload is defined in A8.

`device_wire6.wire.idl` is the canonical fixed-record schema; `wire6.json`
generates Rust and C++ codecs. Segment coefficients are `f32` in local seconds,
with degree at most five. Segment start and duration are device ticks. One
`Segment6Coefficients` record follows the header per actuator. `STATE6` has
one `ActuatorState6` record per actuator. Queue control and safety command
frames carry the fixed records specified by the schema. The session ACK
reports the tick rate, step tick rate, degree limit and queue capacities.

| Quantity | RKD6 bound | Source |
| --- | ---: | --- |
| Maximum actuators | 64 | schema |
| Maximum segment frame | 1,648 bytes | 8 + 36 + 64 × 25 + 4 |
| Step tick default | 40 kHz | LA-D2; board may configure another rate |
| Segment queue depth | negotiated | `SESSION_ACK6.segment_capacity` |
| Event queue depth | negotiated | `SESSION_ACK6.event_capacity` |
| Committed horizon | link latency + 2 × clock uncertainty beyond host horizon | A3 |
| Wire transmission time | `10 × frame_bytes / baud` seconds for 8N1 | serial qualification input |

A host must qualify baud and queue depth against the declared segment rate
before motion. Clock-sync uncertainty and link-loss timeout are deployment
bounds, not hard-coded protocol constants. `TIME_SYNC_REPLY` carries device
receive and transmit ticks for the estimator in A3. The RKD5 target-streaming
path remains supported separately.

The host estimator fits device ticks against host monotonic nanoseconds from
the lowest RTT samples in a bounded window. Its uncertainty is half the
minimum RTT plus the worst selected fit residual. It exposes request cadence,
time mapping and the extra commit horizon (`link latency + 2 × uncertainty`).
When a new sample steps outside the deployment bound, it latches
`clock_sync_lost` and disallows further commits. An RKD6 endpoint sends the
periodic requests and reports that reason in its runtime snapshot.
