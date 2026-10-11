//! Regenerate the current shared Rust/C++ framing fixtures with the production codec.
use robotkit_device_protocol::device_wire6::*;
use robotkit_device_protocol::frame6::{encode_frame6, MAX_FRAME_SIZE};
fn emit(name: &str, kind: u8, body: &[u8]) {
    let mut frame = [0; MAX_FRAME_SIZE];
    let count = encode_frame6(kind, body, &mut frame).unwrap();
    let hex: String = frame[..count].iter().map(|byte| format!("{byte:02x}")).collect();
    println!("{name}\t{hex}");
}
fn main() {
    let mut body = [0; TimeSyncRequest::SIZE];
    TimeSyncRequest {host_send_ns:123456789}.encode(&mut body).unwrap();
    emit("time_sync_request",3,&body); emit("hold",8,&[]); emit("stop",11,&[]);
    let mut begin = SessionBegin6::decode(&[0;SessionBegin6::SIZE]).unwrap();
    begin.session = 7; begin.protocol_version = PROTOCOL_VERSION; begin.expected_controller = [3;16];
    begin.actuator_count = 2; begin.max_degree = 5; begin.step_tick_hz = 40000; begin.max_acceleration = 4.0;
    begin.actuator_max_acceleration[..2].copy_from_slice(&[2.0,4.0]);
    begin.steps_per_unit.fill(400.0); begin.min_step_ticks[..2].fill(1);
    begin.actuator_ratio.fill(1.0); begin.link_loss_timeout_ns = 500000000;
    let mut body = [0;SessionBegin6::SIZE]; begin.encode(&mut body).unwrap();
    emit("session_begin6_v15",1,&body);
    let event = Event6 {queue_revision:1,plan_id:7,path_ticks:123,channel:0,kind:1,
        hold_policy:2,digital:1,analog:0.0,argument:0.0,command:[0;48]};
    let mut body = [0;Event6::SIZE]; event.encode(&mut body).unwrap(); emit("digital_event",16,&body);
    let sensor = Sensor6Header {session:9,timestamp_ticks:10000,sequence:1,slot:0,value_count:6};
    let mut body = vec![0;Sensor6Header::SIZE+6*Sensor6Value::SIZE];
    sensor.encode(&mut body[..Sensor6Header::SIZE]).unwrap();
    for (i,value) in [1.0,240.0,24.0,0.0,0.0,6400.0].iter().enumerate() {
        Sensor6Value {value:*value}.encode(&mut body[Sensor6Header::SIZE+i*Sensor6Value::SIZE..][..Sensor6Value::SIZE]).unwrap();
    }
    emit("sensor6",17,&body);
    let output = ControllerOutput6 {session:7,sequence:1,channel:3,kind:1,digital:1,analog:0.0};
    let mut body = [0;ControllerOutput6::SIZE]; output.encode(&mut body).unwrap();
    emit("controller_output",18,&body);
}
