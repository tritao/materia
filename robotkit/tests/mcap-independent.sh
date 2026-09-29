#!/usr/bin/env bash
set -euo pipefail

robotkit_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -n ${ROBOTKIT_MCAP_FIXTURE:-} ]]; then
  fixture=$ROBOTKIT_MCAP_FIXTURE
else
  output=$(ROBOTKIT_KEEP_MCAP=1 "$robotkit_dir/../haxeon/scripts/haxeon" run \
    --project "$robotkit_dir/tests/haxeon.json")
  printf '%s\n' "$output"
  fixture=$(printf '%s\n' "$output" | sed -n 's/^RobotKit MCAP fixture: //p' | tail -1)
  trap 'test ! -f "$fixture" || unlink "$fixture"; test ! -f "$fixture.incomplete.status" || unlink "$fixture.incomplete.status"' EXIT
fi
test -n "$fixture"
runtime_lib="$robotkit_dir/build/runtime/librobotkit_runtime.so"
if [[ ! -f "$runtime_lib" ]]; then
  runtime_lib="$robotkit_dir/tests/build/host/native/robotkit-world-tests/librobotkit_runtime.so"
fi
test -f "$runtime_lib"

uv run --quiet --with mcap --with msgpack python - "$fixture" "$robotkit_dir/tools/recording/dump" "$runtime_lib" <<'PYCODE'
import ctypes
import json
import subprocess
import sys
import tempfile
import msgpack
from mcap.reader import make_reader
from mcap.writer import Writer, CompressionType

with open(sys.argv[1], "rb") as stream:
    messages = list(make_reader(stream).iter_messages(log_time_order=False))

expected = ["command", "snapshot", "sensor", "fault", "world", "world_event", "process_event"]
assert [channel.topic for _, channel, _ in messages] == ["robotkit/" + name for name in expected]
for ordinal, (schema, channel, message) in enumerate(messages):
    assert schema.encoding == "robotkit-wire"
    assert channel.message_encoding == "msgpack"
    assert channel.metadata["robotkit.schema_version"] == "6"
    spec = json.loads(schema.data)
    assert spec["root"] in spec["declarations"]
    payload = msgpack.unpackb(message.data, raw=False, strict_map_key=False)
    assert isinstance(payload, dict)
    assert set(payload) == {field["id"] for field in spec["declarations"][spec["root"]]["fields"]}
    import runpy
    decode = runpy.run_path(sys.argv[2])["decode"]
    decode(payload, spec["root"], spec["declarations"])
    assert message.publish_time == ordinal
    assert message.log_time > 0
camera = msgpack.unpackb(messages[1][2].data, raw=False, strict_map_key=False)[10][1][13][4]
assert camera == bytes(range(1, 7))
lines = subprocess.check_output([sys.executable, sys.argv[2], sys.argv[1]], text=True).splitlines()
assert len(lines) == len(messages)
assert json.loads(lines[1])["payload"]["sensors"][1]["image"]["pixels"]["size"] == 6
with tempfile.NamedTemporaryFile(suffix=".mcap") as legacy:
    writer = Writer(legacy, compression=CompressionType.NONE)
    writer.start()
    schema_id = writer.register_schema("Legacy", "robotkit-wire", b"{}")
    channel_id = writer.register_channel("robotkit/legacy", "msgpack", schema_id,
                                         {"robotkit.schema_version": "5"})
    writer.add_message(channel_id, 1, bytes([0x80]), 0)
    writer.finish()
    legacy.flush()
    runtime = ctypes.CDLL(sys.argv[3])
    class Handle(ctypes.Structure):
        _fields_ = [("id", ctypes.c_uint32)]
    runtime.rk_recording_reader_open.argtypes = [ctypes.c_char_p, ctypes.POINTER(Handle)]
    handle = Handle()
    assert runtime.rk_recording_reader_open(legacy.name.encode(), ctypes.byref(handle)) == -4
print(f"independent MCAP reader and dump validated {len(messages)} typed messages")
PYCODE
