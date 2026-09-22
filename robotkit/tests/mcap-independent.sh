#!/usr/bin/env bash
set -euo pipefail

robotkit_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
output=$(ROBOTKIT_KEEP_MCAP=1 "$robotkit_dir/../haxeon/scripts/haxeon" run \
  --project "$robotkit_dir/tests/haxeon.json")
printf '%s\n' "$output"
fixture=$(printf '%s\n' "$output" | sed -n 's/^RobotKit MCAP fixture: //p' | tail -1)
test -n "$fixture"
trap 'test ! -f "$fixture" || unlink "$fixture"; test ! -f "$fixture.incomplete.status" || unlink "$fixture.incomplete.status"' EXIT

uv run --quiet --with mcap python - "$fixture" <<'PY'
import json
import sys
from mcap.reader import make_reader

with open(sys.argv[1], "rb") as stream:
    messages = list(make_reader(stream).iter_messages())

expected = [
    "robotkit/command", "robotkit/snapshot", "robotkit/sensor",
    "robotkit/fault", "robotkit/world", "robotkit/world_event",
]
assert [channel.topic for _, channel, _ in messages] == expected
for schema, channel, message in messages:
    assert schema.encoding == "jsonschema"
    assert channel.message_encoding == "json"
    payload = json.loads(message.data)
    assert payload["version"] == 1
    assert int(payload["recordingTimestampNs"]) == message.log_time
    assert message.publish_time == message.log_time
print(f"independent MCAP reader validated {len(messages)} typed messages")
PY
