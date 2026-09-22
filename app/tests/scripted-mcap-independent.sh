#!/usr/bin/env bash
set -euo pipefail

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$repo_dir"
output=$(MATERIA_KEEP_SCRIPT_MCAP=1 "$repo_dir/haxeon/scripts/haxeon" run \
  --project "$repo_dir/app/tests/haxeon.json")
printf '%s\n' "$output"
fixture=$(printf '%s\n' "$output" | sed -n 's/^Materia scripted MCAP fixture: //p' | tail -1)
if [[ "$fixture" != /* ]]; then fixture="$repo_dir/app/tests/$fixture"; fi
test -f "$fixture"
fixture=$(realpath "$fixture")
case "$fixture" in
  */build/scripted-setup-[0-9]*/scripted.mcap) ;;
  *) echo "Unexpected scripted MCAP fixture path: $fixture" >&2; exit 1 ;;
esac

cleanup() {
  test ! -f "$fixture" || unlink "$fixture"
  test ! -f "$fixture.incomplete.status" || unlink "$fixture.incomplete.status"
  rmdir "$(dirname "$fixture")" 2>/dev/null || true
}
trap cleanup EXIT

uv run --quiet --with mcap python - "$fixture" <<'PY'
import json
import sys
from mcap.reader import make_reader

with open(sys.argv[1], "rb") as stream:
    messages = list(make_reader(stream).iter_messages())

assert len(messages) == 8
assert [channel.topic for _, channel, _ in messages].count("robotkit/snapshot") == 2
assert [channel.topic for _, channel, _ in messages].count("robotkit/sensor") == 6
robots = set()
for schema, channel, message in messages:
    assert schema.encoding == "jsonschema"
    assert channel.message_encoding == "json"
    payload = json.loads(message.data)
    assert payload["version"] == 1
    assert int(payload["recordingTimestampNs"]) == message.log_time == message.publish_time
    robots.add(payload["robotId"])
    if channel.topic == "robotkit/sensor":
        sensor = payload["payload"]
        assert sensor["sensorId"] and sensor["frameId"] and sensor["linkId"] == "arm"
        assert len(sensor["mountPosition"]) == 3 and len(sensor["mountRotation"]) == 4
        assert sensor["values"] and int(sensor["sequence"]) >= 0
assert robots == {"materia/robot", "materia/robot-b"}
print("independent MCAP reader validated two scripted robots and six sensor frames")
PY
