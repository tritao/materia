#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
server_project="$repo_dir/robotkit/robotd/haxeon.json"
client_project="$repo_dir/robotkit/tests/integration/haxeon.json"
port="${ROBOTKIT_TEST_PORT:-17942}"
log_dir="$repo_dir/robotkit/tests/build"
server_log="$log_dir/world-tcp-server.log"
mkdir -p "$log_dir"

"$repo_dir/haxeon/scripts/haxeon" build --project "$server_project"
server_mode="--server --once"
client_mode=""
if [[ "${ROBOTKIT_TEST_SESSIONS:-0}" == "1" ]]; then
  server_mode="--server"
  client_mode="--sessions"
fi
setsid "$repo_dir/haxeon/scripts/haxeon" run --project "$server_project" -- \
  $server_mode --multi-joint --robot-id=42 --port="$port" >"$server_log" 2>&1 &
server_pid=$!
cleanup() {
  if kill -0 "$server_pid" 2>/dev/null; then
    kill -- "-$server_pid" 2>/dev/null || kill "$server_pid" 2>/dev/null || true
    wait "$server_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ready=0
for _ in $(seq 1 1200); do
  if python3 - "$port" <<'PY'
import socket
import sys

connection = socket.socket()
connection.settimeout(0.1)
try:
    connection.connect(("127.0.0.1", int(sys.argv[1])))
except OSError:
    sys.exit(1)
finally:
    connection.close()
PY
  then
    ready=1
    break
  fi
  if ! kill -0 "$server_pid" 2>/dev/null; then
    cat "$server_log"
    exit 1
  fi
  sleep 0.05
done

if [[ "$ready" != "1" ]]; then
  cat "$server_log"
  echo "robotd did not accept TCP connections on port $port" >&2
  exit 1
fi
# Let robotd process the probe disconnect before the real controller connects.
sleep 0.1
"$repo_dir/haxeon/scripts/haxeon" run --project "$client_project" -- \
  --port="$port" $client_mode
if [[ "${ROBOTKIT_TEST_SESSIONS:-0}" != "1" ]]; then
  wait "$server_pid"
  trap - EXIT
fi
