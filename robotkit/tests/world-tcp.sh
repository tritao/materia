#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
project="$repo_dir/robotkit/robotd/haxeon.json"
port="${ROBOTKIT_TEST_PORT:-17942}"
log_dir="$repo_dir/robotkit/tests/build"
server_log="$log_dir/world-tcp-server.log"
mkdir -p "$log_dir"

"$repo_dir/haxeon/scripts/haxeon" build --project "$project"
"$repo_dir/haxeon/scripts/haxeon" run --project "$project" -- \
  --server --once --robot-id=42 --port="$port" >"$server_log" 2>&1 &
server_pid=$!
cleanup() {
  if kill -0 "$server_pid" 2>/dev/null; then
    kill "$server_pid" 2>/dev/null || true
    wait "$server_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

for _ in $(seq 1 100); do
  if grep -q "robotd: listening" "$server_log" 2>/dev/null; then
    break
  fi
  if ! kill -0 "$server_pid" 2>/dev/null; then
    cat "$server_log"
    exit 1
  fi
  sleep 0.05
done

grep -q "robotd: listening" "$server_log"
"$repo_dir/haxeon/scripts/haxeon" run --project "$project" -- \
  --world-client --port="$port"
wait "$server_pid"
trap - EXIT
