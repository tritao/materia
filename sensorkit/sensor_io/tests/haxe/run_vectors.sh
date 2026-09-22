#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd "$(dirname "$0")" && pwd)
sensor_root=$(cd "$script_dir/../../.." && pwd)
haxeon_root=${HAXEON_ROOT:-"$sensor_root/../haxeon"}
haxe="$haxeon_root/.tools/haxe/haxe"
hl="$haxeon_root/.tools/hashlink/hl"
output="${TMPDIR:-/tmp}/sensorkit-wire-vectors-$$.hl"
vectors="$sensor_root/sensor_io/tests/fixtures/sensor_wire_vectors.tsv"

if [[ ! -x "$haxe" || ! -x "$hl" ]]; then
	echo "Haxeon tools are missing under $haxeon_root" >&2
	exit 1
fi
trap 'rm -f "$output"' EXIT

"$haxe" --cwd "$haxeon_root" -cp src --run compiler.tools.HaxeonCompiler \
	--output="$output" --entry=SensorWireVectorCheck \
	--root=stdlib \
	--root="$sensor_root/sensor_io/haxe" \
	--root="$sensor_root/sensor_io/tests/haxe" \
	"$sensor_root/sensor_io/tests/haxe/SensorWireVectorCheck.hx"

set +e
if [[ "$(uname -s)" == "Darwin" ]]; then
	DYLD_LIBRARY_PATH="$haxeon_root/out:$haxeon_root/.tools/hashlink${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}" \
		"$hl" "$output" "$vectors"
else
	LD_LIBRARY_PATH="$haxeon_root/out:$haxeon_root/.tools/hashlink${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
		"$hl" "$output" "$vectors"
fi
status=$?
set -e
if [[ $status -ne 42 ]]; then
	echo "SensorKit Haxe wire vectors failed with exit $status" >&2
	exit 1
fi
