#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
artifact="$repo_root/app/build/host/main.hl"
haxeon_root="$repo_root/haxeon"

if [[ ! -f "$artifact" ]]; then
	echo 'Materia has no built app. Run ./haxeon/scripts/haxeon build --project=app/haxeon.json first.' >&2
	exit 1
fi

export HAXEON_HOME="$haxeon_root"
export LD_LIBRARY_PATH="$haxeon_root/out:$haxeon_root/.tools/hashlink:$repo_root/app/build/host/native/app${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$repo_root"
exec "$haxeon_root/.tools/hashlink/hl" "$artifact" "$@"
