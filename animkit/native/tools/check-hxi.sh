#!/usr/bin/env bash
set -euo pipefail
materia_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
exec "$materia_dir/haxeon/packages/animation/native/tools/check-hxi.sh" "$@"
