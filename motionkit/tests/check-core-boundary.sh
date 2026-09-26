#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
if rg -n 'import (robotkit|machinekit)\.' "$root/motionkit/haxe"; then
  echo "MotionKit core imports RobotKit or MachineKit" >&2
  exit 1
fi
