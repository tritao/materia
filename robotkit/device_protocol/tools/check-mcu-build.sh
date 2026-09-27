#!/usr/bin/env bash
set -euo pipefail

device_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target=thumbv7em-none-eabihf

# The package also has POSIX PTY binaries, which are host-only.
cargo build --manifest-path "$device_dir/Cargo.toml" --target "$target" --lib -p robotkit-device-protocol
cargo build --manifest-path "$device_dir/boards/nucleo-g474re/Cargo.toml" --target "$target"
