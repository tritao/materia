#!/usr/bin/env bash
set -euo pipefail
router_repo=$(pwd -P)
router_heavy=${MATERIA_HEAVY_RUN:-/home/joao/dev/materia-cache/tools/heavy-run}
python3 machinekit/examples/sheet-router/firmware/check.py
"$router_heavy" --mem 3G --need 1G --label router-firmware-host env CARGO_TARGET_DIR="$router_repo/build/sheet-router-work/sr15/host" cargo test --offline --manifest-path robotkit/device_protocol/Cargo.toml --features std --lib router
"$router_heavy" --mem 3G --need 1G --label router-g474-check env CARGO_TARGET_DIR="$router_repo/build/sheet-router-work/sr15/arm" cargo check --offline --manifest-path robotkit/device_protocol/boards/nucleo-g474re/Cargo.toml --target thumbv7em-none-eabihf
