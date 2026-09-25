"""Derive the RKD5 accidental-mismatch fingerprint from a deployment artifact.

The layout file is the exact, immutable device configuration distributed to
both the host deployment and firmware build. This tool deliberately does not
interpret hardware-specific fields inside it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

DOMAIN = b"RobotKit RKD5 device layout fingerprint v1\0"
DEFAULT_LOCK = Path(__file__).resolve().parents[1] / "schema/device_wire.lock.json"


def _unique_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    value: dict[str, object] = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"duplicate schema lock key: {key}")
        value[key] = item
    return value


def _reject_nonfinite(value: str) -> object:
    raise ValueError(f"nonfinite schema lock value: {value}")


def fingerprint(layout: bytes, schema_lock: bytes) -> bytes:
    """Return the first 16 bytes of a domain-separated SHA-256 digest."""
    if not layout:
        raise ValueError("deployment layout must not be empty")
    lock = json.loads(schema_lock, object_pairs_hook=_unique_object,
                      parse_constant=_reject_nonfinite)
    if not isinstance(lock, dict):
        raise ValueError("schema lock must be a JSON object")
    canonical_lock = json.dumps(lock, sort_keys=True, separators=(",", ":"),
                                ensure_ascii=False, allow_nan=False).encode("utf-8")
    digest = hashlib.sha256()
    digest.update(DOMAIN)
    for block in (canonical_lock, layout):
        digest.update(len(block).to_bytes(8, "little"))
        digest.update(block)
    return digest.digest()[:16]


def cpp_constant(value: bytes) -> str:
    items = ", ".join(f"0x{byte:02x}" for byte in value)
    return ("#pragma once\n#include <array>\n#include <cstdint>\n\n"
            "namespace robotkit::deployment {\n"
            f"inline constexpr std::array<std::uint8_t, 16> model_fingerprint{{{items}}};\n"
            "}\n")


def rust_constant(value: bytes) -> str:
    items = ", ".join(f"0x{byte:02x}" for byte in value)
    return f"pub const MODEL_FINGERPRINT: [u8; 16] = [{items}];\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("layout", type=Path, help="immutable deployed device layout/configuration file")
    parser.add_argument("--schema-lock", type=Path, default=DEFAULT_LOCK)
    parser.add_argument("--cpp", type=Path, help="write a C++ constexpr header")
    parser.add_argument("--rust", type=Path, help="write a Rust const module")
    args = parser.parse_args()
    value = fingerprint(args.layout.read_bytes(), args.schema_lock.read_bytes())
    if args.cpp:
        args.cpp.write_text(cpp_constant(value), encoding="utf-8")
    if args.rust:
        args.rust.write_text(rust_constant(value), encoding="utf-8")
    print(value.hex())


if __name__ == "__main__":
    main()
