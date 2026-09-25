#!/usr/bin/env python3
"""Generate SensorKit wire artifacts through shared renderers."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from tools.wire.model import Schema
from tools.wire.parser import parse_file
from tools.wire.validate import ValidationError, dump_normalized, normalized, validate_evolution
from tools.wire.backends.cpp import cpp_header, codec_header, codec_cpp
from tools.wire.backends.haxe import haxe_files
from tools.wire.backends.markdown import docs
from sensorkit.wire_vectors import vector_fixture, haxe_vector_check

ROOT = Path(__file__).resolve().parent
SCHEMA = ROOT / "schema/sensor_wire.nkw"
LOCK = ROOT / "schema/sensor_wire.lock.json"

def outputs(schema: Schema, norm: dict[str, Any], config: dict[str, Any]) -> dict[Path, str]:
    haxe = config["haxe"]
    cpp = config["cpp"]
    markdown = config["markdown"]
    paths = config["outputs"]
    result = haxe_files(schema, norm, root=ROOT / haxe["root"], package=haxe["package"],
                        codec_name=haxe["codec_name"], constants_name=haxe["constants_name"],
                        schema_label=haxe["schema_label"], codec_comment=haxe["codec_comment"])
    result.update({
        ROOT / paths["cpp_types"]: cpp_header(schema, norm, namespace=cpp["namespace"], aliases=cpp["aliases"]),
        ROOT / paths["cpp_codec_header"]: codec_header(schema, namespace=cpp["namespace"], reader_namespace=cpp["reader_namespace"], generated_header=cpp["generated_header"]),
        ROOT / paths["cpp_codec_source"]: codec_cpp(schema, norm, namespace=cpp["namespace"], reader_namespace=cpp["reader_namespace"], codec_header_name=cpp["codec_header"], msgpack_header=cpp["msgpack_header"]),
        ROOT / paths["markdown"]: docs(schema, norm, title=markdown["title"], intro=markdown["intro"], codec_name=haxe["codec_name"]),
        ROOT / paths["vectors"]: vector_fixture(norm),
        ROOT / paths["haxe_vectors"]: haxe_vector_check(norm),
        LOCK: dump_normalized(norm),
    })
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail when checked-in generated output is stale")
    parser.add_argument("--schema", type=Path, default=SCHEMA)
    args = parser.parse_args(argv)
    try:
        schema = parse_file(args.schema)
        norm = normalized(schema)
        if LOCK.exists():
            old = json.loads(LOCK.read_text(encoding="utf-8"))
            validate_evolution(norm, old)
        elif args.check:
            raise ValidationError("schema compatibility lock is missing")
        config = json.loads((ROOT / "wire.json").read_text(encoding="utf-8"))
        generated = outputs(schema, norm, config)
        stale: list[Path] = []
        for path, content in generated.items():
            if args.check:
                if not path.exists() or path.read_text(encoding="utf-8") != content:
                    stale.append(path)
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
        if stale:
            for path in stale:
                print(f"stale generated file: {path.relative_to(ROOT)}", file=sys.stderr)
            return 1
        print("SensorKit wire schema is valid" if args.check else "Generated SensorKit wire artifacts")
        return 0
    except (OSError, ValueError, ValidationError) as exc:
        print(f"wire schema error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
