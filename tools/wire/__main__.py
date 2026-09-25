"""Validate or generate artifacts for a configured .wire.idl wire schema."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .parser import parse_file
from .render import plan
from .validate import ValidationError, normalized, validate_evolution


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("validate", "check", "generate"))
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        config_path = args.config.resolve()
        config = json.loads(config_path.read_text(encoding="utf-8"))
        schema_path = (config_path.parent / config["schema"]).resolve()
        lock_path = (config_path.parent / config["lock"]).resolve()
        schema = parse_file(schema_path)
        current = normalized(schema)
        if lock_path.exists():
            validate_evolution(current, json.loads(lock_path.read_text(encoding="utf-8")))
        elif args.command != "generate":
            raise ValidationError(f"schema compatibility lock is missing: {lock_path}")
        if args.command == "validate":
            print(f"Valid wire schema: {schema_path}")
            return 0
        generated = plan(schema, current, config, config_path.parent)
        if args.command == "check":
            stale = [path for path, content in generated.items()
                     if not path.exists() or path.read_text(encoding="utf-8") != content]
            for path in stale:
                print(f"stale generated file: {path.relative_to(config_path.parent)}", file=sys.stderr)
            return 1 if stale else 0
        for path, content in generated.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
        print(f"Generated wire artifacts for {schema_path}")
        return 0
    except (KeyError, OSError, ValueError, ValidationError, ImportError) as exc:
        print(f"wire schema error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
