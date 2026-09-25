"""Validate or generate artifacts for a configured .nkw wire schema."""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from pathlib import Path

from .parser import parse_file
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
        current = normalized(parse_file(schema_path))
        if lock_path.exists():
            validate_evolution(current, json.loads(lock_path.read_text(encoding="utf-8")))
        elif args.command != "generate":
            raise ValidationError(f"schema compatibility lock is missing: {lock_path}")
        if args.command == "validate":
            print(f"Valid wire schema: {schema_path}")
            return 0
        # Transitional consumer generator. Backends move into tools/wire in the
        # next slice; the shared driver already owns schema/lock validation.
        generator = importlib.import_module(config["generator"])
        return generator.main(["--schema", str(schema_path)] + (["--check"] if args.command == "check" else []))
    except (KeyError, OSError, ValueError, ValidationError, ImportError) as exc:
        print(f"wire schema error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
