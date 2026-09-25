"""Plan deterministic generated outputs without writing them."""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any

from .backends.cpp import cpp_header, codec_cpp, codec_header
from .backends.haxe import haxe_files
from .backends.markdown import docs
from .backends.packed_cpp import render as packed_cpp
from .backends.rust import render as packed_rust
from .model import Schema
from .validate import ValidationError, dump_normalized


def plan(schema: Schema, norm: dict[str, Any], config: dict[str, Any], root: Path) -> dict[Path, str]:
    """Return artifacts keyed by absolute path under the manifest directory."""
    outputs: dict[Path, str] = {}

    def add(relative: str, content: str) -> None:
        path = (root / relative).resolve()
        if not path.is_relative_to(root):
            raise ValidationError(f"output escapes manifest directory: {relative}")
        if path in outputs:
            raise ValidationError(f"duplicate output path: {relative}")
        outputs[path] = content

    paths = config.get("outputs", {})
    if "haxe" in config:
        options = config["haxe"]
        haxe_root = (root / options["root"]).resolve()
        if not haxe_root.is_relative_to(root):
            raise ValidationError("Haxe output root escapes manifest directory")
        for path, content in haxe_files(
            schema, norm, root=haxe_root, package=options["package"],
            codec_name=options["codec_name"], constants_name=options["constants_name"],
            schema_label=options["schema_label"], codec_comment=options["codec_comment"],
        ).items():
            add(str(path.relative_to(root)), content)
    if "cpp" in config:
        options = config["cpp"]
        add(paths["cpp_types"], cpp_header(schema, norm, namespace=options["namespace"], aliases=options.get("aliases", {})))
        add(paths["cpp_codec_header"], codec_header(schema, namespace=options["namespace"], reader_namespace=options["reader_namespace"], generated_header=options["generated_header"]))
        add(paths["cpp_codec_source"], codec_cpp(schema, norm, namespace=options["namespace"], reader_namespace=options["reader_namespace"], codec_header_name=options["codec_header"], msgpack_header=options["msgpack_header"]))
    if "packed_cpp" in config:
        add(paths["packed_cpp"], packed_cpp(schema, norm, config["packed_cpp"]["namespace"]))
    if "rust" in config:
        add(paths["rust"], packed_rust(schema, norm))
    if "markdown" in config:
        options = config["markdown"]
        add(paths["markdown"], docs(schema, norm, title=options["title"], intro=options["intro"], codec_name=config.get("haxe", {}).get("codec_name", "wire codec")))
    if "vectors_module" in config:
        vectors = importlib.import_module(config["vectors_module"])
        add(paths["vectors"], vectors.vector_fixture(norm))
        add(paths["haxe_vectors"], vectors.haxe_vector_check(norm))
    add(config["lock"], dump_normalized(norm))
    return outputs
