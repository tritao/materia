#!/usr/bin/env python3
"""Generate SceneKit's backend shaders from its canonical GLSL stages."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SHADERS = ROOT / "shaders"
OUTPUT = ROOT / "src/scene_shader_sources.h"
PROGRAMS = ("scene", "stroke", "workplane", "pick", "postprocess")
SLANGS = {"glsl410": "glsl", "glsl300es": "glsl", "hlsl5": "hlsl",
          "metal_macos": "metal"}
# These blocks match the data uploaded by gpu_executor.cpp.
BLOCKS = {
    ("scene", "vertex"): ((1, "view_params", "mat4 view_projection"),),
    ("scene", "fragment"): (
        (0, "material_uniforms", "vec4 base_color; vec4 material_params; vec4 emissive; vec4 texture_flags"),
        (3, "lighting_uniforms", "vec4 light_position_type[32]; vec4 light_direction_range[32]; "
                                 "vec4 light_color_intensity[32]; vec4 light_cones[32]; "
                                 "vec4 ambient_sky; vec4 ambient_ground; vec4 lighting_mode; "
                                 "vec4 camera_position; vec4 camera_view_direction"),
        (2, "clip_params", "vec4 clip_planes[32]; vec4 clip_plane_count")),
    ("stroke", "vertex"): ((1, "view_params", "mat4 view_projection"),
                             (3, "stroke_view_params", "vec4 stroke_view")),
    ("stroke", "fragment"): ((0, "stroke_color_params", "vec4 stroke_color"),
                               (2, "clip_params", "vec4 clip_planes[32]; vec4 clip_plane_count")),
    ("workplane", "fragment"): ((0, "grid_params", "vec4 grid_eye_spacing; vec4 grid_forward; "
                                                   "vec4 grid_right; vec4 grid_up"),),
    ("pick", "vertex"): ((1, "view_params", "mat4 view_projection"),),
    ("pick", "fragment"): ((2, "clip_params", "vec4 clip_planes[32]; vec4 clip_plane_count"),),
    ("postprocess", "fragment"): ((0, "postprocess_params", "vec4 color_params; "
                                                     "vec4 distortion_params"),
                                      (1, "postprocess_random", "ivec4 random_params")),
}


def stage_source(program: str, stage: str) -> str:
    source = (SHADERS / f"{program}_{stage}.glsl").read_text()
    for placeholder in ("{{VERSION}}", "{{VERTEX_PRECISION}}", "{{FRAGMENT_PRECISION}}"):
        source = source.replace(placeholder, "")
    blocks = []
    for slot, block_name, body in BLOCKS.get((program, stage), ()):
        for declaration in body.split("; "):
            name = "uniform " + declaration.rstrip(";") + ";"
            if name not in source:
                raise RuntimeError(f"missing {name} in {program} {stage}")
            source = source.replace(name, "", 1)
        blocks.append(f"layout(binding={slot}) uniform {block_name} {{ {body}; }};")
    if stage == "fragment":
        textures = re.findall(r"uniform sampler2D (\w+);", source)
        if textures:
            blocks.append(f"layout(binding=0) uniform sampler {program}_sampler;")
        for slot, name in enumerate(textures):
            source = source.replace(f"uniform sampler2D {name};", "", 1)
            blocks.append(f"layout(binding={slot}) uniform texture2D {name}_image;")
            source = re.sub(r"\btexture\(" + name + r"\s*,",
                            f"texture(sampler2D({name}_image, {program}_sampler),", source)
    if re.search(r"\buniform\s+(?:mat4|vec4|ivec4|sampler2D)\b", source):
        raise RuntimeError(f"unmapped uniforms in {program} {stage}")
    return "\n".join(blocks) + "\n" + source


def validate_gl(program: str, variants: dict[str, str], directory: Path, validator: str) -> None:
    for suffix in ("gl", "gles"):
        paths = []
        for stage, extension in (("vertex", "vert"), ("fragment", "frag")):
            path = directory / f"{program}_{suffix}.{extension}"
            path.write_text(variants[f"{program}_{stage}_{suffix}"])
            paths.append(str(path))
        result = subprocess.run([validator, "-l", *paths], capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(f"{program} {suffix} GLSL validation failed:\n"
                               f"{result.stdout}{result.stderr}")


def generate(shdc: str, validator: str) -> bytes:
    symbols: dict[str, str] = {}
    with tempfile.TemporaryDirectory(prefix="scene-shaders-") as temporary:
        directory = Path(temporary)
        for program in PROGRAMS:
            source = (f"@vs {program}_vs\n{stage_source(program, 'vertex')}\n@end\n"
                      f"@fs {program}_fs\n{stage_source(program, 'fragment')}\n@end\n"
                      f"@program {program} {program}_vs {program}_fs\n")
            input_file = directory / f"{program}.glsl"
            input_file.write_text(source)
            slangs = list(SLANGS)
            subprocess.run([shdc, "-i", str(input_file), "-o", str(directory / program),
                            "--slang=" + ":".join(slangs), "-f", "bare", "--no-log-cmdline"],
                           check=True)
            for slang in slangs:
                for stage in ("vertex", "fragment"):
                    path = directory / f"{program}_{program}_{slang}_{stage}.{SLANGS[slang]}"
                    value = path.read_text()
                    suffix = {"glsl410": "gl", "glsl300es": "gles", "hlsl5": "hlsl",
                              "metal_macos": "metal"}[slang]
                    symbols[f"{program}_{stage}_{suffix}"] = value
            validate_gl(program, symbols, directory, validator)
    lines = ["// Generated by scene_render/tools/generate-shaders.py; do not edit.",
             "#pragma once", "", "namespace nkscene::shader_source {"]
    for name, value in sorted(symbols.items()):
        lines.append(f"inline constexpr char {name}[] = {json.dumps(value)};")
    lines.extend(("} // namespace nkscene::shader_source", ""))
    return "\n".join(lines).encode()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shdc", default=os.environ.get("SOKOL_SHDC", "sokol-shdc"))
    parser.add_argument("--validator", default=os.environ.get("GLSLANG_VALIDATOR", "glslangValidator"))
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    try:
        generated = generate(args.shdc, args.validator)
        if args.check:
            if args.output.read_bytes() != generated:
                raise RuntimeError(f"{args.output} is stale; regenerate shaders")
            print("SceneKit shader sources are up to date")
        else:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_bytes(generated)
            print(f"generated {args.output}")
    except (OSError, subprocess.CalledProcessError, RuntimeError) as error:
        print(f"shader generation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
