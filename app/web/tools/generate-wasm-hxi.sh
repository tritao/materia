#!/usr/bin/env bash
# Generates the wasm32 (portable-abi32) Haxe FFI interfaces the browser build
# compiles against, into OUTPUT_DIR.
#
# Each kit already owns a script that audits its C import header and writes the
# desktop (portable-abi64) interface. Rather than duplicating their header,
# include and dependency lists, this runs those scripts against a shim
# haxeon-ffi-audit that swaps the desktop targets for the Emscripten/WASI
# targets and points dependency interfaces at the wasm32 copies written here.
set -euo pipefail

if [[ $# -ne 1 ]]; then
	echo "usage: app/web/tools/generate-wasm-hxi.sh OUTPUT_DIR" >&2
	exit 2
fi

materia_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
haxeon_dir=${HAXEON_DIR:-"$materia_dir/haxeon"}
mkdir -p "$1"
output_dir=$(cd "$1" && pwd)
shim_dir=$(mktemp -d)
trap 'rm -rf -- "${shim_dir:?}"' EXIT

mkdir -p "$shim_dir/scripts"
cat > "$shim_dir/scripts/haxeon-ffi-audit" <<SHIM
#!/usr/bin/env bash
set -euo pipefail
arguments=(--target=wasm32-unknown-wasi --target=wasm32-unknown-emscripten --profile=portable-abi32)
for argument in "\$@"; do
	case \$argument in
		--target=* | --profile=*) ;;
		--dependency-hxi=*.hxi)
			arguments+=("--dependency-hxi=$output_dir/\$(basename "\${argument#--dependency-hxi=}")") ;;
		*) arguments+=("\$argument") ;;
	esac
done
exec "$haxeon_dir/scripts/haxeon-ffi-audit" "\${arguments[@]}"
SHIM
chmod +x "$shim_dir/scripts/haxeon-ffi-audit"

# Dependencies come first: later interfaces name earlier ones with --dependency-hxi.
generators=(
	"nativekit/tools/audit-haxeon-abi.sh nativekit.hxi --output="
	"nativekit/modules/gpu/tools/check-hxi.sh nativekit-gpu.hxi"
	"uikit/tools/check-hxi.sh nativekit-ui.hxi"
	"scenekit/scene/tools/check-hxi.sh nativekit-scene.hxi"
	"scenekit/scene_render/tools/check-hxi.sh nativekit-scene-render.hxi"
	"simkit/sim_core/tools/check-hxi.sh nativekit-sim.hxi"
	"simkit/sim_mujoco/tools/check-hxi.sh nativekit-sim-mujoco.hxi"
	"robotkit/runtime/tools/check-hxi.sh robotkit-runtime.hxi"
	"robotkit/runtime/tools/check-simkit-hxi.sh robotkit-simkit.hxi"
	"robotkit/policy/tools/check-hxi.sh robotkit-policy.hxi"
	"robotkit/inference/tools/check-hxi.sh robotkit-inference.hxi"
	"visionkit/native/tools/check-hxi.sh visionkit.hxi"
	"animkit/native/tools/check-hxi.sh animkit.hxi"
	"stockkit/core/tools/check-hxi.sh stockkit.hxi"
)
for generator in "${generators[@]}"; do
	read -r script name prefix <<<"$generator"
	# NativeKit's audit runs from its repository root and takes --output=PATH.
	(cd "$materia_dir/$(dirname "$(dirname "$script")")" &&
		HAXEON_DIR="$shim_dir" bash "$materia_dir/$script" "${prefix:-}$output_dir/$name") >/dev/null
	echo "generate-wasm-hxi: $name"
done

# CadKit's ABI uses only fixed-width scalars and generated layouts, so it is
# imported for one target and published under the portable profile, like its
# desktop interface (cadkit/scripts/generate-haxeon-hxi).
cadkit_import=$(mktemp)
"$haxeon_dir/scripts/haxeon-ffi-import" --target=wasm32-unknown-emscripten --library=cadkit-core \
	--interface=CadKit --source-label=core/include/cadkit.h --output="$cadkit_import" \
	"$materia_dir/cadkit/core/include/cadkit.h" >/dev/null
sed -e "1s/for wasm32-unknown-emscripten/for portable-abi32/" \
	-e '2s/@target("wasm32-unknown-emscripten")/@target("portable-abi32")/' \
	"$cadkit_import" > "$output_dir/CadKit.hxi"
rm -f -- "${cadkit_import:?}"
echo "generate-wasm-hxi: CadKit.hxi"
