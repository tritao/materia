#!/usr/bin/env bash
# Builds and installs the pinned OCCT once into the shared cache that cadkit/CMakeLists.txt searches, so no checkout
# has to compile it (about an hour). Does nothing when the prefix already exists.
#
#   cadkit/scripts/build-occt-prefix.sh [--jobs N] [--print-prefix]
#
# The prefix is <checkout>/../materia-cache/occt-<revision>-release, where <revision> is the last commit that touched
# cadkit/third_party/occt. Set MATERIA_CACHE_DIR to use another cache directory.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cache="${MATERIA_CACHE_DIR:-$repo_root/../materia-cache}"
jobs="$(nproc)"
print_only=0
while [[ $# -gt 0 ]]; do
	case "$1" in
		--jobs) jobs="${2:?--jobs needs a count}"; shift 2 ;;
		--print-prefix) print_only=1; shift ;;
		*) echo "unknown argument: $1" >&2; exit 2 ;;
	esac
done

revision="$(git -C "$repo_root" log -1 --abbrev=8 --format=%h -- cadkit/third_party/occt)"
if [[ -z "$revision" ]]; then
	echo "cannot determine the pinned OCCT revision from git history" >&2
	exit 1
fi
prefix="$cache/occt-$revision-release"
if [[ "$print_only" == 1 ]]; then
	echo "$prefix"
	exit 0
fi
# The install's identity: the content of its libraries, which is what a cached CadKit result depends on. NativeIdentity reads it.
stamp_prefix() {
	(cd "$prefix/lib" && find . -maxdepth 1 -name 'libTK*' ! -type l | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -d' ' -f1) > "$prefix/occt.identity"
	echo "OCCT identity $(cat "$prefix/occt.identity")"
}
if [[ -d "$prefix/lib/cmake/opencascade" ]]; then
	[[ -f "$prefix/occt.identity" ]] || stamp_prefix
	echo "OCCT $revision is already installed at $prefix"
	exit 0
fi

build="$cache/build-occt-$revision-release"
mkdir -p "$cache"
echo "Building OCCT $revision into $prefix (build tree: $build)"

# The same toolkit selection as cadkit/CMakeLists.txt uses when it compiles OCCT itself.
cmake -S "$repo_root/cadkit/third_party/occt" -B "$build" -G Ninja \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_INSTALL_PREFIX="$prefix" \
	-DCMAKE_INSTALL_RPATH='$ORIGIN' \
	-DBUILD_LIBRARY_TYPE=Shared \
	-DBUILD_MODULE_ApplicationFramework=OFF \
	-DBUILD_MODULE_DataExchange=OFF \
	-DBUILD_MODULE_Draw=OFF \
	-DBUILD_MODULE_Visualization=OFF \
	-DBUILD_MODULE_FoundationClasses=OFF \
	-DBUILD_MODULE_ModelingData=OFF \
	-DBUILD_MODULE_ModelingAlgorithms=OFF \
	-DBUILD_ADDITIONAL_TOOLKITS="TKBO;TKDE;TKDESTEP;TKFillet;TKOffset;TKMesh;TKPrim;TKTopAlgo;TKXSBase" \
	-DBUILD_DOC_Overview=OFF \
	-DBUILD_DOC_RefMan=OFF \
	-DBUILD_RESOURCES=OFF \
	-DINSTALL_TEST_CASES=OFF \
	-DUSE_FREETYPE=OFF \
	-DUSE_OPENGL=OFF \
	-DUSE_XLIB=OFF
cmake --build "$build" --parallel "$jobs"
cmake --install "$build"
stamp_prefix
echo "Installed OCCT $revision at $prefix"
