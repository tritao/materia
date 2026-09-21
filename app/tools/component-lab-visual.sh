#!/usr/bin/env bash
set -euo pipefail

app_dir=$(cd "$(dirname "$0")/.." && pwd)
repo_dir=$(cd "$app_dir/.." && pwd)
mode=${1:-capture}
artifacts="$app_dir/build/component-lab-visual"
goldens="$app_dir/tests/golden/component-lab"

stories=(
	button/states
	text-field/editing
	selection/controls
	controls/value
	text/multiline
	navigation/tabs
	layout/split-view
	design/tokens
)

capture_story() {
	local story=$1
	local name=${story//\//-}
	"$repo_dir/haxeon/scripts/haxeon" run --project "$app_dir/haxeon.json" -- \
		--story="$story" --capture-dir="$artifacts/$name" --frames=3
}

mkdir -p "$artifacts"
for story in "${stories[@]}"; do
	capture_story "$story"
done

case "$mode" in
	capture)
		echo "Component Lab captures: $artifacts"
		;;
	update-goldens)
		mkdir -p "$goldens"
		for story in "${stories[@]}"; do
			name=${story//\//-}
			cp "$artifacts/$name/frame.png" "$goldens/$name.png"
		done
		echo "Component Lab goldens updated: $goldens"
		;;
	compare)
		command -v compare >/dev/null || {
			echo "ImageMagick compare is required" >&2
			exit 2
		}
		failures=0
		mkdir -p "$artifacts/diffs"
		for story in "${stories[@]}"; do
			name=${story//\//-}
			expected="$goldens/$name.png"
			actual="$artifacts/$name/frame.png"
			diff="$artifacts/diffs/$name.png"
			if [[ ! -f "$expected" ]]; then
				echo "Missing golden: $expected" >&2
				failures=$((failures + 1))
			elif ! compare -metric AE "$expected" "$actual" "$diff" 2>"$artifacts/diffs/$name.txt"; then
				echo "Visual mismatch: $story" >&2
				failures=$((failures + 1))
			fi
		done
		exit "$failures"
		;;
	*)
		echo "Usage: $0 [capture|compare|update-goldens]" >&2
		exit 2
		;;
esac
