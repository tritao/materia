#!/usr/bin/env bash
set -euo pipefail
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
fixture=$(mktemp -d /tmp/materia-module-cache.XXXXXX)
trap 'rm -rf -- "$fixture"' EXIT
cat > "$fixture/haxeon.json" <<'EOF'
{"version":1,"package":{"name":"module-cache-fixture"},"entry":"GraphPreview","sourceRoots":["."],"target":"host"}
EOF
cat > "$fixture/GraphPreview.hx" <<'EOF'
class GraphPreview {public static function preview():haxe.io.Bytes return haxe.io.Bytes.ofString(GraphDependency.value());}
EOF
cat > "$fixture/GraphDependency.hx" <<'EOF'
class GraphDependency {public static function value():String return "first";}
EOF
fingerprint() {
  "$repo_root/haxeon/.tools/haxe/haxe" --cwd "$repo_root/haxeon" -cp src -cp "$repo_root/app/tools" \
    --run MateriaProjectFingerprint "$fixture/haxeon.json" GraphPreview preview \
    "$repo_root/app/tools" "$repo_root/haxeon" false
}
first=$(fingerprint)
test "$first" = "$(fingerprint)"
cat > "$fixture/Unrelated.hx" <<'EOF'
class Unrelated {public static function unused():Int return 1;}
EOF
test "$first" = "$(fingerprint)"
cat > "$fixture/GraphDependency.hx" <<'EOF'
class GraphDependency {public static function value():String return "changed";}
EOF
test "$first" != "$(fingerprint)"
echo 'Module graph cache: repeat stable, unrelated module ignored, transitive dependency invalidates (3 checks).'
