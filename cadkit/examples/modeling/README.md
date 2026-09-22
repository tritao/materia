# Modeling with CadKit

`MountingPlate.hx` builds an 80 × 50 × 6 plate from a sketch, subtracts four
radius-3 holes on a 60 × 30 grid, extrudes it, selects the four outer vertical
edges, and applies radius-2 fillets. Its `build()` method returns an owning
`Part`. Its `main()` writes `mounting-plate.step` in the working directory.

The example is compiled and exercised by `./scripts/test-haxeon`. To run it
standalone after building CadKit:

```sh
repo_root="$PWD"
haxeon_root="$repo_root/../haxeon"
"$haxeon_root/.tools/haxe/haxe" --cwd "$haxeon_root" -cp src \
  --run compiler.tools.HaxeonCompiler --target=hl \
  "--output=$repo_root/build/haxeon/mounting-plate.hl" --entry=MountingPlate \
  "--root=$repo_root/haxe/src" "--root=$repo_root/examples/modeling" \
  "--ffi-interface=$repo_root/haxe/abi/CadKit.hxi" \
  "--ffi-projection=$repo_root/haxe/abi/CadKit.hxmap" \
  "$repo_root/examples/modeling/MountingPlate.hx"
LD_LIBRARY_PATH="$repo_root/build/debug/core:$repo_root/build/debug/lin64/gcc/libd:$haxeon_root/out:$haxeon_root/.tools/hashlink" \
  "$haxeon_root/.tools/hashlink/hl" "$repo_root/build/haxeon/mounting-plate.hl"
```

See [the modeling API](../../haxe/MODELING.md) for ownership, selections,
workplanes, additional operations, and document integration.
