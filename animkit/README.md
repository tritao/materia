# AnimKit

AnimKit imports glTF 2.0 characters and props and poses them with
[ozz-animation](https://github.com/guillaumeblanc/ozz-animation). It is the
animation layer that a future HumanKit will build on: one skeleton drives the
visible mesh, and later the collision proxy, landmarks, and attachments too.

The package has three parts:

- `native/`: the `animkit_core` library with a C ABI (`include/animkit.h`).
  It parses glTF with cgltf, decodes PNG/JPEG textures with stb_image, builds
  ozz skeletons and clips, and skins meshes on the CPU.
- `haxe/animkit`: Haxe wrappers. `AnimationAsset` loads a file,
  `AnimationInstance` blends up to four clip layers and reads the deformed
  streams, and `ClipPlayer` plays one clip at a time with crossfades.
- `haxe/animkit/scene`: `SkinnedModel`, which presents an instance in a
  SceneKit scene and republishes its geometry after each evaluation.

```haxe
var asset = AnimationAsset.load("animkit/assets/kenney/character-soldier.glb");
var instance = new AnimationInstance(asset);
var model = new SkinnedModel(scene, instance);
var player = new ClipPlayer(instance);
player.playNamed("walk");

// Each frame:
player.advance(seconds);
model.update();
```

## Import model

- **Every node is a joint.** Each node in the glTF default scene becomes an ozz
  joint named after the node. Skinned meshes, rigid meshes parented to bones,
  and animated props therefore share one pose, and joint names match the
  source file for later rig mapping.
- **Rigid meshes.** A mesh without skin weights follows its own node.
- **Scene space.** glTF is +Y up with models facing +Z. AnimKit outputs
  SceneKit space: +Z up, +X forward, metres. glTF +X, +Y, and +Z map to scene
  +Y, +Z, and +X. The conversion is applied to the joint palette, so source
  data and clips stay untouched.
- **Clips.** Each glTF animation becomes an ozz clip. Linear keys are kept.
  Step keys become hold keys. Cubic-spline keys keep their values but drop
  tangents. ozz quantizes keys, so sampled poses agree with the source to well
  under a millimetre.
- **Materials.** Metallic-roughness factors, alpha mode, double-sidedness, and
  the base color texture on `TEXCOORD_0` are imported. External images are
  resolved beside the file.
- **Not yet supported.** Morph targets and morph-weight channels are skipped
  with a warning, as are non-triangle primitives, other texture slots,
  `KHR_texture_transform`, and KTX2/WebP images. Each asset reports what it
  skipped in `warnings`.

Skinning runs on the CPU. SceneKit has no joint or weight vertex streams yet,
so each frame republishes positions and normals with `setGeometryData`. That
is cheap for a few low-poly characters. GPU skinning is the next step once
profiling asks for it.

## Build and test

```sh
cmake -S animkit/native -B animkit/native/build -GNinja -DCMAKE_BUILD_TYPE=Debug
cmake --build animkit/native/build
ctest --test-dir animkit/native/build -L animkit --output-on-failure
./haxeon/scripts/haxeon run --project animkit/tests/haxeon.json
```

The native test builds a small skinned glTF in memory. It checks rigid and
skinned deformation, looping, clamping, step keys, layer blending, the axis
conversion, and error handling. The Haxe test drives the bundled CC0 Kenney
soldier through a real scene.

After changing `include/animkit.h`, regenerate the committed FFI interface:

```sh
animkit/native/tools/check-hxi.sh
```

## Editor preview

The reference editor can show an animated character walking a circle around
the origin. The preview is runtime-only and is not saved with the document:

```sh
./app/run-built.sh --character=animkit/assets/kenney/character-soldier.glb
./app/run-built.sh --character=path/to/worker.glb --character-clip=Idle
```

Clips whose names contain "walk" move along the circle; other clips play in
place.

## Assets

`assets/kenney/` holds Kenney's Mini Arena soldier (CC0; see its
`License.txt`). It is used by the tests and the editor preview.
