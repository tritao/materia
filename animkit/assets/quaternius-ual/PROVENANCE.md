# Provenance of the Universal Animation Library clips

Source: Quaternius, *Universal Animation Library* and *Universal Animation Library 2* (CC0). The full libraries were
purchased by the project. The character and the 46 free clips are the free Standard glTF (`ual-standard.glb`).

## What is here

| Path | What it is |
|---|---|
| `ual-standard.glb` | The free Standard glTF, unchanged apart from packing its `.bin` into the file. |
| `clips/UAL1/*.glb` | Clips of library 1 that the free glTF lacks, one file each. |
| `clips/UAL2/*.glb` | Clips of library 2, one file each. |
| `clips.json` | One entry per clip: name, library, what it was read from, length, SHA-256 of its file, the retargeting. A clip the free glTF carries is listed with `"file": null`; the free file is its original and is used as it is. |
| `ual-work.glb` | The runtime library: `ual-standard.glb` plus every file in `clips/`. Generated; do not edit. |

Clip names are the library's own (`Crouch_Enter`, `Walk_Carry_Loop`, ...), so a clip is found by the name the author gave it. The clip
file records the same provenance in `asset.extras` and `animations[0].extras`.

## How a clip was made

The libraries are published for Godot: their animations are Godot `Animation` resources on a skeleton with Godot's
humanoid bone names and a T-pose rest, packed in the Godot pack of the library's viewer
(`UAL1_Source.res`, `UAL2_Source.gltf`). `tools/ual/extract_clips.py` reads them from that pack and writes each as a glTF clip
on the free glTF's rig (`DEF-` bones):

1. **Rotations.** For a bone `b` with parent `p`, `local_gltf(b) = D(p)^-1 * local_godot(b) * D(b)`, where `D` is a constant rotation
   per bone (the two skeletons agree on the animated world pose up to it). Every `D` is solved from `PickUp_Table`, which the free
   glTF and the Godot library both carry, and checked against that clip: median error 0.12 degrees over 52 bones, worst 8.6 degrees
   (the Godot form is resampled to 30 fps, the free one is 24 fps).
2. **Hips translation.** The glTF rig is the Godot rig at 0.917 of its size, so a Godot hips translation is rotated into the glTF
   frame and scaled by 0.917, a factor read off `Idle_Loop` (also in both forms), not assumed.
3. **Not carried over.** Tracks for bones the free rig lacks and any track that is neither rotation nor translation.

## Rebuilding

```sh
python3 animkit/tools/ual/extract_clips.py --pck <viewer index.pck> --free <free ual.gltf> \
    --out animkit/assets/quaternius-ual/clips --manifest animkit/assets/quaternius-ual/clips.json <clip names...>
python3 animkit/tools/ual/build_library.py --base animkit/assets/quaternius-ual/ual-standard.glb \
    --manifest animkit/assets/quaternius-ual/clips.json --clips animkit/assets/quaternius-ual/clips \
    --out animkit/assets/quaternius-ual/ual-work.glb
```

`build_library.py` refuses a clip whose file does not match its manifest checksum. The clips chosen are the ones a worker needs:
locomotion, turns, crouching, kneeling and floor work, counters and stations, pushing, sitting, and a few idles.
