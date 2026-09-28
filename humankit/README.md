# HumanKit

HumanKit turns an AnimKit character into a human: a standard skeleton that
code can address by anatomy, landmarks read from the current pose, and rigid
props that follow bones. It is pure Haxe above AnimKit and SceneKit.

```haxe
var worker = AnimationAsset.load("animkit/assets/quaternius/worker.glb");
var human = new HumanCharacter(scene, worker);        // rig detected from joint names
human.attach(wrench, HumanBone.HandR, HumanGrip.handle(0.12));
human.player.playNamed("walk");

// Each frame:
human.advance(seconds);
var head = human.pose.bonePosition(HumanBone.Head);  // model space, metres
```

## Standard skeleton

`HumanBone` names about 30 bones: pelvis, spine segments, neck, and head, plus
shoulder, upper arm, forearm, hand, three knuckles, thigh, shin, foot, and toe
on each side. The names describe anatomy only. An asset's own hierarchy may
differ; the Quaternius rig, for example, parents its feet to the body as IK
controls rather than to the shins.

`RigMapping` maps each standard bone to an asset joint name. Presets cover
Quaternius and Mixamo; Mixamo namespaces such as `mixamorig:` are ignored.
`HumanoidRig.detect` picks the first preset that provides every required
bone, and reports the missing bones otherwise. Adding another character
source means adding one mapping.

## Pose, landmarks, and hand frames

`HumanPose` reads the instance's joint matrices after each `advance`.
`bonePosition` and `boneFrame` return model-space values in SceneKit's
convention (+Z up, +X forward, metres). Bone frames have any exported scale
removed, so props are never scaled by the rig.

Hands get a rig-independent frame derived once from the rest-pose knuckles.
Its origin is the palm centre, +X points along the fingers, and +Y points to
the thumb side. Grip offsets written against it work on every rig, whatever
axes the asset's wrist bones use. Without knuckles, a hand falls back to its
joint frame.

## Attachments

`HumanCharacter.attach` hangs a rigid glTF prop under a node that follows a
bone, at an offset in that bone's frame. `HumanGrip.handle` builds the
offset for a closed-fist grip: the prop's +X axis runs through the fist along
the thumb side, held at a chosen distance along the prop. Attachments move in
the same `advance` as the mesh, and `changedNodes` lists every node a frame
touched.

## Tests

```sh
./haxeon/scripts/haxeon run --project humankit/tests/haxeon.json
```

The test uses the bundled Quaternius worker and CreativeTrio wrench (both
CC0). It checks rig detection and rejection, anatomical landmarks, rigid hand
frames, and that an attached prop tracks the palm through a walk cycle. It
also checks that the prop adds exactly one draw call.

## Editor preview

```sh
./app/run-built.sh --perspective --character=animkit/assets/quaternius/worker.glb \
  --character-hold=animkit/assets/props/wrench.glb
```

Humanoid characters become HumanKit characters; other assets still preview as
plain AnimKit models.
