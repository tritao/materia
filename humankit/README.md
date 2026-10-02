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

## Layout

`haxe/humankit/` holds the character, body, walker, job and proxy types that the app and the simulation use. Three groups that stand apart
have their own subpackage: `rig/` (the skeleton and pose data: `HumanBone`, `HumanoidRig`, `RigMapping`, `HumanPose`, `Mat4`), `action/` (the
job actions and the planner: `Pick`, `Place`, `Press`, `Carry`, `Reach`, `WalkTo`, `ApproachFor`, and the rest), `job/` (the job document and what
builds it into actions: `HumanJob`, `HumanJobSpec`, `HumanJobBuilder` and their result, hold and target types) and `quality/` (`MotionQuality`,
`Naturalness`). `humankit.sim` is the physics worker and `humankit.facility` the AutomationKit adapters, each in its own folder.

## Standard skeleton

`HumanBone` names about 30 bones: pelvis, spine segments, neck, and head, plus
shoulder, upper arm, forearm, hand, three knuckles, thigh, shin, foot, toe, and
toe tip on each side. The names describe anatomy only. An asset's own hierarchy may
differ; the Quaternius rig, for example, parents its feet to the body as IK
controls rather than to the shins.

`RigMapping` maps each standard bone to an asset joint name, optionally with
alternatives tried in order (Quaternius rigs without toe bones end the foot at
`Foot.L_end`). Presets cover Quaternius and Mixamo; Mixamo namespaces such as
`mixamorig:` are ignored.
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

## Walking and reaching

`HumanGait.measure` finds a looping walk clip's natural ground speed: a
planted foot slides backwards past the pelvis at exactly the speed the body
travels. `HumanWalker` walks a character along a route of floor points at a
chosen speed, playing the walk clip at the rate that keeps planted feet still;
it speeds up over the crossfade from idle, brakes to stop at the end of an
open route, turns at a bounded rate, and idles on arrival. `rootTransform()`
is where to place the character.

`HumanCharacter.reach(limb, target)` puts a wrist or ankle on a model-space
target with AnimKit's two-bone IK, on top of whatever clip is playing; elbows
point down and back and knees forward unless a pole is given, and `release`
returns the limb to its animation. Legs need a foot below the shin, which
Quaternius rigs lack (their feet are IK controls parented to the body).

`HumanReachTask` sequences the two: it walks a `HumanWalker` along an open
route and, once the body stops, reaches a limb for a target, ramping the IK
weight in over `rampSeconds`, holding it at full weight for `holdSeconds`
(optionally playing a clip such as `"interact"`), then ramping it back out and
releasing. Drive it the same way as a walker: construct it, then call
`advance(seconds)` every frame until `isDone()`.

## Walking an AutomationKit facility route

HumanKit does not depend on RobotKit or AutomationKit. `humankit/facility`
(the `humankit-facility` package) is the small bridge: `FacilityWalk`
converts an AutomationKit `FacilityRoute`, or any RobotKit navigation `Path`,
into the floor points `HumanWalker.follow` takes, collapsing waypoints closer
together than a millimetre so the route never ends in a zero-length segment.

```haxe
var route = new FacilityRouter(facility).route("dock", "bench");
var points = FacilityWalk.routeFromFacilityRoute(route);
walker.follow(points, route.maximumSpeedMetersPerSecond);
```

The walker ends at the route's last waypoint facing along its last segment,
so a person following a facility route arrives at the destination station
facing along the last lane, not the straight line from the start.

## Body description and collision proxy

`HumanDescription.measure` reads stature, shoulder and hip width, torso, and
limb segment lengths from a rest pose. `scaledTo` describes the same body at
another stature; its `scale` is then the uniform scale for the character's
visual root.

`HumanBodyProxy.standard` stands the body in with fifteen capsules between
standard bones: head, abdomen, chest, and upper arm, forearm, hand, thigh,
shin, and foot on each side. Radii are proportions of stature and lengths
follow the rig's bones. The head reaches the crown, shins stop short of the
ankle so they never sink below the sole, and feet run to the toe tips.
`place(pose, root)` returns every capsule's centre and orientation (the
capsule's local +Z along its bones) through the character's root transform.

`HumanBodyView` draws the capsules or a skeleton of thin bones beneath the
character root, as a `HumanDisplay` of `Mesh`, `Capsules`, or `Skeleton`, to
check the proxy against the mesh.

How the body is layered, who owns what, and where its tuning lives are described in
[BODY.md](BODY.md).

## Characters

The bundled Quaternius worker (`animkit/assets/quaternius/worker.glb`) is the default. The Universal
Animation Library character (`animkit/assets/quaternius-ual/ual-standard.glb`, CC0, 46 clips) is also
supported: its rig matches the `universal` preset, its legs are real chains, and it has a crouch clip, so it
reaches surfaces from half a metre up (see `BODY.md`). Pick it in the editor with the worker's "Character
asset" property: `ual-standard.glb` is the free pack's 46 clips, `ual-work.glb` adds the purchased library's
clips for a worker (carrying walk, turns, crouch enter and exit, kneeling, counters, pushing, sitting). Where each
clip came from is in `animkit/assets/quaternius-ual/PROVENANCE.md`. A character with a `Walk_Carry_Loop` clip
walks with it while its hands carry something (`HumanWalker.setCarrying`, set by `HumanBody.setCarry`).

## Document jobs

`HumanJobSpec.parse` accepts strict, versioned JSON. Version 1 has `version`,
`loop`, and an ordered `steps` array. Steps refer to scene object IDs, so a
saved job can be reopened without Haxe code:

```json
{"version":1,"loop":false,"steps":[
  {"action":"pick","object":"part","hand":"right","from":"rack"},
  {"action":"place","onto":"table","hand":"right"}
]}
```

Available actions are `walkTo` (an object or XY point, with optional `via`
points), `pick`, `place` (optional XY `offset` on the support and
`retreat: "backward"` to keep facing the part for the first step away), `press` (object
anchor or XYZ point), `wait` (seconds), and `playClip` (clip and seconds).
Pick and place hands can be `left`, `right`, or `both`. A pick may name the
surface the object rests on with `from`, as a place names its `onto`: the worker
then stands clear of that surface's edge and leans the upper body over it to keep
the reach comfortable, instead of standing in the edge. The parser rejects unknown fields,
invalid pick and place sequences, and unsupported versions. `toJson()` writes
a normalized spec. Parsed steps use the typed `HumanJobStep` shape; the parser
enforces action-specific fields before publishing them.

`HumanJobBuilder.build(spec, targets, body)` resolves scene boxes through a
`HumanJobTargets` adapter and returns a `HumanJob` plus hold bindings. It
places walk stops outside object footprints and computes pick and place
points from box tops. The simulation layer binds those holds to dynamic
objects; see [the simulation README](sim/README.md).

## Simulation

The `humankit/sim` package (`humankit-sim`) puts a person in a SimKit session:
`HumanActor` turns the proxy into one kinematic session actor and
`pushPose(time, pose, root)` adds a keyframe in simulation time. The person
pushes what they walk into and is never pushed back. A writer that steps the
session pushes one keyframe per tick; one following a realtime session pushes a
few ticks ahead, and the session interpolates between keyframes.

`HumanWorker` reports distance from each worker capsule to the supplied robot
link sphere. Its caller supplies the link pose and an approximate collision
radius. `ApproachFor` stands on the line from the current root to its target;
it does not offset the stance for the active arm's shoulder, so a one-arm reach
can cross the torso slightly.
Holding an object by a moving robot link uses that link's pose from the last
session tick, so the object can trail the gripper by one tick.

## Tests

```sh
./haxeon/scripts/haxeon run --project humankit/tests/haxeon.json
```

The test uses the bundled Quaternius worker and CreativeTrio wrench (both
CC0). It checks rig detection and rejection, anatomical landmarks, rigid hand
frames, and that an attached prop tracks the palm through a walk cycle. It
also checks that the prop adds exactly one draw call, the measured
description, capsule placement against the bones, crown, and floor, uniform
scaling, that the body view draws only the shown mode and follows the pose,
that a gait-matched walk keeps a planted foot within a quarter of the body's
travel and stops, turned, at the end of its route, and that a reach puts the
wrist on its target, bends the elbow down, straightens towards an unreachable
target, blends by weight, and releases cleanly. It also drives a real
`FacilityRouter` route through two lanes with `FacilityWalk` (the person ends
at the destination station, facing along the last lane) and a `HumanReachTask`
against the worker (the wrist reaches the target while the "interact" clip
plays, the body stops at the spot, and the release lets go cleanly).
The app suite covers a person joining the application simulation.

## Editor preview

```sh
./app/run-built.sh --perspective --character=animkit/assets/quaternius/worker.glb \
  --character-hold=animkit/assets/props/wrench.glb [--character-display=capsules|skeleton] \
  [--character-route=0,-2;2,-2;2,1] \
  [--character-reach=0.3,-0.2,1.0 [--character-reach-clip=interact]]
```

Humanoid characters become HumanKit characters; other assets still preview as
plain AnimKit models. A humanoid preview also joins the application
simulation as a person: once a simulation is applied it lives on simulation
time, stands still while the simulation is paused, and walks with each step.
It walks the given route once and idles at its end, or loops round a circle.

`--character-route=X,Y;X,Y;...` gives the route directly, as floor points.
`--character-facility-route=FROM,TO` instead routes a real `FacilityRouter`
route through a small built-in demo facility (stations `dock`, `shelf`, and
`bench`, `app/src/editor/FacilityRouteDemo.hx`), adapted with `FacilityWalk`;
the two are mutually exclusive. Either way, the character walks the resulting
open route once, arriving at its last point facing its last segment.

`--character-reach=X,Y,Z` needs an open route (`--character-route` or
`--character-facility-route`): once the walk arrives, the character reaches
its right hand for the model-space target with a `HumanReachTask`, holding it
briefly with `--character-reach-clip=NAME` playing (default: none) before
releasing.
