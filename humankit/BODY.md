# How a worker's body is put together

A worker's pose is built in layers. Each layer owns one kind of decision, and a later layer may
overwrite an earlier one only through the interface below it.

1. **Clip.** `ClipPlayer` blends the animation: idle, walk, and the crossfades between them. It knows
   nothing about jobs. Clips that start during a fade continue from the blend they were in.
2. **Joint turns.** `HumanCharacter` turns single joints on top of the clip, before IK: `setSpineLean`
   bends the upper body forward and `setHandCurl` curls the fingers (`HumanHand`). A turn rides on the
   animation and carries the joint's children. Each feature applies its turns under its own source
   (`HumanCharacter.LEAN`, `LEFT_FINGERS`, `RIGHT_FINGERS`): a source has one turn per joint and the
   turns of different sources on one joint compose, lowest source first, so a new feature that turns a
   joint another already turns adds to it instead of overwriting it. A feature sets all its turns in one
   batched call.
3. **IK.** `HumanCharacter.reach` solves a limb's wrist or ankle to a target, bending the joint the way
   the animation does (a zero pole), so the elbow cannot flip as the hand passes the shoulder.
4. **Body.** `HumanBody` is what actions talk to. It owns each limb's role, the lean, and the fingers'
   curl, and it steps them every tick.
5. **Proxies.** `HumanBodyProxy` and `humankit-sim` turn the final pose into collision capsules.

## Limbs are small state machines

`LimbControl` keeps, per limb, what it was asked to do (`LimbMode`: `Free`, `Reach`, `Carry`) and what
the character's IK holds now (`applied`). `apply` reconciles the two and is the only place that calls
`reach` or `release` on the character. `Hang` is not a mode anyone asks for: a `Free` arm is held
hanging under its shoulder while the body leans, because a lean swings an unused arm back with the
torso. Since "what is applied" is one recorded value, there is no flag that can go stale.

To add a limb behaviour, add a `LimbMode`, handle it in `LimbControl.apply`, and let an action ask for
it through `HumanBody`. Do not call the character's IK from an action.

## Tuning lives in one place

Every number that shapes how a worker holds itself is a field of `HumanPosture`, in metres and radians,
tuned on the bundled 1.7 m rig: reach comfort, carry offset, finger curls, lean limits, hang target,
belly depth, withdrawal. Build a `HumanBody` with a modified posture to change it. What a rig's bones
are (which joint curls the fingers, where the spine pitches) is not tuning: it is read from the skeleton
or measured, as `HumanBody.leanFor` measures how far a lean moves the shoulder.

## Actions plan; the body performs

`ApproachFor` decides where to stand: the shoulder a comfortable reach from the point, and, given the
surface the point rests on (`from` on a pick, `onto` on a place), far enough back that the belly clears
that surface's edge, leaning the upper body over it for the rest of the reach. `Pick` and `Place` reach
and grasp, and ask the body to straighten up when they are done. Actions set intent (`setLean`,
`setCarry`, `setReachWorld`); they never touch joints.

## Keeping it natural

`MotionQuality` samples a pose each tick and reports, per arm, the bend-plane turn rate, the elbow's
height against the shoulder, its sharpest bend, and the wrist's speed and acceleration.
`humankit/sim/tests` holds the rack job to limits just above ordinary gait, and measures how far the
belly stays from the surface it reaches over. A change that flips an arm or snaps a pose in one tick
fails there with the sample at which it happened. Add a scenario there when a new behaviour has a way
to go wrong that the rack job does not exercise.

## Known limits

- **Low surfaces.** At about a metre or lower the lean reaches its limit before the belly clears the
  surface's edge, leaving it 3 to 4 cm inside for either hand. Reaching that low needs a crouch, which
  the body does not do. The layout sweep (`ScenarioSweepTests`) records this as its `CAPPED_CLEARANCE`.
- **Tuning is for one rig.** `HumanPosture` is in metres, tuned on the bundled 1.7 m worker; it is not
  scaled to other statures. The finger-curl axis and the spine shares are checked on that rig only.
- **Both hands and the left hand** share the same code as the right, and the sweep covers a left-hand
  fetch, but two-handed lean and hang are exercised by far fewer scenarios.
