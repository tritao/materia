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
torso. Since "what is applied" is one recorded value, there is no flag that can go stale. A free arm eases
into hanging over `HumanPosture.hangSeconds`, and back out, independently of how fast the lean moves; tying
the blend to the lean dragged the arm to its hanging pose in a quarter of a second, mid-swing.

To add a limb behaviour, add a `LimbMode`, handle it in `LimbControl.apply`, and let an action ask for
it through `HumanBody`. Do not call the character's IK from an action.

## Tuning lives in one place

Every number that shapes how a worker holds itself is a field of `HumanPosture`, in metres and radians,
tuned on the bundled rig (see `REFERENCE_STATURE`): reach comfort, carry offset, finger curls, lean limits, hang target,
belly depth, withdrawal. Build a `HumanBody` with a modified posture to change it. What a rig's bones
are (which joint curls the fingers, where the spine pitches) is not tuning: it is read from the skeleton
or measured, as `HumanBody.leanFor` measures how far a lean moves the shoulder.

## Actions plan; the body performs

`ApproachFor` decides where to stand: the shoulder a comfortable reach from the point, and, given the
surface the point rests on (`from` on a pick, `onto` on a place), far enough back that the belly clears
that surface's edge, leaning the upper body over it for the rest of the reach and, once the lean is
spent, stretching the arm past comfortable up to `HumanPosture.stretch`. It plans from the body's
standing posture (`HumanBody.standingBone`, captured when the body is built), not from the animated pose,
which bobs with the gait: a reach near a limit must not be possible in one phase of a stride and not in
another. A point is out of reach only beyond the stretch, not beyond the comfortable distance. With two hands
on one object each shoulder stays a little to the side of its hand's grasp point (`HumanPosture.handSpread`),
and that sideways gap takes its share of the arm, so the reach ahead and below is measured with it removed.
The stretch sits a few centimetres short of the solver's hard limit on purpose: it has to absorb the walker's
stopping error. `Pick` and `Place` reach
and grasp, and ask the body to straighten up when they are done. Actions set intent (`setLean`,
`setCarry`, `setReachWorld`); they never touch joints.

## Reaches are held against the torso

A reach target is given in a frame (`ReachSpace`): the world, the body's root, or the torso. A hand that has
just set a part down is withdrawn to a point against the chest, not in the root frame, because the torso
straightens as the worker steps away and a point fixed in the root frame ends up against the shoulder, which
folds the arm and flips the IK. A reach blends in from the animation, and back out, over a time set by how far
the wrist has to go and eased at both ends (`HumanBody.blendSeconds` with `HumanPosture.blendSpeed`; the release
measures its travel with `travelToAnimation`): a fixed 0.2 s dragged a hand 0.6 m at 3 m/s going out and 7 m/s
going in.

## Grips follow the object

A hand's fingers curl on their own (`HumanHand.setCurls`, thumb to pinky). When a pick knows the object's
box, `Pick` asks the body to close the hand to it (`HumanBody.setGrasp`): `graspDepth` measures how far
the box reaches from the palm along the palm's normal as the hand lies at the grasp, and `graspCurls`
finds, for each finger, the curl that brings its own tip that deep. It does so by curling the skeleton in
steps and recording each tip's depth, the way `leanFor` measures a lean, so a shorter finger closes
further than a longer one to the same depth and any rig works. The thumb follows the index finger, an
object thinner than `HumanPosture.pinchBelow` is pinched (the other fingers stay relaxed), and the shape
is kept while the hand holds the object and forgotten when it lets go. A pick with no box closes the
hand to the plain `gripCurl`.

## Jobs that do not describe their surroundings

A worker only leans over an edge and closes its hand to a part's size if the job tells it what it stands at
and picks up. Spec jobs do (`from`, `onto`, and the boxes the builder resolves). A facility job learns it
from the facility: a rack's or station's `Surface` and a slot's `itemHalfExtents` (automationkit), which
`FacilityTargets.surfaceBox` and `slotItemBox` turn into boxes in the facility frame, turned with their
owners. `FacilityFetchJob.deliver` uses them, and its optional surface and part arguments override or supply
what the facility does not say. With neither, the worker stands where its shoulder reaches the point and
grips plainly. A station's pose is where the worker stands at the table, short of it, so its `Surface` sits
ahead of that pose; stepping back to the station at the end of the job is the retreat.

## Keeping it natural

`MotionQuality` samples a pose each tick and reports, per arm, the bend-plane turn rate, the elbow's
height against the shoulder, its sharpest bend, and the wrist's speed and acceleration.
`humankit/sim/tests` holds the rack job to limits just above ordinary gait, and measures how far the
belly stays from the surface it reaches over. `JobGate` applies those rules to any job: the rack sweep
(either hand, three heights, three turns) and the facility sweep (a straight lane, a corner and a U-turn,
at three heights) both run through it. A change that flips an arm or snaps a pose in one tick
fails there with the sample at which it happened. Add a scenario there when a new behaviour has a way
to go wrong that the rack job does not exercise.

## Known limits

- **Low surfaces.** At about a metre or lower the lean reaches its limit before the belly clears the
  surface's edge, leaving it 3 to 4 cm inside for either hand. Reaching that low needs a crouch, which
  the body does not do. The layout sweep (`ScenarioSweepTests`) records this as its `CAPPED_CLEARANCE`.
- **Tuning is checked on one rig.** `HumanPosture` is in metres, tuned on the bundled worker
  (`REFERENCE_STATURE`); a body built without a posture scales its lengths by its stature
  (`HumanPosture.forStature`), but only the bundled rig is swept. The finger-curl axis and the spine
  shares are checked on that rig only.
- **Grips measure one dimension.** The fingers close to the object's depth along the palm's normal and no
  more: they do not wrap a cylinder or a handle differently, the thumb simply follows the index finger,
  and the fingertips reach only about 7 cm from the palm, so a larger object closes the hand as far as it
  goes and no further.
- **Two hands at about a metre.** The arms have about 10 cm of horizontal reach left after the drop from the
  shoulder, and each hand spends some of it sideways, so the worker cannot stand clear of a 0.4 m top without
  crouching: the belly ends about 17 cm inside the edge (the one-handed case measures 3 to 4 cm). The two-hand
  sweep holds that height to completing, placing the part and moving without spikes, and to that measured depth.
- **Both hands and the left hand** share the same code as the right, and the sweep covers a left-hand
  fetch, but two-handed lean and hang are exercised by far fewer scenarios.
