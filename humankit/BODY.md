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

## Crouching

A character with a crouch clip (`Crouch_Idle_Loop`, found by `clipIndex("crouch_idle")`) can lower its body.
The clip is mixed over the animation at a weight (`ClipPlayer.setOverlay`), so 0 stands, 1 is the clip's
full crouch, and anything between is a blend: the legs, pelvis and feet come from the animation, not from
leg IK, which the bundled worker's rig could not take (its feet hang off the body as IK controls).

`ApproachFor` plans it. A body that can crouch tries standing first and then ever deeper (`crouchLevels`),
measuring the shoulder and belly at each depth on the skeleton (`HumanBody.crouchShift`), and takes the
first stance that is comfortable: the belly clears the surface's edge, the lean stays under `comfortLean`,
and the point is no further below the shoulder than the arm comfortably reaches. A body that cannot crouch
has only the standing stance, as before. The body lowers once the worker has arrived and turned, and
`Pick` and `Place` stand it up as they finish; `WalkTo` waits for it to be upright, since a crouched body
does not walk. While a worker walks up to a surface its free arms are held down (`setArmsDown`) so a swing
does not sweep a hand across the part.

Below what a crouch reaches the worker kneels. `PickUp_Kneeling` goes down, picks up and comes back, so its descent (to where the pelvis
is lowest) is measured and held like the crouch clip: depth is the fraction of the pelvis drop, and the arm reaches down as the clip's
does (`HumanCharacter.setKneel`, `HumanBody.setKneel`). A kneel and a crouch are different ways down, not stages of one, so the planner tries
standing, then crouches, and kneels only when the crouch is not comfortable and the kneel is easier (`discomfort`: short of the edge, past
comfortable reach, past a comfortable lean). A body that is down puts its knees and thighs at the height of a low top, so the stance also
keeps them behind the edge wherever they cross the slab's thickness (`legRadius`, `slabMargin`). Kneeling is slower than crouching
(`kneelRate`), the foot hold is off for it (a foot held where it stood would fight the knee on the floor), and `ReleaseLimb` and `WalkTo`
wait for the body to be up. On the library sweep it reaches tops from 0.3 m; a point at 0.2 m is out of reach.

The Universal Animation Library character (`animkit/assets/quaternius-ual`) is the first that crouches;
`UniversalSweepTests` runs the rack job on it from half a metre to a shelf.

## The arm brings the wrist

A hand holds with the point between its wrist and its knuckles, a few centimetres past the wrist (`HumanBody.palmReach`,
measured on the skeleton). The planner used to ask the arm for a comfortable reach to the point itself, so the wrist ended
that much nearer the shoulder than planned and the elbow folded, and it stood too close to clear an edge it then had to lean over.
Reaches are now planned to the wrist (`comfort` and `stretch` of the arm, plus the palm), and `solveReach` starts the wrist a palm short of the point.
That alone cleared the two-hand layouts at a metre that had needed a crouch, and took the elbow gate back to 30 degrees.

## Bending over a deep top

A lean pitches the upper spine, which carries the shoulder forward by only about 7 cm even at its limit (0.7 rad). To reach the middle
of a top 0.8 m deep with the belly clear of its near edge the worker also bends at the hips: `setSpineHinge` pitches the whole
trunk about the lowest spine joint, so the shoulders travel far forward and down. The planner (`ApproachFor`, standing only) uses the
lean first and the hinge, up to `maxHinge`, for what the lean leaves short (`HumanBody.hingeFor` measures it as the lean is measured).
The belly is a hinge's pivot, so it stays behind the edge while the chest goes over. The library character's default demo (0.8 m tops at table
height) now runs: the rendered frames show it bowed over the rack and the table, with the head low, which is the extreme of what the
arm can reach. Reaching across needs the body to stay bent until the part is down, so `ReleaseLimb` waits for it to straighten.

## Two hands

Two hands reach with both arms, so the stance suits the shoulder that is further back (an idle pose that twists a little, as the library's does
by about 18 degrees, puts the left shoulder 12 cm ahead of the right; planned for the left, the right arm overreached and a pick failed), and the body faces
the point with its shoulders square (the root is turned by the idle pose's twist, `HumanBody.standingTwist`, and the shoulders planned where that puts them).

## Holding the feet

While the idle pose shows (standing, and the cross-fade into and out of a walk) a character whose legs are IK chains holds
its feet where they stand: each foot is reached for at the spot it had when the hold began (`HumanBody.holdFeet`), with a hold
that eases in and out (`lockFrom`, `lockSpan`, `lockSeconds`, `lockReach` in `HumanPosture`). The idle pose keeps its feet still
relative to the body, so a body that starts to move used to drag them and one that rose from a crouch slid them. The share of
idle in the pose is `HumanWalker.stanceShare`; in a steady walk the gait already keeps a planted foot still, and the hold is off.
The bundled worker's feet are controls, not a chain, and are left as they were. On the library sweep it takes the worst foot slide
from 0.23 m to 0.11 m, at the cost of some more foot jerk (2089 to 2400 m/s3). The foot is not turned with the leg: the IK
restores the orientation the foot had in the animation (`keep_end_rotation` in the native two-bone solve, on for legs), so reaching
an ankle to a spot moves the ankle and leaves the foot's pitch alone; without it a leg that bent 30 degrees tilted the foot by as much.
The foot tilt `Naturalness` reports (44 degrees worst, in a crouch) is the animation's own heel lift, not the hold's.

A walk from standing does not begin at the top of the walk cycle. `HumanGait.startTime` is the phase of the cycle whose fade from idle
drags a foot least, found by predicting, for each of the cycle's phases, how far a foot standing at idle would slide while the idle pose
fades into the clip over `HumanWalker`'s blend and the body accelerates (`HumanGait.startSlide`, which models the hold for a
character that has one, as the hold pins the feet during that fade and the slide is what happens when it lets go). The walk, the
carrying walk and the backward walk each start at theirs. On the library sweep this took the worst foot slide from 0.13 m to 0.07 m
(the sweep's gate is now 0.09 m, not 0.15), and the bundled worker's stays at 0.04 m. Matching the idle feet by their positions alone,
or only the planted foot's, was tried first: it helped one rig and hurt the other, because a foot about to lift is low and slow for its
first moments and counts as planted, so the prediction has to follow the same rule `Naturalness` uses.

## Stopping and standing up

Braking to a stop, rising from a full crouch and walking off are each measured on their own (`HumanKitTests.stoppingAndRising`, from the floor the worker
stood on): the library character slides a planted foot 0.02 m braking, 0 m rising (the hold pins the feet through it) and 0.02 m walking
off; the bundled worker 0.04 m braking. Inside whole jobs the worst foot slide is 0.07 m, from three places: a foot planted while crouched that
creeps while the worker rises and sets off (0.06 m), a foot as the knee goes down (0.07 m), and the walk after a release (0.06 m). One
cause of slide the scenarios do not show is the route start: the root follows the route exactly while the heading chases it at `turnRate`, so a body
that sets off facing away from the route moved backwards past its planted feet (up to 145 degrees off its facing for a few tenths of a second).
The walker now moves only as far as it faces along the route (`paceShare`, the cosine of the heading's error), which costs a little time at
such a start and cut foot jerk by a quarter. Turning on the spot first, with the turn clips, was tried and is the better motion, but it exposes
every place a turn clip's end is not seamless for an arm that is not held (a reaching arm, a carrying one, a withdrawn one) one after another,
so it is not done; the two-clip turn now starts its second clip in the tick the root takes up the first, so the one jump is at one moment.

## Standing at a surface

Where to stand and how to hold the body is `action/StancePlanner`; `ApproachFor` is only the action that walks there, turns and then lowers and leans.
The planner proposes stances (standing, ever deeper crouches, ever deeper kneels, with the lean and hinge each needs) and ranks them with one measure,
`discomfort`: shortfall to the edge, a point outside comfortable reach, a reach that ends within `reachMargin` of the arm's limit (a stretch),
lean past comfort, and hinge. The body's own state is split the same way: `PostureState` holds the lean, hinge, crouch and kneel goals and eases
them, `FootHold` holds the feet while the idle pose shows, and `HumanBody` composes them with the limbs.

The planner stands the worker square to the edge it works at, not along the line it walked up (the nearest of the four directions the box's
edges face), and it measures the belly's distance to that edge along the line the belly travels, a shoulder's width to the side of the
target's, since a belly line that is not square to an edge meets it at a different distance (a few centimetres, which once put a belly 2
cm inside a bench). A belly held above a surface's top by more than its own height and a margin (`bellyHalfHeight`, `bellyOverhangMargin`)
overhangs the edge instead of meeting it, which is how a worker kneels over a low shelf; the knees and thighs still stay behind it. The
gate's belly clearance uses the same rule. `Naturalness.standOnTheFloorOf` takes a run's floor from another run's, for a run that begins
mid-stride, where the first sample's foot may be in the air.

## Measuring without showing

The planner measures the body by posing it: the shoulder under a lean, the pelvis at a crouch depth, a fingertip at each curl, the wrist
if a reach were let go. Those poses go through `HumanCharacter.probe`, which evaluates the pose and the joint matrices only
(`ak_instance_evaluate_pose`): no skinning, no scene update, no attachments moved. The poses a measurement makes never reach the
scene, and the next `advance` shows the live pose as usual. A bench approach plans in about a millisecond instead of seven.

## Measuring how natural it looks

`MotionQuality` judges the arms (bend-plane turn, hand speed and acceleration, elbow range). `Naturalness` judges the
body from the floor up, one sample per tick: how far a planted foot slides (planted is low and slow for three
samples), where the centre of mass projects against the hull of both feet while both are down, how far a foot goes
into the floor, and the jerk of the pelvis and wrists. `JobGate` carries both and reports which step of the job a
finding came from; the sweeps print the worst of each. Baselines: the bundled worker slides a foot at most 0.04 m over
the rack sweep and keeps its mass 7 cm inside its feet; the library character slides up to 0.23 m where it crouches and turns,
and 2 cm inside (with the hold and the walk's start phase, 0.07 m; see "Holding the feet"). Foot slide before the hold: a steady walk is gait-matched and keeps a planted foot within about 6 cm on both characters
(the library's walk is as good as the bundled one, 6 cm against 5). The rest, up to 23 cm, is getting up to speed and
stopping: while the body accelerates the idle pose that is still showing keeps its feet still, so a planted foot is dragged
by the distance travelled (about half the speed times the ramp, 15 cm), and a crouched worker standing up slides its feet
about 13 cm. A faster cross-fade cuts the drag but throws the arms (hand speed 3 m/s); the cure that was taken is to lock the planted
foot in the world with leg IK (the library's legs are chains, see "Holding the feet") and to start the walk where its feet match the idle stance.
These are limits to hold, not claims of naturalness: nothing yet compares to a reference motion or
renders a frame.

## The elbow's bend

An arm under IK bends its elbow in a plane that has to be chosen. The animation's own bend is the natural choice and is
carried to the new axis (shoulder to wrist) by the smallest rotation taking the animated axis there, which keeps it
continuous as the target moves. That rotation is undefined where the new axis is the opposite of the animated one, and
just short of it the elbow swings through a half turn on a few millimetres of hand travel. A crouched worker bent far
over a low surface hits exactly that: its animated arm points back and up while the reach goes forward and down
(`animatedBend` in `animkit/native/src/runtime.cpp`). The bend is now one field over all axes: the limb's lateral direction
projected perpendicular to the axis, turned about it by the angle the animation's bend makes with the same projection at
the animated axis. It equals the animation's bend at the animated axis and the old carried bend for any hand moving in the
sagittal plane, and its one fault is an axis along the lateral direction (a limb straight out or straight across), where
the carried bend is used. Blending the old bend and the field is not safe: they differ by the twist of the triangle the
two axes and the lateral direction make, anything up to a half turn. `HumanKitTests.elbowStaysPutEverywhere` sweeps the wrist
round the shoulder on twelve great circles at two degrees a step, bundled and library characters, and holds the elbow's
move to 6 cm a step; the old bend moves 8.7 cm on a step.

`HUMANKIT_PROBE=hand,surface,half,yaw[,sample]` makes the humankit sim tests run one rack-to-table layout and print the
gate's verdict and a window of per-tick state (`ArmFlipProbe`); `PROBE_SIDE` chooses the arm it follows.

## Known limits

- **Low surfaces on the bundled worker.** It has no crouch clip, so below about a metre it cannot lower itself; it
  stands clear of the edge and leans, and a surface its arm cannot reach over leaves the belly short (the planner
  reports it as `shortfall`). With the palm's depth in the stand-off (see "The arm brings the wrist") the 1.0 m
  tops of the sweeps, one hand or two, are cleared.
- **Deep tops, low down.** A top 0.8 m deep (the app's rack and table) is reached across by bending at the hips, and at
  every height the sweep covers, 0.3 m up to 1.16 m: the hinge combines with a crouch or a kneel (`hingeWithCrouch`), and
  the planner ranks stances by one discomfort measure that charges the hinge, so a deeper crouch is preferred to a
  hinge. What stopped this was an arm flip, now explained and fixed (see "The elbow's bend" below). One pair of layouts is
  still out of the sweep: the left hand at a 0.3 m top 0.8 m deep. There the free right arm, held hanging while the worker
  leans through the turn of its approach, has its elbow plane turn at 8.3 rad/s (gate 6.5), because the turn clip
  swings that arm out sideways and the bend field follows it faster than the old carried bend did (4.6). A hanging arm
  does not need the animation's bend at all, but giving it a fixed pole made the hand-off to a reach jump, so the cure
  is a bend field the hang and the reach share, or leaning after the turn.
- **Crouch is one clip.** Depth is a blend between standing and the clip's full crouch, so a middle depth is
  a mixed pose, not a clip of its own; the planner reaches down to about half a metre crouched and 0.3 m kneeling, not the floor, and a
  crouched worker does not walk. The clip's feet are not pinned: a foot may lift a few centimetres at full depth.
- **Curls are tuned on the bundled hand.** The library character's relaxed idle hand starts partly curled, so
  a full curl overshoots toward a tight fist and a grip of a thin part cannot open the fingers further than
  the clip. The curl axis is found on the skeleton, but the angles are not calibrated per rig.
- **Tuning is checked on one rig.** `HumanPosture` is in metres, tuned on the bundled worker
  (`REFERENCE_STATURE`); a body built without a posture scales its lengths by its stature
  (`HumanPosture.forStature`), but only the bundled rig is swept. The finger-curl axis and the spine
  shares are checked on that rig only.
- **Grips measure one dimension.** The fingers close to the object's depth along the palm's normal and no
  more: they do not wrap a cylinder or a handle differently, the thumb simply follows the index finger,
  and the fingertips reach only about 7 cm from the palm, so a larger object closes the hand as far as it
  goes and no further.
- **Both hands and the left hand** share the same code as the right, and the sweep covers a left-hand
  fetch, but two-handed lean and hang are exercised by far fewer scenarios.
