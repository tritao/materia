# Hobbing CAM

`camkit.HobbingJob(spec, hob, usableLength, settings, limits)` produces a
ToolpathKit program for an external spur gear and a right-hand hob. GearKit inputs
and settings use mm/radians, cutting speed uses m/min, and axial feed uses mm per
work revolution. Output coordinates are metres/radians, with metre-based feeds.
The setup describes the logical hobber X/Y/Z axes, not a general tool-tip pose.

Defaults are the plan's assumed prototype numbers: 300 m/min, .5 mm/work rev,
one full-depth pass, no shift and no crown. They are not machining recommendations.
RPM is cutting speed / hob circumference; table rpm is hob rpm * starts / teeth.
Strictly increasing radial depths must finish at the target centre distance.
An optional blank outer radius can include radial stock allowance. Each pass has
one static hob-axis shift; shifts occur with X retracted beyond the blank.
C's additive phase changes by -shift*cos(lead angle)/gear pitch radius, retaining
the HB3 tooth registration. Positive-B right-hand climb cutting advances +Z;
conventional cutting reverses Z and retains the spindle/table rotation sense.

## Geometric coverage

Overrun is computed from the intersection of the blank's infinite axial cylinder
and the hob's tilted cutting cylinder. At radial penetration w, the independent
bounds are Y=sqrt(r²-(D-w)²) and V=sqrt(R²-w²). Maximizing
Y*abs(tan(A))+V/cos(A) gives the contact's Z half-span. Maximizing
Y/cos(A)+V*abs(tan(A)) gives its hob-axis half-span. Both objectives are concave
on the common radial interval; a bounded scalar search includes both endpoints
and adds 1e-9 mm outward roundoff allowance. Gash/tooth gaps can only shrink this
continuous-cylinder envelope.

The programmed stroke starts below and ends above the complete face by this
Z half-span plus the configured clearance. Every shift must leave the active
axial half-span inside the finite usable hob length. A short hob is rejected
with the pass and required length. This assumes the supplied usable length is
actually furnished with cutting edges; sharpening, holders and end relief reduce
that length and must be accounted for by the caller.

Crowning is a positive X bump, zero at both face ends and greatest at mid-face:
`X(Z)=pass centre distance + crown*(1-(2*(Z-bottom)/width-1)^2)` inside the face.
This leaves thicker teeth in the middle. Outside the face the bump is zero.
Crown chords and MotionKit corner blending each consume half the supplied
geometric tolerance. This is a programmed head path; the resulting tooth crown
must still be measured from the physical sweep (HB9/HB11).

## Motion contract

A bounded `WaitSignal` startup barrier requires `hob.job_ready`: the hob is at
its B=0 encoder datum, the EGB is released, and the mechanism is ready for this
job. X withdraws before tilt or table-phase positioning. `PositionJoint` sets
A and additive C in radians and logical X/Y/Z in metres. Gearing is engaged
before the spindle starts, so C follows its measured acceleration too. The
existing spindle-at-speed barrier precedes cutting.

`FeedPerRevolution(table, Z, metresPerTurn)` is modal for Cut lines. Lowering
converts it using the active single-spindle gearbox ratio and commanded spindle
rpm, preserving axial Z feed on sloped crown chords. Zero disables this mode for
approach/retract moves. A missing active ratio or stopped spindle is an error.
This is nominal per-revolution feed conversion; it does not make Z an encoder
follower when actual spindle speed departs from its settled target. The EGB
continues to synchronize C to actual measured B.

After the final radial withdrawal, the program stops B and waits on the measured
`hob.stopped` signal with a finite timeout. It deliberately retains the stopped
B/C gear relation. Releasing it without rebasing additive C would jump back to
its ungeared phase; a later job must establish the startup datum again. HB9 owns
the app's measured signal provider, runtime phase/restart handling, actual stock
sweep and collision checks. No signal is assumed true by this CAM.

`HobbingLimits` checks installed X/Y/Z travel, swivel range and hob/table rpm.
Callers that omit limits get geometry planning only. The example adapter reads
these values from the HB6 hobber. PlanCheck remains responsible for dynamic
speed/torque checks; it needs process acceleration stated for spindle ramps.

`cuttingSeconds` and each pass's `seconds` are nominal feed time, including
computed overrun and clearance. They exclude positioning, spindle settling and
acceleration. The reference test times a real MotionKit S-curve under stated
.2 m/s² and 5 m/s³ bounds and requires cutting time within 1% of work turns/table
rpm. This is not a promise about total machine cycle time. Measured total cycle
time is an HB9 acceptance item.


## App execution (HB9)

The flexspline-hobber project's default `cutting` entry runs a coarse m1/z40
example. `HobbingProgramPlayer` binds the CAM to measured simulation axes and
provides the startup, spindle-speed and stop signals. `HobbingStock` sends
measured poses through complete exact-time gash windows, previews ring stock on
an owned worker and presents GearMeasurement traces and qualification warnings.
The streaming ideal comparison matches HB3 within 0.1 µm (observed 0 µm).

The coarse MuJoCo cutting smoke now passes via
`machinekit/examples/flexspline-hobber/check-quick.sh app`. It uses explicit
pitch-to-tip diagnostic inspection, with no ISO qualification. Collision checks
cover conservative cutter/mandrel/table envelopes; retained-memory
checks cover ring stock and buffered poses rather than cumulative allocation.
The demo's sparse stock grid is not a qualified ISO inspection. Reset restores
the referenced starting state before a new job.
