# Live enclosure guard check

Run `haxeon run --project=app/tests/enclosure-guard/haxeon.json` from the worktree,
using its native libraries. This fixture uses MuJoCo because the hobber has rotary
brakes. Heavy builds should use the repository's shared heavy-run queue.

The fixture first builds the complete fitted collision scene, then isolates the
controller sequence from contact forces. It checks actual native door switches,
spindle rotation, controlled stopping before unlock, reclosure, explicit reset
and restart, boundary interruption, native E-stop feedback, and operator controls.
The separate CAD checks remain the geometric clearance evidence.

Collision packing preserves convex vertices, poses, surfaces and filters in the
native hull buffer when primitive slots fill. Explicit contact-pair shapes stay
in primitive slots and their indices are remapped. Both native buffers remain
bounded; geometry is neither discarded nor approximated to admit the scene.

Door endpoint windows are centred on the stop and use a matching assumed 0.5 mm
end-stop reserve. Cutting plans hold every process-owned joint, including a
mechanically coupled follower; the separate door actuator owns their motion.
