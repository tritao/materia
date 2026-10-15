package app;

import robotkit.model.JointType;
import robotkit.model.RobotModel;
import robotkit.runtime.Simulation;
import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;

/**
 * Where physics has some of a robot's joints, read from its links: a device robot's snapshot is the device's
 * reconstruction from step counts (what the controller knows), while a belt that stretches or a carriage that swings
 * moves the links. A prismatic joint's value is the child link's travel along the joint's axis relative to its parent, a
 * revolute joint's the child's turn about it (unwrapped, so a continuous joint like an extruder keeps counting).
 * Values are relative to the first reading, where each joint has the value the caller gives (its reported one when the
 * machine stands at rest, the device having taken it over).
 */
class PhysicalJoints {
	final simulation:Simulation;
	final robotIndex:Int;
	final joints:Array<Int>;
	final model:RobotModel;
	final parents:Array<Int> = [];
	final children:Array<Int> = [];
	/** Each joint's axis in its parent link's frame and in its child link's frame. */
	final parentAxes:Array<Vec3> = [];
	final childAxes:Array<Vec3> = [];
	var referencePosition:Array<Vec3> = [];
	var referenceRotation:Array<Quat> = [];
	var initial:Array<Float> = [];
	var lastValues:Array<Float> = [];
	var referenced = false;
	var stillSince = 0.0;
	/**
	 * How long the links stand still before the machine is taken to be referenced, seconds. A mode of frequency `w` and
	 * amplitude `A` moves a joint by up to `A w dt` between readings `dt` apart, and it does so somewhere in every half period;
	 * so links that moved less than `STILL` per reading for half the slowest mode's period `pi / w` swing by less than
	 * `STILL / (w dt)`. The slowest mode when a device takes a machine over is the backend's generic position law of a driven
	 * joint (`apply_joint_targets`: 10 Hz, a half period of 50 ms; the belts ring at tens of hertz), so half a second is ten
	 * of its half periods, and the referencing error is at most 1.6 um at a 1 ms step. RF13 replaces that law with the motors'
	 * own holds, which are stiffer, so the window still covers them.
	 */
	static inline final SETTLE = 0.5;
	/** The most a joint may move between readings and still be still, in its unit (a tenth of a micrometre; see `SETTLE`). */
	static inline final STILL = 1e-7;
	var turned:Array<Float> = [];

	/** The joints (indices into `model.joints`) to read from `simulation`'s robot `robotIndex`. */
	public function new(simulation:Simulation, robotIndex:Int, model:RobotModel, joints:Array<Int>) {
		this.simulation = simulation;
		this.robotIndex = robotIndex;
		this.model = model;
		this.joints = joints.copy();
		for (index in joints) {
			var joint = model.joints[index];
			if (joint.type != JointType.Prismatic && joint.type != JointType.Revolute && joint.type != JointType.Continuous)
				throw 'Joint "${joint.id}" is not prismatic, revolute or continuous: its physical value is not read from its links';
			parents.push(model.links.indexOf(joint.parent));
			children.push(model.links.indexOf(joint.child));
			var axis = Vec3.fromArray(joint.axis).normalized();
			parentAxes.push(Quat.fromArray(joint.parentFrameRotation).rotate(axis));
			childAxes.push(Quat.fromArray(joint.childFrameRotation).rotate(axis));
		}
	}

	/**
	 * The joints' physical values, in the order given, `time` seconds into the run, when the device reports `reported`.
	 * Until the links have stood still for `SETTLE` seconds the joints are taken to be where the device reports them; then
	 * the links as they stand are where it reports them (a machine that is built in its initial pose and taken over by a
	 * device that counts from the machine's origin settles to the pose its springs hold it in, and is referenced there, as
	 * a machine is by homing), and from then on a value is the reported one plus the links' travel since.
	 */
	public function read(reported:Array<Float>, time:Float):Array<Float> {
		var offsets = [], rotations = [];
		for (slot in 0...joints.length) {
			var parent = simulation.linkPose(robotIndex, parents[slot]), child = simulation.linkPose(robotIndex, children[slot]);
			var inverse = Quat.fromArray(parent.rotation).conjugate();
			offsets.push(inverse.rotate(Vec3.fromArray(child.position).sub(Vec3.fromArray(parent.position))));
			rotations.push(inverse.multiply(Quat.fromArray(child.rotation)));
		}
		if (referencePosition.length == 0) { rebase(offsets, rotations, reported); stillSince = time; }
		var values = [for (slot in 0...joints.length) value(slot, offsets[slot], rotations[slot])];
		if (!referenced) {
			var moved = 0.0;
			for (slot in 0...joints.length) moved = Math.max(moved, Math.abs(values[slot] - lastValues[slot]));
			lastValues = values;
			if (moved > STILL) stillSince = time;
			if (time - stillSince >= SETTLE) {
				rebase(offsets, rotations, reported);
				referenced = true;
				return reported.copy();
			}
			return reported.copy();
		}
		return values;
	}

	/** The links' pose is taken to be where the device says the joints are. */
	function rebase(offsets:Array<Vec3>, rotations:Array<Quat>, reported:Array<Float>):Void {
		referencePosition = offsets.copy();
		referenceRotation = rotations.copy();
		initial = reported.copy();
		turned = [for (_ in joints) 0.0];
		lastValues = reported.copy();
	}

	function value(slot:Int, offset:Vec3, rotation:Quat):Float {
		if (model.joints[joints[slot]].type == JointType.Prismatic)
			return initial[slot] + offset.sub(referencePosition[slot]).dot(parentAxes[slot]);
		// The child's turn since the reference, in its own frame, about the axis (shortest way from the last reading).
		var delta = referenceRotation[slot].conjugate().multiply(rotation);
		if (delta.w < 0) delta = new Quat(-delta.x, -delta.y, -delta.z, -delta.w);
		var vector = new Vec3(delta.x, delta.y, delta.z);
		var angle = 2.0 * Math.atan2(vector.dot(childAxes[slot]), delta.w);
		// A whole number of turns more than the last reading, so a joint that keeps turning keeps counting.
		var turns = Math.round((turned[slot] - angle) / (2.0 * Math.PI));
		turned[slot] = angle + turns * 2.0 * Math.PI;
		return initial[slot] + turned[slot];
	}
}
