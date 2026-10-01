package machinekit.assembly;

import machinekit.component.MachineComponent;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * Assembly laid out by where each member sits in the assembly frame with every joint at zero,
 * rather than by picking connectors on each part. `place` adds a root at its pose, `attach` fixes a
 * member to its parent where it stands, and `move` joins it to its parent on a joint whose axis is
 * an assembly-frame direction. The connectors that do the joining are derived from those poses and
 * named after the members (`to-<child>` on the parent, `attach-<child>` on the child), so members
 * sharing one definition still each keep their own.
 */
class PosedAssembly extends MachineAssembly {
	/** Pose of every member with all joints at zero. */
	final zeroPoses = new Map<String, AssemblyFrame>();

	/** Frame whose local +Y points along `up` and local +Z along `along`; the two must be perpendicular. */
	public static function orient(x:Float, y:Float, z:Float, up:Array<Float>, along:Array<Float>):AssemblyFrame {
		var xx = up[1] * along[2] - up[2] * along[1];
		var xy = up[2] * along[0] - up[0] * along[2];
		var xz = up[0] * along[1] - up[1] * along[0];
		return AssemblyFrames.fromRotationMatrix(x, y, z, [xx, up[0], along[0], xy, up[1], along[1], xz, up[2], along[2]]);
	}

	/** A root member at its assembly-frame pose. */
	function place(id:String, component:MachineComponent, pose:AssemblyFrame):Void {
		addComponent(id, component, pose);
		zeroPoses.set(id, pose);
	}

	/** A member fixed to `parent`, at its pose with every joint at zero. */
	function attach(id:String, component:MachineComponent, pose:AssemblyFrame, parent:String):Void {
		addComponent(id, component);
		zeroPoses.set(id, pose);
		connect(parent, id);
		addMate('$id-mount', "fixed", parent, 'to-$id', id, 'attach-$id');
	}

	/**
	 * A member joined to `parent` by joint `jointId` of `kind` (revolute, continuous, prismatic) along
	 * the assembly-frame `axis` through the member's origin; `pose` is where it sits at zero.
	 */
	function move(jointId:String, kind:String, id:String, component:MachineComponent, pose:AssemblyFrame, parent:String,
			axis:AssemblyVector, initial:Float, limits:AssemblyJointLimits):Void {
		addComponent(id, component);
		zeroPoses.set(id, pose);
		connect(parent, id);
		addMateOnAxis(jointId, kind, parent, 'to-$id', id, 'attach-$id', axis, initial, limits);
	}

	/** Connectors meeting at the child's origin with assembly-aligned axes, so joint axes are assembly directions. */
	function connect(parent:String, child:String):Void {
		var childPose = zeroPose(child);
		var meeting = AssemblyFrames.translation(childPose.x, childPose.y, childPose.z);
		addMemberConnector(parent, 'to-$child', AssemblyFrames.compose(AssemblyFrames.inverse(zeroPose(parent)), meeting));
		addMemberConnector(child, 'attach-$child', AssemblyFrames.compose(AssemblyFrames.inverse(childPose), meeting));
	}

	/** Pose of member `id` with every joint at zero. */
	public function zeroPose(id:String):AssemblyFrame {
		var pose = zeroPoses.get(id);
		if (pose == null) throw 'Assembly has no posed member "$id"';
		return pose;
	}
}
