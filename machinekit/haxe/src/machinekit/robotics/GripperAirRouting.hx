package machinekit.robotics;

import cadkit.modeling.Vector;
import machinekit.component.PortKind;
import machinekit.component.Solids;
import machinekit.pneumatic.RoutedHose;
import machinekit.pneumatic.SolenoidValve;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Reference 6 mm service loops outside each gripper's -Y face. */
class GripperAirRouting {
	public static inline var OUTER_DIAMETER_MM:Float = 6;
	public static inline var PORT_STAND_OFF_MM:Float = 12;

	public static function sideReach(spec:ParallelGripperSpec):Float
		return spec.bodyDepth / 2 + new SolenoidValve(true, false).bodyHeight +
			2 * PORT_STAND_OFF_MM + OUTER_DIAMETER_MM / 2;

	public static function add(result:EndEffector, poses:Map<String, AssemblyFrame>, gripper:String,
			valve:String, spec:ParallelGripperSpec, outlet:String, id:String):Void {
		var bodyPose = poses.get(gripper + "/body"), valvePose = poses.get(valve);
		var cylinderMember = gripper + "/cylinder", cylinderPose = poses.get(cylinderMember);
		if (bodyPose == null || valvePose == null || cylinderPose == null)
			throw "Gripper air line needs solved body, valve and cylinder poses";
		var start = AssemblyFrames.compose(valvePose, result.memberConnectorFrame(valve, "port" + outlet));
		var target = AssemblyFrames.compose(cylinderPose, spec.cylinder().connector("port" + outlet).frame);
		var inverseBody = AssemblyFrames.inverse(bodyPose);
		var from = AssemblyFrames.transformPoint(inverseBody, start.x, start.y, start.z);
		var to = AssemblyFrames.transformPoint(inverseBody, target.x, target.y, target.z);
		// Separate the two loops in depth and height so neither crosses the
		// other's lead-in. Both enter the barrel normally from below and leave
		// the valve normally toward -Y.
		var laneY = from.y - (outlet == "A" ? 2 : 1) * PORT_STAND_OFF_MM;
		var laneZ = from.z - (outlet == "B" ? PORT_STAND_OFF_MM : 0);
		var inBody = [new Vector(from.x, from.y, from.z), new Vector(from.x, laneY, from.z)];
		if (laneZ != from.z) inBody.push(new Vector(from.x, laneY, laneZ));
		inBody.push(new Vector(to.x, laneY, laneZ));
		inBody.push(new Vector(to.x, to.y, laneZ));
		inBody.push(new Vector(to.x, to.y, to.z));
		var hosePose = AssemblyFrames.compose(start, AssemblyFrames.inverse(Solids.axial(0, 0, 0)));
		var toHose = AssemblyFrames.compose(AssemblyFrames.inverse(hosePose), bodyPose);
		var route = [for (point in inBody) {
			var p = AssemblyFrames.transformPoint(toHose, point.x, point.y, point.z);
			new Vector(p.x, p.y, p.z);
		}];
		result.addComponent(id, new RoutedHose('GRIPPER-6MM-$valve-$outlet', route,
			OUTER_DIAMETER_MM, 4, 0.03, PortKind.Pneumatic));
		result.addMate('$id-valve', "fixed", valve, "port" + outlet, id, "start");
		result.connectPorts('$id-in', valve, outlet, id, "input");
		result.connectPorts('$id-out', id, "output", cylinderMember, outlet);
	}
}
