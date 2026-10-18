import cadkit.modeling.Vector;
import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.pneumatic.AirSupply;
import machinekit.pneumatic.RoutedHose;
import machinekit.robotics.ArmGripperTool;
import machinekit.robotics.ArmTool;
import machinekit.robotics.DualArmGripperTool;
import machinekit.robotics.EndEffector;
import machinekit.robotics.ParallelGripperJaw;
import machinekit.robotics.ParallelGripperSpec;
import machinekit.robotics.RobotFlange;
import materia.assembly.AssemblyFrames;

/** Physical support and clearances of the installed pneumatic tools through jaw travel. */
class GripperIntegrityTests {
	public static function main():Void run();
	public static function run():Void {
		for (size in [25, 32, 40]) checkTool(new ArmGripperTool(size), 'single-$size');
		checkTool(new DualArmGripperTool(), "dual-default");
		checkTool(TendingReach.tendingTool(), "machine-tending");
		checkTool(new DualArmGripperTool(32, 50, 60, 16, false, 20, 6, null,
			new ParallelGripperSpec(25, 90, 45, 12, false, 15, 6)), "dual-mixed");
		var jaw = new ParallelGripperJaw(new ParallelGripperSpec(25, 90, 45, 12), -1);
		var recipe = jaw.componentType();
		if (recipe == null) throw "Gripper carriage has no saved recipe";
		var original = jaw.geometry(), restored = recipe.create(jaw.values()).geometry();
		var difference = Math.abs(original.volume() - restored.volume());
		original.close(); restored.close();
		if (difference > 1e-6) throw "Saved wide-opening carriage loses its supporting guide tongue";
		Sys.println("Gripper integrity: supported single, dual, tending and mixed-size tools clear sampled jaw travel");
	}

	static function checkTool(tool:ArmTool, label:String):Void {
		var effector = tool.build(new RobotFlange(50));
		var supplied = new MachineAssembly();
		supplied.include("tool", effector);
		// The remote FRL is a separate grounded service, outside the tool's geometry.
		supplied.addComponent("air", new AirSupply(), AssemblyFrames.translation(0, 0, -1000));
		supplied.connectPorts("air", "air", "air", "tool", "compressedAir");
		var report = AssemblyIntegrity.check(supplied);
		if (report.errors().length != 0)
			throw '$label: ${[for (item in report.errors()) item.message].join("; ")}';
		if (report.statesChecked < 6) throw '$label: jaw travel was not swept';
		checkPorts(effector, label);
		if (Std.isOfType(tool, DualArmGripperTool)) {
			var dual:DualArmGripperTool = cast tool;
			var minX = Math.POSITIVE_INFINITY, minY = minX;
			var maxX = Math.NEGATIVE_INFINITY, maxY = maxX;
			var poses = effector.solvedPoses();
			PosedParts.scope(posed -> {
				for (entry in effector.components()) {
					var box = posed.box(entry.component, poses.get(entry.id));
					minX = Math.min(minX, box.minX); minY = Math.min(minY, box.minY);
					maxX = Math.max(maxX, box.maxX); maxY = Math.max(maxY, box.maxY);
				}
			});
			if (maxX - minX > dual.envelopeWidthMm + 1e-5 || maxY - minY > dual.envelopeDepthMm + 1e-5)
				throw '$label: installed geometry exceeds the advertised tool envelope';
		}
		Sys.println('$label: ${report.statesChecked} integrity states passed');
	}

	static function checkPorts(effector:EndEffector, label:String):Void {
		var poses = effector.solvedPoses();
		for (entry in effector.components()) if (Std.isOfType(entry.component, RoutedHose)) {
			var hose:RoutedHose = cast entry.component;
			var dual = StringTools.startsWith(entry.id, "airHosevalve");
			var valve = dual ? (entry.id.indexOf("valveA") >= 0 ? "valveA" : "valveB") : "valve";
			var gripper = dual ? (valve == "valveA" ? "gripperA" : "gripperB") : "gripper";
			var outlet = entry.id.substr(entry.id.length - 1);
			for (atEnd in [false, true]) {
				var index = atEnd ? hose.route.length - 1 : 0;
				var point = hose.route[index], neighbour = hose.route[atEnd ? index - 1 : index + 1];
				var world = AssemblyFrames.transformPoint(poses.get(entry.id), point.x, point.y, point.z);
				var member = atEnd ? gripper + "/cylinder" : valve;
				var port = AssemblyFrames.compose(poses.get(member), effector.memberConnectorFrame(member, "port" + outlet));
				if (new Vector(world.x - port.x, world.y - port.y, world.z - port.z).length() > 1e-6)
					throw '$label: ${entry.id} misses its port';
				var tangent = neighbour.subtract(point).normalized();
				var direction = AssemblyFrames.transformVector(poses.get(entry.id), tangent.x, tangent.y, tangent.z);
				var normal = AssemblyFrames.transformVector(port, 0, 1, 0);
				if (direction.x * normal.x + direction.y * normal.y + direction.z * normal.z < 0.999999)
					throw '$label: ${entry.id} enters its port obliquely';
			}
		}
	}
}
