package machinekit.robotics;

import machinekit.assembly.MachineAssembly;
import machinekit.component.Solids;
import machinekit.motion.MotorDriver;
import machinekit.motion.PowerSupply;
import materia.assembly.AssemblyFrames;

/** Offset six-axis arm assembled from source modules and the published class DH
 * geometry. Zero is the DH zero pose. The control hardware is offboard and is
 * excluded from the arm's stated mechanical mass.
 */
class CobotArm extends MachineAssembly {
	public final cls:CobotClass;
	public final reference:CobotReference.CobotReferenceRow;
	public final modules:Array<CobotJoint>;
	public final links:Array<CobotLink>;
	public final baseFlange:RobotFlange;
	public final toolFlange:RobotFlange;
	public final tool:Null<EndEffector>;

	public function new(cls:CobotClass, ?armTool:ArmTool) {
		super();
		this.cls = cls;
		reference = CobotReference.of(cls);
		modules = [];
		links = [];
		baseFlange = new RobotFlange(63);
		for (i in 0...6) modules.push(new CobotJoint(reference.modules[i], reference.speeds[i],
			i == 0 ? baseFlange.pilotHeight : 0));
		toolFlange = new RobotFlange(50);
		addComponent("baseFlange", baseFlange, AssemblyFrames.translation(0, 0, baseFlange.thickness));
		addMemberConnector("baseFlange", "moduleSeat", Solids.axial(0, 0, 0));
		addMemberConnector("baseFlange", "floor", Solids.axial(0, 0, -baseFlange.thickness));
		addComponent("joint1", modules[0]);
		addMate("base-joint1", "fixed", "baseFlange", "moduleSeat", "joint1", "stator");
		for (i in 0...6) {
			var nextRadius = i < 5 ? modules[i + 1].diameter / 2 : toolFlange.flangeDiameter / 2;
			var d = reference.d[i] - (i == 5 ? toolFlange.thickness : 0);
			var link = new CobotLink(reference.a[i], d, reference.alpha[i], modules[i], nextRadius);
			links.push(link);
			var linkId = 'link${i + 1}', jointId = 'j${i + 1}', moduleId = 'joint${i + 1}';
			addComponent(linkId, link);
			addMateOnAxis(jointId, "revolute", moduleId, "rotor", linkId, "start",
				{x: 0, y: 1, z: 0}, 0,
				{lower: -2 * Math.PI, upper: 2 * Math.PI, velocity: null, effort: null});
			if (i < 5) {
				addComponent('joint${i + 2}', modules[i + 1]);
				addMate('$linkId-next', "fixed", linkId, "end", 'joint${i + 2}', "stator");
			} else {
				addComponent("toolFlange", toolFlange);
				addMemberConnector("toolFlange", "back", Solids.axial(0, 0, -toolFlange.thickness));
				addMate("tool-flange", "fixed", linkId, "end", "toolFlange", "back");
			}
			var gearboxId = 'gearbox${i + 1}';
			addComponent(gearboxId, modules[i].gearbox);
			addMate('$gearboxId-mount', "fixed", moduleId, "gearheadSeat", gearboxId, "input");
		}
		addComponent("powerSupply", new PowerSupply(48, 60, 6), AssemblyFrames.translation(600, -180, 0));
		for (i in 0...6) {
			var driverId = 'driver${i + 1}', moduleId = 'joint${i + 1}', jointId = 'j${i + 1}';
			addComponent(driverId, new MotorDriver("GENERIC-SERVO-AMP", 10),
				AssemblyFrames.translation(600, i * 80, 0));
			connectPorts('$driverId-power', "powerSupply", 'power${i + 1}', driverId, "power");
			addMotor('drive_$jointId', jointId, moduleId, driverId, 0.5, 'gearbox${i + 1}');
			addEncoder('encoder_$jointId', jointId, moduleId, 'drive_$jointId');
		}
		if (armTool == null) tool = null; else {
			tool = armTool.build(toolFlange);
			include("tool", tool);
			addMate("tool-mount", "fixed", "toolFlange", "face", "tool/plate", "robot");
			armTool.expose(this);
		}
		exposeConnector("floor", "baseFlange", "floor");
		exposeConnector("toolFace", "toolFlange", "face");
	}

	/** The moving mechanics and base plate, excluding offboard control hardware and a tool. */
	public function armMass():Float {
		var mass = baseFlange.massProperties().mass + toolFlange.massProperties().mass;
		for (module in modules) mass += module.massProperties().mass + module.gearbox.massProperties().mass;
		for (link in links) mass += link.massProperties().mass;
		return mass;
	}

	/** Holding torque about each mechanical joint axis, including a point payload
	 * at the flange. The assembly supplies every downstream mass and centre.
	 * Values are signed N m; positions in the CAD state are millimetres.
	 */
	public function holdingTorques(state:cadkit.modeling.AssemblyState, payloadKg:Float):Array<Float> {
		if (payloadKg < 0 || !Math.isFinite(payloadKg)) throw "Payload needs a finite nonnegative mass";
		var parents = new Map<String, String>();
		for (joint in state.definition.joints)
			if (joint.role == materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree)
				parents.set(joint.child, joint.parent);
		var result:Array<Float> = [];
		for (i in 0...6) {
			var root = 'link${i + 1}';
			var frame = state.worldConnector('joint${i + 1}', "rotor");
			var axis = AssemblyFrames.transformVector(frame, 0, 1, 0);
			var tip = state.worldPose("toolFlange");
			var torque = 9.81 * payloadKg * ((tip.x - frame.x) * axis.y - (tip.y - frame.y) * axis.x) / 1000;
			for (entry in components()) {
				var ancestor:Null<String> = entry.id;
				while (ancestor != null && ancestor != root) ancestor = parents.get(ancestor);
				if (ancestor == null) continue;
				var mass = entry.component.massProperties();
				var centre = mass.centreOfMass;
				var point = AssemblyFrames.transformPoint(state.worldPose(entry.id), centre.x, centre.y, centre.z);
				torque += 9.81 * mass.mass * ((point.x - frame.x) * axis.y - (point.y - frame.y) * axis.x) / 1000;
			}
			result.push(torque);
		}
		return result;
	}

	/** A conventional elbow-down pose with a vertical tool axis. */
	public function ready():Array<Float> return [0, -Math.PI / 2, Math.PI / 2, -Math.PI / 2, -Math.PI / 2, 0];
}
