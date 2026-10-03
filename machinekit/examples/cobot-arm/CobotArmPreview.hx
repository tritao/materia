import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import machinekit.assembly.AssemblyPreview;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import haxe.io.Bytes;

/** Each class runs a planned free-motion loop, exercising all six joint drives. */
class CobotArmPreview {
	public static function arm(cls:CobotClass):Bytes {
		var arm = new CobotArm(cls);
		var ready = arm.ready();
		var scene = AssemblyPreview.scene(arm, "cobot-" + arm.reference.designation.toLowerCase());
		var placed = new AssemblyState(scene.assemblyDefinition);
		for (i in 0...6) placed.setJoint('j${i + 1}', ready[i]);
		scene.assemblyState = placed.record();
		var excursions = [[0.12, 0.08, -0.1, 0.08, 0.1, -0.12],
			[-0.12, -0.08, 0.1, -0.08, -0.1, 0.12], [0.0, 0, 0, 0, 0, 0]];
		scene.mission = {loop: true, steps: [for (offset in excursions)
			{kind: "moveJoints", at: {occurrence: "toolFlange", connector: "face"},
				joints: [for (i in 0...6) {joint: 'j${i + 1}', position: ready[i] + offset[i]}]}]};
		return SceneArtifact.encode(scene);
	}
	public static function reach500():Bytes return arm(Reach500);
	public static function reach850():Bytes return arm(Reach850);
	public static function reach900():Bytes return arm(Reach900);
	public static function reach1300():Bytes return arm(Reach1300);
}

/** Independent DH arithmetic checks the mechanically assembled flange. */
class CobotArmChecks {
	static function near(a:Float, b:Float, label:String, tolerance:Float = 0.01):Void
		if (!(Math.abs(a - b) <= tolerance)) throw '$label: expected $b, got $a';

	public static function run():Void {
		for (cls in [Reach500, Reach850, Reach900, Reach1300]) {
			var arm = new CobotArm(cls);
			var model = new AssemblyModel("mm");
			arm.addTo(model, "");
			var definition = model.definition("cobot-arm");
			var state = new AssemblyState(definition);
			var poses = [[0.0, 0, 0, 0, 0, 0], arm.ready(), [0.2, -1, 0.8, -0.5, -0.7, 0.3],
				[-0.4, -0.8, 0.5, 0.6, -1.2, -0.2], [0.6, -1.4, 1.2, -0.4, -0.9, 0.1]];
			for (q in poses) {
				for (i in 0...6) state.setJoint('j${i + 1}', q[i]);
				var expected = dh(arm, q), actual = state.worldPose("toolFlange");
				near(actual.x, expected.x, "DH flange X"); near(actual.y, expected.y, "DH flange Y");
				near(actual.z, expected.z, "DH flange Z");
				var a = AssemblyFrames.toRotationMatrix(actual), b = AssemblyFrames.toRotationMatrix(expected);
				for (i in 0...9) near(a[i], b[i], "DH flange orientation", 1e-8);
			}
			for (i in 0...6) state.setJoint('j${i + 1}', 0);
			checkOverlap(arm, state);
			var zero = state.worldPose("toolFlange");
			for (i in 0...6) for (limit in [-2 * Math.PI, 2 * Math.PI]) {
				state.setJoint('j${i + 1}', limit);
				var at = state.worldPose("toolFlange");
				near(at.x, zero.x, "Full-turn limit X"); near(at.y, zero.y, "Full-turn limit Y");
				near(at.z, zero.z, "Full-turn limit Z");
				state.setJoint('j${i + 1}', 0);
			}
			var shoulder = state.worldPose("joint2"), flange = state.worldPose("toolFlange");
			var dx = flange.x - shoulder.x, dy = flange.y - shoulder.y, dz = flange.z - shoulder.z;
			var reach = Math.sqrt(dx * dx + dy * dy + dz * dz);
			var length = Math.abs(arm.reference.a[1]) + Math.abs(arm.reference.a[2]);
			var offset = arm.reference.d[3] + arm.reference.d[5];
			near(reach, Math.sqrt(length * length + offset * offset + arm.reference.d[4] * arm.reference.d[4]),
				"DH-zero shoulder reference distance");
			var torques = arm.holdingTorques(state, arm.reference.payload);
			for (i in 0...6) if (Math.abs(torques[i]) > arm.modules[i].outputRatedTorque)
				throw "Cobot rated payload exceeds a joint holding torque";
			var mass = arm.armMass();
			if (Math.abs(mass - arm.reference.mass) > arm.reference.mass * 0.1)
				throw '${arm.reference.designation} mass $mass differs from reference ${arm.reference.mass}';
			if (definition.encoders == null || definition.encoders.length != 6) throw "Cobot needs six output encoders";
			Sys.println('${arm.reference.designation}: $mass kg, DH-zero shoulder distance $reach mm, holding N m $torques, DH FK at five poses and full-turn limits passed');
		}
	}

	static function dh(arm:CobotArm, q:Array<Float>):AssemblyFrame {
		var result = AssemblyFrames.translation(0, 0, arm.baseFlange.thickness);
		for (i in 0...6) {
			var c = Math.cos(q[i]), s = Math.sin(q[i]);
			var ca = Math.cos(arm.reference.alpha[i]), sa = Math.sin(arm.reference.alpha[i]);
			var a = arm.reference.a[i], d = arm.reference.d[i];
			result = AssemblyFrames.compose(result, AssemblyFrames.fromRotationMatrix(a * c, a * s, d,
				[c, -s * ca, s * sa, s, c * ca, -c * sa, 0, sa, ca]));
		}
		return result;
	}

	static function checkOverlap(arm:CobotArm, state:AssemblyState):Void {
		var ids:Array<String> = [], parts:Array<Part> = [];
		for (entry in arm.components()) {
			var pose = state.worldPose(entry.id);
			var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
			var local = entry.component.geometry();
			parts.push(local.placed(new Location(new Plane(new Vector(pose.x, pose.y, pose.z),
				new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z)))));
			local.close(); ids.push(entry.id);
		}
		for (i in 0...parts.length) for (j in i + 1...parts.length) {
			var a = parts[i].shape.bounds(), b = parts[j].shape.bounds();
			if (a.get_max().get_x() <= b.get_min().get_x() + 1e-5 || b.get_max().get_x() <= a.get_min().get_x() + 1e-5 ||
				a.get_max().get_y() <= b.get_min().get_y() + 1e-5 || b.get_max().get_y() <= a.get_min().get_y() + 1e-5 ||
				a.get_max().get_z() <= b.get_min().get_z() + 1e-5 || b.get_max().get_z() <= a.get_min().get_z() + 1e-5) continue;
			var common = parts[i].intersect(parts[j]), volume = common.volume();
			common.close();
			if (volume > 1e-3) throw '${arm.reference.designation}: ${ids[i]} overlaps ${ids[j]} by $volume mm³';
		}
		for (part in parts) part.close();
	}
}

function main():Void CobotArmChecks.run();
