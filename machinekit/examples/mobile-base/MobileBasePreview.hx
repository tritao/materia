import machinekit.assembly.AssemblyPreview;
import haxe.io.Bytes;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

/** Materia project entrypoint for the differential-drive mobile base. */
class MobileBasePreview {
	public static inline var ASSEMBLY_ID:String = "mobile-base";

	/** Geometry, joints and initial pose of the base; parts with equal designations share geometry. */
	public static function base():Bytes
		return SceneArtifact.encode(AssemblyPreview.scene(new MobileBase(), ASSEMBLY_ID));
}

/** Geometry builds, the base stands on its wheels and casters, the wheels roll, and nothing collides. */
class MobileBaseChecks {
	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	public static function run():Void {
		var scene = SceneArtifact.decode(MobileBasePreview.base());
		var definition = scene.assemblyDefinition;
		var robot = new MobileBase();
		if (definition == null || definition.occurrences.length != robot.components().length)
			throw "Mobile base preview has the wrong number of occurrences";
		for (part in scene.parts) if (part.volume == null || part.volume <= 0 || part.inertia == null)
			throw 'Mobile base part "${part.id}" has no mass properties';
		var continuous = [for (joint in definition.joints) if (Std.string(joint.type) == "continuous") joint.id];
		if (continuous.join(",") != "wheel_l,wheel_r") throw 'Mobile base should have wheel joints wheel_l, wheel_r, got $continuous';
		if (definition.joints.length != robot.components().length - 1) throw "Every member but the base plate should hang off one joint";

		var model = new AssemblyModel("mm");
		robot.addTo(model, "");
		var state = new AssemblyState(model.definition(MobileBasePreview.ASSEMBLY_ID));
		state.forwardKinematics();
		// It stands on the floor: wheel axles a radius up, caster wheels touching it.
		for (id in ["wheelLeft", "wheelRight"]) {
			var centre = state.worldConnector(id, "centre");
			near(centre.z, robot.wheel.radius, '$id axle height', 1e-9);
			near(centre.x, 0, '$id sits on the centre line of the base', 1e-9);
		}
		for (id in ["casterFront", "casterRear"]) near(state.worldConnector(id, "floor").z, 0, '$id touches the floor', 1e-9);
		near(robot.trackWidth(), 300, "track width", 1e-9);
		near(state.worldConnector("wheelLeft", "centre").y, -state.worldConnector("wheelRight", "centre").y,
			"the wheels are symmetric about the centre line", 1e-9);
		var scan = state.worldConnector("lidar", "scan");
		near(scan.z, MobileBase.DECK_Z + MobileBase.DECK_THICKNESS + LidarPuck.SCAN_HEIGHT, "lidar scan height", 1e-9);

		// A positive wheel angle turns each wheel about +Y, which rolls the base forward along +X:
		// the wheel's local +X (world -X at zero) swings up to +Z at a quarter turn.
		for (joint in ["wheel_l", "wheel_r"]) {
			var id = joint == "wheel_l" ? "wheelLeft" : "wheelRight";
			for (angle in [0.0, Math.PI / 2, 2.0]) {
				state.setJoint(joint, angle);
				state.forwardKinematics();
				var pose = state.worldPose(id);
				var x = AssemblyFrames.transformVector(pose, 1, 0, 0);
				var side = id == "wheelLeft" ? 1 : -1;
				near(x.x, -side * Math.cos(angle), '$joint at $angle: local x, world x', 1e-9);
				near(x.z, side * Math.sin(angle), '$joint at $angle: local x, world z', 1e-9);
				near(state.worldConnector(id, "centre").z, robot.wheel.radius, '$joint at $angle keeps its axle', 1e-9);
			}
			state.setJoint(joint, 0);
		}

		// No two members intersect, with the wheels at rest and turned.
		var ids = [for (entry in robot.components()) entry.id];
		for (angle in [0.0, 0.7]) {
			state.setJoint("wheel_l", angle);
			state.setJoint("wheel_r", -angle);
			state.forwardKinematics();
			var solids = [for (id in ids) posed(robot, state, id)];
			for (i in 0...ids.length) for (j in i + 1...ids.length) {
				var common = solids[i].intersect(solids[j]);
				var volume = common.volume();
				common.close();
				if (volume > 1e-3) throw '${ids[i]} collides with ${ids[j]} at wheel angle $angle: ${Math.round(volume)} mm³';
			}
			for (solid in solids) solid.close();
		}

		var mass = robot.massProperties().mass;
		var bom = robot.billOfMaterials().lines();
		Sys.println('mobile base: ${scene.parts.length} definitions, ${definition.occurrences.length} occurrences, ' +
			'${bom.length} BOM lines, ${Math.round(mass * 10) / 10} kg, track ${robot.trackWidth()} mm');
	}

	static function posed(robot:MobileBase, state:AssemblyState, id:String):Part {
		for (entry in robot.components()) if (entry.id == id) {
			var pose = state.worldPose(id);
			var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
			var local = entry.component.geometry(ComponentDetail.Preview);
			var placed = local.placed(new Location(new Plane(new Vector(pose.x, pose.y, pose.z), new Vector(x.x, x.y, x.z),
				new Vector(z.x, z.y, z.z))));
			local.close();
			return placed;
		}
		throw 'Mobile base has no member "$id"';
	}
}

/** Standalone check of the mobile base example. */
function main():Void MobileBaseChecks.run();
