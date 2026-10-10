import collisionkit.native.NativeCollisionWorld;
import machinekit.gantry.LinearTrack;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.PosedParts;
import machinekit.component.ComponentDetail;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.AssemblyModel;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.manipulation.Manipulator;
import robotkit.collision.CollisionClearance;
import materia.project.SceneArtifact;

class LinearTrackCheck {
	static function check(condition:Bool, message:String):Void
		if (!condition) throw 'Linear track: $message';
	static function near(actual:Float, expected:Float, message:String):Void
		check(Math.isFinite(actual) && Math.abs(actual - expected) <= 1e-7, '$message: $actual, expected $expected');

	public static function run():Void {
		var track = new LinearTrack(3000, 300);
		check(!track.check().hasErrors(), "mechanics and powered drive validate");
		check(track.axisOvertravel("track") >= 50 - 1e-7, "guide blocks retain stated end clearance");
		var export = new AssemblyModel();
		track.addTo(export, "");
		var definition = export.definition();
		var couplings = definition.couplings;
		if (couplings == null) throw "Linear track has no rack coupling";
		check(couplings.length == 1, "one physical rack coupling");
		near(couplings[0].ratio, -0.05, "module 2 / 20 teeth derive the shaft ratio");
		check(definition.actuators != null && definition.actuators.length == 1 &&
			definition.actuators[0].joint == "pinion-turn", "motor drives the pinion");
		var state = new AssemblyState(definition);
		near(state.worldPose("carriage").x, 300, "initial track coordinate");
		near(state.joint("pinion-turn"), -15, "initial shaft follows the rack");
		endClearance(track, state);
		var arm = new RobotArm(false);
		track.includeArm("arm", arm);
		check(track.hasPort("arm/compressedAir"), "arm service boundary is preserved at the track boundary");
		check(track.subassemblies().length == 1 && track.subassemblies()[0].id == "arm", "arm remains one whole include");
		export = new AssemblyModel();
		track.addTo(export, "");
		definition = export.definition();
		state = new AssemblyState(definition);
		for (coordinate in [0.0, 3000.0]) {
			state.setJoint("track", coordinate);
			var floor = state.worldConnector("arm/pedestal", "floor");
			near(floor.x, coordinate, "track translates the arm floor");
			near(floor.z, track.mountZero.z, "arm floor sits on the adapter");
		}
		check(definition.actuators != null && definition.actuators.length == 7, "arm and track preserve all seven drives");
		var scene = SceneArtifact.decode(SceneArtifact.encode(AssemblyPreview.scene(track, "linear-track")));
		var converted = AssemblySimulationBridge.toRobotModel(cast scene.assemblyDefinition,
			AssemblyPhysicalPartView.fromSceneArtifact(scene), scene.assemblyState);
		var model = converted.model;
		check(model.validate().length == 0, "physical model validates");
		var toolFrames = [for (frame in model.frames) if (frame.name == "arm/toolFlange robot flange") frame];
		check(toolFrames.length == 1, "arm tool flange is a physical ownership frame");
		var manipulator = new Manipulator(model, model.links[0].id, toolFrames[0].id);
		check(manipulator.group.count() == 7 && manipulator.external[0], "whole arm ownership derives the external track");
		for (index in 1...7) check(!manipulator.external[index], "six arm joints retain arm ownership");
		check(manipulator.swivel == null, "six-axis arm on a track has no fabricated swivel");
		var reference = [for (_ in 0...7) 0.0];
		var clearance = new CollisionClearance(manipulator, [for (hull in converted.linkHulls)
			{name: hull.part, link: model.links[hull.link].id, vertices: hull.vertices,
				tool: StringTools.startsWith(hull.part, "arm/tool/")}], reference, () -> new NativeCollisionWorld(), 0.003);
		for (coordinate in [manipulator.group.limitsOf(0).lower, manipulator.group.limitsOf(0).upper]) {
			var q = reference.copy(); q[0] = coordinate;
			var hit = clearance.violation(q);
			check(hit == null, 'runtime end clearance at $coordinate m: ' +
				(hit == null ? "clear" : hit.a + " / " + hit.b + ": " + hit.distance + " m"));
		}
		var limits = model.coupledLimits("track");
		check(limits.requireVelocity() > 0 && limits.requireAcceleration() > 0, "rack drive and moving arm mass derive track limits");
		Sys.println('Linear track: 3 m travel, arm ownership and end clearance; ${limits.requireVelocity() * 1000} mm/s, ${limits.requireAcceleration()} m/s²');
	}

	static function endClearance(track:LinearTrack, state:AssemblyState):Void {
		var posed = new PosedParts();
		try {
			var fixed = ["frameLeft", "frameRight", "frameEnd0", "frameEnd1", "rackSupport", "rackFoot0", "rackFoot1", "powerSupply", "driver"];
			var moving = ["carriage", "armMount", "armAdapter", "motor", "motorPlate", "motorSupportLeft", "motorSupportRight", "pinion"];
			for (coordinate in [0.0, 3000.0]) {
				state.setJoint("track", coordinate);
				near(state.worldPose("carriage").x, coordinate, "carriage reaches both travel ends");
				near(state.joint("pinion-turn"), -coordinate / 20, "pinion follows each travel end");
				for (movingId in moving) {
					var first = posed.posed(track.component(movingId), state.worldPose(movingId), ComponentDetail.Preview);
					try {
						for (fixedId in fixed) {
							var second = posed.posed(track.component(fixedId), state.worldPose(fixedId), ComponentDetail.Preview);
							try check(PosedParts.commonVolume(first, PosedParts.boxOf(first), second, PosedParts.boxOf(second)) < 1e-5,
								'$movingId clears $fixedId at $coordinate mm') catch (error:Dynamic) { second.close(); throw error; }
							second.close();
						}
					} catch (error:Dynamic) { first.close(); throw error; }
					first.close();
				}
			}
		} catch (error:Dynamic) { posed.close(); throw error; }
		posed.close();
	}

	static function main():Void run();
}
