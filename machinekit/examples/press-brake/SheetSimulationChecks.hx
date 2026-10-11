import NativeKitSim;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.ManufacturedSheetState;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.process.ProcessBendRule;
import cadkit.sheetmetal.process.ProcessTooling;
import machinekit.pressbrake.SimulatedSheet;
import machinekit.pressbrake.ToolingCatalogue;
import machinekit.pressbrake.ToolProfiles;
import machinekit.pressbrake.ToolProfile;
import machinekit.pressbrake.planning.BendOrientation;
import machinekit.pressbrake.planning.PressStationGeometry;
import machinekit.pressbrake.planning.ManufacturedSheetMotion;
import machinekit.pressbrake.planning.PressSheetPose;
import nativekit.scene.Scene;
import nativekit.sim.MotionType;
import nativekit.sim.MujocoSimWorld;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Focused physical-adapter checks. Actual hydraulic press integration is a separate acceptance gate. */
class SheetSimulationChecks {
	static var checks:Int = 0;
	static function near(actual:Float, expected:Float, tolerance:Float, label:String):Void {
		checks++;
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > tolerance)
			throw label + ": " + actual + " != " + expected;
	}
	public static function main():Void {
		checkSignedContactMotion();
		var steel = exercise("A", 1.5, 7850, 400, 250, 205000);
		var aluminium = exercise("B", 1, 2700, 228, 193, 70000);
		checks++;
		if (aluminium <= steel) throw "Material B must spring back further than material A";
		trace("Sheet physical adapter checks passed (" + checks + " assertions)");
	}
	static function checkSignedContactMotion():Void {
		var sheet = SheetDemoParts.p3(), bends = sheet.bends();
		var tooling = new ProcessTooling("acute", "v12", 0.6, Math.PI / 6, 12, Math.PI / 6, 1);
		var station = new PressStationGeometry("v12", 400, 0, 480);
		for (bend in bends) for (reverse in [false, true]) {
			var other = bends[0].id == bend.id ? bends[1] : bends[0];
			var prior = other.angle * 1.01;
			var initial = new ManufacturedSheetState(sheet.flat()).set(other.id, prior);
			var orientation = new BendOrientation(bend.angle > 0, reverse, 20);
			var motion = new ManufacturedSheetMotion(sheet, initial, bend.id, tooling, station, orientation);
			for (fraction in [0.0, 0.2, 0.6, 1.0]) {
				var angle = bend.angle * fraction;
				var actual = new ManufacturedSheetState(initial).set(bend.id, angle);
				var contact = PressSheetPose.manufactured(sheet, actual, bend.id, tooling, station, orientation);
				var observed = motion.atDepth(contact.depth);
				near(observed.evaluation.bend(bend.id).angle, angle, 1e-9, "Signed depth inversion survives reversed insertion");
				near(observed.evaluation.bend(other.id).angle, prior, 1e-12, "The next stroke preserves prior manufacturing error");
			}
		}
	}
	static function exercise(id:String, thickness:Float, density:Float, uts:Float, yield:Float, youngs:Float):Float {
		var punch = ToolingCatalogue.acute(), die = ToolingCatalogue.dies()[1];
		var tooling = new ProcessTooling(punch.id, die.id, punch.tipRadius, punch.includedAngle,
			die.groove.vWidth, die.groove.includedAngle, die.groove.shoulderRadius);
		var rule = new ProcessBendRule(tooling);
		var stock = new SheetMaterial(id, thickness, BendRule.Process(rule), {uts: uts, yield: yield, youngs: youngs});
		var sheet = SheetDemoParts.p1(stock), bend = sheet.bends()[0];
		var process = rule.predict(stock, bend.angle, bend.edgeA.span.length());
		var scene = Scene.create(), world = MujocoSimWorld.create(scene, {timestep: 0.001, physicsSubsteps: 5});
		var session = new SimSession(scene, world.nativeHandle());
		var floor = session.createObject(MotionType.Static, SimShape.box(3, 3, 0.1), new SimPose(0, 0, -0.1));
		var diePieces:Array<Array<Float>> = [];
		for (piece in ToolProfiles.collisionSection(ToolProfile.SingleV(die), 0.01).convexPieces()) {
			var vertices:Array<Float> = [];
			for (x in [0.0, 0.48]) for (point in piece) {
				vertices.push(x); vertices.push(point.y * 0.001); vertices.push(point.z * 0.001);
			}
			diePieces.push(vertices);
		}
		var carrier = session.createObject(MotionType.Static, SimShape.compoundConvex(diePieces), new SimPose(0, 0, 0.4));
		var part = new SimulatedSheet(session, sheet, sheet.flat(),
			new Transform3(new Vec3(0, 0, 0.4), Quat.identity()), density);
		var root = part.root.handle, area = 0.0;
		for (wall in sheet.walls()) area += wall.outline.area();
		var developed = sheet.evaluate(sheet.flat());
		for (record in sheet.bends()) for (region in record.materialRegions(developed.bend(record.id).allowance)) area += region.area();
		near(part.mass, area * thickness * density * 1e-9, 1e-9, "Physical mass conserves exact developed stock");
		var found = NativeKitSim.nksim_session_get_object_body(session.nativeHandle(), carrier.handle);
		if (found.status != 0) throw "Missing physical carrier";
		var initialBody = NativeKitSim.nksim_session_get_object_body(session.nativeHandle(), part.root.handle);
		if (initialBody.status != 0) throw "Missing physical sheet root";
		part.beginStroke(bend.id, tooling, new PressStationGeometry("v12", 400, 0, 480),
			new BendOrientation(true, false, 0), Transform3.identity(), found.out_body);
		for (i in 0...101) {
			var angle = process.loadedAngle * i / 100;
			var actual = new ManufacturedSheetState(sheet.flat()).set(bend.id, angle);
			var pose = machinekit.pressbrake.planning.PressSheetPose.manufactured(sheet, actual, bend.id, tooling,
				new PressStationGeometry("v12", 400, 0, 480), new BendOrientation(true, false, 0));
			part.driveAtDepth(pose.depth); var time = session.step();
			near(part.measuredAngle(bend.id), angle, Math.PI / 1800, "Native wall angle follows supplied depth");
			var expected = pose.wall(bend.wallA).plane.origin;
			var measured = part.wallFrame(bend.wallA).translation;
			near(measured.sub(new Vec3(expected.x * 0.001, expected.y * 0.001, expected.z * 0.001)).norm(),
				0, 0.00005, "Native root placement follows the contact construction");
		}
		var clock = session.stepIndex(), loaded = part.measuredAngle(bend.id);
		var released = part.release();
		checks++;
		if (part.root.handle != root || haxe.Int64.compare(session.stepIndex(), clock) != 0)
			throw "Springback must preserve the root object and shared clock";
		var releasedBody = NativeKitSim.nksim_session_get_object_body(session.nativeHandle(), part.root.handle);
		checks++;
		if (releasedBody.status != 0 || releasedBody.out_body.rawValue() != initialBody.out_body.rawValue())
			throw "Springback must preserve the native root body";
		var releasedTick = session.step();
		near(part.measuredAngle(bend.id), released, Math.PI / 1800, "Rebuilt physical bodies carry the released angle");
		near(released, bend.angle, Math.PI / 1800, "Nominal springback agrees with the process model");
		// Carry stock clear of the tooling before testing free rest on the table.
		session.stop(); session.holdObject(part.root, found.out_body, new SimPose(0.8, 0, 0.2));
		var transferTick = session.step();
		near(part.measuredAngle(bend.id), released, Math.PI / 1800, "Transfer preserves the manufactured bend");
		for (_ in 0...10) { var stationaryTick = session.step(); }
		session.stop(); session.releaseObject(part.root);
		for (_ in 0...5000) { var tick = session.step(); }
		var resting = part.wallFrame(bend.wallA);
		checks++;
		if (resting.translation.z < -0.01 || resting.translation.z > 0.15)
			throw 'The released part must actually reach the table: ${resting.translation.x},${resting.translation.y},${resting.translation.z}';
		for (_ in 0...1000) { var tick = session.step(); }
		near(part.wallFrame(bend.wallA).translation.sub(resting.translation).norm(), 0, 0.00005,
			"A free bent part rests stably on the table");
		near(part.wallFrame(bend.wallA).rotation.angularDistance(resting.rotation), 0, Math.PI / 1800,
			"The resting part keeps its orientation");
		// A completed part can transfer to a carrier with its fixed children intact.
		session.stop(); session.holdObject(part.root, found.out_body, new SimPose(0.8, 0, 0.2));
		var heldTick = session.step();
		near(session.objectPose(part.root).x, 0.8, 0.00005, "Regrip carries the completed rigid tree");
		near(session.objectPose(part.root).z, 0.6, 0.00005, "Regrip follows the carrier frame");
		near(part.measuredAngle(bend.id), released, Math.PI / 1800, "Regrip preserves the manufactured bend");
		session.stop(); session.releaseObject(part.root);
		Sys.println('Physical sheet $id mass=${part.mass} loaded=$loaded released=$released root=$root');
		part.dispose(); floor.dispose(); carrier.dispose(); session.dispose(); world.dispose(); scene.dispose();
		return Math.abs(loaded - released);
	}
}
