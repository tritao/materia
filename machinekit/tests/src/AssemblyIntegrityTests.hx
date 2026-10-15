import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.AssemblyIntegrity;
import machinekit.assembly.AssemblyIntegrity.IntegrityReport;
import machinekit.assembly.AxisBuilder;
import machinekit.assembly.AxisBuilder.AxisSpec;
import machinekit.assembly.Sense;
import machinekit.assembly.Transmission;
import machinekit.gantry.GantryParts.GantryPlate;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.MotorDriver;
import machinekit.motion.NemaStepper;
import machinekit.structural.SeatedPlate;
import machinekit.transmission.Rack;
import machinekit.transmission.SpurGear;
import materia.assembly.AssemblyFrames;

/** `AssemblyIntegrity` on small assemblies with one defect each, and on their sound versions. */
class AssemblyIntegrityTests {
	static function check(condition:Bool, message:String):Void {
		if (!condition) throw 'Assembly integrity: $message';
	}

	static function codes(report:IntegrityReport):Array<String> return [for (item in report.errors()) item.code];

	static function expectClean(report:IntegrityReport, label:String):Void
		check(report.errors().length == 0, '$label should be sound, found: ${[for (item in report.errors()) item.message].join("; ")}');

	static function expectOnly(report:IntegrityReport, wanted:Array<String>, label:String):Void {
		var found = codes(report);
		for (code in wanted) check(found.indexOf(code) >= 0, '$label should report $code, found $found');
		for (code in found) check(wanted.indexOf(code) >= 0, '$label should not report $code: ${[for (item in report.errors()) item.message].join("; ")}');
	}

	/** A plate on a plate, `lift` mm above (negative: sunk into) the lower one's top. */
	static function stack(lift:Float, baseZ:Float = 0):AxisBuilder {
		var builder = new AxisBuilder();
		builder.place("base", new GantryPlate("Integrity base", 100, 100, 10), AssemblyFrames.translation(0, 0, baseZ));
		builder.attach("top", new GantryPlate("Integrity top", 50, 50, 10), AssemblyFrames.translation(0, 0, baseZ + 10 + lift), "base");
		return builder;
	}

	public static function run():Void {
		expectClean(AssemblyIntegrity.check(stack(0), {floorZ: 0}), "a plate standing on a plate on the floor");
		expectOnly(AssemblyIntegrity.check(stack(5)), ["integrity.support.gap", "integrity.support.ground"], "a plate fixed 5 mm above its base");
		expectOnly(AssemblyIntegrity.check(stack(-2)), ["integrity.overlap"], "a plate sunk 2 mm into its base");
		expectOnly(AssemblyIntegrity.check(stack(0, 5), {floorZ: 0}), ["integrity.support.ground"], "a stack 5 mm above the floor");
		expectClean(AssemblyIntegrity.check(stack(0, 5)), "a stack with no floor stated");
		sweeps();
		seated();
		racks();
		screws();
		Sys.println("assembly integrity: support, interference, sweep and drive checks pass");
	}

	/** A block on a rail meets a stop only at the far end of its travel: the rest state is sound, the sweep is not. */
	static function sweeps():Void {
		function slider(withStop:Bool):AxisBuilder {
			var builder = new AxisBuilder();
			var up = [0.0, 0, 1], along = [1.0, 0, 0];
			builder.place("rail", LinearRail.metric("MGN12C", 300), AxisBuilder.orient(0, 0, 0, up, along));
			builder.slide({id: "x", lower: 0, upper: 100, initial: 0}, "rail", "block", LinearRailBlock.metric("MGN12C"),
				AxisBuilder.orient(50, 0, 0, up, along), {x: 1.0, y: 0.0, z: 0.0});
			if (withStop) builder.attach("stop", new GantryPlate("Integrity stop", 20, 30, 20), AssemblyFrames.translation(170, 0, 0), "rail");
			return builder;
		}
		expectClean(AssemblyIntegrity.check(slider(false)), "a block on a rail");
		expectOnly(AssemblyIntegrity.check(slider(true)), ["integrity.sweep"], "a block that runs into a stop at the end of its travel");
	}

	/** A rail mounted on a plate's seat and a block on the rail: poses, the axis's direction and support all follow from the seats. */
	static function seated():Void {
		var builder = new AxisBuilder();
		var rail = LinearRail.metric("MGN12C", 300);
		// The seat is where the rail's base sits: on the plate's top, +Y (a connector's axis) up, +Z along the rail.
		var plate = new SeatedPlate(400, 100, 10, "Integrity base", [{name: "rail", holes: [],
			frame: AxisBuilder.orient(-150, 0, 10, [0.0, 0, 1], [1.0, 0, 0])}]);
		builder.place("base", plate, AssemblyFrames.translation(0, 0, 0));
		builder.mount("rail", rail, "base", "base", "rail");
		builder.slideOn({id: "x", lower: 0, upper: 200, initial: 50}, "rail", "block", LinearRailBlock.metric("MGN12C"), 50);
		var direction = builder.axisDirection("x");
		check(Math.abs(direction[0] - 1) < 1e-12 && Math.abs(direction[1]) < 1e-12 && Math.abs(direction[2]) < 1e-12,
			'the axis runs along the rail, got $direction');
		var model = new AssemblyModel("mm");
		builder.addTo(model, "");
		var state = new AssemblyState(model.definition("seated"));
		function describe(pose:materia.assembly.AssemblyRecord.AssemblyFrame):String
			return '(${pose.x}, ${pose.y}, ${pose.z} | ${pose.qx}, ${pose.qy}, ${pose.qz}, ${pose.qw})';
		for (x in [0.0, 200.0]) {
			state.setJoint("x", x);
			state.forwardKinematics();
			var block = state.worldPose("block"), railPose = state.worldPose("rail");
			// The block rides on the rail's running surface with the rail's orientation.
			check(Math.abs(block.x - (-100 + x)) < 1e-9 && Math.abs(block.y) < 1e-9 && Math.abs(block.z - (10 + rail.spec.railHeight)) < 1e-9
				&& Math.abs(block.qx - railPose.qx) < 1e-9 && Math.abs(block.qy - railPose.qy) < 1e-9 && Math.abs(block.qz - railPose.qz) < 1e-9,
				'the block rides the rail: at x = $x the block is at ${describe(block)}, the rail at ${describe(railPose)}');
		}
		expectClean(AssemblyIntegrity.check(builder, {floorZ: 0}), "a block on a rail seated on a plate");
	}

	/**
	 * A carriage on a rail whose motor's pinion rolls on a fixed rack. `driveRack` derives the rack's place and the
	 * coupling's sense: rolling on the rack's pitch line, a pinion turning positively about +Y (its shaft's direction)
	 * carries its centre along +X, by one pitch radius per radian. The same drive with the sense reversed by hand must be
	 * found by the drive check.
	 */
	static function racks():Void {
		function axis(reversed:Bool):AxisBuilder {
			var builder = new AxisBuilder();
			var spec:AxisSpec = {id: "x", lower: 0.0, upper: 100.0, initial: 25.0};
			var along = [1.0, 0, 0], up = [0.0, 0, 1];
			builder.place("rail", LinearRail.metric("MGN12C", 300), AxisBuilder.orient(0, 0, -20, up, along));
			builder.slide(spec, "rail", "carriage", LinearRailBlock.metric("MGN12C"), AxisBuilder.orient(60, 0, -20, up, along), {x: 1.0, y: 0.0, z: 0.0});
			var motor = NemaStepper.frame(17);
			builder.attach("motor", motor, AxisBuilder.orient(60, -motor.variant.shaftLength, 10, up, [0, 1, 0]), "carriage");
			var pinion = new SpurGear(1, 20, 8), rack = new Rack(1, 100, 8);
			var pose = AxisBuilder.orient(60, 0, 10, up, [0, 1, 0]);
			var driver = (motorId:String) -> {
				builder.attach("driver", new MotorDriver("GENERIC-TMC2209", new machinekit.power.PhaseRmsCurrent(1.5), 16,
					new machinekit.power.DcVoltage(24)), AssemblyFrames.translation(0, -100, 0), "rail");
				return "driver";
			};
			if (!reversed) builder.driveRack(spec, "pinion", "motor", pinion, pose, [0.0, 0, -1], "rack", rack, "rail", driver);
			else {
				var placement = builder.rackPlacement(spec, pinion, pose, [0.0, 0, -1], rack);
				builder.attach("rack", rack, placement.pose, "rail");
				builder.hang("pinion", pinion, pose, "motor");
				var ratio = builder.addTransmission("pinion-rack", "x", "pinion-turn", RackAndPinion("pinion", "rack"),
					placement.sense == Same ? Opposite : Same);
				builder.addMateOnAxis("pinion-turn", "continuous", "motor", "to-pinion", "pinion", "attach-pinion", {x: 0, y: 1, z: 0},
					ratio * spec.initial);
				builder.addMotor("motor", "pinion-turn", "motor", driver("motor"));
			}
			return builder;
		}
		var derived = axis(false);
		var model = new AssemblyModel("mm");
		derived.addTo(model, "");
		var state = new AssemblyState(model.definition("rack"));
		function turnAt(x:Float):Float {
			state.setJoint("x", x);
			state.forwardKinematics();
			return state.joint("pinion-turn");
		}
		function pinionAngle(x:Float):Float {
			state.setJoint("x", x);
			state.forwardKinematics();
			var pose = state.worldPose("pinion");
			// The pinion's local +X, seen along +Y (its shaft): its angle about +Y.
			var axisX = AssemblyFrames.transformVector(pose, 1, 0, 0);
			return Math.atan2(axisX.x, axisX.z);
		}
		// Module 1, 20 teeth: a 10 mm pitch radius.
		var turn = turnAt(40) - turnAt(30);
		check(Math.abs(turn - 1) < 1e-9, 'the pinion turns +1 rad about +Y for 10 mm along +X, got $turn');
		var posed = pinionAngle(40) - pinionAngle(30);
		check(Math.abs(posed - 1) < 1e-9, 'the pinion\'s pose turns with its joint: +1 rad about +Y for 10 mm, got $posed');
		var drives = [for (item in AssemblyIntegrity.check(derived).errors()) if (StringTools.startsWith(item.code, "integrity.drive")) item];
		check(drives.length == 0, 'the derived rack drive rolls, found: ${[for (item in drives) item.message].join("; ")}');
		var reversed = [for (item in AssemblyIntegrity.check(axis(true)).errors()) item.code];
		check(reversed.indexOf("integrity.drive.slip") >= 0, 'a hand-reversed rack sense is a slip, found $reversed');
	}

	/**
	 * A nut on a slide and a right-hand screw beside it. `screwSense` derives the coupling's sense from the thread and the
	 * slide's direction; the same drive with the sense reversed by hand must be found by the drive check.
	 */
	static function screws():Void {
		function axis(reversed:Bool):AxisBuilder {
			var builder = new AxisBuilder();
			var spec:AxisSpec = {id: "z", lower: 0.0, upper: 50.0, initial: 10.0};
			var up = [0.0, 1, 0], vertical = [0.0, 0, 1];
			builder.place("rail", LinearRail.metric("MGN12C", 200), AxisBuilder.orient(0, 0, 0, up, vertical));
			builder.slide(spec, "rail", "carriage", LinearRailBlock.metric("MGN12C"), AxisBuilder.orient(0, 0, 60, up, vertical), {x: 0.0, y: 0.0, z: 1.0});
			var thread = new LeadScrewThread(MetricTrapezoidal, 8, 2, 4);
			builder.hang("screw", new LeadScrew(thread, 150), AxisBuilder.orient(40, 30, -10, [1.0, 0, 0], vertical), "rail");
			builder.attach("screwNut", new LeadScrewNut(thread, 4, false), AxisBuilder.orient(40, 30, 55, [1.0, 0, 0], vertical), "carriage");
			var sense = builder.screwSense("z", vertical);
			var ratio = builder.addTransmission("screw-lead", "z", "screw-turn", LeadScrew("screw", "screwNut"),
				reversed ? (sense == Same ? Opposite : Same) : sense);
			builder.addMateOnAxis("screw-turn", "continuous", "rail", "to-screw", "screw", "attach-screw", {x: 0, y: 0, z: 1}, ratio * spec.initial);
			return builder;
		}
		var derived = [for (item in AssemblyIntegrity.check(axis(false)).errors()) if (StringTools.startsWith(item.code, "integrity.drive")) item];
		check(derived.length == 0, 'the derived screw sense follows the thread, found: ${[for (item in derived) item.message].join("; ")}');
		var reversed = [for (item in AssemblyIntegrity.check(axis(true)).errors()) item.code];
		check(reversed.indexOf("integrity.drive.slip") >= 0, 'a hand-reversed screw sense is a slip, found $reversed');
	}
}

/** Standalone run. */
function main():Void AssemblyIntegrityTests.run();
