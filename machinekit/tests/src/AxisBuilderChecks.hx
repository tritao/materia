import machinekit.assembly.AxisBuilder;
import machinekit.assembly.AxisBuilder.AxisSpec;
import cadkit.modeling.AssemblyModel;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.NemaStepper;
import machinekit.motion.MotorDriver;
import machinekit.transmission.Rack;
import machinekit.transmission.SpurGear;
import materia.assembly.AssemblyFrames;
import cadkit.modeling.AssemblyState;

/** The rack drive is exercised here before a gantry uses the shared builder. */
class AxisBuilderChecks {
	static function check(condition:Bool, message:String):Void {
		if (!condition) throw 'Axis builder: $message';
	}

	public static function run():Void {
		var builder = new AxisBuilder();
		var axis:AxisSpec = {id: "x", lower: 0.0, upper: 100.0, initial: 25.0};
		var along = [1.0, 0, 0], up = [0.0, 0, 1];
		builder.place("rail", LinearRail.metric("MGN12C", 300), AxisBuilder.orient(0, 0, -20, up, along));
		builder.slide(axis, "rail", "carriage", LinearRailBlock.metric("MGN12C"),
			AxisBuilder.orient(60, 0, -20, up, along), {x: 1.0, y: 0.0, z: 0.0});
		var motor = NemaStepper.frame(17);
		builder.attach("motor", motor, AxisBuilder.orient(60, -motor.variant.shaftLength, 10, up, [0, 1, 0]), "carriage");
		var pinion = new SpurGear(1, 20, 8);
		var rack = new Rack(1, 100, 8);
		builder.driveRack(axis, "pinion", "motor", pinion, AxisBuilder.orient(60, 0, 10, up, [0, 1, 0]),
			[0.0, 1, 0], "rack", rack, AxisBuilder.orient(0, 0, 0, up, along), "rail", 1.0,
			_ -> {
				builder.attach("driver", new MotorDriver("GENERIC-TMC2209", 1.5, 16, 24),
					AssemblyFrames.translation(0, -100, 0), "rail");
				return "driver";
			});
		var model = new AssemblyModel("mm");
		builder.addTo(model, "");
		var definition = model.definition("rack-axis");
		check(definition.occurrences.length == 6, "the rack, pinion, carriage, rail, motor and driver are physical members");
		var couplings = definition.couplings;
		if (couplings == null) throw "Axis builder: missing coupling";
		var actuators = definition.actuators;
		if (actuators == null) throw "Axis builder: missing actuator";
		check(couplings.length == 1 && couplings[0].source == "x" &&
			couplings[0].target == "pinion-turn" && Math.abs(couplings[0].ratio - 0.1) < 1e-12,
			"the module and tooth count derive 0.1 rad/mm without an authored ratio");
		check(actuators.length == 1 && actuators[0].joint == "pinion-turn" &&
			actuators[0].maxRate > 0, "the motor drives the pinion rather than a fictitious linear actuator");
		var state = new AssemblyState(definition);
		check(Math.abs(state.joint("pinion-turn") - 2.5) < 1e-10, "the shaft starts at the linear axis's initial coordinate");
		for (coordinate in [0.0, 100.0]) {
			state.setJoint("x", coordinate);
			check(Math.abs(state.worldPose("carriage").x - (60 + coordinate)) < 1e-9,
				"the carriage reaches each end of its travel");
			check(Math.abs(state.joint("pinion-turn") - coordinate * 0.1) < 1e-10,
				"the pinion follows the carriage through the physical rack relation");
		}
		check(builder.axisOvertravel("x") > 0, "the block has rail room beyond both travel ends");
		var rejected = false;
		try builder.slide({id: "overrun", lower: -100.0, upper: 100.0, initial: 0.0}, "rail", "badCarriage",
			LinearRailBlock.metric("MGN12C"), AxisBuilder.orient(60, 0, -20, up, along), {x: 1.0, y: 0.0, z: 0.0})
		catch (_:Dynamic) rejected = true;
		check(rejected && builder.components().length == 6, "rail overrun is rejected before adding a carriage");
	}
}
