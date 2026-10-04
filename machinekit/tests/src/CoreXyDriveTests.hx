import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import materia.project.SceneArtifact;
import robotkit.device.DeviceBinding;
import robotkit.device.DeviceLayout;
import robotkit.model.DriveLoads;
import robotkit.model.RobotModel;
import robotkit.model.SteadyLoads;

/**
 * The CoreXY plotter's drives in the robot model: each motor follows both axes, and each axis is bound
 * by both motors, which share their force between the axes.
 */
class CoreXyDriveTests {
	static function near(actual:Float, expected:Float, message:String, tolerance:Float):Void {
		if (!(Math.abs(actual - expected) <= tolerance)) throw '$message: expected $expected, got $actual';
	}

	public static function run():Void {
		var scene = SceneArtifact.decode(CoreXyPlotterPreview.plotter());
		var model = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition, AssemblyPhysicalPartView.fromSceneArtifact(scene),
			scene.assemblyState).model;
		var plotter = new CoreXyPlotter();
		var beltDescription = plotter.describe();
		var atUpper = plotter.definition();
		for (joint in atUpper.joints) if (joint.id == "y") {
			joint.limits.lower = null; joint.limits.upper = joint.defaultValue;
		}
		var carriageBelt:machinekit.transmission.TimingBelt = null;
		for (member in plotter.components()) if (member.id == "beltA") carriageBelt = cast member.component;
		var missingEnvelope = false;
		for (path in beltDescription.machine.beltPaths) if (path.belt == "beltA") {
			try machinekit.transmission.BeltStretch.stiffness(carriageBelt, path, "x", "pulleyA", atUpper)
			catch (error:machinekit.transmission.TransmissionDesignError)
				missingEnvelope = error.message.indexOf('without limits for joint "y"') >= 0;
		}
		if (!missingEnvelope) throw "A moving belt axis at its upper default must still require a complete stiffness envelope";
		var radius = plotter.pulleyRadius / 1000;
		if (model.couplings.length != 16 || model.actuators.length != 2)
			throw 'the plotter should have 16 belt couplings and two motors, got ${model.couplings.length} and ${model.actuators.length}';
		var errors = model.validate();
		if (errors.length > 0) throw 'the plotter model should validate: $errors';
		// Leaving Y out of a plan must leave its elastic coordinate free, not clamp it.
		var allLoads = DriveLoads.of(model);
		var xOnly = DriveLoads.of(model, null, ["x"]);
		var fullX = [for (load in allLoads) if (load.axis == "x") load][0];
		near(xOnly[0].stiffness, fullX.stiffness, "partial CoreXY stiffness retains omitted free axis", 1e-6);
		near(xOnly[0].backlash, fullX.backlash, "partial CoreXY lost motion retains full Jacobian", 1e-12);
		var fullForces = [for (load in allLoads) load.axis == "x" ? 10.0 : 0.0];
		var fullDeflections = allLoads[0].elastic.deflections(fullForces);
		near(xOnly[0].elastic.deflections([10.0])[0], fullDeflections[allLoads.indexOf(fullX)],
			"partial CoreXY zero load on omitted axis", 1e-12);
		var partlyWired = robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(model));
		partlyWired.actuators.pop();
		var partialLoads = DriveLoads.of(partlyWired);
		if (partialLoads.length != 2 || partialLoads[0].stiffness != 0 || partialLoads[1].stiffness != 0)
			throw "One bound CoreXY motor must retain two per-axis loads without a singular-matrix throw";
		var partialErrors = partlyWired.validate();
		if (partialErrors.length == 0 || partialErrors[0].indexOf("under-actuated") < 0)
			throw 'One bound CoreXY motor needs a design diagnostic: $partialErrors';
		if (!(partlyWired.coupledLimits("x", new SteadyLoads()).requireEffort() > 0 &&
			partlyWired.coupledLimits("y", new SteadyLoads()).requireEffort() > 0))
			throw "One bound CoreXY motor still gives per-axis effort limits";
		// Each motor's pulley is the sum of two terms, one per axis, of one pitch radius per radian: 1 / R in radians per metre.
		for (id in ["pulleyA-turn", "pulleyB-turn"]) {
			var terms = [for (coupling in model.couplings) if (coupling.follower == id) coupling];
			if (terms.length != 2 || terms[0].leader == terms[1].leader) throw '$id should follow both axes, got $terms';
			for (term in terms) near(Math.abs(term.ratio), 1 / radius, '$id turns a pitch radius per radian', 1e-6);
		}
		var rate = model.actuators[0].requireRate();
		// The axis's speed is the motor's over the sum of the two terms (the box in which every combination of axis
		// speeds stays inside the motor's): rate / (2 / R).
		for (id in ["x", "y"]) near(model.coupledLimits(id).requireVelocity(), rate * radius / 2, 'axis $id speed from both motors', 1e-9);
		// Each axis carries half of each motor's force: the load names both motors at a share of a half.
		for (load in DriveLoads.of(model)) {
			if (load.motors.length != 2) throw 'axis ${load.axis} should be carried by both motors';
			for (motor in load.motors) near(motor.share, 0.5, 'axis ${load.axis} takes half of each motor', 1e-12);
		}
		var x = model.coupledLimits("x"), y = model.coupledLimits("y");
		if (!(x.requireAcceleration() > y.requireAcceleration() && y.requireAcceleration() > 20 && x.requireAcceleration() < 200))
			throw 'the carriage should accelerate harder than the gantry it rides on: ${x.requireAcceleration()} against ${y.requireAcceleration()}';
		// A controller's step rate caps the motors, and through them both axes: 16 microsteps at 40 kHz is 78.5 rad/s.
		var binding = DeviceBinding.bind(model, DeviceLayout.forActuators(model), 40000);
		if (binding.channels.length != 2) throw "the plotter's two motors take two channels";
		var ceiling = 40000.0 / (200.0 * 16.0 / (2 * Math.PI));
		near(binding.model.actuators[0].requireRate(), ceiling, "the step rate is each motor's ceiling", 1e-6);
		near(binding.model.coupledLimits("x").requireVelocity(), ceiling * radius / 2, "and the axes' (250 mm/s)", 1e-9);
		near(binding.model.coupledLimits("x").requireVelocity(), 0.25, "250 mm/s", 1e-4);
		Sys.println('corexy plotter drives: motors ${Math.round(rate * 10) / 10} rad/s, axes to ${Math.round(x.requireVelocity() * 1e4) / 10} mm/s, ' +
			'${Math.round(x.requireAcceleration() * 10) / 10} (x) and ${Math.round(y.requireAcceleration() * 10) / 10} (y) m/s²');
	}
}
