import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import materia.project.SceneArtifact;
import robotkit.device.DeviceBinding;
import robotkit.device.DeviceLayout;
import robotkit.model.DriveLoads;
import robotkit.model.RobotModel;

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
		var radius = plotter.pulleyRadius / 1000;
		if (model.couplings.length != 16 || model.actuators.length != 2)
			throw 'the plotter should have 16 belt couplings and two motors, got ${model.couplings.length} and ${model.actuators.length}';
		var errors = model.validate();
		if (errors.length > 0) throw 'the plotter model should validate: $errors';
		// Each motor's pulley is the sum of two terms, one per axis, of one pitch radius per radian: 1 / R in radians per metre.
		for (id in ["pulleyA-turn", "pulleyB-turn"]) {
			var terms = [for (coupling in model.couplings) if (coupling.follower == id) coupling];
			if (terms.length != 2 || terms[0].leader == terms[1].leader) throw '$id should follow both axes, got $terms';
			for (term in terms) near(Math.abs(term.ratio), 1 / radius, '$id turns a pitch radius per radian', 1e-6);
		}
		var rate = model.actuators[0].planningRate();
		// The axis's speed is the motor's over the sum of the two terms (the box in which every combination of axis
		// speeds stays inside the motor's): rate / (2 / R).
		for (id in ["x", "y"]) near(model.coupledLimits(id).velocity, rate * radius / 2, 'axis $id speed from both motors', 1e-9);
		// Each axis carries half of each motor's force: the load names both motors at a share of a half.
		for (load in DriveLoads.of(model)) {
			if (load.motors.length != 2) throw 'axis ${load.axis} should be carried by both motors';
			for (motor in load.motors) near(motor.share, 0.5, 'axis ${load.axis} takes half of each motor', 1e-12);
		}
		var x = model.coupledLimits("x"), y = model.coupledLimits("y");
		if (!(x.maxAcceleration > y.maxAcceleration && y.maxAcceleration > 20 && x.maxAcceleration < 200))
			throw 'the carriage should accelerate harder than the gantry it rides on: ${x.maxAcceleration} against ${y.maxAcceleration}';
		// A controller's step rate caps the motors, and through them both axes: 16 microsteps at 40 kHz is 78.5 rad/s.
		var binding = DeviceBinding.bind(model, DeviceLayout.forActuators(model), 40000);
		if (binding.channels.length != 2) throw "the plotter's two motors take two channels";
		var ceiling = 40000.0 / (200.0 * 16.0 / (2 * Math.PI));
		near(binding.model.actuators[0].planningRate(), ceiling, "the step rate is each motor's ceiling", 1e-6);
		near(binding.model.coupledLimits("x").velocity, ceiling * radius / 2, "and the axes' (250 mm/s)", 1e-9);
		near(binding.model.coupledLimits("x").velocity, 0.25, "250 mm/s", 1e-4);
		Sys.println('corexy plotter drives: motors ${Math.round(rate * 10) / 10} rad/s, axes to ${Math.round(x.velocity * 1e4) / 10} mm/s, ' +
			'${Math.round(x.maxAcceleration * 10) / 10} (x) and ${Math.round(y.maxAcceleration * 10) / 10} (y) m/s²');
	}
}
