import machinekit.assembly.MachineAssembly;
import machinekit.assembly.ControllerWiring;
import machinekit.control.RobotController;
import machinekit.screwdriving.ScrewPresenter;
import machinekit.standard.SocketHeadCapScrew;
import RobotArm.ArmBlock;

class ControllerWiringTests {
	public static function run():Void {
		var assembly = new MachineAssembly();
		assembly.include("supply", new ScrewPresenter(SocketHeadCapScrew.metric("M6", 30), true));
		assembly.addComponent("cabinet", new RobotController(1, 1, 1, 0));
		assembly.connectPorts("gate-command", "cabinet", "output1", "supply/escapement", "control");
		assembly.connectPorts("present-input", "supply/part-present", "signal", "cabinet", "input1");
		assembly.addComponent("passive", new ArmBlock(10, 10, 10, "steel", "Passive slide"));
		assembly.addMateOnAxis("spring", "prismatic", "supply/escapement", "mount", "passive", "base",
			{x:0, y:1, z:0}, 0, {lower:0, upper:10, velocity:10, effort:10});
		function definition():materia.assembly.AssemblyDefinition {
			var model = new cadkit.modeling.AssemblyModel("mm");
			assembly.addTo(model, "");
			return model.definition("process-wiring");
		}
		var wiring = ControllerWiring.of(assembly, definition());
		if (wiring == null || wiring.controllers.length != 1) throw "Process equipment needs its explicitly wired cabinet";
		var control = wiring.controllers[0];
		if (control.joints.indexOf("supply/escapement-slide") < 0 || control.joints.indexOf("spring") < 0 ||
			control.sensors.indexOf("supply/part-present") < 0)
			throw "The discrete gate, passive carried joint and wired presence head must share the process runtime";
		assembly.addComponent("unwired", new ArmBlock(10, 10, 10, "steel", "Unwired root"));
		assembly.addComponent("unwiredSlide", new ArmBlock(10, 10, 10, "steel", "Unwired slide"));
		assembly.addMateOnAxis("unowned", "prismatic", "unwired", "top", "unwiredSlide", "base",
			{x:0, y:1, z:0}, 0, {lower:0, upper:10, velocity:10, effort:10});
		var refused = false;
		try {var ignored = ControllerWiring.of(assembly, definition());}
		catch (error:Dynamic) refused = Std.string(error).indexOf('Joint "unowned" has no controller') >= 0;
		if (!refused) throw "An unrelated unwired process joint cannot silently borrow a cabinet";
		trace("Process equipment controller wiring passed");
	}
}
