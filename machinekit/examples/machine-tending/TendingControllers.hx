import cadkit.modeling.AssemblyModel;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import machinekit.control.RobotController;
import machinekit.milling.MillPanel;
import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import haxe.io.Bytes;

/** The two independently controlled machines of the tending cell, before interlock logic. */
class TendingControllers extends MachineAssembly {
	public final mill:EnclosedBenchMill;
	public final arm:CobotArm;

	public function new() {
		super();
		mill = new EnclosedBenchMill();
		arm = new CobotArm(Reach850);
		include("mill", mill);
		var baseX = arm.reference.d[3];
		var baseY = mill.opening.y - 250;
		var turn = AssemblyFrames.fromRotationMatrix(0, 0, 0, [0.0, 1, 0, -1, 0, 0, 0, 0, 1]);
		var riserHeight = 600.0;
		addComponent("armRiser", new MillPanel(120, 120, riserHeight, "steel"),
			AssemblyFrames.translation(baseX, baseY, 0));
		include("arm", arm, AssemblyFrames.compose(AssemblyFrames.translation(baseX, baseY, riserHeight), turn));
		addMemberConnector("armRiser", "armSeat", AssemblyFrames.compose(AssemblyFrames.translation(0, 0, riserHeight), turn));
		addMate("arm-riser", "fixed", "armRiser", "armSeat", "arm/baseFlange", "floor");
		addComponent("robotController", new RobotController(), AssemblyFrames.translation(baseX + 350, baseY, 0));
		addComponent("floorMarker", new MillPanel(100, 100, 2, "steel"),
			AssemblyFrames.translation(-1000, -1000, 0));
		for (index in 0...6) connectPorts('robot-axis${index + 1}', "robotController", 'axis${index + 1}',
			'arm/driver${index + 1}', "bus");
		connectPorts("cnc.ready", "mill/cncController", "output1", "robotController", "input1");
		connectPorts("robot.clear", "robotController", "output1", "mill/cncController", "input6");
	}

	public static function preview():Bytes return SceneArtifact.encode(AssemblyPreview.scene(new TendingControllers(), "tending-controllers"));
}

class TendingControllerChecks {
	public static function run():Void {
		var scene = SceneArtifact.decode(TendingControllers.preview());
		if (scene.controllers == null || scene.controllers.length != 2 || scene.signals == null || scene.signals.length != 2)
			throw "Tending cell needs two controllers and two derived signal wires";
		var owners = new Map<String, String>();
		for (control in scene.controllers) for (joint in control.joints) {
			if (owners.exists(joint)) throw 'Joint "$joint" belongs to two controllers';
			owners.set(joint, control.id);
		}
		if (owners.get("mill/mill/x") != "mill/cncController" || owners.get("arm/j1") != "robotController")
			throw "Cabinet wiring did not own the CNC and robot axes";
		for (control in scene.controllers) if (control.id == "mill/cncController") {
			for (sensor in ["mill/door-closed", "mill/door-open", "mill/vise-open", "mill/vise-clamped", "mill/stock-present"])
				if (control.sensors.indexOf(sensor) < 0) throw 'Cabinet input wiring did not own sensor "$sensor"';
		}
		var model = new AssemblyModel("mm");
		new TendingControllers().addTo(model, "");
		if (owners.keys().hasNext() == false) throw "Tending cell has no moving joints";
	}
}
