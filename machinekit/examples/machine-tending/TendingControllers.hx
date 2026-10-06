import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import machinekit.control.RobotController;
import machinekit.milling.MillPanel;
import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import machinekit.robotics.ArmGripperTool;
import machinekit.robot.RobotScene;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import haxe.io.Bytes;
import cnckit.controller.RobotInterfaceChannels;

/** The two independently controlled machines of the tending cell, before interlock logic. */
class TendingControllers extends MachineAssembly {
	public final mill:EnclosedBenchMill;
	public final arm:CobotArm;
	public final reach:TendingReachResult;

	public function new() {
		super();
		mill = new EnclosedBenchMill();
		reach = TendingReach.study(mill);
		arm = new CobotArm(reach.cls, new ArmGripperTool(32, BenchMill.STOCK_WIDTH, 60, 16));
		include("mill", mill);
		exposePort("mains", "mill/mill/powerSupply", "mains");
		var baseX = reach.armBase.x, baseY = reach.armBase.y;
		var riserHeight = reach.riser;
		var riserPose = AssemblyFrames.translation(baseX, baseY, 0);
		addComponent("armRiser", new MillPanel(120, 120, riserHeight, "steel"), riserPose);
		include("arm", arm, reach.armBase);
		addMemberConnector("armRiser", "armSeat", AssemblyFrames.compose(AssemblyFrames.inverse(riserPose), reach.armBase));
		addMate("arm-riser", "fixed", "armRiser", "armSeat", "arm/baseFlange", "floor");
		addComponent("robotController", new RobotController(), AssemblyFrames.translation(baseX + 350, baseY, 0));
		addComponent("floorMarker", new MillPanel(100, 100, 2, "steel"),
			AssemblyFrames.translation(-1000, -1000, 0));
		for (index in 0...6) connectPorts('robot-axis${index + 1}', "robotController", 'axis${index + 1}',
			'arm/driver${index + 1}', "bus");
		connectPorts("robot-gripper-close-coil", "robotController", "valve1", "arm/tool/valve", "coilA");
		connectPorts("robot-gripper-open-coil", "robotController", "valve2", "arm/tool/valve", "coilB");
		connectPorts("robot-gripper-air", "mill/airManifold", "out3", "arm", "compressedAir");
		connectPorts(RobotInterfaceChannels.CNC_READY, "mill/cncController", "output1", "robotController", "input1");
		var cncNames = RobotInterfaceChannels.cncOutputs();
		for (index in 1...cncNames.length)
			connectPorts(cncNames[index], "mill/cncController", 'output${index + 1}', "robotController", 'input${index + 1}');
		var robotNames = RobotInterfaceChannels.robotRequests();
		for (index in 0...robotNames.length)
			connectPorts(robotNames[index], "robotController", 'output${index + 1}', "mill/cncController", 'input${index + 6}');
	}

	public static function preview():Bytes {
		return new TendingControllers().previewArtifact();
	}

	/** Encode this cell instance so checks can use the geometry-derived load pose it owns. */
	public function previewArtifact():Bytes {
		var cell = this;
		var scene = AssemblyPreview.scene(cell, "tending-controllers");
		if (cell.arm.tool == null) throw "Tending cell has no physical gripper tool";
		scene.robotTools = RobotScene.robotTools(cell.arm.tool, "arm/tool");
		if (scene.assemblyDefinition == null || scene.assemblyState == null)
			throw "Tending cell preview needs its assembled machine state";
		var state = new AssemblyState(scene.assemblyDefinition, scene.assemblyState);
		for (spec in cell.mill.mill.specs)
			state.setJoint('mill/mill/${spec.id}', spec.initial);
		scene.assemblyState = state.record();
		return SceneArtifact.encode(scene);
	}
}

class TendingControllerChecks {
	public static function run():Void {
		var scene = SceneArtifact.decode(TendingControllers.preview());
		if (scene.controllers == null || scene.controllers.length != 2 || scene.signals == null ||
			scene.signals.length != RobotInterfaceChannels.cncOutputs().length + RobotInterfaceChannels.robotRequests().length)
			throw "Tending cell needs two controllers and every robot-interface signal wire";
		if (scene.robotTools == null || scene.robotTools.length != 1 || scene.robotTools[0].kind != "gripper" ||
			scene.robotTools[0].gripper == null || scene.robotTools[0].sensor != "arm/tool/gripper/gripped")
			throw "Tending cell must save its mounted physical gripper and jaw feedback";
		var owners = new Map<String, String>();
		for (control in scene.controllers) for (joint in control.joints) {
			if (owners.exists(joint)) throw 'Joint "$joint" belongs to two controllers';
			owners.set(joint, control.id);
		}
		if (owners.get("mill/mill/x") != "mill/cncController" || owners.get("arm/j1") != "robotController")
			throw "Cabinet wiring did not own the CNC and robot axes";
		for (name in RobotInterfaceChannels.cncOutputs()) if (scene.signals.filter(wire -> wire.name == name).length != 1)
			throw 'Cell wiring must publish CNC output "$name" exactly once';
		for (name in RobotInterfaceChannels.robotRequests()) if (scene.signals.filter(wire -> wire.name == name).length != 1)
			throw 'Cell wiring must publish robot request "$name" exactly once';
		for (control in scene.controllers) if (control.id == "mill/cncController") {
			for (sensor in ["mill/door-closed", "mill/door-open", "mill/vise-open", "mill/vise-clamped", "mill/stock-present"])
				if (control.sensors.indexOf(sensor) < 0) throw 'Cabinet input wiring did not own sensor "$sensor"';
		}
		var mill = [for (control in scene.controllers) if (control.id == "mill/cncController") control][0];
		var robot = [for (control in scene.controllers) if (control.id == "robotController") control][0];
		if (robot.channels.indexOf("arm/tool/valve/coilA") < 0 ||
			robot.channels.indexOf("arm/tool/valve/coilB") < 0 ||
			robot.joints.filter(joint -> StringTools.endsWith(joint, "gripper/jaw-left-slide")).length != 1)
			throw "Robot cabinet must own both gripper valve coils and the pneumatic jaw joint";
		var sensor = "mill/door-closed";
		mill.sensors.remove(sensor); robot.sensors.push(sensor);
		rejects(scene, "observes joint", "A field sensor wired away from its joint owner needs a cell signal");
		robot.sensors.remove(sensor); mill.sensors.push(sensor);
		robot.sensors.push(sensor);
		rejects(scene, "unique owner", "A sensor cannot belong to both runtimes");
		robot.sensors.remove(sensor);
		var channel = mill.channels[0]; mill.channels.push(channel);
		rejects(scene, "duplicate or empty channel", "Controller channels must be unique after shared-valve deduplication");
		mill.channels.pop();
		SceneArtifact.encode(scene);
		var cell = new TendingControllers();
		var layout = cell.reach;
		if (layout.cls != Reach850 || layout.riser != 550 || layout.names.length != 15 ||
			layout.jointMargin < 0.1 || layout.wristSine < 0.2 || cell.arm.cls != layout.cls)
			throw 'Tending cell must use the asserted reach result: ' +
				'${layout.cls}/${layout.riser}/${layout.names.length}, joint=${layout.jointMargin}, wrist=${layout.wristSine}';
		var cellScene = AssemblyPreview.scene(cell, "tending-cell-layout");
		var state = new AssemblyState(cellScene.assemblyDefinition, cellScene.assemblyState);
		var baseFloor = state.worldConnector("arm/baseFlange", "floor");
		if (Math.abs(baseFloor.x - layout.armBase.x) > 1e-6 ||
			Math.abs(baseFloor.y - layout.armBase.y) > 1e-6 ||
			Math.abs(baseFloor.z - layout.armBase.z) > 1e-6)
			throw "Tending cell mounting pose must be the pose accepted by the reach study";
		var model = new AssemblyModel("mm");
		cell.addTo(model, "");
		if (owners.keys().hasNext() == false) throw "Tending cell has no moving joints";
	}

	static function rejects(scene:materia.project.SceneArtifact.SceneArtifactData, expected:String, message:String):Void {
		var rejected = false;
		try SceneArtifact.encode(scene) catch (error:Dynamic) rejected = Std.string(error).indexOf(expected) >= 0;
		if (!rejected) throw message;
	}
}
