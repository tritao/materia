import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import machinekit.control.RobotController;
import machinekit.milling.MillPanel;
import BenchMillPreview;
import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import machinekit.robotics.DualArmGripperTool;
import machinekit.robot.RobotScene;
import machinekit.control.DigitalSensor;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import haxe.io.Bytes;
import cnckit.controller.RobotInterfaceChannels;

/** The independently controlled machines and production reach study of the tending cell. */
class TendingControllers extends MachineAssembly {
	public final mill:EnclosedBenchMill;
	public final arm:CobotArm;
	public final reach:TendingReachResult;
	public final production:TendingCyclePlan;
	public final infeedTray:PartTray;
	public final outfeedTray:PartTray;
	public final cellTable:TendingCellTable;

	public function viseSeatPlace():SceneArtifact.SceneArtifactPlace
		return {occurrence: "mill/mill/vise/body", connector: "seat"};
	public function viseLoadPlace():SceneArtifact.SceneArtifactPlace
		return {occurrence: "mill/mill/vise/body", connector: "load"};

	public function outfeedPlace(row:Int, column:Int):SceneArtifact.SceneArtifactPlace {
		var target = outfeedTray.placeTarget(row, column);
		return {occurrence: 'outfeedTray/${target.occurrence}', connector: target.connector};
	}

	public function new(?production:TendingCyclePlan) {
		super();
		this.production = production == null ? TendingCyclePlan.bearingBlocks() : production;
		// Tended stock enters as a free infeed blank; keep the vise empty in the machine body.
		mill = new EnclosedBenchMill(false);
		reach = TendingReach.study(mill, this.production);
		var infeed = [for (tray in reach.trays) if (tray.id == "infeedTray") tray][0];
		var outfeed = [for (tray in reach.trays) if (tray.id == "outfeedTray") tray][0];
		infeedTray = new PartTray(this.production.rows, this.production.columns, BenchMill.STOCK_WIDTH, BenchMill.STOCK_DEPTH,
			infeed.pitchX, infeed.pitchY);
		outfeedTray = new PartTray(this.production.rows, this.production.columns, BenchMill.STOCK_WIDTH, BenchMill.STOCK_DEPTH,
			outfeed.pitchX, outfeed.pitchY);
		cellTable = new TendingCellTable(reach.tableWidth, reach.tableDepth, reach.tableTop, reach.tableOffset);
		var infeedBlanks = new MachineAssembly();
		for (slot in infeed.slots) {
			var local = AssemblyFrames.compose(AssemblyFrames.inverse(infeed.frame), slot.frame);
			infeedBlanks.addComponent('blank-${slot.name.substr(5)}',
				new TendingBlank(BenchMill.STOCK_WIDTH, BenchMill.STOCK_DEPTH, BenchMill.STOCK_HEIGHT), local);
			infeedBlanks.addComponent(slot.name + "-present", new DigitalSensor("presence"),
				TendingReach.presenceHeadFrame(local));
		}
		var outfeedSensors = new MachineAssembly();
		for (slot in outfeed.slots) {
			var local = AssemblyFrames.compose(AssemblyFrames.inverse(outfeed.frame), slot.frame);
			outfeedSensors.addComponent(slot.name + "-present", new DigitalSensor("presence"),
				TendingReach.presenceHeadFrame(local));
		}
		arm = new CobotArm(reach.cls, TendingReach.tendingTool());
		include("mill", mill);
		exposePort("mains", "mill/mill/powerSupply", "mains");
		var baseX = reach.armPlacement.x, baseY = reach.armPlacement.y;
		var riserHeight = reach.riser;
		var cncNames = RobotInterfaceChannels.cncOutputs();
		var robotNames = RobotInterfaceChannels.robotRequests();
		var fieldSensorCount = infeed.slots.length + outfeed.slots.length;
		var riserPose = AssemblyFrames.translation(baseX, baseY, 0);
		addComponent("armRiser", new TendingCellRiser(riserHeight), riserPose);
		include("arm", arm, reach.armPlacement);
		addMemberConnector("armRiser", "armSeat",
			TendingCellRiser.armSeatFrame(arm, riserPose, reach.armPlacement));
		addMate("arm-riser", "fixed", "armRiser", "armSeat", "arm/baseFlange", "floor");
		addComponent("robotController", new RobotController(arm.modules.length,
			cncNames.length + fieldSensorCount, robotNames.length, DualArmGripperTool.VALVE_COIL_OUTPUTS),
			AssemblyFrames.translation(baseX + 350, baseY, 0));
		include(infeed.id, infeedTray, infeed.frame);
		include("infeed", infeedBlanks, infeed.frame);
		include(outfeed.id, outfeedTray, outfeed.frame);
		include("outfeed", outfeedSensors, outfeed.frame);
		include("cellTable", cellTable, reach.tableFrame);
		var robotInput = cncNames.length + 1;
		for (slot in infeed.slots) {
			var id = 'infeed/${slot.name}-present';
			addPresence(id, 'infeedTray/${slot.name}', BenchMill.STOCK_HEIGHT + 5);
			connectPorts('field-${id}', id, "signal", "robotController", 'input${robotInput++}');
		}
		for (slot in outfeed.slots) {
			var id = 'outfeed/${slot.name}-present';
			addPresence(id, 'outfeedTray/${slot.name}', BenchMill.STOCK_HEIGHT + 5);
			connectPorts('field-${id}', id, "signal", "robotController", 'input${robotInput++}');
		}
		addComponent("floorMarker", new MillPanel(100, 100, 2, "steel"),
			AssemblyFrames.translation(-1000, -1000, 0));
		for (index in 0...arm.modules.length) connectPorts('robot-axis${index + 1}', "robotController", 'axis${index + 1}',
			'arm/driver${index + 1}', "bus");
		connectPorts("robot-gripper-a-close-coil", "robotController", "valve1", "arm/tool/valveA", "coilA");
		connectPorts("robot-gripper-a-open-coil", "robotController", "valve2", "arm/tool/valveA", "coilB");
		connectPorts("robot-gripper-b-close-coil", "robotController", "valve3", "arm/tool/valveB", "coilA");
		connectPorts("robot-gripper-b-open-coil", "robotController", "valve4", "arm/tool/valveB", "coilB");
		connectPorts("robot-gripper-air", "mill/airManifold", "out3", "arm", "compressedAir");
		connectPorts(RobotInterfaceChannels.CNC_READY, "mill/cncController", "output1", "robotController", "input1");
		for (index in 1...cncNames.length)
			connectPorts(cncNames[index], "mill/cncController", 'output${index + 1}', "robotController", 'input${index + 1}');
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
		var millProject = SceneArtifact.decode(BenchMillPreview.enclosed());
		var job = millProject.machining;
		if (job == null || job.target == null) throw "Tending cell needs the bench mill's CNC job and target";
		job.axes = [for (axis in job.axes) "mill/" + axis];
		job.spindle = "mill/" + job.spindle;
		job.channelScope = "mill/" + job.channelScope;
		if (job.toolPart != null) job.toolPart = "mill/" + job.toolPart;
		job.controllerId = "mill/cncController";
		job.stocks = [for (slot in reach.trays) if (slot.id == "infeedTray")
			for (blank in slot.slots) 'infeed/blank-${blank.name.substr(5)}'];
		var target = [for (part in millProject.parts) if (part.id == job.target) part];
		if (target.length != 1) throw 'Tending mill target "${job.target}" is missing';
		scene.parts.push(target[0]);
		scene.machining = job;
		if (scene.assemblyDefinition == null || scene.assemblyState == null)
			throw "Tending cell preview needs its assembled machine state";
		var state = new AssemblyState(scene.assemblyDefinition, scene.assemblyState);
		for (spec in cell.mill.mill.specs)
			state.setJoint('mill/mill/${spec.id}', spec.initial);
		var park = cell.reach.parkPose;
		if (park.length != 6) throw "Tending cell needs a six-axis arm parking pose";
		for (index in 0...6)
			state.setJoint('arm/j${index + 1}', park[index]);
		scene.assemblyState = state.record();
		return SceneArtifact.encode(scene);
	}
}

class TendingControllerChecks {
	public static function run():Void {
		var cell = new TendingControllers();
		var scene = SceneArtifact.decode(cell.previewArtifact());
		var sceneDefinition = scene.assemblyDefinition;
		if (sceneDefinition == null) throw "Tending cell has no assembly definition";
		if (scene.controllers == null || scene.controllers.length != 2 || scene.signals == null ||
			scene.signals.length != RobotInterfaceChannels.cncOutputs().length + RobotInterfaceChannels.robotRequests().length)
			throw "Tending cell needs two controllers and every robot-interface signal wire";
		if (scene.robotTools == null || scene.robotTools.length != 2)
			throw "Tending cell must save both mounted grippers";
		for (tool in scene.robotTools) if (tool.kind != "gripper" || tool.gripper == null || tool.sensor == null)
			throw "Each tending gripper needs physical jaw limits and feedback";
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
		var infeedStockIds = [for (item in materia.assembly.AssemblyDefinitionFlattener.flatten(sceneDefinition).occurrences)
			if (StringTools.startsWith(item.id, "infeed/blank-")) item.id];
		var tendingJob = scene.machining;
		var fixedViseStock = [for (item in materia.assembly.AssemblyDefinitionFlattener.flatten(sceneDefinition).occurrences)
			if (item.id == "mill/mill/stock") item.id];
		if (fixedViseStock.length != 0)
			throw "A tending cell starts with an empty vise; its stock arrives as a free infeed blank";
		if (tendingJob == null || infeedStockIds.length != 6 || tendingJob.stocks.length != infeedStockIds.length)
			throw "The tending CNC job needs every dynamic infeed blank as a stock source";
		for (id in infeedStockIds) if (tendingJob.stocks.indexOf(id) < 0)
			throw 'The tending CNC job omits infeed stock "$id"';
		var mill = [for (control in scene.controllers) if (control.id == "mill/cncController") control][0];
		var robot = [for (control in scene.controllers) if (control.id == "robotController") control][0];
		for (channel in ["arm/tool/valveA/coilA", "arm/tool/valveA/coilB", "arm/tool/valveB/coilA", "arm/tool/valveB/coilB"])
			if (robot.channels.indexOf(channel) < 0) throw 'Robot cabinet must own gripper coil "$channel"';
		if (robot.joints.filter(joint -> StringTools.endsWith(joint, "gripperA/jaw-left-slide")).length != 1 ||
			robot.joints.filter(joint -> StringTools.endsWith(joint, "gripperB/jaw-left-slide")).length != 1)
			throw "Robot cabinet must own both gripper cylinder joints";
		PartTrayChecks.run();
		if (sceneDefinition.occurrences.filter(item -> StringTools.startsWith(item.id, "infeed/blank-")).length != 6 ||
			robot.sensors.filter(sensor -> StringTools.startsWith(sensor, "infeed/") || StringTools.startsWith(sensor, "outfeed/")).length != 12)
			throw "Tending cell needs six free infeed blank identities and one presence input per tray slot";
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
		var assemblyState = scene.assemblyState;
		if (assemblyState == null) throw "Tending cell has no arm parking state";
		var parking = new AssemblyState(sceneDefinition, assemblyState);
		for (index in 0...6) if (Math.abs(parking.joint('arm/j${index + 1}') -
				cell.reach.parkPose[index]) > 1e-9)
				throw "Tending cell start state differs from the arm's checked mill-clear park pose";
		var layout = cell.reach;
		if (layout.cls != Reach1300 || layout.riser != 550 || layout.poses.length != 96 ||
			layout.jointMargin < 0.1 || layout.wristSine < 0.2 || cell.arm.cls != layout.cls)
			throw 'Tending cell must use the asserted reach result: ' +
				'${layout.cls}/${layout.riser}/${layout.poses.length}, joint=${layout.jointMargin}, wrist=${layout.wristSine}';
		var cellScene = AssemblyPreview.scene(cell, "tending-cell-layout");
		var cellDefinition = cellScene.assemblyDefinition;
		if (cellDefinition == null) throw "Tending cell layout preview has no assembly definition";
		if (cell.cellTable.bankWidth != layout.tableWidth || cell.cellTable.depth != layout.tableDepth ||
			cell.cellTable.topHeight != layout.tableTop ||
			cellDefinition.occurrences.filter(item -> StringTools.startsWith(item.id, "cellTable/")).length != 10)
			throw "Both tray banks need table tops and four floor-supported legs each";
		var state = new AssemblyState(cellDefinition, cellScene.assemblyState);
		var baseFloor = state.worldConnector("arm/baseFlange", "floor");
		var expectedFloor = TendingCellRiser.armFloorFrame(cell.arm, layout.armPlacement);
		if (!sameFrame(baseFloor, expectedFloor))
			throw "Tending cell riser seat differs from the reach-study arm placement";
		var baseFlange = state.worldPose("arm/baseFlange");
		if (!sameFrame(baseFlange, layout.armBaseFlange))
			throw "Tending cell base-flange frame differs from the fixed mount used by its reach study";
		var model = new AssemblyModel("mm");
		cell.addTo(model, "");
		if (owners.keys().hasNext() == false) throw "Tending cell has no moving joints";
	}

	static function sameFrame(first:materia.assembly.AssemblyRecord.AssemblyFrame,
		second:materia.assembly.AssemblyRecord.AssemblyFrame):Bool {
		var dot = first.qx * second.qx + first.qy * second.qy + first.qz * second.qz + first.qw * second.qw;
		return Math.abs(first.x - second.x) <= 1e-6 && Math.abs(first.y - second.y) <= 1e-6 &&
			Math.abs(first.z - second.z) <= 1e-6 && 1 - Math.abs(dot) <= 1e-9;
	}

	static function rejects(scene:materia.project.SceneArtifact.SceneArtifactData, expected:String, message:String):Void {
		var rejected = false;
		try SceneArtifact.encode(scene) catch (error:Dynamic) rejected = Std.string(error).indexOf(expected) >= 0;
		if (!rejected) throw message;
	}
}
