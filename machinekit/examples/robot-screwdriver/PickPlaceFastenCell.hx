import machinekit.control.RobotController;
import machinekit.control.DigitalSensor;
import machinekit.robotics.DualArmGripperTool;
import machinekit.robotics.ParallelGripperSpec;
import machinekit.screwdriving.ScrewdriverBit;
import machinekit.standard.DeepGrooveBearing;
import machinekit.pneumatic.AirSupply;
import materia.assembly.AssemblyFrames;
import RobotArm.ArmTable;

/** Separate handling and fastening controllers, with fixed tools and sensed tray pockets. */
class PickPlaceFastenCell extends ScrewdrivingCell {
	public static inline final HANDLING_CONTROLLER = "handlingController";
	public static inline final FASTENING_CONTROLLER = "fasteningController";
	public static inline final PART_READY = "part-ready";
	public static inline final FASTENED = "fastened";
	public final handling:RobotArm;
	public final gripper:DualArmGripperTool;
	public final housingTray:PartTray;
	public final plateTray:PartTray;
	public final outfeedTray:PartTray;
	public var handlingPark(default, null):Array<Float> = [];
	final wiredPresenters:Map<String, Bool> = [];

	public function new() {
		super(new ScrewdriverBit(HexSocket(5), 45, 6, 0.05), 0, DeepGrooveBearing.metric("6003"),
			false, false, 30, 15, 180, -450, false, false, true);
		replaceComponent("table", new ArmTable(800, 400, RobotArm.TABLE_TOP));
		var housing = work.flange.housing;
		gripper = new DualArmGripperTool(32, housing.face, 60, 16, false, 12, 6, null,
			new ParallelGripperSpec(32, work.plate.width, 60, 16, false, 12, 6));
		handling = new RobotArm(false, gripper);
		include("handling", handling, {x:-550, y:-450, z:0, qx:0, qy:0, qz:Math.sin(Math.PI/4), qw:Math.cos(Math.PI/4)});
		// Keep a 25 mm gap from the fastening table while supporting all three tray footprints.
		addComponent("handlingTable", new ArmTable(800, 350, RobotArm.TABLE_TOP), AssemblyFrames.translation(-400, -920, 0));
		housingTray = new PartTray(1, 1, housing.face, housing.face, housing.face+20, housing.face+20);
		plateTray = new PartTray(1, 1, work.plate.width, work.plate.width, work.plate.width+20, work.plate.width+20);
		outfeedTray = new PartTray(1, 1, work.plate.width, work.plate.width, work.plate.width+20, work.plate.width+20);
		include("housingTray", housingTray, AssemblyFrames.translation(-650, -850, RobotArm.TABLE_TOP));
		include("plateTray", plateTray, AssemblyFrames.translation(-400, -850, RobotArm.TABLE_TOP));
		include("outfeedTray", outfeedTray, AssemblyFrames.translation(-150, -850, RobotArm.TABLE_TOP));
		addMemberConnector("work/plate", "grasp", AssemblyFrames.alongY(0, 0, work.plate.thickness, 0, 0, 1));
		addMemberConnector("work/flange/housing", "grasp", AssemblyFrames.alongY(0, 0, housing.depth, 0, 0, 1));
		// The placement targets are bottom datums; the runtime preserves the measured grasp offset.
		addMemberConnector("table", "plateSeat", AssemblyFrames.translation(0, 20, RobotArm.TABLE_TOP));
		addMemberConnector("work/plate", "housingSeat", AssemblyFrames.alongY(0, 0, work.plate.thickness, 0, 0, 1));
		addComponent(HANDLING_CONTROLLER, new RobotController(6, 5, 1, 4), AssemblyFrames.translation(-1100, -450, 0));
		addComponent(FASTENING_CONTROLLER, new RobotController(6, 9, 9, 0), AssemblyFrames.translation(-450, 200, 0));
		for (axis in 1...7) {
			connectPorts('handling-axis$axis', HANDLING_CONTROLLER, 'axis$axis', 'handling/driver$axis', "bus");
			connectPorts('fastening-axis$axis', FASTENING_CONTROLLER, 'axis$axis', 'arm/driver$axis', "bus");
		}
		for (index in 0...4) connectPorts('handling-valve${index+1}', HANDLING_CONTROLLER, 'valve${index+1}',
			index < 2 ? "handling/tool/valveA" : "handling/tool/valveB", index%2 == 0 ? "coilA" : "coilB");
		addComponent("handlingAir", new AirSupply(), AssemblyFrames.translation(-1100, -250, 0));
		connectPorts("handling-air", "handlingAir", "air", "handling", "compressedAir");
		connectPorts(PART_READY, HANDLING_CONTROLLER, "output1", FASTENING_CONTROLLER, "input1");
		connectPorts(FASTENED, FASTENING_CONTROLLER, "output1", HANDLING_CONTROLLER, "input1");
		presence("housing-present", "housingTray", housing.face, housing.depth+5, 2, gripper.spec.fingerDepth);
		presence("plate-present", "plateTray", work.plate.width, work.plate.thickness+5, 3, gripper.specB.fingerDepth);
		presence("outfeed-present", "outfeedTray", work.plate.width, work.plate.thickness+5, 4, gripper.specB.fingerDepth);
		// Leave the middle of the plate edge clear for B's contact pad during unload.
		var fixtureHeadX = gripper.specB.fingerDepth/2 + DigitalSensor.BODY_WIDTH/2 + 5;
		addComponent("fixture-present", new DigitalSensor("presence"),
			AssemblyFrames.translation(fixtureHeadX, -500-work.plate.width/2-9, RobotArm.TABLE_TOP));
		addPresence("fixture-present", "table/plateSeat", work.plate.thickness+5);
		connectPorts("fixture-presence-input", "fixture-present", "signal", HANDLING_CONTROLLER, "input5");
		configureSupplies();
	}

	override public function configureSupplies():Void {
		var names = [for (entry in components()) if (StringTools.endsWith(entry.id, "/escapement"))
			entry.id.substr(0, entry.id.length-"/escapement".length)];
		names.sort(Reflect.compare);
		for (name in names) if (!wiredPresenters.exists(name)) {
			var terminal = [for (_ in wiredPresenters) true].length + 2;
			connectPorts(name+"-gate-control", FASTENING_CONTROLLER, 'output$terminal', name+"/escapement", "control");
			connectPorts(name+"-presence-input", name+"/part-present", "signal", FASTENING_CONTROLLER, 'input$terminal');
			wiredPresenters.set(name, true);
		}
	}

	function presence(name:String, tray:String, depth:Float, range:Float, input:Int, fingerDepth:Float):Void {
		var seat = connector(tray+"/slot-0-0");
		var frame = AssemblyFrames.compose(solvedPoses().get(seat.instanceId), memberConnectorFrame(seat.instanceId, seat.connectorName));
		// Keep the physical head outside the finger sweep; the sensed volume stays on the pocket datum.
		addComponent(name, new DigitalSensor("presence"),
			AssemblyFrames.translation(frame.x + fingerDepth/2 + DigitalSensor.BODY_WIDTH/2 + 5,
				frame.y-depth/2-9, frame.z));
		addPresence(name, tray+"/slot-0-0", range);
		connectPorts(name+"-input", name, "signal", HANDLING_CONTROLLER, 'input$input');
	}

	override public function initializeState(state:cadkit.modeling.AssemblyState):Void {
		super.initializeState(state);
		var joints = [for (i in 1...7) 'handling/j$i'];
		var limits = [for (spec in handling.specs) {lower:spec.lower, upper:spec.upper}];
		var park = machinekit.robot.ToolAxisReach.solve(state, joints, limits,
			{x:-450, y:-850, z:500}, {x:0, y:0, z:-1}, [for (spec in handling.specs) spec.initial],
			"handling/tool/gripperA/body", "tcp", {x:-1, y:0, z:0});
		if (park == null) throw "Handling arm has no reachable park clear of the fastening fixture";
		handlingPark = park;
		for (i in 0...park.length) state.setJoint(joints[i], park[i]);
	}
}
