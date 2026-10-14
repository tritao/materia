import cadkit.modeling.AssemblyState;
import machinekit.screwdriving.ScrewdriverBit;
import machinekit.screwdriving.TighteningController;
import machinekit.screwdriving.TighteningSupplyFacet;
import machinekit.robot.EndEffectorControls;
import machinekit.robot.RobotScene;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinitionFlattener;
import machinekit.robotics.EndEffectorSet;
import cadbridge.EndEffectorRuntimeBridge;

/** Equipment identity, real joints and rejected cell designs; no tightening outcomes yet. */
class ScrewdrivingEquipmentTests {
	static var count = 0;
	static function check(value:Bool, message:String):Void {
		count++; if (!value) throw message;
	}
	static function rejects(run:Void -> Void, message:String):Void {
		var refusal = "";
		try run() catch (error:Dynamic) refusal = Std.string(error);
		check(refusal.indexOf(message) >= 0, 'Expected "$message", got "$refusal"');
	}
	public static function run():Void {
		count = 0;
		var cell = new ScrewdrivingCell();
		ScrewdrivingChecks.check(cell);
		check(cell.work.fastenings().require().length == 4, "All four CAD connections are accessible");
		var controls = EndEffectorControls.derive(cell.arm.tool, "arm/tool");
		check(controls.drivers.length == 1 && controls.controls.length == 1 && controls.arcs.length == 0 &&
			controls.suctions.length == 0, "The driver owns one control and is not a suction or torch tool");
		var driver = controls.drivers[0];
		check(driver.channel == "arm/tool/driver/carrier.drive" && driver.directionChannel == "arm/tool/driver/carrier.direction" &&
			driver.programChannel == "arm/tool/driver/carrier.program" && driver.vacuumChannel == "arm/tool/driver/carrier.vacuum",
			"All four channels follow the capability member");
		check(driver.feedJoint == "arm/tool/driver/feed" && driver.spindleJoint == "arm/tool/driver/spindle" &&
			driver.strokeEncoder == "arm/tool/driver/stroke", "The real joints and encoder are named within the driver scope");
		var state = ScrewdrivingPreview.state(cell);
		var initial = state.worldConnector("arm/tool/driver/carrier", "tcp");
		state.setJoint(driver.feedJoint, -10);
		var retracted = state.worldConnector("arm/tool/driver/carrier", "tcp");
		var axis = AssemblyFrames.transformVector(initial, 0, 0, 1);
		check(Math.abs((retracted.x - initial.x) * axis.x + (retracted.y - initial.y) * axis.y +
			(retracted.z - initial.z) * axis.z + 10) < 1e-6, "The feed moves the TCP ten mm backward on its own axis");
		state.setJoint(driver.spindleJoint, Math.PI / 2);
		var turned = state.worldConnector("arm/tool/driver/carrier", "tcp");
		check(Math.abs(turned.x - retracted.x) + Math.abs(turned.y - retracted.y) + Math.abs(turned.z - retracted.z) < 1e-6,
			"Spindle rotation retains the axial TCP");
		var scene = AssemblyPreview.scene(cell, "screwdriver-test");
		scene.robotTools = RobotScene.robotTools(cell.arm.tool, "arm/tool");
		var encoded = SceneArtifact.encode(scene);
		var restored = SceneArtifact.decode(encoded), tools = restored.robotTools;
		if (tools == null || tools.length != 1) throw "Missing driver after artifact round trip";
		var tool = tools[0], data = tool.screwdriver;
		if (data == null) throw "Missing driver capability after artifact round trip";
		check(tool.kind == "screwdriver" && tool.sensor == driver.sensor && data.springRate == driver.springRate &&
			data.strokeMm == driver.strokeMm && data.bit.kind == driver.bit.kind && data.bit.size == driver.bit.size &&
			data.bitMember == "arm/tool/driver/bit",
			"Driver, exact CAD bit member, spring and sensed stroke survive the current artifact format");
		// A portable stroke is in mm even when its mechanical joint uses metres.
		var metric = SceneArtifact.decode(encoded);
		var metricDefinition = AssemblyDefinitionFlattener.flatten(cast metric.assemblyDefinition);
		metricDefinition.lengthUnit = "m";
		for (joint in metricDefinition.joints) if (joint.id == driver.feedJoint) {
			joint.limits.lower = -driver.strokeMm / 1000; joint.limits.upper = 0;
		}
		metric.assemblyDefinition = metricDefinition;
		var metricCopy = SceneArtifact.decode(SceneArtifact.encode(metric));
		var metricTools = metricCopy.robotTools;
		if (metricTools == null || metricTools.length != 1) throw "Missing metre fixture tool";
		var metricDriver = metricTools[0].screwdriver;
		if (metricDriver == null) throw "Missing metre fixture driver";
		check(metricDriver.strokeMm == driver.strokeMm,
			"Feed limits use the assembly's length unit while the capability retains mm");
		var previous = encoded.sub(0, encoded.length); previous.setInt32(4, 28);
		rejects(() -> { var old = SceneArtifact.decode(previous); }, "schema v28 is unsupported");
		var bad = scene.robotTools[0].screwdriver;
		if (bad == null) throw "Missing fixture driver";
		bad.bitMember = "arm/tool/driver/carrier";
		rejects(() -> { var bytes = SceneArtifact.encode(scene); }, "fixed directly to the spindle"); bad.bitMember = data.bitMember;
		bad.springRate = 0; rejects(() -> { var bytes = SceneArtifact.encode(scene); }, "finite positive"); bad.springRate = data.springRate;
		bad.maxTorque += 1; rejects(() -> { var bytes = SceneArtifact.encode(scene); }, "spindle limits"); bad.maxTorque = data.maxTorque;
		bad.springRate += 1; rejects(() -> { var bytes = SceneArtifact.encode(scene); }, "feed effort"); bad.springRate = data.springRate;
		bad.strokeMm += 1; rejects(() -> { var bytes = SceneArtifact.encode(scene); }, "feed limits"); bad.strokeMm = data.strokeMm;
		bad.feedJoint = "arm/j1"; rejects(() -> { var bytes = SceneArtifact.encode(scene); }, "real feed"); bad.feedJoint = data.feedJoint;
		bad.strokeEncoder = "missing"; rejects(() -> { var bytes = SceneArtifact.encode(scene); }, "stroke encoder"); bad.strokeEncoder = data.strokeEncoder;
		bad.directionChannel = scene.robotTools[0].channel; rejects(() -> { var bytes = SceneArtifact.encode(scene); }, "distinct driver channels");
		bad.directionChannel = data.directionChannel;
		bad.bit.size = "-1"; rejects(() -> { var bytes = SceneArtifact.encode(scene); }, "numeric bit size"); bad.bit.size = data.bit.size;
		scene.robotTools[0].sensor = null; rejects(() -> { var bytes = SceneArtifact.encode(scene); }, "needs one screwdriver");
		scene.robotTools[0].sensor = tool.sensor;
		var supply = TighteningSupplyFacet.of(cell.controller);
		check(supply != null && supply.programs.length == 4, "Controller supplies four named program slots");
		rejects(() -> { var controller = new TighteningController([1, 1]); }, "duplicate program");
		rejects(() -> { var controller = new TighteningController([]); }, "needs programs");
		rejects(() -> { var bit = new ScrewdriverBit(HexSocket(0)); }, "positive drive size");
		check(new ScrewdriverBit(HexSocket(5)).matches(HexSocket(5)) &&
			!new ScrewdriverBit(HexSocket(5)).matches(HexHead(5)), "Drive kind and size both determine bit fit");
		rejects(() -> ScrewdrivingChecks.check(new ScrewdrivingCell(new ScrewdriverBit(HexSocket(99)))), "Bit does not match fastening:");
		rejects(() -> ScrewdrivingChecks.check(new ScrewdrivingCell(null, 2000)), "unreachable");
		var declarations = new robotkit.tool.ScrewdriverChannels("start", "reverse", "program", "vacuum").declarations();
		check(declarations.length == 4 && !declarations[0].keepOnStop && !declarations[1].keepOnStop &&
			!declarations[2].keepOnStop && declarations[3].keepOnStop, "Stops disable the spindle while commanded stops retain vacuum");
		var set = new EndEffectorSet();
		set.addComponent("base", new DriverServiceSource()); set.mount("base", "mount");
		set.exposePort("power", "base", "power"); set.exposePort("control", "base", "control"); set.exposePort("vacuum", "base", "vacuum");
		set.changer("manual", "base", "mount", [{robot: "power", tool: "power"}, {robot: "control", tool: "control"}, {robot: "vacuum", tool: "vacuum"}]);
		set.addTool("driver", cell.arm.tool);
		var bundle = EndEffectorRuntimeBridge.toRuntimeFromDesign(set, "driver", "tcp");
		check(bundle.bindings.controls.length == 1 && bundle.runtime.vacuum == null,
			"The bridge preserves the driver binding without creating a suction runtime");
		switch bundle.bindings.controls[0] {
			case Driver(start, direction, program, vacuum, member, port):
				check(start == "driver/tool/driver/carrier.drive" && direction == "driver/tool/driver/carrier.direction" &&
					program == "driver/tool/driver/carrier.program" && vacuum == "driver/tool/driver/carrier.vacuum" &&
					member == "tool/driver/carrier" && port == "control", "The bridge retains all four driver channels and the inlet");
			case _: throw "The bridge lost its driver variant";
		}
		Sys.println('Screwdriving equipment: $count checks passed');
	}
}

/** Source fixture for a manually coupled driver: both services remain real connected ports. */
private class DriverServiceSource extends machinekit.component.MachineComponent {
	public function new() {
		super("DRIVER-TEST-SOURCE", "Driver service test source", "steel", true);
		addConnector("mount", Mount, machinekit.component.Solids.axial(0, 0, 0));
		addPort({name: "power", kind: ElectricalPower, role: Supply,
			iface: machinekit.screwdriving.ScrewdrivingInterfaces.tool(), required: false});
		addPort({name: "control", kind: Signal, role: Supply,
			iface: machinekit.screwdriving.ScrewdrivingInterfaces.tool(), required: false});
		addPort({name: "vacuum", kind: Vacuum, role: Supply,
			iface: machinekit.screwdriving.ScrewdrivingInterfaces.vacuum(), required: false});
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:machinekit.component.ComponentDetail = Preview):cadkit.modeling.Part
		return cadkit.modeling.Part.box(20, 20, 20);
}
