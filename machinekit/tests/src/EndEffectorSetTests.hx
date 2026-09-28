import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
import machinekit.robotics.ToolChangerMaster;
import machinekit.robotics.ToolChangerTool;
import machinekit.robotics.ParallelGripper;
import materia.assembly.AssemblyFrames;

private class TestChangerMaster extends MachineComponent {
	public function new() {
		super("TEST-MASTER", "Test changer master", "steel", true);
		addConnector("mount", Mount, AssemblyFrames.identity());
		addConnector("couple", Mount, AssemblyFrames.translation(0, 10, 0));
		addPort({name: "airIn", kind: Pneumatic, role: Consumer, iface: Unspecified, required: false});
		addPort({name: "airOut", kind: Pneumatic, role: Supply, iface: Unspecified, required: false});
		addPort({name: "lock", kind: Pneumatic, role: Consumer, iface: Unspecified, required: true});
		addBridge("airIn", "airOut");
		declareMass(5, new Vector(), InertiaTensor.zero());
	}

	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(2, 2, 2);
}

private class CoupledMaster extends MachineComponent {
	public function new() {
		super("COUPLED-MASTER", "Coupled master", "steel", true);
		addConnector("mount", Mount, AssemblyFrames.identity());
		addConnector("couple", Mount, AssemblyFrames.translation(0, 10, 0));
		addPort({name: "airOut", kind: Pneumatic, role: Supply, iface: Unspecified, required: false});
		declareMass(5, new Vector(), InertiaTensor.zero());
	}
	override public function couplingKey():String return "test:master";
	override public function couplingConnector():String return "couple";
	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(2, 2, 2);
}

private class CoupledPlate extends MachineComponent {
	public function new() {
		super("COUPLED-PLATE", "Coupled plate", "steel", true);
		addConnector("mount", Mount, AssemblyFrames.identity());
		addPort({name: "airIn", kind: Pneumatic, role: Consumer, iface: Unspecified, required: false});
		declareMass(1, new Vector(), InertiaTensor.zero());
	}
	override public function couplingKey():String return "test:tool";
	override public function couplingConnector():String return "mount";
	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(2, 2, 2);
}

private class TestChangerPlate extends MachineComponent {
	public function new(mass:Float, reach:Float, extraPort:Bool = false) {
		super("TEST-TOOL-PLATE", "Test tool plate", "steel", true);
		addConnector("mount", Mount, AssemblyFrames.identity());
		addConnector("cup", Mount, AssemblyFrames.translation(0, reach, 0));
		addPort({name: "airIn", kind: Pneumatic, role: Consumer, iface: Unspecified, required: true});
		addPort({name: "vacuum", kind: Vacuum, role: Supply, iface: Unspecified, required: false});
		addConversion("airIn", "vacuum");
		if (extraPort) addPort({name: "aux", kind: Signal, role: Consumer,
			iface: Unspecified, required: true});
		declareMass(mass, new Vector(), InertiaTensor.zero());
	}

	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(2, 2, 2);
}

private class TestChangerCup extends MachineComponent {
	public function new() {
		super("TEST-CUP", "Test suction cup", "rubber", true);
		addConnector("mount", Mount, AssemblyFrames.identity());
		addConnector("contact", Face, AssemblyFrames.translation(0, 5, 0));
		addPort({name: "vacuum", kind: Vacuum, role: Consumer, iface: Unspecified, required: true});
		declareMass(0.5, new Vector(), InertiaTensor.zero());
	}

	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(2, 2, 2);
}

class EndEffectorSetTests {
	static function close(actual:Float, expected:Float, label:String):Void
		if (Math.abs(actual - expected) > 1e-6) throw '$label: expected $expected, got $actual';

	static function fails(action:() -> Void, fragment:String):Void {
		try action() catch (e:Dynamic) {
			if (Std.string(e).indexOf(fragment) < 0) throw 'Expected "$fragment", got $e';
			return;
		}
		throw 'Expected failure containing "$fragment"';
	}

	static function tool(mass:Float, reach:Float, extraPort:Bool = false):EndEffector {
		var result = new EndEffector();
		result.addComponent("plate", new TestChangerPlate(mass, reach, extraPort));
		result.addComponent("cup", new TestChangerCup());
		result.mount("plate", "mount");
		result.addMate("cup-mate", "fixed", "plate", "cup", "cup", "mount");
		result.connectPorts("vacuum-line", "plate", "vacuum", "cup", "vacuum");
		result.exposePort("air", "plate", "airIn");
		if (extraPort) result.exposePort("aux", "plate", "aux");
		result.workingFrame("contact", "cup", "contact", true);
		return result;
	}

	static function genericTool(channels:Int, diameter:Float):EndEffector {
		var result = new EndEffector();
		result.addComponent("half", new ToolChangerTool(channels, diameter));
		result.mount("half", "master");
		result.exposePort("air", "half", "airIn1");
		return result;
	}

	static function coupledTool():EndEffector {
		var result = new EndEffector();
		result.addComponent("plate", new CoupledPlate());
		result.mount("plate", "mount");
		result.exposePort("air", "plate", "airIn");
		return result;
	}

	public static function run():Void {
		var gripper = new ParallelGripper(40, 20, 60, 30);
		var preview = gripper.geometry(Preview), envelope = gripper.geometry(Envelope);
		var previewBox = preview.shape.bounds(), envelopeBox = envelope.shape.bounds();
		close(previewBox.get_max().get_x() - previewBox.get_min().get_x(), 40,
			"gripper preview width");
		close(envelopeBox.get_max().get_x() - envelopeBox.get_min().get_x(), 70,
			"gripper full-open envelope width");
		preview.close();
		envelope.close();
		var set = new EndEffectorSet();
		set.addComponent("master", new TestChangerMaster());
		set.mount("master", "mount");
		set.exposePort("air", "master", "airIn");
		set.exposePort("coupleAir", "master", "airOut");
		set.exposePort("lock", "master", "lock");
		set.changer("changer", "master", "couple", [{robot: "coupleAir", tool: "air"}]);
		set.addTool("short", tool(1, 20));
		set.addTool("long", tool(2, 40));
		var short = set.configuration("short"), long = set.configuration("long");
		close(short.massPropertiesAtMount().mass, 6.5, "short mass");
		close(long.massPropertiesAtMount().mass, 7.5, "long mass");
		close(short.mountTFrame("contact").y, 35, "short contact");
		close(long.mountTFrame("contact").y, 55, "long contact");
		if (short.primaryFrame != "contact" || short.billOfMaterials().quantity("TEST-CUP") != 1 ||
			short.billOfMaterials().quantity("TEST-MASTER") != 1)
			throw "Configuration did not preserve selected frame and BOM";
		if (short.portNames().indexOf("air") < 0 || short.portNames().indexOf("lock") < 0 ||
			short.portNames().indexOf("coupleAir") >= 0)
			throw "Configuration has the wrong robot-side interface";
		var upstream = short.upstream("tool/cup", "vacuum");
		if (upstream.port.instanceId != "robot/master" || upstream.port.portName != "airIn" || !upstream.external)
			throw 'Wrong changer upstream: ${upstream.port.instanceId}/${upstream.port.portName}';
		if (short.components().length != 3 || long.components().length != 3)
			throw "Configuration contains inactive components";
		set.addTool("unmapped", tool(1, 20, true));
		fails(() -> set.configuration("unmapped"), "Required consumer port");
		var missingPort = new EndEffector();
		missingPort.addComponent("plate", new TestChangerPlate(1, 20));
		missingPort.mount("plate", "mount");
		fails(() -> set.addTool("missing-port", missingPort), "does not expose mapped port");
		fails(() -> set.configuration("missing"), "Unknown changer tool");

		var generic = new EndEffectorSet();
		generic.addComponent("master", new ToolChangerMaster(1, 60));
		generic.mount("master", "robot");
		generic.exposePort("coupledAir", "master", "airOut1");
		generic.changer("coupling", "master", "tool", [{robot: "coupledAir", tool: "air"}]);
		generic.addTool("matching", genericTool(1, 60));
		if (generic.toolIds().indexOf("matching") < 0) throw "Matching generic changer was rejected";
		fails(() -> generic.addTool("wrong-diameter", genericTool(1, 55)), "does not fit");
		fails(() -> generic.addTool("wrong-channels", genericTool(2, 60)), "does not fit");
		fails(() -> generic.addTool("wrong-half", tool(1, 20)), "matching coupling interface");
		var wrongConnector = new EndEffectorSet();
		wrongConnector.addComponent("master", new ToolChangerMaster(1, 60));
		fails(() -> wrongConnector.changer("coupling", "master", "robot", []), "coupling connector");

		var custom = new EndEffectorSet();
		custom.addComponent("master", new CoupledMaster());
		custom.mount("master", "mount");
		custom.exposePort("coupledAir", "master", "airOut");
		custom.changer("coupling", "master", "couple", [{robot: "coupledAir", tool: "air"}]);
		fails(() -> custom.addTool("wrong-key", coupledTool()), "does not fit");
		fails(() -> custom.addTool("mixed", tool(1, 20)), "matching coupling interface");
	}
}
