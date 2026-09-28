import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
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

	public static function run():Void {
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
	}
}
