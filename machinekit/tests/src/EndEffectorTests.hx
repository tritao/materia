import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.assembly.MachineAssembly.AssemblyBomMass;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorFrames;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

private class EndEffectorTestPart extends MachineComponent {
	public function new(name:String, mass:Float, ?mountFrame:AssemblyFrame) {
		super(name, name, "steel", true);
		addConnector("mount", Mount, mountFrame == null ? AssemblyFrames.identity() : mountFrame);
		addConnector("tip", Mount, AssemblyFrames.translation(0, 20, 0));
		declareMass(mass, new Vector(0, 0, 0), new InertiaTensor(1, 0, 0, 2, 0, 3));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(2, 2, 2);
}

class EndEffectorTests {
	static function close(actual:Float, expected:Float, label:String):Void
		if (Math.abs(actual - expected) > 1e-6) throw '$label: expected $expected, got $actual';

	static function fails(action:() -> Void, fragment:String):Void {
		try action() catch (e:Dynamic) {
			if (Std.string(e).indexOf(fragment) < 0) throw 'Expected "$fragment", got $e';
			return;
		}
		throw 'Expected failure containing "$fragment"';
	}

	public static function run():Void {
		var tool = new EndEffector();
		tool.addComponent("plate", new EndEffectorTestPart("PLATE", 2));
		tool.addComponent("cup", new EndEffectorTestPart("CUP", 1));
		tool.mount("plate", "mount");
		tool.addMate("cup-mate", "fixed", "plate", "tip", "cup", "mount");
		tool.addMemberConnector("cup", "contact", AssemblyFrames.translation(0, 10, 0));
		tool.addBomItem({partNumber: "TUBE", description: "Attached tube", quantity: 1,
			material: "polyurethane"}, 1, Attached(1, "cup", new Vector(0, 10, 0)));
		tool.workingFrame("contact", "cup", "contact", true);
		tool.validate();
		var tcp = tool.mountTFrame("contact");
		close(tcp.raw().y, 30, "contact y");
		var mass = tool.massPropertiesAtMount();
		close(mass.mass, 4, "mass");
		close(mass.centreOfMass.y, 12.5, "centre y");
		var robot = EndEffectorFrames.toRobotFrame(tcp);
		close(robot.position.z, 0.03, "robot metres");
		var approach = EndEffectorFrames.approachYToZ(tcp.raw());
		var axis = AssemblyFrames.transformVector(approach, 0, 0, 1);
		close(axis.z, 1, "robot approach rotation");

		var rotated = new EndEffector();
		var half = Math.sqrt(0.5);
		rotated.addComponent("plate", new EndEffectorTestPart("ROTATED", 2,
			{x: 10, y: 0, z: 0, qx: 0, qy: 0, qz: half, qw: half}));
		rotated.mount("plate", "mount");
		rotated.workingFrame("tip", "plate", "tip", true);
		rotated.validate();
		var rotatedFrame = rotated.mountTFrame("tip");
		close(rotatedFrame.raw().x, 20, "rotated TCP x");
		close(rotatedFrame.raw().y, 10, "rotated TCP y");
		var rotatedMass = rotated.massPropertiesAtMount();
		close(rotatedMass.centreOfMass.x, 0, "rotated mass x");
		close(rotatedMass.centreOfMass.y, 10, "rotated mass y");
		if (rotatedMass.inertia == null) throw "Expected rotated inertia";
		var inertia = rotatedMass.inertia;
		close(inertia.xx, 2, "rotated inertia xx");
		close(inertia.yy, 1, "rotated inertia yy");

		var broken = new EndEffector();
		broken.addComponent("plate", new EndEffectorTestPart("BROKEN", 1));
		broken.workingFrame("bad", "absent", "tip");
		var findings = broken.check().items;
		if (findings.length != 2 || findings[0].code != "eoat.missing-mount" ||
			findings[1].code != "eoat.invalid-frame" || findings[1].subject != "bad")
			throw "End effector diagnostics must collect mount and frame faults";
		broken.diagnostics.error("transmission.parts", "plate", "A source part cannot rebuild");
		var copiedBroken = new EndEffector();
		broken.copyInto(copiedBroken);
		if (copiedBroken.diagnostics.items.length != 1 ||
			copiedBroken.diagnostics.items[0].code != "transmission.parts")
			throw "End effector copies must retain diagnostic findings";

		var missing = new EndEffector();
		missing.addComponent("plate", new EndEffectorTestPart("PLATE", 1));
		fails(() -> missing.validate(), "mount");
		var floating = new EndEffector();
		floating.addComponent("plate", new EndEffectorTestPart("PLATE", 1));
		floating.addComponent("cup", new EndEffectorTestPart("CUP", 1));
		floating.mount("plate", "mount");
		fails(() -> floating.validate(), "not attached");
		var unknown = new EndEffector();
		unknown.addComponent("plate", new EndEffectorTestPart("PLATE", 1));
		unknown.mount("plate", "mount");
		unknown.workingFrame("bad", "plate", "missing");
		fails(() -> unknown.validate(), "Unknown connector");
	}
}
