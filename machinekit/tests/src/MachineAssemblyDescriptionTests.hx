import haxeon.Equality;
import machinekit.assembly.MachineAssembly;
import machinekit.robotics.RobotFlange;
import materia.assembly.AssemblyDefinitionFlattener;

class MachineAssemblyDescriptionTests {
	public static function main():Void {
		run();
	}

	public static function roundTrip(assembly:MachineAssembly, label:String):Void {
		var rebuilt = MachineAssembly.decode(assembly.encode());
		if (!Equality.equals(assembly.describe(), rebuilt.describe()))
			throw '$label description changed after round trip';
		if (!Equality.equals(assembly.billOfMaterials().lines(), rebuilt.billOfMaterials().lines()))
			throw '$label BOM changed after round trip';
		var poses = assembly.solvedPoses(), restored = rebuilt.solvedPoses();
		for (member in assembly.components())
			if (!Equality.equals(poses.get(member.id), restored.get(member.id)))
				throw '$label pose changed for ${member.id}';
	}

	public static function run():Void {
		var assembly = new MachineAssembly();
		assembly.addComponent("first", new RobotFlange(50));
		assembly.addComponent("second", new RobotFlange(50));
		assembly.addMate("join", "fixed", "first", "face", "second", "face");
		assembly.exposeConnector("mount", "first", "face");
		assembly.addBomItem({partNumber: "EXTRA", description: "Unmodelled extra",
			quantity: 1, material: null});
		if (assembly.describe().mechanical.definitions.length != 1)
			throw "Identical recipe parts should share a mechanical definition";
		var detached = assembly.describe();
		detached.mechanical.occurrences[0].initialPose.x = 42;
		if (assembly.describe().mechanical.occurrences[0].initialPose.x == 42)
			throw "Description retained a mutable connector or pose from the builder";
		var encoded = assembly.encode();
		var rebuilt = MachineAssembly.decode(encoded);
		if (!Equality.equals(assembly.describe(), rebuilt.describe()))
			throw "Machine assembly description changed after wire round trip";
		if (rebuilt.components().length != 2)
			throw "Machine assembly lost a member";
		if (rebuilt.connectorNames()[0] != "mount" || rebuilt.billOfMaterials().lines().length != 2)
			throw "Machine assembly lost its exposure or BOM extra";
		nestedRoundTrip();
	}

	static function nestedRoundTrip():Void {
		var inner = new MachineAssembly();
		inner.addComponent("first", new RobotFlange(50));
		inner.addComponent("second", new RobotFlange(50));
		inner.addMate("join", "fixed", "first", "face", "second", "face");
		inner.exposeConnector("mount", "first", "face");
		var middle = new MachineAssembly();
		middle.include("pair", inner);
		middle.exposeConnector("mount", "pair/first", "face");
		var outer = new MachineAssembly();
		outer.addComponent("base", new RobotFlange(50));
		outer.include("unit", middle);
		outer.addMate("attach", "fixed", "base", "face", "unit/pair/first", "face");
		outer.exposeConnector("tip", "unit/pair/second", "face");
		var description = outer.describe();
		if (description.mechanical.assemblies == null || description.mechanical.assemblies.length != 2)
			throw "Included assemblies were not preserved as nested definitions";
		if (description.mechanical.occurrences.length != 2)
			throw "Included members were not grouped into an assembly occurrence";
		var flat = AssemblyDefinitionFlattener.flatten(description.mechanical);
		if (flat.occurrences.length != outer.components().length || flat.joints.length != 2)
			throw "Nested assembly did not flatten to the original members and joints";
		var expected = ["base", "unit/pair/first", "unit/pair/second"];
		for (index in 0...expected.length)
			if (flat.occurrences[index].id != expected[index])
				throw 'Nested member path changed at $index';
		if (flat.joints[0].id != "unit/pair/join" || flat.joints[1].id != "attach")
			throw "Nested joint paths changed during flattening";
		roundTrip(outer, "nested assembly");
	}
}
