import haxeon.Equality;
import machinekit.assembly.MachineAssembly;
import machinekit.robotics.RobotFlange;
import materia.assembly.AssemblyDefinitionFlattener;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.TypedProperty;
import machinekit.document.MachineAssemblyDocuments;
import machinekit.pneumatic.PneumaticManifold;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
import machinekit.robotics.schmalz.SchmalzSxtMaster;
import machinekit.robotics.schmalz.SchmalzSxtTool;

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
		documentRoundTrip();
		changerDocumentRoundTrip();
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
		var document = new Document();
		var root = MachineAssemblyDocuments.defineAssembly(document, outer);
		var reopened = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		var restored = MachineAssemblyDocuments.rebuildAssembly(reopened.element(root.id));
		if (!Equality.equals(outer.describe(), restored.describe()))
			throw "Nested MachineKit assembly changed after document round trip";
		reopened.close();
		document.close();
	}

	static function documentRoundTrip():Void {
		var assembly = new MachineAssembly();
		assembly.addComponent("manifold", new PneumaticManifold(2));
		assembly.addComponent("generator", new VacuumGenerator());
		assembly.connectPorts("air", "manifold", "out1", "generator", "air");
		assembly.exposePort("supply", "manifold", "input");
		var document = new Document();
		var root = MachineAssemblyDocuments.defineAssembly(document, assembly);
		var reopened = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		var loaded = reopened.element(root.id);
		var restored = MachineAssemblyDocuments.rebuildAssembly(loaded);
		if (!Equality.equals(assembly.describe(), restored.describe()))
			throw "MachineKit document changed the assembly description";
		var count = 0;
		for (relationship in reopened.allRelationships()) if (relationship.typeName == MachineAssemblyDocuments.PORT_CONNECTION) {
			count++;
			relationship.setProperty(TypedProperty.text("machinekit.assembly.fromPort", "out2"));
		}
		if (count != 1) throw "Port connection was not saved as a document relationship";
		var edited = MachineAssemblyDocuments.rebuildAssembly(loaded);
		if (edited.describe().machine.portConnections[0].fromPort != "out2")
			throw "Editing a document port connection did not change the rebuilt assembly";
		reopened.close();
		document.close();
	}

	static function changerDocumentRoundTrip():Void {
		var set = new EndEffectorSet();
		set.addComponent("master", new SchmalzSxtMaster("10.07.13.00013"));
		set.mount("master", "robot");
		set.exposePort("robotAir", "master", "airIn1");
		set.exposePort("coupledAir", "master", "airOut1");
		set.changer("bayonet", "master", "tool", [{robot: "coupledAir", tool: "air"}]);
		var tool = new EndEffector();
		tool.addComponent("half", new SchmalzSxtTool("10.07.13.00018"));
		tool.mount("half", "master");
		tool.exposePort("air", "half", "airIn1");
		set.addTool("manual", tool);
		var document = new Document();
		var root = MachineAssemblyDocuments.defineAssembly(document, set);
		var reopened = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		var restored = EndEffectorSet.fromDescription(
			MachineAssemblyDocuments.describeAssembly(reopened.element(root.id)));
		if (!Equality.equals(set.describe(), restored.describe()) ||
			!Equality.equals(set.configuration("manual").billOfMaterials().lines(),
				restored.configuration("manual").billOfMaterials().lines()))
			throw "EOAT set changed after document round trip";
		reopened.close();
		document.close();
	}
}
