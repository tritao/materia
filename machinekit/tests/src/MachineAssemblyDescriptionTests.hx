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
import machinekit.pneumatic.SuctionCup;
import machinekit.component.PortInterface;
import materia.assembly.AssemblyFrames;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
import machinekit.robotics.schmalz.SchmalzSxtMaster;
import machinekit.robotics.schmalz.SchmalzSxtTool;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.transmission.GearPair;
import machinekit.transmission.SpurGear;
import machinekit.assembly.FlangeBearingAssembly;
import machinekit.standard.DeepGrooveBearing;
import machinekit.assembly.LinearAxis;
import pickingstation.StorageRack;
import pickingstation.PickingStationConfig;

class MachineAssemblyDescriptionTests {
	public static function main():Void {
		run();
	}

	public static function roundTrip(assembly:MachineAssembly, label:String, massAvailable:Bool = true):Void {
		var rebuilt = MachineAssembly.decode(assembly.encode());
		if (!Equality.equals(assembly.describe(), rebuilt.describe()))
			throw '$label description changed after round trip';
		if (!Equality.equals(assembly.billOfMaterials().lines(), rebuilt.billOfMaterials().lines()))
			throw '$label BOM changed after round trip';
		if (!Equality.equals(assembly.check().items, rebuilt.check().items))
			throw '$label diagnostics changed after round trip';
		if (massAvailable) {
			var originalMass = assembly.massProperties(), restoredMass = rebuilt.massProperties();
			close(originalMass.mass, restoredMass.mass, '$label mass');
			close(originalMass.centreOfMass.x, restoredMass.centreOfMass.x, '$label mass centre x');
			close(originalMass.centreOfMass.y, restoredMass.centreOfMass.y, '$label mass centre y');
			close(originalMass.centreOfMass.z, restoredMass.centreOfMass.z, '$label mass centre z');
			if (!Equality.equals(originalMass.unaccounted, restoredMass.unaccounted) ||
				!Equality.equals(originalMass.unaccountedInertia, restoredMass.unaccountedInertia))
				throw '$label mass accounting changed after round trip';
			if ((originalMass.inertia == null) != (restoredMass.inertia == null))
				throw '$label inertia availability changed after round trip';
			if (originalMass.inertia != null) {
				var first = requireInertia(originalMass.inertia);
				var second = requireInertia(restoredMass.inertia);
				close(first.xx, second.xx, '$label inertia xx');
				close(first.xy, second.xy, '$label inertia xy');
				close(first.xz, second.xz, '$label inertia xz');
				close(first.yy, second.yy, '$label inertia yy');
				close(first.yz, second.yz, '$label inertia yz');
				close(first.zz, second.zz, '$label inertia zz');
			}
		}
		var poses = assembly.solvedPoses(), restored = rebuilt.solvedPoses();
		for (member in assembly.components())
			if (!Equality.equals(poses.get(member.id), restored.get(member.id)))
				throw '$label pose changed for ${member.id}';
	}

	static function close(a:Float, b:Float, label:String):Void
		if (Math.abs(a - b) > 1e-7 * Math.max(1, Math.abs(a)))
			throw '$label changed: $a versus $b';

	static function requireInertia(value:Null<cadkit.InertiaTensor>):cadkit.InertiaTensor
	{
		if (value != null) return value;
		throw "Missing inertia";
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
		var flangePart = new RobotFlange(50).bom.partNumber;
		var beforeEdit = rebuilt.billOfMaterials().quantity(flangePart);
		rebuilt.addComponent("third", new RobotFlange(50));
		if (rebuilt.billOfMaterials().quantity(flangePart) != beforeEdit + 1)
			throw "Editing a rebuilt assembly did not update its evaluated BOM";
		nestedRoundTrip();
		includedConnectorRuntime();
		suctionInterfaceRoundTrip();
		documentRoundTrip();
		changerDocumentRoundTrip();
		mechanicalRecords();
		libraryRoundTrips();
		EndEffectorSetTests.run();
		MachineKitSmoke.massProperties();
		MachineKitSmoke.ports();
	}

	static function includedConnectorRuntime():Void {
		var inner = new MachineAssembly();
		inner.addComponent("part", new RobotFlange(50));
		var outer = new MachineAssembly();
		outer.addComponent("base", new RobotFlange(50));
		outer.include("unit", inner);
		outer.addMemberConnector("unit/part", "outerMount", AssemblyFrames.identity());
		outer.addMate("attach", "fixed", "base", "face", "unit/part", "outerMount");
		if (outer.solvedPoses().get("unit/part") == null) throw "Outer connector was lost during pose solve";
		if (outer.massProperties().mass <= 0) throw "Outer connector was lost during mass solve";
	}

	static function suctionInterfaceRoundTrip():Void {
		var assembly = new MachineAssembly();
		assembly.addComponent("cup", new SuctionCup(20, 10, null, null, null, PushIn(4)));
		var oldDescription = assembly.describe();
		oldDescription.machine.portBridges = [{occurrence: "cup", fromPort: "old", toPort: "vacuum"}];
		MachineAssembly.fromDescription(oldDescription);
		var rebuilt = MachineAssembly.decode(assembly.encode());
		switch rebuilt.components()[0].component.port("vacuum").iface {
			case PushIn(size): if (size != 4) throw "Suction cup port diameter changed";
			case _: throw "Suction cup port interface changed";
		}
	}

	static function nestedRoundTrip():Void {
		var inner = new MachineAssembly();
		inner.addComponent("first", new RobotFlange(50));
		inner.addComponent("second", new RobotFlange(50));
		inner.addMate("join", "fixed", "first", "face", "second", "face");
		inner.addConstraint("closure", "fixed", "first", "face", "second", "face", 0.2);
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
		if (flat.occurrences.length != outer.components().length || flat.joints.length != 3)
			throw "Nested assembly did not flatten to the original members and joints";
		var expected = ["base", "unit/pair/first", "unit/pair/second"];
		for (index in 0...expected.length)
			if (flat.occurrences[index].id != expected[index])
				throw 'Nested member path changed at $index';
		if (flat.joints[0].id != "unit/pair/join" ||
			flat.joints[1].id != "unit/pair/closure" || flat.joints[1].closureTolerance != 0.2 ||
			flat.joints[2].id != "attach")
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
		if (restored.upstreamChain("generator", "air")[1] != "manifold/out1")
			throw "MachineKit document changed the service source";
		var count = 0;
		for (relationship in reopened.allRelationships()) if (relationship.typeName == MachineAssemblyDocuments.PORT_CONNECTION) {
			count++;
			relationship.setProperty(TypedProperty.text("machinekit.assembly.fromPort", "out2"));
		}
		if (count != 1) throw "Port connection was not saved as a document relationship";
		var edited = MachineAssemblyDocuments.rebuildAssembly(loaded);
		if (edited.describe().machine.portConnections[0].fromPort != "out2" ||
			edited.upstreamChain("generator", "air")[1] != "manifold/out2")
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
		if (restored.configuration("manual").upstream("tool/half", "airOut1").port.instanceId !=
			"robot/master") throw "EOAT service path changed after document round trip";
		reopened.close();
		document.close();
	}

	static function mechanicalRecords():Void {
		var assembly = new MachineAssembly();
		for (id in ["a", "b", "c"]) assembly.addComponent(id, new RobotFlange(50));
		assembly.addMateOnAxis("second", "prismatic", "b", "face", "c", "face",
			{x: 0, y: 1, z: 0});
		assembly.addMateOnAxis("first", "prismatic", "a", "face", "b", "face",
			{x: 0, y: 1, z: 0});
		assembly.addCoupling("linked", "first", "second", 2);
		var definition = assembly.describe().mechanical;
		if (definition.joints.length != 2 || definition.joints[0].id != "second" ||
			definition.joints[1].id != "first" || definition.couplings[0].id != "linked")
			throw "Builder did not retain authored mechanical records";
		var model = new AssemblyModel();
		assembly.addTo(model, "");
		var modelState = model.initialState(), savedState = new AssemblyState(definition);
		modelState.setJoint("first", 5);
		savedState.setJoint("first", 5);
		for (id in ["a", "b", "c"])
			if (!Equality.equals(modelState.worldPose(id), savedState.worldPose(id)))
				throw 'Saved definition changed the pose of "$id"';
		assembly.addMate("extra", "fixed", "a", "face", "c", "face");
		var found = false;
		for (item in assembly.check().items) if (item.code == "assembly.multiple-parents" && item.subject == "c")
			found = true;
		if (!found) throw "Direct mechanical records lost the multiple-parent diagnostic";
		var closure = new MachineAssembly();
		closure.addComponent("root", new RobotFlange(50));
		closure.addComponent("offset", new RobotFlange(50),
			materia.assembly.AssemblyFrames.translation(10, 0, 0));
		closure.addConstraint("check", "fixed", "root", "face", "offset", "face", 0.1);
		if (!rejectsClosure(closure) ||
			closure.describe().mechanical.joints[0].closureTolerance != 0.1 ||
			!rejectsClosure(MachineAssembly.decode(closure.encode())))
			throw "Saved definition lost closure tolerance";
		var document = new Document();
		var root = MachineAssemblyDocuments.defineAssembly(document, closure);
		var reopened = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		var restored = MachineAssemblyDocuments.rebuildAssembly(reopened.element(root.id));
		if (restored.describe().mechanical.joints[0].closureTolerance != 0.1 ||
			!rejectsClosure(restored))
			throw "Document lost closure tolerance";
		reopened.close();
		document.close();
		var defaultClosure = new MachineAssembly();
		defaultClosure.addComponent("a", new RobotFlange(50));
		defaultClosure.addComponent("b", new RobotFlange(50), AssemblyFrames.translation(0.002, 0, 0));
		defaultClosure.addConstraint("default", "fixed", "a", "face", "b", "face");
		if (defaultClosure.describe().mechanical.joints[0].closureTolerance != null ||
			!rejectsClosure(defaultClosure))
			throw "Default closure tolerance was stored or not checked";
		var metreModel = new AssemblyModel("m");
		metreModel.add("a"); metreModel.add("b", AssemblyFrames.translation(0.000002, 0, 0));
		metreModel.connector("a", "point", AssemblyFrames.identity());
		metreModel.connector("b", "point", AssemblyFrames.identity());
		metreModel.constrain("default", "fixed", "a", "point", "b", "point");
		if (metreModel.definition().joints[0].closureTolerance != null)
			throw "AssemblyModel stored its default closure tolerance";
		var metreRejected = false;
		try metreModel.solve() catch (error:String)
			metreRejected = error.indexOf("separated connectors") >= 0;
		if (!metreRejected) throw "Metre closure did not use the length-unit default";
	}

	static function rejectsClosure(assembly:MachineAssembly):Bool {
		try assembly.solvedPoses() catch (error:String)
			return error.indexOf("separated connectors") >= 0;
		return false;
	}

	static function libraryRoundTrips():Void {
		roundTrip(GearPair.mesh(new SpurGear(2, 18, 12), new SpurGear(2, 20, 12)), "gear pair");
		roundTrip(new FlangeBearingAssembly(DeepGrooveBearing.metric("6204")), "flange bearing");
		roundTrip(new LinearAxis(), "linear axis");
		// Picking Station components provide geometry but do not declare hasGeometry or mass.
		roundTrip(new StorageRack(PickingStationConfig.defaults()), "storage rack", false);
	}
}
