import haxeon.Equality;
import machinekit.assembly.MachineAssembly;
import machinekit.robotics.RobotFlange;
import materia.assembly.AssemblyDefinitionFlattener;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.Relationship;
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
		var rebuilt:MachineAssembly = Std.isOfType(assembly, EndEffector)
			? EndEffector.fromDescription(haxeon.wire.JsonWire.decode(assembly.encode()))
			: MachineAssembly.decode(assembly.encode());
		if (!Equality.equals(assembly.describe(), rebuilt.describe())) {
			var before = assembly.encode(), after = rebuilt.encode(), offset = 0;
			while (offset < before.length && offset < after.length && before.charAt(offset) == after.charAt(offset)) offset++;
			throw '$label description changed after round trip at $offset: ' + before.substr(0, offset + 160) + ' versus ' + after.substr(0, offset + 160);
		}
		var originalBom = assembly.billOfMaterials().lines();
		var rebuiltBom = rebuilt.billOfMaterials().lines();
		originalBom.sort((a, b) -> Reflect.compare(a.partNumber, b.partNumber));
		rebuiltBom.sort((a, b) -> Reflect.compare(a.partNumber, b.partNumber));
		if (!Equality.equals(originalBom, rebuiltBom))
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
		if (Std.isOfType(assembly, EndEffector)) {
			var originalPieces = cadbridge.EndEffectorCollision.pieces(cast assembly);
			var rebuiltPieces = cadbridge.EndEffectorCollision.pieces(cast rebuilt);
			if (!Equality.equals(collisionSignature(originalPieces), collisionSignature(rebuiltPieces)))
				throw '$label collision pieces changed after round trip';
		}
	}

	static function collisionSignature(result:cadbridge.EndEffectorCollision.EndEffectorCollisionResult):Array<String> {
		var signatures:Array<String> = [];
		for (piece in result.pieces) {
			var members = piece.memberIds.copy();
			members.sort(Reflect.compare);
			var vertices:Array<String> = [];
			for (index in 0...Std.int(piece.vertices.length / 3)) {
				var at = index * 3;
				vertices.push(Std.string(Math.round(piece.vertices[at] * 1e7)) + "," +
					Std.string(Math.round(piece.vertices[at + 1] * 1e7)) + "," +
					Std.string(Math.round(piece.vertices[at + 2] * 1e7)));
			}
			vertices.sort(Reflect.compare);
			signatures.push(members.join(",") + ":" + vertices.join(";"));
		}
		signatures.sort(Reflect.compare);
		return signatures;
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
		var editableCopy = machinekit.assembly.FrozenAssemblyDefinitions.thaw(detached.mechanical);
		editableCopy.occurrences[0].initialPose.x = 42;
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
		fullEoatDocumentRoundTrip();
		documentEditsAndUndo();
		recipeAssemblyReconcile();
		mechanicalRecords();
		libraryRoundTrips();
		EndEffectorSetTests.run();
		MachineKitSmoke.massProperties();
		MachineKitSmoke.ports();
		RecipeContractTests.run();
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
		var restored = MachineAssembly.decode(outer.encode());
		if (!Equality.equals(outer.solvedPoses().get("unit/part"), restored.solvedPoses().get("unit/part")))
			throw "Outer connector was lost during nested round trip";
		if (restored.subassemblies().length != 1 || restored.subassemblies()[0].id != "unit")
			throw "Included builder was not restored";
	}

	static function suctionInterfaceRoundTrip():Void {
		var assembly = new MachineAssembly();
		assembly.addComponent("cup", new SuctionCup(20, 10, null, null, null, PushIn(4)));
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
		var flat = AssemblyDefinitionFlattener.flatten(machinekit.assembly.FrozenAssemblyDefinitions.thaw(description.mechanical));
		if (flat.occurrences.length != outer.components().length || flat.joints.length != 3)
			throw "Nested assembly did not flatten to the original members and joints";
		var expected = ["base", "unit/pair/first", "unit/pair/second"];
		for (index in 0...expected.length)
			if (flat.occurrences[index].id != expected[index])
				throw 'Nested member path changed at $index';
		var jointById = new Map<String, materia.assembly.AssemblyDefinition.KinematicJoint>();
		for (joint in flat.joints) jointById.set(joint.id, joint);
		var nestedClosure = jointById.get("unit/pair/closure");
		if (!jointById.exists("unit/pair/join") ||
			nestedClosure == null || nestedClosure.closureTolerance != 0.2 ||
			!jointById.exists("attach"))
			throw "Nested joint paths changed during flattening";
		roundTrip(outer, "nested assembly");
		var document = new Document();
		var root = MachineAssemblyDocuments.defineAssembly(document, outer);
		var reopened = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		var restored = MachineAssemblyDocuments.rebuildAssembly(reopened.element(root.id));
		if (!Equality.equals(outer.describe(), restored.describe()))
			throw "Nested MachineKit assembly changed after document round trip";
		var nestedInstance:InstanceElement = null;
		for (element in reopened.allElements()) if (element.kind == "instance") {
			var instance:InstanceElement = cast element;
			if (reopened.definition(instance.definitionId).recipe ==
				cadkit.parametric.AssemblyMemberEvaluator.NESTED_RECIPE &&
				instance.name == "unit") nestedInstance = instance;
		}
		if (nestedInstance == null) throw "Nested assembly is not a document instance";
		var nestedShape = reopened.definitionOutput(nestedInstance, "body");
		if (nestedShape.volume() <= 1) throw "Nested assembly retained placeholder geometry";
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

	static function documentEditsAndUndo():Void {
		var assembly = new MachineAssembly();
		assembly.addComponent("base", new RobotFlange(50));
		assembly.addComponent("slider", new RobotFlange(50));
		assembly.addMateOnAxis("slide", "prismatic", "base", "face", "slider", "face",
			{x: 0, y: 1, z: 0});
		var document = new Document();
		MachineAssemblyDocuments.defineAssembly(document, assembly);
		var root = MachineAssemblyDocuments.defineAssembly(document, assembly);
		var roots = 0;
		for (element in document.allElements()) {
			var kind = element.property("cadkit.assembly.kind");
			if (kind != null && kind.value == "root") roots++;
		}
		if (roots != 1) throw "Saving twice created duplicate assembly roots";
		if (!document.undo()) throw "Saving did not create an undo step";
		if (!document.redo()) throw "Saving did not create a redo step";
		var edited = false;
		for (relationship in document.allRelationships()) if (relationship.typeName == "cadkit.joint") {
			relationship.setProperty(TypedProperty.quantity("cadkit.assembly.limitUpper",
				cadkit.parametric.QuantityKind.Scalar, 12, "1"));
			edited = true;
		}
		if (!edited) throw "Joint was not saved as a relationship";
		var description = MachineAssemblyDocuments.describeAssembly(document.element(root.id));
		var upper = description.mechanical.joints[0].limits.upper;
		if (upper == null || upper != 12.0)
			throw "Typed joint limit edit was lost";
		var victim:InstanceElement = null;
		for (element in document.allElements()) {
			var id = element.property("cadkit.assembly.id");
			if (id != null && id.value == "slider") victim = cast element;
		}
		if (victim == null) throw "Assembly member is not an instance";
		var missing = false;
		try document.removeElement(victim.id)
		catch (error:Dynamic) missing = Std.string(error).indexOf("target of relationship") >= 0;
		if (!missing) throw "Deleting a joint endpoint silently discarded a relationship";
		document.close();
		var free = new MachineAssembly();
		free.addComponent("part", new RobotFlange(50));
		var movedDocument = new Document();
		var movedRoot = MachineAssemblyDocuments.defineAssembly(movedDocument, free);
		for (element in movedDocument.allElements()) {
			var id = element.property("cadkit.assembly.id");
			if (id != null && id.value == "part")
				element.setPlacement(new cadkit.parametric.Placement(new cadkit.modeling.Plane(
					new cadkit.modeling.Vector(25, 0, 0), cadkit.modeling.Vector.X(), cadkit.modeling.Vector.Z())));
		}
		var movedPose = MachineAssemblyDocuments.rebuildAssembly(movedRoot).solvedPoses().get("part");
		if (movedPose == null || movedPose.x != 25)
			throw "Moving a document instance did not move the rebuilt assembly";
		movedDocument.close();
	}

	static function recipeAssemblyReconcile():Void {
		var oldAssembly = new MachineAssembly();
		oldAssembly.addComponent("manifold", new PneumaticManifold(2));
		var saved = new Document();
		MachineAssemblyDocuments.defineAssembly(saved, oldAssembly);
		var savedText = DocumentCodec.encode(saved);
		var freshAssembly = new MachineAssembly();
		freshAssembly.addComponent("manifold", new PneumaticManifold(3));
		var fresh = new Document();
		var root = MachineAssemblyDocuments.defineAssembly(fresh, freshAssembly);
		var diagnostics:Array<String> = [];
		machinekit.document.MachineKitRecipes.reconcileDocument(fresh, savedText, diagnostics);
		var rebuilt = MachineAssemblyDocuments.rebuildAssembly(root);
		if (rebuilt.components()[0].component.ports().length != 4)
			throw "Reconciled assembly did not use current recipe inputs";
		fresh.close();
		saved.close();
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
		tool.addComponent("generator", new VacuumGenerator(null, null, Thread("G1/8-M")));
		tool.addMate("generator-mount", "fixed", "half", "payload", "generator", "mount");
		tool.connectPorts("tool-air", "half", "airOut1", "generator", "air");
		tool.mount("half", "master");
		tool.exposePort("air", "half", "airIn1");
		set.addTool("manual", tool);
		var document = new Document();
		var root = MachineAssemblyDocuments.defineAssembly(document, set);
		var reopened = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		var toolConnections = 0;
		for (relationship in reopened.allRelationships()) if (relationship.typeName == MachineAssemblyDocuments.PORT_CONNECTION &&
			relationship.property("machinekit.assembly.tool") != null) toolConnections++;
		if (toolConnections != 1) throw "Tool port connection was not saved as a scoped relationship";
		var restored = EndEffectorSet.fromDescription(
			MachineAssemblyDocuments.describeAssembly(reopened.element(root.id)));
		if (!Equality.equals(set.describe(), restored.describe())) {
			var before = set.encode(), after = restored.encode(), offset = 0;
			while (offset < before.length && offset < after.length && before.charAt(offset) == after.charAt(offset)) offset++;
			throw "EOAT set changed after document round trip at " + offset + ": " +
				before.substr(offset, 150) + " versus " + after.substr(offset, 150);
		}
		var originalLines = set.configuration("manual").billOfMaterials().lines();
		var restoredLines = restored.configuration("manual").billOfMaterials().lines();
		originalLines.sort((a, b) -> Reflect.compare(a.partNumber, b.partNumber));
		restoredLines.sort((a, b) -> Reflect.compare(a.partNumber, b.partNumber));
		if (!Equality.equals(originalLines, restoredLines))
			throw "EOAT BOM changed after document round trip";
		if (restored.configuration("manual").upstream("tool/half", "airOut1").port.instanceId !=
			"robot/master") throw "EOAT service path changed after document round trip";
		reopened.close();
		document.close();
	}

	static function fullEoatDocumentRoundTrip():Void {
		var original = eoat.EndEffectorExample.build();
		var document = new Document();
		var root = MachineAssemblyDocuments.defineAssembly(document, original);
		var reopened = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		var restored:EndEffectorSet = cast MachineAssemblyDocuments.rebuildAssembly(reopened.element(root.id));
		if (!Equality.equals(original.describe(), restored.describe()))
			throw "EOAT document changed the frozen description";
		for (name in ["short", "long"]) {
			var before = original.configuration(name), after = restored.configuration(name);
			var massBefore = before.massPropertiesAtMount(), massAfter = after.massPropertiesAtMount();
			close(massBefore.mass, massAfter.mass, 'EOAT $name document mass');
			if (!Equality.equals(before.mountTFrame("contact"), after.mountTFrame("contact")))
				throw 'EOAT $name document TCP changed';
			if (!Equality.equals(before.upstream("tool/cup", "vacuum"), after.upstream("tool/cup", "vacuum")))
				throw 'EOAT $name document upstream path changed';
			if (!Equality.equals(collisionSignature(cadbridge.EndEffectorCollision.pieces(before)),
				collisionSignature(cadbridge.EndEffectorCollision.pieces(after))))
				throw 'EOAT $name document collision pieces changed';
		}
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
		if (definition.joints.length != 2 || definition.joints[0].id != "first" ||
			definition.joints[1].id != "second" || definition.couplings[0].id != "linked")
			throw "Builder did not retain authored mechanical records";
		var model = new AssemblyModel();
		assembly.addTo(model, "");
		var modelState = model.initialState(), savedState = new AssemblyState(machinekit.assembly.FrozenAssemblyDefinitions.thaw(definition));
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
		if (new LinearAxis().massProperties().unaccounted.length != 0)
			throw "Linear axis has unaccounted rail mass";
		roundTrip(new StorageRack(PickingStationConfig.defaults()), "storage rack");
		roundTrip(eoat.SchmalzEndEffectorExample.build(), "Schmalz end effector");
		var eoatSet = eoat.EndEffectorExample.build();
		roundTrip(eoatSet.configuration("short"), "EOAT short configuration");
		roundTrip(eoatSet.configuration("long"), "EOAT long configuration");
	}
}
