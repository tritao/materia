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
import machinekit.assembly.Drive;
import machinekit.assembly.DriveDefaults;
import machinekit.motion.ScrewSupport;
import machinekit.transmission.TimingBelt;
import machinekit.motion.NemaStepper;
import machinekit.assembly.MachineAssemblyDescription;
import machinekit.assembly.MachineAssemblyDescription.MemberSource;
import machinekit.assembly.MachineAssemblyDescription.SavedValue;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewThread;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.TimingBeltProfile;
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
		drivesFollowTheirParts();
		motorsDriveJoints();
		drivesCarryAllowances();
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

	/** A coupling's ratio comes from the parts that drive it, so editing a part changes it. */
	static function drivesFollowTheirParts():Void {
		var assembly = new MachineAssembly();
		assembly.addComponent("base", new RobotFlange(50));
		assembly.addComponent("slider", new RobotFlange(50));
		assembly.addComponent("screw", new LeadScrew(new LeadScrewThread(MetricTrapezoidal, 10, 2), 100));
		assembly.addComponent("pulley", new TimingPulley(TimingBeltProfile.GT2, 20, 5, 8));
		assembly.addComponent("driver", new SpurGear(1, 20, 6));
		assembly.addComponent("driven", new SpurGear(1, 40, 6));
		assembly.addMateOnAxis("slide", "prismatic", "base", "face", "slider", "face", {x: 0, y: 1, z: 0}, 10);
		for (turning in ["screw", "pulley", "driver", "driven"])
			assembly.addMateOnAxis('$turning-turn', "continuous", "base", "face", turning,
				turning == "screw" ? "input" : "axis", {x: 0, y: 1, z: 0});
		// A right-hand Tr10 x 2 screw: one turn moves its nut 2 mm back along the screw, from 10 mm.
		var lead = assembly.addDrive("lead", "slide", "screw-turn", Drive.LeadScrew("screw", 1), 10);
		var belt = assembly.addDrive("belt", "slide", "pulley-turn", Drive.Belt("pulley", -1), 10);
		var mesh = assembly.addDrive("mesh", "driver-turn", "driven-turn", Drive.GearMesh("driver", "driven", 1));
		function near(actual:Float, expected:Float, what:String):Void
			if (!(Math.abs(actual - expected) < 1e-12)) throw '$what: $actual, expected $expected';
		near(lead, -Math.PI, "a 2 mm right-hand lead turns the screw -pi rad per mm");
		near(belt, -Math.PI / 20, "a 20-tooth GT2 pulley (40 mm round) turns 1 / pitch radius per mm");
		near(mesh, -0.5, "a 20-tooth gear turns a 40-tooth one back at half speed");
		function ratio(machine:MachineAssembly, id:String):{ratio:Float, offset:Float} {
			for (coupling in machine.describe().mechanical.couplings) if (coupling.id == id)
				return {ratio: coupling.ratio, offset: coupling.offset};
			throw 'No coupling "$id"';
		}
		near(ratio(assembly, "lead").offset, 10 * Math.PI, "the screw is at zero where the slide is at 10 mm");
		// Saved, then rebuilt with a 4 mm pitch: the ratio follows the thread, not the saved number.
		var description = assembly.describe();
		var drives = description.machine.drives;
		if (drives == null || drives.length != 3) throw "The description lost its drives";
		var leadDrive = assembly.drive("lead");
		if (leadDrive == null) throw "The assembly lost its lead screw drive";
		if (leadDrive.kind != "lead-screw" || leadDrive.members[0] != "screw")
			throw "The lead screw drive names its screw";
		var edited:MachineAssemblyDescription = haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(description));
		edited.machine.members = [for (member in edited.machine.members) member.occurrence != "screw" ? member :
			{occurrence: member.occurrence, material: member.material, source: switch member.source {
				case Typed(id, values): Typed(id, [for (value in values) value.name != "pitch" ? value :
					{name: "pitch", value: SavedValue.Number(4)}]);
				case other: other;
			}}];
		var rebuilt = MachineAssembly.fromDescription(edited);
		near(ratio(rebuilt, "lead").ratio, -Math.PI / 2, "a rebuilt drive takes its ratio from the edited screw");
		near(ratio(rebuilt, "lead").offset, 10 * Math.PI / 2, "and keeps the screw at zero at 10 mm");
		near(ratio(rebuilt, "mesh").ratio, -0.5, "an unedited drive keeps its ratio");
		// Through a CadKit document, as the editor would change it.
		var document = new Document();
		var root = MachineAssemblyDocuments.defineAssembly(document, assembly);
		var screwInstance:InstanceElement = null;
		for (element in document.allElements()) {
			var id = element.property("cadkit.assembly.id");
			if (id != null && id.value == "screw") screwInstance = cast element;
		}
		if (screwInstance == null) throw "The screw is not a document instance";
		screwInstance.setOverride("pitch", 4);
		var reopened = MachineAssemblyDocuments.rebuildAssembly(document.element(root.id));
		near(ratio(reopened, "lead").ratio, -Math.PI / 2, "a document edit to the screw's pitch changes its drive");
		document.close();
	}

	/** A drive's allowances: a screw's critical speed, its nut's backlash and drag, a belt's stiffness. */
	static function drivesCarryAllowances():Void {
		function near(actual:Float, expected:Float, what:String, tolerance:Float = 1e-9):Void
			if (!(Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)))) throw '$what: $actual, expected $expected';
		var thread = new LeadScrewThread(MetricTrapezoidal, 10, 2);
		near(thread.rootDiameter(), 7.5, "a Tr10 x 2 thread's root is 7.5 mm");
		var rpm = 60 / (2 * Math.PI);
		var long = new LeadScrew(thread, 600);
		// Fixed at the motor, free at the far end: lambda 1.875 over 0.6 m at 80%, from d_r / 4 L^2 * sqrt(E / rho).
		near(long.criticalSpeed(Fixed, Free) * rpm, 706.1, "a 600 mm screw held at one end whips near 700 rpm", 1e-3);
		near(long.criticalSpeed(Fixed, Simple) * rpm, 3096.5, "a bearing at the far end lifts it over four times", 1e-3);
		near(long.criticalSpeed(Simple, Simple), long.criticalSpeed(Fixed, Free) * Math.PI * Math.PI / (1.875104069 * 1.875104069), "supports change lambda squared", 1e-6);
		near(new LeadScrew(thread, 300).criticalSpeed(Fixed, Free) * rpm, 4 * 706.1, "halving the length quadruples the speed", 1e-3);
		near(long.criticalSpeed(Fixed, Free, 300) * rpm, 4 * 706.1, "as does a support halfway", 1e-3);
		var caught = false;
		try long.criticalSpeed(Free, Free) catch (error:Dynamic) caught = true;
		if (!caught) throw "A screw with both ends free has nothing to hold it";

		var assembly = new MachineAssembly();
		assembly.addComponent("base", new RobotFlange(50));
		assembly.addComponent("slider", new RobotFlange(50));
		assembly.addComponent("screw", long);
		assembly.addMateOnAxis("slide", "prismatic", "base", "face", "slider", "face", {x: 0, y: 1, z: 0});
		assembly.addMateOnAxis("turn", "continuous", "base", "face", "screw", "input", {x: 0, y: 1, z: 0});
		assembly.addDrive("lead", "slide", "turn", Drive.LeadScrew("screw", 1));
		var cap = assembly.supportScrew("lead", Fixed, Free);
		near(cap, long.criticalSpeed(Fixed, Free), "the cap is the screw's critical speed");
		function definition(machine:MachineAssembly):materia.assembly.AssemblyDefinition {
			var model = new AssemblyModel("mm");
			machine.addTo(model, "");
			return model.definition("allowances");
		}
		function turnLimit(built:materia.assembly.AssemblyDefinition):Float {
			for (joint in built.joints) if (joint.id == "turn") {
				var velocity = joint.limits.velocity;
				if (velocity == null) throw "The screw's joint has no speed limit";
				return velocity;
			}
			throw "no screw joint";
		}
		var built = definition(assembly);
		near(turnLimit(built), cap, "the screw's joint is capped at it");
		var coupling = built.couplings[0];
		var backlash = coupling.backlash, drag = coupling.drag, stiffness = coupling.stiffness;
		near(backlash == null ? 0 : backlash, DriveDefaults.LEAD_SCREW_BACKLASH, "a screw drive has its nut's backlash allowance");
		near(drag == null ? 0 : drag, DriveDefaults.LEAD_SCREW_DRAG, "and its drag");
		if (stiffness != null) throw "A screw drive is rigid until it is given a stiffness";
		// Rebuilt from the description with a different screw length, the cap follows the part.
		var description:MachineAssemblyDescription = haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(assembly.describe()));
		var again = definition(MachineAssembly.fromDescription(description));
		near(turnLimit(again), cap, "a rebuilt screw keeps its cap");
		var driveRecord = assembly.drive("lead");
		if (driveRecord == null || driveRecord.nearSupport != "fixed" || driveRecord.farSupport != "free") throw "The drive records how its screw is held";
		caught = false;
		try assembly.supportScrew("lead", Free, Free) catch (error:Dynamic) caught = true;
		if (!caught) throw "Two free ends are refused";

		// A GT2 belt's stiffness: EA from its width, over the strand and the rest of the loop.
		var belt = TimingBelt.twoPulley(GT2, 20, 20, 444, 6);
		near(belt.carriageStiffness(0), 2500 * 6 * (1 / 444 + 1 / (belt.length - 444)), "a belt carriage's stiffness is EA over its two free lengths");
		near(belt.carriageStiffness(0) / TimingBelt.twoPulley(GT2, 20, 20, 444, 12).carriageStiffness(0), 0.5, "a wider belt is proportionally stiffer");
		assembly.addComponent("pulley", new TimingPulley(GT2, 20, 8, 6));
		assembly.addMateOnAxis("pulley-turn", "continuous", "base", "face", "pulley", "axis", {x: 0, y: 1, z: 0});
		assembly.addDrive("belt", "slide", "pulley-turn", Drive.Belt("pulley", 1));
		assembly.setDriveStiffness("belt", belt.carriageStiffness(0));
		var withBelt = definition(assembly);
		var beltCoupling = withBelt.couplings[1];
		var beltStiffness = beltCoupling.stiffness, beltDrag = beltCoupling.drag, beltBacklash = beltCoupling.backlash;
		near(beltStiffness == null ? 0 : beltStiffness, belt.carriageStiffness(0), "the belt's stiffness reaches its coupling");
		near(beltDrag == null ? 0 : beltDrag, DriveDefaults.BELT_DRAG, "with a belt's drag");
		if (beltBacklash != null) throw "A belt has no backlash";
	}

	/** A stepper's actuator comes from its ratings and supply, and follows the motor part. */
	static function motorsDriveJoints():Void {
		function near(actual:Float, expected:Float, what:String, tolerance:Float = 1e-12):Void
			if (!(Math.abs(actual - expected) < tolerance)) throw '$what: $actual, expected $expected';
		var motor = NemaStepper.frame(23);
		// 1.26 N m holding; on 24 V its 2.5 mH winding passes rated 2.8 A up to 24 / (50 x 2.5 mH x 2.8 A).
		var corner = 24 / (50 * 2.5e-3 * 2.8);
		near(motor.pullOutTorque(corner / 2, 24), 1.26, "below the corner speed a stepper pulls its holding torque");
		near(motor.pullOutTorque(2 * corner, 24), 0.63, "above it, its torque falls as 1 / speed");
		near(motor.usableTorque(), 0.63, "half the holding torque is the usable torque");
		near(motor.usableSpeed(24), 2 * corner, "and it holds that up to twice the corner speed");
		near(motor.usableSpeed(48), 4 * corner, "a higher supply keeps the torque to a higher speed");
		if (NemaStepper.frame(23, 70).rating() != null) throw "A generic-length motor has no rating";
		// A Tr10 x 2 thread with a 0.1 friction nut passes about 40% of the motor's work to the nut.
		var thread = new LeadScrewThread(MetricTrapezoidal, 10, 2);
		near(thread.efficiency(), 0.403, "a Tr10 x 2 screw is about 40% efficient", 0.002);
		near(thread.efficiency(0), 1, "a frictionless screw is lossless", 1e-12);
		if (!(new LeadScrewThread(MetricTrapezoidal, 10, 2, 4).efficiency() > thread.efficiency()))
			throw "A steeper lead is more efficient";

		var assembly = new MachineAssembly();
		assembly.addComponent("base", new RobotFlange(50));
		assembly.addComponent("slider", new RobotFlange(50));
		assembly.addComponent("motor", motor);
		assembly.addComponent("screw", new LeadScrew(thread, 100));
		assembly.addMateOnAxis("slide", "prismatic", "base", "face", "slider", "face", {x: 0, y: 1, z: 0});
		assembly.addMateOnAxis("turn", "continuous", "base", "face", "screw", "input", {x: 0, y: 1, z: 0});
		assembly.addDrive("lead", "slide", "turn", Drive.LeadScrew("screw", 1));
		assembly.addMotor("drive", "turn", "motor", 24);
		function definition(machine:MachineAssembly):materia.assembly.AssemblyDefinition {
			var model = new AssemblyModel("mm");
			machine.addTo(model, "");
			return model.definition("motorised");
		}
		function actuatorsOf(definition:materia.assembly.AssemblyDefinition):Array<materia.assembly.AssemblyDefinition.AssemblyActuator> {
			var actuators = definition.actuators;
			if (actuators == null) throw "The assembly has no actuators";
			return actuators;
		}
		function efficiencyOf(definition:materia.assembly.AssemblyDefinition):Float {
			var couplings = definition.couplings;
			if (couplings == null || couplings.length != 1) throw "The assembly has no coupling";
			var efficiency = couplings[0].efficiency;
			if (efficiency == null) throw "The coupling has no efficiency";
			return efficiency;
		}
		var built = definition(assembly);
		var actuators = actuatorsOf(built);
		if (actuators.length != 1 || actuators[0].joint != "turn") throw "The motor drives its joint";
		var rotor = actuators[0].rotorInertia;
		near(actuators[0].maxEffort, 0.63, "the actuator gets the usable torque");
		near(actuators[0].maxRate, 2 * corner, "and the usable speed");
		near(rotor == null ? 0 : rotor, 3.0e-5, "and the rotor's inertia");
		var steps = actuators[0].fullStepsPerRevolution;
		near(steps == null ? 0 : steps, 200, "and 200 full steps a turn from its 1.8 degree step");
		near(efficiencyOf(built), thread.efficiency(), "the lead screw's coupling carries its efficiency");
		var stepper = actuators[0];
		if (stepper.drive != "stepper") throw "A NEMA motor's actuator is a stepper drive";
		var holding = stepper.holdingTorque, curve = stepper.torqueSpeed;
		if (holding == null || curve == null) throw "A stepper's actuator has no pull-out curve";
		near(holding, 1.26, "a stepper records its holding torque");
		var pullOut = robotkit.model.TorqueSpeedCurve.unflatten(curve);
		near(pullOut.torqueAt(corner / 2), 1.26, "its curve holds the holding torque below the corner speed");
		near(pullOut.torqueAt(2 * corner), 0.63, "and the usable torque at the usable speed");
		near(pullOut.torqueAt(4 * corner), 0.315, "and falls as 1 / speed beyond");
		near(pullOut.torqueAt(1.8 * corner), motor.pullOutTorque(1.8 * corner, 24), "within 1% of the motor's pull-out torque between points", 0.01 * 1.26);
		near(pullOut.torqueAt(7 * corner), motor.pullOutTorque(7 * corner, 24), "including the tail", 0.01 * 1.26);
		// A servo part drives a joint the same way, through the MotorDrive hook: its peak torque and maximum speed
		// are what a planner relies on, and its rated torque bounds the average.
		var servoMachine = new MachineAssembly();
		servoMachine.addComponent("base", new RobotFlange(50));
		servoMachine.addComponent("servo", new TestServo());
		servoMachine.addMateOnAxis("turn", "continuous", "base", "face", "servo", "shaft", {x: 0, y: 1, z: 0});
		servoMachine.addMotor("servo-drive", "turn", "servo", 48);
		var servoActuator = actuatorsOf(definition(servoMachine))[0];
		if (servoActuator.drive != "servo") throw "A servo part's actuator is a servo drive";
		near(servoActuator.maxEffort, 1.9, "a servo's usable effort is its peak torque");
		near(servoActuator.maxRate, 500, "and its usable rate its maximum speed");
		var ratedTorque = servoActuator.ratedTorque, encoder = servoActuator.encoderCounts, servoSteps = servoActuator.fullStepsPerRevolution;
		near(ratedTorque == null ? 0 : ratedTorque, 0.64, "its rated torque is kept");
		near(encoder == null ? 0 : encoder, 4096, "and its encoder counts");
		if (servoSteps != null) throw "A servo has no full steps";
		// Rebuilt with the NEMA 17 in the motor's place, the actuator follows the motor.
		var description:MachineAssemblyDescription = haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(assembly.describe()));
		description.machine.members = [for (member in description.machine.members) member.occurrence != "motor" ? member :
			{occurrence: member.occurrence, material: member.material, source: switch member.source {
				case Typed(id, values): Typed(id, [for (value in values) value.name != "model" ? value :
					{name: "model", value: SavedValue.Token("17HS19-1684S1")}]);
				case other: other;
			}}];
		var rebuilt = definition(MachineAssembly.fromDescription(description));
		var rebuiltSteps = actuatorsOf(rebuilt)[0].fullStepsPerRevolution;
		near(rebuiltSteps == null ? 0 : rebuiltSteps, 200, "a rebuilt motor keeps its steps");
		near(actuatorsOf(rebuilt)[0].maxEffort, 0.225, "a rebuilt motor's actuator follows the motor part");
		near(efficiencyOf(rebuilt), thread.efficiency(), "and the screw keeps its efficiency");
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

/** A servo motor with no geometry, to drive a joint through the MotorDrive hook. */
private class TestServo extends machinekit.component.MachineComponent implements machinekit.motion.MotorDrive {
	public function new() {
		super("TEST-SERVO", "test servo motor", null, true);
		addConnector("shaft", machinekit.component.ConnectorRole.Axis, machinekit.component.Solids.axial(0, 0, 0));
	}

	public function actuator(id:String, joint:String, volts:Float, margin:Float):materia.assembly.AssemblyDefinition.AssemblyActuator
		return {id: id, joint: joint, maxEffort: 1.9, maxRate: 500, rotorInertia: 2e-5, drive: "servo",
			ratedTorque: 0.64, peakTorque: 1.9, ratedSpeed: 314, maxSpeed: 500, encoderCounts: 4096,
			torqueSpeed: [0, 1.9, 314, 1.9, 500, 0.64]};
}

