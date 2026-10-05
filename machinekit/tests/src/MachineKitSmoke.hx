import CadKit;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.InertiaTensor;
import RobotArmPreview.RobotArmChecks;
import CncRouterPreview.CncRouterChecks;
import MobileBasePreview.MobileBaseChecks;
import RobotWelderPreview.RobotWelderChecks;
import CoreXyPlotterPreview.CoreXyPlotterChecks;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.LinearAxis;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.InstancePath;
import machinekit.component.PortInterfaces;
import machinekit.assembly.MachineAssembly.AssemblyBomMass;
import machinekit.assembly.FlangeBearingAssembly;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.component.Bom;
import machinekit.component.BomItem;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValue;
import machinekit.component.MachineComponent;
import machinekit.component.MassProperties.MassSource;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.PortInterface;
import machinekit.component.MachineKitComponents;
import machinekit.component.ComponentValues;
import machinekit.document.MachineKitDocuments;
import machinekit.document.MachineKitRecipes;
import machinekit.document.MachineKitDocumentAssembly;
import machinekit.component.Solids;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.Placement;
import cadkit.parametric.DefinitionEvaluatorRegistry;
import cadkit.parametric.DefinitionOutput;
import materia.project.SceneArtifact;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.motion.LeadScrewThread.LeadScrewHand;
import machinekit.motion.LinearBearing;
import machinekit.motion.LinearGuideSystem;
import machinekit.motion.LinearRailSystem;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.NemaStepper;
import machinekit.motion.CasterWheel;
import machinekit.motion.DriveWheel;
import machinekit.motion.FlangeBearingHousing;
import machinekit.motion.PillowBlock;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.SteppedShaft;
import pickingstation.PickingStationConfig;
import pickingstation.StorageRack;
import machinekit.standard.Bushing;
import machinekit.standard.BearingFit.BearingHousingFit;
import machinekit.standard.BearingFit.BearingShaftFit;
import machinekit.standard.ClearanceFit;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexBolt;
import machinekit.standard.HexNut;
import machinekit.standard.ParallelKey;
import machinekit.robotics.EndEffectorPlate;
import machinekit.robotics.ArmJoint;
import machinekit.robotics.ArmLink;
import machinekit.robotics.Pedestal;
import machinekit.robotics.RobotFlange;
import eoat.EndEffectorExampleChecks;
import machinekit.standard.RetainingRing;
import machinekit.standard.ShaftCollar;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.structural.Angle;
import machinekit.structural.Channel;
import machinekit.structural.FlatBar;
import machinekit.structural.FrameAssembly;
import machinekit.structural.FrameAssembly.FrameEndCut;
import machinekit.structural.RectTube;
import machinekit.structural.RoundTube;
import machinekit.structural.TSlotExtrusion;
import machinekit.transmission.GearPair;
import machinekit.transmission.Rack;
import machinekit.transmission.Sprocket;
import machinekit.transmission.SpurGear;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingBelt.BeltWrap;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.TimingBeltProfile;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

private class MassTestBlock extends MachineComponent {
	public function new(?declared:Float, withInertia:Bool = false) {
		super("TEST-BLOCK", "10 mm test block", "aluminium 6061", true);
		addConnector("origin", Mount, AssemblyFrames.identity());
		addConnector("right", Mount, {x: 20, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1});
		if (declared != null) declareMass(declared, new Vector(1, 2, 3),
			withInertia ? new InertiaTensor(2, 0, 0, 3, 0, 4) : null);
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(10, 10, 10);
}

private class MassTestTube extends MachineComponent {
	public function new() super("TEST-TUBE", "Rectangular test tube", "aluminium 6061", true);

	override public function geometry(detail:ComponentDetail = Preview):Part
		return new RectTube(40, 20, 2).geometry(100);
}

private class MasslessTestPart extends MachineComponent {
	public function new() super("TEST-MASSLESS", "Massless test part", "steel", true);
}

private class MissingMassCentre extends MachineComponent {
	public function new() {
		super("MISSING-CENTRE", "Invalid declared mass", "steel", true);
		var centre:Vector = null;
		declareMass(1, centre);
	}
}

private class PortTestComponent extends MachineComponent {
	public function new(name:String) super(name, name, "steel", true);
	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(1, 1, 1);

	public function definePort(name:String, kind:PortKind, role:PortRole, iface:PortInterface,
			required:Bool = false, ?connector:String):Void
		addPort({name: name, kind: kind, role: role, iface: iface, required: required, connector: connector});

	public function defineBridge(from:String, to:String):Void addBridge(from, to);
	public function defineConversion(from:String, to:String):Void addConversion(from, to);

	public function defineConnector(name:String):Void addConnector(name, Mount, AssemblyFrames.identity());
}

class MachineKitSmoke {
	public static function massProperties():Void {
		var tube = new MassTestTube();
		var expected = (40 * 20 - 36 * 16) * 100 * 1e-9 * 2700;
		near(tube.massProperties().mass, expected, "rectangular tube analytic mass", 1e-9);
		near(tube.massProperties().centreOfMass.z, 50, "tube centre of mass");
		check(new MassTestTube().massProperties().mass > 0,
			"Geometry-only override must provide computed mass");
		check(tube.massProperties() == tube.massProperties(), "component mass estimate is cached");
		check(switch tube.massProperties().source { case Computed(Preview): true; default: false; },
			"preview mass source");
		tube.setMaterial("steel");
		near(tube.massProperties().mass, expected * 7850 / 2700, "material change refreshes mass", 1e-9);
		var declared = new MassTestBlock(0.5).massProperties();
		near(declared.mass, 0.5, "declared mass overrides geometry");
		near(declared.centreOfMass.x, 1, "declared centre of mass");
		check(switch declared.source { case Declared: true; default: false; }, "declared mass source");
		check(declared.inertia == null, "declared mass does not silently invent inertia");
		throws(() -> new MissingMassCentre(), "declared centre of mass");
		throws(() -> new MasslessTestPart().massProperties(), "has no geometry or declared mass");

		var block = new MassTestBlock();
		var assembly = new MachineAssembly();
		assembly.addComponent("a", block);
		assembly.addComponent("b", block);
		throws(() -> assembly.encode(), "Code-only member");
		assembly.addMate("link", "fixed", "a", "right", "b", "origin");
		assembly.addBomItem({partNumber: "RAIL-CUT", description: "Unmodelled rail", quantity: 1, material: "steel"});
		var combined = assembly.massProperties();
		near(combined.mass, 0.0054, "two block mass", 1e-9);
		near(combined.centreOfMass.x, 10, "solved assembly centre of mass x");
		near(combined.centreOfMass.z, 5, "solved assembly centre of mass z");
		var childFirst = new MachineAssembly();
		for (id in ["a", "b", "c"]) childFirst.addComponent(id, block);
		childFirst.addMate("second", "fixed", "b", "right", "c", "origin");
		childFirst.addMate("first", "fixed", "a", "right", "b", "origin");
		var childFirstPoses = childFirst.solvedPoses();
		var middlePose = childFirstPoses.get("b"), leafPose = childFirstPoses.get("c");
		if (middlePose == null || leafPose == null) throw "Child-first MachineAssembly poses are missing";
		near(middlePose.x, 20, "child-first MachineAssembly middle pose");
		near(leafPose.x, 40, "child-first MachineAssembly leaf pose");
		var combinedInertia:InertiaTensor = cast combined.inertia;
		near(combinedInertia.xx, 0.09, "two block axial inertia", 1e-8);
		near(combinedInertia.yy, 0.63, "parallel axis inertia", 1e-8);
		near(combinedInertia.zz, 0.63, "parallel axis inertia about z", 1e-8);
		check(combined.unaccountedInertia.length == 0, "computed inertia is fully accounted");
		check(combined.unaccounted.length == 1 && combined.unaccounted[0] == "RAIL-CUT", "unaccounted BOM extras");
		assembly.addBomItem({partNumber: "TUBE-MASS", description: "Tube", quantity: 1,
			material: "polyurethane"}, 2, Point(0.001, new Vector(10, 0, 5)));
		var withTube = assembly.massProperties();
		near(withTube.mass, combined.mass + 0.002, "BOM extra mass uses quantity");
		check(withTube.unaccounted.length == 1 && withTube.unaccounted[0] == "RAIL-CUT",
			"mass-accounted BOM line is not unaccounted");

		var inner = new MachineAssembly();
		inner.addComponent("a", block);
		inner.addComponent("b", block);
		inner.addMate("link", "fixed", "a", "right", "b", "origin");
		var outer = new MachineAssembly();
		outer.include("unit", inner);
		near(outer.massProperties().mass, inner.massProperties().mass, "included assembly mass", 1e-9);
		near(outer.massProperties().centreOfMass.x, inner.massProperties().centreOfMass.x,
			"included assembly centre of mass");
		var outerInertia:InertiaTensor = cast outer.massProperties().inertia;
		var innerInertia:InertiaTensor = cast inner.massProperties().inertia;
		near(outerInertia.yy, innerInertia.yy,
			"included assembly inertia");
		inner.addComponent("late", block);
		check(outer.subassemblies()[0].assembly.components().length == 2,
			"include keeps a snapshot of the source builder");

		var moving = new MachineAssembly();
		moving.addComponent("a", block);
		moving.addComponent("b", block);
		moving.addMateOnAxis("slide", "prismatic", "a", "origin", "b", "origin",
			{x: 1, y: 0, z: 0}, 20);
		var model = new AssemblyModel();
		moving.addTo(model, "");
		var state = model.initialState();
		state.setJoint("slide", 30);
		near(moving.massProperties(state).centreOfMass.x, 15, "configured moving centre of mass");
		var movingInertia:InertiaTensor = cast moving.massProperties(state).inertia;
		near(movingInertia.yy, 1.305, "configured moving inertia", 1e-8);
		var tubeItem:BomItem = {partNumber: "MOVING-TUBE", description: "Tube", quantity: 1, material: "polyurethane"};
		moving.addBomItem(tubeItem, 1, Attached(0.001, "b", new Vector(0, 0, 0)));
		moving.addBomItem({partNumber: "FIXED-TUBE", description: "Fixed tube", quantity: 1,
			material: "polyurethane"}, 1, Point(0.001, new Vector(0, 0, 0)));
		var at30 = moving.massProperties(state);
		state.setJoint("slide", 40);
		var at40 = moving.massProperties(state);
		near(at40.centreOfMass.x - at30.centreOfMass.x,
			10 * (block.massProperties().mass + 0.001) / at40.mass,
			"moving member and attached tube shift centre of mass", 1e-9);
		throws(() -> moving.addBomItem(tubeItem, 1, Attached(0.001, "missing", new Vector())),
			"Unknown assembly member");
		var offsetInner = new MachineAssembly();
		offsetInner.addComponent("part", block);
		offsetInner.addBomItem(tubeItem, 1, Attached(0.001, "part", new Vector(2, 0, 0)));
		var offsetOuter = new MachineAssembly();
		offsetOuter.include("unit", offsetInner,
			{x: 50, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1});
		near(offsetOuter.massProperties().centreOfMass.x,
			50 + (0.001 * 2) /
			(block.massProperties().mass + 0.001), "included attached mass follows include pose");

		var missing = new MachineAssembly();
		missing.addComponent("declared", new MassTestBlock(0.5));
		var incomplete = missing.massProperties();
		check(incomplete.inertia == null && incomplete.unaccountedInertia[0] == "declared",
			"declared mass reports missing inertia");
		var rotated = new MachineAssembly();
		rotated.addComponent("declared", new MassTestBlock(1, true),
			{x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: Math.sqrt(0.5), qw: Math.sqrt(0.5)});
		var rotatedInertia:InertiaTensor = cast rotated.massProperties().inertia;
		near(rotatedInertia.xx, 3, "declared tensor rotates into assembly frame");
		near(rotatedInertia.yy, 2, "declared tensor y moment rotates into assembly frame");
	}

	public static function ports():Void {
		var path = InstancePath.of("robot/tool/cup");
		check(path.segments().join(",") == "robot,tool,cup" && path.parent() == "robot/tool",
			"instance path exposes segments and parent");
		throws(() -> new InstancePath("bad/name"), "cannot contain");
		var pathAssembly = new MachineAssembly();
		throws(() -> pathAssembly.addComponent("bad/name", new MassTestBlock()), "cannot contain");
		check(PortInterfaces.compatible(PushIn(6), PushIn(6)) &&
			!PortInterfaces.compatible(PushIn(6), PushIn(8)) &&
			PortInterfaces.compatible(Plug("M12", 4), Plug("M12", 4)) &&
			!PortInterfaces.compatible(Plug("M12", 4), Plug("M12", 5)) &&
			PortInterfaces.compatible(Thread("G1/8-M"), Thread("G1/8-F")),
			"port interfaces compare by value with thread mating rules");

		var changer = new PortTestComponent("CHANGER");
		changer.defineConnector("airFace");
		changer.definePort("robotAir", Pneumatic, Consumer, PushIn(6), false, "airFace");
		changer.definePort("toolAir", Pneumatic, Passive, PushIn(6));
		changer.defineBridge("robotAir", "toolAir");
		check(changer.ports().length == 2 && changer.bridges().length == 1,
			"component ports and bridge are available");
		throws(() -> changer.definePort("robotAir", Pneumatic, Supply, Unspecified), "Duplicate port");
		throws(() -> changer.definePort("badTube", Pneumatic, Supply, PushIn(0)), "Invalid port interface");
		throws(() -> changer.definePort("bad", Pneumatic, Supply, Unspecified, false, "missing"), "Missing connector");
		throws(() -> changer.defineBridge("robotAir", "missing"), "Missing port");
		var wrongBridge = new PortTestComponent("BRIDGE");
		wrongBridge.definePort("air", Pneumatic, Consumer, Unspecified);
		wrongBridge.definePort("vacuum", Vacuum, Supply, Unspecified);
		throws(() -> wrongBridge.defineBridge("air", "vacuum"), "same kind");
		var supplyInlet = new PortTestComponent("SUPPLY-INLET");
		supplyInlet.definePort("in", Pneumatic, Supply, Unspecified);
		supplyInlet.definePort("out", Pneumatic, Passive, Unspecified);
		throws(() -> supplyInlet.defineBridge("in", "out"), "Supply inlet");

		var manifold = new PortTestComponent("MANIFOLD");
		manifold.definePort("in", Pneumatic, Passive, PushIn(6));
		manifold.definePort("out", Pneumatic, Passive, PushIn(6));
		manifold.defineBridge("in", "out");
		var generator = new PortTestComponent("GENERATOR");
		generator.definePort("air", Pneumatic, Consumer, PushIn(6), true);
		generator.definePort("vacuum", Vacuum, Supply, PushIn(6));
		generator.defineConversion("air", "vacuum");
		var cup = new PortTestComponent("CUP");
		cup.definePort("vacuum", Vacuum, Consumer, PushIn(6), true);

		var tool = new MachineAssembly();
		tool.addComponent("changer", changer);
		tool.addComponent("manifold", manifold);
		tool.addComponent("generator", generator);
		tool.addComponent("cup", cup);
		throws(() -> tool.connectPorts("missing", "none", "air", "cup", "vacuum"), "Unknown assembly member");
		throws(() -> tool.connectPorts("missing", "changer", "none", "cup", "vacuum"), "Unknown port");
		throws(() -> tool.exposePort("bad", "cup", "none"), "Unknown port");
		tool.connectPorts("supply", "changer", "toolAir", "manifold", "in",
			{partNumber: "TUBE-6", description: "6 mm tube", quantity: 1, material: "polyurethane"},
			Attached(0.001, "changer", new Vector(0, 0, 0)));
		tool.connectPorts("air", "manifold", "out", "generator", "air");
		tool.connectPorts("vacuum", "generator", "vacuum", "cup", "vacuum");
		tool.exposePort("robotAir", "changer", "robotAir");
		check(tool.validate().length == 0, "compatible service path validates");
		check(tool.billOfMaterials().quantity("TUBE-6") == 1, "connection line reaches BOM");
		var source = tool.upstream("cup", "vacuum");
		check(source.port.instanceId == "changer" && source.port.portName == "robotAir" && source.external,
			"cup vacuum traces through generator, manifold, and changer bridge");
		var model = new AssemblyModel();
		tool.addTo(model, "tool");
		check(model.definition().joints.length == 0, "port connections are not CAD mates");
		var station = new MachineAssembly();
		station.include("tool", tool);
		check(station.portNames().length == 0, "included ports are not automatically public");
		throws(() -> station.port("tool/robotAir"), "Missing assembly port");
		var top = new MachineAssembly();
		top.include("station", station);
		check(top.portNames().length == 0, "unexposed ports stay private across three levels");
		station.exposePort("airSupply", "tool/changer", "robotAir");
		check(station.port("airSupply").instanceId == "tool/changer", "port can be re-exposed under a new name");
		check(top.portNames().length == 0, "a containing assembly does not inherit exposed ports");
		var includedSource = station.upstream("tool/cup", "vacuum");
		check(includedSource.port.instanceId == "tool/changer" && includedSource.port.portName == "robotAir" && includedSource.external,
			"included service path retains its source");
		check(station.billOfMaterials().quantity("TUBE-6") == 1, "included line is counted once");
		var vacuumSource = new PortTestComponent("VACUUM-SOURCE");
		vacuumSource.definePort("vacuum", Vacuum, Supply, PushIn(6));
		var reversed = new MachineAssembly();
		reversed.addComponent("generator", vacuumSource);
		reversed.addComponent("cup", cup);
		reversed.connectPorts("reverse", "cup", "vacuum", "generator", "vacuum");
		check(reversed.upstream("cup", "vacuum").port.instanceId == "generator",
			"consumer-first connection traces to its supply");
		var unfinished = new MachineAssembly();
		unfinished.addComponent("source", vacuumSource);
		unfinished.addComponent("cup", cup);
		unfinished.addComponent("gripper", generator);
		unfinished.connectPorts("vacuum", "source", "vacuum", "cup", "vacuum");
		check(unfinished.upstream("cup", "vacuum").port.instanceId == "source",
			"upstream works while another required input is unconnected");
		throws(() -> unfinished.validate(), "Required consumer port");
		var incompleteTool = new MachineAssembly();
		incompleteTool.addComponent("cup", cup);
		incompleteTool.exposePort("vacuum", "cup", "vacuum");
		var parent = new MachineAssembly();
		parent.include("tool", incompleteTool);
		check(parent.portNames().length == 0, "included required input is private until re-exposed");
		throws(() -> parent.validate(), "Required consumer port");
		parent.exposePort("tool/vacuum", "tool/cup", "vacuum");
		check(parent.validate().length == 0, "parent must explicitly re-expose an included required input");
		var connectedParent = new MachineAssembly();
		connectedParent.include("tool", incompleteTool);
		connectedParent.addComponent("source", vacuumSource);
		connectedParent.connectPorts("feed", "tool/cup", "vacuum", "source", "vacuum");
		check(connectedParent.validate().length == 0,
			"parent can connect an included required input directly");
		var unwired = new MachineAssembly();
		unwired.addComponent("cup", cup);
		unwired.addTo(new AssemblyModel(), "");
		unwired.massProperties();
		throws(() -> unwired.validate(), "Required consumer port");
		var unrelated = new PortTestComponent("UNRELATED");
		unrelated.definePort("power", ElectricalPower, Consumer, Unspecified, true);
		unrelated.definePort("air", Pneumatic, Supply, Unspecified);
		var power = new PortTestComponent("POWER");
		power.definePort("output", ElectricalPower, Supply, Unspecified);
		var misleading = new MachineAssembly();
		misleading.addComponent("device", unrelated);
		misleading.addComponent("power", power);
		misleading.connectPorts("feed", "power", "output", "device", "power");
		check(misleading.upstream("device", "air").port.instanceId == "device",
			"upstream does not infer service conversion from unrelated power");

		var diagnosticsAssembly = new MachineAssembly();
		diagnosticsAssembly.addComponent("cupA", cup);
		diagnosticsAssembly.addComponent("cupB", cup);
		var diagnosticsSource = new PortTestComponent("DIAGNOSTICS-SOURCE");
		diagnosticsSource.definePort("air", Pneumatic, Supply, PushIn(6));
		diagnosticsAssembly.addComponent("source", diagnosticsSource);
		diagnosticsAssembly.connectPorts("bad-kind", "source", "air", "cupA", "vacuum");
		var findings = diagnosticsAssembly.check().items;
		check(findings.length == 2 && findings[0].code == "port.kind-mismatch" &&
			findings[0].subject == "bad-kind" &&
			findings[1].code == "port.required-unconnected" && findings[1].subject == "cupB/vacuum",
			"diagnostics collect independent connection and required-port faults");
		throws(() -> diagnosticsAssembly.validate(), "mismatched kinds");
		var structure = new MachineAssembly();
		structure.addComponent("root", new MassTestBlock());
		structure.addComponent("child", new MassTestBlock());
		structure.addMate("first", "fixed", "root", "right", "child", "origin");
		structure.addMate("second", "fixed", "root", "origin", "child", "right");
		structure.addCoupling("missing-joints", "absent-a", "absent-b", 1);
		var structuralFindings = structure.check().items;
		check(structuralFindings.length == 2 && structuralFindings[0].code == "assembly.multiple-parents" &&
			structuralFindings[0].subject == "child" &&
			structuralFindings[1].code == "assembly.missing-coupling-joint" &&
			structuralFindings[1].subject == "missing-joints",
			"diagnostics collect independent structural faults");
		throws(() -> structure.validateStructure(), "two parent joints");


		var unconnected = new MachineAssembly();
		unconnected.addComponent("cup", cup);
		throws(() -> unconnected.validate(), "Required consumer port");
		unconnected.exposePort("vacuum", "cup", "vacuum");
		check(unconnected.validate().length == 0, "exposed consumer can be supplied by parent assembly");

		var badKind = new MachineAssembly();
		badKind.addComponent("changer", changer);
		badKind.addComponent("cup", cup);
		badKind.connectPorts("wrong", "changer", "toolAir", "cup", "vacuum");
		throws(() -> badKind.validate(), "mismatched kinds");
		badKind.addComponent("source", vacuumSource);
		badKind.addComponent("otherCup", cup);
		badKind.connectPorts("valid", "source", "vacuum", "otherCup", "vacuum");
		throws(() -> badKind.upstream("otherCup", "vacuum"), "mismatched kinds");
		var supplier = new PortTestComponent("SUPPLIER");
		supplier.definePort("air", Pneumatic, Supply, PushIn(6));
		var doubleSupply = new MachineAssembly();
		doubleSupply.addComponent("a", supplier);
		doubleSupply.addComponent("b", supplier);
		doubleSupply.connectPorts("wrong", "a", "air", "b", "air");
		throws(() -> doubleSupply.validate(), "incompatible roles");
		var consumer = new PortTestComponent("CONSUMER");
		consumer.definePort("air", Pneumatic, Consumer, PushIn(6));
		var doubleConsumer = new MachineAssembly();
		doubleConsumer.addComponent("a", consumer);
		doubleConsumer.addComponent("b", consumer);
		doubleConsumer.connectPorts("wrong", "a", "air", "b", "air");
		throws(() -> doubleConsumer.validate(), "incompatible roles");
		var threaded = new PortTestComponent("THREADED");
		threaded.definePort("air", Pneumatic, Consumer, Thread("G1/8"));
		var mismatch = new MachineAssembly();
		mismatch.addComponent("a", supplier);
		mismatch.addComponent("b", threaded);
		mismatch.connectPorts("adapter-needed", "a", "air", "b", "air");
		mismatch.validateStructure();
		throws(() -> mismatch.validate(), "mismatched interfaces");
		var branch = new MachineAssembly();
		branch.addComponent("a", supplier);
		branch.addComponent("b", consumer);
		branch.addComponent("c", consumer);
		branch.connectPorts("first", "a", "air", "b", "air");
		branch.connectPorts("second", "a", "air", "c", "air");
		throws(() -> branch.validate(), "more than once");
	}

	static function componentRecipes():Void {
		for (recipe in MachineKitComponents.defaultRegistry().all()) {
			var original = recipe.create();
			check(original.type == recipe, 'recipe type mismatch ${recipe.id}');
			var values = original.values();
			var restored = recipe.create(values);
			check(recipe.key(values) == recipe.key(restored.values()), 'recipe values mismatch ${recipe.id}');
			check(original.designation == restored.designation, 'recipe designation mismatch ${recipe.id}');
			var a = original.connectors(), b = restored.connectors();
			check(a.length == b.length, 'recipe connector count mismatch ${recipe.id}');
			for (i in 0...a.length) {
				check(a[i].name == b[i].name && a[i].role == b[i].role,
					'recipe connector mismatch ${recipe.id}');
				check(a[i].frame.x == b[i].frame.x && a[i].frame.y == b[i].frame.y &&
					a[i].frame.z == b[i].frame.z && a[i].frame.qx == b[i].frame.qx &&
					a[i].frame.qy == b[i].frame.qy && a[i].frame.qz == b[i].frame.qz &&
					a[i].frame.qw == b[i].frame.qw, 'recipe connector frame mismatch ${recipe.id}');
			}
			var first = original.geometry(), second = restored.geometry();
			near(first.massProperties().volume, second.massProperties().volume,
				'recipe volume mismatch ${recipe.id}');
			check(first.solidCount() == second.solidCount(), 'recipe solids mismatch ${recipe.id}');
			first.close();
			second.close();
		}
		var bearing = MachineKitComponents.defaultRegistry().byId("machinekit.standard.deep-groove-bearing");
		throws(() -> bearing.create(new ComponentValues().setToken("designation", "NO-BEARING")),
			"Unknown catalog designation");
		var pulley = MachineKitComponents.defaultRegistry().byId("machinekit.transmission.timing-pulley");
		throws(() -> pulley.create(new ComponentValues().setToken("profile", "UNKNOWN")), "Invalid choice");
		var angleType = new ComponentType("test.angle",
			[new ComponentParameter("angle", ComponentParameterType.Angle, ComponentValue.Number(3))],
			function(_:ComponentValues) return DeepGrooveBearing.metric("608"));
		check(angleType.key(null).indexOf("angle:1:3") >= 0,
			"angle keys round above the signed 32-bit integer range");
		var scalarType = new ComponentType("test.scalar",
			[new ComponentParameter("scalar", ComponentParameterType.Scalar, ComponentValue.Number(3))],
			function(_:ComponentValues) return DeepGrooveBearing.metric("608"));
		check(scalarType.key(null).indexOf("scalar:1:3") >= 0,
			"scalar keys round above the signed 32-bit integer range");
		var screwRecipe = MachineKitComponents.defaultRegistry().byId("machinekit.standard.socket-head-cap-screw");
		var steelScrew = screwRecipe.create(new ComponentValues().setToken("material", "steel C45"));
		check(steelScrew.bom.material == "steel C45", "non-default screw material reaches the BOM");
		check(steelScrew.designation == "ISO4762-M5x20-steel-C45" &&
			steelScrew.bom.partNumber != screwRecipe.create().bom.partNumber,
			"non-default screw material has its own designation and BOM identity");
		var customBearing = DeepGrooveBearing.custom({designation: "608", bore: 8, outside: 22, width: 7, chamfer: 0.3}, false);
		check(customBearing.componentType() == null && StringTools.startsWith(customBearing.designation, "CUSTOM-608-D8x22x7-C0.3") &&
			customBearing.bom.partNumber == customBearing.designation && customBearing.bom.typeId == null,
			"custom catalog dimensions produce a code-only, prefixed component");
		var customBearingVariant = DeepGrooveBearing.custom({designation: "608", bore: 8.2, outside: 22, width: 7, chamfer: 0.3}, false);
		check(customBearingVariant.bom.partNumber != customBearing.bom.partNumber,
			"custom bearing dimensions retain distinct BOM identities");
		var customMotor = NemaStepper.custom(NemaStepper.catalog().get("17"), {
			designation: "17HS19-custom", frame: 17, bodyFace: 42, bodyLength: 52,
			shaftDiameter: 5, shaftLength: 25, pilotHeight: 2, mountScrew: "M3",
			tappedMount: true, mountHoleDepth: 4.5
		});
		check(customMotor.componentType() == null && StringTools.startsWith(customMotor.designation, "CUSTOM-17HS19-custom-IF"),
			"custom NEMA variants are code-only and prefixed");
		customCodeOnly(HexBolt.custom(HexBolt.catalog().get("M5"), 20), "hex bolt");
		customCodeOnly(HexNut.custom(HexNut.catalog().get("M5")), "hex nut");
		customCodeOnly(FlatWasher.custom(FlatWasher.catalog().get("M5")), "flat washer");
		customCodeOnly(ParallelKey.custom(ParallelKey.catalog().get("2x2"), 10), "parallel key");
		customCodeOnly(RetainingRing.custom(RetainingRing.catalog().get("8")), "retaining ring");
		customCodeOnly(ShaftCollar.custom(ShaftCollar.catalog().get("8")), "shaft collar");
		customCodeOnly(SocketHeadCapScrew.custom(SocketHeadCapScrew.catalog().get("M5"), 20), "cap screw");
		customCodeOnly(LinearBearing.custom(LinearBearing.catalog().get("LM8UU")), "linear bearing");
		customCodeOnly(PillowBlock.custom(PillowBlock.catalog().get("UCP204")), "pillow block");
		customCodeOnly(LinearRail.custom(LinearRailSystem.catalog().get("MGN12C"), 100), "profile rail");
		customCodeOnly(LinearRailBlock.custom(LinearRailSystem.catalog().get("MGN12C")), "rail block");
		var firstLength = screwRecipe.defaults().setNumber("length", 20.0001);
		var secondLength = screwRecipe.defaults().setNumber("length", 20.0002);
		check(screwRecipe.key(firstLength) == screwRecipe.key(secondLength), "length keys round to a micrometre");
		var conflicted = new Bom();
		conflicted.add({partNumber: "X", description: "same", quantity: 1, material: "steel",
			typeId: "test", valuesKey: "a"});
		throws(() -> conflicted.add({partNumber: "X", description: "same", quantity: 1,
			material: "steel", typeId: "test", valuesKey: "b"}), "conflicting");
	}

	static function customCodeOnly(component:MachineComponent, label:String):Void {
		check(component.componentType() == null && StringTools.startsWith(component.designation, "CUSTOM-"),
			'$label custom spec is code-only and prefixed');
		check(component.bom.partNumber == component.designation && component.bom.typeId == null,
			'$label custom spec has a code-only BOM line');
	}

	static function documentRecipes():Void {
		MachineKitRecipes.register();
		MachineKitRecipes.register();
		var registryDocument = new Document();
		for (registered in MachineKitComponents.defaultRegistry().all()) {
			check(DefinitionEvaluatorRegistry.isRegistered(registered.id), "MachineKit evaluator registration");
			var recipeDefinition = MachineKitDocuments.define(registryDocument, registered);
			var defaultComponent = registered.create();
			var toolInputCount = 0;
			for (tool in defaultComponent.toolSpecs())
				toolInputCount += tool.parameters().length;
			check(recipeDefinition.output("body").purpose == DefinitionOutput.Geometry,
				"MachineKit recipe geometry output");
			check(recipeDefinition.outputs().length == defaultComponent.toolSpecs().length + 1,
				"MachineKit stores geometry and type-level tool outputs");
			check(recipeDefinition.inputs().length == registered.parameters().length + toolInputCount + 1,
				"MachineKit defines typed tool inputs with defaults");
		}
		registryDocument.close();
		var routeDocument = new Document();
		var routeType = MachineKitComponents.defaultRegistry().byId("machinekit.pneumatic.routed-hose");
		var routeDefinition = MachineKitDocuments.define(routeDocument, routeType,
			routeType.defaults().setToken("route", '[{"x":0,"y":0,"z":0},{"x":0,"y":0,"z":80},{"x":30,"y":0,"z":80}]'));
		var routeInstance = routeDocument.createInstance("Hose", routeDefinition);
		check(MachineKitRecipes.component(routeInstance).connector("end").frame.x == 30,
			"text route input builds the saved connector frame");
		var reloadedRoute = DocumentCodec.decode(DocumentCodec.encode(routeDocument));
		var reloadedInstance:cadkit.parametric.InstanceElement = cast reloadedRoute.element(routeInstance.id);
		check(MachineKitRecipes.component(reloadedInstance).connector("end").frame.x == 30,
			"text route input survives document save and reload");
		reloadedRoute.close();
		routeDocument.close();
		var document = new Document();
		var type = MachineKitComponents.defaultRegistry().byId("machinekit.standard.deep-groove-bearing");
		var definition = MachineKitDocuments.define(document, type);
		check(definition.property("machinekit.type") != null && definition.properties().length == 1,
			"only recipe identity is stored");
		var first = document.createInstance("Bearing A", definition);
		var second = document.createInstance("Bearing B", definition);
		check(MachineKitDocuments.partNumber(first) == "608-2Z" && MachineKitDocuments.catalogSource(first) != null,
			"recipe metadata is computed from the instance");
		var firstVolume = first.shape().volume();
		var secondVolume = second.shape().volume();
		check(firstVolume == secondVolume, "shared bearing geometry");
		first.setTypedOverride("detail", "envelope");
		check(first.shape().volume() != firstVolume && second.shape().volume() == secondVolume,
			"detail input selects geometry fidelity per instance");
		first.removeOverride("detail");
		var bearingSeat = document.definitionOutput(first, "bearingSeat").volume();
		check(bearingSeat > 0, "bearing seat tool output");
		check(definition.input("tool_bearingSeat_fit").defaultValue == "Slip",
			"bearing seat fit has a typed definition default");
		check(definition.input("tool_bearingSeat_depth").defaultValue == 7,
			"bearing seat depth has a typed definition default");
		first.setTypedOverride("tool_bearingSeat_fit", "Interference");
		check(document.definitionOutput(first, "bearingSeat").volume() != bearingSeat,
			"bearing seat fit override changes tool geometry");
		first.removeOverride("tool_bearingSeat_fit");
		first.setTypedOverride("tool_bearingSeat_depth", 3.0);
		check(document.definitionOutput(first, "bearingSeat").volume() != bearingSeat,
			"bearing seat depth override changes tool geometry");
		first.removeOverride("tool_bearingSeat_depth");
		var firstBack = first.connector("back").location.plane.origin.z;
		document.setElementPlacement(first, new Placement(new Plane(
			new Vector(10, 20, 30), Vector.X(), Vector.Z())));
		var assembly = new AssemblyModel();
		MachineKitDocumentAssembly.add(assembly, "bearingA", first);
		MachineKitDocumentAssembly.add(assembly, "bearingB", second);
		assembly.mate("bearing-seat", "fixed", "bearingA", "back", "bearingB", "front");
		near(assembly.worldPoint("bearingB", "front").x, 10, "document assembly placement x");
		near(assembly.worldPoint("bearingB", "front").y, 20, "document assembly placement y");
		near(assembly.worldPoint("bearingB", "front").z, firstBack + 30, "document connectors mate by name");
		second.setTypedOverride("designation", "6000");
		check(MachineKitDocuments.partNumber(second) == "6000-2Z", "part number follows the instance override");
		check(second.shape().volume() != firstVolume, "bearing override updates one instance");
		near(first.shape().volume(), firstVolume, "other bearing retains its volume");
		check(second.connector("back").location.plane.origin.z != firstBack &&
			first.connector("back").location.plane.origin.z == firstBack + 30,
			"bearing override updates one connector set");
		var bom = MachineKitDocuments.bom(document);
		check(bom.quantity("608-2Z") == 1 && bom.quantity("6000-2Z") == 1,
			"document BOM groups recipe instances by values");
		var saved = DocumentCodec.encode(document);
		check(DocumentCodec.VERSION == 11, "document version 11");
		var loaded = DocumentCodec.decode(saved);
		check(MachineKitDocuments.bom(loaded).lines().length == 2, "recipe BOM survives save and reload");
		loaded.close();
		check(document.undo() && second.resolvedToken("designation") == "608", "recipe override undo");
		check(MachineKitDocuments.bom(document).quantity("608-2Z") == 2, "BOM follows undo");
		check(document.redo() && second.resolvedToken("designation") == "6000", "recipe override redo");
		var unique = second.makeUnique();
		check(unique.id.value != definition.id.value && first.definitionId.value == definition.id.value,
			"makeUnique isolates one recipe instance");
		var uniqueSaved = DocumentCodec.decode(DocumentCodec.encode(document));
		check(MachineKitDocuments.bom(uniqueSaved).lines().length == 2, "unique recipe survives reload");
		uniqueSaved.close();
		document.close();

		var tools = new Document();
		var screwType = MachineKitComponents.defaultRegistry().byId("machinekit.standard.socket-head-cap-screw");
		var screw = tools.createInstance("Screw", MachineKitDocuments.define(tools, screwType));
		for (name in ["clearanceHole", "tapHole", "counterboreHole"])
			check(tools.definitionOutput(screw, name).volume() > 0, "screw tool output " + name);
		var mediumClearance = tools.definitionOutput(screw, "clearanceHole").volume();
		screw.setTypedOverride("tool_clearanceHole_fit", "Coarse");
		check(tools.definitionOutput(screw, "clearanceHole").volume() != mediumClearance,
			"screw clearance fit override changes tool geometry");
		var screwComponent = SocketHeadCapScrew.metric("M5", 2);
		var counterbore:Null<machinekit.component.ToolSpec> = null;
		for (tool in screwComponent.toolSpecs()) if (tool.name == "counterboreHole") counterbore = tool;
		check(counterbore != null && counterbore.defaults().number("depth") > screwComponent.spec.counterboreDepth,
			"counterbore default depth covers the screw head");
		var nutType = MachineKitComponents.defaultRegistry().byId("machinekit.standard.hex-nut");
		var nut = tools.createInstance("Nut", MachineKitDocuments.define(tools, nutType));
		check(tools.definitionOutput(nut, "pocket").volume() > 0,
			"HexNut exposes a valid pocket tool output");
		var slipHousing = new FlangeBearingHousing(DeepGrooveBearing.metric("608"), BearingHousingFit.Slip);
		var interferenceHousing = new FlangeBearingHousing(DeepGrooveBearing.metric("608"), BearingHousingFit.Interference);
		check(slipHousing.tool("bearingSeat", new ComponentValues()).volume() !=
			interferenceHousing.tool("bearingSeat", new ComponentValues()).volume(),
			"flange bearing housing tool uses its selected fit");
		var motorType = MachineKitComponents.defaultRegistry().byId("machinekit.motion.nema-stepper");
		var motor = tools.createInstance("Motor", MachineKitDocuments.define(tools, motorType));
		check(tools.definitionOutput(motor, "mountingCutout").volume() > 0, "motor cutout output");
		var flangeType = MachineKitComponents.defaultRegistry().byId("machinekit.robotics.robot-flange");
		var flange = tools.createInstance("Flange", MachineKitDocuments.define(tools, flangeType));
		check(flange.connectorNames().indexOf("bolt4") >= 0 && flange.connectorNames().indexOf("bolt5") < 0,
			"flange starts with four bolt connectors");
		flange.setTypedOverride("boltCount", 6);
		check(flange.connectorNames().indexOf("bolt5") >= 0 && flange.connectorNames().indexOf("bolt6") >= 0,
			"flange grows its connector set");
		flange.setTypedOverride("boltCount", 3);
		check(flange.connectorNames().indexOf("bolt3") >= 0 && flange.connectorNames().indexOf("bolt4") < 0,
			"flange shrinks its connector set");
		var railType = MachineKitComponents.defaultRegistry().byId("machinekit.motion.linear-rail");
		var rail = tools.createInstance("Rail", MachineKitDocuments.define(tools, railType));
		var shortMounts = rail.connectorNames().length;
		rail.setOverride("length", 200);
		check(rail.connectorNames().length > shortMounts, "rail length changes mount connectors");
		var firstPartNumber = MachineKitDocuments.partNumber(flange);
		var flangeDefinition = tools.definition(flange.definitionId);
		flangeDefinition.setDefault("pitchCircleDiameter", 50);
		check(MachineKitDocuments.partNumber(flange) != firstPartNumber,
			"definition default changes computed part number");
		tools.close();
	}

	static function documentPreview():Void {
		var editable = MotorShaftBearingsPreview.document();
		var saved = DocumentCodec.encode(editable);
		var baseline = SceneArtifact.decode(MotorShaftBearingsPreview.preview(saved));
		check(baseline.parts.length == 8, "document preview retains shared part geometry");
		var changed:Null<cadkit.parametric.InstanceElement> = null;
		for (element in editable.allElements()) if (element.name == "bearingB") changed = cast element;
		check(changed != null, "document preview exposes bearing instance");
		changed.setTypedOverride("designation", "6000");
		var edited = SceneArtifact.decode(MotorShaftBearingsPreview.preview(DocumentCodec.encode(editable)));
		check(edited.parts.length == 9, "saved bearing dimensions rebuild one preview definition");
		editable.close();
	}
	static function check(value:Bool, message:String):Void {
		if (!value) throw message;
	}

	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void
		check(Math.abs(actual - expected) <= tolerance * Math.max(1, Math.abs(expected)),
			'$message: $actual != $expected');

	static function throws(action:() -> Void, fragment:String):Void {
		try {
			action();
		} catch (error:Dynamic) {
			var message = Std.string(error);
			check(message.indexOf(fragment) >= 0, 'unexpected error "$message"');
			return;
		}
		throw 'expected an error containing "$fragment"';
	}

	static function solid(part:Part, message:String):Void {
		check(part.valid(), '$message is invalid');
		check(part.solidCount() == 1, '$message has ${part.solidCount()} solids');
	}

	static function bounds(part:Part):{minX:Float, minZ:Float, maxX:Float, maxZ:Float} {
		var box = part.shape.bounds();
		return {minX: box.get_min().get_x(), minZ: box.get_min().get_z(),
			maxX: box.get_max().get_x(), maxZ: box.get_max().get_z()};
	}

	static function annulus(outside:Float, inside:Float, length:Float):Float
		return Math.PI * (outside * outside - inside * inside) / 4 * length;

	static function bearings():Void {
		var bearing = DeepGrooveBearing.metric("6204");
		check(bearing.designation == "6204-2Z", "shielded designation");
		check(bearing.bore == 20 && bearing.outside == 47 && bearing.width == 14, "6204 dimensions");
		for (designation in DeepGrooveBearing.catalog().designations())
			DeepGrooveBearing.metric(designation, false);
		throws(() -> DeepGrooveBearing.metric("6299"), 'Unknown deep groove bearing "6299"');
		throws(() -> DeepGrooveBearing.custom({designation: "invalid", bore: 10, outside: 20, width: 6, chamfer: 2}, false),
			"Invalid deep groove bearing");

		var envelope = bearing.geometry(Envelope);
		solid(envelope, "bearing envelope");
		near(envelope.volume(), annulus(47, 20, 14), "bearing envelope volume");
		envelope.close();
		var preview = bearing.geometry();
		solid(preview, "bearing preview");
		var box = bounds(preview);
		near(box.maxX, 23.5, "bearing outside radius");
		near(box.minZ, 0, "bearing front face");
		near(box.maxZ, 14, "bearing back face");
		check(preview.volume() < annulus(47, 20, 14), "preview removes shield recesses");
		preview.close();

		var seat = bearing.housingSeat(10, BearingHousingFit.Interference);
		near(seat.volume(), Math.PI * Math.pow((47 - 0.03) / 2, 2) * 10, "housing seat fit volume");
		near(bearing.journalDiameter(BearingShaftFit.Interference), 20.02, "journal interference fit");
		near(bearing.journalDiameter(BearingShaftFit.Slip), 19.98, "journal slip fit");
		near(bearing.journalDiameterAllowance(0.01), 20.01, "explicit journal allowance");
		var slipSeat = bearing.housingSeat(10, BearingHousingFit.Slip);
		check(slipSeat.volume() > seat.volume(), "housing slip fit is larger than press fit");
		var explicitSeat = bearing.housingSeatAllowance(10, -0.02);
		near(explicitSeat.volume(), Math.PI * Math.pow(46.98 / 2, 2) * 10, "explicit housing allowance");
		seat.close();
		slipSeat.close();
		explicitSeat.close();
		near(bearing.connector("axis").frame.z, 7, "bearing axis connector");
		// Connector +Y is the bearing axis (+Z).
		var axis = AssemblyFrames.transformVector(bearing.connector("back").frame, 0, 1, 0);
		near(axis.z, 1, "bearing connector axis");
	}

	static function screws():Void {
		var screw = SocketHeadCapScrew.metric("M6", 25);
		check(screw.designation == "ISO4762-M6x25", "screw part number");
		near(screw.pitch, 1, "M6 pitch");
		near(screw.threadLength, 24, "M6 thread length");
		near(SocketHeadCapScrew.metric("M6", 12.5).threadLength, 12.5, "short screws are fully threaded");
		check(SocketHeadCapScrew.metric("M6", 12.5).designation == "ISO4762-M6x12.5", "fractional length");
		throws(() -> SocketHeadCapScrew.metric("M7", 20), 'Unknown metric screw size "M7"');
		throws(() -> SocketHeadCapScrew.metric("M6", 0), "positive length");

		var envelope = screw.geometry(Envelope);
		solid(envelope, "screw envelope");
		near(envelope.volume(), Math.PI * (25 * 6 + 9 * 25), "screw envelope volume");
		var preview = screw.geometry(Preview);
		solid(preview, "screw preview");
		var socketArea = 3 * Math.sqrt(3) / 2 * Math.pow(5 / Math.sqrt(3), 2);
		near(envelope.volume() - preview.volume(), socketArea * 3, "hex socket volume");
		var box = bounds(preview);
		near(box.minZ, -25, "screw tip");
		near(box.maxZ, 6, "screw head top");
		envelope.close();
		preview.close();

		near(screw.clearanceDiameter(Fine), 6.4, "fine clearance");
		near(screw.clearanceDiameter(), 6.6, "medium clearance");
		near(screw.clearanceDiameter(Coarse), 7, "coarse clearance");
		var clearance = screw.clearanceHole(10);
		near(clearance.volume(), Math.PI * 3.3 * 3.3 * 10, "clearance hole volume");
		near(bounds(clearance).maxZ, 0, "clearance hole seat");
		clearance.close();
		var tap = screw.tapHole(12);
		near(tap.volume(), Math.PI * 2.5 * 2.5 * 12, "tap hole volume");
		tap.close();
		var counterbore = screw.counterboreHole(15);
		solid(counterbore, "counterbore tool");
		near(counterbore.volume(), Math.PI * (3.3 * 3.3 * (15 - 6.4) + 5.5 * 5.5 * 6.4), "counterbore volume");
		counterbore.close();
		throws(() -> screw.counterboreHole(5), "needs depth");
		for (size in ["M14", "M16", "M20"]) {
			var large = SocketHeadCapScrew.metric(size, 40);
			var largePart = large.geometry(Preview);
			solid(largePart, size + " screw preview");
			largePart.close();
		}
		for (reference in [
			{name: "M14", coarse: 16.5, bore: 24.0, depth: 14.6},
			{name: "M16", coarse: 18.5, bore: 26.0, depth: 16.6},
			{name: "M20", coarse: 24.0, bore: 33.0, depth: 20.6}]) {
			var standard = SocketHeadCapScrew.metric(reference.name, 40);
			near(standard.clearanceDiameter(Coarse), reference.coarse, reference.name + " ISO 273 coarse clearance");
			near(standard.spec.counterboreDiameter, reference.bore, reference.name + " DIN 974-1 counterbore diameter");
			near(standard.spec.counterboreDepth, reference.depth, reference.name + " DIN 974-1 counterbore depth");
		}
	}

	static function motors():Void {
		for (frame in [17, 23, 34]) {
			var motor = NemaStepper.frame(frame);
			var preview = motor.geometry();
			solid(preview, 'NEMA $frame preview');
			var box = bounds(preview);
			near(box.maxX, motor.variant.bodyFace / 2, 'NEMA $frame face');
			near(box.minZ, -motor.bodyLength, 'NEMA $frame body');
			near(box.maxZ, motor.variant.shaftLength, 'NEMA $frame shaft');
			preview.close();
		}
		var motor = NemaStepper.frame(17, 40);
		check(NemaStepper.model("23HS22-2804S").variant.shaftDiameter == 6.35, "named motor variant shaft");
		check(NemaStepper.frame(23).spec.boltSpacing == 47.14, "NEMA frame bolt spacing");
		check(motor.designation == "GENERIC-NEMA17-L40", "motor designation");
		throws(() -> NemaStepper.frame(11), 'Unknown NEMA frame "11"');
		var envelope = motor.geometry(Envelope);
		var shaftBeyondPilot = Math.PI * 2.5 * 2.5 * (24 - 2);
		near(envelope.volume(), 42 * 42 * 40 + Math.PI * 11 * 11 * 2 + shaftBeyondPilot, "motor envelope volume");
		envelope.close();
		near(motor.connector("bolt1").frame.x, 15.5, "bolt connector x");
		near(motor.connector("bolt3").frame.y, -15.5, "bolt connector y");
		near(motor.connector("shaftTip").frame.z, 24, "shaft tip connector");
		check(motor.mountScrew(8).spec.size == "M3", "NEMA 17 uses M3");
		var cutout = motor.mountingCutout(5);
		near(cutout.volume(), Math.PI * (11.1 * 11.1 + 4 * 1.7 * 1.7) * 5, "motor cutout volume");
		cutout.close();
	}

	static function fasteners():Void {
		var bolt = HexBolt.metric("M6", 25);
		check(bolt.designation == "ISO4017-M6x25", "bolt designation");
		near(bolt.pitch, 1, "M6 bolt pitch");
		near(bolt.threadLength, 25, "ISO 4017 bolts are fully threaded");
		throws(() -> HexBolt.metric("M7", 20), 'Unknown hex bolt size "M7"');
		throws(() -> HexBolt.metric("M6", 0), "positive length");

		var corner = bolt.acrossCorners;
		near(corner, 10 / Math.cos(Math.PI / 6), "bolt across corners");
		var envelope = bolt.geometry(Envelope);
		solid(envelope, "bolt envelope");
		near(envelope.volume(), Math.PI * (corner / 2) * (corner / 2) * 4 + Math.PI * 9 * 25, "bolt envelope volume");
		envelope.close();
		var preview = bolt.geometry();
		solid(preview, "bolt preview");
		near(preview.volume(), Math.sqrt(3) / 2 * 100 * 4 + Math.PI * 9 * 25, "bolt preview volume");
		var box = bounds(preview);
		near(box.maxX, 5, "bolt head across flats");
		near(box.minZ, -25, "bolt tip");
		near(box.maxZ, 4, "bolt head top");
		preview.close();
		near(bolt.connector("head").frame.z, 0, "bolt head connector");
		near(bolt.connector("tip").frame.z, -25, "bolt tip connector");

		near(bolt.clearanceDiameter(Fine), 6.4, "bolt fine clearance");
		var clearance = bolt.clearanceHole(10);
		near(clearance.volume(), Math.PI * 3.3 * 3.3 * 10, "bolt clearance hole volume");
		clearance.close();
		var tap = bolt.tapHole(12);
		near(tap.volume(), Math.PI * 2.5 * 2.5 * 12, "bolt tap hole volume");
		tap.close();
		var seatRadius = (corner + 0.5) / 2;
		var counterbore = bolt.counterboreHole(6);
		solid(counterbore, "bolt counterbore tool");
		near(counterbore.volume(), Math.PI * (seatRadius * seatRadius * 4.5 + 3.3 * 3.3 * 1.5), "bolt counterbore volume");
		counterbore.close();
		throws(() -> bolt.counterboreHole(4), "needs depth over");

		var nut = HexNut.metric("M6");
		check(nut.designation == "ISO4032-M6", "nut designation");
		throws(() -> HexNut.metric("M7"), 'Unknown hex nut size "M7"');
		var nutEnvelope = nut.geometry(Envelope);
		solid(nutEnvelope, "nut envelope");
		near(nutEnvelope.volume(), Math.sqrt(3) / 2 * 100 * 5.2, "nut envelope volume");
		nutEnvelope.close();
		var nutPreview = nut.geometry();
		solid(nutPreview, "nut preview");
		near(nutPreview.volume(), Math.sqrt(3) / 2 * 100 * 5.2 - Math.PI * 9 * 5.2, "nut preview volume");
		nutPreview.close();
		near(nut.connector("axis").frame.z, 2.6, "nut axis connector");
		var pocket = nut.pocket(6);
		solid(pocket, "nut pocket");
		near(pocket.volume(), Math.sqrt(3) / 2 * 10.5 * 10.5 * 6, "nut pocket volume");
		pocket.close();
		throws(() -> nut.pocket(5), "needs depth at least");

		var washer = FlatWasher.metric("M6");
		check(washer.designation == "ISO7089-M6", "washer designation");
		throws(() -> FlatWasher.metric("M7"), 'Unknown flat washer size "M7"');
		var washerPart = washer.geometry();
		solid(washerPart, "washer");
		near(washerPart.volume(), Math.PI * (6 * 6 - 3.2 * 3.2) * 1.6, "washer volume");
		washerPart.close();
		near(washer.connector("axis").frame.z, 0.8, "washer axis connector");
	}

	static function dimensions():Void {
		check(Dimension.format(12.7) == "12.7", "12.7 formats without binary noise");
		check(Dimension.format(0.1 + 0.2) == "0.3", "0.1 + 0.2 formats as 0.3");
		check(Dimension.format(20) == "20", "whole numbers format without a decimal point");
		check(Dimension.format(6.35) == "6.35", "6.35 formats exactly");
		check(Dimension.format(3000000.001) == "3000000.001", "large dimensions format beyond the integer range");
		check(Dimension.format(0.0625) == "0.063", "values round to 0.001");
		check(Dimension.format(-2.5) == "-2.5", "negative values keep their sign");
		check(Dimension.format(-0.0001) == "0", "values rounding to zero lose their sign");
	}

	static function shafts():Void {
		var key = ParallelKey.forShaft(6, 6);
		check(key.designation == "DIN6885-B-2x2x6", "key designation");
		check(key.spec.width == 2 && key.spec.height == 2, "key cross-section");
		var keyPreview = key.geometry();
		solid(keyPreview, "key preview");
		near(keyPreview.volume(), 2 * 2 * 6, "key volume");
		keyPreview.close();
		throws(() -> ParallelKey.metric("9x9", 10), 'Unknown parallel key "9x9"');
		throws(() -> ParallelKey.forShaft(100, 10), "No DIN 6885-1 key fits shaft diameter 100");
		throws(() -> ParallelKey.forShaft(0, 10), "positive shaft diameter");
		throws(() -> ParallelKey.forShaft(5.9, 10), "No DIN 6885-1 key fits shaft diameter 5.9");
		throws(() -> ParallelKey.custom({minShaft: 6, maxShaft: 8, width: 2, height: 2, shaftDepth: 2.1, hubDepth: 1}, 10),
			"inconsistent DIN 6885 dimensions");

		var shaft = new SteppedShaft(
			[{diameter: 8, length: 51.5}, {diameter: 6, length: 8.5}],
			[{name: "bearingA", z: 10}, {name: "bearingB", z: 43}],
			[{name: "outputKey", z0: 52, key: key}],
			[{name: "ring", z0: 50, width: 1.2, diameter: 7.6}]
		);
		near(shaft.totalLength, 60, "shaft total length");
		near(shaft.diameterAt(0), 8, "shaft start diameter");
		near(shaft.diameterAt(51.5), 6, "shaft boundary belongs to the next section");
		near(shaft.diameterAt(60), 6, "shaft end diameter");
		throws(() -> shaft.diameterAt(-1), "outside 0..60");
		throws(() -> shaft.diameterAt(61), "outside 0..60");

		var envelope = shaft.geometry(Envelope);
		solid(envelope, "shaft envelope");
		near(envelope.volume(), Math.PI * 16 * 51.5 + Math.PI * 9 * 8.5, "shaft envelope volume");
		envelope.close();
		var preview = shaft.geometry();
		solid(preview, "shaft preview");
		// Groove: an outer annulus 7.6..8 wide 1.2. Keyway: the 2 mm slot's circular segment above y=1.8.
		var grooveVolume = Math.PI * (16 - 3.8 * 3.8) * 1.2;
		var keywayVolume = 6 * (Math.sqrt(8) + 9 * Math.atan2(1, Math.sqrt(8)) - 3.6);
		near(preview.volume(), Math.PI * 16 * 51.5 + Math.PI * 9 * 8.5 - grooveVolume - keywayVolume,
			"preview removes the outer groove annulus and keyway", 1e-3);
		var previewBox = bounds(preview);
		near(previewBox.maxX, 4, "groove leaves the shaft's outer surface elsewhere");
		preview.close();
		check(shaft.designation == "SHAFT-8x51.5-6x8.5-F8:bearingA@10-F8:bearingB@43-G4:ring@50x1.2x7.6-K9:outputKey@52:DIN6885-B-2x2x6-S6x8-D1.2x1",
			"shaft designation includes its faces, keyway and groove");
		check(new SteppedShaft([{diameter: 6.35, length: 20}]).designation == "SHAFT-6.35x20",
			"fractional shaft designation is rounded, not a raw float");

		near(shaft.connector("input").frame.z, 0, "shaft input connector");
		near(shaft.connector("output").frame.z, 60, "shaft output connector");
		near(shaft.connector("bearingA").frame.z, 10, "shaft bearingA connector");
		var seat = shaft.connector("outputKey").frame;
		near(seat.z, 55, "keyway connector z");
		near(seat.y, 3 - 1.2, "keyway connector floor");
		near(shaft.connector("ring").frame.z, 50.6, "groove connector z");
		near(shaft.journalDiameterAt(10, BearingShaftFit.Slip), 7.99, "shaft slip journal fit");
		near(shaft.journalDiameterAllowance(10, 0.02), 8.02, "shaft explicit journal allowance");

		var detailedShaft = new SteppedShaft(
			[{diameter: 12, length: 20}, {diameter: 8, length: 20}], null, null, null,
			{inputChamfer: 1, outputChamfer: 0.5,
				inputThread: {diameter: 10, pitch: 1.5, length: 6},
				outputThread: {diameter: 6, pitch: 1, length: 5},
				shoulders: [{z: 20, fillet: 0.5, reliefWidth: 2, reliefDiameter: 7.5}]});
		check(detailedShaft.designation == "SHAFT-12x20-8x20-CI1-CO0.5-S20-R0.5-U2x7.5-TI10x1.5x6-TO6x1x5",
			"detailed shaft designation includes chamfers, shoulders and threads");
		var otherDetailedShaft = new SteppedShaft(
			[{diameter: 12, length: 20}, {diameter: 8, length: 20}], null, null, null,
			{inputChamfer: 2, outputChamfer: 0.5,
				inputThread: {diameter: 10, pitch: 1.5, length: 6},
				outputThread: {diameter: 6, pitch: 1, length: 5},
				shoulders: [{z: 20, fillet: 0.5, reliefWidth: 2, reliefDiameter: 7.5}]});
		check(otherDetailedShaft.bom.partNumber != detailedShaft.bom.partNumber,
			"different shaft features have different BOM part numbers");
		var shaftBom = new machinekit.component.Bom();
		shaftBom.addComponent(detailedShaft);
		shaftBom.addComponent(otherDetailedShaft);
		check(shaftBom.lines().length == 2, "different shaft features remain separate in the BOM");
		near(detailedShaft.connector("inputThread").frame.z, 3, "input thread connector");
		near(detailedShaft.connector("outputThread").frame.z, 37.5, "output thread connector");
		var detailedEnvelope = detailedShaft.geometry(Envelope);
		var detailedPreview = detailedShaft.geometry();
		solid(detailedEnvelope, "detailed shaft envelope");
		solid(detailedPreview, "detailed shaft preview");
		check(detailedPreview.volume() < detailedEnvelope.volume(), "detailed shaft machining features");
		check(bounds(detailedPreview).maxX <= 6.001, "detailed shaft shoulder envelope");
		detailedPreview.close();
		detailedEnvelope.close();

		throws(() -> new SteppedShaft([{diameter: -1, length: 10}]), "positive diameter and length");
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}], [{name: "x", z: 20}]),
			'Face "x" lies outside the shaft');
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}], null,
			[{name: "k", z0: 5, key: ParallelKey.forShaft(6, 8)}]), 'Keyway "k" lies outside the shaft');
		throws(() -> new SteppedShaft(
			[{diameter: 8, length: 10}, {diameter: 6, length: 10}], null,
			[{name: "k", z0: 8, key: ParallelKey.forShaft(6, 4)}]), 'Keyway "k" must lie within one shaft section');
		throws(() -> new SteppedShaft([{diameter: 2, length: 10}], null,
			[{name: "k", z0: 0, key: ParallelKey.forShaft(6, 4)}]), 'Keyway "k" is deeper than the shaft radius');
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}], null, null,
			[{z0: 0, width: 20, diameter: 6}]), "Retaining ring groove lies outside the shaft");
		throws(() -> new SteppedShaft(
			[{diameter: 8, length: 10}, {diameter: 6, length: 10}], null, null,
			[{z0: 8, width: 4, diameter: 5}]), "Retaining ring groove must lie within one shaft section");
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}], null, null,
			[{z0: 0, width: 2, diameter: 9}]), "Retaining ring groove diameter must be smaller than the shaft");
		throws(() -> new SteppedShaft(
			[{diameter: 10, length: 10}, {diameter: 12, length: 2}, {diameter: 10, length: 10}], null,
			[{name: "k", z0: 8, key: ParallelKey.forShaft(10, 6)}]), 'Keyway "k" must lie within one shaft section');
		throws(() -> new SteppedShaft(
			[{diameter: 10, length: 10}, {diameter: 12, length: 2}, {diameter: 10, length: 10}], null, null,
			[{z0: 9, width: 4, diameter: 9}]), "Retaining ring groove must lie within one shaft section");
		throws(() -> new SteppedShaft([{diameter: 12, length: 40}], null,
			[{name: "k", z0: 0, key: ParallelKey.metric("10x8", 20)}]), 'Keyway "k" is too wide for the shaft');
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}], null, null, null,
			{inputChamfer: 4.1}), "smaller than its radius");
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}], null, null, null,
			{inputThread: {diameter: 8, pitch: 1, length: 4}}), "below the shaft diameter");
		throws(() -> new SteppedShaft([{diameter: 12, length: 20}, {diameter: 8, length: 20}], null, null, null,
			{shoulders: [{z: 20, fillet: 2}]}), "fillet at z=20 is too large");
		throws(() -> new SteppedShaft([{diameter: 8, length: 10}, {diameter: 6, length: 10}], null, null, null,
			{shoulders: [{z: 5, reliefWidth: 1, reliefDiameter: 4}]}), "not a section boundary");
	}

	static function shaftHardware():Void {
		var ring = RetainingRing.forShaft(8);
		check(ring.designation == "DIN471-8", "ring designation");
		check(ring.spec.grooveDiameter == 7.6 && ring.spec.outerDiameter == 12.2, "ring dimensions (DIN 471 d2)");
		check(ring.thickness == 0.8 && ring.grooveSpec().width == 0.9, "ring thickness differs from groove width");
		check(ring.grooveSpec().diameter == 7.6, "ring groove diameter");
		throws(() -> RetainingRing.forShaft(9), 'Unknown retaining ring shaft diameter "9"');
		throws(() -> RetainingRing.forShaft(8.5), 'Unknown retaining ring shaft diameter "8.5"');
		check(RetainingRing.forShaft(12).spec.grooveDiameter == 11.5, "12 mm ring groove diameter");

		var envelope = ring.geometry(Envelope);
		solid(envelope, "ring envelope");
		near(envelope.volume(), annulus(12.2, 7.6, 0.8), "ring envelope volume");
		envelope.close();
		var preview = ring.geometry();
		solid(preview, "ring preview");
		near(preview.volume(), annulus(12.2, 7.6, 0.8) * 8 / 9, "ring preview volume (gapped)");
		preview.close();
		near(ring.connector("seat").frame.z, 0.4, "ring seat connector");

		var collar = ShaftCollar.forShaft(8);
		check(collar.designation == "COLLAR-8", "collar designation");
		check(collar.spec.setScrew == "M4", "collar set screw size");
		throws(() -> ShaftCollar.forShaft(9), 'Unknown shaft collar bore diameter "9"');
		throws(() -> ShaftCollar.forShaft(6.35), 'Unknown shaft collar bore diameter "6.35"');
		var collarPart = collar.geometry();
		solid(collarPart, "collar");
		near(collarPart.volume(), Math.PI * (8 * 8 - 4 * 4) * 11, "collar volume");
		collarPart.close();
		near(collar.connector("axis").frame.z, 5.5, "collar axis connector");
	}

	static function structural():Void {
		var tube = new RectTube(40, 40, 3);
		check(tube.designation == "RECT-40x40x3", "rect tube designation");
		throws(() -> new RectTube(10, 10, 6), "wall is too thick");
		var tubePart = tube.geometry(100);
		solid(tubePart, "rect tube");
		near(tubePart.volume(), (40 * 40 - 34 * 34) * 100, "rect tube volume");
		tubePart.close();

		var round = new RoundTube(20, 2);
		check(round.designation == "ROUND-20x2", "round tube designation");
		throws(() -> new RoundTube(10, 6), "wall is too thick");
		var roundPart = round.geometry(50);
		solid(roundPart, "round tube");
		near(roundPart.volume(), Math.PI * (10 * 10 - 8 * 8) * 50, "round tube volume");
		roundPart.close();

		var angle = new Angle(30, 30, 3);
		check(angle.designation == "ANGLE-30x30x3", "angle designation");
		throws(() -> new Angle(10, 10, 10), "thickness must be less than either leg");
		var anglePart = angle.geometry(40);
		solid(anglePart, "angle");
		near(anglePart.volume(), 171 * 40, "angle volume");
		anglePart.close();

		var channel = new Channel(40, 20, 3);
		check(channel.designation == "CHANNEL-40x20x3", "channel designation");
		throws(() -> new Channel(10, 10, 6), "thickness is too thick for its height");
		throws(() -> new Channel(40, 3, 3), "must be less than the flange width");
		var channelPart = channel.geometry(60);
		solid(channelPart, "channel");
		near(channelPart.volume(), 222 * 60, "channel volume");
		channelPart.close();

		var bar = new FlatBar(50, 5);
		check(bar.designation == "FLAT-50x5", "flat bar designation");
		throws(() -> new FlatBar(-1, 5), "positive width and thickness");
		var barPart = bar.geometry(200);
		solid(barPart, "flat bar");
		near(barPart.volume(), 50 * 5 * 200, "flat bar volume");
		barPart.close();

		var tslot = new TSlotExtrusion(20);
		check(tslot.designation == "GENERIC-TSLOT-20x20", "generic t-slot designation");
		throws(() -> new TSlotExtrusion(-1), "positive size");
		var tslotPart = tslot.geometry(100);
		solid(tslotPart, "t-slot extrusion");
		var tslotBox = bounds(tslotPart);
		near(tslotBox.maxX, 10, "t-slot outer boundary");
		near(tslotBox.minZ, 0, "t-slot start");
		near(tslotBox.maxZ, 100, "t-slot end");
		var tslotVolume = tslotPart.volume();
		check(tslotVolume < 20 * 20 * 100, "t-slot removes material for slots and bore");
		check(tslotVolume > 10 * 20 * 100, "t-slot keeps most of its cross-section");
		// Four slots (3 mm x 6 mm throat plus 2 mm x 7 mm head) and a 3 mm bore, cut end to end.
		near(tslotVolume, (20 * 20 - 4 * (3 * 6 + 2 * 7) - Math.PI * 1.5 * 1.5) * 100, "t-slot volume");
		tslotPart.close();
		check(new FlatBar(12.7, 3.2).designation == "FLAT-12.7x3.2", "fractional flat bar designation");
		check(new TSlotExtrusion(25.4).designation == "GENERIC-TSLOT-25.4x25.4", "fractional generic t-slot designation");
		var hfs = TSlotExtrusion.forProfile("HFS5-2020");
		check(hfs.designation == "HFS5-2020" && hfs.family == "MISUMI HFS5", "catalog t-slot designation");
		near(hfs.slotWidth, 6, "HFS5 slot opening");
		near(hfs.tWidth, 12, "HFS5 slot head width");
		near(hfs.boreDiameter, 4.2, "HFS5 centre bore");
		var hfsPart = hfs.geometry(100);
		solid(hfsPart, "catalog t-slot extrusion");
		near(bounds(hfsPart).maxX, 10, "catalog t-slot outer boundary");
		check(hfsPart.volume() < 20 * 20 * 100, "catalog t-slot removes slot material");
		hfsPart.close();
		var hfs2040 = TSlotExtrusion.forProfile("HFS5-2040");
		check(hfs2040.height == 40, "catalog rectangular t-slot height");
		var hfs2040Part = hfs2040.geometry(50);
		solid(hfs2040Part, "catalog rectangular t-slot extrusion");
		check(hfs2040Part.volume() < 20 * 40 * 50, "catalog rectangular t-slot removes material");
		hfs2040Part.close();
		throws(() -> TSlotExtrusion.forProfile("HFS5-3030"), 'Unknown T-slot profile "HFS5-3030"');

		var frame = new FrameAssembly();
		frame.point("A", 0, 0, 0);
		frame.point("B", 0, 0, 500);
		frame.point("C", 300, 0, 500);
		throws(() -> frame.point("A", 1, 1, 1), 'Duplicate frame point "A"');
		frame.point("D", 300, 0, 0);
		frame.member("upright", "A", "B", tube);
		frame.member("beam", "B", "C", tube);
		frame.member("brace", "C", "D", tslot);
		throws(() -> frame.member("upright", "A", "C", tube), 'Duplicate frame member "upright"');
		throws(() -> frame.member("bad", "A", "Z", tube), 'Unknown frame point "Z"');
		throws(() -> frame.member("bad", "A", "A", tube), 'needs distinct endpoints');
		throws(() -> frame.length("missing"), 'Unknown frame member "missing"');

		var zeroFrame = new FrameAssembly();
		zeroFrame.point("A", 0, 0, 0);
		zeroFrame.point("D", 0, 0, 0);
		zeroFrame.member("zero", "A", "D", tube);
		throws(() -> zeroFrame.geometry("zero"), 'has coincident endpoints');

		near(frame.length("upright"), 500, "upright length");
		near(frame.length("beam"), 300, "beam length");
		var upright = frame.geometry("upright");
		solid(upright, "upright member");
		near(upright.volume(), (40 * 40 - 34 * 34) * 500, "upright member volume");
		var uprightBox = bounds(upright);
		near(uprightBox.maxX, 20, "upright cross-section extent");
		near(uprightBox.minZ, 0, "upright start");
		near(uprightBox.maxZ, 500, "upright end");
		upright.close();
		var beam = frame.geometry("beam");
		solid(beam, "beam member");
		var beamBox = bounds(beam);
		near(beamBox.minX, 0, "beam start");
		near(beamBox.maxX, 300, "beam end");
		near(beamBox.minZ, 480, "beam cross-section low");
		near(beamBox.maxZ, 520, "beam cross-section high");
		beam.close();

		// Section orientation: the channel's local +Y (its 40 mm height) follows the reference;
		// local +X (the 20 mm flanges) is reference x member axis.
		var channelFrame = new FrameAssembly();
		channelFrame.point("P", 0, 0, 0);
		channelFrame.point("Q", 100, 0, 0);
		channelFrame.member("up", "P", "Q", channel);
		channelFrame.member("side", "P", "Q", channel, new Vector(0, 1, 0));
		channelFrame.member("parallel", "P", "Q", channel, new Vector(2, 0, 0));
		throws(() -> channelFrame.member("zeroRef", "P", "Q", channel, new Vector(0, 0, 0)), "reference has zero length");
		var upChannel = channelFrame.geometry("up");
		var upBox = bounds(upChannel);
		near(upBox.minX, 0, "channel member start");
		near(upBox.maxX, 100, "channel member end");
		near(upBox.minZ, 0, "channel height starts on the member axis");
		near(upBox.maxZ, 40, "channel height follows the default +Z reference");
		upChannel.close();
		var sideChannel = channelFrame.geometry("side");
		var sideBox = bounds(sideChannel);
		// Reference +Y: local X = Y x X = -Z, so the flanges hang below the member axis.
		near(sideBox.minZ, -20, "channel flanges follow reference x axis");
		near(sideBox.maxZ, 0, "channel web on the member axis");
		sideChannel.close();
		throws(() -> channelFrame.geometry("parallel"), "reference is parallel to its axis");

		var cutList = frame.cutList();
		check(cutList.length == 2, "cut list groups by profile");
		for (line in cutList) {
			if (line.designation == tube.designation) {
				check(line.quantity == 2, "rect tube cut list quantity");
				near(line.totalLength, 800, "rect tube cut list total length");
			} else if (line.designation == tslot.designation) {
				check(line.quantity == 1, "t-slot cut list quantity");
				near(line.totalLength, 500, "t-slot cut list total length");
			} else {
				throw 'unexpected cut list designation "${line.designation}"';
			}
		}

		var detailedFrame = new FrameAssembly();
		detailedFrame.point("A", 0, 0, 0);
		detailedFrame.point("B", 0, 0, 100);
		detailedFrame.member("detailed", "A", "B", tube, null, Mitre(10));
		near(detailedFrame.length("detailed"), 100, "detailed frame centreline length");
		near(detailedFrame.cutLength("detailed"), 105, "mitred stock length reaches the long point");
		var detailedPart = detailedFrame.geometry("detailed");
		solid(detailedPart, "detailed frame member");
		var detailedBox = bounds(detailedPart);
		near(detailedBox.minZ, -5, "mitre long point extends through the start node");
		near(detailedBox.maxZ, 100, "square frame end remains at its node");
		near(detailedPart.volume(), (40 * 40 - 34 * 34) * 100, "node-centred mitre preserves full stock volume");
		detailedPart.close();
		near(detailedFrame.cutList()[0].totalLength, 105, "mitre cut list includes its long point");
		throws(() -> detailedFrame.member("badSetback", "A", "B", tube, null, Mitre(0), null),
			"end-cut setback must be positive");

		var mitreCorner = new FrameAssembly(), mitreTube = new RectTube(20, 20, 2);
		mitreCorner.point("A", -100, 0, 0);
		mitreCorner.point("N", 0, 0, 0);
		mitreCorner.point("B", 0, 100, 0);
		mitreCorner.member("horizontal", "A", "N", mitreTube, null, null, Mitre(20));
		mitreCorner.member("vertical", "N", "B", mitreTube, null, Mitre(20));
		near(mitreCorner.cutLength("horizontal"), 110, "horizontal mitre cut length to long point");
		near(mitreCorner.cutLength("vertical"), 110, "vertical mitre cut length to long point");
		var horizontalMiter = mitreCorner.geometry("horizontal"), verticalMiter = mitreCorner.geometry("vertical");
		var analyticCornerVolume = 2 * (20 * 20 - 16 * 16) * 100;
		near(horizontalMiter.volume() + verticalMiter.volume(), analyticCornerVolume,
			"two 20 mm mitred tubes preserve analytic stock volume");
		checkOverlap(horizontalMiter, verticalMiter, 0, "L-corner mitres meet without overlap");
		var miterUnion = Solids.union([mitreCorner.geometry("horizontal"), mitreCorner.geometry("vertical")]);
		near(miterUnion.volume(), analyticCornerVolume, "mitred L-corner union matches analytic stock volume");
		miterUnion.close();
		near(mitreCorner.cutList()[0].totalLength, 220, "mitred corner cut list uses long points");

		var copeFrame = new FrameAssembly(), copeTube = new RoundTube(20, 2);
		copeFrame.point("left", -50, 0, 0);
		copeFrame.point("node", 0, 0, 0);
		copeFrame.point("right", 50, 0, 0);
		copeFrame.point("branch", 0, 50, 0);
		copeFrame.member("mateLeft", "left", "node", copeTube);
		copeFrame.member("mateRight", "node", "right", copeTube);
		copeFrame.member("coped", "node", "branch", copeTube, null, Cope(10));
		near(copeFrame.cutLength("coped"), 50, "cope keeps node-to-node stock length");
		var copedPart = copeFrame.geometry("coped"), matePart = copeFrame.geometry("mateLeft");
		check(copedPart.volume() < (Math.PI * (10 * 10 - 8 * 8) * 50), "cope removes stock at the mating axis");
		checkOverlap(matePart, copedPart, 0, "cope cutter axis follows the mating member at the node");
	}

	static function gears():Void {
		var gear = new SpurGear(2, 20, 12);
		check(gear.designation == "SPUR-M2-20T", "spur gear designation");
		near(gear.pitchDiameter, 40, "spur gear pitch diameter");
		near(gear.baseDiameter, 40 * Math.cos(SpurGear.STANDARD_PRESSURE_ANGLE), "spur gear base diameter");
		near(gear.outsideDiameter, 44, "spur gear outside diameter");
		near(gear.rootDiameter, 35, "spur gear root diameter");
		near(gear.pitchToothThickness(), Math.PI, "spur gear pitch tooth thickness");
		throws(() -> new SpurGear(-1, 20, 12), "positive module");
		throws(() -> new SpurGear(2, 5, 12), "at least 6 teeth");
		check(SpurGear.minimumUnshiftedTeeth(SpurGear.STANDARD_PRESSURE_ANGLE) == 18, "20 degree undercut limit");
		throws(() -> new SpurGear(2, 17, 12), "would require undercut");
		var shifted = new SpurGear(2, 17, 12, SpurGear.STANDARD_PRESSURE_ANGLE, 0.1, 0.2);
		check(shifted.designation == "SPUR-M2-17T-X0.1-B0.2", "profile-shifted gear designation");
		near(shifted.outsideDiameter, 38.4, "profile-shifted gear outside diameter");
		near(shifted.rootDiameter, 29.4, "profile-shifted gear root diameter");
		near(shifted.pitchToothThickness(), Math.PI + 0.4 * Math.tan(SpurGear.STANDARD_PRESSURE_ANGLE) - 0.2,
			"profile-shifted gear tooth thickness");
		near(SpurGear.minimumProfileShift(17), 1 - 17 * Math.pow(Math.sin(SpurGear.STANDARD_PRESSURE_ANGLE), 2) / 2,
			"profile-shift minimum");
		throws(() -> new SpurGear(2, 20, 12, SpurGear.STANDARD_PRESSURE_ANGLE, -1), "between -1 and 1.25");
		throws(() -> new SpurGear(2, 20, 12, SpurGear.STANDARD_PRESSURE_ANGLE, 0, -0.1), "non-negative");
		throws(() -> new SpurGear(2, 20, 12, SpurGear.STANDARD_PRESSURE_ANGLE, 0, 4), "leaves no tooth thickness");
		throws(() -> new SpurGear(2, 20, -1), "positive face width");

		var part = gear.geometry();
		solid(part, "spur gear");
		var box = bounds(part);
		near(box.maxX, gear.outsideDiameter / 2, "spur gear outside radius");
		check(box.maxX > gear.pitchDiameter / 2, "spur gear teeth extend past the pitch circle");
		part.close();

		var pinion = new SpurGear(2, 18, 12);
		throws(() -> pinion.centerDistance(new SpurGear(2.5, 20, 12)), "different modules");
		throws(() -> GearPair.mesh(pinion, new SpurGear(2, 20, 12, 25 * Math.PI / 180)), "different pressure angles");
		var pair = GearPair.mesh(pinion, gear);
		MachineAssemblyDescriptionTests.roundTrip(pair, "gear pair");
		near(pair.centerDistance, (pinion.pitchDiameter + gear.pitchDiameter) / 2, "gear pair centre distance");
		near(pair.operatingPressureAngle, SpurGear.STANDARD_PRESSURE_ANGLE, "gear pair operating pressure angle");
		near(pair.ratio(), gear.teeth / pinion.teeth, "gear pair ratio");
		near(pair.pose().x, pair.centerDistance, "gear pair pose offset");
		var pairModel = new AssemblyModel();
		pair.addTo(pairModel, "gearbox");
		var pairState = pairModel.initialState("gear-pair");
		check(pair.connector("inputAxis", "gearbox").instanceId == "gearbox/a",
			"gear pair exposes its input axis through the prefix");
		near(pairState.worldConnector("gearbox/b", "axis").x, pair.centerDistance,
			"gear pair places output gear through MachineAssembly.addTo");
		check(pair.billOfMaterials().quantity(pinion.designation) == 1 &&
			pair.billOfMaterials().quantity(gear.designation) == 1, "gear pair BOM counts both gears");
		var shiftedPair = GearPair.mesh(new SpurGear(2, 17, 12, SpurGear.STANDARD_PRESSURE_ANGLE, 0.1, 0.1),
			new SpurGear(2, 20, 12, SpurGear.STANDARD_PRESSURE_ANGLE, 0.2, 0.05));
		near(shiftedPair.profileShiftSum, 0.3, "shifted gear pair profile shift");
		near(shiftedPair.backlash, 0.15, "shifted gear pair backlash");
		near(Math.tan(shiftedPair.operatingPressureAngle) - shiftedPair.operatingPressureAngle,
			Math.tan(SpurGear.STANDARD_PRESSURE_ANGLE) - SpurGear.STANDARD_PRESSURE_ANGLE
				+ 2 * 0.3 * Math.tan(SpurGear.STANDARD_PRESSURE_ANGLE) / (17 + 20),
			"shifted gear pair involute operating angle");
		near(shiftedPair.centerDistance, (17 + 20) * Math.cos(SpurGear.STANDARD_PRESSURE_ANGLE)
			/ Math.cos(shiftedPair.operatingPressureAngle), "shifted gear pair centre distance");
		near(shiftedPair.pose().x, shiftedPair.centerDistance, "shifted gear pair pose offset");
		var shiftedTwenty = GearPair.mesh(new SpurGear(1, 20, 10, SpurGear.STANDARD_PRESSURE_ANGLE, 0.5),
			new SpurGear(1, 20, 10, SpurGear.STANDARD_PRESSURE_ANGLE, 0.5));
		near(shiftedTwenty.centerDistance, 20.88, "20+20 shifted gear centre distance", 0.02);
		near(shiftedTwenty.operatingPressureAngle, 25.8 * Math.PI / 180, "20+20 shifted gear operating angle", 0.001);
		var minimumSixShift = SpurGear.minimumProfileShift(6);
		throws(() -> new SpurGear(1, 6, 6, SpurGear.STANDARD_PRESSURE_ANGLE, minimumSixShift), "tip tooth thickness");
		throws(() -> new SpurGear(1, 20, 10, SpurGear.STANDARD_PRESSURE_ANGLE, 1.2), "tip tooth thickness");
		throws(() -> GearPair.mesh(
			new SpurGear(2, 40, 12, SpurGear.STANDARD_PRESSURE_ANGLE, -0.9),
			new SpurGear(2, 40, 12, SpurGear.STANDARD_PRESSURE_ANGLE, -0.9)),
			"compatible profile shifts");
		var shiftedASolid = shiftedPair.a.geometry(), shiftedBSolid = shiftedPair.b.geometry();
		var shiftedMaxPenetration = 0.0;
		for (sample in 0...9) {
			var aAngle = 2 * Math.PI * sample / (shiftedPair.a.teeth * 8);
			var bAngle = shiftedPair.bRotation - aAngle * shiftedPair.a.teeth / shiftedPair.b.teeth;
			var aPlaced = shiftedASolid.placed(new Location(new Plane(new Vector(0, 0, 0),
				new Vector(Math.cos(aAngle), Math.sin(aAngle), 0), Vector.Z())));
			var bPlaced = shiftedBSolid.placed(new Location(new Plane(new Vector(shiftedPair.centerDistance, 0, 0),
				new Vector(Math.cos(bAngle), Math.sin(bAngle), 0), Vector.Z())));
			var overlap = aPlaced.intersect(bPlaced);
			shiftedMaxPenetration = Math.max(shiftedMaxPenetration, overlap.volume());
			overlap.close();
			aPlaced.close();
			bPlaced.close();
		}
		shiftedASolid.close();
		shiftedBSolid.close();
		check(shiftedMaxPenetration <= 0.05,
			'shifted gear mesh penetrates by $shiftedMaxPenetration mm^3 over one tooth pitch');
		var aSolid = pinion.geometry(), bSolid = gear.geometry();
		var maxPenetration = 0.0;
		for (sample in 0...17) {
			var aAngle = 2 * Math.PI * sample / (pinion.teeth * 16);
			var bAngle = pair.bRotation - aAngle * pinion.teeth / gear.teeth;
			var aPlaced = aSolid.placed(new Location(new Plane(new Vector(0, 0, 0),
				new Vector(Math.cos(aAngle), Math.sin(aAngle), 0), Vector.Z())));
			var bPlaced = bSolid.placed(new Location(new Plane(new Vector(pair.centerDistance, 0, 0),
				new Vector(Math.cos(bAngle), Math.sin(bAngle), 0), Vector.Z())));
			var overlap = aPlaced.intersect(bPlaced);
			maxPenetration = Math.max(maxPenetration, overlap.volume());
			overlap.close();
			aPlaced.close();
			bPlaced.close();
		}
		aSolid.close();
		bSolid.close();
		// The polyline tooth flanks and CAD booleans can leave tiny numerical overlap.
		check(maxPenetration <= 0.05, 'gear mesh penetrates by $maxPenetration mm^3 over one tooth pitch');
		var badA = pinion.geometry();
		var badBBase = gear.geometry();
		var badB = badBBase.placed(new Location(new Plane(new Vector(pair.centerDistance, 0, 0),
			new Vector(Math.cos(pair.bRotation + Math.PI / gear.teeth),
				Math.sin(pair.bRotation + Math.PI / gear.teeth), 0), Vector.Z())));
		var badOverlap = badA.intersect(badB);
		check(badOverlap.volume() > 1, "wrong tooth phase must produce detectable interference");
		badOverlap.close();
		badA.close();
		badB.close();
		badBBase.close();

		// SpurGear centres a tooth on local angle 0, so `a` points a tooth at the mesh point and
		// `b` must present a tooth space at its local angle pi: (pi - turn) / pitch angle = k + 1/2.
		for (teeth in [20, 21]) {
			var meshed = GearPair.mesh(pinion, new SpurGear(2, teeth, 12));
			var meshedPose = meshed.pose();
			var turn = 2 * Math.atan2(meshedPose.qz, meshedPose.qw);
			near(turn, teeth % 2 == 0 ? Math.PI / teeth : 0.0, 'gear pair turn for $teeth teeth', 1e-9);
			near(meshed.bRotation, turn, 'gear pair bRotation for $teeth teeth', 1e-9);
			var phase = (Math.PI - turn) / (2 * Math.PI / teeth);
			near(phase - Math.ffloor(phase), 0.5, 'gear with $teeth teeth has a tooth space at the mesh point', 1e-9);
			near(meshedPose.x, meshed.centerDistance, 'gear pair offset for $teeth teeth');
		}
		throws(() -> new SpurGear(2, 20, 12, 0.1), "between 14.5 and 25 degrees");
		throws(() -> new SpurGear(2, 20, 12, 0.5), "between 14.5 and 25 degrees");
		check(new SpurGear(2, 33, 12, 14.5 * Math.PI / 180).teeth == 33, "14.5 degree pressure angle accepted");
		check(new SpurGear(2, 20, 12, 25 * Math.PI / 180).teeth == 20, "25 degree pressure angle accepted");
		check(new SpurGear(0.8, 20, 5).designation == "SPUR-M0.8-20T", "fractional module designation");

		var rack = new Rack(2, 10, 12);
		check(rack.designation == "RACK-M2-10T", "rack designation");
		near(rack.length, 10 * Math.PI * 2, "rack length");
		throws(() -> new Rack(2, 0, 12), "at least one tooth");
		throws(() -> new Rack(2, 10, 12, SpurGear.STANDARD_PRESSURE_ANGLE, -1), "non-negative");

		var rackPart = rack.geometry();
		solid(rackPart, "rack");
		var rackBox = bounds(rackPart);
		near(rackBox.minX, 0, "rack face start");
		near(rackBox.maxX, 12, "rack face width");
		near(rackBox.minZ, 0, "rack length start");
		near(rackBox.maxZ, rack.length, "rack length end");
		// Square ends: a 0.01 mm slice at each end is the full bar section below the root line
		// (faceWidth x barHeight); the nearest tooth flank starts about 0.66 mm in.
		for (end in [0.0, 1.0]) {
			var slab = Part.box(100, 100, 1.01);
			var placedSlab = slab.translated(new Vector(0, 0, end == 0 ? -1 : rack.length - 0.01));
			slab.close();
			var cap = rackPart.intersect(placedSlab);
			placedSlab.close();
			near(cap.volume(), 12 * rack.barHeight * 0.01, end == 0 ? "rack start face is square" : "rack end face is square", 1e-3);
			cap.close();
		}
		rackPart.close();
		throws(() -> new Rack(2, 10, 12, 0.5), "between 14.5 and 25 degrees");
		check(new Rack(0.8, 10, 5).designation == "RACK-M0.8-10T", "fractional rack designation");
	}

	static function flangeBearingAssembly():Void {
		var bearing = DeepGrooveBearing.metric("6204");
		var block = new FlangeBearingAssembly(bearing);
		MachineAssemblyDescriptionTests.roundTrip(block, "flange bearing assembly");
		check(block.housing.fit == BearingHousingFit.Slip, "flange assembly uses a named housing fit");
		check(block.housing.mountScrew == "M6", "flange assembly mount screw size");
		near(block.housing.face, 68.4, "flange housing face leaves 1 mm around the M6 heads");
		near(block.housing.depth, 23.4, "flange housing depth");
		near(block.housing.boltSpacing, 56.4, "flange housing bolt spacing");
		for (designation in ["608", "6000", "6001", "6002", "6003", "6204"]) {
			var housing = new FlangeBearingHousing(DeepGrooveBearing.metric(designation));
			var screw = housing.mountScrewPart(10).spec, edge = (housing.face - housing.boltSpacing) / 2;
			check(edge >= screw.headDiameter / 2 + 1 - 1e-9, 'housing for $designation keeps screw heads on the face');
			check(edge > screw.clearanceCoarse / 2, 'housing for $designation bolt holes stay inside the edge');
			var part = housing.geometry();
			solid(part, 'housing for $designation');
			part.close();
		}

		var envelope = block.housing.geometry(Envelope);
		solid(envelope, "flange housing envelope");
		var envelopeVolume = envelope.volume();
		near(envelopeVolume, 68.4 * 68.4 * 23.4 - Math.PI * Math.pow((47 + block.housing.allowance) / 2, 2) * 23.4,
			"flange housing envelope volume");
		envelope.close();
		var preview = block.housing.geometry();
		solid(preview, "flange housing preview");
		check(preview.volume() < envelopeVolume, "preview also removes bolt holes");
		preview.close();

		near(block.housing.connector("bore").frame.z, 11.7, "flange housing bore connector");
		near(block.housing.connector("bolt1").frame.x, 28.2, "flange housing bolt connector x");

		var model = new AssemblyModel();
		block.addTo(model, "flange");
		var definition = model.definition("flange-bearing-assembly");
		check(definition.joints.length == 5, "flange assembly joint count");
		var state = model.initialState("flange-bearing-assembly");
		var depth = block.housing.depth;
		near(state.worldConnector("flange/bearing", "axis").z, depth / 2, "bearing centred in the housing depth");
		near(state.worldConnector("flange/bearing", "front").z, depth / 2 - bearing.width / 2, "bearing front inside the housing");
		check(block.connector("bearingAxis", "flange").instanceId == "flange/bearing",
			"flange assembly exposes its named bearing axis with the caller prefix");
		// Screws seat on the outer face and reach through the mounting face into the frame.
		near(block.screw.length, 35, "flange assembly screw: next standard length over depth + 1.5 d");
		check(block.screw.length >= depth + 1.5 * block.screw.diameter, "flange assembly screw engagement");
		for (i in 1...5) {
			var head = state.worldConnector('flange/screw$i', "head");
			var bolt = block.housing.connector('bolt$i').frame;
			near(head.x, bolt.x, 'flange assembly screw$i on its bolt x');
			near(head.y, bolt.y, 'flange assembly screw$i on its bolt y');
			near(head.z, depth, 'flange assembly screw$i head on the outer face');
			check(state.worldConnector('flange/screw$i', "tip").z < 0, 'flange assembly screw$i tip below the mounting face');
		}
		near(FlangeBearingAssembly.standardScrewLength(20), 20, "standard screw length exact");
		near(FlangeBearingAssembly.standardScrewLength(20.1), 25, "standard screw length rounds up");

		var lines = block.bom().lines();
		check(lines.length == 3, "flange assembly BOM line count");
		check(block.bom().quantity(block.screw.designation) == 4, "flange assembly screw quantity");
		for (entry in block.components()) {
			var part = entry.component.geometry(Envelope);
			check(part.valid(), '${entry.id} envelope is invalid');
			part.close();
		}
	}

	static function wheels():Void {
		var wheel = new DriveWheel(150, 40, 6.35, 40, 10);
		check(connectorNames(wheel) == "bore,centre", "a drive wheel has a bore and a centre");
		near(wheel.connector("centre").frame.z, 30, "the wheel centre is mid-tread");
		near(wheel.radius, 75, "drive wheel radius");
		var solid = wheel.geometry();
		check(solid.valid() && solid.solidCount() == 1, "a drive wheel is one solid");
		var envelope = wheel.geometry(Envelope);
		check(solid.volume() < envelope.volume(), "the bore and dish come out of the drive wheel");
		solid.close();
		envelope.close();
		check(DriveWheel.recipeType().create(wheel.values()).designation == wheel.designation,
			"a drive wheel rebuilds from its recipe values");
		throws(() -> new DriveWheel(150, 40, 40, 40, 10), "bore inside its hub");
		throws(() -> new DriveWheel(60, 40, 8, 40, 10), "hub inside its tread");

		var caster = new CasterWheel(75, 25, 110, 30, 60);
		check(connectorNames(caster) == "mount,swivel,floor", "a caster has a mount, a swivel and a floor contact");
		near(caster.connector("floor").frame.z, -110, "the caster touches the floor its height below the plate");
		near(caster.connector("floor").frame.x, -30, "the caster wheel trails the swivel");
		var body = caster.geometry();
		check(body.valid() && body.solidCount() == 1, "a caster is one solid");
		var extent = bounds(body);
		near(extent.minZ, -110, "the caster wheel reaches the floor", 1e-3);
		near(extent.maxZ, 0, "the caster plate tops out at its mount", 1e-3);
		body.close();
		check(CasterWheel.recipeType().create(caster.values()).designation == caster.designation,
			"a caster rebuilds from its recipe values");
		throws(() -> new CasterWheel(100, 25, 110, 30, 60), "room above its wheel");
	}

	static function pillowBlock():Void {
		var block = PillowBlock.metric("UCP204");
		check(block.designation == "UCP204", "UCP pillow block designation");
		check(block.bearing.designation == "6204-2Z", "UCP pillow block insert bearing");
		near(block.boreDiameter, 20, "UCP pillow block bore");
		near(block.baseWidth, 38, "UCP pillow block base width");
		near(block.length, 127, "UCP pillow block length");
		near(block.shaftHeight, 33.3, "UCP pillow block shaft height");
		near(block.baseHeight, 16, "UCP pillow block base height");
		near(block.overallHeight, 64.5, "UCP pillow block overall height");
		near(block.boltSpacing, 95, "UCP pillow block bolt spacing");
		near(block.mountHoleDiameter, 13, "UCP pillow block mounting hole diameter");
		check(block.mountScrew == "M10", "UCP pillow block mounting screw");
		var envelope = block.geometry(Envelope);
		solid(envelope, "UCP pillow block envelope");
		var envelopeBounds = envelope.shape.bounds();
		near(envelopeBounds.get_min().get_y(), 0, "UCP pillow block base bottom");
		near(envelopeBounds.get_max().get_y(), 64.5, "UCP pillow block top");
		near(envelopeBounds.get_min().get_x(), -63.5, "UCP pillow block length start");
		near(envelopeBounds.get_max().get_x(), 63.5, "UCP pillow block length end");
		near(envelopeBounds.get_min().get_z(), -19, "UCP pillow block width start");
		near(envelopeBounds.get_max().get_z(), 19, "UCP pillow block width end");
		var envelopeVolume = envelope.volume();
		envelope.close();
		var preview = block.geometry();
		solid(preview, "UCP pillow block preview");
		check(preview.volume() < envelopeVolume, "UCP preview removes mounting holes");
		preview.close();
		near(block.connector("axis").frame.y, 33.3, "UCP shaft axis connector height");
		near(block.connector("input").frame.z, -19, "UCP input connector");
		near(block.connector("output").frame.z, 19, "UCP output connector");
		near(block.connector("bolt1").frame.x, -47.5, "UCP first bolt connector");
		near(block.connector("bolt2").frame.x, 47.5, "UCP second bolt connector");
		var model = new AssemblyModel();
		block.addTo(model, "ucp");
		var state = model.initialState("ucp");
		near(state.worldConnector("ucp", "axis").y, 33.3, "UCP axis assembly height");
		near(state.worldConnector("ucp", "base").y, 0, "UCP base assembly face");
		var screw = block.mountScrewPart(20);
		var screwModel = new AssemblyModel();
		block.addTo(screwModel, "block");
		screw.addTo(screwModel, "mount-screw");
		screwModel.mate("mount-screw-seat", "fixed", "block", "bolt1", "mount-screw", "head");
		var screwState = screwModel.initialState("block");
		var screwAxis = AssemblyFrames.transformVector(screwState.worldConnector("mount-screw", "head"), 0, 1, 0);
		near(Math.abs(screwAxis.y), 1, "UCP mounting screw is vertical");
		near(screwAxis.x, 0, "UCP mounting screw has no X tilt");
		near(screwAxis.z, 0, "UCP mounting screw has no Z tilt");
		var bom = new Bom();
		bom.addComponent(block);
		check(bom.quantity("UCP204") == 1, "UCP pillow block BOM");
		check(block.mountScrewPart(20).designation == "ISO4762-M10x20", "UCP mounting screw companion");
		var mountingBom = block.billOfMaterials(true, 20);
		check(mountingBom.quantity("UCP204") == 1, "UCP mounting BOM unit");
		check(mountingBom.quantity("ISO4762-M10x20") == 2, "UCP mounting BOM screws");
		for (reference in [
			{name: "UCP205", bore: 25, length: 140, height: 36.5, boltSpacing: 105, screw: "M10"},
			{name: "UCP206", bore: 30, length: 165, height: 42.9, boltSpacing: 121, screw: "M14"},
			{name: "UCP207", bore: 35, length: 167, height: 47.6, boltSpacing: 127, screw: "M14"},
			{name: "UCP208", bore: 40, length: 184, height: 49.2, boltSpacing: 137, screw: "M14"},
			{name: "UCP209", bore: 45, length: 190, height: 54, boltSpacing: 146, screw: "M14"},
			{name: "UCP210", bore: 50, length: 206, height: 57.2, boltSpacing: 159, screw: "M16"},
			{name: "UCP211", bore: 55, length: 219, height: 63.5, boltSpacing: 171, screw: "M16"},
			{name: "UCP212", bore: 60, length: 241, height: 69.8, boltSpacing: 184, screw: "M16"},
			{name: "UCP213", bore: 65, length: 265, height: 76.2, boltSpacing: 203, screw: "M20"}]) {
			var candidate = PillowBlock.metric(reference.name);
			near(candidate.boreDiameter, reference.bore, reference.name + " bore");
			near(candidate.length, reference.length, reference.name + " length");
			near(candidate.shaftHeight, reference.height, reference.name + " shaft height");
			near(candidate.boltSpacing, reference.boltSpacing, reference.name + " bolt spacing");
			check(candidate.mountScrew == reference.screw, reference.name + " mounting screw");
			check(candidate.mountScrewPart(40).designation == 'ISO4762-${reference.screw}x40',
				reference.name + " mounting screw companion");
			var candidateBom = candidate.billOfMaterials(true, 40);
			check(candidateBom.quantity('ISO4762-${reference.screw}x40') == 2,
				reference.name + " mounting screw BOM quantity");
			var candidatePart = candidate.geometry(Envelope);
			solid(candidatePart, reference.name + " envelope");
			candidatePart.close();
		}
		throws(() -> PillowBlock.metric("UCP299"), 'Unknown pillow block unit "UCP299"');
	}

	static function linearAxis():Void {
		var axis = new LinearAxis();
		MachineAssemblyDescriptionTests.roundTrip(axis, "linear axis");
		check(axis.motor.designation == "23HS22-2804S", "linear axis motor designation");
		check(axis.bearing.designation == "6000-2Z", "linear axis default bearing matches the 10 mm screw");
		check(axis.coupling.designation == "COUPLING-6.35x10-18x30", "linear axis coupling joins motor and screw");
		near(axis.carriage.boreDiameter, 10, "linear axis carriage bore");
		check(axis.nut.lead == 2, "linear axis lead nut");
		check(axis.nut.thread.designation == "TR-D10-P2-S1-RH", "axis default thread is explicit");
		check(axis.screw.thread.designation == axis.nut.thread.designation, "axis screw and nut share thread specification");
		check(axis.guideBearingA.designation == "LM8UU", "linear axis round guide bearing");
		check(axis.guideSystem.bearingDesignation == "LM8UU", "linear axis guide catalog row");
		check(axis.guideSystem.rodFit == BearingShaftFit.Slip, "linear axis guide rod fit");
		check(axis.guideSystem.housingFit == BearingHousingFit.Slip, "linear axis guide seat fit");
		check(axis.guideSpacing >= axis.flangeBearingA.housing.face / 2 + axis.guideBearingA.boreDiameter / 2 + LinearAxis.RAIL_GAP,
			"linear axis guide rods clear the flange housing width");
		near(axis.guideSystem.rodDiameter, 7.99, "linear axis guide rod fit diameter");
		near(axis.guideSystem.seatDiameter, 15.0375, "linear axis guide seat fit diameter");
		near(axis.carriage.guideSeatDiameter, axis.guideSystem.seatDiameter, "carriage uses guide seat fit");
		near(axis.guideSystem.rodLength, axis.length, "guide rods span the axis");
		check(axis.guideRodA == axis.guideSystem.rodA && axis.guideBearingA == axis.guideSystem.bearingA,
			"axis exposes guide system components");
		var carriagePreview = axis.carriage.geometry();
		solid(carriagePreview, "carriage with nut and guide seats");
		check(carriagePreview.volume() < axis.carriage.width * axis.carriage.width * axis.carriage.length,
			"carriage preview cuts mounting and guide holes");
		carriagePreview.close();
		// Layout from the screw input: coupling half (15) + gap (2) + housing (depth), margin (20),
		// carriage (60) + stroke (200), margin (20), housing (depth) flush with the screw end.
		var depth = axis.flangeBearingA.housing.depth;
		near(depth, 14, "6000 housing depth");
		near(axis.bearingAPosition, 15 + 2 + depth / 2, "flange bearing A just past the coupling");
		near(axis.travelMin, 15 + 2 + depth + 30 + 30, "carriage travel starts a margin past flange bearing A");
		near(axis.travelMax - axis.travelMin, 200, "carriage travel equals the stroke");
		near(axis.bearingBPosition, axis.travelMax + 30 + 30 + depth / 2, "flange bearing B a margin past the travel end");
		near(axis.length, 365, "linear axis screw length");
		near(axis.screw.totalLength, axis.length, "linear axis screw total length");
		throws(() -> new LinearAxis(23, 10, -1), "positive stroke");
		throws(() -> new LinearAxis(23, 10, 200, "6001"), "bore does not match the screw diameter");
		throws(() -> new LinearAxis(23, 66), "No catalog deep groove bearing has a 66 mm bore");
		throws(() -> new LinearAxis(23, 10, 200, null, 2), "must clear the flange housing screw heads and lead nut");
		throws(() -> new LinearAxis(23, 8), "explicit thread for a nondefault screw diameter");
		throws(() -> new LinearAxis(23, 8, 200, null, 30, new LeadScrewThread(MetricTrapezoidal, 10, 2)),
			"thread diameter must match the screw diameter");

		for (entry in axis.components()) {
			var part = entry.component.geometry(Envelope);
			check(part.valid(), '${entry.id} envelope is invalid');
			part.close();
		}
		var rail = axis.frame.geometry("rail");
		solid(rail, "linear axis rail");
		var railBox = rail.shape.bounds();
		near(railBox.get_min().get_z(), 21, "rail starts at the screw input");
		near(railBox.get_max().get_z(), 21 + axis.length, "rail ends at the screw end");
		// Beside the screw, clear of the housings (and their screw heads) and the carriage.
		var housing = axis.flangeBearingA.housing;
		check(railBox.get_max().get_y() <= -housing.face / 2 - 1, "rail clears the flange housings");
		check(railBox.get_max().get_y() <= -(housing.boltSpacing + axis.flangeBearingA.screw.spec.headDiameter) / 2 - 1,
			"rail clears the flange housing screw heads");
		check(railBox.get_max().get_y() <= -axis.carriage.width / 2 - 1, "rail clears the carriage");
		rail.close();
		var cutList = axis.frame.cutList();
		check(cutList.length == 1, "linear axis frame cut list");
		near(cutList[0].totalLength, axis.length, "linear axis rail length");

		var model = axis.assembly();
		var definition = model.definition("linear-axis");
		check(definition.joints.length == 16, "linear axis joint count");
		for (joint in definition.joints) if (joint.id == "carriage-slide") {
			check(joint.limits.lower != null && joint.limits.lower == axis.travelMin, "carriage lower limit");
			check(joint.limits.upper != null && joint.limits.upper == axis.travelMax, "carriage upper limit");
		}
		var state = model.initialState("linear-axis");
		near(state.worldConnector("coupling", "axis").z, 21, "coupling centred on the motor shaft tip");
		near(state.worldConnector("screw", "input").z, 21, "screw seats on the motor shaft");
		near(state.worldConnector("carriage", "bore").z, 21 + axis.travelMin, "carriage starts at its lower travel limit");
		near(state.worldConnector("guideBearingA", "axis").z, state.worldConnector("carriage", "bore").z,
			"first guide bearing is aligned at the lower stroke end");
		near(state.worldConnector("guideBearingB", "axis").z, state.worldConnector("carriage", "bore").z,
			"second guide bearing is aligned at the lower stroke end");
		near(state.worldConnector("guideRodA", "input").x, -axis.guideSpacing, "first guide rod offset");
		near(state.worldConnector("guideBearingA", "axis").x, -axis.guideSpacing, "first bearing follows guide rod");
		near(state.worldConnector("leadNut", "mountFace").z, state.worldConnector("carriage", "nutMount").z,
			"lead nut mounts to the carriage face");
		near(state.worldConnector("flangeA-bearing", "axis").z, 21 + axis.bearingAPosition, "flange bearing A on the screw");
		near(state.worldConnector("flangeB-bearing", "axis").z, 21 + axis.bearingBPosition, "flange bearing B on the screw");
		near(state.worldConnector("flangeA-bearing", "axis").z, 45, "flange bearing A position");
		near(state.worldConnector("flangeB-bearing", "axis").z, 379, "flange bearing B position");
		// Housing A mounts toward the motor, housing B is turned over to mount toward the far end:
		// both screw heads face the carriage and their tips point outboard.
		near(state.worldConnector("flangeA-screw1", "head").z, 21 + axis.bearingAPosition + depth / 2, "flange A screw heads inboard");
		check(state.worldConnector("flangeA-screw1", "tip").z < 21 + axis.bearingAPosition - depth / 2, "flange A screw tips outboard");
		near(state.worldConnector("flangeB-screw1", "head").z, 21 + axis.bearingBPosition - depth / 2, "flange B screw heads inboard");
		check(state.worldConnector("flangeB-screw1", "tip").z > 21 + axis.length, "flange B screw tips outboard");
		var prefixedModel = new AssemblyModel();
		axis.addTo(prefixedModel, "axis-x");
		var prefixedState = prefixedModel.initialState("prefixed-linear-axis");
		check(axis.connector("carriageBore", "axis-x").instanceId == "axis-x/carriage",
			"linear axis exposes a prefix-aware carriage connector");
		check(Lambda.exists(prefixedModel.definition("prefixed-linear-axis").joints,
			joint -> joint.id == "axis-x/carriage-slide"),
			"linear axis prefixes its joints");
		axis.setTravel(prefixedState, 5, "axis-x");
		near(prefixedState.worldConnector("axis-x/carriage", "bore").z, 21 + axis.travelMin + 5,
			"linear axis setTravel accepts the same instance prefix");

		axis.setTravel(state, 100);
		near(state.joint("coupling"), axis.nut.rotationFor(100), "screw rotation follows nut lead");
		near(state.worldConnector("carriage", "bore").z, 21 + axis.travelMin + 100, "carriage travels with screw rotation");
		near(state.worldConnector("guideBearingB", "axis").x, axis.guideSpacing, "bearing stays on second guide");
		var housingPart = axis.flangeBearingA.housing.geometry(Envelope);
		var guideRodPart = placedAt(axis.guideRodA.geometry(Envelope), state.worldPose("guideRodA"));
		checkOverlap(housingPart, guideRodPart, 0, "guide rod clears the flange housing");
		axis.setTravel(state, 0);
		var unturned = state.worldPose("carriage");
		axis.setTravel(state, 2);
		near(state.joint("coupling"), -2 * Math.PI,
			"right-hand screw turns negative about +Z to advance the nut along +Z");
		near(state.worldPose("carriage").qz, unturned.qz, "carriage does not rotate with screw");
		throws(() -> axis.setTravel(state, axis.stroke + 1), "outside its stroke");
		var multiAxis = new LinearAxis(23, 10, 200, null, 30,
			new LeadScrewThread(MetricTrapezoidal, 10, 2, 4));
		var multiState = multiAxis.assembly().initialState("linear-axis");
		multiAxis.setTravel(multiState, 8);
		near(multiState.joint("coupling"), -2 * Math.PI,
			"right-hand multi-start axis rotates negative to move 8 mm along +Z");
		near(multiState.worldConnector("carriage", "bore").z, 21 + multiAxis.travelMin + 8,
			"multi-start carriage travel");
		var leftAxis = new LinearAxis(23, 10, 200, null, 30,
			new LeadScrewThread(MetricTrapezoidal, 10, 2, 4, LeftHand));
		var leftState = leftAxis.assembly().initialState("linear-axis");
		leftAxis.setTravel(leftState, 8);
		near(leftState.joint("coupling"), 2 * Math.PI, "left-hand axis rotates positive about +Z to advance along +Z");
		state.setJoint("carriage-slide", axis.travelMax);
		near(state.joint("coupling"), axis.nut.rotationFor(axis.stroke),
			"assembly coupling drives screw rotation from carriage travel");
		throws(() -> state.setJoint("coupling", 0), "driven by coupled joint");
		state.forwardKinematics();
		near(state.worldConnector("guideBearingA", "axis").z, state.worldConnector("carriage", "bore").z,
			"first guide bearing is aligned at the upper stroke end");
		near(state.worldConnector("guideBearingB", "axis").z, state.worldConnector("carriage", "bore").z,
			"second guide bearing is aligned at the upper stroke end");
		var carriageEnd = state.worldConnector("carriage", "bore").z + axis.carriage.length / 2;
		near(state.worldConnector("flangeB-bearing", "axis").z - depth / 2 - carriageEnd, 30,
			"carriage stops a margin short of flange bearing B");

		var bom = axis.bom();
		var lines = bom.lines();
		check(lines.length == 11, "linear axis BOM line count");
		check(bom.quantity(axis.bearing.designation) == 2, "linear axis bearing quantity");
		check(bom.quantity(axis.coupling.designation) == 1, "linear axis coupling in the BOM");
		check(bom.quantity(axis.nut.bom.partNumber) == 1, "linear axis lead nut in the BOM");
		check(bom.quantity(axis.screw.bom.partNumber) == 1, "thread-specific lead screw in the BOM");
		check(bom.quantity(axis.guideRodA.designation) == 2, "linear axis guide rods in the BOM");
		check(bom.quantity(axis.guideBearingA.designation) == 2, "linear axis guide bearings in the BOM");
		check(bom.quantity("RECT-20x15x2-L365") == 1, "linear axis rail in the BOM");
		check(bom.quantity(axis.flangeBearingA.screw.designation) == 8, "linear axis flange assembly screws");

		var railAxis = LinearAxis.forRailProfile("MGN12C");
		var railGuide:machinekit.motion.LinearRailSystem = cast railAxis.railGuide;
		check(railGuide != null, "profile rail axis selects a rail guide");
		near(railGuide.railLength, 254.7, "profile rail axis cut length");
		near(railGuide.stroke, railAxis.stroke, "profile rail axis stroke");
		near(railAxis.railGuideOffset, railAxis.travelMin - railGuide.travelMin,
			"profile rail axis guide offset");
		check(railAxis.carriage.hasRailMount, "profile rail axis carriage mount");
		var railCarriagePart = railAxis.carriage.geometry();
		solid(railCarriagePart, "profile rail axis carriage");
		railCarriagePart.close();
		var railModel = railAxis.assembly();
		var railDefinition = railModel.definition("linear-rail-axis");
		check(railDefinition.joints.length == 16, "profile rail axis joint count");
		var railState = railModel.initialState("linear-rail-axis");
		near(railState.worldConnector("profileBlock1", "rail").z, 21 + railAxis.travelMin,
			"profile block follows carriage at lower travel");
		near(railState.worldConnector("profileRail", "axis").z + railGuide.travelMin,
			21 + railAxis.travelMin, "profile rail lower limit aligns with axis");
		check(railState.closureResiduals().length == 1, "profile rail closure recorded");
		near(railState.closureResiduals()[0].position, 0, "profile rail closure has no transverse error");
		var frameRail = railAxis.frame.geometry("rail");
		var profileRail = placedAt(railGuide.rail.geometry(Envelope), railState.worldPose("profileRail"));
		checkOverlap(frameRail, profileRail, 0, "frame rail clears the profile rail envelope");
		var frameRail2 = railAxis.frame.geometry("rail");
		var profileBlock = placedAt(railGuide.blocks[0].geometry(Envelope), railState.worldPose("profileBlock1"));
		checkOverlap(frameRail2, profileBlock, 0, "frame rail clears the profile block envelope");
		railAxis.setTravel(railState, railAxis.stroke);
		near(railState.worldConnector("profileBlock1", "rail").z, 21 + railAxis.travelMax,
			"profile block follows carriage at upper travel");
		near(railState.worldConnector("profileRail", "axis").z + railGuide.travelMax,
			21 + railAxis.travelMax, "profile rail upper limit aligns with axis");
		var railBom = railAxis.bom();
		check(railBom.quantity(railGuide.rail.bom.partNumber) == 1, "profile rail axis rail in BOM");
		check(railBom.quantity(railGuide.blocks[0].bom.partNumber) == 1, "profile rail axis block in BOM");
		throws(() -> LinearAxis.forRailProfile("MGN99C"), 'Unknown linear rail profile "MGN99C"');
	}

	static function pickingFrames():Void {
		var config = PickingStationConfig.defaults();
		var rack = new StorageRack(config), frame = rack.frame;
		MachineAssemblyDescriptionTests.roundTrip(rack, "storage rack", false);
		var clearWidth = config.rackWidth - 2 * frame.profile.size;
		var clearDepth = config.rackDepth - 2 * frame.profile.size;
		for (cut in frame.memberCuts()) {
			if (cut.name.indexOf("front-rail-") == 0 || cut.name.indexOf("back-rail-") == 0)
				near(cut.length, clearWidth, cut.name + " is trimmed to the post faces");
			else if (cut.name.indexOf("left-rail-") == 0 || cut.name.indexOf("right-rail-") == 0)
				near(cut.length, clearDepth, cut.name + " is trimmed to the post faces");
		}
		var feet = 0;
		for (entry in rack.instances()) if (entry.id.indexOf("rack-01/feet/") == 0) {
			feet++;
			near(Math.abs(entry.pose.x), config.rackWidth / 2 - frame.profile.size / 2,
				"adjustable foot aligns with its post in X");
			near(Math.abs(entry.pose.y), config.rackDepth / 2 - frame.profile.size / 2,
				"adjustable foot aligns with its post in Y");
		}
		check(feet == 4, "rack has one foot aligned to each post");
		check(rack.instances("rack-02")[0].id == "rack-02/frame",
			"storage rack instance listing accepts a caller prefix");
		var secondRackPosition = rack.positions("rack-02")[0];
		check(secondRackPosition.id == "rack-02/shelf-01/bin-01" &&
			secondRackPosition.indicatorId == "rack-02/shelf-01/bin-01/indicator",
			"storage rack positions and indicators accept the caller prefix");
		var rackModel = new AssemblyModel();
		rack.addTo(rackModel, "rack-02");
		var rackOccurrences = rackModel.definition("storage-rack").occurrences;
		check(Lambda.exists(rackOccurrences, occurrence -> occurrence.id == "rack-02/frame") &&
			Lambda.exists(rackOccurrences, occurrence -> occurrence.id == "rack-02/shelf-01/bin-01/indicator"),
			"storage rack adds all occurrences below the caller prefix");
		check(rack.billOfMaterials().quantity("ISO4762-M5x12") == config.shelfCount * 4,
			"storage rack BOM includes shelf fasteners");
		checkOverlap(frame.memberGeometry("front-rail-0"), frame.memberGeometry("front-left"), 0,
			"rack front rail terminates at the post face");
		checkOverlap(frame.memberGeometry("front-rail-0"), frame.memberGeometry("front-right"), 0,
			"rack front rail clears the opposite post");
		checkOverlap(frame.memberGeometry("back-rail-0"), frame.memberGeometry("back-right"), 0,
			"rack back rail terminates at the post face");
		checkOverlap(frame.memberGeometry("left-rail-0"), frame.memberGeometry("front-left"), 0,
			"rack side rail terminates at the post face");
	}

	static function linearRailGuide():Void {
		var guide = LinearGuideSystem.forRailProfile("MGN12C", 300);
		check(guide.spec.family == "HIWIN", "profile rail family");
		check(guide.spec.designation == "MGN12C", "profile rail designation");
		check(guide.blockCount == 1, "profile rail default block count");
		near(guide.spec.railWidth, 12, "MGN12 rail width");
		near(guide.spec.railHeight, 8, "MGN12 rail height");
		near(guide.spec.blockWidth, 27, "MGN12 block width");
		near(guide.spec.blockLength, 34.7, "MGN12 block length");
		near(guide.spec.blockHolePitchB, 20, "MGN12 block hole pitch B");
		near(guide.spec.blockHolePitchC, 15, "MGN12 block hole pitch C");
		near(guide.travelMin, 27.35, "profile rail lower travel");
		near(guide.travelMax, 272.65, "profile rail upper travel");
		near(guide.stroke, 245.3, "profile rail stroke");
		check(guide.rail.holePositions.length == 12, "profile rail mounting-hole count");
		near(guide.rail.holePositions[0], 10, "profile rail first mounting hole");
		near(guide.rail.holePositions[guide.rail.holePositions.length - 1], 285, "profile rail last mounting hole");
		near(guide.rail.connector("mount1").frame.z, 10, "profile rail mount connector");
		near(guide.blocks[0].connector("mount1").frame.z, -10, "profile block first mount connector");

		var railPart = guide.rail.geometry(Envelope);
		solid(railPart, "profile rail envelope");
		var railBounds = bounds(railPart);
		var railBottom = railPart.shape.bounds().get_min().get_y();
		near(railBounds.minX, -6, "profile rail minimum x");
		near(railBounds.maxX, 6, "profile rail maximum x");
		near(railBounds.minZ, 0, "profile rail start");
		near(railBounds.maxZ, 300, "profile rail end");
		railPart.close();
		var blockPart = guide.blocks[0].geometry(Preview);
		solid(blockPart, "profile block envelope");
		var blockBounds = bounds(blockPart);
		var blockBounds3D = blockPart.shape.bounds();
		near(blockBounds.minX, -13.5, "profile block minimum x");
		near(blockBounds.maxX, 13.5, "profile block maximum x");
		near(blockBounds.minZ, -17.35, "profile block minimum z");
		near(blockBounds.maxZ, 17.35, "profile block maximum z");
		near(blockBounds3D.get_min().get_y(), 0, "profile block sits on the rail top");
		near(blockBounds3D.get_max().get_y(), 5, "profile block top above rail top");
		near(blockBounds3D.get_max().get_y() - railBottom, 13,
			"profile block height measured from rail bottom");
		blockPart.close();
		check(guide.blocks[0].connectors().length == 6, "profile block has four mount connectors and two axes");
		near(guide.blocks[0].connector("mount1").frame.x, -7.5, "profile block first mount X");
		near(guide.blocks[0].connector("mount1").frame.z, -10, "profile block first mount Z");
		near(guide.blocks[0].connector("mount2").frame.x, -7.5, "profile block second mount X");
		near(guide.blocks[0].connector("mount2").frame.z, 10, "profile block second mount Z");
		near(guide.blocks[0].connector("mount3").frame.x, 7.5, "profile block third mount X");
		near(guide.blocks[0].connector("mount3").frame.z, -10, "profile block third mount Z");
		near(guide.blocks[0].connector("mount4").frame.x, 7.5, "profile block fourth mount X");
		near(guide.blocks[0].connector("mount4").frame.z, 10, "profile block fourth mount Z");

		var model = guide.assembly();
		var definition = model.definition("linear-rail");
		check(definition.joints.length == 1, "profile rail joint count");
		var state = model.initialState("linear-rail");
		near(state.worldConnector("block1", "rail").z, guide.travelMin, "profile block starts at lower travel");
		near(state.worldConnector("block1", "mount1").z, guide.travelMin - guide.spec.blockHolePitchB / 2,
			"profile block mounting pattern at lower travel");
		guide.setTravel(state, guide.travelMax);
		near(state.worldConnector("block1", "rail").z, guide.travelMax, "profile block reaches upper travel");
		guide.setTravel(state, guide.travelMin);
		throws(() -> guide.setTravel(state, guide.travelMax + 1), "outside its limits");
		throws(() -> LinearGuideSystem.forRailProfile("MGN12C", 50), "leave room for a block");
		throws(() -> LinearGuideSystem.forRailProfile("MGN12C", 300, 20), "too short");
		throws(() -> LinearGuideSystem.forRailProfile("MGN12C", 300, 0), "at least one block");
		throws(() -> LinearGuideSystem.forRailProfile("MGN99C", 300), 'Unknown linear rail profile "MGN99C"');

		var bom = guide.bom();
		check(bom.quantity(guide.rail.bom.partNumber) == 1, "profile rail in BOM");
		check(bom.quantity(guide.blocks[0].bom.partNumber) == 1, "profile block in BOM");
		check(guide.components().length == 2, "profile rail component list");
	}

	static function catalogExtras():Void {
		var bushing = new Bushing(8);
		check(bushing.designation == "BUSHING-8x11x12", "bushing designation");
		check(Bushing.recipeType().create(bushing.values()).designation == bushing.designation,
			"bushing designation survives a recipe round trip");
		throws(() -> new Bushing(-1), "positive bore diameter");
		var bushingPart = bushing.geometry();
		solid(bushingPart, "bushing");
		near(bushingPart.volume(), Math.PI * (5.5 * 5.5 - 4 * 4) * 12, "bushing volume");
		bushingPart.close();
		near(bushing.connector("axis").frame.z, 6, "bushing axis connector");

		var coupling = new ShaftCoupling(5, 8);
		check(coupling.designation == "COUPLING-5x8-14.4x24", "shaft coupling designation");
		check(coupling.setScrew == "M3", "shaft coupling set screw size");
		check(coupling.setScrews.length == 2, "shaft coupling default set screws");
		near(coupling.setScrewHoleDiameter, 2.5, "shaft coupling tap drill");
		throws(() -> new ShaftCoupling(-1, 8), "positive bore diameters");
		var couplingEnvelope = coupling.geometry(Envelope);
		var couplingPart = coupling.geometry();
		solid(couplingEnvelope, "shaft coupling envelope");
		solid(couplingPart, "shaft coupling");
		check(couplingPart.volume() < Math.PI * 7.2 * 7.2 * 24, "shaft coupling removes both bores");
		check(couplingPart.volume() < couplingEnvelope.volume(), "shaft coupling set screw holes");
		check(coupling.connector("setScrew1").role == Mount, "shaft coupling set screw connector");
		near(AssemblyFrames.transformVector(coupling.connector("setScrew1").frame, 0, 1, 0).x, 1,
			"shaft coupling set screw radial axis");
		couplingPart.close();
		couplingEnvelope.close();
		near(coupling.connector("sideB").frame.z, 24, "shaft coupling sideB connector");
		var couplingBom = coupling.billOfMaterials(8);
		check(couplingBom.quantity(coupling.designation) == 1, "shaft coupling BOM body");
		check(couplingBom.quantity(coupling.setScrewPart(8).designation) == 2, "shaft coupling set screw BOM");
		var customCoupling = new ShaftCoupling(5, 8, null, null,
			[{z: 6, angle: 0}, {z: 18, angle: Math.PI / 2}]);
		check(customCoupling.setScrews.length == 2, "custom shaft coupling set screws");
		var customPart = customCoupling.geometry();
		solid(customPart, "custom shaft coupling holes");
		customPart.close();
		throws(() -> new ShaftCoupling(5, 8, null, null, [{z: 4, angle: 0}]), "clear both ends by 1.5 screw diameters");
		throws(() -> new ShaftCoupling(5, 8, null, null, [{z: 24, angle: 0}]), "clear both ends by 1.5 screw diameters");
		throws(() -> new ShaftCoupling(5, 8, null, null, [{z: 6, angle: 0}, {z: 6, angle: 0}]), "must be unique");

		var linearBearing = LinearBearing.metric("LM8UU");
		check(linearBearing.designation == "LM8UU", "linear bearing designation");
		near(linearBearing.guideRodDiameter(BearingShaftFit.Slip), 7.99, "linear bearing shaft fit diameter");
		near(linearBearing.housingSeatDiameter(BearingHousingFit.Slip), 15.0375, "linear bearing housing fit diameter");
		var linearSeat = linearBearing.housingSeat(10, BearingHousingFit.Interference);
		near(linearSeat.volume(), Math.PI * Math.pow((15 - 0.015) / 2, 2) * 10, "linear bearing housing seat tool");
		linearSeat.close();
		throws(() -> LinearBearing.metric("LM9UU"), 'Unknown linear bearing "LM9UU"');
		var linearEnvelope = linearBearing.geometry(Envelope);
		solid(linearEnvelope, "linear bearing envelope");
		near(linearEnvelope.volume(), Math.PI * (7.5 * 7.5 - 4 * 4) * 24, "linear bearing envelope volume");
		var linearBearingPart = linearBearing.geometry();
		solid(linearBearingPart, "linear bearing preview");
		check(linearBearingPart.volume() < linearEnvelope.volume(), "linear bearing preview seal tracks");
		near(bounds(linearBearingPart).maxX, bounds(linearEnvelope).maxX,
			"linear bearing retaining rims stay inside the outside diameter");
		linearBearingPart.close();
		linearEnvelope.close();

		var sprocket = new Sprocket(12.7, 20, 8, 6);
		check(sprocket.designation == "GENERIC-SPROCKET-P12.7-20T", "sprocket designation");
		near(sprocket.pitchDiameter, 12.7 / Math.sin(Math.PI / 20), "sprocket pitch diameter");
		near(sprocket.rollerDiameter, 0.625 * 12.7, "sprocket default roller diameter");
		near(sprocket.rootDiameter, sprocket.pitchDiameter - 0.625 * 12.7, "sprocket root = pitch - roller diameter");
		near(sprocket.outsideDiameter, 12.7 * (0.6 + Math.cos(Math.PI / 20) / Math.sin(Math.PI / 20)),
			"sprocket outside diameter p(0.6 + cot(pi/z))");
		near(new Sprocket(12.7, 20, 8, 6, 7.92).rootDiameter, sprocket.pitchDiameter - 7.92, "sprocket roller override");
		var ansi40 = Sprocket.forChain("ANSI40", 20, 8, 6);
		check(ansi40.designation == "SPROCKET-ANSI40-20T", "chain family designation");
		near(ansi40.rollerDiameter, 7.92, "ANSI40 roller from catalog");
		near(Sprocket.forChain("ANSI35", 20, 6, 6).pitch, 9.525, "ANSI35 pitch from catalog");
		throws(() -> Sprocket.forChain("ANSI45", 20, 8, 6), "Unknown roller chain");
		throws(() -> new Sprocket(12.7, 20, 8, 6, 7.8, "ANSI40"), "must match its catalog entry");
		throws(() -> new Sprocket(12.7, 20, 8, 6, 13), "roller diameter must be positive and less than the pitch");
		throws(() -> new Sprocket(12.7, 5, 8, 6), "at least 8 teeth");
		throws(() -> new Sprocket(12.7, 8, 40, 6), "tooth-root land");
		throws(() -> Sprocket.forChain("ANSI25", 8, 13, 5), "tooth-root land");
		var sprocketPart = sprocket.geometry();
		solid(sprocketPart, "sprocket");
		var sprocketBox = bounds(sprocketPart);
		check(sprocketBox.maxX <= sprocket.outsideDiameter / 2 + 1e-6, "sprocket stays within its outside radius");
		check(sprocketBox.maxX > sprocket.pitchDiameter / 2, "sprocket teeth extend past the pitch circle");
		sprocketPart.close();

		var pulley = new TimingPulley(GT2, 20, 5, 6);
		check(pulley.designation == "PULLEY-GT2-20T", "timing pulley designation");
		near(pulley.pitchDiameter, 2 * 20 / Math.PI, "timing pulley pitch diameter");
		near(pulley.pitchLineDifferential, 0.254, "GT2 pitch line differential");
		near(pulley.outsideDiameter, 2 * 20 / Math.PI - 2 * 0.254, "timing pulley OD = PD - 2 PLD");
		near(new TimingPulley(HTD3M, 20, 5, 6).pitchLineDifferential, 0.381, "3 mm pitch line differential");
		near(new TimingPulley(HTD5M, 20, 5, 6).pitchLineDifferential, 0.5715, "5 mm pitch line differential");
		check(new TimingPulley(T5, 20, 5, 6).designation == "PULLEY-T5-20T", "T5 family has its own designation");
		near(new TimingPulley(T5, 20, 5, 6).pitchLineDifferential, 0.5, "T5 PLD differs from HTD5M");
		near(new TimingPulley(XL, 20, 5, 6).outsideDiameter, 5.08 * 20 / Math.PI - 0.508, "explicit PLD");
		check(new TimingPulley(Custom("CUSTOM", 2.032, 0.254), 20, 5, 6).designation == "PULLEY-CUSTOM-CUSTOM-P2.032-PLD0.254-20T", "fractional pulley designation");
		throws(() -> new TimingPulley(Custom("CUSTOM2032", 2.032, 0.254), 20, 5, 6), "must be \"CUSTOM\"");
		throws(() -> new TimingPulley(GT2, 5, 5, 6), "at least 8 teeth");
		throws(() -> new TimingPulley(GT2, 8, 11, 6), "must clear the bore");
		var pulleyPart = pulley.geometry();
		solid(pulleyPart, "timing pulley");
		var pulleyBox = bounds(pulleyPart);
		check(pulleyBox.maxX <= pulley.outsideDiameter / 2 + 1e-6, "timing pulley stays within its outside radius");
		check(pulleyBox.maxX > pulley.grooveDiameter / 2, "timing pulley lands extend past the groove circle");
		pulleyPart.close();

		// A belt round two 20-tooth GT2 pulleys: two centre distances plus one pulley circumference.
		var belt = TimingBelt.twoPulley(GT2, 20, 20, 100, 6);
		near(belt.length, 2 * 100 + 40, "two-pulley belt length");
		check(belt.teeth == 120, "two-pulley belt tooth count");
		near(belt.centreAdjustment(), 0, "a whole-tooth belt needs no centre adjustment");
		var longer = TimingBelt.twoPulley(GT2, 20, 20, 100.5, 6);
		check(longer.teeth == 121, "a belt 1 mm longer rounds to the next tooth");
		near(longer.slack(), 1, "its slack is the tooth length less the path");
		near(longer.centreAdjustment(), 0.5, "moving one pulley by half the slack makes it whole teeth", 1e-9);
		var strand = belt.strands()[0];
		near(strand.startY, -(2 * 20 / (2 * Math.PI)), "the first strand runs below the pulleys");
		near(belt.pointAt(belt.length + 10).x, belt.pointAt(10).x, "the path wraps round at its length");
		near(belt.pointAt(belt.strands()[0].length + 10).x, 100 + 20 / Math.PI, "the path follows the far pulley", 1e-9);
		check(belt.rotation(0, 0, 1, 0) == 1 && belt.rotation(0, 0, -1, 0) == -1, "a pulley turns with the strand it carries");
		var triangle = new TimingBelt(GT2, 6, [new BeltWrap(0, 0, 10), new BeltWrap(100, 0, 10), new BeltWrap(50, 60, 10)]);
		near(triangle.length, 100 + 2 * Math.sqrt(50 * 50 + 60 * 60) + 2 * Math.PI * 10, "a belt round three equal pulleys is their triangle plus one circle", 1e-6);
		var beltPart = belt.geometry();
		solid(beltPart, "timing belt");
		var beltBox = bounds(beltPart);
		near(beltBox.maxX, 100 + 2 * 20 / (2 * Math.PI) + 1.38 / 2, "timing belt band reaches past the pitch line by half its thickness", 0.01);
		beltPart.close();
		check(MachineKitComponents.defaultRegistry().byId("machinekit.transmission.timing-belt").create(
			MachineKitComponents.defaultRegistry().byId("machinekit.transmission.timing-belt").defaults()) != null, "timing belt recipe builds");
		check(belt.componentType() != null && belt.componentType().create(belt.values()).designation == belt.designation, "belt rebuilds from its recipe");

		var thread = new LeadScrewThread(MetricTrapezoidal, 8, 2);
		var nut = new LeadScrewNut(thread);
		check(nut.designation == "LEADNUT-TR-D8-P2-S1-RH", "lead screw nut designation");
		check(nut.thread.pitch == 2 && nut.thread.starts == 1, "nut carries pitch and starts");
		check(nut.mountScrew == "M3", "lead screw nut mount screw size");
		near(nut.travelPerRevolution(), -2, "right-hand nut moves toward -Z per positive screw revolution");
		near(nut.rotationFor(10), -10 / 2 * 2 * Math.PI, "right-hand nut rotation for travel toward +Z");
		throws(() -> new LeadScrewThread(MetricTrapezoidal, -1, 2), "positive screw diameter");
		throws(() -> new LeadScrewThread(MetricTrapezoidal, 8, -1), "positive pitch");
		throws(() -> new LeadScrewThread(MetricTrapezoidal, 8, 2, 0), "at least one start");
		throws(() -> new LeadScrewNut(thread, 2), "at least 3 mounting bolts");
		throws(() -> new LeadScrewNut(thread, 100), "leaves too little material between mounting holes");
		var multi = new LeadScrewNut(new LeadScrewThread(MetricTrapezoidal, 8, 2, 4));
		check(multi.designation == "LEADNUT-TR-D8-P2-S4-RH", "multi-start nut designation");
		near(multi.lead, 8, "four-start lead is four times pitch");
		near(multi.travelPerRevolution(), -8, "four-start right-hand travel per positive revolution");
		var left = new LeadScrewNut(new LeadScrewThread(MetricTrapezoidal, 8, 2, 4, LeftHand));
		near(left.travelPerRevolution(), 8, "left-hand nut moves toward +Z per positive screw revolution");

		var nutEnvelope = nut.geometry(Envelope);
		solid(nutEnvelope, "lead screw nut envelope");
		near(nutEnvelope.volume(), Math.PI * 5.2 * 5.2 * 16 + Math.PI * 12 * 12 * 3 - Math.PI * 4 * 4 * 19,
			"lead screw nut envelope volume");
		nutEnvelope.close();
		var nutPreview = nut.geometry();
		solid(nutPreview, "lead screw nut preview");
		nutPreview.close();
		near(nut.connector("bore").frame.z, 8, "lead screw nut bore connector");
		near(nut.connector("mount1").frame.z, 19, "lead screw nut mount connector z");
		near(nut.connector("mount1").frame.x, 7.9, "lead screw nut mount connector x");
		for (size in [6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0]) {
			var sized = new LeadScrewNut(new LeadScrewThread(MetricTrapezoidal, size, 2));
			var screw = sized.mountScrewPart(10).spec, r = sized.boltCircleDiameter / 2;
			check(r - screw.clearanceMedium / 2 >= sized.bodyDiameter / 2 + 1 - 1e-9,
				'lead screw nut D$size mount holes clear the body');
			check(r + screw.headDiameter / 2 <= sized.flangeDiameter / 2 - 1 + 1e-9,
				'lead screw nut D$size mount screw heads stay on the flange');
		}
		throws(() -> new LeadScrewNut(new LeadScrewThread(MetricTrapezoidal, 10, 2), 100),
			"too little material between mounting holes");
		check(new LeadScrewNut(new LeadScrewThread(Acme, 6.35, 3.175)).designation == "LEADNUT-ACME-D6.35-P3.175-S1-RH", "fractional nut designation");
	}

	/** `part` (closed) moved to the assembly pose `frame`. */
	static function placedAt(part:Part, frame:AssemblyFrame):Part {
		var x = AssemblyFrames.transformVector(frame, 1, 0, 0), z = AssemblyFrames.transformVector(frame, 0, 0, 1);
		try {
			var result = part.placed(new Location(new Plane(new Vector(frame.x, frame.y, frame.z), new Vector(x.x, x.y, x.z),
				new Vector(z.x, z.y, z.z))));
			part.close();
			return result;
		} catch (error:Dynamic) {
			part.close();
			throw error;
		}
	}

	/** Checks the fused volume of `a` and `b` is their summed volume less `overlap`; closes both. */
	static function checkOverlap(a:Part, b:Part, overlap:Float, message:String):Void {
		var expected = a.volume() + b.volume() - overlap;
		var fused = a.combine(b);
		a.close();
		b.close();
		near(fused.volume(), expected, message, 1e-6);
		fused.close();
	}

	static function connectorNames(component:MachineComponent):String
		return [for (connector in component.connectors()) connector.name].join(",");

	static function armParts():Void {
		var joint = new ArmJoint(100, 70);
		check(connectorNames(joint) == "stator,rotor",
			"a joint module has a stator and a rotor");
		near(joint.connector("rotor").frame.z, 70, "the rotor face sits at the module length");
		var flange = new RobotFlange(31.5);
		var last = new ArmJoint(55, 40, flange);
		check(connectorNames(last) == "stator,tool", "a flanged module ends in a tool connector");
		near(last.connector("tool").frame.z, 40 + flange.thickness, "the tool connector sits on the flange plate");
		throws(() -> new ArmJoint(40, 40, flange), "too narrow");
		throws(() -> new ArmJoint(0, 40), "positive diameter and length");
		var link = new ArmLink(320, 80, 5, 100, PlusX, MinusX, 90);
		near(link.connector("end").frame.z, 320, "a lateral end joint stays at the tube end");
		near(link.connector("end").frame.x, 45, "a lateral end joint is centred on the tube end");
		near(new ArmLink(90, 90, 5, 100, PlusZ, PlusZ, 70).connector("end").frame.z, 90, "an axial end joint sits on the tube end");
		throws(() -> new ArmLink(8, 80, 5, 100, PlusZ, PlusZ, 0), "wall thinner");
		throws(() -> ArmLink.axisFromToken("+Y"), "Unknown arm axis");
		var rebuilt = ArmLink.recipeType().create(link.values());
		check(rebuilt.designation == link.designation, "an arm link rebuilds from its recipe values");
		check(ArmJoint.recipeType().create(last.values()).designation == last.designation,
			"a flanged joint module rebuilds from its recipe values");
		var solid = link.geometry();
		check(solid.valid() && solid.solidCount() == 1, "an arm link is one solid");
		solid.close();
	}

	static function robotics():Void {
		var flange = new RobotFlange(50);
		check(flange.designation == "ISO9409-STYLE-50-4-M6", "robot flange designation");
		check(flange.boltCount == 4, "robot flange ISO bolt count");
		check(flange.mountScrew == "M6", "robot flange mount screw size");
		near(flange.boltCircleDiameter, 50, "robot flange pitch circle");
		near(flange.pilotDiameter, 31.5, "robot flange pilot diameter");
		near(flange.pinDiameter, 6, "robot flange pin diameter");
		near(flange.flangeDiameter, 70, "robot flange outer diameter");
		near(flange.thickness, 9, "robot flange thickness");
		near(flange.pilotHeight, 3, "robot flange pilot height");
		var small = new RobotFlange(31.5);
		check(small.designation == "ISO9409-STYLE-31.5-4-M5", "smallest ISO flange designation");
		near(small.pilotDiameter, 20, "31.5 flange pilot");
		near(small.pinDiameter, 5, "31.5 flange pin");
		var large = new RobotFlange(80);
		check(large.boltCount == 6 && large.mountScrew == "M8", "80 flange uses 6 x M8");
		check(new RobotFlange(50, 6).designation == "FLANGE-50-6-M6", "non-standard bolt count drops the ISO prefix");
		throws(() -> new RobotFlange(-1), "positive diameter");
		throws(() -> new RobotFlange(45), 'Unknown ISO 9409-1 flange size "45"');
		throws(() -> new RobotFlange(50, 2), "at least 3 bolts");
		throws(() -> new RobotFlange(50, 12), "too many bolts for its pitch circle");

		var envelope = flange.geometry(Envelope);
		solid(envelope, "robot flange envelope");
		var envelopeVolume = envelope.volume();
		near(envelopeVolume, Math.PI * 35 * 35 * 9 + Math.PI * 15.75 * 15.75 * 3, "robot flange envelope volume");
		envelope.close();
		var preview = flange.geometry();
		solid(preview, "robot flange preview");
		near(envelopeVolume - preview.volume(), Math.PI * (4 * 3.3 * 3.3 + 3 * 3) * 9, "robot flange bolt and pin holes");
		var flangeBox = bounds(preview);
		near(flangeBox.minZ, -9, "robot flange plate behind its face");
		near(flangeBox.maxZ, 3, "robot flange pilot boss in front of its face");
		preview.close();
		near(flange.connector("bolt1").frame.x, 25, "robot flange bolt1 x");
		near(flange.connector("bolt2").frame.y, 25, "robot flange bolt2 y");
		near(flange.connector("face").frame.z, 0, "robot flange face connector");
		var pin = flange.pinPoint();
		var flangeX = AssemblyFrames.transformVector(flange.connector("face").frame, 1, 0, 0);
		near(flangeX.x, pin.x / 25, "flange X points toward locating pin X");
		near(flangeX.y, pin.y / 25, "flange X points toward locating pin Y");

		var eoat = new EndEffectorPlate(flange);
		var plateX = AssemblyFrames.transformVector(eoat.connector("robot").frame, 1, 0, 0);
		near(plateX.x, flangeX.x, "adapter mount X follows flange pin");
		near(plateX.y, flangeX.y, "adapter mount Y follows flange pin");
		check(eoat.designation == "EOAT-50-9-4xM5-PCD70.5", "end effector plate designation");
		near(eoat.thickness, flange.thickness, "end effector plate default thickness");
		// Default tool circle: flange bolt circle + flange head + tool head + 2 mm web.
		near(eoat.toolBoltCircleDiameter, 50 + 10 + 8.5 + 2, "end effector tool bolt circle clears the flange heads");
		near(eoat.diameter, 70.5 + 8.5 + 2, "end effector plate grows to carry the tool bolts");
		var eoatEnvelope = eoat.geometry(Envelope);
		solid(eoatEnvelope, "end effector plate envelope");
		var eoatEnvelopeVolume = eoatEnvelope.volume();
		near(eoatEnvelopeVolume, Math.PI * 40.5 * 40.5 * 9, "end effector plate envelope volume");
		eoatEnvelope.close();
		var eoatPreview = eoat.geometry();
		solid(eoatPreview, "end effector plate preview");
		// Every hole removes its own full volume: the pilot recess (31.7 x 3.2 deep), 4 flange bolt
		// holes (6.6), the pin hole (6.2), and 4 tool bolt holes (5.5), none overlapping.
		near(eoatEnvelopeVolume - eoatPreview.volume(),
			Math.PI * (15.85 * 15.85 * 3.2 + (4 * 3.3 * 3.3 + 3.1 * 3.1 + 4 * 2.75 * 2.75) * 9),
			"end effector plate tool bolt holes remove material");
		eoatPreview.close();
		throws(() -> new EndEffectorPlate(flange, 5), "must be thicker than the flange's pilot boss");
		throws(() -> new EndEffectorPlate(flange, null, 30), "must clear the flange's pilot recess");
		throws(() -> new EndEffectorPlate(flange, null, 50), "must clear the flange's bolt holes");
		throws(() -> new EndEffectorPlate(flange, null, 70.5, 40), "too close together");
		near(eoat.connector("tool").frame.z, eoat.thickness, "end effector plate tool connector");

		// Flange -> plate: the plate stacks on the flange face, the pilot boss sits in its recess.
		var plateModel = new AssemblyModel();
		flange.addTo(plateModel, "flange");
		eoat.addTo(plateModel, "plate");
		plateModel.mate("tool-mount", "fixed", "flange", "face", "plate", "robot");
		var platePose = plateModel.pose("plate");
		near(platePose.z, 0, "plate sits on the flange face");
		checkOverlap(flange.geometry(), placedAt(eoat.geometry(), platePose), 0, "flange and plate stack without overlap");
		// A plate without the recess would overlap exactly the boss: the boss engages the recess.
		checkOverlap(flange.geometry(Envelope), placedAt(eoat.geometry(Envelope), platePose),
			Math.PI * 15.75 * 15.75 * 3, "flange pilot boss reaches into the plate");

		var pedestal = new Pedestal(flange, 300);
		check(pedestal.designation == "PEDESTAL-50-D70x300-B14-A102-G0x5.6x4-F0x0-C0", "pedestal designation");
		near(pedestal.columnDiameter, 70, "pedestal column defaults to the flange diameter");
		check(pedestal.floorMountScrew == "M10", "pedestal floor screw size");
		near(pedestal.floorBoltCircleDiameter, 70 + 2 * 16, "pedestal floor bolt circle");
		near(pedestal.baseDiameter, 102 + 2 * 16, "pedestal base diameter");
		// Floor screw heads (16 mm) clear the column and stay on the base.
		check(pedestal.floorBoltCircleDiameter / 2 - 8 >= pedestal.columnDiameter / 2 + 1, "floor bolt heads clear the column");
		check(pedestal.floorBoltCircleDiameter / 2 + 8 <= pedestal.baseDiameter / 2, "floor bolt heads stay on the base");
		throws(() -> new Pedestal(flange, 15), "must clear the flange's pilot boss");
		throws(() -> new Pedestal(flange, 300, 55), "wider than the flange bolt circle plus a screw head");
		var pedestalEnvelope = pedestal.geometry(Envelope);
		solid(pedestalEnvelope, "pedestal envelope");
		pedestalEnvelope.close();
		var pedestalPreview = pedestal.geometry();
		solid(pedestalPreview, "pedestal preview");
		pedestalPreview.close();
		near(pedestal.connector("top").frame.z, 300, "pedestal top connector");
		near(AssemblyFrames.transformVector(pedestal.connector("top").frame, 0, 1, 0).z, -1, "pedestal top points into the pedestal");
		var detailedPedestal = new Pedestal(flange, 300, 70, 4,
			{baseThickness: 18, anchorCircleDiameter: 120, gussetHeight: 80, gussetThickness: 6,
				gussetCount: 4, levelingFootDiameter: 24, levelingFootHeight: 6, cablePathDiameter: 20});
		check(detailedPedestal.designation == "PEDESTAL-50-D70x300-B18-A120-G80x6x4-F24x6-C20",
			"detailed pedestal designation");
		var pedestalRoundTrip = Pedestal.recipeType().create(pedestal.values());
		check(pedestalRoundTrip.designation == pedestal.designation,
			"pedestal designation survives a recipe round trip");
		near(detailedPedestal.baseThickness, 18, "detailed pedestal base thickness");
		near(detailedPedestal.floorBoltCircleDiameter, 120, "detailed pedestal anchor circle");
		near(detailedPedestal.anchorHoleDiameter, 11, "detailed pedestal anchor hole");
		near(detailedPedestal.gussetHeight, 80, "detailed pedestal gusset height");
		near(detailedPedestal.levelingFootHeight, 6, "detailed pedestal foot height");
		near(detailedPedestal.cablePathDiameter, 20, "detailed pedestal cable path");
		var detailedEnvelope = detailedPedestal.geometry(Envelope);
		var detailedPreview = detailedPedestal.geometry();
		solid(detailedEnvelope, "detailed pedestal envelope");
		solid(detailedPreview, "detailed pedestal preview");
		near(bounds(detailedEnvelope).minZ, -6, "detailed pedestal feet extend below floor");
		check(detailedPreview.volume() < detailedEnvelope.volume(), "detailed pedestal machining features");
		check(detailedPedestal.connector("cablePath") != null, "detailed pedestal cable connector");
		check(detailedPedestal.connector("anchor1").role == Mount, "detailed pedestal anchor connector");
		near(detailedPedestal.connector("floor").frame.z, -6, "detailed pedestal floor connector");
		for (point in detailedPedestal.floorBoltPattern()) {
			var holeVolume = Part.cylinderSpan(detailedPedestal.anchorHoleDiameter / 2,
				detailedPedestal.baseThickness + 0.1, detailedPedestal.gussetHeight + 0.1, point.x, point.y);
			var gussetOverlap = detailedPreview.intersect(holeVolume);
			holeVolume.close();
			near(gussetOverlap.volume(), 0, "pedestal gussets clear the anchor-hole envelope", 1e-6);
			gussetOverlap.close();
		}
		detailedPreview.close();
		detailedEnvelope.close();
		var pedestalBom = detailedPedestal.billOfMaterials(40);
		check(pedestalBom.quantity(detailedPedestal.bom.partNumber) == 1, "pedestal BOM body");
		check(pedestalBom.quantity(detailedPedestal.floorMountScrewPart(40).designation) == 4, "pedestal anchor BOM");
		throws(() -> new Pedestal(flange, 300, 70, 4, {baseThickness: 300}), "below its height");
		throws(() -> new Pedestal(flange, 300, 70, 4, {anchorCircleDiameter: 80}), "clear the column");
		throws(() -> new Pedestal(flange, 300, 70, 4, {cablePathDiameter: 32}), "pilot recess diameter");

		// Pedestal -> flange: the flange turns over onto the top face, its boss in the top recess.
		var pedestalModel = new AssemblyModel();
		pedestal.addTo(pedestalModel, "pedestal");
		flange.addTo(pedestalModel, "flange");
		pedestalModel.mate("robot-mount", "fixed", "pedestal", "top", "flange", "face");
		var flangePose = pedestalModel.pose("flange");
		var mountedFlange = placedAt(flange.geometry(), flangePose);
		var mountedBox = bounds(mountedFlange);
		near(mountedBox.minZ, 300 - 3, "flange pilot boss drops into the pedestal");
		near(mountedBox.maxZ, 300 + 9, "flange plate sits on the pedestal top");
		// The flange turns over about X, so its pin (and pedestal's matching hole) lands at -y.
		var pinWorld = AssemblyFrames.transformPoint(flangePose, flange.pinPoint().x, flange.pinPoint().y, 0);
		near(pinWorld.x, flange.pinPoint().x, "flange pin x on the pedestal");
		near(pinWorld.y, -flange.pinPoint().y, "flange pin y mirrored on the pedestal");
		near(pinWorld.z, 300, "flange pin on the pedestal top face");
		checkOverlap(mountedFlange, pedestal.geometry(), 0, "flange and pedestal stack without overlap");
		checkOverlap(placedAt(flange.geometry(Envelope), flangePose), pedestal.geometry(Envelope),
			Math.PI * 15.75 * 15.75 * 3, "flange pilot boss reaches into the pedestal");
	}

	static function assembly():Void {
		assemblyValidation();
		var example = new MotorShaftBearings();
		check(example.screw.designation == "ISO4762-M3x10", "selected mount screw");
		var plate = example.plate.geometry();
		solid(plate, "motor plate");
		near(plate.volume(), 62.3 * 62.3 * 6 - Math.PI * (11.1 * 11.1 + 4 * 1.7 * 1.7) * 6, "plate volume");
		plate.close();
		for (entry in example.components()) {
			var part = entry.component.geometry(Envelope);
			check(part.valid(), '${entry.id} envelope is invalid');
			part.close();
		}

		var model = example.assembly();
		var definition = model.definition("motor-shaft-bearings");
		check(definition.joints.length == 10, "assembly joint count");
		var state = model.initialState("motor-shaft-bearings");
		near(state.worldConnector("plate", "bolt2").z, 6, "plate top");
		var head = state.worldConnector("screw2", "head");
		near(head.x, -15.5, "screw head x");
		near(head.y, 15.5, "screw head y");
		near(head.z, 6, "screw head seat");
		near(state.worldConnector("screw2", "tip").z, -4, "screw engagement");
		near(state.worldConnector("bearingA", "front").z, 34, "first bearing");
		near(state.worldConnector("bearingB", "back").z, 74, "second bearing");

		state.setJoint("coupling", Math.PI / 3);
		state.forwardKinematics();
		for (id in ["bearingA", "bearingB"]) {
			var axis = state.worldConnector(id, "axis");
			near(axis.x, 0, '$id stays on the motor axis x', 1e-9);
			near(axis.y, 0, '$id stays on the motor axis y', 1e-9);
		}
		var turned = state.worldPose("bearingA");
		near(2 * Math.atan2(turned.qz, turned.qw), Math.PI / 3, "bearing turns with the shaft");
		near(state.worldConnector("screw2", "head").x, -15.5, "screws do not rotate");

		var lines = example.bom().lines();
		check(lines.length == 7, "BOM line count");
		var bom = example.bom();
		check(bom.quantity("ISO4762-M3x10") == 4, "screw quantity");
		check(bom.quantity("608-2Z") == 2, "bearing quantity");
		check(bom.quantity("17HS19-1684S1") == 1, "motor quantity");
		check(bom.quantity("DIN6885-B-2x2x6") == 1, "key quantity");
		check(bom.quantity("DIN471-8") == 1, "ring quantity");
		var duplicate = new Bom();
		duplicate.add({partNumber: "X", description: "a", quantity: 1, material: null});
		throws(() -> duplicate.add({partNumber: "X", description: "b", quantity: 1, material: null}), "conflicting");
	}

	static function assemblyValidation():Void {
		var bearing = DeepGrooveBearing.metric("608");
		var invalid = new MachineAssembly();
		invalid.addComponent("a", bearing);
		throws(() -> invalid.addMate("x", "fixed", "missing", "axis", "a", "axis"), "Unknown assembly member");
		throws(() -> invalid.addMate("x", "fixed", "a", "missing", "a", "axis"), "Unknown connector");
		throws(() -> invalid.exposeConnector("x", "a", "missing"), "Unknown connector");
		invalid.addComponent("b", bearing);
		throws(() -> invalid.addMate("x", "invalid", "a", "axis", "b", "axis"), "Unsupported assembly joint");
		invalid.addMate("first", "fixed", "a", "axis", "b", "axis");
		throws(() -> invalid.addMate("first", "fixed", "a", "axis", "b", "axis"), "Duplicate assembly operation");
		invalid.addComponent("c", bearing);
		invalid.addMate("second", "fixed", "c", "axis", "b", "axis");
		throws(() -> invalid.validate(), "two parent joints");

		var cycle = new MachineAssembly();
		cycle.addComponent("a", bearing);
		cycle.addComponent("b", bearing);
		cycle.addMate("ab", "fixed", "a", "axis", "b", "axis");
		cycle.addMate("ba", "fixed", "b", "axis", "a", "axis");
		throws(() -> cycle.validate(), "cycle");
		throws(() -> new MachineAssembly().include("nested", cycle), "cycle");

		var coupling = new MachineAssembly();
		coupling.addComponent("a", bearing);
		coupling.addComponent("b", bearing);
		coupling.addMate("ab", "fixed", "a", "axis", "b", "axis");
		coupling.addCoupling("drive", "ab", "missing", 1);
		throws(() -> coupling.validate(), "missing joint");

		var inner = new MachineAssembly();
		inner.addComponent("a", bearing);
		inner.addComponent("b", bearing);
		inner.addMate("ab", "fixed", "a", "axis", "b", "axis");
		inner.validate();
		var outer = new MachineAssembly();
		outer.include("unit", inner);
		outer.validate();
		throws(() -> outer.include("unit", inner), "Duplicate included assembly");
		check(outer.subassemblies().length == 1 && outer.subassemblies()[0].id == "unit" &&
			outer.subassemblies()[0].assembly != inner && outer.subassemblies()[0].assembly.components().length == 2,
			"included assembly keeps an identified snapshot");
		var model = new AssemblyModel();
		outer.addTo(model, "");
		check(model.definition().joints.length == 1, "included joint is prefixed and valid");
	}

	static function catalogMetadata():Void {
		check(ParallelKey.catalog().metadata("2x2").standard == "DIN 6885-1", "key standard metadata");
		check(RetainingRing.catalog().metadata("8").dimensionKind == Nominal, "ring dimension metadata");
		check(RobotFlange.catalog().metadata("50").conformance == GenericApproximation,
			"raised-pilot ISO-style flange is a generic approximation");
		check(NemaStepper.catalog().metadata("17").dimensionKind == Mixed,
			"NEMA frame dimensions carry manufacturer drawing provenance");
		metadataComplete(DeepGrooveBearing.catalog());
		metadataComplete(FlatWasher.catalog());
		metadataComplete(HexBolt.catalog());
		metadataComplete(HexNut.catalog());
		metadataComplete(ParallelKey.catalog());
		metadataComplete(RetainingRing.catalog());
		metadataComplete(ShaftCollar.catalog());
		metadataComplete(SocketHeadCapScrew.catalog());
		metadataComplete(LinearBearing.catalog());
		metadataComplete(NemaStepper.catalog());
		metadataComplete(NemaStepper.variantCatalog());
		metadataComplete(RobotFlange.catalog());
		metadataComplete(Sprocket.chainCatalog());
		check(TSlotExtrusion.catalog().metadata("HFS5-2020").conformance == NominalEnvelope,
			"HFS5 profile carries nominal-envelope metadata");
		metadataComplete(TSlotExtrusion.catalog());
		metadataComplete(LinearRailSystem.catalog());
		metadataComplete(PillowBlock.catalog());
	}

	static function metadataComplete<T>(catalog:Catalog<T>):Void {
		for (designation in catalog.designations()) {
			var metadata = catalog.metadata(designation);
			check(metadata.source != null && metadata.source.length > 0,
				'catalog metadata source missing for $designation');
			if (metadata.sources != null)
				for (source in metadata.sources)
					check(source != null && source.length > 0, 'catalog metadata supplementary source missing for $designation');
		}
	}

	/** A recipe that names its bodies (cadkit/plans/TOPOLOGICAL_NAMING.md, TN4): a face picked on it follows edits. */
	static function namedStepperFaces():Void {
		var short = NemaStepper.frame(17).geometry();
		var faces = short.shape.elementNames(CadKit.ShapeKind.Face);
		check(faces.indexOf("shaft:cyl.side") >= 0 && faces.indexOf("bolt1:cyl.side") >= 0 && faces.indexOf("pilot:cyl.side") >= 0,
			"stepper faces are named by body: " + faces.join(", "));
		var shaft = faces.indexOf("shaft:cyl.side");
		var connector = cadkit.parametric.GeometricConnectors.capture("shaft", short.shape, CadKit.ShapeKind.Face, shaft);
		var long = NemaStepper.frame(23).geometry(); // a thicker shaft: only its name finds it
		var frame = cadkit.parametric.GeometricConnectors.frame(long.shape, connector);
		near(frame.x, 0, "a connector on the shaft is found on a NEMA 23");
		near(frame.y, 0, "a connector on the shaft is found on a NEMA 23");
		short.close();
		long.close();
	}

	/** Occurrences sharing one scene definition keep all of their connectors; a clash is refused. */
	static function assemblyPreviewSharing():Void {
		function model(frameB:materia.assembly.AssemblyRecord.AssemblyFrame):materia.assembly.AssemblyDefinition {
			var model = new AssemblyModel("mm");
			model.add("a");
			model.connector("a", "shared", Solids.axial(0, 0, 0));
			model.connector("a", "onlyA", Solids.axial(1, 0, 0));
			model.add("b");
			model.connector("b", "shared", frameB);
			model.connector("b", "onlyB", Solids.axial(2, 0, 0));
			return model.definition("sharing");
		}
		var merged = model(Solids.axial(0, 0, 0));
		AssemblyPreview.shareDefinitions(merged, ["b" => "a"]);
		check(merged.definitions.length == 1, "shared occurrences keep one definition");
		var names = [for (connector in merged.definitions[0].connectors) connector.name];
		names.sort(Reflect.compare);
		check(names.join(",") == "onlyA,onlyB,shared", 'shared definition carries every connector, got $names');
		check([for (occurrence in merged.occurrences) occurrence.definition].join(",") == "a,a", "both occurrences use it");
		throws(() -> AssemblyPreview.shareDefinitions(model(Solids.axial(0, 0, 5)), ["b" => "a"]),
			'give connector "shared" different frames');
	}

	static function main():Void {
		namedStepperFaces();
		MachineKitNamingAudit.run();
		MachineAssemblyDescriptionTests.run();
		EndEffectorTests.run();
		EndEffectorSetTests.run();
		EndEffectorComponentTests.run();
		EndEffectorExampleChecks.run();
		RobotArmChecks.run();
		CncRouterChecks.run();
		BenchMillPreview.BenchMillChecks.run();
		BenchMillPreview.EnclosedMillChecks.run();
		TendingReach.run();
		CobotArmPreview.CobotArmChecks.run();
		FoldedZRouterCheck.main();
		MobileBaseChecks.run();
		MobileWelderChecks.run(false);
		RobotWelderChecks.run();
		CoreXyPlotterChecks.run();
		CoreXyDriveTests.run();
		assemblyPreviewSharing();
		RecipeContractTests.run();
		MotorDriverTests.run();
		MillPartTests.run();
		PneumaticPartTests.run();
		PowerSupplyTests.run();
		GearboxTests.run();
		componentRecipes();
		documentRecipes();
		documentPreview();
		MachineKitReferenceTests.run();
		massProperties();
		dimensions();
		catalogMetadata();
		bearings();
		screws();
		motors();
		fasteners();
		shafts();
		shaftHardware();
		structural();
		gears();
		flangeBearingAssembly();
		pillowBlock();
		wheels();
		linearAxis();
		linearRailGuide();
		pickingFrames();
		catalogExtras();
		robotics();
		armParts();
		assembly();
		ports();
		trace("MachineKit smoke passed");
	}
}
