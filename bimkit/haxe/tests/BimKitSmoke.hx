import bimkit.BimCodec;
import bimkit.BimDocument;
import bimkit.BimError;
import bimkit.BimSchema;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Placement;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.Feature;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.features.BoxFeature;
import haxe.Json;

private class FailingBimFeature extends Feature {
	public function new()
		super();

	override public function evaluate(context:EvaluationContext):EvaluationResult
		throw "injected downstream evaluation failure";
}

class BimKitSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function near(actual:Float, expected:Float, message:String):Void {
		if (Math.abs(actual - expected) > 0.001)
			throw message + ": expected " + expected + ", got " + actual;
	}

	static function parentId(element:cadkit.parametric.Element):String {
		var parent = element.placementParent;
		if (parent == null)
			return "";
		return parent.elementId.value;
	}

	static function propertyValue(element:cadkit.parametric.Element, name:String):Dynamic {
		var property = element.property(name);
		if (property == null)
			throw "missing property " + name;
		return property.value;
	}

	static function definitionPropertyValue(definition:cadkit.parametric.Definition, name:String):Dynamic {
		var property = definition.property(name);
		if (property == null)
			throw "missing definition property " + name;
		return property.value;
	}

	static function encodePlacement(value:Placement):Dynamic {
		var plane = value.location.plane;
		return {
			origin: {x: plane.origin.x, y: plane.origin.y, z: plane.origin.z},
			xDirection: {x: plane.xDirection.x, y: plane.xDirection.y, z: plane.xDirection.z},
			normal: {x: plane.normal.x, y: plane.normal.y, z: plane.normal.z}
		};
	}

	static function spatialBackbone():Void {
		var model = new BimDocument();
		var project = model.createProject("Project");
		var site = model.createSite("Site", project.id);
		var building = model.createBuilding("Building", site.id);
		var base = model.cad.createLevel("Ground datum", 0);
		var upper = model.cad.createLevel("Upper datum", 3200);
		var ground = model.createStorey("Ground Storey", building.id,
			new cadkit.parametric.ElementReference(model.cad.id, base.id),
			new cadkit.parametric.ElementReference(model.cad.id, upper.id));
		var secondBase = model.cad.createLevel("Upper base datum", 3200);
		var roof = model.cad.createLevel("Roof datum", 6200);
		var upperStorey = model.createStorey("Upper Storey", building.id,
			new cadkit.parametric.ElementReference(model.cad.id, secondBase.id),
			new cadkit.parametric.ElementReference(model.cad.id, roof.id));
		var space = model.createSpace("Lobby", ground.id);
		var wall = model.createWall("Exterior wall", 5000, 200, 3200);
		model.addToStorey(ground.id, wall.id);
		var windowDefinition = model.createWindowDefinition("Shared window type", 1200, 1400, 80, 200);
		var window = model.createWindow("Lobby window", windowDefinition);
		model.addToStorey(ground.id, window.id);
		model.hostOpening(window, wall.id, 900, 900);
		var windowTypeClass = windowDefinition.property("bim.class");
		var uniqueWindowType = window.makeUnique();
		var uniqueTypeClass = uniqueWindowType.property("bim.class");
		check(windowDefinition.subgraph != null && windowDefinition.recipe == cadkit.parametric.Definition.SubgraphRecipe
			&& windowDefinition.output("body").purpose == cadkit.parametric.DefinitionOutput.Geometry
			&& windowDefinition.output("opening").purpose == cadkit.parametric.DefinitionOutput.Tool
			&& windowTypeClass != null && windowTypeClass.value == BimSchema.WindowType
			&& uniqueWindowType.subgraph != null && uniqueTypeClass != null && uniqueTypeClass.value == BimSchema.WindowType,
			"Window Types use authored CadKit definitions and makeUnique copies BIM type properties");
		var doorDefinition = model.createDoorDefinition("Shared door type", 900, 2100, 200);
		var firstDoor = model.createDoor("Ground door", doorDefinition);
		var secondDoor = model.createDoor("Upper door", doorDefinition);
		var upperWall = model.createWall("Upper interior wall", 5000, 200, 3000);
		upperWall.setPlacement(new Placement(new Plane(new Vector(0, 5000, 3200), Vector.X(), Vector.Z())));
		model.addToStorey(ground.id, firstDoor.id);
		model.addToStorey(upperStorey.id, upperWall.id);
		model.addToStorey(upperStorey.id, secondDoor.id);
		model.hostOpening(firstDoor, wall.id, 3000, 0);
		model.hostOpening(secondDoor, upperWall.id, 1200, 0);
		var firstDoorVolume = firstDoor.shape().volume();
		doorDefinition.setDefault("width", 1000);
		check(firstDoor.definitionId.value == secondDoor.definitionId.value
			&& firstDoor.shape().volume() > firstDoorVolume
			&& propertyValue(firstDoor, "bim.class") == BimSchema.Door,
			"Door Type edits update shared instances through the same hosted-instance machinery");
		var uniqueDoorType = secondDoor.makeUnique();
		var uniqueDoorTypeClass = uniqueDoorType.property("bim.class");
		check(uniqueDoorType.subgraph != null && uniqueDoorTypeClass != null && uniqueDoorTypeClass.value == BimSchema.DoorType
			&& secondDoor.definitionId.value == uniqueDoorType.id.value,
			"Door instances use the same authored definition and makeUnique behavior as Windows");

		check(project.kind == "object" && site.kind == "object" && building.kind == "object" && ground.kind == "object"
			&& upperStorey.kind == "object" && project.output == null && ground.output == null,
			"BIM spatial classes use generic geometry-free CadKit objects");
		check(propertyValue(ground, "bim.class") == BimSchema.Storey && propertyValue(wall, "bim.class") == BimSchema.Wall,
			"BIM classification stays in domain properties");
		check(model.aggregateChildren(project.id)[0].id.value == site.id.value
			&& model.aggregateChildren(site.id)[0].id.value == building.id.value
			&& model.aggregateChildren(building.id).length == 2,
			"Project, Site, Building and Storey hierarchy is relationship based");
		var groundBase = model.storeyBaseLevel(ground.id);
		var groundTop = model.storeyTopLevel(ground.id);
		check(model.containedElements(ground.id).length == 4 && model.containedElements(upperStorey.id).length == 2
			&& groundBase != null && groundTop != null
			&& groundBase.elementId.value == base.id.value && groundTop.elementId.value == upper.id.value,
			"Storey contents and Level datums are persistent relationships and references");
		var invalidAggregate = false;
		try
			model.aggregate(project.id, building.id)
		catch (error:Dynamic)
			invalidAggregate = true;
		check(invalidAggregate, "spatial aggregation validates its BIM class hierarchy");
		var invalidLevels = false;
		try
			model.setStoreyLevels(ground.id, new cadkit.parametric.ElementReference(model.cad.id, upper.id),
				new cadkit.parametric.ElementReference(model.cad.id, base.id))
		catch (error:Dynamic)
			invalidLevels = true;
		check(invalidLevels, "Storey base and top Levels retain vertical ordering validation");
		var deletionBlocked = false;
		try
			model.cad.removeElement(wall.id)
		catch (error:Dynamic)
			deletionBlocked = true;
		check(deletionBlocked && model.cad.findElement(wall.id) != null,
			"core element deletion refuses a live spatial relationship endpoint");

		var restored = BimCodec.decode(BimCodec.encode(model));
		var restoredGround = restored.cad.element(ground.id);
		var restoredBase = restored.storeyBaseLevel(ground.id);
		var restoredWindow:cadkit.parametric.InstanceElement = cast restored.cad.element(window.id);
		var restoredWindowType = restored.cad.definition(restoredWindow.definitionId);
		var restoredTypeClass = restoredWindowType.property("bim.class");
		var restoredDoor:cadkit.parametric.InstanceElement = cast restored.cad.element(firstDoor.id);
		var restoredDoorType = restored.cad.definition(restoredDoor.definitionId);
		var restoredDoorTypeClass = restoredDoorType.property("bim.class");
		check(restored.cad.implicitOutputEnabled == false && restored.containedElements(ground.id).length == 4
			&& restored.containedElements(upperStorey.id).length == 2
			&& restored.aggregateChildren(building.id).length == 2
			&& restoredBase != null && restoredBase.elementId.value == base.id.value
			&& restoredWindowType.subgraph != null && restoredTypeClass != null && restoredTypeClass.value == BimSchema.WindowType
			&& restoredDoorType.subgraph != null && restoredDoorTypeClass != null && restoredDoorTypeClass.value == BimSchema.DoorType
			&& restored.relationship(firstDoor.id).wallId.value == wall.id.value
			&& propertyValue(restored.cad.element(space.id), "bim.class") == BimSchema.Space,
			"the unified document codec round-trips spatial data, authored types, and hosted doors");
		restored.removeOpening(window.id);
		check(restored.cad.findElement(window.id) == null && restored.containedElements(ground.id).length == 3,
			"opening deletion removes its host and spatial containment edges together");
		check(restored.undo() && restored.cad.findElement(window.id) != null && restored.containedElements(ground.id).length == 4
			&& restored.relationship(window.id).wallId.value == wall.id.value,
			"opening deletion undo restores identity, host and containment relationships");
		check(restored.redo() && restored.cad.findElement(window.id) == null,
			"opening deletion redo removes all BIM edges again");
		restored.removeOpening(firstDoor.id);
		check(restored.containedElements(ground.id).length == 2, "hosted Door deletion removes its type instance and spatial edge");
		restored.removeWall(wall.id);
		check(restored.containedElements(ground.id).length == 1, "wall deletion removes its Storey containment edge");
		check(restored.undo() && restored.containedElements(ground.id).length == 2,
			"wall deletion undo restores its Storey containment edge");
		restored.removeSpace(space.id);
		check(restored.undo() && restored.containedElements(ground.id).length == 2,
			"Space deletion undo restores its persistent containment");
		check(restored.cad.findElement(restoredGround.id) != null, "spatial edits preserve the geometry-free Storey identity");
		restored.close();
		model.close();
	}

	static function completeBuildingFixture():Void {
		var fixture = new BimBuildingFixture();
		var model = fixture.model;
		check(model.aggregateChildren(fixture.project.id)[0].id.value == fixture.site.id.value
			&& model.aggregateChildren(fixture.site.id)[0].id.value == fixture.building.id.value
			&& model.aggregateChildren(fixture.building.id).length == 2
			&& model.containedElements(fixture.groundStorey.id).length == 9
			&& model.containedElements(fixture.upperStorey.id).length == 5
			&& fixture.windows.length == 2 && fixture.doors.length == 2 && fixture.slabs.length == 2
			&& fixture.spaces.length == 3 && model.cad.outputFeatureOrNull() == null,
			"the integration fixture builds a complete two-storey BIM model with shared types");
		check(fixture.groundWalls[0].shape().volume() > 0 && fixture.slabs[0].shape().volume() == 6000.0 * 5000 * 200,
			"authored walls and slabs publish per-element geometry");
		model.cad.element(fixture.project.id).setProperty(TypedProperty.text(BimSchema.GlobalId, "0abcdefghijklmnopqrstu"));
		model.cad.element(fixture.project.id).setProperty(new TypedProperty("ifc.Pset_ProjectCommon.BuildingPermitID",
			"ifc.identifier", {wrappedValue: "LOCAL-17"}, null, null,
			{propertySet: "Pset_ProjectCommon", sourceType: "IfcIdentifier"}));
		model.cad.element(fixture.groundWalls[0].id).setProperty(TypedProperty.boolean("bim.loadBearing", true));
		fixture.windowType.setProperty(TypedProperty.text("ifc.Pset_WindowCommon.Family", "Timber"));
		var ifc = model.exportIfc();
		var ifcAgain = model.exportIfc();
		check(ifc == ifcAgain
			&& ifc.indexOf("FILE_SCHEMA(('IFC4'))") >= 0
			&& ifc.indexOf("=IFCPROJECT('0abcdefghijklmnopqrstu'") >= 0
			&& ifc.indexOf("=IFCSITE(") >= 0 && ifc.indexOf("=IFCBUILDING(") >= 0
			&& ifc.indexOf("=IFCBUILDINGSTOREY(") >= 0 && ifc.indexOf("=IFCSPACE(") >= 0
			&& ifc.indexOf("=IFCWALL(") >= 0 && ifc.indexOf("=IFCSLAB(") >= 0
			&& ifc.indexOf("=IFCWINDOW(") >= 0 && ifc.indexOf("=IFCDOOR(") >= 0
			&& ifc.indexOf("=IFCWINDOWTYPE(") >= 0 && ifc.indexOf("=IFCDOORTYPE(") >= 0
			&& ifc.indexOf("=IFCRELAGGREGATES(") >= 0 && ifc.indexOf("=IFCRELCONTAINEDINSPATIALSTRUCTURE(") >= 0
			&& ifc.indexOf("=IFCRELDEFINESBYTYPE(") >= 0
			&& ifc.indexOf("=IFCRELVOIDSELEMENT(") >= 0 && ifc.indexOf("=IFCRELFILLSELEMENT(") >= 0
			&& ifc.indexOf("=IFCLOCALPLACEMENT(#") >= 0
			&& ifc.indexOf("Pset_ProjectCommon") >= 0 && ifc.indexOf("Pset_WindowCommon") >= 0
			&& ifc.indexOf("CadKitProperties") >= 0
			&& ifc.indexOf("IFCIDENTIFIER('LOCAL-17')") >= 0 && ifc.indexOf("IFCBOOLEAN(.T.)") >= 0
			&& model.cad.outputFeatureOrNull() == null,
			"IFC4 export maps spatial structure, types, hosted openings, typed Psets, and stable GlobalIds without mutating the document");
		var rootGuidPattern = new EReg("^#[0-9]+=(IFCPROJECT|IFCSITE|IFCBUILDING|IFCBUILDINGSTOREY|IFCSPACE|IFCWALL|IFCSLAB|IFCWINDOW|IFCDOOR|IFCWINDOWTYPE|IFCDOORTYPE|IFCOPENINGELEMENT|IFCPROPERTYSET|IFCREL[A-Z]+)\\('([^']+)'", "");
		var globalIds = new Map<String, Bool>();
		var globalIdCount = 0;
		for (line in ifc.split("\n"))
			if (rootGuidPattern.match(line)) {
				var globalId = rootGuidPattern.matched(2);
				if (!new EReg("^[0-3][0-9A-Za-z_$]{21}$", "").match(globalId))
					throw "invalid IFC GlobalId: " + globalId + " (length " + globalId.length + ")";
				if (globalIds.exists(globalId))
					throw "duplicate IFC GlobalId: " + globalId;
				globalIds.set(globalId, true);
				globalIdCount++;
			}
		if (!globalIds.exists("0abcdefghijklmnopqrstu") || globalIdCount <= 20)
			throw "IFC root identity scan failed: supplied=" + globalIds.exists("0abcdefghijklmnopqrstu")
				+ ", count=" + globalIdCount;
		var saved = BimCodec.encode(model);
		var restored = BimCodec.decode(saved);
		var restoredWindow:cadkit.parametric.InstanceElement = cast restored.cad.element(fixture.windows[0].id);
		var restoredWindowType = restored.cad.definition(restoredWindow.definitionId);
		var restoredDoor:cadkit.parametric.InstanceElement = cast restored.cad.element(fixture.doors[0].id);
		check(restored.aggregateChildren(fixture.building.id).length == 2
			&& restored.containedElements(fixture.groundStorey.id).length == 9
			&& restored.openingsForWall(fixture.groundWalls[0].id).length == 2
			&& restored.openingsForWall(fixture.groundWalls[2].id).length == 1
			&& definitionPropertyValue(restored.cad.definition(restoredDoor.definitionId), "bim.class") == BimSchema.DoorType,
			"the complete spatial, type, and host graph survives a single document save and reopen");

		var sharedWindowVolume = restoredWindow.shape().volume();
		var otherWindow:cadkit.parametric.InstanceElement = cast restored.cad.element(fixture.windows[1].id);
		var otherWindowVolume = otherWindow.shape().volume();
		restoredWindowType.setDefault("width", 1300);
		check(restoredWindow.shape().volume() != sharedWindowVolume && otherWindow.shape().volume() != otherWindowVolume,
			"editing a shared Window Type updates both instances");
		check(restored.undo() && restoredWindowType.input("width").defaultValue == 1200,
			"shared Window Type edits undo");
		check(restored.redo() && restoredWindowType.input("width").defaultValue == 1300, "shared Window Type edits redo");
		check(restored.undo(), "shared Window Type edit returns to the saved dimensions");

		var sharedTypeId = restoredWindow.definitionId.value;
		var uniqueType = restoredWindow.makeUnique();
		var uniqueClass = uniqueType.property("bim.class");
		check(restoredWindow.definitionId.value == uniqueType.id.value && uniqueType.id.value != sharedTypeId
			&& uniqueType.subgraph != null && uniqueClass != null && uniqueClass.value == BimSchema.WindowType,
			"makeUnique preserves authored geometry and BIM type properties");
		check(restored.undo() && restoredWindow.definitionId.value == sharedTypeId, "makeUnique undo restores shared type identity");
		check(restored.redo() && restoredWindow.definitionId.value == uniqueType.id.value, "makeUnique redo restores unique type identity");
		check(restored.undo(), "makeUnique returns to the shared building type");

		var upperHeight = restored.wallRole(fixture.upperWalls[0].id).height;
		var restoredUpperLevel:cadkit.parametric.LevelElement = cast restored.cad.element(fixture.upperLevel.id);
		restoredUpperLevel.setElevation(3300);
		restored.cad.recompute();
		check(restored.wallRole(fixture.upperWalls[0].id).height < upperHeight,
			"Storey Level datum edits update level-driven walls");
		check(restored.undo() && restored.wallRole(fixture.upperWalls[0].id).height == upperHeight,
			"Storey Level changes undo with geometry");
		check(restored.redo() && restored.wallRole(fixture.upperWalls[0].id).height < upperHeight,
			"Storey Level changes redo with geometry");
		check(restored.undo(), "Storey Level changes return to the saved building elevations");

		var doorHost = restored.relationship(restoredDoor.id).wallId.value;
		restored.rehostOpening(restoredDoor.id, fixture.groundWalls[1].id, 4200, 0);
		check(restored.relationship(restoredDoor.id).wallId.value == fixture.groundWalls[1].id.value,
			"Door rehosting uses the same host relationship API as Window");
		check(restored.undo() && restored.relationship(restoredDoor.id).wallId.value == doorHost,
			"host reparenting undo restores the original wall");
		check(restored.redo() && restored.relationship(restoredDoor.id).wallId.value == fixture.groundWalls[1].id.value,
			"host reparenting redo restores the new wall");
		check(restored.undo(), "host reparenting returns to the saved building");

		var removedWindowId = otherWindow.id;
		restored.removeOpening(removedWindowId);
		check(restored.cad.findElement(removedWindowId) == null
			&& restored.openingsForWall(fixture.groundWalls[0].id).length == 1,
			"opening deletion updates containment and host projections");
		check(restored.undo() && restored.cad.findElement(removedWindowId) != null
			&& restored.openingsForWall(fixture.groundWalls[0].id).length == 2,
			"opening deletion undo restores the full relationship graph");
		check(restored.redo() && restored.cad.findElement(removedWindowId) == null,
			"opening deletion redo restores the deleted state");
		check(restored.undo(), "opening deletion returns to the complete saved building");
		var reopenedAgain = BimCodec.decode(BimCodec.encode(restored));
		check(reopenedAgain.containedElements(fixture.groundStorey.id).length == 9
			&& reopenedAgain.openingsForWall(fixture.groundWalls[0].id).length == 2,
			"the fully undone integration model remains saveable and reloadable");
		reopenedAgain.close();
		restored.close();
		fixture.close();
	}

	public static function run():Void {
		spatialBackbone();
		completeBuildingFixture();
		var creation = new BimDocument();
		var created = creation.createWall("Temporary wall", 2000, 200, 2500);
		check(creation.undo() && creation.allWallRoles().length == 0 && creation.cad.findElement(created.id) == null,
			"wall creation undo removes its role, element, and active feature");
		check(creation.redo()
			&& creation.wallRole(created.id).elementId.value == created.id.value
			&& creation.cad.outputFeatureOrNull() == null, "wall creation redo restores the same identity without selecting document output");
		creation.close();

		var model = new BimDocument();
		var firstWall = model.createWall("Wall A", 6000, 200, 3000);
		var secondWall = model.createWall("Wall B", 5000, 250, 3000);
		check(model.cad.outputFeatureOrNull() == null, "BIM elements publish their own geometry without a document output");
		secondWall.setPlacement(new Placement(new Plane(new Vector(8000, 0, 0), Vector.X(), Vector.Z())));
		var definition = model.createWindowDefinition("Window", 1200, 1500, 80, 200);
		var first = model.cad.createInstance("Window A", definition);
		model.hostOpening(first, firstWall.id, 1000, 900);
		near(firstWall.shape().volume(), 6000.0 * 200 * 3000 - 1200.0 * 200 * 1500, "hosted opening cuts its wall");
		check(parentId(first) == firstWall.id.value
			&& first.localPlacement.location.plane.origin.x == 1000, "host coordinates derive the instance placement");
		var placementRejected = false;
		try
			first.setPlacement(Placement.identity())
		catch (error:Dynamic)
			placementRejected = true;
		check(placementRejected, "hosted instances reject independently editable placement");
		check(model.openingsForWall(firstWall.id).length == 1, "host opening index is derived from authoritative relationships");

		var second = model.cad.createInstance("Window B", definition);
		model.hostOpening(second, firstWall.id, 3000, 900);
		var committed = firstWall.shape().volume();
		var committedWindow = first.shape();
		var invalidResize = false;
		try
			definition.setDefault("width", 2200)
		catch (error:Dynamic) {
			var typed:BimError = cast error;
			invalidResize = typed.message.indexOf(first.id.value) >= 0 && typed.message.indexOf(second.id.value) >= 0;
		}
		check(invalidResize
			&& firstWall.shape().volume() == committed
			&& first.shape() == committedWindow
			&& definition.input("width").defaultValue == 1200,
			"definition edits reject overlapping openings before changing committed shapes");
		var failed = false;
		try
			model.moveOpening(second.id, 1500, 900)
		catch (error:Dynamic) {
			var typed:BimError = cast error;
			failed = typed.message.indexOf(first.id.value) >= 0;
		}
		check(failed && firstWall.shape().volume() == committed && model.relationship(second.id).along == 3000,
			"overlap diagnostics identify the opening and preserve committed state");
		var outputBeforeFailure = model.cad.outputFeatureOrNull();
		var featuresBeforeFailure = model.cad.featureCount();
		var validator = model.cad.beforeRecompute;
		var validationCalls = 0;
		model.cad.beforeRecompute = function() {
			validationCalls++;
			if (validationCalls == 2) {
				var failure = model.cad.add(new FailingBimFeature());
				model.cad.trackFeatureCreation(failure);
			}
			if (validator != null)
				validator();
		};
		failed = false;
		try
			model.rehostOpening(first.id, secondWall.id, 500, 700)
		catch (error:Dynamic)
			failed = true;
		model.cad.beforeRecompute = validator;
		model.cad.recompute();
		check(failed
			&& model.relationship(first.id).wallId.value == firstWall.id.value
			&& model.cad.outputFeatureOrNull() == outputBeforeFailure
			&& model.cad.featureCount() == featuresBeforeFailure
			&& firstWall.shape().volume() == committed,
			"failed graph update restores relationships, selected output, and recompute viability");
		var failedReload = BimCodec.decode(BimCodec.encode(model));
		check(failedReload.relationship(first.id).wallId.value == firstWall.id.value, "a failed graph update leaves a reloadable document");
		failedReload.close();

		var legacyGraph:Dynamic = Json.parse(DocumentCodec.encode(model.cad));
		Reflect.setField(legacyGraph, "version", 3);
		Reflect.setField(legacyGraph, "implicitOutput", null);
		Reflect.setField(legacyGraph, "relationships", null);
		var legacyElements:Array<Dynamic> = cast Reflect.field(legacyGraph, "elements");
		for (record in legacyElements)
			Reflect.setField(record, "properties", null);
		var legacyDefinitions:Array<Dynamic> = cast Reflect.field(legacyGraph, "definitions");
		for (record in legacyDefinitions)
			Reflect.setField(record, "properties", null);
		var legacyWalls:Array<Dynamic> = [];
		for (role in model.allWallRoles())
			legacyWalls.push({element: role.elementId.value, uncutOutput: role.uncutOutput.id.toInt()});
		var legacyRelationships:Array<Dynamic> = [];
		for (value in model.allRelationships())
			legacyRelationships.push({
				opening: value.openingId.value,
				wall: value.wallId.value,
				along: value.along,
				sill: value.sill,
				output: value.outputName,
				unhostPlacement: encodePlacement(value.unhostPlacement),
				unhostParent: value.unhostParent == null ? null : {
					document: value.unhostParent.documentId.value,
					element: value.unhostParent.elementId.value
				},
				unhostDepth: value.unhostDepth
			});
		var legacyText = Json.stringify({
			format: BimCodec.LEGACY_FORMAT,
			version: BimCodec.LEGACY_VERSION,
			cadkit: Json.stringify(legacyGraph),
			walls: legacyWalls,
			relationships: legacyRelationships
		});
		var migratedLegacy = BimCodec.decode(legacyText);
		check(migratedLegacy.cad.implicitOutputEnabled == false
			&& migratedLegacy.wallRole(firstWall.id).uncutOutput.id.toInt() == model.wallRole(firstWall.id).uncutOutput.id.toInt()
			&& migratedLegacy.relationship(first.id).wallId.value == firstWall.id.value,
			"legacy BimKit v2 data migrates into typed document properties and relationships");
		migratedLegacy.close();

		model.rehostOpening(first.id, secondWall.id, 500, 700);
		near(firstWall.shape().volume(), 6000.0 * 200 * 3000 - 1200.0 * 200 * 1500, "rehosting rebuilds the old wall cut graph");
		near(secondWall.shape().volume(), 5000.0 * 250 * 3000 - 1200.0 * 250 * 1500, "rehosting rebuilds the new wall cut graph");
		check(model.undo(), "rehosting is undoable as one update");
		check(model.relationship(first.id).wallId.value == firstWall.id.value && parentId(first) == firstWall.id.value,
			"undo restores relationship and derived placement together");
		check(model.redo(), "rehosting is redoable as one update");
		var role = model.wallRole(secondWall.id);
		var fixedBody:cadkit.parametric.features.BoxFeature = cast role.body;
		fixedBody.width.set(1000);
		failed = false;
		try
			model.cad.recompute()
		catch (error:Dynamic)
			failed = true;
		check(failed && secondWall.shape().volume() == 5000.0 * 250 * 3000 - 1200.0 * 250 * 1500,
			"wall resizing validates hosted opening bounds before committing geometry");
		check(model.undo(), "invalid wall resize is undoable");
		model.cad.recompute();
		model.resizeWall(secondWall.id, 5500, 300);
		near(secondWall.shape().volume(), 5500.0 * 300 * 3000 - 1200.0 * 300 * 1500, "wall thickness changes update opening tool depth in one transaction");
		var resizedVolume = secondWall.shape().volume();
		failed = false;
		try
			model.resizeWall(secondWall.id, 1000, 300)
		catch (error:Dynamic)
			failed = true;
		check(failed
			&& secondWall.shape().volume() == resizedVolume, "invalid wall resize rolls back dimensions, tools, and committed geometry");

		var beforeResize = secondWall.shape().volume();
		definition.setDefault("width", 1000);
		model.cad.recompute();
		check(secondWall.shape().volume() > beforeResize, "definition edits invalidate opening tools and wall cuts");

		var restored:BimDocument;
		try {
			restored = BimCodec.decode(BimCodec.encode(model));
		} catch (error:Dynamic) {
			var decodeError:BimError = cast error;
			throw decodeError.message;
		}
		check(restored.relationship(first.id).wallId.value == secondWall.id.value, "hosting relationships survive reload");
		check(restored.openingsForWall(secondWall.id).length == 1, "wall opening index survives reload by derivation");
		check(restored.cad.outputFeatureOrNull() == null, "single-document BIM persistence keeps the output unset");
		var altered:Dynamic = Json.parse(BimCodec.encode(model));
		var alteredElements:Array<Dynamic> = cast Reflect.field(altered, "elements");
		for (record in alteredElements)
			if (Reflect.field(record, "id") == first.id.value) {
				var placement:Dynamic = Reflect.field(record, "placement");
				Reflect.setField(Reflect.field(placement, "origin"), "x", 1000000000);
			}
		var repaired = BimCodec.decode(Json.stringify(altered));
		near(repaired.cad.element(first.id).localPlacement.location.plane.origin.x, 500, "host coordinates derive placement again during reload");
		repaired.close();
		var restoredFirst:cadkit.parametric.InstanceElement = cast restored.cad.element(first.id);
		restored.unhostOpening(first.id);
		check(!restoredFirst.placementDerived && parentId(restoredFirst) == "" && restored.openingsForWall(secondWall.id).length == 0,
			"unhosting restores authored placement and clears the host index");
		check(restored.undo() && restoredFirst.placementDerived, "unhosting undo restores the relation and placement authority");
		check(restored.redo() && !restoredFirst.placementDerived, "unhosting redo restores independent placement");
		restored.removeOpening(first.id);
		check(restored.cad.findElement(first.id) == null, "opening deletion removes its element");
		check(restored.undo() && restored.cad.findElement(first.id) != null, "opening deletion undo preserves identity");
		restored.redo();
		var afterDeletion = BimCodec.decode(BimCodec.encode(restored));
		check(afterDeletion.cad.findElement(first.id) == null, "opening deletion survives reload with inactive old tools");
		afterDeletion.removeWall(secondWall.id);
		check(afterDeletion.allWallRoles().length == 1, "empty wall removal updates the BIM registry");
		check(afterDeletion.undo() && afterDeletion.allWallRoles().length == 2, "wall removal undo preserves its identity and role");
		var duplicateRoot:Dynamic = Json.parse(BimCodec.encode(afterDeletion));
		var relationshipRecords:Array<Dynamic> = cast Reflect.field(duplicateRoot, "relationships");
		relationshipRecords.push(Json.parse(Json.stringify(relationshipRecords[0])));
		failed = false;
		try
			BimCodec.decode(Json.stringify(duplicateRoot))
		catch (error:Dynamic)
			failed = true;
		check(failed, "duplicate persistent relationship identities are rejected during reload");
		afterDeletion.close();
		restored.close();
		model.close();

		var repeated = new RepeatedHostedWindows();
		check(repeated.model.openingsForWall(repeated.firstWall.id).length == 2
			&& repeated.model.openingsForWall(repeated.secondWall.id).length == 2,
			"one definition supplies windows across two level-driven walls");
		var sharedDefinition = repeated.model.cad.definition(repeated.windows[0].definitionId);
		var originalWindowVolume = repeated.windows[0].shape().volume();
		sharedDefinition.setDefault("frameThickness", 100);
		repeated.model.cad.recompute();
		check(repeated.windows[0].shape().volume() != originalWindowVolume
			&& repeated.windows[0].shape().volume() == repeated.windows[2].shape().volume(),
			"a shared definition edit updates windows on both walls");
		repeated.windows[1].setOverride("width", 900);
		repeated.model.cad.recompute();
		check(repeated.windows[1].shape().volume() != repeated.windows[0].shape().volume(), "one hosted window retains its instance override");
		repeated.windows[1].removeOverride("width");
		repeated.model.cad.recompute();
		repeated.model.rehostOpening(repeated.windows[0].id, repeated.secondWall.id, 2200, 900);
		check(repeated.model.openingsForWall(repeated.secondWall.id).length == 3
			&& repeated.windows[0].shape().bounds().get_min().get_y() == 5000,
			"level wall rehosting follows the new placement parent");
		check(repeated.model.undo() && repeated.model.openingsForWall(repeated.firstWall.id).length == 2,
			"level wall rehosting undo restores the original host");
		var wallVolume = repeated.firstWall.shape().volume();
		var windowZ = repeated.windows[0].shape().bounds().get_min().get_z();
		repeated.upper.setElevation(3500);
		repeated.model.cad.recompute();
		check(repeated.firstWall.shape().volume() > wallVolume, "upper level changes wall height and retained cuts");
		var ground:cadkit.parametric.LevelElement = cast repeated.model.cad.elementAt(0);
		ground.setElevation(200);
		repeated.model.cad.recompute();
		near(repeated.windows[0].shape().bounds().get_min().get_z(), windowZ + 200, "base level moves the hosted instance with the wall");
		var committedVolume = repeated.firstWall.shape().volume();
		var committedWindowZ = repeated.windows[0].shape().bounds().get_min().get_z();
		repeated.upper.setElevation(100);
		failed = false;
		try
			repeated.model.cad.recompute()
		catch (error:Dynamic)
			failed = true;
		check(failed
			&& repeated.firstWall.shape().volume() == committedVolume
			&& repeated.windows[0].shape().bounds().get_min().get_z() == committedWindowZ,
			"crossed levels preserve committed wall geometry and window placement");
		check(repeated.model.undo(), "crossed level edit is undoable");
		var repeatedReload = BimCodec.decode(BimCodec.encode(repeated.model));
		check(repeatedReload.openingsForWall(repeated.firstWall.id).length == 2, "level-driven hosting and repeated windows survive reload");
		repeatedReload.close();
		repeated.close();
	}
}
