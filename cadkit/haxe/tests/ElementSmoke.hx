import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.Shape;
import cadkit.parametric.ElementReference;
import cadkit.parametric.LevelElement;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.Placement;
import cadkit.parametric.Definition;
import cadkit.parametric.DefinitionEvaluator;
import cadkit.parametric.DefinitionEvaluatorRegistry;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.ParametricError;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.DefinitionOutputFeature;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.DatumSketchFeature;
import cadkit.parametric.features.LevelExtrudeFeature;
import cadkit.parametric.features.ExtrudeFeature;
import haxe.Json;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;

private class FailingElementFeature extends Feature {
	public function new()
		super();

	override public function evaluate(context:EvaluationContext):EvaluationResult
		throw "intentional element output failure";
}

private class CountingBoxFeature extends BoxFeature {
	public var evaluations:Int;

	public function new() {
		super(2, 3, 4);
		evaluations = 0;
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		evaluations++;
		return EvaluationResult.fromShape(Shape.box(width.value, depth.value, height.value));
	}
}

private class TestBoxDefinitionEvaluator implements DefinitionEvaluator {
	public function new() {}

	public function evaluate(definition:Definition, instance:cadkit.parametric.InstanceElement, output:String):Shape {
		if (output != "body")
			throw new ParametricError("unsupported test box output: " + output);
		var width = instance.resolved("width");
		var height = instance.resolved("height");
		var depth = instance.resolved("depth");
		if (width <= 0 || height <= 0 || depth <= 0)
			throw new ParametricError("test box dimensions must be positive");
		return Shape.box(width, height, depth);
	}
}

class ElementSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function near(value:Float, expected:Float):Void
		check(Math.abs(value - expected) < 1e-6 * Math.max(1, Math.abs(expected)), 'expected $expected, got $value');

	public static function run():Void {
		var emptyDocument = new Document();
		var emptyReload = DocumentCodec.decode(DocumentCodec.encode(emptyDocument));
		check(emptyReload.featureCount() == 0 && emptyReload.elementCount() == 0
			&& emptyReload.outputFeatureOrNull() == null, "empty documents round-trip without a primary output");
		emptyReload.close();
		emptyDocument.close();

		var datumOnlyDocument = new Document();
		var datumOnlyRoot = datumOnlyDocument.createLevel("Root", 500);
		var datumOnlyChild = datumOnlyDocument.createLevel("Upper", 750, 25,
			new ElementReference(datumOnlyDocument.id, datumOnlyRoot.id));
		datumOnlyDocument.createReferencePlane("Grid A", new Plane(new Vector(10, 0, 0), Vector.X(), Vector.Z()));
		var datumOnlyReload = DocumentCodec.decode(DocumentCodec.encode(datumOnlyDocument));
		check(datumOnlyReload.featureCount() == 0 && datumOnlyReload.elementCount() == 3
			&& datumOnlyReload.elementAt(1).id.value == datumOnlyChild.id.value
			&& datumOnlyReload.outputFeatureOrNull() == null, "datum-only documents round-trip without a primary output");
		near(datumOnlyReload.levelElevation(new ElementReference(datumOnlyReload.id, datumOnlyReload.elementAt(1).id)), 1275);
		datumOnlyReload.close();
		datumOnlyDocument.close();

		var document = new Document();
		var box = document.add(new BoxFeature(10, 20, 30));
		var first = document.createElement("First wall", box);
		var second = document.createElement("Second wall", box);
		check(document.elementCount() == 2, "element registry count");
		check(first.id.value != second.id.value, "new elements receive distinct identities");
		check(document.element(first.id) == first && first.output == box, "element lookup preserves output references");
		var firstId = first.id.value;
		first.rename("North wall");
		check(first.name == "North wall" && document.undo(), "element rename is undoable");
		check(first.name == "First wall" && first.id.value == firstId, "rename undo preserves element identity");
		check(document.redo() && first.name == "North wall", "element rename is redoable");

		var copy = document.duplicateElement(first, "North wall copy");
		check(copy.id.value != first.id.value && copy.output == first.output, "duplicating creates a new element identity");
		check(document.undo() && document.findElement(copy.id) == null, "element creation undo removes the record");
		check(document.redo() && document.findElement(copy.id) == copy, "element creation redo restores the same identity");

		var reference = new ElementReference(document.id, second.id);
		document.removeElement(second.id);
		check(reference.state(document) == ElementReference.UnresolvedElement, "removed element references are explicitly unresolved");
		var failed = false;
		try
			document.installElement("Reused identity", box, second.id)
		catch (error:Dynamic)
			failed = true;
		check(failed && reference.state(document) == ElementReference.UnresolvedElement, "removed element identities remain reserved");
		check(document.undo() && reference.state(document) == ElementReference.Resolved && document.element(reference.elementId) == second,
			"removal undo restores the referenced identity");

		var replacement = document.add(new BoxFeature(5, 5, 5));
		first.setOutput(replacement);
		check(first.id.value == firstId && first.output == replacement, "output replacement preserves element identity");
		check(document.undo() && first.output == box, "output replacement is undoable");

		document.setOutput(box);
		document.recompute();
		var stableVolume = first.shape().volume();
		var failing = document.add(new FailingElementFeature());
		first.setOutput(failing);
		failed = false;
		try
			document.recompute()
		catch (error:Dynamic)
			failed = true;
		check(failed && first.shape().volume() == stableVolume, "failed recompute preserves committed element geometry");

		var transaction = document.beginTransaction();
		var transient = document.createElement("Transient", box);
		transient.rename("Renamed transient");
		transaction.cancel();
		check(document.findElement(transient.id) == null, "transaction cancellation restores the element registry");

		var otherDocument = new Document();
		var otherBox = otherDocument.add(new BoxFeature(1, 1, 1));
		failed = false;
		try
			document.createElement("Foreign", otherBox)
		catch (error:Dynamic)
			failed = true;
		check(failed, "cross-document element outputs are rejected");
		otherDocument.close();
		document.close();

		var persisted = new Document();
		var persistedBox = persisted.add(new BoxFeature(12, 13, 14));
		var persistedElement = persisted.createElement("Persisted wall", persistedBox);
		persisted.setOutput(persistedBox);
		persisted.recompute();
		var level = persisted.createLevel("Level 1", 3000);
		var plane = persisted.createReferencePlane("Grid A", new Plane(new Vector(0, 0, 0), Vector.X(), Vector.Z()));
		plane.setPlane(new Plane(new Vector(5, 0, 0), Vector.X(), Vector.Z()));
		check(persisted.undo() && plane.plane.origin.x == 0, "reference-plane edits are transactional");
		level.setElevation(3.5, "m");
		check(level.elevation == 3500 && persisted.undo() && level.elevation == 3000, "level edits are transactional");
		var encoded = DocumentCodec.encode(persisted);
		var opened = DocumentCodec.decode(encoded);
		check(opened.id.value == persisted.id.value, "opening preserves document identity");
		check(opened.elementCount() == 3
			&& opened.elementAt(0).id.value == persistedElement.id.value, "opening preserves element identity");
		check(opened.elementAt(0).shape().volume() == persistedElement.shape().volume(), "element output survives reload");
		check(opened.elementAt(1).kind == "level"
			&& opened.elementAt(2).id.value == plane.id.value, "geometry-free datums survive reload");
		var cloned = DocumentCodec.decode(encoded, true);
		check(cloned.id.value != persisted.id.value && cloned.elementAt(0).id.value == persistedElement.id.value,
			"cloning assigns a document identity distinct from opening");

		var unsupportedVersion:Dynamic = Json.parse(encoded);
		Reflect.setField(unsupportedVersion, "version", 99);
		failed = false;
		try
			DocumentCodec.decode(Json.stringify(unsupportedVersion))
		catch (error:Dynamic)
			failed = true;
		check(failed, "unsupported document versions are rejected");
		var implicitOutput:Dynamic = Json.parse(encoded);
		Reflect.setField(implicitOutput, "version", 1);
		Reflect.setField(implicitOutput, "output", null);
		Reflect.setField(implicitOutput, "elements", null);
		var migrated = DocumentCodec.decode(Json.stringify(implicitOutput));
		check(migrated.elementCount() == 1
			&& migrated.elementAt(0).name == "Model"
			&& migrated.elementAt(0).output == migrated.outputFeature(),
			"implicit version-one output migrates into the element registry");

		var duplicateIds:Dynamic = Json.parse(encoded);
		var duplicateRecords:Array<Dynamic> = cast Reflect.field(duplicateIds, "elements");
		duplicateRecords.push({id: persistedElement.id.value, name: "Duplicate", output: persistedBox.id.toInt()});
		failed = false;
		try
			DocumentCodec.decode(Json.stringify(duplicateIds))
		catch (error:Dynamic)
			failed = true;
		check(failed, "duplicate persisted element IDs are rejected");
		var dangling:Dynamic = Json.parse(encoded);
		var danglingRecords:Array<Dynamic> = cast Reflect.field(dangling, "elements");
		Reflect.setField(danglingRecords[0], "output", 9999);
		failed = false;
		try
			DocumentCodec.decode(Json.stringify(dangling))
		catch (error:Dynamic)
			failed = true;
		check(failed, "dangling persisted element outputs are rejected");

		migrated.close();
		cloned.close();
		opened.close();
		persisted.close();

		var building = new PersistentBuildingModel();
		check(building.walls.length == 4 && building.document.elementCount() == 5, "building example exposes five independent elements");
		var wallIdentity = building.walls[0].id.value;
		building.resizeWall(0, 7500);
		check(building.walls[0].id.value == wallIdentity, "geometry regeneration preserves wall identity");
		var duplicateWall = building.duplicateWall(0, "North wall copy");
		check(duplicateWall.id.value != wallIdentity, "building wall duplication creates a new identity");
		var restoredBuilding = DocumentCodec.decode(DocumentCodec.encode(building.document));
		check(restoredBuilding.elementCount() == 6
			&& restoredBuilding.elementAt(0).id.value == wallIdentity, "building identities survive save and reload");
		restoredBuilding.close();
		building.close();

		var datumDocument = new Document();
		var baseLevel = datumDocument.createLevel("Base", 0);
		var topLevel = datumDocument.createLevel("Top", 3000);
		var baseRef = new ElementReference(datumDocument.id, baseLevel.id);
		var topRef = new ElementReference(datumDocument.id, topLevel.id);
		check(baseRef.state(datumDocument, "reference-plane") == ElementReference.IncompatibleKind, "datum kind mismatch is explicit");
		var profile = datumDocument.add(new SketchFeature("rectangle", 10, 20));
		var between = datumDocument.add(new LevelExtrudeFeature(profile, baseRef, topRef));
		datumDocument.setOutput(between);
		datumDocument.recompute();
		near(between.currentShape().volume(), 600000);
		topLevel.setElevation(4000);
		datumDocument.recompute();
		near(between.currentShape().volume(), 800000);
		var committed = between.currentShape();
		topLevel.setElevation(-10);
		failed = false;
		try
			datumDocument.recompute()
		catch (e:Dynamic)
			failed = true;
		check(failed && between.currentShape() == committed, "crossed levels preserve committed geometry");
		check(datumDocument.undo(), "crossed level undo");
		datumDocument.recompute();
		failed = false;
		try
			datumDocument.removeElement(topLevel.id)
		catch (e:Dynamic)
			failed = true;
		check(failed && topRef.state(datumDocument) == ElementReference.Resolved, "deleting a level referenced by a feature is rejected");
		var datumReload = DocumentCodec.decode(DocumentCodec.encode(datumDocument));
		near(datumReload.result().volume(), 800000);
		datumReload.close();
		datumDocument.close();

		var relativeDocument = new Document();
		var rootLevel = relativeDocument.createLevel("Root", 1000);
		var relativeLevel = relativeDocument.createLevel("Relative", 200, 0,
			new ElementReference(relativeDocument.id, rootLevel.id));
		var relativeSketch = relativeDocument.add(new DatumSketchFeature("rectangle", 10, 20,
			new ElementReference(relativeDocument.id, relativeLevel.id)));
		var relativeExtrude = relativeDocument.add(new ExtrudeFeature(relativeSketch, 0, 0, 5));
		var unrelatedFeature = relativeDocument.add(new CountingBoxFeature());
		relativeDocument.setOutput(relativeExtrude);
		relativeDocument.recompute();
		near(relativeExtrude.currentShape().bounds().get_min().get_z(), 1200);
		check(unrelatedFeature.evaluations == 1, "unrelated feature evaluates once initially");
		rootLevel.setElevation(1500);
		check(relativeSketch.dirty && relativeExtrude.dirty, "ancestor datum changes invalidate dependent features transitively");
		relativeDocument.recompute();
		near(relativeExtrude.currentShape().bounds().get_min().get_z(), 1700);
		check(unrelatedFeature.evaluations == 1, "unrelated feature remains unevaluated after datum change");
		var externalDocument = new Document();
		var externalLevel = externalDocument.createLevel("External", 900);
		var externalSketch = relativeDocument.add(new DatumSketchFeature("rectangle", 5, 5,
			new ElementReference(externalDocument.id, externalLevel.id)));
		externalSketch.restoreActive(false);
		var relativeClone = DocumentCodec.decode(DocumentCodec.encode(relativeDocument), true);
		var clonedRoot:LevelElement = cast relativeClone.elementAt(0);
		var clonedRelativeLevel:LevelElement = cast relativeClone.elementAt(1);
		var clonedSketch:DatumSketchFeature = cast relativeClone.featureAt(0);
		var clonedExternalSketch:DatumSketchFeature = cast relativeClone.featureAt(externalSketch.id.toInt() - 1);
		var clonedRelativeParent = clonedRelativeLevel.relativeTo;
		check(relativeClone.id.value != relativeDocument.id.value
			&& clonedSketch.datum.documentId.value == relativeClone.id.value
			&& clonedRelativeParent != null && clonedRelativeParent.documentId.value == relativeClone.id.value,
			"cloning remaps internal datum references to the clone");
		check(clonedExternalSketch.datum.documentId.value == externalDocument.id.value,
			"cloning preserves external datum reference identities");
		near(relativeClone.result().bounds().get_min().get_z(), 1700);
		clonedRoot.setElevation(1600);
		relativeClone.recompute();
		near(relativeClone.result().bounds().get_min().get_z(), 1800);
		near(relativeDocument.result().bounds().get_min().get_z(), 1700);
		relativeClone.close();
		externalDocument.close();
		relativeDocument.close();

		var twoLevel = new TwoLevelDatumBuilding();
		var wallId = twoLevel.walls[0].id.value;
		var oldRoofZ = twoLevel.roof.shape().bounds().get_max().get_z();
		twoLevel.setUpper(3500);
		check(twoLevel.walls[0].id.value == wallId && twoLevel.roof.shape().bounds().get_max().get_z() > oldRoofZ,
			"level edit updates walls and roof placement");
		var loadedTwoLevel = DocumentCodec.decode(DocumentCodec.encode(twoLevel.document));
		check(loadedTwoLevel.elementCount() == 8, "two-level building datums and geometry survive reload");
		loadedTwoLevel.close();
		twoLevel.close();

		var hierarchy = new Document();
		var parentFeature = hierarchy.add(new BoxFeature(2, 2, 2));
		var childFeature = hierarchy.add(new BoxFeature(1, 1, 1));
		var otherFeature = hierarchy.add(new BoxFeature(2, 2, 2));
		var parentElement = hierarchy.createElement("Parent", parentFeature);
		var childElement = hierarchy.createElement("Child", childFeature);
		var otherParent = hierarchy.createElement("Other parent", otherFeature);
		hierarchy.setOutput(childFeature);
		hierarchy.recompute();
		parentElement.setPlacement(new Placement(new Plane(new Vector(10, 0, 0), Vector.X(), Vector.Z())));
		childElement.setPlacement(new Placement(new Plane(new Vector(2, 0, 0), Vector.X(), Vector.Z())));
		childElement.reparent(new ElementReference(hierarchy.id, parentElement.id), false);
		near(childElement.shape().bounds().get_min().get_x(), 12);
		check(!childFeature.dirty, "placement changes do not rebuild local geometry");
		parentElement.setPlacement(new Placement(new Plane(new Vector(20, 0, 0), Vector.X(), Vector.Z())));
		near(childElement.shape().bounds().get_min().get_x(), 22);
		otherParent.setPlacement(new Placement(new Plane(new Vector(100, 0, 0), Vector.X(), Vector.Z())));
		childElement.reparent(new ElementReference(hierarchy.id, otherParent.id), true);
		near(childElement.shape().bounds().get_min().get_x(), 22);
		check(hierarchy.undo(), "reparent undo");
		near(childElement.shape().bounds().get_min().get_x(), 22);
		check(hierarchy.redo(), "reparent redo");
		failed = false;
		try
			otherParent.reparent(new ElementReference(hierarchy.id, childElement.id), false)
		catch (e:Dynamic)
			failed = true;
		check(failed, "placement parent cycles are rejected");
		failed = false;
		try
			hierarchy.removeElement(otherParent.id)
		catch (e:Dynamic)
			failed = true;
		check(failed, "deleting a referenced placement parent is rejected");
		childElement.reparent(null, true);
		near(childElement.shape().bounds().get_min().get_x(), 22);
		hierarchy.removeElement(otherParent.id);
		near(childElement.shape().bounds().get_min().get_x(), 22);
		check(hierarchy.undo(), "placement parent deletion undo");
		check(hierarchy.redo(), "placement parent deletion redo");
		var restoredHierarchy = DocumentCodec.decode(DocumentCodec.encode(hierarchy));
		near(restoredHierarchy.elementAt(1).shape().bounds().get_min().get_x(), 22);
		restoredHierarchy.close();
		hierarchy.close();

		DefinitionEvaluatorRegistry.register("cadkit.test.box", new TestBoxDefinitionEvaluator());
		var parts = new Document();
		var definition = parts.createDefinition("Box", "cadkit.test.box", [
			new DefinitionInput("width", ParameterKind.Length, "mm", 10),
			new DefinitionInput("height", ParameterKind.Length, "mm", 20),
			new DefinitionInput("depth", ParameterKind.Length, "mm", 30)
		], [new DefinitionOutput("body", DefinitionOutput.Geometry)]);
		var firstPart = parts.createInstance("Box A", definition);
		var secondPart = parts.createInstance("Box B", definition);
		check(parts.definitionEvaluationCount == 1, "identical instances share registered recipe evaluation");
		var oldVolume = firstPart.shape().volume();
		definition.setDefault("width", 12);
		check(firstPart.shape().volume() != oldVolume && firstPart.shape().volume() == secondPart.shape().volume(),
			"shared definition default edit updates instances");
		secondPart.setOverride("width", 14);
		check(secondPart.shape().volume() != firstPart.shape().volume(), "instance override is independent");
		var evaluations = parts.definitionEvaluationCount;
		secondPart.setPlacement(new Placement(new Plane(new Vector(6000, 0, 0), Vector.X(), Vector.Z())));
		check(parts.definitionEvaluationCount == evaluations, "instance movement reuses local geometry");
		secondPart.removeOverride("width");
		check(secondPart.shape().volume() == firstPart.shape().volume(), "override removal restores inheritance");
		check(parts.undo() && secondPart.overrideValue("width") != null, "override removal undo");
		check(parts.redo() && secondPart.overrideValue("width") == null, "override removal redo");
		var duplicateInstance = parts.duplicateInstance(firstPart, "Box copy");
		check(duplicateInstance.id.value != firstPart.id.value, "instance duplication assigns a new identity");
		var instanceOutput = parts.add(new DefinitionOutputFeature(
			new ElementReference(parts.id, firstPart.id), "body"));
		parts.setOutput(instanceOutput);
		parts.recompute();
		failed = false;
		try parts.removeElement(firstPart.id) catch (_:Dynamic) failed = true;
		check(failed && !instanceOutput.dirty, "deleting an instance referenced by a feature is rejected");
		parts.recompute();
		check(!instanceOutput.dirty, "rejected deletion leaves the referenced output current");
		var committedPart = firstPart.shape();
		failed = false;
		try
			definition.setDefault("width", -1)
		catch (e:Dynamic)
			failed = true;
		check(failed && firstPart.shape() == committedPart, "failed definition edit preserves committed instances");
		var loadedParts = DocumentCodec.decode(DocumentCodec.encode(parts));
		check(loadedParts.allDefinitions().length == 1 && loadedParts.elementAt(0).id.value == firstPart.id.value,
			"definition and instance identities survive reload");
		loadedParts.close();
		parts.close();

		var graph = new Document();
		var graphBody = graph.add(new BoxFeature(10, 20, 30));
		var graphOpening = graph.add(new CylinderFeature(2, 10));
		graph.defineTypedParameter("body.width", 10, ParameterKind.Length, "mm").bind(graphBody.width);
		graph.defineTypedParameter("body.depth", 20, ParameterKind.Length, "mm").bind(graphBody.depth);
		graph.defineTypedParameter("body.height", 30, ParameterKind.Length, "mm").bind(graphBody.height);
		graph.defineTypedParameter("opening.radius", 2, ParameterKind.Length, "mm").bind(graphOpening.radius);
		graph.defineTypedParameter("opening.height", 10, ParameterKind.Length, "mm").bind(graphOpening.height);
		var reusable = new Document();
		var inputBindings = new Map<String, String>();
		inputBindings.set("width", "body.width");
		inputBindings.set("depth", "body.depth");
		inputBindings.set("height", "body.height");
		inputBindings.set("openingRadius", "opening.radius");
		inputBindings.set("openingHeight", "opening.height");
		var outputFeatures = new Map<String, Int>();
		outputFeatures.set("frame", graphBody.id.toInt());
		outputFeatures.set("opening", graphOpening.id.toInt());
		var reusableDefinition = reusable.createSubgraphDefinition("Reusable part", graph, [
			new DefinitionInput("width", ParameterKind.Length, "cm", 1),
			new DefinitionInput("depth", ParameterKind.Length, "mm", 20),
			new DefinitionInput("height", ParameterKind.Length, "mm", 30),
			new DefinitionInput("openingRadius", ParameterKind.Length, "mm", 2),
			new DefinitionInput("openingHeight", ParameterKind.Length, "mm", 10)
		], [
			new DefinitionOutput("frame", DefinitionOutput.Geometry),
			new DefinitionOutput("opening", DefinitionOutput.Tool)
		], inputBindings, outputFeatures);
		graph.close();
		var reusableInstance = reusable.createInstance("Part A", reusableDefinition);
		near(reusableInstance.shape().volume(), 6000);
		near(reusable.definitionOutput(reusableInstance, "opening").volume(), Math.PI * 40);
		reusableDefinition.setDefault("width", 1.2);
		near(reusableInstance.shape().volume(), 7200);
		reusableInstance.setOverride("width", 1.4);
		near(reusableInstance.shape().volume(), 8400);
		var reusableElementId = reusableInstance.id.value;
		var sharedInstance = reusable.createInstance("Part B", reusableDefinition);
		sharedInstance.setPlacement(new Placement(new Plane(new Vector(100, 0, 0), Vector.X(), Vector.Z())));
		var reopenedReusable = DocumentCodec.decode(DocumentCodec.encode(reusable));
		near(reopenedReusable.elementAt(0).shape().volume(), 8400);
		var reopenedInstance:cadkit.parametric.InstanceElement = cast reopenedReusable.elementAt(0);
		near(reopenedReusable.definitionOutput(reopenedInstance, "opening").volume(), Math.PI * 40);
		reopenedReusable.close();
		var sharedDefinitionId = reusableInstance.definitionId.value;
		reusableInstance.setPlacement(new Placement(new Plane(new Vector(50, 0, 0), Vector.X(), Vector.Z())));
		var placementBeforeUnique = reusableInstance.localPlacement;
		var uniqueDefinition = reusableInstance.makeUnique();
		check(uniqueDefinition.id.value != sharedDefinitionId && reusableInstance.id.value == reusableElementId,
			"make unique changes definition identity while preserving element identity");
		check(uniqueDefinition.input("width") != reusableDefinition.input("width"),
			"make unique copies typed inputs rather than sharing defaults");
		check(reusableInstance.localPlacement == placementBeforeUnique,
			"make unique preserves the instance's local placement");
		near(reusableInstance.shape().volume(), 8400);
		near(reusable.definitionOutput(reusableInstance, "opening").volume(), Math.PI * 40);
		uniqueDefinition.setDefault("width", 1.6);
		reusableInstance.removeOverride("width");
		near(reusableInstance.shape().volume(), 9600);
		near(sharedInstance.shape().volume(), 7200);
		var reopenedUnique = DocumentCodec.decode(DocumentCodec.encode(reusable));
		var reopenedUniqueInstance:cadkit.parametric.InstanceElement = cast reopenedUnique.elementAt(0);
		check(reopenedUnique.allDefinitions().length == 2 &&
			reopenedUniqueInstance.definitionId.value == uniqueDefinition.id.value,
			"unique definition identity and instance assignment survive save and reopen");
		near(reopenedUniqueInstance.shape().volume(), 9600);
		near(reopenedUnique.definitionOutput(reopenedUniqueInstance, "opening").volume(), Math.PI * 40);
		reopenedUnique.close();
		check(reusable.undo(), "unique default edit can be undone");
		check(reusable.undo(), "instance override removal can be undone");
		check(reusable.undo(), "make unique can be undone");
		var restoredInstance:cadkit.parametric.InstanceElement = cast reusable.elementAt(0);
		check(restoredInstance.definitionId.value == sharedDefinitionId,
			"undo make unique restores the shared definition reference");
		check(reusable.redo(), "make unique can be redone");
		var redoneInstance:cadkit.parametric.InstanceElement = cast reusable.elementAt(0);
		check(redoneInstance.definitionId.value == uniqueDefinition.id.value,
			"redo make unique restores the unique definition reference");
		reusable.close();

		var definitionTransactionDocument = new Document();
		var definitionTransaction = definitionTransactionDocument.beginTransaction();
		definitionTransactionDocument.createDefinition("Transient", "cadkit.test.box", [
			new DefinitionInput("width", ParameterKind.Length, "mm", 100),
			new DefinitionInput("height", ParameterKind.Length, "mm", 100),
			new DefinitionInput("depth", ParameterKind.Length, "mm", 10)
		], [new DefinitionOutput("body", DefinitionOutput.Geometry)]);
		definitionTransaction.cancel();
		check(definitionTransactionDocument.allDefinitions().length == 0, "definition creation is transactional");
		definitionTransactionDocument.close();
	}
}
