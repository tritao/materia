import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ElementReference;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.Placement;
import cadkit.parametric.features.BoxFeature;
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
		return super.evaluate(context);
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
		datumDocument.removeElement(topLevel.id);
		failed = false;
		try
			datumDocument.recompute()
		catch (e:Dynamic)
			failed = true;
		check(failed && topRef.state(datumDocument) == ElementReference.UnresolvedElement, "deleted level fails explicitly");
		check(datumDocument.undo(), "deleted level undo");
		datumDocument.recompute();
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
		hierarchy.removeElement(otherParent.id);
		failed = false;
		try
			childElement.shape()
		catch (e:Dynamic)
			failed = true;
		check(failed, "missing placement parent is explicit");
		check(hierarchy.undo(), "placement parent removal undo");
		var restoredHierarchy = DocumentCodec.decode(DocumentCodec.encode(hierarchy));
		near(restoredHierarchy.elementAt(1).shape().bounds().get_min().get_x(), 22);
		restoredHierarchy.close();
		hierarchy.close();

		var windows = new RepeatedWindows();
		check(windows.document.definitionEvaluationCount == 1, "identical instances share definition evaluation");
		check(windows.definition.output("body").purpose == "geometry" && windows.definition.output("opening").purpose == "tool",
			"definition outputs declare geometry and tool purposes");
		check(windows.document.definitionOutput(windows.first, "opening").volume() > windows.first.shape().volume(),
			"tool outputs are available explicitly and excluded from instance display");
		var firstWindowId = windows.first.id.value;
		var oldVolume = windows.first.shape().volume();
		windows.definition.setDefault("frameThickness", 100);
		check(windows.first.shape().volume() != oldVolume && windows.first.shape().volume() == windows.second.shape().volume(),
			"shared default edit updates instances");
		windows.second.setOverride("width", 1400);
		check(windows.second.shape().volume() != windows.first.shape().volume(), "instance override is independent");
		var evaluations = windows.document.definitionEvaluationCount;
		windows.second.setPlacement(new Placement(new Plane(new Vector(6000, 0, 0), Vector.X(), Vector.Z())));
		check(windows.document.definitionEvaluationCount == evaluations, "instance movement reuses local geometry");
		windows.second.removeOverride("width");
		check(windows.second.shape().volume() == windows.first.shape().volume(), "override removal restores inheritance");
		check(windows.document.undo() && windows.second.overrideValue("width") != null, "override removal undo");
		check(windows.document.redo() && windows.second.overrideValue("width") == null, "override removal redo");
		var windowCopy = windows.document.duplicateInstance(windows.first, "Window copy");
		check(windowCopy.id.value != windows.first.id.value, "instance duplication assigns a new identity");
		var committedWindow = windows.first.shape();
		failed = false;
		try
			windows.definition.setDefault("frameThickness", 700)
		catch (e:Dynamic)
			failed = true;
		check(failed && windows.first.shape() == committedWindow, "failed definition edit preserves committed instances");
		var loadedWindows = DocumentCodec.decode(DocumentCodec.encode(windows.document));
		check(loadedWindows.allDefinitions().length == 1 && loadedWindows.elementAt(2).id.value == firstWindowId,
			"definition and instance identities survive reload");
		check(loadedWindows.allDefinitions()[0].output("opening").purpose == "tool", "definition output purposes survive reload");
		loadedWindows.close();
		windows.close();
		var definitionTransactionDocument = new Document();
		var definitionTransaction = definitionTransactionDocument.beginTransaction();
		definitionTransactionDocument.createWindowDefinition("Transient", 100, 100, 10, 10);
		definitionTransaction.cancel();
		check(definitionTransactionDocument.allDefinitions().length == 0, "definition creation is transactional");
		definitionTransactionDocument.close();
	}
}
