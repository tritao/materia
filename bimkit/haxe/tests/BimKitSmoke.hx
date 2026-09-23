import bimkit.BimCodec;
import bimkit.BimDocument;
import bimkit.BimError;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Placement;
import cadkit.parametric.Feature;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
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

	public static function run():Void {
		var creation = new BimDocument();
		var created = creation.createWall("Temporary wall", 2000, 200, 2500);
		check(creation.undo() && creation.allWallRoles().length == 0 && creation.cad.findElement(created.id) == null,
			"wall creation undo removes its role, element, and active feature");
		check(creation.redo()
			&& creation.wallRole(created.id).elementId.value == created.id.value, "wall creation redo restores the same identity");
		creation.close();

		var model = new BimDocument();
		var firstWall = model.createWall("Wall A", 6000, 200, 3000);
		var secondWall = model.createWall("Wall B", 5000, 250, 3000);
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
		var outputBeforeFailure = model.cad.outputFeature();
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
			&& model.cad.outputFeature() == outputBeforeFailure
			&& model.cad.outputFeature().active
			&& model.cad.featureCount() == featuresBeforeFailure
			&& firstWall.shape().volume() == committed,
			"failed graph update restores relationships, selected output, and recompute viability");
		var failedReload = BimCodec.decode(BimCodec.encode(model));
		check(failedReload.relationship(first.id).wallId.value == firstWall.id.value, "a failed graph update leaves a reloadable document");
		failedReload.close();

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
		var altered:Dynamic = Json.parse(BimCodec.encode(model));
		var alteredCad:Dynamic = Json.parse(Reflect.field(altered, "cadkit"));
		var alteredElements:Array<Dynamic> = cast Reflect.field(alteredCad, "elements");
		for (record in alteredElements)
			if (Reflect.field(record, "id") == first.id.value) {
				var placement:Dynamic = Reflect.field(record, "placement");
				Reflect.setField(Reflect.field(placement, "origin"), "x", 1000000000);
			}
		Reflect.setField(altered, "cadkit", Json.stringify(alteredCad));
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
		var wallRecords:Array<Dynamic> = cast Reflect.field(duplicateRoot, "walls");
		wallRecords.push(wallRecords[0]);
		failed = false;
		try
			BimCodec.decode(Json.stringify(duplicateRoot))
		catch (error:Dynamic)
			failed = true;
		check(failed, "duplicate BIM wall roles are rejected during reload");
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
