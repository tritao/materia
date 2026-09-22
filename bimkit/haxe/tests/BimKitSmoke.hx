import bimkit.BimCodec;
import bimkit.BimDocument;
import bimkit.BimError;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Placement;

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
		var model = new BimDocument();
		var firstWall = model.createWall("Wall A", 6000, 200, 3000);
		var secondWall = model.createWall("Wall B", 5000, 250, 3000);
		secondWall.setPlacement(new Placement(new Plane(new Vector(8000, 0, 0), Vector.X(), Vector.Z())));
		var definition = model.cad.createWindowDefinition("Window", 1200, 1500, 80, 200);
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
		var failed = false;
		try
			model.moveOpening(second.id, 1500, 900)
		catch (error:Dynamic) {
			var typed:BimError = cast error;
			failed = typed.message.indexOf(first.id.value) >= 0;
		}
		check(failed && firstWall.shape().volume() == committed && model.relationship(second.id).along == 3000,
			"overlap diagnostics identify the opening and preserve committed state");

		model.rehostOpening(first.id, secondWall.id, 500, 700);
		near(firstWall.shape().volume(), 6000.0 * 200 * 3000 - 1200.0 * 200 * 1500, "rehosting rebuilds the old wall cut graph");
		near(secondWall.shape().volume(), 5000.0 * 250 * 3000 - 1200.0 * 250 * 1500, "rehosting rebuilds the new wall cut graph");
		check(model.undo(), "rehosting is undoable as one update");
		check(model.relationship(first.id).wallId.value == firstWall.id.value && parentId(first) == firstWall.id.value,
			"undo restores relationship and derived placement together");
		check(model.redo(), "rehosting is redoable as one update");

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
		restored.close();
		model.close();
	}
}
