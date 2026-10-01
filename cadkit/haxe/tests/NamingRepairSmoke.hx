import CadKit;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.TopologyResolution.ResolutionMethod;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.TransformFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/**
	Repairing a broken reference (plans/TOPOLOGICAL_NAMING.md, TN5): a slot splits the face a sketch sits on; the
	reference reports the two pieces; retargeting it to one is one undoable change that the next recompute resolves by
	name.
*/
class NamingRepairSmoke {
	public static function run():Void {
		var document = new Document();
		var plate = document.add(new BoxFeature(60, 40, 10));
		var slot = document.add(new TransformFeature(document.add(new BoxFeature(4, 60, 30)), -50, -10, -5));
		var body = document.add(new BooleanFeature(plate, slot, BooleanOperation.Cut));
		document.recompute();
		var shape = body.currentShape();
		var top = -1;
		for (index in 0...shape.subshapeCount(CadKit.ShapeKind.Face))
			if (shape.elementName(CadKit.ShapeKind.Face, index) == "f1:box.+z")
				top = index;
		var face = shape.subshape(CadKit.ShapeKind.Face, top);
		var sketch = document.add(new ConstrainedSketchFeature(square(), body, null, Vector.X(), 0, false, face));
		face.close();
		document.recompute();
		var reference:cadkit.parametric.TopologyReference = sketch.supportFaceReference;
		check(reference.resolvedBy() == ResolutionMethod.Name && document.brokenReferences().length == 0, "the sketch sits on the top face");

		// The slot moves across the plate: the top face splits and the recompute stops, reporting both pieces.
		slot.x.set(28);
		var failed = false;
		try document.recompute() catch (_:Dynamic) failed = true;
		var candidates = reference.candidates();
		check(failed && reference.state == ReferenceState.Ambiguous && candidates.length == 2, "a split support face is ambiguous between its pieces");
		check(document.brokenReferences().length == 1 && document.brokenReferences()[0] == reference, "the document lists the broken reference");
		var right = candidates[0].x > candidates[1].x ? candidates[0] : candidates[1];
		check(right.describe().indexOf("planar face at (") == 0 && right.describe().indexOf("mm²") > 0, "candidates describe themselves: " + right.describe());

		// Choosing the right-hand piece is one change; the next recompute finds it by name.
		reference.retarget(right);
		document.recompute();
		check(reference.isResolved() && reference.resolvedBy() == ResolutionMethod.Name && reference.currentShape().center().get_x() > 30,
			"the retargeted reference resolves to the chosen piece");
		check(document.brokenReferences().length == 0, "nothing is broken after the repair");

		// Undo puts the broken identity back; redo the choice.
		check(document.undo(), "undo the repair");
		check(reference.state == ReferenceState.Ambiguous && reference.fingerprintData().name == "f1:box.+z", "undo restores the broken reference");
		check(document.redo(), "redo the repair");
		document.recompute();
		check(reference.isResolved() && reference.currentShape().center().get_x() > 30, "redo resolves the chosen piece again");
		document.close();
	}

	static function square():ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		var corners = [[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]];
		for (index in 0...4) {
			sketch.addPoint(new SketchPoint("p" + index, corners[index][0], corners[index][1]));
			sketch.addConstraint(SketchConstraint.fixed("fixed" + index, "p" + index));
		}
		for (index in 0...4)
			sketch.addEntity(SketchEntity.line("edge" + index, "p" + index, "p" + ((index + 1) % 4)));
		return sketch;
	}

	static function check(condition:Bool, message:String):Void {
		if (!condition)
			throw "NamingRepairSmoke: " + message;
	}
}
