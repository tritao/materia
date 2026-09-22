import CadKit;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.LinearPatternFeature;
import cadkit.parametric.features.TransformFeature;

class PatternSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function near(value:Float, expected:Float):Void {
		check(Math.abs(value - expected) < 1e-5, 'expected $expected, got $value');
	}

	public static function run():Void {
		var document = new Document();
		var target = document.add(new BoxFeature(30, 10, 5));
		var tool = document.add(new CylinderFeature(1, 5));
		var positionedTool = document.add(new TransformFeature(tool, 15, 5, 0));
		var holes = document.add(new LinearPatternFeature(positionedTool, 3, 8, Vector.X()));
		var count = document.defineParameter("holes.count", 3);
		var spacing = document.defineParameter("holes.spacing", 8);
		count.bind(holes.count);
		spacing.bind(holes.spacing);
		var cut = document.add(new BooleanFeature(target, holes, BooleanOperation.Cut));
		document.setOutput(cut);
		document.recompute();
		near(document.result().volume(), 30 * 10 * 5 - 3 * Math.PI * 5);

		count.set(5);
		spacing.set(5);
		document.recompute();
		near(document.result().volume(), 30 * 10 * 5 - 5 * Math.PI * 5);
		check(document.undo(), "linear spacing undo");
		check(document.undo(), "linear count undo");
		document.recompute();
		near(document.result().volume(), 30 * 10 * 5 - 3 * Math.PI * 5);
		check(document.redo(), "linear count redo");
		check(document.redo(), "linear spacing redo");
		document.recompute();
		near(document.result().volume(), 30 * 10 * 5 - 5 * Math.PI * 5);

		var failed = false;
		try count.set(3.5) catch (error:Dynamic) failed = true;
		check(failed && count.value == 5, "linear count rejects non-integers atomically");
		var restored = DocumentCodec.decode(DocumentCodec.encode(document));
		near(restored.result().volume(), document.result().volume());
		restored.parameter("holes.count").set(3);
		restored.recompute();
		near(restored.result().volume(), 30 * 10 * 5 - 3 * Math.PI * 5);
		restored.close();
		document.close();

		var additionDocument = new Document();
		var additionTarget = additionDocument.add(new BoxFeature(30, 10, 5));
		var boss = additionDocument.add(new BoxFeature(1, 1, 1));
		var positionedBoss = additionDocument.add(new TransformFeature(boss, 15, 5, 5));
		var bosses = additionDocument.add(new LinearPatternFeature(positionedBoss, 3, 5, Vector.X()));
		var added = additionDocument.add(new BooleanFeature(additionTarget, bosses, BooleanOperation.Fuse));
		additionDocument.setOutput(added);
		additionDocument.recompute();
		near(additionDocument.result().volume(), 30 * 10 * 5 + 3);
		additionDocument.close();

		var twoAxisDocument = new Document();
		var seed = twoAxisDocument.add(new BoxFeature(1, 1, 1));
		var array = twoAxisDocument.add(new LinearPatternFeature(seed, 3, 4, new Vector(1, 1, 0), 2, 6, Vector.Z()));
		twoAxisDocument.setOutput(array);
		twoAxisDocument.recompute();
		near(twoAxisDocument.result().volume(), 6);
		check(array.currentShape().subshapeCount(CadKit.ShapeKind.Solid) == 6, "two-axis linear pattern instance count");
		twoAxisDocument.close();
	}
}
