import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.HoleFeature;
import cadkit.parametric.features.LinearPatternFeature;

class HoleSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function near(value:Float, expected:Float):Void {
		check(Math.abs(value - expected) < 1e-5, 'expected $expected, got $value');
	}

	public static function run():Void {
		var top = new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1);

		var blindDocument = new Document();
		var blindTarget = blindDocument.add(new BoxFeature(20, 20, 10));
		var blindTool = blindDocument.add(HoleFeature.plain(blindTarget, top, Vector.X(), "blind", 0, 0, 4, 3));
		var diameter = blindDocument.defineParameter("hole.diameter", 4);
		var depth = blindDocument.defineParameter("hole.depth", 3);
		diameter.bind(blindTool.diameter);
		depth.bind(blindTool.depth);
		var blindCut = blindDocument.add(new BooleanFeature(blindTarget, blindTool, BooleanOperation.Cut));
		blindDocument.setOutput(blindCut);
		blindDocument.recompute();
		near(blindDocument.result().volume(), 20 * 20 * 10 - Math.PI * 2 * 2 * 3);
		depth.set(5);
		blindDocument.recompute();
		near(blindDocument.result().volume(), 20 * 20 * 10 - Math.PI * 2 * 2 * 5);
		check(blindDocument.undo(), "blind hole depth undo");
		blindDocument.recompute();
		near(blindDocument.result().volume(), 20 * 20 * 10 - Math.PI * 2 * 2 * 3);
		var restoredBlind = DocumentCodec.decode(DocumentCodec.encode(blindDocument));
		near(restoredBlind.result().volume(), blindDocument.result().volume());
		restoredBlind.close();
		blindDocument.close();

		var throughDocument = new Document();
		var throughTarget = throughDocument.add(new BoxFeature(20, 20, 10));
		var throughTool = throughDocument.add(HoleFeature.plain(throughTarget, top, Vector.X(), "through-all", 5, 0, 4));
		var throughCut = throughDocument.add(new BooleanFeature(throughTarget, throughTool, BooleanOperation.Cut));
		throughDocument.setOutput(throughCut);
		throughDocument.recompute();
		near(throughDocument.result().volume(), 20 * 20 * 10 - Math.PI * 2 * 2 * 10);
		near(throughTool.currentShape().bounds().get_min().get_x(), 15 - 2);
		throughTarget.height.set(15);
		throughDocument.recompute();
		near(throughDocument.result().volume(), 20 * 20 * 15 - Math.PI * 2 * 2 * 15);
		throughDocument.close();

		var counterboreDocument = new Document();
		var counterboreTarget = counterboreDocument.add(new BoxFeature(30, 30, 10));
		var counterboreTool = counterboreDocument.add(HoleFeature.counterbore(counterboreTarget, top, Vector.X(),
			"through-all", 0, 0, 4, 1, 8, 2));
		var counterbores = counterboreDocument.add(new LinearPatternFeature(counterboreTool, 2, 16, Vector.X(), 2, 16, Vector.Y()));
		var counterboreCut = counterboreDocument.add(new BooleanFeature(counterboreTarget, counterbores, BooleanOperation.Cut));
		counterboreDocument.setOutput(counterboreCut);
		counterboreDocument.recompute();
		var oneCounterbore = Math.PI * 2 * 2 * 10 + Math.PI * (4 * 4 - 2 * 2) * 2;
		near(counterboreDocument.result().volume(), 30 * 30 * 10 - 4 * oneCounterbore);
		var committedCounterbores = counterboreDocument.result();
		counterboreTool.recessDiameter.set(3);
		var failed = false;
		try counterboreDocument.recompute() catch (error:Dynamic) failed = true;
		check(failed && counterboreDocument.result() == committedCounterbores,
			"invalid counterbore recompute preserves committed geometry");
		counterboreDocument.close();

		var lid = new CounterboredEnclosureLid();
		var initialCounterbore = Math.PI * 2 * 2 * 6 + Math.PI * (4 * 4 - 2 * 2) * 2;
		near(lid.finish.currentShape().volume(), 60 * 40 * 6 - 4 * initialCounterbore);
		lid.resize(80, 50, 8);
		var resizedCounterbore = Math.PI * 2 * 2 * 8 + Math.PI * (4 * 4 - 2 * 2) * 2;
		near(lid.finish.currentShape().volume(), 80 * 50 * 8 - 4 * resizedCounterbore);
		var restoredLid = DocumentCodec.decode(DocumentCodec.encode(lid.document));
		near(restoredLid.result().volume(), lid.finish.currentShape().volume());
		restoredLid.close();
		lid.close();
	}
}
