import cadkit.parametric.DocumentCodec;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sheetmetal.SheetEdgeSelection;
import cadkit.sheetmetal.HemSpec;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchConstraint;
import CadKit;
import cadkit.parametric.Document;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.RecomputeError;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetFeature;
import cadkit.sheetmetal.SheetRipFeature;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.WallEdge;

/** Zero-gap graph seams open a closure and retain its allowance strip. */
class SheetRipSmoke {
	static var assertions:Int = 0;
	static function check(value:Bool, message:String):Void { assertions++; if (!value) throw message; }
	static function codec():Void {
		var builder = new DocumentBuilder(); var sketch = new ConstrainedSketch(); var points = [[0.0, 0.0], [12.0, 0.0], [12.0, 8.0], [0.0, 8.0]];
		for (i in 0...4) {
			var point = sketch.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = sketch.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var edge = sketch.addEntity(SketchEntity.line("edge" + i, "p" + i, "p" + ((i + 1) % 4)));
		}
		var source = builder.constrainedSketch(sketch, new Map()); var base = builder.sheetBase(source, SheetDemoParts.materialA()); base.document.recompute();
		var shape = base.currentShape(); var selected:Null<cadkit.Shape> = null;
		for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = shape.subshape(CadKit.ShapeKind.Face, i); var matches = false;
			try { var edge = SheetEdgeSelection.identify(base.sheet(), face); matches = edge.edge.segment == 1; }
			catch (error:Dynamic) { if (!Std.isOfType(error, SheetDiagnosis)) { face.close(); throw error; } }
			if (matches) { selected = face; break; } face.close();
		}
		if (selected == null) throw "missing saved rip source edge";
		var length = builder.length("rip.return", 3); var hem = builder.sheetHem(base, [selected], HemSpec.Closed, length); selected.close();
		var inactive = builder.sheetRip(hem, hem.bendIds()[0]); inactive.restoreActive(false); builder.output(hem); var document = builder.finish();
		var loaded = DocumentCodec.decode(DocumentCodec.encode(document));
		var rip = Std.downcast(loaded.featureById(inactive.id.toInt()), SheetRipFeature); if (rip == null) throw "unexpected saved rip type";
		check(rip.bend == hem.bendIds()[0] && rip.source.id.toInt() == hem.id.toInt(), "typed rip record preserves graph identities and source");
		check(!rip.active, "inactive rip record round trips without executing a disconnected cut");
		loaded.close(); document.close();
	}
	public static function run():Int {
		assertions = 0;
		var document = new Document(); var source = document.add(new SheetRipRingSource());
		var rip = document.add(new SheetRipFeature(source, 4)); document.setOutput(rip); document.recompute();
		check(rip.sheet().isRipped(4), "document rip records the authored bend seam");
		check(!source.sheet().isRipped(4), "rip leaves its source graph intact");
		var graph = rip.sheet(); var flat = graph.evaluate(graph.flat());
		for (wall in graph.walls()) check(Math.abs(flat.wall(wall.id).plane.origin.z) < 1e-9, "ripped closure unfolds into one plane");
		check(flat.bend(4).allowance == graph.bendAllowance(graph.bend(4)), "rip retains the closing bend strip");
		var solid = SheetSolid.build(graph, graph.flat()); check(CadKit.shapeValidChecked(solid.borrowHandle()) != 0, "ripped flat material is valid"); solid.close();
		var second = document.add(new SheetRipFeature(rip, 2)); document.setOutput(second); var refused = false;
		try { document.recompute(); } catch (error:Dynamic) {
			var failure = Std.downcast(error, RecomputeError); if (failure != null) {
				var diagnosis = Std.downcast(failure.cause, SheetDiagnosis); if (diagnosis != null) refused = diagnosis.code == "sheet.disconnected";
			}
		}
		check(refused, "a rip that disconnects a wall is diagnosed"); check(rip.sheet().isRipped(4) && !rip.sheet().isRipped(2), "failed rip preserves the published graph");
		document.close(); codec(); return assertions;
	}
	static function main():Void { trace("Sheet rip smoke passed (" + run() + " assertions)"); }
}

private class SheetRipRingSource extends SheetFeature {
	public function new() { super(); }
	override public function buildSheet(context:EvaluationContext):Sheet {
		var graph = new Sheet(SheetDemoParts.materialA());
		var walls = [for (i in 0...4) graph.addWall(WallOutline.rectangle(10, 3))];
		for (i in 0...4) { var bend = graph.addBend(walls[i].id, new WallEdge(1), walls[(i + 1) % 4].id, new WallEdge(3), Math.PI / 2, 1); }
		return graph;
	}
}
