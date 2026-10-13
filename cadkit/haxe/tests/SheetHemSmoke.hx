import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.sheetmetal.SheetBaseFeature;
import cadkit.sheetmetal.SheetHemFeature;
import cadkit.sheetmetal.SheetEdgeSelection;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.HemSpec;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchConstraint;
import CadKit;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetHem;
import cadkit.sheetmetal.HemType;
import cadkit.sheetmetal.HemGeometry;
import cadkit.sheetmetal.SheetSide;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.WallEdge;

/** Return flange states use exact zero-contact geometry and a solved circular teardrop bulb. */
class SheetHemSmoke {
	static var assertions:Int = 0;
	static function check(value:Bool, message:String):Void { assertions++; if (!value) throw message; }
	static function near(actual:Float, expected:Float, message:String):Void { check(Math.abs(actual - expected) < 1e-7, message); }
	static function diagnosis(code:String, action:Void->Void):Void {
		var refused = false;
		try { action(); } catch (error:Dynamic) { var failure = Std.downcast(error, SheetDiagnosis); if (failure != null) refused = failure.code == code; }
		check(refused, "hem diagnoses " + code);
	}
	static function sketch():ConstrainedSketch {
		var result = new ConstrainedSketch(); var points = [[0.0, 0.0], [30.0, 0.0], [30.0, 20.0], [0.0, 20.0]];
		for (i in 0...4) {
			var point = result.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = result.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var edge = result.addEntity(SketchEntity.line("edge" + i, "p" + i, "p" + ((i + 1) % 4)));
		}
		return result;
	}
	static function pick(base:SheetBaseFeature):Shape {
		var graph = base.sheet(); var shape = base.currentShape();
		for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = shape.subshape(CadKit.ShapeKind.Face, i);
			try { var edge = SheetEdgeSelection.identify(graph, face);
				if (graph.wall(edge.wall).outline.identity(edge.edge.loop, edge.edge.segment) == "sketch:edge1") return face;
			} catch (error:Dynamic) { if (!Std.isOfType(error, SheetDiagnosis)) { face.close(); throw error; } }
			face.close();
		}
		throw "missing hem test side";
	}
	static function names(feature:SheetHemFeature):String {
		var result = feature.currentShape().elementNames(CadKit.ShapeKind.Face); result.sort(Reflect.compare); return result.join("|");
	}
	static function documentCases():Void {
		for (spec in [HemSpec.Closed, HemSpec.Open(2), HemSpec.Teardrop(0, null), HemSpec.Teardrop(0.2, 1.2)]) for (side in [SheetSide.A, SheetSide.B]) {
			var document = new Document(); var source = document.add(new ConstrainedSketchFeature(sketch()));
			var base = document.add(new SheetBaseFeature(source, SheetDemoParts.materialA())); document.recompute();
			var face = pick(base); var feature = document.add(new SheetHemFeature(base, [face], spec, 10, side)); face.close();
			document.setOutput(feature); document.recompute(); var originalNames = names(feature);
			check(CadKit.shapeValidChecked(feature.currentShape().borrowHandle()) != 0, "document hem is valid");
			var loaded = DocumentCodec.decode(DocumentCodec.encode(document));
			var restored = Std.downcast(loaded.featureById(feature.id.toInt()), SheetHemFeature); if (restored == null) throw "unexpected saved hem type";
			check(restored.type == feature.type && restored.side == side, "typed hem choice and side round trip");
			near(restored.sheet().bend(restored.bendIds()[0]).radius, feature.sheet().bend(feature.bendIds()[0]).radius, "derived hem radius round trips");
			check(names(restored) == originalNames, "hem face names round trip");
			check(restored.wallIds()[0] == feature.wallIds()[0] && restored.bendIds()[0] == feature.bendIds()[0], "hem authored ids round trip");
			feature.length.set(11); document.recompute(); near(feature.sheet().wall(feature.wallIds()[0]).outline.area(), 220, "hem length parameter edits return area");
			check(names(feature) == originalNames, "hem length edits preserve surviving face names");
			check(document.undo(), "undo hem length edit"); document.recompute(); near(feature.length.value, 10, "undo restores hem dimensions");
			check(document.redo(), "redo hem length edit"); document.recompute();
			if (feature.type == HemType.Teardrop && feature.radius == null) {
				base.thickness.set(3); document.recompute(); near(feature.sheet().bend(feature.bendIds()[0]).radius, 2, "hem default bulb radius derives from thickness");
			}
			loaded.close(); document.close();
		}
		var builder = new DocumentBuilder(); var length = builder.length("hem.return", 10);
		var source = builder.constrainedSketch(sketch(), new Map()); var base = builder.sheetBase(source, SheetDemoParts.materialA()); base.document.recompute();
		var face = pick(base); var output = builder.sheetHem(base, [face], HemSpec.Open(2), length); face.close(); builder.output(output); var document = builder.finish();
		length.set(12); document.recompute(); near(output.length.value, 12, "hem builder binds the shared return length"); document.close();
	}
	public static function run():Int {
		assertions = 0;
		for (type in [HemType.Closed, HemType.Open, HemType.Teardrop]) for (side in [SheetSide.A, SheetSide.B]) {
			var graph = new Sheet(SheetDemoParts.materialA()); var wall = graph.addWall(WallOutline.rectangle(30, 20));
			var gap = type == Open ? 2.0 : 0.0;
			var hem = SheetHem.add(graph, wall.id, new WallEdge(1), type, 10, gap, side);
			var bend = hem.bends()[0];
			near(bend.radius, type == Closed ? 0 : 1, "hem radius follows its type");
			check((side == A && bend.angle > 0) || (side == B && bend.angle < 0), "hem side selects the fold sign");
			if (type == Teardrop) {
				near(bend.radius * (1 - Math.cos(Math.abs(bend.angle))) + 10 * Math.sin(Math.abs(bend.angle)), gap, "teardrop tip reaches its authored gap");
				check(Math.abs(bend.angle) > Math.PI, "teardrop return passes a half turn");
			}
			for (state in [hem.flat(), hem.flat().set(bend.id, bend.angle / 2), hem.folded()]) {
				var solid = SheetSolid.build(hem, state);
				check(CadKit.shapeValidChecked(solid.borrowHandle()) != 0, "exact hem flat/folded solid is valid");
				check(solid.subshapeCount(CadKit.ShapeKind.Solid) == 1, "hem material remains one solid");
				var currentAngle = Math.abs(state.angle(bend.id));
				near(solid.volume(), 1200 + 20 * 1.5 * (Math.abs(bend.angle) * (bend.radius + 0.57) + currentAngle * 0.18), "hem state volume follows independent neutral-length integral");
				if (currentAngle > 0) near(hem.evaluate(state).bend(bend.id).curvedNeutralRadius() * currentAngle, Math.abs(bend.angle) * (bend.radius + 0.57), "hem partial state conserves neutral length");
				solid.close();
			}
		}
		var opened = HemGeometry.derive(Teardrop, 10, 0.2, 1.5, 1);
		near(opened.radius * (1 - Math.cos(opened.angle)) + 10 * Math.sin(opened.angle), 0.2, "teardrop supports an explicit tip gap");
		diagnosis("sheet.invalid-hem", function() { var value = HemGeometry.derive(Closed, 10, 1, 1.5, null); });
		diagnosis("sheet.invalid-hem", function() { var value = HemGeometry.derive(Open, 10, 0, 1.5, null); });
		diagnosis("sheet.invalid-hem", function() { var value = HemGeometry.derive(Teardrop, 10, 2, 1.5, 1); });
		diagnosis("sheet.hem-clearance", function() { var value = HemGeometry.derive(Teardrop, 0.1, 0, 1.5, 1); });
		diagnosis("sheet.invalid-angle", function() { cadkit.sheetmetal.BendDimensions.validate(1.5, 1, 2 * Math.PI); });
		documentCases();
		return assertions;
	}
	static function main():Void { trace("Sheet hem smoke passed (" + run() + " assertions)"); }
}
