import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sheetmetal.SheetBaseFeature;
import cadkit.sheetmetal.SheetCutFeature;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetEdgeFlangeFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.SheetCutGeometry;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetSolid;
import cadkit.sheetmetal.SheetState;
import cadkit.sheetmetal.WallOutline;

/** Developed cuts cross walls and bends with exact curves, conserved axes and named native faces. */
class SheetBendCutSmoke {
	static var checks:Int = 0;
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function near(actual:Float, expected:Float, label:String):Void {
		check(Math.isFinite(actual) && Math.abs(actual - expected) <= 1e-5, label + ": expected " + expected + ", got " + actual);
	}
	static function names(shape:Shape):String {
		var names = shape.elementNames(CadKit.ShapeKind.Face); names.sort(Reflect.compare);
		for (i in 0...names.length) check(names[i] != "" && (i == 0 || names[i] != names[i - 1]), "trimmed faces have unique identities");
		return names.join("|");
	}
	static function documentCuts():Void {
		var document = new Document(); var sketch = new ConstrainedSketch();
		var points = [[0.0, 0.0], [20.0, 0.0], [20.0, 100.0], [0.0, 100.0]];
		for (i in 0...4) {
			var added = sketch.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = sketch.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var edge = sketch.addEntity(SketchEntity.line("edge" + i, "p" + i, "p" + ((i + 1) % 4)));
		}
		var profile = document.add(new ConstrainedSketchFeature(sketch));
		var base = document.add(new SheetBaseFeature(profile, SheetDemoParts.materialA())); document.recompute();
		var selected:Null<Shape> = null; var baseShape = base.currentShape();
		for (i in 0...baseShape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = baseShape.subshape(CadKit.ShapeKind.Face, i);
			if (face.surfaceKind() == CadKit.SurfaceKind.Plane && Vector.fromNative(face.faceNormal()).x > 0.999) selected = face;
			else face.close();
		}
		if (selected == null) throw "missing flange attachment";
		var flange = document.add(new SheetEdgeFlangeFeature(base, [selected], 30, Math.PI / 2, 1)); selected.close(); document.recompute();
		var graph = flange.sheet(); var bend = graph.bends()[0]; var allowance = graph.bendAllowance(bend);
		var holeSketch = new ConstrainedSketch(); var center = holeSketch.addPoint(new SketchPoint("center", 20 + allowance / 2, 50));
		var fixed = holeSketch.addConstraint(SketchConstraint.fixed("fixed-center", "center"));
		var circle = holeSketch.addEntity(SketchEntity.circle("hole", "center", 2));
		var radius = holeSketch.addConstraint(SketchConstraint.radius("radius", "hole", 2));
		var hole = document.add(new ConstrainedSketchFeature(holeSketch));
		var cut = document.add(new SheetCutFeature(flange, hole, 1)); document.setOutput(cut); document.recompute();
		check(cut.sheet().bend(bend.id).hasCuts(), "production cut owns derived bend tools");
		var originalNames = names(cut.currentShape()); var originalVolume = cut.currentShape().volume(1e-10);
		var loaded = DocumentCodec.decode(DocumentCodec.encode(document));
		var restored = Std.downcast(loaded.featureById(cut.id.toInt()), SheetCutFeature); if (restored == null) throw "unexpected saved cross-bend cut";
		near(restored.currentShape().volume(1e-10), originalVolume, "cross-bend codec volume");
		check(names(restored.currentShape()) == originalNames, "cross-bend codec identities");
		flange.radius.set(1.1); document.recompute();
		check(cut.sheet().bend(bend.id).cutTools().length == 1, "rebuild derives cuts once from current stock geometry");
		near(cut.sheet().bend(bend.id).radius, 1.1, "cross-bend cut consumes staged bend radius");
		check(names(cut.currentShape()) == originalNames, "cross-bend names survive radius edit");
		check(document.undo(), "undo upstream bend radius"); document.recompute(); near(cut.currentShape().volume(1e-10), originalVolume, "undo cross-bend material geometry");
		check(document.redo(), "redo upstream bend radius"); document.recompute();
		base.thickness.set(1.8); document.recompute(); check(cut.sheet().bend(bend.id).cutTools().length == 1, "thickness rebuild does not accumulate cut tools");
		var committedVolume = cut.currentShape().volume(1e-10); hole.dimension("radius").set(100); var refused = false;
		try { document.recompute(); } catch (error:Dynamic) {
			var failure = Std.downcast(error, RecomputeError); if (failure != null) {
				var diagnosis = Std.downcast(failure.cause, SheetDiagnosis); if (diagnosis != null) refused = diagnosis.code == "sheet.cut-empty";
			}
		}
		check(refused, "invalid cross-bend cut names the consumed wall"); near(cut.currentShape().volume(1e-10), committedVolume, "invalid cross-bend edit retains committed solid");
		check(cut.sheet().bend(bend.id).cutTools().length == 1, "invalid cross-bend edit retains committed graph");
		loaded.close(); document.close();
	}

	public static function run():Int {
		checks = 0;
		for (sign in [1.0, -1.0]) {
			var source = SheetDemoParts.p1(); var bend = source.bends()[0];
			var changed = source.updateBend(bend.id, sign * bend.angle, bend.radius);
			var allowance = source.bendAllowance(changed); var edge = changed.edgeA.span;
			var radius = 2.0; var center = new Point2(edge.start.x + allowance / 2, 50);
			var tool = new WallOutline([Segment.arc(center, radius, 0, 2 * Math.PI)], [], [["circle"]]);
			var cut = SheetCutGeometry.apply(source, 1, tool, "cross-circle");
			near(source.wall(1).outline.area(), 5750, "source graph remains isolated");
			check(cut.bend(changed.id).hasCuts(), "circle cuts bend material");
			near(cut.bend(changed.id).edgeA.span.length(), 100, "cut preserves nominal seam width");
			var half = allowance / 2;
			var stripCutArea = 2 * (half * Math.sqrt(radius * radius - half * half) + radius * radius * Math.asin(half / radius));
			var materialArea = 0.0; for (region in cut.bend(changed.id).materialRegions(allowance)) materialArea += region.area();
			near(materialArea, allowance * 100 - stripCutArea, "independent circular band integral");
			var removedWallArea = 0.0; var originalWallArea = 0.0;
			for (wall in source.walls()) { originalWallArea += wall.outline.area(); removedWallArea += wall.outline.area() - cut.wall(wall.id).outline.area(); }
			near(removedWallArea, Math.PI * radius * radius - stripCutArea, "one circle is shared exactly by all developed domains");
			var identity = "";
			for (fraction in [0.0, 0.5, 1.0]) {
				var state = new SheetState().set(changed.id, changed.angle * fraction);
				var solid = SheetSolid.build(cut, state);
				check(CadKit.shapeValidChecked(solid.borrowHandle()) != 0, "trimmed bend builds a valid native solid");
				near(solid.subshapeCount(CadKit.ShapeKind.Solid), 1, "cross-bend cut remains one solid");
				var zone = cut.evaluate(state).bend(changed.id); var factor = fraction == 0 ? 1 : Math.abs(zone.angle) * (zone.curvedInnerRadius() + cut.thickness() / 2) / allowance;
				var expected = (originalWallArea - removedWallArea) * cut.thickness() + (allowance * 100 - stripCutArea) * cut.thickness() * factor;
				near(solid.volume(1e-10), expected, "independent mapped cut volume");
				var bounds = cut.evaluate(state).bounds(); var native = solid.bounds();
				near(bounds.minX, native.get_min().get_x(), "mapped lower X bound"); near(bounds.maxX, native.get_max().get_x(), "mapped upper X bound");
				near(bounds.minY, native.get_min().get_y(), "mapped lower Y bound"); near(bounds.maxY, native.get_max().get_y(), "mapped upper Y bound");
				near(bounds.minZ, native.get_min().get_z(), "mapped lower Z bound"); near(bounds.maxZ, native.get_max().get_z(), "mapped upper Z bound");
				var current = names(solid); if (fraction == 0) identity = current; else check(current == identity, "cut face identities survive folding");
				solid.close();
			}
		}
		var source = SheetDemoParts.p1();
		var endTool = WallOutline.polygon([new Point2(-10, 90), new Point2(200, 90), new Point2(200, 110), new Point2(-10, 110)]);
		var shortened = SheetCutGeometry.apply(source, 1, endTool, "trim-end");
		for (state in [shortened.flat(), shortened.folded()]) {
			var evaluated = shortened.evaluate(state); near(evaluated.bounds().maxY, 90, "bounds use surviving material rather than nominal bend width");
			var solid = SheetSolid.build(shortened, state); near(solid.bounds().get_max().get_y(), 90, "trimmed native end bound"); solid.close();
		}
		documentCuts();
		return checks;
	}
}
