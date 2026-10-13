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
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetHem;
import cadkit.sheetmetal.HemType;
import cadkit.sheetmetal.HemSpec;
import cadkit.sheetmetal.SheetHemFeature;
import cadkit.sheetmetal.SheetSide;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.BendRadialCuts;
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
	static function partitionAndConnectivity():Void {
		var source = SheetDemoParts.p1(); var bend = source.bends()[0]; var allowance = source.bendAllowance(bend);
		var center = new Point2(allowance / 2, 50);
		var ring = new WallOutline([Segment.arc(center, 0.6, 0, 2 * Math.PI)], [[Segment.arc(center, 0.2, 0, -2 * Math.PI)]], [["outer"], ["inner"]]);
		var cut = source.cutBend(bend.id, ring, "ring"); var area = 0.0;
		for (region in BendRadialCuts.regions(cut, allowance)) { check(region.loops().length == 1, "radial partition removes sector holes"); area += region.area(); }
		near(area, Math.PI * (0.36 - 0.04), "radial partition preserves exact annular area");
		var refused = false; try { var unused = source.evaluate(source.flat()); }
		catch (error:Dynamic) { var diagnosis = Std.downcast(error, SheetDiagnosis); if (diagnosis != null) refused = diagnosis.code == "sheet.cut-isolated-bend-patch"; }
		check(refused, "material islands cannot masquerade as attached bend patches");
		var divided = SheetDemoParts.p1(); var divider = WallOutline.polygon([new Point2(allowance / 2 - 0.1, -1),
			new Point2(allowance / 2 + 0.1, -1), new Point2(allowance / 2 + 0.1, 101), new Point2(allowance / 2 - 0.1, 101)]);
		var changed = divided.cutBend(bend.id, divider, "divide"); var detached = false;
		try { var unused = divided.evaluate(divided.flat()); }
		catch (error:Dynamic) { var diagnosis = Std.downcast(error, SheetDiagnosis); if (diagnosis != null) detached = diagnosis.code == "sheet.cut-bend-disconnected"; }
		check(detached, "an active bend requires a surviving material bridge");
	}

	static function closedHems():Void {
		for (side in [SheetSide.A, SheetSide.B]) {
			var source = new Sheet(SheetDemoParts.materialA()); var base = source.addWall(WallOutline.rectangle(30, 100));
			var hem = SheetHem.add(source, base.id, new WallEdge(1), HemType.Closed, 10, 0, side);
			var bend = hem.bends()[0]; var allowance = hem.bendAllowance(bend);
			for (radius in [0.3, 2.0]) {
				var tool = new WallOutline([Segment.arc(new Point2(30 + allowance / 2, 50), radius, 0, 2 * Math.PI)], [], [["closed-hole"]]);
				var cut = SheetCutGeometry.apply(hem, base.id, tool, "closed-hole");
				for (fraction in [0.0, 0.5, 1.0]) {
					var state = new SheetState().set(bend.id, bend.angle * fraction); var evaluated = cut.evaluate(state);
					var solid = SheetSolid.build(cut, state);
					check(CadKit.shapeValidChecked(solid.borrowHandle()) != 0, "cut closed hem is valid without an artificial gap");
					near(solid.subshapeCount(CadKit.ShapeKind.Solid), 1, "cut closed hem remains one solid");
					var area = 0.0; for (region in cut.bend(bend.id).materialRegions(allowance)) area += region.area();
					var walls = 0.0; for (wall in cut.walls()) walls += wall.outline.area();
					var zone = evaluated.bend(bend.id); var factor = fraction == 0 ? 1 : Math.abs(zone.angle) * (zone.curvedInnerRadius() + cut.thickness() / 2) / allowance;
					near(solid.volume(1e-10), walls * cut.thickness() + area * cut.thickness() * factor, "closed hem cut volume");
					var named = names(solid); solid.close();
				}
			}
		}
	}

	static function otherHems():Void {
		for (type in [HemType.Open, HemType.Teardrop]) for (side in [SheetSide.A, SheetSide.B]) {
			var source = new Sheet(SheetDemoParts.materialA()); var base = source.addWall(WallOutline.rectangle(30, 100));
			var radius:Null<Float> = type == HemType.Open ? null : 1.0;
			var hem = SheetHem.add(source, base.id, new WallEdge(1), type, 10, type == HemType.Open ? 1 : 0.4, side, radius);
			var bend = hem.bends()[0]; var allowance = hem.bendAllowance(bend);
			var tool = new WallOutline([Segment.arc(new Point2(30 + allowance / 2, 50), 0.3, 0, 2 * Math.PI)], [], [["hole"]]);
			var cut = SheetCutGeometry.apply(hem, base.id, tool, "return-hole"); var identity = "";
			for (fraction in [0.0, 0.5, 1.0]) {
				var state = new SheetState().set(bend.id, bend.angle * fraction); var zone = cut.evaluate(state).bend(bend.id);
				var solid = SheetSolid.build(cut, state); check(CadKit.shapeValidChecked(solid.borrowHandle()) != 0, "cut open/teardrop return is valid");
				var factor = fraction == 0 ? 1 : Math.abs(zone.angle) * (zone.curvedInnerRadius() + cut.thickness() / 2) / allowance;
				near(solid.volume(1e-10), (4000 + (allowance * 100 - Math.PI * 0.09) * factor) * cut.thickness(), "cut open/teardrop volume");
				var current = names(solid); if (fraction == 0) identity = current; else check(current == identity, "cut open/teardrop identities survive folding");
				solid.close();
			}
		}
	}

	static function baseFeature(document:Document):SheetBaseFeature {
		var sketch = new ConstrainedSketch();
		var points = [[0.0, 0.0], [20.0, 0.0], [20.0, 100.0], [0.0, 100.0]];
		for (i in 0...4) {
			var added = sketch.addPoint(new SketchPoint("p" + i, points[i][0], points[i][1]));
			var fixed = sketch.addConstraint(SketchConstraint.fixed("fixed" + i, "p" + i));
			var edge = sketch.addEntity(SketchEntity.line("edge" + i, "p" + i, "p" + ((i + 1) % 4)));
		}
		var profile = document.add(new ConstrainedSketchFeature(sketch));
		var base = document.add(new SheetBaseFeature(profile, SheetDemoParts.materialA())); document.recompute();
		return base;
	}
	static function attachment(base:SheetBaseFeature):Shape {
		var selected:Null<Shape> = null; var baseShape = base.currentShape();
		for (i in 0...baseShape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face = baseShape.subshape(CadKit.ShapeKind.Face, i);
			if (face.surfaceKind() == CadKit.SurfaceKind.Plane && Vector.fromNative(face.faceNormal()).x > 0.999) selected = face;
			else face.close();
		}
		if (selected == null) throw "missing flange attachment";
		return selected;
	}

	static function documentHemCuts():Void {
		var document = new Document(); var base = baseFeature(document); var selected = attachment(base);
		var hem = document.add(new SheetHemFeature(base, [selected], HemSpec.Closed, 10)); selected.close(); document.recompute();
		var graph = hem.sheet(); var bend = graph.bends()[0]; var allowance = graph.bendAllowance(bend);
		var sketch = new ConstrainedSketch(); var center = sketch.addPoint(new SketchPoint("center", 20 + allowance / 2, 50));
		var fixed = sketch.addConstraint(SketchConstraint.fixed("fixed", "center")); var circle = sketch.addEntity(SketchEntity.circle("hole", "center", 0.3));
		var radius = sketch.addConstraint(SketchConstraint.radius("radius", "hole", 0.3));
		var profile = document.add(new ConstrainedSketchFeature(sketch)); var cut = document.add(new SheetCutFeature(hem, profile, 1));
		document.setOutput(cut); document.recompute(); var originalNames = names(cut.currentShape()); var originalVolume = cut.currentShape().volume(1e-10);
		var loaded = DocumentCodec.decode(DocumentCodec.encode(document));
		var restored = Std.downcast(loaded.featureById(cut.id.toInt()), SheetCutFeature); if (restored == null) throw "unexpected saved hem cut";
		check(names(restored.currentShape()) == originalNames, "closed-hem cut codec identities"); near(restored.currentShape().volume(1e-10), originalVolume, "closed-hem cut codec volume");
		base.thickness.set(1.8); document.recompute();
		check(names(cut.currentShape()) == originalNames, "closed-hem cut names survive thickness edit");
		check(cut.sheet().bend(bend.id).cutTools().length == 1, "closed-hem cuts are derived once on thickness rebuild");
		hem.length.set(12); document.recompute(); check(names(cut.currentShape()) == originalNames, "closed-hem cut names survive return-length edit");
		profile.dimension("radius").set(0.35); document.recompute(); check(names(cut.currentShape()) == originalNames, "closed-hem cut names survive hole-radius edit");
		check(document.undo(), "undo closed-hem cut radius"); document.recompute();
		check(CadKit.shapeValidChecked(cut.currentShape().borrowHandle()) != 0, "closed-hem cut rebuild stays valid after undo");
		loaded.close(); document.close();
	}
	static function documentCuts():Void {
		var document = new Document(); var base = baseFeature(document); var selected = attachment(base);
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

	static function main():Void { trace("Bend cut smoke passed (" + run() + " assertions)"); }
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
		partitionAndConnectivity();
		closedHems();
		otherHems();
		documentHemCuts();
		documentCuts();
		return checks;
	}
}
