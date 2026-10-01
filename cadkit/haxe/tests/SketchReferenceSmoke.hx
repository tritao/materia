import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchSolveError;

/**
	Reference (measured) dimensions (plan C5.2): measured on the solved
	geometry, never solved for, so they neither over-constrain a sketch nor
	enter its diagnosis, and they are not feature parameters.
*/
class SketchReferenceSmoke {
	public static function run():Void {
		var sketch = rectangle();
		sketch.addConstraint(SketchConstraint.distance("diagonal", "p0", "p2", 1).asReference())
			.addConstraint(SketchConstraint.angle("corner", "bottom", "right", 0).asReference());
		sketch.addPoint(new SketchPoint("c", 30, 0)).addEntity(SketchEntity.circle("ring", "c", 2.5))
			.addConstraint(SketchConstraint.fixed("pin-c", "c")).addConstraint(SketchConstraint.radius("ring-r", "ring", 3))
			.addConstraint(SketchConstraint.radius("ring-measured", "ring", 1).asReference());
		var solved = sketch.solve();
		check(solved.diagnostic.status == "fully-constrained", 'a reference diagonal does not over-constrain: ${solved.diagnostic.status}');
		near(solved.measured("diagonal"), Math.sqrt(125), "the diagonal is measured");
		near(solved.measured("corner"), Math.PI / 2, "the corner angle is measured, signed from the first line to the second");
		near(solved.measured("ring-measured"), 3, "a reference radius reads the solved radius");

		sketch.replaceConstraint(SketchConstraint.distance("width", "p0", "p1", 12));
		near(sketch.solve(solved).measured("diagonal"), Math.sqrt(169), "editing the width updates the measured diagonal");

		var invalid = rectangle();
		invalid.addConstraint(SketchConstraint.horizontal("level", "bottom").asReference());
		var refused = false;
		try invalid.solve() catch (error:SketchSolveError) refused = error.diagnostic.status == "invalid";
		check(refused, "only distances, radii and angles can be references");

		// In a document: a reference is no parameter; a driving dimension made a reference loses its parameter, undoably.
		var document = new Document();
		var feature = document.add(new ConstrainedSketchFeature(sketch));
		check(throws(() -> feature.dimension("diagonal")), "a reference dimension is not a parameter");
		feature.replaceConstraint(SketchConstraint.distance("width", "p0", "p1", 12).asReference());
		check(throws(() -> feature.dimension("width")), "a dimension made a reference loses its parameter");
		check(document.undo() && !throws(() -> feature.dimension("width")), "undo makes it driving again");
		document.recompute();
		var reloaded = DocumentCodec.decode(DocumentCodec.encode(document));
		var reloadedFeature:ConstrainedSketchFeature = cast reloaded.featureAt(0);
		var diagonal = [for (constraint in reloadedFeature.sketch().constraints()) if (constraint.id == "diagonal") constraint];
		check(diagonal.length == 1 && diagonal[0].reference, "a reference dimension survives a save and reload");
		reloaded.close();
		document.close();
	}

	static function rectangle():ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		sketch.addPoint(new SketchPoint("p0", 0, 0)).addPoint(new SketchPoint("p1", 10.3, 0.2))
			.addPoint(new SketchPoint("p2", 9.8, 5.1)).addPoint(new SketchPoint("p3", 0.1, 4.8));
		sketch.addEntity(SketchEntity.line("bottom", "p0", "p1")).addEntity(SketchEntity.line("right", "p1", "p2"))
			.addEntity(SketchEntity.line("top", "p2", "p3")).addEntity(SketchEntity.line("left", "p3", "p0"));
		sketch.addConstraint(SketchConstraint.fixed("origin", "p0"))
			.addConstraint(SketchConstraint.horizontal("h0", "bottom")).addConstraint(SketchConstraint.horizontal("h1", "top"))
			.addConstraint(SketchConstraint.vertical("v0", "right")).addConstraint(SketchConstraint.vertical("v1", "left"))
			.addConstraint(SketchConstraint.distance("width", "p0", "p1", 10)).addConstraint(SketchConstraint.distance("height", "p1", "p2", 5));
		return sketch;
	}

	static function throws(action:()->Dynamic):Bool {
		try action() catch (_:Dynamic) return true;
		return false;
	}

	static function near(actual:Float, expected:Float, label:String):Void {
		if (!(Math.abs(actual - expected) <= 1e-6)) throw '$label: expected $expected, got $actual';
	}

	static function check(value:Bool, label:String):Void {
		if (!value) throw label;
	}
}
