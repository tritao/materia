import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SolvedSketch;

/**
	A solve seeded from a previous solution reuses the parts nothing changed
	and re-solves the rest; the result must equal a cold solve of the same
	sketch.
*/
class SketchIncrementalSmoke {
	public static function run():Void {
		var sketch = new ConstrainedSketch();
		rectangle(sketch, "a.", 0, 10);
		rectangle(sketch, "b.", 30, 10);
		var first = sketch.solve();
		check(first.diagnostic.status == "fully-constrained", "two rectangles solve");
		check([for (_ in first.partCache.keys()) 1].length == 2, "each rectangle is a cached part");

		// A dimension edit re-solves its own rectangle only, and matches a cold solve.
		setValue(sketch, "a.width", 12);
		var edited = sketch.solve(first);
		near(edited.x("a.p1") - edited.x("a.p0"), 12, "the edited rectangle takes its new width");
		near(edited.x("b.p1"), first.x("b.p1"), "the untouched rectangle stays where it was");
		sameAsCold(sketch, edited, "after a dimension edit");

		// Moving a fixed point's authored position changes that part too.
		sketch.replacePoint(new SketchPoint("b.p0", 31, 2));
		var moved = sketch.solve(edited);
		near(moved.x("b.p0"), 31, "a re-authored fixed point moves its rectangle");
		near(moved.y("b.p2"), 7, "the whole rectangle follows it");
		sameAsCold(sketch, moved, "after moving a fixed point");

		// A redundancy found once is still reported when its part is reused.
		sketch.addConstraint(SketchConstraint.distance("a.top-length", "a.p2", "a.p3", 12));
		var redundant = sketch.solve(moved);
		check(redundant.diagnostic.status == "redundant", "adding an implied length is redundant");
		setValue(sketch, "b.width", 11);
		var reused = sketch.solve(redundant);
		check(reused.diagnostic.status == "redundant" && reused.diagnostic.constraintIds.join(",") == "a.top-length,a.v0,a.v1,a.width",
			'the redundant part keeps its diagnosis while the other re-solves: ${reused.diagnostic.constraintIds}');
		sameAsCold(sketch, reused, "after editing the other part");

		// Dragging solves without diagnosis: geometry is exact, the diagnosis is the last one and says so.
		var dragged = reused;
		for (step in 0...5) {
			setValue(sketch, "b.height", 5 + 0.1 * (step + 1));
			dragged = sketch.solve(dragged, null, false);
			near(dragged.y("b.p2") - dragged.y("b.p1"), 5 + 0.1 * (step + 1), 'drag step $step follows the height');
		}
		check(!dragged.diagnostic.diagnosed && dragged.diagnostic.status == "redundant", "a drag reports the previous diagnosis, flagged");
		var released = sketch.solve(dragged);
		check(released.diagnostic.diagnosed, "the solve on release diagnoses again");
		sameAsCold(sketch, released, "after a drag and release");
		// A dragged value that makes the part conflict is still caught while dragging.
		setValue(sketch, "a.top-length", 13);
		var conflicted = false;
		try sketch.solve(released, null, false) catch (error:cadkit.sketch.SketchSolveError) conflicted = error.diagnostic.status == "conflicting";
		check(conflicted, "a conflict during a drag is reported, not hidden behind the old diagnosis");
	}

	static function rectangle(sketch:ConstrainedSketch, prefix:String, dx:Float, width:Float):Void {
		var p = [for (i in 0...4) prefix + "p" + i];
		sketch.addPoint(new SketchPoint(p[0], dx, 0)).addPoint(new SketchPoint(p[1], dx + width + 0.3, 0.2))
			.addPoint(new SketchPoint(p[2], dx + width - 0.2, 5.1)).addPoint(new SketchPoint(p[3], dx + 0.1, 4.8));
		sketch.addEntity(SketchEntity.line(prefix + "bottom", p[0], p[1])).addEntity(SketchEntity.line(prefix + "right", p[1], p[2]))
			.addEntity(SketchEntity.line(prefix + "top", p[2], p[3])).addEntity(SketchEntity.line(prefix + "left", p[3], p[0]));
		sketch.addConstraint(SketchConstraint.fixed(prefix + "origin", p[0]))
			.addConstraint(SketchConstraint.horizontal(prefix + "h0", prefix + "bottom"))
			.addConstraint(SketchConstraint.horizontal(prefix + "h1", prefix + "top"))
			.addConstraint(SketchConstraint.vertical(prefix + "v0", prefix + "right"))
			.addConstraint(SketchConstraint.vertical(prefix + "v1", prefix + "left"))
			.addConstraint(SketchConstraint.distance(prefix + "width", p[0], p[1], width))
			.addConstraint(SketchConstraint.distance(prefix + "height", p[1], p[2], 5));
	}

	static function setValue(sketch:ConstrainedSketch, id:String, value:Float):Void {
		for (c in sketch.constraints())
			if (c.id == id) {
				sketch.replaceConstraint(SketchConstraint.raw(c.id, c.kind, c.first, c.second, c.third, value));
				return;
			}
		throw "unknown constraint " + id;
	}

	/** Every point and the diagnosis agree with a solve that starts from nothing cached. */
	static function sameAsCold(sketch:ConstrainedSketch, solved:SolvedSketch, label:String):Void {
		var cold = sketch.copy().solve();
		check(cold.diagnostic.status == solved.diagnostic.status && cold.diagnostic.degreesOfFreedom == solved.diagnostic.degreesOfFreedom
			&& cold.diagnostic.constraintIds.join(",") == solved.diagnostic.constraintIds.join(","),
			'$label: incremental ${solved.diagnostic.status} ${solved.diagnostic.constraintIds}, cold ${cold.diagnostic.status} ${cold.diagnostic.constraintIds}');
		for (point in sketch.points()) {
			near(solved.x(point.id), cold.x(point.id), '$label: ${point.id}.x');
			near(solved.y(point.id), cold.y(point.id), '$label: ${point.id}.y');
		}
	}

	static function near(value:Float, expected:Float, label:String):Void
		check(Math.abs(value - expected) < 1e-7, '$label: $value, expected $expected');

	static function check(value:Bool, label:String):Void {
		if (!value) throw label;
	}
}
