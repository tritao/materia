import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/**
	"Still free" geometry (plan C5.1): a solved sketch names the points and
	entities its constraints leave free to move, from the null space of each
	part's Jacobian.
*/
class SketchFreedomSmoke {
	public static function run():Void {
		var sketch = new ConstrainedSketch();
		rectangle(sketch);
		var fixed = sketch.solve();
		check(fixed.freePoints.length == 0 && fixed.freeEntities.length == 0,
			'a fully constrained rectangle has nothing free: ${fixed.freePoints} ${fixed.freeEntities}');

		// Without its height the rectangle stretches upwards: its top corners and the edges they touch move.
		sketch.removeConstraint("height");
		var stretchy = sketch.solve();
		check(sorted(stretchy.freePoints) == "p2,p3", 'only the top corners are free: ${stretchy.freePoints}');
		check(sorted(stretchy.freeEntities) == "left,right,top", 'and the edges they touch: ${stretchy.freeEntities}');

		// A circle whose center is fixed but radius is not, and a point no constraint touches.
		sketch.addPoint(new SketchPoint("c", 40, 0)).addEntity(SketchEntity.circle("ring", "c", 3))
			.addConstraint(SketchConstraint.fixed("pin-c", "c")).addPoint(new SketchPoint("loose", 60, 5));
		var mixed = sketch.solve();
		check(mixed.freePoints.indexOf("c") < 0 && mixed.freeEntities.indexOf("ring") >= 0, "a fixed center with a free radius: the circle is free, its center is not");
		check(mixed.freePoints.indexOf("loose") >= 0, "an unconstrained point is free");

		// A drag (no diagnosis) keeps the last answer; the solve on release finds it again.
		var dragged = sketch.solve(mixed, null, false);
		check(sorted(dragged.freePoints) == sorted(mixed.freePoints), 'a drag keeps what is free: ${dragged.freePoints}');
		sketch.addConstraint(SketchConstraint.radius("ring-r", "ring", 3));
		var settled = sketch.solve(dragged);
		check(settled.freeEntities.indexOf("ring") < 0, "a radius fixes the circle");
	}

	static function rectangle(sketch:ConstrainedSketch):Void {
		sketch.addPoint(new SketchPoint("p0", 0, 0)).addPoint(new SketchPoint("p1", 10.3, 0.2))
			.addPoint(new SketchPoint("p2", 9.8, 5.1)).addPoint(new SketchPoint("p3", 0.1, 4.8));
		sketch.addEntity(SketchEntity.line("bottom", "p0", "p1")).addEntity(SketchEntity.line("right", "p1", "p2"))
			.addEntity(SketchEntity.line("top", "p2", "p3")).addEntity(SketchEntity.line("left", "p3", "p0"));
		sketch.addConstraint(SketchConstraint.fixed("origin", "p0"))
			.addConstraint(SketchConstraint.horizontal("h0", "bottom")).addConstraint(SketchConstraint.horizontal("h1", "top"))
			.addConstraint(SketchConstraint.vertical("v0", "right")).addConstraint(SketchConstraint.vertical("v1", "left"))
			.addConstraint(SketchConstraint.distance("width", "p0", "p1", 10)).addConstraint(SketchConstraint.distance("height", "p1", "p2", 5));
	}

	static function sorted(values:Array<String>):String {
		var copy = values.copy();
		copy.sort(Reflect.compare);
		return copy.join(",");
	}

	static function check(value:Bool, label:String):Void {
		if (!value) throw label;
	}
}
