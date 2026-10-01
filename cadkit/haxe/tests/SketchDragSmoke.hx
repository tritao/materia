import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/**
	Soft drag (plan C5.3): a dragged point comes as close to its target as
	the constraints allow, and every constraint still holds.
*/
class SketchDragSmoke {
	public static function run():Void {
		// A rectangle free to stretch upwards: its corner follows the cursor up, but stays on its vertical edge.
		var stretchy = rectangle(false);
		stretchy.solve();
		var pulled = stretchy.drag(["p2" => [12.0, 8.0]]);
		near(pulled.x("p2"), 10, "the corner stays on the vertical edge");
		near(pulled.y("p2"), 8, "and rises to the cursor");
		near(pulled.y("p3"), 8, "the top edge stays horizontal");
		near(pulled.x("p1"), 10, "the width holds");
		check(!pulled.diagnostic.diagnosed, "a drag step is not diagnosed");
		check(stretchy.points()[2].y != 8, "the authored sketch is unchanged");

		// A fully constrained rectangle does not move.
		var rigid = rectangle(true);
		rigid.solve();
		var held = rigid.drag(["p2" => [14.0, 9.0]]);
		near(held.x("p2"), 10, "a fixed rectangle holds its corner (x)");
		near(held.y("p2"), 5, "a fixed rectangle holds its corner (y)");

		// A line of fixed length about a fixed end swings towards the cursor; a loose point goes to it.
		var arm = new ConstrainedSketch();
		arm.addPoint(new SketchPoint("o", 0, 0)).addPoint(new SketchPoint("tip", 10, 0)).addPoint(new SketchPoint("loose", 30, 30))
			.addEntity(SketchEntity.line("bar", "o", "tip"))
			.addConstraint(SketchConstraint.fixed("pin", "o")).addConstraint(SketchConstraint.distance("length", "o", "tip", 10));
		arm.solve();
		var swung = arm;
		var step = arm.solve();
		for (k in 1...5) {
			var angle = Math.PI / 2 * k / 4;
			step = swung.drag(["tip" => [20 * Math.cos(angle), 20 * Math.sin(angle)], "loose" => [3.0, 4.0]], step);
		}
		near(step.x("tip"), 0, "the tip swings round to the cursor's direction (x)");
		near(step.y("tip"), 10, "on its circle (y)");
		near(step.x("loose"), 3, "an unconstrained point goes to the cursor (x)");
		near(step.y("loose"), 4, "an unconstrained point goes to the cursor (y)");
		var released = arm.solve(step);
		check(released.diagnostic.diagnosed, "the solve on release diagnoses again");
	}

	static function rectangle(withHeight:Bool):ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		sketch.addPoint(new SketchPoint("p0", 0, 0)).addPoint(new SketchPoint("p1", 10, 0))
			.addPoint(new SketchPoint("p2", 10, 5)).addPoint(new SketchPoint("p3", 0, 5));
		sketch.addEntity(SketchEntity.line("bottom", "p0", "p1")).addEntity(SketchEntity.line("right", "p1", "p2"))
			.addEntity(SketchEntity.line("top", "p2", "p3")).addEntity(SketchEntity.line("left", "p3", "p0"));
		sketch.addConstraint(SketchConstraint.fixed("origin", "p0"))
			.addConstraint(SketchConstraint.horizontal("h0", "bottom")).addConstraint(SketchConstraint.horizontal("h1", "top"))
			.addConstraint(SketchConstraint.vertical("v0", "right")).addConstraint(SketchConstraint.vertical("v1", "left"))
			.addConstraint(SketchConstraint.distance("width", "p0", "p1", 10));
		if (withHeight) sketch.addConstraint(SketchConstraint.distance("height", "p1", "p2", 5));
		return sketch;
	}

	static function near(actual:Float, expected:Float, label:String):Void {
		if (!(Math.abs(actual - expected) <= 1e-6)) throw '$label: expected $expected, got $actual';
	}

	static function check(value:Bool, label:String):Void {
		if (!value) throw label;
	}
}
