import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchSolver;
import cadkit.solve.JacobianCheck;

/** Every sketch constraint kind's analytic Jacobian agrees with central differences. */
class SketchJacobianSmoke {
	public static function run():Void {
		checkSketch("lines", lines());
		checkSketch("circles", circles());
		checkSketch("lines at 1e-6", scaled(lines(), 1e-6));
		checkSketch("circles at 1e6", scaled(circles(), 1e6));
	}

	/** The authored pose and ten nearby ones (±2% of the sketch size per coordinate). */
	static function checkSketch(label:String, sketch:ConstrainedSketch):Void {
		var probe = SketchSolver.probe(sketch);
		var size = 0.0;
		for (value in probe.variables) size = Math.max(size, Math.abs(value));
		var seed = 777;
		for (trial in 0...11) {
			var x = [for (value in probe.variables) {
				seed = (seed * 1103515245 + 12345) & 0x7fffffff;
				trial == 0 ? value : value + (seed / 0x7fffffff - 0.5) * 0.04 * size;
			}];
			var result = JacobianCheck.compare(probe.residuals, probe.jacobian, x, 1e-5);
			if (!result.passed) throw '$label, pose $trial: $result (row ${result.row} is constraint ${rowOwner(sketch, result.row)})';
		}
	}

	/** Point/line kinds: fixed, coincident, horizontal, vertical, distance, equal, parallel, perpendicular, angle, pointOn, symmetric. */
	static function lines():ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		var coordinates = [[0.3, -0.2], [9.6, 0.4], [10.3, 5.2], [-0.4, 4.7], [3.1, 1.2], [6.8, 7.9], [2.2, 6.1], [7.7, 3.3], [4.9, 2.6]];
		for (i in 0...coordinates.length) sketch.addPoint(new SketchPoint('p$i', coordinates[i][0], coordinates[i][1]));
		sketch.addEntity(SketchEntity.line("l0", "p0", "p1")).addEntity(SketchEntity.line("l1", "p1", "p2"))
			.addEntity(SketchEntity.line("l2", "p2", "p3")).addEntity(SketchEntity.line("l3", "p3", "p0"))
			.addEntity(SketchEntity.line("l4", "p4", "p5"));
		sketch.addConstraint(SketchConstraint.fixed("fixed", "p0"))
			.addConstraint(SketchConstraint.coincident("coincident", "p4", "p8"))
			.addConstraint(SketchConstraint.horizontal("horizontal", "l0"))
			.addConstraint(SketchConstraint.vertical("vertical", "l1"))
			.addConstraint(SketchConstraint.distance("distance", "p0", "p2", 11))
			.addConstraint(SketchConstraint.equal("equal", "l0", "l2"))
			.addConstraint(SketchConstraint.parallel("parallel", "l0", "l2"))
			.addConstraint(SketchConstraint.perpendicular("perpendicular", "l1", "l2"))
			.addConstraint(SketchConstraint.angle("angle", "l3", "l4", 0.9))
			.addConstraint(SketchConstraint.pointOn("pointOn", "p6", "l3"))
			.addConstraint(SketchConstraint.symmetric("symmetric", "p6", "p7", "l4"));
		return sketch;
	}

	/** Circle kinds: radius, equal (circle-circle, line-circle), concentric, pointOn a circle, and every tangency form. */
	static function circles():ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		sketch.addPoint(new SketchPoint("c0", 0, 0)).addPoint(new SketchPoint("c1", 2.9, 0.1)).addPoint(new SketchPoint("c2", 12.1, 0.3))
			.addPoint(new SketchPoint("c3", 0.2, -0.1)).addPoint(new SketchPoint("q0", -8, 5.3)).addPoint(new SketchPoint("q1", 15, 5.1))
			.addPoint(new SketchPoint("q2", 4.1, 3.2)).addPoint(new SketchPoint("q3", 9, -4.4)).addPoint(new SketchPoint("q4", 3.5, 0.9));
		sketch.addEntity(SketchEntity.circle("big", "c0", 5)).addEntity(SketchEntity.circle("inner", "c1", 2))
			.addEntity(SketchEntity.arc("outer", "c2", 4, 0.2, 2.4)).addEntity(SketchEntity.circle("ring", "c3", 7))
			.addEntity(SketchEntity.line("top", "q0", "q1")).addEntity(SketchEntity.line("side", "q3", "q4"));
		sketch.addConstraint(SketchConstraint.radius("radius", "big", 5))
			.addConstraint(SketchConstraint.equal("equal-circles", "inner", "outer"))
			.addConstraint(SketchConstraint.equal("equal-line-circle", "side", "ring"))
			.addConstraint(SketchConstraint.concentric("concentric", "big", "ring"))
			.addConstraint(SketchConstraint.pointOn("pointOn-circle", "q2", "big"))
			.addConstraint(SketchConstraint.tangent("tangent-line-circle", "top", "big"))
			.addConstraint(SketchConstraint.tangent("tangent-circle-line", "outer", "top"))
			.addConstraint(SketchConstraint.tangent("tangent-internal", "big", "inner"))
			.addConstraint(SketchConstraint.tangent("tangent-external", "big", "outer"));
		return sketch;
	}

	static function scaled(source:ConstrainedSketch, factor:Float):ConstrainedSketch {
		var result = new ConstrainedSketch(source.plane, source.units, source.settings);
		for (point in source.points()) result.addPoint(new SketchPoint(point.id, point.x * factor, point.y * factor));
		for (entity in source.entities())
			result.addEntity(switch entity.kind {
				case "circle": SketchEntity.circle(entity.id, entity.first, entity.radius * factor);
				case "arc": SketchEntity.arc(entity.id, entity.first, entity.radius * factor, entity.startAngle, entity.endAngle);
				default: entity;
			});
		for (constraint in source.constraints())
			result.addConstraint(constraint.kind == "distance" || constraint.kind == "radius"
				? SketchConstraint.raw(constraint.id, constraint.kind, constraint.first, constraint.second, constraint.third, constraint.value * factor)
				: constraint);
		return result;
	}

	/** The constraint that emits residual `row` (rows per kind as the solver emits them). */
	static function rowOwner(sketch:ConstrainedSketch, row:Int):String {
		for (constraint in sketch.constraints()) {
			var rows = switch constraint.kind {
				case "fixed", "coincident", "concentric", "angle", "symmetric": 2;
				default: 1;
			};
			if (row < rows) return constraint.id;
			row -= rows;
		}
		return "?";
	}
}
