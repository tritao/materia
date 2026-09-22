package cadkit.sketch;

import cadkit.modeling.Plane;
import cadkit.sketch.SketchSolver;

/** Authored constraint system. Successful solutions are snapshots and never mutate this model. */
class ConstrainedSketch {
	public final plane:Plane;
	public final units:String;
	public final settings:SolverSettings;
	private final pointValues:Array<SketchPoint>;
	private final entityValues:Array<SketchEntity>;
	private final constraintValues:Array<SketchConstraint>;
	private var committed:Null<SolvedSketch>;

	public function new(?plane:Plane, units:String = "mm", ?settings:SolverSettings) {
		this.plane = plane == null ? Plane.XY() : plane;
		this.units = units;
		this.settings = settings == null ? new SolverSettings() : settings;
		pointValues = []; entityValues = []; constraintValues = []; committed = null;
	}

	public function addPoint(point:SketchPoint):ConstrainedSketch { requireUnique(point.id); pointValues.push(point); return this; }
	public function addEntity(entity:SketchEntity):ConstrainedSketch { requireUnique(entity.id); entityValues.push(entity); return this; }
	public function addConstraint(constraint:SketchConstraint):ConstrainedSketch { requireUnique(constraint.id); constraintValues.push(constraint); return this; }
	public function points():Array<SketchPoint> return pointValues.copy();
	public function entities():Array<SketchEntity> return entityValues.copy();
	public function constraints():Array<SketchConstraint> return constraintValues.copy();
	public function lastSolution():Null<SolvedSketch> return committed;
	public function copy():ConstrainedSketch {
		var result = new ConstrainedSketch(plane, units, settings);
		for (point in pointValues)
			result.addPoint(point);
		for (entity in entityValues)
			result.addEntity(entity);
		for (constraint in constraintValues)
			result.addConstraint(constraint);
		return result;
	}

	public function removeConstraint(id:String):Void {
		removeById(constraintValues, id, "constraint");
	}

	public function removeEntity(id:String):Void {
		for (constraint in constraintValues)
			if (constraint.first == id || constraint.second == id || constraint.third == id)
				throw "entity is referenced by constraint: " + constraint.id;
		removeById(entityValues, id, "entity");
	}

	public function removePoint(id:String):Void {
		for (entity in entityValues)
			if (entity.first == id || entity.second == id)
				throw "point is referenced by entity: " + entity.id;
		for (constraint in constraintValues)
			if (constraint.first == id || constraint.second == id || constraint.third == id)
				throw "point is referenced by constraint: " + constraint.id;
		removeById(pointValues, id, "point");
	}

	public function replaceConstraint(value:SketchConstraint):Void {
		replaceById(constraintValues, value.id, value, "constraint");
	}

	public function replaceEntity(value:SketchEntity):Void {
		replaceById(entityValues, value.id, value, "entity");
	}

	private function removeById<T>(values:Array<T>, id:String, kind:String):Void {
		for (index in 0...values.length) {
			if (Reflect.field(values[index], "id") == id) {
				values.splice(index, 1);
				return;
			}
		}
		throw "unknown sketch " + kind + ": " + id;
	}

	private function replaceById<T>(values:Array<T>, id:String, value:T, kind:String):Void {
		for (index in 0...values.length) {
			if (Reflect.field(values[index], "id") == id) {
				values[index] = value;
				return;
			}
		}
		throw "unknown sketch " + kind + ": " + id;
	}

	private function requireUnique(id:String):Void {
		if (id == null || StringTools.trim(id) == "") throw "sketch IDs must be nonempty";
		for (p in pointValues) if (p.id == id) throw "duplicate sketch ID: " + id;
		for (e in entityValues) if (e.id == id) throw "duplicate sketch ID: " + id;
		for (c in constraintValues) if (c.id == id) throw "duplicate sketch ID: " + id;
	}

	/** Solve into a temporary snapshot. A failure throws and leaves lastSolution unchanged. */
	public function solve(?seed:SolvedSketch):SolvedSketch {
		var candidate = SketchSolver.solve(this, seed == null ? committed : seed);
		committed = candidate;
		return candidate;
	}
}
