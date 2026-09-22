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

	private function requireUnique(id:String):Void {
		if (id == null || StringTools.trim(id) == "") throw "sketch IDs must be nonempty";
		for (p in pointValues) if (p.id == id) throw "duplicate sketch ID: " + id;
		for (e in entityValues) if (e.id == id) throw "duplicate sketch ID: " + id;
		for (c in constraintValues) if (c.id == id) throw "duplicate sketch ID: " + id;
	}

	/** Solve into a temporary snapshot. A failure throws and leaves lastSolution unchanged. */
	public function solve():SolvedSketch {
		var candidate = SketchSolver.solve(this);
		committed = candidate;
		return candidate;
	}
}
