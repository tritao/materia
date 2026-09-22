package cadkit.parametric.features;

import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchProfile;
import cadkit.sketch.SolveDiagnostic;
import cadkit.sketch.SolvedSketch;
import cadkit.sketch.SketchSolveError;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchEntity;
import cadkit.parametric.features.ConstrainedSketchChange;

/** Serializable document feature backed by an authored constrained sketch. */
class ConstrainedSketchFeature extends Feature {
	private var authored:ConstrainedSketch;
	private final dimensionSlots:Map<String, Parameter>;
	public var lastDiagnostic(default,null):Null<SolveDiagnostic>;
	public var lastAttemptDiagnostic(default,null):Null<SolveDiagnostic>;
	private var committedSolution:Null<SolvedSketch>;

	public function new(authored:ConstrainedSketch) {
		super();
		this.authored = authored.copy();
		dimensionSlots = new Map();
		lastDiagnostic = null;
		lastAttemptDiagnostic = null;
		committedSolution = null;
		for (constraint in authored.constraints())
			if (isDimensional(constraint.kind))
				dimensionSlots.set(constraint.id,
					new Parameter(this, "constraint." + constraint.id, constraint.value, constraint.kind == "angle" ? -1e300 : 0));
	}

	public function sketch():ConstrainedSketch {
		return authored.copy();
	}

	public function addPoint(point:SketchPoint):Void {
		edit(function(sketch) sketch.addPoint(point));
	}

	public function addEntity(entity:SketchEntity):Void {
		edit(function(sketch) sketch.addEntity(entity));
	}

	public function addConstraint(constraint:SketchConstraint):Void {
		edit(function(sketch) sketch.addConstraint(constraint));
	}

	public function replaceEntity(entity:SketchEntity):Void {
		edit(function(sketch) sketch.replaceEntity(entity));
	}

	public function replaceConstraint(constraint:SketchConstraint):Void {
		edit(function(sketch) sketch.replaceConstraint(constraint));
	}

	public function removePoint(id:String):Void {
		edit(function(sketch) sketch.removePoint(id));
	}

	public function removeEntity(id:String):Void {
		edit(function(sketch) sketch.removeEntity(id));
	}

	public function removeConstraint(id:String):Void {
		edit(function(sketch) sketch.removeConstraint(id));
	}

	private function edit(change:ConstrainedSketch->Void):Void {
		if (document == null)
			throw "constrained sketch edits require an attached document feature";
		var before = authored.copy();
		var after = authored.copy();
		change(after);
		validateReferences(after);
		restoreSketch(after);
		document.recordDocumentChange(new ConstrainedSketchChange(this, before, after.copy()));
	}

	public function restoreSketch(snapshot:ConstrainedSketch):Void {
		var next = snapshot.copy();
		validateReferences(next);
		reconcileDimensionSlots(next);
		authored = next;
		markDirty();
	}

	private function reconcileDimensionSlots(next:ConstrainedSketch):Void {
		var required:Map<String, SketchConstraint> = new Map();
		for (constraint in next.constraints())
			if (isDimensional(constraint.kind))
				required.set(constraint.id, constraint);
		var removed:Array<String> = [];
		for (id in dimensionSlots.keys())
			if (!required.exists(id))
				removed.push(id);
		for (id in removed) {
			var slot = dimensionSlots.get(id);
			if (isBound(slot))
				throw "cannot remove a dimensional constraint while it is bound: " + id;
			unregisterParameter(slot);
			dimensionSlots.remove(id);
		}
		for (id in required.keys()) {
			var constraint = required.get(id);
			if (!dimensionSlots.exists(id))
				dimensionSlots.set(id,
					new Parameter(this, "constraint." + id, constraint.value, constraint.kind == "angle" ? -1e300 : 0));
		}
	}

	private function isBound(slot:Parameter):Bool {
		if (document == null)
			return false;
		for (named in document.namedParameters())
			if (named.contains(slot))
				return true;
		return false;
	}

	private static function validateReferences(sketch:ConstrainedSketch):Void {
		var pointIds:Map<String, Bool> = new Map();
		var entityIds:Map<String, Bool> = new Map();
		for (point in sketch.points())
			pointIds.set(point.id, true);
		for (entity in sketch.entities()) {
			if (!pointIds.exists(entity.first) || (entity.kind == "line" && (entity.second == null || !pointIds.exists(entity.second))))
				throw "entity references a missing point: " + entity.id;
			entityIds.set(entity.id, true);
		}
		for (constraint in sketch.constraints()) {
			var firstIsPoint = constraint.kind == "fixed" || constraint.kind == "coincident" || constraint.kind == "distance"
				|| constraint.kind == "pointOn" || constraint.kind == "symmetric";
			if (firstIsPoint ? !pointIds.exists(constraint.first) : !entityIds.exists(constraint.first))
				throw "constraint references a missing object: " + constraint.id;
			if (constraint.second != null) {
				var secondIsPoint = constraint.kind == "coincident" || constraint.kind == "distance" || constraint.kind == "symmetric";
				if (secondIsPoint ? !pointIds.exists(constraint.second) : !entityIds.exists(constraint.second))
					throw "constraint references a missing object: " + constraint.id;
			}
			if (constraint.third != null && !entityIds.exists(constraint.third))
				throw "constraint references a missing object: " + constraint.id;
		}
	}

	public function dimension(constraintId:String):Parameter {
		var value = dimensionSlots.get(constraintId);
		if (value == null)
			throw "constraint is not dimensional: " + constraintId;
		return value;
	}

	override public function serializationType():String {
		return "constrained-sketch";
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var candidate = new ConstrainedSketch(authored.plane, authored.units, authored.settings);
		for (point in authored.points())
			candidate.addPoint(point);
		for (entity in authored.entities())
			candidate.addEntity(entity);
		for (constraint in authored.constraints()) {
			var slot = dimensionSlots.get(constraint.id);
			var value = slot == null ? constraint.value : slot.value;
			candidate.addConstraint(SketchConstraint.raw(constraint.id, constraint.kind, constraint.first, constraint.second,
				constraint.third, value));
		}
		var solved:SolvedSketch;
		try {
			solved = candidate.solve(committedSolution);
		} catch (error:SketchSolveError) {
			lastAttemptDiagnostic = error.diagnostic;
			throw error;
		}
		var profile = SketchProfile.build(candidate, solved);
		try {
			var result = EvaluationResult.fromShape(profile.shape.cloneShape());
			profile.close();
			committedSolution = solved;
			lastDiagnostic = solved.diagnostic;
			lastAttemptDiagnostic = null;
			return result;
		} catch (error:Dynamic) {
			profile.close();
			throw error;
		}
	}

	private static function isDimensional(kind:String):Bool {
		return kind == "distance" || kind == "radius" || kind == "angle";
	}
}
