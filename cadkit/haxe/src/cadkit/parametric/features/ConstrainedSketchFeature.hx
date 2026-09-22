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

/** Serializable document feature backed by an authored constrained sketch. */
class ConstrainedSketchFeature extends Feature {
	private final authored:ConstrainedSketch;
	private final dimensionSlots:Map<String, Parameter>;
	public var lastDiagnostic(default,null):Null<SolveDiagnostic>;
	public var lastAttemptDiagnostic(default,null):Null<SolveDiagnostic>;
	private var committedSolution:Null<SolvedSketch>;

	public function new(authored:ConstrainedSketch) {
		super();
		this.authored = authored;
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
		return authored;
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
