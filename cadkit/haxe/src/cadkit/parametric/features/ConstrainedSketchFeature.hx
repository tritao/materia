package cadkit.parametric.features;

import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchProfile;
import cadkit.sketch.SolveDiagnostic;
import cadkit.sketch.SolvedSketch;
import cadkit.sketch.SketchSolveError;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchEntity;
import cadkit.parametric.features.ConstrainedSketchChange;
import cadkit.parametric.FeatureId;
import cadkit.parametric.SelectionRecipe;
import cadkit.modeling.Vector;
import cadkit.sketch.FaceWorkplane;

/** Serializable document feature backed by an authored constrained sketch. */
class ConstrainedSketchFeature extends Feature {
	private var authored:ConstrainedSketch;
	private final dimensionSlots:Map<String, Parameter>;
	private final dimensionKinds:Map<String, String>;
	private final retiredDimensionSlots:Map<String, Parameter>;
	private final retiredDimensionKinds:Map<String, String>;
	public var lastDiagnostic(default,null):Null<SolveDiagnostic>;
	public var lastAttemptDiagnostic(default,null):Null<SolveDiagnostic>;
	public var lastSolveSeconds(default,null):Float;
	public var lastProfileBuildSeconds(default,null):Float;
	private var committedSolution:Null<SolvedSketch>;
	private var pendingSolution:Null<SolvedSketch>;
	public final support:Null<Feature>;
	public final supportSelection:Null<SelectionRecipe>;
	public final supportXDirection:Null<Vector>;
	public final supportOffset:Float;
	public final supportFlipped:Bool;

	public function new(authored:ConstrainedSketch, ?support:Feature, ?supportSelection:SelectionRecipe, ?supportXDirection:Vector,
		supportOffset:Float = 0, supportFlipped:Bool = false) {
		super();
		this.authored = authored.copy();
		dimensionSlots = new Map();
		dimensionKinds = new Map();
		retiredDimensionSlots = new Map();
		retiredDimensionKinds = new Map();
		lastDiagnostic = null;
		lastAttemptDiagnostic = null;
		lastSolveSeconds = 0;
		lastProfileBuildSeconds = 0;
		committedSolution = null;
		pendingSolution = null;
		this.support = support;
		this.supportSelection = supportSelection;
		this.supportXDirection = supportXDirection;
		this.supportOffset = supportOffset;
		this.supportFlipped = supportFlipped;
		if ((support == null) != (supportSelection == null) || (support != null && supportXDirection == null))
			throw "attached constrained sketches require support, selection, and X direction together";
		for (constraint in authored.constraints())
			if (isDimensional(constraint.kind)) {
				dimensionSlots.set(constraint.id, new Parameter(this, "constraint." + constraint.id, constraint.value,
					constraint.kind == "angle" ? -1e300 : 0, false, 1e300,
					constraint.kind == "angle" ? ParameterKind.Angle : ParameterKind.Length));
				dimensionKinds.set(constraint.id, constraint.kind);
			}
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

	public function replacePoint(point:SketchPoint):Void {
		edit(function(sketch) sketch.replacePoint(point));
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

		var removedIds:Array<String> = [];
		for (id in dimensionSlots.keys()) {
			if (required.exists(id))
				continue;
			var slot = dimensionSlots.get(id);
			removedIds.push(id);
			if (document != null)
				for (named in document.namedParameters())
					if (named.contains(slot))
						throw new ParametricError("cannot remove dimensional constraint " + id + " while named parameter "
							+ named.name + " is bound; unbind the parameter first");
		}
		for (id in required.keys()) {
			var constraint = required.get(id);
			var existingKind = dimensionKinds.get(id);
			if (existingKind == null)
				existingKind = retiredDimensionKinds.get(id);
			if (existingKind != null && existingKind != constraint.kind)
				throw new ParametricError("cannot change the kind of a dimensional constraint while preserving its parameter: " + id);
		}

		for (id in removedIds) {
			var slot = dimensionSlots.get(id);
			retiredDimensionSlots.set(id, slot);
			retiredDimensionKinds.set(id, dimensionKinds.get(id));
			dimensionSlots.remove(id);
			dimensionKinds.remove(id);
			unregisterParameter(slot);
		}

		for (id in required.keys()) {
			var constraint = required.get(id);
			if (!dimensionSlots.exists(id)) {
				var slot = retiredDimensionSlots.get(id);
				if (slot != null) {
					retiredDimensionSlots.remove(id);
					retiredDimensionKinds.remove(id);
					registerParameter(slot);
					restoreDimensionValue(slot, constraint.value);
				} else {
					slot = new Parameter(this, "constraint." + id, constraint.value,
						constraint.kind == "angle" ? -1e300 : 0, false, 1e300,
						constraint.kind == "angle" ? ParameterKind.Angle : ParameterKind.Length);
				}
				dimensionSlots.set(id, slot);
				dimensionKinds.set(id, constraint.kind);
			} else {
				restoreDimensionValue(dimensionSlots.get(id), constraint.value);
			}
		}
	}

	private function restoreDimensionValue(slot:Parameter, value:Float):Void {
		if (document != null) {
			for (named in document.namedParameters()) {
				if (!named.contains(slot))
					continue;
				for (binding in named.bindings())
					binding.restore(value);
				return;
			}
		}
		slot.restore(value);
	}

	override public function parameterChanged(parameter:Parameter, oldValue:Float):Void {
		synchronizeAuthoredDimension(parameter);
		if (document != null)
			document.recordParameterChange(parameter, oldValue);
		markDirty();
	}

	override public function parameterRestored(parameter:Parameter):Void {
		synchronizeAuthoredDimension(parameter);
		markDirty();
	}

	private function synchronizeAuthoredDimension(parameter:Parameter):Void {
		for (id in dimensionSlots.keys()) {
			if (dimensionSlots.get(id) != parameter)
				continue;
			for (constraint in authored.constraints()) {
				if (constraint.id == id) {
					authored.replaceConstraint(SketchConstraint.raw(constraint.id, constraint.kind, constraint.first,
						constraint.second, constraint.third, parameter.value));
					return;
				}
			}
		}
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

	public function solvedSketch():Null<SolvedSketch> {
		return committedSolution;
	}

	override public function serializationType():String {
		return "constrained-sketch";
	}

	override public function dependencies():Array<FeatureId> {
		return support == null ? [] : [support.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return support == null ? [] : [support];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var plane = authored.plane;
		if (support != null)
			plane = FaceWorkplane.resolve(context.shape(support), supportSelection, supportXDirection, supportOffset, supportFlipped);
		var candidate = new ConstrainedSketch(plane, authored.units, authored.settings);
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
		var solveStarted = Sys.time();
		try {
			solved = candidate.solve(committedSolution, function() return context.isCancelled());
		} catch (error:Dynamic) {
			lastSolveSeconds = Sys.time() - solveStarted;
			context.recordSketchSolve(lastSolveSeconds);
			if (Std.isOfType(error, SketchSolveError)) {
				var solveError:SketchSolveError = cast error;
				lastAttemptDiagnostic = solveError.diagnostic;
			}
			throw error;
		}
		lastSolveSeconds = Sys.time() - solveStarted;
		context.recordSketchSolve(lastSolveSeconds);
		context.checkCancelled();
		var profileStarted = Sys.time();
		var profile:cadkit.modeling.Sketch;
		try {
			profile = SketchProfile.build(candidate, solved);
		} catch (error:Dynamic) {
			lastProfileBuildSeconds = Sys.time() - profileStarted;
			context.recordSketchProfile(lastProfileBuildSeconds);
			throw error;
		}
		lastProfileBuildSeconds = Sys.time() - profileStarted;
		context.recordSketchProfile(lastProfileBuildSeconds);
		try {
			context.checkCancelled();
			var result = EvaluationResult.fromShape(profile.shape.cloneShape());
			profile.close();
			pendingSolution = solved;
			return result;
		} catch (error:Dynamic) {
			profile.close();
			throw error;
		}
	}

	override public function commitEvaluation():Void {
		if (pendingSolution == null)
			return;
		committedSolution = pendingSolution;
		lastDiagnostic = pendingSolution.diagnostic;
		lastAttemptDiagnostic = null;
		pendingSolution = null;
	}

	override public function discardEvaluation():Void {
		pendingSolution = null;
	}

	private static function isDimensional(kind:String):Bool {
		return kind == "distance" || kind == "radius" || kind == "angle";
	}
}
