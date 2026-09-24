package cadkit.parametric;

import cadkit.Operation;
import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;
import cadkit.parametric.Parameter;
import cadkit.parametric.TopologyReference;
import cadkit.parametric.TopologyReferenceUpdate;

/** Base class for a document DAG node. */
class Feature {
	public var id(default, null):FeatureId;
	public var dirty:Bool;
	public var active(default, null):Bool;
	public var shape(default, null):Null<Shape>;
	public var provenance(default, null):Null<Operation>;
	public var document(default, null):Null<Document>;
	public var ownerToken(default, null):Int;

	private final topologyReferences:Array<TopologyReference>;
	private final scalarParameters:Array<Parameter>;

	public function new() {
		id = new FeatureId(0);
		dirty = true;
		active = true;
		shape = null;
		provenance = null;
		document = null;
		ownerToken = 0;
		topologyReferences = [];
		scalarParameters = [];
	}

	/** Active scalar slots; stable names are codec binding keys. */
	public function registerParameter(parameter:Parameter):Void {
		for (existing in scalarParameters)
			if (existing.name == parameter.name)
				throw new ParametricError("duplicate feature parameter: " + parameter.name);
		scalarParameters.push(parameter);
		parameter.setRegistered(true);
	}

	public function unregisterParameter(parameter:Parameter):Void {
		for (index in 0...scalarParameters.length) {
			if (scalarParameters[index] == parameter) {
				scalarParameters.splice(index, 1);
				parameter.setRegistered(false);
				return;
			}
		}
	}

	public function parameter(name:String):Parameter {
		for (value in scalarParameters)
			if (value.name == name)
				return value;
		throw new ParametricError("unknown feature parameter: " + name);
	}

	public function dependencies():Array<FeatureId> {
		return [];
	}

	public function serializationType():String {
		return "";
	}

	public function dependencyFeatures():Array<Feature> {
		return [];
	}

	public function datumDependencies():Array<String>
		return [];

	public function elementDependencies():Array<String>
		return [];

	/** Persistent document elements referenced by this feature outside the feature DAG. */
	public function elementReferences():Array<ElementReference>
		return [];

	public function evaluate(context:EvaluationContext):EvaluationResult {
		throw new ParametricError("feature has no evaluator");
	}

	/**
	 * Commit feature-owned staged state after preparation succeeds. Overrides must be
	 * assignment-only publication hooks; all fallible work belongs in evaluate().
	 */
	public function commitEvaluation():Void {}

	/** Discard feature-owned staged state when any feature in the recompute fails. */
	public function discardEvaluation():Void {}

	public function currentShape():Null<Shape> {
		return shape;
	}

	public function attach(owner:Document, featureId:FeatureId, documentToken:Int):Void {
		if (document != null)
			throw new ParametricError("feature is already attached to a document");
		document = owner;
		ownerToken = documentToken;
		id = featureId;
		dirty = true;
		onAttached();
	}

	/** Hook for features that materialize document-owned references on attach. */
	public function onAttached():Void {}

	public function parameterChanged(parameter:Parameter, oldValue:Float):Void {
		if (document != null)
			document.recordParameterChange(parameter, oldValue);
		markDirty();
	}

	public function parameterRestored(parameter:Parameter):Void {
		markDirty();
	}

	public function markDirty():Void {
		dirty = true;
		if (document != null)
			document.invalidate(this);
	}

	public function restoreActive(value:Bool):Void {
		active = value;
		if (value)
			markDirty();
	}

	public function registerTopologyReference(reference:TopologyReference):Void {
		if (document == null)
			throw new ParametricError("topology references require an attached feature");
		topologyReferences.push(reference);
	}

	public function remapTopologyReferences():TopologyRemapReport {
		var report = new TopologyRemapReport();
		for (reference in topologyReferences)
			report.add(reference.remap());
		return report;
	}

	/** Resolve this feature's references against staged producer results without publishing them. */
	public function prepareTopologyRemaps(staged:Map<Int, EvaluationResult>):Array<TopologyReferenceUpdate> {
		var updates:Array<TopologyReferenceUpdate> = [];
		try {
			for (reference in topologyReferences) {
				var producer = reference.remapTargetFeature();
				var result = staged.get(producer.id.toInt());
				updates.push(reference.prepareRemap(result == null ? producer.currentShape() : result.shape,
					result == null ? producer.provenance : result.operation));
			}
			return updates;
		} catch (error:Dynamic) {
			for (update in updates)
				try update.dispose() catch (_:Dynamic) {}
			throw error;
		}
	}

	/** Reports reference states changed while a staged evaluation was attempted. */
	public function topologyReferenceReport(previous:Array<Int>):TopologyRemapReport {
		var report = new TopologyRemapReport();
		for (index in 0...topologyReferences.length) {
			var reference = topologyReferences[index];
			if (previous[index] != reference.stateGeneration())
				report.add(reference.state);
		}
		return report;
	}

	public function topologyReferenceCount():Int {
		return topologyReferences.length;
	}

	public function topologyReferenceAt(index:Int):TopologyReference {
		if (index < 0 || index >= topologyReferences.length)
			throw new ParametricError("topology reference index is out of range");
		return topologyReferences[index];
	}

	/** Swap in a prepared result; the document retires previous resources after publication. */
	public function install(result:EvaluationResult):Void {
		shape = result.getShape();
		provenance = result.getOperation();
		result.transferOwnership();
		dirty = false;
	}

	public function close():Void {
		onClose();
		for (reference in topologyReferences)
			reference.close();
		topologyReferences.resize(0);
		if (shape != null)
			shape.close();
		if (provenance != null)
			provenance.close();
		shape = null;
		provenance = null;
	}

	/** Hook for features that own construction-time resources. */
	public function onClose():Void {}
}
