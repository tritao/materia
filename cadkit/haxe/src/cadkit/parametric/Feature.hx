package cadkit.parametric;

import cadkit.Operation;
import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;
import cadkit.parametric.Parameter;
import cadkit.parametric.TopologyReference;

/** Base class for a document DAG node. */
class Feature {
	public var id(default, null):FeatureId;
	public var dirty:Bool;
	public var shape(default, null):Null<Shape>;
	public var provenance(default, null):Null<Operation>;
	public var document(default, null):Null<Document>;
	public var ownerToken(default, null):Int;

	private final topologyReferences:Array<TopologyReference>;
	private final scalarParameters:Array<Parameter>;

	public function new() {
		id = new FeatureId(0);
		dirty = true;
		shape = null;
		provenance = null;
		document = null;
		ownerToken = 0;
		topologyReferences = [];
		scalarParameters = [];
	}

	/** Scalar slots registered by Parameter construction; stable names are codec binding keys. */
	public function registerParameter(parameter:Parameter):Void {
		for (existing in scalarParameters)
			if (existing.name == parameter.name)
				throw new ParametricError("duplicate feature parameter: " + parameter.name);
		scalarParameters.push(parameter);
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

	public function evaluate(context:EvaluationContext):EvaluationResult {
		throw new ParametricError("feature has no evaluator");
	}

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

	public function install(result:EvaluationResult):Void {
		if (shape != null)
			shape.close();
		if (provenance != null)
			provenance.close();
		shape = result.getShape();
		provenance = result.getOperation();
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
