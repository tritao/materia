package cadkit.parametric.features;

import CadKit;
import cadkit.Edge;
import cadkit.Operation;
import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.TopologyReference;

/** Applies a constant fillet to a source solid or its selected edges. */
class FilletFeature extends Feature {
	public final source:Feature;
	public final radius:Parameter;
	public final edgeReferences:Array<TopologyReference>;
	private var pendingEdges:Null<Array<Shape>>;
	private var pendingFingerprints:Null<Array<TopologyFingerprint>>;

	public function new(
		source:Feature,
		radiusValue:Float,
		?selectedEdges:Array<Edge>,
		?savedFingerprints:Array<TopologyFingerprint>) {
		super();
		if (selectedEdges != null && savedFingerprints != null)
			throw new ParametricError("fillet selection cannot be both live and serialized");
		if (selectedEdges != null && selectedEdges.length == 0)
			throw new ParametricError("fillet selection must not be empty");
		if (savedFingerprints != null && savedFingerprints.length == 0)
			throw new ParametricError("fillet selection must not be empty");
		this.source = source;
		radius = new Parameter(this, "fillet.radius", radiusValue, 0.0);
		edgeReferences = [];
		pendingEdges = null;
		if (selectedEdges != null) {
			var clonedEdges:Array<Shape> = [];
			try {
				for (edge in selectedEdges) {
					if (edge == null)
						throw new ParametricError("fillet selection contains a null edge");
					var topology = edge.cloneShape();
					if (topology.kind() != CadKit.ShapeKind.Edge) {
						topology.close();
						throw new ParametricError("fillet selection contains a non-edge shape");
					}
					clonedEdges.push(topology);
				}
			} catch (error:Dynamic) {
				for (topology in clonedEdges)
					topology.close();
				throw error;
			}
			pendingEdges = clonedEdges;
		}
		pendingFingerprints = savedFingerprints;
	}

	override public function onAttached():Void {
		if (pendingEdges != null) {
			for (topology in pendingEdges)
				edgeReferences.push(TopologyReference.fromShape(this, topology, source));
			pendingEdges = null;
		} else if (pendingFingerprints != null) {
			for (fingerprint in pendingFingerprints)
				edgeReferences.push(TopologyReference.fromFingerprint(
					this, CadKit.ShapeKind.Edge, fingerprint, source));
			pendingFingerprints = null;
		}
	}

	override public function serializationType():String {
		return "fillet";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var sourceShape = context.shape(source);
		var operation:Operation;
		if (edgeReferences.length == 0) {
			operation = sourceShape.filletOperation(radius.value);
		} else {
			var selected:Array<Edge> = [];
			try {
				for (reference in edgeReferences)
					selected.push(new Edge(reference.resolveFor(
						sourceShape, context.operation(source))));
				operation = sourceShape.filletEdgesOperation(selected, radius.value);
			} catch (error:Dynamic) {
				for (edge in selected)
					edge.close();
				throw error;
			}
			for (edge in selected)
				edge.close();
		}
		return EvaluationResult.fromOperation(operation);
	}

	override public function onClose():Void {
		if (pendingEdges != null) {
			for (topology in pendingEdges)
				topology.close();
			pendingEdges = null;
		}
	}
}
