package cadkit.parametric.features;

import cadkit.Geometry;
import cadkit.Operation;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;

/** Sweeps a face or wire feature by a translation vector. */
class ExtrudeFeature extends Feature {
	public final source:Feature;
	public final x:Parameter;
	public final y:Parameter;
	public final z:Parameter;

	public function new(source:Feature, xValue:Float, yValue:Float, zValue:Float) {
		super();
		this.source = source;
		// Components may be zero; the native operation rejects only a zero vector.
		x = new Parameter(this, "extrude.x", xValue, -1.0e300);
		y = new Parameter(this, "extrude.y", yValue, -1.0e300);
		z = new Parameter(this, "extrude.z", zValue, -1.0e300);
	}

	override public function serializationType():String {
		return "extrude";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var operation:Operation = context.shape(source).extrudeOperation(
			Geometry.vec3(x.value, y.value, z.value));
		return EvaluationResult.fromOperation(operation);
	}
}
