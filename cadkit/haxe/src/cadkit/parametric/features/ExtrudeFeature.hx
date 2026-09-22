package cadkit.parametric.features;

import cadkit.Geometry;
import cadkit.Operation;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.modeling.Vector;

/** Sweeps a face or wire feature by a translation vector. */
class ExtrudeFeature extends Feature {
	public final source:Feature;
	public final x:Parameter;
	public final y:Parameter;
	public final z:Parameter;
	public final amount:Null<Parameter>;
	public final reversed:Bool;
	public final symmetric:Bool;

	public function new(source:Feature, xValue:Float, yValue:Float, zValue:Float, amountValue:Null<Float> = null,
		reversed:Bool = false, symmetric:Bool = false) {
		super();
		this.source = source;
		// Components may be zero; the native operation rejects only a zero vector.
		x = new Parameter(this, "extrude.x", xValue, -1.0e300);
		y = new Parameter(this, "extrude.y", yValue, -1.0e300);
		z = new Parameter(this, "extrude.z", zValue, -1.0e300);
		amount = amountValue == null ? null : new Parameter(this, "extrude.amount", amountValue, 0);
		this.reversed = reversed;
		this.symmetric = symmetric;
	}

	public static function along(source:Feature, amount:Float, direction:Vector, reversed:Bool = false,
		symmetric:Bool = false):ExtrudeFeature {
		var unit = direction.normalized();
		return new ExtrudeFeature(source, unit.x, unit.y, unit.z, amount, reversed, symmetric);
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
		if (amount == null) {
			var operation:Operation = context.shape(source).extrudeOperation(Geometry.vec3(x.value, y.value, z.value));
			return EvaluationResult.fromOperation(operation);
		}
		var direction = new Vector(x.value, y.value, z.value).normalized();
		var sign = reversed ? -1 : 1;
		var delta = direction.scale(amount.value * sign);
		if (!symmetric)
			return EvaluationResult.fromOperation(context.shape(source).extrudeOperation(delta.native()));
		var centered = context.shape(source).translate(delta.scale(-0.5).native());
		try {
			var result = EvaluationResult.fromOperation(centered.extrudeOperation(delta.native()));
			centered.close();
			return result;
		} catch (error:Dynamic) {
			centered.close();
			throw error;
		}
	}
}
