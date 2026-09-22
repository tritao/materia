package cadkit.parametric.features;

import cadkit.parametric.ParameterKind;
import CadKit;
import cadkit.Shape;
import cadkit.modeling.Model;
import cadkit.modeling.Vector;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParametricError;

/** Centered copies along one arbitrary direction and an optional second axis. */
class LinearPatternFeature extends Feature {
	public final source:Feature;
	public final direction:Vector;
	public final secondDirection:Null<Vector>;
	public final count:Parameter;
	public final spacing:Parameter;
	public final secondCount:Parameter;
	public final secondSpacing:Parameter;

	public function new(source:Feature, countValue:Float, spacingValue:Float, direction:Vector,
		secondCountValue:Float = 1, secondSpacingValue:Float = 1, ?secondDirection:Vector) {
		super();
		this.source = source;
		this.direction = normalizedDirection(direction, "linear pattern direction");
		this.secondDirection = secondDirection == null ? null : normalizedDirection(secondDirection, "linear pattern second direction");
		if (this.secondDirection == null && secondCountValue != 1)
			throw new ParametricError("linear pattern needs a second direction when its second count exceeds one");
		if (this.secondDirection != null && Math.abs(this.direction.dot(this.secondDirection)) > 1 - 1e-10)
			throw new ParametricError("linear pattern directions must not be parallel");
		count = new Parameter(this, "linear.count", countValue, 0, true, 10000, ParameterKind.Count);
		spacing = new Parameter(this, "linear.spacing", spacingValue, 0, false, 1e300, ParameterKind.Length);
		secondCount = new Parameter(this, "linear.secondCount", secondCountValue, 0, true, 10000, ParameterKind.Count);
		secondSpacing = new Parameter(this, "linear.secondSpacing", secondSpacingValue, 0, false, 1e300, ParameterKind.Length);
	}

	override public function serializationType():String {
		return "linear-pattern";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var copies:Array<Shape> = [];
		var firstCount = Std.int(count.value);
		var otherCount = Std.int(secondCount.value);
		var otherDirection:Vector = secondDirection == null ? new Vector() : secondDirection;
		try {
			for (other in 0...otherCount) {
				var secondOffset = (other - (otherCount - 1) / 2) * secondSpacing.value;
				for (index in 0...firstCount) {
					var firstOffset = (index - (firstCount - 1) / 2) * spacing.value;
					var translation = direction.scale(firstOffset).add(otherDirection.scale(secondOffset));
					copies.push(context.shape(source).translate(translation.native()));
				}
			}
			var result = EvaluationResult.fromShape(Shape.fromOwnedHandle(CadKit.compoundChecked(Model.refs(copies))));
			for (copy in copies)
				copy.close();
			return result;
		} catch (error:Dynamic) {
			for (copy in copies)
				copy.close();
			throw error;
		}
	}

	private static function normalizedDirection(value:Vector, label:String):Vector {
		if (value == null || value.length() <= 1e-12)
			throw new ParametricError(label + " must be finite and nonzero");
		return value.normalized();
	}
}
