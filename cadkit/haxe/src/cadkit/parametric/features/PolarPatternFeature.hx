package cadkit.parametric.features;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Axis;
import cadkit.modeling.Location;
import cadkit.modeling.Model;
import cadkit.modeling.Vector;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParametricError;

/** Copies a source around an arbitrary axis without duplicating full-circle endpoints. */
class PolarPatternFeature extends Feature {
	public final source:Feature;
	public final axisOrigin:Vector;
	public final axisDirection:Vector;
	public final radialDirection:Vector;
	public final orientInstances:Bool;
	public final count:Parameter;
	public final radius:Parameter;
	public final startAngle:Parameter;
	public final angularSpan:Parameter;

	public function new(source:Feature, countValue:Float, radiusValue:Float, angularSpanValue:Float,
		axisOrigin:Vector, axisDirection:Vector, radialDirection:Vector, orientInstances:Bool = true, startAngleValue:Float = 0) {
		super();
		this.source = source;
		this.axisOrigin = axisOrigin;
		this.axisDirection = normalized(axisDirection, "polar pattern axis");
		var projected = radialDirection.subtract(this.axisDirection.scale(radialDirection.dot(this.axisDirection)));
		this.radialDirection = normalized(projected, "polar pattern radial direction");
		this.orientInstances = orientInstances;
		count = new Parameter(this, "polar.count", countValue, 0, true, 10000);
		radius = new Parameter(this, "polar.radius", radiusValue, 0);
		startAngle = new Parameter(this, "polar.startAngle", startAngleValue, -1e300);
		angularSpan = new Parameter(this, "polar.angularSpan", angularSpanValue, -1e300);
	}

	override public function serializationType():String {
		return "polar-pattern";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var copies:Array<Shape> = [];
		var instanceCount = Std.int(count.value);
		var span = angularSpan.value;
		var fullCircle = Math.abs(Math.abs(span) - 2 * Math.PI) < 1e-10;
		var divisor = fullCircle ? instanceCount : (instanceCount > 1 ? instanceCount - 1 : 1);
		var axis = new Axis(axisOrigin, axisDirection);
		try {
			for (index in 0...instanceCount) {
				var angle = startAngle.value + span * index / divisor;
				var rotatedRadial = rotateVector(radialDirection, axisDirection, angle).scale(radius.value);
				var placement = Location.translation(rotatedRadial);
				if (orientInstances)
					placement = placement.compose(Location.rotation(axis, angle));
				copies.push(placement.apply(context.shape(source)));
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

	private static function normalized(value:Vector, label:String):Vector {
		if (value == null || value.length() <= 1e-12)
			throw new ParametricError(label + " must be finite and nonzero");
		return value.normalized();
	}

	private static function rotateVector(value:Vector, axis:Vector, angle:Float):Vector {
		return value.scale(Math.cos(angle))
			.add(axis.cross(value).scale(Math.sin(angle)))
			.add(axis.scale(axis.dot(value) * (1 - Math.cos(angle))));
	}
}
