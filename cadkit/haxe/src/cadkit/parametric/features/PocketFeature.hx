package cadkit.parametric.features;

import CadKit;
import cadkit.Operation;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParametricError;

/** Subtracts an attached planar profile from a target solid. */
class PocketFeature extends Feature {
	public final target:Feature;
	public final profile:Feature;
	public final mode:String;
	public final depth:Parameter;

	public function new(target:Feature, profile:Feature, mode:String, depthValue:Float = 1) {
		super();
		if (mode != "blind" && mode != "through-all")
			throw new ParametricError("pocket mode must be blind or through-all");
		this.target = target;
		this.profile = profile;
		this.mode = mode;
		depth = new Parameter(this, "pocket.depth", depthValue, 0);
	}

	public static function blind(target:Feature, profile:Feature, depth:Float):PocketFeature {
		return new PocketFeature(target, profile, "blind", depth);
	}

	public static function throughAll(target:Feature, profile:Feature):PocketFeature {
		return new PocketFeature(target, profile, "through-all", 1);
	}

	override public function serializationType():String {
		return "pocket";
	}

	override public function dependencies():Array<FeatureId> {
		return [target.id, profile.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [target, profile];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var targetShape = context.shape(target);
		var profileShape = context.shape(profile);
		var normal = profileNormal(profileShape);
		var span = projectedSpan(targetShape, normal);
		var margin = Math.max(1e-7 * Math.max(1, span), 1e-7);
		var length = mode == "blind" ? depth.value + margin : span + 2 * margin;
		var start = profileShape.translate(normal.scale(margin).native());
		var tool:Null<Shape> = null;
		try {
			tool = start.extrude(normal.scale(-length).native());
			start.close();
			var operation:Operation = targetShape.cutOperation(tool);
			tool.close();
			return EvaluationResult.fromOperation(operation);
		} catch (error:Dynamic) {
			start.close();
			if (tool != null)
				tool.close();
			throw error;
		}
	}

	private static function profileNormal(shape:Shape):Vector {
		if (shape.kind() == CadKit.ShapeKind.Face)
			return Vector.fromNative(shape.faceNormal()).normalized();
		var faces = shape.faces();
		if (faces.count() == 0)
			throw new ParametricError("pocket profile must contain at least one planar face");
		var first = faces.at(0);
		try {
			var normal = Vector.fromNative(first.normal()).normalized();
			first.close();
			return normal;
		} catch (error:Dynamic) {
			first.close();
			throw error;
		}
	}

	private static function projectedSpan(shape:Shape, direction:Vector):Float {
		var bounds = shape.bounds();
		var minimum = bounds.get_min();
		var maximum = bounds.get_max();
		return Math.abs(direction.x) * (maximum.get_x() - minimum.get_x())
			+ Math.abs(direction.y) * (maximum.get_y() - minimum.get_y())
			+ Math.abs(direction.z) * (maximum.get_z() - minimum.get_z());
	}
}
