package cadkit.parametric.features;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Model;
import cadkit.modeling.Vector;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;

/** Reflects a source across a plane, optionally retaining or fusing the source. */
class MirrorFeature extends Feature {
	public final source:Feature;
	public final planeOrigin:Vector;
	public final planeNormal:Vector;
	public final mode:String;

	public function new(source:Feature, planeOrigin:Vector, planeNormal:Vector, mode:String = "copy") {
		super();
		if (mode != "copy" && mode != "both" && mode != "fuse")
			throw new ParametricError("mirror mode must be copy, both, or fuse");
		this.source = source;
		this.planeOrigin = planeOrigin;
		this.planeNormal = planeNormal.normalized();
		this.mode = mode;
	}

	override public function serializationType():String {
		return "mirror";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var sourceShape = context.shape(source);
		var reflection = sourceShape.mirrorOperation(planeOrigin.native(), planeNormal.native());
		if (mode == "copy")
			return EvaluationResult.fromOperation(reflection);
		var reflected:Null<Shape> = null;
		try {
			reflected = reflection.resultShape();
			if (mode == "fuse") {
				var fused = sourceShape.fuseOperation(reflected);
				reflected.close();
				reflection.close();
				return EvaluationResult.fromOperation(fused);
			}
			var compound = Shape.fromOwnedHandle(CadKit.compoundChecked(Model.refs([sourceShape, reflected])));
			reflected.close();
			reflection.close();
			return EvaluationResult.fromShape(compound);
		} catch (error:Dynamic) {
			if (reflected != null)
				reflected.close();
			reflection.close();
			throw error;
		}
	}
}
