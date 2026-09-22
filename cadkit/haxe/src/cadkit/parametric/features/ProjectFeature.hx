package cadkit.parametric.features;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;

/** Directional projection; the native projection API does not supply operation history. */
class ProjectFeature extends Feature {
	public final source:Feature;
	public final target:Feature;
	public final x:Parameter;
	public final y:Parameter;
	public final z:Parameter;

	public function new(source:Feature, target:Feature, direction:Vector) {
		super();
		this.source = source;
		this.target = target;
		x = new Parameter(this, "project.x", direction.x, -1e300);
		y = new Parameter(this, "project.y", direction.y, -1e300);
		z = new Parameter(this, "project.z", direction.z, -1e300);
	}

	override public function serializationType():String {
		return "project";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id, target.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source, target];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		return EvaluationResult.fromShape(Shape.fromOwnedHandle(CadKit.projectChecked(context.shape(source).borrowHandle(),
			context.shape(target).borrowHandle(), new Vector(x.value, y.value, z.value).native())));
	}
}
