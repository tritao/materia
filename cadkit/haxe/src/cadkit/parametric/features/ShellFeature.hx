package cadkit.parametric.features;

import CadKit;
import cadkit.Shape;
import cadkit.Operation;
import cadkit.modeling.Model;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.SelectionRecipe;

class ShellFeature extends Feature {
	public final source:Feature;
	public final thickness:Parameter;
	public final selection:SelectionRecipe;

	public function new(source:Feature, thickness:Float, selection:SelectionRecipe) {
		super();
		if (selection.kind != "face")
			throw new ParametricError("shell requires a face selection");
		this.source = source;
		this.selection = selection;
		this.thickness = new Parameter(this, "shell.thickness", thickness, -1e300);
	}

	override public function serializationType():String {
		return "shell";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var shape = context.shape(source);
		var faces = selection.resolve(shape);
		var solid:Null<Shape> = null;
		try {
			// Boolean results commonly wrap their single solid in a compound.
			if (shape.kind() != CadKit.ShapeKind.Solid) {
				if (shape.subshapeCount(CadKit.ShapeKind.Solid) != 1)
					throw new ParametricError("shell requires exactly one solid");
				solid = shape.subshape(CadKit.ShapeKind.Solid, 0);
				shape = solid;
			}
			var result = EvaluationResult.fromOperation(new Operation(CadKit.shellOperationChecked(shape.borrowHandle(), Model.refs(faces), thickness.value)));
			if (solid != null)
				solid.close();
			for (face in faces)
				face.close();
			return result;
		} catch (error:Dynamic) {
			if (solid != null)
				solid.close();
			for (face in faces)
				face.close();
			throw error;
		}
	}
}
