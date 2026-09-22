package cadkit.parametric.features;

import cadkit.parametric.ParameterKind;
import CadKit;
import cadkit.Shape;
import cadkit.modeling.Model;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;

/** Centered XY grid of independent copies. Count is fixed; spacing is editable.
 * Produces a compound, not a boolean union. Copies do not claim history across regenerations.
 */
class GridFeature extends Feature {
	public final source:Feature;
	public final columns:Int;
	public final rows:Int;
	public final spacingX:Parameter;
	public final spacingY:Parameter;

	public function new(source:Feature, columns:Int, rows:Int, spacingX:Float, spacingY:Float) {
		super();
		if (columns < 1 || rows < 1)
			throw new ParametricError("grid counts must be positive");
		this.source = source;
		this.columns = columns;
		this.rows = rows;
		this.spacingX = new Parameter(this, "grid.spacingX", spacingX, 0, false, 1e300, ParameterKind.Length);
		this.spacingY = new Parameter(this, "grid.spacingY", spacingY, 0, false, 1e300, ParameterKind.Length);
	}

	override public function serializationType():String {
		return "grid";
	}

	override public function dependencies():Array<FeatureId> {
		return [source.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [source];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		var copies:Array<Shape> = [];
		try {
			for (row in 0...rows)
				for (column in 0...columns)
					copies.push(context.shape(source)
						.translate(new Vector((column - (columns - 1) / 2) * spacingX.value, (row - (rows - 1) / 2) * spacingY.value).native()));
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
}
