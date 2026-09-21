package cadkit.parametric;

import cadkit.Shape;
import cadkit.parametric.Feature;
import cadkit.parametric.ParametricError;

/** Reads committed shapes and staged results during recompute. */
class EvaluationContext {
	private final document:Document;
	private final staged:Map<Int, Shape>;

	public function new(document:Document) {
		this.document = document;
		this.staged = new Map<Int, Shape>();
	}

	public function stage(feature:Feature, shape:Shape):Void {
		staged.set(feature.id.toInt(), shape);
	}

	public function shape(feature:Feature):Shape {
		var stagedShape = staged.get(feature.id.toInt());
		if (stagedShape != null)
			return stagedShape;

		var committed = feature.currentShape();
		if (committed == null)
			throw new ParametricError(
				"feature " + feature.id.toInt() + " has not been evaluated");
		return committed;
	}
}
