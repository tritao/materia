package cadkit.parametric;

import cadkit.Operation;
import cadkit.Shape;
import cadkit.parametric.Feature;
import cadkit.parametric.ParametricError;

/** Reads committed shapes and staged results during recompute. */
class EvaluationContext {
	private final document:Document;
	private final staged:Map<Int, Shape>;
	private final stagedOperations:Map<Int, Null<Operation>>;

	public function new(document:Document) {
		this.document = document;
		this.staged = new Map<Int, Shape>();
		this.stagedOperations = new Map<Int, Null<Operation>>();
	}

	public function stage(feature:Feature, result:EvaluationResult):Void {
		staged.set(feature.id.toInt(), result.getShape());
		stagedOperations.set(feature.id.toInt(), result.getOperation());
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
	public function owner():Document return document;

	/** Returns staged history when available, otherwise the committed history. */
	public function operation(feature:Feature):Null<Operation> {
		if (stagedOperations.exists(feature.id.toInt()))
			return stagedOperations.get(feature.id.toInt());
		return feature.provenance;
	}
}
