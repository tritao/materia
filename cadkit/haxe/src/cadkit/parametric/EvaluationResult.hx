package cadkit.parametric;

import cadkit.Operation;
import cadkit.Shape;

/** Staged feature result; ownership transfers to the feature on commit. */
class EvaluationResult {
	public final shape:Shape;
	public final operation:Null<Operation>;
	private var ownsResources:Bool;

	public function new(shape:Shape, operation:Null<Operation>) {
		this.shape = shape;
		this.operation = operation;
		ownsResources = true;
	}

	public function getShape():Shape {
		return shape;
	}

	public function getOperation():Null<Operation> {
		return operation;
	}

	/** Transfer staged resources to the feature that is publishing this result. */
	public function transferOwnership():Void {
		ownsResources = false;
	}

	/**
		This result with its names stamped `tag:` where no input has them (plans/TOPOLOGICAL_NAMING.md, TN-D5): what
		the feature created, as opposed to carried through. This result's shape is released; its operation moves over.
	*/
	public function stamped(tag:String, inputs:Array<Shape>):EvaluationResult {
		var tagged = shape.stamped(tag, inputs);
		ownsResources = false;
		shape.close();
		return new EvaluationResult(tagged, operation);
	}

	public static function fromShape(shape:Shape):EvaluationResult {
		return new EvaluationResult(shape, null);
	}

	public static function fromOperation(operation:Operation):EvaluationResult {
		try {
			return new EvaluationResult(operation.resultShape(), operation);
		} catch (error:Dynamic) {
			operation.close();
			throw error;
		}
	}

	public function dispose():Void {
		if (!ownsResources)
			return;
		ownsResources = false;
		shape.close();
		if (operation != null)
			operation.close();
	}
}
