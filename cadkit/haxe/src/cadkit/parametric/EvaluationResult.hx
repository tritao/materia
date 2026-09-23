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
