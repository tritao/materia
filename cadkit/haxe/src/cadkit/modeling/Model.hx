package cadkit.modeling;

import CadKit;
import cadkit.Shape;
import cadkit.Operation;

/** Owns its shape and optional last operation. Inputs to modeling methods are borrowed.
 * Returned models are independent owners and must be closed, or owned by a Scope.
 */
class Model {
	public final shape:Shape;
	public final operation:Null<Operation>;

	public function new(shape:Shape, operation:Null<Operation> = null) {
		this.shape = shape;
		this.operation = operation;
	}

	public function close():Void {
		shape.close();
		if (operation != null)
			operation.close();
	}

	public function valid():Bool {
		return CadKit.shapeValidChecked(shape.borrowHandle()) != 0;
	}

	public function isEmpty():Bool {
		return shape.subshapeCount(CadKit.ShapeKind.Vertex) == 0;
	}

	public function faces():Selection {
		return Selection.from(shape, CadKit.ShapeKind.Face);
	}

	public function edges():Selection {
		return Selection.from(shape, CadKit.ShapeKind.Edge);
	}

	public function vertices():Selection {
		return Selection.from(shape, CadKit.ShapeKind.Vertex);
	}

	public function wires():Selection {
		return Selection.from(shape, CadKit.ShapeKind.Wire);
	}

	public function solids():Selection {
		return Selection.from(shape, CadKit.ShapeKind.Solid);
	}

	public function generated(kind:CadKit.ShapeKind):Selection {
		return Selection.history(operation, kind, CadKit.HistoryRelation.Generated);
	}

	public function modified(kind:CadKit.ShapeKind):Selection {
		return Selection.history(operation, kind, CadKit.HistoryRelation.Modified);
	}

	public function exportStep(path:String):Void {
		shape.exportStep(path);
	}

	public static function refs(shapes:Array<Shape>):Array<CadKit.ShapeRef> {
		var result:Array<CadKit.ShapeRef> = [];
		for (shape in shapes) {
			var r = new CadKit.ShapeRef();
			r.set_shape(shape.borrowHandle());
			result.push(r);
		}
		return result;
	}

	public static function alignment(size:Float, align:Align):Float {
		return switch (align) {
			case Min: 0;
			case Center: -size / 2;
			case Max: -size;
		};
	}

	public static function positive(value:Float):Void {
		if (!Math.isFinite(value) || value <= 0)
			throw "dimension must be finite and positive";
	}
}
