package cadkit.modeling;

import CadKit;
import cadkit.Shape;
import cadkit.Edge;
import cadkit.Operation;

/** Solid results may contain zero, one, or several solids. */
class Part extends Model {
	public function new(shape:Shape, operation:Null<Operation> = null) {
		super(shape, operation);
		try {
			if (!isEmpty() && solidCount() == 0)
				throw "part requires solids";
		} catch (error:Dynamic) {
			close();
			throw error;
		}
	}

	public static function fromOperation(operation:Operation):Part {
		try {
			return new Part(operation.resultShape(), operation);
		} catch (error:Dynamic) {
			operation.close();
			throw error;
		}
	}

	public static function box(width:Float, depth:Float, height:Float, alignX:Align = Center, alignY:Align = Center, alignZ:Align = Min):Part {
		var shape = Shape.box(width, depth, height);
		try {
			var placed = shape.translate(new Vector(Model.alignment(width, alignX), Model.alignment(depth, alignY), Model.alignment(height, alignZ)).native());
			shape.close();
			return new Part(placed);
		} catch (error:Dynamic) {
			shape.close();
			throw error;
		}
	}

	public static function cylinder(radius:Float, height:Float):Part {
		return new Part(Shape.cylinder(radius, height));
	}

	public static function sphere(radius:Float):Part {
		return new Part(Shape.sphere(radius));
	}

	public static function loft(sections:Array<Curve>, ruled:Bool = false):Part {
		var wires:Array<Curve> = [];
		var shapes:Array<Shape> = [];
		try {
			for (section in sections) {
				var wire = section.asWire();
				wires.push(wire);
				shapes.push(wire.shape);
			}
			var result = fromOperation(new Operation(CadKit.loftOperationChecked(Model.refs(shapes), 1, ruled ? 1 : 0)));
			for (wire in wires)
				wire.close();
			return result;
		} catch (error:Dynamic) {
			for (wire in wires)
				wire.close();
			throw error;
		}
	}

	public function placed(location:Location):Part {
		return fromOperation(location.applyOperation(shape));
	}

	public function translated(delta:Vector):Part {
		return fromOperation(shape.translateOperation(delta.native()));
	}

	public function combine(other:Part, mode:Mode = Add):Part {
		return fromOperation(switch (mode) {
			case Add: shape.fuseOperation(other.shape);
			case Subtract: shape.cutOperation(other.shape);
			case Intersect: shape.commonOperation(other.shape);
		});
	}

	public function subtract(other:Part):Part {
		return combine(other, Subtract);
	}

	public function intersect(other:Part):Part {
		return combine(other, Intersect);
	}

	public function fillet(selection:Selection, radius:Float):Part {
		var edges = selection.edgeValues();
		try {
			var result = fromOperation(shape.filletEdgesOperation(edges, radius));
			for (edge in edges)
				edge.close();
			return result;
		} catch (error:Dynamic) {
			for (edge in edges)
				edge.close();
			throw error;
		}
	}

	public function chamfer(selection:Selection, distance:Float):Part {
		var edges = selection.edgeValues();
		try {
			var result = fromOperation(shape.chamferEdgesOperation(edges, distance));
			for (edge in edges)
				edge.close();
			return result;
		} catch (error:Dynamic) {
			for (edge in edges)
				edge.close();
			throw error;
		}
	}

	public function shell(faces:Selection, thickness:Float):Part {
		return fromOperation(new Operation(CadKit.shellOperationChecked(shape.borrowHandle(), Model.refs(faces.borrowShapes()), thickness)));
	}

	public function volume():Float {
		return shape.volume();
	}

	public function solidCount():Int {
		return shape.subshapeCount(CadKit.ShapeKind.Solid);
	}
}
