package cadkit.modeling;

import CadKit;
import cadkit.Shape;
import cadkit.Operation;

class Curve extends Model {
	public function new(shape:Shape, operation:Null<Operation> = null) {
		super(shape, operation);
		try {
			if (shape.subshapeCount(CadKit.ShapeKind.Face) > 0 || (!isEmpty() && shape.subshapeCount(CadKit.ShapeKind.Edge) == 0))
				throw "curve requires edges or wires";
		} catch (error:Dynamic) {
			close();
			throw error;
		}
	}

	public static function fromOperation(operation:Operation):Curve {
		try {
			return new Curve(operation.resultShape(), operation);
		} catch (error:Dynamic) {
			operation.close();
			throw error;
		}
	}

	public static function line(start:Vector, end:Vector):Curve {
		return new Curve(Shape.fromOwnedHandle(CadKit.lineChecked(start.native(), end.native())));
	}

	public static function arc(start:Vector, middle:Vector, end:Vector):Curve {
		return new Curve(Shape.fromOwnedHandle(CadKit.arcChecked(start.native(), middle.native(), end.native())));
	}

	public static function circle(radius:Float, ?plane:Plane):Curve {
		if (plane == null)
			plane = Plane.XY();
		return new Curve(Shape.fromOwnedHandle(CadKit.circleChecked(plane.origin.native(), plane.normal.native(), radius)));
	}

	public static function spline(points:Array<Vector>):Curve {
		var values:Array<CadKit.Vec3> = [];
		for (p in points)
			values.push(p.native());
		return new Curve(Shape.fromOwnedHandle(CadKit.splineChecked(values)));
	}

	public static function polyline(points:Array<Vector>, closed:Bool = false):Curve {
		var values:Array<CadKit.Vec3> = [];
		for (p in points)
			values.push(p.native());
		return new Curve(Shape.fromOwnedHandle(CadKit.polylineChecked(values, closed ? 1 : 0)));
	}

	public static function wire(curves:Array<Curve>):Curve {
		var edges:Array<Shape> = [];
		try {
			for (curve in curves) {
				for (i in 0...curve.shape.subshapeCount(CadKit.ShapeKind.Edge))
					edges.push(curve.shape.subshape(CadKit.ShapeKind.Edge, i));
			}
			var result = new Curve(Shape.fromOwnedHandle(CadKit.wireChecked(Model.refs(edges))));
			for (edge in edges)
				edge.close();
			return result;
		} catch (error:Dynamic) {
			for (edge in edges)
				edge.close();
			throw error;
		}
	}

	public function asWire():Curve {
		if (shape.kind() == CadKit.ShapeKind.Wire)
			return new Curve(shape.cloneShape());
		return wire([this]);
	}

	public function placed(location:Location):Curve {
		return fromOperation(location.applyOperation(shape));
	}

	public function offset(distance:Float):Curve {
		var wire = asWire();
		try {
			var result = fromOperation(new Operation(CadKit.wireOffsetOperationChecked(wire.shape.borrowHandle(), distance)));
			wire.close();
			return result;
		} catch (error:Dynamic) {
			wire.close();
			throw error;
		}
	}

	public function project(target:Model, direction:Vector):Curve {
		return new Curve(Shape.fromOwnedHandle(CadKit.projectChecked(shape.borrowHandle(), target.shape.borrowHandle(), direction.native())));
	}
}
