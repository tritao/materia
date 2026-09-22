package cadkit.modeling;

import CadKit;
import cadkit.Shape;
import cadkit.Operation;

/** Planar faces with a workplane, including disconnected regions and holes. */
class Sketch extends Model {
	public final plane:Plane;

	public function new(shape:Shape, plane:Plane, operation:Null<Operation> = null) {
		super(shape, operation);
		this.plane = plane;
		try {
			var count = shape.subshapeCount(CadKit.ShapeKind.Face);
			if (count == 0 && !isEmpty())
				throw "sketch requires planar faces";
			if (shape.subshapeCount(CadKit.ShapeKind.Solid) > 0)
				throw "sketch cannot contain solids";
			for (i in 0...count) {
				var face = shape.subshape(CadKit.ShapeKind.Face, i);
				try {
					if (face.surfaceKind() != CadKit.SurfaceKind.Plane
						|| Math.abs(Math.abs(Vector.fromNative(face.faceNormal()).dot(plane.normal)) - 1) > 1e-10
						|| Math.abs(Vector.fromNative(face.center()).subtract(plane.origin).dot(plane.normal)) > 1e-7)
						throw "sketch faces must lie on its workplane";
					face.close();
				} catch (error:Dynamic) {
					face.close();
					throw error;
				}
			}
		} catch (error:Dynamic) {
			close();
			throw error;
		}
	}

	public static function face(outer:Curve, ?holes:Array<Curve>, ?plane:Plane):Sketch {
		if (holes == null)
			holes = [];
		if (plane == null)
			plane = Plane.XY();
		var wires:Array<Curve> = [];
		try {
			var boundary = outer.asWire();
			wires.push(boundary);
			var holeShapes:Array<Shape> = [];
			for (hole in holes) {
				var wire = hole.asWire();
				wires.push(wire);
				holeShapes.push(wire.shape);
			}
			var result = new Sketch(Shape.fromOwnedHandle(CadKit.planarFaceChecked(boundary.shape.borrowHandle(), Model.refs(holeShapes))), plane);
			for (wire in wires)
				wire.close();
			return result;
		} catch (error:Dynamic) {
			for (wire in wires)
				wire.close();
			throw error;
		}
	}

	public static function polygon(points:Array<Vector>, ?plane:Plane):Sketch {
		if (plane == null)
			plane = Plane.XY();
		var world:Array<Vector> = [];
		for (point in points) {
			if (Math.abs(point.z) > 1e-10)
				throw "sketch polygon points must lie in local XY";
			world.push(plane.toWorld(point));
		}
		var wire = Curve.polyline(world, true);
		try {
			var result = face(wire, [], plane);
			wire.close();
			return result;
		} catch (error:Dynamic) {
			wire.close();
			throw error;
		}
	}

	public static function rectangle(width:Float, height:Float, ?plane:Plane, alignX:Align = Center, alignY:Align = Center):Sketch {
		Model.positive(width);
		Model.positive(height);
		var x = Model.alignment(width, alignX);
		var y = Model.alignment(height, alignY);
		return polygon([
			new Vector(x, y),
			new Vector(x + width, y),
			new Vector(x + width, y + height),
			new Vector(x, y + height)
		], plane);
	}

	public static function circle(radius:Float, ?plane:Plane):Sketch {
		if (plane == null)
			plane = Plane.XY();
		var curve = Curve.circle(radius, plane);
		try {
			var result = face(curve, [], plane);
			curve.close();
			return result;
		} catch (error:Dynamic) {
			curve.close();
			throw error;
		}
	}

	/** Horizontal capsule, length includes the round ends. */
	public static function slot(length:Float, width:Float, ?plane:Plane):Sketch {
		Model.positive(width);
		Model.positive(length);
		if (length < width)
			throw "slot length must be at least its width";
		if (length == width)
			return circle(width / 2, plane);
		if (plane == null)
			plane = Plane.XY();
		var r = width / 2;
		var c = (length - width) / 2;
		var edges:Array<Curve> = [];
		var wire:Null<Curve> = null;
		try {
			edges.push(Curve.line(plane.toWorld(new Vector(-c, -r)), plane.toWorld(new Vector(c, -r))));
			edges.push(Curve.arc(plane.toWorld(new Vector(c, -r)), plane.toWorld(new Vector(c + r, 0)), plane.toWorld(new Vector(c, r))));
			edges.push(Curve.line(plane.toWorld(new Vector(c, r)), plane.toWorld(new Vector(-c, r))));
			edges.push(Curve.arc(plane.toWorld(new Vector(-c, r)), plane.toWorld(new Vector(-c - r, 0)), plane.toWorld(new Vector(-c, -r))));
			wire = Curve.wire(edges);
			var result = face(wire, [], plane);
			wire.close();
			for (edge in edges)
				edge.close();
			return result;
		} catch (error:Dynamic) {
			if (wire != null)
				wire.close();
			for (edge in edges)
				edge.close();
			throw error;
		}
	}

	public function placed(location:Location):Sketch {
		var world = location.compose(plane.location()).plane;
		var op = location.applyOperation(shape);
		try {
			return new Sketch(op.resultShape(), world, op);
		} catch (error:Dynamic) {
			op.close();
			throw error;
		}
	}

	public function combine(other:Sketch, mode:Mode = Add):Sketch {
		if (Math.abs(Math.abs(plane.normal.dot(other.plane.normal)) - 1) > 1e-10
			|| Math.abs(other.plane.origin.subtract(plane.origin).dot(plane.normal)) > 1e-7)
			throw "sketches must be coplanar";
		var op = switch (mode) {
			case Add: shape.fuseOperation(other.shape);
			case Subtract: shape.cutOperation(other.shape);
			case Intersect: shape.commonOperation(other.shape);
		};
		try {
			return new Sketch(op.resultShape(), plane, op);
		} catch (error:Dynamic) {
			op.close();
			throw error;
		}
	}

	public function extrude(amount:Float):Part {
		return Part.fromOperation(shape.extrudeOperation(plane.normal.scale(amount).native()));
	}

	public function revolve(axis:Axis, angle:Float = 6.283185307179586):Part {
		return Part.fromOperation(shape.revolveOperation(axis.origin.native(), axis.direction.native(), angle));
	}

	public function sweep(path:Curve):Part {
		var wire = path.asWire();
		try {
			var result = Part.fromOperation(new Operation(CadKit.sweepOperationChecked(shape.borrowHandle(), wire.shape.borrowHandle())));
			wire.close();
			return result;
		} catch (error:Dynamic) {
			wire.close();
			throw error;
		}
	}
}
