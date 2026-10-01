package cadkit.modeling;

import CadKit;
import cadkit.Shape;
import cadkit.PhysicalProperties;
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

	/** A cylinder from z=0 with the given height. */
	public static function cylinder(radius:Float, height:Float):Part
		return new Part(Shape.cylinder(radius, height));

	/** A +Z cylinder spanning z0..z1. */
	public static function cylinderSpan(radius:Float, z0:Float, z1:Float, x:Float = 0, y:Float = 0):Part {
		if (!(radius > 0) || !(z1 > z0)) throw "Cylinder needs a positive radius and length";
		var base = new Part(Shape.cylinder(radius, z1 - z0));
		try {
			var result = base.translated(new Vector(x, y, z0));
			base.close();
			return result;
		} catch (error:Dynamic) {
			base.close();
			throw error;
		}
	}

	/** Revolve a closed (radius, z) half-section about +Z. */
	public static function revolve(section:Array<{r:Float, z:Float}>, angle:Float = Math.PI * 2):Part {
		var sketch = Sketch.polygon([for (point in section) new Vector(point.r, point.z)], Plane.XZ());
		try {
			var result = sketch.revolve(Axis.Z(), angle);
			sketch.close();
			return result;
		} catch (error:Dynamic) {
			sketch.close();
			throw error;
		}
	}

	/** Cylinder along +Y, spanning `y0..y1`. */
	public static function cylinderAlongY(radius:Float, y0:Float, y1:Float, x:Float = 0, z:Float = 0):Part {
		if (!(radius > 0) || !(y1 > y0)) throw "Cylinder needs a positive radius and length";
		var base = Part.cylinder(radius, y1 - y0);
		try {
			var placement = Location.translation(new Vector(x, y0, z)).compose(Location.rotation(Axis.X(), -Math.PI / 2));
			var result = base.placed(placement);
			base.close();
			return result;
		} catch (error:Dynamic) {
			base.close();
			throw error;
		}
	}

	/** Cylinder starting at `origin` along a finite direction. */
	public static function cylinderAlong(radius:Float, origin:Vector, direction:Vector, length:Float):Part {
		if (!(radius > 0) || !(length > 0) || origin == null || direction == null)
			throw "Cylinder needs a positive radius and length";
		var axis = direction.normalized();
		var reference = Math.abs(axis.z) < 0.9 ? Vector.Z() : Vector.X();
		var xDirection = reference.subtract(axis.scale(reference.dot(axis))).normalized();
		var base = Part.cylinder(radius, length);
		try {
			var result = base.placed(new Location(new Plane(origin, xDirection, axis)));
			base.close();
			return result;
		} catch (error:Dynamic) {
			base.close();
			throw error;
		}
	}

	/** Extrude a borrowed local XY polygon from `z0` to `z1`. */
	public static function prism(points:Array<Vector>, z0:Float, z1:Float):Part {
		var sketch = Sketch.polygon(points, Plane.XY().offset(z0));
		try {
			var result = sketch.extrude(z1 - z0);
			sketch.close();
			return result;
		} catch (error:Dynamic) {
			sketch.close();
			throw error;
		}
	}

	/** Fuse borrowed parts into an independently owned result. */
	public static function fuseAll(parts:Array<Part>):Part {
		if (parts.length == 0) throw "Fuse needs at least one part";
		var result = new Part(parts[0].shape.cloneShape());
		try {
			for (i in 1...parts.length) {
				var next = result.combine(parts[i]);
				result.close();
				result = next;
			}
			return result;
		} catch (error:Dynamic) {
			result.close();
			throw error;
		}
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

	/** Subtract borrowed tools, preserving this part and every tool. */
	/**
		This part with every face, edge and vertex name prefixed by `tag:` (plans/TOPOLOGICAL_NAMING.md): name the bodies
		a part is built from (`plate`, `bore`) so their faces keep telling apart, and stay findable, as parameters change.
	**/
	public function named(tag:String):Part {
		return new Part(shape.stamped(tag, []));
	}

	public function subtractAll(tools:Array<Part>):Part {
		if (tools.length == 0) return new Part(shape.cloneShape());
		var tool = Part.fuseAll(tools);
		try {
			var result = subtract(tool);
			tool.close();
			return result;
		} catch (error:Dynamic) {
			tool.close();
			throw error;
		}
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

	public function massProperties():PhysicalProperties {
		return shape.massProperties();
	}

	public function solidCount():Int {
		return shape.subshapeCount(CadKit.ShapeKind.Solid);
	}
}
