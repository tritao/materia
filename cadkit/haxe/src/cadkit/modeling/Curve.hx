package cadkit.modeling;

import CadKit;
import cadkit.Shape;
import cadkit.ShapeKey;
import cadkit.Operation;

class Curve extends Model {
  public function new(shape:Shape, operation:Null<Operation> = null) {
    super(shape, operation);
    try {
      if (shape.subshapeCount(CadKit.ShapeKind.Face) > 0
        || (!isEmpty() && shape.subshapeCount(CadKit.ShapeKind.Edge) == 0)) throw "curve requires edges or wires";
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

  /**
		`curve`, a single edge, with that edge named `id` (a sketch entity, a template role), so the faces swept from it
		are named after it (plans/TOPOLOGICAL_NAMING.md). Takes ownership of `curve`.
	**/
  public static function named(curve:Curve, id:String):Curve {
    try {
      var result = new Curve(curve.shape.withElementNames(CadKit.ShapeKind.Edge, [id]));
      curve.close();
      return result;
    } catch (error:Dynamic) {
      curve.close();
      throw error;
    }
  }

  static function vec(v:Vector):String return ShapeKey.number(v.x) + "," + ShapeKey.number(v.y) + "," + ShapeKey.number(v.z);

  static function leaf(shape:Shape, name:String, params:String):Shape {
    shape.key = ShapeKey.leaf("curve-" + name, params);
    shape.gap = null;
    return shape;
  }

  public static function line(start:Vector, end:Vector):Curve {
    return new Curve(leaf(Shape.fromOwnedHandle(CadKit.lineChecked(start.native(), end.native())), "line", vec(start) + "|" + vec(end)));
  }

  public static function arc(start:Vector, middle:Vector, end:Vector):Curve {
    return new Curve(leaf(Shape.fromOwnedHandle(CadKit.arcChecked(start.native(), middle.native(), end.native())), "arc", vec(start) + "|" + vec(middle) + "|" + vec(end)));
  }

  public static function circle(radius:Float, ?plane:Plane):Curve {
    if (plane == null) plane = Plane.XY();
    return new Curve(leaf(Shape.fromOwnedHandle(CadKit.circleChecked(plane.origin.native(), plane.normal.native(), radius)), "circle", vec(plane.origin) + "|" + vec(plane.normal) + "|" + ShapeKey.number(radius)));
  }

  public static function spline(points:Array<Vector>):Curve {
    var values:Array<CadKit.Vec3> = [];
    for (p in points) values.push(p.native());
    // The same points give the same curve: keyed by their exact bits.
    return new Curve(leaf(Shape.fromOwnedHandle(CadKit.splineChecked(values)), "spline", [for (p in points) vec(p)].join(";")));
  }

  public static function polyline(points:Array<Vector>, closed:Bool = false):Curve {
    var values:Array<CadKit.Vec3> = [];
    for (p in points) values.push(p.native());
    return new Curve(leaf(Shape.fromOwnedHandle(CadKit.polylineChecked(values, closed ? 1 : 0)), "polyline", (closed ? "closed" : "open") + "|" + [for (p in points) vec(p)].join(";")));
  }

  public static function wire(curves:Array<Curve>):Curve {
    var edges:Array<Shape> = [];
    try {
      for (curve in curves) {
        for (i in 0...curve.shape.subshapeCount(CadKit.ShapeKind.Edge)) edges.push(curve.shape.subshape(
          CadKit.ShapeKind.Edge,
          i
        ));
      }
      var wireShape = Shape.fromOwnedHandle(CadKit.wireChecked(Model.refs(edges)));
      // The wire is made of every edge of the curves, in order: keyed by the curves' keys when they all have one.
      var wireKey = ShapeKey.op("wire", "", [for (curve in curves) curve.shape]);
      wireShape.key = wireKey;
      wireShape.gap = wireKey == null ? ShapeKey.gapOf("wire", [for (curve in curves) curve.shape]) : null;
      var result = new Curve(wireShape);
      for (edge in edges) edge.close();
      return result;
    } catch (error:Dynamic) {
      for (edge in edges) edge.close();
      throw error;
    }
  }

  public function asWire():Curve {
    if (shape.kind() == CadKit.ShapeKind.Wire) return new Curve(shape.cloneShape());
    return wire([this]);
  }

  public function placed(location:Location):Curve {
    return fromOperation(location.applyOperation(shape));
  }

  public function offset(distance:Float):Curve {
    var wire = asWire();
    try {
      var result = fromOperation(new Operation(CadKit.wireOffsetOperationChecked(wire.shape.borrowHandle(), distance), null, null, "wire-offset"));
      wire.close();
      return result;
    } catch (error:Dynamic) {
      wire.close();
      throw error;
    }
  }

  public function project(target:Model, direction:Vector):Curve {
    return new Curve(Shape.fromOwnedHandle(CadKit.projectChecked(
      shape.borrowHandle(),
      target.shape.borrowHandle(),
      direction.native()
    )));
  }
}
