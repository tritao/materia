package cadkit.modeling;

import CadKit;
import cadkit.Shape;
import cadkit.ShapeKey;
import cadkit.Operation;

/** Rigid placement. compose(child) applies child first, then this placement. */
class Location {
  public final plane:Plane;

  public function new(plane:Plane) {
    this.plane = plane;
  }

  public static function identity():Location {
    return new Location(Plane.XY());
  }

  public static function translation(delta:Vector):Location {
    return new Location(new Plane(delta, Vector.X(), Vector.Z()));
  }

  public static function rotation(axis:Axis, angle:Float):Location {
    if (!Math.isFinite(angle)) throw "angle must be finite";
    var x = rotateVector(Vector.X(), axis.direction, angle);
    var z = rotateVector(Vector.Z(), axis.direction, angle);
    var origin = axis.origin.subtract(rotateVector(axis.origin, axis.direction, angle));
    return new Location(new Plane(origin, x, z));
  }

  static function rotateVector(v:Vector, n:Vector, a:Float):Vector {
    return v.scale(Math.cos(a)).add(n.cross(v).scale(Math.sin(a))).add(n.scale(n.dot(v) * (1 - Math.cos(a))));
  }

  public function direction(v:Vector):Vector {
    return plane.xDirection.scale(v.x).add(plane.yDirection.scale(v.y)).add(plane.normal.scale(v.z));
  }

  public function compose(child:Location):Location {
    return new Location(new Plane(
      plane.toWorld(child.plane.origin),
      direction(child.plane.xDirection),
      direction(child.plane.normal)
    ));
  }

  public function inverse():Location {
    var x = plane.xDirection;
    var y = plane.yDirection;
    var z = plane.normal;
    var o = plane.origin;
    return new Location(new Plane(
      new Vector(-o.dot(x), -o.dot(y), -o.dot(z)),
      new Vector(x.x, y.x, z.x),
      new Vector(x.z, y.z, z.z)
    ));
  }

  public function applyOperation(shape:Shape):Operation {
    return new Operation(
      CadKit.shapePlaceOperationChecked(
        shape.borrowHandle(),
        plane.origin.native(),
        plane.xDirection.native(),
        plane.normal.native()
      ),
      [shape],
      ShapeKey.op("place-op", placeParams(), [shape]),
      "place"
    );
  }

  /** The placement as key text: exact bits of the plane the shape is carried to. */
  function placeParams():String {
    function v(x:Vector):String return ShapeKey.number(x.x) + "," + ShapeKey.number(x.y) + "," + ShapeKey.number(x.z);
    return v(plane.origin) + "|" + v(plane.xDirection) + "|" + v(plane.normal);
  }

  public function apply(shape:Shape):Shape {
    if (shape.faceAttributes().length > 0) {
      var operation = applyOperation(shape);
      try {
        var result = operation.resultShape();
        operation.close();
        return result;
      } catch (error:Dynamic) {
        operation.close();
        throw error;
      }
    }
    var result = Shape.fromOwnedHandle(CadKit.shapePlaceChecked(
      shape.borrowHandle(),
      plane.origin.native(),
      plane.xDirection.native(),
      plane.normal.native()
    ));
    var key = ShapeKey.op("place", placeParams(), [shape]);
    result.key = key;
    result.gap = key == null ? ShapeKey.gapOf("place", [shape]) : null;
    return result;
  }
}
