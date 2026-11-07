package cadkit.modeling;

import CadKit;
import cadkit.Shape;
import cadkit.PhysicalProperties;
import cadkit.Operation;
import cadkit.ShapeKey;
import cadkit.cache.OpCache;

/** Solid results may contain zero, one, or several solids. */
class Part extends Model {
  public function new(shape:Shape, operation:Null<Operation> = null) {
    super(shape, operation);
    try {
      if (!isEmpty() && solidCount() == 0) throw "part requires solids";
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

  public static function box(
    width:Float,
    depth:Float,
    height:Float,
    alignX:Align = Center,
    alignY:Align = Center,
    alignZ:Align = Min
  ):Part {
    var shape = Shape.box(width, depth, height);
    try {
      var placed = shape.translate(new Vector(
        Model.alignment(width, alignX),
        Model.alignment(depth, alignY),
        Model.alignment(height, alignZ)
      ).native());
      shape.close();
      return new Part(placed);
    } catch (error:Dynamic) {
      shape.close();
      throw error;
    }
  }

  /** A cylinder from z=0 with the given height. */
  public static function cylinder(radius:Float, height:Float):Part return new Part(Shape.cylinder(radius, height));

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
    if (!(radius > 0)
      || !(length > 0) || origin == null || direction == null) throw "Cylinder needs a positive radius and length";
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

  /**
	 * Fuse borrowed parts into an independently owned result.
	 *
	 * The parts are fused as a balanced tree: each boolean joins two halves of similar size, so most of them work on small
	 * shapes. Fusing them one after another instead grows one result through every step, and each boolean (and the validity
	 * check after it) pays for everything fused so far; for a frame of dozens of members that was most of the time to build it.
	 */
  public static function fuseAll(parts:Array<Part>):Part {
    if (parts.length == 0) throw "Fuse needs at least one part";
    return fuseRange(parts, 0, parts.length);
  }

  static function fuseRange(parts:Array<Part>, from:Int, to:Int):Part {
    var count = to - from;
    if (count == 1) return new Part(parts[from].shape.cloneShape());
    if (count == 2) return joined(parts[from], parts[from + 1], Add);
    var middle = from + (count >> 1), left = fuseRange(parts, from, middle), right:Null<Part> = null;
    try {
      right = fuseRange(parts, middle, to);
      var result = joined(left, right, Add);
      left.close();
      right.close();
      return result;
    } catch (error:Dynamic) {
      left.close();
      if (right != null) right.close();
      throw error;
    }
  }

  /**
   * `a` fused with (or less) `b` for `fuseAll` and `subtractAll`: through the cached boolean (`Shape.fuse`, `cut`) when both
   * shapes carry construction keys, which gives a part without an operation history (these results never use it); otherwise as
   * `combine`, with its history.
   */
  static function joined(a:Part, b:Part, mode:Mode):Part {
    if (a.shape.key != null && b.shape.key != null && a.shape.faceAttributes().length == 0 && b.shape.faceAttributes().length == 0)
      return new Part(switch mode {
        case Add: a.shape.fuse(b.shape);
        case Subtract: a.shape.cut(b.shape);
        case Intersect: a.shape.common(b.shape);
      });
    return a.combine(b, mode);
  }

  /**
   * `combine` for a result that does not need the operation's history (its face and edge correspondences): through the cached
   * boolean when both shapes carry construction keys, which returns the canonical form (see `OpCache`); otherwise exactly
   * `combine`. Use it where the result only feeds further booleans, a mesh or a measurement.
   */
  public function combineKept(other:Part, mode:Mode = Add):Part return joined(this, other, mode);

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
      // A loft is a function of its ordered sections and its ruled flag: keyed when every section is.
      var result = fromOperation(new Operation(
        CadKit.loftOperationChecked(Model.refs(shapes), 1, ruled ? 1 : 0),
        shapes, ShapeKey.op("loft", ruled ? "ruled" : "smooth", shapes), "loft"
      ));
      for (wire in wires) wire.close();
      return result;
    } catch (error:Dynamic) {
      for (wire in wires) wire.close();
      throw error;
    }
  }

  /**
   * `loft` for a result that does not need the operation's history (its face and edge correspondences): through the cached
   * constructor when every section is keyed, which returns the canonical form (see `OpCache`); otherwise exactly `loft`. Its key
   * is named `loft-kept`, apart from `loft`'s, so one key never names two result forms.
   */
  public static function loftKept(sections:Array<Curve>, ruled:Bool = false):Part {
    var wires:Array<Curve> = [];
    var shapes:Array<Shape> = [];
    try {
      for (section in sections) {
        var wire = section.asWire();
        wires.push(wire);
        shapes.push(wire.shape);
      }
      if (ShapeKey.op("loft-kept", ruled ? "ruled" : "smooth", shapes) == null) {
        for (wire in wires) wire.close();
        return loft(sections, ruled);
      }
      var result = new Part(OpCache.construct("loft-kept", ruled ? "ruled" : "smooth", shapes, () -> {
        var operation = new Operation(CadKit.loftOperationChecked(Model.refs(shapes), 1, ruled ? 1 : 0), shapes, null, "loft");
        try {
          var built = operation.resultShape();
          operation.close();
          return built;
        } catch (error:Dynamic) {
          operation.close();
          throw error;
        }
      }));
      for (wire in wires) wire.close();
      return result;
    } catch (error:Dynamic) {
      for (wire in wires) wire.close();
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
      case Add:
        shape.fuseOperation(other.shape);
      case Subtract:
        shape.cutOperation(other.shape);
      case Intersect:
        shape.commonOperation(other.shape);
    }
    );
  }

  public function subtract(other:Part):Part {
    return combine(other, Subtract);
  }

  /** Finalize a modeled part before deriving both its mesh and STEP export. */
  public function refined():Part return fromOperation(shape.refineOperation());

  /**
		This part with every face, edge and vertex name prefixed by `tag:` (plans/TOPOLOGICAL_NAMING.md): name the bodies
		a part is built from (`plate`, `bore`) so their faces keep telling apart, and stay findable, as parameters change.
	**/
  public function named(tag:String):Part {
    return new Part(shape.stamped(tag, []));
  }

  /** Subtract borrowed tools, preserving this part and every tool. */
  public function subtractAll(tools:Array<Part>):Part {
    if (tools.length == 0) return new Part(shape.cloneShape());
    var tool = Part.fuseAll(tools);
    try {
      var result = joined(this, tool, Subtract);
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
      for (edge in edges) edge.close();
      return result;
    } catch (error:Dynamic) {
      for (edge in edges) edge.close();
      throw error;
    }
  }

  public function chamfer(selection:Selection, distance:Float):Part {
    var edges = selection.edgeValues();
    try {
      var result = fromOperation(shape.chamferEdgesOperation(edges, distance));
      for (edge in edges) edge.close();
      return result;
    } catch (error:Dynamic) {
      for (edge in edges) edge.close();
      throw error;
    }
  }

  public function shell(faces:Selection, thickness:Float):Part {
    return fromOperation(new Operation(
      CadKit.shellOperationChecked(shape.borrowHandle(), Model.refs(faces.borrowShapes()), thickness),
      [shape], null, "shell"
    ));
  }

  public function volume():Float {
    return shape.volume();
  }

  /** The least distance between this part and `other`; zero where they touch or overlap (see `Shape.distance`). */
  public function distance(other:Part):Float {
    return shape.distance(other.shape);
  }

  public function massProperties():PhysicalProperties {
    return shape.massProperties();
  }

  public function solidCount():Int {
    return shape.subshapeCount(CadKit.ShapeKind.Solid);
  }
}
