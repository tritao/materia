package machinekit.hmi;

import cadkit.modeling.Footprint;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.Dimension;
import cadkit.parametric.recording.DocumentBuilder;

/**
 * Placement helpers for recording an enclosure part as labelled parametric features. Primitives sit at their origin
 * corner and point along +Z, so a placement is a translate and a sideways hole tool is a rotate then a translate.
 */
class PanelRecorder {
  public final b:DocumentBuilder;

  public function new(b:DocumentBuilder) {
    this.b = b;
  }

  public function boxAt(text:String, x0:Float, y0:Float, z0:Float, w:Float, h:Float, d:Float):Feature
    return b.label(b.translate(b.label(b.box(w, h, d), text + " box"), x0, y0, z0), text);

  public function cylinderZ(text:String, radius:Float, z0:Float, z1:Float, x:Float, y:Float):Feature
    return b.label(b.translate(b.label(b.cylinder(radius, z1 - z0), text + " cylinder"), x, y, z0), text);

  /** A cylinder turned from +Z onto `axis` by `angle`, then placed at (x, y, z). */
  public function cylinderTurned(
    text:String,
    radius:Float,
    length:Float,
    axis:Vector,
    angle:Float,
    x:Float,
    y:Float,
    z:Float
  ):Feature
    {
    var solid = b.label(b.cylinder(radius, length), text + " cylinder");
    var turned = b.label(b.rotate(solid, new Vector(), axis, angle), text + " turn");
    return b.label(b.translate(turned, x, y, z), text);
  }

  /** A footprint extruded from z0 to z1, centred at (x, y). */
  public function footprint(text:String, shape:Footprint, x:Float, y:Float, z0:Float, z1:Float):Feature
    return switch shape {
      case Circle(diameter): cylinderZ(text, diameter / 2, z0, z1, x, y);
      case Rect(w, h): boxAt(text, x - w / 2, y - h / 2, z0, w, h, z1 - z0);
      case Shifted(inner, dx, dy): footprint(text, inner, x + dx, y + dy, z0, z1);
      case _: throw "Unknown footprint";
    };

  /** A rounded rectangle centred on the axis, extruded from z0 to z1. */
  public function rounded(text:String, width:Dimension, height:Dimension, radius:Dimension, z0:Float, z1:Float):Feature {
    var plane = new Plane(new Vector(0, 0, z0), Vector.X(), Vector.Z());
    var profile = b.label(b.roundedRectangle(width, height, radius, plane), text + " profile");
    return b.label(b.extrude(profile, z1 - z0), text);
  }

  public function fuse(text:String, a:Feature, tool:Feature):Feature return b.label(b.add(a, tool), text);

  public function cut(text:String, a:Feature, tool:Feature):Feature return b.label(b.subtract(a, tool), text);
}
