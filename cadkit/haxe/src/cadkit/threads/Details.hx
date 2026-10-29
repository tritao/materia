package cadkit.threads;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import materia.project.SurfaceDetailRenderData;
import materia.project.SurfaceDetailRenderData.SurfaceDetailKinds;

/**
 * Cosmetic surface details on named faces of revolution. Like threads they are attributes of a strongly named face, so they
 * follow the face through booleans and placement; nothing is modelled, and a viewer with textures draws the pattern.
 */
class Details {
  /** Attaches `detail` to the face called `name` over its whole axial extent. */
  public static function attach(shape:Shape, name:String, detail:SurfaceDetail):Shape {
    switch detail {
      case Knurl(teeth, depth):
        if (teeth < 2 || !(depth >= 0)) throw 'Knurl on $name needs at least two teeth and a depth';
      case Graduation(divisions, every, slant, endRadius):
        if (divisions < 2 || every < 0 || every > divisions || !(slant > 0)
          || !(endRadius >= 0)) throw 'Graduation on $name needs two or more divisions and its surface size';
    }
    var index = Threads.namedIndex(shape, name);
    var face = shape.subshape(CadKit.ShapeKind.Face, index);
    var minimum = 0.0;
    try {
      minimum = Threads.extent(face).minimum;
      face.close();
    } catch (error:Dynamic) {
      face.close();
      throw error;
    }
    var attributes = shape.faceAttributes();
    attributes.push(ThreadedFace.decoration(name, detail, minimum, 1, null));
    var copy = shape.cloneShape();
    copy.setThreadedFaces(attributes);
    return copy;
  }

  /** Portable display data for every detail on the shape, derived on each mesh rebuild alongside its face ranges. */
  public static function renderData(shape:Shape):Array<SurfaceDetailRenderData> {
    var result:Array<SurfaceDetailRenderData> = [];
    for (attribute in shape.surfaceDetails()) {
      var detail = attribute.detail;
      if (detail == null) continue;
      var index = Threads.namedIndex(shape, attribute.faceName);
      var face = shape.subshape(CadKit.ShapeKind.Face, index);
      try {
        var span = Threads.extent(face), axis = face.faceAxis();
        var direction = Vector.fromNative(axis.get_direction());
        var origin = Vector.fromNative(axis.get_origin()).add(direction.scale(span.minimum));
        var reference = Vector.fromNative(axis.get_reference());
        var kind:Int,
          count:Int, depth = 0.0, every = 0, surface = span.maximum - span.minimum, radiusEnd = axis.get_radius();
        switch detail {
          case Knurl(teeth, d):
            kind = SurfaceDetailKinds.KNURL;
            count = teeth;
            depth = d;
          case Graduation(divisions, e, slant, endRadius):
            kind = SurfaceDetailKinds.GRADUATION;
            count = divisions;
            every = e;
            surface = slant;
            radiusEnd = endRadius;
        }
        result.push({
          faceName: attribute.faceName,
          faceIndex: index,
          kind: kind,
          origin: [origin.x, origin.y, origin.z],
          xDirection: [reference.x, reference.y, reference.z],
          direction: [direction.x, direction.y, direction.z],
          length: span.maximum - span.minimum,
          radius: axis.get_radius(),
          count: count,
          depth: depth,
          numberedEvery: every,
          surface: surface,
          radiusEnd: radiusEnd
        });
        face.close();
      } catch (error:Dynamic) {
        face.close();
        throw 'Detail ${attribute.faceName}: $error';
      }
    }
    return result;
  }
}
