package cadkit.threads;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.threads.SurfaceDetail.DisplaySpec;
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
      case Display(spec):
        if (spec.source == null || spec.source.length == 0) throw 'Display on $name needs a source';
        if (!(spec.width > 0) || !(spec.height > 0) || !Math.isFinite(spec.offsetX)
          || !Math.isFinite(spec.offsetY)) throw 'Display on $name needs a positive size and a finite offset';
        for (binding in spec.bindings) if (binding.signal == null || binding.signal == "" || binding.joint == null
          || binding.joint == "") throw 'Display on $name has a binding without a signal or a joint';
    }
    var index = Threads.namedIndex(shape, name);
    var face = shape.subshape(CadKit.ShapeKind.Face, index);
    var minimum = 0.0;
    try {
      switch detail {
        case Display(_):
          if (face.surfaceKind() != CadKit.SurfaceKind.Plane) throw 'Display on $name needs a planar face';
        case _:
          minimum = Threads.extent(face).minimum;
      }
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
        switch detail {
          case Display(spec):
            result.push(displayData(face, attribute.faceName, index, spec));
            face.close();
            continue;
          case _:
        }
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
          case Display(_):
            throw "unreachable";
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
          radiusEnd: radiusEnd,
          width: 0.0,
          height: 0.0,
          source: "",
          profile: "",
          bindings: [],
          channel: "",
          round: false
        });
        face.close();
      } catch (error:Dynamic) {
        face.close();
        throw 'Detail ${attribute.faceName}: $error';
      }
    }
    return result;
  }

  /** A display's frame: x along the face's reference direction, z out of the face, the picture centred at its offset. */
  static function displayData(face:Shape, name:String, index:Int, spec:DisplaySpec):SurfaceDetailRenderData {
    var normal = Vector.fromNative(face.faceNormal());
    var reference = Vector.fromNative(face.faceAxis().get_reference());
    var z = normal.scale(1.0 / normal.length());
    // The reference direction is in the face's plane; take it orthogonal to the normal to be sure.
    var x = reference.subtract(z.scale(reference.dot(z)));
    x = x.scale(1.0 / x.length());
    var y = z.cross(x);
    var c = Vector.fromNative(face.center());
    var origin = c.add(x.scale(spec.offsetX)).add(y.scale(spec.offsetY));
    return {faceName: name,
      faceIndex: index, kind: SurfaceDetailKinds.DISPLAY, origin: [origin.x, origin.y, origin.z], xDirection: [
        x.x,
        x.y,
        x.z
      ], direction: [
        z.x,
        z.y,
        z.z
      ], length: spec.height, radius: spec.width / 2,
      count: 2, depth: 0.0, numberedEvery: 0, surface: spec.height, radiusEnd: spec.width / 2, width: spec.width,
      height: spec.height, source: spec.source, profile: spec.profile, channel: spec.channel, round: spec.round, bindings: [for (binding in spec.bindings) {
      signal:binding.signal, joint:binding.joint
    }
    ]};
  }
}
