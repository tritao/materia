package materia.project;

/** The kinds of cosmetic surface detail a face can carry (see `cadkit.threads.SurfaceDetail`). */
class SurfaceDetailKinds {
  public static inline var KNURL:Int = 1;
  public static inline var GRADUATION:Int = 2;
  /** A live picture on a planar face: the frame's z is the face normal, and `width` by `height` the picture. */
  public static inline var DISPLAY:Int = 3;
}

/**
 * Derived display data for a cosmetic detail (knurling, a graduated scale) on a surface of revolution, for renderers
 * without a B-rep. The frame has its origin on the axis at the start of the interval, its normal along the axis and its
 * x direction at the reference angle; lengths are in the part's unit, `depth` in millimetres.
 */
/** Which joint feeds one signal of a live display's driver. */
@:wire typedef DisplayBindingData = {
  @:id(1) var signal:String;
  /** An assembly joint id. */
  @:id(2) var joint:String;
}

@:wire typedef SurfaceDetailRenderData = {
  @:id(1) var faceName:String;
  @:id(2) var faceIndex:Int;
  @:id(3) var kind:Int;
  @:id(4) var origin:Array<Float>;
  @:id(5) var xDirection:Array<Float>;
  @:id(6) var direction:Array<Float>;
  /** Axial length of the interval the detail covers. */
  @:id(7) var length:Float;
  /** The face's radius at the start of the interval, in the part's unit. */
  @:id(8) var radius:Float;
  /** Teeth of a knurl; divisions of a graduation. */
  @:id(9) var count:Int;
  /** Depth of a knurl's teeth, millimetres. */
  @:id(10) var depth:Float;
  /** A graduation numbers every this-many divisions; 0 for none. */
  @:id(11) var numberedEvery:Int;
  /** Length of the face along its surface line (the axial length for a cylinder), in the part's unit. */
  @:id(12) var surface:Float;
  /** The face's radius at the far end of the interval, in the part's unit. */
  @:id(13) var radiusEnd:Float;
  /** A display's picture size in the part's unit; 0 for the other kinds. */
  @:id(14) var width:Float;
  @:id(15) var height:Float;
  /** What drives a display (the editor chooses a live source by this name); empty for the other kinds. */
  @:id(16) var source:String;
  /** The operator profile (by name) a display's driver serves; empty when the driver needs none. */
  @:id(17) var profile:String;
  /** Which joint feeds each signal of the driver; empty for the other kinds. */
  @:id(18) var bindings:Array<DisplayBindingData>;
}

/** Display packets are validated before an editor allocates resources. */
class SurfaceDetailRenderDataValues {
  public static function validate(detail:SurfaceDetailRenderData):Void {
    if (detail.faceName == null || detail.faceName.length == 0
      || detail.faceName.length > 4096 || detail.faceIndex < 0 || !Math.isFinite(detail.length) || detail.length <= 0
      || !Math.isFinite(detail.radius) || detail.radius < 0 || !(detail.surface > 0) || !Math.isFinite(detail.radiusEnd)
      || detail.radiusEnd < 0) throw "Surface detail needs a face name, index, positive length and radius";
    if (detail.kind != SurfaceDetailKinds.KNURL && detail.kind != SurfaceDetailKinds.GRADUATION
      && detail.kind != SurfaceDetailKinds.DISPLAY) throw 'Surface detail ${detail.faceName} has an unknown kind';
    if (detail.kind == SurfaceDetailKinds.DISPLAY
      && !(detail.width > 0 && detail.height > 0 && Math.isFinite(detail.width)
      && Math.isFinite(detail.height))) throw 'Display ${detail.faceName} needs a positive size';
    if (detail.kind == SurfaceDetailKinds.DISPLAY && (detail.source == null
      || detail.source.length == 0 || detail.source.length > 256)) throw 'Display ${detail.faceName} needs a source';
    if (detail.bindings == null) throw 'Surface detail ${detail.faceName} has no bindings list';
    for (binding in detail.bindings) if (binding.signal == null || binding.signal.length == 0 || binding.joint == null
      || binding.joint.length == 0) throw 'Display ${detail.faceName} has a binding without a signal or a joint';
    if (detail.count < 2
      || detail.count > 4096 || !Math.isFinite(detail.depth) || detail.depth < 0 || detail.numberedEvery < 0
      || detail.numberedEvery > detail.count) throw 'Surface detail ${detail.faceName} has invalid numbers';
    for (vector in [detail.origin, detail.xDirection, detail.direction]) {
      if (vector == null || vector.length != 3) throw 'Surface detail ${detail.faceName} has an invalid frame';
      for (value in vector) if (!Math.isFinite(value)) throw 'Surface detail ${detail.faceName} frame is non-finite';
    }
    var x = detail.xDirection, z = detail.direction;
    if (Math.abs(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] - 1) > 1e-6
      || Math.abs(z[0] * z[0] + z[1] * z[1] + z[2] * z[2] - 1) > 1e-6 || Math.abs(x[0] * z[0]
      + x[1] * z[1] + x[2] * z[2]) > 1e-6) throw 'Surface detail ${detail.faceName} frame axes are not orthonormal';
  }
}
