package cadkit.threads;

/**
 * A cosmetic pattern on a surface of revolution (a cylinder or cone face), or a display on a plane, drawn as a texture rather than modelled: the solid
 * stays plain, and a viewer with textures shows the pattern, filtered at any distance. Dimensions are millimetres.
 */
enum SurfaceDetail {
  /** Straight knurling: `teeth` around the face, each `depth` deep. */
  Knurl(teeth:Int, depth:Float);

  /**
   * A graduated scale round the face: `divisions` marks, the first at the reference direction and counting counter-clockwise
   * about the axis, with every `numberedEvery`-th mark numbered (0 for none). `slant` is the length of the face along its
   * surface line and `endRadius` its radius at the far end of that line (the near end is the face's own radius), so the
   * marks and numerals keep their proportions on a cone.
   */
  Graduation(divisions:Int, numberedEvery:Int, slant:Float, endRadius:Float);

  /**
   * A live display on a planar face: a picture the viewer fills from a running simulation (see `DisplaySpec`).
   */
  Display(spec:DisplaySpec);
}

/** Which joint feeds one signal of a display's driver. */
class DisplayBinding {
  public final signal:String;
  /** An assembly joint id. */
  public final joint:String;

  public function new(signal:String, joint:String) {
    this.signal = signal;
    this.joint = joint;
  }
}

/**
 * A `width` by `height` picture whose centre is `offsetX` along the face's reference direction and `offsetY` along the face
 * normal crossed with it, from the face's centre. `source` names the driver that fills it, `profile` the operator profile that
 * driver serves, and `bindings` the joints that feed its signals; the pixels are not part of the model.
 */
class DisplaySpec {
  public final width:Float;
  public final height:Float;
  public final offsetX:Float;
  public final offsetY:Float;
  public final source:String;
  public final profile:String;
  public final bindings:Array<DisplayBinding>;

  public function new(
    width:Float,
    height:Float,
    offsetX:Float,
    offsetY:Float,
    source:String,
    profile:String,
    bindings:Array<DisplayBinding>
  ) {
    this.width = width;
    this.height = height;
    this.offsetX = offsetX;
    this.offsetY = offsetY;
    this.source = source;
    this.profile = profile;
    this.bindings = bindings;
  }
}
