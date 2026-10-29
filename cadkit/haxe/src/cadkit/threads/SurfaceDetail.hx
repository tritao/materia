package cadkit.threads;

/**
 * A cosmetic pattern on a surface of revolution (a cylinder or cone face), drawn as a texture rather than modelled: the solid
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
}
