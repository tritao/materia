package toolpathkit.path;

/** What the moves that follow a `Feature` marker are for, as slicers label them. */
enum PrintFeature {
  ExternalPerimeter;
  Perimeter;
  Infill;
  SolidInfill;
  TopSolidInfill;
  Skirt;
  Travel;
  /** Any other label; `name` is the slicer's own text. */
  Other(name:String);
}
