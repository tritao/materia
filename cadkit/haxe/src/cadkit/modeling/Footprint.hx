package cadkit.modeling;

/** A flat shape centred on a position, for layout and clearance work. Millimetres. */
enum Footprint {
  Circle(diameter:Float);
  Rect(width:Float, height:Float);
}
