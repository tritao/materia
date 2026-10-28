package cadkit.modeling;

/**
 * A flat shape centred on a position, for layout and clearance work. Millimetres. `Shifted` is a shape whose centre is
 * off the footprint's own origin, such as a module whose body hangs below the window it is placed by.
 */
enum Footprint {
  Circle(diameter:Float);
  Rect(width:Float, height:Float);
  Shifted(shape:Footprint, dx:Float, dy:Float);
}
