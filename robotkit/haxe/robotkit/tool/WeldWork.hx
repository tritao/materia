package robotkit.tool;

/**
 * The grounded work of a weld: the metal the circuit returns through, which is all an arc can strike
 * to. Points and rays are in the world frame, in metres.
 */
interface WeldWork {
  /** How far the point is from the nearest grounded surface: negative inside the metal, positive infinity with none. */
  function distance(x:Float, y:Float, z:Float):Float;

  /** Where the ray from `o` along the unit vector `d` first meets grounded metal within `range`, or positive infinity. */
  function ray(ox:Float, oy:Float, oz:Float, dx:Float, dy:Float, dz:Float, range:Float):Float;
}
