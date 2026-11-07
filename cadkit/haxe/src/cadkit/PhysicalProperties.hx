package cadkit;

import cadkit.modeling.Vector;
import haxe.io.Bytes;
import haxe.io.BytesInput;
import haxe.io.BytesOutput;

/** Volume-weighted physical properties for a positive-volume CAD shape. */
class PhysicalProperties {
  public final volume:Float;
  public final surfaceArea:Float;
  public final centerOfMass:Vector;
  /** Centroidal inertia for unit density, in length^5 units. */
  public final inertia:InertiaTensor;

  public function new(volume:Float, surfaceArea:Float, centerOfMass:Vector, inertia:InertiaTensor) {
    this.volume = volume;
    this.surfaceArea = surfaceArea;
    this.centerOfMass = centerOfMass;
    this.inertia = inertia;
  }

  /** Return mass in the matching units for a density per cubic kernel unit. */
  public function mass(density:Float):Float {
    if (!Math.isFinite(density) || density < 0) throw "density must be finite and nonnegative";
    return volume * density;
  }

  /** Return centroidal inertia for density per cubic kernel unit. */
  public function inertiaAtDensity(density:Float):InertiaTensor {
    if (!Math.isFinite(density) || density < 0) throw "density must be finite and nonnegative";
    return inertia.scaled(density);
  }

  /** The exact bits of every number, for a cache on disk. */
  public function toBytes():Bytes {
    var out = new BytesOutput();
    for (value in [
      volume,
      surfaceArea,
      centerOfMass.x,
      centerOfMass.y,
      centerOfMass.z,
      inertia.xx,
      inertia.xy,
      inertia.xz,
      inertia.yy,
      inertia.yz,
      inertia.zz
    ]) out.writeDouble(value);
    return out.getBytes();
  }

  /** From `toBytes`; wrong-sized bytes throw. */
  public static function fromBytes(bytes:Bytes):PhysicalProperties {
    if (bytes.length != 11 * 8) throw "Damaged physical properties";
    var input = new BytesInput(bytes);
    var values = [for (_ in 0...11) input.readDouble()];
    return new PhysicalProperties(
      values[0],
      values[1],
      new Vector(values[2], values[3], values[4]),
      new InertiaTensor(values[5], values[6], values[7], values[8], values[9], values[10])
    );
  }
}
