package cadkit;

import CadKit;
import haxe.crypto.Sha256;
import runtime.FloatBits;

/**
 * Construction-history keys. A shape's key is the hash of how it was made: an operation's name and parameters and the keys of
 * its input shapes (a Merkle chain), never of its geometry. Equal keys mean the same sequence of operations on the same
 * inputs, so, CadKit and OCCT being the same, the same shape. A shape made by something that has no key (a spline, a loft, a
 * compound, anything not listed here) has a null key and so has everything built from it: nothing is ever cached under a key
 * that was guessed.
 *
 * A shape's face attributes (threads, surface details) live in Haxe beside the B-rep, so they are part of its key: setting
 * them keys the shape again from its construction key and the exact bytes of the attributes (`Shape.setThreadedFaces`), and an
 * operation's attributes follow from its inputs' attributes (the operation history carries them), so they are inputs of the
 * operation's key like the inputs' construction.
 */
class ShapeKey {
  /** A float's exact bits: -0.0 and 0.0 are different keys (a miss, never a wrong hit). */
  public static function number(value:Float):String return haxe.Int64.toStr(FloatBits.toInt64(value));

  public static function vec(value:CadKit.Vec3):String return number(value.get_x())
    + "," + number(value.get_y()) + "," + number(value.get_z());

  public static function hash(text:String):String return Sha256.encode(text);

  /** SHA-256 of `bytes` as hex text. */
  public static function bytesHash(bytes:haxe.io.Bytes):String {
    var digest = Sha256.make(bytes), out = new StringBuf();
    for (index in 0...digest.length) out.add(StringTools.hex(digest.get(index), 2));
    return out.toString();
  }

  /** The key of a shape made from nothing (a primitive, an import): `name` and its `params` text. */
  public static function leaf(name:String, params:String):String return hash("cadkit-shape/1\n" + name + "\n" + params);

  /** The key of `name` with `params` applied to `inputs`; null when an input has no key. */
  public static function op(name:String, params:String, inputs:Array<Shape>):Null<String> {
    var keys:Array<String> = [];
    for (input in inputs) {
      var key = input.key;
      if (key == null) return null;
      keys.push(key);
    }
    return hash("cadkit-shape/1\n" + name + "\n" + params + "\n" + keys.join("\n"));
  }

  /** What to call an unkeyed result: the first reason an input is unkeyed, else `name`. For coverage reports. */
  public static function gapOf(name:String, inputs:Array<Shape>):String {
    for (input in inputs) if (input.key == null && input.gap != null) return input.gap;
    return name;
  }
}
