package machinekit.cache;

import haxe.io.Bytes;
import haxe.io.BytesInput;
import haxe.io.BytesOutput;
import cadkit.InertiaTensor;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentDisplayMesh.ComponentDisplayMeshData;
import machinekit.component.ComponentValue;
import machinekit.component.ComponentValues;
import machinekit.component.MassProperties;
import machinekit.component.MassProperties.MassSource;

/** Exact (bit-for-bit) byte codecs for the plain values a persistent `Memo` keeps. */
class Codecs {
  public static function floats(list:Array<Float>):Bytes {
    var out = new BytesOutput();
    writeFloats(out, list);
    return out.getBytes();
  }

  public static function unfloats(bytes:Bytes):Array<Float> {
    var input = new BytesInput(bytes);
    var list = readFloats(input);
    done(input, bytes);
    return list;
  }

  public static function displayMesh(mesh:ComponentDisplayMeshData):Bytes {
    var out = new BytesOutput();
    writeFloats(out, mesh.positions);
    writeFloats(out, mesh.normals);
    out.writeInt32(mesh.indices.length);
    for (index in mesh.indices) out.writeInt32(index);
    return out.getBytes();
  }

  public static function undisplayMesh(bytes:Bytes):ComponentDisplayMeshData {
    var input = new BytesInput(bytes);
    var positions = readFloats(input), normals = readFloats(input);
    var indices = [for (_ in 0...input.readInt32()) input.readInt32()];
    done(input, bytes);
    return {positions: positions, normals: normals, indices: indices};
  }

  public static function massProperties(mass:MassProperties):Bytes {
    var out = new BytesOutput();
    out.writeDouble(mass.mass);
    out.writeDouble(mass.centreOfMass.x);
    out.writeDouble(mass.centreOfMass.y);
    out.writeDouble(mass.centreOfMass.z);
    switch mass.source {
      case Computed(Envelope):
        out.writeByte(0);
      case Computed(Preview):
        out.writeByte(1);
      case Declared:
        out.writeByte(2);
      case EffectiveDensity(density):
        out.writeByte(3);
        out.writeDouble(density);
      case Modelled:
        out.writeByte(4);
    }
    var inertia = mass.inertia;
    if (inertia == null) out.writeByte(0);
    else {
      out.writeByte(1);
      for (value in [inertia.xx, inertia.xy, inertia.xz, inertia.yy, inertia.yz, inertia.zz]) out.writeDouble(value);
    }
    return out.getBytes();
  }

  public static function unmassProperties(bytes:Bytes):MassProperties {
    var input = new BytesInput(bytes);
    var mass = input.readDouble();
    var centre = new Vector(input.readDouble(), input.readDouble(), input.readDouble());
    var source:MassSource = switch input.readByte() {
      case 0: Computed(Envelope);
      case 1: Computed(Preview);
      case 2: Declared;
      case 3: EffectiveDensity(input.readDouble());
      case 4: Modelled;
      default: throw "stored mass has an unknown source";
    };
    var inertia:Null<InertiaTensor> = null;
    switch input.readByte() {
      case 0:
      case 1:
        inertia = new InertiaTensor(input.readDouble(), input.readDouble(), input.readDouble(), input.readDouble(), input.readDouble(),
          input.readDouble());
      default:
        throw "stored mass has a bad inertia flag";
    }
    done(input, bytes);
    return new MassProperties(mass, centre, source, inertia);
  }

  /** Component values, names in sorted order so equal values give equal bytes. */
  public static function writeValues(out:BytesOutput, values:ComponentValues):Void {
    var names = values.names();
    names.sort(Reflect.compare);
    out.writeInt32(names.length);
    for (name in names) {
      writeText(out, name);
      switch values.get(name) {
        case Number(value):
          out.writeByte(0);
          out.writeDouble(value);
        case Integer(value):
          out.writeByte(1);
          out.writeInt32(value);
        case Boolean(value):
          out.writeByte(value ? 3 : 2);
        case Token(value):
          out.writeByte(4);
          writeText(out, value);
        case Unset:
          out.writeByte(5);
        case null:
          throw "component value is missing";
      }
    }
  }

  public static function readValues(input:BytesInput):ComponentValues {
    var values = new ComponentValues();
    for (_ in 0...input.readInt32()) {
      var name = readText(input);
      switch input.readByte() {
        case 0: values.setNumber(name, input.readDouble());
        case 1: values.setInteger(name, input.readInt32());
        case 2: values.setBoolean(name, false);
        case 3: values.setBoolean(name, true);
        case 4: values.setToken(name, readText(input));
        case 5: values.setUnset(name);
        default: throw "stored component value has an unknown type";
      }
    }
    return values;
  }

  public static function writeText(out:BytesOutput, text:String):Void {
    var bytes = Bytes.ofString(text);
    out.writeInt32(bytes.length);
    out.write(bytes);
  }

  public static function readText(input:BytesInput):String {
    var length = input.readInt32();
    if (length < 0) throw "stored text has a negative length";
    return input.read(length).toString();
  }

  /** The reader must have consumed all of `bytes`; see `done`. */
  public static function finished(input:BytesInput, bytes:Bytes):Void done(input, bytes);

  static function writeFloats(out:BytesOutput, list:Array<Float>):Void {
    out.writeInt32(list.length);
    for (value in list) out.writeDouble(value);
  }

  static function readFloats(input:BytesInput):Array<Float> return [for (_ in 0...input.readInt32()) input.readDouble()];

  /** A stored value must be consumed exactly; anything else is damage, which the memo treats as a miss. */
  static function done(input:BytesInput, bytes:Bytes):Void {
    var rest = 0;
    try {
      input.readByte();
      rest = 1;
    } catch (error:Dynamic) {}
    if (rest != 0) throw "stored value has trailing bytes";
  }
}
