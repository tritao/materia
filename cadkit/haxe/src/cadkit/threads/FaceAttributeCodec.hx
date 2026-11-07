package cadkit.threads;

import cadkit.threads.SurfaceDetail.DisplayBinding;
import cadkit.threads.SurfaceDetail.DisplaySpec;
import haxe.io.Bytes;
import haxe.io.BytesInput;
import haxe.io.BytesOutput;
import materia.project.ThreadSpecData.ThreadHand;
import materia.project.ThreadSpecData.ThreadProfile;

/**
 * Exact bytes for a shape's face attributes (threads and surface details), which CadKit keeps in Haxe beside the native
 * B-rep. Part of `Shape.toBytes`; a damaged buffer throws.
 */
class FaceAttributeCodec {
  static final version = 1;

  public static function encode(attributes:Array<ThreadedFace>):Bytes {
    var out = new BytesOutput();
    out.writeInt32(version);
    out.writeInt32(attributes.length);
    for (attribute in attributes) {
      text(out, attribute.faceName);
      var spec = attribute.spec;
      text(out, spec.designation);
      out.writeDouble(spec.majorDiameter);
      out.writeDouble(spec.pitch);
      out.writeInt32(spec.starts);
      out.writeByte(switch spec.hand {
        case RightHand:
          1;
        case LeftHand:
          2;
      }
      );
      out.writeByte(spec.internal ? 1 : 0);
      text(out, spec.toleranceClass);
      out.writeByte(switch spec.profile {
        case Metric:
          1;
        case Trapezoidal:
          2;
        case Acme:
          3;
        case Ball:
          4;
      }
      );
      out.writeDouble(attribute.start);
      out.writeInt32(attribute.direction);
      var length = attribute.length;
      out.writeByte(length == null ? 0 : 1);
      if (length != null) out.writeDouble(length);
      var detail = attribute.detail;
      if (detail == null) out.writeByte(0) else switch detail {
        case Knurl(teeth, depth):
          out.writeByte(1);
          out.writeInt32(teeth);
          out.writeDouble(depth);
        case Graduation(divisions, numberedEvery, slant, endRadius):
          out.writeByte(2);
          out.writeInt32(divisions);
          out.writeInt32(numberedEvery);
          out.writeDouble(slant);
          out.writeDouble(endRadius);
        case Display(display):
          out.writeByte(3);
          out.writeDouble(display.width);
          out.writeDouble(display.height);
          out.writeDouble(display.offsetX);
          out.writeDouble(display.offsetY);
          text(out, display.source);
          text(out, display.profile);
          out.writeInt32(display.bindings.length);
          for (binding in display.bindings) {
            text(out, binding.signal);
            text(out, binding.joint);
          }
          text(out, display.channel);
          out.writeByte(display.round ? 1 : 0);
      }
    }
    return out.getBytes();
  }

  public static function decode(bytes:Bytes):Array<ThreadedFace> {
    var input = new BytesInput(bytes);
    if (input.readInt32() != version) throw "Unsupported face attribute bytes";
    var count = input.readInt32();
    if (count < 0 || count > bytes.length) throw "Damaged face attribute bytes";
    var result:Array<ThreadedFace> = [];
    for (_ in 0...count) {
      var faceName = readText(input, bytes);
      var designation = readText(input, bytes);
      var majorDiameter = input.readDouble(), pitch = input.readDouble();
      var starts = input.readInt32();
      var hand = switch input.readByte() {
        case 1:
          ThreadHand.RightHand;
        case 2:
          ThreadHand.LeftHand;
        default:
          throw "Damaged face attribute bytes";
      };
      var internal = flag(input);
      var toleranceClass = readText(input, bytes);
      var profile = switch input.readByte() {
        case 1:
          ThreadProfile.Metric;
        case 2:
          ThreadProfile.Trapezoidal;
        case 3:
          ThreadProfile.Acme;
        case 4:
          ThreadProfile.Ball;
        default:
          throw "Damaged face attribute bytes";
      };
      var spec = new ThreadSpec(designation, majorDiameter, pitch, starts, hand, internal, toleranceClass, profile);
      var start = input.readDouble();
      var direction = input.readInt32();
      var length:Null<Float> = flag(input) ? input.readDouble() : null;
      var detail:Null<SurfaceDetail> = switch input.readByte() {
        case 0:
          null;
        case 1:
          Knurl(input.readInt32(), input.readDouble());
        case 2:
          var divisions = input.readInt32(), numberedEvery = input.readInt32();
          var slant = input.readDouble(), endRadius = input.readDouble();
          Graduation(divisions, numberedEvery, slant, endRadius);
        case 3:
          var width = input.readDouble(), height = input.readDouble();
          var offsetX = input.readDouble(), offsetY = input.readDouble();
          var source = readText(input, bytes), profileName = readText(input, bytes);
          var bindingCount = input.readInt32();
          if (bindingCount < 0 || bindingCount > bytes.length) throw "Damaged face attribute bytes";
          var bindings = [for (_ in 0...bindingCount) {
            var signal = readText(input, bytes);
            new DisplayBinding(signal, readText(input, bytes));
          }
          ];
          var channel = readText(input, bytes);
          Display(new DisplaySpec(width,
            height, offsetX, offsetY, source, profileName, bindings, channel, flag(input)));
        default:
          throw "Damaged face attribute bytes";
      };
      result.push(new ThreadedFace(faceName, spec, start, direction, length, detail));
    }
    var trailing = true;
    try input.readByte() catch (error:Dynamic) trailing = false;
    if (trailing) throw "Trailing face attribute bytes";
    return result;
  }

  static function text(out:BytesOutput, value:String):Void {
    out.writeInt32(Bytes.ofString(value).length);
    out.writeString(value);
  }

  static function readText(input:BytesInput, bytes:Bytes):String {
    var length = input.readInt32();
    if (length < 0 || length > bytes.length) throw "Damaged face attribute bytes";
    return input.readString(length);
  }

  static function flag(input:BytesInput):Bool return switch input.readByte() {
    case 0:
      false;
    case 1:
      true;
    default:
      throw "Damaged face attribute bytes";
  };
}
