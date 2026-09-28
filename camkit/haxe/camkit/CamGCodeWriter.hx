package camkit;

import cnckit.CncChannels;
import cnckit.CncMachine;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ArcPlane;
import toolpathkit.path.Point3;

/** Small LinuxCNC post for CAM IR, using millimetres and absolute XYZ. */
class CamGCodeWriter {
  public static function write(program:CamProgram, setup:CamSetup,
      machine:CncMachine):String {
    if (program == null) throw "G-code export needs a CAM program";
    if (setup == null) throw "G-code export needs a CAM setup";
    setup.validate(program, machine);
    var lines = ["G21 G90 G17 G61"], plane = ArcPlane.XY;
    var index = 0;
    while (index < program.ops.length) {
      var op = program.ops[index];
      switch op {
      case Rapid(geometry, _):
        switch geometry {
          case Line(_, end): lines.push('G0 ${xyz(end)}');
          case _: throw "CAM rapid export needs a line";
        }
      case Feed(geometry, speed, blend, _):
        if (blend != 0.0) throw "CAM G-code export needs exact-stop feeds";
        var command = 'F${number(speed * 60000.0)} ';
        switch geometry {
          case Line(_, end): command += 'G1 ${xyz(end)}';
          case Arc(center, _, _, sweep):
            if (plane != XY) { command += "G17 "; plane = XY; }
            var start = GeometryTools.pointAt(geometry, 0.0);
            var end = GeometryTools.pointAt(geometry,
              GeometryTools.length(geometry));
            command += '${sweep < 0.0 ? "G2" : "G3"} ${xyz(end)} '
              + 'I${millimetres(center.x - start.x)} '
              + 'J${millimetres(center.y - start.y)}';
          case Circular(center, _, _, sweep, arcPlane, _):
            if (plane != arcPlane) {
              command += switch arcPlane {
                case XY: "G17 ";
                case XZ: "G18 ";
                case YZ: "G19 ";
              };
              plane = arcPlane;
            }
            var start = GeometryTools.pointAt(geometry, 0.0);
            var end = GeometryTools.pointAt(geometry,
              GeometryTools.length(geometry));
            var clockwise = arcPlane == XZ ? sweep > 0.0 : sweep < 0.0;
            command += '${clockwise ? "G2" : "G3"} ${xyz(end)} ';
            command += switch arcPlane {
              case XY: 'I${millimetres(center.x - start.x)} J${millimetres(center.y - start.y)}';
              case XZ: 'I${millimetres(center.x - start.x)} K${millimetres(center.z - start.z)}';
              case YZ: 'J${millimetres(center.y - start.y)} K${millimetres(center.z - start.z)}';
            };
        }
        lines.push(command);
      case Dwell(seconds, _): lines.push('G4 P${number(seconds)}');
      case ToolChange(number, _): lines.push('T$number M6');
      case ToolLengthOffset(number, _, _):
        lines.push(number == 0 ? "G49" : 'G43 H$number');
      case OptionalStop(_): lines.push("M1");
      case ProgramStop(_): lines.push("M0");
      case End(_): lines.push("M2");
      case Spindle(channel, value, _):
        if (index + 1 >= program.ops.length)
          throw "CAM spindle export needs a direction/speed pair";
        var next = program.ops[index + 1];
        if (channel == CncChannels.SpindleDirection && value != 0.0) {
          switch next {
            case Spindle(CncChannels.SpindleSpeed, rpm, _):
              if (rpm <= 0.0) throw "CAM spindle start needs positive RPM";
              lines.push('S${number(rpm)} ${value > 0.0 ? "M3" : "M4"}');
            case _: throw "CAM spindle start needs a following speed";
          }
        } else if (channel == CncChannels.SpindleSpeed && value == 0.0) {
          switch next {
            case Spindle(CncChannels.SpindleDirection, direction, _):
              if (direction != 0.0) throw "CAM spindle stop needs zero direction";
              lines.push("M5");
            case _: throw "CAM spindle stop needs a following direction";
          }
        } else throw 'Unsupported CAM spindle sequence on "$channel"';
        index++;
      case Coolant(channel, enabled, _):
        if (!enabled && channel == CncChannels.CoolantMist) {
          if (index + 1 >= program.ops.length) throw "M9 needs both coolant channels";
          switch program.ops[index + 1] {
            case Coolant(CncChannels.CoolantFlood, false, _):
              lines.push("M9"); index++;
            case _: throw "M9 needs both coolant channels";
          }
        } else if (enabled && channel == CncChannels.CoolantMist) lines.push("M7");
        else if (enabled && channel == CncChannels.CoolantFlood) lines.push("M8");
        else throw 'Unsupported CAM coolant channel "$channel"';
      case CutterCompStart(_, _, _, _), CutterCompEnd(_):
        throw "CAM G-code export needs resolved cutter geometry";
      }
      index++;
    }
    return lines.join("\n") + "\n";
  }

  static function xyz(point:Point3):String
    return 'X${millimetres(point.x)} Y${millimetres(point.y)} Z${millimetres(point.z)}';

  static function millimetres(metres:Float):String
    return number(metres * 1000.0);

  /** Six decimal places in millimetres avoid exponent syntax in G-code. */
  static function number(value:Float):String {
    if (!Math.isFinite(value)) throw "CAM G-code cannot export non-finite values";
    var sign = value < 0.0 ? "-" : "";
    var magnitude = Math.abs(value);
    var whole = Math.floor(magnitude);
    var fraction = Math.round((magnitude - whole) * 1000000.0);
    if (fraction == 1000000) { whole++; fraction = 0; }
    if (fraction == 0) return sign + Std.string(whole);
    var digits = StringTools.lpad(Std.string(fraction), "0", 6);
    while (StringTools.endsWith(digits, "0")) digits = digits.substr(0, digits.length - 1);
    return sign + Std.string(whole) + "." + digits;
  }
}
