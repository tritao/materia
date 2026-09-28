package cnckit;

import cnckit.CncMachine;
import toolpathkit.setup.Setup;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ArcPlane;
import toolpathkit.path.Point3;

/** LinuxCNC post for shared toolpath operations, in millimetres. */
class CncWriter {
  public static function write(ops:Array<ToolpathOp>, setup:Setup,
      machine:CncMachine):String {
    if (ops == null) throw "G-code export needs operations";
    if (setup == null) throw "G-code export needs a setup";
    if (machine == null) throw "G-code export needs a controller";
    setup.validate(ops, machine.toolLibrary, machine.travelLower, machine.travelUpper);
    var lines = ["G21 G90 G17 G61"], plane = ArcPlane.XY;
    var activeTolerance = 0.0;
    function setTolerance(tolerance:Float):Void {
      if (!Math.isFinite(tolerance) || tolerance < 0.0)
        throw "CNC G-code needs a nonnegative finite tolerance";
      if (Math.abs(tolerance - activeTolerance) <= 1e-12) return;
      lines.push(tolerance == 0.0 ? "G61" : 'G64 P${millimetres(tolerance)}');
      activeTolerance = tolerance;
    }
    var index = 0;
    while (index < ops.length) {
      var op = ops[index];
      switch op {
      case SetSetup(id, _):
        lines.push('G${machine.controller.gCodeForSetup(id, machine.dialect)}');
      case MachineMove(kind, geometry, speed, tolerance, _):
        switch geometry {
          case Line(_, end):
            if (kind == Rapid) lines.push('G53 G0 ${xyz(end)}');
            else {
              if (speed <= 0.0) throw "CNC G-code feed needs positive speed";
              setTolerance(tolerance);
              lines.push('G53 G1 F${number(speed * 60000.0)} ${xyz(end)}');
            }
          case _: throw "CNC G-code export needs a line for machine move";
        }
      case Move(Rapid, geometry, _, _, _), Move(Link, geometry, _, _, _),
          Move(Retract, geometry, _, _, _):
        switch geometry {
          case Line(_, end): lines.push('G0 ${xyz(end)}');
          case _: throw "CNC rapid export needs a line";
        }
      case Move(_, geometry, speed, blend, _):
        if (speed <= 0.0) throw "CNC G-code feed needs positive speed";
        setTolerance(blend);
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
      case ToolLengthOffset(number, length, _):
        if (number > 0 && Math.abs(machine.controller.toolLength(number) - length) > 1e-9)
          throw 'CNC G-code H$number length disagrees with controller setup';
        lines.push(number == 0 ? "G49" : 'G43 H$number');
      case OptionalStop(_): lines.push("M1");
      case ProgramStop(_): lines.push("M0");
      case End(_): lines.push("M2");
      case Spindle(direction, rpm, _):
        switch direction {
          case Off: lines.push("M5");
          case Clockwise, CounterClockwise:
            if (rpm <= 0.0) throw "CNC spindle start needs positive RPM";
            lines.push('S${number(rpm)} ${direction == Clockwise ? "M3" : "M4"}');
        }
      case Coolant(mist, flood, _):
        if (!mist && !flood) lines.push("M9");
        else {
          if (mist) lines.push("M7");
          if (flood) lines.push("M8");
        }
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
    if (!Math.isFinite(value)) throw "CNC G-code cannot export non-finite values";
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
