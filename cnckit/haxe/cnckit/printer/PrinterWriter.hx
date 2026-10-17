package cnckit.printer;

import cnckit.CncController;
import cnckit.CncWriter;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.MoveKind;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.PrintFeature;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;

/**
  The printer post: writes a program's printing operations as Marlin or Klipper G-code, in
  millimetres with relative extrusion (`M83`). Each bead's `E` is its cross-section times the
  distance over the filament's area, so reading the file back (`CncCompiler` with a printer
  controller) recovers the deposited volume. Layers and features are written as the slicers'
  comments, so other tools and this reader see the same markers.
**/
class PrinterWriter {
  /** `home` writes a leading `G28`: the printer homes before the program. */
  public static function write(program:ToolpathProgram, controller:CncController,
      ?flavour:PrinterFlavour = Klipper, ?home:Bool = true):String {
    if (program == null || controller == null) throw "printer G-code export needs a program and controller";
    if (controller.dialect != Printer || controller.printer == null)
      throw "printer G-code export needs a printer controller";
    var settings = controller.printer;
    var filamentArea = settings.filamentArea();
    var lines = ["G21 ; millimetres", "G90 ; absolute axes", "M83 ; relative extruder"];
    if (home) lines.push("G28 ; home");
    if (flavour == Klipper) {
      var seen = new Map<String, Bool>();
      for (op in program.ops) switch op {
        case PrintObject(name, true, _):
          if (!seen.exists(name)) { seen.set(name, true); lines.push('EXCLUDE_OBJECT_DEFINE NAME=${objectName(name)}'); }
        case _:
      }
    }
    var objectIndex = new Map<String, Int>();
    var area = 0.0;
    var printed:Array<String> = ["", "", ""];
    var printedFeed = "";
    var line = 0;
    function axis(index:Int, value:Float, letter:String):String {
      var text = CncWriter.millimetres(value);
      if (printed[index] == text) return "";
      printed[index] = text;
      return ' $letter$text';
    }
    function feedWord(speed:Float):String {
      if (!(speed > 0.0)) return "";
      var text = CncWriter.number(speed * 60000.0);
      if (text == printedFeed) return "";
      printedFeed = text;
      return ' F$text';
    }
    function extrusion(length:Float):String
      return ' E${CncWriter.number(area * length / filamentArea * 1000.0)}';
    function line3(kind:String, end:Point3, speed:Float, length:Float, extrude:Bool):String {
      var text = kind + axis(0, end.x, "X") + axis(1, end.y, "Y") + axis(2, end.z, "Z");
      if (extrude) text += extrusion(length);
      return text + feedWord(speed);
    }
    for (index in 0...program.ops.length) {
      var op = program.ops[index];
      switch op {
        case SetSetup(id, _):
          if (id != "1") throw 'printer G-code has no work offsets: setup $id';
        case Move(kind, geometry, speed, _, _):
          var extrude = kind == Cut && area > 0.0;
          if (kind == Cut && !(speed > 0.0)) throw "printer G-code feed needs positive speed";
          var command = kind == Cut || kind == Plunge || kind == Ramp ? "G1" : "G0";
          switch geometry {
            case Line(_, end):
              lines.push(line3(command, end, speed, GeometryTools.length(geometry), extrude));
            case Arc(_, _, _, _), Circular(_, _, _, _, _, _):
              var begin = GeometryTools.pointAt(geometry, 0.0);
              var finish = GeometryTools.pointAt(geometry, GeometryTools.length(geometry));
              var plane = switch geometry {
                case Circular(_, _, _, _, p, _): p;
                case _: XY;
              };
              if (plane != XY) throw "printer G-code arcs are in the XY plane";
              if (settings.arcs) {
                var center = switch geometry {
                  case Arc(c, _, _, _), Circular(c, _, _, _, _, _): c;
                  case _: throw "unreachable";
                };
                var sweep = switch geometry {
                  case Arc(_, _, _, s), Circular(_, _, _, s, _, _): s;
                  case _: 0.0;
                };
                var text = (sweep < 0.0 ? "G2" : "G3") + axis(0, finish.x, "X") + axis(1, finish.y, "Y") +
                  axis(2, finish.z, "Z") + ' I${CncWriter.millimetres(center.x - begin.x)} J${CncWriter.millimetres(center.y - begin.y)}';
                if (extrude) text += extrusion(GeometryTools.length(geometry));
                lines.push(text + feedWord(speed));
              } else {
                // Controllers without arcs get chords no more than 2 um off the arc.
                var total = GeometryTools.length(geometry);
                var radius = switch geometry {
                  case Arc(_, r, _, _), Circular(_, r, _, _, _, _): r;
                  case _: 1.0;
                };
                var sweepAngle = switch geometry {
                  case Arc(_, _, _, s), Circular(_, _, _, s, _, _): Math.abs(s);
                  case _: 0.0;
                };
                var step = radius > 4e-6 ? 2.0 * Math.acos(1.0 - 2e-6 / radius) : Math.PI / 2.0;
                var count = Std.int(Math.max(1.0, Math.ceil(sweepAngle / step)));
                var previous = begin;
                for (piece in 1...count + 1) {
                  var point = GeometryTools.pointAt(geometry, total * piece / count);
                  lines.push(line3(command, point, speed, previous.distanceTo(point), extrude));
                  previous = point;
                }
              }
          }
        case MachineMove(_, _, _, _, _): throw "printer G-code has no machine coordinates";
        case Dwell(seconds, _): lines.push('G4 P${CncWriter.number(seconds * 1000.0)}');
        case PositionJoint(_, _, _), WaitSignal(_, _, _, _), FeedPerRevolution(_, _, _, _), ElectronicGear(_, _, _), ReleaseGear(_, _): throw "Printer has no electronic gearbox export binding";
        case Spindle(_, _, _): throw "printer G-code has no spindle";
        case Coolant(_, _, _): throw "printer G-code has no coolant";
        case ToolChange(_, _), ToolLengthOffset(_, _, _): throw "printer G-code has no tool changes or length offsets";
        case OptionalStop(_): lines.push("M1");
        case ProgramStop(_): lines.push("M0");
        case End(_): lines.push("; end of program");
        case Deposit(bead, _, _, _): area = bead;
        case DepositOff(_): area = 0.0;
        case FilamentMove(length, speed, _):
          lines.push('G1 E${CncWriter.number(length * 1000.0)}${feedWord(speed)}');
        case Heat(heater, celsius, wait, _): lines.push(heatLine(heater, celsius, wait, flavour));
        case Fan(fan, duty, _): lines.push(fanLine(fan, duty, flavour));
        case Layer(_, z, _):
          lines.push(";LAYER_CHANGE");
          lines.push(';Z:${CncWriter.millimetres(z)}');
        case Feature(kind, _): lines.push(';TYPE:${featureLabel(kind)}');
        case PrintObject(name, start, _):
          switch flavour {
            case Klipper:
              lines.push('EXCLUDE_OBJECT_${start ? "START" : "END"} NAME=${objectName(name)}');
            case Marlin:
              if (start) {
                if (!objectIndex.exists(name)) {
                  var count = 0;
                  for (key in objectIndex.keys()) count++;
                  objectIndex.set(name, count);
                }
                lines.push('M486 S${objectIndex.get(name)} A"${name}"');
              } else lines.push("M486 S-1");
          }
      }
    }
    return lines.join("\n") + "\n";
  }

  static function heatLine(heater:String, celsius:Float, wait:Bool, flavour:PrinterFlavour):String {
    var target = CncWriter.number(celsius);
    function hotend(tool:String):String return (wait ? "M109" : "M104") + (tool.length == 0 ? "" : ' T$tool') + ' S$target';
    if (heater == "hotend") return hotend("");
    if (StringTools.startsWith(heater, "hotend") && Std.parseInt(heater.substr(6)) != null) return hotend(heater.substr(6));
    if (heater == "bed") return (wait ? "M190" : "M140") + ' S$target';
    if (heater == "chamber") return (wait ? "M191" : "M141") + ' S$target';
    if (flavour == Marlin) throw 'Marlin has no heater "$heater"';
    var set = 'SET_HEATER_TEMPERATURE HEATER=$heater TARGET=$target';
    return wait ? set + '\nTEMPERATURE_WAIT SENSOR=$heater MINIMUM=${CncWriter.number(celsius - 1.0)}' : set;
  }

  static function fanLine(fan:String, duty:Float, flavour:PrinterFlavour):String {
    var value = Std.int(Math.round(duty * 255.0));
    if (fan == "part") return duty <= 0.0 ? "M107" : 'M106 S$value';
    if (StringTools.startsWith(fan, "fan") && Std.parseInt(fan.substr(3)) != null && flavour == Marlin)
      return duty <= 0.0 ? 'M107 P${fan.substr(3)}' : 'M106 P${fan.substr(3)} S$value';
    if (flavour == Marlin) throw 'Marlin has no fan "$fan"';
    return 'SET_FAN_SPEED FAN=$fan SPEED=${CncWriter.number(duty)}';
  }

  /** The label slicers use, which `PrinterInterpreter.feature` reads back. */
  public static function featureLabel(kind:PrintFeature):String
    return switch kind {
      case ExternalPerimeter: "External perimeter";
      case Perimeter: "Perimeter";
      case Infill: "Internal infill";
      case SolidInfill: "Solid infill";
      case TopSolidInfill: "Top solid infill";
      case Skirt: "Skirt/Brim";
      case Travel: "Travel";
      case Other(name): name;
    };

  static function objectName(name:String):String {
    for (index in 0...name.length) {
      var code = name.charCodeAt(index);
      var ok = (code >= 48 && code <= 57) || (code >= 65 && code <= 90) || (code >= 97 && code <= 122) ||
        code == 95 || code == 45 || code == 46;
      if (!ok) throw 'object name "$name" must use letters, digits, "_", "-" and "."';
    }
    if (name.length == 0) throw "object name must not be empty";
    return name;
  }
}
