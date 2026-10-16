package cnckit.printer;

import cnckit.CncController;
import cnckit.CncDiagnostic;
import cnckit.CncDiagnostic.CncSeverity;
import cnckit.PrinterSettings;
import cnckit.interp.CncInterpreter;
import cnckit.parse.CncBlock;
import cnckit.parse.CncComment;
import cnckit.parse.CncWord;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.MoveKind;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.PrintFeature;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;

private enum PendingMarker {
  PendingLayer(index:Int, span:Provenance);
  PendingFeature(kind:PrintFeature, span:Provenance);
}

/**
  Interprets Marlin, Klipper and RepRap G-code into the shared toolpath format.

  `E` is filament, not a coordinate: a move that pushes filament lays a bead whose cross-section is
  the filament pushed times its area over the distance moved, and the program says that (`Deposit`),
  not how far the extruder turned. A new `Deposit` starts when that ratio changes by more than the
  controller's tolerance, so the deposited volume is right to that tolerance. Overrides `M220` and
  `M221` are folded into the moves they scale, as the firmware does. Slicers' comments are read as
  layer, feature and object markers.
**/
class PrinterInterpreter {
  public final controller:CncController;
  public final start:Point3;
  public var ops(default, null):Array<ToolpathOp> = [];
  public var diagnostics(default, null):Array<CncDiagnostic> = [];
  /** Did the program ask to home? */
  public var homes(default, null):Bool = false;

  final settings:PrinterSettings;
  var x = 0.0;
  var y = 0.0;
  var z = 0.0;
  /** Filament position in metres, in the file's own frame (`G92 E` moves its origin). */
  var e = 0.0;
  var metric = true;
  var absolute = true;
  var relativeE = false;
  var motionMode = -1;
  /** The last `F`, in metres per second; NaN before the first. */
  var feed = Math.NaN;
  var speedFactor = 1.0;
  var flowFactor = 1.0;
  var depositArea = 0.0;
  var layerHeight:Float;
  var previousLayerZ:Null<Float> = null;
  var layerCounter = 0;
  var activeObject:Null<String> = null;
  var pending:Array<PendingMarker> = [];
  /**
    Layer markers placed before their height was known: a command between a layer comment and the layer's first move
    (Cura's `;LAYER:n` then `M106`) puts the marker where it belongs, and the first move or `;Z:` gives its height.
  **/
  var unplaced:Array<{op:Int, index:Int, span:Provenance}> = [];
  var warned:Map<String, Bool> = new Map();

  public function new(controller:CncController, start:Point3) {
    if (controller.printer == null) throw "the printer interpreter needs a printer controller";
    this.controller = controller;
    this.start = start;
    settings = controller.printer;
    layerHeight = settings.defaultLayerHeight;
  }

  public function interpret(blocks:Array<CncBlock>):Array<ToolpathOp> {
    ops = [];
    diagnostics = [];
    x = start.x; y = start.y; z = start.z;
    for (block in blocks) {
      var count = ops.length;
      try process(block)
      catch (error:CncDiagnostic) {
        // A refused line leaves nothing of itself behind.
        while (ops.length > count) ops.pop();
        diagnostics.push(error);
      }
    }
    flushMarkers(z);
    return ops.copy();
  }

  function process(block:CncBlock):Void {
    var line = block.span.line;
    // Whole-line comments come first; a trailing comment annotates the command before it.
    if (block.words.length == 0 && block.command == null) {
      for (comment in block.comments) markerFrom(comment);
      return;
    }
    if (block.command != null) extended(block);
    else lettered(block);
    for (comment in block.comments) markerFrom(comment);
  }

  // ---- lettered commands -------------------------------------------------------------

  function lettered(block:CncBlock):Void {
    var words = block.words, line = block.span.line;
    var values:Map<String, CncWord> = new Map();
    var g:Array<CncWord> = [], m:Array<CncWord> = [];
    for (word in words) switch word.letter {
      case "G": g.push(word);
      case "M": m.push(word);
      case _:
        if (values.exists(word.letter)) fail(line, word.column, 'duplicate ${word.letter} word');
        values.set(word.letter, word);
    }
    if (g.length + m.length > 1) fail(line, words[0].column, "printer G-code takes one G or M command per line");
    if (g.length == 0 && m.length == 0) {
      if (motionMode < 0) fail(line, words[0].column, "words need a G0-G3 motion mode");
      motion(motionMode, values, block);
      return;
    }
    if (g.length == 1) gCode(code(g[0], line), g[0], values, block);
    else mCode(code(m[0], line), m[0], values, block);
  }

  function gCode(number:Int, word:CncWord, values:Map<String, CncWord>, block:CncBlock):Void {
    var line = block.span.line;
    switch number {
      case 0, 1, 2, 3:
        motionMode = number;
        motion(number, values, block);
      case 4:
        var p = values.get("P"), s = values.get("S");
        var seconds = p != null ? p.value * 0.001 : s != null ? s.value : 0.0;
        if (!(seconds > 0.0)) fail(line, word.column, "G4 needs a positive P (milliseconds) or S (seconds)");
        placeMarkers();
        ops.push(ToolpathOp.Dwell(seconds, block.span));
      case 10, 11:
        placeMarkers();
        depositOff(block.span);
        if (number == 10) ops.push(ToolpathOp.FilamentMove(-settings.retractLength, settings.retractSpeed, block.span));
        else ops.push(ToolpathOp.FilamentMove(settings.retractLength + settings.unretractExtraLength,
          settings.unretractSpeed, block.span));
      case 17: // XY is the only plane
      case 18, 19: fail(line, word.column, 'G$number arcs outside the XY plane are unsupported');
      case 20: metric = false;
      case 21: metric = true;
      case 28: homes = true;
      case 29, 30: ignored(block, 'G$number', "bed levelling is not modelled");
      case 90: absolute = true;
      case 91: absolute = false;
      case 92:
        var extruder = values.get("E");
        if (extruder == null || values.exists("X") || values.exists("Y") || values.exists("Z"))
          fail(line, word.column, "G92 can only set the extruder position (E): the axes are referenced by homing");
        e = extruder.value * unitScale();
      case _: fail(line, word.column, 'unsupported G$number');
    }
  }

  function mCode(number:Int, word:CncWord, values:Map<String, CncWord>, block:CncBlock):Void {
    var line = block.span.line;
    switch number {
      case 0: placeMarkers(); ops.push(ToolpathOp.ProgramStop(block.span));
      case 1: placeMarkers(); ops.push(ToolpathOp.OptionalStop(block.span));
      case 2, 30: placeMarkers(); depositOff(block.span); ops.push(ToolpathOp.End(block.span));
      case 82: relativeE = false;
      case 83: relativeE = true;
      case 104, 109:
        var target = values.get("S");
        var cooling = values.get("R");
        var celsius = target != null ? target.value : cooling != null ? cooling.value : Math.NaN;
        if (!Math.isFinite(celsius)) fail(line, word.column, 'M$number needs S (or R) temperature');
        heat(toolHeater(values.get("T")), celsius, number == 109, block);
      case 140, 190:
        var target = values.get("S"), cooling = values.get("R");
        var celsius = target != null ? target.value : cooling != null ? cooling.value : Math.NaN;
        if (!Math.isFinite(celsius)) fail(line, word.column, 'M$number needs S (or R) temperature');
        heat("bed", celsius, number == 190, block);
      case 141, 191:
        var target = values.get("S"), cooling = values.get("R");
        var celsius = target != null ? target.value : cooling != null ? cooling.value : Math.NaN;
        if (!Math.isFinite(celsius)) fail(line, word.column, 'M$number needs S (or R) temperature');
        heat("chamber", celsius, number == 191, block);
      case 106, 107:
        var p = values.get("P"), s = values.get("S");
        var duty = number == 107 ? 0.0 : s == null ? 1.0 : s.value / 255.0;
        if (!(duty >= 0.0 && duty <= 1.0)) fail(line, columnOf(s, word.column), "M106 S is 0 to 255");
        placeMarkers();
        ops.push(ToolpathOp.Fan(fanName(p == null ? 0 : Std.int(p.value)), duty, block.span));
      case 220, 221:
        var s = values.get("S");
        if (s == null || !(s.value > 0.0)) fail(line, word.column, 'M$number needs S, a percentage above 0');
        if (number == 220) speedFactor = s.value / 100.0; else flowFactor = s.value / 100.0;
      case 486: object(values, word, block);
      case 17, 18, 73, 75, 76, 77, 84, 108, 117, 118, 155, 300, 413:
        // Display, beeper and stepper-power commands change nothing the toolpath describes.
      case 92, 201, 202, 203, 204, 205, 206, 207, 208, 209, 211, 290, 420, 500, 501, 502, 851, 900:
        ignored(block, 'M$number', "motion limits, calibration and stored settings belong to the machine");
      case _: fail(line, word.column, 'unsupported M$number');
    }
  }

  function heat(heater:String, celsius:Float, wait:Bool, block:CncBlock):Void {
    if (!(celsius >= 0.0)) fail(block.span.line, 1, "temperatures cannot be negative");
    placeMarkers();
    ops.push(ToolpathOp.Heat(heater, celsius, wait, block.span));
  }

  static function toolHeater(tool:Null<CncWord>):String {
    if (tool == null || tool.value == 0.0) return "hotend";
    return "hotend" + Std.int(tool.value);
  }

  static function fanName(index:Int):String return index == 0 ? "part" : "fan" + index;

  // ---- motion --------------------------------------------------------------------------

  function motion(mode:Int, values:Map<String, CncWord>, block:CncBlock):Void {
    var line = block.span.line;
    for (letter in values.keys())
      if ("XYZEFIJRKPS".indexOf(letter) < 0) fail(line, columnOf(values.get(letter), 1), 'unsupported $letter word');
    var f = values.get("F");
    if (f != null) {
      if (!(f.value > 0.0)) fail(line, columnOf(f, 1), "F feed must be positive");
    }
    var scale = unitScale();
    var target = [x, y, z];
    var axes = ["X", "Y", "Z"], hasAxis = false;
    for (axis in 0...3) {
      var word = values.get(axes[axis]);
      if (word == null) continue;
      hasAxis = true;
      target[axis] = absolute ? word.value * scale : target[axis] + word.value * scale;
    }
    var extruder = values.get("E");
    var delta = 0.0, nextE = e;
    if (extruder != null) {
      var amount = extruder.value * scale;
      delta = relativeE ? amount : amount - e;
      nextE = relativeE ? e + amount : amount;
    }
    var from = new Point3(x, y, z), to = new Point3(target[0], target[1], target[2]);
    var speed = f != null ? f.value * scale / 60.0 : feed;
    var effective = speed * speedFactor;
    var filament = delta * flowFactor;
    var geometry:Null<PathGeometry> = null;
    if (mode >= 2) geometry = arc(mode, from, to, values, line, block);
    else if (hasAxis && from.distanceTo(to) > 1e-12) geometry = PathGeometry.Line(from, to);
    var length = geometry == null ? 0.0 : GeometryTools.length(geometry);
    if (geometry == null && mode >= 2) fail(line, block.words[0].column, "an arc needs an endpoint or a full circle's centre");
    if (geometry != null && !Math.isFinite(speed)) fail(line, block.words[0].column, "moves need an F feed");
    if (geometry == null && delta != 0.0 && !Math.isFinite(speed)) fail(line, columnOf(extruder, 1), "an extruder move needs an F feed");
    // Everything is checked: commit the block.
    if (f != null) feed = speed;
    e = nextE;
    flushMarkers(target[2]);
    var span = block.span;
    if (geometry == null) {
      if (filament != 0.0) {
        depositOff(span);
        ops.push(ToolpathOp.FilamentMove(filament, Math.abs(effective), span));
      }
    } else {
      if (filament > 1e-12) {
        var area = filament * settings.filamentArea() / length;
        deposit(area, span);
        ops.push(ToolpathOp.Move(MoveKind.Cut, geometry, effective, settings.pathTolerance, span));
      } else {
        depositOff(span);
        if (filament < -1e-12) {
          warn(block, "CNC_PRINTER_WIPE", "retract while moving became a retract followed by a travel move");
          ops.push(ToolpathOp.FilamentMove(filament, settings.retractSpeed, span));
        }
        ops.push(ToolpathOp.Move(MoveKind.Rapid, geometry, effective, settings.pathTolerance, span));
      }
      x = to.x; y = to.y; z = to.z;
    }
  }

  /** The XY arc of G2/G3 from the current point, with its centre from I/J or its radius from R. */
  function arc(mode:Int, from:Point3, to:Point3, values:Map<String, CncWord>, line:Int, block:CncBlock):PathGeometry {
    var i = values.get("I"), j = values.get("J"), r = values.get("R");
    if (values.exists("K")) fail(line, columnOf(values.get("K"), 1), "G2/G3 arcs are in the XY plane: use I and J");
    if (r != null && (i != null || j != null)) fail(line, columnOf(r, 1), "R and arc-centre offsets conflict");
    if (r == null && i == null && j == null) fail(line, block.words[0].column, "G2/G3 need I and J, or R");
    var scale = unitScale();
    var center = r == null ? CncInterpreter.centerFromOffsets(from, 17,
      i == null ? 0.0 : i.value * scale, j == null ? 0.0 : j.value * scale) :
      CncInterpreter.radiusCenter(from, to, r.value * scale, mode, 17, line, r.column);
    var origin = CncInterpreter.coordinates(from, 17), finish = CncInterpreter.coordinates(to, 17);
    var c = CncInterpreter.coordinates(center, 17);
    var radius = Math.sqrt(Math.pow(origin[0] - c[0], 2) + Math.pow(origin[1] - c[1], 2));
    var endRadius = Math.sqrt(Math.pow(finish[0] - c[0], 2) + Math.pow(finish[1] - c[1], 2));
    if (radius <= 1e-12 || Math.abs(endRadius - radius) > Math.max(1e-8, radius * 1e-5))
      fail(line, block.words[0].column, "arc endpoint is not on its centre circle");
    var sweep = CncInterpreter.arcSweep(from, to, center, mode, 17);
    var begin = Math.atan2(origin[1] - c[1], origin[0] - c[0]);
    var rise = finish[2] - origin[2];
    return Math.abs(rise) <= 1e-12 ? PathGeometry.Arc(center, radius, begin, sweep) :
      PathGeometry.Circular(center, radius, begin, sweep, XY, rise);
  }

  // ---- deposition -----------------------------------------------------------------------

  function deposit(area:Float, span:Provenance):Void {
    if (depositArea > 0.0 && Math.abs(area / depositArea - 1.0) <= settings.ratioTolerance) return;
    // A rounded bead as tall as the layer: a stadium of area h(w - h) + pi h^2 / 4.
    var height = toolpathkit.path.BeadSection.heightForArea(area,layerHeight);
    var width = toolpathkit.path.BeadSection.width(area,height);
    ops.push(ToolpathOp.Deposit(area, width, height, span));
    depositArea = area;
  }

  function depositOff(span:Provenance):Void {
    if (depositArea <= 0.0) return;
    ops.push(ToolpathOp.DepositOff(span));
    depositArea = 0.0;
  }

  // ---- markers ---------------------------------------------------------------------------

  function markerFrom(comment:CncComment):Void {
    var text = comment.text;
    var upper = text.toUpperCase();
    if (upper == "LAYER_CHANGE") pending.push(PendingLayer(layerCounter++, comment.span));
    else if (StringTools.startsWith(upper, "Z:")) {
      var height = Std.parseFloat(StringTools.trim(text.substr(2)));
      if (Math.isFinite(height)) flushMarkers(height * 0.001);
    } else if (StringTools.startsWith(upper, "HEIGHT:")) {
      var height = Std.parseFloat(StringTools.trim(text.substr(7)));
      if (Math.isFinite(height) && height > 0.0) layerHeight = height * 0.001;
    } else if (StringTools.startsWith(upper, "LAYER:")) {
      var index = Std.parseInt(StringTools.trim(text.substr(6)));
      if (index != null && index >= 0) {
        pending.push(PendingLayer(index, comment.span));
        layerCounter = index + 1;
      }
    } else if (StringTools.startsWith(upper, "TYPE:")) pending.push(PendingFeature(feature(text.substr(5)), comment.span));
    else if (StringTools.startsWith(upper, "FEATURE:")) pending.push(PendingFeature(feature(text.substr(8)), comment.span));
    else if (StringTools.startsWith(upper, "MESH:")) {
      var name = StringTools.trim(text.substr(5));
      if (name.length > 0 && name != "NONMESH") startObject(name, comment.span);
    } else if (StringTools.startsWith(text, "printing object ")) {
      var name = text.substr("printing object ".length);
      var marker = name.indexOf(" id:");
      startObject(marker < 0 ? name : name.substr(0, marker), comment.span);
    } else if (StringTools.startsWith(text, "stop printing object ")) {
      var name = text.substr("stop printing object ".length);
      var marker = name.indexOf(" id:");
      endObject(marker < 0 ? name : name.substr(0, marker), comment.span);
    }
  }

  /** The label slicers give a feature, as the shared vocabulary. */
  public static function feature(label:String):PrintFeature {
    var text = StringTools.trim(label), lower = text.toLowerCase();
    function has(part:String):Bool return lower.indexOf(part) >= 0;
    if (has("external perimeter") || has("outer wall") || has("wall-outer") || has("outer perimeter")) return ExternalPerimeter;
    if (has("overhang perimeter") || has("perimeter") || has("inner wall") || has("wall-inner")) return Perimeter;
    if (has("top solid") || has("top surface") || has("skin-top")) return TopSolidInfill;
    if (has("solid infill") || has("bottom surface") || lower == "skin") return SolidInfill;
    if (has("infill") || lower == "fill") return Infill;
    if (has("skirt") || has("brim")) return Skirt;
    if (lower == "travel") return Travel;
    return Other(text);
  }

  /** Places the pending markers at the height the layer has now reached (a move's target, or `;Z:`). */
  function flushMarkers(height:Float):Void {
    for (layer in unplaced) {
      // A refused line takes back what it pushed; a marker it took back is pending again in no form.
      if (layer.op < ops.length) ops[layer.op] = ToolpathOp.Layer(layer.index, layerAt(height), layer.span);
    }
    unplaced = [];
    for (marker in pending) switch marker {
      case PendingLayer(index, span): ops.push(ToolpathOp.Layer(index, layerAt(height), span));
      case PendingFeature(kind, span): ops.push(ToolpathOp.Feature(kind, span));
    }
    pending = [];
  }

  /** Places the pending markers before a command that is not a move, their layer's height still to come. */
  function placeMarkers():Void {
    for (marker in pending) switch marker {
      case PendingLayer(index, span):
        unplaced.push({op: ops.length, index: index, span: span});
        ops.push(ToolpathOp.Layer(index, z, span));
      case PendingFeature(kind, span): ops.push(ToolpathOp.Feature(kind, span));
    }
    pending = [];
  }

  function layerAt(height:Float):Float {
    if (previousLayerZ == null) layerHeight = Math.max(height, 1e-6);
    else if (height - previousLayerZ > 1e-9) layerHeight = height - previousLayerZ;
    previousLayerZ = height;
    return height;
  }

  function startObject(name:String, span:Provenance):Void {
    if (activeObject == name) return;
    placeMarkers();
    if (activeObject != null) ops.push(ToolpathOp.PrintObject(activeObject, false, span));
    ops.push(ToolpathOp.PrintObject(name, true, span));
    activeObject = name;
  }

  function endObject(name:String, span:Provenance):Void {
    if (activeObject != name) return;
    placeMarkers();
    ops.push(ToolpathOp.PrintObject(name, false, span));
    activeObject = null;
  }

  /** Marlin's M486: S selects (and optionally names, with A"...") the object being printed; S-1 ends it. */
  function object(values:Map<String, CncWord>, word:CncWord, block:CncBlock):Void {
    var s = values.get("S");
    if (s == null) {
      ignored(block, "M486", "only object selection (S) is modelled");
      return;
    }
    var index = Std.int(s.value);
    if (index < 0) {
      if (activeObject != null) endObject(activeObject, block.span);
      return;
    }
    var name = values.get("A");
    startObject(name != null && name.text != null ? name.text : "object" + index, block.span);
  }

  // ---- extended commands --------------------------------------------------------------------

  function extended(block:CncBlock):Void {
    var name = block.command, args = block.arguments, line = block.span.line;
    function required(key:String):String {
      var value = args.get(key);
      if (value == null) fail(line, 1, '$name needs $key=');
      return value;
    }
    switch name {
      case "EXCLUDE_OBJECT_DEFINE":
      case "EXCLUDE_OBJECT_START": startObject(required("NAME"), block.span);
      case "EXCLUDE_OBJECT_END": endObject(required("NAME"), block.span);
      case "EXCLUDE_OBJECT": ignored(block, name, "excluding an object happens while printing");
      case "SET_HEATER_TEMPERATURE":
        var target = Std.parseFloat(required("TARGET"));
        if (!Math.isFinite(target)) fail(line, 1, "TARGET is not a number");
        heat(heaterName(required("HEATER")), target, false, block);
      case "TEMPERATURE_WAIT":
        var heater = heaterName(required("SENSOR"));
        var last = ops.length - 1;
        var waited = false;
        if (last >= 0) switch ops[last] {
          case Heat(previous, celsius, false, span) if (previous == heater):
            ops[last] = ToolpathOp.Heat(previous, celsius, true, span);
            waited = true;
          case _:
        }
        if (!waited) ignored(block, name, "it waits for a heater the program did not just set");
      case "SET_FAN_SPEED":
        var duty = Std.parseFloat(required("SPEED"));
        if (!(duty >= 0.0 && duty <= 1.0)) fail(line, 1, "SPEED is 0 to 1");
        placeMarkers();
        ops.push(ToolpathOp.Fan(required("FAN"), duty, block.span));
      case "TURN_OFF_HEATERS":
        heat("hotend", 0.0, false, block);
        heat("bed", 0.0, false, block);
      case _: ignored(block, name, "a macro or setting the toolpath does not model");
    }
  }

  static function heaterName(klipper:String):String {
    if (klipper == "extruder") return "hotend";
    if (StringTools.startsWith(klipper, "extruder") && Std.parseInt(klipper.substr(8)) != null) return "hotend" + klipper.substr(8);
    if (klipper == "heater_bed") return "bed";
    return klipper;
  }

  // ---- helpers -----------------------------------------------------------------------------------

  function unitScale():Float return metric ? 0.001 : 0.0254;

  /** Warns once per command that a command was understood but is not modelled. */
  function ignored(block:CncBlock, name:String, reason:String):Void {
    if (warned.exists(name)) return;
    warned.set(name, true);
    warn(block, "CNC_PRINTER_IGNORED", '$name ignored: $reason');
  }

  function warn(block:CncBlock, code:String, message:String):Void
    diagnostics.push(new CncDiagnostic(Warning, code, block.span, message));

  static function columnOf(word:Null<CncWord>, fallback:Int):Int return word == null ? fallback : word.column;

  static function code(word:CncWord, line:Int):Int {
    if (word.value < 0.0 || word.value != Math.floor(word.value))
      fail(line, word.column, "code must be a non-negative integer");
    return Std.int(word.value);
  }

  static function fail(line:Int, column:Int, message:String):Void
    throw new CncDiagnostic(Error, "CNC_INTERPRET", new Provenance(line, column, 1), message);
}
